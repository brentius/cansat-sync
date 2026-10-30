"""GPL-3.0-only. Bounded local landmark tracking from a predicted camera pose.

Prepare a raw brightness map with FineMatcher on the pad, then call match_filter
for each undistorted grayscale frame. Feed its points, original image pixels,
pixel_var (px squared) and map_var (metres squared) to Eskf.update_landmarks.
The matcher does not change the filter or use GNSS/truth. Camera intrinsics must
describe the supplied image; use coarse.bin_camera if binning the sensor image.

World is ENU, raster row increases north, column east, and ground is z=0.
Map templates are located in predicted-pose rectified image search patches.
The winning ground offset is projected back through that SAME predicted pose:
it supplies an observed image pixel, not a corrected world landmark location.
Camera.project applies camera/body rotation and the IMU-to-camera lever arm.

This is a tracking stage, not acquisition or relocalisation: a wrong prior or
repeated/changed terrain may still produce plausible false matches. Map/site
preparation and frame work have fixed configuration caps. Results have only
synthetic verification; no real imagery or Raspberry Pi timing is claimed.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from .coarse import _bilinear, ncc_valid
from .eskf import exp_so3


@dataclass(frozen=True)
class FineConfig:
    template_px: int = 32
    resolution_m: float = .5
    search_radius_px: int = 32
    max_landmarks: int = 24
    cells: tuple = (6, 4)
    grid_spacing_m: float = 24.
    min_score: float = .5
    min_gap: float = .04
    exclusion_px: int = 3
    min_std: float = 2.
    pixel_sigma: float = .5
    map_sigma_m: float = .15

    def validate(self):
        integers = ((self.template_px, 8, 64), (self.search_radius_px, 4, 64),
                    (self.max_landmarks, 1, 60), (self.exclusion_px, 1, 16))
        if any(not isinstance(v, (int, np.integer)) or isinstance(v, bool) or not lo <= v <= hi
               for v, lo, hi in integers):
            raise ValueError('patch, search, landmark or exclusion size exceeds its integer bounds')
        if self.exclusion_px >= self.search_radius_px:
            raise ValueError('peak exclusion must be smaller than the search radius')
        if (not isinstance(self.cells, tuple) or len(self.cells) != 2
                or any(not isinstance(v, (int, np.integer)) or isinstance(v, bool) or not 1 <= v <= 12
                       for v in self.cells)):
            raise ValueError('cells must be two positive integer counts, each at most 12')
        for name in ('resolution_m', 'grid_spacing_m', 'min_std', 'pixel_sigma', 'map_sigma_m'):
            value = getattr(self, name)
            if not np.isscalar(value) or not np.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        if self.resolution_m < .1 or self.grid_spacing_m < 4:
            raise ValueError('resolution must be at least 0.1 m; site spacing at least 4 m')
        if not (np.isfinite(self.min_score) and 0 < self.min_score < 1
                and np.isfinite(self.min_gap) and 0 < self.min_gap < 1):
            raise ValueError('score and gap gates must be between zero and one')


@dataclass
class FineMatches:
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    pixels: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    pixel_var: np.ndarray = field(default_factory=lambda: np.empty(0))
    map_var: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    scores: np.ndarray = field(default_factory=lambda: np.empty(0))
    gaps: np.ndarray = field(default_factory=lambda: np.empty(0))
    indices: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=int))
    attempted: int = 0
    rejections: dict = field(default_factory=dict)
    reason: str = ''


_dy, _dx = np.mgrid[-1:2, -1:2]
_PEAK_FIT = np.linalg.pinv(np.column_stack((np.ones(9), _dx.ravel(), _dy.ravel(),
                                          _dx.ravel()**2/2, (_dx*_dy).ravel(), _dy.ravel()**2/2)))


def _peak(scores, row, col):
    """Joint quadratic peak (column, row), refusing borders, ridges and flat tops."""
    if not (0 < row < scores.shape[0] - 1 and 0 < col < scores.shape[1] - 1):
        return None
    c = _PEAK_FIT @ scores[row - 1:row + 2, col - 1:col + 2].ravel()
    h = np.array([[c[3], c[4]], [c[4], c[5]]])
    ev = np.linalg.eigvalsh(h)
    if ev[-1] >= -1e-6 or abs(ev[-1]/ev[0]) < .01:
        return None
    delta = -np.linalg.solve(h, c[1:3])
    if not np.isfinite(delta).all() or np.max(np.abs(delta)) > 1:
        return None
    return np.array([col, row]) + delta


def _rotation(value):
    return (value.shape == (3, 3) and np.isfinite(value).all()
            and np.allclose(value.T @ value, np.eye(3), atol=1e-5, rtol=0)
            and abs(np.linalg.det(value) - 1.) < 1e-5)


class FineMatcher:
    """Prepared local map templates; all per-frame work uses only a predicted pose.

    raster is finite raw grayscale brightness at res metres/pixel, origin is
    the ENU coordinate of raster[0,0]. Explicit landmarks are (n,3), z=0.
    Preparation raises ValueError for invalid data or more than MAX_SITES
    candidates; flat/one-dimensional templates are skipped. The input map and
    landmark arrays can be changed/freed afterwards without changing this bank.
    """
    MAX_SITES = 10000

    def __init__(self, raster, res, origin, config=None, landmarks=None):
        self.cfg = cfg = config or FineConfig()
        cfg.validate()
        raster = np.asarray(raster, dtype=float)
        origin = np.asarray(origin, dtype=float)
        if (raster.ndim != 2 or min(raster.shape) < 2 or not np.isfinite(raster).all()
                or not np.isscalar(res) or not np.isfinite(res) or res <= 0
                or origin.shape != (2,) or not np.isfinite(origin).all()):
            raise ValueError('map must be a finite 2D raster with positive resolution and ENU origin')
        self.map_resolution = float(res)
        n = cfg.template_px
        axis = (np.arange(n) - (n - 1)/2)*cfg.resolution_m
        tx, ty = np.meshgrid(axis, axis)
        if landmarks is None:
            half = max(abs(axis[0]), abs(axis[-1])) + res
            axes = [np.arange(origin[i] + half, origin[i] + (raster.shape[1 - i] - 1)*res - half,
                              cfg.grid_spacing_m) for i in range(2)]
            if len(axes[0])*len(axes[1]) > self.MAX_SITES:
                raise ValueError('map grid exceeds MAX_SITES; crop the map or increase site spacing')
            x, y = np.meshgrid(*axes)
            landmarks = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
        landmarks = np.asarray(landmarks, dtype=float)
        if (landmarks.ndim != 2 or landmarks.shape[1] != 3 or len(landmarks) > self.MAX_SITES
                or not np.isfinite(landmarks).all() or np.any(landmarks[:, 2] != 0)):
            raise ValueError('landmarks must be finite (n,3) on z=0, at most MAX_SITES')
        points, templates, strengths, indices = [], [], [], []
        self.preparation_rejections = {'outside_map': 0, 'texture': 0}
        for index, point in enumerate(landmarks):
            t = _bilinear(raster, (point[0] + tx - origin[0])/res, (point[1] + ty - origin[1])/res)
            if not np.isfinite(t).all():
                self.preparation_rejections['outside_map'] += 1
                continue
            # A straight boundary has contrast but cannot locate a point ALONG
            # itself. The smaller structure-tensor eigenvalue measures that.
            gy, gx = np.gradient(t)
            g = np.column_stack((gx.ravel(), gy.ravel()))
            ev = np.linalg.eigvalsh(g.T @ g/g.shape[0])
            if np.std(t) < cfg.min_std or ev[0] <= 1e-6 or ev[0]/max(ev[1], 1e-30) < .02:
                self.preparation_rejections['texture'] += 1
                continue
            points.append(point.copy())
            templates.append(t.astype(np.float32))
            strengths.append(ev[0])
            indices.append(index)
        self.points = np.asarray(points, dtype=float).reshape(-1, 3)
        self.templates = np.asarray(templates, dtype=np.float32).reshape(-1, n, n)
        self.strengths = np.asarray(strengths, dtype=float)
        self.indices = np.asarray(indices, dtype=int)
        for a in (self.points, self.templates, self.strengths, self.indices):
            a.flags.writeable = False
        size = n + 2*cfg.search_radius_px
        axis = (np.arange(size) - (size - 1)/2)*cfg.resolution_m
        sx, sy = np.meshgrid(axis, axis)
        self._offsets = np.column_stack((sx.ravel(), sy.ravel(), np.zeros(size*size)))
        half = axis[-1]
        self._corners = np.array([[-half, -half, 0], [half, -half, 0],
                                  [half, half, 0], [-half, half, 0]])

    @property
    def prepared_bytes(self):
        """Owned array storage; excludes short-lived preparation/frame workspace."""
        return sum(a.nbytes for a in (self.points, self.templates, self.strengths, self.indices,
                                      self._offsets, self._corners))

    def _select(self, camera, p, r, shape):
        """At most one texture-ranked, fully visible search patch per image cell."""
        n = len(self.points)
        corners = (self.points[:, None, :] + self._corners).reshape(-1, 3)
        with np.errstate(divide='ignore', invalid='ignore'):
            uv, depth = camera.project(corners, p, r)
            centres, _ = camera.project(self.points, p, r)
        uv, depth = uv.reshape(n, 4, 2), depth.reshape(n, 4)
        visible = (np.isfinite(uv).all(axis=(1, 2)) & (depth > 1e-6).all(axis=1)
                   & (uv[:, :, 0] >= 0).all(axis=1) & (uv[:, :, 0] <= shape[1] - 1).all(axis=1)
                   & (uv[:, :, 1] >= 0).all(axis=1) & (uv[:, :, 1] <= shape[0] - 1).all(axis=1))
        chosen, cells = [], set()
        for i in np.argsort(-self.strengths, kind='stable'):
            if not visible[i]:
                continue
            cell = tuple(np.floor(centres[i]/[shape[1]/self.cfg.cells[0], shape[0]/self.cfg.cells[1]]).astype(int))
            if cell in cells:
                continue
            cells.add(cell)
            chosen.append(i)
            if len(chosen) >= self.cfg.max_landmarks:
                break
        return chosen, int(n - visible.sum())

    def match(self, image, camera, p, r):
        """Return FineMatches; malformed frame/pose inputs return an empty refusal.

        No image, pose or camera is modified. A refusal does not imply the prior
        is correct. Call acquisition again when tracking is persistently lost.
        """
        result = FineMatches()
        try:
            image = np.asarray(image)
            if image.dtype.kind not in 'iu':        # integer samples convert exactly inside _bilinear
                image = np.asarray(image, dtype=float)
            p, r = np.asarray(p, dtype=float), np.asarray(r, dtype=float)
            if image.ndim != 2 or min(image.shape) < 2 or (image.dtype.kind == 'f' and not np.isfinite(image).all()):
                result.reason = 'invalid_image'
                return result
            if p.shape != (3,) or not np.isfinite(p).all() or not _rotation(r):
                result.reason = 'invalid_pose'
                return result
            extrinsics, lever = np.asarray(camera.r_cb), np.asarray(camera.lever)
            intrinsics = np.asarray([camera.fx, camera.fy, camera.cx, camera.cy], dtype=float)
            if (not np.isfinite(intrinsics).all() or min(camera.fx, camera.fy) <= 0
                    or not _rotation(extrinsics) or lever.shape != (3,) or not np.isfinite(lever).all()
                    or (camera.width and camera.width != image.shape[1])
                    or (camera.height and camera.height != image.shape[0])):
                result.reason = 'invalid_camera'
                return result
        except (ValueError, TypeError, AttributeError):
            result.reason = 'invalid_input'
            return result
        if not len(self.points):
            result.reason = 'no_map_texture'
            return result
        chosen, outside = self._select(camera, p, r, image.shape)
        result.rejections['out_of_view'] = outside
        cfg = self.cfg
        kept, pixels, scores_out, gaps = [], [], [], []

        def reject(reason):
            result.rejections[reason] = result.rejections.get(reason, 0) + 1

        size = cfg.template_px + 2*cfg.search_radius_px
        for i in chosen:
            result.attempted += 1
            uv, depth = camera.project(self.points[i] + self._offsets, p, r)
            if not np.isfinite(uv).all() or np.any(depth <= 1e-6):
                reject('invalid_projection')
                continue
            search = _bilinear(image, uv[:, 0], uv[:, 1]).reshape(size, size)
            if not np.isfinite(search).all():
                reject('outside_image')
                continue
            if np.std(search) < cfg.min_std:
                reject('image_texture')
                continue
            scores = ncc_valid(search, self.templates[i].astype(float))
            row, col = np.unravel_index(np.argmax(scores), scores.shape)
            score = float(scores[row, col])
            if not np.isfinite(score) or score <= cfg.min_score:
                reject('score')
                continue
            other = scores.copy()
            g = cfg.exclusion_px
            other[max(0, row - g):row + g + 1, max(0, col - g):col + g + 1] = -np.inf
            gap = score - float(np.max(other))
            if not np.isfinite(gap) or gap <= cfg.min_gap:
                reject('ambiguous')
                continue
            peak = _peak(scores, row, col)
            if peak is None:
                reject('peak')
                continue
            shift = (peak - cfg.search_radius_px)*cfg.resolution_m
            observed = self.points[i] + np.array([shift[0], shift[1], 0.])
            pixel, z = camera.project(observed[None, :], p, r)
            if not np.isfinite(pixel).all() or z[0] <= 1e-6:
                reject('invalid_projection')
                continue
            kept.append(i)
            pixels.append(pixel[0])
            scores_out.append(score)
            gaps.append(gap)
        if not kept:
            result.reason = 'no_visible_sites' if not chosen else 'all_refused'
            return result
        result.points = self.points[kept].copy()
        result.pixels = np.asarray(pixels)
        result.indices = self.indices[kept].copy()
        result.scores, result.gaps = np.asarray(scores_out), np.asarray(gaps)
        result.pixel_var = np.full(len(kept), cfg.pixel_sigma**2)
        # Resolution allowance is declared, not an uncertainty inferred from
        # NCC score. It cannot cover a wrong datum or a consistent set of aliases.
        map_sigma = max(cfg.map_sigma_m, self.map_resolution/math.sqrt(12.))
        result.map_var = np.tile([map_sigma**2, map_sigma**2, 0.], (len(kept), 1))
        return result

    def match_filter(self, image, camera, nav):
        """Predict at exposure time exactly as Eskf.landmark_model, without updating it."""
        try:
            offset = float(nav.time_offset)
            velocity, rate = np.asarray(nav.v, dtype=float), np.asarray(nav.rate, dtype=float)
            if (not math.isfinite(offset) or velocity.shape != (3,) or rate.shape != (3,)
                    or not np.isfinite(velocity).all() or not np.isfinite(rate).all()):
                return FineMatches(reason='invalid_filter_pose')
            with np.errstate(over='raise', invalid='raise'):
                p = nav.p + velocity*offset
                r = nav.r @ exp_so3(rate*offset)
        except (ValueError, TypeError, AttributeError, OverflowError, FloatingPointError):
            return FineMatches(reason='invalid_filter_pose')
        return self.match(image, camera, p, r)
