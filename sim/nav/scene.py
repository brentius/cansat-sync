"""GPL-3.0-only. SYNTHETIC camera views of a raster world, and the onboard map cut from it.

Simulation support, not flight software. A world (sim/nav/farmland.py) is a set of rasters on one
grid: `flight` is what the ground looks like on the day, `map` what the orthoimage showed. This
module renders what the camera would record from a given pose and prepares the 2 m/px map the
flight code is given. Only NumPy is required.

Grid convention (as farmland.py and code/cansat/coarse.py): raster[row, col] is the value at
east = origin[0] + col*res, north = origin[1] + row*res. A pixel index is the continuous image
coordinate of the pixel centre. Attitudes map body to world; the camera model is cansat.eskf.Camera.

What the renderer does and does not do. Each pixel centre is cast to the ground plane and the world
is sampled there from a box-filter pyramid at the level matching that pixel's ground footprint, so a
pixel integrates roughly the ground it covers. There is no occlusion or parallax (tall things are
painted on the ground plane unless relief is asked for), no lens distortion, no rolling shutter and
no exposure smear; optical blur, gain, offset and noise are simple image-plane operations.
"""
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]/'code'))
from cansat.coarse import _blur  # noqa: E402


def block_mean(a, k):
    rows, cols = (a.shape[0]//k)*k, (a.shape[1]//k)*k
    return a[:rows, :cols].reshape(rows//k, k, cols//k, k).mean(axis=(1, 3))


class Pyramid:
    """Box-filter pyramid of one raster: level i has res*2**i metres per pixel."""

    def __init__(self, raster, res, origin, levels=5):
        self.levels = [(np.asarray(raster, dtype=np.float32), float(res), (float(origin[0]), float(origin[1])))]
        for _ in range(1, levels):
            a, r, o = self.levels[-1]
            if min(a.shape) < 8:
                break
            self.levels.append((block_mean(a, 2).astype(np.float32), 2*r, (o[0] + r/2, o[1] + r/2)))
        self.fill = float(np.mean(self.levels[-1][0]))

    def _level(self, i, east, north):
        a, r, o = self.levels[i]
        x, y = (east - o[0])/r, (north - o[1])/r
        rows, cols = a.shape
        ok = (x >= 0) & (y >= 0) & (x <= cols - 1) & (y <= rows - 1)
        xi = np.clip(np.floor(x).astype(np.int64), 0, cols - 2)
        yi = np.clip(np.floor(y).astype(np.int64), 0, rows - 2)
        dx, dy = (x - xi).astype(np.float32), (y - yi).astype(np.float32)
        out = ((1 - dx)*(1 - dy)*a[yi, xi] + dx*(1 - dy)*a[yi, xi + 1] + (1 - dx)*dy*a[yi + 1, xi] + dx*dy*a[yi + 1, xi + 1])
        return np.where(ok, out, np.float32(self.fill)), ok

    def sample(self, east, north, footprint):
        """World sampled at (east, north) with a pixel footprint of `footprint` metres (scalar or array)."""
        base = self.levels[0][1]
        level = np.clip(np.log2(np.maximum(np.asarray(footprint, dtype=float), 1e-6)/base), 0.0, len(self.levels) - 1.0)
        lo = np.floor(level).astype(int)
        frac = (level - lo).astype(np.float32)
        out = np.zeros(np.shape(east), dtype=np.float32)
        inside = np.ones(np.shape(east), dtype=bool)
        for i in range(len(self.levels)):
            use_lo, use_hi = (lo == i), (lo == i - 1) & (frac > 0)
            if np.ndim(use_lo) == 0:
                use_lo, use_hi = np.full(out.shape, bool(use_lo)), np.full(out.shape, bool(use_hi))
            sel = use_lo | use_hi
            if not sel.any():
                continue
            value, ok = self._level(i, east[sel], north[sel])
            weight = np.where(use_lo[sel], 1 - np.broadcast_to(frac, out.shape)[sel], np.broadcast_to(frac, out.shape)[sel])
            out[sel] += weight*value
            inside[sel] &= ok
        return out, inside


def render(pyramid, camera, p, r, size, height_pyramid=None, blur_px=0.0, noise=0.0, gain=1.0, offset=0.0,
           rng=None, quantise=False):
    """Image (rows, cols) a camera with these intrinsics records from IMU position p and attitude r.

    camera's intrinsics must describe an image of `size`. height_pyramid, if given, is the height above
    ground of what is seen; the ray is then intersected with that surface by fixed-point iteration (no
    occlusion: adequate for hedges seen from near overhead, not for tall buildings at the image edge).
    Returns (image, fraction of pixels that saw the world).
    """
    rows, cols = size
    u, v = np.meshgrid(np.arange(cols, dtype=float), np.arange(rows, dtype=float))
    rays = camera.ray(np.column_stack((u.ravel(), v.ravel()))) @ r.T
    centre = np.asarray(p, dtype=float) + r @ camera.lever
    down = rays[:, 2] < -1e-6
    depth = np.where(down, centre[2]/np.where(down, -rays[:, 2], 1.0), np.inf)
    footprint = depth/camera.fx
    east, north = centre[0] + rays[:, 0]*depth, centre[1] + rays[:, 1]*depth
    if height_pyramid is not None:
        for _ in range(3):
            surface, _ = height_pyramid.sample(np.where(down, east, 0.0), np.where(down, north, 0.0), np.where(down, footprint, 1.0))
            depth = np.where(down, (centre[2] - surface)/np.where(down, -rays[:, 2], 1.0), np.inf)
            east, north = centre[0] + rays[:, 0]*depth, centre[1] + rays[:, 1]*depth
    value, inside = pyramid.sample(np.where(down, east, 0.0), np.where(down, north, 0.0), np.where(down, footprint, 1.0))
    seen = down & inside
    image = np.where(seen, value, pyramid.fill).reshape(rows, cols).astype(float)
    if blur_px > 0:
        image = _blur(image, blur_px)
    image = gain*image + offset
    if noise > 0:
        image = image + noise*(rng or np.random.default_rng(0)).standard_normal(image.shape)
    if quantise:
        image = np.clip(np.rint(image), 0, 255)
    return image, float(seen.mean())


def onboard_map(raster, res, origin, res2=2.0, datum=(0.0, 0.0)):
    """The map as the flight code holds it: block-averaged to res2, with its georeference.

    datum is the map's georeferencing error: a feature truly at x is recorded at x + datum.
    Returns (raster2, res2, origin2).
    """
    k = int(round(res2/res))
    if abs(k*res - res2) > 1e-9 or k < 1:
        raise ValueError('map resolution must be a whole multiple of the world resolution')
    return (block_mean(np.asarray(raster, dtype=float), k), res2,
            (origin[0] + (k - 1)/2*res + datum[0], origin[1] + (k - 1)/2*res + datum[1]))


def stub_world(seed, size_m=1600.0, res=0.5, cells=90, hedge_m=3.0, texture=4.0):
    """A minimal stand-in for farmland.generate: Voronoi fields with dark boundaries. For tests only.

    Returns (flight, map, res, origin): the two rasters share boundaries; `map` has every field's
    brightness redrawn, which is the worst case of seasonal change.
    """
    rng = np.random.default_rng(seed)
    n = int(round(size_m/res))
    origin = (-(n - 1)/2*res, -(n - 1)/2*res)
    seeds = rng.uniform(0, n, (cells, 2))
    yy, xx = np.mgrid[0:n, 0:n].astype(np.float32)
    best = np.full((n, n), np.inf, dtype=np.float32)
    second = np.full((n, n), np.inf, dtype=np.float32)
    label = np.zeros((n, n), dtype=np.int32)
    for i, (sx, sy) in enumerate(seeds):
        d = np.hypot(xx - np.float32(sx), yy - np.float32(sy))
        closer = d < best
        second = np.where(closer, best, np.minimum(second, d))
        label = np.where(closer, i, label)
        best = np.where(closer, d, best)
    hedge = (second - best) < hedge_m/res
    out = []
    for _ in range(2):
        albedo = rng.uniform(80, 180, cells)
        a = albedo[label].astype(np.float32)
        a += texture*rng.standard_normal((n, n)).astype(np.float32)
        a[hedge] = 50.0
        out.append(a)
    return out[0], out[1], res, origin
