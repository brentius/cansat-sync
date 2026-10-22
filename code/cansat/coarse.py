"""GPL-3.0-only. Coarse acquisition: position and heading from one image and the map, with yaw unknown.

The navigation filter (eskf.py) has to be started within a few metres and a
few degrees of the truth, and the fine matcher only finds a landmark that is
already within its +/-16 m window. Before the first fix none of that is known:
the can is somewhere within a few hundred metres of the launch pad and has
been spinning, so its heading is anywhere on the circle. This module produces
the landmark matches that firstfix.solve_first_fix turns into the first pose.

Where it runs. Raspberry Pi Zero 2 W, NumPy only, once per image until a fix
is confirmed. It needs from the STM32: the IMU samples (for Leveller) and the
barometric height above the ground. GPS is not an input.

    on the pad   CoarseMap(raster, res, origin)      features and FFTs of the 1 m/px map
    in flight    Leveller.update(dt, accel, gyro)    tilt from the mean specific force
                 acquire(image, camera, r0, h, cmap) one image -> matches -> FirstFix
                                                     (123 FFTs of 256 x 256, 200 to 320 of 128 x 128)
                 Acquirer.update(...)                two images that agree -> confirmed fix

Method, in four stages.

1. Whole-view search over position and yaw. The image is rectified to the
   ground plane with the levelled attitude r0 (tilt known, yaw arbitrary) and
   the barometric height. A disc of ground, the largest that fits the view, is
   compared with the map at every position in the prior region and at every
   yaw hypothesis by normalised cross-correlation. Rotating the disc is a
   resampling, and because the disc's shape does not depend on yaw the two
   normalisation sums are computed once per image, so each hypothesis costs
   one forward and one inverse FFT of the 256 x 256 stage-1 grid.

   What is correlated is not brightness but edge strength: the gradient
   magnitude at 2 m/px, compressed as g/(g + mean g), blurred and decimated to
   4 m/px. A field that has been ploughed or re-sown since the map image was
   taken changes brightness, and the contrast across a boundary can reverse,
   but the boundary is still there. The blur (8 m) is what lets a hypothesis
   that is a few degrees or a few percent of height out still correlate.

2. Landmark matches at the best hypotheses. Up to 20 sites are chosen in
   the rectified image where the structure is corner-like (smaller eigenvalue
   of the structure tensor, one site per cell of a grid over the view), so a
   straight hedge, which locates a patch only across itself, and tramlines are
   avoided. A 64 m patch of edge strength around each site is turned to the
   hypothesis yaw and located in the 2 m/px map within +/-32 m. Each match is
   a (map point, image pixel) pair that does not depend on the hypothesis
   being exact, only on it being close enough for the patch to correlate.

3. An interim pose from firstfix.solve_first_fix on those pairs: consensus
   and a refinement of all six degrees of freedom, which removes most of the
   levelling error. It is run with relaxed checks (solver_interim), because
   this pose only has to be close enough for stage 4, and that is what lets
   terrain with few corners through: with the full checks here, fixes on
   large-field terrain fell from 40 % of images to 13 % in simulation.

4. Refinement at 1 m/px. With the stage-3 pose, up to 48 patches of 48 m are
   resampled straight onto the map grid, so they are no longer distorted by
   the levelling error, and each is looked for only within 8 m of where that
   pose puts it. A match displaced further than that (the shadow of a tree
   that has moved with the sun, an alias on repeating texture) cannot be
   returned at all. The solver runs again on these matches with its full
   checks, and its answer is the fix, provided its tilt is within 8 degrees
   of the levelled one. If the map is supplied at 2 m/px only, stage 4 is
   skipped and stage 3 runs under the full checks; in simulation that fix is
   less often available and less accurate.

What limits it. A camera above flat ground can hardly tell a tilt from a
sideways shift: only the stretch of the view towards its edges separates
them. With a dozen matches, some of them on shadows that have moved, the
solver can trade one for the other. In simulation that is where the fixes
more than 10 m out came from (about 1 % of fixes on mixed farmland, about
9 % on large fields, typically with 3 degrees of false tilt), and a few
were 20 to 50 m out with 4 to 17 degrees. Two images that agree through
the gyro (Acquirer) removed every one of the latter. Straight boundaries,
which are plentiful where corners are not, are not used: a patch on one is
located across it but not along it, and the solver takes one scalar
precision per match.

The covariance returned with the fix is the solver's scaled by cov_scale
squared plus an allowance for errors common to all matches (see
CoarseConfig): the solver assumes independent match errors, and matches on
shadows that have moved do not err independently.

A whole-view alignment of all six degrees of freedom on the edge images was
tried in place of stages 2 and 3, since it can use straight boundaries that
are useless as landmarks. On terrain whose brightness and shadows differed
from the map it did not separate tilt from a sideways shift and left the
levelling error in place, so it was removed (wiki/acquisition_first_fix.md).

Everything refuses instead of guessing, and acquire, Acquirer.update and
Leveller.update never raise: a fault is returned as a refusal and counted.
Cost is bounded by the configuration (CoarseConfig.problem sets the limits):
the number of yaw hypotheses, candidates and sites are fixed numbers. A
stage-1 candidate that fits about as well as one that has passed is also
taken through all the stages, and if two that are far apart both pass, the
view fits the map in two places and no fix is given.

Conventions. World is east/north/up from the launch pad; attitudes map body
to world as in eskf.py. A raster value [row, col] is at east = origin[0] +
col*res, north = origin[1] + row*res (north increases with row). A pixel index
is the continuous image coordinate of the pixel centre. The yaw hypothesis psi
is the rotation about world up that takes r0 to the true attitude.

Design-stage code. Checked on synthetic rasters only (code/tests/test_coarse.py
and sim/nav/run_acquisition_mc.py); it has not seen a real image or a real
map, and it has not been timed on the Pi (code/tests/bench_onboard.py).
"""
import math
import time
from dataclasses import dataclass, field

import numpy as np

from .eskf import G0, LEVEL, Camera, exp_so3
from .firstfix import FirstFix, solve_first_fix


@dataclass
class CoarseConfig:
    res2: float = 2.0             # m/px, stage-2 grid and the onboard map
    decimate: int = 2             # stage-1 grid is res2*decimate (4 m/px)
    smooth1_m: float = 8.0        # blur of the stage-1 feature, m
    smooth2_m: float = 2.0        # blur of the stage-2 feature, m
    feature: str = 'edges'        # 'edges' (gradient magnitude, compressed) or 'intensity'
    max_angle_deg: float = 50.0   # rays further than this from straight down are not used
    disc_max_m: float = 200.0     # largest stage-1 disc radius
    disc_fill: float = 0.92       # fraction of the inscribed radius used, so the blur stays inside the view
    yaw_step_deg: float = 6.0     # stage-1 yaw hypothesis spacing
    candidates: int = 3           # stage-1 peaks carried to stage 2
    sites: tuple = (5, 4)         # cells across and along the view, one site per cell
    site_floor: float = 0.05      # weakest site kept, as a fraction of the strongest
    template_px: int = 32         # stage-2 patch, in res2 pixels (64 m)
    search_px: int = 16           # stage-2 search half-width, res2 pixels (32 m)
    min_score: float = 0.45       # stage-2 correlation needed
    min_gap: float = 0.03         # stage-2 lead of the peak over the next one
    gap_exclusion_px: int = 3
    min_matches: int = 5          # fewest matches worth sending to the solver
    match_sigma_m: float = 1.2    # per-axis ground precision claimed for a true match
    max_yaw_disagreement_deg: float = 15.0   # solver yaw against the hypothesis it came from
    min_texture: float = 1e-6     # variance of the disc feature below which the view is called featureless
    # Stage 4, used when the map is supplied at res2/2: re-match with the solved pose at that resolution.
    polish: bool = True
    polish_sites: tuple = (8, 6)
    polish_px: int = 48           # patch, in fine map pixels (48 m at 1 m/px)
    polish_search_px: int = 8     # search half-width, fine map pixels
    polish_sigma_m: float = 1.0   # per-axis ground precision claimed for a polished match
    polish_min_matches: int = 8
    # The solver's covariance assumes independent match errors of the stated size. In simulation the
    # errors were 2 to 2.5 times what it implies (matches on moved shadows err together), so the
    # covariance of the fix that is returned is scaled by cov_scale squared. common_error_m is then added
    # to the horizontal position variance for errors shared by every match (hedge shadows, the map's own
    # georeferencing), which no scatter of matches can reveal.
    cov_scale: float = 2.0
    common_error_m: float = 1.0
    # Stage 3. 'solver': all six degrees of freedom from firstfix.solve_first_fix under solver_interim.
    # 'similarity': position, height and yaw only, at the levelled tilt, which cannot trade tilt against
    # position; in simulation it gave fewer fixes on mixed farmland (87 % against 90 % of images) and no
    # fewer wrong ones on large fields, so it is not the default.
    interim: str = 'solver'
    interim_tolerance_m: float = 4.0      # a match within this of the fitted similarity is counted
    interim_min_spread: float = 0.15      # least RMS spread of the counted matches, as a fraction of height
    # The fix's tilt may differ from the levelled tilt by this much. Levelling is good to a few degrees;
    # a larger difference means the solver has traded tilt against position.
    max_tilt_disagreement_deg: float = 8.0
    # Passed to firstfix.solve_first_fix: for the fix that is returned, and (interim = 'solver') for the
    # stage-3 fix when a stage 4 follows. That fix only has to be close enough for stage 4's search, and
    # stage 4 applies the solver's full checks, so stage 3 may accept a weaker geometry.
    solver: dict = field(default_factory=dict)
    solver_interim: dict = field(default_factory=lambda: {'max_sigma': 12.0, 'max_chance': 0.3})
    # Two stage-1 candidates further apart than this that both pass every later check mean the map
    # looks the same in two places: no fix is given. Once one candidate has passed, a later one is
    # still taken through the stages only if its stage-1 correlation is within rival_margin of it;
    # a place that fits as well as the first is the case that matters, and the rest cost time.
    ambiguity_m: float = 30.0
    rival_margin: float = 0.15

    def problem(self):
        """Empty when the configuration can be used, otherwise the name of what cannot.

        The limits bound the work one image can cause (hypotheses, sites, patch sizes) as well as
        catching mistakes. CoarseMap refuses a bad configuration on the pad; acquire refuses it in flight.
        """
        try:
            checks = (
                (self.feature in ('edges', 'intensity'), 'feature'),
                (self.interim in ('solver', 'similarity'), 'interim'),
                (self.res2 > 0 and self.decimate in (1, 2, 4), 'res2 or decimate'),
                (0 <= self.smooth1_m <= 50 and 0 <= self.smooth2_m <= 20, 'smoothing'),
                (3.0 <= self.yaw_step_deg <= 45.0, 'yaw_step_deg'),
                (1 <= int(self.candidates) <= 5, 'candidates'),
                (len(self.sites) == 2 and all(1 <= int(n) <= 8 for n in self.sites), 'sites'),
                (len(self.polish_sites) == 2 and all(1 <= int(n) <= 10 for n in self.polish_sites), 'polish_sites'),
                (16 <= int(self.template_px) <= 64 and 4 <= int(self.search_px) <= 32, 'template_px or search_px'),
                (16 <= int(self.polish_px) <= 96 and 2 <= int(self.polish_search_px) <= 16, 'polish_px or polish_search_px'),
                (1 <= int(self.gap_exclusion_px) <= 8, 'gap_exclusion_px'),
                (10.0 <= self.max_angle_deg <= 70.0, 'max_angle_deg'),
                (20.0 <= self.disc_max_m <= 400.0 and 0.3 <= self.disc_fill <= 1.0, 'disc'),
                (int(self.min_matches) >= 4 and int(self.polish_min_matches) >= 4, 'min_matches'),
                (self.match_sigma_m > 0 and self.polish_sigma_m > 0 and self.cov_scale > 0 and self.common_error_m >= 0, 'sigmas'),
                (self.ambiguity_m > 0 and self.interim_tolerance_m > 0 and self.rival_margin >= 0, 'tolerances'),
            )
        except Exception:
            return 'unreadable'
        return next((name for ok, name in checks if not ok), '')


# --- Small array tools (no SciPy on the flight computer) -------------------------

def _box(a, width, axis):
    """Running mean of odd `width` along one axis, reflecting at the borders."""
    r = width//2
    pad = [(0, 0), (0, 0)]
    pad[axis] = (r + 1, r)
    c = np.cumsum(np.pad(a, pad, mode='reflect'), axis=axis)
    n = a.shape[axis]
    return (c[width:width + n] - c[:n] if axis == 0 else c[:, width:width + n] - c[:, :n])/width


def _blur(a, sigma):
    """Gaussian blur with reflecting borders; sigma in pixels.

    Below two pixels the kernel is applied directly. Above, three running means stand in for it (their
    product is within a few percent of a Gaussian, and whole-pixel widths put its sigma within 0.2 px
    of the one asked for), so the cost does not grow with sigma. Map and image go through this same
    function, so the approximation cancels in the comparison.
    """
    a = np.asarray(a, dtype=float)
    if sigma <= 0:
        return a.copy()
    if sigma < 2.0:
        radius = min(max(1, int(3*sigma + 0.5)), min(a.shape) - 1)
        x = np.arange(-radius, radius + 1)
        kernel = np.exp(-0.5*(x/sigma)**2)
        kernel /= kernel.sum()
        for axis in (0, 1):
            pad = [(0, 0), (0, 0)]
            pad[axis] = (radius, radius)
            b = np.pad(a, pad, mode='reflect')
            out = np.zeros_like(a)
            n = a.shape[axis]
            for i, w in enumerate(kernel):
                out += w*(b[i:i + n] if axis == 0 else b[:, i:i + n])
            a = out
        return a
    ideal = math.sqrt(12*sigma*sigma/3 + 1)
    lower = int(ideal)
    lower -= 1 - lower % 2                                  # largest odd width not above the ideal
    count = int(round((12*sigma*sigma - 3*lower*lower - 12*lower - 9)/(-4.0*lower - 4)))
    widths = [lower]*count + [lower + 2]*(3 - count)
    limit = min(a.shape) - 2
    for width in widths:
        width = max(1, min(width, limit - (1 - limit % 2)))
        if width > 1:
            a = _box(_box(a, width, 0), width, 1)
    return a


def _blur_valid(a, valid, sigma):
    """Gaussian of the valid samples only; NaN where less than half the weight is valid."""
    weight = _blur(valid.astype(float), sigma)
    value = _blur(np.where(valid, a, 0.0), sigma)
    ok = weight > 0.5
    return np.where(ok, value/np.where(ok, weight, 1.0), np.nan), ok


def _bilinear(image, x, y):
    """image sampled at continuous (x = col, y = row); NaN outside the interpolation domain."""
    rows, cols = image.shape
    ok = (x >= 0) & (y >= 0) & (x <= cols - 1) & (y <= rows - 1)
    xi = np.clip(np.floor(x).astype(int), 0, cols - 2)
    yi = np.clip(np.floor(y).astype(int), 0, rows - 2)
    dx, dy = x - xi, y - yi
    out = ((1 - dx)*(1 - dy)*image[yi, xi] + dx*(1 - dy)*image[yi, xi + 1]
           + (1 - dx)*dy*image[yi + 1, xi] + dx*dy*image[yi + 1, xi + 1])
    return np.where(ok, out, np.nan)


def _rot(psi):
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s], [s, c]])


def _rz(psi):
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def heading_between(r, r0):
    """Yaw psi for which r is closest to Rz(psi) r0."""
    m = r @ r0.T
    return math.atan2(m[1, 0] - m[0, 1], m[0, 0] + m[1, 1])


def bin_image(image, b):
    """Mean of b x b blocks; rows and columns that do not fill a block are dropped."""
    image = np.asarray(image, dtype=float)
    if b <= 1:
        return image
    rows, cols = (image.shape[0]//b)*b, (image.shape[1]//b)*b
    return image[:rows, :cols].reshape(rows//b, b, cols//b, b).mean(axis=(1, 3))


def bin_camera(camera, b):
    """Intrinsics of bin_image's output: block centres sit at b*i + (b - 1)/2 in the original."""
    if b <= 1:
        return camera
    half = (b - 1)/2
    return Camera(camera.fx/b, camera.fy/b, (camera.cx - half)/b, (camera.cy - half)/b,
                  int(camera.width//b), int(camera.height//b), camera.r_cb, camera.lever)


def ncc_valid(search, template):
    """Normalised cross-correlation of template at every position where it lies wholly inside search.

    Result [py, px] is for the template's first pixel at search[py, px]. Positions where the search
    patch has no variance are zero. FFT numerator, integral-image normalisation.
    """
    sh, sw = search.shape
    th, tw = template.shape
    shape = (1 << (sh + th - 2).bit_length(), 1 << (sw + tw - 2).bit_length())
    search = search - search.mean()
    tc = template - template.mean()
    raw = np.fft.irfft2(np.fft.rfft2(search, shape)*np.conj(np.fft.rfft2(tc, shape)), s=shape)[:sh - th + 1, :sw - tw + 1]

    def window_sums(a):
        integral = np.pad(a.cumsum(axis=0).cumsum(axis=1), ((1, 0), (1, 0)))
        return integral[th:, tw:] - integral[:-th, tw:] - integral[th:, :-tw] + integral[:-th, :-tw]

    sums, squares = window_sums(search), window_sums(search*search)
    energy = np.maximum(squares - sums*sums/(th*tw), 0.0)
    den = np.sqrt(energy*float(np.sum(tc*tc)))
    floor = 1e-9*max(float(den.max()), 1e-300)
    return np.where(den > floor, raw/np.where(den > floor, den, 1.0), 0.0)


def _subpixel(scores, iy, ix):
    """Parabolic refinement of an interior peak; (dy, dx) in (-1, 1), or None on a border or a flat top."""
    if not (0 < iy < scores.shape[0] - 1 and 0 < ix < scores.shape[1] - 1):
        return None
    out = []
    for lo, mid, hi in ((scores[iy - 1, ix], scores[iy, ix], scores[iy + 1, ix]),
                        (scores[iy, ix - 1], scores[iy, ix], scores[iy, ix + 1])):
        curve = lo - 2*mid + hi
        if not curve < -1e-12:
            return None
        out.append(max(-1.0, min(1.0, 0.5*(lo - hi)/curve)))
    return out[0], out[1]


def edge_strength(intensity, valid=None):
    """Gradient magnitude compressed to g/(g + mean g): 0 on flat ground, towards 1 on a strong edge.

    The compression makes a faint boundary count nearly as much as a strong one, which matters because
    the contrast across a field boundary changes with the season. Returns (feature, valid).
    """
    a = np.asarray(intensity, dtype=float)
    valid = np.isfinite(a) if valid is None else (valid & np.isfinite(a))
    a = np.where(valid, a, 0.0)
    gx, gy = np.zeros_like(a), np.zeros_like(a)
    gx[:, 1:-1] = (a[:, 2:] - a[:, :-2])/2
    gy[1:-1, :] = (a[2:, :] - a[:-2, :])/2
    ok = np.zeros_like(valid)
    ok[1:-1, 1:-1] = (valid[1:-1, 1:-1] & valid[1:-1, 2:] & valid[1:-1, :-2] & valid[2:, 1:-1] & valid[:-2, 1:-1])
    g = np.hypot(gx, gy)
    scale = float(g[ok].mean()) if ok.any() else 0.0
    if scale <= 0:
        return np.zeros_like(a), ok
    return np.where(ok, g/(g + scale), 0.0), ok


def _features(intensity, valid, cfg):
    """Stage-2 feature (res2), stage-1 feature (res2*decimate) and their validity masks."""
    if cfg.feature == 'intensity':
        base, ok = np.where(valid, intensity, 0.0), valid
    else:
        base, ok = edge_strength(intensity, valid)
    f2, ok2 = _blur_valid(base, ok, cfg.smooth2_m/cfg.res2)
    f1, ok1 = _blur_valid(base, ok, cfg.smooth1_m/cfg.res2)
    d = cfg.decimate
    return f2, ok2, f1[::d, ::d], ok1[::d, ::d]


# --- Levelling ---------------------------------------------------------------------

class Leveller:
    """Tilt from the mean specific force, heading carried by the gyro.

    Under an open canopy at terminal speed the can's mean acceleration over a few swing periods is
    close to zero, so its mean specific force points up. The samples are turned into a frame that the
    gyro keeps fixed and averaged there with time constant tau; attitude() is body-to-world with the
    tilt taken from that average and a heading that is arbitrary but continuous, which is what the
    yaw search needs and what lets two fixes be compared (Acquirer).

    What it is not. The average is a first-order low-pass, not a mean over whole swings, and "mean
    acceleration is zero" is false in a sustained turn or gust: an independent pendulum model gave
    2 to 4 degrees of error under 20 degrees of swing and 8 degrees in a spiral glide of 5 m/s on a
    15 m radius. Samples whose magnitude is outside force_band (in g) are not averaged, so the shocks
    of ejection and canopy opening do not leave a false tilt behind; the gyro is integrated regardless.
    update never raises: a sample it cannot use is counted in `ignored`.
    """

    MAX_RATE = 50.0                     # rad/s; the BMI088 saturates at 35

    def __init__(self, tau=2.0, force_band=(0.5, 2.0)):
        self.tau = float(tau)
        if not (math.isfinite(self.tau) and self.tau > 0):
            raise ValueError('the time constant must be positive')
        self.band = (float(force_band[0])*G0, float(force_band[1])*G0)
        self.ignored = 0
        self.reset()

    def reset(self):
        self.rg = LEVEL.copy()          # body to the gyro frame (level if the can is level at the start)
        self.force = np.zeros(3)        # mean specific force in the gyro frame; weight is how much of it is real
        self.weight = 0.0
        self._steps = 0

    def update(self, dt, accel, gyro):
        try:
            dt = float(dt)
            accel, gyro = np.asarray(accel, dtype=float), np.asarray(gyro, dtype=float)
            if accel.shape != (3,) or gyro.shape != (3,) or not 0 < dt <= 1.0:
                self.ignored += 1
                return
            with np.errstate(over='ignore', invalid='ignore'):
                force2, rate2 = float(accel @ accel), float(gyro @ gyro)   # not finite if any component is not
            if not (math.isfinite(force2) and rate2 <= self.MAX_RATE**2):
                self.ignored += 1
                return
            if self.band[0]**2 <= force2 <= self.band[1]**2:
                gain = 1 - math.exp(-dt/self.tau)
                self.force += gain*(self.rg @ exp_so3(gyro*(dt/2)) @ accel - self.force)
                self.weight += gain*(1 - self.weight)
            self.rg = self.rg @ exp_so3(gyro*dt)
            self._steps += 1
            if self._steps % 200 == 0:      # keep it a rotation
                u, _, vt = np.linalg.svd(self.rg)
                self.rg = u @ vt
            if not math.isfinite(float(self.rg.sum() + self.force.sum())):
                self.reset()
                self.ignored += 1
        except Exception:
            self.ignored += 1

    def attitude(self):
        """Body-to-world rotation with arbitrary heading, or None before any usable sample."""
        norm = float(np.linalg.norm(self.force))
        if self.weight <= 0 or norm < 1e-6:
            return None
        up = self.force/norm
        axis = np.array([up[1], -up[0], 0.0])                 # up x z
        s = float(np.linalg.norm(axis))
        if s < 1e-12:
            return self.rg.copy() if up[2] > 0 else None
        return exp_so3(axis/s*math.atan2(s, up[2])) @ self.rg


# --- The onboard map ---------------------------------------------------------------

class CoarseMap:
    """What is prepared on the pad from the map: features, FFTs and the georeference.

    raster[row, col] is the map brightness at (origin[0] + col*res, origin[1] + row*res), at 1 m/px
    (half the stage-2 spacing; kept for stage 4) or at 2 m/px (stage 4 is then skipped). A window
    1024 m square centred on `centre` is used with the defaults, which allows the can to be up to
    about 300 m from `centre`. Raises ValueError (on the pad, not in flight) if the raster does not
    cover the window.
    """

    def __init__(self, raster, res, origin, centre=(0.0, 0.0), n1=256, config=None):
        self.cfg = cfg = config or CoarseConfig()
        if cfg.problem():
            raise ValueError('bad configuration: ' + cfg.problem())
        raster = np.asarray(raster, dtype=float)
        size = n1*cfg.decimate                                  # stage-2 window, pixels
        if abs(res - cfg.res2) < 1e-9:
            per = 1
        elif abs(2*res - cfg.res2) < 1e-9:
            per = 2
        else:
            raise ValueError('map resolution must be the stage-2 resolution or half of it')
        col0 = int(round((centre[0] - origin[0])/res)) - per*size//2
        row0 = int(round((centre[1] - origin[1])/res)) - per*size//2
        if row0 < 0 or col0 < 0 or row0 + per*size > raster.shape[0] or col0 + per*size > raster.shape[1]:
            raise ValueError('map raster does not cover the search window')
        window = raster[row0:row0 + per*size, col0:col0 + per*size]
        if not np.all(np.isfinite(window)):
            raise ValueError('map window contains missing values')
        corner = (origin[0] + col0*res, origin[1] + row0*res)   # world position of the window's pixel [0, 0]
        self.fine, self.res0, self.origin0 = None, None, None
        if per == 2:
            # Keep the 1 m/px window for stage 4; stages 1 to 3 use its 2 x 2 block means.
            self.fine, self.res0, self.origin0 = window, float(res), corner
            window = window.reshape(size, 2, size, 2).mean(axis=(1, 3))
            corner = (corner[0] + res/2, corner[1] + res/2)
        self.n1, self.size = n1, size
        self.res2, self.res1 = cfg.res2, cfg.res2*cfg.decimate
        self.origin = corner                                    # world position of stage-2 pixel [0, 0]
        self.centre = (float(centre[0]), float(centre[1]))
        f2, _, f1, _ = _features(window, np.ones(window.shape, dtype=bool), cfg)
        self.f2 = f2
        self.f1 = f1 - f1.mean()
        self.f1_hat = np.fft.rfft2(self.f1)
        self.f1sq_hat = np.fft.rfft2(self.f1*self.f1)
        self._disc_cache = {}

    def disc(self, radius_px):
        """Offsets of the disc and the map's local sum and energy under it (cached per radius)."""
        k = int(radius_px)
        if k not in self._disc_cache:
            dy, dx = np.mgrid[-k:k + 1, -k:k + 1]
            inside = dx*dx + dy*dy <= k*k
            dy, dx = dy[inside], dx[inside]
            mask = np.zeros((self.n1, self.n1))
            mask[dy % self.n1, dx % self.n1] = 1.0
            m_hat = np.conj(np.fft.rfft2(mask))
            total = np.fft.irfft2(self.f1_hat*m_hat, s=mask.shape)
            square = np.fft.irfft2(self.f1sq_hat*m_hat, s=mask.shape)
            energy = np.maximum(square - total*total/len(dy), 0.0)
            self._disc_cache = {k: (dy, dx, energy)}
        return self._disc_cache[k]

    def world(self, row, col, level=2):
        """World east, north of a (fractional) index of the stage-2 (level 2) or stage-1 (level 1) grid."""
        res = self.res2 if level == 2 else self.res1
        return self.origin[0] + col*res, self.origin[1] + row*res


# --- One image -----------------------------------------------------------------------

@dataclass
class Acquisition:
    """Result of acquire. fix is a firstfix.FirstFix when ok."""
    ok: bool
    reason: str
    fix: FirstFix = None
    yaw: float = math.nan            # yaw hypothesis that produced the fix (or the best one), rad
    score: float = math.nan          # its stage-1 correlation
    lead: float = math.nan           # stage-1 best score minus the best at a different place or heading
    rank: int = -1                   # which stage-1 candidate produced the fix
    candidates: list = field(default_factory=list)   # (yaw, east, north, score) of the IMU, best first
    sites: int = 0                   # stage-2 sites offered
    matches: int = 0                 # matches sent to the solver
    pixels: np.ndarray = None        # (matches, 2) image pixels, in the coordinates of the image given
    points: np.ndarray = None        # (matches, 3) map points
    scores: np.ndarray = None        # (matches,) stage-2 correlations
    disc_m: float = math.nan         # stage-1 disc radius
    coarse_fix: FirstFix = None      # the stage-3 fix, kept when stage 4 replaced or refused it
    polished: bool = False           # fix comes from stage 4
    binning: int = 1
    ffts: dict = field(default_factory=dict)         # FFT size -> count, the cost that scales on the Pi
    seconds: dict = field(default_factory=dict)      # host time per stage


def _inscribed(corners, nadir, cone):
    """Centre and radius of a large circle inside the convex quadrilateral and the cone about nadir."""
    area = 0.0
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        area += a[0]*b[1] - b[0]*a[1]
    sign = 1.0 if area > 0 else -1.0
    edges = []
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        d = b - a
        length = float(np.hypot(d[0], d[1]))
        if length < 1e-9:
            return None
        edges.append((a, d/length))

    def radius(points):
        r = cone - np.hypot(points[:, 0] - nadir[0], points[:, 1] - nadir[1])
        for a, d in edges:
            r = np.minimum(r, sign*(d[0]*(points[:, 1] - a[1]) - d[1]*(points[:, 0] - a[0])))
        return r

    lo = np.maximum(corners.min(axis=0), nadir - cone)
    hi = np.minimum(corners.max(axis=0), nadir + cone)
    best = None
    for _ in range(3):
        gx, gy = np.meshgrid(np.linspace(lo[0], hi[0], 15), np.linspace(lo[1], hi[1], 15))
        pts = np.column_stack((gx.ravel(), gy.ravel()))
        r = radius(pts)
        i = int(np.argmax(r))
        best = (pts[i], float(r[i]))
        span = (hi - lo)/14
        lo, hi = pts[i] - span, pts[i] + span
    return best if best[1] > 0 else None


def _rectify(image, cam, r0, height, cfg):
    """Image resampled onto a ground grid in the levelled frame (origin under the IMU, heading of r0)."""
    rows, cols = image.shape
    corner_px = np.array([(0.0, 0.0), (cols - 1.0, 0.0), (cols - 1.0, rows - 1.0), (0.0, rows - 1.0)])
    rays = cam.ray(corner_px) @ r0.T
    lever = r0 @ cam.lever
    hz = height + lever[2]
    if hz <= 0 or np.any(rays[:, 2] > -1e-3):
        return 'tilt too large'                                 # part of the image is at or above the horizon
    corners = lever[:2] + rays[:, :2]*(hz/-rays[:, 2])[:, None]
    nadir = lever[:2]
    cone = hz*math.tan(math.radians(cfg.max_angle_deg))
    found = _inscribed(corners, nadir, cone)
    if found is None:
        return 'view too small'
    anchor, inscribed = found
    lo = np.maximum(corners.min(axis=0), nadir - cone)
    hi = np.minimum(corners.max(axis=0), nadir + cone)
    half = min(0.5*float(max(hi - lo)), 450.0)
    mid = (lo + hi)/2
    n = int(2*half/cfg.res2)
    n -= n % cfg.decimate
    if n < 4*cfg.template_px:
        return 'view too small'                                 # too low for patches of this size
    origin = mid - (n - 1)/2*cfg.res2
    axis = np.arange(n)*cfg.res2
    gx, gy = np.meshgrid(origin[0] + axis, origin[1] + axis)
    points = np.column_stack((gx.ravel(), gy.ravel(), np.zeros(n*n)))
    pix, depth = cam.project(points, np.array([0.0, 0.0, height]), r0)
    with np.errstate(invalid='ignore'):
        value = _bilinear(image, pix[:, 0], pix[:, 1])
    inside = (depth > 0) & np.isfinite(value) & (np.hypot(points[:, 0] - nadir[0], points[:, 1] - nadir[1]) <= cone)
    return {'intensity': np.where(inside, value, np.nan).reshape(n, n), 'valid': inside.reshape(n, n),
            'origin': origin, 'anchor': anchor, 'inscribed': inscribed, 'n': n}


def _pick_sites(rect, f_valid, cfg, cells=None, patch_px=None):
    """Corner-like sites of the rectified image, one per cell, strongest first: (row, col) indices."""
    cells = cells or cfg.sites
    patch_px = patch_px or cfg.template_px
    a = np.where(rect['valid'], rect['intensity'], 0.0)
    valid = rect['valid']
    gx, gy = np.zeros_like(a), np.zeros_like(a)
    gx[:, 1:-1] = (a[:, 2:] - a[:, :-2])/2
    gy[1:-1, :] = (a[2:, :] - a[:-2, :])/2
    ok = np.zeros_like(valid)
    ok[1:-1, 1:-1] = valid[1:-1, 1:-1] & valid[1:-1, 2:] & valid[1:-1, :-2] & valid[2:, 1:-1] & valid[:-2, 1:-1]
    gx, gy = np.where(ok, gx, 0.0), np.where(ok, gy, 0.0)
    scale = float(np.hypot(gx, gy)[ok].mean()) if ok.any() else 0.0
    if scale <= 0:
        return np.zeros((0, 2), dtype=int)
    # The same compression as the feature, so one bright roof does not own every site.
    norm = 1.0/(np.hypot(gx, gy) + scale)
    gx, gy = gx*norm, gy*norm
    window = patch_px/4
    jxx, jyy, jxy = _blur(gx*gx, window), _blur(gy*gy, window), _blur(gx*gy, window)
    strength = (jxx + jyy)/2 - np.sqrt(((jxx - jyy)/2)**2 + jxy*jxy)
    # A site needs its whole patch, at any heading, inside the view.
    reach = int(math.ceil(patch_px*0.75))
    room = _blur(f_valid.astype(float), reach/2.0) > 0.98
    room[:reach], room[-reach:], room[:, :reach], room[:, -reach:] = False, False, False, False
    strength = np.where(room, strength, 0.0)
    if not strength.max() > 0:
        return np.zeros((0, 2), dtype=int)
    rr, cc = np.nonzero(room)
    r0, r1, c0, c1 = rr.min(), rr.max() + 1, cc.min(), cc.max() + 1
    nx, ny = cells
    floor = cfg.site_floor*float(strength.max())
    sites = []
    for iy in range(ny):
        for ix in range(nx):
            ra, rb = r0 + (r1 - r0)*iy//ny, r0 + (r1 - r0)*(iy + 1)//ny
            ca, cb = c0 + (c1 - c0)*ix//nx, c0 + (c1 - c0)*(ix + 1)//nx
            cell = strength[ra:rb, ca:cb]
            if cell.size == 0:
                continue
            k = int(np.argmax(cell))
            value = float(cell.flat[k])
            if value > floor:
                sites.append((value, ra + k//cell.shape[1], ca + k % cell.shape[1]))
    sites.sort(reverse=True)
    return np.array([(r, c) for _, r, c in sites], dtype=int).reshape(-1, 2)


def _count(ffts, size, n=1):
    ffts[size] = ffts.get(size, 0) + n


def _stage1(rect, f1, cmap, cfg, yaws, prior_radius, ffts):
    """Correlate the disc at every yaw. Returns candidates [(score, yaw, row, col)] best first, and the disc."""
    res1 = cmap.res1
    radius_m = min(cfg.disc_max_m, cfg.disc_fill*rect['inscribed'])
    k = int(radius_m/res1)
    if k < 8:
        return None, 'view too small'
    dy, dx, energy = cmap.disc(k)
    _count(ffts, cmap.n1, 3)
    n1 = cmap.n1
    # Positions where the disc lies inside the map window (no wrap-around), the local map has structure
    # and the IMU would be inside the prior region. The disc centre is the IMU plus the turned anchor,
    # so the region is widened by the anchor's length instead of being redrawn for every yaw.
    anchor = rect['anchor']
    usable = np.zeros((n1, n1), dtype=bool)
    usable[k:n1 - k, k:n1 - k] = True
    usable &= energy > 1e-3*float(energy[k:n1 - k, k:n1 - k].mean())
    grid_n, grid_e = np.mgrid[0:n1, 0:n1]
    grid_e = cmap.origin[0] + grid_e*res1 - cmap.centre[0]
    grid_n = cmap.origin[1] + grid_n*res1 - cmap.centre[1]
    usable &= grid_e**2 + grid_n**2 <= (prior_radius + float(np.hypot(anchor[0], anchor[1])))**2
    weight = np.where(usable, 1.0/np.sqrt(np.where(usable, energy, 1.0)), 0.0)
    off = np.column_stack((dx, dy))*res1                       # map-frame offsets from the disc centre, m
    origin1 = rect['origin']                                   # f1's [0, 0] in the levelled frame
    volume = np.zeros((len(yaws), n1, n1), dtype=np.float32)
    template = np.zeros((n1, n1))
    done = 0
    for i, psi in enumerate(yaws):
        s = anchor + off @ _rot(-psi).T                        # levelled-frame position of each disc sample
        with np.errstate(invalid='ignore'):
            t = _bilinear(f1, (s[:, 0] - origin1[0])/res1, (s[:, 1] - origin1[1])/res1)
        good = np.isfinite(t)
        if good.sum() < 0.9*len(t):
            continue
        t = np.where(good, t, t[good].mean())
        t = t - t.mean()
        power = float(t @ t)
        if power/len(t) < cfg.min_texture:
            return None, 'featureless view'
        template[:] = 0.0
        template[dy % n1, dx % n1] = t
        corr = np.fft.irfft2(cmap.f1_hat*np.conj(np.fft.rfft2(template)), s=template.shape)
        _count(ffts, n1, 2)
        volume[i] = corr*weight
        volume[i] *= 1.0/math.sqrt(power)
        done += 1
    if not done:
        return None, 'view too small'
    # The best peaks of the whole volume. After each one its neighbourhood (nearby places at nearby
    # headings) is blanked, so the next is a rival: somewhere else, or the same place facing another way.
    step = abs(yaws[1] - yaws[0]) if len(yaws) > 1 else math.radians(cfg.yaw_step_deg)
    full = len(yaws) > 2 and abs(len(yaws)*step - 2*math.pi) < 1e-6
    reach = max(1, k//2)
    out = []
    for _ in range(cfg.candidates + 1):
        i, row, col = (int(v) for v in np.unravel_index(int(np.argmax(volume)), volume.shape))
        score = float(volume[i, row, col])
        if not score > 0:
            break
        # Refine the yaw with a parabola through the neighbouring hypotheses at this place.
        psi = yaws[i]
        lo, hi = ((i - 1) % len(yaws), (i + 1) % len(yaws)) if full else (i - 1, i + 1)
        if len(yaws) > 2 and 0 <= lo < len(yaws) and 0 <= hi < len(yaws):
            r0, r1, c0, c1 = max(row - 2, 0), row + 3, max(col - 2, 0), col + 3
            a_, b_, c_ = (float(volume[q, r0:r1, c0:c1].max()) for q in (lo, i, hi))
            curve = a_ - 2*b_ + c_
            if curve < -1e-9:
                psi = psi + step*max(-0.5, min(0.5, 0.5*(a_ - c_)/curve))
        sub = _subpixel(volume[i], row, col) or (0.0, 0.0)
        out.append((score, psi, row + sub[0], col + sub[1]))
        for d in (-2, -1, 0, 1, 2):
            q = (i + d) % len(yaws) if full else i + d
            if 0 <= q < len(yaws):
                volume[q, max(row - reach, 0):row + reach + 1, max(col - reach, 0):col + reach + 1] = -1.0
    if not out:
        return None, 'no agreement with the map'
    return (out, radius_m), 'ok'


def _stage2(rect, f2, sites, cmap, cfg, psi, imu_xy, ffts):
    """Locate each site's patch in the map. Returns (site index, map east, map north, score) rows."""
    t, w, res = cfg.template_px, cfg.search_px, cfg.res2
    grid = (np.arange(t) - (t - 1)/2)*res
    vx, vy = np.meshgrid(grid, grid)
    back = _rot(-psi)
    sx = back[0, 0]*vx + back[0, 1]*vy                         # levelled-frame offsets of the patch samples
    sy = back[1, 0]*vx + back[1, 1]*vy
    turn = _rot(psi)
    origin = rect['origin']
    found = []
    for q, (row, col) in enumerate(sites):
        site = origin + np.array([col, row])*res               # levelled-frame position of the site
        with np.errstate(invalid='ignore'):
            patch = _bilinear(f2, (site[0] + sx - origin[0])/res, (site[1] + sy - origin[1])/res)
        if not np.all(np.isfinite(patch)):
            continue
        centre = imu_xy + turn @ site                          # where the hypothesis puts it on the map
        x0 = (centre[0] - cmap.origin[0])/res - (t - 1)/2
        y0 = (centre[1] - cmap.origin[1])/res - (t - 1)/2
        c0, r0 = int(round(x0)) - w, int(round(y0)) - w
        if r0 < 0 or c0 < 0 or r0 + t + 2*w > cmap.size or c0 + t + 2*w > cmap.size:
            continue
        scores = ncc_valid(cmap.f2[r0:r0 + t + 2*w, c0:c0 + t + 2*w], patch)
        _count(ffts, 1 << (2*t + 2*w - 2).bit_length(), 3)
        iy, ix = np.unravel_index(int(np.argmax(scores)), scores.shape)
        score = float(scores[iy, ix])
        sub = _subpixel(scores, iy, ix)
        if sub is None or score < cfg.min_score:
            continue
        other = scores.copy()
        g = cfg.gap_exclusion_px
        other[max(0, iy - g):iy + g + 1, max(0, ix - g):ix + g + 1] = -1.0
        if score - float(other.max()) < cfg.min_gap:
            continue
        east, north = cmap.world(r0 + iy + sub[0] + (t - 1)/2, c0 + ix + sub[1] + (t - 1)/2)
        found.append((q, east, north, score))
    return found


def _polish(image, camera, fix, r0, height, rect, ok2, cam_rect, b_rect, cmap, cfg, rng, ffts):
    """Stage 4: re-match at the map's full resolution with the solved pose, then solve again.

    Patches are resampled straight onto the map grid with the stage-3 pose, so they are no longer
    distorted by the levelling error, and each is looked for only within a few metres of where that
    pose puts it: a match displaced further than that (a tree's shadow that has moved, an alias) cannot
    be returned at all. Returns (FirstFix, pixels, points, scores), or None when there is too little.
    """
    t, w, res = cfg.polish_px, cfg.polish_search_px, cmap.res0
    sites = _pick_sites(rect, ok2, cfg, cfg.polish_sites, int(math.ceil(t*res/cfg.res2)))
    if len(sites) < cfg.polish_min_matches:
        return None
    # Where each site is on the ground according to the solved pose.
    site_xy = rect['origin'] + sites[:, ::-1]*cfg.res2
    pix, _ = cam_rect.project(np.column_stack((site_xy, np.zeros(len(sites)))), np.array([0.0, 0.0, height]), r0)
    rays = camera.ray(pix*b_rect + (b_rect - 1)/2) @ fix.r.T
    centre = fix.p + fix.r @ camera.lever
    if np.any(rays[:, 2] > -1e-3):
        return None
    ground = centre[:2] + rays[:, :2]*(centre[2]/-rays[:, 2])[:, None]
    b = max(1, int(res*camera.fx/height))
    cam, small = bin_camera(camera, b), bin_image(image, b)
    size = cmap.fine.shape[0]
    span = np.arange(t + 2)
    blur = cfg.smooth2_m/cfg.res2
    pixels, points, scores = [], [], []
    for gx, gy in ground:
        c0 = int(round((gx - cmap.origin0[0])/res - (t - 1)/2))
        r0w = int(round((gy - cmap.origin0[1])/res - (t - 1)/2))
        if r0w - w - 1 < 0 or c0 - w - 1 < 0 or r0w + t + w + 1 > size or c0 + t + w + 1 > size:
            continue
        ex, ny = np.meshgrid(cmap.origin0[0] + (c0 - 1 + span)*res, cmap.origin0[1] + (r0w - 1 + span)*res)
        pts = np.column_stack((ex.ravel(), ny.ravel(), np.zeros(ex.size)))
        uv, depth = cam.project(pts, fix.p, fix.r)
        if np.any(depth <= 0):
            continue
        with np.errstate(invalid='ignore'):
            patch = _bilinear(small, uv[:, 0], uv[:, 1]).reshape(t + 2, t + 2)
        if not np.all(np.isfinite(patch)):
            continue
        template = _blur(edge_strength(patch)[0][1:-1, 1:-1], blur)
        window = cmap.fine[r0w - w - 1:r0w + t + w + 1, c0 - w - 1:c0 + t + w + 1]
        search = _blur(edge_strength(window)[0][1:-1, 1:-1], blur)
        ncc = ncc_valid(search, template)
        _count(ffts, 1 << (2*t + 2*w - 2).bit_length(), 3)
        iy, ix = np.unravel_index(int(np.argmax(ncc)), ncc.shape)
        score = float(ncc[iy, ix])
        sub = _subpixel(ncc, iy, ix)
        if sub is None or score < cfg.min_score:
            continue
        other = ncc.copy()
        g = cfg.gap_exclusion_px
        other[max(0, iy - g):iy + g + 1, max(0, ix - g):ix + g + 1] = -1.0
        if score - float(other.max()) < cfg.min_gap:
            continue
        # The pixel under the patch centre, and the map point the patch was found at.
        mid = np.array([[cmap.origin0[0] + (c0 + (t - 1)/2)*res, cmap.origin0[1] + (r0w + (t - 1)/2)*res, 0.0]])
        pixels.append(camera.project(mid, fix.p, fix.r)[0][0])
        points.append((cmap.origin0[0] + (c0 - w + ix + sub[1] + (t - 1)/2)*res,
                       cmap.origin0[1] + (r0w - w + iy + sub[0] + (t - 1)/2)*res, 0.0))
        scores.append(score)
    if len(points) < cfg.polish_min_matches:
        return None
    pixels, points = np.array(pixels), np.array(points)
    _, depth = camera.project(points, fix.p, fix.r)
    sigma = cfg.polish_sigma_m*camera.fx/np.maximum(depth, 1.0)
    return solve_first_fix(camera, pixels, points, height, sigma, rng, r0=fix.r, **cfg.solver), pixels, points, np.array(scores)


def _similarity(site_xy, map_xy, psi, imu_xy, height, cfg):
    """Stage 3: yaw, scale and position that carry the levelled-frame sites onto their map matches.

    With the tilt held at the levelled value a ground point at u in the levelled frame lies at
    z u + t on the map, z = s exp(i psi) (Appendix L.2), so the fit is linear and cannot trade tilt
    against position. Starting from the stage-1 hypothesis, matches within a shrinking tolerance are
    kept and the fit repeated. Returns (yaw, scale, east, north, kept mask) or None.
    """
    u = site_xy[:, 0] + 1j*site_xy[:, 1]
    m = map_xy[:, 0] + 1j*map_xy[:, 1]
    z, t = complex(math.cos(psi), math.sin(psi)), complex(imu_xy[0], imu_xy[1])
    keep = None
    for tolerance in (4*cfg.interim_tolerance_m, 2*cfg.interim_tolerance_m, cfg.interim_tolerance_m, cfg.interim_tolerance_m):
        keep = np.abs(m - (z*u + t)) <= tolerance
        if keep.sum() < 4:
            return None
        centre = u[keep].mean()
        spread = float(np.sum(np.abs(u[keep] - centre)**2))
        if spread < 1e-9:
            return None
        z = complex(np.sum(np.conj(u[keep] - centre)*(m[keep] - m[keep].mean()))/spread)
        t = complex(m[keep].mean() - z*centre)
    scale = abs(z)
    if (not 0.75 < scale < 1.33 or math.sqrt(spread/keep.sum()) < cfg.interim_min_spread*height
            or abs(wrap(math.atan2(z.imag, z.real) - psi)) > math.radians(cfg.max_yaw_disagreement_deg)):
        return None
    return math.atan2(z.imag, z.real), scale, t.real, t.imag, keep


def acquire(image, camera, r0, height, cmap, prior_radius=300.0, yaw_prior=None, config=None, rng=None):
    """One image to a first fix, or a refusal with the reason. Never raises.

    image: 2-D brightness array. camera: eskf.Camera whose intrinsics describe that image (after
    undistortion). r0: body-to-world attitude with the right tilt and any heading (Leveller.attitude).
    height: barometric height of the IMU above the ground, m. cmap: CoarseMap. prior_radius: how far
    from cmap.centre the can may be, m. yaw_prior: None to search the whole circle, or (psi, half
    width) in radians when a compass gives the rotation from r0's heading to the true one. config:
    thresholds for this call if not the map's own; the feature settings must be the map's.
    """
    out = Acquisition(False, 'fault')
    try:
        return _acquire(image, camera, r0, height, cmap, prior_radius, yaw_prior, config or cmap.cfg, rng, out)
    except Exception as error:                                  # last resort: a refusal, not a crash
        out.ok, out.fix = False, None
        out.reason = 'fault: %s' % type(error).__name__
        return out


def _rotation(m):
    return (m.shape == (3, 3) and np.all(np.isfinite(m)) and float(np.abs(m @ m.T - np.eye(3)).max()) < 1e-6
            and float(np.linalg.det(m)) > 0)


def _acquire(image, camera, r0, height, cmap, prior_radius, yaw_prior, cfg, rng, out):
    clock = time.perf_counter
    t0 = clock()
    if cfg.problem():
        out.reason = 'bad configuration: ' + cfg.problem()
        return out
    if any(getattr(cfg, name) != getattr(cmap.cfg, name) for name in ('res2', 'decimate', 'smooth1_m', 'smooth2_m', 'feature')):
        out.reason = 'configuration does not match the map'
        return out
    image = np.asarray(image, dtype=float)
    r0 = np.asarray(r0, dtype=float)
    height, prior_radius = float(height), float(prior_radius)
    if (image.ndim != 2 or min(image.shape) < 32 or max(image.shape) > 4096 or not _rotation(r0)
            or not (math.isfinite(height) and 10.0 < height < 5000.0) or not (math.isfinite(prior_radius) and prior_radius > 0)
            or not (math.isfinite(camera.fx) and math.isfinite(camera.fy) and camera.fx > 0 and camera.fy > 0)
            or not _rotation(np.asarray(camera.r_cb, dtype=float)) or not np.all(np.isfinite(camera.lever))
            or (camera.width and camera.height and image.shape != (camera.height, camera.width))):
        out.reason = 'bad input'
        return out
    if not np.all(np.isfinite(image)):
        out.reason = 'bad image'
        return out
    # Bring the image to about the map's ground resolution before resampling it.
    b = max(1, int(cfg.res2*camera.fx/height))
    cam = bin_camera(camera, b)
    small = bin_image(image, b)
    out.binning = b
    if min(small.shape) < 16:
        out.reason = 'view too small'
        return out
    rect = _rectify(small, cam, r0, float(height), cfg)
    if isinstance(rect, str):
        out.reason = rect
        return out
    f2, ok2, f1, _ = _features(rect['intensity'], rect['valid'], cfg)
    t1 = clock()
    # Yaw hypotheses. A prior as wide as the circle is no prior; the count is bounded by the configuration.
    count = int(round(360.0/cfg.yaw_step_deg))
    step = 2*math.pi/count
    if yaw_prior is None:
        yaws = np.arange(count)*step
    else:
        centre, width = float(yaw_prior[0]), abs(float(yaw_prior[1]))
        if not (math.isfinite(centre) and math.isfinite(width)):
            out.reason = 'bad input'
            return out
        half = int(math.ceil(width/step))
        yaws = np.arange(count)*step if 2*half + 1 >= count else centre + np.arange(-half, half + 1)*step
    reach = max(0.0, min(float(prior_radius), cmap.size*cmap.res2/2 - cfg.disc_max_m))
    result, reason = _stage1(rect, f1, cmap, cfg, yaws, reach, out.ffts)      # f1 shares rect's grid origin
    t2 = clock()
    out.seconds = {'rectify': t1 - t0, 'stage1': t2 - t1, 'stage4': 0.0}
    if result is None:
        out.reason = reason
        return out
    peaks, out.disc_m = result
    anchor = rect['anchor']
    for score, psi, row, col in peaks:
        e, n = cmap.world(row, col, level=1)
        imu = np.array([e, n]) - _rot(psi) @ anchor
        out.candidates.append((psi, float(imu[0]), float(imu[1]), score))
    out.yaw, out.score = peaks[0][1], peaks[0][0]
    out.lead = peaks[0][0] - peaks[1][0] if len(peaks) > 1 else math.nan
    sites = _pick_sites(rect, ok2, cfg)
    out.sites = len(sites)
    if len(sites) < cfg.min_matches:
        out.reason = 'too little structure'
        out.seconds['stage2'] = clock() - t2
        return out
    # Image pixel of each site, in the coordinates of the image that was passed in.
    site_xy = rect['origin'] + sites[:, ::-1]*cfg.res2
    pix, depth = cam.project(np.column_stack((site_xy, np.zeros(len(sites)))), np.array([0.0, 0.0, float(height)]), r0)
    pix = pix*b + (b - 1)/2
    sigma = cfg.match_sigma_m*camera.fx/np.maximum(depth, 1.0)
    refine = cfg.polish and cmap.fine is not None
    up_level = r0.T @ np.array([0.0, 0.0, 1.0])

    def attempt(psi, e, n):
        """Stages 2 to 4 from one stage-1 candidate: a result, or (reason, interim fix or None)."""
        found = _stage2(rect, f2, sites, cmap, cfg, psi, np.array([e, n]), out.ffts)
        if len(found) < cfg.min_matches:
            return 'too few matches', None
        idx = np.array([f[0] for f in found])
        points = np.array([(f[1], f[2], 0.0) for f in found])
        if refine and cfg.interim == 'similarity':
            fitted = _similarity(site_xy[idx], points[:, :2], psi, (e, n), height, cfg)
            if fitted is None:
                return 'no consensus', None
            yaw, scale, east, north, keep = fitted
            fix = FirstFix(True, 'ok', p=np.array([east, north, scale*height]), r=_rz(yaw) @ r0, inliers=keep)
        else:
            fix = solve_first_fix(camera, pix[idx], points, float(height), sigma[idx], rng, r0=r0,
                                  **(cfg.solver_interim if refine else cfg.solver))
        if fix.ok:
            # The solver must land near the hypothesis that produced its matches.
            d_yaw = abs(wrap(heading_between(fix.r, r0) - psi))
            d_pos = math.hypot(fix.p[0] - e, fix.p[1] - n)
            if d_yaw > math.radians(cfg.max_yaw_disagreement_deg) or d_pos > cfg.search_px*cfg.res2 + 0.25*height:
                fix.ok, fix.reason = False, 'disagrees with the search'
        if not fix.ok:
            return fix.reason, None
        found_at = {'fix': fix, 'interim': None, 'pixels': pix[idx], 'points': points,
                    'scores': np.array([f[3] for f in found]), 'polished': False}
        if refine:
            t3 = clock()
            polished = _polish(image, camera, fix, r0, height, rect, ok2, cam, b, cmap, cfg, rng, out.ffts)
            out.seconds['stage4'] += clock() - t3
            if polished is None or not polished[0].ok:
                # The interim fix alone is not good enough to hand to the filter: better another image.
                return 'refinement: ' + ('too few matches' if polished is None else polished[0].reason), fix
            found_at.update(fix=polished[0], interim=fix, pixels=polished[1], points=polished[2], scores=polished[3], polished=True)
        if float((found_at['fix'].r.T @ np.array([0.0, 0.0, 1.0])) @ up_level) < math.cos(math.radians(cfg.max_tilt_disagreement_deg)):
            return 'tilt disagrees with levelling', found_at['interim'] or found_at['fix']
        return found_at, None

    # The search does not stop at the first candidate that works: a rival place that fits about as
    # well in stage 1 and also passes every later check is the one sign that the map is ambiguous here.
    passed, refusals = [], []
    for rank, (psi, e, n, score) in enumerate(out.candidates[:cfg.candidates]):
        if passed and score < passed[0]['score'] - cfg.rival_margin:
            break
        result, interim = attempt(psi, e, n)
        if isinstance(result, str):
            refusals.append((result, interim))
        else:
            result.update(rank=rank, yaw=psi, score=score)
            passed.append(result)
    out.seconds['stage2'] = clock() - t2 - out.seconds['stage4']
    if not passed:
        out.reason, out.coarse_fix = refusals[0] if refusals else ('no candidates', None)
        return out
    for i, first in enumerate(passed):
        for second in passed[i + 1:]:
            if math.hypot(*(first['fix'].p - second['fix'].p)[:2]) > cfg.ambiguity_m:
                out.coarse_fix = first['fix']
                out.reason = 'ambiguous: the view fits the map in two places'
                return out
    best = max(passed, key=lambda found_at: (int(found_at['fix'].inliers.sum()), -found_at['rank']))
    out.rank, out.yaw, out.score, out.fix, out.coarse_fix = best['rank'], best['yaw'], best['score'], best['fix'], best['interim']
    out.pixels, out.points, out.scores, out.polished = best['pixels'], best['points'], best['scores'], best['polished']
    out.matches = len(out.points)
    out.fix.cov = out.fix.cov*cfg.cov_scale**2
    out.fix.cov[0, 0] += cfg.common_error_m**2
    out.fix.cov[1, 1] += cfg.common_error_m**2
    out.ok, out.reason = True, 'ok'
    return out


# --- Two images that agree -------------------------------------------------------

class Acquirer:
    """Accepts a first fix only when two images give fixes that agree.

    A single image can produce a confident fix in the wrong place, and neither the solver nor the
    navigation filter can tell afterwards (firstfix.py, eskf.py). Two fixes from different images are
    compared through the gyro: the attitude of the second must equal the attitude of the first turned
    by the body rotation in between, and the position must have moved no further than the can can
    fly. When they agree the second fix is confirmed and the pair gives the horizontal velocity the
    filter is started with.

    What this does and does not catch. In simulation it rejected every fix that was 20 m or more out
    because the solver had traded tilt for position: such a fix has several degrees of false tilt,
    and the next image does not repeat it. It does not catch an error both images share. Two images a
    second apart see the same ground, so a map that really is the same in two places fools both
    (acquire refuses that case itself when both places pass), and so would a tilt error that repeats.
    The position test is loose by construction, because the velocity is not yet known. The body
    rotation is taken from the two levelled attitudes, so a change in the levelled tilt between the
    images counts against agreement; that errs towards refusing.
    """

    def __init__(self, cmap, config=None, max_speed=15.0, attitude_tol_deg=3.0, max_gap_s=3.0, min_gap_s=0.2,
                 prior_radius=300.0):
        self.cmap, self.cfg = cmap, config or cmap.cfg
        self.max_speed, self.attitude_tol = max_speed, math.radians(attitude_tol_deg)
        self.max_gap, self.min_gap, self.prior_radius = max_gap_s, min_gap_s, prior_radius
        self.pending = None             # (t, fix, r0) of an unconfirmed fix
        self.attempts = self.fixes = self.disagreements = self.faults = 0

    def update(self, t, image, camera, r0, height, yaw_prior=None, rng=None):
        """Returns (status, acquisition, velocity): status is 'none', 'pending' or 'confirmed'. Never raises."""
        self.attempts += 1
        try:
            return self._update(float(t), image, camera, r0, height, yaw_prior, rng)
        except Exception as error:                              # a fault here must not poison later calls
            self.faults += 1
            self.pending = None
            return 'none', Acquisition(False, 'fault: %s' % type(error).__name__), None

    def _update(self, t, image, camera, r0, height, yaw_prior, rng):
        if not math.isfinite(t):
            return 'none', Acquisition(False, 'bad input'), None
        acq = acquire(image, camera, r0, height, self.cmap, self.prior_radius, yaw_prior, self.cfg, rng)
        if acq.reason.startswith('fault'):
            self.faults += 1
        if self.pending is not None and not 0 <= t - self.pending[0] <= self.max_gap:
            self.pending = None                                 # too old, or the clock went backwards
        if not acq.ok:
            return 'none', acq, None
        self.fixes += 1
        previous = self.pending
        if previous is not None and t - previous[0] < self.min_gap:
            return 'pending', acq, None                         # the same look twice proves nothing; keep the first
        self.pending = (t, acq.fix, np.array(r0, dtype=float))
        if previous is None:
            return 'pending', acq, None
        t_a, fix_a, r0_a = previous
        dt = t - t_a
        predicted = fix_a.r @ r0_a.T @ self.pending[2]          # first attitude carried forward by the gyro
        m = predicted.T @ acq.fix.r
        angle = math.acos(max(-1.0, min(1.0, (m[0, 0] + m[1, 1] + m[2, 2] - 1)/2)))
        moved = acq.fix.p - fix_a.p
        sigma = math.sqrt(max(float(np.trace(fix_a.cov[:2, :2]) + np.trace(acq.fix.cov[:2, :2])), 0.0))
        if angle > self.attitude_tol or math.hypot(moved[0], moved[1]) > self.max_speed*dt + 3*sigma + 2.0:
            self.disagreements += 1
            return 'pending', acq, None
        return 'confirmed', acq, moved/dt
