"""GPL-3.0-only. SYNTHETIC independent checks of code/cansat/coarse.py (coarse acquisition).

This is SYNTHETIC verification code, not flight software, and nothing in it validates flight
performance. It checks coarse.py by methods that do not share its assumptions: its own renderer
(ray casting written here, no pyramid), its own terrain (straight-line fields, round trees, square
roofs; not farmland.py, not scene.py), its own brute-force spatial-domain matcher (no FFT), and its
own pendulum integrator. Nothing is imported from sim/nav/scene.py, and from coarse.py only the
public entry points under test.

Run from the project root:  <python> sim/nav/test_acquisition_independent.py

FINDINGS (all on synthetic rasters, 364 x 272 px camera of f = 181 px at 270 to 320 m; numbers from
longer runs made by hand with the same functions, of which the tests below repeat a small part)
-----------------------------------------------------------------------------------------------
Defects found on the first pass. D1 to D7 were then fixed in coarse.py (11 October 2026) and their tests
now run as ordinary tests; what remains marked expectedFailure is a limit of the map window, described
in its test. As first reported:
 D1 Ambiguous map -> confident false fix. With a 520 m tile of the map duplicated, 5 of 20 images
    fixed on the copy (520 m out, claimed sigma 3 m, reason 'ok'). Acquisition.lead is never tested
    and cannot see a rival at the same heading: stage 1 keeps one peak per yaw slice.
 D2 Acquirer confirmed one such pair (1 of 6): two images of the same terrain share the alias.
 D3 Acquirer 'confirms' the identical frame delivered twice (dt = 1e-6 s).
 D4 Acquirer.update raises TypeError on t=None once a fix is pending, and on every later refused
    image until the next fix.
 D5 Leveller: one sample with dt=inf, or a finite accel of 1e200, disables it for good (NaN or None
    attitude after 10 s of good samples); wrong shapes, dt=None and gyro=1e200 raise.
 D6 The number of yaw hypotheses is unbounded in the yaw prior's width (and in 1/yaw_step_deg).
 D7 r0 is not checked to be a rotation (2*r0 gives 'ok' with a smaller sigma). Minor.
 N1 Misleading reason: 'tilt too large' is also what a level can gets below about 110 m (the view is
    smaller than four templates), for an image smaller than the camera says, and for fx = 0.
 N2 Unknown strings in CoarseConfig.feature and .interim are silently taken as the defaults; a config
    passed to acquire that differs from the map's (feature, smoothing) is not detected.
Held:
 - bin_camera against bin_image: centroid agreement better than 0.05 px for b = 2, 3, 4.
 - Stage 1 against the own brute-force search, 3 poses: same optimum within one grid step (4 m, 6 deg);
   stage-1 error 0.1 to 2.1 m and 0.0 to 0.4 deg over 14 poses.
 - (map point, pixel) pairs, 14 poses, all headings: stage 4 mean (-0.002, 0.014) m in map axes,
   (-0.004, 0.003) m in image axes, standard error 0.007 to 0.010 m; stage 2 mean (0.029, -0.003) and
   (-0.010, -0.002) m, s.e. 0.010 to 0.014 m. Fix error median 0.22 m, max 0.38 m (stage 4).
 - Non-square raster: identical to the last bit. World, map and pose moved by (12345, -6789) m: 1e-12 m.
   Window centre moved by (37, -21) m: 6e-4 m. Datum shift (5, 20) m: fix moves by it to 1e-13 m
   (window moved with it) or 1.2e-3 m (window left in place).
 - acquire: 151 malformed calls, none raised; none gave a wrong fix. Normal call 24 MB peak (traced),
   294 to 327 FFTs (docstring says 'about 240'); worst cases 12 to 14 s (yaw step 1 deg; 40 x 40 sites).
 - Adversarial terrain, 6 pairs = 12 images per case (fixes / false fixes / pairs confirmed / confirmed false):
     duplicate tile   12 / 3 / 5 / 1     periodic lines  0 / 0 / 0 / 0     mirror image   0 / 0 / 0 / 0
     off the map       0 / 0 / 0 / 0     half blank      4 / 0 / 2 / 0     half saturated 0 / 0 / 0 / 0
     height +15 %     11 / 0 / 5 / 0     height -15 %   10 / 0 / 4 / 0     r0 tilted 8 deg 11 / 0 / 5 / 0
 - Covariance, own terrain with field brightness redrawn, 2 deg levelling error, 40 of 40 fixed:
   error median 0.24 m, max 1.7 m against a claimed sigma_h of 3.2 m; 100 % inside 1 sigma (39 %
   expected). Here the covariance is about 10 times too LARGE: one cov_scale cannot be right on
   both this terrain and the farmland it was tuned on.
 - Leveller against the own pendulum: exact when hanging still; 20 deg swing of 2 s period 2.2 deg rms;
   20 deg cone 3.2 deg; 6 s period swing 3.6 deg rms; spiral glide (1.67 m/s^2) 8.2 deg.
"""
import math
import sys
import time
import tracemalloc
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'code'))
from cansat.coarse import Acquirer, CoarseConfig, CoarseMap, Leveller, acquire, bin_camera, bin_image  # noqa: E402
from cansat.eskf import Camera  # noqa: E402

DEG = math.pi/180
F_FULL = 2.5e-3/3.45e-6
# Deliberately off-centre principal point and a three-axis lever arm, so that a dropped offset shows.
FULL = Camera(F_FULL, F_FULL, 733.3, 539.1, 1456, 1088, lever=np.array([0.02, -0.01, 0.045]))


def own_binned(cam, b):
    """Intrinsics after b x b binning, derived here: block i covers pixels b*i .. b*i+b-1, centre b*i+(b-1)/2."""
    return Camera(cam.fx/b, cam.fy/b, (cam.cx - (b - 1)/2)/b, (cam.cy - (b - 1)/2)/b, cam.width//b, cam.height//b,
                  lever=cam.lever.copy())


CAM4 = own_binned(FULL, 4)      # 364 x 272, f = 181 px


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rodrigues(v):
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v/a
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a)*kx + (1 - math.cos(a))*(kx @ kx)


def attitude(yaw, tilt=(0.0, 0.0)):
    """Body to world: level can (x east, y south, z down) tilted in body axes, then turned about up."""
    return rz(yaw) @ np.diag([1.0, -1.0, -1.0]) @ rodrigues([tilt[0], tilt[1], 0.0])


def cast(cam, p, r, uv):
    """Ground (z = 0) east, north seen by pixels uv (n, 2) from IMU position p with attitude r. Own algebra."""
    uv = np.asarray(uv, dtype=float)
    d_body = np.stack(((uv[:, 0] - cam.cx)/cam.fx, (uv[:, 1] - cam.cy)/cam.fy, np.ones(len(uv))), axis=1)
    d_world = d_body @ r.T
    c = np.asarray(p, dtype=float) + r @ cam.lever
    s = c[2]/(-d_world[:, 2])
    return np.stack((c[0] + s*d_world[:, 0], c[1] + s*d_world[:, 1]), axis=1)


def sample(raster, origin, res, east, north, fill=120.0):
    x, y = (east - origin[0])/res, (north - origin[1])/res
    rows, cols = raster.shape
    inside = (x >= 0) & (y >= 0) & (x <= cols - 1) & (y <= rows - 1)
    x0 = np.clip(np.floor(x), 0, cols - 2).astype(np.int64)
    y0 = np.clip(np.floor(y), 0, rows - 2).astype(np.int64)
    fx, fy = x - x0, y - y0
    v = (raster[y0, x0]*(1 - fx)*(1 - fy) + raster[y0, x0 + 1]*fx*(1 - fy)
         + raster[y0 + 1, x0]*(1 - fx)*fy + raster[y0 + 1, x0 + 1]*fx*fy)
    return np.where(inside, v, fill)


def render(world, cam, p, r, ss=2, noise=0.0, rng=None):
    """Image of world = (raster, res, origin) through cam: mean of ss x ss sub-pixel rays per pixel."""
    raster, res, origin = world
    rows, cols = cam.height, cam.width
    u, v = np.meshgrid(np.arange(cols, dtype=float), np.arange(rows, dtype=float))
    out = np.zeros(rows*cols)
    for i in range(ss):
        for j in range(ss):
            du, dv = (i + 0.5)/ss - 0.5, (j + 0.5)/ss - 0.5
            g = cast(cam, p, r, np.stack((u.ravel() + du, v.ravel() + dv), axis=1))
            out += sample(raster, origin, res, g[:, 0], g[:, 1])
    image = (out/(ss*ss)).reshape(rows, cols)
    if noise:
        image = image + noise*rng.standard_normal(image.shape)
    return image


def terrain(seed, n=1280, res=1.0, centre=(0.0, 0.0), lines=16, bright_seed=None, blobs=150, boxes=40, shape=None):
    """Fields cut by random straight boundaries, dark round trees, bright rectangular roofs. Not farmland.py.

    bright_seed redraws every field's brightness on the same geometry. Returns (raster float32, res, origin).
    """
    rows, cols = shape or (n, n)
    geo = np.random.default_rng([seed, 1])
    tone = np.random.default_rng([seed, 2] if bright_seed is None else [seed, 3, bright_seed])
    origin = (centre[0] - (cols - 1)/2*res, centre[1] - (rows - 1)/2*res)
    north, east = np.mgrid[0:rows, 0:cols].astype(np.float32)
    east, north = east*res - (cols - 1)/2*res, north*res - (rows - 1)/2*res     # relative to the centre
    code = np.zeros((rows, cols), dtype=np.int64)
    edge = np.zeros((rows, cols), dtype=bool)
    for _ in range(lines):
        th, c = geo.uniform(0, math.pi), geo.uniform(-0.42, 0.42)*min(rows, cols)*res
        d = east*np.float32(math.cos(th)) + north*np.float32(math.sin(th)) - np.float32(c)
        code = code*2 + (d > 0)
        edge |= np.abs(d) < 2.0
    _, inv = np.unique(code.ravel(), return_inverse=True)
    inv = inv.reshape(rows, cols)
    a = tone.uniform(70, 190, int(inv.max()) + 1)[inv].astype(np.float32)
    a += (3.0*tone.standard_normal((rows, cols))).astype(np.float32)
    a[edge] = 45.0
    for _ in range(blobs):
        ce, cn, rad = geo.uniform(east.min(), east.max()), geo.uniform(north.min(), north.max()), geo.uniform(3, 9)
        r0, r1 = int((cn - rad - north[0, 0])/res), int((cn + rad - north[0, 0])/res) + 2
        c0, c1 = int((ce - rad - east[0, 0])/res), int((ce + rad - east[0, 0])/res) + 2
        r0, c0 = max(r0, 0), max(c0, 0)
        sub = (east[r0:r1, c0:c1] - ce)**2 + (north[r0:r1, c0:c1] - cn)**2 < rad*rad
        a[r0:r1, c0:c1][sub] = 35.0
    for _ in range(boxes):
        ce, cn = geo.uniform(east.min(), east.max()), geo.uniform(north.min(), north.max())
        w, h, th = geo.uniform(6, 20), geo.uniform(6, 20), geo.uniform(0, math.pi)
        rad = 0.75*max(w, h)
        r0, r1 = max(int((cn - rad - north[0, 0])/res), 0), int((cn + rad - north[0, 0])/res) + 2
        c0, c1 = max(int((ce - rad - east[0, 0])/res), 0), int((ce + rad - east[0, 0])/res) + 2
        de, dn = east[r0:r1, c0:c1] - ce, north[r0:r1, c0:c1] - cn
        sub = (np.abs(de*math.cos(th) + dn*math.sin(th)) < w/2) & (np.abs(-de*math.sin(th) + dn*math.cos(th)) < h/2)
        a[r0:r1, c0:c1][sub] = 225.0
    return a, res, origin


def random_pose(rng, reach=150.0, heights=(270.0, 320.0), tilt_sd_deg=6.0, centre=(0.0, 0.0)):
    rad, brg = reach*math.sqrt(rng.uniform()), rng.uniform(0, 2*math.pi)
    p = np.array([centre[0] + rad*math.cos(brg), centre[1] + rad*math.sin(brg), rng.uniform(*heights)])
    yaw = rng.uniform(-math.pi, math.pi)
    r = attitude(yaw, np.radians(rng.normal(0.0, tilt_sd_deg, 2)))
    return p, r


def levelled(r, psi, tilt_error=None):
    """r0 with r = Rz(psi) r0 (exact tilt), optionally with a world-frame tilt error (rotation vector, rad)."""
    r0 = rz(-psi) @ r
    if tilt_error is not None:
        r0 = rodrigues([tilt_error[0], tilt_error[1], 0.0]) @ r0
    return r0


def err_h(acq, p):
    return float(np.hypot(acq.fix.p[0] - p[0], acq.fix.p[1] - p[1]))


def wrap_pi(a):
    return (a + math.pi) % (2*math.pi) - math.pi


# --- shared fixtures, built once ------------------------------------------------------

_CACHE = {}


def base_world():
    if 'world' not in _CACHE:
        _CACHE['world'] = terrain(11)
        _CACHE['cmap'] = CoarseMap(*_CACHE['world'])
    return _CACHE['world'], _CACHE['cmap']


def shot(world, rng, **kw):
    """A random pose, its image through the own renderer, and an exactly levelled r0 with a random heading."""
    p, r = random_pose(rng, **kw)
    psi = rng.uniform(-math.pi, math.pi)
    image = render(world, CAM4, p, r, noise=1.0, rng=rng)
    return p, r, psi, levelled(r, psi), image


# --- own brute-force matcher (spatial domain, no FFT) -----------------------------------

def own_block_mean(a, k):
    rows, cols = (a.shape[0]//k)*k, (a.shape[1]//k)*k
    out = np.zeros((rows//k, cols//k))
    for i in range(k):
        for j in range(k):
            out += a[i:rows:k, j:cols:k]
    return out/(k*k)


def brute_force(image, cam, world, r0, height, easts, norths, yaws, k=8):
    """Normalised correlation of the image with the world rendered at every (yaw, north, east).

    The image and camera are binned by k and the world is box-filtered to k*res, then for each
    hypothesis every image pixel is cast to the ground and the world sampled there. Brightness is
    correlated directly. Returns the score volume [yaw, north, east].
    """
    raster, res, origin = world
    coarse = own_block_mean(np.asarray(raster, dtype=float), k)
    c_origin = (origin[0] + (k - 1)/2*res, origin[1] + (k - 1)/2*res)
    cam_k = own_binned(cam, k)
    small = own_block_mean(np.asarray(image, dtype=float), k)
    u, v = np.meshgrid(np.arange(small.shape[1], dtype=float), np.arange(small.shape[0], dtype=float))
    uv = np.stack((u.ravel(), v.ravel()), axis=1)
    a = small.ravel() - small.mean()
    a /= np.linalg.norm(a)
    ee, nn = np.meshgrid(np.asarray(easts, dtype=float), np.asarray(norths, dtype=float))
    out = np.zeros((len(yaws),) + ee.shape)
    for i, psi in enumerate(yaws):
        g = cast(cam_k, np.array([0.0, 0.0, height]), rz(psi) @ r0, uv)
        s = sample(coarse, c_origin, k*res, ee.ravel()[:, None] + g[None, :, 0], nn.ravel()[:, None] + g[None, :, 1])
        s = s - s.mean(axis=1, keepdims=True)
        out[i] = ((s @ a)/np.maximum(np.linalg.norm(s, axis=1), 1e-12)).reshape(ee.shape)
    return out


# --- 1. geometry and conventions --------------------------------------------------------

class BinningTest(unittest.TestCase):
    def test_bin_camera_matches_bin_image_to_a_twentieth_of_a_pixel(self):
        """Bright dots at known ground points: their centroid in bin_image's output is where bin_camera puts them."""
        rng = np.random.default_rng(1)
        p, r = np.array([12.0, -7.0, 300.0]), attitude(0.7, (0.06, -0.04))
        worst = 0.0
        for b in (2, 3, 4):
            cam_b = bin_camera(FULL, b)
            self.assertLess(abs(cam_b.cx - own_binned(FULL, b).cx) + abs(cam_b.cy - own_binned(FULL, b).cy), 1e-12)
            gx, gy = np.meshgrid(np.linspace(-120, 120, 4), np.linspace(-80, 80, 3))     # well separated dots
            pts = np.column_stack((gx.ravel() + rng.uniform(-9, 9, 12), gy.ravel() + rng.uniform(-9, 9, 12), np.zeros(12)))
            full_px, _ = FULL.project(pts, p, r)
            image = np.zeros((FULL.height, FULL.width))
            v, u = np.mgrid[0:FULL.height, 0:FULL.width]
            for x, y in full_px:
                image += np.exp(-0.5*((u - x)**2 + (v - y)**2)/6.0**2)
            small = bin_image(image, b)
            want, _ = cam_b.project(pts, p, r)
            vs, us = np.mgrid[0:small.shape[0], 0:small.shape[1]]
            for x, y in want:
                w = small*(np.hypot(us - x, vs - y) < 30.0/b)
                worst = max(worst, abs((w*us).sum()/w.sum() - x), abs((w*vs).sum()/w.sum() - y))
        self.assertLess(worst, 0.05)


# --- 2. robustness -------------------------------------------------------------------------

def _cam(**kw):
    base = dict(fx=CAM4.fx, fy=CAM4.fy, cx=CAM4.cx, cy=CAM4.cy, width=CAM4.width, height=CAM4.height,
                r_cb=np.eye(3), lever=CAM4.lever.copy())
    base.update(kw)
    return Camera(**base)


def fuzz_cases(image, r0, height, cmap):
    """(label, args tuple, kwargs) for acquire: one argument at a time made malformed, extreme or hostile."""
    rng = np.random.default_rng(3)
    nan_px, inf_px = image.copy(), image.copy()
    nan_px[100, 100], inf_px[5, 5] = math.nan, math.inf
    half_blank, half_sat = image.copy(), image.copy()
    half_blank[:, :image.shape[1]//2], half_sat[:image.shape[0]//2] = 0.0, 255.0
    images = dict(none=None, text='abc', empty=[], scalar=3.0, three_d=np.zeros((8, 272, 364)), one_px=np.ones((1, 1)),
                  thin=image[:31], zeros=np.zeros_like(image), constant=np.full_like(image, 7.0), nan_pixel=nan_px,
                  inf_pixel=inf_px, all_nan=np.full_like(image, math.nan), boolean=image > 120, complex_=image*(1 + 1j),
                  objects=np.full(image.shape, None, dtype=object), huge_values=image*1e300, negative=-image,
                  uint8=np.clip(image, 0, 255).astype(np.uint8), wrong_size=image[:100, :100],
                  big=rng.uniform(0, 255, (1500, 2000)), ragged=[[1.0, 2.0], [3.0]], noise=rng.uniform(0, 255, image.shape),
                  half_blank=half_blank, half_saturated=half_sat, transposed=image.T.copy())
    cams = dict(none=None, fx_zero=_cam(fx=0.0), fy_zero=_cam(fy=0.0), fx_negative=_cam(fx=-CAM4.fx),
                both_negative=_cam(fx=-CAM4.fx, fy=-CAM4.fy), fx_nan=_cam(fx=math.nan), fx_inf=_cam(fx=math.inf),
                fx_huge=_cam(fx=1e12, fy=1e12), fx_tiny=_cam(fx=1e-9, fy=1e-9), cx_nan=_cam(cx=math.nan),
                cx_far=_cam(cx=1e9), rcb_zero=_cam(r_cb=np.zeros((3, 3))), rcb_double=_cam(r_cb=2*np.eye(3)),
                rcb_mirror=_cam(r_cb=np.diag([1.0, 1.0, -1.0])), rcb_shape=_cam(r_cb=np.eye(2)),
                lever_nan=_cam(lever=np.full(3, math.nan)), lever_huge=_cam(lever=np.array([0.0, 0.0, 1e6])),
                lever_shape=_cam(lever=np.zeros(2)), text='camera')
    tilt = lambda deg: rodrigues([deg*DEG, 0.0, 0.0]) @ r0                                   # noqa: E731
    r0s = dict(none=None, identity=np.eye(3), zeros=np.zeros((3, 3)), doubled=2*r0, mirrored=np.diag([1.0, 1.0, -1.0]) @ r0,
               nan=np.full((3, 3), math.nan), one_nan=np.where(np.eye(3) > 0, math.nan, r0), vector=np.ones(3),
               four=np.eye(4), tilt_45=tilt(45), tilt_60=tilt(60), tilt_89=tilt(89), tilt_120=tilt(120),
               random=rng.standard_normal((3, 3)), huge=r0*1e300, text='r0', sheared=r0 + 0.3*np.ones((3, 3)))
    heights = dict(zero=0.0, negative=-300.0, ten=10.0, just_over_ten=10.001, fifty=50.0, hundred=100.0, k2=2000.0,
                   e9=1e9, e308=1e308, nan=math.nan, inf=math.inf, none=None, text='x', array=np.array([300.0, 300.0]),
                   complex_=300 + 1j, minus_inf=-math.inf)
    radii = dict(zero=0.0, negative=-1.0, tiny=1e-9, e12=1e12, nan=math.nan, inf=math.inf, none=None, text='r')
    priors = dict(zero_width=(0.3, 0.0), nan_centre=(math.nan, 1.0), nan_width=(0.0, math.nan), inf_width=(0.0, math.inf),
                  two_circles=(0.0, 4*math.pi), far_centre=(1e9, 0.1), short=(0.0,), text='ab', number=5,
                  negative_width=(0.0, -1.0), inf_centre=(math.inf, 0.1))
    c = CoarseConfig()
    cfgs = dict(sites_zero=replace(c, sites=(0, 0)), sites_one=replace(c, sites=(1, 1)), sites_short=replace(c, sites=(5,)),
                sites_negative=replace(c, sites=(-3, 4)), sites_many=replace(c, sites=(40, 40)),
                candidates_zero=replace(c, candidates=0), candidates_negative=replace(c, candidates=-1),
                candidates_many=replace(c, candidates=50), template_zero=replace(c, template_px=0),
                template_one=replace(c, template_px=1), template_huge=replace(c, template_px=1000),
                template_odd=replace(c, template_px=33), search_zero=replace(c, search_px=0),
                search_negative=replace(c, search_px=-4), search_huge=replace(c, search_px=5000),
                yaw_step_zero=replace(c, yaw_step_deg=0.0), yaw_step_negative=replace(c, yaw_step_deg=-6.0),
                yaw_step_400=replace(c, yaw_step_deg=400.0), yaw_step_nan=replace(c, yaw_step_deg=math.nan),
                yaw_step_1=replace(c, yaw_step_deg=1.0), decimate_zero=replace(c, decimate=0),
                decimate_three=replace(c, decimate=3), res2_zero=replace(c, res2=0.0), res2_negative=replace(c, res2=-2.0),
                res2_one=replace(c, res2=1.0), feature_bogus=replace(c, feature='bogus'),
                feature_intensity=replace(c, feature='intensity'), min_matches_zero=replace(c, min_matches=0),
                polish_px_zero=replace(c, polish_px=0), polish_px_huge=replace(c, polish_px=1000),
                polish_search_zero=replace(c, polish_search_px=0), polish_sites_zero=replace(c, polish_sites=(0, 0)),
                smooth_zero=replace(c, smooth1_m=0.0, smooth2_m=0.0), smooth_negative=replace(c, smooth1_m=-5.0),
                smooth_huge=replace(c, smooth1_m=1e6, smooth2_m=1e6), angle_zero=replace(c, max_angle_deg=0.0),
                angle_90=replace(c, max_angle_deg=90.0), angle_180=replace(c, max_angle_deg=180.0),
                disc_zero=replace(c, disc_max_m=0.0), disc_huge=replace(c, disc_max_m=1e9),
                fill_zero=replace(c, disc_fill=0.0), fill_five=replace(c, disc_fill=5.0),
                solver_bogus=replace(c, solver={'no_such_option': 1}), interim_similarity=replace(c, interim='similarity'),
                interim_bogus=replace(c, interim='bogus'), gap_negative=replace(c, gap_exclusion_px=-5),
                gap_huge=replace(c, gap_exclusion_px=10**6), floor_two=replace(c, site_floor=2.0),
                texture_nan=replace(c, min_texture=math.nan), sigma_zero=replace(c, match_sigma_m=0.0, polish_sigma_m=0.0),
                text='config', number=7)
    good = (image, CAM4, r0, height, cmap)
    for group, index, values in (('image', 0, images), ('camera', 1, cams), ('r0', 2, r0s), ('height', 3, heights),
                                 ('cmap', 4, dict(none=None, text='map', number=3))):
        for name, value in values.items():
            args = list(good)
            args[index] = value
            yield '%s=%s' % (group, name), tuple(args), {}
    for group, values in (('prior_radius', radii), ('yaw_prior', priors), ('config', cfgs), ('rng', dict(text='x', number=5))):
        for name, value in values.items():
            yield '%s=%s' % (group, name), good, {group: value}


def call_guarded(function, *args, **kw):
    """(result, exception or None, seconds)."""
    started = time.perf_counter()
    try:
        return function(*args, **kw), None, time.perf_counter() - started
    except Exception as error:                                   # the point of the test
        return None, error, time.perf_counter() - started


# --- 3. adversarial terrain -----------------------------------------------------------------

DUP_SHIFT = 520.0       # the duplicated tile is pasted this far east of the original


def lines_world(seed=0, period=24.0, n=1280):
    """Parallel dark lines every `period` metres on plain ground, plus two unique roofs."""
    rng = np.random.default_rng([seed, 9])
    north, east = np.mgrid[0:n, 0:n].astype(np.float32) - np.float32((n - 1)/2)
    th = 0.35
    d = east*np.float32(math.cos(th)) + north*np.float32(math.sin(th))
    a = np.full((n, n), 130.0, dtype=np.float32) + (3.0*rng.standard_normal((n, n))).astype(np.float32)
    a[np.abs((d + period/2) % period - period/2) < 1.5] = 60.0
    for ce, cn in ((-70.0, 40.0), (90.0, -110.0)):
        a[(np.abs(east - ce) < 9) & (np.abs(north - cn) < 6)] = 225.0
    return a, 1.0, (-(n - 1)/2, -(n - 1)/2)


def adversarial_setup(case):
    """(scene world, CoarseMap, pose centre, reach) for a named case; cached."""
    key = 'adv:' + case
    if key not in _CACHE:
        world, cmap = base_world()
        raster, res, origin = world
        centre, reach = (0.0, 0.0), 120.0
        if case in ('duplicate', 'duplicate_edge'):
            # The 520 m square of map around (-200, 0) appears a second time around (+320, 0).
            pasted = raster.copy()
            pasted[380:900, 700:1220] = raster[380:900, 180:700]
            if case == 'duplicate':
                # A 1280 m window: both places lie well inside the area stage 1 can search.
                cmap = CoarseMap(pasted, res, origin, n1=320)
            else:
                # The default 1024 m window centred between them, as first tested: each place is 260 m
                # from the centre, where a tilted view's disc can hang over the edge of the window.
                cmap = CoarseMap(pasted, res, origin, centre=(60.0, 0.0))
            centre, reach = (-200.0, 0.0), 20.0
        elif case == 'periodic':
            world = lines_world()
            cmap = CoarseMap(*world)
        elif case == 'mirror':
            world = (raster[:, ::-1].copy(), res, origin)
        elif case == 'offmap':
            world = terrain(12)
        _CACHE[key] = (world, cmap, centre, reach)
    return _CACHE[key]


def adversarial_trial(case, seed):
    """Two images one second apart through an Acquirer. Returns a dict of what happened.

    'false' means a fix reported ok that is more than 10 m from the truth (or any fix at all where
    no right answer exists: mirror, offmap). The second image is taken from a pose that has moved,
    turned 17 degrees and tilted differently; r0 for both is the truth with its heading removed by
    the same angle, as a gyro-carried Leveller heading would be.
    """
    world, cmap, centre, reach = adversarial_setup(case)
    rng = np.random.default_rng([77, seed, sum(map(ord, case))])
    p, r = random_pose(rng, reach=reach, centre=centre)
    psi = rng.uniform(-math.pi, math.pi)
    p2 = p + np.array([4.0, -3.0, -8.0])
    r2 = rz(0.3) @ r @ rodrigues([0.03, -0.04, 0.0])
    error = None
    if case == 'tilt8':
        brg = rng.uniform(0, 2*math.pi)
        error = (8*DEG*math.cos(brg), 8*DEG*math.sin(brg))
    scale = {'height+15': 1.15, 'height-15': 0.85}.get(case, 1.0)
    acquirer = Acquirer(cmap)
    out = dict(case=case, seed=seed, ok=[], err=[], reason=[], status=[], sigma=[])
    for t, (pp, rr) in enumerate(((p, r), (p2, r2))):
        image = render(world, CAM4, pp, rr, noise=1.0, rng=rng)
        if case == 'half_blank':
            image[:, :image.shape[1]//2] = 0.0
        elif case == 'half_saturated':
            image[:image.shape[0]//2] = 255.0
        status, acq, _ = acquirer.update(float(t), image, CAM4, levelled(rr, psi, error), pp[2]*scale)
        e = err_h(acq, pp) if acq.ok else math.nan
        out['ok'].append(bool(acq.ok))
        out['err'].append(e)
        out['reason'].append(acq.reason)
        out['status'].append(status)
        out['sigma'].append(math.sqrt(acq.fix.cov[0, 0] + acq.fix.cov[1, 1]) if acq.ok else math.nan)
    never = case in ('mirror', 'offmap')
    out['false'] = [ok and (never or e > 10.0) for ok, e in zip(out['ok'], out['err'])]
    out['confirmed_false'] = out['status'][1] == 'confirmed' and (out['false'][0] or out['false'][1])
    return out


# --- 5. Leveller against an own pendulum ------------------------------------------------------

G = 9.80665


def pendulum_imu(duration, length=1.0, swing_deg=(20.0, 0.0), cone=False, spin=0.0, pivot_accel=None, dt=0.01, sub=10):
    """IMU samples of a can hanging on a rigid line of `length` below a pivot (the canopy).

    The pivot moves with a prescribed acceleration pivot_accel(t) (world, m/s^2; None for a steady
    glide, which is inertial). The line direction u (unit, pivot to can) obeys
    u'' = (I - u u^T)(g - a_pivot)/L - |u'|^2 u, integrated here by RK4 at dt/sub. The can's z axis
    lies along the line; it does not twist about the line except at the rate `spin`. The only force
    on the can besides gravity is the line, so the accelerometer reads a_can - g. Returns
    (accel (n, 3), gyro (n, 3), attitudes (n, 3, 3) at the end of each sample). cone=True starts a
    conical swing of half-angle swing_deg[0] instead of a planar one.
    """
    gvec = np.array([0.0, 0.0, -G])
    accel_of = pivot_accel or (lambda t: np.zeros(3))
    a0, b0 = swing_deg[0]*DEG, swing_deg[1]*DEG
    u = np.array([math.sin(a0)*math.cos(b0), math.sin(b0), -math.cos(a0)*math.cos(b0)])
    u /= np.linalg.norm(u)
    v = np.zeros(3)
    if cone:
        omega = math.sqrt(G/(length*math.cos(a0)))
        v = omega*np.cross(np.array([0.0, 0.0, 1.0]), u)
    down = np.array([0.0, 0.0, -1.0])
    axis = np.cross(down, u)
    s = float(np.linalg.norm(axis))
    r = (rodrigues(axis/s*math.atan2(s, float(down @ u))) if s > 1e-12 else np.eye(3)) @ np.diag([1.0, -1.0, -1.0])

    def rate(t, u, v):
        ge = gvec - accel_of(t)
        return v, (ge - u*(u @ ge))/length - (v @ v)*u

    h = dt/sub
    accel, gyro, att = [], [], []
    t = 0.0
    for _ in range(int(round(duration/dt))):
        f_sum, w_sum = np.zeros(3), np.zeros(3)
        for _ in range(sub):
            k1 = rate(t, u, v)
            k2 = rate(t + h/2, u + h/2*k1[0], v + h/2*k1[1])
            k3 = rate(t + h/2, u + h/2*k2[0], v + h/2*k2[1])
            k4 = rate(t + h, u + h*k3[0], v + h*k3[1])
            w = np.cross(u, v) + spin*u                           # world angular velocity, no twist but the spin
            f = accel_of(t) + length*k1[1] - gvec                 # specific force at the can, world
            f_sum += r.T @ f
            w_sum += r.T @ w
            r = rodrigues(w*h) @ r
            u = u + h/6*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0])
            v = v + h/6*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
            u /= np.linalg.norm(u)
            v -= u*(u @ v)
            t += h
        accel.append(f_sum/sub)
        gyro.append(w_sum/sub)
        att.append(r.copy())
    return np.array(accel), np.array(gyro), np.array(att)


def leveller_tilt_error(accel, gyro, att, dt=0.01, tau=2.0, settle=10.0):
    """Angle (deg) between where the Leveller says up is, in body axes, and where it is; after `settle` s."""
    lv = Leveller(tau)
    out = []
    for i in range(len(accel)):
        lv.update(dt, accel[i], gyro[i])
        if (i + 1)*dt >= settle:
            r0 = lv.attitude()
            c = float((r0.T @ np.array([0.0, 0.0, 1.0])) @ (att[i].T @ np.array([0.0, 0.0, 1.0])))
            out.append(math.acos(max(-1.0, min(1.0, c)))/DEG)
    return np.array(out)


# --- tests ------------------------------------------------------------------------------------

def memo(key, function, *args):
    if key not in _CACHE:
        _CACHE[key] = function(*args)
    return _CACHE[key]


class GeometryTest(unittest.TestCase):
    def test_stage1_peak_is_the_brute_force_optimum(self):
        """Own spatial-domain search over 11 x 11 positions (4 m) and 60 yaws (6 deg) lands where stage 1 does."""
        world, cmap = base_world()
        rng = np.random.default_rng(21)
        p, r, psi, r0, image = shot(world, rng)
        acq = acquire(image, CAM4, r0, p[2], cmap)
        yaw, east, north, _ = acq.candidates[0]
        yaws = np.arange(60)*6*DEG
        easts = round(p[0]/4)*4 + 2.0 + np.arange(-5, 6)*4.0       # not centred on the truth
        norths = round(p[1]/4)*4 - 1.0 + np.arange(-5, 6)*4.0
        volume = brute_force(image, CAM4, world, r0, p[2], easts, norths, yaws)
        i, a, b = np.unravel_index(int(np.argmax(volume)), volume.shape)
        self.assertGreater(volume[i, a, b], 0.8)
        self.assertLess(volume[i, a, b] - max(volume[:, 0].max(), volume[:, -1].max(), volume[:, :, 0].max(),
                                              volume[:, :, -1].max()), 0.5)     # sanity: the grid border is lower
        self.assertLessEqual(abs(easts[b] - east), 4.0)
        self.assertLessEqual(abs(norths[a] - north), 4.0)
        self.assertLessEqual(abs(wrap_pi(yaws[i] - yaw))/DEG, 6.0)
        self.assertLess(math.hypot(east - p[0], north - p[1]), 4.0)
        self.assertLess(abs(wrap_pi(yaw - psi))/DEG, 2.0)

    def _pair_bias(self, config, count, seed):
        world, cmap = base_world()
        rng = np.random.default_rng(seed)
        in_map, in_image = [], []
        for _ in range(count):
            p, r, psi, r0, image = shot(world, rng)
            acq = acquire(image, CAM4, r0, p[2], cmap, config=config)
            self.assertTrue(acq.ok, acq.reason)
            self.assertLess(err_h(acq, p), 2.0)
            d = acq.points[:, :2] - cast(CAM4, p, r, acq.pixels)       # map point minus where that pixel really looks
            self.assertLess(float(np.hypot(d[:, 0], d[:, 1]).max()), 4.0)
            in_map.append(d.mean(axis=0))
            in_image.append((np.column_stack((d, np.zeros(len(d)))) @ r)[:, :2].mean(axis=0))   # along image u, v
        for name, sample_means in (('map', np.array(in_map)), ('image', np.array(in_image))):
            mean = sample_means.mean(axis=0)
            se = sample_means.std(axis=0, ddof=1)/math.sqrt(count)
            for axis in (0, 1):
                self.assertLess(abs(mean[axis]), max(4*se[axis], 0.03), '%s axes: mean %s m, standard error %s m' % (name, mean, se))
                self.assertLess(abs(mean[axis]), 0.10)

    def test_stage4_pairs_are_unbiased(self):
        """Measured over 14 poses: mean (-0.002, 0.014) m in map axes, (-0.004, 0.003) m in image axes, s.e. 0.01 m."""
        self._pair_bias(None, 5, 5)

    def test_stage2_pairs_are_unbiased(self):
        """Measured over 14 poses: mean (0.029, -0.003) m in map axes (s.e. 0.010, 0.013), (-0.010, -0.002) in image axes."""
        self._pair_bias(replace(CoarseConfig(), polish=False), 3, 5)

    def test_map_placement_and_georeference(self):
        """A non-square raster, a far-off origin, an odd window centre and a datum shift all give the same fix."""
        world, cmap = base_world()
        raster, res, origin = world
        crop = raster[60:1260, 30:1280]                             # 1200 x 1250, not square
        far = np.array([12345.0, -6789.0])
        cm_far = CoarseMap(crop, res, (origin[0] + 30*res + far[0], origin[1] + 60*res + far[1]), centre=tuple(far))
        cm_odd = CoarseMap(raster, res, origin, centre=(37.0, -21.0))
        datum = np.array([5.0, 20.0])                               # the map records a feature at x as x + datum
        cm_datum = CoarseMap(raster, res, (origin[0] + datum[0], origin[1] + datum[1]), centre=tuple(datum))
        cm_datum0 = CoarseMap(raster, res, (origin[0] + datum[0], origin[1] + datum[1]))     # window not moved with it
        rng = np.random.default_rng(22)
        for _ in range(1):
            p, r, psi, r0, image = shot(world, rng, reach=100.0)
            base = acquire(image, CAM4, r0, p[2], cmap)
            self.assertTrue(base.ok, base.reason)
            for cm, offset, tolerance in ((cm_far, far, 1e-6), (cm_odd, 0.0, 0.05), (cm_datum, datum, 1e-6), (cm_datum0, datum, 0.05)):
                acq = acquire(image, CAM4, r0, p[2], cm)
                self.assertTrue(acq.ok, acq.reason)
                self.assertLess(float(np.abs(acq.fix.p[:2] - offset - base.fix.p[:2]).max()), tolerance)
                self.assertLess(abs(acq.fix.p[2] - base.fix.p[2]), tolerance)


FUZZ_CONFIGS = ('sites_zero', 'sites_short', 'candidates_zero', 'template_huge', 'search_negative',
                'yaw_step_zero', 'yaw_step_negative', 'decimate_zero', 'res2_zero', 'solver_bogus', 'text')
# Left out of the routine run only for time: each takes the full path and passed when run by hand.
FUZZ_SKIP = ('image=big', 'image=noise', 'image=negative', 'image=uint8', 'image=huge_values', 'image=boolean',
             'image=transposed', 'camera=rcb_double', 'r0=doubled', 'prior_radius=e12', 'yaw_prior=two_circles',
             'yaw_prior=negative_width')


def good_frames():
    """Three frames of one descent over the base world: (image, r0, height, pose)."""
    world, _ = base_world()
    rng = np.random.default_rng(41)
    p, r = random_pose(rng, reach=60.0)
    frames = []
    for i in range(3):
        pp, rr = p + i*np.array([4.0, -3.0, -8.0]), rz(0.3*i) @ r
        frames.append((render(world, CAM4, pp, rr, noise=1.0, rng=rng), levelled(rr, 1.0), pp[2], pp))
    return frames


class RobustnessTest(unittest.TestCase):
    def test_acquire_never_raises_and_never_fixes_wrongly_on_bad_arguments(self):
        """About 95 malformed calls (all 151 were run once by hand: none raised, the slowest took 14 s under load)."""
        _, cmap = base_world()
        image, r0, height, p = memo('frames', good_frames)[0]
        slowest = ('', 0.0)
        for label, args, kw in fuzz_cases(image, r0, height, cmap):
            if label in FUZZ_SKIP or (label.startswith('config=') and label[7:] not in FUZZ_CONFIGS):
                continue
            with np.errstate(all='ignore'):
                acq, error, seconds = call_guarded(acquire, *args, **kw)
            self.assertIsNone(error, '%s raised %r' % (label, error))
            self.assertIsInstance(acq.ok, (bool, np.bool_), label)
            self.assertIsInstance(acq.reason, str, label)
            slowest = max(slowest, (label, seconds), key=lambda item: item[1])
            if acq.ok:                                              # the scene is the right one, so a fix must be right
                self.assertTrue(np.all(np.isfinite(acq.fix.p)) and np.all(np.isfinite(acq.fix.cov)), label)
                self.assertLess(err_h(acq, p), 10.0, label)
            else:
                self.assertIsNone(acq.fix, label)
        self.assertLess(slowest[1], 60.0, slowest)

    def test_r0_that_is_not_a_rotation_is_refused(self):
        """FIXED in coarse.py on 11 October 2026; found as a defect: r0 is checked for shape and finiteness only. 2*r0 gives ok, with a SMALLER claimed sigma."""
        _, cmap = base_world()
        image, r0, height, _ = memo('frames', good_frames)[0]
        self.assertFalse(acquire(image, CAM4, 2*r0, height, cmap).ok)

    def test_cost_of_a_normal_call(self):
        """Peak traced memory about 24 MB; FFT count within what the configuration allows (measured 294 to 327)."""
        _, cmap = base_world()
        image, r0, height, _ = memo('frames', good_frames)[0]
        tracemalloc.start()
        try:
            acq, error, seconds = call_guarded(acquire, image, CAM4, r0, height, cmap)
            peak = tracemalloc.get_traced_memory()[1]
        finally:
            tracemalloc.stop()
        self.assertTrue(acq.ok)
        self.assertLess(peak, 60e6)
        self.assertLess(seconds, 60.0)
        c = cmap.cfg
        self.assertEqual(acq.ffts[256], 123)
        self.assertLessEqual(acq.ffts[128], 3*(c.candidates*c.sites[0]*c.sites[1] + c.polish_sites[0]*c.polish_sites[1]))

    def test_work_is_bounded_whatever_the_yaw_prior(self):
        """FIXED in coarse.py on 11 October 2026; found as a defect: the number of yaw hypotheses grows with the prior's width without limit.

        A half-width of 4 pi gives 241 hypotheses instead of 60 (485 FFTs of 256, +50 MB); a width of
        50 rad would give 955 and a 250 MB score volume. Suggested fix: clamp the half-width to pi
        (then use the full-circle list) before building `yaws`.
        """
        _, cmap = base_world()
        image, r0, height, _ = memo('frames', good_frames)[0]
        acq = acquire(image, CAM4, r0, height, cmap, yaw_prior=(1.0, 4*math.pi))
        self.assertLessEqual(acq.ffts[256], 123)

    def test_acquirer_recovers_after_a_refused_image(self):
        _, cmap = base_world()
        frames = memo('frames', good_frames)
        acquirer = Acquirer(cmap)
        self.assertEqual(acquirer.update(0.0, *frames[0][:1], CAM4, *frames[0][1:3])[0], 'pending')
        self.assertEqual(acquirer.update(0.5, np.full((272, 364), math.nan), CAM4, frames[0][1], 300.0)[0], 'none')
        self.assertEqual(acquirer.update(0.6, None, None, None, None)[0], 'none')
        status, acq, velocity = acquirer.update(1.0, frames[1][0], CAM4, frames[1][1], frames[1][2])
        self.assertEqual(status, 'confirmed')
        self.assertLess(err_h(acq, frames[1][3]), 2.0)
        self.assertLess(float(np.abs(velocity - np.array([4.0, -3.0, -8.0])).max()), 1.5)

    def test_acquirer_survives_a_missing_timestamp(self):
        """FIXED in coarse.py on 11 October 2026; found as a defect: Acquirer.update raises TypeError on t=None once a fix is pending, and keeps raising.

        The bad t is stored in self.pending before it is used, so every later refused image raises
        too (t - self.pending[0]) until another fix arrives. Suggested fix: reject a non-finite or
        non-numeric t at the top of update (return 'none'), before touching self.pending.
        """
        _, cmap = base_world()
        frames = memo('frames', good_frames)
        acquirer = Acquirer(cmap)
        acquirer.update(0.0, frames[0][0], CAM4, frames[0][1], frames[0][2])
        raised = 0
        for t, image in ((None, frames[1][0]), (2.0, np.full((272, 364), math.nan)), (3.0, np.full((272, 364), math.nan))):
            raised += call_guarded(acquirer.update, t, image, CAM4, frames[1][1], frames[1][2])[1] is not None
        self.assertEqual(raised, 0)

    def test_acquirer_does_not_confirm_a_fix_with_the_same_image(self):
        """FIXED in coarse.py on 11 October 2026; found as a defect: the identical frame delivered twice (1 microsecond apart) is 'confirmed'.

        Nothing requires the two images to be different evidence: no least time gap, rotation or
        movement. Suggested fix: require dt above a floor (say 0.2 s) before a pair may confirm.
        """
        _, cmap = base_world()
        image, r0, height, _ = memo('frames', good_frames)[0]
        acquirer = Acquirer(cmap)
        acquirer.update(0.0, image, CAM4, r0, height)
        self.assertNotEqual(acquirer.update(1e-6, image, CAM4, r0, height)[0], 'confirmed')


UP_FORCE = np.array([0.0, 0.0, -G])       # specific force on a level can, body axes (z down)


def leveller_after(dt, accel, gyro, tau=2.0):
    """Leveller given 0.5 s of good samples, one odd sample, then 10 s of good ones: (raised, tilt deg or None)."""
    lv = Leveller(tau)
    for _ in range(50):
        lv.update(0.01, UP_FORCE, np.zeros(3))
    raised = call_guarded(lv.update, dt, accel, gyro)[1] is not None
    for _ in range(1000):
        lv.update(0.01, UP_FORCE, np.zeros(3))
    r0 = lv.attitude()
    if r0 is None or not np.all(np.isfinite(r0)):
        return raised, None
    return raised, math.acos(max(-1.0, min(1.0, -r0[2, 2])))/DEG


class LevellerTest(unittest.TestCase):
    def test_non_finite_samples_are_ignored(self):
        z = np.zeros(3)
        for dt, accel, gyro in ((0.0, UP_FORCE, z), (-1.0, UP_FORCE, z), (math.nan, UP_FORCE, z), (0.01, None, z),
                                (0.01, UP_FORCE*math.nan, z), (0.01, UP_FORCE*math.inf, z), (0.01, UP_FORCE, None),
                                (0.01, UP_FORCE, z*math.nan), (0.01, z, z), (1e-300, UP_FORCE, z)):
            raised, tilt = leveller_after(dt, accel, gyro)
            self.assertFalse(raised)
            self.assertLess(tilt, 0.01)

    def test_update_never_raises(self):
        """FIXED in coarse.py on 11 October 2026; found as a defect: Leveller.update raises on a wrong-shaped sample, dt=None, or a finite but absurd gyro rate.

        ValueError for accel of length 2 or 4, IndexError for gyro of length 2, TypeError for dt=None,
        ValueError ('math domain error', from exp_so3) for gyro = 1e200. Suggested fix: validate shapes
        (3,) and magnitudes at the top and wrap the body as acquire does.
        """
        z = np.zeros(3)
        for dt, accel, gyro in ((None, UP_FORCE, z), (0.01, np.zeros(2), z), (0.01, UP_FORCE, np.zeros(2)),
                                (0.01, UP_FORCE, np.full(3, 1e200)), (1e300, UP_FORCE, np.array([0.1, 0.2, 0.3]))):
            self.assertFalse(leveller_after(dt, accel, gyro)[0])

    def test_one_absurd_sample_does_not_disable_levelling_for_good(self):
        """FIXED in coarse.py on 11 October 2026; found as a defect: one sample with dt=inf, or one finite accel of 1e200, leaves attitude() NaN or None.

        dt=inf passes `dt > 0`; exp_so3(gyro*inf) is NaN and self.rg stays NaN for ever (attitude()
        then returns a NaN matrix, which acquire refuses as 'bad input' on every image). accel=1e200
        is finite, so it enters the mean; force becomes inf, then NaN. Ten seconds of good samples do
        not recover either. Suggested fix: accept only 0 < dt < 1 s, |accel| < ~300 m/s^2 and
        |gyro| < ~50 rad/s, and reset the state if it ever becomes non-finite.
        """
        z = np.zeros(3)
        for dt, accel, gyro in ((math.inf, UP_FORCE, z), (0.01, UP_FORCE*1e200, z)):
            tilt = leveller_after(dt, accel, gyro)[1]
            self.assertIsNotNone(tilt)
            self.assertLess(tilt, 1.0)

    def test_static_hang_and_integrator_sanity(self):
        accel, gyro, att = pendulum_imu(12.0, swing_deg=(0.0, 0.0))
        self.assertLess(float(np.abs(accel - UP_FORCE).max()), 1e-9)
        self.assertLess(float(leveller_tilt_error(accel, gyro, att).max()), 1e-6)
        accel, gyro, att = pendulum_imu(4.0, swing_deg=(20.0, 0.0))
        # A can on a line feels only the line: no lateral specific force in body axes however it swings.
        self.assertLess(float(np.abs(accel[:, :2]).max()), 0.02)

    def test_swing_leaves_a_residual_of_about_a_sixth_of_its_amplitude(self):
        """A 20 degree planar swing of 2 s period: Leveller error 2.2 deg rms, 3.2 deg peak (30 s run by hand).

        The exponential mean (tau = 2 s) is a first-order low-pass, gain 1/sqrt(1 + (omega tau)^2) =
        0.16 at this period; it does not average over whole swing periods as the docstring suggests.
        """
        accel, gyro, att = pendulum_imu(10.0, swing_deg=(20.0, 0.0), sub=3)
        error = leveller_tilt_error(accel, gyro, att, settle=6.0)
        rms = math.sqrt(float((error*error).mean()))
        self.assertGreater(rms, 1.2)
        self.assertLess(rms, 3.5)

    def test_steady_turn_is_not_levelled(self):
        """LIMITATION, not a coding error: in a spiral glide the mean specific force does not point up.

        Canopy circling at 5 m/s on a 15 m radius (centripetal 1.67 m/s^2, hanging angle 9.7 deg):
        the Leveller calls the hanging direction 'down', error 8.2 deg mean over 20 s by hand. Any
        sustained horizontal acceleration a gives atan(a/g) of false tilt, larger than the 8 deg
        max_tilt_disagreement_deg gate can then check against.
        """
        spiral = lambda t: 1.67*np.array([-math.cos(t/3.0), -math.sin(t/3.0), 0.0])     # noqa: E731
        accel, gyro, att = pendulum_imu(12.0, swing_deg=(0.0, 0.0), pivot_accel=spiral, sub=3)
        error = leveller_tilt_error(accel, gyro, att, settle=6.0)
        self.assertGreater(float(error.mean()), 5.0)
        self.assertLess(float(error.max()), 12.0)


def duplicate_single(seed, case='duplicate'):
    """One image over the duplicated map: (Acquisition, true position, true yaw hypothesis)."""
    world, cmap, centre, reach = adversarial_setup(case)
    rng = np.random.default_rng([78, seed])
    p, r = random_pose(rng, reach=reach, centre=centre)
    psi = rng.uniform(-math.pi, math.pi)
    image = render(world, CAM4, p, r, noise=1.0, rng=rng)
    return acquire(image, CAM4, levelled(r, psi), p[2], cmap), p, psi


class AdversarialTerrainTest(unittest.TestCase):
    """A handful of trials each; the table in the module docstring comes from 6 pairs (12 images) per case."""

    def trial(self, case, seed):
        return memo('trial:%s:%d' % (case, seed), adversarial_trial, case, seed)

    def test_map_with_a_duplicated_tile_gives_no_false_fix(self):
        """FIXED in coarse.py on 11 October 2026; found as a defect: two equally good places on the map -> a confident fix at one of them.

        5 of 20 single images fixed on the copy, 520 m from the truth, each with a claimed horizontal
        sigma of about 3 m and reason 'ok'. Nothing in acquire compares the best stage-1 peak with a
        rival: Acquisition.lead is computed and never tested, and it cannot see this rival anyway
        (next test). The right answer is a refusal.
        """
        false = sum(sum(self.trial('duplicate', seed)['false']) for seed in (2, 3))
        self.assertEqual(false, 0)

    def test_two_image_confirmation_rejects_the_duplicate(self):
        """FIXED in coarse.py on 11 October 2026; found as a defect: Acquirer confirmed a pair of fixes that were both on the copy (seed 3; 1 of 6 pairs).

        Both images see the same terrain, so they can make the same mistake, and two fixes at the same
        wrong place agree through the gyro perfectly. Two-image agreement is not evidence against an
        alias of the map.
        """
        self.assertFalse(self.trial('duplicate', 3)['confirmed_false'])

    def test_lead_reports_a_rival_peak_at_the_same_heading(self):
        """FIXED in coarse.py on 11 October 2026; found as a defect: stage 1 keeps one peak per yaw hypothesis, so a second place at the SAME heading is invisible.

        `best` holds the argmax of each yaw slice only. With the fix on the copy (seed 3) the true
        place, which correlates equally well at the same yaw, is in no candidate, and lead is 0.57,
        the same as on an unambiguous map. Suggested fix: after taking the best peak, blank its
        neighbourhood (k/2 cells, 2.5 yaw steps) in `volume` and take the maximum of what is left, for
        every candidate in turn; then refuse when the lead is below a margin set from the Monte Carlo.
        """
        acq, p, psi = duplicate_single(3)
        distances = [math.hypot(c[1] - p[0], c[2] - p[1]) for c in acq.candidates]
        self.assertTrue(any(d < 30.0 for d in distances), distances)                       # the true place
        self.assertTrue(any(abs(d - DUP_SHIFT) < 30.0 for d in distances), distances)      # and the copy
        self.assertLess(acq.lead, 0.2)
        self.assertFalse(acq.ok)
        self.assertTrue(acq.reason.startswith('ambiguous'), acq.reason)

    @unittest.expectedFailure
    def test_a_rival_outside_the_searched_area_cannot_be_seen(self):
        """LIMITATION, not fixed: at the edge of the map window the true place may not be searchable.

        Stage 1 can only place its disc where the whole disc lies inside the map window, so with the
        default 1024 m window and a 200 m disc the disc centre must be within 312 m of the map centre,
        and the can, whose disc is offset from it when it is tilted, within about 250 m. Here the can
        is 263 m out and tilted: the true place is not searchable, only the copy is, and the fix lands
        on the copy, 520 m out, with reason 'ok'. The correct behaviour (asserted here) is no false
        fix; what makes that possible is a map window sized for the prior (n1 = 320 for 300 m), which
        is what the other duplicate tests use.
        """
        acq, p, psi = duplicate_single(3, 'duplicate_edge')
        self.assertFalse(acq.ok and err_h(acq, p) > 10.0)

    def test_scenes_with_no_right_answer_are_refused(self):
        """Mirror image of the map, a scene from elsewhere, and bare parallel lines: 0 fixes in 36 images by hand."""
        for case in ('mirror', 'offmap', 'periodic'):
            trial = self.trial(case, 0)
            self.assertEqual(sum(trial['ok']), 0, (case, trial['reason']))

    def test_degraded_inputs_give_a_true_fix_or_none(self):
        """Half the image blank, r0 tilted 8 degrees: no false fix (height errors and more seeds by hand)."""
        for case in ('half_blank', 'tilt8'):
            trial = self.trial(case, 0)
            self.assertEqual(sum(trial['false']), 0, (case, trial['err']))


if __name__ == '__main__':
    unittest.main(verbosity=2, warnings='ignore')
