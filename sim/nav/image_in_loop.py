"""GPL-3.0-only. SYNTHETIC closed-loop experiment: rendered images, real template matching, the flight filter.

Usage: python sim/nav/image_in_loop.py [--descents N] [--budget-min M] [--binning B] [--only a,b] [--quick]
Writes sim/nav/results/image_in_loop.{json,csv}. Only NumPy is required. One process.

sim/nav/run_nav_mc.py generates landmark pixels from geometry plus an assumed error model (0.3 px floor plus
0.15 m on the ground, a few percent uniform outliers). Here the measurement is produced the way the flight
code would produce it. For every 5 Hz frame above 80 m:

  1. an image is rendered at the TRUE camera pose (render_frame: the ray-cast of sim/maths/feasibility.render_image, 2 grey
     levels of noise); the camera centre is the IMU position plus R @ lever, camera-to-world rotation R;
  2. map landmarks are CHOSEN from the filter's PREDICTED pose (a jittered 40 m grid, one per image cell,
     only those whose whole 48 m search patch is predicted to lie inside the image);
  3. each landmark's search patch is rectified with the filter's predicted exposure-time pose onto a 0.5 m
     grid, the 32 x 32 map template (terrain sampled at the map location) is located by normalised
     cross-correlation (sim/maths/models.ncc_fft) within +/-16 m, the existing gates are applied (score
     > 0.5, interior concave quadratic peak, peak gap > 0.04) and the peak is converted back to an ORIGINAL
     image pixel through the same predicted-pose mapping (as sim/nav/ncc_attitude_tolerance.py);
  4. (map point, pixel) pairs go to Eskf.update_landmarks with the Monte Carlo's pixel-variance formula.

Truth is used only to score: each matched pixel is compared with the true pixel of that landmark. A match
is FALSE when its ground-equivalent error exceeds 1.5 map pixels (0.75 m), the criterion of
ncc_attitude_tolerance.py. The paired control re-runs the same descents (same truth, IMU errors, baro,
initial error) with geometry-generated pixels: true pixel plus the Monte Carlo's assumed Gaussian noise, no
outliers, no matcher. 'control' offers the same landmark set rule as the image run (predicted in-view patch);
'control_mc' uses the Monte Carlo's own selection from true pixels (6 x 5 cells, 16 px margin, up to 30), and
for the same descent index reproduces run_nav_mc.run_descent exactly (checked in test_image_in_loop.py).

Everything here is synthetic: terrain is a sum of six sinusoids, not farmland; flat ground unless a variant
says otherwise; no lens distortion, blur, photometric change or map datum error; the unmodelled calibration,
map-position and relief errors of the Monte Carlo's default Sensors are switched OFF so that the only
difference between image and control is the measurement. Timings are host timings on a shared, loaded PC.
"""
import os

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_name, '1')           # one core, like the flight computer; set before NumPy loads

import argparse                                 # noqa: E402
import csv                                      # noqa: E402
import json                                     # noqa: E402
import math                                     # noqa: E402
import platform                                 # noqa: E402
import sys                                      # noqa: E402
import time                                     # noqa: E402
from dataclasses import asdict, dataclass, replace   # noqa: E402
from pathlib import Path                        # noqa: E402

import numpy as np                              # noqa: E402

HERE = Path(__file__).resolve().parent
sys.dont_write_bytecode = True                  # importing must not add .pyc files next to other people's code
sys.path.insert(0, str(HERE))
import ncc_attitude_tolerance as nat            # noqa: E402  (also puts sim/maths on sys.path)
import run_nav_mc as mc                         # noqa: E402  (also puts code/ on sys.path and imports truth)
import truth as tr                              # noqa: E402
from cansat.eskf import ATT, LEVEL, N_ERR, POS, VEL, Eskf, ImuNoise, exp_so3   # noqa: E402
from feasibility import quadratic_peak, surface_height, terrain   # noqa: E402
from models import ncc_fft                      # noqa: E402

OUT = HERE/'results'
DATE = '2026-10-09'
SEED = mc.SEED
DEG = mc.DEG
FULL_SIZE = (mc.IMAGE[1], mc.IMAGE[0])          # (rows, cols) of the sensor
FOCAL = mc.FOCAL                                # full-resolution focal length, px
LEVER = np.array(mc.CAMERA_LEVER)
FRAME_DT = mc.FRAME_DT
NAV_DT = mc.NAV_DT
NOISE_GRAY = 2.0                                # grey levels, white, per rendered pixel
# Fine matcher (ncc_attitude_tolerance.CONFIGS['FINE']): 32 px template, 0.5 m/px map grid, +/-16 m search.
TEMPLATE_PX, SCALE_M, SEARCH_M = 32, 0.5, 16.0
HALF_PX = int(round(SEARCH_M/SCALE_M))
BIG_PX = TEMPLATE_PX + 2*HALF_PX
PATCH_HALF_M = (BIG_PX - 1)/2*SCALE_M           # centre of the search patch to its outermost sample
FALSE_M = nat.CORRECT_MAP_PX*SCALE_M            # a match is false beyond 1.5 map pixels on the ground
CORNERS = np.array([(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)])*PATCH_HALF_M
HEIGHT_EDGES = (300.0, 250.0, 200.0, 150.0, 100.0, 80.0)
FIRST_SETTLED = 10                              # frames, as the Monte Carlo scoring (2 s after the first fix)
LOST_M = 10.0                                   # the Monte Carlo's T1 threshold
LANDMARK_COLS = ('frame', 'height_m', 'depth_m', 'score', 'gap', 'peak_ok', 'matcher_accepted', 'filter_accepted',
                 'err_x_px', 'err_y_px', 'err_ground_m', 'sigma_assumed_px', 'correct')

LIMITS = [
    'SYNTHETIC. The terrain is feasibility.terrain, a sum of six sinusoids with 3.4-17 m wavelengths and a '
    'quasi-periodic lattice, not farmland. Its correlation length sets the matcher\'s precision and its false-match '
    'behaviour; the numbers here do not transfer to real imagery.',
    'Images are rendered at 1/binning of the sensor resolution (default 2: 728 x 544, f = 362.3 px) and point '
    'sampled with bilinear interpolation onto the 0.5 m map grid (no anti-alias filter, no blur, no exposure '
    'change). Half resolution is coarser than the 0.5 m grid above ~180 m and the white image noise is not averaged '
    'by the resampling. The 2-descent full-resolution subset shows the effect is negligible here (rms pixel error '
    '0.016-0.053 px against 0.020-0.048 px at half resolution), but it was not run on the lost-lock descent.',
    'The one descent that lost lock (first-fix draw of descent 2: 9.6 m, 3.0 deg tilt, 2.2 deg yaw, i.e. 2.3-2.7 sigma on '
    'three components) locked onto a near-exact lattice repeat of the procedural texture: the template correlation at '
    'the lock offset (-8.9, -24.3) m is 0.92 (every location above the 0.5 gate) and the texture autocorrelation peaks at '
    '(-9, -25) m with 0.93. Real terrain has its own repeat structure (crop rows, parcels) that is not represented, so '
    'the lock-loss rate here (1 of 6 first-fix draws) is a property of this texture and of six draws, not an estimate.',
    'The filter is conservative with the unmodelled map, relief and calibration errors switched off (NEES of the '
    'horizontal position about 0.3-0.8 instead of 2, in image and control alike), and its map-error inflation dominates '
    'its pixel noise. Position error is therefore nearly insensitive to the matcher\'s noise: image and control agree to '
    'a few percent on locked descents, so position error cannot validate the pixel-noise model; only the direct '
    'measurement of match error can.',
    'Pinhole camera with the exact intrinsics the filter assumes: no calibration error (focal, principal point, '
    'misalignment), no lens distortion, no map datum or per-landmark map error. The flat-ground runs have no relief '
    'error either. The Monte Carlo\'s default unmodelled errors for these are switched off in image and control alike.',
    'No coarse acquisition: the first fix is an initial error drawn from the filter\'s own first-fix sigmas '
    '(3 m, 1-2 deg). The matcher only works inside its +/-16 m window, so this is a tracking test.',
    'A handful of descents per variant; confidence intervals are wide and the variants share truth with the base run.',
    'Landmarks come from a fixed jittered 40 m grid of map points chosen from the PREDICTED pose, not from '
    'pre-selected high-contrast map features. Requiring the whole 48 m search patch inside the image limits the '
    'landmarks near 80 m to a few in the middle of the frame.',
    'Match errors within a frame share one pose error and are not independent; the filter treats them as independent.',
    'Host timings come from a PC shared with a running Monte Carlo and throttling; they are not Raspberry Pi timings.',
]

SENSORS = replace(mc.Sensors(), outlier_rate=0.0, map_xy_sd=0.0, relief_sd=0.0, focal_error=0.0,
                  principal_error=0.0, misalignment=0.0, datum=(0.0, 0.0))


# --- Renderer --------------------------------------------------------------------

def render_frame(pos, r, size, focal, relief=0.0, iterations=None):
    """Radiance image of the procedural terrain: feasibility.render_image computed faster.

    Same camera model (principal point at the image centre, pixel index = continuous coordinate, r maps
    camera to world) and the same ray / height-field fixed-point intersection, with the ray directions built
    by broadcasting and the sinusoids evaluated in float32 (position error below 1e-4 m, radiance below 0.01
    grey levels; checked against render_image in test_image_in_loop.py). Rendering is not flight code.
    iterations: fixed-point passes for relief (default 4; render_image uses 8, the difference is far below 0.1 grey).
    """
    rows, cols = size
    u = (np.arange(cols) - cols/2)/focal
    v = (np.arange(rows) - rows/2)/focal
    ray = [r[i, 0]*u[None, :] + (r[i, 1]*v + r[i, 2])[:, None] for i in range(3)]
    distance = -pos[2]/ray[2]
    for _ in range((4 if iterations is None else iterations) if relief else 0):
        x = (pos[0] + distance*ray[0]).astype(np.float32)
        y = (pos[1] + distance*ray[1]).astype(np.float32)
        distance = (surface_height(x, y, relief) - pos[2])/ray[2]
    x = (pos[0] + distance*ray[0]).astype(np.float32)
    y = (pos[1] + distance*ray[1]).astype(np.float32)
    return terrain(x, y)


# --- Matcher ---------------------------------------------------------------------

def match_landmark(image, xy, cam_pos, cam_r, focal):
    """Locate the map point xy (east, north, z = 0 plane) in an image using ONLY the predicted camera pose.

    cam_pos is the camera centre, cam_r the camera-to-world rotation, focal in image pixels. Returns the
    match, its original-image pixel and the times of the two stages (rectify, correlate); the logic is
    ncc_attitude_tolerance.match_landmark's 'map' layout with the stages timed separately.
    """
    t0 = time.perf_counter()
    search = nat.rectify(image, xy, cam_pos, cam_r, BIG_PX, SCALE_M, focal)
    if not np.all(np.isfinite(search)):
        return {'in_view': False, 'accepted': False, 't_rectify': time.perf_counter() - t0, 't_correlate': 0.0}
    template = nat.map_patch(xy, TEMPLATE_PX, SCALE_M)
    t1 = time.perf_counter()
    scores = ncc_fft(search, template)
    iy, ix = np.unravel_index(np.nanargmax(scores), scores.shape)
    other = scores.copy()
    g = nat.GAP_EXCLUSION_PX
    other[max(0, iy - g):iy + g + 1, max(0, ix - g):ix + g + 1] = np.nan
    score = float(scores[iy, ix])
    gap = float(score - np.nanmax(other))
    peak = quadratic_peak(scores)
    located = np.array([ix, iy], dtype=float) if peak is None else peak
    shift = (located - HALF_PX)*SCALE_M
    uv = nat.ground_to_pixel(np.asarray(xy, dtype=float)[:2] + shift, cam_pos, cam_r, focal, image.shape)[0]
    t2 = time.perf_counter()
    return {'in_view': True, 'score': score, 'gap': gap, 'peak_ok': peak is not None, 'shift': shift, 'uv': uv,
            'accepted': bool(peak is not None and score > nat.SCORE_GATE and gap > nat.GAP_GATE),
            't_rectify': t1 - t0, 't_correlate': t2 - t1}


def patch_in_view(camera, map_xy, p, r, size, binning, margin=1.0):
    """True where all four corners of a landmark's search patch project inside the image (predicted pose).

    camera is the full-resolution eskf.Camera, (p, r) the IMU pose; size the (rows, cols) of the image
    actually used, binning its reduction relative to the sensor. The projected patch is convex, so its
    corners suffice; bilinear sampling needs 0 <= x < cols - 1.
    """
    n = len(map_xy)
    pts = np.concatenate((map_xy[:, None, :2] + CORNERS[None], np.zeros((n, 4, 1))), axis=2).reshape(-1, 3)
    pix, depth = camera.project(pts, p, r)
    pix = pix.reshape(n, 4, 2)/binning
    depth = depth.reshape(n, 4)
    return ((depth > 1.0).all(axis=1) & (pix[..., 0] >= margin).all(axis=1) & (pix[..., 0] <= size[1] - 2 - margin).all(axis=1)
            & (pix[..., 1] >= margin).all(axis=1) & (pix[..., 1] <= size[0] - 2 - margin).all(axis=1))


def select_predicted(camera, map_points, p, r, size, binning, limit, cells=(6, 5)):
    """Landmark indices chosen from the filter's PREDICTED pixels: patch wholly in view, one per image cell.

    Reuses the Monte Carlo's cell rule (run_nav_mc.select_landmarks) on the candidates that pass the patch test.
    Returns (chosen indices, number of in-view candidates).
    """
    ok = np.flatnonzero(patch_in_view(camera, map_points[:, :2], p, r, size, binning))
    if len(ok) == 0:
        return ok, 0
    pix, depth = camera.project(map_points[ok], p, r)
    return ok[mc.select_landmarks(pix, depth, limit, 1.0, cells, margin=0.0)], len(ok)


# --- One descent -----------------------------------------------------------------

@dataclass
class Setup:
    canopy: str = 'moderate'
    mode: str = 'image'              # image | control | control_mc
    binning: int = 2                 # image rendered at 1/binning of 1456 x 1088
    relief: float = 0.0              # m amplitude of the rendered surface; the matcher and the map assume flat
    yaw0_deg: float = 0.0            # if nonzero, the initial yaw error is +/- this (sign drawn per descent)
    noise_gray: float = NOISE_GRAY
    max_landmarks: int = 30
    sensors: object = None           # mc.Sensors; None means SENSORS
    cfg: object = None               # mc.FilterCfg; None means the Monte Carlo default
    stop_height: float = 78.0        # stop once the truth is below this: no imagery is used there
    max_frames: int = 0              # if > 0, stop after this many frames (tests)

    def __post_init__(self):
        self.sensors = SENSORS if self.sensors is None else self.sensors
        self.cfg = mc.FilterCfg() if self.cfg is None else self.cfg


def load_truth(canopy, count, source='auto', duration=23.0):
    """Truth for descents 0..count-1: the Monte Carlo's cached batch if it exists (so descent j here has the
    same truth, IMU and baro errors and first-fix error as run_nav_mc's descent j), else a fresh simulation."""
    directory = mc.truth_dir(canopy, 160)
    if source in ('auto', 'cache') and (directory/'done').exists():
        axis = {'p': 1, 'v': 1, 'r': 1, 'accel': 1, 'gyro': 1, 'delay_p': 2, 'delay_r': 2, 'swing': 1}
        data = {}
        for key, ax in axis.items():
            array = np.load(directory/f'{key}.npy', mmap_mode='r')
            data[key] = np.array(np.take(array, np.arange(count), axis=ax))
        data['source'] = 'cache:' + directory.name
        return data
    if source == 'cache':
        raise FileNotFoundError(directory)
    rng = np.random.default_rng([SEED, 71, sorted(mc.CANOPIES).index(canopy)])
    data = tr.simulate(mc.CANOPIES[canopy], count, rng, duration=duration, nav_dt=NAV_DT, frame_dt=FRAME_DT,
                       delays=mc.DELAYS)
    data = {k: data[k] for k in ('p', 'v', 'r', 'accel', 'gyro', 'delay_p', 'delay_r', 'swing')}
    data['source'] = f'fresh simulation, {count} descents, seed [{SEED}, 71, canopy index]'
    return data


def prepare(truth, j, setup):
    """Everything one descent needs, wired exactly as run_nav_mc.run_descent (same generators, same order)."""
    s, c = setup.sensors, setup.cfg
    steps = truth['accel'].shape[0]
    t_p, t_v, t_r = (np.asarray(truth[k][:, j]) for k in ('p', 'v', 'r'))
    accel_true, gyro_true = np.asarray(truth['accel'][:, j]), np.asarray(truth['gyro'][:, j])
    frames = truth['delay_p'].shape[1]
    r_imu, r_map, r_init, r_pix = (np.random.default_rng([SEED, j, k]) for k in range(4))
    choices = s.camera_jitter or (s.camera_delay,)
    d_idx = np.array([mc.DELAYS.index(d) for d in choices])[np.random.default_rng([SEED, j, 4]).integers(len(choices), size=frames)]
    cam_p = np.asarray(truth['delay_p'][:, :, j])[d_idx, np.arange(frames)]
    cam_r = np.asarray(truth['delay_r'][:, :, j])[d_idx, np.arange(frames)]
    true_delay = np.array(mc.DELAYS)[d_idx]
    sq = math.sqrt(NAV_DT)
    ba = s.accel_bias_sd*r_imu.normal(size=3) + np.cumsum(s.accel_bias_walk*sq*r_imu.normal(size=(steps, 3)), axis=0)
    bg = s.gyro_bias_sd*r_imu.normal(size=3) + np.cumsum(s.gyro_bias_walk*sq*r_imu.normal(size=(steps, 3)), axis=0)
    scale_a, scale_g = 1 + s.accel_scale_sd*r_imu.normal(size=3), 1 + s.gyro_scale_sd*r_imu.normal(size=3)
    accel = scale_a*accel_true + ba + s.accel_noise/sq*r_imu.normal(size=(steps, 3))
    gyro = scale_g*gyro_true + bg + s.gyro_noise/sq*r_imu.normal(size=(steps, 3))
    cross = s.gyro_cross_sd*np.random.default_rng([SEED, j, 5]).normal(size=(3, 3))
    np.fill_diagonal(cross, 0.0)
    gyro = gyro + gyro_true @ cross.T
    baro_bias = s.baro_bias_sd*r_imu.normal() + np.cumsum(s.baro_bias_walk*sq*r_imu.normal(size=steps))
    baro_noise = s.baro_noise*r_imu.normal(size=steps)
    # Map landmarks: a jittered grid around the ground track (the map is exact: the world has the map's points).
    reach = 1.3*t_p[0, 2]
    lo, hi = t_p[:, :2].min(axis=0) - reach, t_p[:, :2].max(axis=0) + reach
    gx, gy = np.meshgrid(np.arange(lo[0], hi[0], s.landmark_spacing), np.arange(lo[1], hi[1], s.landmark_spacing))
    grid = np.column_stack((gx.ravel(), gy.ravel()))
    grid = grid + r_map.uniform(-0.4, 0.4, grid.shape)*s.landmark_spacing
    map_points = np.column_stack((grid, np.zeros(len(grid))))
    world_points = map_points + r_map.normal(size=(len(grid), 3))*np.array([s.map_xy_sd, s.map_xy_sd, s.relief_sd])
    world_points[:, :2] += np.array(s.datum)
    if setup.relief:
        # The rendered surface is z = surface_height; the landmark is where the image shows the template.
        world_points[:, 2] += surface_height(world_points[:, 0], world_points[:, 1], setup.relief)
    cam = mc.nominal_camera()
    sd = np.r_[c.init_pos, c.init_vel, c.init_att, np.full(3, c.init_accel_bias), np.full(3, c.init_gyro_bias),
               c.init_baro_bias, c.init_time_offset]
    e0 = sd*np.r_[r_init.normal(size=16), 0.0]
    if setup.yaw0_deg:
        sign = np.random.default_rng([SEED, j, 7]).choice([-1.0, 1.0])
        e0[8] = sign*setup.yaw0_deg*DEG                          # yaw is the third attitude-error component
    noise = ImuNoise(c.accel_noise*c.noise_scale, c.gyro_noise*c.noise_scale, c.accel_bias_walk,
                     c.gyro_bias_walk, c.baro_bias_walk, gyro_scale=c.gyro_scale, scale_tau=FRAME_DT)
    f = Eskf(t_p[0] - e0[POS], t_v[0] - e0[VEL], t_r[0] @ exp_so3(-e0[ATT]), np.diag(sd**2), noise)
    f.rate = gyro[0].copy()
    return {'t_p': t_p, 't_v': t_v, 't_r': t_r, 'accel': accel, 'gyro': gyro, 'ba': ba, 'bg': bg,
            'baro_bias': baro_bias, 'baro_noise': baro_noise, 'cam_p': cam_p, 'cam_r': cam_r,
            'true_delay': true_delay, 'map_points': map_points, 'world_points': world_points, 'cam': cam,
            'f': f, 'frames': frames, 'steps': steps, 'r_pix': r_pix, 'e0': e0,
            'r_image': np.random.default_rng([SEED, j, 6]), 'swing': np.asarray(truth['swing'][:, j])}


def last_frame(d, stop_height):
    """Index of the last frame whose truth height is at least stop_height."""
    h = d['t_p'][np.arange(d['frames'])*round(FRAME_DT/NAV_DT), 2]
    return int(np.flatnonzero(h >= stop_height)[-1])


def run_descent(truth, j, setup):
    """Fly one descent with the flight filter and return per-frame records and per-landmark rows."""
    d = prepare(truth, j, setup)
    s, c = setup.sensors, setup.cfg
    f, cam = d['f'], d['cam']
    t_p, t_v, t_r = d['t_p'], d['t_v'], d['t_r']
    binning = setup.binning
    size = (FULL_SIZE[0]//binning, FULL_SIZE[1]//binning)
    focal_img = FOCAL/binning
    map_var = None if c.map_var is None else np.array(c.map_var)*c.map_inflation
    base_every = round(FRAME_DT/NAV_DT)
    n_frames = last_frame(d, setup.stop_height) + 1
    if setup.max_frames:
        n_frames = min(n_frames, setup.max_frames)
    k_end = (n_frames - 1)*base_every
    rec = {k: np.full((n_frames,) + shape, np.nan) for k, shape in (
        ('err', (N_ERR,)), ('sig', (N_ERR,)), ('nees', (3,)), ('counts', (8,)), ('times', (4,)),
        ('height', ()), ('pre', (3,)), ('candidates', ()))}
    rows = []
    started = time.perf_counter()
    for k in range(k_end + 1):
        fi = k//base_every
        if k % base_every == 0:
            counts, times = np.zeros(8), np.zeros(4)     # attempted, matcher ok, filter ok, false in, false kept, correct dropped, recovered, in view
            e_pre = f.error_from(t_p[k], t_v[k], t_r[k])
            rec['pre'][fi] = [math.hypot(e_pre[0], e_pre[1]), math.hypot(e_pre[6], e_pre[7])/DEG, abs(e_pre[8])/DEG]
            if f.p[2] >= c.map_floor:
                p_exp = f.p + f.v*f.time_offset                       # the exposure-instant pose the filter predicts
                r_exp = f.r @ exp_so3(f.rate*f.time_offset)
                if setup.mode == 'control_mc':
                    pix_all, depth_all = cam.project(d['world_points'], d['cam_p'][fi], d['cam_r'][fi])
                    chosen = mc.select_landmarks(pix_all, depth_all, s.max_landmarks, s.fov_fraction)
                    candidates = len(chosen)
                else:
                    chosen, candidates = select_predicted(cam, d['map_points'], p_exp, r_exp, size, binning,
                                                          setup.max_landmarks)
                rec['candidates'][fi] = candidates
                counts[7] = candidates
                if len(chosen):
                    m = len(chosen)
                    pix_true, z_true = cam.project(d['world_points'][chosen], d['cam_p'][fi], d['cam_r'][fi])
                    if setup.mode == 'image':
                        t0 = time.perf_counter()
                        centre = d['cam_p'][fi] + d['cam_r'][fi] @ LEVER      # TRUE camera centre
                        image = render_frame(centre, d['cam_r'][fi], size, focal_img, setup.relief)
                        if setup.noise_gray:
                            image += setup.noise_gray*d['r_image'].standard_normal(size, dtype=np.float32)
                        times[0] = time.perf_counter() - t0
                        pred_pos = p_exp + r_exp @ LEVER                      # PREDICTED camera pose
                        found = [match_landmark(image, d['map_points'][i, :2], pred_pos, r_exp, focal_img) for i in chosen]
                        times[1] = sum(x['t_rectify'] for x in found)
                        times[2] = sum(x['t_correlate'] for x in found)
                        in_view = np.array([x['in_view'] for x in found])
                        matched = np.array([x['accepted'] for x in found])
                        pixels = np.full((m, 2), np.nan)
                        score, gap, peak_ok = np.full(m, np.nan), np.full(m, np.nan), np.zeros(m, dtype=bool)
                        for q, x in enumerate(found):
                            if x['in_view']:
                                pixels[q] = x['uv']*binning
                                score[q], gap[q], peak_ok[q] = x['score'], x['gap'], x['peak_ok']
                    else:
                        pixel_sd = np.sqrt(s.pixel_noise**2 + (s.ground_noise*FOCAL/z_true)**2)
                        pixels = pix_true + pixel_sd[:, None]*d['r_pix'].normal(size=(m, 2))
                        d['r_pix'].random(m)                                  # the Monte Carlo's outlier draw (rate 0)
                        in_view = matched = peak_ok = np.ones(m, dtype=bool)
                        score = gap = np.full(m, np.nan)
                    err_px = pixels - pix_true
                    err_ground = np.hypot(err_px[:, 0], err_px[:, 1])*z_true/FOCAL
                    correct = in_view & (err_ground <= FALSE_M)
                    assumed = np.sqrt(0.3**2 + (0.15*FOCAL/z_true)**2)
                    counts[0] = m
                    counts[1] = matched.sum()
                    use = np.flatnonzero(matched)
                    filter_ok = np.zeros(m, dtype=bool)
                    if len(use):
                        _, z_hat = cam.project(d['map_points'][chosen[use]], f.p, f.r)
                        z_hat = np.maximum(z_hat, 1.0)
                        pixel_var = c.pixel_noise**2 + (c.ground_noise*FOCAL/z_hat)**2
                        t0 = time.perf_counter()
                        out = f.update_landmarks(cam, d['map_points'][chosen[use]], pixels[use], pixel_var, map_var,
                                                 c.gate, c.iterations, c.robust, c.recover)
                        times[3] = time.perf_counter() - t0
                        filter_ok[use] = out['accepted']
                        counts[6] = out['recovered']
                    counts[2] = filter_ok.sum()
                    counts[3] = (matched & ~correct).sum()
                    counts[4] = (filter_ok & ~correct).sum()
                    counts[5] = (matched & correct & ~filter_ok).sum()
                    for q in range(m):
                        if in_view[q]:
                            rows.append((fi, t_p[k, 2], z_true[q], score[q], gap[q], float(peak_ok[q]), float(matched[q]),
                                         float(filter_ok[q]), err_px[q, 0], err_px[q, 1], err_ground[q], assumed[q],
                                         float(correct[q])))
            e = f.error_from(t_p[k], t_v[k], t_r[k], d['ba'][k], d['bg'][k], d['baro_bias'][k], d['true_delay'][fi])
            cov = f.cov
            rec['err'][fi] = e
            rec['sig'][fi] = np.sqrt(np.diag(cov))
            try:
                rec['nees'][fi] = [e[:16] @ np.linalg.solve(cov[:16, :16], e[:16]),
                                   e[:2] @ np.linalg.solve(cov[:2, :2], e[:2]),
                                   e[3:5] @ np.linalg.solve(cov[3:5, 3:5], e[3:5])]
            except np.linalg.LinAlgError:
                pass
            rec['counts'][fi] = counts
            rec['times'][fi] = times
            rec['height'][fi] = t_p[k, 2]
        if k % 10 == 0:
            f.update_baro(t_p[k, 2] + d['baro_bias'][k] + d['baro_noise'][k], c.baro_sigma)
        f.propagate(NAV_DT, d['accel'][k], d['gyro'][k])
    rec['rows'] = np.array(rows, dtype=float).reshape(-1, len(LANDMARK_COLS))
    rec['wall_s'] = time.perf_counter() - started
    rec['e0'] = d['e0']
    return rec


# --- Scoring ---------------------------------------------------------------------

def pad(recs, key):
    width = max(len(r[key]) for r in recs)
    out = np.full((len(recs), width) + recs[0][key].shape[1:], np.nan)
    for i, r in enumerate(recs):
        out[i, :len(r[key])] = r[key]
    return out


def percentile(a, q):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return float(np.quantile(a, q)) if len(a) else None


def rms(a):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a**2))) if len(a) else None


def frame_scores(recs, floor=80.0):
    """Navigation scoring of a set of descents, the Monte Carlo's definitions."""
    err, sig, nees, height = pad(recs, 'err'), pad(recs, 'sig'), pad(recs, 'nees'), pad(recs, 'height')
    counts = pad(recs, 'counts')
    above = height >= floor
    settled = above & (np.arange(err.shape[1])[None, :] >= FIRST_SETTLED)
    horiz = np.hypot(err[..., 0], err[..., 1])
    sig_h = np.hypot(sig[..., 0], sig[..., 1])
    tilt = np.hypot(err[..., 6], err[..., 7])/DEG
    yaw = np.abs(err[..., 8])/DEG
    vel = np.hypot(err[..., 3], err[..., 4])
    used = settled & (counts[..., 0] > 0)
    worst = np.where(settled, horiz, -np.inf).max(axis=1)
    offered, matched, accepted = counts[..., 0], counts[..., 1], counts[..., 2]
    low = settled & (accepted < 4)
    runs = []
    for i in range(len(recs)):
        longest = cur = 0
        for flag in low[i]:
            cur = cur + 1 if flag else 0
            longest = max(longest, cur)
        runs.append(longest)
    horiz_valid = np.where(settled, horiz, np.nan)
    return {
        'descents': len(recs), 'settled_frames': int(settled.sum()),
        'horizontal_rms_m': rms(horiz_valid), 'horizontal_p95_m': percentile(horiz_valid, 0.95),
        'horizontal_max_m': float(worst.max()), 'horizontal_median_m': percentile(horiz_valid, 0.5),
        'filter_horizontal_sigma_rms_m': rms(np.where(settled, sig_h, np.nan)),
        'nees_pos2_mean': float(np.nanmean(np.where(settled, nees[..., 1], np.nan))),
        'nees_pos2_median': percentile(np.where(settled, nees[..., 1], np.nan), 0.5),
        'tilt_rms_deg': rms(np.where(settled, tilt, np.nan)), 'yaw_rms_deg': rms(np.where(settled, yaw, np.nan)),
        'filter_tilt_sigma_rms_deg': rms(np.where(settled, np.hypot(sig[..., 6], sig[..., 7])/DEG, np.nan)),
        'velocity_rms_mps': rms(np.where(settled, vel, np.nan)),
        'filter_velocity_sigma_rms_mps': rms(np.where(settled, np.hypot(sig[..., 3], sig[..., 4]), np.nan)),
        'fraction_within_10m_after_2s': float(np.mean(worst <= LOST_M)),
        'landmarks_offered_per_frame': float(np.nanmean(np.where(used, offered, np.nan))),
        'landmarks_matcher_accepted_per_frame': float(np.nanmean(np.where(used, matched, np.nan))),
        'landmarks_filter_accepted_per_frame': float(np.nanmean(np.where(used, accepted, np.nan))),
        'frames_with_fewer_than_4_filter_accepted': int(low.sum()),
        'longest_run_fewer_than_4_accepted': int(max(runs)),
        'recoveries': int(np.nansum(counts[..., 6])),
        'per_descent': [{'horizontal_rms_m': rms(horiz_valid[i]), 'horizontal_max_m': float(worst[i]),
                         'horizontal_at_end_m': float(horiz[i][np.flatnonzero(above[i])[-1]]),
                         'first_error_m': float(horiz[i, 0]), 'lost_lock_10m': bool(worst[i] > LOST_M),
                         'lost_lock_16m': bool(worst[i] > SEARCH_M)} for i in range(len(recs))],
    }, {'horiz': horiz, 'sig_h': sig_h, 'height': height, 'settled': settled}


def fit_noise_model(depth, ex, ey):
    """Least squares for e^2 = a^2 + (b f / z)^2 over both pixel axes: (a px, b m)."""
    x = np.r_[(FOCAL/depth)**2, (FOCAL/depth)**2]
    y = np.r_[ex**2, ey**2]
    design = np.column_stack((np.ones(len(x)), x))
    coef = np.linalg.lstsq(design, y, rcond=None)[0]
    return math.sqrt(max(coef[0], 0.0)), math.sqrt(max(coef[1], 0.0))


def matcher_scores(rows):
    """Measured match quality from per-landmark rows (columns LANDMARK_COLS)."""
    if len(rows) == 0:
        return None
    col = {k: rows[:, i] for i, k in enumerate(LANDMARK_COLS)}
    acc = col['matcher_accepted'] > 0
    keep = col['filter_accepted'] > 0
    correct = col['correct'] > 0
    n = len(rows)

    def block(sel):
        c = sel & acc & correct
        out = {'landmarks_attempted': int(sel.sum()), 'matcher_accepted': int((sel & acc).sum()),
               'matcher_accepted_fraction': float((sel & acc).sum()/max(sel.sum(), 1)),
               'accepted_and_correct': int(c.sum()), 'false_accepted_by_matcher': int((sel & acc & ~correct).sum()),
               'false_rate_matcher': float((sel & acc & ~correct).sum()/max((sel & acc).sum(), 1)),
               'filter_accepted': int((sel & keep).sum()), 'false_kept_by_filter': int((sel & keep & ~correct).sum()),
               'false_rate_after_filter': float((sel & keep & ~correct).sum()/max((sel & keep).sum(), 1)),
               'correct_matches_dropped_by_filter': int((sel & acc & correct & ~keep).sum()),
               'rejected_by_matcher_but_peak_was_correct': int((sel & ~acc & correct).sum()),
               'rejected_by_matcher': int((sel & ~acc).sum())}
        if c.sum() >= 5:
            ex, ey, z = col['err_x_px'][c], col['err_y_px'][c], col['depth_m'][c]
            sd = np.sqrt((np.var(ex) + np.var(ey))/2)
            out.update({'pixel_error_sd_px_correct': float(sd), 'pixel_error_rms_px_correct': rms(np.r_[ex, ey]),
                        'pixel_error_mean_x_px': float(ex.mean()), 'pixel_error_mean_y_px': float(ey.mean()),
                        'pixel_error_robust_sd_px': float(1.4826*np.median(np.abs(np.r_[ex, ey] - np.median(np.r_[ex, ey])))),
                        'assumed_sd_px_rms': rms(col['sigma_assumed_px'][c]),
                        'ratio_measured_to_assumed': float(rms(np.r_[ex, ey])/rms(col['sigma_assumed_px'][c])),
                        'ground_error_rms_m_correct': rms(col['err_ground_m'][c]),
                        'normalised_error_sd': float(rms(np.r_[ex/col['sigma_assumed_px'][c], ey/col['sigma_assumed_px'][c]])),
                        'fraction_beyond_3_assumed_sd': float(np.mean(np.hypot(ex, ey) > 3*col['sigma_assumed_px'][c]))})
        a = sel & acc
        if a.sum() >= 5:
            out['accepted_fraction_beyond_3_assumed_sd_all'] = float(np.mean(
                np.hypot(col['err_x_px'][a], col['err_y_px'][a]) > 3*col['sigma_assumed_px'][a]))
        return out
    total = block(np.ones(n, dtype=bool))
    c = acc & correct
    if c.sum() >= 10:
        a, b = fit_noise_model(col['depth_m'][c], col['err_x_px'][c], col['err_y_px'][c])
        total['fit_floor_px'], total['fit_ground_m'] = a, b
    if c.sum() >= 10:
        miss = np.hypot(col['err_x_px'][c], col['err_y_px'][c])
        total['pixel_error_norm_quantiles_px_correct'] = {'median': percentile(miss, 0.5), 'p90': percentile(miss, 0.9),
                                                          'p99': percentile(miss, 0.99), 'max': float(miss.max())}
    frame = col['frame']
    total['by_frame_group'] = {'frame_0': block(frame == 0), 'frames_1_to_9': block((frame >= 1) & (frame < FIRST_SETTLED)),
                               'frames_10_on': block(frame >= FIRST_SETTLED)}
    total['by_height'] = []
    for lo, hi in zip(HEIGHT_EDGES[1:], HEIGHT_EDGES[:-1]):
        sel = (col['height_m'] >= lo) & (col['height_m'] < hi)
        if sel.sum() == 0:
            continue
        row = block(sel)
        row.update({'height_lo_m': lo, 'height_hi_m': hi, 'mean_depth_m': float(col['depth_m'][sel].mean())})
        total['by_height'].append(row)
    sc = col['score'][acc]
    total['score_quantiles_accepted'] = [percentile(sc, q) for q in (0.05, 0.5, 0.95)]
    return total


def frame_mean_error(rows):
    """How much of the correct-match pixel error is common to a frame (the filter assumes none)."""
    if len(rows) == 0:
        return None
    ok = (rows[:, 6] > 0) & (rows[:, 12] > 0)
    rows = rows[ok]
    frames = rows[:, 0]
    total, common, used = 0.0, 0.0, 0
    for fr in np.unique(frames):
        sel = frames == fr
        if sel.sum() < 4:
            continue
        for axis in (8, 9):
            e = rows[sel, axis]
            total += np.sum(e**2)
            common += sel.sum()*e.mean()**2
        used += int(sel.sum())
    return None if used == 0 else {'landmarks_in_frames_with_4_or_more': used,
                                   'fraction_of_squared_error_in_frame_mean': float(common/total)}


def timing_scores(recs):
    t = pad(recs, 'times')
    c = pad(recs, 'counts')
    ran = c[..., 0] > 0
    n_match = c[..., 0]                                  # one matcher call per landmark attempted
    per_lm = lambda col: float(np.nansum(t[..., col])/max(np.nansum(np.where(ran, n_match, 0)), 1))  # noqa: E731
    return {'frames_with_imagery': int(ran.sum()),
            'render_s_per_frame': float(np.nansum(t[..., 0])/max(ran.sum(), 1)),
            'rectify_s_per_landmark': per_lm(1), 'correlate_s_per_landmark': per_lm(2),
            'rectify_plus_correlate_s_per_frame': float(np.nansum(t[..., 1] + t[..., 2])/max(ran.sum(), 1)),
            'rectify_plus_correlate_s_per_frame_max': float(np.nanmax(t[..., 1] + t[..., 2])),
            'filter_update_s_per_frame': float(np.nansum(t[..., 3])/max(ran.sum(), 1)),
            'landmarks_per_frame': float(np.nansum(np.where(ran, n_match, 0))/max(ran.sum(), 1))}


def by_height_frames(recs, floor=80.0):
    """Horizontal error and landmark counts per height bin (all frames above the floor, after the settle)."""
    stats, arrays = frame_scores(recs, floor)
    horiz, sig_h, height, settled = arrays['horiz'], arrays['sig_h'], arrays['height'], arrays['settled']
    counts = pad(recs, 'counts')
    out = []
    for lo, hi in zip(HEIGHT_EDGES[1:], HEIGHT_EDGES[:-1]):
        sel = settled & (height >= lo) & (height < hi)
        if not sel.any():
            continue
        out.append({'height_lo_m': lo, 'height_hi_m': hi, 'frames': int(sel.sum()),
                    'horizontal_rms_m': rms(np.where(sel, horiz, np.nan)),
                    'horizontal_p95_m': percentile(np.where(sel, horiz, np.nan), 0.95),
                    'filter_sigma_rms_m': rms(np.where(sel, sig_h, np.nan)),
                    'offered': float(np.nanmean(np.where(sel, counts[..., 0], np.nan))),
                    'matcher_accepted': float(np.nanmean(np.where(sel, counts[..., 1], np.nan))),
                    'filter_accepted': float(np.nanmean(np.where(sel, counts[..., 2], np.nan)))})
    return out


def summarise(recs):
    stats, _ = frame_scores(recs)
    rows = np.vstack([r['rows'] for r in recs]) if any(len(r['rows']) for r in recs) else np.zeros((0, len(LANDMARK_COLS)))
    stats['by_height'] = by_height_frames(recs)
    stats['matcher'] = matcher_scores(rows)
    stats['common_mode'] = frame_mean_error(rows)
    stats['timing'] = timing_scores(recs)
    return stats


# --- Sanity checks ---------------------------------------------------------------

def perfect_pose_check(binning=2, heights=(300.0, 200.0, 120.0, 82.0), tilts_deg=(0.0, 8.0), noise=0.0, seed=1,
                       relief=0.0):
    """Render at a pose, match with that SAME pose (no pose error): the matcher must be ~100 % correct with
    sub-pixel error. Also compares the filter's Camera.project with the renderer's geometry."""
    cam = mc.nominal_camera()
    size = (FULL_SIZE[0]//binning, FULL_SIZE[1]//binning)
    focal = FOCAL/binning
    rng = np.random.default_rng(seed)
    errs, ok, score, accepted, geometry = [], [], [], [], 0.0
    for height in heights:
        for tilt in tilts_deg:
            axis = rng.uniform(0, 2*math.pi)
            r = nat.rotation(math.radians(tilt)*np.array([math.cos(axis), math.sin(axis), 0.]))@nat.rotation([0, 0, rng.uniform(-3, 3)])@LEVEL
            p = np.array([rng.uniform(-500, 500), rng.uniform(-500, 500), height])
            centre = p + r @ LEVER
            image = render_frame(centre, r, size, focal, relief)
            if noise:
                image = image + noise*rng.standard_normal(size)
            points =np.column_stack((p[0] + rng.uniform(-0.35, 0.35, 40)*height, p[1] + rng.uniform(-0.25, 0.25, 40)*height,
                                      np.zeros(40)))
            view = patch_in_view(cam, points[:, :2], p, r, size, binning)
            for q in np.flatnonzero(view)[:8]:
                world = points[q].copy()
                if relief:
                    world[2] = float(surface_height(world[0], world[1], relief))
                res = match_landmark(image, points[q, :2], centre, r, focal)
                true_px, _ = cam.project(world[None], p, r)
                filt_uv = true_px[0]/binning
                if not relief:
                    geometry = max(geometry, float(np.max(np.abs(nat.ground_to_pixel(points[q, :2], centre, r, focal, size)[0]
                                                                  - filt_uv))))
                err = np.hypot(*(res['uv']*binning - true_px[0]))
                depth = float(cam.project(world[None], p, r)[1][0])
                errs.append(err)
                ok.append(err*depth/FOCAL <= FALSE_M)
                score.append(res['score'])
                accepted.append(res['accepted'])
    errs = np.array(errs)
    return {'binning': binning, 'noise_gray': noise, 'relief': relief, 'landmarks': len(errs),
            'accepted_fraction': float(np.mean(accepted)), 'correct_fraction': float(np.mean(ok)),
            'pixel_error_median_px': float(np.median(errs)), 'pixel_error_max_px': float(errs.max()),
            'pixel_error_rms_px': rms(errs), 'min_score': float(min(score)),
            'filter_camera_vs_renderer_geometry_max_px': geometry}


# --- Planning and output ---------------------------------------------------------

def clean(value):
    return nat.clean(value)


def csv_rows(variant, mode, j, rec):
    out = []
    for fi in range(len(rec['height'])):
        e, sg, c, t, pre = rec['err'][fi], rec['sig'][fi], rec['counts'][fi], rec['times'][fi], rec['pre'][fi]
        out.append({'variant': variant, 'mode': mode, 'descent': j, 'frame': fi, 'time_s': round(fi*FRAME_DT, 3),
                    'truth_height_m': rec['height'][fi], 'in_view_candidates': rec['candidates'][fi],
                    'attempted': c[0], 'matcher_accepted': c[1], 'filter_accepted': c[2],
                    'false_matcher_accepted': c[3], 'false_filter_accepted': c[4],
                    'correct_dropped_by_filter': c[5], 'filter_recovered': c[6],
                    'prior_horizontal_error_m': pre[0], 'prior_tilt_error_deg': pre[1], 'prior_yaw_error_deg': pre[2],
                    'horizontal_error_m': math.hypot(e[0], e[1]), 'filter_horizontal_sigma_m': math.hypot(sg[0], sg[1]),
                    'nees_pos2': rec['nees'][fi][1], 'tilt_error_deg': math.hypot(e[6], e[7])/DEG,
                    'yaw_error_deg': abs(e[8])/DEG, 'velocity_error_mps': math.hypot(e[3], e[4]),
                    'height_error_m': e[2], 'render_s': t[0], 'rectify_s': t[1], 'correlate_s': t[2], 'filter_s': t[3]})
    return out



def first_update(recs):
    """What the first image does: the matcher meets the full first-fix pose error and the filter has no history."""
    out = []
    for r in recs:
        h = np.hypot(r['err'][:, 0], r['err'][:, 1])
        below = np.flatnonzero(h < 2.0)
        c = r['counts']
        out.append({'prior_horizontal_m': float(r['pre'][0, 0]), 'prior_tilt_deg': float(r['pre'][0, 1]),
                    'prior_yaw_deg': float(r['pre'][0, 2]), 'after_first_update_m': float(h[0]),
                    'max_horizontal_frames_0_to_9_m': float(np.nanmax(h[:FIRST_SETTLED])),
                    'first_frame_below_2m': int(below[0]) if len(below) else None,
                    'frame0_attempted': int(c[0, 0]), 'frame0_matcher_accepted': int(c[0, 1]),
                    'frame0_filter_accepted': int(c[0, 2]), 'frame0_false_matcher': int(c[0, 3]),
                    'frame0_false_filter': int(c[0, 4]), 'recoveries_frames_0_to_9': int(np.nansum(c[:FIRST_SETTLED, 6]))})
    return out


def summarise_all(recs):
    stats = summarise(recs)
    stats['first_update'] = first_update(recs)
    return stats


EXPERIMENTS = [
    # Priority order. 'minimum' is the fewest descents worth keeping when the time budget forces cuts;
    # 'weight' is the cost of a descent relative to the base image run (render + match + filter).
    dict(name='base_moderate', canopy='moderate', overrides={}, modes=('image', 'control', 'control_mc'),
         wanted=6, minimum=4, weight=1.0, note='flat ground, half-resolution images, 2 grey levels of noise'),
    dict(name='relief_2m', canopy='moderate', overrides={'relief': 2.0}, modes=('image', 'control'),
         wanted=3, minimum=2, weight=1.4, note='rendered terrain relief of 2 m amplitude that the matcher and the map do not know'),
    dict(name='yaw_5deg', canopy='moderate', overrides={'yaw0_deg': 5.0}, modes=('image', 'control'),
         wanted=4, minimum=2, weight=1.0, note='initial yaw error of +/-5 deg (sign drawn per descent) in place of the drawn 2 deg'),
    dict(name='noise_10gl', canopy='moderate', overrides={'noise_gray': 10.0}, modes=('image',),
         wanted=3, minimum=2, weight=1.0, control_from='base_moderate',
         note='sensitivity: 10 grey levels of image noise (signal-to-noise 2.6 instead of 13)'),
    dict(name='fullres', canopy='moderate', overrides={'binning': 1}, modes=('image',),
         wanted=2, minimum=1, weight=2.0, control_from='base_moderate',
         note='full 1456 x 1088 images instead of half resolution'),
    dict(name='base_benign', canopy='benign', overrides={}, modes=('image', 'control'),
         wanted=3, minimum=2, weight=1.0, note='stable canopy'),
]
CONTROL_COST_S = 4.0                    # the geometry-only runs of one descent, host seconds


def plan_counts(experiments, counts, done, unit, remaining):
    """Cut the lowest-priority experiments until the work left fits the time left. unit: seconds per weight 1."""
    counts = dict(counts)

    def cost():
        return sum((counts[e['name']] - done[e['name']])*(e['weight']*unit + CONTROL_COST_S) for e in experiments)
    while cost() > remaining:
        for e in reversed(experiments):
            if counts[e['name']] > max(e['minimum'], done[e['name']]):
                counts[e['name']] -= 1
                break
        else:
            for e in reversed(experiments):
                if counts[e['name']] > done[e['name']]:
                    counts[e['name']] = done[e['name']]
                    break
            else:
                break
    return counts


def run_experiments(experiments, budget_s, base_count=0, quick=False, log=print):
    clock = time.perf_counter()
    counts = {e['name']: (base_count if (base_count and e['name'] == 'base_moderate') else e['wanted']) for e in experiments}
    if quick:
        counts = {e['name']: 2 for e in experiments}
    done = {e['name']: 0 for e in experiments}
    results = {e['name']: {'modes': {}, 'descents_run': 0} for e in experiments}
    truths, notes, csv_all, plan_log = {}, {}, [], []
    unit = None
    for e in experiments:
        name, canopy = e['name'], e['canopy']
        while done[name] < counts[name]:
            j = done[name]
            elapsed = time.perf_counter() - clock
            if unit is not None and elapsed + e['weight']*unit*1.1 + CONTROL_COST_S > budget_s:
                log('  stopping %s at %d descents: %.0f s used of %.0f s' % (name, j, elapsed, budget_s))
                plan_log.append('%s cut to %d descents by the time budget' % (name, j))
                counts[name] = j
                break
            if canopy not in truths:
                truths[canopy] = load_truth(canopy, 8, 'auto')
                notes[canopy] = truths[canopy]['source']
            for mode in e['modes']:
                setup = Setup(canopy=canopy, mode=mode, **e['overrides'])
                rec = run_descent(truths[canopy], j, setup)
                results[name]['modes'].setdefault(mode, []).append(rec)
                csv_all += csv_rows(name, mode, j, rec)
                h = np.hypot(rec['err'][:, 0], rec['err'][:, 1])
                log('  %-14s %-10s descent %d: %3d frames %5.1f s | horizontal error %.1f m after frame 0, max after 2 s %.2f m, '
                    'end %.2f m | landmarks %.1f offered %.1f matched %.1f kept' % (
                        name, mode, j, len(h), rec['wall_s'], h[0], np.nanmax(h[FIRST_SETTLED:]), h[-1],
                        np.nanmean(rec['counts'][:, 0]), np.nanmean(rec['counts'][:, 1]), np.nanmean(rec['counts'][:, 2])))
                if unit is None and mode == 'image':
                    unit = rec['wall_s']/e['weight']
                    log('  a base-weight image-in-the-loop descent costs %.0f s on this PC (loaded)' % unit)
            done[name] += 1
            results[name]['descents_run'] += 1
            if unit is not None and sum(done.values()) == 1:
                new = plan_counts(experiments, counts, done, unit, budget_s - (time.perf_counter() - clock) - 30.0)
                if new != counts:
                    plan_log.append('plan after the first descent: %s' % new)
                    log('  time plan: %s' % new)
                counts = new
    return results, notes, csv_all, plan_log, truths, time.perf_counter() - clock


def build_report(experiments, results, notes, sanity, plan_log, elapsed, command):
    report = {'label': 'SYNTHETIC', 'date': DATE, 'seed': SEED, 'python': platform.python_version(),
              'numpy': np.__version__, 'host': platform.platform(), 'command': command, 'truth_source': notes,
              'sanity_true_pose': sanity, 'time_plan': plan_log,
              'settings': {'image_px_sensor': list(mc.IMAGE), 'focal_px_sensor': FOCAL, 'lever_m': list(LEVER),
                           'matcher': {'template_px': TEMPLATE_PX, 'map_scale_m': SCALE_M, 'search_m': SEARCH_M,
                                       'score_gate': nat.SCORE_GATE, 'gap_gate': nat.GAP_GATE,
                                       'gap_exclusion_px': nat.GAP_EXCLUSION_PX, 'search_patch_px': BIG_PX},
                           'false_match_ground_error_m': FALSE_M, 'image_noise_gray_sd': NOISE_GRAY,
                           'terrain_signal_sd_gray': 26.5, 'sensors': asdict(SENSORS), 'filter': asdict(mc.FilterCfg()),
                           'frame_hz': 1/FRAME_DT, 'landmark_cells': [6, 5], 'max_landmarks': 30,
                           'first_settled_frame': FIRST_SETTLED, 'lost_lock_threshold_m': LOST_M,
                           'control_noise': 'true pixel + N(0, 0.3^2 + (0.15 m f/depth)^2) per axis, no outliers',
                           'sensors_unmodelled_errors': 'switched off: map_xy, relief_sd, focal, principal point, misalignment, datum, outliers'},
              'experiments': {}, 'limits': LIMITS}
    for e in experiments:
        name = e['name']
        recs = results[name]['modes']
        if not recs:
            continue
        entry = {'note': e['note'], 'overrides': e['overrides'], 'canopy': e['canopy'],
                 'descents_run': results[name]['descents_run'], 'modes': {}}
        for mode, items in recs.items():
            entry['modes'][mode] = summarise_all(items)
        ref = e.get('control_from')
        if ref and results[ref]['modes'].get('control'):
            n = results[name]['descents_run']
            entry['modes']['control (paired, from ' + ref + ')'] = summarise_all(results[ref]['modes']['control'][:n])
        report['experiments'][name] = entry
    report['runtime_s'] = elapsed
    return report


def fmt(x, spec='.2f'):
    return '  n/a' if x is None else format(x, spec)


def print_report(report):
    for name, ex in report['experiments'].items():
        print('\n%s: %d descents, canopy %s, %s' % (name, ex['descents_run'], ex['canopy'], ex['note']))
        print('  %-30s %6s %6s %6s %6s %6s %5s %5s %5s | %7s %7s %7s' % (
            'mode', 'rms_m', 'p95_m', 'max_m', 'sig_m', 'NEES2', 'tilt', 'yaw', '<10m', 'offered', 'matched', 'kept'))
        for mode, s in ex['modes'].items():
            print('  %-30s %6s %6s %6s %6s %6s %5s %5s %5s | %7.1f %7.1f %7.1f' % (
                mode, fmt(s['horizontal_rms_m']), fmt(s['horizontal_p95_m']), fmt(s['horizontal_max_m']),
                fmt(s['filter_horizontal_sigma_rms_m']), fmt(s['nees_pos2_mean']), fmt(s['tilt_rms_deg']),
                fmt(s['yaw_rms_deg']), fmt(s['fraction_within_10m_after_2s']), s['landmarks_offered_per_frame'],
                s['landmarks_matcher_accepted_per_frame'], s['landmarks_filter_accepted_per_frame']))
        s = ex['modes'].get('image')
        m = s and s.get('matcher')
        if m:
            print('  matcher: accepted %.3f of attempted; false %.4f before the filter, %.4f after; '
                  'rms pixel error of correct matches %s px vs assumed %s px; fitted floor %s px + %s m on the ground'
                  % (m['matcher_accepted_fraction'], m['false_rate_matcher'], m['false_rate_after_filter'],
                     fmt(m.get('pixel_error_rms_px_correct'), '.3f'), fmt(m.get('assumed_sd_px_rms'), '.3f'),
                     fmt(m.get('fit_floor_px'), '.2f'), fmt(m.get('fit_ground_m'), '.3f')))
            for b in m['by_height']:
                print('    %3.0f-%3.0f m: %4d attempted, accepted %.3f, false %.4f -> %.4f, rms err %s px (assumed %s), '
                      'ratio %s' % (b['height_lo_m'], b['height_hi_m'], b['landmarks_attempted'], b['matcher_accepted_fraction'],
                                    b['false_rate_matcher'], b['false_rate_after_filter'],
                                    fmt(b.get('pixel_error_rms_px_correct'), '.3f'), fmt(b.get('assumed_sd_px_rms'), '.3f'),
                                    fmt(b.get('ratio_measured_to_assumed'), '.2f')))
            t = s['timing']
            print('  timing (loaded host): render %.3f s/frame, rectify %.2f ms + correlate %.2f ms per landmark, '
                  '%.1f landmarks -> %.3f s/frame, filter update %.1f ms/frame' % (
                      t['render_s_per_frame'], 1e3*t['rectify_s_per_landmark'], 1e3*t['correlate_s_per_landmark'],
                      t['landmarks_per_frame'], t['rectify_plus_correlate_s_per_frame'], 1e3*t['filter_update_s_per_frame']))


def _num(x):
    return float('nan') if x is None or x == '' else float(x)


PRIOR_ERROR_BINS_M = (0.0, 1.0, 2.0, 4.0, 6.0, 10.0, 1e9)
PRIOR_TILT_BINS_DEG = (0.0, 0.5, 1.0, 2.0, 3.0, 1e9)


def analyse_rows(rows):
    """Post-processing of the per-frame table (no simulation): false matches against the prior pose error the
    matcher faced, and which descents lost lock. rows: dicts as csv_rows (numbers or None / '')."""
    everything = [{k: (v if k in ('variant', 'mode') else _num(v)) for k, v in r.items()} for r in rows]
    rows = [r for r in everything if r['mode'] == 'image']
    keys = list(dict.fromkeys((r['variant'], int(r['descent'])) for r in rows))
    runs = {k: [r for r in rows if (r['variant'], int(r['descent'])) == k] for k in keys}
    lost = {k: v[-1]['horizontal_error_m'] > LOST_M for k, v in runs.items()}

    def tally(sel):
        used = [r for r in sel if r['attempted'] > 0]
        att = sum(r['attempted'] for r in used)
        acc = sum(r['matcher_accepted'] for r in used)
        fm = sum(r['false_matcher_accepted'] for r in used)
        kept = sum(r['filter_accepted'] for r in used)
        ff = sum(r['false_filter_accepted'] for r in used)
        return {'frames': len(used), 'landmarks_attempted': int(att), 'matcher_accepted': int(acc),
                'matcher_accepted_fraction': acc/att if att else None, 'false_accepted_by_matcher': int(fm),
                'false_rate_matcher': fm/acc if acc else None, 'kept_by_filter': int(kept),
                'false_kept_by_filter': int(ff), 'false_rate_after_filter': ff/kept if kept else None}

    def binned(edges, key, sel):
        out = []
        for lo, hi in zip(edges[:-1], edges[1:]):
            part = tally([r for r in sel if lo <= r[key] < hi])
            if part['frames']:
                out.append(dict(part, lo=lo, hi=None if hi > 1e8 else hi))
        return out
    every = [r for v in runs.values() for r in v]
    kept_lock = [r for k, v in runs.items() if not lost[k] for r in v]
    by_height = []
    for lo, hi in zip(HEIGHT_EDGES[1:], HEIGHT_EDGES[:-1]):
        by_height.append(dict(tally([r for r in kept_lock if lo <= r['truth_height_m'] < hi]), height_lo_m=lo, height_hi_m=hi))
    descents = []
    for k, v in runs.items():
        h = np.array([r['horizontal_error_m'] for r in v])
        first = v[0]
        descents.append({'variant': k[0], 'descent': k[1], 'lost_lock': bool(lost[k]),
                         'frame0_prior_horizontal_m': first['prior_horizontal_error_m'],
                         'frame0_prior_tilt_deg': first['prior_tilt_error_deg'],
                         'frame0_prior_yaw_deg': first['prior_yaw_error_deg'],
                         'frame0_attempted': first['attempted'], 'frame0_matcher_accepted': first['matcher_accepted'],
                         'frame0_false_matcher': first['false_matcher_accepted'], 'frame0_filter_accepted': first['filter_accepted'],
                         'frame0_false_filter': first['false_filter_accepted'],
                         'after_frame0_horizontal_m': float(h[0]), 'max_horizontal_frames_1_to_9_m': float(h[1:FIRST_SETTLED].max()),
                         'final_horizontal_m': float(h[-1]),
                         'first_frame_over_10m_after_frame0': next((int(i) for i in range(1, len(h)) if h[i] > LOST_M), None)})
    # Image versus control on the descents where the image run kept lock (settled frames above 80 m only).
    paired = {}
    for variant in dict.fromkeys(k[0] for k in keys):
        ids = [k[1] for k in keys if k[0] == variant and not lost[k]]
        control_variant = variant if any(r['variant'] == variant and r['mode'] == 'control' for r in everything) else 'base_moderate'
        row = {'descents_kept_lock': len(ids), 'descents_total': sum(1 for k in keys if k[0] == variant)}
        for label, v, mode in (('image', variant, 'image'), ('control', control_variant, 'control')):
            sel = [r for r in everything if r['variant'] == v and r['mode'] == mode and int(r['descent']) in ids
                   and r['frame'] >= FIRST_SETTLED and r['truth_height_m'] >= 80.0]
            if sel:
                h = np.array([r['horizontal_error_m'] for r in sel])
                row[label] = {'frames': len(sel), 'horizontal_rms_m': rms(h), 'horizontal_p95_m': percentile(h, 0.95),
                              'horizontal_max_m': float(h.max()),
                              'filter_horizontal_sigma_rms_m': rms([r['filter_horizontal_sigma_m'] for r in sel]),
                              'nees_pos2_mean': float(np.mean([r['nees_pos2'] for r in sel])),
                              'tilt_rms_deg': rms([r['tilt_error_deg'] for r in sel]),
                              'velocity_rms_mps': rms([r['velocity_error_mps'] for r in sel]),
                              'landmarks_filter_accepted_per_frame': float(np.mean([r['filter_accepted'] for r in sel]))}
        paired[variant] = row
    return {'image_descents': len(runs), 'descents_that_lost_lock': int(sum(lost.values())),
            'image_vs_control_on_descents_that_kept_lock': paired,
            'lost_lock_definition': f'horizontal error above {LOST_M:.0f} m at the last frame above 78 m',
            'all_frames': tally(every),
            'false_vs_prior_horizontal_error_m': binned(PRIOR_ERROR_BINS_M, 'prior_horizontal_error_m', every),
            'false_vs_prior_tilt_error_deg': binned(PRIOR_TILT_BINS_DEG, 'prior_tilt_error_deg', every),
            'frame0_only': tally([r for r in every if r['frame'] == 0]),
            'locked_descents_frames_1_to_9': tally([r for r in kept_lock if 1 <= r['frame'] < FIRST_SETTLED]),
            'locked_descents_frames_10_on': tally([r for r in kept_lock if r['frame'] >= FIRST_SETTLED]),
            'locked_descents_by_height': by_height, 'lost_descents_all_frames': tally(
                [r for k, v in runs.items() if lost[k] for r in v]),
            'per_descent': descents}


def diagnose_lock(truth, j, setup=None, seed=5):
    """Re-fly one image-in-the-loop descent and explain where it ends: the horizontal error vector over time and how
    well the terrain correlates with itself at that shift (a lattice alias of the procedural texture would explain it)."""
    rec = run_descent(truth, j, setup or Setup())
    h = np.hypot(rec['err'][:, 0], rec['err'][:, 1])
    shift = rec['err'][-1, :2].copy()                       # truth minus estimate, east / north, m
    rng = np.random.default_rng(seed)
    centres = rng.uniform(-1000.0, 1000.0, (200, 2))
    offsets = nat.patch_grid((0.0, 0.0), TEMPLATE_PX, SCALE_M).reshape(-1, 2)
    ncc = []
    for centre in centres:
        a = terrain(*(centre + offsets).T)
        b = terrain(*(centre + offsets + shift).T)
        ncc.append(np.corrcoef(a, b)[0, 1])
    acf = nat.autocorrelation(0.5, n=512, max_lag=96)
    peaks = []
    inner = acf[1:-1, 1:-1]
    local = ((inner >= acf[:-2, 1:-1]) & (inner >= acf[2:, 1:-1]) & (inner >= acf[1:-1, :-2]) & (inner >= acf[1:-1, 2:])
             & (inner > 0.8))
    for iy, ix in zip(*np.nonzero(local)):
        lag = (np.array([ix + 1 - 96, iy + 1 - 96]))*0.5
        if 3.0 < np.hypot(*lag) <= 45.0:
            peaks.append((float(lag[0]), float(lag[1]), float(inner[iy, ix])))
    peaks.sort(key=lambda t: -t[2])
    nearest = min(peaks, key=lambda t: min(np.hypot(t[0] - shift[0], t[1] - shift[1]), np.hypot(t[0] + shift[0], t[1] + shift[1])),
                  default=None)
    frames = [0, 1, 2, 3, 5, 10, 20, 50, len(h) - 1]
    return {'descent': j, 'final_horizontal_error_m': float(h[-1]), 'initial_error_m': [float(x) for x in rec['e0'][:3]],
            'initial_attitude_error_deg': [float(x/DEG) for x in rec['e0'][6:9]],
            'error_vector_east_north_m_by_frame': {str(i): [float(rec['err'][i, 0]), float(rec['err'][i, 1])] for i in frames},
            'lock_shift_m': [float(shift[0]), float(shift[1])], 'lock_shift_norm_m': float(np.hypot(*shift)),
            'template_ncc_at_lock_shift': {'mean': float(np.mean(ncc)), 'p10': percentile(ncc, 0.1), 'min': float(np.min(ncc)),
                                           'fraction_above_score_gate': float(np.mean(np.array(ncc) > nat.SCORE_GATE))},
            'texture_alias_peaks_acf_above_0p8_within_45m (dx, dy, acf)': [list(p) for p in peaks[:10]],
            'nearest_alias_peak_to_lock_shift': None if nearest is None else list(nearest)}


def make_plots(out_dir):
    """Two PNGs from the saved json/csv (Pillow via sim/nav/plotting.py): horizontal error against height, and the
    measured match error against the assumed model."""
    import plotting
    report = json.loads((out_dir/'image_in_loop.json').read_text(encoding='utf-8'))
    with (out_dir/'image_in_loop.csv').open(newline='', encoding='utf-8') as handle:
        rows = [{k: (v if k in ('variant', 'mode') else _num(v)) for k, v in r.items()} for r in csv.DictReader(handle)]
    lost = {(d['variant'], d['descent']) for d in report['analysis']['per_descent'] if d['lost_lock']}
    series = []
    for label, mode, color, style, keep in (('image in the loop (locked descents)', 'image', '#D55E00', 'line', True),
                                            ('geometry control, same descents', 'control', '#0072B2', 'dash', True),
                                            ('image in the loop, descent that locked onto a texture alias', 'image', '#4D4D4D', 'line', False)):
        sel = [r for r in rows if r['variant'] == 'base_moderate' and r['mode'] == mode and r['truth_height_m'] >= 78
               and (((r['variant'], int(r['descent'])) not in lost) == keep)
               and (keep or (r['variant'], int(r['descent'])) in lost)]
        if not sel:
            continue
        frames = sorted({int(r['frame']) for r in sel})
        height = [np.mean([r['truth_height_m'] for r in sel if int(r['frame']) == f]) for f in frames]
        error = [rms([r['horizontal_error_m'] for r in sel if int(r['frame']) == f]) for f in frames]
        series.append(dict(x=height, y=error, label=label, color=color, style=style))
    first = plotting.line_chart(out_dir/'image_in_loop_horizontal_error.png', series,
                                'Horizontal position error, moderate canopy', 'Height above ground (m)',
                                'RMS horizontal error over descents (m)', logy=True, invert_x=True, legend='bottom',
                                subtitle='SYNTHETIC. Flat procedural terrain, 5 locked descents and 1 that locked onto a texture alias',
                                size=(900, 560), hlines=[(LOST_M, '10 m', '#B22222')])
    series = []
    for name, color in (('base_moderate', '#D55E00'), ('noise_10gl', '#CC79A7'), ('fullres', '#009E73'), ('relief_2m', '#E69F00')):
        ex = report['experiments'].get(name)
        if not ex:
            continue
        table = [b for b in ex['modes']['image']['matcher']['by_height'] if b.get('pixel_error_rms_px_correct')]
        series.append(dict(x=[(b['height_lo_m'] + b['height_hi_m'])/2 for b in table],
                           y=[b['pixel_error_rms_px_correct'] for b in table], label='measured, ' + name,
                           color=color, style='line+points'))
    base = report['experiments']['base_moderate']['modes']['image']['matcher']['by_height']
    series.append(dict(x=[(b['height_lo_m'] + b['height_hi_m'])/2 for b in base], y=[b['assumed_sd_px_rms'] for b in base],
                       label='assumed by run_nav_mc (0.3 px + 0.15 m)', color='#0072B2', style='dash'))
    second = plotting.line_chart(out_dir/'image_in_loop_pixel_error.png', series, 'Pixel error of correct matches',
                                 'Height above ground (m)', 'RMS error per axis (px, full resolution)', logy=True,
                                 invert_x=True, legend='bottom', size=(900, 560),
                                 subtitle='SYNTHETIC. Strong, band-limited texture; real farmland will be worse')
    return [first, second]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plot', action='store_true', help='write the PNGs from an existing json/csv')
    parser.add_argument('--analyse', action='store_true', help='add the post-processing block to an existing json/csv, no simulation')
    parser.add_argument('--diagnose', type=int, default=-1, help='re-fly this base descent and explain where it ends; added to the json')
    parser.add_argument('--descents', type=int, default=0, help='override the base descent count (default 6)')
    parser.add_argument('--budget-min', type=float, default=22.0, help='total experiment time budget')
    parser.add_argument('--only', default='', help='comma list of experiment names')
    parser.add_argument('--quick', action='store_true', help='2 descents per experiment')
    parser.add_argument('--out', default='', help='output directory (default sim/nav/results)')
    parser.add_argument('--skip-sanity', action='store_true')
    args = parser.parse_args()
    clock = time.perf_counter()
    out_dir = Path(args.out) if args.out else OUT
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.plot:
        print('\n'.join(str(p) for p in make_plots(out_dir)))
        return
    if args.analyse or args.diagnose >= 0:
        path = out_dir/'image_in_loop.json'
        report = json.loads(path.read_text(encoding='utf-8'))
        if args.analyse:
            with (out_dir/'image_in_loop.csv').open(newline='', encoding='utf-8') as handle:
                report['analysis'] = nat.clean(analyse_rows(list(csv.DictReader(handle))))
            report['limits'] = LIMITS
            print(json.dumps(report['analysis'], indent=1)[:6000])
        if args.diagnose >= 0:
            report['lock_diagnosis'] = nat.clean(diagnose_lock(load_truth('moderate', 8, 'auto'), args.diagnose))
            print(json.dumps(report['lock_diagnosis'], indent=1))
        path.write_text(json.dumps(report, indent=1, allow_nan=False), encoding='utf-8')
        return
    only = [x for x in args.only.split(',') if x]
    experiments = [e for e in EXPERIMENTS if not only or e['name'] in only]
    for e in list(experiments):                   # a paired control needs the base run
        ref = e.get('control_from')
        if ref and ref not in [x['name'] for x in experiments]:
            experiments.insert(0, next(x for x in EXPERIMENTS if x['name'] == ref))
    print('SYNTHETIC closed-loop image-in-the-loop experiment; host timings are for a loaded shared PC', flush=True)
    sanity = []
    if not args.skip_sanity:
        for kw in ({'noise': 0.0}, {'noise': NOISE_GRAY}, {'noise': NOISE_GRAY, 'binning': 1, 'heights': (300.0, 120.0)}):
            x = perfect_pose_check(**kw)
            sanity.append(x)
            print('sanity (no pose error, binning %d, noise %.0f gl): %d landmarks, accepted %.3f, correct %.3f, pixel error '
                  'median %.3f max %.3f px, min score %.3f, filter-camera vs renderer %.1e px' % (
                      x['binning'], x['noise_gray'], x['landmarks'], x['accepted_fraction'], x['correct_fraction'],
                      x['pixel_error_median_px'], x['pixel_error_max_px'], x['min_score'],
                      x['filter_camera_vs_renderer_geometry_max_px']), flush=True)
    budget = args.budget_min*60.0 - (time.perf_counter() - clock)
    results, notes, csv_all, plan_log, _, _ = run_experiments(experiments, budget, args.descents, args.quick,
                                                              log=lambda s: print(s, flush=True))
    report = build_report(experiments, results, notes, sanity, plan_log, time.perf_counter() - clock, ' '.join(sys.argv))
    report['analysis'] = analyse_rows(csv_all)
    out_json = out_dir/'image_in_loop.json'
    out_json.write_text(json.dumps(nat.clean(report), indent=1, allow_nan=False), encoding='utf-8')
    nat.write_csv(out_dir/'image_in_loop.csv', nat.clean(csv_all))
    print_report(report)
    try:
        make_plots(out_dir)
    except (ImportError, OSError) as error:           # Pillow or fonts missing: the numbers are what matter
        print('plots skipped:', error)
    print('\nRuntime %.0f s; wrote %s and image_in_loop.csv' % (report['runtime_s'], out_json), flush=True)


if __name__ == '__main__':
    main()
