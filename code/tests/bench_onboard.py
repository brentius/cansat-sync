"""GPL-3.0-only. Times the onboard estimation code on whatever machine runs it.

Usage: python code/tests/bench_onboard.py [--json out.json]

Run it on the Raspberry Pi Zero 2 W (Raspberry Pi OS, python3-numpy) to get the
figures that the design documents still mark as unmeasured. On a development
laptop the numbers only show the relative cost of the parts. Each case is the
median of repeated calls on synthetic inputs; nothing here touches hardware.

Budgets come from the proposal's task table (section 16): navigation filter
propagated at 100 Hz and updated at 5 Hz, three landing predictors at 5 Hz.
The "share of one core" column is rate x median time.

The first-fix acquisition (cansat/coarse.py) is timed on a full-resolution
1456 x 1088 image of a synthetic patchwork terrain. It runs once per image
only until a fix is confirmed, so it is reported as a time per image and is
not part of the steady load. The proposal allows 5 s from canopy settling to
the first fix, and two images that agree are needed.
"""
import argparse
import json
import math
import platform
import statistics
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cansat.coarse import CoarseMap, Leveller, acquire  # noqa: E402
from cansat.eskf import LEVEL, N_ERR, Camera, Eskf, ImuNoise  # noqa: E402
from cansat.firstfix import solve_first_fix  # noqa: E402
from cansat.landing import DriftTable, LandingPredictor  # noqa: E402
from cansat.windprofile import WindModel  # noqa: E402

FOCAL = 2.5e-3/3.45e-6
IMAGE = (1456, 1088)


def median_time(fn, repeats):
    samples = []
    for _ in range(repeats):
        start = time.perf_counter()
        fn()
        samples.append(time.perf_counter() - start)
    return statistics.median(samples), max(samples)


def scene(rng, n):
    cam = Camera(FOCAL, FOCAL, IMAGE[0]/2, IMAGE[1]/2, IMAGE[0], IMAGE[1], lever=np.array([0.0, 0.0, 0.045]))
    p = np.array([5.0, -3.0, 200.0])
    pixels = np.column_stack((rng.uniform(60, IMAGE[0] - 60, n), rng.uniform(60, IMAGE[1] - 60, n)))
    rays = cam.ray(pixels) @ LEVEL.T
    depth = p[2]/-rays[:, 2]
    points = p + rays*depth[:, None]
    points[:, 2] = 0.0
    pixels, _ = cam.project(points, p, LEVEL)
    return cam, p, points, pixels + 0.3*rng.normal(size=pixels.shape)


def patchwork(rng, size=1280, cells=70):
    """Random fields with dark boundaries at 1 m/px, world origin at the centre. A timing stimulus only."""
    seeds = rng.uniform(0, size, (cells, 2))
    yy, xx = np.mgrid[0:size, 0:size].astype(np.float32)
    best = np.full((size, size), np.inf, dtype=np.float32)
    second = best.copy()
    label = np.zeros((size, size), dtype=np.int32)
    for i, (sx, sy) in enumerate(seeds):
        d = np.hypot(xx - np.float32(sx), yy - np.float32(sy))
        closer = d < best
        second = np.where(closer, best, np.minimum(second, d))
        label = np.where(closer, i, label)
        best = np.where(closer, d, best)
    raster = rng.uniform(80, 180, cells)[label] + 3.0*rng.standard_normal((size, size))
    raster[(second - best) < 4.0] = 50.0
    return raster, (-(size - 1)/2, -(size - 1)/2)


def view(raster, origin, cam, p, r):
    """What cam sees of the raster from IMU position p, attitude r (nearest-neighbour ray cast)."""
    u, v = np.meshgrid(np.arange(cam.width, dtype=float), np.arange(cam.height, dtype=float))
    rays = cam.ray(np.column_stack((u.ravel(), v.ravel()))) @ r.T
    centre = p + r @ cam.lever
    reach = centre[2]/-rays[:, 2]
    col = np.clip(np.rint(centre[0] + rays[:, 0]*reach - origin[0]).astype(int), 0, raster.shape[1] - 1)
    row = np.clip(np.rint(centre[1] + rays[:, 1]*reach - origin[1]).astype(int), 0, raster.shape[0] - 1)
    return raster[row, col].reshape(cam.height, cam.width)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--json', default='')
    args = parser.parse_args()
    rng = np.random.default_rng(1)
    rows = []

    def add(name, rate_hz, fn, repeats):
        med, worst = median_time(fn, repeats)
        rows.append({'case': name, 'rate_hz': rate_hz, 'median_ms': 1e3*med, 'worst_ms': 1e3*worst,
                     'core_share': rate_hz*med if rate_hz else None})

    cov = np.diag(np.r_[np.full(3, 9.0), np.full(3, 4.0), np.full(3, 3e-4), np.full(3, 0.04), np.full(3, 3e-6), 1.0, 4e-6])
    for landmarks in (8, 12, 30):
        cam, p, points, pixels = scene(rng, landmarks)
        state = {'f': None}

        def fresh():
            state['f'] = Eskf(p + rng.normal(0, 0.5, 3), [4.0, 1.0, -10.5], LEVEL, cov, ImuNoise(gyro_scale=0.005))

        def update():
            fresh()
            state['f'].update_landmarks(cam, points, pixels, 0.3**2 + (0.15*FOCAL/200.0)**2,
                                        np.array([0.9, 0.9, 10.0]), iterations=3, robust=True, recover=True)

        add('ESKF landmark update, %d landmarks' % landmarks, 5.0, update, 60)
    f = Eskf([0.0, 0.0, 200.0], [4.0, 1.0, -10.5], LEVEL, cov, ImuNoise(gyro_scale=0.005))
    accel, gyro = np.array([0.1, -0.2, -9.8]), np.array([0.2, -0.1, 0.8])
    add('ESKF propagate', 100.0, lambda: f.propagate(0.01, accel, gyro), 2000)
    add('ESKF barometer update', 10.0, lambda: f.update_baro(float(f.p[2]), 0.15), 300)
    cam, p, points, pixels = scene(rng, 16)
    add('first fix, 16 matches', 0.0, lambda: solve_first_fix(cam, pixels, points, 200.0, 0.5), 30)

    model = WindModel()
    add('drift table build (once, on the pad)', 0.0, lambda: DriftTable(model, 10.5), 3)
    for label, height in (('at 250 m', 250.0), ('at 20 m', 20.0), ('at 4 m', 4.0)):
        predictor = LandingPredictor(model, 10.5)
        state = {'t': 0.0, 'z': height}

        def step():
            state['t'] += 0.2
            state['z'] = max(state['z'] - 2.1, height - 8.0) if state['z'] > height - 8.0 else height
            predictor.update(state['t'], [0.0, 0.0, state['z']], [4.0, 1.0], 0.04)

        add('landing predictor update ' + label, 15.0, step, 200)

    # First-fix acquisition: one full-resolution image, heading unknown and then known to +/-15 degrees.
    raster, origin = patchwork(rng)
    camera = Camera(FOCAL, FOCAL, IMAGE[0]/2, IMAGE[1]/2, IMAGE[0], IMAGE[1], lever=np.array([0.0, 0.0, 0.045]))
    pose = np.array([40.0, -25.0, 290.0])
    yaw = 2.2
    turn = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    image = view(raster, origin, camera, pose, turn @ LEVEL)
    made = {}
    add('coarse map build (once, on the pad)', 0.0, lambda: made.update(map=CoarseMap(raster, 1.0, origin)), 2)
    result = {}
    add('first fix: one image, heading unknown', 0.0,
        lambda: result.update(whole=acquire(image, camera, LEVEL, pose[2], made['map'])), 5)
    add('first fix: one image, compass to 15 deg', 0.0,
        lambda: result.update(narrow=acquire(image, camera, LEVEL, pose[2], made['map'], yaw_prior=(yaw + 0.1, math.radians(15.0)))), 5)
    leveller = Leveller()
    add('leveller update', 100.0, lambda: leveller.update(0.01, accel, gyro), 2000)
    acquisition = {key: {'ok': bool(a.ok), 'reason': a.reason, 'ffts': a.ffts, 'seconds': a.seconds,
                         'error_m': float(np.hypot(*(a.fix.p - pose)[:2])) if a.ok else None}
                   for key, a in result.items()}

    report = {'label': 'timing on the machine named here, not a flight measurement unless that machine is the Pi',
              'acquisition': acquisition,
              'machine': platform.machine(), 'processor': platform.processor(), 'platform': platform.platform(),
              'python': platform.python_version(), 'numpy': np.__version__, 'error_states': N_ERR, 'cases': rows}
    print('%s | %s | Python %s | NumPy %s' % (report['machine'], report['platform'], report['python'], report['numpy']))
    print('%-44s %8s %10s %10s %10s' % ('case', 'rate Hz', 'median ms', 'worst ms', 'core share'))
    for r in rows:
        share = '%9.1f%%' % (100*r['core_share']) if r['core_share'] is not None else '         -'
        print('%-44s %8s %10.3f %10.3f %s' % (r['case'], ('%g' % r['rate_hz']) if r['rate_hz'] else 'once',
                                              r['median_ms'], r['worst_ms'], share))
    total = sum(r['core_share'] for r in rows if r['core_share'] is not None and '30 landmarks' not in r['case']
                and '8 landmarks' not in r['case'] and 'at 20 m' not in r['case'] and 'at 4 m' not in r['case'])
    print('steady load above 30 m with 12 landmarks: %.1f%% of one core (image matching not included)' % (100*total))
    for key, a in acquisition.items():
        print('first fix (%s): %s, FFTs by size %s, stages %s' % (
            key, 'error %.1f m' % a['error_m'] if a['ok'] else a['reason'], a['ffts'],
            {k: round(v, 3) for k, v in a['seconds'].items()}))
    report['steady_core_share_12_landmarks'] = total
    if args.json:
        Path(args.json).write_text(json.dumps(report, indent=1))
    return 0 if math.isfinite(total) else 1


if __name__ == '__main__':
    sys.exit(main())
