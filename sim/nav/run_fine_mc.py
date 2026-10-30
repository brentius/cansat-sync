"""Rendered image-to-filter Monte Carlo for the candidate fine landmark matcher.

This is a synthetic host experiment. Truth is used only to create sensor/image
inputs and to score outputs; FineMatcher receives the map, image and navigation
estimate. Host timings are diagnostic and say nothing about Pi throughput.
"""
import argparse
import csv
import json
import math
import os
import sys
import time
import platform
from dataclasses import asdict
from functools import lru_cache
from pathlib import Path

for _thread_env in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_thread_env, '1')

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / 'code'))
import farmland  # noqa: E402
from cansat.eskf import ATT, LEVEL, N_ERR, POS, VEL, Camera, Eskf, ImuNoise, exp_so3  # noqa: E402
from cansat.fine import FineConfig, FineMatcher  # noqa: E402
from cansat.coarse import bin_camera  # noqa: E402
from scene import Pyramid, render  # noqa: E402

OUT = HERE / 'results' / 'fine_tracking_20261023'
IMAGE_SIZE = (272, 364)
FOCAL_PX = 724.6 / 4.0
FRAME_DT = 0.2
DESCENT_RATE_MPS = 8.0
SCENARIOS = ('fresh', 'seasonal', 'near_capture', 'outside_capture', 'textureless')


def _finite(value):
    """Convert NumPy values to strict-JSON-safe Python values."""
    if isinstance(value, dict):
        return {str(k): _finite(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_finite(v) for v in value]
    if isinstance(value, np.ndarray):
        return _finite(value.tolist())
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    return value


def score_pixel_errors(errors):
    """Score Euclidean residuals in original image pixels for accepted matches only."""
    e = np.asarray(errors, dtype=float).reshape(-1, 2)
    if not len(e):
        return {'count': 0, 'rmse_px': None, 'median_px': None, 'p95_px': None,
                'false_match_threshold_px': 3.0, 'false_matches_over_threshold': 0, 'false_match_rate': None}
    norm = np.linalg.norm(e, axis=1)
    false = int(np.count_nonzero(norm > 3.0))
    return {'count': int(len(norm)), 'rmse_px': float(np.sqrt(np.mean(norm**2))),
            'median_px': float(np.median(norm)), 'p95_px': float(np.percentile(norm, 95)),
            'false_match_threshold_px': 3.0, 'false_matches_over_threshold': false,
            'false_match_rate': float(false / len(norm))}


def score_ground_errors(errors, threshold_m=0.75):
    """Score accepted candidate ground-coordinate residuals and false matches."""
    e = np.asarray(errors, dtype=float).reshape(-1, 2)
    if not len(e):
        return {'count': 0, 'rmse_m': None, 'median_m': None, 'p95_m': None,
                'false_match_threshold_m': float(threshold_m), 'false_matches_over_threshold': 0,
                'false_match_rate': None}
    norm = np.linalg.norm(e, axis=1)
    false = int(np.count_nonzero(norm > threshold_m))
    return {'count': int(len(norm)), 'rmse_m': float(np.sqrt(np.mean(norm**2))),
            'median_m': float(np.median(norm)), 'p95_m': float(np.percentile(norm, 95)),
            'false_match_threshold_m': float(threshold_m), 'false_matches_over_threshold': false,
            'false_match_rate': float(false / len(norm))}


def _paired_trial_seed(seed, trial):
    """Common sensor/navigation and map seed shared across all scenarios in one trial."""
    return int((seed + trial * 1009) % (2**32))


@lru_cache(maxsize=16)
def _world(seed, seasonal):
    season = farmland.Season(albedo_change=0.95, tramline_change=1.0) if seasonal else None
    return farmland.generate(seed, size_m=800.0, res_m=0.5, style='mixed', season=season)


def _camera():
    full = Camera(724.6, 724.6, 728.0, 544.0, 1456, 1088,
                  lever=np.array([0.0, 0.0, 0.045]))
    return bin_camera(full, 4)


def _initial_filter(rng, scenario):
    p0 = np.array([0.0, 0.0, 300.0])
    baseline_xy = rng.normal(0.0, 1.5, 2)
    yaw = rng.normal(0.0, math.radians(0.6))
    tilt = rng.normal(0.0, math.radians(0.35), 2)
    p0[:2] += baseline_xy
    if scenario == 'near_capture':
        p0[:2] = rng.uniform(-8.0, 8.0, 2)
    elif scenario == 'outside_capture':
        # Well outside the 16 m map-plane reach of the 32 px search at this scale.
        p0[:2] += np.array([140.0, -120.0])
    r0 = LEVEL @ exp_so3(np.array([tilt[0], tilt[1], yaw]))
    v0 = np.array([0.0, 0.0, -8.0])
    sd = np.array([4.0, 4.0, 2.0, 1.5, 1.5, 0.5, math.radians(2), math.radians(2), math.radians(1.5),
                   0.1, 0.1, 0.1, math.radians(0.2), math.radians(0.2), math.radians(0.2), 1.0, 0.01])
    return Eskf(p0, v0, r0, np.diag(sd**2), noise=ImuNoise(accel=0.01, gyro=math.radians(0.03),
                                                            accel_bias=1e-4, gyro_bias=1e-5,
                                                            baro_bias=0.02, time_offset=1e-4))


def _propagate_elapsed(nav, control, frame_i):
    """Advance both paired filters over the 200 ms preceding frames after t=0."""
    if frame_i == 0:
        return
    accel = np.array([0.0, 0.0, -9.80665])
    gyro = np.zeros(3)
    for _ in range(20):
        nav.propagate(0.01, accel, gyro)
        control.propagate(0.01, accel, gyro)


def _trial(scenario, trial_seed, map_seed, frames, trial_index):
    sensor_seed = int((trial_seed ^ 0xA51E) % (2**32))
    sensor_rng = np.random.default_rng(sensor_seed)
    world = _world(map_seed, scenario == 'seasonal')
    flight = world.flight if scenario == 'seasonal' else world.map
    if scenario == 'textureless':
        flight = np.full_like(flight, float(np.mean(flight)))
    map_pyramid = Pyramid(world.map, world.res, world.origin)
    flight_pyramid = Pyramid(flight, world.res, world.origin)
    camera = _camera()
    config = FineConfig()
    matcher = FineMatcher(world.map, world.res, world.origin, config=config)
    nav = _initial_filter(np.random.default_rng(trial_seed), scenario)
    control = _initial_filter(np.random.default_rng(trial_seed), scenario)
    errors, ground_errors, rows = [], [], []
    horizontal_after = []
    horizontal_control = []
    start = time.perf_counter()
    for frame_i in range(frames):
        z = max(80.0, 300.0 - DESCENT_RATE_MPS * frame_i * FRAME_DT)
        truth_p = np.array([0.35 * frame_i * FRAME_DT, -0.18 * frame_i * FRAME_DT, z])
        truth_r = LEVEL
        # Simulated 200 ms inertial propagation, with a 10 Hz barometric update.
        # The first frame is at t=0; no sensor interval elapses before exposure.
        _propagate_elapsed(nav, control, frame_i)
        baro = float(z + sensor_rng.normal(0.0, 0.25))
        nav.update_baro(baro, 0.25)
        control.update_baro(baro, 0.25)
        before = nav.p.copy()
        control_error = float(np.linalg.norm(control.p - truth_p))
        image, coverage = render(flight_pyramid, camera, truth_p, truth_r, IMAGE_SIZE,
                                 blur_px=0.35, noise=1.0, rng=sensor_rng, quantise=True)
        match_start = time.perf_counter()
        result = matcher.match_filter(image, camera, nav)
        match_time = time.perf_counter() - match_start
        truth_pixels, depth = camera.project(result.points, truth_p, truth_r)
        valid = depth > 0
        pixel_errors = result.pixels[valid] - truth_pixels[valid]
        errors.extend(pixel_errors.tolist())
        if np.any(valid):
            ray_world = camera.ray(result.pixels[valid]) @ truth_r.T
            camera_centre = truth_p + truth_r @ camera.lever
            ray_depth = -camera_centre[2] / ray_world[:, 2]
            ray_ground = camera_centre[:2] + ray_world[:, :2] * ray_depth[:, None]
            ge = ray_ground - result.points[valid, :2]
            ground_errors.extend(ge.tolist())
        else:
            ge = np.empty((0, 2))
        filter_before = float(np.linalg.norm(before - truth_p))
        if len(result.points):
            filter_info = nav.update_landmarks(camera, result.points, result.pixels, result.pixel_var,
                                               map_var=result.map_var, iterations=3, robust=True, recover=True)
            rows_filter_accepted = int(np.count_nonzero(filter_info['accepted']))
        else:
            rows_filter_accepted = 0
        filter_after = float(np.linalg.norm(nav.p - truth_p))
        rows.append({
            'scenario': scenario, 'trial_seed': int(trial_seed), 'sensor_seed': sensor_seed,
            'map_seed': int(map_seed), 'trial': int(trial_index),
            'frame': frame_i, 'height_true_m': float(z), 'baro_measurement_m': baro, 'coverage': coverage,
            'matches_attempted': int(result.attempted), 'matches_accepted': int(len(result.points)),
            'rejections': result.rejections, 'reason': result.reason,
            'pixel_rmse_px': float(np.sqrt(np.mean(np.sum(pixel_errors**2, axis=1)))) if pixel_errors.size else None,
            'ground_rmse_m': float(np.sqrt(np.mean(np.sum(ge**2, axis=1)))) if ge.size else None,
            'ground_false_matches_over_0_75m': int(np.count_nonzero(np.linalg.norm(ge, axis=1) > 0.75)) if ge.size else 0,
            'filter_error_before_m': filter_before, 'filter_error_after_m': filter_after,
            'filter_horizontal_error_after_m': float(np.linalg.norm(nav.p[:2] - truth_p[:2])),
            'no_image_control_error_m': control_error,
            'no_image_control_horizontal_error_m': float(np.linalg.norm(control.p[:2] - truth_p[:2])),
            'filter_matches_accepted': 0, 'filter_matches_rejected': int(len(result.points)),
            'match_host_seconds': match_time,
        })
        rows[-1]['filter_matches_accepted'] = rows_filter_accepted
        rows[-1]['filter_matches_rejected'] = int(len(result.points) - rows_filter_accepted)
        horizontal_after.append(rows[-1]['filter_horizontal_error_after_m'])
        horizontal_control.append(rows[-1]['no_image_control_horizontal_error_m'])
    total = time.perf_counter() - start
    return rows, errors, ground_errors, {'total_host_seconds': total, 'mean_match_host_seconds': float(np.mean([r['match_host_seconds'] for r in rows])) if rows else None,
                          'mean_filter_error_before_m': float(np.mean([r['filter_error_before_m'] for r in rows])) if rows else None,
                          'mean_filter_error_after_m': float(np.mean([r['filter_error_after_m'] for r in rows])) if rows else None,
                          'mean_filter_horizontal_after_m': float(np.sqrt(np.mean(np.square(horizontal_after)))) if horizontal_after else None,
                          'endpoint_filter_horizontal_error_m': float(horizontal_after[-1]) if horizontal_after else None,
                          'endpoint_control_horizontal_error_m': float(horizontal_control[-1]) if horizontal_control else None,
                          'mean_no_image_control_error_m': float(np.mean([r['no_image_control_error_m'] for r in rows])) if rows else None,
                          'mean_no_image_control_horizontal_m': float(np.sqrt(np.mean(np.square(horizontal_control)))) if horizontal_control else None}


def run_campaign(trials=1, frames=20, seed=20261023, output=OUT):
    """Run reproducible fixed-seed trials and persist a CSV plus strict JSON summary."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    all_rows, summary, timings, trial_results = [], {}, {}, {}
    for scenario in SCENARIOS:
        scenario_errors, scenario_ground_errors, scenario_rows = [], [], []
        scenario_times = []
        scenario_trial_results = []
        for trial in range(trials):
            paired_seed = _paired_trial_seed(seed, trial)
            map_seed = paired_seed ^ 0x5A17
            rows, errors, ground_errors, stats = _trial(scenario, paired_seed, map_seed, frames, trial)
            all_rows.extend(rows)
            scenario_rows.extend(rows)
            scenario_errors.extend(errors)
            scenario_ground_errors.extend(ground_errors)
            scenario_times.append(stats)
            scenario_trial_results.append({
                'trial': trial, 'trial_seed': paired_seed, 'sensor_seed': paired_seed ^ 0xA51E,
                'map_seed': map_seed,
                'accepted_matches': int(sum(r['matches_accepted'] for r in rows)),
                'filter_matches_accepted': int(sum(r['filter_matches_accepted'] for r in rows)),
                'filter_horizontal_rms_m': float(np.sqrt(np.mean([r['filter_horizontal_error_after_m']**2 for r in rows]))),
                'filter_endpoint_horizontal_error_m': stats['endpoint_filter_horizontal_error_m'],
                'no_image_horizontal_rms_m': float(np.sqrt(np.mean([r['no_image_control_horizontal_error_m']**2 for r in rows]))),
                'no_image_endpoint_horizontal_error_m': stats['endpoint_control_horizontal_error_m'],
            })
        summary[scenario] = {
            'accepted_matches': int(sum(r['matches_accepted'] for r in scenario_rows)),
            'attempted_sites': int(sum(r['matches_attempted'] for r in scenario_rows)),
            'pixel_error_accepted_only': score_pixel_errors(scenario_errors),
            'ground_error_accepted_only': score_ground_errors(scenario_ground_errors),
            'mean_filter_error_before_m': float(np.mean([r['filter_error_before_m'] for r in scenario_rows])) if scenario_rows else None,
            'mean_filter_error_after_m': float(np.mean([r['filter_error_after_m'] for r in scenario_rows])) if scenario_rows else None,
            'filter_matches_accepted': int(sum(r['filter_matches_accepted'] for r in scenario_rows)),
            'filter_matches_rejected': int(sum(r['filter_matches_rejected'] for r in scenario_rows)),
            'horizontal_filter_rms_after_m': float(np.sqrt(np.mean([r['filter_horizontal_error_after_m']**2 for r in scenario_rows]))) if scenario_rows else None,
            'horizontal_filter_endpoint_rms_m': float(np.sqrt(np.mean([t['endpoint_filter_horizontal_error_m']**2 for t in scenario_times]))) if scenario_times else None,
            'horizontal_filter_endpoint_within_10m_fraction': float(np.mean([t['endpoint_filter_horizontal_error_m'] <= 10.0 for t in scenario_times])) if scenario_times else None,
            'mean_no_image_control_error_m': float(np.mean([r['no_image_control_error_m'] for r in scenario_rows])) if scenario_rows else None,
            'horizontal_no_image_control_rms_m': float(np.sqrt(np.mean([r['no_image_control_horizontal_error_m']**2 for r in scenario_rows]))) if scenario_rows else None,
            'horizontal_control_endpoint_rms_m': float(np.sqrt(np.mean([t['endpoint_control_horizontal_error_m']**2 for t in scenario_times]))) if scenario_times else None,
            'rejections': _sum_rejections(scenario_rows),
        }
        timings[scenario] = scenario_times
        trial_results[scenario] = scenario_trial_results
    fieldnames = list(all_rows[0]) if all_rows else []
    # CSV stores the rejection dict as JSON to preserve all categories.
    with (output / 'frames.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in all_rows:
            writer.writerow({**row, 'rejections': json.dumps(row['rejections'], sort_keys=True)})
    payload = {'schema_version': 1, 'kind': 'synthetic_rendered_image_to_filter_monte_carlo',
               'timings_kind': 'host wall clock; not Raspberry Pi timing', 'seed': int(seed),
               'trials': int(trials), 'frames_per_trial': int(frames), 'scenario_count': len(SCENARIOS),
               'scenarios': list(SCENARIOS), 'frame_interval_s': FRAME_DT,
               'descent_rate_mps': DESCENT_RATE_MPS, 'image_size_px': list(IMAGE_SIZE),
               'camera': {'fx_px': FOCAL_PX, 'fy_px': FOCAL_PX, 'cx_px': 181.625,
                          'cy_px': 135.625, 'quarter_sensor_of': [1456, 1088],
                          'binning_factor': 4,
                          'lever_body_m': [0.0, 0.0, 0.045]},
               'map_resolution_m_per_px': 0.5, 'map_source': 'procedural farmland.generate',
               'sensor_model': 'ideal IMU at 100 Hz on straight constant-rate descent; 0.25 m sigma barometer at 5 Hz; no canopy motion',
               'motion_model': {'initial_height_m': 300.0, 'vertical_speed_mps': -DESCENT_RATE_MPS,
                                'horizontal_speed_mps': [0.35, -0.18], 'attitude': 'fixed level',
                                'initial_position_error': '1.5 m sigma horizontal for ordinary scenarios; up to 8 m each axis for near_capture; [140,-120] m for outside_capture',
                                'initial_attitude_error': '0.6 deg sigma yaw and 0.35 deg sigma roll/pitch'},
               'measurement_scoring': {'false_match_ground_threshold_m': 0.75,
                                       'false_match_pixel_threshold_px': 3.0,
                                       'ground_distance_method': 'intersect observed pixel ray at truth pose with flat z=0 and compare to candidate map point'},
               'software': {'python': platform.python_version(), 'numpy': np.__version__},
               'fine_config': asdict(FineConfig()),
               'truth_separation': 'truth used for rendering and scoring only; matcher sees image, map and nav estimate',
               'matcher_gates_changed_after_scoring': False, 'results': summary, 'trial_results': trial_results,
               'host_timings': timings}
    with (output / 'summary.json').open('w', encoding='utf-8') as f:
        json.dump(_finite(payload), f, indent=2, allow_nan=False)
        f.write('\n')
    return {'summary': summary, 'frames': all_rows, 'timings': timings, 'path': str(output)}


def _sum_rejections(rows):
    total = {}
    for row in rows:
        for key, value in (row['rejections'] or {}).items():
            total[key] = total.get(key, 0) + int(value)
    return total


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=int, default=1)
    parser.add_argument('--frames', type=int, default=139)
    parser.add_argument('--seed', type=int, default=20261023)
    parser.add_argument('--output', type=Path, default=OUT)
    args = parser.parse_args(argv)
    if args.trials < 1 or args.frames < 1:
        parser.error('--trials and --frames must be positive')
    result = run_campaign(args.trials, args.frames, args.seed, args.output)
    print(f"Wrote {result['path']} (synthetic; host timing only)")
    return result


if __name__ == '__main__':
    main()
