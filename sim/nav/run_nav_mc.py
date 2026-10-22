"""GPL-3.0-only. Closed-loop Monte Carlo of the navigation filter on synthetic descents.

Usage: python sim/nav/run_nav_mc.py [--descents N] [--workers W] [--only a,b] [--list]

Each scenario pairs a simulated descent class (sim/nav/truth.py) with a sensor
error model and a filter configuration, runs the flight filter
(code/cansat/eskf.py) on every descent and scores it against the simulated
truth. Truth batches are cached under sim/nav/generated/ (git-ignored); results
go to sim/nav/results/nav_mc/.

Everything here is SYNTHETIC. Landmark measurements are generated from
geometry plus an error model; no images are rendered or matched in this file.
Input values carry the provenance tags used in sim/systems/run_sizing.py:
SOURCED (datasheet, named), PROPOSAL, ASSUMED.
"""
import argparse
import hashlib
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]/'code'))
import truth as tr  # noqa: E402
from cansat.eskf import (ATT, BA, BARO, BG, GATE_2DOF, N_ERR, POS, TIME, VEL, Camera, Eskf,  # noqa: E402
                         ImuNoise, exp_so3)
from cansat.firstfix import initial_covariance, solve_first_fix  # noqa: E402

OUT = HERE/'results'/'nav_mc'
CACHE = HERE/'generated'
SEED = 20261008
NAV_DT = 0.01
FRAME_DT = 0.2                                   # PROPOSAL: 5 Hz imagery
DURATION = 28.0
DELAYS = (0.0, 0.001, 0.002, 0.005, 0.010, 0.020)
# SOURCED: Sony IMX296, 1456 x 1088 px, 3.45 um pixels (Raspberry Pi Global
# Shutter Camera). PROVISIONAL lens: 2.5 mm, treated as an ideal pinhole after
# undistortion, so f = 2.5 mm / 3.45 um = 724.6 px.
IMAGE = (1456, 1088)
FOCAL = 2.5e-3/3.45e-6
CAMERA_LEVER = (0.0, 0.0, 0.045)                 # ASSUMED: camera 45 mm below the IMU
DEG = math.pi/180

# Canopy classes. The normal-force law is a generic shape, see truth.py. What
# each does was measured in sim/nav/characterise_descent.py (48 descents):
#   benign    restoring canopy: swing 4 deg RMS, no sustained glide
#   moderate  unstable below 12 deg: swing 6 deg RMS (23 max), glides at 1.9 m/s
#   severe    unstable below 25 deg: swing 5 deg RMS, glides at 4.4 m/s
#   violent   weakly restoring canopy shaken by a random side force: swing
#             19 deg RMS (53 max) at 0.9 Hz, body rates 100 deg/s RMS. This is
#             the only class with swing as large as published drop tests of
#             round canopies (22-27 deg average, Tutt AIAA 2013-1384).
CANOPIES = {
    'benign': tr.Params(),
    'moderate': replace(tr.Params(), cn_k=0.4, trim_deg=12.0),
    'severe': replace(tr.Params(), cn_k=0.5, trim_deg=25.0),
    'violent': replace(tr.Params(), cn_k=0.1, trim_deg=0.0, side_force_sd=0.05),
}


@dataclass
class Sensors:
    """What the simulated hardware actually does (the filter is not told)."""
    # SOURCED: Bosch BMI088 datasheet typical noise densities.
    gyro_noise: float = 0.014*DEG                # rad/s/sqrt(Hz)
    accel_noise: float = 175e-6*tr.G0            # m/s^2/sqrt(Hz)
    # ASSUMED residual after averaging on the pad; the datasheet zero-rate
    # offset is +/-1 dps typical before any calibration.
    gyro_bias_sd: float = 0.1*DEG                # rad/s
    accel_bias_sd: float = 0.020*tr.G0           # SOURCED: BMI088 zero-g offset 20 mg typical
    gyro_bias_walk: float = 2.0e-6               # rad/s^2/sqrt(Hz)
    accel_bias_walk: float = 1.0e-4              # m/s^3/sqrt(Hz)
    # ASSUMED after a turntable calibration. SOURCED uncalibrated values are
    # +/-1 % sensitivity tolerance and +/-1 % cross-axis sensitivity, typical.
    gyro_scale_sd: float = 0.003
    gyro_cross_sd: float = 0.002
    accel_scale_sd: float = 0.003
    imu_mode: str = 'ideal'                      # ideal | mean | inst (see truth.simulate)
    baro_noise: float = 0.15                     # m per 10 Hz sample; MS5611 at OSR 4096 is ~0.1 m
    baro_bias_sd: float = 1.0                    # m, pad drift and port position error (ASSUMED)
    baro_bias_walk: float = 0.02                 # m/sqrt(s)
    camera_delay: float = 0.001                  # s, constant; must be one of DELAYS
    camera_jitter: tuple = ()                    # if set, each image's delay is drawn from these DELAYS
    pixel_noise: float = 0.3                     # px floor: image noise, blur, undistortion residual
    ground_noise: float = 0.15                   # m, correlation-peak precision on the ground
    map_xy_sd: float = 0.3                       # m, each landmark's own map error, fixed for the descent
    relief_sd: float = 1.0                       # m, each landmark's unmodelled height, fixed
    datum: tuple = (0.0, 0.0)                    # m, rigid map offset
    outlier_rate: float = 0.05
    outlier_range: float = 16.0                  # m, the fine-mode search half-width
    alias_count: int = 0                         # landmarks per frame sharing one false shift
    alias_shift: tuple = (8.0, 0.0)              # m
    landmark_spacing: float = 40.0               # m, jittered grid of usable map landmarks
    max_landmarks: int = 30
    # A landmark is usable only if the matcher's whole rectified patch (the
    # +/-16 m search plus half the 16 m template) is inside the image. The
    # image-in-the-loop run showed that ignoring this overstates the landmark
    # count low down (13 offered against 5 usable at 80-100 m).
    search_margin_m: float = 24.0
    fov_fraction: float = 1.0                    # central fraction of the image used
    focal_error: float = 0.001                   # fractional, unmodelled
    principal_error: float = 0.5                 # px, unmodelled
    misalignment: float = 0.05*DEG               # rad per axis, unmodelled camera-to-IMU residual
    frame_dt: float = FRAME_DT                   # s, a multiple of 0.2
    # Coarse matches offered to the first-fix solver (used only when the
    # filter is started from it). ASSUMED: 0.3 px of a 2 m/px map on the
    # ground, and a third of the matches wrong anywhere in the image.
    first_fix_matches: int = 16
    first_fix_ground_noise: float = 0.6          # m
    first_fix_outliers: float = 0.30
    dropout: tuple = ()                          # (start_s, end_s) with no accepted imagery


@dataclass
class FilterCfg:
    """What the filter assumes."""
    gyro_noise: float = 0.014*DEG
    accel_noise: float = 175e-6*tr.G0
    gyro_bias_walk: float = 2.0e-6
    accel_bias_walk: float = 1.0e-4
    baro_bias_walk: float = 0.02
    noise_scale: float = 1.0                     # multiplies the two white densities
    gyro_scale: float = 0.005                    # scale and cross-axis allowance, see ImuNoise
    pixel_noise: float = 0.3
    ground_noise: float = 0.15
    map_var: tuple = (0.3**2, 0.3**2, 1.0**2)    # m^2, or None
    map_inflation: float = 10.0                  # multiplies map_var: a landmark's map error repeats every frame
    iterations: int = 3                          # iterated update; 1 is the plain EKF of the proposal
    robust: bool = True                          # post-fit residual rejection
    recover: bool = True                         # widen the prior when most landmarks disagree with it
    gate: float = GATE_2DOF
    baro_sigma: float = 0.15
    map_floor: float = 80.0                      # PROPOSAL: no map matching below ~80 m
    # Initial 1-sigma: a first fix has just been accepted (ASSUMED quality).
    init_pos: tuple = (3.0, 3.0, 3.0)
    init_vel: tuple = (2.0, 2.0, 1.0)
    init_att: tuple = (1.0*DEG, 1.0*DEG, 2.0*DEG)
    init_accel_bias: float = 0.020*tr.G0
    init_gyro_bias: float = 0.1*DEG
    init_baro_bias: float = 1.0
    init_time_offset: float = 0.002              # s; 1e-7 switches the time-offset state off
    # 'drawn': start from truth plus a draw of the sigmas above. 'first_fix':
    # start from code/cansat/firstfix.py on the first image that yields a fix,
    # with yaw unknown and velocity taken as straight down at the design rate.
    start: str = 'drawn'
    first_fix_vel_sigma: tuple = (6.0, 6.0, 1.5)


def scenario(canopy='moderate', sensors=None, filt=None, note=''):
    return {'canopy': canopy, 'sensors': replace(Sensors(), **(sensors or {})),
            'filter': replace(FilterCfg(), **(filt or {})), 'note': note}


MATCHED = dict(gyro_scale_sd=0.0, gyro_cross_sd=0.0, accel_scale_sd=0.0, camera_delay=0.0, map_xy_sd=0.0,
               relief_sd=0.0, outlier_rate=0.0, focal_error=0.0, principal_error=0.0, misalignment=0.0)
OFF = 1e-7
# The filter with none of the additions: what the proposal describes, iterated or not.
PLAIN = dict(map_var=None, map_inflation=1.0, robust=False, recover=False, gyro_scale=0.0, init_time_offset=OFF)
SINGLE = dict(PLAIN, iterations=1)
FIX_POOR = dict(init_pos=(15.0, 15.0, 5.0), init_vel=(5.0, 5.0, 2.0), init_att=(4*DEG, 4*DEG, 6*DEG))
UNCALIBRATED = dict(gyro_scale_sd=0.01, gyro_cross_sd=0.01, gyro_bias_sd=1.0*DEG)

SCENARIOS = {
    # --- Verification: every error the simulation makes is one the filter models.
    'matched_benign': scenario('benign', MATCHED, PLAIN, 'filter model equals simulation, iterated update'),
    'matched_moderate': scenario('moderate', MATCHED, PLAIN, 'filter model equals simulation, iterated update'),
    'matched_severe': scenario('severe', MATCHED, PLAIN, 'filter model equals simulation, iterated update'),
    'matched_violent': scenario('violent', MATCHED, PLAIN, 'filter model equals simulation, iterated update'),
    'matched_single_benign': scenario('benign', MATCHED, SINGLE, 'the same with one first-order update per image'),
    'matched_single_moderate': scenario('moderate', MATCHED, SINGLE, 'the same with one first-order update per image'),
    'matched_single_severe': scenario('severe', MATCHED, SINGLE, 'the same with one first-order update per image'),
    'matched_single_violent': scenario('violent', MATCHED, SINGLE, 'the same with one first-order update per image'),
    'matched_single_tight_start': scenario('moderate', MATCHED, dict(
        SINGLE, init_pos=(0.3, 0.3, 0.3), init_vel=(0.2, 0.2, 0.1), init_att=(0.1*DEG, 0.1*DEG, 0.2*DEG)),
        'single update started inside the linear regime'),
    'matched_time_state': scenario('moderate', dict(MATCHED, camera_delay=0.002), dict(PLAIN, init_time_offset=0.002),
                                   '2 ms delay drawn at the 1 sigma of the offset prior'),
    'matched_full_filter': scenario('moderate', MATCHED, dict(map_var=None, gyro_scale=0.0, init_time_offset=OFF),
                                    'matched model with post-fit rejection and recovery switched on'),
    # --- Fragility of the plain filter: one unmodelled error at a time.
    'plain_gyro_scale': scenario(sensors=dict(MATCHED, gyro_scale_sd=0.003), filt=PLAIN),
    'plain_gyro_cross_axis': scenario(sensors=dict(MATCHED, gyro_cross_sd=0.002), filt=PLAIN),
    'plain_accel_scale': scenario(sensors=dict(MATCHED, accel_scale_sd=0.003), filt=PLAIN),
    'plain_delay_1ms': scenario(sensors=dict(MATCHED, camera_delay=0.001), filt=PLAIN),
    'plain_calibration': scenario(sensors=dict(MATCHED, focal_error=0.001, principal_error=0.5,
                                               misalignment=0.05*DEG), filt=PLAIN),
    'plain_map_xy': scenario(sensors=dict(MATCHED, map_xy_sd=0.3), filt=dict(PLAIN, map_var=(0.09, 0.09, 0.0))),
    'plain_relief': scenario(sensors=dict(MATCHED, relief_sd=1.0), filt=dict(PLAIN, map_var=(0.0, 0.0, 1.0))),
    'plain_outliers_5pct': scenario(sensors=dict(MATCHED, outlier_rate=0.05), filt=PLAIN),
    'plain_all_errors': scenario(filt=dict(PLAIN, map_var=(0.09, 0.09, 1.0)), note='baseline errors, iterated update only'),
    'proposal_all_errors': scenario(filt=dict(SINGLE, map_var=(0.09, 0.09, 1.0)),
                                    note='baseline errors, single update and gate as the proposal describes'),
    # --- The full filter with every baseline error on.
    'nominal_benign': scenario('benign', note='baseline errors, stable canopy'),
    'nominal_moderate': scenario('moderate', note='baseline errors, oscillating canopy'),
    'nominal_severe': scenario('severe', note='baseline errors, strongly unstable canopy'),
    'nominal_violent': scenario('violent', note='baseline errors, 19 deg RMS swing'),
    'proposal_all_errors_violent': scenario('violent', filt=dict(SINGLE, map_var=(0.09, 0.09, 1.0))),
    # The violent class under the stresses that interact with swing.
    'violent_without_scale_noise': scenario('violent', filt=dict(gyro_scale=0.0)),
    'violent_without_time_state': scenario('violent', filt=dict(init_time_offset=OFF)),
    'violent_delay_10ms': scenario('violent', dict(camera_delay=0.010), dict(init_time_offset=0.010)),
    'violent_delay_10ms_no_time_state': scenario('violent', dict(camera_delay=0.010), dict(init_time_offset=OFF)),
    'violent_jitter_0_to_2ms': scenario('violent', dict(camera_jitter=(0.0, 0.001, 0.002))),
    'violent_jitter_0_to_10ms': scenario('violent', dict(camera_jitter=(0.0, 0.002, 0.005, 0.010))),
    'violent_gyro_uncalibrated': scenario('violent', UNCALIBRATED, dict(init_gyro_bias=1.0*DEG, gyro_scale=0.015)),
    'violent_imu_boxcar_100hz': scenario('violent', dict(imu_mode='mean')),
    'violent_imu_decimated_100hz': scenario('violent', dict(imu_mode='inst')),
    'violent_fov_half': scenario('violent', dict(fov_fraction=0.5)),
    'violent_landmarks_8': scenario('violent', dict(max_landmarks=8)),
    'violent_rate_1hz': scenario('violent', dict(frame_dt=1.0)),
    'violent_gap_3s': scenario('violent', dict(dropout=((8.0, 11.0),))),
    'violent_relief_2m': scenario('violent', dict(relief_sd=2.0), dict(map_var=(0.09, 0.09, 4.0))),
    # What each addition buys: the full filter with one removed.
    'without_iteration': scenario(filt=dict(iterations=1)),
    'without_post_fit_rejection': scenario(filt=dict(robust=False)),
    'without_recovery': scenario(filt=dict(recover=False)),
    'without_time_state': scenario(filt=dict(init_time_offset=OFF)),
    'without_scale_noise': scenario(filt=dict(gyro_scale=0.0)),
    'without_map_inflation': scenario(filt=dict(map_inflation=1.0)),
    'map_inflation_3': scenario(filt=dict(map_inflation=3.0)),
    'map_inflation_30': scenario(filt=dict(map_inflation=30.0)),
    # The same removals where the removed part is under load.
    'poor_first_fix_without_iteration': scenario(filt=dict(FIX_POOR, iterations=1)),
    'outliers_40pct_without_rejection': scenario(sensors=dict(outlier_rate=0.40), filt=dict(robust=False)),
    'poor_first_fix_outliers_20pct': scenario(sensors=dict(outlier_rate=0.20), filt=FIX_POOR),
    'poor_first_fix_outliers_20pct_without_rejection': scenario(sensors=dict(outlier_rate=0.20),
                                                              filt=dict(FIX_POOR, robust=False)),
    'gap_6s_without_recovery': scenario(sensors=dict(dropout=((8.0, 14.0),)), filt=dict(recover=False)),
    'uncalibrated_untold_without_recovery': scenario(sensors=UNCALIBRATED, filt=dict(recover=False)),
    # --- Camera-to-IMU timing (proposal section 12 and Appendix B).
    'delay_0ms': scenario(sensors=dict(camera_delay=0.0)),
    'delay_2ms': scenario(sensors=dict(camera_delay=0.002)),
    'delay_5ms': scenario(sensors=dict(camera_delay=0.005), note='2.5 sigma of the 2 ms offset prior'),
    'delay_10ms_prior_2ms': scenario(sensors=dict(camera_delay=0.010), note='5 sigma of the offset prior'),
    'delay_10ms_prior_10ms': scenario(sensors=dict(camera_delay=0.010), filt=dict(init_time_offset=0.010)),
    'delay_20ms_prior_20ms': scenario(sensors=dict(camera_delay=0.020), filt=dict(init_time_offset=0.020)),
    'delay_10ms_no_time_state': scenario(sensors=dict(camera_delay=0.010), filt=dict(init_time_offset=OFF)),
    'jitter_0_to_2ms': scenario(sensors=dict(camera_jitter=(0.0, 0.001, 0.002))),
    'jitter_0_to_10ms': scenario(sensors=dict(camera_jitter=(0.0, 0.002, 0.005, 0.010)),
                                 note='unsynchronised (USB-like) timestamps'),
    'jitter_0_to_10ms_severe': scenario('severe', dict(camera_jitter=(0.0, 0.002, 0.005, 0.010))),
    # --- Terrain relief the filter does or does not allow for.
    'relief_0p3m': scenario(sensors=dict(relief_sd=0.3), filt=dict(map_var=(0.09, 0.09, 0.09)),
                            note='landmark heights taken from a 1 m surface model'),
    'relief_2m_assumed_1m': scenario(sensors=dict(relief_sd=2.0)),
    'relief_2m_assumed_2m': scenario(sensors=dict(relief_sd=2.0), filt=dict(map_var=(0.09, 0.09, 4.0))),
    'relief_5m_assumed_1m': scenario(sensors=dict(relief_sd=5.0)),
    'relief_5m_assumed_5m': scenario(sensors=dict(relief_sd=5.0), filt=dict(map_var=(0.09, 0.09, 25.0))),
    # --- Landmark geometry: field of view and count (Appendix C).
    'fov_half': scenario(sensors=dict(fov_fraction=0.5), note='central half of the image only, ~53 deg'),
    'fov_third': scenario(sensors=dict(fov_fraction=1/3), note='central third only, ~37 deg'),
    'landmarks_8': scenario(sensors=dict(max_landmarks=8)),
    'landmarks_4': scenario(sensors=dict(max_landmarks=4)),
    'landmarks_12': scenario(sensors=dict(max_landmarks=12)),
    'sparse_map_80m': scenario(sensors=dict(landmark_spacing=80.0)),
    'sparse_map_120m': scenario(sensors=dict(landmark_spacing=120.0)),
    # Cheaper processing: fewer landmarks at a lower image rate.
    'landmarks_12_at_2p5hz': scenario(sensors=dict(max_landmarks=12, frame_dt=0.4)),
    'landmarks_8_at_2p5hz': scenario(sensors=dict(max_landmarks=8, frame_dt=0.4)),
    'landmarks_8_at_1hz': scenario(sensors=dict(max_landmarks=8, frame_dt=1.0)),
    'landmarks_4_at_1hz': scenario(sensors=dict(max_landmarks=4, frame_dt=1.0)),
    'violent_landmarks_8_at_1hz': scenario('violent', dict(max_landmarks=8, frame_dt=1.0)),
    # --- Gyro quality and IMU pre-integration.
    'gyro_icm42688': scenario(sensors=dict(gyro_noise=0.0028*DEG), filt=dict(gyro_noise=0.0028*DEG),
                              note='SOURCED: TDK ICM-42688-P typical 0.0028 dps/rtHz'),
    'gyro_5x_noise': scenario(sensors=dict(gyro_noise=0.07*DEG), filt=dict(gyro_noise=0.07*DEG)),
    'gyro_uncalibrated': scenario(sensors=UNCALIBRATED, filt=dict(init_gyro_bias=1.0*DEG, gyro_scale=0.015),
                                  note='datasheet-typical 1 % scale, 1 % cross-axis, 1 dps offset; filter told'),
    'gyro_uncalibrated_severe': scenario('severe', UNCALIBRATED, dict(init_gyro_bias=1.0*DEG, gyro_scale=0.015)),
    'gyro_uncalibrated_untold': scenario(sensors=UNCALIBRATED, note='as above but the filter assumes a calibrated gyro'),
    'imu_boxcar_100hz': scenario(sensors=dict(imu_mode='mean'), note='1 kHz samples averaged, no coning/sculling'),
    'imu_decimated_100hz': scenario(sensors=dict(imu_mode='inst'), note='one raw sample per 10 ms'),
    'imu_decimated_severe': scenario('severe', dict(imu_mode='inst')),
    # --- Bad matches.
    'outliers_20pct': scenario(sensors=dict(outlier_rate=0.20)),
    'outliers_40pct': scenario(sensors=dict(outlier_rate=0.40)),
    'outliers_60pct': scenario(sensors=dict(outlier_rate=0.60)),
    'alias_5_of_30': scenario(sensors=dict(alias_count=5), note='5 landmarks per frame share an 8 m shift'),
    'alias_12_of_30': scenario(sensors=dict(alias_count=12)),
    'alias_18_of_30': scenario(sensors=dict(alias_count=18), note='the false cluster is the majority'),
    # --- Frame rate and gaps.
    'rate_2p5hz': scenario(sensors=dict(frame_dt=0.4)),
    'rate_1hz': scenario(sensors=dict(frame_dt=1.0)),
    'gap_3s': scenario(sensors=dict(dropout=((8.0, 11.0),)), note='no imagery from 8 s to 11 s'),
    'gap_3s_severe': scenario('severe', dict(dropout=((8.0, 11.0),))),
    'gap_6s': scenario(sensors=dict(dropout=((8.0, 14.0),))),
    # --- Map datum, calibration and first-fix quality.
    'datum_2p5m': scenario(sensors=dict(datum=(2.5, 0.0)), note='rigid 2.5 m map offset'),
    'calibration_poor': scenario(sensors=dict(focal_error=0.005, principal_error=3.0, misalignment=0.3*DEG)),
    'poor_first_fix': scenario(filt=FIX_POOR, note='15 m, 4 to 6 degrees, 5 m/s at the first fix'),
    'poor_first_fix_proposal': scenario(filt=dict(SINGLE, map_var=(0.09, 0.09, 1.0), **FIX_POOR)),
    # --- End to end: started by the first-fix solver, yaw unknown.
    'first_fix_start': scenario(filt=dict(start='first_fix'), note='16 coarse matches, 30 % wrong'),
    'first_fix_start_violent': scenario('violent', filt=dict(start='first_fix')),
    'first_fix_start_50pct_wrong': scenario(sensors=dict(first_fix_outliers=0.50), filt=dict(start='first_fix')),
    'first_fix_start_8_matches': scenario(sensors=dict(first_fix_matches=8), filt=dict(start='first_fix')),
    'first_fix_start_proposal_filter': scenario(filt=dict(SINGLE, map_var=(0.09, 0.09, 1.0), start='first_fix')),
}


# --- Truth cache ---------------------------------------------------------------

KEEP = ('t', 'p', 'v', 'r', 'accel', 'gyro', 'accel_mean', 'gyro_mean', 'accel_inst', 'gyro_inst',
        'delay_p', 'delay_r', 'swing', 'air_velocity')


def truth_dir(canopy, n):
    # The directory name carries a digest of everything the batch depends on,
    # so editing a canopy class can never silently reuse an old batch.
    key = repr((asdict(CANOPIES[canopy]), asdict(tr.WindParams()), DURATION, NAV_DT, FRAME_DT, DELAYS))
    return CACHE/f'truth_{canopy}_n{n}_seed{SEED}_{hashlib.sha256(key.encode()).hexdigest()[:8]}'


def build_truth(canopy, n):
    """Simulate one canopy class once; scenarios differ only in sensors and filter."""
    directory = truth_dir(canopy, n)
    if (directory/'done').exists():
        return directory
    directory.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng([SEED, sorted(CANOPIES).index(canopy)])
    start = time.perf_counter()
    data = tr.simulate(CANOPIES[canopy], n, rng, duration=DURATION, nav_dt=NAV_DT, frame_dt=FRAME_DT, delays=DELAYS)
    for key in KEEP:
        np.save(directory/f'{key}.npy', data[key])
    (directory/'done').write_text('%.1f s' % (time.perf_counter() - start))
    return directory


_TRUTH = {}


def load_truth(canopy, n):
    key = (canopy, n)
    if key not in _TRUTH:
        directory = truth_dir(canopy, n)
        _TRUTH[key] = {k: np.load(directory/f'{k}.npy', mmap_mode='r') for k in KEEP}
    return _TRUTH[key]


# --- One descent ---------------------------------------------------------------

def nominal_camera():
    return Camera(FOCAL, FOCAL, IMAGE[0]/2, IMAGE[1]/2, IMAGE[0], IMAGE[1], lever=np.array(CAMERA_LEVER))


def select_landmarks(pixels, depth, limit, fraction, cells=(6, 5), margin=16.0, search_m=0.0):
    """At most one landmark per image cell, nearest the cell centre, so the
    same map landmark is reused while it stays in view and the set is spread
    across the field (Appendix C). search_m keeps each landmark that far, on
    the ground, from the edge of the usable image. Returns indices."""
    w, h = IMAGE
    x0, x1 = w/2*(1 - fraction) + margin, w/2*(1 + fraction) - margin
    y0, y1 = h/2*(1 - fraction) + margin, h/2*(1 + fraction) - margin
    edge = search_m*FOCAL/np.maximum(depth, 1.0)
    ok = np.flatnonzero((depth > 1.0) & (pixels[:, 0] > x0 + edge) & (pixels[:, 0] < x1 - edge)
                        & (pixels[:, 1] > y0 + edge) & (pixels[:, 1] < y1 - edge))
    if len(ok) == 0:
        return ok
    fx = (pixels[ok, 0] - x0)/(x1 - x0)*cells[0]
    fy = (pixels[ok, 1] - y0)/(y1 - y0)*cells[1]
    cell = np.minimum(fy.astype(int), cells[1] - 1)*cells[0] + np.minimum(fx.astype(int), cells[0] - 1)
    off = (fx % 1 - 0.5)**2 + (fy % 1 - 0.5)**2
    order = np.lexsort((off, cell))
    first = np.r_[True, cell[order][1:] != cell[order][:-1]]
    chosen = ok[order[first]]
    if len(chosen) > limit:
        # Thin evenly across cells rather than dropping one side of the image.
        chosen = chosen[np.round(np.linspace(0, len(chosen) - 1, limit)).astype(int)]
    return chosen


def imu_stream(s, truth, j):
    """What the simulated IMU and barometer of descent j report, with their errors.

    Returns measured specific force and rate (steps, 3), the true accelerometer
    and gyro biases (steps, 3), and the barometer bias and noise (steps,). Its
    own random stream, so scenarios share these draws wherever they can and
    other studies can rebuild exactly the samples the filter was given.
    """
    suffix = {'ideal': '', 'mean': '_mean', 'inst': '_inst'}[s.imu_mode]
    accel_true = np.asarray(truth['accel' + suffix][:, j])
    gyro_true = np.asarray(truth['gyro' + suffix][:, j])
    steps = len(accel_true)
    r_imu = np.random.default_rng([SEED, j, 0])
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
    return accel, gyro, ba, bg, baro_bias, baro_noise


def run_descent(job):
    name, j, n = job
    sc = SCENARIOS[name]
    s, c = sc['sensors'], sc['filter']
    truth = load_truth(sc['canopy'], n)
    steps = truth['accel'].shape[0]
    t_p, t_v, t_r = (np.asarray(truth[k][:, j]) for k in ('p', 'v', 'r'))
    frames = truth['delay_p'].shape[1]
    r_map, r_init, r_pix = (np.random.default_rng([SEED, j, k]) for k in range(1, 4))
    choices = s.camera_jitter or (s.camera_delay,)
    d_idx = np.array([DELAYS.index(d) for d in choices])[np.random.default_rng([SEED, j, 4]).integers(len(choices), size=frames)]
    cam_p = np.asarray(truth['delay_p'][:, :, j])[d_idx, np.arange(frames)]
    cam_r = np.asarray(truth['delay_r'][:, :, j])[d_idx, np.arange(frames)]
    true_delay = np.array(DELAYS)[d_idx]

    accel, gyro, ba, bg, baro_bias, baro_noise = imu_stream(s, truth, j)

    # Map landmarks on a jittered grid around the ground track. The filter is
    # given map coordinates; the world has each landmark displaced and raised.
    reach = 1.3*t_p[0, 2]
    lo, hi = t_p[:, :2].min(axis=0) - reach, t_p[:, :2].max(axis=0) + reach
    gx, gy = np.meshgrid(np.arange(lo[0], hi[0], s.landmark_spacing), np.arange(lo[1], hi[1], s.landmark_spacing))
    grid = np.column_stack((gx.ravel(), gy.ravel()))
    grid = grid + r_map.uniform(-0.4, 0.4, grid.shape)*s.landmark_spacing
    map_points = np.column_stack((grid, np.zeros(len(grid))))
    unit = r_map.normal(size=(len(grid), 3))
    world_points = map_points + unit*np.array([s.map_xy_sd, s.map_xy_sd, s.relief_sd])
    world_points[:, :2] += np.array(s.datum)

    true_cam = nominal_camera()
    calib = r_map.normal(size=6)
    true_cam.fx = true_cam.fy = FOCAL*(1 + s.focal_error*calib[0])
    true_cam.cx += s.principal_error*calib[1]
    true_cam.cy += s.principal_error*calib[2]
    true_cam.r_cb = exp_so3(s.misalignment*calib[3:])
    cam = nominal_camera()

    sd = np.r_[c.init_pos, c.init_vel, c.init_att, np.full(3, c.init_accel_bias), np.full(3, c.init_gyro_bias),
               c.init_baro_bias, c.init_time_offset]
    e0 = sd*np.r_[r_init.normal(size=16), 0.0]
    noise = ImuNoise(c.accel_noise*c.noise_scale, c.gyro_noise*c.noise_scale, c.accel_bias_walk,
                     c.gyro_bias_walk, c.baro_bias_walk, gyro_scale=c.gyro_scale, scale_tau=FRAME_DT)
    # The filter's biases start at zero; the true ones are drawn from its prior
    # widths above, and its bias priors are those widths.
    f = None
    if c.start == 'drawn':
        f = Eskf(t_p[0] - e0[POS], t_v[0] - e0[VEL], t_r[0] @ exp_so3(-e0[ATT]), np.diag(sd**2), noise)
        # In flight the filter has been propagating before the first image
        # arrives, so it already holds a body rate for the time-offset model.
        f.rate = gyro[0].copy()
    r_fix = np.random.default_rng([SEED, j, 6])
    first_fix = np.array([0.0 if f is not None else np.nan, 0.0, np.nan, np.nan])   # frame, refusals, error m, sigma m

    map_var = None if c.map_var is None else np.array(c.map_var)*c.map_inflation
    frame_every = round(s.frame_dt/NAV_DT)
    base_every = round(FRAME_DT/NAV_DT)
    n_rec = frames
    rec = {k: np.full((n_rec,) + shape, np.nan, dtype=np.float32) for k, shape in (
        ('err', (N_ERR,)), ('sig', (N_ERR,)), ('nees', (8,)), ('counts', (6,)), ('innov', ()), ('pl', ()),
        ('height', ()), ('swing', ()))}
    swing = truth['swing']
    for k in range(steps):
        if f is None:
            if k % base_every or k//base_every >= frames:
                continue
            fi = k//base_every
            pix_true, depth = true_cam.project(world_points, cam_p[fi], cam_r[fi])
            chosen = select_landmarks(pix_true, depth, s.first_fix_matches, s.fov_fraction)
            if len(chosen) >= 4:
                m = len(chosen)
                height = t_p[k, 2] + baro_bias[k] + baro_noise[k]
                sigma = np.sqrt(s.pixel_noise**2 + (s.first_fix_ground_noise*FOCAL/depth[chosen])**2)
                pixels = pix_true[chosen] + sigma[:, None]*r_fix.normal(size=(m, 2))
                wrong = r_fix.random(m) < s.first_fix_outliers
                pixels[wrong] = r_fix.uniform(0, 1, (int(wrong.sum()), 2))*np.array(IMAGE)
                told = math.sqrt(c.pixel_noise**2 + (s.first_fix_ground_noise*FOCAL/height)**2)
                # One image: the map error is taken at face value. The inflation
                # used by the filter is for a landmark seen again and again.
                fix = solve_first_fix(cam, pixels, map_points[chosen], height, told,
                                      map_var=None if c.map_var is None else np.array(c.map_var))
                if fix.ok:
                    cov0 = initial_covariance(fix, c.first_fix_vel_sigma, c.init_accel_bias, c.init_gyro_bias,
                                              c.init_baro_bias, c.init_time_offset)
                    f = Eskf(fix.p, [0.0, 0.0, -CANOPIES[sc['canopy']].descent_speed], fix.r, cov0, noise)
                    f.rate = gyro[k].copy()
                    first_fix[0] = fi
                    first_fix[2] = np.hypot(*(fix.p[:2] - t_p[k, :2]))
                    first_fix[3] = math.sqrt(fix.cov[0, 0] + fix.cov[1, 1])
                else:
                    first_fix[1] += 1
            if f is None:
                continue
        if k % base_every == 0 and k//base_every < frames:
            fi = k//base_every
            t_now = k*NAV_DT
            counts = np.zeros(6)
            innov = np.nan
            use = (k % frame_every == 0 and f.p[2] >= c.map_floor
                   and not (c.start == 'first_fix' and fi == first_fix[0])      # that image made the fix
                   and not any(a <= t_now < b for a, b in s.dropout))
            if use:
                pix_true, depth = true_cam.project(world_points, cam_p[fi], cam_r[fi])
                chosen = select_landmarks(pix_true, depth, s.max_landmarks, s.fov_fraction, search_m=s.search_margin_m)
                counts[0] = len(chosen)
                if len(chosen):
                    m = len(chosen)
                    z = np.asarray(depth[chosen])
                    pixel_sd = np.sqrt(s.pixel_noise**2 + (s.ground_noise*FOCAL/z)**2)
                    pixels = pix_true[chosen] + pixel_sd[:, None]*r_pix.normal(size=(m, 2))
                    bad = r_pix.random(m) < s.outlier_rate
                    pixels[bad] += (r_pix.uniform(-s.outlier_range, s.outlier_range, (int(bad.sum()), 2))
                                    * (FOCAL/z[bad])[:, None])
                    if s.alias_count:
                        alias = np.zeros(m, dtype=bool)
                        alias[:min(s.alias_count, m)] = True
                        # A shift on the ground is a pixel shift only up to the
                        # camera's yaw; the level-camera axes are close enough.
                        shift_px = (cam_r[fi].T @ np.array([*s.alias_shift, 0.0]))[:2]
                        pixels[alias] += shift_px*(FOCAL/z[alias])[:, None]
                        bad |= alias
                    # Prediction at the image's time stamp, before the filter's own
                    # time-offset estimate is applied: what a matcher that ignores the
                    # estimate would have to search around.
                    predicted, z_hat = cam.project(map_points[chosen], f.p, f.r)
                    z_hat = np.maximum(z_hat, 1.0)
                    pixel_var = c.pixel_noise**2 + (c.ground_noise*FOCAL/z_hat)**2
                    good = ~bad
                    if good.any():
                        innov = float(np.max(np.linalg.norm(pixels[good] - predicted[good], axis=1)*z_hat[good]/FOCAL))
                    out = f.update_landmarks(cam, map_points[chosen], pixels, pixel_var, map_var, c.gate,
                                             c.iterations, c.robust, c.recover)
                    acc = out['accepted']
                    counts[1:] = acc.sum(), (good & ~acc).sum(), (bad & acc).sum(), bad.sum(), out['recovered']
            e = f.error_from(t_p[k], t_v[k], t_r[k], ba[k], bg[k], baro_bias[k], true_delay[fi])
            cov = f.cov
            rec['err'][fi] = e
            rec['sig'][fi] = np.sqrt(np.diag(cov))
            try:
                rec['nees'][fi] = [e[:16] @ np.linalg.solve(cov[:16, :16], e[:16]),
                                   e[:2] @ np.linalg.solve(cov[:2, :2], e[:2]),
                                   e[3:5] @ np.linalg.solve(cov[3:5, 3:5], e[3:5])] + [
                    e[b] @ np.linalg.solve(cov[b, b], e[b]) for b in (ATT, BA, BG)] + [
                    e[BARO]**2/cov[BARO, BARO], e[2]**2/cov[2, 2]]
            except np.linalg.LinAlgError:
                pass
            rec['counts'][fi] = counts
            rec['innov'][fi] = innov
            rec['pl'][fi] = f.protection_level()
            rec['height'][fi] = t_p[k, 2]
            rec['swing'][fi] = swing[k, j]
        if k % 10 == 0:
            f.update_baro(t_p[k, 2] + baro_bias[k] + baro_noise[k], c.baro_sigma)
        f.propagate(NAV_DT, accel[k], gyro[k])
    rec['first_fix'] = first_fix.astype(np.float32)
    return rec


# --- Scoring -------------------------------------------------------------------

BIN_EDGES = np.arange(300.0, -1.0, -20.0)


def summarise(name, recs):
    sc = SCENARIOS[name]
    floor = sc['filter'].map_floor
    stack = {k: np.stack([r[k] for r in recs]).astype(float) for k in recs[0]}
    err, sig, nees, height = stack['err'], stack['sig'], stack['nees'], stack['height']
    n = err.shape[0]
    horiz = np.hypot(err[:, :, 0], err[:, :, 1])
    tilt = np.hypot(err[:, :, 6], err[:, :, 7])/DEG
    yaw = abs(err[:, :, 8])/DEG
    vel = np.hypot(err[:, :, 3], err[:, :, 4])
    first_fix = stack['first_fix']
    started = np.nan_to_num(first_fix[:, 0], nan=1e9)
    index = np.arange(err.shape[1])[None, :]
    above = (height >= floor) & (index >= started[:, None])   # navigating in the map-matching regime
    settled = above & (index >= started[:, None] + 10)        # 2 s after the first fix

    def rms(a, mask):
        return float(np.sqrt(np.nanmean(np.where(mask, a, np.nan)**2)))

    def over(a, mask, fn):
        return fn(np.where(mask, a, -np.inf), axis=1)

    worst = over(horiz, above, np.max)
    worst_settled = over(horiz, settled, np.max)
    # A descent that never navigated has not met the requirement.
    worst[~above.any(axis=1)] = np.inf
    worst_settled[~settled.any(axis=1)] = np.inf
    nees_mean = np.array([np.nanmean(np.where(settled, nees[:, :, i], np.nan), axis=1) for i in range(8)])
    exceed = np.where(settled, horiz > stack['pl'], np.nan)
    lag1 = []
    for j in range(n):
        for axis in (3, 4):
            x = err[j, settled[j], axis]
            if len(x) > 20:
                x = x - x.mean()
                lag1.append(float(x[1:] @ x[:-1]/(x @ x)))
    counts = stack['counts']
    used = counts[:, :, 0] > 0
    total = np.nansum(np.where(used[..., None], counts, 0), axis=(0, 1))
    curves = {'height': [], 'horiz_rms': [], 'horiz_p95': [], 'horiz_sigma': [], 'tilt_rms_deg': [],
              'yaw_rms_deg': [], 'tilt_sigma_deg': [], 'vel_rms': [], 'vel_sigma': [], 'nees16': [],
              'nees_pos2': [], 'swing_rms_deg': [], 'landmarks': []}
    for hi, lo in zip(BIN_EDGES[:-1], BIN_EDGES[1:]):
        mask = (height <= hi) & (height > lo)
        if not mask.any():
            continue
        curves['height'].append((hi + lo)/2)
        curves['horiz_rms'].append(rms(horiz, mask))
        curves['horiz_p95'].append(float(np.nanquantile(horiz[mask], 0.95)))
        curves['horiz_sigma'].append(rms(np.hypot(sig[:, :, 0], sig[:, :, 1]), mask))
        curves['tilt_rms_deg'].append(rms(tilt, mask))
        curves['yaw_rms_deg'].append(rms(yaw, mask))
        curves['tilt_sigma_deg'].append(rms(np.hypot(sig[:, :, 6], sig[:, :, 7])/DEG, mask))
        curves['vel_rms'].append(rms(vel, mask))
        curves['vel_sigma'].append(rms(np.hypot(sig[:, :, 3], sig[:, :, 4]), mask))
        curves['nees16'].append(float(np.nanmean(nees[:, :, 0][mask])))
        curves['nees_pos2'].append(float(np.nanmean(nees[:, :, 1][mask])))
        curves['swing_rms_deg'].append(rms(stack['swing']/DEG, mask))
        curves['landmarks'].append(float(np.nanmean(counts[:, :, 1][mask])))
    last = [np.flatnonzero(np.isfinite(height[j]) & (height[j] > 5.0)) for j in range(n)]
    final = np.array([horiz[j, i[-1]] if len(i) else np.nan for j, i in enumerate(last)])
    summary = {
        'canopy': sc['canopy'], 'note': sc['note'], 'descents': n,
        'horizontal_rms_m': rms(horiz, settled), 'horizontal_p95_m': float(np.nanquantile(horiz[settled], 0.95)),
        'horizontal_max_m': float(np.max(worst_settled[np.isfinite(worst_settled)], initial=0.0)),
        'filter_horizontal_sigma_m': rms(np.hypot(sig[:, :, 0], sig[:, :, 1]), settled),
        'T1_fraction_within_10m': float(np.mean(worst <= 10.0)),
        'T1_fraction_within_10m_after_2s': float(np.mean(worst_settled <= 10.0)),
        'fraction_within_5m_after_2s': float(np.mean(worst_settled <= 5.0)),
        'tilt_rms_deg': rms(tilt, settled), 'yaw_rms_deg': rms(yaw, settled),
        'filter_tilt_sigma_deg': rms(np.hypot(sig[:, :, 6], sig[:, :, 7])/DEG, settled),
        'height_rms_m': rms(err[:, :, 2], settled),
        'velocity_rms_mps': rms(vel, settled), 'filter_velocity_sigma_mps': rms(np.hypot(sig[:, :, 3], sig[:, :, 4]), settled),
        'velocity_error_lag1_autocorrelation': float(np.mean(lag1)) if lag1 else None,
        'nees16_mean': float(nees_mean[0].mean()), 'nees16_se': float(nees_mean[0].std(ddof=1)/math.sqrt(n)),
        'nees_pos2_mean': float(nees_mean[1].mean()), 'nees_pos2_se': float(nees_mean[1].std(ddof=1)/math.sqrt(n)),
        'nees_vel2_mean': float(nees_mean[2].mean()), 'nees_att3_mean': float(nees_mean[3].mean()),
        'nees_accel_bias3_mean': float(nees_mean[4].mean()), 'nees_gyro_bias3_mean': float(nees_mean[5].mean()),
        'nees_baro_bias1_mean': float(nees_mean[6].mean()), 'nees_height1_mean': float(nees_mean[7].mean()),
        'protection_level_exceedance': float(np.nanmean(exceed)),
        'protection_level_mean_m': rms(stack['pl'], settled),
        'landmarks_offered_per_frame': float(total[0]/max(used.sum(), 1)),
        'landmarks_accepted_per_frame': float(total[1]/max(used.sum(), 1)),
        'good_landmarks_rejected_fraction': float(total[2]/max(total[0] - total[4], 1)),
        'recoveries_per_descent': float(total[5]/n),
        'descents_with_a_recovery': float(np.mean(np.nansum(counts[:, :, 5], axis=1) > 0)),
        'time_offset_error_rms_ms': 1e3*rms(err[:, :, TIME], settled),
        'time_offset_sigma_ms': 1e3*rms(sig[:, :, TIME], settled),
        'bad_landmarks_accepted_fraction': float(total[3]/total[4]) if total[4] else None,
        'prediction_residual_p99_m': float(np.nanquantile(stack['innov'][settled & np.isfinite(stack['innov'])], 0.99))
        if np.isfinite(stack['innov'][settled]).any() else None,
        'prediction_residual_max_m': float(np.nanmax(np.where(settled, stack['innov'], np.nan)))
        if np.isfinite(stack['innov'][settled]).any() else None,
        'horizontal_rms_at_last_sample_m': float(np.sqrt(np.nanmean(final**2))),
        'swing_rms_deg': rms(stack['swing']/DEG, above),
        'first_fix_fraction_on_first_image': float(np.mean(first_fix[:, 0] == 0)),
        'first_fix_fraction_within_5s': float(np.mean(first_fix[:, 0] <= 25)),
        'first_fix_never': float(np.mean(~np.isfinite(first_fix[:, 0]))),
        'first_fix_refusals_per_descent': float(np.mean(first_fix[:, 1])),
        'first_fix_error_rms_m': float(np.sqrt(np.nanmean(first_fix[:, 2]**2))) if np.isfinite(first_fix[:, 2]).any() else None,
        'first_fix_error_max_m': float(np.nanmax(first_fix[:, 2])) if np.isfinite(first_fix[:, 2]).any() else None,
        'first_fix_sigma_m': float(np.sqrt(np.nanmean(first_fix[:, 3]**2))) if np.isfinite(first_fix[:, 3]).any() else None,
    }
    return summary, curves, stack


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--descents', type=int, default=160)
    parser.add_argument('--workers', type=int, default=6)
    parser.add_argument('--only', default='')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    if args.list:
        print('\n'.join(SCENARIOS))
        return
    names = [x for x in args.only.split(',') if x] or list(SCENARIOS)
    OUT.mkdir(parents=True, exist_ok=True)
    for canopy in sorted({SCENARIOS[x]['canopy'] for x in names}):
        t0 = time.perf_counter()
        build_truth(canopy, args.descents)
        print('truth %-9s ready (%.0f s)' % (canopy, time.perf_counter() - t0), flush=True)
    path = OUT/'summary.json'
    report = json.loads(path.read_text()) if path.exists() else {}
    report.update({'label': 'SYNTHETIC', 'seed': SEED, 'nav_dt_s': NAV_DT, 'image_px': IMAGE, 'focal_px': FOCAL})
    report.setdefault('scenarios', {})
    curve_path = OUT/'curves.json'
    all_curves = json.loads(curve_path.read_text()) if curve_path.exists() else {}
    with Pool(args.workers) as pool:
        for name in names:
            t0 = time.perf_counter()
            recs = pool.map(run_descent, [(name, j, args.descents) for j in range(args.descents)], chunksize=4)
            summary, curves, stack = summarise(name, recs)
            sc = SCENARIOS[name]
            summary['sensors'] = asdict(sc['sensors'])
            summary['filter'] = asdict(sc['filter'])
            report['scenarios'][name] = summary
            all_curves[name] = curves
            CACHE.mkdir(exist_ok=True)
            np.savez_compressed(CACHE/f'raw_{name}.npz', **{k: v.astype(np.float32) for k, v in stack.items()})
            print('%-26s %5.1fs  horiz rms %.2f p95 %.2f max %.2f m | sigma %.2f | T1 %.3f | tilt %.3f deg | '
                  'vel %.3f m/s | NEES16 %.1f (se %.1f) pos2 %.2f vel2 %.2f att3 %.2f ba3 %.2f bg3 %.2f baro %.2f | PL exceed %.3f' % (
                      name, time.perf_counter() - t0, summary['horizontal_rms_m'], summary['horizontal_p95_m'],
                      summary['horizontal_max_m'], summary['filter_horizontal_sigma_m'],
                      summary['T1_fraction_within_10m_after_2s'], summary['tilt_rms_deg'],
                      summary['velocity_rms_mps'], summary['nees16_mean'], summary['nees16_se'],
                      summary['nees_pos2_mean'], summary['nees_vel2_mean'], summary['nees_att3_mean'],
                      summary['nees_accel_bias3_mean'], summary['nees_gyro_bias3_mean'],
                      summary['nees_baro_bias1_mean'], summary['protection_level_exceedance']), flush=True)
            path.write_text(json.dumps(report, indent=1))
            curve_path.write_text(json.dumps(all_curves))


if __name__ == '__main__':
    main()
