"""GPL-3.0-only. SYNTHETIC characterisation of swing, spin, glide and false wind in truth.py descents.

Usage: python sim/nav/characterise_descent.py [--quick | --merge NAME ...]
Writes sim/nav/results/descent_characterisation.{json,csv}. Only NumPy is required.
SI units, world east/north/up; angles in degrees and rates in deg/s at the interfaces.
--quick runs 8 descents of a few configurations and writes nothing (a smoke test).
--merge runs the named configurations (for example ones the time budget skipped) and merges
them into the existing result files.

For each canopy configuration a batch of descents from 300 m is simulated with sim/nav/truth.py
and, after a 4 s start-up transient, these are measured across descents and time:

  swing      angle of the can axis from vertical, and of the line from vertical
  rates      transverse body rate sqrt(gx^2 + gy^2) and spin |gz|
  frequency  peak of the averaged power spectrum of the horizontal world components of the can
             axis (body z), compared with pendulum formulas and a linearised model
  glide      horizontal speed of the payload relative to the air, and whether its direction
             persists, wanders or rotates (coning)
  false wind the error of the lag inversion  w_hat = u + tau du/dt,  tau = v_d/g,  against the
             true wind at the payload, instantaneous and in 20 m height-layer means, with the
             naive estimate w_hat = u for comparison
  health     line tension, line length error, peak specific force and rate

Everything here is SYNTHETIC. The canopy normal-force law in truth.py is a generic assumed
shape, not a measured canopy, so these numbers describe the model, not our parachute. The
literature figures quoted in the output are the ones supplied with the task, not re-verified.
This is a verification model, not flight software.

Cost: one process, about 30 s per configuration at 48 descents with dt = 0.005 s (the integrator
is overhead bound, 4-9 ms per RK4 step on a laptop shared with a Monte Carlo, so dt = 0.002 s
would take about 2 minutes per configuration). The step is checked against 0.002 s and 0.001 s on
the moderate class inside the run. Configurations are listed in priority order
and the sweeps are not started once their time budget has been spent; skipped ones are recorded.
"""
import csv
import json
import math
import os
import platform
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_name, '1')     # shared machine: one process, one thread
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.dont_write_bytecode = True      # importing truth must not add .pyc files under sim/nav
sys.path.insert(0, str(HERE))
import truth as tr  # noqa: E402

OUT = HERE/'results'
DATE = '2026-10-09'
SEED = 20261009                 # every configuration uses this seed: winds and starts are paired
N_DESCENTS = 48
DT = 0.005                      # s, integration step; 0.002 and 0.001 are run on one configuration to check it
NAV_DT = 0.01                   # s, truth output step
DURATION = 26.0                 # s, 300 m down to about 27 m
DECAY_DURATION = 16.0           # s, initial-swing decay runs
DT_CHECK_DURATION = 14.0        # s, integration-step check runs
DT_CHECK_STEPS = tuple(sorted({DT, 0.002, 0.001}, reverse=True))   # s, coarse to fine; the finest is the reference
T_SKIP = 4.0                    # s, start-up transient discarded
LAYER_TOP, LAYER_BOTTOM, LAYER_DZ = 300.0, 80.0, 20.0
MIN_LAYER_SAMPLES = 100         # a layer mean needs at least 1 s of samples (a full layer is ~1.9 s)
F_MIN = 0.25                    # Hz, slower axis motion is drift or glide lean, not swing
NFFT = 1 << 14
IMU_ACCEL_RANGE_G, IMU_RATE_RANGE_DPS = 24.0, 2000.0
LINE_DRIFT_FRACTION = 0.01      # a line-length constraint error above this fraction of the line is a failure
SWEEP_BUDGET_S = 19*60.0        # the line-length and apparent-mass sweeps are not started after this
EXTRA_BUDGET_S = 16*60.0        # nor the extra (outside the requested grid) forced-swing runs
CALM = tr.WindParams(speed_10m=0.0, sigma_h=0.0, sigma_w=0.0)
DECAY_START = dict(tilt_sd_deg=20.0, rate_sd_dps=60.0)

LITERATURE = {
    'label': 'figures supplied with the task, not re-verified',
    'tutt_aiaa_2013_1384': '3.6 m extended-skirt canopy: average oscillation 20-27 deg, maximum 25-33 deg, '
                           '0.29-0.45 Hz (data from WADD AFTR 5867)',
    'nasa_sp_8066': 'small models are less stable than large; 10-15 deg pendular or coning motion is normally acceptable',
    'cross_annular': 'described as having good stability',
    'average_oscillation_deg': [20.0, 27.0], 'maximum_oscillation_deg': [25.0, 33.0],
    'acceptable_deg': [10.0, 15.0], 'frequency_hz': [0.29, 0.45]}
FIXED_PIVOT_FALSE_WIND = 2.06   # m/s, earlier fixed-pivot figure (15 deg swing, 1.5 m line)
REQUIREMENT = 0.5               # m/s RMS in 20 m layers


def config(name, group, wind='turb', budget=None, duration=DURATION, start=None, dt=DT, **overrides):
    """budget: None always runs; otherwise the elapsed seconds after which the run is skipped."""
    return dict(name=name, group=group, wind=wind, budget=budget, duration=duration,
                start=start or {}, params=replace(tr.Params(), dt=dt, **overrides))


def configurations():
    """In priority order: if the time budget runs out it is the sweeps that are lost."""
    benign = dict(cn_k=1.0, trim_deg=0.0)
    moderate = dict(cn_k=0.4, trim_deg=12.0)
    severe = dict(cn_k=0.5, trim_deg=25.0)
    out = [
        # The three classes the navigation Monte Carlo uses, in turbulence and in calm air.
        config('mc_benign_k1.0_t0', 'class'),
        config('mc_moderate_k0.4_t12', 'class', **moderate),
        config('mc_severe_k0.5_t25', 'class', **severe),
        config('mc_benign_k1.0_t0_calm', 'calm', wind='calm'),
        config('mc_moderate_k0.4_t12_calm', 'calm', wind='calm', **moderate),
        config('mc_severe_k0.5_t25_calm', 'calm', wind='calm', **severe),
        # Integration-step check on the moderate class (the most oscillatory): the same 48 descents
        # for 14 s at the production step, at 0.002 s and at 0.001 s.
        *[config(f'dtcheck_moderate_dt{dt}', 'dt_check', dt=dt, duration=DT_CHECK_DURATION, **moderate)
          for dt in DT_CHECK_STEPS],
        config('decay_k1.0_t0_calm', 'decay', wind='calm', duration=DECAY_DURATION, start=DECAY_START),
        config('stable_k0.3_t0', 'stable', cn_k=0.3),
        config('stable_k0.1_t0', 'stable', cn_k=0.1)]
    for cn_k in (0.2, 0.5):
        for trim in (5.0, 10.0, 20.0, 30.0):
            out.append(config(f'unstable_k{cn_k}_t{trim:.0f}', 'unstable', cn_k=cn_k, trim_deg=trim))
    # Sweeps. The 0.45 m line and 0.008 kg apparent mass points are the runs above.
    for label, base in (('benign', benign), ('moderate', moderate)):
        for line in (0.3, 0.8, 1.5):
            out.append(config(f'line{line}_{label}_k{base["cn_k"]}_t{base["trim_deg"]:.0f}', 'line_' + label,
                              budget=SWEEP_BUDGET_S, line=line, **base))
    for added in (0.004, 0.02):
        out.append(config(f'added{added}_moderate_k0.4_t12', 'added_mass', budget=SWEEP_BUDGET_S,
                          canopy_added_mass=added, **moderate))
    # EXTRA, outside the requested grid: no (cn_k, trim_deg) pair in the grid sustains a swing of
    # the size the literature gives (the unstable ones glide instead), so a lightly damped
    # restoring canopy is driven by the random canopy side force that truth.py provides.
    forced = dict(cn_k=0.1, side_force_sd=0.05)
    out.append(config('extra_k0.1_t0_side0.05', 'extra', budget=EXTRA_BUDGET_S, **forced))
    out.append(config('extra_k0.1_t0_side0.05_calm', 'extra', wind='calm', budget=EXTRA_BUDGET_S, **forced))
    return out


# --- Small helpers -------------------------------------------------------------

def rms(x, axis=None):
    return np.sqrt(np.mean(np.square(x), axis=axis))


def spread(x):
    """RMS, median, 95th percentile and maximum of a non-negative quantity."""
    return {'rms': float(rms(x)), 'median': float(np.median(x)),
            'p95': float(np.percentile(x, 95)), 'max': float(np.max(x))}


def clean(x):
    """Make a nested result JSON-safe: NumPy scalars to Python, non-finite to None."""
    if isinstance(x, dict):
        return {k: clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [clean(v) for v in x]
    if isinstance(x, np.ndarray):
        return clean(x.tolist())
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (int, np.integer)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        return float(x) if math.isfinite(x) else None
    return x


# --- Reference frequencies and the linearised model ----------------------------

def reference_frequencies(p):
    """Pendulum formulas, Hz, for comparison with the simulated dominant frequency."""
    d = -p.attach[2]
    inertia = p.inertia[0]
    simple = math.sqrt(tr.G0/p.line)/(2*math.pi)
    compound = math.sqrt(p.payload_mass*tr.G0*d/(inertia + p.payload_mass*d*d))/(2*math.pi)
    # Canopy held fixed, line plus rigid can: s = line slope, phi = can tilt,
    #   u'' = -(g/L)(u - d phi),  phi'' = (m g d/I)((u - d phi)/L - phi).
    a = tr.G0/p.line
    b = p.payload_mass*tr.G0*d/inertia
    lam = np.linalg.eigvals(np.array([[a, -a*d], [-b/p.line, b*(1 + d/p.line)]]))
    double = np.sort(np.sqrt(lam.real))/(2*math.pi)
    return {'simple_pendulum_hz': simple, 'can_compound_pendulum_hz': compound,
            'fixed_pivot_double_pendulum_hz': [float(double[0]), float(double[1])]}


def linear_modes(p):
    """Planar small-angle modes about steady vertical descent in still air.

    States: u = canopy minus payload lateral offset, phi = can tilt, and the lateral velocities
    of canopy and payload and the tilt rate. s = (u - d phi)/L is the line slope. To first order
      m_c x_c'' = (q - T - q c) s - (q c/V) x_c'      canopy: axial force follows the line,
      m_p x_p'' = T s - k_p V x_p'                             normal force has slope c at alpha = 0
      I phi''   = T d (s - phi) - c_r phi'
    with q the canopy drag at terminal speed, T the line tension, c = dCN/dalpha at zero, which is
    cn_k for a restoring canopy and -cn_k for one with a trim angle. For c = -cn_k this is the
    UNSTABLE vertical equilibrium, which the canopy leaves; those modes are not where it lives.
    """
    v = p.descent_speed
    m_c = p.canopy_mass + p.canopy_added_mass
    m_p, inertia, d, line = p.payload_mass, p.inertia[0], -p.attach[2], p.line
    payload_drag = 0.5*tr.RHO*p.payload_cds*v*v
    q = (p.payload_mass + p.canopy_mass)*tr.G0 - payload_drag
    tension = p.payload_mass*tr.G0 - payload_drag
    slope = p.cn_k if p.trim_deg <= 0 else -p.cn_k
    k_s = q - tension - q*slope
    c_c = q*slope/v
    c_p = payload_drag/v
    c_r = p.rot_damping[0]
    # y = [u, phi, x_c', x_p', phi']
    a = np.zeros((5, 5))
    a[0, 2], a[0, 3] = 1.0, -1.0
    a[1, 4] = 1.0
    s_u, s_phi = 1.0/line, -d/line
    a[2] = [k_s*s_u/m_c, k_s*s_phi/m_c, -c_c/m_c, 0.0, 0.0]
    a[3] = [tension*s_u/m_p, tension*s_phi/m_p, 0.0, -c_p/m_p, 0.0]
    a[4] = [tension*d*s_u/inertia, tension*d*(s_phi - 1)/inertia, 0.0, 0.0, -c_r/inertia]
    lam = np.linalg.eigvals(a)
    oscillatory = sorted(((abs(x.imag)/(2*math.pi), -1.0/x.real if x.real else math.inf)
                          for x in lam if x.imag > 1e-9), key=lambda m: m[0])
    real = sorted((-1.0/x.real if x.real else math.inf) for x in lam if abs(x.imag) <= 1e-9)
    stable = bool(max(lam.real) < 0)
    return {'about': 'stable vertical descent' if slope > 0 else 'UNSTABLE vertical equilibrium',
            'oscillatory_hz_and_efold_s': [[float(f), float(t)] for f, t in oscillatory],
            'real_time_constants_s': [float(t) for t in real],
            # The slowest real mode of a stable canopy is the lag the wind inversion assumes is v/g.
            'lag_time_constant_s': float(max(real)) if stable and real else None,
            'max_growth_rate_per_s': float(max(lam.real)),
            'v_over_g_s': float(v/tr.G0)}


# --- Measurements --------------------------------------------------------------

def swing_and_rates(d, k0):
    gyro = np.degrees(d['gyro'][k0:])
    return {'swing_deg': spread(np.degrees(d['swing'][k0:])),
            'line_angle_deg': spread(np.degrees(d['line_angle'][k0:])),
            'transverse_rate_dps': spread(np.hypot(gyro[..., 0], gyro[..., 1])),
            'spin_rate_dps': spread(np.abs(gyro[..., 2]))}


def axis_decomposition(axis_h):
    """Split the can-axis tilt into a steady lean per descent and the motion about it, deg."""
    lean = axis_h.mean(axis=0)
    wobble = axis_h - lean
    return {'steady_lean_deg_mean': float(np.degrees(np.arcsin(np.clip(tr.norm(lean), 0, 1))).mean()),
            'about_lean_rms_deg': float(np.degrees(rms(tr.norm(wobble))))}


def swing_spectrum(axis_h):
    """Dominant frequency of the horizontal components of the can axis, (K, n, 2)."""
    samples = len(axis_h)
    x = axis_h - axis_h.mean(axis=0)
    window = np.hanning(samples)
    power = (np.abs(np.fft.rfft(x*window[:, None, None], NFFT, axis=0))**2).mean(axis=(1, 2))
    f = np.fft.rfftfreq(NFFT, NAV_DT)
    band = (f >= F_MIN) & (f <= 20.0)
    first = int(np.argmax(band))
    peak = first + int(np.argmax(power[band]))
    rate_power = power*f*f                      # spectrum of the axis rate, which the gyro sees
    rate_peak = first + int(np.argmax(rate_power[band]))
    resolved = f > 2.0/(samples*NAV_DT)         # beyond the window's main lobe about zero
    any_peak = int(np.argmax(resolved)) + int(np.argmax(power[resolved]))
    total = power[1:].sum()
    # Rotary spectrum: a circular (coning) motion has all its power at one sign of frequency.
    z = (x[..., 0] + 1j*x[..., 1])*window[:, None]
    zp = np.abs(np.fft.fft(z, NFFT, axis=0))**2
    fz = np.fft.fftfreq(NFFT, NAV_DT)
    near = np.abs(np.abs(fz) - f[peak]) <= 0.2
    plus = zp[near & (fz > 0)].sum(axis=0)
    minus = zp[near & (fz < 0)].sum(axis=0)
    rotary = (plus - minus)/np.maximum(plus + minus, 1e-300)
    return {'per_descent_rotary': np.abs(rotary),
            'dominant_hz': float(f[peak]),
            'dominant_is_band_edge': bool(peak == first),
            'rate_spectrum_peak_hz': float(f[rate_peak]),
            'peak_including_drift_hz': float(f[any_peak]),
            'power_fraction_below_f_min': float(power[1:first].sum()/total) if total > 0 else 0.0,
            'rotary_index_abs_mean': float(np.abs(rotary).mean()),
            'resolution_hz': float(2.0/(samples*NAV_DT)), 'f_min_hz': F_MIN}


def glide(air_h):
    """Horizontal payload velocity relative to the air, (K, n, 2)."""
    samples, n, _ = air_h.shape
    speed = tr.norm(air_h)
    mean_speed = speed.mean(axis=0)
    vector_mean = tr.norm(air_h.mean(axis=0))
    persistence = vector_mean/np.maximum(mean_speed, 1e-12)   # 1 fixed direction, 0 averages out
    heading = np.arctan2(air_h[..., 1], air_h[..., 0])
    circular_variance = 1.0 - np.abs(np.exp(1j*heading).mean(axis=0))
    turns = (np.unwrap(heading, axis=0)[-1] - heading[0])/(2*math.pi)
    span = (samples - 1)*NAV_DT
    # Vector autocorrelation (mean not removed, so a steady glide never decorrelates).
    nfft = 1 << int(math.ceil(math.log2(2*samples)))
    spec = np.abs(np.fft.rfft(air_h, nfft, axis=0))**2
    acf = np.fft.irfft(spec, nfft, axis=0)[:samples//2].sum(axis=(1, 2))
    acf = acf/(samples - np.arange(samples//2))
    acf = acf/acf[0] if acf[0] > 0 else acf
    below = np.nonzero(acf < math.exp(-1))[0]
    return {'speed_mean': float(speed.mean()), 'speed_rms': float(rms(speed)),
            'vector_mean_speed': float(vector_mean.mean()),
            'persistence_ratio_mean': float(persistence.mean()),
            'persistence_ratio_median': float(np.median(persistence)),
            'fraction_fixed_direction': float((persistence > 0.8).mean()),
            'fraction_averaging_out': float((persistence < 0.2).mean()),
            'heading_circular_variance_mean': float(circular_variance.mean()),
            'heading_turns_abs_median': float(np.median(np.abs(turns))),
            'heading_rate_abs_median_dps': float(np.median(np.abs(turns))*360.0/span),
            'autocorrelation_time_s': float(below[0]*NAV_DT) if len(below) else float(samples//2*NAV_DT),
            'autocorrelation_time_capped': bool(len(below) == 0),
            'per_descent_persistence': persistence, 'per_descent_speed': mean_speed,
            'per_descent_heading_rate_dps': np.abs(turns)*360.0/span}


def true_wind(d, p):
    """Horizontal wind at the payload centre of mass for every truth sample, (K+1, n, 2)."""
    imu = np.array(p.imu)
    z = d['p'][:, :, 2] - d['r'][:, :, 2, :] @ imu
    wind = tr.Wind(d['wind_mean'] + d['wind_turb'], d['wind_dz'])
    w = np.stack([wind(zk)[:, :2] for zk in z])
    # Cross-check against v - air_velocity, moved from the IMU point to the centre of mass.
    steps = len(d['gyro_inst'])
    lever = np.einsum('knij,knj->kni', d['r'][:steps], tr.cross(d['gyro_inst'], imu))
    other = (d['v'][:steps] - lever - d['air_velocity'][:steps])[..., :2]
    return w, float(np.max(np.abs(other - w[:steps]))), float(rms(lever[..., :2]))


def error_statistics(err, height, keep):
    """Instantaneous and layer-mean RMS of a horizontal error, (K, n, 2), per axis."""
    n = err.shape[1]
    layers = round((LAYER_TOP - LAYER_BOTTOM)/LAYER_DZ)
    layer = np.floor((LAYER_TOP - height)/LAYER_DZ).astype(int)
    ok = keep[:, None] & (layer >= 0) & (layer < layers)
    index = (np.arange(n)[None, :]*layers + layer)[ok]
    count = np.bincount(index, minlength=n*layers).reshape(n, layers)
    full = count >= MIN_LAYER_SAMPLES
    out = {'cells': int(full.sum()), 'layers_populated': int(full.any(axis=0).sum())}
    inst, layer_rms, descent_rms = [], [], []
    for axis in range(2):
        e = err[..., axis]
        total = np.bincount(index, weights=e[ok], minlength=n*layers).reshape(n, layers)
        means = total[full]/count[full]
        per_descent = total.sum(axis=1)/np.maximum(count.sum(axis=1), 1)
        inst.append(float(rms(e[ok])))
        layer_rms.append(float(rms(means)))
        descent_rms.append(float(rms(per_descent)))
    # "rms" is per horizontal axis (east and north pooled); "vector" is the RMS magnitude, sqrt(2) larger.
    for key, a in (('instantaneous', inst), ('layer_mean', layer_rms), ('descent_mean', descent_rms)):
        out[key + '_rms_east_north'] = a
        out[key + '_rms'] = float(math.sqrt((a[0]**2 + a[1]**2)/2))
        out[key + '_rms_vector'] = float(math.hypot(a[0], a[1]))
    return out


def false_wind(d, p, k0, v_d, groups):
    """Error of the lag inversion and of the naive estimate against the true wind.

    groups maps a label to a boolean mask over descents; the statistics are repeated for each.
    """
    u = d['v'][:, :, :2]
    w, check, lever_rms = true_wind(d, p)
    tau = v_d/tr.G0
    dudt = (u[2:] - u[:-2])/(2*NAV_DT)                 # centred, for samples 1 .. K-1
    naive = (u - w)[1:-1]
    lag = naive + tau*dudt
    keep = np.arange(1, len(u) - 1) >= k0
    height = d['p'][1:-1, :, 2]
    in_band = keep[:, None] & (height >= LAYER_BOTTOM)
    # The tau that minimises the instantaneous error. Where there is swing it is set by the
    # swing, not by the lag, so it is a diagnostic and not a better time constant.
    fit = -float((naive*dudt)[in_band].sum()/np.maximum((dudt*dudt)[in_band].sum(), 1e-300))
    out = {'tau_s': float(tau), 'tau_least_squares_s': fit,
           'lag_corrected': error_statistics(lag, height, keep),
           'naive': error_statistics(naive, height, keep),
           'wind_reconstruction_max_error': check, 'imu_lever_velocity_rms': lever_rms, 'by_behaviour': {}}
    for label, mask in groups.items():
        if mask.any():
            out['by_behaviour'][label] = {
                'descents': int(mask.sum()),
                'lag_corrected': error_statistics(lag[:, mask], height[:, mask], keep),
                'naive': error_statistics(naive[:, mask], height[:, mask], keep)}
    return out


def health(d, p, k0):
    fine_length = np.array([f[1] for f in d['fine']])
    fine_tension = np.array([f[2] for f in d['fine']])
    sub = round(NAV_DT/p.dt)
    accel = max(float(np.max(tr.norm(d[k]))) for k in ('accel', 'accel_inst'))/tr.G0
    gyro = max(float(np.max(tr.norm(d[k]))) for k in ('gyro', 'gyro_inst'))
    accel_late = max(float(np.max(tr.norm(d[k][k0:]))) for k in ('accel', 'accel_inst'))/tr.G0
    gyro_late = max(float(np.max(tr.norm(d[k][k0:]))) for k in ('gyro', 'gyro_inst'))
    finite = all(bool(np.isfinite(d[k]).all()) for k in ('p', 'v', 'r', 'accel', 'gyro', 'swing', 'tension'))
    slack = fine_tension <= 0.0
    vz = -d['v'][k0:, :, 2]
    out = {'finite': finite,
           'min_tension_N': float(np.min(fine_tension)),
           'min_tension_after_skip_N': float(np.min(fine_tension[k0*sub:])),
           'mean_tension_N': float(np.mean(fine_tension[k0*sub:])),
           'slack_steps': int(slack.sum()), 'descents_with_slack': int(slack.any(axis=0).sum()),
           'line_length_error_max_m': float(np.max(np.abs(fine_length - p.line))),
           'max_accel_g': accel, 'max_gyro_dps': math.degrees(gyro),
           'max_accel_after_transient_g': accel_late, 'max_gyro_after_transient_dps': math.degrees(gyro_late),
           'descent_speed_mean': float(vz.mean()), 'descent_speed_sd': float(vz.std()),
           'final_height_min_m': float(d['p'][-1, :, 2].min())}
    failures = []
    if not finite:
        failures.append('non-finite values')
    if out['slack_steps']:
        failures.append(f"slack line in {out['descents_with_slack']} descents")
    if not out['line_length_error_max_m'] < LINE_DRIFT_FRACTION*p.line:
        failures.append('line length drift')
    if not accel < IMU_ACCEL_RANGE_G:
        failures.append('specific force beyond IMU range')
    if not math.degrees(gyro) < IMU_RATE_RANGE_DPS:
        failures.append('rate beyond IMU range')
    out['failures'] = failures
    return out


def envelope_decay(t, angle_deg, bin_s=0.25):
    """E-folding time of the ensemble-RMS envelope of an angle, (K+1, n) in degrees."""
    per = round(bin_s/NAV_DT)
    bins = (len(t) - 1)//per
    env = np.array([rms(angle_deg[i*per:(i + 1)*per]) for i in range(bins)])
    centre = (np.arange(bins) + 0.5)*bin_s
    floor = max(0.02*env[0], 0.02)
    low = np.nonzero(env < floor)[0]
    end = int(low[0]) if len(low) else bins

    def efold(a, b):
        b = min(b, end)
        if b - a < 3:
            return None
        return float(-1.0/np.polyfit(centre[a:b], np.log(env[a:b]), 1)[0])

    def reach(level):
        hit = np.nonzero(env < level)[0]
        return float(centre[hit[0]]) if len(hit) else None

    split = round(2.0/bin_s)
    return {'initial_rms_deg': float(rms(angle_deg[0])), 'bin_s': bin_s,
            'envelope_rms_deg': [float(x) for x in env],
            'efold_s': efold(0, bins), 'efold_first_2s_s': efold(0, split), 'efold_after_2s_s': efold(split, bins),
            'fit_end_s': float(centre[end - 1]),
            'time_to_1_over_e_s': reach(env[0]/math.e), 'time_to_10_percent_s': reach(0.1*env[0]),
            'time_below_2deg_s': reach(2.0), 'time_below_1deg_s': reach(1.0), 'time_below_0.5deg_s': reach(0.5)}


def decay(d):
    """Decay of the initial swing: the raw swing angle, and the swing about the relative wind.

    A payload with residual sideways airspeed hangs along its relative wind while that speed
    dies away with the lag time constant, so the raw angle contains a lean that is not swing.
    """
    axis = d['r'][:, :, :, 2]
    air = d['air_velocity']
    cos = (axis*air).sum(-1)/tr.norm(air)
    about_wind = np.degrees(np.arccos(np.clip(cos, -1, 1)))
    return {'swing': envelope_decay(d['t'], np.degrees(d['swing'])),
            'line_angle': envelope_decay(d['t'], np.degrees(d['line_angle'])),
            'swing_about_relative_wind': envelope_decay(d['t'], about_wind),
            'air_speed_horizontal': envelope_decay(d['t'], tr.norm(air[..., :2]))}


def sample_series(d, descent=0):
    """A few rows of one descent, to confirm by eye that the motion is physical."""
    rows = []
    air = d['air_velocity']
    for seconds in (0, 1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26):
        k = round(seconds/NAV_DT)
        if k >= len(d['t']):
            break
        j = descent
        rows.append({'t': float(d['t'][k]), 'height': float(d['p'][k, j, 2]), 'vz': float(d['v'][k, j, 2]),
                     'swing_deg': float(np.degrees(d['swing'][k, j])),
                     'line_deg': float(np.degrees(d['line_angle'][k, j])),
                     'glide_speed': float(np.hypot(air[k, j, 0], air[k, j, 1])),
                     'glide_heading_deg': float(np.degrees(np.arctan2(air[k, j, 1], air[k, j, 0]))),
                     'tension_N': float(d['tension'][min(k, len(d['tension']) - 1), j]),
                     'spin_dps': float(np.degrees(d['gyro'][min(k, len(d['gyro']) - 1), j, 2]))})
    return rows


def characterise(cfg, n):
    p = cfg['params']
    start = time.perf_counter()
    out = {'name': cfg['name'], 'group': cfg['group'], 'wind': cfg['wind'], 'n': n, 'dt': p.dt,
           'duration_s': cfg['duration'], 'start': cfg['start'],
           'cn_k': p.cn_k, 'trim_deg': p.trim_deg, 'line': p.line, 'canopy_added_mass': p.canopy_added_mass,
           'side_force_sd': p.side_force_sd, 'reference': reference_frequencies(p)}
    try:
        d = tr.simulate(p, n, np.random.default_rng(SEED), duration=cfg['duration'],
                        wind_params=CALM if cfg['wind'] == 'calm' else None, start=cfg['start'], keep_fine=True)
    except Exception as error:      # a model limit is reported, never dropped
        out.update(simulate_seconds=time.perf_counter() - start,
                   health={'finite': False, 'failures': [f'simulate raised {type(error).__name__}: {error}']})
        return out
    out['simulate_seconds'] = time.perf_counter() - start
    k0 = round(T_SKIP/NAV_DT)
    h = health(d, p, k0)
    out['health'] = h
    out['sample_series'] = sample_series(d)
    # The planar linearisation about steady descent is only the system's equilibrium when the
    # canopy is restoring; with a trim angle the vertical equilibrium is unstable.
    out['linear_model'] = linear_modes(p) if p.trim_deg <= 0 else None
    if not h['finite']:
        return out
    axis_h = d['r'][k0:, :, :2, 2]
    out.update(swing_and_rates(d, k0))
    out['axis'] = axis_decomposition(axis_h)
    out['spectrum'] = swing_spectrum(axis_h)
    rotary = out['spectrum'].pop('per_descent_rotary')
    g = glide(d['air_velocity'][k0:, :, :2])
    persistence, speed = g.pop('per_descent_persistence'), g.pop('per_descent_speed')
    heading_rate = g.pop('per_descent_heading_rate_dps')
    out['glide'] = g
    # Per descent: a glide in a fixed direction, or something else (coning, wandering, swinging)?
    swing_each = np.degrees(rms(d['swing'][k0:], axis=0))
    groups = {'fixed_direction': persistence > 0.8, 'other': persistence <= 0.8}
    out['behaviour'] = {'criterion': 'fixed_direction: |time-mean air-relative velocity| > 0.8 of its mean speed'}
    for label, mask in groups.items():
        if mask.any():
            out['behaviour'][label] = {
                'descents': int(mask.sum()), 'glide_speed_mean': float(speed[mask].mean()),
                'persistence_ratio_mean': float(persistence[mask].mean()),
                'swing_rms_deg': float(rms(swing_each[mask])),
                'swing_p95_deg': float(np.percentile(np.degrees(d['swing'][k0:, mask]), 95)),
                'heading_rate_abs_median_dps': float(np.median(heading_rate[mask])),
                'rotary_index_abs_mean': float(rotary[mask].mean())}
    out['false_wind'] = false_wind(d, p, k0, h['descent_speed_mean'], groups)
    # The lag inversion multiplies a motion at frequency f by |1 + i 2 pi f tau|; swing is amplified.
    tau = out['false_wind']['tau_s']
    out['false_wind']['inversion_gain_at_dominant_hz'] = float(math.hypot(1.0, 2*math.pi*out['spectrum']['dominant_hz']*tau))
    if cfg['group'] == 'decay':
        out['decay'] = decay(d)
    return out


# --- Output --------------------------------------------------------------------

COLUMNS = [
    ('name', lambda r: r['name']), ('group', lambda r: r['group']), ('wind', lambda r: r['wind']),
    ('cn_k', lambda r: r['cn_k']), ('trim_deg', lambda r: r['trim_deg']), ('line_m', lambda r: r['line']),
    ('canopy_added_mass_kg', lambda r: r['canopy_added_mass']), ('side_force_sd_N', lambda r: r['side_force_sd']),
    ('n', lambda r: r['n']), ('dt_s', lambda r: r['dt']),
    ('swing_rms_deg', lambda r: r['swing_deg']['rms']), ('swing_median_deg', lambda r: r['swing_deg']['median']),
    ('swing_p95_deg', lambda r: r['swing_deg']['p95']), ('swing_max_deg', lambda r: r['swing_deg']['max']),
    ('line_rms_deg', lambda r: r['line_angle_deg']['rms']), ('line_median_deg', lambda r: r['line_angle_deg']['median']),
    ('line_p95_deg', lambda r: r['line_angle_deg']['p95']), ('line_max_deg', lambda r: r['line_angle_deg']['max']),
    ('steady_lean_deg', lambda r: r['axis']['steady_lean_deg_mean']),
    ('about_lean_rms_deg', lambda r: r['axis']['about_lean_rms_deg']),
    ('transverse_rate_rms_dps', lambda r: r['transverse_rate_dps']['rms']),
    ('transverse_rate_p95_dps', lambda r: r['transverse_rate_dps']['p95']),
    ('spin_rms_dps', lambda r: r['spin_rate_dps']['rms']), ('spin_p95_dps', lambda r: r['spin_rate_dps']['p95']),
    ('spin_max_dps', lambda r: r['spin_rate_dps']['max']),
    ('dominant_hz', lambda r: r['spectrum']['dominant_hz']),
    ('dominant_is_band_edge', lambda r: r['spectrum']['dominant_is_band_edge']),
    ('rate_spectrum_peak_hz', lambda r: r['spectrum']['rate_spectrum_peak_hz']),
    ('rotary_index', lambda r: r['spectrum']['rotary_index_abs_mean']),
    ('simple_pendulum_hz', lambda r: r['reference']['simple_pendulum_hz']),
    ('can_compound_pendulum_hz', lambda r: r['reference']['can_compound_pendulum_hz']),
    ('linear_model_lowest_mode_hz', lambda r: r['linear_model']['oscillatory_hz_and_efold_s'][0][0]),
    ('linear_model_lowest_mode_efold_s', lambda r: r['linear_model']['oscillatory_hz_and_efold_s'][0][1]),
    ('linear_model_lag_time_constant_s', lambda r: r['linear_model']['lag_time_constant_s']),
    ('glide_speed_mean', lambda r: r['glide']['speed_mean']), ('glide_speed_rms', lambda r: r['glide']['speed_rms']),
    ('glide_persistence_ratio', lambda r: r['glide']['persistence_ratio_mean']),
    ('glide_fraction_fixed_direction', lambda r: r['glide']['fraction_fixed_direction']),
    ('glide_heading_rate_abs_median_dps', lambda r: r['glide']['heading_rate_abs_median_dps']),
    ('glide_heading_circular_variance', lambda r: r['glide']['heading_circular_variance_mean']),
    ('glide_autocorrelation_time_s', lambda r: r['glide']['autocorrelation_time_s']),
    ('tau_s', lambda r: r['false_wind']['tau_s']),
    ('false_wind_lag_inst_rms', lambda r: r['false_wind']['lag_corrected']['instantaneous_rms']),
    ('false_wind_lag_layer_rms', lambda r: r['false_wind']['lag_corrected']['layer_mean_rms']),
    ('false_wind_lag_layer_rms_vector', lambda r: r['false_wind']['lag_corrected']['layer_mean_rms_vector']),
    ('false_wind_lag_descent_mean_rms', lambda r: r['false_wind']['lag_corrected']['descent_mean_rms']),
    ('false_wind_naive_inst_rms', lambda r: r['false_wind']['naive']['instantaneous_rms']),
    ('false_wind_naive_layer_rms', lambda r: r['false_wind']['naive']['layer_mean_rms']),
    ('false_wind_naive_descent_mean_rms', lambda r: r['false_wind']['naive']['descent_mean_rms']),
    ('inversion_gain_at_dominant_hz', lambda r: r['false_wind']['inversion_gain_at_dominant_hz']),
    ('descent_speed_mean', lambda r: r['health']['descent_speed_mean']),
    ('min_tension_N', lambda r: r['health']['min_tension_N']),
    ('slack_steps', lambda r: r['health']['slack_steps']),
    ('line_length_error_max_m', lambda r: r['health']['line_length_error_max_m']),
    ('max_accel_g', lambda r: r['health']['max_accel_g']), ('max_gyro_dps', lambda r: r['health']['max_gyro_dps']),
    ('max_accel_after_transient_g', lambda r: r['health']['max_accel_after_transient_g']),
    ('max_gyro_after_transient_dps', lambda r: r['health']['max_gyro_after_transient_dps']),
    ('failures', lambda r: '; '.join(r['health']['failures']))]


def cell(row, getter):
    try:
        value = getter(row)
    except (KeyError, IndexError, TypeError):
        return ''
    if isinstance(value, (float, np.floating)):
        return f'{value:.6g}'
    return value


def print_table(results):
    head = (f"{'configuration':32s} {'swing rms/p95/max':>18s} {'line rms':>8s} {'rate rms/p95':>12s} {'spin':>5s} "
            f"{'f_dom':>5s} {'glide':>5s} {'pers':>4s} {'FW lag i/L':>11s} {'FW naive i/L':>12s} {'Tmin':>5s}  failures")
    print(head)
    for r in results:
        if 'swing_deg' not in r:
            print(f"{r['name']:32s} FAILED: {'; '.join(r['health']['failures'])}")
            continue
        s, fw, g = r['swing_deg'], r['false_wind'], r['glide']
        pers = f"{g['persistence_ratio_mean']:4.2f}" if g['speed_mean'] >= 0.1 else '   -'
        print(f"{r['name']:32s} {s['rms']:5.2f}/{s['p95']:5.2f}/{s['max']:5.2f} {r['line_angle_deg']['rms']:8.2f} "
              f"{r['transverse_rate_dps']['rms']:5.1f}/{r['transverse_rate_dps']['p95']:5.1f} "
              f"{r['spin_rate_dps']['rms']:5.1f} {r['spectrum']['dominant_hz']:5.2f} "
              f"{g['speed_mean']:5.2f} {pers} "
              f"{fw['lag_corrected']['instantaneous_rms']:5.2f}/{fw['lag_corrected']['layer_mean_rms']:5.2f} "
              f"{fw['naive']['instantaneous_rms']:5.2f}/{fw['naive']['layer_mean_rms']:6.2f} "
              f"{r['health']['min_tension_N']:5.2f}  {'; '.join(r['health']['failures'])}")


DT_CHECK_KEYS = ['swing_rms_deg', 'swing_p95_deg', 'swing_max_deg', 'line_rms_deg', 'transverse_rate_rms_dps',
                 'transverse_rate_p95_dps', 'dominant_hz', 'glide_speed_mean', 'glide_persistence_ratio',
                 'false_wind_lag_inst_rms', 'false_wind_lag_layer_rms', 'false_wind_naive_inst_rms',
                 'false_wind_naive_layer_rms', 'descent_speed_mean', 'min_tension_N']


def dt_agreement(results):
    """Relative difference of headline statistics against the finest-step run of the same descents."""
    runs = sorted((r for r in results if r['group'] == 'dt_check' and 'swing_deg' in r), key=lambda r: r['dt'])
    if len(runs) < 2:
        return None
    reference, getters = runs[0], dict(COLUMNS)
    out = {'reference_dt': reference['dt'], 'duration_s': reference['duration_s'],
           'note': 'Winds and starting states are identical (same seed). The random spin torque is drawn once per '
                   'step, so its realisation differs with the step; spin_rms_dps therefore differs by sampling '
                   'noise, not by integration error, and is left out of largest_relative_difference.',
           'by_dt': {}}
    for run in runs[1:]:
        stats = {}
        for key in DT_CHECK_KEYS + ['spin_rms_dps']:
            a, b = float(getters[key](reference)), float(getters[key](run))
            stats[key] = {'reference': a, 'this_dt': b, 'relative_difference': (b - a)/a if a else None}
        out['by_dt'][f"{run['dt']:g}"] = {
            'statistics': stats,
            'largest_relative_difference': max(abs(v['relative_difference'] or 0.0)
                                               for k, v in stats.items() if k != 'spin_rms_dps')}
    return out


def sanity_checks(results):
    """Is the model behaving physically? Every row is pass/fail with the number behind it."""
    rows = []

    def add(name, config, value, ok):
        rows.append({'check': name, 'configuration': config, 'value': value, 'pass': bool(ok)})

    for r in results:
        h = r['health']
        add('finite', r['name'], bool(h['finite']), h['finite'])
        if 'swing_deg' not in r:
            continue
        add('tension stays positive (min over all steps, N)', r['name'], h['min_tension_N'], h['min_tension_N'] > 0.05)
        add('mean descent speed within 1 m/s of 10.5', r['name'], h['descent_speed_mean'],
            abs(h['descent_speed_mean'] - 10.5) < 1.0)
        add('can never tumbles: swing after transient below 90 deg', r['name'], r['swing_deg']['max'], r['swing_deg']['max'] < 90)
        add('line length constraint error below 1 % of the line (m)', r['name'], h['line_length_error_max_m'],
            h['line_length_error_max_m'] < LINE_DRIFT_FRACTION*r['line'])
        add('true wind rebuilt from the profile agrees with v - air_velocity (m/s)', r['name'],
            r['false_wind']['wind_reconstruction_max_error'], r['false_wind']['wind_reconstruction_max_error'] < 1e-6)
    for r in results:
        if r['name'] == 'mc_benign_k1.0_t0_calm' and 'swing_deg' in r:
            add('benign calm: swing RMS after transient below 0.5 deg', r['name'], r['swing_deg']['rms'],
                r['swing_deg']['rms'] < 0.5)
            add('benign calm: glide speed below 0.05 m/s', r['name'], r['glide']['speed_mean'],
                r['glide']['speed_mean'] < 0.05)
    return rows


def merge_run(names, n):
    """Run the named configurations and merge them into the existing result files.

    For configurations that the time budget skipped: python characterise_descent.py --merge NAME ...
    """
    path = OUT/'descent_characterisation.json'
    with open(path, encoding='utf-8') as f:
        document = json.load(f)
    order = [c['name'] for c in configurations()]
    todo = [c for c in configurations() if c['name'] in names]
    if len(todo) != len(names):
        raise SystemExit(f'unknown configuration among {names}')
    began = time.perf_counter()
    fresh = {}
    for cfg in todo:
        r = clean(characterise(cfg, n))
        fresh[cfg['name']] = r
        print(f"{cfg['name']}: {r['simulate_seconds']:.0f} s, {'; '.join(r['health']['failures']) or 'ok'}", flush=True)
    runtime = time.perf_counter() - began
    kept = [c for c in document['configurations'] if c['name'] not in fresh]
    results = sorted(kept + list(fresh.values()), key=lambda c: order.index(c['name']))
    skipped = [x for x in document['skipped_for_time'] if x not in fresh]
    print_table(fresh.values())
    write_outputs(OUT, results, skipped, document['dt_check'], document['runtime_s'] + runtime, n,
                  sanity_checks(results), merged=document.get('merged_runs', []) +
                  [{'configurations': list(fresh), 'runtime_s': runtime}])
    print(f"merged {len(fresh)} configurations into {path}")


def main():
    if '--merge' in sys.argv:
        merge_run([a for a in sys.argv[1:] if not a.startswith('--')], N_DESCENTS)
        return
    quick = '--quick' in sys.argv
    n = 8 if quick else N_DESCENTS
    todo = configurations()
    if quick:
        wanted = [a for a in sys.argv[1:] if not a.startswith('--')]
        todo = [c for c in todo if (c['name'] in wanted if wanted else c['group'] in ('class', 'decay'))]
    began = time.perf_counter()
    results, skipped = [], []
    for i, cfg in enumerate(todo):
        elapsed = time.perf_counter() - began
        if cfg['budget'] is not None and elapsed > cfg['budget']:
            skipped.append(cfg['name'])
            print(f"[{i + 1}/{len(todo)}] {cfg['name']}: SKIPPED, {elapsed:.0f} s elapsed, "
                  f"time budget of {cfg['budget']:.0f} s spent", flush=True)
            continue
        r = characterise(cfg, n)
        results.append(r)
        h = r['health']
        note = '; '.join(h['failures']) or 'ok'
        print(f"[{i + 1}/{len(todo)}] {cfg['name']}: {r['simulate_seconds']:.0f} s (elapsed {time.perf_counter() - began:.0f}), "
              f"line error {h.get('line_length_error_max_m', math.nan):.1e} m, "
              f"tension {h.get('min_tension_N', math.nan):.2f} N min, "
              f"descent {h.get('descent_speed_mean', math.nan):.2f} m/s, {note}", flush=True)
    runtime = time.perf_counter() - began
    checks = sanity_checks(results)

    print('\nSYNTHETIC. One descent of each navigation Monte Carlo class (turbulent wind):')
    for r in results:
        if r['group'] == 'class' or (quick and r['group'] == 'decay'):
            print(f"  {r['name']}")
            for row in r.get('sample_series', []):
                print(f"    t={row['t']:5.1f} z={row['height']:6.1f} vz={row['vz']:7.2f} swing={row['swing_deg']:5.2f} "
                      f"line={row['line_deg']:5.2f} glide={row['glide_speed']:5.2f} hdg={row['glide_heading_deg']:6.1f} "
                      f"T={row['tension_N']:5.2f} spin={row['spin_dps']:7.1f}")
    print('\nSYNTHETIC. Angles deg, rates deg/s, frequency Hz, speeds m/s; false wind (FW) per axis, '
          'instantaneous / 20 m layer means; statistics after the first 4 s.')
    print_table(results)
    print('\nSYNTHETIC. By behaviour of each descent: n, glide speed, swing RMS / p95, heading rate deg/s, '
          'false wind lag-corrected instantaneous / layer, naive layer.')
    for r in results:
        for label, b in r.get('behaviour', {}).items():
            if label == 'criterion':
                continue
            fw = r['false_wind']['by_behaviour'][label]
            print(f"{r['name']:30s} {label:15s} n={b['descents']:2d} glide {b['glide_speed_mean']:5.2f} "
                  f"swing {b['swing_rms_deg']:5.2f}/{b['swing_p95_deg']:5.2f} turn {b['heading_rate_abs_median_dps']:6.1f} "
                  f"FW {fw['lag_corrected']['instantaneous_rms']:5.2f}/{fw['lag_corrected']['layer_mean_rms']:5.2f} "
                  f"naive {fw['naive']['layer_mean_rms']:5.2f}")
    for r in results:
        if 'decay' in r:
            for key in ('swing', 'line_angle', 'swing_about_relative_wind', 'air_speed_horizontal'):
                e = r['decay'][key]
                show = lambda x: 'none' if x is None else f'{x:.2f}'      # noqa: E731
                print(f"{r['name']} {key}: e-fold {show(e['efold_s'])} s (first 2 s {show(e['efold_first_2s_s'])}, "
                      f"later {show(e['efold_after_2s_s'])}), to 1/e {show(e['time_to_1_over_e_s'])} s, to 10 % "
                      f"{show(e['time_to_10_percent_s'])} s, below 1 {show(e['time_below_1deg_s'])} s; "
                      f"envelope each 1 s {np.round(e['envelope_rms_deg'][::4][:12], 3).tolist()}")
    agreement = dt_agreement(results)
    if agreement:
        for dt, entry in agreement['by_dt'].items():
            print(f"dt check, step {dt} s against {agreement['reference_dt']:g} s: largest relative difference "
                  f"{entry['largest_relative_difference']:.4f}")
            for key, v in entry['statistics'].items():
                rd = v['relative_difference']
                print(f"  {key:28s} {v['reference']:10.4f} {v['this_dt']:10.4f} {'' if rd is None else f'{rd:+.4f}'}")
    failed = [c for c in checks if not c['pass']]
    print(f"sanity checks: {len(checks) - len(failed)} of {len(checks)} pass")
    for c in failed:
        print(f"  FAILED {c['check']} [{c['configuration']}]: {c['value']}")
    print(f'runtime {runtime:.0f} s, {len(results)} configurations, {len(skipped)} skipped')
    if quick:
        for r in results:
            print(r['name'], json.dumps(clean({k: r.get(k) for k in ('axis', 'spectrum', 'glide', 'behaviour',
                                                                      'false_wind', 'linear_model', 'reference')})))
        return
    write_outputs(OUT, results, skipped, agreement, runtime, n, checks)
    print(f"wrote {OUT/'descent_characterisation.json'} and .csv")


def write_outputs(out_dir, results, skipped, agreement, runtime, n, checks, merged=()):
    out_dir.mkdir(parents=True, exist_ok=True)
    document = {
        'label': 'SYNTHETIC', 'date': DATE,
        'what': 'Swing, spin, glide and false-wind characterisation of sim/nav/truth.py parachute descents',
        'warning': 'The canopy normal-force law is a generic assumed shape, not a measured canopy.',
        'seed': SEED, 'descents_per_configuration': n, 'integration_dt_s': DT, 'truth_rate_hz': 1/NAV_DT,
        'start_height_m': 300.0, 'transient_discarded_s': T_SKIP,
        'layers': {'top_m': LAYER_TOP, 'bottom_m': LAYER_BOTTOM, 'thickness_m': LAYER_DZ,
                   'min_samples': MIN_LAYER_SAMPLES},
        'false_wind_definition': 'w_hat = u + tau du/dt with u the IMU-point horizontal velocity, a centred '
                                 'difference of the 100 Hz truth and tau = mean descent speed / g; error against '
                                 'the true wind at the payload centre of mass; naive is w_hat = u. RMS values '
                                 'are per horizontal axis (east and north pooled).',
        'requirement_layer_rms_m_per_s': REQUIREMENT, 'earlier_fixed_pivot_false_wind_m_per_s': FIXED_PIVOT_FALSE_WIND,
        'literature': LITERATURE,
        'groups': {'class': 'the three classes the navigation Monte Carlo uses, turbulent wind',
                   'calm': 'calm-air repeats: true wind is zero, all air-relative motion is self-induced',
                   'dt_check': 'moderate class, 14 s, same descents at three integration steps',
                   'decay': 'calm air, start tilt sd 20 deg and rate sd 60 deg/s; decay measured from t = 0',
                   'stable': 'restoring canopies', 'unstable': 'canopies with a trim angle',
                   'line_benign': 'line length sweep at cn_k 1.0, trim 0 (0.45 m is mc_benign)',
                   'line_moderate': 'line length sweep at cn_k 0.4, trim 12 deg (0.45 m is mc_moderate)',
                   'added_mass': 'apparent mass sweep at cn_k 0.4, trim 12 deg (0.008 kg is mc_moderate)',
                   'extra': 'OUTSIDE the requested grid: cn_k 0.1 restoring canopy driven by the random canopy '
                            'side force (side_force_sd 0.05 N, correlation time 0.5 s) that truth.py provides'},
        'default_params': asdict(tr.Params()), 'default_wind': asdict(tr.WindParams()),
        'python': platform.python_version(), 'numpy': np.__version__, 'runtime_s': runtime,
        'skipped_for_time': skipped, 'merged_runs': list(merged), 'dt_check': agreement, 'sanity_checks': checks, 'configurations': results}
    with open(out_dir/'descent_characterisation.json', 'w', encoding='utf-8') as f:
        json.dump(clean(document), f, indent=1)
    with open(out_dir/'descent_characterisation.csv', 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['# SYNTHETIC', DATE, f'seed {SEED}', f'{n} descents per configuration',
                         'statistics after the first 4 s; false wind in m/s RMS per horizontal axis'])
        writer.writerow([name for name, _ in COLUMNS])
        for r in results:
            writer.writerow([cell(r, getter) for _, getter in COLUMNS])


if __name__ == '__main__':
    main()
