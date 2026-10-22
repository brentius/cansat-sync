"""GPL-3.0-only. End-to-end Monte Carlo of the wind retrieval and the landing prediction.

Usage: python sim/nav/run_wind_mc.py [--descents N] [--workers W] [--canopies a,b] [--report-only]

Each descent runs the whole chain on simulated data:

    sim/nav/truth.py          multibody descent through a frozen wind profile, to the ground
    code/cansat/eskf.py       the navigation filter (run_nav_mc.run_descent, baseline errors)
    code/cansat/windprofile   wind retrieval from the filter's velocity (image-derived), from a
                              simulated GNSS velocity (GPS-derived), with and without the can's
                              lateral specific force as an airspeed measurement
    code/cansat/landing.py    the flight landing predictor fed by GPS, by TRN and by dead reckoning

and scores it against the simulated truth: wind in 20 m layers (requirement
0.5 m/s), the S1 comparison of image-derived and GPS-derived winds, glide
recovery, and T3's landing-prediction error and calibration.

Everything here is SYNTHETIC. Input values carry the provenance tags used in
run_nav_mc.py: SOURCED, PROPOSAL, ASSUMED. The canopy classes are behaviours to
test against, not predictions for our canopy, and the can's side-force law is
the simulation's own (isotropic drag area), which no measurement supports yet.
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
import run_nav_mc as nm  # noqa: E402
import truth as tr  # noqa: E402
from cansat.eskf import ATT, POS, VEL  # noqa: E402
from cansat.landing import LandingPredictor  # noqa: E402
from cansat.windprofile import WindModel, lateral_airspeed, retrieve  # noqa: E402

OUT = HERE/'results'/'wind_mc'
CACHE = HERE/'generated'
DURATION = 31.0                                  # s, long enough to reach the ground from 300 m
FRAME = round(nm.FRAME_DT/nm.NAV_DT)             # navigation steps per image
EDGES = np.arange(0.0, 301.0, 20.0)              # PROPOSAL: 20 m layers
SCIENCE = slice(4, 15)                           # the layers from 80 m to 300 m (S1's range)
REQUIREMENT = 0.5                                # PROPOSAL: m/s RMS in 20 m layers
PREDICT_AT = (250.0, 200.0, 150.0, 100.0, 80.0, 50.0, 20.0)
V_REF = 10.5
PREDICTORS = ('gps', 'trn', 'dr', 'gps_glide', 'trn_glide')

# The four classes of run_nav_mc plus a mildly gliding one (trim 5 degrees).
CANOPIES = dict(nm.CANOPIES, mild=replace(tr.Params(), cn_k=0.4, trim_deg=5.0))
KEEP = nm.KEEP + ('wind_mean', 'wind_turb')


@dataclass
class Setup:
    """What the wind study assumes beyond the navigation Monte Carlo."""
    # PROPOSAL "GPS Doppler ~0.1 m/s". The MAX-M10S datasheet figure is 0.05 m/s
    # (50 %, at 30 m/s), seen only as a search excerpt (wiki/wind_science_parameter_sources.md).
    gps_velocity_sd: float = 0.1                 # m/s per axis per sample, white
    gps_position_sd: float = 1.5                 # m per axis, constant through a descent (ASSUMED)
    nav_error_tau: float = 2.0                   # s, correlation time told to the retrieval (ASSUMED from sim)
    nav_white_sd: float = 0.03                   # m/s, white part of the navigation velocity error
    # Lateral specific force as airspeed. The true side-force rate constant is the
    # simulation's 0.5*rho*CdS*v/m; the analysis is told it with this fractional error.
    side_force_error_sd: float = 0.10            # ASSUMED accuracy of a ground calibration
    side_force_bias: float = 0.0                 # the same error for every descent (sensitivity runs)
    # Body-fixed lateral offsets the navigation simulation does not contain:
    # cross-axis sensitivity and sensor-to-suspension misalignment acting on the 1 g
    # axial load. BMI088 typical 0.5 % and 0.5 degrees give 5-9 mg.
    extra_offset_sd: float = 0.07                # m/s^2 per axis (about 7 mg), constant (ASSUMED)
    hang_calibration_sd: float = 0.01            # m/s^2 (about 1 mg): what a static hang test leaves (ASSUMED)
    # Each airspeed sample is a Hann-weighted mean over this many 10 ms samples. It has to span
    # several periods of the can's wobble and of the swing (independent check, test_wind_independent.py).
    airspeed_window: int = 200
    airspeed_every: int = 100
    airspeed_sd: float = 0.4                     # m/s per sample told to the retrieval (ASSUMED from sim)
    glide_sd: float = 2.5                        # m/s, prior width of the glide
    glide_tau: float = 20.0                      # s
    anemometer_sd: float = 0.3                   # m/s, mean wind at 6 m from the ground mast (ASSUMED)
    anemometer_height: float = 6.0               # PROPOSAL
    model_sigma: float = 1.0                     # m/s, turbulence intensity told to the retrieval (truth is 1.0)


SETUP = Setup()


# --- Truth to the ground ---------------------------------------------------------

def truth_dir(canopy, n):
    key = repr((asdict(CANOPIES[canopy]), asdict(tr.WindParams()), DURATION, nm.NAV_DT, nm.FRAME_DT, nm.DELAYS))
    return CACHE/f'windtruth_{canopy}_n{n}_seed{nm.SEED}_{hashlib.sha256(key.encode()).hexdigest()[:8]}'


def build_truth(canopy, n):
    directory = truth_dir(canopy, n)
    if (directory/'done').exists():
        return directory
    directory.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng([nm.SEED, sorted(CANOPIES).index(canopy), 77])
    start = time.perf_counter()
    data = tr.simulate(CANOPIES[canopy], n, rng, duration=DURATION, nav_dt=nm.NAV_DT, frame_dt=nm.FRAME_DT,
                       delays=nm.DELAYS)
    for key in KEEP:
        np.save(directory/f'{key}.npy', data[key])
    (directory/'done').write_text('%.1f s' % (time.perf_counter() - start))
    return directory


def prepare(canopy, n):
    """Make run_nav_mc see this study's truth batch and scenarios (once per process)."""
    if (canopy, n) not in nm._TRUTH:
        directory = truth_dir(canopy, n)
        nm._TRUTH[(canopy, n)] = {k: np.load(directory/f'{k}.npy', mmap_mode='r') for k in KEEP}
    nm.CANOPIES.setdefault(canopy, CANOPIES[canopy])
    nm.SCENARIOS.setdefault('wind_' + canopy, nm.scenario(canopy, note='baseline errors, descent to the ground'))
    nm.SCENARIOS.setdefault('wind_dr_' + canopy, nm.scenario(canopy, filt=dict(map_floor=1e9),
                                                             note='IMU and barometer only after the first fix'))
    return nm._TRUTH[(canopy, n)]


# --- Retrieval variants -------------------------------------------------------------

def base_model(setup, **kw):
    return WindModel(sigma=setup.model_sigma, lag_time=V_REF/tr.G0, **kw)


def variants(setup, kappa_assumed):
    """name -> (velocity source, model, uses airspeed, offset known from a hang test)."""
    nav = dict(error_sd=1.0, error_tau=setup.nav_error_tau)
    glide = dict(glide_sd=setup.glide_sd, glide_tau=setup.glide_tau)
    free = setup.extra_offset_sd/kappa_assumed + 0.02*tr.G0/kappa_assumed   # uncalibrated: everything body-fixed
    held = setup.hang_calibration_sd/kappa_assumed
    return {
        'gps': ('gps', base_model(setup), False, False),
        'trn': ('trn', base_model(setup, **nav), False, False),
        'trn_white': ('trn_white', base_model(setup), False, False),
        'gps_glide_state_only': ('gps', base_model(setup, **glide), False, False),
        'gps_air': ('gps', base_model(setup, offset_sd=free, **glide), True, False),
        'gps_air_cal': ('gps', base_model(setup, offset_sd=held, **glide), True, True),
        'trn_air_cal': ('trn', base_model(setup, offset_sd=held, **glide, **nav), True, True),
        'gps_air_cal_no_canopy_model': ('gps', replace(base_model(setup, offset_sd=held), lag_time=None), True, True),
    }


# --- One descent -------------------------------------------------------------------

def layer_truth(profile, dz):
    """True layer-mean horizontal wind (layers, 2) from one frozen profile on a dz grid."""
    out = np.empty((len(EDGES) - 1, 2))
    for i in range(len(EDGES) - 1):
        lo, hi = round(EDGES[i]/dz), round(EDGES[i + 1]/dz)
        out[i] = profile[lo:hi + 1, :2].mean(axis=0)
    return out


def run_descent(job):
    canopy, j, n, setup = job
    truth = prepare(canopy, n)
    p = CANOPIES[canopy]
    rec = nm.run_descent(('wind_' + canopy, j, n))
    rec_dr = nm.run_descent(('wind_dr_' + canopy, j, n))
    t_p, t_v, t_r = (np.asarray(truth[k][:, j]) for k in ('p', 'v', 'r'))
    air = np.asarray(truth['air_velocity'][:, j])
    profile = np.asarray(truth['wind_mean'][j]) + np.asarray(truth['wind_turb'][j])
    dz = tr.WindParams().dz
    rng = np.random.default_rng([nm.SEED, j, 40])

    # Touchdown: first crossing of the ground by the navigated point.
    below = np.flatnonzero(t_p[:, 2] <= 0.0)
    k_land = int(below[0]) if len(below) else len(t_p) - 1
    f = t_p[k_land - 1, 2]/(t_p[k_land - 1, 2] - t_p[k_land, 2]) if k_land else 0.0
    landing = t_p[k_land - 1, :2] + f*(t_p[k_land, :2] - t_p[k_land - 1, :2])
    t_land = (k_land - 1 + f)*nm.NAV_DT

    frames = rec['err'].shape[0]
    fk = np.arange(frames)*FRAME
    ok = (fk < k_land) & np.isfinite(rec['err'][:, 0])
    fk = fk[ok]
    t = fk*nm.NAV_DT
    err, sig = rec['err'][ok].astype(float), rec['sig'][ok].astype(float)
    p_nav, v_nav = t_p[fk] - err[:, POS], t_v[fk] - err[:, VEL]
    z = np.maximum(p_nav[:, 2], 0.05)
    in_map = z >= nm.FilterCfg().map_floor
    v_gps = t_v[fk, :2] + setup.gps_velocity_sd*rng.normal(size=(len(fk), 2))
    gps_bias = setup.gps_position_sd*rng.normal(size=2)

    # Lateral specific force as airspeed, from exactly the samples the filter was given.
    sensors = nm.SCENARIOS['wind_' + canopy]['sensors']
    accel, gyro, ba_true, _, _, _ = nm.imu_stream(sensors, truth, j)
    extra = setup.extra_offset_sd*rng.normal(size=2)
    accel = accel.copy()
    accel[:, :2] += extra
    steps = len(accel)
    grid = np.arange(steps)
    att_err = np.column_stack([np.interp(grid, fk, err[:, a]) for a in (6, 7, 8)])
    r_nav = np.einsum('kij,kjl->kil', t_r[:steps], np.array([nm.exp_so3(-e) for e in att_err]))
    v_up = t_v[:steps, 2] - np.interp(grid, fk, err[:, 5])
    kappa_true = 0.5*tr.RHO*p.payload_cds*V_REF/p.payload_mass
    kappa = kappa_true*(1 + setup.side_force_bias + setup.side_force_error_sd*rng.normal())
    offset_true = ba_true[:, :2].mean(axis=0) + extra
    span = (int(setup.airspeed_window), int(setup.airspeed_every))
    centres, q_raw, q_map = lateral_airspeed(accel, gyro, r_nav, v_up, nm.NAV_DT, kappa, V_REF, p.imu, *span)
    # A hang test measures the whole body-fixed offset; what is left is its error.
    known = offset_true + setup.hang_calibration_sd*rng.normal(size=2)
    _, q_cal, _ = lateral_airspeed(accel - np.r_[known, 0.0], gyro, r_nav, v_up, nm.NAV_DT, kappa, V_REF, p.imu, *span)
    half = int(setup.airspeed_window)//2
    weights = np.hanning(2*half + 3)[1:-1]
    weights /= weights.sum()
    slot = {int(c): i for i, c in enumerate(centres)}
    air_true = np.full((len(fk), 2), np.nan)
    air_raw, air_cal = air_true.copy(), air_true.copy()
    air_map = np.zeros((len(fk), 2, 2))
    for i, k in enumerate(fk):
        m = slot.get(int(k))
        if m is not None and t[i] >= 2.0:
            air_raw[i], air_cal[i], air_map[i] = q_raw[m], q_cal[m], q_map[m]
            air_true[i] = weights @ air[k - half:k + half + 1, :2]

    truth_layers = layer_truth(profile, dz)
    out = {'truth_layers': truth_layers, 'landing': landing, 't_land': t_land,
           'glide_true': air[300:k_land, :2].mean(axis=0),
           'airspeed_error_cal': np.sqrt(np.nanmean((air_cal - air_true)**2)),
           'airspeed_error_raw': np.sqrt(np.nanmean((air_raw - air_true)**2)),
           'airspeed_true_rms': np.sqrt(np.nanmean(air_true**2)),
           'nav_velocity_rms': np.sqrt(np.mean(err[in_map & (t >= 2.0), 3:5]**2)),
           'turns': abs(np.unwrap(np.arctan2(t_r[300:k_land, 1, 0], t_r[300:k_land, 0, 0]))[[0, -1]] @ [-1, 1])/(2*math.pi)}

    # Direct inversion w = u + tau du/dt on the GPS velocity, for reference.
    tau = V_REF/tr.G0
    direct = v_gps[1:-1] + tau*(v_gps[2:] - v_gps[:-2])/(t[2:] - t[:-2])[:, None]
    layer = np.floor(z[1:-1]/20.0).astype(int)
    out['direct_mean'] = np.array([direct[layer == i].mean(axis=0) if np.any(layer == i) else [np.nan, np.nan]
                                   for i in range(len(EDGES) - 1)])

    for name, (source, model, use_air, calibrated) in variants(setup, kappa).items():
        if source == 'gps':
            vel, var, gain = v_gps, setup.gps_velocity_sd**2, None
        else:
            vel = np.where(in_map[:, None], v_nav[:, :2], np.nan)
            if source == 'trn':
                var, gain = setup.nav_white_sd**2, sig[:, 3:5]
            else:
                var, gain = sig[:, 3:5]**2, None
        result = retrieve(model, t, z, vel, var, (air_cal if calibrated else air_raw) if use_air else None,
                          air_map if use_air else None, setup.airspeed_sd**2, error_gain=gain)
        _, mean, sd, count = result.layer_means(EDGES)
        out[name + '_mean'], out[name + '_sd'] = mean, sd
        out[name + '_nis'] = float(np.nanmean(result.nis))
        out[name + '_glide'] = result.glide[len(t)//2]
        out[name + '_glide_sd'] = np.sqrt(result.glide_var[len(t)//2])
        out[name + '_c'] = result.mean_wind[-1]
    out['count'] = count

    # T3: the flight predictor fed by three sources (proposal section 13).
    wp = tr.WindParams()
    anem = (np.asarray(truth['wind_mean'][j])[round(setup.anemometer_height/dz), :2]
            + setup.anemometer_sd*rng.normal(size=2))
    anemometer = (setup.anemometer_height, anem, setup.anemometer_sd)
    err_dr, sig_dr = rec_dr['err'][ok].astype(float), rec_dr['sig'][ok].astype(float)
    p_dr = t_p[fk] - err_dr[:, POS]
    feeds = {
        'gps': (LandingPredictor(WindModel(roughness=wp.roughness), V_REF, anemometer),
                lambda i: (t_p[fk[i]] + np.r_[gps_bias, 0.0], v_gps[i], setup.gps_velocity_sd**2,
                           np.eye(2)*setup.gps_position_sd**2, (1.0, 1.0))),
        'trn': (LandingPredictor(WindModel(roughness=wp.roughness, error_sd=1.0, error_tau=setup.nav_error_tau),
                                 V_REF, anemometer),
                lambda i: (p_nav[i], v_nav[i, :2] if in_map[i] else None, setup.nav_white_sd**2,
                           np.diag(sig[i, 0:2]**2), sig[i, 3:5])),
        'dr': (LandingPredictor(WindModel(roughness=wp.roughness), V_REF, anemometer),
               lambda i: (p_dr[i], None, None, np.diag(sig_dr[i, 0:2]**2), (1.0, 1.0))),
    }
    # The same two with a glide state: a persistent air-relative velocity carries to the ground.
    glide = dict(glide_sd=setup.glide_sd, glide_tau=setup.glide_tau)
    feeds['gps_glide'] = (LandingPredictor(WindModel(roughness=wp.roughness, **glide), V_REF, anemometer), feeds['gps'][1])
    feeds['trn_glide'] = (LandingPredictor(WindModel(roughness=wp.roughness, error_sd=1.0, error_tau=setup.nav_error_tau,
                                                     **glide), V_REF, anemometer), feeds['trn'][1])
    for name, (predictor, feed) in feeds.items():
        e = np.full((len(PREDICT_AT), 2), np.nan)
        d2 = np.full(len(PREDICT_AT), np.nan)
        dt_go = np.full(len(PREDICT_AT), np.nan)
        radius = np.full(len(PREDICT_AT), np.nan)
        taken = 0
        for i in range(len(t)):
            position, velocity, var, pos_cov, gain = feed(i)
            pred = predictor.update(t[i], position, velocity, var, pos_cov, gain)
            while pred is not None and taken < len(PREDICT_AT) and t_p[fk[i], 2] <= PREDICT_AT[taken]:
                e[taken] = np.array([pred.east, pred.north]) - landing
                d2[taken] = e[taken] @ np.linalg.solve(pred.cov, e[taken])
                dt_go[taken] = t[i] + pred.time_to_go - t_land
                radius[taken] = math.sqrt(np.trace(pred.cov))
                taken += 1
        out[f'pred_{name}_err'], out[f'pred_{name}_d2'] = e, d2
        out[f'pred_{name}_time'], out[f'pred_{name}_sigma'] = dt_go, radius
        out[f'pred_{name}_ignored'] = predictor.ignored + predictor.gated + predictor.resets + predictor.faults
    return {k: np.asarray(v, dtype=np.float32) for k, v in out.items()}


# --- Scoring -------------------------------------------------------------------------

def rms(a):
    return float(np.sqrt(np.nanmean(np.asarray(a, dtype=float)**2)))


def summarise(canopy, stack, setup):
    truth = stack['truth_layers']                          # (n, layers, 2)
    names = list(variants(setup, 1.0)) + ['direct']
    result = {'descents': int(truth.shape[0]),
              'true_glide_rms_per_axis': rms(stack['glide_true']),
              'true_glide_speed_mean': float(np.mean(np.hypot(*stack['glide_true'].T))),
              'airspeed_sample_error_rms_calibrated': float(np.mean(stack['airspeed_error_cal'])),
              'airspeed_sample_error_rms_uncalibrated': float(np.mean(stack['airspeed_error_raw'])),
              'airspeed_true_rms': float(np.mean(stack['airspeed_true_rms'])),
              'navigation_velocity_rms_per_axis': float(np.mean(stack['nav_velocity_rms'])),
              'yaw_turns_median': float(np.median(stack['turns'])),
              'wind': {}, 's1': {}, 'landing': {}}
    for name in names:
        err = stack[name + '_mean'] - truth
        entry = {'layer_rms_80_300': rms(err[:, SCIENCE]), 'layer_rms_0_80': rms(err[:, :4]),
                 'layer_rms_by_layer': [rms(err[:, i]) for i in range(err.shape[1])],
                 'descent_mean_bias_rms_80_300': rms(np.nanmean(err[:, SCIENCE], axis=1)),
                 'meets_requirement': bool(rms(err[:, SCIENCE]) <= REQUIREMENT)}
        if name != 'direct':
            sd = stack[name + '_sd']
            inside = np.abs(err[:, SCIENCE]) <= 1.959964*sd[:, SCIENCE]
            entry.update({'stated_sd_80_300': rms(sd[:, SCIENCE]),
                          'coverage95_80_300': float(np.nanmean(inside[np.isfinite(err[:, SCIENCE])])),
                          'nis_mean': float(np.mean(stack[name + '_nis'])),
                          'glide_error_rms': rms(stack[name + '_glide'] - stack['glide_true']),
                          'glide_stated_sd': rms(stack[name + '_glide_sd'])})
        result['wind'][name] = entry
    for a, b in (('trn', 'gps'), ('trn_white', 'gps'), ('trn_air_cal', 'gps_air_cal')):
        diff = stack[a + '_mean'] - stack[b + '_mean']
        by_layer = [rms(diff[:, i]) for i in range(diff.shape[1])]
        heights = (EDGES[:-1] + 10.0)[SCIENCE]
        slope = float(np.polyfit(heights, np.array(by_layer)[SCIENCE], 1)[0])
        result['s1'][f'{a}_minus_{b}'] = {'layer_rms_80_300': rms(diff[:, SCIENCE]), 'layer_rms_by_layer': by_layer,
                                          'slope_rms_per_100m': 100*slope,
                                          'passes_0p5': bool(rms(diff[:, SCIENCE]) <= REQUIREMENT)}
    for name in PREDICTORS:
        e = stack[f'pred_{name}_err']
        d2 = stack[f'pred_{name}_d2']
        result['landing'][name] = {
            'height_m': list(PREDICT_AT),
            'radial_rms_m': [rms(np.hypot(e[:, i, 0], e[:, i, 1])) for i in range(len(PREDICT_AT))],
            'stated_sigma_m': [rms(stack[f'pred_{name}_sigma'][:, i]) for i in range(len(PREDICT_AT))],
            'coverage95': [float(np.nanmean(d2[:, i] <= -2*math.log(0.05))) for i in range(len(PREDICT_AT))],
            'mahalanobis2_mean': [float(np.nanmean(d2[:, i])) for i in range(len(PREDICT_AT))],
            'time_error_rms_s': [rms(stack[f'pred_{name}_time'][:, i]) for i in range(len(PREDICT_AT))],
            'samples_ignored': float(np.sum(stack[f'pred_{name}_ignored']))}
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--descents', type=int, default=160)
    parser.add_argument('--workers', type=int, default=6)
    parser.add_argument('--canopies', default='')
    parser.add_argument('--report-only', action='store_true')
    parser.add_argument('--tag', default='')
    parser.add_argument('--set', default='', help='Setup overrides for a sensitivity run, e.g. model_sigma=2,side_force_bias=-0.3')
    args = parser.parse_args()
    setup = replace(SETUP, **{k: float(v) for k, v in (item.split('=') for item in args.set.split(',') if item)})
    if args.set and not args.tag:
        raise SystemExit('a sensitivity run needs --tag so it cannot overwrite the baseline results')
    names = [x for x in args.canopies.split(',') if x] or list(CANOPIES)
    out_dir = OUT/args.tag if args.tag else OUT
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir/'summary.json'
    report = json.loads(path.read_text()) if path.exists() else {}
    report.update({'label': 'SYNTHETIC', 'seed': nm.SEED, 'setup': asdict(setup), 'layer_edges_m': EDGES.tolist(),
                   'requirement_mps': REQUIREMENT, 'wind_truth': asdict(tr.WindParams())})
    report.setdefault('canopies', {})
    for canopy in names:
        raw = CACHE/f'windraw_{args.tag + "_" if args.tag else ""}{canopy}_n{args.descents}.npz'
        if not args.report_only:
            t0 = time.perf_counter()
            build_truth(canopy, args.descents)
            print('truth %-9s ready (%.0f s)' % (canopy, time.perf_counter() - t0), flush=True)
            t0 = time.perf_counter()
            with Pool(args.workers) as pool:
                recs = pool.map(run_descent, [(canopy, j, args.descents, setup) for j in range(args.descents)],
                                chunksize=2)
            stack = {k: np.stack([r[k] for r in recs]) for k in recs[0]}
            np.savez_compressed(raw, **stack)
            print('%-9s chain done (%.0f s)' % (canopy, time.perf_counter() - t0), flush=True)
        stack = {k: v.astype(float) for k, v in np.load(raw).items()}
        summary = summarise(canopy, stack, setup)
        summary['canopy_params'] = asdict(CANOPIES[canopy])
        report['canopies'][canopy] = summary
        w, s1 = summary['wind'], summary['s1']['trn_minus_gps']
        print('%-9s glide %.2f m/s | wind RMS 80-300 m: gps %.2f trn %.2f gps+air(cal) %.2f gps+air %.2f '
              'no-model %.2f direct %.2f | S1 %.2f | cov95 gps %.2f air %.2f | land RMS at 100 m gps %.1f trn %.1f dr %.1f' % (
                  canopy, summary['true_glide_speed_mean'], w['gps']['layer_rms_80_300'], w['trn']['layer_rms_80_300'],
                  w['gps_air_cal']['layer_rms_80_300'], w['gps_air']['layer_rms_80_300'],
                  w['gps_air_cal_no_canopy_model']['layer_rms_80_300'], w['direct']['layer_rms_80_300'],
                  s1['layer_rms_80_300'], w['gps']['coverage95_80_300'], w['gps_air_cal']['coverage95_80_300'],
                  summary['landing']['gps']['radial_rms_m'][3], summary['landing']['trn']['radial_rms_m'][3],
                  summary['landing']['dr']['radial_rms_m'][3]), flush=True)
        path.write_text(json.dumps(report, indent=1))


if __name__ == '__main__':
    main()
