"""GPL-3.0-only. SYNTHETIC Monte Carlo of the first fix: rendered images, the flight acquisition code, simulated descents.

Usage: python sim/nav/run_acquisition_mc.py [--trials N] [--max-trials N] [--workers W] [--only a,b] [--list] [--summarise]
Writes sim/nav/results/acquisition/{summary.json, attempts.csv, tables.md, *.png}. Only NumPy and Pillow.

What is in the loop. For each trial a simulated descent (sim/nav/truth.py, the batches the navigation Monte
Carlo uses) is placed at a random offset from the launch pad over a procedural farmland world
(sim/nav/farmland.py). The simulated IMU (with the Monte Carlo's sensor errors) drives the flight Leveller
from the start of the descent. At each attempt time an image is rendered at the TRUE pose from the world's
flight-day raster (sim/nav/scene.py) and passed, with the levelled attitude and a barometric height, to
code/cansat/coarse.Acquirer, which is given the map-day raster at 1 m/px. Truth is used only to score.

A fix is CORRECT when it is within 10 m horizontally of the truth (the T1 requirement) and FALSE otherwise. A
false fix is the outcome that matters most, because neither the solver nor the navigation filter can detect
one afterwards (wiki/navigation_closed_loop_simulation.md, section 9).

Position error is not the whole story for what happens next. The fine matcher that takes over looks for each
landmark within a window around where the fix says it should appear, so what it needs is that the fix puts
the ground under each pixel in nearly the right place. A fix that is wrong along the direction in which a tilt
and a shift cancel can be 10 m out in position and still do that. The CAPTURE error is therefore recorded too:
the largest distance, over a grid of pixels in the central 80 % of the image, between the ground point the fix
assigns to a pixel and the true one. 12 m is the position error the fine matcher tolerated in
sim/nav/ncc_attitude_tolerance.py.

Everything here is synthetic. The terrain is a schematic of English farmland whose brightness values are
assumed, not measured from any orthophoto; its seasonal change is a model (fields redrawn, shadows moved);
the ground is flat; the camera is an ideal pinhole with a Gaussian blur. Host timings are from a shared
laptop. Nothing was run on the Raspberry Pi.
"""
import os

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_name, '1')           # one core per worker, like the flight computer

import argparse                                 # noqa: E402
import csv                                      # noqa: E402
import json                                     # noqa: E402
import math                                     # noqa: E402
import platform                                 # noqa: E402
import sys                                      # noqa: E402
import time                                     # noqa: E402
from dataclasses import dataclass, field, replace   # noqa: E402
from multiprocessing import Pool                # noqa: E402
from pathlib import Path                        # noqa: E402

import numpy as np                              # noqa: E402

HERE = Path(__file__).resolve().parent
sys.dont_write_bytecode = True
sys.path.insert(0, str(HERE))
import farmland                                 # noqa: E402
import run_nav_mc as mc                         # noqa: E402  (also puts code/ on sys.path)
import scene                                    # noqa: E402
from cansat import coarse                       # noqa: E402
from cansat.coarse import Acquirer, CoarseConfig, CoarseMap, Leveller, bin_camera, heading_between, wrap   # noqa: E402

OUT = HERE/'results'/'acquisition'
DATE = '2026-10-11'
SEED = 20261011
WORLD_SEED = 2027
WORLDS = 4                                       # worlds per scenario; trial j uses world j % WORLDS
BINNING = 2                                      # images are rendered at half the sensor resolution
SIZE = (mc.IMAGE[1]//BINNING, mc.IMAGE[0]//BINNING)
CORRECT_M = 10.0                                 # T1: horizontal error
CAPTURE_M = 12.0                                 # the fine matcher's measured tolerance to position error
DEG = math.pi/180
COLUMNS = ('scenario', 'trial', 'world', 'attempt', 't', 'height', 'tilt_deg', 'level_err_deg', 'ok', 'reason',
           'status', 'polished', 's1_pos_err', 's1_yaw_err_deg', 's1_score', 's1_lead', 'rank', 'sites', 'matches',
           'inliers', 'err_h', 'err_z', 'err_att_deg', 'sigma_h', 'coarse_err_h', 'match_sd', 'match_far',
           'vel_err', 'ffts_256', 'ffts_128', 'host_s', 'seen', 'capture_rms', 'capture_max')


@dataclass
class Scenario:
    note: str = ''
    style: str = 'mixed'
    season: dict = field(default_factory=dict)       # farmland.Season overrides
    canopy: str = 'moderate'
    times: tuple = (2.0, 3.0, 4.0, 5.0, 6.0, 7.0)    # attempt times, s after the canopy has settled
    levelling: str = 'imu'                           # imu (flight Leveller) | truth (perfect tilt) | none (gyro only)
    compass_sd_deg: float = 0.0                      # if > 0: a yaw prior with this error, searched +/-15 deg
    map_res: float = 1.0                             # 1 m/px (stage 4 on) or 2 m/px
    height_sd: float = 0.03                          # fractional barometric height error, fixed per descent
    dispersion_m: float = 200.0                      # radius of the disc the descent is placed in
    blur_px: float = 0.6                             # optical blur at the rendered (half) resolution
    noise: float = 1.0                               # grey levels per rendered pixel
    config: dict = field(default_factory=dict)       # coarse.CoarseConfig overrides
    trials: int = 0                                  # 0: the command-line default


S = Scenario
IDENTICAL = dict(albedo_change=0.0, tramline_change=0.0, sun_flight=(180.0, 55.0))
SCENARIOS = {
    'baseline': S('mixed farmland, default season, moderate canopy', trials=96),
    # Terrain.
    'style_arable': S('large fields', style='arable', trials=96),
    'style_pasture': S('small fields, many hedges and trees', style='pasture', trials=96),
    'style_open': S('very large fields, few boundaries', style='open', trials=96),
    'style_tramline': S('tramlines strong, boundaries faint', style='tramline', trials=96),
    # What changed between the map and the flight.
    'season_identical': S('map and ground identical', season=IDENTICAL),
    'season_shadows_only': S('only the sun has moved', season=dict(albedo_change=0.0, tramline_change=0.0)),
    'season_fields_only': S('every field redrawn, same sun', season=dict(albedo_change=1.0, sun_flight=(180.0, 55.0))),
    'season_all_fields': S('every field redrawn, sun moved', season=dict(albedo_change=1.0)),
    'season_low_sun': S('sun at 20 degrees elevation', season=dict(sun_flight=(140.0, 20.0))),
    'season_leaf_off': S('hedges and trees bare', season=dict(leaf_off=1.0)),
    'season_exposure': S('gain 0.7, gamma 1.3, offset 10', season=dict(gain=0.7, gamma=1.3, offset=10.0)),
    'arable_identical': S('large fields, map and ground identical', style='arable', season=IDENTICAL),
    'arable_all_fields': S('large fields, every field redrawn', style='arable', season=dict(albedo_change=1.0)),
    # Canopy behaviour.
    'canopy_benign': S('stable canopy', canopy='benign'),
    'canopy_severe': S('25 degree trim, gliding', canopy='severe'),
    'canopy_violent': S('19 degree RMS swing', canopy='violent'),
    'violent_arable': S('19 degree RMS swing over large fields', canopy='violent', style='arable'),
    # What the acquisition is given.
    'compass': S('yaw known to 5 degrees (magnetometer), searched +/-15', compass_sd_deg=5.0),
    'compass_arable': S('the same over large fields', compass_sd_deg=5.0, style='arable'),
    'level_perfect': S('tilt known exactly', levelling='truth'),
    'level_none_violent': S('no levelling (gyro only), violent canopy', levelling='none', canopy='violent'),
    'height_error_8pct': S('barometric height 8 % out (1 sigma)', height_sd=0.08),
    'edge_of_prior': S('descents up to 280 m from the pad', dispersion_m=280.0),
    'blurred_noisy': S('blur 1.5 px and 4 grey levels of noise', blur_px=1.5, noise=4.0),
    # Design choices.
    'map_2m_only': S('2 m/px map: no stage 4', map_res=2.0),
    'feature_intensity': S('brightness correlated instead of edges', config=dict(feature='intensity')),
    'feature_intensity_identical': S('brightness, map and ground identical', config=dict(feature='intensity'), season=IDENTICAL),
    'yaw_step_10': S('yaw hypotheses every 10 degrees', config=dict(yaw_step_deg=10.0)),
    'yaw_step_15': S('yaw hypotheses every 15 degrees', config=dict(yaw_step_deg=15.0)),
    'one_candidate': S('only the best stage-1 peak is tried', config=dict(candidates=1)),
    'interim_similarity': S('stage 3 by a fit at the levelled tilt instead of the solver', config=dict(interim='similarity')),
    'interim_similarity_arable': S('the same over large fields', style='arable', config=dict(interim='similarity')),
    'strict_interim': S('stage 3 under the full solver checks', config=dict(solver_interim={})),
    'strict_interim_arable': S('the same over large fields', style='arable', config=dict(solver_interim={})),
    'window_10m_arable': S('stage 4 search of 10 m, large fields', style='arable', config=dict(polish_search_px=10)),
    'no_tilt_gate_arable': S('large fields, no check of the fix against the levelled tilt', style='arable',
                             config=dict(max_tilt_disagreement_deg=90.0)),
    # Lower down (re-acquisition after a loss of lock).
    'low_200m': S('attempts at 205-165 m', times=(9.0, 10.0, 11.0, 12.0, 13.0)),
    'low_130m': S('attempts at 142-100 m', times=(15.0, 16.0, 17.0, 18.0, 19.0)),
    'low_200m_arable': S('attempts at 205-165 m over large fields', times=(9.0, 10.0, 11.0, 12.0, 13.0), style='arable'),
}


def trials_of(name, default, cap=0):
    trials = SCENARIOS[name].trials or default
    return min(trials, cap) if cap else trials


def capture_error(cam, fix_p, fix_r, p, r):
    """RMS and largest ground distance between where the fix and the truth put a grid of pixels."""
    u, v = np.meshgrid(np.linspace(0.1, 0.9, 7)*(SIZE[1] - 1), np.linspace(0.1, 0.9, 5)*(SIZE[0] - 1))
    body = cam.ray(np.column_stack((u.ravel(), v.ravel())))
    ground = []
    for pos, rot in ((fix_p, fix_r), (p, r)):
        rays = body @ rot.T
        centre = pos + rot @ cam.lever
        if np.any(rays[:, 2] > -1e-3):
            return math.nan, math.nan
        ground.append(centre[:2] + rays[:, :2]*(centre[2]/-rays[:, 2])[:, None])
    d = np.hypot(*(ground[0] - ground[1]).T)
    return float(np.sqrt(np.mean(d*d))), float(d.max())


def run_chunk(job):
    """All trials of one scenario that share one world. Returns the attempt rows."""
    name, wi, trials = job
    sc = SCENARIOS[name]
    cfg = replace(CoarseConfig(), **sc.config)
    world = farmland.generate(WORLD_SEED + wi, style=sc.style, season=farmland.Season(**sc.season))
    pyramid = scene.Pyramid(world.flight, world.res, world.origin)
    cmap = CoarseMap(*scene.onboard_map(world.map, world.res, world.origin, res2=sc.map_res), config=cfg)
    del world
    truth = mc.load_truth(sc.canopy, 160)
    cam = bin_camera(mc.nominal_camera(), BINNING)
    sensors = mc.Sensors()
    rows = []
    for j in range(wi, trials, WORLDS):
        rng = np.random.default_rng([SEED, j, 11])
        radius, bearing = sc.dispersion_m*math.sqrt(rng.uniform()), rng.uniform(0, 2*math.pi)
        offset = np.array([radius*math.cos(bearing), radius*math.sin(bearing), 0.0])
        height_scale = 1 + sc.height_sd*rng.normal()
        compass_error = sc.compass_sd_deg*DEG*rng.normal()
        frame_heading = rng.uniform(-math.pi, math.pi)
        accel, gyro, _, _, baro_bias, baro_noise = mc.imu_stream(sensors, truth, j)
        leveller = Leveller()
        acquirer = Acquirer(cmap)
        k = 0
        for attempt, t in enumerate(sc.times):
            k_end = int(round(t/mc.NAV_DT))
            while k < k_end:
                leveller.update(mc.NAV_DT, accel[k], gyro[k])
                k += 1
            p = np.asarray(truth['p'][k, j]) + offset
            r = np.asarray(truth['r'][k, j])
            if sc.levelling == 'truth':
                r0 = coarse._rz(frame_heading) @ r
            elif sc.levelling == 'none':
                r0 = leveller.rg.copy()
            else:
                r0 = leveller.attitude()
            up_true, up_est = r.T @ [0.0, 0.0, 1.0], r0.T @ [0.0, 0.0, 1.0]
            level_err = math.acos(max(-1.0, min(1.0, float(up_true @ up_est))))
            psi_true = heading_between(r, r0)
            prior = (psi_true + compass_error, 15*DEG) if sc.compass_sd_deg > 0 else None
            image, seen = scene.render(pyramid, cam, p, r, SIZE, blur_px=sc.blur_px, noise=sc.noise,
                                       rng=np.random.default_rng([SEED, j, 12, attempt]), quantise=True)
            height = p[2]*height_scale + baro_bias[k] + baro_noise[k]
            started = time.perf_counter()
            status, acq, velocity = acquirer.update(t, image, cam, r0, height, yaw_prior=prior)
            host = time.perf_counter() - started
            row = dict.fromkeys(COLUMNS, math.nan)
            row.update(scenario=name, trial=j, world=wi, attempt=attempt, t=t, height=p[2],
                       tilt_deg=math.acos(max(-1.0, min(1.0, -r[2, 2])))/DEG, level_err_deg=level_err/DEG,
                       ok=int(acq.ok), reason=acq.reason, status=status, polished=int(acq.polished), rank=acq.rank,
                       sites=acq.sites, matches=acq.matches, s1_score=acq.score, s1_lead=acq.lead,
                       ffts_256=acq.ffts.get(256, 0), ffts_128=acq.ffts.get(128, 0), host_s=host, seen=seen)
            if acq.candidates:
                c = acq.candidates[0]
                row.update(s1_pos_err=math.hypot(c[1] - p[0], c[2] - p[1]), s1_yaw_err_deg=wrap(c[0] - psi_true)/DEG)
            if acq.coarse_fix is not None and acq.coarse_fix.p is not None:
                row['coarse_err_h'] = math.hypot(*(acq.coarse_fix.p - p)[:2])
            if acq.ok:
                fix = acq.fix
                e = fix.p - p
                m = fix.r.T @ r
                rays = cam.ray(acq.pixels) @ r.T
                centre = p + r @ cam.lever
                ground = centre[:2] + rays[:, :2]*(centre[2]/-rays[:, 2])[:, None]
                d = acq.points[:, :2] - ground
                far = np.hypot(d[:, 0], d[:, 1]) > 4.0
                row['capture_rms'], row['capture_max'] = capture_error(cam, fix.p, fix.r, p, r)
                row.update(err_h=math.hypot(e[0], e[1]), err_z=e[2],
                           err_att_deg=math.acos(max(-1.0, min(1.0, (np.trace(m) - 1)/2)))/DEG,
                           sigma_h=math.sqrt(max(float(fix.cov[0, 0] + fix.cov[1, 1]), 0.0)),
                           inliers=int(fix.inliers.sum()), match_far=int(far.sum()),
                           match_sd=float(np.sqrt(np.mean(d[~far]**2))) if (~far).any() else math.nan)
                if velocity is not None:
                    row['vel_err'] = float(np.hypot(*(velocity[:2] - np.asarray(truth['v'][k, j])[:2])))
            rows.append(row)
    return rows


# --- Scoring ---------------------------------------------------------------------

def rate(k, n):
    """Fraction with a Wilson 95 % interval, as [value, low, high]."""
    if n == 0:
        return [math.nan, math.nan, math.nan]
    z, p = 1.96, k/n
    centre, half = (p + z*z/(2*n))/(1 + z*z/n), z*math.sqrt(p*(1 - p)/n + z*z/(4*n*n))/(1 + z*z/n)
    return [p, max(0.0, centre - half), min(1.0, centre + half)]


def stats(values):
    v = np.asarray([x for x in values if math.isfinite(x)], dtype=float)
    if len(v) == 0:
        return {'n': 0}
    return {'n': int(len(v)), 'rms': float(np.sqrt(np.mean(v*v))), 'median': float(np.median(v)),
            'p95': float(np.percentile(v, 95)), 'max': float(v.max()), 'mean': float(v.mean())}


def summarise(name, rows):
    sc = SCENARIOS[name]
    trials = sorted({r['trial'] for r in rows})
    by_trial = {j: sorted((r for r in rows if r['trial'] == j), key=lambda r: r['attempt']) for j in trials}
    n = len(trials)
    fixes = [r for r in rows if r['ok']]
    correct = [r for r in fixes if r['err_h'] <= CORRECT_M]
    confirmed = [r for r in rows if r['status'] == 'confirmed']
    out = {'note': sc.note, 'trials': n, 'attempts': len(rows), 'times': list(sc.times),
           'heights': [float(np.mean([r['height'] for r in rows if r['attempt'] == a])) for a in range(len(sc.times))]}
    out['stage1_right'] = rate(sum(1 for r in rows if r['s1_pos_err'] <= 30 and abs(r['s1_yaw_err_deg']) <= 6), len(rows))
    out['fix_per_attempt'] = rate(len(fixes), len(rows))
    out['false_per_fix'] = rate(len(fixes) - len(correct), len(fixes))
    out['false_per_attempt'] = rate(len(fixes) - len(correct), len(rows))
    out['false_per_confirmed'] = rate(sum(1 for r in confirmed if r['err_h'] > CORRECT_M), len(confirmed))
    # Time to the first fix and to the first confirmed fix, per descent.
    first_fix, first_confirmed, first_false = [], [], 0
    for j in trials:
        seq = by_trial[j]
        first_fix.append(next((r['t'] for r in seq if r['ok']), math.inf))
        first_confirmed.append(next((r['t'] for r in seq if r['status'] == 'confirmed'), math.inf))
        first = next((r for r in seq if r['status'] == 'confirmed'), None)
        first_false += first is not None and first['err_h'] > CORRECT_M
    out['fix_by'] = {str(t): rate(sum(1 for x in first_fix if x <= t), n) for t in sc.times}
    out['confirmed_by'] = {str(t): rate(sum(1 for x in first_confirmed if x <= t), n) for t in sc.times}
    out['first_confirmed_false'] = rate(first_false, sum(1 for x in first_confirmed if math.isfinite(x)))
    out['capture_max'] = stats([r['capture_max'] for r in fixes])
    out['outside_capture_per_fix'] = rate(sum(1 for r in fixes if not r['capture_max'] <= CAPTURE_M), len(fixes))
    out['outside_capture_per_confirmed'] = rate(sum(1 for r in confirmed if not r['capture_max'] <= CAPTURE_M), len(confirmed))
    out['error_over_20m'] = rate(sum(1 for r in fixes if r['err_h'] > 20.0), len(fixes))
    out['error_all_fixes'] = stats([r['err_h'] for r in fixes])
    out['error_correct_fixes'] = stats([r['err_h'] for r in correct])
    out['error_confirmed'] = stats([r['err_h'] for r in confirmed])
    out['attitude_error_deg'] = stats([r['err_att_deg'] for r in correct])
    out['height_error'] = stats([abs(r['err_z']) for r in correct])
    out['velocity_error'] = stats([r['vel_err'] for r in confirmed if r['err_h'] <= CORRECT_M])
    out['coarse_error'] = stats([r['coarse_err_h'] for r in fixes])
    ratio = [r['err_h']/r['sigma_h'] for r in fixes if r['sigma_h'] > 0]
    out['error_over_sigma'] = stats(ratio)
    out['within_2_sigma'] = rate(sum(1 for x in ratio if x <= 2.0), len(ratio))    # 86 % expected for a 2-D Gaussian
    out['match_sd'] = stats([r['match_sd'] for r in fixes])
    out['matches'] = stats([r['matches'] for r in fixes])
    far, total = sum(r['match_far'] for r in fixes), sum(r['matches'] for r in fixes)
    out['matches_beyond_4m'] = rate(int(far), int(total))
    out['level_error_deg'] = stats([r['level_err_deg'] for r in rows])
    out['tilt_deg'] = stats([r['tilt_deg'] for r in rows])
    out['sites'] = stats([r['sites'] for r in rows])
    out['ffts_256'] = stats([r['ffts_256'] for r in rows])
    out['ffts_128'] = stats([r['ffts_128'] for r in rows])
    out['host_s'] = stats([r['host_s'] for r in rows])
    reasons = {}
    for r in rows:
        reasons[r['reason']] = reasons.get(r['reason'], 0) + 1
    out['reasons'] = dict(sorted(reasons.items(), key=lambda kv: -kv[1]))
    return out


def pct(r):
    return '-' if not math.isfinite(r[0]) else '%.0f %%' % (100*r[0])


def pct_ci(r):
    return '-' if not math.isfinite(r[0]) else '%.0f %% (%.0f-%.0f)' % (100*r[0], 100*r[1], 100*r[2])


def num(s, key, spec='%.2f'):
    return '-' if not s.get('n') else spec % s[key]


def tables(summary):
    lines = ['# First-fix acquisition Monte Carlo', '',
             'SYNTHETIC. Generated by `sim/nav/run_acquisition_mc.py` on %s; see its docstring for what is modelled.' % DATE,
             'A fix is correct within %g m of truth. Intervals are Wilson 95 %%.' % CORRECT_M, '',
             'A fix is outside capture when it misplaces the ground under some pixel by more than %g m.' % CAPTURE_M, '',
             '| Scenario | What | Descents | Stage 1 right | Fix per image | False fixes, of fixes | Outside capture, of fixes | Confirmed by 5 s | Confirmed by end | First confirmed fix false | Error RMS / 95 % / max, m | Within 2 sigma |',
             '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    for name, s in summary.items():
        times = [str(t) for t in s['times']]
        five = next((t for t in reversed(times) if float(t) <= 5.0), None)
        e = s['error_all_fixes']
        lines.append('| %s | %s | %d | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
            name, s['note'], s['trials'], pct(s['stage1_right']), pct_ci(s['fix_per_attempt']), pct_ci(s['false_per_fix']),
            pct_ci(s['outside_capture_per_fix']),
            pct_ci(s['confirmed_by'][five]) if five else '-', pct_ci(s['confirmed_by'][times[-1]]),
            pct_ci(s['first_confirmed_false']),
            '-' if not e.get('n') else '%.2f / %.2f / %.2f' % (e['rms'], e['p95'], e['max']), pct(s['within_2_sigma'])))
    lines += ['', '| Scenario | Levelling error RMS, deg | Sites | Matches | Match SD, m | Matches beyond 4 m | Stage-3 error RMS, m | Capture error 95 % / max, m | Error over 20 m, of fixes | Attitude error RMS, deg | Velocity error RMS, m/s | FFTs 256 / 128 (median) | Host s (median) |',
              '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    for name, s in summary.items():
        c = s['capture_max']
        lines.append('| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s / %s | %s |' % (
            name, num(s['level_error_deg'], 'rms'), num(s['sites'], 'median', '%.0f'), num(s['matches'], 'median', '%.0f'),
            num(s['match_sd'], 'median'), pct(s['matches_beyond_4m']), num(s['coarse_error'], 'rms'),
            '-' if not c.get('n') else '%.1f / %.1f' % (c['p95'], c['max']), pct_ci(s['error_over_20m']),
            num(s['attitude_error_deg'], 'rms'), num(s['velocity_error'], 'rms'), num(s['ffts_256'], 'median', '%.0f'),
            num(s['ffts_128'], 'median', '%.0f'), num(s['host_s'], 'median')))
    lines += ['', '## Why an image gave no fix', '']
    for name, s in summary.items():
        lines.append('- **%s**: %s' % (name, ', '.join('%s %d' % kv for kv in s['reasons'].items())))
    return '\n'.join(lines) + '\n'


def plots(summary):
    import plotting
    made = []
    styles = [k for k in ('style_open', 'style_arable', 'baseline', 'style_pasture', 'style_tramline') if k in summary]
    if styles:
        labels = [k.replace('style_', '').replace('baseline', 'mixed') for k in styles]
        made.append(plotting.bar_chart(OUT/'by_terrain.png', labels, [
            {'label': 'fix from one image', 'values': [100*summary[k]['fix_per_attempt'][0] for k in styles], 'color': '#0072B2'},
            {'label': 'confirmed fix within 5 s', 'values': [100*summary[k]['confirmed_by'].get('5.0', [math.nan])[0] for k in styles], 'color': '#009E73'},
            {'label': 'false, of fixes', 'values': [100*summary[k]['false_per_fix'][0] for k in styles], 'color': '#D55E00'}],
            'First fix by terrain', 'percent', 'SYNTHETIC farmland, default seasonal change, yaw unknown', ylim=(0, 100)))
    seasons = [k for k in ('season_identical', 'season_shadows_only', 'season_fields_only', 'baseline', 'season_all_fields',
                           'season_low_sun', 'season_leaf_off', 'season_exposure') if k in summary]
    if seasons:
        labels = [k.replace('season_', '').replace('baseline', 'default').replace('_', ' ') for k in seasons]
        made.append(plotting.bar_chart(OUT/'by_season.png', labels, [
            {'label': 'fix error RMS, m', 'values': [summary[k]['error_all_fixes'].get('rms', math.nan) for k in seasons], 'color': '#0072B2'},
            {'label': 'match SD, m', 'values': [summary[k]['match_sd'].get('median', math.nan) for k in seasons], 'color': '#CC79A7'}],
            'Fix error by what changed since the map', 'metres', 'SYNTHETIC, mixed farmland'))
    timed = [k for k in ('baseline', 'style_arable', 'canopy_violent', 'compass') if k in summary]
    if timed:
        made.append(plotting.line_chart(OUT/'confirmed_by_time.png', [
            {'x': summary[k]['times'], 'y': [100*summary[k]['confirmed_by'][str(t)][0] for t in summary[k]['times']],
             'label': k.replace('style_', '').replace('_', ' '), 'style': 'line+points'} for k in timed],
            'Descents with a confirmed first fix', 'seconds after the canopy has settled', 'percent',
            'SYNTHETIC; one image per second, two agreeing fixes needed', ylim=(0, 100), vlines=[(5.0, 'T1: 5 s', '#D55E00')]))
    return [str(Path(p).name) for p in made]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=48)
    parser.add_argument('--max-trials', type=int, default=0, help='cap on descents per scenario, for a short run')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--only', default='')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--summarise', action='store_true', help='rebuild summary and tables from attempts.csv')
    args = parser.parse_args()
    if args.list:
        for name, sc in SCENARIOS.items():
            print('%-28s %s' % (name, sc.note))
        return 0
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT/'attempts.csv'
    existing = {}
    if path.exists():
        with open(path, newline='') as handle:
            for row in csv.DictReader(handle):
                for key in COLUMNS:
                    if key not in ('scenario', 'reason', 'status'):
                        row[key] = float(row[key]) if row[key] not in ('', 'nan') else math.nan
                for key in ('trial', 'world', 'attempt', 'ok', 'polished'):
                    row[key] = int(row[key])
                existing.setdefault(row['scenario'], []).append(row)
    if not args.summarise:
        names = [x for x in args.only.split(',') if x] or list(SCENARIOS)
        for name in names:
            mc.build_truth(SCENARIOS[name].canopy, 160)
        jobs = [(name, wi, trials_of(name, args.trials, args.max_trials)) for name in names for wi in range(WORLDS)]
        started = time.perf_counter()
        fresh = {name: [] for name in names}
        with Pool(args.workers) as pool:
            for i, rows in enumerate(pool.imap_unordered(run_chunk, jobs), 1):
                if rows:
                    fresh[rows[0]['scenario']].extend(rows)
                print('chunk %d/%d done, %.0f s' % (i, len(jobs), time.perf_counter() - started), flush=True)
        existing.update(fresh)
        with open(path, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=COLUMNS)
            writer.writeheader()
            for name in SCENARIOS:
                for row in sorted(existing.get(name, []), key=lambda r: (r['trial'], r['attempt'])):
                    writer.writerow(row)
    summary = {name: summarise(name, existing[name]) for name in SCENARIOS if existing.get(name)}
    report = {'label': 'SYNTHETIC first-fix acquisition Monte Carlo', 'date': DATE, 'seed': SEED, 'world_seed': WORLD_SEED,
              'worlds_per_scenario': WORLDS, 'correct_m': CORRECT_M, 'render_binning': BINNING,
              'host': platform.platform(), 'python': platform.python_version(), 'numpy': np.__version__,
              'config': {k: v for k, v in vars(CoarseConfig()).items()},
              'scenarios': {name: {'note': sc.note, 'style': sc.style, 'season': sc.season, 'canopy': sc.canopy,
                                   'times': sc.times, 'levelling': sc.levelling, 'compass_sd_deg': sc.compass_sd_deg,
                                   'map_res': sc.map_res, 'height_sd': sc.height_sd, 'dispersion_m': sc.dispersion_m,
                                   'blur_px': sc.blur_px, 'noise': sc.noise, 'config': sc.config}
                            for name, sc in SCENARIOS.items()},
              'summary': summary}
    report['figures'] = plots(summary)
    (OUT/'summary.json').write_text(json.dumps(report, indent=1, default=str))
    (OUT/'tables.md').write_text(tables(summary), encoding='utf-8')
    for name, s in summary.items():
        print('%-28s n %3d  s1 %s  fix/img %s  false %s  >20m %s  no-capture %s  conf5 %s  rms %s  2sig %s' % (
            name, s['trials'], pct(s['stage1_right']), pct(s['fix_per_attempt']), pct_ci(s['false_per_fix']),
            pct(s['error_over_20m']), pct(s['outside_capture_per_fix']),
            pct(s['confirmed_by'].get('5.0', [math.nan])), num(s['error_all_fixes'], 'rms'), pct(s['within_2_sigma'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
