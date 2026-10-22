"""GPL-3.0-only. Tables and figures from the navigation Monte Carlo (SYNTHETIC).

Run after run_nav_mc.py and fix_bounds.py. Writes results/nav_mc/tables.md,
layer_velocity.json and PNG figures beside them. Pillow only; see plotting.py.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import plotting  # noqa: E402
from fix_bounds import budget  # noqa: E402

OUT = HERE/'results'/'nav_mc'
RAW = HERE/'generated'

GROUPS = [
    ('Verification: the filter model equals the simulation',
     ['matched_benign', 'matched_moderate', 'matched_severe', 'matched_violent', 'matched_time_state',
      'matched_full_filter', 'matched_single_benign', 'matched_single_moderate', 'matched_single_severe',
      'matched_single_violent', 'matched_single_tight_start']),
    ('Fragility of the plain filter: one unmodelled error at a time',
     ['matched_moderate', 'plain_accel_scale', 'plain_calibration', 'plain_map_xy', 'plain_relief',
      'plain_gyro_cross_axis', 'plain_gyro_scale', 'plain_outliers_5pct', 'plain_delay_1ms', 'plain_all_errors',
      'proposal_all_errors']),
    ('Full filter, baseline errors, by canopy class',
     ['nominal_benign', 'nominal_moderate', 'nominal_severe', 'nominal_violent', 'proposal_all_errors_violent']),
    ('What each addition buys (moderate canopy)',
     ['nominal_moderate', 'without_iteration', 'without_post_fit_rejection', 'without_recovery',
      'without_time_state', 'without_scale_noise', 'without_map_inflation', 'map_inflation_3', 'map_inflation_30',
      'proposal_all_errors']),
    ('The same removals under load',
     ['poor_first_fix', 'poor_first_fix_without_iteration', 'poor_first_fix_proposal', 'outliers_40pct',
      'outliers_40pct_without_rejection', 'poor_first_fix_outliers_20pct',
      'poor_first_fix_outliers_20pct_without_rejection', 'gap_6s', 'gap_6s_without_recovery',
      'gyro_uncalibrated_untold', 'uncalibrated_untold_without_recovery']),
    ('Camera-to-IMU timing',
     ['delay_0ms', 'nominal_moderate', 'delay_2ms', 'delay_5ms', 'delay_10ms_prior_2ms', 'delay_10ms_prior_10ms',
      'delay_20ms_prior_20ms', 'delay_10ms_no_time_state', 'jitter_0_to_2ms', 'jitter_0_to_10ms',
      'jitter_0_to_10ms_severe']),
    ('Terrain relief', ['relief_0p3m', 'nominal_moderate', 'relief_2m_assumed_1m', 'relief_2m_assumed_2m',
                        'relief_5m_assumed_1m', 'relief_5m_assumed_5m']),
    ('Landmark geometry', ['nominal_moderate', 'fov_half', 'fov_third', 'landmarks_8', 'landmarks_4',
                           'sparse_map_80m', 'sparse_map_120m']),
    ('Gyro and IMU handling', ['nominal_moderate', 'gyro_icm42688', 'gyro_5x_noise', 'gyro_uncalibrated',
                               'gyro_uncalibrated_severe', 'gyro_uncalibrated_untold', 'imu_boxcar_100hz',
                               'imu_decimated_100hz', 'imu_decimated_severe']),
    ('Bad matches', ['nominal_moderate', 'outliers_20pct', 'outliers_40pct', 'outliers_60pct', 'alias_5_of_30',
                     'alias_12_of_30', 'alias_18_of_30']),
    ('Frame rate and gaps', ['nominal_moderate', 'rate_2p5hz', 'rate_1hz', 'gap_3s', 'gap_3s_severe', 'gap_6s']),
    ('Map datum, calibration and first fix', ['nominal_moderate', 'datum_2p5m', 'calibration_poor', 'poor_first_fix']),
    ('The violent canopy (19 deg RMS swing) under the stresses that interact with swing',
     ['nominal_violent', 'violent_without_scale_noise', 'violent_without_time_state', 'violent_delay_10ms',
      'violent_delay_10ms_no_time_state', 'violent_jitter_0_to_2ms', 'violent_jitter_0_to_10ms',
      'violent_gyro_uncalibrated', 'violent_imu_boxcar_100hz', 'violent_imu_decimated_100hz', 'violent_fov_half',
      'violent_landmarks_8', 'violent_rate_1hz', 'violent_gap_3s', 'violent_relief_2m']),
    ('Cheaper processing: fewer landmarks at a lower image rate',
     ['nominal_moderate', 'landmarks_12', 'landmarks_8', 'landmarks_4', 'rate_2p5hz', 'rate_1hz',
      'landmarks_12_at_2p5hz', 'landmarks_8_at_2p5hz', 'landmarks_8_at_1hz', 'landmarks_4_at_1hz',
      'violent_landmarks_8_at_1hz']),
    ('End to end: started by the first-fix solver with unknown yaw',
     ['first_fix_start', 'first_fix_start_violent', 'first_fix_start_50pct_wrong', 'first_fix_start_8_matches',
      'first_fix_start_proposal_filter']),
]
FIX_COLUMNS = [
    ('first_fix_fraction_on_first_image', 'fix on first image', '%.3f'),
    ('first_fix_fraction_within_5s', 'fix within 5 s', '%.3f'), ('first_fix_never', 'never', '%.3f'),
    ('first_fix_refusals_per_descent', 'refusals per descent', '%.2f'),
    ('first_fix_error_rms_m', 'fix error RMS m', '%.2f'), ('first_fix_error_max_m', 'fix error worst m', '%.2f'),
    ('first_fix_sigma_m', 'fix 1σ m', '%.2f'), ('T1_fraction_within_10m', '≤10 m from the fix on', '%.3f'),
    ('horizontal_rms_m', 'tracking RMS m', '%.2f'), ('horizontal_max_m', 'tracking worst m', '%.1f'),
]
COLUMNS = [
    ('horizontal_rms_m', 'RMS m', '%.2f'), ('horizontal_p95_m', '95 % m', '%.2f'),
    ('horizontal_max_m', 'worst m', '%.1f'), ('filter_horizontal_sigma_m', 'filter 1σ m', '%.2f'),
    ('T1_fraction_within_10m_after_2s', '≤10 m', '%.3f'), ('tilt_rms_deg', 'tilt °', '%.3f'),
    ('yaw_rms_deg', 'yaw °', '%.3f'), ('velocity_rms_mps', 'vel m/s', '%.3f'),
    ('nees_pos2_mean', 'NEES pos (2)', '%.1f'), ('nees16_mean', 'NEES all (16)', '%.0f'),
    ('protection_level_exceedance', 'PL exceeded', '%.3f'),
    ('good_landmarks_rejected_fraction', 'good rejected', '%.3f'),
    ('bad_landmarks_accepted_fraction', 'bad accepted', '%.3f'),
    ('descents_with_a_recovery', 'recovered', '%.2f'), ('time_offset_error_rms_ms', 'Δt err ms', '%.2f'),
    ('prediction_residual_p99_m', 'search 99 % m', '%.2f'),
]
NOTE = ('Horizontal error is against simulated truth, over frames above the map floor and at least 2 s after the '
        'first fix; 160 descents per scenario. "≤10 m" is the fraction of descents whose worst such error met T1. '
        'NEES expectations are 2 and 16 (the 16 excludes the time-offset state, and is inflated by errors the '
        'filter absorbs into its bias states, so position NEES is the meaningful consistency figure). '
        '"recovered" is the fraction of descents in which the filter widened its prior at least once. '
        '"Δt err" is the camera time-offset estimate error. "search 99 %" is the 99th percentile over frames of '
        'the largest good-landmark prediction residual on the ground: what the matcher\'s search window must cover.')


def fmt(value, pattern):
    return 'n/a' if value is None or (isinstance(value, float) and not math.isfinite(value)) else pattern % value


def tables(scenarios, layers):
    lines = ['# Navigation Monte Carlo tables (SYNTHETIC)', '', NOTE, '']
    for title, names in GROUPS:
        names = [n for n in names if n in scenarios]
        if not names:
            continue
        lines += ['## ' + title, '', '| Scenario | ' + ' | '.join(c[1] for c in COLUMNS) + ' |',
                  '|---|' + '---:|'*len(COLUMNS)]
        for name in names:
            s = scenarios[name]
            lines.append('| %s | ' % name + ' | '.join(fmt(s.get(key), pattern) for key, _, pattern in COLUMNS) + ' |')
        lines.append('')
        if title.startswith('End to end'):
            lines += ['| Scenario | ' + ' | '.join(c[1] for c in FIX_COLUMNS) + ' |', '|---|' + '---:|'*len(FIX_COLUMNS)]
            for name in names:
                lines.append('| %s | ' % name + ' | '.join(
                    fmt(scenarios[name].get(key), pattern) for key, _, pattern in FIX_COLUMNS) + ' |')
            lines.append('')
    if layers:
        lines += ['## Horizontal velocity error as the wind retrieval would see it', '',
                  'Per axis, 300 m to 80 m. "20 m layer" is the RMS over descents and layers of the layer-mean '
                  'error. "ρ₁" is the lag-1 autocorrelation at 5 Hz and "τ" the 1/e time of the error '
                  'autocorrelation.', '',
                  '| Scenario | sample RMS m/s | 20 m layer RMS m/s | ρ₁ | τ s |', '|---|---:|---:|---:|---:|']
        for name, row in layers.items():
            lines.append('| %s | %.3f | %.3f | %.2f | %.2f |' % (
                name, row['sample_rms_mps'], row['layer20m_rms_mps'], row['lag1'], row['correlation_time_s']))
        lines.append('')
    return '\n'.join(lines)


def layer_velocity(names):
    """Velocity-error statistics the wind retrieval cares about, from the retained raw records."""
    out = {}
    edges = np.arange(300.0, 79.0, -20.0)
    for name in names:
        path = RAW/f'raw_{name}.npz'
        if not path.exists():
            continue
        with np.load(path) as raw:
            err, height = raw['err'][:, :, 3:5].astype(float), raw['height'].astype(float)
        ok = np.isfinite(height) & (height >= 80.0) & (np.arange(height.shape[1])[None, :] >= 10)
        means, acf = [], []
        for j in range(err.shape[0]):
            for hi, lo in zip(edges[:-1], edges[1:]):
                mask = ok[j] & (height[j] <= hi) & (height[j] > lo)
                if mask.sum() >= 5:
                    means.append(err[j, mask].mean(axis=0))
            x = err[j, ok[j]]
            if len(x) > 40:
                x = x - x.mean(axis=0)
                acf.append([np.mean(np.sum(x[k:]*x[:len(x) - k], axis=0)/np.sum(x*x, axis=0)) for k in range(30)])
        acf = np.mean(acf, axis=0)
        below = np.flatnonzero(acf < math.exp(-1))
        out[name] = {'sample_rms_mps': float(np.sqrt(np.mean(err[ok]**2))),
                     'layer20m_rms_mps': float(np.sqrt(np.mean(np.square(means)))),
                     'lag1': float(acf[1]),
                     'correlation_time_s': float(below[0]*0.2) if len(below) else float('nan'),
                     'autocorrelation_5hz': [float(a) for a in acf]}
    return out


def curve(curves, name, key):
    return {'x': np.array(curves[name]['height']), 'y': np.array(curves[name][key])}


def figures(scenarios, curves):
    made = []
    sub = 'SYNTHETIC | 160 descents per scenario | 5 Hz landmarks above 80 m, inertial and barometer only below'
    names = [n for n in ('nominal_benign', 'nominal_moderate', 'nominal_severe', 'nominal_violent') if n in curves]
    floor = [(80.0, 'map floor', '#777777')]
    if names:
        series = []
        for name in names:
            label = name.split('_')[1]
            series.append({**curve(curves, name, 'horiz_rms'), 'label': label + ': RMS error'})
            series.append({**curve(curves, name, 'horiz_sigma'), 'label': label + ': filter 1σ', 'style': 'dash'})
        if 'proposal_all_errors' in curves:
            series.append({**curve(curves, 'proposal_all_errors', 'horiz_rms'), 'label': 'proposal filter: RMS error',
                           'color': '#7b3294', 'width': 3})
        made.append(plotting.line_chart(
            OUT/'error_vs_height.png', series, 'Horizontal position error against height',
            'Height above ground (m)', 'Horizontal error (m)', subtitle=sub, invert_x=True, logy=True,
            hlines=[(10.0, 'T1 requirement 10 m', '#b03030')], vlines=floor))
        heights = curves['nominal_moderate']['height']
        above = [h for h in heights if h >= 80]
        made.append(plotting.line_chart(
            OUT/'error_vs_budget.png',
            [{**curve(curves, 'nominal_moderate', 'horiz_rms'), 'label': 'simulated RMS, relative to the map'},
             {**curve(curves, 'nominal_moderate', 'horiz_p95'), 'label': 'simulated 95th percentile', 'style': 'dash'},
             {'x': np.array(above), 'y': np.array([budget(h, 0.3)['total'] for h in above]),
              'label': 'section 12 budget, σθ = 0.3°'},
             {'x': np.array(above), 'y': np.array([budget(h, 1.0)['total'] for h in above]),
              'label': 'section 12 budget, σθ = 1°'},
             {'x': np.array(above), 'y': np.array([budget(h, 0.3)['attitude'] for h in above]),
              'label': 'budget attitude term alone, 0.3°', 'style': 'dash'}],
            'Simulated error against the proposal error budget', 'Height above ground (m)', 'Horizontal error (m)',
            subtitle='SYNTHETIC | the budget total includes 2.5 m georeferencing, which the simulation reports separately',
            invert_x=True, logy=True, xlim=(80, 300)))
        made.append(plotting.line_chart(
            OUT/'attitude_vs_height.png',
            [s for name in names for s in (
                {**curve(curves, name, 'tilt_rms_deg'), 'label': name.split('_')[1] + ': tilt error'},
                {**curve(curves, name, 'swing_rms_deg'), 'label': name.split('_')[1] + ': true swing', 'style': 'dash'})],
            'Tilt estimate error against the swing it has to track', 'Height above ground (m)', 'Angle (deg, RMS)',
            subtitle=sub, invert_x=True, logy=True, vlines=floor))
        made.append(plotting.line_chart(
            OUT/'velocity_vs_height.png',
            [s for name in names for s in (
                {**curve(curves, name, 'vel_rms'), 'label': name.split('_')[1] + ': RMS error'},
                {**curve(curves, name, 'vel_sigma'), 'label': name.split('_')[1] + ': filter 1σ', 'style': 'dash'})],
            'Horizontal velocity error against height', 'Height above ground (m)', 'Velocity error (m/s)',
            subtitle=sub, invert_x=True, logy=True,
            hlines=[(0.6, 'earlier wind study assumed 0.6 m/s per axis', '#777777')], vlines=floor))

    def bars(path, names, title, keys=('horizontal_rms_m', 'horizontal_p95_m', 'horizontal_max_m', 'filter_horizontal_sigma_m'),
             labels=('RMS error', '95th percentile', 'worst case', 'filter 1σ'), ylabel='Horizontal error (m)',
             hlines=((10.0, 'T1 10 m', '#b03030'),)):
        names = [n for n in names if n in scenarios]
        if len(names) < 2:
            return
        groups = [{'label': label, 'values': [float('nan') if scenarios[n].get(key) is None else scenarios[n][key]
                                              for n in names]} for key, label in zip(keys, labels)]
        low = min(v for g in groups for v in g['values'] if v == v and v > 0)
        made.append(plotting.bar_chart(OUT/path, [n.replace('_', ' ') for n in names], groups, title, ylabel,
                                       subtitle='SYNTHETIC | above 80 m, from 2 s after the first fix',
                                       logy=True, hlines=list(hlines), size=(1300, 640),
                                       ylim=(10**math.floor(math.log10(low*0.7)), None)))

    bars('fragility.png', GROUPS[1][1], 'The plain filter with one unmodelled error at a time')
    bars('additions.png', GROUPS[3][1], 'The full filter with one addition removed')
    bars('under_load.png', GROUPS[4][1], 'The same removals under load')
    bars('timing.png', GROUPS[5][1], 'Camera-to-IMU timing error')
    bars('relief.png', GROUPS[6][1], 'Unmodelled landmark height')
    bars('geometry.png', GROUPS[7][1], 'Field of view and landmark count')
    bars('imu.png', GROUPS[8][1], 'Gyro quality and IMU handling')
    bars('matches.png', GROUPS[9][1] + GROUPS[10][1][1:], 'Bad matches, frame rate and gaps')
    bars('violent.png', GROUPS[12][1], 'The violent canopy (19° RMS swing) under stress')
    bars('cheaper.png', GROUPS[13][1], 'Fewer landmarks at a lower image rate')
    bars('first_fix.png', GROUPS[14][1], 'Started by the first-fix solver with unknown yaw')
    consistency = ['matched_moderate', 'matched_single_moderate', 'nominal_benign', 'nominal_moderate',
                   'nominal_severe', 'nominal_violent', 'without_map_inflation', 'without_scale_noise',
                   'proposal_all_errors']
    bars('consistency_nees.png', consistency, 'Is the filter honest? Horizontal position NEES',
         keys=('nees_pos2_mean',), labels=('mean NEES',), ylabel='Mean NEES (2 if consistent)',
         hlines=((2.0, 'consistent', '#777777'),))
    bars('consistency_protection.png', consistency, 'Epochs where the error exceeded the 95 % protection level',
         keys=('protection_level_exceedance',), labels=('fraction of epochs',), ylabel='Fraction of epochs',
         hlines=((0.05, '5 % allowed', '#777777'),))
    return made


def main():
    report = json.loads((OUT/'summary.json').read_text())
    curves = json.loads((OUT/'curves.json').read_text())
    layers = layer_velocity(['matched_moderate', 'nominal_benign', 'nominal_moderate', 'nominal_severe',
                             'nominal_violent', 'relief_0p3m', 'gyro_icm42688', 'rate_1hz', 'landmarks_8_at_1hz',
                             'first_fix_start', 'proposal_all_errors'])
    (OUT/'layer_velocity.json').write_text(json.dumps(layers, indent=1))
    (OUT/'tables.md').write_text(tables(report['scenarios'], layers), encoding='utf-8')
    made = figures(report['scenarios'], curves)
    print('wrote tables.md, layer_velocity.json and', len(made), 'figures')


if __name__ == '__main__':
    main()
