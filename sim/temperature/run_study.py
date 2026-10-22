"""GPL-3.0-only. Reproducible analogue probe calculations and SYNTHETIC trials.

Usage: python sim/temperature/run_study.py [--out DIR] [--trials 128]
New outputs only; refuses an existing summary to preserve historical evidence.
No measurements or flight software are modified.
"""
import argparse
import csv
import hashlib
import json
import math
import platform
import sys
from pathlib import Path

import numpy as np

import models as m
from step_fit import fit_step

HERE = Path(__file__).resolve().parent
SEED = 20261009


def save_csv(path, rows):
    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def readout_study(out, rng):
    rows, raw = [], {}
    # TI ADS1115 table 6-1: RMS includes quantization floor. Do NOT add a
    # second quantization variance to these noise-equivalent draws.
    sd_volts = np.array([62.5e-6, 125e-6])
    for t, r in zip(m.TE_T_C, m.TE_R_OHM):
        v, _, p = map(float, m.divider(r))
        nominal_t = float(m.sh_temperature(r, m.SH_COEFF))
        draw = rng.normal(size=(20000, 2))*sd_volts + [v, 3.]
        inferred = m.sh_temperature(m.resistance_from_volts(draw[:, 0], draw[:, 1]), m.SH_COEFF)
        err = inferred-nominal_t
        raw[f'noise_equivalent_error_{t:g}C'] = err
        jac = m.readout_jacobian(v, 3.)
        row = dict(temperature_c=float(t), resistance_ohm=float(r), divider_v=v,
                   power_on_uw=p*1e6, continuous_still_air_rise_k=p/m.TE_AIR_G_W_K,
                   beta_bias_k=float(m.beta_temperature(r)-t), nominal_sh_bias_k=nominal_t-t,
                   adc_noise_equivalent_sigma_k=float(np.linalg.norm(jac*sd_volts)),
                   synthetic_adc_rms_k=float(np.sqrt(np.mean(err**2))),
                   synthetic_adc_mean_k=float(np.mean(err)))
        # One contribution at a time; these are bounds/scenarios, not Gaussian
        # sigmas. Calibrated range/channel errors need actual board evidence.
        for name, rr, a, b in (
                ('reference_plus_0p1pct_k', 150150., v, 3.),
                ('ntc_channel_plus_3lsb_k', m.R_REF, v+3*62.5e-6, 3.),
                ('exc_channel_plus_3lsb_k', m.R_REF, v, 3.+3*125e-6),
                ('range_mismatch_plus_0p1pct_k', m.R_REF, v*1.001, 3.),
                ('sequential_exc_change_plus_0p1pct_k', m.R_REF, v, 3.*1.001)):
            row[name] = float(m.sh_temperature(m.resistance_from_volts(a, b, rr), m.SH_COEFF)-nominal_t)
        rows.append(row)
    save_csv(out/'readout.csv', rows)
    np.savez_compressed(out/'readout_noise_equivalent_draws.npz', **raw)
    return rows


def thermal_study(out):
    schedule = m.pair_schedule(128, 10)
    pulse_rows = []
    for t, r in zip(m.TE_T_C, m.TE_R_OHM):
        p = float(m.divider(r)[2])
        for conductance in (0.1e-3, 0.3e-3, 1e-3):
            for tau in (.1, .3, 1., 3.):
                pulse = m.periodic_heating(p, conductance, tau, .1, schedule['on_s'])
                pulse_rows.append(dict(temperature_c=float(t), air_g_w_k=conductance, tau_s=tau,
                                       on_s=schedule['on_s'], **pulse))
    save_csv(out/'pulsed_heating.csv', pulse_rows)
    time_s = np.arange(0, 30.0001, .005)
    # ASSUMED 10.5 m/s, 0.01 K/m ramp plus smooth 1 K inversion across ~13 m.
    air = 15+.105*time_s+.5*np.tanh((time_s-15)/.6)
    traces = dict(t_s=time_s, height_m=315-10.5*time_s, air_c=air)
    rows = []
    for tau in (.2, .5, 1., 3.):
        trace = m.thermal_trace(time_s, air, air+20, 0, m.Thermal(tau_s=tau))
        traces[f'tau_{tau:g}s_c'] = trace
        rows.append(dict(tau_s=tau, t90_s=math.log(10)*tau, smoothing_length_m=10.5*tau,
                         steady_ramp_lag_k=.105*tau,
                         profile_rms_k=float(np.sqrt(np.mean((trace-air)**2))),
                         profile_peak_abs_k=float(abs(trace-air).max())))
    # Separate lead/radiation bias from lag; no inferred lead geometry.
    lead_rows = []
    for ratio in (0., .001, .005, .01, .05):
        bias = 20*ratio/(1+ratio)
        lead_rows.append(dict(lead_to_air_g_ratio=ratio, body_excess_k=20., equilibrium_bias_k=bias))
    save_csv(out/'profile_metrics.csv', rows)
    save_csv(out/'lead_coupling.csv', lead_rows)
    save_csv(out/'synthetic_profile.csv', [dict(zip(traces, values)) for values in zip(*traces.values())])
    np.savez_compressed(out/'synthetic_profile.npz', **traces)
    return pulse_rows, rows, lead_rows, traces


def fit_study(out, rng, trials):
    rows, raw, summaries = [], {}, []
    # Closed-form continuous step provides an independent generator. step
    # lies between samples; fitter receives its independently known timestamp.
    for hz in (1., 10.):
        time_s = np.arange(0, 16+0.5/hz, 1/hz)
        step = 3.137
        reference = np.where(time_s < step, 18., 23.)
        for tau in (.3, 1.):
            elapsed = np.maximum(time_s-step, 0)
            ideal = 18+5*(-np.expm1(-elapsed/tau))
            y = ideal[None, :]+rng.normal(0, .02, (trials, len(time_s)))
            estimates = []
            for i in range(trials):
                result = fit_step(time_s, reference, y[i], step)
                estimates.append(result['tau_s'])
                rows.append(dict(sample_hz=hz, true_tau_s=tau, trial=i,
                                 fitted_tau_s=result['tau_s'], residual_rms_c=result['residual_rms_c'],
                                 residual_lag1=result['residual_lag1'],
                                 informative=result['validity_flags']['tau_profile_informative'],
                                 bound_hit=result['tau_hit_lower_bound'] or result['tau_hit_upper_bound']))
            estimates = np.array(estimates)
            key = f'{hz:g}Hz_tau{tau:g}'
            raw[key+'_probe_c'] = y
            raw[key+'_t_s'] = time_s
            raw[key+'_reference_c'] = reference
            summaries.append(dict(sample_hz=hz, true_tau_s=tau, trials=trials,
                                  noise_sd_k=.02, fitted_tau_median_s=float(np.median(estimates)),
                                  fitted_tau_p025_s=float(np.quantile(estimates, .025)),
                                  fitted_tau_p975_s=float(np.quantile(estimates, .975)),
                                  fitted_tau_rms_error_s=float(np.sqrt(np.mean((estimates-tau)**2)))))
            print('fit MC', key, 'median %.4f s' % np.median(estimates), flush=True)
    save_csv(out/'fit_trials.csv', rows)
    np.savez_compressed(out/'synthetic_step_trials.npz', **raw)
    # A reusable input example, explicitly synthetic, for the actual CSV CLI.
    time_s = raw['10Hz_tau1_t_s']
    save_csv(out/'SYNTHETIC_step_example.csv', [dict(t_s=float(t), reference_c=float(r), probe_c=float(y))
              for t, r, y in zip(time_s, raw['10Hz_tau1_reference_c'], raw['10Hz_tau1_probe_c'][0])])
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=HERE/'results'/'2026-10-09')
    parser.add_argument('--trials', type=int, default=128)
    args = parser.parse_args()
    if args.trials < 2:
        parser.error('need at least two Monte Carlo trials')
    if args.out.exists() and any(args.out.iterdir()):
        parser.error('output directory must be empty or new; preserve previous evidence')
    args.out.mkdir(parents=True, exist_ok=True)
    source_files = (HERE/'models.py', HERE/'run_study.py', HERE/'step_fit.py')
    source_hashes = {x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in source_files}
    rng = np.random.default_rng(SEED)
    readout = readout_study(args.out, rng)
    pulses, profiles, leads, trace = thermal_study(args.out)
    fits = fit_study(args.out, rng, args.trials)
    schedule = [dict(data_rate_sps=sps, sample_hz=hz, **m.pair_schedule(sps, hz))
                for sps in (8, 128, 860) for hz in (1., 10.)]
    save_csv(args.out/'schedules.csv', schedule)
    train = {0, 5, 8}
    held = [row for i, row in enumerate(readout) if i not in train]
    c0, c1 = float(m.adc_code(.1875, 2.048)), float(m.adc_code(3, 4.096))
    bad_t = float(m.sh_temperature(m.resistance_from_volts(c0, c1), m.SH_COEFF))
    summary = dict(label='SYNTHETIC and CALCULATED; no measurements', seed=SEED,
                   python=platform.python_version(), numpy=np.__version__,
                   source_urls=dict(te=m.TE_SOURCE, ads1115='https://www.ti.com/lit/ds/symlink/ads1115.pdf',
                                    opa333='https://www.ti.com/lit/ds/symlink/opa333.pdf'),
                   assumptions=dict(thermal_tau_s=[.1, .2, .3, .5, 1., 3.],
                                    lead_coupling='sensitivity ratios only; not fitted',
                                    step_noise_sd_k=.02, step_amplitude_k=5.,
                                    thermal_model='one node, imposed air/body, nominal R interpolation',
                                    fit_intervals='conditional Monte Carlo spread, not real-data confidence intervals',
                                    adc_noise='TI table RMS as Gaussian noise-equivalent scenario; no added quantization variance'),
                   sh_coefficients=m.SH_COEFF.tolist(),
                   sh_fit_temperature_c=m.TE_T_C[[0, 5, 8]].tolist(),
                   heldout_max_sh_bias_k=max(abs(x['nominal_sh_bias_k']) for x in held),
                   table_max_beta_bias_k=max(abs(x['beta_bias_k']) for x in readout),
                   raw_mixed_pga_error_at25_k=bad_t-25,
                   readout=readout, schedules=schedule, profiles=profiles,
                   lead_coupling=leads, fit_mc=fits,
                   worst_assumed_pulse_peak_k=max(x['peak_k'] for x in pulses))
    (args.out/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    # Reuse existing Pillow chart helper read-only; no navigation files/results
    # are changed and no plotting dependency is introduced.
    sys.path.insert(0, str(HERE.parent/'nav'))
    import plotting
    plotting.line_chart(args.out/'temperature_profile.png',
        [{'x':trace['height_m'], 'y':trace[k], 'label':k.replace('_c', '')} for k in trace if k.endswith('_c')],
        'Air-temperature lag during descent', 'Height (m)', 'Temperature (C)',
        subtitle='SYNTHETIC | assumed 10.5 m/s, no excitation or body heat in these traces', invert_x=True)
    plotting.line_chart(args.out/'conversion_bias.png',
        [{'x':m.TE_T_C, 'y':np.array([x[k] for x in readout]), 'label':label}
         for k, label in (('beta_bias_k','single beta 3976 K'),('nominal_sh_bias_k','three-point nominal fit'))],
        'Conversion error against TE resistance table', 'Source temperature (C)', 'Inferred minus source (K)',
        subtitle='CALCULATED | fitted -20,25,50 C; other points held out; no individual probe calibration')
    files = sorted(x for x in args.out.iterdir() if x.is_file())
    manifest = dict(label='reproducibility manifest', seed=SEED,
        files={x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in files},
        code=source_hashes)
    if source_hashes != {x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in source_files}:
        raise RuntimeError('source changed during run; retain exploratory outputs, rerun to a fresh directory')
    (args.out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    print('wrote', args.out, 'SH held-out max %.5f K' % summary['heldout_max_sh_bias_k'], flush=True)


if __name__ == '__main__':
    main()
