"""GPL-3.0-only. Check retained study evidence independently of the runner."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def check(directory):
    directory = Path(directory)
    manifest = json.loads((directory/'manifest.json').read_text())
    summary = json.loads((directory/'summary.json').read_text())
    for name, expected in manifest['files'].items():
        actual = hashlib.sha256((directory/name).read_bytes()).hexdigest()
        if actual != expected:
            raise AssertionError('changed result: '+name)
    source = Path(__file__).resolve().parent
    for name, expected in manifest['code'].items():
        if hashlib.sha256((source/name).read_bytes()).hexdigest() != expected:
            raise AssertionError('source no longer matches run: '+name)
    assert 'no measurements' in summary['label']
    raw = np.load(directory/'readout_noise_equivalent_draws.npz')
    maximum_relative_difference = 0.
    for row in summary['readout']:
        a = raw[f"noise_equivalent_error_{row['temperature_c']:g}C"]
        assert a.shape == (20000,) and np.isfinite(a).all()
        sd = row['adc_noise_equivalent_sigma_k']
        # 20k draws: sigma sampling SE ~0.5%; 3% allows model nonlinearity.
        difference = abs(float(np.std(a, ddof=1))/sd-1)
        maximum_relative_difference = max(maximum_relative_difference, difference)
        assert difference < .03, (row['temperature_c'], difference)
        assert abs(a.mean()) < 5*sd/np.sqrt(len(a))
    profile = np.load(directory/'synthetic_profile.npz')
    np.testing.assert_allclose(np.diff(profile['height_m'])/np.diff(profile['t_s']), -10.5, rtol=1e-10)
    assert np.isfinite(profile['air_c']).all()
    for row in summary['profiles']:
        np.testing.assert_allclose(row['smoothing_length_m'], 10.5*row['tau_s'])
        np.testing.assert_allclose(row['t90_s'], np.log(10)*row['tau_s'])
        a = profile[f"tau_{row['tau_s']:g}s_c"]-profile['air_c']
        np.testing.assert_allclose(np.sqrt(np.mean(a*a)), row['profile_rms_k'])
    with (directory/'fit_trials.csv').open(newline='') as f:
        fits = list(csv.DictReader(f))
    for group in summary['fit_mc']:
        selected = [r for r in fits if float(r['sample_hz']) == group['sample_hz'] and
                    float(r['true_tau_s']) == group['true_tau_s']]
        assert len(selected) == group['trials']
        values = np.array([float(r['fitted_tau_s']) for r in selected])
        np.testing.assert_allclose(np.median(values), group['fitted_tau_median_s'])
        np.testing.assert_allclose(np.quantile(values, [.025, .975]),
                                   [group['fitted_tau_p025_s'], group['fitted_tau_p975_s']])
        assert not any(r['bound_hit'] == 'True' for r in selected)
    # Independent unit arithmetic, separate from pair_schedule.
    for r in summary['schedules']:
        expected = .002+2*(25e-6+1/(.9*r['data_rate_sps']))
        np.testing.assert_allclose(r['on_s'], expected)
        assert r['feasible'] == (expected*r['sample_hz'] <= 1)
    return dict(result='PASS', label='synthetic output integrity and numerical checks',
                hashed_artifacts=len(manifest['files']), source_files_checked=len(manifest['code']),
                readout_draws=20000*len(summary['readout']), fitted_step_trials=len(fits),
                maximum_adc_sigma_relative_difference=maximum_relative_difference)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    result = check(args.directory)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
