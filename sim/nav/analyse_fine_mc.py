"""GPL-3.0-only. Audit retained fine-tracking CSV/JSON and render host-study figures.

Run: python sim/nav/analyse_fine_mc.py [--input CAMPAIGN] [--output FIGURES]
No simulation is rerun. Bands show the full range across trials, not confidence
intervals. The default reads only the corrected-camera campaign.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from plotting import line_chart, panel

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
STUDY = HERE/'results'/'fine_tracking_20261023'


def load_and_audit(path):
    path = Path(path)
    summary = json.loads((path/'summary.json').read_text(encoding='utf-8'))
    with (path/'frames.csv').open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    camera = summary['camera']
    for field, expected in (('fx_px', 724.6/4), ('fy_px', 724.6/4),
                            ('cx_px', (728 - 1.5)/4), ('cy_px', (544 - 1.5)/4)):
        if not math.isclose(camera[field], expected, abs_tol=1e-12):
            raise ValueError('Invalid study camera ' + field + '; do not use diagnostic pilot outputs')
    expected = {(s, t, f) for s in summary['scenarios'] for t in range(summary['trials'])
                for f in range(summary['frames_per_trial'])}
    actual = {(r['scenario'], int(r['trial']), int(r['frame'])) for r in rows}
    if actual != expected or len(rows) != len(expected):
        raise ValueError('Campaign is incomplete or contains duplicate frames')
    checks = {}
    for scenario in summary['scenarios']:
        subset = [r for r in rows if r['scenario'] == scenario]
        reported = summary['results'][scenario]
        counts = {'accepted_matches': sum(int(r['matches_accepted']) for r in subset),
                  'attempted_sites': sum(int(r['matches_attempted']) for r in subset),
                  'filter_matches_accepted': sum(int(r['filter_matches_accepted']) for r in subset)}
        for key, value in counts.items():
            if reported[key] != value:
                raise ValueError(scenario + ': JSON/CSV disagree on ' + key)
        rms = float(np.sqrt(np.mean([float(r['filter_horizontal_error_after_m'])**2 for r in subset])))
        if not math.isclose(reported['horizontal_filter_rms_after_m'], rms, rel_tol=1e-12, abs_tol=1e-12):
            raise ValueError(scenario + ': JSON/CSV horizontal RMS mismatch')
        false = sum(int(r['ground_false_matches_over_0_75m']) for r in subset)
        if reported['ground_error_accepted_only']['false_matches_over_threshold'] != false:
            raise ValueError(scenario + ': JSON/CSV false-match count mismatch')
        checks[scenario] = {**counts, 'horizontal_rms_recomputed_m': rms, 'false_matches_recomputed': false}
    return summary, rows, checks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=STUDY/'full_corrected')
    parser.add_argument('--output', type=Path, default=STUDY/'figures')
    args = parser.parse_args(argv)
    args.input, args.output = args.input.resolve(), args.output.resolve()
    summary, rows, checks = load_and_audit(args.input)
    args.output.mkdir(parents=True, exist_ok=True)
    images = []
    names = {'fresh': 'Unchanged map appearance', 'seasonal': 'Changed seasonal appearance',
             'near_capture': 'Initial error near capture limit', 'outside_capture': 'Initial error outside capture limit'}
    for scenario, title in names.items():
        frames = range(summary['frames_per_trial'])
        series = []
        for field, label, color in (('filter_horizontal_error_after_m', 'Image + IMU + barometer', '#0072B2'),
                                     ('no_image_control_horizontal_error_m', 'IMU + barometer control', '#D55E00')):
            values = np.array([[float(r[field]) for r in rows if r['scenario'] == scenario and int(r['frame']) == f]
                               for f in frames])
            series.append({'x': np.array(list(frames))*summary['frame_interval_s'],
                           'y': np.median(values, axis=1), 'label': label, 'color': color,
                           'band': (values.min(axis=1), values.max(axis=1))})
        target = args.output/(scenario + '_tracking.png')
        line_chart(target, series, title, 'Elapsed time (s)', 'Horizontal position error (m)',
                   subtitle=f"SYNTHETIC: {summary['trials']} paired descents; median and full trial range",
                   hlines=[(10., '10 m reference', '#666666')], size=(1100, 620), legend='bottom')
        images.append(target)
    panel(args.output/'tracking_overview.png', images, columns=2)
    sources = ['code/cansat/fine.py', 'sim/nav/run_fine_mc.py', 'sim/nav/analyse_fine_mc.py',
               str((args.input/'summary.json').relative_to(ROOT)), str((args.input/'frames.csv').relative_to(ROOT))]
    audit = {'input': str(args.input.relative_to(ROOT)), 'rows_verified': len(rows),
             'checks': checks, 'bands': 'minimum to maximum across paired trials; not confidence intervals',
             'sha256': {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}}
    (args.output/'audit.json').write_text(json.dumps(audit, indent=2) + '\n', encoding='utf-8')
    print(f"Verified {len(rows)} retained frames and wrote {args.output}")


if __name__ == '__main__':
    main()
