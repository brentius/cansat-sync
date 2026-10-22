"""GPL-3.0-only. Tests of the SYNTHETIC first-fix Monte Carlo's bookkeeping (sim/nav/run_acquisition_mc.py).

The scoring is checked on hand-made attempt rows, and one small chunk is flown for real so that the rows
the harness writes can be recomputed from the truth it was given. Needs the cached truth batch of the
'moderate' canopy (sim/nav/generated/, built by run_nav_mc.py or by the first run of the Monte Carlo).
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.dont_write_bytecode = True
sys.path.insert(0, str(HERE))
import run_acquisition_mc as acq  # noqa: E402
import run_nav_mc as mc  # noqa: E402


def row(trial, attempt, t, ok=0, err=math.nan, status='none', sigma=1.5, reason=None, s1=(5.0, 1.0)):
    out = dict.fromkeys(acq.COLUMNS, math.nan)
    out.update(scenario='baseline', trial=trial, world=0, attempt=attempt, t=t, height=300.0 - 10.5*t, ok=ok,
               err_h=err, sigma_h=sigma if ok else math.nan, status=status, reason=reason or ('ok' if ok else 'no consensus'),
               polished=ok, s1_pos_err=s1[0], s1_yaw_err_deg=s1[1], matches=20 if ok else 0, match_far=1 if ok else 0,
               match_sd=0.8 if ok else math.nan, sites=18, ffts_256=123, ffts_128=180, host_s=0.5, level_err_deg=3.0,
               tilt_deg=8.0, err_att_deg=0.4 if ok else math.nan, err_z=0.5 if ok else math.nan,
               capture_rms=0.5*err if ok else math.nan, capture_max=0.6*err if ok else math.nan)
    return out


class Scoring(unittest.TestCase):
    def test_wilson_interval(self):
        p, lo, hi = acq.rate(0, 96)
        self.assertEqual(p, 0.0)
        self.assertEqual(lo, 0.0)
        self.assertAlmostEqual(hi, 0.0385, places=3)            # 1.96^2/(96 + 1.96^2)
        p, lo, hi = acq.rate(48, 96)
        self.assertAlmostEqual((lo + hi)/2, 0.5, places=9)
        self.assertAlmostEqual(hi - lo, 2*1.96*math.sqrt(0.25/96 + 1.96**2/(4*96**2))/(1 + 1.96**2/96), places=9)
        self.assertTrue(all(math.isnan(x) for x in acq.rate(0, 0)))

    def test_stats_ignore_missing_values(self):
        s = acq.stats([3.0, 4.0, math.nan, math.inf])
        self.assertEqual(s['n'], 2)
        self.assertAlmostEqual(s['rms'], math.sqrt(12.5))
        self.assertEqual(acq.stats([math.nan]), {'n': 0})

    def test_summary_counts_fixes_false_fixes_and_confirmation_times(self):
        times = acq.SCENARIOS['baseline'].times
        rows = []
        # Descent 0: fix at 3 s, confirmed at 4 s, both right.
        # Descent 1: a false fix at 2 s that is never confirmed, then right fixes at 5 s and 6 s (confirmed at 6 s).
        # Descent 2: nothing at all.     Descent 3: a false fix at 4 s confirmed (falsely) at 5 s.
        plan = {0: {3.0: (2.0, 'pending'), 4.0: (1.0, 'confirmed'), 5.0: (1.5, 'confirmed')},
                1: {2.0: (40.0, 'pending'), 5.0: (2.5, 'pending'), 6.0: (2.0, 'confirmed')},
                2: {},
                3: {4.0: (25.0, 'pending'), 5.0: (26.0, 'confirmed')}}
        for trial, fixes in plan.items():
            for attempt, t in enumerate(times):
                if t in fixes:
                    rows.append(row(trial, attempt, t, 1, fixes[t][0], fixes[t][1]))
                else:
                    rows.append(row(trial, attempt, t, s1=(80.0, 30.0) if trial == 2 else (5.0, 1.0)))
        s = acq.summarise('baseline', rows)
        self.assertEqual((s['trials'], s['attempts']), (4, 4*len(times)))
        self.assertEqual(s['fix_per_attempt'][0], 8/24)
        self.assertEqual(s['false_per_fix'][0], 3/8)
        self.assertEqual(s['false_per_attempt'][0], 3/24)
        self.assertEqual(s['stage1_right'][0], 18/24)
        self.assertEqual(s['fix_by']['2.0'][0], 1/4)
        self.assertEqual(s['fix_by']['3.0'][0], 2/4)
        self.assertEqual(s['fix_by']['7.0'][0], 3/4)
        self.assertEqual(s['confirmed_by']['4.0'][0], 1/4)
        self.assertEqual(s['confirmed_by']['5.0'][0], 2/4)
        self.assertEqual(s['confirmed_by']['7.0'][0], 3/4)
        self.assertEqual(s['first_confirmed_false'][0], 1/3)
        self.assertEqual(s['false_per_confirmed'][0], 1/4)
        self.assertEqual(s['error_all_fixes']['max'], 40.0)
        self.assertEqual(s['error_over_20m'][0], 3/8)
        self.assertEqual(s['outside_capture_per_fix'][0], 3/8)          # capture is 0.6 x error here: 24, 15, 15.6 m
        self.assertEqual(s['outside_capture_per_confirmed'][0], 1/4)
        self.assertEqual(s['error_correct_fixes']['max'], 2.5)
        self.assertEqual(s['matches_beyond_4m'][0], 8/160)
        self.assertEqual(s['reasons'], {'no consensus': 16, 'ok': 8})
        # 1.5 m sigma: errors 2, 1, 1.5, 2.5, 2 are within 3 m; 40, 25, 26 are not.
        self.assertEqual(s['within_2_sigma'][0], 5/8)
        text = acq.tables({'baseline': s})
        self.assertIn('| baseline |', text)
        self.assertIn('no consensus 16', text)

    def test_scenarios_are_well_formed(self):
        import farmland
        from cansat.coarse import CoarseConfig
        from dataclasses import replace
        for name, sc in acq.SCENARIOS.items():
            self.assertIn(sc.style, farmland.STYLES, name)
            self.assertIn(sc.canopy, mc.CANOPIES, name)
            self.assertIn(sc.levelling, ('imu', 'truth', 'none'), name)
            self.assertIn(sc.map_res, (1.0, 2.0), name)
            farmland.Season(**sc.season)
            replace(CoarseConfig(), **sc.config)
            self.assertTrue(all(0 < t < mc.DURATION for t in sc.times), name)
            self.assertLessEqual(sc.dispersion_m + 30.0, 320.0, name)      # stays inside the 300 m prior with margin for drift


class OneChunk(unittest.TestCase):
    def test_rows_can_be_recomputed_from_the_truth(self):
        directory = mc.truth_dir('moderate', 160)
        if not (directory/'done').exists():
            self.skipTest('no cached truth batch; run sim/nav/run_acquisition_mc.py once')
        acq.SCENARIOS['_test'] = acq.Scenario('test', times=(3.0, 4.0), trials=8)
        try:
            rows = acq.run_chunk(('_test', 1, 8))
        finally:
            del acq.SCENARIOS['_test']
        self.assertEqual([(r['trial'], r['attempt']) for r in rows], [(1, 0), (1, 1), (5, 0), (5, 1)])
        self.assertTrue(all(set(r) == set(acq.COLUMNS) for r in rows))
        truth = mc.load_truth('moderate', 160)
        for r in rows:
            k = int(round(r['t']/mc.NAV_DT))
            self.assertAlmostEqual(r['height'], float(truth['p'][k, r['trial'], 2]), places=6)
            self.assertTrue(0.0 <= r['level_err_deg'] < 30.0)
            self.assertEqual(r['ffts_256'], 123)                      # 60 yaw hypotheses, two FFTs each, three for the disc
            if r['ok']:
                self.assertTrue(r['err_h'] >= 0 and r['sigma_h'] >= 1.0 and r['matches'] >= 8)
            else:
                self.assertTrue(math.isnan(r['err_h']) and r['reason'] != 'ok')
        # The same chunk again gives the same rows (apart from the host time).
        acq.SCENARIOS['_test'] = acq.Scenario('test', times=(3.0, 4.0), trials=8)
        try:
            again = acq.run_chunk(('_test', 1, 8))
        finally:
            del acq.SCENARIOS['_test']
        for a, b in zip(rows, again):
            for key in acq.COLUMNS:
                if key != 'host_s':
                    same = a[key] == b[key] or (isinstance(a[key], float) and math.isnan(a[key]) and math.isnan(b[key]))
                    self.assertTrue(same, key)
        self.assertGreaterEqual(sum(r['ok'] for r in rows), 2)


if __name__ == '__main__':
    unittest.main()
