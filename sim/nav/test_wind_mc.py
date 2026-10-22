"""GPL-3.0-only. Checks of the wind Monte Carlo plumbing (synthetic)."""
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]/'code'))
import run_nav_mc as nm  # noqa: E402
import run_wind_mc as wm  # noqa: E402
import truth as tr  # noqa: E402


class Pieces(unittest.TestCase):
    def test_layer_truth_is_the_height_average_of_the_profile(self):
        z = np.arange(0.0, 420.5, 0.5)
        profile = np.column_stack((0.01*z, np.full(len(z), -2.0), np.zeros(len(z))))
        layers = wm.layer_truth(profile, 0.5)
        self.assertEqual(layers.shape, (15, 2))
        np.testing.assert_allclose(layers[:, 0], 0.01*(wm.EDGES[:-1] + 10.0), atol=1e-12)
        np.testing.assert_allclose(layers[:, 1], -2.0)
        self.assertEqual(wm.EDGES[wm.SCIENCE][0], 80.0)
        self.assertEqual(len(wm.EDGES[:-1][wm.SCIENCE]), 11)

    def test_variants_cover_the_comparisons_the_report_makes(self):
        v = wm.variants(wm.SETUP, 0.0624)
        self.assertEqual(v['gps'][1].glide_sd, 0.0)                     # the proposal's J.1 has no glide state
        self.assertFalse(v['gps'][2] or v['trn'][2])
        self.assertTrue(v['gps_air'][2] and not v['gps_air'][3])
        self.assertIsNone(v['gps_air_cal_no_canopy_model'][1].lag_time)
        # An uncalibrated offset is worth metres per second; a hang test leaves tenths.
        self.assertGreater(v['gps_air'][1].offset_sd, 3.0)
        self.assertLess(v['gps_air_cal'][1].offset_sd, 0.2)
        self.assertEqual(v['trn'][1].error_sd, 1.0)

    def test_rate_constant_matches_the_truth_model_drag(self):
        p = wm.CANOPIES['moderate']
        kappa = 0.5*tr.RHO*p.payload_cds*wm.V_REF/p.payload_mass
        # 1 m/s of sideways airspeed at the design descent speed is about 6.3 mg.
        self.assertAlmostEqual(kappa/tr.G0*1e3, 6.35, places=1)

    def test_truth_directory_depends_on_the_canopy_and_duration(self):
        a, b = wm.truth_dir('benign', 4), wm.truth_dir('mild', 4)
        self.assertNotEqual(a, b)
        self.assertNotEqual(a.name, nm.truth_dir('benign', 4).name)     # never reuses the 28 s navigation batches
        self.assertIn('mild', wm.CANOPIES)
        self.assertEqual(wm.CANOPIES['mild'].trim_deg, 5.0)

    def test_summary_recovers_planted_errors(self):
        rng = np.random.default_rng(0)
        n, layers = 300, len(wm.EDGES) - 1
        truth = rng.normal(size=(n, layers, 2))
        stack = {'truth_layers': truth, 'glide_true': np.full((n, 2), 1.0), 'airspeed_error_cal': np.full(n, 0.4),
                 'airspeed_error_raw': np.full(n, 2.0), 'airspeed_true_rms': np.full(n, 1.3),
                 'nav_velocity_rms': np.full(n, 0.2), 'turns': np.full(n, 1.5)}
        for name in list(wm.variants(wm.SETUP, 1.0)) + ['direct']:
            stack[name + '_mean'] = truth + 0.3*rng.normal(size=truth.shape)
            stack[name + '_sd'] = np.full(truth.shape, 0.3)
            stack[name + '_nis'] = np.ones(n)
            stack[name + '_glide'] = np.full((n, 2), 1.0)
            stack[name + '_glide_sd'] = np.full((n, 2), 0.5)
        stack['gps_mean'] = truth + 1.0                                  # a pure bias the comparison cannot see
        stack['trn_mean'] = truth + 1.0 + 0.2*rng.normal(size=truth.shape)
        for name in wm.PREDICTORS:
            e = 5.0*rng.normal(size=(n, len(wm.PREDICT_AT), 2))
            stack[f'pred_{name}_err'] = e
            stack[f'pred_{name}_d2'] = (e**2).sum(axis=2)/25.0
            stack[f'pred_{name}_time'] = np.zeros((n, len(wm.PREDICT_AT)))
            stack[f'pred_{name}_sigma'] = np.full((n, len(wm.PREDICT_AT)), 5.0*math.sqrt(2))
            stack[f'pred_{name}_ignored'] = np.zeros(n)
        s = wm.summarise('benign', stack, wm.SETUP)
        self.assertAlmostEqual(s['wind']['gps']['layer_rms_80_300'], 1.0, places=6)
        self.assertFalse(s['wind']['gps']['meets_requirement'])
        self.assertAlmostEqual(s['wind']['gps_air_cal']['layer_rms_80_300'], 0.3, delta=0.02)
        self.assertAlmostEqual(s['wind']['gps_air_cal']['coverage95_80_300'], 0.95, delta=0.02)
        self.assertEqual(s['wind']['gps']['coverage95_80_300'], 0.0)
        # S1 sees only the difference: both retrievals are 1 m/s wrong and it reads 0.2.
        self.assertAlmostEqual(s['s1']['trn_minus_gps']['layer_rms_80_300'], 0.2, delta=0.02)
        self.assertTrue(s['s1']['trn_minus_gps']['passes_0p5'])
        self.assertAlmostEqual(s['landing']['gps']['coverage95'][0], 0.95, delta=0.04)
        self.assertAlmostEqual(s['landing']['gps']['radial_rms_m'][0], 5.0*math.sqrt(2), delta=0.5)


class Chain(unittest.TestCase):
    """Two short descents through the whole chain: shapes, finiteness and the paired controls."""

    @classmethod
    def setUpClass(cls):
        cls.saved = (wm.DURATION, wm.CACHE, dict(nm._TRUTH))
        cls.tmp = tempfile.TemporaryDirectory()
        wm.DURATION, wm.CACHE = 9.0, Path(cls.tmp.name)
        wm.build_truth('moderate', 2)
        cls.recs = [wm.run_descent(('moderate', j, 2, wm.SETUP)) for j in range(2)]

    @classmethod
    def tearDownClass(cls):
        wm.DURATION, wm.CACHE = cls.saved[0], cls.saved[1]
        nm._TRUTH.clear()
        nm._TRUTH.update(cls.saved[2])
        cls.tmp.cleanup()

    def test_outputs(self):
        for rec in self.recs:
            self.assertEqual(rec['truth_layers'].shape, (15, 2))
            # 9 s from 300 m reaches about 205 m: the top four layers hold data.
            filled = np.isfinite(rec['gps_mean'][:, 0])
            self.assertTrue(filled[11:].all())
            self.assertTrue(np.all(np.isfinite(rec['gps_air_cal_mean'][filled])))
            self.assertTrue(np.all(rec['gps_sd'][filled] > 0))
            self.assertEqual(float(rec['pred_gps_ignored']), 0.0)
            self.assertTrue(np.isfinite(rec['pred_trn_err'][0]).all())      # a prediction was made at 250 m
            self.assertLess(float(rec['airspeed_error_cal']), float(rec['airspeed_error_raw']))
            self.assertLess(float(rec['nav_velocity_rms']), 1.0)

    def test_the_airspeed_run_differs_only_where_it_should(self):
        rec = self.recs[0]
        self.assertFalse(np.allclose(rec['gps_mean'][12:], rec['gps_air_cal_mean'][12:]))
        np.testing.assert_allclose(rec['gps_glide'], 0.0)                 # no glide state in J.1
        self.assertGreater(float(np.abs(rec['gps_air_cal_glide']).max()), 0.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
