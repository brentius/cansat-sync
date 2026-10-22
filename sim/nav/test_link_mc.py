"""GPL-3.0-only. Checks of the link-in-the-loop Monte Carlo and of the 1 kHz IMU integration (synthetic)."""
import math
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]/'code'))
import run_link_mc as lm  # noqa: E402
import run_nav_mc as mc  # noqa: E402
import truth as tr  # noqa: E402
from cansat import link  # noqa: E402

CUT = (1.0, 1.5)


class LinkInTheLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.saved = (mc.CACHE, mc.DURATION)
        cls.tmp = tempfile.TemporaryDirectory()
        mc.CACHE, mc.DURATION = Path(cls.tmp.name), 3.0
        mc._TRUTH.clear()
        mc.build_truth('moderate', 2)
        for scheme in ('cumulative', 'delta', 'redundant'):
            lm.VARIANTS['test_cut_' + scheme] = lm.variant(scheme, outage=CUT)
        lm.VARIANTS['test_all_' + 'cumulative'] = lm.variant('cumulative', loss=0.05, outage=CUT, ber=1e-3)
        cls.runs = {}

    @classmethod
    def tearDownClass(cls):
        mc._TRUTH.clear()
        mc.CACHE, mc.DURATION = cls.saved
        for name in [n for n in lm.VARIANTS if n.startswith('test_')]:
            del lm.VARIANTS[name]
        try:
            cls.tmp.cleanup()
        except OSError:
            pass

    def run_variant(self, name, j=0):
        if (name, j) not in self.runs:
            self.runs[name, j] = lm.run_descent((name, j, 2))
        return self.runs[name, j]

    def test_this_loop_without_the_link_is_run_nav_mc_exactly(self):
        for j in (0, 1):
            a, b = self.run_variant('baseline', j), self.run_variant('direct', j)
            for key in ('err', 'sig', 'nees', 'counts', 'innov', 'pl', 'height', 'swing'):
                np.testing.assert_array_equal(a[key], b[key], err_msg=key)

    def test_clean_codec_changes_the_result_only_at_quantisation_level(self):
        a = self.run_variant('baseline')
        for scheme, limit in (('cumulative', 2e-3), ('delta', 2e-2), ('redundant', 2e-2)):
            b = self.run_variant(scheme + '_clean')
            rows = np.isfinite(a['height'])
            self.assertEqual(rows.sum(), 15)
            self.assertLess(np.max(abs(a['err'][rows, :3] - b['err'][rows, :3])), limit, scheme)
            sent, undecoded, imu, longest, dropped, concealed, rejected, breaks = b['link']
            self.assertEqual((undecoded, longest, dropped, concealed, rejected, breaks), (0, 1, 0, 0, 0, 0), scheme)
            self.assertEqual(imu, 300)
            self.assertEqual(sent, 300 + 30 + 30 + 15 + 3 + 3)       # imu, baro, gnss, trig, status, pps

    def test_every_scheme_is_given_the_same_losses(self):
        v = lm.VARIANTS['cumulative_loss_5']
        a, b = lm.Uart(v, 1, 300), lm.Uart(lm.VARIANTS['delta_loss_5'], 1, 300)
        np.testing.assert_array_equal(a.hit, b.hit)
        self.assertFalse(np.array_equal(a.hit, lm.Uart(v, 0, 300).hit))

    def test_lost_messages_are_bridged_by_the_cumulative_stream_and_interpolated_by_the_others(self):
        base = self.run_variant('baseline')
        rows = np.isfinite(base['height'])
        got = {s: self.run_variant(s + '_loss_5') for s in ('cumulative', 'delta', 'redundant')}
        for scheme, out in got.items():
            sent, undecoded, imu, longest, dropped, concealed, rejected, breaks = out['link']
            self.assertTrue(np.all(np.isfinite(out['err'][rows])), scheme)
            self.assertGreater(undecoded, 5)
            self.assertLess(imu, 300)
            self.assertGreaterEqual(longest, 2)
            self.assertLess(np.max(abs(out['err'][rows, :3] - base['err'][rows, :3])), 0.5, scheme)
        self.assertEqual(got['cumulative']['link'][5], 0)
        self.assertGreater(got['delta']['link'][5], 5)
        self.assertLess(got['redundant']['link'][5], got['delta']['link'][5])

    def test_an_outage_is_bridged_and_the_images_inside_it_are_not_used(self):
        base = self.run_variant('baseline')
        rows = np.isfinite(base['height'])
        for scheme in ('cumulative', 'delta', 'redundant'):
            out = self.run_variant('test_cut_' + scheme)
            sent, undecoded, imu, longest, dropped, concealed, rejected, breaks = out['link']
            self.assertEqual((longest, dropped, breaks), (51, 3, 0), scheme)
            self.assertEqual(imu, 250)
            self.assertEqual(concealed, 0 if scheme == 'cumulative' else (49 if scheme == 'redundant' else 50))
            self.assertTrue(np.all(np.isfinite(out['err'][rows])), scheme)
            # No landmarks were offered in the three images inside the cut.
            inside = [i for i in range(15) if CUT[0] <= i*0.2 <= CUT[1] + 1e-9]
            self.assertEqual(inside, [5, 6, 7])
            self.assertTrue(np.all(out['counts'][[5, 6], 0] == 0), scheme)
            self.assertGreater(out['counts'][8, 0], 0)
        exact = self.run_variant('test_cut_cumulative')
        # Half a second of inertial data was bridged in one step and three images went unused
        # while the filter was still converging: it stays within a metre of the baseline.
        self.assertLess(np.max(abs(exact['err'][8:15, :3] - base['err'][8:15, :3])), 1.0)

    def test_everything_at_once_still_navigates(self):
        out = self.run_variant('test_all_cumulative')
        rows = np.isfinite(self.run_variant('baseline')['height'])
        self.assertTrue(np.all(np.isfinite(out['err'][rows])))
        self.assertLess(np.max(np.hypot(out['err'][rows, 0], out['err'][rows, 1])[3:]), 3.0)
        summary, _ = lm.summarise('test_all_cumulative', [out, self.run_variant('test_all_cumulative', 1)])
        self.assertEqual(summary['descents'], 2)
        self.assertGreater(summary['frames_not_decoded_fraction'], 0.1)


class OneKilohertz(unittest.TestCase):
    """link.ImuAccumulator.add_sample fed the descent model's own 1 kHz samples, against the
    model's exact pose increments: the check of pre-integration on the STM32 that the 100 Hz
    Monte Carlo cannot make."""

    def test_integrating_1_khz_samples_reproduces_the_exact_increments(self):
        n, seconds = 3, 4.0
        rng = np.random.default_rng(5)
        params = mc.CANOPIES['violent']
        wind, _, _ = tr.make_wind(n, rng)
        sim = tr.Descent(params, wind, n)
        sim.start(rng)
        accs = [link.ImuAccumulator() for _ in range(n)]
        difs = [link.ImuDifferencer() for _ in range(n)]
        gravity = np.array([0.0, 0.0, -tr.G0])*0.01
        _, v0, r0 = sim.imu_state()
        worst_theta = worst_v = 0.0
        drift = np.zeros((n, 3))
        swing = 0.0
        for k in range(round(seconds/params.dt) + 1):
            if k and k % 10 == 0:
                # Truth for the 10 ms just ended, from the pose itself.
                _, v1, r1 = sim.imu_state()
                dtheta = tr.rotation_vectors(np.einsum('nji,njk->nik', r0, r1))
                dv = np.einsum('nji,nj->ni', r0, v1 - v0 - gravity)
                v0, r0 = v1, r1
            force, rate = sim.step(rng)                    # the sample at the start of step k
            for i in range(n):
                accs[i].add_sample(tuple(rate[i]), tuple(force[i]), params.dt)
            if k % 10 == 0:
                for i in range(n):
                    inc = difs[i].feed(accs[i].snapshot(k*1000))
                    if k:
                        e_theta = np.array(inc.dtheta) - dtheta[i]
                        worst_theta = max(worst_theta, float(np.max(abs(e_theta))))
                        worst_v = max(worst_v, float(np.max(abs(np.array(inc.dv) - dv[i]))))
                        drift[i] += e_theta
                swing = max(swing, float(np.max(sim.swing_angle())))
        self.assertGreater(swing, math.radians(15))                # the motion is not a gentle one
        gyro_noise = math.radians(0.014)*math.sqrt(0.01)           # rad per interval
        accel_noise = 175e-6*tr.G0*math.sqrt(0.01)                 # m/s per interval
        self.assertLess(worst_theta, gyro_noise/20)
        self.assertLess(worst_v, accel_noise/3)
        # Summed over 4 s, against the gyro's own random walk over the same time.
        self.assertLess(np.max(abs(drift)), math.radians(0.014)*math.sqrt(seconds)/50)


if __name__ == '__main__':
    unittest.main()
