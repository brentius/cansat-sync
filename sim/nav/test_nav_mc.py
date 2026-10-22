"""GPL-3.0-only. Checks of the Monte Carlo plumbing and the analytic fix bounds (synthetic)."""
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]/'code'))
import fix_bounds as fb  # noqa: E402
import run_nav_mc as mc  # noqa: E402
from cansat.eskf import LEVEL, exp_so3  # noqa: E402

DEG = math.pi/180


class Selection(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(1)
        self.pixels = np.column_stack((rng.uniform(-200, 1700, 4000), rng.uniform(-200, 1300, 4000)))
        self.depth = np.full(4000, 200.0)

    def test_one_per_cell_inside_the_image_and_spread(self):
        chosen = mc.select_landmarks(self.pixels, self.depth, 30, 1.0)
        self.assertEqual(len(chosen), 30)
        px = self.pixels[chosen]
        self.assertTrue(np.all((px[:, 0] > 16) & (px[:, 0] < 1440) & (px[:, 1] > 16) & (px[:, 1] < 1072)))
        cells = set(zip((px[:, 0]//(1456/6)).astype(int), (px[:, 1]//(1088/5)).astype(int)))
        self.assertGreaterEqual(len(cells), 28)
        # Landmarks reach the corners, which is what separates tilt from translation.
        self.assertGreater(np.ptp(px[:, 0]), 1100)
        self.assertGreater(np.ptp(px[:, 1]), 800)

    def test_limit_keeps_the_spread(self):
        chosen = mc.select_landmarks(self.pixels, self.depth, 8, 1.0)
        self.assertEqual(len(chosen), 8)
        self.assertGreater(np.ptp(self.pixels[chosen, 0]), 900)
        self.assertGreater(np.ptp(self.pixels[chosen, 1]), 600)

    def test_central_fraction_and_depth_mask(self):
        chosen = mc.select_landmarks(self.pixels, self.depth, 30, 0.5)
        px = self.pixels[chosen]
        self.assertTrue(np.all(abs(px[:, 0] - 728) < 364) and np.all(abs(px[:, 1] - 544) < 272))
        depth = self.depth.copy()
        depth[::2] = -5.0
        chosen = mc.select_landmarks(self.pixels, depth, 30, 1.0)
        self.assertTrue(np.all(chosen % 2 == 1))
        self.assertEqual(len(mc.select_landmarks(self.pixels, -self.depth, 30, 1.0)), 0)

    def test_search_patch_must_fit_inside_the_image(self):
        # 24 m on the ground is 87 px at 200 m depth and 217 px at 80 m.
        for depth, edge in ((200.0, 87.0), (80.0, 217.4)):
            chosen = mc.select_landmarks(self.pixels, np.full(4000, depth), 30, 1.0, search_m=24.0)
            px = self.pixels[chosen]
            self.assertTrue(np.all((px[:, 0] > 16 + edge - 0.5) & (px[:, 0] < 1440 - edge + 0.5)))
            self.assertTrue(np.all((px[:, 1] > 16 + edge - 0.5) & (px[:, 1] < 1072 - edge + 0.5)))
        self.assertLess(len(chosen), 30)                 # the outer cells are no longer reachable

    def test_same_landmarks_are_reused_under_small_motion(self):
        a = mc.select_landmarks(self.pixels, self.depth, 30, 1.0)
        b = mc.select_landmarks(self.pixels + [3.0, -2.0], self.depth, 30, 1.0)
        self.assertGreater(len(set(a) & set(b)), 22)


class Bounds(unittest.TestCase):
    def setUp(self):
        self.cam = fb.camera()
        self.r = LEVEL @ exp_so3(np.array([8*DEG, 0.0, 0.0]))

    def info(self, height, fraction=1.0, **kw):
        p = np.array([0.0, 0.0, height])
        return fb.information(self.cam, fb.ground_points(self.cam, fb.grid_pixels(fraction), p, self.r), p, self.r, **kw)

    def test_ground_points_project_back_onto_the_grid(self):
        p = np.array([3.0, -4.0, 180.0])
        pixels = fb.grid_pixels()
        points = fb.ground_points(self.cam, pixels, p, self.r)
        np.testing.assert_allclose(points[:, 2], 0.0, atol=1e-9)
        np.testing.assert_allclose(self.cam.project(points, p, self.r)[0], pixels, atol=1e-9)

    def test_attitude_knowledge_and_field_of_view_ordering(self):
        info = self.info(300.0)
        free, prior, known = (fb.horizontal_sigma(info, a) for a in (None, 0.1*DEG, 0))
        self.assertTrue(known < prior < free)
        # Not knowing attitude costs about a factor of four at full field of view...
        self.assertTrue(3.0 < free/known < 5.5)
        # ...and far more once the landmarks are confined to the image centre.
        half, third = fb.horizontal_sigma(self.info(300.0, 0.5)), fb.horizontal_sigma(self.info(300.0, 1/3))
        self.assertTrue(free < half < third)
        self.assertGreater(third/free, 5.0)

    def test_bound_matches_monte_carlo_least_squares(self):
        # Gauss-Newton from the truth with noisy pixels attains the bound.
        rng = np.random.default_rng(2)
        p = np.array([0.0, 0.0, 200.0])
        points = fb.ground_points(self.cam, fb.grid_pixels(), p, self.r)
        info = self.info(200.0)
        pixels, depth, jp, jt = self.cam.jacobians(points, p, self.r)
        sd = np.sqrt(0.3**2 + (0.15*self.cam.fx/depth)**2)
        h = np.concatenate((jp, jt), axis=2).reshape(-1, 6)
        w = np.repeat(1/sd**2, 2)
        solve = np.linalg.solve(h.T @ (h*w[:, None]), (h*w[:, None]).T)
        errors = np.array([solve @ (np.repeat(sd, 2)*rng.normal(size=60)) for _ in range(4000)])
        bound = np.linalg.inv(info)
        scale = np.sqrt(np.outer(np.diag(bound), np.diag(bound)))
        # 4000 draws: sampling error is about 0.02 of each element's scale.
        self.assertLess(np.max(abs(np.cov(errors.T) - bound)/scale), 0.1)

    def test_relief_dominates_once_it_exceeds_a_metre(self):
        clean = fb.horizontal_sigma(self.info(300.0))
        rough = fb.horizontal_sigma(self.info(300.0, map_var=(0.09, 0.09, 25.0)))
        self.assertGreater(rough/clean, 8.0)

    def test_budget_reproduces_the_proposal_table(self):
        # Section 12: attitude 1.6-5.2 m at 300 m, timing 0.1 m, total about 3-6 m.
        low, high = fb.budget(300.0, 0.3), fb.budget(300.0, 1.0)
        self.assertAlmostEqual(low['attitude'], 1.57, places=2)
        self.assertAlmostEqual(high['attitude'], 5.24, places=2)
        self.assertAlmostEqual(low['timing'], 0.105, places=3)
        self.assertAlmostEqual(low['matching'], 0.0141, places=4)
        self.assertTrue(3.0 < low['total'] < 3.3 and 5.7 < high['total'] < 6.0)
        self.assertTrue(2.5 < fb.budget(100.0, 0.3)['total'] < 3.5)


class ClosedLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.saved = (mc.CACHE, mc.DURATION)
        cls.tmp = tempfile.TemporaryDirectory()
        mc.CACHE, mc.DURATION = Path(cls.tmp.name), 3.0
        mc._TRUTH.clear()
        mc.build_truth('moderate', 2)

    @classmethod
    def tearDownClass(cls):
        mc._TRUTH.clear()                      # release the memory maps before the directory goes
        mc.CACHE, mc.DURATION = cls.saved
        try:
            cls.tmp.cleanup()
        except OSError:
            pass

    def test_matched_run_tracks_truth_and_is_reproducible(self):
        a = mc.run_descent(('matched_moderate', 0, 2))
        b = mc.run_descent(('matched_moderate', 0, 2))
        np.testing.assert_array_equal(a['err'], b['err'])
        rows = np.isfinite(a['height'])
        self.assertGreaterEqual(rows.sum(), 14)
        horizontal = np.hypot(a['err'][rows, 0], a['err'][rows, 1])
        # Starts 3 m (1 sigma) out and is pulled in by the first image.
        self.assertLess(np.max(horizontal[3:]), 0.6)
        self.assertGreater(np.nanmin(a['counts'][rows, 1]), 20)        # landmarks accepted every frame
        self.assertTrue(np.all(a['sig'][rows] > 0))

    def test_scenarios_share_truth_and_draws_but_differ_in_errors(self):
        nominal = mc.run_descent(('nominal_moderate', 1, 2))
        matched = mc.run_descent(('matched_moderate', 1, 2))
        np.testing.assert_array_equal(nominal['height'], matched['height'])
        np.testing.assert_array_equal(nominal['swing'], matched['swing'])
        self.assertFalse(np.allclose(nominal['err'], matched['err'], equal_nan=True))

    def test_outliers_are_counted_and_mostly_rejected(self):
        out = mc.run_descent(('outliers_40pct', 0, 2))
        counts = out['counts'][np.isfinite(out['height'])]
        offered, bad, bad_accepted = counts[:, 0].sum(), counts[:, 4].sum(), counts[:, 3].sum()
        self.assertTrue(0.3 < bad/offered < 0.5)
        self.assertLess(bad_accepted/bad, 0.15)

    def test_first_fix_start_acquires_with_unknown_yaw_and_then_tracks(self):
        out = mc.run_descent(('first_fix_start', 0, 2))
        frame, refusals, error, sigma = out['first_fix']
        self.assertTrue(np.isfinite(frame) and frame <= 3)
        self.assertLess(error, 3.0)                                    # the fix itself, metres
        self.assertLess(error, 4*sigma + 0.5)
        rows = np.isfinite(out['height'])
        self.assertEqual(int(np.flatnonzero(rows)[0]), int(frame))     # nothing is recorded before the fix
        horizontal = np.hypot(out['err'][rows, 0], out['err'][rows, 1])
        self.assertLess(np.max(horizontal[2:]), 2.0)
        summary, _, _ = mc.summarise('first_fix_start', [out, mc.run_descent(('first_fix_start', 1, 2))])
        self.assertEqual(summary['first_fix_never'], 0.0)
        self.assertEqual(summary['T1_fraction_within_10m'], 1.0)

    def test_a_descent_that_never_navigates_fails_t1(self):
        out = mc.run_descent(('nominal_moderate', 0, 2))
        lost = {k: (np.full_like(v, np.nan) if k != 'first_fix' else np.array([np.nan, 9, np.nan, np.nan], dtype=np.float32))
                for k, v in out.items()}
        summary, _, _ = mc.summarise('nominal_moderate', [out, lost])
        self.assertEqual(summary['T1_fraction_within_10m'], 0.5)
        self.assertEqual(summary['first_fix_never'], 0.5)

    def test_every_scenario_is_well_formed(self):
        for name, sc in mc.SCENARIOS.items():
            self.assertIn(sc['canopy'], mc.CANOPIES, name)
            self.assertIn(sc['sensors'].camera_delay, mc.DELAYS, name)
            self.assertIn(sc['sensors'].imu_mode, ('ideal', 'mean', 'inst'), name)
            self.assertAlmostEqual(sc['sensors'].frame_dt/mc.FRAME_DT, round(sc['sensors'].frame_dt/mc.FRAME_DT), msg=name)


if __name__ == '__main__':
    unittest.main()
