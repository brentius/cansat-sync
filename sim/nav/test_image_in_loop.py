"""GPL-3.0-only. Fast checks of the SYNTHETIC image-in-the-loop helpers (sim/nav/image_in_loop.py).

Usage: python sim/nav/test_image_in_loop.py
Small images and a few frames keep the whole file to well under 30 s; nothing here is a flight result.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import image_in_loop as iil  # noqa: E402  (also imports run_nav_mc, ncc_attitude_tolerance and the filter)
from feasibility import render_image  # noqa: E402

nat, mc = iil.nat, iil.mc


def pose(height, tilt_deg, yaw=0.7, axis=1.1, xy=(40.0, -25.0)):
    """Body-to-world rotation and IMU position of a level can tilted by tilt_deg and turned by yaw."""
    tilt = nat.rotation(math.radians(tilt_deg)*np.array([math.cos(axis), math.sin(axis), 0.0]))
    r = tilt @ nat.rotation([0.0, 0.0, yaw]) @ iil.LEVEL
    return np.array([xy[0], xy[1], height]), r


class RendererChecks(unittest.TestCase):
    def test_render_frame_equals_render_image(self):
        p, r = pose(140.0, 9.0)
        centre = p + r @ iil.LEVER
        size, focal = (136, 182), 181.15
        for relief, tolerance in ((0.0, 0.01), (2.0, 0.05)):
            fast = iil.render_frame(centre, r, size, focal, relief)
            slow = render_image(centre, r, size, focal, relief)
            self.assertLess(np.max(np.abs(fast - slow)), tolerance, relief)
        # The principal point is at the image centre: the pixel (cols/2, rows/2) sees the ground under the boresight.
        level = iil.LEVEL
        image = iil.render_frame(np.array([5.0, 6.0, 100.0]), level, (100, 120), 100.0)
        self.assertAlmostEqual(image[50, 60], float(nat.terrain(5.0, 6.0)), places=3)


class GeometryChecks(unittest.TestCase):
    def setUp(self):
        self.cam = mc.nominal_camera()
        self.p, self.r = pose(120.0, 7.0)
        # A deliberately wrong predicted pose, as the filter's would be.
        self.p_hat = self.p + np.array([2.5, -1.5, 3.0])
        self.r_hat = nat.rotation([0.02, -0.03, 0.05]) @ self.r

    def test_filter_camera_and_renderer_agree(self):
        """eskf.Camera.project (full resolution, lever) equals the renderer's convention at any binning."""
        rng = np.random.default_rng(1)
        ground = np.column_stack((self.p[:2] + rng.uniform(-60, 60, (30, 2)), np.zeros(30)))
        full, depth = self.cam.project(ground, self.p, self.r)
        for binning in (1, 2, 4):
            size = (iil.FULL_SIZE[0]//binning, iil.FULL_SIZE[1]//binning)
            via = nat.ground_to_pixel(ground[:, :2], self.p + self.r @ iil.LEVER, self.r, iil.FOCAL/binning, size)
            np.testing.assert_allclose(via*binning, full, atol=1e-9)
        self.assertTrue(np.all(depth > 100.0))

    def test_predicted_pose_pixel_round_trip(self):
        """map point -> pixel -> ground -> pixel under the PREDICTED pose, self-consistent to 1e-9 px."""
        rng = np.random.default_rng(2)
        ground = self.p[:2] + rng.uniform(-60, 60, (50, 2))
        for binning in (1, 2):
            size = (iil.FULL_SIZE[0]//binning, iil.FULL_SIZE[1]//binning)
            focal = iil.FOCAL/binning
            centre = self.p_hat + self.r_hat @ iil.LEVER
            pixel = nat.ground_to_pixel(ground, centre, self.r_hat, focal, size)
            back = nat.pixel_to_ground(pixel, centre, self.r_hat, focal, size)
            np.testing.assert_allclose(back, ground, atol=1e-9)
            again = nat.ground_to_pixel(back, centre, self.r_hat, focal, size)
            self.assertLess(np.max(np.abs(again - pixel)), 1e-9)
        # And the filter's own projection of the same map points agrees with the matcher's mapping.
        full, _ = self.cam.project(np.column_stack((ground, np.zeros(50))), self.p_hat, self.r_hat)
        np.testing.assert_allclose(full, nat.ground_to_pixel(ground, self.p_hat + self.r_hat @ iil.LEVER, self.r_hat,
                                                              iil.FOCAL, iil.FULL_SIZE), atol=1e-9)

    def test_selection_keeps_whole_search_patch_in_view(self):
        binning = 2
        size = (iil.FULL_SIZE[0]//binning, iil.FULL_SIZE[1]//binning)
        rng = np.random.default_rng(3)
        gx, gy = np.meshgrid(np.arange(-300.0, 340.0, 40.0), np.arange(-300.0, 340.0, 40.0))
        points = np.column_stack((gx.ravel(), gy.ravel(), np.zeros(gx.size))) + rng.uniform(-16, 16, (gx.size, 3))*[1, 1, 0]
        for height in (300.0, 150.0, 82.0):
            p, r = pose(height, 6.0, xy=(0.0, 0.0))
            chosen, candidates = iil.select_predicted(self.cam, points, p, r, size, binning, 30)
            self.assertGreaterEqual(candidates, len(chosen))
            self.assertLessEqual(len(chosen), 30)
            self.assertGreater(len(chosen), 0)
            centre = p + r @ iil.LEVER
            image = np.zeros(size)
            for q in chosen:        # the matcher's own rectification must find the whole patch inside the image
                patch = nat.rectify(image, points[q, :2], centre, r, iil.BIG_PX, iil.SCALE_M, iil.FOCAL/binning)
                self.assertTrue(np.all(np.isfinite(patch)), (height, q))
        # A landmark whose patch crosses the image edge is not selected.
        p, r = pose(100.0, 0.0, xy=(0.0, 0.0))
        edge = np.array([[iil.FULL_SIZE[1]/2/iil.FOCAL*100.0 - 10.0, 0.0, 0.0]])
        self.assertFalse(iil.patch_in_view(self.cam, edge[:, :2], p, iil.LEVEL, size, binning)[0])


class MatcherChecks(unittest.TestCase):
    def test_noise_free_pixel_recovered_at_known_pose(self):
        """At a perfectly known pose a landmark's pixel is recovered to well under 0.3 px (full resolution)."""
        cam = mc.nominal_camera()
        for height, tilt in ((300.0, 0.0), (120.0, 8.0), (82.0, 10.0)):
            p, r = pose(height, tilt, xy=(0.0, 0.0))
            centre = p + r @ iil.LEVER
            size, focal = (544, 728), iil.FOCAL/2
            image = iil.render_frame(centre, r, size, focal)
            rng = np.random.default_rng(int(height))
            points = np.column_stack((rng.uniform(-0.3, 0.3, 40)*height, rng.uniform(-0.2, 0.2, 40)*height, np.zeros(40)))
            view = np.flatnonzero(iil.patch_in_view(cam, points[:, :2], p, r, size, 2))
            self.assertGreaterEqual(len(view), 3)
            for q in view[:6]:
                found = iil.match_landmark(image, points[q, :2], centre, r, focal)
                true_px, _ = cam.project(points[q:q + 1], p, r)
                self.assertTrue(found['accepted'], (height, q))
                self.assertLess(np.hypot(*(found['uv']*2 - true_px[0])), 0.3, (height, q))
                self.assertGreater(found['score'], 0.99)

    def test_same_answer_as_the_reference_matcher(self):
        p, r = pose(150.0, 5.0, xy=(0.0, 0.0))
        centre = p + r @ iil.LEVER
        image = iil.render_frame(centre, r, (544, 728), iil.FOCAL/2)
        image = image + np.random.default_rng(5).normal(0, 2.0, image.shape)
        pred_pos, pred_r = centre + np.array([2.0, -1.0, 1.0]), nat.rotation([0.01, 0.02, -0.03]) @ r
        point = np.array([3.0, -2.0])
        mine = iil.match_landmark(image, point, pred_pos, pred_r, iil.FOCAL/2)
        ref = nat.match_landmark(image, np.array([*point, 0.0]), pred_pos, pred_r, nat.CONFIGS['FINE'], iil.FOCAL/2)
        self.assertEqual(mine['accepted'], ref['accepted'])
        np.testing.assert_allclose(mine['uv'], ref['uv'], atol=1e-12)
        self.assertAlmostEqual(mine['score'], ref['score'], places=12)

    def test_no_pose_error_is_all_correct_with_subpixel_error(self):
        for noise in (0.0, 2.0):
            x = iil.perfect_pose_check(binning=2, heights=(300.0, 120.0), tilts_deg=(0.0, 8.0), noise=noise)
            self.assertEqual((x['accepted_fraction'], x['correct_fraction']), (1.0, 1.0), noise)
            self.assertLess(x['pixel_error_median_px'], 0.1)
            self.assertLess(x['pixel_error_max_px'], 0.3)
            self.assertLess(x['filter_camera_vs_renderer_geometry_max_px'], 1e-9)

    def test_position_error_beyond_the_window_is_not_scored_correct(self):
        """A 24 m prediction error puts the truth outside the +/-16 m window: nothing can be correct."""
        cam = mc.nominal_camera()
        p, r = pose(120.0, 3.0, xy=(0.0, 0.0))
        centre = p + r @ iil.LEVER
        size, focal = (544, 728), iil.FOCAL/2
        image = iil.render_frame(centre, r, size, focal)
        wrong = centre + np.array([24.0, 0.0, 0.0])
        rng = np.random.default_rng(8)
        points = np.column_stack((rng.uniform(-0.2, 0.2, 20)*120, rng.uniform(-0.15, 0.15, 20)*120, np.zeros(20)))
        view = np.flatnonzero(iil.patch_in_view(cam, points[:, :2], p + [24, 0, 0], r, size, 2))
        for q in view[:6]:
            found = iil.match_landmark(image, points[q, :2], wrong, r, focal)
            true_px, z = cam.project(points[q:q + 1], p, r)
            error_m = np.hypot(*(found['uv']*2 - true_px[0]))*z[0]/iil.FOCAL
            self.assertGreater(error_m, iil.FALSE_M)


class BookkeepingChecks(unittest.TestCase):
    def test_match_statistics(self):
        cols = len(iil.LANDMARK_COLS)
        rows = np.zeros((10, cols))
        idx = {k: i for i, k in enumerate(iil.LANDMARK_COLS)}
        rows[:, idx['height_m']] = 200.0
        rows[:, idx['depth_m']] = 200.0
        rows[:, idx['sigma_assumed_px']] = 0.5
        rows[:8, idx['matcher_accepted']] = 1                       # 8 accepted, 2 rejected
        rows[:8, idx['correct']] = 1
        rows[6:8, idx['correct']] = 0                               # two of the accepted are false
        rows[8, idx['correct']] = 1                                 # a rejected one that was in fact right
        rows[:5, idx['filter_accepted']] = 1                        # the filter keeps 5, none of them false
        rows[:8, idx['err_x_px']] = np.linspace(-0.3, 0.3, 8)
        stats = iil.matcher_scores(rows)
        self.assertEqual(stats['landmarks_attempted'], 10)
        self.assertAlmostEqual(stats['matcher_accepted_fraction'], 0.8)
        self.assertAlmostEqual(stats['false_rate_matcher'], 2/8)
        self.assertAlmostEqual(stats['false_rate_after_filter'], 0.0)
        self.assertEqual(stats['rejected_by_matcher_but_peak_was_correct'], 1)
        self.assertEqual(stats['accepted_and_correct'], 6)
        self.assertEqual(len(stats['by_height']), 1)                # 200 m is the lower edge of the 200-250 bin

    def test_post_processing_separates_lost_lock_descents(self):
        def row(variant, mode, j, frame, prior, attempted, accepted, false, kept, false_kept, error, height=200.0):
            return {'variant': variant, 'mode': mode, 'descent': j, 'frame': frame, 'truth_height_m': height,
                    'attempted': attempted, 'matcher_accepted': accepted, 'false_matcher_accepted': false,
                    'filter_accepted': kept, 'false_filter_accepted': false_kept, 'prior_horizontal_error_m': prior,
                    'prior_tilt_error_deg': 0.1, 'prior_yaw_error_deg': 0.1, 'horizontal_error_m': error,
                    'filter_horizontal_sigma_m': 0.7, 'nees_pos2': 0.5, 'tilt_error_deg': 0.1, 'velocity_error_mps': 0.2}
        rows = []
        for j, lost in ((0, False), (1, True)):
            for frame in range(12):
                first = frame == 0
                rows.append(row('v', 'image', j, frame, 8.0 if first else 0.3, 10, 6 if first else 10,
                                2 if first else 0, 5 if first else 10, 1 if first else 0, 20.0 if lost else 0.4))
                rows.append(row('v', 'control', j, frame, 8.0 if first else 0.3, 10, 10, 0, 10, 0, 0.35))
        a = iil.analyse_rows(rows)
        self.assertEqual((a['image_descents'], a['descents_that_lost_lock']), (2, 1))
        self.assertAlmostEqual(a['frame0_only']['false_rate_matcher'], 4/12)
        self.assertAlmostEqual(a['frame0_only']['false_rate_after_filter'], 2/10)
        self.assertEqual(a['locked_descents_frames_10_on']['false_accepted_by_matcher'], 0)
        pair = a['image_vs_control_on_descents_that_kept_lock']['v']
        self.assertEqual(pair['descents_kept_lock'], 1)
        self.assertAlmostEqual(pair['image']['horizontal_rms_m'], 0.4)      # the lost descent is excluded
        self.assertAlmostEqual(pair['control']['horizontal_rms_m'], 0.35)
        self.assertEqual([x['lost_lock'] for x in a['per_descent']], [False, True])

    def test_noise_model_fit_recovers_known_values(self):
        rng = np.random.default_rng(4)
        depth = rng.uniform(80, 300, 20000)
        sd = np.sqrt(0.2**2 + (0.1*iil.FOCAL/depth)**2)
        a, b = iil.fit_noise_model(depth, sd*rng.normal(size=depth.size), sd*rng.normal(size=depth.size))
        self.assertAlmostEqual(a, 0.2, delta=0.02)
        self.assertAlmostEqual(b, 0.1, delta=0.01)

    def test_time_plan_cuts_lowest_priority_first(self):
        experiments = [dict(name='a', weight=1.0, minimum=2), dict(name='b', weight=1.0, minimum=1),
                       dict(name='c', weight=2.0, minimum=1)]
        counts = {'a': 4, 'b': 3, 'c': 2}
        done = {'a': 1, 'b': 0, 'c': 0}
        got = iil.plan_counts(experiments, counts, done, unit=10.0, remaining=1e6)
        self.assertEqual(got, counts)
        # Work left costs 3*14 + 3*14 + 2*24 = 132 s; with 100 s the last experiment goes first, then the one before.
        got = iil.plan_counts(experiments, counts, done, unit=10.0, remaining=100.0)
        self.assertEqual(got, {'a': 4, 'b': 2, 'c': 1})
        got = iil.plan_counts(experiments, counts, done, unit=10.0, remaining=10.0)
        self.assertEqual(got, {'a': 1, 'b': 0, 'c': 0})             # never below what has already been done


class ClosedLoopChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.truth = iil.load_truth('moderate', 1, 'sim', duration=2.2)

    def test_short_loop_reduces_initial_error(self):
        setup = iil.Setup(binning=4, max_frames=10)
        rec = iil.run_descent(self.truth, 0, setup)
        h = np.hypot(rec['err'][:, 0], rec['err'][:, 1])
        self.assertEqual(len(h), 10)
        self.assertGreater(rec['pre'][0, 0], 3.0)                   # a real first-fix error to start from
        self.assertLess(h[-1], 0.5*rec['pre'][0, 0])
        self.assertLess(h[-1], 2.0)
        self.assertGreater(rec['counts'][-1, 2], 5)                 # the filter is still being fed
        self.assertGreater(rec['rows'].shape[0], 50)
        self.assertTrue(np.all(np.isfinite(rec['err'])))
        # Every offered landmark was matched through the predicted pose, and truth never entered the filter input:
        self.assertTrue(np.all(rec['counts'][:, 1] <= rec['counts'][:, 0]))

    def test_control_modes_run_and_are_cheap(self):
        for mode in ('control', 'control_mc'):
            rec = iil.run_descent(self.truth, 0, iil.Setup(mode=mode, max_frames=10))
            h = np.hypot(rec['err'][:, 0], rec['err'][:, 1])
            self.assertLess(h[-1], 2.0, mode)
            self.assertEqual(np.nansum(rec['times'][:, :3]), 0.0)

    def test_control_reproduces_the_monte_carlo_exactly(self):
        directory = mc.truth_dir('moderate', 160)
        if not (directory/'done').exists():
            self.skipTest('the Monte Carlo truth cache is not present')
        cached = iil.load_truth('moderate', 3)
        j, frames = 2, 40
        rec = iil.run_descent(cached, j, iil.Setup(mode='control_mc', max_frames=frames))
        name = '_image_in_loop_check'
        # search_margin_m=0 is the Monte Carlo's landmark rule as it was when this
        # experiment ran (16 px margin). The Monte Carlo has since adopted the
        # search-patch rule this experiment showed it needed.
        mc.SCENARIOS[name] = mc.scenario('moderate', dict(outlier_rate=0.0, map_xy_sd=0.0, relief_sd=0.0, focal_error=0.0,
                                                          principal_error=0.0, misalignment=0.0,
                                                          search_margin_m=0.0), {})
        try:
            ref = mc.run_descent((name, j, 160))
        finally:
            del mc.SCENARIOS[name]
        np.testing.assert_allclose(rec['err'], ref['err'][:frames].astype(float), rtol=2e-5, atol=2e-5)
        np.testing.assert_allclose(rec['counts'][:, 0], ref['counts'][:frames, 0])
        np.testing.assert_allclose(rec['sig'], ref['sig'][:frames].astype(float), rtol=2e-5, atol=2e-5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
