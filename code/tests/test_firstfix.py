"""GPL-3.0-only. First-fix solver checks on synthetic scenes (no real image, no real map).

Every scene is drawn with fixed seeds: a camera pose with a uniformly random
yaw, up to 20 degrees of tilt in a random direction, 80-350 m of height and
+/-300 m of horizontal position; landmarks at the ground points that random
pixels (kept 40 px inside the frame, as a matcher's template needs) see; and
pixels projected through the Camera of test_eskf.py, which has a lever arm
and a mounting misalignment. Run directly it also prints a sensitivity table
and the aliasing behaviour that no assertion covers.
"""
import math
import sys
import time
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cansat.eskf import ATT, BA, BARO, BG, LEVEL, N_ERR, POS, TIME, VEL, Eskf, ImuNoise, exp_so3, log_so3  # noqa: E402
from cansat.firstfix import FirstFix, initial_covariance, ransac_iterations, solve_first_fix  # noqa: E402
from test_eskf import camera  # noqa: E402

DEG = math.pi/180
WIDTH, HEIGHT = 1456, 1088
EDGE = 40.0                         # landmark margin inside the frame, px


class Scene:
    """Truth pose, matches and the barometer reading of one synthetic image."""

    def __init__(self, rng, n, sigma=1.0, outliers=0.0, tilt=None, baro=0.0, map_sigma=0.0):
        cam = CAMERA
        self.p = np.array([rng.uniform(-300, 300), rng.uniform(-300, 300), rng.uniform(80, 350)])
        angle = rng.uniform(0, 20*DEG) if tilt is None else tilt
        bearing = rng.uniform(0, 2*math.pi)
        self.yaw = rng.uniform(-math.pi, math.pi)
        self.r = (exp_so3(np.array([0, 0, self.yaw])) @ exp_so3(angle*np.array([math.cos(bearing), math.sin(bearing), 0]))
                  @ LEVEL)
        pixels = np.column_stack((rng.uniform(EDGE, WIDTH - EDGE, n), rng.uniform(EDGE, HEIGHT - EDGE, n)))
        rays = cam.ray(pixels) @ self.r.T
        lever = self.r @ cam.lever
        depth = (self.p[2] + lever[2])/-rays[:, 2]
        self.points = self.p + lever + rays*depth[:, None]
        self.points[:, 2] = 0.0
        self.pixels = cam.project(self.points, self.p, self.r)[0] + rng.normal(0, sigma, (n, 2))
        self.outlier = np.zeros(n, dtype=bool)
        k = int(round(outliers*n))
        self.outlier[rng.permutation(n)[:k]] = True
        self.pixels[self.outlier] = np.column_stack((rng.uniform(0, WIDTH, k), rng.uniform(0, HEIGHT, k)))
        self.height = self.p[2]*(1 + baro*rng.choice([-1.0, 1.0]))
        # Map error enters after the pixels are made: the camera saw the real ground.
        self.map = self.points + rng.normal(0, map_sigma, self.points.shape)

    def error(self, fix):
        """[dp (m), dtheta (rad)] of truth relative to the fix, as Eskf.error_from."""
        return np.r_[self.p - fix.p, log_so3(fix.r.T @ self.r)]

    def horizontal(self, fix):
        return float(np.linalg.norm(self.p[:2] - fix.p[:2]))

    def solve(self, **kw):
        return solve_first_fix(CAMERA, self.pixels, self.map, self.height, **kw)


CAMERA = camera()


def scenes(seed, count, *args, **kw):
    rng = np.random.default_rng(seed)
    return [Scene(rng, *args, **kw) for _ in range(count)]


class Noiseless(unittest.TestCase):
    def test_pose_is_recovered_to_rounding_error_in_every_yaw_quadrant(self):
        # 10 % barometer error and a level start: the pose must not depend on either.
        quadrant = np.zeros(4, int)
        solved = np.zeros(4, int)
        worst_p = worst_a = 0.0
        iterations = []
        for s in scenes(1, 200, 12, sigma=0.0, baro=0.10):
            q = int((s.yaw + math.pi)//(math.pi/2))
            quadrant[q] += 1
            fix = s.solve()
            if not fix.ok:
                continue
            solved[q] += 1
            e = s.error(fix)
            worst_p, worst_a = max(worst_p, np.linalg.norm(e[:3])), max(worst_a, np.linalg.norm(e[3:]))
            iterations.append(fix.iterations)
        self.assertTrue(np.all(quadrant >= 30), quadrant)
        np.testing.assert_array_equal(solved, quadrant)
        self.assertLess(worst_p, 1e-6)
        self.assertLess(worst_a, 1e-8)
        self.assertLess(np.max(iterations), 30)
        print('\na. noiseless, 200 scenes: worst position %.1e m, attitude %.1e rad, quadrants %s, '
              'LM steps median %d max %d' % (worst_p, worst_a, quadrant, np.median(iterations), np.max(iterations)))


class NoisyConsistency(unittest.TestCase):
    def test_error_matches_the_returned_covariance(self):
        nees, horizontal, rms, ok = [], [], [], 0
        data = scenes(2, 300, 12, sigma=1.0, baro=0.10)
        for s in data:
            fix = s.solve(pixel_sigma=1.0)
            if not fix.ok:
                continue
            ok += 1
            e = s.error(fix)
            nees.append(e @ np.linalg.solve(fix.cov, e))
            horizontal.append(s.horizontal(fix))
            rms.append(fix.rms_px)
        mean, rms_h = float(np.mean(nees)), float(np.sqrt(np.mean(np.square(horizontal))))
        print('\nb. 1 px, 12 landmarks, 10 %% baro error, 300 scenes: success %.3f, horizontal RMS %.2f m (max %.2f), '
              'mean NEES %.2f (6 expected, +/-%.2f), mean residual %.2f px (%.2f expected)'
              % (ok/300, rms_h, max(horizontal), mean, math.sqrt(12/ok), np.mean(rms), math.sqrt(1 - 6/24)))
        self.assertGreaterEqual(ok, 297)
        self.assertLess(rms_h, 1.0)
        self.assertLess(abs(mean - 6), 4*math.sqrt(2*6/ok))
        # The fit spends 6 of the 24 residual degrees of freedom: their RMS is sqrt(1 - 6/24) of the noise.
        self.assertAlmostEqual(float(np.mean(rms)), math.sqrt(1 - 6/24), delta=0.03)

    def test_map_error_enters_the_covariance(self):
        # The proposal's map error (0.3 m horizontal, 1 m vertical, 1 sigma) with the matching
        # map_var: the covariance must grow to cover it. Without map_var it would be too small.
        sigma, nees, plain = [0.3, 0.3, 1.0], [], []
        for s in scenes(3, 250, 14, sigma=1.0, baro=0.10, map_sigma=sigma):
            fix = s.solve(pixel_sigma=1.0, map_var=np.square(sigma))
            blind = s.solve(pixel_sigma=1.0)
            if fix.ok:
                e = s.error(fix)
                nees.append(e @ np.linalg.solve(fix.cov, e))
            if blind.ok:
                e = s.error(blind)
                plain.append(e @ np.linalg.solve(blind.cov, e))
        print('\nb2. map error 0.3/0.3/1.0 m, 14 landmarks, 250 scenes: success %.3f, mean NEES %.2f with map_var, '
              '%.2f without' % (len(nees)/250, np.mean(nees), np.mean(plain)))
        self.assertGreaterEqual(len(nees), 245)
        self.assertLess(abs(np.mean(nees) - 6), 4*math.sqrt(12/len(nees)))
        self.assertGreater(np.mean(plain), 6 + 4*math.sqrt(12/len(plain)))


class Outliers(unittest.TestCase):
    def run_case(self, seed, fraction):
        solved = false = ok = tp = fp = fn = 0
        for s in scenes(seed, 300, 16, sigma=1.0, outliers=fraction, baro=0.10):
            fix = s.solve()
            if not fix.ok:
                continue
            ok += 1
            horizontal = s.horizontal(fix)
            solved += horizontal < 3.0
            false += horizontal > 10.0
            tp += int(np.sum(fix.inliers & ~s.outlier))
            fp += int(np.sum(fix.inliers & s.outlier))
            fn += int(np.sum(~fix.inliers & ~s.outlier))
        print('\nc. %d %% outliers, 16 landmarks, 300 scenes: fix returned %.3f, error < 3 m %.3f, false fixes '
              '(> 10 m) %d, inlier precision %.4f, recall %.4f' % (100*fraction, ok/300, solved/300, false,
                                                                    tp/(tp + fp), tp/(tp + fn)))
        return solved/300, false, tp/(tp + fp), tp/(tp + fn)

    def test_30_percent_gross_outliers(self):
        solved, false, precision, recall = self.run_case(4, 0.30)
        self.assertGreaterEqual(solved, 0.97)
        self.assertEqual(false, 0)
        self.assertGreater(precision, 0.995)
        self.assertGreater(recall, 0.98)

    def test_40_percent_gross_outliers(self):
        solved, false, precision, recall = self.run_case(5, 0.40)
        self.assertGreaterEqual(solved, 0.95)
        self.assertEqual(false, 0)
        self.assertGreater(precision, 0.995)
        self.assertGreater(recall, 0.98)

    def test_map_error_with_gross_outliers_never_raises(self):
        # Regression: a trial pose built on bad matches can put a landmark
        # almost in the camera's own plane, where its map-error pixel covariance
        # is singular to rounding. That used to raise from the Cholesky factor
        # (found by the Monte Carlo's first-fix start). It must be a refused
        # match, never an exception, and never a false fix.
        sigma, solved, false = [0.3, 0.3, 1.0], 0, 0
        data = scenes(9, 400, 16, sigma=1.0, outliers=0.30, baro=0.10, map_sigma=sigma)
        for s in data:
            fix = s.solve(pixel_sigma=1.0, map_var=np.square(sigma))
            if fix.ok:
                solved += s.horizontal(fix) < 3.0
                false += s.horizontal(fix) > 10.0
        print('\nc2. map error 0.3/0.3/1.0 m with 30 %% outliers, 16 landmarks, %d scenes: error < 3 m %.3f, '
              'false fixes %d' % (len(data), solved/len(data), false))
        # Map error costs fixes (0.86 solved against 1.00 without it) but never a false one.
        self.assertGreaterEqual(solved/len(data), 0.80)
        self.assertEqual(false, 0)

    def test_a_landmark_in_the_camera_plane_is_unusable_not_fatal(self):
        from cansat.firstfix import _measure
        p, r = np.array([0.0, 0.0, 100.0]), LEVEL
        points = np.array([[10.0, 5.0, 0.0], [30.0, -20.0, 100.0 - CAMERA.lever[2] + 1e-13], [-15.0, 8.0, 0.0]])
        pixels = np.full((3, 2), 500.0)
        with np.errstate(all='ignore'):
            rw, hw, depth, res, area, chol = _measure(CAMERA, pixels, points, p, r, np.ones(3),
                                                      np.tile([0.09, 0.09, 1.0], (3, 1)), 100.0)
        # No exception, and the ordinary matches are untouched by their neighbour.
        self.assertTrue(np.all(np.isfinite(rw[[0, 2]])) and np.all(np.isfinite(hw)) and np.all(np.isfinite(chol)))
        clean = _measure(CAMERA, pixels[[0, 2]], points[[0, 2]], p, r, np.ones(2), np.tile([0.09, 0.09, 1.0], (2, 1)), 100.0)
        np.testing.assert_allclose(rw[[0, 2]], clean[0], rtol=1e-12)


class Aliasing(unittest.TestCase):
    SHIFT = 16.0

    def run_case(self, seed, shifted, count):
        """Each fix is classed by where it lands: on the truth, on the shifted copy, or elsewhere."""
        rng = np.random.default_rng(seed)
        tally = {'refused': 0, 'truth': 0, 'shifted': 0, 'elsewhere': 0}
        absorbed = 0
        for _ in range(count):
            s = Scene(rng, 16, sigma=1.0, baro=0.10)
            bearing = rng.uniform(0, 2*math.pi)
            offset = self.SHIFT*np.array([math.cos(bearing), math.sin(bearing), 0.0])
            index = rng.permutation(16)[:shifted]
            s.map[index] += offset                # the matcher found the same texture one tramline over
            fix = s.solve()
            if not fix.ok:
                tally['refused'] += 1
                continue
            away = fix.p[:2] - s.p[:2]
            if np.linalg.norm(away) < 5.0:
                tally['truth'] += 1
            elif np.linalg.norm(away - offset[:2]) < 5.0:
                tally['shifted'] += 1
            else:
                tally['elsewhere'] += 1
            absorbed += int(fix.inliers[index].sum() > 0) if np.linalg.norm(away) < 5.0 else 0
        return tally, absorbed

    def test_minority_alias_is_outvoted(self):
        tally, absorbed = self.run_case(6, 5, 300)
        print('\nd. 5 of 16 matches shifted %.0f m, 300 scenes: %s, true fixes that kept a shifted match: %d'
              % (self.SHIFT, tally, absorbed))
        returned = 300 - tally['refused']
        self.assertGreaterEqual(returned, 285)
        self.assertGreaterEqual(tally['truth'], 0.98*returned)
        self.assertEqual(tally['shifted'], 0)
        self.assertEqual(absorbed, 0)

    def test_majority_alias_is_followed_and_a_tie_is_not_resolved(self):
        # Nothing in one image can tell which cluster is the true ground, so the
        # larger one wins and an even split is mostly refused. Documented, not wished away.
        majority, _ = self.run_case(7, 11, 200)
        tie, _ = self.run_case(8, 8, 200)
        print('\nd. 11 of 16 shifted (majority), 200 scenes: %s\nd. 8 of 16 shifted (tie), 200 scenes:       %s'
              % (majority, tie))
        returned = 200 - majority['refused']
        self.assertGreater(returned, 160)
        self.assertGreater(majority['shifted'], 0.95*returned)
        self.assertLess(majority['truth'], 0.02*returned)
        self.assertGreater(tie['refused'], 0.5*200)


class Degenerate(unittest.TestCase):
    def assert_refused(self, fix, reason):
        self.assertIsInstance(fix, FirstFix)
        self.assertFalse(fix.ok)
        self.assertIn(reason, fix.reason)
        self.assertIsNone(fix.p)
        self.assertIsNone(fix.r)
        self.assertIsNone(fix.cov)
        self.assertFalse(fix.inliers.any())

    def test_fewer_than_four_matches(self):
        s = Scene(np.random.default_rng(9), 12, sigma=0.0)
        for n in range(4):
            fix = solve_first_fix(CAMERA, s.pixels[:n], s.points[:n], s.height)
            self.assert_refused(fix, 'too few')
            self.assertEqual(len(fix.inliers), n)
        # Missing coordinates do not count as matches either.
        pixels = s.pixels.copy()
        pixels[:9] = np.nan
        self.assert_refused(solve_first_fix(CAMERA, pixels, s.points, s.height), 'too few')

    def test_a_missing_coordinate_is_never_an_inlier(self):
        s = Scene(np.random.default_rng(10), 12, sigma=0.5)
        s.pixels[2] = np.nan
        s.map[7, 0] = np.inf
        fix = s.solve(pixel_sigma=0.5)
        self.assertTrue(fix.ok, fix.reason)
        self.assertFalse(fix.inliers[[2, 7]].any())
        self.assertEqual(int(fix.inliers.sum()), 10)

    def test_collinear_landmarks(self):
        rng = np.random.default_rng(11)
        refused = 0
        for _ in range(100):
            s = Scene(rng, 12, sigma=1.0)
            bearing, span = rng.uniform(0, math.pi), rng.uniform(-0.5, 0.5, 10)*s.p[2]
            line = np.column_stack((s.p[0] + span*math.cos(bearing), s.p[1] + span*math.sin(bearing), np.zeros(10)))
            pixels, depth = CAMERA.project(line, s.p, s.r)
            if np.any(depth <= 0) or np.any(abs(pixels - [728, 544]) > [700, 520]):
                continue
            fix = solve_first_fix(CAMERA, pixels + rng.normal(0, 1.0, pixels.shape), line, s.height)
            self.assert_refused(fix, 'degenerate')
            refused += 1
        self.assertGreater(refused, 40)

    def test_all_outliers(self):
        # Pixels unrelated to the map: a pose must never come out, whatever n.
        accepted, count = 0, 0
        for n, seed in ((6, 12), (12, 13), (16, 14), (30, 15)):
            for s in scenes(seed, 100, n, outliers=1.0):
                fix = s.solve()
                accepted += fix.ok
                count += 1
                if not fix.ok:
                    self.assertIsNone(fix.p)
        print('\ne. all-outlier matches (n = 6, 12, 16, 30), %d scenes: %d accepted' % (count, accepted))
        self.assertEqual(accepted, 0)

    def test_unusable_arguments_raise(self):
        s = Scene(np.random.default_rng(16), 8)
        with self.assertRaises(ValueError):
            solve_first_fix(CAMERA, s.pixels, s.points[:5], s.height)
        with self.assertRaises(ValueError):
            solve_first_fix(CAMERA, s.pixels, s.points, 0.0)
        with self.assertRaises(ValueError):
            solve_first_fix(CAMERA, s.pixels, s.points, s.height, pixel_sigma=0.0)


class Search(unittest.TestCase):
    def test_iteration_count_follows_the_ransac_formula(self):
        self.assertEqual(ransac_iterations(0.5, 0.999), 25)         # Appendix G.4: "about 25"
        self.assertGreater(ransac_iterations(0.3), ransac_iterations(0.5))
        self.assertEqual(ransac_iterations(0.9, 0.99), 3)

    def test_same_matches_give_the_same_answer_and_any_seed_gives_the_same_pose(self):
        s = scenes(17, 1, 16, sigma=1.0, outliers=0.4, baro=0.10)[0]
        a, b = s.solve(), s.solve()
        np.testing.assert_array_equal(a.p, b.p)
        self.assertEqual(a.hypotheses, b.hypotheses)
        c = s.solve(rng=np.random.default_rng(99))
        np.testing.assert_allclose(c.p, a.p, atol=1e-6)
        self.assertGreaterEqual(a.hypotheses, 25)

    def test_bookkeeping_fields(self):
        s = scenes(18, 1, 12, sigma=1.0)[0]
        fix = s.solve()
        self.assertTrue(fix.ok)
        self.assertEqual(fix.reason, 'ok')
        self.assertEqual(fix.cov.shape, (6, 6))
        np.testing.assert_array_equal(fix.cov, fix.cov.T)
        self.assertTrue(np.all(np.linalg.eigvalsh(fix.cov) > 0))
        self.assertEqual(fix.inliers.shape, (12,))
        self.assertGreaterEqual(fix.tries, 1)
        self.assertGreater(fix.iterations, 0)
        np.testing.assert_allclose(fix.r @ fix.r.T, np.eye(3), atol=1e-12)


class CovarianceHelper(unittest.TestCase):
    def test_blocks_shape_and_definiteness(self):
        s = scenes(19, 1, 14, sigma=1.0, baro=0.10)[0]
        fix = s.solve()
        self.assertTrue(fix.ok)
        cov = initial_covariance(fix, vel_sigma=[2.0, 3.0, 5.0], accel_bias_sigma=0.05, gyro_bias_sigma=0.003,
                                 baro_bias_sigma=1.5)
        self.assertEqual(cov.shape, (N_ERR, N_ERR))
        np.testing.assert_array_equal(cov, cov.T)
        self.assertGreater(np.linalg.eigvalsh(cov)[0], 0.0)
        np.testing.assert_array_equal(cov[POS, POS], fix.cov[:3, :3])
        np.testing.assert_array_equal(cov[ATT, ATT], fix.cov[3:, 3:])
        np.testing.assert_array_equal(cov[POS, ATT], fix.cov[:3, 3:])
        np.testing.assert_array_equal(cov[ATT, POS], fix.cov[3:, :3])
        self.assertGreater(np.max(abs(cov[POS, ATT])), 0.0)           # tilt and translation are correlated
        np.testing.assert_array_equal(np.diag(cov)[VEL], [4.0, 9.0, 25.0])
        np.testing.assert_allclose(np.diag(cov)[BA], 0.05**2)
        np.testing.assert_allclose(np.diag(cov)[BG], 0.003**2)
        self.assertAlmostEqual(cov[BARO, BARO], 1.5**2)
        self.assertAlmostEqual(cov[TIME, TIME], 0.002**2)              # default camera time offset
        # The same blocks by literal index, so a wrong constant in eskf.py cannot hide here.
        np.testing.assert_array_equal(cov[0:3, 0:3], fix.cov[:3, :3])
        np.testing.assert_array_equal(cov[6:9, 6:9], fix.cov[3:, 3:])
        np.testing.assert_array_equal(cov[0:3, 6:9], fix.cov[:3, 3:])
        # Everything else is zero: one image says nothing joint about the sensor states.
        expected = np.zeros((N_ERR, N_ERR))
        expected[0:3, 0:3], expected[6:9, 6:9] = fix.cov[:3, :3], fix.cov[3:, 3:]
        expected[0:3, 6:9], expected[6:9, 0:3] = fix.cov[:3, 3:], fix.cov[3:, :3]
        expected[3:6, 3:6], expected[9:12, 9:12] = np.diag([4.0, 9.0, 25.0]), np.eye(3)*0.05**2
        expected[12:15, 12:15], expected[15, 15], expected[16, 16] = np.eye(3)*0.003**2, 1.5**2, 0.002**2
        np.testing.assert_array_equal(cov, expected)
        self.assertAlmostEqual(initial_covariance(fix, 1, 1, 1, 1, 0.005)[TIME, TIME], 0.005**2)

    def test_a_failed_fix_cannot_start_the_filter(self):
        s = scenes(20, 1, 12, outliers=1.0)[0]
        fix = s.solve()
        self.assertFalse(fix.ok)
        with self.assertRaises(ValueError):
            initial_covariance(fix, 1.0, 0.1, 0.01, 1.0)

    def test_filter_started_from_a_fix_accepts_the_same_landmarks(self):
        # End to end: pose and covariance go straight into the navigation filter.
        s = scenes(21, 1, 14, sigma=1.0, baro=0.10)[0]
        fix = s.solve()
        cov = initial_covariance(fix, 1.0, 0.05, 0.003, 1.0)
        f = Eskf(fix.p, [0, 0, -10.0], fix.r, cov, ImuNoise())
        out = f.update_landmarks(CAMERA, s.points, s.pixels, 1.0)
        self.assertEqual(int(out['accepted'].sum()), 14)
        self.assertLess(np.median(out['nis']), 6.0)


# --- Sensitivity table (not asserted) ------------------------------------------

def sweep_cell(seed, count, n, tilt, baro, sigma, outliers):
    rng = np.random.default_rng(seed)
    ok, horizontal, ms, false = 0, [], [], 0
    for _ in range(count):
        s = Scene(rng, n, sigma=sigma, outliers=outliers, tilt=tilt*DEG, baro=baro)
        t0 = time.perf_counter()
        fix = s.solve(pixel_sigma=sigma)
        ms.append((time.perf_counter() - t0)*1e3)
        if fix.ok:
            ok += 1
            horizontal.append(s.horizontal(fix))
    false = sum(h > 10 for h in horizontal)
    rms = math.sqrt(np.mean(np.square(horizontal))) if horizontal else math.nan
    return ok/count, rms, false, ms


def print_sweep(outliers, count=100):
    base = dict(n=12, tilt=10, baro=0.10, sigma=1.0)
    axes = (('landmarks', 'n', (4, 6, 8, 12, 20)), ('tilt deg', 'tilt', (0, 10, 20, 30)),
            ('baro error', 'baro', (0.0, 0.1, 0.2, 0.3)), ('pixel noise', 'sigma', (0.5, 1.0, 2.0, 4.0)))
    print('\ng. SYNTHETIC sensitivity, one factor at a time around %s, %.0f %% outliers, %d scenes per cell'
          % (', '.join('%s=%g' % kv for kv in base.items()), 100*outliers, count))
    print('%-12s %8s %9s %12s %6s %10s' % ('factor', 'value', 'success', 'horiz RMS m', 'false', 'median ms'))
    every = []
    for label, key, values in axes:
        for v in values:
            cell = dict(base, **{key: v})
            success, rms, false, ms = sweep_cell(41, count, cell['n'], cell['tilt'], cell['baro'], cell['sigma'],
                                                 outliers)
            every += ms
            print('%-12s %8g %9.3f %12.2f %6d %10.1f' % (label, v, success, rms, false, np.median(ms)))
    print('median solve time over the table: %.1f ms (single process, shared PC)' % np.median(every))


if __name__ == '__main__':
    loaded = unittest.main(exit=False, verbosity=1)
    print_sweep(0.0)
    print_sweep(0.3)
    sys.exit(0 if loaded.result.wasSuccessful() else 1)
