"""GPL-3.0-only. Navigation filter checks against independent numerics (synthetic)."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cansat.eskf import (ATT, BA, BARO, BG, GATE_2DOF, GRAVITY, LEVEL, N_ERR, POS, TIME, VEL,  # noqa: E402
                         Camera, Eskf, ImuNoise, exp_so3, log_so3)

SILENT = ImuNoise(0.0, 0.0, 0.0, 0.0, 0.0)
DEG = math.pi/180


def camera():
    return Camera(724.6, 724.6, 728.0, 544.0, 1456, 1088,
                  r_cb=exp_so3(np.array([0.02, -0.03, 0.05])), lever=np.array([0.004, -0.002, 0.045]))


def attitude(rng, tilt=0.2):
    return LEVEL @ exp_so3(rng.normal(size=3)*np.array([tilt, tilt, 2.0]))


def ground_points(rng, n, centre, half):
    return np.column_stack((centre[0] + rng.uniform(-half, half, n),
                            centre[1] + rng.uniform(-half, half, n), rng.normal(0, 1.0, n)))


def perturbed(f, dx):
    g = Eskf(f.p + dx[POS], f.v + dx[VEL], f.r @ exp_so3(dx[ATT]), f.cov, f.noise,
             f.ba + dx[BA], f.bg + dx[BG], f.baro_bias + dx[BARO], f.time_offset + dx[TIME])
    g.rate = f.rate.copy()
    return g


def observe(cam, truth, points):
    """Pixels of an image stamped now but exposed truth.time_offset later."""
    return cam.project(points, truth.p + truth.v*truth.time_offset,
                       truth.r @ exp_so3(truth.rate*truth.time_offset))[0]


def error(f, truth):
    return f.error_from(truth.p, truth.v, truth.r, truth.ba, truth.bg, truth.baro_bias, truth.time_offset)


class Rotations(unittest.TestCase):
    def test_exp_log_round_trip_and_orthogonality(self):
        rng = np.random.default_rng(1)
        for _ in range(50):
            v = rng.normal(size=3)*rng.uniform(0, 1.5)
            r = exp_so3(v)
            np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-13)
            self.assertAlmostEqual(np.linalg.det(r), 1.0, places=12)
            np.testing.assert_allclose(log_so3(r), v, atol=1e-11)
        np.testing.assert_allclose(exp_so3(np.zeros(3)), np.eye(3))
        np.testing.assert_allclose(exp_so3(np.array([0, 0, math.pi/2])) @ [1, 0, 0], [0, 1, 0], atol=1e-15)


class CameraModel(unittest.TestCase):
    def test_level_camera_axes(self):
        # Level can: +x pixel is east, +y pixel is south, boresight is down.
        cam = Camera(700.0, 700.0, 728.0, 544.0)
        pixels, depth = cam.project(np.array([[10.0, 0, 0], [0, 10.0, 0]]), np.array([0, 0, 100.0]), LEVEL)
        np.testing.assert_allclose(pixels, [[798.0, 544.0], [728.0, 474.0]])
        np.testing.assert_allclose(depth, 100.0)

    def test_jacobians_match_finite_differences(self):
        rng = np.random.default_rng(2)
        cam = camera()
        worst = [0.0, 0.0]
        for _ in range(20):
            p = np.array([rng.normal(0, 30), rng.normal(0, 30), rng.uniform(80, 300)])
            r = attitude(rng)
            points = ground_points(rng, 6, p, 0.6*p[2])
            _, _, jp, jt = cam.jacobians(points, p, r)
            eps = 1e-5
            for axis in range(3):
                e = np.eye(3)[axis]*eps
                fd_p = (cam.project(points, p + e, r)[0] - cam.project(points, p - e, r)[0])/(2*eps)
                fd_t = (cam.project(points, p, r @ exp_so3(e))[0]
                        - cam.project(points, p, r @ exp_so3(-e))[0])/(2*eps)
                worst[0] = max(worst[0], np.max(abs(fd_p - jp[:, :, axis])))
                worst[1] = max(worst[1], np.max(abs(fd_t - jt[:, :, axis])))
        self.assertLess(worst[0], 1e-6)
        self.assertLess(worst[1], 1e-4)

    def test_ray_inverts_projection(self):
        rng = np.random.default_rng(3)
        cam = camera()
        p, r = np.array([5.0, -8.0, 200.0]), attitude(rng)
        points = ground_points(rng, 8, p, 100.0)
        pixels, depth = cam.project(points, p, r)
        rays = cam.ray(pixels)
        rebuilt = p + (cam.lever + rays*depth[:, None]) @ r.T
        np.testing.assert_allclose(rebuilt, points, atol=1e-9)


class Propagation(unittest.TestCase):
    def test_nominal_against_fine_integration(self):
        # Constant body rate and body specific force; reference integrates the
        # kinematics with 1000 times smaller steps.
        w = np.array([0.6, -0.4, 5.0])
        a = np.array([0.8, -0.5, -9.6])
        r0 = LEVEL @ exp_so3(np.array([0.2, -0.1, 0.7]))
        f = Eskf([1.0, 2.0, 300.0], [4.0, -3.0, -10.5], r0, np.eye(N_ERR), SILENT)
        dt, steps, sub = 0.01, 300, 1000
        p, v, r = f.p.copy(), f.v.copy(), f.r.copy()
        h = dt/sub
        dr_half, dr = exp_so3(w*h/2), exp_so3(w*h)
        for _ in range(steps):
            # Sculling-compensated mean specific force, in mid-interval axes.
            r_start, v_start = r.copy(), v.copy()
            for _ in range(sub):
                acc = r @ dr_half @ a + GRAVITY
                p = p + v*h + acc*h*h/2
                v = v + acc*h
                r = r @ dr
            r_mid = r_start @ exp_so3(w*dt/2)
            f.propagate(dt, r_mid.T @ (v - v_start - GRAVITY*dt)/dt, w)
        np.testing.assert_allclose(f.v, v, atol=1e-9)
        np.testing.assert_allclose(f.r, r, atol=1e-9)
        np.testing.assert_allclose(f.rate, w)
        # Position is second order in dt: 3 s of 5 rad/s spin, sub-millimetre.
        self.assertLess(np.linalg.norm(f.p - p), 1e-3)

    def test_error_transition_matches_perturbed_propagation(self):
        rng = np.random.default_rng(4)
        scale = np.r_[np.full(3, 1e-3), np.full(3, 1e-3), np.full(3, 1e-5), np.full(3, 1e-4),
                      np.full(3, 1e-6), 1e-3, 1e-4]
        for _ in range(10):
            dx = rng.normal(size=N_ERR)*scale
            r0 = attitude(rng)
            nominal = Eskf([0, 0, 250.0], [3.0, 1.0, -10.0], r0, np.outer(dx, dx), SILENT,
                           ba=[0.05, -0.02, 0.03], bg=[0.002, -0.001, 0.003])
            truth = perturbed(nominal, dx)
            for k in range(200):
                # The same measurements feed both; they differ only in state.
                accel = np.array([0.5*math.sin(0.05*k), 0.3, -9.7 + 0.4*math.cos(0.03*k)])
                gyro = np.array([0.5*math.cos(0.04*k), -0.3, 3.0])
                nominal.propagate(0.01, accel, gyro)
                truth.propagate(0.01, accel, gyro)
            e = error(nominal, truth)
            # A rank-one covariance propagates as (Phi dx)(Phi dx)^T. Every
            # path is second order in dt, so 10 ms steps agree to 1e-4.
            np.testing.assert_allclose(abs(e), np.sqrt(np.diag(nominal.cov)), rtol=3e-4, atol=1e-9)
            np.testing.assert_allclose(np.outer(e, e), nominal.cov, rtol=0, atol=6e-4*np.max(abs(nominal.cov)))

    def test_process_noise_growth(self):
        noise = ImuNoise(accel=0.01, gyro=0.002, accel_bias=1e-3, gyro_bias=1e-4, baro_bias=0.05,
                         time_offset=1e-4)
        f = Eskf([0, 0, 100.0], [0, 0, 0], LEVEL, np.zeros((N_ERR, N_ERR)), noise)
        for _ in range(100):
            f.propagate(0.01, -LEVEL.T @ GRAVITY, np.zeros(3))
        np.testing.assert_allclose(np.diag(f.cov)[BG], 1e-8, rtol=1e-9)
        np.testing.assert_allclose(f.cov[BARO, BARO], 0.05**2, rtol=1e-9)
        np.testing.assert_allclose(f.cov[TIME, TIME], 1e-8, rtol=1e-9)
        # Attitude variance: white rate noise plus integrated bias walk.
        np.testing.assert_allclose(np.diag(f.cov)[ATT], 0.002**2 + 1e-8/3, rtol=0.03)
        np.testing.assert_allclose(f.p, [0, 0, 100.0], atol=1e-12)

    def test_scale_factor_noise_grows_with_rate(self):
        # 1 % scale error at 2 rad/s acting coherently for 0.2 s per image
        # interval: (0.01*2)^2 * 0.2 rad^2/s of attitude variance.
        noise = ImuNoise(0.0, 0.0, 0.0, 0.0, 0.0, gyro_scale=0.01, scale_tau=0.2)
        for rate, expected in ((0.0, 0.0), (2.0, (0.01*2.0)**2*0.2)):
            f = Eskf([0, 0, 100.0], [0, 0, 0], LEVEL, np.zeros((N_ERR, N_ERR)), noise)
            for _ in range(100):
                f.propagate(0.01, -LEVEL.T @ GRAVITY, np.array([0.0, 0.0, rate]))
            np.testing.assert_allclose(np.diag(f.cov)[ATT], expected, rtol=1e-9, atol=1e-18)


class Updates(unittest.TestCase):
    def setUp(self):
        self.cam = camera()
        self.sd = np.r_[np.full(3, 3.0), np.full(3, 0.5), np.full(3, 0.02), np.full(3, 0.05),
                        np.full(3, 0.003), 1.0, 0.002]
        self.rate = np.array([0.5, -0.4, 1.5])

    def start(self, p, r, scale=1.0):
        f = Eskf(p, [2.0, 1.0, -10.5], r, np.diag((self.sd*scale)**2), SILENT)
        f.rate = self.rate.copy()
        return f

    def test_landmark_model_jacobian_including_time_offset(self):
        rng = np.random.default_rng(11)
        p, r = np.array([4.0, -6.0, 220.0]), attitude(rng, 0.15)
        f = self.start(p, r)
        f.time_offset = 0.004
        points = ground_points(rng, 7, p, 120.0)
        _, _, h, _ = f.landmark_model(self.cam, points, 0.1)
        for column, eps in [(i, 1e-5) for i in range(9)] + [(TIME, 1e-6)]:
            dx = np.zeros(N_ERR)
            dx[column] = eps
            plus = observe(self.cam, perturbed(f, dx), points)
            minus = observe(self.cam, perturbed(f, -dx), points)
            np.testing.assert_allclose(h[:, :, column], (plus - minus)/(2*eps), rtol=2e-5, atol=2e-4)
        np.testing.assert_array_equal(h[:, :, BA], 0.0)
        # 4 ms at 1.6 rad/s and 11 m/s moves the image by pixels, not noise.
        self.assertGreater(np.max(abs(h[:, :, TIME]))*0.004, 2.0)

    def test_landmark_update_is_consistent(self):
        # Truth is drawn from the prior, so the mean posterior NEES should be 17.
        rng = np.random.default_rng(5)
        nees, before, after = [], [], []
        for _ in range(400):
            p = np.array([0.0, 0.0, rng.uniform(100, 300)])
            f = self.start(p, attitude(rng, 0.15), 0.1)         # 0.3 m, 0.1 degree: the linear regime
            dx = rng.normal(size=N_ERR)*self.sd*0.1
            truth = perturbed(f, dx)
            points = ground_points(rng, 20, p, 0.5*p[2])
            pixels = observe(self.cam, truth, points) + rng.normal(0, 0.4, (20, 2))
            before.append(dx[POS][:2] @ dx[POS][:2])
            f.update_landmarks(self.cam, points, pixels, 0.4**2)
            f.update_baro(truth.p[2] + truth.baro_bias + rng.normal(0, 0.3), 0.3)
            e = error(f, truth)
            nees.append(e @ np.linalg.solve(f.cov, e))
            after.append(e[POS][:2] @ e[POS][:2])
        self.assertLess(abs(np.mean(nees) - N_ERR), 4*math.sqrt(2*N_ERR/400))
        # One frame only partly separates tilt from translation (Appendix C),
        # so the horizontal gain over a 0.3 m prior is real but modest.
        self.assertLess(np.mean(after), 0.5*np.mean(before))

    def test_time_offset_is_estimated_from_a_swinging_camera(self):
        # 6 ms of unknown delay, 2 ms prior. The body rate changes direction
        # from frame to frame, which is what makes the offset observable.
        rng = np.random.default_rng(12)
        errors, sigmas = [], []
        for _ in range(40):
            p, r = np.array([0.0, 0.0, 250.0]), attitude(rng, 0.1)
            sd = np.r_[np.full(3, 0.5), np.full(3, 0.2), np.full(3, 0.005), np.full(3, 1e-4),
                       np.full(3, 1e-6), 1e-3, 0.003]
            f = Eskf(p, [2.0, 1.0, -10.5], r, np.diag(sd**2), SILENT)
            truth = perturbed(f, rng.normal(size=N_ERR)*sd)
            for k in range(12):
                rate = np.array([math.cos(0.9*k), math.sin(0.9*k), 0.6])*1.2
                f.rate, truth.rate = rate.copy(), rate.copy()
                points = ground_points(rng, 20, f.p, 0.5*f.p[2])
                pixels = observe(self.cam, truth, points) + rng.normal(0, 0.3, (20, 2))
                f.update_landmarks(self.cam, points, pixels, 0.3**2, iterations=3)
            errors.append(truth.time_offset - f.time_offset)
            sigmas.append(math.sqrt(f.cov[TIME, TIME]))
        self.assertLess(np.mean(sigmas), 3e-4)                       # from 3 ms to under 0.3 ms
        ratio = np.mean((np.array(errors)/np.array(sigmas))**2)
        self.assertTrue(0.4 < ratio < 2.0, ratio)

    def test_gate_rejects_gross_outlier_and_reports_it(self):
        rng = np.random.default_rng(6)
        p, r = np.array([0.0, 0.0, 200.0]), attitude(rng, 0.1)
        f = Eskf(p, [0, 0, -10.5], r, np.diag(self.sd**2)*0.01, SILENT)
        points = ground_points(rng, 12, p, 100.0)
        pixels = self.cam.project(points, p, r)[0] + rng.normal(0, 0.3, (12, 2))
        pixels[3] += [60.0, -45.0]
        out = f.update_landmarks(self.cam, points, pixels, 0.3**2)
        self.assertFalse(out['accepted'][3])
        self.assertEqual(int(out['accepted'].sum()), 11)
        self.assertGreater(out['nis'][3], GATE_2DOF)
        self.assertFalse(out['recovered'])
        self.assertLess(np.linalg.norm(f.p - p), 0.5)

    def test_all_rejected_leaves_state_untouched(self):
        p = np.array([0.0, 0.0, 200.0])
        f = Eskf(p, [0, 0, -10.5], LEVEL, np.diag(self.sd**2)*1e-4, SILENT)
        points = np.array([[10.0, 5.0, 0.0], [-20.0, 8.0, 0.0]])
        pixels = self.cam.project(points, p, LEVEL)[0] + 200.0
        cov = f.cov.copy()
        out = f.update_landmarks(self.cam, points, pixels, 0.3**2, iterations=3, robust=True, recover=True)
        self.assertFalse(out['accepted'].any())
        np.testing.assert_array_equal(f.cov, cov)
        np.testing.assert_array_equal(f.p, p)

    def test_map_error_inflates_noise_through_the_jacobian(self):
        p = np.array([0.0, 0.0, 100.0])
        f = Eskf(p, [0, 0, -10.5], LEVEL, np.eye(N_ERR), SILENT)
        cam = Camera(700.0, 700.0, 728.0, 544.0)
        points = np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
        _, _, _, noise = f.landmark_model(cam, points, 0.0, [0.0, 0.0, 4.0])
        # Nadir: height error does not move the pixel. 45 degrees off axis:
        # 2 m of relief moves it 2 m * f / h = 14 px along x only.
        np.testing.assert_allclose(noise[0], 0.0, atol=1e-12)
        np.testing.assert_allclose(noise[1], [[14.0**2, 0.0], [0.0, 0.0]], atol=1e-9)
        _, _, _, noise = f.landmark_model(cam, points, 0.0, [0.25, 0.25, 0.0])
        np.testing.assert_allclose(noise[0], np.eye(2)*3.5**2, atol=1e-9)

    def wide_start(self, rng, p, r):
        dx = np.zeros(N_ERR)
        dx[POS] = rng.normal(0, [25.0, 25.0, 5.0])
        dx[ATT] = rng.normal(0, [0.12, 0.12, 0.12])
        cov = np.diag(np.r_[25.0, 25.0, 5.0, np.ones(3), np.full(3, 0.12), np.full(3, 0.05),
                            np.full(3, 0.003), 1.0, 1e-4]**2)
        return Eskf(p + dx[POS], [0, 0, -10.5], r @ exp_so3(dx[ATT]), cov, SILENT)

    def test_iterated_update_handles_a_large_initial_error(self):
        errors = {1: [], 4: []}
        for iterations in errors:
            rng = np.random.default_rng(7)
            for _ in range(60):
                p, r = np.array([0.0, 0.0, 300.0]), attitude(rng, 0.15)
                points = ground_points(rng, 25, p, 200.0)
                pixels = self.cam.project(points, p, r)[0] + rng.normal(0, 0.4, (25, 2))
                f = self.wide_start(rng, p, r)
                f.update_landmarks(self.cam, points, pixels, 0.4**2, gate=math.inf, iterations=iterations)
                errors[iterations].append(np.linalg.norm(f.p - p))
        rms = {k: np.sqrt(np.mean(np.square(v))) for k, v in errors.items()}
        self.assertLess(rms[4], 0.5)
        self.assertGreater(rms[1], 3*rms[4])

    def test_post_fit_rejection_removes_mismatches_the_wide_gate_let_through(self):
        # Straight after a first fix the prior is tens of metres wide, so a
        # 12 m mismatch passes the innovation gate. Paired draws.
        results = {}
        for robust in (False, True):
            rng = np.random.default_rng(8)
            errors, kept_bad, kept_good = [], 0, 0
            for _ in range(60):
                p, r = np.array([0.0, 0.0, 300.0]), attitude(rng, 0.15)
                points = ground_points(rng, 25, p, 200.0)
                pixels = self.cam.project(points, p, r)[0] + rng.normal(0, 0.4, (25, 2))
                shift = rng.uniform(8.0, 14.0, (4, 2))*rng.choice([-1.0, 1.0], (4, 2))
                pixels[:4] += shift*724.6/300.0
                f = self.wide_start(rng, p, r)
                out = f.update_landmarks(self.cam, points, pixels, 0.4**2, iterations=4, robust=robust)
                errors.append(np.linalg.norm(f.p - p))
                kept_bad += int(out['accepted'][:4].sum())
                kept_good += int(out['accepted'][4:].sum())
            results[robust] = (np.sqrt(np.mean(np.square(errors))), kept_bad, kept_good)
        self.assertGreater(results[False][1], 150)            # most mismatches accepted by the gate alone
        self.assertGreater(results[False][0], 1.0)
        self.assertEqual(results[True][1], 0)
        self.assertGreater(results[True][2], 0.97*60*21)
        self.assertLess(results[True][0], 0.5)

    def test_recovery_from_an_overconfident_wrong_prior(self):
        # The filter believes it is right to 0.1 m and is 6 m and 1 degree out.
        # Without recovery every good landmark is rejected and nothing moves.
        outcome = {}
        for recover in (False, True):
            rng = np.random.default_rng(9)
            p, r = np.array([0.0, 0.0, 250.0]), attitude(rng, 0.1)
            points = ground_points(rng, 24, p, 150.0)
            pixels = self.cam.project(points, p, r)[0] + rng.normal(0, 0.4, (24, 2))
            sd = np.r_[np.full(3, 0.1), np.full(3, 0.1), np.full(3, 0.001), np.full(3, 0.05),
                       np.full(3, 0.003), 1.0, 1e-4]
            f = Eskf(p + [4.5, -4.0, 0.5], [0, 0, -10.5], r @ exp_so3(np.array([0.012, -0.012, 0.01])),
                     np.diag(sd**2), SILENT)
            # The prior is only doubted after three starved images in a row.
            for frame in range(Eskf.RECOVER_FRAMES):
                out = f.update_landmarks(self.cam, points, pixels, 0.4**2, iterations=4, robust=True, recover=recover)
                if frame < Eskf.RECOVER_FRAMES - 1:
                    self.assertFalse(out['recovered'])
                    self.assertGreater(np.linalg.norm(f.p - p), 5.0)
            outcome[recover] = (out, np.linalg.norm(f.p - p), math.sqrt(f.cov[0, 0]))
        self.assertFalse(outcome[False][0]['recovered'])
        self.assertLess(outcome[False][0]['accepted'].sum(), 3)
        self.assertGreater(outcome[False][1], 5.0)
        self.assertTrue(outcome[True][0]['recovered'])
        self.assertGreater(outcome[True][0]['accepted'].sum(), 20)
        self.assertLess(outcome[True][1], 0.6)
        self.assertGreater(outcome[True][2], 0.1)             # and it no longer claims 0.1 m

    def test_recovery_is_refused_when_the_image_is_mostly_mismatches(self):
        # A correct, confident prior and three images in which 70 % of the
        # matches are wrong in unrelated ways. Widening the prior must not let
        # them in: with no majority that agrees, the filter is left alone.
        rng = np.random.default_rng(10)
        p, r = np.array([0.0, 0.0, 250.0]), attitude(rng, 0.1)
        sd = np.r_[np.full(3, 0.3), np.full(3, 0.1), np.full(3, 0.003), np.full(3, 0.05),
                   np.full(3, 0.003), 1.0, 1e-4]
        f = Eskf(p, [0, 0, -10.5], r, np.diag(sd**2), SILENT)
        cov = f.cov.copy()
        for _ in range(Eskf.RECOVER_FRAMES + 2):
            points = ground_points(rng, 20, p, 150.0)
            pixels = self.cam.project(points, p, r)[0] + rng.normal(0, 0.4, (20, 2))
            pixels[:14] += rng.uniform(-16, 16, (14, 2))*724.6/250.0
            out = f.update_landmarks(self.cam, points, pixels, 0.4**2, iterations=3, robust=True, recover=True)
            self.assertFalse(out['recovered'])
            self.assertLess(np.linalg.norm(f.p - p), 1.0)
        self.assertLess(f.cov[0, 0], 4*cov[0, 0])

    def test_iterated_update_does_not_reset_its_covariance_twice(self):
        # An 8 degree yaw correction. The covariance of an iterated update is
        # linearised at the corrected attitude; compare with the information
        # form built there by finite differences of the measurement.
        rng = np.random.default_rng(13)
        p, r = np.array([0.0, 0.0, 250.0]), attitude(rng, 0.1)
        sd = np.r_[np.full(3, 8.0), np.full(3, 0.5), np.radians([5.0, 5.0, 8.0]), np.full(3, 0.05),
                   np.full(3, 0.003), 1.0, 1e-6]
        prior = np.diag(sd**2)
        points = ground_points(rng, 20, p, 150.0)
        pixels = self.cam.project(points, p, r)[0] + rng.normal(0, 0.3, (20, 2))
        f = Eskf(p + [6.0, -5.0, 3.0], [0, 0, -10.5], r @ exp_so3(np.radians([4.0, -3.0, 8.0])), prior, SILENT)
        r_prior = f.r.copy()
        f.update_landmarks(self.cam, points, pixels, 0.3**2, gate=math.inf, iterations=6)
        self.assertGreater(np.degrees(np.linalg.norm(log_so3(r_prior.T @ f.r))), 6.0)
        _, _, jp, jt = self.cam.jacobians(points, f.p, f.r)
        h = np.zeros((40, N_ERR))
        h[:, POS], h[:, ATT] = jp.reshape(40, 3), jt.reshape(40, 3)
        # The prior, moved to the new axes by the exact right Jacobian of the correction.
        a = log_so3(r_prior.T @ f.r)
        angle = np.linalg.norm(a)
        k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        jr = np.eye(3) - (1 - math.cos(angle))/angle**2*k + (angle - math.sin(angle))/angle**3*(k @ k)
        move = np.eye(N_ERR)
        move[ATT, ATT] = np.linalg.inv(jr)
        expected = np.linalg.inv(np.linalg.inv(move @ prior @ move.T) + h.T @ h/0.3**2)
        scale = np.sqrt(np.outer(np.diag(expected), np.diag(expected)))
        self.assertLess(np.max(abs(f.cov - expected)[:9, :9]/scale[:9, :9]), 0.02)

    def test_baro_update_and_gate(self):
        f = Eskf([0, 0, 100.0], [0, 0, -10.5], LEVEL, np.diag(self.sd**2), SILENT)
        accepted, nis = f.update_baro(140.0, 0.3)
        self.assertFalse(accepted)
        self.assertGreater(nis, 100)
        accepted, _ = f.update_baro(101.0, 0.3)
        self.assertTrue(accepted)
        # Height and its bias share the innovation in proportion to their variances.
        self.assertAlmostEqual(f.p[2] - 100.0, 9.0/(10.0 + 0.09), places=9)
        self.assertAlmostEqual(f.baro_bias, 1.0/(10.0 + 0.09), places=9)

    def test_protection_level_bounds_the_major_axis(self):
        cov = np.eye(N_ERR)
        cov[:2, :2] = [[9.0, 3.0], [3.0, 4.0]]
        f = Eskf([0, 0, 1.0], [0, 0, 0], LEVEL, cov, SILENT)
        self.assertAlmostEqual(f.protection_level(1.0)**2, np.linalg.eigvalsh(cov[:2, :2]).max(), places=12)
        self.assertAlmostEqual(f.protection_level(), 2.447746830681*f.protection_level(1.0), places=9)


if __name__ == '__main__':
    unittest.main()
