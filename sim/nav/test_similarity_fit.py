"""GPL-3.0-only. Tests for similarity_fit.py (SYNTHETIC; fixed seeds)."""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[0]/'maths'))
import similarity_fit as sf  # noqa: E402
import truth  # noqa: E402
from models import wind_cov  # noqa: E402


def log_profile(z, u_star, z0, heading):
    speed = u_star/sf.KAPPA*np.log(z/z0)
    return speed*math.cos(heading), speed*math.sin(heading), speed


class ExactRecovery(unittest.TestCase):
    def test_ols_recovers_noise_free_profile(self):
        z = sf.heights_for(10, 300)
        for u_star, z0 in ((0.43, 0.1), (0.2, 0.003), (0.9, 1.5)):
            fit = sf.ols_log_law(z, log_profile(z, u_star, z0, 0.3)[2])
            self.assertAlmostEqual(float(fit['u_star']), u_star, places=10)
            self.assertAlmostEqual(float(fit['z0'])/z0, 1.0, places=8)
            self.assertTrue(bool(fit['valid']))
            self.assertLess(float(fit['sd_ln_z0']), 1e-6)

    def test_gls_recovers_noise_free_profile(self):
        for lo, hi in sf.RANGES.values():
            z = sf.heights_for(lo, hi)
            for u_star, z0, heading in ((0.43, 0.1, 0.7), (0.2, 0.003, -2.5), (0.9, 1.5, 3.0)):
                u, v, _ = log_profile(z, u_star, z0, heading)
                fit = sf.fit_log_law(z, u, v)
                self.assertAlmostEqual(float(fit['u_star'])/u_star, 1.0, places=6)
                self.assertAlmostEqual(float(fit['ln_z0']), math.log(z0), places=5)
                self.assertAlmostEqual(float(fit['direction_rad']), heading, places=6)
                self.assertLess(float(fit['chi2']), 1e-6)

    def test_gls_recovers_with_anemometer_and_custom_cov(self):
        z = np.array([20.0, 35.0, 80.0, 150.0, 290.0])
        u, v, _ = log_profile(z, 0.5, 0.05, 1.1)
        mean6 = 0.5/sf.KAPPA*math.log(6/0.05)
        cov = np.diag([0.2, 0.3, 0.4, 0.5, 0.6])
        fit = sf.fit_log_law(z, u, v, cov=cov, anemometer=(6.0, mean6*math.cos(1.1), mean6*math.sin(1.1), 0.2))
        self.assertAlmostEqual(float(fit['z0']), 0.05, places=7)
        self.assertAlmostEqual(float(fit['u_star']), 0.5, places=7)

    def test_ols_flags_negative_slope(self):
        z = sf.heights_for(10, 100)
        fit = sf.ols_log_law(z, 10 - np.log(z))
        self.assertFalse(bool(fit['valid']))
        self.assertTrue(math.isnan(float(fit['z0'])))


class Kernel(unittest.TestCase):
    def test_matches_reference_implementation(self):
        z = np.array([2.0, 6.0, 10.0, 55.0, 300.0])
        for sigma, a in ((1.0, 0.5), (2.0, 0.25), (0.5, 1.0)):
            np.testing.assert_allclose(sf.turbulence_cov(z, sigma, a), wind_cov(z, sigma, a), atol=1e-12)

    def test_matches_empirical_covariance_of_make_wind(self):
        z = np.array([2.0, 6.0, 10.0, 30.0, 60.0, 100.0, 120.0])
        wp = replace(truth.WindParams(), sigma_h=1.3)
        rng = np.random.default_rng(11)
        n, acc = 24000, np.zeros((2, len(z), len(z)))
        for _ in range(6):
            _, _, turb = truth.make_wind(n//6, rng, replace(wp, top=120.0))
            x = turb[:, np.rint(z/wp.dz).astype(int), :2]
            acc += np.einsum('nia,nja->aij', x, x)
        want = sf.turbulence_cov(z, wp.sigma_h, wp.eddy_slope, wp.eddy_floor)
        # Sampling sd of a covariance entry is at most sigma^2*sqrt(2/n) = 0.015.
        for axis in range(2):
            np.testing.assert_allclose(acc[axis]/n, want, atol=0.06)

    def test_sampled_mean_is_the_log_law(self):
        z = sf.heights_for(2, 300)
        wp = truth.WindParams()
        u, v, heading = sf.sample_profiles(3, np.random.default_rng(1), replace(wp, sigma_h=1e-9), z)
        speed = sf.friction_velocity(wp.speed_10m, wp.roughness)/sf.KAPPA*np.log(z/wp.roughness)
        np.testing.assert_allclose(np.hypot(u, v), np.tile(speed, (3, 1)), atol=1e-6)
        np.testing.assert_allclose(np.arctan2(v[:, -1], u[:, -1]), heading, atol=1e-6)


class Uncertainty(unittest.TestCase):
    def test_cramer_rao_closed_form_matches_matrix_inverse(self):
        z = sf.heights_for(10, 300)
        cov = sf.turbulence_cov(z)
        u_star, z0 = 0.43, 0.1
        got = sf.linearised_cov(z, cov, u_star, z0)
        # Independent route: full 3-parameter Fisher matrix for (Ax, Ay, ln z0), wind along x.
        amp, h = u_star/sf.KAPPA, np.log(z/z0)
        n = len(z)
        jac = np.zeros((2*n, 3))
        jac[:n, 0], jac[n:, 1], jac[:n, 2] = h, h, -amp
        big = np.kron(np.eye(2), cov)
        inv = np.linalg.inv(jac.T @ np.linalg.solve(big, jac))
        self.assertAlmostEqual(got['sd_ln_z0']/math.sqrt(inv[2, 2]), 1.0, places=8)
        self.assertAlmostEqual(got['sd_u_star']/(sf.KAPPA*math.sqrt(inv[0, 0])), 1.0, places=8)
        self.assertAlmostEqual(math.radians(got['sd_direction_deg'])/(math.sqrt(inv[1, 1])/amp), 1.0, places=8)

    def test_fit_reports_the_fisher_covariance(self):
        z = sf.heights_for(10, 300)
        u, v, _ = log_profile(z, 0.43, 0.1, 0.0)
        fit = sf.fit_log_law(z, u, v)
        want = sf.linearised_cov(z, sf.turbulence_cov(z), 0.43, 0.1)
        np.testing.assert_allclose(fit['cov_ustar_lnz0'], want['cov_ustar_lnz0'], rtol=1e-5)

    def test_gls_covariance_matches_monte_carlo_scatter(self):
        # Low turbulence, so the estimator is close to linear and the bound should be met.
        wp = replace(truth.WindParams(), sigma_h=0.2)
        z = sf.heights_for(10, 300)
        n = 4000
        u, v, _ = sf.sample_profiles(n, np.random.default_rng(5), wp, z)
        fit = sf.fit_log_law(z, u, v, sigma_h=wp.sigma_h)
        u_star = sf.friction_velocity(wp.speed_10m, wp.roughness)
        want = sf.linearised_cov(z, sf.turbulence_cov(z, wp.sigma_h), u_star, wp.roughness)
        err = fit['ln_z0'] - math.log(wp.roughness)
        # sd of a sample sd is 1/sqrt(2n) = 1.1 %; allow 5 sigma plus a little non-linearity.
        self.assertAlmostEqual(np.std(err)/want['sd_ln_z0'], 1.0, delta=0.08)
        self.assertAlmostEqual(np.std(fit['u_star'])/want['sd_u_star'], 1.0, delta=0.08)
        self.assertLess(abs(np.mean(err)), 4*want['sd_ln_z0']/math.sqrt(n) + 0.05*want['sd_ln_z0'])
        self.assertAlmostEqual(np.mean(np.abs(err) <= fit['sd_ln_z0']), 0.683, delta=0.04)
        self.assertAlmostEqual(np.mean(sf.profile_delta(fit, math.log(wp.roughness)) <= sf.CHI2_95), 0.95,
                               delta=0.02)
        self.assertAlmostEqual(np.mean(fit['chi2'])/fit['dof'], 1.0, delta=0.02)

    def test_ols_sandwich_matches_monte_carlo_and_ols_is_overconfident(self):
        wp = replace(truth.WindParams(), sigma_h=0.2)
        z = sf.heights_for(10, 300)
        u, v, _ = sf.sample_profiles(4000, np.random.default_rng(6), wp, z)
        ols = sf.ols_log_law(z, np.hypot(u, v))
        u_star = sf.friction_velocity(wp.speed_10m, wp.roughness)
        want = sf.linearised_cov(z, sf.turbulence_cov(z, wp.sigma_h), u_star, wp.roughness, cov_fit=np.eye(len(z)))
        err = ols['ln_z0'] - math.log(wp.roughness)
        self.assertAlmostEqual(np.std(err)/want['sd_ln_z0'], 1.0, delta=0.1)
        self.assertLess(np.mean(np.abs(err) <= ols['sd_ln_z0']), 0.4)

    def test_anemometer_and_pooling_tighten_the_bound(self):
        z = sf.heights_for(80, 300)
        cov = sf.turbulence_cov(z)
        base = sf.linearised_cov(z, cov, 0.43, 0.1)['sd_ln_z0']
        self.assertLess(sf.linearised_cov(z, cov, 0.43, 0.1, anemometer=(6.0, 0.2))['sd_ln_z0'], 0.5*base)
        self.assertAlmostEqual(sf.linearised_cov(z, cov, 0.43, 0.1, n_pooled=4)['sd_ln_z0'], base/2, places=9)
        self.assertAlmostEqual(sf.linearised_cov(z, cov, 0.43, 0.1, anemometer=(6.0, 1e6))['sd_ln_z0'], base,
                               places=6)

    def test_profile_interval_brackets_the_estimate(self):
        z = sf.heights_for(10, 100)
        u, v, _ = sf.sample_profiles(200, np.random.default_rng(8), truth.WindParams(), z)
        fit = sf.fit_log_law(z, u, v)
        for key, level in (('ln_z0_interval_68', sf.CHI2_68), ('ln_z0_interval_95', sf.CHI2_95)):
            lo, hi = fit[key][:, 0], fit[key][:, 1]
            self.assertTrue(np.all(lo <= fit['ln_z0']) and np.all(fit['ln_z0'] <= hi))
            ends = np.isfinite(lo)
            np.testing.assert_allclose(sf.profile_delta(fit, np.where(ends, lo, fit['ln_z0']))[ends], level,
                                       rtol=1e-5)
        self.assertTrue(np.all(fit['ln_z0_interval_95'][:, 0] <= fit['ln_z0_interval_68'][:, 0]))
        # The closed-form maximum is a maximum: nearby values are no better.
        for step in (-0.05, 0.05, -2.0, 2.0):
            self.assertTrue(np.all(sf.profile_delta(fit, fit['ln_z0'] + step) >= 0))


class Invariance(unittest.TestCase):
    def setUp(self):
        self.z = sf.heights_for(10, 300)
        self.u, self.v, _ = sf.sample_profiles(40, np.random.default_rng(3), truth.WindParams(), self.z)

    def test_rotation_leaves_z0_and_u_star_unchanged(self):
        base = sf.fit_log_law(self.z, self.u, self.v)
        ols = sf.ols_log_law(self.z, np.hypot(self.u, self.v))
        for angle in (0.4, 2.0, -3.0):
            c, s = math.cos(angle), math.sin(angle)
            ur, vr = c*self.u - s*self.v, s*self.u + c*self.v
            fit = sf.fit_log_law(self.z, ur, vr)
            np.testing.assert_allclose(fit['ln_z0'], base['ln_z0'], atol=1e-8)
            np.testing.assert_allclose(fit['u_star'], base['u_star'], rtol=1e-9)
            np.testing.assert_allclose(fit['sd_ln_z0'], base['sd_ln_z0'], rtol=1e-8)
            turn = np.angle(np.exp(1j*(fit['direction_rad'] - base['direction_rad'] - angle)))
            np.testing.assert_allclose(turn, 0.0, atol=1e-8)
            np.testing.assert_allclose(sf.ols_log_law(self.z, np.hypot(ur, vr))['ln_z0'], ols['ln_z0'], atol=1e-9)

    def test_batch_equals_one_at_a_time_and_order_of_heights_is_irrelevant(self):
        batch = sf.fit_log_law(self.z, self.u, self.v)
        one = sf.fit_log_law(self.z, self.u[7], self.v[7])
        self.assertAlmostEqual(float(one['ln_z0']), float(batch['ln_z0'][7]), places=10)
        self.assertEqual(np.shape(one['u_star']), ())
        flipped = sf.fit_log_law(self.z[::-1], self.u[:, ::-1], self.v[:, ::-1])
        np.testing.assert_allclose(flipped['ln_z0'], batch['ln_z0'], atol=1e-7)

    def test_pooling_equals_fitting_the_average_with_scaled_covariance(self):
        cov = sf.turbulence_cov(self.z)
        a = sf.fit_log_law(self.z, self.u[:4].mean(0), self.v[:4].mean(0), n_pooled=4)
        b = sf.fit_log_law(self.z, self.u[:4].mean(0), self.v[:4].mean(0), cov=cov/4)
        self.assertAlmostEqual(float(a['ln_z0']), float(b['ln_z0']), places=10)
        self.assertAlmostEqual(float(a['sd_ln_z0']), float(b['sd_ln_z0']), places=10)

    def test_estimated_scale_ignores_the_assumed_sigma(self):
        a = sf.fit_log_law(self.z, self.u, self.v, sigma_h=1.0, estimate_scale=True)
        b = sf.fit_log_law(self.z, self.u, self.v, sigma_h=3.0, estimate_scale=True)
        np.testing.assert_allclose(a['sd_ln_z0'], b['sd_ln_z0'], rtol=1e-8)
        np.testing.assert_allclose(a['ln_z0_interval_95'], b['ln_z0_interval_95'], rtol=1e-6)


class Validation(unittest.TestCase):
    def setUp(self):
        self.z = sf.heights_for(10, 100)
        self.u, self.v, _ = log_profile(self.z, 0.4, 0.1, 0.2)

    def bad(self, *args, **kwargs):
        with self.assertRaises(ValueError):
            sf.fit_log_law(*args, **kwargs)

    def test_fit_rejects_bad_input(self):
        z, u, v = self.z, self.u, self.v
        self.bad(np.r_[0.0, z[1:]], u, v)
        self.bad(-z, u, v)
        self.bad(z, u[:-1], v[:-1])
        self.bad(z, u, np.tile(v, (2, 1)))
        self.bad(z, np.r_[np.nan, u[1:]], v)
        self.bad(z[:2], u[:2], v[:2])
        self.bad(np.full(5, 20.0), u[:5], v[:5])
        self.bad(z.reshape(2, -1), u, v)
        self.bad(z, u, v, cov=np.eye(3))
        self.bad(z, u, v, cov=-np.eye(len(z)))
        asym = np.eye(len(z))
        asym[0, 1] = 0.5
        self.bad(z, u, v, cov=asym)
        self.bad(z, u, v, sigma_h=0.0)
        self.bad(z, u, v, eddy_slope=-1.0)
        self.bad(z, u, v, noise=-0.1)
        self.bad(z, u, v, n_pooled=0)
        self.bad(z, u, v, anemometer=(6.0, 1.0, 1.0, 0.0))
        self.bad(z, u, v, anemometer=(-6.0, 1.0, 1.0, 0.2))
        self.bad(z, u, v, anemometer=(6.0, np.ones(3), np.ones(3), 0.2))
        self.bad(z, u, v, anemometer=(6.0, math.nan, 1.0, 0.2))

    def test_ols_rejects_bad_input(self):
        for z, s in ((self.z, self.u[:-1]), (np.r_[-1.0, self.z[1:]], self.u), (self.z[:2], self.u[:2]),
                     (np.full(4, 5.0), self.u[:4]), (self.z, np.r_[np.inf, self.u[1:]])):
            with self.assertRaises(ValueError):
                sf.ols_log_law(z, s)


class Study(unittest.TestCase):
    def test_run_case_is_reproducible_and_complete(self):
        case = {'anemometer_se': 0.2, 'n_pooled': 3, 'noise': 0.1, 'index': 4}
        a = sf.run_case(case, n=60)
        b = sf.run_case(case, n=60)
        self.assertEqual(sf._round(a), sf._round(b))
        self.assertEqual(set(a['ranges']), set(sf.RANGES))
        r = a['ranges']['80-300']
        self.assertEqual(r['n_levels'], 111)
        for method in ('ols', 'gls'):
            self.assertTrue(0 <= r[method]['frac_within_x3'] <= r[method]['frac_within_x10'] <= 1)
        self.assertAlmostEqual(a['truth']['u_star'], 0.4*5/math.log(100), places=12)
        self.assertLess(r['analytic']['cramer_rao']['sd_ln_z0'], r['analytic']['ols_on_component']['sd_ln_z0'])

    def test_case_names_are_unique_and_indexed(self):
        cases = sf.build_cases()
        self.assertEqual(len({c['index'] for c in cases.values()}), len(cases))
        self.assertEqual(cases['baseline'], {'index': 0})


if __name__ == '__main__':
    unittest.main()
