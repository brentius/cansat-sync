"""GPL-3.0-only. Wind retrieval checks on synthetic data (no flight data).

Three kinds of check: the discretisation against an independent integration of
the continuous model; the filter and smoother against a dense Gaussian solve of
the same problem; and statistical behaviour on worlds drawn from the model
itself. None of them says anything about a real canopy or a real atmosphere.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cansat.windprofile import (C, E, G, G0, N_AXIS, N_STATE, OFFSET, SQRT3, U, X1, X2, WindModel,  # noqa: E402
                                axis_step, expm, full_step, initial_state, lateral_airspeed,
                                measurement_rows, predict_drift, retrieve)

V_D = 10.5


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def descent(dt=0.4, top=300.0, bottom=20.0):
    t = np.arange(0.0, (top - bottom)/V_D, dt)
    return t, top - V_D*t


def transitions(model, t, z):
    return [full_step(model, z[k], z[k + 1], t[k + 1] - t[k]) for k in range(len(t) - 1)]


def draw_world(model, t, z, rng, velocity_sd=0.1, airspeed_sd=None, maps=None, anemometer=None):
    """States and measurements drawn from the model's own prior and noise."""
    mean, cov = initial_state(model, anemometer)
    states = np.empty((len(t), N_STATE))
    states[0] = mean + np.linalg.cholesky(cov + 1e-12*np.eye(N_STATE)) @ rng.normal(size=N_STATE)
    for k, (phi, q) in enumerate(transitions(model, t, z)):
        vals, vecs = np.linalg.eigh(q)
        states[k + 1] = phi @ states[k] + vecs @ (np.sqrt(np.maximum(vals, 0.0))*rng.normal(size=N_STATE))
    velocity = np.array([measurement_rows(model, zk) @ s for zk, s in zip(z, states)])
    velocity = velocity + velocity_sd*rng.normal(size=velocity.shape)
    airspeed = None
    if airspeed_sd is not None:
        airspeed = np.array([measurement_rows(model, zk, False, m) @ s for zk, s, m in zip(z, states, maps)])
        airspeed = airspeed + airspeed_sd*rng.normal(size=airspeed.shape)
    wind = np.array([[model.wind_row(zk, a) @ s for a in range(2)] for zk, s in zip(z, states)])
    return states, velocity, airspeed, wind


class Discretisation(unittest.TestCase):
    def test_matrix_exponential_of_the_turbulence_block(self):
        for r in (0.01, 0.3, 2.0, 9.0):
            got = expm(np.array([[0.0, 1.0], [-1.0, -2.0]])*r)
            want = math.exp(-r)*np.array([[1 + r, r], [-r, 1 - r]])
            np.testing.assert_allclose(got, want, rtol=0, atol=1e-12)

    def test_step_matches_runge_kutta_moments_with_height_varying_coefficients(self):
        model = WindModel(glide_sd=1.5, error_sd=0.3)
        for z_from, z_to, dt in ((300.0, 297.9, 0.2), (60.0, 49.5, 1.0), (12.0, 3.6, 0.8), (3.0, 0.2, 0.3)):
            speed = (z_from - z_to)/dt
            n = N_AXIS + 1

            def a_d(time):
                z = z_from - speed*time
                a = np.zeros((n, n))
                d = np.zeros((n, n))
                rate = speed/float(model.length(z))
                k = 1/model.lag_time
                a[X1, X2] = rate
                a[X2] = [-rate, -2*rate, 0, 0, 0, 0, 0]
                a[U] = [k*model.sigma, k*model.sigma*SQRT3, -k, k*float(model.shape(z)), k, 0, 0]
                a[G, G] = -1/model.glide_tau
                a[E, E] = -1/model.error_tau
                a[N_AXIS, U] = 1.0
                d[X2, X2] = rate
                d[G, G] = 2*model.glide_sd**2/model.glide_tau
                d[E, E] = 2*model.error_sd**2/model.error_tau
                return a, d

            def rhs(time, phi, q):
                a, d = a_d(time)
                return a @ phi, a @ q + q @ a.T + d

            phi, q = np.eye(n), np.zeros((n, n))
            steps = 4000
            h = dt/steps
            for i in range(steps):
                time = i*h
                k1 = rhs(time, phi, q)
                k2 = rhs(time + h/2, phi + h/2*k1[0], q + h/2*k1[1])
                k3 = rhs(time + h/2, phi + h/2*k2[0], q + h/2*k2[1])
                k4 = rhs(time + h, phi + h*k3[0], q + h*k3[1])
                phi = phi + h/6*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0])
                q = q + h/6*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
            got_phi, got_q = axis_step(model, z_from, z_to, dt, drift=True)
            np.testing.assert_allclose(got_phi, phi, rtol=0, atol=3e-3)
            np.testing.assert_allclose(got_q, q, rtol=0, atol=3e-3)
            fine_phi, fine_q = axis_step(WindModel(glide_sd=1.5, error_sd=0.3, max_dzeta=0.002), z_from, z_to, dt, drift=True)
            np.testing.assert_allclose(fine_phi, phi, rtol=0, atol=2e-5)
            np.testing.assert_allclose(fine_q, q, rtol=0, atol=2e-5)

    def test_prior_wind_covariance_is_the_drop_dryden_kernel(self):
        model = WindModel(sigma=1.3, mean_sd=0.0)
        z = np.array([300.0, 240.0, 150.0, 90.0, 40.0, 12.0, 4.0, 0.6])
        t = (300.0 - z)/V_D
        _, cov = initial_state(model)
        covs, phis = [cov], []
        for phi, q in transitions(model, t, z):
            phis.append(phi)
            covs.append(phi @ covs[-1] @ phi.T + q)
        zeta = model.zeta(z)
        for i in range(len(z)):
            through = np.eye(N_STATE)
            for j in range(i, len(z)):
                if j > i:
                    through = phis[j - 1] @ through
                got = model.wind_row(z[i], 0) @ covs[i] @ through.T @ model.wind_row(z[j], 0)
                r = abs(zeta[i] - zeta[j])
                self.assertAlmostEqual(got, 1.3**2*(1 - r/2)*math.exp(-r), places=9)

    def test_eddy_coordinate_and_shape(self):
        model = WindModel()
        self.assertAlmostEqual(float(model.zeta(1.0)), 2.0)
        self.assertAlmostEqual(float(model.zeta(math.e)), 4.0)
        self.assertAlmostEqual(float(model.zeta(0.5)), 1.0)
        self.assertAlmostEqual(float(model.shape(10.0)), 1.0)
        self.assertAlmostEqual(float(model.shape(100.0)), math.log(1000)/math.log(100))
        self.assertGreaterEqual(float(model.shape(0.0)), 0.0)
        self.assertEqual(float(WindModel(roughness=None).shape(3.0)), 1.0)


class AgainstDenseSolve(unittest.TestCase):
    """The filter, smoother, likelihood and layer variance against one big Gaussian conditioning."""

    def setUp(self):
        self.model = WindModel(glide_sd=2.0, error_sd=0.25, offset_sd=3.0)
        self.t, self.z = descent(dt=1.3, top=280.0, bottom=30.0)
        k = len(self.t)
        rng = np.random.default_rng(11)
        self.maps = np.array([-rot_z(0.4*i) for i in range(k)])
        self.anemometer = (6.0, (3.0, -1.0), 0.4)
        _, self.velocity, self.airspeed, _ = draw_world(self.model, self.t, self.z, rng, 0.1, 0.5, self.maps,
                                                        self.anemometer)
        self.velocity[3] = np.nan                  # a missing velocity sample
        self.airspeed[::2] = np.nan                # airspeed at half the epochs
        self.var_v = 0.1**2*np.ones(k)
        self.var_q = 0.5**2

    def dense(self):
        model, t, z = self.model, self.t, self.z
        k = len(t)
        mean0, cov0 = initial_state(model, self.anemometer)
        steps = transitions(model, t, z)
        means, covs = [mean0], [cov0]
        for phi, q in steps:
            means.append(phi @ means[-1])
            covs.append(phi @ covs[-1] @ phi.T + q)
        big = np.zeros((k*N_STATE, k*N_STATE))
        for i in range(k):
            through = np.eye(N_STATE)
            for j in range(i, k):
                if j > i:
                    through = steps[j - 1][0] @ through
                block = covs[i] @ through.T
                big[i*N_STATE:(i + 1)*N_STATE, j*N_STATE:(j + 1)*N_STATE] = block
                big[j*N_STATE:(j + 1)*N_STATE, i*N_STATE:(i + 1)*N_STATE] = block.T
        rows, ys, noise = [], [], []
        for i in range(k):
            for has, y, var, h in ((np.all(np.isfinite(self.velocity[i])), self.velocity[i], self.var_v[i],
                                    measurement_rows(model, z[i])),
                                   (np.all(np.isfinite(self.airspeed[i])), self.airspeed[i], self.var_q,
                                    measurement_rows(model, z[i], False, self.maps[i]))):
                if has:
                    for a in range(2):
                        row = np.zeros(k*N_STATE)
                        row[i*N_STATE:(i + 1)*N_STATE] = h[a]
                        rows.append(row)
                        ys.append(y[a])
                        noise.append(var)
        h_all, y_all = np.array(rows), np.array(ys)
        prior_mean = np.concatenate(means)
        s = h_all @ big @ h_all.T + np.diag(noise)
        resid = y_all - h_all @ prior_mean
        gain = np.linalg.solve(s, h_all @ big).T
        post_mean = prior_mean + gain @ resid
        post_cov = big - gain @ h_all @ big
        loglik = -0.5*(resid @ np.linalg.solve(s, resid) + np.linalg.slogdet(2*math.pi*s)[1])
        return post_mean.reshape(k, N_STATE), post_cov, loglik, len(ys)

    def test_smoothed_means_covariances_and_likelihood(self):
        out = retrieve(self.model, self.t, self.z, self.velocity, self.var_v, self.airspeed, self.maps, self.var_q,
                       self.anemometer)
        mean, cov, loglik, count = self.dense()
        np.testing.assert_allclose(out.mean, mean, rtol=0, atol=2e-8)
        for i in range(len(self.t)):
            s = slice(i*N_STATE, (i + 1)*N_STATE)
            np.testing.assert_allclose(out.cov[i], cov[s, s], rtol=0, atol=2e-8)
        self.assertAlmostEqual(out.log_likelihood, loglik, places=6)
        self.assertEqual(out.n_obs, count)

    def test_layer_mean_variance_keeps_cross_epoch_terms(self):
        out = retrieve(self.model, self.t, self.z, self.velocity, self.var_v, self.airspeed, self.maps, self.var_q,
                       self.anemometer)
        _, cov, _, _ = self.dense()
        centres, mean, sd, count = out.layer_means([30.0, 110.0, 190.0, 280.1])
        self.assertEqual(count.sum(), len(self.t))
        for layer, (lo, hi) in enumerate(((30.0, 110.0), (110.0, 190.0), (190.0, 280.1))):
            inside = np.flatnonzero((self.z >= lo) & (self.z < hi))
            for axis in range(2):
                weights = np.zeros(len(self.t)*N_STATE)
                for i in inside:
                    weights[i*N_STATE:(i + 1)*N_STATE] = self.model.wind_row(self.z[i], axis)/len(inside)
                self.assertAlmostEqual(sd[layer, axis]**2, weights @ cov @ weights, places=8)
                self.assertAlmostEqual(mean[layer, axis], np.mean(out.wind[inside, axis]), places=10)
        # Dropping the cross terms is not a small error, which is why they are kept.
        inside = np.flatnonzero(self.z >= 190.0)
        diagonal_only = sum(out.wind_var[i, 0] for i in inside)/len(inside)**2
        self.assertGreater(sd[2, 0]**2, 1.5*diagonal_only)

    def test_filtered_state_uses_only_past_data(self):
        full = retrieve(self.model, self.t, self.z, self.velocity, self.var_v, self.airspeed, self.maps, self.var_q)
        cut = 9
        part = retrieve(self.model, self.t[:cut], self.z[:cut], self.velocity[:cut], self.var_v[:cut],
                        self.airspeed[:cut], self.maps[:cut], self.var_q)
        np.testing.assert_allclose(full.mean_f[:cut], part.mean_f, atol=1e-12)
        np.testing.assert_allclose(part.mean[-1], part.mean_f[-1], atol=1e-12)


class Recovery(unittest.TestCase):
    def test_recovers_a_smooth_wind_through_an_independent_lag_integration(self):
        tau = V_D/G0

        def wind(z):
            return np.array([5.0 + 2.0*math.sin(z/40.0), 3.0*math.cos(z/60.0)])

        dt = 0.001
        steps = int(26.0/dt)
        u = wind(300.0) + np.array([1.0, -0.5])
        samples_t, samples_z, samples_u = [], [], []
        for i in range(steps):
            time = i*dt
            if i % 200 == 0:
                samples_t.append(time)
                samples_z.append(300.0 - V_D*time)
                samples_u.append(u.copy())

            def rate(tt, uu):
                return (wind(300.0 - V_D*tt) - uu)/tau

            k1 = rate(time, u)
            k2 = rate(time + dt/2, u + dt/2*k1)
            k3 = rate(time + dt/2, u + dt/2*k2)
            k4 = rate(time + dt, u + dt*k3)
            u = u + dt/6*(k1 + 2*k2 + 2*k3 + k4)
        t, z, vel = np.array(samples_t), np.array(samples_z), np.array(samples_u)
        rng = np.random.default_rng(3)
        vel = vel + 0.02*rng.normal(size=vel.shape)
        out = retrieve(WindModel(roughness=None, lag_time=tau), t, z, vel, 0.02**2)
        truth = np.array([wind(h) for h in z])
        err = (out.wind - truth)[5:-5]
        self.assertLess(np.sqrt(np.mean(err**2)), 0.08)
        # Ignoring the lag (reading the velocity as the wind) is several times worse.
        self.assertGreater(np.sqrt(np.mean((vel - truth)[5:-5]**2)), 0.3)

    def test_consistent_on_worlds_drawn_from_the_model(self):
        model = WindModel(glide_sd=1.5, error_sd=0.2, offset_sd=2.0)
        t, z = descent(dt=0.5, top=300.0, bottom=10.0)
        maps = np.array([-rot_z(0.25*i) for i in range(len(t))])
        rng = np.random.default_rng(5)
        nees, inside, layers, glide_z = [], 0, 0, []
        edges = np.arange(20.0, 301.0, 40.0)
        for _ in range(60):
            states, vel, air, wind = draw_world(model, t, z, rng, 0.1, 0.6, maps)
            out = retrieve(model, t, z, vel, 0.1**2, air, maps, 0.6**2)
            for k in (5, 20, 40):
                for axis in range(2):
                    nees.append((out.wind[k, axis] - wind[k, axis])**2/out.wind_var[k, axis])
            _, mean, sd, count = out.layer_means(edges)
            for i in range(len(edges) - 1):
                sel = (z >= edges[i]) & (z < edges[i + 1])
                true_mean = wind[sel].mean(axis=0)
                inside += int(np.sum(np.abs(mean[i] - true_mean) <= 1.96*sd[i]))
                layers += 2
            glide_z.extend((out.glide[30] - states[30, [G, N_AXIS + G]])/np.sqrt(out.glide_var[30]))
        # 360 nearly independent unit chi-square draws: mean 1 +/- 0.075.
        self.assertLess(abs(np.mean(nees) - 1.0), 0.25)
        self.assertLess(abs(inside/layers - 0.95), 0.03)
        self.assertLess(abs(np.std(glide_z) - 1.0), 0.2)


class GlideObservability(unittest.TestCase):
    def posterior_glide_sd(self, airspeed, spin, offset_sd=3.0):
        model = WindModel(glide_sd=2.0, glide_tau=200.0, offset_sd=offset_sd)
        t, z = descent(dt=0.5, top=300.0, bottom=80.0)
        maps = np.array([-rot_z(spin*ti) for ti in t])
        rng = np.random.default_rng(8)
        _, vel, air, _ = draw_world(model, t, z, rng, 0.1, 0.5, maps)
        out = retrieve(model, t, z, vel, 0.1**2, air if airspeed else None, maps, 0.5**2)
        return float(np.sqrt(out.glide_var[len(t)//2, 0]))

    def test_glide_is_not_observable_from_velocity_alone(self):
        self.assertGreater(self.posterior_glide_sd(airspeed=False, spin=0.5), 1.6)

    def test_airspeed_and_spin_determine_the_glide(self):
        self.assertLess(self.posterior_glide_sd(airspeed=True, spin=0.5), 0.25)

    def test_without_spin_an_uncalibrated_offset_hides_the_glide(self):
        self.assertGreater(self.posterior_glide_sd(airspeed=True, spin=0.0), 1.5)

    def test_a_calibrated_offset_needs_no_spin(self):
        self.assertLess(self.posterior_glide_sd(airspeed=True, spin=0.0, offset_sd=0.15), 0.3)


class LateralAirspeed(unittest.TestCase):
    def scene(self, offset=(0.0, 0.0)):
        dt, n = 0.01, 600
        t = np.arange(n)*dt
        tilt = math.radians(9.0)
        r0 = np.array([[1, 0, 0], [0, math.cos(tilt), -math.sin(tilt)], [0, math.sin(tilt), math.cos(tilt)]]) @ np.diag(
            [1.0, -1.0, -1.0])
        spin = 1.3
        r = np.array([np.array([[math.cos(spin*x), -math.sin(spin*x), 0], [math.sin(spin*x), math.cos(spin*x), 0],
                                [0, 0, 1]]) @ r0 for x in t])
        gyro = np.tile(r0.T @ np.array([0.0, 0.0, spin]), (n, 1))
        air = np.column_stack((1.9 + 0.3*np.sin(0.7*t), -0.8 + 0.2*np.cos(0.4*t), np.full(n, -10.4)))
        speed = np.linalg.norm(air, axis=1)
        kappa_ref, v_ref = 0.0624, 10.5
        lever = np.array([0.005, 0.0, -0.010])
        side = -(kappa_ref*speed/v_ref)[:, None]*np.einsum('kji,kj->ki', r, air)
        accel = side + np.cross(gyro, np.cross(gyro, lever))
        accel[:, 2] = 9.8                                   # the axial component must not matter
        accel[:, :2] += np.array(offset)
        idx, got, maps = lateral_airspeed(accel, gyro, r, air[:, 2], dt, kappa_ref, v_ref, lever, window=100, every=20)
        weights = np.hanning(103)[1:-1]
        weights /= weights.sum()
        want = np.array([weights @ air[i - 50:i + 51, :2] for i in idx])
        return got, want, maps, kappa_ref

    def test_recovers_the_horizontal_airspeed_of_a_tilted_spinning_can(self):
        got, want, _, _ = self.scene()
        # Exact but for the total airspeed being taken from the window average (parts in 10^4 here).
        np.testing.assert_allclose(got, want, rtol=0, atol=2e-3)
        self.assertEqual(len(got), len(np.arange(50, 600 - 50, 20)))

    def test_offset_enters_through_the_returned_map(self):
        offset = np.array([0.03, -0.02])                    # about 3 mg and 2 mg
        got, want, maps, kappa = self.scene(offset)
        predicted = want + np.einsum('mij,j->mi', maps, offset/kappa)
        self.assertGreater(np.abs(got - want).max(), 0.3)   # the offset is worth tenths of a metre per second
        # Linear in the offset to within the change it makes to the total airspeed.
        np.testing.assert_allclose(got, predicted, rtol=0, atol=0.05)


class Landing(unittest.TestCase):
    def test_drift_prediction_is_calibrated_on_worlds_drawn_from_the_model(self):
        model = WindModel(glide_sd=1.0)
        t, z = descent(dt=0.5, top=300.0, bottom=100.0)
        rng = np.random.default_rng(21)
        height = float(z[-1])
        edges = np.linspace(height, 0.0, int(math.ceil(height/2.0)) + 1)
        chain = [axis_step(model, a, b, (a - b)/V_D, drift=True) for a, b in zip(edges[:-1], edges[1:])]
        d2 = []
        for _ in range(150):
            states, vel, _, _ = draw_world(model, t, z, rng, 0.1)
            out = retrieve(model, t, z, vel, 0.1**2)
            drift, cov, time = predict_drift(model, out.mean_f[-1], out.cov_f[-1], height, V_D)
            actual = np.empty(2)
            for axis in range(2):
                x = np.r_[states[-1, axis*N_AXIS:(axis + 1)*N_AXIS], 0.0]
                for phi, q in chain:
                    vals, vecs = np.linalg.eigh(q)
                    x = phi @ x + vecs @ (np.sqrt(np.maximum(vals, 0.0))*rng.normal(size=len(x)))
                actual[axis] = x[N_AXIS]
            e = actual - drift
            d2.append(e @ np.linalg.solve(cov, e))
        self.assertAlmostEqual(time, height/V_D)
        # Mean of 150 chi-square(2) draws: 2 +/- 0.16.
        self.assertLess(abs(np.mean(d2) - 2.0), 0.5)
        self.assertGreater(np.mean(np.array(d2) < -2*math.log(0.05)), 0.9)

    def test_no_drift_left_at_the_ground(self):
        drift, cov, time = predict_drift(WindModel(), np.ones(N_STATE), np.eye(N_STATE), 0.0, V_D)
        self.assertEqual(time, 0.0)
        self.assertFalse(drift.any() or cov.any())

    def test_still_air_drift_is_the_velocity_decaying_at_the_lag(self):
        model = WindModel(sigma=1e-9, roughness=None)
        mean = np.zeros(N_STATE)
        mean[U] = 4.0                                        # moving sideways through still air
        drift, _, _ = predict_drift(model, mean, np.zeros((N_STATE, N_STATE)), 200.0, V_D)
        tau = V_D/G0
        self.assertAlmostEqual(drift[0], 4.0*tau*(1 - math.exp(-200.0/V_D/tau)), places=6)
        self.assertAlmostEqual(drift[1], 0.0, places=9)


class Validation(unittest.TestCase):
    def test_rejects_bad_input(self):
        model = WindModel()
        t, z = descent()
        vel = np.zeros((len(t), 2))
        with self.assertRaises(ValueError):
            retrieve(model, t, z)
        with self.assertRaises(ValueError):
            retrieve(model, t[::-1], z, vel, 0.01)
        with self.assertRaises(ValueError):
            retrieve(model, t, z[:-1], vel, 0.01)
        with self.assertRaises(ValueError):
            retrieve(model, t, z, vel, np.ones(3))
        with self.assertRaises(ValueError):
            axis_step(model, 10.0, 9.0, 0.0)

    def test_switched_off_states_stay_at_zero(self):
        model = WindModel()
        t, z = descent()
        rng = np.random.default_rng(1)
        _, vel, _, _ = draw_world(model, t, z, rng, 0.1)
        out = retrieve(model, t, z, vel, 0.01)
        self.assertEqual(float(np.abs(out.glide).max()), 0.0)
        self.assertEqual(float(np.abs(out.offset).max()), 0.0)
        self.assertEqual(float(np.abs(out.mean[:, [E, N_AXIS + E]]).max()), 0.0)
        self.assertTrue(np.all(np.linalg.eigvalsh(out.cov[len(t)//2]) > -1e-9))
        self.assertEqual(out.mean[:, OFFSET].shape, (len(t), 2))
        self.assertEqual(out.mean[:, C].shape, (len(t),))


if __name__ == '__main__':
    unittest.main(verbosity=2)
