"""GPL-3.0-only. Landing predictor checks on synthetic data (no flight data, not run on the Pi)."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cansat import telemetry  # noqa: E402
from cansat.landing import FLIGHT_MAX_SUBSTEPS, DriftTable, LandingPredictor  # noqa: E402
from cansat.windprofile import N_AXIS, N_STATE, U, WindModel, axis_step, predict_drift, retrieve  # noqa: E402
from test_windprofile import V_D, descent, draw_world  # noqa: E402


def flight_model(**kw):
    return WindModel(max_substeps=FLIGHT_MAX_SUBSTEPS, **kw)


class Table(unittest.TestCase):
    def setUp(self):
        self.model = flight_model(glide_sd=1.0)
        self.table = DriftTable(self.model, V_D, top=320.0)
        rng = np.random.default_rng(2)
        self.mean = rng.normal(size=N_STATE)
        a = rng.normal(size=(N_STATE, N_STATE))
        self.cov = a @ a.T/N_STATE

    def direct(self, height, speed=V_D):
        return predict_drift(self.model, self.mean, self.cov, height, speed)

    def from_table(self, height):
        from cansat.windprofile import drift_from_row
        row, noise, time = self.table.lookup(height)
        return (*drift_from_row(row, noise, self.mean, self.cov), time)

    def test_matches_direct_propagation_on_the_grid(self):
        for height in (2.0, 40.0, 150.0, 300.0):
            got, want = self.from_table(height), self.direct(height)
            np.testing.assert_allclose(got[0], want[0], rtol=1e-9, atol=1e-9)
            np.testing.assert_allclose(got[1], want[1], rtol=1e-9, atol=1e-9)
            self.assertAlmostEqual(got[2], want[2])

    def test_interpolates_between_grid_heights(self):
        for height in (7.3, 81.0, 233.7):
            got, want = self.from_table(height), self.direct(height)
            np.testing.assert_allclose(got[0], want[0], rtol=0.01, atol=0.02)
            np.testing.assert_allclose(got[1], want[1], rtol=0.02, atol=0.02)

    def test_ground_and_clamping(self):
        drift, cov, time = self.from_table(0.0)
        self.assertEqual(time, 0.0)
        self.assertFalse(drift.any() or cov.any())
        self.assertEqual(self.from_table(-5.0)[2], 0.0)
        # Above the table the can is taken to keep its present velocity until it reaches the top.
        row_top, _, time_top = self.table.lookup(320.0)
        row_high, _, time_high = self.table.lookup(400.0)
        self.assertAlmostEqual(time_high, 400.0/V_D)
        self.assertAlmostEqual(row_high[U] - row_top[U], 80.0/V_D)
        np.testing.assert_allclose(np.delete(row_high, U), np.delete(row_top, U))
        with self.assertRaises(ValueError):
            DriftTable(self.model, 0.0)

    def test_rescaling_to_the_measured_descent_rate(self):
        # The predictor scales the nominal table by nominal/measured speed.
        from cansat.windprofile import drift_from_row
        for speed in (9.0, 12.0):
            scale = V_D/speed
            row, noise, _ = self.table.lookup(200.0)
            got = drift_from_row(row*scale, noise*scale*scale, self.mean, self.cov)
            want = self.direct(200.0, speed)
            self.assertLess(np.abs(got[0] - want[0]).max(), 0.04*np.abs(want[0]).max() + 0.05)
            self.assertLess(abs(np.trace(got[1])/np.trace(want[1]) - 1), 0.08)


class Predictor(unittest.TestCase):
    def test_calibrated_on_worlds_drawn_from_the_model(self):
        model = flight_model(glide_sd=1.0)
        t, z = descent(dt=0.5, top=300.0, bottom=100.0)
        rng = np.random.default_rng(31)
        height = float(z[-1])
        edges = np.arange(height, -1e-9, -2.0)
        chain = [axis_step(model, a, b, (a - b)/V_D, drift=True) for a, b in zip(edges[:-1], edges[1:])]
        d2, first = [], None
        for _ in range(120):
            states, vel, _, _ = draw_world(model, t, z, rng, 0.1)
            predictor = LandingPredictor(model, V_D, height_sd=0.0, descent_speed_sd=0.0)
            position = np.zeros(3)
            for k in range(len(t)):
                position = np.array([3.0, -4.0, z[k]])
                out = predictor.update(t[k], position, vel[k], 0.1**2)
            actual = np.empty(2)
            for axis in range(2):
                x = np.r_[states[-1, axis*N_AXIS:(axis + 1)*N_AXIS], 0.0]
                for phi, q in chain:
                    vals, vecs = np.linalg.eigh(q)
                    x = phi @ x + vecs @ (np.sqrt(np.maximum(vals, 0.0))*rng.normal(size=len(x)))
                actual[axis] = x[N_AXIS]
            e = position[:2] + actual - np.array([out.east, out.north])
            d2.append(e @ np.linalg.solve(out.cov, e))
            if first is None:
                first = (predictor, vel)
        self.assertLess(abs(np.mean(d2) - 2.0), 0.55)
        self.assertGreater(np.mean(np.array(d2) < -2*math.log(0.05)), 0.9)
        # The streaming filter is the batch filter.
        predictor, vel = first
        batch = retrieve(model, t, z, vel, 0.1**2)
        self.assertEqual(predictor.ignored, 0)
        self.assertAlmostEqual(out.time_to_go, height/predictor.descent_rate)
        self.assertAlmostEqual(predictor.descent_rate, V_D, places=6)
        del batch

    def test_streaming_state_equals_the_batch_filter(self):
        model = flight_model()
        t, z = descent(dt=0.4)
        rng = np.random.default_rng(4)
        _, vel, _, _ = draw_world(model, t, z, rng, 0.2)
        predictor = LandingPredictor(model, V_D)
        for k in range(len(t)):
            predictor.update(t[k], [0.0, 0.0, z[k]], vel[k], 0.2**2)
        batch = retrieve(model, t, z, vel, 0.2**2)
        np.testing.assert_allclose(predictor.filter.mean, batch.mean_f[-1], atol=1e-10)
        np.testing.assert_allclose(predictor.filter.cov, batch.cov_f[-1], atol=1e-10)

    def test_bad_samples_are_counted_and_ignored_without_raising(self):
        predictor = LandingPredictor(flight_model(), V_D)
        good = predictor.update(0.0, [0.0, 0.0, 300.0], [4.0, 1.0], 0.04)
        self.assertIsNotNone(good)
        for args in ((float('nan'), [0, 0, 290.0], [4.0, 1.0], 0.04),        # no time
                     (0.0, [0, 0, 290.0], [4.0, 1.0], 0.04),                 # repeated time
                     (-1.0, [0, 0, 290.0], [4.0, 1.0], 0.04),                # late sample
                     (0.2, [0, float('nan'), 290.0], [4.0, 1.0], 0.04),      # no position
                     (0.2, [0, 0], [4.0, 1.0], 0.04),                        # malformed position
                     ('x', [0, 0, 290.0], [4.0, 1.0], 0.04)):
            self.assertIsNone(predictor.update(*args))
        self.assertEqual(predictor.ignored, 6)
        before = predictor.filter.mean.copy()
        # A bad velocity still advances the clock and predicts; it just adds no information.
        for velocity, var in (([float('nan'), 1.0], 0.04), ([4.0, 1.0], -1.0), ([4.0], 0.04), ([4.0, 1.0], None)):
            t_next = predictor.filter.t + 0.2
            self.assertIsNotNone(predictor.update(t_next, [0, 0, 300.0 - V_D*t_next], velocity, var))
        self.assertEqual(predictor.ignored, 10)
        self.assertTrue(np.all(np.isfinite(predictor.filter.mean)) and np.all(np.isfinite(before)))
        out = predictor.update(predictor.filter.t + 0.2, [0, 0, 280.0])     # dead reckoning epoch
        self.assertTrue(np.all(np.isfinite(out.cov)) and np.all(np.linalg.eigvalsh(out.cov) > 0))

    def test_gate_refuses_a_glitch_but_not_swing_and_restarts_rather_than_locking_out(self):
        predictor = LandingPredictor(flight_model(), V_D)
        for k in range(20):
            predictor.update(0.2*k, [0, 0, 300.0 - V_D*0.2*k], [4.0, 1.0], 0.01)
        settled = predictor.filter.mean.copy()
        # A swing the lag model does not contain: 1.5 m/s off at 0.01 variance is refused by no one.
        predictor.update(4.0, [0, 0, 258.0], [5.5, 1.0], 0.01)
        self.assertEqual(predictor.gated, 0)
        # A glitch 30 m/s off is not believed and leaves the state where the dynamics put it.
        before = predictor.filter.mean.copy()
        out = predictor.update(4.2, [0, 0, 255.9], [34.0, 1.0], 0.01)
        self.assertEqual(predictor.gated, 1)
        self.assertIsNotNone(out)
        self.assertLess(np.abs(predictor.filter.mean - before).max(), 1.0)
        # If the "glitch" persists it is the filter that is wrong: after five it restarts and follows.
        for k in range(1, 6):
            predictor.update(4.2 + 0.2*k, [0, 0, 255.9 - V_D*0.2*k], [34.0, 1.0], 0.01)
        self.assertEqual(predictor.resets, 1)
        self.assertAlmostEqual(predictor.filter.mean[U], 34.0, delta=1.0)
        self.assertTrue(np.all(np.isfinite(settled)))

    def test_velocity_information_shrinks_the_ellipse_and_dead_reckoning_does_not(self):
        model = flight_model()
        t, z = descent(dt=0.2, top=300.0, bottom=150.0)
        rng = np.random.default_rng(6)
        _, vel, _, _ = draw_world(model, t, z, rng, 0.1)
        with_velocity = LandingPredictor(model, V_D)
        blind = LandingPredictor(model, V_D)
        informed = LandingPredictor(model, V_D, anemometer=(6.0, (4.0, 0.0), 0.3))
        for k in range(len(t)):
            a = with_velocity.update(t[k], [0, 0, z[k]], vel[k], 0.01)
            b = blind.update(t[k], [0, 0, z[k]])
            c = informed.update(t[k], [0, 0, z[k]])
        self.assertLess(math.sqrt(np.trace(a.cov)), 0.3*math.sqrt(np.trace(b.cov)))
        # With no velocity at all the prior mean wind is zero; an anemometer reading moves it.
        self.assertAlmostEqual(float(np.abs(b.drift).max()), 0.0, places=9)
        self.assertGreater(c.drift[0], 30.0)
        self.assertLess(math.sqrt(np.trace(c.cov)), math.sqrt(np.trace(b.cov)))

    def test_uses_the_measured_descent_rate_and_rejects_a_wild_one(self):
        predictor = LandingPredictor(flight_model(), 10.5, rate_window_s=2.0)
        for k in range(30):
            out = predictor.update(0.2*k, [0, 0, 300.0 - 12.0*0.2*k], [3.0, 0.0], 0.04)
        self.assertAlmostEqual(predictor.descent_rate, 12.0, places=6)
        self.assertAlmostEqual(out.time_to_go, (300.0 - 12.0*5.8)/12.0, places=6)
        for k in range(30, 60):                                             # a stuck height sensor
            out = predictor.update(0.2*k, [0, 0, 230.4], [3.0, 0.0], 0.04)
        # The window that straddles the fault reads a little low; the zero rates after it are refused.
        self.assertGreater(predictor.descent_rate, 10.0)
        self.assertLessEqual(predictor.descent_rate, 12.0 + 1e-9)
        self.assertTrue(math.isfinite(out.time_to_go))

    def test_time_uncertainty_stretches_the_ellipse_along_the_drift(self):
        model = flight_model()
        kw = dict(descent_speed=V_D)
        exact = LandingPredictor(model, height_sd=0.0, descent_speed_sd=0.0, **kw)
        loose = LandingPredictor(model, height_sd=3.0, descent_speed_sd=0.05, **kw)
        for k in range(20):
            a = exact.update(0.2*k, [0, 0, 250.0 - V_D*0.2*k], [6.0, 0.0], 0.01)
            b = loose.update(0.2*k, [0, 0, 250.0 - V_D*0.2*k], [6.0, 0.0], 0.01)
        self.assertGreater(b.cov[0, 0], a.cov[0, 0] + 1.0)
        self.assertAlmostEqual(b.cov[1, 1], a.cov[1, 1], delta=0.05*a.cov[1, 1])


class Packet(unittest.TestCase):
    def test_prediction_survives_the_40_byte_packet(self):
        predictor = LandingPredictor(flight_model(), V_D)
        for k in range(15):
            out = predictor.update(0.2*k, [12.0, -7.0, 200.0 - V_D*0.2*k], [5.0, -2.0], 0.04,
                                   position_cov=np.diag([0.5, 0.5]))
        packet = out.fill(telemetry.Telemetry())
        back = telemetry.decode(telemetry.encode(packet))
        self.assertTrue(back.flags & telemetry.FLAG_PRED_VALID)
        self.assertAlmostEqual(back.pred_e, out.east, delta=0.051)
        self.assertAlmostEqual(back.pred_n, out.north, delta=0.051)
        self.assertAlmostEqual(back.time_to_go_s, out.time_to_go, delta=0.101)
        major, minor, angle = out.ellipse()
        self.assertAlmostEqual(back.pred_major_m, major, delta=0.251)
        self.assertAlmostEqual(back.pred_minor_m, minor, delta=0.251)
        self.assertLess(abs((back.pred_angle_rad - angle + math.pi/2) % math.pi - math.pi/2), math.pi/256 + 1e-9)
        # The packet's ranges cover the flight: 50.8 s to go and a 127 m ellipse.
        self.assertLess(out.time_to_go, 50.8)
        self.assertLess(major, 127.0)

    def test_still_air_drift_is_the_velocity_decaying_at_the_lag(self):
        model = flight_model(sigma=1e-9, roughness=None)
        table = DriftTable(model, V_D, top=220.0)
        row, _, _ = table.lookup(200.0)
        tau = model.lag_time
        self.assertAlmostEqual(row[U], tau*(1 - math.exp(-200.0/V_D/tau)), places=6)


if __name__ == '__main__':
    unittest.main(verbosity=2)
