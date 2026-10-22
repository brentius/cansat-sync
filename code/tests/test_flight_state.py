"""GPL-3.0-only. Flight state machine against synthetic flights (no measured data)."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cansat.flight_state import FlightStateMachine, State  # noqa: E402
from sim_flight import random_flight, simulate  # noqa: E402

ORDER = ['ASCENT', 'DESCENT', 'LANDED']


def run(samples):
    machine = FlightStateMachine()
    for t, accel, pressure in samples:
        machine.update(t, accel, pressure)
    return machine


class FlightState(unittest.TestCase):
    def test_nominal_sequence_and_reasons(self):
        samples, truth = simulate()
        m = run(samples)
        self.assertEqual([(e[1], e[2]) for e in m.events], [(s, 'acceleration') for s in ORDER])
        self.assertLess(m.t_launch - truth['t_liftoff'], 0.25)
        self.assertLess(abs(m.t_descent - truth['t_eject']), 0.05)
        self.assertLess(abs(m.t_landed - truth['t_touchdown']), 0.05)
        self.assertLess(abs(m.max_altitude - truth['apogee_m']), 2.0)

    def test_randomised_envelope(self):
        for seed in range(150):
            samples, truth = random_flight(seed)
            m = run(samples)
            self.assertEqual([e[1] for e in m.events], ORDER, seed)
            self.assertTrue(0 < m.t_launch - truth['t_liftoff'] < 0.3, seed)
            # Never declares descent before the can has left the rocket.
            self.assertTrue(0 <= m.t_descent - truth['t_eject'] < 0.1, seed)
            self.assertTrue(0 <= m.t_landed - truth['t_touchdown'] < 0.1, seed)
            self.assertLess(abs(m.max_altitude - truth['apogee_m']), 3.0, seed)

    def test_barometer_only(self):
        for seed in range(40):
            samples, truth = random_flight(seed, accel_ok=False)
            m = run(samples)
            self.assertEqual([(e[1], e[2]) for e in m.events], [(s, 'altitude') for s in ORDER], seed)
            self.assertTrue(0 < m.t_launch - truth['t_liftoff'] < 2.5, seed)
            self.assertTrue(0 < m.t_descent - truth['t_apogee'] < 3.5, seed)
            self.assertTrue(0 < m.t_landed - truth['t_touchdown'] < 5.0, seed)

    def test_accelerometer_only(self):
        for seed in range(40):
            samples, truth = random_flight(seed, baro_ok=False)
            m = run(samples)
            self.assertEqual([(e[1], e[2]) for e in m.events], [(s, 'acceleration') for s in ORDER], seed)
            self.assertLess(abs(m.t_landed - truth['t_touchdown']), 0.1, seed)

    def test_quiet_ejection_falls_back_to_altitude(self):
        for seed in range(40):
            samples, truth = random_flight(seed, eject_spike_g=0.0)
            m = run(samples)
            self.assertEqual(m.events[1][1:], ('DESCENT', 'altitude'), seed)
            self.assertTrue(0 < m.t_descent - truth['t_apogee'] < 3.5, seed)

    def test_timeouts_when_both_cues_are_missing(self):
        samples, truth = simulate(baro_ok=False, eject_spike_g=0.0, touchdown_spike_g=0.0, rest_s=200.0)
        m = run(samples)
        self.assertEqual([(e[1], e[2]) for e in m.events],
                         [('ASCENT', 'acceleration'), ('DESCENT', 'timeout'), ('LANDED', 'timeout')])
        self.assertAlmostEqual(m.t_descent - m.t_launch, 15.0, delta=0.02)
        self.assertAlmostEqual(m.t_landed - m.t_descent, 180.0, delta=0.02)

    def test_three_hour_pad_wait_does_not_launch(self):
        bumps = [(600.0, 0.03, 10.0), (4000.0, 0.05, 14.0), (9000.0, 1.0, 2.5), (10000.0, 0.1, 6.0)]
        samples, _ = simulate(dt=0.05, pad_s=3*3600.0, drift_pa_per_s=200.0/10800, bumps=bumps, launch=False)
        m = run(samples)
        self.assertEqual(m.state, State.PAD)
        self.assertEqual(m.events, [])
        self.assertLess(abs(m.altitude), 2.0)

    def test_being_carried_uphill_does_not_launch(self):
        samples, _ = simulate(dt=0.02, pad_s=120.0, pad_climb_rate=2.0, launch=False)
        self.assertEqual(run(samples).state, State.PAD)

    def test_reference_tracks_weather_drift_before_launch(self):
        # 300 Pa of drift is about 25 m of apparent height if left uncorrected.
        samples, truth = simulate(pad_s=1800.0, dt=0.02, drift_pa_per_s=300.0/1800)
        m = run(samples)
        self.assertEqual([e[1] for e in m.events], ORDER)
        self.assertLess(abs(m.max_altitude - truth['apogee_m']), 3.0)

    def test_low_sample_rate_keeps_order(self):
        for seed in range(20):
            samples, truth = random_flight(seed, dt=0.05)
            m = run(samples)
            self.assertEqual([e[1] for e in m.events], ORDER, seed)
            self.assertTrue(0 <= m.t_descent - truth['t_eject'] < 3.5, seed)
            self.assertTrue(0 <= m.t_landed - truth['t_touchdown'] < 5.0, seed)

    def test_launch_before_reference_is_ready(self):
        samples, _ = simulate(pad_s=1.0)
        m = run(samples)
        self.assertEqual([e[1] for e in m.events], ORDER)
        self.assertIsNotNone(m.baseline_pa)


if __name__ == '__main__':
    unittest.main()
