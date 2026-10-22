"""GPL-3.0-only. LoRa airtime, duty-cycle limiter and link budget."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cansat import lora  # noqa: E402
from cansat.telemetry import PACKET_SIZE  # noqa: E402


class Airtime(unittest.TestCase):
    def test_reference_airtimes(self):
        # Hand-evaluated from the SX1276 datasheet formula; the 13-byte values
        # are the widely published LoRaWAN minimum-frame airtimes.
        cases = [((10, 7), 41.216), ((13, 7), 46.336), ((13, 12), 1155.072), ((PACKET_SIZE, 7), 82.176)]
        for (payload, sf), milliseconds in cases:
            self.assertAlmostEqual(lora.time_on_air(payload, sf)*1e3, milliseconds, places=6)

    def test_low_data_rate_rule(self):
        self.assertFalse(lora.low_data_rate_required(10, 125e3))
        self.assertTrue(lora.low_data_rate_required(11, 125e3))
        self.assertFalse(lora.low_data_rate_required(12, 500e3))

    def test_airtime_scales_inversely_with_bandwidth(self):
        base = lora.time_on_air(40, 7, 125e3)
        self.assertAlmostEqual(lora.time_on_air(40, 7, 250e3), base/2)
        self.assertAlmostEqual(lora.time_on_air(40, 7, 500e3), base/4)

    def test_invalid_parameters(self):
        for args in [(40, 5), (40, 13), (-1, 7)]:
            with self.assertRaises(ValueError):
                lora.time_on_air(*args)


class Link(unittest.TestCase):
    def test_free_space_loss_and_sensitivity(self):
        self.assertAlmostEqual(lora.free_space_loss_db(1000, 868e6), 91.218, places=3)
        self.assertAlmostEqual(lora.sensitivity_dbm(7, 125e3), -124.53, places=2)
        # Each spreading-factor step buys 2.5 dB; each bandwidth doubling costs 3.01 dB.
        self.assertAlmostEqual(lora.sensitivity_dbm(8, 125e3) - lora.sensitivity_dbm(7, 125e3), -2.5)
        self.assertAlmostEqual(lora.sensitivity_dbm(7, 250e3) - lora.sensitivity_dbm(7, 125e3), 3.0103, places=4)

    def test_margin_falls_six_decibels_per_distance_doubling(self):
        near = lora.link_margin_db(500, 868e6, 14, 7, 125e3)
        far = lora.link_margin_db(1000, 868e6, 14, 7, 125e3)
        self.assertAlmostEqual(near - far, 6.0206, places=4)


class Duty(unittest.TestCase):
    def test_one_percent_budget_is_thirty_six_seconds_an_hour(self):
        airtime = lora.time_on_air(PACKET_SIZE, 7)
        limiter = lora.DutyCycleLimiter(0.01)
        sent = 0
        for second in range(3600):
            if limiter.allowed(second, airtime):
                limiter.record(second, airtime)
                sent += 1
        self.assertEqual(sent, int(36/airtime))
        self.assertEqual(sent, 438)
        self.assertLessEqual(limiter.used(3599), 36.0)
        # The first packet leaves the window exactly one hour after it was sent.
        self.assertFalse(limiter.allowed(3599.5, airtime))
        self.assertTrue(limiter.allowed(3600.0, airtime))

    def test_never_exceeds_limit_in_any_window(self):
        airtime = 0.3
        limiter = lora.DutyCycleLimiter(0.1, window_s=60)
        sent = []
        t = 0.0
        while t < 600:
            if limiter.allowed(t, airtime):
                limiter.record(t, airtime)
                sent.append(t)
            t += 0.7
        for start in sent:
            in_window = sum(airtime for s in sent if start <= s < start + 60)
            self.assertLessEqual(in_window, 6.0 + 1e-9)

    def test_reserve_holds_back_flight_airtime(self):
        limiter = lora.DutyCycleLimiter(0.01)
        limiter.record(0, 30.0)
        self.assertTrue(limiter.allowed(1, 1.0))
        self.assertFalse(limiter.allowed(1, 1.0, reserve_s=5.5))

    def test_min_interval(self):
        airtime = lora.time_on_air(PACKET_SIZE, 7)
        self.assertAlmostEqual(lora.min_interval(airtime, 0.10), 0.82176)
        self.assertAlmostEqual(lora.min_interval(airtime, 0.01), 8.2176)

    def test_invalid_limit(self):
        with self.assertRaises(ValueError):
            lora.DutyCycleLimiter(0)


if __name__ == '__main__':
    unittest.main()
