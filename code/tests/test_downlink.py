"""GPL-3.0-only. Downlink scheduling over a whole simulated mission."""
import bisect
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cansat import lora  # noqa: E402
from cansat.downlink import DownlinkScheduler  # noqa: E402
from cansat.flight_state import State  # noqa: E402
from cansat.telemetry import PACKET_SIZE  # noqa: E402

AIRTIME = lora.time_on_air(PACKET_SIZE, 7, 125e3)
PAD_S, FLIGHT_S, LANDED_S = 3*3600, 45, 3600


def mission(scheduler, step=0.1):
    """Three hours on the pad, a 45 s flight, an hour on the ground."""
    sent = []
    for i in range(int((PAD_S + FLIGHT_S + LANDED_S)/step)):
        t = i*step
        state = State.PAD if t < PAD_S else State.DESCENT if t < PAD_S + FLIGHT_S else State.LANDED
        if scheduler.due(t, state):
            sent.append(t)
    return sent


def worst_window(sent, window=3600.0):
    return max(bisect.bisect_right(sent, t) - bisect.bisect_right(sent, t - window) for t in sent)*AIRTIME


class Downlink(unittest.TestCase):
    def test_one_percent_band_keeps_flight_rate_and_limit(self):
        scheduler = DownlinkScheduler(AIRTIME, 0.01)
        self.assertAlmostEqual(scheduler.ground_interval, 9.5217, places=3)
        sent = mission(scheduler)
        flight = [t for t in sent if PAD_S <= t < PAD_S + FLIGHT_S]
        self.assertEqual(len(flight), FLIGHT_S)
        self.assertLess(max(b - a for a, b in zip(flight, flight[1:])), 1.0 + 1e-6)
        self.assertLessEqual(worst_window(sent), 36.0 + 1e-9)

    def test_ten_percent_band_runs_at_one_hertz_throughout(self):
        scheduler = DownlinkScheduler(AIRTIME, 0.10)
        self.assertEqual(scheduler.ground_interval, 1.0)
        sent = mission(scheduler)
        self.assertEqual(len(sent), PAD_S + FLIGHT_S + LANDED_S)
        self.assertLessEqual(worst_window(sent), 360.0)

    def test_unthrottled_one_hertz_would_break_one_percent(self):
        self.assertGreater(3600*AIRTIME, 36.0)

    def test_long_flight_never_exceeds_limit(self):
        # A flight far longer than the reserve degrades the rate, not the limit.
        scheduler = DownlinkScheduler(AIRTIME, 0.01)
        sent = [t/10 for t in range(0, 36000) if scheduler.due(t/10, State.DESCENT)]
        self.assertLessEqual(worst_window(sent), 36.0 + 1e-9)
        self.assertEqual(len(sent), int(36.0/AIRTIME))

    def test_impossible_reserve_is_rejected(self):
        with self.assertRaises(ValueError):
            DownlinkScheduler(AIRTIME, 0.001, flight_reserve_s=60.0)


if __name__ == '__main__':
    unittest.main()
