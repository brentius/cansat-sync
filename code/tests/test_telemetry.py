"""GPL-3.0-only. Telemetry packet codec."""
import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cansat import telemetry as tm  # noqa: E402

# Field -> quantisation step of the packet.
STEPS = dict(pressure_pa=1.0, temperature_c=0.01, altitude_m=0.1, gps_e=0.1, gps_n=0.1, gps_u=0.1,
             trn_e=0.1, trn_n=0.1, trn_major_m=0.1, trn_minor_m=0.1, protection_m=0.5,
             pred_e=0.1, pred_n=0.1, time_to_go_s=0.2, pred_major_m=0.5, pred_minor_m=0.5,
             battery_v=0.01)


def sample(rng):
    return tm.Telemetry(
        seq=rng.randrange(65536), t_ms=rng.randrange(2**32), state=rng.randrange(4),
        flags=tm.FLAG_GPS_VALID | tm.FLAG_TRN_VALID,
        pressure_pa=rng.uniform(95000, 104000), temperature_c=rng.uniform(-10, 35),
        altitude_m=rng.uniform(-20, 420), gps_e=rng.uniform(-900, 900), gps_n=rng.uniform(-900, 900),
        gps_u=rng.uniform(-20, 420), trn_e=rng.uniform(-900, 900), trn_n=rng.uniform(-900, 900),
        trn_major_m=rng.uniform(0, 25), trn_minor_m=rng.uniform(0, 25), trn_angle_rad=rng.uniform(0, math.pi),
        protection_m=rng.uniform(0, 120), landmarks=rng.randrange(60),
        pred_e=rng.uniform(-900, 900), pred_n=rng.uniform(-900, 900), time_to_go_s=rng.uniform(0, 45),
        pred_major_m=rng.uniform(0, 120), pred_minor_m=rng.uniform(0, 120),
        pred_angle_rad=rng.uniform(0, math.pi), battery_v=rng.uniform(3.0, 4.2))


class Codec(unittest.TestCase):
    def test_size_is_forty_bytes(self):
        self.assertEqual(tm.PACKET_SIZE, 40)
        self.assertEqual(len(tm.encode(tm.Telemetry())), 40)

    def test_crc_check_value(self):
        self.assertEqual(tm.crc16(b'123456789'), 0x29B1)

    def test_round_trip_within_half_a_step(self):
        rng = random.Random(1)
        for _ in range(500):
            sent = sample(rng)
            got = tm.decode(tm.encode(sent))
            for name in ('seq', 't_ms', 'state', 'flags', 'landmarks'):
                self.assertEqual(getattr(got, name), getattr(sent, name), name)
            for name, step in STEPS.items():
                self.assertLessEqual(abs(getattr(got, name) - getattr(sent, name)), step/2 + 1e-9, name)
            for name in ('trn_angle_rad', 'pred_angle_rad'):
                error = (getattr(got, name) - getattr(sent, name) + math.pi/2) % math.pi - math.pi/2
                self.assertLessEqual(abs(error), math.pi/512 + 1e-12, name)

    def test_missing_values_survive(self):
        got = tm.decode(tm.encode(tm.Telemetry(seq=7, pressure_pa=101325.0)))
        self.assertEqual(got.pressure_pa, 101325.0)
        for name in STEPS:
            if name != 'pressure_pa':
                self.assertTrue(math.isnan(getattr(got, name)), name)

    def test_out_of_range_saturates_without_aliasing_missing(self):
        got = tm.decode(tm.encode(tm.Telemetry(gps_e=1e6, gps_n=-1e6, trn_major_m=1e3, pressure_pa=2e5,
                                               temperature_c=-1e4, battery_v=9.0, time_to_go_s=1e3)))
        self.assertAlmostEqual(got.gps_e, 3276.7)
        self.assertAlmostEqual(got.gps_n, -3276.7)
        self.assertAlmostEqual(got.trn_major_m, 25.4)
        self.assertEqual(got.pressure_pa, 50000 + 0xFFFE)
        self.assertAlmostEqual(got.temperature_c, -327.67)
        self.assertAlmostEqual(got.battery_v, 5.04)
        self.assertAlmostEqual(got.time_to_go_s, 50.8)

    def test_every_single_bit_error_is_rejected(self):
        packet = tm.encode(sample(random.Random(2)))
        for bit in range(len(packet)*8):
            damaged = bytearray(packet)
            damaged[bit//8] ^= 1 << (bit % 8)
            with self.assertRaises(ValueError):
                tm.decode(bytes(damaged))

    def test_rejects_foreign_and_malformed_packets(self):
        packet = tm.encode(tm.Telemetry())
        with self.assertRaises(ValueError):
            tm.decode(packet[:-1])
        with self.assertRaises(ValueError):
            tm.decode(tm.encode(tm.Telemetry(), magic=0x11))
        self.assertEqual(tm.decode(tm.encode(tm.Telemetry(seq=3), magic=0x11), magic=0x11).seq, 3)

    def test_ellipse_round_trip(self):
        rng = random.Random(3)
        for _ in range(500):
            major, minor = sorted((rng.uniform(0.1, 30), rng.uniform(0.1, 30)), reverse=True)
            angle = rng.uniform(0, math.pi)
            cov = tm.ellipse_to_cov(major, minor, angle)
            got = tm.cov_to_ellipse(*cov)
            self.assertAlmostEqual(got[0], major, places=9)
            self.assertAlmostEqual(got[1], minor, places=9)
            for a, b in zip(tm.ellipse_to_cov(*got), cov):
                self.assertAlmostEqual(a, b, places=8)

    def test_ellipse_rejects_indefinite_matrix(self):
        with self.assertRaises(ValueError):
            tm.cov_to_ellipse(1.0, 2.0, 1.0)


if __name__ == '__main__':
    unittest.main()
