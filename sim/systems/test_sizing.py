"""GPL-3.0-only. Identity, limit and unit checks for the sizing models."""
import math
import unittest

import sizing as sz


class Canopy(unittest.TestCase):
    def test_drag_area_and_speed_are_inverse(self):
        for mass in (0.30, 0.35):
            for speed in (8.0, 10.5, 15.0):
                for rho in (1.18, 1.32):
                    self.assertAlmostEqual(sz.descent_speed(mass, rho, sz.drag_area(mass, speed, rho)), speed)

    def test_reference_diameter(self):
        # 2*0.325*9.80665/(1.225*10.5^2) = 0.04720 m^2; /0.75 = 0.06293 m^2; D = 0.2831 m.
        cd_s = sz.drag_area(0.325, 10.5, 1.225)
        self.assertAlmostEqual(cd_s, 0.047196, places=5)
        self.assertAlmostEqual(sz.nominal_diameter(cd_s, 0.75), 0.2831, places=4)

    def test_scaling_exponents(self):
        base = sz.descent_speed(0.325, 1.225, 0.05)
        self.assertAlmostEqual(sz.descent_speed(0.650, 1.225, 0.05)/base, math.sqrt(2))
        self.assertAlmostEqual(sz.descent_speed(0.325, 2.450, 0.05)/base, 1/math.sqrt(2))
        self.assertAlmostEqual(sz.descent_speed(0.325, 1.225, 0.10)/base, 1/math.sqrt(2))

    def test_fall_distance_limits(self):
        vt = 10.5
        # Early: free fall, with the first drag correction -g^3 t^4 / (12 vt^2).
        t = 0.01
        series = 0.5*sz.G0*t**2 - sz.G0**3*t**4/(12*vt**2)
        self.assertAlmostEqual(sz.fall_distance(vt, t), series, places=12)
        late = 20.0
        self.assertAlmostEqual(sz.fall_distance(vt, late), vt*late - vt**2/sz.G0*math.log(2), places=6)

    def test_fall_distance_matches_numerical_integration(self):
        vt, dt, v, x = 10.5, 1e-4, 0.0, 0.0
        for _ in range(30000):
            a = sz.G0*(1 - (v/vt)**2)
            x += v*dt + 0.5*a*dt*dt
            v += a*dt
        self.assertAlmostEqual(sz.fall_distance(vt, 3.0), x, places=3)
        self.assertAlmostEqual(v, vt*math.tanh(sz.G0*3.0/vt), places=3)

    def test_distance_to_fraction(self):
        vt = 10.5
        x = sz.distance_to_fraction(vt, 0.95)
        t = vt/sz.G0*math.atanh(0.95)
        self.assertAlmostEqual(sz.fall_distance(vt, t), x, places=9)

    def test_drop_inversion_recovers_terminal_speed(self):
        for vt in (3.0, 10.5, 15.0):
            for t in (2.0, 5.0, 12.0):
                h = sz.fall_distance(vt, t)
                self.assertAlmostEqual(sz.terminal_speed_from_drop(h, t), vt, places=6)

    def test_average_speed_understates_terminal_speed(self):
        vt, h = 10.5, 15.0
        t = next(x/1000 for x in range(1, 100000) if sz.fall_distance(vt, x/1000) >= h)
        self.assertLess(h/t, 0.7*vt)
        self.assertAlmostEqual(sz.terminal_speed_from_drop(sz.fall_distance(vt, t), t), vt, places=6)

    def test_drop_faster_than_free_fall_is_rejected(self):
        self.assertIsNone(sz.terminal_speed_from_drop(20.0, 2.0))

    def test_opening_force_round_trip_and_steady_limit(self):
        cd_s = sz.drag_area(0.325, 10.5, 1.225)
        self.assertAlmostEqual(sz.opening_force(cd_s, 1.225, 10.5, cx=1.0), 0.325*sz.G0)
        speed = sz.speed_for_opening_force(50.0, cd_s, 1.225)
        self.assertAlmostEqual(sz.opening_force(cd_s, 1.225, speed), 50.0)

    def test_floor_target(self):
        self.assertAlmostEqual(sz.target_speed_for_floor(10.0, 1.225, 1.225, 0.0), 10.0)
        target = sz.target_speed_for_floor(10.0, 1.225, 1.32, 0.03)
        # Worst case: dense air and a true speed two sigma below the reading.
        self.assertAlmostEqual(target/(1 + 0.06)*math.sqrt(1.225/1.32), 10.0)


class Others(unittest.TestCase):
    def test_timeline(self):
        row = sz.descent_timeline(300.0, 10.5)
        self.assertAlmostEqual(row['time_s'], 28.5714, places=4)
        self.assertAlmostEqual(row['frames_total'], 142.857, places=3)
        self.assertAlmostEqual(row['frames_per_layer'], 9.5238, places=4)
        self.assertAlmostEqual(row['lag_length_m'], 11.2424, places=4)

    def test_energy_round_trip(self):
        mah = sz.required_capacity_mah(1.63, 3.0, 0.88)
        self.assertAlmostEqual(sz.endurance_hours(sz.battery_energy_wh(mah), 1.63, 0.88), 3.0)

    def test_exposure_scaling(self):
        base = sz.photoelectrons(10000, 0.15, 2.4, 1e-3)
        self.assertAlmostEqual(sz.photoelectrons(20000, 0.15, 2.4, 1e-3), 2*base)
        self.assertAlmostEqual(sz.photoelectrons(10000, 0.15, 4.8, 1e-3), base/4)
        self.assertAlmostEqual(sz.photoelectrons(10000, 0.15, 2.4, 0.5e-3), base/2)
        # 10 klx * 0.15 * 0.85 / (4 * 2.4^2) = 55.34 lx on the sensor.
        self.assertAlmostEqual(base, 55.3385*1e16*9e-12*1e-3*0.5, delta=0.01)

    def test_shot_noise_limit(self):
        self.assertAlmostEqual(sz.shot_noise_snr(10000, read_noise=0), 100.0)
        self.assertLess(sz.shot_noise_snr(100, read_noise=5), 10.0)

    def test_density(self):
        self.assertAlmostEqual(sz.air_density(101325.0, 288.15), 1.2250, places=3)


if __name__ == '__main__':
    unittest.main()
