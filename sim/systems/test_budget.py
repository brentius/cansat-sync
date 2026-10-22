"""GPL-3.0-only. Arithmetic, unit, limit, consistency and regression checks for budget.py."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import budget as b  # noqa: E402
import sizing as sz  # noqa: E402


class Triangular(unittest.TestCase):
    def test_moments_match_numerical_integration(self):
        for low, mode, high in ((0.0, 1.0, 4.0), (2.0, 2.0, 5.0), (1.0, 3.0, 3.0), (97.0, 100.0, 106.0)):
            x = np.linspace(low, high, 400001)
            with np.errstate(divide='ignore', invalid='ignore'):
                pdf = np.where(x < mode, 2*(x - low)/((high - low)*(mode - low)),
                               2*(high - x)/((high - low)*(high - mode)))
            pdf = np.nan_to_num(pdf, nan=2/(high - low), posinf=2/(high - low))
            dx = x[1] - x[0]
            area = np.sum((pdf[1:] + pdf[:-1])/2)*dx
            mean = np.sum((pdf[1:]*x[1:] + pdf[:-1]*x[:-1])/2)*dx
            var = np.sum((pdf[1:]*x[1:]**2 + pdf[:-1]*x[:-1]**2)/2)*dx - mean**2
            self.assertAlmostEqual(area, 1.0, places=4)
            self.assertAlmostEqual(b.triangular_mean(low, mode, high), mean, places=4)
            self.assertAlmostEqual(b.triangular_variance(low, mode, high), var, places=3)

    def test_symmetric_and_degenerate(self):
        self.assertAlmostEqual(b.triangular_variance(0.0, 3.0, 6.0), 36/24)
        self.assertEqual(b.triangular_variance(5.0, 5.0, 5.0), 0.0)
        self.assertEqual(b.triangular_mean(5.0, 5.0, 5.0), 5.0)
        np.testing.assert_array_equal(b.triangular_ppf([0.0, 0.3, 1.0], 5.0, 5.0, 5.0), [5.0, 5.0, 5.0])

    def test_ppf_inverts_cdf(self):
        low, mode, high = 2.0, 3.0, 7.0
        u = np.linspace(0, 1, 101)
        x = b.triangular_ppf(u, low, mode, high)
        self.assertEqual(x[0], low)
        self.assertEqual(x[-1], high)
        self.assertTrue(np.all(np.diff(x) > 0))
        cdf = np.where(x < mode, (x - low)**2/((high - low)*(mode - low)),
                       1 - (high - x)**2/((high - low)*(high - mode)))
        np.testing.assert_allclose(cdf, u, atol=1e-12)
        self.assertAlmostEqual(float(b.triangular_ppf((mode - low)/(high - low), low, mode, high)), mode)

    def test_ppf_modes_at_the_ends(self):
        self.assertTrue(np.all(np.isfinite(b.triangular_ppf(np.linspace(0, 1, 11), 1.0, 1.0, 2.0))))
        self.assertTrue(np.all(np.isfinite(b.triangular_ppf(np.linspace(0, 1, 11), 1.0, 2.0, 2.0))))


class Items(unittest.TestCase):
    def test_validation(self):
        with self.assertRaises(ValueError):
            b.Item('x', 'x', 1, 2, 3, 'guess', '')
        with self.assertRaises(ValueError):
            b.Item('x', 'x', 3, 2, 1, 'assumed', '')
        with self.assertRaises(ValueError):
            b.Item('x', 'x', 1, 2, 3, 'datasheet', '')          # no URL
        with self.assertRaises(ValueError):
            b.Item('x', 'x', 1, 2, 3, 'vendor', '')             # no URL
        with self.assertRaises(ValueError):
            b.Item('x', 'x', 1, 2, 3, 'calculated', '')         # no inputs
        self.assertEqual(b.Item('x', 'x', 1, 2, 3, 'assumed', '').values, (1.0, 2.0, 3.0))

    def test_every_line_is_tagged_and_nothing_is_measured(self):
        for it in b.mass_items() + list(b.load_items().values()):
            self.assertIn(it.source, b.SOURCES)
            self.assertNotEqual(it.source, 'measured', it.key)
            self.assertLessEqual(it.low, it.typ)
            self.assertLessEqual(it.typ, it.high)
            self.assertGreater(it.low, 0.0, it.key)
            if it.source in ('datasheet', 'vendor'):
                self.assertTrue(it.url.startswith('https://'), it.key)
            if it.source == 'calculated':
                self.assertTrue(it.inputs, it.key)

    def test_keys_are_unique(self):
        keys = [it.key for it in b.mass_items()]
        self.assertEqual(len(keys), len(set(keys)))


class MassFormulae(unittest.TestCase):
    def test_pcb(self):
        # 100 cm2 x 0.16 cm x 1.85 g/cm3 = 29.6 g of laminate.
        self.assertAlmostEqual(b.pcb_mass_g(100.0, 1.6, 0), 29.6)
        # One full layer of 35 um copper: 0.35 cm3 x (8.96 - 1.85) = 2.4885 g more.
        self.assertAlmostEqual(b.pcb_mass_g(100.0, 1.6, 1, 35.0, 1.0) - 29.6, 2.4885)
        self.assertAlmostEqual(b.pcb_mass_g(50.0, 1.6, 4)/b.pcb_mass_g(100.0, 1.6, 4), 0.5)
        self.assertAlmostEqual(b.pcb_mass_g(100.0, 0.8, 0)/b.pcb_mass_g(100.0, 1.6, 0), 0.5)

    def test_main_board_area_and_mass(self):
        self.assertAlmostEqual(b.MAIN_PCB_MM[0]*b.MAIN_PCB_MM[1]/100, 28.16)
        item = {it.key: it for it in b.mass_items()}['main_pcb_bare']
        # 28.16 x 0.16 x 1.85 = 8.335 g; copper 28.16 x 4 x 0.0035 x 0.6 x 7.11 = 1.682 g.
        self.assertAlmostEqual(item.typ, 8.335 + 1.682, places=2)

    def test_tube(self):
        # Wall equal to the radius is a solid rod: pi r^2 L rho.
        self.assertAlmostEqual(b.tube_mass_g(20.0, 100.0, 10.0, 1.0), math.pi*1.0*10.0)
        # Thin wall tends to pi D t L rho.
        thin = b.tube_mass_g(65.0, 100.0, 0.01, 1.27)
        self.assertAlmostEqual(thin/(math.pi*6.5*0.001*10.0*1.27), 1.0, places=3)
        self.assertEqual(b.tube_mass_g(65.0, 100.0, 0.0, 1.27), 0.0)
        # Typical shell: pi (32.5^2 - 31.3^2) x 105 mm3 x 1.27 = 32.07 g.
        self.assertAlmostEqual(b.tube_mass_g(65.0, 105.0, 1.2, 1.27), 32.07, places=2)

    def test_disc(self):
        self.assertAlmostEqual(b.disc_mass_g(10.0, 10.0, 1.0), math.pi/4)
        self.assertAlmostEqual(b.disc_mass_g(30.0, 2.0, 1.19, hole_diameter_mm=30.0), 0.0)
        self.assertAlmostEqual(b.disc_area_cm2(20.0), math.pi)

    def test_parachute_parts(self):
        self.assertAlmostEqual(b.canopy_fabric_mass_g(0.07, 45.0, 1.3), 4.095)
        self.assertAlmostEqual(b.lines_mass_g(8, 0.45, 0.8), 2.88)
        self.assertEqual(sum(it.parachute for it in b.mass_items()), 3)


class MassBudget(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.items = b.mass_items()
        cls.out, cls.keys, cls.total = b.mass_budget()

    def test_totals_equal_sum_of_items(self):
        for i, case in enumerate(b.CASES):
            self.assertAlmostEqual(self.out['straight_sum_g'][case], sum(it.values[i] for it in self.items))
            self.assertAlmostEqual(self.out['straight_sum_g'][case],
                                   sum(row[case] for row in self.out['items']))
        self.assertAlmostEqual(sum(self.out['by_source_typical_g'].values()),
                               self.out['straight_sum_g']['typical'])
        self.assertEqual(self.out['by_source_typical_g']['measured'], 0)

    def test_parachute_split(self):
        chute = sum(it.typ for it in self.items if it.parachute)
        self.assertAlmostEqual(self.out['straight_sum_g']['typical']
                               - self.out['straight_sum_without_parachute_g']['typical'], chute)

    def test_monte_carlo_reproduces_analytic_moments(self):
        mean, var = b.analytic_moments(self.items)
        n = 400000
        draws = b.sample_items(self.items, n, seed=7)
        total = draws.sum(axis=1)
        self.assertLess(abs(total.mean() - mean), 4*math.sqrt(var/n))
        self.assertAlmostEqual(total.var(ddof=1)/var, 1.0, delta=0.01)
        # Item by item as well, and no draw outside its bounds.
        for j, it in enumerate(self.items):
            self.assertLess(abs(draws[:, j].mean() - it.mean), 5*math.sqrt(it.variance/n) + 1e-12, it.key)
            self.assertAlmostEqual(draws[:, j].var(ddof=1), it.variance, delta=0.02*it.variance + 1e-12)
            self.assertGreaterEqual(draws[:, j].min(), it.low)
            self.assertLessEqual(draws[:, j].max(), it.high)

    def test_items_are_uncorrelated_unless_grouped(self):
        draws = b.sample_items(self.items, 100000, seed=3)
        corr = np.corrcoef(draws.T)
        off = corr[~np.eye(len(self.items), dtype=bool)]
        self.assertLess(np.abs(off).max(), 0.02)
        grouped = b.sample_items(self.items, 100000, seed=3, correlate_groups=True)
        idx = [self.keys.index(k) for k in ('shell', 'end_caps', 'internal_frame')]
        sub = np.corrcoef(grouped[:, idx].T)
        self.assertGreater(sub.min(), 0.98)
        self.assertGreater(grouped.sum(axis=1).var(), draws.sum(axis=1).var())
        self.assertAlmostEqual(grouped.sum(axis=1).mean(), draws.sum(axis=1).mean(), delta=0.2)

    def test_variance_shares(self):
        shares = self.out['variance_shares']
        self.assertAlmostEqual(sum(r['share'] for r in shares), 1.0)
        self.assertEqual([r['share'] for r in shares], sorted((r['share'] for r in shares), reverse=True))
        self.assertEqual(shares[0]['key'], 'shell')

    def test_bounds(self):
        s = self.out['straight_sum_g']
        self.assertGreaterEqual(self.total.min(), s['low'])
        self.assertLessEqual(self.total.max(), s['high'])
        mc = self.out['monte_carlo_independent']
        self.assertLess(mc['p05'], mc['p50'])
        self.assertLess(mc['p50'], mc['p95'])

    def test_regression(self):
        s = self.out['straight_sum_g']
        self.assertAlmostEqual(s['low'], 250.38, places=2)
        self.assertAlmostEqual(s['typical'], 324.20, places=2)
        self.assertAlmostEqual(s['high'], 430.82, places=2)
        self.assertAlmostEqual(self.out['analytic_independent']['mean_g'], 335.13, places=2)
        self.assertAlmostEqual(self.out['analytic_independent']['sigma_g'], 9.337, places=3)
        mc = self.out['monte_carlo_independent']
        self.assertAlmostEqual(mc['mean'], 335.13, delta=0.1)
        self.assertAlmostEqual(mc['p_above_350'], 0.057, delta=0.004)
        self.assertLess(mc['p_below_300'], 0.001)
        self.assertAlmostEqual(self.out['monte_carlo_structure_correlated']['p_above_350'], 0.145, delta=0.006)

    def test_savings_are_ordered(self):
        for row in self.out['savings_g']:
            self.assertLessEqual(row['low'], row['typical'])
            self.assertLessEqual(row['typical'], row['high'])


class Power(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.out = b.power_budget()
        cls.loads = b.load_items()

    def test_milliamp_conversion(self):
        it = b._ma('x', 'x', 100, 100, 100, 'assumed', '')
        self.assertAlmostEqual(it.typ, 0.33)
        self.assertEqual(it.inputs['rail'], '3V3')
        self.assertAlmostEqual(self.loads['mcu'].typ, 60e-3*3.3)
        self.assertEqual(self.loads['pi_busy'].inputs['rail'], '5V1')

    def test_buzzer_current(self):
        # 3.3 V into 15 ohm at 50 % drive = 110 mA mean.
        self.assertAlmostEqual(self.loads['buzzer_on'].typ, 0.5*3.3*3.3/15.0)

    def test_radio_duty_follows_flight_code(self):
        airtime = b.packet_airtime_s()
        self.assertAlmostEqual(airtime, 0.082176, places=6)      # 40 bytes, SF7, 125 kHz
        self.assertAlmostEqual(b.radio_duty(True), airtime/1.0)
        self.assertAlmostEqual(b.radio_duty(False, 0.10), airtime/1.0)
        ground_1pct = b.radio_duty(False, 0.01)
        self.assertLess(ground_1pct, 0.01)
        # One hour on the pad plus the 60 s flight reserve exactly fills the 1 % budget.
        self.assertAlmostEqual(ground_1pct*3600 + 60*airtime, 36.0, places=6)

    def test_battery_power_limits(self):
        self.assertAlmostEqual(float(b.battery_power(1.0, 2.0, 1.0, 1.0)), 3.0)
        self.assertAlmostEqual(float(b.battery_power(0.0, 0.0, 0.87, 0.92, 0.05, 0.1)), 0.0)
        self.assertAlmostEqual(float(b.battery_power(0.87, 0.0, 0.87, 0.92)), 1.0)
        self.assertAlmostEqual(float(b.battery_power(0.0, 0.92, 0.87, 0.92)), 1.0)
        # Pi path: 1 A through 0.1 ohm adds 0.1 W at the rail.
        self.assertAlmostEqual(float(b.battery_power(0.0, b.V_5V1, 1.0, 1.0, r_pi=0.1)), b.V_5V1 + 0.1)

    def test_input_resistance_is_self_consistent(self):
        p, r, v = 5.0, 0.16, 6.0
        total = float(b.battery_power(p, 0.0, 1.0, 1.0, r_in=r, v_pack=v))
        self.assertAlmostEqual(total, p + (total/v)**2*r, places=10)
        self.assertGreater(total, p)
        self.assertAlmostEqual(float(b.battery_power(p, 0.0, 1.0, 1.0, r_in=1e-9, v_pack=v)), p, places=6)
        both = b.battery_power(np.array([1.0, 5.0]), 0.0, 1.0, 1.0, r_in=np.array([0.0, r]), v_pack=v)
        np.testing.assert_allclose(both, [1.0, total])

    def test_rail_sums(self):
        phase = b.phases()['descent']
        p33, p51 = b.rail_power(phase, self.loads, 1)
        want33 = sum(d*self.loads[k].typ for k, d in phase.items() if self.loads[k].inputs['rail'] == '3V3')
        want51 = sum(d*self.loads[k].typ for k, d in phase.items() if self.loads[k].inputs['rail'] == '5V1')
        self.assertAlmostEqual(p33, want33)
        self.assertAlmostEqual(p51, want51)
        self.assertAlmostEqual(p51, 2.8 + 0.8 + 0.15)
        self.assertEqual(b.rail_power({}, self.loads, 1), (0.0, 0.0))
        self.assertEqual(b.rail_power(b.phases()['pad_pi_off'], self.loads, 1)[1], 0.0)

    def test_radio_fractions_sum_to_one(self):
        for name, phase in b.phases().items():
            self.assertAlmostEqual(phase['radio_tx'] + phase['radio_idle'], 1.0, msg=name)
            for duty in phase.values():
                self.assertGreaterEqual(duty, 0.0)
                self.assertLessEqual(duty, 1.0)

    def test_phase_ordering(self):
        ph = self.out['phases']
        for case in b.CASES:
            self.assertLess(ph['pad_pi_off'][case]['battery_w'], ph['pad_pi_idle'][case]['battery_w'])
            self.assertLess(ph['pad_pi_idle'][case]['battery_w'], ph['pad_pi_busy'][case]['battery_w'])
            self.assertLess(ph['pad_pi_busy'][case]['battery_w'], ph['descent'][case]['battery_w'])
            self.assertGreater(ph['beacon'][case]['battery_w'], ph['pad_pi_off'][case]['battery_w'])
        for name, rows in ph.items():
            self.assertLess(rows['low']['battery_w'], rows['typical']['battery_w'], name)
            self.assertLess(rows['typical']['battery_w'], rows['high']['battery_w'], name)
            self.assertGreater(rows['low']['endurance_h'], rows['high']['endurance_h'], name)

    def test_converter_loss_is_bounded(self):
        for rows in self.out['phases'].values():
            for c, case in enumerate(b.CASES):
                rails = rows[case]['rail_3v3_w'] + rows[case]['rail_5v1_w']
                self.assertGreater(rows[case]['battery_w'], rails)
                self.assertLess(rows[case]['battery_w'], rails/min(b.ETA_3V3[c], b.ETA_5V1[c])*1.05)

    def test_endurance_is_energy_over_power(self):
        for rows in self.out['phases'].values():
            for c, case in enumerate(b.CASES):
                self.assertAlmostEqual(rows[case]['endurance_h']*rows[case]['battery_w'], b.usable_energy_wh(c))
                self.assertAlmostEqual(rows[case]['battery_ma_at_7v2']*7.2/1000, rows[case]['battery_w'])
                self.assertAlmostEqual(rows[case]['rail_3v3_ma']*3.3/1000, rows[case]['rail_3v3_w'])

    def test_pack_energy(self):
        self.assertAlmostEqual(b.nominal_energy_wh(1), 3.35*7.2)
        self.assertAlmostEqual(b.usable_fraction(1), 0.88*0.97*0.95)
        self.assertAlmostEqual(b.usable_energy_wh(1), 3.35*7.2*0.88*0.97*0.95)
        self.assertGreater(b.usable_energy_wh(0), b.usable_energy_wh(1))
        self.assertGreater(b.usable_energy_wh(1), b.usable_energy_wh(2))
        for c in range(3):
            self.assertLessEqual(b.usable_fraction(c), 1.0)
        # Same relation as the old sizing module, round trip through its inverse.
        mah = sz.required_capacity_mah(1.0, 3.0, 1.0, nominal_v=7.2, usable_fraction=b.usable_fraction(1))
        self.assertAlmostEqual(sz.battery_energy_wh(mah, 7.2, b.usable_fraction(1)), 3.0)

    def test_scenarios(self):
        for name, steps in b.SCENARIOS.items():
            for c, case in enumerate(b.CASES):
                want = sum(self.out['phases'][ph][case]['battery_w']*h for ph, h in steps)
                row = self.out['scenarios'][name][case]
                self.assertAlmostEqual(row['used_wh'], want)
                self.assertAlmostEqual(row['margin_wh'], row['usable_wh'] - row['used_wh'])
                self.assertAlmostEqual(row['nominal_wh_needed']*b.usable_fraction(c), row['used_wh'])
            self.assertGreaterEqual(sum(h for ph, h in steps if ph.startswith('pad')), b.POWERED_HOURS - 1e-12)
        sc = self.out['scenarios']
        for case in b.CASES:
            self.assertLess(sc['pi_on_late'][case]['used_wh'], sc['pi_on_throughout'][case]['used_wh'])
            self.assertLess(sc['pi_on_throughout'][case]['used_wh'], sc['pi_working_throughout'][case]['used_wh'])

    def test_peak_currents(self):
        peak = b.peak_currents()
        self.assertAlmostEqual(peak['rail_3v3_a'], 0.5664, places=4)
        self.assertAlmostEqual(peak['rail_5v1_a_average_high'], 4.9/5.1)
        self.assertGreater(peak['battery_a_at_6v0']*b.V_PACK_LOW, peak['rail_3v3_a']*3.3)
        for check in peak['checks']:
            self.assertAlmostEqual(check['ratio_limit_over_current'], check['limit_a']/check['current_a'])
            self.assertGreater(check['ratio_limit_over_current'], 1.5, check['check'])

    def test_regression(self):
        ph, sc = self.out['phases'], self.out['scenarios']
        self.assertAlmostEqual(ph['pad_pi_off']['typical']['battery_w'], 0.357, places=3)
        self.assertAlmostEqual(ph['pad_pi_idle']['typical']['battery_w'], 1.286, places=3)
        self.assertAlmostEqual(ph['descent']['typical']['battery_w'], 4.536, places=3)
        self.assertAlmostEqual(ph['pad_pi_idle']['typical']['endurance_h'], 15.21, places=2)
        self.assertAlmostEqual(ph['pad_pi_busy']['high']['endurance_h'], 2.30, places=2)
        self.assertAlmostEqual(sc['pi_on_throughout']['typical']['used_wh'], 4.31, places=2)
        self.assertAlmostEqual(sc['pi_on_late']['typical']['used_wh'], 1.68, places=2)
        self.assertLess(sc['pi_working_throughout']['high']['margin_wh'], 0.0)
        self.assertAlmostEqual(self.out['pack']['usable_wh'][1], 19.559, places=3)

    def test_monte_carlo_sits_inside_the_corners(self):
        mc = b.power_monte_carlo(n=20000, seed=5)
        for name, rows in self.out['phases'].items():
            p = mc['phases'][name]['battery_w']
            self.assertGreater(p['p01'], rows['low']['battery_w'])
            self.assertLess(p['p99'], rows['high']['battery_w'])
            self.assertGreater(mc['phases'][name]['endurance_h']['p01'], rows['high']['endurance_h'])
        self.assertEqual(mc['phases']['pad_pi_idle']['endurance_h']['p_below_3'], 0.0)
        self.assertLess(mc['usable_wh']['p99'], b.usable_energy_wh(0))
        self.assertGreater(mc['usable_wh']['p01'], b.usable_energy_wh(2))


class Descent(unittest.TestCase):
    def test_rule_constants(self):
        self.assertEqual((b.MASS_MIN_G, b.MASS_MAX_G), (300.0, 350.0))
        self.assertEqual((b.CAN_DIAMETER_MM, b.CAN_HEIGHT_MM), (66.0, 115.0))
        self.assertEqual((b.SPEED_MIN, b.POWERED_HOURS, b.CONNECTION_FORCE_N), (10.0, 3.0, 50.0))

    def test_densities(self):
        self.assertAlmostEqual(b.RHO_STD, 1.2250, places=3)
        self.assertAlmostEqual(b.RHO_COLD_DENSE, 103500.0/(287.05*273.15))
        self.assertLess(b.RHO_WARM_THIN, b.RHO_STD)
        self.assertLess(b.RHO_STD, b.RHO_COLD_DENSE)
        self.assertEqual(b.RHO_STD, sz.air_density(101325.0, 288.15))

    def test_area_and_speed_round_trip(self):
        for key, c in b.CANOPIES.items():
            for cd in c['cd']:
                area = b.canopy_area(0.335, 10.5, b.RHO_STD, cd)
                self.assertAlmostEqual(sz.descent_speed(0.335, b.RHO_STD, cd*area), 10.5)
                self.assertAlmostEqual(area*cd, sz.drag_area(0.335, 10.5, b.RHO_STD))
            self.assertLess(c['cd'][0], c['cd'][1])
            self.assertLess(c['cd'][1], c['cd'][2])

    def test_reference_area(self):
        # 2 x 0.335 x 9.80665 / (1.225 x 10.5^2) = 0.04865 m2; / 0.75 = 0.06487 m2; D = 0.2874 m.
        area = b.canopy_area(0.335, 10.5, 1.225, 0.75)
        self.assertAlmostEqual(area, 0.06487, places=5)
        self.assertAlmostEqual(b.canopy_dimension('disc', area), 0.2874, places=4)
        self.assertAlmostEqual(b.canopy_dimension('disc', area), sz.nominal_diameter(area*0.75, 0.75))

    def test_dimensions(self):
        self.assertAlmostEqual(b.canopy_dimension('square', 0.09), 0.30)
        self.assertAlmostEqual(b.canopy_dimension('disc', math.pi/4), 1.0)
        # Cross of arm length L and width 0.3 L covers 2 x 0.3 L^2 - 0.09 L^2 = 0.51 L^2.
        self.assertAlmostEqual(b.canopy_dimension('cross', 0.51), 1.0)
        with self.assertRaises(ValueError):
            b.canopy_dimension('hexagon', 1.0)

    def test_dropsonde_coefficient(self):
        # 2 x 0.323 x 9.80665 / (1.225 x 121) / 0.09 = 0.475.
        self.assertAlmostEqual(b.rd41_cd(), 0.475, places=3)
        lo, typ, hi = b.CANOPIES['square_cone']['cd']
        self.assertLess(lo, b.rd41_cd())
        self.assertLess(b.rd41_cd(), hi)

    def test_sensitivities_match_finite_differences(self):
        m, v, rho = 0.335, 10.5, b.RHO_STD
        cd_s = sz.drag_area(m, v, rho)
        s = b.speed_sensitivity(m, v, rho)
        dm = (sz.descent_speed(m + 1e-6, rho, cd_s) - sz.descent_speed(m - 1e-6, rho, cd_s))/2e-6
        dr = (sz.descent_speed(m, rho + 1e-6, cd_s) - sz.descent_speed(m, rho - 1e-6, cd_s))/2e-6
        self.assertAlmostEqual(s['per_gram_mps']*1000, dm, places=5)
        self.assertAlmostEqual(s['per_kg_m3_mps'], dr, places=5)
        self.assertAlmostEqual(s['per_10g_mps'], 10*s['per_gram_mps'])
        self.assertAlmostEqual(s['per_percent_density_mps'], dr*rho/100, places=6)

    def test_budget_with_fixed_mass(self):
        out = b.descent_budget(np.full(20000, 335.0))
        self.assertAlmostEqual(out['design_mass_g']['mean'], 335.0)
        cold = out['cold_day']['speed_if_trimmed_to_10.5_at_standard']
        warm = out['cold_day']['speed_warm_thin']
        self.assertAlmostEqual(cold, 10.5*math.sqrt(b.RHO_STD/b.RHO_COLD_DENSE))
        self.assertAlmostEqual(cold, 10.115, places=3)
        for key, c in out['canopies'].items():
            trim = c['trimmed_speed_mps']
            self.assertGreaterEqual(trim['p01'], cold)        # only density varies here
            self.assertLessEqual(trim['p99'], warm)
            corners = c['paper_design_corner_speeds_mps']
            self.assertGreaterEqual(c['paper_design_speed_mps']['p01'], corners['slowest'])
            self.assertLessEqual(c['paper_design_speed_mps']['p99'], corners['fastest'])
            self.assertAlmostEqual(sz.descent_speed(0.335, b.RHO_STD, c['cd'][1]*c['paper_design_area_m2']), 10.5)
            for row in c['sizing']:
                self.assertLess(row['area_m2_cd_high'], row['area_m2_cd_typical'])
                self.assertLess(row['area_m2_cd_typical'], row['area_m2_cd_low'])
                self.assertAlmostEqual(row['area_m2_cd_typical']*c['cd'][1], row['cd_s_m2'])
        areas = {k: c['paper_design_area_m2'] for k, c in out['canopies'].items()}
        self.assertLess(areas['annular'], areas['round_flat'])
        self.assertLess(areas['round_flat'], areas['square_cone'])
        lever = {row['mass_g']: row['speed_mps'] for row in out['mass_lever']}
        self.assertAlmostEqual(lever[350.0]/lever[300.0], math.sqrt(350/300))

    def test_floor_target(self):
        out = b.descent_budget(np.full(1000, 335.0))
        target = out['cold_day']['trim_target_for_10_floor_no_error']
        self.assertAlmostEqual(target*math.sqrt(b.RHO_STD/b.RHO_COLD_DENSE), 10.0)
        self.assertGreater(out['cold_day']['trim_target_for_10_floor_one_drop'], target)


if __name__ == '__main__':
    unittest.main()
