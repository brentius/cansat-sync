"""GPL-3.0-only. Independent units, limits and numerical checks (synthetic)."""
import math
import unittest

import numpy as np

import models as m


class Conversion(unittest.TestCase):
    def test_documented_divider_values_and_global_power_bound(self):
        v, i, p = m.divider(10000)
        self.assertAlmostEqual(v, 0.1875)
        self.assertAlmostEqual(i, 18.75e-6)
        self.assertAlmostEqual(p, 3.515625e-6)
        r = np.geomspace(1, 1e9, 300)
        self.assertTrue(np.all(m.divider(r)[2] <= 15e-6))
        self.assertAlmostEqual(float(m.divider(150000)[2]), 15e-6)

    def test_beta_roundtrip(self):
        t = np.linspace(-20, 50, 60)
        np.testing.assert_allclose(m.beta_temperature(m.beta_resistance(t)), t, atol=1e-12)

    def test_three_point_fit_and_independent_held_out_data(self):
        fit = [0, 5, 8]
        np.testing.assert_allclose(m.sh_temperature(m.TE_R_OHM[fit], m.SH_COEFF), m.TE_T_C[fit], atol=1e-9)
        held = [1, 2, 3, 4, 6, 7]
        error = m.sh_temperature(m.TE_R_OHM[held], m.SH_COEFF)-m.TE_T_C[held]
        self.assertLess(float(abs(error).max()), 0.025)
        # Independent sourced point, outside beta25/85's definition interval.
        self.assertGreater(abs(float(m.beta_temperature(32650.9))), 0.5)

    def test_nominal_interpolation_hits_source_and_is_monotonic(self):
        np.testing.assert_allclose(m.nominal_resistance(m.TE_T_C), m.TE_R_OHM, rtol=1e-14)
        self.assertTrue(np.all(np.diff(m.nominal_resistance(np.linspace(-20, 50, 100))) < 0))
        with self.assertRaises(ValueError):
            m.nominal_resistance(-21)

    def test_pga_normalization_and_ratiometric_supply_cancellation(self):
        for vexc in (2.8, 3.0, 3.1):
            vntc = m.divider(10000, vexc)[0]
            self.assertAlmostEqual(float(m.resistance_from_volts(vntc, vexc)), 10000)
        # Exact at these values: 3000 codes at 62.5uV and 24000 at 125uV.
        c0, c1 = m.adc_code(0.1875, 2.048), m.adc_code(3., 4.096)
        self.assertEqual(int(c0), 3000)
        self.assertEqual(int(c1), 24000)
        self.assertAlmostEqual(float(m.resistance_from_volts(m.code_volts(c0, 2.048), m.code_volts(c1, 4.096))), 10000)
        bad = m.resistance_from_volts(float(c0), float(c1))
        self.assertGreater(abs(float(m.sh_temperature(bad, m.SH_COEFF))-25), 10)
        with self.assertRaises(ValueError):
            m.adc_code(3., 2.048)

    def test_readout_derivative_against_finite_difference(self):
        for r in (5000., 10000., 97006.9):
            v = float(m.divider(r)[0])
            j = m.readout_jacobian(v, 3.)
            def temperature(a, b):
                return m.sh_temperature(m.resistance_from_volts(a, b), m.SH_COEFF)
            eps = 1e-7
            diff = [(temperature(v+eps, 3)-temperature(v-eps, 3))/(2*eps),
                    (temperature(v, 3+eps)-temperature(v, 3-eps))/(2*eps)]
            np.testing.assert_allclose(j, diff, rtol=1e-7)

    def test_invalid_readings_are_refused(self):
        for a, b in ((0, 3), (-1, 3), (3, 3), (4, 3), (float('nan'), 3)):
            with self.assertRaises(ValueError):
                m.resistance_from_volts(a, b)
        with self.assertRaises(ValueError):
            m.fit_steinhart_hart([25, 25, 25], [10000, 10000, 10000])


class Thermal(unittest.TestCase):
    def test_pulse_exact_limits_and_energy_balance(self):
        for tau in (0.01, 0.1, 1., 20.):
            full = m.periodic_heating(1e-5, 3e-4, tau, 0.1, 0.1)
            off = m.periodic_heating(1e-5, 3e-4, tau, 0.1, 0)
            self.assertAlmostEqual(full['peak_k'], 1/30)
            self.assertEqual(off['peak_k'], 0)
            pulse = m.periodic_heating(1e-5, 3e-4, tau, 0.1, 0.02)
            self.assertLessEqual(pulse['trough_k'], pulse['mean_k'])
            self.assertGreaterEqual(pulse['peak_k'], pulse['mean_k'])
            # Integrate both exponentials separately to check average heat.
            peak, low, target = pulse['peak_k'], pulse['trough_k'], 1/30
            on_integral = target*.02+(low-target)*tau*(1-math.exp(-.02/tau))
            off_integral = peak*tau*(1-math.exp(-.08/tau))
            self.assertAlmostEqual((on_integral+off_integral)/.1, pulse['mean_k'], places=13)

    def test_rk4_matches_closed_form_no_heating(self):
        t = np.linspace(0, 5, 1001)
        config = m.Thermal(tau_s=.8)
        got = m.thermal_trace(t, 20., 40., 0, config, initial_c=15.)
        expected = 20-5*np.exp(-t/.8)
        np.testing.assert_allclose(got, expected, atol=3e-10)

    def test_lead_heat_equilibrium_and_time_constant(self):
        t = np.linspace(0, 12, 1201)
        config = m.Thermal(tau_s=1., lead_g_w_k=3e-6)
        got = m.thermal_trace(t, 20., 40., 0, config, initial_c=20.)
        equilibrium = (3e-4*20+3e-6*40)/(3e-4+3e-6)
        expected = equilibrium+(20-equilibrium)*np.exp(-t)
        np.testing.assert_allclose(got, expected, atol=1e-10)
        self.assertGreater(equilibrium-20, .19)

    def test_nonlinear_heating_converges_with_timestep(self):
        config = m.Thermal(tau_s=.2)
        a = m.thermal_trace(np.linspace(0, 2, 201), 25, 25, 1, config)
        b = m.thermal_trace(np.linspace(0, 2, 401), 25, 25, 1, config)
        np.testing.assert_allclose(a, b[::2], atol=1e-9)
        # Fixed-power estimate differs very little for this tiny heating rise.
        self.assertAlmostEqual(a[-1]-25, 3.515625e-6/3e-4, delta=1e-5)

    def test_resolved_nonlinear_pulses_match_small_heating_limit(self):
        t = np.arange(3001)*.001
        on = (np.arange(3001) % 100 < 20).astype(float)
        trace = m.thermal_trace(t, 25, 25, on, m.Thermal(tau_s=.2))
        pulse = m.periodic_heating(3.515625e-6, 3e-4, .2, .1, .02)
        # End of last ON interval; temperature-dependent R changes heat only
        # by ~0.01% at these millikelvin rises. This is an approximation check.
        self.assertAlmostEqual(trace[2920]-25, pulse['peak_k'], delta=1e-6)
        self.assertAlmostEqual(trace[3000]-25, pulse['trough_k'], delta=1e-6)

    def test_lag_smears_ramp_by_speed_times_tau(self):
        t = np.arange(0, 12.0001, .002)
        air = 15+.105*t
        got = m.thermal_trace(t, air, air, 0, m.Thermal(tau_s=1.), initial_c=15.)
        # ZOH lag adds half a time step; allow that separate discretization.
        self.assertAlmostEqual(air[-1]-got[-1], .105, delta=.0002)

    def test_bad_thermal_inputs(self):
        for kw in ({'tau_s':0}, {'air_g_w_k':0}, {'lead_g_w_k':-1}):
            with self.assertRaises(ValueError):
                m.Thermal(**kw)
        with self.assertRaises(ValueError):
            m.thermal_trace([0, 1], 25, 25, 0, m.Thermal(tau_s=.1))


class Scheduling(unittest.TestCase):
    def test_schedule_two_completed_conversions_and_rate_tolerance(self):
        candidate = m.pair_schedule(128, 10)
        self.assertTrue(candidate['feasible'])
        self.assertGreater(candidate['on_s'], 2/128)
        self.assertLess(candidate['duty'], .2)
        self.assertFalse(m.pair_schedule(8, 10)['feasible'])
        self.assertTrue(m.pair_schedule(8, 1)['feasible'])
        self.assertAlmostEqual(m.pair_schedule(128, 10, 0, 0, 0, 1)['on_s'], 2/128)

    def test_invalid_schedule_cannot_return_nan_or_optimistic_bound(self):
        for kwargs in ({'rate_fraction':1.1}, {'settle_s':float('nan')}, {'bus_s':float('inf')}):
            with self.assertRaises(ValueError):
                m.pair_schedule(128, 10, **kwargs)


if __name__ == '__main__':
    unittest.main()
