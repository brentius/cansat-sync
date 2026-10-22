# SPDX-License-Identifier: GPL-3.0-only
"""Synthetic checks for the offline thermal step fitter; no measurements."""
import math
import unittest

import numpy as np

from step_fit import fit_step


def irregular_times(seed=3, stop=35.0):
    rng = np.random.default_rng(seed)
    dt = rng.uniform(0.16, 0.58, 90)
    return np.r_[0.0, np.cumsum(dt[dt.cumsum() < stop])]


def independent_zoh(t, u, tau, step_time):
    """Reference recurrence written independently from the production fitter."""
    out = np.empty(len(t))
    out[0] = u[0]
    for k in range(1, len(t)):
        a, b = t[k - 1], t[k]
        if a < step_time < b:
            x_at_event = u[k - 1] + (out[k - 1] - u[k - 1])*math.exp(-(step_time-a)/tau)
            out[k] = u[k] + (x_at_event-u[k])*math.exp(-(b-step_time)/tau)
        else:
            out[k] = u[k - 1] + (out[k - 1]-u[k - 1])*math.exp(-(b-a)/tau)
    return out


class ThermalStepFit(unittest.TestCase):
    step = 5.137

    def test_continuous_analytic_irregular_step_recovers_tau(self):
        t = irregular_times()
        before, after, tau = 18.0, 25.0, 2.4
        u = np.where(t < self.step, before, after)
        # Closed-form continuous response to an ideal step, independent of
        # the fitter's sampled ZOH recurrence, including an off-grid event.
        x = np.where(t < self.step, before,
                     before + (after-before)*(1.0 - np.exp(-(t-self.step)/tau)))
        y = 0.7 + 1.03*x + 0.8*np.exp(-(t-t[0])/tau)
        got = fit_step(t, u, y, self.step)
        self.assertAlmostEqual(got['tau_s'], 2.4, delta=2e-4)
        self.assertAlmostEqual(got['offset_c'], 0.7, delta=2e-3)
        self.assertAlmostEqual(got['gain'], 1.03, delta=2e-4)
        self.assertAlmostEqual(got['initial_transient_c'], 0.8, delta=2e-3)
        self.assertLess(got['residual_rms_c'], 1e-9)
        self.assertAlmostEqual(got['t90_s'], math.log(10)*got['tau_s'])
        self.assertTrue(got['validity_flags']['tau_profile_informative'])

    def test_noisy_step_recovers_time_constant(self):
        t = irregular_times(seed=11, stop=45.0)
        u = np.where(t < self.step, 16.0, 24.0)
        x = independent_zoh(t, u, 4.2, self.step)
        rng = np.random.default_rng(12)
        y = -0.4 + 0.99*x + 0.5*np.exp(-(t-t[0])/4.2) + rng.normal(0, 0.04, len(t))
        got = fit_step(t, u, y, self.step)
        self.assertAlmostEqual(got['tau_s'], 4.2, delta=0.18)
        self.assertLess(got['residual_rms_c'], 0.06)

    def test_irregular_ramp_is_fit_with_actual_intervals(self):
        t = irregular_times(seed=19, stop=40.0)
        knot = float(t[12])
        u = 17.0 + np.clip((t-knot)/8.0, 0.0, 1.0)*6.0
        x = independent_zoh(t, u, 1.7, knot)
        y = 1.2 + 0.97*x + 0.35*np.exp(-(t-t[0])/1.7)
        got = fit_step(t, u, y, knot)
        self.assertAlmostEqual(got['tau_s'], 1.7, delta=0.01)
        self.assertLess(got['residual_rms_c'], 1e-8)

    def test_invalid_or_unexcited_records_are_rejected(self):
        t = np.arange(12.0)
        u = np.where(t < 5, 18.0, 25.0)
        y = u.copy()
        for bad_t in (np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10.0]),
                      np.array([0, 1, 2, 3, np.nan, 5, 6, 7, 8, 9, 10, 11])):
            with self.assertRaises(ValueError):
                fit_step(bad_t, u, y, 5.0)
        with self.assertRaisesRegex(ValueError, 'NaN'):
            fit_step(t, u, np.r_[y[:-1], np.nan], 5.0)
        with self.assertRaisesRegex(ValueError, 'excitation'):
            fit_step(t, np.full_like(t, 20.0), y, 5.0)

    def test_two_pole_response_is_flagged_by_residual_correlation(self):
        t = irregular_times(seed=27, stop=65.0)
        u = np.where(t < self.step, 15.0, 26.0)
        x_fast = independent_zoh(t, u, 0.55, self.step)
        x_slow = independent_zoh(t, x_fast, 5.0, self.step)
        y = 0.5 + x_slow
        got = fit_step(t, u, y, self.step)
        self.assertGreater(got['residual_rms_c'], 0.05)
        self.assertGreater(got['residual_lag1'], 0.3)
        self.assertFalse(got['validity_flags']['small_residual_lag1'])

    def test_constant_probe_is_flagged_as_no_response_and_unidentified(self):
        t = np.arange(0.0, 24.0, 0.3)
        u = np.where(t < self.step, 18.0, 25.0)
        got = fit_step(t, u, np.full_like(t, 21.0), self.step)
        self.assertFalse(got['validity_flags']['probe_response_present'])
        self.assertFalse(got['validity_flags']['tau_profile_informative'])

    def test_undersampled_fast_response_hits_lower_bound_and_is_unidentified(self):
        t = np.arange(0.0, 30.0, 0.7)
        step = 5.35
        before, after, true_tau = 17.0, 25.0, 0.003
        u = np.where(t < step, before, after)
        x = np.where(t < step, before,
                     before + (after-before)*(1.0 - np.exp(-np.maximum(t-step, 0.0)/true_tau)))
        got = fit_step(t, u, 0.4 + x, step)
        self.assertTrue(got['tau_hit_lower_bound'])
        self.assertFalse(got['validity_flags']['tau_profile_informative'])


if __name__ == '__main__':
    unittest.main()
