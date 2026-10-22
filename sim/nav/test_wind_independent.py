"""GPL-3.0-only. Independent checks of cansat/windprofile.py and cansat/landing.py. SYNTHETIC.

Every number here comes from simulation (sim/nav/truth.py) or from formulas
re-derived in this file; nothing is a measurement of the real CanSat. The
checks deliberately avoid the state-space matrices of the code under test:

  1  the lateral specific-force claim, against the multibody simulator, with the
     residual identified through Euler's equation for the can (gyro data only)
  2  retrieve() against an exact Gaussian-process solve built from the kernel
  3  calibration of layer means on truth.make_wind profiles and a lag ODE
     integrated here
  4  DriftTable against brute-force Monte Carlo; LandingPredictor.update fuzzed
  5  a static guard on the NumPy names and Python syntax the flight modules use

Six tests here first recorded defects as expected failures. The code was corrected the
same day and they now run as ordinary tests; the comment above each says what was wrong. Run directly: python sim/nav/test_wind_independent.py
(about three minutes, most of it the three 12 s multibody batches).
"""
import ast
import math
import sys
import unittest
import warnings
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]/'code'))
import truth as tr  # noqa: E402
from cansat import windprofile as wp  # noqa: E402
from cansat.landing import FLIGHT_MAX_SUBSTEPS, DriftTable, LandingPredictor  # noqa: E402

CODE = HERE.parents[1]/'code'/'cansat'
V_D = 10.5
TAU = V_D/tr.G0
NAN, INF = float('nan'), float('inf')


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def boxcar(x, w):
    c = np.cumsum(np.concatenate((np.zeros((1,) + x.shape[1:]), x)), axis=0)
    return (c[w:] - c[:-w])/w


def lag_descent(profile, dz, z0, z_end, u0, h=0.005, tau=TAU, vd=V_D):
    """du/dt = (w(z(t)) - u)/tau down a frozen profile (n, levels), z = z0 - vd*t.

    Exponential integrator, exact for wind linear in time over each step.
    Returns t, z (steps+1,), u, w (steps+1, n) and the drift integral of u (n,).
    """
    steps = int(round((z0 - z_end)/vd/h))
    t = np.arange(steps + 1)*h
    z = z0 - vd*t
    x = np.clip(z/dz, 0, profile.shape[1] - 1.000001)
    i = x.astype(int)
    f = x - i
    w = ((1 - f)*profile[:, i] + f*profile[:, i + 1]).T
    th = h/tau
    e = math.exp(-th)
    c1 = 1 - (1 - e)/th
    c0 = (1 - e)/th - e
    u = np.empty_like(w)
    u[0] = u0
    drift = np.zeros(profile.shape[0])
    for j in range(steps):
        u[j + 1] = e*u[j] + c0*w[j] + c1*w[j + 1]
        drift += 0.5*h*(u[j] + u[j + 1])
    return t, z, u, w, drift


# --- 1. The physics behind lateral_airspeed ---------------------------------------------------

class LateralForcePhysics(unittest.TestCase):
    """a_xy = -kappa*(R^T v_air)_xy at the centre of mass, tested on truth.simulate. SYNTHETIC."""

    DT = 0.01
    SKIP = 100            # 1 s of start transient left out of every statistic

    @classmethod
    def setUpClass(cls):
        base = tr.Params()
        cls.params = base
        cases = {'restoring': base,
                 'gliding': replace(base, cn_k=0.4, trim_deg=12.0),
                 'swinging': replace(base, cn_k=0.1, side_force_sd=0.05)}
        cls.imu = np.array(base.imu)
        inertia, damping = np.array(base.inertia), np.array(base.rot_damping)
        arm = -base.attach[2]
        m = base.payload_mass
        cls.rate_constant = 0.5*tr.RHO*base.payload_cds*V_D/m
        cls.data = {}
        for i, (name, p) in enumerate(cases.items()):
            o = tr.simulate(p, 16, np.random.default_rng(100 + i), duration=12.0)
            k = len(o['accel_inst'])
            r, v_air, om = o['r'][:k], o['air_velocity'][:k], o['gyro_inst']
            alpha = np.gradient(om, cls.DT, axis=0)
            a_com = o['accel_inst'] - np.cross(alpha, cls.imu) - np.cross(om, np.cross(om, cls.imu))
            v_body = np.einsum('knji,knj->kni', r, v_air)
            kappa = 0.5*tr.RHO*base.payload_cds*np.linalg.norm(v_air, axis=2)/m
            claim = -kappa[..., None]*v_body[..., :2]
            residual = a_com[..., :2] - claim
            # Line direction in body axes from Euler's equation alone (no accelerometer):
            # I*alpha + om x I*om + damping*om = attach x (T*n_b), attach = (0, 0, -arm).
            torque = inertia*alpha + np.cross(om, inertia*om) + damping*om
            tension = np.concatenate((o['tension'][:1], o['tension'][:-1]))     # sample nearest each epoch start
            line_xy = np.stack((-torque[..., 1], torque[..., 0]), axis=-1)/(tension[..., None]*arm)
            cls.data[name] = dict(o=o, k=k, r=r, v_air=v_air, om=om, kappa=kappa, claim=claim, residual=residual,
                                  line_xy=line_xy, line_force=tension[..., None]*line_xy/m, tension=tension,
                                  damping_part=np.stack((-(damping*om)[..., 1], (damping*om)[..., 0]), axis=-1)/(arm*m))

    def test_residual_is_the_line_pulling_off_the_can_axis(self):
        """The whole residual is tension times the line's lateral direction cosine in body axes."""
        s = slice(self.SKIP, None)
        for name, d in self.data.items():
            with self.subTest(name):
                self.assertLess(rms((d['residual'] - d['line_force'])[s]), 0.02*rms(d['residual'][s]))
                angle = math.degrees(math.sqrt(np.mean((d['line_xy'][s]**2).sum(-1))))
                self.assertGreater(angle, 0.5)      # 0.8 to 2.2 degrees rms off the axis
                self.assertLess(angle, 3.0)

    def test_claim_fails_sample_by_sample_and_holds_only_after_averaging(self):
        """Residual as an equivalent airspeed error (m/s) after a boxcar of 0.2, 1 and 2 s."""
        measured = {'restoring': (1.07, 0.186, 0.099), 'gliding': (0.98, 0.417, 0.139), 'swinging': (3.49, 0.869, 0.519)}
        s = slice(self.SKIP, None)
        for name, d in self.data.items():
            with self.subTest(name):
                kappa = float(d['kappa'][s].mean())
                self.assertAlmostEqual(kappa, 0.064, delta=0.003)              # 1/s: 1 m/s sideways is 0.06 m/s^2
                raw = rms(d['residual'][s])/kappa
                self.assertGreater(raw, 1.4)                                    # 1.5 to 3.8 m/s at 100 Hz
                got = [rms(boxcar(d['residual'][s], w))/kappa for w in (20, 100, 200)]
                for value, expect in zip(got, measured[name]):
                    self.assertAlmostEqual(value/expect, 1.0, delta=0.3)
                self.assertTrue(raw > got[0] > got[1] > got[2])
        # 1 s is enough for a canopy that damps its swing, not for one that keeps swinging.
        kappa = float(self.data['swinging']['kappa'][s].mean())
        swinging_1s = rms(boxcar(self.data['swinging']['residual'][s], 100))
        self.assertGreater(swinging_1s, 0.6*rms(boxcar(self.data['swinging']['claim'][s], 100)))
        self.assertGreater(swinging_1s/kappa, 0.6)

    def test_residual_spectrum_is_the_can_pendulum_and_the_system_swing(self):
        s = slice(self.SKIP, None)
        spectra = {}
        for name, d in self.data.items():
            x = d['residual'][s][..., 0]
            x = x - x.mean(axis=0)
            power = (np.abs(np.fft.rfft(x*np.hanning(len(x))[:, None], axis=0))**2).mean(axis=1)
            spectra[name] = (np.fft.rfftfreq(len(x), self.DT), power/power.sum())
        freq, power = spectra['restoring']
        # The can as a rigid pendulum on its attachment: sqrt(T*arm/I)/(2 pi), about 3 Hz.
        d = self.data['restoring']
        pendulum = math.sqrt(float(d['tension'].mean())*-self.params.attach[2]/self.params.inertia[0])/(2*math.pi)
        self.assertAlmostEqual(pendulum, 3.1, delta=0.2)
        self.assertAlmostEqual(freq[np.argmax(power)]/pendulum, 1.0, delta=0.2)
        self.assertGreater(power[freq >= 2.0].sum(), 0.85)
        # A canopy that keeps swinging puts the residual at the swing frequency, which 1 s cannot average.
        freq, power = spectra['swinging']
        self.assertGreater(power[(freq >= 0.5) & (freq < 2.0)].sum(), 0.8)
        # There the larger part is the damping torque on the can, which the line must react.
        d = self.data['swinging']
        averaged = boxcar(d['residual'][s], 100)
        self.assertLess(rms(averaged - boxcar(d['damping_part'][s], 100)), 0.8*rms(averaged))

    def _airspeed(self, name, window, bias=None):
        d = self.data[name]
        o, k = d['o'], d['k']
        weights = np.hanning(window + 2)[1:-1]
        weights = weights/weights.sum()
        inverse = np.linalg.inv(np.swapaxes(d['r'][..., :2, :2], -1, -2))
        line_error = np.einsum('knij,knj->kni', inverse, -d['residual']/d['kappa'][..., None])
        out = {key: [] for key in ('air', 'true', 'line', 'map', 'biased')}
        for i in range(16):
            accel = o['accel_inst'][:, i]
            centres, air, offset_map = wp.lateral_airspeed(accel, d['om'][:, i], d['r'][:, i], o['v'][:k, i, 2], self.DT,
                                                           self.rate_constant, V_D, lever=self.imu, window=window)
            idx = (centres - window//2)[:, None] + np.arange(window)[None, :]
            keep = centres > self.SKIP + window//2
            out['air'].append(air[keep])
            out['true'].append(np.einsum('w,mwi->mi', weights, d['v_air'][:, i, :2][idx])[keep])
            out['line'].append(np.einsum('w,mwi->mi', weights, line_error[:, i][idx])[keep])
            if bias is not None:
                _, shifted, _ = wp.lateral_airspeed(accel + bias, d['om'][:, i], d['r'][:, i], o['v'][:k, i, 2], self.DT,
                                                    self.rate_constant, V_D, lever=self.imu, window=window)
                out['biased'].append((shifted - air)[keep])
                out['map'].append(np.einsum('mij,j->mi', offset_map, bias[:2]/self.rate_constant)[keep])
        return {key: np.concatenate(value) for key, value in out.items() if value}

    def test_lateral_airspeed_is_as_good_as_the_residual_allows(self):
        """Error (m/s rms) of lateral_airspeed against true Hann-averaged horizontal air velocity."""
        limits = {   # window: (error bound, bound on error not explained by the line term)
            'restoring': {100: (0.20, 0.04), 200: (0.10, 0.03)},
            'gliding': {100: (0.65, 0.14), 200: (0.31, 0.11)},
            'swinging': {200: (0.40, 0.30)},
        }
        for name, per_window in limits.items():
            for window, (bound, unexplained) in per_window.items():
                with self.subTest(name=name, window=window):
                    got = self._airspeed(name, window)
                    error = got['air'] - got['true']
                    self.assertLess(rms(error), bound)
                    self.assertLess(rms(error - got['line']), unexplained)
                    self.assertLess(np.abs(error.mean(axis=0)).max(), 0.06)
                    if window == 200:
                        self.assertGreater(np.corrcoef(got['air'].ravel(), got['true'].ravel())[0, 1], 0.9)
        # The default 1 s window on the swinging canopy returns more error than signal.
        got = self._airspeed('swinging', 100)
        self.assertGreater(rms(got['air'] - got['true']), 1.2)
        self.assertGreater(rms(got['air'] - got['true']), 1.5*rms(got['true']))

    def test_offset_map_predicts_a_constant_accelerometer_offset(self):
        bias = np.array([0.03, -0.02, 0.5])          # m/s^2; the z part must not matter
        for name, tolerance in (('restoring', 0.06), ('gliding', 0.06), ('swinging', 0.15)):
            with self.subTest(name):
                got = self._airspeed(name, 100, bias)
                self.assertGreater(rms(got['biased']), 0.3)            # 0.036 m/s^2 is about 0.4 m/s of airspeed
                self.assertLess(rms(got['biased'] - got['map']), tolerance*rms(got['biased']))


# --- 2. Exact Gaussian-process solve ------------------------------------------------------------

class ExactGaussianProcess(unittest.TestCase):
    """retrieve() against a dense solve from the kernel sigma^2 (1 - r/2) exp(-r). SYNTHETIC."""

    SIGMA, SLOPE, LAG, RELEASE, NOISE = 1.3, 0.5, 1.07, 4.0, 0.3
    EPOCHS, DT, Z0 = 41, 0.5, 300.0            # 300 m down to 90 m, all above the 1 m floor

    def exact(self, refine, y):
        h = self.DT/refine
        n = refine*(self.EPOCHS - 1) + 1
        t = np.arange(n)*h
        z = self.Z0 - V_D*t
        zeta = np.log(z)/self.SLOPE
        r = np.abs(zeta[:, None] - zeta[None, :])
        k_ww = self.SIGMA**2*(1 - r/2)*np.exp(-r)
        # u = a @ w + b*u0 from du/dt = (w - u)/lag with w linear between grid points.
        th = h/self.LAG
        e = math.exp(-th)
        c1 = 1 - (1 - e)/th
        c0 = (1 - e)/th - e
        a = np.zeros((n, n))
        b = np.ones(n)
        for j in range(n - 1):
            a[j + 1] = e*a[j]
            a[j + 1, j] += c0
            a[j + 1, j + 1] += c1
            b[j + 1] = e*b[j]
        obs = np.arange(0, n, refine)
        k_yy = a[obs] @ k_ww @ a[obs].T + self.RELEASE**2*np.outer(b[obs], b[obs]) + self.NOISE**2*np.eye(self.EPOCHS)
        k_wy = k_ww[obs] @ a[obs].T
        gain = np.linalg.solve(k_yy, k_wy.T).T
        return t[obs], z[obs], gain @ y, self.SIGMA**2 - np.einsum('ij,ij->i', gain, k_wy)

    def test_posterior_mean_and_variance_converge_to_the_retrieval(self):
        rng = np.random.default_rng(5)
        y = rng.normal(0, 2.0, self.EPOCHS).cumsum()*0.3 + rng.normal(0, 0.3, self.EPOCHS)

        def retrieval(max_dzeta):
            model = wp.WindModel(sigma=self.SIGMA, eddy_slope=self.SLOPE, lag_time=self.LAG, roughness=None,
                                 mean_sd=0.0, release_sd=self.RELEASE, max_dzeta=max_dzeta)
            t, z, _, _ = self.exact(1, y)
            return wp.retrieve(model, t, z, np.column_stack((y, -y)), self.NOISE**2)

        fine = retrieval(0.002)
        self.assertGreater(float(np.abs(fine.wind[:, 0]).max()), 3.0)         # the comparison is not of zeros
        mean_error, var_error = [], []
        for refine in (1, 2, 4, 8, 16):
            _, _, mean, var = self.exact(refine, y)
            mean_error.append(float(np.abs(mean - fine.wind[:, 0]).max()))
            var_error.append(float(np.abs(var - fine.wind_var[:, 0]).max()))
        # Second order in my step: each halving cuts the difference by about four.
        for errors in (mean_error, var_error):
            for coarse, finer in zip(errors[:3], errors[1:4]):
                self.assertGreater(coarse/finer, 3.5)
        self.assertLess(mean_error[-1], 3e-4)          # m/s, on winds of several m/s
        self.assertLess(var_error[-1], 1.2e-4)         # (m/s)^2, on variances of 0.1 to 0.5
        np.testing.assert_allclose(fine.wind[:, 1], -fine.wind[:, 0], atol=1e-9)
        # The default sub-step (max_dzeta = 0.05) costs about 1e-3 m/s.
        _, _, mean, var = self.exact(16, y)
        default = retrieval(0.05)
        self.assertLess(float(np.abs(mean - default.wind[:, 0]).max()), 2e-3)
        self.assertLess(float(np.abs(var - default.wind_var[:, 0]).max()), 5e-4)


# --- 3. Calibration on wind the code did not generate ---------------------------------------------

class HonestyOnTruthWind(unittest.TestCase):
    """Layer means retrieved from truth.make_wind profiles through a lag ODE integrated here. SYNTHETIC."""

    EDGES = np.array([2.0, 25.0, 50.0, 100.0, 200.0, 300.0, 400.0])
    PROFILES = 32

    @classmethod
    def setUpClass(cls):
        rng = np.random.default_rng(11)
        cls.wparams = tr.WindParams()
        wind, _, _ = tr.make_wind(cls.PROFILES, rng, cls.wparams)
        profile = np.concatenate((wind.profile[:, :, 0], wind.profile[:, :, 1]))        # east rows then north rows
        top = int(round(400.0/wind.dz))
        u0 = profile[:, top] + rng.normal(0, 3.0, len(profile))
        t, z, u, w, _ = lag_descent(profile, wind.dz, 400.0, 2.0, u0)
        every = 40                                                                     # 5 Hz
        n = cls.PROFILES
        cls.t, cls.z = t[::every], z[::every]
        cls.wind = np.stack((w[::every, :n], w[::every, n:]), axis=-1)                # (K, n, 2)
        velocity = np.stack((u[::every, :n], u[::every, n:]), axis=-1)
        cls.velocity = velocity + rng.normal(0, 0.3, velocity.shape)

    def scores(self, sigma_factor, profiles):
        wparams = self.wparams
        model = wp.WindModel(sigma=wparams.sigma_h*sigma_factor, eddy_slope=wparams.eddy_slope,
                             eddy_floor=wparams.eddy_floor, lag_time=TAU, roughness=wparams.roughness)
        layer, point, widths = [], [], []
        for i in range(profiles):
            ret = wp.retrieve(model, self.t, self.z, self.velocity[:, i], 0.3**2)
            _, mean, sd, count = ret.layer_means(self.EDGES)
            for j in range(len(self.EDGES) - 1):
                last = j == len(self.EDGES) - 2          # the top layer includes its top edge
                inside = (self.z >= self.EDGES[j]) & ((self.z <= self.EDGES[j + 1]) if last else (self.z < self.EDGES[j + 1]))
                self.assertEqual(count[j], inside.sum())
                layer.append((mean[j] - self.wind[inside, i].mean(axis=0))/sd[j])
            point.append((ret.wind - self.wind[:, i])/np.sqrt(ret.wind_var))
            widths.append(sd)
        return np.array(layer), np.array(point), np.array(widths)

    def test_layer_means_are_covered_when_the_model_matches(self):
        layer, point, widths = self.scores(1.0, self.PROFILES)
        self.assertEqual(layer.size, self.PROFILES*6*2)
        self.assertLess(float(widths.mean()), 0.2)                     # the intervals are tight, not vacuous
        self.assertAlmostEqual(float((layer**2).mean()), 1.0, delta=0.2)          # chi-square per degree of freedom
        self.assertAlmostEqual(float((np.abs(layer) < 1.96).mean()), 0.95, delta=0.04)
        self.assertAlmostEqual(float((point**2).mean()), 1.0, delta=0.15)
        self.assertAlmostEqual(float((np.abs(point) < 1.96).mean()), 0.95, delta=0.03)
        per_layer = (layer**2).reshape(self.PROFILES, 6, 2).mean(axis=(0, 2))
        self.assertTrue(np.all((per_layer > 0.55) & (per_layer < 1.6)), per_layer)

    def test_sigma_wrong_by_two_either_way(self):
        matched, matched_point, _ = self.scores(1.0, 16)
        wide, wide_point, _ = self.scores(2.0, 16)
        narrow, narrow_point, _ = self.scores(0.5, 16)
        # Told twice the turbulence: conservative. Told half: overconfident, worst sample by sample.
        self.assertLess(float((wide_point**2).mean()), 0.6)
        self.assertGreater(float((np.abs(wide) < 1.96).mean()), 0.96)
        self.assertLess(float((wide**2).mean()), float((matched**2).mean()))
        self.assertGreater(float((narrow_point**2).mean()), 2.0)
        self.assertLess(float((np.abs(narrow_point) < 1.96).mean()), 0.85)
        self.assertGreater(float((narrow**2).mean()), 1.25)
        self.assertLess(float((np.abs(narrow) < 1.96).mean()), 0.93)


# --- 4. Drift table and landing predictor ---------------------------------------------------------

class DriftTableMonteCarlo(unittest.TestCase):
    """DriftTable against Monte Carlo over truth.make_wind turbulence and the lag ODE. SYNTHETIC."""

    N = 4000                  # profiles; both horizontal components are used, so 8000 samples
    C0, ROUGH = 4.0, 0.1

    @classmethod
    def setUpClass(cls):
        cls.wparams = tr.WindParams(top=320.0, dz=0.25)
        _, _, turb = tr.make_wind(cls.N, np.random.default_rng(3), cls.wparams)
        grid = np.arange(turb.shape[1])*cls.wparams.dz
        cls.shape = np.log(np.maximum(grid, cls.ROUGH*math.e**0.01)/cls.ROUGH)/math.log(10.0/cls.ROUGH)
        cls.turb = np.concatenate((turb[:, :, 0], turb[:, :, 1]))
        cls.model = wp.WindModel(sigma=1.0, lag_time=TAU, roughness=cls.ROUGH, max_substeps=FLIGHT_MAX_SUBSTEPS)
        cls.table = DriftTable(cls.model, V_D, top=320.0)

    def monte_carlo(self, z0, u0, vd=V_D):
        profile = self.turb + self.C0*self.shape
        _, _, _, _, drift = lag_descent(profile, self.wparams.dz, z0, 0.0, u0, h=0.005 if z0 > 50 else 0.001, vd=vd)
        return drift, self.turb[:, int(round(z0/self.wparams.dz))]

    def predicted(self, z0, u0, scale=1.0):
        row, noise, _ = self.table.lookup(z0)
        row, noise = row*scale, noise*scale*scale
        mean = row[wp.U]*u0 + row[wp.C]*self.C0
        var = 0.25*(row[wp.X1]**2 + row[wp.X2]**2) + noise          # prior x1, x2 ~ N(0, I/4)
        cov_w = 0.25*(row[wp.X1] + math.sqrt(3.0)*row[wp.X2])         # with the wind now, sigma = 1
        return mean, var, cov_w

    def test_prior_drift_mean_variance_and_wind_covariance(self):
        samples = 2*self.N
        for z0, u0 in ((300.0, -3.0), (40.0, -3.0), (7.0, 2.0)):
            with self.subTest(z0=z0):
                drift, wind_now = self.monte_carlo(z0, u0)
                mean, var, cov_w = self.predicted(z0, u0)
                sd = float(drift.std())
                # 0.03 m is the table's own error near the ground (2 m steps, 8 sub-steps).
                self.assertLess(abs(float(drift.mean()) - mean), 4*sd/math.sqrt(samples) + 0.03)
                self.assertLess(abs(sd/math.sqrt(var) - 1), 4/math.sqrt(2*samples) + 0.02)
                mc_cov = float(np.cov(drift, wind_now)[0, 1])
                cov_se = math.sqrt((sd**2*float(wind_now.var()) + mc_cov**2)/samples)
                self.assertLess(abs(mc_cov - cov_w), 4*cov_se + 0.02*abs(cov_w))
                self.assertGreater(abs(cov_w), 10*cov_se)             # the x1, x2 columns are really tested

    def test_rescaled_table_is_close_at_a_different_descent_rate(self):
        """Table built for 10.5 m/s used at 8 m/s as LandingPredictor does: 1 % at 300 m, worse lower down."""
        scale = V_D/8.0
        for z0, mean_tol, sd_tol in ((300.0, 0.02, 0.04), (40.0, 0.12, 0.08)):
            with self.subTest(z0=z0):
                drift, _ = self.monte_carlo(z0, 0.0, vd=8.0)
                mean, var, _ = self.predicted(z0, 0.0, scale)
                self.assertLess(abs(mean/float(drift.mean()) - 1), mean_tol)
                self.assertLess(abs(math.sqrt(var)/float(drift.std()) - 1), sd_tol)
                self.assertLess(mean, float(drift.mean()))            # the scaled table undershoots
                # The shortfall is the lag: (scale - 1)*lag*(wind now), which does not scale with 1/speed.
                expect = (scale - 1)*TAU*self.C0*float(np.interp(z0, np.arange(len(self.shape))*self.wparams.dz, self.shape))
                self.assertAlmostEqual(float(drift.mean()) - mean, expect, delta=0.35*expect + 0.1)

    # The docstring that called this rescaling exact (landing.py) was corrected after this review.


class PredictorFuzz(unittest.TestCase):
    """LandingPredictor.update must never raise or return a non-finite prediction. SYNTHETIC."""

    BASE = dict(t=0.6, position=(1.0, 2.0, 293.7), velocity=(1.0, 2.0), velocity_var=0.09, position_cov=None,
                error_gain=(1.0, 1.0))

    @staticmethod
    def started():
        predictor = LandingPredictor(top=320.0)
        for k in range(3):
            predictor.update(0.2*k, (0.0, 0.0, 300.0 - 2.1*k), (1.0, 2.0), 0.09)
        return predictor

    @staticmethod
    def finite(prediction):
        return (math.isfinite(prediction.east) and math.isfinite(prediction.north)
                and math.isfinite(prediction.time_to_go) and bool(np.all(np.isfinite(prediction.cov)))
                and bool(np.all(np.isfinite(prediction.drift))))

    def outcome(self, change, follow_up=1):
        """'ok', or what went wrong with the fuzzed sample or the good samples after it."""
        predictor = self.started()
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            try:
                out = predictor.update(**{**self.BASE, **change})
            except Exception as error:                      # noqa: BLE001 - the point is that nothing may escape
                return 'raised ' + type(error).__name__
            if out is not None and not self.finite(out):
                return 'non-finite output'
            if out is not None and np.linalg.eigvalsh(out.cov)[0] < -1e-9:
                return 'negative covariance'
            result = None
            for k in range(follow_up):
                try:
                    result = predictor.update(0.8 + 0.2*k, (1.0, 2.0, 291.6 - 2.1*k), (1.0, 2.0), 0.09)
                except Exception as error:                  # noqa: BLE001
                    return 'next raised ' + type(error).__name__
            if result is None:
                return 'no prediction afterwards'
            if not self.finite(result):
                return 'non-finite afterwards'
            if np.abs(result.drift).max() > 1e4:
                return 'absurd drift afterwards'
        return 'ok'

    def failures(self, cases, **kwargs):
        return {name: result for name, result in ((n, self.outcome(c, **kwargs)) for n, c in cases.items()) if result != 'ok'}

    def test_inputs_that_are_handled(self):
        cases = {
            't None': dict(t=None), 't nan': dict(t=NAN), 't inf': dict(t=INF), 't string': dict(t='x'),
            't array': dict(t=np.array([1.0, 2.0])), 't backwards': dict(t=0.1), 't repeated': dict(t=0.4),
            't one ulp later': dict(t=0.4 + 1e-15),
            'position None': dict(position=None), 'position nan': dict(position=(0, 0, NAN)),
            'position inf': dict(position=(INF, 0, 1)), 'position short': dict(position=(0, 0)),
            'position string': dict(position='abc'), 'height negative': dict(position=(0, 0, -50.0)),
            'height zero': dict(position=(0, 0, 0.0)), 'height 1e9': dict(position=(0, 0, 1e9)),
            'east 1e300': dict(position=(1e300, 0, 100.0)), 'position nested': dict(position=[[0, 0, 100.0]]),
            'velocity nan': dict(velocity=(NAN, 1)), 'velocity inf': dict(velocity=(INF, 1)),
            'velocity three': dict(velocity=(1, 2, 3)), 'velocity scalar': dict(velocity=1.0),
            'velocity None inside': dict(velocity=(None, 1.0)), 'velocity None': dict(velocity=None),
            'variance None': dict(velocity_var=None), 'variance zero': dict(velocity_var=0.0),
            'variance negative': dict(velocity_var=-1.0), 'variance nan': dict(velocity_var=NAN),
            'variance inf': dict(velocity_var=INF), 'variance 5e-324': dict(velocity_var=5e-324),
            'variance 1e300': dict(velocity_var=1e300),
            'position_cov nan': dict(position_cov=np.full((2, 2), NAN)), 'position_cov 3x3': dict(position_cov=np.eye(3)),
            'position_cov scalar': dict(position_cov=4.0),
        }
        self.assertEqual(self.failures(cases), {})
        predictor = self.started()
        for bad in (dict(t=NAN), dict(position=None), dict(velocity=(NAN, 1.0)), dict(velocity_var=-1.0)):
            predictor.update(**{**self.BASE, **bad})
        self.assertEqual(predictor.ignored, 4)

    def test_a_whole_descent_and_sitting_on_the_ground_stay_finite_and_positive(self):
        predictor = LandingPredictor(top=320.0)
        rng = np.random.default_rng(2)
        for k in range(260):
            height = 300.0 - 2.1*k + rng.normal(0, 1.5)             # goes below zero and then stays down
            out = predictor.update(0.2*k, (0.0, 0.0, height), rng.normal(0, 3.0, 2), 0.09, np.eye(2))
            self.assertTrue(self.finite(out))
            self.assertGreaterEqual(np.linalg.eigvalsh(out.cov)[0], 0.0)
            out.ellipse()
        self.assertEqual(out.time_to_go, 0.0)
        np.testing.assert_allclose(out.drift, 0.0, atol=1e-12)

    # Was a defect, fixed on 10 October 2026 after this review (landing.py:154-158 and 166-167, windprofile.py:215-218, 264): update() says it never
    # raises on bad data, but only t and position are guarded. A velocity_var of the wrong length,
    # an error_gain that is None, a scalar or too short, and strings or ragged lists for velocity,
    # velocity_var or position_cov all raise out of update(). Two numeric extremes do too
    # (OverflowError from a time of 1e308 after a normal one, and from a height of 1e300).
    def test_never_raises(self):
        cases = {
            'variance three': dict(velocity_var=(1.0, 1.0, 1.0)), 'variance 2x2': dict(velocity_var=np.eye(2) + 1),
            'variance empty': dict(velocity_var=()), 'variance string': dict(velocity_var='x'),
            'velocity string': dict(velocity='ab'), 'velocity ragged': dict(velocity=[[1, 2], [3]]),
            'velocity dict': dict(velocity={'a': 1}),
            'position_cov string': dict(position_cov='x'), 'position_cov ragged': dict(position_cov=[[1, 2], [3]]),
            'error_gain None': dict(error_gain=None), 'error_gain scalar': dict(error_gain=1.0),
            'error_gain short': dict(error_gain=(1.0,)), 'error_gain string': dict(error_gain='ab'),
            't 1e308': dict(t=1e308), 't huge int': dict(t=10**400), 'height 1e300': dict(position=(0, 0, 1e300)),
        }
        self.assertEqual(self.failures(cases), {})

    # Was a defect, fixed on 10 October 2026 after this review (landing.py:157-158, windprofile.py:218): a NaN or inf error_gain goes straight into the
    # measurement matrix and turns the whole covariance to NaN for the rest of the flight, even with
    # error_sd = 0 where the gain should not matter. A finite but absurd velocity (1e200) does the
    # same through overflow. Nothing resets the filter.
    def test_one_bad_sample_never_poisons_the_filter(self):
        cases = {'error_gain nan': dict(error_gain=(NAN, 1.0)), 'error_gain inf': dict(error_gain=(INF, 1.0)),
                 'velocity 1e200': dict(velocity=(1e200, 0.0))}
        self.assertEqual(self.failures(cases), {})

    # Was a defect, fixed on 10 October 2026 after this review (landing.py:148-153): any finite later time is accepted, with no bound on the step. One
    # glitched timestamp 30 s ahead makes every real sample after it "late", so update() returns None
    # until the clock catches up (longer than the descent); a step of 1000 s or more also overflows
    # the matrix exponential (windprofile.py:160-162) and the output is NaN from then on.
    def test_recovers_from_a_timestamp_glitch(self):
        cases = {'t jump 30 s': dict(t=30.6), 't jump 1e4 s': dict(t=1e4), 't jump 1e9 s': dict(t=1e9)}
        self.assertEqual(self.failures(cases, follow_up=10), {})

    # Was a defect, fixed on 10 October 2026 after this review (landing.py:157-158): there is no innovation gate. One finite velocity outlier of 1e6 m/s
    # with a normal variance is believed, and the mean-wind state keeps it: the predicted drift is
    # still 1e7 m after the next good sample.
    def test_a_wild_velocity_is_not_believed(self):
        self.assertEqual(self.failures({'velocity 1e6': dict(velocity=(1e6, 0.0))}), {})

    # Was a defect, fixed on 10 October 2026 after this review (landing.py:166-169): position_cov is only checked for shape and finiteness. A negative
    # one makes the returned covariance indefinite and Prediction.ellipse()/fill() then raise
    # ValueError in the telemetry path.
    def test_ellipse_survives_a_bad_position_covariance(self):
        out = self.started().update(**{**self.BASE, 'position_cov': -1e6*np.eye(2)})
        out.ellipse()

    def test_long_gaps_keep_the_step_finite_and_positive(self):
        """The Van Loan step itself is sound to at least 300 s; it overflows somewhere before 1e4 s."""
        model = wp.WindModel(max_substeps=FLIGHT_MAX_SUBSTEPS)
        for dt, z_to in ((1.0, 289.5), (10.0, 195.0), (60.0, 0.0), (300.0, 0.0)):
            phi, q = wp.axis_step(model, 300.0, z_to, dt)
            self.assertTrue(np.all(np.isfinite(phi)) and np.all(np.isfinite(q)))
            self.assertGreater(np.linalg.eigvalsh(q)[0], -1e-12)
        _, q = wp.axis_step(model, 300.0, 0.0, 60.0)
        np.testing.assert_allclose(np.diag(q)[:2], 0.25, atol=1e-3)       # fully decorrelated turbulence
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            _, q = wp.axis_step(model, 300.0, 0.0, 1e4)
        self.assertFalse(np.all(np.isfinite(q)))


# --- 5. Portability guard ---------------------------------------------------------------------------

class Portability(unittest.TestCase):
    """Static check that the flight modules use only long-standing NumPy and Python 3.11 syntax.

    This machine has NumPy 2 and Python 3.11, so nothing here RUNS on NumPy 1.24;
    the guard only shows that no name newer than NumPy 1.24 is referenced.
    """

    IN_NUMPY_1_24 = {
        'abs', 'all', 'any', 'arange', 'array', 'asarray', 'broadcast_to', 'column_stack', 'concatenate', 'convolve',
        'cross', 'diag', 'diff', 'ones', 'linalg.eigh', 'linalg.eigvalsh', 'shape',
        'einsum', 'empty', 'empty_like', 'eye', 'flatnonzero', 'full', 'gradient', 'hanning', 'isfinite', 'ix_',
        'linalg.inv', 'linalg.slogdet', 'linalg.solve', 'linspace', 'log', 'maximum', 'nan', 'ndarray', 'ones_like',
        'outer', 'repeat', 'sort', 'sqrt', 'stack', 'swapaxes', 'where', 'zeros', 'zeros_like'}

    def test_numpy_names_and_syntax(self):
        for name in ('windprofile.py', 'landing.py'):
            source = (CODE/name).read_text(encoding='utf-8')
            tree = ast.parse(source, feature_version=(3, 11))
            used = set()
            for node in ast.walk(tree):
                if isinstance(node, ast.Attribute):
                    parts = []
                    while isinstance(node, ast.Attribute):
                        parts.append(node.attr)
                        node = node.value
                    if isinstance(node, ast.Name) and node.id == 'np':
                        used.add('.'.join(reversed(parts)))
            used = {u for u in used if not any(other != u and other.startswith(u + '.') for other in used)}
            self.assertEqual(used - self.IN_NUMPY_1_24, set(), name)
            imports = {n.module if isinstance(n, ast.ImportFrom) else a.name
                       for n in ast.walk(tree) if isinstance(n, (ast.Import, ast.ImportFrom))
                       for a in (n.names if isinstance(n, ast.Import) else [None])}
            self.assertLessEqual(imports, {'math', 'dataclasses', 'numpy', 'telemetry', 'windprofile'}, name)

    def test_flight_update_does_not_grow_with_flight_time(self):
        predictor = LandingPredictor(top=320.0)
        sizes = []
        for k in range(600):                                   # two minutes at 5 Hz, hovering at 250 m
            predictor.update(0.2*k, (0.0, 0.0, 250.0), (1.0, 2.0), 0.09)
            if k in (10, 599):
                sizes.append({key: np.size(value) for key, value in vars(predictor.filter).items() if key != 'model'}
                             | {key: np.size(value) for key, value in vars(predictor).items()
                                if key not in ('filter', 'table', 'model', 'last')})
        self.assertEqual(sizes[0], sizes[1])
        self.assertEqual(predictor.filter.cov.shape, (wp.N_STATE, wp.N_STATE))


if __name__ == '__main__':
    unittest.main(verbosity=2)
