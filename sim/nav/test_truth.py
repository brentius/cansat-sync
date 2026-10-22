"""GPL-3.0-only. Checks of the synthetic descent model against physics it must obey."""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]/'code'))
sys.path.insert(0, str(HERE.parents[0]/'maths'))
import truth as tr  # noqa: E402
from cansat.eskf import Eskf, ImuNoise, N_ERR  # noqa: E402
from models import transverse  # noqa: E402

CALM = tr.WindParams(speed_10m=0.0, sigma_h=0.0, sigma_w=0.0)
QUIET = dict(spin_torque_sd=0.0, side_force_sd=0.0)
COARSE = dict(dt=0.002, **QUIET)        # ample for the 2 Hz swing modes; halves test time


def run(sim, seconds, rng=None):
    for _ in range(round(seconds/sim.p.dt)):
        sim.step(rng)


def energy(sim):
    """Mechanical energy. A very heavy canopy still takes up p^2/2M, so it is counted."""
    p = sim.p
    return (0.5*sim.canopy_inertial*(sim.vc**2).sum(1) + 0.5*p.payload_mass*(sim.vp**2).sum(1)
            + 0.5*(sim.inertia*sim.om**2).sum(1) + p.canopy_mass*tr.G0*sim.pc[:, 2]
            + p.payload_mass*tr.G0*sim.pp[:, 2])


def vertical_angular_momentum(sim):
    """About the vertical through the (fixed) canopy point."""
    p = sim.p
    spin = np.einsum('nij,nj->ni', tr.quat_matrix(sim.q), sim.inertia*sim.om)
    return (p.payload_mass*tr.cross(sim.pp - sim.pc, sim.vp) + spin)[:, 2]


class Dynamics(unittest.TestCase):
    def test_conservation_on_a_fixed_pivot(self):
        # Immovable weightless canopy, no air, no damping: the can tumbling
        # and spinning on its line conserves energy and angular momentum about
        # the vertical through the pivot. Exercises the tension solve, the
        # rigid-body equations and the integrator together.
        params = replace(tr.Params(), canopy_mass=0.0, canopy_added_mass=1e9, descent_speed=1e5,
                         payload_cds=0.0, rot_damping=(0.0, 0.0, 0.0), **QUIET)
        rng = np.random.default_rng(1)
        wind, _, _ = tr.make_wind(6, rng, CALM)
        sim = tr.Descent(params, wind, 6)
        sim.start(rng, tilt_sd_deg=25.0, rate_sd_dps=120.0, spin_sd_dps=200.0)
        self.assertLess(abs(sim.canopy_cds), 1e-8)
        # Rigid rotation about the canopy point: canopy at rest, line taut.
        sim.vp = sim.vp - sim.vc
        sim.vc = np.zeros_like(sim.vc)
        e0, l0 = energy(sim), vertical_angular_momentum(sim)
        scale = 0.5*params.payload_mass*(sim.vp**2).sum(1) + 0.5*(sim.inertia*sim.om**2).sum(1)
        worst_line, swing, slack = 0.0, 0.0, False
        for _ in range(3000):
            sim.step()
            worst_line = max(worst_line, np.max(abs(sim.line_length() - params.line)))
            swing = max(swing, np.max(sim.swing_angle()))
            slack |= bool(np.any(sim.tension <= 0))
        self.assertFalse(slack)
        self.assertGreater(swing, math.radians(30))                   # a vigorous case
        self.assertLess(np.max(abs(energy(sim) - e0)/scale), 1e-8)
        self.assertLess(np.max(abs(vertical_angular_momentum(sim) - l0)/abs(l0)), 1e-8)
        self.assertLess(worst_line, 1e-8)

    def test_simple_pendulum_limit(self):
        # Line attached at the centre of mass and an immovable weightless
        # canopy: a simple pendulum of the line length.
        params = replace(tr.Params(), attach=(0.0, 0.0, 0.0), canopy_mass=0.0, canopy_added_mass=1e9,
                         descent_speed=1e5, payload_cds=0.0, line=0.8, **QUIET)
        rng = np.random.default_rng(2)
        wind, _, _ = tr.make_wind(1, rng, CALM)
        sim = tr.Descent(params, wind, 1)
        sim.start(rng, tilt_sd_deg=0.0, rate_sd_dps=0.0, spin_sd_dps=0.0, lateral_sd=0.0, position_sd=0.0)
        theta = math.radians(4.0)
        sim.pc = np.array([[0.0, 0.0, 300.0]])
        sim.pp = sim.pc + 0.8*np.array([[math.sin(theta), 0.0, -math.cos(theta)]])
        sim.vc[:] = 0.0
        sim.vp[:] = 0.0
        crossings, previous = [], sim.pp[0, 0]
        for _ in range(6000):
            sim.step()
            x = sim.pp[0, 0] - sim.pc[0, 0]
            if previous > 0 >= x:
                crossings.append(sim.t - sim.p.dt*(0 - x)/(previous - x))
            previous = x
        period = np.mean(np.diff(crossings))
        expected = 2*math.pi*math.sqrt(0.8/tr.G0)*(1 + theta**2/16)
        self.assertAlmostEqual(period/expected, 1.0, delta=2e-4)

    def test_terminal_speed_and_isotropic_drag_limit(self):
        params = replace(tr.Params(), **COARSE)
        rng = np.random.default_rng(3)
        wind, _, _ = tr.make_wind(4, rng, CALM)
        sim = tr.Descent(params, wind, 4)
        sim.start(rng, tilt_sd_deg=5.0, lateral_sd=3.0)
        run(sim, 9.0)
        np.testing.assert_allclose(sim.vp[:, 2], -params.descent_speed, rtol=2e-3)
        # Lateral motion relative to still air has died away and the can hangs plumb.
        self.assertLess(np.max(tr.norm(sim.vp[:, :2])), 0.02)
        self.assertLess(np.max(sim.swing_angle()), math.radians(0.2))

    def test_vertical_release_matches_one_dimensional_solution(self):
        # Released from rest hanging plumb in still air: m_inertial dv/dt = W - k v^2.
        params = replace(tr.Params(), **QUIET)
        rng = np.random.default_rng(4)
        wind, _, _ = tr.make_wind(1, rng, CALM)
        sim = tr.Descent(params, wind, 1)
        sim.start(rng, tilt_sd_deg=0.0, rate_sd_dps=0.0, spin_sd_dps=0.0, lateral_sd=0.0)
        sim.vp[:] = 0.0
        sim.vc[:] = 0.0
        run(sim, 2.0)
        weight = (params.payload_mass + params.canopy_mass)*tr.G0
        inertial = params.payload_mass + params.canopy_mass + params.canopy_added_mass
        k = 0.5*tr.RHO*(sim.canopy_cds + params.payload_cds)
        v, h = 0.0, 1e-5
        for _ in range(200000):
            v += h*(weight - k*v*v)/inertial
        self.assertAlmostEqual(-sim.vp[0, 2], v, delta=2e-4)

    def test_unstable_canopy_settles_near_its_trim_angle(self):
        params = replace(tr.Params(), cn_k=0.4, trim_deg=15.0, **COARSE)
        rng = np.random.default_rng(5)
        wind, _, _ = tr.make_wind(8, rng, CALM)
        sim = tr.Descent(params, wind, 8)
        sim.start(rng, tilt_sd_deg=3.0)
        run(sim, 10.0)
        angles = []
        for _ in range(1500):
            sim.step()
            rel = sim.vc
            d = sim.pc - sim.pp - tr.quat_matrix(sim.q) @ sim.attach
            n = d/tr.norm(d)[:, None]
            axial = (rel*n).sum(1)
            angles.append(np.degrees(np.arctan2(tr.norm(rel - axial[:, None]*n), -axial)))
        mean_alpha = np.mean(angles, axis=0)
        self.assertTrue(np.all(mean_alpha > 8.0) and np.all(mean_alpha < 22.0), mean_alpha)
        # A stable canopy under the same start has no sustained angle of attack.
        stable = tr.Descent(replace(tr.Params(), **COARSE), wind, 8)
        stable.start(np.random.default_rng(5), tilt_sd_deg=3.0)
        run(stable, 10.0)
        self.assertLess(np.max(tr.norm(stable.vc[:, :2])), 0.05)


class Sensors(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = replace(tr.Params(), cn_k=0.4, trim_deg=12.0)
        cls.data = tr.simulate(cls.params, 3, np.random.default_rng(6), duration=10.0,
                               start=dict(spin_sd_dps=200.0))

    def strapdown(self, accel, gyro, j):
        d = self.data
        f = Eskf(d['p'][0, j], d['v'][0, j], d['r'][0, j], np.eye(N_ERR), ImuNoise(0, 0, 0, 0, 0))
        for k in range(len(accel)):
            f.propagate(0.01, accel[k, j], gyro[k, j])
        return (np.linalg.norm(f.p - d['p'][-1, j]), np.linalg.norm(f.v - d['v'][-1, j]),
                np.linalg.norm(f.r - d['r'][-1, j]))

    def test_ideal_increments_reproduce_the_trajectory(self):
        d = self.data
        self.assertGreater(np.degrees(np.max(d['swing'])), 5.0)       # the case is not trivial
        for j in range(3):
            dp, dv, dr = self.strapdown(d['accel'], d['gyro'], j)
            self.assertLess(dv, 1e-8)
            self.assertLess(dr, 1e-9)
            self.assertLess(dp, 5e-3)

    def test_uncompensated_averaging_is_measurably_worse(self):
        d = self.data
        ideal = np.array([self.strapdown(d['accel'], d['gyro'], j) for j in range(3)])
        mean = np.array([self.strapdown(d['accel_mean'], d['gyro_mean'], j) for j in range(3)])
        inst = np.array([self.strapdown(d['accel_inst'], d['gyro_inst'], j) for j in range(3)])
        self.assertTrue(np.all(mean[:, 1] > 100*ideal[:, 1]))
        self.assertTrue(np.all(inst[:, 0].mean() > mean[:, 0].mean()))

    def test_specific_force_is_gravity_when_hanging_steadily(self):
        params = replace(tr.Params(), **COARSE)
        rng = np.random.default_rng(7)
        wind, _, _ = tr.make_wind(2, rng, CALM)
        sim = tr.Descent(params, wind, 2)
        sim.start(rng, tilt_sd_deg=0.0, rate_sd_dps=0.0, spin_sd_dps=0.0, lateral_sd=0.0)
        run(sim, 6.0)
        force, rate = sim.step()
        # Body z is down, so supporting the can reads -g on z.
        np.testing.assert_allclose(force, np.tile([0.0, 0.0, -tr.G0], (2, 1)), atol=2e-3)
        np.testing.assert_allclose(rate, 0.0, atol=1e-6)

    def test_camera_delay_shifts_the_frame_pose(self):
        a = tr.simulate(self.params, 2, np.random.default_rng(8), duration=2.0)
        b = tr.simulate(self.params, 2, np.random.default_rng(8), duration=2.0, camera_delay=0.01)
        # With a 10 ms delay, frame f is the pose one navigation step later.
        np.testing.assert_allclose(b['frame_p'][3], a['p'][61], atol=1e-12)
        np.testing.assert_allclose(a['frame_p'][3], a['p'][60], atol=1e-12)
        np.testing.assert_allclose(b['frame_r'][3], a['r'][61], atol=1e-12)


class WindModel(unittest.TestCase):
    def test_turbulence_follows_the_transverse_kernel_in_scaled_height(self):
        wp = tr.WindParams()
        rng = np.random.default_rng(9)
        n = 6000
        _, _, turb = tr.make_wind(n, rng, wp)
        heights = np.array([10.0, 20.0, 40.0, 80.0, 160.0, 300.0])
        idx = np.round(heights/wp.dz).astype(int)
        zeta = np.log(heights)/wp.eddy_slope
        expected = wp.sigma_h**2*transverse(zeta[:, None] - zeta[None, :])
        for axis in range(2):
            sample = np.cov(turb[:, idx, axis].T, bias=True)
            np.testing.assert_allclose(sample, expected, atol=5/math.sqrt(n))
        # Ratio scaling: 10 to 20 m correlates like 80 to 160 m.
        self.assertAlmostEqual(expected[0, 1], expected[3, 4], places=12)
        np.testing.assert_allclose(turb[:, idx, 2].var(axis=0), wp.sigma_w**2, rtol=0.08)

    def test_mean_wind_is_the_log_law(self):
        wp = tr.WindParams()
        wind, mean, _ = tr.make_wind(3, np.random.default_rng(10), wp, direction=0.0)
        np.testing.assert_allclose(mean[:, 20, 0], 5.0, rtol=1e-12)                  # 10 m
        np.testing.assert_allclose(mean[:, 200, 0], 5.0*math.log(1000)/math.log(100), rtol=1e-12)
        np.testing.assert_allclose(wind(np.array([10.25, 10.25, 10.25])),
                                   (wind.profile[:, 20] + wind.profile[:, 21])/2, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
