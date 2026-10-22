"""Independent deterministic checks for the physics LaTeX derivations.

Standard-library only; synthetic inputs, never measured flight data.
Run with unittest discovery from sim/maths. These calculations verify the
conditional mathematics and expose counterexamples, not real-canopy validity.
"""

import math
from pathlib import Path
import unittest


G = 9.80665
VD = 10.5


def psi_m(zeta):
    """Dyer integrated momentum correction normalized to psi(0)=0."""
    if zeta >= 0:
        return -5 * zeta
    x = (1 - 16 * zeta) ** 0.25
    return (
        2 * math.log((1 + x) / 2)
        + math.log((1 + x * x) / 2)
        - 2 * math.atan(x)
        + math.pi / 2
    )


def phi_m(zeta):
    return 1 + 5 * zeta if zeta >= 0 else (1 - 16 * zeta) ** -0.25


def composite_simpson(fun, a, b, intervals=10000):
    if intervals <= 0 or intervals % 2:
        raise ValueError("An even, positive number of intervals is required")
    h = (b - a) / intervals
    interior = sum(
        (4 if j % 2 else 2) * fun(a + j * h)
        for j in range(1, intervals)
    )
    return h * (fun(a) + fun(b) + interior) / 3


def rk4_step(fun, t, u, dt):
    a = fun(t, u)
    b = fun(t + dt / 2, u + dt * a / 2)
    c = fun(t + dt / 2, u + dt * b / 2)
    d = fun(t + dt, u + dt * c)
    return u + dt * (a + 2 * b + 2 * c + d) / 6


def lora_airtime(payload, sf, bandwidth, coding_rate, crc, implicit, de, preamble):
    blocks = math.ceil(
        (8 * payload - 4 * sf + 28 + 16 * crc - 20 * implicit)
        / (4 * (sf - 2 * de))
    )
    n_payload = 8 + max(blocks * (coding_rate + 4), 0)
    return (preamble + 4.25 + n_payload) * 2**sf / bandwidth


class PhysicsIndependentChecks(unittest.TestCase):
    def test_constant_horizontal_lag_against_rk4(self):
        """Integrate Newton's reduced equation and compare exact step response."""
        for ratio in (0.5, 1.0, 2.0):
            with self.subTest(lateral_coefficient_ratio=ratio):
                tau = VD / (ratio * G)
                u, dt, max_error = 0.0, 0.001, 0.0
                fun = lambda t, state: (5 - state) / tau
                for j in range(5000):
                    u = rk4_step(fun, j * dt, u, dt)
                    exact = 5 * (1 - math.exp(-(j + 1) * dt / tau))
                    max_error = max(max_error, abs(u - exact))
                self.assertLess(max_error, 2e-12)
        self.assertAlmostEqual(VD / G, 1.0707020236268248, places=14)
        self.assertAlmostEqual(VD * VD / G, 11.242371248081659, places=13)

    def test_shear_path_and_lag_are_distinct(self):
        shear = 0.05
        displacement = shear * 100**2 / (2 * VD)
        self.assertAlmostEqual(displacement, 23.80952380952381, places=12)
        self.assertGreater(displacement / 100, 0.23)
        self.assertAlmostEqual(shear * 10**2 / (2 * VD), 0.23809523809523808)
        tau = VD / G
        # Independent quadrature of U(z(t'))-U(z(t)), with U=U0+S*z.
        duration, initial_height, u0 = 100 / VD, 300.0, 5.0
        wind = lambda t: u0 + shear * (initial_height - VD * t)
        numerical = composite_simpson(
            lambda t: wind(t) - wind(duration), 0, duration, 1000
        )
        self.assertAlmostEqual(numerical, displacement, places=11)
        # Verify the transient solution solves tau*u_dot+u=U(t).
        initial_velocity = 2.0
        for t in (0.0, 0.5, 2.0, 10.0):
            transient = initial_velocity - wind(0) - tau * shear * VD
            u = wind(t) + tau * shear * VD + transient * math.exp(-t / tau)
            du = -shear * VD - transient / tau * math.exp(-t / tau)
            self.assertAlmostEqual(tau * du + u, wind(t), places=12)

    def test_hypsometric_forward_inverse(self):
        rd, temperature, p0 = 287.05, 288.15, 101325.0
        for height in (0.0, 20.0, 300.0, 400.0):
            pressure = p0 * math.exp(-G * height / (rd * temperature))
            recovered = rd * temperature / G * math.log(p0 / pressure)
            self.assertAlmostEqual(recovered, height, places=10)

    def test_dry_adiabatic_potential_temperature_invariant(self):
        rd, cp, theta, p_ref = 287.05, 1004.0, 290.0, 100000.0
        for pressure in (98000.0, 101000.0, 103000.0):
            t = theta * (pressure / p_ref) ** (rd / cp)
            recovered = t * (p_ref / pressure) ** (rd / cp)
            self.assertAlmostEqual(recovered, theta, places=11)
        self.assertAlmostEqual(300 * G / cp, 2.9302739043824704, places=12)

    def test_most_integral_and_derivative(self):
        self.assertEqual(psi_m(0.0), 0.0)
        for zeta in (-2.0, -0.1, 0.1, 2.0):
            with self.subTest(zeta=zeta):
                eps = 1e-6
                derivative = (psi_m(zeta + eps) - psi_m(zeta - eps)) / (2 * eps)
                self.assertLess(abs(1 - zeta * derivative - phi_m(zeta)), 2e-9)
                obukhov = 30 if zeta > 0 else -30
                z0, height = 0.1, zeta * obukhov
                quadrature = composite_simpson(
                    lambda q: phi_m(math.exp(q) / obukhov),
                    math.log(z0), math.log(height),
                )
                closed = (
                    math.log(height / z0) - psi_m(height / obukhov)
                    + psi_m(z0 / obukhov)
                )
                self.assertLess(abs(quadrature - closed), 5e-13)

    def test_most_lower_boundary_and_neutral_limit(self):
        z0, displacement, ustar, kappa = 0.1, 2.0, 0.3, 0.4
        for obukhov in (-30.0, 30.0):
            def profile(z):
                h = z - displacement
                return ustar / kappa * (
                    math.log(h / z0) - psi_m(h / obukhov)
                    + psi_m(z0 / obukhov)
                )
            self.assertAlmostEqual(profile(displacement + z0), 0.0, places=12)
        # This limit check is independent of the quadrature/gradient tests.
        h, obukhov = 20.0, 1e12
        corrected = math.log(h / z0) - psi_m(h / obukhov) + psi_m(z0 / obukhov)
        self.assertLess(abs(corrected - math.log(h / z0)), 1.1e-10)

    def test_pendulum_constant_forcing_counterexample(self):
        length, tilt = 1.5, 0.1
        omega_squared = G / length
        forcing = omega_squared * tilt
        angular_velocity = angular_acceleration = 0.0
        # Nonzero equilibrium tilt with zero gyro and gyro derivative.
        residual = angular_acceleration + omega_squared * tilt - forcing
        self.assertEqual(residual, 0.0)
        self.assertNotEqual(tilt, -angular_acceleration / omega_squared)
        self.assertEqual(angular_velocity, 0.0)

    def test_lora_packet_example(self):
        airtime = lora_airtime(40, 7, 125000, 1, 1, 0, 0, 8)
        self.assertAlmostEqual(airtime, 0.082176, places=14)
        self.assertAlmostEqual(100 * airtime, 8.2176, places=12)

    def test_storage_and_corner_blur(self):
        rate = 1280 * 800 * 5
        self.assertEqual(rate, 5120000)
        self.assertEqual(rate * 30, 153600000)
        self.assertEqual(rate * 60, 307200000)
        focal, x, y, exposure = 640, 1.0, 0.625, 0.001
        pitch = math.radians(40)
        pitch_blur = focal * pitch * math.hypot(1 + x * x, x * y) * exposure
        spin_blur = math.hypot(640, 400) * 2 * math.pi * exposure
        self.assertAlmostEqual(pitch_blur, 0.9362255862723828, places=13)
        self.assertAlmostEqual(spin_blur, 4.7420361309718, places=12)

    def test_tex_has_no_embedded_control_bytes(self):
        fragment = Path(__file__).with_name("physics_fragment.tex").read_bytes()
        self.assertFalse([(j, c) for j, c in enumerate(fragment) if c < 32 and c != 10])
        for name, count in (("pre", 3), ("ref", 1), ("cross", 2), ("stall", 2)):
            self.assertEqual(fragment.count((r"\mathrm{" + name + "}").encode()), count)


if __name__ == "__main__":
    unittest.main()
