"""GPL-3.0-only. Synthetic one-dimensional flight for exercising flight software.

A test stimulus, not a validated rocket or canopy model. The launch profile is
shaped to the competition's stated numbers: 10-20 g for 0.5-1.5 s, a 5-7 s
coast and a 300-400 m apogee. Everything here is simulated; nothing is measured.
"""
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cansat.atmosphere import G0, pressure_at  # noqa: E402


def simulate(seed=0, dt=0.01, pad_s=20.0, boost_g=14.0, burnout_speed=85.0, drag_k=0.001,
             eject_delay_s=0.3, eject_spike_g=8.0, eject_pressure_pa=1500.0, inflation_s=0.8,
             descent_speed=10.5, landing_elev_m=0.0, touchdown_spike_g=15.0, rest_s=10.0,
             drift_pa_per_s=0.0, baro_sigma_pa=3.0, baro_lag_s=0.1, accel_sigma=0.5,
             accel_limit_g=16.0, bumps=(), pad_climb_rate=0.0, launch=True,
             accel_ok=True, baro_ok=True, p0=101325.0, temp_k=283.15):
    """Returns ([(t, accel_norm, pressure)], truth).

    bumps: (start_s, duration_s, g) handling shocks while on the pad.
    pad_climb_rate: m/s of slow height gain on the pad (carried uphill).
    """
    rng = random.Random(seed)
    samples, truth = [], {}
    t, h, v = 0.0, 0.0, 0.0
    phase = 'pad'
    phase_t = 0.0
    baro = None
    transient = 0.0
    k_can = G0/35.0**2
    k_canopy = G0/descent_speed**2

    while True:
        spike = 0.0
        if phase == 'pad':
            h += pad_climb_rate*dt
            force = G0
            for start, duration, g in bumps:
                if start <= t < start + duration:
                    force = g*G0
            if t >= pad_s:
                if not launch:
                    break
                phase, truth['t_liftoff'] = 'boost', t
        elif phase == 'boost':
            force = boost_g*G0
            v += (force - G0 - drag_k*v*v)*dt
            if v >= burnout_speed:
                phase, truth['t_burnout'] = 'coast', t
        elif phase == 'coast':
            force = drag_k*v*v
            v += (-G0 - drag_k*v*abs(v))*dt
            if v <= 0 and 't_apogee' not in truth:
                truth['t_apogee'], truth['apogee_m'] = t, h
            if 't_apogee' in truth and t - truth['t_apogee'] >= eject_delay_s:
                phase, phase_t, truth['t_eject'] = 'tumble', t, t
                transient = eject_pressure_pa
        elif phase == 'tumble':
            force = k_can*v*v
            if t - phase_t < 0.03:
                spike = eject_spike_g*G0
            v += (-G0 + k_can*v*v)*dt
            if t - phase_t >= inflation_s:
                phase = 'canopy'
        elif phase == 'canopy':
            force = k_canopy*v*v + 0.3*G0*math.sin(2*math.pi*t)
            v += (-G0 + k_canopy*v*v)*dt
            if h <= landing_elev_m:
                phase, phase_t, truth['t_touchdown'] = 'rest', t, t
                v, h = 0.0, landing_elev_m
        else:
            force = G0
            if t - phase_t < 0.02:
                spike = touchdown_spike_g*G0
            if t - phase_t >= rest_s:
                break
        if phase not in ('pad', 'rest'):
            h += v*dt

        accel = min(abs(max(force, spike) + rng.gauss(0, accel_sigma)), accel_limit_g*G0)
        true_p = pressure_at(h, p0 + drift_pa_per_s*t, temp_k) + transient
        transient *= math.exp(-dt/0.03)
        baro = true_p if baro is None else baro + (1 - math.exp(-dt/baro_lag_s))*(true_p - baro)
        samples.append((t, accel if accel_ok else None,
                        baro + rng.gauss(0, baro_sigma_pa) if baro_ok else None))
        t += dt
    return samples, truth


def random_flight(seed, **overrides):
    """Draws launch and recovery parameters across the stated envelope."""
    rng = random.Random(seed)
    params = dict(
        seed=seed, boost_g=rng.uniform(10, 20), burnout_speed=rng.uniform(80, 95),
        drag_k=rng.uniform(0.0006, 0.0012), eject_delay_s=rng.uniform(0.0, 1.0),
        eject_spike_g=rng.uniform(4, 15), eject_pressure_pa=rng.uniform(0, 3000),
        inflation_s=rng.uniform(0.3, 1.5), descent_speed=rng.uniform(9, 15),
        landing_elev_m=rng.uniform(-15, 15), touchdown_spike_g=rng.uniform(6, 16),
        drift_pa_per_s=rng.uniform(-0.03, 0.03), baro_sigma_pa=rng.uniform(2, 8),
        pad_s=rng.uniform(10, 40))
    params.update(overrides)
    return simulate(**params)
