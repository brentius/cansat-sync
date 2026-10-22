# SPDX-License-Identifier: GPL-3.0-only
"""Offline first-order thermal step identification from a reference/probe CSV.

This is a descriptive model fit for planning air-step tests. It does not qualify
the probe or infer its behavior outside the supplied record.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

TAU_BOUNDS_S = (0.02, 20.0)


def _arrays(time_s, reference_c, probe_c):
    t = np.asarray(time_s, dtype=float)
    u = np.asarray(reference_c, dtype=float)
    y = np.asarray(probe_c, dtype=float)
    if t.ndim != 1 or u.ndim != 1 or y.ndim != 1 or not (len(t) == len(u) == len(y)):
        raise ValueError('time_s, reference_c and probe_c must be equal-length 1-D arrays')
    if len(t) < 8:
        raise ValueError('at least 8 samples are required')
    if not (np.isfinite(t).all() and np.isfinite(u).all() and np.isfinite(y).all()):
        raise ValueError('input contains NaN or infinite values')
    if np.any(np.diff(t) <= 0):
        raise ValueError('timestamps must be strictly increasing')
    return t, u, y


def _filtered_zoh(t, u, tau, step_time_s):
    """Exact ZOH response, splitting an interval that contains step time."""
    x = np.empty_like(u)
    x[0] = u[0]
    for k, (t0, t1) in enumerate(zip(t[:-1], t[1:]), start=1):
        if t0 < step_time_s < t1:
            before = math.exp(-(step_time_s - t0)/tau)
            at_step = before*x[k - 1] + (1.0 - before)*u[k - 1]
            after = math.exp(-(t1 - step_time_s)/tau)
            x[k] = after*at_step + (1.0 - after)*u[k]
        else:
            decay = math.exp(-(t1 - t0)/tau)
            x[k] = decay*x[k - 1] + (1.0 - decay)*u[k - 1]
    return x


def _profile(t, u, y, step_time_s, log_tau):
    tau = math.exp(float(log_tau))
    filtered = _filtered_zoh(t, u, tau, step_time_s)
    transient = np.exp(-(t - t[0])/tau)
    design = np.column_stack((np.ones(len(t)), filtered, transient))
    # Scale columns before conditioning diagnostics; OLS itself is unscaled and
    # uses SVD via lstsq to remain stable near collinearity.
    norms = np.linalg.norm(design, axis=0)
    scaled = design / np.where(norms > 0, norms, 1.0)
    coeff, _, rank, _ = np.linalg.lstsq(design, y, rcond=None)
    residual = y - design @ coeff
    sse = float(residual @ residual)
    cond = float(np.linalg.cond(scaled)) if rank == design.shape[1] else math.inf
    return sse, tau, coeff, residual, cond, int(rank)


def _lag1(residual):
    x = np.asarray(residual, dtype=float)
    x = x - x.mean()
    den = float(x @ x)
    return float(x[1:] @ x[:-1] / den) if den > 0 else 0.0


def fit_step(time_s, reference_c, probe_c, step_time_s):
    """Fit offset + gain*filtered(reference) + initial transient.

    The filter assumes zero-order-hold reference values over each measured
    interval, including irregular intervals. `step_time_s` is the known time
    of the reference air-step; if it falls between samples, the bracketed
    interval is split there. Offset, gain and transient amplitude are profiled
    by linear least squares for each candidate tau. This assumption is not
    appropriate when the reference step time is unknown or the actual input
    changes substantially between samples.
    """
    t, u, y = _arrays(time_s, reference_c, probe_c)
    step_time_s = float(step_time_s)
    if not math.isfinite(step_time_s) or not (t[0] < step_time_s < t[-1]):
        raise ValueError('step time must lie strictly inside the timestamp range')
    pre, post = t < step_time_s, t >= step_time_s
    if pre.sum() < 3 or post.sum() < 3:
        raise ValueError('at least 3 samples are required on each side of step time')
    excursion = float(np.ptp(u))
    input_delta = abs(float(np.median(u[post]) - np.median(u[pre])))
    excitation_floor = max(1e-6, 1e-6*max(1.0, float(np.max(np.abs(u)))))
    if excursion <= excitation_floor or input_delta <= excitation_floor:
        raise ValueError('reference has no usable air-step excitation across step time')

    lo, hi = map(math.log, TAU_BOUNDS_S)
    grid = np.linspace(lo, hi, 101)
    sse_grid = np.array([_profile(t, u, y, step_time_s, q)[0] for q in grid])
    best = int(np.argmin(sse_grid))
    left, right = grid[max(best - 1, 0)], grid[min(best + 1, len(grid) - 1)]
    # Golden-section refinement in log(tau), retaining the bounded grid result.
    ratio = (math.sqrt(5.0) - 1.0)/2.0
    c, d = right - ratio*(right-left), left + ratio*(right-left)
    fc = _profile(t, u, y, step_time_s, c)[0]
    fd = _profile(t, u, y, step_time_s, d)[0]
    for _ in range(96):
        if fc <= fd:
            right, d, fd = d, c, fc
            c = right - ratio*(right-left)
            fc = _profile(t, u, y, step_time_s, c)[0]
        else:
            left, c, fc = c, d, fd
            d = left + ratio*(right-left)
            fd = _profile(t, u, y, step_time_s, d)[0]
    candidates = [grid[best], (left + right)/2]
    fit = min((_profile(t, u, y, step_time_s, q) for q in candidates), key=lambda z: z[0])
    sse, tau, coeff, residual, condition, rank = fit
    lag = _lag1(residual)
    rms = float(math.sqrt(sse/len(t)))
    null_residual = y - y.mean()
    null_sse = float(null_residual @ null_residual)
    scale_floor = np.finfo(float).eps * max(1.0, float(y @ y))
    profile_span_fraction = float((np.max(sse_grid) - np.min(sse_grid)) / max(null_sse, scale_floor))
    improvement_fraction = float(max(0.0, null_sse - sse)/max(null_sse, scale_floor))
    nearby = []
    for direction in (-1.0, 1.0):
        neighbor = math.log(tau) + direction*math.log(1.1)
        if lo <= neighbor <= hi:
            nearby.append(_profile(t, u, y, step_time_s, neighbor)[0])
    local_rise_fraction = (float(max(0.0, min(nearby) - sse)/max(null_sse, scale_floor))
                           if nearby else 0.0)
    bound_tol = 1e-3
    hit_low = tau <= TAU_BOUNDS_S[0]*(1 + bound_tol)
    hit_high = tau >= TAU_BOUNDS_S[1]*(1 - bound_tol)
    response_amplitude = abs(float(coeff[1]))*input_delta
    response_floor = max(1e-6, 1e-6*max(1.0, float(np.max(np.abs(y)))))
    probe_response_present = response_amplitude > max(response_floor, 3.0*rms)
    tau_identifiable = (probe_response_present and not (hit_low or hit_high) and
                        local_rise_fraction > 1e-4 and math.isfinite(condition) and condition < 1e6)
    flags = {
        'probe_response_present': probe_response_present,
        'tau_profile_informative': tau_identifiable,
        'full_rank_nuisance_fit': rank == 3,
        'well_conditioned_nuisance_fit': math.isfinite(condition) and condition < 1e6,
        'tau_inside_bounds': not (hit_low or hit_high),
        'small_residual_lag1': abs(lag) < 0.3,
        'reference_excitation_present': True,
    }
    return {
        'label': 'descriptive first-order thermal fit',
        'data_origin': 'unspecified by fitter',
        'qualification': 'not assessed',
        'model': 'probe = offset + gain * exact-ZOH-filtered(reference) + initial_transient',
        'input_assumption': ('reference is held at each sampled value; a known discontinuity is split at '
                             '--step-time. Ramp curvature between samples or uncertain step timing can bias tau.'),
        'samples': int(len(t)),
        'step_time_s': step_time_s,
        'tau_s': float(tau),
        't63_s': float(tau),
        't90_s': float(math.log(10.0)*tau),
        'offset_c': float(coeff[0]),
        'gain': float(coeff[1]),
        'initial_transient_c': float(coeff[2]),
        'residual_rms_c': rms,
        'residual_lag1': lag,
        'fitted_probe_step_response_c': response_amplitude,
        'profile_sse_span_fraction_of_constant_model': profile_span_fraction,
        'profile_sse_improvement_fraction': improvement_fraction,
        'local_sse_rise_for_10pct_tau_change_fraction': local_rise_fraction,
        'nuisance_design_condition_number': condition,
        'nuisance_design_rank': rank,
        'tau_bounds_s': list(TAU_BOUNDS_S),
        'tau_hit_lower_bound': bool(hit_low),
        'tau_hit_upper_bound': bool(hit_high),
        'validity_flags': flags,
        'validity_notes': [key for key, value in flags.items() if not value],
    }


def _read_csv(path):
    with Path(path).open('r', newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        required = ('t_s', 'reference_c', 'probe_c')
        if reader.fieldnames is None or not set(required).issubset(reader.fieldnames):
            raise ValueError('CSV must contain columns: t_s, reference_c, probe_c')
        rows = list(reader)
    try:
        cols = [[float(row[key]) for row in rows] for key in required]
    except (ValueError, TypeError) as exc:
        raise ValueError('CSV columns must contain numeric values') from exc
    return cols


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_path', type=Path)
    parser.add_argument('--step-time', required=True, type=float,
                        help='known reference air-step time in seconds; a sample bracket is split at this time')
    parser.add_argument('--output', required=True, type=Path, help='separate JSON report path')
    args = parser.parse_args(argv)
    try:
        if args.csv_path.resolve() == args.output.resolve():
            raise ValueError('output report must use a separate path from the input CSV')
        result = fit_step(*_read_csv(args.csv_path), args.step_time)
        result['input_file'] = str(args.csv_path)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
