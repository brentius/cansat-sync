"""GPL-3.0-only. How well can ONE descent determine the log-law parameters u* and z0?

SYNTHETIC feasibility study of mission hypothesis S2 (proposal section 3 and
Appendix K.4): the fitted roughness length "agrees in order of magnitude with
the land cover upwind". Nothing here is a measurement.

Model (sim/nav/truth.py make_wind, proposal Appendix I): the mean wind is the
neutral log law  U(z) = (u*/kappa) ln(z/z0)  in one direction; each horizontal
component carries frozen turbulence with covariance
    sigma_h^2 g(|zeta1 - zeta2|),  g(r) = (1 - r/2) exp(-r),  zeta = ln(z)/a.
A single descent samples ONE realisation of that turbulence, not a time mean.

Two estimators are compared on the same profiles:

  ols_log_law   ordinary least squares of wind SPEED on ln z (Appendix K.4),
                with the textbook OLS uncertainty that assumes independent
                residuals.
  fit_log_law   generalised least squares / maximum likelihood on both
                horizontal components with the turbulence covariance. For a
                fixed ln z0 the mean is linear in two amplitudes; the profile
                likelihood over ln z0 is a ratio of two quadratics, so its
                maximum is found in closed form.

`python sim/nav/similarity_fit.py` regenerates everything under
sim/nav/results/similarity_fit/. Inputs are tagged PROPOSAL / ASSUMED /
SOURCED in INPUTS and recorded in summary.json.

Limits: neutral stratification only; the log law is assumed valid over the
whole fitted height range (really only the lowest ~50-100 m); turbulence is
frozen during the descent; the kernel is the model's own and the fit is told
it (except in the cases that say otherwise).
"""
import csv
import json
import math
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import truth  # noqa: E402

KAPPA = 0.4
LN10 = math.log(10.0)
RESULTS = HERE/'results'/'similarity_fit'
SEED = 20261010
N_PROFILES = 2000
SPACING = 2.0
RANGES = {'80-300': (80.0, 300.0), '10-300': (10.0, 300.0),
          '10-100': (10.0, 100.0), '2-100': (2.0, 100.0)}
ANEMOMETER_HEIGHT = 6.0
CHI2_68, CHI2_95 = 1.0, 3.841458820694124
Z_95 = 1.959963984540054

INPUTS = {
    'kappa': {'value': KAPPA, 'tag': 'SOURCED', 'note': 'von Karman constant, proposal K.4'},
    'log_law_fit': {'value': 'u = alpha ln z + beta, u* = kappa alpha, z0 = exp(-beta/alpha)',
                    'tag': 'PROPOSAL', 'note': 'Appendix K.4'},
    's2_criterion': {'value': 'fitted z0 within a factor 10 of truth', 'tag': 'PROPOSAL',
                     'note': 'section 3, S2: order-of-magnitude agreement'},
    'release_height_m': {'value': 300.0, 'tag': 'PROPOSAL'},
    'sample_spacing_m': {'value': SPACING, 'tag': 'ASSUMED',
                         'note': 'one wind sample every 2 m of height'},
    'height_ranges_m': {'value': RANGES, 'tag': 'ASSUMED',
                        'note': '80-300 m is where image-derived winds exist'},
    'wind_model': {'value': asdict(truth.WindParams()), 'tag': 'ASSUMED',
                   'note': 'truth.WindParams defaults, proposal Appendix I; not measured'},
    'anemometer_height_m': {'value': ANEMOMETER_HEIGHT, 'tag': 'PROPOSAL',
                            'note': 'ground anemometer logged through the flight'},
    'anemometer_standard_error_m_s': {'value': [0.1, 0.2, 0.5], 'tag': 'ASSUMED',
                                      'note': 'standard error of the MEAN wind vector at 6 m, per axis'},
    'profiles_per_case': {'value': N_PROFILES, 'tag': 'ASSUMED'},
    'seed': {'value': SEED, 'tag': 'ASSUMED'},
}

LIMITS = [
    'SYNTHETIC: no measured wind profile enters any number here.',
    'Neutral stratification only; no stability correction (Monin-Obukhov psi) is fitted.',
    'The log law is assumed valid over the whole fitted range; in reality only the lowest ~50-100 m.',
    'Frozen turbulence: the profile does not evolve during the ~28 s descent.',
    "The turbulence kernel is the model's own (eddy scale = eddy_slope*z with no cap aloft); the GLS "
    'fit is told the true kernel and sigma_h unless the case name says otherwise.',
    'Wind samples are taken as error-free except in the noise= cases; retrieval error, height error '
    'and canopy lag are not modelled.',
    'Displacement height is zero and the upwind fetch is homogeneous.',
]


# ----------------------------------------------------------------- the model

def eddy_coordinate(z, eddy_slope=0.5, eddy_floor=1.0):
    """zeta = integral dz/L(z), L = eddy_slope*max(z, eddy_floor); matches truth.make_wind."""
    z = np.asarray(z, float)
    zc = np.maximum(z, eddy_floor)
    return np.where(z <= eddy_floor, z/(eddy_slope*eddy_floor),
                    1/eddy_slope + np.log(zc/eddy_floor)/eddy_slope)


def turbulence_cov(z, sigma_h=1.0, eddy_slope=0.5, eddy_floor=1.0, noise=0.0):
    """Covariance (n, n) of one horizontal wind component at heights z, plus white noise."""
    z = _heights(z)
    if not sigma_h > 0 or not eddy_slope > 0 or not eddy_floor > 0 or noise < 0:
        raise ValueError('sigma_h, eddy_slope, eddy_floor must be positive and noise non-negative')
    q = eddy_coordinate(z, eddy_slope, eddy_floor)
    r = np.abs(q[:, None] - q[None, :])
    return sigma_h**2*(1 - r/2)*np.exp(-r) + noise**2*np.eye(len(z))


def friction_velocity(speed_10m, z0):
    return KAPPA*speed_10m/math.log(10.0/z0)


def _heights(z, minimum=1):
    z = np.asarray(z, float)
    if z.ndim != 1 or len(z) < minimum:
        raise ValueError(f'z must be one-dimensional with at least {minimum} heights')
    if not np.all(np.isfinite(z)) or np.any(z <= 0):
        raise ValueError('heights must be finite and positive')
    return z


def _samples(x, n, name):
    x = np.asarray(x, float)
    if x.ndim < 1 or x.shape[-1] != n:
        raise ValueError(f'{name} must have {n} samples along its last axis')
    if not np.all(np.isfinite(x)):
        raise ValueError(f'{name} contains non-finite values')
    return x


# ------------------------------------------------------- ordinary least squares

def ols_log_law(z, speed):
    """Appendix K.4: least squares of wind speed on ln z, residuals taken as independent.

    z (n,), speed (..., n). Returns alpha, beta, u_star, z0, ln_z0 and the
    uncertainties OLS itself would state (delta method for ln z0). `valid` is
    False where alpha <= 0, for which z0 is undefined (NaN).
    """
    z = _heights(z, 3)
    speed = _samples(speed, len(z), 'speed')
    lnz = np.log(z)
    n = len(z)
    mean_l = lnz.mean()
    dl = lnz - mean_l
    sxx = float(dl @ dl)
    if sxx <= 0:
        raise ValueError('heights must not all be equal')
    alpha = speed @ dl/sxx
    beta = speed.mean(axis=-1) - alpha*mean_l
    resid = speed - alpha[..., None]*lnz - beta[..., None]
    s2 = (resid*resid).sum(axis=-1)/(n - 2)
    var_a = s2/sxx
    var_b = s2*(1/n + mean_l**2/sxx)
    cov_ab = -s2*mean_l/sxx
    valid = alpha > 0
    with np.errstate(divide='ignore', invalid='ignore', over='ignore'):
        safe = np.where(valid, alpha, np.nan)
        ln_z0 = -beta/safe
        var_l = beta**2*var_a/safe**4 + var_b/safe**2 - 2*beta*cov_ab/safe**3
        z0 = np.exp(np.clip(ln_z0, -700, 700))
    return {'alpha': alpha, 'beta': beta, 'u_star': KAPPA*alpha, 'z0': z0, 'ln_z0': ln_z0,
            'sd_alpha': np.sqrt(var_a), 'sd_u_star': KAPPA*np.sqrt(var_a),
            'sd_ln_z0': np.sqrt(np.maximum(var_l, 0)), 'cov_alpha_beta': cov_ab,
            'sd_beta': np.sqrt(var_b), 'residual_sd': np.sqrt(s2), 'valid': valid}


# ---------------------------------------------- generalised least squares / ML

def _augment(z, cov, anemometer_height, anemometer_se, n_pooled):
    """ln z and covariance of the data vector: pooled profile, then the anemometer point."""
    if not n_pooled >= 1:
        raise ValueError('n_pooled must be at least 1')
    lnz = np.log(z)
    cov = cov/float(n_pooled)
    if anemometer_height is None:
        return lnz, cov
    if not (anemometer_height > 0 and anemometer_se > 0):
        raise ValueError('anemometer height and standard error must be positive')
    n = len(z)
    big = np.zeros((n + 1, n + 1))
    big[:n, :n] = cov
    big[n, n] = anemometer_se**2
    return np.append(lnz, math.log(anemometer_height)), big


def _weights(lnz, cov):
    cov = np.asarray(cov, float)
    if cov.shape != (len(lnz), len(lnz)):
        raise ValueError('cov must be (n, n) for n heights')
    if not np.all(np.isfinite(cov)) or not np.allclose(cov, cov.T, rtol=1e-9, atol=1e-12):
        raise ValueError('cov must be finite and symmetric')
    try:
        chol = np.linalg.cholesky(cov)
    except np.linalg.LinAlgError:
        raise ValueError('cov must be positive definite') from None
    wb = np.linalg.solve(cov, np.stack((lnz, np.ones_like(lnz)), axis=1))
    return chol, wb, float(lnz @ wb[:, 0]), float(lnz @ wb[:, 1]), float(wb[:, 1].sum())


def _ratio(c, d, x):
    with np.errstate(all='ignore'):
        f = (c[0] + c[1]*x + c[2]*x*x)/(d[0] + d[1]*x + d[2]*x*x)
    return f


def _quadratic_roots(q2, q1, q0):
    """Real roots (low, high) of q2 x^2 + q1 x + q0, NaN where there are none."""
    with np.errstate(all='ignore'):
        disc = q1*q1 - 4*q2*q0
        sq = np.sqrt(np.where(disc >= 0, disc, np.nan))
        qq = -0.5*(q1 + np.where(q1 >= 0, 1.0, -1.0)*sq)
        r1, r2 = qq/q2, q0/qq
    return np.minimum(r1, r2), np.maximum(r1, r2)


def fit_log_law(z, u, v, cov=None, anemometer=None, sigma_h=1.0, eddy_slope=0.5, eddy_floor=1.0,
                noise=0.0, n_pooled=1, estimate_scale=False):
    """Maximum-likelihood log-law fit to both horizontal wind components.

    z (n,) heights in m; u, v (..., n) east and north wind (leading axes are a
    batch of independent profiles). The mean is (Ax, Ay)*ln(z/z0); the errors
    on u and on v are independent, each with covariance `cov` (n, n). With
    cov=None the model kernel is used: turbulence_cov(z, sigma_h, eddy_slope,
    eddy_floor, noise).

    anemometer   None, or (height_m, u_mean, v_mean, standard_error): a
                 measurement of the MEAN wind vector, independent of the profile.
    n_pooled     u, v are the average of this many descents through the same
                 mean profile with independent turbulence (cov is divided by it).
    estimate_scale  multiply the stated covariance by chi2/dof instead of
                 trusting the absolute size of cov.

    Returns a dict: u_star, z0, ln_z0, direction_rad (direction the fitted wind
    blows towards at the highest sample, atan2(v, u)), amplitude (..., 2),
    sd_u_star, sd_ln_z0, sd_direction_rad, cov_ustar_lnz0 (..., 2, 2) from the
    Fisher information at the estimate, ln_z0_interval_68 / _95 (profile
    likelihood, may be infinite), chi2, dof, scale, log_likelihood. The fit is
    unconstrained in ln z0: check `physical` (z0 below the lowest height).
    """
    z = _heights(z, 3)
    u = _samples(u, len(z), 'u')
    v = _samples(v, len(z), 'v')
    if u.shape != v.shape:
        raise ValueError('u and v must have the same shape')
    if np.ptp(z) <= 0:
        raise ValueError('heights must not all be equal')
    if cov is None:
        cov = turbulence_cov(z, sigma_h, eddy_slope, eddy_floor, noise)
    cov = np.asarray(cov, float)
    if cov.shape != (len(z), len(z)):
        raise ValueError('cov must be (n, n) for n heights')
    height = se = None
    if anemometer is not None:
        height, ua, va, se = anemometer
        ua, va = np.asarray(ua, float), np.asarray(va, float)
        if ua.shape != u.shape[:-1] or va.shape != u.shape[:-1]:
            raise ValueError('anemometer wind must match the batch shape of u')
        if not (np.all(np.isfinite(ua)) and np.all(np.isfinite(va))):
            raise ValueError('anemometer wind contains non-finite values')
        u = np.concatenate((u, ua[..., None]), axis=-1)
        v = np.concatenate((v, va[..., None]), axis=-1)
    lnz, big = _augment(z, cov, height, se, n_pooled)
    chol, wb, sll, sl1, s11 = _weights(lnz, big)
    n = len(lnz)
    pu, qu, pv, qv = u @ wb[:, 0], u @ wb[:, 1], v @ wb[:, 0], v @ wb[:, 1]
    flat_u, flat_v = u.reshape(-1, n).T, v.reshape(-1, n).T
    yu, yv = np.linalg.solve(chol, flat_u), np.linalg.solve(chol, flat_v)
    total = ((yu*yu).sum(axis=0) + (yv*yv).sum(axis=0)).reshape(u.shape[:-1])

    # Explained sum of squares f(L) = N(L)/D(L), L = ln z0; chi2(L) = total - f(L).
    num = (pu*pu + pv*pv, -2*(pu*qu + pv*qv), qu*qu + qv*qv)
    den = (sll, -2*sl1, s11)
    low, high = _quadratic_roots(num[2]*den[1] - num[1]*den[2], 2*(num[2]*den[0] - num[0]*den[2]),
                                 num[1]*den[0] - num[0]*den[1])
    f_low, f_high = _ratio(num, den, low), _ratio(num, den, high)
    f_low = np.where(np.isfinite(f_low), f_low, -np.inf)
    f_high = np.where(np.isfinite(f_high), f_high, -np.inf)
    ln_z0 = np.where(f_low >= f_high, low, high)
    best = np.maximum(f_low, f_high)
    shh = den[0] + den[1]*ln_z0 + den[2]*ln_z0**2
    ax, ay = (pu - ln_z0*qu)/shh, (pv - ln_z0*qv)/shh
    amp = np.hypot(ax, ay)
    chi2 = np.maximum(total - best, 0.0)
    dof = 2*n - 3
    scale = chi2/dof if estimate_scale else np.ones_like(chi2)

    # Fisher information for (amplitude, ln z0) in the along-wind frame.
    det = sll*s11 - sl1*sl1
    sh1 = sl1 - ln_z0*s11
    with np.errstate(divide='ignore', invalid='ignore', over='ignore'):
        var_a = scale*s11/det
        var_l = scale*shh/(amp*amp*det)
        cov_al = scale*sh1/(amp*det)
        var_dir = scale/(amp*amp*shh)
        z0 = np.exp(np.clip(ln_z0, -700, 700))
    cov2 = np.empty(chi2.shape + (2, 2))
    cov2[..., 0, 0] = KAPPA**2*var_a
    cov2[..., 0, 1] = cov2[..., 1, 0] = KAPPA*cov_al
    cov2[..., 1, 1] = var_l
    h_top = lnz[int(np.argmax(z))] - ln_z0
    sign = np.where(h_top >= 0, 1.0, -1.0)
    logdet = 2*np.log(np.diag(chol)).sum()
    out = {'u_star': KAPPA*amp, 'z0': z0, 'ln_z0': ln_z0,
           'direction_rad': np.arctan2(sign*ay, sign*ax), 'amplitude': np.stack((ax, ay), axis=-1),
           'sd_u_star': KAPPA*np.sqrt(var_a), 'sd_ln_z0': np.sqrt(var_l),
           'sd_direction_rad': np.sqrt(var_dir), 'cov_ustar_lnz0': cov2,
           'chi2': chi2, 'dof': dof, 'scale': scale,
           'log_likelihood': -0.5*chi2 - logdet - n*math.log(2*math.pi),
           'physical': ln_z0 < lnz.min(),
           '_profile': (num, den, best)}
    out['ln_z0_interval_68'] = _profile_interval(out, CHI2_68)
    out['ln_z0_interval_95'] = _profile_interval(out, CHI2_95)
    return out


def profile_delta(fit, ln_z0):
    """-2 ln(likelihood ratio) of a given ln z0 against the best fit, in units of the fit's scale."""
    num, den, best = fit['_profile']
    return np.maximum(best - _ratio(num, den, np.asarray(ln_z0, float)), 0.0)/fit['scale']


def _profile_interval(fit, level):
    """Profile-likelihood interval for ln z0 as (..., 2); infinite on a side it cannot bound."""
    num, den, best = fit['_profile']
    t = best - level*fit['scale']
    q2, q1, q0 = num[2] - t*den[2], num[1] - t*den[1], num[0] - t*den[0]
    low, high = _quadratic_roots(q2, q1, q0)
    est = fit['ln_z0']
    bounded = q2 < 0
    none = ~np.isfinite(low) | ~np.isfinite(high)
    below = est <= low
    lo = np.where(bounded, low, np.where(below, -np.inf, high))
    hi = np.where(bounded, high, np.where(below, low, np.inf))
    lo = np.where(none, -np.inf, lo)
    hi = np.where(none, np.inf, hi)
    return np.stack((lo, hi), axis=-1)


# ------------------------------------------------------------- analytic bounds

def _jacobian(lnz, amplitude, ln_z0):
    return np.stack((lnz - ln_z0, -amplitude*np.ones_like(lnz)), axis=1)


def linearised_cov(z, cov_true, u_star, z0, cov_fit=None, anemometer=None, n_pooled=1):
    """ANALYTIC covariance of (u*, ln z0) for a weighted-least-squares fit, linearised at truth.

    cov_fit is the covariance the fit ASSUMES (None: the true one, which gives
    the Cramer-Rao bound / GLS covariance; np.eye(n): ordinary least squares on
    the along-wind component). anemometer is (height_m, standard_error).
    Also returns the direction variance and, for the true weights, the Fisher
    information for (u*, ln z0).
    """
    z = _heights(z, 3)
    height, se = anemometer if anemometer is not None else (None, None)
    lnz, true = _augment(z, np.asarray(cov_true, float), height, se, n_pooled)
    fit = true if cov_fit is None else _augment(z, np.asarray(cov_fit, float), height, se, n_pooled)[1]
    amp, ln_z0 = u_star/KAPPA, math.log(z0)
    jac = _jacobian(lnz, amp, ln_z0)
    wj = np.linalg.solve(fit, jac)
    bread = np.linalg.inv(jac.T @ wj)
    cov = bread @ (wj.T @ true @ wj) @ bread
    wh = np.linalg.solve(fit, jac[:, 0])
    var_dir = float(wh @ true @ wh)/float(jac[:, 0] @ wh)**2/amp**2
    scale = np.diag([KAPPA, 1.0])
    cov = scale @ cov @ scale
    sd_l = math.sqrt(cov[1, 1])
    return {'cov_ustar_lnz0': cov, 'sd_u_star': math.sqrt(cov[0, 0]),
            'sd_u_star_relative': math.sqrt(cov[0, 0])/u_star, 'sd_ln_z0': sd_l,
            'sd_log10_z0': sd_l/LN10, 'correlation': cov[0, 1]/math.sqrt(cov[0, 0]*cov[1, 1]),
            'sd_direction_deg': math.degrees(math.sqrt(var_dir)),
            'p_within_x10_gaussian': math.erf(LN10/(sd_l*math.sqrt(2))),
            'p_within_x3_gaussian': math.erf(math.log(3.0)/(sd_l*math.sqrt(2)))}


# ------------------------------------------------------------------ the study

def heights_for(z_low, z_high, spacing=SPACING):
    return np.arange(z_low, z_high + spacing/2, spacing)


def sample_profiles(n, rng, wp, heights, direction=None, chunk=2000):
    """u, v (n, levels) at `heights` and the mean heading (n,), from truth.make_wind."""
    wp = replace(wp, top=float(np.max(heights)))
    index = np.rint(np.asarray(heights)/wp.dz).astype(int)
    if not np.allclose(index*wp.dz, heights):
        raise ValueError('heights must lie on the wind grid')
    u, v, heading = [], [], []
    for start in range(0, n, chunk):
        m = min(chunk, n - start)
        wind, mean, _ = truth.make_wind(m, rng, wp, direction)
        u.append(wind.profile[:, index, 0])
        v.append(wind.profile[:, index, 1])
        heading.append(np.arctan2(mean[:, -1, 1], mean[:, -1, 0]))
    return np.concatenate(u), np.concatenate(v), np.concatenate(heading)


def _quantiles(x):
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return {'median': None, 'p16': None, 'p84': None, 'robust_sd': None}
    p16, p50, p84 = np.percentile(x, [15.865, 50.0, 84.135])
    return {'median': p50, 'p16': p16, 'p84': p84, 'robust_sd': (p84 - p16)/2}


def _score(ln_ratio, ustar_rel, sd_ln, sd_ustar_rel, valid):
    """Accuracy and honesty of one estimator; invalid fits count against every fraction."""
    log10 = np.where(valid, ln_ratio, np.nan)/LN10
    ok = valid & np.isfinite(ln_ratio)
    with np.errstate(invalid='ignore'):
        within = lambda k: float(np.mean(ok & (np.abs(log10) < math.log10(k))))  # noqa: E731
        cover = lambda err, sd, k: float(np.mean(ok & (np.abs(err) <= k*sd)))    # noqa: E731
        out = {'frac_invalid': float(np.mean(~ok)),
               'frac_within_x10': within(10.0), 'frac_within_x3': within(3.0),
               'log10_z0_ratio': _quantiles(log10),
               'u_star_relative_error': _quantiles(np.where(ok, ustar_rel, np.nan)),
               'frac_u_star_within_20pct': float(np.mean(ok & (np.abs(ustar_rel) < 0.2))),
               'stated_sd_log10_z0_median': float(np.nanmedian(np.where(ok, sd_ln, np.nan))/LN10),
               'coverage_ln_z0_68': cover(ln_ratio, sd_ln, 1.0),
               'coverage_ln_z0_95': cover(ln_ratio, sd_ln, Z_95),
               'coverage_u_star_68': cover(ustar_rel, sd_ustar_rel, 1.0),
               'coverage_u_star_95': cover(ustar_rel, sd_ustar_rel, Z_95)}
    rs = out['log10_z0_ratio']['robust_sd']
    out['overconfidence'] = rs/out['stated_sd_log10_z0_median'] if rs else None
    return out


def run_case(case, n=N_PROFILES, seed=SEED, ranges=RANGES):
    """Monte Carlo and analytic results for one case over every height range.

    case keys (all optional): sigma_h, speed_10m, z0, eddy_slope (truth),
    fit_eddy_slope (what GLS is told), noise, anemometer_se, n_pooled, index.
    """
    wp = replace(truth.WindParams(), sigma_h=case.get('sigma_h', 1.0), speed_10m=case.get('speed_10m', 5.0),
                 roughness=case.get('z0', 0.1), eddy_slope=case.get('eddy_slope', 0.5))
    fit_slope = case.get('fit_eddy_slope', wp.eddy_slope)
    noise = case.get('noise', 0.0)
    se = case.get('anemometer_se')
    pooled = case.get('n_pooled', 1)
    rng = np.random.default_rng(np.random.SeedSequence([seed, case.get('index', 0)]))
    u_star = friction_velocity(wp.speed_10m, wp.roughness)
    amp, ln_z0 = u_star/KAPPA, math.log(wp.roughness)

    grid = heights_for(min(lo for lo, _ in ranges.values()), max(hi for _, hi in ranges.values()))
    # Pooled descents share one mean wind vector; a single descent has a random heading.
    u, v, heading = sample_profiles(n*pooled, rng, wp, grid, direction=None if pooled == 1 else 0.0)
    if noise > 0:
        u = u + rng.normal(0.0, noise, u.shape)
        v = v + rng.normal(0.0, noise, v.shape)
    u, v = u.reshape(n, pooled, -1), v.reshape(n, pooled, -1)
    heading = heading.reshape(n, pooled)[:, 0]
    anem = None
    if se is not None:
        mean6 = amp*(math.log(ANEMOMETER_HEIGHT) - ln_z0)
        anem = (mean6*np.cos(heading) + rng.normal(0.0, se, n), mean6*np.sin(heading) + rng.normal(0.0, se, n))

    result = {'truth': {'u_star': u_star, 'z0': wp.roughness, 'amplitude': amp,
                        'sigma_h_over_u_star': wp.sigma_h/u_star},
              'case': dict(case), 'n_profiles': n, 'ranges': {}}
    for name, (lo, hi) in ranges.items():
        pick = (grid >= lo - 1e-9) & (grid <= hi + 1e-9)
        z = grid[pick]
        up, vp = u[:, :, pick], v[:, :, pick]

        speed = np.hypot(up, vp).reshape(n, -1)
        ols = ols_log_law(np.tile(z, pooled), speed)
        ols_score = _score(ols['ln_z0'] - ln_z0, ols['u_star']/u_star - 1, ols['sd_ln_z0'],
                           ols['sd_u_star']/u_star, ols['valid'])

        cov_fit = turbulence_cov(z, wp.sigma_h, fit_slope, wp.eddy_floor, noise)
        cov_true = turbulence_cov(z, wp.sigma_h, wp.eddy_slope, wp.eddy_floor, noise)
        anemometer = None if anem is None else (ANEMOMETER_HEIGHT, anem[0], anem[1], se)
        fit = fit_log_law(z, up.mean(axis=1), vp.mean(axis=1), cov=cov_fit, anemometer=anemometer,
                          n_pooled=pooled)
        err = fit['ln_z0'] - ln_z0
        finite = np.isfinite(err)
        gls = _score(err, fit['u_star']/u_star - 1, fit['sd_ln_z0'], fit['sd_u_star']/u_star, finite)
        delta = profile_delta(fit, ln_z0)
        s2 = fit['chi2']/fit['dof']
        dirn = np.degrees(np.abs(np.angle(np.exp(1j*(fit['direction_rad'] - heading)))))
        gls.update({
            'coverage_profile_68': float(np.mean(delta <= CHI2_68)),
            'coverage_profile_95': float(np.mean(delta <= CHI2_95)),
            'coverage_ln_z0_68_scale_estimated': float(np.mean(np.abs(err) <= np.sqrt(s2)*fit['sd_ln_z0'])),
            'coverage_ln_z0_95_scale_estimated': float(np.mean(np.abs(err) <= Z_95*np.sqrt(s2)*fit['sd_ln_z0'])),
            'coverage_profile_68_scale_estimated': float(np.mean(delta <= CHI2_68*s2)),
            'coverage_profile_95_scale_estimated': float(np.mean(delta <= CHI2_95*s2)),
            'frac_interval_95_unbounded': float(np.mean(~np.isfinite(fit['ln_z0_interval_95']).all(axis=-1))),
            'frac_unphysical': float(np.mean(~fit['physical'])),
            'chi2_over_dof_median': float(np.median(s2)),
            'direction_error_deg_median': float(np.median(dirn)),
            'ln_z0_std': float(np.std(err)),
        })

        point = None if se is None else (ANEMOMETER_HEIGHT, se)
        analytic = {'cramer_rao': _plain(linearised_cov(z, cov_true, u_star, wp.roughness,
                                                        anemometer=point, n_pooled=pooled)),
                    'gls_as_fitted': _plain(linearised_cov(z, cov_true, u_star, wp.roughness, cov_fit=cov_fit,
                                                           anemometer=point, n_pooled=pooled)),
                    'ols_on_component': _plain(linearised_cov(z, cov_true, u_star, wp.roughness,
                                                              cov_fit=np.eye(len(z)), n_pooled=pooled))}
        q = eddy_coordinate(z, wp.eddy_slope, wp.eddy_floor)
        result['ranges'][name] = {'n_levels': int(len(z)), 'eddy_scales_spanned': float(q.max() - q.min()),
                                  'ols': ols_score, 'gls': gls, 'analytic': analytic}
    return result


def _plain(d):
    return {k: v for k, v in d.items() if not isinstance(v, np.ndarray)}


def build_cases():
    cases = {'baseline': {}}
    for s in (0.5, 2.0):
        cases[f'sigma_h={s:g}'] = {'sigma_h': s}
    for s in (2.0, 8.0):
        cases[f'speed_10m={s:g}'] = {'speed_10m': s}
    for s in (0.03, 0.3):
        cases[f'z0={s:g}'] = {'z0': s}
    for true, told in ((0.25, 0.25), (1.0, 1.0), (0.25, 0.5), (1.0, 0.5), (0.5, 0.25), (0.5, 1.0)):
        cases[f'eddy_slope={true:g},fit_told={told:g}'] = {'eddy_slope': true, 'fit_eddy_slope': told}
    for s in (0.1, 0.3):
        cases[f'noise={s:g}'] = {'noise': s}
    for pooled in (1, 3, 5, 10):
        for se in (None, 0.1, 0.2, 0.5):
            if pooled == 1 and se is None:
                continue
            case = {'n_pooled': pooled}
            if se is not None:
                case['anemometer_se'] = se
            cases[option_name(pooled, se)] = case
    for i, case in enumerate(cases.values()):
        case['index'] = i
    return cases


def option_name(pooled, se):
    return 'baseline' if pooled == 1 and se is None else f'N={pooled},anemometer={"none" if se is None else f"{se:g}"}'


def _round(x):
    if isinstance(x, dict):
        return {k: _round(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_round(v) for v in x]
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (float, np.floating)):
        return float(f'{float(x):.4g}') if math.isfinite(x) else None
    if isinstance(x, np.integer):
        return int(x)
    return x


def write_csv(path, cases):
    cols = ['case', 'range', 'n_levels', 'method', 'frac_within_x10', 'frac_within_x3', 'frac_invalid',
            'log10_ratio_median', 'log10_ratio_p16', 'log10_ratio_p84', 'log10_ratio_robust_sd',
            'u_star_rel_median', 'u_star_rel_p16', 'u_star_rel_p84', 'stated_sd_log10_z0_median',
            'coverage_ln_z0_68', 'coverage_ln_z0_95', 'coverage_profile_68', 'coverage_profile_95',
            'coverage_u_star_68', 'coverage_u_star_95', 'analytic_sd_log10_z0', 'analytic_sd_u_star_rel']
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['# SYNTHETIC: sim/nav/similarity_fit.py; analytic_* columns are linearised, the rest Monte Carlo'])
        w.writerow(cols)
        for cname, case in cases.items():
            for rname, r in case['ranges'].items():
                for method, key in (('ols', 'ols_on_component'), ('gls', 'gls_as_fitted')):
                    m, q, us = r[method], r[method]['log10_z0_ratio'], r[method]['u_star_relative_error']
                    row = [cname, rname, r['n_levels'], method, m['frac_within_x10'], m['frac_within_x3'],
                           m['frac_invalid'], q['median'], q['p16'], q['p84'], q['robust_sd'],
                           us['median'], us['p16'], us['p84'], m['stated_sd_log10_z0_median'],
                           m['coverage_ln_z0_68'], m['coverage_ln_z0_95'], m.get('coverage_profile_68', ''),
                           m.get('coverage_profile_95', ''), m['coverage_u_star_68'], m['coverage_u_star_95'],
                           r['analytic'][key]['sd_log10_z0'], r['analytic'][key]['sd_u_star_relative']]
                    w.writerow([_round(x) if x is not None else '' for x in row])


def headline(cases):
    """Which options put the GLS z0 within a factor 3 in at least 90 % of trials (Monte Carlo)."""
    passing, table = [], {}
    for pooled in (1, 3, 5, 10):
        for se in (None, 0.1, 0.2, 0.5):
            name = option_name(pooled, se)
            row = {r: cases[name]['ranges'][r]['gls']['frac_within_x3'] for r in RANGES}
            table[name] = row
            passing += [f'{name} @ {r}' for r, value in row.items() if value >= 0.9]
    return {'gls_frac_within_x3_by_option': table, 'options_with_gls_within_x3_at_least_0.90': passing,
            'note': 'Monte Carlo fractions; binomial standard error about 0.007 at 0.9 with 2000 profiles'}


def make_figures(cases, out):
    import plotting

    names = list(RANGES)
    base = cases['baseline']['ranges']
    figs = []

    groups = [{'label': f'{m.upper()} within x{k}', 'values': [base[r][m][f'frac_within_x{k}'] for r in names]}
              for k in (10, 3) for m in ('ols', 'gls')]
    figs.append(plotting.bar_chart(
        str(out/'fig1_fraction_within.png'), [f'{r} m' for r in names], groups,
        'SYNTHETIC: one descent, fraction of fitted z0 within a factor of truth',
        'fraction of descents', ylim=(0, 1),
        subtitle='Baseline wind model (5 m/s at 10 m, z0 0.1 m, sigma_h 1 m/s), samples every 2 m, 2000 profiles',
        hlines=[(0.9, '0.9', None)]))

    groups = [{'label': 'OLS stated 68 %', 'values': [base[r]['ols']['coverage_ln_z0_68'] for r in names]},
              {'label': 'GLS Fisher 68 %', 'values': [base[r]['gls']['coverage_ln_z0_68'] for r in names]},
              {'label': 'GLS profile 68 %', 'values': [base[r]['gls']['coverage_profile_68'] for r in names]},
              {'label': 'OLS stated 95 %', 'values': [base[r]['ols']['coverage_ln_z0_95'] for r in names]},
              {'label': 'GLS Fisher 95 %', 'values': [base[r]['gls']['coverage_ln_z0_95'] for r in names]},
              {'label': 'GLS profile 95 %', 'values': [base[r]['gls']['coverage_profile_95'] for r in names]}]
    figs.append(plotting.bar_chart(
        str(out/'fig2_coverage.png'), [f'{r} m' for r in names], groups,
        'SYNTHETIC: does the stated interval for ln z0 contain the truth?', 'fraction of descents covered',
        ylim=(0, 1.1), size=(1700, 680),
        subtitle='Honest intervals reach the dashed 0.68 and 0.95 lines; baseline, one descent',
        hlines=[(0.68, '', None), (0.95, '', None)]))

    pooled = (1, 3, 5, 10)
    for i, r in enumerate(('80-300', '10-300')):
        series = [{'x': list(pooled), 'style': 'line+points',
                   'y': [cases[option_name(p, se)]['ranges'][r]['gls']['frac_within_x3'] for p in pooled],
                   'label': 'no anemometer' if se is None else f'6 m anemometer, s.e. {se:g} m/s'}
                  for se in (None, 0.5, 0.2, 0.1)]
        figs.append(plotting.line_chart(
            str(out/f'fig3{"ab"[i]}_options_{r}.png'), series,
            f'SYNTHETIC: GLS z0 within a factor 3, profile {r} m', 'descents pooled on the same day',
            'fraction within a factor 3', ylim=(0, 1), xlim=(0, 11),
            subtitle='Monte Carlo, 2000 trials per point; dashed line is the 90 % target',
            hlines=[(0.9, '0.9', None)]))

    ses = np.geomspace(0.03, 3.0, 41)
    wp = truth.WindParams()
    u_star = friction_velocity(wp.speed_10m, wp.roughness)
    series = []
    for r, (lo, hi) in RANGES.items():
        z = heights_for(lo, hi)
        cov = turbulence_cov(z, wp.sigma_h, wp.eddy_slope, wp.eddy_floor)
        series.append({'x': list(ses), 'label': f'{r} m, analytic', 'style': 'line',
                       'y': [linearised_cov(z, cov, u_star, wp.roughness, anemometer=(ANEMOMETER_HEIGHT, s))
                             ['sd_log10_z0'] for s in ses]})
    for r in ('80-300', '10-300'):
        series.append({'x': [0.1, 0.2, 0.5], 'style': 'points', 'label': f'{r} m, Monte Carlo',
                       'y': [cases[option_name(1, s)]['ranges'][r]['gls']['log10_z0_ratio']['robust_sd']
                             for s in (0.1, 0.2, 0.5)]})
    figs.append(plotting.line_chart(
        str(out/'fig4_anemometer_bound.png'), series,
        'SYNTHETIC: uncertainty of log10 z0, one descent plus a 6 m anemometer',
        'standard error of the anemometer mean wind (m/s)', 'standard deviation of log10(z0)', logx=True,
        logy=True, subtitle='Lines: Cramer-Rao bound. Points: half the 16-84 % range of the Monte Carlo fits',
        hlines=[(math.log10(3)/1.645, '90 % within x3', None), (1/1.645, '90 % within x10', None)]))

    sens = [k for k in cases if not k.startswith('N=')]
    short = [k.replace('eddy_slope=', 'a=').replace(',fit_told=', '/fit ').replace('speed_10m=', 'U10=')
             .replace('sigma_h=', 'sig=') for k in sens]
    groups = [{'label': f'GLS {r} m', 'values': [cases[k]['ranges'][r]['gls']['frac_within_x10'] for k in sens]}
              for r in ('80-300', '10-300', '2-100')]
    figs.append(plotting.bar_chart(
        str(out/'fig5_sensitivity.png'), short, groups,
        'SYNTHETIC: one descent, GLS z0 within a factor 10 (the S2 criterion)', 'fraction of descents',
        ylim=(0, 1), size=(1700, 680), value_labels=False,
        subtitle='One input changed at a time from the baseline; a = eddy slope (true / told to the fit)'))

    return figs


def cdf_figure(out, n=N_PROFILES, seed=SEED):
    """Empirical CDF of log10(z0_hat/z0) for the baseline; regenerates the baseline profiles."""
    import plotting

    wp = truth.WindParams()
    rng = np.random.default_rng(np.random.SeedSequence([seed, 0]))
    grid = heights_for(2.0, 300.0)
    u, v, _ = sample_profiles(n, rng, wp, grid)
    ln_z0 = math.log(wp.roughness)
    x = np.linspace(-4, 4, 321)
    paths = []
    for method in ('ols', 'gls'):
        series = []
        for name, (lo, hi) in RANGES.items():
            pick = (grid >= lo) & (grid <= hi)
            z = grid[pick]
            if method == 'ols':
                est = ols_log_law(z, np.hypot(u[:, pick], v[:, pick]))['ln_z0']
            else:
                est = fit_log_law(z, u[:, pick], v[:, pick])['ln_z0']
            err = np.sort(np.where(np.isfinite(est), est - ln_z0, np.inf))/LN10
            series.append({'x': list(x), 'y': list(np.searchsorted(err, x)/n), 'label': f'{name} m',
                           'style': 'line'})
        label = 'OLS on speed (K.4)' if method == 'ols' else 'GLS / maximum likelihood'
        paths.append(plotting.line_chart(
            str(out/f'fig0_cdf_{method}.png'), series,
            f'SYNTHETIC: distribution of fitted z0 from one descent, {label}',
            'log10(fitted z0 / true z0)', 'cumulative fraction of descents', ylim=(0, 1), xlim=(-4, 4),
            subtitle='Baseline wind model; the S2 criterion is the band between the dashed lines',
            vlines=[(-1, 'x0.1', None), (1, 'x10', None)]))
    return paths


def main():
    start = time.time()
    RESULTS.mkdir(parents=True, exist_ok=True)
    specs = build_cases()
    cases = {}
    for name, spec in specs.items():
        cases[name] = run_case(spec)
        print(f'{name:38s} ' + '  '.join(
            f'{r}: x10 {c["gls"]["frac_within_x10"]:.2f} x3 {c["gls"]["frac_within_x3"]:.2f}'
            for r, c in cases[name]['ranges'].items()), flush=True)
    summary = {'label': 'SYNTHETIC: Monte Carlo on the truth.make_wind model plus linearised analytic bounds',
               'generator': 'sim/nav/similarity_fit.py', 'inputs': INPUTS, 'limits': LIMITS,
               'definitions': {
                   'ols': 'least squares of wind speed on ln z, uncertainty assuming independent residuals',
                   'gls': 'maximum likelihood on both components with the turbulence covariance, told sigma_h',
                   'frac_within_xK': 'Monte Carlo fraction of trials with |log10(z0_hat/z0)| < log10 K',
                   'coverage_ln_z0_68/95': "fraction with truth inside the method's own +-1 / +-1.96 sd interval",
                   'coverage_profile_68/95': 'same for the GLS profile-likelihood interval',
                   'robust_sd': 'half the 16-84 % range',
                   'overconfidence': 'robust_sd of log10 z0 error divided by the median stated sd',
                   'analytic.cramer_rao': 'inverse Fisher information at truth (ANALYTIC, linearised)',
                   'analytic.gls_as_fitted': 'sandwich covariance of GLS with the covariance it was told',
                   'analytic.ols_on_component': 'sandwich covariance of OLS on the along-wind component'},
               'headline': headline(cases), 'cases': cases}
    figs = cdf_figure(RESULTS) + make_figures(cases, RESULTS)
    summary['figures'] = [Path(p).name for p in figs]
    summary['runtime_s'] = time.time() - start
    (RESULTS/'summary.json').write_text(json.dumps(_round(summary), indent=1))
    write_csv(RESULTS/'similarity_fit.csv', cases)
    print(f'wrote {RESULTS} in {time.time() - start:.0f} s')


if __name__ == '__main__':
    main()
