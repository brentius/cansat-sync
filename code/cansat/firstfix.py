"""GPL-3.0-only. First fix: a full pose from one image's landmark matches.

The navigation filter (eskf.py) linearises about its own estimate, so it has
to be started close to the truth. At that moment yaw is unknown over the whole
circle (no magnetometer), tilt is known only to the ~20 degree swing, the
horizontal position to +/-300 m, and a large fraction of the coarse matcher's
landmarks may be aliases. solve_first_fix turns one image's matches into a
pose with a covariance, or says why it could not. Frames, the attitude error
convention and the camera model are those of eskf.py.

1. Back-projection. Each pixel is cast through an assumed attitude r0 (level
   unless the caller knows better) from the barometric height onto the ground
   plane, giving coordinates u_i in a frame whose yaw and origin are unknown.

2. Consensus. With rotation and scale as one complex number z = s exp(i psi),
   every true match obeys m_i = z u_i + t (Appendix L.2), so two matches fix
   (z, t) and the rest vote. Pairs are drawn without replacement, N =
   ln(1 - p)/ln(1 - w^2) of them (Appendix G.4; 25 for w = 0.5, p = 0.999),
   and more, up to MAX_DRAWS, when the best vote so far is below w n, because
   then w was optimistic. Pairs closer than 0.2 h in the assumed frame are
   skipped: they amplify the distortion below.

   The vote needs a tolerance because r0 is wrong by the tilt theta. To first
   order a pixel at normalised ground position q (|q| = tan alpha) lands
   h theta [e + (e.q) q] from where it should (checked numerically to
   0.03 h theta), e being the horizontal direction the boresight leans. The
   constant part is absorbed by t (Appendix C: tilt looks like translation);
   a similarity cannot absorb Appendix C's tan^2 alpha pattern. Measured on
   this sensor with noise-free matches, the best similarity of 16 true matches
   leaves 0.05 h RMS (0.10 h largest) at 10 degrees, 0.11 h (0.25 h) at 20
   and 0.20 h (0.50 h) at 30, and a two-point hypothesis is supported by 88,
   63 and 47 % of the true matches. TOLERANCE = 0.15 h keeps 99 % of the true
   matches within reach of the best similarity at 10 degrees and 86 % at 20,
   while an unrelated match passes with probability pi 0.15^2/(2.0 x 1.5) =
   2.4 %. It is not critical, because step 3 re-selects: with 40 % outliers
   and 20 degrees of tilt the fraction solved is flat (0.86) from 0.12 to
   0.30. Pixel noise (0.0014 h per pixel) is negligible beside it. Hypotheses
   with |z| outside SCALE_RANGE = 0.6..1.8 contradict the barometer: |z| is
   the true over the barometric height times the stretch of a tilted view,
   which measured 0.75..1.73 for 0 to 30 % of barometer error at up to 20
   degrees of tilt.

3. Refinement. The planar vote is only a hypothesis generator. Starting from
   r0 turned by psi, at t and the barometric height, Levenberg-Marquardt
   solves all six degrees of freedom on the consensus set with the camera's
   own Jacobians, and after every step re-selects the inliers among ALL
   matches by their normalised pixel residual. The gate starts at nine times
   the set's median squared residual and shrinks by 4 per step to the filter's
   2-DoF chi-square bound, so matches the planar approximation mis-sorted are
   corrected as the tilt is recovered. Convergence is a step below 1e-10 (rad,
   or fraction of height) with the inlier set unchanged at the final gate;
   the cap is 30 steps. With map_var the pixel covariance depends on the pose;
   it is held fixed while a step is judged and refreshed after it. The best
   five distinct consensus sets are refined in order of support, and the
   one with most inliers that passes step 4 is kept.

4. Checks. A fix is refused, with the reason, when there are fewer than four
   matches ('too few matches'), no hypothesis gathers three ('no consensus'),
   the iteration does not converge, the whitened Jacobian is ill-conditioned
   ('degenerate geometry'; collinear landmarks leave the rotation about their
   line free), the consensus is weak, the residuals are poor, the position is
   poorly determined, the camera ends up under the ground ('mirror solution')
   or the pose contradicts the tilt (45 degrees) and height (2/3 to 3/2 of the
   barometer) priors ('implausible pose'). Three of these need a word.

   Weak consensus. A match is evidence only if the others predict it. Left
   out, match i is predicted by the rest with covariance R_i (I - P_ii)^-1,
   P_ii being its 2 x 2 block of the hat matrix, so an unrelated match in its
   place would have passed the gate with probability q_i = pi GATE
   sqrt(det R_i)/(det(I - P_ii) image area). The fix is refused when
   (n - k + 1) max_i q_i, the expected number of the other matches that the
   least-checked member's place would admit by chance, exceeds MAX_CHANCE.
   This is what stops a few true matches with weak geometry from absorbing an
   alias along the tilt/translation valley: in 1500 scenes with 75 % outliers
   two fixes carried one such alias and were 20 m and 13 m out while claiming
   1 m (chance 0.026 and 0.012). The threshold is the largest that refused
   both; 1e-3 would also have refused 30 % of valid fixes with 11 true matches
   of 16.

   Poor residuals. The inliers' squared whitened residuals sum to a
   chi-square of 2k - 6 degrees of freedom. A sum beyond its 99.9 % point
   means the stated pixel_sigma is too small for what was fitted; the
   covariance would be optimistic, so the fix is refused.

   Poorly determined. The covariance is the fix's own account of its accuracy.
   A horizontal 1-sigma above MAX_SIGMA = 3 m is refused: the 10 m requirement
   would be 3.3 sigma away, and a fix with 5 m of sigma was 10 m out 1 time
   in 3000 tries at 50 % outliers.

The covariance is (H^T W H)^-1 of [dp, dtheta] at the solution: the
Cramer-Rao bound of Appendix H.5 with no prior. It describes noise on the
matches that were used and nothing else; it does not know about a consistent
set of aliases, which looks exactly like a good fix somewhere else. The
barometer is not an input to the fit, only to its start and plausibility.

Design-stage code checked on synthetic geometry only (code/tests). No real
image has been matched against a real map with it.
"""
import math
from dataclasses import dataclass

import numpy as np

from .eskf import ATT, BA, BARO, BG, GATE_2DOF, LEVEL, N_ERR, POS, TIME, VEL, exp_so3

MIN_MATCHES = 4            # three matches fix six unknowns; the fourth is the first check
TOLERANCE = 0.15           # consensus tolerance as a fraction of height (docstring, step 2)
SCALE_RANGE = (0.6, 1.8)   # plausible |z| of a hypothesis (docstring, step 2)
HEIGHT_RANGE = (1/1.5, 1.5)  # plausible solved height / barometric height
MIN_BASELINE = 0.2         # shortest hypothesis pair, fraction of height: short pairs amplify distortion
MAX_TILT = math.radians(45.0)
MAX_CONDITION = 1.0e4      # of the whitened Jacobian with position in units of height
MAX_CHANCE = 1.0e-2        # tolerated expected number of chance admissions (docstring, step 4)
MAX_RESIDUAL_Z = 3.09       # residual chi-square limit, standard normal quantile (0.1 % false alarms)
MAX_SIGMA = 3.0            # largest horizontal 1-sigma, m, of a fix worth starting the filter from
MAX_DRAWS = 500            # cap on two-point hypotheses scored
_KAPPA2 = 9.0              # gate = 3 x median normalised residual while it is shrinking
_STEP = 1.0e-10            # convergence: Gauss-Newton step, rad and fraction of height
_LAMBDA_MIN = 1.0e-12


@dataclass
class FirstFix:
    """Result of solve_first_fix. p, r and cov are None unless ok is True."""
    ok: bool
    reason: str                 # 'ok', or why there is no fix
    p: np.ndarray = None        # IMU position, world, m
    r: np.ndarray = None        # body-to-world rotation
    cov: np.ndarray = None      # 6 x 6 of [dp (m, world), dtheta (rad, body, right perturbation)]
    inliers: np.ndarray = None  # (n,) bool: matches inside the final gate
    rms_px: float = math.nan    # per-axis RMS pixel residual of the inliers
    hypotheses: int = 0         # two-point hypotheses scored
    tries: int = 0              # hypotheses refined
    iterations: int = 0         # Levenberg-Marquardt steps of the reported refinement
    condition: float = math.nan  # of the whitened Jacobian, position in units of height
    chance: float = math.nan    # expected chance admissions (docstring, step 4)
    chi2: float = math.nan      # sum of the inliers' squared whitened residuals; 2k - 6 degrees of freedom


def ransac_iterations(inlier_fraction=0.5, confidence=0.999, k=2):
    """Draws needed to see one outlier-free minimal set (Appendix G.4)."""
    return math.ceil(math.log(1 - confidence)/math.log(1 - max(inlier_fraction, 1e-3)**k))


def ground_coordinates(camera, pixels, height, r0=LEVEL):
    """Complex ground-plane coordinates of pixels seen from an assumed attitude.

    Relative to the point below the IMU, in the assumed frame's horizontal
    axes. The second result is False for rays that never reach the ground.
    """
    rays = camera.ray(pixels) @ r0.T
    lever = r0 @ camera.lever
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = (height + lever[2])/-rays[:, 2]
    hit = np.isfinite(depth) & (depth > 0)
    xy = lever[:2] + rays[:, :2]*np.where(hit, depth, 0.0)[:, None]
    return xy[:, 0] + 1j*xy[:, 1], hit


def chi2_limit(dof, z=MAX_RESIDUAL_Z):
    """Chi-square quantile for z standard deviations (Wilson-Hilferty, good to 1 % from 2 degrees of freedom)."""
    return dof*(1 - 2/(9*dof) + z*math.sqrt(2/(9*dof)))**3


def _fit_similarity(u, m, mask):
    """Least-squares m = z u + t over a set (closed form in complex numbers)."""
    uc, mc = u[mask].mean(), m[mask].mean()
    du = u[mask] - uc
    z = np.vdot(du, m[mask] - mc)/np.vdot(du, du).real
    return z, mc - z*uc


def _consensus(u, m, hit, z, t, tol):
    return hit & (abs(m - z*u - t) < tol*abs(z))


def _hypotheses(u, m, hit, height, tol, inlier_fraction, confidence, max_draws, rng):
    """Two-point similarity hypotheses, best supported first: z, t, votes and the number scored.

    Pairs are drawn without replacement from those with a usable baseline, so
    a small set is exhausted rather than repeated. The count starts at
    ransac_iterations(inlier_fraction) and is raised, as RANSAC does, to what
    the best vote so far implies: if no hypothesis gathers inlier_fraction of
    the matches the assumed fraction was too optimistic.
    """
    n = len(u)
    i, j = np.triu_indices(n, 1)
    keep = hit[i] & hit[j] & (abs(u[i] - u[j]) > MIN_BASELINE*height) & (abs(m[i] - m[j]) > 0)
    i, j = i[keep], j[keep]
    order = rng.permutation(len(i))
    limit = min(len(i), max_draws)
    need, drawn, best = ransac_iterations(inlier_fraction, confidence), 0, 0
    found = []
    while drawn < min(need, limit):
        pick = order[drawn:min(need, limit)]
        drawn += len(pick)
        z = (m[i[pick]] - m[j[pick]])/(u[i[pick]] - u[j[pick]])
        t = m[i[pick]] - z*u[i[pick]]
        scale = abs(z)
        err = abs(m[None, :] - z[:, None]*u[None, :] - t[:, None])/(tol*scale[:, None])
        inside = (err < 1) & hit[None, :]
        votes = inside.sum(axis=1)
        votes[(scale > SCALE_RANGE[1]) | (scale < SCALE_RANGE[0])] = 0
        # Ties in the vote are broken by how tightly the supporters agree.
        found.append((z, t, votes, np.sum(np.where(inside, err*err, 0.0), axis=1)))
        best = max(best, int(votes.max()))
        need = ransac_iterations(min(inlier_fraction, best/n), confidence)
    if not found:
        return np.zeros(0, complex), np.zeros(0, complex), np.zeros(0, int), 0
    z, t, votes, spread = (np.concatenate(x) for x in zip(*found))
    order = np.lexsort((spread, -votes))
    return z[order], t[order], votes[order], drawn


def _measure(camera, pixels, points, p, r, pixel_var, map_var, scale, chol=None):
    """Every match at a pose: whitened residuals (n, 2) and Jacobians (n, 2, 6),
    depth, raw residuals, sqrt(det) of each match's pixel covariance and the
    Cholesky factors that whitened them (None without map_var).

    Position columns are multiplied by scale (the height), so all six unknowns
    are dimensionless and the singular values of the Jacobian are comparable.
    Map error makes the pixel covariance depend on the pose; pass the factors
    of the pose being stepped from to compare costs under one fixed weighting.
    """
    predicted, depth, jp, jt = camera.jacobians(points, p, r)
    res = pixels - predicted
    h = np.concatenate((jp*scale, jt), axis=2)
    if map_var is None:
        w = 1/np.sqrt(pixel_var)
        return res*w[:, None], h*w[:, None, None], depth, res, pixel_var, None
    if chol is None:
        with np.errstate(over='ignore', invalid='ignore'):
            noise = (jp*map_var[:, None, :]) @ jp.transpose(0, 2, 1)      # as Eskf.landmark_model
            noise[:, 0, 0] += pixel_var
            noise[:, 1, 1] += pixel_var
            # A trial pose can put a landmark almost in the camera's own plane,
            # where its Jacobian is enormous and this 2 x 2 is singular to
            # rounding. Such a match is unusable at that pose, not an error.
            schur = noise[:, 1, 1] - noise[:, 0, 1]*noise[:, 1, 0]/noise[:, 0, 0]
            unusable = ~(np.isfinite(noise).all(axis=(1, 2)) & (noise[:, 0, 0] > 0)
                         & (schur > 1e-9*noise[:, 1, 1]))
        if unusable.any():
            noise[unusable] = np.eye(2)
            res = np.where(unusable[:, None], np.inf, res)
            h = np.where(unusable[:, None, None], 0.0, h)
        chol = np.linalg.cholesky(noise)
    with np.errstate(invalid='ignore'):
        return (np.linalg.solve(chol, res[:, :, None])[:, :, 0], np.linalg.solve(chol, h), depth, res,
                chol[:, 0, 0]*chol[:, 1, 1], chol)


def _refine(camera, pixels, points, p, r, active, pixel_var, map_var, scale, max_iter, image_area):
    """Levenberg-Marquardt on six degrees of freedom with inlier re-selection.

    Returns a FirstFix whose reason is 'ok' when the iteration converged; the
    caller still has to apply the acceptance checks.
    """
    ev = _measure(camera, pixels, points, p, r, pixel_var, map_var, scale)
    lam, gate, its, reason = 1e-3, math.inf, 0, 'ok'
    while True:
        rw, hw, depth, res, area, chol = ev
        d2 = np.sum(rw*rw, axis=1)
        stable = False
        if its:
            # The median tracks how far the fit still is from the noise floor;
            # the forced factor guarantees the final gate is reached, so a fit
            # dragged off by bad matches loses support and fails instead of
            # swallowing them.
            gate = max(GATE_2DOF, min(gate/4, _KAPPA2*float(np.median(d2[active]))))
            selected = (d2 < gate) & (depth > 0)
            stable = gate == GATE_2DOF and np.array_equal(selected, active)
            active = selected
        k = int(active.sum())
        if k < 3:
            return FirstFix(False, 'no consensus', inliers=active, iterations=its)
        ua, s, vt = np.linalg.svd(hw[active].reshape(2*k, 6), full_matrices=False)
        g = s*(ua.T @ rw[active].reshape(-1))                    # V^T H^T r
        condition = s[0]/s[-1] if s[-1] > 0 else math.inf
        if stable and condition < 1e12 and np.max(abs(vt.T @ (g/(s*s)))) < _STEP:
            break
        if its >= max_iter:
            reason = 'did not converge'
            break
        cost = float(d2[active].sum())
        for _ in range(12):
            delta = vt.T @ (g/(s*s + lam*s[0]*s[0]))
            p2, r2 = p + scale*delta[:3], r @ exp_so3(delta[3:])
            ev2 = _measure(camera, pixels, points, p2, r2, pixel_var, map_var, scale, chol)
            # The slack lets a step at the rounding floor through; without it
            # a converged fit with noisy pixels would be reported as stalled.
            if np.all(ev2[2][active] > 0) and float(np.sum(ev2[0][active]**2)) <= cost*(1 + 1e-9) + 1e-18:
                break
            lam *= 10
        else:
            reason = 'did not converge'
            break
        if map_var is not None:
            ev2 = _measure(camera, pixels, points, p2, r2, pixel_var, map_var, scale)    # reweight at the new pose
        p, r, ev, lam, its = p2, r2, ev2, max(lam/10, _LAMBDA_MIN), its + 1
    fix = FirstFix(False, reason, p, r, None, active, float(np.sqrt(np.mean(res[active]**2))),
                   iterations=its, condition=condition)
    if condition < 1e12:
        unit = np.r_[np.full(3, scale), np.ones(3)]
        cov = ((vt.T/(s*s)) @ vt)*np.outer(unit, unit)
        fix.cov = (cov + cov.T)/2          # exactly symmetric, as the filter keeps its own
        fix.chi2 = float(d2[active].sum())
        ub = ua.reshape(k, 2, 6)
        hat = ub @ ub.transpose(0, 2, 1)
        free = (1 - hat[:, 0, 0])*(1 - hat[:, 1, 1]) - hat[:, 0, 1]*hat[:, 1, 0]      # det(I - P_ii)
        with np.errstate(over='ignore'):                         # a match the rest say nothing about: infinite
            fix.chance = (len(active) - k + 1)*float(np.max(
                math.pi*GATE_2DOF*area[active]/np.maximum(free, 1e-300)))/image_area
    return fix


def solve_first_fix(camera, pixels, points, height, pixel_sigma=1.0, rng=None, *, map_var=None, r0=LEVEL,
                    tolerance=TOLERANCE, inlier_fraction=0.5, confidence=0.999, max_draws=MAX_DRAWS,
                    max_tries=5, max_iter=30, min_inliers=MIN_MATCHES, max_chance=MAX_CHANCE,
                    max_sigma=MAX_SIGMA, max_residual_z=MAX_RESIDUAL_Z, max_condition=MAX_CONDITION,
                    max_tilt=MAX_TILT):
    """Pose of the IMU from one image's landmark matches and the barometer.

    pixels (n, 2): undistorted pixel of each match. points (n, 3): its map
    position, world. height: barometric height of the IMU above the ground the
    landmarks lie on. pixel_sigma: per-axis pixel noise of a true match,
    scalar or (n,). rng: source of the hypothesis draws; the default is a
    fixed seed so that the same matches always give the same answer.
    map_var: optional east/north/up landmark variances, (3,) or (n, 3),
    mapped into pixels as Eskf.landmark_model does. r0: assumed attitude, any
    yaw. inlier_fraction and confidence set the first batch of hypotheses
    (ransac_iterations); max_sigma is the largest horizontal 1-sigma, m, a
    fix may claim for itself.

    Returns a FirstFix. Check ok: on failure there is no pose, only a reason.
    """
    pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    total = len(pixels)
    sigma = np.broadcast_to(np.asarray(pixel_sigma, dtype=float), (total,))
    if len(points) != total or not height > 0 or not np.all(sigma > 0):
        raise ValueError('need one map point per pixel, a positive height and positive pixel sigmas')
    # A match with a missing coordinate is never evidence, and never an inlier.
    finite = np.isfinite(pixels).all(axis=1) & np.isfinite(points).all(axis=1)
    pixels, points, pixel_var = pixels[finite], points[finite], sigma[finite]**2
    n = len(pixels)
    none = np.zeros(total, dtype=bool)
    if n < max(min_inliers, MIN_MATCHES):
        return FirstFix(False, 'too few matches', inliers=none)
    if map_var is not None:
        map_var = np.broadcast_to(np.asarray(map_var, dtype=float), (total, 3))[finite]
    ground = float(np.median(points[:, 2]))
    u, hit = ground_coordinates(camera, pixels, height, r0)
    m = points[:, 0] + 1j*points[:, 1]
    rng = np.random.default_rng(0) if rng is None else rng
    z, t, votes, scored = _hypotheses(u, m, hit, height, tolerance*height, inlier_fraction, confidence,
                                      max_draws, rng)
    image_area = float(camera.width*camera.height) or 4.0*camera.cx*camera.cy
    best, tries, seen = FirstFix(False, 'no consensus', inliers=np.zeros(n, dtype=bool)), 0, []
    for zi, ti, vi in zip(z, t, votes):
        if vi < 3 or tries >= max_tries:
            break
        # Polish the two-point guess on its own supporters before using it.
        mask = _consensus(u, m, hit, zi, ti, tolerance*height)
        zi, ti = _fit_similarity(u, m, mask)
        mask = _consensus(u, m, hit, zi, ti, tolerance*height)
        if mask.sum() < 3 or any(np.array_equal(mask, other) for other in seen):
            continue
        seen.append(mask)
        tries += 1
        yaw = math.atan2(zi.imag, zi.real)
        c, s = math.cos(yaw), math.sin(yaw)
        fix = _refine(camera, pixels, points, np.array([ti.real, ti.imag, ground + height]),
                      np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ r0, mask,
                      pixel_var, map_var, height, max_iter, image_area)
        k = int(fix.inliers.sum())
        if fix.reason == 'ok':
            if k < min_inliers:
                fix.reason = 'no consensus'
            elif fix.condition > max_condition:
                fix.reason = 'degenerate geometry'
            elif fix.chance > max_chance:
                fix.reason = 'weak consensus'
            elif fix.chi2 > chi2_limit(2*k - 6, max_residual_z):
                fix.reason = 'poor residuals'
            elif np.sqrt(np.linalg.eigvalsh(fix.cov[:2, :2])[-1]) > max_sigma:
                fix.reason = 'poorly determined'
            elif (fix.p + fix.r @ camera.lever)[2] <= np.max(points[fix.inliers, 2]):
                fix.reason = 'mirror solution'
            elif (-fix.r[2, 2] < math.cos(max_tilt)
                  or not HEIGHT_RANGE[0] < (fix.p[2] - ground)/height < HEIGHT_RANGE[1]):
                fix.reason = 'implausible pose'
        elif fix.reason == 'did not converge' and k >= min_inliers and fix.condition > max_condition:
            fix.reason = 'degenerate geometry'
        fix.ok = fix.reason == 'ok'
        # Keep the best-supported fix, or failing that the failure that came closest.
        if (fix.ok, k) > (best.ok, int(best.inliers.sum())) or tries == 1:
            best = fix
        # A strict majority cannot be outvoted; a minority might be, so keep looking.
        if best.ok and 2*int(best.inliers.sum()) > n:
            break
    best.hypotheses, best.tries = scored, tries
    if best.ok:
        used, best.inliers = best.inliers, none.copy()
        best.inliers[finite] = used
    else:
        best.p = best.r = best.cov = None
        best.inliers, best.rms_px = none, math.nan
    return best


def initial_covariance(fix, vel_sigma, accel_bias_sigma, gyro_bias_sigma, baro_bias_sigma,
                       time_offset_sigma=0.002):
    """Filter covariance (N_ERR x N_ERR) to start an Eskf from a first fix.

    Position, attitude and their cross-covariance come from the fix. One image
    says nothing about velocity, the sensor biases or the camera time offset,
    so those are the caller's priors (scalars, or per-axis sigmas for the
    vectors) and start uncorrelated with everything else.
    """
    if not fix.ok:
        raise ValueError('no first fix: ' + fix.reason)
    cov = np.zeros((N_ERR, N_ERR))
    cov[POS, POS] = fix.cov[:3, :3]
    cov[POS, ATT] = fix.cov[:3, 3:]
    cov[ATT, POS] = fix.cov[3:, :3]
    cov[ATT, ATT] = fix.cov[3:, 3:]
    for block, sigma in ((VEL, vel_sigma), (BA, accel_bias_sigma), (BG, gyro_bias_sigma)):
        cov[block, block] = np.diag(np.broadcast_to(np.asarray(sigma, dtype=float), (3,))**2)
    cov[BARO, BARO] = float(baro_bias_sigma)**2
    cov[TIME, TIME] = float(time_offset_sigma)**2
    return cov
