"""GPL-3.0-only. Error-state Kalman filter for map-relative navigation.

Fuses the IMU, barometric height and individual map landmarks seen by the
downward camera. GPS never enters this filter; it is the truth it is scored
against. The mechanisation follows the mission proposal's Appendix H.

Frames. World is local east/north/up with its origin at the launch pad. Body
axes are fixed to the can with z out of the bottom along the camera boresight,
so a level can has R = diag(1, -1, -1). R maps body to world. The navigated
point is the IMU.

Errors are [dp, dv, dtheta, dba, dbg, dbaro, dt_cam] (17) with the local
(right) attitude error R = R_hat (I + [dtheta]x):

    d(dp)/dt     = dv
    d(dv)/dt     = -R [a]x dtheta - R dba - R n_a
    d(dtheta)/dt = -[w]x dtheta - dbg - n_g

Two states beyond Appendix H: a barometric height bias, and the camera time
offset (an image stamped t was exposed at t + offset). Simulation showed the
proposal's plain "one EKF update per image, chi-square gate per landmark" to be
fragile, so the landmark update here can also iterate, reject on post-fit
residuals and widen its own prior when most landmarks disagree with it; see
update_landmarks. Each of those is optional so the plain form stays testable.

This is a design-stage implementation. Its consistency has been checked only
in simulation (sim/nav); no flight or bench data exist yet.
"""
import math
from dataclasses import dataclass, field

import numpy as np

G0 = 9.80665
GRAVITY = np.array([0.0, 0.0, -G0])
N_ERR = 17
POS, VEL, ATT, BA, BG = slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12), slice(12, 15)
BARO = 15
TIME = 16
# Chi-square acceptance bounds at 99.7 %: -2 ln(0.003) for two degrees of
# freedom (Appendix H.3) and the one-degree value for the barometer.
GATE_2DOF = -2*math.log(0.003)
GATE_1DOF = 8.8075
LEVEL = np.diag([1.0, -1.0, -1.0])


def skew(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def exp_so3(v):
    """Rotation matrix of a rotation vector (Rodrigues)."""
    angle2 = float(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    s = skew(v)
    if angle2 < 1e-16:
        return np.eye(3) + s + s @ s/2
    angle = math.sqrt(angle2)
    return np.eye(3) + math.sin(angle)/angle*s + (1 - math.cos(angle))/angle2*(s @ s)


def log_so3(r):
    """Rotation vector of a rotation matrix, valid away from 180 degrees."""
    cos = min(1.0, max(-1.0, (r[0, 0] + r[1, 1] + r[2, 2] - 1)/2))
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])/2
    angle = math.acos(cos)
    return v if angle < 1e-7 else angle/math.sin(angle)*v


@dataclass
class ImuNoise:
    """Continuous-time densities in SI units per root hertz."""
    accel: float = 2.0e-3          # m/s^2/sqrt(Hz)
    gyro: float = 2.5e-4           # rad/s/sqrt(Hz)
    accel_bias: float = 1.0e-4     # m/s^3/sqrt(Hz)
    gyro_bias: float = 2.0e-6      # rad/s^2/sqrt(Hz)
    baro_bias: float = 0.02        # m/sqrt(s)
    # Gyro scale-factor and cross-axis error (1 sigma, fractional). It makes a
    # rate error proportional to the rate, which a constant bias state cannot
    # follow through swing and spin; it is covered as attitude noise that acts
    # coherently for scale_tau seconds (about one image interval).
    gyro_scale: float = 0.0
    scale_tau: float = 0.2
    time_offset: float = 0.0       # s/sqrt(s), random walk of the camera time offset


@dataclass
class Camera:
    """Pinhole camera. Lens distortion is removed from pixels before use."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 0
    height: int = 0
    r_cb: np.ndarray = field(default_factory=lambda: np.eye(3))    # camera from body
    lever: np.ndarray = field(default_factory=lambda: np.zeros(3))  # optical centre from IMU, body axes, m

    def body_points(self, points, p, r):
        """Map points (n, 3) expressed in body axes from the IMU."""
        return (points - p) @ r

    def project(self, points, p, r):
        """Pixels (n, 2) and camera-frame depth (n,) of world points."""
        pc = (self.body_points(points, p, r) - self.lever) @ self.r_cb.T
        z = pc[:, 2]
        return np.column_stack((self.fx*pc[:, 0]/z + self.cx, self.fy*pc[:, 1]/z + self.cy)), z

    def ray(self, pixels):
        """Unit-depth body-frame directions (n, 3) of pixels."""
        pc = np.column_stack(((pixels[:, 0] - self.cx)/self.fx, (pixels[:, 1] - self.cy)/self.fy,
                              np.ones(len(pixels))))
        return pc @ self.r_cb

    def jacobians(self, points, p, r):
        """Pixels, depth, d(pixel)/d(dp) and d(pixel)/d(dtheta), each (n, 2, 3).

        The attitude derivative uses the lever-free body vector R^T (m - p):
        rotating the body about the IMU also carries the camera round it.
        """
        pb = self.body_points(points, p, r)
        pc = (pb - self.lever) @ self.r_cb.T
        x, y, z = pc[:, 0], pc[:, 1], pc[:, 2]
        jpi = np.zeros((len(points), 2, 3))
        jpi[:, 0, 0] = self.fx/z
        jpi[:, 0, 2] = -self.fx*x/(z*z)
        jpi[:, 1, 1] = self.fy/z
        jpi[:, 1, 2] = -self.fy*y/(z*z)
        jb = jpi @ self.r_cb                                    # d(pixel)/d(body vector)
        pbx = np.zeros((len(points), 3, 3))
        pbx[:, 0, 1], pbx[:, 0, 2] = -pb[:, 2], pb[:, 1]
        pbx[:, 1, 0], pbx[:, 1, 2] = pb[:, 2], -pb[:, 0]
        pbx[:, 2, 0], pbx[:, 2, 1] = -pb[:, 1], pb[:, 0]
        pixels = np.column_stack((self.fx*x/z + self.cx, self.fy*y/z + self.cy))
        return pixels, z, -jb @ r.T, jb @ pbx


class Eskf:
    # Added to the position and attitude variances when most landmarks in an
    # image disagree with the prior: the matcher only finds landmarks within
    # its search window, so the true error cannot be much larger than this.
    RECOVER_POS_M = 16.0
    RECOVER_ATT_RAD = math.radians(3.0)
    # Consecutive images in which most landmarks fail the gate before the
    # prior is doubted. One bad image is far more likely than a wrong filter.
    RECOVER_FRAMES = 3

    def __init__(self, p, v, r, cov, noise=None, ba=None, bg=None, baro_bias=0.0, time_offset=0.0):
        self.p = np.array(p, dtype=float)
        self.v = np.array(v, dtype=float)
        self.r = np.array(r, dtype=float)
        self.ba = np.zeros(3) if ba is None else np.array(ba, dtype=float)
        self.bg = np.zeros(3) if bg is None else np.array(bg, dtype=float)
        self.baro_bias = float(baro_bias)
        self.time_offset = float(time_offset)
        self.cov = np.array(cov, dtype=float)
        if self.cov.shape != (N_ERR, N_ERR):
            raise ValueError('covariance must be %d x %d' % (N_ERR, N_ERR))
        self.noise = noise or ImuNoise()
        self.rate = np.zeros(3)         # last bias-corrected body rate, for the camera time offset
        self._steps = 0
        self._starved = 0               # consecutive images with most landmarks rejected

    # --- Propagation ---------------------------------------------------------

    def propagate(self, dt, accel, gyro):
        """Advance by dt with the mean specific force and rate over the interval.

        accel is the interval's velocity increment divided by dt, expressed in
        the body axes at mid-interval; gyro is its rotation vector divided by
        dt. With increments formed that way the nominal update is exact for
        velocity and second order for position.
        """
        a = accel - self.ba
        w = gyro - self.bg
        dr = exp_so3(w*dt)
        r_half = exp_so3(w*(dt/2))
        r_mid = self.r @ r_half
        acc_w = r_mid @ a + GRAVITY
        self.p = self.p + self.v*dt + acc_w*(dt*dt/2)
        self.v = self.v + acc_w*dt

        # The attitude error is expressed in start-of-step body axes, so the
        # specific force is rotated back to them: R_mid [a]x R_half^T = R [R_half a]x.
        ra = self.r @ skew(r_half @ a)
        phi = np.eye(N_ERR)
        phi[POS, VEL] = np.eye(3)*dt
        phi[POS, ATT] = -ra*(dt*dt/2)
        phi[POS, BA] = -r_mid*(dt*dt/2)
        phi[VEL, ATT] = -ra*dt
        phi[VEL, BA] = -r_mid*dt
        phi[VEL, BG] = ra*(dt*dt/2)
        phi[ATT, ATT] = dr.T
        phi[ATT, BG] = -r_half.T*dt
        n = self.noise
        q = np.zeros(N_ERR)
        q[VEL] = n.accel**2*dt
        q[ATT] = (n.gyro**2 + n.gyro_scale**2*float(w @ w)*n.scale_tau)*dt
        q[BA] = n.accel_bias**2*dt
        q[BG] = n.gyro_bias**2*dt
        q[BARO] = n.baro_bias**2*dt
        q[TIME] = n.time_offset**2*dt
        cov = phi @ self.cov @ phi.T
        cov[np.diag_indices(N_ERR)] += q
        self.cov = (cov + cov.T)/2
        self.r = self.r @ dr
        self.rate = w
        self._steps += 1
        if self._steps % 500 == 0:
            u, _, vt = np.linalg.svd(self.r)
            self.r = u @ vt

    # --- Updates -------------------------------------------------------------

    def _state(self):
        return (self.p.copy(), self.v.copy(), self.r.copy(), self.ba.copy(), self.bg.copy(),
                self.baro_bias, self.time_offset)

    def _set(self, state, dx=None):
        """Restore a saved state, optionally displaced by an error vector."""
        p, v, r, ba, bg, baro, offset = state
        if dx is None:
            self.p, self.v, self.r, self.ba, self.bg = p.copy(), v.copy(), r.copy(), ba.copy(), bg.copy()
            self.baro_bias, self.time_offset = baro, offset
        else:
            self.p, self.v, self.r = p + dx[POS], v + dx[VEL], r @ exp_so3(dx[ATT])
            self.ba, self.bg = ba + dx[BA], bg + dx[BG]
            self.baro_bias, self.time_offset = baro + float(dx[BARO]), offset + float(dx[TIME])

    def _commit(self, dx, cov, reset=True):
        """Inject a correction and store its covariance.

        A covariance linearised at the prior is expressed in the old attitude
        axes and needs the reset I - [dtheta]x/2 to move it to the new ones.
        One linearised at the corrected state (an iterated update) is already
        in the new axes; resetting it again was measurably wrong for 5-10
        degree corrections (independent check, sim/nav/test_independent_checks.py).
        """
        self._set(self._state(), dx)
        if reset:
            jacobian = np.eye(N_ERR)
            jacobian[ATT, ATT] = np.eye(3) - skew(dx[ATT])/2
            cov = jacobian @ cov @ jacobian.T
        self.cov = (cov + cov.T)/2

    def _update(self, residual, h, noise):
        """Joseph-form update."""
        cov = self.cov
        s = h @ cov @ h.T + noise
        gain = np.linalg.solve(s, h @ cov).T
        dx = gain @ residual
        ikh = np.eye(N_ERR) - gain @ h
        self._commit(dx, ikh @ cov @ ikh.T + gain @ noise @ gain.T)
        return dx

    def update_baro(self, height_m, sigma_m, gate=GATE_1DOF):
        """Barometric height of the IMU above the pad datum. Returns (accepted, nis)."""
        h = np.zeros((1, N_ERR))
        h[0, 2] = 1.0
        h[0, BARO] = 1.0
        residual = np.array([height_m - self.p[2] - self.baro_bias])
        noise = np.array([[sigma_m*sigma_m]])
        nis = float(residual[0]**2/(h @ self.cov @ h.T + noise)[0, 0])
        if nis > gate:
            return False, nis
        self._update(residual, h, noise)
        return True, nis

    def landmark_model(self, camera, points, pixel_var, map_var=None):
        """Predicted pixels, stacked Jacobians (n, 2, 17) and noise (n, 2, 2).

        The prediction is made for the exposure instant: the pose is carried
        forward by the time-offset estimate at the current velocity and rate,
        and the offset's own column is the pixel velocity.

        pixel_var: variance per pixel axis, scalar or (n,).
        map_var: variances (3,) or (n, 3) of each landmark's east/north/up map
        error in m^2. They are mapped into pixels and treated as independent
        between landmarks and frames, which is optimistic for a landmark seen
        in consecutive frames.
        """
        r_offset = exp_so3(self.rate*self.time_offset)
        predicted, depth, jp, jt = camera.jacobians(points, self.p + self.v*self.time_offset, self.r @ r_offset)
        n = len(points)
        h = np.zeros((n, 2, N_ERR))
        h[:, :, POS] = jp
        h[:, :, VEL] = jp*self.time_offset
        h[:, :, ATT] = jt @ r_offset.T
        h[:, :, TIME] = jp @ self.v + jt @ self.rate
        noise = np.zeros((n, 2, 2))
        var = np.broadcast_to(np.asarray(pixel_var, dtype=float), (n,))
        noise[:, 0, 0] = noise[:, 1, 1] = var
        if map_var is not None:
            mv = np.broadcast_to(np.asarray(map_var, dtype=float), (n, 3))
            noise += (jp*mv[:, None, :]) @ jp.transpose(0, 2, 1)   # d(pixel)/dm = -d(pixel)/d(dp)
        return predicted, depth, h, noise

    @staticmethod
    def _quadratic(residual, s):
        """r^T S^-1 r for stacks of 2-vectors and 2 x 2 matrices."""
        det = s[:, 0, 0]*s[:, 1, 1] - s[:, 0, 1]*s[:, 1, 0]
        return (s[:, 1, 1]*residual[:, 0]**2 - (s[:, 0, 1] + s[:, 1, 0])*residual[:, 0]*residual[:, 1]
                + s[:, 0, 0]*residual[:, 1]**2)/det

    def _innovations(self, camera, points, pixels, pixel_var, map_var):
        predicted, depth, h, noise = self.landmark_model(camera, points, pixel_var, map_var)
        residual = pixels - predicted
        nis = self._quadratic(residual, h @ self.cov @ h.transpose(0, 2, 1) + noise)
        return residual, depth, h, noise, nis

    def _solve(self, camera, points, pixels, pixel_var, map_var, iterations):
        """Iterated update from the current state, without committing it.

        Returns the correction, its covariance, each landmark's post-fit
        residual normalised by its measurement noise, and the number of
        linearisation passes (after one pass the covariance is still in the
        prior's attitude axes).
        """
        prior, prior_cov = self._state(), self.cov
        prior_sigma = np.sqrt(np.diag(prior_cov))
        m = len(points)
        dx = np.zeros(N_ERR)
        big_noise = np.zeros((2*m, 2*m))
        passes = 0
        try:
            for _ in range(max(iterations, 1)):
                passes += 1
                self._set(prior, dx)
                predicted, _, h, noise = self.landmark_model(camera, points, pixel_var, map_var)
                hm = h.reshape(2*m, N_ERR)
                for j, block in enumerate(noise):
                    big_noise[2*j:2*j + 2, 2*j:2*j + 2] = block
                gain = np.linalg.solve(hm @ prior_cov @ hm.T + big_noise, hm @ prior_cov).T
                new = gain @ ((pixels - predicted).reshape(-1) + hm @ dx)
                # Stop once a pass moves no state by more than 1 % of its prior sigma.
                converged = np.all(np.abs(new - dx) < 0.01*prior_sigma)
                dx = new
                if converged:
                    break
            ikh = np.eye(N_ERR) - gain @ hm
            cov = ikh @ prior_cov @ ikh.T + gain @ big_noise @ gain.T
            self._set(prior, dx)
            predicted, _, _, noise = self.landmark_model(camera, points, pixel_var, map_var)
            post = self._quadratic(pixels - predicted, noise)
        finally:
            self._set(prior)
        return dx, cov, post, passes

    def update_landmarks(self, camera, points, pixels, pixel_var, map_var=None, gate=GATE_2DOF,
                         iterations=1, robust=False, recover=False):
        """Gate each landmark on its own innovation, then update with the rest.

        With the defaults this is the proposal's update: one first-order
        correction from the landmarks that pass the chi-square gate.

        iterations > 1 relinearises (iterated EKF). A single pass is
        overconfident whenever the correction is large, as it is straight
        after a first fix.
        robust re-tests the accepted landmarks against the solution they
        produced and repeats without those that fail. The prior gate is wide
        when the prior is, and lets gross mismatches through.
        recover handles the opposite failure. If fewer than half the landmarks
        pass the gate in RECOVER_FRAMES consecutive images the prior, not the
        image, is taken to be wrong: position and attitude variances are
        widened and the gate is applied again. The result is kept only if
        most landmarks then agree with each other; otherwise the image was the
        problem (mostly mismatches) and the filter is left as it was. It
        cannot detect a majority of mismatches that agree among themselves.

        Returns a dict: 'accepted' mask, each landmark's prior normalised
        innovation squared 'nis', and 'recovered'. Rejected landmarks are
        reported, never silently dropped.
        """
        points = np.asarray(points, dtype=float)
        pixels = np.asarray(pixels, dtype=float)
        n = len(points)
        info = {'accepted': np.zeros(n, dtype=bool), 'nis': np.zeros(n), 'recovered': False}
        if n == 0:
            return info
        pixel_var = np.broadcast_to(np.asarray(pixel_var, dtype=float), (n,))
        if map_var is not None:
            map_var = np.broadcast_to(np.asarray(map_var, dtype=float), (n, 3))
        residual, depth, h, noise, nis = self._innovations(camera, points, pixels, pixel_var, map_var)
        accepted = (depth > 0) & (nis < gate)
        info['nis'] = nis
        self._starved = self._starved + 1 if n >= 6 and accepted.sum() < n/2 else 0
        trusted_cov = None
        if recover and self._starved >= self.RECOVER_FRAMES:
            trusted_cov = self.cov
            widen = np.zeros(N_ERR)
            widen[POS] = self.RECOVER_POS_M**2
            widen[ATT] = self.RECOVER_ATT_RAD**2
            self.cov = self.cov + np.diag(widen)
            residual, depth, h, noise, nis_wide = self._innovations(camera, points, pixels, pixel_var, map_var)
            accepted = (depth > 0) & (nis_wide < gate)
        info['accepted'] = accepted
        dx = None
        if accepted.any() and iterations <= 1 and not robust:
            m = int(accepted.sum())
            big_noise = np.zeros((2*m, 2*m))
            for j, block in enumerate(noise[accepted]):
                big_noise[2*j:2*j + 2, 2*j:2*j + 2] = block
            hm = h[accepted].reshape(2*m, N_ERR)
            gain = np.linalg.solve(hm @ self.cov @ hm.T + big_noise, hm @ self.cov).T
            dx = gain @ residual[accepted].reshape(-1)
            ikh = np.eye(N_ERR) - gain @ hm
            cov, passes = ikh @ self.cov @ ikh.T + gain @ big_noise @ gain.T, 1
        elif accepted.any():
            while True:
                dx, cov, post, passes = self._solve(camera, points[accepted], pixels[accepted], pixel_var[accepted],
                                                    None if map_var is None else map_var[accepted], iterations)
                if not robust or accepted.sum() <= 4 or post.max() <= gate:
                    break
                # Drop the worst group only: a gross mismatch drags the solution
                # and inflates its neighbours' residuals too.
                index = np.flatnonzero(accepted)
                accepted[index[post > max(gate, post.max()/4)]] = False
        if trusted_cov is not None:
            if accepted.sum() < n/2:
                # Even a wide prior finds no majority that agrees: keep the filter as it was.
                self.cov = trusted_cov
                accepted[:] = False
                return info
            info['recovered'] = True
            self._starved = 0
        if dx is not None:
            self._commit(dx, cov, reset=passes == 1)
        return info

    # --- Outputs -------------------------------------------------------------

    def protection_level(self, k=math.sqrt(-2*math.log(0.05))):
        """Horizontal protection level k*sqrt(lambda_max), Appendix D.

        The default k bounds a Gaussian radial error with 95 % probability
        when the covariance is isotropic and is conservative otherwise.
        """
        cxx, cxy, cyy = self.cov[0, 0], self.cov[0, 1], self.cov[1, 1]
        return k*math.sqrt((cxx + cyy)/2 + math.hypot((cxx - cyy)/2, cxy))

    def error_from(self, p, v, r, ba=None, bg=None, baro_bias=None, time_offset=None):
        """Error state of a reference (truth) relative to this estimate."""
        dx = np.zeros(N_ERR)
        dx[POS] = p - self.p
        dx[VEL] = v - self.v
        dx[ATT] = log_so3(self.r.T @ r)
        if ba is not None:
            dx[BA] = ba - self.ba
        if bg is not None:
            dx[BG] = bg - self.bg
        if baro_bias is not None:
            dx[BARO] = baro_bias - self.baro_bias
        if time_offset is not None:
            dx[TIME] = time_offset - self.time_offset
        return dx
