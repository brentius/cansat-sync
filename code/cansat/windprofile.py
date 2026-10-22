"""GPL-3.0-only. Wind profile from the can's own motion: live filter and post-flight retrieval.

The mission proposal's Appendix J model as a continuous-discrete linear Kalman
filter. Each horizontal axis carries

    x1, x2   drop-Dryden turbulence in the eddy-scaled height coordinate (App. I.5)
    u        the can's horizontal velocity, lagging the wind by lag_time
    c        the mean wind at the reference height (unknown, log-law or constant shape)
    g        canopy glide: the air-relative velocity the system trims at
    e        a correlated error in the velocity measurement (the navigation filter's)

and two more states hold a body-fixed lateral offset of the accelerometer:

    wind      w = c*shape(z) + sigma*(x1 + sqrt(3)*x2)
    du/dt     = (w + g - u)/lag_time
    velocity  y = u + gain*e + noise                         from TRN or GPS
    airspeed  q = u - w + offset_map @ offset + noise        from lateral specific force

The airspeed measurement is what makes glide observable. A suspension line can
only pull along its length, and a hanging can lines its axis up with the line,
so averaged over its wobble the lateral specific force in the can's own axes is
the can's own aerodynamic side force over its mass; divided by the can's
side-force rate constant that is its sideways airspeed (lateral_airspeed
below). Sample by sample it is not: the line sits a degree or two off the axis
while the can rocks, and that term is many times larger, so the average must
span several periods of the slowest swing. Without the airspeed g and c cannot
be told apart and glide passes straight into the wind.

Beyond Appendix J: the mean wind, the glide and the offset are estimated rather
than assumed, the release velocity is unknown, and the velocity error is
coloured. Each is switched off by a zero prior width, which leaves J.1.

Where it runs. WindFilter is the flight part: one epoch at a time, fixed-size
14-state arithmetic, no allocation that grows with the flight. It is meant for
the Pi Zero 2 W beside the navigation filter (cansat/landing.py drives it) and
needs only NumPy, which Raspberry Pi OS ships; nothing here is specific to a
NumPy version later than 1.24. retrieve() adds the Rauch-Tung-Striebel smoother
and layer means and is post-flight analysis for a laptop. lateral_airspeed
works on logged 100 Hz IMU samples and is also post-flight. None of it can run
on the STM32F405, which has no Python; a predictor that must survive the loss
of the Pi would need a C port of WindFilter (see landing.py).

Design-stage code checked only in simulation (sim/nav/run_wind_mc.py). Every
default is an assumption; none is a measured property of the CanSat.
"""
import math
from dataclasses import dataclass

import numpy as np

G0 = 9.80665
SQRT3 = math.sqrt(3.0)
N_AXIS = 6                      # states per horizontal axis
X1, X2, U, C, G, E = range(N_AXIS)
N_STATE = 2*N_AXIS + 2          # two axes plus the body-fixed lateral offset
OFFSET = slice(2*N_AXIS, N_STATE)
FREE_VELOCITY_DIFFUSION = 900.0  # (m/s)^2/s on u when there is no canopy model


@dataclass
class WindModel:
    """What the filter assumes. SI units; heights above ground."""
    sigma: float = 1.0               # m/s, turbulence standard deviation of each horizontal component
    eddy_slope: float = 0.5          # eddy scale L = eddy_slope*max(z, eddy_floor)
    eddy_floor: float = 1.0          # m
    lag_time: float | None = 10.5/G0  # s, v_d/g; None drops the canopy model (u is then unconstrained)
    roughness: float | None = 0.1    # m, log-law mean shape; None is a height-independent mean
    reference_height: float = 10.0   # m, where the shape is 1
    mean_sd: float = 10.0            # m/s, prior width of c on each axis
    release_sd: float = 15.0         # m/s, prior width of the first velocity
    glide_sd: float = 0.0            # m/s, stationary width of g; 0 removes the glide state
    glide_tau: float = 20.0          # s, how long a glide direction persists
    error_sd: float = 0.0            # stationary width of e; 0 makes the velocity noise white
    error_tau: float = 1.6           # s
    offset_sd: float = 0.0           # m/s at the reference speed, prior width of the lateral offset
    max_dzeta: float = 0.05          # largest eddy-coordinate step taken with frozen coefficients
    max_substeps: int | None = None  # cap on sub-steps per epoch (flight: bounds the cost near the ground)

    def length(self, z):
        return self.eddy_slope*np.maximum(z, self.eddy_floor)

    def zeta(self, z):
        """Eddy-scaled coordinate, the integral of dz/L from the ground."""
        z = np.maximum(np.asarray(z, dtype=float), 0.0)
        a, f = self.eddy_slope, self.eddy_floor
        return np.where(z <= f, z/(a*f), 1/a + np.log(np.maximum(z, f)/f)/a)

    def shape(self, z):
        """Mean-wind profile shape, 1 at the reference height."""
        z = np.asarray(z, dtype=float)
        if self.roughness is None:
            return np.ones_like(z)
        floor = self.roughness*math.e**0.01     # keeps the log law non-negative at the ground
        return np.log(np.maximum(z, floor)/self.roughness)/math.log(self.reference_height/self.roughness)

    def wind_row(self, z, axis):
        """Row h with wind = h @ state for one axis at height z."""
        h = np.zeros(N_STATE)
        base = axis*N_AXIS
        h[base + X1] = self.sigma
        h[base + X2] = self.sigma*SQRT3
        h[base + C] = float(self.shape(z))
        return h


def expm(a):
    """Matrix exponential by scaling and squaring a Taylor series (small matrices)."""
    norm = float(np.abs(a).sum(axis=0).max())
    scale = max(0, int(math.ceil(math.log2(norm/0.25)))) if norm > 0.25 else 0
    b = a/2**scale
    out = np.eye(len(a)) + b
    term = b
    for k in range(2, 14):
        term = term @ b/k
        out = out + term
    for _ in range(scale):
        out = out @ out
    return out


def _axis_matrices(model, rate, shape, drift):
    """Drift and diffusion of one axis; rate is d(zeta)/dt. drift appends the integral of u."""
    n = N_AXIS + (1 if drift else 0)
    a = np.zeros((n, n))
    d = np.zeros((n, n))
    a[X1, X2] = rate
    a[X2, X1], a[X2, X2] = -rate, -2*rate
    d[X2, X2] = rate
    if model.lag_time is None:
        d[U, U] = FREE_VELOCITY_DIFFUSION
    else:
        k = 1/model.lag_time
        a[U, X1], a[U, X2], a[U, C], a[U, G], a[U, U] = k*model.sigma, k*model.sigma*SQRT3, k*shape, k, -k
    a[G, G] = -1/model.glide_tau
    d[G, G] = 2*model.glide_sd**2/model.glide_tau
    a[E, E] = -1/model.error_tau
    d[E, E] = 2*model.error_sd**2/model.error_tau
    if drift:
        a[N_AXIS, U] = 1.0
    return a, d


def axis_step(model, z_from, z_to, dt, drift=False):
    """Transition and process noise of one axis over dt while the height changes z_from -> z_to.

    Coefficients are frozen at the middle of sub-steps no longer than
    max_dzeta in the eddy coordinate, so the fast decorrelation near the
    ground is resolved whatever the sampling interval.
    """
    if dt <= 0:
        raise ValueError('time must advance')
    span = abs(float(model.zeta(z_from) - model.zeta(z_to)))
    sub = max(1, int(math.ceil(span/model.max_dzeta)))
    if model.max_substeps:
        sub = min(sub, model.max_substeps)
    n = N_AXIS + (1 if drift else 0)
    phi, q = np.eye(n), np.zeros((n, n))
    h = dt/sub
    edges = np.linspace(z_from, z_to, sub + 1)
    zeta = model.zeta(edges)
    for j in range(sub):
        mid = 0.5*(edges[j] + edges[j + 1])
        a, d = _axis_matrices(model, abs(float(zeta[j + 1] - zeta[j]))/h, float(model.shape(mid)), drift)
        block = np.zeros((2*n, 2*n))
        block[:n, :n], block[:n, n:], block[n:, n:] = a, d, -a.T
        ex = expm(block*h)
        f = ex[:n, :n]
        step_q = ex[:n, n:] @ f.T
        phi = f @ phi
        q = f @ q @ f.T + (step_q + step_q.T)/2
    return phi, q


def full_step(model, z_from, z_to, dt):
    phi6, q6 = axis_step(model, z_from, z_to, dt)
    phi, q = np.eye(N_STATE), np.zeros((N_STATE, N_STATE))
    for axis in range(2):
        s = slice(axis*N_AXIS, (axis + 1)*N_AXIS)
        phi[s, s], q[s, s] = phi6, q6
    return phi, q


def _update(mean, cov, y, h, noise):
    """Joseph-form update. Returns mean, covariance, innovation and its covariance."""
    s = h @ cov @ h.T + noise
    gain = np.linalg.solve(s, h @ cov).T
    nu = y - h @ mean
    ikh = np.eye(len(mean)) - gain @ h
    cov = ikh @ cov @ ikh.T + gain @ noise @ gain.T
    return mean + gain @ nu, (cov + cov.T)/2, nu, s


def initial_state(model, anemometer=None):
    """Prior mean and covariance. anemometer = (height, (east, north), sd) informs the mean wind."""
    mean = np.zeros(N_STATE)
    var = np.zeros(N_STATE)
    for axis in range(2):
        b = axis*N_AXIS
        var[b + X1] = var[b + X2] = 0.25
        var[b + U] = model.release_sd**2
        var[b + C] = model.mean_sd**2
        var[b + G] = model.glide_sd**2
        var[b + E] = model.error_sd**2
    var[OFFSET] = model.offset_sd**2
    cov = np.diag(var)
    if anemometer is not None:
        height, wind, sd = anemometer
        for axis in range(2):
            h = np.zeros((1, N_STATE))
            h[0, axis*N_AXIS + C] = float(model.shape(height))
            mean, cov, _, _ = _update(mean, cov, np.array([wind[axis]]), h, np.array([[sd**2]]))
    return mean, cov


def measurement_rows(model, z, velocity=True, airspeed_map=None, error_gain=(1.0, 1.0)):
    """Stacked measurement matrix for one epoch: velocity rows then airspeed rows."""
    rows = []
    if velocity:
        for axis in range(2):
            h = np.zeros(N_STATE)
            h[axis*N_AXIS + U] = 1.0
            h[axis*N_AXIS + E] = error_gain[axis]
            rows.append(h)
    if airspeed_map is not None:
        for axis in range(2):
            h = -model.wind_row(z, axis)
            h[axis*N_AXIS + U] = 1.0
            h[OFFSET] = airspeed_map[axis]
            rows.append(h)
    return np.array(rows)


class WindFilter:
    """The forward filter, one epoch at a time. This is the part that flies.

    advance(t, z) moves the state to a new epoch; update(...) then folds in
    whatever was measured there. mean and cov are the filtered state.
    """

    def __init__(self, model, anemometer=None):
        self.model = model
        self.mean, self.cov = initial_state(model, anemometer)
        self.t = None
        self.z = None

    def advance(self, t, z):
        """Propagate to time t and height z. Returns the transition used (None for the first epoch)."""
        phi = None
        if self.t is not None:
            phi, q = full_step(self.model, self.z, z, t - self.t)
            self.mean = phi @ self.mean
            cov = phi @ self.cov @ phi.T + q
            self.cov = (cov + cov.T)/2
        self.t, self.z = float(t), float(z)
        return phi

    def update(self, velocity=None, velocity_var=None, error_gain=(1.0, 1.0), airspeed=None, airspeed_map=None,
               airspeed_var=None):
        """Measurement update at the current epoch. Returns (innovation, its covariance) or None.

        velocity (2,) with variance (2,); error_gain (2,) scales the correlated
        error state in each velocity component; airspeed (2,) with its 2 x 2
        offset map and variance (2,).
        """
        parts, noise = [], []
        if velocity is not None:
            parts.append(np.asarray(velocity, dtype=float))
            noise.append(np.broadcast_to(np.asarray(velocity_var, dtype=float), (2,)))
        if airspeed is not None:
            parts.append(np.asarray(airspeed, dtype=float))
            noise.append(np.broadcast_to(np.asarray(airspeed_var, dtype=float), (2,)))
            if airspeed_map is None:
                airspeed_map = np.zeros((2, 2))
        if not parts:
            return None
        h = measurement_rows(self.model, self.z, velocity is not None, airspeed_map if airspeed is not None else None,
                             error_gain)
        self.mean, self.cov, nu, s = _update(self.mean, self.cov, np.concatenate(parts), h, np.diag(np.concatenate(noise)))
        return nu, s

    def wind(self):
        """Filtered wind (2,) at the current height and its 2 x 2 covariance."""
        rows = np.array([self.model.wind_row(self.z, axis) for axis in range(2)])
        return rows @ self.mean, rows @ self.cov @ rows.T


class Retrieval:
    """Result of retrieve(). Arrays are indexed by epoch; *_f are filtered, others smoothed."""

    def __init__(self, model, t, z, mean_f, cov_f, mean, cov, gains, loglik, n_obs, nis):
        self.model, self.t, self.z = model, t, z
        self.mean_f, self.cov_f, self.mean, self.cov = mean_f, cov_f, mean, cov
        self._gains = gains
        self.log_likelihood, self.n_obs, self.nis = loglik, n_obs, nis
        rows = np.array([[model.wind_row(zk, axis) for axis in range(2)] for zk in z])   # (K, 2, N)
        self._rows = rows
        self.wind = np.einsum('kan,kn->ka', rows, mean)
        self.wind_var = np.einsum('kan,knm,kam->ka', rows, cov, rows)
        self.wind_f = np.einsum('kan,kn->ka', rows, mean_f)
        self.velocity = mean[:, [U, N_AXIS + U]]
        self.glide = mean[:, [G, N_AXIS + G]]
        self.glide_var = np.stack([cov[:, G, G], cov[:, N_AXIS + G, N_AXIS + G]], axis=1)
        self.mean_wind = mean[:, [C, N_AXIS + C]]
        self.mean_wind_var = np.stack([cov[:, C, C], cov[:, N_AXIS + C, N_AXIS + C]], axis=1)
        self.offset = mean[:, OFFSET]
        self.offset_var = np.stack([cov[:, N_STATE - 2, N_STATE - 2], cov[:, N_STATE - 1, N_STATE - 1]], axis=1)

    def functional(self, weights):
        """Mean and variance of sum_k weights[k] @ state[k], keeping every cross-epoch covariance.

        weights is (K, N_STATE). Cov(x_i, x_j | all data) = G_i ... G_(j-1) P_j for i < j.
        """
        mean = float(np.einsum('kn,kn->', weights, self.mean))
        carry = np.zeros(N_STATE)
        var = 0.0
        for k in range(len(self.t)):
            a = weights[k]
            pa = self.cov[k] @ a
            var += a @ pa + 2*carry @ pa
            if k < len(self._gains):
                carry = self._gains[k].T @ (a + carry)
        return mean, float(var)

    def layer_means(self, edges):
        """Layer-mean wind between consecutive heights in edges.

        Returns centres (L,), mean (L, 2), sd (L, 2), samples (L,). The mean is
        over the epochs inside the layer; a layer with none is NaN.
        """
        edges = np.sort(np.asarray(edges, dtype=float))
        layers = len(edges) - 1
        mean = np.full((layers, 2), np.nan)
        sd = np.full((layers, 2), np.nan)
        count = np.zeros(layers, dtype=int)
        for i in range(layers):
            top = self.z <= edges[i + 1] if i == layers - 1 else self.z < edges[i + 1]
            inside = np.flatnonzero((self.z >= edges[i]) & top)
            count[i] = len(inside)
            if not len(inside):
                continue
            for axis in range(2):
                weights = np.zeros((len(self.t), N_STATE))
                weights[inside] = self._rows[inside, axis]/len(inside)
                m, v = self.functional(weights)
                mean[i, axis], sd[i, axis] = m, math.sqrt(max(v, 0.0))
        return 0.5*(edges[:-1] + edges[1:]), mean, sd, count


def retrieve(model, t, z, velocity=None, velocity_var=None, airspeed=None, airspeed_map=None,
             airspeed_var=None, anemometer=None, error_gain=None):
    """Filter and smooth one logged descent (post-flight).

    t, z            (K,) epoch times (s, increasing) and heights above ground (m)
    velocity        (K, 2) horizontal velocity of the can, NaN rows where there is none
    velocity_var    scalar, (K,) or (K, 2) white-noise variance of each sample
    error_gain      scalar, (K,) or (K, 2) size of the correlated velocity error at each epoch,
                    in units of model.error_sd (so a filter's own sigma with error_sd = 1)
    airspeed        (K, 2) horizontal air-relative velocity from lateral_airspeed, NaN where none
    airspeed_map    (K, 2, 2) how the body-fixed offset enters each airspeed sample
    airspeed_var    scalar, (K,) or (K, 2) variance of each airspeed sample
    anemometer      (height, (east, north), sd) mean wind measured on the ground

    Returns a Retrieval.
    """
    t = np.asarray(t, dtype=float)
    z = np.asarray(z, dtype=float)
    epochs = len(t)
    if z.shape != t.shape or epochs < 2 or np.any(np.diff(t) <= 0):
        raise ValueError('need matching, increasing epoch times and heights')
    if velocity is None and airspeed is None:
        raise ValueError('nothing to retrieve from')

    def per_axis(value, name):
        value = np.asarray(value, dtype=float)
        if value.ndim == 0:
            value = np.full(epochs, float(value))
        if value.ndim == 1:
            value = np.repeat(value[:, None], 2, axis=1)
        if value.shape != (epochs, 2):
            raise ValueError(name + ' has the wrong shape')
        return value

    if velocity is not None:
        velocity = np.asarray(velocity, dtype=float)
        velocity_var = per_axis(velocity_var, 'velocity_var')
    error_gain = per_axis(1.0 if error_gain is None else error_gain, 'error_gain')
    if airspeed is not None:
        airspeed = np.asarray(airspeed, dtype=float)
        airspeed_var = per_axis(airspeed_var, 'airspeed_var')
        if airspeed_map is None:
            airspeed_map = np.zeros((epochs, 2, 2))
        airspeed_map = np.asarray(airspeed_map, dtype=float)

    f = WindFilter(model, anemometer)
    mean_p = np.empty((epochs, N_STATE))
    cov_p = np.empty((epochs, N_STATE, N_STATE))
    mean_f = np.empty_like(mean_p)
    cov_f = np.empty_like(cov_p)
    phis = np.empty((epochs - 1, N_STATE, N_STATE))
    loglik, n_obs, nis = 0.0, 0, np.full(epochs, np.nan)
    for k in range(epochs):
        phi = f.advance(t[k], z[k])
        if k:
            phis[k - 1] = phi
        mean_p[k], cov_p[k] = f.mean, f.cov
        has_v = velocity is not None and bool(np.all(np.isfinite(velocity[k])))
        has_q = airspeed is not None and bool(np.all(np.isfinite(airspeed[k])))
        out = f.update(velocity[k] if has_v else None, velocity_var[k] if has_v else None, error_gain[k],
                       airspeed[k] if has_q else None, airspeed_map[k] if has_q else None,
                       airspeed_var[k] if has_q else None)
        if out is not None:
            nu, s = out
            sol = np.linalg.solve(s, nu)
            nis[k] = float(nu @ sol)/len(nu)
            loglik += -0.5*(float(nu @ sol) + float(np.linalg.slogdet(2*math.pi*s)[1]))
            n_obs += len(nu)
        mean_f[k], cov_f[k] = f.mean, f.cov

    # A state switched off by a zero prior width has zero variance throughout,
    # so the smoother gain is solved on the remaining states only.
    on = np.flatnonzero(np.diag(cov_p[-1]) > 0)
    grid = np.ix_(on, on)
    mean_s, cov_s = mean_f.copy(), cov_f.copy()
    gains = np.zeros_like(phis)
    for k in range(epochs - 2, -1, -1):
        gains[k][:, on] = np.linalg.solve(cov_p[k + 1][grid], (phis[k] @ cov_f[k])[on]).T
        mean_s[k] = mean_f[k] + gains[k] @ (mean_s[k + 1] - mean_p[k + 1])
        c = cov_f[k] + gains[k] @ (cov_s[k + 1] - cov_p[k + 1]) @ gains[k].T
        cov_s[k] = (c + c.T)/2
    return Retrieval(model, t, z, mean_f, cov_f, mean_s, cov_s, gains, loglik, n_obs, nis)


def predict_drift(model, mean, cov, height, descent_speed, step=2.0):
    """Horizontal drift from now to the ground, given the filtered state at `height`.

    The state is propagated to z = 0 with no further measurements at a constant
    descent speed (Appendix J.2). Returns the drift (2,), its covariance (2, 2)
    and the time to touchdown. This is the direct, slow form used as the
    reference; flight code uses the table in cansat/landing.py.
    """
    if height <= 0:
        return np.zeros(2), np.zeros((2, 2)), 0.0
    n = N_AXIS + 1
    edges = np.linspace(height, 0.0, max(2, int(math.ceil(height/step)) + 1))
    phi, q = np.eye(n), np.zeros((n, n))
    for z_from, z_to in zip(edges[:-1], edges[1:]):
        f, step_q = axis_step(model, z_from, z_to, (z_from - z_to)/descent_speed, drift=True)
        phi = f @ phi
        q = f @ q @ f.T + step_q
    return (*drift_from_row(phi[N_AXIS, :N_AXIS], q[N_AXIS, N_AXIS], mean, cov), height/descent_speed)


def drift_from_row(row, noise, mean, cov):
    """Drift (2,) and covariance (2, 2) from one axis's drift row and its process-noise variance."""
    drift = np.empty(2)
    drift_cov = np.empty((2, 2))
    for a in range(2):
        sa = slice(a*N_AXIS, (a + 1)*N_AXIS)
        drift[a] = row @ mean[sa]
        for b in range(2):
            sb = slice(b*N_AXIS, (b + 1)*N_AXIS)
            drift_cov[a, b] = row @ cov[sa, sb] @ row + (noise if a == b else 0.0)
    return drift, drift_cov


def lateral_airspeed(accel, gyro, r, v_up, dt, rate_constant, reference_speed, lever=(0.0, 0.0, 0.0),
                     window=200, every=100, speed_window=40):
    """Horizontal air-relative velocity of the can from its lateral specific force (post-flight).

    accel, gyro     (K, 3) body-frame specific force (m/s^2) and rate (rad/s) at the IMU
    r               (K, 3, 3) body-to-world attitude from the navigation filter
    v_up            (K,) vertical velocity, positive up
    rate_constant   side force per unit mass per unit sideways airspeed (1/s) at reference_speed
                    through the air. It has to be measured for the real can: it depends on
                    where the side force acts as well as on its size
    lever           IMU position from the centre of mass, body axes (m)
    window, every   samples averaged into each output (Hann weights), and their spacing
    speed_window    samples averaged for the total airspeed that scales the side force

    Averaged over the can's wobble the body-lateral specific force a_xy is the
    can's side force over its mass, -kappa*(R^T v_air)_xy, with kappa
    proportional to airspeed. Solving for the horizontal part of v_air needs the
    attitude only to carry a small lateral vector and the tilt of the fall
    line, so a tilt error d costs about descent_speed*d and no more. The
    window must cover several periods of the slowest swing: in simulation 1 s
    left 0.15-1.7 m/s of error depending on the canopy and 2 s left 0.07-0.33.

    Returns (centre indices (M,), airspeed (M, 2), offset_map (M, 2, 2)). A
    constant body-frame accelerometer offset b (m/s^2) adds about
    offset_map @ (b/rate_constant) to the airspeed, which is how the retrieval
    separates it when the can spins.
    """
    accel = np.asarray(accel, dtype=float)
    gyro = np.asarray(gyro, dtype=float)
    lever = np.asarray(lever, dtype=float)
    half = window//2
    weights = np.hanning(2*half + 3)[1:-1]             # 2*half + 1 samples, symmetric about the centre
    weights = weights/weights.sum()
    taps = np.hanning(2*(speed_window//2) + 3)[1:-1]
    taps = taps/taps.sum()
    edge = np.convolve(np.ones(len(accel)), taps, mode='same')
    alpha = np.gradient(gyro, dt, axis=0)
    a_xy = (accel - np.cross(alpha, lever) - np.cross(gyro, np.cross(gyro, lever)))[:, :2]
    inverse = np.linalg.inv(np.swapaxes(r[:, :2, :2], 1, 2))
    fall = r[:, 2, :2]*v_up[:, None]
    speed = np.abs(v_up)
    for _ in range(8):
        # The side-force rate grows with total airspeed. That speed is taken from a short
        # average of the air velocity: long enough to remove the can's own rocking, which
        # would inflate it and bias the scale, short enough to follow the swing.
        scale = (reference_speed/np.maximum(speed, 1.0))/rate_constant
        air = np.einsum('kij,kj->ki', inverse, -a_xy*scale[:, None] - fall)
        slow = np.column_stack([np.convolve(air[:, a], taps, mode='same')/edge for a in range(2)])
        speed = np.sqrt(v_up**2 + (slow*slow).sum(axis=1))
    maps = inverse*(reference_speed/np.maximum(speed, 1.0))[:, None, None]
    centres = np.arange(half, len(accel) - half, every)
    idx = centres[:, None] + np.arange(-half, half + 1)[None, :]
    return centres, np.einsum('w,mwi->mi', weights, air[idx]), -np.einsum('w,mwij->mij', weights, maps[idx])
