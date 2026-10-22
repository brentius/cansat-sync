"""GPL-3.0-only. Synthetic parachute descent for exercising the navigation filter.

A batch of independent descents is integrated together. Each has three parts:

  canopy   a point with apparent mass and axis-referenced aerodynamics; its
           axis is the line, so it has an angle of attack and a normal force
  line     massless and inextensible, from the canopy to the top of the can
  can      a rigid body free to swing and spin about its attachment point

The line tension is whatever keeps the line length constant (solved at
acceleration level each step, with Baumgarte drift correction). The IMU output
follows from rigid-body kinematics at the IMU's position, so the measured
specific force and rate are exactly consistent with the simulated trajectory.

Frames match code/cansat/eskf.py: world east/north/up, body z out of the
bottom of the can, R maps body to world.

Every parameter value is an assumption awaiting measurement, not a property of
the real CanSat. In particular the canopy normal-force law is a generic shape
used to produce a family of swing behaviours; it is not a model of our canopy.
"""
import math
from dataclasses import dataclass

import numpy as np

G0 = 9.80665
RHO = 1.225
SQRT3 = math.sqrt(3.0)


def cross(a, b):
    out = np.empty(np.broadcast_shapes(a.shape, b.shape))
    out[..., 0] = a[..., 1]*b[..., 2] - a[..., 2]*b[..., 1]
    out[..., 1] = a[..., 2]*b[..., 0] - a[..., 0]*b[..., 2]
    out[..., 2] = a[..., 0]*b[..., 1] - a[..., 1]*b[..., 0]
    return out


def norm(a):
    return np.sqrt((a*a).sum(axis=-1))


def quat_matrix(q):
    """Unit quaternions (n, 4) as [w, x, y, z] to body-to-world matrices (n, 3, 3)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    r = np.empty((len(q), 3, 3))
    r[:, 0, 0] = 1 - 2*(y*y + z*z)
    r[:, 0, 1] = 2*(x*y - w*z)
    r[:, 0, 2] = 2*(x*z + w*y)
    r[:, 1, 0] = 2*(x*y + w*z)
    r[:, 1, 1] = 1 - 2*(x*x + z*z)
    r[:, 1, 2] = 2*(y*z - w*x)
    r[:, 2, 0] = 2*(x*z - w*y)
    r[:, 2, 1] = 2*(y*z + w*x)
    r[:, 2, 2] = 1 - 2*(x*x + y*y)
    return r


def quat_multiply(a, b):
    return np.concatenate((
        (a[:, :1]*b[:, :1] - (a[:, 1:]*b[:, 1:]).sum(1, keepdims=True)),
        a[:, :1]*b[:, 1:] + b[:, :1]*a[:, 1:] + cross(a[:, 1:], b[:, 1:])), axis=1)


def quat_from_vector(v):
    angle = norm(v)
    safe = np.where(angle > 1e-12, angle, 1.0)
    return np.concatenate((np.cos(angle/2)[:, None],
                           np.where(angle[:, None] > 1e-12, np.sin(angle/2)[:, None]*v/safe[:, None], v/2)), axis=1)


def rotation_vectors(r):
    """Rotation vectors (n, 3) of rotation matrices (n, 3, 3), away from 180 degrees."""
    cos = np.clip((r[:, 0, 0] + r[:, 1, 1] + r[:, 2, 2] - 1)/2, -1, 1)
    angle = np.arccos(cos)
    v = np.stack((r[:, 2, 1] - r[:, 1, 2], r[:, 0, 2] - r[:, 2, 0], r[:, 1, 0] - r[:, 0, 1]), axis=1)/2
    sin = np.sin(angle)
    scale = np.where(angle > 1e-7, angle/np.where(sin > 1e-12, sin, 1.0), 1.0)
    return v*scale[:, None]


def rotation_matrices(v):
    return quat_matrix(quat_from_vector(v))


@dataclass
class Params:
    # Masses. The 2025-26 rules allow 300-350 g; the BOM targets about 330 g.
    payload_mass: float = 0.310          # kg, can without canopy
    canopy_mass: float = 0.015           # kg, fabric and lines
    # Apparent mass of air moving with the canopy. A 0.28 m canopy encloses
    # about 7 g of air in a hemisphere; published coefficients are of order 1.
    canopy_added_mass: float = 0.008     # kg
    inertia: tuple = (4.3e-4, 4.3e-4, 1.7e-4)   # kg m^2, uniform 66 x 115 mm cylinder
    attach: tuple = (0.0, 0.0, -0.055)   # m, line attachment from the centre of mass, body axes
    imu: tuple = (0.005, 0.0, -0.010)    # m, IMU from the centre of mass, body axes
    line: float = 0.45                   # m, attachment to canopy centre of pressure
    descent_speed: float = 10.5          # m/s at RHO, sets the canopy drag area
    payload_cds: float = 0.003           # m^2, end-on can
    # Canopy normal force, as a multiple of dynamic pressure times drag area:
    #   trim_deg = 0:  cn_k*sin(alpha)                          restoring
    #   trim_deg > 0:  cn_k*sin(alpha)*((alpha/alpha0)^2 - 1)   unstable below alpha0
    # cn_k = 1 with trim_deg = 0 is exactly a point with isotropic drag.
    cn_k: float = 1.0
    trim_deg: float = 0.0
    rot_damping: tuple = (2.0e-3, 2.0e-3, 2.0e-5)   # N m s, swing damping and swivel friction
    spin_torque_sd: float = 4.0e-5       # N m, random torque about the can axis
    spin_torque_tau: float = 1.0         # s
    side_force_sd: float = 0.0           # N, random horizontal force on the canopy
    side_force_tau: float = 0.5          # s
    baumgarte: float = 30.0              # 1/s
    dt: float = 0.001                    # s, integration step


@dataclass
class WindParams:
    """Log-law mean wind with drop-Dryden turbulence (proposal Appendix I)."""
    speed_10m: float = 5.0       # m/s
    roughness: float = 0.1       # m
    sigma_h: float = 1.0         # m/s, each horizontal component
    sigma_w: float = 0.5         # m/s, vertical
    eddy_slope: float = 0.5      # L = eddy_slope * max(z, eddy_floor)
    eddy_floor: float = 1.0      # m
    top: float = 420.0           # m
    dz: float = 0.5              # m


class Wind:
    """Frozen wind profiles (n, levels, 3) on a uniform height grid."""

    def __init__(self, profile, dz):
        self.profile = profile
        self.dz = dz
        self._rows = np.arange(len(profile))
        self._last = profile.shape[1] - 2

    def __call__(self, z):
        x = np.clip(z/self.dz, 0.0, self._last + 0.999999)
        i = x.astype(int)
        f = (x - i)[:, None]
        return (1 - f)*self.profile[self._rows, i] + f*self.profile[self._rows, i + 1]


def make_wind(n, rng, wp=None, direction=None):
    wp = wp or WindParams()
    z = np.arange(0.0, wp.top + wp.dz, wp.dz)
    zc = np.maximum(z, wp.eddy_floor)
    # Eddy-scaled coordinate zeta = integral dz / L(z), piecewise exact.
    zeta = np.where(z <= wp.eddy_floor, z/(wp.eddy_slope*wp.eddy_floor),
                    1/wp.eddy_slope + np.log(zc/wp.eddy_floor)/wp.eddy_slope)
    steps = np.diff(zeta)
    levels = len(z)
    turb = np.empty((n, levels, 3))
    # Horizontal components: two-state realisation of (1 - r/2) exp(-r).
    for axis in range(2):
        x = rng.normal(size=(n, 2))/2                      # stationary covariance I/4
        turb[:, 0, axis] = x[:, 0] + SQRT3*x[:, 1]
        for k, r in enumerate(steps):
            e = math.exp(-r)
            phi = e*np.array([[1 + r, r], [-r, 1 - r]])
            q = np.eye(2)/4 - phi @ phi.T/4
            q = (q + q.T)/2
            vals, vecs = np.linalg.eigh(q)
            root = vecs*np.sqrt(np.maximum(vals, 0.0))
            x = x @ phi.T + rng.normal(size=(n, 2)) @ root.T
            turb[:, k + 1, axis] = x[:, 0] + SQRT3*x[:, 1]
    # Vertical component: longitudinal, exp(-r).
    x = rng.normal(size=n)
    turb[:, 0, 2] = x
    for k, r in enumerate(steps):
        e = math.exp(-r)
        x = e*x + math.sqrt(1 - e*e)*rng.normal(size=n)
        turb[:, k + 1, 2] = x
    turb[:, :, :2] *= wp.sigma_h
    turb[:, :, 2] *= wp.sigma_w
    speed = wp.speed_10m*np.log(np.maximum(z, 1.0)/wp.roughness)/math.log(10.0/wp.roughness)
    speed = np.maximum(speed, 0.0)
    heading = rng.uniform(0, 2*math.pi, n) if direction is None else np.full(n, float(direction))
    mean = np.zeros((n, levels, 3))
    mean[:, :, 0] = np.cos(heading)[:, None]*speed
    mean[:, :, 1] = np.sin(heading)[:, None]*speed
    return Wind(mean + turb, wp.dz), mean, turb


class Descent:
    """Batch integrator. State arrays have one row per descent."""

    def __init__(self, params, wind, n):
        self.p = params
        self.wind = wind
        self.n = n
        p = params
        self.inertia = np.array(p.inertia)
        self.attach = np.array(p.attach)
        self.imu = np.array(p.imu)
        self.damping = np.array(p.rot_damping)
        self.canopy_inertial = p.canopy_mass + p.canopy_added_mass
        weight = (p.payload_mass + p.canopy_mass)*G0
        self.canopy_cds = weight/(0.5*RHO*p.descent_speed**2) - p.payload_cds
        self.trim = math.radians(p.trim_deg)
        self.side = np.zeros((n, 3))
        self.spin = np.zeros(n)
        self.t = 0.0

    # --- Set-up --------------------------------------------------------------

    def start(self, rng, height=300.0, tilt_sd_deg=10.0, rate_sd_dps=20.0, spin_sd_dps=60.0,
              lateral_sd=1.5, position_sd=30.0):
        """Canopy open and settled at roughly terminal speed, with residual swing."""
        n, p = self.n, self.p
        tilt = np.zeros((n, 3))
        tilt[:, :2] = rng.normal(0, math.radians(tilt_sd_deg), (n, 2))
        yaw = np.zeros((n, 3))
        yaw[:, 2] = rng.uniform(-math.pi, math.pi, n)
        level = np.tile(np.array([0.0, 1.0, 0.0, 0.0]), (n, 1))      # R = diag(1, -1, -1)
        self.q = quat_multiply(quat_multiply(level, quat_from_vector(yaw)), quat_from_vector(tilt))
        r = quat_matrix(self.q)
        self.pp = np.column_stack((rng.normal(0, position_sd, (n, 2)), np.full(n, float(height))))
        up = np.array([0.0, 0.0, -1.0])                               # body axis towards the canopy
        arm = self.attach + p.line*up
        self.pc = self.pp + r @ arm
        self.om = np.column_stack((rng.normal(0, math.radians(rate_sd_dps), (n, 2)),
                                   rng.normal(0, math.radians(spin_sd_dps), n)))
        self.vp = self.wind(self.pp[:, 2]) + np.column_stack((rng.normal(0, lateral_sd, (n, 2)),
                                                              np.full(n, -p.descent_speed)))
        # Canopy velocity of a rigid rotation about the centre of mass, so the
        # line starts neither stretching nor going slack.
        self.vc = self.vp + np.einsum('nij,nj->ni', r, cross(self.om, arm))
        if p.spin_torque_sd:
            self.spin = rng.normal(0, p.spin_torque_sd, n)
        if p.side_force_sd:
            self.side[:, :2] = rng.normal(0, p.side_force_sd, (n, 2))
        self.t = 0.0

    # --- Dynamics ------------------------------------------------------------

    def normal_coeff(self, alpha):
        s = np.sin(alpha)
        if self.trim <= 0:
            return self.p.cn_k*s
        return self.p.cn_k*s*((alpha/self.trim)**2 - 1)

    def derivative(self, pc, vc, pp, vp, q, om):
        """State derivative plus (rotation, payload acceleration, angular acceleration, tension)."""
        p = self.p
        r = quat_matrix(q)
        rho_w = r @ self.attach
        d = pc - pp - rho_w
        n = d/norm(d)[:, None]

        rel = vc - self.wind(pc[:, 2])
        v_ax = (rel*n).sum(1)
        perp = rel - v_ax[:, None]*n
        v_perp = norm(perp)
        speed = norm(rel)
        qs = 0.5*RHO*speed*speed*self.canopy_cds
        alpha = np.arctan2(v_perp, -v_ax)
        perp_hat = perp/np.where(v_perp > 1e-9, v_perp, 1.0)[:, None]
        f_c = (qs*np.cos(alpha))[:, None]*n - (qs*self.normal_coeff(alpha))[:, None]*perp_hat + self.side
        f_c[:, 2] -= p.canopy_mass*G0

        rel_p = vp - self.wind(pp[:, 2])
        f_p = -(0.5*RHO*p.payload_cds*norm(rel_p))[:, None]*rel_p
        f_p[:, 2] -= p.payload_mass*G0

        tau0 = -self.damping*om - cross(om, self.inertia*om)
        tau0[:, 2] += self.spin
        n_b = np.einsum('nji,nj->ni', r, n)
        lever_torque = cross(self.attach, n_b)     # per unit tension
        ww = cross(om, cross(om, self.attach))
        dd = vc - vp - np.einsum('nij,nj->ni', r, cross(om, self.attach))
        bias = (dd*dd).sum(1) + 2*p.baumgarte*(d*dd).sum(1) + 0.5*p.baumgarte**2*((d*d).sum(1) - p.line**2)

        def accelerations(tension):
            a_c = (f_c - tension[:, None]*n)/self.canopy_inertial
            a_p = (f_p + tension[:, None]*n)/p.payload_mass
            ang = (tau0 + tension[:, None]*lever_torque)/self.inertia
            return a_c, a_p, ang

        def constraint(tension):
            a_c, a_p, ang = accelerations(tension)
            ddd = a_c - a_p - np.einsum('nij,nj->ni', r, cross(ang, self.attach) + ww)
            return (d*ddd).sum(1) + bias

        # The constraint is linear in tension: two evaluations solve it.
        g0 = constraint(np.zeros(self.n))
        g1 = constraint(np.ones(self.n))
        tension = np.maximum(-g0/(g1 - g0), 0.0)                      # a line cannot push
        a_c, a_p, ang = accelerations(tension)
        dq = 0.5*np.concatenate((-(q[:, 1:]*om).sum(1, keepdims=True),
                                 q[:, :1]*om + cross(q[:, 1:], om)), axis=1)
        return (vc, a_c, vp, a_p, dq, ang), (r, a_p, ang, tension)

    def step(self, rng=None):
        """One RK4 step. Returns the start-of-step IMU specific force and rate."""
        h = self.p.dt
        y = (self.pc, self.vc, self.pp, self.vp, self.q, self.om)
        k1, (r, a_p, ang, tension) = self.derivative(*y)
        imu = self.imu
        a_imu = a_p + np.einsum('nij,nj->ni', r, cross(ang, imu) + cross(self.om, cross(self.om, imu)))
        a_imu[:, 2] += G0
        force = np.einsum('nji,nj->ni', r, a_imu)
        rate = self.om
        k2, _ = self.derivative(*(a + h/2*b for a, b in zip(y, k1)))
        k3, _ = self.derivative(*(a + h/2*b for a, b in zip(y, k2)))
        k4, _ = self.derivative(*(a + h*b for a, b in zip(y, k3)))
        self.pc, self.vc, self.pp, self.vp, self.q, self.om = (
            a + h/6*(b1 + 2*b2 + 2*b3 + b4) for a, b1, b2, b3, b4 in zip(y, k1, k2, k3, k4))
        self.q = self.q/norm(self.q)[:, None]
        self.tension = tension
        if rng is not None:
            p = self.p
            if p.spin_torque_sd:
                e = math.exp(-h/p.spin_torque_tau)
                self.spin = e*self.spin + p.spin_torque_sd*math.sqrt(1 - e*e)*rng.normal(size=self.n)
            if p.side_force_sd:
                e = math.exp(-h/p.side_force_tau)
                self.side[:, :2] = e*self.side[:, :2] + p.side_force_sd*math.sqrt(1 - e*e)*rng.normal(size=(self.n, 2))
        self.t += h
        return force, rate

    # --- Observables ---------------------------------------------------------

    def imu_state(self):
        """Position, velocity and attitude of the IMU point."""
        r = quat_matrix(self.q)
        imu = self.imu
        return self.pp + r @ self.imu, self.vp + np.einsum('nij,nj->ni', r, cross(self.om, imu)), r

    def swing_angle(self):
        """Angle of the can axis from vertical, radians."""
        return np.arccos(np.clip(-quat_matrix(self.q)[:, 2, 2], -1, 1))

    def line_angle(self):
        d = self.pc - self.pp - quat_matrix(self.q) @ self.attach
        return np.arccos(np.clip(d[:, 2]/norm(d), -1, 1))

    def line_length(self):
        return norm(self.pc - self.pp - quat_matrix(self.q) @ self.attach)


def simulate(params, n, rng, duration=28.0, nav_dt=0.01, frame_dt=0.2, camera_delay=0.0,
             wind_params=None, start=None, keep_fine=False, delays=()):
    """Run a batch and return truth and ideal sensor inputs at the navigation rate.

    Returned arrays (K nav steps, F frames, n descents):
      t (K+1,), p, v (K+1, n, 3), r (K+1, n, 3, 3)       IMU-point truth
      accel, gyro (K, n, 3)   exact increments / nav_dt (coning and sculling free)
      accel_mean, gyro_mean   boxcar means of the 1 kHz samples
      accel_inst, gyro_inst   the single sample at the start of each nav step
      frame_t (F,), frame_p (F, n, 3), frame_r (F, n, 3, 3)  pose at frame time + camera_delay
      delay_p (D, F, n, 3), delay_r (D, F, n, 3, 3)  the same for each extra delay in `delays`
      swing, line_angle (K+1, n), tension (K, n), wind_mean, wind_turb
    """
    wind, wind_mean, wind_turb = make_wind(n, rng, wind_params)
    sim = Descent(params, wind, n)
    sim.start(rng, **(start or {}))
    sub = round(nav_dt/params.dt)
    if abs(sub*params.dt - nav_dt) > 1e-12:
        raise ValueError('navigation step must be a multiple of the integration step')
    steps = round(duration/nav_dt)
    frame_every = round(frame_dt/params.dt)
    delay_steps = round(camera_delay/params.dt)
    extra_steps = [round(d/params.dt) for d in delays]
    frames = int((steps*sub - max([delay_steps] + extra_steps))//frame_every) + 1
    out = {k: np.empty((steps + 1, n, 3)) for k in ('p', 'v')}
    out['r'] = np.empty((steps + 1, n, 3, 3))
    for k in ('accel', 'gyro', 'accel_mean', 'gyro_mean', 'accel_inst', 'gyro_inst'):
        out[k] = np.empty((steps, n, 3))
    out['swing'] = np.empty((steps + 1, n))
    out['line_angle'] = np.empty((steps + 1, n))
    out['tension'] = np.empty((steps, n))
    out['air_velocity'] = np.empty((steps + 1, n, 3))
    out['frame_p'] = np.empty((frames, n, 3))
    out['frame_r'] = np.empty((frames, n, 3, 3))
    out['delay_p'] = np.empty((len(delays), frames, n, 3))
    out['delay_r'] = np.empty((len(delays), frames, n, 3, 3))
    fine = [] if keep_fine else None
    gravity_dt = np.array([0.0, 0.0, -G0])*nav_dt

    def record_frame(index):
        if index >= delay_steps and (index - delay_steps) % frame_every == 0:
            f = (index - delay_steps)//frame_every
            if f < frames:
                out['frame_p'][f], _, out['frame_r'][f] = sim.imu_state()
        for i, extra in enumerate(extra_steps):
            if index >= extra and (index - extra) % frame_every == 0 and (index - extra)//frame_every < frames:
                out['delay_p'][i, (index - extra)//frame_every], _, out['delay_r'][i, (index - extra)//frame_every] = sim.imu_state()

    p0, v0, r0 = sim.imu_state()
    out['p'][0], out['v'][0], out['r'][0] = p0, v0, r0
    out['swing'][0], out['line_angle'][0] = sim.swing_angle(), sim.line_angle()
    out['air_velocity'][0] = sim.vp - wind(sim.pp[:, 2])
    record_frame(0)
    for k in range(steps):
        force_sum = np.zeros((n, 3))
        rate_sum = np.zeros((n, 3))
        for j in range(sub):
            force, rate = sim.step(rng)
            if j == 0:
                out['accel_inst'][k], out['gyro_inst'][k] = force, rate
            force_sum += force
            rate_sum += rate
            record_frame(k*sub + j + 1)
            if keep_fine:
                fine.append((sim.t, sim.line_length().copy(), sim.tension.copy()))
        p1, v1, r1 = sim.imu_state()
        dtheta = rotation_vectors(np.einsum('nji,njk->nik', r0, r1))
        r_mid = r0 @ rotation_matrices(dtheta/2)
        out['gyro'][k] = dtheta/nav_dt
        out['accel'][k] = np.einsum('nji,nj->ni', r_mid, v1 - v0 - gravity_dt)/nav_dt
        out['accel_mean'][k] = force_sum/sub
        out['gyro_mean'][k] = rate_sum/sub
        out['tension'][k] = sim.tension
        out['p'][k + 1], out['v'][k + 1], out['r'][k + 1] = p1, v1, r1
        out['swing'][k + 1], out['line_angle'][k + 1] = sim.swing_angle(), sim.line_angle()
        out['air_velocity'][k + 1] = sim.vp - wind(sim.pp[:, 2])
        p0, v0, r0 = p1, v1, r1
    out['t'] = np.arange(steps + 1)*nav_dt
    out['frame_t'] = np.arange(frames)*frame_dt
    out['wind_mean'], out['wind_turb'], out['wind_dz'] = wind_mean, wind_turb, wind.dz
    if keep_fine:
        out['fine'] = fine
    return out
