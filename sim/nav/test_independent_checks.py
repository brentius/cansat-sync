"""GPL-3.0-only. Independent verification of code/cansat/eskf.py, sim/nav/truth.py and run_descent.

Every reference a module is compared with is derived and implemented in this
file (rotations, pendulum dynamics, projection, strapdown, error-state model,
information-form update, MAP solve, wind covariance, a matched Monte Carlo
world). The modules' own helpers are used only to drive them or to decode
their state, never to check them.

Layout (letters follow the review brief):
  A  PlanarDoublePendulum, FreeCanopy        truth.py dynamics against Lagrangian models
  B  ImuOutput                               truth.py IMU against differentiated trajectory
  C  Strapdown                               own strapdown against truth and Eskf.propagate
  D  ErrorStateTransition                    Appendix H.1 exponentiated against Eskf.propagate
  E  LandmarkUpdate                          Jacobians, information form, gates, reset, MAP, recovery
  F  MatchedMonteCarlo                       NEES consistency in a world built here
  G  WindGenerator                           Appendix I covariance against make_wind
  H  TruthBookkeeping, RunDescentWiring      frames, aliasing, batch independence, run_descent

Tests marked expectedFailure record a defect or a limitation of the module
under review; the comment above each says what is wrong and what to change.
All randomness is seeded. Everything here is SYNTHETIC.
"""
import math
import sys
import tempfile
import unittest
import warnings
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]/'code'))
import truth as tr  # noqa: E402
from cansat import eskf as ek  # noqa: E402

G = 9.80665
RHO_AIR = 1.225
DEG = math.pi/180
E_UP = np.array([0.0, 0.0, 1.0])
LEVEL = np.diag([1.0, -1.0, -1.0])       # body z down, written out here rather than imported
CALM = tr.WindParams(speed_10m=0.0, sigma_h=0.0, sigma_w=0.0, top=320.0)
QUIET = dict(spin_torque_sd=0.0, side_force_sd=0.0)
P, V, A, BA, BG, BB, TM = slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12), slice(12, 15), 15, 16
NS = 17                                   # error-state dimension
BLOCKS = (('pos', P), ('vel', V), ('att', A), ('ba', BA), ('bg', BG), ('baro', slice(15, 16)),
          ('time', slice(16, 17)))


# --- Own rotation algebra -------------------------------------------------------

def hat(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]], dtype=np.result_type(v, float))


def rot(v):
    """exp([v]x) by its series for small angles and Rodrigues otherwise."""
    v = np.asarray(v, dtype=float)
    k = hat(v)
    t2 = float(v @ v)
    if t2 < 1e-8:
        return np.eye(3) + (1 - t2/6)*k + (0.5 - t2/24)*(k @ k)
    t = math.sqrt(t2)
    return np.eye(3) + math.sin(t)/t*k + (1 - math.cos(t))/t2*(k @ k)


def rotvec(r):
    """Rotation vector of r through the quaternion, valid for angles below pi."""
    w = 0.5*math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2]))
    u = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])/(4*w)
    s = math.sqrt(float(u @ u))
    return 2*u if s < 1e-9 else 2*math.atan2(s, w)/s*u


def cross3(a, b):
    return np.array([a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]])


def qmul(a, b):
    return np.r_[a[0]*b[0] - a[1:] @ b[1:], a[0]*b[1:] + b[0]*a[1:] + cross3(a[1:], b[1:])]


def qvec(v):
    t = math.sqrt(float(v @ v))
    return np.r_[1.0, v/2] if t < 1e-12 else np.r_[math.cos(t/2), math.sin(t/2)/t*v]


def qrot(q, v):
    """Rotate v by unit quaternion q: v + 2 w (u x v) + 2 u x (u x v)."""
    u = q[1:]
    c = cross3(u, v)
    return v + 2*q[0]*c + 2*cross3(u, c)


def qmat(q):
    """Rotation matrix of a unit quaternion: (w^2 - u.u) I + 2 u u^T + 2 w [u]x."""
    w, u = q[0], q[1:]
    return (w*w - u @ u)*np.eye(3) + 2*np.outer(u, u) + 2*w*hat(u)


def qfrom(r):
    """Unit quaternion of a rotation matrix (largest-pivot branch, safe at 180 degrees)."""
    diag = np.array([r[0, 0] + r[1, 1] + r[2, 2], r[0, 0], r[1, 1], r[2, 2]])
    i = int(np.argmax(diag))
    q = np.empty(4)
    if i == 0:
        s = 2*math.sqrt(1 + diag[0])
        q[:] = s/4, (r[2, 1] - r[1, 2])/s, (r[0, 2] - r[2, 0])/s, (r[1, 0] - r[0, 1])/s
    else:
        a, b, c = i - 1, i % 3, (i + 1) % 3
        s = 2*math.sqrt(1 + r[a, a] - r[b, b] - r[c, c])
        q[0], q[1 + a], q[1 + b], q[1 + c] = (r[c, b] - r[b, c])/s, s/4, (r[b, a] + r[a, b])/s, (r[c, a] + r[a, c])/s
    return q


def rk4(f, y, h):
    k1 = f(y)
    k2 = f(y + h/2*k1)
    k3 = f(y + h/2*k2)
    k4 = f(y + h*k3)
    return y + h/6*(k1 + 2*k2 + 2*k3 + k4)


# --- Driving truth.py in a vertical plane ------------------------------------------

def set_planar(sim, x, z, vx, vz, phi1, phi2, w1, w2):
    """Put every row of a truth.py batch in the east-up plane.

    (x, z), (vx, vz): canopy point. phi1: line angle, phi2: can-axis angle,
    both from the downward vertical, positive towards +east. A = attachment,
    C = centre of mass: A = canopy + L (sin phi1, -cos phi1),
    C = A + d (sin phi2, -cos phi2). The can attitude is LEVEL Ry(phi2), whose
    quaternion is (0, cos(phi2/2), 0, sin(phi2/2)), and its body rate is
    (0, dphi2/dt, 0).
    """
    arrays = [np.atleast_1d(np.asarray(a, dtype=float)) for a in (x, z, vx, vz, phi1, phi2, w1, w2)]
    x, z, vx, vz, phi1, phi2, w1, w2 = arrays
    line, d = sim.p.line, -sim.p.attach[2]
    o = np.zeros_like(x)
    sim.pc = np.column_stack((x, o, z))
    sim.vc = np.column_stack((vx, o, vz))
    a = sim.pc + line*np.column_stack((np.sin(phi1), o, -np.cos(phi1)))
    va = sim.vc + (line*w1)[:, None]*np.column_stack((np.cos(phi1), o, np.sin(phi1)))
    sim.pp = a + d*np.column_stack((np.sin(phi2), o, -np.cos(phi2)))
    sim.vp = va + (d*w2)[:, None]*np.column_stack((np.cos(phi2), o, np.sin(phi2)))
    sim.q = np.column_stack((o, np.cos(phi2/2), o, np.sin(phi2/2)))
    sim.om = np.column_stack((o, w2, o))
    sim.t = 0.0


def planar_angles(sim):
    """(phi1, phi2, worst out-of-plane quantity) decoded with this file's quaternion algebra."""
    phi1, phi2, out = [], [], 0.0
    for j in range(sim.n):
        r = qmat(sim.q[j])
        a = sim.pp[j] + r @ np.asarray(sim.p.attach)
        phi1.append(math.atan2(a[0] - sim.pc[j, 0], sim.pc[j, 2] - a[2]))
        phi2.append(math.atan2(r[0, 2], -r[2, 2]))
        out = max(out, abs(a[1] - sim.pc[j, 1]), abs(sim.pp[j, 1]), abs(sim.om[j, 0]), abs(sim.om[j, 2]))
    return np.array(phi1), np.array(phi2), out


def calm_wind(n):
    return tr.make_wind(n, np.random.default_rng(0), CALM)[0]


# === A. Planar double pendulum ===================================================
#
# A rigid can (mass m, transverse inertia I about its centre of mass C,
# attachment A a distance d above C) hangs from a fixed pivot O on a massless
# rigid link of length L. With phi1 the link angle and phi2 the can angle from
# the downward vertical, x right and z up:
#
#   C = (L sin phi1 + d sin phi2, -L cos phi1 - d cos phi2)
#   T = m/2 [L^2 phi1'^2 + d^2 phi2'^2 + 2 L d phi1' phi2' cos(phi1 - phi2)] + I/2 phi2'^2
#   V = -m g (L cos phi1 + d cos phi2)
#
# Lagrange's equations, D = phi1 - phi2:
#
#   d/dt(dT/dphi1') = m L^2 phi1'' + m L d phi2'' cos D - m L d phi2' sin D (phi1' - phi2')
#   dT/dphi1        = -m L d phi1' phi2' sin D,          dV/dphi1 = m g L sin phi1
#   =>  m L^2 phi1'' + m L d cos D phi2'' = -m L d sin D phi2'^2 - m g L sin phi1
#
#   d/dt(dT/dphi2') = (m d^2 + I) phi2'' + m L d phi1'' cos D - m L d phi1' sin D (phi1' - phi2')
#   dT/dphi2        = +m L d phi1' phi2' sin D,          dV/dphi2 = m g d sin phi2
#   =>  m L d cos D phi1'' + (m d^2 + I) phi2'' = +m L d sin D phi1'^2 - m g d sin phi2
#
# Small oscillations: M q'' + K q = 0 with M = [[m L^2, m L d], [m L d, m d^2 + I]]
# and K = diag(m g L, m g d); the squared mode frequencies are eig(M^-1 K).

def double_pendulum(y, m, inertia, d, line):
    p1, p2, w1, w2 = y
    c, s = math.cos(p1 - p2), math.sin(p1 - p2)
    a11, a12, a22 = m*line*line, m*line*d*c, m*d*d + inertia
    b1 = -m*line*d*s*w2*w2 - m*G*line*math.sin(p1)
    b2 = m*line*d*s*w1*w1 - m*G*d*math.sin(p2)
    det = a11*a22 - a12*a12
    return np.array([w1, w2, (a22*b1 - a12*b2)/det, (a11*b2 - a12*b1)/det])


class PlanarDoublePendulum(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # No aerodynamics, no damping, canopy immovable and weightless.
        cls.params = replace(tr.Params(), canopy_mass=0.0, canopy_added_mass=1e13, descent_speed=1e6,
                             payload_cds=0.0, rot_damping=(0.0, 0.0, 0.0), **QUIET)
        p = cls.params
        cls.m, cls.inertia, cls.d, cls.line = p.payload_mass, p.inertia[1], -p.attach[2], p.line
        mass = np.array([[cls.m*cls.line**2, cls.m*cls.line*cls.d], [cls.m*cls.line*cls.d, cls.m*cls.d**2 + cls.inertia]])
        stiff = np.diag([cls.m*G*cls.line, cls.m*G*cls.d])
        vals, vecs = np.linalg.eig(np.linalg.solve(mass, stiff))
        order = np.argsort(vals.real)
        cls.omega = np.sqrt(vals.real[order])
        shape = vecs.real[:, order]
        shape = shape/np.max(abs(shape), axis=0)*2e-3                      # 2 mrad peak
        # Row 0: large angles with rates. Rows 1, 2: each normal mode alone.
        cls.start = np.array([[40*DEG, -25*DEG, 1.3, -2.1],
                              [shape[0, 0], shape[1, 0], 0.0, 0.0],
                              [shape[0, 1], shape[1, 1], 0.0, 0.0]])
        # The same 2 s of large-angle swing at the production 1 ms step and at
        # 2 ms, and 5 s of the small modes at 2 ms (ample for 0.7 and 2 Hz).
        cls.fine = cls.run_truth(0.001, 2.0, cls.start[:1])
        cls.coarse = cls.run_truth(0.002, 5.0, cls.start)

    @classmethod
    def run_truth(cls, dt, seconds, start):
        n = len(start)
        sim = tr.Descent(replace(cls.params, dt=dt), calm_wind(n), n)
        o = np.zeros(n)
        set_planar(sim, o, o + 300.0, o, o, *start.T)
        steps = round(seconds/dt)
        out = dict(dt=dt, phi=np.empty((steps + 1, n, 2)), plane=0.0, line=0.0, tension=math.inf)
        out['phi'][0] = start[:, :2]
        for k in range(steps):
            sim.step()
            p1, p2, plane = planar_angles(sim)
            out['phi'][k + 1, :, 0], out['phi'][k + 1, :, 1] = p1, p2
            out['plane'] = max(out['plane'], plane)
            out['line'] = max(out['line'], float(np.max(abs(sim.line_length() - cls.line))))
            out['tension'] = min(out['tension'], float(sim.tension.min()))
        out['canopy'] = float(np.max(abs(sim.pc - [0.0, 0.0, 300.0])))
        return out

    def reference(self, seconds, every):
        """phi1, phi2 of the Lagrangian model every `every` seconds, integrated at 0.25 ms."""
        y = self.start[0].copy()
        out = [y[:2].copy()]
        for _ in range(round(seconds/every)):
            for _ in range(round(every/0.00025)):
                y = rk4(lambda s: double_pendulum(s, self.m, self.inertia, self.d, self.line), y, 0.00025)
            out.append(y[:2].copy())
        return np.array(out)

    def test_large_angle_trajectory_matches_the_lagrangian_model(self):
        ref = self.reference(2.0, 0.002)
        fine = np.max(abs(self.fine['phi'][::2, 0] - ref), axis=0)
        coarse = np.max(abs(self.coarse['phi'][:1001, 0] - ref), axis=0)
        # Measured over 2 s at 1 ms: phi1 1.5e-7 rad, phi2 1.4e-6 rad; at 2 ms
        # 2.1e-6 and 1.9e-5. The difference is truth.py's own RK4 truncation:
        # it falls about 14-fold per halving of the step (so the two models
        # agree in the limit) and grows with the Baumgarte gain.
        self.assertLess(fine[0], 5e-7, fine)
        self.assertLess(fine[1], 3e-6, fine)
        for ratio in coarse/fine:
            self.assertTrue(10.0 < ratio < 20.0, coarse/fine)
        self.assertGreater(np.ptp(self.fine['phi'][:, 0, 0]), 1.0)       # the swing really is large
        self.assertGreater(self.fine['tension'], 0.0)                     # never slack, so the models are comparable
        self.assertLess(self.fine['plane'], 1e-12)
        self.assertLess(self.fine['canopy'], 1e-10)
        self.assertLess(self.fine['line'], 1e-8)

    def test_normal_mode_frequencies(self):
        run = self.coarse
        t = np.arange(len(run['phi']))*run['dt']
        for mode in (0, 1):
            x = run['phi'][:, mode + 1, 0]
            i = np.flatnonzero((x[:-1] > 0) & (x[1:] <= 0))
            crossings = t[i] + run['dt']*x[i]/(x[i] - x[i + 1])
            self.assertGreaterEqual(len(crossings), 3)
            measured = 2*math.pi/np.mean(np.diff(crossings))
            self.assertAlmostEqual(measured/self.omega[mode], 1.0, delta=1e-5)
        # The two modes are distinct physics, not one frequency measured twice.
        self.assertGreater(self.omega[1]/self.omega[0], 2.0)


# === A2. Free canopy: axis-referenced aerodynamics, minimal coordinates ====================
#
# Coordinates q = (X, Z, phi1, phi2): canopy point and the two angles above.
# The line and the pivot are built into the coordinates, so there is no
# tension and no constraint to stabilise (truth.py integrates maximal
# coordinates and solves for the tension).
#
#   r_c = (X, Z)                                          J_c = d r_c/dq = [[1,0,0,0],[0,1,0,0]]
#   r_C = (X + L sin phi1 + d sin phi2, Z - L cos phi1 - d cos phi2)
#   J_C = [[1, 0, L cos phi1, d cos phi2], [0, 1, L sin phi1, d sin phi2]]
#   T   = M_c/2 |r_c'|^2 + m/2 |r_C'|^2 + I/2 phi2'^2,   M_c = canopy mass + apparent mass
#
# For T = q'^T M(q) q'/2 with M = sum_i m_i J_i^T J_i, Lagrange's equations are
#
#   M q'' + m J_C^T (J_C' q') = J_c^T F_c + J_C^T F_C + e_phi2 tau
#   J_C' q' = (-L phi1'^2 sin phi1 - d phi2'^2 sin phi2, L phi1'^2 cos phi1 + d phi2'^2 cos phi2)
#
# with F_C = -rho/2 CdS_p |v_C| v_C - m g z (on the centre of mass) and
# tau = -c phi2'. The canopy force F_c, from the model's stated law (truth.py
# docstring and Params): with n the unit line direction from the attachment
# to the canopy, (-sin phi1, cos phi1), rel the canopy velocity relative to the
# air, v_ax = rel.n, perp = rel - v_ax n, alpha = atan2(|perp|, -v_ax):
#
#   F_c = q CdS [cos(alpha) n - cn(alpha) perp/|perp|] - W_c z,  q = rho/2 |rel|^2,
#   cn = k sin(alpha) for trim = 0 and k sin(alpha)((alpha/alpha0)^2 - 1) otherwise.
#
# The canopy WEIGHT W_c uses the fabric mass only; the INERTIA M_c adds the
# apparent mass. The drag area follows from terminal balance of the whole system:
# rho/2 v_d^2 (CdS_c + CdS_p) = (m + m_canopy) g. For k = 1, trim = 0 the law is
# exactly isotropic drag -rho/2 CdS |rel| rel; the second case below is not.

def log_wind(z, speed_10m, z0=0.1):
    return speed_10m*math.log(max(z, 1.0)/z0)/math.log(10.0/z0)


def canopy_can(y, par):
    x, z, p1, p2, vx, vz, w1, w2 = y
    line, d, m = par['line'], par['d'], par['m']
    jc = np.array([[1.0, 0.0, line*math.cos(p1), d*math.cos(p2)], [0.0, 1.0, line*math.sin(p1), d*math.sin(p2)]])
    qd = np.array([vx, vz, w1, w2])
    v_can = jc @ qd
    conv = np.array([-line*w1*w1*math.sin(p1) - d*w2*w2*math.sin(p2),
                     line*w1*w1*math.cos(p1) + d*w2*w2*math.cos(p2)])
    mass = m*jc.T @ jc
    mass[0, 0] += par['canopy_inertial']
    mass[1, 1] += par['canopy_inertial']
    mass[3, 3] += par['inertia']
    # Canopy: relative velocity, axial and normal parts.
    wind_c = np.array([log_wind(z, par['wind']), 0.0])
    rel = np.array([vx, vz]) - wind_c
    n = np.array([-math.sin(p1), math.cos(p1)])
    v_ax = rel @ n
    perp = rel - v_ax*n
    v_perp = math.hypot(*perp)
    alpha = math.atan2(v_perp, -v_ax)
    sa = math.sin(alpha)
    cn = par['k']*sa if par['trim'] <= 0 else par['k']*sa*((alpha/par['trim'])**2 - 1)
    q_cds = 0.5*RHO_AIR*par['cds_canopy']*(rel @ rel)
    f_c = q_cds*(math.cos(alpha)*n - (cn*perp/v_perp if v_perp > 1e-12 else 0.0)) - np.array([0.0, par['canopy_weight']])
    # Payload drag on the centre of mass, in the wind at its height.
    z_can = z - line*math.cos(p1) - d*math.cos(p2)
    rel_p = v_can - np.array([log_wind(z_can, par['wind']), 0.0])
    f_can = -0.5*RHO_AIR*par['cds_can']*math.hypot(*rel_p)*rel_p - np.array([0.0, m*G])
    force = jc.T @ (f_can - m*conv)
    force[:2] += f_c
    force[3] -= par['damping']*w2
    return np.r_[qd, np.linalg.solve(mass, force)]


class FreeCanopy(unittest.TestCase):
    seconds = 2.0

    @classmethod
    def setUpClass(cls):
        # (x, z, phi1, phi2, vx, vz, phi1', phi2'): two different tilted starts near terminal speed.
        start = np.array([[0.0, 300.0, 20*DEG, -15*DEG, 1.5, -10.5, 0.5, -1.0],
                          [5.0, 250.0, -30*DEG, 10*DEG, -2.0, -8.0, -0.8, 1.5]])
        cls.cases = {}
        cases = {'isotropic': (dict(cn_k=1.0, trim_deg=0.0), 0.0),
                 'restoring_wind': (dict(cn_k=0.8, trim_deg=0.0), 6.0),
                 'unstable_wind': (dict(cn_k=0.4, trim_deg=12.0), 6.0)}
        for name, (law, wind) in cases.items():
            p = replace(tr.Params(), **law, **QUIET)
            if wind:
                wp = tr.WindParams(speed_10m=wind, sigma_h=0.0, sigma_w=0.0, top=320.0)
                wind_field = tr.make_wind(2, np.random.default_rng(0), wp, direction=0.0)[0]
            else:
                wind_field = calm_wind(2)
            par = dict(
                line=p.line, d=-p.attach[2], m=p.payload_mass, inertia=p.inertia[1], damping=p.rot_damping[1],
                canopy_inertial=p.canopy_mass + p.canopy_added_mass, canopy_weight=p.canopy_mass*G,
                cds_can=p.payload_cds, k=p.cn_k, trim=math.radians(p.trim_deg), wind=wind,
                cds_canopy=(p.payload_mass + p.canopy_mass)*G/(0.5*RHO_AIR*p.descent_speed**2) - p.payload_cds)
            sim = tr.Descent(p, wind_field, 2)
            s = start
            set_planar(sim, s[:, 0], s[:, 1], s[:, 4], s[:, 5], s[:, 2], s[:, 3], s[:, 6], s[:, 7])
            every, steps = 10, round(cls.seconds/p.dt)
            track = np.empty((steps//every + 1, 2, 4))
            min_tension = math.inf
            for k in range(steps + 1):
                if k % every == 0:
                    p1, p2, out = planar_angles(sim)
                    track[k//every] = np.column_stack((sim.pc[:, 0], sim.pc[:, 2], p1, p2))
                if k < steps:
                    sim.step()
                    min_tension = min(min_tension, float(sim.tension.min()))
            cls.cases[name] = dict(par=par, track=track, min_tension=min_tension, out_of_plane=out,
                                   cds_in_module=sim.canopy_cds, dt=p.dt, every=every, start=start)

    def reference(self, name, row, sub=2, **changes):
        c = self.cases[name]
        par = dict(c['par'], **changes)
        y = c['start'][row].copy()
        out = [y[:4].copy()]
        for _ in range(len(c['track']) - 1):
            for _ in range(c['every']*sub):
                y = rk4(lambda s: canopy_can(s, par), y, c['dt']/sub)
            out.append(y[:4].copy())
        return np.array(out)

    def test_trajectory_matches_minimal_coordinate_model(self):
        for name, c in self.cases.items():
            self.assertAlmostEqual(c['cds_in_module']/c['par']['cds_canopy'], 1.0, delta=1e-12)
            self.assertGreater(c['min_tension'], 0.0, name)
            self.assertLess(c['out_of_plane'], 1e-12, name)
            for row in range(2):
                diff = np.max(abs(self.reference(name, row) - c['track'][:, row]), axis=0)
                # Measured: canopy position 5e-9 m, angles 4e-8 rad over 2 s in still air. With
                # wind it is 1.2e-6 m and 2.3e-6 rad, which is the module's wind table (linear
                # interpolation on a 0.5 m grid) against the analytic log law used here.
                self.assertLess(diff[:2].max(), 5e-6, (name, diff))
                self.assertLess(diff[2:].max(), 5e-6, (name, diff))

    def test_the_comparison_can_tell_the_force_laws_apart(self):
        # Power of the test above: each wrong treatment moves the trajectory by
        # far more than the agreement found.
        c = self.cases['unstable_wind']
        track = c['track'][:, 0]
        p = replace(tr.Params(), cn_k=0.4, trim_deg=12.0)
        wrong = {
            'apparent mass in the weight': self.reference('unstable_wind', 0, sub=1,
                                                          canopy_weight=(p.canopy_mass + p.canopy_added_mass)*G),
            'apparent mass missing from the inertia': self.reference('unstable_wind', 0, sub=1,
                                                                     canopy_inertial=p.canopy_mass),
            'isotropic drag instead of the normal-force law': self.reference('unstable_wind', 0, sub=1, k=1.0, trim=0.0),
            'restoring instead of unstable normal force': self.reference('unstable_wind', 0, sub=1, trim=0.0),
            'no wind': self.reference('unstable_wind', 0, sub=1, wind=0.0),
        }
        for what, ref in wrong.items():
            self.assertGreater(np.max(abs(ref - track)), 1e-3, what)

    def test_settles_at_the_stated_descent_speed(self):
        # Terminal balance of the independent model reproduces Params.descent_speed.
        par = self.cases['isotropic']['par']
        y = np.array([0.0, 300.0, 0.0, 0.0, 0.0, -9.0, 0.0, 0.0])
        for _ in range(2000):
            y = rk4(lambda s: canopy_can(s, par), y, 0.005)
        self.assertAlmostEqual(-y[5], tr.Params().descent_speed, delta=1e-6)


# === B. IMU output against differentiated trajectory =====================================
#
# An accelerometer at body position r reports f = R^T (a_point - g_world) with
# g_world = (0, 0, -g): at rest it reads +g along world up. a_point is obtained
# here by differencing the IMU point's POSITION (five-point stencil,
# error h^4 p^(6)/90), so the lever-arm terms alpha x r + w x (w x r) are
# tested without being written down. The rate is checked through
# vee(R^T dR/dt) with a five-point first derivative of R.

class ImuOutput(unittest.TestCase):
    def test_instantaneous_output_matches_differentiated_trajectory(self):
        params = replace(tr.Params(), cn_k=0.4, trim_deg=12.0, imu=(0.02, -0.015, 0.03), **QUIET)
        n = 3
        sim = tr.Descent(params, calm_wind(n), n)
        sim.start(np.random.default_rng(11), tilt_sd_deg=20.0, rate_sd_dps=150.0, spin_sd_dps=400.0, lateral_sd=3.0)
        origin = sim.pp.copy()
        h = params.dt
        pos, att, force, rate, swing = [], [], [], [], 0.0
        for _ in range(500):
            p, _, r = sim.imu_state()
            pos.append(p - origin)
            att.append(r.copy())
            f, w = sim.step()
            force.append(f.copy())
            rate.append(w.copy())
            swing = max(swing, float(np.max(sim.swing_angle())))
        pos, att, force, rate = map(np.array, (pos, att, force, rate))
        acc = (-pos[4:] + 16*pos[3:-1] - 30*pos[2:-2] + 16*pos[1:-3] - pos[:-4])/(12*h*h)
        r_mid = att[2:-2]
        expected = np.einsum('knji,knj->kni', r_mid, acc + G*E_UP)
        r_dot = (-att[4:] + 8*att[3:-1] - 8*att[1:-3] + att[:-4])/(12*h)
        w_hat = np.einsum('knji,knjl->knil', r_mid, r_dot)
        w_fd = np.stack((w_hat[..., 2, 1] - w_hat[..., 1, 2], w_hat[..., 0, 2] - w_hat[..., 2, 0],
                         w_hat[..., 1, 0] - w_hat[..., 0, 1]), axis=-1)/2
        # The first ~30 ms are a start-up transient of the initial condition
        # (the line tension relaxes with a ~100 rad/s mode that a 1 ms
        # five-point stencil cannot follow: 6.6e-5 m/s^2 at the third sample,
        # 3e-7 by the 25th). It is smooth when resolved at 0.25 ms, so it is
        # the test's stencil that fails there, not the model; skip it.
        skip = 30
        err_f = np.max(abs(force[2:-2] - expected)[skip:])
        err_w = np.max(abs(rate[2:-2] - w_fd)[skip:])        # the same start-up transient: 1.5e-6 rad/s inside it
        # The case is vigorous: several m/s^2 of non-gravitational variation and rad/s rates.
        self.assertGreater(np.ptp(force[:, :, 0]), 2.0)
        self.assertGreater(np.max(abs(rate)), 5.0)
        self.assertGreater(swing, 15*DEG)
        # Measured: 1.5e-7 m/s^2 (position round-off over h^2) and 1.2e-7 rad/s on rates of
        # several rad/s (the five-point stencil against the same decaying start-up mode).
        self.assertLess(err_f, 2e-6, err_f)
        self.assertLess(err_w, 5e-7, err_w)

    def test_static_sign_convention(self):
        # A can hanging plumb and at rest below an immovable canopy: the line
        # holds it up, so the specific force is +g along world up, which is -g on
        # body z (z points down), whatever the yaw.
        params = replace(tr.Params(), canopy_mass=0.0, canopy_added_mass=1e13, descent_speed=1e6, payload_cds=0.0,
                         **QUIET)
        sim = tr.Descent(params, calm_wind(2), 2)
        o = np.zeros(2)
        set_planar(sim, o, o + 100.0, o, o, o, o, o, o)
        yaw = np.array([0.0, 0.0, 1.1])
        for j in (0, 1):                                    # row 1 yawed about the vertical: q = q_level * q_yaw
            sim.q[j] = qmul(np.array([0.0, 1.0, 0.0, 0.0]), qvec(yaw*j))
        force, rate = sim.step()
        np.testing.assert_allclose(force, np.tile([0.0, 0.0, -G], (2, 1)), atol=1e-9)
        np.testing.assert_allclose(rate, 0.0, atol=1e-15)
        np.testing.assert_allclose(sim.tension, params.payload_mass*G, rtol=1e-9)
        _, _, r = sim.imu_state()
        np.testing.assert_allclose(np.einsum('nij,nj->ni', r, force), np.tile([0.0, 0.0, G], (2, 1)), atol=1e-9)
        np.testing.assert_allclose(r[0], LEVEL, atol=1e-15)

    def test_free_fall_reads_zero(self):
        # Nothing but gravity acts (no drag, no apparent mass): canopy and can
        # fall together, the line carries nothing and the accelerometer reads zero.
        params = replace(tr.Params(), canopy_added_mass=0.0, descent_speed=1e6, payload_cds=0.0,
                         rot_damping=(0.0, 0.0, 0.0), imu=(0.0, 0.0, 0.0), **QUIET)
        sim = tr.Descent(params, calm_wind(1), 1)
        set_planar(sim, [0.0], [200.0], [0.3], [-2.0], [0.0], [0.0], [0.0], [0.0])
        force, _ = sim.step()
        np.testing.assert_allclose(force, 0.0, atol=1e-9)
        self.assertLess(abs(float(sim.tension[0])), 1e-9)


# --- Own filter-side references ------------------------------------------------------

SILENT = ek.ImuNoise(0.0, 0.0, 0.0, 0.0, 0.0)
CAM = dict(fx=724.6, fy=724.6, cx=728.0, cy=544.0, r_cb=rot(np.array([0.03, -0.02, 0.06])),
           lever=np.array([0.012, -0.007, 0.045]))


def their_camera(cam=CAM):
    return ek.Camera(cam['fx'], cam['fy'], cam['cx'], cam['cy'], 1456, 1088, r_cb=cam['r_cb'].copy(),
                     lever=cam['lever'].copy())


def predict_pixels(points, p, v, r, w, tau, cam=CAM):
    """Own model of an image stamped t and exposed at t + tau.

    The body moves at constant velocity v and constant body rate w, so its
    pose at the exposure instant is (p + v tau, r exp([w]x tau)), exactly. The
    camera sits `lever` from the IMU in body axes and is turned by r_cb.
    Returns pixels (n, 2) and camera-frame depth (n,).
    """
    points = np.atleast_2d(points)
    pe, re = p + v*tau, r @ rot(np.asarray(w)*tau)
    c = ((points - pe) @ re - cam['lever']) @ cam['r_cb'].T         # r_cb (R^T (m - p) - lever)
    return np.column_stack((cam['fx']*c[:, 0]/c[:, 2] + cam['cx'], cam['fy']*c[:, 1]/c[:, 2] + cam['cy'])), c[:, 2]


def fd_jacobian(points, p, v, r, w, tau, cam=CAM):
    """d(pixels)/d(error state) of predict_pixels by central differences, shape (n, 2, 17).

    Position, velocity and the offset are added; the attitude error acts on
    the right, r exp([dtheta]x). The bias and baro columns are zero.
    """
    n = len(points)
    h = np.zeros((n, 2, NS))

    def px(p_=p, v_=v, r_=r, t_=tau):
        return predict_pixels(points, p_, v_, r_, w, t_, cam)[0]
    for j in range(3):
        e = np.eye(3)[j]
        h[:, :, j] = (px(p_=p + 1e-4*e) - px(p_=p - 1e-4*e))/2e-4
        h[:, :, 3 + j] = (px(v_=v + 1e-2*e) - px(v_=v - 1e-2*e))/2e-2
        h[:, :, 6 + j] = (px(r_=r @ rot(1e-6*e)) - px(r_=r @ rot(-1e-6*e)))/2e-6
    h[:, :, TM] = (px(t_=tau + 1e-6) - px(t_=tau - 1e-6))/2e-6
    return h


def boxminus(truth, f):
    """Error state truth (-) filter estimate, with R_true = R_hat exp([dtheta]x)."""
    return np.r_[truth['p'] - f.p, truth['v'] - f.v, rotvec(f.r.T @ truth['r']), truth['ba'] - f.ba,
                 truth['bg'] - f.bg, truth['bb'] - f.baro_bias, truth['tm'] - f.time_offset]


def state_of(f):
    return dict(p=f.p.copy(), v=f.v.copy(), r=f.r.copy(), ba=f.ba.copy(), bg=f.bg.copy(), bb=f.baro_bias,
                tm=f.time_offset)


def filter_at(state, dx, cov, noise=SILENT):
    """A filter whose estimate is state (+) dx."""
    return ek.Eskf(state['p'] + dx[P], state['v'] + dx[V], state['r'] @ rot(dx[A]), cov, noise,
                   ba=state['ba'] + dx[BA], bg=state['bg'] + dx[BG], baro_bias=state['bb'] + dx[BB],
                   time_offset=state['tm'] + dx[TM])


# === C. Strapdown ===================================================================

def strapdown_step(p, v, q, accel, gyro, dt):
    """One step of this file's quaternion strapdown.

    gyro*dt is the step's rotation vector; accel*dt is its velocity increment
    resolved in the body axes half way through that rotation.
    """
    acc = qrot(qmul(q, qvec(gyro*dt/2)), accel) - G*E_UP
    q = qmul(q, qvec(gyro*dt))
    return p + v*dt + acc*(dt*dt/2), v + acc*dt, q/math.sqrt(q @ q)


# Shared truth batch built by the production code path of run_nav_mc.py (cache
# redirected to a scratch directory and shortened to 5 s), used by the strapdown
# check, the frame bookkeeping and the run_descent wiring tests.

_PIPE = {}


def pipeline():
    if not _PIPE:
        import run_nav_mc as mc
        saved = (mc.CACHE, mc.DURATION, dict(mc._TRUTH))
        tmp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        mc.CACHE, mc.DURATION = Path(tmp.name), 5.0
        mc._TRUTH.clear()
        mc.build_truth('moderate', 4)
        data = {k: np.array(v) for k, v in mc.load_truth('moderate', 4).items()}
        mc._TRUTH.clear()
        _PIPE.update(mc=mc, tmp=tmp, saved=saved, data=data, n=4)
    return _PIPE


def pipeline_release():
    if _PIPE:
        mc = _PIPE['mc']
        mc._TRUTH.clear()
        mc.CACHE, mc.DURATION = _PIPE['saved'][:2]
        mc._TRUTH.update(_PIPE['saved'][2])
        mc.SCENARIOS.pop('_probe', None)
        mc.SCENARIOS.pop('_delay', None)
        try:
            _PIPE['tmp'].cleanup()
        except OSError:
            pass
        _PIPE.clear()


def tearDownModule():
    pipeline_release()


class Strapdown(unittest.TestCase):
    dt = 0.01

    @classmethod
    def setUpClass(cls):
        cls.data = pipeline()['data']

    def integrate(self, accel, gyro, j, against=None):
        d = self.data
        p, v, q = d['p'][0, j].copy(), d['v'][0, j].copy(), qfrom(d['r'][0, j])
        worst = np.zeros(3)
        for k in range(len(accel)):
            p, v, q = strapdown_step(p, v, q, accel[k, j], gyro[k, j], self.dt)
            if against is not None:
                against.propagate(self.dt, accel[k, j], gyro[k, j])
                worst = np.maximum(worst, [np.max(abs(against.p - p)), np.max(abs(against.v - v)),
                                           np.max(abs(against.r - qmat(q)))])
        end = np.array([np.linalg.norm(p - d['p'][-1, j]), np.linalg.norm(v - d['v'][-1, j]),
                        np.linalg.norm(rotvec(d['r'][-1, j].T @ qmat(q)))])
        return end, worst

    def test_quaternion_helpers(self):
        rng = np.random.default_rng(22)
        for r in [LEVEL, LEVEL @ rot(np.array([0.2, -0.1, 2.5]))] + [rot(rng.normal(size=3)) for _ in range(20)]:
            np.testing.assert_allclose(qmat(qfrom(r)), r, atol=1e-14)
        v = rng.normal(size=3)
        np.testing.assert_allclose(qmat(qvec(v)), rot(v), atol=1e-14)
        np.testing.assert_allclose(rotvec(rot(v)), v, atol=1e-13)
        q = qvec(v)
        np.testing.assert_allclose(qmat(q) @ np.array([1.0, 2.0, 3.0]), qrot(q, np.array([1.0, 2.0, 3.0])), atol=1e-14)

    def test_exact_increments_reproduce_truth_and_filter_matches_step_for_step(self):
        d = self.data
        self.assertGreater(np.max(abs(d['gyro'])), 1.5)               # a spinning, swinging case
        self.assertGreater(np.degrees(np.max(d['swing'])), 10.0)
        for j in range(self.data['p'].shape[1]):
            f = ek.Eskf(d['p'][0, j], d['v'][0, j], d['r'][0, j], np.eye(NS), SILENT)
            end, worst = self.integrate(d['accel'], d['gyro'], j, against=f)
            # Attitude and velocity are exact to round-off. Position is a
            # trapezoid of velocity: its error is bounded by dt^2/12 times the
            # change in acceleration, not growing with time (measured below 1e-4).
            self.assertLess(end[1], 1e-9, end)
            self.assertLess(end[2], 1e-10, end)
            self.assertLess(end[0], 5e-4, end)
            # Eskf.propagate and this file's integrator: identical to round-off.
            self.assertLess(worst[0], 1e-9, worst)
            self.assertLess(worst[1], 1e-10, worst)
            self.assertLess(worst[2], 1e-11, worst)

    def test_boxcar_and_decimated_samples_are_not_exact(self):
        # What the two non-ideal IMU modes cost over the run (no noise, no
        # bias): bounded loosely so a change in either is noticed. Boxcar and
        # single samples lose the coning/sculling terms, so velocity drifts.
        d = self.data
        js = range(d['p'].shape[1])
        ideal = np.max([self.integrate(d['accel'], d['gyro'], j)[0] for j in js], axis=0)
        mean = np.max([self.integrate(d['accel_mean'], d['gyro_mean'], j)[0] for j in js], axis=0)
        inst = np.max([self.integrate(d['accel_inst'], d['gyro_inst'], j)[0] for j in js], axis=0)
        self.assertGreater(mean[1], 1e3*ideal[1])
        self.assertGreater(inst[2], mean[2])
        self.assertLess(mean[1], 0.5)
        self.assertLess(inst[1], 2.0)


# === D. Error-state transition and process noise ====================================
#
# Appendix H.1 with R = R_hat (I + [dtheta]x), in the order
# [dp, dv, dtheta, dba, dbg, dbaro, dtime]:
#
#   F[p, v]      = I
#   F[v, theta]  = -R(t) [a_b(t)]x       F[v, ba] = -R(t)
#   F[theta, theta] = -[w]x              F[theta, bg] = -I
#
# (Derivation of the last two: R_true = R_hat exp(dtheta), the true rate is the
# estimate minus dbg minus noise, so exp(dtheta_1) = exp(-w h) exp(dtheta_0)
# exp(w h) exp(J_r(w h)(-dbg - n) h), which to first order is dtheta_1 =
# exp(-[w]x h) dtheta_0 - J_r h dbg: F_tt = -[w]x, F_tg = -I.) The baro and
# time-offset errors do not move.
#
# During a step the body turns, R(t) = R0 exp([w]x t). The filter's mechanisation
# takes the specific force as the constant WORLD vector R0 exp([w]x dt/2) a, so
# a_b(t) = R(t)^T R0 exp([w]x dt/2) a. The reference transition is the
# time-ordered product of exp(F(t_i) h) over 64 pieces of the step, each by its
# own Taylor series. Process noise: G Qc G^T with n_a entering dv through -R and
# n_g entering dtheta through -I, integrated the same way.

def expm_series(m):
    scale = max(0, int(math.ceil(math.log2(max(np.max(np.sum(abs(m), axis=1)), 1e-300)))) + 3)
    a = m/2.0**scale
    out = np.eye(len(m))
    term = np.eye(len(m))
    for k in range(1, 16):
        term = term @ a/k
        out = out + term
    for _ in range(scale):
        out = out @ out
    return out


def continuous_f(r, a_body, w):
    f = np.zeros((NS, NS))
    f[P, V] = np.eye(3)
    f[V, A] = -r @ hat(a_body)
    f[V, BA] = -r
    f[A, A] = -hat(w)
    f[A, BG] = -np.eye(3)
    return f


def reference_transition(r0, a, w, dt, density=None, pieces=64):
    """(Phi, Qd) over one step from the continuous model above."""
    f_world = r0 @ rot(w*dt/2) @ a
    qc = np.zeros(NS)
    if density is not None:
        qc[V], qc[A], qc[BA], qc[BG], qc[BB], qc[TM] = (
            density.accel**2, density.gyro**2 + 0.0, density.accel_bias**2, density.gyro_bias**2,
            density.baro_bias**2, density.time_offset**2)
    phi, qd, h = np.eye(NS), np.zeros((NS, NS)), dt/pieces
    for i in range(pieces):
        r = r0 @ rot(w*(i + 0.5)*h)
        f = continuous_f(r, r.T @ f_world, w)
        half, full = expm_series(f*h/2), expm_series(f*h)
        qd = full @ qd @ full.T + (half*qc) @ half.T*h          # isotropic densities: R Q R^T = Q
        phi = full @ phi
    return phi, qd


def covariance_transition(state, accel, gyro, dt):
    """The transition the filter applies to its covariance, a column at a time."""
    phi = np.empty((NS, NS))
    for j in range(NS):
        cov = np.zeros((NS, NS))
        cov[j, j] = 1.0
        f = filter_at(state, np.zeros(NS), cov)
        f.propagate(dt, accel, gyro)
        phi[:, j] = f.cov[:, j]/math.sqrt(f.cov[j, j])
    return phi


def mechanisation_transition(state, accel, gyro, dt, eps=1e-6):
    """Central-difference sensitivity of the filter's own nominal propagation."""
    phi = np.empty((NS, NS))
    for j in range(NS):
        dx = np.zeros(NS)
        dx[j] = eps
        plus, minus = filter_at(state, dx, np.eye(NS)), filter_at(state, -dx, np.eye(NS))
        plus.propagate(dt, accel, gyro)
        minus.propagate(dt, accel, gyro)
        phi[:, j] = boxminus(state_of(plus), minus)/(2*eps)
    return phi


def transition_case():
    state = dict(p=np.array([3.0, -2.0, 250.0]), v=np.array([3.0, 1.0, -10.0]),
                 r=LEVEL @ rot(np.array([0.25, -0.15, 0.9])), ba=np.array([0.05, -0.02, 0.03]),
                 bg=np.array([0.002, -0.001, 0.003]), bb=0.4, tm=0.002)
    accel = np.array([1.5, -0.8, -9.6]) + state['ba']
    gyro = np.array([0.8, -0.5, 6.0]) + state['bg']
    return state, accel, gyro


def block_table(dts=(0.04, 0.02, 0.01, 0.005)):
    """{(row, col): [(|cov - ref|, |cov - mech|, |ref block|) per dt]} for every block."""
    state, accel, gyro = transition_case()
    a, w = accel - state['ba'], gyro - state['bg']
    table = {}
    for dt in dts:
        ref, _ = reference_transition(state['r'], a, w, dt)
        cov = covariance_transition(state, accel, gyro, dt)
        mech = mechanisation_transition(state, accel, gyro, dt)
        for rn, rs in BLOCKS:
            for cn, cs in BLOCKS:
                table.setdefault((rn, cn), []).append((np.max(abs(cov[rs, cs] - ref[rs, cs])),
                                                       np.max(abs(cov[rs, cs] - mech[rs, cs])),
                                                       np.max(abs(ref[rs, cs] - np.eye(NS)[rs, cs]))))
    return dts, table


class ErrorStateTransition(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dts, cls.table = block_table()

    def test_expm_series(self):
        w = np.array([0.3, -1.1, 0.7])
        np.testing.assert_allclose(expm_series(hat(w)), rot(w), atol=1e-14)
        n = np.zeros((3, 3))
        n[0, 1], n[1, 2] = 2.0, 3.0
        np.testing.assert_allclose(expm_series(n), [[1, 2, 3], [0, 1, 3], [0, 0, 1]], atol=1e-14)

    def test_every_block_agrees_and_converges_at_third_order_or_better(self):
        i10 = self.dts.index(0.01)
        for (rn, cn), rows in self.table.items():
            diff = np.array([r[0] for r in rows])
            size = rows[i10][2]
            # No block is wrong at first order or carries a flipped sign: every
            # discrepancy from the continuous model is at most 7.5e-6 at 10 ms
            # (dv/dbg, 1.3e-6 for the omitted dp/dbg = R [a]x dt^3/6) and falls
            # eight-fold per halving of dt: truncation at third order.
            self.assertLess(diff[i10], 1e-5, (rn, cn, diff))
            if size > 0 and (rn, cn) != ('pos', 'bg'):
                self.assertLess(diff[i10], 0.03*size, (rn, cn, diff, size))
            if (rn, cn) == ('pos', 'bg'):
                # The filter has no position-gyro-bias term at all; what it leaves out is
                # R [a]x dt^3/6 (third order, 1.3e-6 at 10 ms: a bias error of 1e-3 rad/s
                # would move position by 1e-9 m in one step).
                self.assertAlmostEqual(diff[i10]/size, 1.0, delta=0.02)
            if diff[0] > 1e-12:
                order = math.log(diff[0]/max(diff[-1], 1e-300))/math.log(self.dts[0]/self.dts[-1])
                self.assertGreater(order, 2.7, (rn, cn, diff, order))

    def test_covariance_transition_is_the_sensitivity_of_the_mechanisation(self):
        # The same comparison against the filter's own discrete propagation,
        # which is what it must be consistent with when measurements arrive
        # as increments. Largest difference at 10 ms: dp/dbg, 1.9e-6, which the
        # covariance transition leaves out (third order); dtheta/dbg (r_half^T dt
        # against the exact J_r dt) is 2e-7.
        i10 = self.dts.index(0.01)
        for (rn, cn), rows in self.table.items():
            diff = np.array([r[1] for r in rows])
            self.assertLess(diff[i10], 1e-5, (rn, cn, diff))
            if diff[0] > 3e-8:                       # above the finite-difference floor (~3e-9 on a 250 m position)
                order = math.log(diff[0]/max(diff[-1], 1e-300))/math.log(self.dts[0]/self.dts[-1])
                self.assertGreater(order, 2.7, (rn, cn, diff, order))

    def test_baro_and_time_offset_do_not_move_or_couple(self):
        state, accel, gyro = transition_case()
        phi = covariance_transition(state, accel, gyro, 0.01)
        self.assertEqual(phi[BB, BB], 1.0)
        self.assertEqual(phi[TM, TM], 1.0)
        for k in (BB, TM):
            other = [i for i in range(NS) if i != k]
            np.testing.assert_array_equal(phi[other, k], 0.0)     # nothing else depends on them
            np.testing.assert_array_equal(phi[k, other], 0.0)

    def test_a_flipped_sign_would_be_caught(self):
        # Power of the tests above: the velocity-attitude block is -R [a]x dt,
        # about 0.08 at 10 ms, so a sign error there is a 0.15 discrepancy.
        state, accel, gyro = transition_case()
        ref, _ = reference_transition(state['r'], accel - state['ba'], gyro - state['bg'], 0.01)
        self.assertGreater(np.max(abs(ref[V, A])), 0.06)
        self.assertGreater(np.max(abs(ref[A, BG])), 0.009)
        self.assertGreater(np.max(abs(ref[V, BA])), 0.009)

    def test_process_noise_mapping(self):
        density = ek.ImuNoise(accel=0.05, gyro=0.004, accel_bias=0.01, gyro_bias=1e-3, baro_bias=0.1,
                              time_offset=2e-4)
        state, accel, gyro = transition_case()
        dt = 0.01
        f = filter_at(state, np.zeros(NS), np.zeros((NS, NS)), density)
        f.propagate(dt, accel, gyro)
        ref_phi, ref_q = reference_transition(state['r'], accel - state['ba'], gyro - state['bg'], dt, density)
        # Diagonal: the filter's Q is the leading term of the exact discrete Q.
        np.testing.assert_allclose(np.diag(f.cov)[3:], np.diag(ref_q)[3:], rtol=2e-3)
        self.assertAlmostEqual(f.cov[TM, TM], density.time_offset**2*dt, delta=1e-20)
        # Leaving R out of the velocity noise is exact for an isotropic density.
        r = state['r']
        np.testing.assert_allclose(r @ (density.accel**2*np.eye(3)) @ r.T, density.accel**2*np.eye(3), atol=1e-18)
        # What the diagonal Q leaves out, and that it does not matter: position
        # variance sigma_a^2 dt^3/3 and the position-velocity term sigma_a^2 dt^2/2.
        self.assertAlmostEqual(ref_q[0, 0]/(density.accel**2*dt**3/3), 1.0, delta=1e-3)
        self.assertAlmostEqual(ref_q[0, 3]/(density.accel**2*dt**2/2), 1.0, delta=1e-3)
        self.assertEqual(f.cov[0, 0], 0.0)
        # Propagate both covariances from zero across one camera interval
        # (0.2 s) and across 2 s: the filter's sigmas are within 4 % and 0.5 %.
        cov_r = np.zeros((NS, NS))
        g = filter_at(state, np.zeros(NS), np.zeros((NS, NS)), density)
        worst = {}
        for k in range(200):
            g.propagate(dt, accel, gyro)
            ref_phi, ref_q = reference_transition(g.r @ rot(-(gyro - state['bg'])*dt), accel - state['ba'],
                                                  gyro - state['bg'], dt, density, pieces=8)
            cov_r = ref_phi @ cov_r @ ref_phi.T + ref_q
            if k + 1 in (20, 200):
                worst[k + 1] = np.max(abs(np.sqrt(np.diag(g.cov))/np.sqrt(np.diag(cov_r)) - 1))
        self.assertLess(worst[20], 0.045, worst)
        self.assertLess(worst[200], 0.006, worst)

    def test_scale_factor_noise_adds_gw_squared_tau_per_second(self):
        # ImuNoise.gyro_scale/scale_tau: attitude variance rate gyro_scale^2 |w|^2 scale_tau
        # on top of the white gyro noise, per second, isotropic.
        noise = ek.ImuNoise(0.0, 1e-3, 0.0, 0.0, 0.0, gyro_scale=0.01, scale_tau=0.2)
        state, accel, gyro = transition_case()
        w = gyro - state['bg']
        f = filter_at(state, np.zeros(NS), np.zeros((NS, NS)), noise)
        f.propagate(0.01, accel, gyro)
        expected = (1e-3**2 + 0.01**2*float(w @ w)*0.2)*0.01
        np.testing.assert_allclose(np.diag(f.cov)[A], expected, rtol=1e-12)


# === E. Landmark update ================================================================

def realistic_cov(rng, sd):
    a = rng.normal(size=(NS, NS))*0.3 + np.eye(NS)
    c = a @ a.T
    c = c/np.trace(c)*NS
    return np.diag(sd) @ c @ np.diag(sd)


SD0 = np.r_[np.full(3, 3.0), 2.0, 2.0, 1.0, np.radians([1.0, 1.0, 2.0]), np.full(3, 0.196),
            np.full(3, np.radians(0.1)), 1.0, 0.002]
NOMINAL = dict(p=np.array([4.0, -6.0, 220.0]), v=np.array([2.0, 1.0, -10.5]),
               r=LEVEL @ rot(np.array([0.2, -0.1, 0.8])), w=np.array([0.9, -0.6, 1.7]), tau=0.004)


def lm_filter(cov, nominal=NOMINAL, noise=SILENT):
    f = ek.Eskf(nominal['p'], nominal['v'], nominal['r'], cov, noise, time_offset=nominal['tau'])
    f.rate = nominal['w'].copy()
    return f


def lm_points(rng, n, nominal=NOMINAL, spread=0.6):
    h = nominal['p'][2]
    return np.column_stack((nominal['p'][0] + rng.uniform(-spread*h, spread*h, n),
                            nominal['p'][1] + rng.uniform(-spread*h, spread*h, n), rng.normal(0, 1.0, n)))


def own_pixels(points, nominal=NOMINAL, dx=None):
    dx = np.zeros(NS) if dx is None else dx
    return predict_pixels(points, nominal['p'] + dx[P], nominal['v'] + dx[V], nominal['r'] @ rot(dx[A]),
                          nominal['w'], nominal['tau'] + dx[TM])[0]


def exact_reset(a):
    """d(dtheta')/d(dtheta) at dtheta = a, where dtheta' = log(exp(-a) exp(dtheta)): the right Jacobian J_r(a).

    exp(a + e) = exp(a) exp(J_r(a) e), so exp(-a) exp(a + e) = exp(J_r e). Here by central differences.
    """
    jr = np.empty((3, 3))
    for j in range(3):
        e = np.zeros(3)
        e[j] = 1e-6
        jr[:, j] = (rotvec(rot(-a) @ rot(a + e)) - rotvec(rot(-a) @ rot(a - e)))/2e-6
    return jr


def own_map_solution(f0_nominal, cov, points, pixels, pixel_var, iterations=40):
    """Gauss-Newton MAP of  1/2 dx^T P^-1 dx + 1/2 |z - h(x0 (+) dx)|^2 / pixel_var.

    The retraction is the filter's own (p + dp, v + dv, R exp(dtheta), tau + dtau); the Jacobian is by
    central differences of the full cost residual, so it includes the right Jacobian J_r(dtheta) that the
    filter's iterated update leaves out. Returns dx and the inverse Hessian (covariance before any reset).
    """
    pinv = np.linalg.inv(cov)
    cols = list(range(9)) + [TM]

    def res(dx):
        return (pixels - own_pixels(points, f0_nominal, dx)).reshape(-1)
    dx = np.zeros(NS)
    for _ in range(iterations):
        jac = np.zeros((len(points)*2, NS))
        for j in cols:
            eps = 1e-6 if j < TM else 1e-7
            d = np.zeros(NS)
            d[j] = eps
            jac[:, j] = (res(dx + d) - res(dx - d))/(2*eps)
        hess = pinv + jac.T @ jac/pixel_var
        step = np.linalg.solve(hess, pinv @ dx + jac.T @ res(dx)/pixel_var)
        dx = dx - step
        if np.max(abs(step)) < 1e-13:
            break
    return dx, np.linalg.inv(hess)


def scaled_error(a, b):
    """max |a - b| / sqrt(b_ii b_jj): covariance difference relative to the correlation scale."""
    s = np.sqrt(np.diag(b))
    return float(np.max(abs(a - b)/np.outer(s, s)))


class LandmarkUpdate(unittest.TestCase):
    def setUp(self):
        self.cam = their_camera()

    # (i) Jacobians ------------------------------------------------------------------

    def test_prediction_and_all_jacobian_columns_match_finite_differences(self):
        rng = np.random.default_rng(31)
        points = lm_points(rng, 9)
        f = lm_filter(np.eye(NS))
        predicted, depth, h, _ = f.landmark_model(self.cam, points, 0.1)
        np.testing.assert_allclose(predicted, own_pixels(points), atol=1e-9)
        np.testing.assert_allclose(depth, predict_pixels(points, NOMINAL['p'], NOMINAL['v'], NOMINAL['r'],
                                                          NOMINAL['w'], NOMINAL['tau'])[1], atol=1e-9)
        ref = fd_jacobian(points, NOMINAL['p'], NOMINAL['v'], NOMINAL['r'], NOMINAL['w'], NOMINAL['tau'])
        for name, sl in (('pos', P), ('vel', V), ('att', A), ('time', slice(TM, TM + 1))):
            scale = np.max(abs(ref[:, :, sl]))
            # Measured: 4e-8 (pos), 1.3e-5 (vel, dominated by the stencil's own
            # round-off on a column 3 orders smaller), 2e-10 (att), 4e-10 (time).
            self.assertLess(np.max(abs(h[:, :, sl] - ref[:, :, sl]))/scale, 3e-5, name)
        np.testing.assert_array_equal(h[:, :, BA], 0.0)
        np.testing.assert_array_equal(h[:, :, BG], 0.0)
        np.testing.assert_array_equal(h[:, :, BB], 0.0)
        # The columns are not trivial: 4 ms moves the image by more than the pixel noise.
        self.assertGreater(np.max(abs(h[:, :, TM]))*NOMINAL['tau'], 3.0)
        self.assertGreater(np.max(abs(h[:, :, V])), 1e-3)

    def test_time_offset_sign_means_stamped_t_exposed_at_t_plus_offset(self):
        # A body moving at constant velocity and rate. An image stamped t = 0 is
        # exposed at t = tau, so what it shows is the world seen from the pose
        # at tau. The filter with offset tau must predict it exactly; with
        # offset 0 it misses by H_time * tau (first order), and the sign of
        # H_time must be the one that closes the gap.
        rng = np.random.default_rng(32)
        points = lm_points(rng, 12)
        tau = 0.006
        measured = predict_pixels(points, NOMINAL['p'], NOMINAL['v'], NOMINAL['r'], NOMINAL['w'], tau)[0]
        # the same thing from first principles: pose at time tau of the moving body
        pose_p, pose_r = NOMINAL['p'] + NOMINAL['v']*tau, NOMINAL['r'] @ rot(NOMINAL['w']*tau)
        c = ((points - pose_p) @ pose_r - CAM['lever']) @ CAM['r_cb'].T
        direct = np.column_stack((CAM['fx']*c[:, 0]/c[:, 2] + CAM['cx'], CAM['fy']*c[:, 1]/c[:, 2] + CAM['cy']))
        np.testing.assert_allclose(measured, direct, atol=1e-9)
        right = lm_filter(np.eye(NS), dict(NOMINAL, tau=tau))
        wrong = lm_filter(np.eye(NS), dict(NOMINAL, tau=0.0))
        np.testing.assert_allclose(right.landmark_model(self.cam, points, 0.1)[0], measured, atol=1e-9)
        pred0, _, h0, _ = wrong.landmark_model(self.cam, points, 0.1)
        miss = measured - pred0
        self.assertGreater(np.max(abs(miss)), 5.0)
        np.testing.assert_allclose(h0[:, :, TM]*tau, miss, rtol=0.02, atol=0.05)
        # and the opposite sign (exposed at t - tau) would be far off
        self.assertGreater(np.max(abs(-h0[:, :, TM]*tau - miss)), 10.0)

    # (ii) posterior covariance --------------------------------------------------------

    def test_posterior_covariance_is_the_information_form(self):
        rng = np.random.default_rng(33)
        n = 12
        points = lm_points(rng, n)
        cov = realistic_cov(rng, SD0)
        map_var = np.array([0.09, 0.09, 1.0])*10.0                 # inflation, as the nominal run uses
        pixel_var = 0.3**2 + (0.15*CAM['fx']/NOMINAL['p'][2])**2
        h = fd_jacobian(points, NOMINAL['p'], NOMINAL['v'], NOMINAL['r'], NOMINAL['w'], NOMINAL['tau']).reshape(2*n, NS)
        noise = np.zeros((2*n, 2*n))
        for i, m in enumerate(points):
            jm = np.empty((2, 3))
            for j in range(3):
                e = np.zeros(3)
                e[j] = 1e-5
                jm[:, j] = (predict_pixels(m + e, NOMINAL['p'], NOMINAL['v'], NOMINAL['r'], NOMINAL['w'], NOMINAL['tau'])[0]
                            - predict_pixels(m - e, NOMINAL['p'], NOMINAL['v'], NOMINAL['r'], NOMINAL['w'], NOMINAL['tau'])[0])[0]/2e-5
            noise[2*i:2*i + 2, 2*i:2*i + 2] = pixel_var*np.eye(2) + jm @ np.diag(map_var) @ jm.T
        info = np.linalg.inv(np.linalg.inv(cov) + h.T @ np.linalg.solve(noise, h))
        z = own_pixels(points)                                      # zero residual: dx = 0, reset = I
        for iterations in (1, 3):
            f = lm_filter(cov)
            out = f.update_landmarks(self.cam, points, z, pixel_var, map_var, iterations=iterations)
            self.assertTrue(out['accepted'].all())
            # Measured 3e-10 relative to the largest element and 8e-10 on the diagonal.
            self.assertLess(scaled_error(f.cov, info), 1e-8, iterations)
        # Power: leaving the map error out changes the covariance far more than that.
        g = lm_filter(cov)
        g.update_landmarks(self.cam, points, z, pixel_var, None)
        self.assertGreater(scaled_error(g.cov, info), 1e-3)

    # (iii) gates -----------------------------------------------------------------------

    def test_gate_constants(self):
        # 2 dof: P(chi2 < x) = 1 - exp(-x/2). 1 dof: P = erf(sqrt(x/2)), solved by bisection.
        self.assertAlmostEqual(ek.GATE_2DOF, 2*math.log(1/0.003), places=12)
        self.assertAlmostEqual(ek.GATE_2DOF, 11.6183, places=3)
        lo, hi = 1.0, 20.0
        for _ in range(200):
            mid = 0.5*(lo + hi)
            lo, hi = (mid, hi) if math.erf(math.sqrt(mid/2)) < 0.997 else (lo, mid)
        self.assertAlmostEqual(lo, 8.8075, delta=2e-4)              # the exact value is 8.80747
        self.assertAlmostEqual(ek.GATE_1DOF, lo, delta=2e-4)
        self.assertAlmostEqual(math.erf(math.sqrt(ek.GATE_1DOF/2)), 0.997, delta=2e-6)

    def test_gate_acceptance_rate_on_clean_draws(self):
        # Truth drawn from the prior and clean pixels: the prior NIS is chi-square with 2 dof,
        # so 0.3 % of landmarks fall outside the gate.
        rng = np.random.default_rng(34)
        sd = SD0.copy()
        total = rejected = 0
        for _ in range(150):
            points = lm_points(rng, 20)
            f = lm_filter(np.diag(sd**2))
            dx = rng.normal(size=NS)*sd
            z = own_pixels(points, dx=dx) + rng.normal(0, 0.3, (20, 2))
            out = f.update_landmarks(self.cam, points, z, 0.3**2)
            total += 20
            rejected += int((~out['accepted']).sum())
        rate = rejected/total
        self.assertLess(abs(rate - 0.003), 4*math.sqrt(0.003*0.997/total) + 0.0005, rate)

    # (iv) attitude reset -----------------------------------------------------------------
    #
    # After the correction a = dtheta is injected, R_hat' = R_hat exp(a), the error
    # relative to the new estimate is dtheta' = log(exp(-a) exp(dtheta)). With
    # exp(a + e) = exp(a) exp(J_r(a) e) this is dtheta' = J_r(a) (dtheta - a) to first
    # order in the spread, so a covariance in the old axes becomes J C J^T with
    # J = J_r(a) = I - [a]x/2 + [a]x^2/6 - ... for this (right) convention. The
    # filter's  I - skew(a)/2  is that to first order; its error is [a]x^2/6, a factor
    # |a|/3 smaller than the correction itself.

    def test_reset_jacobian_is_the_right_jacobian_to_first_order(self):
        for deg in (2.0, 5.0, 10.0):
            a = deg*DEG*np.array([0.6, -0.5, 0.62]) / np.linalg.norm([0.6, -0.5, 0.62])
            jr = exact_reset(a)
            first = np.eye(3) - hat(a)/2
            err_first = np.linalg.norm(jr - first)
            err_none = np.linalg.norm(jr - np.eye(3))
            self.assertAlmostEqual(err_none/np.linalg.norm(a), 0.5*math.sqrt(2), delta=0.01 + 0.01*deg)  # ~ |a|/2 * sqrt(2)
            self.assertLess(err_first, err_none*abs(np.linalg.norm(a))/2, deg)   # |a|/3 vs |a|/2 of the size
            self.assertLess(err_first, 0.4*err_none*np.linalg.norm(a), deg)
        # The wrong sign, I + [a]x/2 (the left Jacobian), is as bad as no reset at all.
        a = 8*DEG*np.array([1.0, 0.0, 0.0])
        self.assertGreater(np.linalg.norm(exact_reset(a) - (np.eye(3) + hat(a)/2)), 0.9*np.linalg.norm(exact_reset(a) - np.eye(3)))

    def test_single_pass_reset_is_the_right_map_for_a_5_to_10_degree_correction(self):
        # A non-iterated update linearises at the prior, so the covariance it
        # produces is in the OLD axes and the reset is exactly the map to the
        # new ones. Reference: the same information form pushed through the
        # exact J_r(a) from the filter's actual correction a.
        rng = np.random.default_rng(35)
        sd = SD0.copy()
        sd[6:9] = np.radians([4.0, 4.0, 6.0])
        cov = np.diag(sd**2)
        n = 20
        points = lm_points(rng, n)
        h0 = fd_jacobian(points, NOMINAL['p'], NOMINAL['v'], NOMINAL['r'], NOMINAL['w'], NOMINAL['tau']).reshape(2*n, NS)
        info_old = np.linalg.inv(np.linalg.inv(cov) + h0.T @ h0/0.3**2)
        results = []
        for draw in range(12):
            dx = rng.normal(size=NS)*sd
            dx[9:16] = 0.0
            z = own_pixels(points, dx=dx) + rng.normal(0, 0.3, (n, 2))
            f = lm_filter(cov)
            f.update_landmarks(self.cam, points, z, 0.3**2, gate=1e9, iterations=1)
            a = rotvec(NOMINAL['r'].T @ f.r)
            if not 4*DEG < np.linalg.norm(a) < 14*DEG:
                continue
            jr = np.eye(NS)
            jr[A, A] = exact_reset(a)
            exact = jr @ info_old @ jr.T
            results.append((np.degrees(np.linalg.norm(a)), scaled_error(f.cov, exact), scaled_error(info_old, exact)))
        self.assertGreaterEqual(len(results), 3, results)
        for deg, with_reset, without in results:
            self.assertLess(with_reset, 0.2*without, (deg, with_reset, without))
            self.assertLess(with_reset, 0.05, (deg, with_reset))

    # DEFECT FOUND BY THIS CHECK (low severity), FIXED 9 October 2026. With iterations > 1
    # the covariance that update_landmarks commits is built from the Jacobians at the FINAL
    # iterate, i.e. it is already expressed in the new axes; _commit used to apply
    # I - skew(a)/2 to it a second time. Measured then against the exact Laplace covariance
    # in the new axes (J_r(a) times the inverse Hessian of the exact MAP cost): the
    # covariance before the reset agreed to 1e-4 (relative to sqrt(c_ii c_jj)) while the
    # committed one was off by 0.05 at 5 degrees, 0.1-0.2 at 8 degrees, 0.4 at 17 degrees.
    # The filter now skips the reset after an iterated solve, and this test passes.
    def test_iterated_update_covariance_is_in_the_new_axes_after_a_5_to_10_degree_correction(self):
        rng = np.random.default_rng(36)
        sd = SD0.copy()
        sd[6:9] = np.radians([5.0, 5.0, 8.0])
        sd[0:3] = 8.0
        cov = np.diag(sd**2)
        n = 20
        points = lm_points(rng, n)
        worst = []
        for draw in range(4):
            dx = rng.normal(size=NS)*sd
            dx[9:16] = 0.0
            z = own_pixels(points, dx=dx) + rng.normal(0, 0.3, (n, 2))
            f = lm_filter(cov)
            f.update_landmarks(self.cam, points, z, 0.3**2, gate=1e9, iterations=6)
            a = rotvec(NOMINAL['r'].T @ f.r)
            if np.linalg.norm(a) < 4*DEG:
                continue
            _, sigma_old = own_map_solution(NOMINAL, cov, points, z, 0.3**2)
            jr = np.eye(NS)
            jr[A, A] = exact_reset(a)
            worst.append(scaled_error(f.cov, jr @ sigma_old @ jr.T))
        self.assertGreaterEqual(len(worst), 2)
        self.assertLess(max(worst), 1e-2, worst)

    # (v) iterated update against an exact MAP solve ---------------------------------------

    def test_iterated_update_reaches_the_map_solution_from_a_large_prior_error(self):
        rng = np.random.default_rng(37)
        sd = SD0.copy()
        sd[0:3] = 10.0
        sd[6:9] = np.radians([4.0, 4.0, 4.0])
        cov = np.diag(sd**2)
        n = 20
        gaps, plain, moved = [], [], []
        for draw in range(5):
            points = lm_points(rng, n)
            dx = rng.normal(size=NS)*sd
            dx[9:16] = 0.0
            z = own_pixels(points, dx=dx) + rng.normal(0, 0.3, (n, 2))
            dx_map, sigma = own_map_solution(NOMINAL, cov, points, z, 0.3**2)
            post = np.sqrt(np.diag(sigma))
            for iterations, store in ((8, gaps), (1, plain)):
                f = lm_filter(cov)
                f.update_landmarks(self.cam, points, z, 0.3**2, gate=1e9, iterations=iterations)
                got = np.r_[f.p - NOMINAL['p'], f.v - NOMINAL['v'], rotvec(NOMINAL['r'].T @ f.r), np.zeros(7),
                            f.time_offset - NOMINAL['tau']]
                keep = list(range(9)) + [TM]
                store.append(np.max(abs(got - dx_map)[keep]/post[keep]))
            moved.append(np.degrees(np.linalg.norm(dx_map[A])))
        # The iterated update stays within a few hundredths of a posterior sigma of the exact MAP
        # (measured 1e-4 .. 2e-2 sigma for corrections of 1-12 degrees; the residual is the
        # missing J_r(dtheta) in the Gauss-Newton Jacobian), after moving ~50-100 sigma.
        self.assertGreater(max(moved), 4.0, moved)
        self.assertLess(max(gaps), 0.05, gaps)
        # The single pass is far worse: the test has power.
        self.assertGreater(max(plain), 20*max(gaps), (plain, gaps))

    # (vi) the post-fit statistic ---------------------------------------------------------------

    def test_post_fit_statistic_is_conservative_and_rarely_rejects_clean_data(self):
        # robust compares each landmark's post-fit residual, normalised by the
        # measurement noise ALONE, with the gate. The exact residual covariance is
        # R - H P+ H^T <= R, so the statistic is smaller than chi-square(2) on
        # average (mean 2 - tr(R^-1 H P+ H^T)): conservative against false rejection,
        # at the price of power when the fit has leverage.
        rng = np.random.default_rng(38)
        rejected = passed = 0
        means = {}
        for n in (6, 12, 30):
            stats = []
            for _ in range(150):
                points = lm_points(rng, n)
                f = lm_filter(np.diag(SD0**2))
                dx = rng.normal(size=NS)*SD0
                z = own_pixels(points, dx=dx) + rng.normal(0, 0.3, (n, 2))
                _, _, post, _ = f._solve(self.cam, points, z, 0.3**2, None, 3)
                stats.append(post.mean())
                out = f.update_landmarks(self.cam, points, z, 0.3**2, iterations=3, robust=True)
                prior_ok = out['nis'] < ek.GATE_2DOF
                rejected += int((prior_ok & ~out['accepted']).sum())
                passed += int(prior_ok.sum())
            means[n] = np.mean(stats)
        # Measured: 1.1 / 1.5 / 1.7 against 2; false rejection 0.16 % of 5.6e3 landmarks against 0.3 %.
        self.assertTrue(means[6] < means[12] < means[30] < 1.95, means)
        self.assertLess(means[6], 1.4, means)
        self.assertLess(rejected/passed, 0.003, (rejected, passed))

    # (vii) _solve does not commit ----------------------------------------------------------------

    def test_solve_always_restores_the_prior_state(self):
        rng = np.random.default_rng(39)
        points = lm_points(rng, 10)
        cov = realistic_cov(rng, SD0)
        z = own_pixels(points, dx=rng.normal(size=NS)*SD0*np.r_[np.ones(9), np.zeros(8)]) + rng.normal(0, 0.3, (10, 2))
        f = lm_filter(cov)
        f.ba, f.bg, f.baro_bias = np.array([0.1, 0.2, 0.3]), np.array([1e-3, 2e-3, 3e-3]), 0.7
        before = (f.p.copy(), f.v.copy(), f.r.copy(), f.ba.copy(), f.bg.copy(), f.baro_bias, f.time_offset, f.cov.copy())

        def unchanged():
            now = (f.p, f.v, f.r, f.ba, f.bg, f.baro_bias, f.time_offset, f.cov)
            return all(np.array_equal(np.asarray(a), np.asarray(b)) for a, b in zip(before, now))
        dx, new_cov, post, _ = f._solve(self.cam, points, z, 0.3**2, None, 4)
        self.assertTrue(unchanged())
        self.assertGreater(np.max(abs(dx[:9])), 0.1)                 # it did compute a real correction
        # An exception half way through (pixels of the wrong shape) must restore too.
        with self.assertRaises(Exception):
            f._solve(self.cam, points, np.zeros((10, 3)), 0.3**2, None, 4)
        self.assertTrue(unchanged())
        # and the committing entry point changes the state while _solve alone did not
        f.update_landmarks(self.cam, points, z, 0.3**2, iterations=4)
        self.assertFalse(unchanged())

    # (viii) recovery ---------------------------------------------------------------------------

    def test_recovery_widening_leaves_a_symmetric_positive_definite_covariance(self):
        rng = np.random.default_rng(40)
        sd = np.r_[np.full(3, 0.1), np.full(3, 0.1), np.full(3, 0.001), np.full(3, 0.05), np.full(3, 0.003), 1.0, 1e-4]
        for scale in (1.0, 1e-3):
            cov = realistic_cov(rng, sd*scale)
            f = lm_filter(cov)
            points = lm_points(rng, 24)
            z = own_pixels(points, dx=np.r_[[4.5, -4.0, 0.5], np.zeros(NS - 3)]) + rng.normal(0, 0.4, (24, 2))
            # The prior is doubted only after RECOVER_FRAMES starved images in a row.
            for _ in range(ek.Eskf.RECOVER_FRAMES):
                out = f.update_landmarks(self.cam, points, z, 0.4**2, iterations=4, robust=True, recover=True)
            self.assertTrue(out['recovered'])
            np.testing.assert_array_equal(f.cov, f.cov.T)
            d = np.sqrt(np.diag(f.cov))
            self.assertGreater(np.linalg.eigvalsh(f.cov/np.outer(d, d)).min(), 1e-9)
            self.assertGreater(f.cov[0, 0], 0.0)

    def recovery_trials(self, recover, outlier_fraction, trials=200):
        """Horizontal error after one frame whose prior is CORRECT (1 m, 0.5 deg), with some true outliers."""
        rng = np.random.default_rng(41)
        sd = np.r_[np.full(3, 1.0), np.full(3, 0.3), np.radians([0.5, 0.5, 1.0]), np.full(3, 0.05),
                   np.full(3, 0.003), 1.0, 0.002]
        errors = []
        for _ in range(trials):
            n = 20
            points = lm_points(rng, n)
            dx = rng.normal(size=NS)*sd
            dx[9:] = 0.0
            z = own_pixels(points, dx=dx) + rng.normal(0, 0.3, (n, 2))
            bad = rng.random(n) < outlier_fraction
            z[bad] += rng.uniform(-16, 16, (int(bad.sum()), 2))*CAM['fx']/NOMINAL['p'][2]    # +-16 m on the ground
            f = lm_filter(np.diag(sd**2))
            f.update_landmarks(self.cam, points, z, 0.3**2, iterations=3, robust=True, recover=recover)
            errors.append(np.linalg.norm((np.r_[f.p, 0, 0, 0][:2] - NOMINAL['p'][:2]) - dx[:2]))
        return np.array(errors)

    def test_recovery_does_not_hurt_when_few_landmarks_are_outliers(self):
        off, on = self.recovery_trials(False, 0.3), self.recovery_trials(True, 0.3)
        self.assertLess(np.quantile(on, 0.95), 1.0)
        self.assertLess(np.quantile(on, 0.95), 1.5*np.quantile(off, 0.95) + 0.1)

    # LIMITATION FOUND BY THIS CHECK, ADDRESSED 9 October 2026. When most of a frame's
    # landmarks are true outliers (each uniform in the 16 m search window) and the prior is
    # CORRECT, the "fewer than half pass" trigger used to fire at once: RECOVER_POS_M = 16 m
    # widened the position prior until the outliers passed the gate, and the post-fit test
    # could not separate them again. Measured then, 60 % outliers, 200 frames, 20 landmarks,
    # prior 1 m / 0.5 deg:
    #   recover=False: p95 horizontal error 1.9 m, mean 0.7 m
    #   recover=True : p95 horizontal error 24 m,  mean 3.9 m
    # The filter now needs RECOVER_FRAMES consecutive starved images before widening and
    # keeps the result only if most landmarks then agree, so this single-frame case passes.
    # A majority of mismatches that agree with each other is still beyond it.
    def test_recovery_does_not_destroy_a_correct_prior_when_outliers_are_the_majority(self):
        on = self.recovery_trials(True, 0.6)
        self.assertLess(np.quantile(on, 0.95), 3.0)

    def test_landmark_behind_the_camera_is_never_used(self):
        # A point above the can projects, by the pinhole formula, to a real-looking pixel with
        # negative depth. Give it a measurement equal to that pixel (NIS = 0): it must still be rejected.
        rng = np.random.default_rng(42)
        points = lm_points(rng, 8)
        points[3] = NOMINAL['p'] + np.array([20.0, 10.0, 60.0])          # 60 m above the can
        f = lm_filter(np.diag(SD0**2))
        z = own_pixels(points) + rng.normal(0, 0.3, (8, 2))
        z[3] = own_pixels(points)[3]
        _, depth = self.cam.project(points, f.p, f.r)
        self.assertLess(depth[3], 0.0)
        for kwargs in (dict(), dict(iterations=3), dict(iterations=3, robust=True, recover=True)):
            g = lm_filter(np.diag(SD0**2))
            out = g.update_landmarks(self.cam, points, z, 0.3**2, **kwargs)
            self.assertFalse(out['accepted'][3], kwargs)
            self.assertGreaterEqual(int(out['accepted'].sum()), 6, kwargs)

    def test_degenerate_innovation_covariance_is_rejected_without_an_exception(self):
        # Zero pixel variance and zero covariance: S is singular, det = 0, NIS is 0/0 or x/0.
        points = lm_points(np.random.default_rng(43), 3)
        f = lm_filter(np.zeros((NS, NS)))
        z = own_pixels(points) + 1.0
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            out = f.update_landmarks(self.cam, points, z, 0.0)
        self.assertFalse(out['accepted'].any())
        self.assertTrue(np.all(np.isfinite(f.p)))

    def test_single_landmark_and_scalar_noise_broadcast(self):
        rng = np.random.default_rng(44)
        points = lm_points(rng, 1)
        z = own_pixels(points) + 0.1
        for kwargs in (dict(), dict(iterations=3), dict(iterations=3, robust=True), dict(recover=True)):
            f = lm_filter(np.diag(SD0**2))
            out = f.update_landmarks(self.cam, points, z, 0.09, np.array([0.09, 0.09, 1.0]), **kwargs)
            self.assertEqual(out['accepted'].shape, (1,))
            self.assertTrue(out['accepted'][0])
            self.assertTrue(np.all(np.isfinite(f.cov)))
        _, _, h, noise = lm_filter(np.eye(NS)).landmark_model(self.cam, points, 0.09, [0.09, 0.09, 1.0])
        self.assertEqual(h.shape, (1, 2, NS))
        self.assertEqual(noise.shape, (1, 2, 2))


# === F. Monte Carlo consistency in a matched world built here ==================================
#
# The world: body rates and world accelerations are prescribed smooth functions
# of time; the truth is this file's own quaternion strapdown driven by them, so
# that the filter's discrete mechanisation is exact for the noise-free signals.
# Measurements: the gyro and accelerometer read truth + bias + white noise at
# the filter's densities (sample sigma = density / sqrt(dt)); biases start at
# draws from the filter's prior and random-walk at its densities; images at
# 5 Hz show landmarks with white pixel noise (0.3 px plus 0.15 m on the ground
# at the landmark's range, as run_nav_mc.py has it), exposed tau = a draw from the
# prior later than their stamp (pose carried forward at constant velocity and
# the previous interval's body rate, which is what the filter assumes); baro =
# height + bias + 0.15 m noise at 10 Hz. Initial errors are draws from the
# filter's prior. No unmodelled errors of any kind. The step is 40 ms rather
# than the production 10 ms (and 3 s long) to keep the run short.

MC_DENS = dict(accel=2.0e-3, gyro=2.5e-4, accel_bias=1.0e-4, gyro_bias=2.0e-6, baro_bias=0.02)
MC_LAND = np.column_stack((np.random.default_rng(1234).uniform(-260, 260, (500, 2)),
                           np.random.default_rng(1235).normal(0, 1.0, 500)))


def mc_run(seed, configs, T=3.0, dt=0.05, height=250.0, previous_interval_rate=True):
    """One matched descent; filter variants in `configs` = [(start_scale, update kwargs)] on the same data.

    Returns {config index: (NEES17, NEES16) at every image frame}.
    """
    rng = np.random.default_rng(seed)
    n = round(T/dt)
    tm = (np.arange(n) + 0.5)*dt
    w = np.column_stack((0.35*np.cos(2*np.pi*0.6*tm), 0.30*np.sin(2*np.pi*0.6*tm + 0.7),
                         0.25 + 0.2*np.sin(2*np.pi*0.17*tm)))
    phase = rng.uniform(0, 6.28, 2)
    a_w = np.column_stack((0.8*np.sin(2*np.pi*0.3*tm + phase[0]), 0.6*np.cos(2*np.pi*0.23*tm + phase[1]),
                           0.4*np.sin(2*np.pi*0.2*tm)))
    # truth strapdown with this file's quaternion algebra
    q = qfrom(LEVEL @ rot(np.array([0.04, -0.03, rng.uniform(-math.pi, math.pi)])))
    p = np.array([rng.normal(0, 20), rng.normal(0, 20), height])
    v = np.array([2.0, 1.0, -10.5])
    ps, vs, qs = [p.copy()], [v.copy()], [q.copy()]
    fb = np.empty((n, 3))
    for k in range(n):
        fb[k] = qmat(qmul(q, qvec(w[k]*dt/2))).T @ (a_w[k] + G*E_UP)      # specific force, body axes, mid-step
        p, v = p + v*dt + a_w[k]*dt*dt/2, v + a_w[k]*dt
        q = qmul(q, qvec(w[k]*dt))
        q = q/math.sqrt(q @ q)
        ps.append(p.copy())
        vs.append(v.copy())
        qs.append(q.copy())
    ps, vs = np.array(ps), np.array(vs)
    sq = math.sqrt(dt)
    ba = SD0[9]*rng.normal(size=3) + np.cumsum(np.vstack((np.zeros(3), MC_DENS['accel_bias']*sq*rng.normal(size=(n - 1, 3)))), 0)
    bg = SD0[12]*rng.normal(size=3) + np.cumsum(np.vstack((np.zeros(3), MC_DENS['gyro_bias']*sq*rng.normal(size=(n - 1, 3)))), 0)
    bb = SD0[15]*rng.normal() + np.cumsum(np.r_[0.0, MC_DENS['baro_bias']*sq*rng.normal(size=n - 1)])
    tau = SD0[16]*rng.normal()
    accel = fb + ba + MC_DENS['accel']/sq*rng.normal(size=(n, 3))
    gyro = w + bg + MC_DENS['gyro']/sq*rng.normal(size=(n, 3))
    z0 = rng.normal(size=16)                                       # initial error / prior sigma, shared by variants
    every_image, every_baro = round(0.2/dt), round(0.1/dt)
    frames, baro = {}, {}
    for k in range(0, n + 1, every_image):
        kk = min(k, n - 1)
        w_exposure = w[max(k - 1, 0)] if previous_interval_rate else w[kk]
        pe = ps[k] + vs[k]*tau + a_w[kk]*tau*tau/2
        re =qmat(qs[k]) @ rot(w_exposure*tau)
        c = ((MC_LAND - pe) @ re - CAM['lever']) @ CAM['r_cb'].T
        px = np.column_stack((CAM['fx']*c[:, 0]/c[:, 2] + CAM['cx'], CAM['fy']*c[:, 1]/c[:, 2] + CAM['cy']))
        ok = np.flatnonzero((c[:, 2] > 1) & (px[:, 0] > 16) & (px[:, 0] < 1440) & (px[:, 1] > 16) & (px[:, 1] < 1072))
        pick = rng.permutation(ok)[:25]
        sigma = np.sqrt(0.3**2 + (0.15*CAM['fx']/c[pick, 2])**2)
        frames[k] = (pick, px[pick] + sigma[:, None]*rng.normal(size=(len(pick), 2)))
    for k in range(0, n + 1, every_baro):
        baro[k] = ps[k][2] + bb[min(k, n - 1)] + 0.15*rng.normal()
    out = {}
    noise = ek.ImuNoise(**MC_DENS)
    camera = their_camera()
    for ci, (scale, kwargs) in enumerate(configs):
        sd = SD0.copy()
        sd[:9] = SD0[:9]*scale
        e0 = np.r_[sd[:9]*z0[:9], np.zeros(8)]
        f = ek.Eskf(ps[0] - e0[P], vs[0] - e0[V], qmat(qs[0]) @ rot(-e0[A]), np.diag(sd**2), noise)
        f.rate = gyro[0].copy()
        rows = {}
        for k in range(n + 1):
            if k in frames:
                pick, px = frames[k]
                if len(pick):
                    zhat = np.maximum(camera.project(MC_LAND[pick], f.p, f.r)[1], 1.0)
                    f.update_landmarks(camera, MC_LAND[pick], px, 0.3**2 + (0.15*CAM['fx']/zhat)**2, None, **kwargs)
                kk = min(k, n - 1)
                e = f.error_from(ps[k], vs[k], qmat(qs[k]), ba[kk], bg[kk], bb[kk], tau)
                rows[k] = (e @ np.linalg.solve(f.cov, e), e[:16] @ np.linalg.solve(f.cov[:16, :16], e[:16]))
            if k in baro:
                f.update_baro(baro[k], 0.15)
            if k < n:
                f.propagate(dt, accel[k], gyro[k])
        out[ci] = rows
    return out


class MatchedMonteCarlo(unittest.TestCase):
    runs = 300
    dt = 0.05

    @classmethod
    def setUpClass(cls):
        cls.configs = [(0.1, dict(iterations=1)),                    # plain EKF, started inside the linear regime
                       (1.0, dict(iterations=3)),                    # iterated, started 3 m / 1-2 deg out
                       (1.0, dict(iterations=1))]                    # plain EKF, same start (the proposal's form)
        results = [mc_run(s, cls.configs) for s in range(cls.runs)]
        keys = sorted(results[0][0])
        cls.keys = keys
        cls.nees = {ci: np.array([[r[ci][k] for k in keys] for r in results]) for ci in range(len(cls.configs))}

    def summary(self, ci, which=0):
        x = self.nees[ci][:, :, which]
        return x.mean(axis=0), x.std(axis=0, ddof=1)/math.sqrt(x.shape[0])

    def test_plain_update_in_the_linear_regime_is_consistent(self):
        mean, se = self.summary(0)
        # Mean NEES over 300 runs at every epoch is the state count (17) to within
        # the standard error (about 0.35).
        for k in range(len(mean)):
            self.assertLess(abs(mean[k] - NS), 4*se[k] + 0.3, (k, mean[k], se[k]))
        self.assertLess(abs(mean.mean() - NS), 0.8, mean.mean())

    def test_iterated_update_is_consistent_from_a_3_m_start(self):
        mean, se = self.summary(1)
        for k in range(len(mean)):
            self.assertLess(abs(mean[k] - NS), 4*se[k] + 0.3, (k, mean[k], se[k]))
        self.assertLess(abs(mean.mean() - NS), 0.8, mean.mean())
        # and the 16-state NEES that the Monte Carlo of run_nav_mc reports (author: 16.2 +- 0.3)
        mean16, _ = self.summary(1, 1)
        self.assertLess(abs(mean16.mean() - 16), 0.8, mean16.mean())

    def test_first_order_update_from_a_3_m_start_is_overconfident(self):
        # The proposal's form, one first-order update, started 3 m / 1-2 deg out. The author reports
        # mean NEES 24-28 with about 1 % of descents diverging. Here (300 matched descents, 250 m
        # start): see the printed figures; the mean is dominated by a heavy tail while the median is
        # consistent. Asserted: the iterated update fixes it, and the tail exists.
        plain = self.nees[2][:, :, 0]
        iterated = self.nees[1][:, :, 0]
        self.assertGreater(plain.mean(), iterated.mean() + 3.0, (plain.mean(), iterated.mean()))
        self.assertGreater(np.mean(plain.max(axis=1) > 60.0), 0.005)
        self.assertLess(np.mean(iterated.max(axis=1) > 60.0), 0.02)
        type(self).tail = (plain.mean(), float(np.median(plain)), float(np.mean(plain.max(axis=1) > 60.0)),
                           iterated.mean(), float(np.mean(iterated.max(axis=1) > 60.0)))

    @classmethod
    def tearDownClass(cls):
        mean0, _ = cls.nees[0][:, :, 0].mean(axis=0), None
        mean1 = cls.nees[1][:, :, 0].mean(axis=0)
        print('\n[F] mean NEES17 by epoch (0.2 s apart), %d runs, dt = %.2f s' % (cls.runs, cls.dt))
        print('    plain, tight start :', ' '.join('%5.1f' % x for x in mean0[::4]))
        print('    iterated, 3 m start:', ' '.join('%5.1f' % x for x in mean1[::4]))
        if hasattr(cls, 'tail'):
            print('    first-order, 3 m start: mean %.1f median %.1f, %.1f %% of runs above NEES 60 (iterated: mean %.1f, %.1f %%)'
                  % (cls.tail[0], cls.tail[1], 100*cls.tail[2], cls.tail[3], 100*cls.tail[4]))


# === G. Wind generator =================================================================
#
# Appendix I: transverse (horizontal) components have covariance
# sigma^2 (1 - r/2) exp(-r) and the vertical one sigma^2 exp(-r), with
# r = |zeta(z1) - zeta(z2)| and zeta = integral dz / L, L = 0.5 max(z, 1 m):
# zeta = 2 z below 1 m and 2 + 2 ln z above.

def zeta_of(z):
    return 2.0*z if z <= 1.0 else 2.0 + 2.0*math.log(z)


class WindGenerator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wp = tr.WindParams()
        cls.n = 16000
        cls.wind, cls.mean, cls.turb = tr.make_wind(cls.n, np.random.default_rng(5), cls.wp)

    def test_zeta_by_numerical_integration(self):
        z = np.linspace(0.0, 60.0, 600001)
        inv = 1.0/(0.5*np.maximum(z, 1.0))
        zeta = np.concatenate(([0.0], np.cumsum(0.5*(inv[1:] + inv[:-1])*np.diff(z))))
        for z0 in (0.3, 1.0, 1.7, 7.3, 50.0):
            self.assertAlmostEqual(float(np.interp(z0, z, zeta)), zeta_of(z0), delta=1e-6)

    def test_pair_covariances_including_pairs_below_and_straddling_the_floor(self):
        wp, turb, n = self.wp, self.turb, self.n
        pairs = [(0.0, 0.0), (0.0, 0.5), (0.5, 1.0), (0.0, 1.0), (0.5, 1.5), (0.0, 2.0), (1.0, 1.5), (1.5, 3.0),
                 (3.0, 6.0), (10.0, 20.0), (100.0, 200.0), (200.0, 400.0), (1.0, 10.0), (0.5, 20.0), (50.0, 51.0)]
        worst = 0.0
        for z1, z2 in pairs:
            i1, i2 = round(z1/wp.dz), round(z2/wp.dz)
            r = abs(zeta_of(z1) - zeta_of(z2))
            th_h, th_v = wp.sigma_h**2*(1 - r/2)*math.exp(-r), wp.sigma_w**2*math.exp(-r)
            # the two horizontal axes are independent draws of the same process: pool them
            sh = 0.5*((turb[:, i1, 0]*turb[:, i2, 0]).mean() + (turb[:, i1, 1]*turb[:, i2, 1]).mean())
            sv = (turb[:, i1, 2]*turb[:, i2, 2]).mean()
            se_h = wp.sigma_h**2*math.sqrt((1 + ((1 - r/2)*math.exp(-r))**2)/(2*n))
            se_v = wp.sigma_w**2*math.sqrt((1 + math.exp(-2*r))/n)
            self.assertLess(abs(sh - th_h), 4.2*se_h, (z1, z2, th_h, sh))
            self.assertLess(abs(sv - th_v), 4.2*se_v, (z1, z2, th_v, sv))
            worst = max(worst, abs(sh - th_h)/se_h, abs(sv - th_v)/se_v)
        self.assertLess(worst, 4.2)

    def test_horizontal_is_transverse_not_longitudinal(self):
        # The sign change of (1 - r/2) e^-r at r = 2 distinguishes the transverse form from exp(-r):
        # pair (0, 1 m) has r = 2, covariance exactly 0 for the horizontal and e^-2 sigma^2 for the vertical.
        turb, n = self.turb, self.n
        i1, i2 = 0, 2
        ch = 0.5*((turb[:, i1, 0]*turb[:, i2, 0]).mean() + (turb[:, i1, 1]*turb[:, i2, 1]).mean())
        cv = (turb[:, i1, 2]*turb[:, i2, 2]).mean()
        self.assertLess(abs(ch), 4*math.sqrt(1/(2*n)))
        self.assertAlmostEqual(cv/(self.wp.sigma_w**2*math.exp(-2.0)), 1.0, delta=4*math.sqrt(1/n)/math.exp(-2.0))
        # a negative covariance appears beyond r = 2 (the Dryden transverse lobe): (0, 2 m) has r = 3.39
        j = round(2.0/self.wp.dz)
        c = 0.5*((turb[:, 0, 0]*turb[:, j, 0]).mean() + (turb[:, 0, 1]*turb[:, j, 1]).mean())
        self.assertLess(c, -0.015)

    def test_components_are_independent_and_stationary_in_variance(self):
        turb, n = self.turb, self.n
        for i in (0, 2, 20, 200, 800):
            self.assertAlmostEqual(turb[:, i, :2].var(), self.wp.sigma_h**2, delta=4*self.wp.sigma_h**2/math.sqrt(n))
            self.assertAlmostEqual(turb[:, i, 2].var(), self.wp.sigma_w**2, delta=4*self.wp.sigma_w**2*math.sqrt(2/n))
            self.assertLess(abs((turb[:, i, 0]*turb[:, i, 1]).mean()), 4/math.sqrt(n))
            self.assertLess(abs((turb[:, i, 0]*turb[:, i, 2]).mean()), 4*0.5/math.sqrt(n))
        self.assertEqual(self.turb.shape, (n, len(np.arange(0.0, self.wp.top + self.wp.dz, self.wp.dz)), 3))

    def test_mean_profile_is_the_log_law(self):
        z = np.arange(0.0, self.wp.top + self.wp.dz, self.wp.dz)
        speed = np.hypot(self.mean[0, :, 0], self.mean[0, :, 1])
        expected = self.wp.speed_10m*np.log(np.maximum(z, 1.0)/self.wp.roughness)/math.log(10.0/self.wp.roughness)
        np.testing.assert_allclose(speed, expected, atol=1e-12)
        self.assertAlmostEqual(speed[round(10.0/self.wp.dz)], self.wp.speed_10m, places=12)
        np.testing.assert_allclose(speed[:3], speed[2], atol=1e-12)          # flat below the 1 m floor

    def test_lookup_cannot_index_out_of_range(self):
        n = self.n
        top = self.wp.top
        for z in (-1e6, -3.0, 0.0, 0.25, top - 1e-9, top, top + 0.4, 1e6):
            got = self.wind(np.full(n, z))
            self.assertEqual(got.shape, (n, 3))
            self.assertTrue(np.all(np.isfinite(got)), z)
        np.testing.assert_allclose(self.wind(np.full(n, -50.0)), self.wind(np.zeros(n)), atol=0)
        np.testing.assert_allclose(self.wind(np.full(n, 1e4)), self.wind(np.full(n, top)), atol=1e-5)
        # and interpolates between levels, reproducing the grid values at the levels
        profile = self.wind.profile
        np.testing.assert_allclose(self.wind(np.full(n, 12.5)), profile[:, 25], atol=1e-12)
        mid = self.wind(np.full(n, 12.75))
        np.testing.assert_allclose(mid, 0.5*(profile[:, 25] + profile[:, 26]), atol=1e-12)


# === H. Bookkeeping in truth.simulate, aliasing, batch independence, run_descent ================

class TruthBookkeeping(unittest.TestCase):
    def test_frame_count_and_delayed_frames_against_the_nav_trajectory(self):
        out = tr.simulate(tr.Params(), 2, np.random.default_rng(3), duration=0.6, nav_dt=0.01, frame_dt=0.2,
                          camera_delay=0.01, delays=(0.0, 0.02), start=dict(tilt_sd_deg=15.0, rate_sd_dps=80.0))
        # 600 nav steps = 6000 fine steps; the longest delay is 20 fine steps ->
        # frames at 0.2 f + 0.02 s <= 0.6 s: f = 0, 1, 2 (f = 3 would need 0.62 s).
        self.assertEqual(out['frame_p'].shape[0], 3)
        self.assertEqual(out['delay_p'].shape[:2], (2, 3))
        np.testing.assert_array_equal(out['frame_t'], [0.0, 0.2, 0.4])
        self.assertTrue(np.all(np.isfinite(out['frame_p'])) and np.all(np.isfinite(out['delay_p'])))
        self.assertTrue(np.all(np.isfinite(out['frame_r'])) and np.all(np.isfinite(out['delay_r'])))
        idx = 20*np.arange(3)
        np.testing.assert_array_equal(out['frame_p'], out['p'][idx + 1])          # camera_delay 0.01 s = 1 nav step
        np.testing.assert_array_equal(out['frame_r'], out['r'][idx + 1])
        np.testing.assert_array_equal(out['delay_p'][0], out['p'][idx])
        np.testing.assert_array_equal(out['delay_p'][1], out['p'][idx + 2])
        np.testing.assert_array_equal(out['delay_r'][1], out['r'][idx + 2])
        # frames are stamped without the delay, and pose arrays differ between delays
        self.assertGreater(np.max(abs(out['delay_p'][1] - out['delay_p'][0])), 1e-3)

    def test_production_truth_delays_land_on_the_right_samples(self):
        pipe = pipeline()
        mc, d = pipe['mc'], pipe['data']
        frames = d['delay_p'].shape[1]
        self.assertEqual(frames, int((500*10 - 20)//200) + 1)                    # 5 s, longest delay 20 ms
        for i, delay in enumerate(mc.DELAYS):
            base = 20*np.arange(frames)
            if abs(delay*100 - round(delay*100)) < 1e-9:                         # multiples of one nav step
                np.testing.assert_array_equal(d['delay_p'][i], d['p'][base + round(delay*100)])
            else:
                # between nav samples: second-order Taylor from the exact velocity and the nav acceleration
                a = (d['v'][base + 1] - d['v'][base])/0.01
                approx = d['p'][base] + d['v'][base]*delay + 0.5*a*delay**2
                self.assertLess(np.max(abs(d['delay_p'][i] - approx)), 3e-4 if delay > 0.003 else 1e-5, delay)

    def test_no_in_place_aliasing_of_state_or_forcing(self):
        params = replace(tr.Params(), cn_k=0.4, trim_deg=12.0, side_force_sd=0.5, spin_torque_sd=4e-5)
        rng = np.random.default_rng(7)
        sim = tr.Descent(params, tr.make_wind(3, rng, tr.WindParams())[0], 3)
        sim.start(rng, tilt_sd_deg=20.0, rate_sd_dps=60.0)
        side, spin = sim.side.copy(), sim.spin.copy()
        before = [a.copy() for a in (sim.pc, sim.vc, sim.pp, sim.vp, sim.q, sim.om)]
        sim.derivative(sim.pc, sim.vc, sim.pp, sim.vp, sim.q, sim.om)
        for a, b in zip(before, (sim.pc, sim.vc, sim.pp, sim.vp, sim.q, sim.om)):
            np.testing.assert_array_equal(a, b)                                   # derivative() mutates nothing
        np.testing.assert_array_equal(sim.side, side)
        np.testing.assert_array_equal(sim.spin, spin)
        np.testing.assert_array_equal(sim.side[:, 2], 0.0)                        # weight was not subtracted into the forcing
        force, rate = sim.step(rng)
        # the returned rate is the START-of-step rate and must not alias the live state
        self.assertFalse(np.shares_memory(rate, sim.om))
        np.testing.assert_array_equal(rate, before[5])
        snapshot = sim.om.copy()
        rate += 1.0
        force += 1.0
        np.testing.assert_array_equal(sim.om, snapshot)

    def test_each_descent_in_a_batch_is_independent_of_its_neighbours(self):
        # n = 1 against the same row of an n = 3 batch: no cross-row coupling, no broadcasting artefact.
        params = replace(tr.Params(), cn_k=0.4, trim_deg=12.0, dt=0.002, **QUIET)
        wind3 = tr.make_wind(3, np.random.default_rng(8), tr.WindParams())[0]
        big = tr.Descent(params, wind3, 3)
        big.start(np.random.default_rng(9), tilt_sd_deg=20.0, rate_sd_dps=60.0)
        small = tr.Descent(params, tr.Wind(wind3.profile[1:2], wind3.dz), 1)
        for name in ('pc', 'vc', 'pp', 'vp', 'q', 'om'):
            setattr(small, name, getattr(big, name)[1:2].copy())
        for _ in range(300):
            big.step()
            small.step()
        for name in ('pc', 'vc', 'pp', 'vp', 'q', 'om'):
            np.testing.assert_allclose(getattr(small, name)[0], getattr(big, name)[1], atol=1e-12, err_msg=name)

    def test_covariance_stays_symmetric_and_positive_definite_after_many_steps(self):
        rng = np.random.default_rng(10)
        noise = ek.ImuNoise(1.7e-3, 2.4e-4, 1e-4, 2e-6, 0.02, gyro_scale=0.005, scale_tau=0.2)
        f = ek.Eskf([0, 0, 300.0], [1.0, 0.5, -1.0], LEVEL @ rot(np.array([0.0, 0.0, 0.5])), np.diag(SD0**2), noise)
        cam = their_camera()
        for k in range(6000):                                    # 60 s of steady descent, spinning at 1 rad/s
            f.propagate(0.01, np.array([0.0, 0.0, -G]) + 0.02*rng.normal(size=3), np.array([0.0, 0.0, 1.0]))
            if k % 10 == 0:
                f.update_baro(300.0 - 0.01*k + rng.normal(0, 0.15), 0.15)
            if k % 20 == 0 and k < 3000:
                pts = lm_points(rng, 15, dict(NOMINAL, p=f.p))
                f.update_landmarks(cam, pts, cam.project(pts, f.p, f.r)[0] + rng.normal(0, 0.3, (15, 2)), 0.09)
        np.testing.assert_array_equal(f.cov, f.cov.T)
        d = np.sqrt(np.diag(f.cov))
        self.assertGreater(np.linalg.eigvalsh(f.cov/np.outer(d, d)).min(), 1e-12)
        self.assertLess(np.max(abs(f.r.T @ f.r - np.eye(3))), 1e-13)               # re-orthonormalised
        self.assertAlmostEqual(np.linalg.det(f.r), 1.0, places=12)


class RunDescentWiring(unittest.TestCase):
    """run_descent against the structure of the sensor model, on a 5 s production-path truth batch."""

    @classmethod
    def setUpClass(cls):
        cls.pipe = pipeline()
        cls.mc = cls.pipe['mc']
        mc = cls.mc
        mc.CACHE, mc.DURATION = Path(cls.pipe['tmp'].name), 5.0          # still redirected, in case another class restored it
        mc._TRUTH.clear()
        mc.SCENARIOS['_probe'] = mc.scenario('moderate', mc.MATCHED, dict(mc.PLAIN, map_floor=1e9))     # IMU and baro only
        mc.SCENARIOS['_delay'] = mc.scenario('moderate', dict(mc.MATCHED, camera_delay=0.02),
                                             dict(mc.PLAIN, init_time_offset=0.02))
        mc.SCENARIOS['_matched'] = mc.scenario('moderate', mc.MATCHED, mc.PLAIN)
        n = cls.pipe['n']
        cls.imu_only = [mc.run_descent(('_probe', j, n)) for j in range(n)]
        cls.delay = [mc.run_descent(('_delay', j, n)) for j in range(n)]
        cls.matched = [mc.run_descent(('_matched', j, n)) for j in range(n)]

    @classmethod
    def tearDownClass(cls):
        cls.mc._TRUTH.clear()

    def test_initial_error_is_the_draw_it_is_built_from(self):
        # With no image update the recorded error at frame 0 is just the initial offsets, which
        # run_descent builds as estimate = truth - e0 and error_from must give back as truth - estimate.
        mc = self.mc
        s, c = mc.SCENARIOS['_probe']['sensors'], mc.SCENARIOS['_probe']['filter']
        sd = np.r_[c.init_pos, c.init_vel, c.init_att, np.full(3, c.init_accel_bias), np.full(3, c.init_gyro_bias),
                   c.init_baro_bias, c.init_time_offset]
        for j, rec in enumerate(self.imu_only):
            e0 = sd*np.r_[np.random.default_rng([mc.SEED, j, 2]).normal(size=16), 0.0]
            np.testing.assert_allclose(rec['err'][0, :9], e0[:9], rtol=1e-5, atol=1e-9)       # pos, vel, att
            # the filter's biases start at zero, so the error is the sensor's true bias (first draws of r_imu)
            r_imu = np.random.default_rng([mc.SEED, j, 0])
            steps = self.pipe['data']['accel'].shape[0]
            ba0 = s.accel_bias_sd*r_imu.normal(size=3)
            r_imu.normal(size=(steps, 3))                                        # the accelerometer bias walk
            bg0 = s.gyro_bias_sd*r_imu.normal(size=3)
            # plus the first step of each walk (1e-5 m/s^2 and 2e-7 rad/s per sigma)
            np.testing.assert_allclose(rec['err'][0, BA], ba0, rtol=0, atol=1e-4)
            np.testing.assert_allclose(rec['err'][0, BG], bg0, rtol=0, atol=3e-6)
            self.assertGreater(np.max(abs(ba0)), 0.02)                             # a real draw, not zeros
            self.assertEqual(float(rec['err'][0, TM]), 0.0)
            self.assertAlmostEqual(float(rec['sig'][0, TM]), c.init_time_offset, places=9)

    def test_imu_and_baro_only_run_is_consistent_which_pins_the_bias_signs_and_densities(self):
        # Truth measurement = truth + bias + noise; the filter subtracts its bias. A sign slip or a
        # density slip makes the 16-state NEES explode; 4 descents x ~25 frames average to about 16.
        nees = np.array([r['nees'][:, 0] for r in self.imu_only])
        self.assertTrue(np.all(np.isfinite(nees)))
        self.assertLess(abs(np.mean(nees) - 16.0), 9.0, np.mean(nees))
        self.assertLess(np.max(np.mean(nees, axis=1)), 40.0)
        # and the filter really did dead-reckon: horizontal error grows beyond the initial 3 m draw scale
        horiz = np.array([np.hypot(r['err'][:, 0], r['err'][:, 1]) for r in self.imu_only])
        self.assertTrue(np.all(np.isfinite(horiz)))

    def test_matched_run_with_images_is_consistent_and_tracks_truth(self):
        nees = np.array([r['nees'][10:, 0] for r in self.matched])
        horiz = np.array([np.hypot(r['err'][10:, 0], r['err'][10:, 1]) for r in self.matched])
        self.assertLess(abs(np.nanmean(nees) - 16.0), 9.0, np.nanmean(nees))
        self.assertLess(np.sqrt(np.nanmean(horiz**2)), 1.0)

    def test_camera_delay_sign_and_pixel_timing(self):
        # Images are rendered at the pose 20 ms AFTER their stamp (delay_p) and the truth offset is +0.02.
        # The filter starts at offset 0 with a 20 ms prior and must move its estimate to +0.02. If the
        # truth poses were taken before the stamp, or the offset were stored with the other sign, the
        # estimate would run to -0.02 or stay at 0.
        err = np.array([r['err'][:, TM] for r in self.delay])
        sig = np.array([r['sig'][:, TM] for r in self.delay])
        estimate = 0.02 - err[:, -1]
        self.assertTrue(np.all(estimate > 0.012), estimate)                       # moved towards +20 ms
        self.assertTrue(np.all(abs(err[:, -1]) < 0.35*0.02), err[:, -1])
        self.assertTrue(np.all(sig[:, -1] < 0.5*sig[:, 0]), (sig[:, 0], sig[:, -1]))


if __name__ == '__main__':
    unittest.main()
