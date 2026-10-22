"""GPL-3.0-only. Reproducible navigation algebra checks, not flight evidence.

Run this file directly for the numerical report used by navigation_fragment.tex.
Only NumPy and the adjacent reference models are required. All data are synthetic.
"""
import itertools
import json
import math
import unittest
import numpy as np
from models import rotation, skew, expm, rk4, ncc_fft

SEED = 20260930


def rotation_log(r):
    angle = math.acos(np.clip((np.trace(r) - 1) / 2, -1, 1))
    v = np.array([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0],
                  r[1, 0]-r[0, 1]]) / 2
    return v if angle < 1e-7 else angle / math.sin(angle) * v


def right_jacobian(a):
    angle = np.linalg.norm(a)
    s = skew(a)
    if angle < 1e-5:
        return np.eye(3) - s / 2 + s @ s / 6
    return (np.eye(3) - (1-math.cos(angle))/angle**2*s
            + (angle-math.sin(angle))/angle**3*(s@s))


def quaternion_multiply(a, b):
    return np.r_[a[0]*b[0]-np.dot(a[1:], b[1:]),
                 a[0]*b[1:]+b[0]*a[1:]+np.cross(a[1:], b[1:])]


def quaternion_increment(v):
    a = np.linalg.norm(v)
    if a < 1e-12:
        return np.r_[1., v/2]
    return np.r_[math.cos(a/2), math.sin(a/2)*v/a]


def quaternion_rotation(q):
    w, v = q[0], q[1:]
    return (w*w - np.dot(v, v))*np.eye(3) + 2*np.outer(v, v) + 2*w*skew(v)


def lever_observation(m, p, r, rcb, lever):
    pc = rcb @ (r.T @ (m-p)-lever)
    if pc[2] <= 0:
        raise ValueError('point behind camera')
    return np.array([640*pc[0]/pc[2]+640, 640*pc[1]/pc[2]+400])


def lever_jacobians(m, p, r, rcb, lever):
    pb = r.T @ (m-p)
    x, y, z = rcb @ (pb-lever)
    j = np.array([[640/z, 0, -640*x/z**2], [0, 640/z, -640*y/z**2]])
    return -j @ rcb @ r.T, j @ rcb @ skew(pb)


def numerical_report():
    """Preserve input order, seed, and parameters of the documented experiment."""
    rng = np.random.default_rng(SEED)
    k = np.array([[640., 0, 640], [0, 640, 400], [0, 0, 1.]])
    r0 = np.diag([1., -1., -1.])
    rcb = rotation(np.array([.02, -.04, .01]))
    lever = np.array([.04, -.03, .015])
    p = np.array([13., -9., 300.])
    eps = 1e-5
    errors, homerrors = [], []
    for _ in range(100):
        r = r0 @ rotation(rng.normal(0, .15, 3))
        m = np.r_[rng.uniform(-100, 100, 2), 0.]
        hp, ht = lever_jacobians(m, p, r, rcb, lever)
        dp = np.column_stack([
            (lever_observation(m, p+eps*np.eye(3)[j], r, rcb, lever)
             - lever_observation(m, p-eps*np.eye(3)[j], r, rcb, lever))/(2*eps)
            for j in range(3)])
        dt = np.column_stack([
            (lever_observation(m, p, r@rotation(eps*np.eye(3)[j]), rcb, lever)
             - lever_observation(m, p, r@rotation(-eps*np.eye(3)[j]), rcb, lever))/(2*eps)
            for j in range(3)])
        errors.append([np.max(abs(dp-hp)), np.max(abs(dt-ht))])
        centre, rcw = p+r@lever, rcb@r.T
        h = k @ np.column_stack((rcw[:, 0], rcw[:, 1], -rcw@centre))
        image = h @ np.r_[m[:2], 1.]
        homerrors.append(np.max(abs(image[:2]/image[2]
                                   - lever_observation(m, p, r, rcb, lever))))
    a = np.array([.3, -.2, .1])
    reset_fd = np.column_stack([
        (rotation_log(rotation(-a)@rotation(a+1e-6*np.eye(3)[j]))
         - rotation_log(rotation(-a)@rotation(a-1e-6*np.eye(3)[j])))/(2e-6)
        for j in range(3)])
    image, template = rng.normal(size=(13, 16)), rng.normal(size=(5, 4))
    tc = template-template.mean()
    pad = (17, 19)
    corr = np.fft.ifft2(np.fft.fft2(image, pad)
                       * np.conj(np.fft.fft2(tc, pad))).real
    direct = np.empty((9, 13))
    for y in range(9):
        for x in range(13):
            window = image[y:y+5, x:x+4]
            wc = window-window.mean()
            den = np.sqrt(np.sum(wc*wc)*np.sum(tc*tc))
            direct[y, x] = np.sum(wc*tc)/den
            corr[y, x] /= den
    gaussian = rng.normal(size=(200000, 2))
    squared = np.sum(gaussian*gaussian, axis=1)
    j0 = np.diag([.2, .4, .6])
    js = []
    for _ in range(8):
        h = rng.normal(size=(2, 3))
        js.append(h.T@h)
    def objective(ids):
        return (np.linalg.slogdet(j0+sum((js[i] for i in ids), np.zeros((3, 3))))[1]
                - np.linalg.slogdet(j0)[1])
    max_violation = 0.
    comparisons = 0
    for mask in range(256):
        small = {i for i in range(8) if mask >> i & 1}
        for j in range(8):
            if j in small:
                continue
            for b in range(8):
                if b in small or b == j:
                    continue
                large = small | {b}
                max_violation = max(max_violation,
                    objective(large|{j})-objective(large)
                    -objective(small|{j})+objective(small))
                comparisons += 1
    greedy = set()
    for _ in range(3):
        choice = max((i for i in range(8) if i not in greedy),
                     key=lambda i: objective(greedy|{i})-objective(greedy))
        greedy.add(choice)
    optimal = max((objective(ids), ids) for ids in itertools.combinations(range(8), 3))
    sets = {'centre': np.array([-.02, -.01, 0, .01, .02]),
            'same_abs_edge': np.array([-1., -1., 1., 1.]),
            'spread': np.array([-1., -.5, 0, .5, 1.])}
    return {
        'seed': SEED,
        'reprojection_jacobian_max_errors': np.max(errors, axis=0).tolist(),
        'homography_max_error': max(homerrors),
        'exact_reset_Jr_error': float(np.max(abs(reset_fd-right_jacobian(a)))),
        'FFT_NCC_max_error': float(np.max(abs(corr[:9, :13]-direct))),
        'model_NCC_max_error': float(np.max(abs(ncc_fft(image, template)-direct))),
        'tilt_information': {name: float(640**2/.2**2*np.sum((x*x-np.mean(x*x))**2))
                             for name, x in sets.items()},
        'PL_exact95_k': math.sqrt(-2*math.log(.05)),
        'PL95_MC_coverage': float(np.mean(squared <= -2*math.log(.05))),
        'gate997_MC_rejection_rate': float(np.mean(squared > -2*math.log(.003))),
        'PL_k3_exact_tail': math.exp(-9/2),
        'ransac_similarity_trials': math.ceil(math.log(.001)/math.log(.75)),
        'ransac_homography_trials': math.ceil(math.log(.001)/math.log(.9375)),
        'submodular_comparisons': comparisons,
        'submodular_max_violation': max_violation,
        'greedy_set': sorted(greedy), 'optimal_set': list(optimal[1]),
        'greedy_value': objective(greedy), 'opt_value': optimal[0],
        'greedy_opt_ratio': objective(greedy)/optimal[0]}


class NavigationExtra(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = numerical_report()

    def test_homography_with_extrinsics_and_lever(self):
        self.assertLess(self.report['homography_max_error'], 1e-10)

    def test_right_error_reprojection_with_lever(self):
        # The attitude derivative must use pb, not pb-lever, and a plus sign.
        ep, et = self.report['reprojection_jacobian_max_errors']
        self.assertLess(ep, 2e-7)
        self.assertLess(et, 2e-6)

    def test_exact_reset_derivative(self):
        self.assertLess(self.report['exact_reset_Jr_error'], 1e-9)
        np.testing.assert_allclose(right_jacobian(np.zeros(3)), np.eye(3))
        a = np.array([.3, -.2, .1])
        # Independent integration of the differential exponential.
        s = np.linspace(0, 1, 20001)
        integrand = np.array([rotation(-t*a) for t in s])
        np.testing.assert_allclose(np.trapezoid(integrand, s, axis=0),
                                   right_jacobian(a), atol=4e-11, rtol=0)

    def test_quaternion_exact_propagation(self):
        initial = quaternion_increment(np.array([.2, -.4, .1]))
        omega = np.array([.7, -.3, .6])
        duration = .5
        qexact = quaternion_multiply(initial, quaternion_increment(omega*duration))
        qnumeric = rk4(lambda t, q: .5*quaternion_multiply(q, np.r_[0., omega]),
                       initial, duration/100, 100)
        np.testing.assert_allclose(qnumeric, qexact, atol=2e-13, rtol=0)
        np.testing.assert_allclose(quaternion_rotation(qexact),
                                   quaternion_rotation(initial)@expm(skew(omega)*duration),
                                   atol=2e-15, rtol=0)
        self.assertAlmostEqual(np.linalg.norm(qexact), 1., places=14)
        # Different finite rotations do not commute: integrated angular vector
        # cannot in general replace time ordered quaternion propagation.
        a, b = np.array([.2, 0., 0.]), np.array([0., .3, 0.])
        self.assertGreater(np.linalg.norm(rotation(a)@rotation(b)-rotation(a+b)), .03)

    def test_fft_normalization_and_constant_template(self):
        self.assertLess(self.report['FFT_NCC_max_error'], 2e-15)
        self.assertLess(self.report['model_NCC_max_error'], 2e-15)
        self.assertTrue(np.all(np.isnan(ncc_fft(np.arange(36.).reshape(6, 6),
                                               np.ones((2, 2))))))

    def test_tilt_information_degeneracy(self):
        info = self.report['tilt_information']
        self.assertAlmostEqual(info['centre'], 1.4336, places=12)
        self.assertEqual(info['same_abs_edge'], 0.)
        self.assertAlmostEqual(info['spread'], 8960000., places=6)
        # Schur complement, rather than only the residual formula.
        x = np.array([-1., -.5, 0., .5, 1.])
        h = np.column_stack((np.full(len(x), -640/300), -640*(1+x*x)))
        j = h.T@h/.2**2
        self.assertAlmostEqual(j[1, 1]-j[1, 0]**2/j[0, 0], info['spread'], places=5)

    def test_joint_subpixel_peak_needed(self):
        q, d = np.array([[3., 1.], [1., 2.]]), np.array([.23, -.28])
        grad, hess = 2*q@d, -2*q
        full = -np.linalg.solve(hess, grad)
        axis = -grad/np.diag(hess)
        np.testing.assert_allclose(full, d, atol=1e-15)
        np.testing.assert_allclose(axis, [0.1366666666666667, -.165], atol=1e-15)
        self.assertGreater(np.linalg.norm(axis-d), .14)

    def test_gaussian_radial_PL_and_gating(self):
        r = self.report
        self.assertAlmostEqual(r['PL_exact95_k'], 2.44774683068, places=10)
        self.assertLess(abs(r['PL95_MC_coverage']-.95), 5*math.sqrt(.95*.05/200000))
        self.assertLess(abs(r['gate997_MC_rejection_rate']-.003),
                        5*math.sqrt(.003*.997/200000))
        self.assertAlmostEqual(r['PL_k3_exact_tail'], .011108996538242306)

    def test_ransac_rounding(self):
        self.assertEqual(self.report['ransac_similarity_trials'], 25)
        self.assertEqual(self.report['ransac_homography_trials'], 108)
        self.assertLess(1-.75**24, .999)
        self.assertGreaterEqual(1-.75**25, .999)

    def test_normalized_logdet_greedy(self):
        r = self.report
        self.assertEqual(r['submodular_comparisons'], 3584)
        self.assertLessEqual(r['submodular_max_violation'], 2e-13)
        self.assertGreaterEqual(r['greedy_opt_ratio'], 1-(1-1/3)**3)
        self.assertLessEqual(r['greedy_opt_ratio'], 1+1e-14)

    def test_downward_scale_correction_and_signed_velocity(self):
        world = np.array([30., -20.])
        p1, p2 = np.array([4., 7.]), np.array([5.2, 6.4])
        h1, h2, f = 100., 97.9, 640.
        uv1, uv2 = f*(world-p1)/h1, f*(world-p2)/h2
        np.testing.assert_allclose(h1/f*uv1-h2/f*uv2, p2-p1, atol=1e-14)
        # No horizontal motion still causes pixel expansion while descending.
        self.assertGreater(np.linalg.norm(f*(world-p1)/h2-uv1), 1)
        uv3 = f*(world-p2)/h1
        np.testing.assert_allclose(-h1/f*(uv3-uv1), p2-p1, atol=1e-14)

    def test_shared_image_velocity_temporal_covariance(self):
        rng = np.random.default_rng(SEED+1)
        eta = rng.normal(size=(300000, 3))*.2
        b = np.array([100., 97.9, 95.8])/640
        d1, d2 = b[0]*eta[:, 0]-b[1]*eta[:, 1], b[1]*eta[:, 1]-b[2]*eta[:, 2]
        expected = np.array([[.2**2*(b[0]**2+b[1]**2), -.2**2*b[1]**2],
                             [-.2**2*b[1]**2, .2**2*(b[1]**2+b[2]**2)]])
        np.testing.assert_allclose(np.cov(np.stack((d1, d2)), bias=True),
                                   expected, rtol=.008, atol=0)

    def test_TLS_quadratic_lift_exact_cases(self):
        q = np.array([.9, .2, 4., -3.])
        a = np.array([[2., -1., 1., 0.], [1., 2., 0., 1.]])
        m = np.array([5., 2.])
        sigma2, cap = .25, 9.
        original = min(np.sum((m-a@q)**2)/sigma2, cap)
        cases = []
        for w in (0, 1):
            v = w*q
            self.assertEqual(w*w-w, 0)
            np.testing.assert_array_equal(v-w*q, np.zeros(4))
            cases.append(np.sum((w*m-a@v)**2)/sigma2+(1-w)*cap)
        self.assertEqual(original, min(cases))


if __name__ == '__main__':
    print(json.dumps(numerical_report(), indent=2))
    unittest.main(verbosity=2)
