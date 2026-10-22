"""GPL-3.0-only. Tests of the coarse acquisition on a small synthetic terrain.

The terrain is a patchwork of random fields with dark boundaries, rendered by a ray-cast written here
so that the tests do not depend on sim/. It is a test stimulus, not farmland: these tests show that
the geometry, conventions and refusals are right, not how well the method works on real imagery
(sim/nav/run_acquisition_mc.py measures that on a richer synthetic terrain).
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cansat import coarse  # noqa: E402
from cansat.coarse import (Acquirer, CoarseConfig, CoarseMap, Leveller, acquire, bin_camera, bin_image,  # noqa: E402
                           heading_between, ncc_valid, wrap)
from cansat.eskf import LEVEL, Camera, exp_so3  # noqa: E402

FOCAL = 2.5e-3/3.45e-6
FULL = Camera(FOCAL, FOCAL, 728.0, 544.0, 1456, 1088, lever=np.array([0.0, 0.0, 0.045]))
CAM = bin_camera(FULL, 4)                 # the camera the test images are rendered for: 364 x 272
SIZE = (272, 364)
RES = 1.0                                 # world raster, m/px
WORLD_M = 1280.0


def patchwork(seed, cells=70, hedge_m=4.0):
    """Two rasters with the same field boundaries and independently drawn field brightness."""
    rng = np.random.default_rng(seed)
    n = int(WORLD_M/RES)
    seeds = rng.uniform(0, n, (cells, 2))
    yy, xx = np.mgrid[0:n, 0:n].astype(np.float32)
    best = np.full((n, n), np.inf, dtype=np.float32)
    second = np.full((n, n), np.inf, dtype=np.float32)
    label = np.zeros((n, n), dtype=np.int32)
    for i, (sx, sy) in enumerate(seeds):
        d = np.hypot(xx - np.float32(sx), yy - np.float32(sy))
        closer = d < best
        second = np.where(closer, best, np.minimum(second, d))
        label = np.where(closer, i, label)
        best = np.where(closer, d, best)
    hedge = (second - best) < hedge_m/RES
    rasters = []
    for _ in range(2):
        a = rng.uniform(80, 180, cells)[label] + 3.0*rng.standard_normal((n, n))
        a[hedge] = 50.0
        rasters.append(a)
    origin = (-(n - 1)/2*RES, -(n - 1)/2*RES)
    return rasters[0], rasters[1], origin


def render(raster, origin, p, r, rng=None, noise=1.0):
    """Ray-cast of the ground plane through CAM at IMU position p, attitude r."""
    u, v = np.meshgrid(np.arange(SIZE[1], dtype=float), np.arange(SIZE[0], dtype=float))
    rays = CAM.ray(np.column_stack((u.ravel(), v.ravel()))) @ r.T
    centre = p + r @ CAM.lever
    depth = centre[2]/-rays[:, 2]
    east, north = centre[0] + rays[:, 0]*depth, centre[1] + rays[:, 1]*depth
    image = coarse._bilinear(raster, (east - origin[0])/RES, (north - origin[1])/RES)
    image = np.where(np.isfinite(image), image, 128.0).reshape(SIZE)
    if rng is not None and noise:
        image = image + noise*rng.standard_normal(SIZE)
    return image


def to_map(raster, origin, coarse_only=False):
    """The onboard map: the 1 m/px raster itself, or its 2 x 2 block means (no stage 4)."""
    if not coarse_only:
        return raster, RES, origin
    n = raster.shape[0]//2*2
    m = raster[:n, :n].reshape(n//2, 2, n//2, 2).mean(axis=(1, 3))
    return m, 2.0, (origin[0] + RES/2, origin[1] + RES/2)


def attitude(yaw, tilt_x=0.0, tilt_y=0.0):
    return coarse._rz(yaw) @ LEVEL @ exp_so3(np.array([tilt_x, tilt_y, 0.0]))


def angle_between(a, b):
    m = a.T @ b
    return math.acos(max(-1.0, min(1.0, (np.trace(m) - 1)/2)))


class Shared(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.flight, cls.changed, cls.origin = patchwork(11)
        cls.cmap = CoarseMap(*to_map(cls.flight, cls.origin))
        cls.cmap_changed = CoarseMap(*to_map(cls.changed, cls.origin))
        cls.cmap_2m = CoarseMap(*to_map(cls.flight, cls.origin, coarse_only=True))

    def pose(self, rng, tilt_deg=6.0, reach=180.0):
        p = np.array([rng.uniform(-reach, reach), rng.uniform(-reach, reach), rng.uniform(260.0, 310.0)])
        tilt = np.radians(rng.normal(0.0, tilt_deg, 2))
        yaw = rng.uniform(-math.pi, math.pi)
        r = attitude(yaw, *tilt)
        r0 = attitude(rng.uniform(-math.pi, math.pi), *tilt)      # right tilt, unrelated heading
        return p, r, r0


class ArrayTools(unittest.TestCase):
    def test_blur_has_the_requested_width_and_unit_gain(self):
        for sigma in (0.8, 1.5, 2.0, 4.0, 7.3):
            impulse = np.zeros((129, 129))
            impulse[64, 64] = 1.0
            out = coarse._blur(impulse, sigma)
            x = np.arange(129) - 64
            profile = out.sum(axis=0)
            self.assertAlmostEqual(out.sum(), 1.0, places=9)
            self.assertAlmostEqual(float(profile @ x), 0.0, places=9)             # no shift
            self.assertLess(abs(math.sqrt(float(profile @ (x*x))) - sigma), 0.02 if sigma < 2 else 0.2)
        flat = np.full((40, 50), 3.7)
        np.testing.assert_allclose(coarse._blur(flat, 5.0), flat, atol=1e-12)

    def test_box_blur_is_close_to_a_gaussian(self):
        impulse = np.zeros((129, 129))
        impulse[64, 64] = 1.0
        x = np.arange(129) - 64
        exact = np.exp(-0.5*(x/5.0)**2)
        exact /= exact.sum()
        profile = coarse._blur(impulse, 5.0).sum(axis=0)
        self.assertLess(np.abs(profile - exact).max(), 0.05*exact.max())

    def test_ncc_matches_the_definition(self):
        rng = np.random.default_rng(0)
        search = rng.normal(size=(40, 47))
        template = search[9:9 + 12, 21:21 + 15] + 0.05*rng.normal(size=(12, 15))
        fast = ncc_valid(search, template)
        slow = np.zeros_like(fast)
        tc = template - template.mean()
        for i in range(fast.shape[0]):
            for j in range(fast.shape[1]):
                w = search[i:i + 12, j:j + 15]
                wc = w - w.mean()
                slow[i, j] = float((wc*tc).sum()/math.sqrt((wc*wc).sum()*(tc*tc).sum()))
        np.testing.assert_allclose(fast, slow, atol=1e-10)
        self.assertEqual(np.unravel_index(np.argmax(fast), fast.shape), (9, 21))
        # Gain and offset of either input change nothing; a flat search patch scores zero, not NaN.
        np.testing.assert_allclose(ncc_valid(3*search + 7, 0.5*template - 2), fast, atol=1e-10)
        self.assertTrue(np.all(ncc_valid(np.ones((20, 20)), template[:8, :8]) == 0.0))

    def test_binned_camera_describes_the_binned_image(self):
        rng = np.random.default_rng(1)
        points = np.column_stack((rng.uniform(-150, 150, 50), rng.uniform(-100, 100, 50), np.zeros(50)))
        p, r = np.array([3.0, -4.0, 280.0]), attitude(0.7, 0.05, -0.03)
        full, _ = FULL.project(points, p, r)
        for b in (2, 3, 4):
            binned, _ = bin_camera(FULL, b).project(points, p, r)
            np.testing.assert_allclose(binned, (full - (b - 1)/2)/b, atol=1e-9)
        # A brightness ramp keeps its value at the corresponding coordinate after block averaging.
        u, v = np.meshgrid(np.arange(64.0), np.arange(48.0))
        ramp = 2.0*u - 3.0*v
        small = bin_image(ramp, 4)
        self.assertEqual(small.shape, (12, 16))
        self.assertAlmostEqual(small[5, 7], 2.0*(4*7 + 1.5) - 3.0*(4*5 + 1.5))
        self.assertEqual(bin_image(np.zeros((50, 67)), 4).shape, (12, 16))    # ragged edges are dropped

    def test_edge_strength_ignores_gain_and_contrast_sign(self):
        rng = np.random.default_rng(2)
        a = coarse._blur(rng.normal(size=(60, 60)), 2.0)
        e, ok = coarse.edge_strength(a)
        e2, _ = coarse.edge_strength(-4.0*a + 30.0)
        np.testing.assert_allclose(e, e2, atol=1e-12)
        self.assertTrue(np.all((e >= 0) & (e < 1)))
        self.assertFalse(ok[0, 5] or ok[5, 0])
        flat, _ = coarse.edge_strength(np.full((20, 20), 9.0))
        self.assertTrue(np.all(flat == 0))


class Levelling(unittest.TestCase):
    G = 9.80665

    def run_leveller(self, r_of_t, rate, seconds=6.0, dt=0.01):
        lev = Leveller(2.0)
        for k in range(int(seconds/dt)):
            r = r_of_t((k + 0.5)*dt)
            lev.update(dt, r.T @ np.array([0.0, 0.0, self.G]), rate)
        return lev

    def test_static_tilt_is_recovered(self):
        r = attitude(1.1, 0.21, -0.13)
        lev = self.run_leveller(lambda t: r, np.zeros(3))
        up_true, up_est = r.T @ [0, 0, 1.0], lev.attitude().T @ [0, 0, 1.0]
        self.assertLess(math.acos(min(1.0, float(up_true @ up_est))), 1e-9)

    def test_spin_keeps_tilt_and_carries_heading(self):
        rate = np.array([0.0, 0.0, 0.9])                         # body spin about its own axis
        base = attitude(0.3, 0.12, 0.08)
        lev = Leveller(2.0)
        r, marks = base.copy(), {}
        for k in range(600):
            mid = r @ exp_so3(rate*0.005)
            lev.update(0.01, mid.T @ np.array([0.0, 0.0, self.G]), rate)
            r = r @ exp_so3(rate*0.01)
            if k in (299, 599):
                marks[k] = (lev.attitude(), r.copy())
        for est, true in marks.values():
            self.assertLess(math.acos(min(1.0, float((est.T @ [0, 0, 1.0]) @ (true.T @ [0, 0, 1.0])))), 1e-6)
        # The estimated heading turned by exactly as much as the true one.
        (e1, t1), (e2, t2) = marks[299], marks[599]
        self.assertAlmostEqual(wrap(heading_between(e2, e1) - heading_between(t2, t1)), 0.0, places=6)

    def test_bad_samples_are_ignored(self):
        lev = Leveller()
        self.assertIsNone(lev.attitude())
        lev.update(0.01, [float('nan'), 0, 0], [0, 0, 0])
        lev.update(-1.0, [0, 0, -9.8], [0, 0, 0])
        self.assertIsNone(lev.attitude())
        lev.update(0.01, [0, 0, -9.8], [0, 0, 0])
        np.testing.assert_allclose(lev.attitude(), LEVEL, atol=1e-12)


class Acquire(Shared):
    def test_recovers_pose_with_unknown_yaw(self):
        rng = np.random.default_rng(5)
        errors, biases, angles = [], [], []
        for _ in range(12):
            p, r, r0 = self.pose(rng)
            acq = acquire(render(self.flight, self.origin, p, r, rng), CAM, r0, p[2], self.cmap)
            self.assertTrue(acq.ok, acq.reason)
            errors.append(acq.fix.p - p)
            angles.append(angle_between(acq.fix.r, r))
            rays = CAM.ray(acq.pixels) @ r.T
            centre = p + r @ CAM.lever
            ground = centre[:2] + rays[:, :2]*(centre[2]/-rays[:, 2])[:, None]
            biases.append((acq.points[:, :2] - ground)[acq.fix.inliers])
            self.assertLess(abs(wrap(acq.yaw - heading_between(r, r0))), math.radians(4.0))
        errors, biases = np.array(errors), np.concatenate(biases)
        horizontal = np.hypot(errors[:, 0], errors[:, 1])
        self.assertLess(horizontal.max(), 2.0)
        self.assertLess(math.sqrt(float(np.mean(horizontal**2))), 1.0)
        self.assertLess(max(angles), math.radians(0.4))
        # No half-pixel slip anywhere in the chain: the mean match error is a small fraction of a map pixel.
        self.assertLess(float(np.abs(biases.mean(axis=0)).max()), 0.08)
        self.assertLess(float(biases.std(axis=0).max()), 0.6)

    def test_refinement_is_what_makes_the_fix_accurate(self):
        rng = np.random.default_rng(15)
        fine, coarse_only = [], []
        for _ in range(10):
            p, r, r0 = self.pose(rng)
            r0 = r0 @ exp_so3(np.radians(np.r_[rng.normal(0.0, 2.0, 2), 0.0]))
            image = render(self.flight, self.origin, p, r, rng)
            a, b = acquire(image, CAM, r0, p[2], self.cmap), acquire(image, CAM, r0, p[2], self.cmap_2m)
            self.assertTrue(b.polished is False and b.coarse_fix is None)
            if a.ok:
                self.assertTrue(a.polished)
                self.assertIsNotNone(a.coarse_fix)
                fine.append(math.hypot(*(a.fix.p - p)[:2]))
            if b.ok:
                coarse_only.append(math.hypot(*(b.fix.p - p)[:2]))
        self.assertGreaterEqual(len(fine), 8)
        self.assertGreaterEqual(len(coarse_only), 5)
        rms = lambda v: math.sqrt(float(np.mean(np.square(v))))        # noqa: E731
        self.assertLess(rms(fine), 0.6*rms(coarse_only))

    def test_survives_every_field_changing_brightness(self):
        rng = np.random.default_rng(6)
        good = 0
        for _ in range(10):
            p, r, r0 = self.pose(rng)
            acq = acquire(render(self.flight, self.origin, p, r, rng), CAM, r0, p[2], self.cmap_changed)
            if acq.ok:
                self.assertLess(math.hypot(*(acq.fix.p - p)[:2]), 10.0)
                good += 1
        self.assertGreaterEqual(good, 8)

    def test_tolerates_levelling_and_height_error(self):
        rng = np.random.default_rng(7)
        good = 0
        for _ in range(8):
            p, r, r0 = self.pose(rng)
            r0 = r0 @ exp_so3(np.radians(np.r_[rng.normal(0.0, 2.0, 2), 0.0]))
            acq = acquire(render(self.flight, self.origin, p, r, rng), CAM, r0, p[2]*(1 + rng.normal(0.0, 0.03)), self.cmap)
            if acq.ok:
                self.assertLess(math.hypot(*(acq.fix.p - p)[:2]), 10.0)
                good += 1
        self.assertGreaterEqual(good, 7)

    def test_refuses_when_the_map_is_of_somewhere_else(self):
        other, _, origin = patchwork(99)
        rng = np.random.default_rng(8)
        fixes = 0
        for _ in range(6):
            p, r, r0 = self.pose(rng)
            acq = acquire(render(other, origin, p, r, rng), CAM, r0, p[2], self.cmap)
            fixes += acq.ok
            self.assertTrue(acq.reason)
        self.assertEqual(fixes, 0)

    def test_refuses_a_featureless_view(self):
        rng = np.random.default_rng(9)
        for image in (np.full(SIZE, 120.0), 120.0 + rng.standard_normal(SIZE)):
            acq = acquire(image, CAM, LEVEL, 290.0, self.cmap)
            self.assertFalse(acq.ok)
            self.assertIn(acq.reason, ('featureless view', 'too little structure', 'too few matches', 'no consensus',
                                       'weak consensus', 'disagrees with the search', 'poor residuals'))

    def test_compass_prior_narrows_the_search(self):
        rng = np.random.default_rng(10)
        p, r, r0 = self.pose(rng)
        image = render(self.flight, self.origin, p, r, rng)
        psi = heading_between(r, r0)
        whole = acquire(image, CAM, r0, p[2], self.cmap)
        narrow = acquire(image, CAM, r0, p[2], self.cmap, yaw_prior=(psi + math.radians(8.0), math.radians(15.0)))
        self.assertTrue(whole.ok and narrow.ok)
        self.assertLess(math.hypot(*(narrow.fix.p - p)[:2]), 5.0)
        self.assertTrue(narrow.polished)
        # 60 hypotheses against 2*3 + 1, two FFTs each, plus three for the disc.
        self.assertEqual(whole.ffts[256], 3 + 2*60)
        self.assertEqual(narrow.ffts[256], 3 + 2*7)
        # A prior that excludes the truth must not produce a fix somewhere else.
        wrong = acquire(image, CAM, r0, p[2], self.cmap, yaw_prior=(psi + math.pi/2, math.radians(15.0)))
        if wrong.ok:
            self.assertLess(math.hypot(*(wrong.fix.p - p)[:2]), 10.0)

    def test_cost_is_bounded_by_the_configuration(self):
        rng = np.random.default_rng(12)
        cfg = self.cmap.cfg
        for _ in range(3):
            p, r, r0 = self.pose(rng)
            acq = acquire(render(self.flight, self.origin, p, r, rng), CAM, r0, p[2], self.cmap)
            self.assertLessEqual(acq.ffts.get(256, 0), 3 + 2*round(360/cfg.yaw_step_deg))
            self.assertLessEqual(acq.ffts.get(128, 0), 3*(cfg.candidates*cfg.sites[0]*cfg.sites[1]
                                                          + cfg.polish_sites[0]*cfg.polish_sites[1]))
            self.assertLessEqual(acq.sites, cfg.sites[0]*cfg.sites[1])
            self.assertEqual(set(acq.ffts) - {128, 256}, set())

    def test_never_raises(self):
        image = render(self.flight, self.origin, np.array([0.0, 0.0, 280.0]), LEVEL)
        bad_images = (None, 'image', np.zeros((0, 0)), np.zeros((10, 10)), np.full(SIZE, np.nan), np.zeros((4, 5, 6)),
                      np.where(np.arange(SIZE[1]) % 7 == 0, np.inf, image))
        for bad in bad_images:
            acq = acquire(bad, CAM, LEVEL, 280.0, self.cmap)
            self.assertFalse(acq.ok)
            self.assertTrue(acq.reason)
        for r0 in (None, np.zeros((3, 3)), np.full((3, 3), np.nan), np.eye(2), -LEVEL, np.eye(3)):
            self.assertFalse(acquire(image, CAM, r0, 280.0, self.cmap).ok)
        for height in (0.0, -50.0, float('nan'), float('inf'), None, 'high', 1.0e9, 5.0):
            self.assertFalse(acquire(image, CAM, LEVEL, height, self.cmap).ok)
        self.assertFalse(acquire(image, None, LEVEL, 280.0, self.cmap).ok)
        self.assertFalse(acquire(image, CAM, LEVEL, 280.0, None).ok)
        self.assertFalse(acquire(image, CAM, LEVEL, 280.0, self.cmap, yaw_prior=('a', 'b')).ok)
        self.assertFalse(acquire(image, CAM, LEVEL, 280.0, self.cmap, prior_radius=float('nan')).ok)

    def test_refuses_a_view_of_the_horizon(self):
        image = render(self.flight, self.origin, np.array([0.0, 0.0, 280.0]), LEVEL)
        acq = acquire(image, CAM, attitude(0.0, math.radians(60.0)), 280.0, self.cmap)
        self.assertEqual((acq.ok, acq.reason), (False, 'tilt too large'))


class MapChecks(unittest.TestCase):
    def test_map_must_cover_the_window_and_have_the_right_resolution(self):
        raster = np.random.default_rng(0).normal(size=(600, 600))
        with self.assertRaises(ValueError):
            CoarseMap(raster, 1.0, (-300.0, -300.0))                 # right resolution, too small at 1 m/px
        with self.assertRaises(ValueError):
            CoarseMap(raster, 0.5, (-150.0, -150.0))                 # not a resolution it is built for
        with self.assertRaises(ValueError):
            CoarseMap(raster[:400, :400], 2.0, (-400.0, -400.0))
        with self.assertRaises(ValueError):
            CoarseMap(raster, 2.0, (-600.0, -600.0), centre=(300.0, 0.0))
        hole = raster.copy()
        hole[300, 300] = np.nan
        with self.assertRaises(ValueError):
            CoarseMap(hole, 2.0, (-599.0, -599.0))
        cmap = CoarseMap(raster, 2.0, (-599.0, -599.0))
        self.assertEqual((cmap.f1.shape, cmap.f2.shape), ((256, 256), (512, 512)))
        # Index [0, 0] of both grids is the same world point.
        self.assertEqual(cmap.world(0, 0, level=1), cmap.world(0, 0, level=2))
        self.assertAlmostEqual(cmap.world(0, 3, level=1)[0] - cmap.world(0, 0, level=1)[0], 12.0)
        self.assertIsNone(cmap.fine)                                 # a 2 m/px map has no stage 4
        big = np.random.default_rng(1).normal(size=(1100, 1100))
        fine = CoarseMap(big, 1.0, (-549.5, -549.5))
        self.assertEqual(fine.fine.shape, (1024, 1024))
        # A 2 m pixel is the mean of four 1 m pixels and sits at their centre.
        self.assertAlmostEqual(fine.origin[0], fine.origin0[0] + 0.5)
        self.assertAlmostEqual(fine.origin[1], fine.origin0[1] + 0.5)
        self.assertLess(abs(fine.origin0[0] + 512.0), 1.0)


class Confirmation(Shared):
    def frames(self, rng, velocity, dt, spin=0.5):
        p, r, r0 = self.pose(rng, reach=120.0)
        turn = exp_so3(np.array([0.0, 0.0, spin*dt]))
        return ((p, r, r0), (p + np.asarray(velocity)*dt, r @ turn, r0 @ turn))

    def test_two_agreeing_fixes_are_confirmed_and_give_velocity(self):
        rng = np.random.default_rng(20)
        acq = Acquirer(self.cmap)
        velocity = np.array([4.0, -3.0, -10.5])
        (p1, r1, r01), (p2, r2, r02) = self.frames(rng, velocity, 1.0)
        status, first, v = acq.update(10.0, render(self.flight, self.origin, p1, r1, rng), CAM, r01, p1[2])
        self.assertEqual((status, v), ('pending', None))
        status, second, v = acq.update(11.0, render(self.flight, self.origin, p2, r2, rng), CAM, r02, p2[2])
        self.assertEqual(status, 'confirmed')
        self.assertLess(np.linalg.norm(v[:2] - velocity[:2]), 4.0)
        self.assertLess(math.hypot(*(second.fix.p - p2)[:2]), 5.0)
        self.assertEqual((acq.attempts, acq.fixes, acq.disagreements), (2, 2, 0))

    def test_a_fix_that_jumps_is_not_confirmed(self):
        rng = np.random.default_rng(21)
        acq = Acquirer(self.cmap)
        (p1, r1, r01), (p2, r2, r02) = self.frames(rng, [90.0, 0.0, -10.5], 1.0)     # 90 m in one second
        acq.update(10.0, render(self.flight, self.origin, p1, r1, rng), CAM, r01, p1[2])
        status, second, _ = acq.update(11.0, render(self.flight, self.origin, p2, r2, rng), CAM, r02, p2[2])
        self.assertTrue(second.ok)
        self.assertEqual((status, acq.disagreements), ('pending', 1))

    def test_a_fix_whose_heading_contradicts_the_gyro_is_not_confirmed(self):
        rng = np.random.default_rng(22)
        acq = Acquirer(self.cmap)
        (p1, r1, r01), (p2, r2, r02) = self.frames(rng, [3.0, 2.0, -10.5], 1.0)
        acq.update(10.0, render(self.flight, self.origin, p1, r1, rng), CAM, r01, p1[2])
        # The scene really turned 0.5 rad, but the gyro (through r0) says it did not turn at all.
        status, second, _ = acq.update(11.0, render(self.flight, self.origin, p2, r2, rng), CAM, r01, p2[2])
        self.assertTrue(second.ok)
        self.assertEqual(status, 'pending')

    def test_a_stale_pending_fix_is_forgotten(self):
        rng = np.random.default_rng(23)
        acq = Acquirer(self.cmap, max_gap_s=2.0)
        (p1, r1, r01), (p2, r2, r02) = self.frames(rng, [3.0, 2.0, -10.5], 1.0)
        acq.update(10.0, render(self.flight, self.origin, p1, r1, rng), CAM, r01, p1[2])
        status, _, _ = acq.update(15.0, render(self.flight, self.origin, p2, r2, rng), CAM, r02, p2[2])
        self.assertEqual(status, 'pending')                       # too long ago to compare: starts again


class AfterReview(Shared):
    """Behaviour added after the independent review (sim/nav/test_acquisition_independent.py)."""

    def test_a_map_that_repeats_gives_no_fix(self):
        # The strip of map from 460 m west of the pad to the pad is pasted again to the east of it, so a view
        # over the first strip fits equally well 460 m away. Both places are inside the searched area.
        twice = self.flight.copy()
        twice[:, 640:1100] = self.flight[:, 180:640]
        cmap = CoarseMap(twice, RES, self.origin)
        rng = np.random.default_rng(31)
        refused = 0
        for _ in range(4):
            p = np.array([-230.0 + rng.uniform(-10, 10), rng.uniform(-100, 100), rng.uniform(270.0, 300.0)])
            tilt = np.radians(rng.normal(0.0, 2.0, 2))
            r, r0 = attitude(rng.uniform(-math.pi, math.pi), *tilt), attitude(rng.uniform(-math.pi, math.pi), *tilt)
            image = render(twice, self.origin, p, r, rng)
            acq = acquire(image, CAM, r0, p[2], cmap)
            if acq.ok:
                self.assertLess(math.hypot(*(acq.fix.p - p)[:2]), 10.0)      # never the copy
            else:
                refused += acq.reason.startswith('ambiguous')
                self.assertLess(acq.lead, 0.2)
            # On the map without the copy the same image is fixed.
            self.assertTrue(acquire(image, CAM, r0, p[2], self.cmap).ok)
        self.assertGreaterEqual(refused, 3)

    def test_the_same_look_twice_is_not_a_confirmation(self):
        rng = np.random.default_rng(32)
        p, r, r0 = self.pose(rng, reach=120.0)
        image = render(self.flight, self.origin, p, r, rng)
        acq = Acquirer(self.cmap)
        self.assertEqual(acq.update(5.0, image, CAM, r0, p[2])[0], 'pending')
        self.assertEqual(acq.update(5.000001, image, CAM, r0, p[2])[0], 'pending')
        self.assertEqual(acq.update(5.1, image, CAM, r0, p[2])[0], 'pending')
        self.assertEqual(acq.pending[0], 5.0)                                  # the first look is kept
        self.assertEqual(acq.update(5.5, image, CAM, r0, p[2])[0], 'confirmed')

    def test_a_bad_timestamp_is_refused_and_forgotten(self):
        rng = np.random.default_rng(33)
        p, r, r0 = self.pose(rng, reach=120.0)
        image = render(self.flight, self.origin, p, r, rng)
        acq = Acquirer(self.cmap)
        self.assertEqual(acq.update(5.0, image, CAM, r0, p[2])[0], 'pending')
        for bad in (None, 'now', float('nan'), float('inf'), [1.0, 2.0]):
            status, result, velocity = acq.update(bad, image, CAM, r0, p[2])
            self.assertEqual((status, result.ok, velocity), ('none', False, None))
        # Nothing was left behind that makes a later call fail, with or without a fix in it.
        self.assertEqual(acq.update(6.0, np.full(SIZE, 100.0), CAM, r0, p[2])[0], 'none')
        self.assertEqual(acq.update(6.5, image, CAM, r0, p[2])[0], 'pending')
        self.assertEqual(acq.update(7.5, image, CAM, r0, p[2])[0], 'confirmed')

    def test_inputs_that_are_not_what_they_claim_are_refused(self):
        rng = np.random.default_rng(34)
        p, r, r0 = self.pose(rng)
        image = render(self.flight, self.origin, p, r, rng)
        self.assertTrue(acquire(image, CAM, r0, p[2], self.cmap).ok)
        self.assertEqual(acquire(image, CAM, 2.0*r0, p[2], self.cmap).reason, 'bad input')            # not a rotation
        self.assertEqual(acquire(image, CAM, r0 @ np.diag([1.0, 1.0, -1.0]), p[2], self.cmap).reason, 'bad input')
        self.assertEqual(acquire(image[:200], CAM, r0, p[2], self.cmap).reason, 'bad input')          # not the camera's image
        stretched = Camera(CAM.fx, CAM.fy, CAM.cx, CAM.cy, CAM.width, CAM.height, r_cb=2.0*np.eye(3), lever=CAM.lever)
        self.assertEqual(acquire(image, stretched, r0, p[2], self.cmap).reason, 'bad input')
        flat = Camera(0.0, CAM.fy, CAM.cx, CAM.cy, CAM.width, CAM.height)
        self.assertEqual(acquire(image, flat, r0, p[2], self.cmap).reason, 'bad input')

    def test_reasons_say_what_is_wrong(self):
        image = render(self.flight, self.origin, np.array([0.0, 0.0, 60.0]), LEVEL)
        self.assertEqual(acquire(image, CAM, LEVEL, 60.0, self.cmap).reason, 'view too small')         # too low, not tilted
        other = CoarseConfig(feature='intensity')
        image = render(self.flight, self.origin, np.array([0.0, 0.0, 290.0]), LEVEL)
        self.assertEqual(acquire(image, CAM, LEVEL, 290.0, self.cmap, config=other).reason, 'configuration does not match the map')

    def test_work_stays_bounded(self):
        rng = np.random.default_rng(35)
        p, r, r0 = self.pose(rng)
        image = render(self.flight, self.origin, p, r, rng)
        psi = heading_between(r, r0)
        # A prior wider than the circle is the whole circle, not more hypotheses than that.
        for width in (math.pi, 4*math.pi, 50.0):
            acq = acquire(image, CAM, r0, p[2], self.cmap, yaw_prior=(psi, width))
            self.assertEqual(acq.ffts[256], 3 + 2*60)
            self.assertTrue(acq.ok)
        self.assertEqual(acquire(image, CAM, r0, p[2], self.cmap, yaw_prior=(float('nan'), 0.2)).reason, 'bad input')
        # Configurations that would make one image cost seconds or gigabytes are refused, on the pad and in flight.
        for bad in (dict(yaw_step_deg=1.0), dict(yaw_step_deg=0.0), dict(candidates=0), dict(candidates=50), dict(sites=(40, 40)),
                    dict(polish_sites=(0, 3)), dict(template_px=400), dict(gap_exclusion_px=10**6), dict(feature='colour'),
                    dict(interim='guess'), dict(sites='many')):
            cfg = CoarseConfig(**bad)
            self.assertTrue(cfg.problem(), bad)
            self.assertTrue(acquire(image, CAM, r0, p[2], self.cmap, config=cfg).reason.startswith('bad configuration'), bad)
            with self.assertRaises(ValueError):
                CoarseMap(self.flight, RES, self.origin, config=cfg)
        self.assertEqual(CoarseConfig().problem(), '')


class LevellerAfterReview(unittest.TestCase):
    G = 9.80665

    def settled(self, r):
        lev = Leveller(2.0)
        for _ in range(800):
            lev.update(0.01, r.T @ np.array([0.0, 0.0, self.G]), np.zeros(3))
        return lev

    def tilt_error(self, lev, r):
        return math.acos(min(1.0, float((lev.attitude().T @ [0, 0, 1.0]) @ (r.T @ [0, 0, 1.0]))))

    def test_no_sample_can_raise_or_disable_it(self):
        r = attitude(0.4, 0.1, -0.05)
        lev = self.settled(r)
        bad = ((float('inf'), [0, 0, -9.8], [0, 0, 0]), (0.01, [1e200, 0, 0], [0, 0, 0]), (0.01, [0, 0, -9.8], [1e200, 0, 0]),
               (0.01, [0, -9.8], [0, 0, 0]), (0.01, [0, 0, -9.8, 1], [0, 0, 0]), (0.01, [0, 0, -9.8], [0, 0]), (None, [0, 0, -9.8], [0, 0, 0]),
               (0.01, None, [0, 0, 0]), (0.01, 'abc', [0, 0, 0]), (1e300, [0, 0, -9.8], [0, 0, 1.0]), (5.0, [0, 0, -9.8], [0, 0, 0]),
               (0.01, [0, 0, -9.8], [0, 0, 60.0]), (0.0, [0, 0, -9.8], [0, 0, 0]))
        for dt, accel, gyro in bad:
            lev.update(dt, accel, gyro)
        self.assertEqual(lev.ignored, len(bad))
        self.assertLess(self.tilt_error(lev, r), 1e-6)                         # none of them moved the estimate
        with self.assertRaises(ValueError):
            Leveller(tau=0.0)
        with self.assertRaises(ValueError):
            Leveller(tau=float('nan'))

    def test_a_shock_is_not_averaged_into_the_tilt(self):
        # 16 g sideways for 0.1 s, as at ejection or canopy opening. Averaged in, it would leave tens of degrees.
        r = attitude(-1.0, 0.05, 0.12)
        lev = self.settled(r)
        for _ in range(10):
            lev.update(0.01, r.T @ np.array([16*self.G, 0.0, self.G]), np.zeros(3))
        self.assertLess(self.tilt_error(lev, r), 1e-6)
        # Ordinary swing loads, 0.7 g to 1.4 g, are still used.
        steps = lev._steps
        weight = lev.weight
        lev.update(0.01, r.T @ np.array([0.0, 0.0, 1.4*self.G]), np.zeros(3))
        lev.update(0.01, r.T @ np.array([0.0, 0.0, 0.7*self.G]), np.zeros(3))
        self.assertEqual(lev._steps, steps + 2)
        self.assertGreater(lev.weight, weight)


if __name__ == '__main__':
    unittest.main()
