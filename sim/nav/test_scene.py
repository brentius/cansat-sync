"""GPL-3.0-only. Tests of the SYNTHETIC scene renderer and map preparation (sim/nav/scene.py).

These check the renderer's geometry and conventions against the flight camera model, because every
acquisition result depends on the two agreeing to a small fraction of a pixel. Nothing here is flight software.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.dont_write_bytecode = True
sys.path.insert(0, str(HERE))
import scene  # noqa: E402  (puts code/ on sys.path)
from cansat.coarse import _blur, bin_camera, bin_image  # noqa: E402
from cansat.eskf import LEVEL, Camera, exp_so3  # noqa: E402

FOCAL = 2.5e-3/3.45e-6
FULL = Camera(FOCAL, FOCAL, 728.0, 544.0, 1456, 1088, lever=np.array([0.0, 0.0, 0.045]))


def ramp(n, res, origin, a=0.31, b=-0.17, c=100.0):
    east = origin[0] + np.arange(n)*res
    north = origin[1] + np.arange(n)*res
    return c + a*east[None, :] + b*north[:, None]


def smooth_world(seed, n=1400, res=0.5):
    rng = np.random.default_rng(seed)
    a = 128 + 60*_blur(rng.standard_normal((n, n)), 6.0)/0.047
    return np.clip(a, 0, 255), res, (-(n - 1)/2*res, -(n - 1)/2*res)


def attitude(yaw, tx=0.0, ty=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ LEVEL @ exp_so3(np.array([tx, ty, 0.0]))


def shift_between(a, b, reach=3):
    """Sub-pixel (dy, dx) by which b must be moved to match a, from the cross-correlation peak."""
    a, b = a - a.mean(), b - b.mean()
    best = {}
    for dy in range(-reach, reach + 1):
        for dx in range(-reach, reach + 1):
            aa = a[reach + dy:a.shape[0] - reach + dy, reach + dx:a.shape[1] - reach + dx]
            bb = b[reach:b.shape[0] - reach, reach:b.shape[1] - reach]
            best[dy, dx] = float((aa*bb).sum())
    (py, px) = max(best, key=best.get)
    out = []
    for axis, peak in ((0, py), (1, px)):
        lo = best.get((py - 1, px) if axis == 0 else (py, px - 1))
        hi = best.get((py + 1, px) if axis == 0 else (py, px + 1))
        mid = best[py, px]
        out.append(peak + 0.5*(lo - hi)/(lo - 2*mid + hi))
    return out


class PyramidChecks(unittest.TestCase):
    def test_every_level_keeps_the_georeference(self):
        res, origin = 0.5, (-37.25, 12.5)
        raster = ramp(256, res, origin)
        pyramid = scene.Pyramid(raster, res, origin)
        self.assertGreaterEqual(len(pyramid.levels), 4)
        rng = np.random.default_rng(0)
        east, north = rng.uniform(-20, 60, 400), rng.uniform(30, 110, 400)
        expect = 100.0 + 0.31*east - 0.17*north
        for footprint in (0.3, 0.5, 0.7, 1.0, 1.7, 2.0, 3.3, 4.0, 8.0):
            value, inside = pyramid.sample(east, north, footprint)
            self.assertTrue(inside.all())
            np.testing.assert_allclose(value, expect, atol=2e-3)     # a linear field survives box averaging exactly

    def test_a_large_footprint_averages_fine_texture_away(self):
        n = 256
        board = 100.0 + 50.0*((np.indices((n, n)).sum(axis=0) % 2)*2 - 1)
        pyramid = scene.Pyramid(board, 0.5, (0.0, 0.0))
        east, north = np.full(50, 40.0), np.linspace(20.0, 60.0, 50)
        fine, _ = pyramid.sample(east, north, 0.5)
        coarse, _ = pyramid.sample(east, north, 4.0)
        self.assertGreater(float(np.ptp(fine)), 50.0)
        self.assertLess(float(np.abs(coarse - 100.0).max()), 1e-3)

    def test_outside_the_world_is_flagged_and_filled(self):
        pyramid = scene.Pyramid(ramp(64, 1.0, (0.0, 0.0)), 1.0, (0.0, 0.0))
        value, inside = pyramid.sample(np.array([-5.0, 10.0, 80.0]), np.array([10.0, 10.0, 10.0]), 1.0)
        self.assertEqual(inside.tolist(), [False, True, False])
        self.assertTrue(np.all(np.isfinite(value)))


class RenderChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raster, cls.res, cls.origin = smooth_world(4)
        cls.pyramid = scene.Pyramid(cls.raster, cls.res, cls.origin)

    def test_a_pixel_shows_the_ground_the_flight_camera_model_says_it_does(self):
        cam = bin_camera(FULL, 4)
        p, r = np.array([12.0, -20.0, 160.0]), attitude(0.8, 0.07, -0.05)
        image, seen = scene.render(self.pyramid, cam, p, r, (272, 364))
        self.assertEqual(seen, 1.0)
        rng = np.random.default_rng(1)
        points = np.column_stack((rng.uniform(-60, 80, 200), rng.uniform(-90, 50, 200), np.zeros(200)))
        pix, depth = cam.project(points, p, r)
        inside = (pix[:, 0] > 2) & (pix[:, 0] < 361) & (pix[:, 1] > 2) & (pix[:, 1] < 269)
        self.assertGreater(inside.sum(), 100)
        u, v = np.rint(pix[inside, 0]).astype(int), np.rint(pix[inside, 1]).astype(int)
        # What the world holds under each of those pixel centres, at that pixel's footprint.
        centre = p + r @ cam.lever
        rays = cam.ray(np.column_stack((u, v)).astype(float)) @ r.T
        reach = centre[2]/-rays[:, 2]
        expect, _ = self.pyramid.sample(centre[0] + rays[:, 0]*reach, centre[1] + rays[:, 1]*reach, reach/cam.fx)
        np.testing.assert_allclose(image[v, u], expect, atol=1e-3)
        # And those ground points really are the ones the projection was asked about, to within a pixel footprint.
        back = np.column_stack((centre[0] + rays[:, 0]*reach, centre[1] + rays[:, 1]*reach))
        self.assertLess(float(np.abs(back - points[inside, :2]).max()), 0.75*float((reach/cam.fx).max()))

    def test_rendering_binned_equals_binning_the_render(self):
        # The convention that everything downstream rests on: a b x b block mean of an image is the image
        # of bin_camera(camera, b). Getting it wrong would shift the image by (b - 1)/(2 b) = 0.25 px; the
        # two renderings agree to 0.05 px, the rest being the pyramid's approximation of a pixel footprint.
        p, r = np.array([-8.0, 15.0, 150.0]), attitude(-2.1, -0.04, 0.06)
        half = bin_camera(FULL, 2)
        direct, _ = scene.render(self.pyramid, bin_camera(half, 2), p, r, (272, 364))
        binned = bin_image(scene.render(self.pyramid, half, p, r, (544, 728))[0], 2)
        self.assertEqual(direct.shape, binned.shape)
        dy, dx = shift_between(direct, binned)
        self.assertLess(abs(dy), 0.05)
        self.assertLess(abs(dx), 0.05)
        self.assertGreater(float(np.corrcoef(direct.ravel(), binned.ravel())[0, 1]), 0.995)

    def test_noise_blur_gain_and_quantisation(self):
        cam = bin_camera(FULL, 4)
        p, r = np.array([0.0, 0.0, 150.0]), LEVEL
        clean, _ = scene.render(self.pyramid, cam, p, r, (272, 364))
        again, _ = scene.render(self.pyramid, cam, p, r, (272, 364))
        np.testing.assert_array_equal(clean, again)
        noisy, _ = scene.render(self.pyramid, cam, p, r, (272, 364), noise=3.0, rng=np.random.default_rng(5))
        self.assertAlmostEqual(float((noisy - clean).std()), 3.0, delta=0.1)
        scaled, _ = scene.render(self.pyramid, cam, p, r, (272, 364), gain=0.5, offset=10.0)
        np.testing.assert_allclose(scaled, 0.5*clean + 10.0)
        blurred, _ = scene.render(self.pyramid, cam, p, r, (272, 364), blur_px=2.0)
        self.assertLess(float(blurred.std()), float(clean.std()))
        self.assertAlmostEqual(float(blurred.mean()), float(clean.mean()), delta=0.05)
        whole, _ = scene.render(self.pyramid, cam, p, r, (272, 364), quantise=True)
        self.assertTrue(np.all(whole == np.rint(whole)) and whole.min() >= 0 and whole.max() <= 255)

    def test_a_view_that_leaves_the_world_reports_it(self):
        cam = bin_camera(FULL, 4)
        image, seen = scene.render(self.pyramid, cam, np.array([330.0, 0.0, 300.0]), LEVEL, (272, 364))
        self.assertLess(seen, 0.9)
        self.assertTrue(np.all(np.isfinite(image)))

    def test_raised_ground_is_displaced_away_from_the_nadir(self):
        # A 20 m high plateau seen off to one side appears further out than its footprint on the plane.
        n, res = 800, 0.5
        origin = (-(n - 1)/2*res, -(n - 1)/2*res)
        east = origin[0] + np.arange(n)*res
        tone = np.where(east[None, :] > 60.0, 200.0, 60.0)*np.ones((n, 1))
        height = np.where(east[None, :] > 60.0, 20.0, 0.0)*np.ones((n, 1))
        cam = bin_camera(FULL, 4)
        p = np.array([0.0, 0.0, 150.0])
        flat, _ = scene.render(scene.Pyramid(tone, res, origin), cam, p, LEVEL, (272, 364))
        raised, _ = scene.render(scene.Pyramid(tone, res, origin), cam, p, LEVEL, (272, 364),
                                 height_pyramid=scene.Pyramid(height, res, origin))
        edge = lambda im: float(np.argmax(np.abs(np.diff(im[136]))))      # noqa: E731
        step_flat, step_raised = edge(flat), edge(raised)
        # tan(angle) = 60/150 on the plane and 60/130 at the top of the step: 181.2/4*(60/130 - 60/150) px further out.
        self.assertAlmostEqual(step_raised - step_flat, 181.16*(60/130 - 60/150), delta=1.5)


class MapChecks(unittest.TestCase):
    def test_onboard_map_keeps_the_georeference_and_applies_the_datum(self):
        res, origin = 0.5, (-100.25, 40.25)
        raster = ramp(400, res, origin)
        for res2 in (1.0, 2.0):
            m, r, o = scene.onboard_map(raster, res, origin, res2=res2)
            self.assertEqual(r, res2)
            self.assertEqual(m.shape[0], int(400*res/res2))
            east, north = o[0] + 7*res2, o[1] + 3*res2
            self.assertAlmostEqual(m[3, 7], 100.0 + 0.31*east - 0.17*north, places=9)
        _, _, shifted = scene.onboard_map(raster, res, origin, res2=2.0, datum=(3.0, -1.5))
        _, _, plain = scene.onboard_map(raster, res, origin, res2=2.0)
        self.assertAlmostEqual(shifted[0] - plain[0], 3.0)
        self.assertAlmostEqual(shifted[1] - plain[1], -1.5)
        with self.assertRaises(ValueError):
            scene.onboard_map(raster, res, origin, res2=0.75)

    def test_stub_world_is_reproducible_and_shares_its_boundaries(self):
        a = scene.stub_world(2, size_m=200.0, res=1.0, cells=12)
        b = scene.stub_world(2, size_m=200.0, res=1.0, cells=12)
        np.testing.assert_array_equal(a[0], b[0])
        self.assertEqual(a[0].shape, (200, 200))
        self.assertAlmostEqual(a[3][0], -99.5)
        hedge = a[0] == 50.0
        self.assertTrue(0.02 < hedge.mean() < 0.4)
        np.testing.assert_array_equal(hedge, a[1] == 50.0)
        self.assertGreater(float(np.abs(a[0] - a[1])[~hedge].mean()), 10.0)


if __name__ == '__main__':
    unittest.main()
