"""GPL-3.0-only. Fine tracking checks on independently rendered synthetic images."""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'code'))
sys.path.insert(0, str(ROOT/'sim'/'nav'))
from cansat.eskf import Camera, Eskf, LEVEL, N_ERR, exp_so3
from cansat.fine import FineConfig, FineMatcher
from scene import Pyramid, render


def terrain():
    y, x = np.mgrid[-160:160:0.5, -160:160:0.5]
    # Spatially unique low-pass random texture. Three pure sinusoids produce
    # near-identical aliases inside the search window and should be refused.
    texture = np.random.default_rng(241).normal(size=x.shape)
    for _ in range(10):
        texture = (4*texture + np.roll(texture, 1, 0) + np.roll(texture, -1, 0)
                   + np.roll(texture, 1, 1) + np.roll(texture, -1, 1))/8
    return (120 + 24*texture/texture.std() + 5*np.sin(.29*x - .61*y)).astype(np.float32)


class FineTracking(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raster = terrain()
        cls.world = Pyramid(cls.raster, .5, (-160., -160.))
        x, y = np.meshgrid([-40., -15., 15., 40.], [-26., 0., 26.])
        cls.landmarks = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
        cls.camera = Camera(180., 173., 149.3, 123.7, 320, 240,
                            exp_so3(np.array([.01, -.015, .025])), np.array([.02, -.01, .07]))
        cls.p = np.array([2., -3., 105.])
        cls.r = LEVEL @ exp_so3(np.array([.04, -.06, .45]))

    def matcher(self, **kwargs):
        return FineMatcher(self.raster, .5, (-160., -160.), landmarks=self.landmarks, **kwargs)

    def image(self, p=None, r=None, **kwargs):
        return render(self.world, self.camera, self.p if p is None else p,
                      self.r if r is None else r, (240, 320), **kwargs)[0]

    def assert_pixels(self, result, tolerance=.6, minimum=6, p=None, r=None):
        self.assertGreaterEqual(len(result.points), minimum, result.rejections)
        truth, _ = self.camera.project(result.points, self.p if p is None else p,
                                       self.r if r is None else r)
        self.assertLess(np.max(np.linalg.norm(result.pixels - truth, axis=1)), tolerance)
        self.assertTrue(np.all(result.pixel_var > 0))
        self.assertTrue(np.isfinite(result.pixel_var).all())
        np.testing.assert_allclose(result.pixel_var, .5**2)
        self.assertTrue(np.isfinite(result.map_var).all())
        self.assertTrue((result.map_var >= 0).all())
        np.testing.assert_allclose(result.map_var, np.tile([.15**2, .15**2, 0.], (len(result.points), 1)))
        self.assertEqual(result.map_var.shape, (len(result.points), 3))
        self.assertEqual(result.pixels.shape, (len(result.points), 2))

    def test_recovers_pixel_from_translation_with_extrinsics_and_asymmetric_intrinsics(self):
        self.assert_pixels(self.matcher().match(self.image(), self.camera,
                                                self.p + [4., -3., 0.], self.r))

    def test_combined_small_attitude_height_and_position_error(self):
        result = self.matcher().match(self.image(), self.camera, self.p + [3., -2., 1.],
                                      self.r @ exp_so3(np.radians([.2, -.25, .7])))
        self.assert_pixels(result, tolerance=1., minimum=5)

    def test_gain_offset_and_noise_do_not_change_the_coordinate_contract(self):
        image = self.image(gain=1.3, offset=15., noise=.4, rng=np.random.default_rng(3))
        self.assert_pixels(self.matcher().match(image, self.camera, self.p, self.r))

    def test_filter_adapter_uses_exposure_pose_without_mutating_state(self):
        nav = Eskf(self.p, [5., -3., -10.], self.r, np.eye(N_ERR))
        nav.time_offset = .04
        nav.rate = np.array([.02, -.03, .25])
        p = nav.p + nav.v*nav.time_offset
        r = nav.r @ exp_so3(nav.rate*nav.time_offset)
        image = self.image(p, r)
        before = (nav.p.copy(), nav.r.copy(), nav.cov.copy(), image.copy())
        matcher = self.matcher()
        direct = matcher.match(image, self.camera, p, r)
        adapted = matcher.match_filter(image, self.camera, nav)
        self.assert_pixels(adapted, p=p, r=r)
        np.testing.assert_array_equal(adapted.pixels, direct.pixels)
        for old, now in zip(before, (nav.p, nav.r, nav.cov, image)):
            np.testing.assert_array_equal(old, now)

    def test_matches_reduce_pose_error_when_consumed_by_existing_filter(self):
        nav = Eskf(self.p + [2., -1.5, .3], np.zeros(3), self.r,
                   np.diag([9.]*3 + [1.]*3 + [math.radians(2.)**2]*3 + [.01]*8))
        before = np.linalg.norm(nav.p - self.p)
        result = self.matcher().match_filter(self.image(), self.camera, nav)
        info = nav.update_landmarks(self.camera, result.points, result.pixels, result.pixel_var,
                                    map_var=result.map_var, iterations=3, robust=True, recover=True)
        self.assertGreaterEqual(info['accepted'].sum(), 6)
        self.assertLess(np.linalg.norm(nav.p - self.p), before/3)
        self.assertTrue(np.isfinite(nav.cov).all())

    def test_filter_adapter_refuses_nonfinite_timing_and_rate_before_arithmetic(self):
        matcher = self.matcher()
        nav = Eskf(self.p, np.zeros(3), self.r, np.eye(N_ERR))
        for offset, rate in ((float('inf'), np.zeros(3)), (float('nan'), np.zeros(3)),
                             (0., np.array([float('inf'), 0., 0.]))):
            nav.time_offset, nav.rate = offset, rate
            with self.subTest(offset=offset), np.errstate(all='raise'):
                result = matcher.match_filter(self.image(), self.camera, nav)
                self.assertEqual(len(result.points), 0)
                self.assertTrue(result.reason)

    def test_textureless_image_and_map_refuse(self):
        matcher = self.matcher()
        result = matcher.match(np.full((240, 320), 120.), self.camera, self.p, self.r)
        self.assertEqual(len(result.points), 0)
        self.assertGreater(result.attempted, 0)
        flat = FineMatcher(np.ones((100, 100)), 1., (-50., -50.))
        self.assertEqual(len(flat.match(self.image(), self.camera, self.p, self.r).points), 0)

    def test_repetitive_map_has_no_accepted_unique_peak(self):
        y, x = np.mgrid[-160:160:.5, -160:160:.5]
        raster = 100 + 20*np.cos(2*np.pi*x/4) + 20*np.cos(2*np.pi*y/4)
        world = Pyramid(raster, .5, (-160., -160.))
        cam = Camera(200., 200., 160., 120., 320, 240)
        p = np.array([0., 0., 100.])
        image = render(world, cam, p, LEVEL, (240, 320))[0]
        matcher = FineMatcher(raster, .5, (-160., -160.), landmarks=self.landmarks)
        result = matcher.match(image, cam, p, LEVEL)
        self.assertEqual(len(result.points), 0)
        self.assertGreater(result.rejections.get('ambiguous', 0), 0)

    def test_straight_boundary_does_not_provide_a_two_axis_measurement(self):
        y, x = np.mgrid[-160:160:.5, -160:160:.5]
        raster = 100 + 30*np.tanh(x/2.)
        matcher = FineMatcher(raster, .5, (-160., -160.), landmarks=np.array([[0., 0., 0.]]))
        image = render(Pyramid(raster, .5, (-160., -160.)), self.camera, self.p, self.r, (240, 320))[0]
        self.assertEqual(len(matcher.match(image, self.camera, self.p, self.r).points), 0)

    def test_border_and_behind_camera_sites_refuse_with_empty_shapes(self):
        matcher = self.matcher()
        for p, r in ((self.p + [1000., 0., 0.], self.r), (self.p, np.eye(3))):
            result = matcher.match(self.image(), self.camera, p, r)
            self.assertEqual(result.points.shape, (0, 3))
            self.assertEqual(result.pixels.shape, (0, 2))
            self.assertEqual(result.map_var.shape, (0, 3))
            self.assertEqual(result.pixel_var.shape, (0,))

    def test_peak_on_search_boundary_refuses_instead_of_returning_clipped_correction(self):
        cam = Camera(200., 200., 160., 120., 320, 240)
        p = np.array([0., 0., 100.])
        image = render(self.world, cam, p, LEVEL, (240, 320))[0]
        matcher = FineMatcher(self.raster, .5, (-160., -160.), landmarks=np.array([[0., 0., 0.]]))
        # Exactly 16 m is the edge of the configured ground search. Its peak
        # cannot be refined using neighbours on both sides.
        result = matcher.match(image, cam, p + [16., 0., 0.], LEVEL)
        self.assertEqual(result.attempted, 1)
        self.assertEqual(len(result.points), 0)
        self.assertEqual(result.rejections.get('peak'), 1)

    def test_invalid_frames_and_poses_refuse_instead_of_raising(self):
        matcher = self.matcher()
        cases = [(np.full((240, 320), np.nan), self.p, self.r),
                 (np.ones((12, 12)), self.p, self.r),
                 (np.ones((240, 320, 3)), self.p, self.r),
                 (self.image(), [np.nan, 0., 100.], self.r),
                 (self.image(), self.p, np.zeros((3, 3)))]
        for image, p, r in cases:
            with self.subTest(shape=image.shape):
                result = matcher.match(image, self.camera, p, r)
                self.assertEqual(len(result.points), 0)
                self.assertTrue(result.reason)

    def test_preparation_rejects_bad_configuration_and_missing_map(self):
        for cfg in (replace(FineConfig(), template_px=128), replace(FineConfig(), search_radius_px=0),
                    replace(FineConfig(), max_landmarks=1000), replace(FineConfig(), resolution_m=float('nan')),
                    replace(FineConfig(), exclusion_px=100), replace(FineConfig(), pixel_sigma=0)):
            with self.subTest(config=cfg), self.assertRaises(ValueError):
                self.matcher(config=cfg)
        for raster, res in ((np.full((20, 20), np.nan), .5), (self.raster, 0)):
            with self.assertRaises(ValueError):
                FineMatcher(raster, res, (0., 0.))
        with self.assertRaises(ValueError):
            FineMatcher(self.raster, .5, (0., 0.), landmarks=np.zeros((10001, 3)))

    def test_spatial_selection_is_capped_and_preparation_owns_its_data(self):
        raster = self.raster.copy()
        cfg = replace(FineConfig(), max_landmarks=6)
        matcher = FineMatcher(raster, .5, (-160., -160.), config=cfg)
        image = self.image()
        a = matcher.match(image, self.camera, self.p, self.r)
        raster[:] = 0
        b = matcher.match(image, self.camera, self.p, self.r)
        self.assertGreater(len(a.points), 2)
        self.assertLessEqual(a.attempted, 6)
        np.testing.assert_array_equal(a.points, b.points)
        np.testing.assert_array_equal(a.pixels, b.pixels)
        uv, _ = self.camera.project(a.points, self.p, self.r)
        cells = np.floor(uv/[320/cfg.cells[0], 240/cfg.cells[1]]).astype(int)
        self.assertEqual(len(np.unique(cells, axis=0)), len(cells))


if __name__ == '__main__':
    unittest.main()
