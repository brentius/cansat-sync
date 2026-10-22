"""GPL-3.0-only. Fast checks of the SYNTHETIC NCC pose-error tolerance helpers.

Usage: python sim/nav/test_ncc_attitude_tolerance.py
Small binned cameras keep the whole file to a few seconds; nothing here is a flight result.
"""
import math
import sys
import unittest
from pathlib import Path
import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ncc_attitude_tolerance as nat  # noqa: E402  (also puts sim/maths on sys.path)
from feasibility import rectify_patch, render_image  # noqa: E402

QUARTER = {'name': 'IMX296 binned 4x4 (test only)', 'size': (272, 364), 'focal_px': 181.25}
HALF = {'name': 'IMX296 binned 2x2 (test only)', 'size': (544, 728), 'focal_px': 362.5}


def frame(name, camera, height, index, kind=None, value=0.):
    cfg = nat.CONFIGS[name]
    scene = nat.make_scene(nat.SEED, height, index, camera)
    image = nat.scene_image(scene, camera, cfg.get('texture_stretch', 1.))
    true = (scene['pos'], scene['r'])
    return nat.evaluate(image, true, nat.predicted_pose(scene, kind, value) if kind else true, cfg, camera)


class ToleranceChecks(unittest.TestCase):
    def setUp(self):
        self.pos = np.array([40., -25., 120.])
        self.r = nat.attitude(math.radians(63.), math.radians(8.), math.radians(200.))
        self.pred_pos = self.pos+np.array([3., -2., 6.])
        self.pred_r = nat.rotation([.03, -.02, .2])@self.r

    def test_pixel_ground_inverse(self):
        rng = np.random.default_rng(3)
        xy = self.pos[:2]+rng.uniform(-80., 80., (50, 2))
        uv = nat.ground_to_pixel(xy, self.pos, self.r, 725., (1088, 1456))
        np.testing.assert_allclose(nat.pixel_to_ground(uv, self.pos, self.r, 725., (1088, 1456)), xy, atol=1e-9)
        back = nat.ground_to_pixel(nat.pixel_to_ground(uv, self.pos, self.r, 725., (1088, 1456)),
                                   self.pos, self.r, 725., (1088, 1456))
        np.testing.assert_allclose(back, uv, atol=1e-9)
        # The attitude helper tilts by exactly the requested angle and points down at zero tilt.
        self.assertAlmostEqual(math.degrees(math.acos(-self.r[2, 2])), 8., places=10)
        np.testing.assert_allclose(nat.attitude(0., 0., 1.), nat.NADIR, atol=1e-15)
        # Rays at or above the horizon are NaN, not mirrored through the camera.
        sky = nat.pixel_to_ground([[728., 544.]], self.pos, np.eye(3), 725., (1088, 1456))
        self.assertTrue(np.all(np.isnan(sky)))
        behind = nat.ground_to_pixel([[0., 0.]], self.pos, np.eye(3), 725., (1088, 1456))
        self.assertTrue(np.all(np.isnan(behind)))

    def test_map_patch_pixel_round_trip(self):
        """map point -> rectified patch coordinate -> original pixel, self-consistent to <1e-9 px."""
        size, scale, focal, shape = 96, .5, 362.5, (544, 728)
        centre = self.pos[:2]+np.array([12.3, -7.7])
        rng = np.random.default_rng(4)
        points = centre+rng.uniform(-20., 20., (40, 2))
        coords = nat.map_to_patch(points, centre, size, scale)
        direct = nat.ground_to_pixel(points, self.pred_pos, self.pred_r, focal, shape)
        via = nat.ground_to_pixel(nat.patch_to_map(coords, centre, size, scale), self.pred_pos, self.pred_r, focal, shape)
        self.assertLess(np.max(np.abs(via-direct)), 1e-9)
        self.assertEqual(tuple(nat.map_to_patch(centre, centre, size, scale)), ((size-1)/2, (size-1)/2))
        # Rectifying an image that stores its own column/row index must return, at every patch
        # index, the original pixel that the same predicted pose assigns to that patch index.
        vv, uu = np.indices(shape).astype(float)
        index = np.stack(np.meshgrid(np.arange(size), np.arange(size)), axis=-1).reshape(-1, 2)
        expected = nat.ground_to_pixel(nat.patch_to_map(index, centre, size, scale), self.pred_pos, self.pred_r,
                                       focal, shape).reshape(size, size, 2)
        self.assertTrue(np.all((expected >= 0) & (expected < [shape[1]-1, shape[0]-1])))
        for axis, ramp in enumerate((uu, vv)):
            got = nat.rectify(ramp, centre, self.pred_pos, self.pred_r, size, scale, focal)
            self.assertLess(np.max(np.abs(got-expected[..., axis])), 1e-9)

    def test_rectify_equals_existing_helper_for_other_image_sizes(self):
        image = render_image(self.pos, self.r, size=(272, 364), focal=181.25)
        centre = np.r_[self.pos[:2]+np.array([5., -9.]), 0.]
        got = nat.rectify(image, centre, self.pred_pos, self.pred_r, 48, .5, 181.25)
        old = rectify_patch(image, centre, self.pred_pos, self.pred_r, size=48, scale=.5, focal=181.25)
        self.assertTrue(np.all(np.isfinite(got)))
        np.testing.assert_array_equal(got, old)

    def test_zero_pose_error_matches_everything(self):
        for name, camera, height in (('FINE', QUARTER, 80.), ('COARSE', HALF, 300.), ('COARSE_TEX4', HALF, 300.)):
            records = frame(name, camera, height, 0)+frame(name, camera, height, 1)
            self.assertEqual(len(records), 18)
            self.assertTrue(all(x['in_view'] and x['truth_in_window'] for x in records), name)
            self.assertTrue(all(x['correct'] for x in records), name)
            self.assertLess(max(x['ground_error_m'] for x in records), .2*nat.CONFIGS[name]['scale_m'], name)
            self.assertLess(max(np.max(np.abs(x['ideal_shift'])) for x in records), 1e-9, name)
            cell = nat.summarise(records)
            self.assertEqual((cell['correct_frac'], cell['false_frac'], cell['accepted_frac']), (1., 0., 1.))

    def test_known_position_error_is_recovered_in_both_layouts(self):
        """The reported shift equals the truth-derived ideal shift, so signs and offsets agree."""
        for name, camera, height, error in (('FINE', QUARTER, 80., 6.), ('COARSE_TEX4', HALF, 300., 21.)):
            records = frame(name, camera, height, 2, 'position', error)
            self.assertTrue(all(x['correct'] for x in records), name)
            for x in records:
                self.assertAlmostEqual(float(np.linalg.norm(x['ideal_shift'])), error, places=9)
                self.assertLess(np.linalg.norm(x['shift']-x['ideal_shift']), .25*nat.CONFIGS[name]['scale_m'])
        # Beyond the search window the true offset is unreachable, so nothing can be correct.
        records = frame('FINE', QUARTER, 80., 2, 'position', 24.)
        self.assertFalse(any(x['truth_in_window'] or x['correct'] for x in records))

    def test_yaw_180_gives_no_correct_accepted_match(self):
        """Fixed seeds. Accepted matches do occur here (a sinusoid equals its own 180 deg rotation
        up to a shift); every one of them must be scored as false."""
        records = sum((frame('FINE', QUARTER, 80., k, 'yaw', 180.) for k in range(4)), [])
        records += sum((frame('COARSE_TEX4', HALF, 300., k, 'yaw', 180.) for k in range(2)), [])
        self.assertEqual(len(records), 54)
        self.assertFalse(any(x.get('correct') for x in records))
        cell = nat.summarise(records)
        self.assertEqual(cell['correct'], 0)
        self.assertEqual(cell['false'], cell['accepted'])

    def test_thresholds(self):
        got = nat.thresholds([0, 1, 2, 5], [1., .95, .7, .1])
        self.assertEqual(got['tol90_tested'], 1.)
        self.assertAlmostEqual(got['e90_interp'], 1.2)
        self.assertAlmostEqual(got['e50_interp'], 3.)
        self.assertFalse(got['censored90'] or got['censored50'])
        got = nat.thresholds([0, 1, 2, 5], [1., .95, None, .1])     # too few samples cuts the curve
        self.assertEqual((got['tol90_tested'], got['tested_to'], got['e50_interp']), (1., 1., None))
        self.assertTrue(got['censored90'] and got['censored50'])
        got = nat.thresholds([0, 1, 2], [.8, 1., 1.])               # must hold from zero upwards
        self.assertEqual((got['tol90_tested'], got['e90_interp'], got['censored90']), (None, None, False))
        got = nat.thresholds([0, 1, 2], [1., .2, .95])              # no credit after the first failure
        self.assertEqual(got['tol90_tested'], 0.)

    def test_correlation_length_and_smear_prediction(self):
        acf = nat.autocorrelation(.5, n=512, max_lag=64)
        self.assertAlmostEqual(acf[64, 64], 1.)
        np.testing.assert_allclose(acf, acf[::-1, ::-1], atol=1e-9)
        length = nat.correlation_length(acf)*.5
        finer = nat.correlation_length(nat.autocorrelation(.25, n=512, max_lag=64))*.25
        self.assertTrue(2. < length.mean() < 4.)
        self.assertLess(abs(length.mean()-finer.mean()), .1*length.mean())
        # The autocorrelation-smear prediction agrees with a direct pure-rotation measurement
        # when the template spans many texture wavelengths (COARSE: 128 m).
        rng = np.random.default_rng(6)
        for value in (2., 5.):
            warp = nat.warp_matrix('yaw', value)
            predicted = nat.smear_correlation(acf, .5, 64, 2., warp)
            measured, _ = nat.pure_warp_ncc(64, 2., warp, rng, count=30)
            self.assertLess(abs(predicted-measured), .05)
        self.assertAlmostEqual(nat.smear_correlation(acf, .5, 32, .5, np.eye(2)), 1.)
        self.assertIsNone(nat.smear_correlation(acf, .5, 64, 2., nat.warp_matrix('yaw', 90.)))


if __name__ == '__main__':
    unittest.main(verbosity=2)
