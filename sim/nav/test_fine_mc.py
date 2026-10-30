"""Contract checks for the fine-matcher image-to-filter simulation."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / 'code'))
import run_fine_mc as fm  # noqa: E402


class FineMonteCarloContract(unittest.TestCase):
    def test_pixel_score_uses_only_accepted_original_image_residuals(self):
        score = fm.score_pixel_errors([[3.0, 4.0], [0.0, 2.0]])
        self.assertEqual(score['count'], 2)
        self.assertAlmostEqual(score['rmse_px'], (29.0 / 2.0) ** 0.5)
        self.assertAlmostEqual(score['median_px'], 3.5)
        self.assertEqual(score['false_matches_over_threshold'], 1)
        self.assertAlmostEqual(score['false_match_rate'], 0.5)
        self.assertIsNone(fm.score_pixel_errors([])['rmse_px'])

    def test_quarter_sensor_camera_matches_canonical_block_binning(self):
        from cansat.eskf import Camera
        from cansat.coarse import bin_camera
        expected = bin_camera(Camera(724.6, 724.6, 728.0, 544.0, 1456, 1088), 4)
        actual = fm._camera()
        self.assertEqual((actual.width, actual.height), (364, 272))
        self.assertAlmostEqual(actual.cx, 181.625)
        self.assertAlmostEqual(actual.cy, 135.625)
        self.assertEqual((actual.fx, actual.fy, actual.cx, actual.cy),
                         (expected.fx, expected.fy, expected.cx, expected.cy))

    def test_ground_false_match_rate_uses_fixed_map_resolution_threshold(self):
        score = fm.score_ground_errors([[0.3, 0.4], [0.8, 0.0]])
        self.assertEqual(score['false_match_threshold_m'], 0.75)
        self.assertEqual(score['false_matches_over_threshold'], 1)
        self.assertAlmostEqual(score['false_match_rate'], 0.5)

    def test_small_campaign_is_seed_deterministic_except_host_timings(self):
        with tempfile.TemporaryDirectory() as td:
            a = fm.run_campaign(trials=1, frames=2, seed=81, output=Path(td) / 'a')
            b = fm.run_campaign(trials=1, frames=2, seed=81, output=Path(td) / 'b')
            self.assertEqual(a['summary'], b['summary'])
            stable_a = [{k: v for k, v in row.items() if not k.endswith('_seconds')} for row in a['frames']]
            stable_b = [{k: v for k, v in row.items() if not k.endswith('_seconds')} for row in b['frames']]
            self.assertEqual(stable_a, stable_b)
            paired = {name: [r for r in a['frames'] if r['scenario'] == name]
                      for name in ('fresh', 'seasonal', 'textureless')}
            for name in ('seasonal', 'textureless'):
                self.assertEqual([r['baro_measurement_m'] for r in paired['fresh']],
                                 [r['baro_measurement_m'] for r in paired[name]])
                self.assertEqual([r['sensor_seed'] for r in paired['fresh']],
                                 [r['sensor_seed'] for r in paired[name]])

    def test_scenarios_share_paired_trial_and_map_seed(self):
        shared = [fm._paired_trial_seed(83, 2) for _ in fm.SCENARIOS]
        map_seeds = [seed ^ 0x5A17 for seed in shared]
        self.assertEqual(len(set(shared)), 1)
        self.assertEqual(len(set(map_seeds)), 1)

    def test_first_exposure_has_no_elapsed_imu_propagation(self):
        class Counter:
            def __init__(self):
                self.calls = 0

            def propagate(self, *args):
                self.calls += 1

        nav, control = Counter(), Counter()
        fm._propagate_elapsed(nav, control, 0)
        self.assertEqual((nav.calls, control.calls), (0, 0))
        fm._propagate_elapsed(nav, control, 1)
        self.assertEqual((nav.calls, control.calls), (20, 20))

    def test_cli_writes_finite_json_and_per_frame_csv(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / 'campaign'
            fm.main(['--trials', '1', '--frames', '2', '--seed', '82', '--output', str(out)])
            summary = json.loads((out / 'summary.json').read_text(encoding='utf-8'), parse_constant=self._reject_constant)
            self.assertEqual(summary['schema_version'], 1)
            self.assertEqual(summary['seed'], 82)
            self.assertTrue((out / 'frames.csv').is_file())
            rows = (out / 'frames.csv').read_text(encoding='utf-8').splitlines()
            self.assertEqual(len(rows), 1 + 2 * summary['scenario_count'])
            self.assertIn('trial_seed', rows[0])
            self.assertIn('map_seed', rows[0])
            self.assertIn('accepted', rows[0])

    @staticmethod
    def _reject_constant(value):
        raise AssertionError(f'non-finite JSON number {value}')


if __name__ == '__main__':
    unittest.main()
