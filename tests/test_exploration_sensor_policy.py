import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from exploration_sensor_policy import select_rays, self_mask, transform_points
from exploration_core import ExplorationCore, InputError
from test_exploration_core import observation, prior


class SensorPolicyTests(unittest.TestCase):
    def test_source_stamped_body_transform_masks_only_known_envelope(self):
        sensor = np.array([[0., 0., -0.05], [0.1812, 0., -0.05]])
        body = transform_points(sensor, (0., 0., .17), (0., 0., 0., 1.))
        self.assertEqual(self_mask(body).tolist(), [True, False])
        self.assertFalse(self_mask(np.array([[0., 0., .1212]]))[0])
        rotated = transform_points(np.array([[.0, .16, -.05]]), (0., 0., .17), (0., 0., .70710678, .70710678))
        self.assertFalse(self_mask(rotated)[0])

    def test_nearest_navigation_return_per_azimuth_and_visibility_extras(self):
        raw = np.array([
            [1., 0., .2], [2., 0., .2], [0., 1., .2], [0., 2., .2],
            [-1., 0., -.1], [0., -1., 2.], [0., 0., .12],
        ])
        body = raw.copy()
        mapped = raw.copy()
        selected, policy = select_rays(raw, body, mapped, 0., max_rays=6, azimuth_bins=4,
                                       floor_tolerance=.03, navigation_z_min=.03, navigation_z_max=1.5)
        values = {tuple(point) for point in selected}
        self.assertIn((1., 0., .2), values)
        self.assertIn((0., 1., .2), values)
        self.assertNotIn((2., 0., .2), values)
        self.assertNotIn((0., 2., .2), values)
        self.assertNotIn((0., 0., .12), values)
        self.assertEqual(policy['self_filtered_count'], 1)
        self.assertEqual(policy['navigation_bins_retained'], 2)

    def test_ground_reference_is_validated_without_changing_old_fixtures(self):
        record = observation('ground-reference', 1, [{'xyz': [1., 1., 0.], 'kind': 'hit'}])
        record['ground_reference_z'] = -.063
        ExplorationCore(prior()).apply_observation(record)
        record['id'], record['revision'], record['ground_reference_z'] = 'bad-ground', 2, float('nan')
        with self.assertRaisesRegex(InputError, 'ground reference'):
            ExplorationCore(prior()).apply_observation(record)


if __name__ == '__main__':
    unittest.main()
