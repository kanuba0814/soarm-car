import unittest

import numpy as np

from scripts.vlm_grid.geometry import FREE, OCCUPIED, UNKNOWN, bfs_shortest_path, build_grid, configuration_space_grid, reachable_approach


class GeometryTests(unittest.TestCase):
    def test_unknown_cells_are_not_routable(self):
        grid = np.full((5, 5), UNKNOWN, dtype=np.uint8)
        grid[2, :3] = FREE
        route, reason = bfs_shortest_path(grid, (0, 2), (4, 2))
        self.assertIsNone(route)
        self.assertEqual(reason, "goal_footprint_occupied_or_unknown")

    def test_configuration_space_requires_full_free_disk(self):
        grid = np.full((7, 7), FREE, dtype=np.uint8)
        grid[3, 3] = OCCUPIED
        configured = configuration_space_grid(grid, 1)
        self.assertEqual(configured[3, 4], OCCUPIED)
        self.assertEqual(configured[1, 1], FREE)

    def test_unknown_boundary_blocks_footprint(self):
        grid = np.full((7, 7), UNKNOWN, dtype=np.uint8)
        grid[1:6, 1:6] = FREE
        configured = configuration_space_grid(grid, 1)
        self.assertEqual(configured[1, 3], UNKNOWN)
        self.assertEqual(configured[3, 3], FREE)
        route, reason = bfs_shortest_path(configured, (3, 3), (1, 3))
        self.assertIsNone(route)
        self.assertEqual(reason, "goal_footprint_occupied_or_unknown")

    def test_occupied_endpoint_wins_regardless_of_ray_order(self):
        endpoints = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        grid_one, origin_one = build_grid([0.0, 0.0], endpoints, [False, True], .5, .0)
        grid_two, origin_two = build_grid([0.0, 0.0], endpoints[::-1], [True, False], .5, .0)
        self.assertEqual(origin_one.tolist(), origin_two.tolist())
        cell = tuple(np.floor((np.array([1.0, 0.0]) - origin_one) / .5).astype(int))
        self.assertEqual(grid_one[cell[1], cell[0]], OCCUPIED)
        self.assertEqual(grid_two[cell[1], cell[0]], OCCUPIED)

    def test_approach_is_not_target_centroid(self):
        grid = np.full((41, 41), FREE, dtype=np.uint8)
        result, reason = reachable_approach(grid, np.array([0.0, 0.0]), .1, [1.0, 1.0], [2.0, 2.0], .5)
        self.assertIsNone(reason)
        self.assertNotEqual(result["approach_xy_m"], [2.0, 2.0])


if __name__ == "__main__":
    unittest.main()
