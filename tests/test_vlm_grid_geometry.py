import unittest

import numpy as np

from scripts.vlm_grid.geometry import FREE, OCCUPIED, UNKNOWN, astar, inflate_occupied, reachable_approach


class GeometryTests(unittest.TestCase):
    def test_unknown_cells_are_not_routable(self):
        grid = np.full((5, 5), UNKNOWN, dtype=np.uint8)
        grid[2, :3] = FREE
        route, reason = astar(grid, (0, 2), (4, 2))
        self.assertIsNone(route)
        self.assertEqual(reason, "goal_occupied_or_unknown")

    def test_inflation_blocks_robot_radius(self):
        grid = np.full((7, 7), FREE, dtype=np.uint8)
        grid[3, 3] = OCCUPIED
        inflated = inflate_occupied(grid, 1)
        self.assertEqual(inflated[3, 4], OCCUPIED)
        self.assertEqual(inflated[1, 1], FREE)

    def test_approach_is_not_target_centroid(self):
        grid = np.full((41, 41), FREE, dtype=np.uint8)
        result, reason = reachable_approach(grid, np.array([0.0, 0.0]), .1, [1.0, 1.0], [2.0, 2.0], .5)
        self.assertIsNone(reason)
        self.assertNotEqual(result["approach_xy_m"], [2.0, 2.0])


if __name__ == "__main__":
    unittest.main()
