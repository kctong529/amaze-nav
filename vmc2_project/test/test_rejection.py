import unittest
import numpy as np
from perception import (
    Passability, analyze_edge, assess_passability,
    detect_bottlenecks,
)

A = (0.0, 0.0)
B = (1.4, 0.0)


class EarlyRejectionTests(unittest.TestCase):
    def test_without_known_walls_cannot_confirm_blocked(self):
        # A chain of points across the crossing, but not known wall limits.
        pts = np.array([(0.7, t) for t in np.arange(-0.45, 0.46, 0.15)])
        obs = analyze_edge(pts, A, B)
        result = assess_passability(obs, A)
        self.assertEqual(result.status, Passability.UNKNOWN)

    def test_connected_obstacle_disks_barrier_between_known_walls(self):
        pts = np.array([(0.7, t) for t in np.arange(-0.45, 0.46, 0.15)])
        obs = analyze_edge(pts, A, B)
        result = assess_passability(obs, A, corridor_bounds=(-0.5, 0.5))
        self.assertEqual(result.status, Passability.BLOCKED)

    def test_disconnected_obstacles_do_not_confirm_blocked(self):
        pts = np.array([(0.7, t) for t in (-0.42, -0.40, 0.40, 0.42)])
        obs = analyze_edge(pts, A, B)
        result = assess_passability(obs, A, corridor_bounds=(-0.5, 0.5))
        self.assertEqual(result.status, Passability.UNKNOWN)

    def test_empty_observations_are_unknown(self):
        obs = analyze_edge(np.empty((0, 2)), A, B)
        self.assertEqual(assess_passability(obs, A).status, Passability.UNKNOWN)

    def test_wrong_corridor_limits_rejected(self):
        obs = analyze_edge(np.empty((0, 2)), A, B)
        with self.assertRaises(ValueError):
            assess_passability(obs, A, corridor_bounds=(0.5, -0.5))


if __name__ == '__main__':
    unittest.main()
