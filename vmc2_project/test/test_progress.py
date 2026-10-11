import unittest
import numpy as np
from perception import analyze_edge, search_traverse_trajectories, search_with_fallback


def corridor():
    x = np.arange(.1, 1.4, .012)
    points = np.array([(a, b) for a in x for b in (-.35, .35)])
    return analyze_edge(points, (0., 0.), (1.424, 0.))


class ProgressTests(unittest.TestCase):
    def test_near_opening_produces_short_forward_trajectory(self):
        result = search_traverse_trajectories(corridor(), (.70, 0.), robot_yaw=0.)
        self.assertTrue(result.feasible, [r.reason for r in result.rejected[:5]])
        path = np.asarray(result.feasible[0].path_st)
        self.assertGreater(path[-1, 0], path[0, 0])
        self.assertLess(path[-1, 0], 1.424)
        self.assertAlmostEqual(path[1, 1]-path[0, 1], 0., delta=.001)

    def test_far_away_does_not_use_traverse(self):
        result = search_traverse_trajectories(corridor(), (-.5, 0.), robot_yaw=0.)
        self.assertFalse(result.feasible)
        self.assertIn('No observed opening near', result.rejected[0].reason)

    def test_facing_away_requires_alignment(self):
        result = search_traverse_trajectories(corridor(), (.7, 0.), robot_yaw=np.pi)
        self.assertFalse(result.feasible)
        self.assertIn('align first', result.rejected[0].reason)

    def test_combined_search_uses_local_traverse(self):
        result, mode = search_with_fallback(corridor(), (.7, 0.), robot_yaw=0.)
        self.assertTrue(result.feasible)
        self.assertEqual(mode, 'direct destination')

if __name__ == '__main__':
    unittest.main()
