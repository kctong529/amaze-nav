import unittest
import numpy as np
from perception import analyze_edge, search_approach_trajectories, search_with_fallback


def corridor():
    xs = np.arange(.15, 1.40, .015)
    return analyze_edge(np.array([(x, y) for x in xs for y in (-.35, .35)]),
                        (0., 0.), (1.424, 0.))


class ApproachTests(unittest.TestCase):
    def test_short_approach_ends_before_node_and_has_heading_tangent(self):
        obs = corridor()
        result = search_approach_trajectories(obs, (0., 0.), robot_yaw=.15)
        self.assertTrue(result.feasible, [r.reason for r in result.rejected[:3]])
        path = np.asarray(result.feasible[0].path_st)
        self.assertLess(path[-1, 0], obs.length - .05)
        self.assertGreater(path[-1, 0], 0.2)
        self.assertAlmostEqual(float(np.arctan2(*(path[1]-path[0])[::-1])), .15, delta=.03)

    def test_wrong_heading_needs_alignment(self):
        result = search_approach_trajectories(corridor(), (0., 0.), robot_yaw=np.pi)
        self.assertFalse(result.feasible)
        self.assertIn('align first', result.rejected[0].reason)

    def test_search_returns_mode(self):
        _, mode = search_with_fallback(corridor(), (0., 0.), robot_yaw=0.)
        self.assertIn(mode, ('single-gap full-edge', 'short approach only',
                             'two-waypoint full-edge fallback', 'near-opening traverse only', 'next opening only', 'direct destination', 'smooth destination', 'rotate then destination'))

if __name__ == '__main__': unittest.main()
