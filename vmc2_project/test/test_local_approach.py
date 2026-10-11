import unittest
import numpy as np
from perception import analyze_edge, search_approach_trajectories


def corridor():
    xs = np.arange(.15, 1.40, .015)
    return analyze_edge(np.array([(x,y) for x in xs for y in (-.35,.35)]),
                        (0.,0.), (1.424,0.))


class LocalApproachTests(unittest.TestCase):
    def test_goal_near_gap_not_full_edge(self):
        result = search_approach_trajectories(corridor(), (0.,0.), robot_yaw=.1)
        self.assertTrue(result.feasible, [r.reason for r in result.rejected[:5]])
        path = np.asarray(result.feasible[0].path_st)
        self.assertLess(path[-1,0], 1.30)
        self.assertAlmostEqual(np.arctan2(*(path[1]-path[0])[::-1]), .1, delta=.05)

    def test_sideways_approach_can_be_evaluated(self):
        result = search_approach_trajectories(corridor(), (0.,0.), robot_yaw=.5,
                                               max_curvature=12.)
        self.assertGreater(len(result.feasible)+len(result.rejected), 1)

    def test_backwards_requires_alignment(self):
        result = search_approach_trajectories(corridor(), (0.,0.), robot_yaw=np.pi)
        self.assertFalse(result.feasible)
        self.assertIn("align first", result.rejected[0].reason)

if __name__ == '__main__': unittest.main()
