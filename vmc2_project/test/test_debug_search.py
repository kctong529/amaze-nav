import unittest
import numpy as np
from perception import analyze_edge, search_candidate_trajectories, plan_candidate_trajectory


class DebugSearchTests(unittest.TestCase):
    def setUp(self):
        xs = np.linspace(0.2, 1.2, 101)
        points = np.array([(x, y) for x in xs for y in (-0.45, 0.45)])
        self.obs = analyze_edge(points, (0.0, 0.0), (1.424, 0.0))

    def test_collects_feasible_and_rejected_curves(self):
        result = search_candidate_trajectories(self.obs, (0.0, 0.0), robot_yaw=0.0)
        self.assertGreater(len(result.feasible), 0)
        self.assertGreater(len(result.rejected), 0)
        self.assertTrue(all(r.feasible for r in result.feasible))
        self.assertTrue(all(not r.feasible for r in result.rejected))
        self.assertTrue(all(r.path_st for r in result.rejected))

    def test_planner_uses_same_best_curve_as_debug_search(self):
        result = search_candidate_trajectories(self.obs, (0.0, 0.0), robot_yaw=0.0)
        chosen = plan_candidate_trajectory(self.obs, (0.0, 0.0), robot_yaw=0.0)
        self.assertEqual(chosen.path_st, result.feasible[0].path_st)

    def test_empty_scan_does_not_imply_passable(self):
        obs = analyze_edge(np.empty((0, 2)), (0., 0.), (1.424, 0.))
        result = search_candidate_trajectories(obs, (0., 0.), robot_yaw=0.)
        self.assertFalse(result.feasible)
        self.assertFalse(result.rejected)

    def test_initial_heading_tangent_retained(self):
        result = search_candidate_trajectories(self.obs, (0., 0.), robot_yaw=0.1)
        self.assertTrue(result.feasible)
        path = np.array(result.feasible[0].path_st)
        delta = path[1] - path[0]
        self.assertAlmostEqual(np.arctan2(delta[1], delta[0]), 0.1, delta=0.07)


if __name__ == '__main__':
    unittest.main()
