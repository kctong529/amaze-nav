import unittest
import numpy as np
from perception import analyze_edge, find_all_feasible_trajectories, plan_candidate_trajectory


class AllTrajectoriesTests(unittest.TestCase):
    def setUp(self):
        a = (0.0, 0.0)
        b = (1.424, 0.0)
        # Stable synthetic parallel walls with an internal gap.
        xs = np.linspace(0.2, 1.2, 101)
        self.observation = analyze_edge(
            np.array([(x, side) for x in xs for side in (-0.45, 0.45)]), a, b
        )

    def test_every_returned_curve_passes_checks(self):
        results = find_all_feasible_trajectories(
            self.observation, (0.0, 0.0), robot_yaw=0.0,
        )
        self.assertGreater(len(results), 1)
        for result in results:
            self.assertTrue(result.feasible)
            self.assertGreaterEqual(result.min_observed_clearance, 0.129)
            self.assertLessEqual(result.max_curvature, 5.0)
            path = np.asarray(result.path_st)
            tangent = path[1] - path[0]
            self.assertAlmostEqual(float(np.arctan2(tangent[1], tangent[0])), 0.0, delta=0.07)

    def test_first_result_matches_selected_path(self):
        all_results = find_all_feasible_trajectories(self.observation, (0.0, 0.0), robot_yaw=0.0)
        selected = plan_candidate_trajectory(self.observation, (0.0, 0.0), robot_yaw=0.0)
        self.assertTrue(selected.feasible)
        self.assertAlmostEqual(all_results[0].min_observed_clearance, selected.min_observed_clearance, places=8)
        self.assertAlmostEqual(all_results[0].max_curvature, selected.max_curvature, places=8)

    def test_empty_observations_no_candidates(self):
        observation = analyze_edge(np.empty((0, 2)), (0.0, 0.0), (1.424, 0.0))
        self.assertEqual(find_all_feasible_trajectories(observation, (0.0, 0.0), robot_yaw=0.0), [])


if __name__ == '__main__':
    unittest.main()
