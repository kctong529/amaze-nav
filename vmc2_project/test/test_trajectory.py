import unittest
import numpy as np
from perception import (
    analyze_edge, find_feasible_gap_candidates,
    evaluate_trajectory, plan_candidate_trajectory,
    choose_avoidance_target, GapCandidate,
)

START = (0.0, 0.0)
END = (1.424, 0.0)


def corridor_points(left=-0.36, right=0.36):
    xs = np.arange(0.15, 1.36, 0.015)
    return np.array([(x, t) for x in xs for t in (left, right)])


class TrajectoryTests(unittest.TestCase):
    def test_open_corridor_has_feasible_candidate(self):
        obs = analyze_edge(corridor_points(), START, END)
        self.assertGreater(len(find_feasible_gap_candidates(obs)), 0)
        result = plan_candidate_trajectory(obs, START)
        self.assertTrue(result.feasible, result.reason)
        self.assertGreaterEqual(result.min_observed_clearance, 0.129)

    def test_too_narrow_gaps_have_no_candidates(self):
        obs = analyze_edge(corridor_points(-0.10, 0.10), START, END)
        self.assertEqual(find_feasible_gap_candidates(obs), [])
        result = plan_candidate_trajectory(obs, START)
        self.assertFalse(result.feasible)

    def test_curve_near_obstacle_is_rejected(self):
        # A middle return invalidates a straight centerline candidate.
        points = np.vstack([corridor_points(), np.array([[0.7, 0.0]])])
        obs = analyze_edge(points, START, END)
        candidate = GapCandidate(s=0.65, t=0.0, width=0.72, slice_s=0.65)
        result = evaluate_trajectory(obs, START, candidate)
        self.assertFalse(result.feasible)
        self.assertIn('close', result.reason)

    def test_out_of_order_candidate_rejected(self):
        obs = analyze_edge(corridor_points(), START, END)
        result = evaluate_trajectory(obs, (0.75, 0.0), GapCandidate(0.4, 0, 0.6, 0.4))
        self.assertFalse(result.feasible)

    def test_lookahead_target(self):
        obs = analyze_edge(corridor_points(), START, END)
        result = choose_avoidance_target(obs, START, END)
        self.assertIsNotNone(result.target, result.reason)
        self.assertFalse(result.clearance_verified)
        self.assertGreater(result.target[0], 0)
        self.assertLess(result.target[0], END[0])

    def test_no_observations_no_trajectory(self):
        obs = analyze_edge(np.empty((0, 2)), START, END)
        self.assertFalse(plan_candidate_trajectory(obs, START).feasible)


if __name__ == '__main__':
    unittest.main()
