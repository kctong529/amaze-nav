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

    def test_heading_aligned_passes(self):
        obs = analyze_edge(corridor_points(), START, END)
        result = plan_candidate_trajectory(obs, START, robot_yaw=0.0)
        self.assertTrue(result.feasible, result.reason)

    def test_heading_opposite_rejected(self):
        obs = analyze_edge(corridor_points(), START, END)
        result = plan_candidate_trajectory(obs, START, robot_yaw=np.pi)
        self.assertFalse(result.feasible)
        self.assertIn("align first", result.reason)

    def test_heading_rotated_edge(self):
        # Same east-facing corridor geometry rotated 90 degrees into +Y.
        points = corridor_points()
        rotated = np.column_stack((-points[:, 1], points[:, 0]))
        obs = analyze_edge(rotated, (0., 0.), (0., 1.424))
        good = plan_candidate_trajectory(obs, (0., 0.), robot_yaw=np.pi/2)
        bad = plan_candidate_trajectory(obs, (0., 0.), robot_yaw=0.0)
        self.assertTrue(good.feasible, good.reason)
        self.assertFalse(bad.feasible)

    def test_curve_tangent_matches_robot_yaw(self):
        obs = analyze_edge(corridor_points(), START, END)
        yaw = np.deg2rad(12.0)
        result = plan_candidate_trajectory(obs, START, robot_yaw=yaw)
        self.assertTrue(result.feasible, result.reason)
        path = np.asarray(result.path_st)
        first_step = path[1] - path[0]
        actual_yaw = np.arctan2(first_step[1], first_step[0])
        self.assertAlmostEqual(actual_yaw, yaw, delta=np.deg2rad(1.0))

    def test_heading_tangent_on_rotated_edge(self):
        points = corridor_points()
        rotated = np.column_stack((-points[:, 1], points[:, 0]))
        obs = analyze_edge(rotated, (0., 0.), (0., 1.424))
        yaw = np.deg2rad(80)
        result = plan_candidate_trajectory(obs, (0., 0.), robot_yaw=yaw)
        self.assertTrue(result.feasible, result.reason)
        path = np.asarray(result.path_st)
        delta = path[1] - path[0]
        # +s is +world Y; +t is -world X.
        world_delta = np.array([-delta[1], delta[0]])
        actual_yaw = np.arctan2(world_delta[1], world_delta[0])
        self.assertAlmostEqual(actual_yaw, yaw, delta=np.deg2rad(1.0))

    def test_heading_invalid_rejected(self):
        obs = analyze_edge(corridor_points(), START, END)
        with self.assertRaises(ValueError):
            evaluate_trajectory(obs, START, GapCandidate(0.65, 0, 0.72, 0.65), robot_yaw=float('nan'))


if __name__ == '__main__':
    unittest.main()
