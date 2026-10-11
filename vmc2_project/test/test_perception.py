import unittest
import numpy as np
from perception import (
    Passability, analyze_edge, assess_passability, choose_avoidance_target,
    edge_to_world, points_to_edge_frame, cluster_lateral_points, find_gaps,
)

A = (0.712, -2.848)
B = (2.136, -2.848)


class PerceptionTests(unittest.TestCase):
    def test_coordinate_roundtrip(self):
        points = np.array([A, (1.424, -2.648), B])
        st = points_to_edge_frame(points, A, B)
        np.testing.assert_allclose(st, [[0, 0], [.712, .2], [1.424, 0]], atol=1e-9)
        for p, (s, t) in zip(points, st):
            np.testing.assert_allclose(edge_to_world(s, t, A, B), p)

    def test_two_clusters(self):
        points = np.array([[.55, t] for t in [-.42, -.40, -.38, .18, .20, .22]])
        intervals = cluster_lateral_points(points)
        gaps = find_gaps(intervals)
        self.assertEqual(len(gaps), 1)
        self.assertAlmostEqual(gaps[0].width, .56)
        self.assertAlmostEqual(gaps[0].midpoint, -.10)

    def test_empty_and_single_cluster(self):
        self.assertEqual(cluster_lateral_points(np.empty((0, 2))), [])
        self.assertEqual(find_gaps([(-.2, -.1)]), [])

    def test_shifted_opening_and_edge_analysis(self):
        # Synthetic observations at s=0.55, expressed in eastbound world coords.
        st = np.array([[.55, t] for t in [-.42, -.40, -.38, .08, .10, .12]])
        world = st + np.array(A)
        observation = analyze_edge(world, A, B)
        slice_055 = min(observation.slices, key=lambda entry: abs(entry.s - .55))
        self.assertEqual(len(slice_055.gaps), 1)
        self.assertAlmostEqual(slice_055.gaps[0].width, .46)
        self.assertAlmostEqual(slice_055.gaps[0].midpoint, -.15)
        self.assertEqual(assess_passability(observation, A).status, Passability.UNKNOWN)
        target = choose_avoidance_target(observation, A, B)
        self.assertIsNone(target.target)
        self.assertFalse(target.clearance_verified)

    def test_wrong_input_rejected(self):
        with self.assertRaises(ValueError):
            analyze_edge(np.zeros((1, 2)), A, A)
        with self.assertRaises(ValueError):
            analyze_edge(np.zeros((3,)), A, B)


if __name__ == '__main__':
    unittest.main()
