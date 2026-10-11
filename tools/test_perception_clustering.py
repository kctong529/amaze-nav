
import unittest

import numpy as np

from perception_geometry import (
    cluster_lateral_points,
    find_gaps,
)


class TestPerceptionClustering(unittest.TestCase):

    def test_two_obstacle_clusters(self):
        points = np.array([
            [0.70, -0.42],
            [0.71, -0.40],
            [0.72, -0.38],
            [0.70, 0.18],
            [0.71, 0.20],
            [0.72, 0.22],
        ])

        intervals = cluster_lateral_points(
            points,
            max_gap=0.05,
            min_points=3,
        )

        np.testing.assert_allclose(
            intervals,
            [
                (-0.42, -0.38),
                (0.18, 0.22),
            ],
        )

        gaps = find_gaps(intervals)

        self.assertEqual(len(gaps), 1)
        self.assertAlmostEqual(gaps[0]["width"], 0.56)
        self.assertAlmostEqual(gaps[0]["midpoint"], -0.10)

    def test_empty_slice(self):
        points = np.empty((0, 2))

        intervals = cluster_lateral_points(points)

        self.assertEqual(intervals, [])
        self.assertEqual(find_gaps(intervals), [])

    def test_single_obstacle(self):
        points = np.array([
            [0.70, -0.12],
            [0.71, -0.10],
            [0.72, -0.08],
        ])

        intervals = cluster_lateral_points(points)

        self.assertEqual(len(intervals), 1)
        self.assertEqual(find_gaps(intervals), [])

    def test_noise_rejection(self):
        points = np.array([
            [0.70, -0.40],
            [0.71, -0.38],
            [0.72, -0.36],
            [0.71, 0.00],  # Isolated return
            [0.70, 0.30],
            [0.71, 0.32],
            [0.72, 0.34],
        ])

        intervals = cluster_lateral_points(
            points,
            min_points=3,
        )

        self.assertEqual(len(intervals), 2)


if __name__ == "__main__":
    unittest.main()
