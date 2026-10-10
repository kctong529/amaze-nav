import unittest

import numpy as np

from perception_geometry import points_to_edge_frame


class TestEdgeFrame(unittest.TestCase):
    def test_eastbound_edge(self):
        start = (0.712, -2.848)
        end = (2.136, -2.848)

        points = np.array([
            [0.712, -2.848],  # Start
            [1.424, -2.848],  # Halfway
            [2.136, -2.848],  # End
            [1.424, -2.648],  # 0.2 m left
            [1.424, -3.048],  # 0.2 m right
        ])

        result = points_to_edge_frame(points, start, end)

        expected = np.array([
            [0.000, 0.000],
            [0.712, 0.000],
            [1.424, 0.000],
            [0.712, 0.200],
            [0.712, -0.200],
        ])

        np.testing.assert_allclose(result, expected, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
