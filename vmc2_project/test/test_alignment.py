import unittest
import numpy as np
from perception import (analyze_edge, search_intermediate_alignment,
                        AlignmentManeuver)

class AlignmentTests(unittest.TestCase):
    def test_invalid_yaw(self):
        obs = analyze_edge(np.empty((0, 2)), (0., 0.), (1., 0.))
        with self.assertRaises(ValueError):
            search_intermediate_alignment(obs, (0., 0.), robot_yaw=float('nan'))

    def test_no_returns(self):
        obs = analyze_edge(np.empty((0, 2)), (0., 0.), (1., 0.))
        self.assertIsNone(search_intermediate_alignment(obs, (0., 0.), robot_yaw=0.))

    def test_no_openings(self):
        obs = analyze_edge(np.array([[.4, .6], [.5, .6], [.6, .6]]),
                           (0., 0.), (1., 0.))
        self.assertIsNone(search_intermediate_alignment(obs, (0., 0.), robot_yaw=0.))

if __name__ == '__main__':
    unittest.main()
