import math
import unittest
from types import SimpleNamespace

from navigator import GridPathFollowerNode


class TestPurePursuit(unittest.TestCase):

    def setUp(self):
        # Avoid constructing a ROS node.
        self.controller = GridPathFollowerNode.__new__(
            GridPathFollowerNode
        )

        self.controller.v_max = 0.2
        self.controller.w_max = 2.0
        self.controller.v_average = 0.1

    def control(self, pos, yaw, target):
        return self.controller.turtlebot_control(
            pos,
            yaw,
            target,
            dt=0.05,
            Ld=0.25,
        )

    def test_straight_ahead(self):
        v, w = self.control(
            (0.0, 0.0), 0.0, (1.0, 0.0)
        )
        self.assertAlmostEqual(v, 0.1)
        self.assertAlmostEqual(w, 0.0)

    def test_turn_left(self):
        v, w = self.control(
            (0.0, 0.0), 0.0, (0.5, 0.2)
        )
        self.assertGreater(v, 0.0)
        self.assertGreater(w, 0.0)

    def test_turn_right(self):
        v, w = self.control(
            (0.0, 0.0), 0.0, (0.5, -0.2)
        )
        self.assertGreater(v, 0.0)
        self.assertLess(w, 0.0)

    def test_target_behind(self):
        v, w = self.control(
            (0.0, 0.0), 0.0, (-1.0, 0.0)
        )
        self.assertAlmostEqual(v, 0.0)
        self.assertGreater(abs(w), 0.0)

    def test_target_at_robot(self):
        v, w = self.control(
            (0.0, 0.0), 0.0, (0.0, 0.0)
        )
        self.assertEqual((v, w), (0.0, 0.0))

    def test_velocity_limits(self):
        for target in [
            (1.0, 0.0),
            (0.2, 0.2),
            (0.1, 0.4),
            (-0.2, 0.3),
        ]:
            v, w = self.control(
                (0.0, 0.0), 0.0, target
            )

            self.assertTrue(math.isfinite(v))
            self.assertTrue(math.isfinite(w))

            self.assertLessEqual(v, 0.2)
            self.assertLessEqual(abs(w), 2.0)


if __name__ == "__main__":
    unittest.main()
    