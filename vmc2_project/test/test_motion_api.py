"""Tests for the single high-level entry point used by the explorer."""
import unittest
import numpy as np
from perception import EdgeObservation, SliceObservation, Gap, MotionPlan, plan_motion


def observation(block=False):
    points = [[0.4, 0.55], [0.8, -0.52], [1.1, 0.6]]
    if block:
        points.append([1.1, 0.0])
    return EdgeObservation((0., 0.), (1.424, 0.), np.array(points),
                           (SliceObservation(.75, 10,
                           ((-.52, -.48), (.48, .52)), (Gap(-.48, .48),)),))


class MotionApiTests(unittest.TestCase):
    def test_direct_plan_contains_one_translation(self):
        result = plan_motion(observation(), (.9, 0.), robot_yaw=0.)
        self.assertEqual(result.mode, 'direct destination')
        self.assertEqual(len(result.plan.segments_st), 1)
        self.assertFalse(result.plan.requires_rotation)
        self.assertAlmostEqual(result.plan.segments_st[0][-1][0], 1.424)

    def test_alignment_is_not_a_continuous_path(self):
        result = plan_motion(observation(), (.9, 0.), robot_yaw=np.pi / 2)
        self.assertEqual(result.mode, 'rotate then destination')
        self.assertFalse(result.search.feasible)
        self.assertTrue(result.plan.requires_rotation)
        self.assertTrue(result.plan.requires_rescan)
        self.assertEqual(len(result.plan.segments_st), 1)

    def test_multi_leg_representation_preserves_boundaries(self):
        plan = MotionPlan('example', (((0., 0.), (.1, 0.)),
                                      ((.1, 0.), (.2, .1))),
                          rotation_st=(.1, 0.), departure_yaw=.7,
                          requires_rescan=True)
        self.assertEqual(len(plan.segments_st), 2)
        self.assertTrue(plan.requires_rotation)


if __name__ == '__main__':
    unittest.main()
