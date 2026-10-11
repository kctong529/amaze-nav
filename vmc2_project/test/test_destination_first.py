import unittest
import numpy as np
from perception import (EdgeObservation, SliceObservation, Gap,
                        search_direct_destination, search_rotate_to_destination,
                        search_smooth_destination, search_with_fallback)

class DestinationFirstTests(unittest.TestCase):
    @staticmethod
    def observation(obstacle=None):
        points = [[.4, .55],[.8,-.52],[1.1,.6]]
        if obstacle is not None:
            points.append(obstacle)
        return EdgeObservation((0.,0.), (1.424,0.), np.array(points, dtype=float),
            (SliceObservation(.75,10, ((-.52,-.48),(.48,.52)), (Gap(-.48,.48),)),))

    def test_direct_straight_is_first_even_when_opening_exists(self):
        result, mode = search_with_fallback(self.observation(), (.9,0.),
                                            robot_yaw=0., safety_margin=.03)
        self.assertEqual(mode, 'direct destination')
        self.assertTrue(result.feasible)
        self.assertAlmostEqual(result.feasible[0].path_st[-1][0],1.424)

    def test_direct_segment_obstacle_blocks(self):
        r = search_direct_destination(self.observation([1.1,0.]),(.9,0.),robot_yaw=0.)
        self.assertFalse(r.feasible)
        self.assertIn('Too close',r.rejected[0].reason)
        self.assertIsNone(search_rotate_to_destination(
            self.observation([1.1,0.]),(.9,0.),robot_yaw=1.))

    def test_heading_mismatch_proposes_rotation_not_straight_driving(self):
        obs=self.observation()
        r=search_direct_destination(obs,(.9,0.),robot_yaw=np.pi/2)
        self.assertFalse(r.feasible)
        maneuver=search_rotate_to_destination(obs,(.9,0.),robot_yaw=np.pi/2)
        self.assertIsNotNone(maneuver)
        self.assertAlmostEqual(maneuver.departure_yaw,0.)
        diagnostics={}
        search,mode=search_with_fallback(obs,(.9,0.),robot_yaw=np.pi/2,
                                         diagnostics=diagnostics)
        self.assertEqual(mode,'rotate then destination')
        self.assertIn('destination alignment',diagnostics)
        self.assertFalse(search.feasible) # Rotation is a separate motion phase.

    def test_smooth_curve_to_goal(self):
        r=search_smooth_destination(self.observation(),(.65,-.12),robot_yaw=0.)
        self.assertTrue(r.feasible)
        self.assertAlmostEqual(r.feasible[0].path_st[-1][0],1.424)

if __name__ == '__main__': unittest.main()
