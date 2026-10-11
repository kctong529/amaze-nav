import unittest
import numpy as np
from perception import (Gap, SliceObservation, EdgeObservation,
                        search_final_approach, search_with_fallback)

class FinalApproachTests(unittest.TestCase):
    def observed(self, obstacle=None):
        points = np.array([[.45,-.4],[.45,.4]], dtype=float)
        if obstacle is not None:
            points=np.vstack((points,np.array(obstacle,dtype=float).reshape(1,2)))
        return EdgeObservation((0.,0.),(1.5,0.),points,(
            SliceObservation(.35, 12, (), (Gap(-.35,.35),)),
            SliceObservation(.45, 12, (), (Gap(-.35,.35),)),
        ))

    def test_short_incremental_final_goal(self):
        result=search_final_approach(self.observed(),(.9,0.),robot_yaw=0.)
        self.assertTrue(result.feasible)
        self.assertLessEqual(result.feasible[0].path_st[-1][0]-.9,.201)
        self.assertGreater(result.feasible[0].path_st[-1][0],.9)

    def test_passage_not_yet_cleared(self):
        result=search_final_approach(self.observed(),(.50,0.),robot_yaw=0.)
        self.assertFalse(result.feasible)
        self.assertIn("passage remains",result.rejected[0].reason)

    def test_unknown_without_passage_history(self):
        obs=EdgeObservation((0.,0.),(1.5,0.),np.array([[.4,.5]]),())
        result=search_final_approach(obs,(.9,0.),robot_yaw=0.)
        self.assertFalse(result.feasible)
        self.assertIn("Unknown",result.rejected[0].reason)

    def test_forward_obstacle_rejects_straight_continuation(self):
        result=search_final_approach(self.observed([1.0,0.]),(.9,0.),robot_yaw=0.)
        self.assertFalse(result.feasible)

    def test_stage_selected_after_last_passage(self):
        diagnostics={}
        result,mode=search_with_fallback(self.observed(),(.9,0.),robot_yaw=0.,diagnostics=diagnostics)
        self.assertTrue(result.feasible)
        self.assertEqual(mode,"direct destination")
        self.assertIn("direct destination",diagnostics)

    def test_invalid_horizon(self):
        with self.assertRaises(ValueError):
            search_final_approach(self.observed(),(.9,0.),horizon=-.1)

if __name__=='__main__':
    unittest.main()
