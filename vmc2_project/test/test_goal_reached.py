import unittest
import numpy as np
from perception import (Gap, SliceObservation, EdgeObservation,
                        local_goal_reached, search_goal_reached_continuation,
                        search_with_fallback)

class GoalReachedTests(unittest.TestCase):
    def observation(self, extra=None):
        # Same edge coordinates as the bonus case, with a narrow throat
        # near s=1.05 and obstacle points clear of a short southbound segment.
        points = np.array([[1.05,-.35],[1.05,-.058],[1.05,.40]],float)
        if extra is not None:
            points = np.vstack((points, np.array(extra,dtype=float).reshape(1,2)))
        return EdgeObservation((0.,0.),(1.424,0.),points,(
            SliceObservation(1.05,10,(),(Gap(-.346,-.058),)),
        ))

    def test_logged_bonus_pose_goal_reached(self):
        self.assertTrue(local_goal_reached(1.034,-.221,1.05,(-.227,-.177)))

    def test_goal_reached_is_not_cleared(self):
        self.assertTrue(local_goal_reached(1.034,-.221,1.05,(-.227,-.177)))
        # Rear has NOT cleared s=1.05.
        self.assertLess(1.034-.089-.03,1.05)

    def test_reject_misalignment(self):
        self.assertFalse(local_goal_reached(1.034,-.30,1.05,(-.227,-.177)))

    def test_short_straight_continuation(self):
        # Add wall points sufficiently far from centerline of trajectory.
        obs=self.observation()
        result=search_goal_reached_continuation(obs,(1.034,-.221),robot_yaw=0.)
        self.assertTrue(result.feasible, result.rejected)
        self.assertGreater(result.feasible[0].path_st[-1][0],1.034)
        self.assertLessEqual(result.feasible[0].path_st[-1][0]-1.034,.151)

    def test_close_obstacle_rejects(self):
        obs=self.observation([1.12,-.221])
        result=search_goal_reached_continuation(obs,(1.034,-.221),robot_yaw=0.)
        self.assertFalse(result.feasible)
        self.assertIn("Too close", result.rejected[0].reason)

    def test_no_goal_does_not_trigger(self):
        obs=self.observation()
        result=search_goal_reached_continuation(obs,(.65,-.221),robot_yaw=0.)
        self.assertFalse(result.feasible)

    def test_integrated_first_stage(self):
        obs=self.observation()
        result,mode=search_with_fallback(obs,(1.034,-.221),robot_yaw=0.,
                                          safety_margin=.03,max_curvature=8.)
        self.assertTrue(result.feasible)
        self.assertEqual(mode,"smooth destination")

if __name__=='__main__': unittest.main()
