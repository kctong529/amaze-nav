import unittest
import numpy as np
from perception import EdgeObservation, SliceObservation, Gap, search_reposition_then_rotate


def scene(obstacles=None):
    if obstacles is None:
        obstacles=np.array([[.9,-.9],[.9,.9]])
    slices=(SliceObservation(.38,20,((-.5,-.25),(.25,.5)),(Gap(-.25,.25),)),
            SliceObservation(.55,20,((-.5,.05),(.35,.5)),(Gap(.05,.35),)))
    return EdgeObservation((0.,0.),(1.,0.),np.asarray(obstacles,dtype=float).reshape(-1,2),slices)

class RepositionTests(unittest.TestCase):
    def test_lateral_then_rotate(self):
        # Facing +t (+90 degrees) while intended graph direction is +s.
        result=search_reposition_then_rotate(scene(),(.20,-.12),robot_yaw=np.pi/2)
        self.assertIsNotNone(result)
        self.assertGreater(result.rotation_st[1],-.12)
        self.assertGreater(result.next_gap.s,result.rotation_st[0])
        self.assertGreaterEqual(result.observed_clearance,.129)
        self.assertAlmostEqual(result.first_leg_st[-1][0],.20,places=5)
        self.assertEqual(result.first_leg_st[-1],result.second_leg_st[0])
    def test_no_obstacles_no_certificate(self):
        self.assertIsNone(search_reposition_then_rotate(scene(np.empty((0,2))),(.2,0.),robot_yaw=0))
    def test_obstacle_at_robot_rejected(self):
        self.assertIsNone(search_reposition_then_rotate(scene(np.array([[.2,-.12]])),(.2,-.12),robot_yaw=np.pi/2))
    def test_invalid_config(self):
        with self.assertRaises(ValueError):
            search_reposition_then_rotate(scene(),(.2,0.),robot_yaw=float('nan'))
    def test_diagnostic_distinguishes_clearance(self):
        log={}
        search_reposition_then_rotate(scene(np.array([[.2,-.12]])),(.2,-.12),robot_yaw=0.,diagnostics=log)
        self.assertGreater(log['start_clearance'],0)

if __name__=='__main__': unittest.main()
