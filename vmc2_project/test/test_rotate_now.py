import unittest
import numpy as np
from perception import (EdgeObservation, SliceObservation, Gap, search_rotate_now)

class RotateNowTests(unittest.TestCase):
    def make_obs(self, walls=True):
        ys=np.linspace(-0.5,0.5,21)
        pts=np.vstack([np.column_stack([np.full(len(ys),s),ys]) for s in ([0.9] if walls else [])]) if walls else np.empty((0,2))
        slices=(SliceObservation(.30,10, ((-.4,-.3),(.3,.4)), (Gap(-.3,.3),)),)
        return EdgeObservation((0.,0.),(1.,0.),pts,slices)
    def test_no_observations(self):
        self.assertIsNone(search_rotate_now(self.make_obs(False),(.05,0.),robot_yaw=0.))
    def test_obstacle_at_start(self):
        o=self.make_obs(False)
        o=EdgeObservation(o.start,o.end,np.array([[.05,0.]]),o.slices)
        self.assertIsNone(search_rotate_now(o,(.05,0.),robot_yaw=0.))
    def test_invalid_settings(self):
        with self.assertRaises(ValueError):
            search_rotate_now(self.make_obs(),(.05,0.),robot_yaw=float('nan'))
    def test_valid_short_straight(self):
        o=EdgeObservation((0.,0.),(1.,0.),np.array([[.5,-.6],[.5,.6]]),
                          (SliceObservation(.3,10,((-.45,-.3),(.3,.45)),(Gap(-.3,.3),)),))
        m=search_rotate_now(o,(.05,0.),robot_yaw=1.2)
        self.assertIsNotNone(m)
        self.assertAlmostEqual(m.alignment_st[0],.05)
        self.assertGreater(m.next_gap.s,m.alignment_st[0])
        self.assertLess(abs(m.departure_yaw),.2)

if __name__=='__main__': unittest.main()
