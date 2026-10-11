import unittest
import numpy as np
from perception import (EdgeObservation, SliceObservation, Gap,
                        search_two_waypoint_trajectories, search_with_fallback)

class FallbackTests(unittest.TestCase):
    def setUp(self):
        # Two wide staggered openings. Each retains a legal center interval.
        self.obs = EdgeObservation(
            (0.,0.),(1.424,0.),
            np.array([[0.58,-.65],[.58,-.40],[.58,.25],[.58,.65],
                      [.94,-.65],[.94,-.25],[.94,.4],[.94,.65]]),
            (SliceObservation(.55,10, ((-.6,-.4),(.25,.65)), (Gap(-.4,.25),)),
             SliceObservation(.95,10, ((-.65,-.25),(.4,.65)), (Gap(-.25,.4),)))
        )

    def test_all_evaluations_keep_paths(self):
        result = search_two_waypoint_trajectories(self.obs,(0.,0.),robot_yaw=0.,max_pairs=9)
        self.assertGreater(len(result.feasible)+len(result.rejected),0)
        self.assertTrue(any(r.path_st for r in result.feasible+result.rejected))

    def test_tangent_and_two_waypoint_interpolation(self):
        result = search_two_waypoint_trajectories(self.obs,(0.,0.),robot_yaw=0.,max_pairs=9)
        with_paths = [r for r in result.feasible+result.rejected if r.path_st]
        self.assertTrue(with_paths)
        path=np.asarray(with_paths[0].path_st)
        d=path[1]-path[0]
        self.assertGreater(d[0],0)
        self.assertAlmostEqual(float(d[1]),0.,delta=.02)
        # First and second waypoints appear at segment joins.
        self.assertLess(np.min(np.linalg.norm(path-[.55,-.173],axis=1)),.25)

    def test_fallback_mode(self):
        result, mode=search_with_fallback(self.obs,(0.,0.),robot_yaw=0.,max_fallback_pairs=3)
        self.assertIn(mode,('single-gap full-edge','short approach only','two-waypoint full-edge fallback','next opening only', 'direct destination', 'smooth destination', 'rotate then destination'))
        self.assertIsInstance(result.feasible,tuple)
        self.assertIsInstance(result.rejected,tuple)

if __name__=='__main__': unittest.main()
