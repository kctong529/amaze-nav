import unittest
import numpy as np
from perception import analyze_edge, search_rotate_then_approach


def corridor():
    xs=np.arange(.15,1.40,.015)
    pts=np.array([(x,y) for x in xs for y in (-.35,.35)])
    return analyze_edge(pts,(0.,0.),(1.424,0.))


class HeadingAlignmentTests(unittest.TestCase):
    def test_alternative_heading_finds_path(self):
        result=search_rotate_then_approach(corridor(),(0.,0.),robot_yaw=1.5)
        self.assertIsNotNone(result)
        self.assertGreater(result.headings_tested,1)
        self.assertGreater(result.headings_with_paths,0)
        self.assertTrue(result.continuation.feasible)
        self.assertGreater(len(result.continuation.path_st),2)
        self.assertGreater(abs(result.yaw_change),.15)
    def test_no_points_returns_none(self):
        from perception import EdgeObservation
        o=corridor()
        empty=EdgeObservation(o.start,o.end,np.empty((0,2)),o.slices)
        self.assertIsNone(search_rotate_then_approach(empty,(0.,0.),robot_yaw=1.2))
    def test_obstacle_at_rotation_point_returns_none(self):
        from perception import EdgeObservation
        o=corridor()
        blocked=EdgeObservation(o.start,o.end,np.vstack([o.obstacle_points_st,[[0.,0.]]]),o.slices)
        self.assertIsNone(search_rotate_then_approach(blocked,(0.,0.),robot_yaw=1.2))
    def test_bad_yaw(self):
        with self.assertRaises(ValueError):
            search_rotate_then_approach(corridor(),(0.,0.),robot_yaw=float('nan'))

if __name__=='__main__':unittest.main()
