import unittest
import numpy as np
from perception import _progress_rejection, _shift_gap_candidates, GapCandidate

class TuningTests(unittest.TestCase):
    def test_small_backward_motion_allowed(self):
        path = np.array([[0,0],[0.1,0],[0.085,0.01],[0.3,0.0]])
        self.assertIsNone(_progress_rejection(path,backtrack_budget=.05,max_length_ratio=3.0))
        self.assertEqual(_progress_rejection(path,backtrack_budget=.01,max_length_ratio=3.0), 'Trajectory reverses along the edge')

    def test_loop_and_length_rejected(self):
        path = np.array([[0,0],[.4,.8],[.8,-.8],[1,0]])
        self.assertEqual(_progress_rejection(path,backtrack_budget=.05,max_length_ratio=1.3),'Excessive trajectory length')

    def test_no_net_progress_rejected(self):
        self.assertEqual(_progress_rejection(np.array([[0,0],[.01,0]]),backtrack_budget=.05,max_length_ratio=3),'Insufficient net forward progress')

    def test_shift_candidates_keep_original_slice(self):
        base=[GapCandidate(.5,.12,.4,.5)]
        shifted=_shift_gap_candidates(base,1.4,(-.1,0,.1))
        self.assertEqual([round(c.s,2) for c in shifted],[.4,.5,.6])
        self.assertTrue(all(c.slice_s==.5 for c in shifted))

if __name__=='__main__': unittest.main()
