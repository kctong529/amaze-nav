import unittest
import numpy as np
from perception import EdgeObservation, SliceObservation, Gap, OpeningProgressTracker

A=(0.0,0.0); B=(1.0,0.0)
def obs():
    return EdgeObservation(A,B,np.empty((0,2)),(SliceObservation(0.5,4,(),(Gap(-0.3,0.3),)),))

class OpeningTests(unittest.TestCase):
    def test_approaching(self):
        r=OpeningProgressTracker()
        _,g=r.update(obs(),(0.2,0.0)); self.assertEqual(g[0].state,'AHEAD')
    def test_entry_then_cleared(self):
        r=OpeningProgressTracker()
        r.update(obs(),(0.5,0.0))
        _,g=r.update(obs(),(0.75,0.0)); self.assertEqual(g[0].state,'CLEARED')
    def test_cross_outside_is_off_path(self):
        r=OpeningProgressTracker()
        _,g=r.update(obs(),(0.75,0.40)); self.assertEqual(g[0].state,'OFF_PATH')
        self.assertTrue(g[0].passed_by_longitudinal_rule)
    def test_not_entered_if_outside(self):
        r=OpeningProgressTracker()
        _,g=r.update(obs(),(0.5,0.25)); self.assertEqual(g[0].state,'MISALIGNED')
    def test_reset_on_reverse_edge(self):
        r=OpeningProgressTracker();r.update(obs(),(0.5,0.0))
        o=EdgeObservation(B,A,np.empty((0,2)),(SliceObservation(0.5,4,(),(Gap(-0.3,0.3),)),))
        _,g=r.update(o,(0.25,0.0)); self.assertEqual(g[0].state,'OFF_PATH')

if __name__=='__main__': unittest.main()
