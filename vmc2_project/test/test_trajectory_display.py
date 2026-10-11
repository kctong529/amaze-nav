import unittest
from dataclasses import dataclass
import numpy as np
from trajectory_display import select_representatives, group_rejections

@dataclass
class Trial:
    path_st: np.ndarray
    reason: str = ""

class DisplayTests(unittest.TestCase):
    def test_limits_and_keeps_first(self):
        trials = [Trial(np.array([[0.,0.],[.5, x],[1.,0.]])) for x in range(15)]
        selection = select_representatives(trials, 4)
        self.assertEqual(len(selection), 4)
        self.assertIs(selection[0], trials[0])
        self.assertIs(selection[1], trials[-1])

    def test_empty_and_zero(self):
        self.assertEqual(select_representatives([], 4), [])
        self.assertEqual(select_representatives([Trial(np.array([[0.,0.],[1.,0.]]))], 0), [])

    def test_reason_groups(self):
        ts = [Trial(np.array([[0.,0.],[1.,float(i)]]), reason) for reason in
              ("Too close to measured returns", "Excessive curvature", "Something else") for i in range(6)]
        selected = group_rejections(ts, per_reason=2)
        self.assertEqual(len(selected), 4)
        self.assertEqual(set(x.reason for x in selected), {"Too close to measured returns", "Excessive curvature"})

if __name__ == "__main__": unittest.main()
