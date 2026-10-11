import unittest
import numpy as np
from perception import (Gap, SliceObservation, EdgeObservation, find_local_passages,
                        select_next_local_passage, search_next_opening, search_with_fallback)

class NextOpeningTests(unittest.TestCase):
    def observation(self):
        # Two different passages with a missing observation between them.
        slices = (
            SliceObservation(.35, 8, (), (Gap(-.30,.30),)),
            SliceObservation(.45, 8, (), (Gap(-.25,.25),)),
            SliceObservation(.75, 8, (), (Gap(.05,.45),)),
            SliceObservation(.85, 8, (), (Gap(.08,.48),)),
        )
        return EdgeObservation((0.,0.), (2.,0.),
                               np.array([[.4,-.5],[.4,.5],[.8,-.5],[.8,.6]]), slices)

    def test_group_adjacent_overlapping_slices(self):
        passages=find_local_passages(self.observation(),robot_radius=.09,safety_margin=.02)
        self.assertEqual(len(passages),2)
        self.assertAlmostEqual(passages[0].s_end,.45)
        self.assertAlmostEqual(passages[1].s_start,.75)
        self.assertTrue(passages[1].usable_left < passages[1].usable_right)

    def test_selects_next_after_first_passed(self):
        p=select_next_local_passage(self.observation(),(.60,0.),robot_radius=.09,safety_margin=.02)
        self.assertAlmostEqual(p.s_start,.75)

    def test_does_not_discard_passage_while_crossing(self):
        p=select_next_local_passage(self.observation(),(.40,0.),robot_radius=.09,safety_margin=.02)
        self.assertAlmostEqual(p.s_start,.35)

    def test_no_gap_does_not_mean_open_corridor(self):
        obs=EdgeObservation((0.,0.),(2.,0.),np.empty((0,2)),())
        self.assertIsNone(select_next_local_passage(obs,(.4,0.)))
        result, passage=search_next_opening(obs,(.4,0.))
        self.assertFalse(result.feasible)
        self.assertIsNone(passage)

    def test_invalid_parameters(self):
        with self.assertRaises(ValueError):
            find_local_passages(self.observation(), safety_margin=-.01)

if __name__ == '__main__':
    unittest.main()
