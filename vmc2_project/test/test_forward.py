import unittest
import numpy as np
from perception import (
    analyze_edge, find_feasible_gap_candidates,
    select_forward_observation, search_with_fallback,
)


def corridor():
    s = np.arange(0.05, 1.40, 0.01)
    returns = np.array([(x, y) for x in s for y in (-0.35, 0.35)])
    return analyze_edge(returns, (0., 0.), (1.424, 0.))


class ForwardOpeningTests(unittest.TestCase):
    def test_before_opening_keeps_it(self):
        obs = corridor()
        view = select_forward_observation(obs, (0.35, 0.))
        self.assertTrue(any(abs(s.s - .45) < .001 for s in view.slices))

    def test_near_opening_keeps_it_until_rear_clears(self):
        obs = corridor()
        view = select_forward_observation(obs, (.55, 0.))
        self.assertTrue(any(abs(s.s - .45) < .001 for s in view.slices))

    def test_passed_opening_is_removed_next_remains(self):
        obs = corridor()
        view = select_forward_observation(obs, (.75, 0.))
        self.assertFalse(any(abs(s.s - .45) < .001 for s in view.slices))
        self.assertTrue(any(abs(s.s - .85) < .001 for s in view.slices))
        # Still check the entire measured point cloud for clearance.
        np.testing.assert_array_equal(view.obstacle_points_st, obs.obstacle_points_st)
        self.assertTrue(all(c.slice_s >= .75 - .089 - .03 - 1e-9
                            for c in find_feasible_gap_candidates(view)))

    def test_after_every_observed_opening_no_old_candidate_reused(self):
        obs = corridor()
        view = select_forward_observation(obs, (1.6, 0.))
        self.assertEqual(view.slices, ())
        self.assertEqual(find_feasible_gap_candidates(view), [])

    def test_invalid_settings_rejected(self):
        with self.assertRaises(ValueError):
            select_forward_observation(corridor(), (.5, 0.), passed_tolerance=-.01)

    def test_integrated_search_near_opening(self):
        result, mode = search_with_fallback(corridor(), (.7, 0.), robot_yaw=0.)
        self.assertTrue(result.feasible)
        self.assertEqual(mode, 'direct destination')


if __name__ == '__main__':
    unittest.main()
