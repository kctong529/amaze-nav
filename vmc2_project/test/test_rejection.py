
import unittest

import numpy as np

from perception import (
    Passability,
    Gap,
    SliceObservation,
    EdgeObservation,
    analyze_edge,
    assess_passability,
    detect_bottlenecks,
)


A = (0.0, 0.0)
B = (1.4, 0.0)


def obs_with_slices(*sections):
    return EdgeObservation(A, B, np.empty((0, 2)), tuple(sections))


def section(s, gaps, cluster_count=3):
    intervals = tuple(
        (float(i), float(i) + 0.01)
        for i in range(cluster_count)
    )
    return SliceObservation(
        s,
        20,
        intervals,
        tuple(Gap(*g) for g in gaps),
    )


class EarlyRejectionTests(unittest.TestCase):
    def test_all_detected_openings_narrow_are_blocked(self):
        obs = obs_with_slices(
            section(0.5, [(-0.4, -0.2), (-0.1, 0.1)])
        )
        result = assess_passability(obs, A)

        self.assertEqual(result.status, Passability.BLOCKED)
        self.assertAlmostEqual(result.minimum_observed_width, 0.2)

    def test_one_wide_gap_prevents_block_at_that_slice(self):
        obs = obs_with_slices(
            section(0.5, [(-0.4, -0.1), (0.0, 0.1)])
        )

        self.assertEqual(
            assess_passability(obs, A).status,
            Passability.UNKNOWN,
        )

    def test_other_wide_slice_does_not_overrule_narrow_slice(self):
        obs = obs_with_slices(
            section(0.3, [(-0.4, -0.1)]),
            section(0.6, [(-0.2, 0.0)]),
        )

        self.assertEqual(
            assess_passability(obs, A).status,
            Passability.BLOCKED,
        )

    def test_past_narrow_slice_is_ignored(self):
        obs = obs_with_slices(
            section(0.3, [(-0.2, 0.0)])
        )

        self.assertEqual(
            assess_passability(obs, (0.55, 0.0)).status,
            Passability.UNKNOWN,
        )

    def test_empty_observations_are_unknown(self):
        obs = analyze_edge(np.empty((0, 2)), A, B)

        self.assertEqual(
            assess_passability(obs, A).status,
            Passability.UNKNOWN,
        )

    def test_too_few_clusters_or_missing_gap_are_unknown(self):
        obs = obs_with_slices(
            section(0.5, [(-0.2, 0.0)], 1),
            section(0.7, [], 3),
        )

        self.assertEqual(
            assess_passability(obs, A).status,
            Passability.UNKNOWN,
        )

    def test_exact_width_threshold_is_not_blocked(self):
        obs = obs_with_slices(
            section(0.5, [(0.0, 0.238)])
        )

        self.assertEqual(
            assess_passability(obs, A).status,
            Passability.UNKNOWN,
        )

    def test_default_margin_is_consistent_with_planner(self):
        # 0.24 m exceeds the 0.238 m standard threshold
        # but does not exceed the 0.258 m threshold.
        obs = obs_with_slices(
            section(0.5, [(0.0, 0.24)])
        )

        self.assertEqual(
            assess_passability(obs, A).status,
            Passability.UNKNOWN,
        )
        self.assertEqual(detect_bottlenecks(obs), [])
        self.assertEqual(
            assess_passability(
                obs,
                A,
                safety_margin=0.04,
            ).status,
            Passability.BLOCKED,
        )

    def test_no_longer_accepts_corridor_bounds(self):
        obs = obs_with_slices()

        with self.assertRaises(TypeError):
            assess_passability(
                obs,
                A,
                corridor_bounds=(-0.5, 0.5),
            )


if __name__ == "__main__":
    unittest.main()
