"""Integration tests for graph navigation and recovery state transitions.

Scenarios A–E mirror the original ``My_project_role_sim.py``. Each drives
``navigator.control_step`` repeatedly with fake sensors and a simple unicycle
model. These tests do not validate real obstacle detection or motor control.
"""
import unittest
from pathlib import Path

from navigation_sim import MAZE_BLOCKED, load_navigator, run


class NavigationIntegrationTests(unittest.TestCase):
    """Verify the five original offline routes and recovery scenarios."""

    @classmethod
    def setUpClass(cls):
        navigator = Path(__file__).resolve().parents[1] / 'src' / 'navigator.py'
        load_navigator(navigator)

    def assert_navigation(self, result):
        """Report each preserved scenario check as a named subtest."""
        for name, passed in result.checks.items():
            with self.subTest(check=name):
                self.assertTrue(passed, (
                    f'{name}; modes={result.mode_history}, '
                    f'removed={result.removed_edges}, '
                    f'time={result.elapsed:.1f}s, '
                    f'offset={result.max_offset:.3f}m, errors={result.problems}'
                ))

    def test_a_unobstructed_navigation(self):
        """An unobstructed run reaches the exit without backtracking."""
        result = run((0.0, 0.0), [], 0.75)
        self.assert_navigation(result)
        self.assertEqual(result.removed_edges, [])
        self.assertNotIn('BACKTRACK', result.mode_history)

    def test_b_single_blocked_edge(self):
        """One simulated blockage triggers BACKTRACK, REPLAN, then FOLLOW."""
        result = run((3.560, -4.272), ['3,2-3,3'], 0.75)
        self.assert_navigation(result)
        self.assertEqual(result.removed_edges, [frozenset(((3, 2), (3, 3)))])
        self.assertEqual(result.mode_history[:3], ['BACKTRACK', 'REPLAN', 'FOLLOW'])

    def test_c_multiple_blockages(self):
        """Successive discoveries remove each simulated blocked edge at most once."""
        result = run((0.0, 0.0), MAZE_BLOCKED, 0.75)
        self.assert_navigation(result)
        self.assertGreaterEqual(len(result.removed_edges), 2)
        self.assertEqual(len(result.removed_edges), len(set(result.removed_edges)))

    def test_d_late_blockage_detection(self):
        """Recovery still works when the obstacle is reported later in an edge."""
        result = run((0.0, 0.0), MAZE_BLOCKED, 0.35)
        self.assert_navigation(result)
        self.assertIn('BACKTRACK', result.mode_history)
        self.assertIn('REPLAN', result.mode_history)

    def test_e_navigation_from_checkpoint_two(self):
        """Navigation can begin at a non-entry grid node."""
        result = run((4.984, -4.272), MAZE_BLOCKED, 0.75)
        self.assert_navigation(result)
        self.assertEqual(result.first_path[0], '(3,3)')


if __name__ == '__main__':
    unittest.main()
