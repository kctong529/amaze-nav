"""Unit tests for the supplied navigator graph and state-machine code.

The test node replaces ROS IO, perception decisions, motion targets, and
controller output. Tests exercise one planning call or one control step at a
time; multi-step offline runs live in ``test_navigation_integration.py``.
The original navigator and its course-provided A* implementation are unchanged.
"""
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock


class FakeClock:
    now_s = 0.0

    def now(self):
        return types.SimpleNamespace(nanoseconds=int(self.now_s * 1e9))


CLOCK = FakeClock()


class FakeNode:
    def __init__(self, *args, **kwargs):
        self._logger = Mock()

    def get_clock(self):
        return CLOCK

    def get_logger(self):
        return self._logger

    def create_publisher(self, *args, **kwargs):
        return types.SimpleNamespace(publish=Mock())

    def create_subscription(self, *args, **kwargs):
        return None

    def create_timer(self, *args, **kwargs):
        return None


def stub_dependencies():
    """Provide only the ROS interfaces needed to import the template node."""
    def add(name, **members):
        module = types.ModuleType(name)
        module.__dict__.update(members)
        sys.modules[name] = module

    add('rclpy', init=lambda: None, spin=lambda node: None, shutdown=lambda: None)
    add('rclpy.node', Node=FakeNode)
    add('rclpy.parameter', Parameter=object)
    add('rcl_interfaces')
    add('rcl_interfaces.msg', SetParametersResult=object)
    add('geometry_msgs')
    add('geometry_msgs.msg', Twist=lambda: types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.0), angular=types.SimpleNamespace(z=0.0)))
    add('nav_msgs')
    add('nav_msgs.msg', Odometry=object)
    add('sensor_msgs')
    add('sensor_msgs.msg', LaserScan=object)
    # The tests do not exercise visualization.
    import cv2
    cv2.namedWindow = lambda *a, **k: None
    cv2.imshow = lambda *a, **k: None
    cv2.waitKey = lambda *a, **k: -1


stub_dependencies()
source = Path(__file__).resolve().parents[1] / 'src' / 'navigator.py'
spec = importlib.util.spec_from_file_location('navigator_under_test', source)
nav = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = nav
spec.loader.exec_module(nav)


class NavigatorTestCase(unittest.TestCase):
    """Initialize a headless navigator with deterministic motion stubs."""

    def setUp(self):
        CLOCK.now_s = 0.0
        self.node = nav.GridPathFollowerNode()
        self.node.pose_ready = True
        self.node.dwell_until = None
        self.node.lookahead_dist = 0.35
        self.node.v_max = 0.22
        self.node.w_max = 2.0
        self.node.x, self.node.y, self.node.yaw = 0.712, 0.0, 0.0
        self.node.publish_cmd = Mock()
        self.node.publish_stop = Mock()
        self.node.draw_scene = Mock()
        self.node.turtlebot_control = Mock(return_value=(0.10, 0.0))
        self.node.obstacle_avoided_target = Mock(return_value=(1.0, 0.0))
        self.node.is_passable = Mock(return_value=True)


class GraphTests(NavigatorTestCase):
    """Verify graph connectivity and course-template A* routing."""

    def test_maze_graph_has_25_nodes_and_40_undirected_edges(self):
        self.assertEqual(len(self.node.graph.positions), 25)
        self.assertEqual(len(list(self.node.graph.edges())), 40)

    def test_astar_finds_shortest_grid_route(self):
        start = (0.712, 0.0)
        end = (6.408, -5.696)
        path = self.node.astar_path(self.node.graph, start, end)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], end)
        self.assertEqual(len(path), 9)
        self.assertTrue(all(self.node.graph.has_edge(a, b) for a, b in zip(path, path[1:])))

    def test_astar_reroutes_around_removed_edge(self):
        graph = self.node.graph
        a, b = (0.712, 0.0), (2.136, 0.0)
        graph.remove_edge(a, b)
        path = self.node.astar_path(graph, a, b)
        self.assertNotIn((a, b), list(zip(path, path[1:])))
        self.assertEqual(len(path), 4)

    def test_astar_returns_empty_if_goal_is_disconnected(self):
        graph = self.node.graph
        goal = (6.408, -5.696)
        graph.remove_edge((4.984, -5.696), goal)
        graph.remove_edge((6.408, -4.272), goal)
        self.assertEqual(self.node.astar_path(graph, (0.712, 0.0), goal), [])


class PlanningTests(NavigatorTestCase):
    """Check waypoint composition and unreachable-goal behavior."""

    def test_plan_from_entry_includes_goal_without_duplicate_nodes(self):
        path = self.node.plan_from((0.0, 0.0))
        self.assertEqual(path[0], (0.0, 0.0))
        self.assertEqual(path[1], (0.712, 0.0))
        self.assertEqual(path[-1], self.node.goal_pos)
        self.assertEqual(len(path), 11)
        self.assertTrue(all(a != b for a, b in zip(path, path[1:])))

    def test_plan_from_graph_node_does_not_duplicate_start(self):
        start = (3.560, -4.272)
        path = self.node.plan_from(start)
        self.assertEqual(path[0], start)
        self.assertNotEqual(path[0], path[1])

    def test_plan_from_unreachable_goal_stays_at_start(self):
        graph = self.node.graph
        graph.remove_edge((4.984, -5.696), (6.408, -5.696))
        graph.remove_edge((6.408, -4.272), (6.408, -5.696))
        self.assertEqual(self.node.plan_from((3.560, -4.272)), [(3.560, -4.272)])


class StateMachineTests(NavigatorTestCase):
    """Test individual FOLLOW, BACKTRACK, REPLAN, and DONE transitions."""

    def setUp(self):
        super().setUp()
        self.a = (0.712, 0.0)
        self.b = (2.136, 0.0)
        self.node.full_path = [self.a, self.b, (3.560, 0.0)]
        self.node.x, self.node.y = 1.0, 0.0

    def test_follow_emits_controller_command_on_passable_edge(self):
        self.node.control_step()
        self.assertEqual(self.node.mode, 'FOLLOW')
        self.node.turtlebot_control.assert_called_once()
        self.node.publish_cmd.assert_called_with(0.10, 0.0)
        self.assertEqual(self.node.active_edge, (self.a, self.b))

    def test_blockage_triggers_backtrack_and_remembers_graph_edge(self):
        self.node.is_passable.return_value = False
        self.node.control_step()
        self.assertEqual(self.node.mode, 'BACKTRACK')
        self.assertEqual(self.node.prev_node, self.a)
        self.assertEqual(self.node.block_edge, (self.a, self.b))
        self.node.publish_stop.assert_called()

    def test_backtrack_arrival_enters_replan(self):
        self.node._set_mode('BACKTRACK')
        self.node.prev_node = self.a
        self.node.x = self.a[0] + 0.01
        CLOCK.now_s = 1.0
        self.node.dwell_until = None
        self.node.control_step()
        self.assertEqual(self.node.mode, 'REPLAN')

    def test_replan_removes_blockage_and_starts_from_previous_node(self):
        self.node._set_mode('REPLAN')
        self.node.prev_node = self.a
        self.node.block_edge = (self.a, self.b)
        CLOCK.now_s = 1.0
        self.node.dwell_until = None
        self.node.control_step()
        self.assertFalse(self.node.graph.has_edge(self.a, self.b))
        self.assertEqual(self.node.full_path[0], self.a)
        self.assertEqual(self.node.mode, 'FOLLOW')
        self.assertIsNone(self.node.block_edge)

    def test_goal_reached_enters_done(self):
        self.node.x, self.node.y = self.node.goal_pos
        self.node.control_step()
        self.assertEqual(self.node.mode, 'DONE')
        self.node.publish_stop.assert_called()


if __name__ == '__main__':
    unittest.main()
