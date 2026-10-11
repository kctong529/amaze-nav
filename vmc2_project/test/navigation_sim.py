"""Deterministic, ROS-free harness for navigator state-machine integration tests.

Load ``navigator.py`` through :func:`load_navigator`, then run a scenario with
:func:`run`. The harness substitutes ROS interfaces, obstacle decisions, and
velocity control so the tests exercise graph planning and state transitions
without requiring Gazebo or the unfinished control/perception workstreams.

A simulated successful run demonstrates logical navigation and approximate
kinematic progress, not physical clearance or safe TurtleBot operation.
"""

import importlib.util
import math
import sys
import types
from dataclasses import dataclass
from unittest.mock import patch


XS = [0.712, 2.136, 3.560, 4.984, 6.408]
YS = [0.0, -1.424, -2.848, -4.272, -5.696]
NODES = {(round(x, 3), round(y, 3)): (row, col)
         for row, y in enumerate(YS) for col, x in enumerate(XS)}

# Fixture edges from the original offline scenario, not sensor-derived evidence.
# They must never be imported by the production navigator.
MAZE_BLOCKED = [
    '0,1-0,2', '1,0-1,1', '1,1-1,2', '1,3-1,4', '2,1-2,2',
    '3,2-3,3', '4,2-4,3', '0,3-1,3', '2,0-3,0',
]

CENTRE_LINES = [((-0.3, 0.0), (0.712, 0.0)),
                ((6.408, -5.696), (7.5, -5.696))]
CENTRE_LINES += [((XS[c], y), (XS[c + 1], y))
                 for y in YS for c in range(4)]
CENTRE_LINES += [((x, YS[r]), (x, YS[r + 1]))
                 for x in XS for r in range(4)]

# Time and last velocity command shared by the ROS stand-ins during one run.
# Reset at the start of each scenario; not a real ROS clock or publisher.
SIM = {'time': 0.0, 'command': (0.0, 0.0)}
nav = None


@dataclass
class SimulationResult:
    """Observed outcomes of one simulated navigation run.

    ``mode_history`` records transitions (not every control tick);
    ``removed_edges`` and ``blocked_edges`` use unordered pairs of grid indices.
    ``checks`` contains the preserved assertions from the original harness.
    """

    goal_reached: bool
    mode_history: list
    removed_edges: list
    blocked_edges: set
    first_path: list
    visited_nodes: list
    replan_starts: list
    max_offset: float
    elapsed: float
    problems: list
    checks: dict


class FakeLogger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass

    def debug(self, *args, **kwargs):
        pass


class FakeClock:
    def now(self):
        return types.SimpleNamespace(nanoseconds=int(SIM['time'] * 1e9))


class FakePublisher:
    def publish(self, msg):
        SIM['command'] = (float(msg.linear.x), float(msg.angular.z))


class FakeNode:
    def __init__(self, *args, **kwargs):
        self._logger = FakeLogger()

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return FakeClock()

    def create_publisher(self, *args, **kwargs):
        return FakePublisher()

    def create_subscription(self, *args, **kwargs):
        return None

    def create_timer(self, *args, **kwargs):
        return None

    def destroy_node(self):
        pass


class FakeTwist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)


def install_ros_stubs():
    """Register minimal ROS message/node modules for importing ``navigator.py``.

    These substitutes intentionally implement only the APIs used by the test
    harness. OpenCV window calls are disabled to keep tests headless.
    """
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
    add('geometry_msgs.msg', Twist=FakeTwist)
    add('nav_msgs')
    add('nav_msgs.msg', Odometry=object)
    add('sensor_msgs')
    add('sensor_msgs.msg', LaserScan=object)

    try:
        import cv2
    except ImportError:
        add('cv2')
        import cv2
    cv2.namedWindow = lambda *args, **kwargs: None
    cv2.imshow = lambda *args, **kwargs: None
    cv2.waitKey = lambda *args, **kwargs: -1


def load_navigator(path):
    """Import the unmodified navigator under test with ROS stand-ins installed.

    This must be called before :func:`run`; it does not patch production files.
    """
    global nav
    install_ros_stubs()
    spec = importlib.util.spec_from_file_location('navigator_sim_under_test', path)
    nav = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = nav
    spec.loader.exec_module(nav)
    return nav


def node_index(point):
    return NODES.get((round(point[0], 3), round(point[1], 3)))


def node_name(point):
    index = node_index(point)
    if index is not None:
        return f'({index[0]},{index[1]})'
    return f'({point[0]:.3f},{point[1]:.3f})'


def parse_blocked_edges(specifications):
    blocked = set()
    for specification in specifications:
        start, end = specification.split('-')
        start = tuple(int(value) for value in start.split(','))
        end = tuple(int(value) for value in end.split(','))
        blocked.add(frozenset((start, end)))
    return blocked


def fake_passability(blocked_edges, detection_distance):
    """Return a navigator-compatible boolean perception stub.

    A listed undirected graph edge becomes ``False`` once the robot is within
    ``detection_distance`` metres of that segment's midpoint. All other edges
    return ``True``. This is deliberately *not* the v1.1 LiDAR gap classifier.
    """
    def is_passable(self, segment_yaw):
        _, segment, _ = nav.nearest_point_on_polyline(
            self.full_path, (self.x, self.y)
        )
        start, end = self.full_path[segment:segment + 2]
        edge = frozenset((node_index(start), node_index(end)))
        if None in edge or edge not in blocked_edges:
            return True
        midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
        return math.dist((self.x, self.y), midpoint) > detection_distance

    return is_passable


def fake_target(self, robot_position, segment_yaw):
    _, segment, _ = nav.nearest_point_on_polyline(self.full_path, robot_position)
    return nav.lookahead_point(self.full_path[segment:segment + 2],
                               robot_position, 0.35)


def fake_controller(self, robot_position, heading, target, dt, lookahead):
    """Drive a simple turn-then-forward law, independent of Shaan's controller."""
    dx = target[0] - robot_position[0]
    dy = target[1] - robot_position[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-3:
        return 0.0, 0.0

    error = nav.angle_wrap(math.atan2(dy, dx) - heading)
    angular = max(-2.0, min(2.0, 2.5 * error))
    if abs(error) > math.radians(30):
        return 0.0, angular

    linear = 0.22 * math.cos(error) * min(1.0, distance / 0.15)
    return max(linear, 0.03), angular


def is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def run(start, blocked_spec, detection_distance, max_time=400.0):
    """Execute one offline scenario until ``DONE``, an error, or timeout.

    Args:
        start: Initial robot pose as a world-frame ``(x, y)`` coordinate.
        blocked_spec: Test-only strings ``"row,col-row,col"`` describing edges.
        detection_distance: Distance from edge midpoint at which fake
            perception first reports a configured blockage, in metres.
        max_time: Maximum simulated time, in seconds.

    Returns:
        SimulationResult with route, transitions, removed edges, kinematic
        measurements, and the original scenario checks.

    The simulation runs at 0.05 s per step. It injects stand-ins only for
    this run and restores the navigator's class methods afterwards.
    """
    if nav is None:
        raise RuntimeError('Call load_navigator() before run().')

    SIM['time'] = 0.0
    SIM['command'] = (0.0, 0.0)
    blocked = parse_blocked_edges(blocked_spec)
    navigator_type = nav.GridPathFollowerNode
    original_path_plan = navigator_type.path_plan

    def initial_path_plan(self, start_from=None):
        if not getattr(self, '_first_plan_done', False):
            self._first_plan_done = True
            self.start_pos = start
            start_from = start
        return original_path_plan(self, start_from)

    # Patch before construction: __init__ calls path_plan() immediately.
    # Context managers restore the navigator methods even if the run fails.
    with (patch.object(navigator_type, 'is_passable',
                       fake_passability(blocked, detection_distance)),
          patch.object(navigator_type, 'obstacle_avoided_target', fake_target),
          patch.object(navigator_type, 'turtlebot_control', fake_controller),
          patch.object(navigator_type, 'path_plan', initial_path_plan)):
        node = navigator_type()
        if not is_number(getattr(node, 'lookahead_dist', None)):
            node.lookahead_dist = 0.35
        if not is_number(getattr(node, 'v_max', None)):
            node.v_max = 0.22
        if not is_number(getattr(node, 'w_max', None)):
            node.w_max = 2.0
        pid = getattr(node, 'pid_angular', None)
        if pid is None or not is_number(getattr(pid, 'kp', None)):
            node.pid_angular = nav.PID(kp=1.5, ki=0.0, kd=0.05, i_limit=0.5)

        transitions = []
        removed_edges = []
        problems = []
        replan_starts = []
        visited_nodes = []
        max_offset = 0.0
        timestep = 0.05

        original_set_mode = node._set_mode
        original_remove_edge = node.graph.remove_edge

        def record_mode(mode):
            if node.mode != mode:
                transitions.append(mode)
            original_set_mode(mode)

        def record_removal(a, b):
            removed_edges.append(frozenset((node_index(a), node_index(b))))
            original_remove_edge(a, b)

        node._set_mode = record_mode
        node.graph.remove_edge = record_removal
        first_path = [node_name(point) for point in node.full_path]

        x, y = start
        heading = 0.0
        if len(node.full_path) >= 2:
            a, b = node.full_path[:2]
            heading = math.atan2(b[1] - a[1], b[0] - a[0])
        node.x, node.y, node.yaw, node.pose_ready = x, y, heading, True

        while SIM['time'] < max_time:
            SIM['time'] += timestep
            previous_mode = node.mode
            try:
                node.control_step()
            except Exception as exc:
                problems.append(f'control_step() raised {type(exc).__name__}: {exc}')
                break

            if previous_mode == 'REPLAN' and node.mode == 'FOLLOW':
                first = node.full_path[0] if node.full_path else None
                replan_starts.append((first, node.prev_node))
            if node.mode == 'DONE':
                break

            linear, angular = SIM['command']
            x += linear * math.cos(heading) * timestep
            y += linear * math.sin(heading) * timestep
            heading = nav.angle_wrap(heading + angular * timestep)
            node.x, node.y, node.yaw = x, y, heading

            nearest_node = min(NODES, key=lambda point: math.dist(point, (x, y)))
            index = NODES[nearest_node]
            if math.dist(nearest_node, (x, y)) < 0.06:
                if not visited_nodes or visited_nodes[-1] != index:
                    visited_nodes.append(index)

            offset = min(
                math.dist(nav.nearest_point_on_polyline(list(edge), (x, y))[2], (x, y))
                for edge in CENTRE_LINES
            )
            max_offset = max(max_offset, offset)

    # Preserve the old harness's observable acceptance checks. Corridor
    # offset is geometric proximity, not a collision or clearance guarantee.
    goal_reached = node.mode == 'DONE'
    duplicate_removals = len(removed_edges) != len(set(removed_edges))
    backtrack_count = transitions.count('BACKTRACK')
    bad_starts = [(first, previous) for first, previous in replan_starts
                  if previous is not None and first != previous]
    checks = {
        'goal reached (mode DONE)': goal_reached,
        'only blocked edges removed': all(edge in blocked for edge in removed_edges),
        'no duplicate removals or backtrack/replan loop': (
            not duplicate_removals
            and backtrack_count <= len(removed_edges) + (0 if goal_reached else 1)
        ),
        'new plans start at previous node': not bad_starts,
        'corridor offset below 0.25 m': max_offset < 0.25,
        'no control_step errors': not problems,
    }
    return SimulationResult(
        goal_reached=goal_reached,
        mode_history=transitions,
        removed_edges=removed_edges,
        blocked_edges=blocked,
        first_path=first_path,
        visited_nodes=visited_nodes,
        replan_starts=replan_starts,
        max_offset=max_offset,
        elapsed=SIM['time'],
        problems=problems,
        checks=checks,
    )
