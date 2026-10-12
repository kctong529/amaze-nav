#!/usr/bin/env python3
import heapq
import math
import re
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Tuple, Dict, Set, List, Iterable, Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import sys

from perception import (
    Passability, analyze_edge, assess_passability, edge_to_world, plan_motion,
    points_to_edge_frame, select_next_local_passage,
)

Coord = Tuple[float, float]
Index = Tuple[int, int]
DIRS = {"up": (-1, 0), "down": (1, 0), "left": (0, -1), "right": (0, 1)}


@dataclass(frozen=True)
class PlanningResult:
    generation: int
    edge: Tuple[Coord, Coord]
    scan_sequence: int
    scan_received_at: float  # monotonic wall clock
    source_pose: Tuple[float, float, float]
    observation: object
    assessment: object
    centerline_occupied: bool
    motion_report: object
    elapsed_ms: float


def centerline_occupied(observation, robot_xy, window, radius) -> bool:
    """True if observed points lie on the edge centerline within ``window`` ahead."""
    if observation.length <= 1e-9:
        return False
    obstacle_points = observation.obstacle_points_st
    if len(obstacle_points) == 0:
        return False
    start = np.asarray(observation.start, dtype=float)
    end = np.asarray(observation.end, dtype=float)
    tangent = (end - start) / observation.length
    robot_s = float(np.dot(np.asarray(robot_xy, dtype=float) - start, tangent))
    ahead = obstacle_points[:, 0] - robot_s
    return bool(np.any((ahead >= 0.0) & (ahead <= window) &
                       (np.abs(obstacle_points[:, 1]) <= radius)))


def compute_planning_result(generation, edge, scan_sequence, scan_received_at,
                            source_pose, scan, laser_offset_xy, window, radius, corridor_bounds):
    """Worker-only function. Uses a sensor snapshot; never accesses the ROS node."""
    started = time.perf_counter()
    x, y, yaw = source_pose
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    valid = (np.isfinite(ranges) & (ranges >= scan.range_min) &
             (ranges <= scan.range_max))
    if valid.any():
        local_x = ranges[valid] * np.cos(angles[valid]) + laser_offset_xy[0]
        local_y = ranges[valid] * np.sin(angles[valid]) + laser_offset_xy[1]
        c, sn = math.cos(yaw), math.sin(yaw)
        points = np.column_stack((x + c * local_x - sn * local_y,
                                  y + sn * local_x + c * local_y))
    else:
        points = np.empty((0, 2), dtype=float)

    observation = analyze_edge(points, *edge)
    assessment = assess_passability(observation, (x, y),
                                    robot_radius=0.089, safety_margin=0.03,
                                     corridor_bounds=corridor_bounds)
    occupied = centerline_occupied(observation, (x, y), window, radius)

    report = plan_motion(observation, (x, y), robot_yaw=yaw)
    return PlanningResult(generation, edge, scan_sequence, scan_received_at,
                          source_pose, observation, assessment, occupied, report,
                          (time.perf_counter() - started) * 1000.0)

# ---------------- Grid graph ----------------
@dataclass
class GridGraph:
    """
    Lightweight grid graph whose nodes are (row, col) indices mapped
    to metric (x, y) world coordinates. Edges connect 4-neighborhood cells.
    """
    H: int
    W: int
    positions: Dict[Index, Coord]   # (r,c) -> (x,y)
    coord2idx: Dict[Coord, Index]   # (x,y) -> (r,c)
    adj: Dict[Index, Set[Index]]    # adjacency

    @classmethod
    def from_node_graph(cls, node_graph: List[List[List[float]]]):
        """
        Build a GridGraph from a 2D list of XY coordinates.

        Args:
            node_graph: HxW list; node_graph[r][c] is [x, y] for grid cell (r, c).

        Returns:
            GridGraph: A fully connected 4-neighborhood graph with
            (r,c)<->(x,y) mappings initialized.
        """
        H, W = len(node_graph), len(node_graph[0])
        positions: Dict[Index, Coord] = {}
        coord2idx: Dict[Coord, Index] = {}
        for r in range(H):
            for c in range(W):
                xy = tuple(float(v) for v in node_graph[r][c])
                positions[(r, c)] = xy
                coord2idx[xy] = (r, c)
        adj: Dict[Index, Set[Index]] = {(r, c): set() for r in range(H) for c in range(W)}
        for r in range(H):
            for c in range(W):
                for dr, dc in DIRS.values():
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < H and 0 <= nc < W:
                        adj[(r, c)].add((nr, nc))
                        adj[(nr, nc)].add((r, c))
        return cls(H, W, positions, coord2idx, adj)

    def index_of(self, coord: Coord, eps: float = 1e-9) -> Index:
        """
        Look up the grid index for a given (x, y) coordinate.

        Args:
            coord: (x, y) world coordinate.
            eps: absolute tolerance for fallback floating-point comparison.

        Returns:
            (r, c) integer grid index.

        Raises:
            KeyError: if the coordinate is not found (within tolerance).
        """
        x, y = float(coord[0]), float(coord[1])
        key = (x, y)
        if key in self.coord2idx:
            return self.coord2idx[key]
        for idx, (px, py) in self.positions.items():
            if math.isclose(px, x, abs_tol=eps) and math.isclose(py, y, abs_tol=eps):
                return idx
        raise KeyError(f"Coordinate {coord} not found.")

    def coord_of(self, idx: Index) -> Coord:
        """
        Get the (x, y) coordinate for a given grid index.

        Args:
            idx: (r, c) integer grid index.

        Returns:
            (x, y) world coordinate.
        """
        return self.positions[idx]

    def has_edge(self, a: Coord, b: Coord) -> bool:
        """
        Check whether the edge between two coordinates exists.

        Args:
            a: (x, y) of first node.
            b: (x, y) of second node.

        Returns:
            True if a<->b exists, else False.
        """
        ia, ib = self.index_of(a), self.index_of(b)
        return (ib in self.adj[ia]) and (ia in self.adj[ib])

    def remove_edge(self, a: Coord, b: Coord):
        """
        Remove the edge between two coordinates if the edge exists

        Args:
            a: (x, y) of first node.
            b: (x, y) of second node.
        """
        ia, ib = self.index_of(a), self.index_of(b)
        self.adj[ia].discard(ib)
        self.adj[ib].discard(ia)

    def edges(self) -> Iterable[Tuple[Coord, Coord]]:
        """
        Iterate all unique edges in coordinate space.

        Yields:
            Tuples (coord_a, coord_b) for each edge once.
        """
        seen = set()
        for a_idx, nbrs in self.adj.items():
            for b_idx in nbrs:
                key = tuple(sorted([a_idx, b_idx]))
                if key not in seen:
                    seen.add(key)
                    yield (self.coord_of(a_idx), self.coord_of(b_idx))

    def _compute_normalizer(self, extra_points: Optional[List[Coord]] = None,
                            img_size=700, margin=50):
        """
        Prepare a coordinate->pixel mapping for visualization.

        Args:
            extra_points: optional extra (x, y) points to include in bounds.
            img_size: output square image size in pixels.
            margin: padding in pixels.

        Returns:
            (to_px, img_size, margin) where to_px(x, y)->(u, v) pixel function.
        """
        xs = [p[0] for p in self.positions.values()]
        ys = [p[1] for p in self.positions.values()]
        if extra_points:
            xs.extend([p[0] for p in extra_points])
            ys.extend([p[1] for p in extra_points])
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        pad_x = 0.05 * (xmax - xmin if xmax > xmin else 1.0)
        pad_y = 0.05 * (ymax - ymin if ymax > ymin else 1.0)
        xmin -= pad_x; xmax += pad_x
        ymin -= pad_y; ymax += pad_y
        xr = xmax - xmin if xmax > xmin else 1.0
        yr = ymax - ymin if ymax > ymin else 1.0
        S = img_size - 2 * margin
        def to_px(x, y):
            nx = (x - xmin) / xr
            ny = (y - ymin) / yr
            nx = 0.0 if nx < 0.0 else (1.0 if nx > 1.0 else nx)
            ny = 0.0 if ny < 0.0 else (1.0 if ny > 1.0 else ny)
            u = margin + int(nx * S)
            v = margin + int((1.0 - ny) * S)
            return (u, v)
        return to_px, img_size, margin

    def base_image(self, extra_points: Optional[List[Coord]] = None,
                   img_size=700, margin=50) -> Tuple[np.ndarray, callable]:
        """
        Create a blank white image and a to_px mapping for drawing the graph.

        Args:
            extra_points: optional (x, y) points to include in normalization.
            img_size: output image size in pixels.
            margin: border margin in pixels.

        Returns:
            (img, to_px) where img is an (H x H x 3) uint8 canvas,
            and to_px is a function mapping (x, y)->(u, v).
        """
        to_px, H, _ = self._compute_normalizer(extra_points, img_size, margin)
        img = np.ones((H, H, 3), dtype=np.uint8) * 255
        return img, to_px

# -------- path util ----------
def nearest_point_on_polyline(path: List[Coord], p: Coord) -> Tuple[float, int, Coord]:
    """
    Project a point onto a polyline and find the closest point/segment.

    Args:
        path: list of waypoints [(x, y), ...].
        p: query point (x, y).

    Returns:
        (s, seg_idx, proj) where:
          - s is arclength from path[0] to the projection point (meters),
          - seg_idx is the index of the segment [path[i], path[i+1]] containing proj,
          - proj is the projected (x, y) point on the path.
    """
    if len(path) == 1:
        return 0.0, 0, path[0]
    def dot(a, b): return a[0]*b[0] + a[1]*b[1]
    def sub(a, b): return (a[0]-b[0], a[1]-b[1])
    def add(a, b): return (a[0]+b[0], a[1]+b[1])
    cum = [0.0]
    for i in range(1, len(path)):
        cum.append(cum[-1] + math.dist(path[i-1], path[i]))
    best_s = 0.0; best_pt = path[0]; best_seg = 0
    for i in range(1, len(path)):
        a = path[i-1]; b = path[i]
        ab = sub(b, a); ap = sub(p, a)
        ab2 = dot(ab, ab) if ab != (0.0, 0.0) else 1e-9
        t = max(0.0, min(1.0, dot(ap, ab) / ab2))
        proj = add(a, (ab[0]*t, ab[1]*t))
        s_here = cum[i-1] + math.dist(a, proj)
        if i == 1 or math.dist(p, proj) < math.dist(p, best_pt):
            best_s = s_here; best_pt = proj; best_seg = i - 1
    return best_s, best_seg, best_pt

def lookahead_point(path: List[Coord], p: Coord, Ld: float) -> Coord:
    """
    Find the point at arclength along a polyline from the projection of p.

    Args:
        path: list of waypoints [(x, y), ...].
        p: current position (x, y).
        Ld: lookahead distance in meters.

    Returns:
        (x, y) of the lookahead point clamped to the end of the path.
    """
    if not path: return p
    if len(path) == 1: return path[0]
    s, _, _ = nearest_point_on_polyline(path, p)
    cum = [0.0]
    for i in range(1, len(path)):
        cum.append(cum[-1] + math.dist(path[i-1], path[i]))
    target_s = s + Ld
    if target_s >= cum[-1]: return path[-1]
    for i in range(1, len(path)):
        if cum[i] >= target_s:
            seg_s = target_s - cum[i-1]
            a = path[i-1]; b = path[i]
            seg_len = math.dist(a, b)
            t = seg_s / seg_len
            return (a[0] + (b[0]-a[0]) * t, a[1] + (b[1]-a[1]) * t)
    return path[-1]

def angle_wrap(a: float) -> float:
    """
    Wrap an angle to the range [-pi, pi].

    Args:
        a: angle in radians.

    Returns:
        Wrapped angle in radians.
    """
    while a > math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a

def yaw_from_quat(x, y, z, w) -> float:
    """
    Extract yaw (heading about +Z) from a quaternion.

    Args:
        x, y, z, w: quaternion components.

    Returns:
        Yaw in radians in the odometry frame
    """
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

# --------- Simple PID ----------
@dataclass
class PID:
    """
    Minimal PID controller with anti-windup (integrator clamping).
    Call update(e, dt) each cycle to get the control effort.
    """
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    i: float = 0.0
    prev_e: Optional[float] = None
    i_limit: float = 1.0
    angle_error: bool = False  # Wrap derivative across +/-pi for yaw errors.
    derivative_alpha: float = 0.25  # Low-pass filter LiDAR/odometry noise.
    filtered_d: float = 0.0

    def reset(self):
        """
        Reset internal integrator and derivative memory.
        """
        self.i = 0.0
        self.prev_e = None
        self.filtered_d = 0.0

    def update(self, e: float, dt: float) -> float:
        """
        Compute PID output for a given error and timestep.

        Args:
            e: control error (setpoint - measurement).
            dt: time step in seconds.

        Returns:
            Control effort (float).
        """
        if dt <= 0.0 or self.prev_e is None:
            raw_d = 0.0
        else:
            de = angle_wrap(e - self.prev_e) if self.angle_error else e - self.prev_e
            raw_d = float(np.clip(de / dt, -3.0, 3.0))
        self.filtered_d = ((1.0 - self.derivative_alpha) * self.filtered_d
                           + self.derivative_alpha * raw_d)
        if dt > 0.0:
            self.i += e * dt
            self.i = float(np.clip(self.i, -self.i_limit, self.i_limit))
        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * self.filtered_d

# --------------- Node ---------------
class GridPathFollowerNode(Node):
    """
    ROS2 node that:
    - Plans on a grid using A* between start/goal nodes,
    - Follows the path with a suitable control mechanism,
    - Backtracks and replans if an obstacle is not passable
    """
    def __init__(self):
        """
        Initialize parameters, build graph, subscribe/publish ROS topics,
        and compute the initial path.
        """
        super().__init__("grid_path_follower")

        # --- Motion and geometry parameters ---
        self.goal_tol = 0.05
        self.node_tol = 0.05
        # TODO: YOUR CODE HERE: ~3 lines: set the lookahead distance for pure-pursuit based control, and the maximum allowed linear and angular velocities
        self.lookahead_dist = 0.25  # Initial test value, meters
        self.v_max = 0.22          # Fixed maximum forward speed (m/s).
        self.w_max = 2.8           # Maximum angular speed (rad/s).
        self.v_average = 0.22      # Nominal forward speed, constrained by curvature/braking.
        self.rotate_heading_tol = math.radians(70.0)  # Only recover from gross misalignment
        self.braking_deceleration = 0.65  # Brake a little later; keep the full stop at nodes.

        # Controller context set by FOLLOW; None selects conservative speed.
        self._controller_segment = None
        self._controller_waypoint_distance = None
        # Reserved for future PID-assisted tracking; not used by pure pursuit.
        # Cross-track correction for the currently proposed translation polyline.
        # Keep Ki at zero until an actual tracking bias has been measured.
        self.pid_angular = PID(kp=1.2, ki=0.0, kd=0.08, i_limit=0.2)
        self.max_pid_correction = 0.35
        # Optional, narrow-passage-only centering. Never replaces plan validation.
        self.pid_centering = PID(kp=0.8, ki=0.0, kd=0.05)
        # Separate rotation controllers avoid derivative history crossing modes.
        self.pid_edge_rotation = PID(kp=1.5, ki=0.0, kd=0.08, angle_error=True)
        self.pid_maneuver_rotation = PID(kp=1.5, ki=0.0, kd=0.08, angle_error=True)
        self.center_max_w = 0.12
        self.center_max_speed = 0.15  # Modest increase only when narrow-passage centering is active.
        self.center_full_strength_width = 0.35
        self.center_max_passage_width = 0.45
        # Precise heading is required only BEFORE a confirmed narrow entrance.
        # Never force an in-place turn once the robot is inside the throat.
        self.narrow_entry_width = 0.35
        self.narrow_entry_yaw_tolerance = math.radians(2.0)
        self.narrow_entry_max_turn_w = 0.30
        self.narrow_entry_max_distance = 0.55
        self._narrow_entry_log_at = -1e9
        self._center_log_at = -1e9
        self.tracking_path: Optional[List[Coord]] = None
        self.local_clearance: Optional[float] = None
        self.tight_speed = 0.19
        self.cautious_speed = 0.21
        self.tight_clearance = 0.14
        self.free_clearance = 0.22
        # Classification alone does not justify deleting a graph edge.
        self.block_forward_window = 0.45
        self.block_centerline_radius = 0.119
        # Trusted *physical* wall-to-wall corridor bounds in edge-frame t.
        # Leave empty until surveyed; never infer these from LiDAR extrema.
        # Populate both directions separately, since reversing an edge flips t.
        self.trusted_corridor_bounds = {}

        # --- Helpers to halt turtlebot ---
        self.dwell_s = 0.0  # No routine pause; explicit safety stops still apply
        self.dwell_until: Optional[float] = None

        # --- Start and goal positions. TODO: OPTIONAL: modify for testing purposes as needed ---
        self.start_pos = (0.0, 0.0)
        self.goal_pos  = (7.120, -5.696)

        # --- State ---
        self.pose_ready = False                    # to check if odometry values are being received
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0 # turtlebot pose
        self.scan: Optional[LaserScan] = None      # stores the latest scan
        self.mode = 'None'                         # operation mode
        # Internal safety latch; never exposed as a fifth navigation state.
        self.safety_hold_reason: Optional[str] = None
        # Per-edge odometry breadcrumbs, traversed in reverse without a 180-degree turn.
        self.edge_pose_history = []
        self.backtrack_history = []
        self.backtrack_cursor = -1
        self.backtrack_phase = None  # PLAN_RETURN, FOLLOW_RETURN (internal only)
        self.backtrack_reverse_origin = None
        self.backtrack_reverse_target = None
        self.backtrack_return_edge = None
        self.backtrack_reverse_distance = 0.10
        self.backtrack_arrival_logged = False
        self.backtrack_sample_spacing = 0.025
        self.backtrack_max_sample_jump = 0.16
        self.backtrack_speed_max = 0.045
        self.backtrack_lookahead = 0.10
        self.backtrack_heading_limit = math.radians(20.0)
        self.backtrack_rear_sector = math.radians(45.0)
        self.backtrack_scan_max_gap = math.radians(5.0)
        self.backtrack_check_distance = 0.16
        self.backtrack_log_at = -1e9
        self.last_t = self._now()

        # --- Grid (5x5) ---
        self.node_graph = [
            [[0.712, 0], [2.136, 0], [3.560, 0], [4.984, 0], [6.408, 0]],
            [[0.712, -1.424], [2.136, -1.424], [3.560, -1.424], [4.984, -1.424], [6.408, -1.424]],
            [[0.712, -2.848], [2.136, -2.848], [3.560, -2.848], [4.984, -2.848], [6.408, -2.848]],
            [[0.712, -4.272], [2.136, -4.272], [3.560, -4.272], [4.984, -4.272], [6.408, -4.272]],
            [[0.712, -5.696], [2.136, -5.696], [3.560, -5.696], [4.984, -5.696], [6.408, -5.696]],
        ]
        # Graph and initial plan
        self.graph = GridGraph.from_node_graph(self.node_graph)
        """
        GridGraph.from_node_graph returns:
                 GridGraph(
                    H=5, 
                    W=5, 
                    positions={(0, 0): (0.712, 0.0), (0, 1): (2.136, 0.0), ....}
                    coord2idx={(0.712, 0.0): (0, 0), (2.136, 0.0): (0, 1), ....}
                    adj={(0, 0): {(1, 0), (0, 1)}, (0, 1): {(0, 2), (1, 1), (0, 0)}, (0, 2): {(0, 1), (1, 2), (0, 3)}, ........., (4, 4): {(3, 4), (4, 3)}}
                   )
        """
        self.full_path:  List[Coord] = []
        self.prev_node : Optional[Coord] = None
        self.block_edge: Optional[Tuple[Coord, Coord]] = None
        self.active_edge: Optional[Tuple[Coord, Coord]] = None
        self.latest_scan_time: Optional[float] = None
        self.perception_observation = None
        self.motion_report = None
        self.motion_target: Optional[Coord] = None
        self.rotation_target_yaw: Optional[float] = None
        self.rotation_position: Optional[Coord] = None
        # Explicit maneuver execution: APPROACH -> ROTATE -> RESCAN.
        self.maneuver_stage = None
        self.maneuver_scan_sequence = -1
        self.min_planning_scan_sequence = -1
        self.rotation_tolerance = math.radians(5.0)
        # Unsafe small in-place correction: allow a bounded stationary replan,
        # never waive footprint clearance or execute an unvalidated translation.
        self.small_rotation_retry_limit = math.radians(15.0)
        self.unsafe_rotation_max_replans = 2
        self.unsafe_rotation_replans = 0
        self.unsafe_rotation_retry_edge = None
        self.robot_clearance_radius = 0.119  # 0.089 m radius + 0.03 m margin
        self.translation_guard_distance = 0.50
        # One fresh narrow-gap BLOCKED assessment rejects the current edge.
        # UNKNOWN never rejects an edge solely because planning is slow.
        self._last_block_diagnostic_at = -1e9
        self.scan_sequence = 0
        self.edge_heading_tolerance = math.radians(8.0)  # Normal-edge alignment; do not deadlock at startup.
        self.aligned_edge = None
        self.edge_alignment_rate = 1.5
        self.edge_alignment_w_max = 1.15  # Cap turning speed during node alignment.
        self.backtrack_settle_s = 0.30  # Stop before replan and next edge alignment.
        # Only used for the first FOLLOW edge immediately after REPLAN.
        self._stabilize_after_replan = False
        self._post_replan_hysteresis_until = 0.0
        self._alignment_settle_started = None
        self._alignment_settle_duration = 0.06  # Slightly shorter pause; measured yaw-rate gate stays unchanged.
        self._alignment_angular_velocity_tol = 0.12  # Avoid an excessively strict stationary gate at startup.
        self._follow_tracking_log_at = -1e9
        self.backtrack_arrival_tol = 0.08  # Avoid turning in place over a few centimetres.
        self.backtrack_speed_max = 0.045  # Maximum reverse speed for checked short retreat.
        self.reverse_heading_tolerance = math.radians(20.0)
        self.reverse_lateral_tolerance = 0.12
        self.reverse_scan_max_gap = math.radians(8.0)
        self.reverse_rear_sector = math.radians(35.0)
        self.backtrack_turn_max = 0.8  # Recovery-only angular limit (rad/s).
        self.backtrack_turn_threshold = math.radians(25.0)
        self.backtrack_near_turn_threshold = math.radians(12.0)
        self.backtrack_near_distance = 0.30
        self._backtrack_log_at = -1e9
        self.backtrack_measured_v = None
        self.backtrack_measured_w = None
        # Contact diagnostics only: these counters do NOT change motion or safety.
        self._contact_last_cmd_v = 0.0
        self._contact_last_cmd_w = 0.0
        self._contact_stall_since = None
        self._contact_last_warning_at = -1e9
        self._contact_start_pose = None
        self.edge_alignment_min_angular_speed = 0.10
        self._alignment_log_time = -1e9
        self._planner_log_time = -1e9
        self._planner_slow_threshold_ms = 50.0
        self.scan_max_age = 0.5
        # Plan validity is judged by how far the robot moved/turned since the
        # plan's source pose, not by age: the robot is stopped while it waits,
        # so a 2-s-old plan from the same pose is still valid. The age limit is
        # only a backstop and must exceed the worst-case worker time.
        self.plan_max_age = 3.0  # Backstop, from the SOURCE SCAN.
        self.plan_pose_max_displacement = 0.10
        self.plan_max_yaw_change = math.radians(25.0)
        self.fast_block_last_scan = -1
        # Consecutive fresh planner failures are a separate operational state,
        # NOT a geometric BLOCKED classification.
        self.no_motion_failure_count = 0
        self.no_motion_last_scan = -1
        self.no_motion_threshold = 3
        self.no_motion_min_duration = 2.0
        self.no_motion_since = None
        self.recovery_requested = False
        # Bounded stationary retries for planner failures; never mark an edge
        # BLOCKED solely because plan_motion() found no local trajectory.
        self.no_motion_recovery_attempts = 0
        self.no_motion_max_recoveries = 2
        self.no_motion_recovery_edge = None

        self.planning_interval = 0.20  # Request at most 5 jobs/s; never overlap.
        self._planner_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="lidar-planner")
        self._planner_future = None
        self._planning_generation = 0
        self._planning_latest = None
        self._last_submit_at = -1e9
        self._last_submitted_scan = -1
        self._last_consumed_scan = -1
        self._accepted_at = None
        self._accepted_scan_at = None
        self._accepted_edge = None
        self._accepted_scan_sequence = -1
        self.accepted_plan_max_age = 3.0  # Worker can take >2 s; still require fresh LiDAR.
        self._worker_warning_at = -1e9
        self._plan_diagnostic_at = -1e9
        self._last_reported_plan_scan = -1
        self._last_scan_wall = None
        self.laser_offset_xy = (-0.032, 0.0)  # Verified base_footprint -> base_scan XY offset in Gazebo.

        #TODO: OPTIONAL: you can manually remove edges here using 'self.graph.remove_edge([..., ...], [..., ...]))

        # --- ROS I/O ---
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 20)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 20)
        self.timer = self.create_timer(1.0/20.0, self.control_step)

        # --- Modes of operation ---
        """
            1. FOLLOW: when in default mode of following the A* generated path  
            2. BACKTRACK: when an unpassable obstacle is encountered (go back to immediately previous node) 
            3. REPLAN: recalculate A* to get new path to the goal from the current node
            4. DONE: when goal has been reached
        """
        self._set_mode('FOLLOW')

        # --- Vizualization ---
        self.win = "Grid A* Path Follower"
        cv2.namedWindow(self.win, cv2.WINDOW_AUTOSIZE)

        # --- Initial plan ----
        self.path_plan(self.start_pos)
        self.get_logger().info(f"Planned {len(self.full_path)} waypoints.")

    # --- OPTIONAL helper: time ---
    def _now(self) -> float:
        """
        Get ROS2 clock time in seconds (float).

        Returns:
            Current time in seconds as a float.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    # --- Helper to set the mode of operation ---
    def _contact_pose_log(self, event: str):
        if not self.pose_ready:
            return
        edge = self.active_edge
        xte = float('nan')
        if edge is not None:
            (ax, ay), (bx, by) = edge
            length = math.hypot(bx - ax, by - ay)
            if length > 1e-9:
                xte = ((bx - ax) * (self.y - ay) -
                       (by - ay) * (self.x - ax)) / length
        self.get_logger().warn(
            f'[CONTACT DIAG] {event}: mode={self.mode} '
            f'pose=({self.x:.3f},{self.y:.3f},yaw={math.degrees(self.yaw):+.1f}deg) '
            f'edge={edge} xte={xte:+.3f}m '
            f'cmd=({self._contact_last_cmd_v:+.3f},'
            f'{self._contact_last_cmd_w:+.3f}) '
            f'odom=({self.backtrack_measured_v},'
            f'{self.backtrack_measured_w})')

    def _set_mode(self, new_mode: str):
        """
        Switch high-level mode and enforce a short dwell (pause).

        Args:
            new_mode: one of {'FOLLOW', 'BACKTRACK', 'REPLAN', 'DONE'}.
        """
        if new_mode not in ('FOLLOW', 'BACKTRACK', 'REPLAN', 'DONE'):
            raise ValueError(f'Invalid navigation state: {new_mode}')
        if self.mode != new_mode:
            if (self.mode in ('FOLLOW', 'BACKTRACK', 'REPLAN') or
                    new_mode in ('BACKTRACK', 'REPLAN')):
                self._contact_pose_log(f'before {self.mode} -> {new_mode}')
            self.mode = new_mode
            self.pid_centering.reset()
            self.pid_edge_rotation.reset()
            self.pid_maneuver_rotation.reset()
            self.publish_stop()
            self.dwell_until = self._now() + self.dwell_s
            self.get_logger().info(f"Mode -> {self.mode}, pausing {self.dwell_s}s")

    def _safety_hold(self, reason: str):
        """Latch zero-velocity control without introducing another navigation mode.

        A stopped FOLLOW or BACKTRACK remains in that high-level state. Manual
        inspection/reset is required; it must not automatically resume on a
        later scan or on completion of an in-flight planning worker.
        """
        self.publish_stop()
        if self.safety_hold_reason is None:
            self.safety_hold_reason = reason
            self.get_logger().error(f'Safety hold in {self.mode}: {reason}')
        self._invalidate_planning()

    # --- planning ---
    # --- A* implementation ---
    def astar_path(self, graph: GridGraph, start: Coord, goal: Coord) -> List[Coord]:
        """
        Run A* on the GridGraph between two coordinates.

        Args:
            graph: GridGraph instance.
            start: (x, y) start coordinate
            goal:  (x, y) goal coordinate 

        Returns:
            List of (x, y) coordinates along the path from start node to goal node,
            inclusive. Empty list if no path.

        Hint: use the relevant GriGraph functions)
        """
        start_idx = graph.index_of(start)
        goal_idx = graph.index_of(goal)
        def h(i: Index) -> float:
            x1, y1 = graph.coord_of(i); x2, y2 = graph.coord_of(goal_idx)
            return math.hypot(x2 - x1, y2 - y1)
        def c(i: Index, j: Index) -> float:
            x1, y1 = graph.coord_of(i); x2, y2 = graph.coord_of(j)
            return math.hypot(x2 - x1, y2 - y1)
        open_heap = [(h(start_idx), start_idx)]
        came: Dict[Index, Index] = {}
        g = {start_idx: 0.0}
        in_open = {start_idx}
        while open_heap:
            _, cur = heapq.heappop(open_heap)
            in_open.discard(cur)
            if cur == goal_idx:
                path_idx = [cur]
                while cur in came:
                    cur = came[cur]
                    path_idx.append(cur)
                path_idx.reverse()
                return [graph.coord_of(i) for i in path_idx]
            for nbr in graph.adj[cur]:
                t = g[cur] + c(cur, nbr)
                if t < g.get(nbr, float('inf')):
                    came[nbr] = cur
                    g[nbr] = t
                    f = t + h(nbr)
                    if nbr not in in_open:
                        heapq.heappush(open_heap, (f, nbr))
                        in_open.add(nbr)
        return []

    def debug_stop(self, reason: str):
        """Log a stop reason at most once per second."""
        now = self._now()
        if now - getattr(self, "_last_stop_log", -1e9) >= 1.0:
            self.get_logger().warn(f"Navigation stopped: {reason}")
            self._last_stop_log = now
        self.publish_stop()

    def nearest_graph_coord(self, p: Coord) -> Coord:
        """
        Find the nearest graph node (in metric space) to a given point.

        Args:
            p: (x, y) query position.

        Returns:
            (x, y) of the closest node in the GridGraph.
        """
        best = None; best_d = float('inf')
        for xy in self.graph.coord2idx.keys():
            d = math.dist(p, xy)
            if d < best_d: best_d = d; best = xy
        return best

    # ---  Invoke A* to return the nodes that define path ---
    def plan_from(self, start_pose: Coord) -> List[Coord]:
        """
        Build a waypoint list: start_pose -> nearest start node -> A* nodes
        -> nearest goal node -> goal.

        Args:
            start_pose: (x, y) where to start planning from.

        Returns:
            List of (x, y) waypoints including start_pose and goal_pos.
        """
        # TODO: YOUR CODE HERE: ~ 3 lines
        # 1. Get the closest graph node to the robot’s current starting position (ns)
        # 2. Get the closest graph node to the goal position (ng)
        # 3. Run A* to get a path (sequence of graph nodes) from ns → ng
        ns = self.nearest_graph_coord(start_pose)
        ng = self.nearest_graph_coord(self.goal_pos)
        grid_path = self.astar_path(self.graph, ns, ng)

        path: List[Coord] = [start_pose]

        # TODO: YOUR CODE HERE: ~8 lines
        # 5. If the start node is different from the current robot position, append it to 'path'
        # 6. Append all waypoints from the A* grid path (avoiding duplicate points)
        # 7. Ensure the final node (ng) is included
        # 8. Ensure the actual goal position is included as the last waypoint
        if not grid_path:
            self.get_logger().warn("A* found NO route to the goal!")
            return path
        if math.dist(start_pose, ns) > 1e-6:
            path.append(ns)
        for p in grid_path:
            if math.dist(path[-1], p) > 1e-6:
                path.append(p)
        if math.dist(path[-1], ng) > 1e-6:
            path.append(ng)
        goal = (float(self.goal_pos[0]), float(self.goal_pos[1]))
        if math.dist(path[-1], goal) > 1e-6:
            path.append(goal)

        # Return the full planned path as a list of coordinates
        return path

    def path_plan(self, start_from: Optional[Coord] = None):
        """
        (Re)plan a global path and store it in self.full_path.

        Args:
            start_from: optional (x, y) to start planning from; if None,
                        uses current odom pose (if available) else start_pos.
        """
        if start_from is None:
            start_from = (self.x, self.y) if self.pose_ready else self.start_pos
        self.full_path = self.plan_from(start_from)
        self.get_logger().info(f"Path planned from {start_from} with {len(self.full_path)} waypoints.")

    # --- callbacks ---
    def odom_cb(self, msg: Odometry):
        """
        Odometry callback. Updates pose estimate (x, y, yaw).

        Args:
            msg: nav_msgs/Odometry message.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.backtrack_measured_v = float(msg.twist.twist.linear.x)
        self.backtrack_measured_w = float(msg.twist.twist.angular.z)
        self.pose_ready = True
        # Sustained commanded-forward motion with essentially no measured
        # movement MAY mean contact, wheel slip, or an odometry issue.
        # Report suspicion only; never infer collision from odometry alone.
        now_wall = time.monotonic()
        if (self.mode in ('FOLLOW', 'BACKTRACK')
                and self._contact_last_cmd_v >= 0.10
                and abs(self.backtrack_measured_v) <= 0.025):
            if self._contact_stall_since is None:
                self._contact_stall_since = now_wall
                self._contact_start_pose = (self.x, self.y)
            elif (now_wall - self._contact_stall_since >= 0.7
                  and now_wall - self._contact_last_warning_at >= 2.0):
                displacement = math.dist((self.x, self.y), self._contact_start_pose)
                if displacement < 0.025:
                    self._contact_pose_log(
                        f'POSSIBLE CONTACT: cmd forward but odom nearly still '
                        f'for {now_wall-self._contact_stall_since:.1f}s; '
                        f'pose displacement={displacement:.3f}m')
                    self._contact_last_warning_at = now_wall
        else:
            self._contact_stall_since = None
            self._contact_start_pose = None
        # Keep actual traversed poses, not the intended/possibly curved local plan.
        if self.mode == 'FOLLOW' and self.active_edge is not None:
            pose = (self.x, self.y, self.yaw)
            if not self.edge_pose_history:
                self.edge_pose_history.append(pose)
            else:
                delta = math.dist(pose[:2], self.edge_pose_history[-1][:2])
                if delta >= self.backtrack_sample_spacing:
                    if delta > self.backtrack_max_sample_jump:
                        # Do not silently bridge a localization discontinuity.
                        self.edge_pose_history.clear()
                        self.get_logger().error('Lost backtrack history: odometry pose jumped')
                    else:
                        self.edge_pose_history.append(pose)

    def scan_cb(self, msg: LaserScan):
        """
        LaserScan callback. Stores the latest scan.

        Args:
            msg: sensor_msgs/LaserScan message.
        """
        self.scan = msg
        self.latest_scan_time = self._now()
        self._last_scan_wall = time.monotonic()
        self.scan_sequence += 1

    def _scan_points_world(self) -> Optional[np.ndarray]:
        """Project fresh LaserScan endpoints using the current odometry pose.

        The scan is assumed planar and aligned with base_link. This is an
        approximation, not timestamp-matched TF. Check scanner extrinsics
        and timestamp consistency before autonomous driving.
        """
        if (self.scan is None or not self.pose_ready or
                self.latest_scan_time is None or
                self._now() - self.latest_scan_time > self.scan_max_age):
            return None
        scan = self.scan
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = (np.isfinite(ranges) & (ranges >= scan.range_min) &
                 (ranges <= scan.range_max))
        if not valid.any():
            return np.empty((0, 2), dtype=float)
        local_x = ranges[valid] * np.cos(angles[valid]) + self.laser_offset_xy[0]
        local_y = ranges[valid] * np.sin(angles[valid]) + self.laser_offset_xy[1]
        c, sn = math.cos(self.yaw), math.sin(self.yaw)
        return np.column_stack((
            self.x + c * local_x - sn * local_y,
            self.y + sn * local_x + c * local_y,
        ))

    def _update_perception(self, edge: Tuple[Coord, Coord]) -> bool:
        """Refresh one edge observation; False means missing or stale scan."""
        points = self._scan_points_world()
        if points is None:
            self.perception_observation = None
            self.motion_report = None
            return False
        self.perception_observation = analyze_edge(points, *edge)
        return True

    def _invalidate_planning(self):
        """Invalidate completed and in-flight results (workers cannot be killed)."""
        self.no_motion_failure_count = 0
        self.no_motion_last_scan = -1
        self.no_motion_since = None
        self._planning_generation += 1
        self._planning_latest = None
        self._last_consumed_scan = -1
        self.motion_target = None
        self.motion_report = None
        self.tracking_path = None
        self.local_clearance = None
        self.rotation_target_yaw = None
        self.rotation_position = None
        self.maneuver_stage = None
        self.maneuver_scan_sequence = -1
        self._accepted_at = None
        self._accepted_scan_at = None
        self._accepted_edge = None
        self._accepted_scan_sequence = -1

    def _poll_planning_worker(self):
        """Nonblocking completion handling, called only in ROS control callback."""
        future = self._planner_future
        if future is None or not future.done():
            return
        self._planner_future = None
        try:
            result = future.result()  # Already done; this does not wait.
        except Exception as exc:
            self.get_logger().error(f"Asynchronous planning failed: {exc!r}")
            return
        if result.elapsed_ms >= 50.0 and time.monotonic() - self._worker_warning_at >= 1.0:
            self.get_logger().warn(f"Slow planning worker: {result.elapsed_ms:.1f} ms; "
                                   "control callback remained nonblocking")
            self._worker_warning_at = time.monotonic()
        if result.generation != self._planning_generation or result.edge != self.active_edge:
            return
        # Old source measurements cannot become fresh merely because the worker finished.
        if time.monotonic() - result.scan_received_at > self.plan_max_age:
            self.get_logger().warn(
                f"Discarded completed plan from scan {result.scan_sequence}: "
                f"source scan is {time.monotonic() - result.scan_received_at:.2f}s old")
            return
        self._planning_latest = result
        # Count distinct, completed plans only. Pending computations are not
        # failures, and an accepted motion (including rotation) resets the count.
        if result.scan_sequence != self.no_motion_last_scan:
            self.no_motion_last_scan = result.scan_sequence
            if result.motion_report.plan is not None and self.mode == 'FOLLOW':
                self.no_motion_recovery_attempts = 0
            if result.motion_report.plan is None:
                if self.no_motion_failure_count == 0:
                    self.no_motion_since = time.monotonic()
                self.no_motion_failure_count += 1
            else:
                self.no_motion_failure_count = 0
                self.no_motion_since = None
        if result.scan_sequence != self._last_reported_plan_scan:
            self._last_reported_plan_scan = result.scan_sequence
            plan = result.motion_report.plan
            if plan is None:
                self.get_logger().warn(
                    f"Planner scan {result.scan_sequence}: NO feasible motion "
                    f"(mode={result.motion_report.mode}, "
                    f"obstacles={len(result.observation.obstacle_points_st)}, "
                    f"elapsed={result.elapsed_ms:.0f}ms)")
            else:
                self.get_logger().info(
                    f"Planner scan {result.scan_sequence}: "
                    f"mode={plan.mode}, rotation={plan.requires_rotation}, "
                    f"segments={len(plan.segments_st)}, "
                    f"elapsed={result.elapsed_ms:.0f}ms")

    def _request_planning(self, edge):
        """Submit one immutable snapshot, never queue overlapping work."""
        if self._planner_future is not None:
            return
        now = time.monotonic()
        if (now - self._last_submit_at < self.planning_interval or
                self.scan is None or not self.pose_ready or
                self._last_scan_wall is None or
                now - self._last_scan_wall > self.scan_max_age or
                self.scan_sequence == self._last_submitted_scan or
                self.scan_sequence <= self.min_planning_scan_sequence):
            return
        self._last_submit_at = now
        self._last_submitted_scan = self.scan_sequence
        pose = (float(self.x), float(self.y), float(self.yaw))
        self._planner_future = self._planner_pool.submit(
            compute_planning_result, self._planning_generation, edge,
            self.scan_sequence, self._last_scan_wall, pose, self.scan,
            tuple(self.laser_offset_xy), self.block_forward_window,
            self.block_centerline_radius, self.trusted_corridor_bounds.get(edge))

    def _current_planning_result(self, edge):
        """Require latest scan and an edge-, time-, and pose-compatible plan."""
        result = self._planning_latest
        now = time.monotonic()
        if result is None or self._last_scan_wall is None:
            return None
        if (result.generation != self._planning_generation or
                result.edge != edge or
                now - self._last_scan_wall > self.scan_max_age or
                now - result.scan_received_at > self.plan_max_age or
                math.dist((self.x, self.y), result.source_pose[:2]) >
                self.plan_pose_max_displacement):
            return None
        yaw_change = abs(angle_wrap(self.yaw - result.source_pose[2]))
        if (yaw_change > self.plan_max_yaw_change and
                not self._is_in_place_rotation(result, (self.x, self.y))):
            return None
        return result

    def _is_in_place_rotation(self, result, robot_pos) -> bool:
        """True if the plan's current step is rotating in place at robot_pos.

        Its target is an absolute yaw, so it stays valid while the robot turns;
        without this exemption the yaw gate would halt every rotation after 5 deg.
        """
        plan = result.motion_report.plan
        if plan is None or not plan.requires_rotation or plan.rotation_st is None:
            return False
        rotation_xy = edge_to_world(*plan.rotation_st, result.observation.start,
                                    result.observation.end)
        return math.dist(robot_pos, rotation_xy) <= self.node_tol

    def _apply_planning_result(self, result, robot_pos):
        """Latch exactly the FIRST executable leg; never execute hypothetical post-turn legs."""
        self.perception_observation = result.observation
        self.motion_report = result.motion_report
        self.motion_target = None
        self.tracking_path = None
        self.local_clearance = None
        self.rotation_target_yaw = None
        self.rotation_position = None
        self.maneuver_stage = None
        plan = result.motion_report.plan
        if plan is None:
            return
        if plan.requires_rotation:
            if plan.rotation_st is None or plan.departure_yaw is None:
                return
            self.rotation_target_yaw = plan.departure_yaw
            self.pid_maneuver_rotation.reset()
            self.rotation_position = edge_to_world(
                *plan.rotation_st, result.observation.start, result.observation.end)
            self.maneuver_scan_sequence = result.scan_sequence
            self.maneuver_stage = ('ROTATE' if math.dist(robot_pos, self.rotation_position)
                                   <= self.node_tol else 'APPROACH')
        if plan.segments_st and len(plan.segments_st[0]) >= 2:
            segment = [edge_to_world(st[0], st[1], result.observation.start,
                                     result.observation.end)
                       for st in plan.segments_st[0]]
            self.tracking_path = segment
            self.local_clearance = plan.clearance
            if self.maneuver_stage == 'APPROACH':
                # Follow the first leg ONLY up to the rotation station.
                self.motion_target = lookahead_point(segment, robot_pos,
                                                      self.lookahead_dist)
            elif self.maneuver_stage is None:
                self.motion_target = lookahead_point(segment, robot_pos,
                                                      self.lookahead_dist)
        if self.maneuver_stage is not None:
            if self.maneuver_stage == 'ROTATE':
                clear, details = self._rotation_clearance_details(robot_pos)
                if clear is False:
                    self.get_logger().warn(
                        f'Proposed in-place rotation station is currently obstructed: {details}')
                elif clear is None:
                    self.get_logger().warn(
                        f'Cannot verify proposed rotation station yet: {details}')
                else:
                    self.get_logger().info(f'Rotation station clearance: {details}')
            self.get_logger().info(
                f"Maneuver {plan.mode}: stage={self.maneuver_stage}, "
                f"rotation_station={self.rotation_position}, "
                f"heading={math.degrees(self.rotation_target_yaw):.1f}deg; "
                "post-rotation translation REQUIRES rescan")

    def _rotation_clearance_details(self, robot_pos):
        """Measured obstacle clearance for a stationary full-body rotation.

        Uses the same projected LiDAR endpoints as the existing safety guard;
        does not infer that unobserved space is free. The base footprint is
        approximated as a disk centered on the odometry origin.
        """
        points = self._scan_points_world()
        if points is None or len(points) == 0:
            return None, "missing/stale/empty LiDAR observations"
        center = np.asarray(robot_pos, dtype=float)
        distances = np.linalg.norm(points - center, axis=1)
        idx = int(np.argmin(distances))
        obstacle = points[idx]
        nearest = float(distances[idx])
        required = self.robot_clearance_radius
        message = (
            f"nearest={nearest:.3f}m, required={required:.3f}m, "
            f"margin={nearest-required:+.3f}m, "
            f"obstacle_xy=({obstacle[0]:.3f},{obstacle[1]:.3f}), "
            f"robot_xy=({center[0]:.3f},{center[1]:.3f}), "
            f"yaw={math.degrees(self.yaw):+.1f}deg, "
            f"scan={self.scan_sequence}, returns={len(points)}"
        )
        return nearest >= required, message

    def _guard_motion(self, robot_pos, path, *, rotation=False):
        """Conservative, lightweight observed-return veto; unknown returns never certify free."""
        points = self._scan_points_world()
        if points is None or len(points) == 0:
            return False, 'missing/stale/empty LiDAR observations'
        position = np.asarray(robot_pos, dtype=float)
        if rotation:
            clear, details = self._rotation_clearance_details(robot_pos)
            if clear is None:
                return False, details
            if not clear:
                return False, f'obstacle within rotation footprint: {details}'
            return True, f'rotation clearance observed: {details}'
        if path is None or len(path) < 2:
            return False, 'no executable translation leg'
        # Check from the closest point of the active polyline forward a braking-sized
        # horizon. A LiDAR hit on the swept robot disk vetoes translation.
        _, index, projected = nearest_point_on_polyline(path, robot_pos)
        pieces = [np.asarray(robot_pos, dtype=float), np.asarray(projected, dtype=float)]
        pieces.extend(np.asarray(pt, dtype=float) for pt in path[index + 1:])
        travelled = 0.0
        for a, b in zip(pieces, pieces[1:]):
            displacement = b - a
            length = float(np.linalg.norm(displacement))
            if length <= 1e-9:
                continue
            limit = min(length, self.translation_guard_distance - travelled)
            if limit <= 0:
                break
            end = a + displacement * (limit / length)
            ab = end - a
            t = np.clip(((points - a) @ ab) / max(float(ab @ ab), 1e-9), 0, 1)
            distances = np.linalg.norm(points - (a + t[:, None] * ab), axis=1)
            if np.any(distances < self.robot_clearance_radius):
                return False, 'LiDAR obstacle intersects remaining forward trajectory'
            travelled += limit
        return True, 'no observed obstacle along checked trajectory'

    def _begin_backtrack(self, robot_pos):
        """Plan a forward-driving return to the previous node; no reverse motion."""
        self.publish_stop()
        if self.prev_node is None:
            self._safety_hold('Cannot start backtrack: previous node missing')
            return
        self.backtrack_return_edge = None
        self.backtrack_phase = 'PLAN_RETURN'
        self.backtrack_history = []
        self.backtrack_reverse_origin = None
        self.backtrack_reverse_target = None
        self.get_logger().info('BACKTRACK: plan_motion() toward previous node (in-place rotation allowed)')

    def _guard_backtrack(self, robot_pos, target, heading):
        """Fail closed on missing rear coverage or occupied immediate swept segment.

        Assumes base_scan axes are aligned with base_link (+X forward), with the
        configured small laser XY translation. Requires fresh actual rear rays.
        This checks observed returns; it cannot certify unobserved free space.
        """
        if self.scan is None or self._last_scan_wall is None or time.monotonic() - self._last_scan_wall > self.scan_max_age:
            return False, 'missing/stale LiDAR scan'
        scan = self.scan
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        rear_delta = np.arctan2(np.sin(angles - math.pi), np.cos(angles - math.pi))
        mask = np.abs(rear_delta) <= self.backtrack_rear_sector
        selected = np.argsort(rear_delta[mask])
        measured = rear_delta[mask][selected]
        values = ranges[mask][selected]
        if len(measured) < 3 or measured[0] > -self.backtrack_rear_sector + self.backtrack_scan_max_gap or measured[-1] < self.backtrack_rear_sector - self.backtrack_scan_max_gap or np.max(np.diff(measured)) > self.backtrack_scan_max_gap:
            return False, 'rear sector not fully covered by scanner'
        # LaserScan commonly encodes "no obstacle detected within max range" as +inf.
        # That is not a defective reading. NaN, -inf, and finite out-of-range
        # readings remain unconfirmed, so recovery must stop on those.
        finite = np.isfinite(values)
        no_return = np.isposinf(values)
        invalid = ~(finite | no_return)
        invalid |= finite & ((values < scan.range_min) | (values > scan.range_max))
        if np.any(invalid):
            return False, (
                'rear sector contains invalid/unconfirmed beams '
                f'(invalid={np.count_nonzero(invalid)}, '
                f'no_return={np.count_nonzero(no_return)}, total={len(values)})'
            )
        # Only finite measured returns can represent an obstacle in this sector.
        # A +inf beam means nothing was detected within the scanner's range.
        clearance = self.robot_clearance_radius + 0.02
        if np.any(values[finite] < clearance + 0.04):
            return False, 'obstacle too close in rear sector'
        dx, dy = target[0]-robot_pos[0], target[1]-robot_pos[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return False, 'degenerate return target'
        horizon = min(length, self.backtrack_check_distance)
        endpoint = (robot_pos[0] + horizon*dx/length, robot_pos[1]+horizon*dy/length)
        # Evaluate the actual reverse swept segment, not the unrelated forward
        # FOLLOW polyline. A hit inside the inflated footprint is still unsafe.
        points = self._scan_points_world()
        if points is None or len(points) == 0:
            return False, 'missing/stale/empty LiDAR obstacle endpoints'
        a = np.asarray(robot_pos, dtype=float)
        b = np.asarray(endpoint, dtype=float)
        ab = b - a
        t = np.clip(((points - a) @ ab) / max(float(ab @ ab), 1e-9), 0.0, 1.0)
        closest = a + t[:, None] * ab
        distance = np.linalg.norm(points - closest, axis=1)
        i = int(np.argmin(distance))
        min_clearance = float(distance[i])
        required = self.robot_clearance_radius
        if min_clearance < required:
            hit = points[i]
            center_distance = float(np.linalg.norm(hit - a))
            return False, (
                f'reverse swept path occupied: clearance={min_clearance:.3f}m '
                f'< required={required:.3f}m; '
                f'obstacle=({hit[0]:.3f},{hit[1]:.3f}), '
                f'distance_from_robot={center_distance:.3f}m, '
                f'along_segment={t[i]*horizon:.3f}m, '
                f'robot=({a[0]:.3f},{a[1]:.3f}), '
                f'target=({b[0]:.3f},{b[1]:.3f}), '
                f'scan={self.scan_sequence}'
            )
        return True, (f'reverse swept path observed clear: '
                      f'nearest={min_clearance:.3f}m, required={required:.3f}m')

    def _drive_backtrack(self, robot_pos, robot_heading, dt):
        """Plan and execute a forward-driving return, with guarded in-place turns."""
        if self.prev_node is None:
            self._safety_hold('Missing previous node during BACKTRACK')
            return
        if math.dist(robot_pos, self.prev_node) <= self.backtrack_arrival_tol:
            self.publish_stop()
            self._invalidate_planning()
            self.active_edge = None
            self.backtrack_phase = None
            self._set_mode('REPLAN')
            return

        if self.backtrack_phase == 'PLAN_RETURN':
            self.publish_stop()
            # Direct the existing planner from the stopped recovery pose toward
            # the previous node; the maneuver executor handles in-place turns.
            if self.backtrack_return_edge is None:
                if math.dist(robot_pos, self.prev_node) < 0.1:
                    self._set_mode('REPLAN')
                    return
                self.backtrack_return_edge = (tuple(robot_pos), self.prev_node)
                self._invalidate_planning()
                self.active_edge = self.backtrack_return_edge
                self.aligned_edge = self.active_edge
            self._poll_planning_worker()
            self._request_planning(self.backtrack_return_edge)
            result = self._current_planning_result(self.backtrack_return_edge)
            if result is None or result.motion_report.plan is None:
                if self.no_motion_failure_count >= self.no_motion_threshold:
                    self._safety_hold('No feasible forward return trajectory')
                return
            self._accept_new_plan(result, robot_pos)
            if self.maneuver_stage is None and self.tracking_path is None:
                return
            self.backtrack_phase = 'FOLLOW_RETURN'
            self.get_logger().info(f'BACKTRACK: executing return plan ({result.motion_report.plan.mode})')
            return

        if self.backtrack_phase == 'FOLLOW_RETURN':
            edge = self.backtrack_return_edge
            self._poll_planning_worker()
            self._request_planning(edge)
            result = self._current_planning_result(edge)
            self._accept_new_plan(result, robot_pos)
            if self._execute_maneuver(robot_pos, robot_heading, dt):
                return
            if not self._accepted_path_valid(edge):
                self.publish_stop()
                if self.no_motion_failure_count >= self.no_motion_threshold:
                    self._safety_hold('Return plan no longer feasible')
                return
            if self.tracking_path is None:
                self.publish_stop()
                return
            valid, reason = self._guard_motion(robot_pos, self.tracking_path)
            if not valid:
                self._safety_hold(f'Unsafe return translation: {reason}')
                return
            target = lookahead_point(self.tracking_path, robot_pos, self.lookahead_dist)
            self._controller_segment = edge
            self._controller_waypoint_distance = math.dist(robot_pos, self.prev_node)
            v, w = self.turtlebot_control(robot_pos, robot_heading, target, dt,
                                          self.lookahead_dist)
            self.publish_cmd(v, w)
            self.draw_scene(robot=robot_pos, target=target)
            return
        self._safety_hold('Unknown internal BACKTRACK phase')

    def _execute_maneuver(self, robot_pos, robot_heading, dt):
        """Returns True when an accepted staged maneuver owns the control cycle."""
        if self.maneuver_stage is None:
            return False
        if (self._last_scan_wall is None or
                time.monotonic() - self._last_scan_wall > self.scan_max_age):
            self.debug_stop('Staged maneuver: stale LiDAR')
            return True
        if self.maneuver_stage == 'APPROACH':
            if (self._accepted_at is None or time.monotonic() - self._accepted_at > self.accepted_plan_max_age):
                self.publish_stop()
                self._safety_hold('Approach plan expired; no motion')
                return True
            if self.rotation_position is None:
                self.debug_stop('Missing rotation station')
                return True
            distance = math.dist(robot_pos, self.rotation_position)
            if distance <= self.node_tol:
                self.publish_stop()
                self.maneuver_stage = 'ROTATE'
                self.get_logger().info('Reached rotation station; rotating in place')
                return True
            valid, reason = self._guard_motion(robot_pos, self.tracking_path)
            if not valid:
                self.publish_stop()
                self._safety_hold(f'Unsafe approach to rotation station: {reason}')
                return True
            if self.tracking_path is None:
                self.debug_stop('No approach translation leg')
                return True
            target = lookahead_point(self.tracking_path, robot_pos, self.lookahead_dist)
            self._controller_segment = self.active_edge
            self._controller_waypoint_distance = distance
            v, w = self.turtlebot_control(robot_pos, robot_heading, target, dt,
                                          self.lookahead_dist)
            self.publish_cmd(v, w)
            self.draw_scene(robot=robot_pos, target=target)
            return True
        if self.maneuver_stage == 'ROTATE':
            if self.rotation_position is None or self.rotation_target_yaw is None:
                self.debug_stop('Missing rotation command')
                return True
            station_drift = math.dist(robot_pos, self.rotation_position)
            if station_drift > self.node_tol:
                self.debug_stop(
                    f'Drifted {station_drift:.3f}m from rotation station '
                    f'{self.rotation_position}; cancelling stale maneuver')
                self._invalidate_planning()
                return True
            error = angle_wrap(self.rotation_target_yaw - robot_heading)
            # Do not require rotational clearance when no further rotation is
            # commanded. This also handles a target already reached on entry.
            tolerance = (self.narrow_entry_yaw_tolerance
                         if (self.motion_report is not None
                             and self.motion_report.plan is not None
                             and self.motion_report.plan.mode == "center then align narrow entry")
                         else self.rotation_tolerance)
            if abs(error) <= tolerance:
                self.publish_stop()
                self.pid_maneuver_rotation.reset()
                self.get_logger().info('Rotation complete; invalidating hypothetical continuation and requesting fresh scan')
                completed_scan = self.scan_sequence
                self.unsafe_rotation_replans = 0
                self._invalidate_planning()
                self.min_planning_scan_sequence = completed_scan
                # A newly planned trajectory will be required before ANY forward motion.
                self.aligned_edge = self.active_edge
            else:
                # Keep the existing collision guard for actual in-place motion.
                valid, reason = self._guard_motion(robot_pos, None, rotation=True)
                if not valid:
                    self.publish_stop()
                    # This is a possible alternative-planning opportunity, not
                    # permission to rotate through an obstacle. If a small turn
                    # is unsafe, discard the staged maneuver and request a plan
                    # from a newer scan at the current pose. Strictly bounded:
                    # repeated rotate-now proposals must eventually stop.
                    if self.unsafe_rotation_retry_edge != self.active_edge:
                        self.unsafe_rotation_retry_edge = self.active_edge
                        self.unsafe_rotation_replans = 0
                    if (abs(error) <= self.small_rotation_retry_limit
                            and self.unsafe_rotation_replans < self.unsafe_rotation_max_replans):
                        self.unsafe_rotation_replans += 1
                        attempt = self.unsafe_rotation_replans
                        failed_scan = self.scan_sequence
                        self.get_logger().warn(
                            f'Unsafe small rotation ({math.degrees(error):+.1f}deg): '
                            f'{reason}. Stationary fresh-plan retry '
                            f'{attempt}/{self.unsafe_rotation_max_replans}; '
                            'no rotation or translation commanded.')
                        self._invalidate_planning()
                        self.min_planning_scan_sequence = max(
                            self.min_planning_scan_sequence, failed_scan)
                        self.draw_scene(robot=robot_pos)
                        return True
                    self._safety_hold(
                        f'Unsafe rotation (alternative replans exhausted or turn is large): '
                        f'{reason}; target_yaw={math.degrees(self.rotation_target_yaw):+.1f}deg, '
                        f'error={math.degrees(error):+.1f}deg, '
                        f'rotation_station={self.rotation_position}'
                    )
                    return True
                turn_w = self.pid_maneuver_rotation.update(error, dt)
                self.publish_cmd(0.0, float(np.clip(turn_w, -self.w_max,
                                                     self.w_max)))
            self.draw_scene(robot=robot_pos)
            return True
        self.debug_stop(f'Invalid maneuver stage {self.maneuver_stage}')
        return True

    def _log_passability_diagnostics(self, edge, observation, assessment, robot_xy):
        """Rate-limited observation report; does not influence edge rejection."""
        now = time.monotonic()
        if now - getattr(self, "_last_passability_debug", -1e9) < 2.0:
            return
        self._last_passability_debug = now
        start = np.asarray(observation.start, dtype=float)
        end = np.asarray(observation.end, dtype=float)
        direction = (end - start) / observation.length
        robot_s = float(np.dot(np.asarray(robot_xy) - start, direction))
        cutoff = robot_s - 0.089
        radius = 0.119
        forward = [sec for sec in observation.slices if sec.s >= cutoff]
        details = []
        for sec in forward:
            near = [(float(lo), float(hi)) for lo, hi in sec.obstacle_intervals
                    if lo <= radius and hi >= -radius]
            near_gaps = [(round(g.left, 3), round(g.right, 3), round(g.width, 3))
                         for g in sec.gaps
                         if g.left <= radius and g.right >= -radius]
            if near or near_gaps:
                details.append((sec, near, near_gaps))
        details.sort(key=lambda item: item[0].s)
        example = []
        for sec, near, gaps in details[:4]:
            clusters = [(round(a, 3), round(b, 3)) for a,b in sec.obstacle_intervals]
            example.append(f"s={sec.s:.2f}: clusters={clusters}, near_clusters={near}, near_gaps={gaps}")
        points = observation.obstacle_points_st
        upcoming = points[points[:, 0] >= cutoff] if len(points) else points
        if len(upcoming):
            dist = np.maximum(np.abs(upcoming[:, 1]) - radius, 0.0)
            closest = upcoming[int(np.argmin(dist))]
            nearest = f"nearest forward return st=({closest[0]:.2f},{closest[1]:.2f})"
        else:
            nearest = "no forward returns in observation band"
        self.get_logger().warn(
            f"[PASSABILITY DEBUG] edge={self.node_name(edge[0])} -> "
            f"{self.node_name(edge[1])}, scan={self.scan_sequence}, "
            f"status={assessment.status.name}, robot_s={robot_s:.2f}, "
            f"cutoff={cutoff:.2f}, forward_slices={len(forward)}, "
            f"near_corridor_slices={len(details)}, "
            f"{nearest}; {assessment.reason}; sample: "
            + (" | ".join(example) if example else "none")
        )

    def _fast_blockage_check(self, edge, result) -> bool:
        """Reject persistent all-narrow-gap evidence or a trusted-wall barrier.

        Both criteria are evaluated on the current LiDAR scan; safety vetoes
        and guarded recovery remain separate.
        """
        if self.scan_sequence == self.fast_block_last_scan:
            return False
        if (self._last_scan_wall is None or
                time.monotonic() - self._last_scan_wall > self.scan_max_age):
            return False
        points = self._scan_points_world()
        if points is None:
            return False
        self.fast_block_last_scan = self.scan_sequence
        robot_xy = (self.x, self.y)
        observation = analyze_edge(points, *edge)
        trusted = self.trusted_corridor_bounds.get(edge)
        assessment = assess_passability(
            observation, robot_xy, robot_radius=0.089, safety_margin=0.03,
            corridor_bounds=trusted)
        self._log_passability_diagnostics(edge, observation, assessment, robot_xy)
        # Confirm heuristic detections across distinct scans. A known physical
        # barrier may be rejected immediately; brief clusters should not erase
        # graph connectivity permanently.
        if assessment.status != Passability.BLOCKED:
            self._gap_block_edge = None
            self._gap_block_count = 0
            return False
        trusted_barrier = assessment.reason.startswith("Confirmed inflated-point barrier")
        if not trusted_barrier:
            # At a graph intersection, clutter immediately beyond the start node
            # is not sufficient evidence to delete the entire outgoing edge.
            # This only filters the heuristic; live collision vetoes stay active.
            start = np.asarray(edge[0], dtype=float)
            end = np.asarray(edge[1], dtype=float)
            axis = (end - start) / np.linalg.norm(end - start)
            robot_s = float(np.dot(np.asarray(robot_xy) - start, axis))
            match = re.search(r"at s=([0-9.]+) m", assessment.reason)
            narrow_s = float(match.group(1)) if match else None
            if (narrow_s is not None and narrow_s < 0.25
                    and robot_s < 0.20):
                self._gap_block_edge = None
                self._gap_block_count = 0
                return False
            if getattr(self, "_gap_block_edge", None) == edge:
                self._gap_block_count = getattr(self, "_gap_block_count", 0) + 1
            else:
                self._gap_block_edge = edge
                self._gap_block_count = 1
            if self._gap_block_count < 3:
                return False
        self._gap_block_edge = None
        self._gap_block_count = 0
        self.get_logger().warn(
            f"Passability BLOCKED on {self.node_name(edge[0])} -> "
            f"{self.node_name(edge[1])}: {assessment.reason}; "
            f"trusted_bounds={trusted}; scan={self.scan_sequence}")
        return True

    # --- Template interface: passability ---
    def is_passable(self, seg_dir_yaw: float) -> bool:
        """Return whether the current edge has not been confirmed BLOCKED.

        Keep the template interface while delegating to the existing fresh-scan
        blockage classifier (including its multi-scan confirmation). UNKNOWN
        is not classified as BLOCKED; motion still requires the independent
        planning and collision guards in control_step().
        """
        if self.active_edge is None:
            return False
        return not self._fast_blockage_check(self.active_edge, None)

    # --- Template interface: obstacle-avoiding lookahead target ---
    def obstacle_avoided_target(self, robot_pos: Coord,
                                seg_dir_yaw: float) -> Coord:
        """Choose lookahead along the accepted perception-planned trajectory.

        The current plan already incorporates obstacle avoidance via
        plan_motion(); never fabricate a target when no accepted path exists.
        Callers must validate freshness and collision clearance separately.
        """
        if self.tracking_path is None or len(self.tracking_path) < 2:
            raise ValueError("No accepted local trajectory for lookahead")
        return lookahead_point(self.tracking_path, robot_pos,
                               self.lookahead_dist)

    def turtlebot_control(self, robot_pos: Coord, robot_h: float, target: Coord,
                          dt: float, Ld: float) -> Tuple[float, float]:
        """Continuous geometric pure pursuit; no cross-track PID or normal-curve stops.

        Only a gross backward-facing lookahead triggers an in-place recovery turn.
        The caller controls whether a plan is safe enough to execute.
        """
        dx, dy = target[0] - robot_pos[0], target[1] - robot_pos[1]
        c, sn = math.cos(robot_h), math.sin(robot_h)
        forward = c * dx + sn * dy
        lateral = -sn * dx + c * dy
        distance_sq = forward * forward + lateral * lateral
        if distance_sq < 1e-8:
            return 0.0, 0.0
        heading_error = math.atan2(lateral, forward)
        if abs(heading_error) > self.rotate_heading_tol:
            return 0.0, float(np.clip(1.5 * heading_error, -self.w_max, self.w_max))

        curvature = 2.0 * lateral / distance_sq
        v = min(self.v_average, self.v_max)
        # Slow on sharp turns while respecting the angular-velocity limit.
        if abs(curvature) > 1e-6:
            v = min(v, self.w_max / abs(curvature))
        v *= max(0.65, math.cos(heading_error))

        # Brake for the *end* of the currently executed edge only. Other
        # intermediate points within a sampled trajectory are not stop points.
        remaining = self._controller_waypoint_distance
        if remaining is not None:
            v = min(v, math.sqrt(2.0 * self.braking_deceleration *
                                 max(0.0, remaining - self.node_tol)))

        # Measured clearance reduces speed in tight spaces; it does not
        # establish that unknown space is collision-free.
        clearance = self.local_clearance
        if clearance is not None and np.isfinite(clearance):
            if clearance < self.tight_clearance:
                v = min(v, self.tight_speed)
            elif clearance < self.free_clearance:
                fraction = (clearance - self.tight_clearance) / (
                    self.free_clearance - self.tight_clearance)
                v = min(v, self.tight_speed + fraction *
                        (self.cautious_speed - self.tight_speed))
        return float(v), float(np.clip(curvature * v, -self.w_max, self.w_max))

    def _accepted_path_valid(self, edge) -> bool:
        """An accepted command path can outlive a pending asynchronous update.

        Never drive with a stale scan or an unboundedly old path. This check
        cannot certify collision freedom; the fast blockage test remains active.
        """
        now = time.monotonic()
        return (self._accepted_edge == edge and self.tracking_path is not None
                and len(self.tracking_path) >= 2
                and self._accepted_at is not None
                and now - self._accepted_at <= self.accepted_plan_max_age
                and self._last_scan_wall is not None
                and now - self._last_scan_wall <= self.scan_max_age)

    def _accept_new_plan(self, result, robot_pos):
        """Accept each fresh worker output at most once, with path hysteresis.

        A replacement similar to the active route is ignored to prevent
        per-scan lookahead jumping. A rejected or incomplete plan never erases
        an unexpired accepted trajectory.
        """
        if self.maneuver_stage is not None:
            return  # A latched rotate/approach maneuver is atomic.
        if result is None or result.scan_sequence <= self._accepted_scan_sequence:
            return
        self._accepted_scan_sequence = result.scan_sequence
        plan = result.motion_report.plan
        if plan is None:
            return
        if self._accepted_path_valid(result.edge) and not plan.requires_rotation:
            if plan.segments_st and len(plan.segments_st[0]) >= 2:
                candidate = [edge_to_world(st[0], st[1],
                             result.observation.start, result.observation.end)
                             for st in plan.segments_st[0]]
                # Compare the lookahead targets, not the first sampled point;
                # the latter always moves forward with robot odometry.
                old_target = lookahead_point(self.tracking_path, robot_pos,
                                             self.lookahead_dist)
                new_target = lookahead_point(candidate, robot_pos,
                                             self.lookahead_dist)
                # Do not apply larger hysteresis after replanning: a 14 cm
                # lookahead difference can hide meaningful steering corrections.
                # Existing path age and live safety guards remain in effect.
                hysteresis = 0.08
                if math.dist(old_target, new_target) < hysteresis:
                    # Keep the old path AND its original validity timestamps.
                    # Otherwise tiny replans can refresh stale geometry forever.
                    return
        self._apply_planning_result(result, robot_pos)
        if self.tracking_path is not None or self.maneuver_stage is not None:
            self._accepted_at = time.monotonic()
            self._accepted_scan_at = result.scan_received_at
            self._accepted_edge = result.edge

    # --- OUR HELPERS (Workstream C) ---
    def node_name(self, p: Coord) -> str:
        """'(r,c)' if p is a graph node, else the rounded (x, y). Only for log messages."""
        key = (float(p[0]), float(p[1]))
        if key in self.graph.coord2idx:
            r, c = self.graph.coord2idx[key]
            return f"({r},{c})"
        return f"({p[0]:.3f},{p[1]:.3f})"

    def on_new_edge(self, ga: Coord, gb: Coord):
        """Called once when FOLLOW starts driving a new edge ga -> gb."""
        self.get_logger().info(f"Now driving edge {self.node_name(ga)} -> {self.node_name(gb)}")
        # Begin recording the actual motion for this edge, including curved maneuvers.
        self.edge_pose_history = [(self.x, self.y, self.yaw)]
        self.backtrack_history = []
        self.backtrack_cursor = -1
        # Forget what perception remembered about the previous edge.
        # Agree the variable names with the perception owner (Ki Chun).
        self.perception_observation = None
        self.motion_report = None
        self.motion_target = None
        self.tracking_path = None
        self.local_clearance = None
        self.pid_angular.reset()
        self.pid_centering.reset()
        self.pid_edge_rotation.reset()
        self.pid_maneuver_rotation.reset()
        self._alignment_settle_started = None
        self.rotation_target_yaw = None
        self.rotation_position = None
        self.no_motion_recovery_attempts = 0
        self.no_motion_recovery_edge = (ga, gb)
        self.unsafe_rotation_replans = 0
        self.unsafe_rotation_retry_edge = (ga, gb)
        self._invalidate_planning()

    # --- Publisher helpers ---
    def publish_cmd(self, v: float, w: float):
        """
        Publish a geometry_msgs/Twist command.

        Args:
            v: linear velocity (m/s), applied to msg.linear.x.
            w: angular velocity (rad/s), applied to msg.angular.z.
        """
        msg = Twist(); msg.linear.x = float(v); msg.angular.z = float(w)
        self._contact_last_cmd_v = float(v)
        self._contact_last_cmd_w = float(w)
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        """
        Convenience: publish zero velocities to stop the turtlebot.
        """
        self.publish_cmd(0.0, 0.0)

    # --- Vizualization helper ---
    def draw_scene(self, robot: Optional[Coord] = None, target: Optional[Coord] = None):
        """
        Render the grid, nodes, path, turtlebot and target for debugging.

        Args:
            robot: optional (x, y) turtlebot position to plot.
            target: optional (x, y) target point to plot.
        """
       
        extra = [self.start_pos, self.goal_pos]
        if robot: extra.append(robot)
        if target: extra.append(target)
        img, to_px = self.graph.base_image(extra_points=extra, img_size=800, margin=60)
        # edges (grey)
        for a, b in self.graph.edges():
            cv2.line(img, to_px(*a), to_px(*b), (220, 220, 220), 2)
        # nodes (purple)
        for xy in self.graph.coord2idx.keys():
            cv2.circle(img, to_px(*xy), 6, (240, 32, 160), -1)
        # path (black)
        if self.full_path and len(self.full_path) >= 2:
            pts = np.array([to_px(*p) for p in self.full_path], dtype=np.int32)
            cv2.polylines(img, [pts], isClosed=False, color=(30, 30, 30), thickness=3)

        # start (red) & goal (green)
        cv2.circle(img, to_px(*self.start_pos), 9, (0, 0, 220), -1)
        cv2.circle(img, to_px(*self.goal_pos), 9, (0, 220, 0), -1)
        # turtlebot
        if robot is not None:
            pr = to_px(*robot)
            cv2.circle(img, pr, 7, (0, 165, 255), -1)
            hx = robot[0] + math.cos(self.yaw) * 0.3
            hy = robot[1] + math.sin(self.yaw) * 0.3
            cv2.arrowedLine(img, pr, to_px(hx, hy), (0, 165, 255), 2, tipLength=0.3)
        # lookahead target (blue)
        if target is not None:
            cv2.circle(img, to_px(*target), 7, (200, 0, 0), -1) 
        # HUD
        cv2.putText(img, f"Mode: {self.mode}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 20, 20), 2)
        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow(self.win, img); cv2.waitKey(1)
    
    # --- Main control loop ----
    def control_step(self):
        """
        Main control loop
        """

        # --- Stamping current time ---
        now = self._now()
        dt = max(1e-3, now - self.last_t)
        self.last_t = now

        # Internal safety stop is independent of the template's four modes.
        if self.safety_hold_reason is not None:
            self.publish_stop()
            self.draw_scene(robot=(self.x, self.y) if self.pose_ready else None)
            return

        if self.mode == 'DONE':
            self.publish_stop()
            self.draw_scene(robot=(self.x, self.y) if self.pose_ready else None)
            return

        # --- No odometry values, no calculated A* path, or path is only one coordinate ---
        if not self.pose_ready or self.full_path is None or len(self.full_path) < 2:
            self.debug_stop("Waiting for odometry or a usable path")
            self.draw_scene()
            return

        # --- Current turtlebot position ---
        robot_position = (self.x, self.y)
        robot_heading  = self.yaw

        # --- Check if goal reached ---
        if math.dist(robot_position, self.goal_pos) < self.goal_tol:
            self.debug_stop("Goal reached")
            self._set_mode('DONE')
            self.get_logger().info(f'GOAL REACHED!!!')
            self.draw_scene(robot=robot_position)
            return

        # --- (Optional) Pause motion ---
        if self.dwell_until is not None and now < self.dwell_until:
            self.publish_stop()  # Expected dwell; avoid a stop-warning every cycle.
            self.draw_scene(robot=robot_position)
            return
        else:
            self.dwell_until = None

        # Follow the first remaining waypoint pair until its destination is reached.
        # Geometric nearest-segment selection can jump to an adjacent graph edge
        # during turns and incorrectly attribute LiDAR evidence to that edge.
        seg_idx = 0
        ga = self.full_path[seg_idx]     # segment’s start point
        gb = self.full_path[seg_idx + 1] # segment’s end point
        seg_vec = (gb[0] - ga[0], gb[1] - ga[1]) # vector pointing from a → b (i.e. local forward direction)
        seg_dir_yaw = math.atan2(seg_vec[1], seg_vec[0]) # heading (yaw angle) of this segment in the world frame
        dist_to_b = math.dist(robot_position, gb) # robot’s straight-line distance to the next waypoint (b)

        # --- Check arrival at the node 'b' and (Optional) pause motion ---
        if self.mode == 'FOLLOW' and dist_to_b <= self.node_tol:
            if seg_idx + 1 < len(self.full_path):
                self.full_path = self.full_path[seg_idx + 1:]
            self.publish_stop()  # Deliberate boundary stop before assessing next edge.
            self._invalidate_planning()
            self.active_edge = None
            self.aligned_edge = None
            self.edge_pose_history = []
            self.draw_scene(robot=robot_position, target=gb)
            return

        # --- MODE: REPLAN ---
        if self.mode == 'REPLAN':
            self.debug_stop("Replanning after blockage")
            # TODO: YOUR CODE HERE: 
            # ~2 lines: Remove the edge if the edge is blocked 
            # ~1 line:  Set start_from to the previous node
            # ~1 line:  Replan the global path from new start point and store it in self.full_path.
            # ~1 line:  Set the mode back to FOLLOW
            if self.block_edge is not None:
                a, b = self.block_edge
                if a in self.graph.coord2idx and b in self.graph.coord2idx and self.graph.has_edge(a, b):
                    self.graph.remove_edge(a, b)
                    self.get_logger().info(f"Removed blocked edge {self.node_name(a)} <-> {self.node_name(b)}")
                else:
                    self.get_logger().warn(f"Cannot remove {self.block_edge}: not a graph edge")
            start_from = self.prev_node if self.prev_node is not None else (self.x, self.y)
            self.path_plan(start_from)
            self.block_edge = None
            self.tracking_path = None
            self.local_clearance = None
            self.pid_angular.reset()
            self._invalidate_planning()
            self.active_edge = None
            self.aligned_edge = None
            self.edge_pose_history = []
            self.backtrack_history = []
            self._stabilize_after_replan = True
            self._post_replan_hysteresis_until = 0.0
            self._alignment_settle_started = None
            if len(self.full_path) < 2:
                self.debug_stop("No alternative route")
                self.get_logger().error("No alternative route; stopping")
                self._set_mode('DONE')
            else:
                self._set_mode('FOLLOW')
            self.draw_scene(robot=robot_position)
            return

        if self.mode == 'DONE':
            self.debug_stop(f"Terminal state: {self.mode}")
            self.draw_scene(robot=robot_position)
            return

        # --- MODE: BACKTRACK ---
        # Template mode: the perception planner proposes the return trajectory;
        # guarded in-place rotation remains available (no fifth mode).
        if self.mode == 'BACKTRACK':
            self._drive_backtrack(robot_position, robot_heading, dt)
            return

        # --- MODE: FOLLOW ---
        self._poll_planning_worker()
        if (ga, gb) != self.active_edge:
            self.active_edge = (ga, gb)
            self.on_new_edge(ga, gb)
            self.aligned_edge = None

        # At the start of each new edge, rotate in place before deciding
        # passability or beginning forward motion. Once aligned, allow the
        # local obstacle-avoidance trajectory to change heading naturally.
        if self.aligned_edge != self.active_edge:
            heading_error = angle_wrap(seg_dir_yaw - robot_heading)

            if abs(heading_error) > self.edge_heading_tolerance:
                self._alignment_settle_started = None
                log_now = time.monotonic()
                if log_now - self._alignment_log_time >= 1.0:
                    self.get_logger().info(
                        f"Aligning edge: yaw={math.degrees(robot_heading):.1f} deg, "
                        f"target={math.degrees(seg_dir_yaw):.1f} deg, "
                        f"error={math.degrees(heading_error):.1f} deg"
                    )
                    self._alignment_log_time = log_now
                # Preserve fast coarse turns, but slow the last few degrees
                # to prevent repeatedly overshooting a narrow entrance.
                fine_turn_limit = (0.35 if abs(heading_error) < math.radians(12.0)
                                   else self.edge_alignment_w_max)
                angular_speed = float(np.clip(
                    self.pid_edge_rotation.update(heading_error, dt),
                    -fine_turn_limit,
                    fine_turn_limit,
                ))

                # Avoid a minimum angular velocity near the goal heading:
                # forcing it close to tolerance can create a limit cycle.
                if (abs(heading_error) > math.radians(15.0) and
                        abs(angular_speed) < self.edge_alignment_min_angular_speed):
                    angular_speed = math.copysign(
                        min(self.edge_alignment_min_angular_speed, self.edge_alignment_w_max),
                        heading_error,
                    )

                self.publish_cmd(0.0, angular_speed)
                self.draw_scene(robot=robot_position, target=gb)
                return

            # Heading tolerance alone is insufficient: do not start translation
            # while the chassis is still rotating from the in-place turn.
            self.publish_stop()
            measured_w = self.backtrack_measured_w
            if (measured_w is None or not math.isfinite(measured_w)
                    or abs(measured_w) > self._alignment_angular_velocity_tol):
                self._alignment_settle_started = None
                self.draw_scene(robot=robot_position, target=gb)
                return
            if self._alignment_settle_started is None:
                self._alignment_settle_started = time.monotonic()
            if (time.monotonic() - self._alignment_settle_started
                    < self._alignment_settle_duration):
                self.draw_scene(robot=robot_position, target=gb)
                return
            self.aligned_edge = self.active_edge
            self.pid_edge_rotation.reset()
            self._alignment_settle_started = None
            if self._stabilize_after_replan:
                self._post_replan_hysteresis_until = time.monotonic() + 1.0
                self._stabilize_after_replan = False
            self.get_logger().info(
                f"Edge alignment complete (yaw={math.degrees(robot_heading):.1f} deg, "
                f"target={math.degrees(seg_dir_yaw):.1f} deg, "
                f"error={math.degrees(heading_error):.1f} deg); "
                "waiting for validated local motion plan")
            self.draw_scene(robot=robot_position, target=gb)
            return

        # Planning runs in a single worker thread. These operations NEVER wait.
        self._request_planning((ga, gb))
        result = self._current_planning_result((ga, gb))

        # Narrow-entry centering/reposition is selected by plan_motion().
        # Do not override a validated translation with heading-only rotation.

        # If distinct fresh planning attempts repeatedly yield no executable
        # translation/rotation, enter guarded recovery instead of claiming
        # the edge is geometrically BLOCKED. Do not recover while a valid
        # accepted path or an active staged maneuver exists.
        elapsed_failure = (time.monotonic() - self.no_motion_since
                           if self.no_motion_since is not None else 0.0)
        if (self.no_motion_failure_count >= self.no_motion_threshold
                and elapsed_failure >= self.no_motion_min_duration
                and self.maneuver_stage is None
                and not self._accepted_path_valid((ga, gb))):
            self.publish_stop()
            if self.no_motion_recovery_edge != (ga, gb):
                self.no_motion_recovery_edge = (ga, gb)
                self.no_motion_recovery_attempts = 0
            if self.no_motion_recovery_attempts < self.no_motion_max_recoveries:
                self.no_motion_recovery_attempts += 1
                attempt = self.no_motion_recovery_attempts
                # Discard the failed generation. Wait for a newer LiDAR scan;
                # do not issue any translation or turn without a valid plan.
                failed_scan = self.scan_sequence
                self.get_logger().warn(
                    f'NO_FEASIBLE_MOTION on {self.node_name(ga)} -> '
                    f'{self.node_name(gb)} after {self.no_motion_failure_count} '
                    f'failed plans ({elapsed_failure:.1f}s); stationary recovery '
                    f'{attempt}/{self.no_motion_max_recoveries}, awaiting fresh scan. '
                    'Edge is NOT classified BLOCKED.')
                self._invalidate_planning()
                self.min_planning_scan_sequence = max(
                    self.min_planning_scan_sequence, failed_scan)
                self.draw_scene(robot=robot_position)
                return
            self.get_logger().error(
                f'No validated motion on {self.node_name(ga)} -> '
                f'{self.node_name(gb)} after {self.no_motion_max_recoveries} '
                'stationary retries; edge NOT classified BLOCKED. '
                'Manual inspection required.')
            self._safety_hold('Local planner exhausted stationary retries; no validated motion')
            return

        # Reject only a certified cross-corridor barrier on a fresh scan.
        # UNKNOWN and slow planning are not themselves grounds for rejection.
        if not self.is_passable(seg_dir_yaw):
            self.publish_stop()
            self.prev_node = ga
            self.block_edge = (ga, gb)
            self._invalidate_planning()
            self._contact_pose_log('EDGE BLOCKED, before recovery')
            self.get_logger().info(
                f"Edge {self.node_name(ga)} -> {self.node_name(gb)} is BLOCKED")
            distance_to_start = math.dist(robot_position, ga)
            if distance_to_start <= self.backtrack_arrival_tol:
                self.get_logger().info(
                    f'Blocked edge rejected at start node: distance={distance_to_start:.3f}m; '
                    'skip BACKTRACK and replan')
                self._set_mode('REPLAN')
                self.dwell_until = now + self.backtrack_settle_s
            else:
                self._set_mode('BACKTRACK')
                self._begin_backtrack(robot_position)
            self.draw_scene(robot=robot_position)
            return

        # Accept newly computed plans without resetting controller state on
        # every 20-Hz tick, and keep tracking the previous accepted path while
        # the next asynchronous planning job runs.
        self._accept_new_plan(result, robot_position)
        if self._execute_maneuver(robot_position, robot_heading, dt):
            return
        if not self._accepted_path_valid((ga, gb)) and self.rotation_target_yaw is None:
            self._controller_segment = None
            diagnostic_now = time.monotonic()
            if diagnostic_now - self._plan_diagnostic_at >= 2.0:
                self._plan_diagnostic_at = diagnostic_now
                pending = self._planner_future is not None and not self._planner_future.done()
                current = self._planning_latest
                reason = "awaiting first planning result" if current is None else (
                    "planner returned no executable path" if current.motion_report.plan is None else
                    "result rejected by pose/yaw/scan validity gates or path not accepted")
                scan_age = (diagnostic_now - self._last_scan_wall
                            if self._last_scan_wall is not None else float('inf'))
                self.get_logger().warn(
                    f"Stationary at edge {self.node_name(ga)} -> {self.node_name(gb)}: "
                    f"{reason}; worker_busy={pending}; scan_age={scan_age:.2f}s; "
                    f"heading_error={math.degrees(angle_wrap(seg_dir_yaw-robot_heading)):.1f}deg")
            self.debug_stop("No recent accepted path / LiDAR data")
            self.draw_scene(robot=robot_position)
            return
        if self.tracking_path is not None:
            self.motion_target = self.obstacle_avoided_target(
                robot_position, seg_dir_yaw)
        if self.motion_target is None:
            self._controller_segment = None
            self.debug_stop("No safe translation in the fresh local plan")
            self.draw_scene(robot=robot_position)
            return
        valid, reason = self._guard_motion(robot_position, self.tracking_path)
        if not valid:
            self.debug_stop(f'Translation veto: {reason}')
            self.draw_scene(robot=robot_position)
            return
        target = self.motion_target
        self._controller_waypoint_distance = dist_to_b
        self._controller_segment = (ga, gb)
        v_cmd, w_cmd = self.turtlebot_control(robot_position, robot_heading, target, dt, self.lookahead_dist) # Get linear and angular velocity values from your control block
        # Refine a *validated* local trajectory only when the current fresh
        # observation shows a connected, narrow passage around the robot.
        # If the opening is ahead, ambiguous, or conflicts with the planned
        # target, pure pursuit keeps sole control.
        centering_active = False
        if (result is not None and result.edge == (ga, gb)
                and v_cmd > 0.03 and self.maneuver_stage is None
                and abs(angle_wrap(robot_heading - seg_dir_yaw)) < math.radians(18.0)):
            passage = select_next_local_passage(
                result.observation, robot_position,
                robot_radius=0.089, safety_margin=0.03)
            if passage is not None and len(passage.slice_positions) >= 3:
                robot_st = points_to_edge_frame(
                    np.asarray([robot_position], dtype=float), ga, gb)[0]
                target_st = points_to_edge_frame(
                    np.asarray([target], dtype=float), ga, gb)[0]
                robot_s, robot_t = map(float, robot_st)
                observed_width = (passage.usable_right - passage.usable_left
                                  + 2.0 * (0.089 + 0.03))
                if (passage.s_start <= robot_s + 0.05
                        and passage.s_end >= robot_s + 0.10
                        and observed_width < self.center_max_passage_width
                        and passage.usable_left <= robot_t <= passage.usable_right
                        and abs(float(target_st[1]) - passage.target_t) <= 0.10):
                    error = passage.target_t - robot_t
                    correction = float(np.clip(
                        self.pid_centering.update(error, dt),
                        -self.center_max_w, self.center_max_w))
                    # Fade centering out as the observed opening gets wider.
                    # At >= 0.45 m pure pursuit is the sole steering controller.
                    blend = float(np.clip(
                        (self.center_max_passage_width - observed_width)
                        / (self.center_max_passage_width - self.center_full_strength_width),
                        0.0, 1.0))
                    correction *= blend
                    # Existing translation guard above remains authoritative.
                    w_cmd = float(np.clip(w_cmd + correction, -self.w_max, self.w_max))
                    if blend > 0.0:
                        v_cmd = min(v_cmd, self.center_max_speed)
                    centering_active = True
                    if time.monotonic() - self._center_log_at >= 0.5:
                        self._center_log_at = time.monotonic()
                        self.get_logger().info(
                            f'[CENTERING] error={error:+.3f}m '
                            f'width={observed_width:.3f}m '
                            f'correction={correction:+.3f}rad/s '
                            f'slices={len(passage.slice_positions)}')
        if not centering_active:
            self.pid_centering.reset()
        self.publish_cmd(v_cmd, w_cmd) # publish the velocities
        # Distinguish drift from the graph edge from drift off the accepted
        # avoidance trajectory. Diagnostic only: no steering or safety changes.
        diagnostic_now = time.monotonic()
        if diagnostic_now - self._follow_tracking_log_at >= 0.5:
            self._follow_tracking_log_at = diagnostic_now
            edge_dx, edge_dy = gb[0] - ga[0], gb[1] - ga[1]
            edge_length = math.hypot(edge_dx, edge_dy)
            edge_cross_track = ((edge_dx * (robot_position[1] - ga[1])
                                - edge_dy * (robot_position[0] - ga[0]))
                                / edge_length) if edge_length > 1e-9 else 0.0
            _, _, projected = nearest_point_on_polyline(
                self.tracking_path, robot_position)
            path_error = math.dist(robot_position, projected)
            age = (diagnostic_now - self._accepted_at
                   if self._accepted_at is not None else float('inf'))
            heading_error = math.degrees(angle_wrap(seg_dir_yaw - robot_heading))
            self.get_logger().info(
                f'[FOLLOW TRACKING] edge_xte={edge_cross_track:+.3f}m '
                f'path_error={path_error:.3f}m '
                f'edge_heading_error={heading_error:+.1f}deg '
                f'cmd_v={v_cmd:.3f}m/s cmd_w={w_cmd:+.3f}rad/s '
                f'odom_w={self.backtrack_measured_w} '
                f'path_age={age:.2f}s')
        self.draw_scene(robot=robot_position, target=target)


# --------------- entry ---------------
def main():
    rclpy.init()
    node = GridPathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node._planner_pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
