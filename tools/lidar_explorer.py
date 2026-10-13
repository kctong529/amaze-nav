
#!/usr/bin/env python3
"""OpenCV/ROS debugging explorer for Perception Module v1.1."""

from collections import Counter

import cv2
import re
import numpy as np
import rclpy

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from perception import analyze_edge, detect_bottlenecks, assess_passability, OpeningProgressTracker, TrajectoryResult, plan_motion
from trajectory_display import select_representatives, group_rejections

from perception_geometry import (
    transform_points_2d,
    points_to_edge_frame,
    extract_slice,
    cluster_lateral_points,
    find_gaps,
)

GRID_X = [0.712, 2.136, 3.560, 4.984, 6.408]
GRID_Y = [0.000, -1.424, -2.848, -4.272, -5.696]

SENSOR_WINDOW = "LiDAR Explorer - Sensor Frame"
ODOM_WINDOW = "LiDAR Explorer - Odom Frame"
EDGE_WINDOW = "LiDAR Explorer - Edge Frame"

TEST_EDGE_START = (0.712, -2.848)
TEST_EDGE_END = (2.136, -2.848)

EDGE_LENGTH = 1.424

SLICE_THICKNESS = 0.10
SLICE_STEP = 0.10
CLUSTER_MAX_GAP = 0.05
CLUSTER_MIN_POINTS = 3

ROBOT_WIDTH = 0.178
SAFETY_MARGIN = 0.03

REQUIRED_WIDTH = ROBOT_WIDTH + 2 * SAFETY_MARGIN


def scan_to_points(scan: LaserScan) -> np.ndarray:
    """Convert valid LaserScan measurements to Nx2 sensor-frame points."""
    ranges = np.asarray(scan.ranges, dtype=float)

    angles = (
        scan.angle_min
        + np.arange(len(ranges)) * scan.angle_increment
    )

    valid = (
        np.isfinite(ranges)
        & (ranges >= scan.range_min)
        & (ranges <= scan.range_max)
    )

    return np.column_stack((
        ranges[valid] * np.cos(angles[valid]),
        ranges[valid] * np.sin(angles[valid]),
    ))


def generate_grid_edges():
    """Enumerate the directed grid edges available for manual inspection."""
    edges = []

    # Horizontal edges
    for row in range(len(GRID_Y)):
        for col in range(len(GRID_X) - 1):
            edges.append((
                (GRID_X[col], GRID_Y[row]),
                (GRID_X[col + 1], GRID_Y[row]),
            ))

    # Vertical edges
    for row in range(len(GRID_Y) - 1):
        for col in range(len(GRID_X)):
            edges.append((
                (GRID_X[col], GRID_Y[row]),
                (GRID_X[col], GRID_Y[row + 1]),
            ))

    return edges


def choose_current_directed_edge(edges, position, heading, node_radius=0.30):
    """Return (directed edge index, explanation) from robot pose and grid edges.

    Within node_radius of a grid vertex, heading determines the *outgoing*
    edge at that vertex. This avoids selecting an incoming edge when the
    robot is near an intersection. Outside node neighborhoods, project the
    position onto segments and prefer heading among near-equal distances.
    """
    position = np.asarray(position, dtype=float)
    heading = np.asarray(heading, dtype=float)
    if position.shape != (2,) or heading.shape != (2,):
        raise ValueError("position and heading must be XY vectors")
    heading_norm = float(np.linalg.norm(heading))
    if heading_norm < 1e-9:
        raise ValueError("heading vector must be nonzero")
    heading = heading / heading_norm

    nodes = sorted({tuple(a) for a, _ in edges} | {tuple(b) for _, b in edges})
    nearest_node = min(nodes, key=lambda node: np.linalg.norm(position - node))
    node_distance = float(np.linalg.norm(position - nearest_node))
    if node_distance <= node_radius:
        outgoing = []
        for idx, (a, b) in enumerate(edges):
            if tuple(a) != nearest_node:
                continue
            direction = np.asarray(b, dtype=float) - np.asarray(a, dtype=float)
            direction /= np.linalg.norm(direction)
            alignment = float(np.dot(heading, direction))
            outgoing.append((-alignment, idx))
        if outgoing:
            negative_alignment, idx = min(outgoing)
            return idx, (f"near node {nearest_node} ({node_distance:.3f} m), "
                         f"outgoing heading alignment={-negative_alignment:+.2f}")

    candidates = []
    for idx, (a, b) in enumerate(edges):
        start, end = np.asarray(a, float), np.asarray(b, float)
        vector = end - start
        fraction = float(np.clip(np.dot(position - start, vector) /
                                 np.dot(vector, vector), 0.0, 1.0))
        nearest = start + fraction * vector
        distance = float(np.linalg.norm(position - nearest))
        alignment = float(np.dot(heading, vector / np.linalg.norm(vector)))
        candidates.append((distance, -alignment, idx))
    closest = min(item[0] for item in candidates)
    nearby = [item for item in candidates if item[0] <= closest + 0.10]
    distance, neg_alignment, idx = min(nearby, key=lambda item: (item[1], item[0], item[2]))
    return idx, (f"along edge ({distance:.3f} m away), "
                 f"heading alignment={-neg_alignment:+.2f}")


def put_multiline_text(
    image, text, origin, font_scale=0.55,
    color=(30, 30, 30), thickness=1, line_spacing=25,
):
    x, y = origin

    for i, line in enumerate(text.splitlines()):
        cv2.putText(
            image,
            line,
            (x, y + i * line_spacing),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA,
        )


# Small diagnostic views; the edge/trajectory view retains its original size.
DIAGNOSTIC_VIEW_SIZE = 340
# Fixed world extents match the original larger views, just at fewer pixels.
# Sensor: 700 px / 150 px/m = 4.67 m across; Odom: 700 / 100 = 7 m.
SENSOR_VIEW_SCALE = 72     # pixels per metre; fixed across scans
ODOM_VIEW_SCALE = 48       # pixels per metre; fixed across scans


def _draw_fixed_grid(image, to_pixel, center_xy, scale, size):
    """Half-metre grid at a constant scale, including when no returns exist."""
    cx, cy = center_xy
    half_extent = size / (2.0 * scale)
    xs = np.arange(np.floor((cx - half_extent) * 2) / 2,
                   cx + half_extent + 0.5, 0.5)
    ys = np.arange(np.floor((cy - half_extent) * 2) / 2,
                   cy + half_extent + 0.5, 0.5)
    for x in xs:
        px, _ = to_pixel(float(x), cy)
        if 0 <= px < size:
            cv2.line(image, (px, 0), (px, size - 1), (235, 235, 235), 1)
    for y in ys:
        _, py = to_pixel(cx, float(y))
        if 0 <= py < size:
            cv2.line(image, (0, py), (size - 1, py), (235, 235, 235), 1)


def draw_lidar_points(points, size=DIAGNOSTIC_VIEW_SIZE):
    """Fixed-scale sensor view: sensor origin at centre, +X up and +Y left."""
    image = np.full((size, size, 3), 255, dtype=np.uint8)
    scale = SENSOR_VIEW_SCALE

    def to_pixel(x, y):
        return (int(size / 2 - y * scale), int(size / 2 - x * scale))

    _draw_fixed_grid(image, to_pixel, (0.0, 0.0), scale, size)
    for x, y in points:
        px, py = to_pixel(x, y)
        if 0 <= px < size and 0 <= py < size:
            cv2.circle(image, (px, py), 1, (0, 140, 255), -1)
    origin_px = to_pixel(0.0, 0.0)
    cv2.circle(image, origin_px, 4, (255, 100, 0), -1)
    cv2.arrowedLine(image, origin_px,
                    (origin_px[0], origin_px[1] - 20),
                    (255, 100, 0), 1, tipLength=0.3)
    cv2.putText(image, f"Sensor: {len(points)} points", (8, 19),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 30, 30), 1)
    cv2.imshow(SENSOR_WINDOW, image)


def draw_world_points(points, sensor_xy=None, size=DIAGNOSTIC_VIEW_SIZE):
    """Fixed-scale odom-aligned view, following the sensor without zoom changes."""
    image = np.full((size, size, 3), 255, dtype=np.uint8)
    origin = (0.0, 0.0) if sensor_xy is None else sensor_xy
    center_x, center_y = origin
    scale = ODOM_VIEW_SCALE

    def to_pixel(x, y):
        return (int(size / 2 + (x - center_x) * scale),
                int(size / 2 - (y - center_y) * scale))

    _draw_fixed_grid(image, to_pixel, (center_x, center_y), scale, size)
    for x, y in points:
        px, py = to_pixel(x, y)
        if 0 <= px < size and 0 <= py < size:
            cv2.circle(image, (px, py), 1, (0, 140, 255), -1)
    cv2.circle(image, to_pixel(*origin), 4, (255, 100, 0), -1)
    cv2.putText(image, f"Odom: ({origin[0]:.2f}, {origin[1]:.2f}) m",
                (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (30, 30, 30), 1)
    cv2.imshow(ODOM_WINDOW, image)


def draw_gap_annotations(
    image,
    intervals,
    gaps,
    to_pixel,
    slice_s,
    required_width,
):
    """
    Annotate observed obstacle intervals and candidate
    openings on the edge-frame visualization.
    """
    if intervals is None:
        intervals = []

    if gaps is None:
        gaps = []

    _, slice_y = to_pixel(slice_s, 0.0)

    height, width = image.shape[:2]

    # Mark the observed extent of each obstacle cluster.
    for t_min, t_max in intervals:
        x1, _ = to_pixel(slice_s, t_min)
        x2, _ = to_pixel(slice_s, t_max)

        # Red: observed obstacle interval.
        cv2.line(
            image,
            (x1, slice_y),
            (x2, slice_y),
            (0, 0, 255),
            7,
        )

        # Mark the lateral extent of the cluster.
        for x in (x1, x2):
            cv2.circle(
                image,
                (x, slice_y),
                4,
                (0, 0, 255),
                -1,
            )

    # Display the candidate openings between clusters.
    for gap in gaps:
        left = gap["left"]
        right = gap["right"]
        midpoint = gap["midpoint"]
        gap_width = gap["width"]

        x_left, _ = to_pixel(slice_s, left)
        x_right, _ = to_pixel(slice_s, right)
        x_middle, _ = to_pixel(slice_s, midpoint)

        # Green means the measured candidate width
        # meets our initial clearance threshold.
        # Amber means it is narrower than the threshold.
        if gap_width >= required_width:
            color = (0, 160, 0)
        else:
            color = (0, 165, 255)

        annotation_y = slice_y - 35

        # Width indicator.
        cv2.line(
            image,
            (x_left, annotation_y),
            (x_right, annotation_y),
            color,
            2,
        )

        cv2.line(
            image,
            (x_left, annotation_y - 5),
            (x_left, annotation_y + 5),
            color,
            2,
        )

        cv2.line(
            image,
            (x_right, annotation_y - 5),
            (x_right, annotation_y + 5),
            color,
            2,
        )

        # Candidate midpoint.
        cv2.circle(
            image,
            (x_middle, slice_y),
            5,
            color,
            -1,
        )

        label = f"{gap_width:.3f} m"

        # Keep the text inside the visualization.
        text_x = max(5, min(x_middle - 40, width - 100))
        text_y = max(20, annotation_y - 10)

        cv2.putText(
            image,
            label,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
        )


def draw_edge_frame(
    edge_points,
    slice_points,
    slice_s,
    robot_st=None,
    obstacle_intervals=None,
    candidate_gaps=None,
    trajectory_st=None,
    alignment_st=None,
    motion_segments_st=None,
    departure_yaw=None,
    planner_label=None,
    edge_yaw=0.0,
    all_trajectories_st=None,
    rejected_trajectories=None,
    display_mode="feasible",
    display_limit=15,
    total_feasible=0,
    total_rejected=0,
    required_width=REQUIRED_WIDTH,
    nav_status=None,
    size=700,
    scale=300,
):
    """
    Visualize LiDAR points in edge-aligned coordinates.

    Vertical axis: s (distance along edge)
    Horizontal axis: t (lateral offset)

    Positive s points upward.
    Positive t points toward the edge's left side (drawn leftward).
    """
    image = np.full((size, size, 3), 255, dtype=np.uint8)

    # Place the start of the edge near the bottom.
    cx = size // 2
    cy = size - 100

    def to_pixel(s, t):
        return (
            int(cx - t * scale),
            int(cy - s * scale),
        )

    # Display the edge centerline.
    start_px = to_pixel(0.0, 0.0)
    end_px = to_pixel(EDGE_LENGTH, 0.0)

    cv2.line(
        image,
        start_px,
        end_px,
        (255, 0, 0),
        2,
    )

    # Draw a reference line at t=0.
    cv2.line(
        image,
        (cx, 0),
        (cx, size - 1),
        (220, 220, 220),
        1,
    )

    # Highlight the cross-sectional band.
    half = SLICE_THICKNESS / 2

    _, upper_y = to_pixel(slice_s + half, 0)
    _, lower_y = to_pixel(slice_s - half, 0)

    cv2.rectangle(
        image,
        (0, upper_y),
        (size - 1, lower_y),
        (230, 255, 230),
        -1,
    )

    # Draw the selected cross-section.
    _, slice_y = to_pixel(slice_s, 0)

    cv2.line(
        image,
        (0, slice_y),
        (size - 1, slice_y),
        (0, 160, 0),
        1,
    )

    # Draw all nearby points in orange.
    for s, t in edge_points:
        px, py = to_pixel(s, t)

        if 0 <= px < size and 0 <= py < size:
            cv2.circle(
                image,
                (px, py),
                2,
                (0, 140, 255),
                -1,
            )

    # Highlight selected points in green.
    for s, t in slice_points:
        px, py = to_pixel(s, t)

        if 0 <= px < size and 0 <= py < size:
            cv2.circle(
                image,
                (px, py),
                4,
                (0, 170, 0),
                -1,
            )

    draw_gap_annotations(
        image,
        obstacle_intervals,
        candidate_gaps,
        to_pixel,
        slice_s,
        required_width,
    )

    # Draw only a representative sample in debug mode, under viable paths.
    if display_mode == "debug" and rejected_trajectories:
        for trial in rejected_trajectories:
            if len(trial.path_st) < 2:
                continue
            if trial.reason == "Too close to measured returns":
                color = (70, 70, 210)     # red in OpenCV BGR
            elif trial.reason == "Excessive curvature":
                color = (190, 115, 160)   # purple
            else:
                color = (165, 165, 165)   # grey
            coords = np.asarray(
                [to_pixel(s, t) for s, t in trial.path_st], dtype=np.int32
            )
            cv2.polylines(image, [coords], False, color, 1, cv2.LINE_AA)

    # Thin light-blue paths: every feasible sampled trajectory from the
    # last press of 't'. The selected best path is drawn over them.
    if display_mode != "best" and all_trajectories_st:
        for path in all_trajectories_st:
            if len(path) > 1:
                coords = np.asarray([to_pixel(s, t) for s, t in path], dtype=np.int32)
                cv2.polylines(image, [coords], False, (235, 190, 100), 1, cv2.LINE_AA)

    # Independently planned translation legs are NEVER joined across a rotation.
    segments = motion_segments_st if motion_segments_st is not None else (
        (trajectory_st,) if trajectory_st is not None else ()
    )
    for index, segment in enumerate(segments):
        if segment is None or len(segment) < 2:
            continue
        coords = np.asarray([to_pixel(s, t) for s, t in segment], dtype=np.int32)
        color = (0, 145, 0) if index == 0 else (185, 95, 40)
        cv2.polylines(image, [coords], False, color, 3, cv2.LINE_AA)
        # Mark the endpoint of each independently executable translation leg.
        cv2.circle(image, tuple(coords[-1]), 4, color, -1)

    if alignment_st is not None:
        ax, ay = to_pixel(*alignment_st)
        cv2.circle(image, (ax, ay), 7, (0, 210, 255), -1)
        cv2.putText(image, "STOP / ROTATE", (ax + 10, ay - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 125, 190), 1)
        if departure_yaw is not None:
            # Convert world heading to edge-local drawing coordinates.
            # Callers provide world yaw; the selected edge direction is
            # supplied through edge_yaw in this view below.
            delta = departure_yaw - edge_yaw
            ex = ax - int(45 * np.sin(delta))
            ey = ay - int(45 * np.cos(delta))
            cv2.arrowedLine(image, (ax, ay), (ex, ey),
                            (0, 125, 190), 2, cv2.LINE_AA, tipLength=0.25)

    if planner_label:
        cv2.putText(image, f"Plan: {planner_label[:45]}", (15, 76),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (40, 40, 40), 1)

    # Draw the robot in edge coordinates.
    if robot_st is not None:
        px, py = to_pixel(*robot_st)

        if 0 <= px < size and 0 <= py < size:
            cv2.circle(
                image,
                (px, py),
                7,
                (255, 0, 0),
                -1,
            )

    # Compact display counters; computation is unaffected by display limits.
    cv2.putText(
        image,
        f"Mode: {display_mode}  |  +/- {display_limit}",
        (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (45, 45, 45), 1,
    )
    cv2.putText(
        image,
        f"Evaluated: {total_feasible} feasible / {total_rejected} rejected",
        (15, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (45, 45, 45), 1,
    )

    # Show dimensions of observed openings (not assumed free corridors).
    required = required_width
    cv2.putText(image, f"Gap widths (observed) | required: {required:.3f} m",
                (15, 101), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (35, 35, 35), 1)
    if candidate_gaps:
        for idx, gap in enumerate(sorted(candidate_gaps, key=lambda g: g["width"], reverse=True)[:3]):
            usable = max(0.0, gap["width"] - required)
            text = (f"{idx+1}: t=[{gap['left']:+.3f}, {gap['right']:+.3f}] m "
                    f"W={gap['width']:.3f} m; usable={usable:.3f} m")
            cv2.putText(image, text, (15, 122 + 20 * idx),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.41,
                        (0, 115, 0) if gap["width"] >= required else (0, 120, 190), 1)
        if len(candidate_gaps) > 3:
            cv2.putText(image, f"(+{len(candidate_gaps)-3} more; press p for all slices)",
                        (15, 182), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (65, 65, 65), 1)
    else:
        cv2.putText(image, "No internal gap measured at selected slice",
                    (15, 122), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (70, 70, 70), 1)

    # Compact live passability status, separate from trajectory results.
    # The label is right-aligned so it does not obscure the main trajectory view.
    status = nav_status or "NO POSE"
    label = f"NAV CHECK: {status}"
    label_color = ((0, 0, 180) if status == "BLOCKED" else
                   (55, 110, 155) if status == "UNKNOWN" else
                   (0, 120, 0) if status == "CLEAR" else (95, 95, 95))
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size, _ = cv2.getTextSize(label, font, 0.48, 1)
    cv2.putText(image, label, (size - text_size[0] - 12, 28),
                font, 0.48, label_color, 1, cv2.LINE_AA)

    # A compact two-row footer keeps controls visible without obscuring paths.
    # Reserve the lower-right corner for the selected longitudinal position.
    slice_label = f"s={slice_s:.2f} m"
    footer_scale = 0.40
    slice_label_size, _ = cv2.getTextSize(slice_label, font, footer_scale, 1)
    slice_x = size - slice_label_size[0] - 12
    footer_rows = (
        "h/l edges  j/k slices  c current  a auto",
        "t plan  b check  p log  v view  +/- count  z/x curve  m/n margin",
    )
    for row_index, footer_text in enumerate(footer_rows):
        baseline_y = size - 35 + row_index * 20
        available_width = (size - 24) if row_index == 0 else (slice_x - 22)
        scale = footer_scale
        while scale > 0.28 and cv2.getTextSize(footer_text, font, scale, 1)[0][0] > available_width:
            scale -= 0.02
        cv2.putText(image, footer_text, (12, baseline_y), font, scale,
                    (75, 75, 75), 1, cv2.LINE_AA)
    cv2.putText(image, slice_label, (slice_x, size - 15),
                font, footer_scale, (70, 70, 70), 1, cv2.LINE_AA)

    cv2.imshow(EDGE_WINDOW, image)


class LidarExplorer(Node):
    def __init__(self):
        super().__init__("lidar_explorer")

        # ROS subscriptions.
        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        # TF listener and buffer.
        self.tf_buffer = Buffer(node=self)  # Clear stale TF data on ROS clock jumps.
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # Latest observations.
        self.latest_odom = None
        self.latest_world_points = None
        self.latest_edge_observation = None
        self.planner_safety_margin = SAFETY_MARGIN
        self.opening_tracker = OpeningProgressTracker(robot_radius=ROBOT_WIDTH / 2, safety_margin=self.planner_safety_margin)
        self.latest_opening_diagnostics = ()
        self.planner_max_curvature = 8.0
        self.latest_robot_pose = None  # Base pose from odometry, not sensor origin.
        self.latest_passability = None
        self.latest_slice_results = []
        self.latest_trajectory = None
        self.latest_alignment_st = None
        self.latest_plan = None
        self.latest_feasible_trajectories = []
        self.latest_rejected_trajectories = []
        self.latest_sensor_xy = None
        self.latest_sensor_yaw = None
        self.display_mode = "feasible"
        self.display_limit = 15

        # The latest scan awaiting its timestamp-matched TF.
        self.pending_scan = None
        self.last_scan_stamp_ns = None

        # Diagnostic counters.
        self.received_scans = 0
        self.transformed_scans = 0
        self.dropped_scans = 0

        # Cross-section parameters.
        self.slice_thickness = SLICE_THICKNESS
        self.slice_step = SLICE_STEP
        base_edges = generate_grid_edges()
        # Preserve original manual selection order; add reverse directions so
        # auto-selection can match the robot heading on either traversal.
        self.edges = base_edges + [(b, a) for a, b in base_edges]

        # Initially select the previously tested edge.
        self.edge_index = self.edges.index((
            (0.712, -2.848),
            (2.136, -2.848),
        ))

        self.slice_s = 0.712
        self.focus_narrowest_on_next_scan = False
        self.auto_current_edge = False
        self.last_log_time = self.get_clock().now()

        # Initialize all windows and paint immediately, even before /scan or TF
        # arrive. Small sensor/odom views sit beside the larger edge view.
        for name in (SENSOR_WINDOW, ODOM_WINDOW, EDGE_WINDOW):
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(SENSOR_WINDOW, DIAGNOSTIC_VIEW_SIZE, DIAGNOSTIC_VIEW_SIZE)
        cv2.resizeWindow(ODOM_WINDOW, DIAGNOSTIC_VIEW_SIZE, DIAGNOSTIC_VIEW_SIZE)
        cv2.resizeWindow(EDGE_WINDOW, 700, 700)
        cv2.moveWindow(SENSOR_WINDOW, 30, 35)
        cv2.moveWindow(ODOM_WINDOW, 395, 35)
        cv2.moveWindow(EDGE_WINDOW, 760, 35)
        placeholder = np.full((DIAGNOSTIC_VIEW_SIZE, DIAGNOSTIC_VIEW_SIZE, 3), 255, np.uint8)
        for name in (SENSOR_WINDOW, ODOM_WINDOW):
            cv2.imshow(name, placeholder)
        cv2.imshow(EDGE_WINDOW, np.full((700, 700, 3), 255, np.uint8))
        # Request visible, above-other-window placement without requiring mouse
        # focus. TOPMOST is backend/desktop-dependent, so fail gracefully.
        if hasattr(cv2, "WND_PROP_TOPMOST"):
            for name in (SENSOR_WINDOW, ODOM_WINDOW, EDGE_WINDOW):
                try:
                    cv2.setWindowProperty(name, cv2.WND_PROP_TOPMOST, 1)
                except cv2.error:
                    pass
        cv2.waitKey(1)  # Process initial window show/resize/move events.

        # Check for available TF data at 20 Hz.
        self.create_timer(0.05, self.process_pending_scan)

        # Process OpenCV GUI events independently of TF availability.
        self.create_timer(0.05, self.update_gui)
        # Auto mode reevaluates both the directed edge and its diagnostic slice
        # once per second, using the next transformed LiDAR scan.
        self.create_timer(1.0, self.refresh_current_edge)

    @property
    def selected_edge(self):
        """Currently selected directed edge (world coordinates)."""
        return self.edges[self.edge_index]

    def log_selected_edge(self):
        """Log the selected edge for reproducible manual testing."""
        start, end = self.selected_edge

        self.get_logger().info(
            f"Selected edge {self.edge_index + 1}/"
            f"{len(self.edges)}: {start} -> {end}"
        )
        
    def reset_edge_diagnostics(self):
        """Invalidate edge-frame data and cached candidate paths after selection."""
        self.latest_edge_observation = None
        self.latest_slice_results = []
        self.opening_tracker.reset()
        self.latest_opening_diagnostics = ()
        self.latest_trajectory = None
        self.latest_alignment_st = None
        self.latest_plan = None
        self.latest_feasible_trajectories = []
        self.latest_rejected_trajectories = []
        self.latest_passability = None

    def select_nearest_edge(self, *, only_on_change=False):
        """Find the likely current directed edge from position and heading.

        Automatic updates also refocus the passability-selected slice, even
        when the directed edge is unchanged. Explicit 'c' forces an immediate
        edge selection and refocus.
        """
        if self.latest_sensor_xy is None or self.latest_sensor_yaw is None:
            if not only_on_change:
                self.get_logger().warn("No transformed scan/pose yet; cannot locate current edge")
            return
        position = np.asarray(self.latest_sensor_xy, dtype=float)
        heading = np.array((np.cos(self.latest_sensor_yaw),
                            np.sin(self.latest_sensor_yaw)))
        index, context = choose_current_directed_edge(self.edges, position, heading)
        if only_on_change and index == self.edge_index:
            # Keep the current edge and cached plan, but reevaluate the
            # diagnostic slice on the next transformed scan.
            self.focus_narrowest_on_next_scan = True
            return
        self.edge_index = index
        self.slice_s = 0.712
        self.reset_edge_diagnostics()
        self.focus_narrowest_on_next_scan = True
        self.log_selected_edge()
        self.get_logger().info(
            f"Current edge selection: {context}; "
            "focusing passability rejection slice (or narrowest observed gap) on next scan"
        )

    def refresh_current_edge(self):
        """On auto mode, refresh the edge and its passability slice each second."""
        if self.auto_current_edge:
            self.select_nearest_edge(only_on_change=True)

    def log_slice_profile(self):
        """Log measured gap intervals for each longitudinal slice."""
        if not self.latest_slice_results:
            return

        start, end = self.selected_edge

        self.get_logger().info(
            f"Width profile: {start} -> {end}"
        )

        for result in self.latest_slice_results:
            gaps = result["gaps"]

            if gaps:
                gap_text = ", ".join(
                    f"[{gap['left']:+.2f}, {gap['right']:+.2f}] "
                    f"w={gap['width']:.3f}"
                    for gap in gaps
                )
            else:
                gap_text = "none observed"

            self.get_logger().info(
                f"s={result['s']:.2f} | "
                f"points={result['point_count']:2d} | "
                f"clusters={len(result['intervals'])} | "
                f"gaps={gap_text}"
            )
    
    def reset_after_clock_rewind(self, previous_ns, current_ns):
        """Discard pre-reset TF and observations when Gazebo rewinds /clock."""
        self.tf_buffer.clear()
        self.latest_robot_pose = None
        self.latest_passability = None
        self.pending_scan = None
        self.latest_odom = None
        self.latest_world_points = None
        self.latest_sensor_xy = None
        self.latest_sensor_yaw = None
        self.latest_robot_st = None
        self.reset_edge_diagnostics()
        self.focus_narrowest_on_next_scan = False
        self.last_log_time = self.get_clock().now()
        self.get_logger().warn(
            f"Detected simulation-time rewind ({previous_ns / 1e9:.3f} -> "
            f"{current_ns / 1e9:.3f} s); cleared stale TF and explorer state"
        )

    def scan_callback(self, scan):
        """Receive a scan and retain it until its TF is available."""
        stamp_ns = int(scan.header.stamp.sec) * 1_000_000_000 + int(scan.header.stamp.nanosec)
        # Permit small out-of-order deliveries but detect a real Gazebo reset.
        if (self.last_scan_stamp_ns is not None
                and stamp_ns < self.last_scan_stamp_ns - 1_000_000_000):
            self.reset_after_clock_rewind(self.last_scan_stamp_ns, stamp_ns)
        self.last_scan_stamp_ns = max(stamp_ns, self.last_scan_stamp_ns or stamp_ns) if (
            self.last_scan_stamp_ns is not None
            and stamp_ns >= self.last_scan_stamp_ns - 1_000_000_000
        ) else stamp_ns
        self.received_scans += 1

        points = scan_to_points(scan)

        # Sensor-frame drawing does not require TF.
        draw_lidar_points(points)

        # Keep only the newest scan for this live visualization.
        if self.pending_scan is not None:
            self.dropped_scans += 1

        self.pending_scan = scan

    def process_pending_scan(self):
        """Transform a pending scan when its timestamped TF is ready."""
        if self.pending_scan is None:
            return

        scan = self.pending_scan

        try:
            transform = self.tf_buffer.lookup_transform(
                "odom",
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
            )

        except TransformException:
            # TF may be slightly behind the scan.
            # Retry when the timer fires again.
            return

        # Matching transform found.
        self.pending_scan = None

        points = scan_to_points(scan)

        self.latest_world_points = transform_points_2d(
            points,
            transform.transform.translation,
            transform.transform.rotation,
        )

        start, end = self.selected_edge

        edge_length = float(np.linalg.norm(
            np.asarray(end) - np.asarray(start)
        ))

        # The reusable perception API now owns edge filtering,
        # cross-sectional sampling, clustering and gap estimation.
        observation = analyze_edge(
            self.latest_world_points,
            start,
            end,
            slice_step=SLICE_STEP,
            slice_thickness=SLICE_THICKNESS,
            cluster_max_gap=CLUSTER_MAX_GAP,
            cluster_min_points=CLUSTER_MIN_POINTS,
            lateral_range=1.0,
        )
        self.latest_edge_observation = observation
        # Match the navigator's base-position input rather than the TF sensor
        # origin. Disallow comparisons when odometry is far from scan time.
        base_pose = None
        if self.latest_odom is not None:
            odom = self.latest_odom
            odom_ns = odom.header.stamp.sec * 1_000_000_000 + odom.header.stamp.nanosec
            scan_ns = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
            if abs(odom_ns - scan_ns) <= 300_000_000:
                op = odom.pose.pose
                oq = op.orientation
                oyaw = float(np.arctan2(
                    2.0 * (oq.w * oq.z + oq.x * oq.y),
                    1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z),
                ))
                base_pose = (float(op.position.x), float(op.position.y), oyaw)
        self.latest_robot_pose = base_pose
        tracked_xy = base_pose[:2] if base_pose is not None else (
            transform.transform.translation.x, transform.transform.translation.y)
        self.latest_robot_st, self.latest_opening_diagnostics = self.opening_tracker.update(
            observation, tracked_xy,
        )
        self.latest_passability = (
            assess_passability(observation, base_pose[:2],
                               robot_radius=ROBOT_WIDTH / 2, safety_margin=SAFETY_MARGIN)
            if base_pose is not None else None
        )

        bottlenecks = detect_bottlenecks(
            self.latest_edge_observation,
            robot_radius=ROBOT_WIDTH / 2,
            safety_margin=self.planner_safety_margin,
        )

        if bottlenecks:
            for bottleneck in bottlenecks:
                self.get_logger().warn(
                    f"Possible bottleneck at s={bottleneck.s:.2f} m: "
                    f"widest={bottleneck.widest_gap:.3f} m, "
                    f"required={bottleneck.required_width:.3f} m",
                    throttle_duration_sec=2.0,
                )

        filtered_points = observation.obstacle_points_st

        # Preserve the width-profile structure expected by the 'p' key logger.
        self.latest_slice_results = [
            {
                "s": section.s,
                "point_count": section.point_count,
                "intervals": section.obstacle_intervals,
                "gaps": [
                    {
                        "left": gap.left,
                        "right": gap.right,
                        "width": gap.width,
                        "midpoint": gap.midpoint,
                    }
                    for gap in section.gaps
                ],
            }
            for section in observation.slices
        ]

        if self.focus_narrowest_on_next_scan:
            self.focus_narrowest_on_next_scan = False
            # Prefer the precise section named in perception.assess_passability()'s
            # BLOCKED result. This is the first restrictive section AHEAD of the
            # robot, not necessarily the globally narrowest observed opening.
            # The current PassabilityResult API exposes its s in `reason`, not
            # as a structured attribute; if the message changes, fall back.
            selected_s = None
            result = self.latest_passability
            if result is not None and result.status.name == "BLOCKED":
                match = re.search(r"All observed internal openings are narrow at s=([+-]?\d+(?:\.\d+)?) m", result.reason)
                if match:
                    reported_s = float(match.group(1))
                    if observation.slices:
                        section = min(observation.slices, key=lambda item: abs(item.s - reported_s))
                        if abs(section.s - reported_s) <= 0.055:
                            selected_s = section.s
                            self.get_logger().info(
                                f"Current edge: focused passability rejection slice "
                                f"at s={selected_s:.2f} m"
                            )
            if selected_s is None:
                candidates = [
                    (max(gap.width for gap in section.gaps), section.s)
                    for section in observation.slices if section.gaps
                ]
                if candidates:
                    limiting_width, selected_s = min(candidates)
                    self.get_logger().info(
                        f"Current edge: no narrow-gap rejection slice; "
                        f"focused narrowest observed opening at s={selected_s:.2f} m "
                        f"(widest gap W={limiting_width:.3f} m)"
                    )
                else:
                    self.get_logger().info(
                        "Current edge: no measured internal gaps; keeping midpoint slice"
                    )
            if selected_s is not None:
                self.slice_s = float(selected_s)

        # Select points belonging to the current cross-section.
        slice_points = extract_slice(
            filtered_points,
            self.slice_s,
            SLICE_THICKNESS,
        )

        obstacle_intervals = cluster_lateral_points(
            slice_points,
            max_gap=CLUSTER_MAX_GAP,
            min_points=CLUSTER_MIN_POINTS,
        )

        candidate_gaps = find_gaps(obstacle_intervals)

        # Convert the LiDAR sensor's position into edge coordinates.
        sensor_xy = (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

        self.latest_sensor_xy = sensor_xy
        q = transform.transform.rotation
        self.latest_sensor_yaw = float(np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        ))

        sensor_st = points_to_edge_frame(
            np.asarray([tracked_xy]),
            start,
            end,
        )[0]

        # Only render diverse examples; never change search or scoring.
        alternatives = [
            trial for trial in self.latest_feasible_trajectories
            if trial is not self.latest_trajectory
        ]
        shown_feasible = select_representatives(alternatives, self.display_limit)
        shown_rejected = group_rejections(
            self.latest_rejected_trajectories,
            per_reason=min(5, self.display_limit),
            include_other=False,
        ) if self.display_mode == "debug" else []

        # Draw the edge-aligned visualization.
        draw_edge_frame(
            filtered_points,
            slice_points,
            self.slice_s,
            robot_st=sensor_st,
            obstacle_intervals=obstacle_intervals,
            candidate_gaps=candidate_gaps,
            trajectory_st=(self.latest_trajectory.path_st if self.latest_trajectory is not None and self.latest_trajectory.feasible else None),
            alignment_st=self.latest_alignment_st,
            motion_segments_st=(self.latest_plan.segments_st if self.latest_plan is not None else None),
            departure_yaw=(self.latest_plan.departure_yaw if self.latest_plan is not None else None),
            planner_label=(self.latest_plan.mode if self.latest_plan is not None else None),
            edge_yaw=float(np.arctan2(self.selected_edge[1][1]-self.selected_edge[0][1], self.selected_edge[1][0]-self.selected_edge[0][0])),
            all_trajectories_st=[t.path_st for t in shown_feasible],
            rejected_trajectories=shown_rejected,
            display_mode=self.display_mode,
            display_limit=self.display_limit,
            total_feasible=len(self.latest_feasible_trajectories),
            total_rejected=len(self.latest_rejected_trajectories),
            required_width=ROBOT_WIDTH + 2 * self.planner_safety_margin,
            nav_status=(self.latest_passability.status.name if self.latest_passability is not None else None),
        )

        # Draw the odometry-frame visualization.
        draw_world_points(
            self.latest_world_points,
            sensor_xy=sensor_xy,
        )

        self.transformed_scans += 1

        self.get_logger().info(
            "First successful timestamp-aligned TF transform",
            once=True,
        )

        self.get_logger().info(
            f"Cross-section at s={self.slice_s:.3f} m: "
            f"{len(slice_points)} points",
            throttle_duration_sec=2.0,
        )

    def odom_callback(self, msg):
        """Store odometry and periodically report the robot pose."""
        self.latest_odom = msg

        elapsed = (
            self.get_clock().now() - self.last_log_time
        ).nanoseconds * 1e-9

        if elapsed < 1.0:
            return

        pose = msg.pose.pose
        q = pose.orientation

        yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.get_logger().info(
            f"position=({pose.position.x:.3f}, "
            f"{pose.position.y:.3f}), "
            f"yaw={np.degrees(yaw):.1f} deg, "
            f"scans={self.received_scans}, "
            f"transformed={self.transformed_scans}, "
            f"replaced={self.dropped_scans}"
        )

        self.last_log_time = self.get_clock().now()

    def update_gui(self):
        """Poll nonblocking OpenCV keys and update diagnostic display controls."""
        key = cv2.waitKey(1) & 0xFF

        if key == ord("k"):
            self.slice_s = min(
                1.424,
                self.slice_s + SLICE_STEP,
            )

        elif key == ord("j"):
            self.slice_s = max(
                0.0,
                self.slice_s - SLICE_STEP,
            )

        elif key == ord("l"):
            self.edge_index = (
                self.edge_index + 1
            ) % len(self.edges)

            self.slice_s = 0.712
            self.reset_edge_diagnostics()
            self.log_selected_edge()

        elif key == ord("h"):
            self.edge_index = (
                self.edge_index - 1
            ) % len(self.edges)

            self.slice_s = 0.712
            self.reset_edge_diagnostics()
            self.log_selected_edge()

        elif key == ord("c"):
            self.select_nearest_edge()

        elif key == ord("a"):
            self.auto_current_edge = not self.auto_current_edge
            self.get_logger().info(
                f"Automatic current-edge selection: {'ON' if self.auto_current_edge else 'OFF'} "
                "(1 s interval; refresh passability slice even on the same edge)"
            )
            if self.auto_current_edge:
                self.refresh_current_edge()

        elif key == ord("v"):
            modes = ("best", "feasible", "debug")
            self.display_mode = modes[(modes.index(self.display_mode) + 1) % len(modes)]
            self.get_logger().info(f"Trajectory display: {self.display_mode}")

        elif key in (ord("+"), ord("=")):
            self.display_limit = min(60, self.display_limit + 5)
            self.get_logger().info(f"Trajectory alternatives: {self.display_limit}")

        elif key in (ord("-"), ord("_")):
            self.display_limit = max(0, self.display_limit - 5)
            self.get_logger().info(f"Trajectory alternatives: {self.display_limit}")

        elif key == ord("p"):
            self.log_slice_profile()

        elif key == ord("x"):
            self.planner_max_curvature = min(15.0, self.planner_max_curvature + 1.0)
            self.get_logger().info(f"Planner max curvature: {self.planner_max_curvature:.1f} /m")

        elif key == ord("z"):
            self.planner_max_curvature = max(2.0, self.planner_max_curvature - 1.0)
            self.get_logger().info(f"Planner max curvature: {self.planner_max_curvature:.1f} /m")

        elif key == ord("m"):
            self.planner_safety_margin = max(0.0, round(self.planner_safety_margin - 0.01, 3))
            self.opening_tracker.safety_margin = self.planner_safety_margin
            self.opening_tracker.reset()
            self.latest_edge_observation = None
            self.latest_trajectory = None
            self.latest_alignment_st = None
            self.latest_plan = None
            self.latest_feasible_trajectories = []
            self.latest_rejected_trajectories = []
            self.get_logger().info(
                f"Safety margin={self.planner_safety_margin:.3f} m; "
                f"required opening={ROBOT_WIDTH + 2 * self.planner_safety_margin:.3f} m. "
                "Wait for next scan, then press t."
            )

        elif key == ord("n"):
            self.planner_safety_margin = min(0.06, round(self.planner_safety_margin + 0.01, 3))
            self.opening_tracker.safety_margin = self.planner_safety_margin
            self.opening_tracker.reset()
            self.latest_edge_observation = None
            self.latest_trajectory = None
            self.latest_alignment_st = None
            self.latest_plan = None
            self.latest_feasible_trajectories = []
            self.latest_rejected_trajectories = []
            self.get_logger().info(
                f"Safety margin={self.planner_safety_margin:.3f} m; "
                f"required opening={ROBOT_WIDTH + 2 * self.planner_safety_margin:.3f} m. "
                "Wait for next scan, then press t."
            )

        elif key == ord("t"):
            self.get_logger().info(
                f"Planner geometry: radius={ROBOT_WIDTH / 2:.3f} m, "
                f"safety_margin={self.planner_safety_margin:.3f} m, "
                f"required_clearance={ROBOT_WIDTH / 2 + self.planner_safety_margin:.3f} m"
            )
            if self.latest_edge_observation is None or self.latest_robot_pose is None:
                self.get_logger().info("No usable scan/pose yet (navigator mode requires /odom within 0.3s of /scan)")
            else:
                start, end = self.selected_edge
                robot_s, robot_t = self.latest_robot_st
                edge_yaw = float(np.arctan2(end[1] - start[1], end[0] - start[0]))
                rel_yaw = float(np.arctan2(
                    np.sin(self.latest_sensor_yaw - edge_yaw),
                    np.cos(self.latest_sensor_yaw - edge_yaw),
                ))
                self.get_logger().info(
                    f"Opening diagnostic: edge={start}->{end}, "
                    f"robot=(s={robot_s:.3f}, t={robot_t:.3f}), "
                    f"yaw_relative={np.degrees(rel_yaw):+.1f} deg"
                )
                # Print nearby candidates first; log all observed gaps for transparency.
                for opening in sorted(self.latest_opening_diagnostics,
                                      key=lambda g: abs(g.slice_s - robot_s)):
                    self.get_logger().info(
                        f"Opening s={opening.slice_s:.2f} "
                        f"gap=[{opening.gap_left:+.3f}, {opening.gap_right:+.3f}] "
                        f"usable_t=[{opening.usable_left:+.3f}, {opening.usable_right:+.3f}] "
                        f"state={opening.state}, entered={opening.observed_entry}, "
                        f"behind_robot_by_s={opening.passed_by_longitudinal_rule}"
                    )
                pose = self.latest_robot_pose
                report = plan_motion(
                    self.latest_edge_observation,
                    pose[:2],
                    robot_yaw=pose[2],
                    max_curvature=self.planner_max_curvature,
                    robot_radius=ROBOT_WIDTH / 2,
                    safety_margin=self.planner_safety_margin,
                )
                result = assess_passability(
                    self.latest_edge_observation, pose[:2],
                    robot_radius=ROBOT_WIDTH / 2, safety_margin=SAFETY_MARGIN)
                self.latest_passability = result
                self.get_logger().info(
                    f"NAV CHECK: {result.status.name}; "
                    f"planner={'PROPOSAL' if report.plan else 'HOLD'} "
                    f"({report.mode}); reason={result.reason}")
                if result.status.name == "BLOCKED" and report.plan is not None:
                    self.get_logger().warn(
                        "PLANNER HAS PROPOSAL, PASSABILITY SAYS BLOCKED. "
                        "Navigator may reject edge; no command is simulated.")
                search = report.search
                self.latest_plan = report.plan
                self.latest_alignment_st = (
                    report.plan.rotation_st if report.plan is not None else None
                )
                self.latest_feasible_trajectories = list(search.feasible)
                self.latest_rejected_trajectories = list(search.rejected)
                self.latest_trajectory = (
                    search.feasible[0] if search.feasible else None
                )
                if report.plan is not None and self.latest_trajectory is None:
                    # Debug compatibility: a hypothetical future translation is
                    # displayed, but NOT added to the current-heading search stats.
                    first = report.plan.segments_st[0]
                    self.latest_trajectory = TrajectoryResult(
                        True, "Proposed phase after rotation (re-scan required)",
                        first, report.plan.clearance, None,
                        report.plan.candidate,
                    )
                passage = report.diagnostics.get("selected_passage")
                if passage is not None:
                    self.get_logger().info(
                        f"Selected passage: s=[{passage.s_start:.2f}, {passage.s_end:.2f}], "
                        f"usable_t=[{passage.usable_left:+.3f}, {passage.usable_right:+.3f}]"
                    )
                for stage, result in report.diagnostics.items():
                    if hasattr(result, "feasible") and hasattr(result, "rejected"):
                        reasons = Counter(r.reason for r in result.rejected)
                        self.get_logger().info(
                            f"Stage {stage}: {len(result.feasible)} feasible; "
                            f"rejected={dict(reasons)}"
                        )
                self.get_logger().info(f"Selected plan: {report.mode}")
                if report.plan is not None:
                    p = report.plan
                    self.get_logger().info(
                        f"Motion phases: {len(p.segments_st)} translation leg(s); "
                        f"rotation={p.requires_rotation}; "
                        f"departure_yaw={np.degrees(p.departure_yaw):.1f} deg"
                        if p.requires_rotation else
                        f"Motion phases: {len(p.segments_st)} translation leg(s), no rotation"
                    )
                    if p.requires_rescan:
                        self.get_logger().warn(
                            "Hypothetical continuation ONLY: stop, rotate, "
                            "obtain a fresh scan, and replan before translating"
                        )
                else:
                    self.get_logger().info("No motion proposal; hold position")
                self.get_logger().info(
                    f"Current-heading search: {len(search.feasible)} feasible / "
                    f"{len(search.rejected)} rejected"
                )

        elif key == ord("b"):
            if self.latest_edge_observation is None:
                self.get_logger().info("No edge observation available yet")
            else:
                # Use the pose corresponding to the transformed scan, not
                # the edge start: previously passed restrictions are irrelevant.
                start, end = self.selected_edge
                pose = self.latest_robot_pose
                if pose is None:
                    self.get_logger().info("No sufficiently recent robot pose available")
                    return
                result = assess_passability(
                    self.latest_edge_observation,
                    robot_position=pose[:2],
                    robot_radius=ROBOT_WIDTH / 2,
                    safety_margin=self.planner_safety_margin,
                )
                self.latest_passability = result
                self.get_logger().info(
                    f"Edge {start} -> {end}: {result.status.name} - "
                    f"{result.reason}; narrowest flagged gap="
                    f"{result.minimum_observed_width}"
                )


def main():
    rclpy.init()
    node = LidarExplorer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
