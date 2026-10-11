
#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from perception import analyze_edge, detect_bottlenecks, assess_passability, plan_candidate_trajectory, search_candidate_trajectories
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
SAFETY_MARGIN = 0.04

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


def draw_lidar_points(points, size=700, scale=150):
    """Draw LiDAR points in the robot's local sensor frame."""
    image = np.full((size, size, 3), 255, dtype=np.uint8)

    cx = size // 2
    cy = size // 2

    # Draw a reference grid every 0.5 m.
    grid_step = int(0.5 * scale)

    for offset in range(0, size // 2, grid_step):
        for sign in (-1, 1):
            x = cx + sign * offset
            y = cy + sign * offset

            cv2.line(
                image, (x, 0), (x, size - 1),
                (230, 230, 230), 1,
            )
            cv2.line(
                image, (0, y), (size - 1, y),
                (230, 230, 230), 1,
            )

    # Sensor-frame convention:
    # +X points upward, +Y points left.
    for x, y in points:
        px = int(cx - y * scale)
        py = int(cy - x * scale)

        if 0 <= px < size and 0 <= py < size:
            cv2.circle(
                image, (px, py), 2, (0, 140, 255), -1
            )

    # Sensor origin.
    cv2.circle(
        image, (cx, cy), 7, (255, 100, 0), -1
    )

    # Forward direction (+X).
    cv2.arrowedLine(
        image,
        (cx, cy),
        (cx, cy - 45),
        (255, 100, 0),
        2,
    )

    cv2.putText(
        image,
        f"{len(points)} LiDAR points",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (30, 30, 30),
        2,
    )

    cv2.imshow(SENSOR_WINDOW, image)


def draw_world_points(points, sensor_xy=None, size=700, scale=100):
    """
    Draw LiDAR points in the odom frame with a camera that
    follows the sensor position without rotating.

    Points remain expressed in odom coordinates.
    """
    image = np.full((size, size, 3), 255, dtype=np.uint8)

    # Center the camera on the sensor.
    if sensor_xy is not None:
        center_x, center_y = sensor_xy
    else:
        center_x, center_y = 0.0, 0.0

    cx = size // 2
    cy = size // 2

    def to_pixel(x, y):
        return (
            int(cx + (x - center_x) * scale),
            int(cy - (y - center_y) * scale),
        )

    # Draw odom-aligned grid lines every 0.5 m.
    grid_spacing = 0.5

    x_min = center_x - cx / scale
    x_max = center_x + cx / scale
    y_min = center_y - cy / scale
    y_max = center_y + cy / scale

    x_values = np.arange(
        np.floor(x_min / grid_spacing) * grid_spacing,
        x_max + grid_spacing,
        grid_spacing,
    )

    y_values = np.arange(
        np.floor(y_min / grid_spacing) * grid_spacing,
        y_max + grid_spacing,
        grid_spacing,
    )

    for x in x_values:
        px, _ = to_pixel(x, center_y)
        cv2.line(image, (px, 0), (px, size - 1),
                 (230, 230, 230), 1)

    for y in y_values:
        _, py = to_pixel(center_x, y)
        cv2.line(image, (0, py), (size - 1, py),
                 (230, 230, 230), 1)

    # Draw LiDAR observations.
    for x, y in points:
        px, py = to_pixel(x, y)

        if 0 <= px < size and 0 <= py < size:
            cv2.circle(image, (px, py), 2, (0, 140, 255), -1)

    # Sensor position is at the center of the view.
    cv2.circle(image, (cx, cy), 7, (255, 100, 0), -1)

    cv2.putText(
        image,
        f"Odom position: ({center_x:.2f}, {center_y:.2f}) m",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (30, 30, 30),
        2,
    )

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
    all_trajectories_st=None,
    rejected_trajectories=None,
    display_mode="feasible",
    display_limit=15,
    total_feasible=0,
    total_rejected=0,
    size=700,
    scale=300,
):
    """
    Visualize LiDAR points in edge-aligned coordinates.

    Vertical axis: s (distance along edge)
    Horizontal axis: t (lateral offset)

    Positive s points upward.
    Positive t points rightward.
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
        REQUIRED_WIDTH,
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

    # Overlay best candidate trajectory in thicker green.

    if trajectory_st is not None and len(trajectory_st) > 1:
        coords = np.asarray([to_pixel(s,t) for s,t in trajectory_st], dtype=np.int32)
        cv2.polylines(image, [coords], False, (0, 155, 0), 2)

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
        f"Mode: {display_mode}  (+/-: {display_limit} alternatives)",
        (15, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (45, 45, 45), 1,
    )
    cv2.putText(
        image,
        f"Evaluated: {total_feasible} feasible / {total_rejected} rejected",
        (15, 114), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (45, 45, 45), 1,
    )

    # Labels.
    cv2.putText(
        image,
        f"Slice s={slice_s:.2f} m",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (30, 30, 30),
        2,
    )

    cv2.putText(
        image,
        f"Selected points: {len(slice_points)}",
        (15, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (30, 30, 30),
        2,
    )

    put_multiline_text(
        image,
        "Use [ and ] to move the slice\n"
        "Use , and . to change the edge\n"
        "Use p to log; b to assess; t to plan\n"
        "Use v to change view; +/- to adjust count",
        (15, size - 60),
    )

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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # Latest observations.
        self.latest_odom = None
        self.latest_world_points = None
        self.latest_edge_observation = None
        self.latest_slice_results = []
        self.latest_trajectory = None
        self.latest_feasible_trajectories = []
        self.latest_rejected_trajectories = []
        self.latest_sensor_xy = None
        self.latest_sensor_yaw = None
        self.display_mode = "feasible"
        self.display_limit = 15

        # The latest scan awaiting its timestamp-matched TF.
        self.pending_scan = None

        # Diagnostic counters.
        self.received_scans = 0
        self.transformed_scans = 0
        self.dropped_scans = 0

        # Cross-section parameters.
        self.slice_thickness = SLICE_THICKNESS
        self.slice_step = SLICE_STEP
        self.edges = generate_grid_edges()

        # Initially select the previously tested edge.
        self.edge_index = self.edges.index((
            (0.712, -2.848),
            (2.136, -2.848),
        ))

        self.slice_s = 0.712
        self.last_log_time = self.get_clock().now()

        # Prepare both OpenCV windows.
        cv2.namedWindow(SENSOR_WINDOW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(ODOM_WINDOW, cv2.WINDOW_NORMAL)

        cv2.moveWindow(SENSOR_WINDOW, 40, 40)
        cv2.moveWindow(ODOM_WINDOW, 780, 40)

        cv2.namedWindow(EDGE_WINDOW, cv2.WINDOW_NORMAL)
        cv2.moveWindow(EDGE_WINDOW, 1520, 40)

        # Check for available TF data at 20 Hz.
        self.create_timer(0.05, self.process_pending_scan)

        # Process OpenCV GUI events independently of TF availability.
        self.create_timer(0.05, self.update_gui)

    @property
    def selected_edge(self):
        return self.edges[self.edge_index]

    def log_selected_edge(self):
        start, end = self.selected_edge

        self.get_logger().info(
            f"Selected edge {self.edge_index + 1}/"
            f"{len(self.edges)}: {start} -> {end}"
        )
        
    def log_slice_profile(self):
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
    
    def scan_callback(self, scan):
        """Receive a scan and retain it until its TF is available."""
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

        bottlenecks = detect_bottlenecks(
            self.latest_edge_observation,
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

        # Keep the legacy format for the existing 'p' key logger.
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
            np.asarray([sensor_xy]),
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
            all_trajectories_st=[t.path_st for t in shown_feasible],
            rejected_trajectories=shown_rejected,
            display_mode=self.display_mode,
            display_limit=self.display_limit,
            total_feasible=len(self.latest_feasible_trajectories),
            total_rejected=len(self.latest_rejected_trajectories),
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
        key = cv2.waitKey(1) & 0xFF

        if key == ord("]"):
            self.slice_s = min(
                1.424,
                self.slice_s + SLICE_STEP,
            )

        elif key == ord("["):
            self.slice_s = max(
                0.0,
                self.slice_s - SLICE_STEP,
            )

        elif key == ord("."):
            self.edge_index = (
                self.edge_index + 1
            ) % len(self.edges)

            self.slice_s = 0.712
            self.latest_edge_observation = None
            self.latest_slice_results = []
            self.latest_trajectory = None
            self.latest_feasible_trajectories = []
            self.latest_rejected_trajectories = []
            self.log_selected_edge()

        elif key == ord(","):
            self.edge_index = (
                self.edge_index - 1
            ) % len(self.edges)

            self.slice_s = 0.712
            self.latest_edge_observation = None
            self.latest_slice_results = []
            self.latest_trajectory = None
            self.latest_feasible_trajectories = []
            self.latest_rejected_trajectories = []
            self.log_selected_edge()

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

        elif key == ord("t"):
            if self.latest_edge_observation is None or self.latest_sensor_xy is None:
                self.get_logger().info("No transformed scan available yet")
            else:
                search = search_candidate_trajectories(
                    self.latest_edge_observation,
                    self.latest_sensor_xy,
                    robot_yaw=self.latest_sensor_yaw,
                )
                self.latest_feasible_trajectories = list(search.feasible)
                self.latest_rejected_trajectories = list(search.rejected)
                if search.feasible:
                    self.latest_trajectory = search.feasible[0]
                else:
                    self.latest_trajectory = plan_candidate_trajectory(
                        self.latest_edge_observation,
                        self.latest_sensor_xy,
                        robot_yaw=self.latest_sensor_yaw,
                    )
                from collections import Counter
                counts = Counter(trial.reason for trial in search.rejected)
                self.get_logger().info(
                    f"Evaluated {len(search.feasible) + len(search.rejected)} curves: "
                    f"feasible={len(search.feasible)}, rejected={dict(counts)}"
                )
                trial = self.latest_trajectory
                self.get_logger().info(
                    f"Feasible sampled curves: {len(self.latest_feasible_trajectories)} "
                    f"(including alternative heading-tangent control lengths)"
                )
                self.get_logger().info(
                    f"Trajectory: feasible={trial.feasible}; {trial.reason}; "
                    f"clearance={trial.min_observed_clearance}; "
                    f"curvature={trial.max_curvature}; candidate={trial.candidate}"
                )

        elif key == ord("b"):
            if self.latest_edge_observation is None:
                self.get_logger().info("No edge observation available yet")
            else:
                # No trusted physical corridor limits have been supplied.
                # This must NOT be interpreted as a confirmed blockage.
                start, end = self.selected_edge
                result = assess_passability(
                    self.latest_edge_observation,
                    robot_position=start,
                )
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
