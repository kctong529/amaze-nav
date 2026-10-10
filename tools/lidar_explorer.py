
#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from perception_geometry import (
    transform_points_2d,
    points_to_edge_frame,
    extract_slice,
)

SENSOR_WINDOW = "LiDAR Explorer - Sensor Frame"
ODOM_WINDOW = "LiDAR Explorer - Odom Frame"
EDGE_WINDOW = "LiDAR Explorer - Edge Frame"

TEST_EDGE_START = (0.712, -2.848)
TEST_EDGE_END = (2.136, -2.848)

EDGE_LENGTH = 1.424

SLICE_THICKNESS = 0.10
SLICE_STEP = 0.10


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


def draw_edge_frame(
    edge_points,
    slice_points,
    slice_s,
    robot_st=None,
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
            int(cx + t * scale),
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

    cv2.putText(
        image,
        "Use [ and ] to move the slice",
        (15, size - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (30, 30, 30),
        1,
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

        # The latest scan awaiting its timestamp-matched TF.
        self.pending_scan = None

        # Diagnostic counters.
        self.received_scans = 0
        self.transformed_scans = 0
        self.dropped_scans = 0

        # Cross-section parameters.
        self.slice_thickness = SLICE_THICKNESS
        self.slice_step = SLICE_STEP
        self.slice_s = EDGE_LENGTH / 2

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

        # Convert LiDAR points to the selected edge's coordinates.
        edge_points = points_to_edge_frame(
            self.latest_world_points,
            TEST_EDGE_START,
            TEST_EDGE_END,
        )

        # Restrict the visualization to points near the edge.
        near_edge = (
            (edge_points[:, 0] >= -0.2)
            & (edge_points[:, 0] <= EDGE_LENGTH + 0.2)
            & (np.abs(edge_points[:, 1]) <= 1.0)
        )

        filtered_points = edge_points[near_edge]

        # Select points belonging to the current cross-section.
        slice_points = extract_slice(
            filtered_points,
            self.slice_s,
            SLICE_THICKNESS,
        )

        # Convert the LiDAR sensor's position into edge coordinates.
        sensor_xy = (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

        sensor_st = points_to_edge_frame(
            np.asarray([sensor_xy]),
            TEST_EDGE_START,
            TEST_EDGE_END,
        )[0]

        # Draw the edge-aligned visualization.
        draw_edge_frame(
            filtered_points,
            slice_points,
            self.slice_s,
            robot_st=sensor_st,
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
                EDGE_LENGTH,
                self.slice_s + SLICE_STEP,
            )

        elif key == ord("["):
            self.slice_s = max(
                0.0,
                self.slice_s - SLICE_STEP,
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
