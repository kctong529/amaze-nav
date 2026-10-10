#!/usr/bin/env python3

import numpy as np
import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def scan_to_points(scan: LaserScan) -> np.ndarray:
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
    """
    Display LiDAR points in the robot's local frame.

    size: image dimensions in pixels
    scale: pixels per metre
    """
    image = np.full((size, size, 3), 255, dtype=np.uint8)

    cx = size // 2
    cy = size // 2

    # Draw a reference grid every 0.5 m
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

    # LiDAR points: +x points upward, +y points left.
    for x, y in points:
        px = int(cx - y * scale)
        py = int(cy - x * scale)

        if 0 <= px < size and 0 <= py < size:
            cv2.circle(
                image, (px, py), 2, (0, 140, 255), -1
            )

    # Robot position
    cv2.circle(image, (cx, cy), 7, (255, 100, 0), -1)

    # Forward direction
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

    cv2.imshow("LiDAR Explorer - Sensor Frame", image)
    cv2.waitKey(1)


class LidarExplorer(Node):
    def __init__(self):
        super().__init__("lidar_explorer")

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )

        self.last_log_time = self.get_clock().now()
    
    def scan_callback(self, scan):
        points = scan_to_points(scan)

        draw_lidar_points(points)

        elapsed = (
            self.get_clock().now() - self.last_log_time
        ).nanoseconds * 1e-9

        if elapsed >= 1.0:
            self.get_logger().info(
                f"{len(points)} valid LiDAR points, "
                f"frame={scan.header.frame_id}"
            )
            self.last_log_time = self.get_clock().now()


def main():
    rclpy.init()
    node = LidarExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
