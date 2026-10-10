#!/usr/bin/env python3

import numpy as np
import rclpy

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
