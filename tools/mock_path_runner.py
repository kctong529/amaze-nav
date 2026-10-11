#!/usr/bin/env python3
"""Standalone ROS 2 mock-path test for Shaan's pure-pursuit controller.

Uses the existing navigator's lookahead and turtlebot_control functions without
starting the unfinished A*/BACKTRACK state machine. NO perception integration.
Requires fresh odometry and LiDAR; uses a conservative front-sector stop only,
not general obstacle avoidance. Start in dry-run; enable motion explicitly.
"""

import argparse
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from navigator import GridPathFollowerNode, lookahead_point, yaw_from_quat

PATHS = {
    "straight": [(0.712, 0.0), (2.136, 0.0)],
    "turn": [(2.136, -4.272), (2.136, -5.696), (3.560, -5.696)],
    "bonus": [(6.408, -4.272), (6.408, -5.696)],  # Dry-run only; obstacle avoidance is not implemented.
    # Controller-only full-route fixture from docs/controller-debug.md.
    # Each consecutive pair is one adjacent graph edge (1.424 m).
    "loop": [
        (0.712, 0.0),
        (2.136, 0.0),
        (2.136, -1.424),
        (2.136, -2.848),
        (2.136, -4.272),
        (2.136, -5.696),
        (3.560, -5.696),
        (3.560, -4.272),
        (3.560, -2.848),
        (3.560, -1.424),
        (3.560, 0.0),
        (4.984, 0.0),
        (6.408, 0.0),
        (6.408, -1.424),
        (6.408, -2.848),
        (6.408, -4.272),
        (4.984, -4.272),
        (4.984, -5.696),
    ],
}


def front_clearance(scan: LaserScan, half_angle: float = math.radians(25)) -> Optional[float]:
    """Return minimum *valid* forward range, or None when no valid rays exist.

    This is a short-range emergency check, NOT swept-trajectory clearance.
    """
    valid = []
    observed_beams = 0
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        forward_angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(forward_angle) > half_angle:
            continue
        if math.isinf(r) and r > 0:
            # LaserScan +inf commonly means no return within range_max.
            observed_beams += 1
        elif math.isfinite(r) and scan.range_min <= r <= scan.range_max:
            observed_beams += 1
            valid.append(float(r))
    if observed_beams < 3:
        return None
    return min(valid) if valid else float("inf")


def path_is_sequential(path):
    """Enable explicit per-edge tracking for the full multi-turn test fixture."""
    return len(path) > 3


class MockPathRunner(Node):
    """Track a fixed world-frame polyline using navigator's tested controller."""

    def __init__(self, path_name: str, enable_motion: bool, speed: float, lookahead: float, gui: bool):
        super().__init__("mock_path_runner")
        self.path = PATHS[path_name]
        if path_name == "loop":
            # Guard against accidental skipped nodes in the debug fixture.
            for a, b in zip(self.path, self.path[1:]):
                dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
                if not ((abs(dx-1.424) < 1e-5 and dy < 1e-5)
                        or (abs(dy-1.424) < 1e-5 and dx < 1e-5)):
                    raise ValueError(f"Non-adjacent loop edge: {a} -> {b}")
        self.enable_motion = enable_motion
        self.v_max = min(speed, 0.20)
        self.v_average = speed  # Requested cruise speed; pure pursuit may reduce it for sharp curvature.
        self.w_max = 2.0
        self.lookahead = lookahead
        self.goal_tolerance = 0.05
        self.start_tolerance = 0.30
        self.stop_distance = 0.22
        self.x = self.y = self.yaw = 0.0
        self.odom_time = None
        self.scan_time = None
        self.scan: Optional[LaserScan] = None
        self.last_step_time = time.monotonic()
        self.last_log_time = 0.0
        self.finished = False
        self.gui = gui
        self.target = None
        # A long controller test should visit its nodes in sequence instead of
        # allowing a whole-route lookahead to cut across a corner.
        self.segment_index = 0
        self.waypoint_tolerance = 0.07
        self.pause_until = None
        self._start_accepted = False
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.timer = self.create_timer(0.05, self.step)
        if gui:
            import cv2
            self.cv2 = cv2
            cv2.namedWindow("Mock path runner", cv2.WINDOW_NORMAL)
        self.get_logger().info(
            f"Path={path_name}: {self.path}, motion={'ENABLED' if enable_motion else 'DRY RUN'}, "
            f"speed={speed:.2f} m/s, lookahead={lookahead:.2f} m"
        )

    def on_odom(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.odom_time = time.monotonic()

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def command(self, v=0.0, w=0.0):
        cmd = Twist()
        cmd.linear.x = float(v if self.enable_motion else 0.0)
        cmd.angular.z = float(w if self.enable_motion else 0.0)
        self.pub.publish(cmd)

    def draw(self):
        if not self.gui or self.odom_time is None:
            return
        import numpy as np
        cv2 = self.cv2
        img = np.full((660, 660, 3), 255, np.uint8)
        all_points = self.path + [(self.x, self.y)]
        xmin, xmax = min(p[0] for p in all_points) - .4, max(p[0] for p in all_points) + .4
        ymin, ymax = min(p[1] for p in all_points) - .4, max(p[1] for p in all_points) + .4
        scale = min(560/(xmax-xmin), 560/(ymax-ymin))
        def pix(p):
            return (int(50 + (p[0]-xmin)*scale), int(610 - (p[1]-ymin)*scale))
        cv2.polylines(img, [np.array([pix(p) for p in self.path], dtype=np.int32)], False, (60, 60, 60), 2)
        for p in self.path:
            cv2.circle(img, pix(p), 5, (150, 0, 150), -1)
        robot = (self.x, self.y)
        cv2.circle(img, pix(robot), 7, (0, 140, 240), -1)
        tip = (self.x+.2*math.cos(self.yaw), self.y+.2*math.sin(self.yaw))
        cv2.arrowedLine(img, pix(robot), pix(tip), (0, 140, 240), 2)
        if path_is_sequential(self.path):
            active = self.path[min(self.segment_index + 1, len(self.path) - 1)]
            cv2.circle(img, pix(active), 10, (60, 170, 30), 2)
            cv2.putText(img, f"Segment {self.segment_index+1}/{len(self.path)-1}", (15, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 0), 1)
        if self.target:
            cv2.circle(img, pix(self.target), 5, (220, 80, 0), -1)
        mode = "MOTION" if self.enable_motion else "DRY RUN"
        cv2.putText(img, mode, (15, 27), cv2.FONT_HERSHEY_SIMPLEX, .8, (0,0,0), 2)
        cv2.imshow("Mock path runner", img)
        cv2.waitKey(1)

    def step(self):
        now = time.monotonic()
        dt = min(.2, max(.001, now - self.last_step_time))
        self.last_step_time = now
        reason = None
        if self.finished:
            reason = "finished"
        elif self.odom_time is None or now - self.odom_time > .5:
            reason = "missing or stale odometry"
        elif self.scan_time is None or now - self.scan_time > .5:
            reason = "missing or stale scan"
        elif (self.segment_index == len(self.path) - 2
              and math.dist((self.x, self.y), self.path[-1]) <= self.goal_tolerance):
            self.finished = True
            reason = "goal reached"
        elif (not self._start_accepted
              and math.dist((self.x, self.y), self.path[0]) > self.start_tolerance):
            reason = "robot not at mock-path start; teleport first"
        else:
            clearance = front_clearance(self.scan)
            if clearance is None:
                reason = "no valid forward scan rays"
            elif clearance < self.stop_distance:
                reason = f"front obstacle at {clearance:.2f} m"
        if reason:
            self.command()
            if now - self.last_log_time >= 1.0:
                self.get_logger().info("STOP: " + reason)
                self.last_log_time = now
            self.draw()
            return
        robot = (self.x, self.y)
        self._start_accepted = True
        if self.pause_until is not None and now < self.pause_until:
            self.command()
            self.draw()
            return
        self.pause_until = None
        if path_is_sequential(self.path):
            waypoint = self.path[self.segment_index + 1]
            distance_to_waypoint = math.dist(robot, waypoint)
            # Preserve sequential node arrivals: do not jump to the next segment
            # merely because the global polyline's nearest projection changed.
            if distance_to_waypoint <= (self.goal_tolerance if self.segment_index == len(self.path)-2
                                        else self.waypoint_tolerance):
                if self.segment_index < len(self.path)-2:
                    self.segment_index += 1
                    self.pause_until = now + 0.25
                    self.target = self.path[self.segment_index + 1]
                    self.command()
                    self.get_logger().info(
                        f"WAYPOINT {self.segment_index}/{len(self.path)-1} reached; "
                        f"next={self.target} (pause, then reorient)")
                    self.draw()
                    return
            # The current segment endpoint is the target, so the robot reaches
            # each graph node before being allowed to steer into the next edge.
            # At 0.20 m/s, reduce speed near the node to prevent overshoot.
            target = waypoint
        else:
            distance_to_waypoint = math.dist(robot, self.path[-1])
            target = lookahead_point(self.path, robot, self.lookahead)
        self.target = target
        # Reuse exactly the navigator class method; the runner supplies its parameters.
        v, w = GridPathFollowerNode.turtlebot_control(self, robot, self.yaw, target, dt, self.lookahead)
        # Rotate in place when the next waypoint lies substantially off-heading.
        heading_error = math.atan2(math.sin(math.atan2(target[1]-self.y, target[0]-self.x)-self.yaw),
                                   math.cos(math.atan2(target[1]-self.y, target[0]-self.x)-self.yaw))
        if abs(heading_error) > math.radians(25 if path_is_sequential(self.path) else 70):
            v = 0.0
            w = max(-self.w_max, min(self.w_max, 1.5*heading_error))
        if path_is_sequential(self.path):
            # Proportional approach near the waypoint; never exceed user speed.
            # When the target is sideways/behind, rotation is handled above.
            v = min(v, max(0.02, 0.8 * distance_to_waypoint))
        self.command(v, w)
        if now - self.last_log_time >= .5:
            self.get_logger().info(
                f"pose=({self.x:.3f},{self.y:.3f},{math.degrees(self.yaw):.1f}deg) "
                f"lookahead=({target[0]:.3f},{target[1]:.3f}) "
                f"v={v:.3f} w={w:.3f} dist_goal={math.dist(robot,self.path[-1]):.3f}"
            )
            self.last_log_time = now
        self.draw()

    def shutdown(self):
        self.command(0.0, 0.0)
        if self.gui:
            self.cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path", choices=PATHS, default="straight")
    parser.add_argument("--enable-motion", action="store_true", help="Opt in to publishing nonzero cmd_vel")
    parser.add_argument("--speed", type=float, default=0.20,
                        help="Requested cruise speed in m/s (default: 0.20; use 0.05 for initial tests)")
    parser.add_argument("--lookahead", type=float, default=0.25,
                        help="Pure-pursuit lookahead in meters (default: 0.25)")
    parser.add_argument("--gui", action="store_true", help="Show OpenCV visualization")
    args, ros_args = parser.parse_known_args()
    if not (0 < args.speed <= 0.2 and 0.05 <= args.lookahead <= 0.6):
        parser.error("Require 0 < speed <= 0.20, 0.05 <= lookahead <= 0.60")
    rclpy.init(args=ros_args)
    node = MockPathRunner(args.path, args.enable_motion, args.speed, args.lookahead, args.gui)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
