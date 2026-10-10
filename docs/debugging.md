# RViz and debugging guide

**Documentation:** [README](../README.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Task ownership](tasks.md) · [Two-day plan](development.md) · [Teleport tests](testing.md) · [RViz & diagnostics](debugging.md)

---

## What to Monitor in RViz (and Other Debug Views)

**Start with available ROS data; only add custom markers when they help.** The provided `navigator.py` subscribes to `/scan` and `/odom` and publishes `/cmd_vel`, but **does not publish** a `nav_msgs/Path`, occupancy grid, or RViz obstacle/target markers. The template uses OpenCV `draw_scene()` for its graph and planned path. RViz therefore will not show those custom objects unless we explicitly publish corresponding visualization messages.

| Workstream | RViz display / topic | What to verify |
|---|---|---|
| Perception | **LaserScan** → `/scan` | Obstacle arcs match Gazebo; no rotated/mirrored scan; empty and blocked gaps look plausible |
| Perception | **TF** (if frames are published) | Laser/sensor pose and robot body frame are aligned with the selected fixed frame |
| Perception (optional) | **Marker / MarkerArray** → e.g. `/amaze_nav/obstacles` | Transformed and accumulated obstacle points stay fixed in the world while the robot moves |
| Perception (optional) | **Marker / MarkerArray** → e.g. `/amaze_nav/clearance` | Candidate free openings or rejected regions correspond to visible obstacle geometry |
| Control | **Odometry** → `/odom` | Translation and rotation reflect the intended segment, without overshooting the waypoint |
| Control (optional) | **Marker** → e.g. `/amaze_nav/target` | Lookahead target is ahead of the robot, shifts around obstacles, remains inside the corridor |
| Planning (optional) | **Path** → e.g. `/amaze_nav/path` | Active waypoints reroute after a blocked edge is removed; no unexpected detours |
| Planning (optional) | **MarkerArray** → e.g. `/amaze_nav/graph` | Blocked edges disappear; current/previous nodes are correct |

**RViz setup notes**

- Set **Fixed Frame** to an actually published, mutually consistent frame (often `odom` in a simple simulation); use RViz's frame/status errors to detect missing transforms. Do not assume a `map` frame exists.
- Add **LaserScan** with Topic `/scan`, then **Odometry** with Topic `/odom` where supported by the current RViz configuration. Check `ros2 topic list` and `ros2 topic echo /scan --once` if data is missing.
- `/cmd_vel` is a `geometry_msgs/Twist` command, **not a built-in spatial path**. Inspect it through `ros2 topic echo /cmd_vel`, PlotJuggler, or `rqt_plot` (e.g. linear.x and angular.z), rather than assuming RViz can plot it as a trajectory.
- For exact numeric telemetry, `rqt_plot` can compare `/odom` behavior against commands; RViz is better for geometric alignment, target markers, and obstacle positions.
- If RViz reports no transform from the LaserScan `header.frame_id` to the Fixed Frame, either provide the relevant TF or temporarily choose a compatible frame for scan inspection. A scan displayed in a sensor-fixed frame is useful for perception debugging, but it is not a world-fixed accumulated map.

**Minimal shared debug output worth adding later:** a target marker, a local obstacle marker collection, and optionally a `nav_msgs/Path` for the current A* plan. Publish all markers with valid header frames and timestamps. Diagnostic text logs should include the active edge, navigation mode, passability assessment, and key clearance values. These additions are development aids, not requirements imposed by the supplied template.

For precise edge starting poses and scenarios, see [teleport tests](testing.md).
