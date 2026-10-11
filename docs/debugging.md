# RViz and debugging guide

**Documentation:** [README](../README.md) · [Setup](setup.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Perception strategy](perception-strategy.md) · [Perception API](perception-api.md) · [Task ownership](tasks.md) · [Development plan](development.md) · [Ground-truth survey](edge-ground-truth.md) · [Controller debug](controller-debug.md) · [Navigator tests](testing.md) · [Offline tests](navigation-offline-tests.md) · **RViz & debugging**

---

Start with [Environment setup](setup.md) to confirm that Gazebo finds the included models and `/scan` and `/odom` are publishing.

## What to Monitor in RViz (and Other Debug Views)

For the proposed LiDAR centering controller, log or visualize: left and right **segment-perpendicular** clearance, signed lateral centering error, confidence/validity, `ω_PP`, weighted `ω_c`, combined `/cmd_vel` angular command, and whether centering is suppressed near a node/turn. These are **proposed diagnostics** and require new publishers/logs; RViz does not show these values automatically. Use `rqt_plot` for numeric time series if published.

**Start with available ROS data; only add custom markers when they help.** The provided `navigator.py` subscribes to `/scan` and `/odom` and publishes `/cmd_vel`, but **does not publish** a `nav_msgs/Path`, occupancy grid, or RViz obstacle/target markers. The template uses OpenCV `draw_scene()` for its graph and planned path. RViz therefore will not show those custom objects unless we explicitly publish corresponding visualization messages.

| Workstream | RViz display / topic | What to verify |
|---|---|---|
| Perception | **LaserScan** → `/scan` | Obstacle arcs match Gazebo; no rotated/mirrored scan; empty and blocked gaps look plausible |
| Perception | **TF** (if frames are published) | Laser/sensor pose and robot body frame are aligned with the selected fixed frame |
| Perception (optional) | **Marker / MarkerArray** → e.g. `/amaze_nav/obstacles` | Transformed and accumulated obstacle points stay fixed in the world while the robot moves |
| Perception (optional) | **Marker / MarkerArray** → e.g. `/amaze_nav/clearance` | Stage 1: measured cross-sections and rejected openings; Stage 2: shifted lookahead, predicted short pure-pursuit arc, and closest observed obstacle distance; verify any edge-blocked verdict is supported by an unavoidable bottleneck or failed route feasibility |
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

**Two-stage telemetry to log:** edge ID, `W_min` per candidate opening, which boundaries support it, whether an inadequate bottleneck is unavoidable, predicted short-motion minimum Euclidean clearance, unknown-space warning, and final clear/unknown/blocked assessment. These are proposal-specific diagnostics, not existing ROS topics.

**Minimal shared debug output worth adding later:** a target marker, a local obstacle marker collection and a short predicted-motion arc, and optionally a `nav_msgs/Path` for the current A* plan. Publish all markers with valid header frames and timestamps. Diagnostic text logs should include the active edge, navigation mode, passability assessment, and key clearance values. These additions are development aids, not requirements imposed by the supplied template.

For precise edge starting poses and scenarios, see [teleport tests](testing.md).

For manual edge exploration, rosbag recording, and ground-truth evidence, see [the 40-edge survey](edge-ground-truth.md). For RViz acceptance during autonomous navigation, see [navigator tests](testing.md).

For controller-only troubleshooting (debug-mode commands, exact teleports, the 18-waypoint loop, and waypoint-arrival pitfalls), see [controller-debug.md](controller-debug.md).

**Stage 2 RViz check:** Draw the original pure-pursuit target, shifted obstacle-aware target, and **predicted short controller arc** in distinct markers. Overlay the effective robot radius (or swept footprint), mark its closest approach to a LiDAR surface, and show known-versus-unknown space where possible. Do **not** interpret a point target or a straight line to it as a validated trajectory. Visual markers are proposed additions, not template publishers.


## Standalone LiDAR trajectory explorer (implemented)

`tools/run_lidar_explorer.sh` runs the live OpenCV sensor, fixed-odom, and directed-edge views. It imports the installed/local `perception.py`; it does **not** command `/cmd_vel`. In the edge view, blue denotes the directed edge and robot; orange denotes observed obstacle returns; green marks the selected current-heading translation, brown marks a **separate post-rotation translation**; yellow marks `STOP / ROTATE`, with a heading arrow. Light blue curves are alternative feasible samples; red/purple/grey are selected rejected samples in debug mode. Do not treat joined-looking lines as one drivable curve.

Press `t` to recompute and log the plan; `v` cycles best/feasible/debug views, `+` and `-` adjust candidate count, `m`/`n` change the extra safety margin, `j`/`k` change curvature, `p` logs the profile, and `b` runs conservative edge assessment. The plan label refers to a search **strategy**, not the robot's real-time controller state. See [Perception API](perception-api.md).
