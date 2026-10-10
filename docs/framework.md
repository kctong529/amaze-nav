# Understanding the provided navigation framework

**Documentation:** [README](../README.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Task ownership](tasks.md) · [Two-day plan](development.md) · [Teleport tests](testing.md) · [RViz & diagnostics](debugging.md)

---

The course supplies a partially implemented `navigator.py`. **A* graph search already exists**; the main work is connecting the planner, LiDAR perception, obstacle-aware targeting, control, and state transitions. The assignment permits adding helpers but cautions against unnecessarily modifying or removing provided code.

## Grid and path planning

| Function or class | Responsibility | Status |
|---|---|---|
| `GridGraph` | Node coordinates, 4-neighbor connectivity, and graph representation | Provided |
| `GridGraph.remove_edge()` | Removes an impassable edge | Provided |
| `astar_path()` | A* search over the grid | Implemented |
| `nearest_graph_coord()` | Locates closest grid node | Provided |
| `plan_from()` | Assembles graph path into usable waypoints | **TODO** |
| `path_plan()` | Stores newly planned route in `self.full_path` | Provided |
| `nearest_point_on_polyline()` | Finds current nearest path segment | Provided |
| `lookahead_point()` | Finds lookahead point along path | Provided |

## Perception and local avoidance

| Function | Responsibility | Status |
|---|---|---|
| `scan_cb()` | Saves latest LiDAR `/scan` message | Provided |
| `odom_cb()` | Updates position and yaw from `/odom` | Provided |
| `is_passable(seg_dir_yaw)` | Determine whether current edge should remain in use | **TODO** |
| `obstacle_avoided_target(robot_pos, seg_dir_yaw)` | Choose an obstacle-aware target in world coordinates | **TODO** |

`seg_dir_yaw` is the **world/odometry-frame segment heading**, not a LiDAR-relative angle. `is_passable()` is called repeatedly while in `FOLLOW`, so it can reconsider its decision as new readings arrive. Avoid conflating an obstacle on the centerline with a blocked edge. Perception and target selection should ideally share a geometric representation; see [methodology](methodology.md).

## Control and utilities

| Function or class | Responsibility | Status |
|---|---|---|
| `PID` | Optional PID controller with anti-windup | Provided |
| `turtlebot_control(robot_pos, robot_h, target, dt, Ld)` | Return `(v_cmd, w_cmd)` in m/s and rad/s | **TODO** |
| `publish_cmd()` / `publish_stop()` | Publish `/cmd_vel`, or zero velocity | Provided |
| `angle_wrap()` | Normalize angle into ±π | Provided |
| `yaw_from_quat()` | Extract heading from quaternion | Provided |

Configure `lookahead_dist`, `v_max`, `w_max`, and optionally PID gains in the constructor. The suggested controller blends pure pursuit with PID, but PID is optional.

## Main state machine

`control_step()` runs on a 20 Hz timer; it reads pose, checks goal and waypoint progress, and dispatches by mode. Some transition logic is still incomplete.

| Mode | Intended behavior | Main TODO |
|---|---|---|
| `FOLLOW` | Check passage; select target; command motion | Save blocked edge and initiate recovery when needed |
| `BACKTRACK` | Return to last node | Complete previous-node bookkeeping and arrival transition |
| `REPLAN` | Remove blocked edge; plan alternate route | Wire graph modification and new path into follow mode |
| `DONE` | Stop at goal | Largely provided |

```text
control_step()
  pose + waypoint checks
       ├── REPLAN: remove edge -> A* -> FOLLOW
       ├── BACKTRACK: return to previous node -> REPLAN
       └── FOLLOW: is_passable?
                       ├── no -> BACKTRACK
                       └── yes -> avoidance target -> control -> /cmd_vel
```

## ROS interfaces and visualization

- `/scan`: `sensor_msgs/LaserScan`, subscribed.
- `/odom`: `nav_msgs/Odometry`, subscribed.
- `/cmd_vel`: `geometry_msgs/Twist`, published.
- `draw_scene()`: existing **OpenCV** graph/path/robot/target visualization (not an RViz publisher).
- `_set_mode()`: switches operating modes and applies a short dwell.
- `main()`: initializes and spins the ROS node.

The template does **not** publish a `nav_msgs/Path`, occupancy grid, or obstacle/target RViz markers by default. See [RViz diagnostics](debugging.md) before assuming these topics exist.

## What to implement

1. Planning integration: `plan_from()`, remove confirmed blocked edges, and replan.
2. Perception: `is_passable()` and optional scan-processing helpers.
3. Local avoidance: `obstacle_avoided_target()`.
4. Motion control: `turtlebot_control()` and gains/limits.
5. Recovery state transitions and reliability testing.

See [suggested ownership](tasks.md) and [two-day implementation plan](development.md).
