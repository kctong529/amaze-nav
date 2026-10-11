# Controller-only debug guide (Shaan)

**Documentation:** [README](../README.md) · [Setup](setup.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Task ownership](tasks.md) · [Two-day plan](development.md) · [Ground-truth survey](edge-ground-truth.md) · [Controller debug](controller-debug.md) · [Navigator tests](testing.md) · [RViz & diagnostics](debugging.md)

> **Current controller-only test setup (11 October 2026):** Use [`mock-path-runner.md`](mock-path-runner.md) and [`mock-path-loop.md`](mock-path-loop.md) for the standalone runner. Forward speed is capped at **0.22 m/s** on all segments. The experimental 0.50 m/s adaptive policy was reverted after excessive drift. This guide describes an *alternative* debug switch inside `navigator.py`; do not enable both runners simultaneously.

## Goal

Test the **pure-pursuit baseline** of `turtlebot_control()` independently of A* route selection, LiDAR passability, corridor-centering PID, and obstacle avoidance. Use the **obstacle-free** Gazebo world. The test injects a known path into `self.full_path` and temporarily bypasses perception. **Do not submit with the test mode enabled.**

## Test routes

| Mode | Path | Initial yaw | Tests |
| --- | --- | --- | --- |
| `straight` | `(0.712, 0.0)` → `(2.136, 0.0)` | `0°` (east) | Straight tracking, velocity limits, stopping |
| `turn` | `(2.136, -4.272)` → `(2.136, -5.696)` → `(3.560, -5.696)` | `-90°` (south) | Intermediate waypoint, 90° left turn, final stopping |
| `loop` | Long, multi-turn route drawn on the course map (full waypoints below) | `0°` (east) | End-to-end path tracking across multiple nodes and corners |

Each grid edge is 1.424 m. For `loop`, **every pair of consecutive waypoints must be adjacent in `GridGraph`**; the complete route has 18 waypoints and 17 graph edges. Keeping intermediate nodes also lets the template detect waypoint arrivals. The route is intended to be clear in the obstacle-free world, but its safety depends on tracking accuracy at corners. A supplementary heading-alignment test can run `straight` with the robot initially facing north (`+90°`).

## 1. Add an explicit, reversible debug switch

The supplied `navigator.py` already imports `sys` but not `os`. Add near the other imports:

```python
import os
```

Directly after `super().__init__("grid_path_follower")` in `GridPathFollowerNode.__init__`, add:

```python
# Test-only switch; unset for normal navigation.
self.controller_test = os.environ.get("AMAZE_NAV_CONTROLLER_TEST", "").strip().lower()
if self.controller_test not in ("", "straight", "turn", "loop"):
    raise ValueError("AMAZE_NAV_CONTROLLER_TEST must be straight, turn, loop, or unset")

self.debug_paths = {
    "straight": [(0.712, 0.0), (2.136, 0.0)],
    "turn": [(2.136, -4.272), (2.136, -5.696), (3.560, -5.696)],
    # Longer route traced from the red line on the maze diagram.
    # Every neighboring graph node is retained as a waypoint.
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
```

After the existing `self.graph = GridGraph.from_node_graph(self.node_graph)` line, validate every mock route against the graph:

```python
# Reject shortcuts or other invalid test edges.
for name, path in self.debug_paths.items():
    for a, b in zip(path, path[1:]):
        if not self.graph.has_edge(a, b):
            raise ValueError(f"Invalid edge in {name}: {a} -> {b}")
```

This check must run **after** the grid graph has been constructed, because `has_edge()` needs `self.graph`. In particular, long straight stretches must retain all intermediate nodes: skipping from `(2.136, 0.0)` directly to `(2.136, -5.696)` is not a valid graph edge.

In the existing **Start and goal positions** section, keep the normal assignments and override only for test mode:

```python
self.start_pos = (0.0, 0.0)
self.goal_pos = (7.120, -5.696)

if self.controller_test:
    test_path = self.debug_paths[self.controller_test]
    self.start_pos = test_path[0]
    self.goal_pos = test_path[-1]
```

At the end of `__init__`, replace **only** the original `self.path_plan(self.start_pos)` call with:

```python
if self.controller_test:
    self.full_path = list(self.debug_paths[self.controller_test])
    self.get_logger().info(
        f"CONTROLLER TEST [{self.controller_test}]: {self.full_path}"
    )
else:
    self.path_plan(self.start_pos)

self.get_logger().info(f"Planned {len(self.full_path)} waypoints.")
```

This is preferable to changing `astar_path()` or deleting any of the provided planning code: the production branch still behaves as originally intended.

## 2. Bypass perception only in controller-test mode

In `is_passable()`, add a test-mode return **before** the production implementation:

```python
def is_passable(self, seg_dir_yaw: float) -> bool:
    if self.controller_test:
        return True
    # TODO: normal LiDAR passability implementation
    return False
```

In `obstacle_avoided_target()`, do the same for lookahead selection:

```python
def obstacle_avoided_target(self, robot_pos: Coord, seg_dir_yaw: float) -> Coord:
    if self.controller_test:
        return lookahead_point(self.full_path, robot_pos, self.lookahead_dist)
    # TODO: normal obstacle-aware target implementation
    raise NotImplementedError("Obstacle avoidance not implemented")
```

These bypasses are **only for the obstacle-free controller tests**. They are not collision avoidance and must not be enabled for the obstacle-containing world.

**LiDAR centering must be disabled in all three isolated debug routes** (set its confidence/weight to zero or bypass the centering helper only when `self.controller_test` is set). This ensures the controller test passes without requiring scan geometry. After these tests pass, evaluate the centering correction separately using real `/scan` data and the normal navigator; do not set the centering weight permanently to zero.

## 3. Implement/configure the controller

For the current controller, confirm that `self.lookahead_dist`, `self.v_max`, `self.w_max`, and any PID gains are configured before testing; never leave `...` placeholders. The current translational speed cap is **0.22 m/s**. Do not restore the abandoned 0.50 m/s fast-straight policy.

Implement `turtlebot_control(robot_pos, robot_h, target, dt, Ld)` so it returns `(v_cmd, w_cmd)` and respects the configured speed limits. The template publishes that pair to `/cmd_vel` at 20 Hz. Keep speeds conservative for the first run.

Recommended controller behavior at corners: reduce or eliminate forward speed while the heading error is large; rotate to the new segment direction before advancing. This is a suggested strategy, not an implemented feature of the template.

## 4. Start the obstacle-free simulation

Follow [Environment setup](setup.md) to install the models and ROS 2 package already included in this repository, configure model paths, and build/source the workspace. Configure the course launch file to use `project_track_no_obs` rather than `project_track` (the exact launch option depends on your local course package).

```bash
ros2 launch vmc2_project project.launch.py
```

**Ensure only one node is publishing control commands**. If the launch file automatically starts `navigator.py`, stop or disable that process until after teleporting. Do not run keyboard teleoperation simultaneously with the controller test.

## 5. Teleport *before* starting the controller

Stop the navigator process before teleporting, per the assignment. After teleporting, launch the navigator with the matching debug mode.

### Test A: straight, heading east (`0°`)

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
  "{state: {name: 'burger', pose: {position: {x: 0.712, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, reference_frame: 'world'}}"
```

### Test B: south, then turn east (`-90°`)

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
  "{state: {name: 'burger', pose: {position: {x: 2.136, y: -4.272, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.707107, w: 0.707107}}, reference_frame: 'world'}}"
```

### Test C: long multi-turn route (`loop`)

This is the red route from the shared maze screenshot, sampled at every adjacent grid node. Start at `(0.712, 0.0)`, facing east; finish at `(4.984, -5.696)`. The route goes **east → south → east → north → east → south → west → south**.

Use the same teleport command as **Test A**, since both start at `(0.712, 0.0)` with yaw `0°`. Before starting, select the **obstacle-free** Gazebo world. The red line indicates an intended clear route; it is not a guarantee that aggressive corner cutting will be collision-free in the simulated track.

For this test, validate each intermediate waypoint in the OpenCV path view. The template should shorten `self.full_path` as it reaches nodes. If the robot cuts corners or oscillates, diagnose the controller and path-segment selection before involving LiDAR.

### Optional heading-alignment variant: straight route, initially north (`+90°`)

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
  "{state: {name: 'burger', pose: {position: {x: 0.712, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.707107, w: 0.707107}}, reference_frame: 'world'}}"
```

## 6. Run the navigator in the chosen test mode

Use the same command you normally use to execute the course's `navigator.py`; prefix it with the environment variable. For example, if the package exposes the script as a ROS executable:

```bash
AMAZE_NAV_CONTROLLER_TEST=straight ros2 run vmc2_project navigator.py
# OR
AMAZE_NAV_CONTROLLER_TEST=turn ros2 run vmc2_project navigator.py
# OR
AMAZE_NAV_CONTROLLER_TEST=loop ros2 run vmc2_project navigator.py
```

The exact `ros2 run` executable name depends on how your course package installs the script. Alternatively, execute `navigator.py` directly with `python3` from the sourced ROS environment, if that is how your workspace is configured. Always teleport first.

**To restore normal mode**, launch without `AMAZE_NAV_CONTROLLER_TEST` (or unset it):

```bash
unset AMAZE_NAV_CONTROLLER_TEST
```

## 7. Monitor and debug

- **OpenCV `Grid A* Path Follower` window:** Verify the fixed path, robot position, and lookahead target. The provided `draw_scene()` already displays these.
- **RViz `/odom` (Odometry):** Verify the robot starts at the expected position and orientation and reaches each waypoint. Set Fixed Frame to an available frame such as `odom`.
- **`/cmd_vel`:** Inspect commanded linear and angular velocities. Use `ros2 topic echo /cmd_vel` or `rqt_plot /cmd_vel/linear/x /cmd_vel/angular/z` (if installed).
- **`/scan`:** Optional in the obstacle-free world; helps verify that the selected path is not approaching walls.
- **Optional proposed markers:** A `/amaze_nav/target` marker or `/amaze_nav/path` Path message would be useful in RViz but **are not published by the provided template**.

Look for: large initial heading error, lateral drift, overshoot at nodes, oscillation near the goal, corner cutting, speed saturation, or failure to stop.

## 8. Definition of done

**Straight route:**

- [ ] Follows `(0.712, 0.0)` → `(2.136, 0.0)` without contacting walls.
- [ ] Reaches the end within the configured goal tolerance (`0.05 m`).
- [ ] Stops and reports `DONE`.
- [ ] Respects configured linear/angular speed limits.
- [ ] Repeats successfully three times.

**One-turn route:**

- [ ] Travels south from `(2.136, -4.272)` to `(2.136, -5.696)`.
- [ ] Recognizes the intermediate waypoint (template `node_tol = 0.05 m`).
- [ ] Turns east toward `(3.560, -5.696)` without cutting into walls.
- [ ] Reaches the final goal and stops.
- [ ] Repeats successfully three times.

**Long multi-turn route (`loop`):**

- [ ] Starts at `(0.712, 0.0)` with heading east, then follows the full 18-waypoint route.
- [ ] Successfully makes all direction changes (east → south → east → north → east → south → west → south).
- [ ] Reaches each intermediate graph node before the route advances, without substantial corner cutting.
- [ ] Does not contact walls or obstacles in the obstacle-free world.
- [ ] Finishes at `(4.984, -5.696)` within goal tolerance and reports `DONE`.
- [ ] Completes three consecutive runs with consistent behavior.

**Optional alignment test:** The robot starts facing north but turns toward the eastbound path before significantly advancing.

## 9. Template pitfalls

1. **Waypoint trimming:** `control_step()` removes completed path segments after reaching a waypoint. `self.full_path` will get shorter during Test B; this is expected.
2. **Final goal vs. path length:** The template returns early when `len(self.full_path) < 2`, *before* evaluating the `goal_tol` check. If the final waypoint gets trimmed while the robot is not yet within `goal_tol`, the robot may appear stuck rather than enter `DONE`. Test and, if needed, handle this as a separate final-goal edge case.
3. **Nearest segment at corners:** The template chooses the nearest segment of the remaining polyline, which can switch early near a corner. Watch for unexpected target jumps and corner cutting.
4. **Avoid competing publishers:** Only the active test controller should publish `/cmd_vel`.
5. **Teleport and odometry:** Confirm `/odom` matches the teleported pose. If not, use the observed odometry frame carefully when interpreting world-coordinate waypoints.
6. **No final hard-coding:** Remove or disable test mode before submitting. The final navigator must use normal path planning and perception.

## 10. Quick handoff

When the short tests and (ideally) the long route pass, share the chosen PID/lookahead/speed parameters, a short video or logs, and any observed failures with the perception and planning teammates. They should then be able to call `turtlebot_control()` with obstacle-aware targets without changing its interface. The proposed LiDAR centering layer is integrated and validated afterward, not simulated by these three paths.

## Why the longer route is useful

The two short tests establish the basics, while `loop` tests what Shaan will actually need for integration: a controller that survives repeated graph-node transitions and changes of heading without depending on the unfinished perception or replanning functions. In particular, a controller that works for a single edge can still fail due to a lookahead point that jumps across a corner or `nearest_point_on_polyline()` selecting an unintended segment.

Do not use `loop` in the final submitted maze navigator: it is a fixed **controller test fixture**, not a learned map or valid production pathfinding strategy. Run it only in the obstacle-free world.
