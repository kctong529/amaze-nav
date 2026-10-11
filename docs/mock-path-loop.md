# Controller-only full-route test

This is an extension of [`mock-path-runner.md`](mock-path-runner.md), based on Shaan's [`controller-debug.md`](controller-debug.md). **It does not depend on the pending Perception Module v1.0 PR** and does not modify the production `navigator.py` graph navigation.

## Purpose and status

The straight and single-turn routes have been exercised successfully in Gazebo. The 18-waypoint `loop` fixture is the next **unvalidated** experiment: confirm that Shaan's pure-pursuit controller can repeatedly arrive at graph nodes, reorient, negotiate corners and stop after a long route in the **obstacle-free** course world.

The word `loop` comes from the existing controller-debug guide; the path is not a closed geometrical loop. The full route contains **18 nodes, 17 adjacent edges** and ends at `(4.984, -5.696)`.

## Changes to the standalone runner

`tools/mock_path_runner.py` now accepts `--path loop`, preserving the existing `straight` and `turn` options. For the long route only, it tracks the current **edge index** explicitly instead of finding a lookahead on the entire polyline. It targets each graph waypoint in order, slows when approaching a waypoint, briefly stops after arrival and rotates in place for heading errors above 25°. A waypoint is considered reached within 0.07 m, except for the final goal which uses 0.05 m. These are **test settings**, not verified controller-performance guarantees.

This conservative, stop-and-turn behavior differs intentionally from cutting a continuous rounded corner. It is more suitable for verifying exact intermediate-node arrivals. The runner still uses the existing `GridPathFollowerNode.turtlebot_control()` function to generate translation commands; the extra heading and node-transition logic belongs to this standalone test harness only.

## Route

```python
LOOP = [
    (0.712, 0.0), (2.136, 0.0),
    (2.136, -1.424), (2.136, -2.848),
    (2.136, -4.272), (2.136, -5.696),
    (3.560, -5.696), (3.560, -4.272),
    (3.560, -2.848), (3.560, -1.424),
    (3.560, 0.0), (4.984, 0.0),
    (6.408, 0.0), (6.408, -1.424),
    (6.408, -2.848), (6.408, -4.272),
    (4.984, -4.272), (4.984, -5.696),
]
```

The direction sequence is east → south → east → north → east → south → west → south. Each edge is 1.424 m, for a total path length of **24.208 m**.

## Preparation

1. Start the **obstacle-free** Gazebo project world, not the obstacle-containing maze. Consult [`setup.md`](setup.md) for the course launch configuration.
2. Stop any other nodes publishing `/cmd_vel`, including `navigator.py` and keyboard teleoperation.
3. Teleport the robot **while the mock runner is stopped** to `(0.712, 0.0)` with yaw `0°`:

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
  "{state: {name: 'burger', pose: {position: {x: 0.712, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, reference_frame: 'world'}}"
```

4. Verify that `/odom` agrees with the world pose; the runner assumes the same coordinate system for odometry and the mock route.

## Launch

From the workspace, with ROS 2 sourced:

```bash
cd ~/shared/ros2_ws
source install/setup.bash
export PYTHONPATH="/usr/lib/python3/dist-packages:$PWD/src/vmc2_project/src${PYTHONPATH:+:$PYTHONPATH}"
```

First run with **motion disabled**:

```bash
python3 ~/shared/tools/mock_path_runner.py --path loop --gui
```

Inspect the route, initial robot pose and first target `(2.136, 0.0)`. The dry run publishes only zero velocity commands.

For the first full-route test, enable motion at a conservative speed:

```bash
python3 ~/shared/tools/mock_path_runner.py --path loop --speed 0.10 --enable-motion --gui
```

If that succeeds repeatedly, try 0.20 m/s:

```bash
python3 ~/shared/tools/mock_path_runner.py --path loop --speed 0.20 --enable-motion --gui
```

The `--speed` value is an upper cruise speed; the runner automatically slows near each intermediate node.

## Expected behavior and evidence

- OpenCV highlights the active waypoint and displays `Segment N/17`.
- Terminal logs report `WAYPOINT N/17 reached` when the robot enters each intermediate tolerance.
- The robot briefly stops at each node, turns toward the next edge, and resumes driving.
- The last node is `(4.984, -5.696)`; the runner reports `STOP: goal reached`, publishing zero velocity.
- Check that no segments are skipped; if oscillation or overshoot prevents a waypoint from being reached, stop and inspect the logs rather than increasing the tolerance blindly.
- Validate at least one full run at 0.10 m/s before testing 0.20 m/s. Record reached-node count, lateral errors, unexpected stops, collisions and total time; repeat for reliability.

## Limitations

This is **not obstacle-aware motion**. The front-sector LiDAR stop check (0.22 m) is an emergency safeguard only; it cannot verify clearance around corners, alongside the robot or while rotating. The simulator must be configured for a verified obstacle-free route. The current runner also has no global route recovery if a waypoint is missed. This test checks the controller and waypoint transitions before Perception Module v1.0 is merged.
