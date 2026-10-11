# Mock-path controller test

The [`tools/mock_path_runner.py`](../tools/mock_path_runner.py) script validates the TurtleBot3 pure-pursuit controller on predefined paths in Gazebo Classic. It is a **controller debug tool**, not an autonomous navigation mode: it bypasses A*, graph-edge passability, backtracking, and Perception Module v1.0.

**Validation status (11 October 2026):** The straight and 90°-turn paths have been tested successfully at **0.05 m/s**. The runner's default requested speed is **0.20 m/s**; that speed has not yet been verified in these tests.

## Architecture and scope

```text
Fixed path (world-frame waypoints)
        │
        ▼
lookahead_point(path, odometry_position, lookahead)
        │
        ▼
GridPathFollowerNode.turtlebot_control(...)
        │
        ▼
/cmd_vel → TurtleBot3 → /odom
        ▲
        └────── /scan (forward-sector stop check)
```

The runner imports `lookahead_point()`, `yaw_from_quat()`, and `GridPathFollowerNode.turtlebot_control()` from [`navigator.py`](../ros2_ws/src/vmc2_project/src/navigator.py), **without instantiating the main navigator node**. It operates at 20 Hz. A basic forward-facing scan check can stop the robot, but **does not provide obstacle avoidance or swept-path collision checking**.

## Prerequisites

- ROS 2 Humble, Gazebo Classic, and the TurtleBot3 `burger` model.
- The simulation publishes `/odom` and `/scan`, accepts `/cmd_vel`, and provides `/set_entity_state` for teleportation.
- Build the workspace and source its environment before running.
- Stop `navigator.py`, teleoperation, or other nodes that would publish competing `/cmd_vel` commands.
- Ensure the robot's `/odom` coordinates agree with the hardcoded map/world coordinates. Teleporting in Gazebo does not necessarily reset the odometry origin.

The examples below assume a repository checkout under `~/shared`, with the ROS workspace at `ros2_ws/` and the debug script at `tools/mock_path_runner.py`.

```bash
cd ~/shared/ros2_ws
source install/setup.bash
export PYTHONPATH="/usr/lib/python3/dist-packages:$PWD/src/vmc2_project/src${PYTHONPATH:+:$PYTHONPATH}"
python3 ../tools/mock_path_runner.py --path straight --gui
```

`/usr/lib/python3/dist-packages` is placed first to avoid the NumPy 2.x / system OpenCV binary incompatibility seen in the course's ROS environment. Without `--enable-motion`, the runner logs calculated commands but publishes **zero** velocities.

## Command-line options

| Flag | Default | Meaning |
|---|---|---|
| `--path {straight,turn,bonus}` | `straight` | Select a predefined path |
| `--speed FLOAT` | `0.20` | Requested cruise speed (m/s), in `(0, 0.20]` |
| `--lookahead FLOAT` | `0.25` | Pure-pursuit lookahead (m), in `[0.05, 0.60]` |
| `--enable-motion` | Disabled | Explicitly permit nonzero `/cmd_vel` |
| `--gui` | Disabled | Show the OpenCV path, robot pose, and target |

Pure pursuit may reduce speed to respect the angular velocity limit. The runner does not currently limit acceleration or deceleration.

## Test paths

| Path | World-frame waypoints (m) | Test objective |
|---|---|---|
| `straight` | `(0.712, 0.0)` → `(2.136, 0.0)` | Eastbound straight-line tracking |
| `turn` | `(2.136, -4.272)` → `(2.136, -5.696)` → `(3.560, -5.696)` | Southbound travel followed by a 90° left turn |
| `bonus` | `(6.408, -4.272)` → `(6.408, -5.696)` | **Diagnostics only**; do not enable motion without obstacle-aware planning |

Verify that the first two routes are unobstructed in the *current* maze before enabling motion.

### Reset the starting pose

Use `/set_entity_state` with the model name `burger`. For the straight path, face east (yaw 0):

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
"{state: {name: 'burger', pose: {position: {x: 0.712, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, reference_frame: 'world'}}"
```

For the turn path, start at `(2.136, -4.272)` facing south (yaw −90°):

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
"{state: {name: 'burger', pose: {position: {x: 2.136, y: -4.272, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.70710678, w: 0.70710678}}, reference_frame: 'world'}}"
```

After teleportation, inspect `/odom` or the runner's logged pose. Do not enable motion until the measured position and heading match the intended start.

## Test procedure

1. **Dry run:** Launch `--path straight --gui`; check pose, target direction, and calculated `v`/`w`. Confirm that no other node is sending `/cmd_vel`.
2. **Reproduce the validated straight test:** Run with `--path straight --speed 0.05 --enable-motion --gui`. Expect positive forward speed, near-zero turning on the centerline, and a complete stop near the endpoint.
3. **Test increased speed:** Reset the pose, then repeat with `--speed 0.20`. Compare tracking error, stopping distance, and overshoot; **do not record this as validated until observed**.
4. **Test the corner:** Reset to the turn start; run `--path turn` at `0.05`, then `0.10`, then `0.20 m/s` only if earlier tests show adequate clearance and tracking. Pure pursuit may cut inside the corner.
5. **Record observations:** Note target, odometry pose and heading, `v`/`w`, final position error, unexpected stops, and any visible oscillation or corner cutting.

Example motion command (from `~/shared/ros2_ws` after the environment setup above):

```bash
python3 ../tools/mock_path_runner.py --path straight --speed 0.05 --enable-motion --gui
```

Repeat with `--path turn` for the turn test. The 1.424 m straight segment takes approximately 28.5 seconds at constant `0.05 m/s`, or 7.1 seconds at `0.20 m/s`, excluding start/stop and steering corrections.

## Runtime safeguards

The runner checks:

- Odometry and LiDAR freshness: maximum age **0.5 s** each, using monotonic time.
- Distance to final goal: stops within **0.05 m**.
- Initial pose: must be within **0.30 m** of the first waypoint.
- Forward scan coverage: at least three valid rays within **±25°**; positive infinite ranges count as no return.
- Closest measured forward obstacle: stops when closer than **0.22 m**.
- Heading error: suspends translation and rotates when error exceeds **70°**.

The `0.22 m` threshold is **not a speed-aware stopping-distance guarantee**. It does not protect against side obstacles or corner cutting. For unexpected behavior, stop the runner and halt the robot. The runner sends zero velocity on Ctrl+C, but a separate command timeout remains important for process crashes.

## Interpretation and next steps

A successful run validates **tracking a fixed reference path**, not obstacle-aware navigation. Specifically:

- This script never calls `analyze_edge()` or `plan_motion()` from Perception Module v1.0.
- It follows the hardcoded polyline, which might depart from a collision-free corridor when pure pursuit rounds a corner.
- It does not handle perception-generated rotation phases, trajectory changes, or global replanning.

For integration, replace the fixed path used by `lookahead_point()` with the **selected translation segment** from Perception Module v1.0. Execute rotation proposals as separate stop-and-rotate phases, then obtain a fresh scan and replan. Maintain independent collision/staleness checks while following any proposal. See [Perception API](perception-api.md) and [Perception strategy](perception-strategy.md) for the planner's responsibilities.
