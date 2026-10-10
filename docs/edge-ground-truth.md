# Edge Passability Ground-Truth Survey

**Documentation:** [README](../README.md) · [Setup](setup.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Task ownership](tasks.md) · [Two-day plan](development.md) · [Ground-truth survey](edge-ground-truth.md) · [Controller debug](controller-debug.md) · [Navigator tests](testing.md) · [RViz & diagnostics](debugging.md)

**Purpose:** Manually inspect **every one of the 40 undirected edges** in the provided 5 × 5 node graph and record evidence-based ground-truth passability labels *before* implementing `is_passable()`. This is a data-collection protocol, not a record of confirmed passability. **All entries start unclassified.**

## What counts as ground truth?

In addition to a `P/B/U` judgment, optionally note whether visible side boundaries support reliable corridor-centering measurements. This is diagnostic geometry, **not** a substitute for the physical-traversability label and not data to hard-code into the submitted navigator.

For this survey, an edge is **passable** when the TurtleBot can traverse between its adjacent node centers inside the intended corridor, with safe clearance and no contact, under careful manual control. An edge is **blocked** only if direct inspection and repeated attempts give convincing evidence that no collision-free route exists for the robot footprint. A failed drive, poor steering, or missing LiDAR information **does not prove blockage**. Mark such cases **uncertain**, and leave them out of binary evaluation until investigated.

Record cross-sectional opening-width evidence **separately** from minimum clearance along a successfully traversed (or attempted) path; a narrow opening only proves an edge blocked if that opening is unavoidable and there is no alternative route inside the allowed corridor. Record observations separately for **A → B** and **B → A** where feasible. The underlying obstacle geometry should be the same, but visibility, access, turning space, and driving outcomes can differ by direction. Do not label a geometrically passable edge blocked solely because the current controller cannot traverse it.

Labels: `P` = confirmed traversable; `B` = convincingly physically blocked; `U` = uncertain/untested. Include confidence (`high`/`medium`/`low`) and evidence. A geometric opening estimate alone is not confirmation.

## 1. Prepare the environment

1. Follow the [repository environment setup](setup.md) to install the **already-extracted** models from this repository, configure `GAZEBO_MODEL_PATH`, and build/source the included `vmc2_project` package. Use the **obstacle-containing** world for ground-truth classification; the obstacle-free model is only for checking sensing and manual control.
2. Launch the simulation from a terminal with the workspace and model paths configured:

   ```bash
   ros2 launch vmc2_project project.launch.py
   ```

3. **Ensure `navigator.py` is not publishing commands** during teleoperation. The unfinished template can also fail if run as-is; inspect the launch configuration and disable the navigator node if it launches automatically. Keep Gazebo and the robot sensor publishers running. Do not run two `/cmd_vel` controllers at once.
4. In another sourced terminal, confirm interfaces and start manual control (if the package is installed):

   ```bash
   ros2 topic list
   ros2 topic info /scan
   ros2 topic info /odom
   ros2 topic info /cmd_vel
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

   If teleop is unavailable, install/use a compatible ROS 2 keyboard teleop. Begin at low speed; manual teleop does not provide automatic collision prevention.
5. Start `rviz2`. Use the `odom` fixed frame **if TF supports it**. Add LaserScan (`/scan`), Odometry (`/odom`), TF (`/tf`), and a Grid. Inspect `header.frame_id` in `/scan` and `/odom` and resolve transform errors before relying on overlays. The provided `draw_scene()` is OpenCV-based and does **not** publish RViz markers.
6. For repeatable measurements, keep the world configuration, robot footprint assumptions, speed limits, and sensor settings consistent. Use **simulation time** when comparing timestamps.

## 2. Test a particular edge

**At each edge:**

1. Stop all motion and stop the navigator before teleporting (as required by the assignment).
2. Teleport the robot to node **A**, oriented toward **B**. Use the script below; it supplies a valid quaternion for east/west/north/south.
3. Wait for odometry and `/scan` to settle. Confirm the robot is at the intended node, pointing along the edge. Be alert to collisions immediately after teleporting; node-center clearance itself must be checked.
4. In RViz, first assess the obstacle geometry **without moving**. Note any visible gap, occluded space, and suspected bottleneck. Save a screenshot or bag recording.
5. Drive slowly toward B, steering around obstacles if a collision-free corridor is visible. Stop with room to brake; **never use contact as the method to discover width**. Record whether the robot center reaches B within the navigation node's typical `node_tol = 0.05 m`, where practical.
6. If the edge is ambiguous, try another approach or observation pose, optionally repeat from B toward A. Do not mark an edge blocked merely because a single scan lacks a gap or a teleop attempt failed.
7. Record label, direction tested, minimum *observed* clearance (if measured), margins, confidence, outcome, and evidence. Distinguish `unknown` from `occupied` regions when interpreting scans.
8. Return to a safe pose; teleport to the next edge start. Capture at least one recorded scan/observation from challenging edges.

### Reusable teleport helper

Create `teleport_edge.py` **locally** with the following content. It prints a ROS 2 service call for any neighboring A → B edge; run the printed command in a ROS-sourced terminal. This avoids manually calculating quaternion signs.

```python
#!/usr/bin/env python3
import math
import shlex
import sys

X = [0.712, 2.136, 3.560, 4.984, 6.408]
Y = [0.0, -1.424, -2.848, -4.272, -5.696]

# usage: python3 teleport_edge.py start_row start_col end_row end_col
ra, ca, rb, cb = map(int, sys.argv[1:5])
assert all(0 <= v < 5 for v in (ra, ca, rb, cb))
assert abs(ra - rb) + abs(ca - cb) == 1, "Nodes must be neighbors"
xa, ya = X[ca], Y[ra]
xb, yb = X[cb], Y[rb]
yaw = math.atan2(yb - ya, xb - xa)
z, w = math.sin(yaw / 2), math.cos(yaw / 2)
payload = (
    "{state: {name: 'burger', pose: {position: "
    f"{{x: {xa:.3f}, y: {ya:.3f}, z: 0.0}}, orientation: "
    f"{{x: 0.0, y: 0.0, z: {z:.8f}, w: {w:.8f}}}}}, "
    "reference_frame: 'world'}}"
)
print("ros2 service call /set_entity_state "
      "gazebo_msgs/srv/SetEntityState " + shlex.quote(payload))
```

Example, for the final southbound challenge:

```bash
python3 teleport_edge.py 3 4 4 4
# Copy and run the printed ros2 service call.
```

**Note:** For manual driving, you do **not** need to change `self.start_pos`, because you are not running the navigator. If later running the navigator from a teleported start, the assignment requires setting `self.start_pos` to the same position. Teleporting to one node does not force A* to choose a particular next edge.

## 3. Capture evidence and observations

Open a separate sourced terminal to record each edge:

```bash
ros2 bag record -o bags/R0C0_to_R0C1 /scan /odom /tf /tf_static
```

Stop the recording after the trial, and use a **different output name for every run** (e.g., `R3C4_to_R4C4_trial1`). ROS bags enable later replay of identical input data. Be careful not to mix replayed and live publishers on the same topics.

During each run monitor:

| Tool/display | Check | Why it matters |
|---|---|---|
| RViz LaserScan `/scan` | Obstacle surfaces, occlusion, angular sparsity, disappearing returns | Whether a visible opening is actually supported by sensor evidence |
| RViz Odometry `/odom` and TF | Pose, heading, transforms, any sudden jumps | Whether observations from different positions can be aligned |
| Gazebo top/orthographic view | Actual robot body vs. obstacle geometry | Visual ground-truth support for clearance and collision assessment |
| `/cmd_vel` via `ros2 topic echo` or `rqt_plot` | Intended forward/turn speed, stopping | Distinguish controller/teleop limitations from geometry |
| Optional custom Marker/PointCloud2 display | Accumulated points, candidate route, closest clearance | **Not available in template**; only if a visualization node is added |

The LiDAR shows first returns, **not all surfaces behind them**. RViz LaserScan decay is a visual history, not a registered occupancy map. A physical gap measurement should be conservative and should state how it was obtained; report `not measured` if necessary. Be careful with contact: a trial with any touch is not a successful no-collision pass.

### Per-trial record template

```text
Edge ID:
Direction: A -> B / B -> A
World: obstacle-containing / obstacle-free
Start pose + heading:
Label: P / B / U
Confidence: high / medium / low
Outcome: traversed to node / stopped safely / collision / aborted
Obstacle geometry and available routes:
Estimated narrowest clearance (m): value + method / not measured
Unknown or occluded regions:
Number of attempts and opposite-direction results:
Relevant ROS bag, screenshot/video paths:
Notes (steering, sensor artifacts, possible false blockage):
```

## 4. Ground-truth inventory: all 40 edges

Node labels use `RrCc` (row increases southward; column increases eastward). All 40 edges below are **initially `U`**. Fill in the status as evidence is collected. To avoid conflating geometry and controllability, enter an edge as `P` only after a clean traversal, `B` only with convincing geometric evidence, and otherwise `U`. Use the trial notes for directional differences.

### Horizontal edges (20)

| ID | A (west) | B (east) | Label | Confidence | Evidence / observations |
|---|---|---|---|---|---|
| H00 | R0C0 (0.712, 0.000) | R0C1 (2.136, 0.000) | U | — | — |
| H01 | R0C1 (2.136, 0.000) | R0C2 (3.560, 0.000) | U | — | — |
| H02 | R0C2 (3.560, 0.000) | R0C3 (4.984, 0.000) | U | — | — |
| H03 | R0C3 (4.984, 0.000) | R0C4 (6.408, 0.000) | U | — | — |
| H10 | R1C0 (0.712, -1.424) | R1C1 (2.136, -1.424) | U | — | — |
| H11 | R1C1 (2.136, -1.424) | R1C2 (3.560, -1.424) | U | — | — |
| H12 | R1C2 (3.560, -1.424) | R1C3 (4.984, -1.424) | U | — | — |
| H13 | R1C3 (4.984, -1.424) | R1C4 (6.408, -1.424) | U | — | — |
| H20 | R2C0 (0.712, -2.848) | R2C1 (2.136, -2.848) | U | — | — |
| H21 | R2C1 (2.136, -2.848) | R2C2 (3.560, -2.848) | U | — | — |
| H22 | R2C2 (3.560, -2.848) | R2C3 (4.984, -2.848) | U | — | — |
| H23 | R2C3 (4.984, -2.848) | R2C4 (6.408, -2.848) | U | — | — |
| H30 | R3C0 (0.712, -4.272) | R3C1 (2.136, -4.272) | U | — | — |
| H31 | R3C1 (2.136, -4.272) | R3C2 (3.560, -4.272) | U | — | — |
| H32 | R3C2 (3.560, -4.272) | R3C3 (4.984, -4.272) | U | — | — |
| H33 | R3C3 (4.984, -4.272) | R3C4 (6.408, -4.272) | U | — | — |
| H40 | R4C0 (0.712, -5.696) | R4C1 (2.136, -5.696) | U | — | — |
| H41 | R4C1 (2.136, -5.696) | R4C2 (3.560, -5.696) | U | — | — |
| H42 | R4C2 (3.560, -5.696) | R4C3 (4.984, -5.696) | U | — | — |
| H43 | R4C3 (4.984, -5.696) | R4C4 (6.408, -5.696) | U | — | — |

### Vertical edges (20)

| ID | A (north) | B (south) | Label | Confidence | Evidence / observations |
|---|---|---|---|---|---|
| V00 | R0C0 (0.712, 0.000) | R1C0 (0.712, -1.424) | U | — | — |
| V01 | R0C1 (2.136, 0.000) | R1C1 (2.136, -1.424) | U | — | — |
| V02 | R0C2 (3.560, 0.000) | R1C2 (3.560, -1.424) | U | — | — |
| V03 | R0C3 (4.984, 0.000) | R1C3 (4.984, -1.424) | U | — | — |
| V04 | R0C4 (6.408, 0.000) | R1C4 (6.408, -1.424) | U | — | — |
| V10 | R1C0 (0.712, -1.424) | R2C0 (0.712, -2.848) | U | — | — |
| V11 | R1C1 (2.136, -1.424) | R2C1 (2.136, -2.848) | U | — | — |
| V12 | R1C2 (3.560, -1.424) | R2C2 (3.560, -2.848) | U | — | — |
| V13 | R1C3 (4.984, -1.424) | R2C3 (4.984, -2.848) | U | — | — |
| V14 | R1C4 (6.408, -1.424) | R2C4 (6.408, -2.848) | U | — | — |
| V20 | R2C0 (0.712, -2.848) | R3C0 (0.712, -4.272) | U | — | — |
| V21 | R2C1 (2.136, -2.848) | R3C1 (2.136, -4.272) | U | — | — |
| V22 | R2C2 (3.560, -2.848) | R3C2 (3.560, -4.272) | U | — | — |
| V23 | R2C3 (4.984, -2.848) | R3C3 (4.984, -4.272) | U | — | — |
| V24 | R2C4 (6.408, -2.848) | R3C4 (6.408, -4.272) | U | — | — |
| V30 | R3C0 (0.712, -4.272) | R4C0 (0.712, -5.696) | U | — | — |
| V31 | R3C1 (2.136, -4.272) | R4C1 (2.136, -5.696) | U | — | — |
| V32 | R3C2 (3.560, -4.272) | R4C2 (3.560, -5.696) | U | — | — |
| V33 | R3C3 (4.984, -4.272) | R4C3 (4.984, -5.696) | U | — | — |
| V34 | R3C4 (6.408, -4.272) | R4C4 (6.408, -5.696) | U | — | — |

## 5. Priority edges for clearance analysis

These four edges were identified by the team as good clearance tests, with the last one regarded as the hardest **hypothesis to investigate**, not a confirmed classification:

| Priority | Edge | Suggested teleport command | Initial heading |
|---|---|---|---|
| 1 | (3.560, −5.696) → (4.984, −5.696) | `python3 teleport_edge.py 4 2 4 3` | East / 0° |
| 2 | (4.984, −1.424) → (6.408, −1.424) | `python3 teleport_edge.py 1 3 1 4` | East / 0° |
| 3 | (4.984, −2.848) → (6.408, −2.848) | `python3 teleport_edge.py 2 3 2 4` | East / 0° |
| 4 | (6.408, −4.272) → (6.408, −5.696) | `python3 teleport_edge.py 3 4 4 4` | South / −90° |

Use these to refine the measurement protocol, but **do not stop after these four**: the goal is a label for every edge, including apparently empty corridors. When time is limited, mark remaining edges `U` instead of guessing.

## 6. Review and consolidate ground truth

Before using the resulting labels to evaluate `is_passable()`:

- Verify that each `P` edge has a documented **contact-free traversal**, ideally repeated and tested in both directions when possible.
- Verify that each `B` edge has geometric or repeated observational evidence of insufficient continuous clearance, rather than merely a failed maneuver or a limited LiDAR viewpoint.
- Resolve conflicting evidence or downgrade the edge to `U` with a clear note.
- Separate **ground-truth labels** from the detector's predicted `True`/`False` values; only then calculate false positives/negatives on the confirmed subset.
- Remember the assignment prohibits hard-coding or prerecorded maze-specific knowledge **in the submitted navigator**. This survey is for testing, debugging, and algorithm validation, not to build a fixed blocked-edge lookup table into `navigator.py`.

**Completion criterion:** All 40 graph edges have been explicitly reviewed and assigned `P`, `B`, or `U`, with evidence notes. The desired final dataset has confident P/B labels wherever safely determinable; any remaining uncertainty must be visible rather than silently resolved.

## Interpreting Stage 2 observations

Keep ground-truth `P` / `B` / `U` labels based on observed physical traversal. For future detector comparison, separately record the **minimum short-horizon clearance predicted for an obstacle-shifted lookahead**. An apparently safe short arc is not proof that the whole edge can be traversed; failing one local target also does not establish that the edge is blocked. The survey remains independent of the algorithm and must not become a prerecorded obstacle map.
