# Teleport-based testing and acceptance

**Documentation:** [README](../README.md) · [Setup](setup.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Perception strategy](perception-strategy.md) · [Perception API](perception-api.md) · [Task ownership](tasks.md) · [Development plan](development.md) · [Ground-truth survey](edge-ground-truth.md) · [Controller debug](controller-debug.md) · **Navigator tests** · [Offline tests](navigation-offline-tests.md) · [RViz & debugging](debugging.md)

---

**Offline tests first:** For Nhan’s A* / `FOLLOW → BACKTRACK → REPLAN` logic without real perception or controller code, see [offline navigation tests](navigation-offline-tests.md). Its mocked blocked edges are test fixtures only, not Gazebo or LiDAR validation.

**Prerequisite:** Complete [Environment setup](setup.md) first; the repository already contains the extracted models and ROS package.

For an end-to-end run, start Gazebo first and then start `navigator.py` in a separate sourced terminal using [the navigator launch instructions](setup.md#7-run-the-autonomous-navigator). For perception-only investigation use [LiDAR Explorer](setup.md#8-launch-and-use-lidar-explorer), which does not send velocity commands. Once the robot is positioned with navigator control disabled, choose an edge with `,`/`.`, assess with `b`, and inspect motion candidates with `t`; these visual trials are distinct from an autonomous checkpoint run.

**Scope:** This document validates the implemented navigator, including perception decisions, control, and replanning. For **manual teleoperation, ROS bag collection, and the team’s complete P-clear/P-constrained/B labels for all 40 edges**, use the [ground-truth survey](edge-ground-truth.md). Do not treat the four cases below as a complete ground-truth inventory.

**Ground-truth reference:** [Annotated 40-edge map and table](edge-ground-truth.md#current-annotated-ground-truth). Green = constrained passable, red = blocked, unmarked = unobstructed passable. Include all three classes when evaluating false-blocked decisions and traversal success.

## Latest integration checkpoint (12 October 2026)

The first integrated perception/planning/control baseline has been exercised in Gazebo. **Reported result: three checkpoints reached in approximately 2 minutes 30 seconds.** This is an observed *partial run*, not evidence that the full maze was completed within the assignment's 2:30 target. The checkpoint IDs, timing clock, world seed/configuration, and run video/log are not included in this documentation snapshot.

For the next comparable test, record start/finish times, which checkpoints were reached, whether BACKTRACK/REPLAN occurred, wall contacts or safety holds, and whether PID narrow-passage centering or center-first entrance repositioning activated. Perform at least three clean-start repetitions and report each duration rather than assuming this first observed time is repeatable.

The integrated navigator uses pure pursuit for translation, PID-assisted turns/conditional narrow-passage centering, asynchronous `plan_motion()` results, live LiDAR guards, and forward-driving return plans (no reverse-driving recovery). The new center-first entrance maneuver should be treated as experimental until repeatable tight-gap entry is demonstrated.

## Controller-only testing (before integration)

For Shaan’s independent control work, use [controller-debug.md](controller-debug.md) instead of these obstacle-world acceptance tests. It defines the `AMAZE_NAV_CONTROLLER_TEST` switch, exact teleport poses, and mocked `straight`, `turn`, and `loop` paths. The 18-waypoint `loop` explicitly preserves all intermediate graph nodes and validates every consecutive edge with `GridGraph.has_edge()`. Perception bypasses are **test-only** and should not be used for the obstacle-containing world. Disable the proposed LiDAR centering correction for these isolated pure-pursuit runs (`λ_confidence = 0`); evaluate centering separately after integration.

## Teleport-Based Smoke Tests

The course instructions explicitly allow teleporting the TurtleBot to any graph node for testing. This is useful for checking one workstream at a time instead of driving from the maze entrance after every code change.

**Before teleporting:** stop the `navigator.py` control node, as required by the assignment. Leave Gazebo running. After teleporting, update `self.start_pos` in `navigator.py` to the same `(x, y)` location **before restarting the navigator**. The template initially sets `self.start_pos = (0.0, 0.0)`; leaving that unchanged can produce an inconsistent planned start. The command below sets yaw to zero (facing positive world X), so adjust orientation if your test edge points another way. Be mindful of the robot's final pose and odometry when reusing a simulator session.

**Checkpoint 1: a convenient central-ish test node**

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState   "{state: {name: 'burger', pose: {position: {x: 3.560, y: -4.272, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, reference_frame: 'world'}}"
```

Set in `navigator.py` for this test:

```python
self.start_pos = (3.560, -4.272)
```

**Checkpoint 2: alternative nearby start**

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState   "{state: {name: 'burger', pose: {position: {x: 4.984, y: -4.272, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, reference_frame: 'world'}}"
```

Use `self.start_pos = (4.984, -4.272)` for this test. The PDF also supplies a teleport command to checkpoint 3 `(7.120, -5.696)`; that point is the template's goal, so it is generally less useful for testing an ordinary edge traversal.

## Four Priority Navigator Tests

Rather than relying only on checkpoint 1, use the following **four directed graph edges** as a repeatable perception test suite. These are proposed repeatable acceptance cases, selected by the team; whether and where they contain a traversable gap must be established from the actual obstacle world and LiDAR data, not assumed from the graph coordinates alone.

| Order | Directed edge A → B (metres) | Teleport position (A) | Initial heading | Purpose |
|---|---|---|---|---|
| 1 | `(3.560, -5.696)` → `(4.984, -5.696)` | `(3.560, -5.696)` | `0°` (+X) | First clearance-estimation case |
| 2 | `(4.984, -1.424)` → `(6.408, -1.424)` | `(4.984, -1.424)` | `0°` (+X) | Second obstacle/gap geometry case |
| 3 | `(4.984, -2.848)` → `(6.408, -2.848)` | `(4.984, -2.848)` | `0°` (+X) | Third obstacle/gap geometry case |
| 4 | **`(6.408, -4.272)` → `(6.408, -5.696)`** | `(6.408, -4.272)` | **`-90°` (−Y)** | **Final challenge: passage-clearance and steering validation** |

All four are adjacent nodes in the template's grid; their nominal length is **1.424 m**. For each test, record **both stages separately**: (1) cross-sectional minimum observed gap width `W_min`, the opening/boundaries it refers to, and whether its bottleneck is demonstrably **unavoidable**; (2) minimum **Euclidean clearance along the predicted short pure-pursuit motion toward the shifted target**, compared with the effective robot radius plus safety margin, plus corridor/turning and stopping feasibility. Re-evaluate after each scan; a single locally safe motion does not prove the whole edge passable. A verified too-narrow *unavoidable* opening is rejectable; a wide slice does **not** prove passability. Distinguish **unknown/unseen** space from measured clearance.

**Copy-pasteable teleport examples (stop the navigator first; keep Gazebo running):**

**Test 1 — horizontal, bottom row**

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState "{state: {name: 'burger', pose: {position: {x: 3.560, y: -5.696, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, reference_frame: 'world'}}"
```

Set `self.start_pos = (3.560, -5.696)`.

**Test 2 — horizontal, upper right**

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState "{state: {name: 'burger', pose: {position: {x: 4.984, y: -1.424, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, reference_frame: 'world'}}"
```

Set `self.start_pos = (4.984, -1.424)`.

**Test 3 — horizontal, middle right**

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState "{state: {name: 'burger', pose: {position: {x: 4.984, y: -2.848, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, reference_frame: 'world'}}"
```

Set `self.start_pos = (4.984, -2.848)`.

**Test 4 — final challenge, southbound (−Y)**

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState "{state: {name: 'burger', pose: {position: {x: 6.408, y: -4.272, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.70710678, w: 0.70710678}}, reference_frame: 'world'}}"
```

Set `self.start_pos = (6.408, -4.272)`. This orientation faces `−Y` (`yaw = −π/2`) toward `(6.408, -5.696)`.

**Important test setup:** Teleportation and changing `self.start_pos` **do not guarantee the A* planner chooses the proposed edge**—it may select another equally short route to the default goal. For a focused test, temporarily direct an isolated edge test/harness or a test-only waypoint path from A to B (e.g., temporarily set the test goal to B and confirm the chosen path). Restore the original goal, template settings, and graph for final runs; do not hard-code blocked edges or fixed obstacle locations into the submitted solution. Inspect `/odom` after teleporting: the simulator state, odometry, and navigator's planned start must agree before interpreting clearance results.

**Additional LiDAR-centering acceptance checks:**

- [ ] On a well-observed straight corridor, verify `d_L`, `d_R`, the signed centering error and confidence against RViz geometry.
- [ ] Confirm the correction steers toward the side with more clearance without losing graph-path progress.
- [ ] Confirm centering is disabled at junctions/90° turns or when one boundary is not reliably observed.
- [ ] Compare pure pursuit alone with corrected steering over repeated runs: lateral deviation, oscillation, time, and minimum clearance.
- [ ] Immediate collision-stop logic always overrides the blended steering.

**Navigator perception acceptance checks for each priority edge:**

- [ ] RViz `/scan` geometry matches Gazebo at the initial pose; check the heading especially for Test 4.
- [ ] Visualize LiDAR point cloud, selected opening, shifted lookahead target, and short predicted motion, and robot clearance footprint in a consistent frame.
- [ ] Report cross-sectional `W_min` and whether a narrow bottleneck is unavoidable, **plus** minimum short-horizon motion clearance; check whether their locations change as new scans arrive.
- [ ] Verify that the chosen maneuver stays free of observed obstacles; do not call unseen space clear.
- [ ] Log passability status, minimum clearance estimate, and reason for the decision at multiple positions along the edge.
- [ ] Repeat at least three times, noting correct traversals, false-blocked outcomes, and collisions/unsafe approaches.

For RViz, the most helpful additional debug objects would be an **active-edge line**, **predicted short motion**, **closest-obstacle/bottleneck marker**, and **effective robot-radius circle**. These are proposed marker publishers, not topics already present in the template; the existing `/scan` and `/odom` can be inspected without adding them.

**Suggested navigator smoke-test sequence**

1. Launch the obstacle-free world and teleport to checkpoint 1; confirm `/odom` reflects the new pose.
2. Start the navigator with a matching `self.start_pos`; verify the planned waypoint list and drive one graph edge.
3. Switch to the obstacle world and run the four **[edge-specific tests](#edge-specific-perception-and-clearance-tests)** directed-edge tests in sequence; reserve the southbound edge `(6.408, -4.272) → (6.408, -5.696)` for the hardest clearance/avoidance trial.
4. Exercise a blocked-edge case and verify `FOLLOW → BACKTRACK → REPLAN → FOLLOW` and the graph change.
5. Restore the intended production `start_pos`, remove temporary blocked-edge test hooks, and rerun from the required start before claiming the full task works.

If your ROS/Gazebo environment uses a different model name, inspect the spawned model name rather than silently assuming `burger`; the command above uses the name given in the assignment. The supplied launch/setup determines which world is active.

## Quick Shared Test Notes

Keep one compact shared note or chat thread for tests—no issue tracker required. For each useful run, record: **(a)** directed edge and teleport pose, **(b)** world and changed tuning parameters, **(c)** pass/fail and any collision or false blockage, and **(d)** a screenshot or short recording when it clarifies a problem. For the four perception cases in §8.1, note the estimated clearance and whether the result is repeatable. This is enough to guide debugging and later document the demo.

For display setup, see [RViz & debugging](debugging.md).

**Comparison with ground truth:** Before interpreting an `is_passable()` result as correct or incorrect, consult the corresponding evidence-based label in [the survey](edge-ground-truth.md). Map `P-clear` and `P-constrained` to expected `True`, and `B` to expected `False`. The current annotation labels every edge; separately document which labels have direct traversal evidence.
