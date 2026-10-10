# Suggested task division

**Documentation:** [README](../README.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Task ownership](tasks.md) · [Two-day plan](development.md) · [Teleport tests](testing.md) · [RViz & diagnostics](debugging.md)

---

## Goal

Divide the implementation into three reasonably independent workstreams while keeping a **single integrated `navigator.py`** as the final deliverable. The course template already contains graph utilities, an A* implementation, ROS subscriptions/publishing, and most of the navigation state-machine structure. The work is primarily to complete the missing functions and make them operate reliably together.

The division below is based on **interfaces and dependencies**, rather than assigning one person to each of the three course keywords regardless of workload.

## Proposed Ownership

| Proposed lead | Workstream | Primary responsibility | Key functions / code |
|---|---|---|---|
| **Ki Chun Tong** | **LiDAR perception and passability** | Interpret scans, assess usable clearance, distinguish obstacles that can be avoided from truly blocked edges | `is_passable()`, scan-processing helpers, optional LiDAR visualization |
| **Shaan Salian** | **Motion control and local avoidance** | Get reliable waypoint tracking and turns; select safe obstacle-aware targets and command velocities | `turtlebot_control()`, `obstacle_avoided_target()`, controller parameters |
| **Vo Dinh Nhan** | **Path planning, state transitions, and integration tests** | Finish path assembly and `FOLLOW`/`BACKTRACK`/`REPLAN` transitions; validate end-to-end behavior | `plan_from()`, relevant TODOs in `control_step()`, scenario testing |

**These names are placeholders for a team decision**, not a judgment of anyone's preferences or strengths. Feel free to swap leads. All three members should review the shared perception/control interface and participate in full-maze testing.

### Workstream A — LiDAR perception and passability

**Suggested lead: Ki Chun Tong**

**Deliverables**
- [ ] Convert valid `/scan` ranges and angles to Cartesian obstacle points, with correct robot/sensor frame handling.
- [ ] Visualize obstacle points relative to the robot and intended edge; annotate useful clearances for debugging.
- [ ] Account for the TurtleBot's stated **0.178 m width** and a *tunable*, experimentally justified safety margin.
- [ ] Implement `is_passable(seg_dir_yaw)` without treating every obstacle directly ahead as an impassable edge.
- [ ] Provide obstacle/clearance information that the avoidance controller can reuse.
- [ ] Document ambiguous/unknown observations and define when there is sufficient evidence to mark an edge blocked.
- [ ] Optionally accumulate obstacle points **per active edge** if single-scan perception demonstrably fails due to occlusion.

**Initial scope:** single-scan Cartesian points, clearance tests, and visualization. **Defer** full occupancy-grid mapping and probabilistic log-odds unless tests justify them.

**Definition of done:**
- [ ] From a teleported start near a relevant obstacle (see [teleport-based tests](testing.md)), valid LiDAR returns appear in RViz and align plausibly with the visible Gazebo obstacles.
- [ ] An empty corridor stays passable; a bypassable obstacle is not prematurely classified as blocked; a genuinely blocked passage is rejected with repeatable evidence.
- [ ] Obstacle points or diagnostics follow the robot’s pose and remain in the correct coordinate frame when it rotates.
- [ ] The passability/clearance assessment can be explained from recorded scan data; perception publishes no velocity commands itself.
- [ ] Record clearance plots, the selected candidate opening/trajectory, and the classification for **all four edge tests in [the four edge tests](testing.md#edge-specific-perception-and-clearance-tests)**, including the final challenge edge; compare results across repeated observations and at least three trials per edge.

**Quick test:** Use the four edge-specific perception cases in **[the four edge tests](testing.md#edge-specific-perception-and-clearance-tests)**, starting with `(3.560, -5.696) → (4.984, -5.696)` and progressing to the designated challenge `(6.408, -4.272) → (6.408, -5.696)`. Set the teleport pose and `self.start_pos` consistently; use a test-only edge selection so A* does not choose an unrelated segment.

### Workstream B — Motion control and local obstacle avoidance

**Suggested lead: Shaan Salian**

**Deliverables**
- [ ] Set `lookahead_dist`, `v_max`, `w_max`, and optional PID parameters.
- [ ] Implement `turtlebot_control(robot_pos, robot_h, target, dt, Ld)` to return `(v_cmd, w_cmd)`.
- [ ] Establish reliable heading alignment, straight driving, turns, and waypoint arrival on the **obstacle-free** track.
- [ ] Implement `obstacle_avoided_target(robot_pos, seg_dir_yaw)` using the agreed perception output.
- [ ] Ensure targets remain in the intended corridor and immediate motion has adequate clearance.
- [ ] Add safe speed reduction/stopping when nearby space is uncertain, including during backtracking.
- [ ] Record repeatability and tuning results for controller parameters.

**Definition of done:**
- [ ] From a chosen start node on the **obstacle-free** world (e.g. checkpoint 1), the robot aligns with the next edge, travels to the subsequent waypoint, and settles inside `node_tol` without oscillation or collision.
- [ ] The published `/cmd_vel` values respect configured limits; robot motion observed on `/odom` follows the intended target.
- [ ] Once perception is integrated, a feasible avoidance target is tracked without contact with obstacles; the robot slows or stops when immediate clearance is not established.
- [ ] Repeat the same short run at least three times and record failures and controller parameters.

**Quick test:** Teleport to **checkpoint 1** `(3.560, -4.272)` in the obstacle-free world and drive a single adjacent edge, then repeat with an obstacle-containing segment. Watch `/odom`, `/cmd_vel`, `/scan`, and the current lookahead target (if published as a marker).

### Workstream C — Planning, navigation state machine, and testing

**Suggested lead: Vo Dinh Nhan**

**Deliverables**
- [ ] Complete `plan_from()` using the provided `nearest_graph_coord()` and `astar_path()`.
- [ ] Complete the `FOLLOW` TODO: record `(ga, gb)` and the previous node when the passage is determined blocked, then enter `BACKTRACK`.
- [ ] Complete the `BACKTRACK` TODO: return to the previous node, stop within tolerance, then enter `REPLAN`.
- [ ] Complete the `REPLAN` TODO: remove the blocked edge, replan from the correct node, then return to `FOLLOW`.
- [ ] Check that edge identity and waypoint progress remain stable when the robot deviates around obstacles.
- [ ] Run integration scenarios: no obstacles, one blocked edge, alternate route, repeated runs, and recovery after a failed approach.
- [ ] Keep shared test notes, demo evidence, and final run instructions.

**Important:** `astar_path()` is **already implemented in the supplied template**. The substantial work here is path assembly, state transitions, edge bookkeeping, and integration/reliability testing—not rewriting A*.

**Definition of done:**
- [ ] Starting from a known teleported node, `plan_from()` produces a valid waypoint sequence and the robot progresses through the expected states.
- [ ] When a selected edge is confirmed blocked, the controller stops, returns to the previous node, removes the **correct** graph edge, and produces an alternative route when one exists.
- [ ] No state transition removes unrelated edges or repeatedly alternates between `BACKTRACK` and `REPLAN`.
- [ ] The route shown by `draw_scene()` matches the active graph and the robot’s `/odom` position; repeated short scenario runs behave consistently.

**Quick test:** Use **checkpoint 1** `(3.560, -4.272)` or **checkpoint 2** `(4.984, -4.272)` to avoid restarting the entire maze. For the planning unit test, use a known simulated blocked edge or a test double for `is_passable()`; this is **test-only** and must not become a hard-coded blocked edge in the final solution.

## Interface Agreement — Do This Together First

Before implementing the workstreams independently, agree on these contracts in a short team meeting:

1. **Edge identity:** How do we store the currently attempted graph edge? When exactly do we reset temporary LiDAR observations?
2. **Coordinate frames:** LiDAR angles are sensor/robot-relative; `seg_dir_yaw` and waypoints are in the world/odometry frame. How will helpers transform between them?
3. **Shared perception result:** What obstacle points, clearance estimates, or candidate targets does avoidance receive? Avoid running inconsistent geometric checks in two separate functions.
4. **Uncertainty and safety:** What does the robot do when an edge is neither known clear nor definitively blocked? `is_passable()` is boolean, so **unknown must not silently mean safe forward movement**.
5. **Stopping and clearance:** Choose initial conservative values for robot-radius approximation, safety margin, and slow/stop distances, then tune experimentally.
6. **Code integration:** Helpers may be added, but avoid removing/modifying provided template code unnecessarily, in line with the assignment instructions.

A simple agreed helper such as `get_local_obstacles()` (returning points in a documented frame) is a useful first interface. A richer shared result can be introduced later if needed.

See also [two-day sequencing and risk management](development.md), [test acceptance cases](testing.md), and [RViz diagnostics](debugging.md).
