# Suggested task division

**Documentation:** [README](../README.md) · [Setup](setup.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Perception strategy](perception-strategy.md) · [Perception API](perception-api.md) · **Task ownership** · [Development plan](development.md) · [Ground-truth survey](edge-ground-truth.md) · [Controller debug](controller-debug.md) · [Navigator tests](testing.md) · [Offline tests](navigation-offline-tests.md) · [RViz & debugging](debugging.md)

---

All teammates should first complete [Environment setup](setup.md) so controller, perception, and planning tests use the same course assets.

## Goal

Divide the implementation into three reasonably independent workstreams while keeping a **single integrated `navigator.py`** as the final deliverable. The course template already contains graph utilities, an A* implementation, ROS subscriptions/publishing, and most of the navigation state-machine structure. The work is primarily to complete the missing functions and make them operate reliably together.

The division below is based on **interfaces and dependencies**, rather than assigning one person to each of the three course keywords regardless of workload.

**Status note (12 October 2026):** This checklist captures the original division of work; unchecked boxes are not a reliable snapshot of current completion. The subsystems have now been integrated, with an observed three-checkpoint Gazebo run of roughly 2:30. Full-run validation is outstanding.

## Proposed Ownership

| Proposed lead | Workstream | Primary responsibility | Key functions / code |
|---|---|---|---|
| **Ki Chun Tong** | **LiDAR perception and passability** | Interpret scans, estimate trustworthy left/right corridor clearances and confidence, and distinguish avoidable from blocked edges | `is_passable()`, scan-processing helpers, optional LiDAR visualization |
| **Shaan Salian** | **Motion control and local avoidance** | Select an obstacle-aware lookahead target and command `/cmd_vel`; use pure pursuit as the baseline and optionally add LiDAR centering | `turtlebot_control()`, `obstacle_avoided_target()`, controller parameters |
| **Vo Dinh Nhan** | **Path planning, state transitions, and integration tests** | Finish path assembly and `FOLLOW`/`BACKTRACK`/`REPLAN` transitions; validate end-to-end behavior | `plan_from()`, relevant TODOs in `control_step()`, scenario testing |

**These names are placeholders for a team decision**, not a judgment of anyone's preferences or strengths. Feel free to swap leads. All three members should review the shared perception/control interface and participate in full-maze testing.

### Workstream A — LiDAR perception and passability

**Suggested lead: Ki Chun Tong**

**Deliverables**
- [ ] Convert valid `/scan` ranges and angles to Cartesian obstacle points, with correct robot/sensor frame handling.
- [ ] Visualize obstacle points relative to the robot and intended edge; annotate useful clearances for debugging.
- [ ] Account for the TurtleBot's stated **0.178 m width** and a *tunable*, experimentally justified safety margin.
- [ ] **Stage 1:** estimate candidate cross-sectional opening widths; flag openings narrower than the robot width plus tuned margins, and only reject the *edge* when the bottleneck is confirmed unavoidable.
- [ ] **Stage 2:** identify a feasible opening and help assess the **short motion induced by a shifted pure-pursuit target**, using Euclidean clearance, swept robot footprint, corridor bounds, stopping distance, and observed-versus-unknown space. A full local trajectory is not required.
- [ ] Implement `is_passable(seg_dir_yaw)` using both stages; distinguish a bypassable obstacle or an unknown region from confirmed edge blockage.
- [ ] Provide obstacle/clearance information that the avoidance controller can reuse.
- [ ] Estimate segment-aligned left/right clearance `d_L`, `d_R`, plus validity/confidence; suppress centering when a side wall is not identified reliably.
- [ ] Compare the detector with the complete [annotated ground-truth inventory](edge-ground-truth.md#current-annotated-ground-truth), including clear, constrained, and blocked edges. Document any ambiguous sensor observations separately from the existing ground-truth labels.
- [ ] Optionally accumulate obstacle points **per active edge** if single-scan perception demonstrably fails due to occlusion.

**Initial scope:** single-scan Cartesian points, clearance tests, and visualization. **Defer** full occupancy-grid mapping and probabilistic log-odds unless tests justify them.

**Definition of done:**
- [ ] From a teleported start near a relevant obstacle (see [teleport-based tests](testing.md)), valid LiDAR returns appear in RViz and align plausibly with the visible Gazebo obstacles.
- [ ] An empty corridor stays passable; a bypassable obstacle is not prematurely classified as blocked; a genuinely blocked passage is rejected with repeatable evidence.
- [ ] Obstacle points or diagnostics follow the robot’s pose and remain in the correct coordinate frame when it rotates.
- [ ] The passability/clearance assessment can be explained from recorded scan data; perception publishes no velocity commands itself.
- [ ] Record `W_min` for candidate openings, whether any narrow slice is **unavoidable**, minimum clearance along the predicted short motion, and the classification for **all four edge tests in [the four edge tests](testing.md#edge-specific-perception-and-clearance-tests)**, including the final challenge edge; compare results across repeated observations and at least three trials per edge.

**Quick test:** Use the four edge-specific perception cases in **[the four edge tests](testing.md#edge-specific-perception-and-clearance-tests)**, starting with `(3.560, -5.696) → (4.984, -5.696)` and progressing to the designated challenge `(6.408, -4.272) → (6.408, -5.696)`. Set the teleport pose and `self.start_pos` consistently; use a test-only edge selection so A* does not choose an unrelated segment.

### Workstream B — Motion control and local obstacle avoidance

**Suggested lead: Shaan Salian**

**Deliverables**
- [ ] **Select an obstacle-aware lookahead target and command `/cmd_vel`** (core responsibility in the supplied template).
- [ ] Implement `obstacle_avoided_target(robot_pos, seg_dir_yaw)` by adjusting the normal pure-pursuit lookahead point toward a suitable detected opening; keep target and predicted short motion inside safe corridor space.
- [ ] Implement `turtlebot_control(robot_pos, robot_h, target, dt, Ld)` to return `(v_cmd, w_cmd)` and drive the target using pure pursuit as the baseline.
- [ ] Set `lookahead_dist`, `v_max`, `w_max`, and optional PID parameters.
- [ ] Establish reliable heading alignment, straight driving, turns, and waypoint arrival on the **obstacle-free** track using the [`straight`, `turn`, and `loop` controller debug routes](controller-debug.md).
- [ ] **Optional enhancement:** Add lateral centering only when perception supplies valid side measurements, using a bounded angular PID correction; disable/reset it near turns, at junctions, or when measurements are unreliable.
- [ ] Check the **predicted pure-pursuit short motion**, not just the target point or the straight line to it; ensure adequate swept-footprint clearance and safe stopping.
- [ ] Add safe speed reduction/stopping when nearby space is uncertain, including during backtracking.
- [ ] Record repeatability and tuning results for controller parameters.

**Definition of done:**
- [ ] Complete the [controller-only debug tests](controller-debug.md): `straight` from `(0.712, 0.0)` to `(2.136, 0.0)`, `turn` from `(2.136, -4.272)` via `(2.136, -5.696)` to `(3.560, -5.696)`, and then the full 18-waypoint `loop` without skipped graph edges or collisions.
- [ ] An obstacle-aware lookahead target is selected and `publish_cmd()` sends bounded `/cmd_vel` commands that track it; observe actual movement on `/odom`.
- [ ] Once perception is integrated, a feasible avoidance target is tracked without contact with obstacles; the robot slows or stops when immediate clearance is not established.
- [ ] Repeat the same short run at least three times and record failures and controller parameters.

**Quick test:** Follow [controller-debug.md](controller-debug.md) to select `AMAZE_NAV_CONTROLLER_TEST=straight`, `turn`, or `loop`, teleport to the corresponding start pose, and bypass unfinished perception only in debug mode. Watch `/odom`, `/cmd_vel`, and the OpenCV path view. After the isolated controller tests pass, test obstacle-aware targeting with the integrated navigator; see [navigator tests](testing.md).

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
3. **Shared perception result:** Agree on `d_L`, `d_R`, validity/confidence, active edge frame, and candidate opening/shifted target, and short-motion clearance; avoid inconsistent geometric checks in separate functions.
4. **Blending policy:** Pure pursuit is baseline; choose confidence gating, centering gain/sign, turn suppression, PID reset, and safety precedence before integration.
4. **Uncertainty and safety:** What does the robot do when an edge is neither known clear nor definitively blocked? `is_passable()` is boolean, so **unknown must not silently mean safe forward movement**.
5. **Stopping and clearance:** Choose initial conservative values for robot-radius approximation, safety margin, and slow/stop distances, then tune experimentally.
6. **Code integration:** Helpers may be added, but avoid removing/modifying provided template code unnecessarily, in line with the assignment instructions.

A simple agreed helper such as `get_local_obstacles()` (returning points in a documented frame) is a useful first interface. A richer shared result can be introduced later if needed.

See also [two-day sequencing and risk management](development.md), [test acceptance cases](testing.md), and [RViz diagnostics](debugging.md).

**Perception evidence:** Ki Chun can collect the initial [edge-ground-truth survey](edge-ground-truth.md) using manual teleoperation (without running the navigator), with teammates helping on ambiguous edges. The team has now annotated all 40 edges: unmarked edges are `P-clear`, green are `P-constrained`, and red are `B`. Compare `is_passable()` against those recorded labels; keep actual trial outcomes and evidence separate from the survey classifications. See the [four automated navigator checks](testing.md#four-priority-navigator-tests).
