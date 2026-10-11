# Perception, obstacle avoidance, and navigation methodology

**Documentation:** [README](../README.md) · [Setup](setup.md) · [Framework](framework.md) · **Methodology** · [Perception strategy](perception-strategy.md) · [Perception API](perception-api.md) · [Task ownership](tasks.md) · [Development plan](development.md) · [Ground-truth survey](edge-ground-truth.md) · [Controller debug](controller-debug.md) · [Navigator tests](testing.md) · [Offline tests](navigation-offline-tests.md) · [RViz & debugging](debugging.md)

---

**Status:** This document records the original methodology and alternative designs. The Perception Module v1.1 trajectory planner is implemented; see the [Perception Module v1.1 planning strategy](perception-strategy.md) for the implemented decision process and [API reference](perception-api.md) for interfaces and limitations. The controller integration remains incomplete. The 5 × 5 graph gives candidate links, not a prerecorded obstacle map. The challenge is to distinguish a truly blocked edge from a passage with a safe path around obstacles.

> **Specification vs. enhancement:** The provided navigator expects `obstacle_avoided_target()` to select an obstacle-aware lookahead target and `turtlebot_control()` / `publish_cmd()` to command `/cmd_vel`. LiDAR-based corridor-centering PID is an **optional addition**, not a replacement for those required functions.

## Why front-range thresholds are insufficient

The nearest return directly ahead can indicate an obstacle but cannot distinguish a fully blocked corridor from one with lateral clearance. Angled, staggered, and occluding obstacles complicate identifying an actual bottleneck. A pair of ranges separated by angle Δθ obeys the cosine rule:

$$d=\sqrt{r_1^2+r_2^2-2r_1r_2\cos(\Delta\theta)}$$

But that measures the distance between **two returns**, not automatically the minimum navigable gap. What ultimately matters is a collision-free traversal; initially we validate safe short motions toward the next node, not a complete local curve in advance.

## LiDAR geometry and frame conversion

A valid range-angle pair `(r, θ)` becomes a sensor-frame point:

$$x=r\cos\theta,\quad y=r\sin\theta$$

For an aligned sensor at the robot origin and robot world pose `(x_r, y_r, ψ)`, the corresponding world point is:

$$x_w=x_r+r\cos(\psi+\theta),\quad y_w=y_r+r\sin(\psi+\theta)$$

Account for an actual sensor offset/rotation, and match readings with appropriately timed pose estimates before accumulating scans. The segment world heading `seg_dir_yaw` differs from the relative scan angle; for an aligned sensor, the segment-relative scan direction is `angle_wrap(seg_dir_yaw - self.yaw)`.

## Minimum clearance and obstacle inflation

The assignment states a **0.178 m** TurtleBot width. With the **default 0.03 m margin** on each side:

$$W_{required}=0.178+2(0.03)=0.238\text{ m}$$

For a circular-footprint approximation:

$$R_{effective}=0.178/2+0.04=0.129\text{ m}$$

These are **tuning examples**, not prescribed safe values; verify the actual footprint and realistic control error. Inflate obstacle geometry by the effective radius, then evaluate the robot centerline against inflated obstacles.

## Candidate methods

| Method | Strength | Limitation | Initial priority |
|---|---|---|---|
| Simple front scan threshold | Fast and easy | Cannot tell bypassable from blocked | Safety check only |
| Single-scan gap detection | Low overhead | Occlusion and unknown space | Start here |
| Cross-sectional clearance | Flags any remaining slice whose **detected internal openings** are all too narrow (operational early rejection; may miss unobserved bypasses) | Diagonal surfaces, occlusion, and possible bypasses complicate rejection | **Stage 1: early geometric filter** |
| Per-edge accumulated obstacle points | Retains prior observations, moderate overhead | Unknown space remains implicit | Add if needed |
| Occupancy grid with local pathfinding | Represents observed free, occupied, unknown | Mapping and planning complexity | Defer |

### Cross-sectional minimum gap

Express points in an edge-aligned frame with longitudinal `s` and lateral `t`, relative to origin `(x₀,y₀)` and edge yaw `ψ`:

$$s=(x-x_0)\cos\psi+(y-y_0)\sin\psi$$
$$t=-(x-x_0)\sin\psi+(y-y_0)\cos\psi$$

At each cross-section `s`, identify **relevant free intervals** between observed boundaries, and estimate `W(s) = t_left(s) - t_right(s)` for each candidate passage. Along a single consistently identified passage, the minimum **observed** cross-sectional width is `W_min = min_s W(s)`.

**Stage 1 — reject definitely too-narrow openings:** An opening with a **verified, unavoidable bottleneck** narrower than the robot's required width (`W_required = 0.178 + 2m`) cannot accommodate the chosen footprint and margin. Reject **that opening**; reject the **whole edge** only when observation establishes that every permitted route must cross an inadequate bottleneck. Do not conclude that arbitrary pairs of nearby LiDAR points bound the whole corridor. A narrow slice might be bypassed elsewhere, and incomplete/occluded surfaces should remain **unknown**, not blocked.

**Stage 2 — short-horizon motion feasibility (chosen baseline):** For openings that survive Stage 1, **do not require a complete obstacle-aware curve to the next graph node**. First choose a feasible local opening (often a gap midpoint), shift the *existing* pure-pursuit lookahead target toward it, and predict the **short motion that `turtlebot_control()` would actually command**. Evaluate the swept robot footprint along this predicted arc or straight segment, including minimum Euclidean distance to observed obstacle surfaces, corridor bounds, motion feasibility, and ability to stop inside known-safe space. Sample densely or use segment/arc clearance calculations. A straight line to a lookahead point is **not** necessarily the pure-pursuit arc. Reject or modify an unsafe target, and repeat after the next scan.

A locally clear prediction means **safe to take that limited motion**, not that the whole 1.424 m edge has been proven passable. Keep **CLEAR / UNKNOWN / BLOCKED** assessments separate from immediate motion authorization. Lack of LiDAR returns does not certify unobserved space; if no safe short motion or observation maneuver is established, stop. In the current implemented early-rejection policy, any remaining slice whose detected internal gaps are all too narrow yields `BLOCKED`; this is not a certified full-corridor barrier. With insufficient opening evidence, return `UNKNOWN`. A graph planner should consider reobservation before permanently removing an edge. **Full local-trajectory planning** is implemented in Perception Module v1.1; see the [current strategy](perception-strategy.md) and [API reference](perception-api.md). The alternatives below provide design rationale if simple target shifts repeatedly fail on diagonal or staggered obstacles.

The final passability decision should combine both stages: **too-narrow unavoidable opening → reject; plausible opening → test a continuous safe trajectory; insufficient evidence → unknown / stop or gather more observations safely**. This is a proposed strategy, not functionality already implemented in the template.

### Per-edge accumulated point clouds

Optionally start a fresh, **world-fixed** obstacle-point buffer when committing to a new edge; convert incoming readings into that frame and downsample nearby points. Preserve observations through pauses and rotations and reset on the next traversal. Do not reset solely because the nearest geometric segment changes during avoidance. Past points help with occlusion but a lack of points in an unseen region is **not** evidence it is free.

### Occupancy grid if experiments demand it

An edge-local grid tracks **occupied**, **observed free**, and **unknown** cells, using ray traversal as well as endpoints. Optionally fuse repeated scans using clamped log-odds occupancy updates; inflate obstacles and check free-space connectivity. This is more principled for unknown space, but likely overkill for the initial two-day implementation.

## Proposed baseline: path tracking with LiDAR corridor centering

**Design proposal, not implemented behavior.** When the nominal path is unobstructed, use pure pursuit as the primary controller and a *confidence-weighted lateral centering correction* from LiDAR. This can still help when an obstacle is off to the side: the robot should not abandon the planned graph edge just because a side return is close.

Given reliable, approximately perpendicular left/right corridor clearances `d_L` and `d_R` measured in an **edge-aligned frame**, define:

$$e_c=\frac{d_L-d_R}{2}$$

A positive `e_c` indicates more room to the left; a positive angular correction should turn the robot left, assuming ROS positive yaw is counterclockwise. This sign must be checked in Gazebo. Do **not** directly assume raw `/scan` indices 90 and 270 are perpendicular side distances; compute ray angles from `angle_min` and `angle_increment`, transform points into the segment frame, and reject unsuitable boundaries.

The optional centering PID produces

$$\omega_c=K_p e_c+K_i\int e_c\,dt+K_d\frac{de_c}{dt}$$

and the tracking controller produces `ω_PP` (for conventional pure pursuit, `ω_PP = 2v sin(α)/L_d`). Proposed combined command:

$$\omega_{cmd}=\mathrm{clip}(\omega_{PP}+\lambda_{confidence}\omega_c,-\omega_{max},\omega_{max})$$

Start with `K_i = K_d = 0`; tune only if repeated runs justify additional terms. Unlike a weighted average of two angular velocities, the additive correction leaves normal pure-pursuit steering intact when the centering correction is zero. `λ_confidence` should be **zero** when side-wall geometry is missing, ambiguous, asymmetric due to a junction, or unreliable, and reduced near graph-node turns. Reset or freeze PID integral/derivative state when the correction is disabled; clamp velocity and apply independent immediate-collision checks.

**Obstacles directly on the intended motion are a different situation:** choose a candidate opening, shift the normal pure-pursuit lookahead target toward it in `obstacle_avoided_target()`, and validate the **predicted short commanded motion** before moving. A midpoint between two returns is a candidate only, not a guarantee of clearance. `is_passable()` must distinguish genuinely blocked corridors from temporarily unknown ones. Do not let blending override an emergency stop.

### Interface for independent work

- **Ki Chun (perception):** estimate `d_L`, `d_R`, their validity/confidence, and any local obstacle/trajectory clearance; provide a small shared helper or state object.
- **Shaan (control):** track the chosen target with pure pursuit, optionally add the centering PID correction when valid, and enforce stopping/velocity constraints. Keep the provided `turtlebot_control()` signature; a helper or cached perception result can supply lateral error and confidence.
- **Nhan (planning):** retain graph/state-machine ownership; turning and segment transitions must signal when centering is unreliable so it does not fight the next waypoint.

For standalone controller tests, **disable LiDAR centering (`λ_confidence = 0`)** so the mocked obstacle-free paths validate pure pursuit independently. Later repeat with centering enabled in the obstacle-free world to detect heading/offset oscillation and corner interference. See [controller debug](controller-debug.md) and [navigator tests](testing.md).

## Share perception with target selection

`is_passable()` and `obstacle_avoided_target()` should share the same observed geometry, gap candidates, and immediate safety assessment. **Stage 1** screens demonstrably too-narrow unavoidable bottlenecks. **Stage 2** proposes an obstacle-shifted pure-pursuit lookahead target, predicts the ensuing *short* controller motion, and checks its swept-footprint clearance and stopping feasibility. The selected target is passed to `turtlebot_control()`; the existing `publish_cmd()` sends `/cmd_vel`. A validated local motion is not evidence that the complete graph edge is passable. Do not add a separate velocity publisher to perception.

The boolean `is_passable()` cannot directly encode **clear / unknown / blocked**. Keep uncertainty as internal state or diagnostics. Returning true in an unknown situation must not cause motion into unseen or insufficiently cleared space: stop, rotate, or approach only when the immediate motion is known safe. Confirm blocked-edge decisions with adequate evidence rather than a single transient scan.

## Ground truth versus runtime perception

Use the [manual edge survey](edge-ground-truth.md) to establish evidence-based `P` / `B` / `U` labels independently of the algorithm. These observations are **evaluation data**, never a pre-recorded blocked-edge table for the submitted navigator. Evaluate false-blocked and false-passable decisions only on confirmed labels; investigate unknown edges separately. The four focused cases in [navigator tests](testing.md#four-priority-navigator-tests) are a smaller regression suite, not the whole maze.

## Incremental development recommendation

1. Establish A* path assembly and pure-pursuit tracking in the **obstacle-free** world.
2. Visualize single-scan LiDAR points and check immediate clearance.
3. For selected edges, run the **two-stage** check: locate/validate cross-sectional bottlenecks, then evaluate obstacle-shifted lookahead targets and their predicted short motions; ensure stopping remains safe.
4. Add a bounded per-edge point buffer only if single-scan occlusion causes real problems.
5. Add occupancy-grid mapping only if unknown-space representation is proven necessary.

See [four edge-specific trials](testing.md#edge-specific-perception-and-clearance-tests), [debug views](debugging.md), and [two-day schedule](development.md).
