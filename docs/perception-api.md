# Perception Module v1.1 — API and navigator integration

**Documentation:** [README](../README.md) · [Setup](setup.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Perception strategy](perception-strategy.md) · **Perception API** · [Task ownership](tasks.md) · [Development plan](development.md) · [Ground-truth survey](edge-ground-truth.md) · [Controller debug](controller-debug.md) · [Navigator tests](testing.md) · [Offline tests](navigation-offline-tests.md) · [RViz & debugging](debugging.md)

---

**Status:** Tested in a standalone LiDAR/OpenCV explorer; **not integrated into `navigator.py`**. The source is `vmc2_project/src/perception.py`, which is independent of ROS. `tools/lidar_explorer.py` adapts ROS TF/LaserScan observations into the API. This document describes implemented behavior, not guarantees of physical safety.

**For the algorithm and motivation, start with [the planning strategy](perception-strategy.md).** This document specifies API calls, types and integration requirements.

## Coordinate contract

- `analyze_edge(points_world, edge_start, edge_end, ...)` accepts an `N×2` NumPy array in a fixed XY frame and a **directed** edge in the **same** frame. Preserve timestamp-matched TF when producing scan points.
- Edge coordinates are `(s,t)` in meters, where `s` advances from `start` to `end`, and positive `t` lies to the **left** of the directed edge. In the explorer, positive `t` is rendered left of the blue centerline.
- `robot_position` and `observation.start/end` must use that same fixed frame. `robot_yaw` is **radians** in that frame, *not* the yaw relative to the edge.
- `observation.obstacle_points_st` contains **observed obstacle returns**. No ray-traced free-space visibility is available. A curve clearing these returns is **not a certificate of free space**.

## Recommended high-level integration

```python
from perception import analyze_edge, plan_motion

observation = analyze_edge(points_world, edge_start, edge_end)
report = plan_motion(
    observation, robot_position_xy, robot_yaw=robot_yaw,
    robot_radius=0.089, safety_margin=0.03, max_curvature=8.0,
)

if report.plan is None:
    # Stop / reobserve / recover. Do NOT automatically block the graph edge.
    ...
elif report.plan.requires_rotation:
    # STOP: proposed departure_yaw is in world/odom radians.
    # Translate along any FIRST leg only when currently appropriate;
    # rotate at rotation_st, RE-SCAN, then revalidate before translation.
    ...
else:
    # Use a short lookahead from the first proposed segment; controller
    # must independently enforce local stopping and tracking constraints.
    path_st = report.plan.segments_st[0]
    ...
```

`plan_motion(...) -> PlanningReport` is the recommended **single entry point**; callers should not independently copy the ordered fallback tree. It returns:

- `report.mode`: selected *strategy* string;
- `report.plan`: `MotionPlan` or `None` when no motion was proposed;
- `report.search`: ordinary current-heading `TrajectorySearchResult` with `feasible` and `rejected` candidates, which **excludes** proposed post-rotation trajectories;
- `report.diagnostics`: stage-specific results and recovery diagnostics.

`MotionPlan` contains `segments_st` (one or two **separate** translation paths), `rotation_st` (optional), `departure_yaw` (optional, fixed-frame radians), `requires_rescan`, optional observed `clearance`, and optional `candidate`. If rotation is proposed, the prospective continuation is **hypothetical**, not permission to drive. Do not concatenate separate segments and send them to pure pursuit as a single path.

## Detected-opening passability policy

`assess_passability(observation, robot_position, *, robot_radius=0.089, safety_margin=0.03) -> PassabilityResult` examines cross-sections not entirely behind the robot. At each eligible slice (at least two retained obstacle clusters and one or more internal gaps), it calculates the widest **detected internal** opening. If this width is **strictly smaller** than `2 * (robot_radius + safety_margin)`, it returns `Passability.BLOCKED` immediately. A wide opening at another slice does not undo this early rejection. At equality, the slice does not trigger rejection.

With the standard defaults (`robot_radius=0.089 m`, `safety_margin=0.03 m`), the threshold is **0.238 m**. Callers may override the margin explicitly for both assessment and planning (for example, `0.04 m` gives **0.258 m**). Insufficient clusters, no internal gap, or all detected gaps wide enough yields `UNKNOWN`, never `CLEAR`.

**Interpretation:** `BLOCKED` means *no sufficiently wide internal opening was detected at a remaining sampled cross-section*. This is an intentionally aggressive **operational early rejection**, not a proof that the physical corridor is impassable: unobserved routes outside the retained obstacle clusters are not represented. Avoid permanently deleting a graph edge from a single scan without an explicit reobservation policy. No trusted corridor-boundary argument is accepted.

## Other useful public functions

| API | Purpose | Output / limitation |
|---|---|---|
| `analyze_edge` | Cluster LiDAR slice returns into lateral obstacles and candidate gaps | `EdgeObservation`; an unseen gap is not inferred free |
| `points_to_edge_frame`, `edge_to_world` | Transform points / waypoint coordinates | Use the same edge and fixed frame |
| `detect_bottlenecks` | Flag potentially narrow internal gaps | Candidates only, not evidence whole edge is blocked |
| `assess_passability` | Detected-opening early rejection | `PassabilityResult`; `BLOCKED` on any remaining slice whose detected internal gaps are all narrower than `2(r+m)`; otherwise `UNKNOWN` |
| `find_local_passages`, `select_next_local_passage` | Group adjacent overlapping usable gaps | Observation-dependent; does not prove connected free space |
| `OpeningProgressTracker` | Debug estimated entering, traversing, cleared, off-path states across scans | Stateful; reset on edge/parameter changes |
| `search_with_fallback` | Lower-level ordered translation search (used internally by `plan_motion`) | `(TrajectorySearchResult, mode)`; rotation represented only in diagnostics |
| `search_direct_destination`, `search_smooth_destination` | Inspect the two highest-priority destination searches | `TrajectorySearchResult` |
| `search_*` lower-level strategies | Experimental local trajectory and rotation proposal generators | For unit testing or diagnostics; prefer `plan_motion` for callers |
| `choose_avoidance_target` | Lower-level convenience lookahead helper | Does **not** encode stopped rotations or full motion phases; avoid as sole controller interface |

### Integration boundaries

1. **Ki Chun / perception:** build observations, choose a motion proposal, convert edge-frame path samples to fixed-frame waypoints with `edge_to_world`, and surface `UNKNOWN` rather than inventing passability.
2. **Shaan / control:** enforce stop/rotate/translate phases, follow a short revalidated lookahead, publish `/cmd_vel`, and enforce collision stop independently. A proposed path is not a velocity command.
3. **Nhan / graph:** own directed edge choice, blocked-edge evidence policy, and A* state transitions. A failed local search is **not** automatically a blocked edge.

Do not run the explorer's OpenCV windows within `navigator.py`. `perception.py` uses NumPy only, not `rclpy` or `cv2`. The ROS package installs `src/perception.py` beside `navigator.py` using `CMakeLists.txt`.

## Exactly how many modes?

The planner currently exposes **14 strategy labels** through `plan_motion` / its ordered helpers. These are **not 14 independent robot control states**. They can be grouped into four operational families:

| Operational family | Strategy labels | Count |
|---|---|---:|
| Reach the graph destination | `direct destination`, `smooth destination`, `rotate then destination` | 3 |
| Approach/traverse a local passage | `goal-reached continuation only`, `near-opening traverse only`, `next opening only`, `final approach only`, `single-gap full-edge`, `short approach only`, `two-waypoint full-edge fallback` | 7 |
| Rotation/reposition recovery | `rotate now`, `align first`, `intermediate alignment`, `lateral reposition` | 4 |
| **Total** | | **14** |

Underneath those names, **only three motion-phase types** matter for eventual execution: `TRANSLATE`, `ROTATE_IN_PLACE`, and `STOP/REOBSERVE` (conceptual controller protocol; *not* yet a defined `navigator.py` enum). A plan may contain more than one phase. There are also **three explorer display modes** (`best`, `feasible`, `debug`), unrelated to planner modes. `Passability` has **three statuses** (`CLEAR`, `UNKNOWN`, `BLOCKED`); those are evidence classifications, also unrelated to controller modes.

## Actual priority (summary; see the [strategy walkthrough](perception-strategy.md))

`plan_motion` uses this destination-first priority: direct destination → smooth destination → rotate-then-destination → goal-reached continuation → near-opening traverse → next passage → final approach → single-gap fallback → local approach → two-gap full-edge. If no current-heading proposal exists, additional rotation/reposition options run: rotate-now → hypothetical heading alignment → intermediate alignment → lateral reposition.

Some specialized search helpers intentionally overlap. They remain available for diagnostics and regression testing, while `plan_motion` owns strategy selection so callers do not duplicate orchestration. Consolidate further only after confirming behavioral equivalence in Gazebo.

## Limits and required safeguards

- All reported clearance is relative to sampled **observed returns** and assumes a circular robot footprint. Verify the robot collision envelope and margins. Do not equate a geometric candidate with validated motion.
- `max_curvature` approximates tracking constraints; in-place rotation is a *separate* maneuver.
- A frame mismatch, timestamp mismatch, stale scan, or changing occlusion can invalidate paths.
- Preserve a pending rotation through controller execution and **replan after re-scan**. A debug visualization is not a motion authorization.
- Initial test parameters: radius `0.089 m`, margin `0.03 m`, maximum curvature `8 m⁻¹`. These are not hardware safety guarantees.

## Validation

From the extracted repository root, with ROS Python paths available:

```bash
PYTHONPATH="/usr/lib/python3/dist-packages:$PWD/vmc2_project/src:$PWD/tools${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m unittest discover -s vmc2_project/test -p 'test_*.py' -v
```

The refactor is validated by unit tests and Python compilation. **An integrated Gazebo run has been reported (three checkpoints in approximately 2:30), but the result is not a full acceptance test.**
