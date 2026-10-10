# Two-day implementation and integration plan

**Documentation:** [README](../README.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Task ownership](tasks.md) · [Two-day plan](development.md) · [Ground-truth survey](edge-ground-truth.md) · [Controller debug](controller-debug.md) · [Navigator tests](testing.md) · [RViz & diagnostics](debugging.md)

---

## Suggested Implementation Order

| Phase | Primary lead | Supporting work | Shared checkpoint |
|---|---|---|---|
| **1. Skeleton and baseline** | Shaan: [controller debug](controller-debug.md) (`straight`, `turn`, then `loop`) | Vo Dinh Nhan: complete `plan_from()`; Ki Chun: LiDAR visualization | Robot follows nodes on obstacle-free track; scans visible |
| **2. First obstacle** | Ki Chun: two-stage passability (cross-sectional bottleneck screening → short-horizon motion clearance), plus side-clearance/confidence estimate | Shaan: gate LiDAR centering + avoidance target; Vo Dinh Nhan: blocked-edge transitions | Robot either passes a simple obstacle safely or replans correctly |
| **3. Integration** | Vo Dinh Nhan: state-machine tests | Ki Chun + Shaan: perception/controller refinements | No contradictory perception/steering outcomes |
| **4. Robustness** | **All three** | Repeat trials, tune margins, investigate occlusion and narrow passages | Document success rate, collision failures, and timing |
| **5. Submission** | **All three** | Review code, record Gazebo runs, prepare gala slides | Reproducible result from the submitted code |

Parallel work is useful, but **integrate the first working baseline on Day 1**. Do not wait for each workstream to be complete before testing the full navigation loop.

## Two-Day Collaboration Plan

**Keep coordination lightweight.** We do not need GitHub issues or a formal pull-request workflow for a two-day project. The ownership table in §2 is the task list; coordinate directly in chat and at the simulator.

| Time | Ki Chun — perception | Shaan — control | Vo Dinh Nhan — planning/integration | Joint milestone |
|---|---|---|---|---|
| **Day 1, first half** | Inspect LiDAR and survey selected edges manually; begin [ground truth](edge-ground-truth.md) | Run [controller-only `straight` and `turn` tests](controller-debug.md) in the obstacle-free world | Complete `plan_from()` and basic waypoint/state behavior | One edge traversed; scans visible |
| **Day 1, second half** | Test first two clearance edges in [four selected edge tests](testing.md#edge-specific-perception-and-clearance-tests) | Run the full [18-waypoint `loop` controller test](controller-debug.md), then add obstacle-aware target selection | Wire up blocked-edge transitions and run the first integration test | One obstacle case tested end-to-end |
| **Day 2, first half** | Compare detector on confirmed survey edges, prioritizing the final challenge | Tune avoidance and stopping behavior | Fix state transitions and repeatability failures | Most difficult segment tested |
| **Day 2, second half** | Support tuning and capture scan/clearance evidence | Support full run and capture control evidence | Coordinate final full-maze tests | Stable submission, demo recording, and brief presentation |

**Ground-truth timebox:** Survey as many of the 40 graph edges as practical, but do not sacrifice integration or the submission deadline to force a P/B label. Mark unresolved edges `U` and keep the label inventory in [the ground-truth protocol](edge-ground-truth.md). For full task acceptance, use [navigator tests](testing.md).

**Practical working agreement:**

- Spend 10 minutes agreeing on shared data formats and who edits which parts of `navigator.py`.
- Work in separate branches or separate copies if needed; have **one person integrate changes** into the main file at regular checkpoints.
- Make small commits when a function or test is working, but do not require issues, PR reviews, or formal approvals.
- After each smoke test, share just: **edge, starting pose, result, and one important observation** (plus a screenshot when useful).
- Prioritize a working, collision-free baseline over advanced mapping or polished tooling. Stop adding features when they threaten the final integration window.

## Key Risks and Responsibilities

| Risk | First owner | Mitigation |
|---|---|---|
| LiDAR detects a nearby obstacle but incorrectly marks an otherwise passable edge blocked | Perception | Differentiate obstacle proximity from corridor reachability; test edge cases |
| Steering heads toward a gap that lacks clearance | Perception + control | Shared gap/target geometry, predicted pure-pursuit motion checks, and safe stopping |
| Robot turns incorrectly at grid nodes or oscillates near a waypoint | Control | Test on obstacle-free track first and tune tolerances |
| Backtracking itself causes a collision | Control + navigation | Apply clearance-aware control during recovery, not just `FOLLOW` |
| Incorrect edge removal or path bookkeeping | Navigation | Small deterministic graph/state tests and visual inspection |
| Perception returns “safe” merely because space is unobserved | All | Explicit conservative policy for uncertainty and stopping |
| Time spent building unnecessary mapping infrastructure | All | Validate simple approaches first; add persistent mapping only if justified |

## Immediate Next Actions

1. **Confirm or swap the proposed leads.**
2. Start three parallel tasks directly: [mocked-path obstacle-free controller tests](controller-debug.md), `plan_from()`, and LiDAR visualization.
3. Agree on coordinate frames and a tiny perception-control interface (`d_L`, `d_R`, confidence/validity, clearance). Tune pure pursuit first with centering disabled, then integrate the lateral correction.
4. Combine the first working changes on Day 1 and demonstrate one successful edge traversal before expanding complexity.
5. Keep a short shared log of experiments, parameter values, failures, and improvements; this will also help with the project gala and optional effort documentation.

---

**Overall principle:** divide ownership, not responsibility. Perception, avoidance, and the state machine are tightly coupled, so the final system must be designed and tested together.

For direct testing, see [teleport commands](testing.md); for markers and observations, see [RViz guide](debugging.md).

## Local avoidance scope decision

Implement **shifted obstacle-aware lookahead + predicted short-motion clearance** first. Ki Chun screens gaps and supplies geometry; Shaan chooses and tracks the shifted target; both validate the near-term swept footprint. Repeat as scans update. A complete precomputed local curve to the next graph node is **not** required in the initial two-day scope; revisit it only if focused tests show that the simple strategy cannot pass a specific opening.
