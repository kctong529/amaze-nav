# Perception, obstacle avoidance, and navigation methodology

**Documentation:** [README](../README.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Task ownership](tasks.md) · [Two-day plan](development.md) · [Ground-truth survey](edge-ground-truth.md) · [Navigator tests](testing.md) · [RViz & diagnostics](debugging.md)

---

**Status: design discussion, not a finalized algorithm.** The 5 × 5 graph gives candidate links, not a prerecorded obstacle map. The challenge is to distinguish a truly blocked edge from a passage with a safe path around obstacles.

## Why front-range thresholds are insufficient

The nearest return directly ahead can indicate an obstacle but cannot distinguish a fully blocked corridor from one with lateral clearance. Angled, staggered, and occluding obstacles complicate identifying an actual bottleneck. A pair of ranges separated by angle Δθ obeys the cosine rule:

$$d=\sqrt{r_1^2+r_2^2-2r_1r_2\cos(\Delta\theta)}$$

But that measures the distance between **two returns**, not automatically the minimum navigable gap. What matters is whether a continuous collision-free trajectory exists through the corridor.

## LiDAR geometry and frame conversion

A valid range-angle pair `(r, θ)` becomes a sensor-frame point:

$$x=r\cos\theta,\quad y=r\sin\theta$$

For an aligned sensor at the robot origin and robot world pose `(x_r, y_r, ψ)`, the corresponding world point is:

$$x_w=x_r+r\cos(\psi+\theta),\quad y_w=y_r+r\sin(\psi+\theta)$$

Account for an actual sensor offset/rotation, and match readings with appropriately timed pose estimates before accumulating scans. The segment world heading `seg_dir_yaw` differs from the relative scan angle; for an aligned sensor, the segment-relative scan direction is `angle_wrap(seg_dir_yaw - self.yaw)`.

## Minimum clearance and obstacle inflation

The assignment states a **0.178 m** TurtleBot width. With an *illustrative* 0.04 m margin on **each side**:

$$W_{required}=0.178+2(0.04)=0.258\text{ m}$$

For a circular-footprint approximation:

$$R_{effective}=0.178/2+0.04=0.129\text{ m}$$

These are **tuning examples**, not prescribed safe values; verify the actual footprint and realistic control error. Inflate obstacle geometry by the effective radius, then evaluate the robot centerline against inflated obstacles.

## Candidate methods

| Method | Strength | Limitation | Initial priority |
|---|---|---|---|
| Simple front scan threshold | Fast and easy | Cannot tell bypassable from blocked | Safety check only |
| Single-scan gap detection | Low overhead | Occlusion and unknown space | Start here |
| Cross-sectional clearance | Exposes bottlenecks | Boundary matching, connectedness difficult | Useful diagnostic |
| Per-edge accumulated obstacle points | Retains prior observations, moderate overhead | Unknown space remains implicit | Add if needed |
| Occupancy grid with local pathfinding | Represents observed free, occupied, unknown | Mapping and planning complexity | Defer |

### Cross-sectional minimum gap

Express points in an edge-aligned frame with longitudinal `s` and lateral `t`, relative to origin `(x₀,y₀)` and edge yaw `ψ`:

$$s=(x-x_0)\cos\psi+(y-y_0)\sin\psi$$
$$t=-(x-x_0)\sin\psi+(y-y_0)\cos\psi$$

At a cross-section `s`, estimate relevant left/right boundaries and gap width `W(s)=t_left(s)-t_right(s)`. The minimum **observed** width along the edge is `min W(s)`. But visible returns may be incomplete, and sufficient width in individual slices does not guarantee a feasible continuous trajectory.

### Per-edge accumulated point clouds

Optionally start a fresh, **world-fixed** obstacle-point buffer when committing to a new edge; convert incoming readings into that frame and downsample nearby points. Preserve observations through pauses and rotations and reset on the next traversal. Do not reset solely because the nearest geometric segment changes during avoidance. Past points help with occlusion but a lack of points in an unseen region is **not** evidence it is free.

### Occupancy grid if experiments demand it

An edge-local grid tracks **occupied**, **observed free**, and **unknown** cells, using ray traversal as well as endpoints. Optionally fuse repeated scans using clamped log-odds occupancy updates; inflate obstacles and check free-space connectivity. This is more principled for unknown space, but likely overkill for the initial two-day implementation.

## Share perception with target selection

`is_passable()` and `obstacle_avoided_target()` should not make contradictory decisions. A possible shared helper generates candidate paths (centerline, left and right offsets, or curves), estimates clearance to observed obstacles, tests motion feasibility and corridor bounds, and chooses a near-term lookahead point. An isolated clear point is not enough; a safe route must be connected and trackable.

The boolean `is_passable()` cannot directly encode **clear / unknown / blocked**. Keep uncertainty as internal state or diagnostics. Returning true in an unknown situation must not cause motion into unseen or insufficiently cleared space: stop, rotate, or approach only when the immediate motion is known safe. Confirm blocked-edge decisions with adequate evidence rather than a single transient scan.

## Ground truth versus runtime perception

Use the [manual edge survey](edge-ground-truth.md) to establish evidence-based `P` / `B` / `U` labels independently of the algorithm. These observations are **evaluation data**, never a pre-recorded blocked-edge table for the submitted navigator. Evaluate false-blocked and false-passable decisions only on confirmed labels; investigate unknown edges separately. The four focused cases in [navigator tests](testing.md#four-priority-navigator-tests) are a smaller regression suite, not the whole maze.

## Incremental development recommendation

1. Establish A* path assembly and pure-pursuit tracking in the **obstacle-free** world.
2. Visualize single-scan LiDAR points and check immediate clearance.
3. Test obstacle-aware targets on the selected edges and ensure stops remain safe.
4. Add a bounded per-edge point buffer only if single-scan occlusion causes real problems.
5. Add occupancy-grid mapping only if unknown-space representation is proven necessary.

See [four edge-specific trials](testing.md#edge-specific-perception-and-clearance-tests), [debug views](debugging.md), and [two-day schedule](development.md).
