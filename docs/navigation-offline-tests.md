# Offline navigation tests

**Documentation:** [README](../README.md) · [Setup](setup.md) · [Framework](framework.md) · [Methodology](methodology.md) · [Perception strategy](perception-strategy.md) · [Perception API](perception-api.md) · [Task ownership](tasks.md) · [Development plan](development.md) · [Ground-truth survey](edge-ground-truth.md) · [Controller debug](controller-debug.md) · [Navigator tests](testing.md) · **Offline tests** · [RViz & debugging](debugging.md)

---

**Scope:** The navigation tests exercise the course navigator's grid planning and recovery state machine independently of perception, velocity control, ROS, and Gazebo. They do **not** establish physical collision avoidance or successful sensor-based maze traversal. No modifications to the template `navigator.py` are required.

## Test files

| File | Responsibility |
|---|---|
| `vmc2_project/test/test_navigation.py` | Unit tests for graph construction, A* path selection, `plan_from`, and individual control-step transitions |
| `vmc2_project/test/test_navigation_integration.py` | Five deterministic multi-step simulations, corresponding to Nhan's original scenarios A–E |
| `vmc2_project/test/navigation_sim.py` | Reusable ROS stand-ins, fake obstacle decisions, simple controller, simulated robot dynamics, and result collection; this is **not** itself a discovered test module |

## Run

From the repository root (`$HOME/shared/amaze-nav`; run `cd "$HOME/shared/amaze-nav"` first):

```bash
python3 -m unittest discover -s vmc2_project/test -p 'test_navigation*.py' -v
```

To run only the integration scenarios:

```bash
python3 -m unittest discover -s vmc2_project/test -p 'test_navigation_integration.py' -v
```

Tests require Python 3 and NumPy; importing `navigator.py` also requires an OpenCV (`cv2`) module in the unit-test setup. The integration helper can substitute a minimal `cv2` module when unavailable. Neither test suite needs a running ROS daemon or Gazebo.

## Scenarios A–E

| Scenario | Start | Simulated obstruction | Expected behavior |
|---|---|---|---|
| A | Maze entrance `(0, 0)` | None | Reach goal without backtracking |
| B | Checkpoint `(3.560, -4.272)` | One graph edge | Return to previous node, remove that edge, replan and finish |
| C | Maze entrance | Nine test-fixture blocked edges | Discover multiple blocked edges as encountered and reach the exit |
| D | Maze entrance | Same fixture, smaller detection distance | Recover when a blockage is noticed farther into an edge |
| E | Checkpoint `(4.984, -4.272)` | Same fixture | Start from a different graph node and reach the exit |

The hardcoded blocked-edge list lives **only in the offline test helper**; it is not perception ground truth available to the robot. Fake passability returns a boolean based on distance to a configured edge midpoint. The actual Perception Module v1.1 has different semantics (`BLOCKED`/`UNKNOWN`) and must be tested separately.

## What the checks mean

The integration harness runs `control_step()` repeatedly with a fixed 0.05 s timestep and a simple unicycle model. It records mode transitions, removed graph edges, visited nodes, and maximum lateral offset from ideal graph corridors. The assertions verify arrival at `DONE`, legal and nonduplicate edge removal, consistent replanning origins, limited simulated centerline deviation, and absence of exceptions.

The controller is a *test double*, not the team's TurtleBot controller. The 0.25 m offset check is a geometric sanity check under idealized kinematics, **not** a collision-clearance test. Timing is simulated and should not be compared directly with Gazebo performance.

For real LiDAR/perception behavior and safe obstacle traversal, follow the [navigator acceptance tests](testing.md), [perception API](perception-api.md), and [controller debug tests](controller-debug.md).
