# amaze-nav

**Documentation:** **README** · [Setup](docs/setup.md) · [Framework](docs/framework.md) · [Methodology](docs/methodology.md) · [Perception strategy](docs/perception-strategy.md) · [Perception API](docs/perception-api.md) · [Task ownership](docs/tasks.md) · [Development plan](docs/development.md) · [Ground-truth survey](docs/edge-ground-truth.md) · [Controller debug](docs/controller-debug.md) · [Navigator tests](docs/testing.md) · [Offline tests](docs/navigation-offline-tests.md) · [RViz & debugging](docs/debugging.md)

---

**Perception Module v1.1** — standalone LiDAR perception and trajectory planning baseline. This is the component version, **not** a release version for the overall amaze-nav project.

**LiDAR-based perception, obstacle avoidance, and autonomous TurtleBot maze navigation with ROS 2.**

Course project for **MEC-E5012 Vehicle Mechatronics: Control**, Aalto University.

The TurtleBot must navigate a **5 × 5 graph maze** without collisions. Node positions are known, but some passages are blocked or require steering around obstacles. We combine LiDAR perception, local control, and **A*** replanning; the implementation must generalize to different obstacle configurations, not rely on a prerecorded obstacle map.

## At a glance

| Component | Role |
|---|---|
| **Perception** | Interpret `/scan` to assess clearance and passage traversability |
| **Avoidance & control** | Select an obstacle-aware lookahead target and command `/cmd_vel` using pure pursuit; optionally add LiDAR corridor-centering PID |
| **A\* planning** | Follow the grid and replan when an edge is blocked |

The provided navigator uses `FOLLOW → BACKTRACK → REPLAN → FOLLOW` for blocked passages and `DONE` on arrival. **Status:** Standalone LiDAR explorer and ROS-independent perception/trajectory planner implemented and unit-tested; integration with `navigator.py`, controller execution, and autonomous maze navigation remains unfinished.

## Documentation

- [Passability detection audit](docs/perception-passability-audit.md) — known false-positive/false-negative risks, integration guidance, and test gaps.
- [Perception Module v1.1 — planning strategy](docs/perception-strategy.md) — start here: destination-first decisions, LiDAR gap detection, trajectory rejection, opening fallbacks, recovery, and replanning.
- [Perception Module v1.1 — API and integration](docs/perception-api.md) — exact functions, motion plans, 14 strategy labels, and controller contract.
- [Environment setup](docs/setup.md) — clone the repository, configure ROS 2 and Gazebo model paths, build, launch, and troubleshoot.
- [Provided framework & function guide](docs/framework.md) — what's already implemented versus TODOs, modes, ROS interfaces.
- [Perception & navigation methodology](docs/methodology.md) — two-stage passability (cross-sectional gap screening + short-horizon motion clearance), pure pursuit, optional LiDAR centering, and alternatives.
- [Team responsibilities](docs/tasks.md) — suggested owners, deliverables, definitions of done, interface agreements.
- [Two-day implementation plan](docs/development.md) — integration sequence, priorities, risks, lightweight collaboration.
- [Edge passability ground-truth survey](docs/edge-ground-truth.md) — manually investigate all 40 edges and record evidence before evaluating perception.
- [Controller-only debug tests](docs/controller-debug.md) — reversible `straight`, `turn`, and `loop` mocked paths in the obstacle-free world.
- [Offline navigation tests](docs/navigation-offline-tests.md) — ROS-free unit and state-machine integration tests (scenarios A–E).
- [Navigator acceptance tests](docs/testing.md) — repeatable teleport tests, four priority edges, integration checks.
- [RViz & debugging](docs/debugging.md) — available topics, optional markers, and data to monitor.

## Quick start

The repository already contains the extracted `vmc2_project/`, `project_track/`, and `project_track_no_obs/` directories. **No separate archive download or extraction is required.** In the course's Ubuntu / ROS 2 Humble environment, configure your workspace and Gazebo model directory using [Environment setup](docs/setup.md) (paths depend on your machine).

After following that guide to install the local ROS package and models, set `GAZEBO_MODEL_PATH`, build/source the workspace, and launch:

```bash
cd "$ROS_WS"
colcon build --symlink-install --packages-select vmc2_project
source "$ROS_WS/install/setup.bash"
ros2 launch vmc2_project project.launch.py
```

Use the [ground-truth survey](docs/edge-ground-truth.md) for teleoperation or [navigator tests](docs/testing.md) for repeatable acceptance tests. Use the obstacle-free model for initial controller tests; consult the launch file for how that world is selected.

## Targets

Navigate without hitting walls, aim for the **2 min 30 s simulation-time** full-run target, and achieve repeatable performance. Submit the course package and video evidence as specified in the assignment; see [the test guide](docs/testing.md).

## Team

- Ki Chun Tong
- Shaan Salian
- Vo Dinh Nhan

## Acknowledgements

Developed as part of **MEC-E5012 Vehicle Mechatronics: Control** at Aalto University. The initial ROS 2 navigation framework and Gazebo maze models are course materials.
