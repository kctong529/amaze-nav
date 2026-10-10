# amaze-nav

**LiDAR-based perception, obstacle avoidance, and autonomous TurtleBot maze navigation with ROS 2.**

Course project for **MEC-E5012 Vehicle Mechatronics: Control**, Aalto University.

The TurtleBot must navigate a **5 × 5 graph maze** without collisions. Node positions are known, but some passages are blocked or require steering around obstacles. We combine LiDAR perception, local control, and **A*** replanning; the implementation must generalize to different obstacle configurations, not rely on a prerecorded obstacle map.

## At a glance

| Component | Role |
|---|---|
| **Perception** | Interpret `/scan` to assess clearance and passage traversability |
| **Avoidance & control** | Select an obstacle-aware lookahead target and command `/cmd_vel` using pure pursuit; optionally add LiDAR corridor-centering PID |
| **A\* planning** | Follow the grid and replan when an edge is blocked |

The provided navigator uses `FOLLOW → BACKTRACK → REPLAN → FOLLOW` for blocked passages and `DONE` on arrival. **Status: under development.** Planned methods and ownership are proposals, not claims of implemented behavior.

## Documentation

- [Provided framework & function guide](docs/framework.md) — what's already implemented versus TODOs, modes, ROS interfaces.
- [Perception & navigation methodology](docs/methodology.md) — two-stage passability (cross-sectional gap screening + short-horizon motion clearance), pure pursuit, optional LiDAR centering, and alternatives.
- [Team responsibilities](docs/tasks.md) — suggested owners, deliverables, definitions of done, interface agreements.
- [Two-day implementation plan](docs/development.md) — integration sequence, priorities, risks, lightweight collaboration.
- [Edge passability ground-truth survey](docs/edge-ground-truth.md) — manually investigate all 40 edges and record evidence before evaluating perception.
- [Controller-only debug tests](docs/controller-debug.md) — reversible `straight`, `turn`, and `loop` mocked paths in the obstacle-free world.
- [Navigator acceptance tests](docs/testing.md) — repeatable teleport tests, four priority edges, integration checks.
- [RViz & debugging](docs/debugging.md) — available topics, optional markers, and data to monitor.

## Quick start

The course provides `vmc2_project`, `project_track`, and `project_track_no_obs`. Extract the world models into `~/.gazebo/models/`, place `vmc2_project` in a ROS 2 workspace, then build and source the workspace:

```bash
colcon build --packages-select vmc2_project
source install/setup.bash
ros2 launch vmc2_project project.launch.py
```

Start with the **obstacle-free** world and use the supplied launch setup to select the environment. See [the manual survey](docs/edge-ground-truth.md) for teleoperated exploration or [navigator tests](docs/testing.md) for controlled acceptance tests and the requirement to update `self.start_pos` after teleporting.

## Targets

Navigate without hitting walls, aim for the **2 min 30 s simulation-time** full-run target, and achieve repeatable performance. Submit the course package and video evidence as specified in the assignment; see [the test guide](docs/testing.md).

## Team

- Ki Chun Tong
- Shaan Salian
- Vo Dinh Nhan

## Acknowledgements

Developed as part of **MEC-E5012 Vehicle Mechatronics: Control** at Aalto University. The initial ROS 2 navigation framework and Gazebo maze models are course materials.
