# amaze-nav

**LiDAR-based perception, obstacle avoidance, and autonomous maze navigation with TurtleBot and ROS 2.**

amaze-nav is a robotics project developed for **MEC-E5012 Vehicle Mechatronics: Control** at Aalto University.

## Team Members

- Ki Chun Tong
- Shaan Salian
- Vo Dinh Nhan

The goal is to navigate a TurtleBot through a maze containing obstacles of varying shapes and configurations, without colliding with walls or other obstacles.

The project combines three core components:

- **Perception:** Interpret LiDAR measurements to determine whether a passage is traversable.
- **Motion control:** Follow planned paths while steering around obstacles and maintaining safe clearance.
- **Path planning:** Use A* to find routes through the maze and replan when a passage is blocked.

## The Challenge

The maze is represented as a **5 × 5 grid graph**, with nodes connected by potential passages. While the node coordinates are known, the traversability of each edge is not.

Some passages are unobstructed, others require navigating around obstacles, and some are completely blocked.

The robot must distinguish between these cases using its onboard LiDAR sensor.

A key challenge is that detecting an obstacle directly ahead does not necessarily mean the passage is blocked. The robot must evaluate whether sufficient space remains to navigate around it, taking its physical dimensions into account.

The implementation should generalize to different obstacle configurations rather than relying on a prerecorded maze map or hard-coded obstacle positions.

## Navigation Architecture

The provided ROS 2 navigation framework uses four operating modes:

| Mode | Description |
|---|---|
| `FOLLOW` | Follow the current planned route while evaluating obstacles |
| `BACKTRACK` | Return to the previous node when the current passage is blocked |
| `REPLAN` | Remove the blocked edge and compute an alternative route |
| `DONE` | Stop when the destination is reached |

The navigation pipeline follows this general structure:

```text
              Odometry + LiDAR
                     |
                     v
             Global A* Planning
                     |
                     v
              FOLLOW Mode
                     |
              Is Passable?
                /       \
              YES        NO
               |          |
               v          v
        Obstacle-aware  BACKTRACK
        Lookahead           |
               |           v
               v         REPLAN
        Motion Control      |
               |           |
               v           |
         Next Waypoint <----+
               |
               v
          Goal Reached?
            /      \
           NO      YES
           |        |
         FOLLOW    DONE
```

## Understanding the Provided Navigation Framework

The course provides a partially implemented `navigator.py` containing the grid representation, A* implementation, ROS 2 interfaces, visualization utilities, and the main navigation state machine.

Most of the supporting infrastructure is already available. Our primary responsibility is to complete the missing navigation logic and integrate perception, control, and path planning.

### 1. Grid and Path Planning

| Function / Class | Purpose | Status |
|---|---|---|
| `GridGraph` | Represents the 5 × 5 grid, node coordinates, and neighboring connections | Provided |
| `GridGraph.remove_edge()` | Removes an impassable connection from the graph | Provided |
| `astar_path()` | Computes a shortest path through the graph using A* | Implemented |
| `nearest_graph_coord()` | Finds the graph node closest to a given position | Provided |
| `plan_from()` | Converts the robot's current position and A* result into a waypoint list | **TODO** |
| `path_plan()` | Calls `plan_from()` and updates `self.full_path` | Provided |
| `nearest_point_on_polyline()` | Finds the closest path segment to the robot | Provided |
| `lookahead_point()` | Calculates a lookahead point along a path | Provided |

**Important distinction:** `astar_path()` is already implemented. We mainly need to complete `plan_from()` and connect replanning to the existing graph utilities.

### 2. Perception and Obstacle Avoidance

| Function | Purpose | Status |
|---|---|---|
| `scan_cb()` | Receives and stores the latest `/scan` LiDAR message | Provided |
| `odom_cb()` | Updates the robot's position and heading from `/odom` | Provided |
| `is_passable()` | Determines whether the current graph edge remains traversable | **TODO** |
| `obstacle_avoided_target()` | Generates a lookahead target that accounts for obstacles | **TODO** |

These are the main functions involved in our perception methodology.

**`is_passable(seg_dir_yaw)`**

- Input: Current path segment heading in the world frame.
- Output: Boolean indicating whether the current edge should still be considered passable.
- Available data: `self.scan`, `self.x`, `self.y`, and `self.yaw`.
- Proposed implementation: Analyze LiDAR obstacle geometry and determine whether a sufficiently clear route remains through the corridor.

The function is called repeatedly during `FOLLOW` mode, allowing its assessment to change as new observations become available.

**`obstacle_avoided_target(robot_pos, seg_dir_yaw)`**

- Input: Current robot position and path segment heading.
- Output: An `(x, y)` lookahead target in the world frame.
- Proposed implementation: Start with a pure-pursuit target and adjust it laterally to avoid obstacles while remaining within the corridor.

Ideally, both functions should share the same obstacle representation and clearance assessment.

### 3. Motion Control

| Function / Class | Purpose | Status |
|---|---|---|
| `PID` | Optional PID controller with integrator clamping | Provided |
| `turtlebot_control()` | Computes linear and angular velocity commands | **TODO** |
| `publish_cmd()` | Publishes velocities to `/cmd_vel` | Provided |
| `publish_stop()` | Commands zero linear and angular velocity | Provided |
| `angle_wrap()` | Normalizes angles to $[-\pi,\pi]$ | Provided |
| `yaw_from_quat()` | Converts quaternion orientation into yaw | Provided |

**`turtlebot_control(robot_pos, robot_h, target, dt, Ld)`**

This function implements the actual motion controller.

It receives the robot's position, heading, lookahead target, elapsed time, and lookahead distance.

It must return:

```python
(v_cmd, w_cmd)
```

where `v_cmd` is linear velocity in m/s and `w_cmd` is angular velocity in rad/s.

The template suggests combining pure pursuit with optional PID control. The main objective is to follow the selected target accurately while respecting velocity limits and maintaining clearance from obstacles.

The initialization code also requires us to configure the lookahead distance, maximum velocities, and controller parameters.

### 4. Main Navigation State Machine

The `control_step()` function is the main coordinator of the navigation system. It runs at a configured frequency of 20 Hz.

Much of its structure is already provided, but several state transitions still contain TODOs.

| Mode | Existing behavior | Our responsibility |
|---|---|---|
| `FOLLOW` | Determines the current path segment and calls perception and control functions | Implement passability, obstacle avoidance, and transition to `BACKTRACK` |
| `BACKTRACK` | Constructs a return path toward the previous node | Select the previous node and detect when backtracking is complete |
| `REPLAN` | Provides the location for replanning logic | Remove the blocked edge, calculate a new path, and return to `FOLLOW` |
| `DONE` | Stops the robot when the goal tolerance is satisfied | No major implementation required |

The relevant workflow is:

```text
control_step()
    |
    +-- Get robot position and heading
    |
    +-- Check goal and waypoint arrival
    |
    +-- REPLAN?
    |      Remove blocked graph edge
    |      Recalculate path
    |
    +-- BACKTRACK?
    |      Follow return path to previous node
    |
    +-- FOLLOW
           |
           +-- is_passable()
           |      |
           |      +-- False --> BACKTRACK
           |
           +-- obstacle_avoided_target()
           |
           +-- turtlebot_control()
           |
           +-- publish_cmd()
```

### 5. ROS 2 Interfaces and Visualization

| Component | Role |
|---|---|
| `/scan` | LiDAR distance measurements |
| `/odom` | Robot position and orientation estimates |
| `/cmd_vel` | Linear and angular velocity commands |
| `draw_scene()` | OpenCV visualization of the graph, path, robot, and lookahead target |
| `_set_mode()` | Handles mode transitions and introduces a short pause |
| `main()` | Initializes and runs the ROS 2 node |

The existing `draw_scene()` is useful for inspecting the planned route and tracking behavior.

We may add a separate visualization for LiDAR obstacle points, candidate trajectories, and clearance measurements while developing the perception component.

### 6. Summary of Required Implementation

The main implementation work can be grouped into four areas:

| Area | Functions / Tasks |
|---|---|
| **Path planning** | Complete `plan_from()` and connect blocked-edge removal to replanning |
| **Perception** | Implement `is_passable()` and any LiDAR processing helpers |
| **Obstacle avoidance** | Implement `obstacle_avoided_target()` |
| **Motion control** | Implement `turtlebot_control()`, configure controller parameters, and complete backtracking behavior |

The provided functions should be reused wherever possible. The assignment permits adding helper functions but advises against removing or modifying the existing template code.

Our initial priority is to establish reliable motion control in the obstacle-free environment, followed by LiDAR visualization and obstacle avoidance.

## Methodology

### 1. LiDAR-Based Perception

The TurtleBot uses LiDAR measurements to identify nearby obstacles and assess the available free space.

Each valid LiDAR measurement consists of a distance $r_i$ and an angle $\theta_i$. These measurements can be converted into Cartesian coordinates:

$$x_i=r_i\cos(\theta_i),\qquad y_i=r_i\sin(\theta_i)$$

Rather than relying solely on the minimum distance directly ahead, we intend to analyze the spatial distribution of obstacles to identify possible passages.

Potential approaches include:

- Detecting gaps between observed obstacle surfaces.
- Evaluating clearance along candidate trajectories.
- Accumulating LiDAR observations across successive scans to improve perception of partially occluded obstacles.

The TurtleBot's width is approximately **0.178 m**. Collision checking must account for the robot's footprint and an additional safety margin.

### 2. Obstacle Avoidance

A passage may remain traversable even when its nominal centerline is obstructed.

The obstacle-avoidance component is responsible for generating a suitable lookahead target that guides the robot through available free space.

Our proposed approach is to evaluate candidate trajectories based on their clearance from obstacles, feasibility, and progress toward the next waypoint.

The perception and obstacle-avoidance components should share a consistent understanding of the environment.

### 3. Motion Control

The controller converts a lookahead target into linear and angular velocity commands published to `/cmd_vel`.

The provided framework supports pure-pursuit-based path following and optional PID control.

The controller must balance path-tracking accuracy, obstacle clearance, and smooth motion, particularly when navigating narrow passages or turning at graph nodes.

### 4. A* Path Planning

The maze is modeled as a graph with known node coordinates and initially available connections between neighboring nodes.

A* computes a route from the current node to the destination.

When the perception system determines that an edge is impassable, the robot backtracks, removes that edge from its graph representation, and computes an alternative route.

This allows the robot to adapt to obstacle configurations without having prior knowledge of which passages are blocked.

## Development Approach

We plan to develop the navigation system incrementally:

- [ ] Implement and test A* path generation.
- [ ] Establish reliable motion control on the obstacle-free track.
- [ ] Visualize LiDAR measurements and obstacle coordinates.
- [ ] Implement basic collision detection and safe stopping.
- [ ] Evaluate candidate passages and obstacle-avoidance targets.
- [ ] Integrate blocked-edge detection with backtracking and replanning.
- [ ] Investigate edge-local accumulation of LiDAR observations if needed.
- [ ] Test navigation across different obstacle configurations.
- [ ] Evaluate reliability and completion time.

Advanced occupancy-grid mapping is being considered but is not an initial requirement. We aim to prioritize simpler approaches that can be validated experimentally.

## Project Structure

The project builds on the `vmc2_project` ROS 2 package provided with the assignment.

The principal implementation is located in `navigator.py`, which contains the navigation node, graph utilities, sensor callbacks, and control pipeline.

The exact repository structure will be documented as development progresses.

## Running the Simulation

The project uses **ROS 2 and Gazebo**.

The course provides:

- `vmc2_project`: ROS 2 package containing the launch configuration and navigation template.
- `project_track`: Maze environment containing obstacles.
- `project_track_no_obs`: Obstacle-free environment for initial testing.

Extract the provided Gazebo models into:

```text
~/.gazebo/models/
```

Place the ROS 2 package in your workspace, build it, and source the workspace:

```bash
colcon build --packages-select vmc2_project
source install/setup.bash
```

Launch the simulation:

```bash
ros2 launch vmc2_project project.launch.py
```

The navigation implementation is developed within the provided ROS 2 package.

## Project Requirements

The primary objective is to navigate the entire maze without colliding with walls or obstacles.

Additional project goals include:

- Completing the maze in less than **2 minutes and 30 seconds of simulation time**.
- Achieving reliable performance over repeated runs.
- Supporting different obstacle configurations without hard-coded obstacle locations.
- Demonstrating and documenting the navigation methodology and results.

## Status

**Under development.**

The current focus is on understanding LiDAR-based passability detection and designing the interface between perception, obstacle avoidance, and motion control.

Implementation decisions may change based on simulation results.

## Acknowledgements

Developed by **Ki Chun Tong, Shaan Salian, and Vo Dinh Nhan** as part of **MEC-E5012 Vehicle Mechatronics: Control** at Aalto University.

The initial ROS 2 navigation framework and Gazebo maze environment are provided as course materials.
