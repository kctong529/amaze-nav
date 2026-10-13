# Environment setup — amaze-nav

**Documentation:** [README](../README.md) · **Setup** · [Framework](framework.md) · [Methodology](methodology.md) · [Perception strategy](perception-strategy.md) · [Perception API](perception-api.md) · [Task ownership](tasks.md) · [Development plan](development.md) · [Ground-truth survey](edge-ground-truth.md) · [Controller debug](controller-debug.md) · [Navigator tests](testing.md) · [Offline tests](navigation-offline-tests.md) · [RViz & debugging](debugging.md)

---

This guide sets up the course-provided TurtleBot maze environment from **files already included in the `amaze-nav` repository**. You do **not** need to download or unzip the course archives again. The commands below are intended for the course's **Ubuntu / ROS 2 Humble environment**, even if you clone the repository on a different host operating system.

## 1. Prerequisites and repository contents

Have ROS 2 Humble, Gazebo Classic, `colcon`, and the TurtleBot3 packages available in the course environment. The repository should already contain:

```text
amaze-nav/
├── README.md
├── docs/
├── project_track/             # Gazebo model: obstacles
├── project_track_no_obs/      # Gazebo model: no internal obstacles
└── vmc2_project/              # ROS 2 package and launch file
```

Open an Ubuntu terminal **inside the repository root**:

```bash
cd "$HOME/shared/amaze-nav"  # repository root; assumed current directory below
export REPO_ROOT="$(pwd -P)"
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

printf 'Repository: %s\nHOME: %s\nROS distro: %s\n' \
  "$REPO_ROOT" "$HOME" "$ROS_DISTRO"
```

Do not run `colcon` or Gazebo Classic natively on a Mac just because the repository was cloned there. Use the course Ubuntu container / VM.

## 2. Choose a ROS 2 workspace and Gazebo model directory

The following is the expected directory layout for this repository. All paths are relative to `$HOME`, not a hard-coded username:

```bash
export REPO_ROOT="$HOME/shared/amaze-nav"   # assumed current working directory
export ROS_WS="$HOME/shared/ros2_ws"
export MAZE_MODELS="$HOME/.gazebo/models"

mkdir -p "$ROS_WS/src" "$MAZE_MODELS"
```

The paths are **siblings**, not nested: `amaze-nav` is the Git checkout, `ros2_ws` is the ROS workspace, and `.gazebo/models` is the per-user Gazebo model directory. Commands involving `tools/` and `vmc2_project/` assume the current directory is `$REPO_ROOT`, unless a `cd` is shown explicitly.

## 3. Install the included package and models

Install the ROS package and the maze models from the existing repository:

```bash
# Link the tracked package into the separate ROS workspace so edits in the
# repository are reflected in the workspace without repeated copying.
if [ ! -e "$ROS_WS/src/vmc2_project" ]; then
  ln -s "$REPO_ROOT/vmc2_project" "$ROS_WS/src/vmc2_project"
fi

# Install model folders in Gazebo's conventional user directory.
cp -a "$REPO_ROOT/project_track" "$MAZE_MODELS/"
cp -a "$REPO_ROOT/project_track_no_obs" "$MAZE_MODELS/"
```

If `$ROS_WS/src/vmc2_project` already exists, check whether it is a symlink to `$REPO_ROOT/vmc2_project` or a separate copy before proceeding. Do not silently overwrite it: a separate copy can become stale relative to the Git checkout. The symlink is optional if you already have a correctly configured source package in the workspace.

Check the model locations (the names must correspond to the model URI names expected by the launch/world files):

```bash
ls -la "$MAZE_MODELS/project_track" "$MAZE_MODELS/project_track_no_obs"
find "$MAZE_MODELS" -maxdepth 3 \
  \( -name model.config -o -name model.sdf \)
```

You should find each model's `model.config` and `model.sdf` inside its own folder. There should be no extra `project_track/project_track/` nesting.

## 4. Add the maze models to Gazebo's search path

The **parent directory** of the two model folders, `$HOME/.gazebo/models`, must be visible to Gazebo via `GAZEBO_MODEL_PATH`.

```bash
case ":${GAZEBO_MODEL_PATH:-}:" in
  *":$MAZE_MODELS:"*) ;;
  *) export GAZEBO_MODEL_PATH="$MAZE_MODELS${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}" ;;
esac

echo "$GAZEBO_MODEL_PATH"
```

Keep the existing TurtleBot3 model paths; don't overwrite the variable with the maze directory alone. If you later make the settings permanent in `~/.bashrc`, define `MAZE_MODELS` first and use the same guarded export to avoid duplicate path entries.

## 5. Build and source the ROS package

```bash
cd "$ROS_WS"
colcon build --symlink-install --packages-select vmc2_project
source "$ROS_WS/install/setup.bash"
ros2 pkg prefix vmc2_project
```

If `colcon` cannot find `vmc2_project`, check its `package.xml` and that it is located beneath `$ROS_WS/src`:

```bash
find "$ROS_WS/src" -name package.xml
```

**Every new terminal** needs the ROS environment and workspace sourced. If `ROS_WS`, `MAZE_MODELS`, or the model-path export were defined only in another terminal, set them again, or add them to your shell startup file.

## 6. Launch the simulation

```bash
ros2 launch vmc2_project project.launch.py
```

The provided launch file supports the maze environment; check `project.launch.py` or `ros2 launch vmc2_project project.launch.py --show-args` for the exact method of choosing the obstacle-free world. Do not assume a particular launch argument exists.

For manual exploration, make sure the unfinished autonomous navigator is not simultaneously publishing `/cmd_vel` commands.

## 7. Run the autonomous navigator

Run this in a **second Ubuntu terminal**, after Gazebo has started. Source the ROS 2 environment and the built workspace in this terminal too:

```bash
source /opt/ros/humble/setup.bash
export ROS_WS="$HOME/shared/ros2_ws"
source "$ROS_WS/install/setup.bash"
export TURTLEBOT3_MODEL=burger

PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}" \
  ros2 run vmc2_project navigator.py
```

The explicit `PYTHONPATH` is the workaround used in the course container; omit it if the normal ROS environment already imports all dependencies. Keep Gazebo running in its own terminal. **Do not run keyboard teleoperation or another navigator at the same time:** only one process should command `/cmd_vel`.

The navigator subscribes to `/odom` and `/scan`, publishes `/cmd_vel`, and displays its graph/path in an OpenCV window. Its main states are `FOLLOW` (drive along a planned edge), `BACKTRACK` (return toward the previous node), `REPLAN` (find another route), and `DONE`. It may stop temporarily while waiting for a fresh scan or a validated motion plan. To terminate it, press **Ctrl+C in the navigator terminal**, and verify the robot stops.

**Check the installed executable:** `ros2 pkg prefix vmc2_project` shows the active package prefix. If editing the source does not affect the launched code, rebuild and re-source this workspace. The navigator is not started by simply opening Gazebo unless your local launch configuration explicitly includes it.

## 8. Launch and use LiDAR Explorer

`tools/lidar_explorer.py` is a **read-only ROS 2/OpenCV diagnostic tool**, separate from the autonomous `navigator.py`. It subscribes to `/scan` and `/odom`, uses TF to transform scans, and opens three windows. It **does not** publish `/cmd_vel`, so it will not drive the robot. Start Gazebo first, then run the explorer in a separate terminal with a graphical desktop:

```bash
cd "$HOME/shared/amaze-nav"  # repository root
source /opt/ros/humble/setup.bash
source "$HOME/shared/ros2_ws/install/setup.bash"
bash tools/run_lidar_explorer.sh
```

The launcher finds the repository from its own `tools/` directory and uses the sibling workspace at `$HOME/shared/ros2_ws`. The expected layout is:

```text
$HOME/
├── shared/
│   ├── amaze-nav/                  # repository root (current directory)
│   │   ├── tools/
│   │   │   ├── run_lidar_explorer.sh
│   │   │   ├── lidar_explorer.py
│   │   │   ├── trajectory_display.py
│   │   │   └── perception_geometry.py
│   │   └── vmc2_project/src/perception.py
│   └── ros2_ws/                   # ROS workspace, sibling of amaze-nav
│       └── src/vmc2_project -> ../../amaze-nav/vmc2_project
└── .gazebo/models/
    ├── project_track/
    └── project_track_no_obs/
```

`perception_geometry.py` is imported directly by the explorer, and `perception.py` is loaded from `$ROS_WS/src/vmc2_project/src`. The updated launcher sets `PYTHONPATH` to include Ubuntu's system Python packages and the workspace package's `src` folder, then runs `python3 tools/lidar_explorer.py`. It checks that both files exist; ROS 2, OpenCV and NumPy must also be installed. **Use the updated launcher supplied with this documentation**—the old launcher incorrectly expected `ros2_ws` *inside* `amaze-nav`. Do not run the explorer with `ros2 run vmc2_project lidar_explorer.py`.

### Three visualization windows

- **Sensor Frame:** raw valid LiDAR returns in sensor-local coordinates (+X forward, +Y left).
- **Odom Frame:** the latest scan transformed through TF into the fixed odometry frame, displayed with a camera centered on the sensor.
- **Edge Frame:** selected directed grid edge in `(s, t)` coordinates, cross-sectional slice and candidate openings, robot position, selected plan and representative feasible/rejected trajectories. `s` points along the edge and `+t` to its left.

Select/focus an OpenCV window before pressing a key. Controls verified against `lidar_explorer.py`:

| Key | Action |
| --- | --- |
| `[` / `]` | Move the selected cross-sectional slice backward / forward by 0.10 m |
| `,` / `.` | Select the previous / next directed graph edge; reset edge-specific diagnostics |
| `p` | Print the selected slice's profile to the terminal |
| `b` | Run and log passability assessment of the selected edge at the latest observed robot position |
| `t` | Run `plan_motion()` on the latest transformed observation and show/log feasible and rejected trajectory candidates |
| `v` | Cycle `best` → `feasible` → `debug` display modes |
| `+` / `-` | Increase / decrease number of displayed alternatives (steps of five) |
| `k` / `j` | Increase / decrease the maximum curvature used for *explorer planning* (1/m) |
| `n` / `m` | Increase / decrease the *explorer* safety margin by 0.01 m |

**Typical workflow:** position the robot using Gazebo/teleoperation **with the navigator stopped**, choose the directed edge with `,` / `.`, inspect slices with `[` / `]`, press `p` for slice details, `b` for passability, then `t` to sample and visualize candidate motions. Press `v` to compare viable paths and rejection examples. Allow a fresh scan after changing the margin with `m`/`n` before running `t` again.

`k`/`j` and `m`/`n` change parameters **in the explorer**, not those used by an independently running navigator. Displayed trajectories are candidate paths checked against observed returns; unobserved space is not guaranteed free. A proposed segment after a `STOP / ROTATE` step is **hypothetical** until a fresh scan and a new plan. Stop the explorer with **Ctrl+C** in its terminal; its windows should close on shutdown.

For troubleshooting, diagnostic interpretation and links to the perception API, see [RViz & debugging](debugging.md#standalone-lidar-trajectory-explorer-implemented) and [Perception API](perception-api.md).

## 9. Verify sensors and visualize in RViz

In another sourced terminal:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic hz /scan
ros2 topic hz /odom
rviz2
```

In RViz, add `LaserScan` (`/scan`), `Odometry` (`/odom`), `TF`, and `Grid`. Select an available fixed frame with valid transforms (often `odom`). If installed, keyboard teleoperation can be run separately:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Only run teleoperation when the automated navigator is stopped/disabled.

## 10. Troubleshooting: Gazebo shows neither the maze nor the robot

First verify the chosen paths:

```bash
printf 'REPO_ROOT=%s\nROS_WS=%s\nMAZE_MODELS=%s\n' \
  "$REPO_ROOT" "$ROS_WS" "$MAZE_MODELS"
echo "$GAZEBO_MODEL_PATH"
find "$MAZE_MODELS" -maxdepth 3 \
  \( -name model.config -o -name model.sdf \)
ros2 pkg prefix vmc2_project
```

If the models are not found, check that they are directly under `$HOME/.gazebo/models`, append that directory to `GAZEBO_MODEL_PATH`, and fully restart Gazebo:

```bash
export MAZE_MODELS="$HOME/.gazebo/models"
export GAZEBO_MODEL_PATH="$MAZE_MODELS${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
```

An absent model path is a plausible cause of an empty maze, but **it does not by itself prove why the TurtleBot is missing**. If the scene remains empty, inspect the launch file and spawning logs:

```bash
find "$ROS_WS/src" -name project.launch.py
ros2 topic list | grep -E 'scan|odom|cmd_vel'
ros2 service list | grep -E 'spawn_entity|get_entity'
```

Look for `Unable to find uri`, `model://project_track`, spawning errors, or an incorrect world selection. Also inspect Gazebo's model list and reset the camera view.

## 11. Setup checklist

- [ ] Repository includes `vmc2_project`, `project_track`, and `project_track_no_obs`.
- [ ] `ROS_WS` points to the correct workspace.
- [ ] The ROS package source is discoverable under `$ROS_WS/src`.
- [ ] Both Gazebo model directories contain their model files.
- [ ] `GAZEBO_MODEL_PATH` includes the model directory's parent.
- [ ] `TURTLEBOT3_MODEL=burger` is configured.
- [ ] `colcon build` succeeds and `ros2 pkg prefix vmc2_project` works.
- [ ] The maze and TurtleBot appear in Gazebo.
- [ ] `/scan` and `/odom` publish.
- [ ] RViz displays LiDAR correctly.
- [ ] The navigator runs in a separate terminal and receives `/odom` and `/scan`.
- [ ] The explorer launcher/script paths have been checked, and its diagnostic window opens (when installed).

Next: [edge ground-truth exploration](edge-ground-truth.md) or [controller-only debug tests](controller-debug.md).
