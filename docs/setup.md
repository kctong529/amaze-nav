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
cd /path/to/amaze-nav   # substitute your own cloned repository location
export REPO_ROOT="$(pwd -P)"
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

printf 'Repository: %s\nHOME: %s\nROS distro: %s\n' \
  "$REPO_ROOT" "$HOME" "$ROS_DISTRO"
```

Do not run `colcon` or Gazebo Classic natively on a Mac just because the repository was cloned there. Use the course Ubuntu container / VM.

## 2. Choose a ROS 2 workspace and Gazebo model directory

These paths are configurable and **must not assume anyone's username**.

```bash
# Pick a workspace; the course environment often uses $HOME/shared/ros2_ws.
export ROS_WS="$HOME/shared/ros2_ws"

# Gazebo's conventional per-user model directory:
export MAZE_MODELS="$HOME/.gazebo/models"

# If your course environment instead uses a shared model directory,
# change only this variable, for example:
# export MAZE_MODELS="$HOME/shared/.gazebo/models"

mkdir -p "$ROS_WS/src" "$MAZE_MODELS"
```

If you're already using a different ROS workspace, set `ROS_WS` to that directory. Check where your existing course models live before choosing `MAZE_MODELS`.

## 3. Install the included package and models

Copy the repository's existing directories into the appropriate ROS/Gazebo locations:

```bash
# Install/update the ROS package source in the workspace.
# Do not copy onto itself if the repository already lives at that path.
if [ "$REPO_ROOT/vmc2_project" != "$ROS_WS/src/vmc2_project" ]; then
  cp -a "$REPO_ROOT/vmc2_project" "$ROS_WS/src/"
fi

# Install both maze models. Remove old copies *only if you intend to replace them*.
cp -a "$REPO_ROOT/project_track" "$MAZE_MODELS/"
cp -a "$REPO_ROOT/project_track_no_obs" "$MAZE_MODELS/"
```

**Development note:** If your clone is outside `ROS_WS/src`, this copy is a separate working tree. Edits made to the original repository will not automatically appear in the workspace copy (or vice versa). For active development, it is often better to **clone the repository directly under `$ROS_WS/src/`**, or explicitly link the package from there using a symbolic link supported by your environment. If cloning the repository under `$ROS_WS/src/amaze-nav`, the package is nested one directory below `src`; `colcon` can discover packages recursively when run from the workspace root.

Check the model locations (the names must correspond to the model URI names expected by the launch/world files):

```bash
ls -la "$MAZE_MODELS/project_track" "$MAZE_MODELS/project_track_no_obs"
find "$MAZE_MODELS" -maxdepth 3 \
  \( -name model.config -o -name model.sdf \)
```

You should find each model's `model.config` and `model.sdf` inside its own folder. There should be no extra `project_track/project_track/` nesting.

## 4. Add the maze models to Gazebo's search path

The **parent directory** of the two model folders must be visible to Gazebo. This was important in the course container where the models were stored under `$HOME/shared/.gazebo/models` but that directory was not included in `GAZEBO_MODEL_PATH`.

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

## 7. Verify sensors and visualize in RViz

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

## 8. Troubleshooting: Gazebo shows neither the maze nor the robot

First verify the chosen paths:

```bash
printf 'REPO_ROOT=%s\nROS_WS=%s\nMAZE_MODELS=%s\n' \
  "$REPO_ROOT" "$ROS_WS" "$MAZE_MODELS"
echo "$GAZEBO_MODEL_PATH"
find "$MAZE_MODELS" -maxdepth 3 \
  \( -name model.config -o -name model.sdf \)
ros2 pkg prefix vmc2_project
```

If the models are installed at `$HOME/shared/.gazebo/models`, **but the model search path lists only TurtleBot3's own models**, point `MAZE_MODELS` at the shared directory, update `GAZEBO_MODEL_PATH`, and fully restart Gazebo:

```bash
export MAZE_MODELS="$HOME/shared/.gazebo/models"
export GAZEBO_MODEL_PATH="$MAZE_MODELS${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
```

An absent model path is a plausible cause of an empty maze, but **it does not by itself prove why the TurtleBot is missing**. If the scene remains empty, inspect the launch file and spawning logs:

```bash
find "$ROS_WS/src" -name project.launch.py
ros2 topic list | grep -E 'scan|odom|cmd_vel'
ros2 service list | grep -E 'spawn_entity|get_entity'
```

Look for `Unable to find uri`, `model://project_track`, spawning errors, or an incorrect world selection. Also inspect Gazebo's model list and reset the camera view.

## 9. Setup checklist

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

Next: [edge ground-truth exploration](edge-ground-truth.md) or [controller-only debug tests](controller-debug.md).
