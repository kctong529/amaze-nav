#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SHARED_DIR="$(dirname "$SCRIPT_DIR")"

PERCEPTION_DIR="$SHARED_DIR/ros2_ws/src/vmc2_project/src"

if [[ ! -f "$PERCEPTION_DIR/perception.py" ]]; then
    echo "Error: perception.py not found at $PERCEPTION_DIR" >&2
    exit 1
fi

# Preserve ROS 2's existing Python paths while prioritizing
# Ubuntu's NumPy installation for OpenCV compatibility.
export PYTHONPATH="/usr/lib/python3/dist-packages:$PERCEPTION_DIR${PYTHONPATH:+:$PYTHONPATH}"

exec python3 "$SCRIPT_DIR/lidar_explorer.py"
