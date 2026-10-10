#!/usr/bin/env bash
set -e

PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}" \
    python3 "$(dirname "$0")/lidar_explorer.py"
