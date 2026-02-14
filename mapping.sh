#!/bin/bash
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

if [ ! -f "install/setup.bash" ]; then
  echo "找不到 install/setup.bash，请先 colcon build"
  exit 1
fi

source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE_DIR/fastdds_no_shm.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch pct_planner mapping_launch.py
