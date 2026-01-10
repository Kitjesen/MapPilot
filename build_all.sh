#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

echo ">>> [1/2] Building PCT Planner C++ Core..."
# Go to planner directory
cd src/global_planning/PCT_planner/planner

# Check if build script exists
if [ ! -f "build.sh" ]; then
    echo "Error: build.sh not found in $(pwd)"
    exit 1
fi

./build.sh

echo ">>> [2/2] Building ROS 2 Workspace (pct_planner only)..."
# Go back to workspace root (assuming script is in root)
cd ../../../../..

# Verify we are at root
if [ ! -f "fastdds_no_shm.xml" ] && [ ! -d "src" ]; then
    echo "Warning: Not in workspace root? Current dir: $(pwd)"
fi

colcon build --packages-select pct_adapters pct_planner --symlink-install

echo ">>> All Done! Don't forget to: source install/setup.bash"
