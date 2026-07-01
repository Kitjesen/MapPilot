#!/bin/bash
# LingTu MuJoCo Simulation — Full Algorithm Stack
# Usage: bash sim/launch/sim_full.sh
#
# Starts:
#   1. MuJoCo physics + sensor bridge (replaces LiDAR + SLAM)
#   2. C++ autonomy nodes (terrain + local planner + path follower)
#   3. Python full stack (global planning + semantic + safety + gateway)

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NAV_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$NAV_ROOT"

# Source ROS2
source /opt/ros/humble/setup.bash
source /opt/nav/install/setup.bash 2>/dev/null || true

# Cleanup function
cleanup() {
    echo "Stopping all processes..."
    kill $BRIDGE_PID $AUTONOMY_PID 2>/dev/null
    wait 2>/dev/null
    echo "Done."
}
trap cleanup EXIT INT TERM

echo "============================================"
echo "  LingTu MuJoCo Simulation (Full Stack)"
echo "============================================"

# 1. MuJoCo bridge (sensor replacement)
echo "[1/3] Starting MuJoCo bridge..."
python3 src/drivers/sim/mujoco_ros2_bridge.py > /tmp/mujoco_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 3

# Verify bridge is running
if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "ERROR: MuJoCo bridge failed to start. Check /tmp/mujoco_bridge.log"
    exit 1
fi
echo "  Bridge OK (PID $BRIDGE_PID)"

# 2. C++ autonomy (real algorithms)
echo "[2/3] Starting C++ autonomy nodes..."
ros2 launch /opt/nav/launch/subsystems/autonomy.launch.py > /tmp/autonomy.log 2>&1 &
AUTONOMY_PID=$!
sleep 4
echo "  Autonomy OK (PID $AUTONOMY_PID)"

# 3. Python full stack
echo "[3/3] Starting Python navigation stack..."
echo ""
PYTHONPATH="src:." python3 lingtu.py sim
