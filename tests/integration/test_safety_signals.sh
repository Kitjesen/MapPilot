#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# T7: Safety signal chain integration test
#
# Starts terrainAnalysis + localPlanner + pathFollower (3 real C++ nodes),
# then runs test_safety_signals.py which injects synthetic sensor data and
# verifies: normal fwd motion → obstacle stop=2 → recovery after clear.
#
# Platform: S100P robot (192.168.66.190, user: sunrise)
#
# Usage:
#   bash tests/integration/test_safety_signals.sh
# ─────────────────────────────────────────────────────────────────────────────

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WORKSPACE_DIR"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

TF_PID=""
TA_PID=""
LP_PID=""
PF_PID=""

cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up T7 nodes...${NC}"
    for pid in $TF_PID $TA_PID $LP_PID $PF_PID; do
        [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
    done
    sleep 1
    pkill -9 -f terrainAnalysis 2>/dev/null || true
    pkill -9 -f localPlanner 2>/dev/null || true
    pkill -9 -f pathFollower 2>/dev/null || true
    pkill -9 -f "static_transform_publisher.*map.*odom" 2>/dev/null || true
}
trap cleanup EXIT

echo "=========================================="
echo -e "${BOLD}  T7: Safety Signal Chain Integration Test${NC}"
echo "  terrainAnalysis + localPlanner + pathFollower"
echo "=========================================="

# Kill leftover nodes
echo -e "${YELLOW}Killing leftover nodes...${NC}"
pkill -9 -f terrainAnalysis 2>/dev/null || true
pkill -9 -f localPlanner 2>/dev/null || true
pkill -9 -f pathFollower 2>/dev/null || true
pkill -9 -f "static_transform_publisher.*map.*odom" 2>/dev/null || true
sleep 1

# ── Environment ───────────────────────────────────────────────────────────────
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${RED}ROS2 Humble not found${NC}"; exit 1
fi
source /opt/ros/humble/setup.bash
if [ ! -f "install/setup.bash" ]; then
    echo -e "${RED}install/ not found. Run: make build${NC}"; exit 1
fi
source install/setup.bash

# ── Locate paths folder ───────────────────────────────────────────────────────
LP_SHARE=$(ros2 pkg prefix local_planner 2>/dev/null)/share/local_planner || true
PATHS_DIR="${LP_SHARE}/paths"
if [ ! -d "$PATHS_DIR" ]; then
    PATHS_DIR="install/local_planner/share/local_planner/paths"
fi
echo -e "${BLUE}Paths dir: ${PATHS_DIR}${NC}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="/tmp/t7_logs"
mkdir -p "$LOG_DIR"

# ── Node 1: Static TF map -> odom ────────────────────────────────────────────
echo -e "${BLUE}[1/4] Static TF: map -> odom${NC}"
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 map odom \
    > "$LOG_DIR/tf.log" 2>&1 &
TF_PID=$!

# ── Node 2: terrainAnalysis ──────────────────────────────────────────────────
echo -e "${BLUE}[2/4] terrainAnalysis${NC}"
ros2 run terrain_analysis terrainAnalysis --ros-args \
    -r /Odometry:=/nav/odometry \
    -r /cloud_map:=/nav/map_cloud \
    -r /terrain_map:=/nav/terrain_map \
    -p scanVoxelSize:=0.1 \
    -p obstacleHeightThre:=0.2 \
    -p groundHeightThre:=0.1 \
    > "$LOG_DIR/terrain.log" 2>&1 &
TA_PID=$!

# ── Node 3: localPlanner ─────────────────────────────────────────────────────
echo -e "${BLUE}[3/4] localPlanner (checkObstacle:=true)${NC}"
ros2 run local_planner localPlanner --ros-args \
    -r /Odometry:=/nav/odometry \
    -r /cloud_map:=/nav/map_cloud \
    -r /terrain_map:=/nav/terrain_map \
    -r /way_point:=/nav/way_point \
    -r /speed:=/nav/speed \
    -r /stop:=/nav/stop \
    -r /slow_down:=/nav/slow_down \
    -r /path:=/nav/local_path \
    -p pathFolder:="$PATHS_DIR" \
    -p autonomyMode:=true \
    -p autonomySpeed:=1.0 \
    -p useTerrainAnalysis:=true \
    -p checkObstacle:=true \
    -p twoWayDrive:=false \
    > "$LOG_DIR/local_planner.log" 2>&1 &
LP_PID=$!

# ── Node 4: pathFollower ─────────────────────────────────────────────────────
echo -e "${BLUE}[4/4] pathFollower${NC}"
ros2 run local_planner pathFollower --ros-args \
    -r /Odometry:=/nav/odometry \
    -r /path:=/nav/local_path \
    -r /speed:=/nav/speed \
    -r /stop:=/nav/stop \
    -r /slow_down:=/nav/slow_down \
    -r /cmd_vel:=/nav/cmd_vel \
    -p autonomyMode:=true \
    -p autonomySpeed:=1.0 \
    > "$LOG_DIR/path_follower.log" 2>&1 &
PF_PID=$!

# ── Wait and verify ──────────────────────────────────────────────────────────
echo ""
echo -e "${BLUE}Waiting for nodes to initialize (8s)...${NC}"
for i in $(seq 1 8); do echo -n "."; sleep 1; done
echo ""

echo -e "${YELLOW}Node liveness check:${NC}"
ALL_ALIVE=true
for NAME_PID in "TF:$TF_PID" "terrain:$TA_PID" "localPlanner:$LP_PID" "pathFollower:$PF_PID"; do
    NAME="${NAME_PID%%:*}"
    PID="${NAME_PID##*:}"
    if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
        echo -e "  ${GREEN}OK${NC}  $NAME (PID=$PID)"
    else
        echo -e "  ${RED}DEAD${NC}  $NAME (PID=$PID)"
        echo "    Log: cat $LOG_DIR/${NAME,,}.log"
        ALL_ALIVE=false
    fi
done

if [ "$ALL_ALIVE" = false ]; then
    echo -e "${RED}Some nodes died. Cannot proceed.${NC}"
    exit 1
fi
echo ""

# ── Run Python test harness ──────────────────────────────────────────────────
echo -e "${BOLD}Running T7 test harness...${NC}"
echo ""
python3 "${SCRIPT_DIR}/test_safety_signals.py"
TEST_EXIT=$?

echo ""
if [ $TEST_EXIT -eq 0 ]; then
    echo -e "${GREEN}T7 Safety Signal test PASSED${NC}"
else
    echo -e "${RED}T7 Safety Signal test FAILED${NC}"
    echo ""
    echo "Logs:"
    echo "  cat $LOG_DIR/terrain.log"
    echo "  cat $LOG_DIR/local_planner.log"
    echo "  cat $LOG_DIR/path_follower.log"
fi

exit $TEST_EXIT
