#!/bin/bash
# MuJoCo 全栈导航: MuJoCo sim + terrain_analysis + global_planner + pct_path_adapter + localPlanner + pathFollower
#
# 前置: 先运行 gen_scene_pcd.py 生成 /tmp/sim_maps/building_nav.pcd
#       再用 build_tomogram_from_pcd 生成 /tmp/sim_maps/building_nav.pickle
#
# 用法: bash /tmp/test_viz_global.sh [goal_x] [goal_y] [goal_z] [monitor_sec]
set -e

GOAL_X=${1:-20.0}
GOAL_Y=${2:-12.0}
GOAL_Z=${3:-0.0}
MONITOR_SEC=${4:-60}

# cleanup
pkill -9 -f sim_viz_full 2>/dev/null || true
pkill -9 -f mujoco_ros2_bridge 2>/dev/null || true
pkill -9 -f terrainAnalysis 2>/dev/null || true
pkill -9 -f localPlanner 2>/dev/null || true
pkill -9 -f pathFollower 2>/dev/null || true
pkill -9 -f pct_path_adapter 2>/dev/null || true
pkill -9 -f global_planner 2>/dev/null || true
pkill -9 -f pct_planner_astar 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
source /home/sunrise/data/SLAM/navigation/install/setup.bash 2>/dev/null || true

MAP_DIR=/tmp/sim_maps
MAP_FILE=${MAP_DIR}/building_nav

# Verify map exists
if [ ! -f "${MAP_FILE}.pickle" ] && [ ! -f "${MAP_FILE}.pcd" ]; then
    echo "ERROR: Map not found at ${MAP_FILE}.pickle or .pcd"
    echo "Run gen_scene_pcd.py first to generate the map"
    exit 1
fi

echo "=== MuJoCo Full-Stack Navigation (with Global Planner) ==="
echo "Goal: ($GOAL_X, $GOAL_Y, $GOAL_Z), monitor ${MONITOR_SEC}s"
echo "Map: ${MAP_FILE}"

# 1. MuJoCo viz bridge
echo "[1/6] Starting MuJoCo viz bridge..."
DISPLAY=:0 python3 /tmp/sim_viz_full.py > /tmp/sim_viz.log 2>&1 &
VIZ_PID=$!
sleep 4
if ! kill -0 $VIZ_PID 2>/dev/null; then
    echo "ERROR: viz died!"; tail -20 /tmp/sim_viz.log; exit 1
fi

# 2. terrain_analysis
echo "[2/6] Starting terrain_analysis..."
ros2 run terrain_analysis terrainAnalysis \
    --ros-args \
    -r /Odometry:=/nav/odometry \
    -r /cloud_map:=/livox/lidar \
    -r /terrain_map:=/nav/terrain_map \
    -p scanVoxelSize:=0.1 \
    -p decayTime:=5.0 \
    -p noDecayDis:=3.0 \
    -p clearingDis:=15.0 \
    > /tmp/terrain_analysis.log 2>&1 &
TA_PID=$!
sleep 2

# 3. global_planner.py (C++ ele_planner.so — 生产级规划器)
echo "[3/6] Starting global_planner (C++ ele_planner.so)..."
PCT_SHARE=$(ros2 pkg prefix pct_planner)/share/pct_planner
PLANNER_SCRIPT=${PCT_SHARE}/planner/scripts/global_planner.py

# ele_planner.so 需要 numpy 1.x；使用 venv 但保留系统 PYTHONPATH
VENV_PYTHON=/tmp/venv_np1/bin/python3
if [ ! -f "$VENV_PYTHON" ]; then
    echo "ERROR: numpy 1.x venv not found at $VENV_PYTHON"
    echo "Create it: python3 -m venv /tmp/venv_np1 && /tmp/venv_np1/bin/pip install 'numpy<2'"
    exit 1
fi

$VENV_PYTHON ${PLANNER_SCRIPT} \
    --ros-args \
    -r /goal_pose:=/nav/goal_pose \
    -r /pct_path:=/nav/global_path \
    -r /pct_planner/status:=/nav/planner_status \
    -p map_file:="${MAP_FILE}.pickle" \
    -p obstacle_thr:=50 \
    -p tomogram_ground_h:=0.0 \
    > /tmp/global_planner.log 2>&1 &
GP_PID=$!
sleep 3
if ! kill -0 $GP_PID 2>/dev/null; then
    echo "ERROR: global_planner died!"; tail -20 /tmp/global_planner.log; exit 1
fi
echo "  global_planner started (PID=$GP_PID)"

# 4. pct_path_adapter
echo "[4/6] Starting pct_path_adapter..."
ros2 run pct_adapters pct_path_adapter \
    --ros-args \
    -r /pct_path:=/nav/global_path \
    -r /Odometry:=/nav/odometry \
    -r /planner_waypoint:=/nav/way_point \
    -p waypoint_distance:=1.5 \
    -p arrival_threshold:=0.8 \
    -p stuck_timeout_sec:=30.0 \
    -p max_replan_count:=2 \
    > /tmp/pct_adapter.log 2>&1 &
PA_PID=$!
sleep 1

# 5. localPlanner (with obstacle avoidance ON)
echo "[5/6] Starting localPlanner..."
PATHS_DIR=/home/sunrise/data/SLAM/navigation/install/local_planner/share/local_planner/paths
ros2 run local_planner localPlanner \
    --ros-args \
    -r /Odometry:=/nav/odometry \
    -r /cloud_map:=/livox/lidar \
    -r /terrain_map:=/nav/terrain_map \
    -r /way_point:=/nav/way_point \
    -p pathFolder:="$PATHS_DIR" \
    -p autonomyMode:=true \
    -p autonomySpeed:=1.0 \
    -p useTerrainAnalysis:=true \
    -p checkObstacle:=true \
    -p useCost:=true \
    -p slopeWeight:=3.0 \
    -p twoWayDrive:=false \
    > /tmp/local_planner.log 2>&1 &
LP_PID=$!
sleep 1

# 6. pathFollower
echo "[6/6] Starting pathFollower..."
ros2 run local_planner pathFollower \
    --ros-args \
    -r /Odometry:=/nav/odometry \
    -r /cmd_vel:=/nav/cmd_vel \
    -p autonomyMode:=true \
    -p autonomySpeed:=1.0 \
    -p stuck_timeout:=15.0 \
    -p stuck_dist_thre:=0.15 \
    > /tmp/path_follower.log 2>&1 &
PF_PID=$!
sleep 3

echo ""
echo "All 6 nodes started. Waiting 5s for terrain stabilization..."
sleep 5

# Send goal_pose (NOT way_point — global_planner subscribes to this)
echo "=== Sending goal_pose to ($GOAL_X, $GOAL_Y, $GOAL_Z) ==="
ros2 topic pub /nav/goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}, orientation: {w: 1.0}}}" \
    --once > /dev/null 2>&1 &

sleep 2

# Check if global planner found a path
echo "=== Global Planner Status ==="
tail -5 /tmp/global_planner.log
echo ""

# Monitor
for i in $(seq 1 $MONITOR_SEC); do
    sleep 1
    POS=$(tail -5 /tmp/sim_viz.log 2>/dev/null | grep -oP 'pos=\([^)]+\)' | tail -1)
    CMD=$(tail -5 /tmp/sim_viz.log 2>/dev/null | grep -oP 'cmd=\([^)]+\)' | tail -1)
    ADAPTER=$(tail -1 /tmp/pct_adapter.log 2>/dev/null | head -1)
    if [ -n "$POS" ]; then
        echo "  t=${i}s: $POS $CMD"
    fi
    # Check for goal_reached
    if grep -q "goal_reached" /tmp/pct_adapter.log 2>/dev/null; then
        echo ""
        echo "*** GOAL REACHED! ***"
        break
    fi
done

echo ""
echo "=== Final state ==="
echo "-- sim_viz --"
tail -3 /tmp/sim_viz.log
echo "-- global_planner --"
tail -3 /tmp/global_planner.log
echo "-- pct_adapter --"
tail -5 /tmp/pct_adapter.log
echo "-- local_planner --"
tail -3 /tmp/local_planner.log
echo ""
echo "PIDs: VIZ=$VIZ_PID TA=$TA_PID GP=$GP_PID PA=$PA_PID LP=$LP_PID PF=$PF_PID"
