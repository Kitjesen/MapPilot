#!/bin/bash
# MuJoCo 最小闭环全栈测试
# MuJoCo sim → terrain_analysis → localPlanner → pathFollower → cmd_vel → MuJoCo
#
# 关键 remap 发现 (2026-03-06):
#   - terrain_analysis / localPlanner / pathFollower 内部订阅 /Odometry (大写 O)
#   - 需要 -r /Odometry:=/slam/odometry 才能接收 sim bridge 的里程计
#   - localPlanner 需要 autonomyMode:=true + useTerrainAnalysis:=true
#   - PointCloud2 必须包含 intensity 字段 (XYZI, 16 bytes/point)
#   - 机器人 geom group=0, 环境 geom group=1, LiDAR 只检测 group=1
#
# 用法 (在 S100P 机器人上):
#   scp src/drivers/sim/mujoco_ros2_bridge.py sunrise@192.168.66.190:/tmp/
#   scp sim/scripts/test_fullloop.sh sunrise@192.168.66.190:/tmp/
#   ssh sunrise@192.168.66.190 'bash /tmp/test_fullloop.sh'
set -e
source /opt/ros/humble/setup.bash
source /home/sunrise/data/SLAM/navigation/install/setup.bash 2>/dev/null || true

GOAL_X=${1:-5.0}
GOAL_Y=${2:-3.0}
GOAL_Z=${3:-0.4}
MONITOR_SEC=${4:-30}

echo "=== MuJoCo Full Loop Test ==="
echo "Goal: ($GOAL_X, $GOAL_Y, $GOAL_Z), monitor ${MONITOR_SEC}s"

# cleanup
pkill -9 -f mujoco_ros2_bridge 2>/dev/null || true
pkill -9 -f terrainAnalysis 2>/dev/null || true
pkill -9 -f localPlanner 2>/dev/null || true
pkill -9 -f pathFollower 2>/dev/null || true
sleep 1

# 1. MuJoCo sim bridge
echo "[1/4] Starting MuJoCo sim bridge..."
python3 /tmp/mujoco_ros2_bridge.py > /tmp/sim_bridge.log 2>&1 &
SIM_PID=$!
sleep 3
if ! kill -0 $SIM_PID 2>/dev/null; then
    echo "ERROR: sim bridge died!"; cat /tmp/sim_bridge.log; exit 1
fi

# 2. terrain_analysis
echo "[2/4] Starting terrain_analysis..."
ros2 run terrain_analysis terrainAnalysis \
    --ros-args \
    -r /Odometry:=/slam/odometry \
    -r /cloud_map:=/livox/lidar \
    -r /terrain_map:=/nav/terrain_map \
    -p scanVoxelSize:=0.1 \
    -p decayTime:=5.0 \
    -p noDecayDis:=3.0 \
    -p clearingDis:=15.0 \
    > /tmp/terrain_analysis.log 2>&1 &
TA_PID=$!
sleep 2

# 3. localPlanner
echo "[3/4] Starting localPlanner..."
PATHS_DIR=/home/sunrise/data/SLAM/navigation/install/local_planner/share/local_planner/paths
ros2 run local_planner localPlanner \
    --ros-args \
    -r /Odometry:=/slam/odometry \
    -r /cloud_map:=/livox/lidar \
    -r /terrain_map:=/nav/terrain_map \
    -r /way_point:=/nav/way_point \
    -p pathFolder:="$PATHS_DIR" \
    -p autonomyMode:=true \
    -p autonomySpeed:=1.0 \
    -p useTerrainAnalysis:=true \
    -p checkObstacle:=false \
    -p slopeWeight:=3.0 \
    -p twoWayDrive:=false \
    > /tmp/local_planner.log 2>&1 &
LP_PID=$!
sleep 1

# 4. pathFollower
echo "[4/4] Starting pathFollower..."
ros2 run local_planner pathFollower \
    --ros-args \
    -r /Odometry:=/slam/odometry \
    -r /cmd_vel:=/nav/cmd_vel \
    -p autonomyMode:=true \
    -p autonomySpeed:=1.0 \
    -p stuck_timeout:=10.0 \
    -p stuck_dist_thre:=0.15 \
    > /tmp/path_follower.log 2>&1 &
PF_PID=$!
sleep 2

echo "All 4 nodes started. Waiting 5s for terrain stabilization..."
sleep 5

# Send way_point
echo "=== Sending way_point to ($GOAL_X, $GOAL_Y, $GOAL_Z) ==="
ros2 topic pub /nav/way_point geometry_msgs/msg/PointStamped \
    "{header: {frame_id: 'odom'}, point: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}}" \
    --rate 4 > /dev/null 2>&1 &
WP_PID=$!

# Monitor
for i in $(seq 1 $MONITOR_SEC); do
    sleep 1
    POS=$(tail -5 /tmp/sim_bridge.log 2>/dev/null | grep -oP 'pos=\([^)]+\)' | tail -1 || echo "?")
    CMD=$(tail -5 /tmp/sim_bridge.log 2>/dev/null | grep -oP 'cmd=\([^)]+\)' | tail -1 || echo "?")
    echo "  t=${i}s: $POS $CMD"
done

kill $WP_PID 2>/dev/null

echo ""
echo "=== Final state ==="
tail -3 /tmp/sim_bridge.log
echo ""
tail -5 /tmp/path_follower.log
echo ""
echo "To clean up: kill $SIM_PID $TA_PID $LP_PID $PF_PID"
