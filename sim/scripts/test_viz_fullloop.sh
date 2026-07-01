#!/bin/bash
# Kill old processes
pkill -9 -f sim_viz_full 2>/dev/null || true
pkill -9 -f mujoco_ros2_bridge 2>/dev/null || true
pkill -9 -f terrainAnalysis 2>/dev/null || true
pkill -9 -f localPlanner 2>/dev/null || true
pkill -9 -f pathFollower 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
source /home/sunrise/data/SLAM/navigation/install/setup.bash 2>/dev/null || true

echo "=== Launching MuJoCo 3D Viz ==="
DISPLAY=:0 python3 /tmp/sim_viz_full.py > /tmp/sim_viz.log 2>&1 &
VIZ_PID=$!
sleep 4

if ! kill -0 $VIZ_PID 2>/dev/null; then
    echo "ERROR: viz died!"
    cat /tmp/sim_viz.log
    exit 1
fi

echo "Viz started (PID=$VIZ_PID)"

# Start terrain_analysis
echo "Starting terrain_analysis..."
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

# Start localPlanner
echo "Starting localPlanner..."
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
    -p checkObstacle:=true \
    -p useCost:=true \
    -p slopeWeight:=3.0 \
    -p twoWayDrive:=false \
    > /tmp/local_planner.log 2>&1 &
LP_PID=$!
sleep 1

# Start pathFollower
echo "Starting pathFollower..."
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
sleep 3

echo "All nodes started. Sending waypoint..."

# Send waypoint to goal (22, 12, 0.5)
ros2 topic pub /nav/way_point geometry_msgs/msg/PointStamped \
    "{header: {frame_id: 'odom'}, point: {x: 22.0, y: 12.0, z: 0.5}}" \
    --rate 4 > /dev/null 2>&1 &
WP_PID=$!

echo ""
echo "=== All running ==="
echo "VIZ=$VIZ_PID TA=$TA_PID LP=$LP_PID PF=$PF_PID WP=$WP_PID"
echo "Logs: /tmp/sim_viz.log /tmp/terrain_analysis.log /tmp/local_planner.log /tmp/path_follower.log"
echo ""

# Monitor for 30s
for i in $(seq 1 30); do
    sleep 1
    POS=$(tail -5 /tmp/sim_viz.log 2>/dev/null | grep -oP 'pos=\([^)]+\)' | tail -1)
    CMD=$(tail -5 /tmp/sim_viz.log 2>/dev/null | grep -oP 'cmd=\([^)]+\)' | tail -1)
    if [ -n "$POS" ]; then
        echo "  t=${i}s: $POS $CMD"
    fi
done

echo ""
echo "=== Final state ==="
tail -3 /tmp/sim_viz.log
echo "---"
tail -3 /tmp/path_follower.log
echo ""
echo "PIDs still running: VIZ=$VIZ_PID TA=$TA_PID LP=$LP_PID PF=$PF_PID"
