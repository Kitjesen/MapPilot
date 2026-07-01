#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/sunrise/data/SLAM/navigation/install/setup.bash
NAV_DIR=/home/sunrise/data/SLAM/navigation
BAG_DIR=/home/sunrise/data/slam_datasets/legkilo_corridor

echo "============================================================"
echo "  SLAM Dataset Robustness Test - Point-LIO + Leg-KILO"
echo "  Dataset: Unitree Go1 quadruped + VLP-16 (corridor, vibration)"
echo "  Time: $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

pkill -f pointlio_node 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
pkill -f "ros2 bag play" 2>/dev/null || true
pkill -f nova_nav_bridge 2>/dev/null || true
sleep 2

BASE_CONFIG="$NAV_DIR/install/pointlio/share/pointlio/config/velody16.yaml"
CONFIG_FILE="/tmp/vlp16_legkilo.yaml"
sed -e 's/scan_line: 32/scan_line: 16/' \
    -e 's|lid_topic:.*|lid_topic: "/lidar/raw_frame"|' \
    -e 's|imu_topic:.*|imu_topic: "/imu/raw"|' \
    "$BASE_CONFIG" > "$CONFIG_FILE"

echo "[1/3] Starting Point-LIO..."
ros2 run pointlio pointlio_node --ros-args \
    --params-file "$CONFIG_FILE" \
    -r cloud_registered_body:=/test/registered_cloud \
    -r cloud_registered:=/test/map_cloud \
    -r aft_mapped_to_init:=/test/odometry \
    > /tmp/slam_dataset_test.log 2>&1 &
SLAM_PID=$!
sleep 5

if ! kill -0 $SLAM_PID 2>/dev/null; then
    echo "[ERROR] Point-LIO failed!"
    cat /tmp/slam_dataset_test.log
    exit 1
fi
echo "  PID: $SLAM_PID"

echo "[2/3] Playing bag..."
ros2 bag play "$BAG_DIR" --rate 1.0 &
PLAY_PID=$!
sleep 3

echo "Waiting for init..."
for i in $(seq 1 30); do
    ODOM=$(timeout 3 ros2 topic echo /test/odometry --once --no-arr 2>/dev/null | head -3)
    if [ -n "$ODOM" ]; then
        echo "  Init OK (${i}s)"
        break
    fi
    sleep 1
done

echo ""
echo "[3/3] Monitoring..."
echo "t,x,y,z" > /tmp/slam_dataset_trajectory.csv

CRASH=false
INIT_X="" ; INIT_Y="" ; INIT_Z=""
MAX_DRIFT=0
TOTAL=0 ; OK_COUNT=0

for i in $(seq 1 100); do
    sleep 5
    if ! kill -0 $SLAM_PID 2>/dev/null; then
        CRASH=true
        echo "CRASH at $((i*5))s"
        break
    fi
    if ! kill -0 $PLAY_PID 2>/dev/null; then
        echo "BAG_DONE at $((i*5))s"
        break
    fi
    TOTAL=$((TOTAL+1))
    ODOM_MSG=$(timeout 3 ros2 topic echo /test/odometry --once 2>/dev/null || true)
    if [ -z "$ODOM_MSG" ]; then
        printf "%4ds | NO_DATA\n" "$((i*5))"
        continue
    fi
    OK_COUNT=$((OK_COUNT+1))
    PX=$(echo "$ODOM_MSG" | grep -A3 "position:" | grep "x:" | head -1 | awk '{print $2}')
    PY=$(echo "$ODOM_MSG" | grep -A3 "position:" | grep "y:" | head -1 | awk '{print $2}')
    PZ=$(echo "$ODOM_MSG" | grep -A3 "position:" | grep "z:" | head -1 | awk '{print $2}')
    SEC=$((i*5))
    if [ -z "$INIT_X" ]; then INIT_X="${PX:-0}"; INIT_Y="${PY:-0}"; INIT_Z="${PZ:-0}"; fi
    DRIFT=$(echo "$PX $PY $PZ $INIT_X $INIT_Y $INIT_Z" | awk '{dx=$1-$4;dy=$2-$5;dz=$3-$6;printf "%.3f",sqrt(dx*dx+dy*dy+dz*dz)}')
    MAX_DRIFT=$(echo "$DRIFT $MAX_DRIFT" | awk '{if($1>$2) print $1; else print $2}')
    printf "%4ds | %8.3f | %8.3f | %8.3f | drift=%sm\n" "$SEC" "${PX:-0}" "${PY:-0}" "${PZ:-0}" "$DRIFT"
    echo "$SEC,${PX:-0},${PY:-0},${PZ:-0}" >> /tmp/slam_dataset_trajectory.csv
done

echo ""
ODOM_HZ=$(timeout 6 ros2 topic hz /test/odometry 2>&1 | grep "average rate:" | tail -1 | awk '{print $3}' || echo "?")
echo "Hz: $ODOM_HZ"
echo "MaxDrift: $MAX_DRIFT"
echo "Samples: $OK_COUNT/$TOTAL"
echo "Crash: $CRASH"
echo ""
echo "=== TRAJECTORY ==="
cat /tmp/slam_dataset_trajectory.csv
echo ""
echo "=== LOG ==="
cat /tmp/slam_dataset_test.log

kill $SLAM_PID $PLAY_PID 2>/dev/null || true
wait $SLAM_PID $PLAY_PID 2>/dev/null || true
echo "DONE"
