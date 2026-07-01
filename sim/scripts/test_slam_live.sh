#!/bin/bash
# ============================================================
# SLAM 实时质量评估 — 用真实 LiDAR 验证 Point-LIO / Fast-LIO2
# ============================================================
# 用法:
#   bash test_slam_live.sh [duration_sec] [slam_profile]
#     duration_sec: 测试持续时间 (默认 60)
#     slam_profile: pointlio | fastlio2 (默认 pointlio)
#
# 测试指标:
#   1. SLAM 存活性 (无崩溃)
#   2. 里程计输出频率
#   3. 位置稳定性 (静止时漂移量)
#   4. IMU 输入频率
#   5. 点云输出频率
#   6. 轨迹记录
# ============================================================

set -e
source /opt/ros/humble/setup.bash
source /home/sunrise/data/SLAM/navigation/install/setup.bash

NAV_DIR=/home/sunrise/data/SLAM/navigation
DURATION="${1:-60}"
SLAM_PROFILE="${2:-pointlio}"

echo "============================================"
echo "  SLAM 实时质量评估"
echo "  持续时间: ${DURATION}s"
echo "  SLAM:     ${SLAM_PROFILE}"
echo "  时间:     $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================"
echo ""

# 清理残留
echo "[准备] 清理残留进程..."
pkill -f pointlio_node 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
pkill -f livox_ros_driver2_node 2>/dev/null || true
pkill -f nova_nav_bridge 2>/dev/null || true
sleep 2

# ---- 1. 启动 LiDAR ----
echo "[1/4] 启动 Livox Mid-360..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py 2>/dev/null &
LIDAR_PID=$!
echo "  等待 LiDAR 初始化 (8s)..."
sleep 8

# 验证 LiDAR (用 topic list + hz 检测，避免 CustomMsg echo 超时)
LIDAR_OK=false
LIDAR_TOPICS=$(ros2 topic list 2>/dev/null)
if echo "$LIDAR_TOPICS" | grep -q "/livox/lidar"; then
    LIDAR_OK=true
    echo "  LiDAR: OK (话题 /livox/lidar 已注册)"
elif echo "$LIDAR_TOPICS" | grep -q "/lidar/scan"; then
    LIDAR_OK=true
    echo "  LiDAR: OK (话题 /lidar/scan 已注册)"
fi

if [ "$LIDAR_OK" = false ]; then
    echo "  [ERROR] LiDAR 话题未注册，检查硬件连接"
    echo "  当前话题: $(echo "$LIDAR_TOPICS" | tr '\n' ' ')"
    kill $LIDAR_PID 2>/dev/null
    exit 1
fi

# 检查 IMU
if echo "$LIDAR_TOPICS" | grep -q "/livox/imu"; then
    echo "  IMU:   OK (话题 /livox/imu 已注册)"
elif echo "$LIDAR_TOPICS" | grep -q "/imu/data"; then
    echo "  IMU:   OK (话题 /imu/data 已注册)"
else
    echo "  [WARN] IMU 话题未发现"
fi

# ---- 2. 启动 SLAM ----
echo ""
echo "[2/4] 启动 SLAM (${SLAM_PROFILE})..."

if [ "$SLAM_PROFILE" = "pointlio" ]; then
    CONFIG_FILE="$NAV_DIR/config/pointlio.yaml"
    ros2 run pointlio pointlio_node --ros-args \
        --params-file "$CONFIG_FILE" \
        -r cloud_registered_body:=/slam/registered_cloud \
        -r cloud_registered:=/slam/map_cloud \
        -r aft_mapped_to_init:=/slam/odometry \
        -r livox/lidar:=/lidar/scan \
        -r livox/imu:=/imu/data \
        > /tmp/slam_test.log 2>&1 &
    SLAM_PID=$!
elif [ "$SLAM_PROFILE" = "fastlio2" ]; then
    CONFIG_FILE="$NAV_DIR/install/fastlio2/share/fastlio2/config/lio.yaml"
    ros2 run fastlio2 lio_node --ros-args \
        -p config_path:="$CONFIG_FILE" \
        -r /Odometry:=/slam/odometry \
        -r /cloud_registered_body:=/slam/registered_cloud \
        -r /cloud_registered:=/slam/map_cloud \
        > /tmp/slam_test.log 2>&1 &
    SLAM_PID=$!
else
    echo "[ERROR] 未知 SLAM profile: $SLAM_PROFILE"
    kill $LIDAR_PID 2>/dev/null
    exit 1
fi

sleep 5

if ! kill -0 $SLAM_PID 2>/dev/null; then
    echo "[ERROR] SLAM 启动失败!"
    echo "最后 20 行日志:"
    tail -20 /tmp/slam_test.log
    kill $LIDAR_PID 2>/dev/null
    exit 1
fi
echo "  SLAM PID: $SLAM_PID"

# ---- 3. 等待 SLAM 初始化 ----
echo ""
echo "[3/4] 等待 SLAM 初始化..."
INIT_OK=false
for i in $(seq 1 15); do
    if timeout 2 ros2 topic echo /slam/odometry --once >/dev/null 2>&1; then
        INIT_OK=true
        echo "  SLAM 初始化完成 (${i}s)"
        break
    fi
    echo "  等待... (${i}/15)"
    sleep 1
done

if [ "$INIT_OK" = false ]; then
    echo "[ERROR] SLAM 初始化超时 (15s)"
    echo "最后 20 行日志:"
    tail -20 /tmp/slam_test.log
    kill $SLAM_PID $LIDAR_PID 2>/dev/null
    exit 1
fi

# ---- 4. 监控和记录 ----
echo ""
echo "[4/4] 监控 SLAM 输出 (${DURATION}s)..."
echo ""
echo " SEC | POS_X    | POS_Y    | POS_Z    | STATUS"
echo "-----|----------|----------|----------|--------"

TRAJECTORY_FILE="/tmp/slam_live_trajectory.csv"
echo "t,x,y,z,qx,qy,qz,qw" > $TRAJECTORY_FILE

CRASH=false
SAMPLE_INTERVAL=3
NUM_SAMPLES=$((DURATION / SAMPLE_INTERVAL))

# 记录初始位置用于漂移检测
INIT_X=""
INIT_Y=""
INIT_Z=""
MAX_DRIFT=0

for i in $(seq 1 $NUM_SAMPLES); do
    sleep $SAMPLE_INTERVAL

    # 检查 SLAM 是否还活着
    if ! kill -0 $SLAM_PID 2>/dev/null; then
        CRASH=true
        SEC=$((i * SAMPLE_INTERVAL))
        echo "[CRASH] SLAM 在 ${SEC}s 处崩溃!"
        break
    fi

    # 采集里程计
    ODOM_MSG=$(timeout 2 ros2 topic echo /slam/odometry --once 2>/dev/null || true)
    if [ -z "$ODOM_MSG" ]; then
        SEC=$((i * SAMPLE_INTERVAL))
        printf "%4ds | ---      | ---      | ---      | NO_DATA\n" "$SEC"
        continue
    fi

    POS_X=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "x:" | head -1 | awk '{print $2}')
    POS_Y=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "y:" | head -1 | awk '{print $2}')
    POS_Z=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "z:" | head -1 | awk '{print $2}')

    # 四元数
    QX=$(echo "$ODOM_MSG" | grep -A 6 "orientation:" | grep "x:" | head -1 | awk '{print $2}')
    QY=$(echo "$ODOM_MSG" | grep -A 6 "orientation:" | grep "y:" | head -1 | awk '{print $2}')
    QZ=$(echo "$ODOM_MSG" | grep -A 6 "orientation:" | grep "z:" | head -1 | awk '{print $2}')
    QW=$(echo "$ODOM_MSG" | grep -A 6 "orientation:" | grep "w:" | head -1 | awk '{print $2}')

    SEC=$((i * SAMPLE_INTERVAL))

    # 漂移检测 (静止时应接近0)
    if [ -z "$INIT_X" ]; then
        INIT_X="${POS_X:-0}"
        INIT_Y="${POS_Y:-0}"
        INIT_Z="${POS_Z:-0}"
    fi

    # 计算累积漂移 (用 awk 做浮点运算)
    DRIFT=$(echo "$POS_X $POS_Y $POS_Z $INIT_X $INIT_Y $INIT_Z" | awk '{
        dx=$1-$4; dy=$2-$5; dz=$3-$6;
        printf "%.4f", sqrt(dx*dx+dy*dy+dz*dz)
    }')

    # 更新最大漂移
    MAX_DRIFT=$(echo "$DRIFT $MAX_DRIFT" | awk '{if($1>$2) print $1; else print $2}')

    printf "%4ds | %8.3f | %8.3f | %8.3f | OK (drift=%.3fm)\n" \
        "$SEC" "${POS_X:-0}" "${POS_Y:-0}" "${POS_Z:-0}" "$DRIFT"

    # 记录轨迹
    echo "$SEC,${POS_X:-0},${POS_Y:-0},${POS_Z:-0},${QX:-0},${QY:-0},${QZ:-0},${QW:-1}" >> $TRAJECTORY_FILE
done

# ---- 频率测量 ----
echo ""
echo "测量话题频率 (5s 采样)..."

ODOM_HZ=$(timeout 5 ros2 topic hz /slam/odometry 2>/dev/null | grep "average rate:" | tail -1 | awk '{print $3}' || echo "?")
CLOUD_HZ=$(timeout 5 ros2 topic hz /slam/registered_cloud 2>/dev/null | grep "average rate:" | tail -1 | awk '{print $3}' || echo "?")
IMU_HZ=$(timeout 5 ros2 topic hz /livox/imu 2>/dev/null | grep "average rate:" | tail -1 | awk '{print $3}' || echo "?")
LIDAR_HZ=$(timeout 5 ros2 topic hz /livox/lidar 2>/dev/null | grep "average rate:" | tail -1 | awk '{print $3}' || echo "?")

# ---- 录制 10s bag 用于后续测试 ----
echo ""
echo "录制 10s 数据包..."
BAG_DIR="/tmp/slam_test_bag_$(date +%Y%m%d_%H%M%S)"
timeout 12 ros2 bag record \
    /livox/lidar /livox/imu /slam/odometry /slam/registered_cloud \
    -o "$BAG_DIR" 2>/dev/null &
RECORD_PID=$!
sleep 12
kill $RECORD_PID 2>/dev/null || true
wait $RECORD_PID 2>/dev/null || true

BAG_SIZE="?"
if [ -d "$BAG_DIR" ]; then
    BAG_SIZE=$(du -sh "$BAG_DIR" 2>/dev/null | awk '{print $1}')
fi

# ---- 结果汇总 ----
echo ""
echo "============================================"
echo "  SLAM 实时测试结果"
echo "============================================"

if [ "$CRASH" = true ]; then
    echo "  状态:       SLAM CRASHED"
    echo "  日志:       /tmp/slam_test.log"
    echo ""
    echo "  最后 20 行日志:"
    tail -20 /tmp/slam_test.log
    RESULT="FAIL"
else
    echo "  状态:       SLAM SURVIVED (${DURATION}s)"

    # 最终位姿
    FINAL_MSG=$(timeout 2 ros2 topic echo /slam/odometry --once 2>/dev/null || true)
    FX=$(echo "$FINAL_MSG" | grep -A 3 "position:" | grep "x:" | head -1 | awk '{print $2}')
    FY=$(echo "$FINAL_MSG" | grep -A 3 "position:" | grep "y:" | head -1 | awk '{print $2}')
    FZ=$(echo "$FINAL_MSG" | grep -A 3 "position:" | grep "z:" | head -1 | awk '{print $2}')
    echo "  最终位置:   (${FX:-?}, ${FY:-?}, ${FZ:-?})"
    echo "  最大漂移:   ${MAX_DRIFT}m"
    RESULT="PASS"
fi

echo ""
echo "  话题频率:"
echo "    LiDAR 输入:    ${LIDAR_HZ} Hz (期望 ~10)"
echo "    IMU 输入:      ${IMU_HZ} Hz (期望 ~200)"
echo "    Odometry 输出: ${ODOM_HZ} Hz (期望 ~10)"
echo "    Cloud 输出:    ${CLOUD_HZ} Hz (期望 ~10)"
echo ""
echo "  轨迹文件:   $TRAJECTORY_FILE"
echo "  SLAM 日志:  /tmp/slam_test.log"
echo "  录制数据包: $BAG_DIR ($BAG_SIZE)"
echo ""

# 质量判定
echo "  质量判定:"
QUALITY_OK=true

# 检查漂移 (静止时应 < 0.5m)
DRIFT_OK=$(echo "$MAX_DRIFT" | awk '{if($1 < 0.5) print "true"; else print "false"}')
if [ "$DRIFT_OK" = "true" ]; then
    echo "    [PASS] 位置漂移 < 0.5m (静止)"
else
    echo "    [WARN] 位置漂移 >= 0.5m: ${MAX_DRIFT}m"
    QUALITY_OK=false
fi

# 检查频率
echo "    [INFO] 里程计频率: ${ODOM_HZ} Hz"

if [ "$RESULT" = "PASS" ]; then
    echo ""
    echo "  结论: SLAM ${SLAM_PROFILE} 实时测试 PASS"
else
    echo ""
    echo "  结论: SLAM ${SLAM_PROFILE} 实时测试 FAIL"
fi

echo "============================================"

# 清理
echo ""
echo "清理进程..."
kill $SLAM_PID $LIDAR_PID 2>/dev/null || true
wait $SLAM_PID $LIDAR_PID 2>/dev/null || true
echo "完成."
