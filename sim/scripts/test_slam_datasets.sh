#!/bin/bash
# ============================================================
# SLAM 数据集鲁棒性测试 — Point-LIO / Fast-LIO2
# ============================================================
# 用法: bash test_slam_datasets.sh <bag_dir> [slam_profile] [dataset_type]
#   bag_dir:       ROS2 bag 目录 (已转换)
#   slam_profile:  pointlio | fastlio2 (默认 pointlio)
#   dataset_type:  legkilo | avia | mid360 (默认 legkilo)
#
# 数据集话题映射:
#   legkilo: /lidar/raw_frame (VLP-16 PointCloud2), /imu/raw (Imu)
#            ⚠ 仅 Point-LIO 支持 (velody16 配置), Fast-LIO2 不支持 PointCloud2
#   avia:    /livox/lidar (CustomMsg), /livox/imu (Imu) — 原生 Livox Avia
#   mid360:  /livox/lidar (CustomMsg), /livox/imu (Imu) — 原生 Livox Mid-360
#
# 注意:
#   - 不要使用 --clock 播放 bag (会导致 ros2 topic echo 超时)
#   - Fast-LIO2 仅支持 Livox CustomMsg, 不支持标准 PointCloud2
#
# 测试指标:
#   1. SLAM 是否崩溃
#   2. 里程计输出频率
#   3. 位置轨迹
#   4. 最终位姿
# ============================================================

set -e
source /opt/ros/humble/setup.bash
source /home/sunrise/data/SLAM/navigation/install/setup.bash

NAV_DIR=/home/sunrise/data/SLAM/navigation
BAG_DIR="$1"
SLAM_PROFILE="${2:-pointlio}"
DATASET_TYPE="${3:-legkilo}"

if [ -z "$BAG_DIR" ]; then
    echo "用法: $0 <bag_dir> [pointlio|fastlio2] [legkilo|avia|mid360]"
    echo ""
    echo "可用数据集:"
    ls -d /home/sunrise/data/slam_datasets/*/ 2>/dev/null
    exit 1
fi

if [ ! -d "$BAG_DIR" ]; then
    echo "[ERROR] 目录不存在: $BAG_DIR"
    exit 1
fi

echo "============================================"
echo "  SLAM 数据集鲁棒性测试"
echo "  数据集: $(basename $BAG_DIR) ($DATASET_TYPE)"
echo "  SLAM:   $SLAM_PROFILE"
echo "  时间:   $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================"
echo ""

# 清理残留
echo "[准备] 清理残留进程..."
pkill -f pointlio_node 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
pkill -f "ros2 bag play" 2>/dev/null || true
pkill -f nova_nav_bridge 2>/dev/null || true
sleep 2

# ---- 确定 SLAM 配置和话题映射 ----
REMAP_ARGS=""

# ---- 为 legkilo 数据集生成临时 VLP-16 配置 ----
if [ "$DATASET_TYPE" = "legkilo" ] && [ "$SLAM_PROFILE" = "pointlio" ]; then
    BASE_CONFIG="$NAV_DIR/install/pointlio/share/pointlio/config/velody16.yaml"
    CONFIG_FILE="/tmp/vlp16_legkilo.yaml"
    # 复制基础配置并修正: scan_line→16, 话题直接指向 bag 话题
    sed -e 's/scan_line: 32/scan_line: 16/' \
        -e 's|lid_topic:.*|lid_topic: "/lidar/raw_frame"|' \
        -e 's|imu_topic:.*|imu_topic: "/imu/raw"|' \
        "$BASE_CONFIG" > "$CONFIG_FILE"
    echo "  [INFO] 已生成 VLP-16 临时配置: $CONFIG_FILE"
fi

if [ "$SLAM_PROFILE" = "pointlio" ]; then
    case "$DATASET_TYPE" in
        legkilo)
            # CONFIG_FILE 已在上面生成, 话题已内置无需 remap
            REMAP_ARGS=""
            ;;
        avia)
            CONFIG_FILE="$NAV_DIR/config/pointlio.yaml"
            # Avia CustomMsg: topics match yaml config (livox/lidar, livox/imu)
            REMAP_ARGS=""
            ;;
        mid360)
            CONFIG_FILE="$NAV_DIR/config/pointlio.yaml"
            REMAP_ARGS=""
            ;;
        *)
            echo "[ERROR] 未知数据集类型: $DATASET_TYPE"
            exit 1
            ;;
    esac

    echo "[1/3] 启动 Point-LIO ($DATASET_TYPE 配置)..."
    echo "  配置: $CONFIG_FILE"
    ros2 run pointlio pointlio_node --ros-args \
        --params-file "$CONFIG_FILE" \
        -r cloud_registered_body:=/test/registered_cloud \
        -r cloud_registered:=/test/map_cloud \
        -r aft_mapped_to_init:=/test/odometry \
        $REMAP_ARGS \
        > /tmp/slam_dataset_test.log 2>&1 &
    SLAM_PID=$!

elif [ "$SLAM_PROFILE" = "fastlio2" ]; then
    # Fast-LIO2 仅支持 Livox CustomMsg, 不支持标准 PointCloud2
    case "$DATASET_TYPE" in
        legkilo)
            echo "[ERROR] Fast-LIO2 不支持 VLP-16 (PointCloud2) 数据集"
            echo "  Fast-LIO2 硬编码订阅 livox_ros_driver2::msg::CustomMsg"
            echo "  请改用 Point-LIO: $0 $BAG_DIR pointlio legkilo"
            exit 1
            ;;
        avia|mid360)
            REMAP_ARGS=""
            ;;
        *)
            echo "[ERROR] 未知数据集类型: $DATASET_TYPE"
            exit 1
            ;;
    esac

    CONFIG_FILE="$NAV_DIR/install/fastlio2/share/fastlio2/config/lio.yaml"

    echo "[1/3] 启动 Fast-LIO2 ($DATASET_TYPE 配置)..."
    echo "  配置: $CONFIG_FILE"
    ros2 run fastlio2 lio_node --ros-args \
        -p config_path:="$CONFIG_FILE" \
        -r /Odometry:=/test/odometry \
        -r /cloud_registered_body:=/test/registered_cloud \
        -r /cloud_registered:=/test/map_cloud \
        $REMAP_ARGS \
        > /tmp/slam_dataset_test.log 2>&1 &
    SLAM_PID=$!
else
    echo "[ERROR] 未知 SLAM profile: $SLAM_PROFILE"
    exit 1
fi

sleep 5

if ! kill -0 $SLAM_PID 2>/dev/null; then
    echo "[ERROR] SLAM 启动失败!"
    echo "最后 20 行日志:"
    tail -20 /tmp/slam_dataset_test.log
    exit 1
fi
echo "  SLAM PID: $SLAM_PID"

# ---- 播放数据集 ----
echo ""
echo "[2/3] 播放数据集..."
# 不使用 --clock: 模拟时间会导致 ros2 topic echo --once 超时
ros2 bag play "$BAG_DIR" --rate 1.0 &
PLAY_PID=$!
sleep 3

# 等待 SLAM 初始化
echo "等待 SLAM 初始化..."
INIT_OK=false
for i in $(seq 1 20); do
    if timeout 3 ros2 topic echo /test/odometry --once >/dev/null 2>&1; then
        INIT_OK=true
        echo "  SLAM 初始化完成 (${i}s)"
        break
    fi
    if ! kill -0 $SLAM_PID 2>/dev/null; then
        echo "[CRASH] SLAM 在初始化阶段崩溃!"
        tail -20 /tmp/slam_dataset_test.log
        kill $PLAY_PID 2>/dev/null
        exit 1
    fi
    sleep 1
done

if [ "$INIT_OK" = false ]; then
    echo "[WARN] SLAM 初始化超时 (20s), 继续监控..."
fi

# ---- 监控 ----
echo ""
echo "[3/3] 监控 SLAM 输出..."
echo ""
echo " SEC | POS_X    | POS_Y    | POS_Z    | STATUS"
echo "-----|----------|----------|----------|--------"

TRAJECTORY_FILE="/tmp/slam_dataset_trajectory.csv"
echo "t,x,y,z" > $TRAJECTORY_FILE

CRASH=false
SAMPLE_INTERVAL=5
MAX_SAMPLES=60  # 最多监控 300s

INIT_X=""
INIT_Y=""
INIT_Z=""
MAX_DRIFT=0
LAST_X=""
LAST_Y=""

for i in $(seq 1 $MAX_SAMPLES); do
    sleep $SAMPLE_INTERVAL

    # 检查 SLAM 是否还活着
    if ! kill -0 $SLAM_PID 2>/dev/null; then
        CRASH=true
        SEC=$((i * SAMPLE_INTERVAL))
        echo "[CRASH] SLAM 在 ${SEC}s 处崩溃!"
        break
    fi

    # 检查 bag 是否还在播放
    if ! kill -0 $PLAY_PID 2>/dev/null; then
        echo "[INFO] 数据集播放完毕"
        break
    fi

    # 采集里程计
    ODOM_MSG=$(timeout 3 ros2 topic echo /test/odometry --once 2>/dev/null || true)
    if [ -z "$ODOM_MSG" ]; then
        SEC=$((i * SAMPLE_INTERVAL))
        printf "%4ds | ---      | ---      | ---      | NO_DATA\n" "$SEC"
        continue
    fi

    POS_X=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "x:" | head -1 | awk '{print $2}')
    POS_Y=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "y:" | head -1 | awk '{print $2}')
    POS_Z=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "z:" | head -1 | awk '{print $2}')

    SEC=$((i * SAMPLE_INTERVAL))

    if [ -z "$INIT_X" ]; then
        INIT_X="${POS_X:-0}"
        INIT_Y="${POS_Y:-0}"
        INIT_Z="${POS_Z:-0}"
    fi

    DRIFT=$(echo "$POS_X $POS_Y $POS_Z $INIT_X $INIT_Y $INIT_Z" | awk '{
        dx=$1-$4; dy=$2-$5; dz=$3-$6;
        printf "%.3f", sqrt(dx*dx+dy*dy+dz*dz)
    }')
    MAX_DRIFT=$(echo "$DRIFT $MAX_DRIFT" | awk '{if($1>$2) print $1; else print $2}')

    printf "%4ds | %8.3f | %8.3f | %8.3f | OK (drift=%.2fm)\n" \
        "$SEC" "${POS_X:-0}" "${POS_Y:-0}" "${POS_Z:-0}" "$DRIFT"

    echo "$SEC,${POS_X:-0},${POS_Y:-0},${POS_Z:-0}" >> $TRAJECTORY_FILE
    LAST_X="$POS_X"
    LAST_Y="$POS_Y"
done

# ---- 频率测量 ----
echo ""
echo "测量话题频率 (5s 采样)..."
ODOM_HZ=$(timeout 5 ros2 topic hz /test/odometry 2>/dev/null | grep "average rate:" | tail -1 | awk '{print $3}' || echo "?")

# ---- 结果汇总 ----
echo ""
echo "============================================"
echo "  SLAM 数据集测试结果"
echo "============================================"

if [ "$CRASH" = true ]; then
    echo "  状态:       SLAM CRASHED"
    echo "  日志:       /tmp/slam_dataset_test.log"
    echo ""
    echo "  最后 20 行日志:"
    tail -20 /tmp/slam_dataset_test.log
    RESULT="FAIL"
else
    echo "  状态:       SLAM SURVIVED"
    FINAL_MSG=$(timeout 3 ros2 topic echo /test/odometry --once 2>/dev/null || true)
    FX=$(echo "$FINAL_MSG" | grep -A 3 "position:" | grep "x:" | head -1 | awk '{print $2}')
    FY=$(echo "$FINAL_MSG" | grep -A 3 "position:" | grep "y:" | head -1 | awk '{print $2}')
    FZ=$(echo "$FINAL_MSG" | grep -A 3 "position:" | grep "z:" | head -1 | awk '{print $2}')
    echo "  最终位置:   (${FX:-?}, ${FY:-?}, ${FZ:-?})"
    echo "  最大位移:   ${MAX_DRIFT}m"
    RESULT="PASS"
fi

echo "  里程计频率: ${ODOM_HZ} Hz"
echo "  轨迹文件:   $TRAJECTORY_FILE"
echo "  SLAM 日志:  /tmp/slam_dataset_test.log"
echo ""
echo "  结论: SLAM ${SLAM_PROFILE} + ${DATASET_TYPE} 数据集 → ${RESULT}"
echo "============================================"

# 清理
kill $SLAM_PID $PLAY_PID 2>/dev/null || true
wait $SLAM_PID $PLAY_PID 2>/dev/null || true
