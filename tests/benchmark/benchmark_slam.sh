#!/bin/bash
# SLAM 性能基准测试
# 测试 FAST-LIO2 的处理速度和资源占用

echo "SLAM 性能测试"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 检查是否有 rosbag 数据
ROSBAG_DIR="$HOME/rosbags"
if [ ! -d "$ROSBAG_DIR" ] || [ -z "$(ls -A $ROSBAG_DIR 2>/dev/null)" ]; then
    echo "⚠️  未找到 rosbag 数据，跳过 SLAM 性能测试"
    echo "   提示: 将测试数据放到 $ROSBAG_DIR"
    echo ""
    return 0
fi

# 检查 ROS 2 环境
if ! command -v ros2 &> /dev/null; then
    echo "⚠️  ROS 2 未安装，跳过测试"
    return 0
fi

source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

# 查找第一个 rosbag
ROSBAG=$(find "$ROSBAG_DIR" -name "*.db3" | head -1)
if [ -z "$ROSBAG" ]; then
    echo "⚠️  未找到有效的 rosbag 文件"
    return 0
fi

ROSBAG_DIR=$(dirname "$ROSBAG")
echo "使用 rosbag: $ROSBAG_DIR"

# 启动 SLAM 节点（后台）
echo "启动 SLAM 节点..."
ros2 launch slam/fastlio2/launch/lio_launch.py > /dev/null 2>&1 &
SLAM_PID=$!

# 等待节点启动
sleep 5

# 播放 rosbag 并计时
echo "播放 rosbag..."
START=$(date +%s.%N)
ros2 bag play "$ROSBAG_DIR" --rate 1.0 > /dev/null 2>&1
END=$(date +%s.%N)

# 计算处理时间
DURATION=$(echo "$END - $START" | bc)
echo "  处理时间: ${DURATION}s"

# 获取 CPU 和内存使用情况
if command -v ps &> /dev/null; then
    CPU=$(ps -p $SLAM_PID -o %cpu --no-headers 2>/dev/null || echo "N/A")
    MEM=$(ps -p $SLAM_PID -o %mem --no-headers 2>/dev/null || echo "N/A")
    echo "  CPU 使用率: ${CPU}%"
    echo "  内存使用率: ${MEM}%"
fi

# 清理
kill $SLAM_PID 2>/dev/null || true
wait $SLAM_PID 2>/dev/null || true

echo ""
