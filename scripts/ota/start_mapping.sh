#!/bin/bash
# ============================================================================
# start_mapping.sh — 建图系统 systemd 启动入口
#
# 此脚本由 mapping.service 调用，负责:
#   1. source ROS 2 环境
#   2. source colcon install 环境
#   3. 设置 FastDDS 配置
#   4. 启动建图 launch 文件 (LiDAR + FASTLIO2 + PGO)
#
# 安装到机器人:
#   sudo cp scripts/ota/start_mapping.sh /opt/robot/navigation/start_mapping.sh
#   sudo chmod +x /opt/robot/navigation/start_mapping.sh
# ============================================================================
set -e

NAV_DIR="/opt/robot/navigation/current"

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source 导航工作空间 (建图节点也在同一 colcon workspace)
if [ -f "$NAV_DIR/install/setup.bash" ]; then
    source "$NAV_DIR/install/setup.bash"
else
    echo "ERROR: $NAV_DIR/install/setup.bash 不存在"
    exit 1
fi

# FastDDS 配置
if [ -f "$NAV_DIR/fastdds_no_shm.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$NAV_DIR/fastdds_no_shm.xml"
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "Starting Mapping System (LiDAR + FASTLIO2 + PGO)..."

# 启动建图系统 (mapping_launch.py)
exec ros2 launch "$NAV_DIR/launch/mapping_launch.py"
