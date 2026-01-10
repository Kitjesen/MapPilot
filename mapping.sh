#!/bin/bash
# ========================================
# SLAM 建图启动脚本 (ROS 2 Humble)
# ========================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 获取脚本所在目录（工作空间根目录）
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   SLAM 建图系统启动${NC}"
echo -e "${BLUE}========================================${NC}"

# 检查是否已经 source 环境
if [ -f "install/setup.bash" ]; then
    echo -e "${GREEN}[1/5] Source ROS 2 环境...${NC}"
    source install/setup.bash
else
    echo -e "${RED}错误: 找不到 install/setup.bash${NC}"
    echo -e "${YELLOW}请先编译工作空间: colcon build${NC}"
    exit 1
fi

# 设置 FastDDS 共享内存配置
export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE_DIR/fastdds_no_shm.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo -e "${GREEN}[2/5] 环境配置完成${NC}"
echo -e "  - Workspace: $WORKSPACE_DIR"
echo -e "  - FastDDS: 禁用共享内存"
echo ""

# 传感器选择
echo -e "${YELLOW}请选择传感器类型:${NC}"
echo -e "  ${GREEN}1)${NC} Orbbec Gemini 330 相机 (深度 + RGB)"
echo -e "  ${GREEN}2)${NC} Livox 激光雷达 (Mid-360/Avia)"
echo -e "  ${GREEN}3)${NC} 跳过传感器 (使用已有数据/其他传感器)"
read -p "输入选项 [1-3]: " SENSOR_CHOICE

# 启动传感器
case $SENSOR_CHOICE in
    1)
        echo -e "${GREEN}[3/5] 启动 Orbbec Gemini 330 相机...${NC}"
        gnome-terminal --tab --title="Orbbec Camera" -- bash -c "
            cd $WORKSPACE_DIR
            source install/setup.bash
            export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE_DIR/fastdds_no_shm.xml
            ros2 launch orbbec_camera gemini_330_series.launch.py
            exec bash
        " &
        sleep 3
        ;;
    2)
        echo -e "${GREEN}[3/5] 启动 Livox 激光雷达...${NC}"
        echo -e "${YELLOW}提示: 确保已连接 Livox 设备并配置好 IP${NC}"
        gnome-terminal --tab --title="Livox Driver" -- bash -c "
            cd $WORKSPACE_DIR
            source install/setup.bash
            export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE_DIR/fastdds_no_shm.xml
            ros2 launch livox_ros_driver2 msg_MID360_launch.py
            exec bash
        " &
        sleep 3
        ;;
    3)
        echo -e "${YELLOW}[3/5] 跳过传感器启动${NC}"
        ;;
    *)
        echo -e "${RED}无效选项，跳过传感器启动${NC}"
        ;;
esac

# 启动 FAST-LIO2 (SLAM 前端)
echo -e "${GREEN}[4/5] 启动 FAST-LIO2 (SLAM)...${NC}"
gnome-terminal --tab --title="FAST-LIO2" -- bash -c "
    cd $WORKSPACE_DIR
    source install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE_DIR/fastdds_no_shm.xml
    ros2 launch fastlio2 lio_launch.py
    exec bash
" &
sleep 2

# 启动 PGO (后端优化)
echo -e "${GREEN}[5/5] 启动 PGO (位姿图优化)...${NC}"
gnome-terminal --tab --title="PGO Backend" -- bash -c "
    cd $WORKSPACE_DIR
    source install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE_DIR/fastdds_no_shm.xml
    ros2 launch pgo pgo_launch.py
    exec bash
" &
sleep 2

# 启动 RViz2 可视化
echo -e "${GREEN}启动 RViz2 可视化...${NC}"
gnome-terminal --tab --title="RViz2 - Mapping" -- bash -c "
    cd $WORKSPACE_DIR
    source install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE_DIR/fastdds_no_shm.xml
    rviz2 -d src/slam/pgo/rviz/pgo.rviz
    exec bash
" &

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}✓ 建图系统已启动！${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}使用说明:${NC}"
echo -e "  1. 在 RViz2 中查看实时建图效果"
echo -e "  2. 移动机器人/传感器进行环境扫描"
echo -e "  3. 建图完成后，运行 ${GREEN}./save_map.sh${NC} 保存地图"
echo -e "  4. 按 Ctrl+C 可在各终端窗口中停止对应节点"
echo ""
echo -e "${YELLOW}话题查看:${NC}"
echo -e "  - 激光雷达点云: ${BLUE}ros2 topic echo /livox/lidar${NC}"
echo -e "  - SLAM 里程计: ${BLUE}ros2 topic echo /Odometry${NC}"
echo -e "  - PGO 地图: ${BLUE}ros2 topic echo /pgo/map${NC}"
echo ""
