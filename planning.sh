#!/bin/bash
# ========================================
# 3D 全局规划系统启动脚本 (ROS 2 Humble)
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
echo -e "${BLUE}   3D 全局规划系统启动 (PCT Planner)${NC}"
echo -e "${BLUE}========================================${NC}"

# 检查是否已经 source 环境
if [ -f "install/setup.bash" ]; then
    echo -e "${GREEN}[1/6] Source ROS 2 环境...${NC}"
    source install/setup.bash
else
    echo -e "${RED}错误: 找不到 install/setup.bash${NC}"
    echo -e "${YELLOW}请先编译工作空间: colcon build${NC}"
    exit 1
fi

# 设置 FastDDS 共享内存配置
export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE_DIR/fastdds_no_shm.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo -e "${GREEN}[2/6] 环境配置完成${NC}"
echo -e "  - Workspace: $WORKSPACE_DIR"
echo -e "  - FastDDS: 禁用共享内存"
echo ""

# 检查地图文件
TOMOGRAM_DIR="$WORKSPACE_DIR/src/global_planning/PCT_planner/rsc/tomogram"
if [ ! -d "$TOMOGRAM_DIR" ] || [ -z "$(ls -A $TOMOGRAM_DIR/*.pickle 2>/dev/null)" ]; then
    echo -e "${RED}警告: 未找到地图文件！${NC}"
    echo -e "${YELLOW}请先运行建图系统生成地图，或手动放置 .pickle 地图文件到:${NC}"
    echo -e "  $TOMOGRAM_DIR"
    echo ""
    read -p "是否继续启动（可能无法规划）? [y/N]: " CONTINUE
    if [[ ! "$CONTINUE" =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}已取消启动${NC}"
        exit 1
    fi
fi

# 定位模式选择
echo -e "${YELLOW}请选择定位模式:${NC}"
echo -e "  ${GREEN}1)${NC} 假定位 (Fake Localization) - 使用 RViz '2D Pose Estimate' 设置位置"
echo -e "  ${GREEN}2)${NC} 真定位 (FAST-LIO2 Localization) - 需要已有地图"
echo -e "  ${GREEN}3)${NC} 跳过定位 (使用外部定位系统)"
read -p "输入选项 [1-3]: " LOC_CHOICE

# 启动传感器 (可选)
echo ""
echo -e "${YELLOW}是否启动传感器?${NC}"
echo -e "  ${GREEN}1)${NC} Orbbec Gemini 330 相机"
echo -e "  ${GREEN}2)${NC} Livox 激光雷达"
echo -e "  ${GREEN}3)${NC} 跳过 (已有传感器或不需要)"
read -p "输入选项 [1-3]: " SENSOR_CHOICE

case $SENSOR_CHOICE in
    1)
        echo -e "${GREEN}[3/6] 启动 Orbbec Gemini 330 相机...${NC}"
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
        echo -e "${GREEN}[3/6] 启动 Livox 激光雷达...${NC}"
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
        echo -e "${YELLOW}[3/6] 跳过传感器启动${NC}"
        ;;
esac

# 启动定位系统
case $LOC_CHOICE in
    1)
        echo -e "${GREEN}[4/6] 启动假定位 (Fake Localization)...${NC}"
        echo -e "${YELLOW}  ⚠ 启动后，在 RViz 中使用 '2D Pose Estimate' 工具设置机器人初始位置${NC}"
        gnome-terminal --tab --title="Fake Localization" -- bash -c "
            cd $WORKSPACE_DIR
            source install/setup.bash
            export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE_DIR/fastdds_no_shm.xml
            python3 src/global_planning/PCT_planner/planner/scripts/fake_localization.py
            exec bash
        " &
        sleep 2
        ;;
    2)
        echo -e "${GREEN}[4/6] 启动 FAST-LIO2 定位...${NC}"
        gnome-terminal --tab --title="FAST-LIO2 Localization" -- bash -c "
            cd $WORKSPACE_DIR
            source install/setup.bash
            export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE_DIR/fastdds_no_shm.xml
            ros2 launch fastlio2 lio_launch.py
            exec bash
        " &
        sleep 2
        ;;
    3)
        echo -e "${YELLOW}[4/6] 跳过定位系统启动${NC}"
        ;;
esac

# 启动 PCT 全局规划器
echo -e "${GREEN}[5/6] 启动 PCT 全局规划器...${NC}"
gnome-terminal --tab --title="PCT Global Planner" -- bash -c "
    cd $WORKSPACE_DIR
    source install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE_DIR/fastdds_no_shm.xml
    python3 src/global_planning/PCT_planner/planner/scripts/global_planner.py
    exec bash
" &
sleep 2

# 启动 RViz2 可视化
echo -e "${GREEN}[6/6] 启动 RViz2 可视化...${NC}"
gnome-terminal --tab --title="RViz2 - Planning" -- bash -c "
    cd $WORKSPACE_DIR
    source install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=$WORKSPACE_DIR/fastdds_no_shm.xml
    rviz2 -d src/global_planning/PCT_planner/rsc/rviz/pct_ros.rviz
    exec bash
" &

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}✓ 3D 全局规划系统已启动！${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}使用说明:${NC}"
echo -e "  1. 在 RViz2 中查看地图和机器人位置"
if [[ "$LOC_CHOICE" == "1" ]]; then
    echo -e "  2. ${GREEN}使用 '2D Pose Estimate' 工具设置机器人起始位置${NC}"
    echo -e "  3. ${GREEN}使用 'Publish Point' 工具点击地图设置目标点${NC}"
else
    echo -e "  2. ${GREEN}使用 'Publish Point' 工具点击地图设置目标点${NC}"
fi
echo -e "  4. 规划器会自动计算并发布路径"
echo -e "  5. 按 Ctrl+C 可在各终端窗口中停止对应节点"
echo ""
echo -e "${YELLOW}话题查看:${NC}"
echo -e "  - 地图点云: ${BLUE}ros2 topic echo /pct_planner/tomogram${NC}"
echo -e "  - 规划路径: ${BLUE}ros2 topic echo /pct_planner/path${NC}"
echo -e "  - 机器人位姿: ${BLUE}ros2 topic echo /Odometry${NC}"
echo ""
echo -e "${YELLOW}调试工具:${NC}"
echo -e "  - 检查地图: ${BLUE}python3 src/global_planning/PCT_planner/planner/scripts/check_map.py${NC}"
echo ""
