#!/bin/bash
# ========================================
# 保存 SLAM 地图脚本 (ROS 2 Humble)
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
echo -e "${BLUE}   保存 SLAM 地图${NC}"
echo -e "${BLUE}========================================${NC}"

# 检查是否已经 source 环境
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo -e "${RED}错误: 找不到 install/setup.bash${NC}"
    echo -e "${YELLOW}请先编译工作空间: colcon build${NC}"
    exit 1
fi

# 设置 FastDDS 配置
export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE_DIR/config/fastdds_no_shm.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 地图保存目录
MAP_DIR="$WORKSPACE_DIR/maps"
PCD_DIR="$WORKSPACE_DIR/src/global_planning/PCT_planner/rsc/pcd"
TOMOGRAM_DIR="$WORKSPACE_DIR/src/global_planning/PCT_planner/rsc/tomogram"
SEMANTIC_DIR="$WORKSPACE_DIR/maps/semantic"

# 创建目录
mkdir -p "$MAP_DIR"
mkdir -p "$PCD_DIR"
mkdir -p "$TOMOGRAM_DIR"
mkdir -p "$SEMANTIC_DIR"

# 获取当前时间戳
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAP_NAME="map_${TIMESTAMP}"

echo -e "${GREEN}地图将保存为: ${MAP_NAME}${NC}"
echo ""

# 选择保存类型
echo -e "${YELLOW}请选择保存类型:${NC}"
echo -e "  ${GREEN}1)${NC} 保存 PGO 地图点云 (.pcd)"
echo -e "  ${GREEN}2)${NC} 保存 PGO 地图 + 生成 PCT 地图 (推荐)"
echo -e "  ${GREEN}3)${NC} 仅生成 PCT Tomogram (需要已有 .pcd 文件)"
read -p "输入选项 [1-3]: " SAVE_CHOICE

case $SAVE_CHOICE in
    1)
        echo ""
        echo -e "${GREEN}[1/1] 保存 PGO 地图点云...${NC}"
        echo -e "${YELLOW}正在调用 PGO 保存服务...${NC}"
        
        # 调用 PGO 保存服务 (interface/srv/SaveMaps, 服务名: /pgo/save_maps)
        if ros2 service list 2>/dev/null | grep -q "/pgo/save_maps"; then
            echo -e "${YELLOW}调用 /pgo/save_maps 服务...${NC}"
            ros2 service call /pgo/save_maps interface/srv/SaveMaps \
                "{file_path: '$MAP_DIR/$MAP_NAME', save_patches: false}"
        else
            echo -e "${YELLOW}警告: /pgo/save_maps 服务不可用，尝试 FastLIO2 /save_map 服务...${NC}"
            if ros2 service list 2>/dev/null | grep -q "^/save_map$"; then
                ros2 service call /save_map interface/srv/SaveMaps \
                    "{file_path: '$MAP_DIR/$MAP_NAME', save_patches: false}"
            else
                echo -e "${RED}错误: 未找到地图保存服务，请确认 SLAM 节点正在运行${NC}"
                echo -e "${YELLOW}可用服务: $(ros2 service list 2>/dev/null | head -20)${NC}"
                exit 1
            fi
        fi
        
        echo -e "${GREEN}✓ PGO 地图已保存到: $MAP_DIR/${MAP_NAME}.pcd${NC}"
        ;;
        
    2)
        echo ""
        echo -e "${GREEN}[1/3] 保存 PGO 地图点云...${NC}"
        
        # 保存 PGO 地图 (interface/srv/SaveMaps, 服务名: /pgo/save_maps)
        if ros2 service list 2>/dev/null | grep -q "/pgo/save_maps"; then
            echo -e "${YELLOW}调用 /pgo/save_maps 服务...${NC}"
            ros2 service call /pgo/save_maps interface/srv/SaveMaps \
                "{file_path: '$MAP_DIR/$MAP_NAME', save_patches: false}"
        else
            echo -e "${YELLOW}警告: PGO 服务不可用，尝试 FastLIO2 /save_map ...${NC}"
            ros2 service call /save_map interface/srv/SaveMaps \
                "{file_path: '$MAP_DIR/$MAP_NAME', save_patches: false}"
        fi
        
        # 复制到 PCT PCD 目录
        echo -e "${GREEN}[2/3] 复制 PCD 到 PCT 工作目录...${NC}"
        if [ -f "$MAP_DIR/${MAP_NAME}.pcd" ]; then
            cp "$MAP_DIR/${MAP_NAME}.pcd" "$PCD_DIR/${MAP_NAME}.pcd"
            ln -sf "$PCD_DIR/${MAP_NAME}.pcd" "$PCD_DIR/latest.pcd"
            echo -e "${GREEN}✓ PCD 文件已复制${NC}"
        else
            echo -e "${RED}错误: 未找到生成的 PCD 文件${NC}"
            exit 1
        fi
        
        # 生成 PCT Tomogram
        echo -e "${GREEN}[3/3] 生成 PCT Tomogram (3D 地图)...${NC}"
        echo -e "${YELLOW}这可能需要几分钟，请耐心等待...${NC}"
        
        cd "$WORKSPACE_DIR/src/global_planning/PCT_planner/tomography/scripts"
        python3 tomography.py --scene "$MAP_NAME"
        
        if [ $? -eq 0 ]; then
            echo ""
            echo -e "${GREEN}✓ PCT Tomogram 已生成！${NC}"
            echo -e "  - PCD: $PCD_DIR/${MAP_NAME}.pcd"
            echo -e "  - Tomogram: $TOMOGRAM_DIR/${MAP_NAME}.pickle"
        else
            echo -e "${RED}错误: Tomogram 生成失败${NC}"
            exit 1
        fi
        ;;
        
    3)
        echo ""
        echo -e "${YELLOW}可用的 PCD 文件:${NC}"
        ls -lh "$PCD_DIR"/*.pcd 2>/dev/null || echo -e "${RED}未找到 PCD 文件${NC}"
        echo ""
        
        read -p "输入 PCD 文件名 (不含路径和扩展名): " PCD_NAME
        
        if [ -f "$PCD_DIR/${PCD_NAME}.pcd" ]; then
            echo -e "${GREEN}生成 PCT Tomogram...${NC}"
            cd "$WORKSPACE_DIR/src/global_planning/PCT_planner/tomography/scripts"
            python3 tomography.py --scene "$PCD_NAME"
            
            if [ $? -eq 0 ]; then
                echo -e "${GREEN}✓ PCT Tomogram 已生成: $TOMOGRAM_DIR/${PCD_NAME}.pickle${NC}"
            else
                echo -e "${RED}错误: Tomogram 生成失败${NC}"
                exit 1
            fi
        else
            echo -e "${RED}错误: 未找到文件 $PCD_DIR/${PCD_NAME}.pcd${NC}"
            exit 1
        fi
        ;;
        
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac

# ── 保存语义数据 (房间-物体 KG, 拓扑记忆, 场景图) ──
echo ""
echo -e "${GREEN}保存语义地图数据...${NC}"
SEMANTIC_SAVE_DIR="$SEMANTIC_DIR/${MAP_NAME}"
mkdir -p "$SEMANTIC_SAVE_DIR"

# 复制语义运行时数据 (如果存在)
if [ -f "$SEMANTIC_DIR/room_object_kg.json" ]; then
    cp "$SEMANTIC_DIR/room_object_kg.json" "$SEMANTIC_SAVE_DIR/room_object_kg.json"
    echo -e "${GREEN}✓ 房间-物体知识图谱已保存${NC}"
fi
if [ -f "$SEMANTIC_DIR/topo_memory.json" ]; then
    cp "$SEMANTIC_DIR/topo_memory.json" "$SEMANTIC_SAVE_DIR/topo_memory.json"
    echo -e "${GREEN}✓ 拓扑记忆已保存${NC}"
fi

# 尝试通过 ROS2 服务通知 semantic_planner 保存数据
if ros2 topic list 2>/dev/null | grep -q "/nav/semantic/scene_graph"; then
    echo -e "${YELLOW}通知语义规划器保存数据...${NC}"
    # 订阅并保存最新场景图
    timeout 5s ros2 topic echo /nav/semantic/scene_graph --once > "$SEMANTIC_SAVE_DIR/scene_graph_snapshot.json" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ 场景图快照已保存${NC}"
    fi
fi

# 创建符号链接指向最新语义数据
ln -sf "$SEMANTIC_SAVE_DIR" "$SEMANTIC_DIR/latest"

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}✓ 地图保存完成！${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}保存的文件:${NC}"
echo -e "  - 原始地图: ${BLUE}$MAP_DIR/${MAP_NAME}.pcd${NC}"
if [[ "$SAVE_CHOICE" == "2" ]] || [[ "$SAVE_CHOICE" == "3" ]]; then
    echo -e "  - PCT 地图: ${BLUE}$TOMOGRAM_DIR/${MAP_NAME}.pickle${NC}"
fi
echo -e "  - 语义数据: ${BLUE}$SEMANTIC_SAVE_DIR/${NC}"
echo ""
echo -e "${YELLOW}下一步:${NC}"
echo -e "  1. 设置地图路径环境变量:"
echo -e "     ${GREEN}export NAV_MAP_PATH=\"$TOMOGRAM_DIR/${MAP_NAME}\"${NC}"
echo -e "  2. 启动完整导航栈 (推荐):"
echo -e "     ${GREEN}make navigation${NC}  或  ${GREEN}ros2 launch launch/navigation_run.launch.py${NC}"
echo -e "  3. 单独测试 PCT 规划器:"
echo -e "     ${GREEN}./planning.sh${NC}  (会自动检测最新地图)"
echo ""
echo -e "${YELLOW}可视化 Tomogram:${NC}"
echo -e "  ${BLUE}python3 src/global_planning/PCT_planner/tomography/scripts/visualize_tomogram.py --scene ${MAP_NAME}${NC}"
echo ""
