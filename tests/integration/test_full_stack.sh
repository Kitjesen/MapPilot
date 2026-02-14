#!/bin/bash
# 全系统启动测试
# 验证所有关键节点能否正常启动

set -e

echo "=========================================="
echo "  全系统启动测试"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 检查 ROS 2 环境
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${RED}❌ ROS 2 Humble 未安装${NC}"
    exit 1
fi

source /opt/ros/humble/setup.bash

# 检查编译产物
if [ ! -d "install/" ]; then
    echo -e "${RED}❌ install/ 目录不存在，请先运行 make build${NC}"
    exit 1
fi

source install/setup.bash

# 1. 启动系统（后台）
echo "1. 启动导航系统..."
ros2 launch launch/navigation_run.launch.py > /tmp/nav_launch.log 2>&1 &
LAUNCH_PID=$!

echo "   进程 PID: $LAUNCH_PID"

# 等待系统启动
echo "2. 等待系统启动 (30s)..."
for i in {1..30}; do
    echo -n "."
    sleep 1
done
echo ""

# 3. 检查关键节点
echo "3. 检查关键节点..."
EXPECTED_NODES=(
    "/grpc_gateway"
    "/mode_manager"
    "/health_monitor"
    "/task_manager"
    "/path_follower"
    "/local_planner"
)

MISSING_NODES=()
for node in "${EXPECTED_NODES[@]}"; do
    if timeout 3 ros2 node list 2>/dev/null | grep -q "$node"; then
        echo -e "  ${GREEN}✓${NC} $node"
    else
        echo -e "  ${RED}✗${NC} $node (缺失)"
        MISSING_NODES+=("$node")
    fi
done

# 4. 检查关键话题
echo ""
echo "4. 检查关键话题..."
EXPECTED_TOPICS=(
    "/nav/odometry"
    "/nav/terrain_map"
    "/nav/path"
    "/cmd_vel"
)

MISSING_TOPICS=()
for topic in "${EXPECTED_TOPICS[@]}"; do
    if timeout 3 ros2 topic list 2>/dev/null | grep -q "$topic"; then
        echo -e "  ${GREEN}✓${NC} $topic"
    else
        echo -e "  ${RED}✗${NC} $topic (缺失)"
        MISSING_TOPICS+=("$topic")
    fi
done

# 5. 检查 TF 树
echo ""
echo "5. 检查 TF 树..."
if timeout 3 ros2 run tf2_ros tf2_echo map body 2>&1 | grep -q "At time"; then
    echo -e "  ${GREEN}✓${NC} TF 树正常 (map → body 可达)"
else
    echo -e "  ${YELLOW}⚠${NC} TF 树不完整"
fi

# 6. 检查 gRPC Gateway
echo ""
echo "6. 检查 gRPC Gateway..."
if nc -z localhost 50051 2>/dev/null; then
    echo -e "  ${GREEN}✓${NC} gRPC Gateway 端口 50051 可达"
else
    echo -e "  ${RED}✗${NC} gRPC Gateway 未运行"
fi

# 7. 清理
echo ""
echo "7. 清理..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 2
pkill -P $LAUNCH_PID 2>/dev/null || true
wait $LAUNCH_PID 2>/dev/null || true

# 8. 结果
echo ""
echo "=========================================="
if [ ${#MISSING_NODES[@]} -eq 0 ] && [ ${#MISSING_TOPICS[@]} -eq 0 ]; then
    echo -e "${GREEN}✅ 全系统启动测试通过${NC}"
    echo "=========================================="
    exit 0
else
    echo -e "${RED}❌ 全系统启动测试失败${NC}"
    echo "=========================================="
    echo ""
    if [ ${#MISSING_NODES[@]} -gt 0 ]; then
        echo "缺失节点:"
        for node in "${MISSING_NODES[@]}"; do
            echo "  - $node"
        done
    fi
    if [ ${#MISSING_TOPICS[@]} -gt 0 ]; then
        echo "缺失话题:"
        for topic in "${MISSING_TOPICS[@]}"; do
            echo "  - $topic"
        done
    fi
    echo ""
    echo "查看日志: cat /tmp/nav_launch.log"
    exit 1
fi
