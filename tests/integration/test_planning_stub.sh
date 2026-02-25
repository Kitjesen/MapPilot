#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# test_planning_stub.sh — stub 模式规划流水线集成测试
#
# 无需 LiDAR / IMU 硬件：使用 slam_profile:=stub + planner_profile:=stub
# 提供假里程计 (TF) 后跑 test_planning_pipeline.py
#
# 用法:
#   cd <workspace_root>
#   bash tests/integration/test_planning_stub.sh
# ─────────────────────────────────────────────────────────────────────────────

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WORKSPACE_DIR"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

LAUNCH_PID=""
PCT_PID=""
TF_PID=""

cleanup() {
    echo ""
    echo -e "${YELLOW}清理进程...${NC}"
    # 精确 kill 已记录的 PID (P0 fix: PCT_PID/TF_PID 原先未 kill)
    [ -n "$PCT_PID"    ] && kill "$PCT_PID"    2>/dev/null || true
    [ -n "$TF_PID"     ] && kill "$TF_PID"     2>/dev/null || true
    if [ -n "$LAUNCH_PID" ]; then
        kill "$LAUNCH_PID" 2>/dev/null || true
        sleep 1
        pkill -P "$LAUNCH_PID" 2>/dev/null || true
    fi
    # 兜底: 清理可能残留的 ROS2 节点
    pkill -f "terrainAnalysis"          2>/dev/null || true
    pkill -f "localPlanner"             2>/dev/null || true
    pkill -f "pathFollower"             2>/dev/null || true
    pkill -f "pct_path_adapter"         2>/dev/null || true
    pkill -f "static_transform_publisher" 2>/dev/null || true
}
trap cleanup EXIT

echo "=========================================="
echo "  规划流水线 Stub 模式集成测试"
echo "=========================================="
echo ""

# ── 检查环境 ──────────────────────────────────────────────────────────────────
# 注意: 不使用 set -e，避免后台节点启动/sleep 被 SIGINT 中断时意外退出
# 关键错误用显式 exit 1 处理
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${RED}❌ ROS2 Humble 未安装${NC}"
    exit 1
fi
source /opt/ros/humble/setup.bash || { echo -e "${RED}❌ source ROS2 失败${NC}"; exit 1; }

if [ ! -f "install/setup.bash" ]; then
    echo -e "${RED}❌ install/ 未找到，请先 make build${NC}"
    exit 1
fi
source install/setup.bash || { echo -e "${RED}❌ source workspace 失败${NC}"; exit 1; }

# ── 启动自主导航子系统（stub SLAM + stub 规划器）────────────────────────────
echo -e "${BLUE}[1/4] 启动 autonomy 子系统 (stub 模式)...${NC}"
# autonomy.launch.py 只启动 terrainAnalysis / terrainAnalysisExt / localPlanner / pathFollower
# 不依赖 LiDAR, SLAM, 硬件
ros2 launch launch/subsystems/autonomy.launch.py \
    maxSpeed:=0.5 \
    autonomyMode:=true \
    autonomySpeed:=0.3 \
    > /tmp/autonomy_stub.log 2>&1 &
LAUNCH_PID=$!
echo "   autonomy PID: $LAUNCH_PID"

# ── 启动 PCT Adapter（需要单独启动，因为 planner_pct profile 包含全局规划器）──
echo -e "${BLUE}[2/4] 启动 pct_path_adapter（单节点）...${NC}"
ros2 run pct_adapters pct_path_adapter \
    --ros-args \
    -r /pct_path:=/nav/global_path \
    -r /planner_waypoint:=/nav/way_point \
    -r /Odometry:=/nav/odometry \
    -p waypoint_distance:=0.5 \
    -p arrival_threshold:=0.5 \
    -p stuck_timeout_sec:=30.0 \
    > /tmp/pct_adapter_stub.log 2>&1 &
PCT_PID=$!
echo "   pct_path_adapter PID: $PCT_PID"

# ── 发布静态 TF: map → odom (identity) ───────────────────────────────────────
echo -e "${BLUE}[3/4] 发布静态 TF map → odom...${NC}"
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 map odom > /tmp/tf_stub.log 2>&1 &
TF_PID=$!

# ── 等待节点启动 ──────────────────────────────────────────────────────────────
echo -e "${BLUE}[4/4] 等待节点启动 (8s)...${NC}"
for i in $(seq 1 8); do
    echo -n "."
    sleep 1
done
echo ""

# ── 快速节点存活检查 (P2 fix: 预检失败时快速退出) ────────────────────────────
echo ""
echo -e "${YELLOW}── 节点存活预检 ──${NC}"
PRECHECK_FAIL=0
for node in terrainAnalysis terrainAnalysisExt localPlanner pathFollower pct_path_adapter; do
    if ros2 node list 2>/dev/null | grep -q "/$node"; then
        echo -e "  ${GREEN}✓${NC} $node"
    else
        echo -e "  ${RED}✗${NC} $node"
        PRECHECK_FAIL=1
    fi
done
if [ "$PRECHECK_FAIL" -eq 1 ]; then
    echo -e "${RED}❌ 节点未完全启动，中止测试${NC}"
    echo "  查看启动日志: cat /tmp/autonomy_stub.log  cat /tmp/pct_adapter_stub.log"
    exit 1
fi

# ── 运行 Python 集成测试 ──────────────────────────────────────────────────────
echo ""
echo -e "${BLUE}运行 test_planning_pipeline.py ...${NC}"
echo ""

if python3 tests/integration/test_planning_pipeline.py; then
    echo ""
    echo -e "${GREEN}✅ 规划流水线 Stub 集成测试通过${NC}"
    EXIT_CODE=0
else
    echo ""
    echo -e "${RED}❌ 规划流水线 Stub 集成测试失败${NC}"
    echo "  查看日志:"
    echo "    cat /tmp/autonomy_stub.log"
    echo "    cat /tmp/pct_adapter_stub.log"
    EXIT_CODE=1
fi

# cleanup 通过 trap 自动执行
exit $EXIT_CODE
