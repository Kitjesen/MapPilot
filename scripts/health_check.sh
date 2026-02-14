#!/bin/bash
# MapPilot 系统健康检查
# 用途: 快速诊断系统状态，检测潜在问题
# 使用: bash scripts/health_check.sh

set -e

echo "=========================================="
echo "  MapPilot 3D NAV 健康检查"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

check_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

check_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

FAIL_COUNT=0
WARN_COUNT=0

# 1. 检查 ROS 2 环境
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "1. ROS 2 环境检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    check_pass "ROS 2 Humble 已安装"
    ROS_VERSION=$(ros2 --version 2>/dev/null | head -1)
    check_info "版本: $ROS_VERSION"
else
    check_fail "ROS 2 Humble 未找到"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# 2. 检查编译产物
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "2. 编译产物检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -d "install/" ]; then
    check_pass "install/ 目录存在"
    PKG_COUNT=$(find install/ -name "package.xml" 2>/dev/null | wc -l)
    check_info "已安装包数量: $PKG_COUNT"

    # 检查关键包
    CRITICAL_PACKAGES=("remote_monitoring" "base_autonomy" "slam")
    for pkg in "${CRITICAL_PACKAGES[@]}"; do
        if [ -d "install/$pkg" ]; then
            check_pass "  关键包: $pkg"
        else
            check_warn "  关键包缺失: $pkg"
            WARN_COUNT=$((WARN_COUNT + 1))
        fi
    done
else
    check_warn "install/ 目录不存在，请先运行 make build"
    WARN_COUNT=$((WARN_COUNT + 1))
fi

# 3. 检查版本一致性
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "3. 版本一致性检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -f "VERSION" ]; then
    VERSION=$(cat VERSION)
    check_pass "VERSION 文件存在: $VERSION"

    # 检查 CHANGELOG 是否包含当前版本
    if grep -q "$VERSION" docs/CHANGELOG.md 2>/dev/null; then
        check_pass "CHANGELOG.md 包含当前版本"
    else
        check_warn "CHANGELOG.md 未更新到当前版本"
        WARN_COUNT=$((WARN_COUNT + 1))
    fi
else
    check_fail "VERSION 文件不存在"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# 4. 检查系统服务（如果在机器人上运行）
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "4. 系统服务检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
SERVICES=("navigation.service" "mapping.service")
for service in "${SERVICES[@]}"; do
    if systemctl is-active --quiet "$service" 2>/dev/null; then
        check_pass "$service 运行中"
        UPTIME=$(systemctl show "$service" -p ActiveEnterTimestamp --value 2>/dev/null)
        check_info "  启动时间: $UPTIME"
    elif systemctl list-unit-files | grep -q "$service" 2>/dev/null; then
        check_warn "$service 已安装但未运行"
        WARN_COUNT=$((WARN_COUNT + 1))
    else
        check_info "$service 未安装（开发环境正常）"
    fi
done

# 5. 检查 ROS 2 节点（如果系统正在运行）
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "5. ROS 2 节点检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if command -v ros2 &> /dev/null && [ -d "install/" ]; then
    source install/setup.bash 2>/dev/null || true
    NODE_COUNT=$(timeout 3 ros2 node list 2>/dev/null | wc -l || echo "0")

    if [ "$NODE_COUNT" -gt 0 ]; then
        check_pass "检测到 $NODE_COUNT 个活跃节点"

        # 检查关键节点
        CRITICAL_NODES=("/grpc_gateway" "/mode_manager" "/health_monitor")
        for node in "${CRITICAL_NODES[@]}"; do
            if timeout 2 ros2 node list 2>/dev/null | grep -q "$node"; then
                check_pass "  关键节点: $node"
            else
                check_warn "  关键节点缺失: $node"
                WARN_COUNT=$((WARN_COUNT + 1))
            fi
        done
    else
        check_info "未检测到活跃节点（系统未启动）"
    fi
else
    check_info "跳过节点检查（ros2 不可用或未编译）"
fi

# 6. 检查关键话题频率（如果系统正在运行）
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "6. 关键话题频率检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ "$NODE_COUNT" -gt 0 ]; then
    TOPICS=("/nav/odometry" "/nav/terrain_map" "/nav/path" "/cmd_vel")
    for topic in "${TOPICS[@]}"; do
        if timeout 2 ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
            # 尝试获取频率（最多等待 3 秒）
            HZ=$(timeout 3 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
            if [ -n "$HZ" ] && (( $(echo "$HZ > 0.5" | bc -l 2>/dev/null || echo 0) )); then
                check_pass "$topic: ${HZ} Hz"
            else
                check_warn "$topic: 频率过低或无数据"
                WARN_COUNT=$((WARN_COUNT + 1))
            fi
        else
            check_info "$topic: 话题不存在（系统未完全启动）"
        fi
    done
else
    check_info "跳过话题检查（系统未运行）"
fi

# 7. 检查 TF 树完整性
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "7. TF 树检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if command -v ros2 &> /dev/null && [ "$NODE_COUNT" -gt 0 ]; then
    # 检查关键 TF 变换
    TF_PAIRS=("map odom" "odom body" "body lidar")
    for pair in "${TF_PAIRS[@]}"; do
        read -r parent child <<< "$pair"
        if timeout 2 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 | grep -q "At time" 2>/dev/null; then
            check_pass "TF: $parent → $child 可达"
        else
            check_warn "TF: $parent → $child 不可达"
            WARN_COUNT=$((WARN_COUNT + 1))
        fi
    done
else
    check_info "跳过 TF 检查（系统未运行）"
fi

# 8. 检查磁盘空间
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "8. 磁盘空间检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
DISK_USAGE=$(df -h . | tail -1 | awk '{print $5}' | sed 's/%//')
DISK_AVAIL=$(df -h . | tail -1 | awk '{print $4}')
if [ "$DISK_USAGE" -lt 80 ]; then
    check_pass "磁盘使用率: ${DISK_USAGE}% (可用: $DISK_AVAIL)"
elif [ "$DISK_USAGE" -lt 90 ]; then
    check_warn "磁盘使用率: ${DISK_USAGE}% (可用: $DISK_AVAIL) - 建议清理"
    WARN_COUNT=$((WARN_COUNT + 1))
else
    check_fail "磁盘使用率: ${DISK_USAGE}% (可用: $DISK_AVAIL) - 空间不足！"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# 9. 检查日志文件
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "9. 日志文件检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
LOG_DIRS=("/var/log/nav-system" "$HOME/.ros/log")
for log_dir in "${LOG_DIRS[@]}"; do
    if [ -d "$log_dir" ]; then
        LOG_SIZE=$(du -sh "$log_dir" 2>/dev/null | awk '{print $1}')
        LOG_COUNT=$(find "$log_dir" -type f 2>/dev/null | wc -l)
        check_pass "$log_dir: $LOG_SIZE ($LOG_COUNT 文件)"

        # 检查最近的错误日志
        RECENT_ERRORS=$(find "$log_dir" -type f -mtime -1 -exec grep -l "ERROR\|FATAL" {} \; 2>/dev/null | wc -l)
        if [ "$RECENT_ERRORS" -gt 0 ]; then
            check_warn "  最近 24 小时内有 $RECENT_ERRORS 个日志文件包含错误"
            WARN_COUNT=$((WARN_COUNT + 1))
        fi
    else
        check_info "$log_dir: 不存在"
    fi
done

# 10. 检查 gRPC Gateway
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "10. gRPC Gateway 检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
GRPC_PORT=50051
if command -v nc &> /dev/null; then
    if nc -z localhost $GRPC_PORT 2>/dev/null; then
        check_pass "gRPC Gateway 端口 $GRPC_PORT 可达"

        # 尝试简单的健康检查（如果有 grpcurl）
        if command -v grpcurl &> /dev/null; then
            if timeout 2 grpcurl -plaintext localhost:$GRPC_PORT list 2>/dev/null | grep -q "robot.v1"; then
                check_pass "  gRPC 服务响应正常"
            else
                check_warn "  gRPC 服务无响应"
                WARN_COUNT=$((WARN_COUNT + 1))
            fi
        fi
    else
        check_info "gRPC Gateway 未运行（开发环境正常）"
    fi
else
    check_info "nc 命令不可用，跳过端口检查"
fi

# 11. 检查配置文件
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "11. 配置文件检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
CONFIG_FILES=(
    "config/robot_config.yaml"
    "config/topic_contract.yaml"
    "src/remote_monitoring/config/grpc_gateway.yaml"
)
for config in "${CONFIG_FILES[@]}"; do
    if [ -f "$config" ]; then
        check_pass "$config 存在"

        # 检查 YAML 语法（如果有 python）
        if command -v python3 &> /dev/null; then
            if python3 -c "import yaml; yaml.safe_load(open('$config'))" 2>/dev/null; then
                check_pass "  YAML 语法正确"
            else
                check_fail "  YAML 语法错误"
                FAIL_COUNT=$((FAIL_COUNT + 1))
            fi
        fi
    else
        check_warn "$config 不存在"
        WARN_COUNT=$((WARN_COUNT + 1))
    fi
done

# 12. 检查 Git 状态
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "12. Git 状态检查"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -d ".git" ]; then
    check_pass "Git 仓库已初始化"

    # 检查未提交的更改
    UNCOMMITTED=$(git status --porcelain 2>/dev/null | wc -l)
    if [ "$UNCOMMITTED" -eq 0 ]; then
        check_pass "工作区干净"
    else
        check_info "有 $UNCOMMITTED 个未提交的更改"
    fi

    # 检查当前分支
    BRANCH=$(git branch --show-current 2>/dev/null)
    check_info "当前分支: $BRANCH"
else
    check_warn "不是 Git 仓库"
    WARN_COUNT=$((WARN_COUNT + 1))
fi

# 13. 总结
echo ""
echo "=========================================="
echo "  健康检查完成"
echo "=========================================="
echo ""

if [ $FAIL_COUNT -eq 0 ] && [ $WARN_COUNT -eq 0 ]; then
    echo -e "${GREEN}✅ 系统状态良好，所有检查通过${NC}"
    EXIT_CODE=0
elif [ $FAIL_COUNT -eq 0 ]; then
    echo -e "${YELLOW}⚠️  系统基本正常，但有 $WARN_COUNT 个警告${NC}"
    EXIT_CODE=0
else
    echo -e "${RED}❌ 发现 $FAIL_COUNT 个严重问题和 $WARN_COUNT 个警告${NC}"
    EXIT_CODE=1
fi

echo ""
echo "提示:"
echo "  - 如需启动系统: make mapping 或 make navigation"
echo "  - 如需编译系统: make build"
echo "  - 如需运行测试: make test"
echo ""

exit $EXIT_CODE
