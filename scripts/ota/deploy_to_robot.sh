#!/bin/bash
# ============================================================================
# deploy_to_robot.sh — 一键部署导航系统到机器人
#
# 适用场景:
#   - 快速部署 install/ + config/ + systemd 到目标机器人
#   - 自动重启服务并验证
#
# 用法:
#   ./scripts/ota/deploy_to_robot.sh sunrise@192.168.123.10
#
# 前置条件:
#   - SSH 免密登录已配置 (ssh-copy-id)
#   - install/ 已编译 (colcon build)
# ============================================================================
set -euo pipefail

# ---- 颜色 ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ---- 工作空间 ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

REMOTE_NAV_DIR="/home/sunrise/data/SLAM/navigation"
SERVICES="nav-slam nav-autonomy nav-planning nav-grpc ota-daemon"

# ---- 参数检查 ----
if [ $# -lt 1 ] || [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    echo "用法: $0 <user@host>"
    echo ""
    echo "示例:"
    echo "  $0 sunrise@192.168.123.10"
    echo ""
    echo "步骤:"
    echo "  1. rsync install/ 到机器人"
    echo "  2. rsync config/ 到机器人"
    echo "  3. 同步 systemd 服务文件"
    echo "  4. daemon-reload"
    echo "  5. 重启导航服务"
    echo "  6. 验证 nav-grpc 是否启动"
    exit 0
fi

TARGET="$1"

# ---- 前置验证 ----
if [ ! -d "$WORKSPACE_DIR/install" ]; then
    echo -e "${RED}ERROR: install/ 不存在，请先运行 colcon build${NC}"
    exit 1
fi

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  一键部署导航系统${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "  目标: ${TARGET}"
echo -e "  路径: ${REMOTE_NAV_DIR}"
echo ""

# ---- Step 1: 同步 install/ ----
echo -e "${GREEN}[1/6] 同步 install/ ...${NC}"
rsync -avz --exclude='.git' \
    "$WORKSPACE_DIR/install/" \
    "${TARGET}:${REMOTE_NAV_DIR}/install/"
echo -e "  ${GREEN}完成${NC}"

# ---- Step 2: 同步 config/ ----
echo -e "${GREEN}[2/6] 同步 config/ ...${NC}"
rsync -avz \
    "$WORKSPACE_DIR/config/" \
    "${TARGET}:${REMOTE_NAV_DIR}/config/"
echo -e "  ${GREEN}完成${NC}"

# ---- Step 3: 同步 systemd 服务文件 ----
echo -e "${GREEN}[3/6] 同步 systemd 服务文件 ...${NC}"
rsync -avz \
    "$WORKSPACE_DIR/systemd/" \
    "${TARGET}:~/systemd-staging/"
echo -e "  ${GREEN}完成${NC}"

# ---- Step 4: 安装服务文件并 daemon-reload ----
echo -e "${GREEN}[4/6] 安装服务文件并 daemon-reload ...${NC}"
ssh "$TARGET" "sudo cp ~/systemd-staging/*.service /etc/systemd/system/ && sudo systemctl daemon-reload"
echo -e "  ${GREEN}完成${NC}"

# ---- Step 5: 重启导航服务 ----
echo -e "${GREEN}[5/6] 重启导航服务 ...${NC}"
ssh "$TARGET" "sudo systemctl restart ${SERVICES}"
echo -e "  ${GREEN}完成${NC}"

# ---- Step 6: 验证服务状态 ----
echo -e "${GREEN}[6/6] 验证服务状态 (等待 5 秒) ...${NC}"
VERIFY_RESULT=$(ssh "$TARGET" "bash -c 'sleep 5 && systemctl is-active nav-grpc && echo OK || echo FAIL'")

echo ""
echo -e "${BLUE}============================================${NC}"
if echo "$VERIFY_RESULT" | grep -q "OK"; then
    echo -e "${GREEN}  部署成功! nav-grpc 运行正常${NC}"
else
    echo -e "${RED}  部署异常! nav-grpc 未正常启动${NC}"
    echo -e "${YELLOW}  请检查: ssh ${TARGET} 'journalctl -u nav-grpc -n 50'${NC}"
fi
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "${YELLOW}验证命令:${NC}"
echo -e "  ssh ${TARGET} 'systemctl status ${SERVICES}'"
echo -e "  ssh ${TARGET} 'journalctl -u nav-grpc -f'"
