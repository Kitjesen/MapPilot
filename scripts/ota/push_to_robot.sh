#!/bin/bash
# ============================================================================
# push_to_robot.sh — 直接推送导航功能包到机器人 (不经过 GitHub)
#
# 适用场景:
#   - 开发/调试阶段快速迭代
#   - 内网环境无外网访问
#   - 紧急修复快速部署
#
# 用法:
#   ./scripts/ota/push_to_robot.sh sunrise@192.168.1.100
#   ./scripts/ota/push_to_robot.sh sunrise@192.168.1.100 --build    # 先编译再推送
#   ./scripts/ota/push_to_robot.sh sunrise@192.168.1.100 --restart   # 推送后重启服务
#   ./scripts/ota/push_to_robot.sh sunrise@192.168.1.100 --packages-select fastlio2 local_planner
#
# 前置条件:
#   - SSH 免密登录已配置 (ssh-copy-id)
#   - 机器人端已有 /opt/robot/navigation/ 目录结构
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
cd "$WORKSPACE_DIR"

VERSION=$(cat VERSION 2>/dev/null | tr -d '[:space:]' || echo "dev")

# ---- 参数解析 ----
ROBOT_TARGET=""
DO_BUILD=false
DO_RESTART=false
PACKAGES_SELECT=""
SYNC_ONLY=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --build|-b)
            DO_BUILD=true
            shift
            ;;
        --restart|-r)
            DO_RESTART=true
            shift
            ;;
        --packages-select)
            shift
            PACKAGES_SELECT="$*"
            break
            ;;
        --sync-only)
            SYNC_ONLY=true
            shift
            ;;
        --help|-h)
            echo "用法: $0 <user@robot_ip> [选项]"
            echo ""
            echo "选项:"
            echo "  -b, --build               推送前先 colcon build"
            echo "  -r, --restart             推送后重启导航服务"
            echo "  --packages-select PKG...  仅推送指定包 (增量更新)"
            echo "  --sync-only               仅 rsync install/ (不打包不安装)"
            echo "  -h, --help                显示帮助"
            echo ""
            echo "示例:"
            echo "  $0 sunrise@192.168.1.100 --build --restart"
            echo "  $0 sunrise@192.168.1.100 --packages-select local_planner --restart"
            echo "  $0 sunrise@192.168.1.100 --sync-only"
            exit 0
            ;;
        -*)
            echo -e "${RED}未知选项: $1${NC}"
            exit 1
            ;;
        *)
            if [ -z "$ROBOT_TARGET" ]; then
                ROBOT_TARGET="$1"
            fi
            shift
            ;;
    esac
done

if [ -z "$ROBOT_TARGET" ]; then
    echo -e "${RED}ERROR: 请指定机器人地址，如: sunrise@192.168.1.100${NC}"
    echo -e "用法: $0 <user@robot_ip> [选项]"
    exit 1
fi

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  导航包推送 v${VERSION}${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "  目标: ${ROBOT_TARGET}"
echo ""

# ---- Step 1: 编译 (可选) ----
if [ "$DO_BUILD" = true ]; then
    echo -e "${GREEN}[1/4] 编译中...${NC}"
    if [ -n "$PACKAGES_SELECT" ]; then
        colcon build --packages-select $PACKAGES_SELECT --cmake-args -DCMAKE_BUILD_TYPE=Release
    else
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    fi
else
    echo -e "${YELLOW}[1/4] 跳过编译 (使用现有 install/)${NC}"
fi

# 验证 install 目录
if [ ! -d "$WORKSPACE_DIR/install" ]; then
    echo -e "${RED}ERROR: install/ 不存在, 请先 colcon build 或加 --build${NC}"
    exit 1
fi

# ---- Step 2: 连接检查 ----
echo -e "${GREEN}[2/4] 检查连接...${NC}"
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$ROBOT_TARGET" "echo connected" > /dev/null 2>&1; then
    echo -e "${RED}ERROR: 无法连接到 ${ROBOT_TARGET}${NC}"
    echo -e "  请确认:"
    echo -e "    1. SSH 免密登录: ssh-copy-id ${ROBOT_TARGET}"
    echo -e "    2. 机器人在线且网络可达"
    exit 1
fi
echo -e "  连接成功"

# ---- 快速同步模式 (开发用) ----
if [ "$SYNC_ONLY" = true ]; then
    echo -e "${GREEN}[3/4] rsync 同步 install/ 到机器人...${NC}"

    NAV_CURRENT="/opt/robot/navigation/current"

    # 确保远端目录存在
    ssh "$ROBOT_TARGET" "sudo mkdir -p ${NAV_CURRENT}/install && sudo chown -R \$(whoami) ${NAV_CURRENT}"

    if [ -n "$PACKAGES_SELECT" ]; then
        # 仅同步指定包
        for pkg in $PACKAGES_SELECT; do
            if [ -d "$WORKSPACE_DIR/install/$pkg" ]; then
                echo -e "  同步: ${pkg}"
                rsync -azP --delete \
                    "$WORKSPACE_DIR/install/$pkg/" \
                    "${ROBOT_TARGET}:${NAV_CURRENT}/install/$pkg/"
            else
                echo -e "${YELLOW}  跳过: ${pkg} (install/$pkg 不存在)${NC}"
            fi
        done
    else
        # 同步整个 install/
        rsync -azP --delete \
            "$WORKSPACE_DIR/install/" \
            "${ROBOT_TARGET}:${NAV_CURRENT}/install/"
    fi

    # 同步配置文件
    rsync -azP "$WORKSPACE_DIR/config/fastdds_no_shm.xml" "${ROBOT_TARGET}:${NAV_CURRENT}/"
    if [ -d "$WORKSPACE_DIR/launch" ]; then
        rsync -azP "$WORKSPACE_DIR/launch/" "${ROBOT_TARGET}:${NAV_CURRENT}/launch/"
    fi

    echo -e "${GREEN}[4/4] 同步完成${NC}"

    if [ "$DO_RESTART" = true ]; then
        echo -e "${GREEN}重启导航服务...${NC}"
        ssh "$ROBOT_TARGET" "sudo systemctl restart navigation.service 2>/dev/null || echo '提示: systemd 服务未配置, 请手动重启'"
    fi

    echo -e "${GREEN}推送完成!${NC}"
    exit 0
fi

# ---- Step 3: 打包并传输 ----
echo -e "${GREEN}[3/4] 打包并传输...${NC}"

# 先打包
bash "$SCRIPT_DIR/build_nav_package.sh" --skip-build

# 找到产物
ARCH=$(dpkg --print-architecture 2>/dev/null || uname -m)
TARBALL="$WORKSPACE_DIR/dist/navigation-${VERSION}-${ARCH}.tar.gz"

if [ ! -f "$TARBALL" ]; then
    echo -e "${RED}ERROR: 找不到打包产物: ${TARBALL}${NC}"
    exit 1
fi

TARBALL_SIZE=$(du -sh "$TARBALL" | awk '{print $1}')
echo -e "  传输: $(basename $TARBALL) (${TARBALL_SIZE})"

# 传输到机器人
REMOTE_STAGING="/tmp/nav_ota_staging"
ssh "$ROBOT_TARGET" "mkdir -p $REMOTE_STAGING"
scp "$TARBALL" "${ROBOT_TARGET}:${REMOTE_STAGING}/"
scp "$SCRIPT_DIR/install_nav.sh" "${ROBOT_TARGET}:${REMOTE_STAGING}/"

# ---- Step 4: 远程安装 ----
echo -e "${GREEN}[4/4] 远程安装...${NC}"

PACKAGE_NAME="navigation-${VERSION}-${ARCH}"
ssh "$ROBOT_TARGET" << REMOTEEOF
set -e
cd ${REMOTE_STAGING}

# 解压
tar xzf ${PACKAGE_NAME}.tar.gz

# 安装
sudo bash install_nav.sh --package-dir ${REMOTE_STAGING}/${PACKAGE_NAME} --force

# 清理
rm -rf ${REMOTE_STAGING}

echo "远程安装完成"
REMOTEEOF

if [ "$DO_RESTART" = true ]; then
    echo -e "${GREEN}重启导航服务...${NC}"
    ssh "$ROBOT_TARGET" "sudo systemctl restart navigation.service 2>/dev/null || echo '提示: systemd 服务未配置, 请手动重启'"
fi

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${GREEN}推送完成!${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "  版本: ${VERSION}"
echo -e "  目标: ${ROBOT_TARGET}"
echo -e ""
echo -e "${YELLOW}验证:${NC}"
echo -e "  ssh ${ROBOT_TARGET} 'cat /opt/robot/navigation/current/metadata.json'"
echo -e "  ssh ${ROBOT_TARGET} 'ls -la /opt/robot/navigation/current'"
