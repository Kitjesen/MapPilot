#!/bin/bash
# ============================================================================
# install_nav.sh — 在机器人端安装导航功能包 (原子安装 + 自动回滚)
#
# 此脚本随 navigation-*.tar.gz 一起打包，在机器人端执行。
# 也可由 OTA 系统的 apply_action="install_script" 自动调用。
#
# 用法:
#   sudo ./install_nav.sh                # 安装当前目录的包
#   sudo ./install_nav.sh --rollback     # 回滚到上一版本
#   sudo ./install_nav.sh --dry-run      # 仅检查，不安装
#
# 安装路径:
#   /opt/lingtu/nav/                    — 导航系统根目录
#   /opt/lingtu/nav/install/            — colcon install 产物
#   /opt/lingtu/nav/current -> v1.0.0  — 指向当前版本的软链接
#   /opt/lingtu/nav/v1.0.0/             — 版本化安装目录
#   /opt/lingtu/nav/previous -> v0.9.0 — 指向上一版本 (回滚用)
# ============================================================================
set -euo pipefail

# ---- 颜色 ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ---- 配置 ----
NAV_ROOT="/opt/lingtu/nav"
OTA_DIR="/opt/lingtu/nav/ota"
BACKUP_DIR="${OTA_DIR}/backup"
SERVICE_NAME="navigation.service"  # systemd 服务名
MAX_VERSIONS=3  # 最多保留多少个旧版本

# ---- 参数解析 ----
ROLLBACK=false
DRY_RUN=false
FORCE=false
PACKAGE_DIR=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --rollback)
            ROLLBACK=true
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --force)
            FORCE=true
            shift
            ;;
        --package-dir)
            PACKAGE_DIR="$2"
            shift 2
            ;;
        --help|-h)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --rollback      回滚到上一版本"
            echo "  --dry-run       仅检查，不安装"
            echo "  --force         跳过安全检查"
            echo "  --package-dir   指定解压后的包目录"
            echo "  -h, --help      显示帮助"
            exit 0
            ;;
        *)
            # 如果是目录，当作 package-dir
            if [ -d "$1" ]; then
                PACKAGE_DIR="$1"
            else
                echo -e "${RED}未知参数: $1${NC}"
                exit 1
            fi
            shift
            ;;
    esac
done

# ---- 回滚模式 ----
if [ "$ROLLBACK" = true ]; then
    echo -e "${YELLOW}=== 回滚模式 ===${NC}"

    if [ ! -L "$NAV_ROOT/previous" ]; then
        echo -e "${RED}ERROR: 没有可回滚的版本 (previous 链接不存在)${NC}"
        exit 1
    fi

    PREV_VERSION=$(readlink "$NAV_ROOT/previous" | xargs basename)
    CURR_VERSION=$(readlink "$NAV_ROOT/current" | xargs basename 2>/dev/null || echo "none")

    echo -e "  当前版本: ${CURR_VERSION}"
    echo -e "  回滚到:   ${PREV_VERSION}"

    if [ "$DRY_RUN" = true ]; then
        echo -e "${YELLOW}DRY-RUN: 不执行实际操作${NC}"
        exit 0
    fi

    # 停止服务
    echo -e "${GREEN}停止导航服务...${NC}"
    systemctl stop "$SERVICE_NAME" 2>/dev/null || true
    sleep 2

    # 切换链接
    ln -sfn "$NAV_ROOT/$PREV_VERSION" "$NAV_ROOT/current"

    # 记录回滚事件
    echo "{\"event\":\"rollback\",\"from\":\"$CURR_VERSION\",\"to\":\"$PREV_VERSION\",\"time\":\"$(date -u +%Y-%m-%dT%H:%M:%SZ)\"}" \
        >> "$OTA_DIR/ota_events.log"

    # 重启服务
    echo -e "${GREEN}重启导航服务...${NC}"
    systemctl start "$SERVICE_NAME" 2>/dev/null || true

    echo -e "${GREEN}回滚完成: ${PREV_VERSION}${NC}"
    exit 0
fi

# ---- 安装模式 ----

# 确定包目录
if [ -z "$PACKAGE_DIR" ]; then
    PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi

if [ ! -f "$PACKAGE_DIR/metadata.json" ]; then
    echo -e "${RED}ERROR: 找不到 metadata.json, 当前目录不是有效的导航包${NC}"
    echo -e "  检查的路径: $PACKAGE_DIR/metadata.json"
    exit 1
fi

# Read metadata (pass path via env var to avoid shell injection through path names)
VERSION=$(LINGTU_PKG="$PACKAGE_DIR" python3 -c "import json,os; print(json.load(open(os.path.join(os.environ['LINGTU_PKG'],'metadata.json')))['version'])" 2>/dev/null || echo "unknown")
BUILD_TIME=$(LINGTU_PKG="$PACKAGE_DIR" python3 -c "import json,os; print(json.load(open(os.path.join(os.environ['LINGTU_PKG'],'metadata.json')))['build_time'])" 2>/dev/null || echo "unknown")
GIT_COMMIT=$(LINGTU_PKG="$PACKAGE_DIR" python3 -c "import json,os; print(json.load(open(os.path.join(os.environ['LINGTU_PKG'],'metadata.json')))['git_commit'])" 2>/dev/null || echo "unknown")

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  导航功能包安装 v${VERSION}${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "  构建时间: ${BUILD_TIME}"
echo -e "  Git: ${GIT_COMMIT}"
echo ""

# ---- 安全检查 ----
if [ "$FORCE" = false ]; then
    # 检查是否有导航进程在运行
    if pgrep -f "lio_node|localPlanner|local_planner|terrain_analysis|grpc_gateway" > /dev/null 2>&1; then
        echo -e "${YELLOW}WARNING: 检测到导航进程正在运行${NC}"
        echo -e "  建议先停止: sudo systemctl stop ${SERVICE_NAME}"
        echo -e "  或使用 --force 跳过此检查"

        if [ "$DRY_RUN" = false ]; then
            read -p "是否继续 (将自动停止服务)? [y/N]: " CONTINUE
            if [[ ! "$CONTINUE" =~ ^[Yy]$ ]]; then
                exit 1
            fi
        fi
    fi

    # 检查磁盘空间 (至少 500MB)
    AVAIL_MB=$(df -m "$NAV_ROOT" 2>/dev/null | tail -1 | awk '{print $4}' || echo "999999")
    AVAIL_MB=${AVAIL_MB:-999999}
    if [ "$AVAIL_MB" -lt 500 ]; then
        echo -e "${RED}ERROR: 磁盘空间不足 (可用: ${AVAIL_MB}MB, 需要: 500MB+)${NC}"
        exit 1
    fi
fi

if [ "$DRY_RUN" = true ]; then
    echo -e "${YELLOW}DRY-RUN: 检查通过, 不执行安装${NC}"
    exit 0
fi

# ---- 写入事务日志 ----
mkdir -p "$OTA_DIR/backup"
TXN_FILE="${BACKUP_DIR}/txn_navigation.json"
cat > "$TXN_FILE" << TXNEOF
{
  "artifact": "navigation",
  "version": "${VERSION}",
  "status": "installing",
  "target_path": "${NAV_ROOT}/${VERSION}",
  "started_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
TXNEOF

echo -e "${GREEN}[1/5] 事务日志已写入${NC}"

# ---- 停止服务 ----
echo -e "${GREEN}[2/5] 停止导航服务...${NC}"
systemctl stop "$SERVICE_NAME" 2>/dev/null || true
sleep 2

# 确保所有导航进程停止
pkill -f "lio_node" 2>/dev/null || true
pkill -f "localPlanner" 2>/dev/null || true
pkill -f "grpc_gateway" 2>/dev/null || true
sleep 1

# ---- 安装新版本 ----
echo -e "${GREEN}[3/5] 安装 v${VERSION}...${NC}"

mkdir -p "$NAV_ROOT"
VERSION_DIR="${NAV_ROOT}/${VERSION}"

# 如果目标版本目录已存在，备份
if [ -d "$VERSION_DIR" ]; then
    echo -e "${YELLOW}  版本目录已存在, 覆盖: ${VERSION_DIR}${NC}"
    rm -rf "$VERSION_DIR"
fi

# 复制文件
mkdir -p "$VERSION_DIR"
cp -a "$PACKAGE_DIR/install" "$VERSION_DIR/"
cp -a "$PACKAGE_DIR/metadata.json" "$VERSION_DIR/"
cp "$PACKAGE_DIR/fastdds_no_shm.xml" "$VERSION_DIR/" 2>/dev/null || true

if [ -d "$PACKAGE_DIR/launch" ]; then
    cp -a "$PACKAGE_DIR/launch" "$VERSION_DIR/"
fi

echo -e "  安装到: ${VERSION_DIR}"

# ---- 切换版本 (原子操作) ----
echo -e "${GREEN}[4/5] 切换版本链接...${NC}"

# 保存 previous 指向旧 current
if [ -L "$NAV_ROOT/current" ]; then
    OLD_VERSION=$(readlink "$NAV_ROOT/current" | xargs basename)
    ln -sfn "$NAV_ROOT/$OLD_VERSION" "$NAV_ROOT/previous"
    echo -e "  上一版本: ${OLD_VERSION} (已保存到 previous)"
fi

# 原子切换 current
ln -sfn "$VERSION_DIR" "$NAV_ROOT/current"
echo -e "  当前版本: ${VERSION}"

# ---- 清理旧版本 ----
# 保留最近 MAX_VERSIONS 个版本
VERSION_DIRS=$(ls -dt "$NAV_ROOT"/[0-9]* 2>/dev/null || true)
VERSION_COUNT=$(echo "$VERSION_DIRS" | wc -w)
if [ "$VERSION_COUNT" -gt "$MAX_VERSIONS" ]; then
    echo -e "  清理旧版本 (保留最近 ${MAX_VERSIONS} 个)..."
    echo "$VERSION_DIRS" | tail -n +$((MAX_VERSIONS + 1)) | while read old_dir; do
        # 不删除 current/previous 指向的目录
        if [ "$(readlink -f "$NAV_ROOT/current")" = "$(readlink -f "$old_dir")" ] ||
           [ "$(readlink -f "$NAV_ROOT/previous")" = "$(readlink -f "$old_dir")" ]; then
            continue
        fi
        echo -e "  删除: $(basename "$old_dir")"
        rm -rf "$old_dir"
    done
fi

# ---- 重启服务 ----
echo -e "${GREEN}[5/5] 启动导航服务...${NC}"
systemctl start "$SERVICE_NAME" 2>/dev/null || {
    echo -e "${YELLOW}  WARNING: systemd 服务未配置, 请手动启动${NC}"
    echo -e "  cd ${NAV_ROOT}/current && source install/setup.bash"
}

# ---- 清理事务日志 ----
rm -f "$TXN_FILE"

# 更新 installed_manifest
MANIFEST_FILE="${OTA_DIR}/installed_manifest.json"
TMP_MANIFEST="${MANIFEST_FILE}.tmp"
INSTALLED_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

# 使用 Python 更新 manifest (原子写入)
LINGTU_MANIFEST_FILE="$MANIFEST_FILE" \
LINGTU_TMP_MANIFEST="$TMP_MANIFEST" \
LINGTU_VERSION="$VERSION" \
LINGTU_INSTALLED_AT="$INSTALLED_AT" \
LINGTU_VERSION_DIR="$VERSION_DIR" \
LINGTU_GIT_COMMIT="$GIT_COMMIT" \
python3 << 'PYEOF'
import json, os

manifest_path = os.environ["LINGTU_MANIFEST_FILE"]
try:
    with open(manifest_path) as f:
        manifest = json.load(f)
except (FileNotFoundError, json.JSONDecodeError):
    manifest = {"artifacts": {}}

manifest["artifacts"]["navigation"] = {
    "version": os.environ["LINGTU_VERSION"],
    "installed_at": os.environ["LINGTU_INSTALLED_AT"],
    "install_path": os.environ["LINGTU_VERSION_DIR"],
    "git_commit": os.environ["LINGTU_GIT_COMMIT"],
}

tmp_path = os.environ["LINGTU_TMP_MANIFEST"]
with open(tmp_path, "w") as f:
    json.dump(manifest, f, indent=2)

os.rename(tmp_path, manifest_path)
PYEOF

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${GREEN}安装完成! v${VERSION}${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e ""
echo -e "  安装路径: ${NAV_ROOT}/current -> ${VERSION}"
echo -e "  回滚命令: sudo ./install_nav.sh --rollback"
echo -e ""
echo -e "${YELLOW}手动启动 (如 systemd 未配置):${NC}"
echo -e "  export NAV_DIR=${NAV_ROOT}/current"
echo -e "  source \$NAV_DIR/install/setup.bash"
echo -e "  export FASTRTPS_DEFAULT_PROFILES_FILE=\$NAV_DIR/fastdds_no_shm.xml"
echo -e "  python3 \$NAV_DIR/lingtu.py nav"
