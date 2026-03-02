#!/bin/bash
# ============================================================================
# build_nav_package.sh — 构建 ROS2 导航功能包并打包为可部署制品
#
# 输出:
#   dist/navigation-<version>-<arch>.tar.gz   — 完整导航包 (colcon install + 启动脚本)
#   dist/navigation-<version>-<arch>.sha256    — SHA256 校验文件
#
# 用法:
#   ./scripts/ota/build_nav_package.sh                    # 构建所有包
#   ./scripts/ota/build_nav_package.sh --packages-select fastlio2 local_planner  # 仅指定包
#   ./scripts/ota/build_nav_package.sh --skip-build       # 跳过编译，仅打包 (已编译时)
#   ./scripts/ota/build_nav_package.sh --incremental       # 增量包 (仅选定包)
# ============================================================================
set -euo pipefail

# ---- 颜色 ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ---- 工作空间根目录 ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$WORKSPACE_DIR"

# ---- 版本号 ----
if [ -f "$WORKSPACE_DIR/VERSION" ]; then
    VERSION=$(cat "$WORKSPACE_DIR/VERSION" | tr -d '[:space:]')
else
    VERSION="0.0.0"
    echo -e "${YELLOW}WARNING: VERSION 文件不存在, 使用默认版本 $VERSION${NC}"
fi

ARCH=$(dpkg --print-architecture 2>/dev/null || uname -m)
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
DIST_DIR="$WORKSPACE_DIR/dist"
STAGING_DIR="/tmp/nav_package_staging_$$"
PACKAGE_NAME="navigation-${VERSION}-${ARCH}"

# ---- 参数解析 ----
SKIP_BUILD=false
PACKAGES_SELECT=""
INCREMENTAL=false
BUILD_TYPE="Release"

while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --packages-select)
            shift
            PACKAGES_SELECT="$*"
            break
            ;;
        --incremental)
            INCREMENTAL=true
            shift
            ;;
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --help|-h)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --skip-build              跳过 colcon build, 仅打包"
            echo "  --packages-select PKG...  仅构建指定包"
            echo "  --incremental             增量模式: 仅打包指定包 (不含基础依赖)"
            echo "  --debug                   Debug 模式构建"
            echo "  -h, --help                显示帮助"
            exit 0
            ;;
        *)
            echo -e "${RED}未知参数: $1${NC}"
            exit 1
            ;;
    esac
done

# ---- 清理 ----
cleanup() {
    rm -rf "$STAGING_DIR" 2>/dev/null || true
}
trap cleanup EXIT

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  Navigation Package Builder v${VERSION}${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "  架构: ${ARCH}"
echo -e "  构建类型: ${BUILD_TYPE}"
echo ""

# ---- Step 1: 构建 PCT Planner C++ 核心 (如有) ----
if [ "$SKIP_BUILD" = false ]; then
    PCT_BUILD="$WORKSPACE_DIR/src/global_planning/PCT_planner/planner/build.sh"
    if [ -f "$PCT_BUILD" ]; then
        echo -e "${GREEN}[1/4] 构建 PCT Planner C++ 核心...${NC}"
        (cd "$(dirname "$PCT_BUILD")" && bash build.sh)
    else
        echo -e "${YELLOW}[1/4] 跳过 PCT Planner (build.sh 不存在)${NC}"
    fi

    # ---- Step 2: colcon build ----
    echo -e "${GREEN}[2/4] colcon build...${NC}"

    COLCON_ARGS="--cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}"

    if [ -n "$PACKAGES_SELECT" ]; then
        echo -e "  包选择: ${PACKAGES_SELECT}"
        colcon build --packages-select $PACKAGES_SELECT $COLCON_ARGS
    else
        colcon build $COLCON_ARGS
    fi
else
    echo -e "${YELLOW}[1/4] 跳过构建 (--skip-build)${NC}"
    echo -e "${YELLOW}[2/4] 跳过构建 (--skip-build)${NC}"
fi

# ---- Step 3: 验证编译产物 ----
echo -e "${GREEN}[3/4] 验证编译产物...${NC}"

if [ ! -d "$WORKSPACE_DIR/install" ]; then
    echo -e "${RED}ERROR: install/ 目录不存在, 请先 colcon build${NC}"
    exit 1
fi

# 列出已编译的包
INSTALLED_PACKAGES=$(ls "$WORKSPACE_DIR/install/" 2>/dev/null | grep -v 'setup\.' | grep -v 'local_setup\.' | grep -v '_order' || true)
PACKAGE_COUNT=$(echo "$INSTALLED_PACKAGES" | wc -w)
echo -e "  已编译的包 (${PACKAGE_COUNT}):"
for pkg in $INSTALLED_PACKAGES; do
    echo -e "    - ${pkg}"
done

# ---- Step 4: 打包 ----
echo -e "${GREEN}[4/4] 打包制品...${NC}"

mkdir -p "$DIST_DIR"
mkdir -p "$STAGING_DIR/$PACKAGE_NAME"

# 复制 install 目录 (colcon 编译产物)
echo -e "  复制 install/ 目录..."
cp -a "$WORKSPACE_DIR/install" "$STAGING_DIR/$PACKAGE_NAME/install"

# 复制运行时必需的配置和脚本
echo -e "  复制运行时文件..."
cp "$WORKSPACE_DIR/VERSION" "$STAGING_DIR/$PACKAGE_NAME/"
cp "$WORKSPACE_DIR/config/fastdds_no_shm.xml" "$STAGING_DIR/$PACKAGE_NAME/"

# 复制 launch 文件
if [ -d "$WORKSPACE_DIR/launch" ]; then
    cp -a "$WORKSPACE_DIR/launch" "$STAGING_DIR/$PACKAGE_NAME/"
fi

# 复制安装脚本 (自身)
cp "$SCRIPT_DIR/install_nav.sh" "$STAGING_DIR/$PACKAGE_NAME/" 2>/dev/null || true

# 复制 system_manifest.json (如已在 dist/ 中生成)
if [ -f "$DIST_DIR/system_manifest.json" ]; then
    cp "$DIST_DIR/system_manifest.json" "$STAGING_DIR/$PACKAGE_NAME/"
fi

# 写入元数据
cat > "$STAGING_DIR/$PACKAGE_NAME/metadata.json" << METAEOF
{
  "name": "navigation",
  "version": "${VERSION}",
  "arch": "${ARCH}",
  "build_type": "${BUILD_TYPE}",
  "build_time": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "build_host": "$(hostname)",
  "git_commit": "$(git rev-parse --short HEAD 2>/dev/null || echo 'unknown')",
  "git_branch": "$(git branch --show-current 2>/dev/null || echo 'unknown')",
  "packages": [$(echo "$INSTALLED_PACKAGES" | sed 's/^/"/;s/$/"/' | paste -sd, -)],
  "incremental": ${INCREMENTAL}
}
METAEOF

# 创建 tarball
TARBALL="${DIST_DIR}/${PACKAGE_NAME}.tar.gz"
echo -e "  创建 tarball..."
(cd "$STAGING_DIR" && tar czf "$TARBALL" "$PACKAGE_NAME")

# 计算 SHA256
SHA256=$(sha256sum "$TARBALL" | awk '{print $1}')
echo "$SHA256  ${PACKAGE_NAME}.tar.gz" > "${DIST_DIR}/${PACKAGE_NAME}.sha256"

# 输出结果
TARBALL_SIZE=$(du -sh "$TARBALL" | awk '{print $1}')

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${GREEN}打包完成!${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "  文件: ${TARBALL}"
echo -e "  大小: ${TARBALL_SIZE}"
echo -e "  SHA256: ${SHA256}"
echo -e "  包数量: ${PACKAGE_COUNT}"
echo ""
echo -e "${YELLOW}下一步:${NC}"
echo -e "  推送到机器人:"
echo -e "    ${BLUE}./scripts/ota/push_to_robot.sh sunrise@<robot_ip>${NC}"
echo -e ""
echo -e "  或生成 manifest 并发布到 GitHub:"
echo -e "    ${BLUE}python3 scripts/ota/generate_manifest.py --version v${VERSION} --artifacts-dir ./dist/ --output ./dist/manifest.json${NC}"
echo -e "    ${BLUE}gh release create v${VERSION} dist/* --title \"v${VERSION}\"${NC}"
