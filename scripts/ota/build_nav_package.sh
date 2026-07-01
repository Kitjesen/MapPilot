#!/bin/bash
# ============================================================================
# build_nav_package.sh 鈥?鏋勫缓 ROS2 瀵艰埅鍔熻兘鍖呭苟鎵撳寘涓哄彲閮ㄧ讲鍒跺搧
#
# 杈撳嚭:
#   dist/navigation-<version>-<arch>.tar.gz   鈥?瀹屾暣瀵艰埅鍖?(colcon install + 鍚姩鑴氭湰)
#   dist/navigation-<version>-<arch>.sha256    鈥?SHA256 鏍￠獙鏂囦欢
#
# 鐢ㄦ硶:
#   ./scripts/ota/build_nav_package.sh                    # 鏋勫缓鎵€鏈夊寘
#   ./scripts/ota/build_nav_package.sh --packages-select fastlio2 nav_kernel  # product subset
#   ./scripts/ota/build_nav_package.sh --skip-build       # 璺宠繃缂栬瘧锛屼粎鎵撳寘 (宸茬紪璇戞椂)
#   ./scripts/ota/build_nav_package.sh --incremental       # 澧為噺鍖?(浠呴€夊畾鍖?
# ============================================================================
set -euo pipefail

# ---- 棰滆壊 ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ---- 宸ヤ綔绌洪棿鏍圭洰褰?----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$WORKSPACE_DIR"

# ---- 鐗堟湰鍙?----
if [ -f "$WORKSPACE_DIR/VERSION" ]; then
    VERSION=$(cat "$WORKSPACE_DIR/VERSION" | tr -d '[:space:]')
else
    VERSION="0.0.0"
    echo -e "${YELLOW}WARNING: VERSION 鏂囦欢涓嶅瓨鍦? 浣跨敤榛樿鐗堟湰 $VERSION${NC}"
fi

ARCH=$(dpkg --print-architecture 2>/dev/null || uname -m)
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
DIST_DIR="$WORKSPACE_DIR/dist"
STAGING_DIR="/tmp/nav_package_staging_$$"
PACKAGE_NAME="navigation-${VERSION}-${ARCH}"

# ---- 鍙傛暟瑙ｆ瀽 ----
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
            echo "鐢ㄦ硶: $0 [閫夐」]"
            echo ""
            echo "閫夐」:"
            echo "  --skip-build              璺宠繃 colcon build, 浠呮墦鍖?
            echo "  --packages-select PKG...  浠呮瀯寤烘寚瀹氬寘"
            echo "  --incremental             澧為噺妯″紡: 浠呮墦鍖呮寚瀹氬寘 (涓嶅惈鍩虹渚濊禆)"
            echo "  --debug                   Debug 妯″紡鏋勫缓"
            echo "  -h, --help                鏄剧ず甯姪"
            exit 0
            ;;
        *)
            echo -e "${RED}鏈煡鍙傛暟: $1${NC}"
            exit 1
            ;;
    esac
done

# ---- 娓呯悊 ----
cleanup() {
    rm -rf "$STAGING_DIR" 2>/dev/null || true
}
trap cleanup EXIT

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  Navigation Package Builder v${VERSION}${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "  鏋舵瀯: ${ARCH}"
echo -e "  鏋勫缓绫诲瀷: ${BUILD_TYPE}"
echo ""

# ---- Step 1: 鏋勫缓 PCT Planner C++ 鏍稿績 (濡傛湁) ----
if [ "$SKIP_BUILD" = false ]; then
    PCT_BUILD="$WORKSPACE_DIR/src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/build.sh"
    if [ -f "$PCT_BUILD" ]; then
        echo -e "${GREEN}[1/4] 鏋勫缓 PCT Planner C++ 鏍稿績...${NC}"
        (cd "$(dirname "$PCT_BUILD")" && bash build.sh)
    else
        echo -e "${YELLOW}[1/4] 璺宠繃 PCT Planner (build.sh 涓嶅瓨鍦?${NC}"
    fi

    # ---- Step 2: colcon build ----
    echo -e "${GREEN}[2/4] colcon build...${NC}"

    COLCON_ARGS="--cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}"

    if [ -n "$PACKAGES_SELECT" ]; then
        echo -e "  鍖呴€夋嫨: ${PACKAGES_SELECT}"
        colcon build --packages-select $PACKAGES_SELECT $COLCON_ARGS
    else
        colcon build $COLCON_ARGS
    fi
else
    echo -e "${YELLOW}[1/4] 璺宠繃鏋勫缓 (--skip-build)${NC}"
    echo -e "${YELLOW}[2/4] 璺宠繃鏋勫缓 (--skip-build)${NC}"
fi

# ---- Step 3: 楠岃瘉缂栬瘧浜х墿 ----
echo -e "${GREEN}[3/4] 楠岃瘉缂栬瘧浜х墿...${NC}"

if [ ! -d "$WORKSPACE_DIR/install" ]; then
    echo -e "${RED}ERROR: install/ 鐩綍涓嶅瓨鍦? 璇峰厛 colcon build${NC}"
    exit 1
fi

# 鍒楀嚭宸茬紪璇戠殑鍖?INSTALLED_PACKAGES=$(ls "$WORKSPACE_DIR/install/" 2>/dev/null | grep -v 'setup\.' | grep -v 'local_setup\.' | grep -v '_order' || true)
PACKAGE_COUNT=$(echo "$INSTALLED_PACKAGES" | wc -w)
echo -e "  宸茬紪璇戠殑鍖?(${PACKAGE_COUNT}):"
for pkg in $INSTALLED_PACKAGES; do
    echo -e "    - ${pkg}"
done

# ---- Step 4: 鎵撳寘 ----
echo -e "${GREEN}[4/4] 鎵撳寘鍒跺搧...${NC}"

mkdir -p "$DIST_DIR"
mkdir -p "$STAGING_DIR/$PACKAGE_NAME"

# 澶嶅埗 install 鐩綍 (colcon 缂栬瘧浜х墿)
echo -e "  澶嶅埗 install/ 鐩綍..."
cp -a "$WORKSPACE_DIR/install" "$STAGING_DIR/$PACKAGE_NAME/install"

# 澶嶅埗杩愯鏃跺繀闇€鐨勯厤缃拰鑴氭湰
echo -e "  澶嶅埗杩愯鏃舵枃浠?.."
cp "$WORKSPACE_DIR/VERSION" "$STAGING_DIR/$PACKAGE_NAME/"
cp "$WORKSPACE_DIR/config/fastdds_no_shm.xml" "$STAGING_DIR/$PACKAGE_NAME/"

# 澶嶅埗 launch 鏂囦欢
if [ -d "$WORKSPACE_DIR/launch" ]; then
    cp -a "$WORKSPACE_DIR/launch" "$STAGING_DIR/$PACKAGE_NAME/"
fi

# 澶嶅埗瀹夎鑴氭湰 (鑷韩)
cp "$SCRIPT_DIR/install_nav.sh" "$STAGING_DIR/$PACKAGE_NAME/" 2>/dev/null || true

# 澶嶅埗 system_manifest.json (濡傚凡鍦?dist/ 涓敓鎴?
if [ -f "$DIST_DIR/system_manifest.json" ]; then
    cp "$DIST_DIR/system_manifest.json" "$STAGING_DIR/$PACKAGE_NAME/"
fi

# 鍐欏叆鍏冩暟鎹?cat > "$STAGING_DIR/$PACKAGE_NAME/metadata.json" << METAEOF
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

# 鍒涘缓 tarball
TARBALL="${DIST_DIR}/${PACKAGE_NAME}.tar.gz"
echo -e "  鍒涘缓 tarball..."
(cd "$STAGING_DIR" && tar czf "$TARBALL" "$PACKAGE_NAME")

# 璁＄畻 SHA256
SHA256=$(sha256sum "$TARBALL" | awk '{print $1}')
echo "$SHA256  ${PACKAGE_NAME}.tar.gz" > "${DIST_DIR}/${PACKAGE_NAME}.sha256"

# 杈撳嚭缁撴灉
TARBALL_SIZE=$(du -sh "$TARBALL" | awk '{print $1}')

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${GREEN}鎵撳寘瀹屾垚!${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "  鏂囦欢: ${TARBALL}"
echo -e "  澶у皬: ${TARBALL_SIZE}"
echo -e "  SHA256: ${SHA256}"
echo -e "  鍖呮暟閲? ${PACKAGE_COUNT}"
echo ""
echo -e "${YELLOW}涓嬩竴姝?${NC}"
echo -e "  鎺ㄩ€佸埌鏈哄櫒浜?"
echo -e "    ${BLUE}./scripts/ota/push_to_robot.sh sunrise@<robot_ip>${NC}"
echo -e ""
echo -e "  鎴栫敓鎴?manifest 骞跺彂甯冨埌 GitHub:"
echo -e "    ${BLUE}python3 scripts/ota/generate_manifest.py --version v${VERSION} --artifacts-dir ./dist/ --output ./dist/manifest.json${NC}"
echo -e "    ${BLUE}gh release create v${VERSION} dist/* --title \"v${VERSION}\"${NC}"
