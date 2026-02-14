#!/bin/bash
# ══════════════════════════════════════════════════════════
# 导航系统完整编译脚本
#
# 步骤:
#   1. PCT Planner C++ Core (pybind11)
#   2. ROS 2 工作区 (所有包)
#   3. OTA Daemon (独立 CMake)
#
# 用法:
#   ./build_all.sh              # 全量编译
#   ./build_all.sh --ros-only   # 仅 ROS 包
#   ./build_all.sh --pct-only   # 仅 PCT 核心
# ══════════════════════════════════════════════════════════
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

MODE="${1:-all}"

# ── 1. PCT Planner C++ Core ──
build_pct_core() {
    echo ">>> [PCT] Building PCT Planner C++ Core..."
    cd "$WORKSPACE_DIR/src/global_planning/PCT_planner/planner"
    if [ ! -f "build.sh" ]; then
        echo "Warning: build.sh not found in $(pwd), skipping PCT core"
        return 0
    fi
    ./build.sh
    cd "$WORKSPACE_DIR"
}

# ── 2. ROS 2 工作区 ──
build_ros() {
    echo ">>> [ROS] Building ROS 2 Workspace..."
    cd "$WORKSPACE_DIR"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo ">>> [ROS] Done. Run: source install/setup.bash"
}

# ── 3. OTA Daemon (独立 CMake, 不在 colcon 管理内) ──
build_ota() {
    echo ">>> [OTA] Building OTA Daemon..."
    local OTA_DIR="$WORKSPACE_DIR/src/ota_daemon"
    local OTA_BUILD="$OTA_DIR/build"
    local OTA_INSTALL="$WORKSPACE_DIR/install/ota_daemon"
    mkdir -p "$OTA_BUILD"
    cd "$OTA_BUILD"
    cmake .. -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$OTA_INSTALL"
    make -j"$(nproc)"
    make install
    cd "$WORKSPACE_DIR"
    echo ">>> [OTA] Installed to $OTA_INSTALL"
}

# ── 执行 ──
case "$MODE" in
    --pct-only)
        build_pct_core
        ;;
    --ros-only)
        build_ros
        ;;
    --ota-only)
        build_ota
        ;;
    all|*)
        build_pct_core
        echo ""
        build_ros
        echo ""
        build_ota
        ;;
esac

echo ""
echo ">>> All Done! Don't forget to: source install/setup.bash"
