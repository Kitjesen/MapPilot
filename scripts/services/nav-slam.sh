#!/bin/bash
# ── nav-slam.service 启动脚本 ──
# 启动 Fast-LIO2 SLAM (remappings 定义在 launch/subsystems/slam.launch.py)
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

echo "[nav-slam] Starting Fast-LIO2..."
exec ros2 launch "${NAV_DIR}/launch/subsystems/slam.launch.py"
