#!/bin/bash
# ── nav-lidar.service 启动脚本 ──
# 启动 Livox MID360 LiDAR 驱动
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

echo "[nav-lidar] Starting Livox MID360 driver..."
exec ros2 launch "${NAV_DIR}/launch/subsystems/lidar.launch.py"
