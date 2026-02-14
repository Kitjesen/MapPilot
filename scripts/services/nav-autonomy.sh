#!/bin/bash
# ── nav-autonomy.service 启动脚本 ──
# 启动地形分析 + 局部规划 + 路径跟踪
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

echo "[nav-autonomy] Starting terrain analysis + local planner + path follower..."
exec ros2 launch "${NAV_DIR}/launch/subsystems/autonomy.launch.py"
