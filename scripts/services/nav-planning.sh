#!/bin/bash
# ── nav-planning.service 启动脚本 ──
# 启动 Localizer + PCT 全局规划 + 路径适配器
#
# 参数 (通过 systemd EnvironmentFile 或命令行):
#   NAV_MAP_PATH  - 地图文件路径 (不含扩展名)
#   NAV_INIT_X/Y/Z/YAW - 初始位姿
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

# 默认值 (可通过 /etc/nav/planning.env 覆盖)
NAV_INIT_X="${NAV_INIT_X:-0.0}"
NAV_INIT_Y="${NAV_INIT_Y:-0.0}"
NAV_INIT_Z="${NAV_INIT_Z:-0.0}"
NAV_INIT_YAW="${NAV_INIT_YAW:-0.0}"

echo "[nav-planning] map_path=${NAV_MAP_PATH:-<from env>}"
exec ros2 launch "${NAV_DIR}/launch/subsystems/planning.launch.py" \
    x:="$NAV_INIT_X" y:="$NAV_INIT_Y" z:="$NAV_INIT_Z" yaw:="$NAV_INIT_YAW"
