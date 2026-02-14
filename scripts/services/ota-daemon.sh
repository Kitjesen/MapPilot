#!/bin/bash
# ── ota-daemon.service 启动脚本 ──
# 启动 OTA Daemon (文件管理/OTA更新, 端口 50052)
# 注意: OTA Daemon 不依赖 ROS2, 但 source env.sh 以获取 NAV_DIR
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

CONFIG="${NAV_DIR}/src/ota_daemon/config/ota_daemon.yaml"

echo "[ota-daemon] Starting OTA Daemon on port 50052..."
exec "${NAV_DIR}/install/ota_daemon/bin/ota_daemon" -c "$CONFIG"
