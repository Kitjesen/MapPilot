#!/bin/bash
# ── nav-grpc.service 启动脚本 ──
# 启动 gRPC Gateway (远程监控/控制入口)
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

echo "[nav-grpc] Starting gRPC Gateway on port 50051..."
exec ros2 launch "${NAV_DIR}/launch/subsystems/grpc.launch.py"
