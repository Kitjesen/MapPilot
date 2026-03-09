#!/bin/bash
# ── nav-driver.service 启动脚本 ──
# 启动 han_dog_bridge — cmd_vel → gRPC Walk() 驱动层
#
# 参数 (通过 /etc/nav/driver.env 或 export 设置):
#   DOG_HOST  - brainstem gRPC 地址 (默认 127.0.0.1)
#   DOG_PORT  - brainstem gRPC 端口 (默认 13145)
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

DOG_HOST="${DOG_HOST:-127.0.0.1}"
DOG_PORT="${DOG_PORT:-13145}"

echo "[nav-driver] Connecting to brainstem at ${DOG_HOST}:${DOG_PORT}"
exec ros2 run robot_driver han_dog_bridge --ros-args \
    -r /cmd_vel:=/nav/cmd_vel \
    -r /stop:=/nav/stop \
    -r /Odometry:=/nav/dog_odometry \
    -p dog_host:="${DOG_HOST}" \
    -p dog_port:="${DOG_PORT}" \
    -p auto_enable:=true \
    -p auto_standup:=true \
    -p cmd_vel_timeout_ms:=200.0
