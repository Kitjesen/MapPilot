#!/bin/bash
# ══════════════════════════════════════════════════════════
# 导航系统公共环境变量 (所有 nav-*.service 的唯一事实来源)
# ══════════════════════════════════════════════════════════
#
# NAV_DIR 推导:
#   自动从本脚本位置推导 (scripts/services/../../ = 工作区根)
#   通过 /opt/nav 符号链接调用时, 自动指向链接目标
#
# 也可以手动 export NAV_DIR 覆盖 (优先级最高)
# ══════════════════════════════════════════════════════════

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export NAV_DIR="${NAV_DIR:-$(cd "$SCRIPT_DIR/../.." && pwd)}"

export ROS_DISTRO="${ROS_DISTRO:-humble}"
# RMW: 使用系统默认 (rmw_fastrtps_cpp)
# export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

# Source ROS2 + workspace
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${NAV_DIR}/install/setup.bash"

# 日志目录
export NAV_LOG_DIR="${NAV_LOG_DIR:-${NAV_DIR}/logs}"
mkdir -p "$NAV_LOG_DIR"

# 地图目录 (供 nav-planning 等使用)
export NAV_MAP_DIR="${NAV_MAP_DIR:-${NAV_DIR}/maps}"
