#!/usr/bin/env bash
# record_bag.sh — 录一段 rosbag,包含 LingTu 导航栈所有关键 topic.
#
# 用法:
#   bash scripts/hardware/record_bag.sh                  # 默认 60 秒
#   bash scripts/hardware/record_bag.sh 180              # 录 180 秒
#   bash scripts/hardware/record_bag.sh 60 goto_test     # 60 秒 + 自定义前缀
#
# 包存到:~/data/bags/<prefix>_<timestamp>/
#
# Topics 选型:SLAM / 控制 / 规划 的最小必要集,方便离线回放调参。
# 不包含相机 raw(数据量爆炸),只留 camera_info 做时间对齐用.

set -e

DURATION="${1:-60}"
PREFIX="${2:-rec}"
STAMP=$(date +%Y%m%d_%H%M%S)
BAG_DIR="${HOME}/data/bags/${PREFIX}_${STAMP}"

# Source BOTH system ROS2 AND LingTu workspace (for livox_ros_driver2 custom msgs)
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
NAV_SETUP="${HOME}/data/SLAM/navigation/install/setup.bash"
if [[ -f "$NAV_SETUP" ]]; then
  source "$NAV_SETUP"
fi

mkdir -p "$(dirname "$BAG_DIR")"

echo "==== LingTu Bag Record ===="
echo "  duration : ${DURATION}s"
echo "  output   : ${BAG_DIR}"
echo ""

# Topic list — grouped by concern
TOPICS=(
  # --- SLAM input ---
  /lidar/raw_frame          # Livox CustomMsg (needs livox_ros_driver2 sourced)
  /imu/raw
  # --- SLAM output ---
  /slam/odometry
  /slam/map_cloud
  /slam/registered_cloud
  /slam/localization_quality
  /slam/localization_health
  # --- planning & control ---
  /nav/goal_pose
  /nav/cmd_vel
  # --- exploration (TARE, if running) ---
  /exploration/way_point
  /exploration/path
  /exploration/runtime
  /exploration/finish
  # --- camera meta (for time alignment, cheap) ---
  /camera/color/camera_info
  /camera/depth/camera_info
)

timeout "$DURATION" ros2 bag record \
  -s sqlite3 \
  -o "$BAG_DIR" \
  "${TOPICS[@]}" \
  2>&1 | grep -v "Discarding message because" | tail -20 || true

# Print summary
if [[ -d "$BAG_DIR" ]]; then
  SIZE=$(du -sh "$BAG_DIR" | awk '{print $1}')
  echo ""
  echo "==== DONE ===="
  echo "  Path    : $BAG_DIR"
  echo "  Size    : $SIZE"
  echo ""
  echo "Info:"
  ros2 bag info "$BAG_DIR" | grep -E "Topic|count|type" | head -20
fi
