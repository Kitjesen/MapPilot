#!/usr/bin/env bash
# ROS2 compatibility capture for calibration and legacy dataset workflows.
# This is not a Product recorder. Field recording uses `scripts/lingtu record`
# and the native CycloneDDS/MCAP implementation.
#
# Usage:
#   bash scripts/compat/ros2/hardware/record_bag.sh
#   bash scripts/compat/ros2/hardware/record_bag.sh 180
#   bash scripts/compat/ros2/hardware/record_bag.sh 60 goto_test

set -e

DURATION="${1:-60}"
PREFIX="${2:-rec}"
STAMP=$(date +%Y%m%d_%H%M%S)
BAG_DIR="${HOME}/data/bags/${PREFIX}_${STAMP}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
NAV_SETUP="${HOME}/data/SLAM/navigation/install/setup.bash"
if [[ -f "$NAV_SETUP" ]]; then
  source "$NAV_SETUP"
fi

mkdir -p "$(dirname "$BAG_DIR")"

echo "==== LingTu ROS2 Compatibility Capture ===="
echo "  duration : ${DURATION}s"
echo "  output   : ${BAG_DIR}"
echo ""

TOPICS=(
  /lidar/raw_frame
  /imu/raw
  /slam/odometry
  /slam/map_cloud
  /slam/registered_cloud
  /slam/localization_quality
  /slam/localization_health
  /nav/goal_pose
  /nav/cmd_vel
  /exploration/way_point
  /exploration/path
  /exploration/runtime
  /exploration/finish
  /camera/color/camera_info
  /camera/depth/camera_info
)

timeout "$DURATION" ros2 bag record \
  -s sqlite3 \
  -o "$BAG_DIR" \
  "${TOPICS[@]}" \
  2>&1 | grep -v "Discarding message because" | tail -20 || true

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
