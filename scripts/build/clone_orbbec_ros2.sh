#!/usr/bin/env bash
set -euo pipefail

REPO_URL="${LINGTU_ORBBEC_ROS2_REPO:-https://github.com/orbbec/OrbbecSDK_ROS2.git}"
REF="${LINGTU_ORBBEC_ROS2_REF:-v2.8.6}"
DEST="${LINGTU_ORBBEC_ROS2_DIR:-src/drivers/real/camera/deps/orbbec/OrbbecSDK_ROS2}"

if [[ -d "$DEST/.git" ]]; then
  git -C "$DEST" fetch --tags --depth 1 origin "$REF"
  git -C "$DEST" checkout "$REF"
else
  if [[ -e "$DEST" ]]; then
    echo "refusing to overwrite non-git path: $DEST" >&2
    exit 1
  fi
  mkdir -p "$(dirname "$DEST")"
  git clone --depth 1 --branch "$REF" "$REPO_URL" "$DEST"
fi

test -f "$DEST/orbbec_camera/launch/gemini_330_series.launch.py"
test -f "$DEST/orbbec_camera/package.xml"
test -f "$DEST/orbbec_camera_msgs/package.xml"
test -f "$DEST/orbbec_description/package.xml"

echo "Orbbec ROS2 driver ready at $DEST ($(git -C "$DEST" describe --tags --always))"
