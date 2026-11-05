#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -f "$tmp/dds" "$tmp/out"; rmdir "$tmp"' EXIT
printf '#!/usr/bin/env bash\nprintf "%%s\\n" "$@"\n' > "$tmp/dds"
chmod +x "$tmp/dds"
export LINGTU_CAMERA_DDS_BIN="$tmp/dds"
export LINGTU_ORBBEC_CAPTURE_BIN=/bin/true
export LINGTU_REALSENSE_CAPTURE_BIN=/bin/true
export LINGTU_CAMERA_STATUS_FILE="$tmp/status.json"
export LINGTU_REALSENSE_SERIAL_NUMBER=test-d435i
export LINGTU_CAMERA_COLOR_WIDTH=320 LINGTU_CAMERA_COLOR_HEIGHT=240 LINGTU_CAMERA_COLOR_FPS=15
export LINGTU_CAMERA_DEPTH_WIDTH=320 LINGTU_CAMERA_DEPTH_HEIGHT=240 LINGTU_CAMERA_DEPTH_FPS=15
export LINGTU_CAMERA_CAPTURE_TIMEOUT_MS=600 LINGTU_CAMERA_STARTUP_FRAME_TIMEOUT_MS=2000
export LINGTU_CAMERA_CAPTURE_STALE_TIMEOUT_MS=3000 LINGTU_CAMERA_FRAME_ID=fixture_camera
export LINGTU_CAMERA_COLOR_SHM=/fixture_color LINGTU_CAMERA_DEPTH_SHM=/fixture_depth LINGTU_CAMERA_INFO_SHM=/fixture_info
export LINGTU_CAMERA_SHM_SLOT_COUNT=4 LINGTU_CAMERA_SHM_SLOT_CAPACITY_BYTES=1048576 LINGTU_CAMERA_PUBLISH_IMAGE_DDS=0
export LINGTU_ORBBEC_CONNECT_TIMEOUT_MS=2500 LINGTU_ORBBEC_PRODUCT_ID=0x0800 LINGTU_ORBBEC_DEVICE_INDEX=0
export LINGTU_ORBBEC_SDK_CONFIG= LINGTU_ORBBEC_ENABLE_FRAME_SYNC=0
LINGTU_CAMERA_DRIVER=realsense_native bash "$root/scripts/deploy/thunder/run_camera_dds.sh" > "$tmp/out"
grep -qx test-d435i "$tmp/out"
grep -qx -- --color-width "$tmp/out"
if grep -Eq -- '^--(product-id|uid|device-index|connect-timeout-ms|sdk-config)$' "$tmp/out"; then
  echo 'Orbbec-only arguments leaked into D435i capture' >&2; exit 1
fi
LINGTU_CAMERA_DRIVER=orbbec_native bash "$root/scripts/deploy/thunder/run_camera_dds.sh" > "$tmp/out"
grep -qx -- --product-id "$tmp/out"
if grep -qx test-d435i "$tmp/out"; then exit 1; fi
if LINGTU_CAMERA_DRIVER=unknown bash "$root/scripts/deploy/thunder/run_camera_dds.sh" > "$tmp/out" 2>&1; then
  echo 'unsupported driver accepted' >&2; exit 1
fi
echo 'camera driver launcher: PASS'
