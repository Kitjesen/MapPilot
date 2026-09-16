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
