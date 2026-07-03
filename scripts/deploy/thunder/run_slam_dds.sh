#!/usr/bin/env bash
set -euo pipefail

SLAM_BIN="${LINGTU_SLAM_BIN:-/opt/lingtu/current/build/slam_core/lingtu_slam_cyclone_runtime}"
if [ ! -x "$SLAM_BIN" ]; then
  echo "ERROR: native SLAM DDS runtime is missing or not executable: $SLAM_BIN" >&2
  echo "Build it with: LINGTU_SLAM_BUILD_DDS_RUNTIME=ON LINGTU_SLAM_BUILD_PYTHON_BINDINGS=OFF bash scripts/build/build_slam_core.sh" >&2
  exit 2
fi

args=(
  "$SLAM_BIN"
  --backend "${LINGTU_SLAM_BACKEND:-fastlio2}"
  --mode "${LINGTU_SLAM_MODE:-mapping}"
  --config "${LINGTU_SLAM_CONFIG:-/opt/lingtu/current/src/localization/fastlio2/config/mid360_s100p.yaml}"
  --domain-id "${LINGTU_DDS_DOMAIN_ID:-0}"
  --tick-hz "${LINGTU_SLAM_TICK_HZ:-50}"
  --log-status-s "${LINGTU_SLAM_LOG_STATUS_S:-5}"
  --status-json "${LINGTU_SLAM_STATUS_JSON:-/tmp/lingtu_slam_status.json}"
  --status-json-hz "${LINGTU_SLAM_STATUS_JSON_HZ:-10}"
  --cloud-snapshot-dir "${LINGTU_SLAM_CLOUD_SNAPSHOT_DIR:-/dev/shm/lingtu_slam}"
  --cloud-snapshot-hz "${LINGTU_SLAM_CLOUD_SNAPSHOT_HZ:-5}"
)

if [ -n "${LINGTU_SLAM_MAP:-}" ]; then
  args+=(--map "$LINGTU_SLAM_MAP")
fi

exec "${args[@]}"
