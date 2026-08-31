#!/usr/bin/env bash
set -euo pipefail

source /opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh slam

SLAM_BIN="${LINGTU_SLAM_BIN:-/opt/lingtu/current/build/slam_core/slamd}"
SLAM_CONFIG="${LINGTU_SLAM_CONFIG:?LINGTU_SLAM_CONFIG is required from the Product session}"
if [ ! -x "$SLAM_BIN" ]; then
  echo "ERROR: native SLAM DDS runtime is missing or not executable: $SLAM_BIN" >&2
  echo "Build it with: LINGTU_SLAM_BUILD_DDS_RUNTIME=ON bash scripts/build/build_slam_core.sh" >&2
  exit 2
fi

args=(
  "$SLAM_BIN"
  --backend "${LINGTU_SLAM_BACKEND:-fastlio2}"
  --mode "${LINGTU_SLAM_MODE:-mapping}"
  --config "${SLAM_CONFIG}"
  --domain-id "${LINGTU_DDS_DOMAIN_ID:-0}"
  --tick-hz "${LINGTU_SLAM_TICK_HZ:-50}"
  --log-status-s "${LINGTU_SLAM_LOG_STATUS_S:-5}"
  --status-json "${LINGTU_SLAM_STATUS_JSON:-/tmp/lingtu_slam_status.json}"
  --status-json-hz "${LINGTU_SLAM_STATUS_JSON_HZ:-10}"
  --cloud-snapshot-dir "${LINGTU_SLAM_CLOUD_SNAPSHOT_DIR:-/dev/shm/lingtu_slam}"
  --cloud-snapshot-hz "${LINGTU_SLAM_CLOUD_SNAPSHOT_HZ:-5}"
  --lidar-scan-snapshot-hz "${LINGTU_SLAM_LIDAR_SCAN_SNAPSHOT_HZ:-10}"
  --track-against-map-period-s "${LINGTU_SLAM_TRACK_AGAINST_MAP_PERIOD_S:-5}"
)

if [ -n "${LINGTU_SLAM_MAP:-}" ]; then
  args+=(--map "$LINGTU_SLAM_MAP")
fi

track_seed_file="${LINGTU_SLAM_TRACK_SEED_FILE:-}"
if [ -z "$track_seed_file" ] && [ -n "${LINGTU_SLAM_MAP:-}" ]; then
  track_seed_file="$(dirname "$LINGTU_SLAM_MAP")/track_seed.json"
fi
if [ -n "$track_seed_file" ]; then
  args+=(--track-against-map-seed-file "$track_seed_file")
fi

if [ -n "${LINGTU_SLAM_TRACK_INITIAL_YAW:-}" ]; then
  args+=(
    --track-against-map-initial-pose
    "${LINGTU_SLAM_TRACK_INITIAL_X:-0}"
    "${LINGTU_SLAM_TRACK_INITIAL_Y:-0}"
    "${LINGTU_SLAM_TRACK_INITIAL_Z:-0}"
    "$LINGTU_SLAM_TRACK_INITIAL_YAW"
  )
fi

exec "${args[@]}"
