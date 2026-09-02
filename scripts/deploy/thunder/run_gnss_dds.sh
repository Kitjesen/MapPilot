#!/usr/bin/env bash
# Start the native WTRTK-980 serial -> CycloneDDS GNSS publisher.

set -euo pipefail

source /opt/lingtu/config/thunder-runtime-env.sh

: "${LINGTU_REPO:=/opt/lingtu/current}"
: "${LINGTU_GNSS_DDS_BIN:=${LINGTU_REPO}/bin/lingtu_gnss_dds}"
: "${LINGTU_GNSS_DEVICE:=/dev/wtrtk980}"
: "${LINGTU_GNSS_BAUD:=115200}"
: "${LINGTU_GNSS_TIMEOUT_MS:=1000}"
: "${LINGTU_GNSS_FRAME_ID:=gnss_antenna}"
: "${LINGTU_GNSS_MAP_FRAME_ID:=map}"
: "${LINGTU_GNSS_STATUS_FILE:=/dev/shm/lingtu/gnss_status.json}"
: "${LINGTU_GNSS_FIX_TOPIC:=rt/gnss/fix}"
: "${LINGTU_GNSS_STATUS_TOPIC:=rt/gnss/status}"
: "${LINGTU_GNSS_ODOM_TOPIC:=rt/gnss/odom}"
: "${LINGTU_GNSS_PUBLISH_ODOM:=0}"
: "${LINGTU_DDS_DOMAIN_ID:=0}"
: "${LINGTU_CYCLONEDDS_PREFIX:=}"

if [ -n "${LINGTU_CYCLONEDDS_PREFIX}" ] && [ -d "${LINGTU_CYCLONEDDS_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${LINGTU_CYCLONEDDS_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

if [ ! -x "${LINGTU_GNSS_DDS_BIN}" ]; then
    echo "ERROR: native GNSS DDS publisher is missing or not executable: ${LINGTU_GNSS_DDS_BIN}" >&2
    echo "Build it with: bash scripts/build/build_gnss_dds.sh" >&2
    exit 2
fi

args=(
    --device "${LINGTU_GNSS_DEVICE}"
    --baud "${LINGTU_GNSS_BAUD}"
    --timeout-ms "${LINGTU_GNSS_TIMEOUT_MS}"
    --domain-id "${LINGTU_DDS_DOMAIN_ID}"
    --frame-id "${LINGTU_GNSS_FRAME_ID}"
    --map-frame-id "${LINGTU_GNSS_MAP_FRAME_ID}"
    --status-file "${LINGTU_GNSS_STATUS_FILE}"
    --fix-topic "${LINGTU_GNSS_FIX_TOPIC}"
    --status-topic "${LINGTU_GNSS_STATUS_TOPIC}"
    --odom-topic "${LINGTU_GNSS_ODOM_TOPIC}"
    --publish-odom "${LINGTU_GNSS_PUBLISH_ODOM}"
)

if [ -n "${LINGTU_GNSS_ORIGIN_LAT:-}" ] && [ -n "${LINGTU_GNSS_ORIGIN_LON:-}" ]; then
    args+=(--origin-lat "${LINGTU_GNSS_ORIGIN_LAT}")
    args+=(--origin-lon "${LINGTU_GNSS_ORIGIN_LON}")
    args+=(--origin-alt "${LINGTU_GNSS_ORIGIN_ALT:-0}")
fi

exec "${LINGTU_GNSS_DDS_BIN}" "${args[@]}"
