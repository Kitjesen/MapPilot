#!/usr/bin/env bash
# Run the optional native camera SHM runtime.
#
# The Orbbec native capture binary writes local binary records. The camera DDS
# publisher wraps that capture process and publishes color/depth/info topics.

set -euo pipefail

if [ -f /opt/lingtu/config/thunder-runtime-env.sh ]; then
    # shellcheck disable=SC1091
    source /opt/lingtu/config/thunder-runtime-env.sh
fi

: "${LINGTU_REPO:=/opt/lingtu/current}"
: "${LINGTU_CAMERA_DDS_BIN:=${LINGTU_REPO}/build/camera_dds/lingtu_camera_dds}"
: "${LINGTU_ORBBEC_CAPTURE_BIN:=${LINGTU_REPO}/build/orbbec_native/orbbec_capture}"
: "${LINGTU_CAMERA_STATUS_FILE:=/dev/shm/lingtu/camera_status.json}"
: "${LINGTU_CAMERA_FRAME_ID:=camera_link}"
: "${LINGTU_CAMERA_COLOR_TOPIC:=rt/camera/color}"
: "${LINGTU_CAMERA_DEPTH_TOPIC:=rt/camera/depth}"
: "${LINGTU_CAMERA_INFO_TOPIC:=rt/camera/info}"
: "${LINGTU_CAMERA_COLOR_SHM:=/lingtu_camera_color}"
: "${LINGTU_CAMERA_DEPTH_SHM:=/lingtu_camera_depth}"
: "${LINGTU_CAMERA_INFO_SHM:=/lingtu_camera_info}"
: "${LINGTU_CAMERA_SHM_SLOT_COUNT:=16}"
: "${LINGTU_CAMERA_SHM_SLOT_CAPACITY_BYTES:=1048576}"
: "${LINGTU_CAMERA_PUBLISH_IMAGE_DDS:=0}"
: "${LINGTU_DDS_DOMAIN_ID:=0}"
: "${LINGTU_CYCLONEDDS_PREFIX:=}"
: "${LINGTU_ORBBEC_SERIAL_NUMBER:=}"
: "${LINGTU_ORBBEC_UID:=}"
: "${LINGTU_ORBBEC_PRODUCT_ID:=0x0800}"
: "${LINGTU_ORBBEC_DEVICE_INDEX:=0}"
: "${LINGTU_ORBBEC_CONNECT_TIMEOUT_MS:=10000}"
: "${LINGTU_ORBBEC_SDK_CONFIG:=}"
: "${LINGTU_ORBBEC_ENABLE_FRAME_SYNC:=0}"
: "${LINGTU_CAMERA_COLOR_WIDTH:=640}"
: "${LINGTU_CAMERA_COLOR_HEIGHT:=480}"
: "${LINGTU_CAMERA_COLOR_FPS:=30}"
: "${LINGTU_CAMERA_DEPTH_WIDTH:=640}"
: "${LINGTU_CAMERA_DEPTH_HEIGHT:=480}"
: "${LINGTU_CAMERA_DEPTH_FPS:=30}"
: "${LINGTU_CAMERA_CAPTURE_TIMEOUT_MS:=1000}"
: "${LINGTU_CAMERA_STARTUP_FRAME_TIMEOUT_MS:=10000}"
: "${LINGTU_CAMERA_CAPTURE_STALE_TIMEOUT_MS:=15000}"
: "${LINGTU_CAMERA_CAPTURE_MAX_FRAMES:=}"

if [ -n "${LINGTU_CYCLONEDDS_PREFIX}" ] && [ -d "${LINGTU_CYCLONEDDS_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${LINGTU_CYCLONEDDS_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

if [ ! -x "${LINGTU_CAMERA_DDS_BIN}" ]; then
    echo "ERROR: native camera DDS publisher is missing or not executable: ${LINGTU_CAMERA_DDS_BIN}" >&2
    echo "Build it with: bash scripts/build/build_camera_dds.sh" >&2
    exit 2
fi

if [ ! -x "${LINGTU_ORBBEC_CAPTURE_BIN}" ]; then
    echo "ERROR: Orbbec native capture binary is missing or not executable: ${LINGTU_ORBBEC_CAPTURE_BIN}" >&2
    echo "Build it with: bash scripts/build/build_orbbec_native.sh" >&2
    exit 2
fi

mkdir -p "$(dirname "${LINGTU_CAMERA_STATUS_FILE}")"

capture_args=(
    --color-width "${LINGTU_CAMERA_COLOR_WIDTH}"
    --color-height "${LINGTU_CAMERA_COLOR_HEIGHT}"
    --color-fps "${LINGTU_CAMERA_COLOR_FPS}"
    --depth-width "${LINGTU_CAMERA_DEPTH_WIDTH}"
    --depth-height "${LINGTU_CAMERA_DEPTH_HEIGHT}"
    --depth-fps "${LINGTU_CAMERA_DEPTH_FPS}"
    --connect-timeout-ms "${LINGTU_ORBBEC_CONNECT_TIMEOUT_MS}"
    --timeout-ms "${LINGTU_CAMERA_CAPTURE_TIMEOUT_MS}"
    --startup-frame-timeout-ms "${LINGTU_CAMERA_STARTUP_FRAME_TIMEOUT_MS}"
)

if [ -n "${LINGTU_ORBBEC_SERIAL_NUMBER}" ]; then
    capture_args+=(--serial-number "${LINGTU_ORBBEC_SERIAL_NUMBER}")
fi
if [ -n "${LINGTU_ORBBEC_UID}" ]; then
    capture_args+=(--uid "${LINGTU_ORBBEC_UID}")
fi
if [ -n "${LINGTU_ORBBEC_PRODUCT_ID}" ] && [ "${LINGTU_ORBBEC_PRODUCT_ID}" != "0" ]; then
    capture_args+=(--product-id "${LINGTU_ORBBEC_PRODUCT_ID}")
fi
if [ -n "${LINGTU_ORBBEC_DEVICE_INDEX}" ]; then
    capture_args+=(--device-index "${LINGTU_ORBBEC_DEVICE_INDEX}")
fi
if [ -n "${LINGTU_ORBBEC_SDK_CONFIG}" ]; then
    capture_args+=(--sdk-config "${LINGTU_ORBBEC_SDK_CONFIG}")
fi
case "${LINGTU_ORBBEC_ENABLE_FRAME_SYNC}" in
    1|true|TRUE|yes|YES|on|ON)
        capture_args+=(--enable-frame-sync)
        ;;
esac
if [ -n "${LINGTU_CAMERA_CAPTURE_MAX_FRAMES}" ]; then
    capture_args+=(--max-frames "${LINGTU_CAMERA_CAPTURE_MAX_FRAMES}")
fi

runtime_args=(
    --domain-id "${LINGTU_DDS_DOMAIN_ID}"
    --capture-bin "${LINGTU_ORBBEC_CAPTURE_BIN}"
    --frame-id "${LINGTU_CAMERA_FRAME_ID}"
    --color-topic "${LINGTU_CAMERA_COLOR_TOPIC}"
    --depth-topic "${LINGTU_CAMERA_DEPTH_TOPIC}"
    --info-topic "${LINGTU_CAMERA_INFO_TOPIC}"
    --color-shm "${LINGTU_CAMERA_COLOR_SHM}"
    --depth-shm "${LINGTU_CAMERA_DEPTH_SHM}"
    --info-shm "${LINGTU_CAMERA_INFO_SHM}"
    --shm-slot-count "${LINGTU_CAMERA_SHM_SLOT_COUNT}"
    --shm-slot-capacity-bytes "${LINGTU_CAMERA_SHM_SLOT_CAPACITY_BYTES}"
    --status-file "${LINGTU_CAMERA_STATUS_FILE}"
    --capture-stale-timeout-ms "${LINGTU_CAMERA_CAPTURE_STALE_TIMEOUT_MS}"
)
case "${LINGTU_CAMERA_PUBLISH_IMAGE_DDS}" in
    1|true|TRUE|yes|YES|on|ON)
        runtime_args+=(--publish-image-dds)
        ;;
esac

exec "${LINGTU_CAMERA_DDS_BIN}" \
    "${runtime_args[@]}" \
    -- "${capture_args[@]}"
