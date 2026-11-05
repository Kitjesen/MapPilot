#!/usr/bin/env bash
# Run the Product-scoped native WebRTC preview over the existing color SHM.
# Product launch settings are supplied by session.env.

set -euo pipefail

if [ -f /opt/lingtu/config/thunder-runtime-env.sh ]; then
    # shellcheck disable=SC1091
    source /opt/lingtu/config/thunder-runtime-env.sh
fi

: "${LINGTU_REPO:=/opt/lingtu/current}"
: "${LINGTU_VIDEO_BIN:=${LINGTU_REPO}/bin/lingtu_camera_webrtc}"
: "${LINGTU_VIDEO_RUNTIME_ENV:=${LINGTU_REPO}/lib/lingtu/video/env.sh}"
: "${LINGTU_VIDEO_ENCODER:?Set an exact target encoder in /etc/lingtu/video.env}"

if [ ! -x "${LINGTU_VIDEO_BIN}" ]; then
    echo "ERROR: native camera WebRTC runtime is missing or not executable: ${LINGTU_VIDEO_BIN}" >&2
    echo "Install the target-built lingtu_camera_webrtc artifact before starting the Product." >&2
    exit 2
fi
if [ ! -r "${LINGTU_VIDEO_RUNTIME_ENV}" ]; then
    echo "ERROR: pinned camera-video runtime environment is missing: ${LINGTU_VIDEO_RUNTIME_ENV}" >&2
    echo "Install the target-built GStreamer runtime; service start never builds dependencies." >&2
    exit 2
fi

# shellcheck disable=SC1090
source "${LINGTU_VIDEO_RUNTIME_ENV}"
mkdir -p "$(dirname "${LINGTU_VIDEO_STATUS_FILE}")"

args=(
    --color-shm "${LINGTU_VIDEO_COLOR_SHM}"
    --listen "${LINGTU_VIDEO_LISTEN}"
    --encoder "${LINGTU_VIDEO_ENCODER}"
    --fps "${LINGTU_VIDEO_FPS}"
    --min-bitrate "${LINGTU_VIDEO_MIN_BITRATE}"
    --start-bitrate "${LINGTU_VIDEO_START_BITRATE}"
    --max-bitrate "${LINGTU_VIDEO_MAX_BITRATE}"
    --max-age-ms "${LINGTU_VIDEO_MAX_AGE_MS}"
    --idle-timeout-ms "${LINGTU_VIDEO_IDLE_TIMEOUT_MS}"
    --metrics-interval-ms "${LINGTU_VIDEO_METRICS_INTERVAL_MS}"
    --metrics "${LINGTU_VIDEO_METRICS_FILE}"
    --status-file "${LINGTU_VIDEO_STATUS_FILE}"
    --stun-server "${LINGTU_VIDEO_STUN_SERVER}"
)

exec "${LINGTU_VIDEO_BIN}" "${args[@]}"
