#!/usr/bin/env bash
# Give NTP a short opportunity to settle before the Livox timestamp timeline is
# anchored. Offline field starts must never be blocked by wall-clock sync.

set -uo pipefail

readonly LINGTU_TIME_SYNC_WAIT_MAX_SECONDS=15
readonly default_wait_seconds=8
requested_wait_seconds="${LINGTU_TIME_SYNC_WAIT_SECONDS:-${default_wait_seconds}}"
marker_path="${LINGTU_TIME_SYNC_WAIT_MARKER:-/run/lingtu/time-sync-wait.done}"

if [[ -e "${marker_path}" ]]; then
    echo "LingTu time sync wait: already attempted this boot; continuing immediately"
    exit 0
fi

mark_wait_complete() {
    local marker_dir
    marker_dir="$(dirname -- "${marker_path}")"
    if mkdir -p -- "${marker_dir}" 2>/dev/null; then
        : >"${marker_path}" 2>/dev/null || true
    fi
}
trap mark_wait_complete EXIT

wait_seconds="${default_wait_seconds}"
# Accept at most two canonical decimal digits. This rejects leading-zero
# values such as 08 and bounds the input before Bash arithmetic sees it.
if [[ "${requested_wait_seconds}" =~ ^(0|[1-9][0-9]?)$ ]]; then
    wait_seconds=$((10#${requested_wait_seconds}))
    if (( wait_seconds > LINGTU_TIME_SYNC_WAIT_MAX_SECONDS )); then
        wait_seconds="${LINGTU_TIME_SYNC_WAIT_MAX_SECONDS}"
    fi
else
    echo "LingTu time sync wait: invalid duration, using ${default_wait_seconds}s" >&2
fi

if (( wait_seconds == 0 )); then
    echo "LingTu time sync wait: disabled; continuing immediately"
    exit 0
fi

if ! command -v timedatectl >/dev/null 2>&1 || ! command -v timeout >/dev/null 2>&1; then
    echo "LingTu time sync wait: timing tools unavailable; continuing without synchronized wall clock" >&2
    exit 0
fi

if timeout --signal=TERM --kill-after=1s "${wait_seconds}s" bash -c '
    while true; do
        state="$(timedatectl show --property=NTPSynchronized --value 2>/dev/null || true)"
        case "${state}" in
            yes|true|1) exit 0 ;;
        esac
        sleep 0.25
    done
'; then
    echo "LingTu time sync wait: wall clock synchronized"
else
    echo "LingTu time sync wait: ${wait_seconds}s bound reached; continuing without synchronized wall clock" >&2
fi

exit 0
