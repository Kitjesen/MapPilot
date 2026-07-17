#!/usr/bin/env bash
# Bridge a local HTTP liveness probe to the systemd service watchdog.

set -euo pipefail

health_url=""
startup_timeout_s=60
poll_interval_s=2
notify_bin="${LINGTU_SYSTEMD_NOTIFY_BIN:-systemd-notify}"
curl_bin="${LINGTU_HTTP_WATCHDOG_CURL_BIN:-curl}"

usage() {
    echo "Usage: $0 --health-url URL [--startup-timeout-s N] [--poll-interval-s N] -- COMMAND [ARG ...]" >&2
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --health-url)
            health_url="${2:-}"
            shift 2
            ;;
        --startup-timeout-s)
            startup_timeout_s="${2:-}"
            shift 2
            ;;
        --poll-interval-s)
            poll_interval_s="${2:-}"
            shift 2
            ;;
        --)
            shift
            break
            ;;
        *)
            usage
            exit 64
            ;;
    esac
done

if [ -z "${health_url}" ] || [ "$#" -eq 0 ]; then
    usage
    exit 64
fi
if ! command -v "${notify_bin}" >/dev/null 2>&1; then
    echo "http_watchdog: systemd notification command is unavailable: ${notify_bin}" >&2
    exit 69
fi
if ! command -v "${curl_bin}" >/dev/null 2>&1; then
    echo "http_watchdog: curl command is unavailable: ${curl_bin}" >&2
    exit 69
fi

child_pid=""
forward_signal() {
    local signal="$1"
    local exit_code="$2"
    trap - EXIT TERM INT
    if [ -n "${child_pid}" ] && kill -0 "${child_pid}" 2>/dev/null; then
        kill -s "${signal}" "${child_pid}" 2>/dev/null || true
        wait "${child_pid}" 2>/dev/null || true
    fi
    exit "${exit_code}"
}
cleanup_child() {
    local exit_code=$?
    forward_signal TERM "${exit_code}"
}
trap cleanup_child EXIT
trap 'forward_signal TERM 143' TERM
trap 'forward_signal INT 130' INT

start_epoch="$(date +%s)"
ready=0

"$@" &
child_pid=$!

while kill -0 "${child_pid}" 2>/dev/null; do
    if "${curl_bin}" --fail --silent --show-error --max-time 1 "${health_url}" >/dev/null 2>&1; then
        if [ "${ready}" -eq 0 ]; then
            "${notify_bin}" READY=1 WATCHDOG=1 \
                "STATUS=ready; HTTP liveness responded at ${health_url}"
            ready=1
        else
            "${notify_bin}" WATCHDOG=1 \
                "STATUS=healthy; HTTP liveness advancing at ${health_url}"
        fi
    elif [ "${ready}" -eq 0 ]; then
        now_epoch="$(date +%s)"
        if [ $((now_epoch - start_epoch)) -ge "${startup_timeout_s}" ]; then
            echo "http_watchdog: liveness did not respond within ${startup_timeout_s}s: ${health_url}" >&2
            forward_signal TERM 70
        fi
    fi
    sleep "${poll_interval_s}"
done

set +e
wait "${child_pid}"
child_status=$?
set -e
exit "${child_status}"
