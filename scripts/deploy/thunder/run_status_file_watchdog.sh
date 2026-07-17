#!/usr/bin/env bash
# Bridge an atomic status-file heartbeat to the systemd service watchdog.

set -euo pipefail

status_file=""
startup_timeout_s=30
poll_interval_s=0.5
notify_bin="${LINGTU_SYSTEMD_NOTIFY_BIN:-systemd-notify}"

usage() {
    echo "Usage: $0 --status-file PATH [--startup-timeout-s N] [--poll-interval-s N] -- COMMAND [ARG ...]" >&2
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --status-file)
            status_file="${2:-}"
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

if [ -z "${status_file}" ] || [ "$#" -eq 0 ]; then
    usage
    exit 64
fi
if ! command -v "${notify_bin}" >/dev/null 2>&1; then
    echo "status_watchdog: systemd notification command is unavailable: ${notify_bin}" >&2
    exit 69
fi

status_signature() {
    stat -c '%i:%Y:%s' "${status_file}" 2>/dev/null || true
}

child_pid=""
cleanup_child() {
    local exit_code=$?
    trap - EXIT TERM INT
    if [ -n "${child_pid}" ] && kill -0 "${child_pid}" 2>/dev/null; then
        kill -s TERM "${child_pid}" 2>/dev/null || true
        wait "${child_pid}" 2>/dev/null || true
    fi
    exit "${exit_code}"
}
forward_signal() {
    local signal="$1"
    local exit_code="$2"
    trap - TERM INT
    if [ -n "${child_pid}" ] && kill -0 "${child_pid}" 2>/dev/null; then
        kill -s "${signal}" "${child_pid}" 2>/dev/null || true
    fi
    if [ -n "${child_pid}" ]; then
        wait "${child_pid}" 2>/dev/null || true
    fi
    exit "${exit_code}"
}
trap cleanup_child EXIT
trap 'forward_signal TERM 143' TERM
trap 'forward_signal INT 130' INT

initial_signature="$(status_signature)"
last_signature="${initial_signature}"
start_epoch="$(date +%s)"
ready=0

"$@" &
child_pid=$!

while kill -0 "${child_pid}" 2>/dev/null; do
    signature="$(status_signature)"
    if [ -n "${signature}" ] && [ "${signature}" != "${last_signature}" ]; then
        last_signature="${signature}"
        if [ "${ready}" -eq 0 ]; then
            "${notify_bin}" READY=1 WATCHDOG=1 \
                "STATUS=ready; status heartbeat observed at ${status_file}"
            ready=1
        else
            "${notify_bin}" WATCHDOG=1 \
                "STATUS=healthy; status heartbeat advancing at ${status_file}"
        fi
    elif [ "${ready}" -eq 0 ]; then
        now_epoch="$(date +%s)"
        if [ $((now_epoch - start_epoch)) -ge "${startup_timeout_s}" ]; then
            echo "status_watchdog: no new status heartbeat within ${startup_timeout_s}s: ${status_file}" >&2
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
