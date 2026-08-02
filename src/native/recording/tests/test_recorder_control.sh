#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "usage: test_recorder_control.sh RECORDER FAKE_WORKER" >&2
  exit 2
fi

RECORDER="$1"
FAKE_WORKER="$2"
TMP_DIR="$(mktemp -d)"
SESSION_DIR=""
MANAGER_PID=""

cleanup() {
  if [ -n "$MANAGER_PID" ]; then
    kill "$MANAGER_PID" 2>/dev/null || true
  fi
  if [ -n "$SESSION_DIR" ] && [ -f "$SESSION_DIR/worker.pid" ]; then
    WORKER_PID="$(tr -cd '0-9' <"$SESSION_DIR/worker.pid")"
    if [ -n "$WORKER_PID" ]; then
      kill "$WORKER_PID" 2>/dev/null || true
    fi
  fi
  rm -rf -- "$TMP_DIR"
}
trap cleanup EXIT

"$RECORDER" start \
  --root "$TMP_DIR" \
  --prefix control-flow \
  --startup-timeout-ms 5000 \
  --product test-product \
  --run-plan-fingerprint test-fingerprint \
  --robot-id test-robot \
  --dds off \
  --camera on \
  --camera-recorder "$FAKE_WORKER" >"$TMP_DIR/start.json"

grep -Fq '"control_version":1' "$TMP_DIR/start.json"
grep -Fq '"state":"recording"' "$TMP_DIR/start.json"
SESSION_DIR="$(find "$TMP_DIR" -mindepth 1 -maxdepth 1 -type d -name 'control-flow_*' -print -quit)"
test -n "$SESSION_DIR"
MANAGER_PID="$(sed -n 's/.*"manager_pid":\([0-9][0-9]*\).*/\1/p' "$SESSION_DIR/session.json")"
test -n "$MANAGER_PID"

for _ in $(seq 1 200); do
  if [ -s "$SESSION_DIR/session.json" ] && \
     grep -Fq '"state":"recording"' "$SESSION_DIR/session.json" && \
     [ -s "$SESSION_DIR/worker.ready" ]; then
    break
  fi
  if ! kill -0 "$MANAGER_PID" 2>/dev/null; then
    echo "recording manager exited before becoming ready" >&2
    cat "$SESSION_DIR/logs/manager.log" >&2
    exit 1
  fi
  sleep 0.05
done

grep -Fq '"state":"recording"' "$SESSION_DIR/session.json"
grep -Fq '"product":"test-product"' "$SESSION_DIR/session.json"
grep -Fq '"run_plan_fingerprint":"test-fingerprint"' "$SESSION_DIR/session.json"
grep -Fq '"robot_id":"test-robot"' "$SESSION_DIR/session.json"

"$RECORDER" status "$SESSION_DIR" >"$TMP_DIR/status-live.json"
grep -Fq '"state":"recording"' "$TMP_DIR/status-live.json"
"$RECORDER" status --root "$TMP_DIR" >"$TMP_DIR/catalog-live.json"
grep -Fq '"control_version":1' "$TMP_DIR/catalog-live.json"
grep -Fq '"state":"recording"' "$TMP_DIR/catalog-live.json"

"$RECORDER" stop --root "$TMP_DIR" --timeout-ms 5000 >"$TMP_DIR/stop.json"
for _ in $(seq 1 200); do
  if ! kill -0 "$MANAGER_PID" 2>/dev/null; then
    break
  fi
  sleep 0.05
done
if kill -0 "$MANAGER_PID" 2>/dev/null; then
  echo "recording manager did not exit after stop" >&2
  exit 1
fi
MANAGER_PID=""

grep -Fq '"control_version":1' "$TMP_DIR/stop.json"
grep -Fq '"state":"completed"' "$TMP_DIR/stop.json"
grep -Fq '"state":"completed"' "$SESSION_DIR/session.json"
test -s "$SESSION_DIR/camera_color.mcap"
test -f "$SESSION_DIR/logs/manager.log"
test -s "$SESSION_DIR/logs/camera_color.stdout.log"
test -f "$SESSION_DIR/logs/camera_color.stderr.log"
"$RECORDER" status "$SESSION_DIR" >"$TMP_DIR/status-completed.json"

mkdir "$TMP_DIR/stale"
cat >"$TMP_DIR/stale/session.json" <<EOF
{"version":1,"session_id":"stale","state":"recording","session_directory":"$TMP_DIR/stale","manager_pid":2147483647}
EOF
if "$RECORDER" status "$TMP_DIR/stale" >"$TMP_DIR/status-stale.json" \
    2>"$TMP_DIR/status-stale.err"; then
  echo "status accepted a stale active session" >&2
  exit 1
fi
grep -Fq 'stale session' "$TMP_DIR/status-stale.err"
if "$RECORDER" stop "$TMP_DIR/stale" >"$TMP_DIR/stop-stale.json" \
    2>"$TMP_DIR/stop-stale.err"; then
  echo "stop accepted a stale active session" >&2
  exit 1
fi
grep -Fq 'refusing to signal a stale session' "$TMP_DIR/stop-stale.err"

(
  flock -x 9
  if "$RECORDER" start \
    --root "$TMP_DIR" \
    --prefix lock-probe \
    --dds off \
    --camera on \
    --camera-recorder "$FAKE_WORKER" \
    9>&- >"$TMP_DIR/locked-start.json"; then
    echo "start ignored the native recording control lock" >&2
    exit 1
  fi
) 9>"$TMP_DIR/.control.lock"
grep -Fq '"code":"recording_control_busy"' "$TMP_DIR/locked-start.json"
