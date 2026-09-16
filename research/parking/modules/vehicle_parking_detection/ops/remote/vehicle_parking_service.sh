#!/usr/bin/env bash
set -Eeuo pipefail

ACTION="${ACTION:-status}"
ROOT="${ROOT:-$HOME/hongsenpang/Yolov11_project/vehicle_parking_detection}"
MODEL_ROOT="${MODEL_ROOT:-$HOME/hongsenpang/Yolov11_project/rdk_board_validate/v2_yolo11s_static_640x320_rect_rgb_int8}"
HBM="${HBM:-$MODEL_ROOT/hbm/patrol_v2_yolo11s_static_640x320_rect_rgb_int8.hbm}"
CONFIG="${CONFIG:-$ROOT/vehicle_parking_detection/configs/site_rois.yaml}"
POINT_ID="${POINT_ID:-no_parking_01}"
LOG_DIR="$ROOT/logs"
OUTPUT_ROOT="$ROOT/output"
SESSION="${SESSION:-vehicle_parking_detection}"
WATCHDOG_SESSION="${WATCHDOG_SESSION:-${SESSION}_camera_watchdog}"
WATCHDOG_PID="$LOG_DIR/${WATCHDOG_SESSION}.pid"
TOPIC="${TOPIC:-/image_combine_jpeg}"
TYPE="${TYPE:-}"
MODE="${MODE:-detect}"
MIPI_SH="${MIPI_SH:-$HOME/hongsenpang/Yolov11_project/rdk_board_validate/mipi_cam_service/remote_rdk_mipi_cam_service.sh}"
CAMERA_WATCHDOG="${CAMERA_WATCHDOG:-1}"
CAMERA_WATCHDOG_INTERVAL="${CAMERA_WATCHDOG_INTERVAL:-5}"
CAMERA_STALE_RESTART_SECONDS="${CAMERA_STALE_RESTART_SECONDS:-8}"
CAMERA_RESTART_COOLDOWN_SECONDS="${CAMERA_RESTART_COOLDOWN_SECONDS:-20}"
RUN_SECONDS="${RUN_SECONDS:-0}"
VIEW="${VIEW:-bottom}"
ROTATION="${ROTATION:-ccw90}"
RECORD_FPS="${RECORD_FPS:-15}"
PROCESS_INTERVAL="${PROCESS_INTERVAL:-0.0667}"
INFER_INTERVAL="${INFER_INTERVAL:-0.0667}"
INPUT_FORMAT="${INPUT_FORMAT:-rgb_i8_centered}"
INPUT_HEIGHT="${INPUT_HEIGHT:-640}"
INPUT_WIDTH="${INPUT_WIDTH:-320}"
HBM_BACKEND="${HBM_BACKEND:-hbm_runtime}"
CONF="${CONF:-0.15}"
IOU="${IOU:-0.50}"
DETECTION_TTL="${DETECTION_TTL:-1.0}"
TRACK_IOU_THRESHOLD="${TRACK_IOU_THRESHOLD:-0.10}"
TRACK_CENTER_DISTANCE_RATIO="${TRACK_CENTER_DISTANCE_RATIO:-0.85}"
TRACK_MAX_MISSED="${TRACK_MAX_MISSED:-12}"
TRACK_MAX_AGE_SECONDS="${TRACK_MAX_AGE_SECONDS:-30.0}"
RETURN_TAR="${RETURN_TAR:-$ROOT/vehicle_parking_return.tgz}"
ALARM_TAR="${ALARM_TAR:-$ROOT/vehicle_parking_alarms.tgz}"
SPLIT_SIZE="${SPLIT_SIZE:-128M}"

mkdir -p "$LOG_DIR" "$OUTPUT_ROOT"

is_tmux_running() {
  command -v tmux >/dev/null 2>&1 && tmux has-session -t "$SESSION" 2>/dev/null
}

is_runtime_running() {
  pgrep -af "vehicle_parking_detection.rdk_vehicle_parking_runtime" >/dev/null 2>&1
}

is_watchdog_running() {
  if [ -f "$WATCHDOG_PID" ]; then
    local pid
    pid="$(cat "$WATCHDOG_PID" 2>/dev/null || true)"
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && return 0
  fi
  return 1
}

latest_run_dir() {
  if [ -f "$LOG_DIR/latest_run_dir.txt" ]; then
    cat "$LOG_DIR/latest_run_dir.txt"
  else
    find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort | tail -n 1
  fi
}

status_service() {
  echo "=== vehicle parking detection status ==="
  echo "date: $(date)"
  echo "root: $ROOT"
  echo "output_root: $OUTPUT_ROOT"
  echo "config: $CONFIG"
  echo "point_id: $POINT_ID"
  echo "mode: $MODE"
  echo "record_fps: $RECORD_FPS"
  echo "infer_interval: $INFER_INTERVAL"
  echo "target_infer_fps: $(python3 - <<PY
v=float("$INFER_INTERVAL")
print(0 if v <= 0 else round(1.0 / v, 2))
PY
)"
  if is_tmux_running; then
    echo "tmux_session=RUNNING"
  else
    echo "tmux_session=NOT_RUNNING"
  fi
  if is_watchdog_running; then
    echo "camera_watchdog=RUNNING pid=$(cat "$WATCHDOG_PID" 2>/dev/null || true)"
  else
    echo "camera_watchdog=NOT_RUNNING"
  fi
  echo
  echo "processes:"
  pgrep -af "vehicle_parking_detection.rdk_vehicle_parking_runtime|hrt_model_exec" || true
  echo
  local latest
  latest="$(latest_run_dir || true)"
  echo "latest_run: $latest"
  if [ -n "$latest" ] && [ -d "$latest" ]; then
    echo
    echo "latest status:"
    cat "$latest/latest_status.json" 2>/dev/null || true
    echo
    echo "alarm files:"
    ls -lh "$latest"/alarm_events.jsonl "$latest"/latest_alarm_event.json 2>/dev/null || true
    find "$latest/alarm_images" -maxdepth 1 -type f 2>/dev/null | tail -n 10 || true
  fi
  echo
  echo "camera watchdog log:"
  tail -n 40 "$LOG_DIR/vehicle_parking_camera_watchdog.log" 2>/dev/null || true
  echo
  echo "last log:"
  tail -n 80 "$LOG_DIR"/vehicle_parking_*.log 2>/dev/null || true
}

start_camera_watchdog() {
  local out_dir="$1"
  if [ "$MODE" != "detect" ] || [ "$CAMERA_WATCHDOG" != "1" ]; then
    return 0
  fi
  if [ ! -f "$MIPI_SH" ]; then
    echo "WARNING: camera watchdog disabled, missing MIPI_SH=$MIPI_SH"
    return 0
  fi
  if is_watchdog_running; then
    echo "CAMERA_WATCHDOG_ALREADY_RUNNING pid=$(cat "$WATCHDOG_PID" 2>/dev/null || true)"
    return 0
  fi
  nohup env \
    ACTION=camera_watchdog \
    ROOT="$ROOT" \
    OUT_DIR="$out_dir" \
    SESSION="$SESSION" \
    MIPI_SH="$MIPI_SH" \
    CAMERA_WATCHDOG_INTERVAL="$CAMERA_WATCHDOG_INTERVAL" \
    CAMERA_STALE_RESTART_SECONDS="$CAMERA_STALE_RESTART_SECONDS" \
    CAMERA_RESTART_COOLDOWN_SECONDS="$CAMERA_RESTART_COOLDOWN_SECONDS" \
    bash "$ROOT/vehicle_parking_detection/ops/remote/vehicle_parking_service.sh" \
    >>"$LOG_DIR/vehicle_parking_camera_watchdog.log" 2>&1 &
  echo $! >"$WATCHDOG_PID"
  echo "CAMERA_WATCHDOG_STARTED pid=$(cat "$WATCHDOG_PID") stale_seconds=$CAMERA_STALE_RESTART_SECONDS interval=$CAMERA_WATCHDOG_INTERVAL"
}

stop_camera_watchdog() {
  if is_watchdog_running; then
    local pid
    pid="$(cat "$WATCHDOG_PID" 2>/dev/null || true)"
    echo "stopping camera watchdog pid=$pid"
    kill "$pid" 2>/dev/null || true
    sleep 1
    kill -9 "$pid" 2>/dev/null || true
  fi
  rm -f "$WATCHDOG_PID"
}

camera_watchdog_loop() {
  local out_dir="${OUT_DIR:-$(latest_run_dir || true)}"
  local last_restart_at=0
  echo "=== vehicle parking camera watchdog start ==="
  date
  echo "out_dir=$out_dir"
  echo "mipi_sh=$MIPI_SH"
  echo "stale_restart_seconds=$CAMERA_STALE_RESTART_SECONDS interval=$CAMERA_WATCHDOG_INTERVAL cooldown=$CAMERA_RESTART_COOLDOWN_SECONDS"
  while true; do
    if ! is_runtime_running; then
      echo "$(date '+%F %T') runtime not running; watchdog exits"
      return 0
    fi
    local status="$out_dir/latest_status.json"
    if [ -f "$status" ]; then
      local values stale frames
      values="$(python3 - "$status" <<'PY' 2>/dev/null || true
import json, sys
path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)
print(float(data.get("source_stale_seconds") or 0.0), int(data.get("latest_source_frame_id") or -1))
PY
)"
      stale="$(echo "$values" | awk '{print $1}')"
      frames="$(echo "$values" | awk '{print $2}')"
      stale="${stale:-0}"
      frames="${frames:--1}"
      local now
      now="$(date +%s)"
      if python3 - "$stale" "$CAMERA_STALE_RESTART_SECONDS" "$frames" <<'PY' >/dev/null 2>&1
import sys
stale = float(sys.argv[1])
threshold = float(sys.argv[2])
frames = int(float(sys.argv[3]))
raise SystemExit(0 if frames >= 0 and stale >= threshold else 1)
PY
      then
        if [ $((now - last_restart_at)) -ge "$CAMERA_RESTART_COOLDOWN_SECONDS" ]; then
          echo "$(date '+%F %T') source stale ${stale}s >= ${CAMERA_STALE_RESTART_SECONDS}s; restarting MIPI camera"
          ACTION=stop bash "$MIPI_SH" >>"$LOG_DIR/vehicle_parking_camera_watchdog.log" 2>&1 || true
          sleep 2
          ACTION=start bash "$MIPI_SH" >>"$LOG_DIR/vehicle_parking_camera_watchdog.log" 2>&1 || true
          last_restart_at="$now"
          echo "$(date '+%F %T') MIPI restart requested"
        fi
      fi
    fi
    sleep "$CAMERA_WATCHDOG_INTERVAL"
  done
}

start_service() {
  if [ ! -f "$CONFIG" ]; then
    echo "ERROR: missing config: $CONFIG"
    return 2
  fi
  if [ "$MODE" = "detect" ] && [ ! -f "$HBM" ]; then
    echo "ERROR: missing HBM: $HBM"
    return 3
  fi
  if [ ! -f /opt/tros/humble/setup.bash ]; then
    echo "ERROR: /opt/tros/humble/setup.bash not found"
    return 4
  fi
  if ! python3 -c "import yaml, cv2, numpy" >/dev/null 2>&1; then
    echo "ERROR: board Python needs PyYAML, OpenCV, and NumPy"
    return 5
  fi
  if [ "$MODE" = "detect" ] && ! python3 -c "import hbm_runtime" >/dev/null 2>&1; then
    echo "ERROR: hbm_runtime is required for high-FPS detect mode"
    return 6
  fi
  if is_tmux_running || is_runtime_running; then
    echo "VEHICLE_PARKING_ALREADY_RUNNING"
    status_service
    return 0
  fi

  local run_name="${RUN_NAME:-$(date +%Y%m%d_%H%M%S)}"
  local out_dir="$OUTPUT_ROOT/$run_name"
  mkdir -p "$out_dir"
  echo "$out_dir" >"$LOG_DIR/latest_run_dir.txt"

  local hbm_arg=""
  if [ "$MODE" = "detect" ]; then
    hbm_arg="--hbm '$HBM'"
  fi

  local run_cmd
  run_cmd="cd '$ROOT'; export PYTHONPATH='$ROOT'; set +u; source /opt/tros/humble/setup.bash; set -u; echo '=== vehicle parking runtime start ==='; date; python3 -m vehicle_parking_detection.rdk_vehicle_parking_runtime --mode '$MODE' --config '$CONFIG' --point-id '$POINT_ID' --topic '$TOPIC' --type '$TYPE' $hbm_arg --output-dir '$out_dir' --seconds '$RUN_SECONDS' --view '$VIEW' --rotation '$ROTATION' --record-fps '$RECORD_FPS' --process-interval '$PROCESS_INTERVAL' --infer-interval '$INFER_INTERVAL' --input-format '$INPUT_FORMAT' --input-height '$INPUT_HEIGHT' --input-width '$INPUT_WIDTH' --hbm-backend '$HBM_BACKEND' --conf '$CONF' --iou '$IOU' --detection-ttl '$DETECTION_TTL' --track-iou-threshold '$TRACK_IOU_THRESHOLD' --track-center-distance-ratio '$TRACK_CENTER_DISTANCE_RATIO' --track-max-missed '$TRACK_MAX_MISSED' --track-max-age-seconds '$TRACK_MAX_AGE_SECONDS' 2>&1 | tee -a '$LOG_DIR/vehicle_parking_$run_name.log'"

  if command -v tmux >/dev/null 2>&1; then
    tmux new-session -d -s "$SESSION" "bash -lc \"$run_cmd\""
    echo "VEHICLE_PARKING_STARTED mode=tmux session=$SESSION output=$out_dir"
  else
    nohup bash -lc "$run_cmd" >>"$LOG_DIR/vehicle_parking_nohup.log" 2>&1 &
    echo $! >"$LOG_DIR/vehicle_parking_nohup.pid"
    echo "VEHICLE_PARKING_STARTED mode=nohup pid=$(cat "$LOG_DIR/vehicle_parking_nohup.pid") output=$out_dir"
  fi

  local should_verify_start=1
  if [ "${RUN_SECONDS:-0}" != "0" ] && [ "${RUN_SECONDS:-0}" -le 3 ] 2>/dev/null; then
    should_verify_start=0
  fi
  sleep 3
  if [ "$should_verify_start" = "1" ] && ! is_runtime_running; then
    echo "ERROR: vehicle parking runtime exited during startup"
    tail -n 120 "$LOG_DIR/vehicle_parking_$run_name.log" 2>/dev/null || true
    status_service
    return 7
  fi
  if is_runtime_running; then
    start_camera_watchdog "$out_dir"
  fi
  status_service
}

stop_service() {
  echo "=== stop vehicle parking detection ==="
  stop_camera_watchdog
  if is_tmux_running; then
    tmux kill-session -t "$SESSION"
    echo "tmux session stopped: $SESSION"
  fi
  if is_runtime_running; then
    pkill -f "vehicle_parking_detection.rdk_vehicle_parking_runtime" || true
  fi
  status_service
}

package_all() {
  cd "$ROOT"
  local tmp="${RETURN_TAR}.tmp.$$"
  rm -f "$tmp"
  tar -czf "$tmp" logs output
  mv "$tmp" "$RETURN_TAR"
  ls -lh "$RETURN_TAR"
  echo "$RETURN_TAR"
}

package_latest() {
  cd "$ROOT"
  local latest
  latest="$(latest_run_dir || true)"
  if [ -z "$latest" ] || [ ! -d "$latest" ]; then
    echo "ERROR: no latest output run found under $OUTPUT_ROOT" >&2
    return 2
  fi
  local run_name
  run_name="$(basename "$latest")"
  local tmp="${RETURN_TAR}.tmp.$$"
  local paths=("output/$run_name")
  [ -f "logs/latest_run_dir.txt" ] && paths+=("logs/latest_run_dir.txt")
  [ -f "logs/vehicle_parking_${run_name}.log" ] && paths+=("logs/vehicle_parking_${run_name}.log")
  [ -f "logs/vehicle_parking_nohup.log" ] && paths+=("logs/vehicle_parking_nohup.log")
  [ -f "logs/vehicle_parking_nohup.pid" ] && paths+=("logs/vehicle_parking_nohup.pid")
  rm -f "$tmp"
  tar -czf "$tmp" "${paths[@]}"
  mv "$tmp" "$RETURN_TAR"
  ls -lh "$RETURN_TAR"
  echo "$RETURN_TAR"
}

package_latest_split() {
  cd "$ROOT"
  local latest
  latest="$(latest_run_dir || true)"
  if [ -z "$latest" ] || [ ! -d "$latest" ]; then
    echo "ERROR: no latest output run found under $OUTPUT_ROOT" >&2
    return 2
  fi
  local run_name
  run_name="$(basename "$latest")"
  local tmp="${RETURN_TAR}.tmp.$$"
  local paths=("output/$run_name")
  [ -f "logs/latest_run_dir.txt" ] && paths+=("logs/latest_run_dir.txt")
  [ -f "logs/vehicle_parking_${run_name}.log" ] && paths+=("logs/vehicle_parking_${run_name}.log")
  [ -f "logs/vehicle_parking_nohup.log" ] && paths+=("logs/vehicle_parking_nohup.log")
  [ -f "logs/vehicle_parking_nohup.pid" ] && paths+=("logs/vehicle_parking_nohup.pid")
  rm -f "$tmp" "$RETURN_TAR" "$RETURN_TAR".sha256 "$RETURN_TAR".part.*
  tar -czf "$tmp" "${paths[@]}"
  mv "$tmp" "$RETURN_TAR"
  sha256sum "$RETURN_TAR" >"$RETURN_TAR.sha256"
  split -b "$SPLIT_SIZE" -d -a 4 "$RETURN_TAR" "$RETURN_TAR.part."
  ls -lh "$RETURN_TAR"
  echo "parts:"
  ls -1 "$RETURN_TAR".part.*
  echo "sha256:"
  cat "$RETURN_TAR.sha256"
}

package_alarms() {
  cd "$ROOT"
  mapfile -t paths < <(find output \( -type d -name alarm_images -o -type f \( -name alarm_events.jsonl -o -name latest_alarm_event.json -o -name latest_status.json \) \) 2>/dev/null | sort)
  if [ "${#paths[@]}" -eq 0 ]; then
    mkdir -p empty_alarm_package
    tar -czf "$ALARM_TAR" empty_alarm_package
  else
    tar -czf "$ALARM_TAR" logs/latest_run_dir.txt "${paths[@]}" 2>/dev/null || tar -czf "$ALARM_TAR" "${paths[@]}"
  fi
  echo "$ALARM_TAR"
}

case "$ACTION" in
  start_record)
    MODE=record
    start_service
    ;;
  start_detect|start)
    MODE=detect
    start_service
    ;;
  status) status_service ;;
  stop) stop_service ;;
  package_all) package_all ;;
  package_latest) package_latest ;;
  package_latest_split) package_latest_split ;;
  package_alarms) package_alarms ;;
  camera_watchdog) camera_watchdog_loop ;;
  *)
    echo "ERROR: unknown ACTION=$ACTION"
    echo "Expected: start_record, start_detect, status, stop, package_all, package_latest, package_latest_split, package_alarms"
    exit 2
    ;;
esac
