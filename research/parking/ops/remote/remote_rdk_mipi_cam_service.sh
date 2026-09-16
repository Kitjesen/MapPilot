#!/usr/bin/env bash
set -Eeuo pipefail

ACTION="${ACTION:-status}"
ROOT="${ROOT:-$HOME/hongsenpang/Yolov11_project/rdk_board_validate/mipi_cam_service}"
LOG_DIR="$ROOT/logs"
SESSION="${SESSION:-rdk_mipi_cam}"
LAUNCH_CMD="${LAUNCH_CMD:-ros2 launch mipi_cam mipi_cam_dual_channel_websocket.launch.py}"

mkdir -p "$LOG_DIR"

is_tmux_running() {
  command -v tmux >/dev/null 2>&1 && tmux has-session -t "$SESSION" 2>/dev/null
}

is_launch_running() {
  pgrep -af "mipi_cam_dual_channel_websocket.launch.py" >/dev/null 2>&1
}

start_service() {
  if is_tmux_running || is_launch_running; then
    echo "MIPI_CAM_ALREADY_RUNNING"
    status_service
    return 0
  fi

  if [ ! -f /opt/tros/humble/setup.bash ]; then
    echo "ERROR: /opt/tros/humble/setup.bash not found"
    return 2
  fi

  local run_cmd
  run_cmd="cd '$ROOT'; set +u; source /opt/tros/humble/setup.bash; set -u; echo '=== mipi_cam start ==='; date; $LAUNCH_CMD 2>&1 | tee -a '$LOG_DIR/mipi_cam_launch.log'"

  if command -v tmux >/dev/null 2>&1; then
    tmux new-session -d -s "$SESSION" "bash -lc \"$run_cmd\""
    echo "MIPI_CAM_STARTED mode=tmux session=$SESSION"
  else
    nohup bash -lc "$run_cmd" >>"$LOG_DIR/mipi_cam_nohup.log" 2>&1 &
    echo $! >"$LOG_DIR/mipi_cam_nohup.pid"
    echo "MIPI_CAM_STARTED mode=nohup pid=$(cat "$LOG_DIR/mipi_cam_nohup.pid")"
  fi
  sleep 2
  status_service
}

status_service() {
  echo "=== mipi_cam status ==="
  echo "date: $(date)"
  echo "root: $ROOT"
  echo "session: $SESSION"
  if is_tmux_running; then
    echo "tmux_session=RUNNING"
  else
    echo "tmux_session=NOT_RUNNING"
  fi
  echo
  echo "processes:"
  pgrep -af "mipi_cam|websocket|ros2 launch" || true
  echo
  echo "topics:"
  if [ -f /opt/tros/humble/setup.bash ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/tros/humble/setup.bash
    set -u
    ros2 topic list 2>/dev/null | grep -Ei "image|camera|mipi|jpeg|raw" || true
  fi
  echo
  echo "last log:"
  tail -n 40 "$LOG_DIR/mipi_cam_launch.log" 2>/dev/null || true
}

stop_service() {
  echo "=== stop mipi_cam ==="
  if is_tmux_running; then
    tmux kill-session -t "$SESSION"
    echo "tmux session stopped: $SESSION"
  fi
  if is_launch_running; then
    pkill -f "mipi_cam_dual_channel_websocket.launch.py" || true
    sleep 1
  fi
  pkill -f "mipi_cam_node|mipi_cam.*component|websocket" || true
  sleep 1
  status_service
}

case "$ACTION" in
  start) start_service ;;
  status) status_service ;;
  stop) stop_service ;;
  *)
    echo "ERROR: unknown ACTION=$ACTION, expected start/status/stop"
    exit 2
    ;;
esac
