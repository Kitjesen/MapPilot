#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  sim/scripts/launch_lingtu_gazebo_industrial_demo.sh start [--rviz] [--gate]
  sim/scripts/launch_lingtu_gazebo_industrial_demo.sh stop
  sim/scripts/launch_lingtu_gazebo_industrial_demo.sh status

Starts a visible LingTu-only Gazebo industrial-yard demo:
  Gazebo scene + Thunder proxy + LiDAR bridge + LingTu nav chain + frontier map driver + RViz.

LingTu profile contract: sim_industrial.
This never starts hardware drivers and never publishes to a real robot.
EOF
}

repo_root() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  cd "$script_dir/../.." && pwd
}

pid_alive() {
  local pid="${1:-}"
  [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1
}

kill_pid_file() {
  local file="$1"
  [[ -f "$file" ]] || return 0
  local pid
  pid="$(cat "$file" 2>/dev/null || true)"
  if pid_alive "$pid"; then
    kill "$pid" >/dev/null 2>&1 || true
    sleep 0.5
  fi
  if pid_alive "$pid"; then
    kill -9 "$pid" >/dev/null 2>&1 || true
  fi
  rm -f "$file"
}

prepare_env() {
  local root="$1"
  set +u
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f "$root/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$root/install/setup.bash"
  fi
  set -u
  export PYTHONPATH="$root/src:$root:${PYTHONPATH:-}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-82}"
  if [[ ! "$ROS_DOMAIN_ID" =~ ^[0-9]+$ || "$ROS_DOMAIN_ID" == "0" || "$ROS_DOMAIN_ID" -ge 232 ]]; then
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID is unsafe for this demo; use 1..231." >&2
    exit 2
  fi
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  export GZ_PARTITION="${GZ_PARTITION:-lingtu_industrial_${ROS_DOMAIN_ID}_$(date +%s)}"
  export IGN_PARTITION="${IGN_PARTITION:-$GZ_PARTITION}"
  export LINGTU_PROFILE="${LINGTU_PROFILE:-sim_industrial}"
}

start_demo() {
  local root="$1"
  shift
  local with_rviz=0
  local with_gate=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --rviz) with_rviz=1 ;;
      --gate) with_gate=1 ;;
      -h|--help) usage; exit 0 ;;
      *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
    esac
    shift
  done

  if [[ "${LINGTU_GAZEBO_DEMO_SKIP_PRESTOP:-0}" != "1" ]]; then
    stop_demo "$root" >/dev/null 2>&1 || true
  fi
  prepare_env "$root"

  local run_dir="${LINGTU_GAZEBO_INDUSTRIAL_RUN_DIR:-$root/artifacts/server_sim_closure/lingtu_gazebo_industrial_demo}"
  mkdir -p "$run_dir"
  rm -f "$run_dir"/*.pid "$run_dir"/*.log "$run_dir"/*report*.json "$run_dir"/*trace*.json

  local world="$root/sim/worlds/gazebo/lingtu_gazebo_industrial_park.sdf"
  local rviz_config="${LINGTU_GAZEBO_INDUSTRIAL_RVIZ:-$root/sim/planning/lingtu_industrial_demo.rviz}"
  local python_bin="${LINGTU_PYTHON:-/usr/bin/python3}"
  [[ -x "$python_bin" ]] || python_bin="python3"

  echo "run_dir=$run_dir" | tee "$run_dir/status.txt"
  echo "LINGTU_PROFILE=$LINGTU_PROFILE" | tee -a "$run_dir/status.txt"
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION" | tee -a "$run_dir/status.txt"

  ros2 launch "$root/launch/gazebo_simulation.launch.py" \
    world:="$world" \
    headless:=false \
    use_bridge:=true \
    spawn_robot:=true \
    spawn_x:=-6.0 \
    spawn_y:=0.0 \
    spawn_z:=0.0 \
    spawn_yaw:=0.0 \
    >"$run_dir/gazebo.log" 2>&1 &
  echo $! >"$run_dir/gazebo.pid"

  sleep "${LINGTU_GAZEBO_BOOT_WAIT_SEC:-8}"

  ros2 launch "$root/sim/planning/sim_navigation.launch.py" \
    use_sim_robot:=false \
    use_terrain_passthrough:=false \
    flatten_global_path_z:=true \
    use_gazebo_line_planner:=true \
    gazebo_line_require_grid:=true \
    goal_x:=12.0 \
    goal_y:=0.0 \
    goal_z:=0.0 \
    >"$run_dir/navigation.log" 2>&1 &
  echo $! >"$run_dir/navigation.pid"

  sleep "${LINGTU_NAV_BOOT_WAIT_SEC:-4}"

  local continue_after_pass="${LINGTU_GAZEBO_DEMO_CONTINUE_AFTER_PASS_SEC:-600}"
  local timeout_sec="${LINGTU_GAZEBO_DEMO_TIMEOUT_SEC:-600}"
  if [[ "$with_gate" -eq 1 ]]; then
    continue_after_pass="${LINGTU_GAZEBO_DEMO_GATE_CONTINUE_SEC:-180}"
    timeout_sec="${LINGTU_GAZEBO_DEMO_GATE_TIMEOUT_SEC:-300}"
  fi

  "$python_bin" "$root/sim/scripts/gazebo_frontier_exploration_smoke.py" \
    --timeout-sec "$timeout_sec" \
    --continue-after-pass-sec "$continue_after_pass" \
    --coverage-size-m 38 \
    --coverage-resolution-m 0.15 \
    --room-min-x -8.0 \
    --room-max-x 24.0 \
    --room-min-y -9.0 \
    --room-max-y 9.0 \
    --min-frontier-goal-x -5.0 \
    --max-trajectory-abs-y-m 9.0 \
    --min-frontier-goal-count 2 \
    --min-lidar-map-updates 3 \
    --min-free-cells 30 \
    --min-occupied-cells 10 \
    --frontier-safe-distance-m 0.80 \
    --max-local-path-unknown-ratio 1.0 \
    --require-trajectory-quality \
    --json \
    --json-out "$run_dir/frontier_report.json" \
    --trace-out "$run_dir/frontier_trace.json" \
    >"$run_dir/frontier.log" 2>&1 &
  echo $! >"$run_dir/frontier.pid"

  if [[ "$with_rviz" -eq 1 ]]; then
    rviz2 -d "$rviz_config" >"$run_dir/rviz.log" 2>&1 &
    echo $! >"$run_dir/rviz.pid"
  fi

  echo "$run_dir" >"$root/artifacts/server_sim_closure/lingtu_gazebo_industrial_demo_latest.txt"
  echo "Started LingTu Gazebo industrial demo."
  echo "Logs: $run_dir"
  echo "Stop: bash sim/scripts/launch_lingtu_gazebo_industrial_demo.sh stop"
}

stop_demo() {
  local root="$1"
  local run_dir="${LINGTU_GAZEBO_INDUSTRIAL_RUN_DIR:-$root/artifacts/server_sim_closure/lingtu_gazebo_industrial_demo}"
  kill_pid_file "$run_dir/rviz.pid"
  kill_pid_file "$run_dir/frontier.pid"
  kill_pid_file "$run_dir/navigation.pid"
  kill_pid_file "$run_dir/gazebo.pid"
  pkill -f "gazebo_frontier_exploration_smoke.py.*lingtu_gazebo_industrial_demo" >/dev/null 2>&1 || true
  pkill -f "sim_navigation.launch.py.*gazebo_line_require_grid" >/dev/null 2>&1 || true
  pkill -f "gazebo_simulation.launch.py.*lingtu_gazebo_industrial_park.sdf" >/dev/null 2>&1 || true
  pkill -f "gz_sim.launch.py.*lingtu_gazebo_industrial_park" >/dev/null 2>&1 || true
  pkill -f "ign gazebo.*lingtu_gazebo_industrial_park" >/dev/null 2>&1 || true
  pkill -f "[i]gn gazebo server" >/dev/null 2>&1 || true
  pkill -f "[i]gn gazebo gui" >/dev/null 2>&1 || true
  pkill -f "rviz2.*lingtu_industrial_demo.rviz" >/dev/null 2>&1 || true
  pkill -f "sim.engine.bridge.gazebo_runtime_adapter" >/dev/null 2>&1 || true
  pkill -f "sim.engine.bridge.gazebo_cmd_vel_adapter" >/dev/null 2>&1 || true
  pkill -f "ros_gz_bridge.*lingtu_gazebo_industrial_park" >/dev/null 2>&1 || true
  echo "Stopped LingTu Gazebo industrial demo."
}

status_demo() {
  local root="$1"
  local run_dir="${LINGTU_GAZEBO_INDUSTRIAL_RUN_DIR:-$root/artifacts/server_sim_closure/lingtu_gazebo_industrial_demo}"
  echo "run_dir=$run_dir"
  for name in gazebo navigation frontier rviz; do
    local file="$run_dir/$name.pid"
    if [[ -f "$file" ]] && pid_alive "$(cat "$file")"; then
      echo "$name: running pid=$(cat "$file")"
    else
      echo "$name: stopped"
    fi
  done
}

main() {
  local root
  root="$(repo_root)"
  local cmd="${1:-}"
  case "$cmd" in
    start) shift; start_demo "$root" "$@" ;;
    stop) stop_demo "$root" ;;
    status) status_demo "$root" ;;
    -h|--help|"") usage ;;
    *) echo "Unknown command: $cmd" >&2; usage; exit 2 ;;
  esac
}

main "$@"
