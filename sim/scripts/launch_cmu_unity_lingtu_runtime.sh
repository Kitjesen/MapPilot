#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  sim/scripts/launch_cmu_unity_lingtu_runtime.sh start [--gate] [--rviz]
  sim/scripts/launch_cmu_unity_lingtu_runtime.sh gate
  sim/scripts/launch_cmu_unity_lingtu_runtime.sh rviz
  sim/scripts/launch_cmu_unity_lingtu_runtime.sh status
  sim/scripts/launch_cmu_unity_lingtu_runtime.sh stop

Environment:
  LINGTU_CMU_AUTONOMY_WS  CMU autonomy_stack workspace
  ROS_DOMAIN_ID           isolated ROS 2 domain, defaults to 75
  DISPLAY                 GUI display, defaults to :1
  LINGTU_CMU_RUN_DIR      log/artifact directory
  LINGTU_CMU_RVIZ_CONFIG  optional RViz config override
  LINGTU_CMU_DISABLE_NATIVE_NAV
                          defaults to 1; stops CMU localPlanner/pathFollower
                          so LingTu is the only /cmd_vel source
  LINGTU_CMU_TOMOGRAM     same-source tomogram passed to LingTu PCT
  LINGTU_CMU_AUTO_TOMOGRAM
                          defaults to 1 for PCT; captures a CMU Unity
                          same-source tomogram before LingTu starts
  LINGTU_CMU_TOMOGRAM_TOPICS
                          comma-separated capture topics; defaults to
                          /nav/map_cloud,/nav/terrain_map_ext
  LINGTU_CMU_TOMOGRAM_DURATION_SEC / LINGTU_CMU_TOMOGRAM_MODE
                          defaults to 20 and official
  LINGTU_CMU_PLANNER      LingTu planner backend, defaults to pct
  LINGTU_CMU_EXPLORATION_AUTO_START
                          defaults to 1; set 0 for explicit goal demos
  LINGTU_CMU_START_CMU_TARE
                          defaults to 1; set 0 to launch CMU sensors/world only
  LINGTU_CMU_TARE_SCENARIO
                          defaults to indoor_large; passed to CMU TARE
  LINGTU_CMU_TARE_AUTOSTART
                          defaults to 1; set 0 so the runtime gate owns
                          /exploration/start instead of letting TARE start
                          before validation subscribers are ready. start --gate
                          defaults this to 0 unless explicitly set.
  LINGTU_CMU_ENABLE_FRONTIER
                          defaults to 0; set 1 to run LingTu wavefront frontier
  LINGTU_CMU_ALLOW_DIRECT_GOAL_FALLBACK
                          defaults to 1; set 0 for strict PCT validation
                          without direct-goal bypass
  LINGTU_CMU_PLAN_SAFETY_POLICY
                          start --gate defaults to reject for PCT unless set
  LINGTU_CMU_LOCAL_SCAN_ENABLE
                          defaults to 1; publishes a robot-centered crop of
                          CMU /registered_scan for LingTu/TARE
  LINGTU_CMU_TARE_REGISTERED_SCAN_TOPIC
                          defaults to /lingtu/registered_scan_local so CMU TARE
                          consumes LingTu's local scan view, not the full map
  LINGTU_CMU_NAV_CLOUD_Z_MIN / LINGTU_CMU_NAV_CLOUD_Z_MAX
                          obstacle-height filter for /nav/registered_cloud,
                          defaults to 0.30 / 2.00 to match OccupancyGridModule
  LINGTU_CMU_GATE_REQUIRE_NO_PRIMARY_REPLAN
                          defaults to 0; set 1 to forbid LingTu PCT repaired
                          goals in the gate. Repaired goals keep PCT selected
                          and are expected in cluttered exploration.
  LINGTU_CMU_GATE_REQUIRED_MAP_TOPICS
                          space- or comma-separated map-growth topics; defaults
                          to /nav/map_cloud and /nav/terrain_map_ext
  LINGTU_CMU_WAYPOINT_THRESHOLD / LINGTU_CMU_FINAL_WAYPOINT_THRESHOLD
                          LingTu waypoint reach radii for CMU exploration
  LINGTU_CMU_STUCK_TIMEOUT / LINGTU_CMU_STUCK_DIST_THRE
                          LingTu stuck detection tuning for CMU exploration
  LINGTU_CMU_DOWNSAMPLE_DIST
                          global path spacing passed to NavigationModule
  LINGTU_CMU_AUTO_SESSION defaults to 1; starts Gateway exploring session with
                          slam_profile=none for external CMU/Unity simulation
EOF
}

repo_root() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  cd "$script_dir/../.." && pwd
}

stop_runtime() {
  local cmu_ws="${LINGTU_CMU_AUTONOMY_WS:-/home/bsrl/hongsenpang/autonomy_stack_mecanum_wheel_platform}"
  kill_pattern() {
    local pattern="$1"
    local signal="${2:-TERM}"
    local pid
    while IFS= read -r pid; do
      [[ -z "$pid" || "$pid" == "$$" || "$pid" == "${PPID:-}" ]] && continue
      kill "-$signal" "$pid" 2>/dev/null || true
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  }

  kill_pattern '[c]mu_unity_lingtu_adapter'
  kill_pattern '[c]mu_unity_lingtu_stack.py'
  kill_pattern '[c]mu_unity_runtime_gate.py'
  kill_pattern '[l]aunch_cmu_unity_lingtu_runtime.sh start'
  kill_pattern '[s]ystem_simulation_with_exploration_planner.launch'
  kill_pattern '[s]ystem_simulation.launch'
  kill_pattern '[r]viz2.*vehicle_simulator.rviz'
  kill_pattern '[r]viz2.*cmu_unity_lingtu_runtime.rviz'
  kill_pattern '[M]odel.x86_64'
  kill_pattern "$cmu_ws/install/"
  kill_pattern "$cmu_ws/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64"
  sleep 1
  kill_pattern '[c]mu_unity_lingtu_adapter' KILL
  kill_pattern '[c]mu_unity_lingtu_stack.py' KILL
  kill_pattern '[c]mu_unity_runtime_gate.py' KILL
  kill_pattern '[l]aunch_cmu_unity_lingtu_runtime.sh start' KILL
  kill_pattern '[s]ystem_simulation_with_exploration_planner.launch' KILL
  kill_pattern '[s]ystem_simulation.launch' KILL
  kill_pattern '[r]viz2.*vehicle_simulator.rviz' KILL
  kill_pattern '[r]viz2.*cmu_unity_lingtu_runtime.rviz' KILL
  kill_pattern '[M]odel.x86_64' KILL
  kill_pattern "$cmu_ws/install/" KILL
  kill_pattern "$cmu_ws/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64" KILL
}

status_runtime() {
  pgrep -af 'rviz2|Model.x86_64|default_server_endpoint|sensorScanGeneration|terrainAnalysis|tare_planner_node|vehicleSimulator|cmu_unity_lingtu|pathFollower|localPlanner' || true
}

wait_for_endpoint() {
  local run_dir="$1"
  for i in $(seq 1 30); do
    if ss -ltn 2>/dev/null | grep -q ':10000'; then
      echo "endpoint_ready_after=${i}s" | tee "$run_dir/startup_state.txt"
      return 0
    fi
    sleep 1
  done
  echo "endpoint_not_ready" | tee "$run_dir/startup_state.txt"
  return 1
}

wait_for_gateway() {
  local run_dir="$1"
  local port="${LINGTU_CMU_GATEWAY_PORT:-5050}"
  for i in $(seq 1 40); do
    if curl -fsS "http://127.0.0.1:${port}/api/v1/session" >/dev/null 2>&1; then
      echo "gateway_ready_after=${i}s" | tee -a "$run_dir/startup_state.txt"
      return 0
    fi
    sleep 1
  done
  echo "gateway_not_ready" | tee -a "$run_dir/startup_state.txt"
  return 1
}

start_gateway_exploring_session() {
  local run_dir="$1"
  local port="${LINGTU_CMU_GATEWAY_PORT:-5050}"
  local session_payload explore_payload
  session_payload="$(curl -fsS \
    -X POST "http://127.0.0.1:${port}/api/v1/session/start" \
    -H 'content-type: application/json' \
    -d '{"mode":"exploring","slam_profile":"none"}' 2>&1 || true)"
  printf '%s\n' "$session_payload" > "$run_dir/session_start.json"

  explore_payload="$(curl -fsS \
    -X POST "http://127.0.0.1:${port}/api/v1/explore/start" \
    -H 'content-type: application/json' \
    -d '{}' 2>&1 || true)"
  printf '%s\n' "$explore_payload" > "$run_dir/explore_start.json"

  echo "Gateway exploring session requested"
  echo "  session_start=$run_dir/session_start.json"
  echo "  explore_start=$run_dir/explore_start.json"
}

stop_cmu_native_navigation() {
  local cmu_ws="${LINGTU_CMU_AUTONOMY_WS:-/home/bsrl/hongsenpang/autonomy_stack_mecanum_wheel_platform}"
  local patterns=(
    "$cmu_ws/install/local_planner/lib/local_planner/[l]ocalPlanner"
    "$cmu_ws/install/local_planner/lib/local_planner/[p]athFollower"
  )
  local pattern pid signal
  for signal in TERM KILL; do
    for pattern in "${patterns[@]}"; do
      while IFS= read -r pid; do
        [[ -z "$pid" || "$pid" == "$$" || "$pid" == "${PPID:-}" ]] && continue
        kill "-$signal" "$pid" 2>/dev/null || true
      done < <(pgrep -f "$pattern" 2>/dev/null || true)
    done
    sleep 0.2
  done
}

guard_cmu_native_navigation() {
  local run_dir="$1"
  (
    for _ in $(seq 1 90); do
      stop_cmu_native_navigation
      sleep 1
    done
  ) > "$run_dir/cmu_native_nav_guard.log" 2>&1 &
  echo $! > "$run_dir/cmu_native_nav_guard.pid"
}

prepare_tare_scenario() {
  local cmu_ws="$1"
  local run_dir="$2"
  local scenario src dst scenario_name tare_scan_topic manual_start
  local sed_args=()

  scenario="${LINGTU_CMU_TARE_SCENARIO:-indoor_large}"
  tare_scan_topic="${LINGTU_CMU_TARE_REGISTERED_SCAN_TOPIC:-/lingtu/registered_scan_local}"
  manual_start="${LINGTU_CMU_TARE_AUTOSTART:-1}"
  if [[ "$manual_start" != "0" && "$tare_scan_topic" == "/registered_scan" ]]; then
    printf '%s\n' "$scenario"
    return 0
  fi

  src="$cmu_ws/install/tare_planner/share/tare_planner/${scenario}.yaml"
  if [[ ! -f "$src" ]]; then
    echo "CMU TARE scenario config missing: $src" >&2
    return 2
  fi

  scenario_name="lingtu_manual_start_${ROS_DOMAIN_ID:-manual}"
  dst="$cmu_ws/install/tare_planner/share/tare_planner/${scenario_name}.yaml"
  if [[ "$manual_start" == "0" ]]; then
    sed_args+=(-e 's/kAutoStart[[:space:]]*:[[:space:]]*true/kAutoStart : false/')
  fi
  sed_args+=(-e "s#sub_registered_scan_topic_[[:space:]]*:[[:space:]]*.*#sub_registered_scan_topic_ : ${tare_scan_topic}#")
  sed -E "${sed_args[@]}" "$src" > "$dst"
  if ! grep -q 'sub_registered_scan_topic_' "$dst"; then
    printf '\nsub_registered_scan_topic_ : %s\n' "$tare_scan_topic" >> "$dst"
  fi
  if [[ "$manual_start" == "0" ]] && ! grep -q 'kAutoStart : false' "$dst"; then
    echo "Failed to disable kAutoStart in generated CMU TARE scenario: $dst" >&2
    return 2
  fi
  if ! grep -q "sub_registered_scan_topic_ : ${tare_scan_topic}" "$dst"; then
    echo "Failed to set TARE registered scan topic in generated scenario: $dst" >&2
    return 2
  fi
  echo "CMU TARE scenario=${scenario_name} registered_scan=${tare_scan_topic} manual_start=${manual_start}" \
    | tee -a "$run_dir/startup_state.txt" >&2
  printf '%s\n' "$scenario_name"
}

capture_cmu_unity_tomogram() {
  local root="$1"
  local run_dir="$2"
  local planner auto topics_raw topic duration min_points max_points voxel_size z_min z_max mode
  local attempts attempt retry_sleep
  local pcd_out tomogram_out json_out
  local topic_args=()
  local topic_values=()
  local flat_args=()

  planner="${LINGTU_CMU_PLANNER:-pct}"
  if [[ "$planner" != "pct" ]]; then
    return 0
  fi
  if [[ -n "${LINGTU_CMU_TOMOGRAM:-}" ]]; then
    echo "Using explicit CMU same-source tomogram: $LINGTU_CMU_TOMOGRAM" \
      | tee -a "$run_dir/startup_state.txt"
    return 0
  fi

  auto="${LINGTU_CMU_AUTO_TOMOGRAM:-1}"
  if [[ "$auto" != "1" ]]; then
    echo "PCT planner requires LINGTU_CMU_TOMOGRAM or LINGTU_CMU_AUTO_TOMOGRAM=1" >&2
    return 2
  fi

  topics_raw="${LINGTU_CMU_TOMOGRAM_TOPICS:-${LINGTU_CMU_TOMOGRAM_TOPIC:-/nav/map_cloud,/nav/terrain_map_ext}}"
  IFS=',' read -r -a topic_values <<< "$topics_raw"
  for topic in "${topic_values[@]}"; do
    topic="${topic//[[:space:]]/}"
    [[ -z "$topic" ]] && continue
    topic_args+=(--topic "$topic")
  done
  if [[ "${#topic_args[@]}" == "0" ]]; then
    echo "No CMU Unity tomogram capture topics configured" >&2
    return 2
  fi
  duration="${LINGTU_CMU_TOMOGRAM_DURATION_SEC:-20}"
  attempts="${LINGTU_CMU_TOMOGRAM_CAPTURE_RETRIES:-3}"
  retry_sleep="${LINGTU_CMU_TOMOGRAM_CAPTURE_RETRY_SLEEP_SEC:-5}"
  min_points="${LINGTU_CMU_TOMOGRAM_MIN_POINTS:-500}"
  max_points="${LINGTU_CMU_TOMOGRAM_MAX_POINTS:-250000}"
  voxel_size="${LINGTU_CMU_TOMOGRAM_VOXEL_SIZE:-0.08}"
  z_min="${LINGTU_CMU_TOMOGRAM_Z_MIN:--0.2}"
  z_max="${LINGTU_CMU_TOMOGRAM_Z_MAX:-2.5}"
  mode="${LINGTU_CMU_TOMOGRAM_MODE:-official}"
  if [[ "$mode" == "flat_traversability" ]]; then
    if [[ "${LINGTU_CMU_TOMOGRAM_FLAT_DEFAULT_FREE:-0}" == "1" ]]; then
      flat_args+=(--flat-default-free)
    fi
    flat_args+=(--flat-floor-z-max "${LINGTU_CMU_TOMOGRAM_FLAT_FLOOR_Z_MAX:-0.35}")
    flat_args+=(--flat-obstacle-z-min "${LINGTU_CMU_TOMOGRAM_FLAT_OBSTACLE_Z_MIN:-0.35}")
    flat_args+=(--flat-obstacle-inflation-cells "${LINGTU_CMU_TOMOGRAM_FLAT_OBSTACLE_INFLATION_CELLS:-1}")
  fi
  pcd_out="$run_dir/cmu_unity_same_source.pcd"
  tomogram_out="$run_dir/cmu_unity_same_source_tomogram.pickle"
  json_out="$run_dir/cmu_unity_tomogram_capture.json"

  echo "Capturing CMU Unity same-source tomogram for LingTu PCT" \
    | tee -a "$run_dir/startup_state.txt"
  echo "  topics=$topics_raw duration=${duration}s mode=$mode" \
    | tee -a "$run_dir/startup_state.txt"

  for attempt in $(seq 1 "$attempts"); do
    echo "  capture_attempt=$attempt/$attempts" | tee -a "$run_dir/startup_state.txt"
    if /usr/bin/python3 "$root/sim/scripts/cmu_unity_tomogram_capture.py" \
      "${topic_args[@]}" \
      --duration-sec "$duration" \
      --min-points "$min_points" \
      --max-points "$max_points" \
      --voxel-size "$voxel_size" \
      --z-min "$z_min" \
      --z-max "$z_max" \
      --pcd-out "$pcd_out" \
      --tomogram-out "$tomogram_out" \
      --build-tomogram \
      --tomogram-mode "$mode" \
      --resolution "${LINGTU_CMU_TOMOGRAM_RESOLUTION:-0.2}" \
      --slice-dh "${LINGTU_CMU_TOMOGRAM_SLICE_DH:-0.3}" \
      --ground-h "${LINGTU_CMU_TOMOGRAM_GROUND_H:-0.0}" \
      "${flat_args[@]}" \
      --json-out "$json_out" \
      > "$run_dir/cmu_unity_tomogram_capture.log" 2>&1; then
      export LINGTU_CMU_TOMOGRAM="$tomogram_out"
      echo "CMU Unity same-source tomogram ready: $LINGTU_CMU_TOMOGRAM" \
        | tee -a "$run_dir/startup_state.txt"
      return 0
    fi
    if [[ "$attempt" != "$attempts" ]]; then
      echo "CMU Unity tomogram capture attempt failed; retrying in ${retry_sleep}s" \
        | tee -a "$run_dir/startup_state.txt"
      sleep "$retry_sleep"
    fi
  done

  echo "CMU Unity tomogram capture failed after ${attempts} attempts" >&2
  return 2
}

setup_runtime_env() {
  local root cmu_ws run_dir
  root="$1"
  cmu_ws="$2"

  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS Humble setup not found at /opt/ros/humble/setup.bash" >&2
    return 2
  fi
  if [[ ! -f "$cmu_ws/install/setup.bash" ]]; then
    echo "CMU workspace is not built: $cmu_ws/install/setup.bash missing" >&2
    return 2
  fi
  if [[ ! -x "$cmu_ws/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64" ]]; then
    echo "CMU Unity binary missing under $cmu_ws" >&2
    return 2
  fi

  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$cmu_ws/install/setup.bash"
  set -u

  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-75}"
  if [[ "$ROS_DOMAIN_ID" == "0" ]]; then
    echo "Refusing default ROS_DOMAIN_ID=0 for simulation command relay" >&2
    return 2
  fi
  export DISPLAY="${DISPLAY:-:1}"
  if [[ -z "${XAUTHORITY:-}" && -f "$HOME/.Xauthority" ]]; then
    export XAUTHORITY="$HOME/.Xauthority"
  fi
  export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  export PYTHONPATH="$root/src:$root:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}"
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD="${PYTEST_DISABLE_PLUGIN_AUTOLOAD:-1}"
}

launch_operator_rviz() {
  local root cmu_ws run_dir rviz_config
  root="$(repo_root)"
  cmu_ws="${LINGTU_CMU_AUTONOMY_WS:-/home/bsrl/hongsenpang/autonomy_stack_mecanum_wheel_platform}"
  run_dir="${LINGTU_CMU_RUN_DIR:-$root/artifacts/server_sim_closure/cmu_unity_runtime_latest}"

  setup_runtime_env "$root" "$cmu_ws"
  rviz_config="${LINGTU_CMU_RVIZ_CONFIG:-$root/tests/planning/cmu_unity_lingtu_runtime.rviz}"
  if [[ ! -f "$rviz_config" ]]; then
    echo "CMU RViz config missing: $rviz_config" >&2
    return 2
  fi

  mkdir -p "$run_dir"
  if pgrep -f '[r]viz2.*vehicle_simulator.rviz' >/dev/null 2>&1 \
    || pgrep -f '[r]viz2.*cmu_unity_lingtu_runtime.rviz' >/dev/null 2>&1; then
    echo "CMU RViz operator view already running"
    return 0
  fi

  (
    cd "$cmu_ws"
    rviz2 -d "$rviz_config"
  ) > "$run_dir/rviz.log" 2>&1 &
  echo $! > "$run_dir/rviz.pid"
  echo "CMU RViz operator view started"
  echo "  config=$rviz_config"
  echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "  DISPLAY=$DISPLAY"
  echo "  log=$run_dir/rviz.log"
}

record_latest_run_dir() {
  local root="$1"
  local run_dir="$2"
  local artifacts_dir latest_value base
  artifacts_dir="$root/artifacts/server_sim_closure"
  mkdir -p "$artifacts_dir"
  latest_value="$run_dir"
  case "$run_dir" in
    "$root"/*) latest_value="${run_dir#$root/}" ;;
  esac
  printf '%s\n' "$latest_value" > "$artifacts_dir/cmu_unity_runtime_latest.txt"
  base="$(basename "$run_dir")"
  if [[ "$base" == cmu_unity_tare_pct_visible_* ]]; then
    printf '%s\n' "$latest_value" > "$artifacts_dir/cmu_unity_tare_pct_visible_latest.txt"
  fi
}

start_runtime() {
  local run_gate="$1"
  local run_rviz="$2"
  local root cmu_ws run_dir cmu_launch_file cmu_tare_scenario
  local cmu_launch_args=()
  root="$(repo_root)"
  cmu_ws="${LINGTU_CMU_AUTONOMY_WS:-/home/bsrl/hongsenpang/autonomy_stack_mecanum_wheel_platform}"
  run_dir="${LINGTU_CMU_RUN_DIR:-$root/artifacts/server_sim_closure/cmu_unity_runtime_latest}"

  setup_runtime_env "$root" "$cmu_ws"

  mkdir -p "$run_dir"
  run_dir="$(cd "$run_dir" && pwd)"
  if [[ "$run_gate" == "1" && -z "${LINGTU_CMU_TARE_AUTOSTART+x}" ]]; then
    export LINGTU_CMU_TARE_AUTOSTART=0
  fi
  if [[ "$run_gate" == "1" && "${LINGTU_CMU_PLANNER:-pct}" == "pct" && -z "${LINGTU_CMU_PLAN_SAFETY_POLICY+x}" ]]; then
    export LINGTU_CMU_PLAN_SAFETY_POLICY=reject
  fi
  record_latest_run_dir "$root" "$run_dir"
  stop_runtime
  sleep 2

  setup_runtime_env "$root" "$cmu_ws"

  (
    cd "$cmu_ws"
    cmu_launch_file="${LINGTU_CMU_LAUNCH_FILE:-system_simulation_with_exploration_planner.launch}"
    if [[ "${LINGTU_CMU_START_CMU_TARE:-1}" == "0" ]]; then
      cmu_launch_file="system_simulation.launch"
    fi
    if [[ "$cmu_launch_file" == "system_simulation_with_exploration_planner.launch" ]]; then
      cmu_tare_scenario="$(prepare_tare_scenario "$cmu_ws" "$run_dir")"
      cmu_launch_args+=(exploration_planner_config:="$cmu_tare_scenario")
    fi
    ros2 launch vehicle_simulator "$cmu_launch_file" "${cmu_launch_args[@]}"
  ) > "$run_dir/cmu_ros_launch.log" 2>&1 &
  echo $! > "$run_dir/cmu_ros_launch.pid"

  wait_for_endpoint "$run_dir"
  if [[ "${LINGTU_CMU_DISABLE_NATIVE_NAV:-1}" == "1" ]]; then
    guard_cmu_native_navigation "$run_dir"
    for _ in $(seq 1 8); do
      stop_cmu_native_navigation
      sleep 0.5
    done
    echo "CMU native localPlanner/pathFollower disabled; LingTu owns /nav/cmd_vel -> /cmd_vel"
  fi

  (
    cd "$cmu_ws"
    ./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64
  ) > "$run_dir/unity.log" 2>&1 &
  echo $! > "$run_dir/unity.pid"

  sleep 8
  if [[ "${LINGTU_CMU_DISABLE_NATIVE_NAV:-1}" == "1" ]]; then
    stop_cmu_native_navigation
  fi
  adapter_args=(--relay-cmd-vel-to-sim)
  if [[ "${LINGTU_CMU_LOCAL_SCAN_ENABLE:-1}" != "0" ]]; then
    adapter_args+=(
      --local-registered-scan-topic "${LINGTU_CMU_LOCAL_REGISTERED_SCAN_TOPIC:-/lingtu/registered_scan_local}"
      --local-scan-odom-topic "${LINGTU_CMU_LOCAL_SCAN_ODOM_TOPIC:-/state_estimation}"
      --local-scan-radius "${LINGTU_CMU_LOCAL_SCAN_RADIUS:-8.0}"
      --nav-cloud-z-min "${LINGTU_CMU_NAV_CLOUD_Z_MIN:-0.30}"
      --nav-cloud-z-max "${LINGTU_CMU_NAV_CLOUD_Z_MAX:-2.00}"
    )
    if [[ "${LINGTU_CMU_LOCAL_REGISTERED_CLOUD:-1}" != "0" ]]; then
      adapter_args+=(--local-registered-cloud)
    fi
  fi
  /usr/bin/python3 -m sim.engine.bridge.cmu_unity_lingtu_adapter "${adapter_args[@]}" \
    > "$run_dir/adapter.log" 2>&1 &
  echo $! > "$run_dir/adapter.pid"

  sleep 2
  capture_cmu_unity_tomogram "$root" "$run_dir"

  lingtu_stack_args=(--planner "${LINGTU_CMU_PLANNER:-pct}")
  if [[ -n "${LINGTU_CMU_TOMOGRAM:-}" ]]; then
    lingtu_stack_args+=(--tomogram "$LINGTU_CMU_TOMOGRAM")
  fi
  if [[ -n "${LINGTU_CMU_SAFE_GOAL_TOLERANCE:-}" ]]; then
    lingtu_stack_args+=(--safe-goal-tolerance "$LINGTU_CMU_SAFE_GOAL_TOLERANCE")
  fi
  if [[ -n "${LINGTU_CMU_PLAN_SAFETY_POLICY:-}" ]]; then
    lingtu_stack_args+=(--plan-safety-policy "$LINGTU_CMU_PLAN_SAFETY_POLICY")
  fi
  if [[ "${LINGTU_CMU_EXPLORATION_AUTO_START:-1}" == "0" ]]; then
    lingtu_stack_args+=(--no-exploration-auto-start)
  fi
  if [[ "${LINGTU_CMU_ENABLE_FRONTIER:-0}" == "1" ]]; then
    lingtu_stack_args+=(--enable-frontier)
  fi
  if [[ "${LINGTU_CMU_ALLOW_DIRECT_GOAL_FALLBACK:-1}" == "0" ]]; then
    lingtu_stack_args+=(--disable-direct-goal-fallback)
  fi
  /usr/bin/python3 "$root/sim/scripts/cmu_unity_lingtu_stack.py" "${lingtu_stack_args[@]}" \
    > "$run_dir/lingtu_stack.log" 2>&1 &
  echo $! > "$run_dir/lingtu_stack.pid"

  if [[ "${LINGTU_CMU_AUTO_SESSION:-1}" == "1" ]]; then
    if wait_for_gateway "$run_dir"; then
      start_gateway_exploring_session "$run_dir"
    fi
  fi

  echo "CMU Unity + LingTu runtime started"
  echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "  DISPLAY=$DISPLAY"
  echo "  logs=$run_dir"

  if [[ "$run_rviz" == "1" ]]; then
    launch_operator_rviz
  fi

  if [[ "$run_gate" == "1" ]]; then
    sleep 10
    local gate_args=(
      --publish-start \
      --gateway-start-exploration-session \
      --timeout-sec "${LINGTU_CMU_GATE_TIMEOUT_SEC:-180}" \
      --require-motion-progress \
      --required-progress-topic /nav/way_point \
      --min-motion-progress-m 0.25 \
      --min-unique-waypoints "${LINGTU_CMU_GATE_MIN_UNIQUE_WAYPOINTS:-2}" \
      --unique-waypoint-topic /nav/way_point \
      --require-path-topic /nav/global_path \
      --require-path-topic /nav/local_path \
      --late-window-sec "${LINGTU_CMU_GATE_LATE_WINDOW_SEC:-60}" \
      --min-late-odom-delta-m "${LINGTU_CMU_GATE_MIN_LATE_ODOM_DELTA_M:-0.5}" \
      --min-late-cmd-vel-samples "${LINGTU_CMU_GATE_MIN_LATE_CMD_VEL_SAMPLES:-10}" \
      --min-late-path-samples "${LINGTU_CMU_GATE_MIN_LATE_PATH_SAMPLES:-5}" \
      --min-late-map-area-delta-m2 "${LINGTU_CMU_GATE_MIN_LATE_MAP_AREA_DELTA_M2:-1.0}" \
      --require-tare-strategy-quality \
      --min-tare-strategy-paths "${LINGTU_CMU_GATE_MIN_TARE_STRATEGY_PATHS:-1}" \
      --require-frontier-no-gain-stall \
      --json-out "$run_dir/report.json" \
      --strict
    )
    if [[ "${LINGTU_CMU_GATE_ALLOW_FLAT_LATE_MAP_AFTER_TOTAL_GROWTH:-1}" == "1" ]]; then
      gate_args+=(--allow-flat-late-map-after-total-growth)
    fi
    local required_map_topic required_map_topics
    required_map_topics="${LINGTU_CMU_GATE_REQUIRED_MAP_TOPICS:-/nav/map_cloud /nav/terrain_map_ext}"
    required_map_topics="${required_map_topics//,/ }"
    for required_map_topic in $required_map_topics; do
      gate_args+=(--required-map-topic "$required_map_topic")
    done
    if [[ "${LINGTU_CMU_GATE_REQUIRE_PCT:-0}" == "1" ]]; then
      gate_args+=(
        --require-planner-diagnostics
        --require-no-planner-fallback
        --require-planner-path-safety
        --require-same-source-tomogram
      )
    fi
    if [[ "${LINGTU_CMU_GATE_REQUIRE_NO_PRIMARY_REPLAN:-0}" == "1" ]]; then
      gate_args+=(--require-no-primary-replan)
    fi
    /usr/bin/python3 "$root/sim/scripts/cmu_unity_runtime_gate.py" "${gate_args[@]}"
  fi
}

run_gate_only() {
  local root run_dir
  root="$(repo_root)"
  run_dir="${LINGTU_CMU_RUN_DIR:-$root/artifacts/server_sim_closure/cmu_unity_runtime_latest}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-75}"
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  export PYTHONPATH="$root/src:$root:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}"
  local gate_args=(
    --publish-start \
    --gateway-start-exploration-session \
    --timeout-sec "${LINGTU_CMU_GATE_TIMEOUT_SEC:-180}" \
    --require-motion-progress \
    --required-progress-topic /nav/way_point \
    --min-motion-progress-m 0.25 \
    --min-unique-waypoints "${LINGTU_CMU_GATE_MIN_UNIQUE_WAYPOINTS:-2}" \
    --unique-waypoint-topic /nav/way_point \
    --require-path-topic /nav/global_path \
    --require-path-topic /nav/local_path \
    --late-window-sec "${LINGTU_CMU_GATE_LATE_WINDOW_SEC:-60}" \
    --min-late-odom-delta-m "${LINGTU_CMU_GATE_MIN_LATE_ODOM_DELTA_M:-0.5}" \
    --min-late-cmd-vel-samples "${LINGTU_CMU_GATE_MIN_LATE_CMD_VEL_SAMPLES:-10}" \
    --min-late-path-samples "${LINGTU_CMU_GATE_MIN_LATE_PATH_SAMPLES:-5}" \
    --min-late-map-area-delta-m2 "${LINGTU_CMU_GATE_MIN_LATE_MAP_AREA_DELTA_M2:-1.0}" \
    --require-tare-strategy-quality \
    --min-tare-strategy-paths "${LINGTU_CMU_GATE_MIN_TARE_STRATEGY_PATHS:-1}" \
    --require-frontier-no-gain-stall \
    --json-out "$run_dir/report.json" \
    --strict
  )
  if [[ "${LINGTU_CMU_GATE_ALLOW_FLAT_LATE_MAP_AFTER_TOTAL_GROWTH:-1}" == "1" ]]; then
    gate_args+=(--allow-flat-late-map-after-total-growth)
  fi
  local required_map_topic required_map_topics
  required_map_topics="${LINGTU_CMU_GATE_REQUIRED_MAP_TOPICS:-/nav/map_cloud /nav/terrain_map_ext}"
  required_map_topics="${required_map_topics//,/ }"
  for required_map_topic in $required_map_topics; do
    gate_args+=(--required-map-topic "$required_map_topic")
  done
  if [[ "${LINGTU_CMU_GATE_REQUIRE_PCT:-0}" == "1" ]]; then
    gate_args+=(
      --require-planner-diagnostics
      --require-no-planner-fallback
      --require-planner-path-safety
      --require-same-source-tomogram
    )
  fi
  if [[ "${LINGTU_CMU_GATE_REQUIRE_NO_PRIMARY_REPLAN:-0}" == "1" ]]; then
    gate_args+=(--require-no-primary-replan)
  fi
  /usr/bin/python3 "$root/sim/scripts/cmu_unity_runtime_gate.py" "${gate_args[@]}"
}

cmd="${1:-start}"
shift || true
case "$cmd" in
  start)
    gate=0
    rviz=0
    for arg in "$@"; do
      case "$arg" in
        --gate) gate=1 ;;
        --rviz) rviz=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $arg" >&2; usage; exit 2 ;;
      esac
    done
    start_runtime "$gate" "$rviz"
    ;;
  gate)
    run_gate_only
    ;;
  rviz)
    launch_operator_rviz
    ;;
  status)
    status_runtime
    ;;
  stop)
    stop_runtime
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown command: $cmd" >&2
    usage
    exit 2
    ;;
esac
