#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  sim/scripts/mujoco/launch_fastlio2_live.sh gate
  sim/scripts/mujoco/launch_fastlio2_live.sh explore
  sim/scripts/mujoco/launch_fastlio2_live.sh tare
  sim/scripts/mujoco/launch_fastlio2_live.sh tare-video
  sim/scripts/mujoco/launch_fastlio2_live.sh tare-moving-obstacle
  sim/scripts/mujoco/launch_fastlio2_live.sh tare-moving-obstacle-video
  sim/scripts/mujoco/launch_fastlio2_live.sh inspection
  sim/scripts/mujoco/launch_fastlio2_live.sh inspection-video
  sim/scripts/mujoco/launch_fastlio2_live.sh inspection-loop-video
  sim/scripts/mujoco/launch_fastlio2_live.sh inspection-moving-obstacle
  sim/scripts/mujoco/launch_fastlio2_live.sh inspection-moving-obstacle-video
  sim/scripts/mujoco/launch_fastlio2_live.sh demo
  sim/scripts/mujoco/launch_fastlio2_live.sh video
  sim/scripts/mujoco/launch_fastlio2_live.sh pct-moving-obstacle
  sim/scripts/mujoco/launch_fastlio2_live.sh pct-moving-obstacle-video
  sim/scripts/mujoco/launch_fastlio2_live.sh native-dds-sensors
  sim/scripts/mujoco/launch_fastlio2_live.sh native-dds-gate
  sim/scripts/mujoco/launch_fastlio2_live.sh status
  sim/scripts/mujoco/launch_fastlio2_live.sh stop

LingTu profile contract: sim_mujoco_live / mujoco_fastlio2_live.

Commands:
  gate     Raw MID-360 + IMU live gate. Disabled until a real localization backend is wired.
  explore  Same source path, with LingTu frontier/navigation driving /nav/cmd_vel.
  tare     Same source path, with native TARE selecting exploration goals.
  tare-video
           TARE live exploration gate plus MP4 evidence output.
  tare-moving-obstacle
           TARE live exploration gate with moving obstacle returns injected into raw LiDAR.
  tare-moving-obstacle-video
           Same live TARE dynamic-obstacle gate with MP4 evidence output enabled.
  inspection
           Fast-LIO live mapping plus LingTu Navigation patrol checkpoints.
  inspection-video
           Inspection patrol gate plus MP4 evidence output.
  inspection-loop-video
           Large loop patrol with PCT planning, live Fast-LIO output, local paths,
           cmd_vel closure, and MP4 evidence.
  inspection-moving-obstacle
           Inspection patrol with route-crossing moving obstacle returns injected into raw LiDAR.
  inspection-moving-obstacle-video
           Inspection patrol with route-crossing moving obstacle returns and MP4 evidence.
  demo     Long-running explore plus RViz display for map/cloud/path/motion review.
  video    Explore gate plus MP4 evidence output.
  pct-moving-obstacle
           Saved large_terrain tomogram -> native PCT -> ROS2 local planner/path follower
           -> MuJoCo motion with a route-crossing moving obstacle.
  pct-moving-obstacle-video
           Same gate with MP4 evidence output enabled.
  native-dds-sensors
           ROS-free MuJoCo raw MID-360/IMU publisher for native C++ DDS SLAM.
  native-dds-gate
           Same publisher, but fails unless C++ SLAM publishes odometry/map/health.

Environment overrides:
  LINGTU_MUJOCO_LIVE_WORLD=industrial_park
  LINGTU_MUJOCO_LIVE_DURATION_GATE=30
  LINGTU_MUJOCO_LIVE_DURATION_EXPLORE=90
  LINGTU_MUJOCO_LIVE_DURATION_TARE=180
  LINGTU_MUJOCO_LIVE_DURATION_INSPECTION=120
  LINGTU_MUJOCO_LIVE_MAX_WALL_TIME_S=0
  LINGTU_MUJOCO_LIVE_DURATION_DEMO=300
  LINGTU_MUJOCO_LIVE_DURATION_CLOCK=wall   # wall or sim; explore/video default to sim
  LINGTU_MUJOCO_LIVE_DRIVE_VX=0.25
  LINGTU_MUJOCO_LIVE_DRIVE_VY=0.0
  LINGTU_MUJOCO_LIVE_DRIVE_WZ=0.0
  LINGTU_MUJOCO_LIVE_FRONTIER_GOAL_TIMEOUT=240
  LINGTU_MUJOCO_LIVE_TARE_MIN_GOALS=2
  LINGTU_MUJOCO_LIVE_TARE_GOAL_TIMEOUT=180
  LINGTU_MUJOCO_LIVE_TARE_SCENARIO=indoor
  LINGTU_MUJOCO_LIVE_INSPECTION_GOALS="0.8,0.0;1.6,0.2;2.4,0.4"
  LINGTU_MUJOCO_LIVE_INSPECTION_MIN_CHECKPOINTS=3
  LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_TIMEOUT=900  # sim-clock default; override for faster local runs
  LINGTU_MUJOCO_LIVE_INSPECTION_DOWNSAMPLE_DIST=0.35
  LINGTU_MUJOCO_LIVE_INSPECTION_WAYPOINT_THRESHOLD=0.50
  LINGTU_MUJOCO_LIVE_INSPECTION_FINAL_WAYPOINT_THRESHOLD=0.50
  LINGTU_MUJOCO_LIVE_INSPECTION_PATH_GOAL_TOLERANCE=0.12
  LINGTU_MUJOCO_LIVE_INSPECTION_PATH_LOOKAHEAD=1.5
  LINGTU_MUJOCO_LIVE_INSPECTION_PATH_MIN_SPEED=0.15
  LINGTU_MUJOCO_LIVE_CMD_VEL_SIM_LINEAR_SCALE=1.0
  LINGTU_MUJOCO_LIVE_CMD_VEL_SIM_ANGULAR_SCALE=1.0
  LINGTU_MUJOCO_LIVE_CMD_VEL_MUX_SOURCE_TIMEOUT=5.0
  LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_LIMIT=0.25
  LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_LIMIT=0.45
  LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_ACCEL_LIMIT=0.5
  LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_ACCEL_LIMIT=1.0
  LINGTU_MUJOCO_LIVE_NAV_MAX_LINEAR_SPEED=0.25
  LINGTU_MUJOCO_LIVE_NAV_MAX_ANGULAR_Z=0.45
  LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_YAW_RATE_START=0.0
  LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_MIN_SCALE=1.0
  LINGTU_MUJOCO_LIVE_MID360_SAMPLES_PER_FRAME=15000
  LINGTU_MUJOCO_LIVE_LOCALIZATION_BACKEND=  # no default; portable_lio was removed
  LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_FILTER_NUM=4
  LINGTU_MUJOCO_LIVE_FASTLIO_SCAN_RESOLUTION=0.15
  LINGTU_MUJOCO_LIVE_FASTLIO_MAP_RESOLUTION=0.3
  LINGTU_MUJOCO_LIVE_FASTLIO_NEAR_SEARCH_NUM=5
  LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER=5
  LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_COV_INV=1000
  LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT=livox_custom_msg
  LINGTU_MUJOCO_LIVE_FASTLIO_TIME_DIFF_LIDAR_TO_IMU=0.0
  LINGTU_MUJOCO_LIVE_FASTLIO_VERTICAL_VELOCITY_CONSTRAINT=off
  LINGTU_MUJOCO_LIVE_MAX_ANGULAR_SATURATION_RATIO=0.35
  LINGTU_MUJOCO_LIVE_IMU_ACC_MODE=finite_difference
  LINGTU_MUJOCO_LIVE_SCAN_TIME_PROFILE=physical_rolling
  LINGTU_MUJOCO_LIVE_MAX_FASTLIO_Z_DRIFT_M=1.0
  LINGTU_MUJOCO_LIVE_MAX_FASTLIO_YAW_DRIFT_RAD=0.5
  LINGTU_MUJOCO_LIVE_RUNTIME_FAULT_CONFIRM_SAMPLES=2
  LINGTU_MUJOCO_LIVE_SAVE_MAP_ARTIFACTS=1
  LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM=0
  LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_MODE=robot_crossing
  LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_COUNT=2
  LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_PERIOD_S=8
  LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_LATERAL_AMPLITUDE_M=0.85
  LINGTU_MUJOCO_LIVE_RVIZ=tests/planning/mujoco_fastlio2_live.rviz
  LINGTU_MUJOCO_LIVE_RUN_DIR=artifacts/server_sim_closure/mujoco_fastlio2_live
  LINGTU_MUJOCO_LIVE_ROS_DOMAIN_ID=83
  LINGTU_MUJOCO_LIVE_PCT_SOURCE_REPORT=artifacts/server_sim_closure/large_terrain/report.large_local.json
  LINGTU_MUJOCO_LIVE_PCT_ROUTE=terrain_long
  LINGTU_MUJOCO_NATIVE_DDS_DURATION=10
  LINGTU_MUJOCO_NATIVE_DDS_DOMAIN_ID=83
  LINGTU_MUJOCO_NATIVE_DDS_PUBLISH_HZ=10
  LINGTU_MUJOCO_NATIVE_DDS_IMU_HZ=200
  LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_MODE=sensor
  LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_CONDITIONING=realistic
  LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_LOWPASS_HZ=30
  LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_MAX_DYNAMIC_MPS2=6
  LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_MAX_SLEW_MPS3=400
  LINGTU_MUJOCO_NATIVE_DDS_DRIVE_MODE=policy
  LINGTU_MUJOCO_NATIVE_DDS_MIN_LOCALIZATION_QUALITY=0.5
  LINGTU_MUJOCO_NATIVE_DDS_MAX_ODOM_ABS_M=100
  LINGTU_MUJOCO_NATIVE_DDS_MAX_ODOM_Z_ABS_M=20
  LINGTU_MUJOCO_NATIVE_DDS_POLICY_PATH=
  LINGTU_MUJOCO_NATIVE_DDS_PUBLISHER_BIN=build/livox_sdk2_stream/livox_sdk2_stream
  LINGTU_MUJOCO_NATIVE_DDS_SLAM_STATUS_JSON=artifacts/mujoco_native_dds_slam/status.json
EOF
}

repo_root() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  cd "$script_dir/../../.." && pwd
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
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD="${PYTEST_DISABLE_PLUGIN_AUTOLOAD:-1}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${LINGTU_MUJOCO_LIVE_ROS_DOMAIN_ID:-83}}"
  if [[ ! "$ROS_DOMAIN_ID" =~ ^[0-9]+$ || "$ROS_DOMAIN_ID" == "0" || "$ROS_DOMAIN_ID" -ge 232 ]]; then
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID is unsafe for this demo; use 1..231." >&2
    exit 2
  fi
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  export LINGTU_PROFILE="${LINGTU_PROFILE:-sim_mujoco_live}"
  export LINGTU_RUNTIME_CONTRACT="${LINGTU_RUNTIME_CONTRACT:-mujoco_fastlio2_live}"
  # Keep the live gate on the same native-PCT raw-path mode as large_terrain
  # unless the caller explicitly opts into the GPMP optimizer.
  export LINGTU_PCT_OPTIMIZE_TRAJECTORY="${LINGTU_PCT_OPTIMIZE_TRAJECTORY:-0}"
}

prepare_native_dds_env() {
  local root="$1"
  export PYTHONPATH="$root/src:$root:${PYTHONPATH:-}"
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD="${PYTEST_DISABLE_PLUGIN_AUTOLOAD:-1}"
  export LINGTU_PROFILE="${LINGTU_PROFILE:-sim_mujoco_live}"
  export LINGTU_RUNTIME_CONTRACT="${LINGTU_RUNTIME_CONTRACT:-mujoco_fastlio2_live}"
}

cleanup_stale_fastlio2() {
  local root="$1"
  local run_root="$2"
  if [[ "${LINGTU_MUJOCO_LIVE_CLEAN_STALE_FASTLIO:-1}" == "0" ]]; then
    return 0
  fi
  local patterns=(
    "$root/install/lib/fastlio2/lio_node.*mujoco_fastlio2_live"
    "ros2 run fastlio2 lio_node.*mujoco_fastlio2_live"
    "$root/install/lib/fastlio2/lio_node.*$run_root"
    "ros2 run fastlio2 lio_node.*$run_root"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -f "$pattern" 2>/dev/null || true
  done
}

detect_display_env() {
  if [[ -n "${LINGTU_MUJOCO_LIVE_DISPLAY:-}" ]]; then
    export DISPLAY="$LINGTU_MUJOCO_LIVE_DISPLAY"
  elif [[ -z "${DISPLAY:-}" ]]; then
    local detected=""
    detected="$(who | awk '{for (i=1; i<=NF; i++) if ($i ~ /^:[0-9]+$/) {print $i; exit}}' 2>/dev/null || true)"
    if [[ -z "$detected" && -S /tmp/.X11-unix/X1 ]]; then
      detected=":1"
    fi
    if [[ -z "$detected" && -d /tmp/.X11-unix ]]; then
      local sock=""
      sock="$(find /tmp/.X11-unix -maxdepth 1 -type s -name 'X*' -printf '%f\n' 2>/dev/null | sort -V | head -n 1 || true)"
      if [[ -n "$sock" ]]; then
        detected=":${sock#X}"
      fi
    fi
    export DISPLAY="${detected:-:0}"
  fi

  if [[ -n "${LINGTU_MUJOCO_LIVE_XAUTHORITY:-}" ]]; then
    export XAUTHORITY="$LINGTU_MUJOCO_LIVE_XAUTHORITY"
  elif [[ -z "${XAUTHORITY:-}" ]]; then
    local uid
    uid="$(id -u)"
    if [[ -r "/run/user/$uid/gdm/Xauthority" ]]; then
      export XAUTHORITY="/run/user/$uid/gdm/Xauthority"
    elif [[ -r "$HOME/.Xauthority" ]]; then
      export XAUTHORITY="$HOME/.Xauthority"
    fi
  fi

  export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
}

run_gate() {
  local root="$1"
  local mode="$2"
  prepare_env "$root"

  local run_root="${LINGTU_MUJOCO_LIVE_RUN_DIR:-$root/artifacts/server_sim_closure/mujoco_fastlio2_live}"
  cleanup_stale_fastlio2 "$root" "$run_root"
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  local run_dir="$run_root/$mode-$stamp"
  mkdir -p "$run_dir"

  local world="${LINGTU_MUJOCO_LIVE_WORLD:-industrial_park}"
  local python_bin="${LINGTU_PYTHON:-/usr/bin/python3}"
  [[ -x "$python_bin" ]] || python_bin="python3"

  local duration="${LINGTU_MUJOCO_LIVE_DURATION_GATE:-30}"
  local duration_clock="${LINGTU_MUJOCO_LIVE_DURATION_CLOCK:-wall}"
  local drive_source="fixed"
  local nav_data_source="${LINGTU_MUJOCO_LIVE_NAV_DATA_SOURCE:-fastlio2}"
  local strict="${LINGTU_MUJOCO_LIVE_STRICT:-1}"
  local video_out=""
  local show_mujoco_window="${LINGTU_MUJOCO_LIVE_SHOW_MUJOCO:-0}"
  local moving_obstacle_mode="${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_MODE:-none}"
  local moving_obstacle_default_count="1"
  local moving_obstacle_default_period="10"
  local moving_obstacle_default_start="8"
  local frontier_args=()

  case "$mode" in
    gate)
      ;;
    explore)
      duration="${LINGTU_MUJOCO_LIVE_DURATION_EXPLORE:-90}"
      duration_clock="${LINGTU_MUJOCO_LIVE_DURATION_CLOCK:-sim}"
      drive_source="nav_cmd_vel"
      frontier_args=(
        "--run-lingtu-frontier"
        "--frontier-min-goals" "${LINGTU_MUJOCO_LIVE_FRONTIER_MIN_GOALS:-3}"
        "--frontier-goal-timeout" "${LINGTU_MUJOCO_LIVE_FRONTIER_GOAL_TIMEOUT:-120}"
      )
      ;;
    tare|tare-video|tare-moving-obstacle|tare-moving-obstacle-video)
      duration="${LINGTU_MUJOCO_LIVE_DURATION_TARE:-180}"
      duration_clock="${LINGTU_MUJOCO_LIVE_DURATION_CLOCK:-sim}"
      drive_source="nav_cmd_vel"
      frontier_args=(
        "--run-lingtu-tare"
        "--tare-min-goals" "${LINGTU_MUJOCO_LIVE_TARE_MIN_GOALS:-2}"
        "--tare-start-delay" "${LINGTU_MUJOCO_LIVE_TARE_START_DELAY:-0}"
        "--tare-goal-timeout" "${LINGTU_MUJOCO_LIVE_TARE_GOAL_TIMEOUT:-180}"
        "--tare-scenario" "${LINGTU_MUJOCO_LIVE_TARE_SCENARIO:-indoor}"
      )
      if [[ "$mode" == "tare-video" || "$mode" == "tare-moving-obstacle-video" ]]; then
        video_out="$run_dir/mujoco_fastlio2_tare.mp4"
      fi
      if [[ "$mode" == "tare-moving-obstacle" || "$mode" == "tare-moving-obstacle-video" ]]; then
        moving_obstacle_mode="${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_MODE:-robot_crossing}"
        moving_obstacle_default_count="2"
        moving_obstacle_default_period="8"
        moving_obstacle_default_start="12"
      fi
      ;;
    inspection|inspection-video|inspection-loop|inspection-loop-video|inspection-moving-obstacle|inspection-moving-obstacle-video)
      duration="${LINGTU_MUJOCO_LIVE_DURATION_INSPECTION:-120}"
      duration_clock="${LINGTU_MUJOCO_LIVE_DURATION_CLOCK:-sim}"
      drive_source="nav_cmd_vel"
      local inspection_default_goals="${LINGTU_MUJOCO_LIVE_INSPECTION_GOALS:-0.8,0.0;1.6,0.2;2.4,0.4}"
      local inspection_default_min_checkpoints="${LINGTU_MUJOCO_LIVE_INSPECTION_MIN_CHECKPOINTS:-3}"
      local inspection_default_planner="${LINGTU_MUJOCO_LIVE_INSPECTION_PLANNER:-astar}"
      local inspection_default_goal_timeout="${LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_TIMEOUT:-900}"
      if [[ "$duration_clock" != "sim" && -z "${LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_TIMEOUT:-}" ]]; then
        inspection_default_goal_timeout="90"
      fi
      if [[ "$mode" == "inspection-loop" || "$mode" == "inspection-loop-video" ]]; then
        duration="${LINGTU_MUJOCO_LIVE_DURATION_INSPECTION:-240}"
        # Large-loop goals are in the live planning frame. In Fast-LIO mode
        # that frame is odom, whose world-frame origin is recorded separately
        # in the runtime report and checked by the aggregation gate.
        inspection_default_goals="${LINGTU_MUJOCO_LIVE_INSPECTION_GOALS:-4.8,0.0;4.8,5.7;0.0,5.7;0.0,0.0}"
        inspection_default_min_checkpoints="${LINGTU_MUJOCO_LIVE_INSPECTION_MIN_CHECKPOINTS:-4}"
        inspection_default_planner="${LINGTU_MUJOCO_LIVE_INSPECTION_PLANNER:-pct}"
      fi
      frontier_args=(
        "--run-lingtu-inspection"
        "--inspection-goals=$inspection_default_goals"
        "--inspection-min-checkpoints" "$inspection_default_min_checkpoints"
        "--inspection-start-delay" "${LINGTU_MUJOCO_LIVE_INSPECTION_START_DELAY:-0}"
        "--inspection-goal-timeout" "$inspection_default_goal_timeout"
        "--inspection-planner" "$inspection_default_planner"
      )
      if [[ -n "${LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM:-}" ]]; then
        frontier_args+=("--inspection-tomogram" "$LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM")
      fi
      if [[ "$mode" == "inspection-video" || "$mode" == "inspection-loop-video" || "$mode" == "inspection-moving-obstacle-video" ]]; then
        video_out="$run_dir/mujoco_fastlio2_inspection.mp4"
      fi
      if [[ "$mode" == "inspection-moving-obstacle" || "$mode" == "inspection-moving-obstacle-video" ]]; then
        moving_obstacle_mode="${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_MODE:-robot_crossing}"
        moving_obstacle_default_count="3"
        moving_obstacle_default_period="6"
        moving_obstacle_default_start="2"
      fi
      ;;
    demo)
      duration="${LINGTU_MUJOCO_LIVE_DURATION_DEMO:-300}"
      drive_source="nav_cmd_vel"
      nav_data_source="${LINGTU_MUJOCO_LIVE_NAV_DATA_SOURCE:-mujoco_ground_truth}"
      strict="${LINGTU_MUJOCO_LIVE_DEMO_STRICT:-0}"
      show_mujoco_window="${LINGTU_MUJOCO_LIVE_SHOW_MUJOCO:-1}"
      frontier_args=(
        "--run-lingtu-frontier"
        "--frontier-min-goals" "${LINGTU_MUJOCO_LIVE_FRONTIER_MIN_GOALS:-1}"
        "--frontier-goal-timeout" "${LINGTU_MUJOCO_LIVE_FRONTIER_GOAL_TIMEOUT:-240}"
      )
      ;;
    video)
      duration="${LINGTU_MUJOCO_LIVE_DURATION_VIDEO:-${LINGTU_MUJOCO_LIVE_DURATION_EXPLORE:-90}}"
      duration_clock="${LINGTU_MUJOCO_LIVE_DURATION_CLOCK:-sim}"
      drive_source="nav_cmd_vel"
      frontier_args=(
        "--run-lingtu-frontier"
        "--frontier-min-goals" "${LINGTU_MUJOCO_LIVE_FRONTIER_MIN_GOALS:-3}"
        "--frontier-goal-timeout" "${LINGTU_MUJOCO_LIVE_FRONTIER_GOAL_TIMEOUT:-120}"
      )
      video_out="$run_dir/mujoco_fastlio2_live.mp4"
      ;;
    *)
      echo "Unknown run mode: $mode" >&2
      exit 2
      ;;
  esac

  echo "run_dir=$run_dir" | tee "$run_dir/status.txt"
  echo "LINGTU_PROFILE=$LINGTU_PROFILE" | tee -a "$run_dir/status.txt"
  echo "LINGTU_RUNTIME_CONTRACT=$LINGTU_RUNTIME_CONTRACT" | tee -a "$run_dir/status.txt"
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" | tee -a "$run_dir/status.txt"
  echo "world=$world duration=$duration duration_clock=$duration_clock drive_source=$drive_source nav_data_source=$nav_data_source strict=$strict" | tee -a "$run_dir/status.txt"
  echo "moving_obstacle_mode=$moving_obstacle_mode count=${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_COUNT:-$moving_obstacle_default_count} start=${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_START_S:-$moving_obstacle_default_start} period=${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_PERIOD_S:-$moving_obstacle_default_period}" | tee -a "$run_dir/status.txt"

  local cmd_vel_timeout_default="0.75"
  local cmd_vel_mux_source_timeout_default="0.5"
  local cmd_vel_linear_limit_default="0.25"
  local cmd_vel_angular_limit_default="0.45"
  local nav_max_linear_speed_default="0.25"
  local nav_max_angular_z_default="0.45"
  local nav_turn_speed_yaw_rate_start_default="0.0"
  local nav_turn_speed_min_scale_default="1.0"
  local runtime_fault_confirm_samples_default="2"
  local runtime_motion_fault_min_sim_m_default="0.25"
  local fastlio_ieskf_max_iter_default="5"
  local max_angular_saturation_ratio_default="0.35"
  local inspection_downsample_dist_default="0.35"
  local inspection_waypoint_threshold_default="0.50"
  local inspection_final_waypoint_threshold_default="0.50"
  local inspection_complete_path_on_goal_proximity_default="0"
  local inspection_goal_proximity_completion_threshold_default=""
  local inspection_path_goal_tolerance_default="0.12"
  local inspection_path_lookahead_default="1.5"
  local inspection_path_min_speed_default="0.15"
  local inspection_path_yaw_rate_gain_default="7.5"
  local inspection_path_stop_yaw_rate_gain_default="7.5"
  local inspection_path_dir_diff_thre_default="0.1"
  if [[ "$drive_source" == "nav_cmd_vel" && "$duration_clock" == "sim" ]]; then
    cmd_vel_timeout_default="0"
    cmd_vel_mux_source_timeout_default="5.0"
  fi
  if [[ "$mode" == "inspection-loop" || "$mode" == "inspection-loop-video" ]]; then
    inspection_downsample_dist_default="1.0"
    # Keep checkpoint completion close enough that the next PCT start remains
    # on the validated path, while avoiding long low-speed spin near corners.
    cmd_vel_linear_limit_default="0.45"
    cmd_vel_angular_limit_default="0.25"
    nav_max_linear_speed_default="0.45"
    nav_max_angular_z_default="0.25"
    inspection_waypoint_threshold_default="0.65"
    inspection_final_waypoint_threshold_default="0.65"
    inspection_complete_path_on_goal_proximity_default="1"
    inspection_goal_proximity_completion_threshold_default="0.65"
    inspection_path_lookahead_default="2.0"
    inspection_path_min_speed_default="0.15"
    inspection_path_yaw_rate_gain_default="2.5"
    inspection_path_stop_yaw_rate_gain_default="2.5"
    inspection_path_dir_diff_thre_default="1.8"
    nav_turn_speed_yaw_rate_start_default="0.0"
    nav_turn_speed_min_scale_default="1.0"
  fi
  if [[ "$mode" == inspection-moving-obstacle* ]]; then
    cmd_vel_angular_limit_default="0.25"
    nav_max_angular_z_default="0.20"
    runtime_fault_confirm_samples_default="6"
    runtime_motion_fault_min_sim_m_default="1.0"
    fastlio_ieskf_max_iter_default="10"
    max_angular_saturation_ratio_default="0.40"
  fi
  echo "cmd_vel_timeout=${LINGTU_MUJOCO_LIVE_CMD_VEL_TIMEOUT:-$cmd_vel_timeout_default} cmd_vel_mux_source_timeout=${LINGTU_MUJOCO_LIVE_CMD_VEL_MUX_SOURCE_TIMEOUT:-$cmd_vel_mux_source_timeout_default}" | tee -a "$run_dir/status.txt"
  echo "cmd_vel_linear_limit=${LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_LIMIT:-$cmd_vel_linear_limit_default} nav_max_linear_speed=${LINGTU_MUJOCO_LIVE_NAV_MAX_LINEAR_SPEED:-$nav_max_linear_speed_default} cmd_vel_angular_limit=${LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_LIMIT:-$cmd_vel_angular_limit_default} nav_max_angular_z=${LINGTU_MUJOCO_LIVE_NAV_MAX_ANGULAR_Z:-$nav_max_angular_z_default} nav_turn_speed_yaw_rate_start=${LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_YAW_RATE_START:-$nav_turn_speed_yaw_rate_start_default} nav_turn_speed_min_scale=${LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_MIN_SCALE:-$nav_turn_speed_min_scale_default} runtime_fault_confirm_samples=${LINGTU_MUJOCO_LIVE_RUNTIME_FAULT_CONFIRM_SAMPLES:-$runtime_fault_confirm_samples_default} runtime_motion_fault_min_sim_m=${LINGTU_MUJOCO_LIVE_RUNTIME_MOTION_FAULT_MIN_SIM_M:-$runtime_motion_fault_min_sim_m_default} fastlio_ieskf_max_iter=${LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER:-$fastlio_ieskf_max_iter_default} max_angular_saturation_ratio=${LINGTU_MUJOCO_LIVE_MAX_ANGULAR_SATURATION_RATIO:-$max_angular_saturation_ratio_default}" | tee -a "$run_dir/status.txt"
  echo "inspection_downsample_dist=${LINGTU_MUJOCO_LIVE_INSPECTION_DOWNSAMPLE_DIST:-$inspection_downsample_dist_default} inspection_waypoint_threshold=${LINGTU_MUJOCO_LIVE_INSPECTION_WAYPOINT_THRESHOLD:-$inspection_waypoint_threshold_default} inspection_final_waypoint_threshold=${LINGTU_MUJOCO_LIVE_INSPECTION_FINAL_WAYPOINT_THRESHOLD:-$inspection_final_waypoint_threshold_default} inspection_complete_path_on_goal_proximity=${LINGTU_MUJOCO_LIVE_INSPECTION_COMPLETE_PATH_ON_GOAL_PROXIMITY:-$inspection_complete_path_on_goal_proximity_default} inspection_goal_proximity_completion_threshold=${LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_PROXIMITY_COMPLETION_THRESHOLD:-$inspection_goal_proximity_completion_threshold_default} inspection_path_lookahead=${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_LOOKAHEAD:-$inspection_path_lookahead_default} inspection_path_yaw_rate_gain=${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_YAW_RATE_GAIN:-$inspection_path_yaw_rate_gain_default} inspection_path_dir_diff_thre=${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_DIR_DIFF_THRE:-$inspection_path_dir_diff_thre_default}" | tee -a "$run_dir/status.txt"

  local cmd=(
    "$python_bin" "$root/sim/scripts/mujoco/live_gate.py"
    "--world" "$world"
    "--duration" "$duration"
    "--duration-clock" "$duration_clock"
    "--max-wall-time-s" "${LINGTU_MUJOCO_LIVE_MAX_WALL_TIME_S:-0}"
    "--drive-source" "$drive_source"
    "--drive-vx" "${LINGTU_MUJOCO_LIVE_DRIVE_VX:-0.25}"
    "--drive-vy" "${LINGTU_MUJOCO_LIVE_DRIVE_VY:-0.0}"
    "--drive-wz" "${LINGTU_MUJOCO_LIVE_DRIVE_WZ:-0.0}"
    "--nav-data-source" "$nav_data_source"
    "--work-dir" "$run_dir/work"
    "--json-out" "$run_dir/report.json"
    "--partial-json-out" "$run_dir/report.partial.json"
  )
  cmd+=(
    "--cmd-vel-timeout" "${LINGTU_MUJOCO_LIVE_CMD_VEL_TIMEOUT:-$cmd_vel_timeout_default}"
    "--cmd-vel-mux-source-timeout" "${LINGTU_MUJOCO_LIVE_CMD_VEL_MUX_SOURCE_TIMEOUT:-$cmd_vel_mux_source_timeout_default}"
    "--cmd-vel-linear-limit" "${LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_LIMIT:-$cmd_vel_linear_limit_default}"
    "--cmd-vel-angular-limit" "${LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_LIMIT:-$cmd_vel_angular_limit_default}"
    "--cmd-vel-sim-linear-scale" "${LINGTU_MUJOCO_LIVE_CMD_VEL_SIM_LINEAR_SCALE:-1.0}"
    "--cmd-vel-sim-angular-scale" "${LINGTU_MUJOCO_LIVE_CMD_VEL_SIM_ANGULAR_SCALE:-1.0}"
    "--cmd-vel-linear-accel-limit" "${LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_ACCEL_LIMIT:-0.5}"
    "--cmd-vel-angular-accel-limit" "${LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_ACCEL_LIMIT:-1.0}"
    "--nav-max-linear-speed" "${LINGTU_MUJOCO_LIVE_NAV_MAX_LINEAR_SPEED:-$nav_max_linear_speed_default}"
    "--nav-max-angular-z" "${LINGTU_MUJOCO_LIVE_NAV_MAX_ANGULAR_Z:-$nav_max_angular_z_default}"
    "--nav-turn-speed-yaw-rate-start" "${LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_YAW_RATE_START:-$nav_turn_speed_yaw_rate_start_default}"
    "--nav-turn-speed-min-scale" "${LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_MIN_SCALE:-$nav_turn_speed_min_scale_default}"
    "--inspection-downsample-dist" "${LINGTU_MUJOCO_LIVE_INSPECTION_DOWNSAMPLE_DIST:-$inspection_downsample_dist_default}"
    "--inspection-waypoint-threshold" "${LINGTU_MUJOCO_LIVE_INSPECTION_WAYPOINT_THRESHOLD:-$inspection_waypoint_threshold_default}"
    "--inspection-final-waypoint-threshold" "${LINGTU_MUJOCO_LIVE_INSPECTION_FINAL_WAYPOINT_THRESHOLD:-$inspection_final_waypoint_threshold_default}"
    "--inspection-path-goal-tolerance" "${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_GOAL_TOLERANCE:-$inspection_path_goal_tolerance_default}"
    "--inspection-path-lookahead" "${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_LOOKAHEAD:-$inspection_path_lookahead_default}"
    "--inspection-path-min-speed" "${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_MIN_SPEED:-$inspection_path_min_speed_default}"
    "--inspection-path-yaw-rate-gain" "${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_YAW_RATE_GAIN:-$inspection_path_yaw_rate_gain_default}"
    "--inspection-path-stop-yaw-rate-gain" "${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_STOP_YAW_RATE_GAIN:-$inspection_path_stop_yaw_rate_gain_default}"
    "--inspection-path-dir-diff-thre" "${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_DIR_DIFF_THRE:-$inspection_path_dir_diff_thre_default}"
    "--mid360-samples-per-frame" "${LINGTU_MUJOCO_LIVE_MID360_SAMPLES_PER_FRAME:-15000}"
    "--localization-backend" "${LINGTU_MUJOCO_LIVE_LOCALIZATION_BACKEND:-}"
    "--fastlio-lidar-input" "${LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT:-livox_custom_msg}"
    "--fastlio-lidar-filter-num" "${LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_FILTER_NUM:-4}"
    "--fastlio-scan-resolution" "${LINGTU_MUJOCO_LIVE_FASTLIO_SCAN_RESOLUTION:-0.15}"
    "--fastlio-map-resolution" "${LINGTU_MUJOCO_LIVE_FASTLIO_MAP_RESOLUTION:-0.3}"
    "--fastlio-near-search-num" "${LINGTU_MUJOCO_LIVE_FASTLIO_NEAR_SEARCH_NUM:-5}"
    "--fastlio-ieskf-max-iter" "${LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER:-$fastlio_ieskf_max_iter_default}"
    "--fastlio-lidar-cov-inv" "${LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_COV_INV:-1000}"
    "--fastlio-time-diff-lidar-to-imu" "${LINGTU_MUJOCO_LIVE_FASTLIO_TIME_DIFF_LIDAR_TO_IMU:-0.0}"
    "--fastlio-vertical-velocity-constraint" "${LINGTU_MUJOCO_LIVE_FASTLIO_VERTICAL_VELOCITY_CONSTRAINT:-off}"
    "--scan-time-profile" "${LINGTU_MUJOCO_LIVE_SCAN_TIME_PROFILE:-physical_rolling}"
    "--imu-acc-mode" "${LINGTU_MUJOCO_LIVE_IMU_ACC_MODE:-finite_difference}"
    "--max-fastlio-z-drift-m" "${LINGTU_MUJOCO_LIVE_MAX_FASTLIO_Z_DRIFT_M:-1.0}"
    "--max-fastlio-yaw-drift-rad" "${LINGTU_MUJOCO_LIVE_MAX_FASTLIO_YAW_DRIFT_RAD:-0.5}"
    "--runtime-fault-confirm-samples" "${LINGTU_MUJOCO_LIVE_RUNTIME_FAULT_CONFIRM_SAMPLES:-$runtime_fault_confirm_samples_default}"
    "--runtime-motion-fault-min-sim-m" "${LINGTU_MUJOCO_LIVE_RUNTIME_MOTION_FAULT_MIN_SIM_M:-$runtime_motion_fault_min_sim_m_default}"
    "--max-angular-saturation-ratio" "${LINGTU_MUJOCO_LIVE_MAX_ANGULAR_SATURATION_RATIO:-$max_angular_saturation_ratio_default}"
    "--map-artifact-voxel-size" "${LINGTU_MUJOCO_LIVE_MAP_ARTIFACT_VOXEL_SIZE:-0.10}"
    "--map-artifact-max-points" "${LINGTU_MUJOCO_LIVE_MAP_ARTIFACT_MAX_POINTS:-250000}"
    "--map-artifact-max-span-m" "${LINGTU_MUJOCO_LIVE_MAP_ARTIFACT_MAX_SPAN_M:-120.0}"
    "--tomogram-resolution" "${LINGTU_MUJOCO_LIVE_TOMOGRAM_RESOLUTION:-0.20}"
    "--tomogram-slice-dh" "${LINGTU_MUJOCO_LIVE_TOMOGRAM_SLICE_DH:-0.25}"
    "--tomogram-ground-h" "${LINGTU_MUJOCO_LIVE_TOMOGRAM_GROUND_H:-0.0}"
    "--tomogram-max-cells" "${LINGTU_MUJOCO_LIVE_TOMOGRAM_MAX_CELLS:-50000000}"
  )
  case "${LINGTU_MUJOCO_LIVE_INSPECTION_COMPLETE_PATH_ON_GOAL_PROXIMITY:-$inspection_complete_path_on_goal_proximity_default}" in
    1|true|TRUE|yes|YES|on|ON)
      cmd+=("--inspection-complete-path-on-goal-proximity")
      ;;
  esac
  if [[ -n "${LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_PROXIMITY_COMPLETION_THRESHOLD:-$inspection_goal_proximity_completion_threshold_default}" ]]; then
    cmd+=(
      "--inspection-goal-proximity-completion-threshold"
      "${LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_PROXIMITY_COMPLETION_THRESHOLD:-$inspection_goal_proximity_completion_threshold_default}"
    )
  fi
  if [[ "${LINGTU_MUJOCO_LIVE_SAVE_MAP_ARTIFACTS:-1}" == "0" || "${LINGTU_MUJOCO_LIVE_SAVE_MAP_ARTIFACTS:-1}" == "false" ]]; then
    cmd+=("--no-save-map-artifacts")
  fi
  if [[ "${LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM:-0}" == "1" || "${LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM:-0}" == "true" || "${LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM:-0}" == "yes" ]]; then
    cmd+=("--build-tomogram")
  fi
  if [[ -n "${LINGTU_MUJOCO_LIVE_START:-}" ]]; then
    cmd+=("--start=$LINGTU_MUJOCO_LIVE_START")
  fi
  if [[ "$strict" == "1" || "$strict" == "true" || "$strict" == "yes" ]]; then
    cmd+=("--strict")
  fi
  if [[ "${#frontier_args[@]}" -gt 0 ]]; then
    cmd+=("${frontier_args[@]}")
  fi
  if [[ "$moving_obstacle_mode" != "none" ]]; then
    cmd+=(
      "--moving-obstacle-mode" "$moving_obstacle_mode"
      "--moving-obstacle-count" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_COUNT:-$moving_obstacle_default_count}"
      "--moving-obstacle-start-s" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_START_S:-$moving_obstacle_default_start}"
      "--moving-obstacle-duration-s" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_DURATION_S:-0}"
      "--moving-obstacle-period-s" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_PERIOD_S:-$moving_obstacle_default_period}"
      "--moving-obstacle-forward-m" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_FORWARD_M:-2.0}"
      "--moving-obstacle-forward-step-m" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_FORWARD_STEP_M:-0.8}"
      "--moving-obstacle-lateral-phase-step-rad" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_LATERAL_PHASE_STEP_RAD:-1.57079632679}"
      "--moving-obstacle-lateral-amplitude-m" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_LATERAL_AMPLITUDE_M:-0.85}"
      "--moving-obstacle-along-amplitude-m" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_ALONG_AMPLITUDE_M:-0.25}"
      "--moving-obstacle-radius-m" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_RADIUS_M:-0.16}"
      "--moving-obstacle-height-m" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_HEIGHT_M:-0.60}"
      "--moving-obstacle-point-spacing" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_POINT_SPACING:-0.10}"
      "--moving-obstacle-intensity" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_INTENSITY:-220.0}"
      "--moving-obstacle-robot-radius-m" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_ROBOT_RADIUS_M:-0.28}"
    )
  fi
  if [[ "$show_mujoco_window" == "1" || "$show_mujoco_window" == "true" || "$show_mujoco_window" == "yes" ]]; then
    cmd+=("--show-mujoco-window" "--mujoco-window-fps" "${LINGTU_MUJOCO_LIVE_MUJOCO_WINDOW_FPS:-10}")
  fi
  if [[ -n "$video_out" ]]; then
    cmd+=("--video-out" "$video_out" "--video-fps" "${LINGTU_MUJOCO_LIVE_VIDEO_FPS:-8}")
  fi

  printf '%q ' "${cmd[@]}" >"$run_dir/command.txt"
  echo >>"$run_dir/command.txt"
  echo "latest_run_dir=$run_dir" >"$run_root/latest.txt"
  set +e
  "${cmd[@]}" 2>&1 | tee "$run_dir/gate.log"
  local rc=${PIPESTATUS[0]}
  set -e
  if [[ ! -f "$run_dir/report.json" ]]; then
    "$python_bin" - "$run_dir" "$rc" "$mode" "$world" "$nav_data_source" <<'PY'
import json
import sys
from pathlib import Path

run_dir = Path(sys.argv[1])
rc = int(sys.argv[2])
mode = sys.argv[3]
world = sys.argv[4]
nav_data_source = sys.argv[5]
partial_path = run_dir / "report.partial.json"
partial = {}
partial_error = ""
if partial_path.exists():
    try:
        raw_partial = json.loads(partial_path.read_text(encoding="utf-8"))
        if isinstance(raw_partial, dict):
            partial = raw_partial
    except Exception as exc:
        partial_error = f"{type(exc).__name__}: {exc}"
partial_faults = [
    str(item)
    for item in partial.get("runtime_faults", [])
    if isinstance(item, (str, int, float)) and str(item)
]
runtime_faults = list(
    dict.fromkeys([*partial_faults, "runtime_report_missing_after_launcher"])
)
report = {
    "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
    "ok": False,
    "simulation_only": True,
    "real_robot_motion": False,
    "cmd_vel_sent_to_hardware": False,
    "world": world,
    "nav_data_source": nav_data_source,
    "remaining_gaps": ["runtime_report_missing_after_launcher"],
    "runtime_faults": runtime_faults,
    "launcher_failure": {
        "ok": False,
        "mode": mode,
        "returncode": rc,
        "reason": "mujoco_live_gate exited without writing report.json",
        "run_dir": str(run_dir),
        "command": str(run_dir / "command.txt"),
        "gate_log": str(run_dir / "gate.log"),
        "status": str(run_dir / "status.txt"),
        "partial_report": str(partial_path) if partial_path.exists() else "",
        "partial_report_error": partial_error,
    },
    "partial_report": partial,
    "partial_report_path": str(partial_path) if partial_path.exists() else "",
    "outputs": partial.get("outputs", {}) if isinstance(partial.get("outputs"), dict) else {},
    "lingtu_inspection": (
        partial.get("lingtu_inspection", {})
        if isinstance(partial.get("lingtu_inspection"), dict)
        else {}
    ),
    "simulation_path": (
        partial.get("simulation_path", {})
        if isinstance(partial.get("simulation_path"), dict)
        else {}
    ),
    "video": partial.get("video", {}) if isinstance(partial.get("video"), dict) else {},
    "gate_wall_timeout": (
        partial.get("gate_wall_timeout", {})
        if isinstance(partial.get("gate_wall_timeout"), dict)
        else {}
    ),
    "runtime_fault_events": (
        partial.get("runtime_fault_events", [])
        if isinstance(partial.get("runtime_fault_events"), list)
        else []
    ),
}
(run_dir / "report.json").write_text(
    json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
PY
  fi
  cleanup_stale_fastlio2 "$root" "$run_root"
  return "$rc"
}

run_pct_moving_obstacle() {
  local root="$1"
  local visual_mode="${2:-}"
  prepare_env "$root"

  local run_root="${LINGTU_MUJOCO_LIVE_RUN_DIR:-$root/artifacts/server_sim_closure/mujoco_fastlio2_live}"
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  local run_dir="$run_root/pct-moving-obstacle-$stamp"
  mkdir -p "$run_dir"

  local python_bin="${LINGTU_PYTHON:-/usr/bin/python3}"
  [[ -x "$python_bin" ]] || python_bin="python3"

  local source_report="${LINGTU_MUJOCO_LIVE_PCT_SOURCE_REPORT:-artifacts/server_sim_closure/large_terrain/report.large_local.json}"
  if [[ "$source_report" != /* && ! -f "$source_report" && -f "$root/$source_report" ]]; then
    source_report="$root/$source_report"
  fi
  if [[ ! -f "$source_report" ]]; then
    echo "PCT source report not found: $source_report" >&2
    echo "Expected a large_terrain report with map.pcd + tomogram.pickle generated first." >&2
    exit 2
  fi

  local report_out="$run_dir/report.json"
  local closure_out="$run_dir/closure_summary.json"
  local strict="${LINGTU_MUJOCO_LIVE_PCT_STRICT:-1}"
  local run_closure="${LINGTU_MUJOCO_LIVE_PCT_CLOSURE:-1}"
  local default_moving_count="1"
  local default_moving_period="14"
  local default_pct_timeout="360"
  local default_video_fps="12"
  if [[ "$visual_mode" == "video" ]]; then
    default_moving_count="2"
    default_moving_period="10"
    default_pct_timeout="600"
    default_video_fps="4"
  fi
  local video_requested="${LINGTU_MUJOCO_LIVE_PCT_VIDEO:-0}"
  if [[ "$visual_mode" == "video" ]]; then
    video_requested="1"
  fi
  local video_out=""
  if [[ "$video_requested" == "1" || "$video_requested" == "true" || "$video_requested" == "yes" ]]; then
    if [[ -n "${LINGTU_MUJOCO_LIVE_PCT_VIDEO_OUT:-}" ]]; then
      video_out="$LINGTU_MUJOCO_LIVE_PCT_VIDEO_OUT"
      if [[ "$video_out" != /* ]]; then
        video_out="$run_dir/$video_out"
      fi
    else
      video_out="$run_dir/pct_moving_obstacle.mp4"
    fi
    mkdir -p "$(dirname "$video_out")"
  fi

  echo "run_dir=$run_dir" | tee "$run_dir/status.txt"
  echo "LINGTU_PROFILE=$LINGTU_PROFILE" | tee -a "$run_dir/status.txt"
  echo "LINGTU_RUNTIME_CONTRACT=$LINGTU_RUNTIME_CONTRACT" | tee -a "$run_dir/status.txt"
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" | tee -a "$run_dir/status.txt"
  echo "source_report=$source_report" | tee -a "$run_dir/status.txt"
  echo "route=${LINGTU_MUJOCO_LIVE_PCT_ROUTE:-terrain_long} strict=$strict" | tee -a "$run_dir/status.txt"
  echo "moving_obstacle_count=${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_COUNT:-$default_moving_count} period=${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_PERIOD_S:-$default_moving_period}" | tee -a "$run_dir/status.txt"
  if [[ -n "$video_out" ]]; then
    echo "video_out=$video_out layout=${LINGTU_MUJOCO_LIVE_PCT_VIDEO_LAYOUT:-evidence}" | tee -a "$run_dir/status.txt"
  fi

  local cmd=(
    "$python_bin" "$root/sim/scripts/mujoco/native_pct_gate.py"
    "--source-report" "$source_report"
    "--route" "${LINGTU_MUJOCO_LIVE_PCT_ROUTE:-terrain_long}"
    "--planner" "${LINGTU_MUJOCO_LIVE_PCT_PLANNER:-pct}"
    "--timeout-s" "${LINGTU_MUJOCO_LIVE_PCT_TIMEOUT:-$default_pct_timeout}"
    "--moving-obstacle-mode" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_MODE:-route_crossing}"
    "--moving-obstacle-route-ratio" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_ROUTE_RATIO:-0.5}"
    "--moving-obstacle-count" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_COUNT:-$default_moving_count}"
    "--moving-obstacle-route-ratio-step" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_ROUTE_RATIO_STEP:-0.08}"
    "--moving-obstacle-lateral-phase-step-rad" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_LATERAL_PHASE_STEP_RAD:-1.57079632679}"
    "--moving-obstacle-start-s" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_START_S:-70}"
    "--moving-obstacle-duration-s" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_DURATION_S:-72}"
    "--moving-obstacle-period-s" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_PERIOD_S:-$default_moving_period}"
    "--moving-obstacle-lateral-amplitude-m" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_LATERAL_AMPLITUDE_M:-0.85}"
    "--moving-obstacle-along-amplitude-m" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_ALONG_AMPLITUDE_M:-0.25}"
    "--moving-obstacle-radius-m" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_RADIUS_M:-0.16}"
    "--moving-obstacle-height-m" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_HEIGHT_M:-0.60}"
    "--moving-obstacle-intensity" "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_INTENSITY:-220.0}"
    "--json-out" "$report_out"
  )
  if [[ -n "$video_out" ]]; then
    cmd+=(
      "--video-out" "$video_out"
      "--video-layout" "${LINGTU_MUJOCO_LIVE_PCT_VIDEO_LAYOUT:-evidence}"
      "--video-fps" "${LINGTU_MUJOCO_LIVE_PCT_VIDEO_FPS:-$default_video_fps}"
      "--video-width" "${LINGTU_MUJOCO_LIVE_PCT_VIDEO_WIDTH:-1280}"
      "--video-height" "${LINGTU_MUJOCO_LIVE_PCT_VIDEO_HEIGHT:-720}"
    )
  fi
  if [[ "$strict" == "1" || "$strict" == "true" || "$strict" == "yes" ]]; then
    cmd+=("--strict")
  fi

  printf '%q ' "${cmd[@]}" >"$run_dir/command.txt"
  echo >>"$run_dir/command.txt"
  echo "latest_run_dir=$run_dir" >"$run_root/latest.txt"
  "${cmd[@]}" 2>&1 | tee "$run_dir/gate.log"

  if [[ "$run_closure" == "1" || "$run_closure" == "true" || "$run_closure" == "yes" ]]; then
    local closure_cmd=(
      "$python_bin" "$root/sim/scripts/server_sim_closure.py"
      "--required" "native_pct_mujoco"
      "--required-only"
      "--native-pct-mujoco-report" "$report_out"
      "--json-out" "$closure_out"
    )
    printf '%q ' "${closure_cmd[@]}" >"$run_dir/closure_command.txt"
    echo >>"$run_dir/closure_command.txt"
    "${closure_cmd[@]}" 2>&1 | tee "$run_dir/closure.log"
  fi
}

run_native_dds_sensors() {
  local root="$1"
  local mode="${2:-sensors}"
  prepare_native_dds_env "$root"

  local run_root="${LINGTU_MUJOCO_NATIVE_DDS_RUN_DIR:-$root/artifacts/mujoco_native_dds_sensors}"
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  local run_dir="$run_root/$mode-$stamp"
  mkdir -p "$run_dir"

  local python_bin="${LINGTU_PYTHON:-/usr/bin/python3}"
  [[ -x "$python_bin" ]] || python_bin="python3"

  local world="${LINGTU_MUJOCO_NATIVE_DDS_WORLD:-${LINGTU_MUJOCO_LIVE_WORLD:-industrial_park}}"
  local domain_id="${LINGTU_MUJOCO_NATIVE_DDS_DOMAIN_ID:-${LINGTU_DDS_DOMAIN_ID:-83}}"
  local duration="${LINGTU_MUJOCO_NATIVE_DDS_DURATION:-10}"
  local publisher_bin="${LINGTU_MUJOCO_NATIVE_DDS_PUBLISHER_BIN:-$root/build/livox_sdk2_stream/livox_sdk2_stream}"
  local slam_status_json="${LINGTU_MUJOCO_NATIVE_DDS_SLAM_STATUS_JSON:-${LINGTU_SLAM_STATUS_JSON:-$root/artifacts/mujoco_native_dds_slam/status.json}}"
  local publish_odom_prior="${LINGTU_MUJOCO_NATIVE_DDS_PUBLISH_ODOM_PRIOR:-0}"
  local require_slam="${LINGTU_MUJOCO_NATIVE_DDS_REQUIRE_SLAM_OUTPUT:-0}"
  if [[ "$mode" == "gate" ]]; then
    require_slam="1"
  fi

  local cmd=(
    "$python_bin" "$root/sim/scripts/mujoco/native_dds_sensors.py"
    "--world" "$world"
    "--duration" "$duration"
    "--publish-hz" "${LINGTU_MUJOCO_NATIVE_DDS_PUBLISH_HZ:-10}"
    "--imu-hz" "${LINGTU_MUJOCO_NATIVE_DDS_IMU_HZ:-200}"
    "--imu-acc-mode" "${LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_MODE:-sensor}"
    "--imu-acc-conditioning" "${LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_CONDITIONING:-realistic}"
    "--imu-acc-lowpass-hz" "${LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_LOWPASS_HZ:-30}"
    "--imu-acc-max-dynamic-mps2" "${LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_MAX_DYNAMIC_MPS2:-6}"
    "--imu-acc-max-slew-mps3" "${LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_MAX_SLEW_MPS3:-400}"
    "--imu-acc-axis-scale" "${LINGTU_MUJOCO_NATIVE_DDS_IMU_ACC_AXIS_SCALE:-auto}"
    "--imu-gyro-axis-scale" "${LINGTU_MUJOCO_NATIVE_DDS_IMU_GYRO_AXIS_SCALE:-1,1,1}"
    "--settle-s" "${LINGTU_MUJOCO_NATIVE_DDS_SETTLE_S:-3.0}"
    "--warmup-s" "${LINGTU_MUJOCO_NATIVE_DDS_WARMUP_S:-2.0}"
    "--drive-ramp-s" "${LINGTU_MUJOCO_NATIVE_DDS_DRIVE_RAMP_S:-5.0}"
    "--scan-time-profile" "${LINGTU_MUJOCO_NATIVE_DDS_SCAN_TIME_PROFILE:-physical_rolling}"
    "--physical-rolling-sample-mode" "${LINGTU_MUJOCO_NATIVE_DDS_PHYSICAL_ROLLING_SAMPLE_MODE:-subscan}"
    "--timestamp-clock" "${LINGTU_MUJOCO_NATIVE_DDS_TIMESTAMP_CLOCK:-sim_hardware}"
    "--sim-hardware-realtime-factor" "${LINGTU_MUJOCO_NATIVE_DDS_SIM_HARDWARE_REALTIME_FACTOR:-1.0}"
    "--imu-timestamp-clock" "${LINGTU_MUJOCO_NATIVE_DDS_IMU_TIMESTAMP_CLOCK:-}"
    "--lidar-timestamp-clock" "${LINGTU_MUJOCO_NATIVE_DDS_LIDAR_TIMESTAMP_CLOCK:-}"
    "--no-publish-odom-prior"
    "--min-localization-quality" "${LINGTU_MUJOCO_NATIVE_DDS_MIN_LOCALIZATION_QUALITY:-0.5}"
    "--max-odom-abs-m" "${LINGTU_MUJOCO_NATIVE_DDS_MAX_ODOM_ABS_M:-100}"
    "--max-odom-z-abs-m" "${LINGTU_MUJOCO_NATIVE_DDS_MAX_ODOM_Z_ABS_M:-20}"
    "--min-sim-motion-for-odom-check-m" "${LINGTU_MUJOCO_NATIVE_DDS_MIN_SIM_MOTION_FOR_ODOM_CHECK_M:-0.5}"
    "--min-slam-motion-ratio" "${LINGTU_MUJOCO_NATIVE_DDS_MIN_SLAM_MOTION_RATIO:-0.5}"
    "--max-slam-motion-ratio" "${LINGTU_MUJOCO_NATIVE_DDS_MAX_SLAM_MOTION_RATIO:-1.6}"
    "--min-sim-yaw-for-odom-check-rad" "${LINGTU_MUJOCO_NATIVE_DDS_MIN_SIM_YAW_FOR_ODOM_CHECK_RAD:-0.2}"
    "--max-slam-yaw-error-rad" "${LINGTU_MUJOCO_NATIVE_DDS_MAX_SLAM_YAW_ERROR_RAD:-0.15}"
    "--domain-id" "$domain_id"
    "--publisher-bin" "$publisher_bin"
    "--slam-status-json" "$slam_status_json"
    "--drive-mode" "${LINGTU_MUJOCO_NATIVE_DDS_DRIVE_MODE:-policy}"
    "--drive-profile" "${LINGTU_MUJOCO_NATIVE_DDS_DRIVE_PROFILE:-arc}"
    "--drive-vx" "${LINGTU_MUJOCO_NATIVE_DDS_DRIVE_VX:-0.10}"
    "--drive-vy" "${LINGTU_MUJOCO_NATIVE_DDS_DRIVE_VY:-0.0}"
    "--drive-wz" "${LINGTU_MUJOCO_NATIVE_DDS_DRIVE_WZ:-0.04}"
    "--n-rays" "${LINGTU_MUJOCO_NATIVE_DDS_N_RAYS:-6400}"
    "--mid360-samples-per-frame" "${LINGTU_MUJOCO_NATIVE_DDS_MID360_SAMPLES_PER_FRAME:-15000}"
    "--max-points" "${LINGTU_MUJOCO_NATIVE_DDS_MAX_POINTS:-15000}"
    "--json-out" "$run_dir/report.json"
  )
  if [[ "$require_slam" == "1" || "$require_slam" == "true" || "$require_slam" == "yes" ]]; then
    cmd+=("--require-slam-output")
  fi
  local allow_kinematic_fastlio_acceptance="${LINGTU_MUJOCO_NATIVE_DDS_ALLOW_KINEMATIC_FASTLIO_ACCEPTANCE:-0}"
  if [[ "$allow_kinematic_fastlio_acceptance" == "1" || "$allow_kinematic_fastlio_acceptance" == "true" || "$allow_kinematic_fastlio_acceptance" == "yes" ]]; then
    cmd+=("--allow-kinematic-fastlio-acceptance")
  fi
  if [[ "$publish_odom_prior" == "1" || "$publish_odom_prior" == "true" || "$publish_odom_prior" == "yes" ]]; then
    cmd=("${cmd[@]/--no-publish-odom-prior/--publish-odom-prior}")
  fi
  if [[ -n "${LINGTU_MUJOCO_NATIVE_DDS_START:-}" ]]; then
    cmd+=("--start" "$LINGTU_MUJOCO_NATIVE_DDS_START")
  fi
  if [[ -n "${LINGTU_MUJOCO_NATIVE_DDS_PATTERN:-}" ]]; then
    cmd+=("--mid360-pattern" "$LINGTU_MUJOCO_NATIVE_DDS_PATTERN")
  fi
  if [[ -n "${LINGTU_MUJOCO_NATIVE_DDS_POLICY_PATH:-}" ]]; then
    cmd+=("--policy-path" "$LINGTU_MUJOCO_NATIVE_DDS_POLICY_PATH")
  fi

  echo "run_dir=$run_dir" | tee "$run_dir/status.txt"
  echo "LINGTU_PROFILE=$LINGTU_PROFILE" | tee -a "$run_dir/status.txt"
  echo "LINGTU_RUNTIME_CONTRACT=$LINGTU_RUNTIME_CONTRACT" | tee -a "$run_dir/status.txt"
  echo "world=$world domain_id=$domain_id duration=$duration require_slam_output=$require_slam" | tee -a "$run_dir/status.txt"
  echo "publisher_bin=$publisher_bin" | tee -a "$run_dir/status.txt"
  echo "slam_status_json=$slam_status_json" | tee -a "$run_dir/status.txt"
  printf '%q ' "${cmd[@]}" >"$run_dir/command.txt"
  echo >>"$run_dir/command.txt"
  echo "latest_run_dir=$run_dir" >"$run_root/latest.txt"
  "${cmd[@]}" 2>&1 | tee "$run_dir/native_dds_sensors.log"
}

wait_for_topics() {
  local timeout_s="$1"
  shift
  local deadline=$((SECONDS + timeout_s))
  local missing=("$@")

  while [[ "$SECONDS" -lt "$deadline" ]]; do
    local topics
    topics="$(ros2 topic list 2>/dev/null || true)"
    missing=()
    for topic in "$@"; do
      if ! grep -Fxq "$topic" <<<"$topics"; then
        missing+=("$topic")
      fi
    done
    if [[ "${#missing[@]}" -eq 0 ]]; then
      return 0
    fi
    sleep 1
  done

  echo "Topics not visible after ${timeout_s}s: ${missing[*]}" >&2
  return 1
}

run_demo() {
  local root="$1"
  prepare_env "$root"
  detect_display_env

  export LINGTU_MUJOCO_LIVE_DURATION_EXPLORE="${LINGTU_MUJOCO_LIVE_DURATION_DEMO:-${LINGTU_MUJOCO_LIVE_DURATION_EXPLORE:-300}}"

  local run_root="${LINGTU_MUJOCO_LIVE_RUN_DIR:-$root/artifacts/server_sim_closure/mujoco_fastlio2_live}"
  mkdir -p "$run_root"
  local rviz_config="${LINGTU_MUJOCO_LIVE_RVIZ:-$root/tests/planning/mujoco_fastlio2_live.rviz}"
  if [[ "$rviz_config" != /* ]]; then
    rviz_config="$root/$rviz_config"
  fi
  local rviz_log="$run_root/demo_rviz.log"

  echo "Starting LingTu MuJoCo Fast-LIO live demo."
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "DISPLAY=${DISPLAY:-}"
  echo "XAUTHORITY=${XAUTHORITY:-}"
  echo "RViz config=$rviz_config"
  echo "Duration=${LINGTU_MUJOCO_LIVE_DURATION_EXPLORE}s"

  if command -v rviz2 >/dev/null 2>&1; then
    rviz2 -d "$rviz_config" >"$rviz_log" 2>&1 &
    local rviz_pid=$!
    echo "rviz_pid=$rviz_pid"
    echo "$rviz_pid" >"$run_root/demo_rviz.pid"
    sleep 2
    if ! kill -0 "$rviz_pid" >/dev/null 2>&1; then
      echo "RViz exited early; see $rviz_log" >&2
    fi
  else
    echo "rviz2 is not installed or not on PATH; live ROS topics are still running." >&2
  fi

  run_gate "$root" "demo" &
  local gate_pid=$!
  echo "gate_pid=$gate_pid"

  wait_for_topics 45 \
    /lidar/raw_frame \
    /imu/raw \
    /cloud_registered \
    /cloud_map \
    /Odometry \
    /slam/odometry \
    /slam/registered_cloud \
    /slam/map_cloud \
    /nav/cmd_vel \
    /nav/exploration_grid \
    /nav/global_path \
    /nav/local_path || true

  wait "$gate_pid"
}

status_demo() {
  local root="$1"
  local run_root="${LINGTU_MUJOCO_LIVE_RUN_DIR:-$root/artifacts/server_sim_closure/mujoco_fastlio2_live}"
  echo "run_root=$run_root"
  if [[ -f "$run_root/latest.txt" ]]; then
    cat "$run_root/latest.txt"
  fi
  pgrep -af "mujoco_live_gate.py" || true
  pgrep -af "mujoco_native_dds_sensors.py" || true
}

stop_demo() {
  pkill -f "mujoco_live_gate.py" >/dev/null 2>&1 || true
  pkill -f "mujoco_native_dds_sensors.py" >/dev/null 2>&1 || true
  pkill -f "fastlio.*mujoco_fastlio2_live" >/dev/null 2>&1 || true
  pkill -f "mujoco_fastlio2_live.rviz" >/dev/null 2>&1 || true
  echo "Stopped LingTu MuJoCo Fast-LIO live gate processes."
}

main() {
  local root
  root="$(repo_root)"
  local cmd="${1:-}"
  case "$cmd" in
    gate) run_gate "$root" "gate" ;;
    explore) run_gate "$root" "explore" ;;
    tare) run_gate "$root" "tare" ;;
    tare-video) run_gate "$root" "tare-video" ;;
    tare-moving-obstacle) run_gate "$root" "tare-moving-obstacle" ;;
    tare-moving-obstacle-video) run_gate "$root" "tare-moving-obstacle-video" ;;
    inspection) run_gate "$root" "inspection" ;;
    inspection-video) run_gate "$root" "inspection-video" ;;
    inspection-loop) run_gate "$root" "inspection-loop" ;;
    inspection-loop-video) run_gate "$root" "inspection-loop-video" ;;
    inspection-moving-obstacle) run_gate "$root" "inspection-moving-obstacle" ;;
    inspection-moving-obstacle-video) run_gate "$root" "inspection-moving-obstacle-video" ;;
    demo) run_demo "$root" ;;
    video) run_gate "$root" "video" ;;
    pct-moving-obstacle|moving-obstacle) run_pct_moving_obstacle "$root" ;;
    pct-moving-obstacle-video|moving-obstacle-video) run_pct_moving_obstacle "$root" "video" ;;
    native-dds-sensors) run_native_dds_sensors "$root" "sensors" ;;
    native-dds-gate) run_native_dds_sensors "$root" "gate" ;;
    status) status_demo "$root" ;;
    stop) stop_demo ;;
    -h|--help|"") usage ;;
    *) echo "Unknown command: $cmd" >&2; usage; exit 2 ;;
  esac
}

main "$@"
