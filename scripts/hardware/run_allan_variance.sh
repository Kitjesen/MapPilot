#!/usr/bin/env bash
# run_allan_variance.sh — one-shot Allan Variance calibration for Livox Mid-360 IMU.
#
# Background:
#   The lio.yaml / pointlio.yaml IMU noise params (na/ng/nba/nbg) ship at
#   conservative defaults (na=0.01, ng=0.01). Mid-360's built-in IMU is
#   the TDK InvenSense ICM-40609-D, whose typical gyro noise is
#   ~6.6e-5 rad/s/√Hz — i.e. the default ng is ~150x too high, causing
#   IEKF to under-trust the gyro and accumulate covariance during static
#   periods (one of the reported divergence modes). This script automates
#   the standard Allan Variance procedure to land sensor-specific values.
#
# Phases (run individually or via `all`):
#   record   ros2 bag record /livox/imu (default 3h, IEEE 952-1997 minimum)
#   analyze  ros2 run allan_variance_ros2 + scripts/analysis.py → imu.yaml
#   apply    tools/calibration/apply_calibration.py → lio.yaml + pointlio.yaml
#
# Usage:
#   bash scripts/hardware/run_allan_variance.sh all              # 3h end-to-end
#   bash scripts/hardware/run_allan_variance.sh record           # just record
#   bash scripts/hardware/run_allan_variance.sh analyze <path>   # analyze existing bag
#   bash scripts/hardware/run_allan_variance.sh apply <imu.yaml> # apply existing yaml
#
# Env overrides:
#   LINGTU_AV_DURATION=10800   # bag duration (s), default 3h
#   LINGTU_AV_OUT=~/data/imu_calib  # output base dir
#   LINGTU_AV_TS=20260426_120000    # timestamp for resume
#   LINGTU_AV_TOPIC=/livox/imu      # source topic
#
# Pre-flight:
#   - Robot stationary on a vibration-isolated surface (foam/rubber pad)
#   - Steady ambient temperature (no AC vents, no direct sunlight)
#   - All other ROS2 nodes shut down (avoid CPU thermal noise)

set -euo pipefail

find_repo_root() {
  local dir="$1"
  while [[ "$dir" != "/" ]]; do
    if [[ -f "$dir/pyproject.toml" && -f "$dir/AGENTS.md" ]]; then
      printf '%s\n' "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(find_repo_root "$SCRIPT_DIR")"

DURATION_SEC="${LINGTU_AV_DURATION:-10800}"
OUT_BASE="${LINGTU_AV_OUT:-$HOME/data/imu_calib}"
TS="${LINGTU_AV_TS:-$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="$OUT_BASE/$TS"
TOPIC="${LINGTU_AV_TOPIC:-/livox/imu}"
CONFIG_YAML="$REPO_ROOT/tools/calibration/imu/config/livox_mid360_imu.yaml"
ANALYSIS_PY="$REPO_ROOT/tools/calibration/imu/allan_variance_ros2/src/allan_variance_ros2/scripts/analysis.py"

log()   { printf '\033[1;36m[av]\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m[av WARN]\033[0m %s\n' "$*" >&2; }
fatal() { printf '\033[1;31m[av FATAL]\033[0m %s\n' "$*" >&2; exit 1; }

require_ros2() {
  command -v ros2 >/dev/null || fatal "ros2 not on PATH (source /opt/ros/humble/setup.bash?)"
}

phase_record() {
  require_ros2
  mkdir -p "$OUT_DIR"
  local bag_dir="$OUT_DIR/imu_static_bag"
  if [[ -d "$bag_dir" ]]; then
    warn "bag dir $bag_dir already exists — skipping record (use LINGTU_AV_TS=new to force)"
    echo "$bag_dir"
    return 0
  fi
  log "Recording $TOPIC for ${DURATION_SEC}s → $bag_dir"
  log "Robot MUST stay stationary the whole time. Press Ctrl-C only to abort."
  log "(Estimated finish: $(date -d "+${DURATION_SEC} seconds" '+%Y-%m-%d %H:%M:%S'))"
  ros2 bag record "$TOPIC" -o "$bag_dir" --max-bag-duration "$DURATION_SEC" \
       --duration "$DURATION_SEC"
  log "Bag recorded: $bag_dir"
  echo "$bag_dir"
}

phase_analyze() {
  require_ros2
  local bag_dir="${1:-$OUT_DIR/imu_static_bag}"
  [[ -d "$bag_dir" ]] || fatal "bag dir not found: $bag_dir"
  mkdir -p "$OUT_DIR"
  log "Running allan_variance on $bag_dir"
  pushd "$OUT_DIR" >/dev/null
  ros2 run allan_variance_ros2 allan_variance "$bag_dir" "$CONFIG_YAML"
  [[ -f allan_variance.csv ]] || fatal "allan_variance.csv not produced"
  log "Fitting noise model → imu.yaml"
  python3 "$ANALYSIS_PY" --data allan_variance.csv --config "$CONFIG_YAML"
  popd >/dev/null
  local out_yaml="$OUT_DIR/imu.yaml"
  [[ -f "$out_yaml" ]] || fatal "imu.yaml not produced"
  log "Analysis done: $out_yaml"
  echo "$out_yaml"
}

phase_apply() {
  local imu_yaml="${1:-$OUT_DIR/imu.yaml}"
  [[ -f "$imu_yaml" ]] || fatal "imu yaml not found: $imu_yaml"
  log "Applying $imu_yaml → lio.yaml + pointlio.yaml (apply_calibration.py runs sanity checks)"
  python3 "$REPO_ROOT/tools/calibration/apply_calibration.py" --imu "$imu_yaml"
  log "Apply complete. Reminder: re-run python -m pytest src/runtime/tests/test_calibration_check.py"
}

main() {
  local cmd="${1:-all}"; shift || true
  case "$cmd" in
    record)  phase_record ;;
    analyze) phase_analyze "${1:-}" ;;
    apply)   phase_apply   "${1:-}" ;;
    all)
      log "=== Phase 1/3: record ==="
      local bag; bag=$(phase_record)
      log "=== Phase 2/3: analyze ==="
      local yaml; yaml=$(phase_analyze "$bag")
      log "=== Phase 3/3: apply ==="
      phase_apply "$yaml"
      log "All phases complete. Output: $OUT_DIR"
      ;;
    -h|--help|help)
      sed -n '2,30p' "$0"
      ;;
    *) fatal "Unknown command: $cmd (try: record / analyze / apply / all)" ;;
  esac
}

main "$@"
