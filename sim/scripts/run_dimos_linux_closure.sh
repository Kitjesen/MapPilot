#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash sim/scripts/run_dimos_linux_closure.sh [--dry-run]
  bash sim/scripts/run_dimos_linux_closure.sh --execute

Runs the DimOS benchmark closure sequence on a prepared Linux ROS2/PCT/MuJoCo
simulation host:
  1. set isolated simulation environment
  2. source ROS 2 Humble and the local install space
  3. run read-only host preflight
  4. run strict missing-gate closure only if preflight is green
  5. regenerate the DimOS gap report with runtime dataflow

Default mode is --dry-run. Use --execute only on the isolated simulation host.

Options:
  --execute                         Run commands instead of printing them.
  --dry-run                         Print commands without running them.
  --ros-domain-id ID                Isolated ROS domain. Default: current ROS_DOMAIN_ID or 75.
  --host-preflight-out PATH         Host preflight JSON output path.
  --summary-out PATH                server_sim_closure summary JSON output path.
  --gap-out PATH                    DimOS gap report JSON output path.
  -h, --help                        Show this help.
EOF
}

repo_root() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  cd "$script_dir/../.." && pwd
}

EXECUTE=0
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-75}"
HOST_PREFLIGHT_OUT="artifacts/server_sim_closure/host_preflight_dimos_benchmark.json"
SUMMARY_OUT="artifacts/server_sim_closure/summary_dimos_benchmark_24h.json"
GAP_OUT="artifacts/server_sim_closure/dimos_gap_report_dimos_benchmark_24h.json"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      EXECUTE=1
      ;;
    --dry-run)
      EXECUTE=0
      ;;
    --ros-domain-id)
      shift
      ROS_DOMAIN_ID_VALUE="${1:?missing --ros-domain-id value}"
      ;;
    --host-preflight-out)
      shift
      HOST_PREFLIGHT_OUT="${1:?missing --host-preflight-out value}"
      ;;
    --summary-out)
      shift
      SUMMARY_OUT="${1:?missing --summary-out value}"
      ;;
    --gap-out)
      shift
      GAP_OUT="${1:?missing --gap-out value}"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

ROOT="$(repo_root)"
cd "$ROOT"

print_shell_line() {
  local command="$1"
  printf '+ %s\n' "$command"
}

print_command() {
  local arg
  printf '+'
  for arg in "$@"; do
    printf ' %q' "$arg"
  done
  printf '\n'
}

run_preflight() {
  local preflight_target_out="$1"
  shift
  local preflight_tmp_out="${preflight_target_out}.tmp.$$"
  local -a command=("$@" "--json-out" "$preflight_tmp_out")
  print_command "${command[@]}"
  if [[ "$EXECUTE" != "1" ]]; then
    return
  fi

  mkdir -p "$(dirname "$preflight_target_out")"
  rm -f "$preflight_tmp_out"
  set +e
  "${command[@]}"
  local status=$?
  set -e
  if [[ "$status" -ne 0 ]]; then
    rm -f "$preflight_tmp_out"
    echo "Host preflight failed; refusing to reuse stale ${preflight_target_out}." >&2
    exit "$status"
  fi
  if [[ ! -f "$preflight_tmp_out" ]]; then
    echo "Host preflight did not write ${preflight_tmp_out}." >&2
    exit 3
  fi
  mv "$preflight_tmp_out" "$preflight_target_out"
}

run_runtime_closure() {
  local -a command=("$@" "--json-out" "$SUMMARY_OUT")
  print_command "${command[@]}"
  if [[ "$EXECUTE" != "1" ]]; then
    return 0
  fi

  mkdir -p "$(dirname "$SUMMARY_OUT")"
  rm -f "$SUMMARY_OUT"
  set +e
  "${command[@]}"
  local status=$?
  set -e
  return "$status"
}

run_gap_report() {
  local -a command=("$@" "--json-out" "$GAP_OUT")
  print_command "${command[@]}"
  if [[ "$EXECUTE" != "1" ]]; then
    return 0
  fi

  mkdir -p "$(dirname "$GAP_OUT")"
  rm -f "$GAP_OUT"
  if [[ ! -f "$SUMMARY_OUT" ]]; then
    echo "Runtime closure did not write ${SUMMARY_OUT}; refusing to reuse stale gap report." >&2
    return 4
  fi

  set +e
  "${command[@]}"
  local status=$?
  set -e
  return "$status"
}

require_execute_host() {
  if [[ "$EXECUTE" != "1" ]]; then
    return
  fi
  if [[ "$(uname -s)" != "Linux" ]]; then
    echo "DimOS closure runtime gates require a Linux simulation host." >&2
    exit 2
  fi
  if [[ ! "$ROS_DOMAIN_ID_VALUE" =~ ^[0-9]+$ || "$ROS_DOMAIN_ID_VALUE" == "0" || "$ROS_DOMAIN_ID_VALUE" -ge 232 ]]; then
    echo "ROS_DOMAIN_ID must be in 1..231 for an isolated simulation run." >&2
    exit 2
  fi
}

source_runtime_env() {
  export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
  export MUJOCO_GL="${MUJOCO_GL:-egl}"
  export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-egl}"
  export PYTHONPATH="$ROOT/src:$ROOT:${PYTHONPATH:-}"
  if [[ "$EXECUTE" != "1" ]]; then
    print_shell_line "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}"
    print_shell_line 'export MUJOCO_GL=${MUJOCO_GL:-egl}'
    print_shell_line 'export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-egl}'
    print_shell_line "export PYTHONPATH=${ROOT}/src:${ROOT}:\${PYTHONPATH:-}"
    print_shell_line 'source /opt/ros/humble/setup.bash'
    print_shell_line 'source install/setup.bash 2>/dev/null || true'
    return
  fi
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
  else
    echo "/opt/ros/humble/setup.bash is missing; continuing to host preflight." >&2
  fi
  if [[ -f "$ROOT/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "$ROOT/install/setup.bash"
    set -u
  fi
  # Some setup files clear ROS_DOMAIN_ID; restore the isolated domain before
  # the host preflight guard checks ROS-backed simulation gates.
  export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
}

require_preflight_green() {
  if [[ "$EXECUTE" != "1" ]]; then
    return
  fi
  python3 sim/scripts/dimos_host_preflight_guard.py "$HOST_PREFLIGHT_OUT"
}

require_execute_host
source_runtime_env

run_preflight "$HOST_PREFLIGHT_OUT" python3 sim/scripts/server_sim_closure.py --preset dimos_benchmark --required-only --host-preflight
require_preflight_green
RUNTIME_STATUS=0
run_runtime_closure python3 sim/scripts/server_sim_closure.py --preset dimos_benchmark --required-only --run-missing --strict --max-report-age-s 86400 || RUNTIME_STATUS=$?
GAP_STATUS=0
run_gap_report python3 sim/scripts/dimos_gap_report.py --summary "$SUMMARY_OUT" --include-dataflow --host-preflight-report "$HOST_PREFLIGHT_OUT" --format json || GAP_STATUS=$?
if [[ "$GAP_STATUS" -ne 0 && ! -f "$GAP_OUT" ]]; then
  echo "DimOS gap report failed and did not write ${GAP_OUT}; refusing to reuse stale gap report." >&2
  exit "$GAP_STATUS"
fi
if [[ "$RUNTIME_STATUS" -ne 0 ]]; then
  exit "$RUNTIME_STATUS"
fi
exit "$GAP_STATUS"
