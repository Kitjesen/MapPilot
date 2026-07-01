#!/usr/bin/env bash
set -euo pipefail

# Prepare and verify a Linux server for LingTu portable planning/MuJoCo gates.
#
# This script is intentionally non-motion: it builds and validates software
# gates only. It does not send robot goals, cmd_vel, or service commands to
# physical hardware.
#
# Common usage on a server:
#   cd /path/to/lingtu
#   LINGTU_CONDA_ENV=thunder2 bash scripts/deploy/setup_server_ros_pct.sh
#
# Optional ROS2 compatibility lane, for legacy SLAM parity only:
#   LINGTU_INSTALL_ROS2=1 LINGTU_RUN_ROS2_FASTLIO2=1 \
#     bash scripts/deploy/setup_server_ros_pct.sh
#
# Optional FishROS path for that compatibility lane, when official apt setup is
# blocked in China:
#   LINGTU_INSTALL_ROS2=1 LINGTU_USE_FISHROS=1 \
#     LINGTU_RUN_ROS2_FASTLIO2=1 bash scripts/deploy/setup_server_ros_pct.sh

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

INSTALL_ROS2="${LINGTU_INSTALL_ROS2:-0}"
USE_FISHROS="${LINGTU_USE_FISHROS:-0}"
ROS_DISTRO="${LINGTU_ROS_DISTRO:-}"
CONDA_ENV="${LINGTU_CONDA_ENV:-}"
SKIP_APT="${LINGTU_SKIP_APT:-0}"
RUN_MUJOCO="${LINGTU_RUN_MUJOCO:-1}"
RUN_PCT="${LINGTU_RUN_PCT:-1}"
PCT_BUILD_LEGACY_NATIVE="${LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE:-0}"
RUN_MULTIFLOOR="${LINGTU_RUN_MULTIFLOOR:-1}"
RUN_NAV_KERNEL="${LINGTU_RUN_NAV_KERNEL:-1}"
RUN_ROS2_LOCAL_PLANNER="${LINGTU_RUN_ROS2_LOCAL_PLANNER:-0}"
RUN_ROS2_FASTLIO2="${LINGTU_RUN_ROS2_FASTLIO2:-0}"
RUN_ROUTECHECK_PREFLIGHT="${LINGTU_RUN_ROUTECHECK_PREFLIGHT:-1}"
SETUP_CLOSURE_MAX_REPORT_AGE_S="${LINGTU_SETUP_CLOSURE_MAX_REPORT_AGE_S:-21600}"
INSTALL_SYSTEM_DEPS="${LINGTU_INSTALL_SYSTEM_DEPS:-1}"
INSTALL_PYTHON_DEPS="${LINGTU_INSTALL_PYTHON_DEPS:-1}"
RUN_VERIFY="${LINGTU_RUN_VERIFY:-1}"
SUDO_PASSWORD="${LINGTU_SUDO_PASSWORD:-}"
APT_RETRIES="${LINGTU_APT_RETRIES:-3}"
APT_TIMEOUT="${LINGTU_APT_TIMEOUT:-30}"
MID360_PATTERN_SHA256="448821576a658673e8f7929992c8c0d687eb052657d7b584d038729a83da1bfb"

log() {
  printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

have() {
  command -v "$1" >/dev/null 2>&1
}

join_by_comma() {
  local IFS=,
  printf '%s' "$*"
}

need_sudo() {
  if [[ "${SKIP_APT}" == "1" ]]; then
    return 1
  fi
  if [[ "$(id -u)" == "0" ]]; then
    return 0
  fi
  if ! have sudo; then
    return 1
  fi
  if sudo -n true >/dev/null 2>&1; then
    return 0
  fi
  [[ -n "${SUDO_PASSWORD}" ]]
}

ros_compat_requested() {
  [[ "${INSTALL_ROS2}" == "1" \
    || "${RUN_ROS2_FASTLIO2}" == "1" ]]
}

sudo_cmd() {
  if [[ "$(id -u)" == "0" ]]; then
    "$@"
  elif sudo -n true >/dev/null 2>&1; then
    sudo "$@"
  elif [[ -n "${SUDO_PASSWORD}" ]]; then
    printf '%s\n' "${SUDO_PASSWORD}" | sudo -S "$@"
  else
    sudo "$@"
  fi
}

apt_get() {
  sudo_cmd apt-get \
    -o "Acquire::Retries=${APT_RETRIES}" \
    -o "Acquire::http::Timeout=${APT_TIMEOUT}" \
    -o "Acquire::https::Timeout=${APT_TIMEOUT}" \
    "$@"
}

detect_ros_distro() {
  if [[ -n "${ROS_DISTRO}" ]]; then
    printf '%s\n' "${ROS_DISTRO}"
    return
  fi
  # shellcheck disable=SC1091
  source /etc/os-release
  case "${VERSION_CODENAME:-}" in
    jammy) printf 'humble\n' ;;
    noble) printf 'jazzy\n' ;;
    *) printf 'humble\n' ;;
  esac
}

activate_conda_env() {
  if [[ -z "${CONDA_ENV}" ]]; then
    return
  fi
  if have conda; then
    # shellcheck disable=SC1091
    source "$(conda info --base)/etc/profile.d/conda.sh"
    conda activate "${CONDA_ENV}"
    return
  fi
  if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
    # shellcheck disable=SC1091
    source "${HOME}/miniconda3/etc/profile.d/conda.sh"
    conda activate "${CONDA_ENV}"
    return
  fi
  if [[ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]]; then
    # shellcheck disable=SC1091
    source "${HOME}/anaconda3/etc/profile.d/conda.sh"
    conda activate "${CONDA_ENV}"
    return
  fi
  log "conda env requested (${CONDA_ENV}) but conda was not found; continuing with system Python"
}

install_ros2_official() {
  local distro="$1"
  if [[ "${SKIP_APT}" == "1" ]]; then
    log "LINGTU_SKIP_APT=1; skipping ROS2/system package installation"
    return
  fi
  if ! need_sudo; then
    log "sudo is unavailable; cannot install ROS2/system packages"
    return 1
  fi

  # shellcheck disable=SC1091
  source /etc/os-release
  log "installing ROS2 ${distro} and build dependencies on Ubuntu ${VERSION_ID:-unknown}"

  install_system_deps

  if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
    sudo_cmd mkdir -p /usr/share/keyrings
    local ros_key_tmp
    local ros_list_tmp
    ros_key_tmp="$(mktemp)"
    ros_list_tmp="$(mktemp)"
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "${ros_key_tmp}"
    printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' \
      "$(dpkg --print-architecture)" "${VERSION_CODENAME}" >"${ros_list_tmp}"
    sudo_cmd install -m 0644 "${ros_key_tmp}" /usr/share/keyrings/ros-archive-keyring.gpg
    sudo_cmd install -m 0644 "${ros_list_tmp}" /etc/apt/sources.list.d/ros2.list
    rm -f "${ros_key_tmp}" "${ros_list_tmp}"
  fi

  apt_get update
  apt_get install -y "ros-${distro}-desktop" \
    "ros-${distro}-pcl-conversions" \
    "ros-${distro}-tf2-geometry-msgs"

  if ! rosdep --version >/dev/null 2>&1; then
    log "rosdep still unavailable after install"
  elif [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo_cmd rosdep init || true
  fi
  rosdep update || true
}

install_system_deps() {
  if [[ "${INSTALL_SYSTEM_DEPS}" != "1" || "${SKIP_APT}" == "1" ]]; then
    return
  fi
  if ! need_sudo; then
    log "sudo is unavailable; skipping apt system dependencies"
    return
  fi

  log "installing base build/runtime dependencies"
  apt_get update
  local packages=(
    curl gnupg lsb-release software-properties-common \
    build-essential cmake git cargo python3-dev python3-pip python3-venv \
    libboost-all-dev libeigen3-dev libpcl-dev libyaml-cpp-dev \
    libopencv-dev patchelf
  )
  if ros_compat_requested; then
    packages+=(python3-colcon-common-extensions python3-rosdep)
  fi
  apt_get install -y "${packages[@]}"
}

install_ros2_fishros() {
  if [[ "${SKIP_APT}" == "1" ]]; then
    log "LINGTU_SKIP_APT=1; skipping FishROS"
    return
  fi
  log "running FishROS installer; select ROS2 and the distro matching this OS when prompted"
  wget http://fishros.com/install -O /tmp/fishros
  # FishROS is intentionally interactive. It is useful on remote terminals when
  # official apt mirrors are slow or blocked.
  bash -lc ". /tmp/fishros"
}

source_ros_if_present() {
  local distro="$1"
  if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${distro}/setup.bash"
    set -u
  fi
  if [[ -f "${ROOT}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${ROOT}/install/setup.bash"
    set -u
  fi
}

print_environment() {
  log "environment"
  uname -a
  if [[ -f /etc/os-release ]]; then
    sed -n '1,8p' /etc/os-release
  fi
  printf 'root=%s\n' "${ROOT}"
  printf 'python=%s\n' "$(python3 --version 2>&1)"
  printf 'python_path=%s\n' "$(command -v python3 || true)"
  printf 'ros_compat_requested=%s\n' "$(ros_compat_requested && printf '1' || printf '0')"
  printf 'ros2=%s\n' "$(command -v ros2 || true)"
  printf 'colcon=%s\n' "$(command -v colcon || true)"
  printf 'cmake=%s\n' "$(cmake --version 2>/dev/null | head -1 || true)"
  printf 'gcc=%s\n' "$(gcc --version 2>/dev/null | head -1 || true)"
  if have nvidia-smi; then
    nvidia-smi --query-gpu=name,memory.total --format=csv,noheader | head -8 || true
  fi
}

install_python_deps() {
  if [[ "${INSTALL_PYTHON_DEPS}" != "1" ]]; then
    log "LINGTU_INSTALL_PYTHON_DEPS=0; skipping Python dependency installation"
    return
  fi
  log "installing Python verification dependencies"
  if ! python3 -m pip --version >/dev/null 2>&1; then
    if python3 -m ensurepip --user >/dev/null 2>&1; then
      log "bootstrapped pip with ensurepip"
    elif need_sudo; then
      log "pip missing; installing python3-pip"
      apt_get update
      apt_get install -y python3-pip
    else
      log "pip is missing and sudo is unavailable; install python3-pip or use a conda env"
      return 1
    fi
  fi

  local pip_target=("--user")
  if python3 - <<'PY'
import os, sys
raise SystemExit(0 if (sys.prefix != sys.base_prefix or os.environ.get("CONDA_PREFIX")) else 1)
PY
  then
    pip_target=()
  fi

  python3 -m pip install "${pip_target[@]}" -q --upgrade pip \
    || python3 -m pip install "${pip_target[@]}" --break-system-packages -q --upgrade pip
  python3 -m pip install "${pip_target[@]}" -q pytest numpy scipy pyyaml requests fastapi httpx uvicorn \
    || python3 -m pip install "${pip_target[@]}" --break-system-packages -q pytest numpy scipy pyyaml requests fastapi httpx uvicorn
  if [[ "${RUN_MUJOCO}" == "1" ]]; then
    python3 -m pip install "${pip_target[@]}" -q mujoco onnxruntime \
      || python3 -m pip install "${pip_target[@]}" --break-system-packages -q mujoco onnxruntime
  fi
}

build_pct_runtime() {
  if [[ "${RUN_PCT}" != "1" ]]; then
    return
  fi
  if [[ "${PCT_BUILD_LEGACY_NATIVE}" == "1" ]]; then
    log "building PCT legacy native/GTSAM runtime for parity baselines"
    LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=1 \
      JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}" \
      bash "${ROOT}/src/nav/services/plan/global_planner/algorithm/pct/runtime/build_legacy_native_x86_64.sh"
    return
  fi

  log "building PCT Rust GPMP runtime"
  python3 scripts/build/build_rust_kernels.py --target gpmp_trajectory_optimizer --release

  local arch
  case "$(uname -m)" in
    x86_64|amd64) arch="x86_64" ;;
    aarch64|arm64) arch="aarch64" ;;
    *) arch="$(uname -m)" ;;
  esac
  local release_dir="${ROOT}/src/kernels/planning/gpmp_trajectory_optimizer/target/release"
  local out_dir="${ROOT}/src/nav/services/plan/global_planner/algorithm/pct/runtime/rust/${arch}"
  mkdir -p "${out_dir}"
  cp -a "${release_dir}/gpmp_optimize" "${out_dir}/"
  cp -a "${release_dir}/liblingtu_gpmp_trajectory_optimizer.so" "${out_dir}/"
  chmod +x "${out_dir}/gpmp_optimize"
  log "PCT Rust GPMP runtime artifacts installed to ${out_dir}"
}

build_nav_kernel_runtime() {
  if [[ "${RUN_NAV_KERNEL}" != "1" ]]; then
    return
  fi
  log "building nav_kernel nanobind runtime for production local planning"
  bash "${ROOT}/scripts/build/build_nav_kernel.sh" --clean
}

verify_mid360_pattern_asset() {
  if [[ "${RUN_MUJOCO}" != "1" ]]; then
    return
  fi
  local pattern="${ROOT}/sim/assets/livox/mid360.npy"
  if [[ ! -f "${pattern}" ]]; then
    log "missing official MID-360 scan pattern asset: sim/assets/livox/mid360.npy"
    return 1
  fi
  python3 - "${pattern}" "${MID360_PATTERN_SHA256}" <<'PY'
import hashlib
import sys
from pathlib import Path

path = Path(sys.argv[1])
expected = sys.argv[2]
digest = hashlib.sha256(path.read_bytes()).hexdigest()
if digest != expected:
    print(f"MID-360 pattern SHA256 mismatch: {digest} != {expected}", file=sys.stderr)
    raise SystemExit(1)
print(f"MID-360 pattern ok: {path} {digest}")
PY
}

warn_deprecated_ros2_local_planner_setup() {
  if [[ "${RUN_ROS2_LOCAL_PLANNER}" != "1" ]]; then
    return
  fi
  log "LINGTU_RUN_ROS2_LOCAL_PLANNER=1 is deprecated and ignored; nav_kernel/nanobind is the production local-planning runtime"
}

verify_ros2_fastlio2_runtime() {
  if [[ "${RUN_ROS2_FASTLIO2}" != "1" ]]; then
    return
  fi
  if ! have ros2; then
    log "ros2 is unavailable; cannot verify fastlio2 executable"
    return 1
  fi

  log "verifying ROS2 fastlio2 executable"
  local executables
  if ! executables="$(ros2 pkg executables fastlio2 2>&1)"; then
    printf '%s\n' "${executables}" >&2
    log "fastlio2 package is not visible; source install/setup.bash or rerun the colcon build"
    return 1
  fi
  if ! grep -Eq "^[[:space:]]*fastlio2[[:space:]]+lio_node([[:space:]]|\$)" <<<"${executables}"; then
    printf '%s\n' "${executables}" >&2
    log "fastlio2 executable missing: lio_node"
    return 1
  fi

  if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
    log "livox_ros_driver2 package is not visible; fastlio2 CustomMsg dependency is incomplete"
    return 1
  fi
}

build_ros2_local_planner_setup() {
  local _distro="$1"
  if [[ "${RUN_ROS2_LOCAL_PLANNER}" != "1" ]]; then
    log "skipping legacy ROS2 local_planner build; nav_kernel/nanobind is the production local-planning runtime"
    return
  fi
  warn_deprecated_ros2_local_planner_setup
}

build_ros2_fastlio2_runtime() {
  local distro="$1"
  if [[ "${RUN_ROS2_FASTLIO2}" != "1" ]]; then
    log "LINGTU_RUN_ROS2_FASTLIO2=0; skipping optional ROS2 fastlio2 build"
    return
  fi
  if ! have ros2; then
    log "ros2 is unavailable; cannot build ROS2 fastlio2 package"
    return 1
  fi
  if ! have colcon; then
    log "colcon is unavailable; install python3-colcon-common-extensions first"
    return 1
  fi

  log "building ROS2 fastlio2 package and dependencies"
  colcon build \
    --packages-up-to fastlio2 \
    --merge-install \
    --cmake-args -DBUILD_TESTING=OFF
  source_ros_if_present "${distro}"
  verify_ros2_fastlio2_runtime
}

run_verification() {
  if [[ "${RUN_VERIFY}" != "1" ]]; then
    log "LINGTU_RUN_VERIFY=0; skipping verification commands"
    return
  fi

  local closure_required=()
  export PYTHONPATH="${ROOT}/src:${ROOT}:${PYTHONPATH:-}"
  # ROS2 installs launch_testing pytest hooks globally. They can import
  # unrelated script-style tests during focused Python checks, so keep this
  # verification lane plugin-clean unless the caller opts out explicitly.
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD="${PYTEST_DISABLE_PLUGIN_AUTOLOAD:-1}"

  if [[ "${RUN_PCT}" == "1" ]]; then
    log "PCT runtime inspection"
    python3 - <<'PY'
from nav.services.plan.global_planner.algorithm.pct.runtime.api import inspect_pct_runtime
import json
print(json.dumps(inspect_pct_runtime(), indent=2, ensure_ascii=False))
PY
  fi

  log "focused API/runtime tests"
  local focused_tests=(
    src/runtime/tests/test_gateway_commands.py
    src/runtime/tests/test_gateway_route_split.py
  )
  if [[ "${RUN_PCT}" == "1" ]]; then
    focused_tests+=(src/runtime/tests/test_pct_runtime.py)
  fi
  if [[ "${RUN_MUJOCO}" == "1" ]]; then
    focused_tests+=(src/runtime/tests/test_sim_runtime_adapters.py)
  fi
  local existing_tests=()
  local test_path
  for test_path in "${focused_tests[@]}"; do
    if [[ -f "${test_path}" ]]; then
      existing_tests+=("${test_path}")
    else
      log "skipping missing focused test: ${test_path}"
    fi
  done
  if [[ "${#existing_tests[@]}" -gt 0 ]]; then
    python3 -m pytest "${existing_tests[@]}" -q --tb=short
  fi

  if [[ "${RUN_PCT}" == "1" ]]; then
    log "strict PCT planning gate"
    python3 sim/scripts/multifloor_nav_validation.py \
      --output-dir artifacts/server_pct_gate \
      --route same_floor \
      --planners pct \
      --skip-mujoco \
      --strict \
      --json-out artifacts/server_pct_gate/report.json
  fi

  if [[ "${RUN_PCT}" == "1" && "${RUN_MULTIFLOOR}" == "1" ]]; then
    log "multi-floor exploration/local-planning closure gate, simulation-only"
    python3 sim/scripts/multifloor_nav_validation.py \
      --output-dir artifacts/server_sim_closure/multifloor_exploration \
      --route matrix \
      --planners pct,astar \
      --skip-mujoco \
      --frontier-loop \
      --local-planner-backend nanobind \
      --require-production-local-planner \
      --strict \
      --json-out artifacts/server_sim_closure/multifloor_exploration/report.json
    closure_required+=(multifloor_exploration)
  fi

  if [[ "${RUN_ROUTECHECK_PREFLIGHT}" == "1" ]]; then
    log "Gateway routecheck preflight closure gate, non-motion"
    python3 sim/scripts/routecheck_preflight_gate.py \
      --map server_sim_demo \
      --goal-x 1.0 \
      --goal-y 0.0 \
      --goal-yaw 0.0 \
      --json-out artifacts/server_sim_closure/routecheck/summary.json \
      --strict
    closure_required+=(routecheck_preflight)
  fi

  if [[ "${RUN_MUJOCO}" == "1" ]]; then
    log "MuJoCo bridge-loop gate, simulation-only"
    python3 sim/scripts/multifloor_nav_validation.py \
      --output-dir artifacts/server_mujoco_bridge \
      --route same_floor \
      --planners astar \
      --bridge-loop \
      --strict \
      --json-out artifacts/server_mujoco_bridge/report.json
  fi

  if [[ "${#closure_required[@]}" -gt 0 ]]; then
    log "server simulation closure summary for setup-generated gates"
    python3 sim/scripts/server_sim_closure.py \
      --required "$(join_by_comma "${closure_required[@]}")" \
      --required-only \
      --max-report-age-s "${SETUP_CLOSURE_MAX_REPORT_AGE_S}" \
      --json-out artifacts/server_sim_closure_summary_setup.json \
      --strict
  fi
}

main() {
  local distro
  distro="$(detect_ros_distro)"

  activate_conda_env
  print_environment

  if ! have ros2 && [[ "${INSTALL_ROS2}" == "1" ]]; then
    if [[ "${USE_FISHROS}" == "1" ]]; then
      install_ros2_fishros
    else
      install_ros2_official "${distro}"
    fi
  fi

  source_ros_if_present "${distro}"
  install_system_deps
  install_python_deps
  build_pct_runtime
  build_nav_kernel_runtime
  build_ros2_local_planner_setup "${distro}"
  build_ros2_fastlio2_runtime "${distro}"
  verify_mid360_pattern_asset
  run_verification

  log "server setup and non-motion verification complete"
}

main "$@"
