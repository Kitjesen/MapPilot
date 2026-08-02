#!/usr/bin/env bash
set -euo pipefail

# Prepare and verify a Linux host for LingTu native planning and simulation gates.
#
# This script is intentionally non-motion: it builds and validates software
# gates only. It does not send robot goals, cmd_vel, or service commands to
# physical hardware.
#
# Common usage on a validation host:
#   cd /path/to/lingtu
#   LINGTU_CONDA_ENV=thunder2 bash sim/scripts/setup_linux_validation_host.sh

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

CONDA_ENV="${LINGTU_CONDA_ENV:-}"
SKIP_APT="${LINGTU_SKIP_APT:-0}"
RUN_MUJOCO="${LINGTU_RUN_MUJOCO:-1}"
RUN_PCT="${LINGTU_RUN_PCT:-1}"
PCT_BUILD_LEGACY_NATIVE="${LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE:-0}"
RUN_MULTIFLOOR="${LINGTU_RUN_MULTIFLOOR:-1}"
RUN_NAV_KERNEL="${LINGTU_RUN_NAV_KERNEL:-1}"
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

install_system_deps() {
  if [[ "${INSTALL_SYSTEM_DEPS}" != "1" || "${SKIP_APT}" == "1" ]]; then
    return
  fi
  if ! need_sudo; then
    log "sudo is unavailable; skipping apt system dependencies"
    return
  fi

  log "installing native build/runtime dependencies"
  apt_get update
  local packages=(
    build-essential cmake git cargo python3-dev python3-pip python3-venv
    libboost-all-dev libeigen3-dev libpcl-dev libyaml-cpp-dev
    libopencv-dev patchelf
  )
  apt_get install -y "${packages[@]}"
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

run_verification() {
  if [[ "${RUN_VERIFY}" != "1" ]]; then
    log "LINGTU_RUN_VERIFY=0; skipping verification commands"
    return
  fi

  local closure_required=()
  export PYTHONPATH="${ROOT}/src:${ROOT}:${PYTHONPATH:-}"
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD="${PYTEST_DISABLE_PLUGIN_AUTOLOAD:-1}"

  if [[ "${RUN_PCT}" == "1" ]]; then
    log "PCT runtime inspection"
    python3 - <<'PY'
from nav.services.plan.global_planner.algorithm.pct.runtime.api import inspect_pct_runtime
import json
print(json.dumps(inspect_pct_runtime(), indent=2, ensure_ascii=False))
PY
  fi

  local focused_tests=()
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
    log "focused native runtime tests"
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
  activate_conda_env
  print_environment
  install_system_deps
  install_python_deps
  build_pct_runtime
  build_nav_kernel_runtime
  verify_mid360_pattern_asset
  run_verification

  log "native validation-host setup and non-motion verification complete"
}

main "$@"