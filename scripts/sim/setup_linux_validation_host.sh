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
#   LINGTU_CONDA_ENV=thunder2 bash scripts/sim/setup_linux_validation_host.sh

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

CONDA_ENV="${LINGTU_CONDA_ENV:-}"
SKIP_APT="${LINGTU_SKIP_APT:-0}"
RUN_MUJOCO="${LINGTU_RUN_MUJOCO:-1}"
INSTALL_SYSTEM_DEPS="${LINGTU_INSTALL_SYSTEM_DEPS:-1}"
INSTALL_PYTHON_DEPS="${LINGTU_INSTALL_PYTHON_DEPS:-1}"
RUN_VERIFY="${LINGTU_RUN_VERIFY:-1}"
SUDO_PASSWORD="${LINGTU_SUDO_PASSWORD:-}"
APT_RETRIES="${LINGTU_APT_RETRIES:-3}"
APT_TIMEOUT="${LINGTU_APT_TIMEOUT:-30}"

log() {
  printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

have() {
  command -v "$1" >/dev/null 2>&1
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

verify_mid360_pattern_asset() {
  if [[ "${RUN_MUJOCO}" != "1" ]]; then
    return
  fi
  local pattern="${ROOT}/sim/packages/sensors/livox/mid360/assets/mid360.npy"
  if [[ ! -f "${pattern}" ]]; then
    log "missing official MID-360 scan pattern asset: sim/packages/sensors/livox/mid360/assets/mid360.npy"
    return 1
  fi
  python3 - "${pattern}" <<'PY'
import sys
from pathlib import Path
import numpy as np

path = Path(sys.argv[1])
angles = np.load(path, mmap_mode="r")
if angles.ndim != 2 or angles.shape[1] != 2:
    raise SystemExit(f"invalid MID-360 pattern shape: {angles.shape}")
print(f"MID-360 pattern ok: {path} {angles.shape}")
PY
}

run_verification() {
  if [[ "${RUN_VERIFY}" != "1" ]]; then
    log "LINGTU_RUN_VERIFY=0; skipping verification commands"
    return
  fi

  export PYTHONPATH="${ROOT}/src:${ROOT}:${PYTHONPATH:-}"
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD="${PYTEST_DISABLE_PLUGIN_AUTOLOAD:-1}"

  local focused_tests=()
  if [[ "${RUN_MUJOCO}" == "1" ]]; then
    focused_tests+=(tests/runtime/test_sim_runtime_adapters.py)
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

}

main() {
  activate_conda_env
  print_environment
  install_system_deps
  install_python_deps
  verify_mid360_pattern_asset
  run_verification

  log "native validation-host setup and non-motion verification complete"
}

main "$@"
