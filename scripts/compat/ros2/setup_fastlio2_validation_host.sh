#!/usr/bin/env bash
set -euo pipefail

# Prepare and verify the optional ROS 2 Fast-LIO2 compatibility lane.
#
# Common usage:
#   cd /path/to/lingtu
#   LINGTU_INSTALL_ROS2=1 LINGTU_RUN_ROS2_FASTLIO2=1 \
#     bash scripts/compat/ros2/setup_fastlio2_validation_host.sh
#
# When official package mirrors are blocked:
#   LINGTU_INSTALL_ROS2=1 LINGTU_USE_FISHROS=1 \
#     LINGTU_RUN_ROS2_FASTLIO2=1 \
#     bash scripts/compat/ros2/setup_fastlio2_validation_host.sh

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "${ROOT}"

INSTALL_ROS2="${LINGTU_INSTALL_ROS2:-0}"
USE_FISHROS="${LINGTU_USE_FISHROS:-0}"
ROS_DISTRO="${LINGTU_ROS_DISTRO:-}"
RUN_ROS2_FASTLIO2="${LINGTU_RUN_ROS2_FASTLIO2:-0}"
SKIP_APT="${LINGTU_SKIP_APT:-0}"
INSTALL_SYSTEM_DEPS="${LINGTU_INSTALL_SYSTEM_DEPS:-1}"
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

install_build_deps() {
  if [[ "${INSTALL_SYSTEM_DEPS}" != "1" || "${SKIP_APT}" == "1" ]]; then
    return
  fi
  if ! need_sudo; then
    log "sudo is unavailable; skipping apt build dependencies"
    return
  fi

  log "installing compatibility build dependencies"
  apt_get update
  apt_get install -y \
    curl wget gnupg lsb-release software-properties-common \
    build-essential cmake git python3-dev \
    libeigen3-dev libpcl-dev libyaml-cpp-dev \
    python3-colcon-common-extensions python3-rosdep
}

install_ros2_official() {
  local distro="$1"
  if [[ "${SKIP_APT}" == "1" ]]; then
    log "LINGTU_SKIP_APT=1; skipping ROS 2 package installation"
    return
  fi
  if ! need_sudo; then
    log "sudo is unavailable; cannot install ROS 2 packages"
    return 1
  fi

  # shellcheck disable=SC1091
  source /etc/os-release
  log "installing ROS 2 ${distro} on Ubuntu ${VERSION_ID:-unknown}"
  install_build_deps

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

install_ros2_fishros() {
  if [[ "${SKIP_APT}" == "1" ]]; then
    log "LINGTU_SKIP_APT=1; skipping FishROS"
    return
  fi
  log "running FishROS installer; select ROS 2 and the matching distro"
  wget http://fishros.com/install -O /tmp/fishros
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

verify_ros2_fastlio2_runtime() {
  if ! have ros2; then
    log "ros2 is unavailable; cannot verify fastlio2 executable"
    return 1
  fi

  log "verifying ROS 2 fastlio2 executable"
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

build_ros2_fastlio2_runtime() {
  local distro="$1"
  if [[ "${RUN_ROS2_FASTLIO2}" != "1" ]]; then
    log "LINGTU_RUN_ROS2_FASTLIO2=0; skipping optional ROS 2 fastlio2 build"
    return
  fi
  if ! have ros2; then
    log "ros2 is unavailable; cannot build ROS 2 fastlio2 package"
    return 1
  fi
  if ! have colcon; then
    log "colcon is unavailable; install python3-colcon-common-extensions first"
    return 1
  fi

  log "building ROS 2 fastlio2 package and dependencies"
  colcon build \
    --packages-up-to fastlio2 \
    --merge-install \
    --cmake-args -DBUILD_TESTING=OFF
  source_ros_if_present "${distro}"
  verify_ros2_fastlio2_runtime
}

main() {
  local distro
  distro="$(detect_ros_distro)"

  install_build_deps
  if ! have ros2 && [[ "${INSTALL_ROS2}" == "1" ]]; then
    if [[ "${USE_FISHROS}" == "1" ]]; then
      install_ros2_fishros
    else
      install_ros2_official "${distro}"
    fi
  fi

  source_ros_if_present "${distro}"
  build_ros2_fastlio2_runtime "${distro}"
  log "ROS 2 Fast-LIO2 compatibility setup complete"
}

main "$@"