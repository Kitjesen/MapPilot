#!/usr/bin/env bash
set -euo pipefail

# Build a repo-local PCL install for one LingTu C++ feature profile.
# The PCL source and install prefix live under third_party/ (ignored by git).
# This avoids making LingTu's build depend on the Ubuntu-provided libpcl-dev
# version, while still leaving PCL out of portable/default Python runtime paths.
#
# Profiles:
#   octoplanner3d-pcd-converter: PCL common/io/octree for .pcd -> OctoMap
#     conversion in OctoPlanner3D headless builds. No ROS 2 packages and no
#     broader SLAM dependency promise.
#   slam-native: broader PCL component set for Fast-LIO2/localizer/PGO-style
#     native packages. Use this intentionally; it is not implied by the
#     OctoPlanner3D converter profile.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

PCL_VERSION="${LINGTU_PCL_VERSION:-pcl-1.14.1}"
PCL_REPO="${LINGTU_PCL_REPO:-https://github.com/PointCloudLibrary/pcl.git}"
THIRD_PARTY_ROOT="${LINGTU_THIRD_PARTY_ROOT:-${ROOT}/third_party}"
PCL_PROFILE="${LINGTU_PCL_PROFILE:-octoplanner3d-pcd-converter}"
BUILD_TYPE="${LINGTU_BUILD_TYPE:-Release}"
BUILD_JOBS="${LINGTU_BUILD_JOBS:-}"
CONFIGURE_ONLY="${LINGTU_PCL_CONFIGURE_ONLY:-0}"
PCL_SOURCE_DIR=""
PCL_BUILD_DIR=""
PCL_PREFIX=""
PCL_PROFILE_DESCRIPTION=""
PCL_CMAKE_COMPONENT_FLAGS=()

usage() {
  cat <<'EOF'
Usage:
  bash scripts/build/build_vendored_pcl.sh [--profile PROFILE] [--configure-only]

Profiles:
  octoplanner3d-pcd-converter     PCL common/io/octree for OctoPlanner3D .pcd conversion; no ROS 2.
  slam-native                     Broader PCL pack for SLAM/native packages.

Environment overrides:
  LINGTU_PCL_VERSION              Default: pcl-1.14.1
  LINGTU_PCL_REPO                 Default: https://github.com/PointCloudLibrary/pcl.git
  LINGTU_THIRD_PARTY_ROOT         Default: <repo>/third_party
  LINGTU_PCL_SOURCE_DIR           Default: <third_party>/pcl
  LINGTU_PCL_BUILD_DIR            Default: <third_party>/build/<version>-<profile>
  LINGTU_PCL_PREFIX               Default: <third_party>/install/<version>-<profile>
  LINGTU_BUILD_JOBS               Default: nproc
  LINGTU_PCL_CONFIGURE_ONLY=1     Configure but do not compile/install.
EOF
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --profile)
        if [[ $# -lt 2 ]]; then
          printf 'ERROR: --profile requires a value.\n' >&2
          usage >&2
          exit 2
        fi
        PCL_PROFILE="$2"
        shift 2
        ;;
      --configure-only)
        CONFIGURE_ONLY=1
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        printf 'ERROR: unknown option: %s\n' "$1" >&2
        usage >&2
        exit 2
        ;;
    esac
  done
}

set_profile_flags() {
  case "${PCL_PROFILE}" in
    octoplanner3d-pcd-converter)
      PCL_PROFILE_DESCRIPTION="PCL common/io/octree for OctoPlanner3D .pcd -> OctoMap conversion; no ROS 2 or full SLAM pack."
      PCL_CMAKE_COMPONENT_FLAGS=(
        -DBUILD_2d=OFF
        -DBUILD_common=ON
        -DBUILD_features=OFF
        -DBUILD_filters=OFF
        -DBUILD_geometry=OFF
        -DBUILD_io=ON
        -DBUILD_kdtree=OFF
        -DBUILD_keypoints=OFF
        -DBUILD_ml=OFF
        -DBUILD_octree=ON
        -DBUILD_outofcore=OFF
        -DBUILD_people=OFF
        -DBUILD_recognition=OFF
        -DBUILD_registration=OFF
        -DBUILD_sample_consensus=OFF
        -DBUILD_search=OFF
        -DBUILD_segmentation=OFF
        -DBUILD_simulation=OFF
        -DBUILD_stereo=OFF
        -DBUILD_surface=OFF
        -DBUILD_tracking=OFF
        -DBUILD_visualization=OFF
      )
      ;;
    slam-native)
      PCL_PROFILE_DESCRIPTION="Broader PCL pack for SLAM/native packages; larger than the OctoPlanner3D converter subset."
      PCL_CMAKE_COMPONENT_FLAGS=(
        -DBUILD_2d=OFF
        -DBUILD_common=ON
        -DBUILD_features=OFF
        -DBUILD_filters=ON
        -DBUILD_geometry=OFF
        -DBUILD_io=ON
        -DBUILD_kdtree=ON
        -DBUILD_keypoints=OFF
        -DBUILD_ml=OFF
        -DBUILD_octree=ON
        -DBUILD_outofcore=OFF
        -DBUILD_people=OFF
        -DBUILD_recognition=OFF
        -DBUILD_registration=ON
        -DBUILD_sample_consensus=ON
        -DBUILD_search=ON
        -DBUILD_segmentation=OFF
        -DBUILD_simulation=OFF
        -DBUILD_stereo=OFF
        -DBUILD_surface=OFF
        -DBUILD_tracking=OFF
        -DBUILD_visualization=OFF
      )
      ;;
    *)
      printf 'ERROR: unknown LINGTU_PCL_PROFILE=%s\n' "${PCL_PROFILE}" >&2
      usage >&2
      exit 2
      ;;
  esac
}

resolve_paths() {
  set_profile_flags
  PCL_SOURCE_DIR="${LINGTU_PCL_SOURCE_DIR:-${THIRD_PARTY_ROOT}/pcl}"
  PCL_BUILD_DIR="${LINGTU_PCL_BUILD_DIR:-${THIRD_PARTY_ROOT}/build/${PCL_VERSION}-${PCL_PROFILE}}"
  PCL_PREFIX="${LINGTU_PCL_PREFIX:-${THIRD_PARTY_ROOT}/install/${PCL_VERSION}-${PCL_PROFILE}}"
}

log() {
  printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

have() {
  command -v "$1" >/dev/null 2>&1
}

nproc_fallback() {
  if have nproc; then
    nproc
  else
    echo 4
  fi
}

clone_pcl() {
  mkdir -p "${THIRD_PARTY_ROOT}"
  if [[ -d "${PCL_SOURCE_DIR}/.git" ]]; then
    log "PCL source already present: ${PCL_SOURCE_DIR}"
    git -C "${PCL_SOURCE_DIR}" describe --tags --always || true
    return
  fi

  log "cloning ${PCL_REPO} ${PCL_VERSION} -> ${PCL_SOURCE_DIR}"
  if ! git clone --depth 1 --filter=blob:none --branch "${PCL_VERSION}" "${PCL_REPO}" "${PCL_SOURCE_DIR}"; then
    log "filtered clone failed; retrying plain shallow clone"
    rm -rf "${PCL_SOURCE_DIR}"
    git clone --depth 1 --branch "${PCL_VERSION}" "${PCL_REPO}" "${PCL_SOURCE_DIR}"
  fi
}

find_pcl_config() {
  find "${PCL_PREFIX}" -name PCLConfig.cmake -print -quit 2>/dev/null || true
}

write_env_file() {
  local pcl_config pcl_dir env_file
  pcl_config="$(find_pcl_config)"
  if [[ -z "${pcl_config}" ]]; then
    return 1
  fi
  pcl_dir="$(dirname "${pcl_config}")"
  env_file="${PCL_PREFIX}/lingtu-pcl-env.sh"
  cat > "${env_file}" <<EOF
# shellcheck shell=bash
# Source this file before building LingTu native packages with repo-local PCL.
export LINGTU_USE_VENDORED_PCL=1
export LINGTU_PCL_PROFILE="${PCL_PROFILE}"
export LINGTU_PCL_PREFIX="${PCL_PREFIX}"
export PCL_DIR="${pcl_dir}"
export CMAKE_PREFIX_PATH="${PCL_PREFIX}:\${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${PCL_PREFIX}/lib:${PCL_PREFIX}/lib64:\${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="${PCL_PREFIX}/lib/pkgconfig:${PCL_PREFIX}/lib64/pkgconfig:\${PKG_CONFIG_PATH:-}"
EOF
  log "wrote ${env_file}"
}

configure_pcl() {
  mkdir -p "${PCL_BUILD_DIR}" "${PCL_PREFIX}"
  log "configuring PCL ${PCL_VERSION} (${PCL_PROFILE})"
  printf 'Profile description: %s\n' "${PCL_PROFILE_DESCRIPTION}"
  printf 'Source: %s\nBuild:  %s\nPrefix: %s\n' "${PCL_SOURCE_DIR}" "${PCL_BUILD_DIR}" "${PCL_PREFIX}"
  cmake -S "${PCL_SOURCE_DIR}" -B "${PCL_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${PCL_PREFIX}" \
    -DPCL_SHARED_LIBS=ON \
    -DBUILD_apps=OFF \
    -DBUILD_examples=OFF \
    -DBUILD_global_tests=OFF \
    -DBUILD_tools=OFF \
    "${PCL_CMAKE_COMPONENT_FLAGS[@]}" \
    -DWITH_CUDA=OFF \
    -DWITH_DOCS=OFF \
    -DWITH_LIBUSB=OFF \
    -DWITH_OPENGL=OFF \
    -DWITH_OPENNI=OFF \
    -DWITH_OPENNI2=OFF \
    -DWITH_PCAP=OFF \
    -DWITH_QT=OFF \
    -DWITH_VTK=OFF
}

build_and_install_pcl() {
  local jobs
  jobs="${BUILD_JOBS:-$(nproc_fallback)}"
  log "building PCL with ${jobs} job(s)"
  cmake --build "${PCL_BUILD_DIR}" --parallel "${jobs}"
  log "installing PCL -> ${PCL_PREFIX}"
  cmake --install "${PCL_BUILD_DIR}"
  write_env_file
}

check_host() {
  case "$(uname -s 2>/dev/null || echo unknown)" in
    MINGW*|MSYS*|CYGWIN*)
      if [[ "${LINGTU_ALLOW_WINDOWS_PCL_BUILD:-0}" != "1" ]]; then
        printf 'ERROR: build_vendored_pcl.sh is intended for Linux/WSL robot-native builds.\n' >&2
        printf 'Run it inside WSL/Linux, or set LINGTU_ALLOW_WINDOWS_PCL_BUILD=1 if you intentionally want a Windows-native PCL build.\n' >&2
        return 1
      fi
      ;;
  esac
}

main() {
  parse_args "$@"
  resolve_paths
  check_host
  clone_pcl
  if [[ -n "$(find_pcl_config)" && "${CONFIGURE_ONLY}" != "1" ]]; then
    log "vendored PCL already installed for profile ${PCL_PROFILE}: $(find_pcl_config)"
    write_env_file
    exit 0
  fi
  configure_pcl
  if [[ "${CONFIGURE_ONLY}" == "1" ]]; then
    log "configure-only requested; not building PCL"
    exit 0
  fi
  build_and_install_pcl
  log "vendored PCL ready for profile ${PCL_PROFILE}"
  printf 'Source env: %s\n' "${PCL_PREFIX}/lingtu-pcl-env.sh"
}

main "$@"
