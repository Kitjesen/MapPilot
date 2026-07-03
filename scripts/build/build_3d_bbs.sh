#!/usr/bin/env bash
set -euo pipefail

# Build the CPU-only 3D-BBS global relocalization library into third_party/.
# The installed prefix is consumed by the native Fast-LIO SLAM runtime through
# CPU_BBS3D_ROOT; no ROS 2 packages are involved.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

BBS3D_REPO="${LINGTU_BBS3D_REPO:-https://github.com/KOKIAOKI/3d_bbs.git}"
BBS3D_REF="${LINGTU_BBS3D_REF:-41529a34a2fb9618b5ff560fb3c2363f1615666d}"
THIRD_PARTY_ROOT="${LINGTU_THIRD_PARTY_ROOT:-${ROOT}/third_party}"
BBS3D_SOURCE_DIR="${LINGTU_BBS3D_SOURCE_DIR:-${THIRD_PARTY_ROOT}/3d_bbs}"
BBS3D_BUILD_DIR="${LINGTU_BBS3D_BUILD_DIR:-${THIRD_PARTY_ROOT}/build/3d_bbs-cpu}"
BBS3D_PREFIX="${CPU_BBS3D_ROOT:-${LINGTU_BBS3D_PREFIX:-${BBS3D_SOURCE_DIR}/install}}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
CONFIGURE_ONLY="${LINGTU_BBS3D_CONFIGURE_ONLY:-0}"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/build/build_3d_bbs.sh [--configure-only]

Environment overrides:
  LINGTU_BBS3D_REPO              Default: https://github.com/KOKIAOKI/3d_bbs.git
  LINGTU_BBS3D_REF               Default: 41529a34a2fb9618b5ff560fb3c2363f1615666d
  LINGTU_THIRD_PARTY_ROOT        Default: <repo>/third_party
  LINGTU_BBS3D_SOURCE_DIR        Default: <third_party>/3d_bbs
  LINGTU_BBS3D_BUILD_DIR         Default: <third_party>/build/3d_bbs-cpu
  CPU_BBS3D_ROOT                 Default: <third_party>/3d_bbs/install
  LINGTU_BBS3D_CONFIGURE_ONLY=1  Configure but do not compile/install.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
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

mkdir -p "${THIRD_PARTY_ROOT}" "$(dirname "${BBS3D_BUILD_DIR}")"

if [[ ! -d "${BBS3D_SOURCE_DIR}/.git" ]]; then
  git clone --recursive "${BBS3D_REPO}" "${BBS3D_SOURCE_DIR}"
fi

git -C "${BBS3D_SOURCE_DIR}" fetch --tags --depth 1 origin "${BBS3D_REF}" || true
git -C "${BBS3D_SOURCE_DIR}" checkout --detach "${BBS3D_REF}"
git -C "${BBS3D_SOURCE_DIR}" submodule update --init --recursive

cmake -S "${BBS3D_SOURCE_DIR}" -B "${BBS3D_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${BBS3D_PREFIX}" \
  -DBUILD_CUDA=OFF \
  -DBUILD_TESTING=OFF

if [[ "${CONFIGURE_ONLY}" == "1" ]]; then
  echo "Configured 3D-BBS CPU build at ${BBS3D_BUILD_DIR}"
  exit 0
fi

cmake --build "${BBS3D_BUILD_DIR}" --parallel "${JOBS}"
cmake --install "${BBS3D_BUILD_DIR}"

if [[ ! -f "${BBS3D_PREFIX}/include/cpu_bbs3d/bbs3d.hpp" ]]; then
  echo "ERROR: missing installed 3D-BBS header: ${BBS3D_PREFIX}/include/cpu_bbs3d/bbs3d.hpp" >&2
  exit 1
fi
if [[ ! -f "${BBS3D_PREFIX}/lib/libcpu_bbs3d.so" && ! -f "${BBS3D_PREFIX}/lib64/libcpu_bbs3d.so" ]]; then
  echo "ERROR: missing installed 3D-BBS library under ${BBS3D_PREFIX}/lib" >&2
  exit 1
fi

echo "CPU_BBS3D_ROOT=${BBS3D_PREFIX}"
