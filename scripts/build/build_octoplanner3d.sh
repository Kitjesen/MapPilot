#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SOURCE_DIR="${ROOT}/src/nav/cpp/planning/global/octoplanner"
VENDOR_DIR="${LINGTU_OCTOPLANNER3D_SOURCE_DIR:-${SOURCE_DIR}/vendor}"
BUILD_DIR="${LINGTU_OCTOPLANNER3D_BUILD_DIR:-${ROOT}/build/octoplanner3d_headless}"
JOBS="${LINGTU_BUILD_JOBS:-${JOBS:-$(nproc 2>/dev/null || echo 2)}}"
REQUIRE_PCL=OFF

case "${1:-}" in
  "") ;;
  --require-pcl) REQUIRE_PCL=ON ;;
  -h|--help)
    echo "Usage: bash scripts/build/build_octoplanner3d.sh [--require-pcl]"
    exit 0
    ;;
  *)
    echo "Unknown option: $1" >&2
    exit 2
    ;;
esac

case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*)
    echo "OctoPlanner3D must be built on Linux, WSL, or S100P." >&2
    exit 3
    ;;
esac

cmake_args=(
  -S "${SOURCE_DIR}"
  -B "${BUILD_DIR}"
  -DOCTOPLANNER3D_SOURCE_DIR="${VENDOR_DIR}"
  -DOCTOPLANNER3D_REQUIRE_PCL="${REQUIRE_PCL}"
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
)
if [[ -n "${PCL_DIR:-}" ]]; then
  cmake_args+=(-DPCL_DIR="${PCL_DIR}")
fi
cmake "${cmake_args[@]}"
cmake --build "${BUILD_DIR}" --parallel "${JOBS}" --target \
  octoplanner3d_headless \
  octoplanner3d_edit_octomap \
  octoplanner3d_pcd_to_octomap

cat <<EOF
Built OctoPlanner3D tools in ${BUILD_DIR}
  export LINGTU_OCTOMAP_EDITOR="${BUILD_DIR}/octoplanner3d_edit_octomap"
  export LINGTU_MAP_ARTIFACT_CONVERTER="${BUILD_DIR}/octoplanner3d_pcd_to_octomap"
EOF
