#!/usr/bin/env bash
set -euo pipefail

for deprecated_var in \
  LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME \
  LINGTU_SLAM_BUILD_CYCLONE_DDS_RUNTIME \
  LINGTU_SLAM_WITH_FASTLIO2 \
  LINGTU_SLAM_FASTLIO2; do
  if [[ -v "$deprecated_var" ]]; then
    if [[ "$deprecated_var" == "LINGTU_SLAM_WITH_FASTLIO2" || "$deprecated_var" == "LINGTU_SLAM_FASTLIO2" ]]; then
      replacement_var="LINGTU_SLAM_LTLIO"
    else
      replacement_var="LINGTU_SLAM_BUILD_DDS_RUNTIME"
    fi
    echo "ERROR: $deprecated_var was removed; use $replacement_var instead." >&2
    exit 2
  fi
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_SLAM_CORE_BUILD_DIR:-$ROOT/build/slam_core}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
LTLIO_BACKEND="${LINGTU_SLAM_LTLIO:-ON}"
BUILD_DDS_RUNTIME="${LINGTU_SLAM_BUILD_DDS_RUNTIME:-ON}"
DEFAULT_BBS3D_ROOT="$ROOT/third_party/3d_bbs/install"
CPU_BBS3D_ROOT="${CPU_BBS3D_ROOT:-${LINGTU_BBS3D_PREFIX:-$DEFAULT_BBS3D_ROOT}}"
DEFAULT_SMALL_GICP_ROOT="$ROOT/third_party/research_localization/small_gicp"
SMALL_GICP_ROOT="${SMALL_GICP_ROOT:-${LINGTU_SMALL_GICP_ROOT:-$DEFAULT_SMALL_GICP_ROOT}}"
REQUIRE_BBS3D="${LINGTU_REQUIRE_BBS3D:-OFF}"

if [ -n "${LINGTU_CYCLONEDDS_PREFIX:-}" ]; then
  export CMAKE_PREFIX_PATH="${LINGTU_CYCLONEDDS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi

cmake_bool_on() {
  case "${1^^}" in
    1|ON|TRUE|YES) return 0 ;;
    *) return 1 ;;
  esac
}

check_ltlio_deps() {
  local probe_dir="$BUILD_DIR/.ltlio_dep_probe"
  mkdir -p "$probe_dir"
  cat > "$probe_dir/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.20)
project(lingtu_ltlio_dep_probe LANGUAGES C CXX)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
include("${LINGTU_ROOT}/scripts/build/cmake/gtsam.cmake")
find_package(yaml-cpp REQUIRED)
CMAKE

  if ! cmake -S "$probe_dir" -B "$probe_dir/build" -DLINGTU_ROOT="$ROOT" >/tmp/lingtu_ltlio_dep_probe.log 2>&1; then
    cat >&2 <<'EOF'
LTLIO native backend is ROS-free, but these C++ libraries must be visible to CMake:
  Eigen3, PCL(common/io/filters/kdtree/registration), locked GTSAM, yaml-cpp

Ubuntu baseline:
  sudo apt install -y libeigen3-dev libpcl-dev libyaml-cpp-dev

GTSAM is required by the online LIO-SAM backend. Use scripts/build/prepare_gtsam.sh
to build the locked SDK with system Eigen, then expose its prefix through
CMAKE_PREFIX_PATH. Existing older SDKs must be rebuilt for this source version.

Dependency probe output:
EOF
    cat /tmp/lingtu_ltlio_dep_probe.log >&2
    exit 2
  fi
}

if cmake_bool_on "$LTLIO_BACKEND"; then
  check_ltlio_deps
fi

cmake -S "$ROOT/src/localization/slam/cpp" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DLINGTU_SLAM_BUILD_TESTS="${LINGTU_SLAM_BUILD_TESTS:-ON}" \
  -DLINGTU_SLAM_BUILD_DDS_RUNTIME="$BUILD_DDS_RUNTIME" \
  -DLINGTU_SLAM_LTLIO_BACKEND="$LTLIO_BACKEND" \
  -DCPU_BBS3D_ROOT="$CPU_BBS3D_ROOT" \
  -DSMALL_GICP_ROOT="$SMALL_GICP_ROOT" \
  -DLINGTU_ENABLE_SMALL_GICP="${LINGTU_ENABLE_SMALL_GICP:-ON}" \
  -DLINGTU_REQUIRE_BBS3D="$REQUIRE_BBS3D"

cmake --build "$BUILD_DIR" --parallel "$JOBS"

if [[ "${LINGTU_SLAM_BUILD_TESTS:-ON}" == "ON" && -x "$BUILD_DIR/test_slam_contract" ]]; then
  "$BUILD_DIR/test_slam_contract"
  (
    cd "$BUILD_DIR"
    ctest --output-on-failure \
      -R 'messages_(icp_diagnostics|relocalization_gate|ieskf_initialization|ikd_tree_concurrency)'
  )
fi

if cmake_bool_on "$BUILD_DDS_RUNTIME"; then
  BIN="$BUILD_DIR/slamd"
  CONTROL_BIN="$BUILD_DIR/slamctl"
  if [[ ! -x "$BIN" ]]; then
    echo "ERROR: build finished but native SLAM DDS runtime is missing: $BIN" >&2
    echo "Rebuild with: LINGTU_SLAM_BUILD_DDS_RUNTIME=ON bash scripts/build/build_slam_core.sh" >&2
    exit 1
  fi
  if [[ ! -x "$CONTROL_BIN" ]]; then
    echo "ERROR: build finished but native SLAM control tool is missing: $CONTROL_BIN" >&2
    exit 1
  fi
  echo "$BIN"
  echo "$CONTROL_BIN"
fi
