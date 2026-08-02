#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NAV_CPP_DIR="$ROOT/src/nav/cpp"
BUILD_DIR="${LINGTU_NAV_ENDPOINT_BUILD_DIR:-$ROOT/build/nav_endpoint}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
BUILD_TESTS="${LINGTU_NAV_CPP_BUILD_TESTS:-ON}"
RUN_TESTS="${LINGTU_NAV_ENDPOINT_RUN_TESTS:-$BUILD_TESTS}"

ROOT="$(realpath -m -- "$ROOT")"
NAV_CPP_DIR="$(realpath -m -- "$NAV_CPP_DIR")"
BUILD_DIR="$(realpath -m -- "$BUILD_DIR")"

CMAKE_CACHE="$BUILD_DIR/CMakeCache.txt"
if [[ -f "$CMAKE_CACHE" ]]; then
  CACHE_SOURCE=""
  while IFS= read -r cache_line; do
    case "$cache_line" in
      CMAKE_HOME_DIRECTORY:INTERNAL=*)
        CACHE_SOURCE="${cache_line#CMAKE_HOME_DIRECTORY:INTERNAL=}"
        ;;
    esac
  done < "$CMAKE_CACHE"
  if [[ -z "$CACHE_SOURCE" ]]; then
    echo "ERROR: cannot identify the CMake source recorded in $CMAKE_CACHE" >&2
    exit 1
  fi

  CACHE_SOURCE="$(realpath -m -- "$CACHE_SOURCE")"
  if [[ "$CACHE_SOURCE" != "$NAV_CPP_DIR" ]]; then
    echo "ERROR: refusing to reuse navigation build directory configured from: $CACHE_SOURCE" >&2
    echo "Expected source: $NAV_CPP_DIR" >&2
    echo "Use a fresh build directory or remove $BUILD_DIR manually." >&2
    exit 1
  fi
fi

if [ -n "${LINGTU_CYCLONEDDS_PREFIX:-}" ]; then
  export CMAKE_PREFIX_PATH="${LINGTU_CYCLONEDDS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  if [ -d "${LINGTU_CYCLONEDDS_PREFIX}/bin" ]; then
    export PATH="${LINGTU_CYCLONEDDS_PREFIX}/bin:${PATH}"
  fi
  multiarch="$(cc -dumpmachine 2>/dev/null || true)"
  for lib_dir in "${LINGTU_CYCLONEDDS_PREFIX}/lib" "${LINGTU_CYCLONEDDS_PREFIX}/lib/${multiarch}"; do
    if [ -d "$lib_dir" ]; then
      export LD_LIBRARY_PATH="${lib_dir}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    fi
  done
  if [ -n "$multiarch" ] && [ -d "${LINGTU_CYCLONEDDS_PREFIX}/include/${multiarch}" ]; then
    export CFLAGS="-I${LINGTU_CYCLONEDDS_PREFIX}/include/${multiarch} ${CFLAGS:-}"
    export CXXFLAGS="-I${LINGTU_CYCLONEDDS_PREFIX}/include/${multiarch} ${CXXFLAGS:-}"
  fi
fi

cmake -S "$NAV_CPP_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DLINGTU_NAV_CPP_BUILD_ENDPOINT=ON \
  -DLINGTU_NAV_CPP_BUILD_TESTS="$BUILD_TESTS"

cmake --build "$BUILD_DIR" --parallel "$JOBS"

case "${RUN_TESTS,,}" in
  1|on|true|yes)
    test_catalog="$(ctest --test-dir "$BUILD_DIR" -N)"
    for required_test in \
      test_nav_endpoint_config \
      test_explore_control \
      test_explore_input \
      test_tare_policy \
      test_nav_client \
      test_nav_input_state_projector \
      test_rolling_segment_effect_coordinator \
      test_inspection_command_coordinator \
      test_control_loop_health \
      test_control_loop_runtime_guard \
      test_teleop_safety \
      test_active_occupancy_gate \
      test_far_c_api \
      test_far_planner \
      test_nav_loop \
      test_path_follower_core \
      test_local_planner_core; do
      if ! grep -Fq "$required_test" <<<"$test_catalog"; then
        echo "ERROR: required navigation test is missing from CTest: $required_test" >&2
        echo "Install the system GTest development package and reconfigure the build." >&2
        exit 1
      fi
    done
    ctest --test-dir "$BUILD_DIR" --output-on-failure
    ;;
esac

for bin_name in \
  navd \
  lingtu_traversability_dds \
  lingtu_explore_dds \
  lingtu_nav_control \
  lingtu_motion_mock_dds; do
  BIN="$BUILD_DIR/$bin_name"
  if [[ ! -x "$BIN" ]]; then
    echo "ERROR: build finished but native navigation DDS endpoint is missing: $BIN" >&2
    echo "Rebuild with: bash scripts/build/build_nav_endpoint.sh" >&2
    exit 1
  fi
done

NAV_CLIENT_LIB="$BUILD_DIR/liblingtu_nav_client.so"
if [[ ! -f "$NAV_CLIENT_LIB" ]]; then
  echo "ERROR: native navigation client library is missing: $NAV_CLIENT_LIB" >&2
  exit 1
fi

INSPECTION_LIB="$BUILD_DIR/inspection/liblingtu_inspection.so"
if [[ ! -f "$INSPECTION_LIB" ]]; then
  echo "ERROR: native inspection route library is missing: $INSPECTION_LIB" >&2
  exit 1
fi

INSPECTION_EVIDENCE_BRIDGE_LIB="$BUILD_DIR/liblingtu_inspection_evidence_bridge.so"
if [[ ! -f "$INSPECTION_EVIDENCE_BRIDGE_LIB" ]]; then
  echo "ERROR: native inspection evidence bridge library is missing: $INSPECTION_EVIDENCE_BRIDGE_LIB" >&2
  exit 1
fi

if ldd "$BUILD_DIR/navd" | grep -Eq 'libpcl_|libvtk'; then
  echo "ERROR: online navigation endpoint must not link PCL/VTK; keep PCD conversion in the offline converter" >&2
  exit 1
fi

echo "$BUILD_DIR/navd"
