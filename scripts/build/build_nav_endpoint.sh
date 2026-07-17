#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_NAV_ENDPOINT_BUILD_DIR:-$ROOT/build/nav_endpoint}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
RUN_TESTS="${LINGTU_NAV_ENDPOINT_RUN_TESTS:-1}"

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

cmake -S "$ROOT/src/nav/services/endpoint/cpp" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

cmake --build "$BUILD_DIR" --parallel "$JOBS"

case "${RUN_TESTS,,}" in
  1|on|true|yes)
    test_catalog="$(ctest --test-dir "$BUILD_DIR" -N)"
    for required_test in \
      test_nav_endpoint_config \
      test_nav_client \
      test_teleop_safety \
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
  lingtu_nav_native_endpoint \
  lingtu_traversability_dds \
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

echo "$BUILD_DIR/lingtu_nav_native_endpoint"
