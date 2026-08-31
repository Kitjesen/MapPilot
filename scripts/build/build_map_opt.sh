#!/usr/bin/env bash
# Build and test the short-lived native saved-map optimizer tools.
# Set LINGTU_POSE_GRAPH_OPT_LIBRARY to reuse a prebuilt Rust kernel library;
# otherwise the standalone CMake project builds the kernel with Cargo.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_MAP_OPT_BUILD_DIR:-${ROOT}/build/map_opt}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
PGO_BIN="${BUILD_DIR}/lt_pgo"
LOOP_VERIFY_BIN="${BUILD_DIR}/lt_loop_verify"
CMAKE_ARGS=(
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
  -DBUILD_TESTING=ON
)
if [[ -n "${LINGTU_POSE_GRAPH_OPT_LIBRARY:-}" ]]; then
  CMAKE_ARGS+=(
    -DLINGTU_POSE_GRAPH_OPT_LIBRARY="${LINGTU_POSE_GRAPH_OPT_LIBRARY}"
  )
fi

cmake -S "${ROOT}/src/localization/opt" -B "${BUILD_DIR}" \
  "${CMAKE_ARGS[@]}"
cmake --build "${BUILD_DIR}" \
  --target \
    lt_pgo \
    lt_loop_verify \
    pgo_test \
    loop_information_test \
    constraint_assembly_test \
  --parallel "${JOBS}"
ctest --test-dir "${BUILD_DIR}" \
  --output-on-failure \
  -R '^(pgo_core|loop_information|constraint_assembly)$'

if [[ ! -x "${PGO_BIN}" ]]; then
  echo "ERROR: native saved-map optimizer is missing after build: ${PGO_BIN}" >&2
  exit 1
fi
if [[ ! -x "${LOOP_VERIFY_BIN}" ]]; then
  echo "ERROR: native loop verifier is missing after build: ${LOOP_VERIFY_BIN}" >&2
  exit 1
fi

echo "${PGO_BIN}"
echo "${LOOP_VERIFY_BIN}"
