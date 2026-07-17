#!/usr/bin/env bash
# Build the native maps service used by the Python Module adapters.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_MAPS_BUILD_DIR:-${ROOT}/build/maps}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
LIB="${BUILD_DIR}/liblingtu_maps.so"

cmake -S "${ROOT}/src/maps" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DLINGTU_MAPS_BUILD_TESTS=OFF
cmake --build "${BUILD_DIR}" --target lingtu_maps_c_api --parallel "${JOBS}"

if [[ ! -f "${LIB}" ]]; then
  echo "ERROR: native maps library is missing after build: ${LIB}" >&2
  exit 1
fi
if command -v nm >/dev/null 2>&1 \
  && ! nm -D "${LIB}" | grep -Fq "lingtu_maps_service_get_map_points_json"; then
  echo "ERROR: native maps library lacks the saved-map point query ABI" >&2
  exit 1
fi

echo "${LIB}"
