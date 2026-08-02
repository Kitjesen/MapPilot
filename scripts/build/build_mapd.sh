#!/usr/bin/env bash
# Build the native typed-DDS maps runtime and the map service C ABI.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_MAPD_BUILD_DIR:-${ROOT}/build/maps}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
MAPD="${BUILD_DIR}/mapd"
MAPCTL="${BUILD_DIR}/lingtu-mapctl"
MAPS_LIB="${BUILD_DIR}/liblingtu_maps.so"

cmake -S "${ROOT}/src/maps" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DLINGTU_MAPS_BUILD_MAPD=ON \
  -DLINGTU_MAPS_BUILD_TESTS=ON \
  -DLINGTU_MAPS_BUILD_TOOLS=ON
cmake --build "${BUILD_DIR}" \
  --target \
    mapd \
    lingtu_mapctl \
    lingtu_maps_c_api \
    lingtu_maps_mapd_engine_test \
    lingtu_maps_map_activation_test \
    lingtu_maps_mapd_uds_test \
    lingtu_maps_mapd_dds_test \
    lingtu_maps_service_c_api_test \
  --parallel "${JOBS}"

ctest --test-dir "${BUILD_DIR}" \
  --output-on-failure \
  -R '^(lingtu_maps_mapd_engine_test|lingtu_maps_map_activation_test|lingtu_maps_mapd_uds_test|lingtu_maps_mapd_dds_test|lingtu_maps_service_c_api_test)$'

if [[ ! -x "${MAPD}" ]]; then
  echo "ERROR: native maps runtime is missing after build: ${MAPD}" >&2
  exit 1
fi
if [[ ! -x "${MAPCTL}" ]]; then
  echo "ERROR: native map control CLI is missing after build: ${MAPCTL}" >&2
  exit 1
fi
if [[ ! -f "${MAPS_LIB}" ]]; then
  echo "ERROR: native maps service library is missing after build: ${MAPS_LIB}" >&2
  exit 1
fi

echo "${MAPD}"
echo "${MAPCTL}"
