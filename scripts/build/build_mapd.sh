#!/usr/bin/env bash
# Build the native typed-DDS maps runtime and local management endpoint.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_MAPD_BUILD_DIR:-${ROOT}/build/maps}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
MAPD="${BUILD_DIR}/mapd"
MAPCTL="${BUILD_DIR}/lingtu-mapctl"
PRUNE_BUILD_DIR="${LINGTU_PRUNE_BUILD_DIR:-${ROOT}/build/prune}"
PRUNE="${PRUNE_BUILD_DIR}/prune"
OCTOMAP_BUILD_DIR="${LINGTU_OCTOPLANNER3D_BUILD_DIR:-${ROOT}/build/octoplanner3d_headless}"
OCTOMAP_CONVERTER="${OCTOMAP_BUILD_DIR}/octoplanner3d_pcd_to_octomap"

if [ -n "${LINGTU_CYCLONEDDS_PREFIX:-}" ]; then
  export CMAKE_PREFIX_PATH="${LINGTU_CYCLONEDDS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi

# SaveMap requires a real PCL-backed .pcd -> OctoMap converter by default.
LINGTU_OCTOPLANNER3D_BUILD_DIR="${OCTOMAP_BUILD_DIR}" \
JOBS="${JOBS}" \
  bash "${ROOT}/scripts/build/build_octoplanner3d.sh" --require-pcl

LINGTU_PRUNE_BUILD_DIR="${PRUNE_BUILD_DIR}" \
LINGTU_BUILD_JOBS="${JOBS}" \
LINGTU_PRUNE_ERASOR2=OFF \
CMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  bash "${ROOT}/scripts/build/build_prune.sh"

cmake -S "${ROOT}/src/maps" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DLINGTU_MAPS_BUILD_MAPD=ON \
  -DLINGTU_MAPS_BUILD_TESTS=ON \
  -DLINGTU_MAPS_BUILD_TOOLS=ON
cmake --build "${BUILD_DIR}" \
  --target \
    mapd \
    lingtu_mapctl \
    lingtu_maps_mapd_engine_test \
    lingtu_maps_mapd_service_dispatch_test \
    lingtu_maps_mapd_save_coordinator_test \
    lingtu_maps_save_map_test \
    lingtu_maps_map_activation_test \
    lingtu_maps_mapd_uds_test \
    lingtu_maps_mapd_dds_test \
  --parallel "${JOBS}"

(
  cd "${BUILD_DIR}"
  ctest \
    --output-on-failure \
    -R '^(lingtu_maps_mapd_engine_test|lingtu_maps_mapd_service_dispatch_test|lingtu_maps_mapd_save_coordinator_test|lingtu_maps_save_map_test|lingtu_maps_map_activation_test|lingtu_maps_mapd_uds_test|lingtu_maps_mapd_dds_test)$'
)

if [[ ! -x "${MAPD}" ]]; then
  echo "ERROR: native maps runtime is missing after build: ${MAPD}" >&2
  exit 1
fi
if [[ ! -x "${MAPCTL}" ]]; then
  echo "ERROR: native map control CLI is missing after build: ${MAPCTL}" >&2
  exit 1
fi
if [[ ! -x "${PRUNE}" ]]; then
  echo "ERROR: native map prune runtime is missing after build: ${PRUNE}" >&2
  exit 1
fi
if [[ ! -x "${OCTOMAP_CONVERTER}" ]]; then
  echo "ERROR: native OctoMap converter is missing after build: ${OCTOMAP_CONVERTER}" >&2
  exit 1
fi
echo "${MAPD}"
echo "${MAPCTL}"
echo "${PRUNE}"
echo "${OCTOMAP_CONVERTER}"
