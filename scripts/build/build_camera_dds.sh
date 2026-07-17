#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_CAMERA_DDS_BUILD_DIR:-${ROOT}/build/camera_dds}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"

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

cmake -S "${ROOT}/src/drivers/real/camera/native" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build "${BUILD_DIR}" --target lingtu_camera_dds --parallel "${JOBS}"

BIN="${BUILD_DIR}/lingtu_camera_dds"
if [[ ! -x "$BIN" ]]; then
  echo "ERROR: build finished but native camera DDS publisher is missing: $BIN" >&2
  echo "Rebuild with: bash scripts/build/build_camera_dds.sh" >&2
  exit 1
fi

echo "$BIN"
