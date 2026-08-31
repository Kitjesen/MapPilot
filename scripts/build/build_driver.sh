#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_DRIVER_BUILD_DIR:-${ROOT}/build/driver}"
BACKEND="${LINGTU_DRIVER_BACKEND:?set LINGTU_DRIVER_BACKEND to doso or go2}"

cmake -S "${ROOT}/src/drivers/real/motion" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" \
  -DLINGTU_DRIVER_BUILD_RUNTIME=ON \
  -DLINGTU_DRIVER_BACKEND="${BACKEND}" \
  -DBUILD_TESTING="${LINGTU_DRIVER_BUILD_TESTS:-ON}"
cmake --build "${BUILD_DIR}" --parallel "${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"

case "${LINGTU_DRIVER_RUN_TESTS:-1}" in
  1|on|ON|true|TRUE|yes|YES)
    (cd "${BUILD_DIR}" && ctest --output-on-failure)
    ;;
esac

BIN="${BUILD_DIR}/lingtu_driver"
[[ -x "${BIN}" ]] || { echo "ERROR: driver binary is missing: ${BIN}" >&2; exit 1; }
echo "${BIN} (${BACKEND})"
