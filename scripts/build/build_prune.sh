#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DEFAULT_BUILD_DIR="${REPO_ROOT}/build/prune"
if grep -qiE "(microsoft|wsl)" /proc/version 2>/dev/null; then
  DEFAULT_BUILD_DIR="${REPO_ROOT}/build/prune_wsl"
fi
BUILD_DIR="${LINGTU_PRUNE_BUILD_DIR:-${DEFAULT_BUILD_DIR}}"
SRC_DIR="${REPO_ROOT}/src/maps/prune/cpp"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
BUILD_TARGETS=(prune)
CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release -DLINGTU_PRUNE_ERASOR2=OFF)

case "${LINGTU_ERASOR2_USE_RERUN_STUB:-ON}" in
  0|OFF|off|FALSE|false|NO|no)
    CMAKE_ARGS+=(-DLINGTU_ERASOR2_USE_RERUN_STUB=OFF)
    ;;
  *)
    CMAKE_ARGS+=(-DLINGTU_ERASOR2_USE_RERUN_STUB=ON)
    ;;
esac

case "${LINGTU_PRUNE_ERASOR2:-OFF}" in
  1|ON|on|TRUE|true|YES|yes)
    CMAKE_ARGS+=(-DLINGTU_PRUNE_ERASOR2=ON)
    BUILD_TARGETS+=(erasor2_stage erasor2_clean)
    ;;
esac

cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" "${CMAKE_ARGS[@]}"
cmake --build "${BUILD_DIR}" -j "${JOBS}" --target "${BUILD_TARGETS[@]}"

printf 'Built %s\n' "${BUILD_DIR}/prune"
if [[ " ${BUILD_TARGETS[*]} " == *" erasor2_clean "* ]]; then
  printf 'Built %s\n' "${BUILD_DIR}/erasor2_stage"
  printf 'Built %s\n' "${BUILD_DIR}/erasor2_clean"
fi
