#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DEFAULT_BUILD_DIR="${REPO_ROOT}/build/prune"
if grep -qiE "(microsoft|wsl)" /proc/version 2>/dev/null; then
  DEFAULT_BUILD_DIR="${REPO_ROOT}/build/prune_wsl"
fi
BUILD_DIR="${LINGTU_ERASOR2_STAGE_BUILD_DIR:-${LINGTU_PRUNE_BUILD_DIR:-${DEFAULT_BUILD_DIR}}}"
SRC_DIR="${REPO_ROOT}/src/maps/prune/cpp"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"

cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" -j "${JOBS}" --target erasor2_stage

printf 'Built %s\n' "${BUILD_DIR}/erasor2_stage"
