#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
BUILD_DIR="${LINGTU_ERASOR2_BUILD_DIR:-${REPO_ROOT}/build/research-erasor2}"
cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
cmake --build "${BUILD_DIR}" -j "${LINGTU_BUILD_JOBS:-4}" --target erasor2_stage erasor2_clean
