#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_LIVOX_SDK2_STREAM_BUILD_DIR:-$ROOT/build/livox_sdk2_stream}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"

if [ -n "${LINGTU_CYCLONEDDS_PREFIX:-}" ]; then
  export CMAKE_PREFIX_PATH="${LINGTU_CYCLONEDDS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi

cmake -S "$ROOT/src/drivers/real/lidar/sdk2_stream" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DLINGTU_LIVOX_SDK2_STREAM_BUILD_DDS="${LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS:-ON}"
cmake --build "$BUILD_DIR" --parallel "$JOBS"

BIN="$BUILD_DIR/livox_sdk2_stream"
if [[ ! -x "$BIN" ]]; then
  echo "build finished but binary is missing: $BIN" >&2
  exit 1
fi

echo "$BIN"
