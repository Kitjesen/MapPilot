#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="$ROOT/src/kernels/gateway/pointcloud_codec"
BUILD="$SRC/build"

cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
cmake --build "$BUILD" -j"${JOBS:-$(nproc 2>/dev/null || echo 4)}"

echo "Built pointcloud codec in $BUILD"
