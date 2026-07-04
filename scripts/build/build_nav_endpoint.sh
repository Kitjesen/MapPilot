#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_NAV_ENDPOINT_BUILD_DIR:-$ROOT/build/nav_endpoint}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"

if [ -n "${LINGTU_CYCLONEDDS_PREFIX:-}" ]; then
  export CMAKE_PREFIX_PATH="${LINGTU_CYCLONEDDS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi

cmake -S "$ROOT/src/nav/services/endpoint/cpp" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

cmake --build "$BUILD_DIR" --parallel "$JOBS"

for bin_name in lingtu_nav_native_endpoint lingtu_traversability_dds; do
  BIN="$BUILD_DIR/$bin_name"
  if [[ ! -x "$BIN" ]]; then
    echo "ERROR: build finished but native navigation DDS endpoint is missing: $BIN" >&2
    echo "Rebuild with: bash scripts/build/build_nav_endpoint.sh" >&2
    exit 1
  fi
done

echo "$BUILD_DIR/lingtu_nav_native_endpoint"
