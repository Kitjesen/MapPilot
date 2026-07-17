#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_DDS_PROBE_BUILD_DIR:-$ROOT/build/dds_probe}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"

if [ -n "${LINGTU_CYCLONEDDS_PREFIX:-}" ]; then
  export CMAKE_PREFIX_PATH="${LINGTU_CYCLONEDDS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${LINGTU_CYCLONEDDS_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

IDLC="${CYCLONEDDS_IDLC_EXECUTABLE:-$(command -v idlc || true)}"
if [ -z "$IDLC" ]; then
  echo "ERROR: CycloneDDS idlc not found; install cyclonedds-tools." >&2
  exit 2
fi

mkdir -p "$BUILD_DIR"
(
  cd "$BUILD_DIR"
  "$IDLC" -l c "$ROOT/src/message/idl/lingtu_slam.idl"
)

CC="${CC:-cc}"
CXX="${CXX:-c++}"
"$CC" -std=c11 -O2 \
  -I"$BUILD_DIR" \
  -c "$BUILD_DIR/lingtu_slam.c" \
  -o "$BUILD_DIR/lingtu_slam.o"
"$CXX" -std=c++17 -O2 \
  -I"$BUILD_DIR" \
  -I"$ROOT/src" \
  "$ROOT/scripts/diagnostics/native/dds_probe.cpp" \
  "$BUILD_DIR/lingtu_slam.o" \
  -lddsc \
  -o "$BUILD_DIR/lingtu_dds_probe"

echo "$BUILD_DIR/lingtu_dds_probe"
