#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_DDS_PROBE_BUILD_DIR:-$ROOT/build/dds_probe}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"

if [ -n "${LINGTU_CYCLONEDDS_PREFIX:-}" ]; then
  export CMAKE_PREFIX_PATH="${LINGTU_CYCLONEDDS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${LINGTU_CYCLONEDDS_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

if [ -n "${CYCLONEDDS_IDLC_EXECUTABLE:-}" ]; then
  IDLC="${CYCLONEDDS_IDLC_EXECUTABLE}"
elif [ -n "${LINGTU_CYCLONEDDS_PREFIX:-}" ] && \
    [ -x "${LINGTU_CYCLONEDDS_PREFIX}/bin/idlc" ]; then
  IDLC="${LINGTU_CYCLONEDDS_PREFIX}/bin/idlc"
else
  IDLC="$(command -v idlc || true)"
fi
if [ -z "$IDLC" ]; then
  echo "ERROR: CycloneDDS idlc not found; install cyclonedds-tools." >&2
  exit 2
fi

IDLC_PREFIX="$(cd "$(dirname "$IDLC")/.." && pwd)"
IDLC_LIBRARY_PATH=""
for lib_dir in "${IDLC_PREFIX}/lib" "${IDLC_PREFIX}/lib64"; do
  if [ -d "${lib_dir}" ]; then
    IDLC_LIBRARY_PATH="${IDLC_LIBRARY_PATH:+${IDLC_LIBRARY_PATH}:}${lib_dir}"
  fi
done

DDS_INCLUDE_ARGS=()
DDS_LINK_ARGS=(-lddsc)
if [ -n "${LINGTU_CYCLONEDDS_PREFIX:-}" ]; then
  DDS_INCLUDE_ARGS=(-I"${LINGTU_CYCLONEDDS_PREFIX}/include")
  DDS_LINK_ARGS=(
    -L"${LINGTU_CYCLONEDDS_PREFIX}/lib"
    -Wl,-rpath,"${LINGTU_CYCLONEDDS_PREFIX}/lib"
    -lddsc)
fi

mkdir -p "$BUILD_DIR"
(
  cd "$BUILD_DIR"
  env LD_LIBRARY_PATH="${IDLC_LIBRARY_PATH}" \
    "$IDLC" -l c "$ROOT/src/message/idl/messages.idl"
)

CC="${CC:-cc}"
CXX="${CXX:-c++}"
"$CC" -std=c11 -O2 \
  "${DDS_INCLUDE_ARGS[@]}" \
  -I"$BUILD_DIR" \
  -c "$BUILD_DIR/messages.c" \
  -o "$BUILD_DIR/messages.o"
"$CXX" -std=c++17 -O2 \
  "${DDS_INCLUDE_ARGS[@]}" \
  -I"$BUILD_DIR" \
  -I"$ROOT/src" \
  "$ROOT/tools/diagnostics/dds_probe.cpp" \
  "$BUILD_DIR/messages.o" \
  "${DDS_LINK_ARGS[@]}" \
  -o "$BUILD_DIR/lingtu_dds_probe"

echo "$BUILD_DIR/lingtu_dds_probe"
