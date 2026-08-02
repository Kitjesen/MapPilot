#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_NATIVE_RECORDING_BUILD_DIR:-$ROOT/build/native-recording}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
BUILD_DDS="${LINGTU_RECORDING_BUILD_DDS:-ON}"

case "$BUILD_DDS" in
  ON|OFF) ;;
  *)
    echo "ERROR: LINGTU_RECORDING_BUILD_DDS must be ON or OFF." >&2
    exit 2
    ;;
esac

if [ -n "${LINGTU_CYCLONEDDS_PREFIX:-}" ]; then
  export PATH="${LINGTU_CYCLONEDDS_PREFIX}/bin:${PATH}"
  export CMAKE_PREFIX_PATH="${LINGTU_CYCLONEDDS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${LINGTU_CYCLONEDDS_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

command -v cmake >/dev/null 2>&1 || {
  echo "ERROR: cmake is required." >&2
  exit 2
}
if [ "$BUILD_DDS" = ON ]; then
  command -v idlc >/dev/null 2>&1 || {
    echo "ERROR: CycloneDDS idlc is required when DDS tools are enabled; install cyclonedds-tools." >&2
    exit 2
  }
fi

cmake -S "$ROOT/src/native/recording" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DLINGTU_RECORDING_BUILD_DDS="$BUILD_DDS" \
  -DLINGTU_RECORDING_BUILD_TESTS=ON
cmake --build "$BUILD_DIR" --parallel "${BUILD_JOBS:-2}"
ctest --test-dir "$BUILD_DIR" --output-on-failure

printf '%s\n' \
  "$BUILD_DIR/lingtu_recorder" \
  "$BUILD_DIR/lingtu_camera_recorder" \
  "$BUILD_DIR/lingtu_camera_player"
if [ "$BUILD_DDS" = ON ]; then
  printf '%s\n' \
    "$BUILD_DIR/lingtu_dds_recorder" \
    "$BUILD_DIR/lingtu_dds_player"
fi
