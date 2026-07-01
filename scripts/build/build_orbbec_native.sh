#!/usr/bin/env bash
set -euo pipefail

SDK_ROOT="${LINGTU_ORBBEC_ROS2_DIR:-src/drivers/real/camera/OrbbecSDK_ROS2}/orbbec_camera/SDK"
SRC_DIR="${LINGTU_ORBBEC_NATIVE_SOURCE_DIR:-src/drivers/real/camera/native}"
CAPTURE_SRC="${LINGTU_ORBBEC_NATIVE_CAPTURE_SOURCE:-$SRC_DIR/capture_process.cpp}"
CAMERA_SRC="${LINGTU_ORBBEC_NATIVE_CAMERA_SOURCE:-$SRC_DIR/camera.cpp}"
OUT_DIR="${LINGTU_ORBBEC_NATIVE_BUILD_DIR:-build/orbbec_native}"

case "$(uname -m)" in
  aarch64|arm64) SDK_ARCH="arm64" ;;
  x86_64|amd64) SDK_ARCH="x64" ;;
  *) echo "unsupported architecture: $(uname -m)" >&2; exit 1 ;;
esac

mkdir -p "$OUT_DIR"
SDK_LIB="$SDK_ROOT/lib/$SDK_ARCH"
RUNTIME_LIB="$OUT_DIR/lib"
mkdir -p "$RUNTIME_LIB"

install_orbbec_runtime_lib() {
  local versioned="$SDK_LIB/libOrbbecSDK.so.2.8.6"
  if [[ ! -f "$versioned" ]]; then
    echo "missing Orbbec SDK runtime library: $versioned" >&2
    exit 1
  fi

  cp -f "$versioned" "$RUNTIME_LIB/libOrbbecSDK.so.2.8.6"

  if ln -sfn "libOrbbecSDK.so.2.8.6" "$RUNTIME_LIB/libOrbbecSDK.so.2" 2>/dev/null; then
    ln -sfn "libOrbbecSDK.so.2" "$RUNTIME_LIB/libOrbbecSDK.so"
  else
    cp -f "$RUNTIME_LIB/libOrbbecSDK.so.2.8.6" "$RUNTIME_LIB/libOrbbecSDK.so.2"
    cp -f "$RUNTIME_LIB/libOrbbecSDK.so.2.8.6" "$RUNTIME_LIB/libOrbbecSDK.so"
  fi
}

install_orbbec_runtime_lib

c++ -std=c++17 -O2 \
  -I"$SDK_ROOT/include" \
  -I"$SRC_DIR" \
  "$CAPTURE_SRC" \
  "$CAMERA_SRC" \
  -L"$RUNTIME_LIB" \
  -Wl,-rpath,"$PWD/$RUNTIME_LIB" \
  -Wl,-rpath,"$PWD/$SDK_LIB" \
  -lOrbbecSDK \
  -o "$OUT_DIR/orbbec_capture"

LD_LIBRARY_PATH="$PWD/$RUNTIME_LIB:${LD_LIBRARY_PATH:-}" "$OUT_DIR/orbbec_capture" --self-test >/dev/null
echo "$OUT_DIR/orbbec_capture"
