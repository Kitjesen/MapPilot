#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${LINGTU_ORBBEC_SDK_ROOT:-}" ]]; then
  SDK_ROOT="$LINGTU_ORBBEC_SDK_ROOT"
  SDK_SOURCE="configured"
elif [[ -d "src/drivers/real/camera/deps/orbbec/OrbbecSDK" ]]; then
  SDK_ROOT="src/drivers/real/camera/deps/orbbec/OrbbecSDK"
  SDK_SOURCE="pure_sdk"
else
  ORBBEC_ROS2_DIR="${LINGTU_ORBBEC_ROS2_DIR:-src/drivers/real/camera/deps/orbbec/OrbbecSDK_ROS2}"
  if [[ ! -d "$ORBBEC_ROS2_DIR" && -d "src/drivers/real/camera/OrbbecSDK_ROS2" ]]; then
    ORBBEC_ROS2_DIR="src/drivers/real/camera/OrbbecSDK_ROS2"
  fi
  SDK_ROOT="$ORBBEC_ROS2_DIR/orbbec_camera/SDK"
  SDK_SOURCE="ros2_wrapper_fallback"
fi

if [[ "$SDK_SOURCE" == "ros2_wrapper_fallback" ]]; then
  echo "[build_orbbec_native] using OrbbecSDK_ROS2 bundled SDK as compatibility fallback; set LINGTU_ORBBEC_SDK_ROOT or run scripts/build/fetch_orbbec_sdk.sh for the pure SDK path" >&2
fi

SRC_DIR="${LINGTU_ORBBEC_NATIVE_SOURCE_DIR:-src/drivers/real/camera/native}"
IMPL_DIR="${LINGTU_ORBBEC_IMPL_SOURCE_DIR:-src/drivers/real/camera/impl/orbbec}"
CAPTURE_SRC="${LINGTU_ORBBEC_NATIVE_CAPTURE_SOURCE:-$SRC_DIR/capture_process.cpp}"
CAMERA_SRC="${LINGTU_ORBBEC_NATIVE_CAMERA_SOURCE:-$IMPL_DIR/camera.cpp}"
SDK_SRC="${LINGTU_ORBBEC_NATIVE_SDK_SOURCE:-$SRC_DIR/sdk.cpp}"
OUT_DIR="${LINGTU_ORBBEC_NATIVE_BUILD_DIR:-build/orbbec_native}"

case "$(uname -m)" in
  aarch64|arm64) SDK_ARCH="arm64" ;;
  x86_64|amd64) SDK_ARCH="x64" ;;
  *) echo "unsupported architecture: $(uname -m)" >&2; exit 1 ;;
esac

mkdir -p "$OUT_DIR"
if [[ -d "$SDK_ROOT/lib/$SDK_ARCH" ]]; then
  SDK_LIB="$SDK_ROOT/lib/$SDK_ARCH"
elif [[ -d "$SDK_ROOT/lib" ]]; then
  SDK_LIB="$SDK_ROOT/lib"
elif [[ -d "$SDK_ROOT/lib64" ]]; then
  SDK_LIB="$SDK_ROOT/lib64"
else
  echo "missing Orbbec SDK library directory under: $SDK_ROOT" >&2
  exit 1
fi
RUNTIME_LIB="$OUT_DIR/lib"
mkdir -p "$RUNTIME_LIB"

install_orbbec_runtime_lib() {
  local libs=("$SDK_LIB"/libOrbbecSDK.so*)
  if [[ ! -e "${libs[0]}" ]]; then
    echo "missing Orbbec SDK runtime library under: $SDK_LIB" >&2
    exit 1
  fi

  cp -a "${libs[@]}" "$RUNTIME_LIB/"

  if [[ ! -e "$RUNTIME_LIB/libOrbbecSDK.so" ]]; then
    local primary
    primary="$(find "$RUNTIME_LIB" -maxdepth 1 -type f -name 'libOrbbecSDK.so*' | sort -V | tail -n 1)"
    if [[ -z "$primary" ]]; then
      echo "failed to install Orbbec SDK runtime library" >&2
      exit 1
    fi
    if ! ln -sfn "$(basename "$primary")" "$RUNTIME_LIB/libOrbbecSDK.so" 2>/dev/null; then
      cp -f "$primary" "$RUNTIME_LIB/libOrbbecSDK.so"
    fi
  fi
}

install_orbbec_runtime_lib

c++ -std=c++17 -O2 \
  -I"$SDK_ROOT/include" \
  -I"$SRC_DIR" \
  -I"$IMPL_DIR" \
  "$CAPTURE_SRC" \
  "$SDK_SRC" \
  "$CAMERA_SRC" \
  -L"$RUNTIME_LIB" \
  -Wl,-rpath,"$PWD/$RUNTIME_LIB" \
  -Wl,-rpath,"$PWD/$SDK_LIB" \
  -lOrbbecSDK \
  -o "$OUT_DIR/orbbec_capture"

LD_LIBRARY_PATH="$PWD/$RUNTIME_LIB:${LD_LIBRARY_PATH:-}" "$OUT_DIR/orbbec_capture" --self-test >/dev/null
echo "$OUT_DIR/orbbec_capture"
