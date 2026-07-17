#!/bin/bash
# build.sh — Build livox_calib_standalone (local or Docker)
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

# ── Option 1: Local build (requires PCL, OpenCV, Ceres, Eigen installed) ──
if [ "$1" = "local" ]; then
    mkdir -p "$SCRIPT_DIR/build"
    cd "$SCRIPT_DIR/build"
    cmake .. -DCMAKE_BUILD_TYPE=Release
    cmake --build . -j$(nproc)
    echo "Build complete: $SCRIPT_DIR/build/livox_calib"
    exit 0
fi

# ── Option 2: Docker build ──
echo "Building with Docker..."
# Docker context = parent dir so we can access both livox_camera_calib and livox_calib_standalone
docker build \
    -f "$SCRIPT_DIR/Dockerfile" \
    -t livox_calib_standalone \
    "$PARENT_DIR" 2>&1

echo ""
echo "Build successful! Usage:"
echo "  docker run --rm -v /path/to/data:/data livox_calib_standalone"
echo "  # Extract binary:"
echo "  docker create --name tmp_calib livox_calib_standalone"
echo "  docker cp tmp_calib:/src/livox_calib_standalone/build/livox_calib ./livox_calib"
echo "  docker rm tmp_calib"
