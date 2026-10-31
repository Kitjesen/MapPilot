#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_CAMERA_VIDEO_BUILD_DIR:-${ROOT}/build/camera_video}"
RUNTIME_PREFIX="${LINGTU_VIDEO_RUNTIME_PREFIX:-${ROOT}/build/deps/video/prefix}"
if [[ ! -r "${RUNTIME_PREFIX}/env.sh" || ! -r "${RUNTIME_PREFIX}/runtime-arch" ]]; then
  echo "Prepared camera-video runtime is missing under ${RUNTIME_PREFIX}." >&2
  echo "Run scripts/build/prepare_camera_video.sh explicitly before make build." >&2
  exit 2
fi
case "${LINGTU_VIDEO_TARGET_ARCH:-$(uname -m)}" in
  amd64|x86_64) TARGET_ARCH=x86_64 ;;
  arm64|aarch64) TARGET_ARCH=aarch64 ;;
  *) TARGET_ARCH="${LINGTU_VIDEO_TARGET_ARCH:-$(uname -m)}" ;;
esac
RUNTIME_ARCH="$(tr -d '[:space:]' < "${RUNTIME_PREFIX}/runtime-arch")"
if [[ "${RUNTIME_ARCH}" != "${TARGET_ARCH}" ]]; then
  echo "Camera-video runtime architecture ${RUNTIME_ARCH} does not match target ${TARGET_ARCH}." >&2
  exit 2
fi
# shellcheck disable=SC1090
source "${RUNTIME_PREFIX}/env.sh"
RUNTIME_MULTIARCH="$(gcc -print-multiarch 2>/dev/null || true)"
if [[ -z "${RUNTIME_MULTIARCH}" || ! -d "${RUNTIME_PREFIX}/lib/${RUNTIME_MULTIARCH}" ]]; then
  echo "Prepared camera-video runtime has no native compiler libdir under ${RUNTIME_PREFIX}." >&2
  exit 2
fi
cmake -S "${ROOT}/src/drivers/real/camera/native/video" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" -DBUILD_TESTING=OFF \
  -DLINGTU_VIDEO_RUNTIME_PREFIX="${RUNTIME_PREFIX}" \
  -DLINGTU_VIDEO_RUNTIME_MULTIARCH="${RUNTIME_MULTIARCH}"
cmake --build "${BUILD_DIR}" --target lingtu_camera_webrtc lingtu_camera_video \
  --parallel "${LINGTU_BUILD_JOBS:-$(nproc)}"
echo "${BUILD_DIR}/lingtu_camera_webrtc"
echo "${BUILD_DIR}/lingtu_camera_video"
