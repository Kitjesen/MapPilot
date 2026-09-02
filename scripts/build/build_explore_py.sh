#!/usr/bin/env bash
# Build and install the lingtu_explore_kernel Python extension.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SOURCE_DIR="${ROOT}/src/explore/cpp"
BUILD_DIR="${SOURCE_DIR}/build_nb"
PYTHON="${PYTHON:-python3}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"

case "${1:-}" in
  "") ;;
  --clean) rm -rf -- "${BUILD_DIR}" ;;
  -h|--help)
    echo "Usage: bash scripts/build/build_explore_py.sh [--clean]"
    exit 0
    ;;
  *)
    echo "Unknown option: $1" >&2
    exit 2
    ;;
esac

command -v cmake >/dev/null 2>&1 || { echo "ERROR: cmake is required." >&2; exit 2; }
command -v "${PYTHON}" >/dev/null 2>&1 || { echo "ERROR: ${PYTHON} is required." >&2; exit 2; }
"${PYTHON}" - <<'PY'
import sysconfig
from pathlib import Path

header = Path(sysconfig.get_path("include") or "") / "Python.h"
if not header.is_file():
    raise SystemExit("ERROR: Python development headers are required.")
PY

GEN_DIR="${BUILD_DIR}/dds_generated"
MESSAGE_IDL="${ROOT}/src/message/idl/messages.idl"
if command -v idlc >/dev/null 2>&1; then
  mkdir -p "${GEN_DIR}"
  (cd "${GEN_DIR}" && idlc -l c "${MESSAGE_IDL}")
fi

DDS=OFF
if { command -v pkg-config >/dev/null 2>&1 && pkg-config --exists CycloneDDS; } || \
   [[ -n "${CycloneDDS_DIR:-}" ]]; then
  DDS=ON
fi

cmake -S "${SOURCE_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DLINGTU_DDS_GEN_DIR="${GEN_DIR}" \
  -DLINGTU_EXPLORE_CPP_BUILD_PYTHON_BINDINGS=ON \
  -DLINGTU_EXPLORE_CPP_WITH_DDS="${DDS}"
cmake --build "${BUILD_DIR}" --target lingtu_explore_kernel --parallel "${JOBS}"

MODULE="$(find "${BUILD_DIR}" -type f -name 'lingtu_explore_kernel*.so' -print -quit)"
if [[ -z "${MODULE}" ]]; then
  echo "ERROR: lingtu_explore_kernel shared library was not produced." >&2
  exit 1
fi
find "${ROOT}/src" -maxdepth 1 \( -type f -o -type l \) \
  -name 'lingtu_explore_kernel*.so' -delete
cp -f -- "${MODULE}" "${ROOT}/src/$(basename "${MODULE}")"

PYTHONPATH="${ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" "${PYTHON}" - <<'PY'
import lingtu_explore_kernel as kernel

assert kernel.TarePolicy
assert kernel.TarePolicyConfig
assert kernel.Grid2D
print(f"lingtu_explore_kernel ready (DDS={kernel.HAS_DDS})")
PY
