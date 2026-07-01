#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../../../../../.." && pwd)"

pct_planner_DIR=""
for candidate in \
  "${REPO_ROOT}/src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner" \
  "${REPO_ROOT}/src/nav/services/plan/global_planner/algorithm/pct/vendor/PCT_planner/planner"; do
  if [[ -f "${candidate}/lib/CMakeLists.txt" ]]; then
    pct_planner_DIR="${candidate}"
    break
  fi
done

if [[ -z "${pct_planner_DIR}" ]]; then
  echo "PCT planner native source directory not found under src/nav/services/plan/global_planner/algorithm/pct/vendor" >&2
  exit 2
fi

LIB_ROOT="${pct_planner_DIR}/lib"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"
PCT_CPU_ARCH_FLAGS="${PCT_CPU_ARCH_FLAGS:--march=x86-64-v2}"
BUILD_LEGACY_NATIVE="${LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE:-0}"

if [[ "${BUILD_LEGACY_NATIVE}" != "1" ]]; then
  cat >&2 <<'EOF'
[pct-runtime] This helper builds the legacy Linux/GTSAM native PCT modules.
[pct-runtime] LingTu now uses the Rust gpmp_trajectory_optimizer runtime by default.
[pct-runtime] Set LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=1 only for native/Rust parity baselines.
EOF
  exit 2
fi

ARCH="$(uname -m)"
case "${ARCH}" in
  x86_64|amd64) CANONICAL_ARCH="x86_64" ;;
  aarch64|arm64) CANONICAL_ARCH="aarch64" ;;
  *) CANONICAL_ARCH="${ARCH}" ;;
esac

PY_ABI="$(python3 - <<'PY'
import sys
print(f"{sys.version_info.major}{sys.version_info.minor}")
PY
)"

OUT_DIR="${SCRIPT_DIR}/native/${CANONICAL_ARCH}"
BUILD_DIR="${LIB_ROOT}/build_runnable_${CANONICAL_ARCH}_py${PY_ABI}"
GTSAM_DIR="${LIB_ROOT}/3rdparty/gtsam-4.1.1"
OSQP_DIR="${LIB_ROOT}/3rdparty/osqp"
GTSAM_INSTALL="${GTSAM_DIR}/install"
OSQP_INSTALL="${OSQP_DIR}/install"

if [[ "${CANONICAL_ARCH}" != "x86_64" ]]; then
  echo "This helper is intended for host x86_64 builds. Detected: ${CANONICAL_ARCH}" >&2
  exit 2
fi

echo "[pct-runtime] repo=${REPO_ROOT}"
echo "[pct-runtime] source=${pct_planner_DIR}"
echo "[pct-runtime] python=cp${PY_ABI} arch=${CANONICAL_ARCH}"
echo "[pct-runtime] cpu_flags=${PCT_CPU_ARCH_FLAGS}"

if [[ ! -d "${GTSAM_INSTALL}/lib/cmake/GTSAM" ]]; then
  echo "[pct-runtime] building GTSAM into ${GTSAM_INSTALL}"
  rm -rf "${GTSAM_DIR}/build" "${GTSAM_INSTALL}"
  cmake -S "${GTSAM_DIR}" -B "${GTSAM_DIR}/build" \
    -DCMAKE_INSTALL_PREFIX="${GTSAM_INSTALL}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
  cmake --build "${GTSAM_DIR}/build" -j "${JOBS}"
  cmake --install "${GTSAM_DIR}/build"
fi

if [[ ! -d "${OSQP_INSTALL}" ]]; then
  echo "[pct-runtime] building OSQP into ${OSQP_INSTALL}"
  rm -rf "${OSQP_DIR}/build" "${OSQP_INSTALL}"
  cmake -S "${OSQP_DIR}" -B "${OSQP_DIR}/build" \
    -DCMAKE_INSTALL_PREFIX="${OSQP_INSTALL}" \
    -DCMAKE_BUILD_TYPE=Release
  cmake --build "${OSQP_DIR}/build" -j "${JOBS}"
  cmake --install "${OSQP_DIR}/build"
fi

echo "[pct-runtime] building PCT native modules"
rm -rf "${BUILD_DIR}"
cmake -S "${LIB_ROOT}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPCT_CPU_ARCH_FLAGS="${PCT_CPU_ARCH_FLAGS}" \
  -DLINGTU_PCT_BUILD_NATIVE_MODULES=ON \
  -DLINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=ON \
  -DCMAKE_PREFIX_PATH="${GTSAM_INSTALL};${OSQP_INSTALL}" \
  -DGTSAM_DIR="${GTSAM_INSTALL}/lib/cmake/GTSAM"
cmake --build "${BUILD_DIR}" -j "${JOBS}"

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"

cp "${BUILD_DIR}"/src/a_star/a_star*.so "${OUT_DIR}/"
cp "${BUILD_DIR}"/src/trajectory_optimization/traj_opt*.so "${OUT_DIR}/"
cp "${BUILD_DIR}"/src/ele_planner/ele_planner*.so "${OUT_DIR}/"
cp "${BUILD_DIR}"/src/map_manager/py_map_manager*.so "${OUT_DIR}/"
cp "${BUILD_DIR}"/src/a_star/liba_star_search.so "${OUT_DIR}/"
cp "${BUILD_DIR}"/src/trajectory_optimization/libgpmp_optimizer.so "${OUT_DIR}/"
cp "${BUILD_DIR}"/src/ele_planner/libele_planner_lib.so "${OUT_DIR}/"
cp "${BUILD_DIR}"/src/map_manager/libmap_manager.so "${OUT_DIR}/"
cp "${BUILD_DIR}"/src/common/smoothing/libcommon_smoothing.so "${OUT_DIR}/"

if compgen -G "${GTSAM_INSTALL}/lib/libgtsam.so*" >/dev/null; then
  cp -a "${GTSAM_INSTALL}"/lib/libgtsam.so* "${OUT_DIR}/"
fi
if compgen -G "${GTSAM_INSTALL}/lib/libmetis-gtsam.so*" >/dev/null; then
  cp -a "${GTSAM_INSTALL}"/lib/libmetis-gtsam.so* "${OUT_DIR}/"
fi

if command -v patchelf >/dev/null 2>&1; then
  for so in "${OUT_DIR}"/*.so*; do
    [[ -f "${so}" || -L "${so}" ]] || continue
    patchelf --set-rpath '$ORIGIN' "${so}" 2>/dev/null || true
  done
fi

echo "[pct-runtime] wrote ${OUT_DIR}"
ls -la "${OUT_DIR}"
