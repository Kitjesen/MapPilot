#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")"; pwd)
BUILD_LEGACY_NATIVE="${LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE:-0}"

if [[ "${BUILD_LEGACY_NATIVE}" != "1" ]]; then
  cat >&2 <<'EOF'
[pct-planner] build_thirdparty.sh builds legacy third-party native dependencies, including GTSAM.
[pct-planner] Normal LingTu runtime uses the Rust gpmp_trajectory_optimizer process backend.
[pct-planner] Set LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=1 only for native/Rust parity baselines.
EOF
  exit 2
fi

# build gtsam
cd "${ROOT_DIR}/lib/3rdparty/gtsam-4.1.1"
rm -rf build install
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j"${JOBS_GTSAM:-${JOBS:-6}}" && make install

# build osqp
cd "${ROOT_DIR}/lib/3rdparty/osqp"
rm -rf build install
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release
make -j"${JOBS_OSQP:-${JOBS:-4}}" && make install
