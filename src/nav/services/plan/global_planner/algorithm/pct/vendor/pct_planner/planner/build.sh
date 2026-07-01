#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")"; pwd)
BUILD_LEGACY_NATIVE="${LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE:-0}"

if [[ "${BUILD_LEGACY_NATIVE}" != "1" ]]; then
  cat >&2 <<'EOF'
[pct-planner] planner/build.sh builds the legacy Linux/GTSAM native modules.
[pct-planner] Normal LingTu runtime uses the Rust gpmp_trajectory_optimizer process backend.
[pct-planner] Set LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=1 only for native/Rust parity baselines.
EOF
  exit 2
fi

cd "${ROOT_DIR}/lib"

mkdir -p build

cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release \
  -DLINGTU_PCT_BUILD_NATIVE_MODULES=ON \
  -DLINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=ON
make -j"${JOBS:-6}"
cp ./src/a_star/a_star*.so ../
cp ./src/trajectory_optimization/traj_opt*.so ../
cp ./src/ele_planner/ele_planner*.so ../
cp ./src/map_manager/py_map_manager*.so ../
cp ./src/common/smoothing/libcommon_smoothing.so ../
cd ..

# optional runtime hints for legacy native parity baselines
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${ROOT_DIR}/lib/3rdparty/gtsam-4.1.1/install/lib"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ROOT_DIR}/lib/build/src/common/smoothing"
export PYTHONPATH="${PYTHONPATH:-}:${ROOT_DIR}/lib"
# pybind11-stubgen -o ./ a_star
# pybind11-stubgen -o ./ traj_opt
# pybind11-stubgen -o ./ ele_planner
# pybind11-stubgen -o ./ py_map_manager
# cp ./a_star-stubs/__init__.pyi ./a_star.pyi
# cp ./traj_opt-stubs/__init__.pyi ./traj_opt.pyi
# cp ./ele_planner-stubs/__init__.pyi ./ele_planner.pyi
# cp ./py_map_manager-stubs/__init__.pyi ./py_map_manager.pyi
