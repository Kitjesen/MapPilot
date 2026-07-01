# Build Guide

What to install on a fresh machine before LingTu can compile and start.

## Prerequisites

- Ubuntu 22.04 (aarch64 on the S100P; x86_64 supported for `dev` / `sim` /
  CI)
- ROS 2 Humble Desktop is optional. It is only required for legacy
  ROS-compatible SLAM/service packages; the default LingTu Python/C++ runtime,
  `nav_kernel`, and OctoPlanner3D builds do not need ROS2.
- 鈮?8 GB RAM, 鈮?4 CPU cores

`$NAV_DIR` below stands for the workspace root. On the S100P it is
typically `/home/sunrise/data/inovxio/lingtu` (kept reachable as
`~/data/SLAM/navigation` via symlink); on a dev machine it is wherever
the repo was cloned.

## System packages

```bash
sudo apt update && sudo apt install -y \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    python3-pip \
    git \
    cmake

pip3 install numpy scipy scikit-learn
```

Optional system packages:

| Package | Used by |
|---------|---------|
| `ros-humble-desktop`, `ros-humble-pcl-conversions`, `ros-humble-tf2-geometry-msgs` | Legacy ROS-compatible SLAM/services only |
| `libgrpc++-dev`, `protobuf-compiler-grpc` | `remote_monitoring` (C++ gRPC gateway) |
| `libssl-dev`, `libcurl4-openssl-dev` | OTA daemon |
| `ros-humble-joy` | Physical joystick teleop |

## Third-party C++ libraries

### 1. Sophus (Lie groups, required by SLAM)

```bash
cd ~
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j$(nproc) && sudo make install
```

### 2. GTSAM 4.1.1 (PCT global planner, required)

GTSAM is vendored under `src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/3rdparty/`.

```bash
cd $NAV_DIR/src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/3rdparty/gtsam-4.1.1
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc) && make install
```

Add the install prefix to `LD_LIBRARY_PATH`:

```bash
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$NAV_DIR/src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib" >> ~/.bashrc
source ~/.bashrc
```

### 3. Livox SDK2 (LiDAR driver, required only when using a Livox lidar)

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

### 4. libdatachannel (optional WebRTC video)

```bash
cd ~
git clone https://github.com/nicknsy/libdatachannel.git
cd libdatachannel
git submodule update --init --recursive
cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j$(nproc)
sudo cmake --install build
```

If `libdatachannel` is missing, `remote_monitoring` still builds but the
WebRTC code path is disabled.

### 5. DUFOMap (dynamic obstacle removal, optional)

The Lingtu helper script `scripts/build/build_dufomap.sh` builds the DUFOMap
binary used by the offline post-mapping cleaner; rerun it on first
install.

### 6. TARE exploration

The product `tare_explore` profile uses LingTu's in-process TARE policy and
does not require the CMU TARE source tree, OR-Tools, or `tare_planner_node`.
External CMU benchmark runs must provide their own CMU workspace.

## Build the runtime

### Product default: native planner kernels, no ROS2

```bash
cd $NAV_DIR
bash scripts/build/build_nav_kernel.sh --clean
bash scripts/build/build_octoplanner3d.sh
python -m pytest src/runtime/tests/test_terrain_local_planner_contract.py -q
python lingtu.py runtime-audit
```

`build_nav_kernel.sh` produces `_nav_kernel.so`, the nanobind backend used by
`LocalPlannerModule` and `PathFollowerModule`. `build_octoplanner3d.sh`
builds the headless C++ global-planner executable used by the product navigation
profiles.

### Native nav_kernel convenience target

```bash
make nav_kernel
```

### Subsets

```bash
# Local autonomy kernel without ROS2
bash scripts/build/build_nav_kernel.sh --clean

# Product global planner only, no ROS2 workspace needed
bash scripts/build/build_octoplanner3d.sh

# ROS compatibility workspace only
bash scripts/build/build_ros_workspace.sh
```

`make build` remains a compatibility shortcut for the ROS/colcon workspace.
Do not use it as the default product build unless you are intentionally working
on ROS-backed SLAM/service packages.

## Tests

```bash
# Framework tests 鈥?Python only, no ROS2, no hardware
python -m pytest src/runtime/tests/ -q

# ROS compatibility tests
make test

# Standalone C++ nav_kernel tests
cd src/nav/kernel && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
./test_path_follower_core
./test_benchmark

# Standalone C++ local planner tests
cd ../../services/plan/local_planner/cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release -DLOCAL_PLANNER_CPP_BUILD_TESTS=ON
cmake --build build -j$(nproc)
./build/test_local_planner_core
```

## Verifying the build

```bash
# Product global planner wrapper is available
python3 -c "from nav.services.plan.global_planner.backends.octoplanner3d.backend import OctoPlanner3DBackend; print('Success')"

# Optional ROS2 compatibility nodes are discoverable after the ROS workspace build
ros2 pkg list | grep -E "fastlio2|remote_monitoring"

# `lingtu` CLI is on PATH after `pip install -e .`
lingtu --version
lingtu --list

# Stub profile builds the full Module graph without hardware
python lingtu.py stub
```

## Common errors

| Symptom | Fix |
|---------|-----|
| `GTSAM not found` | `ls $NAV_DIR/src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4` and re-run the install step |
| `libgtsam.so.4: cannot open shared object file` | `LD_LIBRARY_PATH` is missing the GTSAM install prefix; re-source `~/.bashrc` |
| `ModuleNotFoundError: planner_py` | PCT legacy compatibility was not built; prefer OctoPlanner3D for product runtime, or run the explicit PCT compatibility setup before parity tests. |
| `Livox SDK2 not found` | `ls /usr/local/lib/liblivox_lidar_sdk_shared.so`, rerun the SDK install |
| Sophus complains about `fmt` | re-run `cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON` and rebuild |
| `ele_planner.so` import fails on x86 | Expected for legacy PCT native binaries. Use the default `octoplanner3d` backend on dev machines, or `--planner pct` only when validating PCT compatibility. |

## Approximate build times

| Package | Dependencies | Time |
|---------|--------------|------|
| `nav_kernel` | Eigen, nanobind | ~30 s |
| `octoplanner3d` | CMake, OctoMap artifact | ~1 min |
| `pct_planner` compatibility | GTSAM, pybind11 | ~1 min |
| `remote_monitoring` | gRPC, libdatachannel (optional) | ~1 min |
| `fastlio2` | Sophus, Livox SDK2 | ~2 min |
| GTSAM 4.1.1 | Boost, Eigen | ~10 min (one-time) |
| `ota_daemon` | gRPC, yaml-cpp, OpenSSL, CURL | ~30 s |

## What this guide deliberately does **not** cover

- `navigation_run.launch.py`, `navigation_bringup.launch.py`,
  `launch/subsystems/`, `scripts/legacy/`, `scripts/services/`, and any
  systemd service installer that referred to those paths 鈥?they were
  deleted and are no longer relevant.
- A separate "deploy" path. Today the workflow is
  `make build && lingtu <profile>` (optionally `--daemon`). On the S100P
  the `scripts/lingtu` shell wrapper is the operator's CLI; see
  `docs/archive/AGENTS.md` and `docs/01-getting-started/`.
