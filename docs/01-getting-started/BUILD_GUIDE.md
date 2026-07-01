# Build Guide

What to install on a fresh machine before LingTu can compile and start.

## Prerequisites

- Ubuntu 22.04 (aarch64 on the S100P; x86_64 supported for `dev` / `sim` /
  CI)
- ROS 2 Humble Desktop (only required for the C++ stack 鈥?Python tests
  do not need it)
- 鈮?8 GB RAM, 鈮?4 CPU cores

`$NAV_DIR` below stands for the workspace root. On the S100P it is
typically `/home/sunrise/data/inovxio/lingtu` (kept reachable as
`~/data/SLAM/navigation` via symlink); on a dev machine it is wherever
the repo was cloned.

## System packages

```bash
sudo apt update && sudo apt install -y \
    ros-humble-desktop \
    ros-humble-pcl-conversions \
    ros-humble-tf2-geometry-msgs \
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

GTSAM is vendored under `src/global_planning/pct_planner/planner/lib/3rdparty/`.

```bash
cd $NAV_DIR/src/global_planning/pct_planner/planner/lib/3rdparty/gtsam-4.1.1
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc) && make install
```

Add the install prefix to `LD_LIBRARY_PATH`:

```bash
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$NAV_DIR/src/global_planning/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib" >> ~/.bashrc
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

### 6. TARE planner (only for `tare_explore` profile)

```bash
bash scripts/build/fetch_ortools.sh
bash scripts/build/build_tare.sh
```

`exploration("tare")` raises if the binary is not present, which makes a
broken setup visible immediately.

## Build the workspace

### Full build

```bash
cd $NAV_DIR
make build           # source ROS Humble + colcon build (Release)
source install/setup.bash
```

`make build` is `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
inside a ROS Humble shell. Use `make build-debug` for `-DCMAKE_BUILD_TYPE=Debug`.

### Native nav_core only (no ROS2 needed)

```bash
make nav_core
```

This drives `scripts/build/build_nav_core.sh` to produce `_nav_core.so` (the
nanobind backend used by `LocalPlannerModule` / `PathFollowerModule`).

### Subsets

```bash
# Base autonomy without SLAM
colcon build --packages-select \
    local_planner terrain_analysis terrain_analysis_ext visualization_tools

# Global planner only
colcon build --packages-select pct_planner pct_adapters

# Remote monitoring only
colcon build --packages-select remote_monitoring

# SLAM only
colcon build --packages-select fastlio2 hba pgo
```

## Tests

```bash
# Framework tests 鈥?Python only, no ROS2, no hardware
python -m pytest src/core/tests/ -q

# Colcon tests
make test

# Standalone C++ nav_core tests
cd src/nav/core && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
./test_local_planner_core
./test_path_follower_core
./test_benchmark
```

## Verifying the build

```bash
# C++ ROS2 nodes are discoverable
ros2 pkg list | grep -E "fastlio2|local_planner|pct_planner|remote_monitoring"

# PCT C++ Python wrapper imports
python3 -c "from planner_py import OfflineElePlanner; print('Success')"

# `lingtu` CLI is on PATH after `pip install -e .`
lingtu --version
lingtu --list

# Stub profile builds the full Module graph without hardware
python lingtu.py stub
```

## Common errors

| Symptom | Fix |
|---------|-----|
| `GTSAM not found` | `ls $NAV_DIR/src/global_planning/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4` and re-run the install step |
| `libgtsam.so.4: cannot open shared object file` | `LD_LIBRARY_PATH` is missing the GTSAM install prefix; re-source `~/.bashrc` |
| `ModuleNotFoundError: planner_py` | `colcon build --packages-select pct_planner` then `source install/setup.bash` |
| `Livox SDK2 not found` | `ls /usr/local/lib/liblivox_lidar_sdk_shared.so`, rerun the SDK install |
| Sophus complains about `fmt` | re-run `cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON` and rebuild |
| `tare_planner_node` missing on `tare_explore` start | Run `scripts/build/fetch_ortools.sh` and `scripts/build/build_tare.sh` |
| `ele_planner.so` import fails on x86 | Expected 鈥?the binary is aarch64-only. Use `--planner astar` on dev machines. |

## Approximate build times

| Package | Dependencies | Time |
|---------|--------------|------|
| `local_planner` | PCL, Eigen | ~30 s |
| `terrain_analysis` | PCL | ~20 s |
| `pct_planner` | GTSAM, pybind11 | ~1 min |
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
