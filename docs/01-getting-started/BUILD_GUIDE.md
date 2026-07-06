# Build Guide

This guide covers a fresh machine setup for LingTu. The default product path is
native Python/C++ plus DDS services. ROS 2 is optional and only needed for
explicit compatibility packages and replay checks.

## Prerequisites

- Ubuntu 22.04 on S100P/aarch64 for field deployment.
- x86_64 Linux or Windows development hosts for framework, docs, and some
  simulation work.
- Python 3.10.12, matching `.python-version`.
- At least 8 GB RAM and 4 CPU cores.

`$NAV_DIR` below means the repository root. On the robot it is usually:

```text
/home/sunrise/data/SLAM/navigation
```

## Python Environment

Preferred:

```bash
cd $NAV_DIR
uv sync --locked
uv run --locked python lingtu.py --list
```

Install optional extras only for the profiles you need:

```bash
uv sync --locked --extra dev
uv sync --locked --extra vision --extra ml --extra llm --extra nlp
uv sync --locked --extra perception --extra vector
```

## System Packages

Product/native build basics:

```bash
sudo apt update
sudo apt install -y \
  git \
  cmake \
  build-essential \
  python3-pip \
  libeigen3-dev \
  libboost-all-dev \
  libpcl-dev \
  libyaml-cpp-dev
```

Optional packages:

| Package | Used By |
| --- | --- |
| `ros-humble-desktop`, `ros-humble-pcl-conversions`, `ros-humble-tf2-geometry-msgs` | legacy ROS-compatible SLAM/services only |
| `cyclonedds-dev`, `cyclonedds-tools` | native DDS service builds and diagnostics |
| `libgrpc++-dev`, `protobuf-compiler-grpc` | gRPC compatibility/monitoring surfaces |
| `libssl-dev`, `libcurl4-openssl-dev` | OTA/service tooling |
| `ros-humble-joy` | ROS joystick compatibility checks |

## Third-Party Components

### Sophus

Required by SLAM/localization C++ code:

```bash
cd ~
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
cmake -B build -DSOPHUS_USE_BASIC_LOGGING=ON
cmake --build build -j
sudo cmake --install build
```

### Livox SDK2

Required only for real Livox hardware ingestion:

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
cmake -B build
cmake --build build -j
sudo cmake --install build
```

### DUFOMap

Optional save-time cleanup helper:

```bash
cd $NAV_DIR
bash scripts/build/build_dufomap.sh
```

### TARE

The product `tare_explore` profile uses LingTu's current exploration contract.
External CMU TARE benchmark runs must provide their own external workspace and
should be treated as compatibility/evaluation work.

## Product Build

Native navigation kernel:

```bash
cd $NAV_DIR
bash scripts/build/build_nav_kernel.sh --clean
```

OctoPlanner3D:

```bash
bash scripts/build/build_octoplanner3d.sh
```

Convenience target:

```bash
make nav_kernel
```

ROS compatibility workspace, only when needed:

```bash
bash scripts/build/build_ros_workspace.sh
```

Do not treat `make build` or a sourced ROS/colcon overlay as the default
product build. Use it only when working on ROS-backed compatibility packages.

## Tests

Framework tests, no ROS 2 or hardware:

```bash
python -m pytest src/runtime/tests/ -q
```

Focused product contract tests:

```bash
python -m pytest src/localization/tests/test_native_slam_contract.py -q
python -m pytest sim/tests/test_mujoco_saved_map_quality_gate.py -q
```

C++ nav kernel:

```bash
cd src/nav/kernel
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/test_path_follower_core
./build/test_benchmark
```

C++ local planner:

```bash
cd src/nav/services/plan/local_planner/cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release -DLOCAL_PLANNER_CPP_BUILD_TESTS=ON
cmake --build build -j
./build/test_local_planner_core
```

ROS compatibility tests:

```bash
make test
```

## Verification

```bash
python lingtu.py --list
python lingtu.py stub
python lingtu.py runtime-audit
```

On the robot:

```bash
bash scripts/lingtu status
bash scripts/lingtu svc status
```

## Common Errors

| Symptom | Fix |
| --- | --- |
| `ModuleNotFoundError: runtime` | Set `PYTHONPATH=src:.` or run through `uv run --locked`. |
| `Livox SDK2 not found` | Build/install Livox SDK2 and verify `/usr/local/lib/liblivox_lidar_sdk_shared.so`. |
| `Sophus`/`fmt` build errors | Rebuild Sophus with `-DSOPHUS_USE_BASIC_LOGGING=ON`. |
| `planner_py` missing | PCT compatibility was not built; prefer `octoplanner3d` for product runtime. |
| `_nav_kernel` import fails | Re-run `scripts/build/build_nav_kernel.sh --clean`. |
| `ros2: command not found` | Normal for product builds; only ROS compatibility work needs ROS 2. |

## Scope

This guide deliberately does not cover removed launch files, deleted service
installers, or legacy root-level navigation facades. Use `scripts/lingtu` for
field operations and the architecture documents under `docs/architecture/` for
runtime contracts.
