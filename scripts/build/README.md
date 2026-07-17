# Build helper scripts

This folder contains build-time helpers. Runtime profile launches live in
`lingtu.py`; robot deployment and service installation live under
`scripts/deploy/` and `scripts/ota/`.

Run commands from the repository root unless a script says otherwise.

| Script | Purpose |
|--------|---------|
| `build_ros_workspace.sh` | Build the ROS 2 compatibility workspace with `colcon`; skips legacy PCT ROS packages by default. |
| `build_rust_kernels.py` | Build portable Rust kernels, including PCT, pose-graph, and camera-LiDAR optimizer artifacts. |
| `build_vendored_pcl.sh` | Build a repo-local PCL install for either the OctoPlanner3D converter subset or the broader SLAM-native profile. |
| `build_3d_bbs.sh` | Build the CPU-only 3D-BBS global relocalization library under `third_party/3d_bbs/install`. |
| `build_octoplanner3d.sh` | Build the optional OctoPlanner3D headless global-planner executable. |
| `clone_orbbec_ros2.sh` | Restore the local OrbbecSDK ROS2 driver source under `src/drivers/real/camera/deps/orbbec/OrbbecSDK_ROS2`. |
| `build_orbbec_native.sh` | Build the no-ROS RGB-D stream executable from LingTu camera service SDK + Orbbec adapter. |
| `build_livox_sdk2_stream.sh` | Build the no-ROS Livox SDK2 stream executable used by `LidarModule(transport="sdk2")`. |
| `build_pointcloud_codec.sh` / `build_pointcloud_codec.ps1` | Build the C++ PCLD encoder used by Gateway live point-cloud WebSocket frames. |
| `build_slam_core.sh` | Build the C++ SLAM core, optional Python `_native` runner, and optional C++ DDS runtime. Uses `LINGTU_SLAM_FASTLIO2=ON` by default; set `OFF` only for contract tests. |
| `build_nav_endpoint.sh` | Build the no-ROS C++ CycloneDDS navigation endpoint that bridges DDS goal/cancel/instruction and Gateway path/cmd output. |
| `build_driver.sh` | Build and test the no-ROS C++ Thunder `driver` that consumes `rt/nav/cmd_vel` and calls Brainstem `Walk`. |
| `build_native_runtime.sh` | Build `lt_native`, `lt_pgo`, `lt_hba`, and read-only `lt_loop_verify`; use `--install-user-bin` on robots to expose the commands. |
| `build_prune.sh` | Build LingTu's clean-room C++ saved-map cleaner plus the ERASOR2 staging adapter. |
| `build_map_cleaning.sh` | Compatibility wrapper for `build_prune.sh`. |
| `build_erasor2_stage.sh` | Build LingTu's no-ROS C++ ERASOR2 staging adapter for saved-map cleanup experiments. |
| `fetch_erasor2.sh` | Fetch the pinned GPLv3 ERASOR2 checkout under `third_party/research_nav/ERASOR2` for optional robot-side tests. |

## Orbbec camera build

The preferred dependency shape is the pure Orbbec SDK under:

```text
src/drivers/real/camera/deps/orbbec/OrbbecSDK
```

or an installed SDK selected with:

```bash
export LINGTU_ORBBEC_SDK_ROOT=/path/to/OrbbecSDK
```

The current ROS2 vendor package remains supported only as a fallback because it
contains `orbbec_camera/SDK`.

## Native Gateway point-cloud build

Build the native Gateway point-cloud encoder when the browser live-cloud path
must avoid Python-side packing:

```bash
bash scripts/build/build_pointcloud_codec.sh
```

On Windows:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/build/build_pointcloud_codec.ps1
```

The Python `runtime.utils.binary_codec` wrapper automatically loads the shared
library from `src/kernels/gateway/pointcloud_codec/build` or from
`LINGTU_POINTCLOUD_CODEC_LIB`.

## Native SLAM build

Fast-LIO2 is built through `scripts/build/build_slam_core.sh`; it does not need
ROS when `src/localization/slam/cpp` is used. It does need normal C++ libraries:
Eigen3, PCL, and yaml-cpp. The script now checks those before CMake so a
robot image fails with a clear dependency message instead of a half-built SLAM
tree.

Global saved-map relocalization additionally needs the CPU 3D-BBS library. Build
it once before the SLAM core when no initial pose should be required:

```bash
bash scripts/build/build_3d_bbs.sh
LINGTU_REQUIRE_BBS3D=ON bash scripts/build/build_slam_core.sh
```

The default install prefix is `third_party/3d_bbs/install`, and
`build_slam_core.sh` passes that as `CPU_BBS3D_ROOT` automatically.

```bash
bash scripts/build/build_slam_core.sh
```

For contract-only CI without the real Fast-LIO2 algorithm:

```bash
LINGTU_SLAM_FASTLIO2=OFF bash scripts/build/build_slam_core.sh
```

For the process-split LiDAR/IMU -> SLAM hot path, build the C++ DDS runtime and
skip Python bindings:

```bash
LINGTU_SLAM_BUILD_CYCLONE_DDS_RUNTIME=ON \
LINGTU_SLAM_BUILD_PYTHON_BINDINGS=OFF \
bash scripts/build/build_slam_core.sh
```

That produces `build/slam_core/lingtu_slam_cyclone_runtime`. The runtime uses
CycloneDDS C++ plus `src/message/idl/lingtu_slam.idl`; it does not link ROS 2,
`rclcpp`, `sensor_msgs`, or `livox_ros_driver2`.

To publish real MID-360 SDK2 scans directly onto the native DDS topics,
build the Livox stream with DDS enabled:

```bash
LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS=ON \
bash scripts/build/build_livox_sdk2_stream.sh
```

Run it with `--dds --domain-id <N> --publish-freq 10` so it publishes
scan-level `rt/lidar/raw_frame` as `lingtu.dds.LivoxFrame`, diagnostic
packet-level `rt/lidar/raw_packet`, and `rt/imu/raw` as `lingtu.dds.Imu`.

Build the native navigation DDS boundary:

```bash
bash scripts/build/build_nav_endpoint.sh
```

That produces `build/nav_endpoint/lingtu_nav_native_endpoint`. It subscribes
to `rt/slam/odometry`, `rt/slam/registered_cloud`, `rt/nav/goal_pose`,
`rt/nav/global_path`, and `rt/nav/cancel`. With `LINGTU_ACTIVE_OCTOMAP` set,
goals call OctoPlanner3D in-process, then the C++ navigation loop publishes
`rt/nav/global_path`, `rt/nav/local_path`, and `rt/nav/cmd_vel`. The older
Gateway-polling `lingtu_nav_cyclone_endpoint` target was removed; use
`lingtu_nav_native_endpoint` for robot-side navigation.

Build the native Thunder motion driver:

```bash
bash scripts/build/build_driver.sh
```

This produces `build/driver/lingtu_driver` and runs both the fail-closed safety
core test and a typed DDS to Brainstem gRPC integration test. It uses the
existing product dependencies `cyclonedds-dev`, `cyclonedds-tools`,
`libgrpc++-dev`, `libprotobuf-dev`, and `protobuf-compiler-grpc`.

The older `LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME=ON` target produces
`lingtu_slam_dds_runtime`, which is C++ but still uses `rclcpp` and ROS 2
message headers. Keep it only for compatibility tests.

## Native runtime commands

Build the no-ROS runtime command helpers used by product map-save
optimization:

```bash
bash scripts/build/build_native_runtime.sh
```

On a robot image, install command symlinks into `~/.local/bin` so
`lingtu.service` can discover `lt_pgo` and `lt_hba` through its normal PATH:

```bash
bash scripts/build/build_native_runtime.sh --install-user-bin
```

`lt_pgo` is the default save-time optimizer. `lt_hba` is available when the
save request explicitly asks for the higher-density HBA variant.

## Saved-map cleaning

The product direction is LingTu's clean-room native C++ cleaner:

```bash
bash scripts/build/build_prune.sh
```

That produces `build/prune/prune`. It reads
`map.pcd + patches/*.pcd + poses.txt`, removes low-support dynamic ghost
candidates into `map.removed.pcd`, writes `map.clean.pcd`, and can apply the
cleaned map in place with:

```bash
build/prune/prune \
  --map-dir /home/sunrise/data/nova/maps/<map> \
  --overwrite \
  --apply
```

`--apply` backs up the original as `map.pcd.preclean` before replacing
`map.pcd`. `maps.prune.runtime.apply_dynamic_filter_step1half()` uses this
native command by default; set `LINGTU_MAP_CLEANER=dufomap` only for legacy
comparison.

ERASOR2 is GPLv3 and stays in `third_party/research_nav/ERASOR2` as reference
or as a separately run optional tool. `build_prune.sh` also builds
`erasor2_stage` for experiments, but that path is not the product
default.

Robot-side ERASOR2 optional backend check:

```bash
bash scripts/diagnostics/native/prune_check.sh
bash scripts/diagnostics/native/prune_check.sh --fetch --build
```

The first command proves the default LingTu cleaner and ERASOR2 staging adapter
on the robot. The second command also fetches the GPLv3 upstream checkout and
attempts the optional ERASOR2 backend build. Missing Eigen/PCL/OpenCV/yaml-cpp
is reported as a structured blocker.

For S100P, the optional ERASOR2 build uses LingTu's no-op `rerun_sdk` stub by
default. This avoids a robot-side GitHub download during CMake configure. Set
`LINGTU_ERASOR2_USE_RERUN_STUB=OFF` only for a visualization build.

Related root-level helpers:

| Script | Purpose |
|--------|---------|
| `scripts/build/build_nav_kernel.sh` | Build the Python `lingtu_nav_kernel` nanobind extension without a full ROS 2 workspace. |
| `scripts/build/build_dufomap.sh` | Build the DUFOMap cleanup binary used by map-save filtering. |

## OctoPlanner3D PCD converter build on WSL/Linux

There are two separate native PCL scopes:

- `octoplanner3d-pcd-converter`: PCL `common`/`io`/`octree` plus the
  upstream OctoMap libraries used by `octoplanner3d_headless` to convert
  `.pcd -> .bt`. This is **not** a full SLAM PCL pack and does not imply
  Fast-LIO2/localizer/PGO support.
- `slam-native`: broader PCL components for SLAM/native packages. Use it only
  when that larger surface is intentionally being built.

Exact WSL sequence used for the OctoPlanner3D converter subset:

```bash
# From the repository root inside WSL/Linux, for example:
#   cd /mnt/d/inovxio/brain/lingtu

export LINGTU_PCL_PROFILE=octoplanner3d-pcd-converter
export LINGTU_PCL_VERSION=pcl-1.14.1
export LINGTU_PCL_PREFIX="$PWD/third_party/install/pcl-1.14.1-wsl-io2"
export LINGTU_PCL_BUILD_DIR="$PWD/third_party/build/pcl-1.14.1-wsl-io2"
export LINGTU_BUILD_JOBS="${LINGTU_BUILD_JOBS:-$(nproc)}"

bash scripts/build/build_vendored_pcl.sh --profile octoplanner3d-pcd-converter
source "$LINGTU_PCL_PREFIX/lingtu-pcl-env.sh"

export LINGTU_OCTOPLANNER3D_SOURCE_DIR="$PWD/third_party/OctoPlanner3D-ROS2"
export LINGTU_OCTOPLANNER3D_BUILD_DIR="$PWD/build/octoplanner3d_headless_wsl_pcl"

bash scripts/build/build_octoplanner3d.sh --require-pcl
export LINGTU_OCTOPLANNER3D_EXECUTABLE="$LINGTU_OCTOPLANNER3D_BUILD_DIR/octoplanner3d_headless"
```

For `.bt` OctoMap input only, PCL is not required; build the default headless
wrapper without `--require-pcl`. Windows/MuJoCo paths should consume `.bt`
artifacts and should not require PCL/ROS 2 by default.

Lightweight diagnostics after a WSL converter build:

```bash
printf 'PCL_DIR=%s\nCMAKE_PREFIX_PATH=%s\n' "$PCL_DIR" "$CMAKE_PREFIX_PATH"
bash scripts/build/build_octoplanner3d.sh --diagnose
ldd "$LINGTU_OCTOPLANNER3D_BUILD_DIR/octoplanner3d_headless" | grep -E 'pcl|octomap'
```

Expected diagnostic hits include repo-local `libpcl_common`, `libpcl_io`,
`libpcl_octree`, and OctoPlanner3D's `liboctomap`/`liboctomath` libraries. If
the converter build was requested but those PCL cache entries are missing,
check that `lingtu-pcl-env.sh` was sourced and that `PCL_DIR` points under the
same `LINGTU_PCL_PREFIX` used to build PCL.

## Rust Kernel Build

Build the portable Rust kernels from any host with Rust installed:

```powershell
python scripts\build\build_rust_kernels.py --release
```

For the PCT Rust runtime only:

```powershell
python scripts\build\build_rust_kernels.py --target gpmp_trajectory_optimizer --release
```

For the camera-LiDAR Rust calibration optimizer only:

```powershell
python scripts\build\build_rust_kernels.py --target camera_lidar_optimizer --release
```

`build_ros_workspace.sh` builds `gpmp_trajectory_optimizer` and
`camera_lidar_optimizer` by default before the ROS compatibility workspace. That
gives PCT a ready `gpmp_optimize` runtime binary and keeps the camera-LiDAR
default build on the Rust-required optimizer path. It skips legacy ROS PCT
packages unless `LINGTU_BUILD_LEGACY_PCT_ROS=1`. Set
`LINGTU_BUILD_RUST_KERNELS=0` only when explicitly testing a legacy native
comparison path.
