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
| `clone_orbbec_ros2.sh` | Restore the local OrbbecSDK ROS2 driver source under `src/drivers/real/camera/OrbbecSDK_ROS2`. |
| `build_orbbec_native.sh` | Build the no-ROS Orbbec SDK RGB-D stream executable used by `OrbbecNativeCameraModule`. |
| `build_livox_sdk2_stream.sh` | Build the no-ROS Livox SDK2 stream executable used by `LidarModule(transport="sdk2")`. |
| `build_slam_core.sh` | Build the C++ SLAM core, optional Python `_native` runner, and optional C++ DDS runtime. Uses `LINGTU_SLAM_FASTLIO2=ON` by default; set `OFF` only for contract tests. |
| `build_nav_endpoint.sh` | Build the no-ROS C++ CycloneDDS navigation endpoint that bridges DDS goal/cancel/instruction and Gateway path/cmd output. |

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

The older `LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME=ON` target produces
`lingtu_slam_dds_runtime`, which is C++ but still uses `rclcpp` and ROS 2
message headers. Keep it only for compatibility tests.

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
