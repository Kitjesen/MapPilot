# Native build entrypoints

Run these commands from the repository root. Compilation outputs go under
`build/`. Native release packaging installs them into the ignored standard
prefix `install/linux-<arch>/<config>/{bin,lib,etc,share}` before assembling
`dist/`; Product startup never resolves a `build/` path.

On Windows x64, use the same CMake install rules with
`--prefix install/windows-x64/Release`. The Linux OTA packager does not package
Windows binaries.

## Field runtime

| Target | Command |
| --- | --- |
| Robot driver | `bash scripts/build/build_driver.sh` |
| SLAM | `bash scripts/build/build_slam_core.sh` |
| Map daemon | `bash scripts/build/build_mapd.sh` |
| Navigation endpoint | `bash scripts/build/build_nav_endpoint.sh` |
| Exploration Python binding | `bash scripts/build/build_explore_py.sh` |
| Native DDS/MCAP recording | `bash scripts/build/build_native_recording.sh` |
| Native map cleaner | `bash scripts/build/build_prune.sh` |

Select a driver for a target image with
`LINGTU_DRIVER_BACKEND=go2` or `LINGTU_DRIVER_BACKEND=doso`.

## Sensors and Gateway

| Target | Command |
| --- | --- |
| Livox SDK2 stream | `bash scripts/build/build_livox_sdk2_stream.sh` |
| Orbbec native stream | `bash scripts/build/build_orbbec_native.sh` |
| Camera DDS endpoint | `bash scripts/build/build_camera_dds.sh` |
| GNSS DDS endpoint | `bash scripts/build/build_gnss_dds.sh` |
| Gateway point-cloud codec | `cmake -S src/kernels/gateway/pointcloud_codec -B src/kernels/gateway/pointcloud_codec/build && cmake --build src/kernels/gateway/pointcloud_codec/build` |

On Windows, configure the same point-cloud codec source with a Visual Studio
generator, then run `cmake --build src/kernels/gateway/pointcloud_codec/build --config Release`.

## Windows native simulation

- `prepare_cyclonedds_windows.ps1`
- `prepare_slam_dependencies_windows.ps1`
- `build_slam_core_windows.ps1`
- `build_mujoco_native_dds_windows.ps1`
- `verify_cyclonedds_windows_sdk.ps1`

The files under `cmake/`, `locks/`, `vcpkg/`, and `provenance/` are inputs to
that release build. They are not Product runtime state.

## Native planner builds

Product default: native planner kernels, no ROS2.

| Target | Command |
| --- | --- |
| OctoPlanner3D | `bash scripts/build/build_octoplanner3d.sh` |
| Optional vendored PCL | `bash scripts/build/build_vendored_pcl.sh` |

ROS 2 Humble Desktop is optional and belongs only to an explicit compatibility
or algorithm-validation path.

## Optional research builds

- `fetch_erasor2.sh` restores the optional map-cleaning comparison source used
  by `build_prune.sh`.

`fetch_orbbec_sdk.sh` fetches the field-tested standalone Orbbec SDK v2.8.7
into ignored `build/deps/orbbec-sdk/`; the native camera does not use a ROS2
wrapper.

Product startup never runs build scripts. ProductControl consumes installed
artifacts through the resolved RunPlan.
