# Native build entrypoints

Run these commands from the repository root. Outputs go under `build/` unless a
script documents an explicit SDK or third-party prefix.

## Field runtime

| Target | Command |
| --- | --- |
| Robot driver | `bash scripts/build/build_driver.sh` |
| SLAM | `bash scripts/build/build_slam_core.sh` |
| Map daemon | `bash scripts/build/build_mapd.sh` |
| Navigation endpoint | `bash scripts/build/build_nav_endpoint.sh` |
| Exploration endpoint | `bash scripts/build/build_explore_kernel.sh` |
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
| Gateway point-cloud codec | `bash scripts/build/build_pointcloud_codec.sh` |

Windows uses the matching PowerShell entrypoint when one exists.

## Windows native simulation

- `prepare_cyclonedds_windows.ps1`
- `prepare_slam_dependencies_windows.ps1`
- `build_slam_core_windows.ps1`
- `build_mujoco_native_dds_windows.ps1`
- `verify_cyclonedds_windows_sdk.ps1`

The files under `cmake/`, `locks/`, `vcpkg/`, and `provenance/` are inputs to
that release build. They are not Product runtime state.

## Optional research builds

- `fetch_erasor2.sh` restores the optional map-cleaning comparison source used
  by `build_prune.sh`.
- `build_vendored_pcl.sh` and `build_octoplanner3d.sh` prepare optional native
  planner dependencies.

`fetch_orbbec_sdk.sh` fetches the field-tested standalone Orbbec SDK v2.8.7
into ignored `build/deps/orbbec-sdk/`; the native camera does not use a ROS2
wrapper.

Product startup never runs build scripts. ProductControl consumes installed
artifacts through the resolved RunPlan.
