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

## Windows native development and simulation

WSL is not required. Use PowerShell 7, Visual Studio 2022 C++ Build Tools
with the Windows SDK, CMake 3.27+, Git, and Rust stable for
`x86_64-pc-windows-msvc`. Cargo builds the Rust pose-graph kernel linked into
the C++ SLAM backend. A missing Cargo executable is a build-environment error;
running an already-built SLAM release does not require Cargo.

For a new checkout, prepare the pinned native dependencies, then build SLAM.
The preparation steps need network access on their first run. Run from the
repository root in a Visual Studio developer PowerShell 7 session:

```powershell
$lingtuRoot = (Get-Location).Path
$vcpkgRoot = Join-Path $lingtuRoot 'third_party/toolchains/vcpkg'
$slamInstall = Join-Path $lingtuRoot 'third_party/install/slam-windows'
$slamDeps = Join-Path $slamInstall 'x64-windows'
$binaryCache = Join-Path $lingtuRoot 'third_party/cache/vcpkg'
$ddsLock = Get-Content scripts/build/locks/cyclonedds-windows-x64.json -Raw | ConvertFrom-Json
$ddsSdk = Join-Path $lingtuRoot "third_party/sdk/cyclonedds-$($ddsLock.tag)-windows-x64"

& ./scripts/build/prepare_cyclonedds_windows.ps1 -SdkRoot $ddsSdk
& ./scripts/build/prepare_slam_dependencies_windows.ps1 -VcpkgRoot $vcpkgRoot -InstallRoot $slamInstall -BinaryCache $binaryCache
& ./scripts/build/build_slam_core_windows.ps1 -DependencyPrefix $slamDeps -CycloneDDSPrefix $ddsSdk -VcpkgRoot $vcpkgRoot -VcpkgInstallRoot $slamInstall -VcpkgBinaryCache $binaryCache
```

`build_slam_core_windows.ps1` builds `slamd`, `slamctl`, and tests, runs CTest,
and verifies the staged executables. Existing configured trees must keep their
original dependency prefixes; use an explicit absolute `-BuildDir` for a new
configuration when changing SDK locations. Do not reuse WSL/Linux CMake caches
or libraries in a Windows build.

After a successful configuration, the focused Fast-LIO2 and online-loop checks
can be rebuilt independently. Keep `$slamDeps` and `$ddsSdk` set to the actual
prefixes used by that build so Windows can find their DLLs:

```powershell
cmake --build build/slam-core-windows-x64 --config Release --target test_fastlio2_mock_flow online_mapping_test --parallel 4
$env:PATH = "$(Join-Path $slamDeps 'bin');$(Join-Path $ddsSdk 'bin');$env:PATH"
ctest --test-dir build/slam-core-windows-x64 -C Release --output-on-failure -R '^(messages_fastlio2_mock_flow|online_mapping)$'
```

These two checks passed on Windows for commit `53b2845c` on 2026-09-17.
They exercise native SLAM flow and synthetic online loop correction, not
supervised robot motion or complete saved-map navigation acceptance.

`build_mujoco_native_dds_windows.ps1` builds the simulation sensor bridge;
`verify_cyclonedds_windows_sdk.ps1` checks a prepared DDS SDK. Preparing or
building SLAM alone does not stage every native role required by a Product.

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
