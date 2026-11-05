# Native build entrypoints

Run these commands from the repository root. Compilation outputs go under
`build/`. Native release packaging installs them into the ignored standard
prefix `install/linux-<arch>/<config>/{bin,lib,etc,share}` before assembling
`dist/`; Product startup never resolves a `build/` path.

On Windows x64, use the same CMake install rules with
`--prefix install/windows-x64/Release`. The Linux OTA packager does not package
Windows binaries.

Component defaults come from `config/system/settings/*.yaml`. Native builds
generate `lingtu/settings.hpp` with `scripts/build/settings.py` using the project's
existing Python/PyYAML dependency. CMake tracks both the YAML and generator:
an ordinary rebuild updates the header after either changes. Generated headers
stay in the build directory and are installed with public headers; never edit
them or copy their values into C++ initializers. The direct Orbbec build script
runs the same generator. No YAML parser is added to the native hot path.

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
`make build` builds the robot runtime only. Set `LINGTU_CAMERA_DRIVER` to the
selected RobotConfig's `camera.capture_driver` for both build and native
packaging: `orbbec_native` (default) or `realsense_native`. The matching SDK
must be available. `make build-web` and `make package-web` build/package the
operator UI independently; see [Deployment](../deploy/README.md).

## Sensors and Gateway

| Target | Command |
| --- | --- |
| Livox SDK2 stream | `bash scripts/build/build_livox_sdk2_stream.sh` |
| Orbbec native stream | `bash scripts/build/build_orbbec_native.sh` |
| RealSense native stream | `bash scripts/build/build_realsense_native.sh` |
| Camera DDS endpoint | `bash scripts/build/build_camera_dds.sh` |
| Camera SHM → H.264 / native WebRTC | `bash scripts/build/build_camera_video.sh` (`video` Product role) |
| GNSS DDS endpoint | `bash scripts/build/build_gnss_dds.sh` |
| Gateway point-cloud codec | `cmake -S src/kernels/gateway/pointcloud_codec -B src/kernels/gateway/pointcloud_codec/build && cmake --build src/kernels/gateway/pointcloud_codec/build` |

On Windows, configure the same point-cloud codec source with a Visual Studio
generator, then run `cmake --build src/kernels/gateway/pointcloud_codec/build --config Release`.

### Video runtime

Prepare the approved GStreamer/WebRTC versions on the build machine before
building video or running `make build`:

```bash
bash scripts/build/prepare_camera_video.sh
bash scripts/build/build_camera_video.sh
```

Versions are fixed in `locks/video.json` and `locks/video-cargo.lock`. Preparation
uses an isolated prefix; Product startup never downloads or builds dependencies.
The release carries the video executable and its target-built runtime under
`lib/lingtu/video`. Configure the exact encoder with `config/video.env.example`;
software encoding is explicit, and target hardware/BSP qualification remains
separate. A runtime built in x86/WSL must not be used in an ARM release.

## GTSAM CPU dependency package

The online and saved-map optimizers consume the exact version in
[`locks/gtsam.json`](locks/gtsam.json), currently **4.3.0**. Upgrading this source
requires rebuilding the SDK and SLAM/optimizer binaries together; an old 4.0.2
installation cannot satisfy the new custom-factor interface.

On Linux, prepare Eigen, PCL, TBB and the C++ toolchain using the selected
platform's packages, then build the dependency without modifying `/usr/local`:

```bash
export LINGTU_GTSAM_PREFIX="$PWD/build/deps/gtsam-4.3.0"
LINGTU_BUILD_JOBS=2 bash scripts/build/prepare_gtsam.sh
export CMAKE_PREFIX_PATH="$LINGTU_GTSAM_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
bash scripts/build/build_slam_core.sh
```

The script uses system Eigen, shared libraries, Release mode and portable CPU
instructions. It disables Python wrappers, CUDA, unstable components and unused
Boost features/serialization. riSAM is in the main library and needs none of
those optional components. Upstream Pose3/Rot3 defaults remain unchanged.
The SDK includes its build settings and upstream license notices. Its archive
is named `dist/gtsam-<version>-<distribution><version>-<architecture>.tar.gz`.
The name comes from `/etc/os-release` and `uname -m`; it identifies the build
environment, not qualification on every machine with that name.

`LINGTU_GTSAM_BUILD_ROOT` can place the source/cache on a Linux filesystem for
faster WSL builds. The prefix and build root are separate from a running robot's
installation. Downloaded published SDKs will use the same layout and existing
`CMAKE_PREFIX_PATH`; this change does not publish a remote SDK download endpoint.
Keep compiler/runtime and Eigen/TBB compatibility when using an SDK on another
host. GTSAM and bundled METIS/Cephes runtime libraries are staged by the native CMake
install rules; PCL, TBB and the platform runtime remain target prerequisites.

For Windows, the existing SLAM vcpkg manifest selects the pinned upstream 4.3.0
overlay while retaining the other dependency baseline. Prepare that SDK again
before invoking the existing Windows SLAM build. Linux and Windows libraries
cannot be mixed. The offline [riSAM comparison](../../src/localization/online_mapping/README.md#offline-risam-comparison-gtsam-430)
does not switch the production backend from iSAM2 plus Huber.

On 2026-10-15, the CPU SDK was built on WSL Ubuntu 24.04 x86_64 (GCC 13.3,
Eigen 3.4, PCL 1.14), archived, extracted into another prefix and used to build
the native SLAM and saved-map optimizer. All eight distinct selected native
tests passed, including the offline riSAM comparison. Installed `slamd`,
`slamctl` and `lt_pgo` loaded outside the checkout with `LD_LIBRARY_PATH` unset;
GTSAM, METIS and Cephes resolved from the installed `lib/` directory. Evidence
is retained locally in `build/gtsam-upgrade-20261015/summary.md`. This does not
qualify an ARM/NX or Windows package, a complete container image, live sensors,
or robot motion.

## Windows native development and simulation

WSL is not required. Use PowerShell 7, Visual Studio 2022 C++ Build Tools
with the Windows SDK, CMake 3.27+ and Git. The current native SLAM mapping
chain links GTSAM/iSAM2 and PCL through `localization/online_mapping`; it does not build
the historical Rust pose-graph kernel. Separate legacy optimization tools
that still use that kernel require Rust/Cargo at build time.

For a new checkout, prepare the pinned native dependencies, then build SLAM.
The manifest includes GTSAM for the six-degree-of-freedom LIO-SAM backend. Existing
dependency prefixes must be refreshed before rebuilding this backend. The
2026-09-24 adapter was exercised on NX with GTSAM 4.0.2; that result does not
establish Windows build parity.
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

The pinned CycloneDDS source snapshot receives
`patches/cyclonedds-cdr-geometric-growth.patch` before compilation. It preserves
CDR encoding and DDS QoS, but grows the serialization buffer geometrically to
avoid repeated copying when hit bounds follow large occupancy bitmaps. The
upstream checkout remains unchanged. `evidence/source.json` records the exact
local patch. Verification rejects an older unpatched SDK even at the same
upstream commit; prepare the replacement in a new directory and explicitly use
that prefix when configuring a new consumer build. Existing installed SDKs are
never overwritten.

`build_slam_core_windows.ps1` builds `slamd`, `slamctl`, and tests, runs CTest,
and verifies the staged executables. Existing configured trees must keep their
original dependency prefixes; use an explicit absolute `-BuildDir` for a new
configuration when changing SDK locations. Do not reuse WSL/Linux CMake caches
or libraries in a Windows build.

After a successful configuration, the focused LTLIO and online-loop checks
can be rebuilt independently. Keep `$slamDeps` and `$ddsSdk` set to the actual
prefixes used by that build so Windows can find their DLLs:

```powershell
cmake --build build/slam-core-windows-x64 --config Release --target test_ltlio_mock_flow online_mapping_worker_test --parallel 4
$env:PATH = "$(Join-Path $slamDeps 'bin');$(Join-Path $ddsSdk 'bin');$env:PATH"
ctest --test-dir build/slam-core-windows-x64 -C Release --output-on-failure -R '^(messages_ltlio_mock_flow|online_mapping_worker)$'
```

The earlier versions of these checks passed on Windows for commit `53b2845c` on
2026-09-17; the online worker target was renamed during the 2026-11-02 cleanup.
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

- Optional map-cleaning experiments and their build/fetch entrypoints live in
  [`research/maps/`](../../research/maps/README.md). `build_prune.sh` builds only
  the production cleaner and does not configure ERASOR2, Patchwork++ or UFOMap.

`fetch_orbbec_sdk.sh` fetches the field-tested standalone Orbbec SDK v2.8.7
into ignored `build/deps/orbbec-sdk/`; the native camera does not use a ROS2
wrapper.

Product startup never runs build scripts. ProductControl consumes installed
artifacts through the resolved RunPlan.
