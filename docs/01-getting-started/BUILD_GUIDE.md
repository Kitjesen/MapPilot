# Build Guide

This guide sets up LingTu on a fresh development host or prepares the native
artifacts required by a supported field deployment. Choose the smallest build
track that covers the work you are doing. A successful compile is not
authorization to start a robot service or issue a motion command.

> **Status:** Current<br>
> **Audience:** Contributors, integrators, and deployment maintainers<br>
> **Runs on:** Windows x64 native-MuJoCo hosts, Linux/WSL native-build hosts, and supported field robots

## Build tracks at a glance

| Track | Use it for | Native hardware needed | What it does not prove |
| --- | --- | --- | --- |
| Portable Python | Framework, docs, contracts, and unit tests | No | C++ artifacts, simulation, or field runtime |
| MuJoCo simulation | In-process simulation and simulation tests | No | Real sensor timing, calibration, gait, or field readiness |
| Windows native MuJoCo | Native CycloneDDS simulation processes and Windows Product-port work | No | Linux/S100P support or Product acceptance until the exact Windows transaction passes |
| Linux/WSL native | Navigation kernel, SLAM, planner, and endpoint artifacts | No during build | A deployed robot service or safe motion |
| Field deployment | Installing the native DDS product services on a target | Supported target, controlled session | Readiness until no-motion and field gates pass |

The default product direction is native Python/C++ plus typed DDS. ROS 2 is
optional and only belongs to explicit compatibility, replay, or evaluation
work. Do not source a ROS 2/colcon environment or install ROS packages merely
to run the normal local or native Thunder product path.

**Product default: native planner kernels, no ROS2.**
ROS 2 Humble Desktop is optional and belongs only to explicit compatibility,
replay, calibration, or benchmark work.

## Prerequisites

### Repository and interpreter

Start in the repository root. LingTu requires Python 3.10 or newer;
[`.python-version`](../../.python-version) records the current project default.
Use a supported `uv` installation to create the environment.

```bash
uv --version
uv sync --locked --extra dev
uv run --locked python -m lingtu.control --help
```

Expected result:

- `uv sync --locked` completes without modifying `uv.lock`.
- The list command prints the current checkout's profile catalog.
- No LingTu runtime modules or hardware services are started by these checks.

If `uv` says the lockfile needs an update, stop. The lockfile and project
metadata are inconsistent for that checkout. Restore the intended revision or
make an explicit, reviewed dependency update; do not use an unlocked run as a
workaround.

### Host scope

| Host | Supported first work | Notes |
| --- | --- | --- |
| Windows x64 | Portable work, native MuJoCo DDS/SLAM artifacts, and all declared Product RunPlan targets | First-class MuJoCo target. W1-W3 build and compile gates pass; exact Product execution remains pending. |
| Linux/WSL x86_64 | Portable work plus the native ELF build tracks | Parallel MuJoCo target and the closest workstation ABI shape to the S100P deployment; it does not substitute for Windows-native evidence. |
| S100P/Linux controller | Native services and supervised field deployment | Treat as a controlled deployment target, not a general development shell. |

## Select the Python environment

Extras are intentionally separated so a workstation does not acquire robot,
ML, or web dependencies by accident. Add only the extras needed for the
profile or test you intend to run.

| Need | Locked sync command | Notes |
| --- | --- | --- |
| Framework tests and developer tools | `uv sync --locked --extra dev` | Includes pytest, lint/type tooling, and Gateway runtime dependencies used by development. |
| Gateway without all developer tools | `uv sync --locked --extra gateway` | Adds FastAPI, Uvicorn, and WebSocket support. |
| MuJoCo simulation | `uv sync --locked --extra sim-mujoco --extra dev` | Required before the `sim` profile. |
| Common local semantic backends | `uv sync --locked --extra vision --extra ml --extra llm --extra nlp` | Install only when the selected detector/LLM path needs them. |
| Heavy perception or vector storage | `uv sync --locked --extra perception --extra vector` | Optional; do not add to a lightweight framework environment by default. |
| Thunder Python integration | `uv sync --locked --extra thunder` | Field-target dependency group; use only in the approved target environment. |

After changing extras, verify configuration without starting a profile:

```bash
uv run --locked python -m lingtu.control switch teleop --robot doso/thunder_v4 --env sim --dry-run --json
uv run --locked python -m pytest src/runtime/tests/test_runtime_graph_contract.py -q
```

The local command convention remains:

```bash
uv run --locked python -m lingtu.control <switch|status|stop> [product] [options]
```

## Portable Python verification

Use this track before native work. It is the fastest way to prove that the
checkout, Product compiler, and Module contracts are coherent.

```bash
uv run --locked python -m pytest src/runtime/tests/ -q
uv run --locked python -m lingtu.control --help
uv run --locked python tools/validate/validate_architecture_boundaries.py
uv run --locked python tools/validate/validate_topics.py
```

Expected result:

- Framework tests cover the runtime surface without needing hardware.
- Runtime graph, architecture, and topic validators inspect declarations and
  contracts; they do not create a robot session.
- Passing this track does not prove the simulation or field runtime.

For a safe interactive framework smoke after the checks pass:

```bash
uv run --locked python -m lingtu.control switch teleop --robot doso/thunder_v4 --env sim --dry-run
```

Use `health`, `connections`, and `quit` in the TTY REPL. See
[Quick Start](../QUICKSTART.md#local-prove-the-framework-path) for expected
behavior and boundaries.

## MuJoCo simulation setup

The in-process `sim` profile uses the MuJoCo dependency group. It can move a
simulated robot, but it has no permission to command field hardware.

```bash
uv sync --locked --extra dev --extra sim-mujoco
uv run --locked python -m lingtu.control switch teleop --robot doso/thunder_v4 --env sim --dry-run --json
uv run --locked python sim/scripts/mujoco/native_navigation_acceptance.py --help
```

Before recording an evaluation result, verify that the resolved runtime uses the
simulation source and sink. Do not attach a field endpoint to this command as a
shortcut to hardware validation.

Use [`sim/README.md`](../../sim/README.md) for worlds, drive modes, saved-map
quality gates, and simulation claim boundaries. The
[simulation integration contract](../architecture/SIMULATION_INTEGRATION_CONTRACT.md)
defines the adapter and command-sink rules.

## Windows native MuJoCo artifacts

Windows-native Product parity is an active P0 requirement. The transport layer
is already buildable without WSL. Use the existing helper with a complete
Windows CycloneDDS SDK prefix:

```powershell
$env:LINGTU_CYCLONEDDS_PREFIX = "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\sdk"
pwsh -NoProfile -File scripts\build\build_mujoco_native_dds_windows.ps1 `
  -BuildDirectory "D:\inovxio\brain\lingtu\build\windows-native-dds-adapter"
```

The helper configures a Windows-only build tree, compiles the MuJoCo native DDS
runtime, runs CTest, and stages `ddsc.dll` beside the executables. The Product
chain uses, among other generated targets:

```text
build/windows-native-dds-adapter/Release/lingtu_mujoco_sensor_publisher.exe
build/windows-native-dds-adapter/Release/lingtu_mujoco_driver_bridge.exe
```

The canonical Windows outputs use application-local CycloneDDS runtime
closure. These four files must exist beside, or in the staged `bin` directory
with, their corresponding executables:

```text
build/windows-native-dds-adapter/Release/ddsc.dll
build/nav-cpp/windows-x64-nav-endpoint/Release/ddsc.dll
build/maps-windows/Release/ddsc.dll
build/slam-core-windows-x64/stage/bin/ddsc.dll
```

All four current copies have SHA-256
`203ece8c0b2c00380f632c0d85380f5381354957e0fb78155e1de8cf7d996887`.
This closes the canonical Windows `0xC0000135` missing-`ddsc.dll` failure
class. It does not repair an executable copied out of these directories. The
old `-root-d` trees are quarantined under
`C:/Users/99563/.codex/tmp/lingtu-stale-builds`; no configuration, fallback,
alias, or junction may select them.

With `PATH` reduced to the real Windows `System32`, the staged command smokes
reach their documented usage output instead of failing in the loader. A usage
exit proves dependency loading only; it is not a Product pass.

The Windows navigation preset requires both dependency prefixes. Configure,
build, and test the one canonical navigation tree with:

```powershell
$env:LINGTU_CYCLONEDDS_PREFIX = "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\sdk"
$env:LINGTU_OCTOMAP_PREFIX = "C:\opt\octomap-1.10.0-msvc-x64"
Push-Location src\nav\cpp
cmake --preset windows-x64-nav-endpoint --fresh
cmake --build --preset windows-x64-nav-endpoint
ctest --preset windows-x64-nav-endpoint
Pop-Location
```

The preset writes only to
`build/nav-cpp/windows-x64-nav-endpoint`. Use compatible MSVC x64 CycloneDDS
and OctoMap prefixes; do not add a second build-directory name to distinguish
which dependency copy was used.

Install the official Microsoft Visual C++ Redistributable x64 on the host.
CycloneDDS is app-local, but the MSVC runtime is a central prerequisite; these
outputs are therefore not fully self-contained. Do not copy the Microsoft CRT
into the application directories. The Windows helper and ProductControl fail
closed before build/test or child startup unless the 64-bit runtime is
registered and `System32` contains these PE32+/x64 files:

```text
msvcp140.dll
msvcp140_2.dll
vcruntime140.dll
vcruntime140_1.dll
vcomp140.dll
```

The build helper derives its minimum runtime version from the configured MSVC
toolset. Product startup derives the requirement from the exact SHA-bound PE
artifacts and their declared dependencies, so upgrading the toolset cannot
silently leave an older runtime preflight behind.

This proves the Windows CycloneDDS transport boundary; the separate W2 wrapper
below produces `slamd.exe`. The Fast-LIO ikd-tree no longer directly uses
`pthread`, `unistd`, or `usleep`. It uses the C++17 standard-library thread,
mutex, condition-variable, atomic, and clock facilities. The port also fixes
split-axis equal-point deletion, `working_flag` lifetime, empty rebuild replay,
and the required rebuild/read synchronization. The cleanup removed swallowed
background exceptions, 100-microsecond polling, a global range-query lock, the
unused benchmark, and excessive fault injection. The focused concurrency test
passed 20 repetitions on Windows and 20 on Linux; Linux ASan/UBSan and leak
checks also pass. Native Linux TSan and dedicated S100P performance evidence
remain open. W1B is complete; a DDS production runtime can no longer configure
with the real Fast-LIO2 backend disabled, and its focused contract suite passes
45 tests.

W2 uses PowerShell 7 (`pwsh`) and three fail-closed steps. First prepare and
verify the source-locked CycloneDDS 11.0.1 SDK:

```powershell
pwsh -NoProfile -File scripts\build\prepare_cyclonedds_windows.ps1 `
  -SourceRoot "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\source" `
  -BuildRoot "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\build" `
  -InstallRoot "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\install" `
  -SdkRoot "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\sdk" `
  -PreflightOnly

pwsh -NoProfile -File scripts\build\prepare_cyclonedds_windows.ps1 `
  -SourceRoot "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\source" `
  -BuildRoot "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\build" `
  -InstallRoot "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\install" `
  -SdkRoot "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\sdk"
```

Then prepare the pinned vcpkg x64-windows dependency prefix:

```powershell
pwsh -NoProfile -File scripts\build\prepare_slam_dependencies_windows.ps1 `
  -VcpkgRoot "D:\inovxio\brain\lingtu\third_party\toolchains\vcpkg" `
  -InstallRoot "D:\inovxio\brain\lingtu\third_party\install\slam-windows" `
  -BinaryCache "D:\inovxio\brain\lingtu\third_party\cache\vcpkg" `
  -PreflightOnly

pwsh -NoProfile -File scripts\build\prepare_slam_dependencies_windows.ps1 `
  -VcpkgRoot "D:\inovxio\brain\lingtu\third_party\toolchains\vcpkg" `
  -InstallRoot "D:\inovxio\brain\lingtu\third_party\install\slam-windows" `
  -BinaryCache "D:\inovxio\brain\lingtu\third_party\cache\vcpkg"
```

Finally run the SLAM wrapper with all five required absolute prefixes. The
dependency prefix must be the `x64-windows` child of the vcpkg install root:

```powershell
pwsh -NoProfile -File scripts\build\build_slam_core_windows.ps1 `
  -DependencyPrefix "D:\inovxio\brain\lingtu\third_party\install\slam-windows\x64-windows" `
  -CycloneDDSPrefix "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\sdk" `
  -VcpkgRoot "D:\inovxio\brain\lingtu\third_party\toolchains\vcpkg" `
  -VcpkgInstallRoot "D:\inovxio\brain\lingtu\third_party\install\slam-windows" `
  -VcpkgBinaryCache "D:\inovxio\brain\lingtu\third_party\cache\vcpkg" `
  -BuildDir "D:\inovxio\brain\lingtu\build\slam-core-windows-x64" `
  -PreflightOnly

pwsh -NoProfile -File scripts\build\build_slam_core_windows.ps1 `
  -DependencyPrefix "D:\inovxio\brain\lingtu\third_party\install\slam-windows\x64-windows" `
  -CycloneDDSPrefix "D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\sdk" `
  -VcpkgRoot "D:\inovxio\brain\lingtu\third_party\toolchains\vcpkg" `
  -VcpkgInstallRoot "D:\inovxio\brain\lingtu\third_party\install\slam-windows" `
  -VcpkgBinaryCache "D:\inovxio\brain\lingtu\third_party\cache\vcpkg" `
  -BuildDir "D:\inovxio\brain\lingtu\build\slam-core-windows-x64"
```

`-DependencyPrefix` must contain the MSVC x64 Eigen, PCL, and yaml-cpp CMake
packages. `-CycloneDDSPrefix` must contain the pinned CycloneDDS 11.0.1 SDK,
including its verified build-source/file-inventory record, x64 `idlc.exe`, and
`ddsc.dll`. The vcpkg root
must be a clean detached checkout at the manifest baseline; the install root
and binary cache are separate mandatory inputs. `-BuildDir` must be an
absolute, Windows-only Visual Studio 2022 x64 tree. `-PreflightOnly` is
read-only: it validates inputs and any existing cache, then exits before CMake
configure.

The authoritative fresh CycloneDDS SDK for this evidence run is
`D:\inovxio\brain\lingtu\third_party\integration\cyclonedds-windows-20260817-d\sdk`.
It was built from the pinned official 11.0.1 source with Visual Studio 2022
`v143`, x64, Release, and `/MD`. Its build-source/file-inventory record binds commit
`e54e991f75a3e67f8e628da3171122e36ea5b872` and tree
`56508d35826c362782fc8a388cad351a3d491f51`; PE/x64, DLL closure, IDL smoke,
consumer compile/link, and sanitized DLL-search checks passed. The pinned vcpkg
MSVC x64 prefix and the real Fast-LIO-enabled `slamd.exe`/`slamctl.exe` build
are complete; the native suite passed 7/7 CTest.

The formal runtime stage is
`build/slam-core-windows-x64/stage`. Its `evidence/` directory contains the
stage build-source/file-inventory record `stage-receipt.json`,
`stage-files.sha256`, `runtime-dependencies.tsv`, and
`sbom.spdx.json`; package notices live under `licenses/`. The fresh stage has
exactly 42 files. Its audit passed all 21 PE files as x64, found zero unresolved
non-system imports, and passed the unchanged-input reuse path. The SPDX 2.3
inventory contains 13 packages, 35 files, and 52 relationships, including the
exact `LingTu -> CycloneDDS` `DYNAMIC_LINK` relationship. The staged closure
attributes 18 runtime DLLs to their vcpkg packages, `ddsc.dll` to CycloneDDS,
and only `slamd.exe` plus `slamctl.exe` to the LingTu local build. A real raw
LiDAR/IMU loopback reached Fast-LIO `TRACKING`; typed `slamctl` status and the
repository readiness loader passed, while a DDS domain mismatch failed closed.

The active implementation order is maintained in
[`current-roadmap.md`](../plans/current-roadmap.md): W1-W3 are complete at their
code/build/RunPlan compile gates; use the W4 exact coordinator for real Product
evidence and run the W5 Windows and Linux/WSL matrices independently.

The Windows SLAM dependency closure is complete for the current development
evidence. W2 pins one package baseline, rejects mixed MinGW/Conda/MSVC binaries,
and records the selected architecture, versions, paths, DLLs, and artifact
hashes. This build evidence still does not clear the distribution release gate.

Never configure a Linux build directory with Visual Studio or a Windows build
directory with a Linux, WSL, MinGW, MSYS, or UCRT64 generator/toolchain. Do not
pass MinGW/MSYS package prefixes to the Windows wrapper. In particular, the
RunPlan-selected Windows tree is `build/windows-native-dds-adapter`;
`build/mujoco_native_dds` is reserved for Linux. In the current checkout that
second directory contains a Visual Studio cache and is not Linux build
evidence; recreate it from Linux in a clean, dedicated tree before using the
canonical path. Use a fresh build directory whenever its generator or target
platform changes, and never treat a mixed CMake cache as release evidence.

Windows Product support is complete only after all seven Products pass their
exact ProductControl transaction with PE/DLL-only native artifacts, with
`explore` accepted separately in `live` and `map` variants. Shared
RunPlan-declared Python feeder/Host processes remain interpreter-owned. A WSL
process launched from PowerShell is Linux evidence, not Windows-native
evidence.

W3 compiles all seven Products on Windows, with `explore` resolved in both
`live` and `map` variants, for eight selected RunPlan targets. The canonical chain
covers sensor, driver, mapd, traversability, navd, exploration, and the Host
navigation client alongside the staged SLAM runtime. Fresh contract evidence is
86 Product-compile tests, 141 runtime-graph tests, and 55 RunPlan
identity/environment tests. Startup stages are 10 for sensor/driver; 20 for
feeder, SLAM, traversability, and nav; 30 for mapd; and 50 for
exploration/Host.

RunPlan uses schema v8 and rejects older plans with an instruction to switch
the Product again. It carries final launch parameters and binds each selected
artifact and dependency by its real path and
SHA-256 digest, then checks PE32+/x64 and the registered 64-bit VC++ runtime
before startup. There is no separate per-process MSVC build record and no set
of four extra JSON files. This is intentionally different from the CycloneDDS
SDK and formal SLAM-stage build-source/file-inventory records, which document
how those packages were assembled.

Windows Product processes use DDS domain `17`; Linux Product processes use
domain `231`. The MuJoCo adapter CTests use low-domain slots `10`-`16`, `18`,
and `19`. Their configuration rejects Product domain `17` and any domain whose
CycloneDDS fixed ports reach the Windows dynamic-port range. Do not override a
Product RunPlan with a test domain.

Run the contract checks with the repository virtual environment:

```powershell
.venv\Scripts\python.exe -m pytest src\lingtu\assembly\tests\test_compile.py -q
.venv\Scripts\python.exe -m pytest src\runtime\tests\test_runtime_graph_contract.py -q
.venv\Scripts\python.exe -m pytest src\lingtu\tests\test_run_plan.py -q
```

W4's `teleop` exact coordinator has passed code review and focused tests. W5
has started, and the fresh Windows exact-`teleop` technical report has
`ok=true`, `blockers=[]`, all four readiness gates, a passing attach-only
scenario, 0.164 m path/0.158 m net physical motion, terminal zero, stop,
cleanup, and rollback. The report still declares
`evidence_scope=component_e2e`, the manifest remains `coverage=component`, and
`product_acceptance_passed=false`. Repeatability remains in progress; the exact
Product PASS count is zero.

Fresh component verification is: maps 32/32; the 17 selected navigation tests
passed three consecutive runs; SLAM 7/7 plus the formal stage audit; adapter
endpoint apply/ACK 50/50, bridge stress 20/20, and the fresh complete adapter
suite 18/18. The IMU test fix adds only best-effort DDS discovery warm-up and a
read condition; production code is unchanged. Treat the screenshot-class
loader error as closed only for canonical outputs; do not treat it as full
navigation or W5 Product acceptance.

The build is also subject to the release provenance and license gate. The
official FAST_LIO, ikd-Tree, and IKFoM repositories publish GPL-2.0 licenses;
porting or replacing only the ikd-tree concurrency layer does not by itself
authorize a closed-source `slamd` distribution. Record the bundled source
origin, exact revision, modifications, notices, and the licenses of Fast-LIO,
ikd-tree, IKFoM, PCL, Eigen, yaml-cpp, and CycloneDDS. Before distributing either
platform's artifacts, document commercial authorization for the complete
copyright chain, a reviewed GPL-compliant release boundary, or a clean-room
replacement decision. A successful compile does not clear unresolved
provenance or license obligations. Upstream license and origin notices are kept
under `scripts/build/provenance/slam/legal/`; no local checksum receipt is
treated as release authorization.

Linux and the S100P board do not have the Windows header-availability failure:
their toolchains provide `pthread` and `unistd`. They can still encounter
platform-independent worker shutdown, join, race, or destruction defects. The
current Linux W1A lifecycle and sanitizer gates passed; native Linux TSan and
dedicated S100P performance evidence remain open. Windows and Linux/WSL
evidence remain target-specific and cannot satisfy each other's Product gates.

## Native Linux/WSL prerequisites

Linux-native builds should run in Linux, WSL, or the target S100P environment.
This is a parallel build track, not the replacement for the Windows-native
MuJoCo target. The exact package set depends on the selected artifacts; the
following baseline is grounded in the native SLAM, navigation endpoint, and
driver build scripts.

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  pkg-config \
  python3-dev \
  libeigen3-dev \
  libpcl-dev \
  libyaml-cpp-dev \
  cyclonedds-dev \
  cyclonedds-tools \
  libgrpc++-dev \
  libprotobuf-dev \
  protobuf-compiler-grpc \
  libgtest-dev
```

Why these packages matter:

| Dependency | Native surface |
| --- | --- |
| C++ compiler, CMake | Native navigation and service builds |
| Eigen3, PCL, yaml-cpp | ROS-free Fast-LIO2 SLAM core |
| CycloneDDS development/tools | Typed DDS SLAM, navigation endpoint, and diagnostics |
| gRPC and protobuf toolchain | Native Thunder driver integration |
| GoogleTest development files | Required CTest coverage in the navigation endpoint build |

The scripts perform their own dependency probes. For example,
`build_slam_core.sh` checks Eigen3, the required PCL components, and yaml-cpp
before configuring Fast-LIO2. Prefer the script's explicit error over manually
changing CMake cache entries.

## Build native artifacts in dependency order

Run all commands below from the repository root. They build artifacts and run
their configured tests; they do not install or start field services.

### 1. Native SLAM and Livox DDS ingress

Build the process-split native path when the target will ingest real MID-360
data through typed DDS:

```bash
LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS=ON \
  bash scripts/build/build_livox_sdk2_stream.sh

LINGTU_SLAM_BUILD_DDS_RUNTIME=ON \
  bash scripts/build/build_slam_core.sh
```

Expected outputs include the Livox stream executable and
`build/slam_core/slamd`. The default Fast-LIO2 build is
ROS-free; its dependency probe checks the native C++ libraries listed above.

For global saved-map relocalization without an initial pose, build the optional
CPU 3D-BBS library before requiring it:

```bash
bash scripts/build/build_3d_bbs.sh
LINGTU_REQUIRE_BBS3D=ON bash scripts/build/build_slam_core.sh
```

### 2. Native navigation and driver boundaries

Build the navigation endpoint and the Thunder driver:

```bash
bash scripts/build/build_nav_endpoint.sh
LINGTU_DRIVER_BACKEND=<go2|doso> bash scripts/build/build_driver.sh
```

`build_nav_endpoint.sh` runs CTest by default and requires its navigation,
teleop-safety, path-follower, and local-planner tests to be present. It verifies
the expected native navigation endpoint binaries and client library. The driver
build similarly runs its fail-closed safety and typed DDS-to-Brainstem tests by
default.

### 4. OctoPlanner3D and saved-map tools

Build the Linux/WSL/S100P headless planner. Require PCL only when the host must
convert PCD inputs to OctoMap artifacts:

```bash
bash scripts/build/build_octoplanner3d.sh --require-pcl
```

If the PCL-backed build cannot find the correct libraries, use the script's
diagnostic mode before changing paths:

```bash
bash scripts/build/build_octoplanner3d.sh --diagnose
```

The repository includes a separate vendored-PCL flow for this converter scope;
follow [`scripts/build/README.md`](../../scripts/build/README.md) when a
system PCL installation is not appropriate.

For LingTu's default clean-room saved-map cleaner:

```bash
bash scripts/build/build_prune.sh
```

DUFOMap and ERASOR2 remain optional compatibility or comparison surfaces. Do
not add them to a field build unless the selected map-cleaning plan explicitly
requires them.

## Verify a native build without commanding a robot

Use the smallest checks that validate the artifacts you built:

```bash
uv run --locked python -m pytest src/runtime/tests/ -q
uv run --locked python tools/validate/validate_architecture_boundaries.py
uv run --locked python tools/validate/validate_topics.py
python -m lingtu.control switch nav --robot unitree/go2 --env real --map MAP_NAME --dry-run --json
bash scripts/build/build_octoplanner3d.sh --diagnose
```

The first four commands inspect Python contracts and the resolved Product
boundary; the ProductControl command is a dry run and does not start a field
session. The final diagnostic reports the
configured planner build and linkage state. Native build scripts also run their
targeted tests unless their documented test control environment variable has
been deliberately changed.

A green build means the artifacts compiled and their local checks passed. It
does not prove that the target robot has correct calibration, live sensor data,
localization, a valid map, route safety, or permission to move.

## Deploy native field services only in a controlled session

Service installation changes the target system. Perform it only on the approved
Linux controller, with no active mission, and according to the
[Thunder deployment guide](../04-deployment/README.md).

```bash
LINGTU_BRAINSTEM_HOST=REMOTE_BRAINSTEM_IP \
LINGTU_BRAINSTEM_PORT=13145 \
  bash scripts/deploy/thunder/install_services.sh field-cpp
```

This endpoint is mandatory because Brainstem runs on the separate robot-control
computer. The installer rejects loopback and persists the validated endpoint
to `/opt/lingtu/config/brainstem.env`.

The normal product service chain is native DDS:

```text
lt-lidar
  -> lt-slam
  -> LingTu Modules / Gateway / MCP
  -> lt-nav
  -> typed DDS command boundary
```

After the deployment procedure starts the services, use the robot-side CLI for
observation before making any session change:

```bash
systemctl --no-pager --full status 'lt-*.service'
bash scripts/lingtu status
PYTHONPATH=src python -m diagnostics.field.doctor
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/odometry"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/map_cloud"
```

These checks are necessary but do not authorize motion. Continue with saved-map
artifact validation, relocalization, a no-motion route preview, and the
applicable field gate as described in
[Quick Start](../QUICKSTART.md#field-prepare-validate-then-supervise) and
[`lingtu_cli.md`](../04-deployment/lingtu_cli.md).

## Troubleshooting by boundary

| Symptom | Likely boundary | First response |
| --- | --- | --- |
| `uv --locked` refuses to run | Dependency lock integrity | Restore/reconcile the intended lockfile change; do not run unlocked. |
| `FastAPI not installed` | Optional Gateway dependency | Sync the `gateway` or `dev` extra if Gateway is intentionally required. |
| Fast-LIO CMake probe fails | Native SLAM C++ dependencies | Install/repair Eigen3, PCL components, and yaml-cpp; rerun the script. |
| Native endpoint test catalog is incomplete | C++ test dependency/build configuration | Install GoogleTest development files and reconfigure with the build script. |
| OctoPlanner3D PCL converter cannot link | PCL prefix/configuration | Run `build_octoplanner3d.sh --diagnose`; follow the vendored-PCL guide if needed. |
| A Product launch asks for ROS 2 | Compatibility configuration leaked into a native path | Check the selected Product, env, and RunPlan; ROS 2 is not a normal native field prerequisite. |
| A field service is running but navigation is blocked | Runtime readiness, not compilation | Stop treating it as a build issue; inspect localization, map artifacts, route preview, and safety via `scripts/lingtu`. |

## Related references

- [Get Started](./README.md) — staged onboarding and motion boundaries.
- [Quick Start](../QUICKSTART.md) — Product/env selection, lifecycle, and field
  preflight.
- [`scripts/build/README.md`](../../scripts/build/README.md) — detailed
  build-script behavior and specialized native flows.
- [Thunder deployment](../04-deployment/README.md) — service installation,
  release, diagnostics, and recovery.
- [Architecture index](../architecture/README.md) — the current contracts that
  native artifacts must satisfy.
