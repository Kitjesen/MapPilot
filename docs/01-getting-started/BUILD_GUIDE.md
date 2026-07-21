# Build Guide

This guide sets up LingTu on a fresh development host or prepares the native
artifacts required by a supported field deployment. Choose the smallest build
track that covers the work you are doing. A successful compile is not
authorization to start a robot service or issue a motion command.

> **Status:** Current<br>
> **Audience:** Contributors, integrators, and deployment maintainers<br>
> **Runs on:** Local development hosts, Linux/WSL native-build hosts, and supported field robots

## Build tracks at a glance

| Track | Use it for | Native hardware needed | What it does not prove |
| --- | --- | --- | --- |
| Portable Python | Framework, docs, contracts, and unit tests | No | C++ artifacts, simulation, or field runtime |
| MuJoCo simulation | In-process simulation and simulation tests | No | Real sensor timing, calibration, gait, or field readiness |
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
uv run --locked python lingtu.py --list --all
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
| Windows | Portable Python, documentation, static checks, selected tooling | Build Linux-native C++ planner/endpoint artifacts in WSL or Linux. |
| Linux/WSL | Portable work plus native C++ build tracks | OctoPlanner3D's headless wrapper is a Linux/WSL/S100P build. |
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
uv run --locked python lingtu.py show-config stub --json
uv run --locked python lingtu.py runtime-audit
```

The local command convention remains:

```bash
uv run --locked python lingtu.py <command-or-profile> [options]
```

## Portable Python verification

Use this track before native work. It is the fastest way to prove that the
checkout, profile resolver, and Module contracts are coherent.

```bash
uv run --locked python -m pytest src/runtime/tests/ -q
uv run --locked python lingtu.py --list --all
uv run --locked python lingtu.py runtime-contract
uv run --locked python lingtu.py runtime-audit
```

Expected result:

- Framework tests cover the runtime surface without needing hardware.
- `runtime-contract` and `runtime-audit` inspect declarations and contracts;
  they do not create a robot session.
- Passing this track does not prove the simulation or field runtime.

For a safe interactive framework smoke after the checks pass:

```bash
uv run --locked python lingtu.py stub --no-gateway
```

Use `health`, `connections`, and `quit` in the TTY REPL. See
[Quick Start](../QUICKSTART.md#local-prove-the-framework-path) for expected
behavior and boundaries.

## MuJoCo simulation setup

The in-process `sim` profile uses the MuJoCo dependency group. It can move a
simulated robot, but it has no permission to command field hardware.

```bash
uv sync --locked --extra dev --extra sim-mujoco
uv run --locked python lingtu.py runtime-spec sim --json
uv run --locked python lingtu.py sim
```

Before recording an evaluation result, verify that the resolved runtime uses the
simulation source and sink. Do not attach a field endpoint to this command as a
shortcut to hardware validation.

Use [`sim/README.md`](../../sim/README.md) for worlds, drive modes, saved-map
quality gates, and simulation claim boundaries. The
[simulation integration contract](../architecture/SIMULATION_INTEGRATION_CONTRACT.md)
defines the adapter and command-sink rules.

## Native Linux/WSL prerequisites

Native builds should run in Linux, WSL, or the target S100P environment. The
exact package set depends on the selected artifacts; the following baseline is
grounded in the native SLAM, navigation endpoint, and driver build scripts.

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
| C++ compiler, CMake, Python headers | Navigation kernel and nanobind extension build |
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

### 1. Optional local navigation kernel

Use this for the nanobind navigation-kernel extension on a Linux/WSL build
host. It requires CMake, a C++17 compiler, and Python development headers.

```bash
bash scripts/build/build_nav_kernel.sh --clean
```

The script verifies the generated `lingtu_nav_kernel` import after building. It
uses the host Python selected by the script, so do not assume that an artifact
built under one interpreter can be imported by an unrelated environment.

### 2. Native SLAM and Livox DDS ingress

Build the process-split native path when the target will ingest real MID-360
data through typed DDS:

```bash
LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS=ON \
  bash scripts/build/build_livox_sdk2_stream.sh

LINGTU_SLAM_BUILD_DDS_RUNTIME=ON \
LINGTU_SLAM_BUILD_PYTHON_BINDINGS=OFF \
  bash scripts/build/build_slam_core.sh
```

Expected outputs include the Livox stream executable and
`build/slam_core/lingtu_slam_cyclone_runtime`. The default Fast-LIO2 build is
ROS-free; its dependency probe checks the native C++ libraries listed above.

For global saved-map relocalization without an initial pose, build the optional
CPU 3D-BBS library before requiring it:

```bash
bash scripts/build/build_3d_bbs.sh
LINGTU_REQUIRE_BBS3D=ON bash scripts/build/build_slam_core.sh
```

### 3. Native navigation and driver boundaries

Build the navigation endpoint and the Thunder driver:

```bash
bash scripts/build/build_nav_endpoint.sh
bash scripts/build/build_driver.sh
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

For the native map-save optimizer commands used by deployed services:

```bash
bash scripts/build/build_native_runtime.sh --install-user-bin
```

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
uv run --locked python lingtu.py runtime-audit
uv run --locked python lingtu.py runtime-spec nav --endpoint thunder_field --json
bash scripts/build/build_octoplanner3d.sh --diagnose
```

The first three commands inspect Python contracts and the resolved field
boundary; they do not start a field session. The final diagnostic reports the
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
LINGTU_BRAINSTEM_HOST=<remote-brainstem-ip> \
LINGTU_BRAINSTEM_PORT=13145 \
  bash scripts/deploy/thunder/install_services.sh field-cpp
```

This endpoint is mandatory because Brainstem runs on the separate robot-control
computer. The installer rejects loopback and persists the validated endpoint
to `/opt/lingtu/config/brainstem.env`.

The normal product service chain is native DDS:

```text
lingtu-livox-dds
  -> lingtu-slam-dds
  -> LingTu Modules / Gateway / MCP
  -> lingtu-nav-dds
  -> typed DDS command boundary
```

After the deployment procedure starts the services, use the robot-side CLI for
observation before making any session change:

```bash
bash scripts/lingtu svc status
bash scripts/lingtu status
bash scripts/lingtu doctor
bash scripts/lingtu dataflow /nav/odometry
bash scripts/lingtu dataflow /nav/map_cloud
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
| A product launch asks for ROS 2 | Compatibility configuration leaked into a native path | Check the selected profile/endpoint; ROS 2 is not a normal native field prerequisite. |
| A field service is running but navigation is blocked | Runtime readiness, not compilation | Stop treating it as a build issue; inspect localization, map artifacts, route preview, and safety via `scripts/lingtu`. |

## Related references

- [Get Started](./README.md) — staged onboarding and motion boundaries.
- [Quick Start](../QUICKSTART.md) — profile selection, lifecycle, and field
  preflight.
- [`scripts/build/README.md`](../../scripts/build/README.md) — detailed
  build-script behavior and specialized native flows.
- [Thunder deployment](../04-deployment/README.md) — service installation,
  release, diagnostics, and recovery.
- [Architecture index](../architecture/README.md) — the current contracts that
  native artifacts must satisfy.
