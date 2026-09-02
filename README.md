# LingTu

<p align="center">
  <img src="docs/assets/lingtu-logo-v3.png" alt="LingTu autonomous navigation logo" width="240">
</p>

<p align="center">
  <strong>Native autonomous navigation for quadruped robots.</strong><br>
  Mapping, localization, planning, perception, simulation, and field control in one product runtime.
</p>

<p align="center">
  <a href="https://github.com/Kitjesen/MapPilot/actions/workflows/test-python.yml"><img src="https://github.com/Kitjesen/MapPilot/actions/workflows/test-python.yml/badge.svg?branch=main" alt="Python Tests and Lint"></a>
  <a href="https://github.com/Kitjesen/MapPilot/actions/workflows/nav-core-tests.yml"><img src="https://github.com/Kitjesen/MapPilot/actions/workflows/nav-core-tests.yml/badge.svg?branch=main" alt="Navigation C++ Tests"></a>
  <a href="https://github.com/Kitjesen/MapPilot/actions/workflows/native-motion-build.yml"><img src="https://github.com/Kitjesen/MapPilot/actions/workflows/native-motion-build.yml/badge.svg?branch=main" alt="Native Motion Build"></a>
  <a href="https://github.com/Kitjesen/MapPilot/actions/workflows/slam-aarch64-build.yml"><img src="https://github.com/Kitjesen/MapPilot/actions/workflows/slam-aarch64-build.yml/badge.svg?branch=main" alt="Native SLAM aarch64 Build"></a>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/version-2.3.0-00C7D9" alt="Version 2.3.0">
  <img src="https://img.shields.io/badge/field%20core-ROS--free-00A67E" alt="ROS-free field core">
  <img src="https://img.shields.io/badge/transport-CycloneDDS-18232B" alt="CycloneDDS">
  <img src="https://img.shields.io/badge/Python-%E2%89%A53.10-18232B" alt="Python 3.10 or newer">
  <img src="https://img.shields.io/badge/C%2B%2B-17-18232B" alt="C++17">
  <a href="#project-license"><img src="https://img.shields.io/badge/license-not%20declared-lightgrey" alt="Project license not declared"></a>
</p>

<p align="center">
  <a href="docs/QUICKSTART.md">Quick start</a> ·
  <a href="docs/architecture/README.md">Architecture</a> ·
  <a href="docs/04-deployment/README.md">Deployment</a> ·
  <a href="docs/07-testing/README.md">Validation</a>
</p>

## Overview

LingTu is an autonomous navigation platform for quadruped robots. A compiled
Product declares native field processes plus one Python Host; typed DDS joins
the field processes, while Modules and wires organize Host-local behavior.

> [!IMPORTANT]
> **The field runtime is ROS-free by default.** LiDAR, SLAM, maps, navigation,
> final motion control, and the robot driver run as native C++ services over
> typed CycloneDDS contracts. ROS 2 Humble is optional and limited to explicit
> compatibility, replay, and algorithm-validation paths.

## At a Glance

| Item | Current contract |
| --- | --- |
| Version | `2.3.0` (`VERSION` and `pyproject.toml`) |
| Field target | S100P / RDK X5, `aarch64`, Ubuntu |
| Runtime environments | Exactly `real` and `sim` |
| Native data plane | CycloneDDS with IDL-generated typed messages |
| Main implementation | C++17 hot paths, Python 3.10+ Host and semantic/API layer |
| Simulation | MuJoCo 3.10 physics; optional Unreal presentation workspace |
| ROS status | Not required by the production field runtime; compatibility only |
| Operating Products | `teleop`, `teleop_avoid`, `map`, `explore`, `nav`, `tracking`, `inspection` |

## Repository Model

The repository uses four independent axes; directory names do not duplicate
them:

| Path or value | Meaning |
| --- | --- |
| `src/` | Function-owned production source: localization, maps, navigation, perception, drivers, runtime, and Product control. |
| `env=real` / `env=sim` | The two ProductControl runtime environments, resolved from `config/runtime_graph/envs/`. They are not source roots. |
| `sim/` | The simulation workspace and `sim.*` Python namespace: packages, sessions, runtime, evaluation, tools, and tests. |
| `build/` | Ignored developer compilation trees. Production paths must not point here. |
| `install/` | Ignored staged installs at `install/<platform>-<arch>/<config>/{bin,lib,etc,share}`. |
| `dist/` | Ignored final release archives and manifests. |

There is intentionally no root `real/`, `environments/`, or tracked `bin/`.
Python commands are generated from `pyproject.toml`; native release commands
are installed into `bin/` under the selected install or release prefix.

## Key Capabilities

- Native LiDAR, IMU, camera, GNSS, SLAM, map, navigation, and driver endpoints.
- Saved-map localization, 2D occupancy, 3D OctoMap, global planning, and local avoidance.
- CMU and SCAN local-planning backends behind one Product-selected navigation runtime.
- Assisted teleoperation, autonomous navigation, exploration, tracking, and inspection modes.
- MuJoCo sensor/control simulation with the same typed command and data contracts used by field services.
- Gateway, Web UI, MCP, semantic planning, perception, memory, and mission-level orchestration.

## Runtime Pipeline

```text
LiDAR / IMU / camera
  -> SLAM and localization
  -> map and cost layers
  -> perception and semantic memory
  -> decision and mission control
  -> global planning and local control
  -> safety arbitration
  -> robot command boundary
```

### Core Concepts

The main runtime concepts are:

| Concept | Meaning |
| --- | --- |
| `env` | The outer runtime environment: exactly `real` or `sim`. A simulator backend is internal env configuration. |
| `Product` | An env-independent declaration of one mode's Host graph, logical native roles, topics, and capabilities. |
| `RunPlan` | The immutable result of resolving one Product inside one env. |
| `ProductControl` | The only apply, switch, quiesce, restart, stop, readiness, and rollback transaction inside one fixed env. |
| `Host` | The Python process containing Gateway, Agent, MCP, semantic logic, and selected adapters. |
| `Module` | A Host-local runtime component with typed input and output ports. |
| `Blueprint` | A declarative assembly of Modules and wires inside one Host process. |
| Stack factory | A helper such as `driver()`, `slam()`, `planner()`, or `navigation()` that adds a coherent group of modules. |
| Native service | A C++ process used for high-rate work such as LiDAR ingest, SLAM, and final robot navigation output. |
| Compatibility adapter | A bridge for ROS 2, replay, simulation, or older algorithm paths. |
| `RobotConfig` | Static physical robot/device/calibration data used internally by the real env, never a runtime selector. |
| Endpoint | A concrete HTTP, DDS, or native-service communication access point, never a deployment identity. |

## Current Runtime Shape

Status: current product contract for LingTu 2.3.0 as of 2026-08-31.

The runtime is split by responsibility:

| Layer | Owns |
| --- | --- |
| Hardware | Robot driver, LiDAR, camera, GNSS, IMU ingress. |
| Localization | Fast-LIO2, saved-map localization, SLAM status. |
| Maps | Native `mapd` live layers/scene and native saved-map records/artifacts; Python layers are development/simulation compatibility. |
| Perception | Detection, embeddings, reconstruction, scene graph. |
| Decision | Semantic planner, LLM/tool loop, goal grounding, visual servo intent. |
| Navigation | Native `navd` owns field planning, local avoidance, tracking, task state, and final logical velocity; Python navigation is development/simulation compatibility. |
| Safety | Native endpoint and driver gates own field command safety; Host Modules cover local/simulation policy. |
| Interface | CLI, Gateway, MCP, teleop, status streams. |

High layers communicate through ports, messages, and explicit wires. They should
not import lower-level implementation details directly.

## Local/Host Blueprint API

This API is implemented and usable. The stack factories are exported from
`lingtu.assembly.stacks`, and `autoconnect()` builds a module graph by
combining explicit wires with type/name-based port matching.

Verified locally with:

```powershell
$env:PYTHONPATH="src"
@'
from runtime.blueprint import autoconnect
from lingtu.assembly.stacks import driver, planner, safety

system = autoconnect(
    driver("stub"),
    planner("mock"),
    safety(),
).build()

print(system.health())
'@ | python -
```

Example API:

```python
from runtime.blueprint import autoconnect
from lingtu.assembly.stacks import driver, planner, safety

system = autoconnect(
    driver("stub"),
    planner("mock"),
    safety(),
).build()

system.start()
system.stop()
```

Field Products are resolved inside `env=real` into RunPlans and applied by
ProductControl through its internal systemd runner. Blueprint does not start LiDAR, SLAM, mapd,
navd, traversability, or
the driver; it only materializes the Host graph. The factories below remain
useful for local development, simulation, and the Host portion of a Product.

Common stack factories:

| Factory | Purpose |
| --- | --- |
| `driver(robot)` | Robot or simulation driver. |
| `lidar(enabled=True)` | Livox MID-360 module. |
| `slam(profile)` | SLAM/localization module or adapter. |
| `maps()` | Development/simulation map layers plus the low-rate Host map adapter. |
| `perception(detector, encoder)` | Detection, embeddings, reconstruction. |
| `memory()` | Semantic, episodic, tagged, vector, and temporal memory modules. |
| `planner(llm)` | Semantic planner, LLM module, visual servo module. |
| `navigation(planner_backend)` | Development/simulation navigation Modules; field Products use native `navd`. |
| `exploration(backend)` | `none` or TARE; wavefront is enabled from the navigation stack, not this factory. |
| `safety()` | Safety ring, geofence, velocity mux. |
| `gateway(port)` | REST, SSE, WebSocket, MCP, teleop/status surface. |

## Quick Start

Resolve a simulation Product without starting it:

```bash
python -m lingtu.control switch teleop --robot doso/thunder_v4 --env sim --dry-run --json
```

Common lifecycle commands:

```bash
python -m lingtu.control switch teleop --robot doso/thunder_v4 --env sim
python -m lingtu.control status --robot doso/thunder_v4 --env sim --json
python -m lingtu.control stop --robot doso/thunder_v4 --env sim
```

`python -m lingtu.control` is the local and robot-side Product lifecycle entry.

Robot-side operations should be run on the target Linux controller:

```bash
bash scripts/lingtu --robot unitree/go2 --env real status --json
bash scripts/lingtu --robot unitree/go2 --env real switch map
bash scripts/lingtu --robot unitree/go2 --env real switch nav --map building_a
bash scripts/lingtu --robot unitree/go2 --env real stop
```

## Runtime Selections

`teleop`, `teleop_avoid`, `map`, `explore`, `nav`, `tracking`, and
`inspection` are declared under
`config/runtime_graph/products/` and operated through ProductControl. Each
compiled Product includes native process roles and one Blueprint-owned Host
graph.

TARE remains an internal exploration policy; it is not a separate Product.

MuJoCo component scripts are explicit developer tools. They do not introduce a
second lifecycle or a third environment beyond `real` and `sim`.

## Runtime Dataflow

### Robot Navigation

```text
Web / CLI / MCP
  -> Gateway and semantic planner
  -> mission goal/status
  -> native navigation service
  -> logical /nav/cmd_vel (typed DDS wire topic rt/nav/cmd_vel)
  -> lingtu-driver
  -> remote Brainstem gRPC
```

In `env=real`, the native navigation service owns logical `/nav/cmd_vel`, encoded on DDS as
`rt/nav/cmd_vel`, and the unique `lingtu-driver` service forwards checked
commands to the remote Brainstem controller. Python modules
keep mission state, semantic planning, maps, status, and safety policy, but
they do not start a competing robot velocity writer in the field default path.
The driver will not become ready unless Brainstem is remote, mutually
authenticated when TLS is configured, lease-owned by `lingtu-driver`, motor
output is enabled, and a checked zero command has been acknowledged.

### Sensor, SLAM, And Maps

```text
Livox MID-360 / IMU / simulator
  -> native SLAM service
  -> odometry + registered cloud + map cloud + localization health
  -> map layers
  -> Gateway, navigation, safety, exploration, semantic consumers
```

`map_cloud`, costmaps, UI images, and saved-map artifacts are separate
interfaces. Do not collapse them into one generic "map" concept.

### Saved Map Flow

```text
SLAM scan/map output
  -> keyframe patches + poses
  -> native map cleanup
  -> occupancy / octomap / cost artifact build
  -> canonical <map_root>/<map_id>/ map package
```

Typical saved-map artifacts:

| Artifact | Meaning |
| --- | --- |
| `map.pcd` | Cleaned navigation map and source for derived artifacts. |
| `patches/*.pcd` | Keyframe/scan patches used by cleanup and artifact builders. |
| `poses.txt` | Patch poses used for occupancy raycasting. |
| `metadata.json` | Map package metadata and planner artifact status. |
| `occupancy.npz` | 2D occupancy/cost artifact. |
| `octomap.ot` | OctoPlanner3D 3D map artifact. |

## Source Layout

```text
src/lingtu/             ProductControl, RunPlan, real/sim lifecycle, assembly
src/runtime/            Module, Blueprint, ports, registry, transports
src/drivers/            Robot, simulation, LiDAR, camera, teleop backends
src/localization/       SLAM modules and native localization adapters
src/nav/                Host navigation surface and native C++ navigation
src/maps/               Saved maps, map layers, artifacts, native map adapters
src/explore/            Wavefront/TARE exploration algorithms and adapters
src/perception/         Detection, encoding, reconstruction, scene perception
src/decision/           Semantic planner, goal resolver, LLM, visual servo
src/memory/             Semantic, episodic, tagged, vector, temporal memories
src/gateway/            REST, SSE, WebSocket, MCP, teleop, runtime status
sim/                    MuJoCo worlds, scripts, validation gates
web/                    React/Vite dashboard
config/                 Robot, device, DDS, DUFOMap, semantic configuration
tools/calibration/      Offline camera, IMU, LiDAR-IMU, camera-LiDAR calibration
scripts/                Build, deploy, diagnostics, robot-side operations
docs/                   Architecture, deployment, testing, plans, and worklogs
```

## Build And Test

Python framework and decision tests:

```bash
python -m pytest tests/runtime/ -q
python -m pytest src/decision/tests/ -q
```

Canonical C++ navigation core (path follower, local planner, FAR, and tests):

```bash
cmake -S src/nav/cpp -B build/nav-cpp -DCMAKE_BUILD_TYPE=Release \
  -DLINGTU_NAV_CPP_BUILD_TESTS=ON \
  -DLINGTU_NAV_CPP_BUILD_ENDPOINT=OFF
cmake --build build/nav-cpp -j
ctest --test-dir build/nav-cpp --output-on-failure
```
Robot builds should run on the target Linux environment with the required
native dependencies and DDS services installed.

## Documentation

Start here:

- [`docs/architecture/README.md`](docs/architecture/README.md)
- [`docs/architecture/FIELD_PRODUCTS.md`](docs/architecture/FIELD_PRODUCTS.md)
- [`docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md`](docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md)
- [`docs/architecture/MAP_SERVICE_CONTRACT.md`](docs/architecture/MAP_SERVICE_CONTRACT.md)
- [`docs/07-testing/simulation/MUJOCO_NAVIGATION_ACCEPTANCE.md`](docs/07-testing/simulation/MUJOCO_NAVIGATION_ACCEPTANCE.md)
- [`docs/07-testing/simulation/MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md`](docs/07-testing/simulation/MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md)
- [`docs/04-deployment/lingtu_cli.md`](docs/04-deployment/lingtu_cli.md)
- [`sim/README.md`](sim/README.md)

## Known Boundaries

- Real robot readiness requires evidence from the target robot, not only local
  simulation.
- MuJoCo validates software flow, but not physical LiDAR timing, calibration,
  network behavior, or gait stability.
- The accepted MuJoCo native-DDS navigation gate proves a bounded simulated
  navigation/control chain. It is not a substitute for field fault injection or
  a hardware motion campaign.
- ChromaDB, LLMs, WebRTC, Rerun, and heavy perception backends are optional.
- TARE requires its external binary/submodule build when using the external
  runtime.

## Referenced Projects And Acknowledgements

LingTu is an integration-heavy robotics runtime. The table below names the main
external projects, algorithms, and systems that this checkout references,
adapts, vendors, or uses as validation inspiration. It is not a substitute for
the license files inside vendored subtrees.

| Project | LingTu role |
| --- | --- |
| [DimOS](https://github.com/dimensionalOS/dimos) | Evidence bar and benchmark inspiration for simulation closure, runtime dataflow checks, and readiness gating. LingTu uses DimOS-style validation ideas; DimOS is not a runtime dependency. |
| FAST-LIO2 | LiDAR-inertial odometry and mapping reference under `src/localization/fastlio2/`, with LingTu wrapping the runtime boundary for native/DDS use. |
| Livox SDK2 / livox_ros_driver2 | MID-360 LiDAR device protocol, SDK integration, and ROS compatibility source under `src/drivers/real/lidar/`. |
| CycloneDDS | Typed DDS transport used by native services and message definitions. |
| OctoPlanner3D / OctoMap | 3D saved-map global planning backend and occupancy-map runtime. |
| TARE / CMU exploration stack | Exploration architecture and compatibility target; the external CMU runtime is not vendored as a normal Python dependency. |
| DUFOMap | Optional saved-map cleanup and dynamic-obstacle filtering reference used by the map-save pipeline. |
| MuJoCo | Simulation backend for local validation, sensor generation, and route execution gates. |
| MobileCLIP / CLIP, YOLO-World / YOLOE, Grounding DINO | Semantic perception and open-vocabulary perception backends used behind registry-selected modules. |
| ROS 2 Humble | Compatibility, replay, and algorithm-check surface only; not the default runtime API. |

Thanks to the DimOS project in particular for setting a stronger standard for
robotics evidence: claims should be backed by explicit runtime gates, dataflow
proof, and reproducible artifacts instead of only code-path inspection.

## Contributing

1. Read [`AGENTS.md`](AGENTS.md) and the
   [development guide](docs/03-development/README.md).
2. Keep field hot paths native and ROS-free; add ROS only through an explicit
   compatibility boundary.
3. Run the narrowest affected tests first, then the relevant native, Python,
   Web, or simulation gate.
4. Do not claim real-robot readiness without target-side S100P evidence.
5. Open changes from a focused branch and include known validation gaps.

## Project License

This checkout does not currently declare a project-level license in a root
`LICENSE` file. Do not assume MIT/open-source redistribution rights until that
file is added. Vendored and third-party components keep their own license files
inside their subtrees.
