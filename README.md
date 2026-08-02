# LingTu

LingTu is an autonomous navigation system for quadruped robots. A compiled
Product declares native field processes plus one Python Host; typed DDS joins
the field processes, while Modules and wires organize Host-local behavior.

The physical robot path is not a ROS 2 application graph. The robot-side hot
path uses native C++ services and typed DDS for high-rate sensor, SLAM, and
navigation data. ROS 2 remains available for compatibility, replay, and
algorithm checks, but it is not the default product API.
ROS2 Humble is optional for compatibility services, not a requirement for the
native Thunder product path.

## What LingTu Does

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

The main runtime concepts are:

| Concept | Meaning |
| --- | --- |
| `env` | The outer runtime environment: exactly `real` or `sim`. A simulator backend is internal env configuration. |
| `Product` | An env-independent declaration of one mode's Host graph, logical native roles, topics, and capabilities. |
| `RunPlan` | The immutable, fingerprinted result of resolving one Product inside one env. |
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

Status: current product contract as of 2026-07-28.

The runtime is split by responsibility:

| Layer | Owns |
| --- | --- |
| Hardware | Robot driver, LiDAR, camera, GNSS, IMU ingress. |
| Localization | Fast-LIO2, Point-LIO, saved-map localization, SLAM status. |
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

Local framework checks:

```bash
python lingtu.py --list
python lingtu.py stub
python lingtu.py dev
python lingtu.py sim
```

Common lifecycle commands:

```bash
python lingtu.py status
python lingtu.py health
python lingtu.py log -f
python lingtu.py stop
```

`lingtu.py` is the local application entry for development and simulation.
ProductControl-managed field products intentionally reject direct startup so they
cannot run with missing native processes.

Robot-side operations should be run on the target Linux controller:

```bash
bash scripts/lingtu status
bash scripts/lingtu map start
bash scripts/lingtu map save building_a
bash scripts/lingtu nav start building_a
bash scripts/lingtu nav goal 5 3
bash scripts/lingtu svc status
```

## Runtime Selections

Use `python lingtu.py --list` for the normal profile list and
`python lingtu.py --list --all` for the full registered catalog in the current
checkout.

Profiles are configuration inputs. Field Products are operator-managed runtime
contracts; the two concepts share resolver machinery but are not one
architecture category.

### Local And Validation Profiles

| Profile | Purpose |
| --- | --- |
| `stub` | Framework-only local test profile. |
| `dev` | Semantic pipeline with mock dependencies. |
| `lite` | Local Thunder hardware diagnostic Host; no independent systemd lifecycle. |
| `sim` | MuJoCo simulation path. |
| `sim_nav` | No-ROS navigation simulation; uses native planner extensions when built. |
| `portable_mujoco` | Portable no-ROS MuJoCo planning/sensor path. |
| `sim_mujoco_live`, `sim_mujoco_octo_live` | Legacy MuJoCo validation harnesses shown only by `--list --all`. |

### Field Products

`teleop`, `teleop_avoid`, `map`, `explore`, `nav`, `tracking`, and
`inspection` are declared under
`config/runtime_graph/products/` and operated through ProductControl. Each
compiled Product includes native process roles and one Blueprint-owned Host
graph.

TARE remains an internal exploration policy; it is not a separate Product.

Some compatibility and validation profiles are intentionally visible only in
the full catalog. They are useful for replay, ROS 2 checks, and algorithm
comparisons, but they are not the default physical-robot path.

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
  -> map optimization / cleanup
  -> optimized map.pcd + metadata
  -> occupancy / octomap / cost layers
```

Typical saved-map artifacts:

| Artifact | Meaning |
| --- | --- |
| `map.pcd` | Optimized navigation map. |
| `map.raw.pcd` | Raw SLAM or builder output. |
| `patches/*.pcd` | Keyframe/scan patches for optimization and cleanup. |
| `poses.txt` | Patch poses for optimization and occupancy raycasting. |
| `map_optimization.json` | Optimization status and schema metadata. |
| `metadata.json` | Map package metadata and planner artifact status. |
| `occupancy.npz` | 2D occupancy/cost artifact. |
| `octomap.ot` | OctoPlanner3D 3D map artifact. |
| `tomogram.pickle` | Optional legacy/PCT artifact. |

## Source Layout

```text
cli/                    CLI, profiles, REPL, daemon lifecycle
src/runtime/            Module, Blueprint, ports, registry, transports, devices
src/drivers/            Robot, simulation, LiDAR, camera, teleop backends
src/localization/       SLAM modules, native adapters, bridges, GNSS/NTRIP
src/nav/                Navigation, safety, maps, planning, exploration
src/perception/         Detection, encoding, reconstruction, scene perception
src/decision/           Semantic planner, goal resolver, LLM, visual servo
src/memory/             Semantic, episodic, tagged, vector, temporal memories
src/gateway/            REST, SSE, WebSocket, MCP, teleop, runtime status
sim/                    MuJoCo worlds, scripts, validation gates
web/                    React/Vite dashboard
config/                 Robot, device, DDS, DUFOMap, semantic configuration
calibration/            Camera, IMU, LiDAR-IMU, camera-LiDAR calibration
scripts/                Build, deploy, diagnostics, robot-side operations
docs/                   Architecture, deployment, testing, plans, archives
```

## Build And Test

Python framework and decision tests:

```bash
python -m pytest src/runtime/tests/ -q
python -m pytest src/decision/tests/ -q
```

Canonical C++ navigation core (path follower, local planner, FAR, and tests):

```bash
cmake -S src/nav/cpp -B build/nav-cpp -DCMAKE_BUILD_TYPE=Release \
  -DLINGTU_NAV_CPP_BUILD_TESTS=ON \
  -DLINGTU_NAV_CPP_BUILD_ENDPOINT=OFF \
  -DLINGTU_NAV_CPP_BUILD_PYTHON=OFF
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
- [`docs/07-testing/MUJOCO_NAVIGATION_ACCEPTANCE.md`](docs/07-testing/MUJOCO_NAVIGATION_ACCEPTANCE.md)
- [`docs/07-testing/MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md`](docs/07-testing/MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md)
- [`docs/04-deployment/lingtu_cli.md`](docs/04-deployment/lingtu_cli.md)
- [`docs/07-testing/ALGORITHM_VALIDATION_FLOW.md`](docs/07-testing/ALGORITHM_VALIDATION_FLOW.md)
- [`sim/README.md`](sim/README.md)

## Known Boundaries

- Real robot readiness requires evidence from the target robot, not only local
  simulation.
- MuJoCo validates software flow, but not physical LiDAR timing, calibration,
  network behavior, or gait stability.
- The accepted MuJoCo native-DDS navigation gate proves a bounded simulated
  navigation/control chain. It is not a substitute for field fault injection or
  a hardware motion campaign.
- ROS 2 PGO/HBA/PCT paths are compatibility or experiment surfaces unless a
  profile explicitly selects them.
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
| Point-LIO | High-bandwidth LiDAR-inertial odometry reference under `src/localization/pointlio/` and `config/pointlio.yaml`. |
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

## Project License

This checkout does not currently declare a project-level license in a root
`LICENSE` file. Do not assume MIT/open-source redistribution rights until that
file is added. Vendored and third-party components keep their own license files
inside their subtrees.
