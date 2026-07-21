# LingTu

LingTu is an autonomous navigation runtime for quadruped robots. It connects
robot hardware, SLAM, maps, perception, semantic decision making, planning,
safety, and operator interfaces through explicit runtime modules.

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
| `Module` | A runtime component with typed input and output ports. |
| `Blueprint` | A declarative assembly of modules and wires. |
| Stack factory | A helper such as `driver()`, `slam()`, `planner()`, or `navigation()` that adds a coherent group of modules. |
| Native service | A C++ process used for high-rate work such as LiDAR ingest, SLAM, and final robot navigation output. |
| Compatibility adapter | A bridge for ROS 2, replay, simulation, or older algorithm paths. |

## Current Runtime Shape

The runtime is split by responsibility:

| Layer | Owns |
| --- | --- |
| Hardware | Robot driver, LiDAR, camera, GNSS, IMU ingress. |
| Localization | Fast-LIO2, Point-LIO, saved-map localization, SLAM status. |
| Maps | Occupancy, voxel, ESDF, elevation, traversability, saved-map artifacts. |
| Perception | Detection, embeddings, reconstruction, scene graph. |
| Decision | Semantic planner, LLM/tool loop, goal grounding, visual servo intent. |
| Navigation | Global planning, local planning, path following, mission state. |
| Safety | Stop handling, geofence, velocity arbitration. |
| Interface | CLI, Gateway, MCP, teleop, status streams. |

High layers communicate through ports, messages, and explicit wires. They should
not import lower-level implementation details directly.

## Composable Blueprint API

This API is implemented and usable. The stack factories are exported from
`runtime.blueprints.stacks`, and `autoconnect()` builds a module graph by
combining explicit wires with type/name-based port matching.

Verified locally with:

```powershell
$env:PYTHONPATH="src"
@'
from runtime.blueprint import autoconnect
from runtime.blueprints.stacks import driver, planner, safety

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
from runtime.blueprints.stacks import driver, planner, safety

system = autoconnect(
    driver("stub"),
    planner("mock"),
    safety(),
).build()

system.start()
system.stop()
```

Production profiles use the same idea, but with additional stacks for LiDAR,
SLAM, maps, perception, navigation, Gateway, and robot-side services. Hardware
and native-service stacks still require the target robot configuration and
Linux runtime dependencies; the API exists, but not every stack is expected to
start on a local Windows checkout without those services.

Common stack factories:

| Factory | Purpose |
| --- | --- |
| `driver(robot)` | Robot or simulation driver. |
| `lidar(enabled=True)` | Livox MID-360 module. |
| `slam(profile)` | SLAM/localization module or adapter. |
| `maps()` | Occupancy, voxel, ESDF, elevation, traversability, map manager. |
| `perception(detector, encoder)` | Detection, embeddings, reconstruction. |
| `memory()` | Semantic, episodic, tagged, vector, and temporal memory modules. |
| `planner(llm)` | Semantic planner, LLM module, visual servo module. |
| `navigation(planner_backend)` | Mission, global planner, local planner, path follower, command output. |
| `exploration(backend)` | Exploration supervisor or strategy integration. |
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

Mapping and navigation from the interactive CLI:

```bash
python lingtu.py map
> map save building_a
> map list

python lingtu.py nav
> map use building_a
> navigate 5 3
> go charging station
> stop
```

Robot-side operations should be run on the target Linux controller:

```bash
bash scripts/lingtu status
bash scripts/lingtu map start
bash scripts/lingtu map save building_a
bash scripts/lingtu nav start building_a
bash scripts/lingtu nav goal 5 3
bash scripts/lingtu svc status
```

## Profiles

Use `python lingtu.py --list` for the normal profile list and
`python lingtu.py --list --all` for the full registered catalog in the current
checkout.

| Profile | Purpose |
| --- | --- |
| `stub` | Framework-only local test profile. |
| `dev` | Semantic pipeline with mock dependencies. |
| `sim` | MuJoCo simulation path. |
| `sim_nav` | Pure-Python navigation simulation. |
| `map` | Build and save a map. |
| `nav` | Saved-map navigation. |
| `teleop` | Manual control through Gateway/MCP/teleop. |
| `tracking` | Visual or semantic target tracking. |
| `inspection` | Patrol and scheduled inspection workflows. |
| `explore` | Frontier exploration/debug entry. |
| `tare_explore` | TARE-style exploration integration. |

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
  -> typed DDS command topic
  -> robot controller
```

The robot-side navigation service owns the final velocity writer in the physical
runtime. Python modules keep mission state, semantic planning, maps, status,
and safety policy, but they do not directly write competing velocity commands
to the robot.

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
- [`docs/architecture/PRODUCT_MODE_RUNTIME_CONTRACT.md`](docs/architecture/PRODUCT_MODE_RUNTIME_CONTRACT.md)
- [`docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md`](docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md)
- [`docs/architecture/MAP_SERVICE_CONTRACT.md`](docs/architecture/MAP_SERVICE_CONTRACT.md)
- [`docs/04-deployment/lingtu_cli.md`](docs/04-deployment/lingtu_cli.md)
- [`docs/07-testing/ALGORITHM_VALIDATION_FLOW.md`](docs/07-testing/ALGORITHM_VALIDATION_FLOW.md)
- [`sim/README.md`](sim/README.md)

## Known Boundaries

- Real robot readiness requires evidence from the target robot, not only local
  simulation.
- MuJoCo validates software flow, but not physical LiDAR timing, calibration,
  network behavior, or gait stability.
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
