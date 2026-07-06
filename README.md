# LingTu

LingTu is a module-first autonomous navigation runtime for quadruped robots.
It targets Thunder/S100P class field robots while keeping simulation, replay,
and legacy compatibility paths explicit.

The product runtime is not a ROS 2 graph. The field path is native DDS plus
LingTu Modules. ROS 2 code remains only where a compatibility adapter or
isolated algorithm check explicitly says so.

## Current Product Contract

- Runtime unit: `Module`.
- Orchestration unit: `Blueprint`.
- Product field endpoint: `thunder_field`.
- Field transport: native DDS.
- Field navigation owner: C++ `lingtu-nav-dds`.
- Field SLAM owner: C++ `lingtu-slam-dds`.
- Robot hardware boundary: Thunder/S100P services, not direct Python motor
  control.
- ROS 2: compatibility and algorithm replay only, not the default product API.

The root contract is documented in
[`docs/architecture/PRODUCT_MODE_RUNTIME_CONTRACT.md`](docs/architecture/PRODUCT_MODE_RUNTIME_CONTRACT.md).
Architecture documents must name Module boundaries first and backend details
second.

## Runtime Dataflow

### Field Navigation

```text
Web / CLI / MCP
  -> GatewayModule / MCPServerModule / SemanticPlannerModule
  -> nav.mission goal/status contract
  -> native field endpoint boundary
  -> lingtu-nav-dds
  -> DDS /nav/cmd_vel
  -> robot controller
```

The default `thunder_field` branch intentionally avoids duplicate velocity
writers. Python keeps mission state, goals, maps, status, semantic planning,
and safety contracts. The native navigation endpoint owns the final field
`/nav/cmd_vel` writer.

### Sensor, SLAM, And Map Inputs

```text
Livox MID-360 / IMU or simulator endpoint
  -> native SLAM endpoint
  -> SlamAdapterModule odometry + map_cloud + localization_status
  -> Occupancy / Voxel / ESDF / Elevation / Traversability layers
  -> Gateway, mission, planner, exploration, safety
```

`map_cloud` is the SLAM point-cloud product. `fused_cost` is the navigation
safety costmap produced from occupancy, ESDF proximity, elevation, and
traversability. UI images, raw point clouds, costmaps, and saved-map artifacts
are different contracts and should not be collapsed into one "map" concept.

### Module-Owned Local Navigation

Simulation, local-driver, and compatibility profiles can still run the
in-process navigation chain:

```text
nav.mission
  -> nav.local_planner
  -> nav.path_follower
  -> nav.velocity_mux
  -> nav.out or simulated driver
```

This is not the default field execution path for `thunder_field`.

## Map Save Contract

The product save API is `save_slam_map`. `save_pgo_map` is kept only as a
compatibility alias for old ROS 2 adapter callers.

The native saved-map flow is:

```text
Fast-LIO scan/map output
  -> keyframe patches + poses
  -> native patch pose-graph loop correction
  -> native voxel refine / cleanup
  -> optimized map.pcd + metadata
  -> occupancy / octomap / cost layers
```

A complete saved-map directory may contain:

| Artifact | Meaning |
| --- | --- |
| `map.pcd` | optimized navigation map |
| `map.raw.pcd` | raw SLAM/builder output before save-time optimization |
| `patches/*.pcd` | keyframe or scan patches used by map optimization and cleanup |
| `poses.txt` | patch poses used by map optimization and occupancy raycasting |
| `map_optimization.json` | status, loop count, refine status, point counts, schema metadata |
| `metadata.json` | map package metadata and planner artifact status |
| `occupancy.npz` | 2D occupancy/cost artifact |
| `octomap.ot` | OctoPlanner3D 3D map artifact |
| `tomogram.pickle` | optional legacy/PCT artifact |

Legacy ROS 2 PGO/HBA packages were not converted into the product DDS data
plane. Their remaining role is compatibility, offline inspection, or historical
algorithm comparison. Product map saving is owned by the native map-save
adapter and its metadata gate.

## Profile Catalog

The canonical profile catalog lives in `src/runtime/profiles/catalog/`.
Use `python lingtu.py --list` for product profiles and
`python lingtu.py --list --all` for the full registered catalog on the current
checkout.

| Profile | Primary Use |
| --- | --- |
| `teleop` | manual control through Gateway/MCP/teleop entry |
| `teleop_avoid` | manual control with live map/costmap avoidance |
| `lite` | lightweight field entry with minimal runtime surface |
| `map` | build and save a map |
| `tracking` | field target tracking through the native endpoint |
| `nav` | saved-map navigation through the native endpoint |
| `super_lio` | Super-LIO field evaluation profile |
| `super_lio_relocation` | Super-LIO relocalization evaluation profile |
| `inspection` | scheduled/patrol/semantic inspection through navigation goals |
| `explore` | wavefront frontier compatibility/debug entry |
| `tare_explore` | field exploration through TARE + native endpoint |
| `stub` | framework-only local test profile |
| `dev` | semantic pipeline with mock dependencies |
| `sim` / `portable_mujoco` | in-process MuJoCo simulation |
| `sim_nav` | pure-Python navigation simulation |
| `sim_mujoco_live` | MuJoCo LiDAR/IMU through live SLAM-style validation |
| `sim_mujoco_octo_live` | MuJoCo + OctoPlanner3D validation |
| `sim_mujoco_pct_live` | legacy/PCT MuJoCo validation |
| `sim_gazebo` | Gazebo simulation compatibility entry |
| `sim_industrial` | industrial MuJoCo scene simulation entry |
| `sim_cmu_tare` | CMU/TARE simulation compatibility entry |

Compatibility aliases such as `thunder-map`, `thunder-nav`, and
`thunder-explore` should resolve to canonical product profiles. New docs and
scripts should prefer the canonical profile names.

## Quick Start

```bash
# Local checkout
python lingtu.py --list
python lingtu.py --list --all
python lingtu.py stub
python lingtu.py sim

# uv-managed environment, when available
uv sync --locked
uv run --locked python lingtu.py --list
uv run --locked python lingtu.py stub
```

Common lifecycle commands:

```bash
python lingtu.py status
python lingtu.py health
python lingtu.py log -f
python lingtu.py stop
```

Mapping and navigation:

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

Field operations use the robot-side CLI. Run these on the S100P/sunrise board
after SSH, or through the aliases documented in
[`docs/04-deployment/lingtu_cli.md`](docs/04-deployment/lingtu_cli.md). They
are not the normal local Windows development commands.

```bash
bash scripts/lingtu status                 # robot service/session snapshot
bash scripts/lingtu map start              # switch SLAM to mapping mode
bash scripts/lingtu map save building_a    # save map bundle and artifacts
bash scripts/lingtu nav start building_a   # start saved-map navigation
bash scripts/lingtu nav goal 5 3           # send a map-frame goal
bash scripts/lingtu svc status             # systemd/native service status
```

## Composable Blueprint API

```python
from runtime.blueprint import autoconnect
from runtime.blueprints.stacks import *

system = autoconnect(
    driver("thunder", dog_host="192.168.66.190"),
    lidar(enabled=True),
    slam("localizer"),
    maps(),
    perception("bpu", "mobileclip"),
    memory(),
    planner("kimi"),
    navigation("octoplanner3d"),
    exploration("none"),
    safety(),
    gateway(5050),
).build()

system.start()
```

Module dependencies flow downward only. High layers consume lower-layer
messages through ports and wires, not direct package imports.

## Layer Model

```text
L0  Safety       SafetyRingModule, GeofenceManagerModule, CmdVelMux
L1  Hardware     Driver, CameraBridge, LiDAR, SLAM, GNSS
L2  Maps         Occupancy, Voxel, ESDF, Elevation, Traversability, MapManager
L3  Perception   Detector, Encoder, Reconstruction, SemanticMapper, memories
L4  Decision     SemanticPlanner, LLM, VisualServo
L5  Planning     NavigationModule, planner backend, waypoint/path tracking
L6  Interface    Gateway, MCP, Teleop, optional WebRTC/Rerun
```

All shared runtime APIs live under `src/runtime/`. Modules should depend on
`runtime` contracts and their own layer, not on unrelated feature packages.

## Velocity Arbitration

All Module-owned velocity candidates go through `CmdVelMux` before reaching a
driver or endpoint boundary.

| Source | Priority | Timeout |
| --- | ---: | ---: |
| Teleop joystick / Gateway / MCP | 100 | 0.5 s |
| VisualServo tracking | 80 | 0.5 s |
| Navigation recovery | 60 | 0.5 s |
| PathFollower autonomy | 40 | 0.5 s |

The mux selects the highest-priority active source and applies the configured
near-field safety/costmap checks. In the default field branch, the native
navigation service is the only DDS `/nav/cmd_vel` writer.

## Visual Servo Entry

Visual servo commands enter through Gateway or MCP and publish two different
outputs:

```text
Far target  -> goal_pose -> NavigationModule / native mission goal
Near target -> cmd_vel   -> CmdVelMux -> field or simulation command boundary
```

The runtime hot-switch entry is target/mode switching inside profiles that
already load `VisualServoModule`. It does not dynamically add VisualServo to
lightweight profiles.

## Simulation

The primary simulation path is MuJoCo with ray-cast LiDAR and simulated IMU.
The `mujoco_lidar` backend publishes XYZI hit points suitable for SLAM,
mapping, costmaps, and obstacle checks.

Simulation evidence must be read with the correct scope:

- Raw MuJoCo LiDAR/IMU checks validate simulated sensor generation.
- Fast-LIO or saved-map gates validate the SLAM/map chain on that simulated
  input.
- Sim-only odometry priors are diagnostic tools, not production localization.
- Sunrise/S100P field claims require field runtime evidence, not only local
  MuJoCo artifacts.

See [`sim/README.md`](sim/README.md) for detailed simulation commands and
legacy ROS compatibility gates.

## Backends

| Surface | Current Backends |
| --- | --- |
| Driver | `thunder`, `stub`, `sim_mujoco`, `sim_endpoint` |
| SLAM | `fastlio2`, `pointlio`, `localizer`, `bridge`, `none` |
| Detector | `yoloe`, `yolo_world`, `bpu`, `grounding_dino` |
| Encoder | `clip`, `mobileclip` |
| LLM | `kimi`, `openai`, `claude`, `qwen`, `mock` |
| Planner | `octoplanner3d` default, `pct` legacy/manual experiment |
| PathFollower | `nav_kernel`, `pure_pursuit`, `pid` |
| Exploration | `none`, `tare` |

Prefer registry-backed backend selection over direct backend imports in
business logic.

## Source Layout

```text
cli/                    CLI, profiles, REPL, daemon lifecycle
src/runtime/            Module, Blueprint, ports, registry, transports, devices
src/drivers/            Thunder, simulation, LiDAR, teleop backends
src/localization/       SLAM modules, native adapters, bridges, GNSS/NTRIP
src/nav/                Navigation, safety, maps, planning, exploration
src/perception/         Detection, encoding, reconstruction, scene perception
src/decision/           Semantic planner, goal resolver, LLM, visual servo
src/memory/             Semantic, episodic, tagged, vector, temporal memories
src/gateway/            REST, SSE, WS, MCP, teleop, runtime status
sim/                    MuJoCo worlds, scripts, validation gates
web/                    React/Vite dashboard
config/                 Robot, device, DDS, DUFOMap, semantic configuration
calibration/            Camera, IMU, LiDAR-IMU, camera-LiDAR calibration
scripts/                Build, deploy, diagnostics, robot-side operations
docs/                   Architecture, deployment, testing, plans, archives
```

## Build And Test

Framework tests:

```bash
python -m pytest src/runtime/tests/ -q
python -m pytest src/localization/tests/test_native_slam_contract.py -q
python -m pytest sim/tests/test_mujoco_saved_map_quality_gate.py -q
```

C++ navigation kernel:

```bash
cd src/nav/kernel
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/test_benchmark
./build/test_path_follower_core
```

C++ local planner:

```bash
cd src/nav/services/plan/local_planner/cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release -DLOCAL_PLANNER_CPP_BUILD_TESTS=ON
cmake --build build -j
./build/test_local_planner_core
```

S100P builds should run on the target Linux environment with the required C++
dependencies and native DDS services installed.

## Deployment Notes

The current S100P field runtime uses native services such as:

- `lingtu-livox-dds`
- `lingtu-slam-dds`
- `lingtu-nav-dds`
- `lingtu`

Normal field navigation should not source ROS 2 or a colcon overlay. Use ROS 2
only for explicit compatibility checks, old adapters, or replay gates that
document that requirement.

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

- Field readiness requires real S100P/sunrise evidence.
- MuJoCo maps can validate the software chain, but they do not prove physical
  LiDAR timing, gait stability, or calibration quality.
- Legacy ROS 2 PGO/HBA/PCT paths are compatibility or experiment surfaces
  unless a product contract explicitly selects them.
- ChromaDB, LLMs, WebRTC, Rerun, and heavy perception backends are optional.
- TARE requires its external binary/submodule build.

## Project License

This checkout does not currently declare a project-level license in a root
`LICENSE` file. Do not assume MIT/open-source redistribution rights until that
file is added. Vendored and third-party components keep their own license files
inside their subtrees.
