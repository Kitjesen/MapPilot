# AGENTS.md

This file provides guidance to Codex when working in this repository.

## Project Overview

LingTu (灵途) is an autonomous navigation system for quadruped robots in
outdoor/off-road environments.

- **Platform**: S100P (RDK X5, Nash BPU 128 TOPS, aarch64), native CycloneDDS, Ubuntu Linux
- **Languages**: Python for the Host framework and semantic/API stack; C++ for field LiDAR, SLAM, maps, terrain, navigation, and driver hot paths
- **Architecture**: Assembly resolves one Product inside one fixed `env` (`real` or `sim`) into a fingerprinted RunPlan; ProductControl owns switching and systemd effects; Blueprint materializes only the Python Host graph
- **Canonical architecture guide**: `docs/architecture/README.md`

## Working Rules

- Keep orchestration scopes separate. `lingtu.assembly` resolves Product + env without side effects. `lingtu.control.ProductControl` is constructed for one fixed env, publishes one immutable RunPlan, stages the transient session, owns product-level operations, and invokes the internal `lingtu.systemd.SystemdRunner`. Blueprint owns the Python Host graph and optional Python workers. Product source is ROS-free: use native typed DDS at process boundaries and direct calls inside a C++ service.
- Field mode changes must call `python -m lingtu.control switch` directly or through the thin `scripts/lingtu` adapter. Do not add product-switch policy, systemd ordering, map activation, or readiness loops back to Bash or Gateway.
- Prefer existing factories, registries, modules, and utilities before adding new abstractions.
- Keep diffs small, reversible, and behavior-preserving unless the task explicitly requests a behavior change.
- No new dependencies without an explicit request.
- Python comments in new code should be English. Chinese comments exist in legacy code and may remain.
- Preserve user or teammate changes already present in the worktree.
- Verify before claiming completion. Run the narrowest useful tests for the touched surface.

## Fixed Runtime Vocabulary

Use these terms with exactly one meaning:

| Term | Owns | Does not own |
| --- | --- | --- |
| `env` | The outer runtime environment. Public values are exactly `real` and `sim`; a simulation backend is internal env configuration. | Product behavior or a communication endpoint. |
| `Product` | Immutable, env-independent declaration of one operating mode: Host graph, logical native roles, topics, and capabilities. | Deployment targets or runtime side effects. |
| `RunPlan` | Fingerprinted resolution of one Product inside one env, including concrete processes and Host configuration. | User-authored mode selection, mutable runtime state, or side effects. |
| `ProductControl` | The complete switch/quiesce/restart/stop transaction, transient session, readiness, rollback, and current RunPlan commit inside its fixed env. | Env switching or domain algorithms. |
| `Host` | The managed Python application process containing Gateway, Agent, adapters, and selected Python Modules. | Native LiDAR/SLAM/navigation process ownership. |
| `Blueprint` | Constructing and wiring Modules inside one Host process. | Product switching, systemd, native endpoints, or DDS topology ownership. |
| `Module` | One typed in-process Host runtime unit. | Cross-process lifecycle management. |
| `DDS` | Native typed cross-process data plane. | Product policy or process lifecycle. |
| `RobotConfig` | Static physical robot, device, and calibration data referenced internally by the real env. | Runtime mode selection. |
| `Endpoint` | A concrete HTTP, DDS, or native-service communication access point. | A deployment environment or Product identity. |

Reject changes that resolve the same Product independently in CLI, systemd,
and Host. ProductControl publishes one fingerprinted RunPlan; its internal
systemd runner and Host consume that exact artifact. Do not describe Blueprint as unused: it
is active inside every Python Host, but it is not the product process
orchestrator. Do not add Product-specific Host lifecycle implementations beside
Blueprint.

## Quick Start

```bash
# Framework tests
python -m pytest src/runtime/tests/ -q

# CLI
python lingtu.py                 # interactive profile selector in a TTY
python lingtu.py --list          # list profiles
python lingtu.py stub            # framework testing only
python lingtu.py dev             # semantic pipeline on stub driver
python lingtu.py sim             # MuJoCo simulation
python lingtu.py sim_nav         # pure-Python navigation sim
# Field products are operator-managed through scripts/lingtu, not lingtu.py:
bash scripts/lingtu map start
bash scripts/lingtu nav start <map>

# Lifecycle commands
python lingtu.py status
python lingtu.py health
python lingtu.py log -f
python lingtu.py stop

# Local development overrides
python lingtu.py dev --llm mock
python lingtu.py sim_nav --planner octoplanner3d
```

## Local/Host Blueprint API

This API builds one Python Host graph for local development or for the Host
portion of a compiled Product. It does not launch field-native processes.

```python
from runtime.blueprint import autoconnect
from lingtu.assembly.stacks import *

system = autoconnect(
    driver(driver_backend="thunder"),               # local-only driver selection; real Product uses Env RobotConfig
    lidar(enabled=True),                            # Livox MID-360 hardware adapter
    slam("localizer"),                              # managed SLAM/localization
    maps(),                                         # dev/sim map layers + low-rate Host map adapter
    perception("bpu", "mobileclip"),                # detector, encoder, reconstruction
    memory(),                                       # semantic, episodic, tagged, vector, temporal memory
    planner("kimi"),                                # semantic planner, LLM, visual servo
    navigation("octoplanner3d"),                    # dev/sim autonomy; field Products use native navd
    exploration("none"),                            # "none" or TARE; wavefront lives in navigation(enable_frontier=True)
    safety(),                                       # safety ring, geofence, cmd_vel mux
    gateway(5050),                                  # REST, SSE, WS teleop/camera, MCP, WHEP proxy
).build()
system.start()
```

## Architecture

### Layer Hierarchy

```text
L0  Safety       -> SafetyRingModule + GeofenceManagerModule + CmdVelMux
L1  Hardware     -> Driver + camera + LiDAR + SLAM + GNSS
L2  Maps         -> OccupancyGrid + VoxelGrid + ESDF + ElevationMap + TraversabilityCost
L3  Perception   -> Detector + Encoder + Reconstruction + SemanticMapper + memories
L4  Decision     -> SemanticPlanner + LLM + VisualServo
L5  Planning     -> NavigationModule + planner backend + waypoint/path tracking
L6  Interface    -> Gateway + MCP + Teleop + WHEP/JPEG camera paths + optional Rerun
```

High layers may depend on lower layers only. L5 to L2 waypoint/path dispatch is
message flow, not a package dependency.

### Dependency Direction

```text
All Modules -> core/ (Module, In/Out, Registry, utils, msgs)
               ^ only legal shared dependency direction

nav/       must not import perception/, decision/, drivers/, gateway/
perception/ must not import nav/, decision/, drivers/, gateway/
decision/ may consume perception outputs through runtime messages, not direct nav/gateway imports
drivers/   must not import nav/, semantic/ except lazy blueprint registration
gateway/   must not import nav/, semantic/, drivers/
```

Use `runtime.registry.get(...)` and `@register(...)` for pluggable backends. Avoid
direct backend imports in business logic.

## Host And Development Stack Factories

These factories assemble the Python Host or local/simulation graphs. They do
not declare or launch field-native processes; ProductControl owns those.

Factories live under `src/lingtu/assembly/stacks/`.

| Factory | Purpose |
| --- | --- |
| `driver(driver_backend=...)` | Local Profile driver connection |
| `lidar(enabled=True)` | Livox LiDAR hardware adapter |
| `sim_lidar(scene_xml=...)` | Simulated point-cloud provider |
| `slam(profile)` | Managed SlamModule or native endpoint adapter |
| `maps()` | Development/simulation map layers plus the low-rate Host map adapter |
| `perception(det, enc)` | Detector, Encoder, Reconstruction |
| `memory(save_dir)` | SemanticMapper, Episodic, Tagged, Vector, Temporal, MissionLogger |
| `planner(llm)` | LLMModule, SemanticPlanner, VisualServo |
| `navigation(planner)` | Development/simulation navigation Modules; field Products use native `navd` |
| `exploration(backend)` | `none` or `tare`; wavefront was removed from this stack |
| `safety()` | SafetyRing, Geofence, CmdVelMux |
| `gateway(port)` | GatewayModule, MCPServerModule, TeleopModule, WHEP proxy, optional Rerun |

## Pluggable Backends

| Surface | Backends |
| --- | --- |
| Driver | `thunder`, `stub`, `sim_mujoco`, `sim_endpoint` |
| SLAM | `fastlio2`, `pointlio`, `localizer`, `bridge`, `none` |
| Detector | `yoloe`, `yolo_world`, `bpu`, `grounding_dino` |
| Encoder | `clip`, `mobileclip` |
| LLM | `kimi`, `openai`, `claude`, `qwen`, `mock` |
| Planner | `octoplanner3d` default; `pct` legacy/manual experiment |
| PathFollower | `nav_kernel`, `pid` |
| Exploration | `none`, `tare` |

## Runtime Selection Inputs

Local Profile and Host defaults live under `src/runtime/profiles/catalog/`.
Product declarations live only under `config/runtime_graph/products/`; env
implementations live only under `config/runtime_graph/envs/`.

Profile is a configuration-resolver input; it is not a runtime ownership
category. Do not infer architecture from an arbitrary sample of profile names.
Keep local/development profiles separate from operator-managed field Products.

### Local And Development Profiles

| Profile | Driver backend | SLAM | LLM | Planner | Semantic | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| `stub` | `stub` | `none` | `mock` | `octoplanner3d` | no | framework tests |
| `dev` | `stub` | `none` | `mock` | `octoplanner3d` | yes | semantic pipeline dev |
| `lite` | `thunder` | `none` | `mock` | `direct` | no | local Thunder hardware diagnostics; no independent systemd lifecycle |
| `sim` | `sim_mujoco` | `bridge` | `mock` | `octoplanner3d` | yes | MuJoCo + native stack |
| `sim_nav` | `stub` | `none` | `mock` | `octoplanner3d` | no | pure-Python nav sim |

### Field Products

The operator-managed Product set is `teleop`, `teleop_avoid`, `map`,
`explore`, `nav`, `tracking`, and `inspection`.
Their authoritative declarations are
`config/runtime_graph/products/*.yaml`. ProductControl resolves one of them
inside its fixed `real` or `sim` env into a RunPlan. `sim` requires an
explicit internal backend when more than one implementation is available.
Physical robot data comes from `config/robot_config.yaml`; it is not a public
selector and does not participate in runtime identity.

## Source Map

| Path | Role |
| --- | --- |
| `cli/` | argparse CLI, profiles, REPL, daemon lifecycle, external status commands |
| `src/runtime/` | Module, Blueprint, streams, transports, registry, devices, utils, framework tests |
| `src/nav/` | Navigation mission, global/local planning, realtime safety, endpoint execution |
| `src/maps/` | Persistent map assets, map packages, artifact builders, reusable map layers, MapsModule |
| `src/perception/` | perception, tracking, reconstruction, scene understanding |
| `src/decision/` | semantic planner, LLM, visual servo, goal resolution |
| `src/memory/` | semantic map, episodic/tagged/vector/temporal memories, KG |
| `src/drivers/` | Thunder driver, simulation drivers, LiDAR, teleop |
| `src/gateway/` | FastAPI gateway, MCP server, SSE/WS endpoints |
| `src/nav/local/` | terrain, local planner, path follower Python modules and local autonomy backends |
| `src/nav/services/plan/` | Planner service boundary, OctoPlanner3D backend, and explicit PCT legacy backend |
| `src/localization/` | Fast-LIO2, Point-LIO, PGO, localizer, GNSS bridge, NTRIP client |
| `src/explore/` | Canonical wavefront/TARE exploration algorithms and supervisor |
| `src/message/` | Typed CycloneDDS message contracts, IDL schemas, codec, and generated stubs |
| `src/native/` | C++ native runtime: recording (MCAP), component manifest, boundary registry |
| `tools/calibration/` | offline camera, IMU, LiDAR-IMU, and camera-LiDAR calibration tools |
| `config/` | robot/device/DDS/DUFOMap/semantic configuration |
| `launch/` | algorithm bridge launch files only |
| `scripts/` | build, deploy, OTA, diagnostics, robot-side `lingtu` ops CLI |
| `web/` | React/Vite dashboard |
| `docs/` | current docs plus archived architecture and design material |

## Key Files

| File | Purpose |
| --- | --- |
| `lingtu.py` | primary local Host/Profile CLI entry |
| `src/runtime/profiles/catalog/` | local Profile and Host-default source of truth |
| `config/runtime_graph/envs/` | `real` and `sim` implementation mapping source of truth |
| `config/robot_config.yaml` | physical `RobotConfig` referenced by the real env |
| `src/runtime/profiles/catalog/host_defaults.py` | unified, explicitly classified Host Blueprint defaults |
| `src/lingtu/assembly/profile_builder.py` | profile-to-Blueprint assembly |
| `src/lingtu/run_plan.py` | immutable, fingerprinted Product resolution consumed by systemd and Host |
| `src/lingtu/control.py` | Product switch, quiesce, restart, stop, rollback, and current-plan commit |
| `src/lingtu/systemd.py` | internal process execution and readiness implementation used only by ProductControl |
| `src/lingtu/assembly/full_stack_wiring.py` | critical explicit full-stack wires |
| `src/lingtu/assembly/stacks/` | composable stack factories |
| `src/runtime/module.py` | Module base class |
| `src/runtime/stream.py` | `In[T]` / `Out[T]` ports and backpressure policies |
| `src/runtime/blueprint.py` | Blueprint, autoconnect, explicit wire support |
| `src/runtime/registry.py` | plugin registry |
| `src/runtime/devices/` | `hw` module and hardware registry |
| `src/runtime/utils/calibration_check.py` | startup calibration self-check |
| `src/nav/navigation.py` | mission FSM, global planning, and recovery |
| `src/nav/services/plan/global_planner/service.py` | OctoPlanner3D global planner dispatch, map artifact gate, and safe-goal search |
| `src/nav/services/safety/safety_ring.py` | safety evaluator and reflexes |
| `src/nav/services/safety/velocity_mux.py` | Python Module/simulation velocity arbitration |
| `src/maps/modules/service.py` | map lifecycle Module facade over native map services |
| `src/maps/services/` | persistent map query/control/storage/build application services |
| `src/maps/artifacts.py` | saved-map metadata, hash, frame, and same-source validation |
| `src/maps/services/pipeline.py` | saved-map build pipeline and native prune-command boundary |
| `src/maps/prune/cpp/` | native save-time dynamic-map pruning implementation |
| `src/decision/goals/resolver.py` | fast/slow goal resolution |
| `src/decision/modules/visual_servo.py` | bbox/depth visual servo |
| `src/decision/tasks/agent.py` | multi-turn tool-calling agent loop |
| `src/memory/modules/semantic_mapper_module.py` | scene graph to semantic/topological maps |
| `src/memory/modules/vector_memory_module.py` | CLIP/ChromaDB vector search with numpy fallback |
| `src/localization/slam/module.py` | managed Fast-LIO2/Point-LIO/localizer mode |
| `src/gateway/gateway_module.py` | FastAPI REST/WS/SSE, map save hooks, drift watchdog |
| `scripts/lingtu` | robot-side unified operations CLI |
| `scripts/build/build_dufomap.sh` | aarch64 DUFOMap build helper |
| `scripts/build/build_explore_kernel.sh` | native exploration/TARE policy kernel build helper |
| `config/robot_config.yaml` | physical robot and calibration source of truth |
| `config/devices.yaml` | hardware device registry |
| `config/dufomap.toml` | LingTu-tuned DUFOMap config |
| `docs/04-deployment/lingtu_cli.md` | DUFOMap operations and restore notes |

## Build And Test

```bash
# Python framework tests
python -m pytest src/runtime/tests/ -q

# Useful focused tests
python -m pytest src/runtime/tests/test_calibration_check.py -q
python -m pytest src/runtime/tests/test_localization_health.py -q
python -m pytest src/runtime/tests/test_scene_mode_detector.py -q

# Canonical portable C++ navigation tests
cmake -S src/nav/cpp -B build/nav-cpp -DCMAKE_BUILD_TYPE=Release \
  -DLINGTU_NAV_CPP_BUILD_TESTS=ON \
  -DLINGTU_NAV_CPP_BUILD_ENDPOINT=OFF \
  -DLINGTU_NAV_CPP_BUILD_PYTHON=OFF
cmake --build build/nav-cpp -j
ctest --test-dir build/nav-cpp --output-on-failure

# Native C++ runtime build on S100P
bash scripts/build/build_nav_endpoint.sh
```

Known C++ caveat: `test_validation` has expected NaN/Inf IEEE-compliance
failures under `-ffast-math`.

## C++ Performance Notes

`src/nav/cpp/` is the canonical C++ algorithm/runtime tree. `src/nav/kernel/`
contains only the Python extension loader. aarch64 performance is critical.

Key optimizations:

- SoA memory layout in `local_planner.hpp`
- CSR sparse layout in `local_planner.hpp`
- `scorePathFast` LUT in `local_planner_scoring.hpp`
- OpenMP path scoring and terrain parallelism
- LTO in CMake paths; relaxed floating-point math is not enabled globally

## Critical Files

Be especially careful with these shared surfaces:

- `src/runtime/module.py`
- `src/runtime/blueprint.py`
- `src/runtime/stream.py`
- `src/runtime/registry.py`
- `src/runtime/utils/`
- `src/lingtu/assembly/profile_builder.py`
- `src/lingtu/assembly/full_stack_wiring.py`
- `src/perception/.../instance_tracker.py`
- `src/decision/goals/resolver.py`
- `src/nav/navigation.py`
- `src/nav/services/safety/safety_ring.py`
- `config/robot_config.yaml`
- `config/devices.yaml`

## Backpressure Policies

```python
self.image.set_policy("latest")                   # drop if busy
self.imu.set_policy("throttle", interval=0.02)    # max 50 Hz
self.lidar.set_policy("sample", n=5)              # every fifth message
self.detections.set_policy("buffer", size=10)     # batch of 10
```

## Transport Decoupling

```python
bp.wire("SafetyRingModule", "stop_cmd", "Driver", "stop_signal")
bp.wire("PerceptionModule", "scene_graph", "SemanticPlannerModule", "scene_graph", delivery="dds")
bp.wire("SLAMModule", "map_cloud", "TerrainModule", "map_cloud", delivery="shm")
```

Use explicit wires for critical fan-in/fan-out or ambiguous port names. See
`src/lingtu/assembly/full_stack_wiring.py`.

## Explicit Host/Simulation Wires

The wires below describe Blueprint-owned graphs. Field-native LiDAR, SLAM,
mapd, traversability, navd, and driver edges are declared by Product topics and
are not duplicate Blueprint wires.

Important explicit wires include:

- Safety stop to driver and NavigationModule
- SLAM or driver odometry into NavigationModule, Gateway, map layers, safety, and semantic consumers
- SLAM `map_cloud` into occupancy, voxel, elevation, terrain, rerun, and gateway consumers
- Camera/depth/camera_info into perception, reconstruction, visual servo, and teleop/JPEG streaming
- Localization health from SlamModule/SlamAdapterModule into SafetyRing, NavigationModule, and DepthVisualOdom
- Gateway and MCP instruction/goal fan-in into SemanticPlannerModule and NavigationModule
- TraversabilityCost outputs into NavigationModule, LocalPlannerModule, and Gateway
- SemanticPlanner `servo_target` into VisualServo, then dual-channel `goal_pose`/`cmd_vel`
- Teleop, VisualServo, Navigation recovery, and PathFollower `cmd_vel` into CmdVelMux
- CmdVelMux `driver_cmd_vel` into the selected driver and SafetyRing monitor

## Semantic Navigation

### Goal Resolution Chain

```text
instruction
  -> tag lookup
  -> fast scene-graph + CLIP match
  -> vector memory search
  -> topology/frontier exploration
  -> visual servo
```

`goal_resolver.py` implements fast/slow resolution:

- Fast path: keyword, spatial, CLIP, and detector-confidence fusion.
- Slow path: LLM reasoning with selective grounding.
- Entropy trigger: high candidate entropy plus low confidence escalates to slow path.
- LERa recovery: Look, Explain, Replan after subgoal failure.

### Visual Servo

`visual_servo_module.py` publishes two channels:

- Far target: `goal_pose -> NavigationModule`
- Near target: `cmd_vel -> CmdVelMux -> Driver`

### Multi-Turn Agent Loop

`agent_loop.py` exposes tool-style actions:

```text
navigate_to, navigate_to_object, detect_object, query_memory,
tag_location, say, done
```

## LLM Configuration

```bash
export MOONSHOT_API_KEY="..."      # Kimi / Moonshot
export OPENAI_API_KEY="sk-..."     # OpenAI
export ANTHROPIC_API_KEY="sk-ant-" # Claude
export DASHSCOPE_API_KEY="sk-..."  # Qwen / DashScope
```

Use `mock` for offline and deterministic framework work.

## SLAM And Localization

| Mode | `slam_profile` | Backend | Use case |
| --- | --- | --- | --- |
| Mapping | `fastlio2` | SLAMModule -> Fast-LIO2 | first visit and map build |
| Localization | `localizer` | SLAMModule -> Fast-LIO2 + ICP localizer | navigate against a saved map |
| Field navigation (`env=real`) | `bridge` + `localization_adapter="cpp_slam_status"` | `CppSlamStatusAdapterModule` | the `nav` Product's real-env Host adapter; ingests C++ SLAM status/localization through the native DDS contract, with no ROS 2 dependency |
| None | `none` | no SLAM module | stub/dev/sim_nav/lite |

The `nav` Product's real-env Host uses the native `cpp_slam_status` adapter because
robot-side SLAM services own the Livox device and publish status/localization
over the field DDS endpoint. Do not switch it to managed localizer mode unless
the process ownership and sensor lifecycle are intentionally changed.

## Exploration

- `explore` is the only field exploration Product.
- `lingtu explore start` selects the live-mapping route.
- `lingtu explore start --map MAP` selects the saved-map localization route.
- TARE remains an internal/native exploration policy name and local
  compatibility backend, not another operator Product.
- The `exploration()` factory only supports `none` and `tare`; `wavefront` was
  intentionally removed from that factory.

## REPL Commands

```text
Navigation: go/navigate <target> | nav <x> <y> [yaw] | stop | cancel | status
SLAM/GNSS:  slam status|fastlio2|localizer|stop | gnss status
Map:        map list | save <name> | use <name> | build <name> | delete <name> | rename <old> <new>
Semantic:   smap status | rooms | save | load <dir> | query <text>
Memory:     vmem query <text> | vmem stats
LLM:        llm status | llm test | llm backends | chat <prompt>
Agent:      agent <multi-step instruction>
Teleop:     teleop status | teleop release
Monitor:    info | health | watch | live | module <name> | connections | log | config | history
Rerun:      rerun on | rerun off | rerun status
```

## MCP Server

`MCPServerModule` exposes auto-discovered `@skill` methods via JSON-RPC at
`http://<robot>:8090/mcp`.

Tool categories include navigation, perception, memory, semantic map, visual
servo, planning, and system health.

```bash
export LINGTU_HOST=ROBOT_IP_OR_HOSTNAME
codex mcp add --transport http lingtu http://${LINGTU_HOST}:8090/mcp
```

## Gateway And Teleop

- REST/SSE/teleop WebSocket share Gateway port `5050`.
- Teleop WebSocket: `ws://<robot>:5050/ws/teleop`.
- MCP JSON-RPC runs on port `8090`.
- Low-latency camera video uses the go2rtc WHEP proxy; `/ws/camera` remains the JPEG fallback.
- Optional Rerun visualization is enabled with `--rerun`.

Teleop joystick payload:

```json
{"type": "joy", "lx": 0.5, "ly": 0.0, "az": -0.3}
```

## Python CmdVelMux Priority

Python Module, simulation, and explicit compatibility velocity sources use
`CmdVelMux`. The `real` env performs final motion ownership inside the native
Nav service and publishes logical `/nav/cmd_vel` on DDS wire
topic `rt/nav/cmd_vel`; only `lingtu-driver` may forward it to Brainstem.

| Source | Priority | Timeout |
| --- | ---: | ---: |
| Teleop joystick | 100 | 0.5 s |
| VisualServo PD tracking | 80 | 0.5 s |
| Navigation recovery | 60 | 0.5 s |
| PathFollower autonomy | 40 | 0.5 s |

Highest-priority active source wins.

## S100P Deployment

- SSH: `ssh sunrise@<robot-ip>` (current lab address is tracked in `docs/CURRENT.md`)
- Navigation code: `~/data/SLAM/navigation/` symlinked to `~/data/inovxio/lingtu/`
- Deployment target: versioned releases under `/opt/lingtu/releases/`, with
  `/opt/lingtu/current` as the active application symlink
- CycloneDDS runtime: C++ `dds/dds.h` + `idlc` from apt `cyclonedds-dev`/`cyclonedds-tools`
  (S100P currently uses 0.8.2 successfully)
- Python: 3.10.12; `cyclonedds-python` is optional for development diagnostics and
  must not be required by the robot main path
- DUFOMap binary: `~/src/dufomap/build/dufomap_run`
- Map directory resolution: `$NAV_MAP_DIR`, then existing `~/data/nova/maps`, otherwise `~/data/lingtu/maps`

## Robot-Side Operations CLI

`scripts/lingtu` is the single entry point for field operations.

```bash
export LINGTU_HOST=ROBOT_IP_OR_HOSTNAME
alias lingtu='ssh sunrise@${LINGTU_HOST} "bash ~/data/SLAM/navigation/scripts/lingtu"'
alias lingwatch='ssh -t sunrise@${LINGTU_HOST} "bash ~/data/SLAM/navigation/scripts/lingtu watch"'
```

| Command | Purpose |
| --- | --- |
| `lingtu status` | one-shot status board |
| `lingtu watch` | continuous status watch |
| `lingtu map start|save <name>|end|list|restore <name>` | mapping lifecycle |
| `lingtu nav start <map>|stop|goal X Y [YAW]` | navigation lifecycle and goals |
| `lingtu svc status|restart <logical-process>|reapply` | read-only RunPlan process status and thin ProductControl recovery adapter |
| `lingtu log drift|dufomap|error|tail|all` | journal filters |
| `lingtu health` | raw Gateway health dump |

## Dynamic Obstacle Removal

The field runtime uses three distinct layers:

- `mapd` consumes scan-time `MapObservation` and owns live voxel column
  carving, decay, bounded accumulation, and `/maps/scene`.
- standalone native traversability owns the rolling control-risk layer and is
  the only `/nav/traversability` writer.
- save-time native pruning cleans the persistent map transaction before
  derived artifacts are committed.

`GatewayModule._on_map_cloud` and Python map layers are development/simulation
fallbacks only; they are not field map ownership and must not compete with
`mapd`.

DUFOMap reads `<map>/patches/*.pcd` plus `poses.txt`, writes a cleaned
`map.pcd`, and backs up the original as `map.pcd.predufo`.

Disable save-time filtering with:

```bash
export LINGTU_SAVE_DYNAMIC_FILTER=0
```

Reference: `docs/04-deployment/lingtu_cli.md`.

## SLAM Drift Watchdog

Gateway monitors odometry drift and restarts robot-side SLAM services when
Fast-LIO2 diverges.

Environment controls:

```bash
LINGTU_DRIFT_WATCHDOG=0
LINGTU_DRIFT_WATCHDOG_INTERVAL=60
LINGTU_DRIFT_WATCHDOG_XY_LIMIT=...
LINGTU_DRIFT_WATCHDOG_V_LIMIT=...
LINGTU_DRIFT_WATCHDOG_COOLDOWN=300
```

## Calibration

Offline calibration tools live under `tools/calibration/`; results are applied to
`config/robot_config.yaml`.

| Step | Tool | Output |
| --- | --- | --- |
| Camera intrinsics | `tools/calibration/camera/calibrate_intrinsic.py` | camera matrix and distortion |
| IMU noise | `tools/calibration/imu/allan_variance_ros2/` | noise parameters |
| LiDAR-IMU extrinsics | `tools/calibration/lidar_imu/LiDAR_IMU_Init/` | `r_il`, `t_il` |
| Camera-LiDAR extrinsics | `tools/calibration/camera_lidar/direct_visual_lidar_calibration/` | camera pose |
| Apply | `tools/calibration/apply_calibration.py` | robot_config + SLAM configs |
| Verify | `tools/calibration/verify.py` | sanity checks |

`full_stack_blueprint()` runs `run_calibration_check()` at startup. FAIL-level
issues block startup; WARN-level issues log and continue.

## Known Limitations

- Fast Path is rule/fusion based, not a learned policy.
- S100P has no CUDA; avoid Open3D GPU assumptions.
- Kimi/Qwen/OpenAI/Claude slow path depends on valid API keys.
- ChromaDB is optional; VectorMemory falls back to numpy brute-force search.
- Framework tests are mock-heavy; real hardware validation still needs S100P.
- TARE requires its external binary/submodule build.
