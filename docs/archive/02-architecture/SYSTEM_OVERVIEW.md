# System Overview

LingTu is the autonomous-navigation stack on top of the Thunder quadruped
(brainstem CMS) and the S100P compute board. This document describes the
software architecture as it is actually wired today; the source of truth
is `src/runtime/blueprints/full_stack.py` and the stack factories under
`src/runtime/blueprints/stacks/`.

## Layer model

```
L6  Interface     GatewayModule (HTTP / SSE / WS / WebRTC)
                  MCPServerModule (JSON-RPC at :8090)
                  TeleopModule (joystick + camera relay)
L5  Planning      NavigationModule (mission FSM, goal handling, replan)
                  GlobalPlannerService (A* / PCT backend)
                  WaypointTracker (arrival + stuck detection)
                  ExplorationSupervisorModule + TAREExplorerModule (optional)
L4  Decision      SemanticPlannerModule (Fast/Slow goal resolution + AgentLoop)
                  GoalResolver, FrontierScorer, ActionExecutor
                  VisualServoModule (BBox + PersonTracker)
                  LLMClient (kimi / openai / claude / qwen / mock)
L3  Perception    PerceptionModule (YOLO/BPU detector + EncoderModule)
                  ReconstructionModule, SemanticMapperModule
                  EpisodicMemory, TaggedLocations, VectorMemory, RoomObjectKG
L2  Maps          OccupancyGridModule, ESDFModule, ElevationMapModule
                  VoxelGridModule, TraversabilityCostModule
                  TerrainModule + LocalPlannerModule + PathFollowerModule
                  MapManagerModule (list / save / use / build)
L1  Hardware      ThunderDriver (gRPC â†?brainstem) / sim drivers
                  CameraBridgeModule (Orbbec / sim camera)
                  Lidar / SimLidar
                  SLAMModule + SlamBridgeModule + DepthVisualOdomModule
                  GnssModule + GnssBridgeModule (optional, RTK)
L0  Safety        SafetyRingModule, CmdVelMux, GeofenceManagerModule
                  DeviceManager (config/devices.yaml)
```

A higher layer may depend on a lower one. The `nav/`, `semantic/`,
`drivers/`, and `gateway/` Python packages do not import each other; they
all import `core/`. Cross-stack wiring is centralised in
`src/runtime/blueprints/full_stack.py`.

## Composable stacks

Each factory in `src/runtime/blueprints/stacks/` returns a `Blueprint` and is
composed with `runtime.blueprint.autoconnect`:

| Factory | Backends / behaviour |
|---------|----------------------|
| `driver(robot, **cfg)` | `thunder`, `stub`, `sim_mujoco`, `sim_ros2` (auto-detected via `runtime.registry`) |
| `lidar(ip, enabled)` / `sim_lidar(scene_xml)` | Real Livox / sim point cloud |
| `slam(profile)` | `fastlio2`, `pointlio`, `localizer`, `bridge`, `none`. C++ runs as a systemd service; Python only bridges via DDS. |
| `maps(**cfg)` | OccupancyGrid + Voxel + ESDF + Elevation + TraversabilityCost + MapManager |
| `perception(detector, encoder, **cfg)` | Adds `CameraBridgeModule` if the driver doesn't expose color/depth natively |
| `memory(save_dir)` | SemanticMapper + Episodic + Tagged + Vector + Reconstruction |
| `planner(llm, save_dir)` | SemanticPlanner + LLMClient + VisualServo + AgentLoop |
| `navigation(planner, tomogram, enable_native)` | NavigationModule + autonomy chain (`cmu` / `nanobind` / `simple` / `nav_core` / `pid`) |
| `exploration(backend)` | `tare` (CMU TARE planner via NativeModule) or `none`. Wavefront frontier was removed from this stack on 2026-04-25 â€?it still exists as `WavefrontFrontierExplorer` invoked from `navigation()` when `enable_frontier=True`. |
| `safety()` | SafetyRingModule + CmdVelMux + GeofenceManagerModule |
| `gateway(port, mcp_port, enable_teleop, enable_rerun)` | Gateway + MCP + Teleop + optional WebRTC + optional Rerun |

`full_stack_blueprint` calls these factories conditionally (`enable_semantic`,
`enable_map_modules`, `enable_gateway`, `enable_teleop`, `enable_native`,
`enable_rerun`) and adds two optional in-line blueprints for
`DeviceManager` (loads `config/devices.yaml`) and the GNSS subsystem
(`GnssModule` + `GnssBridgeModule` + optional `NtripClientModule`, gated by
`gnss.enabled` in `config/robot_config.yaml`).

## Pluggable backends (`runtime.registry`)

| Category | Registered names |
|----------|------------------|
| `driver` | `thunder`, `stub`, `sim_mujoco`, `sim_ros2` |
| `slam` | `fastlio2`, `pointlio`, `localizer`, `bridge`, `none` |
| `detector` | `yoloe`, `yolo_world`, `bpu`, `grounding_dino` |
| `encoder` | `clip`, `mobileclip` |
| `llm` | `kimi`, `openai`, `claude`, `qwen`, `mock` |
| `planner_backend` | `astar` (pure Python), `pct` (C++ `ele_planner.so`) |
| `path_follower_backend` | `nav_core` (C++ nanobind), `pure_pursuit`, `pid` |

All backends register themselves with `@register("category", "name")`. No
blueprint code branches on the backend name.

## Profiles

Defined in `cli/profiles_data.py`. The shipping set is:

| Profile | Default robot | SLAM | LLM | Planner | Native | Semantic | Use |
|---------|---------------|------|-----|---------|--------|----------|-----|
| `stub` | stub | none | mock | astar | no | no | Framework smoke |
| `dev` | stub | none | mock | astar | no | yes | Semantic pipeline development |
| `sim` | sim_mujoco | bridge | mock | astar | yes | yes | MuJoCo full stack |
| `sim_nav` | stub | none | mock | astar | no | no | Pure-Python nav, ROS2-free |
| `map` | s100p (sim_ros2 driver) | fastlio2 | mock | astar | no | no | Build a map |
| `nav` | s100p | bridge | qwen | pct | no | yes | Navigate using a saved tomogram |
| `explore` | s100p | fastlio2 | qwen | pct | no | yes (frontier on) | Wavefront exploration |
| `tare_explore` | s100p | fastlio2 | qwen | pct | no | yes (TARE backend) | CMU TARE explorer |

`ROBOT_PRESETS` maps robot keys (`stub`, `sim`, `ros2`, `s100p`,
`navigate`, `thunder`) to driver / SLAM / detector / encoder defaults that
get merged in unless the profile already specifies them.

## Cross-stack wires (the contract)

`full_stack.py` defines explicit wires that wouldn't be unambiguous via
auto-wire. The set is reproduced in `AGENTS.md` section 12 and includes:

- `SafetyRingModule.stop_cmd â†?driver.stop_signal` and
  `â†?NavigationModule.stop_signal`
- `localization.odometry â†?NavigationModule.odometry` (priority over driver dead-reckoning)
- Autonomy chain: `NavigationModule.waypoint â†?LocalPlannerModule.waypoint`,
  `TerrainModule.terrain_map â†?LocalPlannerModule.terrain_map`,
  `LocalPlannerModule.local_path â†?PathFollowerModule.local_path` and
  `â†?SafetyRingModule.path`
- Visual servo: `SemanticPlannerModule.servo_target â†?VisualServoModule`,
  `VisualServoModule.goal_pose â†?NavigationModule.goal_pose`,
  `VisualServoModule.nav_stop â†?NavigationModule.stop_signal`
- `cmd_vel` mux: `Teleop / VisualServo / Navigation.recovery_cmd_vel /
  PathFollower â†?CmdVelMux â†?driver.cmd_vel`. `SafetyRingModule.cmd_vel`
  observes the mux output.
- Localization health: `SlamBridgeModule.localization_status â†?
  SafetyRing / Navigation / DepthVisualOdom`
- Visual odometry fusion: `DepthVisualOdomModule.visual_odometry â†?
  SlamBridgeModule.visual_odom`
- TraversabilityCost fan-in/out: `OccupancyGrid / ElevationMap / ESDF /
  Terrain â†?TraversabilityCostModule â†?Navigation.costmap +
  LocalPlanner.esdf + Gateway.{costmap, slope_grid}`

## Goal resolution flow

```
Instruction (user / Web / MCP / REPL)
   â†?
SemanticPlannerModule.on_instruction
   â†?optional TaskDecomposer
   â†?
GoalResolver._try_resolve(scene_graph_json, instruction)
   â”œâ”€ Tag lookup     (TaggedLocationStore exact / fuzzy)
   â”œâ”€ Fast Path      (keyword + CLIP fusion, ~0.17 ms, threshold 0.75)
   â”œâ”€ Vector memory  (CLIP embedding search in ChromaDB or numpy fallback)
   â”œâ”€ Frontier       (FrontierScorer information-gain on TopologySemGraph)
   â””â”€ Visual servo   (VLM bbox detection + PD tracking, near/far split)
   â†?
goal_pose â†?NavigationModule
   â†?
GlobalPlannerService.plan() â†?WaypointTracker â†?waypoint stream
   â†?
LocalPlannerModule â†?PathFollowerModule â†?CmdVelMux â†?driver
```

## Network ports

| Port | Service | Module |
|------|---------|--------|
| 5050 | REST + SSE + `/ws/teleop` | `GatewayModule` (also serves `TeleopModule`'s WS endpoint) |
| 8090 | MCP JSON-RPC | `MCPServerModule` |
| 9090 | Rerun web UI (when `--rerun`) | `RerunBridgeModule` |

## Storage

- `~/data/nova/maps/<name>/` (legacy) or `~/data/lingtu/maps/<name>/` (new
  default) â€?`map.pcd`, `tomogram.pickle`, `map.pgm` + `map.yaml`,
  `patches/`, optional `map.pcd.predufo` (DUFOMap pre-clean backup).
  Override with `NAV_MAP_DIR`.
- `.lingtu/run.{pid,json}` â€?daemon lifecycle (`cli/run_state.py`).
- `logs/<timestamp>_<profile>/lingtu.log` â€?per-run log
  (`cli/logging_util.py`).

## Sensor calibration

The `calibration/` toolbox covers camera intrinsics (OpenCV checkerboard),
IMU Allan variance, LiDARâ€“IMU extrinsics (HKU MARS LI-Init), and
camera-LiDAR target-less extrinsics (koide3). `calibration/apply_calibration.py`
writes results to `config/robot_config.yaml` and to the SLAM YAMLs.
`src/runtime/utils/calibration_check.py` runs `run_calibration_check` at
startup (see `full_stack_blueprint`); fatal errors block the build.

## Related docs

- [`AGENTS.md`](../AGENTS.md) â€?primary engineering reference
- [`MODULE_FIRST_GUIDELINE.md`](../MODULE_FIRST_GUIDELINE.md) â€?architectural rules
- [`PLANNER_SELECTION.md`](./PLANNER_SELECTION.md) â€?PCT vs A* decision
- [`gnss_integration.md`](./gnss_integration.md) â€?GNSS / RTK fusion
- [`TASK_ORCHESTRATION.md`](./TASK_ORCHESTRATION.md) â€?multi-agent task triggering
- [`TOPIC_CONTRACT.md`](./TOPIC_CONTRACT.md) â€?ROS2 topic contract used by C++ nodes
