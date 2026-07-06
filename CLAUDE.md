# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LingTu (灵�? is an autonomous navigation system for quadruped robots in outdoor/off-road environments.

- **Platform**: S100P (RDK X5, Nash BPU 128 TOPS, aarch64) | ROS2 Humble | Ubuntu 22.04
- **Languages**: Python (framework + semantic modules), C++ (SLAM/terrain/planner)
- **Architecture**: Module-First �?Module is the only runtime unit, Blueprint is the only orchestration
- **Guideline**: `docs/archive/MODULE_FIRST_GUIDELINE.md` �?8 rules for how code should be structured

## Quick Start

```bash
# Framework tests (no ROS2 needed, runs on any machine)
python -m pytest src/runtime/tests/ -q       # 1226 tests

# CLI with interactive REPL (profile-based, recommended)
python lingtu.py                          # interactive profile selector
python lingtu.py stub                     # no hardware, framework testing
python lingtu.py sim                      # MuJoCo simulation (full stack)
python lingtu.py dev                      # semantic pipeline, no C++ nodes
python lingtu.py nav                      # real S100P saved-map navigation (BPU + Qwen)
python lingtu.py explore                  # exploration, no pre-built map
python lingtu.py map                      # mapping mode (SLAM + save)
python lingtu.py --list                   # list all profiles

# Override any profile flag
python lingtu.py nav --llm mock           # real robot but mock LLM
python lingtu.py nav --daemon             # background daemon (S100P navigation)
python lingtu.py stop                     # stop running daemon

# Composable factory API (each line = one functional stack)
from runtime.blueprint import autoconnect
from runtime.blueprints.stacks import *
system = autoconnect(
    driver("thunder", dog_host="192.168.66.190"),  # L1 Robot + camera bridge
    slam("localizer"),                           # L1 SLAM / localization
    maps(),                                      # L2 OccupancyGrid + ESDF + ElevationMap
    perception("bpu"),                           # L3 Detector + Encoder + Reconstruction
    memory(),                                    # L3 SemanticMapper + Episodic + Tagged + VectorMemory
    planner("kimi"),                             # L4 SemanticPlanner + LLM + VisualServo
    navigation("astar"),                         # L5 Navigation + Autonomy chain
    safety(),                                    # L0 SafetyRing + Geofence
    gateway(5050),                               # L6 HTTP/WS/SSE + MCP + Teleop
).build()
system.start()
```

## Architecture —Module-First with Composable Stacks

Module is the only runtime unit. Blueprint composes Modules. Factory functions bundle related Modules into reusable stacks.

### Layer Hierarchy

```
L0  Safety    �?SafetyRingModule + GeofenceManagerModule + CmdVelMux
L1  Hardware  �?Driver + CameraBridge + SLAM (managed/bridge/localizer)
L2  Maps      �?OccupancyGrid + ESDF + ElevationMap + Terrain + LocalPlanner + PathFollower
L3  Perception �?Detector + Encoder + Reconstruction + SemanticMapper + Episodic + Tagged + VectorMemory
L4  Decision  �?SemanticPlanner + LLM + VisualServo (bbox tracking + person following)
L5  Planning  �?NavigationModule (A*/PCT + WaypointTracker + mission FSM + goal safety)
L6  Interface �?Gateway + MCP + Teleop
```

High layers �?low layers only. L5→L2 (waypoint→PathFollower) is command dispatch, not dependency.

### Composable Stack Factories (`src/runtime/blueprints/stacks/`)

| Factory | Returns | Modules |
|---------|---------|---------|
| `driver(robot)` | Blueprint | Driver + CameraBridge (auto-detect) |
| `slam(profile)` | Blueprint | SLAMModule or SlamBridgeModule |
| `maps()` | Blueprint | OccupancyGrid + ESDF + ElevationMap |
| `perception(det, enc)` | Blueprint | Detector + Encoder + Reconstruction |
| `memory(save_dir)` | Blueprint | SemanticMapper + Episodic + Tagged + VectorMemory |
| `planner(llm)` | Blueprint | SemanticPlanner + LLM + VisualServo |
| `navigation(planner)` | Blueprint | NavigationModule + Autonomy chain |
| `safety()` | Blueprint | SafetyRing + Geofence |
| `gateway(port)` | Blueprint | Gateway + MCP + Teleop |
| `exploration(backend)` | Blueprint | TARE exploration (wavefront removed 2026-04) |
| `lidar(ip, enabled)` | Blueprint | Livox MID-360 hardware driver (decoupled from SLAM) |
| `sim_lidar(scene_xml)` | Blueprint | Simulated PointCloud2 from MuJoCo scene geometry |

### Pluggable Backends (via Registry)

| Module | Backends |
|--------|----------|
| Driver | `thunder` (gRPC鈫抌rainstem), `stub` (testing), `sim_mujoco`, `sim_ros2` |
| SLAM | `fastlio2`, `pointlio`, `localizer` (ICP on pre-built map), `bridge` (external ROS2) |
| Detector | `yoloe`, `yolo_world`, `bpu` (Nash hardware), `grounding_dino` |
| Encoder | `clip` (ViT-B/32), `mobileclip` (edge) |
| LLM | `kimi`, `openai`, `claude`, `qwen`, `mock` |
| Planner | `astar` (pure Python), `pct` (C++ ele_planner.so) |
| PathFollower | `nav_kernel` (C++ nanobind), `pure_pursuit`, `pid` |

All backends registered via `@register("category", "name")` in `runtime.registry`. Zero if/else.

### Profiles

Full definitions in `cli/profiles_data.py` (14 named profiles + 7 robot presets = 21 total combinations).

| Profile | Robot Preset | SLAM | Semantic | Use Case |
|---------|-------------|------|----------|----------|
| `map` | s100p | fastlio2 | no | Build a new map of the environment |
| `nav` | s100p | bridge | yes | Navigate using a saved map (PCT) |
| `explore` | s100p | fastlio2 | yes | Wavefront frontier exploration |
| `tare_explore` | s100p | fastlio2 | yes | CMU TARE hierarchical exploration |
| `super_lio` | s100p | super_lio | yes | Evaluate Super-LIO external LIO backend |
| `super_lio_relocation` | s100p | super_lio_relocation | yes | Super-LIO relocation against saved map |
| `sim` | sim | bridge | yes | MuJoCo full simulation |
| `sim_mujoco_live` | sim_gazebo | none | no | MuJoCo raw MID-360 + Fast-LIO live sim |
| `sim_gazebo` | sim_gazebo | none | yes | Gazebo/GZ ROS-native simulation |
| `sim_industrial` | sim_gazebo | none | yes | Engineering industrial-yard simulation |
| `sim_cmu_tare` | sim_gazebo | none | no | CMU Unity + external TARE simulation |
| `sim_nav` | stub | none | no | Pure-Python nav sim (no ROS2/C++) |
| `dev` | stub | none | yes | Semantic pipeline dev |
| `stub` | stub | none | no | Framework testing |

### Backpressure Policies

```python
self.image.set_policy("latest")                   # drop if busy
self.imu.set_policy("throttle", interval=0.02)    # max 50Hz
self.lidar.set_policy("sample", n=5)              # every 5th
self.detections.set_policy("buffer", size=10)      # batch of 10
```

### Transport Decoupling (per-wire)

```python
bp.wire("Safety", "stop_cmd", "Driver", "stop_signal")                          # callback (0 latency)
bp.wire("Perception", "scene_graph", "Planner", "scene_graph", transport="dds")  # decoupled
bp.wire("SLAM", "cloud", "Terrain", "cloud", transport="shm")                   # high bandwidth
```

## Source Directory (`src/`)

| Directory | Role |
|-----------|------|
| `core/` | Framework: Module, Blueprint, Transport, Registry, stacks/, utils, msgs, tests (948) |
| `nav/` | NavigationModule, SafetyRing, CmdVelMux, GlobalPlannerService, WaypointTracker, OccupancyGrid, ESDF, ElevationMap |
| `semantic/` | perception/ (Detector+Encoder), planner/ (SemanticPlanner+LLM+VisualServo+AgentLoop), reconstruction/ (flattened �?one-level subdirs, no nested modules/) |
| `memory/` | SemanticMapper, EpisodicMemory, TaggedLocations, VectorMemory, RoomObjectKG, TopologySemGraph |
| `drivers/` | thunder/ (ThunderDriver + CameraBridge), sim/ (stub, MuJoCo, ROS2), TeleopModule |
| `gateway/` | GatewayModule (FastAPI HTTP/WS/SSE), MCPServerModule (MCP tools) |
| `nav/local/` + `nav/kernel/` | Terrain + LocalPlanner + PathFollower Modules (Python), backed by `nav_kernel` (nanobind C++) and `nav/services/plan/local_planner/cpp` (standalone C++); `base_autonomy/` no longer exists as a path, and the historical ROS2 shells under `nav/local/legacy_ros/` were retired (see `docs/architecture/ROS2_DECOUPLING_MIGRATION_PLAN.md`) |
| `slam/` | SLAMModule (Fast-LIO2/Point-LIO/Localizer), SlamBridgeModule, C++ SLAM nodes |
| `global_planning/` | pct_planner (C++ ele_planner.so) + _AStarBackend / _PCTBackend (via Registry) |
| `exploration/` | TARE exploration (ExplorationSupervisorModule, TareExplorerModule, compatibility shim) |
| `webrtc/` | WebRTC video streaming (WebRTCStreamModule) |

Note: `calibration/` and `sim/` live at repo root (not under `src/`). See [Sensor Calibration](#sensor-calibration-calibration) section. For `sim/`, see the `sim/engine/` documentation below.

## Key Files

| File | Purpose |
|------|---------|
| `docs/REPO_LAYOUT.md` | Top-level directory map (where `src/`, `scripts/`, `tools/`, —live) |
| `lingtu.py` | CLI entry point —profiles + REPL (`main_nav.py` kept as alias) |
| `src/runtime/blueprints/full_stack.py` | Top-level blueprint (~60 lines, calls 9 stack factories) |
| `src/runtime/blueprints/stacks/` | 9 composable factory functions |
| `src/runtime/module.py` | Module base class (In/Out, @skill, @rpc, layer tags) |
| `src/runtime/stream.py` | Out[T]/In[T] ports (5 backpressure policies, thread-safe) |
| `src/runtime/blueprint.py` | Blueprint builder (autoconnect, per-wire transport, auto_wire) |
| `src/runtime/registry.py` | Plugin registry (@register decorator) |
| `src/nav/mission/navigation.py` | Global planner + WaypointTracker + mission FSM |
| `src/nav/planning/global_planner.py` | A*/PCT backend + safe-goal search |
| `src/nav/mission/waypoint_tracker.py` | Arrival + stuck detection |
| `src/nav/safety/safety_ring.py` | Safety reflex + evaluator + dialogue |
| `src/nav/safety/velocity_mux.py` | Priority-based cmd_vel arbitration (L0) |
| `src/decision/semantic_planner_module.py` | 5-level fallback + multi-turn AgentLoop |
| `src/decision/goal_resolver.py` | Fast-Slow dual-process + KG hot-reload |
| `src/decision/visual_servo_module.py` | BBoxNavigator + PersonTracker (dual channel) |
| `src/decision/agent_loop.py` | Multi-turn LLM tool calling (7 tools) |
| `src/memory/modules/semantic_mapper_module.py` | SceneGraph →RoomObjectKG + TopologySemGraph |
| `src/memory/modules/vector_memory_module.py` | CLIP + ChromaDB vector search |
| `src/drivers/teleop_module.py` | WebSocket joystick + camera stream |
| `src/localization/slam_module.py` | SLAM managed mode (fastlio2/pointlio/localizer) |
| `src/localization/bridge.py` | ROS2 SLAM bridge mode (map鈫抩dom TF transform point) |
| `src/gateway/gateway_module.py` | FastAPI HTTP/WS/SSE + drift watchdog + save hooks |
| `src/nav/services/dynamic_filter.py` | DUFOMap wrapper (subprocess repack/run/backup) |
| `src/nav/services/map_manager_module.py` | Save pipeline: PGO →DUFOMap →tomogram →occupancy |
| `scripts/lingtu` | **Unified Operations CLI** (status/watch/map/nav/svc/log/health) |
| `scripts/build/build_dufomap.sh` | Idempotent aarch64 build of DUFOMap (apt + patch + cmake) |
| `scripts/diagnostics/dufomap_offline_test.py` | Standalone validator: run DUFOMap on existing map, print stats |
| `src/nav/services/frame_contract.py` | Central frame-id definitions + integrity checks for all coordinate frames |
| `src/nav/ros2_waypoint_bridge_module.py` | ROS2 WaypointBridge: accepts /nav/goal goals from external ROS2 nodes |
| `src/gateway/templates/map_viewer.html` | Embedded client-side map viewer served by GatewayModule |
| `config/robot_config.yaml` | Robot physical parameters (single source of truth) |
| `config/dufomap.toml` | Lingtu-tuned DUFOMap config (Livox Mid-360 thresholds) |
| `docs/05-specialized/dynamic_obstacle_removal.md` | DUFOMap Phase 2 design + roadmap |
| `sim/engine/README.md` | Simulation platform core —SimEngine, MuJoCo, bridge, scenarios, worlds |

## Build and Test Commands

```bash
# Framework tests (primary, no ROS2 needed)
python -m pytest src/runtime/tests/ -q                    # 1226 tests, ~5s

# Fast tests (no ROS2, no sim, no slow �?skip heavy markers)
python -m pytest src/runtime/tests/ src/nav/tests/ src/gateway/tests/ src/memory/tests/ -q -m "not slow and not ros2 and not sim"

# Simulation tests (MuJoCo/Gazebo contract validation, no robot needed)
python -m pytest sim/tests/ -q                         # 509 scenario-level tests

# C++ nav_kernel tests (standalone, no ROS2)
cd src/nav/kernel && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
./test_benchmark                                       # 12 benchmarks
./test_path_follower_core                              # 15 tests

# C++ local planner tests (standalone, no ROS2)
cd ../../services/plan/local_planner/cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release -DLOCAL_PLANNER_CPP_BUILD_TESTS=ON
cmake --build build -j$(nproc)
./build/test_local_planner_core                        # 30 tests

# ROS2 build (for C++ nodes on S100P only)
source /opt/ros/humble/setup.bash
make build                                              # colcon release build
```

## C++ Performance (nav_kernel)

`src/nav/kernel/` is a header-only C++ algorithm library, exposed to Python via nanobind. Performance-critical for aarch64 deployment.

### Acceleration Libraries

| Library | Version | Purpose |
|---------|---------|---------|
| taskflow | 3.8.0 | Benchmark task parallelism support |
| OpenMP | �?| Parallel for (terrain + scoring) |

### Key Optimizations

| Optimization | File | Effect |
|------|------|------|
| SoA memory layout | local_planner.hpp | Cache-friendly point/path storage |
| CSR sparse format | local_planner.hpp | Cache-contiguous, eliminates pointer chasing |
| scorePathFast LUT | local_planner_scoring.hpp | pow025 table replaces sqrt(sqrt) |
| OpenMP parallel scoring | local_planner.hpp | 36 rotation directions in parallel |
| terrain parallelization | terrain_runtime.hpp | 2601 voxel nth_element parallel |
| LTO | CMakeLists.txt | Cross-function inlining |

CMakeLists.txt supports both ROS2 ament_cmake and standalone paths. OpenMP is
optional; relaxed floating-point math is not enabled globally.

## Critical Files —Do Not Break

- `src/runtime/module.py` —Module base class (all modules depend on it)
- `src/runtime/blueprint.py` —Blueprint + autoconnect (system assembly)
- `src/runtime/stream.py` —In[T]/Out[T] ports (data flow backbone)
- `src/runtime/registry.py` —Plugin registry (all backends depend on it)
- `src/runtime/utils/` —Cross-layer utilities (18+ files import from here)
- `src/perception/.../instance_tracker.py` —Scene graph builder
- `src/semantic/planner/.../goal_resolver.py` �?-level resolution chain
- `config/robot_config.yaml` —Robot physical parameters

## Module Dependency Rules

```
All Modules ──�?core/ (Module, In/Out, Registry, utils, msgs)
                 �?only legal dependency direction

nav/          does NOT import semantic/, drivers/, gateway/
semantic/     does NOT import nav/, drivers/, gateway/
drivers/      does NOT import nav/, semantic/ (lazy import in blueprints only)
gateway/      does NOT import nav/, semantic/, drivers/
```

Planner backends resolved via `runtime.registry.get("planner_backend", name)`, not direct import.

## Semantic Navigation

### 5-Level Goal Resolution Chain

```
Instruction: "去上次放背包的地�?
  �?1. Tag Lookup     —exact/fuzzy match in TaggedLocationStore     →goal_pose
2. Fast Path      —scene graph keyword + CLIP matching (<200ms) →goal_pose
3. Vector Memory  —CLIP embedding search in ChromaDB            →goal_pose
4. Frontier       —topology graph information gain exploration   →goal_pose
5. Visual Servo   —VLM bbox detection + PD tracking             →goal_pose/cmd_vel
```

### Fast-Slow Dual-Process (`goal_resolver.py`)

**Fast Path** (System 1, ~0.17ms): Direct scene graph matching —keyword + spatial reasoning, confidence fusion (label 35%, CLIP 35%, detector 15%, spatial 15%). Target: >70% hit rate, threshold 0.75.

**Slow Path** (System 2, ~2s): LLM reasoning with ESCA selective grounding —filters 200 objects to ~15 objects (92.5% token reduction), then calls LLM. Returns OmniNav hierarchical room hint.

**AdaNav Entropy Trigger**: Shannon entropy over candidate scores. If `score_entropy > 1.5` and `confidence < 0.85`, forced escalation to Slow Path.

**LERa Failure Recovery**: 3-step Look-Explain-Replan on subgoal failure. After 2nd consecutive failure: LLM decides `retry_different_path | expand_search | requery_goal | abort`.

### Visual Servo (`visual_servo_module.py`)

Two output channels based on distance:
- Far (> 3m): `goal_pose →NavigationModule →planning stack`
- Near (< 3m): `cmd_vel →CmdVelMux →Driver (PD servo, bypasses planner)`

Components: BBoxNavigator (bbox+depth→D鈫扨D), PersonTracker (VLM select+CLIP Re-ID), vlm_bbox_query (open-vocab detection).

### Multi-Turn Agent Loop (`agent_loop.py`)

`agent_instruction` port triggers observe鈫抰hink鈫抋ct cycle with 7 LLM tools:
`navigate_to`, `navigate_to_object`, `detect_object`, `query_memory`, `tag_location`, `say`, `done`.
Max 10 steps / 120s timeout. Supports OpenAI function-calling + text JSON fallback.

### LLM Configuration

```bash
export MOONSHOT_API_KEY="..."         # Kimi (default, China-direct)
export OPENAI_API_KEY="sk-..."        # OpenAI
export ANTHROPIC_API_KEY="sk-ant-..." # Claude
export DASHSCOPE_API_KEY="sk-..."     # Qwen (China fallback)
```

## SLAM / Localization

| Mode | slam_profile | Backend | Use Case |
|------|-------------|---------|----------|
| Mapping | `fastlio2` | SLAMModule →C++ Fast-LIO2 | First visit, build map |
| Localization | `localizer` | SLAMModule →Fast-LIO2 + ICP Localizer | Navigate with pre-built map |
| Bridge | `bridge` | SlamBridgeModule →ROS2 subscriber | External SLAM (systemd) |
| None | `none` | —| stub/dev mode |

Localizer requires Fast-LIO2 companion (provides `/cloud_registered` + `/Odometry`).
SLAM odometry is explicitly wired to NavigationModule (priority over driver dead-reckoning).

## REPL Commands

```
Navigation:  go/navigate <target> | stop | cancel | status
Map:         map list | save <name> | use <name> | build <name> | delete <name>
Semantic:    smap status | rooms | save | load <dir> | query <text>
Vector:      vmem query <text> | vmem stats
Agent:       agent <multi-step instruction>
Teleop:      teleop status | teleop release
Monitor:     health | watch <port> | module <name> | connections | log | config
```

## MCP Server (AI Agent Control)

Auto-discovered @skill methods exposed via JSON-RPC at `http://<robot>:8090/mcp`.

| Category | Tools |
|----------|-------|
| Navigation | `navigate_to`, `navigate_to_object`, `stop`, `get_navigation_status`, `set_mode` |
| Perception | `get_scene_graph`, `detect_objects`, `get_robot_position` |
| Memory | `query_memory`, `query_location` (vector), `list_tagged_locations`, `tag_location` |
| Semantic Map | `get_room_summary`, `query_room_for_object`, `get_exploration_target` |
| Visual Servo | `find_object`, `follow_person`, `stop_servo`, `get_servo_status` |
| Planning | `send_instruction`, `decompose_task` |
| System | `get_health`, `list_modules`, `get_config` |

```bash
# Connect Claude Code to the robot
claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp
```

## Teleop (Remote Control)

WebSocket joystick at `ws://<robot>:5050/ws/teleop`:
- Phone/browser sends `{"type": "joy", "lx": 0.5, "ly": 0.0, "az": -0.3}`
- GatewayModule forwards raw WS joy messages to TeleopModule via `joy_input` port
- TeleopModule manages all teleop state (active/idle/release) and joy scaling
- TeleopModule publishes `teleop_active: Out[bool]` for NavigationModule pause/resume
- cmd_vel goes through CmdVelMux (priority-based arbitration), not directly to driver
- 3s idle auto-release runs in TeleopModule's background thread
- Robot streams JPEG camera frames back

## cmd_vel Priority Arbitration (CmdVelMux)

All cmd_vel sources are routed through CmdVelMux (L0) for priority-based arbitration:

| Source | Priority | Timeout |
|--------|----------|---------|
| Teleop (joystick) | 100 | 0.5s |
| VisualServo (PD tracking) | 80 | 0.5s |
| Recovery (stuck backup) | 60 | 0.5s |
| PathFollower (autonomy) | 40 | 0.5s |

Highest-priority active source wins. A source is "active" if it published within 0.5s.
When a source times out, the mux falls through to the next lower-priority source.

## Explicit Wires (Cross-Stack, in `full_stack.py`)

```python
# Safety →all actuators
bp.wire("SafetyRingModule", "stop_cmd", driver_name, "stop_signal")
bp.wire("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")

# SLAM odometry priority
bp.wire(slam_module_name, "odometry", "NavigationModule", "odometry")

# Autonomy chain (when enable_native=True)
bp.wire("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint")
bp.wire("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map")
bp.wire("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")

# Visual servo dual channel
bp.wire("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target")
bp.wire("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose")

# CmdVelMux —priority-based velocity arbitration
bp.wire("TeleopModule",      "cmd_vel",          "CmdVelMux", "teleop_cmd_vel")
bp.wire("VisualServoModule", "cmd_vel",          "CmdVelMux", "visual_servo_cmd_vel")
bp.wire("NavigationModule",  "recovery_cmd_vel", "CmdVelMux", "recovery_cmd_vel")
bp.wire("PathFollowerModule", "cmd_vel",         "CmdVelMux", "path_follower_cmd_vel")
bp.wire("CmdVelMux", "driver_cmd_vel", driver_name, "cmd_vel")

# Teleop active →Navigation pause/resume
bp.wire("TeleopModule", "teleop_active", "NavigationModule", "teleop_active")
```

## S100P Deployment

- **SSH**: `ssh sunrise@192.168.66.190`
- **Nav code**: `~/data/SLAM/navigation/` (symlink →`~/data/inovxio/lingtu/`)
- **Nav deploy**: `/opt/lingtu/nav/`
- **CycloneDDS**: Built from source at `~/cyclonedds/install/` (Unitree approach)
- **Python**: 3.10.12, cyclonedds==0.10.5
- **DUFOMap binary**: `~/src/dufomap/build/dufomap_run` (see `scripts/build/build_dufomap.sh`)
- **Default map dir**: `~/data/nova/maps/` (was `~/data/inovxio/data/maps/` —migrated)

## Operations CLI (`scripts/lingtu`)

单一入口 CLI 取代多个零散脚本�?curl / systemctl。建议本�?`alias`:

```bash
alias lingtu='ssh sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu"'
alias lingwatch='ssh -t sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu watch"'
```

| 子命�?| 用�?|
|---|---|
| `lingtu status` | 一�?8 区状�?(session / SLAM / robot / mission / path / ctrl / map / log) |
| `lingtu watch` | `watch -c -n 1` 持续刷新 �?建图/导航时副屏开这个 |
| `lingtu map start\|save <name>\|end\|list` | 建图 session 生命周期 |
| `lingtu nav start <map>\|stop\|goal X Y [YAW]` | 导航 session + 发目�?|
| `lingtu svc status\|restart [slam\|lingtu\|all]` | systemctl wrapper |
| `lingtu log drift\|dufomap\|error\|tail\|all` | journalctl 过滤�?|
| `lingtu health` | REST `/api/v1/health` 原样 dump |

## Dynamic Obstacle Removal (Phase 1 + 2)

建图过程 + 保存时双重过�?消除�?物走过留下的拖尾�?
- **Phase 1** `voxel hit-count voting` (`gateway_module.py:_on_map_cloud` mapping 分支)
  - 每帧 map_cloud �? 每个 voxel hit_count +1
  - �?SSE 前过�?`hit < LINGTU_MAP_MIN_HITS` (默认 3) �?voxel
  - **只影�?Web 实时视图**
- **Phase 2** `DUFOMap (ray-casting + void detection)`
  - 保存地图时在 PGO 之后 tomogram 之前跑一�?offline filter
  - �?`<map>/patches/*.pcd` + `poses.txt`, 写回干净 `map.pcd`, 备份�?`map.pcd.predufo`
  - **影响磁盘 PCD** �?导航时加载就是干净底图
  - 跑的�?C++ binary `~/src/dufomap/build/dufomap_run` + `config/dufomap.toml` (Lingtu 调参)
  - env `LINGTU_SAVE_DYNAMIC_FILTER=0` 关闭

详见 `docs/05-specialized/dynamic_obstacle_removal.md`�?
## SLAM Drift Watchdog

Fast-LIO2 IEKF 长时间静置会协方差发�?xy 飘到 10^12 米。Gateway 后台线程
(`_drift_watchdog_loop`) �?60s 检�?odom,超阈值自�?

1. `svc.stop("slam","slam_pgo","localizer")` �?终结飞掉�?IEKF
2. �?`self._odom` 缓存
3. SSE �?`slam_drift` 事件
4. 按当�?session mode `svc.ensure(...)` 重拉服务
5. 300s 冷却防抖

Env: `LINGTU_DRIFT_WATCHDOG=0` (�? / `_INTERVAL` / `_XY_LIMIT` / `_V_LIMIT` / `_COOLDOWN`�?
## Sensor Calibration (`calibration/`)

出厂标定工具箱，覆盖 S100P 全部传感器。标定结果统一写入 `config/robot_config.yaml`�?
### 标定流程 (SOP)

| Step | 内容 | 工具 | 时间 |
|------|------|------|------|
| 1 | 相机内参 (棋盘�?9×6) | `calibration/camera/calibrate_intrinsic.py` (OpenCV) | ~5 min |
| 2 | IMU 噪声 (Allan Variance) | `calibration/imu/allan_variance_ros2/` (Autoliv) | ~2-3 hr |
| 3 | LiDAR-IMU 外参 (8 字运�? | `calibration/lidar_imu/LiDAR_IMU_Init/` (HKU-MARS) | ~2 min |
| 4 | 相机-LiDAR 外参 (target-less) | `calibration/camera_lidar/direct_visual_lidar_calibration/` (koide3) | ~10 min |
| 5 | 一键应�?| `calibration/apply_calibration.py` �?robot_config.yaml + SLAM configs | 秒级 |
| 6 | 一键验�?| `calibration/verify.py` (焦距/畸变/旋转/投影�?sanity check) | 秒级 |

### 标定参数输出

```yaml
# config/robot_config.yaml
camera:
  fx, fy, cx, cy              # Step 1 相机内参
  dist_k1..k3, dist_p1..p2    # Step 1 畸变系数
  position_x/y/z              # Step 4 相机-LiDAR 外参
  roll, pitch, yaw            # Step 4 旋转

lidar:
  offset_x/y/z                # Step 3 LiDAR-IMU 外参 (t_il)
  roll, pitch, yaw            # Step 3 旋转 (r_il)
```

`apply_calibration.py` 同时同步�?`src/localization/fastlio2/config/lio.yaml` �?`config/pointlio.yaml` (na, ng, nba, nbg, r_il, t_il)�?
### 运行时校�?
`src/runtime/utils/calibration_check.py` �?`full_stack_blueprint()` 启动时校验标定参数：
- FAIL �?(如焦距为 0、旋转矩阵非正交) �?阻止启动
- WARN �?(如畸变系数全�? �?日志警告，不阻断

### 关键文件

| File | Purpose |
|------|---------|
| `calibration/README.md` | 完整 SOP 文档 (含命令行示例) |
| `calibration/apply_calibration.py` | �?4 类标定结果写�?robot_config + SLAM 配置 |
| `calibration/verify.py` | 一键验�? 参数范围 + 投影�?+ 跨配置一致�?|
| `calibration/camera/calibrate_intrinsic.py` | 相机内参 (capture/calibrate/verify 三合一) |
| `calibration/lidar_imu/ros2_adapter/` | ROS2→ROS1 bridge 适配�?(rosbag 回放) |
| `src/runtime/utils/calibration_check.py` | 运行时标定参数校�?(启动时调�? |
| `config/robot_config.yaml` | 标定参数最终归�?(single source of truth) |

## Code Style

- **C++**: Google style (`.clang-format`, 2-space indent, 100 col)
- **Python**: English comments in new code. Chinese comments exist in legacy code.
- **Framework**: All Modules use `runtime.Module` base with In[T]/Out[T] type hints
- **No ROS2 in Modules**: rclpy only in explicit compat/bridge adapters; NativeModule is legacy ROS2 compatibility only

## Known Limitations

- Fast Path uses rule-based matching (not learned policies)
- S100P has no CUDA �?Open3D GPU features unavailable; use CPU nav_kernel/nanobind terrain and local-planning kernels
- Kimi API key may expire —Slow Path unavailable without valid LLM key
- ChromaDB optional —VectorMemoryModule falls back to numpy brute-force search
- Framework tests (1226) are mock-based —real hardware integration tests need S100P
- C++ test_validation 6 tests fail under `-ffast-math` (NaN/Inf IEEE compliance) —expected
