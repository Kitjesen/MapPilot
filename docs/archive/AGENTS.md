# AGENTS.md �?LingTu Agent Guide

This file provides guidance to AI coding agents working with the LingTu codebase.

## 1. Project Overview

LingTu (閻忕敻鈧? is an autonomous navigation system for quadruped robots in outdoor/off-road environments.

- **Platform**: S100P (RDK X5, Nash BPU 128 TOPS, aarch64) | ROS2 Humble | Ubuntu 22.04
- **Languages**: Python (framework + semantic modules), C++ (SLAM/terrain/planner)
- **Architecture**: Module-First �?Module is the only runtime unit, Blueprint is the only orchestration
- **Guideline**: `docs/MODULE_FIRST_GUIDELINE.md` �?8 rules for how code should be structured

## 2. Quick Start

```bash
# Framework tests (no ROS2 needed, runs on any machine)
python -m pytest src/runtime/tests/ -q       # ~1000+ framework tests

# CLI with interactive REPL (profile-based)
python lingtu.py                          # interactive profile selector
python lingtu.py stub                     # no hardware, framework testing
python lingtu.py sim                      # MuJoCo simulation (full stack)
python lingtu.py sim_nav                  # pure-Python sim, no ROS2 / C++
python lingtu.py dev                      # semantic pipeline, no C++ nodes
python lingtu.py nav                      # real S100P navigation (BPU + Qwen + PCT)
python lingtu.py explore                  # wavefront frontier exploration
python lingtu.py tare_explore             # CMU TARE hierarchical exploration
python lingtu.py map                      # mapping mode (SLAM + save)
python lingtu.py --list                   # list all profiles

# Override any profile flag
python lingtu.py nav --llm mock           # real robot but mock LLM
python lingtu.py nav --daemon             # background daemon
python lingtu.py stop                     # stop running daemon

# Composable factory API (each line = one functional stack)
from runtime.blueprint import autoconnect
from runtime.blueprints.stacks import *
system = autoconnect(
    driver("thunder", host="192.168.66.190"),
    slam("localizer"),
    maps(),
    perception("bpu"),
    memory(),
    planner("kimi"),
    navigation("astar"),
    safety(),
    gateway(5050),
).build()
system.start()
```

## 3. Architecture �?Dual-Layer System

LingTu is a **hybrid repo** with two layers that run together:

1. **Python Module-First Layer** �?High-level orchestration, semantic intelligence, and the entire framework (`core/`, `nav/`, `semantic/`, `memory/`, `gateway/`, `drivers/`). Started via `python lingtu.py`.
2. **C++/ROS2 Layer** �?Low-level SLAM, terrain analysis, local planning, and path following (`src/localization/`, `src/base_autonomy/`, `src/nav/services/plan/`). Managed as subprocesses by Python `NativeModule` or run via `ros2 launch`.

### Layer Hierarchy

```
L0  Safety       �?SafetyRingModule + GeofenceManagerModule
L1  Hardware     �?Driver + CameraBridge + SLAM (managed/bridge/localizer)
L2  Maps         �?OccupancyGrid + ESDF + ElevationMap + Terrain + LocalPlanner + PathFollower
L3  Perception   �?Detector + Encoder + Reconstruction + SemanticMapper + Episodic + Tagged + VectorMemory
L4  Decision     �?SemanticPlanner + LLM + VisualServo (bbox tracking + person following)
L5  Planning     �?NavigationModule (A*/PCT + WaypointTracker + mission FSM + goal safety)
L6  Interface    �?Gateway + MCP + Teleop
```

High layers depend on low layers only. L5閳�? (waypoint閳墾athFollower) is command dispatch, not dependency.

### Module-First Principles

1. **Module is the only runtime unit** �?no separate ROS2 Node + Module pairs
2. **Blueprint is the only orchestration** �?no `ros2 launch` for Python modules
3. **Communication via In/Out + Transport** �?not rclpy pub/sub
4. **C++ nodes managed by NativeModule** �?started as watchdog-supervised subprocesses
5. **Pluggable backends via Registry** �?`@register("category", "name")`, zero if/else
6. **Message types from `runtime.msgs`** �?not `sensor_msgs.msg` or `nav_msgs.msg`
7. **Configuration via constructor parameters** �?not ROS2 `declare_parameter`

### Composable Stack Factories (`src/runtime/blueprints/stacks/`)


| Factory                | Returns   | Modules                                           |
| ---------------------- | --------- | ------------------------------------------------- |
| `driver(robot)`        | Blueprint | Driver + CameraBridge (auto-detect)               |
| `slam(profile)`        | Blueprint | SLAMModule or SlamBridgeModule                    |
| `maps()`               | Blueprint | OccupancyGrid + ESDF + ElevationMap               |
| `perception(det, enc)` | Blueprint | Detector + Encoder + Reconstruction               |
| `memory(save_dir)`     | Blueprint | SemanticMapper + Episodic + Tagged + VectorMemory |
| `planner(llm)`         | Blueprint | SemanticPlanner + LLM + VisualServo               |
| `navigation(planner)`  | Blueprint | NavigationModule + Autonomy chain                 |
| `safety()`             | Blueprint | SafetyRing + Geofence                             |
| `gateway(port)`        | Blueprint | Gateway + MCP + Teleop                            |


### Pluggable Backends (via Registry)


| Module       | Backends                                                                             |
| ------------ | ------------------------------------------------------------------------------------ |
| Driver       | `thunder` (gRPC閳妼rainstem), `stub` (testing), `sim_mujoco`, `sim_ros2`               |
| SLAM         | `fastlio2`, `pointlio`, `localizer` (ICP on pre-built map), `bridge` (external ROS2) |
| Detector     | `yoloe`, `yolo_world`, `bpu` (Nash hardware), `grounding_dino`                       |
| Encoder      | `clip` (ViT-B/32), `mobileclip` (edge)                                               |
| LLM          | `kimi`, `openai`, `claude`, `qwen`, `mock`                                           |
| Planner      | `astar` (pure Python), `pct` (C++ ele_planner.so)                                    |
| PathFollower | `nav_core` (C++ nanobind), `pure_pursuit`, `pid`                                     |


### Profiles (Complete �?see `cli/profiles_data.py`)


| Profile          | Driver     | SLAM     | LLM  | Planner | Native | Semantic | Gateway | Use Case                       |
| ---------------- | ---------- | -------- | ---- | ------- | ------ | -------- | ------- | ------------------------------ |
| `stub`           | stub       | none     | mock | astar   | no     | no       | yes     | Framework testing              |
| `dev`            | stub       | none     | mock | astar   | no     | yes      | yes     | Semantic pipeline dev          |
| `sim`            | sim_mujoco | bridge   | mock | astar   | yes    | yes      | yes     | MuJoCo full simulation         |
| `sim_nav`        | stub       | none     | mock | astar   | no     | no       | yes     | Pure-Python sim, no ROS2 / C++ |
| `map`            | s100p      | fastlio2 | mock | astar   | no     | no       | yes     | Build map with SLAM            |
| `nav`            | s100p      | bridge   | qwen | pct     | no     | yes      | yes     | Navigate with pre-built map    |
| `explore`        | s100p      | fastlio2 | qwen | pct     | no     | yes      | yes     | Wavefront exploration          |
| `tare_explore`   | s100p      | fastlio2 | qwen | pct     | no     | yes      | yes     | CMU TARE explorer              |


### Robot Presets

Robot presets provide hardware-specific defaults that get merged into profiles via `--robot`:


| Preset    | `robot`    | `slam_profile` | `detector` | `encoder`  | Extra                                       |
| --------- | ---------- | -------------- | ---------- | ---------- | ------------------------------------------- |
| `stub`    | stub       | none           | yoloe      | mobileclip | �?                                          |
| `sim`     | sim_mujoco | bridge         | yoloe      | mobileclip | �?                                          |
| `ros2`    | sim_ros2   | bridge         | yoloe      | mobileclip | �?                                          |
| `s100p`   | sim_ros2   | localizer      | bpu        | mobileclip | �?                                          |
| `thunder` | thunder    | localizer      | bpu        | mobileclip | `dog_host=192.168.66.190`, `dog_port=13145` |


## 4. Repository Layout

```
lingtu/
閳规壕鏀㈤埞�?lingtu.py              # Primary entry (Module-First CLI + REPL)
閳规壕鏀㈤埞�?main_nav.py            # Alias �?same as lingtu.py
閳规壕鏀㈤埞�?lingtu_cli.py          # pip console script `lingtu`
閳规壕鏀㈤埞�?cli/                   # CLI implementation (profiles, REPL, daemon)
閳规壕鏀㈤埞�?src/                   # Python packages + ROS2 packages (colcon)
�?  閳规壕鏀㈤埞�?core/              # Framework: Module, Blueprint, Transport, Registry, stacks, tests
�?  閳规壕鏀㈤埞�?nav/               # NavigationModule, SafetyRing, GlobalPlanner, OccupancyGrid, ESDF
�?  閳规壕鏀㈤埞�?semantic/          # perception/ (Detector+Encoder), planner/ (GoalResolver+LLM+VisualServo)
�?  閳规壕鏀㈤埞�?memory/            # SemanticMapper, EpisodicMemory, Tagged, VectorMemory, KG
�?  閳规壕鏀㈤埞�?drivers/           # thunder/ (gRPC), sim/ (stub, MuJoCo, ROS2), TeleopModule
�?  閳规壕鏀㈤埞�?gateway/           # GatewayModule (FastAPI), MCPServerModule
�?  閳规壕鏀㈤埞�?base_autonomy/     # C++ terrain + local planner + path follower (nanobind)
�?  閳规壕鏀㈤埞�?nav/services/plan/   # C++ PCT planner + Python adapters
�?  閳规壕鏀㈤埞�?slam/              # C++ SLAM (Fast-LIO2, Point-LIO, PGO, Localizer)
�?  閳规柡鏀㈤埞�?reconstruction/    # 3D reconstruction
閳规壕鏀㈤埞�?config/                # YAML / DDS / robot params
閳规壕鏀㈤埞�?launch/                # ROS2 launch (legacy / bridge stacks)
閳规壕鏀㈤埞�?sim/                   # MuJoCo simulation assets + scripts
閳规壕鏀㈤埞�?tests/                 # Integration & planning tests
閳规壕鏀㈤埞�?tools/                 # Robot-side helpers (dashboard, BPU export)
閳规壕鏀㈤埞�?scripts/               # ws/, deploy/, ota/, proto/ helpers
閳规壕鏀㈤埞�?docs/                  # Architecture, guides, ADRs
閳规壕鏀㈤埞�?Makefile               # colcon build, test, docker targets
閳规壕鏀㈤埞�?pyproject.toml         # Root package: lingtu (setuptools)
閳规壕鏀㈤埞�?requirements.txt       # S100P runtime deps
閳规柡鏀㈤埞�?docker-compose.yml     # Container orchestration
```

## 5. Source Directory Detail (`src/`)


| Directory          | Role                                                                                                             |
| ------------------ | ---------------------------------------------------------------------------------------------------------------- |
| `core/`            | Framework: Module, Blueprint, Transport, NativeModule, Registry, stacks/, utils, msgs, tests (640)               |
| `core/msgs/`       | Message types: geometry, nav, sensor, semantic (replaces ROS2 msg types)                                         |
| `core/transport/`  | Backends: Local (callback), DDS (cyclonedds), SHM, Adapter                                                       |
| `runtime/blueprints/` | System blueprints: `full_stack.py` calls 9 stack factories                                                       |
| `core/spec/`       | Protocol interfaces (16 specs)                                                                                   |
| `core/tests/`      | 640 framework tests (pytest, no ROS2 dependency)                                                                 |
| `nav/`             | NavigationModule, SafetyRing, GlobalPlannerService, WaypointTracker, OccupancyGrid, ESDF, ElevationMap           |
| `semantic/`        | perception/ (Detector+Encoder+SceneGraph), planner/ (SemanticPlanner+LLM+VisualServo+AgentLoop), reconstruction/ |
| `memory/`          | SemanticMapper, EpisodicMemory, TaggedLocations, VectorMemory, RoomObjectKG, TopologySemGraph                    |
| `drivers/`         | thunder/ (ThunderDriver + CameraBridge), sim/ (stub, MuJoCo, ROS2), TeleopModule                                 |
| `gateway/`         | GatewayModule (FastAPI HTTP/WS/SSE), MCPServerModule (MCP tools)                                                 |
| `base_autonomy/`   | TerrainModule + LocalPlannerModule + PathFollowerModule (C++ nanobind backends)                                  |
| `slam/`            | SLAMModule (Fast-LIO2/Point-LIO/Localizer), SlamBridgeModule, C++ SLAM nodes                                     |
| `nav/services/plan/` | pct_planner (C++ ele_planner.so) + _AStarBackend / _PCTBackend (via Registry)                                    |


## 6. Key Files


| File                                                  | Purpose                                                                 |
| ----------------------------------------------------- | ----------------------------------------------------------------------- |
| `lingtu.py`                                           | CLI entry point �?profiles + REPL                                       |
| `src/runtime/blueprints/full_stack.py`                   | Top-level blueprint (~190 lines, 9 stack factories + cross-stack wires) |
| `src/runtime/blueprints/stacks/`                         | 9 composable factory functions                                          |
| `src/runtime/module.py`                                  | Module base class (In/Out, @skill, @rpc, layer tags)                    |
| `src/runtime/stream.py`                                  | Out[T]/In[T] ports (5 backpressure policies, thread-safe)               |
| `src/runtime/blueprint.py`                               | Blueprint builder (autoconnect, per-wire transport, auto_wire)          |
| `src/runtime/registry.py`                                | Plugin registry (@register decorator)                                   |
| `src/nav/mission/navigation_module.py`                        | Global planner + WaypointTracker + mission FSM                          |
| `src/nav/services/plan/global_planner/service.py`                   | A*/PCT backend + _find_safe_goal BFS                                    |
| `src/nav/services/safety/safety_ring_module.py`                       | Safety reflex + evaluator + dialogue                                    |
| `src/decision/.../semantic_planner_module.py` | 5-level fallback + multi-turn AgentLoop                                 |
| `src/decision/.../goal_resolver.py`           | Fast-Slow dual-process + KG hot-reload                                  |
| `src/decision/.../visual_servo_module.py`     | BBoxNavigator + PersonTracker (dual channel)                            |
| `src/decision/.../agent_loop.py`              | Multi-turn LLM tool calling (7 tools)                                   |
| `src/memory/modules/semantic_mapper_module.py`        | SceneGraph �?RoomObjectKG + TopologySemGraph                            |
| `src/memory/modules/vector_memory_module.py`          | CLIP + ChromaDB vector search                                           |
| `src/drivers/teleop_module.py`                        | WebSocket joystick + camera stream                                      |
| `src/localization/slam_module.py`                             | SLAM managed mode (fastlio2/pointlio/localizer)                         |
| `config/robot_config.yaml`                            | Robot physical parameters (single source of truth)                      |


## 7. Build and Test Commands

```bash
# Framework tests (primary, no ROS2 needed)
python -m pytest src/runtime/tests/ -q                    # ~1000+ tests

# Native nav_core only (no ROS2 needed) �?used by Python autonomy chain
make nav_core                                          # builds _nav_runtime.so

# ROS2 build (for C++ nodes on S100P only)
source /opt/ros/humble/setup.bash
make build                                              # colcon release build
```

## 8. Critical Files �?Do Not Break

- `src/runtime/module.py` �?Module base class (all modules depend on it)
- `src/runtime/blueprint.py` �?Blueprint + autoconnect (system assembly)
- `src/runtime/stream.py` �?In[T]/Out[T] ports (data flow backbone)
- `src/runtime/registry.py` �?Plugin registry (all backends depend on it)
- `src/runtime/utils/` �?Cross-layer utilities (18+ files import from here)
- `src/perception/.../instance_tracker.py` �?Scene graph builder
- `src/decision/.../goal_resolver.py` �?5-level resolution chain
- `config/robot_config.yaml` �?Robot physical parameters

## 9. Module Dependency Rules

```
All Modules 閳光偓閳光偓�?core/ (Module, In/Out, Registry, utils, msgs)
                 �?only legal dependency direction

nav/          does NOT import semantic/, drivers/, gateway/
semantic/     does NOT import nav/, drivers/, gateway/
drivers/      does NOT import nav/, semantic/ (lazy import in blueprints only)
gateway/      does NOT import nav/, semantic/, drivers/
```

Planner backends resolved via `runtime.registry.get("planner_backend", name)`, not direct import.

## 10. Backpressure Policies

```python
self.image.set_policy("latest")                   # drop if busy
self.imu.set_policy("throttle", interval=0.02)    # max 50Hz
self.lidar.set_policy("sample", n=5)              # every 5th
self.detections.set_policy("buffer", size=10)      # batch of 10
```

## 11. Transport Decoupling (per-wire)

```python
bp.wire("Safety", "stop_cmd", "Driver", "stop_signal")                          # callback (0 latency)
bp.wire("Perception", "scene_graph", "Planner", "scene_graph", transport="dds")  # decoupled
bp.wire("SLAM", "cloud", "Terrain", "cloud", transport="shm")                   # high bandwidth
```

## 12. Explicit Wires (Cross-Stack, in `full_stack.py`)

```python
# Safety �?all actuators
bp.wire("SafetyRingModule", "stop_cmd", driver_name, "stop_signal")
bp.wire("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")

# SLAM odometry priority for NavigationModule
bp.wire("SlamBridgeModule", "odometry", "NavigationModule", "odometry")
# SLAM map_cloud �?all consumers (OccupancyGrid, ElevationMap, Terrain, �?
bp.wire("SlamBridgeModule", "map_cloud", "OccupancyGridModule", "map_cloud")
# (and the same for ElevationMapModule, TerrainModule, VoxelGridModule, RerunBridgeModule, GatewayModule)

# Autonomy chain (when enable_native=True; otherwise Python autonomy module replaces it)
bp.wire("NavigationModule", "waypoint",   "LocalPlannerModule", "waypoint")
bp.wire("TerrainModule",   "terrain_map", "LocalPlannerModule", "terrain_map")
bp.wire("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
bp.wire("LocalPlannerModule", "local_path", "SafetyRingModule",  "path")

# Visual servo dual channel
bp.wire("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target")
bp.wire("VisualServoModule",     "goal_pose",    "NavigationModule",  "goal_pose")
bp.wire("VisualServoModule",     "nav_stop",     "NavigationModule",  "stop_signal")

# CmdVelMux: priority arbitration over all cmd_vel sources
bp.wire("TeleopModule",       "cmd_vel",          "CmdVelMux", "teleop_cmd_vel")
bp.wire("VisualServoModule",  "cmd_vel",          "CmdVelMux", "visual_servo_cmd_vel")
bp.wire("NavigationModule",   "recovery_cmd_vel", "CmdVelMux", "recovery_cmd_vel")
bp.wire("PathFollowerModule", "cmd_vel",          "CmdVelMux", "path_follower_cmd_vel")
bp.wire("CmdVelMux",          "driver_cmd_vel",    driver_name, "cmd_vel")
# SafetyRing observes the mux winner
bp.wire("CmdVelMux",          "driver_cmd_vel",   "SafetyRingModule", "cmd_vel")

# Teleop active signal pauses NavigationModule
bp.wire("TeleopModule", "teleop_active", "NavigationModule", "teleop_active")
```

## 13. Semantic Navigation

### 5-Level Goal Resolution Chain

```
Instruction: "go to where I left the backpack"
  �?
1. Tag Lookup     �?exact/fuzzy match in TaggedLocationStore     �?goal_pose
2. Fast Path      �?scene graph keyword + CLIP matching (<200ms) �?goal_pose
3. Vector Memory  �?CLIP embedding search in ChromaDB            �?goal_pose
4. Frontier       �?topology graph information gain exploration   �?goal_pose
5. Visual Servo   �?VLM bbox detection + PD tracking             �?goal_pose/cmd_vel
```

### Fast-Slow Dual-Process (`goal_resolver.py`)

**Fast Path** (System 1, ~0.17ms): Direct scene graph matching �?keyword + spatial reasoning, confidence fusion (label 35%, CLIP 35%, detector 15%, spatial 15%). Target: >70% hit rate, threshold 0.75.

**Slow Path** (System 2, ~2s): LLM reasoning with ESCA selective grounding �?filters 200 objects to ~15 objects (92.5% token reduction), then calls LLM. Returns OmniNav hierarchical room hint.

**AdaNav Entropy Trigger**: Shannon entropy over candidate scores. If `score_entropy > 1.5` and `confidence < 0.85`, forced escalation to Slow Path.

**LERa Failure Recovery**: 3-step Look-Explain-Replan on subgoal failure. After 2nd consecutive failure: LLM decides `retry_different_path | expand_search | requery_goal | abort`.

### Visual Servo (`visual_servo_module.py`)

Two output channels based on distance:

- Far (> 3m): `goal_pose �?NavigationModule �?planning stack`
- Near (< 3m): `cmd_vel �?CmdVelMux �?driver` (PD servo, bypasses planner; mux gives it priority 80, above PathFollower at 40)

Components: BBoxNavigator (bbox+depth�?D閳墾D), PersonTracker (VLM select+CLIP Re-ID), vlm_bbox_query (open-vocab detection).

### Multi-Turn Agent Loop (`agent_loop.py`)

`agent_instruction` port triggers observe閳姲hink閳妺ct cycle with 7 LLM tools:
`navigate_to`, `navigate_to_object`, `detect_object`, `query_memory`, `tag_location`, `say`, `done`.
Max 10 steps / 120s timeout. Supports OpenAI function-calling + text JSON fallback.

### LLM Configuration

```bash
export MOONSHOT_API_KEY="..."         # Kimi (default, China-direct)
export OPENAI_API_KEY="sk-..."        # OpenAI
export ANTHROPIC_API_KEY="sk-ant-..." # Claude
export DASHSCOPE_API_KEY="sk-..."     # Qwen (China fallback)
```

## 14. SLAM / Localization


| Mode         | slam_profile | Backend                                | Use Case                    |
| ------------ | ------------ | -------------------------------------- | --------------------------- |
| Mapping      | `fastlio2`   | SLAMModule �?C++ Fast-LIO2             | First visit, build map      |
| Localization | `localizer`  | SLAMModule �?Fast-LIO2 + ICP Localizer | Navigate with pre-built map |
| Bridge       | `bridge`     | SlamBridgeModule �?ROS2 subscriber     | External SLAM (systemd)     |
| None         | `none`       | �?                                     | stub/dev mode               |


Localizer requires Fast-LIO2 companion (provides `/cloud_registered` + `/Odometry`).
SLAM odometry is explicitly wired to NavigationModule (priority over driver dead-reckoning).

## 15. C++/ROS2 Layer �?Coordinate Frames

### Frame Hierarchy

```
map (global map frame) �?fixed world reference
 閳规柡鏀㈤埞�?odom (odometry frame) �?published by PGO/Localizer �?perception + path adaptation
     閳规柡鏀㈤埞�?body (robot body frame) �?Fast-LIO2 output �?local planning + control
         閳规柡鏀㈤埞�?lidar (LiDAR sensor frame) �?raw sensor
```

- **Perception** (terrain_analysis) works in **odom** frame
- **Local planning** (local_planner) works in **body** frame
- **Global planning** (pct_planner) works in **map** frame

### TF Transforms

```
map �?odom: Published by PGO or Localizer (10-20Hz)
odom �?body: Published by Fast-LIO2 (/Odometry, 100Hz)
body �?lidar: Static transform (extrinsics from lio.yaml)
```

## 16. C++/ROS2 Layer �?Topics and Services

### Core Topics


| Topic               | Type         | Publisher                              | Subscriber       | Frame     |
| ------------------- | ------------ | -------------------------------------- | ---------------- | --------- |
| `/livox/lidar`      | CustomMsg    | livox_ros_driver2                      | fastlio2         | lidar     |
| `/livox/imu`        | Imu          | livox_ros_driver2                      | fastlio2         | body      |
| `/cloud_registered` | PointCloud2  | fastlio2                               | PGO, Localizer   | body      |
| `/cloud_map`        | PointCloud2  | fastlio2                               | terrain_analysis | odom      |
| `/Odometry`         | Odometry     | fastlio2                               | all modules      | odom閳妼ody |
| `/terrain_map`      | PointCloud2  | terrain_analysis                       | local_planner    | odom      |
| `/way_point`        | PointStamped | TaskManager (sole publisher)           | local_planner    | odom      |
| `/planner_waypoint` | PointStamped | pct_path_adapter                       | TaskManager      | odom      |
| `/pct_path`         | Path         | pct_planner                            | pct_path_adapter | map       |
| `/path`             | Path         | local_planner                          | pathFollower     | body      |
| `/cmd_vel`          | Twist        | pathFollower                           | robot_driver     | body      |
| `/slow_down`        | Int8         | local_planner                          | pathFollower     | �?        |
| `/stop`             | Int8         | local_planner, SafetyGate, TaskManager | pathFollower     | �?        |


### Services


| Service             | Type       | Provider  | Purpose                     |
| ------------------- | ---------- | --------- | --------------------------- |
| `/relocalize`       | Relocalize | Localizer | Load map and relocalize     |
| `/relocalize_check` | IsValid    | Localizer | Check relocalization status |
| `/pgo/save_maps`    | SaveMaps   | PGO       | Save optimized maps         |
| `/save_map`         | SaveMaps   | fastlio2  | Save current map            |


## 17. Task Orchestration (gRPC Remote Monitoring)

The C++ `remote_monitoring` layer provides gRPC services for the Flutter client.

### End-to-End Flow

```
Flutter App 閳光偓閳光偓gRPC閳光偓閳光偓�?ControlService (guard: Lease + AUTONOMOUS)
  �?TaskManager (sole /way_point publisher)
    �?map閳姪dom tf2 transform
    �?sequential waypoint dispatch
    �?/way_point (odom) �?local_planner �?pathFollower �?robot_driver
```

### gRPC RPCs (proto: `src/robot_proto/proto/control.proto`)


| RPC                        | Purpose                                                    |
| -------------------------- | ---------------------------------------------------------- |
| `StartTask`                | Start navigation/mapping/patrol (needs Lease + AUTONOMOUS) |
| `CancelTask`               | Cancel current task                                        |
| `PauseTask` / `ResumeTask` | Suspend/resume                                             |
| `GetTaskStatus`            | Query progress                                             |
| `GetActiveWaypoints`       | Query active waypoints (source/list/progress)              |
| `ClearWaypoints`           | Clear waypoints and stop (needs Lease)                     |
| `SetMode`                  | Switch mode (IDLE/MANUAL/TELEOP/AUTONOMOUS/MAPPING)        |
| `EmergencyStop`            | E-stop (suspends TaskManager)                              |
| `StreamTeleop`             | Bidirectional teleop stream                                |
| `AcquireLease`             | Acquire operation lease                                    |


### Waypoint Coordinate Convention

All waypoints published on `/way_point` are in **odom** frame. App waypoints (map frame) are transformed by TaskManager via tf2 `map閳姪dom`. The `local_planner` reads `/way_point` x,y directly as odom coordinates with no further transformation.

## 18. MCP Server (AI Agent Control)

Auto-discovered @skill methods exposed via JSON-RPC at `http://<robot>:8090/mcp`.


| Category     | Tools                                                                              |
| ------------ | ---------------------------------------------------------------------------------- |
| Navigation   | `navigate_to`, `navigate_to_object`, `stop`, `get_navigation_status`, `set_mode`   |
| Perception   | `get_scene_graph`, `detect_objects`, `get_robot_position`                          |
| Memory       | `query_memory`, `query_location` (vector), `list_tagged_locations`, `tag_location` |
| Semantic Map | `get_room_summary`, `query_room_for_object`, `get_exploration_target`              |
| Visual Servo | `find_object`, `follow_person`, `stop_servo`, `get_servo_status`                   |
| Planning     | `send_instruction`, `decompose_task`                                               |
| System       | `get_health`, `list_modules`, `get_config`                                         |


```bash
claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp
```

## 19. Teleop (Remote Control)

WebSocket joystick at `ws://<robot>:5050/ws/teleop`:

- Phone/browser sends `{"type": "joy", "lx": 0.5, "ly": 0.0, "az": -0.3}`
- Robot streams JPEG camera frames back (or H.264 via `WebRTCStreamModule` when `aiortc` is installed)
- 3 s idle �?auto-releases to autonomous navigation
- cmd_vel priority (`CmdVelMux`): Teleop 100 > VisualServo 80 > Recovery 60 > PathFollower 40, each with 0.5 s freshness

## 20. CLI Reference (`cli/`)

### Entry Points


| Entry           | Role                                                                                |
| --------------- | ----------------------------------------------------------------------------------- |
| `lingtu.py`     | Primary entry �?sets project root, calls `cli.bootstrap.init`, then `cli.main.main` |
| `main_nav.py`   | Backward-compatible alias (same as `lingtu.py`)                                     |
| `lingtu_cli.py` | Target of pip console script (`pyproject.toml` �?`lingtu = "lingtu_cli:main"`)      |


### CLI Arguments

```bash
lingtu [profile] [options]
```


| Argument          | Type       | Effect                                                                     |
| ----------------- | ---------- | -------------------------------------------------------------------------- |
| `profile`         | positional | Profile name (`stub`, `dev`, `sim`, `map`, `nav`, `explore`)               |
| `stop`            | positional | Stop running daemon (SIGTERM to PID from `.lingtu/run.json`)               |
| `status`          | positional | Show current external run status from `.lingtu/run.json`                   |
| `show-config`     | positional | Print the resolved config for a profile without starting the system         |
| `log`             | positional | Print the current run log; use `-f` to follow                              |
| `doctor`          | positional | Run `scripts/doctor.py` diagnostics                                        |
| `rerun`           | positional | Run `scripts/rerun_live.py` visualization                                  |
| `--list`          | flag       | List all profiles and exit                                                 |
| `--daemon` / `-d` | flag       | Fork as Unix daemon (implies no REPL)                                      |
| `--follow` / `-f` | flag       | Follow output for `lingtu log`                                             |
| `--lines`         | int        | Number of lines to show for `lingtu log` (default 80)                      |
| `--force`         | flag       | Force action for commands such as `lingtu stop`                            |
| `--robot`         | str        | Override robot preset (`stub`, `sim`, `ros2`, `s100p`, `thunder`)          |
| `--dog-host`      | str        | Override Thunder robot host IP                                             |
| `--dog-port`      | int        | Override Thunder robot gRPC port                                           |
| `--detector`      | str        | Override detector backend (`yoloe`, `bpu`, `yolo_world`, `grounding_dino`) |
| `--encoder`       | str        | Override encoder backend (`clip`, `mobileclip`)                            |
| `--llm`           | str        | Override LLM backend (`kimi`, `openai`, `claude`, `qwen`, `mock`)          |
| `--planner`       | str        | Override planner backend (`astar`, `pct`)                                  |
| `--tomogram`      | str        | Override tomogram pickle path                                              |
| `--gateway-port`  | int        | Override gateway HTTP port (default 5050)                                  |
| `--no-semantic`   | flag       | Disable semantic pipeline                                                  |
| `--no-gateway`    | flag       | Disable HTTP/WS/MCP gateway                                                |
| `--no-native`     | flag       | Disable C++ autonomy stack (use Python simple/pid)                         |
| `--rerun`         | flag       | Enable Rerun visualization bridge                                          |
| `--no-repl`       | flag       | No interactive REPL; block on signal                                       |
| `--log-level`     | str        | Root log level (default `INFO`)                                            |


### Profile Resolution Order

1. Profile dict from `PROFILES[name]` (or interactive picker if no positional arg)
2. Robot preset from `ROBOT_PRESETS[robot_key]` merged in (profile fields take priority)
3. CLI overrides (`--llm`, `--detector`, etc.) override everything
4. Resulting dict passed to `full_stack_blueprint(**cfg)`

### CLI Source Files


| File                   | Role                                                               |
| ---------------------- | ------------------------------------------------------------------ |
| `cli/__init__.py`      | Package entry                                                      |
| `cli/bootstrap.py`     | `init()`: prepend `src/`, semantic subdirs to `sys.path`           |
| `cli/main.py`          | Argparse, profile resolution, blueprint build, REPL/daemon loop    |
| `cli/profiles_data.py` | `ROBOT_PRESETS`, `PROFILES`, `_ACTIVE_TOMOGRAM`                    |
| `cli/repl.py`          | `LingTuREPL` (cmd.Cmd subclass) �?interactive commands             |
| `cli/ui.py`            | Banner, interactive profile picker, `list_profiles`, `cmd_stop`    |
| `cli/run_state.py`     | PID file + `.lingtu/run.json` for daemon lifecycle                 |
| `cli/runtime_extra.py` | `preflight`, `kill_residual_ports`, `health_check`, `daemonize`    |
| `cli/logging_util.py`  | `setup_logging` �?stderr + `logs/<timestamp>_<profile>/lingtu.log` |
| `cli/paths.py`         | Project root, `.lingtu/` dir, `logs/` base                         |
| `cli/term.py`          | TTY detection, ANSI color helpers                                  |


### Environment Variables


| Variable            | Default            | Used By                                                           |
| ------------------- | ------------------ | ----------------------------------------------------------------- |
| `NAV_MAP_DIR`       | `~/data/nova/maps` | `profiles_data.py` (`_ACTIVE_TOMOGRAM`), `repl.py` (map commands) |
| `MOONSHOT_API_KEY`  | �?                 | Kimi LLM backend                                                  |
| `OPENAI_API_KEY`    | �?                 | OpenAI LLM backend                                                |
| `ANTHROPIC_API_KEY` | �?                 | Claude LLM backend                                                |
| `DASHSCOPE_API_KEY` | �?                 | Qwen LLM backend                                                  |


### REPL Commands


| Command       | Alias       | Usage                                                                      |
| ------------- | ----------- | -------------------------------------------------------------------------- |
| `navigate`    | `nav`       | `navigate x y [z]` �?pose goal to NavigationModule                         |
| `go`          | �?          | `go <natural language>` �?semantic instruction                             |
| `stop`        | �?          | Emergency stop (value 2 to all stop_signal ports)                          |
| `cancel`      | �?          | Cancel current navigation mission                                          |
| `status`      | `s`         | Module list + mission state                                                |
| `health`      | `h`         | System health report                                                       |
| `map`         | �?          | `list` / `save <name>` / `use <name>` / `build <name>` / `delete <name>`   |
| `smap`        | �?          | `status` / `rooms` / `save` / `load <dir>` / `query <text>` (semantic map) |
| `agent`       | �?          | `agent <multi-step instruction>` (AgentLoop)                               |
| `vmem`        | �?          | `query <text>` / `stats` (vector memory)                                   |
| `teleop`      | �?          | `status` / `release`                                                       |
| `rerun`       | �?          | `on` / `off` / `status` (Rerun visualization bridge)                       |
| `watch`       | `w`         | `watch [interval]` �?auto-refresh status (default 2s)                      |
| `live`        | �?          | Full-screen dashboard with hotkeys (`s/g/x/n/q`)                           |
| `module`      | `m`         | `module <name>` �?inspect one module (tab completion)                      |
| `connections` | `c`         | List all wires                                                             |
| `log`         | �?          | `log debug|info|warning|error` �?set log level                             |
| `config`      | �?          | Print current profile configuration                                        |
| `quit`        | `q`, `exit` | Exit REPL                                                                  |


### Daemon Mode

```bash
python lingtu.py s100p --daemon    # fork to background, log to logs/
python lingtu.py status            # inspect current run
python lingtu.py log -f            # follow the current run log
python lingtu.py show-config nav   # print resolved nav config
python lingtu.py stop              # SIGTERM to running daemon
```

Daemon uses Unix double-fork (`setsid`), writes PID to `.lingtu/run.pid`, state to `.lingtu/run.json`. On startup, `kill_residual_ports` cleans ports 5050, 8090 via `fuser -k`.

## 21. Configuration Files


| File                                       | Purpose                                            |
| ------------------------------------------ | -------------------------------------------------- |
| `config/robot_config.yaml`                 | Robot physical parameters (single source of truth) |
| `config/semantic_planner.yaml`             | Semantic planner configuration                     |
| `config/semantic_perception.yaml`          | Perception pipeline configuration                  |
| `config/semantic_exploration.yaml`         | Exploration parameters                             |
| `config/topic_contract.yaml`               | Port name/type contracts                           |
| `config/layer_contract.yaml`               | Layer dependency contracts                         |
| `config/qos_profiles.yaml`                 | QoS/backpressure profiles                          |
| `config/cyclonedds.xml`                    | CycloneDDS configuration                           |
| `src/localization/fastlio2/config/lio.yaml`        | Fast-LIO2 parameters                               |
| `src/localization/pgo/config/pgo.yaml`             | PGO parameters                                     |
| `src/localization/localizer/config/localizer.yaml` | Localizer parameters                               |


## 22. S100P Deployment

- **SSH**: `ssh sunrise@192.168.66.190`
- **Nav code**: `~/data/inovxio/lingtu/`
- **Nav deploy**: `/opt/lingtu/nav/`
- **CycloneDDS**: Built from source at `~/cyclonedds/install/`
- **Python**: 3.10.12, cyclonedds==0.10.5

## 23. Code Style

- **C++**: Google style (`.clang-format`, 2-space indent, 100 col)
- **Python**: English comments in new code. Chinese comments exist in legacy code.
- **Framework**: All Modules use `runtime.Module` base with In[T]/Out[T] type hints
- **No ROS2 in Modules**: rclpy only in Bridge modules and C++ NativeModule launchers

## 24. Known Limitations

- Fast Path uses rule-based matching (not learned policies)
- S100P has no CUDA �?Open3D GPU features unavailable, use C++ terrain_analysis instead
- Kimi API key may expire �?Slow Path unavailable without valid LLM key
- ChromaDB optional �?VectorMemoryModule falls back to numpy brute-force search
- Framework tests (640) are mock-based �?real hardware integration tests need S100P

## 25. Related Documentation


| Document                                          | Purpose                                           |
| ------------------------------------------------- | ------------------------------------------------- |
| `CLAUDE.md`                                       | Claude Code-specific guidance (mirrors this file) |
| `docs/MODULE_FIRST_GUIDELINE.md`                  | 8 rules for Module-First architecture             |
| `docs/REPO_LAYOUT.md`                             | Top-level directory map                           |
| `docs/QUICKSTART.md`                              | Getting started guide                             |
| `docs/TUNING.md`                                  | Parameter tuning guide                            |
| `src/base_autonomy/TERRAIN_ANALYSIS_EXPLAINED.md` | Terrain analysis deep dive                        |
| `src/remote_monitoring/README.md`                 | gRPC Gateway documentation                        |
| `src/robot_proto/proto/control.proto`             | Task control protocol definition                  |


---

## 26. Feature-by-Feature Breakdown and Quick Verification

### 26.1 Localization (鐎规矮缍?

**What it does**: Matches live LiDAR scans against a pre-built PCD map using ICP to produce a corrected `map閳姪dom` TF transform. Requires Fast-LIO2 running as the odometry source.

**Key source files**:


| File                                            | Role                                                                                |
| ----------------------------------------------- | ----------------------------------------------------------------------------------- |
| `src/localization/localizer/`                           | C++ ICP localizer node                                                              |
| `src/localization/localizer/config/localizer.yaml`      | `static_map_path`, ICP resolutions, `update_hz`                                     |
| `src/localization/launch/localizer_launch.py` | Launches Fast-LIO2 + localizer + RViz                                               |
| `src/localization/slam_module.py`                       | Python `SLAMModule` �?`_setup_localizer()` starts LIO + localizer via NativeModule  |
| `src/localization/bridge.py`                | `SlamBridgeModule` �?subscribes to `/nav/odometry` + `/nav/map_cloud` via DDS/rclpy |
| `src/runtime/blueprints/stacks/slam.py`            | `slam("localizer")` �?ensures systemd services, adds `SlamBridgeModule`             |


**Data flow**:

```
Livox LiDAR �?Fast-LIO2 (/cloud_registered + /Odometry)
                 �?
           Localizer (ICP match against PCD)
                 �?
           TF: map閳姪dom + /localization_quality
                 �?
           SlamBridgeModule �?Python odometry/map_cloud ports
```

**Quick verification**:


| Method                     | Command                                                                                    | Needs             |
| -------------------------- | ------------------------------------------------------------------------------------------ | ----------------- |
| Framework ports/registry   | `python -m pytest src/runtime/tests/test_new_modules.py::TestSLAMModule -q`                   | Nothing           |
| Bridge module smoke        | `python -m pytest src/runtime/tests/test_integration_36.py -q`                                | Nothing           |
| Profile build check        | `python -c "from runtime.blueprints.stacks import slam; bp = slam('localizer'); print('OK')"` | Nothing           |
| Full ROS2 localizer        | `ros2 launch localizer localizer_launch.py` + `ros2 service call /relocalize ...`          | ROS2 + LiDAR      |
| Check localization quality | `ros2 topic echo /localization_quality`                                                    | Running localizer |


**Key parameters** (`localizer.yaml`):

- `static_map_path` �?PCD map file (empty = no auto-load, must call `/relocalize`)
- `update_hz` �?ICP update rate
- Rough/refine ICP resolutions and score thresholds

---

### 26.2 Mapping (瀵ゅ搫娴?

**What it does**: Builds a 3D point cloud map using Fast-LIO2 (LiDAR-Inertial odometry) with optional PGO loop closure. The map is saved as PCD and then processed offline into a Tomogram for planning.

**Key source files**:


| File                                          | Role                                                      |
| --------------------------------------------- | --------------------------------------------------------- |
| `src/localization/fastlio2/`                          | C++ Fast-LIO2 SLAM node                                   |
| `src/localization/pgo/`                               | C++ Pose Graph Optimization (loop closure)                |
| `src/localization/slam_module.py`                     | `_setup_fastlio2()` �?starts Livox + LIO + PGO            |
| `src/runtime/blueprints/stacks/slam.py`          | `slam("fastlio2")` �?ensures `slam` + `slam_pgo` services |
| `src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/tomography/` | Offline PCD �?Tomogram conversion                         |


**Data flow**:

```
Livox LiDAR �?Fast-LIO2 (/cloud_map + /Odometry)
                 �?
              PGO (loop closure, optional)
                 �?
           /pgo/save_maps �?map.pcd
                 �?(offline)
           tomography.py �?tomogram.pickle
```

**Quick verification**:


| Method              | Command                                                                                                                                                           | Needs        |
| ------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------ |
| Framework smoke     | `python -m pytest src/runtime/tests/test_new_modules.py::TestSLAMModule -q`                                                                                          | Nothing      |
| Profile `map` check | `python -c "from runtime.blueprints.full_stack import full_stack_blueprint; bp = full_stack_blueprint(slam_profile='fastlio2', enable_semantic=False); print('OK')"` | Nothing      |
| Map REPL            | `python lingtu.py map` then `map save test_map`                                                                                                                   | ROS2 + LiDAR |
| ROS2 direct         | `ros2 launch fastlio2_launch.py` + `ros2 service call /save_map ...`                                                                                          | ROS2 + LiDAR |
| Tomogram generation | `cd src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner && python3 tomography/scripts/tomography.py --scene Common`                                                                   | Saved PCD    |


**Key parameters**:

- `lio.yaml`: `lidar_type`, `scan_line`, `map_resolution`, `max_map_points`, extrinsics (`t_il`, `r_il`)
- `pgo.yaml`: keyframe deltas, loop search radius, score thresholds
- Map storage: `~/data/nova/maps/<name>/map.pcd` (default `NAV_MAP_DIR`)

---

### 26.3 Global Navigation (閸忋劌鐪€佃壈�?

**What it does**: Given a `goal_pose`, plans a global path using A* (or PCT) on a 2D costmap or tomogram, then dispatches waypoints to the local planner. Handles mission FSM (IDLE閳墾LANNING閳墮XECUTING閳壌UCCESS/STUCK/FAILED), stuck detection, and replanning.

**Key source files**:


| File                                                            | Role                                                                            |
| --------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| `src/nav/mission/navigation_module.py`                                  | `NavigationModule` �?mission FSM, goal handling, replan logic                   |
| `src/nav/services/plan/global_planner/service.py`                             | `GlobalPlannerService` �?A*/PCT backend, `_find_safe_goal` BFS, path downsample |
| `src/nav/mission/tracking/waypoint_tracker.py`                                   | `WaypointTracker` �?2D arrival detection, stuck timeout                         |
| `src/nav/services/map_layers/occupancy_grid_module.py`                              | `OccupancyGridModule` �?LiDAR �?2D costmap (feeds live A* replanning)           |
| `src/nav/services/map_layers/esdf_module.py`                                        | `ESDFModule` �?signed distance field from occupancy                             |
| `src/nav/services/map_layers/elevation_map_module.py`                               | `ElevationMapModule` �?per-cell height                                          |
| `src/nav/services/plan/global_planner/algorithm/pct/backend.py` | Registers `_AStarBackend` and `_PCTBackend`                                     |
| `src/runtime/blueprints/stacks/navigation.py`                      | `navigation()` stack factory                                                    |


**Data flow**:

```
goal_pose (from Gateway/MCP/SemanticPlanner)
    �?
NavigationModule._on_goal �?_plan()
    �?
GlobalPlannerService.plan(start, goal)
    �?_find_safe_goal (BFS snap to free cell)
    �?_AStarBackend.plan (2D A* on costmap/tomogram)
    �?_downsample (sparse waypoints, 2m apart)
    �?
WaypointTracker.reset(path) �?first waypoint
    �?
NavigationModule publishes waypoint (PoseStamped)
    �?
LocalPlannerModule �?PathFollowerModule �?cmd_vel �?Driver

Parallel: OccupancyGridModule �?costmap �?GlobalPlannerService.update_map
          (live replanning when new obstacles detected, �?s cooldown)
```

**Mission states**: `IDLE` �?`PLANNING` �?`EXECUTING` �?`SUCCESS` | `STUCK` (replan up to 3x) | `FAILED` | `CANCELLED`

**Quick verification**:


| Method                | Command                                                                        | Needs       |
| --------------------- | ------------------------------------------------------------------------------ | ----------- |
| NavigationModule FSM  | `python -m pytest src/runtime/tests/test_new_modules.py::TestNavigationModule -q` | Nothing     |
| Safe goal + planner   | `python -m pytest src/runtime/tests/test_integration_36.py -q`                    | Nothing     |
| WaypointTracker stuck | `python -m pytest src/runtime/tests/test_new_features.py -q`                      | Nothing     |
| Full wiring check     | `python -m pytest src/runtime/tests/test_cross_module_integration.py -q`          | Nothing     |
| Non-native blueprint  | `python -m pytest src/runtime/tests/test_non_native_navigation_blueprint.py -q`   | Nothing     |
| PCT adapter logic     | `python3 tests/planning/test_pct_adapter_logic.py`                             | Nothing     |
| Stub profile (live)   | `python lingtu.py stub` then REPL: `go 10 5`                                   | Nothing     |
| Sim profile (live)    | `python lingtu.py sim` then REPL: `go 10 5`                                    | MuJoCo deps |


**Key parameters** (`NavigationModule`):

- `planner`: `"astar"` or `"pct"` (registry backend)
- `tomogram`: path to `.pickle` (PCT/A* map source)
- `waypoint_threshold`: 1.5m (arrival distance)
- `stuck_timeout`: 10s / `stuck_dist_thre`: 0.15m
- `max_replan_count`: 3
- `downsample_dist`: 2.0m (waypoint spacing)
- `allow_direct_goal_fallback`: True (single-waypoint if no map)

---

### 26.4 Local Planning (鐏炩偓闁劏顫夐�?

**What it does**: Receives a global waypoint + terrain map, selects a collision-free local path from a pre-computed path library (CMU-style) or simple straight-line, then follows it with Pure Pursuit to produce `cmd_vel`.

**Key source files**:


| File                                                | Role                                                         |
| --------------------------------------------------- | ------------------------------------------------------------ |
| `src/nav/local/terrain_module.py`       | `TerrainModule` �?point cloud �?terrain map + traversability |
| `src/nav/local/local_planner.py` | `LocalPlannerModule` �?waypoint + terrain �?local path       |
| `src/nav/local/path_follower_module.py` | `PathFollowerModule` �?local path �?cmd_vel                  |
| `src/nav/local/autonomy_module.py`      | `add_autonomy_stack()` �?adds all three modules              |
| `src/base_autonomy/terrain_analysis/`               | C++ terrain analysis node                                    |
| `src/base_autonomy/local_planner/`                  | C++ local planner + pathFollower                             |


**Data flow**:

```
NavigationModule �?waypoint (PoseStamped)
    �?
LocalPlannerModule (In: waypoint + terrain_map + odometry)
    �?score candidate paths against obstacle grid
    �?select best group, lowest penalty
    �?
local_path (Path) �?PathFollowerModule
    �?Pure Pursuit: lookahead + yaw rate limiting
    �?
cmd_vel (Twist) �?Driver
```

**Backends**:

- Terrain: `nanobind` (C++ `_nav_runtime.TerrainAnalysisCore`), `native` (subprocess), `simple`
- Local planner: `cmu` (CMU-style 12,348 candidate paths), `simple` (straight line)
- Path follower: `nav_core` (C++ nanobind), `pure_pursuit` (native), `pid`

**Quick verification**:


| Method                         | Command                                                                          | Needs               |
| ------------------------------ | -------------------------------------------------------------------------------- | ------------------- |
| Module smoke (simple/pid)      | `python -m pytest src/runtime/tests/test_new_modules.py::TestLocalPlannerModule -q` | Nothing             |
| PathFollower smoke             | `python -m pytest src/runtime/tests/test_new_modules.py::TestPathFollowerModule -q` | Nothing             |
| Wiring Nav閳墶ocal閳墯ollow閳墬river | `python -m pytest src/runtime/tests/test_cross_module_integration.py -q`            | Nothing             |
| Non-native stack               | `python -m pytest src/runtime/tests/test_non_native_navigation_blueprint.py -q`     | Nothing             |
| C++ nodes (ROS2)               | `ros2 launch local_planner local_planner.launch`                                 | ROS2 + built C++    |
| Stub planning pipeline         | `bash tests/integration/test_planning_stub.sh`                                   | ROS2 + `make build` |


**Key parameters**:

- `LocalPlannerModule`: `_PATH_RANGE=3.5m`, `_OBSTACLE_HEIGHT_THRE=0.5m`, `_DIR_THRE=90deg`
- `PathFollowerModule`: `max_speed`, `lookahead`, `yaw_rate_limit`
- C++ `localPlanner`: `vehicleHeight`, `vehicleWidth`, `adjacentRange`
- C++ `pathFollower`: `baseLookAheadDis_`, `lookAheadRatio_`, `slowRate1/2/3`

---

### 26.5 Semantic Navigation (鐠囶厺绠熺€佃壈�?

**What it does**: Resolves natural language instructions ("go to the red chair") into `goal_pose` through a 5-level fallback chain: tag lookup �?fast scene-graph matching �?vector memory search �?frontier exploration �?visual servo. Includes LERa failure recovery.

**Key source files**:


| File                                                               | Role                                                      |
| ------------------------------------------------------------------ | --------------------------------------------------------- |
| `src/decision/semantic_planner/semantic_planner_module.py` | Unified planner: instruction �?`_try_resolve` �?goal_pose |
| `src/decision/semantic_planner/goal_resolver.py`           | `GoalResolver` �?fusion weights, KG reload                |
| `src/decision/semantic_planner/fast_path.py`               | `fast_resolve()` �?System 1 matching (~0.17ms)            |
| `src/decision/semantic_planner/slow_path.py`               | `async resolve()` �?tag + AdaCoT + LLM (~2s)              |
| `src/decision/semantic_planner/agent_loop.py`              | Multi-turn LLM tool calling (7 tools, 10 steps max)       |
| `src/decision/semantic_planner/task_decomposer.py`         | Rules-based instruction decomposition                     |
| `src/decision/semantic_planner/action_executor.py`         | LERa failure recovery strategies                          |
| `src/perception/semantic_perception/perception_module.py` | RGB-D + odom �?scene graph                                |
| `src/perception/semantic_perception/instance_tracker.py`  | Multi-object tracking, scene graph builder                |
| `src/memory/modules/semantic_mapper_module.py`                     | Scene graph �?RoomObjectKG + TopologySemGraph             |
| `src/memory/modules/vector_memory_module.py`                       | CLIP + ChromaDB vector search                             |
| `src/memory/modules/tagged_locations_module.py`                    | Named location store                                      |
| `src/memory/modules/episodic_module.py`                            | Episodic memory for LLM context                           |


**Data flow**:

```
instruction (str) from Gateway/MCP
    �?
SemanticPlannerModule._on_instruction
    �?optional TaskDecomposer
    �?
_try_resolve(instruction, scene_graph_json):
    閳规壕鏀?Level 2: GoalResolver.fast_resolve() �?keyword + CLIP fusion
    �?  weights: label 35%, CLIP 35%, detector 15%, spatial 15%
    �?  if confidence �?0.75 �?goal_pose �?
    �?
    閳规壕鏀?Level 3: VectorMemoryModule.query_location(instruction)
    �?  CLIP embedding search �?if score �?0.3 �?goal_pose �?
    �?
    閳规壕鏀?Level 4: FrontierScorer.get_best_frontier()
    �?  topology graph information gain �?goal_pose (EXPLORING)
    �?
    閳规柡鏀?Level 5: servo_target.publish("find:<instruction>")
        �?VisualServoModule (see Tracking below)

LERa Recovery (on mission_status STUCK/FAILED):
    ActionExecutor.lera_recover �?retry_different_path | expand_search | requery_goal | abort
```

**Agent loop** (triggered by `agent_instruction` port):

```
agent_instruction �?AgentLoop(llm_client, 7 tools, max_steps=10, timeout=120s)
    �?observe閳姲hink閳妺ct cycle
    tools: navigate_to, navigate_to_object, detect_object,
           query_memory, tag_location, say, done
```

**Quick verification**:


| Method                   | Command                                                                        | Needs              |
| ------------------------ | ------------------------------------------------------------------------------ | ------------------ |
| Planner module wiring    | `python -m pytest src/runtime/tests/test_semantic_planner_modules.py -q`          | Nothing            |
| Visual servo integration | `python -m pytest src/runtime/tests/test_visual_servo_semantic_integration.py -q` | Nothing            |
| Full semantic pipeline   | `python -m pytest src/runtime/tests/test_sim_semantic_pipeline_blueprint.py -q`   | Nothing            |
| GoalResolver unit tests  | `python -m pytest src/decision/tests/test_goal_resolver.py -q`          | `semantic_common`  |
| Fast resolve tests       | `python -m pytest src/decision/tests/test_fast_resolve.py -q`           | `semantic_common`  |
| BBoxNavigator tests      | `python -m pytest src/decision/tests/test_bbox_navigator.py -q`         | `semantic_common`  |
| PersonTracker tests      | `python -m pytest src/decision/tests/test_person_tracker.py -q`         | `semantic_common`  |
| Instance tracker tests   | `python -m pytest src/perception/tests/test_instance_tracker.py -q`   | `semantic_common`  |
| Dev profile (live)       | `python lingtu.py dev` then REPL: `go red chair`                               | Nothing (mock LLM) |
| Dev with real LLM        | `python lingtu.py dev --llm kimi` then REPL: `go red chair`                    | `MOONSHOT_API_KEY` |


**Key parameters**:

- `SemanticPlannerModule`: `fast_path_threshold=0.75`, `lera_cooldown=15s`, `save_dir`
- `GoalResolver`: confidence weights (label/CLIP/detector/spatial), KG reload interval (120s)
- `PerceptionModule`: `detector_type`, `confidence_threshold`, `skip_frames`
- `VectorMemoryModule`: `store_interval`, `max_results`, `persist_dir`

---

### 26.6 Tracking and Following (鐠虹喕閲?

**What it does**: Tracks and follows objects or people using visual servo. Two modes: **Find** (detect and approach a named object) and **Follow** (track a specific person). Dual output channels: far-range publishes `goal_pose` to NavigationModule, close-range publishes `cmd_vel` directly to driver with PD control.

**Key source files**:


| File                                                           | Role                                                |
| -------------------------------------------------------------- | --------------------------------------------------- |
| `src/decision/semantic_planner/visual_servo_module.py` | `VisualServoModule` �?dual-channel servo controller |
| `src/decision/semantic_planner/bbox_navigator.py`      | `BBoxNavigator` �?bbox + depth �?3D �?PD control    |
| `src/decision/semantic_planner/person_tracker.py`      | `PersonTracker` �?VLM select + CLIP Re-ID           |


**Data flow �?Find mode** (object tracking):

```
servo_target = "find:red chair"
    �?
VisualServoModule._tick_find
    �?match label in scene_graph.objects
    �?BBoxNavigator.update(bbox, depth)
    �?compute 3D position + distance
    �?
    閳规壕鏀?distance > 3m (far): goal_pose �?NavigationModule �?planning stack
    �?
    閳规柡鏀?distance �?3m (near): cmd_vel �?Driver directly (PD servo)
                              + nav_stop �?NavigationModule (pause planner)
```

**Data flow �?Follow mode** (person following):

```
servo_target = "follow:person_description"
    �?
VisualServoModule._tick_follow
    �?PersonTracker.update(scene_objects, rgb_frame)
    �?get_follow_waypoint �?goal_pose
    �?
NavigationModule �?planning stack (always far-range for follow)
```

**Mutual exclusion**: When close-range servo is active, `nav_stop=1` is published to pause NavigationModule so PathFollower stops producing conflicting `cmd_vel`.

**Wiring in `full_stack.py`**:

```python
bp.wire("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target")
bp.wire("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose")
bp.wire("VisualServoModule", "nav_stop", "NavigationModule", "stop_signal")
bp.wire("VisualServoModule", "cmd_vel", driver_name, "cmd_vel")
```

**Quick verification**:


| Method                 | Command                                                                        | Needs             |
| ---------------------- | ------------------------------------------------------------------------------ | ----------------- |
| Servo integration test | `python -m pytest src/runtime/tests/test_visual_servo_semantic_integration.py -q` | Nothing           |
| BBoxNavigator unit     | `python -m pytest src/decision/tests/test_bbox_navigator.py -q`         | `semantic_common` |
| PersonTracker unit     | `python -m pytest src/decision/tests/test_person_tracker.py -q`         | `semantic_common` |
| Full wiring check      | `python -m pytest src/runtime/tests/test_cross_module_integration.py -q`          | Nothing           |
| MCP tools              | `python -m pytest src/decision/tests/test_mcp_server.py -q`             | `semantic_common` |


**Key parameters**:

- `VisualServoModule`: `servo_takeover_distance=3.0m`, `follow_distance`, `lost_timeout`
- `BBoxNavigator` (`BBoxNavConfig`): `target_distance`, `max_linear_speed`, `max_angular_speed`, `lost_timeout`
- cmd_vel priority: Teleop > VisualServo > PathFollower

---

## 27. Testing Infrastructure Summary

### Test Categories


| Category         | Location                         | Count       | ROS2? | Hardware? | Command                                              |
| ---------------- | -------------------------------- | ----------- | ----- | --------- | ---------------------------------------------------- |
| Core framework   | `src/runtime/tests/`                | ~640        | No    | No        | `python -m pytest src/runtime/tests/ -q`                |
| Semantic planner | `src/decision/tests/`     | ~30 files   | No    | No        | `python -m pytest src/decision/tests/ -q`     |
| Perception       | `src/perception/tests/` | ~15 files   | No    | No        | `python -m pytest src/perception/tests/ -q` |
| Planning logic   | `tests/planning/`                | ~5 files    | No    | No        | `python3 tests/planning/test_pct_adapter_logic.py`   |
| Integration      | `tests/integration/`             | ~10 scripts | Yes   | Some      | `make test-integration`                              |
| Benchmark        | `tests/benchmark/`               | 3 scripts   | Yes   | No        | `make benchmark`                                     |
| ROS2 colcon      | Various `package.xml`            | �?          | Yes   | No        | `make test`                                          |


### Quick Verification Ladder (fastest to most complete)

```bash
# 1. Framework only (no deps, ~50s) �?validates Module/Blueprint/Registry/Transport
python -m pytest src/runtime/tests/ -q

# 2. Semantic planner (may skip without semantic_common)
python -m pytest src/decision/tests/ -q

# 3. Perception (may skip without semantic_common/cv2)
python -m pytest src/perception/tests/ -q

# 4. Stub profile (live REPL, no hardware)
python lingtu.py stub

# 5. Dev profile (semantic pipeline, no hardware, mock LLM)
python lingtu.py dev

# 6. Sim profile (MuJoCo + full stack)
python lingtu.py sim

# 7. ROS2 stub pipeline (needs built workspace)
bash tests/integration/test_planning_stub.sh

# 8. Full integration (needs ROS2 + built workspace)
make test-integration
```

### Profile-Based Verification


| Profile   | What it exercises                                                                | How to start               |
| --------- | -------------------------------------------------------------------------------- | -------------------------- |
| `stub`    | Framework, Blueprint wiring, Gateway, A* planner (no semantic, no hardware)      | `python lingtu.py stub`    |
| `dev`     | All of `stub` + perception + memory + semantic planner + visual servo (mock LLM) | `python lingtu.py dev`     |
| `sim`     | Full algorithm stack with MuJoCo physics simulation + native C++ autonomy        | `python lingtu.py sim`     |
| `map`     | SLAM mapping mode (Fast-LIO2 + PGO)                                              | `python lingtu.py map`     |
| `s100p`   | Real robot with localizer + semantic + BPU detector                              | `python lingtu.py s100p`   |
| `explore` | Real robot with SLAM + full native + semantic (no pre-built map)                 | `python lingtu.py explore` |


