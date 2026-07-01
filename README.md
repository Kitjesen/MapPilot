<p align="center">
  <img src="docs/assets/logo.png" alt="LingTu" width="480" />
</p>

<h1 align="center">LingTu (灵途</h1>

<p align="center">
  <strong>Autonomous navigation for quadruped robots in real-world environments</strong>
</p>

<p align="center">
  <a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS2-Humble-blue" alt="ROS2" /></a>
  <a href="#platform"><img src="https://img.shields.io/badge/Platform-S100P_(RDK_X5)-green" alt="Platform" /></a>
  <img src="https://img.shields.io/badge/Framework_tests-1300%2B_passed-brightgreen" alt="Framework tests" />
  <img src="https://img.shields.io/badge/Version-2.3.0-orange" alt="Version" />
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-lightgrey" alt="License" /></a>
</p>

---

## What is LingTu?

LingTu is a full-stack autonomous navigation system that runs on quadruped robots. It handles everything from LiDAR SLAM and terrain analysis to semantic navigation ("go to the kitchen") and remote control via a web dashboard.

Simulation and framework tests are not field-readiness evidence. The strict algorithm claim boundary is tracked in `docs/07-testing/ALGORITHM_VALIDATION_FLOW.md`.

> **Recent restructuring (v2.3.0, Sprint 5-6):** The project underwent a major engineering overhaul —module test directories separated from core/, directory naming unified to snake_case, ROS2 extracted to bridge modules, security hardening applied (CORS, shell injection), and engineering standards enforced (mypy strict, ruff rules). See [CHANGELOG.md](CHANGELOG.md) for details.

**Key capabilities:**
- **SLAM** —Fast-LIO2 mapping + ICP localization against pre-built maps
- **Terrain-aware planning** —OctoPlanner3D by default, with explicit A*/PCT compatibility backends
- **Semantic navigation** —Natural language goals via LLM (Kimi/Qwen/OpenAI/Claude)
- **Web Dashboard** —Real-time camera, telemetry, map management, joystick teleop
- **MCP Server** —AI agent control interface (16 tools for LLM-based task execution)
- **Camera auto-recovery** —Gateway/module watchdog with optional ROS2
  compatibility fallback

## Platform

| Component | Specification |
|-----------|--------------|
| Compute | S100P (RDK X5, Nash BPU 128 TOPS, aarch64) |
| LiDAR | Livox MID-360 |
| RGB-D Camera | Orbbec Gemini 335 |
| OS | Ubuntu 22.04; ROS2 Humble is optional for compatibility services |
| Motion Control | Brainstem (gRPC :13145) |

Dual-board architecture: **Nav Board** (S100P) runs LingTu; **Dog Board** runs Brainstem motor control. Connected via Ethernet.

## Quick Start

### Python environment

Use `uv` for reproducible Python installs. `uv.lock` is the source of truth for
resolved package versions, and `.python-version` pins the default interpreter to
Python 3.10.12 to match the S100P runtime.

```bash
uv sync --locked
uv run --locked python lingtu.py --list
```

Install extras only for the profile you are running:

```bash
uv sync --locked --extra vision --extra ml --extra llm --extra nlp
uv sync --locked --extra perception --extra vector   # heavy semantic deps
uv sync --locked --extra dev                         # test/lint tooling
```

### Framework tests (no hardware needed)

```bash
uv run --locked --extra dev python -m pytest src/runtime/tests/ -q    # framework tests only; not full algorithm closure
```

### Run on robot

```bash
# Interactive profile picker
uv run --locked python lingtu.py

# Common profiles
uv run --locked python lingtu.py stub                   # no hardware, framework testing
uv run --locked python lingtu.py dev                    # semantic pipeline, no C++ nodes
uv run --locked python lingtu.py sim                    # MuJoCo simulation (full stack)
uv run --locked python lingtu.py map                    # SLAM mapping mode
uv run --locked python lingtu.py nav                    # navigate using a saved map
uv run --locked python lingtu.py explore                # exploration, no pre-built map

# Override flags
uv run --locked python lingtu.py nav --llm mock         # real robot but mock LLM
uv run --locked python lingtu.py nav --daemon           # background daemon (systemd)
```

### Lifecycle commands

```bash
uv run --locked python lingtu.py status                 # PID, profile, uptime, module count
uv run --locked python lingtu.py status --json          # machine-readable
uv run --locked python lingtu.py log -f                 # follow logs
uv run --locked python lingtu.py stop                   # graceful stop
uv run --locked python lingtu.py restart                # stop + relaunch
uv run --locked python lingtu.py doctor                 # diagnostics
```

## Architecture

<p align="center">
  <img src="docs/assets/lingtu-current-architecture.png" alt="LingTu current system architecture" width="920" />
</p>

LingTu is organized as a Module-First runtime: `Module` is the runtime unit,
`Blueprint` is the composition unit, and `contracts` define the message
boundaries used by Gateway, navigation, mapping, semantics, and safety.

```
L0  Safety       SafetyRingModule + GeofenceManager + CmdVelMux
L1  Hardware     Driver + CameraBridge + LiDAR + SLAM
L2  Maps         OccupancyGrid + ESDF + ElevationMap + Terrain + PathFollower
L3  Perception   Detector (BPU/YOLOE) + CLIP Encoder + SemanticMapper + Memory
L4  Decision     SemanticPlanner + LLM + VisualServo + AgentLoop
L5  Planning     NavigationModule (OctoPlanner3D/A*/PCT + WaypointTracker + mission FSM)
L6  Interface    GatewayModule (HTTP/WS/SSE) + MCPServer + TeleopModule
```

High layers →low layers only. All modules use the `runtime.Module` base class with `In[T]`/`Out[T]` typed ports. Modules are composed via `Blueprint` and factory functions.

### Composable Stack API

```python
from runtime.blueprint import autoconnect
from runtime.blueprints.stacks import *

system = autoconnect(
    driver("thunder", host="192.168.66.190"),
    slam("localizer"),
    maps(),
    perception("bpu"),
    memory(),
    planner("kimi"),
    navigation("octoplanner3d"),
    safety(),
    gateway(5050),
).build()
system.start()
```

### Pluggable Backends

| Module | Backends |
|--------|----------|
| Driver | `thunder` (gRPC→Brainstem), `stub`, `sim_mujoco`, `sim_ros2` |
| SLAM | `fastlio2`, `pointlio`, `localizer` (ICP), `bridge` (adapter-backed localization: ROS2 or LCM endpoint) |
| Detector | `yoloe`, `yolo_world`, `bpu` (Nash 128 TOPS), `grounding_dino` |
| LLM | `kimi`, `openai`, `claude`, `qwen`, `mock` |
| Planner | `octoplanner3d` (default), `pct` (legacy/manual experiment), `astar` (legacy test backend) |

All backends registered via `@register("category", "name")`. Zero if/else.

## Web Dashboard

Frosted-glass UI served at `http://<robot>:5050`. Access via SSH tunnel:

```bash
ssh -L 5050:localhost:5050 sunrise@192.168.66.190
# Then open http://localhost:5050
```

### Tabs

| Tab | Description |
|-----|-------------|
| **Console** | Camera live feed + GPS globe + MiniMap + AI chat —all draggable/resizable widgets |
| **Scene** | RViz-style 2D/3D view —click to send nav goals, layer toggles, map management |
| **Map** | 3D point cloud viewer + map CRUD (save/activate/rename/delete) |
| **SLAM** | Switch between mapping (fastlio2) and localization (localizer) modes |

### Features

- **Floating Widgets** —Drag, resize, z-order. Layout persisted to localStorage
- **Camera HUD** —SLAM Hz, speed, battery, temperature, latency overlay
- **`/` Command autocomplete** —Type `/` for Claude-Code-style command dropdown
- **Quick commands** —One-click chips: 停止, 状态 去充电站, 回原点- **Brand Modals** —Glassmorphism dialogs replace native prompt/confirm
- **Camera auto-rotate** —Backend `cv2.rotate()` compensates for sideways mounting

### Tech

React 19 + TypeScript + Vite 8 + CSS Modules. WebGL point cloud rendering. Canvas 2D maps. Zero runtime CSS libraries. 18 components, ~80KB CSS, ~290KB JS (gzipped ~90KB).

## REPL Commands

```
go/navigate <target>     Navigate to coordinate or semantic target
stop / cancel            Emergency stop / cancel navigation
status                   System status
map list|save|use|delete Map management
smap status|rooms|query  Semantic map
agent <instruction>      Multi-step LLM agent
teleop status|release    Remote control
health                   Module health
watch <port>             Live port monitor
```

## MCP Server (AI Agent Control)

16 tools exposed via JSON-RPC at `http://<robot>:8090/mcp`:

| Category | Tools |
|----------|-------|
| Navigation | `navigate_to`, `navigate_to_object`, `stop`, `get_navigation_status` |
| Perception | `get_scene_graph`, `detect_objects`, `get_robot_position` |
| Memory | `query_memory`, `query_location`, `list_tagged_locations`, `tag_location` |
| Planning | `send_instruction`, `decompose_task` |
| System | `get_health`, `list_modules`, `get_config` |

```bash
# Connect Claude Code to the robot
claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp
```

## Semantic Navigation

### 5-Level Goal Resolution

```
Instruction: "去上次放背包的地方
  →1. Tag Lookup     —exact/fuzzy match in TaggedLocationStore     →goal_pose
2. Fast Path      —scene graph keyword + CLIP matching (<200ms) →goal_pose
3. Vector Memory  —CLIP embedding search in ChromaDB            →goal_pose
4. Frontier       —topology graph exploration                   →goal_pose
5. Visual Servo   —VLM bbox detection + PD tracking             →cmd_vel
```

### Fast-Slow Dual Process

- **Fast Path** (System 1, ~0.17ms): Direct scene graph matching
- **Slow Path** (System 2, ~2s): LLM reasoning with ESCA selective grounding

### Multi-Turn Agent Loop

`agent <instruction>` triggers observe→think→act with 7 LLM tools. Max 10 steps / 120s timeout.

## Mapping & Navigation Workflow

### 1. Build a map

```bash
python lingtu.py map                    # Start SLAM mapping
# Drive the robot around, then in REPL:
> map save office_2f                    # Saves map.pcd + tomogram.pickle + occupancy.npz
> map list                              # Verify
```

### 2. Navigate

```bash
python lingtu.py nav                    # Start with localization
# In REPL:
> map use office_2f                     # Activate map
> go 5 3                                # Navigate to (5, 3)
> go 找到餐桌                            # Semantic navigation
```

### 3. Dashboard

```bash
# SSH tunnel
ssh -L 5050:localhost:5050 sunrise@192.168.66.190
# Browser: http://localhost:5050
# Scene tab →click on map to send goals
```

## cmd_vel Priority Arbitration

All velocity sources go through CmdVelMux (L0):

| Source | Priority | Timeout |
|--------|----------|---------|
| Teleop (joystick) | 100 | 0.5s |
| VisualServo (tracking) | 80 | 0.5s |
| Recovery (stuck backup) | 60 | 0.5s |
| PathFollower (autonomy) | 40 | 0.5s |

Highest-priority active source wins. Sources timeout after 0.5s of silence.

## Source Layout

```
src/
|-- runtime/           Module, Blueprint, ports, registry, msgs, transport, TF
|-- drivers/           Real/sim robot and sensor backends
|-- slam/              SLAM/localization modules and portable SLAM code
|-- nav/               Navigation FSM, maps, safety, exploration, planner services
|-- nav/services/plan/ OctoPlanner3D default plus explicit PCT compatibility
|-- nav/local/         Local planner, terrain, path follower modules and path banks
|-- nav/kernel/        C++/nanobind local navigation kernel
|-- perception/        Detection, encoding, scene perception, reconstruction
|-- decision/          Semantic planner, goal resolver, LLM, visual servo
|-- memory/            Semantic, episodic, tagged, vector, temporal memories
|-- gateway/           REST/SSE/WS, MCP, teleop, media, runtime status
|-- kernels/           Portable compute kernel paths and ABI contracts
|-- lingtu/            Local package API and runtime handoff

web/                   React Dashboard (Vite)
config/                Robot/device/runtime configuration
calibration/           Camera/LiDAR/IMU calibration toolbox
scripts/               Build, deploy, ops, diagnostics, robot-side commands
tools/                 Developer validation, benchmark, offline analysis helpers
```

## Configuration

```yaml
# config/robot_config.yaml (key sections)
camera:
  rotate: 270           # Compensate sideways mounting (0/90/180/270)
  fx: 615.0             # Intrinsics (overridden by ROS2 CameraInfo)

lidar:
  offset_x: 0.0         # LiDAR-IMU extrinsics

occupancy_grid:
  resolution: 0.2       # Grid cell size (meters)

gateway:
  api_key: ""           # Set LINGTU_API_KEY env var to enable auth
```

### LLM backends

```bash
export MOONSHOT_API_KEY="..."           # Kimi (default, China-accessible)
export DASHSCOPE_API_KEY="..."          # Qwen (fallback)
export OPENAI_API_KEY="sk-..."          # OpenAI
export ANTHROPIC_API_KEY="sk-ant-..."   # Claude
```

## Deployment

### systemd (production)

```bash
# On S100P robot
sudo systemctl enable lingtu camera
sudo systemctl start lingtu
```

Services: `lingtu.service` (navigation), `camera.service` (Orbbec), `localization.service` (Fast-LIO2), `localizer.service` (ICP).

Super-LIO remains an experimental field-evaluation backend and is not the
default `nav` localization path. See
`docs/04-deployment/super_lio_backend.md` for its build, smoke, rollback, and
route-validation gate.

### Update

```bash
ssh sunrise@192.168.66.190
cd ~/data/inovxio/lingtu
git pull origin main
cd web && npm run build
sudo systemctl restart lingtu
```

## Testing

```bash
# Python framework tests (no ROS2 needed)
python -m pytest src/runtime/tests/ -q              # ~1300+ core framework tests

# Module-specific test suites (9 directories total)
python -m pytest src/gateway/tests/ -q           # Gateway route/runtime/auth tests
python -m pytest src/nav/tests/ -q               # Navigation FSM/patrol/geofence tests
python -m pytest src/localization/tests/ -q              # SLAM backend status/bridge tests
python -m pytest src/memory/tests/ -q            # Semantic/episodic/tagged tests

# C++ nav_kernel tests
cd src/nav/kernel && mkdir -p build && cd build
cmake .. && make -j$(nproc)
./test_benchmark                                  # 12 benchmarks

# C++ local planner tests
cd ../../services/plan/local_planner/cpp
cmake -B build -DLOCAL_PLANNER_CPP_BUILD_TESTS=ON
cmake --build build -j$(nproc)
./build/test_local_planner_core                   # 30 tests

# Full ROS2 build
source /opt/ros/humble/setup.bash
make build
```

## Sensor Calibration

Factory calibration toolbox in `calibration/`:

| Step | Tool | Time |
|------|------|------|
| Camera intrinsics | `calibration/camera/calibrate_intrinsic.py` | ~5 min |
| IMU noise | `calibration/imu/allan_variance_ros2/` | ~2 hr |
| LiDAR-IMU extrinsics | `calibration/lidar_imu/LiDAR_IMU_Init/` | ~2 min |
| Camera-LiDAR extrinsics | `calibration/camera_lidar/direct_visual_lidar_calibration/` | ~10 min |
| Apply all | `calibration/apply_calibration.py` →robot_config.yaml | instant |
| Verify | `calibration/verify.py` | instant |

## Version History

| Version | Date | Highlights |
|---------|------|------------|
| v2.3.0 | 2026-05-31 | Sprint 5-6: Module boundary enforcement, 9 test directories, directory restructuring (PascalCase→snake_case, flattening), ROS2 extraction, security hardening (CORS/shell-injection), mypy+ruff engineering standards, FrameContract monolith extraction, +52 new tests |
| v2.2.0 | 2026-04-13 | Dashboard Arc/Raycast →Frosted Glass redesign, DDS GIL fix, camera 3-level auto-recovery, FloatingWidget system |
| v2.1.0 | 2026-04-06 | Enterprise hardening, C++ nav_kernel SIMD optimization, 1226 tests |
| v1.0.0 | 2026-03 | Initial release |

See [CHANGELOG.md](CHANGELOG.md) for full details.

## License

MIT License. See [LICENSE](LICENSE).

---

<p align="center">
  <strong>穹沛科技 (Inovxio)</strong> —让机器人走进真实世界
</p>
