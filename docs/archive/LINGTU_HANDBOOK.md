---
title: "LingTu System Handbook"
subtitle: "Autonomous navigation full stack for outdoor quadruped robots"
---

# LingTu System Handbook

> **Audience**: engineers who need a full-system mental model + field operators who drive the robot.
> **Reading order on first pass**: sections 1-3 (overview) and section 19 (field ops). The rest is reference.
> **Version**: v1.9, 2026-04-25.

---

## Part I 鈥?System Overview

### 1. What LingTu Is

LingTu is the autonomous-navigation full stack for quadruped robots in outdoor / off-road environments.

Positioning:
- Not a SLAM library, not a planner library 鈥?it is the complete pipeline from sensors to motors.
- Not a teleop tool 鈥?it is an autonomous decision system. Give it a goal and it plans, drives, avoids and recovers on its own.
- Not a simulation toy 鈥?it runs on real hardware (S100P / RDK X5).

Target hardware:
- Compute: S100P (Nash BPU 128 TOPS, aarch64, Ubuntu 22.04 + ROS 2 Humble)
- LiDAR: Livox MID-360 (10 Hz, 360 deg x 59 deg)
- Camera: Orbbec Gemini 335 (color + depth, mounted vertically 鈥?see camera.rotate)
- IMU: Livox built-in fused at 200 Hz
- GNSS (optional): WTRTK-980 RTK + NTRIP RTCM injection
- Motion control: Brainstem quadruped board over gRPC (port 13145)

Design principles:
1. Module-First 鈥?Module is the only runtime unit, Blueprint is the only assembly unit.
2. Composition over inheritance 鈥?nine stack factory functions assemble the full system.
3. Pluggable backends via Registry 鈥?algorithm selection is `@register("category", "name")`, no `if/else` branching.
4. C++ owns the hot paths 鈥?Python orchestrates, C++ does the heavy lifting through nanobind.
5. Raw DDS instead of rclpy 鈥?production avoids the ROS 2 Python runtime; rclpy is only a fallback.

### 2. Layer Architecture

```
L6  Interface     Gateway (HTTP / WS / SSE) | MCP server | Teleop
-----------------------------------------------------------------
L5  Planning      NavigationModule | GlobalPlannerService | WaypointTracker
-----------------------------------------------------------------
L4  Decision      SemanticPlanner | LLM | VisualServo | AgentLoop
-----------------------------------------------------------------
L3  Perception    Detector (BPU) | Encoder (CLIP) | Reconstruction
                  SemanticMapper | Episodic | Tagged | VectorMemory
-----------------------------------------------------------------
L2  Maps          OccupancyGrid | ESDF | ElevationMap | Terrain
                  LocalPlanner | PathFollower | TraversabilityCost
-----------------------------------------------------------------
L1  Hardware      ThunderDriver | CameraBridge | SLAM (Fast-LIO2 / Localizer)
                  GNSS (GnssModule + GnssBridge + NtripClient)
-----------------------------------------------------------------
L0  Safety        SafetyRing | GeofenceManager | CmdVelMux
```

Higher layers depend on lower layers; lower layers never depend on higher ones. Cross-layer communication goes through Module ports, never via direct imports.

### 3. Core Concepts

| Concept | Meaning | Code entry |
|---|---|---|
| Module | Runtime unit with In / Out ports + lifecycle (`setup` / `start` / `stop`) | `core.module.Module` |
| Port | Typed message channel with backpressure policy | `core.stream.In[T]`, `Out[T]` |
| Blueprint | DSL for assembling modules 鈥?supports `autoconnect`, explicit `wire`, multiple transports | `core.blueprint.Blueprint` |
| Stack | Pre-assembled group of related modules (driver / slam / maps / ...) | `core.blueprints.stacks.*` |
| Registry | Plugin system via `@register("slam", "fastlio2")` | `core.registry` |
| NativeModule | C++ subprocess wrapper with watchdog + SIGTERM | `core.native_module` |
| Profile | Launch preset; CLI is `python lingtu.py <profile>` | `cli/profiles_data.py` |
| Session | Whole-system state machine (idle / mapping / navigating); Gateway is single source of truth | `gateway_module.py` |

Backpressure policies (per `Out` port):

| Policy | Behaviour | Use case |
|---|---|---|
| `all` | broadcast everything (default) | low-rate signals |
| `latest` | keep only newest, drop old | high-rate sensors |
| `throttle(dt)` | rate-limit | downsample |
| `sample(n)` | one of every N | decimate |
| `buffer(size)` | batch | batched processing |

### 4. Profiles (`cli/profiles_data.py`)

| Profile | Purpose |
|---|---|
| `stub` | Framework-only, no hardware |
| `dev` | Semantic-pipeline development on a laptop |
| `map` | Mapping mode, `slam_profile=fastlio2`, semantics off |
| `nav` | **Production navigation**, `slam_profile=bridge`, planner = PCT |
| `explore` | Wavefront frontier exploration |
| `tare_explore` | CMU TARE hierarchical exploration |
| `sim` | MuJoCo full-stack simulation |
| `sim_nav` | Pure-Python nav simulation, no ROS 2 / no C++ |

Robot presets (`ROBOT_PRESETS` in the same file): `stub`, `sim`, `ros2`, `s100p`, `navigate`, `thunder`. The `nav` profile defaults to `s100p` and uses `slam_profile=bridge` because the SLAM stack is owned by an external `slam.service` (systemd) on the robot.

---

## Part II 鈥?Sensing and Localization

### 5. Sensors

| Device | Topic | Rate | Consumer |
|---|---|---|---|
| Livox MID-360 | `/nav/lidar_scan`, `/nav/imu` | 10 / 200 Hz | Fast-LIO2 |
| Orbbec Gemini 335 | `/camera/{color,depth}/image_raw`, `/camera/camera_info` | ~7 Hz default, 30 Hz tunable | CameraBridgeModule |
| GNSS WTRTK-980 | `/gnss/fix` (NMEA serial -> DDS) | 10 Hz | GnssModule |
| IMU (Livox) | `/nav/imu` | 200 Hz | Fast-LIO2 |

CameraBridge has two subscriber back ends (`src/compat/ros2/camera_bridge.py`):
- DDS-first via raw cyclonedds 鈥?runs without a full ROS 2 environment, this is the production path.
- rclpy fallback 鈥?used when ROS 2 Python is available; adds watchdog + USB reset + L1/L2/L3 self-recovery.

The Thunder hardware package does not own ROS bridge code; camera bridge adapters stay in `compat/ros2` and are composed explicitly by the product stack when camera input is enabled.

The camera is mounted vertically on the body, so `config/robot_config.yaml::camera.rotate=270`. CameraBridge applies `cv2.rotate` in `_on_ros2_color`; downstream modules always see "upright" frames.

### 6. SLAM and Localization

Four SLAM modes selectable via `slam_profile`:

| Mode | `slam_profile` | What it does | When |
|---|---|---|---|
| Mapping | `fastlio2` | Fast-LIO2 builds a fresh map | `lt session map`, profile `map` |
| Localization | `localizer` | Fast-LIO2 + ICP localizer aligned to a saved map | profile `s100p` |
| Bridge | `bridge` | Subscribe to topics from external `slam.service` | profile `nav` (production default) |
| Off | `none` | No SLAM | `stub` / `dev` |

TF chain:

```
map --(Localizer ICP)--> odom --(Fast-LIO2)--> body --(static calibration)--> lidar / camera
```

- The localizer publishes `map -> odom` only after ICP converges.
- Fast-LIO2 publishes `odom -> body` continuously.
- `body -> sensors` is published statically from calibration results.

`SlamBridgeModule` subscribes to `/nav/odometry`, `/nav/map_cloud`, `/nav/saved_map_cloud`, `/localization_quality`, `/tf`, then exposes the `map -> odom` transform on `map_odom_tf: Out[dict]` so the Gateway can transform `map_cloud` into the saved map's frame before pushing it to the browser.

Drift watchdog (Gateway background thread `_drift_watchdog_loop`):
- Symptom: long static dwell (hours) drives the Fast-LIO2 IEKF covariance to infinity, odometry reports values like `(-500000, 200000)`.
- Watchdog: every 60 s checks odometry; on threshold breach calls `svc.stop("slam","slam_pgo","localizer")`, clears the cached odom, emits SSE event `slam_drift`, restores services with `svc.ensure(...)` for the active session mode, then sits in a 300 s cool-down.
- Tunable env: `LINGTU_DRIFT_WATCHDOG=0` to disable; `_INTERVAL`, `_XY_LIMIT`, `_V_LIMIT`, `_COOLDOWN` for thresholds.

Localization-quality semantics: float, **lower is better**. `0.0` ideal, `0.1-0.3` normal, `>0.3` warning, `-1.0` ICP failed.

### 7. Map Layers

Five representations coexist, each with a single purpose:

| Map | Type | Purpose | Producer |
|---|---|---|---|
| PCD | 3D point cloud (PCL) | Raw SLAM output, ICP reference | `Fast-LIO2 map save` |
| OccupancyGrid | 2D `uint8` grid | 2D A*, RVIZ display | `OccupancyGridModule` (point-cloud projection) |
| ElevationMap | 2D `float` grid | Slope / step analysis | `ElevationMapModule` |
| ESDF | 2D signed-distance field | LocalPlanner cost | `ESDFModule` |
| Tomogram | Multi-slice height layers | **PCT global planner input** | `_build_tomogram` from PCD |

`MapManagerModule.save` runs the full pipeline:
1. Fast-LIO2 emits `map.pcd` + `poses.txt`.
2. `_build_tomogram(pcd)` -> `tomogram.pickle`.
3. DUFOMap offline filter (Phase 2) 鈥?ray-casting + void detection on `<map>/patches/*.pcd`. Writes a clean `map.pcd`, backs the original up as `map.pcd.predufo`. Gated by `LINGTU_SAVE_DYNAMIC_FILTER=1`. Binary at `~/src/dufomap/build/dufomap_run`, config `config/dufomap.toml`.
4. `_build_occupancy_snapshot` -> `occupancy.npz` + `map.pgm` + `map.yaml` (nav2-compatible).

Default storage: `~/data/lingtu/maps/<name>/` (legacy `~/data/nova/maps/` still honoured if it exists, see `cli/profiles_data.py::_default_map_dir`).

`TraversabilityCostModule` (L2 fusion) consumes `OccupancyGrid + ElevationMap + ESDF + Terrain`, emits `fused_cost` to NavigationModule and `slope_grid` to Gateway.

---

## Part III 鈥?Planning and Control

### 8. Global Planner (PCT)

`pct_planner` from HKU/HKUST (GPLv2). Paper: "Efficient Trajectory Planning for Off-Road" (arXiv 2310.07780).

Artifacts (aarch64 only 鈥?x86 dev boxes auto-fall-back to `_AStarBackend`):
- `ele_planner.so` 鈥?3D A* on a hex grid with multi-slice height handling.
- `traj_opt.so` 鈥?GPMP (Gaussian-process motion planning) for jerk-minimal smoothing.
- `a_star.so`, `libele_planner_lib.so`.

Steps in `planner_wrapper.py::TomogramPlanner.plan()`:
1. World -> grid index (`pos2idx`).
2. Height -> slice index (`pos2slice`) 鈥?handles stairs / multi-storey.
3. C++ `ele_planner.so` runs 3D A* with traversability cost and slice-switch penalty.
4. C++ `traj_opt.so` smooths with GPMP.
5. Grid -> world.

Output is `np.ndarray (N, 3)` including z. Performance on S100P, 30 m playground: 80-300 ms / plan.

`global_planner_service.py::_find_safe_goal` does a BFS off the user-supplied goal to the nearest free cell when the goal lands inside an obstacle; if BFS fails, `mission_status` becomes `FAILED`.

### 9. Local Planner (CMU pre-sampled)

Source: CMU `base_autonomy` (TARE team). Core: `src/nav/core/include/nav_core/local_planner_core.hpp`.

Strategy: pre-generate ~1000 candidate paths offline, pick the best per frame online. **No online MPC.**

- Library: `src/base_autonomy/local_planner/paths/*.ply` 鈥?7 groups, ~150 paths each, MATLAB-generated arcs of 1-2 m.
- Online (10 Hz):
  1. Rotate the library by robot yaw across 36 directions (OpenMP parallel).
  2. For each path's voxels, check terrain map for collision -> mark blocked.
  3. `scorePath` = `(1 - sqrt(sqrt(heading_error))) * rotation_cost^2 * traversability_weight`.
  4. Aggregate by group -> pick best group -> pick best path.
  5. Emit `local_path` (20-30 points).

Acceleration ledger:

| Technique | Effect |
|---|---|
| SoA + CSR sparse | cache-friendly, eliminates stride-4 access |
| `scorePathFast` LUT (pow025 lookup) | **2.08x** |
| OpenMP across 36 rotations | uses all aarch64 cores |
| xsimd NEON / AVX | batched rotate + distance |
| LTO + `-ffast-math` | cross-function inlining + relaxed FP |

Frame time: 3-8 ms.

### 10. Path Follower (Pure Pursuit)

Source: CMU base_autonomy adapted for quadrupeds. Core: `src/nav/core/include/nav_core/path_follower_core.hpp`.

```
L = clamp(v * k_lookahead, L_min, L_max)        # adaptive lookahead
P = find_lookahead_point(local_path, L)
alpha = heading_to_P - robot_yaw
kappa = 2 * sin(alpha) / L                      # pure-pursuit curvature
cmd.linear.x  = v_target                        # 0.3-0.8 m/s conservative
cmd.angular.z = kappa * v_target
```

Quadruped tweaks: `no_rot_at_goal=true` (no in-place spin at the goal), acceleration limiter to suppress jitter from `v_target` jumps. Frame time: < 1 ms.

### 11. CmdVelMux (priority arbitration)

Every cmd_vel source goes through `CmdVelMux`; nothing reaches the driver directly:

| Source | Priority | Timeout |
|---|---|---|
| Teleop (joystick / WS) | **100** | 0.5 s |
| VisualServo | 80 | 0.5 s |
| Navigation recovery (back-up) | 60 | 0.5 s |
| PathFollower (cruise) | 40 | 0.5 s |

The highest-priority source whose last publish is within timeout wins. When the active source goes silent, the mux falls through to the next lower priority. With nothing active, output is zero. Operator joystick input automatically pre-empts PathFollower.

### 12. Mission FSM

NavigationModule splits a long global path into waypoints (WaypointTracker) and feeds the current target to the local planner each frame.

```
IDLE --goto--> PLANNING --ok--> EXECUTING --arrived--> ARRIVED
  ^               |                  |                    |
  |             fail                stuck                  |
  +-- cancel -- FAILED <----- RECOVERING <-can't recover---+
```

Stuck detection: 2 s of < 0.1 m motion -> recovery (back up 0.3 s + replan). Three consecutive recovery failures -> `FAILED`.

---

## Part IV 鈥?Semantics and Intelligence

### 13. SemanticPlannerModule 鈥?five-level goal resolver

User says "go to the kitchen" or "find the red chair". `goal_resolver.py` runs:

```
1. Tag lookup        exact / fuzzy match in TaggedLocationStore   -> goal_pose
2. Fast Path         scene-graph keyword + CLIP match (<200 ms)  -> goal_pose
3. Vector memory     CLIP embedding + ChromaDB search             -> goal_pose
4. Frontier          information-gain exploration on topo graph   -> goal_pose
5. Visual servo      VLM bbox detection + PD tracking             -> goal_pose / cmd_vel
```

Fast / Slow dual process:
- Fast Path (System 1, ~0.17 ms): scene-graph keywords + CLIP score + spatial reasoning. Fusion weights `label 0.35 + CLIP 0.35 + detector 0.15 + spatial 0.15`, threshold 0.75.
- Slow Path (System 2, ~2 s): LLM reasoning with ESCA selective grounding (200 objects -> ~15 -> token reduction ~92.5%).
- AdaNav escalation: when candidate-score Shannon entropy > 1.5 and confidence < 0.85, force the Slow Path even if Fast Path produced a hit.
- LERa recovery: 3-step Look-Explain-Replan on subgoal failure; after the second consecutive failure the LLM picks `retry_different_path | expand_search | requery_goal | abort`.

### 14. VisualServoModule 鈥?visual servo + person follow

Two output channels selected by distance:
- Far (> 3 m): emit `goal_pose` -> NavigationModule -> normal planning.
- Near (< 3 m): emit `cmd_vel` -> CmdVelMux (priority 80) -> bypass planner, PD-track directly.

Components: `BBoxNavigator` (bbox + depth -> 3D -> PD), `PersonTracker` (VLM select + CLIP Re-ID for occlusion robustness), `vlm_bbox_query` (open-vocab detection 鈥?Grounding-DINO or YoloE).

### 15. Memory Layers

| Module | Stores | Use |
|---|---|---|
| SemanticMapperModule | SceneGraph -> RoomObjectKG + TopologySemGraph; auto-saves every 30 s | scene-level object knowledge |
| EpisodicMemoryModule | Robot events | rollback / reports |
| TaggedLocationsModule | User-named places ("kitchen", "charger") | top-priority goal resolution |
| VectorMemoryModule | CLIP embeddings | fuzzy semantic search |
| TopologicalMemory | Topology nodes + connectivity | frontier exploration |

Storage: `data/memory/{global,sites,robots,missions}/`. Markdown is the long-term truth; the vector store is a search-acceleration cache only.

### 16. AgentLoop 鈥?multi-step LLM tool use

`src/semantic/planner/.../agent_loop.py` runs an observe -> think -> act loop with seven LLM tools:

`navigate_to(x, y)`, `navigate_to_object(label)`, `detect_object(query)`, `query_memory(text)`, `tag_location(name)`, `say(text)`, `done(summary)`.

Limits: 10 steps / 120 s. Supports OpenAI function-calling, with a JSON-text fallback for providers that lack it.

LLM back ends registered through `@register("llm", ...)`:

| Name | Env var |
|---|---|
| `kimi` | `MOONSHOT_API_KEY` (China-direct) |
| `openai` | `OPENAI_API_KEY` |
| `claude` | `ANTHROPIC_API_KEY` |
| `qwen` | `DASHSCOPE_API_KEY` (China fallback) |
| `mock` | none 鈥?keyword path |

---

## Part V 鈥?Safety and Interfaces

### 17. Safety

`SafetyRingModule` (L0):
- Inputs: obstacle map, safety distance, dialogue state, current local path.
- Output: `stop_cmd` (0 = clear, 1 = soft slow-down, 2 = hard estop).
- Hard stop: any obstacle within 0.3 m forward and speed > 0.2 m/s.
- Soft stop: obstacle within 0.8 m forward.
- Dialogue overrides: user "continue" releases soft stop; "stop" upgrades to hard stop.

`GeofenceManagerModule`: reads `config/geofence.yaml` (circles or polygons), publishes a boundary point cloud as a hard obstacle to LocalPlanner; crossing the fence triggers safety stop.

E-stop fan-out (in `full_stack.py`):

```python
bp.wire("SafetyRingModule", "stop_cmd", driver_name, "stop_signal")
bp.wire("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")
```

### 18. Gateway / Dashboard / MCP

`GatewayModule` (L6, `src/gateway/gateway_module.py`) 鈥?FastAPI + uvicorn on port 5050.

- HTTP: ~30 endpoints under `/api/v1/*`.
- SSE: `/api/v1/events` streams `odometry`, `mission`, `scene_graph`, `map_cloud`, `slam_drift`, ...
- WebSocket: `/ws/teleop` (joystick + JPEG video), `/ws/cloud` (binary point cloud), `/ws/webrtc/*` (video).
- Static hosting of the dashboard (`web/dist/`).

Key endpoints:
- `GET  /api/v1/session` 鈥?current mode + map + ICP quality.
- `POST /api/v1/session/start {mode, map_name}` 鈥?start mode (auto-launches slam / localizer services).
- `POST /api/v1/session/end` 鈥?return to idle.
- `POST /api/v1/goal {x, y, z?}` 鈥?send a navigation goal.
- `POST /api/v1/slam/relocalize {map_name, x, y, yaw}` 鈥?manual relocalization.
- `POST /api/v1/cmd/stop` 鈥?emergency stop.
- `POST /api/v1/bag/start {duration, prefix}` 鈥?start rosbag recording.
- `POST /api/v1/webrtc/bitrate {bps}` 鈥?adjust video bitrate.
- `GET  /api/v1/webrtc/stats` 鈥?video stats.
- `POST /api/v1/map_cloud/reset` 鈥?clear the browser-side cumulative point cloud.

`MCPServerModule` (`src/gateway/mcp_server.py`): JSON-RPC at `:8090/mcp`, auto-discovers `@skill` methods. 16 tools across navigation, perception, memory, semantic map, visual servo, planning, system. Connect from Claude Code with `claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp`.

Pose persistence: a successful `/slam/relocalize` writes `~/.lingtu/last_nav_pose.json` atomically. The next `session/start navigating` background worker calls `relocalize` 2.5 s later with the persisted pose, so power-cycles do not require another shift+click.

### 19. Teleop and Bag Recording

`TeleopModule` (`src/drivers/teleop_module.py`):
- Subscribes to `color_image`, encodes JPEG at 30 fps, pushes to `/ws/teleop`.
- Receives joystick frames `{"type": "joy", "lx": 0.5, "ly": 0.0, "az": -0.3}` from the same WebSocket.
- 3 s idle auto-release.
- Emits cmd_vel to CmdVelMux (priority 100).

Bag recording (`scripts/record_bag.sh`): `ros2 bag record` over 13 topics (no raw camera 鈥?payloads explode) into `~/data/bags/<prefix>_<ts>/`:

```
/nav/{lidar_scan, imu, odometry, map_cloud, registered_cloud, cmd_vel, goal_pose}
/localization_quality
/exploration/{way_point, path, runtime, finish}
/camera/camera_info
```

Video path comparison:

| Path | Latency | Bitrate | CPU | Status |
|---|---|---|---|---|
| JPEG over WS (`/ws/teleop`) | ~200 ms | ~4 Mbps | ~30% of one core | Production |
| WebRTC + libx264 H.264 | ~100 ms target | ~1.5 Mbps | ~30% of one core | Bitrate API works; aarch64 libx264 encoder stalls |
| Phase B: `hobot_codec` BPU hardware encode | < 50 ms | ~1.5 Mbps | < 5% | Roadmap |

JPEG is currently sufficient; WebRTC waits on hardware encode.

---

## Part VI 鈥?Operations and Field

### 20. S100P Layout

```
~/data/SLAM/navigation/                  -> ~/data/inovxio/lingtu/  (symlink)
~/data/inovxio/lingtu/                   source root
  鈹溾攢鈹€ lingtu.py                          CLI entry
  鈹溾攢鈹€ cli/profiles_data.py               profile + robot presets
  鈹溾攢鈹€ src/                               Python source
  鈹溾攢鈹€ install/                           colcon build (ROS 2 packages)
  鈹斺攢鈹€ web/dist/                          dashboard static assets
/tmp/lingtu_nav.log                      tmux runtime log
~/data/lingtu/maps/                      maps (legacy ~/data/nova/maps still honoured)
  鈹斺攢鈹€ <map_name>/
       鈹溾攢鈹€ map.pcd                       SLAM point cloud
       鈹溾攢鈹€ tomogram.pickle               PCT input
       鈹溾攢鈹€ map.pgm + map.yaml            nav2-compatible
       鈹斺攢鈹€ poses.txt                     PGO output
~/data/bags/                             rosbag storage
~/.lingtu/last_nav_pose.json             relocalization persistence
~/src/dufomap/build/dufomap_run          DUFOMap offline filter binary
/etc/systemd/system/                     slam.service / localizer.service / camera.service
/usr/local/bin/lt                        field CLI
/opt/ros/humble/                         ROS 2 Humble
```

systemd services on the robot:
- `slam.service` 鈥?Fast-LIO2 + Livox driver
- `localizer.service` 鈥?ICP localizer (loads saved map)
- `camera.service` 鈥?Orbbec launch
- `lingtu.service` 鈥?currently unused; field operations launch via tmux

Production launch (stable tmux):

```bash
tmux kill-session -t lingtu 2>/dev/null
tmux new-session -d -s lingtu -x 200 -y 50 \
  'cd ~/data/SLAM/navigation && python3 lingtu.py nav --llm mock --no-repl 2>&1 | tee /tmp/lingtu_nav.log'
```

Firewall: `iptables ROBOT_REMOTE` chain DROPs by default 鈥?open `5050` and `8090` after `lt restart`.
DDS: production uses raw cyclonedds (no rclpy). Local Windows dev requires extra DDS XML.

### 21. Field Operations

5-second health check:

```bash
ssh sunrise@192.168.66.190
lt
```

Read the output:

| Field | Green | Yellow | Red |
|---|---|---|---|
| mode | navigating / mapping | idle | empty |
| icp_quality | 0 < x < 0.3 | 0.3-0.5 | 0 / -1 |
| localizer_ready | true | 鈥?| false |
| /nav/odometry | 8-10 Hz | 3-8 | < 3 / empty |
| /localization_quality | ~10 Hz | 鈥?| empty |

Anything red sends you to section 23.

Standard navigation flow (90% of field tasks):

```bash
# Step 1 鈥?confirm mode
lt                             # check mode
lt session nav                 # idle -> nav (loads last-used map)
lt session nav my_custom_map   # specify map

# Step 2 鈥?relocalize (give ICP a sane initial pose)
lt reloc 0 0 0                 # at map origin, facing east
lt reloc 2.5 -1.3 1.57         # at (2.5, -1.3) facing north
# Wait ~3 s for icp_quality < 0.3 to confirm.

# Step 3 鈥?send goal
lt nav 5 3                     # go to (5, 3)

# Step 4 鈥?monitor or stop
lt                             # mission status
lt stop                        # estop, cmd_vel zeroed

# Step 5 鈥?record a bag for failure repro
lt record start
lt record
lt record stop
```

Mapping flow:

```bash
lt session end
lt session map
# Walk / drive / teleop the robot through the area.
lt map save my_lab_20260425
lt session end
lt map use my_lab_20260425
lt session nav my_lab_20260425
```

Obstacle avoidance is automatic 鈥?no toggle. As long as session is `navigating`, ICP quality is good and a goal is set, LocalPlanner re-scores 1000 candidates per frame, Terrain emits traversability, and CmdVelMux selects the final velocity.

### 22. Operations CLI (`scripts/lingtu`)

Aliases (recommended on the developer machine):

```bash
alias lingtu='ssh sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu"'
alias lingwatch='ssh -t sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu watch"'
```

Commands:

| Subcommand | Purpose |
|---|---|
| `lingtu status` | One-screen status across 8 sections (session / SLAM / robot / mission / path / control / map / log) |
| `lingtu watch` | `watch -c -n 1` continuous refresh 鈥?keep on a side monitor while mapping or navigating |
| `lingtu map start \| save <name> \| end \| list` | Mapping session lifecycle |
| `lingtu nav start <map> \| stop \| goal X Y [YAW]` | Navigation session + send goal |
| `lingtu svc status \| restart [slam\|lingtu\|all]` | systemctl wrapper |
| `lingtu log drift \| dufomap \| error \| tail \| all` | journalctl filter |
| `lingtu health` | dump `/api/v1/health` raw |

### 23. Troubleshooting

| Symptom | Diagnosis | Fix |
|---|---|---|
| Dashboard does not open | `ping 192.168.66.190` fails | Check Wi-Fi / robot power |
| `lt` returns "connection refused" | `ps -ef \| grep lingtu` empty | `lt restart` |
| `icp_quality = -1` or `0` | ICP cannot align from `(0,0,0)` | `lt reloc X Y YAW` (estimate true pose) |
| Position shows `(-5000, 20000)`-ish | Fast-LIO2 static drift | `sudo systemctl restart slam` then `lt reloc` |
| Camera view black | `lt cam`; if `lsusb \| grep Bootloader` | **Physically replug the Orbbec USB** |
| Plan fails | `lt log` for the cause (goal in obstacle?) | Pick another goal |
| Map cloud and saved map misaligned | Localizer not ready | Reloc first |
| `lt` command missing | Not installed | `scp lt -> /usr/local/bin/lt && chmod +x` |

Field gotchas worth remembering:
- Orbbec `color_fps:=30` locks the bootloader 鈥?never pass it to `gemini_330_series.launch.py`; you will physically need to replug.
- rclpy + uvicorn + aiortc together break thread scheduling 鈥?stay on the DDS fallback.
- `iptables ROBOT_REMOTE` DROPs `5050` / `8090` by default 鈥?open both after every deploy / reboot.
- SSH command lines with semicolons return exit 255 鈥?wrap in a heredoc script.

### 24. Calibration

Calibration toolbox: `calibration/` (sits at the repo root, not under `src/`). Covers all S100P sensors. Final values land in `config/robot_config.yaml` (single source of truth).

SOP:

| Step | Item | Tool | Time |
|---|---|---|---|
| 1 | Camera intrinsics (9x6 checkerboard) | `calibration/camera/calibrate_intrinsic.py` | ~5 min |
| 2 | IMU noise (Allan Variance) | `calibration/imu/allan_variance_ros2/` | 2-3 hr |
| 3 | LiDAR-IMU extrinsics (figure-eight motion) | `calibration/lidar_imu/LiDAR_IMU_Init/` | ~2 min |
| 4 | Camera-LiDAR extrinsics (target-less) | `calibration/camera_lidar/direct_visual_lidar_calibration/` | ~10 min |
| 5 | One-shot apply | `calibration/apply_calibration.py` | seconds |
| 6 | One-shot verify | `calibration/verify.py` | seconds |

Runtime check: `src/core/utils/calibration_check.py` runs at `full_stack_blueprint` startup. FAIL-level errors (zero focal length, non-orthogonal rotation) abort the launch; WARN-level (e.g. all-zero distortion coefficients) only logs.

---

## Part VII 鈥?Reference

### 25. Performance on S100P (aarch64)

| Stage | Latency |
|---|---|
| PCT global plan (30 m site) | 80-300 ms |
| LocalPlanner frame (1000 paths x 36 rotations) | 3-8 ms |
| PathFollower frame | < 1 ms |
| CmdVelMux arbitration | < 0.1 ms |
| Fast Path goal resolve | 0.17 ms |
| Slow Path (LLM) goal resolve | ~2 s |
| JPEG encode (720p) | 30-35 ms |
| CLIP encode (1 frame) | 45 ms (BPU) / 150 ms (CPU) |
| **Steady-state goal -> cmd_vel** | **< 15 ms** |
| **Cold start goal -> motion** | **~300 ms** |

### 26. Known Limitations and Roadmap

Red 鈥?production blocking:

| # | Issue | Status |
|---|---|---|
| R1 | Fast-LIO2 static drift > 1 h | **Fixing** (C++ IEKF covariance ceiling) |
| R2 | WebRTC libx264 encoder stalls on aarch64 | Roadmap (`hobot_codec` BPU hardware encode) |

Yellow 鈥?field UX:

| # | Issue | Status |
|---|---|---|
| Y1 | Camera defaults to ~7 fps | No stable 30 fps `color_fps=0` parameter combination found |
| Y2 | PCT only on aarch64 | Accepted 鈥?A* fallback |
| Y3 | ChromaDB optional | Accepted 鈥?VectorMemory soft-fails to numpy |
| Y4 | All 1226 framework tests are mock-based | Roadmap 鈥?real ROS 2 integration tests |

Green 鈥?optimisation:

| # | Issue | Status |
|---|---|---|
| G1 | `gateway_module.py` exceeds 2700 lines, tightly coupled | Roadmap 鈥?split Session / SSE / HTTP |
| G2 | No monitoring / alerting | Roadmap 鈥?Prometheus + WeChat Work |

### 27. CLI / HTTP / FAQ

`lt` commands (after SSH):

```
lt                      One-shot system snapshot
lt nav X Y              Send navigation goal
lt reloc X Y YAW        Relocalize (yaw in radians)
lt stop                 Emergency stop

lt session nav [MAP]    Switch to navigation mode
lt session map          Switch to mapping mode
lt session end          End session

lt map list             List maps
lt map save <name>      Save
lt map use <name>       Activate

lt record start | stop  Bag recording
lt log | lt logf        View logs
lt restart              Restart LingTu
lt cam                  Restart camera
```

Key paths cheat sheet:

```
~/data/SLAM/navigation/                   LingTu source (symlink)
/tmp/lingtu_nav.log                       Runtime log
~/data/lingtu/maps/                       Maps
~/data/bags/                              Rosbag
~/.lingtu/last_nav_pose.json              Relocalization persistence
/etc/systemd/system/{slam,localizer,camera}.service
/usr/local/bin/lt                         Field CLI
config/robot_config.yaml                  Robot parameters (calibration + speed + safety + ...)
cli/profiles_data.py                      Profile and robot-preset definitions
```

FAQ:

**Q: Why not nav2 directly?**
A: nav2 is heavy; quadrupeds don't need its behaviour-tree machinery; PCT's 3D terrain-aware planning has no nav2 equivalent; we want Module-First, not nav2 lifecycle nodes. OccupancyGrid is still nav2-compatible (`map.pgm` + `map.yaml`) 鈥?maps remain interoperable.

**Q: Can the robot still be remotely controlled while mapping?**
A: Yes. While in mapping mode, `lt nav X Y` (PCT plans through already-mapped area) and the joystick both work. Manual driving usually produces a better map.

**Q: Does it work offline?**
A: Yes. Use `--llm mock` 鈥?Fast Path is enough for the keyword cases. Only the multi-step Slow Path needs cloud LLM.

**Q: Can I run `lingtu.py` on a Windows laptop?**
A: Yes (profiles `dev` or `sim`). PCT auto-falls back to A*, CameraBridge runs in stub mode, SLAM is `none`. Good for code work and framework tests; not real navigation.

**Q: Does the robot lose its position on power loss?**
A: No 鈥?`~/.lingtu/last_nav_pose.json` persists the last successful relocalization. The next `session/start navigating` background worker restores it automatically.

**Q: Do the dashboard and `lt` conflict?**
A: No. Both call the same backend API. Browser for the picture, `lt` for commands is the common combo.

---

## Acknowledgements

- **pct_planner**: HKU/HKUST Bowen Yang and Jie Cheng, GPLv2.
- **base_autonomy** (LocalPlanner / PathFollower / Terrain): CMU Chao Cao team, TARE.
- **Fast-LIO2**: HKU-Mars Wei Xu.
- **Orbbec SDK**: Orbbec.
- **Livox ROS 2 driver**: Livox open source.

---

*This handbook is the single source of truth for LingTu. Subordinate documents under `docs/*` cover specific subjects; keep this document in sync when behaviour changes.*
