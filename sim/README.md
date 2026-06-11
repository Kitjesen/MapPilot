# LingTu Simulation Environment

> A hardware-free, full-stack simulation framework for quadruped robot navigation. Built on MuJoCo physics with ray-cast LiDAR, it enables end-to-end testing of SLAM, terrain-aware planning, semantic navigation, and person-following 鈥?all without a physical robot.

## 1. Overview

The simulation environment mirrors the real-world LingTu deployment stack while replacing hardware sensors and actuators with physics-accurate MuJoCo models. This enables rapid iteration on navigation algorithms, regression testing, and data collection for learning-based methods.

**Supported capabilities:**

- Full 6-DOF rigid body dynamics with contact
- Ray-cast LiDAR simulation (Livox Mid-360 pattern, 360掳 FoV)
- RGB-D camera rendering
- Person-following behavioral simulation with FSM control
- Semantic search and exploration
- Direct integration with both ROS2 and pure-Python LingTu stacks

## 2. System Architecture

The system has three layers:

**Physics** 鈥?MuJoCo loads a world XML + robot MJCF, runs the physics step each tick, and reads ray-cast LiDAR data from the sensor plugin.

**Bridge** 鈥?Three bridge options connect physics to navigation:

| Bridge | Path | Notes |
|--------|------|-------|
| `mujoco_ros2_bridge.py` | MuJoCo 鈫?ROS2 topics 鈫?C++ autonomy stack | Full ROS2, same as real robot |
| `nova_nav_bridge.py` | MuJoCo 鈫?Python LingTu modules directly | No ROS2 dependency, fastest iteration |
| `mujoco_viz_bridge.py` | MuJoCo 鈫?visualization only | Rendering, no navigation |

**Navigation** 鈥?Either the ROS2 C++ autonomy stack (terrain + local planner + path follower) or the pure-Python LingTu module stack (`python lingtu.py sim`).

Worlds, robots, and bridges are independent 鈥?any world can host any robot, and either bridge connects to navigation.

Default product runtime contract: `enable_native=False` keeps motion ownership
inside LingTu Modules. PCT/A* produce global plans in `NavigationModule`, then
`LocalPlannerModule` (`nanobind`) feeds `PathFollowerModule` (`nav_core`), which
publishes through `CmdVelMux` to the selected simulation driver. The ROS2 C++
`localPlanner`/`pathFollower` bridge remains a native gate/legacy experiment,
not the default product runtime.

### Current DimOS-Style Closure Status

The latest target-host DimOS benchmark run is still red, not a full readiness
claim. On the Linux ROS2/MuJoCo/PCT host,
`artifacts/server_sim_closure/host_preflight_after_large_loop_report_guard.json`
is green and the aggregate gap report
`artifacts/server_sim_closure/dimos_gap_after_large_loop_report_guard.json`
records 12 of 13 required gates passing. `pct_saved_map_navigation` now has real PCT
saved-map evidence with `selected_planner=pct`,
`pct_optimizer_enabled=false`, and
`pct_planner_path_mode=native_astar_raw_path`; this is not an A* fallback. The
remaining required blocker is `large_loop_closure`, so do not claim full
`PCT_planner + MuJoCo` DimOS closure until all 13 gates pass.

## 3. Scenes

Four pre-built environments are provided, each targeting different navigation challenges:

| Scene | File | Description | Terrain Generation |
|-------|------|-------------|-------------------|
| Open Field | `worlds/mujoco/open_field.xml` | Flat ground for basic validation | None |
| Spiral Terrain | `worlds/mujoco/spiral_terrain.xml` | 4-layer spiral ramp with elevation changes | `gen_terrain_mesh.py` |
| Building | `worlds/mujoco/building_scene.xml` | Multi-room indoor building with corridors | None |
| Factory | `worlds/mujoco/factory_scene.xml` | Warehouse layout with shelving and obstacles | None |

Additional composite scenes are kept in `worlds/mujoco/` alongside the base
environments (for example, `go2_room_nova.xml` places a Go2 in a furnished
room). `sim/worlds/mujoco/` is the canonical MuJoCo scene path used by profiles
and runtime contracts. Gazebo SDF scenes reside in `sim/worlds/gazebo/`.

## 4. LiDAR Simulation

Two approaches are supported for point cloud generation:

### 4.1 Ray-Cast Plugin (Default)

Uses the [mujoco_ray_caster](https://github.com/Albusgive/mujoco_ray_caster) C++ sensor plugin, which calls `mj_ray()` natively within the physics step. No GPU required. Outputs a dense point cloud directly from sensor data.

**Build instructions:**

```bash
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco/plugin
git clone https://github.com/Albusgive/mujoco_ray_caster.git
cd .. && mkdir build && cd build
cmake .. && cmake --build . -j8
export MUJOCO_PLUGIN_PATH=$(pwd)/bin/mujoco_plugin
```

### 4.2 OmniPerception (GPU, Large-Scale)

Based on [OmniPerception](https://github.com/aCodeDog/OmniPerception) (CoRL 2025). Uses Warp/CUDA GPU ray tracing with the exact Livox Mid-360 scanning pattern. Recommended for RL training and batch simulation where throughput matters.

```bash
pip install warp-lang[extras]
cd LidarSensor && pip install -e .
```

## 5. Person Following

The `following/` module implements a complete person-following pipeline for behavioral evaluation.

### 5.1 Architecture

The following system is structured as five independent layers:

| Layer | Module | Responsibility |
|-------|--------|---------------|
| Scene | `engine/` | Physics, robot spawning, person spawning |
| Person | `person/` | Simulated human movement (RoomAwareWalk, waypoint paths) |
| Perception | `perception/` | Simulated detection and tracking with configurable noise |
| Controller | `controller/` | Motion commands from perception input |
| Metrics | `metrics/` | Distance error, tracking loss rate, response latency |

### 5.2 Behavioral FSM

The `FollowingBehavior` state machine manages five states:

| State | Trigger In | Trigger Out |
|-------|-----------|-------------|
| **FOLLOW** | target re-acquired / target moves | target stopped 鈫?WAIT, target lost 鈫?SEARCH |
| **WAIT** | target stopped | target moves 鈫?FOLLOW |
| **SEARCH** | target lost | target found 鈫?FOLLOW, timeout 鈫?EXPLORE |
| **EXPLORE** | search timeout | target found 鈫?FOLLOW, timeout 鈫?RECOVER |
| **RECOVER** | explore timeout | (terminal) |

### 5.3 Controller Comparison

Four controllers were benchmarked on stop-walk-stop scenarios:

| Controller | Mean Distance Error | Tracking Loss Rate |
|-----------|--------------------|--------------------|
| PurePursuit | 0.23 m | 4.2% |
| **PurePursuit + Velocity Prediction** | **0.14 m** | **1.8%** |
| PID with Lookahead | 0.19 m | 3.1% |
| Predictive MPC | 0.16 m | 2.4% |

PurePursuit with velocity prediction achieves the best overall performance and is the default.

## 6. Quick Start

### 6.1 Prerequisites

```bash
pip install mujoco numpy scipy
bash sim/scripts/install_deps.sh        # optional: installs all deps
```

### 6.2 Basic Simulation (No ROS2)

```bash
# Legacy Go1 demos require optional sim/robots/go1_playground assets.
python sim/scripts/go1_indoor_nav.py    # legacy Go1 indoor demo
python lingtu.py sim                    # Full LingTu stack in simulation
```

### 6.2.1 Product Tasks On Simulation Endpoints

LingTu now keeps the task profile separate from the runtime connection layer.
Use these entries when validating the product stack instead of calling scattered
gate scripts directly:

**Simulation endpoint required:** product task profiles only count as
simulation when they are bound to an explicit simulation endpoint such as
`--endpoint mujoco_live`, `--endpoint gazebo`, or `--endpoint cmu_unity`. Do not treat bare `nav`, `map`, or `explore` as simulation; those profiles may
target the robot-side runtime depending on profile and host configuration.

```bash
python lingtu.py explore --endpoint mujoco_live        # MuJoCo raw MID-360 + Fast-LIO
python lingtu.py explore --endpoint gazebo --record    # Gazebo industrial demo + RViz
python lingtu.py tare_explore --endpoint cmu_unity     # CMU Unity external TARE adapter
```

The older `sim_mujoco_live`, `sim_industrial`, and `sim_cmu_tare` profiles stay
as compatibility aliases for CI/server gates.

### 6.2.2 Headless Navigation Smoke Mode

`MujocoDriverModule` supports two drive modes:

| Mode | Purpose |
|------|---------|
| `policy` | Default. Uses the ONNX gait policy and MuJoCo contacts. |
| `kinematic` | Applies `cmd_vel` directly to the floating base for deterministic headless navigation tests. |

Use kinematic mode when validating that LingTu planning, path following,
`CmdVelMux`, odometry, and exploration are wired correctly without requiring a
working gait checkpoint:

```bash
LINGTU_SIM_DRIVE_MODE=kinematic python lingtu.py sim
```

Kinematic mode proves the navigation stack can command route-level motion in
simulation. It is not evidence that the Thunder v3 RL gait policy is healthy;
policy-mode smoke tests should still be run before claiming gait-level fidelity.
Run the policy-mode smoke tests on a machine that has MuJoCo, ONNX Runtime, and
`policy_251119.onnx` available:

```bash
PYTHONPATH=src:. python -m pytest src/core/tests/test_sim_runtime_compat.py -q \
  -k "policy_cmd_vel or full_stack_policy_mode"

PYTHONPATH=src:. python sim/scripts/policy_nav_smoke.py \
  --world open_field \
  --direct-duration 4 \
  --goal-distance 0.8 \
  --nav-duration 14 \
  --nav-local-planner-backend nanobind \
  --nav-path-follower-backend nav_core \
  --nav-costmap-wait 3 \
  --nav-path-min-speed 0.12 \
  --nav-path-max-speed 0.35 \
  --nav-waypoint-threshold 0.30 \
  --nav-final-waypoint-threshold 0.25 \
  --nav-path-goal-tolerance 0.25 \
  --max-nav-dist-to-goal 0.30 \
  --min-nav-motion 0.35 \
  --nav-max-angular-z 0.1
```

For the full server-side closure report, run:

```bash
PYTHONPATH=src:. python sim/scripts/routecheck_preflight_gate.py \
  --map server_sim_demo \
  --goal-x 1.0 \
  --goal-y 0.0 \
  --goal-yaw 0.0 \
  --json-out artifacts/server_sim_closure/routecheck/summary.json \
  --strict

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py \
  --preset g4_server_full_sim \
  --required-only \
  --json-out artifacts/server_sim_closure_summary_g4_current.json \
  --strict
```

The summary is an evidence aggregator. The aggregator does not launch missing gates on its
own; run the command shown in each failed gate before treating the closure as
complete. A passing full summary must report `ok=true`,
`simulation_only=true`, `real_robot_motion=false`,
`cmd_vel_sent_to_hardware=false`, and `missing_or_failed=[]`.
When `--required` names only a subset, non-required failures are reported in
`optional_missing_or_failed` and `optional_gaps`; do not treat those as full
closure evidence.

Before running host-specific gates, use the preflight mode to check local
capabilities without launching any gate command:

```bash
PYTHONPATH=src:. python sim/scripts/server_sim_closure.py \
  --host-preflight \
  --preset g4_server_full_sim \
  --required-only \
  --json-out -
```

`--json-out -` prints only to stdout and does not create a report artifact.
When refreshing a preset from a workstation that cannot satisfy every gate
requirement, add `--skip-host-blocked` to `--run-missing`; host-blocked gates
are recorded in `gate_runs` and remain missing instead of being launched. This
is a diagnostic mode only. Do not use it for DimOS acceptance or to turn
host-blocked runtime gates into a readiness claim.

Current full closure gates:

| Gate | Required evidence |
| --- | --- |
| `gateway_runtime_acceptance` | Gateway runtime data plane, stage evidence, and non-motion command whitelist through ModulePort streams |
| `routecheck_preflight` | Gateway non-motion baseline/candidate route preflight with zero published goal/cmd_vel/stop |
| `blocked_route_replan_preflight` | Gateway non-motion blocked-route replanning preview with synthetic route obstruction and zero published motion commands |
| `navigation_replay_deviation` | Offline replay/deviation over routecheck-derived, JSON trace, or recorded topic JSONL global_path, local_path, cmd_vel, and odometry with no hardware output |
| `multifloor_exploration` | multi-floor matrix, frontier loop, LiDAR localization contract, native PCT, nanobind local planning, nav_core tracking |
| `large_terrain` | PCT/native planning on large terrain routes with path safety |
| `native_pct_mujoco` | native PCT route through ROS2 local planner/path follower into MuJoCo kinematic motion |
| `dynamic_obstacle_local_planner` | nanobind local planner replans around changing obstacles |
| `fastlio2_dynamic_inspection` | live MID-360/IMU through Fast-LIO2 into PCT/local planning with moving-obstacle video evidence |
| `moving_obstacle_sweep` | moving-obstacle inspection evidence across speed and density bins |
| `large_loop_closure` | long-range live Fast-LIO/PCT/local planner loop with bounded drift and return closure |
| `gazebo_runtime` | ROS-native Gazebo TF, odometry, point-cloud frame smoke, frontier progress, and post-pass no-gain/stall observation |
| `saved_map_relocalize` | saved-map relocalization contract for localizer navigation mode |
| `pct_saved_map_navigation` | saved-map PCT navigation after relocalization through localPlanner/pathFollower and MuJoCo motion |

Recorded navigation topic logs can be materialized directly into the replay
gate before aggregation:

```bash
PYTHONPATH=src:. python sim/scripts/server_sim_closure.py \
  --required navigation_replay_deviation \
  --required-only \
  --navigation-replay-deviation-topic-jsonl artifacts/recorded_topics.jsonl \
  --json-out artifacts/server_sim_closure/navigation_replay_deviation/closure_summary.json \
  --strict
```

This remains a non-motion DimOS-style replay/deviation check. It does not prove
the MuJoCo/PCT live closed loop by itself.

For the current DimOS-style closure sequence, use the target-host runner instead
of launching individual runtime gates by hand:

```bash
bash sim/scripts/run_dimos_linux_closure.sh --dry-run
bash sim/scripts/run_dimos_linux_closure.sh --execute --ros-domain-id <isolated_domain>
```

The runner is dry-run by default, refuses non-Linux `--execute`, runs read-only
host preflight before runtime gates, and stops when host preflight is red. The
generated gap report also checks the summary file age and requires dataflow rows
for every runtime gate before `lingtu_readiness.ok` can become true.

Latest target-host evidence from 2026-06-08 is still red overall. In the
isolated Linux workspace `/home/bsrl/hongsenpang/lingtu_dimos_20260608`, the
current sourced host preflight artifact
`artifacts/server_sim_closure/host_preflight_after_large_loop_report_guard.json`
reports `ok=true`, `blocked_gates=[]`, and all 13 DimOS required gates
runnable. The current strict summary
`artifacts/server_sim_closure/summary_after_large_loop_report_guard.json`
reports `ok=false`, and the current gap report
`artifacts/server_sim_closure/dimos_gap_after_large_loop_report_guard.json`
reports
`lingtu_readiness.ok=false`, `lingtu_readiness.claim_allowed=false`,
`host_preflight_ok=true`, `gap_counts.passed=12`, `gap_counts.failed=1`,
`runtime_dataflow_complete=true`, and `runtime_dataflow_ok=false`.

The remaining failed gate is `large_loop_closure`. `gazebo_runtime`,
`saved_map_relocalize`, and `pct_saved_map_navigation` have moved forward and
are now green in the latest target-host evidence. The saved-map PCT report
records `plan_preview.ok=true`, `selected_planner=pct`,
`pct_optimizer_enabled=false`, `pct_planner_path_mode=native_astar_raw_path`,
and `native_gate.ok=true` with `fallback_used=false`. This is real PCT
saved-map evidence, not an A* fallback. Do not claim full
`PCT_planner + MuJoCo + saved-map` DimOS readiness until `large_loop_closure`
also passes and the aggregate reaches 13/13. The latest source live run
`artifacts/server_sim_closure/mujoco_fastlio2_live/inspection-loop-video-20260608_164410/`
ended without writing a child `report.json` before the launcher fallback was
installed, so a red diagnostic report now records
`runtime_report_missing_after_launcher`. Current guards reject stale
`latest.txt` aggregation after failed live launches, require
`large_loop_closure` to evaluate only fresh reports, reject stale
current-summary payloads, reject stale nested child reports, and keep failed
runtime summaries red in the gap report.

Older target-host evidence from 2026-06-08 is kept below for diagnosis. The
environment blocker has moved forward. In the isolated Linux workspace
`/home/bsrl/hongsenpang/lingtu_dimos_20260608`, `local_planner` and Fast-LIO2
have been built/sourced. The current sourced host preflight artifact
`artifacts/server_sim_closure/host_preflight_dimos_benchmark_after_fastlio2_build_domain75.json`
reports `ok=true`, `failed_checks=[]`, `blocked_gates=[]`, and all 13 DimOS
required gates runnable. `ros2 pkg executables local_planner` lists both
`localPlanner` and `pathFollower`, `ros2 pkg executables fastlio2` lists
`fastlio2 lio_node`, and the hardware subscriber audit, PCT native runtime,
MuJoCo headless, and official MID-360 scan-pattern checks pass.

The PCT native runtime is loadable on that host, `large_terrain` is green for
native PCT discrete planning, and `native_pct_mujoco` now passes at
`artifacts/server_sim_closure/native_pct_mujoco/report_after_goal_reached_fix.json`.
That gate records `selected_planner=pct`, `fallback_used=false`,
`pct_native_backend_used=true`, `path_count=175`, `cmd_count_nonzero=484`,
`moved_m=3.4101`, `final_distance_m=0.4944`, and `reached_goal=true`. This is a
real PCT-to-ROS2-local-planner/path-follower-to-MuJoCo kinematic motion pass,
not an A* fallback.

The DimOS benchmark is still not green. A full target-host execute after the
Fast-LIO2 build produced
`artifacts/server_sim_closure/summary_after_fastlio2_build_execute.json` with
7/13 required gates passed. After the observed Python list/NumPy type faults
were fixed and the live Fast-LIO launcher was hardened, a clean
`ROS_DOMAIN_ID=76` inspection generated
`artifacts/server_sim_closure/mujoco_fastlio2_live/inspection-moving-obstacle-video-20260608_064916/report.json`.
That run proves the live chain is partially connected: Fast-LIO publishes
registered/map clouds and odometry, Navigation receives `/nav/map_cloud`,
`/nav/registered_cloud`, and `/nav/odometry`, PCT emits one global path, the
local planner emits local paths, and `/nav/cmd_vel` has nonzero samples. It is
still red because Fast-LIO motion diverges from MuJoCo motion, the inspection
reaches `0/3` checkpoints, moving-obstacle points are missing/empty, and
`body_to_camera` frame evidence is missing.

The historical domain76 refreshed summary and gap report are
`artifacts/server_sim_closure/summary_after_fastlio2_cleanup_domain76_refreshed.json`
and
`artifacts/server_sim_closure/dimos_gap_after_fastlio2_cleanup_domain76_refreshed.json`.
They showed 7/13 required gates passed with `lingtu_readiness.ok=false` and
`claim_allowed=false`. At that stage, remaining red or missing gates were
`fastlio2_dynamic_inspection`, `moving_obstacle_sweep`,
`large_loop_closure`, `gazebo_runtime`, `saved_map_relocalize`, and
`pct_saved_map_navigation`. Until those are green, this repository must not
claim full `PCT_planner + MuJoCo + Fast-LIO/saved-map` DimOS readiness.

Older fresh target-host evidence after the MuJoCo live-gate hardening was
stricter and still red. With `ROS_DOMAIN_ID=83`, the host preflight artifact
`artifacts/server_sim_closure/host_preflight_after_motion_window_domain83.json`
reports `ok=true`, `failed_checks=[]`, `blocked_gates=[]`, and all 13 DimOS
required gates runnable. The newest focused inspection run is
`artifacts/server_sim_closure/mujoco_fastlio2_live_motion_window/inspection-moving-obstacle-video-20260608_074940/report.json`.
It proves useful connectivity (`runtime_evidence.ok=true`, moving obstacles
published 81 updates with 288 points max, `global_planner=pct`, 2 global paths,
98 local paths, nonzero `/nav/cmd_vel`, and 374 video frames), but it is not a
pass: `ok=false`, Fast-LIO motion is `3.278m` while MuJoCo motion is `1.459m`
(`motion_scale_ratio=2.2468`), angular saturation ratio is `0.5822`, and the
inspection reaches only `1/3` required checkpoints. The fresh summary and gap
artifacts are:

- `artifacts/server_sim_closure/dimos_benchmark_after_motion_window.json`
- `artifacts/server_sim_closure/dimos_gap_after_motion_window.json`
- `artifacts/server_sim_closure/dimos_gap_after_motion_window.md`

Those domain83 reports used a 7200s max-age window, marked older reports stale,
and therefore showed `lingtu_readiness.ok=false`, `claim_allowed=false`, and `0/13`
required gates passed in that freshness window. This is intentional: stale green artifacts must
not be used to claim DimOS readiness.

For server bootstrap verification, `scripts/deploy/setup_server_ros_pct.sh`
runs the setup-safe subset it generates itself: the multi-floor closure gate,
the Gateway routecheck preflight gate, and a strict summary at
`artifacts/server_sim_closure_summary_setup.json`. Disable the routecheck
preflight only with `LINGTU_RUN_ROUTECHECK_PREFLIGHT=0`.

### 6.2.2 Full Simulation Validation Gate

Run the server-side validation gate before claiming simulation readiness. It is
simulation-only: it does not connect to robot services, publish real robot
commands, or manage systemd units.

```bash
PYTHONPATH=src:. python sim/scripts/full_sim_validation.py \
  --run-mujoco \
  --require-all \
  --nav-duration 10 \
  --json-out artifacts/full_sim_validation_mujoco.json
```

The gate records JSON evidence for required worlds, the multi-floor building
scene, Fast-LIO2 localization metric contracts, global/local planning wiring,
frontier exploration, person-tracking behavior, MuJoCo LiDAR point clouds, and
kinematic full-stack navigation motion through path follower and `CmdVelMux`.
Use `--require-all` when blocked checks should fail the run instead of being
reported as incomplete evidence.

Policy-mode gait validation is opt-in because it requires a compatible ONNX
checkpoint:

```bash
PYTHONPATH=src:. python sim/scripts/full_sim_validation.py \
  --run-mujoco \
  --run-policy \
  --policy-path /path/to/compatible_policy.onnx \
  --require-all \
  --json-out artifacts/full_sim_validation_policy.json
```

If the checkpoint is missing or incompatible, the policy check is reported as
blocked or failed explicitly; it is not folded into a kinematic navigation pass.
The report records the ONNX input/output shapes and SHA-256 hash, so policy
results remain attributable to an exact checkpoint.

#### Policy-Mode Soak On Sunrise

Use `sunrise` only as a simulation compute host for this check. Do not start
robot services, publish real `/nav/cmd_vel`, send Gateway goals, or run
`scripts/lingtu nav`, `scripts/lingtu map`, `ros2 topic pub`, or
`systemctl restart robot-*` during the soak.

```bash
ssh sunrise@192.168.66.190
cd ~/data/SLAM/navigation
mkdir -p artifacts
export PYTHONPATH=src:.
export LINGTU_SIM_DRIVE_MODE=policy
export PYTHONIOENCODING=utf-8

python sim/scripts/policy_nav_smoke.py \
  --world open_field \
  --direct-duration 4 \
  --goal-distance 0.8 \
  --nav-duration 14 \
  --nav-local-planner-backend nanobind \
  --nav-path-follower-backend nav_core \
  --nav-costmap-wait 3 \
  --nav-path-min-speed 0.12 \
  --nav-path-max-speed 0.35 \
  --nav-waypoint-threshold 0.30 \
  --nav-final-waypoint-threshold 0.25 \
  --nav-path-goal-tolerance 0.25 \
  --max-nav-dist-to-goal 0.30 \
  --min-nav-motion 0.35 \
  --nav-max-angular-z 0.1 \
  --json-out artifacts/policy_nav_smoke_open_field.json
```

Treat skipped policy tests as `BLOCKED`, not `PASS`. The pass gate is: policy
loads, all poses stay finite, direct policy motion exceeds `0.20 m`, body height
stays inside `0.35 m < z < 0.55 m`, roll/pitch stay below the script gate,
full-stack nav emits costmap, waypoint, local path, path-follower command and
mux command, `direct_fallback == 0`, and full-stack policy motion exceeds
`0.35 m`.

Common failures:

| Symptom | Likely Cause | First Checks |
| --- | --- | --- |
| policy test skipped | MuJoCo, `onnxruntime`, or checkpoint missing | Check `_POLICY_CANDIDATES` and the sunrise brainstem checkout. |
| `UnsupportedPolicyInputError` | Wrong checkpoint contract | Do not pad or truncate; wire the original observation builder first. |
| little forward motion | Direction/action scale, standing pose, or joint order mismatch | Check `PolicyRunner.build_obs`, `ACTION_SCALE`, `MJ_TO_DART`, and `DART_TO_MJ`. |
| low body height or high roll/pitch | Bad warm-up, contact, actuator mapping, or incompatible policy | Check MJCF actuator-to-joint resolution and policy warm-up. |
| no costmap | LiDAR scene/body/group problem | Run the MuJoCo LiDAR smoke and inspect `get_lidar_points()`. |
| no waypoint/local path/mux command | Navigation wiring, not gait | Compare the kinematic full-stack test first. |
| `direct_fallback > 0` | Planner or adapter path generation failed | Treat as a full-stack failure, not a gait pass. |

Policy mode currently follows the brainstem `StandardObservationBuilder`
contract: 57 values per frame, optionally stacked as `57 * N` history frames.
The preferred navigation gait checkpoint is brainstem's default
`model/policy_251119.onnx`; place it at
`sim/robots/nova_dog/model/policy_251119.onnx`, repo-root
`model/policy_251119.onnx`, or a sibling `brainstem` checkout using the same
relative `model/policy_251119.onnx` path. If that file is unavailable,
`sim/robots/nova_dog/policy.onnx` is the verified development fallback. Its
metadata is recorded in `sim/robots/nova_dog/policy_manifest.json`; the current
server-verified asset is a single-frame `57`-input policy with SHA256
`c672253ffb89ae4f0c766615e7028a9a676572c77fc1741474e552eae55b2672`.
Newer Thunder checkpoints with non-57-multiple inputs, such as 76-D
RobotLab-style exports, need their original training observation field order,
normalization, and action semantics wired into a dedicated observation builder
before running. The runner intentionally refuses to pad or truncate those inputs
because that produces misleading "policy runs but robot will not walk" failures.

### 6.2.2 Multi-Floor LiDAR Localization / Planning Validation

Use this gate for hardware-free validation of the navigation dataflow in a
rich two-floor scene. It generates a MuJoCo XML scene, a PCT-compatible
tomogram, and a synthetic map cloud, then validates:

- synthetic scan-to-map localization health (`LOCKED` samples in `map` frame)
- native PCT availability, ABI/lib path, tomogram hash, and route evidence
- PCT floor-graph composition with a stairs transition edge
- A* same-floor fallback planning
- `global_path -> local_path -> cmd_vel -> CmdVelMux` dataflow in memory only
- continuous odometry replay through the native `nav_core` path follower
- optional MuJoCo bridge-loop validation:
  `GlobalPath + WaypointTracker -> LocalPlanner -> PathFollower(nav_core) -> CmdVelMux -> MujocoDriver`
- Wavefront frontier exploration candidate generation

The script does not start Gateway, ROS services, robot services, or any real
driver. It must keep `cmd_vel_sent_to_hardware=false`.

This is not a real robot, gait policy, ROS2 SLAM, or physical stairs gate.
Reports must include `validation_level=kinematic_module_ports`,
`physical_gait_verified=false`, `slam_verified=false`,
`real_lidar_verified=false`, and `cross_floor_physical_verified=false`.
`--bridge-loop` uses kinematic MuJoCo motion unless a separate policy-mode gate
is run.

```bash
PYTHONPATH=src:. python sim/scripts/multifloor_nav_validation.py \
  --output-dir artifacts/multifloor_matrix \
  --route matrix \
  --planners pct,astar \
  --strict \
  --json-out artifacts/multifloor_matrix/report.json
```

To validate simulated motion through the module ports, add `--bridge-loop`.
This moves only the MuJoCo robot. It still must report
`real_robot_motion=false` and `cmd_vel_sent_to_hardware=false`.
Within `multifloor_nav_validation.py`, the default local planner backend is
`simple`; use
`--local-planner-backend nanobind --require-production-local-planner` when the
native local planner has been built and the report must fail on simple fallback.

```bash
PYTHONPATH=src:. python sim/scripts/multifloor_nav_validation.py \
  --output-dir artifacts/multifloor_matrix_bridge \
  --route matrix \
  --planners pct,astar \
  --local-planner-backend nanobind \
  --require-production-local-planner \
  --bridge-loop \
  --strict \
  --json-out artifacts/multifloor_matrix_bridge/report.json
```

For focused debugging, run a single route:

```bash
PYTHONPATH=src:. python sim/scripts/multifloor_nav_validation.py \
  --output-dir artifacts/multifloor_cross_floor \
  --route cross_floor \
  --planners pct,astar \
  --strict \
  --json-out artifacts/multifloor_cross_floor/report.json

PYTHONPATH=src:. python sim/scripts/multifloor_nav_validation.py \
  --output-dir artifacts/multifloor_same_floor \
  --route same_floor \
  --planners pct,astar \
  --strict \
  --json-out artifacts/multifloor_same_floor/report.json
```

For shorter bridge-loop debugging, limit the matrix:

```bash
PYTHONPATH=src:. python sim/scripts/multifloor_nav_validation.py \
  --output-dir artifacts/multifloor_lower_bridge \
  --route matrix \
  --matrix-routes same_floor,lower_approach \
  --planners astar \
  --bridge-loop \
  --local-planner-backend nanobind \
  --require-production-local-planner \
  --strict \
  --json-out artifacts/multifloor_lower_bridge/report.json
```

Expected result for a strict `pct,astar` run on a host with native PCT
available:

- `passed=true`
- `validation_level=kinematic_module_ports`
- `physical_gait_verified=false`
- `slam_verified=false`
- `real_lidar_verified=false`
- `cross_floor_physical_verified=false`
- `mujoco_scene_load.ok=true`
- `lidar_localization.ok=true`
- `native_pct_gate.ok=true`
- `native_pct_gate.runtime_ok=true`
- `native_pct_gate.tomogram_sha256` is populated
- `local_planner_backend_verified=nanobind` when production local planning is required
- `production_local_planner_verified=true` when production local planning is required
- `planning[pct].feasible=true`
- `planning[pct].status=pass`
- `planning[pct].backend_available=true`
- `planning[pct].native_backend_used=true`
- `planning[pct].path_validation.ok=true`
- `command_flow.ok=true`
- `command_flow.cmd_vel_sent_to_hardware=false`
- `tracking_replay.ok=true`
- `tracking_replay.backend_actual=nav_core`
- `tracking_replay.cmd_vel_sent_to_hardware=false`
- when `--bridge-loop` is enabled:
  - `mujoco_bridge_loop.ok=true`
  - every segment has `reached_goal=true`
  - every segment has `nonzero_linear_xy_cmd_count>0`
  - every segment has `max_linear_xy>0.02`
  - every segment has `moved_m>0.05`
  - every segment reports `path_follower_backend=nav_core`
  - production local-planner segments report `local_path_trim_start_dist_m=0.2`
  - `mujoco_bridge_loop.cmd_vel_sent_to_hardware=false`
- `exploration.ok=true`

The bridge loop intentionally fails if the modules only exchange messages but
all velocity commands stay zero. This catches dense or degenerate local paths
that are not actually trackable by `PathFollowerModule(backend="nav_core")`.
`LocalPlannerModule(backend="nanobind")` also filters untrackable placeholder
paths before publishing, so a one-point `{0,0,0}` recovery/stop result clears
the local path instead of masquerading as a valid plan.

For the cross-floor route, A* is expected to fail because it has no floor graph
or vertical transition model. PCT is required, but this script validates
cross-floor as floor-graph composition: per-floor native PCT segments plus an
explicit stairs transition edge. It does not prove that native PCT solved one
single 3D multi-floor problem, and it does not prove physical stair climbing.
The cross-floor PCT result must report `native_backend_used=true`,
`transition_validation.ok=true`, `native_pct_single_plan_verified=false`, and
`transition_motion_verified=false`.

If native PCT cannot load on the current host or the tomogram is missing, the
report must use `planning[pct].status=blocked`,
`planning[pct].blocked=true`, and `native_pct_gate.blocked=true`. That is not a
pass; it is explicit evidence that this machine cannot validate the production
PCT route yet.

If `tracking_replay.backend_actual` falls back to `pid`, build the native
backend first:

```bash
bash scripts/build_nav_core.sh
```

#### Native PCT + ROS2 Local Planner + MuJoCo Showcase Gate

Use this gate when a demo needs stronger evidence than the in-process
`nanobind` module loop. It reuses a multifloor validation report that already
contains native PCT output, starts the ROS2 native `localPlanner` and
`pathFollower` executables, feeds them PCT waypoints, and applies the native
`/cmd_vel` stream to a kinematic MuJoCo robot.

The script is still simulation-only. It does not start Gateway, CmdVelMux,
robot drivers, robot services, or hardware bridges. Before launching
`pathFollower`, it checks the selected ROS domain for existing `/cmd_vel` or
`/nav/cmd_vel` subscribers and refuses to run unless the domain is isolated.
When `--video-out` is used on a headless server, the script defaults
`MUJOCO_GL=egl` for offscreen rendering unless that environment variable is
already set.

Obstacle checking is enabled by default. The gate publishes scene metadata
obstacles to the native local planner and applies a conservative waypoint
safety filter before handing PCT output to the path follower. This prevents a
demo from passing only because it tracked a path through a pillar. Use
`--disable-obstacle-check` or `--disable-waypoint-safety-filter` only for
debugging, and do not report those runs as obstacle-avoidance evidence.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH=src:.

python sim/scripts/native_pct_mujoco_gate.py \
  --generate-source-report \
  --route same_floor \
  --ros-domain-id 93 \
  --strict \
  --json-out artifacts/native_pct_mujoco_gate/report.json \
  --video-out artifacts/native_pct_mujoco_gate/navigation.mp4
```

For an App/Web-style evidence video, use the four-panel layout. It records the
front camera RGB panel, robot-frame local planner view, MuJoCo observer view,
and live point-cloud/map view in one MP4. The report also captures sampled
native `/path` messages in `local_path_samples`, so the video can distinguish
global route display from real local-planner output.

For algorithm demos, prefer `--video-layout scene_overlay --sim-vehicle
omni_cart`. It keeps the same native PCT and ROS2 `localPlanner` output, but
draws the global preview, native local `/path`, actual trail, and LiDAR points
directly into one MuJoCo observer scene. Use the quadruped visualization only
when gait, contact, or body-yaw behavior is the thing being evaluated.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
export PYTHONPATH=src:.

python sim/scripts/native_pct_mujoco_gate.py \
  --source-report artifacts/large_terrain_nav_validation_pct_ros_v10/report.json \
  --route terrain_long \
  --planner pct \
  --artifact-dir artifacts/native_pct_evidence_video \
  --json-out artifacts/native_pct_evidence_video/report.json \
  --video-out artifacts/native_pct_evidence_video/navigation_evidence.mp4 \
  --video-layout scene_overlay \
  --sim-vehicle omni_cart \
  --video-fps 1 \
  --video-width 1920 \
  --video-height 1080 \
  --camera-fovy 78 \
  --front-path-lookahead-m 16 \
  --scene-overlay-point-limit 500 \
  --n-rays 64 \
  --lidar-sample-count 256 \
  --timeout-s 260 \
  --sample-limit 260 \
  --trajectory-sample-stride 5 \
  --max-speed 0.45 \
  --max-lateral-speed 0.45 \
  --omni-lookahead-m 1.3 \
  --omni-min-speed 0.14 \
  --omni-yaw-gain 0.35 \
  --omni-max-yaw-rate 0.16 \
  --omni-yaw-deadband 0.16 \
  --ros-domain-id 198 \
  --strict
```

Expected proof fields:

- `ok=true`
- `pct_native_backend_used=true`
- `pct_runtime_ok=true`
- `video.image_source="MuJoCo observer renderer with 3D scene overlays"` for scene overlay videos
- `video.lidar_source.kind="MuJoCo ray-cast simulated Livox MID-360 style scan"`
- `sim_vehicle=omni_cart` for algorithm showcase videos
- `drive_adapter=omni_local_path_tracker` for algorithm showcase videos
- `backend.local_planner=cmu_ros2_native/localPlanner`
- `backend.path_follower=cmu_ros2_native/pathFollower`
- `backend.sim_driver=MuJoCoEngine(kinematic)`
- `reached_goal=true`
- `path_count>0`
- `local_path_samples` is non-empty for evidence videos
- `cmd_count_nonzero>0`
- `moved_m` exceeds the configured `--min-motion-m`
- `obstacle_aware.enabled=true`
- `obstacle_clearance.collision=false`
- `real_robot_motion=false`
- `cmd_vel_sent_to_hardware=false`
- `video.exists=true` when `--video-out` is provided

This is the right gate for showing that global planning, ROS2 native local
planning, native path following, and simulated robot motion are connected. It
still does not prove real Fast-LIO2 localization, policy gait stability,
real-device latency, safety mux delivery to a robot driver, or long-duration
field robustness.

#### Large Terrain Planning Asset Gate

Use this gate before recording large-field motion videos. It builds a
24m-class synthetic terrain with boundary walls, boulders, a central narrow
gate, rough/high-cost zones, a slope band, and a no-go ditch. The gate validates
global planning assets only: no ROS2 nodes, Gateway, driver, CmdVelMux, or real
robot command path is started.

```bash
PYTHONPATH=src:. python sim/scripts/large_terrain_nav_validation.py \
  --output-dir artifacts/large_terrain_nav_validation \
  --json-out artifacts/large_terrain_nav_validation/report.json \
  --strict
```

Expected proof fields:

- `ok=true`
- `validation_level=global_planning_assets`
- `simulation_only=true`
- `real_robot_motion=false`
- `cmd_vel_sent_to_hardware=false`
- each case has `planning[0].feasible=true`
- each case has `path_safety.ok=true`
- gate routes have `gate_crossing.passed_gate=true`

Current routes are `terrain_short`, `terrain_long`, `terrain_narrow_gap`, and
`terrain_slope_bypass`. The next stronger gate is to feed this report into the
native ROS2 local-planner + MuJoCo video loop and require
`obstacle_clearance.collision=false`.

### 6.3 Full Stack with ROS2

This is a legacy ROS launch / smoke contract. The current server-side closure
uses the G4 gates above; keep this launch path stable for compatibility, but do
not treat its historical `run_global_planner.py` reference as the current
planner validation entrypoint. Run it only in an isolated `ROS_DOMAIN_ID` with
no robot bridge, robot driver, or hardware command subscriber present. See
`sim/launch/README.md` before using this path.

```bash
source /opt/ros/humble/setup.bash
ros2 launch sim/launch/sim.launch.py world:=building_scene
```

### 6.4 Send Navigation Goals

```bash
# Via REPL
python lingtu.py sim
> go 5 3
> go 鎵惧埌椁愭

# Via ROS2
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}}}"
```

The ROS2 `/goal_pose` example is for isolated simulation domains only. Export
an isolated `ROS_DOMAIN_ID` first and verify there is no hardware subscriber on
`/goal_pose`, `/cmd_vel`, or `/nav/cmd_vel` before publishing. Do not run this
example on the robot ROS domain.

### 6.5 Person Following Benchmark

```bash
python sim/scripts/benchmark_following.py
# Results saved to sim/output/benchmark/
```

## 7. Robots

| Robot | Directory | Control | Notes |
|-------|-----------|---------|-------|
| Unitree Go2 | `robots/go2/` | RL policy (48D obs, action_scale=0.5) | MuJoCo Playground compatible |
| Thunder v3 | `assets/{urdf,xml,mjcf}/` | LingTu MuJoCo adapter | Current upstream CAD asset baseline |
| Thunder v3 compatibility | `robots/nova_dog/robot_with_camera.xml` | LingTu MuJoCo adapter | Legacy path now mirrors the current Thunder v3 runtime XML |
| Thunder v3 URDF compatibility | `robots/thunder.urdf` | URDF | Compatibility path with mesh references adjusted to `assets/meshes/` |

## 8. Datasets

Offline LiDAR/IMU datasets for algorithm development and testing:

| Dataset | Source | Use Case |
|---------|--------|----------|
| `datasets/Avia/` | Livox Avia | LiDAR-inertial odometry testing |
| `datasets/legkilo*/` | Legged robot | Kinematic-inertial-LiDAR fusion |

### 8.1 ROS2 Bag Replay Gates

There are four distinct replay levels. Do not report the lower level as proof
of the higher level:

| Gate | Script | Proves | Does not prove |
|------|--------|--------|----------------|
| Raw CDR bridge replay | `sim/scripts/rosbag_slam_bridge_replay.py` | Real rosbag2 CDR messages can enter `SlamBridgeModule` and produce localization health | Fast-LIO2/Super-LIO/localizer algorithm output |
| Fast-LIO2 algorithm replay | `sim/scripts/fastlio2_rosbag_replay_gate.py` | Real `/imu_raw + /points_raw` rosbag replay through `fastlio2/lio_node`, then `/Odometry + /cloud_registered` into `SlamBridgeModule` | Saved-map relocalization |
| MuJoCo live Fast-LIO2 gate | `python lingtu.py sim_mujoco_live gate` | Live MuJoCo LiDAR/IMU publishes `/points_raw + /imu_raw`, Fast-LIO2 publishes `/Odometry + /cloud_map`, and canonical `/nav/*` topics are present | Real robot gait, real MID-360 driver timing, saved-map relocalization |
| MuJoCo live PCT closed-loop gate | `python lingtu.py sim_mujoco_pct_live` | Same MuJoCo/Fast-LIO source path, then native PCT route, local planner/path follower, `/nav/cmd_vel`, moving obstacle, and MP4 evidence | Real robot gait, real MID-360 driver timing, saved-map relocalization |
| Localizer replay | localizer node with `/Odometry + /cloud_registered` and a static map | ICP localizer contract against a map | BBS3D global relocalization unless `libcpu_bbs3d` is installed |

Server evidence from the 2026-05-07 validation run:

```bash
# Raw rosbag2 CDR -> SlamBridge
source /opt/ros/humble/setup.bash
PYTHONPATH=src:.:$PYTHONPATH python3 sim/scripts/rosbag_slam_bridge_replay.py \
  --bag artifacts/rosbag_replay_subset/grass_raw_odom_cloud_tiny \
  --odom-topic /state_SDK \
  --cloud-topic /points_raw \
  --json-out artifacts/rosbag_replay_subset/grass_raw_bridge_report.json \
  --strict

# Fast-LIO2 node -> SlamBridge
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/.third_party/livox_sdk2/lib:$LD_LIBRARY_PATH"
PYTHONPATH=src:.:$PYTHONPATH python3 sim/scripts/fastlio2_rosbag_replay_gate.py \
  --bag artifacts/rosbag_replay_subset/grass_fastlio2_aligned_window \
  --imu-topic /imu_raw \
  --lidar-topic /points_raw \
  --playback-rate 1.0 \
  --json-out artifacts/fastlio2_rosbag_replay/grass_window_final_tracking/report.json \
  --strict

# MuJoCo live LiDAR/IMU -> Fast-LIO2 -> canonical /nav/*.
python lingtu.py sim_mujoco_live gate

# Same source path plus LingTu frontier/navigation driving /nav/cmd_vel.
python lingtu.py sim_mujoco_live explore

# Long-running live review: starts exploration and opens RViz with cloud,
# map, path, odometry, and TF displays.
python lingtu.py sim_mujoco_live demo

# Same as explore, with MP4 evidence.
python lingtu.py sim_mujoco_live video

# PCT closed loop with moving-obstacle and MP4 evidence.
python lingtu.py sim_mujoco_pct_live
```

Expected Fast-LIO2 strict evidence:

- `ok=true`
- `real_rosbag_replay_verified=true`
- `slam_algorithm_output_verified=true`
- `bridge_verified=true`
- `outputs.fastlio2_odometry > 0`
- `outputs.fastlio2_cloud_registered > 0`
- `final_bridge_status.state=TRACKING`

Expected MuJoCo live strict evidence:

- `ok=true`
- `simulation_only=true`
- `real_robot_motion=false`
- `live_mujoco_lidar_verified=true`
- `live_mujoco_imu_verified=true`
- `slam_algorithm_output_verified=true`
- `outputs.fastlio2_odometry > 0`
- `outputs.fastlio2_cloud_map > 0`
- `final_bridge_status.state=TRACKING`

MuJoCo live LiDAR topic inventory:

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| `/points_raw` | `livox_ros_driver2/msg/CustomMsg` by default, or `sensor_msgs/msg/PointCloud2` with `--fastlio-lidar-input timed_pointcloud2` | `body` | Raw MID-360-pattern LiDAR input to Fast-LIO2 |
| `/imu_raw` | `sensor_msgs/msg/Imu` | `body` | Raw simulated IMU input to Fast-LIO2 |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | `body` | Fast-LIO2 registered scan output |
| `/cloud_map` | `sensor_msgs/msg/PointCloud2` | `odom` | Fast-LIO2 local map output |
| `/Odometry` | `nav_msgs/msg/Odometry` | `odom -> body` | Fast-LIO2 localization output |
| `/nav/registered_cloud` | `sensor_msgs/msg/PointCloud2` | `body` | LingTu canonical current cloud |
| `/nav/map_cloud` | `sensor_msgs/msg/PointCloud2` | `odom` | LingTu canonical map cloud |
| `/nav/odometry` | `nav_msgs/msg/Odometry` | `odom -> body` | LingTu canonical odometry |
| `/nav/exploration_grid` | `nav_msgs/msg/OccupancyGrid` | `odom` | Occupancy map used by exploration |
| `/nav/global_path` | `nav_msgs/msg/Path` | `odom` | LingTu global path for RViz and gates |
| `/nav/local_path` | `nav_msgs/msg/Path` | `odom` | LingTu local path for RViz and gates |
| `/nav/cmd_vel` | `geometry_msgs/msg/TwistStamped` | `body` | LingTu navigation command fed back to MuJoCo only |

For large MuJoCo scenes, the live gate injects a temporary `<size
memory="64M"/>` into the scene before the engine merges the world with the
robot XML. `MuJoCoEngine.load()` preserves that size block during merge so rich
factory scenes do not fail with a constraint-stack overflow. The original scene
XML is not modified.

The LEG-KILO bag has storage timestamps and message header timestamps that are
not aligned at the beginning of the file. Fast-LIO2 synchronizes on message
header time, so use the extracted aligned window rather than the first raw
messages in the bag.

## 9. ROS2 Interface

| Topic | Message Type | Direction | Rate |
|-------|-------------|-----------|------|
| `/mujoco/pos_w_pointcloud` | `sensor_msgs/PointCloud2` | Sim 鈫?Nav | 10 Hz |
| `/nav/odometry` | `nav_msgs/Odometry` | Sim 鈫?Nav | 50 Hz |
| `/nav/cmd_vel` | `geometry_msgs/TwistStamped` | Nav 鈫?Sim | 50 Hz |
| TF (`map鈫抩dom鈫抌ody`) | `tf2_msgs/TFMessage` | Sim 鈫?Nav | 50 Hz |

## 10. Directory Reference

| Directory | Contents |
|-----------|----------|
| `engine/` | Simulation engine framework (physics loop, MuJoCo wrappers, scenario management) |
| `worlds/` | Canonical MuJoCo XML and Gazebo SDF world/scene definitions |
| `robots/` | Compatibility robot model and policy paths for older scripts |
| `assets/` | Canonical Thunder v3 robot assets, meshes, MJCF/URDF, and Livox assets |
| `sensors/` | Standalone sensor simulators and fallbacks |
| `bridge/` | Physics 鈫?navigation bridges (ROS2, direct Python, visualization) |
| `following/` | Person-following simulation (FSM, controllers, perception, metrics) |
| `semantic/` | Legacy semantic simulation test residue |
| `datasets/` | Offline LiDAR/IMU datasets |
| `scripts/` | Stable launcher, gate, validation, demo, and benchmark script contract |
| `validation/` | Full-system validation package used by scripts and tests |
| `evaluation/` | Offline SLAM evaluation manifests and dataset tooling |
| `external_scenes/` | Optional external/license-constrained scene placeholders |
| `meshes/` | Legacy mesh path; keep until references are fully audited |
| `launch/` | Legacy ROS launch / smoke contract files |
| `maps/` | Reserved empty placeholder for future simulation map fixtures |
| `output/` | Local generated outputs; reproducible evidence should prefer `artifacts/` |
| `configs/` | Reserved empty placeholder for future simulation configuration fixtures |

## 11. Dependencies

| Package | Version | Required |
|---------|---------|----------|
| MuJoCo | >= 3.0 | Yes |
| Python | >= 3.10 | Yes |
| NumPy | >= 1.24 | Yes |
| SciPy | >= 1.10 | Yes |
| ROS2 Humble | latest | Optional (for ROS2 bridge) |
| warp-lang | >= 0.10 | Optional (GPU LiDAR) |
