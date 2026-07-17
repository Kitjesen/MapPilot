# LingTu Simulation

`sim/` contains the hardware-free validation surface for LingTu. The primary
runtime is MuJoCo with simulated Thunder/Go2-style robot assets, ray-cast
LiDAR, simulated IMU, cameras, map gates, and navigation smoke tests.

Simulation can validate software wiring and algorithm contracts. It does not
prove field readiness by itself. Claims about S100P/sunrise deployment require
field runtime evidence.

## Stable Root Contract

| Path | Purpose |
| --- | --- |
| `engine/` | Importable simulation engine framework and MuJoCo wrappers |
| `worlds/` | Canonical MuJoCo XML and Gazebo SDF scenes |
| `assets/` | Thunder assets, meshes, MJCF/URDF, LiDAR scan-pattern assets |
| `robots/` | Robot model and policy assets used by simulation entrypoints |
| `sensors/` | Standalone sensor simulators and lightweight fallbacks |
| `bridge/` | Physics-to-navigation bridges and visualization adapters |
| `planning/` | Simulation planning probes and route fixtures |
| `following/` | Person-following simulation controllers and metrics |
| `datasets/` | Offline LiDAR/IMU datasets and metadata |
| `evaluation/` | Offline SLAM/navigation evaluation helpers |
| `scripts/` | Public launchers, gates, demos, and benchmark entrypoints |
| `validation/` | Importable full-system validation package |
| `tests/` | Simulation integration and filesystem contract tests |
| `launch/` | Legacy ROS launch/smoke compatibility files |

`sim/scripts/mujoco/*` holds the canonical MuJoCo script implementations.
Older `sim/scripts/<name>.py` files are compatibility wrappers only when
profiles, deploy scripts, or field notes still reference them.

## Runtime Paths

### In-Process Module Simulation

```bash
python lingtu.py sim
python lingtu.py portable_mujoco
```

This path loads `MujocoDriverModule` inside the Module graph. Motion stays
inside the simulated driver:

```text
MujocoDriverModule odometry/map_cloud
  -> maps / navigation / safety
  -> nav.velocity_mux
  -> MujocoDriverModule cmd_vel
```

Use it for fast wiring, map-layer, local-planner, visual-servo, and semantic
pipeline checks.

### MuJoCo Legacy Module Gates

```bash
python lingtu.py sim_mujoco_live gate
python lingtu.py sim_mujoco_live video
python lingtu.py sim_mujoco_octo_live octo-moving-obstacle-video
```

These profiles use external launchers and the `mujoco_fastlio2_live` contract.
They validate legacy downstream Python Module wiring and video gates from
normalized odometry/map-cloud evidence. They are simulation gates, not field
proof or native DDS equivalence.

### MuJoCo Native DDS Gate

```bash
python sim/scripts/mujoco/native_dds_sensors.py --lidar-backend mujoco_lidar
```

This is the real-equivalent simulation path: MuJoCo MID-360/IMU records feed
the same native DDS sensor boundary used by the field robot, then native C++
SLAM publishes `/slam/*` for map/nav/explore validation. Runtime Graph names
this endpoint `mujoco_native_dds`.

### Product Tasks On Simulation Endpoints

Product profiles can bind to explicit simulation endpoints:

```bash
python lingtu.py explore --endpoint mujoco_live
python lingtu.py tare_explore --endpoint mujoco_live
python lingtu.py explore --endpoint gazebo --record
```

Do not treat bare `nav`, `map`, or `explore` as simulation. Without an
explicit endpoint they may target the product field runtime.

## LiDAR And IMU

The current product simulation backend is `mujoco_lidar`. It produces real
MuJoCo ray-cast XYZI hit points for mapping, costmaps, obstacle checks, and
SLAM-style validation.

Key rules:

- Use `mujoco_lidar` for product-style simulated LiDAR.
- `ray_caster_lidar` is a fallback/compatibility backend.
- Simulated LiDAR scan timing, IMU sampling, and publish time must come from a
  single simulated hardware clock model for Fast-LIO-style validation.
- Ground-truth or odometry-prior shortcuts are diagnostics only and must not be
  presented as production localization.

Relevant scripts:

```bash
python sim/scripts/mujoco/native_dds_sensors.py --lidar-backend mujoco_lidar
python sim/scripts/mujoco/live_gate.py --mujoco-lidar-backend mujoco_lidar
python sim/scripts/mujoco/record_thunderv4_mid360_policy.py
```

## Map Validation

Saved-map gates must distinguish raw scans, live map products, saved maps, and
filtered navigation slices.

Important artifacts:

| Artifact | Meaning |
| --- | --- |
| `map.pcd` | optimized saved map used by navigation |
| `map.raw.pcd` | raw SLAM/builder output |
| `patches/*.pcd` | keyframe or scan patches |
| `poses.txt` | patch poses |
| `trajectory.txt` | per-keyframe odometry path written by native save-map |
| `map_optimization.json` | loop/refine status and point counts |
| `occupancy.npz` | 2D occupancy/cost artifact |
| `octomap.ot` | OctoPlanner3D artifact |

Useful gates:

```bash
PYTHONPATH=src:. python sim/scripts/mujoco/saved_map_quality_gate.py \
  --pcd path/to/map.pcd \
  --json-out artifacts/mujoco_saved_map_quality/report.json

PYTHONPATH=src:. python sim/scripts/mujoco/saved_map_plan_gate.py \
  --map-source mujoco_lidar \
  --json-out artifacts/mujoco_saved_map_plan/report.json

PYTHONPATH=src:. python sim/scripts/mujoco/saved_map_tracking_gate.py \
  --scene-preset corridor \
  --converter "${LINGTU_MAP_ARTIFACT_CONVERTER}" \
  --planner-executable "${LINGTU_OCTOPLANNER3D_EXECUTABLE}" \
  --json-out artifacts/mujoco_saved_map_tracking/report.json \
  --strict
```

### Continuous Mapping Quality Gate (3–5 min)

Short endpoint motion gates are necessary but not sufficient for sim radar
mapping acceptance. Use the continuous gate when claiming long-run stability:

```bash
PYTHONPATH=src:. python sim/scripts/mujoco/continuous_mapping_quality_gate.py \
  --world industrial_park \
  --duration 180 \
  --domain-id 231 \
  --drive-profile box_explore \
  --run-dir artifacts/mujoco_continuous_mapping_gate_<timestamp>
```

The gate orchestrates one isolated native DDS session (bridge + SLAM runtime +
periodic status sampling + native `save-map`) and fails unless bridge,
continuity, scale convergence, and saved-map quality all pass.

Sunrise remote runner:

```bash
python sim/scripts/run_sunrise_continuous_mapping_gate.py \
  --host 192.168.66.13 \
  --duration 180 \
  --domain-id 231
```

Keep isolated `--domain-id` in **`200–232`**. Production robot SLAM uses domain
`0`. See
[2026-07-06 continuous mapping field run](../docs/07-testing/field-runs/2026-07-06-mujoco-continuous-mapping-gate.md)
for the first 180 s verdict matrix and remaining scale-drift blocker.

When a top-down map looks thick, smeared, or duplicated, inspect these in order:

1. LiDAR hit geometry in the sensor frame.
2. LiDAR/IMU timestamps and scan-end timing.
3. LiDAR-to-body extrinsics.
4. SLAM odometry motion versus MuJoCo ground-truth motion.
5. Saved-map patch poses and `map_optimization.json`.
6. Height slice filters used for 2D occupancy preview.

## Motion Modes

`MujocoDriverModule` supports two drive modes:

| Mode | Purpose |
| --- | --- |
| `policy` | Uses the ONNX gait policy and MuJoCo contacts |
| `kinematic` | Applies `cmd_vel` directly to the floating base for deterministic headless tests |

Kinematic mode is useful for route-level software validation:

```bash
LINGTU_SIM_DRIVE_MODE=kinematic python lingtu.py sim
```

It does not prove gait stability. For policy motion:

```bash
PYTHONPATH=src:. python sim/scripts/policy_nav_smoke.py \
  --world open_field \
  --direct-duration 4 \
  --goal-distance 0.8 \
  --nav-duration 14
```

## Planning And Replay Gates

Non-motion planner preflight:

```bash
PYTHONPATH=src:. python sim/scripts/routecheck_preflight_gate.py \
  --map server_sim_demo \
  --goal-x 1.0 \
  --goal-y 0.0 \
  --json-out artifacts/routecheck_preflight/report.json
```

Replay/deviation gate:

```bash
PYTHONPATH=src:. python sim/scripts/navigation_replay_deviation_gate.py \
  --json-out artifacts/navigation_replay_deviation/report.json
```

Server aggregate gates live in `sim/scripts/server_sim_closure.py`. Treat
aggregate reports as simulation readiness only unless they reference real
field evidence.

## Scenes

Common MuJoCo scenes live under `sim/worlds/mujoco/`:

| Scene | Purpose |
| --- | --- |
| `open_field.xml` | Flat basic validation |
| `building_scene.xml` | Indoor room/corridor validation |
| `factory_scene.xml` | Industrial obstacle validation |
| `spiral_terrain.xml` | Elevation and terrain checks |

Dense local industrial scenes are preferred for LiDAR visibility and saved-map
quality checks. Sparse scenes can produce low hit counts and misleading videos.

## Tests

Fast contract tests:

```bash
python -m pytest sim/tests/test_mujoco_saved_map_quality_gate.py -q
python -m pytest sim/tests/test_mujoco_live_ground_truth_gate.py -q
python -m pytest sim/tests/test_continuous_mapping_quality_gate.py -q
```

Broader compatibility tests:

```bash
python -m pytest sim/tests/test_sim_runtime_compat.py -q
```

The broader suite may require MuJoCo assets, ONNX Runtime, policy checkpoints,
and optional external packages.

## Claim Boundaries

- Simulation can prove Module wiring, map artifact shape, and controlled
  algorithm behavior.
- Simulation cannot prove real MID-360 timing, real IMU noise, Thunder gait
  stability, physical calibration, or field safety.
- ROS launch files under `sim/launch/` are legacy compatibility checks.
- PCT and ROS2 local-planner gates are compatibility/benchmark surfaces unless
  a profile explicitly selects them.
