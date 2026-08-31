# LingTu Simulation

Status: current simulation contract as of 2026-08-17. Simulation reports are
not field evidence unless a dated field-run document says the same behavior was
verified on a robot.

`sim/` contains LingTu's hardware-free simulation platform and validation
surfaces. **MuJoCo** is the sole simulation clock and physics authority: it owns
dynamics, contacts, ray-casting, body/joint state, inertial streams, and truth
snapshots. **Unreal Engine 5** is the high-fidelity Visual Runtime: it follows
immutable MuJoCo truth snapshots and owns presentation plus render sensors only.
No UE Actor transform is written back as simulation truth.

## Full Pipeline

```text
Authoring/Import & Asset Conditioning
  -> Versioned Packages/Catalog
  -> deterministic SessionCompiler/ResolvedSessionBundle
  -> SessionRuntime (Physics/Control/Visual/Sensor/Scenario Modules)
  -> DDS/SHM/Pixel Streaming Adapters
  -> Recorder/Replay/Episode/Qualification
  -> Cook/Distribution/Operations
```

Product integration keeps the core LingTu contract intact: `lingtu.assembly`
resolves Product + fixed `env` once into one immutable RunPlan, and
ProductControl owns staging, readiness, rollback, and current-plan commit. A
simulation RunPlan carries the already-resolved Physics, Visual, Sensor,
Control, Transport, and optional Scenario plans at one staged bundle path. The
plans share the `session_id` declared by `session.yaml`. CLI, systemd, Host, and
SessionRuntime must not resolve the Product or compile the bundle independently.
`RunAllocation` contains only runtime facts.

| Surface | Role | Current status |
| --- | --- | --- |
| MuJoCo 3.10 | Physics authority and sensor raycast backend | Active |
| G4 — RobotSimUE / UE 5.8.1 Editor | Visual Runtime and render-sensor backend | Pass: live 21-body contract, real RGB/depth frames, and inspected 1920x1080 hero render; Cook/package remains G9 |
| RGB/depth render streams | 640x480@30 camera SHM streams | Same-run Editor evidence active; not Cook/distribution proof |
| Native typed DDS sensors | IMU, truth odom, Mid360 Adapter surfaces | Pass in one coordinated SessionRuntime run; Mid360 emits field-preserving native typed frames |
| ScenarioRuntime | Deterministic evaluator and Coordinator dispatcher contract | Deterministic pedestrian vertical slice passes through MuJoCo kinematic proxy, raycast/readback truth, and UE VisualPlan materialization; behavior timeline authoring, animated crowd assets, vehicles, and scale qualification remain open |
| G7 — ProductControl/RunPlan | Resolved Physics, Visual, Sensor, Control, Transport, and optional Scenario bundle | Pass: `src/lingtu/assembly/tests/test_simulation.py` passes all 27 tests |
| SimStudio | Optional local developer application in `tools/simstudio/`; not a ProductControl Product | Package Library, guided Robot/World archive intake, Session Composer, Run Monitor, Replay Browser, and FactoryPark 2D editing |
| Catalog/import | Deterministic package query, intake, qualification, and promotion | Content-addressed archive inspection plus guided Robot/World import and promotion APIs pass; package update/remove/quarantine administration, license-policy enforcement, generic asset dependency management, and production URDF/DCC wiring remain open |
| Recorder/replay/episode | Evidence and closure chain | Full truth/command JSONL recording, content hashes, Episode closure, UE-only timestamp replay, bounded SimStudio controls, validated pageable timeline/frame browsing, and content-addressed Camera SHM RGB/depth payload capture/replay pass; typed-DDS sensor payload capture, Studio-triggered visual replay, and formal qualification verdicts remain open |
| Pixel Streaming | Human presentation/input Adapter | Qualification pending |
| Cook/stage/package/DLC/Pak | Distribution and operations chain | Trusted Windows policy, preflight, deterministic dry-run, atomic release design, and tests pass; actual Shipping Cook and packaged smoke remain unproven |

Scene publication deliberately separates mutable authoring state from runtime
content. A `SceneDraft` is revision-controlled and may be edited; publishing
one exact revision creates a new versioned `WorldPackage`, refreshes the live
Studio Catalog without restarting the service, and can be selected immediately
for a new Session. The published MuJoCo collision graph remains authoritative
and geometry-preserving. Its schema-validated world visual projection now lets
RobotSimUE transactionally materialize runtime-mode box and cylinder entities
without duplicating level-baked content; UE collision stays disabled because
MuJoCo owns contact truth.

The first published-world end-to-end proof is
`build/simstudio-e2e/live-runs/robot-yard-safety-live-v2`: 21 accepted Thunder
body bindings, Visual `ACTIVE`, real RGB/depth first frames,
and a 1920x1080 capture containing the FactoryPark level, Thunder, and the
published safety-zone entities. This run intentionally used the
`visual-applied` gate; IMU, Mid360, and truth odometry publishers were not
launched and are not claimed by this artifact.

The first plan-driven mixed-robot visual proof is
`build/simstudio-e2e/thunder-omni-open-field-hf-v1/live-runs-v2/thunder-omni-live-v2`:
one composed MuJoCo scene with ThunderV4 and OmniCart, 25 streamed dynamic
entities, 23 applied UE body bindings, Visual `ACTIVE`, and real 640x480 RGB/depth
first frames for both robot instances. The inspected 1920x1080 first-frame PNG
has SHA-256
`432aa8e6f20a51b0f7b7cd63e4e0168b2c5406de7dd94736f33d68482d86aed7`.
This is a plan-driven multi-robot Editor/runtime proof; OmniCart still uses its
declared primitive visual projection rather than a production PBR asset.

Current Editor same-run evidence is in
`build/live-runs-open-field-hf/thunderv4-openfield-hf-same-session-20260807-d`:
21 applied MuJoCo-to-UE body bindings, Physics/Control/Visual/Sensors all
`ACTIVE`, real RGB/depth SHM frames, typed DDS IMU/truth odom, and 20 Mid360
clouds. The session closes as `STOPPED` with episode status `SUCCEEDED` at
MuJoCo time 1.9 s. Treat this as passing Editor/runtime evidence; Cook and
packaged-distribution proof remain separate gates.

Fresh full-recording evidence is in
`build/motion-replay-qa-20260809-a/runs/thunder-replay-source-v3`: 121
generation-stamped truth frames and accepted commands close as `SUCCEEDED`.
The same recording was then presented by RobotSimUE without launching Physics
at `build/motion-replay-qa-20260809-a/replays/thunder-visual-replay-v1` (121
frames presented, zero dropped, four captured UE frames).

SimStudio exposes committed recordings through run-owned, path-free timeline
and frame read models. The Replay Browser lists VALID/INVALID recordings,
pages at most 200 frame summaries per request, scrubs by frame index, and
invalidates cached details when `recording_id` or `session_id` changes.
It does not launch RobotSimUE replay; the verified UE-only replay above remains
a separate runtime/CLI capability.

The first real Camera SHM payload recording proof is
`build/simstudio-payload-e2e-v4/artifacts/runs/a01908c096c146c3b6861db12c73de34`:
five generation-stamped truth frames contain eight RGB/depth references and
6,144,000 referenced bytes. All eight payloads pass content-addressed SHA-256
validation and deterministic replay with zero dropped samples. The compiled
depth wire contract is `16UC1`, `depth_scale=0.001`, while retaining the
requested metric `32FC1` encoding in SensorPlan provenance. SimStudio's public
frame projection removes the internal blob path.

The second robot control proof is in
`build/omni-cart-control-20260809-b/runs/omni-cart-forward-v1`: the analytic
differential-drive ControllerPackage advances OmniCart 0.755987 m in 2 s through
two named MuJoCo wheel actuators, with Physics and Control both `ACTIVE` and the
Episode closed as `SUCCEEDED`.

Inspected UE presentation evidence is
`build/unreal-open-field-hf/final-hf8/OpenField_HF_1920x1080.png`: 1920x1080,
21 body bindings, 21 visual links, 105 VisualOnly props, zero material compile
errors, and SHA-256 `827fc3bf342ba49e0a16510119485c43e72812949d7874faf94db1666dc1c9fd`.

See [`ROADMAP.md`](../ROADMAP.md) for the simulation roadmap.
See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the detailed architecture,
authority matrix, migration tree, and delivery gates.

Simulation can validate software wiring and algorithm contracts. It does not
prove field readiness by itself. Claims about S100P/sunrise deployment require
field runtime evidence.

## Stable Root Contract

The stable root follows the full simulation product chain, while preserving the
existing manifest-only/heavy-asset split.

| Layer | Path | Purpose |
| --- | --- | --- |
| Authoring/Import | `tools/`, `toolchains/` | Asset conditioning, deterministic import settings, provenance, license, and tool-version evidence; Blender/DCC is not runtime |
| Package | `packages/` | Versioned Robot, Controller, Sensor, SensorRig, World, and Scenario manifests |
| SessionSpec | `scenarios/catalog/` | User-authored session selections; SessionSpecs are not packages |
| Catalog/Compiler | `catalog/` | Resolves `session.yaml` into module-specific plans that share one `session_id` |
| Assets | `worlds/` | Canonical MuJoCo XML and Gazebo SDF scenes |
| Assets | `assets/` | Thunder assets, meshes, MJCF/URDF, LiDAR scan-pattern assets |
| Assets | `robots/` | Robot model, mesh, and policy assets used by simulation entrypoints |
| Assets | `sensor_rigs/` | Sensor mounting asset directories; package manifests live under `packages/sensor_rigs/` |
| Assets | `controllers/` | Controller asset directories; package manifests live under `packages/controllers/` |
| SessionRuntime | `runtime/` | `runtime/physics/`, `runtime/control/`, `runtime/visual/RobotSimUE/`, `runtime/sensors/`, `runtime/scenario/`, and `runtime/coordinator/` |
| Adapter | `adapters/` | Typed DDS, camera SHM, and related transport Adapters |
| Compatibility | `engine/`, `bridge/`, `sensors/` | Existing in-process/fallback simulation surfaces; not the generic Runtime |
| Validation | `scripts/`, `validation/`, `tests/` | Public launchers, gates, demos, benchmark entrypoints, and filesystem contracts |
| Research/Data | `fixtures/`, `planning/`, `following/`, `datasets/`, `evaluation/`, `maps/`, `semantic/`, `external_scenes/`, `experiments/`, `diagnostics/` | Fixtures, offline datasets, evaluation helpers, and compatibility/research surfaces |

`sim/runtime/physics/` plus `sim/runtime/coordinator/` is the canonical generic
MuJoCo Runtime path. `sim/scripts/mujoco/*` retains product acceptance and
legacy/field-architecture simulation gates; it is not a second generic Runtime.
Top-level MuJoCo compatibility wrappers are retired; dated field notes may keep
their historical command text as evidence.

Package migration is manifest-only: checked-in package manifests live under
`sim/packages/{robots,controllers,sensors,sensor_rigs,worlds,scenarios}/...`, while MJCF,
meshes, policies, world XML, and other heavy assets stay in `sim/robots/` and
`sim/worlds/`. SessionSpecs are not packages and remain under `sim/scenarios/`.

## Runtime Paths

### Resolved Session Runtime

The generic runtime path starts from one resolved SessionBundle. Run-specific
PIDs, logs, ports, and shared-memory names remain in `RunAllocation`:

```powershell
python -m sim.catalog `
  sim/scenarios/catalog/thunder_omni_contract/session.yaml `
  --repo-root . --output-dir build/runtime-session

python -m sim.runtime.coordinator build/runtime-session `
  --repo-root . `
  --run-root build/runtime-runs `
  --mujoco-host build/mujoco-runtime-win-dsdk/Release/lingtu_mujoco_headless.exe `
  --steps 5 --reset
```

This path composes the world, ThunderV4, and OmniCart into one `mjModel` and
drives `READY -> RUNNING -> PAUSED -> STOPPED`. Its output is
`session.runtime.json`. On Windows, RobotSimUE validates the same SessionBundle
and consumes immutable, generation-stamped body snapshots through the Session
mailbox. Controller policy execution, full native-DDS Product qualification,
Cooked delivery, and packaged smoke remain separate gates.

Formal Windows live launcher:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/sim/run_robotsimue_live.ps1 `
  -Bundle build/session-bundles/thunderv4-unreal `
  -UnrealEditor "D:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe" `
  -MujocoHost build/mujoco-runtime-physics-win/Release/lingtu_mujoco_headless.exe `
  -RunRoot build/live-runs `
  -RunId thunderv4-ue-live-proof-YYYYMMDD `
  -Gate visual-applied
```

### MuJoCo Product Gate

```bash
python sim/scripts/mujoco/native_dds_sensors.py --lidar-backend mujoco_lidar
```

This is the field-architecture compatibility/Product gate: MuJoCo MID-360/IMU records feed
the same native DDS sensor boundary used by the field robot, then native C++
SLAM publishes `/slam/*` for map/nav/explore validation. It is separate from
the generic SessionRuntime Mid360 pipeline and does not close the new G5
same-coordinated-run DDS gate. The only Product backend selector is `mujoco`;
`mujoco_native` and `mujoco_host` are retired.

Current native-DDS acceptance commands are:

```bash
PYTHONPATH=src:. python sim/scripts/mujoco/native_navigation_acceptance.py \
  --manifest config/runtime_graph/acceptance/mujoco_industrial_park_60m_navigation_acceptance.json \
  --mode motion \
  --record-video \
  --out-dir artifacts/mujoco_native_nav_60m
```

The `nav` Product catalog binds this immutable 59.94 m scenario directly to
`native_navigation_acceptance.py`. It requires real Fast-LIO2 localization,
physical rolling LiDAR, goal arrival, and at least 50 m of path length, net XY
displacement, and goal-distance reduction. Repeated runs are orchestration,
not a second acceptance implementation.

The command below is a Linux/WSL component-diagnostic compatibility path. If it
is used from a Windows shell, keep high-rate native status/log files on WSL
ext4:

```powershell
$env:PYTHONPATH = "src;."
python sim/scripts/mujoco/native_navigation_acceptance.py `
  --manifest config/runtime_graph/acceptance/mujoco_industrial_park_60m_navigation_acceptance.json `
  --mode motion `
  --out-dir '\\wsl.localhost\Ubuntu\tmp\lingtu_native_nav_60m'
```

Those children are Linux/CycloneDDS binaries. Running them from PowerShell is
Linux evidence and must never be labeled Windows-native Product evidence.
Windows-native parity for all Products is the active P0 target and requires a
RunPlan whose selected native artifacts are PE/DLL-only, including a verified
Fast-LIO `slamd.exe`; declared Python feeder/Host processes remain
interpreter-owned. The Windows `/mnt/<drive>` mount is a 9p path and is too slow
for high-rate status snapshots, so this compatibility path should use a
`\\wsl.localhost\<distro>\...` output directory. The runner performs a bounded
WSL preflight and reports `wsl_runtime_unavailable` before starting workers; it
never turns a missing native runtime into a passing result.

For an interactive assisted-teleop obstacle demo on Windows + WSL2:

```powershell
$env:PYTHONPATH = "src;."
python sim/scripts/mujoco/teleop_avoid_wasd.py --scenario obstacle_stop
```

The interactive demo defaults to the existing `mujoco_navigation_fixture` state
provider. It publishes MuJoCo ground-truth pose, TF, localization health and the
live registered LiDAR cloud over typed DDS so the run isolates the local
avoidance chain; it is not evidence that Fast-LIO2 localization passed. Use
`--state-provider fastlio2` when the full simulated SLAM chain also needs to be
validated.

The demo opens a passive MuJoCo viewer. Hold `Shift` as the deadman and use
`W/S` for forward/reverse, `A/D` for lateral intent, `Q/E` for yaw, `Space`
to command zero, and `Esc` to exit. Keyboard input publishes typed operator
requests through one persistent native command client, so direction changes do
not relaunch WSL processes. A 350 ms keyboard-heartbeat timeout sends typed
zero plus stop and ends the stream if the operator process stalls; restart the
demo before motion can resume. The native endpoint still owns LocalPlanner,
PathFollower, final safety, and `/nav/cmd_vel`. Use
`--scenario free|obstacle_slow|obstacle_stop|terrain_soft|terrain_hard`
to compare behavior. `sim/controllers/doso/thunder_v4/locomotion/keyboard.py --keyboard` is a gait-policy
debug tool that bypasses LingTu planning and must not be used as local-avoidance
evidence.

Use [`docs/07-testing/simulation/MUJOCO_NAVIGATION_ACCEPTANCE.md`](../docs/07-testing/simulation/MUJOCO_NAVIGATION_ACCEPTANCE.md)
for the exact claim boundary. The current native-DDS navigation component gate
measures the simulated map/plan/local-follow/final-DDS-command/control loop; it
does not prove the ProductControl transaction, field fault handling, or
physical locomotion.

### Products In `env=sim`

ProductControl resolves a Product inside `env=sim`; the backend is internal
environment implementation configuration, not a Product or Endpoint:

```bash
python -m lingtu.control switch teleop_avoid --robot doso/thunder_v4 --env sim --dry-run --json
python -m sim.scripts.mujoco.product_acceptance \
  --run-plan <published-teleop-avoid-run-plan.json> \
  --runner sim/scripts/mujoco/teleop_avoid_native_acceptance.py \
  --manifest config/runtime_graph/acceptance/mujoco_teleop_avoid_native_acceptance.json \
  --state-root <isolated-product-state-dir> --dry-run --json
```

Component scripts remain development tools; they do not select a Product
deployment environment.

The Product acceptance catalog is explicit about claim scope:

| Product | Gate scope | Verified now | Deliberately not claimed |
| --- | --- | --- | --- |
| `teleop` | Component | Typed operator lifecycle, Driver command/ACK, and terminal-zero component contracts | Exact ProductControl switch/current/ledger/readiness/stop/cleanup/rollback |
| `teleop_avoid` | Component | Eight named local-avoidance scenario contracts | Exact Product lifecycle and the complete matrix under that lifecycle |
| `map` | Component | Attached exact-session sensing, RGB-D, SaveMap, and nonempty-PCD contracts | Exact Product switch/stop/rollback and Product promotion |
| `explore` (`live`, `map`) | Component | Route-specific SLAM/exploration/planning contracts | Exact attached Product lifecycle, repeatability, and Product promotion |
| `nav` | Component | Bound 59.94 m scenario, planning/following, and terminal-zero contracts | A current exact Product run, repeated >=50 m evidence, and rollback |
| `tracking` | Component | Stable simulated person ID, exact target selection, VisualServo map-goal request, and visual stop | Native goal execution, continuous following motion, exact Product lifecycle, and field evidence |
| `inspection` | Component | Attached task/ACK/status/RGB/report/recording contracts | Exact Product switch/stop/rollback, long-duration/fault, and field evidence |

Every currently selected MuJoCo manifest is `component`; the exact Product PASS
count is zero. `component` means the Product is cataloged and its native
subchain is measured without overstating the missing lifecycle. Windows and
Linux/WSL require separate exact Product reports; a WSL run does not promote
Windows.

### Offline Unity Import

Unity is not a simulation runtime backend. The former launcher, ROS 2 relay,
RViz view, runtime contract, and evidence surface have been removed. Existing
Unity semantic exports can be converted offline with
`lingtu-maps-import-unity`; this does not start a simulator or prove Product
runtime behavior.

## LiDAR And IMU

The product-style simulation backend is `mujoco_lidar`. It produces MuJoCo
ray-cast XYZI hit points for mapping, costmaps, obstacle checks, and SLAM-style
validation. MuJoCo remains the authority for LiDAR raycasts, IMU primitives,
and truth odometry; render cameras belong to RobotSimUE.

Current status:

| Stream | Status | Claim boundary |
| --- | --- | --- |
| RGB/depth | Same-run Editor evidence active at 640x480@30 through camera SHM with basis `real_rendered_frame_to_camera_shm`. | Not Cook/distribution proof. |
| IMU | Native typed DDS stream is `ACTIVE` in the canonical coordinated run. | Editor/runtime same-session proof; not physical IMU evidence. |
| Truth odometry | Native typed DDS stream is `ACTIVE`; truth odom remains validation data and is not estimator input by default. | Editor/runtime same-session proof; not an estimated odometry claim. |
| Mid360 | Pattern cursor -> MuJoCo raycast -> typed `Mid360FrameSample` -> native publisher produced 20 clouds while preserving `offset_time_ns`, `reflectivity`, `tag`, and `line`. | Same coordinated-run G5 proof; not physical LiDAR evidence. |

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
python sim/scripts/mujoco/native_navigation_acceptance.py --mode motion
python sim/scripts/mujoco/record_thunderv4_mid360_policy.py
```

## Map Validation

Saved-map gates must distinguish raw scans, live map products, saved maps, and
filtered navigation slices.

Important artifacts:

| Artifact | Meaning |
| --- | --- |
| `map.pcd` | saved point cloud used by navigation |
| `patches/*.pcd` | keyframe or scan patches |
| `poses.txt` | patch poses |
| `trajectory.txt` | per-keyframe odometry path written by native save-map |
| `occupancy.npz` | 2D occupancy/cost artifact |
| `octomap.ot` | OctoPlanner3D artifact |

Useful gates:

```bash
PYTHONPATH=src:. python sim/scripts/mujoco/saved_map_quality_gate.py \
  --pcd path/to/map.pcd \
  --json-out artifacts/mujoco_saved_map_quality/report.json

PYTHONPATH=src:. python sim/scripts/saved_map_relocalize_runtime_gate.py \
  --map-pcd path/to/map.pcd \
  --preflight-only \
  --strict
```

Runtime tracking and motion are validated separately through
`mujoco/native_navigation_acceptance.py`; the removed Python combined
plan/tracking gate is not a Product runtime path.

### Continuous Mapping Quality Gate (3–5 min)

Short endpoint motion gates are necessary but not sufficient for sim radar
mapping acceptance. Use the continuous gate when claiming long-run stability:

```bash
PYTHONPATH=src:. python sim/scripts/mujoco/continuous_mapping_quality_gate.py \
  --world industrial_park \
  --duration 180 \
  --domain-id 231 \
  --drive-profile box_explore \
  --saved-map-dir /path/to/mapd/saved/map \
  --run-dir artifacts/mujoco_continuous_mapping_gate_<timestamp>
```

The gate orchestrates one isolated native DDS session (bridge + SLAM runtime +
periodic status sampling), but it does not save maps. Supply a map directory
already produced through the public SDK/Gateway -> native `mapd save_map` chain.
The directory must contain `map.pcd` and `trajectory.txt` from the matching run;
the trajectory time join rejects unrelated captures. The gate fails unless
bridge, continuity, scale convergence, and saved-map quality all pass.

Sunrise remote runner:

```bash
python sim/scripts/run_sunrise_continuous_mapping_gate.py \
  --host "$LINGTU_SIM_HOST" \
  --duration 180 \
  --domain-id 231
```

Set `LINGTU_SIM_HOST=FIELD_COMPUTE_HOST` before using the remote runner.

Keep isolated `--domain-id` in **`200–232`**. Production robot SLAM uses domain
`0`. See
[2026-07-06 continuous mapping field run](../docs/07-testing/field-runs/2026-07-06-mujoco-continuous-mapping-gate.md)
for the first 180 s verdict matrix and remaining scale-drift blocker.

When a top-down map looks thick, smeared, or duplicated, inspect these in order:

1. LiDAR hit geometry in the sensor frame.
2. LiDAR/IMU timestamps and scan-end timing.
3. LiDAR-to-body extrinsics.
4. SLAM odometry motion versus MuJoCo ground-truth motion.
5. Saved-map patch poses and their `patches/*.pcd` inputs.
6. Height slice filters used for 2D occupancy preview.

## Motion Modes

`MujocoDriverModule` supports two drive modes:

| Mode | Purpose |
| --- | --- |
| `policy` | Uses the ONNX gait policy and MuJoCo contacts |
| `kinematic` | Applies `cmd_vel` directly to the floating base for deterministic headless tests |

Kinematic mode is useful for route-level software validation:

```bash
python sim/scripts/mujoco/native_navigation_acceptance.py --help
```

It does not prove gait stability. For policy motion:

```bash
PYTHONPATH=src:. python sim/scripts/mujoco/native_navigation_acceptance.py --help
```

## Planning And Replay Gates

Replay/deviation gate:

```bash
PYTHONPATH=src:. python sim/scripts/navigation_replay_deviation_gate.py \
  --json-out artifacts/navigation_replay_deviation/report.json
```

Server aggregate gates live in `sim/scripts/sim_diagnostics.py`. Treat
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
python -m pytest sim/tests/test_continuous_mapping_quality_gate.py -q
python -m pytest sim/tests/test_mujoco_native_navigation_acceptance.py -q
```

Broader Product acceptance tests:

```bash
python -m pytest sim/tests/test_mujoco_product_acceptance.py -q
```

The broader suite may require MuJoCo assets, ONNX Runtime, policy checkpoints,
and optional external packages.

## Claim Boundaries

- Simulation can prove Module wiring, map artifact shape, and controlled
  algorithm behavior.
- Simulation cannot prove real MID-360 timing, real IMU noise, Thunder gait
  stability, physical calibration, or field safety.
- Product simulation uses the same native navigation endpoint shape as `real`.
