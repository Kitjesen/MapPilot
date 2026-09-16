# Simulation

**Status:** Current simulation architecture and usage contract
**Audience:** Simulation, Product, native runtime, and qualification maintainers
**Runs on:** `env=sim` on Windows x64 and Linux/WSL x86_64

Root `sim/` is the simulation workspace. `env=sim` is the ProductControl
environment value. A component can use the workspace without becoming a
Product run.

Simulation does not prove real MID-360 timing, physical IMU noise, hardware
calibration, gait stability, or field safety. Windows-native and Linux/WSL are
separate evidence targets.

## Click a navigation goal in MuJoCo (Windows)

For simulation recording (OpenCV PNG/MP4 and imageio GIF output), install the
recording extra in the project environment:

```powershell
.venv/Scripts/python.exe -m pip install -e '.[sim-recording]'
```

This includes the MuJoCo and vision dependencies. OpenCV uses its headless
package; the interactive MuJoCo window is provided by GLFW. Native DDS simulation
and the Python semantic imports do not require a ROS2 runtime.

Start a saved-map navigation Product with the viewer:

```powershell
.venv/Scripts/python.exe -m lingtu.control switch nav --robot doso/thunder_v4 --env sim --backend mujoco --local-planner scan --map MAP_NAME --viewer
```

Use a saved map corresponding to the selected simulation world. Wait for Product
startup and localization to complete, then left-click a ground surface in the
MuJoCo window. An orange marker shows the selected world point; the text overlay
reports submission or rejection. Dragging still controls the camera. No browser
is required. The current simulation Host/Gateway must be running: the viewer
submits through its existing navigation API, and native planning and motion gates
still apply. `teleop` and `teleop_avoid` do not accept point-navigation goals.

Picks preserve surface height. Normal MuJoCo navigation defaults to Fast-LIO2,
using synthetic IMU and LiDAR measurements rather than ground-truth poses.
Picks are converted using the current
SLAM transform and a simulation body pose matched to the SLAM timestamp (within
50 ms). Missing surfaces, robot-body picks, stale localization,
and another Gateway session are rejected. A submitted goal is an acknowledgement,
not evidence of arrival. This mouse input currently supports Windows only.

The viewer connects directly to the loopback Gateway, without using system HTTP
proxies. Rejections show the Gateway's specific blocker, such as
`local_collision_stale` or `native_control_loop_unhealthy`. Under heavy machine
load these gates can prevent motion even when the selected-point marker appears.
The overlay prioritizes current input and control-loop health over an inactive
historical timeout. Missing or unready control-loop health displays a waiting
state, even if navigation inputs are ready.

The Windows click-navigation window handles queued mouse events and draws on one
thread. Picking uses the retained scene, camera and body state of the displayed
frame before pending display updates are applied. A resize requires a new frame
before accepting another click. Simulation pose history is collected on every
physics observation, independently of rendering, for matching the SLAM timestamp.
Unchanged paths and obstacle
geometry are reused; map overlays use the inverse of the pick transform in
Fast-LIO2 mode. Status from another Product session or older than one second is
shown as unavailable rather than ready. Closing the viewer cancels its HTTP read.
An unavailable `map_odom_tf` during initial map alignment rejects a pick and
keeps the window alive. Feeder error logs retain the underlying viewer traceback.
Without `--viewer`, ProductControl resolves the MuJoCo runtime as headless even
when the selected preset declares preview mode.

`slam_runtime` owns localization; IMU and LiDAR endpoints own their respective
sensor streams. A SLAM configuration filename never enables truth-prior injection.
The separate `/sim/truth/odom` stream is evaluation evidence, not estimator input
by default. The old `localization=truth` sensor fixture remains an explicit
diagnostic path; its mixed sensor/localization ownership and shared send channel
are not the intended Product architecture and still need replacement before it
can be presented as a complete navigation solution.

Pending LiDAR work retains only the newest frame. Frames exceeding the RunPlan's
cloud age limit are dropped with accounting, while Fast-LIO2 IMU delivery remains
ordered. Restart through ProductControl to generate a RunPlan using the corrected
default. An existing saved RunPlan retains its original localization selection.

### Short SCAN click acceptance

The short fixture exercises actual Fast-LIO2 and the same Gateway endpoint as
the window. It uses ProductControl for startup, shutdown and rollback, and
requires a matching task to reach its terminal state, physical arrival, no
entity collisions, fresh inputs and the driver's final zero acknowledgement:

```powershell
$env:PYTHONPATH = "$PWD/src;$PWD"
.venv/Scripts/python.exe -m sim.scripts.mujoco.native_navigation_acceptance --product-map C:/Users/99563/data/lingtu/maps/factory_workshop_v2 --out-dir build/scan-click-new-run --no-viewer
```

Use a fresh output directory. `--product-map` names an existing saved map;
the default `config/acceptance/mujoco/scan_click.json` fixture matches the current
nav Product's `factory_workshop@2.0.0` scene, starting at `(61, 16.5)` with yaw
`pi/2` and a two-metre forward goal. Its
`goal_world` is independent evaluation input, not estimator input. This HTTP
test does not exercise a physical operating-system mouse gesture. The saved
map's `source_profile` must match the fixture; a map from another scene is
rejected before Product startup. Selecting a saved map does not change the
Product's simulation world or physical spawn.

The 2026-09-10 Windows Product runs are **not qualified**. Run 11 exposed a
TF publication storage bug: the planning start remained near the odometry
origin while the target and collision ROI used the translated map frame.
After fixing the production DDS publication boundary, run 12 plans from the
correct map coordinates, physically travels 2.144 m and reaches the goal
with about 0.208 m XY error and no entity contact. Its readiness fraction and
LiDAR drop rate still fail the unchanged gates, so arrival alone does not
qualify the complete Product. See the Go2 record below for separate lifecycle
and shutdown results.
Separate truth-localization diagnostics passed a two-metre goal and a static
shelf detour, including step-level entity-contact checks and terminal zero
acknowledgements; these do not qualify Fast-LIO2 Product navigation or Go2
hardware. The initial-pose interface accepts explicit `x y z yaw`; the
three-value `x y yaw` form retains Z=0. Geometry-derived saved maps require the
body-centre height in their map frame.

The dynamic crossing test remains failed: after repairing current-frame
collision hits and the MID360 angular sampling clock, run 04 still records
58 physical contact steps. Later arrival and stopping do not qualify avoidance.
Current results, outstanding frame/throughput work and field boundaries are
recorded in the [Go2 README](../config/robots/unitree/go2/README.md#2026-09-10-扩展验收).

## Canonical chain

```text
Packages
  -> Catalog / SessionCompiler
  -> ResolvedSessionBundle
  -> SessionRuntime
  -> Physics / Control / Visual / Sensor / Scenario
  -> DDS / SHM / Pixel Streaming adapters
  -> Recording / Replay / Qualification
  -> Distribution
```

[`sim/ARCHITECTURE.md`](../sim/ARCHITECTURE.md) owns the implementation-level
interfaces. This page describes stable repository and Product boundaries.

## Workspace ownership

| Root | Owns |
| --- | --- |
| `sim/packages/` | Versioned robots, controllers, sensors, sensor rigs, worlds, scenarios, and payloads |
| `sim/sessions/` | Product presets and example SessionSpecs |
| `sim/catalog/` | Single-root resolution, compilation, management, and importers |
| `sim/contracts/` | Timebase and simulation schemas |
| `sim/runtime/` | Coordinator, physics, control, visual, sensors, scenario, recording, replay, and qualification |
| `sim/adapters/` | Typed DDS, camera SHM, and explicit Gazebo compatibility |
| `sim/compat/` | Legacy direct Python engine, compatible sensors, and reference assets |
| `sim/diagnostics/` | Simulation reports and Gazebo TF checks |
| `sim/evaluation/` | SLAM/navigation evaluation data and package qualification records |
| `sim/distribution/` | Windows Cook, staging, packaging, and release operations |
| `sim/tools/` | Asset, world, planning, and toolchain helpers |
| `sim/scripts/mujoco/` | Stable Product and native acceptance entrypoints |
| `sim/tests/` | Simulation regression contracts |

`sim/packages/` is the only Catalog root. Package-owned assets remain with
their manifest. `compat/` is not scanned by CatalogResolver.

Package kinds are `robots`, `controllers`, `sensors`, `sensor_rigs`, `worlds`,
`scenarios`, and `payloads`. A session selects versions and parameters; it
does not own PIDs, ports, DDS domains, SHM names, or logs.

## Compilation and identity

SessionCompiler produces module-specific plans with one shared `session_id`.
It does not emit a god object that exposes every Module's private settings.

Product plus `env=sim` is resolved once by `lingtu.assembly`. The RunPlan
carries the already selected Physics, Visual, Sensor, Control, Transport, and
optional Scenario plans. CLI, Host, and SessionRuntime do not compile the
bundle again.

`RunAllocation` contains runtime allocation only: PIDs, ports, DDS domain, SHM
names, and log paths. It is not a second RunPlan.

A reset changes `reset_generation`. Commands, snapshots, and samples from an
older generation must be rejected or ignored.

## Authority

One contact-coupled session has one `mjModel` and `mjData`. MuJoCo is the sole
simulation clock, dynamics, contact, raycast, body/joint state, and truth
authority.

RobotSimUE follows immutable truth snapshots. Unreal owns presentation and
render sensors only; UE Actor transforms cannot be written back as simulation
truth. Pixel Streaming is presentation and teleoperation, not an algorithm
sensor contract.

| Module | Owns |
| --- | --- |
| Physics | Clock, dynamics, contacts, truth, and raycast |
| Control | Policy/PD scheduling, actuator binding, and stale-command safe stop |
| Visual | UE presentation, RGB, depth, and segmentation |
| Sensor | Multi-rate scheduling, sample identity, and stream readiness |
| Scenario | Deterministic events, criteria, and verdict input |
| Recorder | Evidence and replay material; never simulation truth |

MuJoCo owns IMU, truth odometry, and MID-360 firing/raycast behavior.
RobotSimUE owns RGB, depth, and segmentation. Truth odometry is not estimator
input by default.

## Adapter rules

DDS adapters translate typed process contracts. SHM adapters use the canonical
`lingtu.camera.shm_frame.v1` transport. An adapter cannot advance the clock,
infer a pose, mutate a package, or become a second configuration source.

MID-360 adapters preserve `offset_time_ns`, `reflectivity`, `tag`, and `line`.
Evidence names the session, allocation, generation, and observed stream
identity.

A simulation command sink never forwards to the field hardware driver.

## Worlds and scenarios

A WorldPackage owns static identity and the physics/render facets. A
ScenarioPackage owns dynamic actors, events, stop conditions, and
qualification criteria.

The session composes one physics scene. Physics and visual facets compile from
the same source world and cannot represent contradictory truth.

Dynamic scenario intent is applied explicitly through a dispatcher. UE
collision or query results cannot feed back as navigation, LiDAR, or
qualification truth.

## Product integration

```text
Product + env=sim
  -> RunPlan
  -> ProductControl
  -> direct-child process supervision
  -> exact ResolvedSessionBundle
  -> readiness / rollback / cleanup
```

Preview without side effects:

```bash
python -m lingtu.control switch teleop_avoid \
  --robot doso/thunder_v4 --env sim --dry-run --json
```

A component script does not select Product env. Component PASS does not equal
Product PASS. A WSL result does not promote Windows-native status.

Do not write or ship Python SLAM as a simulation substitute for the native
Product owner. Reports must keep `no_python_slam=true` when claiming native
SLAM boundary equivalence.

## Runtime stream and frame contract

Simulation backends normalize a sensor, log, or simulator source into
canonical runtime stream tokens. This contract is not a ROS2 topic browser;
adapters may observe native streams or topics.

```text
sensor/log/simulator source
  -> endpoint adapter
  -> slam_or_relayed_localization_map
  -> map layers and exploration
  -> global planning
  -> local planning and following
  -> command_boundary
```

The canonical frame chain is:

```text
map -> odom -> body -> lidar_link
```

`map->odom` and `body->lidar` may be static. `odom->body` must be observed from
live odometry or an equivalent relayed state stream.

`diagnostics.runtime_contract` exposes `frame_links`, `runtime_data_flow`, and
`resolved_runtime_data_flow.<data_source>`. For example,
`mujoco_fastlio2_live` resolves to `/lidar/raw_frame + /imu/raw` at its endpoint,
then into canonical SLAM outputs.

The comparison path `mujoco_fastlio2_live -> field` evaluates normalized
architecture, not physical equivalence. Publisher identity, frames, command
sink, and `runtime.blockers` remain part of the verdict.

An endpoint is a concrete HTTP, DDS, or native-service access point. It is not
an env, Product, deployment identity, or RobotConfig selector.

Useful inspection surfaces include:

```bash
curl -fsS http://127.0.0.1:5050/api/v1/diagnostics/runtime-contract
curl -fsS http://127.0.0.1:5050/api/v1/runtime/dataflow
curl -fsS http://127.0.0.1:5050/api/v1/navigation/status
```

Direct-process records may contain `launcher` and `launcher_args`. Those are
private implementation details after RunPlan selection.

## Stable component commands

Resolve an example session:

```powershell
python -m sim.catalog resolve `
  sim/sessions/examples/thunder_omni_contract/session.yaml `
  --repo-root . --output-dir build/runtime-session
```

Run the generic coordinator:

```powershell
python -m sim.runtime.coordinator build/runtime-session `
  --repo-root . `
  --run-root build/runtime-runs `
  --mujoco-host build/mujoco-runtime-win-dsdk/Release/lingtu_mujoco_headless.exe `
  --steps 5 --reset
```

Run live visual integration without encoding a local Unreal path in docs:

```powershell
python -m sim.runtime.coordinator.live_visual `
  build/session-bundles/thunderv4-unreal `
  --unreal-editor <UnrealEditor.exe> `
  --mujoco-host <lingtu_mujoco_headless.exe> `
  --run-root build/live-runs `
  --run-id <run-id> `
  --gate visual-applied
```

Discover the stable Product gate:

```bash
python -m sim.scripts.mujoco.product_acceptance --help
```

## Verification

```bash
python -m pytest tests/sim/test_mujoco_product_acceptance.py -q
python -m pytest tests/lingtu/assembly/test_simulation.py -q
```

A Product report also needs exact RunPlan identity, readiness, terminal zero,
cleanup, and rollback. See [Testing](./testing.md).
