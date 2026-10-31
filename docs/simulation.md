# Simulation

**Status:** Current simulation architecture and usage contract
**Audience:** Simulation, Product, native runtime, and qualification maintainers
**Runs on:** `env=sim` on Windows x64 and Linux/WSL x86_64

Use simulation to exercise the same mapping -> durable save -> localization ->
navigation -> safe stop workflow without field hardware. Simulation evidence
does not prove MID-360 timing, physical calibration, gait stability, or field
motion safety. Windows-native and Linux/WSL results are separate evidence.

Thunder V4 uses CTS MoE v39 for both ordinary terrain and stairs. Its robot
package's `defaults.controller` selects the single CTS controller package;
Catalog compiles the artifact, manifest and adapter into the RunPlan's
`control_plan`. Both the compatibility MuJoCo feeder and plan-driven control
runtime use the same CTS runner. Stair profile names refer to that same model.
See [Thunder controller selection](../sim/packages/controllers/doso/thunder_v4/README.md)
for the maintained configuration interface, source bundle and historical
baseline boundaries. Earlier policy or navigation reports do not qualify a
new run with this selection.

## Offline semantic task-flow check

Before starting MuJoCo, the LLM task-decomposition and SceneGraph grounding
chain can be checked with a deterministic fixture:

```powershell
.venv/Scripts/python.exe -m sim.evaluation.task_flow --json
```

The fixture represents `先去会议室，然后找到桌子旁边的箱子` as two symbolic
subgoals, binds them to a hierarchical `building` SceneGraph, and verifies map
identity, the `box-1` relation target, and the absence of motion commands. The
structured response is local test input; no remote LLM, Product process, or
robot is started. A passing result is semantic evidence and does not qualify
MuJoCo dynamics or field motion.

## Click a navigation goal in MuJoCo (Windows)

Install the locked project environment with the simulation recording extra:

```powershell
uv sync --locked --extra dev --extra sim-recording
```

Start mapping, wait for its readiness, save the map through the Web or Gateway
SDK, and wait for the durable save operation to reach `SUCCEEDED`. Then switch
directly to `nav` with that exact saved-map identity:

```powershell
.venv/Scripts/python.exe -m lingtu.control switch map --robot doso/thunder_v4 --env sim --backend mujoco --viewer
.venv/Scripts/python.exe -m lingtu.control status --robot doso/thunder_v4 --env sim --json
.venv/Scripts/python.exe -m lingtu.control switch nav --robot doso/thunder_v4 --env sim --backend mujoco --local-planner scan --map MAP_NAME --viewer
```

Wait for the selected map, fresh localization, navigation inputs, and control
loop to be ready. Left-click a ground surface in the MuJoCo window. The viewer
submits the map-space goal through the running Gateway; an orange marker or
accepted request does not prove arrival. Missing surfaces, robot-body picks,
stale localization, and mismatched Product sessions are rejected.

Cancel a navigation task through the operator stop action. End or recover the
whole Product through ProductControl, then verify terminal zero in the run
evidence:

```powershell
.venv/Scripts/python.exe -m lingtu.sdk.cli --host 127.0.0.1 --port 5050 stop
.venv/Scripts/python.exe -m lingtu.control stop --robot doso/thunder_v4 --env sim
```

After a failed switch, inspect `status` before retrying; ProductControl either
restores the previous committed Product or leaves no committed Product. Normal
MuJoCo navigation uses synthetic IMU and LiDAR with native LTLIO. Truth
odometry remains evaluation evidence rather than estimator input. The
interactive point-navigation viewer currently supports Windows; headless runs
remain available on supported simulation hosts.

### Short SCAN click acceptance

This fixture exercises native LTLIO, Gateway goal submission, arrival,
collision checks, terminal task state, final zero acknowledgement, and cleanup:

```powershell
$env:PYTHONPATH = "$PWD/src;$PWD"
$mapDir = 'D:\path\to\factory-workshop-saved-map'
.venv/Scripts/python.exe -m sim.scripts.mujoco.native_navigation_acceptance --manifest config/acceptance/mujoco/scan_click.json --product-map $mapDir --out-dir build/scan-click-new-run --no-viewer
```

Use a fresh output directory and a saved map whose `source_profile` matches the
fixture. This command does not exercise a physical mouse gesture or field
hardware. Current scenarios, arguments, and result interpretation are owned by
[`sim/scripts/README.md`](../sim/scripts/README.md).

For navigation isolation, `scan_sequence.json` explicitly selects MuJoCo truth
localization and submits A, B, then A again in one Product session. The native
sensor publisher owns this truth fixture; it does not start `slamd`. Each leg
must reach its exact task terminal, satisfy independent physical position
evidence, and confirm native terminal stop before the next goal is sent.
`goal` is the map-frame body reference used by the planner; in this factory
fixture its Z is 0.45 m. `goal_world` is the independent world reference for
physical XY error measurement, so its ground Z remains 0.0 m.

```powershell
.venv/Scripts/python.exe -m sim.scripts.mujoco.native_navigation_acceptance --manifest config/acceptance/mujoco/scan_sequence.json --product-map $mapDir --out-dir build/scan-sequence-new-run --no-viewer
```

This fixture verifies the navigation chain and repeated goal handoff. It does
not qualify LTLIO or field localization. Keep the actual SLAM click run and
truth sequence reports separate. The inspection component fixture likewise
contains three points; it checks ordered native completion and one persisted
RGB evidence record for each point, rather than inferring route completion
from its last point alone.

Full Gateway inspection is not currently qualified on Windows: the `mapd`
query server requires Linux AF_UNIX support, and native recording session
control is Linux-only. A direct typed-DDS inspection experiment can test the
native executor and evidence capture, but does not qualify Gateway admission,
task-bound recording, or MCAP. Preserve the public API failure alongside the
component result.

The truth fixture sends odometry directly to native navigation. It currently
does not supply map-frame odometry to the Host inspection evidence worker;
capture therefore reports `odometry_missing`. A separate three-point route
with no capture actions can qualify native arrival, stop, and point progression,
but its PASS must not qualify photo evidence. SDK `position()` consumes the
current Gateway `localization.odometry` schema and reports unavailable pose
instead of returning a default zero position.

The headless factory sensor rig uses 640 × 480 RGB-D at 1 Hz to fit measured
local rendering capacity; LiDAR remains 10 Hz and IMU remains 200 Hz. Camera
rendering runs from independent state snapshots so it cannot hold the physics
loop while drawing a frame. Report actual rates and dropped frames against the
selected sensor rig, rather than assuming camera declaration implies delivery.

MuJoCo LiDAR filters environment returns hidden by robot geometry: the nearest
robot surface blocks the ray, while the robot hit itself is not published as
an environment point. A body's visual mesh is used for optical shape when
available; collision-only bodies use their physical geometry. The sensor
housing is excluded for the supported sensor mounts. A collision-only bracket
sharing a body with a visual mesh needs its own optical asset or body; a coarse
contact proxy is not a reliable laser surface. Visual meshes, installed sensor
extrinsics, sampled rays and simulated returns are approximations, not a field
calibration. CPU MuJoCo-LiDAR is the validated path; other backends must be
verified in their actual runtime before making equivalent claims.

## Rendered RGB-D goal diagnostic

`rgbd_goal_acceptance` computes a goal from a rendered RGB/depth region and
passes it to native navigation. It isolates visual projection with explicit
truth localization; it does not qualify object detection, LTLIO, a complete
Product, or field hardware.

```powershell
$mapDir = 'D:\path\to\industrial-park-saved-map'
.venv/Scripts/python.exe -m sim.scripts.mujoco.rgbd_goal_acceptance --map-dir $mapDir --out-dir build/rgbd-clear-new --case clear
.venv/Scripts/python.exe -m sim.scripts.mujoco.rgbd_goal_acceptance --map-dir $mapDir --out-dir build/rgbd-obstacle-new --case obstacle
```

Each output directory must be new. `--prepare-only` records projection evidence
without starting motion. The map's `source_profile` must match the industrial
park fixture. A valid projection or reached state alone is not a pass; inspect
projection, contact, arrival, and stop evidence together.

## Continuous RGB-D follow diagnostic

`rgbd_follow_acceptance` uses a rendered pixel fixture and production tracking
and visual-servo modules to send successive native navigation goals:

```powershell
.venv/Scripts/python.exe -m sim.scripts.mujoco.rgbd_follow_acceptance --smoke --out-dir build/rgbd-follow-short-new
.venv/Scripts/python.exe -m sim.scripts.mujoco.rgbd_follow_acceptance --out-dir build/rgbd-follow-long-new
```

The fixture is not learned person recognition, multi-person identity, a
`tracking` Product run, or Go2 locomotion. Ground-truth actor state is used only
for rendering and scoring. Keep terminal stop and collision checks when tuning
goal update thresholds. Known limitations and command ownership remain in
[`sim/scripts/README.md`](../sim/scripts/README.md) and Git history.

## Canonical chain

```text
Packages -> Catalog / SessionCompiler -> ResolvedSessionBundle
         -> SessionRuntime -> Physics / Control / Visual / Sensor / Scenario
         -> DDS / SHM adapters -> Recording / Replay / Qualification
```

[`sim/ARCHITECTURE.md`](../sim/ARCHITECTURE.md) owns detailed interfaces and
authority. This page keeps only stable operator paths and evidence boundaries.

## Workspace ownership

| Root | Owns |
| --- | --- |
| `sim/packages/` | Versioned robot, controller, sensor, world, and scenario assets |
| `sim/sessions/` | Session specifications and Product presets |
| `sim/catalog/` | Resolution and deterministic compilation |
| `sim/runtime/` | Physics, control, sensors, visual runtime, recording, and qualification |
| `sim/adapters/` | Typed DDS, camera SHM, and explicit compatibility seams |
| `sim/evaluation/` | Offline evaluation data and reports |
| `sim/distribution/` | Windows cook, staging, packaging, and release operations |
| `sim/scripts/mujoco/` | Stable Product and native acceptance entrypoints |

Open each root's README through [`sim/README.md`](../sim/README.md) for its full
catalog. `sim/packages/` is the only Catalog root; compatibility assets are not
silently promoted into it.

## Compilation and identity

ProductControl resolves Product plus `env=sim` once. Its internal execution
snapshot carries the selected Physics, Visual, Sensor, Control, Transport, and
optional Scenario plans. SessionCompiler produces module-specific plans sharing one `session_id`;
`RunAllocation` contains only PIDs, ports, DDS domains, shared-memory names, and
log paths. A reset changes `reset_generation`, invalidating older commands and
samples.

## Authority

One contact-coupled session has one MuJoCo model and data instance. MuJoCo owns
clock, dynamics, contacts, raycasts, body state, IMU, truth odometry, and
MID-360 firing. RobotSimUE follows immutable snapshots and owns presentation,
RGB, depth, and segmentation. UE transforms never become physics truth.

Recorder output is evidence, not simulation truth. A simulation command sink
never forwards to a field hardware driver.

## Adapter rules

DDS and SHM adapters translate typed contracts; they do not advance time,
infer poses, mutate packages, or become configuration owners. MID-360 adapters
preserve `offset_time_ns`, `reflectivity`, `tag`, and `line`. Evidence identifies
the session, allocation, reset generation, and observed stream.

## Worlds and scenarios

A WorldPackage owns static physics and visual facets. A ScenarioPackage owns
dynamic actors, events, stop conditions, and qualification criteria. Both
facets describe one physics scene; visual collision or query results cannot
feed back as navigation or qualification truth.

## Product integration

```text
ProductControl request (Product + env=sim)
  -> internal execution snapshot
  -> direct-child supervision
  -> ResolvedSessionBundle -> readiness / rollback / cleanup
```

Preview resolution without starting processes:

```bash
python -m lingtu.control switch map \
  --robot doso/thunder_v4 --env sim --backend mujoco --dry-run --json
```

A component script does not select or qualify a Product. A WSL result does not
promote Windows-native status, and no simulation result promotes field status.

## Runtime stream and frame contract

```text
sensor source -> endpoint adapter -> native SLAM -> map and terrain
              -> global and local planning -> checked command boundary
```

The canonical frame chain is `map -> odom -> body -> lidar_link`.
`odom -> body` must come from live odometry or an equivalent relayed state.
Runtime inspection endpoints include:

```bash
curl -fsS http://127.0.0.1:5050/api/v1/diagnostics/runtime-contract
curl -fsS http://127.0.0.1:5050/api/v1/runtime/dataflow
curl -fsS http://127.0.0.1:5050/api/v1/navigation/status
```

## Stable component commands

Resolve and run the generic coordinator without starting a Product:

This is a **Physics smoke test with zero actuator output**; it does not load
the selected policy. For policy execution, see the separate
[controlled runner](../sim/runtime/coordinator/controlled_run.py).

```powershell
python -m sim.catalog resolve `
  sim/sessions/examples/thunder_omni_contract/session.yaml `
  --repo-root . --output-dir build/runtime-session
python -m sim.runtime.coordinator build/runtime-session `
  --repo-root . --run-root build/runtime-runs `
  --mujoco-host build/mujoco-runtime-win-dsdk/Release/lingtu_mujoco_headless.exe `
  --steps 5 --reset
```

The stable Product gate requires an exact internal execution snapshot, runner,
and manifest rather than a Product name. Use the maintained target commands in
[`sim/scripts/README.md`](../sim/scripts/README.md); do not reconstruct an
acceptance invocation from an old report.

## Verification

Use [Testing](./testing.md) to select the narrowest gate. A Product report needs
the exact internal execution snapshot and saved-map identities, readiness, terminal task state,
final zero acknowledgement, cleanup, and rollback result. Do not infer these
from process liveness or a component PASS.
