# Navigation Capability Evidence Matrix

Status: current evidence ledger
Audited: 2026-08-17
Scope: native field products, portable C++ cores, Host adapters, simulation, and field acceptance

This document answers two different questions separately:

1. Does implementation exist?
2. Has that implementation been connected and proven as a product capability?

A source file or unit test is not enough to call a capability complete.

## Evidence Levels

| Mark | Evidence |
| --- | --- |
| C | A real implementation and typed contract exist. |
| P | A Product owns and starts the implementation with one declared writer. |
| T | Deterministic unit or component tests cover normal and fail-closed behavior. |
| S | MuJoCo or a labelled replay validates the integrated behavior. |
| F | S100P and MID-360 field evidence validates timing, resources, and safety. |

A product capability is complete only when it has C, P, T and the appropriate S/F
evidence. "Partial" below does not mean absent; it means a specific layer or
acceptance gate is still missing.

The MuJoCo backend now compiles exact RunPlans for all seven Products, with
separate exact `explore:live` and `explore:map` manifests. Every MuJoCo manifest
currently selected by that backend still declares `coverage=component`.
RunPlan wiring is implementation evidence; it is not a Product pass.

## Current Capability Matrix

| Capability | Current implementation | Evidence | Verdict | Remaining product gap |
| --- | --- | --- | --- | --- |
| Map Server | Native map store, records, artifacts, active slots, map graph, save transactions, mapd realtime layers and scene publication under `src/maps`. | C, P, T; S partial; F pending | Strong partial | Persistent control/query still crosses a thin Python Host facade in some paths. The typed mapd control/query endpoint and field save/recovery evidence are not yet the only path. |
| Route Server | `maps::MapGraph` provides persisted nodes/edges, map-qualified portal transitions, edge health/enabled state, and shortest routes. | C, T; P/S/F pending | Foundation only | No independent typed route request/status/event lifecycle, route-operation plugin contract, traffic closure updates, or route progress telemetry. |
| Waypoint Follower | Native inspection store/executor and endpoint runtime support multi-point routes, dwell/actions, retries, skip/stop policies, pause/resume/cancel, revisions, and evidence. | C, P, T; S substrate only; F pending | Implemented for inspection; Product acceptance pending | Archive one exact ProductControl MuJoCo report covering the typed task/event/evidence lifecycle, then field evidence. It is not a generic waypoint server. |
| Lifecycle Manager | `ProductControl` owns apply, switch, stop, readiness, rollback, and immutable RunPlans. `real + systemd` uses `SystemdRunner`; `sim + subprocess` uses direct-child supervision. | C, P, T; S implemented; F partial | LingTu equivalent implemented | Do not import the ROS lifecycle manager. Continue crash/restart and release rollback evidence on S100P. |
| Collision Monitor | Native swept-footprint safety evaluates cloud/traversability and produces stop/slow/limited commands. Local planning also checks the full footprint. | C, P, T; S not currently accepted; F pending | Strong partial | The current safety chain needs a passing end-to-end MuJoCo report and field proof. A reusable multi-zone/TTC policy contract is still missing. |
| Velocity Smoother | Portable ROS-free C++ core in `nav_kernel::VelocitySmoother`, shared by direct teleop, assisted teleop, and autonomy before final safety. RunPlan configuration reaches `navd`; hard-zero paths reset state and safety-limited commands rebase it to the applied output. | C, T; native endpoint build proven; P/S/F pending | Integrated, default disabled | No Product enables it yet. Add status diagnostics, tune and accept a MuJoCo canary, then collect field ramp/reversal/takeover/stop evidence before promotion. Closed-loop mode remains unavailable until a typed velocity-feedback source exists. |
| Path Smoother | Some planners and path followers smooth implicitly or limit steering. | Internal behavior only | Missing as an independent capability | Add a typed, replaceable, collision-aware smoother contract between global planning and path activation. Never smooth through an obstacle or destroy planner kinematic constraints. |
| Docking / Charging | No formal Product, dock database, detector contract, final approach controller, or charging proof. | None | Missing | Requires hardware contact/charge evidence, a dock pose source, staged navigation, final control, retry/undock lifecycle, and a simulation fixture. |
| Target Following | The `tracking` Product selects a stable RGB-D person ID, tracks/ReIDs the person in Host, and sends bounded map-frame goals to native navigation. Native navigation remains the only motion owner. | C, P, T; static S wiring; dynamic S/F pending | Integrated, not dynamically accepted | Add moving-person MuJoCo evidence for continuity, occlusion/reacquisition and physical standoff, then supervised field timing and safety evidence. |

## What Is Already Real

### Map Server

The repository is not missing a map implementation. Existing native surfaces
include:

- `src/maps/cpp/mapd/`: realtime `MapObservation` ingestion and scene/layer publication.
- `src/maps/cpp/store.cpp`, `service.cpp`, `save.cpp`: records, query/control, and save transactions.
- `src/maps/cpp/map_graph.cpp`: persisted inter-map graph and route search.
- `src/maps/cpp/layers/`: voxel, rolling occupancy, exploration projection, and semantic occupancy.
- `src/maps/include/lingtu/maps/layers/`: occupancy, elevation, ESDF, traversability, and related contracts.
- `src/maps/prune/cpp/`: native save-time dynamic-map pruning.

The remaining map work is boundary convergence and product evidence, not a new
Map Server rewrite.

### Waypoint Following

Inspection is the product-grade multi-waypoint implementation:

- `src/nav/inspection/inspection.cpp`
- `src/nav/inspection/store.cpp`
- `src/nav/cpp/endpoint/nav/runtime/inspection/`
- `config/runtime_graph/products/inspection.yaml`

Calling this capability "missing" would be inaccurate. The accurate statement
is: generic waypoint execution is missing, while inspection waypoint execution
already exists.

### Lifecycle

LingTu deliberately replaces ROS node lifecycle with product/process lifecycle:

```text
Product + env -> immutable RunPlan -> ProductControl
                                      |-- real + systemd -> SystemdRunner
                                      `-- sim + subprocess -> direct children
```

Blueprint remains active only inside the Python Host and is not the field
process lifecycle manager.

## Dynamic Obstacle And Residual Status

The algorithms exist:

- `tests/maps/cpp/voxel_layer_test.cpp`: column carving, adjacent-height
  isolation, voxel voting, and decay.
- `tests/maps/cpp/rolling_occupancy_test.cpp`: ray clearing, rolling
  bounds, independent time decay, and frame/order rejection.
- `tests/maps/cpp/mapd_engine_test.cpp`: exact observation pose/origin,
  epoch/sequence handling, floor isolation, resource caps, and timer-driven
  decay.
- `tests/nav/cpp/endpoint/test_motion_layer.cpp`: old obstacle residue
  cleared by later rays, current hit retention, static-cell protection,
  dynamic-track confirmation, and expiry.

The current native component-chain moving-person gate now exists:

- `sim/scripts/mujoco/teleop_avoid_native_acceptance.py` defines the optional
  `moving_person_clear` scenario.
- `sim/scripts/mujoco/native_dds_sensors.py` drives one deterministic MuJoCo
  mocap person through the live scan.
- `sim/scripts/mujoco/map_scene_roi_monitor.py` consumes the bounded native
  `MapScene` ABI and records only ROI counts for `live`, `voxel`, and
  `accumulated` clouds.
- `tests/sim/test_mujoco_dynamic_obstacle_scene.py` checks detection, clearing,
  persistent-residual failure, exact scene geometry, and map/control ownership.

This closes the old test-design gap, but it is not field evidence. A current
strict MuJoCo report and a long-running MID-360 report are still required.
The 2026-07-28 strict preflight did not start the scene because the selected
LiDAR/IMU/camera publisher processes, SLAM, mapd, traversability, navd,
navigation-control/client,
and cmd-velocity tap binaries were older than their current source/IDL closure.
That fail-closed result is a release-provenance blocker, not simulation
evidence and not proof that clearing passed or failed.
Therefore:

```text
column carving code: implemented
decay code: implemented
rolling obstacle layer: implemented
person residual component acceptance: gate implemented; Product report not yet archived
long-duration resource acceptance: not complete
```

### Required MuJoCo Scenarios

| Scenario | Required assertion |
| --- | --- |
| Person crosses and leaves | Vacated occupancy clears within the configured p95 deadline. |
| Person stops, then leaves | Multi-frame confirmation does not make a dynamic obstacle permanent. |
| Person passes behind a wall | Clearing rays do not create false free space through occlusion. |
| Static wall and thin pole | Static retention and thin-obstacle retention stay above thresholds. |
| Adjacent floor at same XY | Column carving does not clear another height band. |
| Glass/reflection noise | Isolated returns expire without erasing stable structure. |
| 30-60 minute loop | Voxel cells, memory, CPU, DDS bytes, and clear latency remain bounded. |
| Epoch reset/relocalization | Old obstacle generation is cleared atomically. |

Use ProductControl to verify exact RunPlan delegation and
wiring. The selected manifest still controls the evidence class, so the current
result remains component evidence:

```bash
python -m sim.scripts.mujoco.product_acceptance \
  --run-plan <published-teleop-avoid-run-plan.json> \
  --runner sim/scripts/mujoco/teleop_avoid_native_acceptance.py \
  --manifest config/runtime_graph/acceptance/mujoco_teleop_avoid_native_acceptance.json \
  --state-root <isolated-product-state-dir> --json
```

`teleop_avoid_native_acceptance.py` is the attached scenario adapter used by
that Product transaction; it is not a standalone executable. The focused
Explore runners below remain component diagnostics. A verified RunPlan does
not promote them while their manifests declare `coverage=component`:

```bash
python sim/scripts/mujoco/explore_native_acceptance.py \
  --manifest config/runtime_graph/acceptance/mujoco_explore_native_acceptance.json \
  --artifact-dir artifacts/mujoco-explore-live \
  --strict

python sim/scripts/mujoco/explore_native_acceptance.py \
  --manifest config/runtime_graph/acceptance/mujoco_explore_map_native_acceptance.json \
  --artifact-dir artifacts/mujoco-explore-map \
  --strict
```
`moving_person_clear` is the current typed-DDS dynamic-obstacle case. It fails
unless the person is observed, both voxel and
accumulated layers return near their pre-person baseline after the configured
decay grace, scene identity stays fixed, and generations remain monotonic.


A component report is accepted only when it records exact source/binary
provenance, scenario metrics, cleanup/zero evidence, and no blockers. Product
acceptance additionally requires a `coverage=product` manifest and the complete
ProductControl transaction.

The source and RunPlan topology assign raw LiDAR/IMU publication and native
Fast-LIO `slamd` to independent processes. The MuJoCo catalog declares private
Windows and Linux commands, and compilation selects one host platform before
artifact resolution or lifecycle work. A RunPlan therefore cannot mix PE and
ELF processes.

Windows x64 staging now includes the declared Fast-LIO `slamd.exe`, and the
SLAM-dependent Product RunPlan compile contract is covered. This closes the old
missing-artifact and compile-wiring gap; it does not prove full Windows-native
Product acceptance. Linux/WSL separately needs the shared native DDS publisher
binary and driver bridge rebuilt for that platform. Each platform still needs
its own fresh exact ProductControl report. WSL is not Windows-native evidence,
and simulator-truth odometry is not SLAM evidence.

### Dataset Replay

MuJoCo validates the control loop but cannot reproduce all LiDAR artifacts.
Use both:

- [HeLiMOS](https://sites.google.com/view/helimos/home) for point-wise moving
  object labels across heterogeneous LiDARs, including solid-state sensors.
- [Dynamic Map Benchmark](https://kth-rpl.github.io/DynamicMap_Benchmark/) for
  ghost-removal and static-map preservation metrics across online and offline
  dynamic-map methods.

Dataset replay validates filtering quality; it does not replace closed-loop
MuJoCo or S100P safety acceptance.

## Upstream Adoption Decision

[Navigation2](https://github.com/ros-navigation/navigation2) is the main mature
reference because it contains map, route, waypoint, smoother, collision,
docking, following, and lifecycle packages in one maintained system. LingTu
must not vendor its ROS wrappers. The useful parts are portable algorithms,
state/failure semantics, parameters, and tests.

| Upstream capability | LingTu decision |
| --- | --- |
| Map Server | Keep LingTu maps. Nav2 is primarily a 2D map-server/lifecycle surface and is not a replacement for LingTu's 3D artifact service. |
| Route Server | Adapt route operation, closure, reroute, and status semantics onto `maps::MapGraph`; do not import actions/pluginlib. |
| Waypoint Follower | Keep native InspectionExecutor. Reuse only generic task-executor and failure-reporting ideas if a non-inspection server is later required. |
| Lifecycle Manager | Do not import. ProductControl already owns this responsibility through the real systemd and sim direct-child runners. |
| Collision Monitor | Adapt configurable polygon/velocity-polygon and TTC policy semantics. Preserve the LingTu swept-footprint core and single final writer. |
| Velocity Smoother | Adapted now as an original ROS-free portable core, following the public open/closed-loop and per-axis limit semantics. |
| Smoother Server | Add a LingTu `PathSmoother` contract first. Select an algorithm only after collision and kinematic requirements are explicit. |
| Docking | Adapt the staged lifecycle and smooth-control-law mathematics after dock hardware evidence and contracts exist. |
| Following | Reuse the same smooth-control-law family as docking, with target timeout and lost-target search, behind an independent LingTu Product. |

Before copying any upstream source, record the exact package license and commit.
The Navigation2 repository is mixed-license by package; the root license says
unmarked files are Apache-2.0, but each package must still be checked. No direct
source was copied into the velocity smoother added by this change.

## Command Ordering Contract

The product command order must be:

```text
autonomy / teleop intent
  -> VelocitySmoother
  -> final CollisionMonitor / CommandSafety
  -> unique /nav/cmd_vel writer
  -> Driver
```

Emergency stop, stale safety input, authority loss, and cancel use an immediate
zero command and bypass gradual smoothing. Putting a smoother after collision
checking would turn a stop into a deceleration ramp and is forbidden.

## Delivery Order

This file reports capability and evidence state; it does not own future work.
Velocity/path smoothing, dynamic-obstacle, following, and docking delivery order
lives in the [current roadmap](../plans/current-roadmap.md).

## Claims That Are Not Allowed Yet

Do not claim any of the following until the corresponding evidence is archived:

- "Dynamic people never leave residuals."
- "Velocity smoothing is active in the field command path."
- "Docking is supported."
- "The tracking Product continuously follows and reacquires a person in MuJoCo or the field" without archived dynamic evidence.
- "Map Server is fully native end to end" while a required control/query path
  still depends on the Python Host facade.
- "Product complete" based only on unit tests.
