# Navigation Capability Evidence Matrix

Status: current evidence ledger
Audited: 2026-07-28
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

## Current Capability Matrix

| Capability | Current implementation | Evidence | Verdict | Remaining product gap |
| --- | --- | --- | --- | --- |
| Map Server | Native map store, records, artifacts, active slots, map graph, save transactions, mapd realtime layers and scene publication under `src/maps`. | C, P, T; S partial; F pending | Strong partial | Persistent control/query still crosses a thin Python Host facade in some paths. The typed mapd control/query endpoint and field save/recovery evidence are not yet the only path. |
| Route Server | `maps::MapGraph` provides persisted nodes/edges and shortest routes; `src/nav/building` adds multi-map/floor orchestration. | C, T; P only in building flow | Partial | No independent typed route request/status/event lifecycle, route-operation plugin contract, traffic closure updates, or route progress telemetry. |
| Waypoint Follower | Native inspection store/executor and endpoint runtime support multi-point routes, dwell/actions, retries, skip/stop policies, pause/resume/cancel, revisions, and evidence. | C, P, T | Complete for inspection | It is not a generic waypoint server. GPS waypoints and arbitrary per-waypoint task plugins are separate future capabilities. |
| Lifecycle Manager | `ProductControl -> RunPlan -> systemd` owns apply, switch, stop, readiness, rollback, and immutable Product plans. | C, P, T; F partial | LingTu equivalent implemented | Do not import the ROS lifecycle manager. Continue crash/restart and release rollback evidence on S100P. |
| Collision Monitor | Native swept-footprint safety evaluates cloud/traversability and produces stop/slow/limited commands. Local planning also checks the full footprint. | C, P, T; S not currently accepted; F pending | Strong partial | The current safety chain needs a passing end-to-end MuJoCo report and field proof. A reusable multi-zone/TTC policy contract is still missing. |
| Velocity Smoother | Portable ROS-free C++ core in `nav_kernel::VelocitySmoother`: component limits, accel/decel, open/closed loop, timeout, deadband, proportional scaling, and hard-stop bypass. | C, T | Kernel implemented | It is deliberately not yet in the final command path. Product wiring, diagnostics, configuration, MuJoCo tuning, and field evidence remain. |
| Path Smoother | Some planners and path followers smooth implicitly or limit steering. | Internal behavior only | Missing as an independent capability | Add a typed, replaceable, collision-aware smoother contract between global planning and path activation. Never smooth through an obstacle or destroy planner kinematic constraints. |
| Docking / Charging | No formal Product, dock database, detector contract, final approach controller, or charging proof. | None | Missing | Requires hardware contact/charge evidence, a dock pose source, staged navigation, final control, retry/undock lifecycle, and a simulation fixture. |
| Target Following | Perception tracking/ReID, visual servo foundations, and `sim/following` exist. The current `tracking` Product tracks a map-frame goal, not a person/object. | C/T for foundations; no P | Partial foundation | Add a typed target observation, independent follow lifecycle, lost-target search, standoff controller, safety ownership, Product declaration, and acceptance suite. |

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
- `src/nav/cpp/endpoint/inspection/`
- `config/runtime_graph/products/inspection.yaml`

Calling this capability "missing" would be inaccurate. The accurate statement
is: generic waypoint execution is missing, while inspection waypoint execution
already exists.

### Lifecycle

LingTu deliberately replaces ROS node lifecycle with product/process lifecycle:

```text
ProductControl -> immutable RunPlan -> systemd
```

Blueprint remains active only inside the Python Host and is not the field
process lifecycle manager.

## Dynamic Obstacle And Residual Status

The algorithms exist:

- `src/maps/tests/cpp/voxel_layer_test.cpp`: column carving, adjacent-height
  isolation, voxel voting, and decay.
- `src/maps/tests/cpp/rolling_occupancy_test.cpp`: ray clearing, rolling
  bounds, independent time decay, and frame/order rejection.
- `src/maps/tests/cpp/mapd_engine_test.cpp`: exact observation pose/origin,
  epoch/sequence handling, floor isolation, resource caps, and timer-driven
  decay.
- `src/nav/cpp/tests/endpoint/test_motion_layer.cpp`: old obstacle residue
  cleared by later rays, current hit retention, static-cell protection,
  dynamic-track confirmation, and expiry.

The current Product-chain moving-person gate now exists:

- `sim/scripts/mujoco/teleop_avoid_native_acceptance.py` defines the optional
  `moving_person_clear` scenario.
- `sim/scripts/mujoco/native_dds_sensors.py` drives one deterministic MuJoCo
  mocap person through the live scan.
- `sim/scripts/mujoco/map_scene_roi_monitor.py` consumes the bounded native
  `MapScene` ABI and records only ROI counts for `live`, `voxel`, and
  `accumulated` clouds.
- `sim/tests/test_mujoco_dynamic_obstacle_scene.py` checks detection, clearing,
  persistent-residual failure, exact scene geometry, and map/control ownership.

This closes the old test-design gap, but it is not field evidence. A current
strict MuJoCo report and a long-running MID-360 report are still required.
The 2026-07-28 strict preflight did not start the scene because the selected
sensor publisher, SLAM, mapd, traversability, navd, navigation-control/client,
and cmd-velocity tap binaries were older than their current source/IDL closure.
That fail-closed result is a release-provenance blocker, not simulation
evidence and not proof that clearing passed or failed.
Therefore:

```text
column carving code: implemented
decay code: implemented
rolling obstacle layer: implemented
person residual product acceptance: gate implemented; passing run not yet archived
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

Use the existing runners as the starting point:

```bash
python sim/scripts/moving_obstacle_sweep_gate.py
python sim/scripts/mujoco/teleop_avoid_native_acceptance.py \
  --manifest config/runtime_graph/acceptance/mujoco_teleop_avoid_native_acceptance.json \
  --scenario moving_person_clear \
  --strict

python sim/scripts/mujoco/explore_native_acceptance.py \
  --manifest config/runtime_graph/acceptance/mujoco_explore_native_acceptance.json \
  --artifact-dir artifacts/mujoco-explore \
  --strict
```
Important: `moving_obstacle_sweep_gate.py --run-matrix` still launches
`launch_fastlio2_live.sh` and requires the legacy PCT inspection report shape.
It is an evidence aggregator and migration source, not an accepted current
`mapd/navd` Product gate. Use `moving_person_clear` for the current typed-DDS
chain. That scenario fails unless the person is observed, both voxel and
accumulated layers return near their pre-person baseline after the configured
decay grace, scene identity stays fixed, and generations remain monotonic.


A report is accepted only when it records exact source/binary provenance,
scenario metrics, cleanup/zero evidence, and no blockers.

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
| Lifecycle Manager | Do not import. ProductControl/systemd already owns this responsibility. |
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

## Delivery Plan

### P0: Evidence Before More Features

1. Keep the new velocity smoother as a tested core until endpoint integration
   has explicit authority-transition and hard-stop tests.
2. Make the portable MotionLayer residual test part of normal CTest.
3. Run the MuJoCo dynamic matrix and archive one provenance-complete report.
4. Run HeLiMOS and Dynamic Map Benchmark replay for filter quality.
5. Run the same residual metrics on MID-360 data.

### P1: Product Velocity Smoothing

1. Add Product configuration for x/y/yaw limits, open/closed loop, timeout,
   feedback age, deadband, and proportional scaling.
2. Keep one smoother state across autonomy/teleop authority transitions.
3. Run smoothing before final safety.
4. Publish limited/timed-out/feedback-age diagnostics.
5. Add MuJoCo step, reversal, stale-feedback, takeover, and emergency-stop
   acceptance.
6. Enable in a field Product only after the current unsmoothed chain remains
   available as an explicit rollback Product version.

### P2: Path Smoother

Define:

```text
PathSmoother::Smooth(path, map_generation, constraints, deadline)
  -> path | failure
```

The result must preserve start/goal, frame, map generation, collision clearance,
maximum curvature, and planner direction/cusp semantics. Failure retains the
original validated path or fails closed according to Product policy; it never
publishes an unchecked path.

### P3: Following Product

Add:

```text
TargetObservation -> FollowController -> VelocitySmoother
                  -> CommandSafety -> Driver
```

Required states: `IDLE, ACQUIRING, FOLLOWING, SEARCHING, PAUSED, SUCCESS,
FAILED, CANCELLED`. Required evidence includes ID continuity, target timeout,
occlusion recovery, standoff error, bystander rejection, and safety override.

### P4: Docking Product

Docking starts only after these contracts exist:

- versioned dock database and staging pose;
- detector/refiner output with covariance and freshness;
- charging/contact/stall feedback from hardware;
- collision-aware final approach;
- retry, cancel, undock, and manual recovery;
- MuJoCo dock fixture followed by real charger evidence.

## Claims That Are Not Allowed Yet

Do not claim any of the following until the corresponding evidence is archived:

- "Dynamic people never leave residuals."
- "Velocity smoothing is active in the field command path."
- "Docking is supported."
- "The tracking Product is target following."
- "Map Server is fully native end to end" while a required control/query path
  still depends on the Python Host facade.
- "Product complete" based only on unit tests.
