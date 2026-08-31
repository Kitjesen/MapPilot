# Native Data Plane Migration

Status: native navigation and realtime `mapd` data planes implemented; persistent
map control migration and S100P field acceptance remain open

## Target

```text
C++ SLAM -> typed DDS -> C++ Maps / C++ Nav -> typed DDS -> Driver
                                  |
                               HostBus
                                  |
                       Gateway / Agent / MCP
```

Blueprint owns the in-process Module graph of each Python Host, including the
Host declared by the `map` Field Product. It does not own native processes, DDS
topology, navigation algorithms, or high-rate map computation.

## Ownership

| Surface | Product owner |
| --- | --- |
| Navigation execution and final `/nav/cmd_vel` | C++ `navd` |
| Realtime map layers and `/maps/scene` | C++ `mapd` |
| Persistent maps | C++ `MapsServiceCore` and `MapStore`; typed `mapd` control/query is still pending |
| Control risk grid | C++ traversability endpoint |
| Native state entering the Host | one `HostBus` |
| Gateway, Agent, MCP | Python Host |

Field products fail closed when `HostBus` or its native ABI is unavailable.
There is no Python navigation-state fallback at Product startup. Development
and simulation Blueprints may retain Python implementations of the same contracts.

## Native Navigation Batch

### Completed

1. `messages.idl` defines `MapObservation` and `NavigationState`.
2. C++ SLAM publishes `/slam/map_observation` once per accepted scan.
3. C++ `navd` publishes authoritative `/nav/state`.
4. The persistent native navigation client exposes the latest state through a
   capability-gated C ABI.
5. `HostBus` projects native state into the Host graph, and Gateway uses it
   instead of Python `mission_status` whenever native state is present.
6. Field Product declarations require `/nav/state`; products with SLAM also
   require `/slam/map_observation`.
7. `host.bus` is critical in every field Product. Missing native support,
   missing state, or a failed poll blocks Host readiness.
8. Occupancy, voxel, elevation, and semantic consumers reject an old epoch,
   duplicate sequence, and out-of-order sequence.
9. Observation-capable map layers receive `MapObservation` only. The old
   parallel `map_cloud` feed to those layers was removed.
10. `NavigationGoalStatus` is a separate per-request lifecycle contract. It
    contains `boot_id`, a monotonic event cursor, `request_id`, state,
    `goal_epoch`, and terminal reason.
11. The native client retains the latest result per request and exposes goal
    status through C++ and C ABI queues with bounded memory and restart-aware
    deduplication.
12. `HostBus` publishes native `NavigationState`, `NavigationGoalStatus`,
    global path, and local path to Gateway, MCP, Agent, and `NavSkills`.
13. Gateway, MCP, Agent, and `NavSkills` use `/nav/state` for the current
    system snapshot and `/nav/goal/status` for a specific request result.
14. Native global/local path telemetry uses a two-stage C ABI copy. A sample
    is retained when the caller's buffer is too small; paths are never
    silently truncated.
15. Field and simulation Products use the native navigation endpoint. The Host
    retains command, goal, skill, and status adapters rather than a second
    planner and motion-control implementation.

OMG IDL reserves `sequence`. Wire members therefore use
`state_sequence`, `event_sequence`, and `observation_sequence`; C++ and Host
APIs normalize these fields to `sequence`.

### MapObservation invariants

Each observation carries one accepted incremental scan and its exact scan-time
geometry:

```text
header timestamp
reset_epoch + sequence
sensor_frame + map_frame
map <- sensor transform
sensor origin in map frame
incremental PointCloud2 scan
pose quality
```

Consumers must:

- accept a larger `reset_epoch`;
- accept only increasing `sequence` within an epoch;
- clear epoch-bound obstacle state when the epoch advances;
- never combine the observation scan with latest odometry.

The native SLAM process creates a restart-safe epoch from wall-clock time plus
a process-local monotonic counter. A SLAM reset advances it.

### NavigationState invariants

`NavigationState` is deliberately compact:

```text
header / boot_id / sequence / control_mode
lifecycle_state / active_request_id / goal_epoch
map_id / map_content_epoch
planning_state / execution_state / recovery_state
progress / authority / hold_reason / failure_code
```

Timing, input age, loop performance, and DDS statistics belong in
`NavigationDiagnostics`, not this lifecycle message.

## Native Maps Batch

### Completed

1. C++ `mapd` consumes typed `/slam/map_observation` directly. It uses the
   observation's scan-time transform and sensor origin and rejects duplicate,
   old-epoch, out-of-order, malformed, or unsafe-pose samples.
2. One process owns the realtime voxel, accumulated block grid, occupancy,
   elevation, ESDF, traversability scene layer, decay timer, and scene
   snapshot. These map layers are not instantiated as Python Modules in field
   Products.
3. `mapd` publishes `/maps/state`, live/voxel/accumulated clouds, four grid
   layers, and one coherent `/maps/scene`.
4. Publication cursors advance only after a successful DDS write. Failed
   generations remain pending for retry, and readiness requires successful
   state, cloud, grid, and scene publication for the current generation.
5. Input point count, serialized cloud bytes, fields, point step, strings,
   scene bytes, voxel snapshots, live voxel storage, and accumulated block
   storage have explicit product limits. Capacity rejection is observable and
   blocks readiness.
6. Persistent block-grid ray insertion preflights cell and block capacity per
   ray. A rejected ray publishes neither free-space misses nor its occupied
   hit, preventing partial free-space corruption.
7. Decay runs from an independent timer after input stops. Column carving uses
   a sensor-relative height band and has an adjacent-floor regression test.
8. The native navigation client exposes `MapScene` with an append-only,
   capability-gated C ABI. Its two-phase copy retains a sample on insufficient
   consumer capacity and applies client-side point, grid, and total-byte
   limits before Python allocation.
9. `HostBus` independently tracks map state and scene cursors, freshness,
   producer boot, epoch, observation sequence, generation, invalid samples,
   and capacity rejection. Field readiness requires aligned current
   `/maps/state` and `/maps/scene`.
10. Gateway receives `MapSceneFrame` from `HostBus`. Map readiness failures
    block `ready`, `motion_ready`, and `data_ready`; they are not merely
    diagnostic details.

## Product Contract

| Topic | Declared writer | Current consumer |
| --- | --- | --- |
| `/slam/map_observation` | native SLAM runtime | C++ `mapd` |
| `/maps/state` | C++ `mapd` | native client -> `HostBus` readiness |
| `/maps/live_cloud` | C++ `mapd` | native/map diagnostics |
| `/maps/voxel_cloud` | C++ `mapd` | native/map diagnostics |
| `/maps/accumulated_cloud` | C++ `mapd` | native/map diagnostics |
| `/maps/occupancy` | C++ `mapd` | map scene/query consumers |
| `/maps/elevation` | C++ `mapd` | map scene/query consumers |
| `/maps/esdf` | C++ `mapd` | map scene/query consumers |
| `/maps/scene` | C++ `mapd` | native client -> `HostBus` -> Gateway |
| `/nav/state` | native nav runtime | persistent native client -> `HostBus` |
| `/nav/goal/status` | native nav runtime | persistent native client -> `HostBus` |
| `/nav/global_path` | native nav runtime | persistent native client -> `HostBus` -> Gateway |
| `/nav/local_path` | native nav runtime | persistent native client -> `HostBus` -> Gateway |
| `/nav/cmd_vel` | native nav runtime | native driver |
| `/nav/traversability` | native traversability endpoint | native nav runtime |

The Runtime Graph, typed DDS endpoint contract, robot route, topic registry,
QoS registry, and Product declarations all contain the two new topics. This is a
declared ownership baseline. Runtime discovery of actual DDS writers remains a
field acceptance gate and must not be inferred from YAML alone.

## Transitional State

The realtime map data plane exists, but the full map product is not complete:

- Persistent map control/query/event APIs are still exposed through the Host
  map-service boundary over C++ `MapsServiceCore`; `mapd` does not yet own that
  typed endpoint.
- SaveMap still needs one field-verified snapshot barrier from SLAM through
  staging, dynamic cleanup, artifact build, validation, and atomic
  `MapRecord` publication.
- Gateway keeps development compatibility paths for legacy map streams and
  map-service HTTP conversion. Field Products must use `HostBus` MapScene and
  may not instantiate Python high-rate map layers.
- Point-cloud and scene IDL sequences are currently application-bounded rather
  than IDL-bounded. Readers reject oversized products before application
  allocation, but DDS deserialization allocation remains a transport-level
  hardening item.
- Field visual-servo velocity output does not yet have a native
  operator-motion bridge. Far-goal submission uses the typed goal service;
  near-goal direct velocity remains development-only until that authority
  contract is added.

No document or readiness response may claim S100P product acceptance until
writer discovery, fault injection, replay, and performance gates pass on the
target.

## Verification

The current batches are covered by:

- IDL/codegen and QoS tests;
- Runtime Graph, endpoint, route, Product compile, and Blueprint tests;
- generated IDL plus real CycloneDDS C++ navigation state, goal-status, and
  path/MapScene telemetry tests under WSL;
- C++ SLAM epoch/sequence contract tests;
- C++ mapd engine, DDS, local service endpoint, storage-capacity, atomic-ray, decay,
  cross-floor carving, and scene publication tests;
- Python HostBus, Gateway readiness, Product compilation, and scene projection
  tests;
- map observation transform, exact-origin, dedupe, epoch reset, and wiring
  tests.

The focused mapd DDS/service and native client MapScene tests build and pass
under WSL/CycloneDDS. Actual S100P process startup, writer discovery, load,
MID-360 replay, and failure injection remain field acceptance gates.

## Next Gates

1. Move persistent map control/query/event ownership behind the typed C++
   `mapd` endpoint; keep Gateway as an HTTP/Host adapter only.
2. Complete and field-verify the SaveMap snapshot barrier and transaction.
3. Remove remaining field compatibility reads of legacy map streams and direct
   map files; retain them only in explicit development profiles.
4. Bound large sequences in generated transport contracts, or prove equivalent
   CycloneDDS resource limits before deployment.
5. Build generated IDL, SLAM, `navd`, `mapd`, native client, and driver
   together on S100P; discover actual DDS writers and verify QoS.
6. Run MID-360 replay and S100P performance gates before declaring the native
   data plane product-ready.
