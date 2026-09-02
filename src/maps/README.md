# LingTu Maps

`src/maps` contains LingTu's live-map runtime and saved-map lifecycle. Native
C++ `mapd` is the sole map-management owner. Gateway keeps only a stateless
same-host UDS transport; Python does not own map state, save coordination,
artifact building, or activation.

## Start Here

| Question | Code |
| --- | --- |
| Where does live mapping run? | `cpp/mapd/` |
| Where is the incremental map updated? | `cpp/mapd/engine.cpp`, `cpp/layers/` |
| Who stores maps? | `cpp/store.cpp`, `cpp/service_lifecycle.cpp` |
| Who may activate a map? | ProductControl, through `cpp/mapd/activation.cpp` and the native service |
| How does Gateway call native maps? | Stateless `runtime/endpoints/mapd.py` UDS transport to `mapd` |
| How is a map saved and built? | `cpp/mapd/save_coordinator.cpp`, `cpp/save.cpp`, `cpp/build/` |
| Where is saved-map cleanup? | `prune/` |
| Where are the tests? | `tests/` |

## How It Runs

Live mapping:

```text
SLAM /slam/map_observation
  -> mapd DDS input
  -> LiveMapEngine
  -> voxel, occupancy, and scene state
  -> /maps/* DDS output
```

Saving a map:

```text
map save
  -> mapd save_map
  -> SlamMapSnapshotRequest over typed DDS
  -> slamd freezes the snapshot and returns SlamMapSnapshotAck
  -> SaveMapEngine builds and validates the candidate
  -> complete patch bundle automatically enters native PGO
  -> MapStore
```

Starting a Product with a saved map:

```text
ProductControl switch
  -> lingtu-mapctl stage
  -> mapd activation
  -> start the selected Product processes
```

The `map` Product starts LiDAR, IMU, SLAM, maps, navigation, driver, camera, and
Host because mapping needs those dependencies. The `maps` / `mapd` process owns
live map state and saved-map lifecycle.

## Responsibilities

- `mapd` owns realtime map state, saved-map management, and DDS map output.
- `MapStore` owns saved-map content identity and active-map state.
- `MapsServiceCore` owns native queries, lifecycle changes, and artifact builds.
- `SaveCoordinator` owns the native snapshot handoff and submits the accepted
  snapshot to `SaveMapEngine`. `begin_save_map` and
  `provide_save_map_snapshot` are internal C++ steps, not public actions.
- SLAM produces observations and snapshots; it does not own saved-map lifecycle.
- Navigation consumes map artifacts; it does not activate or mutate maps.
- ProductControl is the only public activation owner.
- Gateway and SDK clients use `mapd` for map data operations; ProductControl
  uses `lingtu-mapctl` for activation. None of them edits map files directly.

## Live Publication Timing

The field defaults separate control-critical local geometry from heavier map
products:

| Output | Default | Meaning |
| --- | ---: | --- |
| `/maps/state` | 2 Hz | Periodic process and map health. |
| `/maps/live_cloud`, `/maps/voxel_cloud`, `/maps/local_collision` | up to 10 Hz | Change-driven realtime outputs. Local collision follows this path so navigation does not wait for the low-rate artifact layers. Control Products publish only local collision data; the two visualization clouds remain empty compatibility samples. |
| `/maps/accumulated_cloud`, `/maps/occupancy`, `/maps/elevation`, `/maps/esdf` | up to 2 Hz | Change-driven, heavier map products. |
| `/maps/scene` | up to 2 Hz | Change-driven coherent snapshot. An unchanged generation is not retransmitted periodically. |

The engine follows the same split internally. Observation integration advances
the map generation without rebuilding every output. A realtime view builds only
the bounded voxel and local-collision snapshots; a complete view adds the
accumulated cloud, occupancy, elevation, and ESDF. Each view is cached once per
generation. The status snapshot reports `realtime_snapshot_generation`,
`complete_snapshot_generation`, `realtime_snapshot_builds`, and
`complete_snapshot_builds` so this behavior can be checked on the robot.

These values are publication limits, not proof of field frequency. The state
publication acts as a generation synchronization barrier and can advance a
pending output. Measure the actual wire rate and maximum sample gap with
`lingtu_dds_probe` on the target.

RunPlan also selects whether mapd maintains the extended live layers. The
`map` Product keeps voxel, accumulated, 2-D occupancy, elevation, and ESDF
layers for mapping inspection. Control-focused Products (`teleop_avoid`, `nav`,
`explore`, `tracking`, and `inspection`) keep the rolling 3-D occupancy needed for
`/maps/local_collision`, but skip the extended layer updates and publish empty
compatibility samples on the non-control layer channels, including the live and
voxel clouds embedded in `/maps/scene`. Saved-map artifacts
remain built by the SaveMap pipeline and are unaffected by this runtime choice.

The rolling 3-D occupancy keeps compact observed/occupied bit indexes. Collision
snapshots, decay, and window rolls therefore visit active cells instead of
rescanning the full grid. This changes neither the configured resolution nor
the collision AABB; the SCAN profile remains `200 x 200 x 100` at `0.05 m`.

The traversability process owns the separate terrain-risk grid used by control.
Its rolling exploration occupancy is constructed only by the `explore` Product;
other control Products do not spend CPU or memory publishing frontier snapshots
that have no consumer.

Map observation ingress intentionally keeps one latest pending sample. If the
worker is busy, a newer observation replaces the pending one so stale geometry
cannot build a backlog. `replaced_observations` is the overload signal: after
the sparse-map changes it should remain near zero under field load; growing
values call for profiling or input-rate reduction, not an unbounded queue.

`/maps/local_collision` is reliable but volatile, keeps only one sample, and
expires after 500 ms. `navd` also rejects collision geometry older than 500 ms.
This prevents a late-joining or stalled planner from treating replayed local
geometry as current.

Current `real` and `sim` Products use the native map runtime. The former Python
`MapsModule`, `MapdServiceClient`, and save pipeline have been removed.

## Names

| Name | Meaning |
| --- | --- |
| `map` | Product used to create a map. |
| `maps` | Logical native process. |
| `mapd` | Native live-map executable. |
| `lt-maps.service` | systemd unit that runs native `mapd`. |
| `lingtu_maps` | Native static library used by `mapd` and native tools. |
| `lingtu-mapctl` | Field operations client. |

User lifecycle commands stay simple:

```text
scripts/lingtu --robot unitree/go2 --env real switch map
scripts/lingtu --robot unitree/go2 --env real status
scripts/lingtu --robot unitree/go2 --env real stop
```

Save and list APIs are map-data operations, not a second Product lifecycle.

## Directory

```text
src/maps/
  include/lingtu/maps/   native public contracts
  cpp/mapd/              live process, DDS, activation, query socket
  cpp/layers/            realtime map layers
  cpp/build/             saved-map artifact builders
  cpp/store.cpp          saved-map persistence
  cpp/service.cpp        native queries and artifact validation
  cpp/service_places.cpp POI and map graph
  cpp/service_lifecycle.cpp map create/delete/rename/retire operations
  cpp/service_jobs.cpp   save, build, and maintenance operations
  cpp/save.cpp           save job
  prune/                 save-time dynamic-object cleanup
  tests/                 native C++ tests
```

Python does not live in this tree. The stateless UDS endpoint is
`src/runtime/endpoints/mapd.py`; semantic places live in
`src/memory/spatial/places.py`; reconstruction taxonomy lives in
`src/perception/reconstruction/taxonomy.py`.

Do not add another map store, active-map file, or map activation transaction in
Gateway, CLI, navigation, or simulation code. Extend the native service and use
the existing adapters.

## Saved Map

A saved map is a directory managed by `MapStore`. Its source point cloud and
derived planning artifacts are published together by the native save/build
workflow. Readers use service queries such as `get_record`, `get_bundle`,
and `validate_artifacts`; they do not construct artifact paths themselves.

C++ assigns every committed saved-map artifact set a positive numeric
`content_epoch`. The map-local `.content_epoch` file is the final commit marker;
the root `.content_epoch_counter` only allocates monotonic values. Neither file
is a map version, history, or active-map pointer. Gateway and DDS carry
the integer returned by C++ and do not parse or create `map:v...`/`map:e...`
identities. POIs, inter-map graph edges, and lifecycle state are separate
metadata and do not advance the saved-artifact epoch.

Artifact builds keep a small native transaction manifest while publishing. A
dead writer is recovered before the stale build lock is removed: incomplete
publishes roll back, while a committed epoch keeps the new artifacts. Ordinary
readers stay fail-closed while that recovery is pending.

The canonical control action names live in `cpp/mapd/service_dispatch.cpp`. Add
one name for a new operation; do not add spelling aliases or a Python command
router.

## Build And Test

```powershell
cmake -S src\maps -B build\maps -DLINGTU_MAPS_BUILD_TESTS=ON
cmake --build build\maps --config Release
ctest --test-dir build\maps -C Release --output-on-failure

.venv\Scripts\python.exe -m pytest tests\runtime\test_mapd_endpoint.py -q
```

## Remaining Work

- A full runtime test still needs to prove observation -> live scene -> save ->
  activation -> reload. Unit tests do not replace that evidence.
