# LingTu Maps

`src/maps` contains LingTu's live-map runtime and saved-map lifecycle. Native
C++ `mapd` is the sole map-management owner. Gateway keeps only a stateless
same-host UDS transport; Python does not own map state, save coordination,
artifact building, or activation.

## Start Here

| Question | Code |
| --- | --- |
| Where is the public C++ header catalogue? | [`include/lingtu/maps/README.md`](include/lingtu/maps/README.md) |
| Where does live mapping run? | `cpp/mapd/` |
| Where is the incremental map updated? | `cpp/mapd/engine.cpp`, `cpp/layers/` |
| Who stores maps? | `cpp/storage/store.cpp`, `cpp/service_lifecycle.cpp` |
| Who may activate a map? | ProductControl, through `cpp/mapd/activation.cpp` and the native service |
| How does Gateway call native maps? | Stateless `runtime/endpoints/mapd.py` UDS transport to `mapd` |
| How is a map saved and built? | `cpp/mapd/save.cpp`, `cpp/save.cpp`, `cpp/build/`, and header algorithms under `include/lingtu/maps/octomap/` |
| Where is saved-map cleanup? | `prune/` |
| Where are the native maps tests? | `../../tests/maps/cpp/` |

## How It Runs

Product map previews use the existing native artifact endpoint: `mapd`
`open_artifact(source_pointcloud)` → Gateway `/api/v1/maps/{name}/pcd` → React
Maps. Gateway forwards the descriptor's map ID, content epoch, and frame in
HTTP headers; previewing does not activate that map. `/map/viewer` redirects
to the product page. Display styles and recording-only export configuration
belong to the UI/tools, not to map save or navigation activation.

Live mapping:

```text
SLAM /slam/map_observation
  -> mapd DDS input
  -> LiveMapEngine
  -> voxel, occupancy, and scene state
  -> /maps/* DDS output
```

`MapCollisionLayer` carries three packed 3D bitmaps with one geometry and
generation: inflated collision, measured occupancy, and known free space.
When neither measured bit is set, the cell is unknown. Evidence-only changes
advance generation even if inflation is unchanged. SCAN queries the inflated
bitmap for body collision: a clear collision query does not assert observed
free space or ground support. OctoPlanner checks the autonomous route's 3D
support and unknown-space evidence in the saved map; the display surface
projection does not authorize motion. See the [local collision contract](../nav/cpp/planning/local/scan/README.md).
Rebuild/deploy mapd and navd together when changing this DDS type.

Saving a map:

```text
map save
  -> mapd save_map
  -> SlamMapSnapshotRequest over typed DDS
  -> slamd freezes the snapshot and returns SlamMapSnapshotAck
  -> SaveMapEngine copies and validates the frozen source
  -> complete patch bundle enters native PGO (or records why it was skipped)
  -> calibrated visibility cleanup and source voxel filtering
  -> build the 3D OctoMap required by saved-map navigation
  -> validate the required artifacts
  -> transactionally commit the complete candidate to MapStore
```

The 2D occupancy projection, ESDF, and traversability are optional derived
artifacts, omitted from the default save and `build_navigation_package` paths.
The field 3D navigation chain does not consume them. An explicit occupancy
request still builds the projection; requesting ESDF or traversability also
builds their required projection. Failed optional work only blocks a save
when that work was requested.

Map list and health queries check metadata, artifact presence, and the full
OctoMap file header without parsing large point-cloud or tree payloads. They
reuse the activation summary under the existing map lock. Actual activation
continues to read and validate payloads; a positive list summary is not a
substitute for successful activation.

Observations acquired after the snapshot belong to a later save. Saving does
not switch Products, activate the saved map, or publish a navigation goal.
Successful saving under an existing name replaces that map; failed commits
restore the previous version, but successful commits do not retain a history
copy. Use a new name when preserving an existing field map.

The public save-operation response exposes `operation.processing` with
separate optimization and cleanup outcomes. `SUCCEEDED` means the requested
files were saved and validated. It does not imply that PGO was performed or
that localization, footprint clearance, and a route have passed. A skipped
optimization is reported as such, while the source map may still be saved.

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

### Saved point-cloud subset edits

The Web editor submits ordered inclusive XYZ boxes to native
`save_edited_source` through Gateway's `POST /api/v1/maps/{name}/save_as`.
`delete_inside` removes points in a box; `keep_inside` removes points outside it.
mapd checks the source content epoch under its map lock and transactionally saves
the complete filtered XYZ cloud under a new name. The source stays unchanged;
retained points keep their coordinates without downsampling.

An empty ordered edit list copies the source unchanged for resolution comparison.
New mapping sessions save a 10 cm OctoMap by default. The Web map-status panel
offers 5, 6, 10 and 20 cm for a named copy, with 10 cm preselected. It
then rebuilds the actual OctoMap using the copied point cloud and measured rays.
The source map stays intact and the target is loaded through normal ProductControl
activation. This does not change the independent live local collision resolution.

Only a complete measured-ray bundle is copied unchanged: poses, patches,
`patch_bundle.manifest`, and `scan_origin.txt`. Incomplete evidence and old derived
artifacts are not copied. The new source starts invalidated; a separate OctoMap
build uses the returned `build_options`. Rebuild failure leaves the saved source
available for retry. The saved-ray builder still checks hits against retained
geometry, so deleting points does not invent free space beyond measured returns.

An identical lost-response retry can confirm the saved target while it remains
invalidated with reason `edited_source`. The source identity/epoch and ordered
operations must match; an unrelated existing target remains a conflict. Saving
and rebuilding do not activate a map or authorize motion. See the
[API contract](../../docs/api.md#saved-point-cloud-subset-editing).

### Saved ground and obstacle projection

`occupancy.npz` / `map.pgm` use 0.20 m cells. The exporter fits observed local
lower surfaces over 0.60 m neighborhoods using the shared C++ ground estimator,
then marks returns 0.10–2.00 m above that local plane as occupied. It does not
use one whole-map height percentile: that would classify a higher floor or a
ramp as an obstacle. Insufficient or non-planar support stays unknown. A local
surface fit alone cannot establish that a tabletop is walkable ground.

This projection has no sensor rays and therefore does not declare free cells.
It is distinct from the native 3D OctoMap used for navigation; producing valid
files does not prove localization, body clearance, or footprint support.

The navigation OctoMap records its build evidence for diagnostics:

- When a save keeps `poses.txt`, `scan_origin.txt`, and `patches/`, the builder
  replays each saved scan from its sensor origin. An endpoint matches retained
  geometry when its voxel remains in the cleaned `map.pcd`, or when a retained
  point is within 5 cm in 3D. The metric fallback uses the same confirmation
  distance as prune, avoiding a lost hit when a sampled surface and its measured
  return fall on opposite sides of a voxel boundary. Hits are written at the
  measured endpoint, never at an unobserved sampled representative.
  Removed endpoints still contribute their measured free prefix, but no hit or
  inferred free space at or beyond the endpoint. Within a scan, updates are
  deduplicated and hits win over misses. Voxels keep the hit/miss log-odds their
  rays accumulated; space no ray reached stays unknown.
  The last metre of a ray does not lower a retained voxel, or a cell
  beside one in the same layer: near its endpoint a ray grazing a floor stays
  within one voxel of the surface, and OctoMap would mark those surface cells
  (and 5 cm holes in the sampled floor) free although the ray never passed
  below the surface; the planner reads such a free cell as a drop. Beyond this
  near-end guard, retained voxels already hit by saved scans can use a local
  measured surface: a miss is skipped only when the ray segment inside that
  voxel stays wholly on one side, outside the surface tolerance. Actual
  crossings and uncertain fits still clear the voxel. Fits select the surface
  near the measured hit height, preserving distinct floors at the same XY.
  Every occupied voxel is a
  measured hit; a guarded cell no endpoint reached stays unknown. Evidence
  counts and fixed-input offline results are recorded in
  [the 2026-09-27 validation record](../../docs/operations/maps-evidence-20260927.md).
  An offline route is not field-motion acceptance.
- Without those files the builder marks occupied voxels from sampled points and
  adds the configured free envelope above supports. The metadata labels that
  source as a preview, but it is still a usable OctoMap artifact; route
  feasibility is decided by the 3D planner rather than by the source label.
- `octomap.ot` is written with OctoMap's full encoding, never `writeBinary()`,
  which would reduce every voxel to occupied/free. `metadata.json` records
  `artifacts.octomap.encoding: full_log_odds`, the `sensor_model` the build used
  (OctoMap's defaults, which is what every reader applies), and
  `navigation_ready`: `true` only for a saved-ray build, as a diagnostic
  provenance field. Reuse requires `full_log_odds`, the current builder version
  (`0.3.2`), and matching build inputs; saved-ray statistics are checked when
  the source actually contains saved rays. The version change rebuilds old
  ray artifacts without the local surface guard; it does not rerun cleanup or restore missing source
  geometry.
- The build claims saved rays only when the scans are complete: `poses.txt`
  and `scan_origin.txt` are present and, when SLAM wrote a
  `patch_bundle.manifest`, no patch was dropped. A replay missing scans would
  leave their surfaces without ray evidence. A complete bundle covers selected
  keyframes, not every incoming scan. The fused PCD can contain surfaces absent
  from those keyframes; PCD-only points do not become saved-ray occupancy.
  The [dataset diagnosis](../../build/dataset-planning-fix-20261021/README.md)
  records this coverage gap and the grazing-ray fix. That direct replay-save
  experiment did not run the Product SaveService prune phase.
- MapStore activation checks the map lifecycle, metadata identity, and a valid
  full `octomap.ot` artifact (`full_log_odds`). It does not reject a map because
  its provenance is `sampled_points` or because `navigation_ready` is false.
  `OctoPlanner3D` decides whether the 3D geometry admits a route, and SCAN
  performs the live collision check before motion. A point-cloud source may
  still fail planning when it contains unknown space; that is a planner result,
  not a separate evidence gate. `list_maps` keeps `map_evidence` as a
  diagnostic label: `saved_rays`, `preview`, or `none`.
- PCD import reads `ascii`, `binary`, and PCL `binary_compressed` (LZF) data.
- A voxel edit (`edit_octomap_voxels`, the workbench zone marker) sets each
  voxel in the zone to the clamped occupied or free value, keeps the evidence of
  every other voxel, and reports `changed_voxels` next to `edited_voxels`.
  `occupied`/`preblocked` mark occupied; `free`/`traversable`/`clear` mark free.
  An edit keeps the artifact's evidence provenance.
  Shapes are a 3D `sphere` or an axis-aligned cube (`box`), both using a
  center and one radius; independent wall length, thickness, and height are
  not supported. `clear` writes known free space; it does not remove an edit
  and restore the original sensor evidence.
- Voxel edits persist in the saved OctoMap and `voxel_edits.jsonl`. Reopening
  that artifact retains them, but rebuilding from PCD or saved rays does not
  replay the journal and replaces their occupancy effect. Some source-edit
  operations also remove the journal with other derived artifacts.
- Editing a saved OctoMap does not refresh a running navigation process's
  private map snapshot or inject the edit into mapd's live collision bitmap.
  Global planning can consume the edited artifact after loading it; local
  SCAN and final braking checks still consume the live bitmap. These edits
  are therefore not complete glass-wall collision support. The remaining
  work is tracked in the [map-editing queue](../../docs/roadmap.md#地图编辑与人工障碍约束2026-10-21).

Deploy `mapd`, `lingtu-mapctl`, and `prune` together. The save transaction must
preserve `scan_origin.txt`, poses, patches, and `map.before_clean.pcd` for calibrated
visibility cleanup and later reprocessing. Preserve an existing map while
evaluating a cleaned copy; never fill missing floor support just to pass a path
check. If automatic PGO is skipped, inspect `map_optimization.json` rather than
interpreting a successful save as a successful global optimization.

The [map file and save-task guide](../../docs/operations/map-files.md) defines
the permanent artifacts, legacy `map.pcd.preclean` compatibility, failed-save
evidence and retry behavior. Standalone prune still emits `map.clean.pcd` for
inspection; new formal map packages keep `map.pcd` as the current point cloud.

## Responsibilities

SaveMap first durably archives the SLAM snapshot. `auto_process` defaults to true;
false leaves the operation `CAPTURED` until `process_save_map`. `source_saved`
is derived from the accepted receipt and retained source, not from the final map.
Capture archives survive processing success, failure and cancellation. Processing
works on a copy and keeps the existing final-map commit/rollback boundary.

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

The low-rate mapd query protocol and Python client allow JSON responses up to
4 MiB. SceneView requests up to 30,000 points through
`/api/v1/maps/{name}/points`, bound to the active map and live scene identity.
Oversized replies fail with `response_too_large`; a smaller custom
`--query-max-json-bytes` can reject an otherwise valid preview.

MapView reads the original PCD through `/api/v1/maps/{name}/pcd` and samples
it in the browser: up to 500,000 points for binary PCD or 300,000 for ASCII.
This artifact stream is independent of the JSON response limit. Neither
display sampling nor the query limit changes live DDS publication or saved
map contents.

The SDK writes PCD downloads to a temporary file and verifies the advertised
HTTP byte count before replacing the destination. An early EOF, including an
empty interrupted response, preserves the previous map and removes the
partial file. Cancelling an artifact iterator before its first read also
releases its descriptor when the iterator and artifact are discarded.

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

The accumulated cloud integrates sensor rays. A new hit at the same XY does
not erase surfaces at other heights: ray misses lower the cells they traverse,
and the existing configured decay still applies. This inspection layer does
not supply the saved map or the local collision bitmap.

The rolling 3-D occupancy keeps compact observed/occupied bit indexes. Collision
snapshots and window rolls therefore visit active cells instead of rescanning
the full grid. Generic map profiles may also enable time-based decay. The SCAN
profile disables it to match upstream GridMap semantics: cells clear through
ray misses or rolling-window eviction. Its grid remains `200 x 200 x 100` at
`0.05 m`.

The traversability process owns the separate terrain-risk grid used by control.
Its rolling exploration occupancy is constructed only by the `explore` Product;
other control Products do not spend CPU or memory publishing frontier snapshots
that have no consumer.

Map observation ingress intentionally keeps one latest pending sample. If the
worker is busy, a newer observation replaces the pending one so stale geometry
cannot build a backlog. `replaced_observations` is the overload signal: after
the sparse-map changes it should remain near zero under field load; growing
values call for profiling or input-rate reduction, not an unbounded queue.
Only `MAPPING` and `TRACKING` observations enter this queue. `LOCALIZING`,
`DEGRADED`, and other pose states are rejected at submission, before they can
replace a pending usable observation.

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
  include/lingtu/maps/   native public contracts; see its README.md catalogue
  cpp/mapd/              live process, DDS, activation, save handoff, UDS queries
  cpp/layers/            realtime 3D layers, local and exploration projections
  cpp/build/             saved-map build orchestration and grid artifacts
  cpp/io/                JSON, PCD, saved scans, and patch-bundle I/O
  cpp/storage/           saved-map persistence, locks, health, maintenance, save transaction
  cpp/semantic/          semantic occupancy, taxonomy, persistence, and Unity import
  cpp/graph.cpp          persisted POI/portal graph and routing
  cpp/service.cpp        native queries and artifact validation
  cpp/service_places.cpp POI and map graph
  cpp/service_lifecycle.cpp map create/delete/rename/retire operations
  cpp/service_jobs.cpp   save, build, and maintenance operations
  prune/                 save-time dynamic-object cleanup
  ../../tests/maps/cpp/  native maps tests (repository-level)
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

The canonical control action names live in `cpp/mapd/dispatch.cpp`. Add
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
