# LingTu Maps

`src/maps` is the canonical home for LingTu's map domain. It is not a navigation
helper folder. It owns spatial products used by localization, planning, safety,
Gateway visualization, saved-map lifecycle, and future multi-map switching.

This package is C++ first. Python owns the current Module/Blueprint shell and
application-service orchestration; realtime map math, map storage, and artifact
transactions stay in C++.

## Design Inputs

The folder layout follows three proven patterns:

- Nav2 costmaps: a map product is assembled from multiple layers, including
  static, obstacle, voxel, inflation, and filter layers.
- Open3D VoxelBlockGrid: 3D maps should prefer sparse global allocation with
  locally dense blocks/SoA-friendly memory instead of a scattered voxel hash for
  long-lived high-rate mapping.
- Autoware map components: maps are system assets, not planner internals. A map
  domain loads and serves point cloud / vector representations to downstream
  consumers.

References:

- https://github.com/ros-navigation/navigation2
- https://github.com/isl-org/Open3D
- https://github.com/autowarefoundation/autoware_core

## Non Goals

- No ROS dependency in core maps code.
- No Gateway, CLI, or web UI implementation here.
- No mission FSM or planner ownership here.
- No direct hardware driver ownership here.
- No Python implementation for realtime point-cloud accumulation, ESDF, or
  traversability once the migration is complete.

## Current Layout

```text
src/maps/
  include/lingtu/maps/       C++ model, service, store, layer, and build contracts
  cpp/                       C++ store/service/C ABI implementations
  cpp/layers/                Voxel and semantic-occupancy implementations
  cpp/build/                 PCD, occupancy, grid artifact, and build transaction code
  adapters/python/           ctypes adapters for the Python Module shell
  adapters/dds/              typed DDS map-output adapter
  adapters/native/           ROS-free SLAM map-save integration
  adapters/ros2/             explicit compatibility-only map-save adapter
  services/                  Persistent map query/control/build application services
  modules/                   Runtime map layer and maps.service Module adapters
  map_save.py                Map-save adapter contract
  artifacts.py               Same-source metadata/hash validation
  paths.py                   Map roots, exchange roots, and active-map resolution
  pcd.py                     Read-only PCD preview parser
  tests/                     C++, Python, and architecture contract tests
```

## Runtime Names

Use short runtime aliases. Class names and file names should not encode legacy
implementation details.

| Alias | Role |
| --- | --- |
| `maps.service` | Persistent map query/control/build facade. |
| `maps.voxel` | Live 3D voxel/column-carving layer. |
| `maps.occupancy` | Live 2D occupancy/exploration grid. |
| `maps.elevation` | Live elevation statistics layer. |
| `maps.esdf` | Distance-field layer. |
| `maps.traversability` | Fused terrain/obstacle risk layer. |

The runtime graph must use `maps.service`; `nav.maps` is retired because the
map lifecycle is not owned by navigation.

## Runtime Shell Boundary

The current LingTu runtime still assembles products with Python `Blueprint` and
Python `Module` classes. That is an orchestration shell, not the map core.

For the map domain, the intended split is:

```text
Python Blueprint / MapsModule
  -> ctypes C ABI
  -> C++ MapsServiceCore / MapStore / layer kernels
```

Python is allowed to own:

- port subscription and publication in the existing Module runtime;
- JSON command parsing while Gateway/CLI still talk to Module ports;
- source-snapshot requests to the currently selected SLAM runtime;
- transport-envelope packing for native command/query responses.

Python must not own:

- realtime voxel/occupancy/elevation/ESDF/traversability math;
- map id validation, active-map state, artifact scanning, health gate;
- long-term MapRecord/ArtifactIndex semantics.

`lingtu_maps` is a runtime requirement. `MapStorageService` does not fall back
to a Python filesystem implementation when the native library is absent;
startup fails with a clear error instead. This keeps development, CI, and robot
deployments on the same map semantics.

## Current Implementation Status

Implemented in C++ now:

- `MapStore`: map id validation, map directory lifecycle, artifact scan,
  artifact SHA256 hash, active-map state, and basic health/capability
  reporting.
- `MapsServiceCore`: native query/control facade for list, create, delete,
  rename, retire, set active, clear active, record, health, validation, and
  bundle queries.
- `MapPipelineCore`: build lifecycle state for artifact work, including
  begin-build, finish-build, latest build status, and concurrent build lock.
- Native PCD import/crop: ASCII/binary XYZ PCD read, bounds crop, invert crop,
  voxel dedupe, derived-artifact invalidation, backup, and binary PCD write.
- Native source-map mutation transaction: import/crop/save-source publish
  `map.pcd` through a staged source transaction, invalidate stale derived
  artifacts, and keep old artifacts visible until the replacement source cloud
  commits. The runtime save-source path writes SLAM/live snapshot output into a
  staging directory, then C++ owns the dynamic-filter subprocess, optimizer
  subprocess, PCD reload/filter, auxiliary artifact copy (`poses.txt`,
  `trajectory.txt`, `patches/`, cleaner outputs, `map_optimization.json`), and
  final publish.
- Native source-backup restore: `map.pcd.preclean` / `map.pcd.predufo` is
  validated and staged; active navigation is cleared before commit, and both
  artifacts and active state are rolled back if publication fails.
- Native occupancy snapshot builder: saved-map PCD projection, z filtering,
  grid generation, staging writes, and committed `occupancy.npz` / `map.pgm` /
  `map.yaml` output.
- Stable C ABI plus ctypes adapter for the current Python Module shell.
- Native voxel/column-carving data layer for realtime scene/map accumulation.
- Native semantic occupancy layer with bounded log-odds evidence, 3D ray
  free-space updates, geometry covariance, dominant semantic class, local
  radius query limits, frame/time/taxonomy validation, full-map replacement,
  returned partial-update diagnostics, and deterministic memory limiting. This
  is the map-side SOCC-ICP adoption; ICP pose estimation stays in localization.
- Immutable `SemanticMapChunk` SoA snapshots expose generation-checked local
  queries without exposing the internal voxel hash table. The stable semantic
  C ABI supports update, metadata, count/fill query, snapshot, validate,
  atomic save, and open/load of arbitrary-voxel-size artifacts.
- `semantic_map.bin` is a versioned native artifact with frame, taxonomy,
  generation, deterministic SoA payload, size checks, and checksum validation.
  The live semantic module saves it through a typed request/result handshake;
  map save then validates and catalogs it through `MapsServiceCore`.
- Unity simulation truth imports are native and transactional. C++ parses the
  real `environment/Categories.csv` and `object_list.txt` formats, resolves
  labels through `config/semantic_taxonomy.json`, excludes dynamic classes by
  default, voxelizes oriented object bounds, and publishes a validated
  `semantic_map.bin` only after the complete import succeeds. The stable C ABI
  and typed `import_unity_semantics` control action expose the same operation;
  Python does not parse or voxelize the scene.
- A semantic artifact is query/localization data only. It is deliberately
  excluded from the navigation-ready artifact gate.
- `MapSceneFrame` + `/maps/scene` product contract: the voxel layer now
  publishes a layered scene frame, and Gateway consumes it as the canonical
  clean live-map view while retaining `voxel_cloud` only as compatibility.
- Native ESDF and traversability artifact builders from `occupancy.npz`.
- Native OctoMap artifact path: C++ owns the build contract, timeout, error
  classification, and transaction boundary. When LingTu Maps is built with
  OctoMap development libraries (`LINGTU_MAPS_ENABLE_NATIVE_OCTOMAP=ON` and
  `find_package(octomap)` succeeds), build mode `native_octomap` embeds a real
  `octomap::OcTree` writer. Builds without OctoMap keep the external
  OctoPlanner3D/OctoMap converter as an explicit compatibility mode and report
  `native_octomap_unavailable` instead of emitting fake `.ot` artifacts.
- Native manual OctoMap editing: embedded OctoMap builds edit voxels directly;
  other builds use the C++ process boundary. `octomap`, `metadata.json`, and
  `voxel_edits.jsonl` publish as one transaction with failure injection tests.
- Native navigation-package transaction wrapper: C++ builds occupancy, ESDF,
  traversability, OctoMap, and metadata inside
  `.builds/<build_id>_transaction/staging_map`; only after every step succeeds
  does it publish the package to the real map directory. Failed builds do not
  expose new half-ready navigation artifacts.
- Native POI, map graph, rollback, multi-active, lifecycle, and artifact-job contracts:
  the C++ service owns file-backed POI records, graph edges, active-map history,
  named active slots, `RETIRED` state, and the persistent artifact-job journal.
  The native worker owns queue consumption, lease/heartbeat, cancellation,
  retry, crash recovery, and progress. These are product contracts, not Python
  dictionaries.
- Gateway saved-map list, PCD download, and JSON point preview now call the
  maps service contract instead of constructing `map.pcd` / artifact paths in
  the route.
- Gateway session map activation and relocalization use the maps bundle/control
  contract (`navigation_safety_3d`, `source_pointcloud`, `set_active`). Without
  `maps.service`, Gateway reports the map unavailable instead of guessing from
  files on disk.
- Gateway saved-map loading, artifact validation, active-map lookup, and voxel
  edit overlay queries also use MapsService; they do not fall back to direct
  filesystem scans.
- `ListMaps`, `GetRecord`, `GetActiveMap`, `GetMapHealth`, and `GetMapBundle`
  are returned directly from `MapsServiceCore`. Python no longer scans the map
  directory to rebuild or override native records on the query path.
- Runtime artifact validation is native. Strict activation requires source
  PCD, metadata, at least one planning artifact, the requested frame, and a
  valid version manifest/checksum set when the map is versioned.
- GlobalPlanner active-map lookup uses the lightweight `MapStore` bundle C ABI.
  It no longer reads `map_record.json`, scans an `active` directory, or starts a
  second SaveMap/artifact worker for read-only planner queries.
- Gateway calls the public `MapsModule` contract (`list_maps`,
  `get_active_map`, `get_map_bundle`, `get_map_points`,
  `validate_map_artifacts`, `set_active_map`). It no longer probes private
  `_get_*` / `_map_*` methods.
- Gateway artifact export resolves every bundle URI underneath the bundle's
  declared map directory and rejects absolute or relative path escape.
- Gateway PCD import accepts only existing `.pcd` files under
  `LINGTU_MAP_IMPORT_DIR` (default `<map-root>/.exchange/import`). Version
  package import/export is likewise restricted in both Python and C++ to the
  configured exchange roots.
- C ABI command responses are execute-once and retryable: a truncated response
  reports the exact required size and is cached until the caller retrieves it.
- Gateway keeps only a transient browser point-cloud cache in
  `cloud_scene_cache.py`. It is presentation state and cannot be used as a
  saved-map or planning truth source.
- CLI preflight queries and activates saved maps through native
  `MapsServiceCore`; REPL map mutations are rejected when `maps.service` is not
  running.

Remaining runtime integration work:

- Python still owns the Module/Blueprint runtime shell and calls the runtime
  SLAM adapter to collect a source snapshot. The postprocess and publish path
  after the snapshot lands in a staging directory is native C++.
- `MapPipelineService` and `MapControlService` are thin command adapters around
  `NativeMapsService`; map algorithms, subprocess ownership, filesystem
  transactions, identity, active state, records, and queries are native.
- The runtime port uses typed `MapControlRequest`, but `MapsModule` remains a
  Python Module/Blueprint shell. A standalone C++ process endpoint is still
  required before that shell can disappear entirely.
- Persistent map mutation is owned by maps control/pipeline services. Gateway,
  CLI, Manager, Planner, and REPL are consumers and may not rename, delete,
  activate, or invalidate map-package files directly.
- The native artifact worker is a persistent local scheduler, not a distributed
  cluster scheduler. A standalone maps endpoint can host the same worker
  without changing the service contract.

Until those are migrated, Python may dispatch commands, but it must call into
the native C++ service for map identity, active state, artifact inventory,
health, bundle query, and build lifecycle state.

### Unity Semantic Source

The supported source directory is the Unity navigation environment output,
not an ad-hoc JSON export:

```text
<scene>/
  environment/Categories.csv
  object_list.txt
```

`Categories.csv` uses the upstream columns
`name,cleaned,nyuId,nyu40id,nyuClass,nyu40class`. Each object-list row is
`id x y z size_x size_y size_z yaw "label"`. Labels are normalized against the
LingTu taxonomy; unmapped labels are reported and omitted unless
`include_unknown_geometry` is explicitly enabled. `person`, `animal`, and
`vehicle` are omitted by default because persistent semantic assets must not
turn transient actors into static map truth.

Standalone native import:

```bash
cmake -S src/maps -B build/maps -DLINGTU_MAPS_BUILD_TOOLS=ON
cmake --build build/maps --target lingtu_maps_import_unity -j
build/maps/lingtu-maps-import-unity \
  --scene <scene> \
  --taxonomy config/semantic_taxonomy.json \
  --output <map-package>/semantic_map.bin
```

Product control uses `MapControlRequest(action="import_unity_semantics")`
with `name`, `scene_dir`, and optional bounded import settings. The native
pipeline stages under `.builds/<build_id>_transaction`, preserves the previous
artifact on any parse, limit, ABI, or publish failure, and marks the result
query-only (`navigation_ready=false`).

## Domain Boundary

Maps owns:

- `MapRecord`, `MapArtifact`, `MapHealth`, `MapBundle`, and POI model.
- Map asset storage, index, hashes, and URI resolution.
- Active map selection, validation, health gate, and rollback hooks.
- Runtime point-cloud ingest and snapshot capture contracts.
- Derived map layers: voxel, occupancy, elevation, ESDF, traversability.
- Artifact build pipeline: point cloud, occupancy, octomap, ESDF, traversability.
- Capability query API for planners and visualizers.

Maps does not own:

- SLAM or localization algorithms.
- Global planner, local planner, path follower, or mission FSM.
- Gateway routes, frontend rendering, CLI command parsing, or MCP transport.
- DDS/ROS/WebRTC transport core.

## Data Plane

The C++ data plane is the main product path:

```text
Livox / SLAM point cloud
  -> MapObservationFrame(accepted incremental scan + exact map<-sensor pose)
  -> maps.ingest
  -> maps.voxel / maps.occupancy / maps.elevation
  -> maps.semantic (geometry-only or exact uint16 labels)
  -> maps.esdf / maps.traversability
  -> maps.bundle(capability)
  -> nav planner / safety / gateway adapter
```

The primary cloud payload should be a binary view over contiguous memory, not a
Python object graph:

```text
PointCloudView
  frame_id
  stamp_ns
  fields: XYZ or XYZI
  layout: SoA preferred for hot paths, AoS accepted at ingest boundary
  storage: borrowed, shared-memory, or owned span
```

For long-lived 3D maps, prefer block-based sparse storage with dense local
blocks. A simple hash map is acceptable only for transitional layers and tests.

Realtime ray updates must consume `MapObservationFrame`, never accumulated
`map_cloud`. The observation carries the accepted sensor-frame scan, exact
state-at-scan transform, origin, sequence, and timestamp. Full `MapCloudFrame`
snapshots remain a save/export boundary and cannot reconstruct per-ray origins.

## Control Plane

Control plane commands are strongly typed and separate from data-plane ingest:

```text
CreateMap
SaveMap
DeleteMap
RenameMap
RestoreSourceBackup
SetActiveMap
BuildArtifact
SetPOI
```

## SaveMap Product Chain

`SaveMap` is a durable C++ workflow, not a sequence of Python file operations.
Python is limited to asking the active SLAM and semantic producers to write an
immutable snapshot into the capture directory allocated by the native job.

```text
Gateway / CLI / MCP
  -> MapsModule snapshot adapter
  -> SaveMapEngine::Begin(request_id, requirements)
  -> SLAM map.pcd + optional semantic_map.bin capture
  -> SaveMapEngine::ProvideSnapshot(snapshot metadata)
  -> native source filter / optimizer
  -> native occupancy / OctoMap / ESDF / traversability build
  -> required-artifact and SHA-256 verification
  -> immutable version directory
  -> atomic current_version.txt replacement
  -> MapStore / MapBundle readers
```

The durable layout is:

```text
<map-root>/.save_jobs/<job-id>/
  job.state                    atomic latest status snapshot
  events.jsonl                 append-only phase and commit journal

<map-root>/<map-id>/
  current_version.txt          atomic logical commit pointer
  compatibility_version.txt   root-view synchronization marker
  .versions/
    00000000000000000001/
      map.pcd
      occupancy.npz
      octomap.ot
      esdf.npz
      traversability.npz
      semantic_map.bin         optional/required by request
      metadata.json
      save_manifest.json       snapshot identity and artifact SHA-256 list
      save_manifest.sha256     manifest integrity sidecar
      artifact_checksums.sha256
                               machine-readable checksum index for every artifact
```

`MapStore::ContentPath()` follows `current_version.txt`; `MapRecord.version` is
the committed version number and artifact URIs point into the immutable version
directory. Files mirrored at the map root are a transitional compatibility
view and are not the source of truth.

Save job states are `WAITING_SNAPSHOT`, `QUEUED`, `RUNNING`, `SUCCEEDED`,
`FAILED`, and `CANCELLED`. Phases are `CAPTURE`, `VALIDATE`, `PROCESS_SOURCE`,
`BUILD_ARTIFACTS`, `VERIFY`, `COMMIT`, and `DONE`.

Recovery rules are explicit:

- a crash before `current_version.txt` changes leaves the old version current;
- a prepared but uncommitted version is removed and the captured job is queued
  again on service startup;
- a crash after the pointer changes is recovered as success and the
  compatibility view is resynchronized;
- required artifact failure, hash mismatch, cancellation, or unhealthy SLAM
  cannot publish a partial current version;
- every read, activation, recovery, version listing, and rollback verifies the
  manifest plus the checksum index and recomputes every declared artifact SHA;
- `request_id` is the idempotency key; the same payload replays its job, while
  a different payload with the same key is rejected;
- external cleaner, optimizer, and converter processes receive cooperative
  cancellation and are terminated by the native process runner;
- failed/cancelled jobs can be retried through the same persisted job.
- when `activate_on_success=true`, activation is part of the request contract:
  a committed version with failed activation is reported as
  `FAILED/activation_failed_after_commit`; the verified version is retained for
  diagnosis or explicit activation instead of being deleted or misreported;
- SaveMap, delete, rename, version listing, and rollback share the native
  `<map-root>/.map_locks/<map-id>` write lock; a map cannot disappear or move
  during commit.

The immutable version and `current_version.txt` are canonical. The mutable
root view is compatibility-only. It is fully staged before publication and
`compatibility_version.txt` is written last. Save status reports
`compatibility_ready` and `compatibility_message`; a canonical commit remains
recoverable even when the legacy view is explicitly marked degraded.

Runtime control operations are `save`, `save_status`, `cancel_save`,
`retry_save`, `list_map_versions`, and `rollback_map_version`. Gateway exposes
the same contract at
`POST /api/v1/map/save`, `GET /api/v1/maps/save-jobs/{job_id}`,
`GET /api/v1/maps/save-jobs`,
`POST /api/v1/maps/save-jobs/{job_id}/cancel`, and
`POST /api/v1/maps/save-jobs/{job_id}/retry`. Verified immutable versions are
available at `GET /api/v1/maps/{name}/versions`; rollback atomically changes the
logical version pointer through
`POST /api/v1/maps/{name}/versions/{version}/rollback` and then rebuilds the
root compatibility view.

Queries are capability-based:

```text
ListMaps
GetActiveMap
GetMapMetadata
GetArtifact
GetMapBundle(capability)
GetPOI
GetMapHealth
```

Planners must not read raw file paths directly. They ask for a `MapBundle` by
capability:

| Capability | Expected map product |
| --- | --- |
| `global_2d_planning` | Occupancy/cost grid. |
| `terrain_reasoning` | Elevation + traversability. |
| `trajectory_optimization` | ESDF. |
| `collision_3d` | Octomap or voxel interface. |
| `visualization` | Downsampled point cloud + layered overlays. |

## Consolidation Status

The map domain no longer has an implementation under `src/nav/services/map`.
The canonical Python application boundary is `maps.services`; it coordinates
the C++ store/service/pipeline and the current Module runtime adapter. It is not
a compatibility package and must not be moved back under navigation.

Completed in the consolidation pass:

- persistent map services moved to `src/maps/services`;
- metadata and same-source validation moved to `maps.artifacts`;
- map-root/path and PCD preview helpers moved out of Gateway;
- map-domain tests moved under `src/maps/tests`;
- Python source was removed from the generated `src/maps/build` directory;
- CMake output is now documented and tested under repository-level `build/maps`;
- PCD-only maps report `STALE`; only maps with planning artifacts report
  `READY`.
- the canonical runtime alias changed from `nav.maps` to `maps.service`;
- Gateway restore/rename routes now delegate to `MapControlService` instead of
  mutating directories directly;
- restoring a pre-clean PCD invalidates derived artifacts, marks the map
  `STALE`, and deactivates it until rebuild succeeds;
- Gateway runtime dataflow diagnostics reuse `maps.artifacts` validation rather
  than maintaining a second SHA/provenance implementation.

Current progress:

- C++ contracts exist under `include/lingtu/maps`.
- `maps.voxel` core exists as `layers::VoxelLayerCore`.
- `maps.semantic_occupancy` core exists as
  `layers::SemanticOccupancyLayerCore`. It is a ROS/PCL/Python-free C++ layer
  and does not vendor the SOCC-ICP or Radix runtime.
- `VoxelLayerCore` currently implements range/z filtering, XYZ/XYZI
  interleaved input, XYZ/XYZI SoA input, per-frame voxel dedupe, XY
  column-carving, decay, containment query, and SoA snapshot output.
- `lingtu_maps` C ABI exposes the voxel layer for non-C++ runtimes without
  ROS, PCL, or Python headers in the C++ library.
- `maps.adapters.python.voxel.NativeVoxelLayer` is a thin ctypes owner for that
  C ABI.
- Live runtime layer Modules now live under `maps.modules`. They are thin
  Module/port adapters around `lingtu_maps`; realtime voxel, occupancy,
  elevation, ESDF, and traversability math is native-only.
- `MapStore` and its C ABI now own map ids, artifact scanning, and active-map
  state using `active_map.txt` instead of the legacy `active` symlink.
- `MapsServiceCore` and its C ABI now own native map control/query operations:
  list, get record, get active, health, create, delete, rename, set active, and
  clear active. Command size probes do not execute side effects.
- `maps.adapters.python.service.NativeMapsService` is a thin ctypes shell over
  `MapsServiceCore`; `MapStorageService` calls this native service for control,
  query, source-map transaction, and artifact build operations.
- Import, crop, and save-source replacement all expose
  `transactional_visibility: staged_until_commit`. Successful operations remove
  temporary source transaction directories and invalidate stale derived
  navigation artifacts before a rebuild.
- Saved-map OctoMap artifact building is owned by `MapPipelineCore`. Build mode
  `native_octomap` embeds `octomap::OcTree` when the OctoMap development
  package is available at CMake configure time; external OctoPlanner3D/OctoMap
  conversion remains an explicit compatibility mode with native timeout,
  output capture, and structured error reporting.
- `GetMapBundle(capability)` now exists in `MapsServiceCore` and is exposed
  through the C ABI / Python adapter. Runtime record and bundle queries return
  the native response directly.
- `GetMapTypes` now exists in `MapsServiceCore`; the runtime `get_map_types`
  command returns the native catalog directly.
- `BuildEsdfArtifact` and `BuildTraversabilityArtifact` now exist in C++.
  They read native `occupancy.npz`, write staged `esdf.npz` /
  `traversability.npz`, and expose `trajectory_optimization` /
  `traversability` bundles through the C ABI and Python adapter.
- Save-source postprocess is native: `CommitSavedSourceJson` stages the source
  snapshot, runs dynamic filtering and optimizer commands from C++, copies
  auxiliary files/directories, invalidates stale derived artifacts, and commits
  the new source map only after the full transaction succeeds.
- `SaveMapEngine` now owns the complete durable save workflow: typed request
  and snapshot contracts, idempotent jobs, persistent status and event journal,
  cancellation, retry, crash recovery, artifact verification, immutable
  versions, and atomic current-version publication.
- `MapStore` resolves record/bundle queries through the committed version
  pointer; artifact URIs and hashes no longer depend on mutable root files.
- Native POI, map graph, rollback/history, named active slots, retirement, and
  artifact-job contracts are exposed through `MapsServiceCore`, the C ABI, and
  the Python adapter. The native worker consumes persistent jobs with leases,
  heartbeat, cancellation, retry, and recovery.
- `ListMaps`, `GetMapBundle(source_pointcloud)`, and `GetMapPoints` now back
  the Gateway map list, PCD download, and saved-map points routes. Gateway no
  longer parses `map.pcd` itself for `/api/v1/maps/{name}/points`, and it no
  longer constructs the `/api/v1/maps/{name}/pcd` path itself.
- Frontend scene rendering has been split into layer files under
  `web/src/components/scene3d/layers/*` for live cloud, saved map, costmap, and
  slope overlays.

## Remaining Migration Worklist

These are the remaining product hardening items. They should extend the native
contracts above, not add another Python map implementation.

1. Source snapshot ownership:
   Python still asks the current SLAM/runtime adapter to materialize a snapshot
   directory. After that directory exists, filtering, optimization, validation,
   auxiliary copy, and publish are native. A future robot-side map daemon can
   replace the snapshot call without changing the C++ commit contract.
2. Native `MapRecord` provenance depth:
   record/bundle generation is C++, but health provenance should include the
   exact source snapshot id, optimizer command hash, cleaner command hash, and
   artifact dependency graph.
3. Native process endpoint:
   SaveMap orchestration is native and the C ABI request/snapshot structs are
   typed. A standalone C++ process endpoint can later replace the Python
   Module transport shell without changing the map-domain workflow.
4. Semantic-weighted localization:
   the stable semantic chunk ABI and guarded geometry MapIcp path are connected
   in `src/localization`. Label/confidence-aware residual weighting and
   planarity-aware correspondence rejection remain localization work; maps must
   not own pose estimation.
5. Field verification:
   run MID-360 replay and S100P aarch64 gates for dynamic residue, corridor
   degeneracy, save/recovery power loss, and sustained scene throughput.

## Build And Run

Build the native maps library and C++ tests:

```powershell
cmake -S src\maps -B build\maps -DLINGTU_MAPS_BUILD_TESTS=ON
cmake --build build\maps --config Release
ctest --test-dir build\maps -C Release --output-on-failure
```

Use the native library from Python tests or a local run:

```powershell
$env:LINGTU_MAPS_LIB=(Resolve-Path build\maps\Release\lingtu_maps.dll).Path
python -m pytest src\maps\tests -q
```

`VoxelGridModule` backend selection:

| Value | Behavior |
| --- | --- |
| `cpp` / `native` | Require native backend; startup fails if missing. |

Blueprint config passes this through as `voxel_backend` or
`voxel_grid.backend`.

## Quality Gates

- No `rclcpp`, `tf2_ros`, `sensor_msgs`, or ROS headers in `src/maps/include`.
- No Python/numpy dependency in C++ layer implementations.
- No Gateway or nav mission imports in maps core.
- C++ unit tests must cover frame metadata, cloud layout, layer update, bundle
  query, and artifact validation.
- Python map-domain tests live under `src/maps/tests`.
