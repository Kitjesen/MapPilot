# MapService Spatial Data OS Plan

Status: Phase 1 implemented; Phase 1.5 modularization implemented; map type
catalog, bundle contract, and lifecycle event port implemented. Current product
priorities are tracked in `../plans/current-roadmap.md`.

Authoritative implementation contract: `MAP_SERVICE_CONTRACT.md`.

MapService is being upgraded from a map-file command handler into a spatial
data service. The current implementation keeps the file-backed runtime and
`map_command` transport, but the original monolithic `maps.py` has been split
into storage, pipeline, control, query/API, runtime-ingest, and command-adapter
layers under `src/nav/services/map/`.

## Landed In This Slice

| Area | Status | Code |
| --- | --- | --- |
| MapRecord model | Done | `src/nav/services/map/records.py` |
| Artifact model | Done | `src/nav/services/map/records.py` |
| MapHealth model | Done, heuristic only | `src/nav/services/map/records.py` |
| map_record.json | Done | written beside `metadata.json` |
| Capability lookup | Done for file-backed maps | `get_map_bundle` returns schema-stable bundle |
| Type catalog | Done | `get_map_types` returns states, classes, artifacts, aliases, and capabilities |
| Active-map lifecycle | Partial | active/retired states supported |
| Map listing | Improved | includes `record`, `health`, `capabilities` |
| Storage boundary | Done for file backend | `src/nav/services/map/storage.py` |
| Runtime bridge | Done for current PointCloud2 input | `src/nav/services/map/runtime_bridge.py` |
| Build pipeline extraction | Done for current builders | `src/nav/services/map/pipeline.py` |
| Control plane extraction | Done for current lifecycle ops | `src/nav/services/map/control.py` |
| Query/API extraction | Done for current query ops | `src/nav/services/map/api.py` |
| Lifecycle event stream | Done for current stateful actions | `MapService.map_event` |
| Unsafe map-name rejection | Done | shared `validate_map_name()` |
| Tests | Done for slice | `src/nav/tests/test_maps_service.py` |

## Requirement Gap Matrix

| Product Requirement | Current Implementation | Gap |
| --- | --- | --- |
| MapService as Spatial Data Platform | Started: records, artifact graph, health, capabilities, and service layers exist | Still file-backed and command-port based |
| Standard map asset management | Supports pointcloud, occupancy, tomogram, octomap metadata | ESDF and semantic artifacts are only modeled, not built |
| Multi-representation MapRecord | `MapRecord` persisted as `map_record.json` | No DB/index, no cross-map graph |
| Artifact abstraction | `Artifact(type, uri, hash, source_map_id, generator, build_config)` exists | Artifact dependencies are flat, not full DAG traversal |
| MapHealth | `localization_stability`, `planning_success_rate`, `collision_rate`, `map_freshness`, `overall_score` exist | Scores are heuristic from artifact gate, not runtime metrics |
| Capability-based supply | `get_map_bundle(capability)` maps capability to schema-stable bundle | Planner still reads file paths directly |
| Strong Control Plane API | Command routing is isolated in `map/command_router.py`; lifecycle ops live in `MapControlService`; command names exist: `create_map`, `retire_map`, `build_artifact`, etc. | Still transported as JSON through `map_command`; no typed command dataclasses yet |
| Data Plane API | Existing `map_cloud: PointCloud2` is retained | No `MapCloudFrame` with FULL/INCREMENTAL/KEYFRAME semantics |
| Query API | `MapAPIService` owns `list`, `get_record`, `get_active_map`, `get_map_health`, `get_map_bundle`, `get_map_types`, and POI CRUD | Facade wrappers remain for compatibility |
| Event Stream | `map_event` publishes `map.event` for lifecycle/artifact/validation events | Gateway/SSE surfacing not done |
| Lifecycle | create/save/build/active/retire exist | No rollback, update versioning, or retired archive policy |
| Map Graph | Not implemented | Need `map_edges.json` before any DB |
| Planner decoupling | Boundary exists through `get_map_bundle`; planner migration not done | `GlobalPlanner` can still resolve `octomap.bt`/`tomogram.pickle` paths |

## Map Classes

| Class | Artifact | Current State |
| --- | --- | --- |
| `saved_point_cloud` | `map.pcd` | Supported |
| `static_2d_occupancy` | `occupancy.npz` | Supported |
| `portable_2d_occupancy` | `map.pgm`, `map.yaml` | Supported |
| `global_2_5d_terrain` | `tomogram.pickle` | Supported |
| `global_3d_occupancy` | `octomap.bt` | Supported through external converter |
| `esdf` | `esdf.npz` | Modeled only |
| `semantic` | `semantic.json` | Modeled only |
| runtime costmap/elevation/esdf | Module streams | Not persisted by MapService |

## Current Code Size

| File | Role | Lines |
| --- | --- | ---: |
| `src/nav/services/maps.py` | MapService Module facade and compatibility wrappers | 565 |
| `src/nav/services/map/pipeline.py` | Save/build pipeline and occupancy/tomogram/octomap helpers | 952 |
| `src/nav/services/map/records.py` | MapRecord, artifact, health, capability model | 275 |
| `src/nav/services/map/storage.py` | File-backed storage, active state, POI persistence, name validation | 223 |
| `src/nav/services/map/runtime_bridge.py` | live map-cloud ingest and SLAM/save adapter boundary | 220 |
| `src/nav/services/map/api.py` | query API, POI API, active artifact helpers | 213 |
| `src/nav/services/map/control.py` | create/delete/retire/rename/set-active lifecycle operations | 189 |
| `src/nav/services/map/command_router.py` | JSON command adapter routing through the facade | 88 |
| `src/nav/tests/test_maps_service.py` | MapService focused tests | 382 |
| `src/nav/services/plan/global_planner/artifacts.py` | Planner active artifact resolver | 65 |
| `src/nav/tests/test_planning_map_artifacts.py` | Planner artifact resolver tests | 50 |

## Current Module Boundary

`nav.services.maps.MapService` remains the runtime Module. It owns ports,
MCP/skill wrappers, health summary, and compatibility methods. It should not
grow new filesystem, builder, or API logic.

`MapStorageService` is the only place that resolves map names into directories.
All map names must pass `validate_map_name()`, which rejects absolute paths,
path traversal, path separators, leading dot/dash names, and disallowed
characters.

`MapPipelineService` is intentionally isolated even though it is still Python.
This is the replacement seam for future C++ or Rust builders. Control/query
logic must not depend on how occupancy, tomogram, octomap, ESDF, or semantic
artifacts are built.

`MapRuntimeBridge` is the compatibility data-plane adapter for today's
`PointCloud2` input and SLAM save adapter. The future `MapCloudFrame` contract
should land here first, then flow into the pipeline.

`MapControlService` owns lifecycle mutation. `MapAPIService` owns read/query
operations. `command_router.py` stays a JSON compatibility adapter and
deliberately routes through facade wrappers until Gateway/MCP/debug callers are
migrated.

## Phase Plan

### Phase 1: File-backed MapRecord

Status: done.

Acceptance:

- every built saved map can write `map_record.json`
- `list` exposes `record`, `capabilities`, and `health`
- retired maps cannot be activated
- `get_map_bundle(capability)` returns a typed bundle with record context
- `get_map_types` exposes the stable type catalog for UI and planners

### Phase 2: Planner Bundle Boundary

Goal: planners ask for capabilities, not raw filenames.

Work:

- add a small `MapBundleProvider` around MapService records
- route `GlobalPlanner` map selection through `get_map_bundle`
- keep backend constructors file-backed internally until native interfaces exist
- preserve old explicit path override for tests and emergency operation

Acceptance:

- `octoplanner3d` requests `navigation_safety_3d`
- `pct` requests `terrain_reasoning`
- `astar` requests `path_planning_2d` first, then tomogram fallback
- no planner code scans `<map_dir>/active` directly

### Phase 3: Typed Control/Query API

Goal: stop spreading raw action strings inside MapService internals.

Status: started. JSON action routing now lives in
`src/nav/services/map/command_router.py`; lifecycle/query/build work is already
delegated to services through `MapService` facade wrappers. Handlers still use
raw dictionaries at the transport edge.

Work:

- add minimal command/query dataclasses inside the map package
- keep JSON `map_command` as compatibility adapter
- return structured errors with stable reason codes
- keep `map_event` output as the runtime event source
- migrate Gateway/MCP callers from private facade wrappers only after tests lock
  their new public query path

Acceptance:

- command parsing is isolated in one adapter
- handlers receive typed objects
- events emitted for created/saved/active_changed/artifact_built/validation_failed

### Phase 4: Data Plane Upgrade

Goal: replace raw `map_cloud` semantics with explicit map-frame ingest.

Work:

- introduce `MapCloudFrame`
- support `FULL`, `INCREMENTAL`, `KEYFRAME`
- define transform/frame contract
- keep current `PointCloud2` adapter as compatibility input

Acceptance:

- Super-LIO snapshot save uses `FULL` or latest accumulated keyframes
- incremental frames update only buffers, not saved map artifacts
- keyframes can produce `poses.txt + patches/` compatible artifacts

### Phase 5: Map Graph

Goal: support multiple maps without starting with a database.

Work:

- add `map_edges.json`
- model map transitions: door, stairs, elevator, outdoor gate
- expose graph in `get_active_map` and `list`

Acceptance:

- map A/B adjacency is queryable
- inactive maps remain addressable
- no cross-map planner work until this model is stable

### Phase 6: Real Health Metrics

Goal: replace heuristic health with runtime evidence.

Work:

- feed localization stability from SLAM/localization health
- track planning success/failure counters
- track collision/estop/safety intervention counters
- compute freshness from last build/update timestamps

Acceptance:

- `active_allowed` can fail for real runtime reasons, not only missing files
- Gateway readiness exposes the same MapHealth payload

## Deliberate Non-Goals For Now

- No Postgres/SQLite/Redis graph until file-backed records become a bottleneck.
- No full ESDF builder in MapService yet.
- No semantic map storage until perception/memory ownership is clarified.
- No planner native memory interface until file-backed capability boundary is stable.
- No C++/Rust rewrite of control/query/storage code. Those layers are not the
  hot path; native work should target artifact builders and heavy geometry
  kernels behind `MapPipelineService`.
