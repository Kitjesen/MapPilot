# Map Service Contract

Status: current implementation contract.

MapService is the spatial data service for saved maps. It stores map assets,
describes their lifecycle, exposes capability-based lookup, and keeps planner
code away from raw directory scanning.

## Abstract

LingTu navigation consumes several map representations at different planning
layers. A saved SLAM point cloud is not enough for navigation: global planning
needs a 3D occupancy representation, legacy terrain planners may need a 2.5D
tomogram, UI and compatibility tools often need a 2D occupancy map, and future
local optimization will need distance fields.

The product contract is therefore not "a map is a folder". A map is a versioned
bundle of typed artifacts with capabilities. Callers request a capability, and
MapService returns the artifact that satisfies it.

## Runtime Boundary

| Layer | Responsibility | Code |
| --- | --- | --- |
| Module facade | Ports, skills, health, compatibility wrappers | `src/nav/services/maps.py` |
| Storage | map-name validation, active-map state, POI persistence, record I/O | `src/nav/services/map/storage.py` |
| Control | create, delete, rename, activate, retire | `src/nav/services/map/control.py` |
| Pipeline | save/build point cloud, occupancy, tomogram, OctoMap artifacts | `src/nav/services/map/pipeline.py` |
| Query API | list, record, active map, health, bundle lookup, type catalog | `src/nav/services/map/api.py` |
| Records | MapRecord, Artifact, MapHealth, map type catalog | `src/nav/services/map/records.py` |
| Command adapter | JSON `map_command` compatibility route | `src/nav/services/map/command_router.py` |
| Events | Lifecycle, artifact, and validation notifications | `MapService.map_event`, Gateway SSE `type=map_event` |

## Runtime Ingest

MapService accepts map clouds through two inputs:

| Port | Type | Status | Semantics |
| --- | --- | --- | --- |
| `map_cloud_frame` | `MapCloudFrame` | canonical | `FULL` replaces the live snapshot; `KEYFRAME` and `INCREMENTAL` append. |
| `map_cloud` | `PointCloud2` | compatibility | converted to `MapCloudFrame(mode="FULL")`. |

`MapCloudFrame` lives in `runtime.msgs.map` so SLAM, drivers, bridges, and
future DDS/LCM adapters can produce it without importing `nav`. Its schema is
`map.cloud_frame`; transport adapters must preserve at least mode,
timestamp, frame id, map id, source, sequence, metadata, and float32 points.

## Current Implementation Status

Completed:

- `MapCloudFrame` is the canonical runtime map-cloud input.
- `PointCloud2` compatibility input is converted to
  `MapCloudFrame(mode="FULL")`.
- Live map snapshots can save a binary XYZ `map.pcd` without requiring
  Open3D at runtime.
- `MapPipeline` reads MapService-generated binary XYZ PCD directly.
- The tested artifact chain is:
  `MapCloudFrame -> map.pcd -> occupancy.npz -> octomap.bt -> map.bundle`.
- Global planners request map capabilities from MapService instead of scanning
  map folders directly.

Still pending:

- Direct native OctoMap and ESDF builders should replace sidecar/converter
  dependence where field deployment needs lower latency or fewer packages.
- Semantic map artifacts are modeled but still need a selected builder and
  lifecycle policy.

## Map Types

| Class | Artifact | Capability | Current state |
| --- | --- | --- | --- |
| `saved_point_cloud` | `map.pcd` | `source_pointcloud` | built |
| `static_2d_occupancy` | `occupancy.npz` | `path_planning_2d` | built |
| `portable_2d_occupancy` | `map.yaml` | `occupancy_yaml` | built |
| `portable_2d_occupancy` | `map.pgm` | `occupancy_image` | built |
| `global_2_5d_terrain` | `tomogram.pickle` | `terrain_reasoning` | built |
| `global_3d_occupancy` | `octomap.bt` | `navigation_safety_3d` | built through configured converter |
| `esdf` | `esdf.npz` | `trajectory_optimization` | modeled, builder pending |
| `semantic` | `semantic.json` | `semantic_query` | modeled, builder pending |
| `map_metadata` | `metadata.json` | `artifact_provenance` | built |

`occupancy` remains a compatibility alias for `occupancy_grid` in inventory
responses. New code should use `occupancy_grid`.

`build_artifact` accepts artifact type, artifact name, map class, or capability.
ESDF and semantic requests are recognized but return `builder_unavailable`
until a native builder is selected.

## MapRecord

Every saved map can write `map_record.json` using schema `map.record`.

Required fields:

| Field | Meaning |
| --- | --- |
| `map_id` | stable map name inside the current file-backed store |
| `lineage_id` | identity shared by renamed or future versioned descendants |
| `version` | integer version inside one lineage |
| `version_id` | `<lineage_id>:v<version>` |
| `state` | one of `EMPTY`, `CREATED`, `BUILDING`, `READY`, `ACTIVE`, `STALE`, `FAILED`, `RETIRED` |
| `scope.map_dir` | absolute local artifact directory |
| `scope.frame_id` | frame used by saved artifacts |
| `artifacts[]` | typed artifacts with hash, generator, class, capability, and role |
| `capabilities[]` | capabilities currently satisfiable by this map |
| `health` | activation and quality summary |

## Bundle Query

The stable lookup API is:

```json
{
  "action": "get_map_bundle",
  "name": "field_2026_06_30",
  "capability": "navigation_safety_3d"
}
```

Successful responses use schema `map.bundle` and include:

| Field | Meaning |
| --- | --- |
| `map_id` | selected map |
| `version_id` | selected version |
| `state` | lifecycle state |
| `frame_id` | artifact frame |
| `capability` | requested capability |
| `artifact` | primary artifact satisfying the capability |
| `artifacts` | all available artifacts in the record |
| `available_capabilities` | full capability list |
| `health` | current quality/activation summary |
| `record` | full MapRecord snapshot |

Planner code should request capabilities:

| Planner | Primary capability | Fallback |
| --- | --- | --- |
| OctoPlanner3D | `navigation_safety_3d` | reject or explicit emergency override |
| PCT legacy | `terrain_reasoning` | none |
| 2D A* / UI preview | `path_planning_2d` | `terrain_reasoning` projection |
| future optimizer | `trajectory_optimization` | `navigation_safety_3d` clearance check |

The current file-backed integration is implemented by
`src/nav/services/plan/global_planner/artifacts.py`: it reads active
`map_record.json` first, then falls back to legacy active filenames.

## Type Catalog

`get_map_types` returns schema `map.types` for UI, Gateway, tests, and
docs. It includes the related record and bundle schema versions, lifecycle
states, classes, artifacts, aliases, and capability-to-artifact mapping.

## Events

`MapService.map_event` publishes `map.event` dictionaries for stateful map
operations. Gateway forwards the same payload through SSE as
`{"type":"map_event","data":...}`. Query actions do not emit events.

| Event | Meaning |
| --- | --- |
| `map.created` | a map directory and record were created |
| `map.saved` | a map was saved |
| `map.artifact_built` | a runtime artifact was built |
| `map.active_changed` | active map changed |
| `map.renamed` | map id changed |
| `map.retired` | map is no longer activatable |
| `map.deleted` | map directory was deleted |
| `map.validation_failed` | artifact gate or validation blocked an operation |

## External Builder Candidates

No external builder is required for the current contract changes. The next
native-builder selection should be explicit:

| Need | Candidate | Reason |
| --- | --- | --- |
| OctoMap library and dynamic EDT | `OctoMap/octomap` | direct C++ OctoMap support |
| TSDF/ESDF mapping | `ethz-asl/voxblox` | mature voxel TSDF/ESDF mapping |
| incremental ESDF | `HKUST-Aerial-Robotics/FIESTA` | focused incremental ESDF planner support |

These packages should enter LingTu as optional native builders or sidecars, not
as imports inside normal Modules.

## Remaining Work

1. Add typed command/query dataclasses behind the JSON command adapter.
2. Implement or integrate native builders for ESDF and direct OctoMap output.
3. Add field evidence for the map artifact builder once the selected converter
   or native builder is deployed on the target runtime.
