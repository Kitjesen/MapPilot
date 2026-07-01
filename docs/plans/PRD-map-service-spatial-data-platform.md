# PRD: MapService Spatial Data Platform

Status: active.

## Problem

Navigation, UI, Gateway, and planners need maps as typed spatial data, not as
loose files in a directory. The system must answer three questions reliably:

- what map assets exist;
- which artifact satisfies a planner or UI capability;
- what changed when a map is created, built, activated, retired, or rejected.

## Goal

Turn MapService into the product boundary for saved maps while keeping the
current file-backed storage.

```text
map.cloud_frame -> MapService
map_command     -> MapService -> MapRecord / map.bundle / map.event
```

## Scope

| Area | Requirement |
| --- | --- |
| Type catalog | Expose `map.types` for classes, artifacts, aliases, and capabilities. |
| Record | Persist `map.record` in each saved map directory. |
| Bundle | Return `map.bundle` by capability, not raw filename. |
| Lifecycle | Support create, save, build, activate, retire, rename, delete. |
| Events | Publish `map.event` for lifecycle, artifact, and validation events. |
| Runtime ingest | Accept `map.cloud_frame` as the typed map-cloud data plane. |
| Gateway | Forward `map_event` through existing SSE as `type=map_event`. |
| Planner handoff | Let global planning resolve active maps through record capabilities first. |
| Compatibility | `build_artifact` accepts artifact type, artifact name, map class, or capability. |

## Non-Goals

- No database in this phase.
- No new transport. Existing Module ports remain enough.
- No native ESDF or OctoMap builder vendoring in this phase.
- No UI redesign in this phase.

## Contracts

| Contract | Producer | Consumer |
| --- | --- | --- |
| `map.types` | `MapAPIService.get_map_types()` | UI, Gateway, docs, tests |
| `map.record` | `MapStorageService.write_map_record()` | MapService, planner artifact resolver |
| `map.bundle` | `MapAPIService.get_map_bundle()` and active record lookup | GlobalPlanner, UI, Gateway |
| `map.event` | `MapService.map_event`, Gateway SSE `type=map_event` | UI, Gateway, monitoring |
| `map.cloud_frame` | SLAM, map builders, bridge adapters | MapService, future map artifact builders |

## Runtime Ingest

`MapCloudFrame` is the canonical map-cloud input. It does not define the
transport. The current Python module port is:

```text
MapService.map_cloud_frame: In[MapCloudFrame]
```

Frame modes:

| Mode | Meaning in MapService |
| --- | --- |
| `FULL` | Replace the current live map snapshot. |
| `KEYFRAME` | Append a keyframe cloud to the live map snapshot. |
| `INCREMENTAL` | Append a delta cloud to the live map snapshot. |

Legacy `map_cloud: PointCloud2` remains supported and is converted to
`MapCloudFrame(mode="FULL")` at the MapService boundary.

## Event Model

Events are emitted only for stateful map operations and validation failures.
Queries such as `list`, `get_record`, and `get_map_bundle` do not emit events.

| Event | Trigger |
| --- | --- |
| `map.created` | map created |
| `map.saved` | map saved |
| `map.artifact_built` | tomogram, occupancy, octomap, or named artifact built |
| `map.active_changed` | active map changed |
| `map.renamed` | map renamed |
| `map.retired` | map retired |
| `map.deleted` | map deleted |
| `map.validation_failed` | artifact gate or validation failure blocks a lifecycle operation |

Minimum payload:

```json
{
  "schema_version": "map.event",
  "event": "map.active_changed",
  "action": "set_active",
  "map_id": "field_01",
  "success": true,
  "message": "",
  "record_version": "field_01:v1",
  "timestamp": "2026-06-30T00:00:00+00:00"
}
```

## Type Compatibility

The catalog covers all planned map classes, but only some are buildable today.

| Map class | Build status |
| --- | --- |
| `saved_point_cloud` | produced by `save` |
| `static_2d_occupancy` | buildable through `build_artifact(path_planning_2d)` |
| `portable_2d_occupancy` | produced with occupancy snapshot as `map.yaml` and `map.pgm` |
| `global_2_5d_terrain` | buildable through `build_artifact(terrain_reasoning)` |
| `global_3d_occupancy` | buildable through `build_artifact(navigation_safety_3d)` |
| `esdf` | modeled only; returns `builder_unavailable` |
| `semantic` | modeled only; returns `builder_unavailable` |
| `map_metadata` | produced by artifact builders |

Failed map commands include `reason_code`; this is the stable field UI and
Gateway clients should branch on.

## Acceptance Checks

```bash
python -m pytest src/runtime/tests/test_map_messages.py -q
python -m pytest src/nav/tests/test_maps_service.py -q
python -m pytest src/nav/tests/test_planning_map_artifacts.py -q
```

Passing means:

- type, record, bundle, and event schemas are stable;
- lifecycle operations publish events;
- validation failures publish `map.validation_failed`;
- planners can resolve active map artifacts from records first;
- legacy active-file fallback still works.

## Next Work

1. Wire SLAM and bridge producers to publish `MapCloudFrame` directly.
2. Add native builder selection for ESDF and direct OctoMap generation.
3. Add UI display for map lifecycle events.
