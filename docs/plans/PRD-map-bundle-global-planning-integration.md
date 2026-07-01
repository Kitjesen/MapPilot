# PRD: Map Bundle To Global Planning Integration

Status: in progress.

## Problem

Global planning should not select map inputs by guessing filenames under
`<map_dir>/active`. MapService already records map classes, artifacts, health,
and capabilities. GlobalPlanner must consume that contract so planners can be
swapped by capability instead of by directory convention.

## Goal

Make the saved-map path into global planning:

```text
MapRecord -> map.bundle -> GlobalPlanner -> backend constructor
```

The old active-directory scan remains as compatibility fallback until all saved
maps reliably contain `map_record.json`.

## Scope

| Area | Requirement |
| --- | --- |
| Planner map selection | Resolve planner input from `map_record.json` capability first. |
| Fallback | Keep explicit path override and active-directory filenames. |
| Diagnostics | Include selected `map_bundle` in artifact gate reports when available. |
| Tests | Cover bundle-first lookup and legacy fallback. |
| Docs | Keep map/planning contracts aligned with the shipped behavior. |

## Non-Goals

- No new database or index.
- No new runtime wire from MapService into GlobalPlanner.
- No ESDF builder in this slice.
- No OctoMap/Voxblox/FIESTA vendoring in this slice.

## Capability Mapping

| Planner | Primary capability | Fallback |
| --- | --- | --- |
| `octoplanner3d` | `navigation_safety_3d` | explicit path, then `active/octomap.bt` |
| `pct` | `terrain_reasoning` | explicit path, then `active/tomogram.pickle` |
| `astar` | `path_planning_2d` | `terrain_reasoning`, then legacy filenames |

## Interfaces

`SavedMapArtifacts` owns the file-backed lookup:

```text
planner_map_bundle(planner_name) -> map.bundle dict
planner_map_path(planner_name) -> artifact uri string
static_occupancy_path(map_path="") -> occupancy.npz path
```

`GlobalPlannerArtifactGateMixin` keeps validation:

```text
_validate_map_artifact_gate() -> gate dict with optional map_bundle
```

## Acceptance Checks

```bash
python -m pytest src/nav/tests/test_planning_map_artifacts.py -q
python -m pytest src/nav/tests/test_maps_service.py src/nav/tests/test_global_planner_contracts.py -q
```

Passing means:

- active `map_record.json` selects the planner artifact by capability;
- stale record URIs are ignored;
- explicit map path still wins;
- old active filenames still work;
- planner gate diagnostics include bundle context when present.

## Next Slice

After this lands, update planner status/UI surfaces to show selected
`map_id`, `version_id`, `capability`, and artifact type. Do not add a new map
transport until a real cross-process planner requires it.
