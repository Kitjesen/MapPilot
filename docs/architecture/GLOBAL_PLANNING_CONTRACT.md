# Global Planning Contract

Status: current contract
Audience: navigation/planner maintainers, Gateway/UI preview maintainers
Replaced by: not replaced

This document defines LingTu's global planning boundary. The goal is fast
backend replacement without leaking OctoPlanner3D, PCT, subprocess, or payload
details into Navigation, Gateway, UI, or transport code.

## 1. Location

| Surface | Path |
| --- | --- |
| Public planning contracts | `src/nav/services/plan/contracts.py` |
| Planner service factory | `src/nav/services/plan/factory.py` |
| Map-backed planner service | `src/nav/services/plan/global_planner/service.py` |
| Saved-map bundle lookup | `src/nav/services/plan/global_planner/artifacts.py` |
| Mapless/direct service | `src/nav/services/plan/mapless/direct.py` |
| Backend runtime adapter | `src/nav/services/plan/global_planner/backend_runtime.py` |
| OctoPlanner3D backend | `src/nav/services/plan/global_planner/algorithm/octoplanner3d_planner.py` |
| Native planning contract | `src/nav/cpp/planning/global/global_planner_contract.hpp` |
| Native OctoPlanner3D | `src/nav/cpp/planning/global/octoplanner/` |
| Native FAR option | `src/nav/cpp/planning/global/far/` |
| Active map gates | `src/nav/cpp/endpoint/active_octomap_gate.*`, `active_occupancy_gate.*` |
| PCT legacy backend | `src/nav/services/plan/global_planner/algorithm/pct/planner.py` |
| Preview API helper | `src/nav/services/plan/preview.py` |
| Mission integration | `src/nav/runtime/planning.py` |

## 2. Boundary

Global planning is the saved-map Goal route. It requires a current pose and a
target in the planning frame plus a validated active map artifact. Map-free
exploration does not enter this boundary; it uses the native `Live` rolling
segment route.

Navigation calls the planner through one method:

```text
PlannerService.plan_request(GlobalPlanRequest) -> GlobalPlanResult
```

Backends implement the same logical contract. A backend may also keep a legacy
`plan(start, goal)` method, but that is compatibility only.

## 3. Input

Saved-map input is selected before backend construction:

```text
active map_record.json
  -> map.bundle capability lookup
  -> planner artifact uri
  -> backend constructor
```

For OctoPlanner3D, the selected product artifact is the
`navigation_safety_3d` capability, currently backed by `octomap.ot`. `map.pcd`
is the source point-cloud artifact and rebuild input; it is not the normal
planning-time input once the bundle has a valid 3D occupancy artifact.

FAR consumes the active `occupancy.npz` trinary grid. It is an explicit 2D
option, not an automatic fallback from OctoPlanner3D. Both native backends must
validate the active Maps record, artifact hash, source, frame, and version and
plan from a private immutable snapshot.

Explicit map paths still win for tests and emergency operation. Legacy active
filenames remain fallback for older saved maps.

```text
GlobalPlanRequest
  start: [x, y, z]
  goal: [x, y, z]
  safe_goal_tolerance: float
  frame_id: string
  request_id: string
  map_version: string
  map_generation: uint64
  expected_map_identity: MapIdentity
```

All coordinates are in the planning frame. Frame mismatch is blocked before
planning; the planner must not guess transforms.

Live map updates use:

```text
GlobalPlanningMap
  grid: 2-D float grid
  resolution: meters per cell
  origin: [x, y] | null
  frame_id: string
  map_version: string
  generation: uint64
  source: string
```

## 4. Output

The canonical output is both Python-friendly and UI/SDK-friendly:

```text
GlobalPlanResult
  schema_version: lingtu.global_plan.v1
  path: [[x, y, z], ...]
  plan_ms: float
  reached_goal: bool
  error: string
  frame_id: string
  request_id: string
  map_version: string
  adjusted_goal: [x, y, z] | null
  diagnostics: object
  report: object
```

`GlobalPlanResult.to_wire()` is the JSON payload for Gateway, UI, SDK, Dart,
Rust, or replay consumers.

## 5. Service Responsibilities

`GlobalPlanner` may do these orchestration steps:

1. validate saved-map artifacts;
2. update or validate live planning grid;
3. adjust unsafe goals to nearby reachable cells;
4. call the selected backend;
5. evaluate path safety;
6. repair partial or unsafe paths when policy allows;
7. downsample path output;
8. report selected planner, fallback, adjusted goal, diagnostics, and policy.

It must not:

- own mission state;
- publish robot commands;
- expose backend subprocess schemas to UI;
- import Gateway or driver code;
- treat ROS 2 topics as its public API.

## 6. Backend Responsibilities

Backends are algorithm implementations. They should accept `GlobalPlanRequest`
and return `GlobalPlanResult`.

| Backend | Role |
| --- | --- |
| `octoplanner3d` | Default map-backed product planner. |
| `far` | Explicit native 2D visibility-graph planner over validated occupancy. |
| `pct` | Legacy/manual experiment planner. |
| `direct` | Development-only mapless helper; never a field Goal fallback. |

Backend-specific map formats, native libraries, subprocesses, or diagnostics
must be normalized before leaving the planner service boundary.

## 7. Runtime Data Flow

The physical product path is native:

```text
typed Goal DDS
  -> navd
  -> selected active artifact gate
  -> OctoPlanner3D or FAR
  -> GlobalPlanResult identity/epoch check
  -> NavLoop
  -> final safety and control authority
  -> /nav/cmd_vel
```

Python `Navigation -> PlannerService -> LocalPlanner -> PathFollower` remains
the Module/simulation path. It is not the owner of the field endpoint.

The preview path is non-motion:

```text
Gateway preview command
  -> Navigation.preview_plan()
  -> PlanPreviewService
  -> PlannerService.plan_request()
  -> global_plan wire payload
```

## 8. Transport Policy

Global planning uses module ports in-process by default. Cross-process or
cross-language delivery must wrap the wire payload with an explicit transport
contract.

| Transport | Policy |
| --- | --- |
| local port | default Navigation to local planner path. |
| Gateway JSON | UI/SDK preview and status payloads. |
| DDS/LCM | Adapter boundary only; must preserve schema version, frame, and timestamp. |
| ROS 2 | Compatibility bridge only, not the planner contract. |

## 9. Acceptance Checks

```bash
python -m pytest src/nav/tests/test_global_planner_contracts.py -q
python -m pytest src/nav/tests/test_planning_service_factory.py -q
python -m pytest src/nav/tests/test_global_planner_diagnostics.py -q
python -m pytest src/runtime/tests/test_nav_chain_efficiency.py -q
python -m pytest src/gateway/tests/test_gateway_commands.py -k "preview or click_navigation" -q
python -m pytest tests/contracts/test_nav_cpp_build_boundaries.py -q
bash scripts/build/build_nav_endpoint.sh
```
