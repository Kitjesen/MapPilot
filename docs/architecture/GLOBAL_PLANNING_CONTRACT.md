# Global Planning Contract

Status: current native Product contract

Global planning runs inside C++ `navd` for both `real` and `sim`. The Host sends
a typed goal and consumes lifecycle status and path telemetry; it does not load
maps or execute a second planner.

## Source of truth

| Surface | Path |
| --- | --- |
| Backend-neutral request/result | `src/nav/cpp/planning/global/contract.hpp` |
| OctoPlanner3D | `src/nav/cpp/planning/global/octoplanner/` |
| FAR | `src/nav/cpp/planning/global/far/` |
| Active OctoMap gate | `src/nav/cpp/endpoint/nav/input/active/octomap.*` |
| Active occupancy gate | `src/nav/cpp/endpoint/nav/input/active/occupancy.*` |
| Goal planning lifecycle | `src/nav/cpp/endpoint/nav/runtime/goal/` |
| Route execution | `src/nav/cpp/navigation/` |

## Runtime flow

```text
typed goal command
  -> navd goal admission
  -> current localization + declared active map
  -> OctoPlanner3D or explicit FAR
  -> request/map identity check
  -> admitted map-frame route
  -> Executor
```

The command acknowledgement means only that `navd` admitted the request. The
correlated goal-status lifecycle reports planning, active path, reached,
cancelled, or failed.

## Inputs

A saved-map request requires:

- a finite target in the configured planning frame;
- current localization;
- one declared active map;
- the artifact required by the selected backend;
- a request identity used to reject completion from an older goal or map.

OctoPlanner3D consumes the active 3-D OctoMap artifact. FAR consumes the active
trinary occupancy artifact. Map storage and activation remain owned by the maps
service; the planner reads a private validated snapshot.

ProductControl resolves the saved-map bundle once and publishes paths by
consumer responsibility:

| Runtime value | Consumer | Meaning |
| --- | --- | --- |
| `LINGTU_SLAM_MAP` | localization | Exact saved point cloud used for relocalization. |
| `NAV_GLOBAL_PLANNER` | `navd` | Explicit global-planner selector: `octoplanner3d` or `far`. |
| `OCTOPLANNER_MAP_PATH` | OctoPlanner3D | Exact 3-D OctoMap artifact. Set only when OctoPlanner3D is selected. |
| `FAR_OCCUPANCY_PATH` | FAR | Exact trinary occupancy artifact. Set only when FAR is selected. |
| `EXPLORE_OCCUPANCY_PATH` | Explore map route | Exact occupancy artifact used to restore saved coverage state. |
| `LINGTU_MAP_ID`, `LINGTU_MAP_CONTENT_EPOCH`, `LINGTU_MAP_FRAME` | every saved-map consumer | Identity checked against the selected artifact and planning result. |

`NAV_MAP_DIR` is the maps service storage root; it is not a planner or Explore
input. `navd` also retains explicit `--map` for standalone tools and tests, but
Product execution receives exactly the map variable required by
`NAV_GLOBAL_PLANNER`; it never infers an artifact type from a generic path.

Frame mismatch is rejected before planning. The planner never guesses a
transform or silently switches map artifacts.

## Backends

| Backend | Role |
| --- | --- |
| `octoplanner3d` | Default saved-map 3-D planner. |
| `far` | Explicit 2-D visibility-graph planner over active occupancy. |

FAR is a configured alternative, not an automatic recovery path from an
OctoPlanner3D failure. Live exploration beyond the saved-map boundary uses the
native rolling-segment contract; it is not another saved-map planner backend.

## Output and ownership

A successful result is a verified route in the configured planning frame.
`navd` publishes `/nav/global_path` for telemetry and passes the route directly
to `Executor` in memory.

Global planning does not:

- publish robot commands;
- own local planning or tracking;
- own map storage;
- import Gateway or driver code;
- expose planner-specific payloads as a Product API.

## Acceptance

Changes must prove the selected backend, map gate, goal correlation, stale-result
rejection, cancellation, and failure status through the native C++ tests and the
native endpoint Product acceptance flow. Simulation evidence uses the same
endpoint contract but does not replace field evidence.
