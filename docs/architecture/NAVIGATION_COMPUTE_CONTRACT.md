# Navigation Compute Contract

Status: current contract
Audience: navigation compute, planner, safety, and driver-boundary maintainers
Replaced by: not replaced

This document defines the compute boundary for global planning, local planning,
safety checks, and velocity output. It is intentionally narrower than the full
system design.

## 1. Ownership

| Layer | Owner | Responsibility | Not responsible for |
| --- | --- | --- | --- |
| Mission | `src/nav/navigation.py` | Goal lifecycle, mission FSM, recovery, status. | Algorithm internals. |
| Global planning service | `src/nav/services/plan/` | `GlobalPlanRequest -> GlobalPlanResult`, map-backed planner dispatch. | Module graph, UI schema, robot driver commands. |
| Native global planner | `src/nav/cpp/planning/global/` | OctoPlanner3D default and explicit FAR implementation. | Map storage, transport, mission state. |
| Module/sim planner adapters | `src/nav/services/plan/global_planner/algorithm/` | Python adapter/registry boundary and legacy manual planners. | Field motion authority. |
| Map safety | `src/nav/services/safety/`, map modules | Costmap/traversability safety gates. | Local trajectory scoring. |
| Local planning | `src/nav/local/`, `src/nav/local/` | Short-horizon trajectory and control hints. | Semantic goal resolution. |
| Path following | `src/nav/local/path_follower.py` | Convert local path into velocity intent. | Velocity priority arbitration. |
| Velocity mux | `src/nav/services/safety/velocity_mux.py` | Choose the active velocity source. | Path generation. |

## 2. Global Planner Contract

Code source: `src/nav/services/plan/contracts.py`.

```text
GlobalPlanRequest
  start: [x, y, z]
  goal: [x, y, z]
  safe_goal_tolerance: float
  frame_id: string
  request_id: string
  map_version: string

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

Rules:

- `Navigation` depends on `GlobalPlanRequest` and `GlobalPlanResult`, not an
  OctoPlanner3D-specific payload.
- `GlobalPlanner` may run safe-goal search, path safety repair, fallback
  selection, and downsampling, but it must report those decisions in
  `GlobalPlanResult.report`.
- `GlobalPlanResult.to_wire()` is the UI/SDK-facing JSON shape.
- Gateway may forward `global_plan` as data. Gateway must not import `nav`
  internals to understand planner classes.

## 3. Data Flow

```text
goal_pose + odometry + map artifacts
  -> Navigation
  -> GlobalPlannerService.plan_request()
  -> GlobalPlanResult
  -> Navigation.global_path
  -> LocalPlannerModule
  -> PathFollowerModule
  -> CmdVelMux
  -> Driver
```

Safety fan-in:

```text
OccupancyGrid / ESDF / Elevation / Traversability
  -> TraversabilityCostModule.fused_cost
  -> Navigation.costmap
  -> global replan or safety rejection
```

## 4. Map Inputs

`GlobalPlanningMap` is the service-level map payload:

```text
grid: 2-D float grid
resolution: meters/cell
origin: [x, y] | null
frame_id: map frame
map_version: string
source: string
```

Planner backends may maintain richer native state, but the service boundary
must accept this map shape for live costmap updates and tests.

## 5. What Costmap Means

`costmap` is a global safety and replanning input. It is not the local planner's
primary scoring representation.

Allowed:

- update the global planner's live safety grid;
- reject unsafe goals or unsafe paths;
- trigger global replanning;
- expose safety state to Gateway.

Not allowed:

- silently replace terrain/local planner scoring with fused costmap values;
- wire `TraversabilityCostModule.fused_cost` directly into local planner inputs
  unless the local planner contract is explicitly upgraded and tested.

## 6. Required Wires

These wires must exist in product navigation graphs:

```text
SLAM/Driver.odometry -> NavigationModule.odometry
TraversabilityCostModule.fused_cost -> NavigationModule.costmap
NavigationModule.global_path -> LocalPlannerModule.global_path
NavigationModule.waypoint -> LocalPlannerModule.waypoint
TerrainModule.terrain_map -> LocalPlannerModule.terrain_map
LocalPlannerModule.local_path -> PathFollowerModule.local_path
LocalPlannerModule.control_hint -> PathFollowerModule.control_hint
PathFollowerModule.cmd_vel -> CmdVelMux.path_follower_cmd_vel
CmdVelMux.driver_cmd_vel -> Driver.cmd_vel
SafetyRing.stop_cmd -> Driver.stop_signal
```

## 7. Frame Contract

Global and local planning use a map/world planning frame. Body-frame data must
be transformed before entering global planning. If odometry, costmap, or goal
frames disagree, Navigation must reject or block planning rather than guessing.

The detailed frame rules live in `ros_frame_contract.md`.

## 8. Backend Policy

| Backend | Role |
| --- | --- |
| `octoplanner3d` | Default map-backed global planner. |
| `far` | Explicit native 2D planner over validated active occupancy. No silent fallback. |
| `pct` | Legacy/manual experiment backend. |
| `direct` | Mapless or explicit direct-goal path only. |

Backend-specific subprocesses, native binaries, diagnostics, and payload schemas
belong behind the planner backend boundary. They must not leak into
`Navigation`, Gateway, or UI code.

## 9. Validation

Required checks after touching this contract:

```bash
python -m pytest src/nav/tests/test_global_planner_contracts.py -q
python -m pytest src/runtime/tests/test_nav_chain_efficiency.py -q
python -m pytest src/nav/tests/test_navigation_frame_contract.py -q
```

Broader checks:

```bash
python -m pytest src/runtime/tests/test_stack_registry_resolution.py -q
python -m pytest src/gateway/tests/test_gateway_commands.py -k "preview or click_navigation" -q
```

