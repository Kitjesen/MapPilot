# Navigation - maps, safety, planning execution

`src/nav/` owns navigation behavior: safety checks, goal command handling,
global-plan dispatch, mission state, frontier exploration goals, inspection
route execution, local path tracking, and velocity arbitration. Persistent map
products live in `src/maps/`. `nav/` should not import `decision/`,
`perception/`, `drivers/`, or `gateway/`; cross-layer behavior belongs in
Blueprint wiring and typed ports.

For a file-by-file purpose index, start with `FILES.md`.

## Runtime chains

The default `env=real` Product path is a native service chain:

```text
MCP/Agent/CLI
  -> NavSkills
  -> GoalService
  -> native field endpoint boundary
  -> lingtu-nav-dds
  -> DDS /nav/cmd_vel
  -> lingtu-driver
```

In that branch, Python owns goal admission, Gateway/MCP coordination, map
service requests, semantic coordination, and status presentation. C++
`lingtu-nav-dds` owns the final field `/nav/cmd_vel` writer, and the native
Thunder `lingtu_driver` process is the unique hardware sink.

Simulation, local-driver, and compatibility profiles can still run the
Module-owned local execution chain:

```text
MCP/Agent/CLI
  -> NavSkills
  -> GoalService
  -> Navigation
  -> LocalPlanner
  -> PathFollower
  -> VelocityMux
  -> Driver
```

`PlannerService` is not a Module in that chain. It is an internal planning
interface owned by `Navigation`:

```text
Navigation._plan()
  -> PlannerService.plan(start, goal)
  -> GlobalPlanner or MaplessDirectPlannerService
  -> backend.plan(...)
```

Main implementation locations:

| Runtime piece | File | Role |
| --- | --- | --- |
| NavSkills | `src/nav/skills/skills_module.py` | L6 MCP/agent command adapter and status reader. |
| GoalService | `src/nav/services/goals.py` | External goal/cancel/patrol command entry. |
| Navigation | `src/nav/navigation.py` | Mission FSM, global planning call, waypoint dispatch. |
| PlannerService | `src/nav/services/plan/contracts.py` | Internal planner interface consumed by `Navigation`. |
| GlobalPlanner | `src/nav/services/plan/global_planner/service.py` | Map-backed global planning coordinator. |
| LocalPlanner | `src/nav/local/local_planner.py` | Local obstacle-avoidance Module around `nav_kernel`. |
| PathFollower | `src/nav/local/path_follower.py` | Converts `local_path` into velocity commands. |
| VelocityMux | `src/nav/services/safety/velocity_mux.py` | Final velocity arbiter before the driver. |
| Native field endpoint | `src/nav/cpp/endpoint/` | Typed DDS boundary, global planner, local loop, safety, and final command authority. |
| Navigation C++ | `src/nav/cpp/` | FAR/OctoPlanner3D, local planner, path follower, client ABI, and endpoint build. |

## Product contracts

- A goal is never a motor command. Goals enter `Navigation`, become a
  mission, pass through global planning, waypoint tracking, local planning /
  following, velocity muxing, and safety checks before a driver can receive
  motion.
- Frontend, CLI, MCP, semantic planner, and patrol routes are goal sources.
  They provide target intent and display state; they do not own path planning
  or obstacle avoidance.
- Navigation start pose comes from localization/odometry. Frontend clients
  provide destination goals only, except for explicit simulation or diagnostic
  tooling.
- Development and simulation navigation Modules stay Host-scoped and ROS-free. ROS 2 code belongs
  under `*/adapters/ros2/`; `nav/` keeps no ROS runtime implementation.
- OctoPlanner3D is the default map-backed global planner for 3D saved maps.
  Native FAR is an explicit 2D occupancy option; it never activates as a
  silent fallback. A*, PCT, and direct modes remain compatibility, simulation,
  or lightweight paths outside the native field backend catalog.

## Global planner contract

- Service entry: `services/plan/factory.py:create_planner_service(...)`
  returns a `PlannerService` for `Navigation`.
- Map-backed runtime: `services/plan/global_planner/service.py:GlobalPlanner`.
  The `lite` Profile mapless runtime uses `services/plan/mapless/direct.py`.
- Backend registry key: `planner_backend`. Backend classes are constructed as
  `BackendCls(map_path, obstacle_thr)`.
- Backend function names are mandatory: `plan(start, goal)` and
  `update_map(grid, resolution=0.2, origin=None)`. The service boundary rejects
  registered backends missing either callable.
- `start`, `goal`, and returned path points are in the active planning frame
  (`map` by default). Backends do not perform TF conversion; adapters must
  normalize external data before it reaches `Navigation`.
- A global planner returns sparse route waypoints only. It never publishes
  `cmd_vel`; local planning, path following, safety, and `VelocityMux` own motion.

## Mission lifecycle

`Navigation` (runtime id `nav.mission`) owns the navigation task lifecycle. It
is not a planner backend and it is not a service API layer.

```text
nav.services.goals / nav.services.patrol / gateway
  -> nav.mission Navigation
  -> nav.services.plan GlobalPlanner
  -> nav.local LocalPlanner
  -> nav.local PathFollower
  -> nav.services.safety VelocityMux
```

- Services parse commands and own assets; mission owns execution state.
- Mission calls `GlobalPlanner`; services should not push waypoint execution.
- LocalPlanner and PathFollower are downstream runtime modules, not mission
  subroutines.
- New mission behavior goes into `runtime/` only when it changes task
  execution. Data definitions stay in `model/`, tracking in `tracking/`.

## Module map

| Area | Files | Responsibility |
| --- | --- | --- |
| Mission execution | `navigation.py`, `model/`, `runtime/`, `tracking/` | Goal handling, planning requests, waypoint tracking, recovery, and mission FSM. |
| Building / multi-floor | `building/` | Floor-aware mission orchestration, lift transitions, and native map/relocalization gates. "Building" means physical buildings (floors, elevators), not software architecture. |
| Native command adapter | `commands/module.py` | Typed goal/cancel/stop/teleop command forwarding to the native navigation endpoint. |
| Inspection | `inspection/` | Inspection route execution facade (`service.py`) plus the native C++ core built into the nav endpoint. |
| Localization monitor | `localization_monitor/` | Localization quality watch and pause/stop signaling. |
| Global planning dispatch | `services/plan/global_planner/service.py`, `cpp/planning/global/` | Select OctoPlanner3D by default, admit explicit native FAR, validate maps/paths, and keep PCT manual-only. |
| Planner service boundary | `services/plan/` | `Navigation`'s planner boundary. `services/plan/factory.py` chooses map-backed `GlobalPlanner` or mapless `MaplessDirectPlannerService`; `services/plan/mapless/direct_path.py` owns the `lite` Profile direct planner. |
| Maps | `../maps/modules/`, `../maps/services/` | L2 realtime map layers plus the saved-map lifecycle service used by navigation, safety, gateway preview, and local autonomy. |
| Safety | `services/safety/`, `services/geofence.py` | Safety reflexes, geofence, plan checks, priority velocity mux. |
| Frontier exploration | `exploration/` | Wavefront frontier goals and traversability-enriched frontier previews. |
| Process boundary | `cpp/endpoint/navd` (`lingtu-nav-dds.service`) | Owns typed DDS goal/cancel/inspection input, active map gates, global/local planning, and final command output. The Python graph has no competing field motion writer. |
| Navigation services | `services/` | Map lifecycle, goal commands, patrol routes, optional schedules, and geofence state. |
| Map lifecycle | `../maps/modules/service.py`, `../maps/services/` | Save/use/build/delete maps through the native maps service contract, map optimization metadata, and artifact builders. |
| C++ hot path | `cpp/` | Local planner/path follower kernels, native global planners, nanobind binding, endpoint, and C++ tests. |

## Service set

| Service | Module | Status |
| --- | --- | --- |
| MCP/agent navigation | `NavSkills` | Mounted as `nav.skills`; all commands pass through `GoalService`. |
| Saved maps and POIs | `MapsModule` | Mounted by `maps()` when map modules are enabled. |
| Goal commands | `GoalService` | Mounted by the full stack service layer; Gateway/MCP coordinate goals and cancels pass through it before `Navigation`. |
| Patrol routes | `PatrolManagerModule` | Mounted by the full stack service layer; emits route waypoints to `Navigation`. |
| Scheduled patrols | `TaskSchedulerModule` | Implemented but opt-in; enable with `enable_scheduler=True`. |
| Geofence | `GeofenceManagerModule` | Mounted by `safety()` and wired to stop navigation/driver. |

## Removed compatibility files

Removed compatibility surfaces should not be reintroduced unless a live profile,
package contract, or external deployment still imports them. In particular, the
old root-level navigation facades were removed; import from the owning
subpackage instead.

## Exploration and planning boundaries

These modules sound similar but are separate on purpose. Do not merge or move
them just to make the tree look tidier.

| Concept | Location | Used by | What it does | What it is not |
| --- | --- | --- | --- | --- |
| Wavefront frontier exploration | `../explore/frontier.py` | Local development/simulation compatibility graph | Finds free/unknown boundaries on the occupancy grid and publishes exploration goals. | Not the field `explore` Product endpoint. |
| Traversable frontier preview | `../explore/traversable_frontier.py` | Optional navigation-stack preview/inspection | Enriches wavefront candidates with traversability and optional semantic evidence, publishes ranked candidate dictionaries. | Not the TARE planner. |
| Semantic frontier scoring | `decision/frontier_scorer.py` | `SemanticPlanner` and goal-resolution logic | Scores frontier candidates using language, grounding, uncertainty, and semantic context. | Not a Module that drives navigation goals by itself. |
| TARE policy | `../explore/tare/` and `../explore/cpp/` | Native field `explore` endpoint and local compatibility graphs | Selects exploration targets through the native/adapter integration. | An internal algorithm name, not another Product. |
| Global planning | `nav/cpp/planning/global/`, plus Python Module/sim dispatch under `nav/services/plan/global_planner/` | `navd` or `Navigation` | Native OctoPlanner3D default, explicit FAR option, and Python simulation adapters; PCT is legacy/manual only. | Not an exploration policy. |

Product/local graph split:

| Selection | Exploration source | Stack settings |
| --- | --- | --- |
| `lingtu explore start` | Native TARE policy, `Live` route | Mapping plus identity-bound rolling segments; no saved-map GlobalPlanner goal. |
| `lingtu explore start --map MAP` | Native TARE policy, `Map` route | Localization against the exact saved map, then normal global/local navigation. |
| Local compatibility Profile | Wavefront or TARE adapter | Explicit Blueprint development selection; never a field Product fallback. |
| `nav` | User/app/semantic goals | No autonomous exploration source by default. |

## Velocity ownership

All motion commands go through `VelocityMux`.

| Source | Priority | Timeout |
| --- | ---: | ---: |
| Teleop joystick | 100 | 0.5 s |
| VisualServo PD tracking | 80 | 0.5 s |
| Navigation recovery | 60 | 0.5 s |
| PathFollower autonomy | 40 | 0.5 s |

Highest-priority active source wins. New motion producers must publish through
the mux instead of writing directly to a driver.

## Tests

Put package-owned unit tests under `src/nav/tests/`. Put cross-package contract
tests in `src/runtime/tests/` only when the behavior spans Blueprint/profile/full
stack wiring. Simulation gates belong under root `sim/tests/`.

Recommended Python baselines:

```bash
python -m pytest src/nav/tests/ -m "not ros2 and not sim"
python -m pytest src/nav/tests/ -m "not ros2"
python -m pytest src/nav/tests -q
```

The C++ tests under `src/nav/cpp/tests/` require the CMake flow from the root
project guidance; they are not collected by pytest.
