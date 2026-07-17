# Navigation - maps, safety, planning execution

`src/nav/` owns the runtime navigation chain: map products, safety checks,
global-plan dispatch, mission state, frontier exploration goals, tracking, and
velocity arbitration. It should not import `decision/`, `perception/`, `drivers/`, or
`gateway/`; cross-layer behavior belongs in Blueprint wiring and typed ports.

For a file-by-file purpose index, start with `FILES.md`.

## Runtime chains

The default physical `thunder_field` product path is a native endpoint chain:

```text
MCP/Agent/CLI
  -> NavSkills
  -> GoalService
  -> native field endpoint boundary
  -> lingtu-nav-dds
  -> DDS /nav/cmd_vel
```

In that branch, Python owns goals, mission/status, map products, semantic
coordination, and safety contracts. C++ `lingtu-nav-dds` owns the final field
`/nav/cmd_vel` writer so the robot does not receive duplicate velocity
commands.

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
| LocalPlanner | `src/nav/services/plan/local_planner/service.py` | Local obstacle-avoidance Module around `nav_kernel`. |
| PathFollower | `src/nav/local/path_follower.py` | Converts `local_path` into velocity commands. |
| VelocityMux | `src/nav/services/safety/velocity_mux.py` | Final velocity arbiter before the driver. |
| nav_kernel C++ | `src/nav/kernel/include/nav_kernel/` | Hot-path local planner, terrain, and path follower kernels. |

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
- Normal navigation modules stay Module-First and ROS-free. ROS 2 code belongs
  under `*/adapters/ros2/`; `nav/` keeps no ROS runtime implementation.
- OctoPlanner3D is the map-backed global planning path for real saved-map /
  3D-map navigation. A*, PCT, and direct modes remain compatibility, simulation,
  or lightweight profile paths unless a profile explicitly selects them.

## Global planner contract

- Service entry: `services/plan/factory.py:create_planner_service(...)`
  returns a `PlannerService` for `Navigation`.
- Map-backed runtime: `services/plan/global_planner/service.py:GlobalPlanner`.
  Thunder Lite/mapless runtime uses `services/plan/compat/direct.py`.
- Backend registry key: `planner_backend`. Backend classes are constructed as
  `BackendCls(tomogram_path, obstacle_thr)`.
- Backend function names are mandatory: `plan(start, goal)` and
  `update_map(grid, resolution=0.2, origin=None)`. The service boundary rejects
  registered backends missing either callable.
- `start`, `goal`, and returned path points are in the active planning frame
  (`map` by default). Backends do not perform TF conversion; adapters must
  normalize external data before it reaches `Navigation`.
- A global planner returns sparse route waypoints only. It never publishes
  `cmd_vel`; local planning, path following, safety, and `VelocityMux` own motion.

## Module map

| Area | Files | Responsibility |
| --- | --- | --- |
| Mission execution | `navigation.py`, `model/`, `runtime/`, `tracking/` | Goal handling, planning requests, waypoint tracking, recovery, and mission FSM. |
| Global planning dispatch | `services/plan/global_planner/service.py` | Select OctoPlanner3D/A*/PCT planner, validate paths, find safe nearby goals. |
| Planner service boundary | `services/plan/` | `Navigation`'s planner boundary. `services/plan/factory.py` chooses map-backed `GlobalPlanner` or mapless `MaplessDirectPlannerService`; `services/plan/compat/direct_path.py` owns the Thunder Lite direct planner. |
| Maps | `services/map/layers/`, `services/maps.py` | L2 realtime map layers plus the saved-map lifecycle service used by navigation, safety, gateway preview, and local autonomy. |
| Safety | `safety/`, `services/geofence.py` | Safety reflexes, geofence, plan checks, priority velocity mux. |
| Frontier exploration | `exploration/` | Wavefront frontier goals and traversability-enriched frontier previews. |
| Process boundary | C++ `lingtu-nav-dds` | Owns typed DDS goal/cancel input and global/local path plus final command output. The Python graph has no `nav.in` or `nav.out` adapter modules. Map visualization remains a separate map-domain concern. |
| Navigation services | `services/` | Map lifecycle, goal commands, patrol routes, optional schedules, and geofence state. |
| Map lifecycle | `services/maps.py` | Save/use/build/delete maps through the native map-save adapter, map optimization metadata, and artifact builders. |
| C++ hot path | `kernel/` | Header-first local planner/path follower/terrain kernels plus nanobind binding and C++ tests. Pytest does not cover this directory. |

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
| Wavefront frontier exploration | `nav/exploration/frontier_explorer_module.py` | `explore` profile through `navigation(enable_frontier=True)` | Finds free/unknown boundaries on the occupancy grid and publishes `exploration_goal` into `Navigation.goal_pose`. | Not selected by `exploration()` and not a semantic planner component. |
| Traversable frontier preview | `nav/exploration/traversable_frontier_module.py` | Optional navigation stack preview/inspection | Enriches wavefront candidates with traversability and optional semantic evidence, publishes ranked candidate dictionaries. | Not the TARE planner. |
| Semantic frontier scoring | `decision/frontier_scorer.py` | `SemanticPlanner` and goal-resolution logic | Scores frontier candidates using language, grounding, uncertainty, and semantic context. | Not a Module that drives navigation goals by itself. |
| TARE exploration | `nav/exploration/tare/` | `tare_explore` profile through the navigation exploration stack | Runs CMU TARE hierarchical exploration via native/adapter integration. | Not the wavefront explorer and not enabled in the `explore` profile. |
| OctoPlanner3D/PCT global planning | `nav/services/plan/global_planner/service.py`, OctoPlanner3D and explicit PCT compatibility under `nav/services/plan/global_planner/algorithm/` | `Navigation` | Python dispatch plus headless OctoPlanner3D and explicit PCT compatibility planners. | Not an exploration policy. |

Profile split:

| Profile | Exploration source | Stack settings |
| --- | --- | --- |
| `explore` | Wavefront frontier in `nav/` | `navigation(enable_frontier=True)`, `exploration_backend="none"`. |
| `tare_explore` | TARE in `nav/exploration/tare/` | `navigation(enable_frontier=False)`, `exploration_backend="tare"`. |
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

The C++ tests under `src/nav/kernel/tests/` require the CMake flow from the root
project guidance; they are not collected by pytest.
