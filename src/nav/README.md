# Navigation - maps, safety, planning execution

`src/nav/` owns the runtime navigation chain: map products, safety checks,
global-plan dispatch, mission state, frontier exploration goals, tracking, and
velocity arbitration. It should not import `semantic/`, `drivers/`, or
`gateway/`; cross-layer behavior belongs in Blueprint wiring and typed ports.

## Module map

| Area | Files | Responsibility |
| --- | --- | --- |
| Mission execution | `navigation_module.py`, `waypoint_tracker.py` | Goal handling, A*/PCT planning requests, waypoint tracking, recovery, mission FSM. |
| Global planning dispatch | `global_planner_service.py` | Select A*/PCT backend, validate paths, find safe nearby goals. |
| Maps | `occupancy_grid_module.py`, `voxel_grid_module.py`, `esdf_module.py`, `elevation_map_module.py`, `traversability_cost_module.py` | L2 map layers used by navigation, safety, gateway preview, and local autonomy. |
| Safety | `safety_ring_module.py`, `plan_safety.py`, `cmd_vel_mux_module.py`, `services/geofence_manager_module.py` | Safety reflexes, geofence, plan checks, priority velocity mux. |
| Frontier exploration | `frontier_explorer_module.py`, `traversable_frontier_module.py` | Wavefront frontier goals and traversability-enriched frontier previews. |
| Endpoint bridge ports | Blueprint aliases `EndpointWaypointBridgeModule`, `EndpointPathBridgeModule`, `EndpointGridBridgeModule` | Optional external visualization/control endpoints. ROS 2 implementations live under `compat/ros2/nav/`; `nav/` keeps no ROS runtime implementation. |
| Map lifecycle | `services/map_manager_module.py` | Save/use/build/delete maps, including PGO/DUFOMap save-time cleanup. |

## Exploration and planning boundaries

These modules sound similar but are separate on purpose. Do not merge or move
them just to make the tree look tidier.

| Concept | Location | Used by | What it does | What it is not |
| --- | --- | --- | --- | --- |
| Wavefront frontier exploration | `nav/frontier_explorer_module.py` | `explore` profile through `navigation(enable_frontier=True)` | Finds free/unknown boundaries on the occupancy grid and publishes `exploration_goal` into `NavigationModule.goal_pose`. | Not selected by `exploration()` and not a semantic planner component. |
| Traversable frontier preview | `nav/traversable_frontier_module.py` | Optional navigation stack preview/inspection | Enriches wavefront candidates with traversability and optional semantic evidence, publishes ranked candidate dictionaries. | Not the TARE planner. |
| Semantic frontier scoring | `semantic/planner/frontier_scorer.py` | `SemanticPlannerModule` and goal-resolution logic | Scores frontier candidates using language, grounding, uncertainty, and semantic context. | Not a Module that drives navigation goals by itself. |
| TARE exploration | `exploration/` | `tare_explore` profile through `exploration(backend="tare")` | Runs CMU TARE hierarchical exploration via native/ROS 2 integration. | Not the wavefront explorer and not enabled in the `explore` profile. |
| A*/PCT global planning | `nav/global_planner_service.py` + `global_planning/` | `NavigationModule` | Python dispatch plus pure-Python A* and native PCT backends. | Not an exploration policy. |

Profile split:

| Profile | Exploration source | Stack settings |
| --- | --- | --- |
| `explore` | Wavefront frontier in `nav/` | `navigation(enable_frontier=True)`, `exploration_backend="none"`. |
| `tare_explore` | TARE in `exploration/` | `navigation(enable_frontier=False)`, `exploration_backend="tare"`. |
| `nav` | User/app/semantic goals | No autonomous exploration source by default. |

## Velocity ownership

All motion commands go through `CmdVelMux`.

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
tests in `src/core/tests/` only when the behavior spans Blueprint/profile/full
stack wiring. Simulation gates belong under root `sim/tests/`.
