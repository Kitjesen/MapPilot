# src/nav file guide

Use this as the quick index for the navigation package.  File names in this
folder are long because most runtime files are named after their Module or
service boundary.  Do not rename them casually: Blueprint factories, plugin
registration, tests, and external deployments may import them directly.

## Read order

| Need | Start here |
| --- | --- |
| Field native navigation data flow | `../message/idl/`, `cpp/endpoint/`, `cpp/engine/`, `cpp/include/nav_kernel/` |
| Module/sim goal-to-motion flow | `navigation.py` -> `services/plan/global_planner/service.py` -> `tracking/waypoint_tracker.py` |
| Why a plan is accepted or rejected | `services/plan/global_planner/service.py`, `services/safety/plan_safety.py`, `services/plan/global_planner/path_feasibility.py` |
| What map artifact is loaded or saved | `../maps/modules/service.py`, `../maps/services/`, `../maps/artifacts.py` |
| Live map layers | `../maps/modules/occupancy.py`, `../maps/modules/voxel_grid.py`, `../maps/modules/esdf.py`, `../maps/modules/elevation.py`, `../maps/modules/traversability.py` |
| Safety stop or velocity ownership | `services/safety/safety_ring.py`, `services/safety/velocity_mux.py`, `services/geofence.py` |
| Frontier exploration | `exploration/frontier_explorer_module.py`, `exploration/traversable_frontier_module.py` |
| Thunder Lite / mapless navigation | `services/plan/factory.py`, `services/plan/compat/direct.py`, `services/plan/compat/direct_path.py` |
| C++ local planning hot paths | `local/cpp/`, `kernel/bindings/bindings.cpp`; map-layer algorithms live in `src/maps/include/lingtu/maps/layers/` |
| Building / multi-floor missions | `building/orchestrator.py`, `building/lift.py`, `building/native_navigation.py` |
| Native command adapter | `commands/module.py`, `adapters/native/commands.py` |
| Inspection execution | `inspection/service.py`, `inspection/inspection.cpp` |

## Main runtime chains

The physical `thunder_field` chain is native C++ service + DDS. Python keeps
the task, Gateway, map-management, semantic, and status layers around it, but
does not own local planning, path following, or final `/nav/cmd_vel` publishing.

```text
lingtu-livox-dds
  -> lingtu-slam-dds
  -> lingtu-traversability-dds
  -> lingtu-nav-dds
  -> /nav/cmd_vel
  -> lingtu-driver
```

The Python Module chain below remains for simulation, local-driver, tests, and
compatibility profiles. Its Module chain and internal planner call are separate:

```text
MCP/Agent/CLI
  -> NavSkills
  -> GoalService
  -> Navigation
  -> LocalPlanner
  -> PathFollower
  -> VelocityMux
  -> Driver

Navigation._plan()
  -> PlannerService.plan(start, goal)
  -> GlobalPlanner / MaplessDirectPlannerService
  -> backend.plan(...)
```

| Piece | File |
| --- | --- |
| NavSkills | `skills/skills_module.py` |
| GoalService | `services/goals.py` |
| Navigation | `navigation.py` |
| PlannerService contract | `services/plan/contracts.py` |
| Planner service factory | `services/plan/factory.py` |
| GlobalPlanner | `services/plan/global_planner/service.py` |
| LocalPlanner | `local/local_planner.py` |
| PathFollower | `local/path_follower.py` |
| VelocityMux | `services/safety/velocity_mux.py` |
| Local planner C++ core | `local/cpp/local_planner.hpp` |
| Path follower C++ core | `kernel/include/nav_kernel/path_follower_core.hpp` |

## adapters/

Navigation-owned process-boundary adapters live here. Generic DDS mechanics
remain in `runtime/adapters`, while navigation command semantics must not.

| File | Role |
| --- | --- |
| `adapters/native/abi.py` | Owns the process-wide native command-library session and C ABI validation. |
| `adapters/native/commands.py` | Sends typed goal, cancel, stop, and teleop commands to the native navigation endpoint. |
| `adapters/native/inspection_commands.py` | Sends typed inspection lifecycle commands. |
| `adapters/native/inspection_store.py` | Accesses the native inspection route store. |
| `adapters/dds/tare_bridge.py` | DDS bridge for the external TARE process boundary. |

## Top-level Python files

| File | Role |
| --- | --- |
| `__init__.py` | Package marker plus lazy re-exports of `Navigation`, `MissionMode`, `MissionState`. |
| `navigation.py` | Mission FSM Module shell: ports, config, setup, lifecycle, health. |

## Mission execution (navigation.py + model/ + runtime/ + tracking/)

The mission layer accepts typed goals and route commands, turns them into
lifecycle events, calls the global planner, publishes waypoints, and handles
pause, cancel, timeout, stuck, and recovery.

| File | Role |
| --- | --- |
| `navigation.py` | Mission state machine shell. |
| `model/state.py` | Mission state enum, transition table, and transition rejection helpers. |
| `model/status.py` | Mission status payload builder and typed status schema. |
| `model/frame_contract.py` | Shared planning-frame checks for odometry, goals, costmaps, and saved-map artifacts. |
| `model/geometry.py` | Small geometry helpers shared by mission code. |
| `model/policy.py` | Mission policy helpers. |
| `model/recovery.py` | Recovery decision helpers. |
| `runtime/fsm.py` | Mission state/event application and status publishing. |
| `runtime/control.py` | Goal, stop, cancel, teleop, costmap, and frame-jump control handlers. |
| `runtime/planning.py` | Global planner call, path validation, path publication. |
| `runtime/execution.py` | Odometry updates, waypoint progress, completion, timeout. |
| `runtime/recovery.py` | Runtime recovery motion after stuck events. |
| `tracking/waypoint_tracker.py` | Tracks progress along a planned waypoint path and reports completion/stuck status. |

## building/

Building / multi-floor navigation orchestration (floors, lift transitions,
per-floor map and relocalization gates). The name refers to physical buildings,
not software architecture.

| File | Role |
| --- | --- |
| `building/orchestrator.py` | Floor-aware mission orchestration across maps. |
| `building/lift.py` | Lift/elevator transition execution. |
| `building/localization.py` | Floor localization adapter. |
| `building/native_navigation.py` | Native map/relocalization gates for multi-floor runs. |
| `building/model.py` | Shared building-navigation dataclasses. |

## commands/

| File | Role |
| --- | --- |
| `commands/module.py` | Typed native command adapter Module: forwards goal/cancel/stop/teleop commands to the native navigation endpoint. |

## inspection/

Inspection route execution: Python service facade plus the native C++
inspection core built into the nav endpoint.

| File | Role |
| --- | --- |
| `inspection/service.py` | Inspection service facade. |
| `inspection/inspection.cpp`, `inspection/inspection.hpp` | Native inspection execution core. |
| `inspection/store.cpp`, `inspection/store.hpp` | Native inspection route store. |
| `inspection/c_api.cpp`, `inspection/c_api.h` | C ABI used by the Python adapter. |
| `inspection/CMakeLists.txt` | Native inspection build rules. |

## localization_monitor/

| File | Role |
| --- | --- |
| `localization_monitor/monitor_module.py` | Watches localization quality and raises pause/stop signals. |

## services/plan/

| File | Role |
| --- | --- |
| `services/plan/contracts.py` | Protocols for planner backends and planner services. `Navigation` should depend on this boundary, not a concrete planner. |
| `services/plan/factory.py` | Creates either map-backed `GlobalPlanner` or mapless `MaplessDirectPlannerService`. |
| `services/plan/global_planner/service.py` | Map-backed global planner coordinator. Selects OctoPlanner3D by default, keeps PCT as an explicit legacy backend, validates map artifacts, and reports diagnostics. |
| `services/plan/compat/direct.py` | Lightweight planner service for Thunder Lite/local runtimes that must avoid map-backed planner imports. |
| `services/plan/compat/direct_path.py` | Direct start-to-goal planner used by mapless mode. |
| `services/plan/global_planner/algorithm/octoplanner3d.py` | LingTu OctoPlanner3D runtime binding and planner registration. |
| `services/plan/global_planner/algorithm/far.py` | Thin native FAR registration; no Python planner fallback. |
| `cpp/planning/global/octoplanner/` | Canonical product OctoPlanner3D source, headless runtime, and system-OctoMap build boundary. |
| `cpp/planning/global/far/` | Independent native 2D visibility-graph planner and stable C ABI. |
| `services/plan/global_planner/algorithm/pct/` | Explicit legacy/manual PCT runtime, loader, and vendored PCT planner sources. Not the default product backend. |
| `services/plan/global_planner/path_feasibility.py` | Ground path feasibility checks such as z excursion, slope, and segment length. |

## local/

| File | Role |
| --- | --- |
| `local/local_planner.py` | Local planner Module boundary. Receives odometry/terrain/waypoint/global path and publishes `local_path` plus `control_hint`. |
| `local/local_planner_runtime.py` | Runtime backend setup and fail-closed native validation. |
| `local/native.py` | Creates the `lingtu_nav_kernel.LocalPlanner` nanobind backend. |
| `local/cmu_py.py` | Explicit development-only Python scorer. |
| `local/parameters.py` | Runtime config to C++/Python local planner parameter mapping. |
| `local/path_tables.py` | CMU PLY/text path-bank loader. |
| `local/geometry.py` | Path-point coercion, planning-origin, corridor-goal, and line helpers. |
| `local/obstacles.py` | Terrain/boundary/added-obstacle point-cloud merge helpers. |
| `local/models.py` | Shared backend dataclasses and CMU constants. |
| `local/local_planner_backend.py` | Typed backend bundles used by runtime setup. |
| `local/path_follower.py` | Converts `local_path` into velocity commands. |
| `local/path_follower_runtime.py` | Path-follower backend setup. |
| `local/path_follower_backend.py` | Path-follower backend bundles. |
| `local/terrain.py` | Terrain analysis Module boundary. |
| `local/terrain_backend.py` | Terrain backend bundles. |
| `local/output.py` | Local-path output helpers. |
| `local/pcl_ops.py` | Point-cloud operations helpers. |
| `local/stack.py` | Local execution stack wiring helpers. |
| `local/contracts.py` | Local planner/follower contract types. |
| `local/cpp/local_planner.hpp` | Full C++ local-planner implementation and optimized data layout. |
| `local/cpp/local_planner_scoring.hpp` | C++ local-planner scoring and voxel helpers. |
| `local/paths/` | Precomputed CMU candidate path bank used by the local planner. |

## maps/ (lives at src/maps)

Realtime map-layer Modules live under `src/maps/modules/`. This is L2
spatial state, not mission logic. Mission consumes these outputs through wires
such as `TraversabilityCostModule.fused_cost`.

| File | Role |
| --- | --- |
| `../maps/modules/occupancy.py` | Builds 2D occupancy and exploration grids from point clouds. |
| `../maps/modules/voxel_grid.py` | Builds and queries a 3D voxel map from point clouds. |
| `../maps/modules/esdf.py` | Builds a distance field from occupancy. |
| `../maps/modules/elevation.py` | Builds elevation statistics from point clouds. |
| `../maps/modules/traversability.py` | Fuses occupancy, ESDF, elevation slope/step/roughness, and terrain risk into navigation cost. |
| `../maps/modules/service.py` | Persistent map lifecycle Module facade over the native maps service. |

## cpp/

ROS-free C++ algorithm kernels. Python Modules call these through
`lingtu_nav_kernel`, but the kernels do not import runtime, ROS, PCL, or gateway code.

| File | Role |
| --- | --- |
| `cpp/include/nav_kernel/path_follower_core.hpp` | Path follower control math. |
| `cpp/include/nav_kernel/terrain_core.hpp` | Local terrain analysis from point clouds. |
| `cpp/include/nav_kernel/simd_accel.hpp` | SIMD acceleration helpers. |
| `cpp/include/nav_kernel/types.hpp` | Shared C++ nav types. |
| `cpp/include/nav_kernel/validation.hpp` | C++ validation helpers. |
| `cpp/bindings/` | Nanobind bridge exposing portable kernels as `lingtu_nav_kernel`. |
| `cpp/tests/` | Portable C++ planner/follower/FAR tests. |
| `cpp/endpoint/` | Product DDS process, active artifact gates, final safety, and endpoint tests. |

## services/safety/

Safety services own reflex stop, plan safety, and velocity arbitration.

| File | Role |
| --- | --- |
| `services/safety/plan_safety.py` | Shared path/cost safety checks used by planner services. |
| `services/safety/safety_ring.py` | Runtime safety evaluator and stop publisher. |
| `services/safety/velocity_mux.py` | Final velocity arbiter before the driver. |

## exploration/

| File | Role |
| --- | --- |
| `exploration/frontier_explorer_module.py` | Wavefront frontier explorer that publishes exploration goals into `Navigation`. |
| `exploration/traversable_frontier_module.py` | Traversability-aware frontier preview/candidate ranking. Does not own robot motion by itself. |
| `exploration/tare/` | CMU TARE hierarchical exploration integration (module, policy, supervisor, topics). |

## services/

| File | Role |
| --- | --- |
| `../maps/modules/service.py` | Saved-map lifecycle Module facade: save, list, use, delete, build artifacts, expose map skills. |
| `../maps/services/` | Internal helpers for `MapsModule`: storage, records, command routing, runtime snapshot bridge, and artifact build pipeline. |
| `services/goals.py` | Map-frame goal entry. Converts JSON/Gateway/MCP goal requests into `goal_pose`, `patrol_goals`, or `cancel`. |
| `services/frame_transforms.py` | Frame normalization helpers for goal and map data. |
| `services/geofence.py` | Polygon geofence monitor and hard-stop source. |
| `services/patrol.py` | Stores patrol routes and emits patrol waypoint sequences (`PatrolManagerModule`). |
| `services/scheduler.py` | Simple queued task scheduler for navigation-related commands. |

## tests/

| Test file | Main coverage |
| --- | --- |
| `test_nav_modules.py` | Legacy broad tests for tracker, planner service, map layers, and safety. |
| `test_navigation_frame_contract.py` | Planning-frame, goal, retry, stop/cancel, and mission-state contracts. |
| `test_mission_state.py` | Mission state enum and transition table behavior. |
| `test_global_planner_diagnostics.py` | Planner backend diagnostics, fallback, OctoPlanner3D artifact gate behavior. |
| `test_global_planner_contracts.py` | Global planner backend contract checks. |
| `test_nav_services.py` | Broad service/module tests, including ROS2-marked compatibility coverage. |
| `test_goal_service.py` | Goal entry validation and routing behavior. |
| `test_planning_map_artifacts.py` | Map artifact validation for planning. |
| `test_planning_service_factory.py` | Planner service factory and mapless-vs-map-backed import boundary. |
| `test_plan_service_layout.py` | Planner service file layout guardrails. |
| `test_lite_planner_backend.py` | DirectPath backend behavior. |
| `test_path_feasibility.py` | Ground path feasibility checks. |
| `test_velocity_mux_contract.py` | Velocity mux priority/timeout contract. |
| `test_safety_ring_contract.py` | Safety ring public contract. |
| `test_geofence.py` | Geofence command and intrusion behavior. |
| `test_patrol.py` | Patrol route storage and dispatch behavior (`services/patrol.py`). |
| `test_scheduler.py` | Task scheduler command behavior. |
| `test_nav_corridor_scenario.py` | Corridor scenario and A* detour behavior. |
| `test_nav_semantic.py` | Shared message and semantic nav primitives. |
| `test_nav_skills.py` | NavSkills L6 adapter, compatibility alias, and profile wiring. |
| `test_command_module.py`, `test_command_client.py` | Native command adapter module and client. |
| `test_inspection_service.py`, `test_inspection_command_client.py` | Inspection facade and native inspection commands. |
| `test_building_mission_orchestrator.py`, `test_lift_transition_executor.py`, `test_floor_localization_adapter.py`, `test_native_building_navigation_port.py` | Building / multi-floor orchestration coverage. |
| `test_open_rmf_*.py` | Open-RMF sidecar and single-robot bridge lifecycle. |
| `test_tomogram_builder_no_open3d.py` | Tomogram artifact builder fallback without Open3D. |

## Naming rules

- `*_module.py`: runtime Module with ports, lifecycle, registry, and often MCP skills.
- `*_service.py`: service boundary or coordinator used by a Module.
- `*_builder.py`: artifact construction helper.
- `*_contract.py`: frame or interface contract logic.
- `cpp/`: canonical C++ hot paths; `kernel/` is the Python extension loader and `local/` owns Python runtime/assets.
