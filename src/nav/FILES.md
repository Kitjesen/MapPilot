# src/nav file guide

Use this as the quick index for the navigation package.  File names in this
folder are long because most runtime files are named after their Module or
service boundary.  Do not rename them casually: Blueprint factories, plugin
registration, tests, and external deployments may import them directly.

## Read order

| Need | Start here |
| --- | --- |
| Field native navigation data flow | `../message/idl/`, `services/endpoint/cpp/`, `services/plan/cpp/`, `kernel/include/nav_kernel/` |
| Module/sim goal-to-motion flow | `mission/navigation.py` -> `services/plan/global_planner/service.py` -> `mission/tracking/waypoint_tracker.py` |
| Why a plan is accepted or rejected | `services/plan/global_planner/service.py`, `services/safety/plan_safety.py`, `services/plan/global_planner/path_feasibility.py` |
| What map artifact is loaded or saved | `services/maps.py`, `services/map_layers/map_artifact_builder.py` |
| Live map layers | `services/map_layers/occupancy_grid_module.py`, `services/map_layers/voxel_grid_module.py`, `services/map_layers/esdf_module.py`, `services/map_layers/elevation_map_module.py`, `services/map_layers/traversability_cost_module.py` |
| Safety stop or velocity ownership | `services/safety/safety_ring.py`, `services/safety/velocity_mux.py`, `services/geofence.py` |
| Frontier exploration | `exploration/frontier_explorer_module.py`, `exploration/traversable_frontier_module.py` |
| Thunder Lite / mapless navigation | `services/plan/factory.py`, `services/plan/compat/direct.py`, `services/plan/compat/direct_path.py` |
| C++ map/local planning hot paths | `kernel/include/nav_kernel/map_layers_core.hpp`, `services/plan/local_planner/cpp/`, `kernel/bindings/bindings.cpp` |

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
```

The Python Module chain below remains for simulation, local-driver, tests, and
compatibility profiles. Its Module chain and internal planner call are separate:

```text
Gateway/MCP/CLI
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
| GoalService | `services/goals.py` |
| Navigation | `mission/navigation.py` |
| PlannerService contract | `services/plan/contracts.py` |
| Planner service factory | `services/plan/factory.py` |
| GlobalPlanner | `services/plan/global_planner/service.py` |
| LocalPlanner | `services/plan/local_planner/service.py` |
| PathFollower | `local/path_follower.py` |
| VelocityMux | `services/safety/velocity_mux.py` |
| Local planner C++ core | `services/plan/local_planner/cpp/local_planner.hpp` |
| Path follower C++ core | `kernel/include/nav_kernel/path_follower_core.hpp` |

## Top-level Python files

| File | Role | Runtime type |
| --- | --- | --- |
| `__init__.py` | Package marker only. | Package |

## services/plan/

| File | Role |
| --- | --- |
| `services/plan/contracts.py` | Protocols for planner backends and planner services. `Navigation` should depend on this boundary, not a concrete planner. |
| `services/plan/factory.py` | Creates either map-backed `GlobalPlanner` or mapless `MaplessDirectPlannerService`. |
| `services/plan/global_planner/service.py` | Map-backed global planner coordinator. Selects OctoPlanner3D by default, keeps PCT as an explicit legacy backend, validates map artifacts, and reports diagnostics. |
| `services/plan/compat/direct.py` | Lightweight planner service for Thunder Lite/local runtimes that must avoid map-backed planner imports. |
| `services/plan/compat/direct_path.py` | Direct start-to-goal planner used by mapless mode. |
| `services/plan/global_planner/algorithm/octoplanner3d.py` | LingTu OctoPlanner3D runtime binding and planner registration. |
| `services/plan/global_planner/algorithm/OctoPlanner3D/` | Full OctoPlanner3D algorithm source plus LingTu native headless bridge. |
| `services/plan/global_planner/algorithm/pct/` | Long-term PCT algorithm, runtime loader, and vendored PCT planner sources. |
| `services/plan/global_planner/path_feasibility.py` | Ground path feasibility checks such as z excursion, slope, and segment length. |
| `services/plan/local_planner/service.py` | Local planner Module boundary. Receives odometry/terrain/waypoint/global path and publishes `local_path` plus `control_hint`. |
| `services/plan/local_planner/runtime.py` | Runtime backend setup for `nanobind`, `cmu_py`, and `simple`. |
| `services/plan/local_planner/native.py` | Creates the `lingtu_nav_kernel.LocalPlanner` nanobind backend and the `cmu_py` path-table bundle. |
| `services/plan/local_planner/cmu_py.py` | Pure-Python CMU local planner scorer and path decision fallback. |
| `services/plan/local_planner/parameters.py` | Runtime config to C++/Python local planner parameter mapping. |
| `services/plan/local_planner/path_tables.py` | CMU PLY/text path-bank loader. |
| `services/plan/local_planner/geometry.py` | Path-point coercion, planning-origin, corridor-goal, and line helpers. |
| `services/plan/local_planner/obstacles.py` | Terrain/boundary/added-obstacle point-cloud merge helpers. |
| `services/plan/local_planner/models.py` | Shared backend dataclasses and CMU constants. |
| `services/plan/local_planner/backend.py` | Compatibility exports only; do not add new implementation here. |
| `services/plan/local_planner/cpp/local_planner.hpp` | Full C++ local-planner implementation and optimized data layout. |
| `services/plan/local_planner/cpp/local_planner_scoring.hpp` | C++ local-planner scoring and voxel helpers. |
| `services/plan/local_planner/paths/` | Precomputed CMU candidate path bank used by the local planner. |

## mission/

| File | Role |
| --- | --- |
| `mission/navigation.py` | Mission state machine. Receives goals/instructions, calls the planner service, publishes global paths and waypoint commands, handles recovery and stop/cancel. |
| `mission/tracking/waypoint_tracker.py` | Tracks progress along a planned waypoint path and reports completion/stuck status. |
| `mission/model/frame_contract.py` | Shared planning-frame checks for odometry, goals, costmaps, and saved-map artifacts. |
| `mission/model/state.py` | Mission state enum, transition table, and transition rejection helpers. |
| `mission/model/status.py` | Mission status payload builder and typed status schema. |
| `mission/model/geometry.py` | Small geometry helpers shared by mission code. |

## services/map_layers/

Realtime map-layer Modules. This is L2 spatial state, not mission logic and
not the saved-map lifecycle service. Mission consumes these outputs through
wires such as `TraversabilityCostModule.fused_cost`.

| File | Role |
| --- | --- |
| `services/map_layers/occupancy_grid_module.py` | Builds 2D occupancy and exploration grids from point clouds. |
| `services/map_layers/voxel_grid_module.py` | Builds and queries a 3D voxel map from point clouds. |
| `services/map_layers/cpp_backend.py` | Thin adapter between Module payloads and `lingtu_nav_kernel` map-layer C++ types. |
| `services/map_layers/esdf_module.py` | Builds a 2D distance field from occupancy; uses `nav_kernel` when available. |
| `services/map_layers/elevation_map_module.py` | Builds elevation statistics from point clouds; uses `nav_kernel` when available. |
| `services/map_layers/traversability_cost_module.py` | Fuses occupancy, ESDF, elevation slope/step/roughness, and terrain risk into navigation cost. |
| `services/map_layers/map_artifact_builder.py` | Builds or reuses OctoMap artifacts and writes same-source metadata. |

## kernel/

ROS-free C++ algorithm kernels. Python Modules call these through
`lingtu_nav_kernel`, but the kernels do not import runtime, ROS, PCL, or gateway code.

| File | Role |
| --- | --- |
| `kernel/include/nav_kernel/map_layers_core.hpp` | Elevation grid, exact 2D ESDF, slope/step/roughness terrain risk, and fused traversability cost. |
| `kernel/include/nav_kernel/path_follower_core.hpp` | Path follower control math. |
| `kernel/include/nav_kernel/terrain_core.hpp` | Local terrain analysis from point clouds. |
| `kernel/bindings/bindings.cpp` | Nanobind bridge exposing the C++ kernels as `lingtu_nav_kernel`. |

## services/safety/

Safety services own reflex stop, plan safety, and velocity arbitration.

| File | Role |
| --- | --- |
| `services/safety/plan_safety.py` | Shared path/cost safety checks used by planner services. |
| `services/safety/safety_ring.py` | Runtime safety evaluator and stop publisher. |
| `services/safety/velocity_mux.py` | Final velocity arbiter before the driver. |

## safety/

| File | Role |
| --- | --- |
| `safety/plan_safety.py` | Shared path safety checks against grids/tomograms/backends. |
| `safety/velocity_mux.py` | Priority arbiter for teleop, visual servo, navigation recovery, and path follower velocity commands. |
| `safety/safety_ring.py` | Safety monitor and reflex stop producer. Checks odometry, velocity freshness, localization, path tracking, and exposes MCP skills. |

## exploration/

| File | Role |
| --- | --- |
| `exploration/frontier_explorer_module.py` | Wavefront frontier explorer that publishes exploration goals into `Navigation`. |
| `exploration/traversable_frontier_module.py` | Traversability-aware frontier preview/candidate ranking. Does not own robot motion by itself. |

## services/

| File | Role |
| --- | --- |
| `services/maps.py` | Saved-map lifecycle: save, list, use, delete, build tomogram/occupancy/octomap artifacts, expose map MCP skills. |
| `services/map/` | Internal helpers for `MapService`: storage, records, command routing, runtime snapshot bridge, and artifact build pipeline. |
| `services/goals.py` | Map-frame goal entry. Converts JSON/Gateway/MCP goal requests into `goal_pose`, `patrol_goals`, or `cancel`. |
| `services/geofence.py` | Polygon geofence monitor and hard-stop source. |
| `services/patrol.py` | Stores patrol routes and emits patrol waypoint sequences. |
| `services/scheduler.py` | Simple queued task scheduler for navigation-related commands. |

## core/

| Path | Role |
| --- | --- |
| `kernel/include/nav_kernel/path_follower_core.hpp` | C++ path follower kernel. |
| `kernel/include/nav_kernel/terrain_core.hpp` | Terrain map and traversability kernel. |
| `kernel/include/nav_kernel/pct_adapter_core.hpp` | PCT adapter support code. |
| `kernel/include/nav_kernel/simd_accel.hpp` | SIMD acceleration helpers. |
| `kernel/include/nav_kernel/types.hpp` | Shared C++ nav types. |
| `kernel/include/nav_kernel/validation.hpp` | C++ validation helpers. |
| `kernel/bindings/bindings.cpp` | Nanobind Python binding for the `lingtu_nav_kernel` extension. |
| `kernel/tests/*.cpp` | Mixed C++ kernel tests. These are not collected by pytest; build with CMake. |
| `services/plan/local_planner/cpp/tests/*.cpp` | Local-planner C++ tests. Build from the local planner CMake directory. |

## tests/

| Test file | Main coverage |
| --- | --- |
| `test_nav_modules.py` | Legacy broad tests for tracker, planner service, map layers, and safety. |
| `test_navigation_frame_contract.py` | Planning-frame, goal, retry, stop/cancel, and mission-state contracts. |
| `test_global_planner_diagnostics.py` | Planner backend diagnostics, fallback, OctoPlanner3D artifact gate behavior. |
| `test_nav_services.py` | Broad service/module tests, including ROS2-marked compatibility coverage. |
| `test_maps_service.py` | Map manager command and artifact behavior. |
| `test_map_artifact_builder.py` | OctoMap artifact builder and metadata behavior. |
| `test_planning_service_factory.py` | Planner service factory and mapless-vs-map-backed import boundary. |
| `test_lite_planner_backend.py` | DirectPath backend behavior. |
| `test_path_feasibility.py` | Ground path feasibility checks. |
| `test_cmd_vel_mux_contract.py` | Velocity mux priority/timeout contract. |
| `test_safety_ring_contract.py` | Safety ring public contract. |
| `test_geofence.py` | Geofence command and intrusion behavior. |
| `test_patrol.py` | Patrol route storage and dispatch behavior. |
| `test_scheduler.py` | Task scheduler command behavior. |
| `test_nav_corridor_scenario.py` | Corridor scenario and A* detour behavior. |
| `test_nav_semantic.py` | Shared message and semantic nav primitives. |

## Naming rules

- `*_module.py`: runtime Module with ports, lifecycle, registry, and often MCP skills.
- `*_service.py`: service boundary or coordinator used by a Module.
- `*_builder.py`: artifact construction helper.
- `*_contract.py`: frame or interface contract logic.
- `core/`: C++ hot path; do not reorganize with Python-only cleanup.
