# Simulation Folder And Modularization Goals

Date: 2026-05-31

## Goal

Close the LingTu server-side simulation validation loop without real robot
motion. The target evidence must cover multi-floor scenes, LiDAR/SLAM
localization, global planning, local planning, tracking, and exploration. Every
motion command claim must preserve `simulation_only=true`,
`real_robot_motion=false`, and `cmd_vel_sent_to_hardware=false`.

## Stop Conditions

- Do not run real robot services, publish to hardware topics, or start
  `lingtu.py nav/map/explore` against a physical robot.
- Accept a simulation gate only from fresh JSON evidence, not from stale
  artifacts or README claims.
- Treat Dimos benchmark closure and G4 full simulation closure as separate
  contracts until their required gate sets are explicitly aligned.

## Current Folder Contract

| Path | Role | Decision |
| --- | --- | --- |
| `sim/engine/` | Simulation runtime core, world registry, MuJoCo/Gazebo/CMU adapters. | Keep canonical. |
| `sim/worlds/` | Canonical XML/SDF world and scene definitions used by profiles and contracts. | Keep canonical. |
| `sim/assets/` | Canonical Thunder v3 robot assets, MJCF/URDF/meshes, Livox pattern assets. | Keep canonical. |
| `sim/robots/` | Compatibility robot asset paths and policy fallback paths for older scripts. | Keep as compatibility index. |
| `sim/scripts/` | Stable launcher, gate, validation, demo, and benchmark script contract. | Do not physically split; add indexes/tests instead. |
| `sim/validation/` | Full-system validation package imported by script wrappers and tests. | Keep canonical. |
| `sim/evaluation/` | Offline SLAM evaluation manifests and dataset tooling. | Keep separate from live simulation gates. |
| `sim/following/` | Person-following simulation and behavior metrics. | Keep separate; add gate only when claiming person tracking. |
| `sim/bridge/` | Legacy top-level bridge entrypoints. | Keep as legacy until references are removed. |
| `sim/launch/` | ROS 2/MuJoCo launch wrappers. | Keep; document legacy vs current runtime contract. |
| `sim/sensors/` | Sensor simulators/fallbacks not owned by runtime engine. | Boundary README added; keep contract locked. |
| `sim/datasets/` | Offline datasets and metadata. | Boundary README added; keep dataset/large-file policy locked. |
| `sim/output/` | Generated local outputs. | Prefer `artifacts/` for reproducible evidence. |
| `sim/external_scenes/` | External/license-constrained scene placeholders. | Keep. |
| `sim/meshes/` | Legacy mesh path. | Removed 2026-05-31 �?duplicates of sim/assets/meshes/. No code references. |
| `sim/maps/`, `sim/configs/` | Reserved placeholders. | Keep only if referenced; otherwise prune later. |
| `sim/semantic/` | Legacy semantic simulation residue. | Move to tests or mark experimental later. |

## Migration Status

Completed:

- Core scene migration: `sim/scenes/*.xml` moved into `sim/worlds/`.
- Core robot entry migration: `sim/robot/thunder.urdf` moved under
  `sim/robots/`.
- Current stable runtime entrypoints are `sim/worlds`, `sim/assets`,
  `sim/robots`, `sim/scripts`, and `lingtu.py --endpoint ...`.

Intentional non-migration:

- `sim/scripts/<name>` remains a stable path contract. The earlier idea to move
  scripts into physical `gates/`, `validation/`, and `demos/` folders is not the
  current plan because profiles, runtime contracts, tests, and CI refer to these
  paths directly.

Known cleanup targets:

- `sim/README.md` stale `scenes/` and `robot/` directory references were
  corrected in the same pass that created this goal file.
- `sim/bridge/nova_nav_bridge.py` has been moved off the deleted `sim/robot`
  defaults and now points at `sim/robots/nova_dog/robot_with_camera.xml` plus
  `sim/robots/nova_dog/policy.onnx`.
- `sim/planning/` contains legacy planning simulation tests/assets that are still
  referenced by simulation contracts and profile graph tests.

## Modularization Targets

P0: safety and command-chain contracts

- MCP `stop_cmd` must reach the driver stop input and Navigation stop input.
- MCP `cmd_vel` must never bypass `CmdVelMux`; it is treated as an external
  manual control source and enters `CmdVelMux.teleop_cmd_vel`.
- MCP `set_mode("estop")` must publish both `stop_cmd` and a zero `cmd_vel` so
  estop mode follows the same safety/mux path as the explicit stop tool.
- Gateway and MCP backend route tables must stay in parity until moved to a
  shared metadata source.

P1: plugin and backend contracts

- Planner backend switching is currently wrapped by `NavigationModule`; decide
  whether global planning becomes a first-class Module or remains an internal
  service with an explicit no-hot-switch contract.
- `local_planner`, `path_follower`, and `terrain` expose status but do not yet
  implement runtime backend switching.
- Registry-backed plugins are still constrained by module-local static
  allowlists in several places.
- Perception has three plugin surfaces: `PerceptionModule`, legacy
  `DetectorModule`/`EncoderModule`, and `PerceptionFactory`; one canonical
  surface should be selected.

Status: `PerceptionModule` constructor now validates detector/encoder backends
against the live registry catalog, so newly registered providers can be used as
startup configuration as well as runtime switch targets.

Motion backends remain fail-closed for runtime reconfiguration:
`NavigationModule` planner, `LocalPlannerModule`, `PathFollowerModule`,
`TerrainModule`, and `SlamBridgeModule` all inherit the default unsupported
response unless a future change adds a specifically tested safe switch path.

P2: observability and evidence

- Diagnostics should preserve plugin fields such as `fallback_backend`,
  `reconfigurable`, `capabilities`, and readiness, not only backend/degraded
  fields.
- Stack factories should report missing required modules as explicit health
  issues instead of silently building partial graphs after `ImportError`.
- ESDF wiring into `LocalPlannerModule` is currently reserved/cached; do not
  claim ESDF affects local planning until a behavior test proves it.
- Closure reports should present missing required gates in canonical execution
  order, not alphabetic order, so the next-action queue matches
  `required_gate_sequence`.

## Current Algorithm Chain

PCT and A* are global planner backends. Current `nav`, `explore`, and
`tare_explore` style profiles keep `enable_native=False`, so local planning and
tracking use the in-process CMU-style chain:
`LocalPlannerModule(backend="nanobind") -> PathFollowerModule(backend="nav_kernel")
-> CmdVelMux -> driver`.

The external CMU ROS 2 `localPlanner/pathFollower` chain is still available for
dedicated native simulation gates such as `native_pct_mujoco`; it is not the
default Python Module runtime path.

Algorithm handoff contract:

1. SLAM or driver odometry fans into Navigation, maps, local planner, path
   follower, safety, Gateway, and exploration consumers.
2. Map cloud fans into occupancy, voxel, elevation, terrain, Gateway, and
   visualization consumers.
3. `NavigationModule` owns global planning through `GlobalPlannerService`
   (`pct` or `astar`) and publishes `global_path` plus waypoints.
4. `LocalPlannerModule` consumes `global_path`, odometry, costmap, terrain, and
   reserved ESDF input, then publishes `local_path`.
5. `PathFollowerModule` tracks `local_path` into `cmd_vel`.
6. `CmdVelMux` arbitrates teleop, visual servo, Navigation recovery, and
   path-follower commands before the selected driver sees velocity commands.
7. `SafetyRingModule` monitors mux output and publishes stop commands to the
   driver and Navigation.

Known weak points:

- PCT native replanning does not currently consume live fused costmap as a full
  dynamic replanning source; dynamic response is mainly local-planner and
  safety-layer evidence.
- ESDF is wired but not yet behavior-proven in local scoring.
- TARE publishes exploration goals/path strategy into Navigation; it does not
  own direct driver command output.
- Optional planning/observability wires can still degrade quietly when a module
  or port is absent; only L0 stop wiring is currently fail-closed.

## Simulation Evidence Matrix

| Area | Required gates/tests |
| --- | --- |
| Multi-floor | `multifloor_exploration`; route matrix with PCT and A*. |
| LiDAR/SLAM localization | `fastlio2_dynamic_inspection`, `large_loop_closure`. |
| Global planning | `large_terrain`, `native_pct_mujoco`, `pct_saved_map_navigation`. |
| Local planning | `dynamic_obstacle_local_planner`, `moving_obstacle_sweep`, `native_pct_mujoco`. |
| Nav tracking | `multifloor_exploration`, `native_pct_mujoco`. |
| Person/target tracking | Add a dedicated person-tracking simulation gate before claiming this. |
| Exploration | `gazebo_runtime`, `multifloor_exploration`; add `mujoco_tare_exploration` only when claiming TARE. |
| Saved-map relocalization | `saved_map_relocalize`, `pct_saved_map_navigation`; optional `bbs3d_kidnapped_relocalize`. |

## Current G4 Closure Snapshot

Latest local aggregator run:

```bash
PYTHONPATH=src:. python sim/scripts/server_sim_closure.py \
  --preset g4_server_full_sim \
  --required-only \
  --json-out artifacts/server_sim_closure_summary_g4_current.json
```

Result on 2026-05-31:

- `ok=false`
- `simulation_only=true`
- `real_robot_motion=false`
- `cmd_vel_sent_to_hardware=false`
- Fresh verified required gates: `gateway_runtime_acceptance`,
  `routecheck_preflight`, and `dynamic_obstacle_local_planner`.
- Required gates still blocking: `multifloor_exploration`, `large_terrain`,
  `native_pct_mujoco`,
  `fastlio2_dynamic_inspection`, `moving_obstacle_sweep`,
  `large_loop_closure`, `gazebo_runtime`, `saved_map_relocalize`, and
  `pct_saved_map_navigation`.

Fresh gate detail:

| Gate | Current status | Evidence / blocker |
| --- | --- | --- |
| `routecheck_preflight` | Pass | `artifacts/server_sim_closure/routecheck/summary.json`; non-motion routecheck kept published `goal_pose`, `cmd_vel`, and `stop_cmd` counts at zero. |
| `dynamic_obstacle_local_planner` | Pass | `artifacts/server_sim_closure/dynamic_obstacle_local_planner/report.json`; nanobind backend verified dynamic replan, obstacle response, clear-path recovery, and non-hardware command fields. |
| `gateway_runtime_acceptance` | Pass | `artifacts/server_sim_closure/gateway_runtime_acceptance/report.json`; when the default local HTTP Gateway is absent, non-motion acceptance now falls back to an in-process `GatewayModule` stub with `snapshot_source=in_process_stub`, `simulation_only=true`, `real_robot_motion=false`, `cmd_vel_sent_to_hardware=false`, complete product links, ModulePort-primary dataflow, command whitelist, and 8 runtime stages. This is data-plane contract evidence only, not live motion or field readiness. |
| `multifloor_exploration` | Fail | `artifacts/server_sim_closure/multifloor_exploration/report.json`; upper/cross-floor far safe-goal projections are no longer counted as feasible requested-goal plans. The gate now reports `environment_runtime` plus `planning_tracking`, and the highest-priority runtime blocker is `PCT native runtime unavailable` on this Windows/Python 3.13 host. |
| `large_terrain` | Fail | `artifacts/server_sim_closure/large_terrain/report.json`; assets and non-motion flags refresh successfully, but PCT native runtime is unavailable because the checked-in native modules are Linux ELF / Python 3.10-oriented while this host is Windows / Python 3.13. |

Stop condition: do not claim G4 full simulation health until every required gate
above is fresh and passing in `artifacts/server_sim_closure_summary_g4_current.json`.

## Next Execution Goals

G1. Done in this pass: lock MCP command-chain wiring with profile graph tests.

G2. Done in the continuation pass: create `g4_server_full_sim` as the
server-side full simulation preset. It is Dimos plus
`multifloor_exploration`, and README, algorithm gate constants, closure
script preset selection, default freshness checks, and pytest expectations are
aligned.

G3. Done in the current continuation: add the folder boundary goal file, fix
the live `sim/README.md` directory table, and add per-folder boundary indexes
for `sim/sensors/`, `sim/datasets/`, and legacy `sim/bridge/` without moving
stable `sim/scripts` paths.

G4. Partially done: add backend allowlist/catalog parity tests and make
`terrain/cmu` a registered NativeModule alias instead of an invisible
allowlist-only backend. Existing runtime backend switch tests still lock the
fail-closed/no-hot-switch behavior.

G5. Done in the continuation pass: diagnostics active backend status now
preserves extended metadata such as `fallback_backend`, `reconfigurable`,
`capabilities`, and nested readiness fields.

G6. Active now: turn the G4 closure snapshot into a repeatable execution queue.
Local non-motion gates are refreshed first; ROS2/MuJoCo/Linux-native gates stay
blocked until a host with ROS 2 Humble, native PCT libraries, and MuJoCo EGL can
generate fresh artifacts without connecting to robot hardware.

G7. Done in the current continuation: order `missing_or_failed`,
`remaining_gaps`, optional gaps, and generated missing-gate commands by
canonical gate order instead of alphabetic order.

G8. Done in the current continuation: add `runtime.plugin_seed` as the central
built-in plugin registration seed for safe core surfaces. Only pure registration
triggers are wired through it now (`driver` stack, `GlobalPlannerService`, and
CLI backend validation). Gateway/WebRTC/visualization remain explicit opt-in
seed groups because they include optional runtime surfaces or duplicate
registration names.

G9. Done in the current continuation: make `gateway-runtime-acceptance
--acceptance-mode non_motion` repeatable without a live HTTP Gateway by using a
strict in-process stub fallback only when all required default-local Gateway
fetches fail. The fallback does not call `GatewayModule.start()`, disables the
blackbox recorder through a temporary env override, and asserts command output
ports remain unpublished.

G10. Done in the xhigh continuation: organize the remaining simulation folder
contracts without moving stable entrypoints. `sim/engine/README.md` now defines
the runtime-core boundary, `artifacts/server_sim_closure/README.md` defines the
generated evidence boundary, `sim/scripts/README.md` indexes the stable
validation helpers, and `sim/README.md` points full-closure users at the
canonical G4 summary artifact.

G11. Done in the xhigh continuation: move another set of stack factories toward
module-first plugin resolution. `safety()`, `lidar()`, and `gateway()` now
resolve their built-in modules through the registry seed path with explicit
fallbacks, and tests lock fake registry substitution for those stack factories.
This does not make motion backends hot-swappable; runtime switching remains
fail-closed unless a specifically tested safe path exists.

G12. Next parallel target: split the remaining work into three lanes.
Lane A fixes or documents the PCT native runtime contract for Windows/Python
3.13 versus Linux/Python 3.10 evidence hosts. Lane B turns the remaining
artifact-contract gates (`fastlio2_dynamic_inspection`,
`moving_obstacle_sweep`, `large_loop_closure`, `gazebo_runtime`,
`saved_map_relocalize`, `pct_saved_map_navigation`, `native_pct_mujoco`) into
fresh runnable/non-runnable evidence with exact host requirements. Lane C
continues module-first pluginization for planner, local planner, path follower,
terrain, SLAM, perception, gateway tools, and exploration without introducing
new dependencies.

G13. Done in the current xhigh continuation: `server_sim_closure` now publishes
structured `host_requirements` for each gate, and propagates them into
`missing_required_commands`, `next_actions`, and the per-gate summary entry.
The current G4 summary artifact therefore distinguishes local non-motion Python
gates from Linux/ROS 2/MuJoCo/PCT-native gates before a worker tries to execute
the queue.

## Current Verification Evidence

Fresh checks from this pass:

```bash
python -m pytest src/runtime/tests/test_plugin_seed.py src/runtime/tests/test_stack_registry_resolution.py src/runtime/tests/test_sim_runtime_compat.py::test_sim_boundary_indexes_document_stable_contracts src/runtime/tests/test_server_sim_closure.py::test_server_sim_closure_multifloor_surfaces_pct_runtime_blocker src/runtime/tests/test_multifloor_sim_validation.py::test_partial_global_plan_is_kept_for_diagnostics_but_not_feasible src/runtime/tests/test_multifloor_sim_validation.py::test_far_projected_safe_goal_is_not_counted_as_requested_goal -q
# 10 passed

python -m py_compile src/runtime/blueprints/stacks/_registry.py src/runtime/blueprints/stacks/safety.py src/runtime/blueprints/stacks/lidar.py src/runtime/blueprints/stacks/gateway.py src/drivers/real/lidar/lidar_module.py src/nav/services/safety/cmd_vel_mux_module.py src/nav/services/geofence.py sim/scripts/multifloor_nav_validation.py sim/scripts/server_sim_closure.py
# passed

PYTHONPATH=src:. python sim/scripts/large_terrain_nav_validation.py --output-dir artifacts/server_sim_closure/large_terrain --planners pct,astar --json-out artifacts/server_sim_closure/large_terrain/report.json
# exited 0; report ok=false, simulation_only=true, real_robot_motion=false,
# cmd_vel_sent_to_hardware=false; environment_blockers=["PCT native runtime unavailable"]

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --json-out artifacts/server_sim_closure_summary_g4_current.json
# exited 0; summary ok=false, simulation_only=true, real_robot_motion=false,
# cmd_vel_sent_to_hardware=false; verified true: gateway_runtime_acceptance,
# routecheck_preflight, dynamic_obstacle_local_planner; multifloor and
# large_terrain now surface environment_runtime/planning_tracking blockers;
# next_actions and missing_required_commands include host_requirements.

python -m pytest src/runtime/tests/test_server_sim_closure.py::test_server_sim_closure_summary_lists_missing_required_commands src/runtime/tests/test_server_sim_closure.py::test_server_sim_closure_next_actions_separate_runtime_blocker_from_missing_report -q
# 2 passed

python -m pytest src/runtime/tests/test_server_sim_closure.py -q
# 121 passed

python -m pytest src/runtime/tests/test_plugin_seed.py src/runtime/tests/test_backend_status.py src/runtime/tests/test_registry.py -q
# 28 passed

python -m pytest src/runtime/tests/test_sim_runtime_compat.py::test_sim_boundary_indexes_document_stable_contracts src/runtime/tests/test_sim_runtime_compat.py::test_legacy_nova_nav_bridge_uses_current_robot_paths src/runtime/tests/test_sim_runtime_compat.py::test_sim_mujoco_full_stack_routes_autonomy_cmds_through_mux src/runtime/tests/test_sim_runtime_compat.py::test_full_stack_mux_wiring_tolerates_legacy_nav_without_recovery_cmd -q
# 4 passed

python -m pytest src/runtime/tests/test_server_sim_closure.py -q -k "g4_summary_required_gate_sequence_preserves_core_order or g4_missing_required_gates_preserve_core_order or g4_required_gates_are_default_freshness_required or readme_current_full_closure_gates_match_g4_server_full_sim_preset or readme_full_closure_command_uses_g4_server_full_sim_preset or server_sim_closure_g4_server_full_sim_preset_selects_closure_gate_set"
# 6 passed, 114 deselected

python -m py_compile src/runtime/plugin_seed.py cli/main.py src/runtime/blueprints/stacks/driver.py src/nav/services/plan/global_planner/service.py sim/scripts/server_sim_closure.py
# passed

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --json-out artifacts/server_sim_closure_summary_g4_current.json
# exited 0; summary ok=false, simulation_only=true, real_robot_motion=false,
# cmd_vel_sent_to_hardware=false; verified true: gateway_runtime_acceptance,
# routecheck_preflight, dynamic_obstacle_local_planner;
# blocking required gates remain: multifloor_exploration, large_terrain,
# native_pct_mujoco, fastlio2_dynamic_inspection, moving_obstacle_sweep,
# large_loop_closure, gazebo_runtime, saved_map_relocalize,
# pct_saved_map_navigation

python -m pytest src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_passes_non_motion_without_ros2_topic src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_non_motion_exposes_top_level_sim_safety_flags src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_non_motion_uses_local_stub_when_gateway_is_down src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_field_does_not_use_local_stub_when_gateway_is_down src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_in_process_stub_stays_non_motion src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_simulation_passes_without_real_runtime_evidence src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_field_passes_with_live_samples -q
# 7 passed

python -m py_compile src/runtime/gateway_runtime_acceptance.py src/gateway/tests/test_gateway_runtime_acceptance.py
# passed

PYTHONPATH=src:. python lingtu.py gateway-runtime-acceptance --acceptance-mode non_motion --json-out artifacts/server_sim_closure/gateway_runtime_acceptance/report.json
# exited 0; report ok=true, snapshot_source=in_process_stub,
# runtime_contract=in_process_stub, simulation_only=true,
# real_robot_motion=false, cmd_vel_sent_to_hardware=false, blockers=[]

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --required gateway_runtime_acceptance --required-only --gateway-runtime-acceptance-report artifacts/server_sim_closure/gateway_runtime_acceptance/report.json --json-out artifacts/server_sim_closure_gateway_runtime_acceptance_check.json --strict
# exited 0; gateway_runtime_acceptance passed as a single required gate

python -m pytest src/runtime/tests/test_server_sim_closure.py -q
# 119 passed

python -m pytest src/runtime/tests/test_backend_status.py -q
# 13 passed

python -m pytest src/runtime/tests/test_runtime_backend_switch.py -q
# 7 passed

python -m pytest src/gateway/tests/test_gateway_runtime_status.py -q
# 78 passed

python -m pytest src/runtime/tests/test_profile_graph_snapshots.py -q
# 33 passed

python -m py_compile src/runtime/algorithm_gates.py sim/scripts/server_sim_closure.py src/nav/local/terrain_module.py src/gateway/routes/diagnostics.py
# passed

python -m pytest src/runtime/tests/test_profile_graph_snapshots.py::test_profile_graph_snapshot_locks_safety_gateway_and_mux_edges -q
# 1 passed

python -m pytest src/runtime/tests/test_sim_runtime_compat.py::test_sim_mujoco_full_stack_routes_autonomy_cmds_through_mux src/runtime/tests/test_sim_runtime_compat.py::test_full_stack_mux_wiring_tolerates_legacy_nav_without_recovery_cmd -q
# 2 passed

python -m pytest src/gateway/tests/test_gateway_runtime_status.py::test_gateway_and_mcp_backend_route_tables_stay_in_parity src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_tool_uses_gateway_guard src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_tool_guards_motion_without_gateway_module src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_reads_nested_navigation_state_without_gateway_module -q
# 4 passed

python -m pytest src/runtime/tests/test_sim_runtime_compat.py::test_legacy_nova_nav_bridge_uses_current_robot_paths -q
# 1 passed

python -m py_compile sim/bridge/nova_nav_bridge.py
# passed

python -m pytest src/runtime/tests/test_new_modules.py::TestMCPServerModule::test_set_mode_via_skill src/runtime/tests/test_new_modules.py::TestMCPServerModule::test_set_mode_estop_publishes_stop_and_zero_twist src/runtime/tests/test_new_modules.py::TestMCPServerModule::test_set_mode_invalid -q
# 3 passed

python -m pytest src/gateway/tests/test_gateway_runtime_status.py::test_gateway_and_mcp_backend_route_tables_stay_in_parity src/gateway/tests/test_mcp_auth.py -q
# 7 passed

python -m py_compile src/gateway/mcp_server.py
# passed

python -m pytest src/runtime/tests/test_server_sim_closure.py -q -k "dimos_required_gates_come_from_core_algorithm_gate_constant or g4_server_full_sim_required_gates_come_from_core_algorithm_gate_constant or readme_current_full_closure_gates_match_g4_server_full_sim_preset or readme_full_closure_command_uses_g4_server_full_sim_preset or g4_summary_required_gate_sequence_preserves_core_order or g4_required_gates_are_default_freshness_required or dimos_summary_required_gate_sequence_preserves_core_order or server_sim_closure_g4_server_full_sim_preset_selects_closure_gate_set"
# 8 passed, 111 deselected

python -m pytest src/runtime/tests/test_backend_status.py::test_autonomy_backend_registry_names_are_visible src/runtime/tests/test_backend_status.py::test_autonomy_backend_allowlists_match_registry_catalog src/runtime/tests/test_backend_status.py::test_terrain_cmu_backend_uses_native_setup -q
# 3 passed

python -m pytest src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route_exposes_active_backend_status -q
# 1 passed

python -m py_compile src/nav/local/terrain_module.py src/gateway/routes/diagnostics.py
# passed

python -m pytest src/runtime/tests/test_perception_module.py::TestDetectorConfiguration::test_constructor_accepts_registered_detector_and_encoder_plugins src/runtime/tests/test_perception_module.py::TestDetectorConfiguration::test_unknown_perception_backend_fails_fast src/runtime/tests/test_perception_module.py::TestDetectorConfiguration::test_perception_backend_registry_names_are_visible -q
# 3 passed

python -m pytest src/runtime/tests/test_runtime_backend_switch.py::test_perception_reconfigure_detector_rejects_unknown_backend src/runtime/tests/test_runtime_backend_switch.py::test_perception_reconfigure_detector_updates_health_status src/runtime/tests/test_runtime_backend_switch.py::test_perception_reconfigure_encoder_updates_health_status -q
# 3 passed

python -m py_compile src/perception/semantic_perception/perception_module.py
# passed

python -m pytest src/runtime/tests/test_runtime_backend_switch.py::test_motion_modules_reconfigure_backend_fails_closed -q
# 5 passed

python -m pytest src/runtime/tests/test_sim_runtime_compat.py::test_sim_boundary_indexes_document_stable_contracts -q
# 1 passed

python -m pytest src/runtime/tests/test_server_sim_closure.py::test_g4_missing_required_gates_preserve_core_order -q
# 1 passed

python -m pytest src/runtime/tests/test_runtime_backend_switch.py -q
# 12 passed

python -m pytest src/runtime/tests/test_backend_status.py -q
# 13 passed

python -m pytest src/runtime/tests/test_perception_module.py::TestDetectorConfiguration -q
# 10 passed

python -m pytest src/runtime/tests/test_sim_runtime_compat.py::test_legacy_nova_nav_bridge_uses_current_robot_paths src/runtime/tests/test_sim_runtime_compat.py::test_sim_boundary_indexes_document_stable_contracts src/runtime/tests/test_sim_runtime_compat.py::test_sim_mujoco_full_stack_routes_autonomy_cmds_through_mux src/runtime/tests/test_sim_runtime_compat.py::test_full_stack_mux_wiring_tolerates_legacy_nav_without_recovery_cmd -q
# 4 passed

python -m pytest src/gateway/tests/test_gateway_runtime_status.py::test_gateway_and_mcp_backend_route_tables_stay_in_parity src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_tool_uses_gateway_guard src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_tool_guards_motion_without_gateway_module src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_reads_nested_navigation_state_without_gateway_module src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route_exposes_active_backend_status -q
# 5 passed

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --json-out NUL
# exited 0; summary ok=false, simulation_only=true, real_robot_motion=false
# verified true: gateway_runtime_acceptance, routecheck_preflight,
# dynamic_obstacle_local_planner
# still failed/missing/stale: multifloor_exploration, large_terrain,
# native_pct_mujoco, fastlio2_dynamic_inspection, moving_obstacle_sweep,
# large_loop_closure, gazebo_runtime, saved_map_relocalize,
# pct_saved_map_navigation
```

Stale path search still has expected historical references in plan/archive
documents. The live `sim/README.md` and `sim/bridge/nova_nav_bridge.py` checks
no longer match `sim/robot` or `sim/scenes` runtime paths.

## 2026-05-31 Continuation: G4 Contract And Stack Pluginization

G14. Done in this continuation pass: make the G4 evidence contract explicit
enough for agents and CI to separate missing/stale artifacts from host-runtime
blockers. `server_sim_closure.py` now exposes `expected_report_path`,
`accepted_patterns`, and `host_requirements` on each gate summary, on
`missing_required_commands`, and on `next_actions`; it also adds the missing
`--multifloor-exploration-report` CLI override. `native_pct_mujoco` accepts
the historical `report.*.server.json` pattern, while `saved_map_relocalize`
is documented as ROS2 + MuJoCo/Fast-LIO + localizer + saved-map assets, not a
PCT-native gate.

G15. Done in this continuation pass: extend stack-level pluginization without
changing Module wiring names. `safety`, `lidar`, `gateway`, `navigation`,
external `tare` exploration, and `perception` now resolve registered
implementations while preserving canonical aliases such as `NavigationModule`,
`SafetyRingModule`, `GatewayModule`, `PerceptionModule`, and
`WavefrontFrontierExplorer`. This keeps existing `full_stack_wiring.py`
string contracts stable when a local backend is swapped instantly.

Latest safe validation evidence:

```bash
python -m py_compile sim/scripts/server_sim_closure.py sim/scripts/multifloor_nav_validation.py src/runtime/blueprints/stacks/safety.py src/runtime/blueprints/stacks/lidar.py src/runtime/blueprints/stacks/gateway.py src/runtime/blueprints/stacks/navigation.py src/runtime/blueprints/stacks/exploration.py src/runtime/blueprints/stacks/perception.py src/perception/semantic_perception/perception_module.py src/runtime/tests/test_stack_registry_resolution.py src/runtime/tests/test_plugin_seed.py src/runtime/tests/test_server_sim_closure.py src/runtime/tests/test_sim_runtime_compat.py
# passed

python -m pytest src/runtime/tests/test_stack_registry_resolution.py src/runtime/tests/test_plugin_seed.py -q
# 9 passed

python -m pytest src/runtime/tests/test_server_sim_closure.py -q
# 124 passed

python -m pytest src/runtime/tests/test_multifloor_sim_validation.py -q
# 26 passed

python -m pytest src/runtime/tests/test_tare_exploration.py src/runtime/tests/test_traversable_frontier_module.py::test_full_stack_can_add_traversable_frontier_without_wiring_it_to_control src/runtime/tests/test_sim_runtime_compat.py::test_full_stack_wires_frontier_exploration_goal_to_navigation -q
# 42 passed

python -m pytest src/runtime/tests/test_sim_semantic_pipeline_blueprint.py src/runtime/tests/test_perception_factory_registry.py src/runtime/tests/test_perception_module.py::TestDetectorConfiguration -q
# 18 passed

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --json-out artifacts/server_sim_closure_summary_g4_current.json
# exited 0; summary ok=false, simulation_only=true,
# real_robot_motion=false, cmd_vel_sent_to_hardware=false.
# verified true: gateway_runtime_acceptance, routecheck_preflight,
# dynamic_obstacle_local_planner.
# still blocked: multifloor_exploration, large_terrain,
# native_pct_mujoco, fastlio2_dynamic_inspection, moving_obstacle_sweep,
# large_loop_closure, gazebo_runtime, saved_map_relocalize,
# pct_saved_map_navigation.
```

## 2026-05-31 Continuation: Map/Planner/Memory Pluginization

G16. Done in this continuation pass: extend stack-level registry replacement
to `maps`, `planner`, and `memory` without changing canonical Module names.
The map stack now resolves `OccupancyGridModule`, `VoxelGridModule`,
`ESDFModule`, `ElevationMapModule`, `TraversabilityCostModule`,
`ROS2GridBridgeModule`, and `MapManagerModule` through the registry. The
planner stack resolves `SemanticPlannerModule`, `LLMModule`, and
`VisualServoModule`. The memory stack resolves `SemanticMapperModule`,
`EpisodicMemoryModule`, `TaggedLocationsModule`, `VectorMemoryModule`,
`TemporalMemoryModule`, and `MissionLoggerModule`. `MapManagerModule` and
`ROS2GridBridgeModule` now have explicit built-in registry entries. This moves
local map/planner/memory swaps onto the same plugin surface as the earlier
safety/navigation/perception work while keeping full-stack wire names stable.

Latest safe validation evidence for G16:

```bash
python -m pytest src/runtime/tests/test_stack_registry_resolution.py::test_maps_stack_prefers_registered_modules_with_canonical_aliases src/runtime/tests/test_stack_registry_resolution.py::test_planner_stack_prefers_registered_modules_with_canonical_aliases src/runtime/tests/test_stack_registry_resolution.py::test_memory_stack_prefers_registered_modules_with_canonical_aliases -q
# RED before implementation: 3 failed because maps/planner/memory ignored registry replacements.

python -m py_compile src/runtime/blueprints/stacks/maps.py src/runtime/blueprints/stacks/planner.py src/runtime/blueprints/stacks/memory.py src/nav/services/maps.py src/compat/ros2/nav/grid_bridge.py src/runtime/plugin_seed.py src/runtime/tests/test_stack_registry_resolution.py src/runtime/tests/test_plugin_seed.py
# passed

python -m pytest src/runtime/tests/test_stack_registry_resolution.py src/runtime/tests/test_plugin_seed.py -q
# 12 passed

python -m pytest src/runtime/tests/test_profile_graph_snapshots.py -q
# 33 passed

python -m pytest src/runtime/tests/test_sim_semantic_pipeline_blueprint.py -q
# 4 passed

python -m pytest src/runtime/tests/test_cross_module_integration.py -q
# 1 passed; emitted existing nanobind refleak diagnostics after pytest success.

python -m pytest src/runtime/tests/test_sim_nav_e2e.py -q
# 4 passed

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --json-out artifacts/server_sim_closure_summary_g4_current.json
# exited 0; summary remains ok=false. Verified true:
# gateway_runtime_acceptance, routecheck_preflight,
# dynamic_obstacle_local_planner. The same 9 G4 gates remain blocked by
# PCT-native/ROS2/MuJoCo/localizer host requirements or missing/stale reports.
```

## 2026-05-31 Continuation: SLAM And Sim-LiDAR Pluginization

G17. Done in this continuation pass: extend stack-level registry replacement
to `slam` and `sim_lidar` while preserving non-motion validation boundaries.
`slam("bridge", manage_services=False)` now resolves `SlamBridgeModule` and
`DepthVisualOdomModule` through registry categories `slam_bridge/default` and
`visual_odom/depth`, retaining the canonical aliases consumed by full-stack
wiring. `sim_lidar()` now resolves `SimPointCloudProvider` through
`sim_lidar/pointcloud`, so synthetic LiDAR sources can be swapped without
rewiring downstream map/planning modules.

Latest safe validation evidence for G17:

```bash
python -m pytest src/runtime/tests/test_stack_registry_resolution.py::test_slam_stack_prefers_registered_bridge_and_visual_odom_modules src/runtime/tests/test_stack_registry_resolution.py::test_sim_lidar_stack_prefers_registered_pointcloud_provider -q
# RED before implementation: 2 failed because slam/sim_lidar ignored registry replacements.

python -m py_compile src/runtime/blueprints/stacks/slam.py src/runtime/blueprints/stacks/sim_lidar.py src/localization/depth_visual_odom_module.py src/drivers/sim/pointcloud.py src/runtime/plugin_seed.py src/runtime/tests/test_stack_registry_resolution.py src/runtime/tests/test_plugin_seed.py
# passed

python -m pytest src/runtime/tests/test_stack_registry_resolution.py src/runtime/tests/test_plugin_seed.py -q
# 14 passed

python -m pytest src/runtime/tests/test_profile_graph_snapshots.py -q
# 33 passed

python -m pytest src/runtime/tests/test_sim_nav_e2e.py -q
# 4 passed

python -m pytest src/runtime/tests/test_localization_health.py src/localization/tests/test_slam_backend_status.py src/localization/tests/test_slam_gnss_fusion.py -q
# 121 passed

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --json-out artifacts/server_sim_closure_summary_g4_current.json
# exited 0; summary remains ok=false, simulation_only=true,
# real_robot_motion=false, cmd_vel_sent_to_hardware=false.
```

## 2026-05-31 Continuation: Optional Surfaces And Seed Preservation

G18. Done in this continuation pass: close the remaining stack-factory
pluginization gaps found by the parallel architecture/explore pass.
`perception()` now resolves optional encoder, reconstruction, dataset recorder,
and keyframe exporter modules through registry categories while keeping the
stable aliases consumed by full-stack wiring. `navigation()` now resolves the
optional ROS 2 path bridge through `navigation/ros2_path_bridge`. The local
TARE exploration branch now resolves `TAREExplorerModule` and
`ExplorationSupervisorModule` through the same registry-first path as the
external TARE branch.

G19. Done in this continuation pass: harden the simulation folder boundary
contract for future agents. `sim/scripts/README.md` now classifies scripts by
safety level instead of implying that every script is runnable in the current
host. `sim/launch/README.md` documents the legacy ROS launch contract and
requires an isolated ROS domain with no hardware command subscribers. The top
level `sim/README.md` now states that product tasks must use a simulation
endpoint and that bare `nav`, `map`, and `explore` profiles are not simulation
entrypoints.

G20. Done in this continuation pass: make the autonomy helper assembly
registry-first. `add_autonomy_stack()` now resolves terrain, local planner, and
path follower modules through the plugin registry before adding them to a
blueprint. This preserves canonical aliases (`TerrainModule`,
`LocalPlannerModule`, `PathFollowerModule`) while allowing a local detector,
local planner, follower, or terrain plugin to be swapped at startup without
rewiring downstream modules.

G21. Done in this continuation pass: fix the highest-risk plugin seed bug from
the parallel review. `seed_builtin_plugins()` now snapshots pre-existing
registry entries and restores those exact entries after built-in seed imports,
so a caller-provided plugin override is not silently overwritten while missing
built-in siblings are still filled in.

Latest safe validation evidence for G18-G21:

```bash
python -m pytest src/runtime/tests/test_stack_registry_resolution.py::test_perception_optional_tool_modules_prefer_registered_modules src/runtime/tests/test_stack_registry_resolution.py::test_navigation_optional_ros2_path_bridge_prefers_registered_module src/runtime/tests/test_plugin_seed.py::test_reconstruction_plugin_seed_registers_optional_modules src/runtime/tests/test_plugin_seed.py::test_navigation_plugin_seed_registers_ros2_path_bridge -q
# RED before implementation: optional perception/reconstruction and ROS2 path
# bridge surfaces ignored registry replacements or were missing seed entries.

python -m pytest src/runtime/tests/test_stack_registry_resolution.py::test_exploration_local_tare_prefers_registered_modules_with_canonical_aliases -q
# RED before implementation: local TARE used direct imports and ignored registry
# replacements.

python -m pytest src/runtime/tests/test_sim_runtime_compat.py::test_sim_boundary_indexes_document_stable_contracts src/runtime/tests/test_server_sim_closure.py::test_server_sim_closure_summary_only_mode_is_explicit -q
# RED before implementation: simulation boundary docs lacked launch/safety
# contracts and summary-only closure output did not expose explicit execution
# fields.

python -m pytest src/runtime/tests/test_stack_registry_resolution.py::test_autonomy_stack_prefers_registered_modules_with_canonical_aliases -q
# RED before implementation: add_autonomy_stack used eager concrete class
# imports and ignored registry replacements.

python -m pytest src/runtime/tests/test_plugin_seed.py::test_builtin_plugin_seed_preserves_preexisting_plugin_registrations -q
# RED before implementation: built-in plugin seeding overwrote a pre-existing
# plugin registration for the same category/name.

python -m py_compile src/runtime/registry.py src/runtime/plugin_seed.py src/runtime/blueprints/stacks/exploration.py src/runtime/blueprints/stacks/perception.py src/runtime/blueprints/stacks/navigation.py src/nav/local/__init__.py src/nav/local/autonomy_module.py src/runtime/tests/test_plugin_seed.py src/runtime/tests/test_stack_registry_resolution.py src/runtime/tests/test_server_sim_closure.py src/runtime/tests/test_sim_runtime_compat.py
# passed

python -m pytest src/runtime/tests/test_stack_registry_resolution.py src/runtime/tests/test_plugin_seed.py -q
# 19 passed

python -m pytest src/runtime/tests/test_server_sim_closure.py -q
# 124 passed

python -m pytest src/runtime/tests/test_sim_runtime_compat.py::test_sim_boundary_indexes_document_stable_contracts src/runtime/tests/test_tare_exploration.py::TestExplorationStackFactory -q
# 13 passed

python -m pytest src/runtime/tests/test_profile_graph_snapshots.py -q
# 33 passed

python -m pytest src/runtime/tests/test_sim_nav_e2e.py -q
# 4 passed

python -m pytest src/runtime/tests/test_cross_module_integration.py -q
# 1 passed; emitted existing nanobind refleak diagnostics and deprecation
# warnings after pytest success.

python -m pytest src/nav/tests/local/test_autonomy_modules.py -q
# 18 passed

python -m pytest src/runtime/tests/test_backend_status.py src/runtime/tests/test_runtime_backend_switch.py -q
# 25 passed

python -m pytest src/runtime/tests/test_sim_semantic_pipeline_blueprint.py src/runtime/tests/test_perception_factory_registry.py src/runtime/tests/test_perception_module.py::TestDetectorConfiguration -q
# 18 passed

python -m pytest src/runtime/tests/test_multifloor_sim_validation.py -q
# 26 passed

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --json-out artifacts/server_sim_closure_summary_g4_current.json
# exited 0; summary remains ok=false, execution_mode=summary_only,
# run_missing=false, gate_runs=[], simulation_only=true,
# real_robot_motion=false, cmd_vel_sent_to_hardware=false.

PYTHONPATH=src:. python sim/scripts/multifloor_nav_validation.py --skip-mujoco
# exited 0; non-motion report refreshed. Synthetic LiDAR localization and
# nav_kernel tracking replay passed; production PCT native runtime remains
# unavailable on this Windows / Python 3.13 host.
```

Current G4 full simulation closure is still intentionally not complete.
`artifacts/server_sim_closure_summary_g4_current.json` remains `ok=false` with
the same nine required gates blocking:
`multifloor_exploration`, `large_terrain`, `native_pct_mujoco`,
`fastlio2_dynamic_inspection`, `moving_obstacle_sweep`, `large_loop_closure`,
`gazebo_runtime`, `saved_map_relocalize`, and `pct_saved_map_navigation`.

Next execution goals from this checkpoint were executed as G22-G24 below.

## 2026-05-31 Continuation: Seed Hygiene And Remaining Gate Queue

G22. Done in this continuation pass: registry seeding the built-in `driver`
group no longer mutates `sys.path` in the normal repo runtime. The legacy
`drivers.real.thunder.connection` module still registers the compatibility
`driver/nova_dog` backend, but its path fallback now runs only when `core`
cannot be imported. `drivers.real.thunder.__init__` now exports legacy
attributes lazily, so importing `drivers.real.thunder.han_dog_module` does not
also import legacy blueprint helpers.

G23. Done in this continuation pass: SLAM plugin seeding and
`slam("bridge", enable_visual_backup=True)` no longer import OpenCV just to
register or build the depth visual odometry module. `DepthVisualOdomModule`
keeps its `visual_odom/depth` registry entry at import time, but caches a lazy
`cv2` import only when camera distortion maps, ORB activation, LK tracking, or
PnP solving actually use OpenCV.

G24. Done as a planning/audit checkpoint, not as gate execution: the remaining
G4 gates were sorted into host-specific execution lanes. The current Windows /
Python 3.13 host can refresh none of the nine remaining required gates as a
passing full-closure artifact because even the local non-ROS gates now require
native PCT modules built for Linux/S100P and CPython 3.10. The current G4
summary artifact remains summary-only and safe:
`execution_mode=summary_only`, `run_missing=false`, `gate_runs=[]`,
`simulation_only=true`, `real_robot_motion=false`, and
`cmd_vel_sent_to_hardware=false`.

G24 execution queue:

| Order | Gate | Required host lane | Expected report |
| ---: | --- | --- | --- |
| 1 | `large_terrain` | Linux or S100P/aarch64, CPython 3.10 ABI, PCT native libs; no ROS/MuJoCo required. | `artifacts/server_sim_closure/large_terrain/report.json` |
| 2 | `multifloor_exploration` | Linux or S100P/aarch64, CPython 3.10 ABI, PCT native libs; no ROS/MuJoCo required. | `artifacts/server_sim_closure/multifloor_exploration/report.json` |
| 3 | `native_pct_mujoco` | Linux + ROS 2 Humble + MuJoCo EGL + PCT native; isolated simulation domain. | `artifacts/server_sim_closure/native_pct_mujoco/report.json` |
| 4 | `fastlio2_dynamic_inspection` | Linux + ROS 2 Humble + MuJoCo EGL + PCT native; isolated simulation domain. | `artifacts/server_sim_closure/mujoco_fastlio2_live*/inspection*/*/report.json` |
| 5 | `moving_obstacle_sweep` | Linux + ROS 2 Humble + MuJoCo EGL + PCT native; depends on inspection reports/video. | `artifacts/server_sim_closure/moving_obstacle_sweep/report.json` |
| 6 | `large_loop_closure` | Linux + ROS 2 Humble + MuJoCo EGL + PCT native; isolated simulation domain. | `artifacts/server_sim_closure/large_loop_closure/report.json` |
| 7 | `gazebo_runtime` | Linux + ROS 2 Humble + Gazebo/Ignition; isolated `ROS_DOMAIN_ID` and Gazebo partition. | `artifacts/server_sim_closure/gazebo_runtime_explore/report_grid_astar_odomfoot.json` |
| 8 | `saved_map_relocalize` | Linux + ROS 2 Humble + MuJoCo/Fast-LIO live feed + localizer runtime + saved-map assets; no PCT requirement. | `artifacts/server_sim_closure/saved_map_relocalize_runtime/report.json` |
| 9 | `pct_saved_map_navigation` | Linux + ROS 2 Humble + MuJoCo EGL + PCT native; run after saved-map relocalization evidence exists. | `artifacts/server_sim_closure/pct_saved_map_navigation/report.json` |

Use `artifacts/server_sim_closure_summary_g4_current.json` field
`next_actions[]` as the command source of truth for these gates. Do not use
`--run-missing` on the Windows host; run the commands manually on an isolated
simulation host that has no physical robot drivers or hardware command
subscribers.

Latest safe validation evidence for G22-G24:

```bash
python -m pytest src/runtime/tests/test_plugin_seed.py::test_driver_plugin_seed_does_not_mutate_sys_path src/runtime/tests/test_plugin_seed.py::test_slam_plugin_seed_does_not_import_cv2_for_registration_only -q
# RED before implementation: 2 failed because driver seeding mutated sys.path
# and SLAM seeding imported cv2. After implementation: 2 passed.

python -m py_compile src/drivers/real/thunder/__init__.py src/drivers/real/thunder/blueprints.py src/drivers/real/thunder/connection.py src/localization/depth_visual_odom_module.py src/runtime/tests/test_plugin_seed.py
# passed

python -m pytest src/runtime/tests/test_plugin_seed.py -q
# 6 passed

python -m pytest src/runtime/tests/test_han_dog_module.py src/drivers/tests/test_driver_spec.py -q
# 22 passed

python -m pytest src/runtime/tests/test_stack_registry_resolution.py::test_slam_stack_prefers_registered_bridge_and_visual_odom_modules src/runtime/tests/test_stack_registry_resolution.py::test_slam_stack_visual_backup_does_not_import_cv2_at_build_time src/runtime/tests/test_plugin_seed.py::test_slam_plugin_seed_does_not_import_cv2_for_registration_only src/localization/tests/test_slam_backend_status.py src/localization/tests/test_slam_gnss_fusion.py -q
# 41 passed
```

Next execution goals:

- G25: if legacy `driver/nova_dog` is no longer required, remove it from
  default driver seeding or move it behind an explicit compatibility group.
- G26: add a machine-checkable isolated-host preflight for the G24 queue
  before any Linux/ROS2/MuJoCo gate is run.
- G27: refresh the first two PCT-native non-ROS gates (`large_terrain` then
  `multifloor_exploration`) on a Linux/S100P CPython 3.10 host, then re-run
  `server_sim_closure` summary-only.

## 2026-05-31 Continuation: Host Preflight Gate Queue

G26. Done in this continuation pass: `server_sim_closure.py` now has a
read-only `--host-preflight` mode for the G4 queue. This mode does not run any
gate command, rejects `--run-missing`, writes no hardware command, and reports
host suitability before a Linux/ROS2/MuJoCo/PCT gate is attempted. Its report
schema is explicit about the safety boundary:
`execution_mode=host_preflight_only`, `run_missing=false`, `gate_runs=[]`,
`simulation_only=true`, `real_robot_motion=false`, and
`cmd_vel_sent_to_hardware=false`.

G26a. Done after parallel safety review: `--json-out -` is now stdout-only, so
host preflight can be run without creating a report artifact. This closes the
strict read-only caveat for preflight use cases where "read-only" means no gate
execution, no hardware command, and no filesystem report write.

The current Windows / Python 3.13 host preflight artifact is
`artifacts/server_sim_closure_host_preflight_g4_current.json`. It marks only
the local non-motion gates as runnable:
`gateway_runtime_acceptance`, `routecheck_preflight`, and
`dynamic_obstacle_local_planner`. The remaining nine gates are blocked:
`multifloor_exploration`, `large_terrain`, `native_pct_mujoco`,
`fastlio2_dynamic_inspection`, `moving_obstacle_sweep`, `large_loop_closure`,
`gazebo_runtime`, `saved_map_relocalize`, and `pct_saved_map_navigation`.

Primary blockers on this host:

- PCT-native gates require Linux or S100P/aarch64 plus CPython 3.10-compatible
  native PCT modules. Current host is Windows / AMD64 / CPython 3.13.
- ROS gates require ROS 2 Humble sourced and a nonzero isolated
  `ROS_DOMAIN_ID`.
- Hardware-command safety gates require a ROS graph audit showing no physical
  robot subscribers on hardware command topics.
- Gazebo gates require `gz` or `ign`.
- Saved-map relocalization requires ROS 2 Humble plus the localizer runtime and
  saved-map assets in an isolated simulation domain.

Latest safe validation evidence for G26:

```bash
python -m pytest src/runtime/tests/test_server_sim_closure.py::test_server_sim_host_preflight_blocks_pct_gate_on_wrong_host src/runtime/tests/test_server_sim_closure.py::test_server_sim_host_preflight_accepts_local_non_motion_gate src/runtime/tests/test_server_sim_closure.py::test_server_sim_host_preflight_requires_isolated_ros_domain_and_hardware_audit -q
# 3 passed

python -m pytest src/runtime/tests/test_server_sim_closure.py::test_server_sim_host_preflight_cli_writes_read_only_report src/runtime/tests/test_server_sim_closure.py::test_server_sim_host_preflight_rejects_run_missing_mix -q
# 2 passed

python -m py_compile sim/scripts/server_sim_closure.py src/runtime/tests/test_server_sim_closure.py
# passed

python -m pytest src/runtime/tests/test_server_sim_closure.py -q
# 130 passed

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --host-preflight --preset g4_server_full_sim --required-only --json-out artifacts/server_sim_closure_host_preflight_g4_current.json
# exited 0; execution_mode=host_preflight_only, run_missing=false,
# gate_runs=[], simulation_only=true, real_robot_motion=false,
# cmd_vel_sent_to_hardware=false. Runnable gates: gateway_runtime_acceptance,
# routecheck_preflight, dynamic_obstacle_local_planner. Blocked gates: the
# nine host-specific G4 gates listed above.

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --host-preflight --preset g4_server_full_sim --required-only --json-out -
# exited 0; no file named '-' was created. stdout reported
# execution_mode=host_preflight_only, run_missing=false, gate_runs=[],
# simulation_only=true, real_robot_motion=false,
# cmd_vel_sent_to_hardware=false.

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --json-out artifacts/server_sim_closure_summary_g4_current.json
# exited 0; execution_mode=summary_only, run_missing=false, gate_runs=[].
# Current G4 full closure remains ok=false with the same nine required gates
# missing or failed.
```

Next execution goals from this checkpoint:

- G27: run `--host-preflight --strict` on the isolated Linux/S100P CPython 3.10
  PCT host. Stop if PCT runtime, Python ABI, or safety flags fail.
- G28: refresh `large_terrain` and `multifloor_exploration` on that host, then
  rerun summary-only G4 closure and archive the reports under
  `artifacts/server_sim_closure/`.
- G29: on the isolated ROS 2 Humble simulation host, capture the zero-subscriber
  ROS graph audit for hardware command topics before any MuJoCo/Gazebo gate.
- G30: only after G27-G29 pass, run the MuJoCo/Fast-LIO/Gazebo/saved-map gates
  in G24 order and refresh the full `server_sim_closure` summary.
- G31: close remaining sim folder migration gaps: `sim/launch/sim.launch.py`
  references a missing `sim/scripts/run_global_planner.py`, legacy
  `test_*.sh` scripts still hard-code `/tmp/nova_sim/robot/*`, optional Go1
  assets are documented but absent, and `sim/semantic`, `sim/meshes`,
  `sim/maps`, `sim/configs`, plus local generated-output folders still need a
  prune/archive/README decision.
- G32: make detector/encoder runtime replacement truly hot-swappable by
  preloading the new backend outside the frame-processing lock, atomically
  swapping on success, and shutting down the old backend after the swap.
- G33: unify standalone `DetectorModule` and `EncoderModule` with the
  perception provider registry so there is one backend contract for both
  stack-level and standalone use.
- G34: either implement runtime reconfigure for `TerrainModule`,
  `LocalPlannerModule`, and `PathFollowerModule`, or remove those unsupported
  targets from the Gateway backend-switch surface.
- G35: add a runtime evidence gate that records whether `_nav_kernel` actually
  loaded on the target host. The configuration can say `nanobind/nav_kernel`,
  but the current modules may fall back to `cmu_py/pid` if native bindings are
  missing; full simulation evidence must record the backend that actually ran.
- G36: document the algorithm chain in the closure evidence:
  `LiDAR/SLAM -> SlamBridgeModule -> maps/Terrain -> TraversabilityCost +
  NavigationModule(PCT/A*) -> LocalPlannerModule(nav_kernel/CMU) ->
  PathFollowerModule(nav_kernel) -> CmdVelMux -> Driver/SafetyRing`.
- G37: do not claim a DIMOS or external-method performance win until the same
  scene, seed, map/tomogram, planner inputs, and hardware/runtime envelope have
  fresh metrics for coverage, path length, time-to-goal, collision clearance,
  replan count, tracking error, CPU/latency, large-loop closure, and moving
  obstacle behavior.

Parallel-agent findings merged into this checkpoint:

- Simulation folder boundary: `sim/engine`, `sim/worlds`, `sim/assets`, and
  `sim/scripts` now have clear roles, but legacy launch/script references and
  optional Go1/external assets still need a cleanup decision.
- Safety preflight: core constraints hold (`gate_runs=[]`, no `--run-missing`,
  no hardware command). The stricter no-artifact case is now covered by
  `--json-out -`. Remaining caution: hardware subscriber detection is a ROS
  graph heuristic, so ROS gates still need a zero-subscriber audit artifact on
  the target simulation host.
- Moduleization: most stack factories are registry-first, but runtime hot-swap
  is not complete. Perception backend replacement currently loads under a
  shared lock and can block frame processing; standalone Detector/Encoder
  modules still have direct backend branches; motion backend reconfigure is
  advertised by Gateway but not implemented in all motion modules.
- Algorithm chain: current local planning is CMU/nav_kernel style point-cloud
  voxel scoring through `LocalPlannerModule`, not DWA/TEB and not ESDF-first.
  `TARE` enters through exploration; `wavefront` is enabled inside the
  navigation stack for the `explore` profile. DIMOS comparison remains an
  evidence task, not a conclusion.

## 2026-05-31 Continuation: Legacy Sim Cleanup And Backend Evidence

G31. Done in this continuation pass: close the high-signal simulation folder
migration gaps without moving stable entrypoints. `sim/scripts/run_global_planner.py`
now exists as a guarded legacy ROS launch wrapper that requires an isolated
nonzero `ROS_DOMAIN_ID` and only remaps planner/path/status topics, not
hardware velocity topics. Legacy Nova shell helpers now default to repo-local
`sim/robots/nova_dog/robot_with_camera.xml` through `LINGTU_SIM_DIR` and
`LINGTU_NOVA_SCENE_XML`, and stale `/tmp/nova_sim` guidance was removed.
Optional Go1 playground assets are documented as external placeholders and not
part of the G4 server closure.

G32. Done in this continuation pass: `PerceptionModule` detector/encoder
runtime replacement now creates and loads candidate backends outside
`_backend_lock`, swaps active references under the lock, disposes the previous
backend after successful swap, and disposes a failed candidate while keeping
the previous active backend intact. This closes the frame-processing lock and
resource leak issues found by the xhigh audit.

G33. Done in this continuation pass: standalone `DetectorModule` and
`EncoderModule` now resolve through the same `perception_detector` and
`perception_encoder` registries used by `PerceptionModule`. Built-in
standalone providers cover `yoloe`, `yolo_world`, `bpu`, `grounding_dino`,
`clip`, and `mobileclip`; GroundingDINO standalone config maps confidence to
`box_threshold`.

G34. Still open: motion runtime backend switching remains fail-closed for
`TerrainModule`, `LocalPlannerModule`, and `PathFollowerModule`. Keep Gateway
or MCP surfaces honest: either implement a tested safe switch path later, or
continue reporting `backend_reconfigure_unsupported` instead of implying a hot
swap exists.

G35. Done for current non-motion evidence surfaces: dynamic obstacle,
multifloor command-flow, large-terrain, and multifloor global-planner reports
now distinguish requested backend/planner from actual effective backend/planner.
`dynamic_obstacle_local_planner` records `algorithm_backends.local_planner`
from the actual module fallback state and marks path follower as
`not_exercised`. `multifloor_nav_validation` records actual local planner and
path follower backends in command-flow and propagates them to route/matrix
reports. `large_terrain_nav_validation` carries `GlobalPlannerService`
`last_plan_report` through `planner_requested`, `selected_planner`,
`fallback_reason`, `plan_safety_policy`, and `rejected_plans`, while explicitly
marking local planner/path follower as `not_exercised`. Multifloor PCT gate
now exposes the same requested/effective planner split and does not count a
PCT-to-A* fallback as native PCT evidence.

G36. Done in this continuation pass: the algorithm chain is documented in this
plan, validation scripts expose requested-versus-actual backend evidence, and
`server_sim_closure` now lifts per-gate `algorithm_backends` into one
top-level closure table. This lets the G4 summary show which gates actually ran
`nanobind`/`nav_kernel`, which gates only exercised global planning, and which
motion stages remain `not_exercised` without digging into each raw report.

G37. Still open: no DIMOS or external-method performance win is claimed. The
next valid comparison must use the same scene, seed, map/tomogram, planner
inputs, and runtime envelope, then report coverage, path length, time-to-goal,
clearance/collisions, replan count, tracking error, CPU/latency, large-loop
closure, and moving-obstacle behavior.

Latest safe validation evidence for G31-G35:

```bash
python -m pytest src/runtime/tests/test_dynamic_obstacle_local_planner_gate.py src/runtime/tests/test_multifloor_sim_validation.py::test_command_flow_reports_requested_and_effective_algorithm_backends src/runtime/tests/test_multifloor_sim_validation.py::test_pct_global_plan_reports_effective_planner_after_fallback src/runtime/tests/test_large_terrain_scenario.py::test_large_terrain_validation_report_is_non_motion_and_route_safe src/runtime/tests/test_large_terrain_scenario.py::test_large_terrain_validation_records_effective_global_planner_when_service_falls_back src/runtime/tests/test_sim_runtime_compat.py::test_legacy_sim_launch_global_planner_entrypoint_exists_and_is_guarded src/runtime/tests/test_sim_runtime_compat.py::test_legacy_manual_nova_scripts_default_to_current_robot_asset_paths src/runtime/tests/test_sim_runtime_compat.py::test_optional_go1_asset_contract_has_placeholder_readme src/runtime/tests/test_sim_runtime_compat.py::test_sim_boundary_indexes_document_stable_contracts -q
# 10 passed

python -m pytest src/runtime/tests/test_runtime_backend_switch.py src/runtime/tests/test_perception_decoupled.py -q
# 33 passed

python -m py_compile sim/scripts/run_global_planner.py sim/scripts/dynamic_obstacle_local_planner_gate.py sim/scripts/multifloor_nav_validation.py sim/scripts/large_terrain_nav_validation.py src/perception/semantic_perception/perception_module.py src/perception/semantic_perception/detector_module.py src/perception/semantic_perception/encoder_module.py
# passed

python -m pytest src/runtime/tests/test_multifloor_sim_validation.py -q
# 28 passed

python -m pytest src/runtime/tests/test_large_terrain_scenario.py -q
# 11 passed

PYTHONPATH=src:. python sim/scripts/dynamic_obstacle_local_planner_gate.py --json-out artifacts/server_sim_closure/dynamic_obstacle_local_planner/report.json
# exited 0; ok=true, backend_requested=nanobind, backend_actual=nanobind,
# algorithm_backends.local_planner.backend_actual=nanobind,
# algorithm_backends.path_follower.status=not_exercised,
# simulation_only=true, cmd_vel_sent_to_hardware=false.

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --host-preflight --preset g4_server_full_sim --required-only --json-out artifacts/server_sim_closure_host_preflight_g4_current.json
# exited 0; execution_mode=host_preflight_only, run_missing=false,
# gate_runs=0, runnable=3, blocked=9, simulation_only=true,
# real_robot_motion=false, cmd_vel_sent_to_hardware=false.

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --json-out artifacts/server_sim_closure_summary_g4_current.json
# exited 0; execution_mode=summary_only, ok=false, run_missing=false,
# gate_runs=0, dynamic_obstacle_local_planner verified=true,
# simulation_only=true, real_robot_motion=false,
# cmd_vel_sent_to_hardware=false.

python -m pytest src/runtime/tests/test_server_sim_closure.py::test_server_sim_closure_summarizes_algorithm_backends_from_gate_reports -q
# RED before implementation: KeyError on summary["algorithm_backends"].
# After implementation: 1 passed.

python -m pytest src/runtime/tests/test_server_sim_closure.py::test_server_sim_closure_summarizes_algorithm_backends_from_gate_reports src/runtime/tests/test_server_sim_closure.py::test_server_sim_closure_can_summarize_required_only src/runtime/tests/test_dynamic_obstacle_local_planner_gate.py src/runtime/tests/test_multifloor_sim_validation.py::test_command_flow_reports_requested_and_effective_algorithm_backends src/runtime/tests/test_multifloor_sim_validation.py::test_pct_global_plan_reports_effective_planner_after_fallback src/runtime/tests/test_large_terrain_scenario.py::test_large_terrain_validation_report_is_non_motion_and_route_safe src/runtime/tests/test_large_terrain_scenario.py::test_large_terrain_validation_records_effective_global_planner_when_service_falls_back -q
# 8 passed

python -m py_compile sim/scripts/server_sim_closure.py src/runtime/tests/test_server_sim_closure.py
# passed

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --json-out artifacts/server_sim_closure_summary_g4_current.json
# exited 0; execution_mode=summary_only, ok=false, run_missing=false,
# gate_runs=0, algorithm_backends currently includes
# dynamic_obstacle_local_planner from fresh local evidence, and safety fields
# remain simulation_only=true, real_robot_motion=false,
# cmd_vel_sent_to_hardware=false.
```
