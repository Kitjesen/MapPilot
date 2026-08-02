# sim/scripts Index

Status: current script index as of 2026-07-18. This is a live contract/index;
dated reports and old closure notes remain evidence only.

This directory is a stable script contract, not a package boundary. The current
MuJoCo command and test interface lives only under `sim/scripts/mujoco/`.

## Canonical MuJoCo Entrypoints

Use these paths for new commands, docs, and tests:

| Entrypoint | Role |
| --- | --- |
| `mujoco/launch_fastlio2_live.sh` | MuJoCo + Fast-LIO2 live simulation launcher |
| `mujoco/live_gate.py` | MuJoCo live LiDAR/IMU simulation gate |
| `mujoco/native_dds_sensors.py` | MuJoCo native DDS sensor publisher gate |
| `mujoco/native_navigation_acceptance.py` | Native DDS navigation acceptance harness: MuJoCo sensors/processes, C++ SLAM/traversability/nav endpoint, final DDS command tap, optional video |
| `mujoco/long_range_navigation_acceptance.py` | Repeated 50-70 m native-DDS navigation campaign runner over `native_navigation_acceptance.py` |
| `mujoco/native_control_mode_acceptance.py` | Run/evaluate wrapper for native endpoint `autonomy`, `teleop`, and `teleop_avoid` control-mode promotion |
| `mujoco/saved_map_plan_gate.py` | Same-source saved-map planning gate |
| `mujoco/saved_map_tracking_gate.py` | Saved-map global path plus MuJoCo tracking gate |
| `mujoco/saved_map_quality_gate.py` | Saved-map PCD/plan quality gate |
| `mujoco/continuous_mapping_quality_gate.py` | 3-5 min continuous native DDS mapping gate (bridge + continuity + scale + map quality) |
| `run_sunrise_continuous_mapping_gate.py` | SSH runner for the continuous mapping gate on sunrise |
| `mujoco/native_pct_gate.py` | Native PCT + MuJoCo gate |
| `mujoco/navigation_audit.py` | MuJoCo navigation wiring audit |
| `mujoco/record_policy_nav_video.py` | Policy navigation video recorder |
| `mujoco/record_thunderv4_mid360_policy.py` | Thunderv4 MID-360 policy-scene recording helper |
| `mujoco/record_thunderv4_stair_showcase.py` | Thunderv4 stair showcase recorder |

## Other Public Entrypoints

These non-MuJoCo root-level paths remain current public entrypoints:

| Entrypoint | Role |
| --- | --- |
| `server_sim_closure.py` | Server-side simulation evidence aggregator |
| `pct_saved_map_navigation_gate.py` | Saved-map PCT navigation gate |
| `saved_map_relocalize_runtime_gate.py` | Saved-map relocalization runtime gate |
| `routecheck_preflight_gate.py` | Gateway route preflight gate |
| `multifloor_nav_validation.py` | Multi-floor validation wrapper |
| `policy_nav_smoke.py` | OctoPlanner + nanobind/nav_kernel policy-mode navigation smoke test |
| `run_dimos_linux_closure.sh` | Target-host DimOS closure runner |
| `launch_lingtu_gazebo_industrial_demo.sh` | Gazebo industrial simulation launcher |

## Safety Classes

Use the safety class before running a script:

| Safety class | Meaning | Examples |
| --- | --- | --- |
| summary-only unless --run-missing | Reads existing reports and writes an aggregate summary. It must not launch missing gates unless `--run-missing` is passed. Explicit non-motion source materialization, such as `server_sim_closure.py --navigation-replay-deviation-topic-jsonl`, may write the requested local replay report before summarizing. Use `--host-preflight` to check host suitability without gate execution, `--skip-host-blocked` with `--run-missing` only for diagnostic local aggregation of host-blocked gates, and `--json-out -` for stdout-only reporting. Acceptance runs should use `run_dimos_linux_closure.sh`, which preflights first and does not pass `--skip-host-blocked` to runtime execution. | `server_sim_closure.py`, `dimos_gap_report.py` |
| local non-motion | Runs local Python checks, asset generation, or in-memory module dataflow. It must report `real_robot_motion=false` and `cmd_vel_sent_to_hardware=false`. | `multifloor_nav_validation.py --skip-mujoco`, `large_terrain_nav_validation.py`, `routecheck_preflight_gate.py`, `blocked_route_replan_gate.py`, `navigation_replay_deviation_gate.py` |
| simulated motion only | May move a MuJoCo/Gazebo/Unity simulated robot. It must stay disconnected from physical robot drivers and hardware command subscribers. | `policy_nav_smoke.py`, `mujoco/native_navigation_acceptance.py`, `mujoco/long_range_navigation_acceptance.py`, `mujoco/native_control_mode_acceptance.py`, `mujoco/native_pct_gate.py`, `mujoco/live_gate.py` |
| ROS2 isolated simulation | May source ROS 2, launch sim nodes, or publish sim topics. Use an isolated `ROS_DOMAIN_ID`; never run on a robot ROS domain. | `gazebo_runtime_gate.py`, `mujoco/launch_fastlio2_live.sh`, `launch_lingtu_gazebo_industrial_demo.sh` |
| legacy manual | Historical helpers or dataset scripts. They can source install spaces, start subprocesses, or assume local assets; they are not part of the G4 closure unless another gate explicitly consumes their report. | `_run_legkilo_test.sh`, `run_legkilo_test.sh`, `test_*.sh`, legacy Go1 demos |

## Gate Scripts

- `server_sim_closure.py` - G4 evidence aggregator; summary-only unless --run-missing or an explicit non-motion/report-only source is provided. `--navigation-replay-deviation-topic-jsonl <path>` materializes the replay/deviation report from recorded topic JSONL, `--moving-obstacle-sweep-report-glob <glob>` re-aggregates existing moving-obstacle live child reports without launching the ROS2/MuJoCo matrix, `--host-preflight` reports whether the current host can safely run the selected gates without launching them, and `--skip-host-blocked` lets diagnostic `--run-missing` runs skip and report gates blocked by that preflight. Do not use `--skip-host-blocked` in DimOS readiness acceptance.
- `dimos_host_preflight_guard.py` - Read-only guard used by the Linux closure runner. It exits 0 only when a host preflight report is green; red reports exit 3 and print `host_setup_plan` failed checks plus diagnostic commands.
- `run_dimos_linux_closure.sh` - Target-host DimOS closure runner for the current benchmark sequence. Default is `--dry-run`; `--execute` is required before it sources ROS 2 or launches missing gates. It refuses non-Linux `--execute`, enforces an isolated ROS domain, runs host preflight before `--run-missing`, and always stops on red preflight. If `/opt/ros/humble/setup.bash` is missing, it still writes the preflight diagnostic report and then refuses runtime gates through the guard. Red preflight refusal prints the `host_setup_plan` failed checks and diagnostic commands. When runtime gates write a red summary, the runner still regenerates the DimOS gap report before returning a nonzero status.
- `dimos_gap_report.py` - DimOS-style gap matrix over the `dimos_benchmark` gate sequence; reads an existing closure summary or current reports, embedded `run_missing_host_preflight` evidence, `--host-preflight` read-only host suitability evidence, `--host-preflight-report <json>` evidence produced earlier by `server_sim_closure.py --host-preflight`, and `--include-dataflow` runtime-report dataflow blockers; `--format shell` exports a command checklist without launching gates and auto-enables the same read-only dataflow inspection if the flag was omitted. Host setup diagnostics are emitted as executable commands, while blocked gate commands stay commented as `# BLOCKED:`.
- `setup_linux_validation_host.sh` - Canonical ROS-free Linux validation-host setup for PCT, MuJoCo, nav-kernel, MID-360 asset, multi-floor, and routecheck gates. The optional Fast-LIO2 compatibility setup is quarantined at `scripts/compat/ros2/setup_fastlio2_validation_host.sh`.
- `pct_runtime_preflight.py` - Read-only PCT planner-runtime diagnostic. It inspects the selected runtime (default `rust_process`) and writes canonical `pct_planner_runtime` plus `pct_planner_runtime_ok` evidence to `artifacts/server_sim_closure/pct_runtime_preflight/report.json`. Use non-strict mode to capture blockers and `--strict` to require the selected runtime. Explicit `native` selection is parity-only and reports its own legacy host/ABI requirements; those requirements are not default readiness checks.
- `routecheck_preflight_gate.py` - Gateway route preflight with no goal or cmd_vel publication.
- `blocked_route_replan_gate.py` - Gateway blocked-route replanning preflight with a synthetic route obstruction and no goal or cmd_vel publication.
- `navigation_replay_deviation_gate.py` - Offline replay/deviation check for routecheck-derived, JSON trace, or recorded topic JSONL global path, local path, cmd_vel, and odometry traces without hardware output.
- `gateway_goal_dry_run_gate.py` - Gateway dry-run goal contract.
- `dynamic_obstacle_local_planner_gate.py` - Dynamic-obstacle nanobind local planner gate.
- `large_loop_closure_gate.py` - Large-loop closure report validator.
- `moving_obstacle_sweep_gate.py` - Moving-obstacle sweep report validator.
- `fastlio_speed_boundary_gate.py` - Fast-LIO speed-boundary gate.
- `mujoco/live_gate.py` - MuJoCo live LiDAR/IMU plus Fast-LIO2 simulation gate.
- `mujoco/native_navigation_acceptance.py` - Current native-DDS navigation acceptance harness. It verifies the native process chain and final DDS command tap in MuJoCo; it is not field readiness.
- `mujoco/long_range_navigation_acceptance.py` - Repeated 50-70 m native-DDS navigation campaign runner. A single accepted run is not the same as a completed `10/10` campaign.
- `mujoco/native_control_mode_acceptance.py` - Current control-mode promotion wrapper. It has separate `run` and `evaluate` actions and rejects handwritten summaries.
- `policy_nav_smoke.py` - Current product-style simulated motion smoke: OctoPlanner is the configured global planner, LocalPlanner runs the nanobind backend, PathFollower runs nav_kernel, and commands stay inside the MuJoCo policy driver through nav.velocity_mux.
- `mujoco/native_pct_gate.py` - Legacy compatibility coverage for native PCT plus ROS2 local planner/path follower into MuJoCo simulation. `--contract-only` validates the PCT source-report/no-fallback/same-source artifact contract without launching ROS2 or MuJoCo; it also emits `command_generation` with source-report fingerprint and the localPlanner/pathFollower command contract. This is a compatibility wiring check, not the current product local-autonomy runtime.
- `mujoco/saved_map_tracking_gate.py` - Current native saved-map tracking check: builds/loads `map.pcd` and `octomap.ot`, runs OctoPlanner3D, feeds the global path through `lingtu_nav_kernel.LocalPlanner` and `lingtu_nav_kernel.compute_control`, then applies the resulting cmd_vel to a MuJoCo kinematic robot. It is simulated motion only and never connects to robot hardware.
- `gazebo_runtime_gate.py` - Gazebo runtime simulation gate; requires ROS2 isolated simulation and records frontier post-pass no-gain/stall observation evidence.
- `pct_saved_map_navigation_gate.py` - Saved-map PCT navigation gate. `--contract-only --source-report <json>` checks relocalization/source-report/map/tomogram binding, including same-source hash identity, without running PCT preview or MuJoCo motion; it is a saved-map wiring check, not full navigation evidence.
- `saved_map_relocalize_contract_gate.py` / `saved_map_relocalize_runtime_gate.py` - Saved-map relocalization gates. `saved_map_relocalize_runtime_gate.py --preflight-only` checks saved-map assets, localizer config, host markers, and ROS 2 Python importability without launching MuJoCo/Fast-LIO/localizer processes.

## Validation And Diagnosis

- `multifloor_nav_validation.py` - Multi-floor navigation validation. Default safe mode is local non-motion; `--bridge-loop` changes it to simulated motion only.
- `large_terrain_nav_validation.py` - Large-terrain global-planning asset validation. It does not exercise local planner or path follower backends and reports those algorithm surfaces as `not_exercised`.
- PCT cases record the selected `pct_planner_runtime` and the summary
  `pct_planner_runtime_ok`. The isolated child disables trajectory optimization
  by default with `LINGTU_PCT_OPTIMIZE_TRAJECTORY=0`, reporting
  `pct_optimizer_enabled=false` and `pct_planner_path_mode=astar_raw_path`.
  The other canonical path mode is `optimized_trajectory`; neither value means
  that the service fell back to a different planner, and this gate is not a
  MuJoCo motion-loop pass.
- The 2026-06-08 target-host preflight checks both the ROS2 `local_planner`
  package and the official MID-360 scan-pattern asset before closed-loop
  MuJoCo/PCT gates. If `ros2 pkg executables local_planner` does not list
  `localPlanner` and `pathFollower`, or if `sim/assets/livox/mid360.npy` is
  missing or has the wrong SHA-256, `run_dimos_linux_closure.sh --execute`
  stops at `dimos_host_preflight_guard.py` and does not launch runtime gates.
- The latest target-host preflight
  `artifacts/server_sim_closure/host_preflight_after_large_loop_report_guard.json`
  is green after the ROS 2, MuJoCo, PCT, Fast-LIO2, local planner, localizer,
  MID-360 asset, and hardware-subscriber checks. `native_pct_mujoco`,
  `gazebo_runtime`, `saved_map_relocalize`, and `pct_saved_map_navigation` now
  pass on the target host; the saved-map PCT report records
  `selected_planner=pct`,
  `pct_optimizer_enabled=false`, and
  `pct_planner_path_mode=astar_raw_path`. The overall DimOS closure
  remains red at 12/13 required gates because `large_loop_closure` still fails.
  The latest large-loop live run ended without writing a child report before
  the launcher fallback was installed; future no-report exits are recorded as
  red `runtime_report_missing_after_launcher` diagnostics instead of allowing
  stale aggregation.
- `full_sim_validation.py` - Compatibility wrapper for the full simulation validation gate; canonical G4 closure aggregation is `server_sim_closure.py`.
- `large_loop_diagnosis_matrix.py` - Large-loop diagnosis matrix.
- `render_slam_validation_screenshots.py` - SLAM validation screenshot renderer.
- `run_slam_dataset_test_v2.py` - SLAM dataset test runner.

## Demo And Runtime Entrypoints

- `run_sim.py` - Generic simulation launcher.
- `run_person_following.py` - Person-following simulation launcher.
- `run_semantic_full_stack.py` - Semantic full-stack simulation launcher.
- `demo_search.py` - Search demo.
- `policy_nav_smoke.py` - OctoPlanner + nanobind/nav_kernel policy-mode navigation smoke test; simulated motion only.
- `nav_overlay.py` - Navigation overlay visualization.
- `view_scene.py` - Scene viewer.
- `benchmark_following.py` - Person-following benchmark.
- `mujoco/record_policy_nav_video.py` / `render_gazebo_frontier_video.py` - Video recording/rendering helpers.
- `go1_indoor_nav.py` / `go1_nav_full.py` - legacy Go1 demos; require optional `sim/robots/go1_playground/` assets and are not part of current G4 closure.

## Shell Launchers And Legacy Helpers


- `launch_cmu_unity_baseline.sh` - external upstream benchmark launcher only. It does not invoke ProductControl or produce a RunPlan. Manual relay experiments may use `sim/engine/bridge/cmu_unity_lingtu_adapter.py`; they are adapter evidence, not a LingTu Product runtime.
- `launch_lingtu_gazebo_industrial_demo.sh` - Gazebo industrial simulation demo launcher.
- `_run_legkilo_test.sh` / `run_legkilo_test.sh` - legacy/manual dataset helper scripts. They may source ROS/install spaces, start SLAM dataset processes, or clean up external processes; do not include them in the G4 server closure.
- `test_*.sh` - integration smoke helpers. Treat as legacy manual unless a gate documents a stricter contract.
- `install_deps.sh` - optional dependency installer.
- `fastlio_speed_scan_plan.sh` - speed scan helper.

## Tooling

- `algorithm_dataflow_summary.py` - Thin CLI wrapper around `sim.diagnostics.dataflow_report` for flat live Fast-LIO reports; `dimos_gap_report.py --include-dataflow` and Gateway benchmark diagnostics use the same core parser for aggregate/wrapper/native gate reports.
