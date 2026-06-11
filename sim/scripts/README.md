# sim/scripts Index

This directory is a stable script contract, not a package boundary. Many
profiles, tests, and evidence gates refer to `sim/scripts/<name>` directly, so
scripts stay in place and are grouped by naming convention.

## Safety Classes

Use the safety class before running a script:

| Safety class | Meaning | Examples |
| --- | --- | --- |
| summary-only unless --run-missing | Reads existing reports and writes an aggregate summary. It must not launch missing gates unless `--run-missing` is passed. Explicit non-motion source materialization, such as `server_sim_closure.py --navigation-replay-deviation-topic-jsonl`, may write the requested local replay report before summarizing. Use `--host-preflight` to check host suitability without gate execution, `--skip-host-blocked` with `--run-missing` only for diagnostic local aggregation of host-blocked gates, and `--json-out -` for stdout-only reporting. Acceptance runs should use `run_dimos_linux_closure.sh`, which preflights first and does not pass `--skip-host-blocked` to runtime execution. | `server_sim_closure.py`, `dimos_gap_report.py` |
| local non-motion | Runs local Python checks, asset generation, or in-memory module dataflow. It must report `real_robot_motion=false` and `cmd_vel_sent_to_hardware=false`. | `multifloor_nav_validation.py --skip-mujoco`, `large_terrain_nav_validation.py`, `routecheck_preflight_gate.py`, `blocked_route_replan_gate.py`, `navigation_replay_deviation_gate.py` |
| simulated motion only | May move a MuJoCo/Gazebo/Unity simulated robot. It must stay disconnected from physical robot drivers and hardware command subscribers. | `policy_nav_smoke.py`, `native_pct_mujoco_gate.py`, `mujoco_fastlio2_live_gate.py` |
| ROS2 isolated simulation | May source ROS 2, launch sim nodes, or publish sim topics. Use an isolated `ROS_DOMAIN_ID`; never run on a robot ROS domain. | `gazebo_runtime_gate.py`, `launch_mujoco_fastlio2_live.sh`, `launch_lingtu_gazebo_industrial_demo.sh` |
| legacy manual | Historical helpers or dataset scripts. They can source install spaces, start subprocesses, or assume local assets; they are not part of the G4 closure unless another gate explicitly consumes their report. | `_run_legkilo_test.sh`, `run_legkilo_test.sh`, `test_*.sh`, legacy Go1 demos |

## Gate Scripts

- `server_sim_closure.py` - G4 evidence aggregator; summary-only unless --run-missing or an explicit non-motion/report-only source is provided. `--navigation-replay-deviation-topic-jsonl <path>` materializes the replay/deviation report from recorded topic JSONL, `--moving-obstacle-sweep-report-glob <glob>` re-aggregates existing moving-obstacle live child reports without launching the ROS2/MuJoCo matrix, `--host-preflight` reports whether the current host can safely run the selected gates without launching them, and `--skip-host-blocked` lets diagnostic `--run-missing` runs skip and report gates blocked by that preflight. Do not use `--skip-host-blocked` in DimOS readiness acceptance.
- `dimos_host_preflight_guard.py` - Read-only guard used by the Linux closure runner. It exits 0 only when a host preflight report is green; red reports exit 3 and print `host_setup_plan` failed checks plus diagnostic commands.
- `run_dimos_linux_closure.sh` - Target-host DimOS closure runner for the current benchmark sequence. Default is `--dry-run`; `--execute` is required before it sources ROS 2 or launches missing gates. It refuses non-Linux `--execute`, enforces an isolated ROS domain, runs host preflight before `--run-missing`, and always stops on red preflight. If `/opt/ros/humble/setup.bash` is missing, it still writes the preflight diagnostic report and then refuses runtime gates through the guard. Red preflight refusal prints the `host_setup_plan` failed checks and diagnostic commands. When runtime gates write a red summary, the runner still regenerates the DimOS gap report before returning a nonzero status.
- `dimos_gap_report.py` - DimOS-style gap matrix over the `dimos_benchmark` gate sequence; reads an existing closure summary or current reports, embedded `run_missing_host_preflight` evidence, `--host-preflight` read-only host suitability evidence, `--host-preflight-report <json>` evidence produced earlier by `server_sim_closure.py --host-preflight`, and `--include-dataflow` runtime-report dataflow blockers; `--format shell` exports a command checklist without launching gates and auto-enables the same read-only dataflow inspection if the flag was omitted. Host setup diagnostics are emitted as executable commands, while blocked gate commands stay commented as `# BLOCKED:`.
- `pct_runtime_preflight.py` - Read-only PCT native runtime diagnostic. It checks host platform, CPython ABI, PCT extension modules, and shared libraries, then writes `artifacts/server_sim_closure/pct_runtime_preflight/report.json`. Use non-strict mode before setup to capture blockers; use `--strict` after `scripts/deploy/setup_server_ros_pct.sh` to prove PCT-backed gates are runnable.
- `routecheck_preflight_gate.py` - Gateway route preflight with no goal or cmd_vel publication.
- `blocked_route_replan_gate.py` - Gateway blocked-route replanning preflight with a synthetic route obstruction and no goal or cmd_vel publication.
- `navigation_replay_deviation_gate.py` - Offline replay/deviation check for routecheck-derived, JSON trace, or recorded topic JSONL global path, local path, cmd_vel, and odometry traces without hardware output.
- `gateway_goal_dry_run_gate.py` - Gateway dry-run goal contract.
- `dynamic_obstacle_local_planner_gate.py` - Dynamic-obstacle local planner gate.
- `large_loop_closure_gate.py` - Large-loop closure report validator.
- `moving_obstacle_sweep_gate.py` - Moving-obstacle sweep report validator.
- `fastlio2_rosbag_replay_gate.py` - Fast-LIO2 rosbag replay gate.
- `fastlio_speed_boundary_gate.py` - Fast-LIO speed-boundary gate.
- `mujoco_fastlio2_live_gate.py` - MuJoCo live LiDAR/IMU plus Fast-LIO2 simulation gate.
- `native_pct_mujoco_gate.py` - Native PCT plus ROS2 local planner/path follower into MuJoCo simulation. `--contract-only` validates the PCT source-report/no-fallback/same-source artifact contract without launching ROS2 or MuJoCo; it also emits `command_generation` with source-report fingerprint and the localPlanner/pathFollower command contract. This is a wiring check, not runtime motion evidence.
- `gazebo_runtime_gate.py` - Gazebo runtime simulation gate; requires ROS2 isolated simulation and records frontier post-pass no-gain/stall observation evidence.
- `pct_saved_map_navigation_gate.py` - Saved-map PCT navigation gate. `--contract-only --source-report <json>` checks relocalization/source-report/map/tomogram binding, including same-source hash identity, without running PCT preview or MuJoCo motion; it is a saved-map wiring check, not full navigation evidence.
- `saved_map_relocalize_contract_gate.py` / `saved_map_relocalize_runtime_gate.py` - Saved-map relocalization gates. `saved_map_relocalize_runtime_gate.py --preflight-only` checks saved-map assets, localizer config, host markers, and ROS 2 Python importability without launching MuJoCo/Fast-LIO/localizer processes.
- `cmu_unity_runtime_gate.py` / `cmu_unity_sim_gate.py` - CMU Unity runtime and contract gates, including strict late-window frontier/no-gain stall evidence and TARE strategy-quality stats.

## Validation And Diagnosis

- `multifloor_nav_validation.py` - Multi-floor navigation validation. Default safe mode is local non-motion; `--bridge-loop` changes it to simulated motion only.
- `large_terrain_nav_validation.py` - Large-terrain global-planning asset validation. It does not exercise local planner or path follower backends and reports those algorithm surfaces as `not_exercised`.
- The current 2026-06-08 target-host large-terrain report is green only for
  native PCT discrete planning. The gate disables the unstable GPMP optimizer
  with `LINGTU_PCT_OPTIMIZE_TRAJECTORY=0` inside the isolated child process and
  records `pct_optimizer_enabled=false` plus
  `pct_planner_path_mode=native_astar_raw_path`. This is not an A* fallback and
  not a MuJoCo motion-loop pass.
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
  `pct_planner_path_mode=native_astar_raw_path`. The overall DimOS closure
  remains red at 12/13 required gates because `large_loop_closure` still fails.
  The latest large-loop live run ended without writing a child report before
  the launcher fallback was installed; future no-report exits are recorded as
  red `runtime_report_missing_after_launcher` diagnostics instead of allowing
  stale aggregation.
- `full_sim_validation.py` - Compatibility wrapper for the full simulation validation gate; canonical G4 closure aggregation is `server_sim_closure.py`.
- `large_loop_diagnosis_matrix.py` - Large-loop diagnosis matrix.
- `render_slam_validation_screenshots.py` - SLAM validation screenshot renderer.
- `run_slam_dataset_test_v2.py` - SLAM dataset test runner.
- `cmu_unity_tomogram_capture.py` - CMU Unity tomogram capture helper.
- `run_global_planner.py` - Legacy ROS launch compatibility wrapper for `sim/launch/sim.launch.py`. It requires an isolated nonzero `ROS_DOMAIN_ID` and must not be used as the current G4 planner evidence source.

## Demo And Runtime Entrypoints

- `run_sim.py` - Generic simulation launcher.
- `run_person_following.py` - Person-following simulation launcher.
- `run_semantic_full_stack.py` - Semantic full-stack simulation launcher.
- `demo_search.py` - Search demo.
- `policy_nav_smoke.py` - Policy-mode navigation smoke test; simulated motion only.
- `cmu_unity_lingtu_stack.py` - CMU Unity LingTu stack helper for controlled simulation experiments; not the default product runtime.
- `nav_overlay.py` - Navigation overlay visualization.
- `view_scene.py` - Scene viewer.
- `benchmark_following.py` - Person-following benchmark.
- `record_policy_nav_video.py` / `render_gazebo_frontier_video.py` - Video recording/rendering helpers.
- `go1_indoor_nav.py` / `go1_nav_full.py` - legacy Go1 demos; require optional `sim/robots/go1_playground/` assets and are not part of current G4 closure.

## Shell Launchers And Legacy Helpers

- `launch_mujoco_fastlio2_live.sh` - MuJoCo + Fast-LIO2 live simulation launcher for the `sim_mujoco_live` contract. Use ROS2 isolated simulation.
- `launch_cmu_unity_lingtu_runtime.sh` / `launch_cmu_unity_baseline.sh` - CMU Unity runtime and baseline launchers.
- `launch_lingtu_gazebo_industrial_demo.sh` - Gazebo industrial simulation demo launcher.
- `_run_legkilo_test.sh` / `run_legkilo_test.sh` - legacy/manual dataset helper scripts. They may source ROS/install spaces, start SLAM dataset processes, or clean up external processes; do not include them in the G4 server closure.
- `test_*.sh` - integration smoke helpers. Treat as legacy manual unless a gate documents a stricter contract.
- `install_deps.sh` - optional dependency installer.
- `fastlio_speed_scan_plan.sh` - speed scan helper.

## Tooling

- `algorithm_dataflow_summary.py` - Thin CLI wrapper around `core.dimos_runtime_dataflow` for flat live Fast-LIO reports; `dimos_gap_report.py --include-dataflow` and Gateway benchmark diagnostics use the same core parser for aggregate/wrapper/native gate reports.
- `rosbag_slam_bridge_replay.py` - Raw rosbag to `SlamBridgeModule` replay.
