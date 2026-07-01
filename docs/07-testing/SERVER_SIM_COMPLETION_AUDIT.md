# Server Simulation Completion Audit

Last audited: 2026-05-15.

Superseded status, 2026-06-08: this document preserves the 2026-05 completion
audit but no longer describes the current DimOS readiness state. Current
authoritative artifacts are
`artifacts/server_sim_closure/summary_after_native_pct_goal_fix.json` and
`artifacts/server_sim_closure/dimos_gap_after_native_pct_goal_fix.json`; they
report `lingtu_readiness.ok=false`, `claim_allowed=false`, 7/13 required gates
passed, and 6 failed or missing. The current host preflight is green at
`artifacts/server_sim_closure/host_preflight_after_mid360_sync.json`, and
`native_pct_mujoco` is green at
`artifacts/server_sim_closure/native_pct_mujoco/report_after_goal_reached_fix.json`.
Remaining red or missing gates are `fastlio2_dynamic_inspection`,
`moving_obstacle_sweep`, `large_loop_closure`, `gazebo_runtime`,
`saved_map_relocalize`, and `pct_saved_map_navigation`.

This audit converts the active objective, "complete the testing push, verify
everything, start with the core algorithms", into concrete server-side
deliverables. It is intentionally stricter than a green test run: every claim
below must map to a runtime artifact, a closure evaluator, and a regression
test or an explicit boundary.

## Historical Result

Server closure summary:

`artifacts/server_sim_closure/summary_freshness_24h.json`

Core algorithm recheck summary:

`artifacts/server_sim_closure/summary_core_algorithms_recheck.json`

Historical server verification after the 2026-05 TARE/PCT repair and
core-algorithm recheck:

- Historical 24-hour required closure:
  `artifacts/server_sim_closure/summary_freshness_24h.json` reports
  `ok=true`, `13` required gates passed, `remaining_gaps=[]`,
  `simulation_only=true`, `real_robot_motion=false`, and
  `cmd_vel_sent_to_hardware=false` under the older gate set. This is not the
  current DimOS readiness result.
- Server regression after the CMU runtime freshness and MuJoCo LiDAR fallback
  fixes: `src/core/tests/test_server_sim_closure.py`,
  `src/core/tests/test_sim_runtime_compat.py`, and
  `src/core/tests/test_mujoco_mid360_pattern.py`: `88 passed`.
- CMU Unity runtime freshness is now satisfied by the stricter current report
  `artifacts/server_sim_closure/cmu_unity_pct_strict/report_unique_waypoints.json`.
  That report is evaluated by both `cmu_unity_runtime` and
  `cmu_unity_pct_strict`; it records `/nav/way_point` unique count `17`,
  `/nav/cmd_vel` non-zero samples `2779`, odom moved `2.4868 m`,
  registered scan area growth `59.6875 m2`, `/nav/terrain_map_ext` growth
  `39.3125 m2`, planner `pct`, `fallback_used=false`, and
  `reached_goal=true`.
- MuJoCo MID-360 fallback LiDAR now uses the current MuJoCo `mj_multiRay`
  signature and offsets the ray origin outside the robot collision shell. The
  regression `test_mujoco_driver_default_robot_emits_lidar_points` verifies
  the default MuJoCo robot emits non-empty LiDAR points.
- Broad core/gate pytest set covering PCT runtime, planner dispatch, plan
  safety, corridor navigation, algorithm closure, map occupancy, localization,
  TARE supervisor, CMU Unity gates, Gazebo contracts, saved-map relocalize,
  server closure, and dynamic-obstacle local planner: `339 passed`.
- Supplemental native/PCT/path-follower/MID-360 set:
  `src/core/tests/test_native_build_contracts.py`,
  `src/core/tests/test_mujoco_mid360_pattern.py`,
  `sim/planning/test_path_follower_logic.py`, and
  `sim/planning/test_pct_adapter_logic.py`: `58 passed`.
- Core map/localization/SLAM/relocalize subset:
  `src/core/tests/test_algorithm_closure.py`,
  `src/core/tests/test_navigation_frame_contract.py`,
  `src/core/tests/test_map_occupancy.py`,
  `src/core/tests/test_localization_health.py`,
  `src/core/tests/test_slam_bridge_tf.py`,
  `src/core/tests/test_slam_stack_services.py`,
  `src/core/tests/test_saved_map_relocalize_contract_gate.py`, and
  `src/core/tests/test_gateway_session_map_contract.py`: `184 passed`.
- Fresh CMU Unity TARE/PCT strict runtime with unique waypoint guard:
  `artifacts/server_sim_closure/cmu_unity_pct_strict/report_unique_waypoints.json`
  reports `ok=true`, `/nav/way_point` unique count `17`, `/nav/cmd_vel`
  non-zero samples `2779`, odom moved `2.4868 m`, `/nav/registered_cloud`
  grew `59.6875 m2`, `/nav/terrain_map_ext` grew `39.3125 m2`, planner
  selected `pct`, `fallback_used=false`, and `reached_goal=true`.
- Historical `server_sim_closure.py --required-only --strict`: `ok=true`.
- Historical required gates: `13`.
- Historical remaining gaps in that required simulation closure: `[]`.
- Current 2026-06-08 DimOS result:
  `artifacts/server_sim_closure/summary_after_native_pct_goal_fix.json` and
  `artifacts/server_sim_closure/dimos_gap_after_native_pct_goal_fix.json`
  report `lingtu_readiness.ok=false`, `claim_allowed=false`, 7/13 required
  gates passed, and 6 failed or missing.
- All required gates report `simulation_only=true`, `real_robot_motion=false`,
  and `cmd_vel_sent_to_hardware=false`.
- Core Python regression set:
  `src/core/tests/test_native_build_contracts.py`,
  `test_planner_backends.py`, `test_pct_planner_package_manifest.py`,
  `test_native_pct_mujoco_gate.py`, `test_mujoco_mid360_pattern.py`,
  `test_dynamic_obstacle_local_planner_gate.py`,
  `test_terrain_local_planner_contract.py`,
  `sim/planning/test_path_follower_logic.py`, and
  `sim/planning/test_pct_adapter_logic.py`: `97 passed`.
- `src/nav/core` C++ standalone tests:
  `2331/2331` passed with the default CMake configuration. The generated
  `CMakeCache.txt` records `NAV_CORE_BUILD_PYTHON_BINDINGS:BOOL=OFF`.
- Fresh core runtime gates:
  large terrain PCT/A*, dynamic-obstacle local planner, routecheck preflight,
  and native PCT MuJoCo all passed under
  `artifacts/core_algorithm_recheck/`.

## 2026-05-15 Core Algorithm Recheck

This recheck was run on the server after the user asked to focus on the core
algorithms. It is separated from the closure summary because several closure
items intentionally reuse latest runtime reports, while the rows below are
fresh reruns from this pass.

| Surface | Fresh evidence | Result |
| --- | --- | --- |
| Native C++ local planning core | `artifacts/core_algorithm_recheck/nav_core_default_ctest_LastTest.log` | `2331/2331` CTest cases passed with default CMake. The previously failing NaN/Inf validation cases passed after removing global unsafe math optimization. |
| Python planner/contracts | Server pytest command covering native build contracts, planner backend dispatch, PCT package manifest, native PCT MuJoCo evaluator, MID-360 pattern, dynamic local planner evaluator, terrain local planner contract, path follower logic, and PCT adapter logic | `97 passed`. |
| Large terrain global planning | `artifacts/core_algorithm_recheck/large_terrain/report.json` | Four routes passed with PCT primary and selected, `fallback_used=false`, `path_safety.ok=true`, and `blocked_sample_count=0`. Routes: `terrain_short`, `terrain_long`, `terrain_narrow_gap`, `terrain_slope_bypass`. |
| Dynamic-obstacle local planner | `artifacts/core_algorithm_recheck/dynamic_obstacle_local_planner/report.json` | `ok=true`; backend `nanobind`; dynamic replan, obstacle response, and clear-path recovery all verified; minimum clearance `0.4223 m`. |
| Routecheck preflight | `artifacts/core_algorithm_recheck/routecheck/summary.json` | `outcome=pass`; baseline and candidate selected `pct`; `path_safety_ok=true`; published `cmd_vel=0`, `goal_pose=0`, `stop_cmd=0`. |
| PCT to MuJoCo closed loop | `artifacts/core_algorithm_recheck/native_pct_mujoco/report.json` | `ok=true`; planner `pct`; `fallback_used=false`; native PCT runtime used; route `terrain_long`; robot moved `21.3675 m`; final distance `0.4999 m`; goal reached. |
| MID-360 pattern in MuJoCo gates | Same native PCT MuJoCo report plus `sim/assets/livox/mid360.npy` | Forced pattern path `sim/assets/livox/mid360.npy`, hash `448821576a658673e8f7929992c8c0d687eb052657d7b584d038729a83da1bfb`, samples per frame `24000`. |

Defect found and fixed during the recheck:

- `src/nav/core/CMakeLists.txt` previously applied a global unsafe math
  optimization. On the server this caused six validation tests around NaN/Inf
  rejection to fail. The option was removed because safety validation must
  preserve `std::isfinite` semantics.
- `src/nav/core/CMakeLists.txt` now defaults
  `NAV_CORE_BUILD_PYTHON_BINDINGS=OFF` so standalone C++ algorithm tests do not
  depend on Python/nanobind headers. The dedicated Python extension path remains
  `scripts/build/build_nav_core.sh`.
- `scripts/build/build_nav_core.sh` was converted to ASCII-only output to avoid
  mojibake in server terminals.

## Prompt-To-Artifact Checklist

| Requirement | Runtime evidence | Closure/test evidence | Status |
| --- | --- | --- | --- |
| Gazebo frontier exploration uses LiDAR-derived occupancy, not fake coverage | `artifacts/server_sim_closure/gazebo_runtime_explore/report_grid_astar_odomfoot.json` | `_eval_gazebo_runtime`; negative tests for missing nav loop, fake publishers, missing frontier, missing TARE contract | Verified for Gazebo wavefront frontier |
| Gazebo robot actually moves and maps grow | Same Gazebo report: frontier map grew `9.39 m2`, odom moved `1.9403 m`, clearance min `0.3206 m` | `test_server_sim_closure_rejects_gazebo_without_navigation_loop` and related tests | Verified |
| CMU Unity adapter is connected to LingTu without hardware output | `artifacts/server_sim_closure/cmu_unity_pct_strict/report_unique_waypoints.json` | `_eval_cmu_unity_runtime`; weak no-motion test; strict-report-as-runtime tests | Verified for runtime adapter through the stricter current report |
| CMU Unity TARE/PCT waypoints drive LingTu command/mapping loop | Same strict CMU report: `/nav/way_point` unique count `17`, `/nav/cmd_vel` non-zero samples `2779`, odom moved `2.4868 m`, registered scan grew `59.6875 m2` | Runtime closure evaluator | Verified |
| CMU Unity same-source PCT uses real CMU map-derived tomogram and does not fall back to A* | `artifacts/server_sim_closure/cmu_unity_tare_pct_controlled_start/report.json`: controlled-start TARE waypoint, `/nav/registered_cloud` `9` samples, `/nav/terrain_map_ext` grew `88.75 m2`, odom moved `3.5489 m` | `_eval_cmu_unity_pct_strict`; tests reject planner fallback and direct-goal bypass; command forces `LINGTU_CMU_AUTO_SESSION=0` and gate-owned `/exploration/start` | Verified for current official tomogram run |
| PCT global planning runs into ROS2 localPlanner/pathFollower and MuJoCo motion | `artifacts/server_sim_closure/native_pct_mujoco/report.json` | `_eval_native_pct_mujoco`; tests reject non-PCT, fallback, missing local-path/obstacle evidence, bad MID-360 pattern | Verified for native MuJoCo route |
| Large-terrain PCT route set has safe paths | `artifacts/server_sim_closure/large_terrain/report.json` | `_eval_large_terrain`; closure complete-set test | Verified |
| Dynamic obstacle local planner responds to changing obstacles | `artifacts/server_sim_closure/dynamic_obstacle_local_planner/report.json` | `_eval_dynamic_obstacle_local_planner`; weak report test | Verified for fixed phase matrix |
| Fast-LIO2 live consumes MuJoCo LiDAR/IMU and bridges output | `artifacts/server_sim_closure/fastlio2_live/report.json` | `_eval_fastlio2_live`; new negative test rejects missing SLAM/bridge/MID-360 evidence | Verified for MuJoCo live gate |
| MID-360 pattern is enforced where MuJoCo LiDAR gates are used | `sim/assets/livox/mid360.npy`, hash `448821576a658673e8f7929992c8c0d687eb052657d7b584d038729a83da1bfb` | `_require_mid360_pattern`; tests reject bad hash/path and missing pattern metadata | Verified for MuJoCo gates |
| Saved-map relocalize contract is present for nav mode | `artifacts/server_sim_closure/saved_map_relocalize/report.json` | `_eval_saved_map_relocalize`; weak report test | Verified as contract/status |
| Routecheck planning preflight is non-motion | `artifacts/server_sim_closure/routecheck/summary.json` | `_eval_routecheck_preflight`; actual gate test and weak counters tests | Verified as non-motion preflight |
| Gateway dry run does not publish hardware commands | `artifacts/server_sim_closure/gateway_dry_run/report.json` | `_eval_gateway_dry_run`; gateway dry-run test | Verified |
| ONNX policy navigation smoke has path/follower/mux/waypoint evidence and no direct fallback | `artifacts/server_sim_closure/policy_nav/report.json` | `_eval_policy_nav`; tests reject direct fallback and missing policy/path/follower/mux/waypoints | Verified for policy smoke |

## Boundaries

Verified claims are simulation claims only. Do not expand them into field
readiness claims.

Do not claim yet:

- TARE runtime planning inside Gazebo. Gazebo currently verifies the TARE source
  contract; runtime TARE evidence comes from CMU Unity.
- PCT inside the Gazebo frontier gate. Gazebo frontier currently uses grid/A*;
  PCT runtime evidence is CMU Unity strict PCT and native MuJoCo PCT.
- Real robot SLAM quality. Fast-LIO2 live is a MuJoCo LiDAR/IMU runtime gate,
  not a field mapping or saved-map relocalization accuracy test.
- Pure PCT safety for every CMU Unity waypoint/environment. The strict PCT
  report proves one controlled-start TARE waypoint on the current official
  tomogram, not every seed or goal.
- Physical gait or hardware motion. All closure evidence is simulation-only.

## Next Expansion Targets

- Add multi-goal CMU Unity PCT strict seed coverage.
- Add Gazebo MID-360 pattern parity, or keep documenting that Gazebo LiDAR is a
  separate runtime model.
- Add a true saved-map relocalization runtime replay, not only bridge contract.
- Add TARE runtime inside Gazebo only if Gazebo becomes a TARE delivery profile.
