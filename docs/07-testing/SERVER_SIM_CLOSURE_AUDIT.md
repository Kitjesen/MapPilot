# Server Simulation Closure Audit

Last audited: 2026-05-15.

Superseded status, 2026-06-08: this audit is historical and must not be used
as the current DimOS readiness claim. The current DimOS benchmark evidence is
`artifacts/server_sim_closure/summary_after_native_pct_goal_fix.json` plus
`artifacts/server_sim_closure/dimos_gap_after_native_pct_goal_fix.json`, which
report `lingtu_readiness.ok=false`, `claim_allowed=false`, 7 of 13 required
gates passed, and 6 failed or missing. The current passing target-host
preflight is
`artifacts/server_sim_closure/host_preflight_after_mid360_sync.json`, and
`native_pct_mujoco` now passes at
`artifacts/server_sim_closure/native_pct_mujoco/report_after_goal_reached_fix.json`.

This audit records the product delivery boundary for the current simulation
work. It separates what is verified from what remains a claim gap.

## Historical Decision

The 2026-05 server-side simulation closure previously passed for the then
required product gates:

- Gazebo wavefront frontier exploration from a Gazebo LiDAR-derived occupancy
  grid.
- Gazebo navigation loop with non-zero command and odometry movement.
- Gazebo map growth, cumulative cloud growth, static obstacle stability, and
  frame/topic synchronization.
- CMU Unity runtime adapter with TARE waypoint, navigation command output,
  odometry movement, and map-area growth.
- CMU Unity same-source official tomogram PCT gate with TARE waypoint,
  LingTu PCT selected, no planner fallback, no direct-goal fallback, command
  output, odometry movement, and map-area growth.
- Native PCT no-fallback route execution through the in-process local
  planner/path follower in MuJoCo.
- Fast-LIO2 live LiDAR/IMU bridge contract.
- Saved-map relocalization/localizer contract.
- Routecheck non-motion planning preflight.
- Dynamic obstacle local planner response.
- ONNX gait policy navigation smoke.

The historical 2026-05 closure summary is:

`artifacts/server_sim_closure/summary_freshness_24h.json`

It reported `ok=true`, `13` required gates passed, `remaining_gaps=[]`,
`simulation_only=true`, `real_robot_motion=false`, and
`cmd_vel_sent_to_hardware=false` under the older gate set. That artifact is
superseded by the 2026-06-08 DimOS 13-gate result: 7/13 passed,
`lingtu_readiness.ok=false`, and `claim_allowed=false`. The historical server
regression set
`src/runtime/tests/test_server_sim_closure.py`,
`src/runtime/tests/test_sim_runtime_adapters.py`, and
`src/runtime/tests/test_mujoco_mid360_pattern.py` reports `88 passed`.

## Historical Required Gate Evidence

| Gate | Evidence | Status | Key result |
| --- | --- | --- | --- |
| Gazebo runtime | `artifacts/server_sim_closure/gazebo_runtime_explore/report_grid_astar_odomfoot.json` | Passed | Frontier map grew `9.39 m2`; odom moved `1.9403 m`; obstacle clearance min `0.3206 m`; cloud/odom skew `0.0 ms`. |
| CMU Unity runtime | `artifacts/server_sim_closure/cmu_unity_pct_strict/report_unique_waypoints.json` | Passed | Runtime freshness is satisfied by the stricter PCT/TARE report; `/nav/way_point` had `17` unique waypoints; `/nav/cmd_vel` had `2779` non-zero samples; odom moved `2.4868 m`; registered scan area grew `59.6875 m2`. |
| CMU Unity same-source PCT | `artifacts/server_sim_closure/cmu_unity_tare_pct_controlled_start/report.json` | Passed | Controlled-start TARE waypoint selected PCT with `fallback_used=false` and `direct_goal_fallback.used=false`; odom moved `3.5489 m`; `/nav/registered_cloud` had `9` samples; `/nav/terrain_map_ext` grew `88.75 m2`. |
| CMU Unity same-source PCT with unique waypoint guard | `artifacts/server_sim_closure/cmu_unity_pct_strict/report_unique_waypoints.json` | Passed | TARE/PCT strict runtime selected PCT with `fallback_used=false`; `/nav/way_point` had `17` unique waypoints; `/nav/cmd_vel` had `2779` non-zero samples; odom moved `2.4868 m`; `/nav/terrain_map_ext` grew `39.3125 m2`. |
| CMU Unity preflight | `artifacts/server_sim_closure/cmu_unity_sim/report.json` | Passed | Humble checkout/build/assets/topic/remap/adapter contracts all passed. |
| Native PCT MuJoCo | `artifacts/server_sim_closure/native_pct_mujoco/report.json` | Passed | Source tomogram -> native PCT -> localPlanner -> pathFollower -> MuJoCo motion; fallback is forbidden; reached goal; moved `21.3684 m`; final distance `0.4998 m`; min clearance `0.5968 m`. |
| Large terrain planning | `artifacts/server_sim_closure/large_terrain/report.json` | Passed | Four routes validated with PCT primary and safe paths. |
| Multi-floor exploration | `artifacts/server_sim_closure/multifloor_exploration/report.json` | Passed | Four route cases, frontier loop enabled, production local planner verified. |
| Fast-LIO2 live | `artifacts/server_sim_closure/fastlio2_live/report.json` | Passed | Live MuJoCo LiDAR/IMU into Fast-LIO2 and SlamBridge output verified. |
| Saved-map relocalize | `artifacts/server_sim_closure/saved_map_relocalize/report.json` | Passed | `nav` uses localizer; relocalize services and localizer health contract verified. |
| Routecheck preflight | `artifacts/server_sim_closure/routecheck/summary.json` | Passed | Baseline and candidate PCT route previews feasible without publishing motion commands. |
| Dynamic obstacle local planner | `artifacts/server_sim_closure/dynamic_obstacle_local_planner/report.json` | Passed | Nanobind local planner replanned around changing obstacles; min clearance `0.4223 m`. |
| Policy navigation | `artifacts/server_sim_closure/policy_nav/report.json` | Passed | Policy loaded, local path/path follower/mux/waypoints observed, nav state `SUCCESS`. |
| Gateway dry run | `artifacts/server_sim_closure/gateway_dry_run/report.json` | Passed | Goal preview path published without driver or hardware command output. |

## Product Claim Boundary

Claim as verified:

- We can run a server-side ROS2/Gazebo exploration gate where wavefront frontier
  decisions come from a LiDAR-derived occupancy map, not a fake coverage grid.
- The Gazebo run verifies LiDAR-derived occupancy, frontier goals, grid/A*
  global path, local path, non-zero `/nav/cmd_vel`, non-zero odometry movement,
  map growth, obstacle clearance, room-bound trajectory quality, and
  synchronized frames.
- The CMU Unity line is integrated as a benchmark/runtime adapter for TARE-style
  exploration; it has runtime evidence for real waypoint, command, odometry, and
  map-area growth.
- The CMU Unity line now also has a same-source official tomogram PCT proof:
  `/nav/terrain_map_ext` was captured to PCD/tomogram, then the wrapper ran
  with `LINGTU_CMU_ALLOW_DIRECT_GOAL_FALLBACK=0`. The strict runtime gate uses
  controlled TARE start (`LINGTU_CMU_AUTO_SESSION=0`,
  `LINGTU_CMU_EXPLORATION_AUTO_START=0`, gate-owned `/exploration/start`) so
  it captures the full TARE -> LingTu PCT -> path follower -> Unity motion
  sequence. The latest report selected PCT, reached the goal, and did not use
  A* fallback.
- PCT is verified in a separate no-fallback chain: large-terrain tomogram source
  report selects native PCT, `fallback_used=false`, `path_safety.ok=true`,
  then the same PCT route drives ROS2 `localPlanner`, `pathFollower`,
  `/cmd_vel`, and MuJoCo odometry to the goal. The current source tomogram hash
  is `bd5dbec747f3c47859ff98b3797f0c7af77a1ac328a23a956a83686fff579f97`.
- All verified gates are simulation-only and do not send commands to real
  hardware.

Do not claim yet:

- TARE runtime planning inside Gazebo. Gazebo currently verifies the TARE source,
  launch, bridge, and supervisor contract; the runtime exploration proof is from
  the CMU Unity adapter gate.
- PCT inside the Gazebo frontier gate. Gazebo frontier currently validates
  LiDAR occupancy plus grid/A* path publication; PCT is validated by the native
  PCT MuJoCo no-fallback gate.
- Field SLAM quality from Gazebo. Gazebo validates LiDAR-derived occupancy and
  accumulated diagnostic clouds; real field mapping remains Fast-LIO2/Super-LIO
  plus saved-map validation on hardware.
- Pure PCT-only safety for every possible CMU Unity TARE waypoint or environment
  variant. The latest same-source PCT gate proves one current official tomogram
  run with PCT selected and no fallback; broader route coverage still needs
  more goals and scene seeds.

## Verification Commands

Targeted unit/evaluator verification:

```bash
python -m pytest src/runtime/tests/test_server_sim_closure.py -q
```

Native PCT no-fallback runtime verification:

```bash
PYTHONPATH=src:. python3 sim/scripts/mujoco/native_pct_gate.py \
  --source-report artifacts/server_sim_closure/large_terrain/report.json \
  --route terrain_long \
  --planner pct \
  --timeout-s 180 \
  --json-out artifacts/server_sim_closure/native_pct_mujoco/report.json
```

Closure summary verification:

```bash
python sim/scripts/server_sim_closure.py \
  --required multifloor_exploration,large_terrain,native_pct_mujoco,dynamic_obstacle_local_planner,fastlio2_live,policy_nav,gateway_dry_run,routecheck_preflight,gazebo_runtime,cmu_unity_sim,cmu_unity_runtime,cmu_unity_pct_strict,saved_map_relocalize \
  --required-only \
  --max-report-age-s 86400 \
  --json-out artifacts/server_sim_closure/summary_freshness_24h.json \
  --strict
```
