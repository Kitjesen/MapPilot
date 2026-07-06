# Algorithm Validation Flow

Last updated: 2026-05-21.

This is the minimum evidence chain before claiming that the LingTu navigation
algorithm surface is healthy in simulation. It does not claim readiness on any
real hardware target.

## Product Profile vs Strict Benchmark

Do not treat ROS2 topics as the product acceptance API. LingTu's product-facing
inspection acceptance is Gateway + ModulePorts + server-side artifacts. ROS2
topics remain valid evidence for adapter boundaries such as Gazebo, Fast-LIO,
TARE, and external SLAM services, but they are not the only route for runtime
dataflow verification.

## Communication Evidence Boundary

Previous server-linked validation counts as communication and runtime-evidence
testing, not as a hardware navigation pass. The evidence proves that the
product-facing surfaces can be exercised remotely through Gateway, ModulePorts,
saved artifacts, and server-side reports. It does not prove real hardware
motion, and it does not prove that the native LiDAR->C++ Fast-LIO path is
active.

Use these documents as the evidence entry points:

| Evidence | Meaning |
| --- | --- |
| `docs/07-testing/field-runs/` | Dated field and simulation evidence. Current entries must name the active product profile, services, map artifact, planner, and blocker. |
| `artifacts/server_sim_closure/...` | Archived server reports and videos produced by the server closure gates. |
| `python lingtu.py real-runtime-evidence --duration-sec 20 --json-out artifacts/thunder_field_runtime/report.json` | Endpoint communication probe for Gateway/runtime dataflow; read-only collector, no goal or velocity publishing. |
| Gateway `:5050` and MCP `:8090` checks | Control-plane communication evidence only; they do not prove planner or SLAM correctness unless paired with stage evidence. |

The product-facing preset is `inspection_mvp`:

```bash
PYTHONPATH=src:. python sim/scripts/server_sim_closure.py \
  --preset inspection_mvp \
  --required-only \
  --max-report-age-s 86400 \
  --json-out artifacts/server_sim_closure/summary_inspection_mvp_24h.json
```

It requires routecheck preflight, large-terrain map/tomogram, live Fast-LIO
inspection, local dynamic-obstacle planning, and moving-obstacle sweep evidence.
The strict `dimos_benchmark` remains the 10-gate reference suite for full
algorithm closure. A passing `inspection_mvp` is not a substitute for a passing
`dimos_benchmark`; it is a narrower product readiness claim.

## Current Evidence Snapshot

The whole-algorithm simulation claim remains red until the current strict input
contract also passes long-range loop closure and refreshed dynamic-obstacle
speed/density video evidence. The strict live MuJoCo/Fast-LIO input contract is
now `scan_time_profile=physical_rolling`: the simulator accumulates actual
MuJoCo subscans over the MID-360 scan window and reports real per-point time
offsets. The older `synthetic_rolling` and `instantaneous` profiles are
diagnostic ablations, not the preferred acceptance input.

Fresh physical-rolling evidence:

- Fixed forward+yaw Fast-LIO gate:
  `artifacts/server_sim_closure/fixed_forward_yaw_physical_pass/gate-20260521_154007/report.json`.
  It reports `ok=true`, `remaining_gaps=[]`,
  `fastlio2_moved_m=1.0834` versus `sim_moved_m=1.1233`,
  `z_delta_error_m=0.01006`, `yaw_delta_error_rad=0.05614`, and
  `scan_time_model_contract="physical_subscans_with_actual_sim_time_offsets"`.
- One-goal inspection gate with live Fast-LIO, saved large-terrain tomogram,
  PCT, local planner/path follower, and nonzero `/nav/cmd_vel`:
  `artifacts/server_sim_closure/inspection_onegoal_physical_fastlio/inspection-20260521_154351/report.json`.
  It reports `ok=true`, `remaining_gaps=[]`,
  `lingtu_inspection.verified=true`, `patrol_state=SUCCESS`,
  `successful_navigation_goal_count=1`, `global_planner=pct`,
  `global_path_count=1`, `local_path_count=125`, `waypoint_count=5`,
  `nav_cmd_vel_nonzero=true`, `started_after_slam_ready=true`,
  `fastlio2_moved_m=2.0402` versus `sim_moved_m=2.0373`,
  `z_delta_error_m=0.01891`, and `yaw_delta_error_rad=0.0001496`.

Claim boundary: the fresh evidence proves the live Fast-LIO mapping/localization
chain and a one-goal closed-loop inspection under saved tomogram/PCT/local
planning in simulation. It does not prove the whole algorithm is healthy:
the current physical-rolling large-loop and dynamic speed/density gates are
red, and real hardware readiness remains out of scope.

The latest strict `dimos_benchmark` summary is still not closed:

- Current strict benchmark summary:
  `artifacts/server_sim_closure/summary_dimos_benchmark_physical_rolling_current.json`.
  It reports `ok=false`, `missing_or_failed=["large_loop_closure","moving_obstacle_sweep"]`,
  and `algorithm_validation.claim_allowed=false`. Eight required gates are
  green, but the two whole-algorithm stress gates are red.
- Current physical-rolling moving-obstacle speed/density sweep:
  `artifacts/server_sim_closure/moving_obstacle_sweep/report_physical_rolling_20260521.json`.
  All four required bins ran (`slow:sparse`, `slow:dense`, `fast:sparse`,
  `fast:dense`) with `scan_time_profile=physical_rolling`, the
  `physical_subscans_with_actual_sim_time_offsets` contract, video evidence,
  moving-obstacle geometry present, no trail collision, and planner margins
  above 0.73 m. The aggregate still reports `ok=false` and
  `passed_case_count=0`: each child stays at one of three inspection
  checkpoints and shows Fast-LIO motion/Z divergence under the dynamic scene.
  The sweep summary now promotes each child report's Fast-LIO consistency
  checks and `minimal_red_defect`, so this failure class is visible at the
  aggregate level.
- Current same-source physical-rolling large-loop rerun:
  `artifacts/server_sim_closure/large_loop_physical_rolling_same_source/large_loop_closure_report.json`
  over child runtime report
  `artifacts/server_sim_closure/large_loop_physical_rolling_same_source/inspection-loop-video-20260521_161429/report.json`.
  It proves same-source world/tomogram artifacts, `physical_rolling` scan time,
  a written video (`video_frame_count=1801`), PCT global planning
  (`global_path_count=2`), local planning (`local_path_count=1131`), and
  nonzero navigation commands. It still reports `ok=false` after the 900 s wall
  guard, with only one of four checkpoints reached and the loop/path-length
  acceptance checks red. This red class is now reported as
  `planning_tracking` when local paths and cmd_vel are live; true ROS/gate
  exceptions remain `validation_harness`.
- The earlier hardened remote summary has every then-required gate green except
  `large_loop_closure`, but it predates the current physical-rolling dynamic
  sweep acceptance boundary. It is historical evidence only, not a current
  full-benchmark pass.
- Previous aligned large-loop diagnostics remain useful for root cause:
  `artifacts/server_sim_closure/large_loop_closure/report_after_gate_hardening.json`
  over child runtime report
  `artifacts/server_sim_closure/diagnosis_matrix/large_loop_variants/fastlio_rolling_fd_smooth_ang025_20s_aligned/inspection-loop-video-20260521_082041/report.json`.
- Direct blocker: live Fast-LIO Z drift still exceeds the 1.0 m gate after
  time-aligning MuJoCo truth to Fast-LIO odometry header stamps. The latest
  aligned PCT/local-planning run has `time_alignment.time_aligned=true` with
  `first_dt_s=0` and `last_dt_s=0`, but still reports
  `z_delta_error_m=10.3896` and a runtime Z fault. PCT and local planning were
  active in that same report (`global_planner=pct`, `local_path_count=196`,
  `nav_cmd_vel_nonzero=217`, `patrol_state=PATROLLING`). The hardened
  large-loop aggregate reports
  `minimal_red_defect.blocking_subsystem="slam_localization"`.
- The guarded large-loop entry run also shows the same blocker class on a fresh
  route entry: `global_planner=pct`, `global_path_points_max=15`,
  `local_path_count=89`, `local_path_points_max=101`,
  `nav_cmd_vel_nonzero=79`, but `fastlio2_yaw_delta_error_rad=0.8177` and the
  diagnostic segment report reaches `max_z_delta_error_m=1.2677`. The aggregate
  again reports `minimal_red_defect.blocking_subsystem="slam_localization"`.
- New route-level A/B input-model evidence reverses the earlier short-entry
  assumption: the large-loop route with `scan_time_profile=instantaneous`
  fails almost immediately
  (`artifacts/server_sim_closure/large_loop_turnspeed_diag_15k/inspection-loop-20260521_132208/report.json`),
  with `fastlio2_moved_m=144.7474` versus `sim_moved_m=0.1669`,
  `z_delta_error_m=78.9128`, and `yaw_delta_error_rad=0.6444`.
  The same tuned route using `scan_time_profile=synthetic_rolling`
  (`artifacts/server_sim_closure/large_loop_turnspeed_roll_15k/inspection-loop-20260521_132910/report.json`)
  removes that early motion/Z blow-up: `fastlio2_moved_m=1.4478`
  versus `sim_moved_m=0.8890`, and `z_delta_error_m=0.3416`.
  It still fails strict validation on yaw drift (`1.7839 rad`) plus patrol
  timeout, so this is a root-cause narrowing step, not a pass.
- The live gate and launcher now default to
  `scan_time_profile=physical_rolling` for MuJoCo MID-360 validation. Fast-LIO
  still receives per-point scan offsets for deskewing, but those offsets are
  now tied to actual MuJoCo subscan capture times rather than being applied to
  a single snapshot raycast. `synthetic_rolling` and `instantaneous` remain
  explicit ablations.
- The previous default-instantaneous large-loop run was
  `artifacts/server_sim_closure/large_loop_default_instantaneous/inspection-loop-video-20260521_105340/report.json`,
  aggregated as
  `artifacts/server_sim_closure/large_loop_closure/report_default_instantaneous_aggregated.json`.
  This improves the input contract but still fails later as SLAM/localization:
  PCT/local/cmd are active (`local_path_count=240`, `nav_cmd_vel_nonzero=259`),
  then Fast-LIO degeneracy rises to `max_degenerate_dof_count=5`,
  `max_condition_number=1e12`, and odometry diverges (`z_delta_error_m=45.9778`,
  `yaw_delta_error_rad=2.8301`).
- Long live gates now have an explicit wall-time guard. A launcher smoke run at
  `artifacts/server_sim_closure/diagnosis_matrix/wall_timeout_launcher_smoke/gate-20260521_103103/report.json`
  proves that timeout produces `ok=false`,
  `gate_wall_timeout.triggered=true`, and a structured runtime fault instead of
  silently hanging.
- The current speed boundary on the same large-terrain world is explicit:
  `large_terrain_static_20s` passes (`z_delta_error_m=0.0146`),
  `large_terrain_fixed_vx005_20s` passes (`z_delta_error_m=0.0295`),
  `large_terrain_fixed_vx010_20s` fails with motion divergence and
  `z_delta_error_m=1.6507`,
  `large_terrain_fixed_vx015_20s` fails narrowly with
  `z_delta_error_m=1.0180`, and
  `large_terrain_fixed_vx025_20s` fails hard with
  `z_delta_error_m=31.5056` plus yaw/motion divergence.
- The diagnostic-only speed/config boundary report is
  `artifacts/server_sim_closure/diagnosis_matrix/fastlio_speed_boundary/report.json`.
  It reports `boundary_characterized=true`, `algorithm_pass=false`,
  `claim_allowed=false`, `green_speed_mps=0.05`,
  `first_red_speed_mps=0.10`, and red speeds `[0.10, 0.15, 0.25]`.
  This describes the current defect boundary; it does not make the navigation
  algorithm healthy.
  The gate now also preserves `linear_y` and planar speed in each fixed-control
  case, so PCT command-shape controls can separate straight speed, lateral
  velocity, and yaw-rate sensitivity.
- The refined speed/scan/tuning boundary report is
  `artifacts/server_sim_closure/diagnosis_matrix/fastlio_speed_boundary/refined_report.json`.
  It aggregates 21 fixed-control cases, reports
  `boundary_characterized=true`, `claim_allowed=false`, and
  `state="non_monotonic_or_unstable"`. The lowest reproduced red speed is
  0.06 m/s with `blocking_subsystem="slam_localization"`,
  `primary_metric="fastlio2_z_consistency"`, and
  `z_delta_error_m=1.4906` against a 1.0 m limit. Historical controls include
  green runs up to 0.15 m/s, so the correct interpretation is instability /
  non-monotonic Fast-LIO behavior, not a clean speed threshold.
- The yaw-rate boundary is now separated from the straight-line speed boundary
  instead of mixing all fixed controls into one speed axis. The latest report is
  `artifacts/server_sim_closure/diagnosis_matrix/yaw_rate_boundary/summary_vx_wz_report.json`.
  It aggregates four fixed controls and reports `boundary_characterized=true`.
  Straight `vx=0.10,wz=0.0` is a `slam_localization` red item with
  `z_delta_error_m=2.2031`. Turning has three red cases, but only one is a
  SLAM red item: `vx=0.05,wz=0.25` with `z_delta_error_m=1.2398`. The
  `vx=0.0,wz=0.25` spin and `vx=0.10,wz=0.45` turn have Fast-LIO Z/yaw within
  limits and are classified as `validation_runtime` because they hit the
  wall-time guard before reaching the requested sim duration. This distinction
  prevents treating simulator throughput/time-budget failures as SLAM drift.
- The latest time-offset A/B on fixed `vx=0.10,wz=0.0` does not clear the
  blocker. `time_diff_lidar_to_imu=-0.010` is worse, `0.0` remains red,
  `+0.010` improves Z drift, and `+0.020` regresses again. The best current
  time-offset plus Fast-LIO tuning control is
  `artifacts/server_sim_closure/diagnosis_matrix/time_diff_boundary/td_p010_tuned/gate-20260521_113645/report.json`;
  it still reports `ok=false` with `z_delta_error_m=1.7469`. This proves
  timing alignment is influential, but not the whole fix.
- The latest MID-360 point-density diagnostic found a reproducible fixed-drive
  green window at lower sample counts. On the same large-terrain world with
  fixed `vx=0.10,wz=0.0`, `samples_per_frame=12000` and `15000` both reached
  20.02 s simulation time with Fast-LIO Z/yaw/motion checks green when the wall
  guard was widened to 180 s. `18000` failed with `z_delta_error_m=8.8011`, and
  the original 24000-sample baseline remains red. Summary artifact:
  `artifacts/server_sim_closure/diagnosis_matrix/point_density_boundary/summary.json`.
  This is diagnostic-only because reducing simulator LiDAR density changes the
  validation input profile; it narrows the next fix surface to MID-360 pattern
  sampling / Fast-LIO input conditioning before the large-loop acceptance gate
  can be claimed.
- The follow-up 15k-sample PCT/local large-loop diagnostic did not transfer the
  fixed-control improvement into a route-level pass. Runtime artifact:
  `artifacts/server_sim_closure/large_loop_samples_15000_60s/inspection-loop-video-20260521_115818/report.json`;
  aggregate:
  `artifacts/server_sim_closure/large_loop_samples_15000_60s/large_loop_closure_report.json`.
  It used `nav_data_source=fastlio2`, the then-default
  `scan_time_profile=instantaneous`,
  `global_planner=pct`, `local_path_count=207`, and
  `nav_cmd_vel_nonzero=218`, but failed at 20.38 s with
  `z_delta_error_m=594.5294`, `yaw_delta_error_rad=1.1938`, and
  `fastlio2_moved_m=1226.861` versus `sim_moved_m=3.0639`. The diagnostic
  segment report first crosses the Z gate around 18-20 s. This keeps the
  blocker in `slam_localization`, but shifts the next experiment from straight
  fixed drive to PCT command-shape/yaw-rate/lateral-motion sensitivity.
- The first 15k-sample command-shape boundary matrix narrows that PCT-route
  blocker. Summary:
  `artifacts/server_sim_closure/diagnosis_matrix/command_shape_boundary_summary.json`.
  It aggregates six fixed-control cases and reports
  `boundary_characterized=true`. Low-speed yaw/lateral controls are green:
  `vx=0.0,vy=0.08,wz=0.45`, `vx=0.10,vy=0.08,wz=0.45`, and
  `vx=0.0,vy=0.0,wz=0.45` all pass Fast-LIO Z/yaw/motion checks. High-speed
  straight motion is also green at `vx=0.25,vy=0.0,wz=0.0`. The first red
  turning SLAM case is `vx=0.25,vy=0.0,wz=0.45`, with
  `z_delta_error_m=26.0378`, `yaw_delta_error_rad=1.2663`, and motion
  divergence (`fastlio2_moved_m=104.0555` versus `sim_moved_m=0.4910`). The
  paired lateral case `vx=0.25,vy=0.08,wz=0.45` is also red, but only by Z
  drift (`z_delta_error_m=1.2869`) while yaw and motion consistency stay within
  limits. This makes the current minimal reproduction high forward speed plus
  high yaw rate, not lateral velocity alone and not global PCT planning.
- The first tuned command-shape controls show improvement but not closure.
  Summary:
  `artifacts/server_sim_closure/diagnosis_matrix/command_shape_tuned_summary.json`.
  The run uses `samples_per_frame=15000`, `time_diff_lidar_to_imu=+0.010`,
  `lidar_filter_num=2`, `scan_resolution=0.10`, `map_resolution=0.20`,
  `near_search_num=8`, `ieskf_max_iter=8`, and `lidar_cov_inv=500`. The
  lateral high-turn case becomes green:
  `vx=0.25,vy=0.08,wz=0.45`, `z_delta_error_m=0.1027`,
  `yaw_delta_error_rad=0.1629`, motion consistent. The no-lateral high-turn
  case improves from hard motion/yaw/Z divergence to Z-only red:
  `vx=0.25,vy=0.0,wz=0.45`, `z_delta_error_m=2.5252`, yaw and motion green.
  Reducing yaw rate did not produce a clean threshold:
  `vx=0.25,vy=0.0,wz=0.35` is yaw-red (`yaw_delta_error_rad=0.8437`) and
  `vx=0.25,vy=0.0,wz=0.25` is Z-red (`z_delta_error_m=1.1825`). This is still
  `claim_allowed=false` and keeps the blocker in `slam_localization`.
- New Fast-LIO red-item instrumentation is now emitted as
  `fastlio_large_loop_diagnostic_report` in live MuJoCo/Fast-LIO reports. It is
  diagnostic-only and summarizes segment consistency, IMU sample statistics,
  scan timing statistics, and command trajectory statistics without changing
  any pass/fail threshold. Smoke artifact:
  `artifacts/server_sim_closure/diagnosis_matrix/diagnostic_instrumentation_smoke/report.json`
  includes `schema_version="lingtu.fastlio_large_loop_diagnostics.v1"`,
  `diagnostic_only=true`, 200 IMU samples, 16902 scan timing points, and 200
  command samples.
- Fast-LIO tuning controls are now reportable through the live gate. The best
  current 0.25 m/s tuning control (`lidar_filter_num=2`,
  `scan_resolution=0.10`, `map_resolution=0.20`, `near_search_num=8`,
  `ieskf_max_iter=8`, `lidar_cov_inv=500`) improves the failure but remains
  red with `z_delta_error_m=1.1448`. A lower measurement weight control is
  worse (`z_delta_error_m=2.3991`).
- A conservative full-chain PCT control is green but is not a large-loop
  acceptance pass:
  `artifacts/server_sim_closure/diagnosis_matrix/large_loop_variants/fastlio_pct_conservative_v005_20s/inspection-loop-video-20260521_084605/report.json`.
  It uses `nav_data_source=fastlio2`, has PCT global planning and local paths
  active (`global_path_count=1`, `local_path_count=191`), has no runtime
  faults, and reports `z_delta_error_m=0.0439`. It intentionally sets
  `min_required_checkpoints=0` for a 20 s low-speed control, so it does not
  prove loop closure or mission completion.
- The large-loop diagnosis matrix is also red:
  `artifacts/server_sim_closure/diagnosis_matrix/summary.json`. It currently
  reports `blocking_failures=["slam_localization"]`.
  - Baseline live Fast-LIO/PCT run: PCT and local planning are active, but
    Fast-LIO consistency remains the blocker.
  - Time-aligned runtime guard: comparing MuJoCo truth to Fast-LIO odometry
    timestamps removes async latest-vs-latest ambiguity; the aligned guard
    still fails on Z drift.
  - Fixed-motion controls: static and 0.05 m/s are green on the same world,
    while 0.10, 0.15, and 0.25 m/s expose the current motion-speed boundary.
  - Conservative full-chain control: 0.05 m/s PCT/local/cmd/Fast-LIO is green
    for 20 s, but mission completion and loop closure remain unproven.
  - The failing large-loop logs include Fast-LIO degeneracy and IEKF
    non-convergence warnings, so the next corrective work should focus on
    SLAM observability/timing/model fidelity rather than weakening thresholds.

This means the dynamic obstacle gate has historical hardened evidence, but the
current physical-rolling dynamic sweep still needs a fresh video-backed refresh.
The whole algorithm cannot be called healthy until both that sweep and the
large-loop Fast-LIO/PCT/local-planning gate pass in the current strict
benchmark.

## Claim Boundary

The algorithm is considered simulation-healthy only when the strict server
closure summary is green with fresh reports:

```bash
PYTHONPATH=src:. python3 sim/scripts/server_sim_closure.py \
  --preset dimos_benchmark \
  --required-only \
  --run-missing \
  --strict \
  --max-report-age-s 86400 \
  --json-out artifacts/server_sim_closure/summary_dimos_benchmark_24h.json
```

The summary must report:

- `ok=true`
- `missing_or_failed=[]`
- `remaining_gaps=[]`
- `algorithm_validation.claim_allowed=true`
- `algorithm_validation.flow_ok=true`
- every required stage in `algorithm_validation.validation_flow` has
  `status="passed"`
- `simulation_only=true`
- `real_robot_motion=false`
- `cmd_vel_sent_to_hardware=false`

For gates that declare `runtime_contract`, the server closure summary must also
surface `evidence.runtime_evidence` from `runtime.diagnostics.runtime_evidence`; this is the
shared check for simulation-only status, hardware command isolation, path
evidence, command evidence, runtime contract name/health, and any required
`frame_evidence` links such as `map->odom`, `odom->body`, and `body->lidar`.
When a gate requires the canonical runtime data-flow contract, it must also
surface `data_flow_evidence` for each `runtime_data_flow` stage so a failed
summary identifies the broken stage instead of only reporting missing final
topics. Each stage must preserve `inputs`, `outputs`, `owner`, `frame_role`,
and `map_dependency`; otherwise a gate could accidentally prove a topic exists
while hiding the module boundary or live-map/artifact dependency it came from.
For operator review, the expected concrete path comes from
`resolved_runtime_data_flow.<data_source>` in `runtime_contract_manifest()` and
`config/topic_contract.yaml`; the template-level `source:data_source.*` and
`sink:data_source.*` placeholders are not acceptance evidence by themselves.
The algorithm surface for each stage is explicit in
`runtime_data_flow_stage_algorithm_interfaces`, so a review can check that
global planning, local planning, exploration, and SLAM interfaces are bound to
the stage that actually owns their inputs and outputs instead of inferring the
relationship from final topics alone.
Use the manifest export when reviewing the canonical interface itself:

```bash
python lingtu.py runtime-contract \
  --json-out artifacts/runtime_contract_manifest.json
```

Use `python lingtu.py runtime-contract` without `--json` for the operator
summary of frames, frame links, real topic frame rules, real runtime data flow,
data sources, profile-to-data-source bindings, saved-map artifact formats,
adapter aliases/relays, stage interfaces, and algorithm interfaces.

Before collecting future hardware evidence, run the offline contract audit. It
validates the Python manifest, YAML operations contract, runtime
profile specs, ROS frame-contract documentation mirror, validation-gate
self-description, and real report collector coverage without launching ROS or
publishing any control topic:

```bash
python lingtu.py runtime-audit \
  --json-out artifacts/runtime_contract_audit.json
```

Use `python lingtu.py runtime-audit` without `--json` for the operator summary
of checks, validation-gate order, and commands. Use `--json` or `--json-out`
for machine-readable evidence. The audit payload includes
top-level `validation_gate` for the `runtime_audit` acceptance step and
`checks.runtime_validation_gates.acceptance`, including each gate's required
prior gates, proof scope, and operator summary sections. Treat that
machine-readable sequence as the operator order: `runtime_audit` first,
`saved_map_artifact_gate` when saved map/tomogram/occupancy/PCT artifacts are
used, then `real_runtime_evidence` only when an actual endpoint runtime is
under review.

Saved-map navigation also requires the persisted map artifacts to prove their
source chain. Before using a saved map for localization or PCT planning, run the
artifact provenance gate against the map directory:

```bash
python lingtu.py saved-map-artifact-gate <map-dir> \
  --require-tomogram \
  --require-occupancy \
  --json-out artifacts/saved_map_artifacts/report.json
```

Use `python lingtu.py saved-map-artifact-gate <map-dir> ...` without `--json`
for the operator summary of frame, expected source, required artifacts,
metadata health, artifact checksums, and blockers. Use `--json` or
`--json-out` for machine-readable evidence. The JSON payload includes
`validation_gate` for the `saved_map_artifact_gate` acceptance step, prior
gate, proof scope, and operator summary sections. This gate is offline and does
not require ROS. It reads `metadata.json`,
recomputes checksums for `map.pcd`, `tomogram.pickle`, and `occupancy.npz`, and
rejects missing metadata, checksum drift, and derived artifacts whose
`source_map_sha256` no longer matches the current `map.pcd`.

Endpoint runtime reports use the same contract shape. For the current
server/simulation/endpoint target, they are communication evidence only and
must not be called real-hardware evidence. During an endpoint run, collect
and validate one report through the unified CLI entry:

```bash
python lingtu.py real-runtime-evidence \
  --duration-sec 20 \
  --json-out artifacts/thunder_field_runtime/report.json
```

This command wraps the read-only collector and embedded gate. The collector
subscribes to existing ROS 2 topics, samples TF, and inspects `/nav/cmd_vel`
subscribers. It does not publish goals, cmd_vel, or any robot-control topic.
Without `--json`, the CLI prints an operator summary that groups missing
evidence by `Topic frame evidence`, `Frame link evidence`,
`Stage evidence matrix`, and `Data-flow evidence`. The stage matrix is the
first runtime-dataflow view: `slam_or_relayed_localization_map` must prove live
localization and map-cloud output, `global_planning` must prove a nonempty
global path, `local_planning_and_following` must prove local path plus nonzero
`/nav/cmd_vel`, and `command_boundary` must prove the configured command route.
`Data-flow evidence` remains the detailed per-stage inputs/outputs record. A
failed local probe on a non-ROS workstation is useful only as a wiring
diagnostic: errors such as missing `rclpy` prove the collector did not observe
the real ROS 2 runtime, not that the robot pipeline passed.
The saved report and standalone gate payload also include `validation_gate`,
copied from the same runtime validation-gate descriptor, so archived evidence
keeps its required prior gates, proof scope, and operator summary sections.
Gateway clients can read the same contract from
`/api/v1/app/capabilities.validation_gates`; the OpenAPI schema declares each
gate's command, collector/gate commands, ROS/endpoint requirements, validates
list, checks, and coverage map so UI and operations tooling do not need to infer
the validation sequence from ad hoc text.
The gate uses
`runtime.diagnostics.runtime_evidence.validate_real_runtime_evidence` and requires
`runtime_contract.name=thunder_field`, the configured command boundary,
`map->odom->body->lidar_link` frame
evidence, concrete `resolved_runtime_data_flow.thunder_field` stage evidence, and
positive global path, local path, and `/nav/cmd_vel` observations. For endpoint
evidence, `/nav/map_cloud` and `/nav/global_path` must report `frame_id` as
`map`; the looser simulation/replay tolerance for `odom` is not sufficient to
prove the saved-map/global-planning boundary.

`algorithm_validation.claim_boundary` is also part of the claim. It must keep
these roles explicit: global planning source is `static_saved_map_tomogram`,
realtime mapping/localization source is `fastlio2_lidar_imu`, and
`live_costmap_role` is `local_planning_and_safety_only`.

## Runtime Validation Ladder

Use this ladder as the planned promotion path from no-actuation evidence to a
future hardware campaign. Current work stops at server/simulation/endpoint
evidence unless a real hardware target is explicitly scheduled.

1. Replay endpoint, no actuation: prove the product graph consumes recorded
   localization/map data and emits `global_path`, `local_path`, and `cmd_vel`
   without any hardware command sink.
2. MuJoCo raw Fast-LIO: feed raw LiDAR/IMU into Fast-LIO, then run the same
   navigation, local planning, tracking, and command-mux graph against the
   simulator endpoint.
3. Dynamic obstacle sweep: run density and speed cases, reject collisions,
   missing coverage, unstable control, and reports that do not preserve the
   simulation-only command boundary.
4. Future hardware dry safety: confirm startup, SLAM/localization health, map
   source, `CmdVelMux` route, and no automatic actuation.
5. Future hardware controlled run: allow operator-enabled actuation only after
   replay, MuJoCo raw, dynamic-obstacle, and dry-safety evidence is current and
   green.

## Required Chain

| Stage | Evidence gate | What it proves |
| --- | --- | --- |
| Map asset | `large_terrain` | A large-scene map/tomogram exists and PCT/A* route checks are valid on that static asset. |
| Static global planning | `native_pct_mujoco` | Saved tomogram -> PCT global path -> local planner/path follower -> simulated motion, with no hidden fallback. |
| Local dynamic avoidance | `dynamic_obstacle_local_planner` | The local planner replans around changing obstacle phases and recovers after the path clears. |
| Realtime SLAM/nav chain | `fastlio2_dynamic_inspection` | Raw MID-360-style LiDAR plus IMU feed Fast-LIO2, producing `/nav/odometry` and `/nav/map_cloud` used by LingTu navigation on the same large-terrain world/tomogram pair. |
| Dynamic density/speed range | `moving_obstacle_sweep` | The live Fast-LIO/PCT/local-planner/cmd_vel chain survives slow/fast and sparse/dense moving-obstacle bins on the same large-terrain world/tomogram pair; each child case must carry `live_nav_chain.ok=true`. |
| Long-range loop | `large_loop_closure` | A larger same-source route closes the loop with bounded odometry drift and return-to-start error while still using local path planning. |
| Command safety | `routecheck_preflight` | Route preview does not publish motion commands. |
| ROS integration smoke | `gazebo_runtime` | ROS topics, map growth, navigation loop, and publisher identity work in a ROS-native simulator. |
| Saved-map relocalization | `saved_map_relocalize` | Saved map/localizer health is locked on live Fast-LIO-derived input. |
| Saved-map navigation | `pct_saved_map_navigation` | Same-source saved map/tomogram -> PCT -> local planning/path following -> motion. |

## Minimal Whole-Algorithm Acceptance Matrix

This is the smallest matrix that can support a claim that the simulation
algorithm surface is healthy. A diagnostic gate can explain a blocker, but it
cannot override a failed acceptance gate.

| Capability | Acceptance evidence | Current status |
| --- | --- | --- |
| Realtime mapping/localization | `fastlio2_dynamic_inspection` and `large_loop_closure` both use live MID-360-style LiDAR/IMU through Fast-LIO. | Partial: physical-rolling fixed motion and one-goal inspection are green, but the current same-source physical-rolling large-loop is red. |
| Static map/tomogram planning | `large_terrain` and `native_pct_mujoco` prove saved tomogram -> PCT/native path -> simulated motion. | Evidence-backed in current summaries, subject to freshness. |
| Local planning and dynamic avoidance | `dynamic_obstacle_local_planner` plus `moving_obstacle_sweep` prove local replanning and live Fast-LIO -> PCT -> local path -> cmd_vel across slow/fast and sparse/dense bins. | Red for the full claim: the fresh physical-rolling sweep has videos and obstacle clearance evidence, but all four bins fail because the live Fast-LIO/nav chain does not complete the inspection checkpoints. |
| Long-range mission stability | `large_loop_closure` proves same-source large route, local paths, nonzero cmd_vel, bounded drift, and video. | Red for the full claim: the fresh same-source physical-rolling rerun has PCT/local/cmd/video evidence, but times out with only one of four checkpoints reached. |
| Saved-map lifecycle | `saved_map_relocalize` and `pct_saved_map_navigation` prove same-source map use, relocalization health, PCT, and motion. | Evidence-backed in current summaries, subject to freshness. |
| Command safety | `routecheck_preflight` proves route preview has no hardware motion output. | Evidence-backed. |
| Diagnostic boundary | `fastlio_speed_boundary_gate` and yaw-rate controls characterize motion conditions that reproduce Fast-LIO or validation-runtime red items. | Diagnostic-only: `claim_allowed=false`; useful for root cause, not acceptance. |

## Non-Negotiable Interpretation

- Fast-LIO live LiDAR/IMU is the realtime mapping/localization source during
  normal navigation validation.
- Strict live simulation validation uses `scan_time_profile=physical_rolling`.
  `synthetic_rolling` and `instantaneous` are ablations used to diagnose input
  model sensitivity.
- PCT plans globally on static saved map/tomogram artifacts.
- Live costmap and dynamic obstacles are local-planning and safety inputs. They
  are not the global planning source.
- Live world, saved map, and tomogram must be same-source for PCT/Fast-LIO
  validation. A mixed live world/tomogram report is rejected even if short
  motion metrics look healthy.
- Same-source is not only a directory check. Large-loop reports must also
  prove the MuJoCo start pose, tomogram/map frame, and inspection goals are in
  the same frame through `metadata.json` fields such as `map_frame` and
  `map_frame_origin_world_xy`.
- A single short patrol is not enough for algorithm health; it must be paired
  with a dynamic obstacle sweep and a large loop closure gate.
- A dynamic obstacle sweep child report is rejected if it only proves moving
  obstacle publication and video counters. It must also prove realtime
  `nav_data_source=fastlio2`, `/points_raw` + `/imu_raw` -> Fast-LIO ->
  `/nav/odometry` + `/nav/map_cloud`, PCT inspection, local path output,
  nonzero nav cmd_vel, and no planner fallback.
- Passing these gates is still simulation/replay evidence, not physical robot
  field evidence.

## Failure Triage

If the strict summary is red, do not weaken the claim. Classify the blocker:

| Failure | First interpretation |
| --- | --- |
| Missing report | The validation has not run. Run the printed `missing_required_commands`. |
| Stale report | The evidence is too old for a current claim. Rerun the gate. |
| `gate_exception`, ROS publisher context invalid, or `gate_wall_timeout.triggered=true` | Validation harness/runtime defect or timeout boundary. Keep the algorithm claim red, preserve the child report, and rerun with the wall-time guard instead of killing the process externally. |
| Fast-LIO out-of-order messages or cross-run contamination | Validation infrastructure isolation defect; rerun with isolated ROS domains before judging the algorithm. |
| Live world and tomogram are from different artifact roots | Validation input defect; rerun with `LINGTU_MUJOCO_LIVE_WORLD` and `LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM` pointing to the same generated map directory. |
| Fast-LIO drift or Z consistency failure after isolation | SLAM/localization defect or simulator timing defect; inspect raw sensor timestamps and Fast-LIO diagnostics. |
| Fixed-control Fast-LIO passes only after changing MID-360 sample density | Diagnostic input-model boundary, not acceptance. Carry the candidate profile into a large-loop diagnostic, then decide whether the simulator validation profile or Fast-LIO conditioning must change. |
| Fixed-control Fast-LIO fails only when high forward speed combines with high yaw rate | SLAM/localization observability or IMU/LiDAR motion-model boundary. Tune or limit the turning command shape and rerun the same large-loop acceptance gate; do not mark a capped diagnostic as acceptance unless the cap is a product requirement. |
| PCT fallback or unsafe path | Global planning/path-safety defect; do not count the run as a PCT pass. |
| Local path missing while global path exists | Local planning/path follower integration defect. |
| Dynamic obstacle published but trail collision/margin fails | Local avoidance defect. |
| Video missing or undecodable | Visual evidence is invalid even if metrics exist. |

When `large_loop_closure` is red, run the control matrix before changing
thresholds:

```bash
PYTHONPATH=src:. python3 sim/scripts/large_loop_diagnosis_matrix.py \
  --baseline-fastlio artifacts/server_sim_closure/mujoco_fastlio2_live_after_diag/inspection-loop-video-20260521_064739/report.json \
  --truth-nav artifacts/server_sim_closure/diagnosis_matrix/truth_nav_report.json \
  --fixed-fastlio artifacts/server_sim_closure/diagnosis_matrix/fixed_fastlio_report.json \
  --json-out artifacts/server_sim_closure/diagnosis_matrix/summary.json
```

The matrix must be clean or every `blocking_failures[]` entry must be fixed and
rerun. A green dynamic obstacle sweep cannot override a red large-loop diagnosis.

## Stop Condition

Stop only when the current strict summary is green, or when the red summary has
a concrete blocker list that is reproduced and assigned to validation
infrastructure, SLAM/localization, global planning, local planning, or artifact
freshness.
