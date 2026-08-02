# DimOS Benchmark Gap Matrix

Status: research comparison; not current LingTu acceptance evidence

Last updated: 2026-06-08.

This matrix converts the DimOS-style validation pattern into LingTu gates. It is
not a product claim that LingTu matches DimOS feature-for-feature. It is a
repeatable acceptance surface for the parts of DimOS that are relevant to this
project: replay/simulation first, fixed waypoint missions, frontier progress,
dynamic obstacle response, saved-map lifecycle, and explicit command safety.

## Reference

- Official repository: <https://github.com/dimensionalOS/dimos>
- Native navigation documentation:
  <https://github.com/dimensionalOS/dimos/blob/main/docs/capabilities/navigation/native/index.md>
- Nav stack documentation:
  <https://github.com/dimensionalOS/dimos/blob/main/docs/capabilities/navigation/nav_stack.md>
- Testing documentation:
  <https://github.com/dimensionalOS/dimos/blob/main/docs/development/testing.md>
- CLI/replay/simulation usage:
  <https://github.com/dimensionalOS/dimos/blob/main/docs/usage/cli.md>

DimOS is useful here mainly as a validation model, not as an algorithm to copy.
Its documented Go2 native stack uses replay and simulation entry points,
voxel/cost mapping, continual replanning, frontier exploration, and visible
CLI/MCP operations. LingTu's stack is different: ROS 2 Humble, Livox
MID-360/Fast-LIO2, PCT, local planner/path follower, saved map/tomogram
lifecycle, and S100P deployment constraints.

The public DimOS repository does not expose a navigation leaderboard that can be
copied one-for-one. The useful pattern is its layered evidence surface:
fast pytest checks, heavier self-hosted/ROS checks, MuJoCo replay/simulation
entry points, rosbag-style output deviation tests, and e2e replanning tests
such as blocking one route and requiring replanning through another route.
LingTu should therefore keep strict JSON gates with concrete artifacts instead
of relying on a single demo video.

Concrete upstream anchors used for this matrix:

| DimOS validation surface | Upstream artifact | LingTu counterpart |
| --- | --- | --- |
| No-hardware replay | `dimos --replay run unitree-go2` in the README; described as quadruped navigation replay with SLAM, costmap, and A* planning. | Saved-map relocalization plus `pct_saved_map_navigation`; add real-log replay before field claims. |
| Simulation run | `dimos --simulation run unitree-go2` and `unitree-g1-sim`. | MuJoCo and Gazebo runtime gates, both marked simulation-only. |
| Modular nav stack | `create_nav_stack()` composes terrain analysis, local planner, path follower, PGO, FAR/Simple planner, and optional TARE. | Fast-LIO/PCT/local planner/path follower gates, with TARE and wavefront exploration kept as separate surfaces. |
| Rosbag/deviation tests | `test_local_planner_rosbag.py`, `test_path_follower_rosbag.py`, `test_far_planner_rosbag.py`, and `test_pgo_rosbag.py`. | `navigation_replay_deviation` now covers local path, cmd velocity ratio, odometry motion, endpoint error, and path tracking drift for recorded traces; PGO/loop correction remains covered by live loop gates. |
| Dynamic replanning e2e | `test_dimsim_path_replaning.py` adds a wall when the robot approaches a door and requires replanning to the target. | `blocked_route_replan_preflight` covers no-motion route blocking at Gateway preview level; `moving_obstacle_sweep` remains stricter with continuous moving actors, speed/density bins, clearance, live Fast-LIO/PCT/local/cmd proof, and video evidence. |
| Heavy-test separation | DimOS testing docs separate default tests from self-hosted/LFS/ROS/CUDA/MuJoCo/DimSim tests. | LingTu must report which evidence is local pytest, remote simulation, replay, or hardware; local green tests are not enough for algorithm health. |

## DimOS Benchmark Evidence Preset

Run this preset when the question is "do we have enough DimOS benchmark evidence to
trust the navigation algorithm surface?"

```bash
PYTHONPATH=src:. python3 sim/scripts/server_sim_closure.py \
  --preset dimos_benchmark \
  --required-only \
  --run-missing \
  --strict \
  --max-report-age-s 86400 \
  --json-out artifacts/server_sim_closure/summary_dimos_benchmark_24h.json
```

Run that command only after the target-host preflight is green. On a mixed
development workstation, use `--host-preflight` or the target-host runner
dry-run first; `--skip-host-blocked` is diagnostic-only and must not be used to
claim DimOS closure.

## Latest Target-Host Evidence, 2026-06-08

Current latest target-host evidence supersedes the older domain75/domain76/
domain83/domain108/domain138 historical notes below. It was generated in
`/home/bsrl/hongsenpang/lingtu_dimos_20260608` with `ROS_DOMAIN_ID=79`,
`MUJOCO_GL=egl`, and `PYOPENGL_PLATFORM=egl`.

- Host preflight:
  `artifacts/server_sim_closure/host_preflight_after_large_loop_report_guard.json`
  reports `ok=true`, `blocked_gates=[]`, and all 13 required gates runnable.
- Historical strict summary from that rerun window:
  `artifacts/server_sim_closure/summary_after_large_loop_report_guard.json`
  reports `ok=false` with `missing_or_failed=["large_loop_closure"]`; the
  runner and large-loop aggregation keep the closure red instead of reusing a
  stale child report after a failed live launch.
- Current gap report:
  `artifacts/server_sim_closure/dimos_gap_after_large_loop_report_guard.json`
  reports `lingtu_readiness.ok=false`,
  `lingtu_readiness.claim_allowed=false`, `host_preflight_ok=true`,
  `gap_counts.required=13`, `gap_counts.passed=12`,
  `gap_counts.failed=1`, `runtime_dataflow_checked=true`,
  `runtime_dataflow_complete=true`, and `runtime_dataflow_ok=false`.

The passing required gates are `gateway_runtime_acceptance`,
`routecheck_preflight`, `blocked_route_replan_preflight`,
`navigation_replay_deviation`, `large_terrain`, `native_pct_mujoco`,
`dynamic_obstacle_local_planner`, `fastlio2_dynamic_inspection`,
`moving_obstacle_sweep`, `gazebo_runtime`, `saved_map_relocalize`, and
`pct_saved_map_navigation`. The only remaining failed required gate is
`large_loop_closure`. The `pct_saved_map_navigation`
report at `artifacts/server_sim_closure/pct_saved_map_navigation/report.json`
is now green with `plan_preview.ok=true`, `selected_planner=pct`,
`pct_planner_runtime.runtime=rust_process`, `pct_planner_runtime_ok=true`,
`global_planner_source=source_report/pct_tomogram`,
`pct_optimizer_enabled=false`, `pct_planner_path_mode=astar_raw_path`,
`path_count=7`, and `native_gate.ok=true` with `fallback_used=false`. This is
canonical PCT saved-map evidence from the selected planner runtime, with no
planner fallback.

The acceptance consumer fails closed unless `pct_planner_runtime` is a mapping
with a selected runtime and `ok=true`, the report also records
`pct_planner_runtime_ok=true`, the global-planner source is exactly
`source_report/pct_tomogram`, and the path mode is `astar_raw_path` or
`optimized_trajectory`. Legacy `pct_runtime_ok` and
`native_astar_raw_path` fields are not accepted as default readiness evidence.

This still does not permit a DimOS readiness claim. The latest
`large_loop_closure` report has `ok=false`, `passed_case_count=0`, and
`blockers=["no passing large-loop runtime report"]`. The source live run
`artifacts/server_sim_closure/mujoco_fastlio2_live/inspection-loop-video-20260608_164410/`
ended without writing a child `report.json` before the launcher-level fallback
was installed. A red diagnostic child report now records
`runtime_report_missing_after_launcher`, and future launcher runs write the same
kind of red report automatically when a child exits without a report. Until
`lingtu_readiness.ok=true`, `claim_allowed=true`, and all 13 required gates
pass, do not claim `PCT_planner + MuJoCo` or saved-map navigation as full DimOS
closure.

Historical Linux target-host work was executed in the isolated workspace
`/home/bsrl/hongsenpang/lingtu_dimos_20260608` with `ROS_DOMAIN_ID=75`.
The earlier red preflight was real: the ROS 2 `local_planner` package was not
visible, so the closure runner stopped before launching runtime gates. That
blocker has now been moved forward by building the `local_planner` package with
`colcon`, verifying `localPlanner` and `pathFollower`, and syncing the official
MID-360 scan-pattern asset into `sim/assets/livox/mid360.npy`.

The older target-host preflight artifact
`artifacts/server_sim_closure/host_preflight_after_mid360_sync.json` reported
`ok=true`, `failed_checks=[]`, and `blocked_gates=[]` before Fast-LIO2 package
availability was audited. That audit exposed a real blocker in
`host_preflight_dimos_benchmark_after_fastlio_guard.json`, but it has now moved
forward: after building/sourcing Fast-LIO2, the current sourced target-host
preflight artifact is
`artifacts/server_sim_closure/host_preflight_dimos_benchmark_after_fastlio2_build_domain75.json`.
It reports `ok=true`, `failed_checks=[]`, `blocked_gates=[]`, and all 13 DimOS
required gates runnable. `ros2 pkg executables fastlio2` exposes
`fastlio2 lio_node`, `ros2_local_planner.ok=true`, hardware subscriber audit is
green, the selected PCT planner runtime was green in that historical target-host
report, MuJoCo headless is green, and the official MID-360 asset is present. The older
`host_preflight_after_pct_raw_path.json`,
`host_preflight_after_local_planner_guard.json`,
`host_preflight_after_mid360_sync.json`, and
`host_preflight_dimos_benchmark_after_fastlio_guard.json` reports remain useful
only as historical diagnostics.

The historical PCT legacy-native parity diagnostic
`artifacts/server_sim_closure/pct_runtime_preflight_remote_after_build_report.json`
reports `ok=true` with Linux x86_64, Python `py310`, all required legacy PCT
extension modules present, and no missing shared libraries. It inventories the
old Linux/GTSAM runtime and is not the current default planner contract. The
`native_pct_mujoco` report at
`artifacts/server_sim_closure/native_pct_mujoco/report_after_goal_reached_fix.json`
is retained as historical ROS2 compatibility evidence. Current readiness accepts
only the canonical selected-runtime fields, including
`pct_planner_runtime={"runtime":"rust_process","ok":true}`,
`pct_planner_runtime_ok=true`, and
`global_planner_source=source_report/pct_tomogram`. The old
`pct_native_backend_used` field is historical only and is not accepted as
current readiness evidence. The recorded run had `path_count=175`,
`cmd_count_nonzero=484`, `moved_m=3.4101`, `final_distance_m=0.4944`, and
`reached_goal=true`. Its trajectory-quality check was green with
`final_progress_ratio=0.8593` and `goal_reached_override=true`, which prevented
a successful near-goal run from being rejected only because the projected
route-progress ratio was below the long-route threshold.

That does not make the DimOS benchmark green. A full target-host execute after
the Fast-LIO2 build produced
`artifacts/server_sim_closure/summary_after_fastlio2_build_execute.json`; the
strict result is still 7 of 13 required gates passed and 6 of 13 failed or
missing. After fixing the observed Python runtime type faults in
`OccupancyGridModule` and `WaypointTracker`, disabling the live-gate GPMP
optimizer by default, and cleaning stale Fast-LIO2 nodes between runs, a clean
`ROS_DOMAIN_ID=76` inspection generated
`artifacts/server_sim_closure/mujoco_fastlio2_live/inspection-moving-obstacle-video-20260608_064916/report.json`.
That report is useful because it no longer dies in the PCT native optimizer and
it proves the live chain is partially connected:
`fastlio2_cloud_registered=55`, `fastlio2_cloud_map=55`,
`fastlio2_odometry=55`, `nav_map_cloud=55`, `nav_registered_cloud=55`,
`nav_odometry=55`, `global_path_count=1`, `local_path_count=55`,
`nav_cmd_vel=62`, and `nav_cmd_vel_nonzero=18`.

The same report is still red. Its current primary dataflow blocker is
`fastlio_motion_consistency`: Fast-LIO reports about `0.983m` motion while the
MuJoCo body reports about `0.00072m`, the inspection reaches `0/3`
checkpoints, moving-obstacle point updates are missing/empty, and
`body_to_camera` frame evidence is missing. The refreshed target-host summary
and gap report are:

- `artifacts/server_sim_closure/summary_after_fastlio2_cleanup_domain76_refreshed.json`
- `artifacts/server_sim_closure/dimos_gap_after_fastlio2_cleanup_domain76_refreshed.json`

They report `lingtu_readiness.ok=false`, `claim_allowed=false`,
`gap_counts.required=13`, `gap_counts.passed=7`, and
`gap_counts.failed=6`. The passing gates are
`gateway_runtime_acceptance`,
`routecheck_preflight`, `blocked_route_replan_preflight`,
`navigation_replay_deviation`, `large_terrain`, `native_pct_mujoco`, and
`dynamic_obstacle_local_planner`. The failing or missing gates are
`fastlio2_dynamic_inspection`, `moving_obstacle_sweep`, `large_loop_closure`,
`gazebo_runtime`, `saved_map_relocalize`, and `pct_saved_map_navigation`.
`fastlio2_dynamic_inspection` is no longer a host-blocked gate; it is now a
runtime-red gate. The runtime dataflow checker classifies the red edges as
`fastlio_motion_consistency` for `fastlio2_dynamic_inspection`,
`report_missing` for `moving_obstacle_sweep`, `large_loop_closure`, and
`pct_saved_map_navigation`, `gazebo_tf_topic_contract` for `gazebo_runtime`,
and `same_source_map_metadata_contract` for `saved_map_relocalize`.

Fresh target-host evidence after the MuJoCo live-gate hardening was generated
in the same Linux workspace. With `ROS_DOMAIN_ID=83`,
`artifacts/server_sim_closure/host_preflight_after_motion_window_domain83.json`
reports `ok=true`, `failed_checks=[]`, `blocked_gates=[]`, and all 13 required
gates runnable. The latest focused live inspection report is
`artifacts/server_sim_closure/mujoco_fastlio2_live_motion_window/inspection-moving-obstacle-video-20260608_074940/report.json`.
It confirms that several earlier blockers moved forward:
`runtime_evidence.ok=true`, `body_to_camera` frame evidence is present,
moving obstacles publish non-empty points, PCT is the selected global planner,
global/local path and `/nav/cmd_vel` evidence exist, and no hardware command
sink is used. It is still a failed gate: `ok=false`, Fast-LIO reports
`3.278m` motion against `1.459m` MuJoCo motion
(`motion_scale_ratio=2.2468`), angular saturation ratio is `0.5822`, and the
inspection reaches only `1/3` checkpoints.

The fresh summary and gap outputs are:

- `artifacts/server_sim_closure/dimos_benchmark_after_motion_window.json`
- `artifacts/server_sim_closure/dimos_gap_after_motion_window.json`
- `artifacts/server_sim_closure/dimos_gap_after_motion_window.md`

They intentionally used a strict freshness window (`max_report_age_s=7200`).
Older successful artifacts were stale under that window, so the domain83
fresh-window status at that stage was `lingtu_readiness.ok=false`,
`claim_allowed=false`, and `0/13` required gates passed. This does not
contradict the earlier 7/13 historical summary; it prevents stale artifacts
from being reused as a readiness claim.

The previous upstream blocker was the `large_terrain` legacy PCT
GPMP/height-smoother crash. The recorded report at
`artifacts/server_sim_closure/large_terrain/report.json` is green because the
gate ran PCT in an isolated child process with
`LINGTU_PCT_OPTIMIZE_TRAJECTORY=0`. Each route records
`pct_planner_runtime={"runtime":"rust_process","ok":true}`,
`pct_planner_runtime_ok=true`, `pct_optimizer_enabled=false`,
`pct_planner_path_mode=astar_raw_path`, and child process return code 0. This is
a selected PCT planner-runtime discrete-path pass over same-source map/tomogram
assets, not an A* fallback. The `native_pct_mujoco` artifact remains historical
ROS2 compatibility evidence for the PCT-to-local-planner/path-follower-to-MuJoCo
kinematic motion loop; it is not the current default runtime contract. Neither
artifact proves that Fast-LIO live inspection, moving-obstacle sweep,
large-loop closure, Gazebo runtime, saved-map relocalization, or saved-map PCT
navigation are healthy.

For a reviewer-facing gap matrix without manually reading every gate report:

```bash
PYTHONPATH=src:. python3 sim/scripts/dimos_gap_report.py \
  --summary artifacts/server_sim_closure/summary_dimos_benchmark_24h.json \
  --host-preflight \
  --include-dataflow \
  --format markdown \
  --json-out artifacts/server_sim_closure/dimos_gap_report.md
```

Omit `--summary` to build the matrix from the current
`artifacts/server_sim_closure/` reports. The generated report preserves the
canonical `dimos_benchmark` gate order, classifies each blocker, and keeps the
next strict rerun command next to the failing gate. `--host-preflight` is
read-only: it checks whether the current host can run the selected gates, but it
does not launch ROS, MuJoCo, PCT, or any gate command. When host checks fail,
the report also includes a `Host Setup Blockers` section that groups blocked
gates by failed check such as `pct_planner_runtime`, `ros2_humble`,
`mujoco_headless`, or `isolated_ros_domain`. The default PCT check asks whether
the selected planner runtime is available; it does not require Linux, CPython
3.10, or GTSAM native parity. An unavailable runtime is reported as
`PCT planner runtime unavailable`.

If the input summary already came from `server_sim_closure.py --run-missing
--skip-host-blocked`, the gap report reads the embedded
`run_missing_host_preflight` and `gate_runs[].status="host_blocked"` evidence.
Those rows are shown as `host_blocked` without requiring a second preflight
command.

`--include-dataflow` inspects existing runtime gate reports and attaches a
`runtime_dataflow` section. For flat live Fast-LIO reports it summarizes
`outputs` and `lingtu_inspection` directly. For aggregate or wrapper gates it
follows child report paths when present, or uses embedded case evidence such as
`live_nav_chain`, `best_case`, `minimal_red_defect`, and `native_gate`. The
reported edges identify where a stale or red runtime report stopped: Fast-LIO
feedback, PCT/global path, local path, cmd_vel, MuJoCo motion, or checkpoints.
This is diagnostic evidence only; a stale or missing report remains red even if
an older child report shows an internally connected data path.
`lingtu_readiness.ok` requires this runtime-dataflow pass; a green gate summary
without `--include-dataflow` is treated as an incomplete claim boundary, not a
closed DimOS-style proof.

For `pct_saved_map_navigation`, the parser keeps the wrapper semantics instead
of reducing the report to the embedded native gate. Its dataflow row must show
saved-map relocalization, PCT preview, source-report handoff, native PCT
backend, local planner/path follower command output, MuJoCo motion, and final
goal evidence. This prevents a passing embedded native run from hiding a failed
saved-map relocalization prerequisite.

The same offline artifact parser is used by the Gateway diagnostic endpoint
`/api/v1/diagnostics/algorithm-benchmark/latest`: the endpoint remains
read-only, but its `dimos_gap` payload includes the same runtime-dataflow row
briefs when the selected benchmark summary points at readable gate reports.
This is separate from the live Gateway `/api/v1/runtime/dataflow` endpoint,
which observes current Module/Gateway ports rather than historical gate
artifacts.

The runtime-dataflow summary also carries a cross-gate chain named
`pct_mujoco_and_fastlio_live`. It requires `native_pct_mujoco` dataflow for
PCT -> localPlanner/pathFollower -> MuJoCo evidence plus at least one live
Fast-LIO gate (`fastlio2_dynamic_inspection`, `moving_obstacle_sweep`, or
`large_loop_closure`). The live Fast-LIO gate must also prove PCT planner
provenance in that same run (`global_planner=pct` and no planner fallback or
repair). Two separately green reports are not enough: `same_run_proven=false`
keeps `lingtu_readiness.ok=false` until the Fast-LIO live report itself proves
PCT -> local path -> cmd_vel -> MuJoCo motion in the same runtime. A failing
cross-gate chain keeps `lingtu_readiness.ok=false` even if the individual gate
summary appears green.

Same-source proof is artifact-level, not just a boolean flag. The live Fast-LIO
report must carry `map_artifacts.source_contract.same_source_pcd=true`, a
non-empty `map_artifacts.assets.map_pcd.sha256`, positive
`map_pcd.point_count`, and, when a tomogram is present,
`same_source_tomogram=true` with
`map_artifacts.assets.tomogram.source_map_sha256` matching the map PCD sha.
`map_artifacts.ok=true` or matching parent directories alone are not sufficient
to prove DimOS-style same-run map/tomogram provenance.

When the blockers are ROS/PCT host setup related, use the split native and
compatibility entrypoints instead of inventing a parallel installer:

```bash
PYTHONPATH=src:. python3 sim/scripts/pct_runtime_preflight.py \
  --json-out artifacts/server_sim_closure/pct_runtime_preflight/report.json

PYTHONPATH=src:. python3 sim/scripts/saved_map_relocalize_runtime_gate.py \
  --preflight-only \
  --json-out artifacts/server_sim_closure/saved_map_relocalize_runtime_preflight/report.json

bash sim/scripts/setup_linux_validation_host.sh
LINGTU_INSTALL_ROS2=1 LINGTU_RUN_ROS2_FASTLIO2=1 \
  bash scripts/compat/ros2/setup_fastlio2_validation_host.sh

PYTHONPATH=src:. python3 sim/scripts/pct_runtime_preflight.py \
  --strict \
  --json-out artifacts/server_sim_closure/pct_runtime_preflight/report.json

PYTHONPATH=src:. python3 sim/scripts/saved_map_relocalize_runtime_gate.py \
  --preflight-only \
  --json-out artifacts/server_sim_closure/saved_map_relocalize_runtime_preflight/report.json

PYTHONPATH=src:. python3 sim/scripts/server_sim_closure.py \
  --preset dimos_benchmark \
  --required-only \
  --host-preflight \
  --json-out artifacts/server_sim_closure/host_preflight_dimos_benchmark.json
```

The first PCT runtime preflight is non-strict so the JSON report can capture
the current ABI, platform, extension-module, and shared-library blockers
without aborting a setup script. After the native
`sim/scripts/setup_linux_validation_host.sh` finishes, run the same check with
`--strict`; it must pass before treating PCT-backed gates such as
`large_terrain`, `native_pct_mujoco`, `moving_obstacle_sweep`, and
`pct_saved_map_navigation` as runnable. The saved-map relocalize preflight is
also non-motion: it checks same-source map assets, localizer config, current
host markers, and ROS 2 Python importability, but it does not launch
MuJoCo/Fast-LIO/localizer or prove `runtime_relocalization_validated=true`.
The final `server_sim_closure.py --host-preflight` confirms the whole DimOS
gate set, including ROS 2 Humble, isolated `ROS_DOMAIN_ID`, MuJoCo/Gazebo
availability, and hardware-subscriber safety checks.

Do not combine `--host-preflight` with `--run-missing`: preflight is read-only,
while `--run-missing` launches gate commands. For the DimOS target-host closure,
use `sim/scripts/run_dimos_linux_closure.sh`; it runs preflight first and
refuses runtime gates while the host is red. `--skip-host-blocked` remains a
developer diagnostic mode for local aggregation, not an acceptance path.

The generated JSON also contains `execution_plan`. It is intentionally
conservative: if the host preflight is red, `ok_to_run_missing=false` and the
gate commands are listed under `blocked_gate_commands` for later execution on a
passing simulation host. This prevents a local Windows/Python report from being
mistaken for proof that the ROS/PCT/MuJoCo pipeline can run here.
Runnable and blocked command phases use `order=dimos_dependency_order` so
upstream assets such as `large_terrain` scene/tomogram reports are generated
before downstream MuJoCo/PCT stress gates, even when those downstream gates are
higher priority in the gap matrix.

To export the same execution plan as a shell-readable command checklist:

```bash
PYTHONPATH=src:. python3 sim/scripts/dimos_gap_report.py \
  --summary artifacts/server_sim_closure/summary_dimos_benchmark_24h.json \
  --host-preflight-report artifacts/server_sim_closure/host_preflight_dimos_benchmark.json \
  --include-dataflow \
  --format shell \
  --json-out artifacts/server_sim_closure/dimos_execution_plan.sh
```

Use `--host-preflight` when you want `dimos_gap_report.py` to run the read-only
checks in-process. Use `--host-preflight-report <json>` when the preflight was
already produced by `server_sim_closure.py --host-preflight`; the report will
preserve the source as `file:<path>` so environment claims remain traceable.
When `--summary <json>` is supplied, the summary file's own modification time
is checked against `--max-report-age-s`; a stale summary remains red even if its
embedded gate payloads look green. Runtime-dataflow closure is also complete
only when every required runtime gate has an inspected dataflow row, so a subset
of green runtime reports cannot unlock `lingtu_readiness.ok`.

Blocked gate commands are emitted as `# BLOCKED:` comments until host preflight
passes. Host setup diagnostics are emitted as executable commands in the
`host_setup` phase: the shell checklist runs the non-strict PCT runtime
diagnostic, the server setup script, the strict PCT runtime diagnostic, and the
full host preflight before any downstream gate command is uncommented.

The report also includes `pipeline_trace`, a static code-path map for the
PCT/MuJoCo navigation chain. It names the code surfaces for
`GlobalPlannerService`, PCT backend loading, native `localPlanner`/
`pathFollower`, MuJoCo motion execution, Fast-LIO feedback, and saved-map PCT
navigation. This trace answers where the program path lives, but it is not a
runtime pass signal; the matching gate still has to pass.

For local source-report wiring checks that do not require ROS 2 or MuJoCo,
`mujoco/native_pct_gate.py` also supports a contract-only mode:

```bash
PYTHONPATH=src:. python3 sim/scripts/mujoco/native_pct_gate.py \
  --contract-only \
  --strict \
  --source-report artifacts/server_sim_closure/large_terrain/report.json \
  --route same_floor \
  --json-out artifacts/server_sim_closure/native_pct_mujoco/contract_report.json
```

This mode validates the PCT source report, no-fallback selection, native
runtime fields, tomogram presence, path safety, and same-source map/tomogram
artifact proof. It also emits `command_generation` with the source-report
fingerprint and the exact `localPlanner`/`pathFollower` command contract that
would be launched on a ROS 2 simulation host. Its report is marked
`validation_only=true` and
`claim_boundary=contract_only_no_ros_mujoco_motion`; it must not be counted as
`native_pct_mujoco` runtime evidence because it does not launch ROS 2
`localPlanner`/`pathFollower`, create a MuJoCo engine, publish cmd_vel, or prove
goal-reaching motion.

The saved-map wrapper has the same local-only boundary when an already-built
PCT source report is available:

```bash
PYTHONPATH=src:. python3 sim/scripts/pct_saved_map_navigation_gate.py \
  --contract-only \
  --strict \
  --relocalize-report artifacts/server_sim_closure/saved_map_relocalize/report.json \
  --tomogram artifacts/server_sim_closure/saved_map_relocalize/same_source_map/tomogram.pickle \
  --source-report artifacts/server_sim_closure/pct_saved_map_navigation/pct_saved_map_source_report.json \
  --json-out artifacts/server_sim_closure/pct_saved_map_navigation/contract_report.json
```

That mode checks relocalization health, saved-map tomogram resolution,
source-report map/tomogram/scene binding, no-fallback PCT selection, and
same-source artifact metadata. It is marked
`claim_boundary=contract_only_no_pct_preview_or_mujoco_motion`, so it is only a
local wiring check; the full `pct_saved_map_navigation` gate still has to run
PCT preview and native MuJoCo motion on the simulation host.

`dimos_benchmark` requires:

| Gate | DimOS-style evidence covered |
| --- | --- |
| `gateway_runtime_acceptance` | Product-visible Gateway/ModulePort runtime streams, stage evidence, and non-motion command whitelist. |
| `routecheck_preflight` | Non-motion route preview and command-safety counters. |
| `blocked_route_replan_preflight` | Non-motion blocked-route preview: baseline path intersects a synthetic obstruction, candidate path replans around it, and no motion command is published. |
| `navigation_replay_deviation` | Offline replay/deviation over routecheck-derived plans, JSON traces, or recorded topic JSONL global path, local path, cmd_vel, and odometry; it stays non-motion and fails when the source evidence is missing. |
| `large_terrain` | Fixed large-scene route matrix over saved tomogram assets. |
| `native_pct_mujoco` | PCT global path through local planner/path follower into simulated motion. |
| `dynamic_obstacle_local_planner` | Local replanning around changing obstacle phases. |
| `fastlio2_dynamic_inspection` | Raw LiDAR/IMU -> Fast-LIO2 -> LingTu nav plus PCT patrol and moving obstacles with video. |
| `moving_obstacle_sweep` | Aggregates live dynamic-obstacle videos across required speed and point-density bins, with each bin proving the Fast-LIO -> PCT -> local path -> cmd_vel chain. |
| `large_loop_closure` | Large loop route with live Fast-LIO, PCT, local path/cmd_vel closure, video, and return-to-start drift bounds. |
| `gazebo_runtime` | ROS-native smoke, navigation loop, map growth, frontier exploration, and publisher identity. |
| `saved_map_relocalize` | Saved-map/localizer contract, same-source metadata, live Fast-LIO, and localization health. |
| `pct_saved_map_navigation` | Saved map/tomogram -> PCT -> local planning/path following -> motion. |

Recorded navigation topics can now enter the aggregate gate directly with
`server_sim_closure.py --navigation-replay-deviation-topic-jsonl <path>`.
That keeps the DimOS-style replay/deviation lane close to rosbag tests while
preserving the boundary that live PCT/MuJoCo closure still requires the
runtime gates below.

Existing moving-obstacle child reports can also be re-aggregated locally
without launching the ROS2/MuJoCo matrix:

```bash
PYTHONPATH=src:. python3 sim/scripts/server_sim_closure.py \
  --preset dimos_benchmark \
  --required-only \
  --moving-obstacle-sweep-report-glob \
    'artifacts/server_sim_closure/mujoco_fastlio2_live*/inspection*/*/report.json' \
  --json-out artifacts/server_sim_closure/summary_dimos_benchmark_24h.json
```

This path writes the standard
`artifacts/server_sim_closure/moving_obstacle_sweep/report.json` aggregate with
`execution_mode=report_only_aggregate`. It still requires the child reports to
prove physical-rolling scan timing, Fast-LIO2 nav data source, PCT planner
provenance, local path, nonzero cmd_vel, same-source map/tomogram artifacts,
and slow/fast x sparse/dense coverage. It does not run the matrix or create new
runtime evidence; it only makes already-captured live child reports visible to
the closure summary. Do not combine it with `--host-preflight`, because
preflight is read-only.

## Current Gaps To Keep Visible

| Gap | Why it matters | Next gate shape |
| --- | --- | --- |
| Fresh full-stack runtime evidence is expensive and environment-bound | A green stale artifact can hide current regressions. | Keep `--max-report-age-s` on benchmark summaries and rerun stale gates before claims. |
| Dynamic obstacle sweep runtime evidence is newly required | Passing one speed/density, or passing videos without proving live navigation chain wiring, does not prove crowd robustness. | Run `moving_obstacle_sweep` after generating slow/fast x sparse/dense live inspection videos. Each child case must carry `live_nav_chain.ok=true`; the aggregate must carry `required_live_nav_chain=true`. |
| Long-range loop closure runtime evidence is newly required | Short patrol proves local consistency, not large loop drift recovery. | Run `large_loop_closure` and require Fast-LIO/PCT/local path/cmd_vel plus start/end closure error. |
| Saved-map assets must remain same-source | Pairing a relocalization report with a different or merely newer tomogram/map can create false failures or false passes. | Keep saved-map relocalize and PCT navigation gates tied to the same `same_source_map` directory. |
| Real S100P hardware is outside the simulation closure | Simulation-only gates must not be called field readiness. | Keep `navigation_replay_deviation` green on real-log traces first, then add hardware dry-run with command boundary and localization health evidence. |
| Video evidence can become a weak proxy | Frame counters alone do not prove the MP4 artifact exists or is decodable. | Strict dynamic-inspection evaluation resolves the video path, requires a non-empty file, and decodes the first frame when OpenCV is available. |

The previous 8-gate `dimos_benchmark` required summary on 2026-05-21 is green:

- Report:
  `artifacts/server_sim_closure/summary_dimos_benchmark_after_saved_map_fix.json`
- `ok=true`, `missing_or_failed=[]`, `remaining_gaps=[]`.
- Verified required gates:
  `routecheck_preflight`, `large_terrain`, `native_pct_mujoco`,
  `dynamic_obstacle_local_planner`, `fastlio2_dynamic_inspection`,
  `gazebo_runtime`, `saved_map_relocalize`, and
  `pct_saved_map_navigation`.

The current preset is stricter than that artifact. It now also requires
`gateway_runtime_acceptance`, `blocked_route_replan_preflight`,
`navigation_replay_deviation`,
`moving_obstacle_sweep`, and `large_loop_closure`, so the old green summary is
no longer enough to claim full benchmark closure. Those reports must be freshly
generated before a new `dimos_benchmark` summary can be green.

Historical stricter status from the earlier remote physical-rolling reruns and
gate hardening. Keep this section as diagnostic history; the authoritative
current status is the 2026-06-08 target-host evidence section above:

- The strict live MuJoCo/Fast-LIO input default is now
  `scan_time_profile=physical_rolling`. This supersedes the earlier
  `synthetic_rolling` default because the accepted model must accumulate
  actual MuJoCo subscans over the MID-360 scan window and publish real
  per-point offsets. Fresh physical-rolling evidence is green for fixed
  forward+yaw Fast-LIO
  (`artifacts/server_sim_closure/fixed_forward_yaw_physical_pass/gate-20260521_154007/report.json`)
  and for a one-goal inspection using live Fast-LIO, saved large-terrain
  tomogram, PCT, local planner/path follower, and nonzero cmd_vel
  (`artifacts/server_sim_closure/inspection_onegoal_physical_fastlio/inspection-20260521_154351/report.json`).
  This is not a DimOS benchmark pass until the large-loop and dynamic-obstacle
  speed/density gates are refreshed under the same physical-rolling default.
- Historical strict summary from that rerun window:
  `artifacts/server_sim_closure/summary_dimos_benchmark_physical_rolling_current.json`
  has `ok=false`, `missing_or_failed=["large_loop_closure","moving_obstacle_sweep"]`,
  and `algorithm_validation.claim_allowed=false`.
- `moving_obstacle_sweep` was red under that strict evaluator:
  `artifacts/server_sim_closure/moving_obstacle_sweep/report_physical_rolling_20260521.json`.
  It covers `slow:sparse`, `slow:dense`, `fast:sparse`, and `fast:dense` with
  `scan_time_profile=physical_rolling`, the
  `physical_subscans_with_actual_sim_time_offsets` contract, real video files,
  moving obstacles, and no trail collision. All four child cases still fail:
  the robot reaches only one of three inspection checkpoints, and the child
  reports show Fast-LIO motion/Z divergence under the dynamic scene. The gate
  summary now surfaces `fastlio2_consistency`, `blocking_subsystems`, and
  `minimal_red_defect` so this failure is visible without manually opening
  every child report.
- `large_loop_closure` was red under that same-source strict evaluator:
  `artifacts/server_sim_closure/large_loop_physical_rolling_same_source/large_loop_closure_report.json`
  over child runtime report
  `artifacts/server_sim_closure/large_loop_physical_rolling_same_source/inspection-loop-video-20260521_161429/report.json`.
  It proves same-source world/tomogram artifacts, `physical_rolling` scan time,
  a written video (`video_frame_count=1801`), PCT global planning
  (`global_path_count=2`), local planning (`local_path_count=1131`), and
  nonzero navigation commands. It still fails the acceptance gate after the
  900 s wall guard with only one of four checkpoints reached. The gate now
  classifies a timeout with live local paths and cmd_vel as
  `planning_tracking`, not `validation_harness`, while true gate exceptions
  still remain `validation_harness`.
- The large-loop launcher default in `server_sim_closure` now uses explicit
  retest speed controls for this gate:
  `LINGTU_MUJOCO_LIVE_NAV_MAX_LINEAR_SPEED=0.45`,
  `LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_LIMIT=0.45`, and
  `LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_ACCEL_LIMIT=0.8`, all still overridable by
  environment variables. This does not change acceptance thresholds; it tests
  the hypothesis that the same-source run was progress-limited rather than
  only SLAM-limited.
- The benchmark summary must therefore keep `missing_or_failed` containing
  both `large_loop_closure` and `moving_obstacle_sweep`, and
  `algorithm_validation.claim_allowed` must be `false`.
- The current large-loop control matrix is also red:
  `artifacts/server_sim_closure/diagnosis_matrix/summary.json`.
  It now reports `slam_localization` as the blocking failure:
  - Static large-terrain Fast-LIO control is green
    (`z_delta_error_m=0.0146`), so startup alone is not the blocker.
  - 0.05 m/s fixed-motion Fast-LIO control is green
    (`sim_moved_m=0.9472`, `z_delta_error_m=0.0295`).
  - 0.10, 0.15, and 0.25 m/s controls are red. The 0.25 m/s control fails
    hard with `z_delta_error_m=31.5056`, yaw error `1.8064rad`, and motion
    divergence.
  - The best current 0.25 m/s Fast-LIO tuning control improves the red item
    but does not clear it (`z_delta_error_m=1.1448` with
    `lidar_filter_num=2`, `near_search_num=8`, and `ieskf_max_iter=8`).
  - A conservative full-chain PCT/Fast-LIO/local-planner control is green for
    a 20 s low-speed segment:
    `artifacts/server_sim_closure/diagnosis_matrix/large_loop_variants/fastlio_pct_conservative_v005_20s/inspection-loop-video-20260521_084605/report.json`.
    It proves the realtime chain is wired (`nav_data_source=fastlio2`,
    `global_path_count=1`, `local_path_count=191`, no runtime faults), but it
    uses `min_required_checkpoints=0`, so it is not a loop-closure pass.
  - Latest route-level scan-time A/B narrows the input-model issue:
    `instantaneous` fails almost immediately with motion/Z blow-up
    (`fastlio2_moved_m=144.7474`, `sim_moved_m=0.1669`,
    `z_delta_error_m=78.9128`), while `synthetic_rolling` removes that early
    Z/motion failure (`fastlio2_moved_m=1.4478`, `sim_moved_m=0.8890`,
    `z_delta_error_m=0.3416`) but still fails on yaw drift and patrol timeout.
    The validation launcher now defaults to `physical_rolling`; the older
    `synthetic_rolling` result remains a diagnostic ablation, not an acceptance
    pass, because `large_loop_closure` remains red.
  - The logs still show Fast-LIO degeneracy and IEKF non-convergence warnings,
    so the current blocker remains SLAM/localization robustness plus closed-loop
    yaw/path-following behavior, not PCT global planning or local path
    generation alone.
- The Fast-LIO speed/config diagnostic boundary is now explicit:
  `artifacts/server_sim_closure/diagnosis_matrix/fastlio_speed_boundary/report.json`.
  It is a diagnostic-only gate (`algorithm_pass=false`,
  `claim_allowed=false`), not an acceptance gate. Current fixed-drive evidence
  has `green_speed_mps=0.05`, `first_red_speed_mps=0.10`, and red speeds
  `[0.10, 0.15, 0.25]`. The minimal red defect is classified as
  `slam_localization` with time-aligned motion/Z drift at 0.10 m/s. The best
  known tuning control at 0.25 m/s improves the error but remains red.
  The diagnostic case schema now keeps `linear_y` and planar speed, which is
  required for the next PCT command-shape matrix.
- The refined speed/scan/tuning boundary is also diagnostic-only:
  `artifacts/server_sim_closure/diagnosis_matrix/fastlio_speed_boundary/refined_report.json`.
  It aggregates the older coarse controls plus refined 0.060/0.075/0.090/0.100
  m/s controls, instantaneous-vs-rolling scan timing controls, and the best
  known tuned control. A later route-level A/B made `synthetic_rolling` the
  interim MuJoCo MID-360 default, and that has since been superseded by
  `physical_rolling`.
  It reports `boundary_characterized=true`,
  `claim_allowed=false`, `fixed_control_count=21`, and
  `state="non_monotonic_or_unstable"`. The lowest reproduced red speed is
  0.06 m/s with `z_delta_error_m=1.4906`; because other historical controls
  include green runs up to 0.15 m/s, this is an instability boundary rather
  than a clean monotonic speed limit. This section predates the
  `physical_rolling` default and remains useful as a diagnostic boundary, not
  as current acceptance evidence.
- The yaw-rate diagnostic boundary is now explicit:
  `artifacts/server_sim_closure/diagnosis_matrix/yaw_rate_boundary/summary_vx_wz_report.json`.
  It keeps straight speed and turn-rate cases separate. The current summary has
  four fixed controls. Straight `vx=0.10,wz=0.0` is a `slam_localization` red
  item (`z_delta_error_m=2.2031`). Turning has one SLAM red item,
  `vx=0.05,wz=0.25` (`z_delta_error_m=1.2398`), while spin
  `vx=0.0,wz=0.25` and sharper turn `vx=0.10,wz=0.45` are classified as
  `validation_runtime` because Fast-LIO Z/yaw passed but the run hit the
  wall-time guard. This diagnostic report cannot green-light the benchmark; it
  narrows the current large-loop blocker.
- The time-offset diagnostic boundary is also red. The fixed `vx=0.10,wz=0.0`
  controls show `time_diff_lidar_to_imu=+0.010` improves Z drift relative to
  `0.0`, while `-0.010` and `+0.020` are worse. The best combined
  `+0.010` plus tuned Fast-LIO control is still above threshold
  (`z_delta_error_m=1.7469`), so time offset is not sufficient to close
  `large_loop_closure`.
- The MID-360 point-density diagnostic is the strongest new root-cause clue.
  On the same fixed `vx=0.10,wz=0.0` control, 12000 and 15000 samples per frame
  pass for 20.02 s sim time with an expanded wall-time guard, while 18000 and
  the original 24000-sample profile are red. Summary:
  `artifacts/server_sim_closure/diagnosis_matrix/point_density_boundary/summary.json`.
  This remains diagnostic-only; reducing samples is not an acceptance shortcut
  until the same profile passes the large-loop gate and the validation profile
  decision is documented.
- The first route-level 15k-sample transfer check is red:
  `artifacts/server_sim_closure/large_loop_samples_15000_60s/large_loop_closure_report.json`.
  The route had PCT and local planning active (`global_planner=pct`,
  `local_path_count=207`, `nav_cmd_vel_nonzero=218`) and no wall timeout, but
  Fast-LIO diverged after roughly 20 s (`fastlio2_moved_m=1226.861` versus
  `sim_moved_m=3.0639`, `z_delta_error_m=594.5294`). The fixed-control green
  window therefore does not close the DimOS benchmark gap by itself; the next
  matrix needs command-shape controls that reproduce PCT yaw/lateral behavior
  before changing acceptance thresholds.
- The first 15k-sample command-shape matrix has now reproduced a smaller
  failing boundary:
  `artifacts/server_sim_closure/diagnosis_matrix/command_shape_boundary_summary.json`.
  It aggregates six fixed-control reports. Low-speed yaw/lateral cases pass,
  and `vx=0.25,vy=0.0,wz=0.0` passes as a high-speed straight control. The
  first red turning SLAM case is `vx=0.25,vy=0.0,wz=0.45`
  (`z_delta_error_m=26.0378`, `yaw_delta_error_rad=1.2663`,
  `fastlio2_moved_m=104.0555` versus `sim_moved_m=0.4910`). Adding
  `vy=0.08` at the same `vx/wz` is also red, but only by Z drift
  (`z_delta_error_m=1.2869`) with yaw/motion checks still green. This keeps the
  DimOS benchmark red while narrowing the next optimization to Fast-LIO
  robustness under high forward speed plus high yaw rate.
- The first tuned command-shape matrix is still diagnostic-only:
  `artifacts/server_sim_closure/diagnosis_matrix/command_shape_tuned_summary.json`.
  With 15k MID-360 samples, `time_diff_lidar_to_imu=+0.010`, and the best
  current Fast-LIO tuning, the lateral turn
  `vx=0.25,vy=0.08,wz=0.45` becomes green (`z_delta_error_m=0.1027`,
  `yaw_delta_error_rad=0.1629`). The no-lateral turn at the same speed remains
  red by Z drift (`vx=0.25,vy=0.0,wz=0.45`,
  `z_delta_error_m=2.5252`). Lower yaw rates are also not clean:
  `wz=0.35` is yaw-red and `wz=0.25` is Z-red. This improves the failure mode
  but does not close `large_loop_closure`.
- Live MuJoCo/Fast-LIO reports now carry
  `fastlio_large_loop_diagnostic_report` as diagnostic evidence. This report is
  not an acceptance signal; it preserves the evidence needed to debug a red
  large-loop result: segment consistency, IMU statistics, scan timing
  statistics, and command trajectory statistics. The current instrumentation
  smoke artifact is
  `artifacts/server_sim_closure/diagnosis_matrix/diagnostic_instrumentation_smoke/report.json`.

Without `--run-missing`, `server_sim_closure.py` only evaluates existing
artifacts and returns the ordered missing gate commands in
`missing_required_commands`. With `--run-missing`, it executes missing or stale
required gates in preset order, records `gate_runs`, and then summarizes again.

The strict summary now includes `algorithm_validation.validation_flow`. That
flow is the review surface for the whole algorithm: map asset, static global
planning, local dynamic avoidance, realtime Fast-LIO mapping/localization,
long-range loop closure, saved-map lifecycle, command safety, and ROS
integration. A green child gate cannot override a red stage. The claim boundary
also states that PCT global planning uses static saved map/tomogram artifacts,
while live costmap and moving obstacles are local-planning and safety inputs.

Dynamic sweep command shape:

```bash
PYTHONPATH=src:. python3 sim/scripts/moving_obstacle_sweep_gate.py \
  --run-matrix \
  --child-run-root artifacts/server_sim_closure/moving_obstacle_sweep/children \
  --inspection-tomogram artifacts/server_sim_closure/large_terrain_odom/tomogram.pickle \
  --report-glob 'artifacts/server_sim_closure/mujoco_fastlio2_live*/inspection*/*/report.json' \
  --required-speed-bins slow,fast \
  --required-density-bins sparse,dense \
  --required-scan-time-profile physical_rolling \
  --require-video-file \
  --json-out artifacts/server_sim_closure/moving_obstacle_sweep/report.json \
  --strict
```

Large loop command shape:

```bash
bash sim/scripts/mujoco/launch_fastlio2_live.sh inspection-loop-video

PYTHONPATH=src:. python3 sim/scripts/large_loop_closure_gate.py \
  --report artifacts/server_sim_closure/mujoco_fastlio2_live/<inspection-loop-run>/report.json \
  --required-scan-time-profile physical_rolling \
  --require-video-file \
  --json-out artifacts/server_sim_closure/large_loop_closure/report.json \
  --strict
```

Historical domain108 saved-map diagnosis, superseded by the latest target-host
evidence above: the saved-map relocalize gate was updated to accept both historical
`lingtu.same_source_map_artifacts.*` metadata and the current
`lingtu.saved_map_artifacts.*` schema, and host preflight was updated to require
the ROS 2 `localizer_node` executable before the Linux closure runner can start
runtime gates. At that domain108 stage, the target report used live MuJoCo
MID-360/IMU through Fast-LIO, `/nav/relocalize`, and localizer health, but it
was still a failed gate:

- `artifacts/server_sim_closure/saved_map_relocalize_runtime/report.json`
- `map_metadata_contract.ok=true`, `schema_version=lingtu.saved_map_artifacts.v1`.
- `/nav/relocalize` service returns success and
  `localizer.latest_health_state=LOCKED`.
- At that stage the gate remained red because `live_feed.ok=false`:
  `fastlio2_moved_m=1.555`, `sim_moved_m=0.029`,
  `motion_scale_ratio=54.29`, and `yaw_per_meter` is too high.

At that same domain108 stage, the saved-map PCT gate refused stale or failed
relocalization prerequisites. The report was present and fresh, but it was still
red:

- `artifacts/server_sim_closure/pct_saved_map_navigation/report.json`
- `relocalize_report_freshness.fresh=true`.
- `relocalization.ok=false` because the relocalize gate itself was red.
- `plan_preview.skipped=true` and `native_gate.skipped=true` with
  `reason=saved_map_relocalization_prerequisite_failed`, so no PCT preview or
  native saved-map navigation proof is attempted until relocalization passes.

The saved-map runtime gate now reads `same_source_map/metadata.json` for the
MuJoCo world and scan-time profile instead of defaulting to an unrelated
scene. New same-source map artifacts also record `scan_time_profile`,
`nav_data_source`, and `fastlio_lidar_input`.

## Acceptance Boundary

Passing the current `dimos_benchmark` means LingTu has current
simulation/replay-style evidence across realtime mapping/localization, PCT
global planning, local avoidance, dynamic-obstacle speed/density coverage,
large-loop drift bounds, frontier smoke, saved-map navigation, and command
safety. It still does not prove physical gait robustness, real MID-360 timing
on the robot, or field SLAM quality.
