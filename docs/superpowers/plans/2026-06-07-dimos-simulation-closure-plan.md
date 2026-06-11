# DimOS Simulation Closure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring LingTu's simulation and planning evidence up to the DimOS-level claim bar without overstating what has not run on the correct host.

**Architecture:** Keep Module-First boundaries intact. Treat MuJoCo/Fast-LIO2/PCT/saved-map proof as a gated evidence pipeline: host preflight, native planner load, live simulator data flow, same-source map artifacts, then aggregate benchmark closure.

**Tech Stack:** Python 3.10 target runtime, ROS 2 Humble, MuJoCo EGL/headless runtime, existing PCT native modules, pytest, existing LingTu server simulation closure gates.

---

## Current Evidence Boundary

- Code chain exists for MuJoCo live and PCT-oriented runtime entry points in `src/core/cli/runtime_endpoint.py`, `sim/scripts/native_pct_mujoco_gate.py`, `sim/scripts/moving_obstacle_sweep_gate.py`, and `sim/scripts/large_loop_closure_gate.py`.
- PCT native runtime is represented by `src/global_planning/pct_planner_runnable/runtime.py` and the navigation path is dispatched through `src/nav/global_planner_service.py`.
- Fast-LIO2 and live navigation evidence is evaluated through `sim/scripts/server_sim_closure.py` and data-flow cross gates in `src/core/dimos_runtime_dataflow.py`.
- Current local Windows/Python 3.13 host is not enough to prove DimOS closure: PCT native artifacts are Linux/Python 3.10-oriented, ROS 2 Humble is not sourced, and MuJoCo/Fast-LIO2 live proof has not been freshly produced here.
- Latest target-host gap state is still red overall: `readiness_ok=false`, `claim_allowed=false`, 13 required gates, 7 passed, and 6 failed/missing. The target-host preflight is now green after building/sourcing ROS 2 `local_planner` and Fast-LIO2, verifying `localPlanner`, `pathFollower`, and `fastlio2 lio_node`, and syncing the official MID-360 scan-pattern asset. `native_pct_mujoco` now passes with native PCT, ROS 2 local planner/path follower, and MuJoCo kinematic motion. `fastlio2_dynamic_inspection` is no longer host-blocked, but remains runtime-red on motion consistency, checkpoints, moving-obstacle points, and frame evidence. Failed/missing gates are `fastlio2_dynamic_inspection`, `moving_obstacle_sweep`, `large_loop_closure`, `gazebo_runtime`, `saved_map_relocalize`, and `pct_saved_map_navigation`.
- Current Windows/Python 3.13 host still cannot prove full DimOS closure. `sim/tests/test_native_pct_mujoco_gate.py` passes locally after lazy NumPy and pure-Python fallback fixes for light geometry helpers, but Fast-LIO/live/saved-map runtime proof still requires Linux/ROS 2 Humble/Python 3.10/PCT native/MuJoCo EGL.
- Saved-map/PCT source binding is now enforced in code: explicit tomograms must match the relocalization report's `map_pcd` sibling `tomogram.pickle`, native PCT reads `assets.map_metadata` for map provenance, and server closure independently checks the relocalize-report `map_pcd` to tomogram path binding.
- `server_sim_closure.py --run-missing` is now dependency-aware: it re-evaluates a gate's generated report after a successful process exit and records downstream gates as `dependency_blocked` when prerequisites are missing, host-blocked, failed, or report-failed in the same run.
- `dimos_gap_report.py` now keeps two separate truths: `highest_priority_blocker` still identifies the highest-risk missing surface, while `next_steps` and shell execution phases are dependency ordered and annotate downstream gates with unmet prerequisite reports.
- `dynamic_obstacle_local_planner` now reports Windows/MINGW NumPy rejection as structured `host_guard` environment evidence instead of only a free-form error string; this preserves the red gate while making the blocker machine-readable.

## DimOS Gap Priority

| Priority | Gap | Evidence status | Why it matters |
| --- | --- | --- | --- |
| P0 | MuJoCo + Fast-LIO2 + native PCT same-source closure | Fresh Linux/ROS2/MuJoCo/PCT-native report exists but is red: live chain partially connects, motion/checkpoint/dynamic-obstacle/frame evidence fails | This is the main "3D planning actually ran" claim. |
| P0 | Dynamic obstacle moving sweep | Gate exists; current gap report says highest blocker is `moving_obstacle_sweep` | DimOS-style simulation must prove replanning/tracking under moving obstacles. |
| P0 | Large-loop live localization closure | Gate exists; needs same-source artifact and loop-error proof | Prevents claiming mapping/localization stability from disconnected artifacts. |
| P1 | Saved-map relocalize -> PCT saved-map navigation | Wrapper/gates exist; missing fresh runtime proof | Required before saying saved-map navigation is connected end to end. |
| P1 | Host preflight as first-class input | CLI support exists; report must be generated and attached before running blocked gates | Prevents false failures and false greens on the wrong machine. |
| P2 | Gazebo/CMU parity and stress cases | Gates exist but are secondary to DimOS core | Useful after the core MuJoCo/PCT/Fast-LIO2 chain is green. |

## File Structure

- Modify: `sim/scripts/server_sim_closure.py`
  - Hard gate live same-source `map.pcd` and tomogram artifact proof.
  - Ensure `saved_map_relocalize` and BBS3D relocalize run-missing commands source ROS 2 and configure MuJoCo headless mode before launching runtime relocalization.
  - Prevent run-missing from launching data-dependent downstream gates until their prerequisite reports are verified.
- Modify: `sim/scripts/pct_saved_map_navigation_gate.py`
  - Bind saved-map navigation to the relocalized map source instead of accepting unrelated explicit tomograms.
- Modify: `sim/scripts/native_pct_mujoco_gate.py`
  - Prefer map provenance metadata over obstacle metadata when extracting same-source artifacts.
  - Keep light geometry, obstacle, lidar-sampling, and omni-tracker helper tests runnable on NumPy-unsafe hosts without claiming runtime closure.
- Modify: `sim/scripts/moving_obstacle_sweep_gate.py`
  - Preserve child live report outputs, map artifacts, and planner provenance.
- Modify: `sim/scripts/large_loop_closure_gate.py`
  - Preserve best-case live report same-source artifacts.
- Modify: `src/core/dimos_runtime_dataflow.py`
  - Cross-gate proof that PCT/MuJoCo and Fast-LIO2 use the same artifacts.
- Modify: `sim/scripts/dimos_gap_report.py`
  - Accept explicit host preflight JSON so local planning and remote runtime proof can be separated.
  - Emit dependency-ordered next steps and dependency-blocked shell comments for downstream gates.
- Modify: `sim/tests/test_server_sim_closure.py`
  - Regression tests that reject boolean-only same-source claims.
- Modify: `sim/tests/test_moving_obstacle_sweep_gate.py`
  - Regression tests that aggregation does not drop same-source artifact fields.
- Modify: `sim/tests/test_large_loop_closure_gate.py`
  - Regression tests that large-loop best-case evidence keeps artifact provenance.
- Modify: `docs/07-testing/DIMOS_BENCHMARK_GAP_MATRIX.md`
  - Document the current claim bar and how to run the gate sequence.

## Task 1: Lock Same-Source Artifact Claims

**Files:**
- Modify: `sim/scripts/server_sim_closure.py`
- Modify: `sim/tests/test_server_sim_closure.py`

- [x] **Step 1: Add evaluator-level artifact proof**

Require `map_artifacts.ok=true`, `source_contract.same_source_pcd=true`, non-empty `assets.map_pcd.sha256`, positive `assets.map_pcd.point_count`, `source_contract.same_source_tomogram=true`, non-empty `assets.tomogram.sha256`, and `assets.tomogram.source_map_sha256 == assets.map_pcd.sha256`.

- [x] **Step 2: Reject missing map hash**

Run:

```bash
python -m pytest sim/tests/test_server_sim_closure.py -q -k "fastlio2_dynamic_inspection or moving_obstacle_sweep or large_loop_closure"
```

Expected: dynamic inspection, moving obstacle sweep, and large-loop closure all reject missing or mismatched same-source artifacts.

## Task 2: Preserve Child Runtime Evidence Through Aggregators

**Files:**
- Modify: `sim/scripts/moving_obstacle_sweep_gate.py`
- Modify: `sim/scripts/large_loop_closure_gate.py`
- Modify: `sim/tests/test_moving_obstacle_sweep_gate.py`
- Modify: `sim/tests/test_large_loop_closure_gate.py`

- [x] **Step 1: Keep full live child report surface**

Preserve `outputs`, `navigation_chain.last_plan_report`, `deliverable_contract`, `map_artifacts`, `assets`, `world`, `inspection_tomogram`, and source-parent fields when aggregating child reports.

- [x] **Step 2: Verify aggregation keeps provenance**

Run:

```bash
python -m pytest sim/tests/test_moving_obstacle_sweep_gate.py sim/tests/test_large_loop_closure_gate.py -q
```

Expected: all aggregation tests pass and same-source fields are still visible in aggregate reports.

## Task 3: Generate Host Preflight Before Runtime Gates

**Files:**
- Modify: `docs/07-testing/DIMOS_BENCHMARK_GAP_MATRIX.md`
- Use: `sim/scripts/dimos_gap_report.py`
- Use: `sim/scripts/server_sim_closure.py`

- [ ] **Step 1: Produce host preflight report on the target Linux/ROS2 runtime**

Run on the Linux/S100P-class runtime, not on the current Windows/Python 3.13 host:

```bash
python sim/scripts/server_sim_closure.py --host-preflight --json-out artifacts/server_sim_closure/host_preflight/report.json
```

Expected: report records ROS 2 Humble, MuJoCo EGL, PCT native ABI, and hardware-safety isolation status.

- [ ] **Step 2: Feed preflight into the DimOS gap report**

Run:

```bash
python sim/scripts/dimos_gap_report.py --include-dataflow --host-preflight-report artifacts/server_sim_closure/host_preflight/report.json --json-out artifacts/server_sim_closure/dimos_gap/report.json
```

Expected: missing gates are separated from environment-blocked gates; `ok_to_run_missing` is true only when preflight is actually green.

## Task 4: Prove Native PCT + MuJoCo + Fast-LIO2 Live Closure

**Files:**
- Use: `sim/scripts/native_pct_mujoco_gate.py`
- Use: `sim/scripts/server_sim_closure.py`
- Use: `src/global_planning/pct_planner_runnable/runtime.py`

- [x] **Step 0: Make native gate contract helpers locally testable**

Keep route metadata, moving-obstacle geometry, lidar sample padding/downsampling, trajectory correctness, local obstacle evidence, and omni-cart target selection testable even when the current host refuses unsafe NumPy imports. This is a local verification aid only; it does not satisfy the runtime gate.

Verified locally:

```bash
python -m pytest sim/tests/test_native_pct_mujoco_gate.py -q
```

Expected/current: 21 passed.

- [ ] **Step 1: Run native PCT gate against generated terrain**

Run:

```bash
python sim/scripts/server_sim_closure.py --preset dimos_benchmark --required native_pct_mujoco --run-missing --strict --json-out artifacts/server_sim_closure/native_pct_mujoco/summary.json
```

Expected: `native_pct_mujoco` passes with `selected_planner=pct`, no fallback, nonzero local path evidence, Mid-360 pattern evidence, and simulation-only safety flags.

- [ ] **Step 2: Run Fast-LIO2 dynamic inspection gate**

Run:

```bash
python sim/scripts/server_sim_closure.py --preset dimos_benchmark --required fastlio2_dynamic_inspection --run-missing --strict --json-out artifacts/server_sim_closure/fastlio2_dynamic_inspection/summary.json
```

Expected: `fastlio2_dynamic_inspection` passes with live MuJoCo lidar/IMU, Fast-LIO2 odometry/map, PCT inspection patrol, moving obstacle evidence, nonzero `nav_cmd_vel`, and artifact-level same-source proof.

- [x] **Step 2.5: Convert current-host large-terrain crash into structured red evidence**

Current Windows/Python 3.13 could not execute `large_terrain_nav_validation.py`
because NumPy import exits before the script reaches PCT diagnostics. The gate
now guards that host before importing NumPy/nav runtime and writes a fresh red
report instead of leaving stale evidence.

Current artifact:

```text
artifacts/server_sim_closure/large_terrain/report.json
```

Current local evidence:

- `execution_mode=host_guard`
- `environment.blocked_reason=windows_mingw_numpy_not_accepted`
- `environment.claim_boundary=environment_blocked_no_algorithm_claim`
- `native_runtime.python_tag=py313`
- existing PCT native modules are Linux/CPython 3.10 `.so` files, not runnable
  on this Windows/Python 3.13 host

The refreshed DimOS gap report is:

```text
artifacts/server_sim_closure/dimos_gap_report_current_after_large_terrain_host_guard.json
```

That Windows artifact remains a local host-guard diagnostic, not target-host
runtime proof. The follow-up Linux target-host work in
`/home/bsrl/hongsenpang/lingtu_dimos_20260608` moved the environment blocker
forward: the earlier `ros2_local_planner` preflight failure was fixed, Fast-LIO2
was built/sourced, and the current sourced preflight
`artifacts/server_sim_closure/host_preflight_dimos_benchmark_after_fastlio2_build_domain75.json`
reports `ok=true`, `failed_checks=[]`, `blocked_gates=[]`, and all 13 required
gates runnable. Separately,
`artifacts/server_sim_closure/pct_runtime_preflight_remote_after_build_report.json`
reports a loadable Linux x86_64 / Python `py310` PCT native runtime.

The latest target-host summary and gap reports are now
`artifacts/server_sim_closure/summary_after_fastlio2_cleanup_domain76_refreshed.json`
and
`artifacts/server_sim_closure/dimos_gap_after_fastlio2_cleanup_domain76_refreshed.json`.
They still report `lingtu_readiness.ok=false`, `claim_allowed=false`, 7/13
required gates passed, and 6 failed or missing. The focused live report
`artifacts/server_sim_closure/mujoco_fastlio2_live/inspection-moving-obstacle-video-20260608_064916/report.json`
proves Fast-LIO clouds/odometry, PCT global planning, local path generation,
and nonzero `/nav/cmd_vel`, but remains red on Fast-LIO-vs-MuJoCo motion
consistency, `0/3` checkpoints, missing moving-obstacle points, and missing
`body_to_camera` frame evidence.

The previous upstream executable blocker was native PCT inside `large_terrain`.
That gate now passes at `artifacts/server_sim_closure/large_terrain/report.json`
with same-source map/tomogram artifacts, `native_runtime.ok=true`, all four PCT
routes using `native_backend_used=true`, `pct_optimizer_enabled=false`,
`pct_planner_path_mode=native_astar_raw_path`, and child return code 0. This is
not A* fallback, but it is also not proof that the GPMP optimizer or MuJoCo
motion loop has run. The target-host `native_pct_mujoco` gate now passes at
`artifacts/server_sim_closure/native_pct_mujoco/report_after_goal_reached_fix.json`
with `selected_planner=pct`, `fallback_used=false`,
`pct_native_backend_used=true`, `path_count=175`, `cmd_count_nonzero=484`,
`moved_m=3.4101`, `final_distance_m=0.4944`, and `reached_goal=true`. That
proves the PCT-to-ROS2-local-planner/path-follower-to-MuJoCo kinematic motion
gate, while Fast-LIO live inspection, moving-obstacle sweep, large-loop
closure, Gazebo runtime, saved-map relocalization, and saved-map PCT navigation
remain red or missing.

Verified locally:

```bash
python -m pytest sim/tests/test_server_sim_closure.py sim/tests/test_dimos_gap_report.py sim/tests/test_pct_saved_map_navigation_gate.py sim/tests/test_native_pct_mujoco_gate.py sim/tests/test_dynamic_obstacle_local_planner_gate.py sim/tests/test_large_terrain_nav_validation_host_guard.py sim/tests/test_large_terrain_scenario.py sim/tests/test_algorithm_dataflow_summary.py src/gateway/tests/test_gateway_route_split.py -q
```

Current result: 278 passed, 4 skipped.

## Task 5: Close Moving Obstacles And Large Loops

**Files:**
- Use: `sim/scripts/moving_obstacle_sweep_gate.py`
- Use: `sim/scripts/large_loop_closure_gate.py`
- Use: `sim/scripts/server_sim_closure.py`

- [ ] **Step 1: Run physical rolling moving obstacle matrix**

Run:

```bash
python sim/scripts/server_sim_closure.py --preset dimos_benchmark --required moving_obstacle_sweep --run-missing --strict --json-out artifacts/server_sim_closure/moving_obstacle_sweep/summary.json
```

Expected: slow/fast and sparse/dense pairs pass, each child has `live_nav_chain.ok=true`, PCT planner provenance, video artifact evidence, and same-source map/tomogram proof.

- [ ] **Step 2: Run large-loop closure**

Run:

```bash
python sim/scripts/server_sim_closure.py --preset dimos_benchmark --required large_loop_closure --run-missing --strict --json-out artifacts/server_sim_closure/large_loop_closure/summary.json
```

Expected: best case uses PCT, physical rolling scan profile, path length over threshold, loop/yaw error under threshold, nonzero local path/cmd_vel, video evidence, and same-source artifact proof.

## Task 6: Prove Saved-Map Navigation

**Files:**
- Use: `sim/scripts/server_sim_closure.py`
- Use: saved-map relocalization runtime reports under `artifacts/server_sim_closure/`
- Modify: `sim/scripts/pct_saved_map_navigation_gate.py`
- Modify: `sim/scripts/native_pct_mujoco_gate.py`
- Modify: `sim/tests/test_pct_saved_map_navigation_gate.py`
- Modify: `sim/tests/test_server_sim_closure.py`

- [x] **Step -1: Fix saved-map relocalize run command environment**

`server_sim_closure.py --run-missing` now launches `saved_map_relocalize_runtime_gate.py` through a ROS 2/MuJoCo headless shell environment instead of a bare local Python command.
`pct_saved_map_navigation` now explicitly consumes `artifacts/server_sim_closure/saved_map_relocalize_runtime/report.json` instead of relying on latest-report discovery.
`run_missing_required_gates()` now treats process exit 0 as necessary but not sufficient: it re-runs the gate evaluator on the generated report before marking the prerequisite usable, and blocks `pct_saved_map_navigation` when `saved_map_relocalize` is not verified.

Verified locally:

```bash
python -m pytest sim/tests/test_server_sim_closure.py -q -k "saved_map_relocalize_command or saved_map_relocalize_next_action or host_preflight or run_missing"
```

- [x] **Step -0.5: Block downstream DimOS gates when prerequisites fail**

`run_missing_required_gates()` now blocks `native_pct_mujoco` and `fastlio2_dynamic_inspection` until `large_terrain` is verified, blocks `moving_obstacle_sweep` and `large_loop_closure` until `fastlio2_dynamic_inspection` is verified, and blocks `pct_saved_map_navigation` until `saved_map_relocalize` is verified.

Verified locally:

```bash
python -m pytest sim/tests/test_server_sim_closure.py -q -k "run_missing_blocks_pct_saved_map_navigation or run_missing_runs_pct_saved_map_navigation or run_missing_blocks_live_dependents or run_missing_executes_missing_required_gate or run_missing_records_host_blocked_gate"
```

Expected/current: 5 passed.

- [x] **Step 0: Lock relocalize map to PCT tomogram source**

Require `pct_saved_map_navigation_gate.py` to derive the default tomogram from `relocalize_report.map_pcd` and reject an explicit tomogram from a different map directory. Require `server_sim_closure.py` to open the relocalize report and verify `tomogram == dirname(map_pcd)/tomogram.pickle`.

Verified locally:

```bash
python -m pytest sim/tests/test_pct_saved_map_navigation_gate.py sim/tests/test_server_sim_closure.py -q -k "saved_map_relocalize or pct_saved_map_navigation or resolve_tomogram"
python -m pytest sim/tests/test_dimos_gap_report.py -q -k "pct_saved_map_navigation or cross_gate or runtime_dataflow"
```

- [ ] **Step 1: Run saved-map relocalization**

Run:

```bash
python sim/scripts/server_sim_closure.py --preset dimos_benchmark --required saved_map_relocalize --run-missing --strict --json-out artifacts/server_sim_closure/saved_map_relocalize/summary.json
```

Expected: localizer uses generated saved-map assets, relocalization succeeds, no hardware command topic is active, and live SLAM dependency is explicit.

- [ ] **Step 2: Run PCT saved-map navigation**

Run:

```bash
python sim/scripts/server_sim_closure.py --preset dimos_benchmark --required pct_saved_map_navigation --run-missing --strict --json-out artifacts/server_sim_closure/pct_saved_map_navigation/summary.json
```

Expected: navigation starts only after relocalization evidence, PCT produces a path from saved map/tomogram, and no A* or direct-goal fallback is counted as success.

## Task 7: Refresh The Single Gap Report

**Files:**
- Use: `sim/scripts/dimos_gap_report.py`
- Modify if needed: `docs/07-testing/DIMOS_BENCHMARK_GAP_MATRIX.md`

- [ ] **Step 1: Rebuild final DimOS gap report**

Run:

```bash
python sim/scripts/dimos_gap_report.py --include-dataflow --host-preflight-report artifacts/server_sim_closure/host_preflight/report.json --json-out artifacts/server_sim_closure/dimos_gap/report.json
```

Expected: `readiness_ok=true`, `runtime_dataflow_ok=true`, no P0 required gates missing, and all artifact-level same-source evidence present.

- [ ] **Step 2: Update the public matrix**

Record the final status in `docs/07-testing/DIMOS_BENCHMARK_GAP_MATRIX.md` with exact report paths, timestamps, and remaining risks.

## 2026-06-08 Target-Host Evidence Update

Current status after implementing the focused MuJoCo live-gate fixes:

- Host preflight is green with `ROS_DOMAIN_ID=83`:
  `artifacts/server_sim_closure/host_preflight_after_motion_window_domain83.json`
  reports all 13 required gates runnable.
- The latest focused inspection report is
  `artifacts/server_sim_closure/mujoco_fastlio2_live_motion_window/inspection-moving-obstacle-video-20260608_074940/report.json`.
  It proves the raw live chain is connected (`runtime_evidence.ok=true`),
  moving-obstacle point injection is non-empty, `body_to_camera` frame evidence
  exists, PCT is selected, local path and nonzero `/nav/cmd_vel` evidence exist,
  and video evidence is written.
- The gate is still red: Fast-LIO motion is `3.278m` vs `1.459m` MuJoCo motion
  (`motion_scale_ratio=2.2468`), angular saturation ratio is `0.5822`, and the
  inspection reaches only `1/3` required checkpoints.
- Fresh summary/gap artifacts:
  `artifacts/server_sim_closure/dimos_benchmark_after_motion_window.json`,
  `artifacts/server_sim_closure/dimos_gap_after_motion_window.json`, and
  `artifacts/server_sim_closure/dimos_gap_after_motion_window.md`.
- With `max_report_age_s=7200`, stale older reports are rejected; current fresh
  status is `lingtu_readiness.ok=false`, `claim_allowed=false`, `0/13` required
  gates passed.

Do not claim `PCT_planner + MuJoCo + Fast-LIO/saved-map` DimOS readiness from
the older 7/13 historical summary. The next implementation lane should fix
Fast-LIO motion consistency and control saturation in
`fastlio2_dynamic_inspection`, then rerun moving-obstacle sweep, large-loop
closure, saved-map relocalization, and PCT saved-map navigation.
