# P0 Readiness Audit

Last updated: 2026-05-12

Superseded status, 2026-06-08: this audit is not a current PASS record. The
current DimOS simulation-closure artifacts are
`artifacts/server_sim_closure/summary_after_native_pct_goal_fix.json` and
`artifacts/server_sim_closure/dimos_gap_after_native_pct_goal_fix.json`; they
report `lingtu_readiness.ok=false`, `claim_allowed=false`, 7/13 required gates
passed, and 6 failed or missing. `native_pct_mujoco` now passes, but
Fast-LIO live inspection, moving-obstacle sweep, large-loop closure, Gazebo
runtime, saved-map relocalization, and saved-map PCT navigation remain red or
missing.

This audit separates implementation readiness from completion evidence. It is
not a PASS record for robot navigation. The active validation target is
server-side full simulation only, with no real robot motion. S100P field
evidence remains listed as a later product-claim boundary.

## Completion Criteria

LingTu can be claimed as P0 field-ready only when all of these are true:

| Criterion | Evidence required | Current status |
|---|---|---|
| P0 scripts use current Gateway contracts | Static contract tests and script syntax checks | PASS |
| Field motion never uses implicit goals | `p0_all.sh` and `p0_goto.sh` require explicit goal input | PASS |
| Route preview gates motion | Route preview passes before `RUN`, and `RUN` precedes `/api/v1/goal` | PASS |
| E-stop script fails safe after operator starts motion | Post-prompt failure paths send `/api/v1/stop` cleanup | PASS |
| L2.5 server simulation closure is fresh | Current DimOS 13-gate closure must return `lingtu_readiness.ok=true` and `claim_allowed=true` | RED: current 2026-06-08 result is 7/13 with `claim_allowed=false` |
| S100P mapping run passes | `p0_mapping.sh` field log plus saved map artifacts | MISSING |
| S100P route safety run passes | `p0_route_safety.sh` field log shows no-motion safe route | MISSING |
| S100P goto run passes | `p0_goto.sh` field log reaches `SUCCESS` | MISSING |
| S100P e-stop run passes | `p0_estop.sh` field log proves stop from non-zero speed | MISSING |
| S100P exploration run passes | `p0_explore.sh` field log starts/stops exploration | MISSING |

## Current Local Evidence

Recent commits:

- `1330266b Document simulation readiness evidence boundary`
- `ff2f9fd9 Keep L2.5 closure wrapper runnable`
- `5fdda01a Harden P0 motion confirmation gates`
- `f59d042d Harden P0 field flow safety`

Validated locally:

```bash
python -m pytest src/core/tests/test_server_setup_contract.py -q --tb=short
bash -n docs/07-testing/l25_fresh_closure.sh
bash -n docs/07-testing/p0_all.sh docs/07-testing/p0_goto.sh docs/07-testing/p0_estop.sh docs/07-testing/p0_route_safety.sh docs/07-testing/p0_mapping.sh docs/07-testing/p0_explore.sh docs/07-testing/p0_cold_boot.sh
git diff --check
```

The L1 pre-commit hook also passed full `python -m pytest src/core/tests/ -q`
for the two most recent commits. The hook prints known nanobind leak diagnostics
but exits with `[L1 pre-commit] OK`.

## Historical Server-Side L2.5 Closure Snapshot

Authoritative server:

- Host: `gpu8x3090`
- Workspace: `/home/bsrl/hongsenpang/lingtu_closure_22f127b2`
- Summary artifact:
  `artifacts/server_sim_closure/summary_current_codex.json`

Latest strict summary command run on the server:

```bash
PYTHONPATH=src:. python3 sim/scripts/server_sim_closure.py \
  --json-out artifacts/server_sim_closure/summary_current_codex.json \
  --strict
```

Historical result: `ok=true`, `simulation_only=true`,
`real_robot_motion=false`, `cmd_vel_sent_to_hardware=false`,
`missing_or_failed=[]`, `remaining_gaps=[]` under the older gate set.

Current 2026-06-08 DimOS result:
`artifacts/server_sim_closure/summary_after_native_pct_goal_fix.json` and
`artifacts/server_sim_closure/dimos_gap_after_native_pct_goal_fix.json` report
`lingtu_readiness.ok=false`, `claim_allowed=false`, 7/13 required gates passed,
and 6 failed or missing.

Historical strict PASS gates:

- `dynamic_obstacle_local_planner`
- `fastlio2_live`
- `gateway_dry_run`
- `gazebo_runtime`
- `large_terrain`
- `multifloor_exploration`
- `native_pct_mujoco`
- `policy_nav`
- `saved_map_relocalize`

Server evidence covers rich terrain, multi-floor exploration, LiDAR/SLAM
localization and tracking, global route planning, local dynamic-obstacle
avoidance, policy navigation, saved-map relocalization, Gateway dry-run flow,
and simulation-only safety boundaries. The strict closure explicitly reports no
real robot motion and no hardware `cmd_vel` publication.

Local workstation note:

```text
PowerShell interpreter: Python 3.13 on AMD64
WSL interpreter: Python 3.12 on x86_64, with no numpy installed
PCT native artifacts observed locally: cp310/aarch64 and cp310/x86_64 files
Missing runtime: runnable PCT native modules for the active local Python ABI
```

Those local ABI/dependency gaps are not authoritative product evidence. The
active validation target is the server-side full simulation closure above.

## Field Evidence Still Required

Run on S100P after deploying the current branch:

```bash
LINGTU_P0_GOAL_X=<safe_x> LINGTU_P0_GOAL_Y=<safe_y> bash docs/07-testing/p0_all.sh
```

For exploration:

```bash
python lingtu.py explore --daemon
bash docs/07-testing/p0_explore.sh 30
```

Required artifacts:

- `~/data/nav_logs/*_p0_cold_boot.log`
- `~/data/nav_logs/*_p0_mapping.log`
- `~/data/nav_logs/*_p0_route_safety.log`
- `~/data/nav_logs/*_p0_goto.log`
- `~/data/nav_logs/*_p0_estop.log`
- `~/data/nav_logs/*_p0_explore.log`
- Saved map directory containing `map.pcd`, `tomogram.pickle`,
  `occupancy.npz`, and `map.pgm` / `map.yaml`
- Field run entry under `docs/07-testing/field-runs/YYYY-MM-DD.md`

## Product Claim Boundary

Current claim allowed:

```text
Server-side full simulation closure passes for navigation, obstacle avoidance,
LiDAR/SLAM localization, tracking, planning, and exploration with no real robot
motion and no hardware cmd_vel publication. The P0 field validation scripts are
implemented and locally guarded by contract tests.
```

Current claim not allowed:

```text
The robot has proven autonomous navigation, obstacle avoidance, and exploration
on S100P.
```

That claim requires S100P P0 field PASS evidence.
