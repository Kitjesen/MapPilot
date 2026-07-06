# LingTu Regression Suite

Current layers, from local commit to server/simulation closure. Hardware P0
scripts are retained as optional legacy procedures, not the current target.

Commit and push acceptance criteria live in
[`COMMIT_PUSH_POLICY.md`](COMMIT_PUSH_POLICY.md). The short version is:
commit messages follow the Lore protocol in `AGENTS.md`, local commits must pass
the L1 test gate, and pushes must pass the L2 gate plus any focused checks for
the touched subsystem.

Current P0 readiness evidence and remaining product-claim blockers are tracked
in [`P0_READINESS_AUDIT.md`](P0_READINESS_AUDIT.md).
The strict navigation algorithm claim boundary is defined in
[`ALGORITHM_VALIDATION_FLOW.md`](ALGORITHM_VALIDATION_FLOW.md).
Simulation boundaries are defined in
[`../architecture/SIMULATION_INTEGRATION_CONTRACT.md`](../architecture/SIMULATION_INTEGRATION_CONTRACT.md):
LingTu absorbs simulator topic contracts, not simulator product profiles.

| Layer | Trigger | Content | Time | Required |
|---|---|---|---|---|
| L1 pre-commit hook | `git commit` | `pytest src/runtime/tests/ -q` must pass | ~90 s | Yes |
| L2 pre-push hook | `git push` | L1 plus `stub` profile build/start smoke | ~30 s extra | Yes |
| L2.5 server simulation closure | Before navigation demos or simulation-backed claims | `server_sim_closure.py` strict summary over the relevant simulation gates | Host-dependent | Yes for simulation-backed navigation claims |
| L3 hardware P0 | Explicit hardware campaign only | Run the P0 scripts and capture video | ~30 min | No for current server/sim work |

---

## Install local L1 / L2 hooks

```bash
# One-shot install into .git/hooks/ (not version-controlled)
bash docs/07-testing/install_hooks.sh
```

After install:
- `git commit` runs `pytest src/runtime/tests/ -q`; the commit aborts on failure.
- `git push` runs pytest plus a stub profile graph smoke and a minimal offline stub lifecycle smoke.

Bypass (emergencies only):

```bash
git commit --no-verify -m "..."
git push --no-verify
```

The PR description must state the reason; the reviewer will challenge it.

---

## L2.5 server simulation closure

Use this layer before claiming that navigation, localization, planning,
tracking, exploration, or Gateway command safety works beyond unit tests. It is
hardware-free evidence only: it must report `simulation_only=true`,
`real_robot_motion=false`, and `cmd_vel_sent_to_hardware=false`.

The current strict live Fast-LIO simulation input default is
`scan_time_profile=physical_rolling`, which accumulates actual MuJoCo subscans
over the MID-360 scan window instead of applying offsets to a single snapshot.
Physical-rolling fixed motion and one-goal inspection evidence is green, but a
full algorithm claim still requires refreshed large-loop closure and dynamic
obstacle speed/density video gates under the same profile.

Full closure summary:

```bash
bash docs/07-testing/l25_fresh_closure.sh
```

The summary is an aggregator. If it reports `missing_or_failed`, run the
command printed for each failed gate, then rerun the summary.
When a command uses `--required` to validate only a subset, non-required
failures appear as `optional_missing_or_failed`; that output is useful setup
evidence but not a full closure pass.
Each accepted report includes `report_mtime` and `report_age_s`. The wrapper
enforces `--max-report-age-s`, defaulting to 21600 seconds; override with
`LINGTU_L25_MAX_REPORT_AGE_S` only when the review explicitly accepts older
simulation evidence.

The full gate set is designed to cover:

- multi-floor exploration, LiDAR localization contract, native PCT, local planning, and nav_kernel tracking;
- large-terrain global planning and path-safety checks;
- native PCT through in-process local planner/path follower into MuJoCo motion;
- dynamic-obstacle local planner replanning;
- live MuJoCo LiDAR/IMU through Fast-LIO2 and SlamBridge;
- ONNX policy navigation dataflow;
- Gateway dry-run command safety;
- routecheck preflight without publishing goal, cmd_vel, or stop;
- ROS-native Gazebo frame/topic smoke;
- isolated CMU Humble Unity/TARE benchmark preflight plus LingTu adapter contract;
- saved-map relocalization contract.

Server bootstrap verification uses the setup-safe subset it produces itself:

```bash
bash scripts/deploy/setup_server_ros_pct.sh
```

That script writes `artifacts/server_sim_closure_summary_setup.json` for the
multi-floor and routecheck-preflight gates. It is not a substitute for the full
closure summary unless those are the only changed surfaces. The setup summary
uses `--required-only` plus `--max-report-age-s`, defaulting to 21600 seconds;
override with `LINGTU_SETUP_CLOSURE_MAX_REPORT_AGE_S` when needed.

Operators can also inspect the latest routecheck artifact through Gateway:

```bash
curl -s http://127.0.0.1:5050/api/v1/diagnostics/routecheck/latest | jq .
```

The response exposes `report_mtime`, `report_age_s`, `non_motion`,
`simulation_only`, `real_robot_motion`, `cmd_vel_sent_to_hardware`, and the
`published` counters at the top level so stale or motion-producing evidence is
visible without parsing the nested summary.

---

## Optional Hardware P0 Scripts

These scripts are not part of the current server/simulation target. Use them
only when a real hardware campaign is explicitly scheduled.

Field-run session notes (bugs, bag paths, BACKLOG links) live in
[`field-runs/`](field-runs/README.md) — one `YYYY-MM-DD.md` per on-robot session.

Each P0 script is self-contained, uses `set -e`, and writes its log to `~/data/nav_logs/YYYYMMDD_HHMMSS_<script>.log`.

| Script | Coverage | Input | Pass criteria |
|---|---|---|---|
| `p0_cold_boot.sh` | System cold start | None | At least 20 modules up, Gateway `/api/v1/health` returns `status="ok"` and is stable for 3 minutes |
| `p0_mapping.sh` | Map -> save -> activate | Walk the robot for ~3 min | MapManager `/api/v1/maps` save and `set_active` succeed; `map.pcd` + `tomogram.pickle` + `occupancy.npz` + `map.pgm` / `map.yaml` are produced |
| `p0_route_safety.sh` | No-motion route safety | Pre-loaded active map and idle robot | `/api/v1/navigation/plan` returns a feasible path with `path_safety.ok=true` and the active command source remains unchanged |
| `p0_goto.sh` | Point goal -> arrive | Pre-loaded active map plus explicit `GOAL_X GOAL_Y`; route preview must pass and operator must type `RUN` | `/api/v1/navigation/status` reports `state="SUCCESS"` |
| `p0_estop.sh` | Gateway stop reflex | Operator starts non-zero robot motion, then presses Enter in the script | `POST /api/v1/stop` succeeds and `/api/v1/state` reports navigation speed below `0.01 m/s` for 3 consecutive ticks within 2 seconds |
| `p0_explore.sh` | Exploration start/stop | `explore` or `tare_explore` profile in a safe open area | `/api/v1/explore/status` is ready, `/api/v1/explore/start` reports active exploration, and `/api/v1/explore/stop` leaves `exploring=false` |

### Legacy Hardware Procedure

```bash
ssh sunrise@192.168.66.190
cd ~/data/SLAM/navigation
git pull --ff-only origin main
LINGTU_P0_GOAL_X=2.0 LINGTU_P0_GOAL_Y=0.0 bash docs/07-testing/p0_all.sh
```

The aggregate script starts Gateway sessions through `/api/v1/session/start`:
`mapping` with `slam_profile=fastlio2`, then `navigating` with
`slam_profile=localizer` and the map saved by P0-02. It refuses to choose a
motion target automatically; set `LINGTU_P0_GOAL_X` and `LINGTU_P0_GOAL_Y`, then
type `RUN` after the no-motion route preview shows a safe path.

Exploration is opt-in for the aggregate script because it requires a dedicated
runtime profile and an open test area:

```bash
python lingtu.py explore --daemon   # or start the systemd unit with explore/tare_explore
bash docs/07-testing/p0_explore.sh 30

LINGTU_P0_RUN_EXPLORE=1 LINGTU_P0_EXPLORE_DURATION=30 \
  LINGTU_P0_GOAL_X=2.0 LINGTU_P0_GOAL_Y=0.0 \
  bash docs/07-testing/p0_all.sh
```

After the run:
1. Record `PASS / FAIL / BLOCKED` plus reason in `docs/07-testing/field-runs/YYYY-MM-DD.md`.
2. Update `docs/plans/current-roadmap.md` when a blocker changes priority or is cleared.
3. Failure cases get their own issue or a dated note under `docs/07-testing/field-runs/`.

---

## Simulation-only policy soak on Sunrise

Use Sunrise as a compute host for MuJoCo policy validation only. This gate must
not start robot services, publish real hardware commands, or run the hardware
`scripts/lingtu map|nav` commands.

Run it after changing MuJoCo assets, ONNX policy loading, `PolicyRunner`,
`MujocoDriverModule`, or sim navigation wiring:

```bash
ssh sunrise@192.168.66.190
cd ~/data/SLAM/navigation
mkdir -p artifacts
PYTHONPATH=src:. PYTHONIOENCODING=utf-8 LINGTU_SIM_DRIVE_MODE=policy \
  python sim/scripts/policy_nav_smoke.py \
    --world open_field \
    --direct-duration 6 \
    --nav-duration 60 \
    --goal-distance 1.0 \
    --json-out artifacts/policy_nav_smoke_open_field.json
```

Expected pass evidence: policy metadata shows `obs_history [1, 285]` to
`actions [1, 16]`, direct and full-stack nav motion both exceed the configured
thresholds, `nav_state` reaches `SUCCESS`, and `direct_fallback` remains zero.
The detailed runbook is in `sim/README.md`.

---

## Rules

- Never run validation commands ad hoc over SSH; always go through a script.
- Never bypass pre-commit / pre-push without an emergency rationale in the PR.
- New PRs that change product behavior must name the affected product mode and validation gate.
- Blocked field items must be re-evaluated in the next dated field-run note.

---

## Script inventory

- `install_hooks.sh` - install L1 / L2 git hooks.
- `l25_fresh_closure.sh` - L2.5 strict closure summary with freshness enforcement.
- `p0_cold_boot.sh` - P0-01 cold start.
- `p0_mapping.sh` - P0-02 mapping.
- `p0_route_safety.sh` - P0-03 no-motion route safety preview.
- `p0_goto.sh` - P0-04 navigate to a point.
- `p0_estop.sh` - P0-05 emergency stop.
- `p0_explore.sh` - P0-06 exploration start/stop, opt-in for aggregate runs.
- `p0_all.sh` - chain P0-01 through P0-05, with optional P0-06.

The single tracked baseline document remaining in this folder is `SUPERVISION_BENCHMARK_BASELINE_2026-04-05.md` (frozen for the supervision tracker migration).
