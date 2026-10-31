# Testing and Evidence

**Status:** Current validation and claim policy
**Audience:** Developers, release engineers, simulation users, and field operators
**Runs on:** Local development, native simulation, target compute, and physical robots

Choose a test for the claim being made. Before running it, state which failure
it can reveal and what changes if it fails.

## Evidence levels

| Level | Proves | Does not prove |
| --- | --- | --- |
| Local contract | Named schema, algorithm, ownership, or failure behavior | Cross-process timing, physics, or robot behavior |
| Native simulation | Named component/Product on that simulator and platform | Field calibration, networking, or physical safety |
| Field no-motion | Release identity, ownership, freshness, readiness, stop barriers | Locomotion |
| Supervised field motion | Recorded bounded scenario on the named robot | Other maps, speeds, environments, or releases |

A running process is not readiness; a component pass is not a Product pass.
Windows and Linux/WSL require independent evidence. Simulation never proves
physical motion safety.

## Local checks

Use the affected unit/contract test first, then only relevant checks:

| Change | Check |
| --- | --- |
| Documentation and Web guide links | `python -m pytest tests/docs/test_documentation_navigation.py -q` |
| Documentation structure and local links | `python tools/validate/validate_docs.py` |
| Generated API inventory | `python tools/docs/extract_api_docs.py --check` |
| Package ownership/imports | `python tools/validate/validate_architecture_boundaries.py` |
| Topic contracts | `python tools/validate/validate_topics.py` |

Native changes also need the owning build/tests. Mock results do not replace a
failed native build. Component test locations are indexed in
[tests/README.md](../tests/README.md).

The [official point-cloud regression](../tools/bench/README.md) retains pinned
OctoPlanner3D PCD inputs, fixed routes, missing-evidence/obstacle/support
rejections, and native path verification. Run it when changing saved-point
conversion or global planning. It is offline algorithm evidence; its inferred
free-space fixture does not qualify field map construction or robot motion.

## Product acceptance

Qualify the [delivery workflow](product.md#完整交付流程) through one
ProductControl transaction and its recorded identity:

```text
ProductControl switch (Product + robot + env)
  -> product_session_id
  -> declared processes/streams -> readiness -> scenario
  -> terminal zero -> cleanup and rollback evidence
```

A dry run, manual process assembly, or separate component runs cannot satisfy
this gate. Save completion, fresh localization after restart, goal outcome,
and stopped output must belong to the workflow being claimed.

## Simulation gates

Record platform, Product/component scope, package/session, scenario, clock or
generation, localization provider, and artifacts. The default `teleop_avoid`
component manifest uses simulated LiDAR/IMU through native LTLIO, Mapd,
SCAN, final arbitration, and the MuJoCo driver. Explicit
`mujoco_navigation_fixture` truth pose/ground isolates navigation; it cannot
qualify LTLIO, raw sensor-to-map behavior, or real robot calibration.

Observe final commands, odometry, physical motion, and terminal zero, not just
a planned path. RobotSimUE adds presentation/render-sensor evidence; Editor
runs do not qualify Shipping builds. Real MID-360 data is required for new
field terrain or hardware-specific claims. See [Simulation](simulation.md).

## MuJoCo stair regression

For SCAN obstacle diagnostics, `config/acceptance/mujoco/local_scan_lidar.json`
publishes only simulated MID-360 ray returns to the navigation cloud; it does
not add the fixture's synthetic flat ground. Pair it with a saved ray map of
the same scene using `--world`, `--map-dir`, and `--no-prepare-assets`. Without
an explicit map, it inherits the synthetic saved-map builder from
`local_scan.json`. Pose and localization health are still MuJoCo truth, so
this is not a SLAM or real-robot test.

The experimental `thunderv4_stairs` session reuses the twelve-step showcase
geometry, with a 0.005 s physics step. Its controller binds 4998 for ordinary
travel and the supplied CTS MoE v39 model 61128 for both stair directions;
these bindings are test inputs, not qualified stair profiles for a real robot.
Normal Product presets are unchanged. Select a different simulation session
through ProductControl's `env_config.session` (CLI `--sim-session`), so the
catalog is resolved once inside the usual RunPlan.

```powershell
.venv/Scripts/python.exe -m sim.scripts.mujoco.stair_policy_probe --help
.venv/Scripts/python.exe -m sim.scripts.mujoco.native_navigation_acceptance `
  --manifest config/acceptance/mujoco/stairs.json `
  --product-map <saved-stair-map-directory> --out-dir <new-evidence-directory>
.venv/Scripts/python.exe -m sim.scripts.mujoco.native_navigation_acceptance `
  --manifest config/acceptance/mujoco/stairs_down.json `
  --product-map <saved-stair-map-directory> --out-dir <separate-descent-directory>
```

The sequence stops if ascent fails. The separate descent manifest starts on the
upper landing so a failed ascent does not leave descent untested. Run the two
Product sessions sequentially.

Use a saved ray map of that same world with native recognized stair annotations.
Report whether the map came from virtual sensor poses, a walking robot, or SLAM;
an ideal noise-free virtual survey is only an execution-chain input. Do not add
synthetic floor support to make the route pass.

Record ascent and descent separately, the actual loaded model before/after each
handoff, XYZ arrival, body/leg collisions and terminal stop. Foot/wheel support
on a tread is distinct from a body impact or a foot hitting its vertical riser.
A successful fixed-model ascent, a flat-ground model switch, and an independent
exit restoration cannot be combined into a claim that the entire stair route
passed. MuJoCo policy swapping also does not validate a physical DOSO Grounded
transition or a Unitree firmware mode API.

The 2026-10-12 final diagnostic runs (`native-sequence-v14` ascent and
`native-descent-v6` descent, separate 120 s manifests) both remain unqualified:
they confirmed actual 4998-to-1119 entry changes, but reached neither exit nor
original-model restoration. LiDAR scheduler drops and intermittent input readiness
also failed. The earlier 90 s descent v5 made continuous progress near its deadline;
retain that result separately. The final heading/collision contract tests pass,
but do not establish physical traversal. `stop_verified` here records zero-command
acknowledgment and process shutdown, not measured physical settling. The reports,
source snapshot and task-window plots are under `build/stairs-mujoco-20261012/`.

The current experimental
[`thunderv4_cts_moe` controller](../sim/packages/controllers/doso/thunder_v4/cts_moe/README.md)
selects v39 model 61128. Its adapter has been checked against the supplied
runtime, but it has not yet qualified complete navigation and exit restoration.
The earlier `student_80708.pt` observations under `build/cts-moe-20261012/`
and the 1119 Product runs above are historical results, not tests of v39.
These isolated locomotion probes do not replace native Product acceptance.

## Repeated inspection qualification

The multi-round manifest
[`inspection_three_loops.json`](../config/acceptance/mujoco/inspection_three_loops.json)
inherits the ordinary three-point route and requests three rounds with a
two-second interval. The inspection acceptance runner compares nine ordered
`(loop_index, point_index)` capture records and native point-completion facts.
It also requires the native waiting state between rounds. Coordinates are
fixed before starting and cannot move between rounds to improve the result.

The runner attaches to the declared inspection RunPlan and normal Gateway
start path. Native recording is required to retain event continuity, final
zero output and its matching driver acknowledgement. A local recorder
start/stop test proves that component; a dry-run proves resolution only.
Candidate RuntimeGraph injection must be named as an experiment and cannot
qualify the unmodified public configuration.

Qualify pause/resume and endpoint-restart continuation separately, with the
same task, map and route revision. After a restart, no goal or evidence request
may be sent before explicit resume. Already committed points must not be
captured twice or counted as unvisited; a skip must remain a skip.

Evidence acceptance verifies capture and retained records. It does not prove
the equipment is normal. Indicator/door accuracy requires labelled equipment
images and a real configured image-capable service; mocked readings and local
HTTP fixtures prove wiring only. An absent or ambiguous observation remains
inconclusive. Human review must retain the original observation and its history.
Manual charging after reaching a standby point does not establish automatic
charging or unattended operation.

## Field no-motion order

1. Resolve the exact Product and confirm target, release, network, map, and
   calibration identity.
2. Start through ProductControl; check unique process/topic owners and fresh
   sensors, localization, map, navigation, and driver state.
3. Prove stop barriers and zero output; record the result before motion.

```bash
bash scripts/lingtu status
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
PYTHONPATH=src python -m diagnostics.field.soak --duration 120 --interval 2 --json --strict
```

Executable procedures live under `scripts/gates/field/`; deployment and
recovery instructions are in [Operations](operations.md).

## Supervised motion

Use a separately approved bounded scenario after no-motion gates pass. The
operator needs the map, route, speed bounds, clearance, stop mechanism, and
terminal condition. Record final zero output and driver acknowledgement.
An API ACK proves admission, not actuator behavior.

## Evidence record

Store logs, JSON, MCAP, images, and reports in `artifacts/` or an external
store. Include revision/release, Product/robot/env, Product session, platform,
scenario/map/configuration, commands, observed result, PASS/FAIL/BLOCKED, and
the remaining blocker. Link the bundle from the release, issue, or review.
When diagnosing a lifecycle transaction, the internal execution snapshot may
also be recorded; it is evidence for that ProductControl run, not a separate
acceptance stage.
Existing field handoffs retain their date and scope; they do not supersede
current configuration or substitute for a fresh acceptance run.

Keep each run in its own directory. Put build intermediates and test caches
in `build/`; archive completed evidence bundles outside the checkout without
separating reports from their data. Do not discard the only copy of a map,
model, calibration, or referenced field result. See the
[artifact storage policy](../artifacts/README.md).

## Algorithm result comparisons

As requested on 2026-10-21, present other algorithms alongside LingTu when
reporting algorithm results. Include a matched previous LingTu version and an
independently executed upstream or external baseline where evidence exists.
Mark missing external results as not run; do not silently omit them or substitute
an internal adaptation for the author's implementation. Published paper numbers
belong in a separate reference table, with their input and evaluation differences.

Use the same frozen recording and eligible reference timestamps for localization
comparisons. Show total and height error, coverage/loss and runtime scope together.
Preserve both reference-frame hypotheses when certification is unresolved. An
author's full configuration versus LingTu's configuration is a system comparison;
it does not isolate a single mechanism. Keep module ablations labelled separately.

For planning, freeze the same map, requests, robot geometry, unknown-space and
support policy, endpoint handling, and motion limits before comparing algorithms.
Report route return, independent executable geometry, local trajectory generation
and actual task completion separately. Different defaults or map resolutions may
be shown as a historical system comparison, but not as a controlled algorithm
ranking. Saved-ray maps and modeled-free PCD maps are distinct evidence sources.

Retain the data tables, exact source reports and plotting program beside each
figure. Failure and unmeasured cells remain visible. Paper figures use Arial
10 pt. The [current comparison record](../build/algorithm-comparison-20261021/README.md)
separates official Elevator-LIO/GenZ-LIO, LingTu revisions, historical PCT and
OctoPlanner3D results, and the missing same-condition planning baselines.

## Current claim boundary

The repository has local contracts and selected native simulation gates.
Autonomous field readiness requires fresh target, no-motion, fault-injection,
and supervised-motion evidence for each claimed robot and release. Remaining
gates are in [Roadmap](roadmap.md).
