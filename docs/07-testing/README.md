# LingTu Testing and Validation

Status: current validation index as of 2026-09-03.

This directory contains reusable acceptance definitions and dated validation
evidence. It does not own Product behavior, deployment topology, or the active
roadmap.

## Directory

```text
docs/07-testing/
  README.md          this engineering index
  WEB_GUIDE.md       curated reader-facing guide
  field/             reusable real-robot and target-compute gates
  simulation/        reusable simulator gates and fidelity contracts
  field-runs/        immutable date-prefixed evidence records

scripts/gates/
  field/             executable P0 field procedures
```

Commit and push checks are contributor workflow, so they live in
[`docs/03-development/COMMIT_PUSH_POLICY.md`](../03-development/COMMIT_PUSH_POLICY.md).

## Evidence Levels

| Level | Proves | Does not prove |
| --- | --- | --- |
| Local contract | The named schema, algorithm, ownership rule, or fail-closed behavior in focused tests. | Cross-process timing, simulation physics, or robot behavior. |
| Native simulation | The named native component chain in the selected simulator and platform. | Field calibration, networking, or physical safety. |
| Field no-motion | Release identity, process ownership, topic freshness, readiness, and stop barriers on the target. | Permission or evidence for locomotion. |
| Supervised field motion | Only the recorded bounded scenario on the named physical robot. | Untested environments, speeds, maps, or future releases. |

A lower level never implies a higher one. A running process is not the same as
Product readiness, and component evidence is not a Product pass.

## Start Here

| Goal | Entry |
| --- | --- |
| Prepare or validate a physical robot | [Field validation](field/README.md) |
| Run MuJoCo or another simulator gate | [Simulation validation](simulation/README.md) |
| Read previous results | [Validation run records](field-runs/README.md) |
| Understand the general validation ladder | [Reader-facing guide](WEB_GUIDE.md) |
| Check current capability evidence | [Navigation capability matrix](../architecture/NAVIGATION_CAPABILITY_MATRIX.md) |

The Go2 EDU + external MID-360 deployment and assisted-avoidance procedure is
maintained in
[`go2_edu_mid360_teleop_avoid.md`](../04-deployment/go2_edu_mid360_teleop_avoid.md).
Its actual results belong in a new dated record under `field-runs/`.

## Dated Evidence Index

This is the complete record list, newest first. Open a record for its exact
environment, scope, result, blockers, and evidence paths. The evidence files
are immutable; corrections to current guidance belong outside `field-runs/`.

| Date | Record |
| --- | --- |
| 2026-08-12 | [UE5 Playable Pre-Acceptance](field-runs/2026-08-12-ue5-playable-preacceptance.md) |
| 2026-08-11 | [Current Teleop / Teleop-Avoid Evidence Check](field-runs/2026-08-11-current-teleop-avoid-status.md) |
| 2026-08-10 | [MuJoCo Native Teleop Evidence](field-runs/2026-08-10-mujoco-teleop-native.md) |
| 2026-07-27 | [Sunrise Navigation Read-Only Audit](field-runs/2026-07-27-sunrise-navigation-readonly-audit.md) |
| 2026-07-20 | [FAR Native Endpoint aarch64 Validation](field-runs/2026-07-20-worm-aarch64-far-validation.md) |
| 2026-07-20 | [Sunrise S100P Native Validation](field-runs/2026-07-20-sunrise-s100p-validation.md) |
| 2026-07-19 | [Navigation C++ Contract Validation](field-runs/2026-07-19-nav-cpp-contract-validation.md) |
| 2026-07-18 | [Worm aarch64 Native Validation](field-runs/2026-07-18-worm-aarch64-native-validation.md) |
| 2026-07-10 | [LocalPlanner to PathFollower Field Audit](field-runs/2026-07-10-local-planner-path-follower.md) |
| 2026-07-08 | [Sunrise Runtime Migration Status](field-runs/2026-07-08-sunrise-runtime-migration-status.md) |
| 2026-07-07 | [Sunrise Native Nav No-Motion Run](field-runs/2026-07-07-sunrise-native-nav-no-motion.md) |
| 2026-07-06 | [Sunrise Profile Sequential Validation](field-runs/2026-07-06-profile-sequential-validation.md) |
| 2026-07-06 | [MuJoCo Continuous Mapping Quality Gate](field-runs/2026-07-06-mujoco-continuous-mapping-gate.md) |
| 2026-07-05 | [Sunrise Native Nav CmdVel Smoke](field-runs/2026-07-05-sunrise-nav-cmd-vel-smoke.md) |
| 2026-07-04 | [LiDAR to SLAM Field Session](field-runs/2026-07-04.md) |
| 2026-07-04 | [Thunder V4 MuJoCo Policy Baseline](field-runs/2026-07-04-thunderv4-mujoco-policy.md) |
| 2026-07-04 | [Native DDS Field Closure](field-runs/2026-07-04-native-dds-closure.md) |
| 2026-04-17 | [Field Session](field-runs/2026-04-17.md) |

## Focused Local Checks

Run the smallest check that can expose the failure under review. Common entry
points are:

```bash
python tools/validate/validate_docs.py
python -m pytest tests/docs/test_documentation_navigation.py -q
python -m pytest tests/runtime/ -q
```

Subsystem changes should use their owning package build and test instructions.
Do not replace a failed native build with a mock-only result.

## Record A Result

Create one immutable file:

```text
docs/07-testing/field-runs/YYYY-MM-DD-brief-name.md
```

Record the target, platform, Product, env, Product session ID, RunPlan, selected
binaries, exact commands, observed inputs/outputs, PASS/FAIL/BLOCKED result, and
remaining blocker. Label simulation, field-compute, field no-motion, and field
motion explicitly.

## Rules

- Never present local or simulation evidence as field evidence.
- Never use unit-test success to claim a Product capability is delivered.
- Field motion starts only after the applicable no-motion gate passes and an
  operator explicitly begins a supervised scenario.
- Record blocked gates as blocked; do not manufacture samples or timestamps.
- Keep plans in `docs/plans/current-roadmap.md`, current contracts in
  `docs/architecture/`, and one-off results in `field-runs/`.
- When a reusable gate changes, update its owning section index and repair all
  repository links in the same change.
