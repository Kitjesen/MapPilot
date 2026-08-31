# LingTu Testing and Validation

Status: current validation index as of 2026-08-23.

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
  simulation/        executable simulation compatibility gates
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

## Focused Local Checks

Run the smallest check that can expose the failure under review. Common entry
points are:

```bash
python tools/validate/validate_docs.py
python -m pytest tests/docs/test_documentation_navigation.py -q
python -m pytest src/runtime/tests/ -q
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
