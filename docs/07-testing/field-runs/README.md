# Validation Run Records

Status: dated evidence index. The directory name is retained for historical
continuity; records may cover field robots, target compute, or simulation when
the environment is stated explicitly.

These records do not replace current architecture, deployment, or testing
contracts.

Use one file per run:

```text
YYYY-MM-DD-brief-name.md
```

Each record should include:

- evidence level: simulation, field-compute, field no-motion, or supervised
  field motion
- robot, simulator, or host
- active Product, env, Product session ID, endpoint, and `session_mode`
- active map path when relevant
- services that were running
- command or API used
- PASS / FAIL / BLOCKED result
- blockers and next action

Do not keep project backlog state here. Current product targets live in
`docs/plans/current-roadmap.md`.

When a dated note conflicts with a current contract, keep the note unchanged as
historical evidence and update the active contract/index outside this folder.

## Reusable Gates

Reusable procedures live outside this evidence directory:

- [Field gates](../field/README.md)
- [Simulation gates](../simulation/README.md)

## Recent records

| Date | Record | Scope | Result |
| --- | --- | --- | --- |
| 2026-08-11 | [current-teleop-avoid-status](2026-08-11-current-teleop-avoid-status.md) | Repository, Web teleop, local native build, MuJoCo teleop/teleop_avoid audit, and Sunrise reachability | PARTIAL: Web teleop entry and local Product contract pass; teleop_avoid Product acceptance still open; Sunrise SSH auth blocks inspection |
| 2026-08-10 | [mujoco-teleop-native](2026-08-10-mujoco-teleop-native.md) | Native typed teleop, simulated physical driver ACK, stop, and cleanup | PASS within diagnostic component scope; not a Product or Brainstem field pass |
| 2026-07-27 | [sunrise-navigation-readonly-audit](2026-07-27-sunrise-navigation-readonly-audit.md) | Read-only deployed navigation, map, and process ownership audit | Evidence only; see record for blockers |
| 2026-07-20 | [sunrise-s100p-validation](2026-07-20-sunrise-s100p-validation.md) | Real S100P preflight, isolated aarch64 native build, DDS map input chain, and MID-360 link check | PARTIAL: software gates pass; sensor and control gates open |
| 2026-07-20 | [worm-aarch64-far-validation](2026-07-20-worm-aarch64-far-validation.md) | FAR and native exploration lifecycle build/test gates on aarch64 | PASS within stated boundary |
| 2026-07-18 | [`worm-aarch64-native-validation`](2026-07-18-worm-aarch64-native-validation.md) | Isolated aarch64 build, native CTest, DDS dataflow, and no-motion endpoint startup | PASS within stated boundary |

Older records remain for traceability but are intentionally not summarized
here. Simulation evidence must not be presented as a robot run, and target-
compute evidence without physical motion must not be presented as a motion pass.
