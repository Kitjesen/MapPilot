# Field Run Records

Status: evidence index. Files in this directory are dated field or
field-compute observations; they do not replace current architecture,
deployment, or testing contracts.

Store dated real-robot or field-compute validation notes here.

Use one file per run:

```text
YYYY-MM-DD-brief-name.md
```

Each record should include:

- robot or host
- active profile, endpoint, `session_mode`, and `product_session`
- active map path when relevant
- services that were running
- command or API used
- PASS / FAIL / BLOCKED result
- blockers and next action

Do not keep project backlog state here. Current product targets live in
`docs/plans/current-roadmap.md`.

When a dated note conflicts with a current contract, keep the note unchanged as
historical evidence and update the active contract/index outside this folder.

## Recent records

| Date | Record | Scope | Result |
| --- | --- | --- | --- |
| 2026-07-20 | [sunrise-s100p-validation](2026-07-20-sunrise-s100p-validation.md) | Real S100P preflight, isolated aarch64 native build, DDS map input chain, and MID-360 link check | PARTIAL: software gates pass; sensor and control gates open |
| 2026-07-20 | [worm-aarch64-far-validation](2026-07-20-worm-aarch64-far-validation.md) | FAR and native exploration lifecycle build/test gates on aarch64 | PASS within stated boundary |
| 2026-07-18 | [`worm-aarch64-native-validation`](2026-07-18-worm-aarch64-native-validation.md) | Isolated aarch64 build, native CTest, DDS dataflow, and no-motion endpoint startup | PASS within stated boundary |
