# LingTu Docs Triage

Status: current cleanup decision
Date: 2026-06-30

This document decides how the docs tree should be read and cleaned. It is a
triage map, not another architecture contract.

## 1. Rules

- Current behavior lives in `README.md`, `CURRENT.md`, `architecture/*CONTRACT.md`,
  `QUICKSTART.md`, `REPO_LAYOUT.md`, and package-local READMEs.
- Plans live in `plans/`. A plan is not current behavior.
- Evidence lives in `07-testing/`. Evidence is date-bound.
- Historical or branch-specific material lives in `archive/`.
- If a document has mojibake, branch names, old robot names, or old ROS-first
  assumptions, it is not authoritative until rewritten.

## 2. Root Docs

| File | Decision | Reason |
| --- | --- | --- |
| `README.md` | keep current | Main entrypoint. |
| `CURRENT.md` | keep current | One-page authority map. |
| `QUICKSTART.md` | keep, needs cleanup | Useful operator doc, but has old references and mojibake. |
| `REPO_LAYOUT.md` | keep current | Placement rule source. |
| `TUNING.md` | keep current | Current tuning summary. |
| `known_gaps.md` | keep current | Current product gaps. |
| `engineering_boundaries_phase1.md` | demote to plan/archive candidate | Phase-1 migration note, mostly superseded by current contracts. |
| `REVIEW_2026Q2.md` | archive candidate | Historical branch review, contains mojibake and branch-specific claims. |

## 3. Architecture Docs

| File | Decision | Reason |
| --- | --- | --- |
| `README.md` | keep current | Architecture directory index. |
| `SYSTEM_DESIGN.md` | keep current | Paper-style current architecture. |
| `GLOBAL_PLANNING_CONTRACT.md` | keep current | Global planner boundary and UI/transport payload. |
| `NAVIGATION_COMPUTE_CONTRACT.md` | keep current | Navigation compute chain. |
| `local_planner_io_contract.md` | keep current | Local planner I/O and known gap. |
| `LINGTU_RUNTIME_BUS_DECISION.md` | keep current | Port/Wire/Transport decision. |
| `ros_frame_contract.md` | keep current | Frame contract. |
| `semantic_layer_contract.md` | keep current, review later | Small contract; still useful. |
| `SIMULATION_INTEGRATION_CONTRACT.md` | keep current, review later | Current sim boundary, but long and likely needs condensation. |
| `ROS_ROLE_REPLACEMENT_MAP.md` | migration reference | Useful during ROS replacement, not core architecture entry. |
| `DART_RUST_PACKAGE_MIGRATION.md` | plan/proposal | Future UI/package direction. |
| `PORTABLE_LEAN_PACKAGE_MATRIX.md` | plan/proposal | Packaging decision support, not runtime contract. |
| `POSE_GRAPH_OPT_GTSAM_COVERAGE.md` | evidence/reference | Large coverage note, not architecture entry. |
| `THUNDER_RUNTIME_REFACTOR_PLAN.md` | plan/proposal | Refactor plan. |
| `TRAVEXPLORER_LINGTU_ADOPTION.md` | plan/reference | Adoption note, not current core contract. |

## 4. Plans

`docs/plans/` now keeps only active forward-looking work. Retired PRDs and
historical plans were removed; use git history when old context is needed.

| File | Decision |
| --- | --- |
| `current-roadmap.md` | current product/runtime roadmap and remaining validation gates |

## 5. Testing Docs

| Area | Decision |
| --- | --- |
| `07-testing/README.md` | current validation entrypoint |
| `07-testing/*AUDIT*.md` | evidence snapshot, date-bound |
| `07-testing/*GATE*.md` | gate definitions or evidence, verify before citing |
| `07-testing/p0_*.sh` | runnable/manual checks |
| `07-testing/SIMULATION_*` | simulation planning/reference, not field readiness |
| `07-testing/field-runs/` | field evidence, date-bound |

## 6. Next Cleanup Order

1. Fix `QUICKSTART.md` current profile/source references and mojibake.
2. Move or clearly mark `REVIEW_2026Q2.md` as historical.
3. Condense `SIMULATION_INTEGRATION_CONTRACT.md` into a short current contract,
   leave evidence details in `07-testing/`.
4. Split oversized PRDs only after their implementation boundaries stabilize.
