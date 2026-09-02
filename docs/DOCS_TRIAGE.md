# Documentation Policy And Cleanup Ledger

Status: current documentation policy
Updated: 2026-09-03

This file defines where documentation belongs and records destructive cleanup.
It is not an architecture or runtime contract.

## Information Architecture

| Location | Owns | Must not contain |
| --- | --- | --- |
| `docs/README.md` | Public documentation entry and reading order. | Detailed contracts, plans, or dated evidence. |
| `docs/CURRENT.md` | Current authority, support, and claim map. | Build logs or implementation history. |
| `docs/01-*` through `docs/10-*` | Task-oriented guides and operator procedures. | Competing architecture definitions. |
| `docs/product/` | Product intent and acceptance semantics. | Runtime implementation detail or evidence. |
| `docs/architecture/` | Current contracts and accepted architecture decisions. | Research notes, abandoned proposals, dated run results. |
| `docs/api/` | Generated API inventories and generation instructions. | Hand-maintained API snapshots. |
| `docs/plans/` | The active roadmap and explicitly indexed bounded plans. | Completed plans or historical PRDs. |
| `docs/research/` | Upstream reviews, algorithm investigations, and migration studies. | Product claims or acceptance evidence. |
| `docs/worklogs/` | Short session-continuity notes and reviewed recovered transcripts. | Runtime authority or acceptance evidence. |
| `docs/assets/` | Images referenced by maintained documentation. | Unlinked screenshots or generated paper output. |
| `docs/07-testing/` | Validation index plus reader-facing guide. | Unclassified gates, executable scripts, or one-off proposals. |
| `docs/07-testing/field/` | Reusable real-robot and target-compute gates. | Dated results or simulation contracts. |
| `docs/07-testing/simulation/` | Reusable simulator gates and fidelity contracts. | Field-readiness claims. |
| `docs/07-testing/field-runs/` | Immutable, date-prefixed validation evidence. | Reusable gates, current backlog, or architecture truth. |
| `scripts/gates/field/` | Executable P0 field procedures. | Python diagnostics or prose-only acceptance definitions. |

Package-local `README.md` files explain only that package's public boundary,
build, and focused tests. They link to central contracts instead of copying
system architecture.

## Authority Rules

1. Running configuration and typed schemas outrank prose when they conflict.
2. `docs/architecture/` describes accepted current behavior only.
3. A plan, research note, unit test, or source file does not prove a Product
   capability. Use the evidence levels in
   [`NAVIGATION_CAPABILITY_MATRIX.md`](architecture/NAVIGATION_CAPABILITY_MATRIX.md).
4. Field claims require a dated record and exact Product/binary provenance.
5. Historical context belongs in git history. Do not recreate `docs/archive/`
   or tool-generated `docs/superpowers/` trees.
6. Hidden tool state such as `.qoder/`, `.hermes/`, `.codex/`, and `.omx/` is
   not maintained product documentation.

## Cleanup Performed On 2026-07-28

Deleted because a current contract or git history already preserves the useful
information:

- dated `docs/superpowers/` implementation plans;
- Q2 and phase-one architecture reviews;
- retired Thunder and Python navigation transport plans;
- duplicate simulation closure audits, delivery backlogs, and scene-selection
  notes;
- generated progress-report Markdown, PDF, DOCX, and its one-off generator;
- the dated hand-maintained Gateway API snapshot;
- superseded naming and Dart/Rust migration proposals.

Moved without changing their conclusions:

- upstream and algorithm investigations into `docs/research/`;
- reusable semantic-memory validation into `docs/07-testing/`;
- dated MuJoCo policy evidence into `docs/07-testing/field-runs/`.

## Cleanup Performed On 2026-08-23

- grouped reusable field and simulation gates under dedicated indexes;
- moved executable P0 procedures from `docs/` to `scripts/gates/field/`;
- moved contributor commit/push policy to `docs/03-development/`;
- moved the undated native endpoint checklist out of the dated evidence folder.

## Cleanup Performed On 2026-09-03

- reduced `CURRENT.md`, the roadmap, and known gaps to their declared roles;
- removed completed Message/DDS planning, duplicate tuning guidance, stale
  deployment workflow notes, and unreferenced media;
- linked every current deployment guide, architecture contract, API index,
  worklog, and dated evidence record from an owning index;
- corrected current commands and heading links after runtime/test migrations;
- kept all dated field-run evidence and recovered worklogs unchanged.

## Placement Checklist

Before adding a document, answer these questions:

1. Is this current behavior, intended work, research, a reusable gate, or dated
   evidence?
2. Which existing index will link to it?
3. What code/config/schema is the source of truth?
4. What status and date make its authority unambiguous?
5. Can an existing document be updated instead of creating another file?

Every maintained page must be reachable from an owning index. Keep an image
only when a maintained page or repository entry point references it.

Run the repository documentation guard before review:

```bash
python tools/validate/validate_docs.py
```
