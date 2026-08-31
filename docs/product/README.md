# Product Documentation

Status: current product documentation index

This directory defines operator-facing product intent and acceptance semantics.
It does not own runtime architecture, implementation plans, or dated test
evidence.

## Current Definitions

| Product document | Scope |
| --- | --- |
| [Inspection product](./inspection-product.md) | Users, operating loop, evidence integrity, task report behavior, and product claim boundaries. |
| [Simulation product](./simulation-studio-product.md) | Confirmed UE5 Runtime, UE5 Create, and SimStudio management boundaries, MATRiX benchmark decisions, and first playable vertical-slice acceptance. Live qualification remains pending. |

Runtime behavior remains authoritative in `docs/architecture/`; unfinished
implementation work belongs in `docs/plans/current-roadmap.md`; dated proof
belongs in `docs/07-testing/field-runs/`.
