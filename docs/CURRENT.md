# Current Documentation Map

Status: current routing map

Use this file when deciding which document is authoritative.

For cleanup decisions and archive candidates, see `DOCS_TRIAGE.md`.

## Authoritative Now

| Topic | Source of truth |
| --- | --- |
| System architecture | `docs/architecture/SYSTEM_DESIGN.md` |
| Module, Blueprint, Port/Wire model | `docs/architecture/LINGTU_RUNTIME_BUS_DECISION.md` |
| Global planning input/output | `docs/architecture/GLOBAL_PLANNING_CONTRACT.md` |
| Saved map types and artifact bundles | `docs/architecture/MAP_SERVICE_CONTRACT.md` |
| Navigation compute chain | `docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md` |
| Local planner I/O | `docs/architecture/local_planner_io_contract.md` |
| Frame contract | `docs/architecture/ros_frame_contract.md` |
| Repository placement | `docs/REPO_LAYOUT.md` |
| Runtime quickstart | `docs/QUICKSTART.md` |
| Known product gaps | `docs/known_gaps.md` |

## Not Authoritative By Default

| Location | How to use |
| --- | --- |
| `docs/archive/` | Placeholder only. Old snapshots were removed; use git history. |
| `docs/superpowers/` | Work plans and old execution notes. Not product contract. |
| `docs/07-testing/*AUDIT*.md` | Evidence snapshots. Date-bound, not architecture. |
| `docs/plans/` | Forward-looking PRDs and migration plans. Not shipped behavior. |
| `docs/09-paper/` | Publication drafts and build output. Not runtime docs. |

## Current Product Defaults

| Area | Current position |
| --- | --- |
| Runtime unit | `Module` |
| Orchestration unit | `Blueprint` |
| Dataflow model | `Port -> Wire -> Transport` |
| Product global planner | `octoplanner3d` through `GlobalPlanner` |
| Legacy planner | `pct`, explicit/manual compatibility |
| UI global path payload | `lingtu.global_plan.v1` |
| ROS 2 role | Compatibility adapter only |
