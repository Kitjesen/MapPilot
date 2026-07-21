# Current Documentation Map

Status: current routing map as of 2026-07-18

Use this file when deciding which document is authoritative.

For cleanup decisions and archive candidates, see `DOCS_TRIAGE.md`.

## Curated Documentation Entry Points

The pages below are task-oriented navigation aids. They do not supersede the
contracts and references listed later in this file.

| Need | Curated entry point |
| --- | --- |
| Choose a local, simulation, or field starting path | `01-getting-started/README.md` |
| Learn Module-First concepts | `02-concepts/README.md` |
| Find the owning development surface | `03-development/README.md` |
| Build a REST, SDK, MCP, SSE, or teleoperation integration | `09-integrations/README.md` |
| Map, navigate, use semantic goals, or explore | `05-guides/README.md` |
| Operate and diagnose a running system | `06-operations/README.md` |
| Understand control ownership, stop/recovery, and movement boundaries | `10-safety/README.md` |
| Prepare a field target without target-specific addresses or credentials | `04-deployment/WEB_GUIDE.md` |
| Select a local, simulation, or no-motion field validation gate | `07-testing/WEB_GUIDE.md` |
| Find CLI, REST, MCP, and configuration references | `08-reference/README.md` |

The root `README.md` is the public documentation home. Keep plans, dated audit
reports, and field-run evidence out of its primary reading paths.

## Status Terms

| Label | Meaning | How to use it |
| --- | --- | --- |
| Current | Describes the supported product behavior or an active contract. | It may be used as an implementation or operator reference. |
| Reference | Lists stable interfaces, commands, configuration, or generated inventory. | Check its scope and generation date before relying on an individual entry. |
| Evidence | Records the result of a dated test, simulation run, or field session. | It supports only the named claim and environment. |
| Plan | Describes intended work, not shipped behavior. | Do not cite it as a runtime contract. |
| Historical | Retained context that requires revalidation before reuse. | Prefer a current replacement or git history. |

## Authoritative Now

| Topic | Source of truth |
| --- | --- |
| System architecture | `docs/architecture/SYSTEM_DESIGN.md` |
| Module, Blueprint, Port/Wire model | `docs/architecture/LINGTU_RUNTIME_BUS_DECISION.md` |
| Runtime Graph endpoint/topic contract | `config/runtime_graph/README.md` |
| Global planning input/output | `docs/architecture/GLOBAL_PLANNING_CONTRACT.md` |
| Saved map types and artifact bundles | `docs/architecture/MAP_SERVICE_CONTRACT.md` |
| Navigation compute chain | `docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md` |
| Local planner I/O | `docs/architecture/local_planner_io_contract.md` |
| Frame contract | `docs/architecture/ros_frame_contract.md` |
| Repository placement | `docs/REPO_LAYOUT.md` |
| Runtime quickstart | `docs/QUICKSTART.md` |
| SDK, REST, MCP, SSE, and teleoperation integration usage | `docs/09-integrations/README.md` |
| Software control ownership, motion gate, and stop/recovery usage | `docs/10-safety/README.md` |
| Known product gaps | `docs/known_gaps.md` |
| Native field endpoint | `config/runtime_graph/endpoints/thunder_field.yaml` |
| Native DDS IDL and typed message contracts | `src/message/idl/README.md` |
| Thunder driver deployment boundary | `scripts/deploy/thunder/lingtu-driver.service` |
| MuJoCo native-DDS navigation acceptance | `docs/07-testing/MUJOCO_NAVIGATION_ACCEPTANCE.md` |
| Native endpoint control-mode promotion gate | `docs/07-testing/MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md` |

## Not Authoritative By Default

| Location | How to use |
| --- | --- |
| `docs/archive/` | Placeholder only. Old snapshots were removed; use git history. |
| `docs/superpowers/` | Work plans and old execution notes. Not product contract. |
| `docs/07-testing/*AUDIT*.md` | Evidence snapshots. Date-bound, not architecture. |
| `docs/plans/` | Forward-looking PRDs and migration plans. Not shipped behavior. |
| `docs/09-paper/` | Publication drafts and build output. Not runtime docs. |
| `.qoder/`, `.hermes/`, `.codex/`, `.omx/` | Tool-generated workspace memory. Regenerate with the owning tool; do not treat as product documentation. |

## Current Product Defaults

| Area | Current position |
| --- | --- |
| Runtime unit | `Module` |
| Orchestration unit | `Blueprint` |
| Dataflow model | `Port -> Wire -> Transport` |
| Product global planner | `octoplanner3d` through `GlobalPlanner` |
| Compatibility planners | `direct` for lightweight/direct paths; `pct` and `astar` for explicit legacy/manual comparison |
| UI global path payload | `lingtu.global_plan.v1` |
| ROS 2 role | Compatibility, replay, benchmark, and legacy gate adapter only |
| Physical field endpoint | `thunder_field` |
| Field command output | `endpoint_only`: logical `/nav/cmd_vel`, DDS wire `rt/nav/cmd_vel`, then `lingtu-driver` |
| Brainstem boundary | Remote gRPC target loaded from `/opt/lingtu/config/brainstem.env` |
| Gateway / MCP defaults | Gateway `5050`, MCP JSON-RPC `8090` |

## Current Evidence Boundaries

| Claim | Minimum current source |
| --- | --- |
| Local framework health | Focused Python tests and `stub`/`dev` profile inspection |
| MuJoCo native navigation/control behavior | `MUJOCO_NAVIGATION_ACCEPTANCE.md` and matching artifacts/manifests |
| Native endpoint control-mode promotion | `MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md`; handwritten JSON summaries are not promotion evidence |
| Physical robot readiness | Fresh field-run evidence under `docs/07-testing/field-runs/` plus robot-side `scripts/lingtu` diagnostics |
| Historical context | Dated audit, plan, and field-run files only; revalidate before citing as current behavior |

## Current Lab Address Note

Reusable docs should use `$LINGTU_ROBOT_HOST`, `$ROBOT_HOST`, or `<robot-ip>`.
Keep the active lab target in the operator environment or a dated, access-
controlled field record. Do not embed live field endpoints in public or
Web-bundled documentation.
