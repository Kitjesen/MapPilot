# LingTu Current Status

Status: runtime and documentation snapshot as of 2026-07-31

## Usability Snapshot

This snapshot separates code-level verification from target and motion acceptance.

| Surface | Status now | Safe use | Still required |
| --- | --- | --- | --- |
| Local/validation Host Profiles (`stub`, `dev`, `sim`, `sim_nav`, `portable_mujoco`, `lite`) | Development/contract-ready | Blueprint/Module development, configuration inspection, and focused tests | Hardware behavior still needs its named target gate |
| Product assembly and switch preview | Contract-ready | Resolve one Product in fixed `env`, inspect its fingerprinted RunPlan, and copy the ProductControl command | A preview never proves Linux processes started or became ready |
| Gateway, SDK, and map HTTP contracts | Integration-ready | Client development, auth/route contracts, saved-map queries, and read-only switch planning | Typed native map control/query migration is still active |
| ProductControl and SystemdRunner | Code and contract verified | Review and test switch, stop, readiness, rollback, and conflict cleanup transactions | Rebuild the release and execute the transaction on Linux/systemd |
| Native C++ LiDAR/SLAM/map/navigation/driver chain | Not revalidated in this cleanup | Source and contracts may be developed | Rebuild current C++ artifacts and pass native simulation/replay gates |
| S100P field autonomy and motion | Not accepted | No-motion diagnostics only after deployment | Fresh release provenance, no-motion readiness, fault injection, then bounded supervised motion |
| Super-LIO | Lab-only experimental integration | Stationary external ROS 2 evaluation using `integrations/super_lio/` | Typed adapter, ProductControl ownership, native release inclusion, and S100P acceptance |

Overall verdict: the repository is usable for local development and interface/architecture validation. It is not yet evidence that the current checkout is production-ready for autonomous S100P motion.

The remaining promotion order is maintained in `plans/current-roadmap.md`; dated simulation and field evidence lives under `07-testing/`.

## Documentation Map

Use this file when deciding which document is authoritative.

For documentation placement and the deletion ledger, see `DOCS_TRIAGE.md`.

## Curated Documentation Entry Points

The pages below are task-oriented navigation aids. They do not supersede the
contracts and references listed later in this file.

| Need | Curated entry point |
| --- | --- |
| Understand the inspection product and result-acceptance model | `product/README.md` |
| Choose a local, simulation, or field starting path | `01-getting-started/README.md` |
| Learn Product, Host, Blueprint, Module, and DDS concepts | `02-concepts/README.md` |
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
| Inspection product intent and acceptance semantics | `docs/product/inspection-product.md` |
| System architecture | `docs/architecture/SYSTEM_DESIGN.md` |
| Module, Blueprint, Port/Wire model | `docs/architecture/LINGTU_RUNTIME_BUS_DECISION.md` |
| Runtime Graph env/Product/topic contract | `config/runtime_graph/README.md` |
| Global planning input/output | `docs/architecture/GLOBAL_PLANNING_CONTRACT.md` |
| Saved map types and artifact bundles | `docs/architecture/MAP_SERVICE_CONTRACT.md` |
| Navigation compute chain | `docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md` |
| Navigation capability maturity and evidence gaps | `docs/architecture/NAVIGATION_CAPABILITY_MATRIX.md` |
| Native navigation/map data-plane migration boundary | `docs/architecture/NATIVE_DATA_PLANE_MIGRATION.md` |
| Local planner I/O | `docs/architecture/local_planner_io_contract.md` |
| Frame contract | `docs/architecture/ros_frame_contract.md` |
| Repository placement | `docs/REPO_LAYOUT.md` |
| Runtime quickstart | `docs/QUICKSTART.md` |
| SDK, REST, MCP, SSE, and teleoperation integration usage | `docs/09-integrations/README.md` |
| Software control ownership, motion gate, and stop/recovery usage | `docs/10-safety/README.md` |
| Known product gaps | `docs/known_gaps.md` |
| Physical runtime Env | `config/runtime_graph/envs/real.yaml` |
| Native DDS IDL and typed message contracts | `src/message/idl/README.md` |
| Thunder driver deployment boundary | `scripts/deploy/thunder/lingtu-driver.service` |
| MuJoCo native-DDS navigation acceptance | `docs/07-testing/MUJOCO_NAVIGATION_ACCEPTANCE.md` |
| Native endpoint control-mode promotion gate | `docs/07-testing/MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md` |

## Not Authoritative By Default

| Location | How to use |
| --- | --- |
| `docs/research/` | Upstream evaluations and algorithm investigations. Not product contract or acceptance evidence. |
| `docs/07-testing/field-runs/` | Dated evidence snapshots. They support only the named run and environment. |
| `docs/plans/` | Forward-looking PRDs and migration plans. Not shipped behavior. |
| `.qoder/`, `.hermes/`, `.codex/`, `.omx/` | Tool-generated workspace memory. Regenerate with the owning tool; do not treat as product documentation. |

## Current Product Defaults

| Area | Current position |
| --- | --- |
| Host runtime unit | `Module` |
| Host graph assembly | `Blueprint` |
| Product operation | `ProductControl(env).switch(Product) -> RunPlan -> systemd` |
| Dataflow model | `Port -> Wire -> Transport` |
| Product global planner | `octoplanner3d` through `GlobalPlanner` |
| Compatibility planners | `direct` for lightweight/direct paths; `pct` and `astar` for explicit legacy/manual comparison |
| UI global path payload | `lingtu.global_plan.v1` |
| ROS 2 role | Compatibility, replay, benchmark, and legacy gate adapter only |
| Physical runtime env | `real` |
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
