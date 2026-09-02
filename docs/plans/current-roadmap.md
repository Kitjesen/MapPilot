# LingTu Current Roadmap

Status: active product roadmap
Updated: 2026-09-03

This file contains remaining cross-domain work only. Current behavior belongs
in [`../architecture/`](../architecture/README.md), open gaps in
[`../known_gaps.md`](../known_gaps.md), and dated results in
[`../07-testing/field-runs/`](../07-testing/field-runs/README.md).

## Fixed Boundaries

- Public runtime environments are exactly `real` and `sim`.
- Product declarations are env-independent. ProductControl resolves one Product
  once into one RunPlan and owns its complete lifecycle transaction.
- `real` uses the internal SystemdRunner; `sim` owns direct child processes.
- Native sensor, SLAM, maps, traversability, navigation, and driver processes
  own the field hot path through typed DDS.
- `navd` owns final navigation arbitration. Only `lingtu-driver` forwards the
  checked command to the selected robot adapter.
- The Python Host owns Gateway, Agent, MCP, semantic behavior, and selected
  low-rate adapters. Blueprint owns only the in-process Host Module graph.
- Windows x64 and Linux/WSL x86_64 are separate native MuJoCo Product targets.
  Neither platform inherits the other's evidence.
- S100P/RDK X5 Ubuntu aarch64 is the separate field target. Simulation never
  substitutes for its safety and driver evidence.

The exact Products, roles, topics, and env mappings come from
[`config/runtime_graph/`](../../config/runtime_graph/README.md), not this plan.

## Active Work Board

| Priority | Work | Completion gate | State |
| --- | --- | --- | --- |
| P0 | Complete native MuJoCo Product parity. | Every Product passes the exact ProductControl lifecycle, scenario, terminal-zero, cleanup, rollback, and repeatability gate on Windows and Linux/WSL independently. | Active |
| P0 | Produce the canonical native release. | A release assembled from `install/<platform>-<arch>/<config>/{bin,lib,etc,share}` passes package, installer, activation, and rollback checks without production references to `build/`. | Active |
| P0 | Stabilize mapping/no-map `teleop_avoid`. | Native sensor -> SLAM -> map/risk -> nav -> driver remains ready through representative free-space, obstacle, stop, and cleanup scenarios. | Active |
| P0 | Close S100P physical command safety. | Fresh provenance, no-motion readiness, fault injection, bounded supervised motion, terminal zero, and driver ACK pass on the target. | Blocked on preceding gates |
| P1 | Prove dynamic-obstacle clearing and resource bounds. | Moving-person, labelled replay, and long-duration MID-360 runs meet residual, thin-obstacle, reset, CPU, memory, and DDS-volume limits. | Evidence pending |
| P1 | Finish motion/path smoothing. | Velocity ramp, reversal, rotation, emergency zero, and safety ordering pass; a collision-aware path smoother has an explicit contract and failure policy. | Partial |
| P1 | Finish typed native map control/query. | Persistent field operations no longer require a competing Python map-management path. | Partial |
| P2 | Productize route, following, and docking. | Each capability has an explicit Product, typed lifecycle, safety boundary, simulator evidence, and physical evidence where applicable. | Foundations only |
| P2 | Stabilize cross-language schemas. | DDS/shared-memory payloads carry explicit version, frame, timestamp, and bounds with compatibility tests. | Open |

The capability-by-capability evidence view remains in
[`NAVIGATION_CAPABILITY_MATRIX.md`](../architecture/NAVIGATION_CAPABILITY_MATRIX.md).

## P0 Execution Order

1. Build every selected native artifact from the current IDL and source tree.
2. Resolve each target Product and reject missing, stale, mixed-platform, or
   undeclared artifacts before process startup.
3. Run the exact ProductControl transaction from a fresh state root.
4. Attach the named MuJoCo scenario to the committed Product session.
5. Prove terminal zero, child cleanup, and rollback before promotion.
6. Repeat independently on Windows and Linux/WSL.
7. Assemble and validate the canonical Linux field release.
8. Run S100P no-motion and fault-injection gates before any supervised motion.

The previous portability, production-SLAM separation, Windows staging, RunPlan
compile, and coordinator implementation phases are complete foundations. Their
details live in the owning contracts, build guide, tests, and Git history; they
are not repeated as active roadmap sections.

## Product Acceptance Matrix

Run the declared Products, including both `explore` variants, through their
exact manifests. Promotion requires an archived report with no blockers,
matching Product session identity, declared artifacts, verified rollback, and
terminal physical zero acknowledgement.

Component tests, a dry run, a catalog compile, or a manually assembled process
chain cannot promote a manifest from component to Product coverage.

## Release And License Gates

Field packaging must consume the canonical install prefix. The temporary
dual-layout package exists only for the documented systemd rollback window;
current units use `/opt/lingtu/current/{bin,lib,etc/lingtu,share/lingtu}`.

FAST_LIO, ikd-Tree, and IKFoM provenance and license obligations require an
explicit release decision. Replacing only one container does not establish a
closed-source distribution boundary.

Before commercial distribution, choose and document one complete path:

- authorization covering the full copyright chain;
- a reviewed GPL-compliant distribution boundary; or
- a clean-room replacement of the affected SLAM core.

This is an engineering release gate, not legal advice. Local hashes, lock files,
or an unreviewed external Windows fork do not clear it.

## Acceptance Order

For each behavior change:

1. Pass the focused algorithm and fail-closed contract tests.
2. Build the affected native artifacts from the current schema/source closure.
3. Pass strict preflight with unique owners and no stale artifacts.
4. Pass the named replay or native MuJoCo scenario.
5. Pass target no-motion readiness and fault injection.
6. Run bounded supervised field motion only when the earlier gates pass.
7. Add a new date-prefixed evidence record; never rewrite an old run.

## Sources Of Truth

| Topic | Source |
| --- | --- |
| Product and env declarations | [`config/runtime_graph/`](../../config/runtime_graph/README.md) |
| Product/Host/process ownership | [`SYSTEM_DESIGN.md`](../architecture/SYSTEM_DESIGN.md) |
| Native navigation dataflow | [`NAVIGATION_RUNTIME_DATAFLOW.md`](../architecture/NAVIGATION_RUNTIME_DATAFLOW.md) |
| Map lifecycle and artifacts | [`MAP_SERVICE_CONTRACT.md`](../architecture/MAP_SERVICE_CONTRACT.md) |
| Field process ownership | [`FIELD_PRODUCTS.md`](../architecture/FIELD_PRODUCTS.md) |
| Simulation platform boundary | [`SIMULATION_INTEGRATION_CONTRACT.md`](../architecture/SIMULATION_INTEGRATION_CONTRACT.md) |
| Build commands | [`BUILD_GUIDE.md`](../01-getting-started/BUILD_GUIDE.md) |
| Validation levels and gates | [`07-testing/README.md`](../07-testing/README.md) |

## Claim Discipline

Do not claim that a Product is complete from unit or component tests, that WSL
is Windows-native evidence, or that simulation proves S100P behavior. Dynamic
obstacle, smoothing, following, docking, SaveMap/PGO, and release claims remain
limited to the latest explicit evidence gate.
