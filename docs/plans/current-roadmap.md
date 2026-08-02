# LingTu Current Roadmap

Status: active product roadmap
Updated: 2026-07-29

The target is a ROS-free, typed-DDS field runtime with one owner for every
control-critical output. This file contains remaining work only. Shipped
contracts live in `docs/architecture/`; run results live in
`docs/07-testing/field-runs/`.

## Fixed Product Boundary

```text
LiDAR / IMU
  -> native sensor process
  -> native SLAM
  -> MapObservation + odometry + registered cloud
  -> mapd (map state and scene) + traversability (control-risk grid)
  -> navd (planning, local avoidance, tracking, final command gate)
  -> /nav/cmd_vel
  -> driver
  -> Brainstem

HostBus <-> Gateway / Agent / MCP
```

Ownership is fixed:

- `navd` is the only field navigation state and `/nav/cmd_vel` writer.
- standalone native traversability is the only `/nav/traversability` writer;
  mapd must not compete for that control topic.
- `mapd` owns realtime map ingestion/layers/scene; native map store services own
  persistent records and artifacts.
- the Python Host translates operator/API traffic and consumes native state. It
  does not run a second field planner, map hot path, or command writer.
- Blueprint assembles Modules inside one Host process only. ProductControl
  and its internal SystemdRunner own Product/process lifecycle.
- every field Product that declares a Python Host carries one Blueprint graph
  in the same fingerprinted RunPlan; Product-specific Host lifecycle forks are
  not part of the architecture.
- missing or stale native components fail startup; there is no field Python
  fallback.

## Runtime Surface Classification

| Surface | Classification | Decision |
| --- | --- | --- |
| `Product`, `ProductControl`, `RunPlan` | current field control plane | Keep. Product is compiled once; ProductControl owns switching and applies the exact RunPlan. |
| `Host`, `Blueprint`, `Module` | current scoped Host runtime | Keep. They assemble Gateway, Agent, MCP, semantic logic, adapters, and local/dev Modules; they do not own field processes. |
| `src/nav/cpp`, `navd` | current field navigation owner | Keep and validate as the unique planner/tracker/final-command implementation. |
| `src/maps/cpp/mapd`, native map store/build | current field map owner | Keep. mapd owns realtime layers/scene; native map services own persistent products. |
| Python navigation and realtime map Modules | development/simulation compatibility | Keep for `dev` and `sim_nav`; every Product that declares native maps now explicitly forbids Python realtime map Modules. |
| Python `maps.service` | transitional low-rate Host adapter | Keep only until typed native map control/query fully replaces its file and business operations; it must never own realtime field layers. |
| Gateway map preview paths | active visualization compatibility | `/api/v1/map/points` remains live accumulated point-cloud JSON compatibility used by Rerun/evidence/Web until those consumers migrate to WS/DDS. `/api/v1/maps/{name}/points` is saved-map JSON; `/api/v1/maps/{name}/pcd` is raw saved-map PCD. |
| Taskless inspection endpoints returning `410` | retired API tombstones | Removed after the client inventory found no callers; task-addressed inspection APIs are the only Gateway contract. |
| `MapManagerModule`, old nav C++ paths, ROS2 TARE path | retired names and paths | Removed. Static contracts prevent their return; do not recreate shims. |
| Root `Testing/` | generated CTest debris | Deleted from maintained files and ignored. |

The former broad Module-centric slogan is retired. The accurate statement is
"Product-defined field runtime with a scoped Module/Blueprint Host."

## Product Modes

The YAML files under `config/runtime_graph/products/` are authoritative.

| Product | SLAM | Saved map | Control owner | Purpose |
| --- | --- | ---: | --- | --- |
| `teleop` | none | no | operator | Direct operator motion through native authority and final safety. |
| `teleop_avoid` | mapping | no | operator | Operator-assisted local detours using live odometry, cloud, maps scene, and standalone traversability. |
| `map` | mapping | no | operator | Build and transactionally save map products. |
| `explore` | mapping or localization | optional | exploration endpoint | No map grows a live map; a map selects saved-map coverage. |
| `nav` | localization | yes | native navigation | Saved-map autonomous navigation. |
| `tracking` | localization | yes | native navigation | Follow explicit map-frame goals without semantic selection. |
| `inspection` | localization | yes | native navigation | Execute persisted multi-point inspection tasks. |

`explore` is the only public exploration Product. The presence of a map selects
the map/localization variant; absence of a map selects live mapping. TARE may
remain an internal policy name but must not reappear as an operator Product.

Do not reintroduce a localization map requirement into `teleop_avoid`. Its
current acceptance manifest must run mapping/no-map and must not promote a
temporary localization PCD.

## Current Work Board

| Priority | Work | Completion gate | Status |
| --- | --- | --- | --- |
| P0 | Rebuild a self-contained native release from the current IDL/source closure. | Strict Product preflight selects only release artifacts, reports hashes/mtimes, and has `blockers=[]`; release includes and tests mapd and the DDS probe. | Active |
| P0 | Stabilize MuJoCo mapping/no-map SLAM for `teleop_avoid`. | Native sensor -> SLAM -> MapObservation remains tracking for the full representative run; no covariance/finite-data gate failure; no test-specific production threshold weakening. | Active |
| P0 | Close `teleop_avoid` Product behavior. | Free-space plus obstacle scenarios prove typed operator authority, local path, final command, driver acknowledgement, cleanup, and zero barrier with exact provenance. | Active |
| P0 | Close physical command safety on S100P. | No-motion readiness and fault injection pass before bounded supervised motion; driver executable and ACK freshness are proven. | Blocked on preceding gates |
| P1 | Make typed native map control/query the only field map-service path. | Gateway performs no map file I/O or business-state inference; save/query/recovery survive Host restart. | Active |
| P1 | Prove dynamic-obstacle clearing and long-run resource bounds. | `moving_person_clear`, labelled replay, and MID-360 runs meet residual, thin-obstacle, CPU, memory, DDS-byte, and epoch-reset limits. | Gate implemented; accepted run pending |
| P1 | Integrate native velocity smoothing without weakening safety. | Smoother runs before final safety; emergency/stale/cancel zero bypasses ramping; authority transitions and rollback Product are tested. | Kernel complete; Product wiring pending |
| P1 | Add a replaceable collision-aware path-smoother contract. | Preserves frame, map generation, endpoints, clearance, curvature, and planner direction/cusps; failure is explicit and fail-closed. | Not started |
| P2 | Productize generic route operations. | Typed route request/status/event lifecycle, closure/reroute semantics, and progress telemetry over `maps::MapGraph`. | Foundations only |
| P2 | Add target following. | Independent Product, typed target observation, lost-target behavior, standoff controller, safety ownership, MuJoCo and field evidence. | Foundations only |
| P2 | Add docking/charging. | Dock database, pose/covariance, staged controller, hardware contact/charge feedback, retry/undock lifecycle, simulation and physical evidence. | Missing |

Detailed capability maturity and upstream adoption decisions live in
[`NAVIGATION_CAPABILITY_MATRIX.md`](../architecture/NAVIGATION_CAPABILITY_MATRIX.md).
Architecture/product gaps are summarized in [`known_gaps.md`](../known_gaps.md).

## Acceptance Order

For each Product change:

1. Prove focused algorithm and fail-closed tests.
2. Build every affected native artifact from the current schema/source closure.
3. Pass strict preflight with unique writers and no stale artifacts.
4. Pass the named native MuJoCo or replay scenario.
5. Pass target no-motion readiness and fault injection.
6. Run bounded supervised field motion only when the previous gates pass.
7. Add a new date-prefixed field record; do not edit old evidence.

## Source Of Truth

| Topic | Source |
| --- | --- |
| Product declarations | `config/runtime_graph/products/*.yaml` |
| Endpoint/process/topic declarations | `config/runtime_graph/` |
| Product/Host/process ownership | `docs/architecture/SYSTEM_DESIGN.md` |
| Native navigation dataflow | `docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md` |
| Map lifecycle and products | `docs/architecture/MAP_SERVICE_CONTRACT.md` |
| Field Products | `docs/architecture/FIELD_PRODUCTS.md` |
| Capability maturity | `docs/architecture/NAVIGATION_CAPABILITY_MATRIX.md` |
| Validation gates | `docs/07-testing/README.md` |

## Claim Discipline

Until matching evidence exists, do not claim that dynamic people never leave
residuals, velocity smoothing is active in the field, target following or
docking is supported, map services are fully native end to end, or a Product
is complete from unit tests alone.
