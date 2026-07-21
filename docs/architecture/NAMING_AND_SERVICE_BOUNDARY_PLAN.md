# Naming And Service Boundary Plan

Status: cleanup plan
Audience: repo architecture cleanup maintainers
Replaced by: `MODULE_SERVICE_BOUNDARY.md` for current naming rules

This plan keeps LingTu's existing architecture words, but fixes their
responsibilities. The problem is not that `Blueprint`, `Module`, `Service`,
`Agent`, and `Kernel` are bad names. The problem is that current code often
uses them as if they all mean "entry point".

## 1. Problem Statement

LingTu currently has four related sources of confusion.

| Problem | Current symptom | Required correction |
| --- | --- | --- |
| Vocabulary confusion | `Module`, `Service`, `Blueprint`, `Agent`, and `Kernel` are not consistently separated. | Keep the words, but make each one have one job. |
| Directory confusion | `nav/services/map`, `maps`, `lingtu/assembly`, and `gateway/services` overlap. | Services must live in the domain that owns the capability. |
| Runtime confusion | Python sometimes looks like the core because it owns Blueprint, Module shells, and service facades. | Python may be the runtime shell; domain compute and durable semantics should move to native/domain services. |
| Product-chain confusion | Map, localization, planning, Gateway, DDS topics, and Module wires are described from different viewpoints. | Assembly is the product composition view; Blueprint is the graph mechanism; service contracts are the capability view. |

## 2. Keep These Words

Do not rename these globally yet:

```text
Blueprint
Module
Service
Agent
Kernel
```

They are short enough and already known in the codebase. The cleanup is a
boundary cleanup, not a vocabulary replacement project.

## 3. Hard Definitions

| Word | Owns | Must not own | Example |
| --- | --- | --- | --- |
| `Blueprint` | Materialize one application Module graph, resolve wires/transports, and return its runtime handle. | Product selection, native service lifecycle, domain business logic. | `runtime/blueprint.py` |
| `Assembly` | Choose LingTu modules, aliases, wires, route contracts, and profile-level config. | Domain business logic, native service supervision, map files, planner algorithms, HTTP routes. | `lingtu/assembly/stacks/maps.py` |
| `Module` | Runtime behavior with typed ports, lifecycle hooks, callbacks, status. | Durable domain model semantics when they can live in a service, hot algorithms when they can live in a kernel. | `GatewayModule`, `Navigation`, `VoxelGridModule` |
| `Service` | Domain capability API: maps, goals, patrol, planning, localization, relocalization, memory query. | Product composition, frontend routes, raw transport clients hidden inside business logic. | `MapsServiceCore`, `GoalService`, `PlannerService` |
| `Agent` | High-level decision loop: task decomposition, semantic reasoning, tool use, service orchestration. | Low-level data transport, map file access, local planner kernels, direct hardware control. | semantic task agent |
| `Kernel` | Pure compute and hot data paths, usually C++: planners, grid layers, registration, filters. | Runtime lifecycle, profile selection, API routing, business ownership. | `LocalPlannerCore`, maps voxel/ESDF/traversability cores |

## 4. One-Sentence Rules

```text
Assembly declares the LingTu product graph.
Blueprint materializes the Module graph.
Module runs the ports.
Service provides a domain capability.
Agent decides what to do by calling services.
Kernel computes fast and stays business-free.
```

These rules should be used in code review. If a file violates its word, rename
or move the file before adding more logic.

## 5. Service Ownership Rule

Services are not a shared dumping ground. A service must live under the domain
that owns the capability.

| Capability | Owning domain | Correct home |
| --- | --- | --- |
| Map assets, active map, artifacts, POI, map graph, bundles | Maps | `src/maps/service`, `src/maps/cpp`, `src/maps/adapters` |
| Goal intake, cancel, stop, patrol definition | Navigation | `src/nav/services/goals`, `src/nav/services/patrol` |
| Global planning request/result, preview, backend selection | Navigation planning | `src/nav/services/planning` |
| Local planner and path follower runtime | Navigation local runtime | `src/nav/local` |
| Safety policy and velocity arbitration | Navigation safety | `src/nav/safety` |
| SLAM status, localization health, relocalize against saved map | Localization | `src/localization/service`, `src/localization/adapters` |
| Semantic task decomposition and tool use | Agents/decision | `src/decision/agents` or `src/agents` |
| External HTTP/SSE/WS/MCP API | Gateway | `src/gateway/routes`, `src/gateway/adapters` |

Gateway is not a domain service. It is an external interface that calls domain
services.

Local planner is not a service by default. It is a runtime Module plus a
Kernel. Only query/preview/selection APIs around it may be services.

## 6. Target Directory Shape

This is the target shape after cleanup. It intentionally keeps existing names.

```text
src/runtime/
  blueprint.py
  module.py
  stream.py
  transports/
  runtime_interface.py

src/lingtu/
  assembly/
    profile_builder.py
    products/
    stacks/
    wires/

src/maps/
  service/          # domain service API if/when split from adapters
  include/
  cpp/
  modules/          # thin Module adapters only
  adapters/         # Python/DDS/Gateway/native ABI adapters

src/nav/
  mission/          # mission FSM and execution owner
  services/
    goals/
    patrol/
    planning/
  local/            # local planner, path follower, terrain runtime modules
  safety/

src/localization/
  service/
  kernel/
  adapters/

src/decision/
  agents/
  services/

src/gateway/
  routes/
  adapters/
  services/         # gateway-local helpers only, not domain ownership
```

## 7. What Must Move

| Current location | Problem | Target |
| --- | --- | --- |
| `src/nav/services/map` | Maps was incorrectly owned by navigation. | Completed: `src/maps/services` + `src/maps/adapters/python` |
| `src/nav/services/plan` | Name is too generic and mixes planner service, compat paths, and algorithms. | `src/nav/services/planning` plus `src/nav/planning` or `src/nav/local` as needed |
| local planner code under the old planner-service tree | Local planner is runtime execution, not a service. | Completed: `src/nav/local/local_planner.py`, `path_follower.py`, and `cpp/` |
| Gateway map helpers that inspect map files directly | Gateway should call maps service/bundle first. | `src/gateway/adapters/maps` or maps service API |
| Semantic task loops mixed with service helpers | Agent behavior and service capability are different. | `src/decision/agents` and `src/decision/services` |

## 8. Migration Plan

### Phase 0 - Lock vocabulary

- Add this document as the review contract.
- Update `docs/architecture/README.md` to reference it.
- Do not perform global class renames in this phase.

Acceptance:

- New code review can cite this document.
- No new top-level `services` dumping ground is introduced.

### Phase 1 - Map service leaves `nav` (completed)

- Keep `src/maps` as the owning domain for map core and artifacts.
- Keep the public map service API under `src/maps/services`.
- Keep Python only as a thin Module/runtime adapter.
- Make Gateway routes use maps query/bundle/control contracts first.

Acceptance:

- Ordinary map logic has no `nav.services.map` implementation dependency.
- Static guard rejects reintroducing `nav.services.map`.

### Phase 2 - Navigation service cleanup

- Rename `src/nav/services/plan` to `src/nav/services/planning` or split it
  into a planning service package plus algorithm packages.
- Move local planner/path follower runtime out of service directories.
- Keep `PlannerService` as the global planning capability boundary, not as a
  peer Module in the runtime graph.

Acceptance:

- Mission calls planning service APIs.
- Local planner/path follower are clearly runtime Modules under `nav/local`.
- Global planner algorithms are not hidden behind "backend" names that obscure
  the actual algorithm.

### Phase 3 - Localization service boundary

- Move relocalize/map-match/status capability APIs under localization.
- Keep ROS/DDS/native endpoint code in adapters.
- Keep ICP/BBS3D/NDT/GICP style compute in localization kernels.

Acceptance:

- Gateway relocalize route calls a localization service adapter.
- Nav mission consumes localization status and transforms; it does not own
  relocalization algorithms.

### Phase 4 - Agent layer clarification

- Put high-level semantic/task agents under `decision/agents` or `src/agents`.
- Agents may call goal, map, planning, localization, memory, and perception
  services.
- Agents must not directly read map files, planner artifacts, DDS topics, or
  hardware SDKs.

Acceptance:

- Tool-calling and task decomposition code is visibly separated from service
  implementation.
- Agent tests mock service contracts, not files and topics.

### Phase 5 - Static guards

Add repository guards for:

- `gateway` direct map-file access outside approved compatibility adapters.
- `nav` importing maps internals instead of maps service/bundle contracts.
- `decision/agents` importing kernels or transport clients directly.
- Service packages that import Blueprint composition code.
- Kernels importing runtime Module, Gateway, or transport packages.

Acceptance:

- Boundary regressions fail in CI.
- Exceptions must live in documented adapters with a short rationale.

## 9. Naming Rules For New Files

Use names that describe the owned boundary.

Good:

```text
maps/service/store.cpp
maps/adapters/python/service.py
nav/services/planning/service.py
nav/local/planner/module.py
decision/agents/task_agent.py
gateway/adapters/maps.py
```

Avoid:

```text
nav/services/map
nav/services/local_planner
gateway/services/map_service
common/service_utils
agent_service_module
```

If a name needs three nouns to explain itself, the boundary is probably wrong.

## 10. Review Checklist

Before adding or moving code, answer these questions:

1. Is this LingTu product composition? Put it in `lingtu/assembly`.
2. Is this generic Module graph materialization? Put it in `runtime/blueprint.py`.
3. Is this a runtime port owner? Make it a Module.
4. Is this a domain capability? Put it in that domain's Service.
5. Is this high-level task decision logic? Put it in Agent code.
6. Is this hot compute or algorithm logic? Put it in Kernel code.
7. Does it talk to a protocol, hardware, OS process, or file-backed external
   state? Put it behind an adapter or explicit service storage boundary.
8. Would moving this file make a domain easier to understand from its directory
   name alone? If yes, move it now.

## 11. Non-Goals

- Do not rename `Blueprint` and `Module` globally just for style.
- Do not remove Python runtime shells until an equivalent runtime exists.
- Do not hide product DDS/LCM/ROS boundaries inside domain services.
- Do not create one global `src/services` package.
- Do not move algorithms while changing their behavior unless tests pin the
  old behavior first.

## 12. First Implementation Slice

The first safe slice should be:

1. Keep `Blueprint` and `Module` names.
2. Move map service ownership documentation to `src/maps`.
3. Add static import guard for `maps.services`.
4. Create `src/nav/services/planning` as the intended global-planning service
   home.
5. Move local planner runtime package out of service naming.
6. Update architecture docs and `docs/REPO_LAYOUT.md`.

This slice should not change algorithm behavior.
