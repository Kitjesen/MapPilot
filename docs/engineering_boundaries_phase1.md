# LingTu Engineering Boundaries, Phase 1

> **Status: historical migration reference.** This document records the first
> boundary-cleanup phase and intentionally retains the paths and terminology
> used at that time. For the current package map and runtime contract, use
> [Repository Layout](REPO_LAYOUT.md),
> [System Design](architecture/SYSTEM_DESIGN.md), and
> [Module / Service Boundary](architecture/MODULE_SERVICE_BOUNDARY.md).

Phase 1 does not move directories. It names the target ownership boundaries and
adds verification around contracts and composition before any larger package
migration.

## Target Boundaries

| Target package | Current surfaces | Responsibility |
| --- | --- | --- |
| `runtime` | `src/runtime/module.py`, `src/runtime/stream.py`, lifecycle helpers | Module lifecycle, ports, scheduling, backpressure |
| `contracts` | `src/runtime/contracts/` | Runtime-checkable schemas for high-risk dict messages |
| `composition` | `src/runtime/runtime_profiles.py`, `src/lingtu/assembly/` | Profile compilation, stack factories, explicit wires |
| `platform` | `src/drivers/`, `src/localization/`, `config/devices.yaml`, `config/robot_config.yaml` | Robot hardware, ROS 2 bridges, SLAM/localization adapters |
| `mapping` | `src/nav/services/*map*`, `src/nav/services/dynamic_filter.py` | Occupancy, ESDF, elevation, traversability, map lifecycle |
| `navigation` | `src/nav/`, `src/nav/services/plan/`, `src/nav/local/` | Mission FSM, global planning, local planning, cmd_vel arbitration |
| `semantics` | `src/perception/`, `src/decision/`, `src/memory/` | Perception, goal resolution, LLM/tool loop, semantic memory |
| `interfaces` | `src/gateway/`, `src/gateway/media/`, `scripts/lingtu` | REST/SSE/WS/MCP, dashboard, field operations CLI |
| `ops` | `scripts/`, `.github/`, deployment helpers | Build, deployment, diagnostics, CI |

## Dependency Rules

- `contracts` must stay data-only and cannot import runtime modules.
- `composition` may import stack factories and contracts, but must not start
  hardware, systemd services, ROS 2 processes, or calibration checks in graph
  tests.
- `interfaces` can translate external API payloads into module ports, but
  should not own navigation, SLAM, mapping, or semantic business rules.
- `navigation`, `mapping`, and `semantics` communicate through module ports and
  contracts. Avoid direct cross-domain imports except registry/factory glue.
- `platform` owns hardware/service details. Higher layers consume its ports and
  status contracts, not concrete driver internals.

## Phase 1 Verification Gates

- Contract tests must reject missing required fields, invalid enum values,
  invalid numeric ranges, and schema-version mismatches.
- Profile graph tests must compile `stub`, `dev`, `sim`, `map`, `nav`, and
  `explore` without startup side effects and must fail on dangling explicit
  wires.
- Gateway split work must keep endpoint behavior stable through helper and route
  registration tests.
- Navigation closure tests must prove that global path, waypoint dispatch,
  visual servo, traversability fusion, localization speed scaling, and cmd_vel
  priority still form a reachable control chain.
