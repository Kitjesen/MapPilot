# `src/runtime` File Map

Use this file as a human index. It answers two questions:

1. Where should I look first?
2. Does this file really belong in `src/runtime`?

## Fast Routing

| Task | File or folder |
| --- | --- |
| module lifecycle, skills, RPC decorators | `module.py` |
| input/output ports and backpressure | `stream.py` |
| module graph builder and explicit wires | `blueprint.py` |
| backend/plugin registration | `registry.py` |
| product stack factories | `blueprints/stacks/` |
| product data-flow wiring | `blueprints/wires/` |
| product-specific assembly | `blueprints/products/` |
| profile-to-blueprint build path | `blueprints/profile_builder.py` |
| runtime profile catalog | `profiles/catalog/` |
| topics, frames, runtime names | `runtime_interface.py` |
| shared message dataclasses | `msgs/` |
| shared runtime contract dataclasses | `contracts/` |

## Core Kernel

These are the files most modules are allowed to depend on directly.

| Path | Role |
| --- | --- |
| `module.py` | Module base class, lifecycle, `@skill`, `@rpc` |
| `stream.py` | `In[T]` / `Out[T]` ports and backpressure policies |
| `blueprint.py` | Blueprint builder, explicit wires, system handle |
| `registry.py` | Backend and plugin registry |
| `clock.py` | Real/sim clock abstraction |

## Shared Contracts

These define common language between modules. They should stay ROS-free.

| Path | Role |
| --- | --- |
| `msgs/` | Shared message dataclasses |
| `contracts/` | Runtime contract dataclasses, including simulation contracts |
| `runtime_interface.py` | Canonical topics, frames, remappings, runtime manifest |
| `profiles/` | Runtime resolver, catalog, endpoint config, binding policy |
| `diagnostics/` | Runtime evidence, acceptance, and audit tooling |

## Product Assembly

These files decide which modules exist and how data moves between them. They
should describe structure, not implement algorithms.

| Path | Role |
| --- | --- |
| `blueprints/stacks/` | Reusable stack factories such as driver, maps, navigation, gateway |
| `blueprints/wires/` | Explicit product data-flow definitions |
| `blueprints/products/` | Product-specific stack composition |
| `blueprints/full_stack_wiring.py` | Applies wire specs to the built module graph |
| `blueprints/profile_builder.py` | Profile-to-blueprint entry |
| `introspection/profile_graph.py` | Profile graph inspection helpers |
| `introspection/module_graph.py` | Blueprint-exported module graph manifest |

## Platform-Neutral Helpers

These provide shared plumbing. They should not make navigation decisions.

| Path | Role |
| --- | --- |
| `transport/` | Local, DDS, SHM, LCM transport implementations |
| `portable/` | Portable topic transport and frame contracts |
| `tf/` | Lightweight transform tree and buffer |
| `devices/` | Device manager and decoder abstractions |
| `config.py`, `config_loader.py`, `yaml_helpers.py` | Config loading |
| `service_manager.py`, `external_service_module.py` | External service lifecycle |
| `utils/` | Small cross-cutting helpers |
| `resource_monitor/` | Resource monitor |
| `introspection/` | Graph and text visualization |
| `diagnostics/` | Acceptance gates, evidence reports, and audit helpers |

## Diagnostics And Audit Material

These files are useful, but they are not the clean runtime kernel. They live in
`diagnostics/` so the runtime root stays focused on module lifecycle, transport,
profiles, and product graph assembly. Treat them as managed debt: keep them
working, but avoid adding new product behavior here.

| Path | Role |
| --- | --- |
| `diagnostics/runtime_evidence.py` | Runtime evidence report tooling |
| `diagnostics/dimos_runtime_dataflow.py` | Data-flow audit tooling |
| `diagnostics/dimos_gap.py` | Gap audit tooling |
| `diagnostics/gateway_runtime_acceptance.py` | Gateway acceptance evaluator |
| `diagnostics/inspection_acceptance.py` | Inspection acceptance evaluator |
| `diagnostics/product_field_check.py` | Field deployment sanity checks |
| `diagnostics/runtime_validation_gates.py` | Runtime validation gates |
| `diagnostics/migration_catalog.py` | Package and kernel migration catalog |
| `diagnostics/efficiency_status.py` | Benchmark claim metadata and status helpers |

## Legacy Compatibility Material

These files and packages still exist for compatibility or tooling. They should
not become product behavior.

| Path | Role |
| --- | --- |
| `native_module.py` | Legacy native/ROS process wrapper |
| `native_install.py` | Legacy install-layout helper |
| `adapters/ros2/` | ROS 2 compatibility adapters only |
| `dds.py`, `adapters/dds/` | Python DDS utilities; field control loop belongs to native C++ DDS services |
| `runtime_policy.py` | Runtime policy checks |
| `runtime_switch.py` | Runtime override/switch support |
| `worker.py`, `worker_manager.py`, `coordinator.py` | Worker helpers |

## Placement Rule

If a file knows about a specific planner, detector, robot behavior, REST/MCP
endpoint, or ROS node, it usually does not belong in `src/runtime`.

Use these destinations instead:

| Code knows about... | Put it in |
| --- | --- |
| navigation missions, waypoints, safety, local planner flow | `src/nav/` |
| global planning algorithms or OctoPlanner3D/PCT adapters | `src/nav/services/plan/` |
| terrain or C++ autonomy kernels | `src/nav/local/`, `src/nav/kernel/` |
| SLAM/localization algorithms | `src/localization/` |
| perception, memory, decision goals, visual servo | `src/perception/`, `src/decision/`, `src/memory/` |
| robot hardware, teleop hardware, simulated drivers | `src/drivers/` |
| HTTP, WebSocket, SSE, MCP API behavior | `src/gateway/` |
| ROS 2 compatibility | `src/*/adapters/ros2/` |
