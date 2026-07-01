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
| runtime profile catalog | `runtime/catalog/` |
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
| `contracts/` | Runtime contract dataclasses |
| `runtime_interface.py` | Canonical topics, frames, remappings, runtime manifest |
| `runtime/` | Runtime resolver, catalog, endpoint config, binding policy |

## Product Assembly

These files decide which modules exist and how data moves between them. They
should describe structure, not implement algorithms.

| Path | Role |
| --- | --- |
| `blueprints/stacks/` | Reusable stack factories such as driver, maps, navigation, gateway |
| `blueprints/wires/` | Explicit product data-flow definitions |
| `blueprints/products/` | Product-specific stack composition |
| `blueprints/full_stack.py` | Compatibility full-stack entry |
| `blueprints/full_stack_wiring.py` | Applies wire specs to the built module graph |
| `blueprints/profile_builder.py` | Profile-to-blueprint entry |
| `blueprints/profile_graph.py` | Graph inspection helpers |
| `blueprints/catalog/` | Compatibility facade for `runtime/catalog/` |

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

## Legacy Or Audit Material

These files are useful, but they are not the clean runtime kernel. Treat them
as managed debt: keep them working, but avoid adding new product behavior here.

| Path | Role |
| --- | --- |
| `native_module.py` | Legacy native/ROS process wrapper |
| `native_install.py` | Legacy install-layout helper |
| `runtime_evidence.py` | Runtime evidence report tooling |
| `dimos_runtime_dataflow.py` | Data-flow audit tooling |
| `dimos_gap.py` | Gap audit tooling |
| `gateway_runtime_acceptance.py` | Gateway acceptance evaluator |
| `inspection_acceptance.py` | Inspection acceptance evaluator |
| `product_field_check.py` | Field deployment sanity checks |
| `runtime_validation_gates.py` | Runtime validation gates |
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
