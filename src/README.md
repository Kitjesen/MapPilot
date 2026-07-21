# src/ - LingTu source layout

`src/` is organized by runtime infrastructure, product assembly, and functional
domains. Normal Modules depend on `runtime/`, communicate through typed ports,
and are selected by `lingtu.assembly`. Domain packages should not import each
other for control flow.

## Top-level packages

| Package | Role | Put here |
| --- | --- | --- |
| `runtime/` | Module framework and runtime infrastructure. | `Module`, `In`/`Out`, messages, registry, Blueprint, profiles, transports, TF, device utilities. |
| `lingtu/` | Product-facing API, assembly, and runtime handoff. | `Robot`, product Module recipes, profile build entry points, local package API. |
| `nav/` | Navigation domain. | Mission FSM, goal services, global planning, local planning, safety, exploration, inspection route execution, native endpoint shell. |
| `perception/` | Scene perception domain. | Detectors, encoders, trackers, scene graph, reconstruction. |
| `decision/` | Goal reasoning and semantic action domain. | Semantic planner, LLM wrapper, goal resolver, visual servo, task decomposition. |
| `memory/` | Memory domain. | Semantic map, episodic/tagged/vector/temporal memories, KG-backed stores. |
| `drivers/` | Hardware and simulation endpoints. | Real/sim robot drivers, LiDAR/camera/GNSS sources, driver adapters. |
| `localization/` | SLAM and localization domain. | Native SLAM status/localization, relocalization, GNSS fusion, ROS-free and compatibility adapters. |
| `maps/` | Map domain. | Map service facade, persistent map packages, C++ map store/build artifacts, live map-layer Modules. |
| `message/` | Cross-process wire contracts. | DDS topic registry, IDL, Python DDS type tags, and C++ topic/type aliases. |
| `gateway/` | External interface domain. | REST/SSE/WS, MCP, media, visualization, command/status services. |
| `kernels/` | Cross-domain portable compute kernels. | Rust/C ABI kernels that are not owned by one Python Module package. |
| `diagnostics/` | Field diagnostics. | Readiness, inspection, deployment, and runtime evidence helpers. |

`src/nav/cpp/` is the canonical navigation C++ tree; `src/nav/kernel/` is only its Python extension loader. `src/kernels/` is for
cross-domain portable kernels such as SLAM pose-graph, calibration, and planning
optimizers.

## Main runtime entry

```text
lingtu.py
  -> cli/main.py
  -> lingtu.runtime / runtime.profiles
  -> lingtu.assembly
  -> Module graph
```

Product stack factories live under `src/lingtu/assembly/stacks/`; the generic
graph mechanism remains `src/runtime/blueprint.py`.

## Navigation chain

```text
Gateway / MCP / CLI
  -> GoalService
  -> native Nav Endpoint or Navigation Module
  -> OctoPlanner3D / local planner / path follower
  -> /nav/cmd_vel
  -> lingtu-driver
```

`GlobalPlanner` is an internal planning service used by `Navigation`; it is not
a peer Module in the Python runtime graph. The default physical
`thunder_field` product path uses native typed DDS services and the unique
`lingtu-driver` hardware sink; simulation and compatibility profiles may still
use the Python Module-owned local autonomy chain.

## Boundary rules

```text
All Modules -> runtime/

nav/        must not import perception/, decision/, drivers/, gateway/
perception/ must not import nav/, drivers/, gateway/
decision/   may consume perception/memory messages through ports, not direct runtime ownership
drivers/    must not import nav/ or decision/ for behavior
gateway/    must not own planning, perception, SLAM, or driver algorithms
```

External protocols stay under the owning domain's `adapters/` folder or a
typed endpoint boundary, for example compatibility adapters in their owning
domain, `runtime/adapters/`, `runtime/endpoints/dds/`, and
`message/idl/`. ROS adapters are compatibility-only; product process
boundaries use native typed DDS/SHM contracts.

## Tests

| Location | Use |
| --- | --- |
| `runtime/tests/` | Framework, profile, Blueprint, and cross-domain contract gates. |
| `<domain>/tests/` | Domain-owned unit tests. |
| `tests/contracts/` | Repository-wide migration and boundary scans. |
| `sim/tests/` | Simulation integration and validation gates. |

Run the narrowest relevant tests first, then broaden when a change crosses
domain boundaries.
