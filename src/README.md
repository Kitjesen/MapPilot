# src/ - LingTu source layout

`src/` is organized by runtime infrastructure plus functional domains. Normal
Modules depend on `runtime/`, communicate through typed ports, and are assembled
by Blueprints. Domain packages should not import each other for control flow.

## Top-level packages

| Package | Role | Put here |
| --- | --- | --- |
| `runtime/` | Module framework and runtime infrastructure. | `Module`, `In`/`Out`, messages, registry, Blueprint, profiles, transports, TF, device utilities. |
| `lingtu/` | Product-facing local API and runtime handoff. | `Robot`, profile resolution entry points, local package API. |
| `nav/` | Navigation domain. | Mission FSM, goal/map services, global planning, local planning, safety, exploration, nav kernel binding. |
| `perception/` | Scene perception domain. | Detectors, encoders, trackers, scene graph, reconstruction. |
| `decision/` | Goal reasoning and semantic action domain. | Semantic planner, LLM wrapper, goal resolver, visual servo, task decomposition. |
| `memory/` | Memory domain. | Semantic map, episodic/tagged/vector/temporal memories, KG-backed stores. |
| `drivers/` | Hardware and simulation endpoints. | Real/sim robot drivers, LiDAR/camera/GNSS sources, driver adapters. |
| `slam/` | SLAM and localization domain. | SLAM bridge, localization, relocalization, GNSS fusion, portable SLAM adapters. |
| `gateway/` | External interface domain. | REST/SSE/WS, MCP, media, visualization, command/status services. |
| `kernels/` | Cross-domain portable compute kernels. | Rust/C ABI kernels that are not owned by one Python Module package. |

`src/nav/kernel/` is the navigation C++ kernel. `src/kernels/` is for
cross-domain portable kernels such as SLAM pose-graph, calibration, and planning
optimizers.

## Main runtime entry

```text
lingtu.py
  -> cli/main.py
  -> lingtu.runtime / runtime.profiles
  -> runtime.blueprints
  -> Module graph
```

Blueprint stack factories live under `src/runtime/blueprints/stacks/`.

## Navigation chain

```text
Gateway / MCP / CLI
  -> GoalService
  -> Navigation
  -> GlobalPlanner
  -> LocalPlanner
  -> PathFollower
  -> VelocityMux
  -> Driver
```

`GlobalPlanner` is an internal planning service used by `Navigation`; it is not
a peer Module in the runtime graph.

## Boundary rules

```text
All Modules -> runtime/

nav/        must not import perception/, decision/, drivers/, gateway/
perception/ must not import nav/, drivers/, gateway/
decision/   may consume perception/memory messages through ports, not direct runtime ownership
drivers/    must not import nav/ or decision/ for behavior
gateway/    must not own planning, perception, SLAM, or driver algorithms
```

External protocols stay under the owning domain's `adapters/` folder, for
example `nav/adapters/ros2/`, `slam/adapters/`, `drivers/adapters/`, and
`runtime/adapters/`.

## Tests

| Location | Use |
| --- | --- |
| `runtime/tests/` | Framework, profile, Blueprint, and cross-domain contract gates. |
| `<domain>/tests/` | Domain-owned unit tests. |
| `tests/contracts/` | Repository-wide migration and boundary scans. |
| `sim/tests/` | Simulation integration and validation gates. |

Run the narrowest relevant tests first, then broaden when a change crosses
domain boundaries.
