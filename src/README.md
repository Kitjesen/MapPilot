# src/ - LingTu source layout

`src/` is organized by runtime infrastructure, product assembly, and functional
domains. Normal Modules depend on `runtime/`, communicate through typed ports,
and are selected by `lingtu.assembly`. Domain packages should not import each
other for control flow.

## Top-level packages

The physical layout stays flat; these are the five ownership groups:

```text
foundation       runtime/  message/
product          lingtu/
robot capability drivers/  localization/  maps/  nav/  explore/
                 perception/  decision/  memory/
external/ops     gateway/  diagnostics/
shared native    native/  kernels/
```

| Package | Role | Put here |
| --- | --- | --- |
| `runtime/` | Module framework and runtime infrastructure. | `Module`, `In`/`Out`, messages, registry, Blueprint, profiles, transports, TF, device utilities. |
| `lingtu/` | Product control, assembly, and runtime handoff. | `ProductControl`, RunPlan, real/sim lifecycle, Host assembly, remote SDK. |
| `nav/` | Navigation domain. | Host commands/goals/skills, inspection integration, native C++ planning and endpoint code. |
| `explore/` | Exploration domain. | Frontier selection, TARE integration, exploration SDK, and native exploration kernels. |
| `perception/` | Scene perception domain. | Detectors, encoders, trackers, scene graph, reconstruction. |
| `decision/` | Goal reasoning and semantic action domain. | Semantic planner, LLM wrapper, goal resolver, visual servo, task decomposition. |
| `memory/` | Memory domain. | Semantic map, episodic/tagged/vector/temporal memories, KG-backed stores. |
| `drivers/` | Hardware and simulation endpoints. | Real/sim robot drivers, LiDAR/camera/GNSS sources, driver adapters. |
| `localization/` | SLAM and localization domain. | Native SLAM status/localization, relocalization, GNSS fusion, ROS-free and compatibility adapters. |
| `maps/` | Map domain. | Map service facade, persistent map packages, C++ map store/build artifacts, live map-layer Modules. |
| `message/` | Cross-process wire contracts. | Native topic metadata, IDL, and C++ topic/QoS contracts. |
| `gateway/` | External interface domain. | REST/SSE/WS, MCP, media, visualization, command/status services. |
| `kernels/` | Cross-domain portable compute kernels. | Rust/C ABI kernels that are not owned by one Python Module package. |
| `native/` | Shared native services. | Cross-domain C++ runtime support, MCAP recording, and replay components. |
| `diagnostics/` | Field diagnostics. | Readiness, inspection, deployment, and runtime evidence helpers. |

`src/nav/cpp/` is the canonical navigation implementation. `src/kernels/` is
reserved for portable compute shared by more than one domain.

## Runtime entries

```text
installed lingtu / python -m lingtu.control
  -> ProductControl -> RunPlan -> real or sim lifecycle

python -m lingtu.real.host
  -> published RunPlan -> lingtu.assembly -> managed Host Module graph
```

Product stack factories live under `src/lingtu/assembly/stacks/`; the generic
graph mechanism remains `src/runtime/blueprint.py`.

## Navigation chain

```text
Gateway / MCP / CLI
  -> GoalService
  -> Commands / native adapter
  -> typed DDS nav endpoint
  -> native planner / local controller
  -> rt/nav/cmd_vel
  -> lingtu-driver
```

Both `env=real` and `env=sim` use the native endpoint shape. Development Host
Blueprints keep the same command and status contracts; they do not install a
second Python planner, tracker, safety mux, or motion controller.

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
