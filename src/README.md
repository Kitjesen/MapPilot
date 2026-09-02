# src/ - LingTu source layout

`src/` uses one placement rule: code lives with the function that owns it.
Products do not get parallel source trees. Product declarations live in
[`config/runtime_graph/products/`](../config/runtime_graph/products/), while
[`lingtu/`](lingtu/README.md) resolves those declarations and owns their
lifecycle. A Product selects functional code; it does not own that code.

Normal Modules depend on `runtime/`, communicate through typed ports, and are
selected by `lingtu.assembly`. Domain packages should not import each other for
control flow.

## Source families

The physical layout stays flat. The family names below are navigation labels,
not directories: do not create `src/product_control/`, `src/capabilities/`,
`src/platform/`, or `src/compute/`.

### product_control — Product 控制面

| Package guide | Owns |
| --- | --- |
| [`lingtu/`](lingtu/README.md) | ProductControl, RunPlan, Product assembly, and real/sim lifecycle routing. |

### capabilities — 机器人功能域

| Package guide | Owns |
| --- | --- |
| [`decision/`](decision/README.md) | Goal reasoning, semantic planning, task decomposition, and visual servo. |
| [`drivers/`](drivers/README.md) | Robot and sensor backends plus hardware/simulation adapters. |
| [`explore/`](explore/README.md) | Frontier selection, TARE, exploration supervision, native policy, and the exploration endpoint source. |
| [`localization/`](localization/README.md) | SLAM, localization, relocalization, and GNSS fusion. |
| [`maps/`](maps/README.md) | Live and persistent map services, layers, stores, and native map components. |
| [`memory/`](memory/README.md) | Semantic, episodic, tagged, vector, temporal, and graph-backed memory. |
| [`nav/`](nav/README.md) | Navigation commands, goals, skills, planning, tracking, safety, and native endpoint composition. |
| [`perception/`](perception/README.md) | Detection, encoding, tracking, scene graph, and reconstruction. |

### platform — 共享平台与接口

| Package guide | Owns |
| --- | --- |
| [`runtime/`](runtime/README.md) | Module, ports, registry, Blueprint, transports, TF, and runtime infrastructure. |
| [`message/`](message/README.md) | Native cross-process topic metadata, IDL, QoS, and generated contracts. |
| [`gateway/`](gateway/README.md) | REST, SSE, WebSocket, MCP, media, visualization, and external command/status services. |
| [`diagnostics/`](diagnostics/README.md) | Field readiness, acceptance, and runtime evidence helpers. |

### compute — 共享底层计算

| Package guide | Owns |
| --- | --- |
| [`kernels/`](kernels/README.md) | Portable Rust/C ABI compute kernels shared across functional domains. |
| [`native/`](native/README.md) | Shared native services that belong to neither one domain nor one portable kernel. |

`real/` and `sim/` below an owner describe environment implementations, not
Product ownership. `src/nav/cpp/` remains the canonical navigation
implementation. Exploration endpoint sources live in
`src/explore/cpp/endpoint/`; the existing nav endpoint CMake still composes the
public `lingtu_explore_dds` executable. `src/kernels/` is reserved for portable
compute shared by more than one domain.

Create lower-level folders only when the owner needs them: `modules/`,
`adapters/<protocol>/`, `cpp/` or `rust/`, `real/` or `sim/`, `tests/`, and
`contracts/`. Do not add ambiguous buckets such as `common/`, `misc/`,
`helpers/`, `new/`, or `v2/`.

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
