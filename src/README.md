# src/ - LingTu source layout

`src/` uses one placement rule: code lives with the function that owns it.
Products do not get parallel source trees. Product declarations live in
[`config/runtime_graph/products/`](../config/runtime_graph/products/), while
[`lingtu/`](lingtu/README.md) resolves those declarations and owns their
lifecycle. A Product selects functional code; it does not own that code.

Normal Modules depend on `runtime/`, communicate through typed ports, and are
selected by `lingtu.assembly`. Domain packages should not import each other for
control flow.

## Mental model

`src/` answers **who owns this behavior?** It does not answer which Product or
environment happens to run it.

| Question | Source of truth |
| --- | --- |
| Which capabilities make up a Product? | [`config/runtime_graph/products/`](../config/runtime_graph/products/) |
| Does it run in `real` or `sim`? | [`config/runtime_graph/envs/`](../config/runtime_graph/envs/) and `lingtu/` lifecycle code |
| Where does production behavior belong? | The functional owner under this directory |
| Where do simulation packages, worlds, evaluation, and distribution belong? | Root [`sim/`](../sim/README.md), the simulation workspace |
| Where do compiled and release outputs belong? | Root `build/`, `install/`, and `dist/`. Checked-in generated message views remain beside their schema sources. |

## Directory map

The physical layout is intentionally flat. The category column is only a
reading aid; do not create wrapper directories such as `capabilities/`,
`platform/`, `compute/`, `common/`, or `misc/`.

| Category | Package guide | Owns |
| --- | --- | --- |
| Product control | [`lingtu/`](lingtu/README.md) | ProductControl, RunPlan, Product assembly, and real/sim lifecycle routing. |
| Runtime platform | [`runtime/`](runtime/README.md) | Host Module, ports, registry, Blueprint, local delivery, and TF. No Product parsing or process orchestration. |
| Runtime platform | [`message/`](message/README.md) | Native cross-process topic metadata, IDL, QoS, and generated contracts. |
| Runtime platform | [`gateway/`](gateway/README.md) | REST, SSE, WebSocket, MCP, media, visualization, and external command/status services. |
| Runtime platform | [`diagnostics/`](diagnostics/README.md) | Field readiness, acceptance, and runtime evidence helpers. |
| Capability | [`drivers/`](drivers/README.md) | Robot and sensor backends plus hardware/simulation adapters. |
| Capability | [`localization/`](localization/README.md) | SLAM, localization, relocalization, and GNSS fusion. |
| Capability | [`maps/`](maps/README.md) | Live and persistent map services, layers, stores, pruning, and native map components. |
| Capability | [`nav/`](nav/README.md) | Navigation commands, goals, skills, planning, tracking, safety, and native endpoint composition. |
| Capability | [`explore/`](explore/README.md) | Frontier selection, TARE, exploration supervision, native policy, and exploration endpoint source. |
| Capability | [`perception/`](perception/README.md) | Detection, encoding, tracking, scene graph, and reconstruction. |
| Capability | [`decision/`](decision/README.md) | Goal reasoning, semantic planning, task decomposition, and visual servo. |
| Capability | [`memory/`](memory/README.md) | Semantic, episodic, tagged, vector, temporal, and graph-backed memory. |
| Shared compute | [`kernels/`](kernels/README.md) | Portable, cross-domain compute kernels implemented in Rust or C++ behind a stable ABI. |
| Shared native | [`native/`](native/README.md) | Native services that belong to neither one functional domain nor one portable kernel. |

`real/` and `sim/` below an owner describe environment implementations, not
Product ownership. `src/nav/cpp/` remains the canonical navigation
implementation. Exploration endpoint sources live in
`src/explore/cpp/endpoint/`; the existing nav endpoint CMake still composes the
public `lingtu_explore_dds` executable. `src/kernels/` is reserved for portable
compute shared by more than one domain.

Create lower-level folders only when the owner needs them: `modules/`,
`adapters/<protocol>/`, `cpp/` or `rust/`, and `real/` or `sim/`.
Repository-owned tests and contracts live under root `tests/`, grouped by
source owner. Do not add ambiguous buckets such as `common/`, `misc/`,
`helpers/`, `new/`, or `v2/`.

## Source and install boundary

Language does not decide the top-level directory; ownership does. Python owns
the Host, semantic, and API layers. C++ owns field LiDAR, SLAM, maps, terrain,
navigation, and driver hot paths. Rust is used for bounded portable kernels
where it already has a measured or integration-backed reason to exist.

```text
src/<owner>/                              checked-in source
  -> build/<component>/                   ignored compilation tree
  -> install/<platform>-<arch>/<config>/  ignored bin/lib/etc/share prefix
  -> dist/                                ignored release archives
```

Do not add `src/bin/`, a tracked root `bin/`, or production references to
`build/`. Python user commands come from `pyproject.toml [project.scripts]`;
native executables are installed by their owning CMake project.

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
| `tests/<owner>/` | Unit and integration tests owned by one source domain. |
| `tests/contracts/` | Repository-wide migration and boundary scans. |
| `tests/integration/` | Tests spanning two or more source owners. |
| `tests/sim/` | Simulation integration and validation gates. |

Run the narrowest relevant tests first, then broaden when a change crosses
domain boundaries.
