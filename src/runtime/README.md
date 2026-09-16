# Host runtime

`runtime/` owns how typed Python Modules are constructed, connected, started,
and stopped inside one Host. It does not decide which Product runs or which
native processes to launch.

## Find the owner

| Work | Entry |
| --- | --- |
| Module lifecycle and ports | `module.py`, `stream.py` |
| Host construction and wiring mechanism | `blueprint.py`, `wiring.py` |
| Backend lookup | `registry.py` |
| In-process payloads and shared values | `msgs/`, `contracts/` |
| Local delivery and external route metadata | `transport/local.py`, `route_contract/` |
| Coordinate frames and transforms | `tf/` |
| DDS-to-Module conversion | `endpoints/dds/adapters.py` |
| Legacy topic aliases | `adapters/topics.py` |
| Product/Env resolution and process contracts | [`../lingtu/assembly/graph/`](../lingtu/assembly/graph/) |
| Product compilation and Host recipes | [`../lingtu/assembly/`](../lingtu/assembly/) |
| Topics, wire fields, and numeric enums | [`../message/`](../message/README.md) |
| Diagnostic projections and field evidence | [`../diagnostics/`](../diagnostics/README.md) |

## Host model

```text
Module       = one typed in-process runtime unit
In / Out     = input/output ports
WireSpec     = one declared connection
Blueprint    = construction and wiring before startup
SystemHandle = the running Host graph
```

Module lifecycle is `preflight -> setup -> start -> stop`. Assembly selects
the Modules; Blueprint implements their construction and lifecycle. Native
planning, map management, safety arbitration, and device I/O stay with their
domain owners.

ProductControl owns the whole Product lifecycle. Its RunPlan and process
contracts belong to `lingtu/`, not to this generic framework.

## Communication

Module-to-Module delivery is local. External route metadata records the topic,
transport, and single-writer constraints at a process seam; it does not alter
the Host's internal wires.

`route_contract.robot()` reads only `message/catalog.py`. The same catalogue
reader feeds the message generator and Product graph. It does not load Product
or Env YAML, import assembly, or start a process.

The replay preset describes explicit LCM compatibility bindings.
`route_contract.sim()` describes local development delivery; it is not
`env=sim` and does not launch a simulator. Real and simulation Products use
their resolved native DDS process contracts.

## Minimal Module

```python
from runtime.module import Module
from runtime.stream import In, Out


class Doubler(Module):
    value: In[float]
    doubled: Out[float]

    def setup(self):
        self.value.subscribe(lambda x: self.doubled.publish(x * 2.0))
```

Connect Modules through Blueprint wires; resolve backend implementations
through the existing registry. Do not introduce a second orchestration layer
inside a Module.

## Dependency rule

`runtime` must not import `lingtu`, `diagnostics`, Gateway, or domain
implementations. `message` must not import Host runtime or Product assembly.
These rules are enforced in `config/architecture_layers.yaml`.

There is no `runtime.graph`, `ProductRuntime`, or `runtime_interface.py`
compatibility facade. Callers use the owning package directly.

## Focused checks

```bash
python -m pytest tests/runtime/test_route_contract.py tests/runtime/test_topic_catalog_generation.py
python -m pytest tests/contracts/test_runtime_architecture_boundaries.py
```
