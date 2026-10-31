# Host runtime

`runtime/` owns how typed Python Modules are constructed, connected, started,
and stopped inside one Host. It does not decide which Product runs or which
native processes to launch.

## Find the owner

| Work | Entry |
| --- | --- |
| Module lifecycle and ports | `module.py`, `stream.py` |
| Host construction and wiring mechanism | `host.py`, `wiring.py` |
| Backend lookup | `registry.py` |
| AI skill declaration and shared discovery | `module.py`, `skills.py` |
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
SystemHandle = the running Host graph
```

Module lifecycle is `preflight -> setup -> start -> stop`. Assembly selects
the Modules; `host.py` implements their construction and lifecycle. Its
`HostBuilder` helper stays inside the assembly implementation. Native
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

Connect Modules through the Host's declared wires; resolve backend implementations
through the existing registry. Do not introduce a second orchestration layer
inside a Module.

## Skill contract

`@skill` marks an existing Module method for discovery. Bare `@skill` remains
supported; use the configured form when the caller needs parameter or result
semantics:

```python
@skill(
    description="Query a Follow by the exact task/session pair returned by start_follow.",
    parameters={
        "follow_task_id": "Follow task ID, not a person tracker ID.",
        "product_session_id": "Product session returned with the task.",
    },
    output_schema=FOLLOW_TASK_SCHEMA,
    read_only=True,
)
def get_follow_task(self, follow_task_id: str, product_session_id: str) -> dict:
    ...
```

The Python signature remains the source of input types, required arguments and
defaults. `parameters` only adds descriptions; a name absent from the signature
is an authoring error. `description` explains when to call, the operation's
effect, what its result proves, and any follow-up query. `output_schema` describes
an object result; `read_only` maps to MCP's `readOnlyHint`. Omit an unknown hint
rather than labelling every old method as read-only.

`SkillInfo.to_tool()` renders the descriptor. `skills.discover_skills()` binds
one descriptor and method per name, collapses aliases of the same instance, and
rejects different owners with the same name. MCP explicitly retains its two
built-in fallback names; Agent supplies its existing blocklist before discovery
and keeps task-owned motion handlers. Neither adapter silently replaces a
different owner.

MCP returns object results as both `structuredContent` and JSON text. A legacy
JSON string is decoded only for a declared object output contract. Agent tool
conversion includes the result schema and behavior hint in its function
description, because OpenAI function tools have no MCP `outputSchema` field.
Agent object/list results use JSON text rather than Python dictionary repr.
Input signature binding and existing domain validation remain in force; this
upgrade does not add a general JSON Schema value validator.

The declaration owns no execution state. Follow's admission receipt is not
completion; query by `follow_task_id` plus `product_session_id`, and wait for
the domain terminal result after cancellation. Native navigation, Product
readiness, task ownership and motion authorization retain their existing owners.
These are protocol contracts, following the
[MCP tool specification](https://modelcontextprotocol.io/specification/2025-11-25/server/tools),
not a new task engine or a navigation algorithm.

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
