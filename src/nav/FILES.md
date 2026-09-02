# Navigation file index

This is a short index of the current navigation implementation.

| Need | Start here |
| --- | --- |
| Product navigation flow | `README.md`, `cpp/endpoint/README.md` |
| Host goal/cancel/stop entry | `services/goals.py`, `commands/module.py` |
| Agent/MCP navigation surface | `skills/skills_module.py` |
| Native client boundary | `adapters/native/` |
| Inspection | `inspection/service.py`, `cpp/inspection/`, `cpp/endpoint/nav/runtime/inspection/` |
| Global planning | `cpp/planning/global/` |
| Local planning | `cpp/planning/local/` |
| CMU path bank | `cpp/planning/local/cmu/paths/` |
| Tracking and route execution | `cpp/tracking/`, `cpp/navigation/` |
| Final safety and authority | `cpp/endpoint/nav/safety/`, `cpp/endpoint/nav/control/` |
| DDS process boundary | `cpp/endpoint/nav/dds/`, `cpp/endpoint/nav/runtime/` |
| Native command client | `cpp/client/` |
| Tests | `../../tests/nav/`, `../../tests/explore/` |

## Runtime split

The Host accepts commands and presents capabilities. Native `navd` performs
global planning, local planning, tracking, recovery, safety, and final command
publication. This ownership is the same in `real` and `sim`; there is no Python
planner or motion-control fallback.

## Dependency direction

```text
Host skills/services -> native adapters -> navd

navd endpoint -> navigation -> planning + tracking
navd endpoint -> safety + control authority -> DDS command output
```

Planning and tracking code do not depend on DDS or Product lifecycle. Endpoint
code owns transport and orchestration, while the driver remains the hardware
command sink.
