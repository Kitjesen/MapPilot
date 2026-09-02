# LingTu Product Control

A Product is one complete robot working mode. `ProductControl` fixes one Robot
and one Env (`real` or `sim`), then exposes only `switch`, `status`, and `stop`.
Process selection, Host assembly, readiness, rollback, and the internal plan
file stay behind that interface.

Remote control of an already deployed robot belongs to `lingtu.sdk`, not this
package.

## Quick Start

```python
from lingtu.control import ProductControl

control = ProductControl(robot="doso/thunder_v4", env="sim")
control.switch("teleop")
control.status()
control.stop()
```

## Public API

| Entry | Purpose |
|-------|---------|
| `ProductControl` | Local `switch`, `status`, and `stop` |
| `lingtu.sdk` | Remote client for an already running robot |

`assembly`, real/sim switching, process runners, and plan files are internal
implementation. Blueprint still owns only Modules inside the Python Host;
ProductControl owns the complete Product lifecycle.

`switch()` returns the committed `product_session_id` and typed readiness
evidence, but never exposes the RunPlan path. `stop()` accepts optional
`expected_product` and `expected_product_session_id` guards so in-repo
coordinators can stop only the exact Product run they started.

Standalone camera, LiDAR, SLAM, navigator, and detector wrappers were removed.
Those capabilities live in domain Module packages and are selected by
`lingtu.assembly`; the generic Blueprint mechanism remains in
`runtime.blueprint`.

Blueprint is active only inside the Python Host. It does not start LiDAR, SLAM,
navigation, traversability, or driver services.

## Layout

| Path | Responsibility |
| --- | --- |
| `control.py` | Public `switch`, `status`, `stop` and the CLI |
| `run_plan.py` | Internal resolved Product data and JSON load/save |
| `real/` | Real-robot switch transaction and systemd ownership |
| `sim/` | Simulation switch transaction and direct child ownership |
| `assembly/` | Product compilation and Host Blueprint construction |
| `sdk/` | Remote Gateway client |

Product behavior is declared in `config/runtime_graph/products/*.yaml`.
Assembly translates `host.capabilities` into Modules and writes final validated
navigation values to RunPlan v8 `launch.parameters`. ProductControl applies
`--set` during compilation; runners only execute the published RunPlan.

## Testing

```bash
python -m pytest \
  tests/lingtu \
  tests/lingtu/real \
  tests/lingtu/sim \
  tests/lingtu/assembly \
  tests/lingtu/sdk
```
