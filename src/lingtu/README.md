# LingTu Product API

`src/lingtu` is the product-facing package. It compiles a Profile and Endpoint
into one `Product`, builds its application Blueprint, and delegates native
process lifecycle to Launcher. Domain algorithms do not live here.

Remote control of an already deployed robot belongs to `lingtu.sdk`, not this
package.

## Quick Start

```python
from lingtu import Robot

robot = Robot("sim").start()
robot.go_to(5.0, 3.0, z=0.0)
robot.go("go to the gym")
robot.follow("person in red")
robot.save_map("building_a")
robot.shutdown()
```

## Public API

| Entry | File | Purpose |
|-------|------|---------|
| `Robot` | `robot.py` | All-in-one local robot facade over the runtime profile builder |
| `lingtu.runtime` | `runtime.py` | Resolve profiles and build Module-First systems directly |
| `lingtu.assembly` | `assembly/` | LingTu product recipes: products, stacks, wires, and static graph inspection |
| `ProductControl` | `control.py` | Resolve the active Profile/Endpoint and perform plan-owned process operations |
| `Launcher` | `launcher.py` | Apply, stop, or restart native RuntimePlan processes with readiness evidence |
| `lingtu.plugin_seed` | `plugin_seed.py` | Product plugin catalog; imports built-in module groups so their `@register` decorators populate `runtime.registry` |
| `lingtu.sdk` | `sdk/` | Remote client SDK for an already running robot or gateway |

Standalone camera, LiDAR, SLAM, navigator, and detector wrappers were removed.
Those capabilities live in domain Module packages and are selected by
`lingtu.assembly`; the generic Blueprint mechanism remains in
`runtime.blueprint`.

## Plugin Seed Boundary

`lingtu.plugin_seed` answers "which built-in LingTu modules are part of this
product package?" It does not select the active profile and does not build a
system. Runtime code calls `seed_builtin_plugins(groups=(...))` when it needs a
category in the registry.

The product global-planner category seeds `octoplanner3d` and the explicit
native `far` option. OctoPlanner3D remains the default; FAR selection must come
from the resolved product/runtime contract. Legacy PCT/A*/direct planning code
stays outside the product backend catalog.

## Testing

```bash
python -m pytest src/lingtu/tests/
```
