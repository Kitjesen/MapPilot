# LingTu Product API

`src/lingtu` owns Product assembly and field control. A Product is an
environment-independent mode declaration. `ProductControl`, fixed to either
`real` or `sim`, compiles it into one fingerprinted `RunPlan`, stages the
transient session, and applies its processes. The Python Host consumes that
exact RunPlan; domain algorithms do not live here.

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
| `lingtu.runtime` | `runtime.py` | Resolve local-development Profiles and build their Python Host graph |
| `lingtu.assembly` | `assembly/` | Compile Products and assemble Host stacks, wires, and static graph inspection |
| `RunPlan` | `run_plan.py` | Immutable, environment-resolved Host/process contract and fingerprint |
| `ProductControl` | `control.py` | Single API for Product planning, switching, stopping, and restart |
| Product switch | `product_switch.py` | Fail-closed field transition and readiness transaction |
| systemd runner | `systemd.py` | Internal ProductControl process executor; not a second public control plane |
| `lingtu.plugin_seed` | `plugin_seed.py` | Product plugin catalog; imports built-in module groups so their `@register` decorators populate `runtime.registry` |
| `lingtu.sdk` | `sdk/` | Remote client SDK for an already running robot or gateway |

Standalone camera, LiDAR, SLAM, navigator, and detector wrappers were removed.
Those capabilities live in domain Module packages and are selected by
`lingtu.assembly`; the generic Blueprint mechanism remains in
`runtime.blueprint`.

Blueprint is active only inside the Python Host. It assembles Gateway,
semantic/agent features, adapters, and other Host-local Modules. It does not
start LiDAR, SLAM, navigation, traversability, or driver services;
ProductControl owns those process boundaries.

## Plugin Seed Boundary

`lingtu.plugin_seed` answers "which built-in LingTu modules are part of this
package?" It does not select the active Product or local Profile and does not build a
system. Runtime code calls `seed_builtin_plugins(groups=(...))` when it needs a
category in the registry.

The product global-planner category seeds `octoplanner3d` and the explicit
native `far` option. OctoPlanner3D remains the default; FAR selection must come
from the resolved Product/RunPlan contract. Legacy PCT/A*/direct planning code
stays outside the product backend catalog.

## Testing

```bash
python -m pytest src/lingtu/tests/
```
