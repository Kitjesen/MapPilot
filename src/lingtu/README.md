# LingTu Local Facade

`src/lingtu` is intentionally small. It exposes one high-level facade,
`Robot`, the direct runtime builder, the remote SDK, and the product plugin
catalog used during startup.

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
| `lingtu.plugin_seed` | `plugin_seed.py` | Product plugin catalog; imports built-in module groups so their `@register` decorators populate `runtime.registry` |
| `lingtu.sdk` | `sdk/` | Remote client SDK for an already running robot or gateway |
| `lingtu.ros2_plugin_seed` | `ros2_plugin_seed.py` | Optional ROS2 compatibility plugin groups; not a normal business-module entry |

Standalone camera, LiDAR, SLAM, navigator, and detector wrappers were removed.
Those capabilities live in the normal Blueprint stacks and module packages.

## Plugin Seed Boundary

`lingtu.plugin_seed` answers "which built-in LingTu modules are part of this
product package?" It does not select the active profile and does not build a
system. Runtime code calls `seed_builtin_plugins(groups=(...))` when it needs a
category in the registry.

The product global-planner category currently seeds only `octoplanner3d`.
Legacy PCT/A*/direct planning code is kept outside the product backend catalog
unless a dedicated compatibility entry explicitly opts into it.

## Testing

```bash
python -m pytest src/lingtu/tests/
```
