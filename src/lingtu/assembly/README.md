# LingTu Assembly

`lingtu.assembly` turns one resolved Product and `real`/`sim` environment into
the Host Blueprint and native process data stored in a RunPlan. It declares
what should run; `ProductControl` performs the lifecycle operation.

```text
Product + env
  -> compiler.py
       -> products/configuration.py   resolve Host inputs once
       -> products/host.py            build one Host Blueprint
            -> stacks/                add Modules
            -> wires/full_stack.py    connect Modules
       -> Runtime Graph               resolve native processes
       -> native_nav.py               compile native-nav settings
       -> simulation.py               compile sim-only data
  -> RunPlan
  -> ProductControl
```

## Files

| Path | Purpose |
| --- | --- |
| `compiler.py` | The Product-to-RunPlan entry and Blueprint rebuild path. |
| `products/configuration.py` | Resolve Product, environment, and robot inputs. |
| `products/host.py` | Build the Product Host Blueprint. |
| `stacks/` | Add small groups of Modules. |
| `wires/full_stack.py` | Apply all cross-stack connections once. |
| `native_nav.py` | Convert resolved navigation settings to native process config. |
| `simulation.py` | Compile simulation session and acceptance data. |
| `validation.py` | Check the already-resolved Product assembly. |
| `host_bus.py` | Adapt the native command ABI to typed Host messages. |
| `parameters.py` | Resolve bounded runtime parameter profiles. |

## Rules

- Resolve Product and environment inputs once in `compiler.py`.
- Stacks add Modules; wires connect them; neither owns process lifecycle.
- Inspect the real Blueprint/RunPlan instead of maintaining a second static graph.
- Keep mapping, planning, SLAM, filtering, control, HTTP handling, and native
  process supervision in their owning packages.
- Do not open hardware or network connections while declaring a Blueprint.
