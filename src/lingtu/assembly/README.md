# LingTu Assembly

`lingtu.assembly` resolves one Product in `real` or `sim` into an immutable
RunPlan and constructs its Python Host Module graph. `ProductControl` owns
starting, switching, readiness, rollback, and stopping the complete Product.
Declaring or compiling the graph does not start Modules or native processes.

## Layout and responsibilities

```text
assembly/
  compile_plan.py                 compile the RunPlan; rebuild its Host graph
  build_scripts.py                select native build scripts by process role
  config/
    product.py                    resolve Product, Env, and RobotConfig inputs
    endpoint_bindings.py          choose declared endpoint/transport bindings
    native_navigation.py          compile native navigation configuration
    native_parameters.py          native parameter schema, bounds, and defaults
    simulation.py                 compile simulation session/acceptance data
    validation.py                 validate the resolved Product assembly
  graph/                          load declarations, resolve variants/processes
  host/
    build.py                      assemble selected Host Modules
    native_state_bridge.py        runtime native-state adapter (HostBus)
    plugin_catalog.py             LingTu's concrete built-in plugin catalog
    stacks/                       add groups of Modules and their settings
      composition.py              select and combine the required groups
      semantic_tasks.py           semantic and Agent task Modules
      localization.py             localization/SLAM-status Modules
      native_commands.py          command services and native-state bridge
      startup_checks.py           selected Host startup preflight
      settings.py                 resolved configuration for Module groups
    wiring/
      connect.py                  apply cross-stack connections once
      context.py                  inspect selected Modules for wiring
      localization.py             odometry, localization, and map connections
      navigation.py               navigation connections
      semantic.py                 semantic task connections
      gateway.py                  Gateway status and media connections
```

The other files in `host/stacks/` group driver, LiDAR, perception, navigation,
exploration, memory, measurements, and Gateway Modules. Stacks select and add
Modules; wiring connects their typed ports. Neither owns native process
lifecycle. `runtime.host.HostBuilder` provides the generic in-process assembly
mechanism; `host/build.py` supplies LingTu's concrete selection and configuration.

`host/plugin_catalog.py` declares the built-in importable Module catalog.
`runtime.plugin_seed` owns the generic registration/seed mechanism, and factories
resolve implementations through the registry. The catalog is separate from
Product resolution and does not select which capabilities a Product enables.

`host/native_state_bridge.py` is runtime adapter code: after Host starts,
`HostBus` reads native state, deduplicates events, converts native payloads, and
publishes typed Host messages. Its placement beside Host assembly keeps this
concrete adapter out of generic `runtime/`; it does not make native I/O part of
RunPlan compilation.

## Host is part of the current runtime

Host remains the managed Python application for Gateway, semantic/Agent tasks,
adapters, and the Modules selected by the Product. Native C++ services own field
LiDAR, SLAM, map state, terrain/traversability, navigation, and driver hot paths.
Native navigation owns final motion arbitration; only `lingtu-driver` forwards
the checked command to Brainstem. Host does not replace that chain.

```text
Product + Env + selected RobotConfig/session inputs
  -> ProductControl calls compile_plan.py once
       -> config/                 resolve and validate configuration
       -> graph/                  resolve concrete native processes
       -> host/build.py           declare selected Modules and connections
  -> immutable RunPlan
  -> ProductControl starts the declared processes
       -> real: systemd -> lingtu.real.host
       -> sim: direct child -> src/lingtu/sim/host.py
  -> Host entry loads the published RunPlan
  -> RunPlan.build() -> HostBuilder.build() -> system.start()
```

The real Host entry is [`lingtu/real/host.py`](../real/host.py), managed as
`lt-host.service`. The simulation Host entry is
[`lingtu/sim/host.py`](../sim/host.py), declared by the selected sim backend.
Both consume `LINGTU_RUN_PLAN`, start the Module graph, and stop it on exit.
The simulation entry also publishes Host readiness. Neither entry resolves the
Product again or takes ownership of the native services. Public lifecycle
commands remain `python -m lingtu.control` or the thin `scripts/lingtu` adapter.

## Configuration and defaults

Product behavior and capabilities come from `config/system/products/*.yaml`;
`config/system/envs/*.yaml` selects concrete implementations. `config/product.py`
resolves these with the selected RobotConfig for `real` and the supported
simulation inputs for `sim`.

Native tuning parameters in `config/native_parameters.py` are applied in this
order, with later values overriding earlier ones:

```text
schema defaults -> Env parameters -> Product parameters -> session overrides
```

The same schema is used by Product compilation and RunPlan validation/export.
Bounds and cross-parameter constraints remain active after resolution. Ordinary
speed defaults use `DEFAULT_LINEAR_SPEED_MPS = 1.0`, shared with
`config/native_navigation.py`. Native C++ structs use
`nav_kernel::kDefaultLinearSpeedMps = 1.0` across planning, tracking, execution,
and command limits. Product profiles still declare their own motion contract:
the current 1 m/s-per-axis profiles use a sqrt(2) norm budget. An explicit lower
limit remains binding; these defaults do not replace calibration, stair/recovery
profiles, or final motion arbitration.

Host configuration has a separate merge: common defaults, Product Host settings,
real-only defaults for missing fields, Env implementation `host_config`, physical
RobotConfig fields for `real`, and permitted caller overrides. Product SLAM mode
and reserved robot/environment fields retain their declared owners; callers
cannot override them through arbitrary Host settings. `DEFAULT_GATEWAY_PORT`
lives in `config/product.py`; the planning frame uses `runtime.tf.frames.map_frame_id()`.
There is no separate `products/runtime_paths.py` defaults layer.

## Important path changes

These internal modules were moved directly; imports and tooling use the new
paths without compatibility packages.

| Previous path | Current path |
| --- | --- |
| `compiler.py` | `compile_plan.py` |
| `deployment.py` | `build_scripts.py` |
| `products/configuration.py` | `config/product.py` |
| `products/host.py` | `host/build.py` |
| `native_nav.py`, `parameters.py` | `config/native_navigation.py`, `config/native_parameters.py` |
| `host_bus.py`, `plugins.py` | `host/native_state_bridge.py`, `host/plugin_catalog.py` |
| `stacks/planner.py`, `stacks/slam.py` | `host/stacks/semantic_tasks.py`, `host/stacks/localization.py` |
| `stacks/services.py`, `stacks/system.py`, `stacks/stack_config.py` | `host/stacks/native_commands.py`, `host/stacks/startup_checks.py`, `host/stacks/settings.py` |
| `wires/full_stack.py`, `wires/slam.py` | `host/wiring/connect.py`, `host/wiring/localization.py` |

The remaining stack and wiring files moved under `host/stacks/` and
`host/wiring/`. `graph/` retains its existing ownership. The `products/` package
and its re-exports were removed; import configuration from `config.product` and
Host construction from `host.build`.

The unused `exploration_backend_for_config()` and its private `_adapter_name()`
were removed with their isolated helper test. Active exploration settings still
come from `host/stacks/settings.py`; exploration support remains available.
The former `products/runtime_paths.py` constants were merged into configuration,
and its forwarding package was removed. Document rendering in `graph/render.py`
remains a supported diagnostic helper.

## Working rules

- Resolve Product and environment inputs once in `compile_plan.py`.
- Inspect the assembled Host and resolved configuration instead of maintaining
  a second static Module graph.
- Keep algorithms, HTTP handling, and process supervision in their owning
  packages.
- Do not open hardware or network connections while declaring Host Modules.
