# Development

**Status:** Current contributor guide
**Audience:** Contributors, maintainers, and integration engineers
**Runs on:** Local development, native builds, simulation, and field-targeted builds

Change the layer that owns the behavior. Use [Architecture](architecture.md)
for system boundaries and [Testing](testing.md) for the evidence required.

## Product-focused simplification

Prioritize the [shared delivery workflow](product.md#完整交付流程).
Reduce duplicate ownership and maintenance work; file count is not a product
acceptance criterion.

- Retain the runtime, build, installation, operator, diagnostic, and test
  support needed for delivery. Preserve existing robots and supported Products.
- Give each responsibility one implementation and configuration owner. Reuse
  existing factories, registries, and interfaces before adding abstractions.
- Trace Product/Env declarations, registries, schemas, build/install paths,
  public entrypoints, and imports before declaring something unused.
- Remove obsolete duplicates and disposable output. Preserve uncommitted work,
  calibration, maps, models, field recordings, and unique investigation data.
- Change one responsibility at a time, updating consumers and documentation
  together. Tidying does not justify changing algorithms, public contracts,
  dependency versions, or support policy.
- State the reason and focused verification for each change. Run clean-build,
  package, and end-to-end gates at the corresponding delivery milestone.

## Find the owner first

| Change | Owner |
| --- | --- |
| Product roles, topics, capabilities | `config/system/products/` |
| Real/sim implementation mapping | `config/system/envs/` |
| Product resolution and lifecycle | `src/lingtu/assembly/`, `src/lingtu/control.py` |
| Host Modules, internal assembly, messages, registry | `src/runtime/` |
| Planning and motion safety | `src/nav/cpp/` |
| Maps, localization, drivers | Their matching `src/` package |
| Perception, decision, memory, Gateway | Their matching `src/` package |
| Simulation packages and sessions | `sim/` |
| Build/deploy commands and developer tools | `scripts/` and [tools](../tools/README.md) |

Detailed placement is in [the source guide](../src/README.md).

## Ordinary change loop

1. Define the user-visible behavior and current owner.
2. Implement one slice through the existing Product, HostBuilder, or adapter.
3. Run the narrowest test that could disprove the behavior, then affected
   lint/type/build checks. Before each check, name the failure and response.
4. Update the owning documentation and review the exact diff.

Preserve unrelated changes. Report baseline failures separately from new
ones. Use small, reversible commits and do not add dependencies without approval.

## Modules and HostBuilders

A Module is a typed in-process Host unit. Keep construction cheap; acquire
resources in its lifecycle and clean up only what it owns. Use typed `In`/`Out`
ports and explicit command, map, localization, and external-boundary wires.
Use `auto_wire()` only for unique, unambiguous local matches.

HostBuilder constructs the Host graph; it does not resolve Products, manage
systemd, or start native processes. A new class becomes Product behavior only
when declarations, assembly, wiring, and focused graph tests agree.

## Backends and adapters

Use the runtime registry and owning factory for selectable implementations.
Record the boundary's owner and failure behavior: devices need calibration and
reconnect rules; DDS needs schema, frame, timestamp, QoS, and stale handling;
shared memory needs payload lifetime and readiness; replay needs timing/reset
and source identity. ROS compatibility must remain explicitly selected.

Import direction is enforced by
[`config/architecture_layers.yaml`](../config/architecture_layers.yaml).

## Native code

Python owns Host and semantic/API behavior; C++ owns field sensor, SLAM, map,
terrain, navigation, and driver hot paths. Portable Rust kernels need an
explicit owner and build path. Native install rules publish `bin`, `lib`,
`etc/lingtu`, and `share/lingtu`; deployed code must not require build directories.
Record provenance, license, target, and use when adopting third-party code.

## Configuration and parameters

Product behavior belongs in Product YAML, environment bindings in Env YAML,
and physical devices/calibration in RobotConfig and `config/devices.yaml`.
Keep algorithm defaults with their owner and expose only bounded tuning knobs.
Do not embed credentials, robot addresses, map choices, or env selection in a
Product, or weaken field thresholds without representative evidence.

## Tests and style

See [Testing](testing.md#local-checks) for checks by task. Run Ruff only on
changed Python files, `bash -n` on changed shell scripts, and parser-only
checks on changed PowerShell scripts. New Python comments are English.
Keep detailed implementation notes beside their owner and run evidence in
its designated artifact store.
