# Simulation package contracts

These schemas define the build-time boundary for the generic simulation
runtime. YAML is the authoring format; the resolver converts it to the
canonical JSON-compatible values consumed by runtime plans.

The resolver in `sim/catalog/` is deliberately stricter than the broad JSON
Schema vocabulary: unknown keys are rejected, package references are exact
`id@version` references, and every declared file is checked against the local
repository root and hashed into `session.lock.json`.

The contracts are split by ownership:

- `robot-package.v1`: mechanism, frames, interfaces, and default references.
- `controller-package.v1`: policy artifact, adapter ABI, timing, and actuator order.
- `sensor-package.v1`: one sensor model and its output contract.
- `sensor-rig-package.v1`: sensor instances and their parent frames.
- `world-package.v1`: world physics/visual projection and entity facets.
- `session.v1`: the requested world, robot instances, seed, and runtime mode.

`session.lock.json` and `physics.plan.json` are generated artifacts. Ports,
DDS domains, shared-memory names, PIDs, and log paths belong to a later
`RunAllocation` and must not affect `session_digest`.
