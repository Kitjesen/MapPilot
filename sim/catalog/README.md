# Simulation catalog

The catalog resolves one simulation preset into deterministic runtime plans.

## Contents

| Path | Responsibility |
| --- | --- |
| `resolver.py` | Discover package manifests from the single `sim/packages/` root and compile one SessionSpec. |
| `composer.py` | Turn a validated SessionIntent into `session.yaml` and the same compiled plans. |
| `importers/` | Inspect external Robot/World sources, qualify drafts, and promote packages. |
| `management.py` | Expose read-only package list, inspect, dependency, and qualification queries. |
| `visual_binding.py`, `visual_projection.py` | Validate compiled visual bindings without launching a runtime. |

## Boundary

All package manifests and their owned assets live under `sim/packages/`.
The resolver does not launch MuJoCo, Unreal, DDS, or a controller process.

## Entry points

Resolve the default Thunder V4 preset with:

```powershell
python -m sim.catalog resolve sim/sessions/products/doso/thunder_v4/default.yaml `
  --repo-root . `
  --output-dir catalog-contract
```

SimCatalog management and authoring commands use the same resolver instance and emit
machine-readable JSON envelopes:

```powershell
python -m sim.catalog list --repo-root . --kind robot
python -m sim.catalog inspect --repo-root . thunderv4@1.0.3 --kind robot
python -m sim.catalog dependencies --repo-root . thunderv4@1.0.3 --kind robot
python -m sim.catalog qualification --repo-root . thunderv4@1.0.3 --kind robot
python -m sim.catalog compose --repo-root . session.intent.yaml --output-dir authored-session
```

`SimCatalog` is read-only: list, inspect, validation, dependency, and
qualification queries never allocate ports or start MuJoCo, Unreal, DDS, or a
controller process. External qualification evidence lives at
`sim/evaluation/package_qualifications/<kind>/<id>/<version>.qualification.json` and identifies the
package by kind, ID, and version. Missing evidence is reported as `unverified`;
invalid evidence fails closed with a stable diagnostic code.

`SessionComposer` accepts `lingtu.sim.session-intent.v1`, emits one
`lingtu.sim.session.v1` file, and calls the existing `CatalogResolver.resolve`
boundary. It does not parse packages independently and does not re-read
compiled plans. RunAllocation remains a later runtime concern and is absent
from SessionIntent and `session.yaml`.

Generated bundles are published atomically beneath the service-owned
`build/simstudio/` root by default. Relative `--output-dir` values name a child
inside that root; absolute paths are accepted only when they remain inside the
configured `--artifact-root`. Existing bundle directories are immutable and
fail with `SIMCATALOG_ARTIFACT_CONFLICT` instead of being overwritten.

The authored input is `session.yaml`. A composed bundle keeps it beside the
generated plans; direct resolution writes the same plans from an existing
SessionSpec. Every plan carries the same `session_id`.

- `physics.plan.json`: one session-level MuJoCo composition plan with one
  namespace per robot instance.
- `visual.plan.json`: Unreal-facing world and robot binding intents, direct
  package identity, projection path/schema/structure, spawn transforms, and the
  explicit MuJoCo-to-Unreal coordinate conversion. It deliberately contains no
  fixed robot body count.
- `sensor.plan.json`: static sensor stream declarations generated from
  SensorPackage and SensorRigPackage when `runtime.required_bindings` includes
  `sensors`.
- `control.plan.json`: static controller package, adapter,
  timing, state channel, actuator channel, and command-channel declarations
  when `runtime.required_bindings` includes `control`.
- `transport.intent.json`: channel intent derived from sensor and control
  plans. It names external and in-process contracts, but never assigns DDS
  domains, ports, shared-memory names, command payloads, or runtime clocks.

The `physics.plan.json` file is the input to the C++
`PhysicsSceneComposer`; package-owned initial keyframes are resolved here and
combined with per-instance spawn transforms by the Physics Runtime. Runtime
allocation (DDS domain, ports, shared-memory
names, PIDs, and logs) is intentionally not part of these plans.
The RobotSimUE host validates the complete bundle identity, while its Visual
Runtime consumes `visual.plan.json` plus each referenced
`lingtu.sim.robot-visual-projection.v1` document and its Sensor Runtime consumes
`sensor.plan.json`; Unreal does not re-read SessionSpec, RobotPackage,
RobotConfig, or MJCF. Robot visual projections are instance-agnostic; RobotSimUE
applies `visual.plan.json` robot instance IDs and namespaces at runtime. In
headless physics sessions, the backend roles are still explicit:
`{"physics": "mujoco", "visual": null}`. MuJoCo owns physics; Unreal is an
optional visual/sensor role, not an alternate physics backend.

Product-selected presets live under `sim/sessions/products/<vendor>/<model>/`. A preset
only composes a world, robot, sensor rig, controller, spawn, and runtime
bindings; it does not own those assets. Package manifests are discovered from
the single `sim/packages/` root.
Use `sim/tools/assets/build_robot_visual_projection.py` to build or refresh the
robot projection files referenced by RobotPackage `visual.projection`.
