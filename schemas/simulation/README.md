# Simulation JSON contracts

This directory is the source of truth for the stable simulation package, plan,
and runtime JSON contract families indexed below. It is not an inventory of
every code-local diagnostic, evidence, or UI envelope that happens to carry a
`schema` field. YAML remains the catalog authoring format; `sim/catalog/`
resolves it into the canonical JSON-compatible packages and plans described
here.

These are not the repository's universal messages. DDS wire types live in
`src/message/idl/`, Python in-process messages in `src/runtime/msgs/`, HTTP
models in `src/gateway/schemas.py`, and topic ownership/QoS in
`config/runtime_graph/topics.yaml`.

## Contract index

| Schema ID | File | Producer | Consumer | Lifecycle / version |
| --- | --- | --- | --- | --- |
| `lingtu.sim.controller-package.v1` | `controller.v1.json` | Catalog authors/tools | Catalog resolver, control-plan compiler | Immutable package v1 |
| `lingtu.sim.entity-visual-projection.v1` | `entity-visual.v1.json` | Visual projection builder | Resolver, RobotSimUE | Immutable projection v1 |
| `https://lingtu.inovxio.local/schemas/simulation/factory-park.v1.json` | `factory-park.v1.json` | Factory Park authoring/export | Factory Park import/runtime tools | Immutable facility document v1 |
| `lingtu.sim.payload-visual-projection.v1` | `payload-visual.v1.json` | Payload visual builder | Resolver, RobotSimUE | Immutable projection v1 |
| `lingtu.sim.payload-package.v1` | `payload.v1.json` | Catalog authors/tools | Catalog resolver | Immutable package v1 |
| `lingtu.sim.qualification-record.v1` | `qualification.v1.json` | Qualification tools | Catalog admission/inspection | Immutable evidence v1 |
| `lingtu.sim.robot-import-request.v1` | `robot-import.v1.json` | Import CLI/UI | Robot importer | One-shot request v1 |
| `lingtu.sim.robot-visual-projection.v1` | `robot-visual.v1.json` | Robot visual builder | Resolver, RobotSimUE | Immutable projection v1 |
| `lingtu.sim.robot-package.v1` | `robot.v1.json` | Catalog authors/importer | Catalog resolver | Immutable package v1 |
| `lingtu.sim.scenario-package.v1` | `scenario.v1.json` | Catalog authors/tools | Catalog resolver | Immutable package v1 |
| `lingtu.sim.sensor-rig-package.v1` | `sensor-rig.v1.json` | Catalog authors/tools | Catalog resolver | Immutable package v1 |
| `lingtu.sim.sensor-package.v1` | `sensor.v1.json` | Catalog authors/tools | Catalog resolver | Immutable package v1 |
| `lingtu.sim.ue-asset-library.v1` | `ue-assets.v1.json` | UE asset inventory builder | Visual builders, RobotSimUE tooling | Generated library v1 |
| `lingtu.sim.world-import-request.v1` | `world-import.v1.json` | Import CLI/UI | World importer | One-shot request v1 |
| `lingtu.sim.world-visual-projection.v1` | `world-visual.v1.json` | World visual builder | Resolver, RobotSimUE | Immutable projection v1 |
| `lingtu.sim.world-package.v1` | `world.v1.json` | Catalog authors/importer | Catalog resolver | Immutable package v1 |
| `lingtu.sim.session-intent.v1` | `session-intent.v1.json` | SimStudio/authoring clients | SessionComposer | Authoring request v1 |
| `lingtu.sim.session-intent.v2` | `session-intent.v2.json` | SimStudio/authoring clients | SessionComposer | Authoring request v2; payload-aware |
| `lingtu.sim.session.v1` | `session.v1.json` | SessionComposer/catalog authors | Catalog resolver | Resolved request v1 |
| `lingtu.sim.session.v2` | `session.v2.json` | SessionComposer/catalog authors | Catalog resolver | Resolved request v2; payload-aware |
| `lingtu.sim.control-plan.v1` | `control-plan.v1.json` | Catalog resolver | Control runtime, coordinator | Frozen bundle plan v1 |
| `lingtu.sim.physics-plan.v1` | `physics-plan.v1.json` | Catalog resolver | C++ PhysicsSceneComposer | Frozen bundle plan v1 |
| `lingtu.sim.physics-plan.v2` | `physics-plan.v2.json` | Catalog resolver | C++ PhysicsSceneComposer | Frozen bundle plan v2; payload-aware |
| `lingtu.sim.scenario-plan.v1` | `scenario-plan.v1.json` | Catalog resolver | Scenario runtime, coordinator | Frozen bundle plan v1 |
| `lingtu.sim.sensor-plan.v1` | `sensor-plan.v1.json` | Catalog resolver | Sensor runtime, coordinator | Frozen bundle plan v1 |
| `lingtu.sim.transport-intent.v1` | `transport-intent.v1.json` | Catalog resolver | Run allocator/coordinator | Frozen resource intent v1 |
| `lingtu.sim.visual-plan.v1` | `visual-plan.v1.json` | Catalog resolver | RobotSimUE | Frozen bundle plan v1 |
| `lingtu.sim.visual-plan.v2` | `visual-plan.v2.json` | Catalog resolver | RobotSimUE | Frozen bundle plan v2; payload-aware |
| `lingtu.sim.run-allocation.v1` | `run-allocation.v1.json` | Runtime coordinator allocator | Runtime processes, gates, qualification | Immutable per-run allocation v1 |
| `lingtu.sim.session-runtime.v1` | `session-runtime.v1.json` | RuntimeCoordinator, visual replay runtime | Gates, qualification, replay, SimStudio | Mutable per-run manifest v1; explicit live/replay variants; atomically replaced |
| `lingtu.sim.truth-snapshot.v1` | `truth-snapshot.v1.json` | Coordinator projection of MuJoCo state | RobotSimUE, recording, replay | Session-bound runtime snapshot v1; sequence/generation stamped |
| `lingtu.sim.mujoco-snapshot.v1` | `mujoco-snapshot.v1.json` | `lingtu_mujoco_snapshot` | Offline inspection/preview tools | Standalone inspection snapshot v1; no session identity |

## Ownership rules

- Package and projection documents are immutable catalog inputs. Unknown keys
  are rejected, references use `id@version`, and declared files stay inside the
  repository/package boundary.
- Resolved plans are generated bundle artifacts. Runtime code consumes them; it
  must not reparse package YAML or independently resolve product intent.
- `run-allocation.v1` fixes per-run resources. `session-runtime.v1` is separate
  mutable state and is atomically replaced. Its live variant carries complete
  generation-stamped bindings; its visual-replay variant carries recorded-clock
  authority and replay provenance without pretending that a physics process ran.
- `truth-snapshot.v1` is the session-bound coordinator-to-visual/recording
  contract. `mujoco-snapshot.v1` is intentionally separate because the
  standalone CLI has neither `session_id` nor `physics_step` and uses raw
  MuJoCo body identity.
- v1 remains an active compatibility boundary while v2 adds payload-aware
  session, physics, and visual plans. Do not remove v1 until every declared
  consumer has migrated.

Build or refresh robot visual projections with
`sim/tools/assets/build_robot_visual_projection.py`. RobotSimUE consumes the
resolved visual plan plus referenced projection documents; it does not own
catalog resolution.
