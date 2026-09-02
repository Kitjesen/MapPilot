# Simulation Runtime Contract

Status: current
Audience: simulation, catalog, runtime, native-DDS bridge, UE, and acceptance maintainers
Replaced by: not replaced

This contract fixes the vocabulary and ownership layers of the simulation
platform itself: packages, runtimes, the runtime coordinator, adapters,
engines, and processes. The product-facing boundary of `env=sim` (backend
selection, evidence rules, forbidden claims) stays in
[`SIMULATION_INTEGRATION_CONTRACT.md`](./SIMULATION_INTEGRATION_CONTRACT.md);
this document defines what happens *inside* a simulation backend.

## Purpose

Simulation is being productized as a generic runtime platform layered over
MuJoCo (physics) and Unreal Engine 5 (rendering). Every layer must have exactly
one name, one owner, and a fixed boundary. The former `sim/` mixture of engine
code, acceptance tests, diagnostics, and experiments is the consequence of
layers existing without names; this contract is the correction.

## Fixed Runtime Vocabulary

Use these terms with exactly one meaning:

| Term | Owns | Does not own |
| --- | --- | --- |
| `Package` (`RobotPackage` / `ControllerPackage` / `SensorPackage` / `SensorRigPackage` / `WorldPackage` / `ScenarioPackage`) | Versioned static definitions and assets. A `SessionSpec` references packages but is not itself a package. | Running state, processes, DDS topology, run allocation, or any runtime side effect. |
| Catalog + Session Compiler | Resolving one `session.yaml` into backend plans with one shared `session_id`, direct package identities, paths, schemas, and structures. This is the first boundary of the generic simulation Runtime. | Launching MuJoCo, Unreal, DDS, or a controller process. Runtime allocation (DDS domain, ports, shared-memory names, PIDs, logs) stays in `RunAllocation`. |
| Runtime | Loading resources, creating model/instance/sensor/controller objects, advancing computation, maintaining state, receiving commands, emitting output. Concretely: `MujocoRuntime` advances physics and produces `SimulationSnapshot` (sequence, physics_step, sim_time_ns, model_generation, reset_generation). | Protocol conversion, session orchestration, rendering. A runtime is a responsibility, not necessarily a process. |
| Runtime Coordinator (session runtime) | Orchestrating one simulation session: reading the catalog plans, allocating resources (DDS domain, ports, shared-memory names, PIDs, logs), starting and stopping processes, driving the session state machine (READY / RUNNING / PAUSED / STOPPED), propagating reset, and emitting a session runtime manifest. | Package resolution, navigation algorithms, `navd`/`mapd`/traversability state. |
| Adapter | Protocol conversion across boundaries: `SimulationSnapshot` to typed DDS `JointState`, LingTu commands to runtime `ControlCommand`, camera frames to SHM. | Advancing physics, creating robots, rendering scenes. |
| Engine | Base capability: MuJoCo (physics: dynamics, contacts, sensors, ray casting), Unreal Engine (rendering, world, camera output). | How the platform productizes that capability. |
| Process / Server | The program that hosts a runtime: `mujoco_headless`, `native_dds_sensors`, `RobotSimUE`, WSL-wrapped native binaries. | The runtime logic itself; the same runtime code may be embedded in different processes. |

Translation reference (Chinese, informal): Runtime = 运行时执行层;
Runtime Coordinator = 会话运行协调器; Package = 静态定义与资产;
Adapter = 协议转换层; Engine = 基础能力引擎; Process = 承载进程.

## Layer Structure

```text
Robot / Controller / Sensor / SensorRig / World / Scenario packages
                         + SessionSpec
static definitions
        ↓
Catalog Resolver + Session Compiler (deterministic, side-effect free)
        ↓
ResolvedSessionBundle
session.yaml + physics/visual/sensor/control/scenario plans
             + transport.intent.json
one session_id across every plan
        ↓
Runtime Coordinator (session orchestration, resource allocation)
        ↓
┌─────────────────────────────────────────────┐
│ Physics Runtime    — MujocoRuntime + PhysicsSceneComposer   │
│ Robot Runtime      — robot instance create/bind/update/destroy │
│ Sensor Runtime     — sampling schedule, pose, noise, timestamp, sequence │
│ Controller Runtime — command inbox, policy/kinematic drive modes │
│ Visual Runtime     — RobotSimUE consumes visual.plan.json only │
└─────────────────────────────────────────────┘
        ↓
Adapter (protocol conversion)
        ↓
LingTu typed DDS (`/slam/*`, `/nav/cmd_vel`, ...) -> LingTu product runtime
```

The RobotSimUE host validates the resolved bundle identity, then each UE module
consumes only its compiled view: Visual Runtime consumes `visual.plan.json`,
Sensor Runtime consumes `sensor.plan.json`, and Session Runtime consumes run
facts plus immutable snapshots. Unreal never re-reads `SessionSpec`, package
manifests, `RobotConfig`, or MJCF.

## Ownership Rules

1. **Runtime is a responsibility, not a process.** `MujocoRuntime` may be
   embedded in `mujoco_headless`, in a DDS bridge process, or later inside
   RobotSimUE. Reuse the runtime core; do not duplicate it per host.
2. **Runtime is not an Adapter.** A runtime advances and maintains the
   simulation; an adapter converts protocols. `sim/adapters/dds/` bridge code is
   an adapter even when it is collocated with runtime code.
3. **Runtime is not an Engine.** MuJoCo is the physics engine; `MujocoRuntime`
   is the productized execution layer over it. The same separation applies to
   Unreal Engine vs. the visual runtime.
4. **Catalog is deterministic and side-effect free.** Allocations, PIDs, and
   logs are coordinator facts, never catalog facts; they stay in
   `RunAllocation`.
5. **Coordinator does not own navigation domain logic.** It orchestrates
   processes and allocates resources; it must not import or reimplement
   `navd`/`mapd`/traversability code. Navigation stays reachable only through
   typed DDS contracts.
6. **Packages never start processes.** Only runtimes and the coordinator do.

## Current Implementation Status

| Layer | Location | Status |
| --- | --- | --- |
| Package + Catalog | `sim/packages/`, `sim/sessions/`, `sim/catalog/` | Shipped; resolves `session.yaml` into physics, visual, sensor, control, and transport plans with one `session_id`; ScenarioPlan remains optional until a scenario package is selected. |
| Physics Runtime | `sim/runtime/physics/` (`MujocoRuntime`, `PhysicsSceneComposer`) | Shipped; a typed `PhysicsScenePlan` composes one session `mjModel`/`mjData`, and `SimulationSnapshot` carries session identity, stable body IDs, `model_generation`, and `reset_generation`. |
| Robot / Sensor / Controller Runtime | `sim/runtime/control/`, `sim/runtime/sensors/`, compiled contracts, and explicitly isolated compatibility implementations under `sim/compat/engine/mujoco/` / `sim/scripts/mujoco/` | The Controller scheduler/safe-stop core, ThunderV4 TorchScript + PD adapter, MuJoCo torque sink, generation-stamped Sensor scheduler, and per-stream readiness are implemented. The fixed Windows slice assembles RGB/depth camera SHM at 30 Hz, physics IMU at 200 Hz, Mid360 at 10 Hz, and truth odometry at 100 Hz. Component and UE Automation evidence is current; the fresh same-run playable qualification remains pending. |
| Visual Runtime | `sim/catalog/visual_binding.py` + `sim/runtime/visual/RobotSimUE/` | The current Win64 Editor target, generated ThunderV4 assets, native bundle/snapshot loader, capacity-one Session mailbox, generation gate, all 21 live body transforms, deferred RGB/depth render capture, camera SHM publication, and runtime HUD/input modules compile and pass the 2026-08-12 full `LingTuSim` Automation suite. The first truth transport remains localhost UDP JSON; a strict live playable PASS and Shipping distribution are not yet proven. |
| Adapter | `sim/adapters/dds/`, `sim/adapters/shm/` | Truth odometry typed DDS and the canonical Camera SHM ABI are implemented and tested on Windows. A transport contract is not evidence that its source sensor is active. |
| Runtime Coordinator | `sim/runtime/coordinator/` | Validates plan files by schema, `session_id`, package identity, referenced paths, and structure; creates a separate `RunAllocation`; owns the MuJoCo/RobotSimUE/native-publisher process trees; drives Controller and Sensor lifecycles; aggregates per-stream evidence into facet readiness; emits `session.runtime.json`; and publishes paced latest-only truth snapshots to RobotSimUE. The manual diagnostic and strict runner contracts are implemented; the pending fresh live gates, final evidence bundle, and strict verdict remain the product blocker. |
| Python engine path | `sim/compat/engine/mujoco/engine.py` (`MujocoDriverModule`) | Existing development path used by the direct engine CLI and simulation scripts; not the generic Runtime or a Coordinator substitute. |

## Migration Rules

- Name every new unit by its contract layer before writing it; a name that
  spans layers is a design defect, not a wording choice.
- Move code between layers with `git mv` and a passing narrow test per move;
  do not rewrite behavior during a move.
- Coordinator code must not `import` from `src/nav`, `src/maps`, or
  `lingtu.control`; process boundaries and typed DDS are the only bridges.
- Simulation evidence rules remain those of
  [`SIMULATION_INTEGRATION_CONTRACT.md`](./SIMULATION_INTEGRATION_CONTRACT.md):
  this contract changes internal structure, not claim boundaries.
