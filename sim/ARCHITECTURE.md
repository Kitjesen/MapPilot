# Simulation architecture

This file is the canonical architecture for the high-fidelity simulation
payload under `sim/`. LingTu Product contracts remain authoritative in
[`docs/architecture/README.md`](../docs/architecture/README.md) and the
simulation runtime contract remains in
[`docs/architecture/SIM_RUNTIME_CONTRACT.md`](../docs/architecture/SIM_RUNTIME_CONTRACT.md).

## Canonical Product Chain

```text
Authoring/Import & Asset Conditioning
        ->
Versioned Packages/Catalog
        ->
SessionCompiler
        ->
ResolvedSessionBundle
  session.yaml + physics/control/visual/sensor/scenario/transport plans
  one session_id across every plan
        ->
SessionRuntime
  Physics Module + Control Module + Visual Module + Sensor Module + Scenario Module
        ->
DDS / SHM / Pixel Streaming Adapters
        ->
Recorder / Replay / Episode / Qualification
        ->
Cook / Distribution / Operations
```

`lingtu.assembly` resolves one Product inside one fixed `env` into one
immutable RunPlan. ProductControl stages that RunPlan and owns readiness,
rollback, and current-plan commit. For the high-fidelity simulation payload,
that RunPlan carries the already-resolved Physics, Visual, Sensor, Control,
Transport, and optional Scenario plans at one staged bundle path. Each plan
carries the `session_id` from `session.yaml`. CLI, systemd, Host, and
SessionRuntime must not resolve the Product or compile the bundle again.

`RunAllocation` is ephemeral runtime allocation: PIDs, ports, DDS domain,
shared-memory names, and log paths. It is not a second RunPlan. Generation
identity belongs to SessionRuntime state and snapshots, not allocation ownership.

## Vocabulary

Use deep-module vocabulary precisely:

| Term | Meaning in simulation |
| --- | --- |
| Module | Anything with an Interface and implementation. The fixed LingTu Host `Module` meaning remains intact: one typed in-process Host runtime unit. Simulation Modules are runtime modules at different seams, not Host Modules unless they run inside Host. |
| Interface | Everything a caller must know: inputs, outputs, invariants, generation/reset rules, error modes, readiness, timing, and performance expectations. Keep Interfaces small. |
| Seam | The location where a Module Interface lives and behavior can vary without editing callers. |
| Adapter | A concrete implementation at a Seam that translates one Interface to another, such as DDS, camera SHM, Pixel Streaming, or a test stand-in. |

The compiler emits module-specific plans under one `session_id`. It must not emit a
god object that forces every runtime Module to know every other Module's
private configuration.

## Source Ownership

| Layer | Canonical source | Owns | Does not own |
| --- | --- | --- | --- |
| Authoring/Import & Asset Conditioning | `tools/`, `toolchains/`, DCC export recipes | Frames, units, scale, stable link IDs, collision/LOD/material/sensor surfaces, deterministic import settings, provenance, license, and tool versions. Blender/DCC is offline tooling only. | Runtime lifecycle, Product resolution, ports, PIDs, SHM names, DDS domains. |
| Robot Package | `robots/<vendor>/<model>/` | Robot identity, articulated bodies, stable links, actuator declarations, mesh/MJCF/URDF assets, package version. | Controller policy selection, runtime allocation, Product mode. |
| Controller Package | `controllers/<vendor>/<model>/` | Controller identity, model/policy assets, actuator binding requirements, supported robot/package constraints. | Physics stepping, navigation Product selection. |
| Sensor Package | `sensors/` | Sensor model, stream kind, timing contract, payload fields, transport intent. | Concrete runtime endpoint names or PIDs. |
| SensorRig Package | `sensor_rigs/<vendor>/<model>/` | Sensor instances, mounts, frames, calibration references. | Runtime process allocation or scenario selection. |
| World Package | `packages/worlds/` | World identity, static assets, physics/render references, provenance. | Dynamic scenario events. |
| Scenario Package | `packages/scenarios/` and `scenarios/` | Scenario declarations, dynamic actors, events, stop conditions, qualification criteria. | Package version identity for Robot/World/Controller/Sensor, ports, PIDs, SHM, processes. |
| Simulation preset | `presets/<vendor>/<model>/*.yaml` | Chooses package versions and scenario parameters for compilation. | A package. It contains no allocation values and owns no runtime side effects. |
| Catalog/Compiler | `catalog/` | Resolves `session.yaml` into module-specific plans with one shared `session_id`, direct package identities, paths, schemas, and structures. | Runtime readiness, Product switching, mutable state. |
| SessionRuntime | `runtime/` | Runs an already-resolved bundle through small Module Interfaces. | Product resolution or bundle compilation. |
| Adapters | `adapters/`, RobotSimUE plugins | Transport translation to DDS, SHM, Pixel Streaming, and test endpoints. | Physics authority or configuration source. |
| Recorder/Replay/Qualification | existing native recorder plus simulation hooks | Evidence capture, deterministic replay inputs, episode closure, qualification verdicts. | Simulation truth authority. |
| Cook/Distribution/Operations | `runtime/visual/RobotSimUE/`, `scripts/`, release tooling | Editor/build/Cook/package/stage/smoke artifacts and operating procedures. | Recompiling or changing the qualified session plans. |

Robot, controller, sensor, and sensor-rig manifests stay with the assets they
describe. Worlds, scenarios, and payloads remain versioned packages under
`sim/packages/`.

## Runtime Interfaces

### SessionRuntime External Interface

SessionRuntime accepts:

- `ResolvedSessionBundle` path; its plans must carry the expected `session_id`.
- `RunAllocation` path produced during staging.
- Lifecycle commands: prepare, start, pause, reset, stop.
- Generation identity: `model_generation`, `reset_generation`, monotonic
  sequence, and MuJoCo simulation time.

SessionRuntime publishes:

- Lifecycle/readiness state per required internal Module.
- Immutable truth snapshots from Physics to consumers.
- Per-stream sensor readiness and failure reasons.
- Adapter evidence paths for DDS, SHM, Pixel Streaming, and recorder surfaces.

Fail-closed rules:

- Mismatched `session_id`, missing bundle artifact, wrong plan schema or package
  identity, stale generation, non-monotonic sequence/time, stream identity drift,
  or required Module failure blocks readiness.
- One active stream cannot activate the whole Sensor facet.
- Reset changes `reset_generation`; stale commands/snapshots from older
  generations must be rejected or ignored.

### Internal Modules

| Module | Small Interface | Authority and constraints |
| --- | --- | --- |
| Physics | Plan in, command samples in, immutable truth snapshots and raycast results out. | MuJoCo is the sole simulation clock and physics authority. |
| Control | Controller plan, actuator bindings, command deadlines, and actuation outputs. | Schedules policy/PD control, binds stable actuators, and fails closed on stale/missing commands. |
| Visual | `visual.plan.json`, `RunAllocation`, immutable truth snapshots, readiness evidence. | RobotSimUE follows truth snapshots and owns presentation/render sensors only. It must not write UE Actor transforms back as truth. |
| Sensor | `sensor.plan.json`, clock ticks, backend samples, per-stream readiness. | Owns multi-rate scheduling and sample identity; MuJoCo owns inertial/truth/Mid360 raycast streams, UE owns render streams. |
| Scenario | `scenario.plan.json`, deterministic event clock, evaluator snapshot. | Computes deterministic scenario results. A dispatcher must explicitly apply accepted effects to Physics/Visual before they affect runtime. |
| Recorder | Bundle/allocation identity, clock, commands, snapshots, stream samples, verdicts. | Captures evidence and replay material; it is never a simulation authority. |

## Authority Matrix

| State or stream | Authority | Consumers |
| --- | --- | --- |
| Simulation clock, contacts, body/joint state | MuJoCo Physics Module | Control, Visual, Sensor, recorder |
| Robot/world presentation | RobotSimUE Visual Module | Render sensors, Pixel Streaming, operator view |
| RGB/depth/segmentation | RobotSimUE render-sensor backend | SHM/DDS adapters, recorder, qualification |
| IMU and truth odometry | MuJoCo sensor backend | Native typed DDS adapters, validation; truth odometry is not estimator input by default |
| Mid360 firing pattern, raycast hits, point metadata | Sensor Module + MuJoCo raycast backend | Native typed DDS adapter, SLAM, recorder |
| Motion command arbitration | LingTu typed command contract + Control Module | MuJoCo actuator sink |
| Dynamic actor intent and evaluator verdicts | Scenario Module | Dispatcher, qualification, recorder |
| Pixel Streaming | RobotSimUE presentation adapter | Human operator/client only |

No Adapter may advance physics, infer a pose, mutate a package, or become a
second configuration source.

## Adapter Constraints

- DDS Adapters translate typed LingTu process contracts. They do not resolve
  Products, compile sessions, or own readiness policy beyond their Interface.
- Camera SHM Adapters use the canonical two-slot, CRC-protected
  `lingtu.camera.shm_frame.v1` transport and names from `RunAllocation`.
- Pixel Streaming is presentation and teleoperation only. It is not an
  algorithm sensor contract and cannot qualify RGB/depth publication by itself.
- Native sensor Adapters must preserve declared fields. The Mid360 pipeline
  preserves `offset_time_ns`, `reflectivity`, `tag`, and `line`.
- Adapter evidence must name the `session_id`, run allocation, generation, and
  observed stream identities.

## Recorder, Replay, Episode, Qualification

Recorder integration must capture the resolved bundle identity, run allocation,
clock, commands, truth snapshots, DDS/SHM streams, scenario events, and
qualification verdicts. Replay must reconstruct timing without becoming a new
clock authority. Episode closure requires terminal state, stream completeness,
failure reasons, and session/generation consistency. Qualification gates must
state whether evidence is Editor, packaged, local native-DDS, Product same-run,
field, or distribution proof.

## Cook, Distribution, Operations

A distribution claim requires more than Editor success:

- Toolchain lock and build manifest.
- RobotSimUE Cook/package output.
- Staged session bundle and packaged binary smoke.
- Pixel Streaming smoke when operator presentation is in scope.
- Recorder/replay/episode closure for the same package.
- Operations notes for artifact storage, versioning, logs, and rollback.

Cooked artifacts must consume the same resolved session plans qualified by the
gate. Packaging must not recompile the session or mutate package manifests.

## Current Evidence And Status

Canonical Editor-session evidence path:
`build/live-runs-open-field-hf/thunderv4-openfield-hf-same-session-20260807-d`.

Inspected UE hero-image evidence:
`build/unreal-open-field-hf/final-hf8/OpenField_HF_1920x1080.png`
(SHA-256 `827fc3bf342ba49e0a16510119485c43e72812949d7874faf94db1666dc1c9fd`).
| Slice | Current evidence | Status |
| --- | --- | --- |
| Toolchain | UE 5.8.1 Editor, Visual Studio/MSVC, Windows SDK, and MuJoCo 3.10.0 are present on the Windows host and the canonical headless runtime binary exists. | Verified for Editor/runtime work; shipping support still needs Cook/package evidence. Blender is an authoring tool, not a runtime prerequisite. |
| RobotSimUE build and UE automation | RobotSimUE Win64 Editor build and automation pass are recorded in local evidence. | Editor build/automation verified. Cook/package not qualified. |
| Bundle identity | The plans, `session.runtime.json`, and `episode_result.json` carry the same `session_id`; `run-allocation.json` carries only runtime allocation. | Correct session/runtime-allocation boundary. |
| Physics to Visual same-run proof | The single-robot run contains 21 MuJoCo bodies and UE logs `LINGTU_VISUAL_FRAME_APPLIED ... bindings=21`. A second plan-driven session composes ThunderV4 plus OmniCart in one MuJoCo scene, streams 25 dynamic entities, applies 23 UE body bindings, and captures both robot types at 1920x1080. A separate inspected UE 5.8.1 hero render has 21 body bindings, 21 visual links, 105 VisualOnly props, zero material compile errors, and a matching success-sentinel hash. | G4 Editor visual/presentation and mixed-robot instantiation proofs pass. Cook/distribution remains the separate G9 proof. |
| Render sensors | RGB and depth each publish a real 640x480 frame through camera SHM and both streams are `ACTIVE` in the same Editor run. | Editor same-run render proof passes; not packaged-distribution proof. |
| IMU and truth odometry | Both typed DDS streams are `ACTIVE` in the same coordinated session and carry the shared `session_id` and generations. | Same-session G5 qualification passes. |
| Mid360 | MuJoCo raycast skips the owning robot subtree while preserving other-instance hits; the native typed publisher emitted 20 clouds / 4,304,048 bytes and preserves `offset_time_ns`, `reflectivity`, `tag`, and `line`. | Same-session G5 qualification passes. |
| ScenarioRuntime | Strict deterministic evaluator, `ScenarioDispatcher` seam, and Coordinator scheduling now use MuJoCo snapshots across prepare/warmup/advance/reset. | Coordinator dispatch contract passes fail-closed tests; concrete Physics/Visual dispatcher backends remain pending. |
| Recorder/replay/episode | A fresh Thunder run records 121 complete truth/command frames and closes `SUCCEEDED`; RobotSimUE then presents all 121 frames in an UE-only run with zero drops and no Physics process. SimStudio exposes validated run-owned timeline pages and single-frame detail through a read-only Replay Browser. A real UE Camera SHM run records and replays eight RGB/depth samples (6,144,000 referenced bytes) with content-addressed SHA-256 validation and zero drops. | Core recording, visual replay, bounded Studio controls, indexed/scrubbable evidence browsing, and RGB/depth payload recording pass; typed-DDS payload capture, Studio-triggered visual replay, and formal qualification verdicts remain pending. |
| Pixel Streaming | Presentation role is defined. | Qualification pending. |
| Cook/stage/package/DLC/Pak | Trusted policy, toolchain preflight, deterministic BuildCookRun plan, isolated staging, artifact hashing, and atomic release code are covered by tests. | Actual Shipping Cook is not yet evidence: the current managed Codex sandbox denies UnrealBuildTool's mandatory `%LOCALAPPDATA%/UnrealEngine/XmlConfigCache-*` write. Run outside that sandbox, then require packaged smoke, Pixel Streaming, artifact retention, and rollback proof. |

## Source And Migration Tree

Canonical ownership:

```text
sim/
  tools/, toolchains/                 current+target: authoring/import conditioning
  robots/                             Robot manifests, MJCF, meshes, visual projection
  controllers/                        Controller manifests, policy and runtime adapter
  sensors/                            Sensor manifests and simulation implementations
  sensor_rigs/                        Model-specific sensor mounting and calibration
  presets/                            Product-selected simulation compositions
  packages/
    worlds/                           current+target: World package manifests
    scenarios/                        current+target: Scenario package manifests
    payloads/                         current+target: Payload package manifests
  catalog/                            current+target: SessionCompiler/ResolvedSessionBundle
  runtime/
    coordinator/                      current+target: SessionRuntime lifecycle and RunAllocation
    physics/                          current+target: MuJoCo Physics Module
    control/                          current+target: Control Module
    visual/RobotSimUE/                current+target: Visual Module and render sensors
    sensors/                          current+target: Sensor Module scheduling/backends
    scenario/                         current: evaluator + dispatcher seam; target: concrete Physics/Visual sinks
  adapters/
    dds/                              current+target: typed native DDS Adapters
    shm/                              current+target: camera SHM Adapter
  evidence/
    recording/                        target-only logical home: recording docs and hooks; no empty scaffolding
    replay/                           target-only logical home: replay docs and hooks; no empty scaffolding
    episodes/                         target-only logical home: episode closure docs and hooks; no empty scaffolding
  validation/qualification/           target-only logical home: qualification gates and verdicts; no empty scaffolding
  distribution/                       target-only logical home: Cook/stage/package/DLC/Pak/ops docs and hooks; no empty scaffolding
  worlds/, assets/                    current+target: shared world and visual assets
  engine/, bridge/                    current: compatibility surfaces; target: compatibility/
  tests/, validation/, scripts/       current+target: gates and launchers
  datasets/, experiments/, planning/  current: validation/research; target: validation-owned or external artifact storage
```

The catalog scans these canonical roots directly. It does not scan retired
robot/controller/sensor package mirrors and does not provide compatibility
aliases for them.

## Delivery Gates

Remaining execution order is **finish G10 UE playable live acceptance -> broaden G6
authoring/assets -> finish G8 replay/qualification UI -> G9 Shipping proof**. G4/G5
component evidence is necessary input to G10; it is never a playable-product PASS.

| Gate | Scope | Current state | Next proof |
| --- | --- | --- | --- |
| G0 | Interface/authority | Pass | Keep MuJoCo as sole clock/physics authority, UE as follower/render owner, and Adapters non-authoritative. |
| G1 | Authoring conditioning | Partial | Robot/World guided import and FactoryPark SceneDraft publication now preserve source provenance, content identity, terrain alignment, exact primitive collision geometry, and a validated UE world-entity projection. Complete generalized frames/units/scale gates, collision/LOD/material/sensor surfaces, license policy, and production URDF/DCC qualification. |
| G2 | Packages/compiler | Pass | Keep Robot, Controller, Sensor, SensorRig, World, Scenario packages allocation-free; keep SessionCompiler deterministic with module-specific plans. |
| G3 | Physics/control | Pass | Preserve MuJoCo clock/physics authority, controller scheduling, actuator binding, reset generation, and stale-command safe stop. |
| G4 | UE visual + camera | Pass (component) | The live 21-body contract, both RGB/depth SHM streams, and an inspected 1920x1080 UE hero render pass in Editor evidence; these do not qualify the playable product, and Cook/distribution remains G9. |
| G5 | Native sensors | Pass (component) | IMU, truth odom, and Mid360 are `ACTIVE` in a coordinated SessionRuntime run; Mid360 field preservation and fail-closed identity/generation checks are covered. This evidence must be reproduced in the single G10 run. |
| G6 | Scenario dispatch and scene authoring | Pass for one deterministic pedestrian slice and one published static-world slice; authoring remains partial | Evaluator, Coordinator scheduling, authority filtering, generation/sequence checks, MuJoCo kinematic proxy/readback/raycast truth, Scenario VisualPlan sink, and validated static WorldPackage projection all pass. SimStudio publishes a validated FactoryPark SceneDraft, refreshes its Catalog, hands the new world to Session Composer, and RobotSimUE materializes its runtime entities. Next broaden to timeline authoring, animated pedestrian/crowd assets, vehicles, and larger-scale qualification without changing authority ownership. |
| G7 | Single-RunPlan Product integration | Pass | ProductControl/RunPlan carries Physics, Visual, Sensor, Control, Transport, and optional Scenario bundle; `src/lingtu/assembly/tests/test_simulation.py` passes all 27 tests. Preserve single-source resolution. |
| G8 | Recorder/replay/episode | Partial | Full truth/command recording, UE-only timestamp replay, session/generation checks, bounded SimStudio recording controls, identity-checked timeline/frame browsing, and real content-addressed RGB/depth payload capture/replay pass; add typed-DDS payload capture, Studio-triggered visual replay, and qualification verdicts. |
| G9 | Cook/distribution/ops | Partial | Trusted preflight/dry-run and atomic release implementation pass; execute Shipping Cook outside the current AppData-restricted sandbox, then require packaged smoke, Pixel Streaming qualification, DLC/Pak, artifact retention, and rollback proof for the same session plans. |
| G10 | UE5 playable vertical slice | Pending fresh live acceptance | The current Editor modules compile and pass 11/11 camera plus 86/86 full `LingTuSim` Automation. One provenance-bound run must still pass the 12-second performance gate, 60-second stability, real foreground input and upright/release-stop motion, fixed six-maneuver recording/HUD/shutdown evidence, and the independent strict verifier. Web previews and cross-run component evidence cannot satisfy this gate. |
