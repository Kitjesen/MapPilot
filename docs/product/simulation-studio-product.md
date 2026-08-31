# LingTu Simulation Product Definition

Status: **confirmed product boundary; first playable slice live qualification pending**
Date: 2026-08-12
Benchmark: [zsibot/MATRiX at `6ec0b354`](https://github.com/zsibot/matrix/tree/6ec0b354b93b0dd0ccdfb2d1c012fb5cc3f52a30)
(2026-08-09 UTC; immutable documentation snapshot)

## Register

product

## Problem Statement

LingTu already has substantial robot simulation infrastructure: deterministic
packages and SessionBundles, MuJoCo physics/control, UE5 visual projection and render
sensors, FactoryPark, scenario dispatch, recording, replay, qualification, and a Web
management service. The user still does not experience it as one coherent product.

The main failure is a product-surface mismatch. A Web authoring mock was treated as
the desired UE5 game interface, while the actual UE runtime remained a read-only
engineering HUD. The system can produce impressive evidence, but a user cannot yet
open LingTu, choose a prepared robot/world/sensor configuration, take visible control,
understand authority and readiness, perform a complete maneuver, record it, and close
the run from one game-like experience.

MATRiX demonstrates the missing product layer: UE is the primary simulation
experience, MuJoCo remains the robot-physics engine, and robot/map/sensor selection,
runtime loading, controls, DLC maps, networking, and distribution form a usable
simulation product. LingTu should learn from that product structure without copying
MATRiX's transport or weakening LingTu's stronger package, safety, identity, and
evidence contracts.

This benchmark is deliberately documentation-level. The selected public MATRiX
snapshot describes the product structure, but its repository tree does not contain
the referenced UeSim project, controller implementation, MJCF/config payloads, or
Pak/DLC binaries. LingTu therefore treats those descriptions as product-design input,
not as independently reproduced implementation or performance evidence. The MATRiX
README calls the current documentation `v1.0.7`, while the repository `VERSION` and
GitHub releases use different versioning; this document pins a commit rather than
claiming a corresponding release artifact.

## Product Thesis

LingTu Simulation is an evidence-first, game-like UE5 robot simulation product for
industrial inspection development and validation. It lets users configure a
session identified by `session_id`, operate and observe a robot in a high-fidelity
world, create controlled worlds and scenarios, and obtain reproducible evidence
whose physics truth comes only from MuJoCo.

“Game-like” describes immediacy, spatial interaction, camera language, input quality,
and progressive disclosure. It must never turn simulation truth into fictional health,
power, rarity, score, or animation-only status.

## Current Delivery Status

The three product surfaces and the single end-to-end acceptance seam below are the
current implementation contract. They are no longer a proposal. The first playable
slice is **not yet qualified**. The dated build, Automation, and latest live-failure
evidence is recorded in
[`2026-08-12 UE5 playable pre-acceptance`](../07-testing/field-runs/2026-08-12-ue5-playable-preacceptance.md):

- The Windows `RobotSimUEEditor` target, camera deadline policy, UE input/status/HUD
  modules, and the exact Session/Visual/Sensor contracts compile on UE 5.8.1.
- The dated camera Automation report has `11/11` successful tests and the full
  `LingTuSim` report has `86/86` successful tests (67 without warnings, 19 with
  expected test-log warnings, zero failed or not-run).
- The latest live attempt failed before `RUNNING` because a cold first camera frame
  exhausted the former one-second readback deadline. The stage-specific startup
  deadline fix is compiled and Automation-tested, but it still requires a fresh live
  run before it can support a product claim.
- Remaining product evidence is one fresh 12-second performance gate, the 60-second
  stability gate, foreground `Shift+W` upright motion and release-stop, the complete
  recording/HUD/shutdown bundle, and an independently revalidated strict verdict.

No Web preview, component test, offline screenshot, Automation report, or successful
process startup can substitute for that final same-run evidence.

## Users

1. **Simulation operator / demonstrator** — launches a prepared world and robot,
   drives or supervises it, changes view, records a run, and explains results.
2. **Robot engineer** — validates controllers, motion, sensors, coordinates, timing,
   resets, and failure behavior against MuJoCo truth.
3. **World/scenario designer** — builds industrial worlds, places qualified elements,
   authors routes and dynamic actors, validates physics representation, and publishes
   immutable versions.
4. **Validation engineer** — runs repeatable qualifications, compares recordings,
   inspects sensor/evidence integrity, and rejects incomplete claims.
5. **Package administrator/integrator** — imports, versions, qualifies, promotes,
   distributes, and retires robots, worlds, sensors, controllers, and scenarios.

## Product Surfaces

### 1. UE5 Runtime — primary playable product

Owns:

- High-fidelity world presentation and render sensors.
- Robot-follow, inspection, free, and tactical cameras.
- Keyboard/gamepad input as authorized control intent.
- A quiet runtime HUD showing objective, robot, control owner, readiness, sensor,
  recording, replay, and qualification state.
- Runtime menu, camera selection, recording controls, and safe stop/exit requests.
- Visible dynamic scenarios and immutable MuJoCo truth projection.

Does not own:

- Physics, contacts, robot truth, simulation time, or authoritative pause state.
- Independent package resolution or source MJCF/YAML parsing.
- Optimistic readiness or local-only “success” animation.

### 2. UE5 Create — immersive 3D authoring mode

Owns:

- Selecting qualified catalog elements in spatial context.
- Placement previews, snapping, move/rotate/scale constraints, properties, undo/redo,
  routes, spawn points, sensor mounts, and scenario timeline interactions.
- Visual distinction between PhysicsShared and VisualOnly elements.
- Submitting revisioned authoring operations and displaying validation diagnostics.

Does not own:

- Direct mutation of published packages or runtime physics truth.
- Unqualified arbitrary assets that bypass provenance, units, collision, or license
  gates.
- Publishing without a validated SceneDraft revision and immutable WorldPackage.

### 3. SimStudio — management and evidence product

Owns:

- Package library, import, versioning, qualification, promotion, and administration.
- Session composition, prepared launch profiles, runs, lifecycle, and concurrency.
- Recording/replay/evidence/qualification browsing and release evidence.
- Advanced diagnostics and exact manifests, artifact identities, bindings, and errors.

Does not own:

- Pretending a static Web image is the UE live world.
- Being the primary spatial driving or 3D authoring surface.
- Replacing UE with a Web “game” shell.
- Advertising `env=sim` field Products that the current environment mapping cannot
  actually resolve and launch.

## Core Product Journeys

### Play / Validate

1. Choose a prepared World, Robot, SensorRig, Controller, and optional Scenario.
2. Prepare a deterministic SessionBundle and show blockers before launching UE.
3. Enter the UE world and wait for Physics, Control, Visual, and Sensors to become
   current-generation ready.
4. Take explicit control, drive the robot, switch camera, inspect tactical context,
   and observe requested, accepted, and measured motion separately.
5. Start/stop a recording and close the session safely.
6. Open the resulting evidence, trajectory, sensor summary, video, and qualification
   verdict in SimStudio.

### Create / Publish

1. Open a revisioned SceneDraft in UE Create mode.
2. Select qualified elements, place and edit them in 3D, author route/spawn/scenario
   data, and undo mistakes.
3. Validate boundaries, support surfaces, stable IDs, units, collision authority,
   performance budget, and provenance.
4. Publish one immutable WorldPackage version.
5. Select that exact version in a new session and prove the same geometry exists in
   MuJoCo truth and UE presentation.

### Review / Reproduce

1. Select one immutable recording/evidence set by run ID and `session_id`.
2. Inspect exact lifecycle, command, truth, sensor, visual, and qualification facts.
3. Replay at recorded time without silently launching a second physics authority.
4. Compare only with verifier-owned tolerances and retain an auditable verdict.

## User Stories

1. As a simulation operator, I want one obvious way to launch a prepared session, so that I do not assemble runtime commands manually.
2. As a simulation operator, I want to see the selected world, robot, sensor rig, and controller before launch, so that I do not run the wrong configuration.
3. As a simulation operator, I want blockers shown before UE opens, so that a missing package or mismatched session never becomes a half-running session.
4. As a simulation operator, I want the UE world to be the primary experience, so that the product feels spatial and immediate rather than like a dashboard.
5. As a simulation operator, I want a calm HUD, so that the world remains visible and important exceptions receive attention.
6. As a simulation operator, I want keyboard and gamepad controls, so that I can operate the robot naturally during demonstrations and tests.
7. As a simulation operator, I want camera controls separate from robot controls, so that looking around cannot accidentally command motion.
8. As a simulation operator, I want the current control owner shown, so that I know whether teleoperation, autonomy, replay, or a safety stop has authority.
9. As a simulation operator, I want requested input, accepted command, and measured motion distinguished, so that a visual response cannot hide rejected control.
10. As a simulation operator, I want a safe stop that remains visible without opening a menu, so that I can stop motion immediately.
11. As a simulation operator, I want the menu to request an authoritative pause, so that a local overlay never falsely claims the simulation is paused.
12. As a simulation operator, I want follow, inspection, free, and tactical views, so that I can operate and explain the run from useful perspectives.
13. As a simulation operator, I want one-button recording controls, so that evidence is captured without leaving the world.
14. As a simulation operator, I want recording state and elapsed time visible, so that I know whether evidence is actually being committed.
15. As a robot engineer, I want the HUD to show simulation time, snapshot age, generation, and control rate, so that stale projection is obvious.
16. As a robot engineer, I want each required sensor to show identity and positive sample count, so that “configured” is not confused with “working.”
17. As a robot engineer, I want robot visuals bound by stable body IDs, so that the rendered pose can be traced to MuJoCo truth.
18. As a robot engineer, I want reset generation changes shown and stale frames rejected, so that a reset cannot blend two simulations.
19. As a robot engineer, I want motion qualification based on body-frame translation and unwrapped yaw, so that camera movement cannot pass a robot test.
20. As a robot engineer, I want failed maneuvers to name direction, magnitude, and drift, so that controller defects are actionable.
21. As a world designer, I want to select catalog elements in UE, so that I can compose a world in its real spatial context.
22. As a world designer, I want placement previews and support-surface validation, so that invalid geometry is rejected before publication.
23. As a world designer, I want PhysicsShared and VisualOnly clearly distinguished, so that appearance never silently becomes physics authority.
24. As a world designer, I want move, rotate, constrained scale, properties, and undo/redo, so that authoring is practical rather than a read-only list.
25. As a world designer, I want revision conflicts detected, so that concurrent work cannot overwrite another draft silently.
26. As a world designer, I want publication to create a new immutable version, so that past sessions remain reproducible.
27. As a scenario designer, I want routes, triggers, pedestrians, vehicles, and behavior timing on one timeline, so that dynamic tests are repeatable.
28. As a validation engineer, I want one evidence directory to prove the whole playable run, so that separate green checks cannot be combined into a false claim.
29. As a validation engineer, I want screenshots and video mapped to exact truth frames, so that visual media remains auditable.
30. As a validation engineer, I want missing sensor, recording, UI, or motion evidence to fail closed, so that partial runs are never marketed as complete.
31. As a validation engineer, I want replay to use verifier-owned tolerances, so that a recording cannot declare its own easy pass criteria.
32. As a package administrator, I want every robot, world, sensor, controller, and scenario versioned with provenance, so that a session can be reconstructed.
33. As a package administrator, I want Cook/package/DLC readiness separate from Editor readiness, so that development evidence is not presented as a distributable product.
34. As a product owner, I want the public capability matrix backed by dated evidence, so that roadmap language never outruns delivered behavior.

## MATRiX Benchmark Decisions

Evidence basis: the pinned [README](https://github.com/zsibot/matrix/blob/6ec0b354b93b0dd0ccdfb2d1c012fb5cc3f52a30/README.md#L15-L37),
[maintainer guide](https://github.com/zsibot/matrix/blob/6ec0b354b93b0dd0ccdfb2d1c012fb5cc3f52a30/docs/MAINTAINER_GUIDE.md#L7-L31),
[robot/map guide](https://github.com/zsibot/matrix/blob/6ec0b354b93b0dd0ccdfb2d1c012fb5cc3f52a30/docs/Robots_and_Maps.md#L54-L65),
and [motion-control guide](https://github.com/zsibot/matrix/blob/6ec0b354b93b0dd0ccdfb2d1c012fb5cc3f52a30/docs/Motion_Control_CN.md#L7-L16)
describe the UE + MuJoCo split, launcher-selected robot/map/sensors, runtime control
roles, Pak/DLC map loading, networking, and operator controls. Conditional or
unpublished capabilities—including exact performance, shipped DLC contents, Linux
runtime availability, and the private UeSim implementation—are not accepted as proven
benchmark facts.

| MATRiX capability | LingTu decision |
| --- | --- |
| UE + MuJoCo split | Keep; LingTu already enforces a stronger immutable-truth boundary |
| Graphical robot/map/sensor configuration | Provide a product launcher/prepared-session surface; do not make UE re-resolve packages |
| Runtime MJCF | Keep behind compiler-produced PhysicsPlan and SessionBundle rather than letting UE parse arbitrary source files |
| Map DLC/Pak | Adopt after the first playable slice and Shipping Cook proof |
| Zenoh | Do not copy by default; keep LingTu typed DDS/SHM and explicit adapters |
| Gamepad/embedded control | Adopt game-quality input, but route it through LingTu control authority and evidence |
| Pixel Streaming | Add after local packaged runtime passes; treat it as presentation/input only |
| Multi-robot and RL | Preserve architectural compatibility; do not put them in the first playable slice |

## First Playable Vertical Slice

### Name and single acceptance seam

`lingtu.sim.ue5-playable-vertical-slice.v1`

The only top-level pass is:

```text
ResolvedSessionBundle
  -> one owned FactoryPark RobotSimUE runtime
  -> UE human input intent
  -> LingTu Control authority
  -> MuJoCo truth
  -> UE truth projection + HUD
  -> one committed evidence bundle
```

Component tests remain necessary, but they cannot independently qualify the product.

### Fixed content

- World: `factory_park_hf`
- Robot: black/graphite `thunderv4`, instance `thunder_01`
- Sensor rig: Thunder navigation rig with RGB, depth, IMU, Mid360, and truth odom
- Controller: declared ThunderV4 locomotion controller package
  (`thunderv4_locomotion@1.0.0`)
- Required runtime bindings: Physics, Control, Visual, Sensors
- Runtime UI modes: Drive, Tactical, Menu; Build remains an honest preview of the
  second product slice and must not claim editing capability

### Required interactions

- Keyboard: forward, backward, left, right, turn left, turn right.
- Gamepad: equivalent movement plus camera look where supported.
- Robot input and camera input are separate mappings.
- Camera: robot follow, inspection, free/tactical toggle.
- Menu: resume, request pause/resume, stop recording, safe stop, exit.
- Recording: start, active, stop/commit with visible authoritative state.

### Required HUD truth

- Session identity and current model/reset generation.
- Physics/Control/Visual/Sensors readiness.
- Robot and world identity.
- Current control owner and safety state.
- Requested command versus accepted/observed motion.
- Snapshot age/simulation time.
- Required sensor sample counts or exact blockers.
- Recording state, elapsed time, and committed artifact identity.
- Current camera and UI mode.

### Motion acceptance

- Forward/backward/left/right each produce command-aligned body-frame displacement
  of at least `0.08 m` in the expected signed direction.
- Left/right turn each produce command-aligned unwrapped yaw of at least `0.35 rad`.
- Each nominal in-place turn has horizontal drift no greater than `0.10 m`.
- Time, frame span, quaternion, sequence, and generation evidence are complete and
  monotonic.
- Nonzero input with insufficient motion, wrong direction, excessive drift, or stale
  truth fails the slice.

### Evidence acceptance

- Same `run_id`, `session_id`, model generation, and reset generation across all
  required evidence.
- At least one 1920×1080 UE first-frame capture showing FactoryPark and the black
  ThunderV4.
- A continuous labelled and raw recording of at least 20 seconds. Each required
  30 Hz render stream must contribute at least 600 mapped source frames, with no
  black-frame event or decode error.
- Truth trajectory and input/accepted-command mapping for all six maneuvers.
- Exact five-stream sensor summary with positive current-generation samples.
- UI evidence for Drive, Tactical, and Menu plus recording-active state.
- Clean runtime shutdown, zero post-exit command, and no owned UE/MuJoCo process leak.
- A single final verdict that rejects missing, mismatched, stale, unsupported, or
  partially captured facets.

## Implementation Decisions

- Preserve the existing Package/Catalog/SessionCompiler/SessionBundle architecture.
- Implement UE controls as a new input adapter into the existing Control boundary;
  do not let UI code write robot transforms or call MuJoCo directly.
- Keep the HUD's read model separate from transport mailboxes and simulation
  authority. It observes current-generation public runtime state.
- Evolve the current Slate module only where it provides stable C++ view-model and
  automation seams. Introduce UMG/Common UI only after explicitly enabling the
  plugins and proving focus/input/lifecycle behavior.
- Use one runtime UI state model for keyboard, gamepad, displayed mode, and automated
  evidence; avoid parallel Blueprint and C++ state machines.
- Treat authoritative pause, safe stop, recording, and exit as asynchronous requests
  with pending/accepted/rejected/confirmed states.
- Keep the first slice single-robot and fixed-content to exercise the real pipeline
  rather than building a broad but unqualified launcher.
- Drive the first slice from a known ResolvedSessionBundle. Do not imply that every
  field Product (`map`, `nav`, `inspection`, and others) is already supported by the
  current `env=sim` ProductControl mapping.
- The second slice extends the same runtime with revisioned UE Create operations and
  WorldPackage publication; it does not replace the first slice.
- SimStudio remains the deep package/run/evidence surface and consumes the same
  runtime/evidence contracts.

## Testing Decisions

- The highest test seam launches one owned, non-mocked RobotSimUE + MuJoCo runtime
  from a ResolvedSessionBundle and evaluates one evidence bundle after clean shutdown.
- C++ Automation covers UI mode/input state, viewport lifecycle, status read models,
  focus, and fail-closed unavailable states.
- Python contract tests cover launch assembly, process ownership, session/generation
  identity, evidence completeness, frame freshness, and final verdicts.
- Real MuJoCo evidence proves commanded body motion; UE screenshots/video prove only
  projection and interaction, never physics.
- Existing motion-recording, live-visual, FactoryPark, visual replay, sensor evidence,
  and UE Automation tests are reused rather than duplicated.
- Adversarial tests cover missing bindings, stale frame directories, bad map fallback,
  no motion, wrong-direction motion, excessive turn drift, sensor zero samples,
  recording failure, UE exit, and process leak.

## Out of Scope for the First Playable Slice

- Free-form text-to-world generation.
- Full UE 3D placement/edit/publish workflow.
- Multi-robot live control.
- Reinforcement-learning training.
- Pixel Streaming and browser control.
- Runtime DLC/Pak mounting.
- Linux distribution.
- Shipping installer and public release.
- A fictional game progression, economy, combat, avatar, or score system.

## Further Notes

- The current worktree contains substantial untracked simulation and UE UI code.
  Product work must not be considered safely delivered until those files are
  inventoried, reviewed, and committed intentionally.
- The public GitHub repository does not currently expose all local simulation work;
  GitHub is therefore not yet a complete project-status source.
- No issue-tracker publishing integration is configured in this workspace. This file
  is the local product source of truth; implementation work must preserve this
  boundary directly.
- Any issue decomposition must preserve the single end-to-end acceptance seam rather
  than turning component greens into a product PASS.
