# Simulation runtimes

This directory owns execution logic for one resolved simulation session.

## Contents

| Module | Path | Responsibility |
| --- | --- | --- |
| Physics Runtime | `physics/` | Compose and advance one MuJoCo scene and emit immutable snapshots. |
| Runtime Coordinator | `coordinator/` | Validate a SessionBundle, allocate a run, and manage process lifecycle. |
| Visual Runtime | `visual/RobotSimUE/` | Present plan-driven worlds and robot snapshots in Unreal Engine. |
| Controller Runtime | `control/` | Plan-driven fixed-rate scheduling, stable actuator binding, generation gates, fail-closed command freshness, and registry-selected production adapters. ThunderV4 ONNX + PD and OmniCart analytic differential drive both reach named MuJoCo actuators. |
| Sensor Runtime | `sensors/` | Plan-driven multi-rate scheduling, immutable per-stream qualification, typed samples, and endpoint lifecycle. Truth odometry, IMU, and field-preserving Mid360 publish through native typed DDS; RobotSimUE RGB/depth publish through camera SHM in coordinated Editor runs. |
| Scenario Runtime | `scenario/` | Strictly load optional compiled scenario plans, validate authority/generation/clock contracts, and publish deterministic dynamic-entity snapshots from MuJoCo simulation time. |
| Recording | `recording/` | Persist run-owned truth, commands, episode closure, and referenced sensor payloads. |
| Replay | `replay/` | Validate recordings and present deterministic timeline or visual replay without becoming physics authority. |
| Qualification | `qualification/` | Build simulation verdicts and evidence records from completed runtime outputs. |
| Process/platform support | `process_owner.py`, `windows_cpu_isolation.py`, `windows_timing.py` | Own direct child process trees and optional Windows timing/CPU controls shared by runtimes. |

## Boundary

Protocol conversion does not belong here. Adapters live under
`sim/adapters/`; package parsing and resolution live under `sim/catalog/`.
Pixel Streaming is a presentation/input adapter, not a camera or navigation
sensor transport.

## Canonical package roots

The catalog has one manifest root for each package kind:

| Package kind | Canonical root |
| --- | --- |
| Robot | `sim/packages/robots/` |
| Controller | `sim/packages/controllers/` |
| Sensor | `sim/packages/sensors/` |
| Sensor rig | `sim/packages/sensor_rigs/` |
| World | `sim/packages/worlds/` |
| Scenario | `sim/packages/scenarios/` |
| Payload | `sim/packages/payloads/` |

Package-owned assets live below the same `sim/packages/<kind>/` root as their
manifests. Direct-engine compatibility scenes and Thunder v3 reference assets
live under `sim/compat/`; production runtime code must not discover packages
there.

Session readiness is deliberately two-level. `BindingReadiness` qualifies the
Physics, Visual, Sensors, and Control facets; `SensorReadiness` qualifies every
compiled SensorPlan stream. The Sensors facet becomes ACTIVE only after every
required stream is ACTIVE for the current model/reset generation.

## Migration compatibility seam

Package manifest discovery has one root: `sim/packages/`. The compiler resolves
that catalog into a SessionBundle; canonical runtimes consume the compiled
plans and must not probe package manifests or `sim/compat/` directly.

`runtime/control/thunderv4.py` is an isolated model-specific adapter selected
by a compiled controller contract. Likewise, RobotSimUE
`Scripts/build_thunderv4_preview.py` and `Scripts/build_open_field_hf.py` are
model/world-specific Unreal Editor builders for fixture/preview asset
conditioning and evidence only. They are not launch paths and must not be
invoked by the Runtime Coordinator, ProductControl, or a packaged simulation
runtime.
