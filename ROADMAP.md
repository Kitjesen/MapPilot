# LingTu Roadmap

Status: current summary as of 2026-08-10

The detailed active work board is
[`docs/plans/current-roadmap.md`](docs/plans/current-roadmap.md). Capability
maturity is tracked separately in
[`NAVIGATION_CAPABILITY_MATRIX.md`](docs/architecture/NAVIGATION_CAPABILITY_MATRIX.md).

## Current Target

LingTu is converging on a ROS-free, typed-DDS field runtime:

```text
native sensor -> native SLAM -> mapd / traversability -> navd
              -> /nav/cmd_vel -> driver -> Brainstem

HostBus <-> Gateway / Agent / MCP
```

`navd` owns navigation state and the final logical command. Standalone native
traversability owns `/nav/traversability`; `mapd` owns map state and scene.
The Python Host is an API/business integration boundary, not a duplicate
navigation or high-rate map runtime.

## Delivery Priorities

| Priority | Outcome | Status |
| --- | --- | --- |
| P0 | Self-contained native release and zero-blocker strict Product preflight. | Active |
| P0 | Stable MuJoCo mapping/no-map SLAM and `teleop_avoid` closed-loop acceptance. | Active |
| P0 | S100P no-motion readiness, fault injection, then supervised motion. | Blocked on preceding gates |
| P1 | Native typed map control/query as the only field path. | Active |
| P1 | Dynamic-obstacle residual and long-duration resource evidence. | Gate implemented; accepted run pending |
| P1 | Product integration of native velocity smoothing and a replaceable path smoother. | Partial |
| P2 | Generic route operations, target following, and docking/charging Products. | Foundations or missing |

## Simulation Roadmap

### Phase 1 — Current: MuJoCo (Active)

MuJoCo is the primary simulation backend for algorithm validation and Product
acceptance gates. It provides ray-cast LiDAR, simulated IMU, and kinematic/policy
drive modes for headless CI and visual debugging.

- **Scope**: Algorithm wiring, map artifacts, controlled scenario behavior
- **Limits**: Cannot prove real MID-360 timing, IMU noise, gait stability, or field safety
- **Entrypoints**: `python sim/scripts/policy_nav_smoke.py`, native-DDS acceptance scripts under `sim/scripts/mujoco/`

### Phase 2 — Active: Unreal Engine Rendering Layer

UE5 serves as the high-fidelity rendering and visualization front-end, while
MuJoCo remains the physics engine underneath. This separates concerns — MuJoCo
owns robot dynamics, contacts, and sensor ray-casting; UE5 owns visual output,
scene rendering, and the operator interface.

**Architecture:**

```text
UE5 (Rendering & Visualization)
  → Photorealistic scene rendering (Lumen/Nanite)
  → Camera output (RGB, depth, segmentation)
  → Operator UI and interactive viewer
  ↓  pose + sensor data sync (shared memory or gRPC)
MuJoCo (Physics Engine — unchanged)
  → Robot dynamics (contacts, joints, gait policy execution)
  → LiDAR ray-casting (MID-360 pattern)
  → IMU simulation
  ↓  odometry / registered_cloud / imu
LingTu Runtime (unchanged native DDS contract)
  → navd / mapd / traversability
  ↓  /nav/cmd_vel
  → MuJoCo robot controller
```

**Why this separation:**

| Concern | Owner | Reason |
| --- | --- | --- |
| Physics accuracy | MuJoCo | Deterministic, fast, already validated |
| Visual fidelity | UE5 | Lumen/Nanite photorealistic rendering |
| Robot dynamics | MuJoCo | ONNX gait policy, contact-rich simulation |
| Camera simulation | UE5 | Realistic sensor noise, lighting, materials |
| CI/CD determinism | MuJoCo | Headless, reproducible, fast |
| Demo & operator UX | UE5 | Immersive 3D viewer, multi-camera |

UE5 replaces MuJoCo's basic `MjvCamera` viewer, not its physics. The physics
engine and LingTu runtime are unchanged; UE5 is a rendering front-end that
consumes MuJoCo's state and renders it photorealistically.

**Current UE status:**
- Engine/toolchain: Unreal Engine 5.8.1 Editor and Win64 C++ build verified
- MuJoCo → UE sync: plan-driven UDP loopback truth snapshots with generation,
  sequence, and simulation-time checks; UE remains a non-authoritative follower
- Scene pipeline: immutable VisualPlan plus integrity-bound robot/world visual
  projections; ThunderV4 and OmniCart are instantiated in one live UE session
- Render sensors: real 640x480@30 RGB/depth Camera SHM streams are active and
  their payloads now pass content-addressed recording/replay verification
- Remaining target: generic terrain/material/asset management, richer production
  robot visuals, Pixel Streaming qualification, and Windows Shipping Cook/package

## Codebase Health (2026-08-04 Audit)

### Completed

- [x] Fixed `test_subprocess_runner.py` RunPlan simulation validation
- [x] Cleaned repository root (temp files, model artifacts, stale logs)

### In Progress

- [ ] Reorganize `sim/` directory to separate engine, tests, bridge, assets
- [ ] Add `src/message/` to architecture documentation
- [x] Move former runtime-profile helpers to their actual Product, assembly, navigation, and driver owners
- [ ] Add UE simulation placeholder under `sim/compat/engine/ue/`

### Known Gaps

- `src/runtime/` still mixes infrastructure, route contracts, and adapters — needs decomposition
- C++ code is spread across 6+ directories without a unified build convention
- `sim/` directory mixes engine code, acceptance tests, diagnostics, and experiments
- `src/message/` exists but is undocumented in AGENTS.md
- `src/native/` contents don't match AGENTS.md description

See [`docs/known_gaps.md`](docs/known_gaps.md) for the full architecture gap list.

## Evidence Rules

- Source and unit tests prove implementation, not Product completion.
- MuJoCo proves only the named simulation scenario.
- UE simulation proves perception and visual fidelity, not field readiness.
- No-motion readiness does not prove safe locomotion.
- Physical claims require date-prefixed evidence under
  `docs/07-testing/field-runs/`.
- Research and plans are never runtime truth; use `docs/CURRENT.md` to find the
  authoritative contract.
