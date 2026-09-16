# Roadmap

**Status:** Active product roadmap
**Audience:** Product, runtime, release, simulation, and field maintainers
**Runs on:** Planning across local, simulation, release, and S100P targets

This page contains only remaining cross-domain work. Shipped behavior belongs
in the owning code, configuration, and current documentation.

## Fixed boundaries

- Public runtime environments are exactly `real` and `sim`.
- Product declarations are env-independent.
- ProductControl resolves once and owns the full lifecycle.
- `real` uses systemd; `sim` owns direct child processes.
- Native sensor, SLAM, maps, traversability, navigation, and driver processes
  own the field hot path through typed DDS.
- `navd` owns final navigation arbitration; only `lingtu-driver` forwards the
  checked command to the selected robot adapter.
- Blueprint owns only the in-process Python Host Module graph.
- Windows x64, Linux/WSL x86_64, and S100P aarch64 require separate evidence.

## Active work

| Priority | Work | Completion gate |
| --- | --- | --- |
| P0 | Native MuJoCo Product parity | Every Product passes lifecycle, scenario, terminal-zero, cleanup, rollback, and repeatability on Windows and Linux/WSL independently |
| P0 | Canonical native release | `install/.../{bin,lib,etc,share}` passes package, install, activation, and rollback without production references to `build/` |
| P0 | Stable mapping/no-map `teleop_avoid` | Native sensor-to-command flow survives representative free-space, obstacle, stop, cleanup, and repeatability scenarios |
| P0 | S100P physical command safety | Fresh provenance, no-motion, fault injection, bounded supervised motion, terminal zero, and driver ACK pass on target |
| P1 | Dynamic-obstacle clearing and resource bounds | Labelled replay and long MID-360 runs meet residual, thin-obstacle, reset, CPU, memory, and DDS-volume limits |
| P1 | Motion and collision-aware path smoothing | Ramp, reversal, rotation, emergency zero, and safety ordering pass with an explicit path-smoother failure policy |
| P1 | Typed native map control/query | Persistent field operations no longer require a competing Python map-management path |
| P1 | Online global mapping backend | Resolve rejected-keyframe coverage, verify corrected save/reload, measure paced ARM performance, then complete supervised mapping/navigation; follow [the mapping execution plan](../src/localization/opt/NEXT_STEPS.md) |
| P2 | Route, following, and docking Products | Each has an explicit Product, typed lifecycle, safety boundary, simulation evidence, and physical evidence where applicable |
| P2 | Cross-language schema stability | DDS and shared-memory payloads carry explicit version, frame, timestamp, bounds, and compatibility tests |

## Execution order

1. Build each selected native artifact from the current source and IDL.
2. Resolve the exact Product and reject stale, missing, mixed-platform, or
   undeclared artifacts.
3. Run ProductControl from a fresh state root.
4. Attach the named simulation scenario to the same Product session.
5. Prove terminal zero, cleanup, and rollback.
6. Repeat independently on Windows and Linux/WSL.
7. Assemble and validate the canonical Linux field release.
8. Run S100P no-motion and fault injection before supervised motion.

## Terrain research threshold

Do not add PCA ground segmentation merely because another project uses it.

Only clean-room implement the necessary portion if representative real MID-360
data shows that the current terrain core repeatedly misclassifies slopes,
stairs, or mixed cells and the failure changes motion safety.

The acceptance set must compare the current terrain owner and the candidate on
the same labelled data, including thin obstacles, reset behavior, CPU, memory,
and latency on the S100P target. Simulation alone cannot trigger adoption.

## Release and license gates

FAST_LIO, ikd-Tree, and IKFoM provenance and license obligations require an
explicit release decision. Replacing one container does not establish a
closed-source distribution boundary.

Before commercial distribution, choose one complete path:

- authorization covering the relevant copyright chain;
- a reviewed GPL-compliant distribution boundary; or
- a clean-room replacement of the affected SLAM core.

This is an engineering release gate, not legal advice.

## Non-goals

- Do not make ROS 2 the Product API.
- Do not expose planner backend internals to Gateway or Web.
- Do not add a second Product lifecycle, map hot path, field planner, or command
  writer.
- Do not duplicate algorithms under root `real/` and `sim/` trees.
- Do not add a custom transport framework before the typed schema boundary is
  stable.

## Claim discipline

Unit tests do not prove a Product. WSL does not prove Windows-native behavior.
Simulation does not prove S100P motion safety. A roadmap item closes only when
its named evidence gate passes and the owning contract is updated.
