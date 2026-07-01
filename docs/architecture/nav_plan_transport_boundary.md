# Navigation Planner Transport Boundary

This document defines the execution boundary around LingTu local planning.  The
goal is to let the current in-process `nav.local_planner` and a future standalone
C++ local-planner process share the same data contract.

## Runtime Choice

Default runtime is in-process direct callbacks:

```text
NavigationModule -> LocalPlannerModule -> PathFollower/SafetyRing
```

This is the lowest-latency path and remains the default for safety-critical
robot operation.

Set `nav_plan_transport` to make the local planner boundary explicit:

```bash
python lingtu.py sim_nav --nav-plan-transport local
python lingtu.py sim_nav --nav-plan-transport shm
python lingtu.py sim_nav --nav-plan-transport dds
```

The setting only affects Navigation <-> LocalPlanner execution wires.  It does
not move `SafetyRing`, `CmdVelMux`, or driver command wires away from their
normal low-latency path.

## Transport Policy

Use direct callbacks when modules run in the same Python process.

Use `shm` for high-bandwidth same-host data such as terrain point clouds or
future dense local grids.

Use `dds` for low-frequency semantic/control streams across processes or
machines, such as goals, paths, planner status, and operator-visible state.

Use `dual` only when the same topic must be visible to both same-host native
processes and external DDS observers.

## Local Planner Topics

| Topic | Producer | Consumer | Purpose |
| --- | --- | --- | --- |
| `/nav/global_path` | `nav.mission` | `nav.local_planner` | Global route corridor |
| `/nav/way_point` | `nav.mission` | `nav.local_planner` | Current staged local goal |
| `/nav/local_planner/clear_path` | `nav.mission` | `nav.local_planner` | Drop stale local path |
| `/nav/terrain_map` | `nav.terrain` | `nav.local_planner` | Near-field obstacle cloud |
| `/nav/terrain_map_ext` | `nav.terrain` | `nav.local_planner` | Extended near-field obstacle cloud |
| `/nav/traversability` | `nav.terrain` | `nav.local_planner` | Risk grid and terrain class |
| `/nav/local_path` | `nav.local_planner` | `nav.path_follower`, `nav.safety` | Trackable local path |
| `/nav/local_planner/control_hint` | `nav.local_planner` | `nav.path_follower` | Slow/stop/recovery hints |

## Direct vs Indirect Data

Small messages are passed directly by value through the selected transport.

Large point-cloud/grid messages should move through shared memory or an indirect
artifact reference once the C++ process boundary is enabled.  The canonical
topic name remains the same; only the payload representation changes behind the
transport adapter.

## C++ Process Migration

The standalone C++ local planner should subscribe to the same input topics and
publish the same output topics.  LingTu Python should then replace the
in-process `LocalPlannerModule` with a thin process supervisor, not duplicate
planning decisions in Python.

