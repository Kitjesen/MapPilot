# Native Navigation Endpoint

Status: current native endpoint contract as of 2026-07-18.

This folder contains C++ native navigation processes.

## What an endpoint is

In LingTu, an endpoint is a long-running process at a process or machine
boundary. It owns transport, process lifecycle, readiness, and fail-closed
control authority. It is not a planner algorithm, a domain service, a REST
route, or a Blueprint.

For native navigation, the relationship is:

```text
Profile / runtime graph
  -> selects required endpoint binaries and topic contracts

Gateway or native client
  -> typed navigation command over DDS
  -> navd
       -> direct C++ calls: selected OctoPlanner3D/FAR + NavLoop
       -> NavLoop: LocalPlanner + PathFollower
       -> input gates + TeleopSafety + E-stop ownership
       -> typed final /nav/cmd_vel over DDS
  -> lingtu-driver
  -> robot hardware
```

Blueprint assembles in-process Modules. The native endpoint is the product
process used when navigation crosses the Python runtime boundary or runs on the
robot as a separately supervised service. DDS topics connect that process to
SLAM, maps, Gateway/native clients, and the driver. Algorithms remain direct
C++ calls inside the process; they are not split into extra DDS nodes.

Sibling endpoints have narrower ownership:

- `lingtu_traversability_dds` converts synchronized motion and cloud input into
  terrain/traversability products.
- `lingtu_explore_dds` consumes exploration grids and publishes exploration
  goals.
- `lingtu-driver` is the only process allowed to forward final motion to the
  physical robot.

| File | Role |
| --- | --- |
| `nav_native_endpoint.cpp` | Main process loop and control-mode ownership. It orchestrates DDS drains, selected global planner, `NavLoop`, safety gates, and output publication. |
| `active_octomap_gate.*` | Validates and snapshots the active 3D artifact for OctoPlanner3D. |
| `active_occupancy_gate.*` | Validates and snapshots the active trinary grid for FAR. |
| `nav_dds_runtime.*` | Typed CycloneDDS readers/writers owned by the endpoint. |
| `nav_endpoint_config.*` | CLI/environment parsing and `autonomy` / `teleop` / `teleop_avoid` contract. |
| `nav_endpoint_messages.*` | DDS-to-kernel decoding and frame validation. |
| `estop_latch_store.hpp` | Persistent software E-stop marker used across endpoint restarts. |
| `nav_control.cpp` | Diagnostic CLI only; Gateway uses `src/nav/cpp/client/client.*` instead. |
| `traversability_dds.cpp` | DDS traversability/terrain producer. |
| `explore_dds.cpp` | Exploration goal publisher. |
| `motion_mock_dds.cpp` | Motion mock endpoint for tests/simulation. |
| `nav_status_writer.*` | JSON status file writer, including input-gate and driver-control readiness fields. |

`lingtu_nav_control teleop-stream` is the terminal/simulation diagnostic ingress.
It keeps one native command client alive, accepts latest `VX VY WZ` lines on
stdin, and drains queued input to the newest state. A 350 ms input-heartbeat
timeout sends typed zero plus stop and ends the stream; a fresh process is
required before motion can resume. `quit`, EOF, and error cleanup use the same
fail-closed zero-plus-stop boundary. It remains an intent client; it never
publishes final `/nav/cmd_vel` directly.

The heavy planning/following logic should stay outside this endpoint shell.
Current split:

```text
nav_native_endpoint.cpp
  -> nav_dds_runtime: typed DDS boundary
  -> process lifecycle + control mode
  -> src/nav/cpp/engine/nav_loop.cpp
     -> target selection, local planning, path following
```

`/nav/local_path` is an output/telemetry topic. `PathFollower` consumes the
in-memory local path within `NavLoop::tick()` before DDS publication.

Control modes:

- `autonomy`: accepts goals/global paths; rejects teleop velocity requests.
- `teleop`: accepts velocity requests with stale/limit gates; requires no SLAM.
- `teleop_avoid`: accepts velocity requests only with fresh localization and
  obstacle context; applies `TeleopSafety`.

Field autonomy and assisted control require fresh driver-control readiness from
`lingtu-driver`. The endpoint treats missing, stale, rejected, or lease-lost
driver control state as a fail-closed input-gate blocker and clears endpoint
motion rather than continuing with cached commands. This is a hardware-output
gate only; simulation harnesses can still use isolated mock/tap consumers when
their manifests declare a simulation command sink.

The deployed endpoint persists its software E-stop under `/var/lib/lingtu`.
ClearEstop is accepted only with a fresh typed request and only after a zero
command is published successfully; restarting the process never clears it.

`/nav/cmd_vel` publication is final command output. Python `CmdVelMux` and
legacy global-path publishers must not run as competing writers in the product
field endpoint.

OctoPlanner3D remains the default. FAR is selected only with
`LINGTU_NAV_GLOBAL_PLANNER=far` and consumes the validated active
`occupancy.npz`; unknown-space fallback remains disabled unless explicitly
enabled. The status snapshot always reports `global_planner` and `planner_map`
so Gateway readiness can reject deployment/profile drift.

Remaining split target:

- keep `nav_native_endpoint.cpp` as a small process `main`;
- move goal handling and per-mode command arbitration into dedicated endpoint
  controllers without changing the direct planner/follower call chain.
