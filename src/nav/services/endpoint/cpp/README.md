# Native Navigation Endpoint

This folder contains C++ native navigation processes.

| File | Role |
| --- | --- |
| `nav_native_endpoint.cpp` | Main process loop and control-mode ownership. It orchestrates DDS drains, OctoPlanner3D, `NavLoop`, safety gates, and output publication. |
| `nav_dds_runtime.*` | Typed CycloneDDS readers/writers owned by the endpoint. |
| `nav_endpoint_config.*` | CLI/environment parsing and `autonomy` / `teleop` / `teleop_avoid` contract. |
| `nav_endpoint_messages.*` | DDS-to-kernel decoding and frame validation. |
| `estop_latch_store.hpp` | Persistent software E-stop marker used across endpoint restarts. |
| `nav_control.cpp` | Diagnostic CLI only; Gateway uses `src/nav/commands/cpp/client.*` instead. |
| `traversability_dds.cpp` | DDS traversability/terrain producer. |
| `explore_dds.cpp` | Exploration goal publisher. |
| `motion_mock_dds.cpp` | Motion mock endpoint for tests/simulation. |
| `nav_status_writer.*` | JSON status file writer. |

`lingtu_nav_control teleop-stream` is the terminal/simulation diagnostic
ingress. It keeps one native command client alive, accepts latest `VX VY WZ`
lines on stdin, and drains queued input to the newest state. A 350 ms
input-heartbeat timeout sends typed zero plus stop and ends the stream; a fresh
process is required before motion can resume. `quit`, EOF, and error cleanup
use the same fail-closed zero-plus-stop boundary. It remains an intent client;
it never publishes final `/nav/cmd_vel` directly.


The heavy planning/following logic should stay outside this endpoint shell.
Current split:

```text
nav_native_endpoint.cpp
  -> nav_dds_runtime: typed DDS boundary
  -> process lifecycle + control mode
  -> src/nav/services/plan/cpp/nav_loop.cpp
     -> target selection, local planning, path following
```

`/nav/local_path` is an output/telemetry topic. `PathFollower` consumes the
in-memory local path within `NavLoop::tick()` before DDS publication.

Control modes:

- `autonomy`: accepts goals/global paths; rejects teleop velocity requests.
- `teleop`: accepts velocity requests with stale/limit gates; requires no SLAM.
- `teleop_avoid`: accepts velocity requests only with fresh localization and
  obstacle context; applies `TeleopSafety`.

The deployed endpoint persists its software E-stop under `/var/lib/lingtu`.
ClearEstop is accepted only with a fresh typed request and only after a zero
command is published successfully; restarting the process never clears it.

Remaining split target:

- keep `nav_native_endpoint.cpp` as a small process `main`;
- move goal handling and per-mode command arbitration into dedicated endpoint
  controllers without changing the direct planner/follower call chain.
