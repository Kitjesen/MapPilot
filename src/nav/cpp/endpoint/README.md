# Native Navigation Endpoint

Status: current native endpoint contract as of 2026-07-24.

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
| `input/nav_input_state_projector.*` | Projects ordered TF, odometry, driver-control, cloud, terrain, traversability, and localization samples into endpoint state; owns input-gate evaluation, epoch-local recovery state, and deferred obstacle snapshots. |
| `plan/active_octomap_gate.*` | Validates and snapshots the active 3D artifact for OctoPlanner3D. |
| `plan/active_occupancy_gate.*` | Validates and snapshots the active trinary grid for FAR. |
| `plan/global_plan_task.*` | Async global-plan work and stale-result identity checks. |
| `plan/goal_plan_controller.*` | Owns goal admission, async completion, request lifecycle, stale-plan rejection, and rolling-to-generic handoff ordering. |
| `plan/rolling_map_segment_executor.*` | Plans and revalidates the observed-free rolling-map prefix used beyond the saved-map boundary. |
| `plan/rolling_segment_lifecycle.*` | Transport-free event/effect lifecycle plus rolling-segment executor orchestration for admission, idempotency, cancellation, terminal retry, and fail-closed recovery. |
| `plan/rolling_segment_effect_coordinator.*` | Applies ordered lifecycle effects, reports transport success/failure, and recursively executes fail-closed feedback without owning DDS or `NavLoop`. |
| `nav_dds_runtime.*` | Typed CycloneDDS readers/writers owned by the endpoint. |
| `nav_endpoint_config.*` | CLI/environment parsing and `autonomy` / `teleop` / `teleop_avoid` contract. |
| `nav_endpoint_messages.*` | DDS-to-kernel decoding and frame validation. |
| `inspection/inspection_runtime_controller.*` | Owns transport-free inspection evidence, settling, progress, map identity, goal/action dispatch, and status cadence orchestration. |
| `inspection/inspection_command_coordinator.*` | Owns inspection command validation, FIFO idempotent ACK replay, Start/Pause/Resume/Cancel admission, zero-motion downgrade, and observable ACK publication feedback. |
| `motion/command_ingress_controller.*` | Owns typed-command validation, idempotency journals, replay, dispatch, and ACK diagnostics. |
| `motion/motion_stop_coordinator.*` | Owns ordered fail-closed motion clearing, E-stop transitions, driver-authority loss, and confirmed final shutdown zero. |
| `motion/teleop_admission_controller.*` | Owns transport-free teleop freshness, takeover admission, fail-closed zero policy, and admission diagnostics. |
| `motion/teleop_tick_controller.*` | Computes one transport-free assisted/direct teleop safety tick and returns publication intents and diagnostics. |
| `motion/autonomy_tick_controller.*` | Computes one transport-free NavLoop/final-safety tick and returns rolling, goal, and inspection outcomes. |
| `motion/estop_latch_store.hpp` | Persistent software E-stop marker used across endpoint restarts. |
| `motion/nav_control.cpp` | Diagnostic CLI only; Gateway uses `src/nav/cpp/client/client.*` instead. |
| `traversability/traversability_dds.cpp` | DDS traversability/terrain producer. |
| `explore/explore_dds.cpp` | Exploration goal publisher. |
| `explore/explore_goal_lifecycle.*` | DDS-free matching and terminal-state handling for TARE goals. |
| `motion/motion_mock_dds.cpp` | Motion mock endpoint for tests/simulation. |
| `status/nav_status_writer.*` | JSON status serialization, including input-gate and driver-control readiness fields. |
| `status/nav_status_publisher.*` | Owns steady-clock cadence, live sampling, diagnostics assembly, and asynchronous snapshot submission outside the process main. |
| `status/control_loop_health.*` | Maintains a transport-free rolling distribution of completed control-loop ticks and reports whether the configured schedule has enough evidence and remains healthy. |
| `motion/control_loop_runtime_guard.*` | Converts mature unhealthy control-loop evidence into an explicitly recoverable motion hold; it owns no transport and performs no motion side effects itself. |
| `status/nav_status_endpoint_adapter.*` | Projects `EndpointState` into the compact, transport-free status view. |
| `status/inspection_status_file_writer.*` | Latest-wins background persistence for inspection run status; file I/O never runs on the motion loop. |
| `status/active_inspection_map_cache.*` | Background active-map polling with O(1), stale-aware, fail-closed snapshots for inspection admission and monitoring. |

`lingtu_nav_control teleop-stream` is the terminal/simulation diagnostic ingress.
It keeps one native command client alive, accepts latest `VX VY WZ` lines on
stdin, and drains queued input to the newest state. A 350 ms input-heartbeat
timeout sends an `OperatorMotion` zero, releases its motion authority, sends a
global `Stop`, and ends the stream; a fresh process is required before motion
can resume. `quit`, EOF, and error cleanup use the same fail-closed boundary.
It remains an intent client; it never publishes final `/nav/cmd_vel` directly.

The heavy planning/following logic should stay outside this endpoint shell.
Current split:

```text
nav_native_endpoint.cpp
  -> nav_dds_runtime: typed DDS boundary
  -> process lifecycle + control mode
  -> rolling_segment_lifecycle: ordered domain events/effects
     -> rolling_map_segment_executor: safe-prefix planning + revalidation
  -> src/nav/cpp/engine/nav_loop.cpp
     -> target selection, local planning, path following
```

`/nav/local_path` is an output/telemetry topic. `PathFollower` consumes the
in-memory local path within `NavLoop::tick()` before DDS publication.

A command acknowledgement only admits a request to planning. The request-correlated
`/nav/goal/status` lifecycle is authoritative:

- `Planning`: async global planning has begun.
- `PathActive`: a verified global path is active in `NavLoop`.
- `Failed`: planning or execution cannot continue; TARE can immediately reselect.
- `Reached` and `Cancelled`: terminal outcomes for the exact request ID.

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

`/nav/cmd_vel` publication is final command output. Python in-process velocity
arbitration and
legacy global-path publishers must not run as competing writers in the product
field endpoint.

OctoPlanner3D remains the default. FAR is selected only with
`LINGTU_NAV_GLOBAL_PLANNER=far` and consumes the validated active
`occupancy.npz`; unknown-space fallback remains disabled unless explicitly
enabled. The status snapshot always reports `global_planner` and `planner_map`
so Gateway readiness can reject deployment/profile drift.

OctoPlanner3D never hands a static-map-outside request to `LocalPlanner`. A target
outside the static map with exhausted endpoint snapping reports
`goal_outside_static_map`; an in-map target with no safe snap point reports
`goal_snap_exhausted`.

## Directed exploration

`lingtu_explore_dds` accepts a session-scoped directed target on the existing
exploration-control channel. The target is bound to the fresh rolling-map epoch,
has a bounded TTL, and is exposed with its revision in endpoint status.

It is deliberately a soft TARE preference: it changes scoring only among
frontiers that are already observed free and reachable in the current rolling
snapshot. Retargeting cancels a pending exploration goal, clears the queued
goal, and asks TARE to select again. TTL expiry, a map reset, stop, and a new
exploration session clear the preference.

It does not authorize a raw click target or a static-map-outside path in navd.
Normal click navigation retains the OctoPlanner3D static-boundary gate. Smooth
physical travel beyond the saved-map boundary is available only through the
TARE-specific rolling-map segment fallback: the correlated TARE goal must fail
with exactly `goal_outside_static_map`, then navd may accept a typed segment
bound to a fresh, live rolling-map identity. navd executes only a short,
observed-free and terrain-safe prefix, and revalidates that prefix before every
NavLoop tick. A stale grid, map-epoch change, unsafe cell, cancellation, or
generic navigation takeover stops the segment and reports a terminal status.

This does not remove the profile's saved-map localization requirement. It is a
safe extension outside the static planning artifact after localization is
available, not a map-free first-visit SLAM mode.

Completed split:

- goal admission, async planning completion, correlated lifecycle status, and
  rolling-to-generic handoff now live in `plan/goal_plan_controller.*`.
- typed-command validation, replay protection, dispatch, and ACK diagnostics now
  live in `motion/command_ingress_controller.*`.
- ordered fail-closed motion clearing, E-stop transitions, authority loss, and
  shutdown-zero confirmation now live in `motion/motion_stop_coordinator.*`.
- `OperatorMotion` claim/sample/release requests use the transport-free
  freshness, takeover, and fail-closed admission policy in
  `motion/teleop_admission_controller.*`; non-zero output still comes only from
  the later `NavLoop` safety-arbitration tick.
- direct/assisted teleop and autonomy `NavLoop` computation now live in separate
  transport-free tick controllers. The endpoint retains only DDS effects and
  task-lifecycle transitions.
- periodic navigation status cadence, sampling, and snapshot assembly now live
  in `status/nav_status_publisher.*`; `nav_native_endpoint.cpp` provides only a
  compact state projection and injected live-data samplers.
- inspection evidence intake, post-arrival settling, progress watchdog,
  active-map validation, two-phase goal/action dispatch, and status cadence now
  live in `inspection/inspection_runtime_controller.*`.
- inspection command validation, FIFO ACK replay, admission, Pause/Cancel
  zero-motion downgrade, and ACK publication feedback now live in the completed
  `InspectionCommandCoordinator` seam. The endpoint injects map, route,
  authority, motion-clear, status, and DDS ACK adapters.
- periodic inspection status writes and active-map reads now run on background
  workers. The 20 Hz motion loop only serializes the latest status and consumes
  an O(1), stale-aware map snapshot.
- ordered sensor/control projection, input-gate evaluation, epoch-local input
  recovery, and deferred live-obstacle snapshots now live in
  `input/nav_input_state_projector.*`. The process main retains DDS drain
  interleaving plus synchronous rolling, inspection, goal, and motion effects
  at epoch boundaries.
- rolling-segment effect ordering, failure-policy handling, and recursive
  feedback now live in `plan/rolling_segment_effect_coordinator.*`. The endpoint
  injects concrete authority, path, DDS publication, and motion-clear actions.

## Control-loop health

The endpoint keeps its default 20 Hz (50 ms period) motion/safety schedule.
`ControlLoopHealth` observes completed ticks without owning transport, using a
rolling 600-sample window and a 100-valid-sample warm-up. A warming snapshot is
reported as `ready=false`; it is useful for startup visibility but is not field
acceptance evidence.
Gateway admission fails closed when this object is absent or malformed. An
explicit `ready=false, reason=warming_up` snapshot is the only non-blocking
not-ready state; field acceptance still waits for a ready, healthy window.

With the default policy, a ready window is unhealthy when the deadline-miss
ratio exceeds 5%, p95 utilization exceeds 90%, or three consecutive ticks miss
their deadline. A deadline miss is `overrun_ms > 0`; utilization is completed
work divided by the configured period, where
`work_ms = max(0, loop_ms - sleep_ms)`.

The status snapshot exposes `ready`, `healthy`, `reason`, `period_ms`,
`window_samples`, `total_samples`, loop, work, and overrun distributions
(`mean`, `p50`, `p95`, `p99`, `max`), `deadline_misses`,
`deadline_miss_ratio`, current and maximum miss streaks, and p95 and maximum
utilization. The `loop_ms` distribution records actual cadence and jitter; it
is diagnostic evidence, while health blocking remains based on work utilization
and overruns so ordinary scheduling jitter is not promoted into a new policy.

The 10 Hz LiDAR cadence does not by itself justify changing the control-loop
rate: keep 20 Hz unless ready, field-recorded distributions show a measured need
and sufficient headroom.

`ControlLoopRuntimeGuard` turns mature unhealthy evidence into a runtime motion
hold. A single mature unhealthy sample enters a suspect state only. Two
consecutive mature unhealthy samples latch the guard, while
`consecutive_deadline_misses` latches immediately. The first latch requests one
motion clear; after that the endpoint keeps zero motion fresh instead of
reissuing clear-motion on every tick.

Recovery is explicit. The guard requires a stable healthy confirmation window
derived from `tick_hz` (20 samples at the default 20 Hz, about one second) before
it permits resume. It does not auto-resume. The normal resume command still goes
through the existing zero-confirmation path; a failed resume completion re-latches
the guard. This hold is not the persisted software E-stop under `/var/lib/lingtu`,
and restarting the endpoint does not make it equivalent to ClearEstop.

While the guard is latched or recovered, the endpoint marks operator resume as
required and all command admissions remain gated by the held control authority.
New goals, global paths, teleop takeover, inspection actions, and TARE rolling
handoffs must not bypass the explicit resume requirement.


Endpoint shell completion seam:

- `nav_native_endpoint.cpp` intentionally retains process lifecycle, globally
  ordered DDS drains, explicit effect adapters, the 20 Hz motion/safety
  schedule, live diagnostics projection, and shutdown-zero confirmation;
- do not add shallow input-router or output-bridge modules solely to reduce line
  count. Extract again only when a second adapter or measured defect proves a
  real seam;
- keep the default 20 Hz loop independent from the 10 Hz LiDAR update rate and
  raise it only from measured latency/overrun evidence;
- local software completion does not replace the
  [S100P field acceptance checklist](../../../../docs/07-testing/field-runs/NATIVE_ENDPOINT_REFACTOR_FIELD_ACCEPTANCE.md).
