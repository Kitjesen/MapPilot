# Native navigation endpoints

This directory contains the native process boundaries for navigation. Endpoint
code owns DDS I/O, process composition, readiness, command admission, final
motion authority, status publication, and shutdown. Planning and tracking
algorithms remain in `planning/`, `navigation/`, and `tracking/`.

## Processes

```text
SLAM / LiDAR / Mapd / Gateway / Driver
                 |
                 | typed DDS
                 v
              navd
        +--------+---------+
        |                  |
 global Planner       Route Executor
                           |
                    LocalPlanner -> Follower
                           |
                      MotionCandidate
                           |
                 FinalControl + Safety
                           |
                 typed final velocity
                           v
                         Driver
```

Algorithms call each other directly inside `navd`. DDS is the cross-process
data plane; it does not choose a Product, planner backend, or recovery policy.

The deployed executable names remain:

- `navd`: goal navigation and assisted teleoperation.
- `lingtu_explore_dds`: exploration lifecycle and route requests.
- `lingtu_traversability_dds`: terrain and traversability production.
- `lingtu_nav_client`: native command/status client.

## Directory ownership

| Path | Owns |
| --- | --- |
| `nav/main.cpp` | `navd` composition and process lifetime. |
| `nav/dds/` | Generated-message decode/encode, bounded reads, owning batches, and publication. |
| `nav/input/` | Ordinary C++ sensor state, frame math, freshness, active-map gates, and obstacle projection. |
| `nav/command/` | Ordered command admission, identity, cancellation, and terminal events. |
| `nav/control/` | Teleop/autonomy candidates, authority, smoothing, and final arbitration. |
| `nav/runtime/` | The 20 Hz loop and endpoint-only goal, inspection, rolling, and configuration lifecycle. |
| `nav/safety/` | E-stop, geofence, hard command limits, and confirmed zero motion. |
| `nav/status/` | Typed lifecycle/status projection and asynchronous persistence. |
| `src/explore/cpp/endpoint/` | Explore process entry and lifecycle source, composed here as `lingtu_explore_dds`. |
| `traversability/main.cpp` | Traversability process entry; supporting files own terrain-grid production. |
| `tools/` | Diagnostics only; never part of the robot command path. |

## navd data boundary

`nav/dds/Dds` exposes three operations to the runtime loop:

```cpp
SensorBatch takeSensors(double now);
CommandBatch takeCommands(double now);
PublishReceipt publish(const OutputEvent& output);
```

`SensorBatch` and `CommandBatch` own their data. DDS loaned samples are
copied before return. Frame/epoch/freshness semantics belong to `nav/input/`,
not to the transport adapter. The loop therefore works with ordinary C++ values
and never calls per-topic `drainXxx` or `writeXxx` methods.

Inputs include pose/TF, obstacle and collision data, traversability, driver and
localization state, goals, operator motion, cancellation, and lifecycle events.
Outputs include local-path telemetry, final velocity, acknowledgements, goal
state, inspection/exploration state, warnings, and stop confirmation.

## Boundaries

- `src/message/idl` remains the only wire contract.
- DDS IDL, topic names, QoS, C client ABI, Product values, and executable target
  names are not defined or changed here.
- Generated `lingtu_dds_*` values may appear only in DDS adapters, standalone
  endpoint process entries, clients/tools, and DDS-focused tests.
- `Executor`, `LocalPlanner`, and `Follower` do not include DDS headers.
- `/nav/local_path` is telemetry. Follower consumes the in-memory
  `LocalPlan`; it does not subscribe to its own published path.
- `/nav/cmd_vel` is the checked final output. The Host must not publish a
  competing robot velocity.

Command ACK means admission, not physical completion. Request-correlated goal
status remains authoritative for planning, active execution, cancellation,
failure, and arrival.
