# Transport Backends

This package owns inter-module communication backends.
Modules must not depend on backend-specific message types. They publish normal
Python objects through `In[T]` and `Out[T]`; adapters serialize at the boundary.

## Communication Plan

Use the smallest delivery mode that matches the seam:

| Seam | Default |
| --- | --- |
| Same-process Module graph | `local` callback / `LocalTransport` |
| LiDAR, IMU, and other mature sensor buses across processes | typed CycloneDDS adapter |
| Same-host high-bandwidth image IPC | POSIX SHM rings with typed status/CameraInfo |
| Thunder endpoint commands, paths, status, inspection, and replay | typed DDS endpoint contract |
| ROS2 compatibility windows | explicit DDS/ROS2 adapter |

DDS is suitable for sensor streams when the topic schema and QoS are explicit.
Do not use the generic DDS transport adapter as the product sensor or navigation
wire; it rejects registered product topics and only remains for unregistered
test/debug topics. Use typed IDL adapters such as the Livox, IMU, and
native C++ navigation endpoint instead.

Protobuf is a wire-format option, not a transport. Add it only when a real
external boundary needs generated language bindings. For point clouds, prefer a
binary payload plus a versioned header/schema over protobuf `repeated` points.

## Backends

| Backend | Scope | Notes |
| --- | --- | --- |
| `local` | Same process | Default for local/development Profiles and tests. |
| `shm` | Same host | High-bandwidth image/point-cloud IPC. |
| `dds` | Same host / LAN | CycloneDDS backend. Use typed adapters for product streams; generic transport is compatibility/testing only and rejects registered product topics. |

## Module Wire Delivery

`Blueprint.wire(..., delivery=...)` selects how one Module `Out` port reaches
one Module `In` port. This is a local graph delivery choice, not the system bus
architecture.

Supported delivery values:

- `None` or `callback`: direct in-process callback, zero-copy.
- `local`: in-process `LocalTransport` bus, mainly for tests and local replay.
- `shm`: same-host shared-memory adapter for selected high-bandwidth links.
- `dds`: generic DDS adapter for explicit compatibility/debug links.

Product sensor, SLAM, and navigation process boundaries should not be modeled
by sprinkling `delivery="dds"` across ordinary Module wires. Use typed endpoint
contracts under `src/runtime/endpoints/dds/` and `src/message/dds.py` for those
boundaries.

## Typed DDS Endpoint

Product Thunder field communication uses the typed DDS contract in
`src/runtime/endpoints/dds/contracts.py`. The default contract is
`thunder_dds_v1`, backed by the topic/type registry in
`src/message/dds.py` and the native IDL under `src/message/idl/`. Registered
product topics bind to LingTu-owned DDS types for LiDAR, IMU, odometry,
point-cloud, path, final velocity, inspection command/status/evidence, and
camera metadata. ROS-compatible topic shapes are adapter windows, not the
product ownership model.

The default field graph uses:

- Native LiDAR, SLAM, traversability, and navigation endpoint processes publish
  and consume typed DDS topics directly.
- Camera color/depth image payloads use SHM rings by default; optional image
  DDS exists for diagnostics/compatibility only.
- C++ `lingtu-nav-dds` owns navigation goal/path/status and final
  `/nav/cmd_vel` publication. The old Python nav DDS Module adapters were
  removed to prevent field double writers.
- C++ `lingtu_driver` is the unique motor-command sink. It subscribes
  `rt/nav/cmd_vel`, holds the Brainstem `lingtu-driver` lease, and calls
  `WalkChecked`.

The Python endpoint runner in `src/runtime/endpoints/dds/endpoint_runner.py`
remains a replay/diagnostic/smoke surface. It is not the product Brainstem
command sink and must not be used to create a second `/nav/cmd_vel` writer in a
field deployment. Its field systemd unit, installer, and deployment wrapper are
physically removed; no Product RunPlan can start it.

For replay or endpoint-smoke diagnostics, the Python runner can publish
contract-bound JSONL sensor/localization records. Do not run this as the field
motor command sink:

```bash
LINGTU_ENDPOINT_JSONL_PATH=/data/thunder/localization.jsonl \
  PYTHONPATH=src python -m runtime.endpoints.dds.endpoint_runner --source jsonl --once
```

Validate the Thunder field deployment boundary:

```bash
python tools/validate/validate_real_deployment.py
```

## Endpoint JSON Replay Payloads

The shared JSON envelope in `json_codec.py` remains available for explicit
replay adapters and tests, and carries:

- `format`: currently `lingtu.transport.json.v1`
- `type`: the Python `runtime.msgs.*` type name when available
- `payload`: the message `to_dict()` payload, or a plain JSON value

For `env=real`, the command output mode is `endpoint_only`: LingTu does
not include an in-process `ThunderDriver` in the default field graph. Real
motor actuation is the native C++ `lingtu_driver` process, not a Python DDS
command sink. If a replay endpoint or diagnostic source uses Python DDS, missing
`cyclonedds-python` must fail closed and must not affect the native driver
service.

Source plugins live under `src/runtime/adapters/endpoint_sources/`. They are
for replay, simulation, and bounded diagnostics outside the normal Module graph.
The built-in JSONL source reads normalized JSONL records from a file or
external process stdout, then publishes them as contract-bound endpoint
messages. The built-in smoke source provides a data-flow check:

```bash
PYTHONPATH=src python -m runtime.endpoints.dds.endpoint_runner --transport local --source smoke --once --json
```

## Boundary Rule

DDS and SHM details stay inside `runtime.transport` or explicit compatibility
adapters. Product modules and `runtime.msgs` should not import `cyclonedds` or
ROS message packages directly.

Keep normal Module wires in-process. Use typed endpoints for cross-process
product links:

```python
bp.wire("SlamAdapterModule", "map_cloud", "nav.terrain", "map_cloud")
bp.wire("GatewayModule", "goal_pose", "nav.mission", "goal_pose")
```
