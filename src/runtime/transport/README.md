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
| Same-host high-bandwidth image or point-cloud IPC | `shm` |
| Thunder endpoint commands, paths, status, and replay | typed DDS endpoint contract |
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
| `local` | Same process | Default for lightweight product profiles and tests. |
| `shm` | Same host | High-bandwidth image/point-cloud IPC. |
| `dds` | Same host / LAN | CycloneDDS backend. Use typed adapters for product streams; generic transport is compatibility/testing only and rejects registered product topics. |

## Module Wire Delivery

`Blueprint.wire(..., delivery=...)` selects how one Module `Out` port reaches
one Module `In` port. This is a local graph delivery choice, not the system bus
architecture. Existing `transport=` arguments are accepted as a compatibility
alias for framework-level tests and explicit adapter experiments.

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
`thunder_field_dds_v1`, backed by the topic/type registry in
`src/message/dds.py`. Registered product topics bind to concrete IDL/C++ message
types such as Livox `CustomMsg`, `sensor_msgs/Imu`, `nav_msgs/Odometry`,
`sensor_msgs/PointCloud2`, `nav_msgs/Path`, and `geometry_msgs/TwistStamped`.

The default field graph uses:

- `DDSLocalizationAdapterModule` for `/lidar/raw_frame`, `/imu/raw`, and
  `/slam/*` localization/map streams.
- C++ `lingtu-nav-dds` for navigation goal/path/cmd_vel DDS. The old Python
  nav DDS Module adapters were removed to prevent field double writers.

External Thunder endpoint processes should use
`src/runtime/endpoints/dds/endpoint_service.py` through the runnable
`scripts/deploy/thunder/run_dds_endpoint_service.py` entrypoint only for the
temporary Python Brainstem command sink. Product navigation planning uses the
C++ `lingtu-nav-dds` service for goal/path/cmd_vel DDS.

For the default endpoint process, use the product source group. It always
includes the Brainstem command sink and adds JSONL sensor/localization input
when a JSONL provider is configured:

```bash
LINGTU_ENDPOINT_JSONL_PATH=/data/thunder/localization.jsonl \
  python scripts/deploy/thunder/run_dds_endpoint_service.py --source thunder_field
```

Validate the default Thunder endpoint deployment boundary:

```bash
python tools/validate/validate_thunder_field_deployment.py
```

## Endpoint JSON Replay Payloads

The shared JSON envelope in `json_codec.py` remains available for explicit
replay adapters and tests, and carries:

- `format`: currently `lingtu.transport.json.v1`
- `type`: the Python `runtime.msgs.*` type name when available
- `payload`: the message `to_dict()` payload, or a plain JSON value

For `thunder_field`, the command output mode is `endpoint_only`: LingTu does
not include an in-process `ThunderDriver` in the default field graph, and the
endpoint source owns translation from DDS command velocity to robot hardware.
This Python command sink currently needs `cyclonedds-python` if enabled, but it
is not a main-path algorithm dependency; missing Python DDS must fail closed
rather than restart-loop the robot services. Removing `cyclonedds-python` from
real motor actuation requires a dedicated C++ Thunder control sink that
subscribes `rt/nav/cmd_vel`; do not fold that hardware role into the planner
endpoint.

Source plugins live under `src/runtime/adapters/endpoint_sources/`.
Deployment-specific motion command sinks are passed as `module:factory`
sources outside the LingTu module graph. The runner accepts comma-separated
source names, so control, localization publishing, sensor publishing, and
replay can remain separate endpoint plugins while sharing one supervised
endpoint process. The built-in JSONL source reads normalized JSONL records from
a file or external process stdout, then publishes them as contract-bound
endpoint messages. The built-in smoke source provides a data-flow check:

```bash
python scripts/deploy/thunder/run_dds_endpoint_service.py --transport local --source smoke --once --json
```

## Boundary Rule

DDS and SHM details stay inside `runtime.transport` or explicit compatibility
adapters. Product modules and `runtime.msgs` should not import `cyclonedds` or
ROS message packages directly.

Keep normal Module wires in-process. Use typed endpoints for cross-process
product links:

```python
bp.wire("SlamBridgeModule", "map_cloud", "nav.terrain", "map_cloud")
bp.wire("GatewayModule", "goal_pose", "nav.mission", "goal_pose")
```
