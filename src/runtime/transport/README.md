# Transport Backends

This package owns inter-module and cross-process communication backends.
Modules must not depend on backend-specific message types. They publish normal
Python objects through `In[T]` and `Out[T]`; adapters serialize at the boundary.

## Communication Plan

Use the smallest transport that matches the seam:

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
| `lcm` | Cross process / same LAN | Optional lightweight IPC. Requires the external `lcm` package only when selected. |
| `dds` | Same host / LAN | CycloneDDS backend. Use typed adapters for product streams; generic transport is compatibility/testing only and rejects registered product topics. |
| `dual` | Migration only | SHM plus DDS during compatibility transitions. |

## Typed DDS Endpoint

Product Thunder field communication uses the typed DDS contract in
`src/runtime/adapters/dds/contracts.py`. The default contract is
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
`src/runtime/adapters/dds/endpoint_service.py` through the runnable
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

## Legacy LCM Payloads

`lcm` uses `TransportAdapter` with the versioned JSON codec in
`json_codec.py`. The envelope is readable outside Python and carries:

- `format`: currently `lingtu.transport.json.v1`
- `type`: the Python `runtime.msgs.*` type name when available
- `payload`: the message `to_dict()` payload, or a plain JSON value

Do not use pickle for LCM endpoint traffic. High-rate binary payloads such as
large point clouds should get explicit schema-specific encoders instead of
falling back to generic Python serialization.

Legacy LCM endpoint contracts live in `src/runtime/adapters/lcm/contracts.py`.
The compatibility contract is `thunder_field_lcm_v1`; it names
product-neutral schemas and LCM channels for odometry, point clouds,
localization health, paths, active waypoints, command velocity, goals, cancels,
and semantic instructions.
Schema-aware endpoint payloads live in
`src/runtime/adapters/lcm/endpoint_codec.py`, including binary-safe point cloud and IMU
envelopes. The endpoint localization input adapter is
`src/runtime/adapters/lcm/localization_adapter.py`. Navigation output leaves
through `nav.out`, implemented by `src/runtime/adapters/lcm/nav_output.py`;
it publishes global paths, local paths, active waypoints, and the final
`VelocityMux` command velocity after LingTu safety arbitration. Navigation
input enters through `nav.in`, implemented by
`src/runtime/adapters/lcm/nav_input.py`; it receives goals, mission cancels,
and semantic instructions from the endpoint contract.
For `thunder_field`, the command output mode is `endpoint_only`: LingTu does
not include an in-process `ThunderDriver` in the default field graph, and the
endpoint source owns translation from DDS command velocity to robot hardware.
This Python command sink currently needs `cyclonedds-python` if enabled, but it
is not a main-path algorithm dependency; missing Python DDS must fail closed
rather than restart-loop the robot services. Removing `cyclonedds-python` from
real motor actuation requires a dedicated C++ Thunder control sink that
subscribes `rt/nav/cmd_vel`; do not fold that hardware role into the planner
endpoint.

Source plugins implement `src/runtime/adapters/lcm/source.py`. Deployment-specific
motion command sinks are passed as `module:factory` sources outside the LingTu
module graph. The runner accepts comma-separated source names, so control,
localization publishing, sensor publishing, and replay can remain
separate endpoint plugins while sharing one supervised endpoint process. The
built-in `src/runtime/adapters/lcm/sources/jsonl.py` source reads normalized JSONL
records from a file or external process stdout, then publishes them as
contract-bound endpoint messages. Use it for no-ROS replay and for external
SLAM/localization sidecars that are not ROS nodes. The built-in
`src/runtime/adapters/lcm/sources/smoke.py` source provides a data-flow check:

```bash
python scripts/deploy/thunder/run_dds_endpoint_service.py --transport local --source smoke --once --json
```

Validate the JSONL provider before using it as an endpoint source:

```bash
python tools/validate/validate_lcm_jsonl_feed.py /data/thunder/localization.jsonl --require-field-inputs
```

## Boundary Rule

LCM, DDS, and SHM details stay inside `runtime.transport` or explicit compatibility
adapters. Product modules and `runtime.msgs` should not import `lcm`,
`cyclonedds`, or ROS message packages directly.

Use explicit wires for cross-process links:

```python
bp.wire("SlamBridgeModule", "map_cloud", "nav.terrain", "map_cloud", transport="shm")
bp.wire("GatewayModule", "goal_pose", "nav.mission", "goal_pose", transport="lcm")
```
