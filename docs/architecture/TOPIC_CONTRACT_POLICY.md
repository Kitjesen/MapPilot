# Topic Contract Policy

Status: current guardrail
Audience: runtime, adapter, diagnostics, and topic-contract maintainers
Replaced by: not replaced

This document defines how LingTu owns product/runtime topic names. The goal is
not to make topic usage harder. The goal is to prevent ordinary modules from
silently inventing or drifting product topics.

## Rule

`runtime.runtime_interface.TOPICS` is the Python source of truth for canonical
runtime topics.

Ordinary Python modules must not hardcode canonical topic strings such as:

```text
/lidar/raw_frame
/imu/raw
/slam/odometry
/slam/map_cloud
/nav/cmd_vel
/nav/global_path
/exploration/start
/camera/color/image_raw
/tf
/tf_static
```

Use the contract instead:

```python
from runtime.runtime_interface import TOPICS

scan_topic = TOPICS.lidar_scan
imu_topic = TOPICS.imu
odom_topic = TOPICS.odometry
cmd_topic = TOPICS.cmd_vel
```

## Why This Should Not Break Runtime Behavior

This guard is a static source rule. It does not change published DDS names,
Module ports, message schemas, or runtime wiring by itself.

The safe migration pattern is:

```python
# Bad in ordinary module code:
topic = "/slam/odometry"

# Good:
topic = TOPICS.odometry
```

Both evaluate to the same string. The difference is that future renames,
schema checks, frame checks, DDS bindings, and docs can now use one source.

The guard prevents three failure modes:

- one module publishes `/nav/cmd_vel` while another subscribes `/cmd_vel`;
- a topic string changes in DDS but Python status/debug code keeps the old name;
- LiDAR/IMU/SLAM topic names drift between `runtime_interface`, DDS IDL, route
  contracts, and Gateway diagnostics.

## Allowed Hardcoded Topic Layers

Only boundary and contract code may contain literal canonical topic strings.

Allowed layers:

```text
src/runtime/runtime_interface.py
src/message/
src/runtime/route_contract/
src/runtime/endpoints/
src/runtime/adapters/
src/runtime/transport/
src/runtime/tf/
src/localization/adapters/
src/localization/launch/
src/perception/adapters/
src/nav/adapters/
src/drivers/adapters/
src/drivers/real/lidar/
```

Reason:

- contract files define names;
- message files bind names to DDS/IDL types;
- adapters translate external protocols into LingTu topics;
- endpoint and transport code touches process/network boundaries;
- `runtime/tf` owns the ROS-free `/tf` and `/tf_static` runtime bus surface;
- `drivers/real/lidar` owns the physical Livox boundary.

Ordinary mission, planner, map, service, perception, decision, Gateway business
logic, and diagnostics should use `TOPICS`.

## Current Static Guard

The guard lives in:

```text
src/runtime/tests/test_message_contract_location.py
```

It scans ordinary Python sources under:

```text
cli
src/decision
src/drivers
src/gateway
src/localization
src/memory
src/nav
src/perception
src/runtime
```

It skips tests, Python cache directories, build output, vendored code, and
known external driver packages.

It builds the forbidden topic set from `runtime.runtime_interface.TOPICS`, plus
`/tf` and `/tf_static`. This means new canonical topics are automatically
covered after they are added to `TOPICS`.

## Fixing A Failure

If the test reports:

```text
src/nav/foo.py:10 hardcodes '/nav/cmd_vel'
```

Fix ordinary module code by importing the canonical constant:

```python
from runtime.runtime_interface import TOPICS

topic = TOPICS.cmd_vel
```

If the file is genuinely a protocol boundary, move the code into an adapter,
endpoint, route-contract, message, or transport package. Do not silence the test
with an inline ignore unless the code has been moved to a documented boundary.

## LiDAR Status

The Python contract guard now covers the LiDAR and IMU canonical topics:

```text
/lidar/raw_frame
/lidar/raw_packet
/imu/raw
```

That means ordinary Python modules cannot hardcode these names. They must use:

```python
TOPICS.lidar_scan
TOPICS.raw_lidar_packet
TOPICS.imu
```

This is only a source-contract guarantee. It does not prove real MID-360
hardware is currently producing data.

Real LiDAR health still needs runtime evidence:

- Livox SDK2 initializes successfully;
- MID-360 device handle is discovered;
- point-cloud callback receives stable frames;
- IMU callback receives stable samples;
- endpoint publishes `/lidar/raw_frame` and `/imu/raw`;
- downstream SLAM publishes `/slam/odometry` and `/slam/map_cloud`;
- Gateway or field diagnostics show frame rate, timestamp, dropped-frame, and
  DDS publication evidence.

## Verified Progress

Implemented:

- Python canonical topic guard based on `TOPICS`;
- `/tf` and `/tf_static` included in the guarded set;
- compiled Product and lifecycle consumers no longer keep a second topic list;
- runtime-graph validator changed to use `TOPICS`;
- runtime-audit wording changed to use `TOPICS`;
- failure message now explains how to fix violations.

Focused verification:

```powershell
$env:PYTHONPATH='src'; python -m pytest src/runtime/tests/test_message_contract_location.py src/runtime/tests/test_runtime_graph_contract.py src/runtime/tests/test_route_contract.py -q
```

Latest result:

```text
31 passed
```

## Remaining Work

Next guards should cover:

- C++ endpoint topic literals: require `src/message/cpp/topics.hpp` or an
  equivalent generated constant source;
- `config/runtime_graph/*.yaml`: require all topics to match
  `runtime_interface.TOPICS` and route-contract declarations;
- adapter literal audit: adapter packages may hardcode topics, but only topics
  declared in `TOPICS`, route contracts, or external legacy alias tables;
- LiDAR doctor command: report SDK/device/DDS/SLAM evidence instead of relying
  on source-level guarantees.
