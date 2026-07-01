# Transport Backends

This package owns inter-module and cross-process communication backends.
Modules must not depend on backend-specific message types. They publish normal
Python objects through `In[T]` and `Out[T]`; adapters serialize at the boundary.

## Backends

| Backend | Scope | Notes |
| --- | --- | --- |
| `local` | Same process | Default for lightweight product profiles and tests. |
| `shm` | Same host | High-bandwidth image/point-cloud IPC. |
| `lcm` | Cross process / same LAN | Optional lightweight IPC. Requires the external `lcm` package only when selected. |
| `dds` | Compatibility | CycloneDDS backend for existing ROS/DDS bridge windows. |
| `dual` | Migration only | SHM plus DDS during compatibility transitions. |

## LCM Payloads

`lcm` uses `TransportAdapter` with the versioned JSON codec in
`json_codec.py`. The envelope is readable outside Python and carries:

- `format`: currently `lingtu.transport.json.v1`
- `type`: the Python `core.msgs.*` type name when available
- `payload`: the message `to_dict()` payload, or a plain JSON value

Do not use pickle for LCM endpoint traffic. High-rate binary payloads such as
large point clouds should get explicit schema-specific encoders instead of
falling back to generic Python serialization.

Field endpoint contracts live in `src/compat/lcm/contracts.py`. The Thunder
field contract is `thunder_field_lcm_v1`; it names product-neutral schemas and
LCM channels for odometry, point clouds, localization health, paths, active
waypoints, command velocity, goals, cancels, and semantic instructions.
Schema-aware endpoint payloads live in
`src/compat/lcm/endpoint_codec.py`, including binary-safe point cloud and IMU
envelopes. The field localization ingress adapter is
`src/compat/lcm/localization_adapter.py`. The field navigation egress adapter is
`src/compat/lcm/path_command_adapter.py`; it publishes global paths, local
paths, active waypoints, and the final `CmdVelMux` command velocity after
LingTu safety arbitration. The field navigation command ingress adapter is
`src/compat/lcm/navigation_command_adapter.py`; it receives goals, mission
cancels, and semantic instructions from the endpoint contract.
For `thunder_field`, the command output mode is `endpoint_only`: LingTu does
not include an in-process `ThunderDriver` in the default field graph, and the
endpoint source owns translation from LCM command velocity to robot hardware.

External Thunder endpoint processes should use
`src/compat/lcm/endpoint_service.py` through the runnable
`src/compat/lcm/endpoint_runner.py` entrypoint. That service publishes
normalized sensor/localization streams into LingTu and consumes LingTu
path/cmd_vel outputs without importing the `lcm` package directly in product
or hardware code. Deployment uses
`scripts/deploy/thunder/run_lcm_endpoint_service.py` and the
`lingtu-thunder-lcm-endpoint.service` unit for the field endpoint process.
Source plugins implement `src/compat/lcm/source.py`; the built-in
`src/compat/lcm/sources/thunder_brainstem.py` source is the no-ROS Brainstem
motion command sink for Thunder field deployments. It consumes `/nav/cmd_vel`
from the endpoint contract and forwards it to hardware outside the LingTu
module graph. The runner accepts comma-separated source names, so Brainstem
control, localization publishing, sensor publishing, and replay can remain
separate endpoint plugins while sharing one supervised endpoint process. The
built-in `src/compat/lcm/sources/jsonl.py` source reads normalized JSONL
records from a file or external process stdout, then publishes them as
contract-bound endpoint messages. Use it for no-ROS replay and for external
SLAM/localization sidecars that are not ROS nodes. The built-in
`src/compat/lcm/sources/smoke.py` source provides a no-ROS data-flow check:

```bash
python scripts/deploy/thunder/run_lcm_endpoint_service.py --transport local --source smoke --once --json
```

For the default field endpoint process, use the product source group. It
always includes the Brainstem command sink and adds JSONL sensor/localization
ingress when a JSONL provider is configured:

```bash
LINGTU_ENDPOINT_JSONL_PATH=/data/thunder/localization.jsonl \
  python scripts/deploy/thunder/run_lcm_endpoint_service.py --source thunder_field
```

Validate the JSONL provider before using it as a field endpoint source:

```bash
python tools/validate/validate_lcm_jsonl_feed.py /data/thunder/localization.jsonl --require-field-inputs
```

Validate the default Thunder field deployment boundary:

```bash
python tools/validate/validate_thunder_field_deployment.py
```

## Boundary Rule

LCM, DDS, and SHM details stay inside `core.transport` or explicit compatibility
adapters. Product modules and `core.msgs` should not import `lcm`,
`cyclonedds`, or ROS message packages directly.

Use explicit wires for cross-process links:

```python
bp.wire("SLAMModule", "map_cloud", "TerrainModule", "map_cloud", transport="shm")
bp.wire("GatewayModule", "goal_pose", "NavigationModule", "goal_pose", transport="lcm")
```
