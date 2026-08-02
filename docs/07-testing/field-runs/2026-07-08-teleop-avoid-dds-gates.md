# 2026-07-08 Teleop Avoid Typed DDS Gate Smoke

## Scope

Validate the teleop_avoid command path without Python DDS:

```text
operator velocity request
-> local UDP 127.0.0.1:7520
-> C++ lingtu_teleop_bridge
-> DDS /nav/teleop_cmd_vel
-> C++ nav_endpoint
-> stale / pose / obstacle / traversability / limit gates
-> DDS /nav/cmd_vel
```

This was a smoke test on `sunrise@192.168.66.13` using an isolated DDS domain
and the C++ motion mock. It did not replace the live `/opt/lingtu/current`
service binaries.

## Build

Temporary source tree:

```text
/tmp/lingtu_teleop_check
```

Built targets:

```text
lingtu_nav_native_endpoint
lingtu_nav_control
lingtu_motion_mock_dds
lingtu_teleop_bridge
```

Result:

```text
build rc 0
CycloneDDS dev: 0.8.2-5
Eigen3: 3.4.0
CMake: 3.22.1
```

Warnings were pre-existing unused-parameter / unused-variable warnings in
OctoPlanner3D and local planner headers.

## Results

| Gate | DDS domain | Input | Expected | Observed |
| --- | ---: | --- | --- | --- |
| Accepted teleop | 13 | `/nav/teleop_cmd_vel` vx=0.2, wz=0.1 at 20 Hz | final `/nav/cmd_vel` equals request | `reason=accepted`, output vx=0.2, wz=0.1 |
| Stale timeout | 13 | stop teleop stream | final stop | `reason=stale`, output zero |
| Obstacle stop | 14 | body-frame point at x=0.30m, height=0.40m | final stop | `reason=obstacle_stop`, obstacle_distance=0.30 |
| Traversability stop | 15 | `/nav/traversability` cost=100 | final stop | `reason=terrain_stop`, traversability_cost=100 |
| Limit clamp | 16 | teleop request vx=2.0 | clamp to max speed | `reason=limited`, output vx=0.4 |
| Gateway bridge shape | 17 | UDP `0.2 0 0.1` at 20 Hz to `lingtu_teleop_bridge` | final `/nav/cmd_vel` equals request | `reason=accepted`, output vx=0.2, wz=0.1, `teleop_cmd=39` |

Motion mock evidence:

```text
accepted: cmd_vel received, pose advanced
obstacle_stop: cmd_vel received as zero, pose stayed at origin
terrain_stop: cmd_vel received as zero, pose stayed at origin
limited: cmd_vel received as vx=0.4, pose advanced
```

## Notes

> Superseded behavior: this field run validated the earlier stop-only
> `teleop_avoid` implementation. Current builds optionally route translational
> operator intent through native LocalPlanner and PathFollower before the same
> final safety gate. Keep the evidence below as historical proof of the DDS
> gates, not as the current planning dataflow contract.

- `/nav/teleop_cmd_vel` remains the typed DDS operator velocity request topic.
- `/nav/cmd_vel` remains the final post-safety command and must stay single-writer.
- `nav_endpoint` status names are intentionally shorter than the binary name.
- Field Gateway should use `LINGTU_TELEOP_BRIDGE_ADDR=127.0.0.1:7520`.
- `cyclonedds-python` is not required for the field teleop path. The environment
  switch used during this historical run has since been retired; current
  Products compile the command boundary into `command_output_mode`.
- At the time of this run, `teleop_avoid` did not invoke OctoPlanner3D or
  PathFollower. Current assisted teleop still does not invoke OctoPlanner3D, but
  it does use native LocalPlanner and PathFollower for short-horizon detours.

## Remaining Field Check

The gate logic is verified in an isolated DDS domain. Before enabling robot
motion, run the same profile against the live field domain with low speed limits
and actuator output still guarded by the normal field procedure.
