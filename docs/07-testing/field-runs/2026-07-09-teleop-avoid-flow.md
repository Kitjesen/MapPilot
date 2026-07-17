# 2026-07-09 Teleop Avoid Flow Acceptance

## Purpose

Validate the teleop avoidance flow beyond a direct DDS command tool:

```text
operator UDP packet
  -> lingtu_teleop_bridge
  -> DDS /nav/teleop_cmd_vel
  -> lingtu_nav_native_endpoint
  -> obstacle/traversability command arbitration
  -> DDS /nav/cmd_vel
  -> downstream command consumer
```

This test still used isolated DDS domains and a motion mock, not the real motor
consumer.

## Safety Boundary

- Test DDS domains: `63`, `64`, `65`
- Production DDS domain: `0`
- Production remained no-motion:

```text
production domain_id=0
production publish_cmd_vel=false
production cmd_vel_published=0
```

## Setup

- `lingtu_teleop_bridge`: UDP to `/nav/teleop_cmd_vel`
- `lingtu_nav_native_endpoint`: C++ command arbiter
- `lingtu_motion_mock_dds`: downstream `/nav/cmd_vel` consumer
- `lingtu_nav_control cloud`: injected `/slam/registered_cloud` obstacle point

Endpoint flags:

```text
--publish-cmd-vel 1
--check-obstacle 1
--use-traversability-cost 0
```

Command input:

```text
vx=0.18 vy=0.00 wz=0.00
udp_rate=20Hz
udp_samples=40
```

## Cases

### No Obstacle

```text
reason=accepted
request.vx=0.18
output.vx=0.18
teleop_cmd=19
cmd_vel_published=19
motion_mock_received_final=50
final_pose_x=0.413573
```

### Slow Obstacle

Injected registered-cloud point:

```text
x=0.90 y=0.00 z=0.30 height=0.50
```

Result:

```text
reason=obstacle_slow
obstacle_distance_m=0.84283
request.vx=0.18
output.vx=0.063
teleop_cmd=19
cmd_vel_published=19
teleop_slows=19
motion_mock_received_final=34
final_pose_x=0.105395
```

### Stop Obstacle

Injected registered-cloud point:

```text
x=0.35 y=0.00 z=0.30 height=0.50
```

Result:

```text
reason=obstacle_stop
obstacle_distance_m=0.35
request.vx=0.18
output.vx=0.0
teleop_cmd=18
cmd_vel_published=18
teleop_stops=18
motion_mock_received_final=33
final_pose_x=0
```

## Conclusion

The teleop avoidance flow works through the bridge and native endpoint:

- remote-style UDP command reaches `/nav/teleop_cmd_vel`;
- C++ endpoint accepts, slows, or stops based on obstacle context;
- final `/nav/cmd_vel` is published and received by a downstream consumer.

The remaining product gate is a short controlled motor-side test where
production `publish_cmd_vel` is temporarily enabled and cancel/stop latency is
measured against the real driver.
