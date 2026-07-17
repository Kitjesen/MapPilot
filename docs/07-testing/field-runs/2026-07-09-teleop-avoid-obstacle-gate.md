# 2026-07-09 Teleop Avoid Obstacle Gate Acceptance

## Purpose

Verify that `teleop_avoid` command arbitration behaves correctly before real
motor output:

```text
/nav/teleop_cmd_vel
  + /slam/odometry
  + /slam/registered_cloud
  -> lingtu_nav_native_endpoint
  -> /nav/cmd_vel
```

This test checks the obstacle safety gate only. It does not claim autonomous
path planning or global rerouting.

## Safety Boundary

- Test ran on sunrise in isolated DDS domains `60`, `61`, and `62`.
- Production DDS domain `0` stayed in no-motion mode:

```text
production domain_id=0
production publish_cmd_vel=false
production cmd_vel_published=0
```

## Setup

- Endpoint: `lingtu_nav_native_endpoint`
- Downstream command consumer: `lingtu_motion_mock_dds`
- Command publisher: `lingtu_nav_control teleop`
- Obstacle publisher: `lingtu_nav_control cloud`
- Endpoint flags:

```text
--publish-cmd-vel 1
--check-obstacle 1
--use-traversability-cost 0
```

## Cases

### 1. No Obstacle

Input command:

```text
vx=0.18 vy=0.00 wz=0.00
```

Result:

```text
reason=accepted
output.vx=0.18
teleop_stops=0
teleop_slows=0
cmd_vel_published=61
motion_mock_received_cmd_vel=60
mock_pose_x=0.424237
```

The command passes through.

### 2. Slow Obstacle

Injected obstacle:

```text
body-frame point x=0.90 y=0.00 z=0.30 height=0.50
```

Result:

```text
reason=obstacle_slow
obstacle_distance_m=0.865724
request.vx=0.18
output.vx=0.063
teleop_slows=12
teleop_stops=0
cmd_vel_published=59
motion_mock_received_cmd_vel=45
mock_pose_x=0.139687
```

The endpoint applied the configured slow scale.

### 3. Stop Obstacle

Injected obstacle:

```text
body-frame point x=0.35 y=0.00 z=0.30 height=0.50
```

Result:

```text
reason=obstacle_stop
obstacle_distance_m=0.35
request.vx=0.18
output.vx=0.0
teleop_stops=13
teleop_slows=0
cmd_vel_published=60
motion_mock_received_cmd_vel=44
mock_pose_x=0
```

The endpoint vetoed forward motion and published zero velocity.

## Conclusion

The registered-cloud obstacle gate in `teleop_avoid` is working:

- no obstacle: command accepted;
- obstacle in slow zone: command slowed;
- obstacle in stop zone: command stopped.

The next separate gate is real-motor validation, where production
`publish_cmd_vel` is enabled for a short controlled test window and real
cancel/stop timing is measured.
