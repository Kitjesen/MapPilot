# 2026-07-09 CmdVel Loopback Acceptance

## Purpose

Verify that the native C++ navigation endpoint can publish the final
`/nav/cmd_vel` DDS command and that a downstream hardware-side consumer can
receive it.

This was intentionally run in an isolated DDS domain. It did not enable motor
output on the production robot domain.

## Setup

- Robot host: sunrise field board
- Test DDS domain: `232`
- Production DDS domain: `0`
- Nav endpoint:
  - binary: `/opt/lingtu/current/build/nav_endpoint/lingtu_nav_native_endpoint`
  - `--publish-cmd-vel 1`
  - `--check-obstacle 0`
  - `--use-traversability-cost 0`
  - status: `/tmp/lingtu_cmd_vel_loopback_nav.json`
- Motion mock:
  - binary: `/opt/lingtu/current/build/nav_endpoint/lingtu_motion_mock_dds`
  - subscribes `/nav/cmd_vel`
  - publishes mock `/slam/odometry` and `/tf`
  - status: `/tmp/lingtu_cmd_vel_loopback_motion.json`
- Command tool:
  - `/opt/lingtu/current/build/nav_endpoint/lingtu_nav_control teleop 0.18 0.00 0.25 --duration-s 1.2 --rate-hz 12 --domain-id 232`

## Result

The loopback passed.

Native nav endpoint status:

```text
domain_id=232
publish_cmd_vel=true
teleop_cmd=15
teleop_outputs=65
cmd_vel_published=65
```

Motion mock status:

```text
domain_id=232
cmd_vel_received=61
mock_odometry_published=301
pose_x=0.262374
pose_y=0.0487985
pose_yaw=0.372812
```

This proves:

```text
/nav/teleop_cmd_vel
  -> lingtu_nav_native_endpoint
  -> /nav/cmd_vel
  -> downstream command consumer
```

## Production Safety Check

After the loopback test:

```text
domain232 test processes: none
production domain_id=0
production publish_cmd_vel=false
production cmd_vel_published=0
lingtu-teleop-dds.service=inactive
lingtu-teleop-dds.service enabled=disabled
```

The production robot remained in no-motion mode.

## Code Notes

- `motion_mock_dds` now uses the same DDS QoS profiles as the real topic
  contracts, so command-loopback tests do not fail due to reader/writer QoS
  mismatch.
- `teleop_bridge` also uses the topic QoS contract for `/nav/teleop_cmd_vel`.

## Remaining Gate Before Real Motion

For physical low-speed motion, remove or override the production no-motion
drop-in only for a controlled test window, then verify:

```text
teleop remote request
  -> /nav/teleop_cmd_vel
  -> safety-gated C++ endpoint
  -> /nav/cmd_vel
  -> real motor-side consumer
  -> timely cancel/stop
```
