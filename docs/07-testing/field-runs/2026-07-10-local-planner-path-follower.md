# LocalPlanner to PathFollower Field Audit

Date: 2026-07-10

Host: `sunrise@192.168.66.13`

## Claim

The field navigation product has one native C++ owner for local planning and
path following. `LocalPlannerCore` and `PathFollowerCore` execute in the same
`lingtu_nav_native_endpoint` tick. Python does not duplicate this chain in the
`env=real` Product runtime.

## Static And Unit Evidence

- Endpoint CTest: 9/9 passed.
- PathFollower core: 21/21 passed.
- Path edge cases: 14/14 passed.
- LocalPlanner core: 39/39 passed.
- Frame decoding, path echo, input gate, real-dt acceleration, relocation, final
  target, rolling-horizon slowdown, and padded rotation footprint all have
  regression coverage.

The field acceleration contract is `delta_v <= max_accel_mps2 * delta_t`.
`20 Hz` supplies a nominal `delta_t=0.05 s`; it is not an acceleration unit.

Local follow-up after the field run added explicit
`LINGTU_NAV_MAX_SPEED_MPS`, `LINGTU_NAV_MAX_ACCEL_MPS2`, and
`path_follower.nominal_dt_s` reporting. These follow-up changes passed local
C++ and contract tests but are not yet deployed because sunrise became
unreachable. The production evidence below applies to the previously deployed
real-dt fix; it must not be used to claim the new status fields are live.

Local follow-up evidence:

```text
test_nav_endpoint_config: 1 passed
PathFollower core:        22 passed
Path edge cases:          14 passed
NavLoop:                   6 passed
LocalPlanner core:        39 passed
Python fallback guards:    5 passed
deployment contracts:     39 passed
profile ownership:         3 passed
```

`scripts/build/build_nav_endpoint.sh` now runs CTest after every default build,
so an old test executable cannot be mistaken for validation of a new source
tree. Set `LINGTU_NAV_ENDPOINT_RUN_TESTS=0` only for an explicit diagnostic
build, never for a release/install build.
The endpoint CTest catalog includes its endpoint/config tests plus the complete
PathFollower and LocalPlanner core suites.

## Isolated DDS Domain 77

Topology:

```text
lingtu_motion_mock_dds
  -> rt/slam/odometry + rt/tf
lingtu_nav_native_endpoint
  -> LocalPlannerCore -> PathFollowerCore -> rt/nav/cmd_vel
rt/nav/cmd_vel
  -> lingtu_motion_mock_dds pose integration
```

Five-metre path sample:

```text
active_path=true
local_path_points=101
target=[5,0,0]
target_distance_m=2.025809
cmd_vel.vx=0.4
mock_pose.x=2.99836
outputs=154
cmd_vel_published=154
frame rejects=0
input_gate.ready=true
```

This proves that the rolling local-path endpoint no longer causes an early
`0.3 m/s` final-goal slowdown.

Two-metre completion sample:

```text
active_path=false
last_local.reason=goal_reached
last_local.target=[2,0,0]
target_distance_m=0.345668
cmd_vel=[0,0,0]
motion_mock.active_cmd=false
timing_ms.nav_tick=0.127045
timing_ms.overrun=0
```

The completion target is the real final waypoint, not a default origin value.
All temporary Domain 77 processes were stopped after the test.

## Production Domain 0

`lingtu-nav-dds.service` was restarted onto the tested binary and checked
without publishing a goal or path.

```text
service=active
NRestarts=0
ExecMainStatus=0
input_gate.ready=true
frame rejects=0
timing_ms.overrun=0
publish_cmd_vel=false
```

The running `/proc/<pid>/exe` and installed binary shared SHA256:

```text
420f01798a450f98e014799194baf6bdb396ee012763dbbf8f0e6dad8a985b68
```

The `no-motion-preview.conf` production override remains installed. Domain 0
therefore validates input health and runtime ownership without actuating the
robot.

## Pending Reconnect Gate

When sunrise is reachable again:

1. Sync the changed navigation source, CMake, build script, and service unit.
2. Run `bash scripts/build/build_nav_endpoint.sh`; the required CTest catalog
   and all tests must pass before installation.
3. Repeat the Domain 77 motion-mock path and completion checks.
4. Install/restart `lingtu-nav-dds.service` while preserving
   `no-motion-preview.conf`, then verify the status `path_follower` object and
   zero frame rejects on Domain 0.

## Native Driver Follow-up

The hardware command boundary after `/nav/cmd_vel` is now a LingTu-owned C++
service:

```text
lingtu-nav-dds
  -> rt/nav/cmd_vel
  -> lingtu-driver
  -> Brainstem RobotControl.Walk(Vector3)
```

Local Linux/WSL verification completed without sunrise:

```text
build/driver/lingtu_driver: built
IDL generation: passed
Brainstem proto generation: passed
test_driver_core: passed
test_driver_io: passed
CTest: 2/2 passed
```

`test_driver_io` publishes a typed DDS `TwistStamped` on an isolated domain,
reads it through the product C++ reader, normalizes it in the fail-closed core,
and verifies the resulting call on a C++ `RobotControl.Walk` test server.

The product catalog now installs `lingtu-driver.service`. Its installer stops
and disables the old Python `lingtu-thunder-dds-endpoint.service`, and the new
unit declares a systemd conflict so both command sinks cannot own the robot at
the same time. Idle readiness is based on a fresh
`/dev/shm/lingtu/driver_status.json` with `ready=true` and `connected=true`;
an idle `/nav/cmd_vel` topic is not treated as a failure.

All native motion publishers now stamp `/nav/cmd_vel` and
`/nav/operator_motion/sample` with the canonical `body` frame. The driver still accepts
`base_link` only as a compatibility input. Its main loop also sends a
best-effort zero command with reason `fault` before propagating an unexpected
C++ exception; normal service shutdown uses reason `shutdown`.

S100P deployment remains pending because sunrise is currently unreachable.
On reconnect, build with `bash scripts/build/build_driver.sh`, preserve the
nav no-motion override, install the `driver` catalog service, verify the status
heartbeat and zero startup command, then run any nonzero motion check only as
an explicit controlled field test.
