# LingTu native field driver

`lingtu_driver` is the single native motion-output process selected by the
real-environment RunPlan. Its common loop consumes `rt/nav/cmd_vel`, enforces
freshness and single-writer gates, publishes `rt/driver/control_state`, and
delegates vendor I/O through `Body`.

Supported adapters:

| `RobotConfig.driver.backend` | C++ robot | Transport |
| --- | --- | --- |
| `doso` | `Doso` | Brainstem C++ Client SDK |
| `go2` | `Go2` | Unitree SDK2 high-level Sport API |

A robot implementation only translates the common physical `Velocity`/stop/control-state
contract to one vendor API. `Velocity` is always expressed as `vx_mps`,
`vy_mps`, and `yaw_rps`; no normalized `[-1, 1]` representation exists in the
LingTu driver. The common driver owns DDS, command freshness, speed limits,
single-writer enforcement, status, and shutdown safety.

DOSO SDK 0.5 or newer owns gRPC, mTLS, Lease, sequence, retry, checked stop,
and typed Brainstem control/body state. `Doso` only maps those public types to
LingTu's neutral `Body` interface. The motion adapter does not start IMU or
joint telemetry streams; sensor consumers opt into those long-lived streams
separately.

`Body` also exposes the neutral actions `Stand`, `Sit`, `Recover`, and `Damp`,
plus body and health snapshots. Go2 maps all four actions to SDK2. Doso maps
`Stand` and `Sit` to its authenticated Brainstem posture RPCs and reports
`Recover`/`Damp` as unsupported. `Damp` is never used as navigation stop or
fault handling. In SDK2 it selects damping mode:
LingTu treats that as an explicit posture-releasing action, not as motor power
isolation or an emergency stop, because the robot may lower under gravity.

The robot driver is physical robot configuration, not a Product or operating mode.
Change it in the selected model's `robot.yaml`, then rebuild and deploy the matching
driver. ProductControl publishes that value in `/run/lingtu/session.env` and
restarts the driver when the resolved configuration changes.

## Why the robot is selected at build time

The Doso build uses the Ubuntu CycloneDDS C runtime. Unitree SDK2 ships its
own CycloneDDS C/C++ runtime. Linking both implementations into one process can
interpose identical `dds_*` symbols across incompatible builds. Therefore each
driver binary contains exactly one robot implementation:

```bash
LINGTU_DRIVER_BACKEND=doso bash scripts/build/build_driver.sh
LINGTU_DRIVER_BACKEND=go2 bash scripts/build/build_driver.sh
```

For Go2, all DDS calls in this process link to SDK2's CycloneDDS. For Doso,
they link to the system CycloneDDS. Different LingTu processes may use their
normal system runtime and communicate over the DDS wire protocol.

The Go2 executable carries an `RPATH` to the selected SDK2 prefix. This also
keeps SDK2's indirect `libddsc` dependency on the same runtime and prevents the
system CycloneDDS C library from being loaded into that process. The field
launcher verifies the resolved `libddsc`/`libddscxx` paths with `ldd` before
starting the driver.

The Go2 target must have SDK2 installed at `/opt/unitree_robotics`, or set
`LINGTU_UNITREE_SDK2_PREFIX` consistently for both build and service startup.
The Doso target must have DOSO SDK 0.5+ (`BrainstemClient`) installed for CMake discovery;
set `LINGTU_BRAINSTEM_CLIENT_PREFIX` when it is not under a standard prefix.

## Runtime configuration

The exact values come from RobotConfig through the RunPlan; the systemd unit
does not own a second copy.

- Go2 requires `driver.network_interface`, `driver.network_address`, and
  `driver.probe_ip`. SDK2 still discovers the robot over the interface rather
  than connecting to the probe IP. Product compilation binds every native DDS
  process to that same interface, while deployment persists the static address
  without adding a default route. Driver startup requires carrier, the exact
  address, and a route to the probe host before SDK2 starts.
- Doso requires `driver.target` plus its mTLS files.
- Both adapters use the same speed, command-timeout, status, and shutdown
  contracts.

## Brainstem lease timing

`brainstem::Client::refresh()` performs one acquire-or-renew operation; the SDK
does not create a background timer. LingTu calls it once when the command-writer
gate becomes ready. After a successful acquisition, the polling loop schedules
the next call no earlier than 100 ms later, so the effective cadence is at most
10 Hz and is quantized by `poll_hz`; configuration rejects polling below 10 Hz.
Brainstem's default gRPC motion lease is 300 ms, and every accepted checked
velocity command also renews that lease. With no valid command writer it is not
called; after a failure LingTu retries on `driver.reconnect_interval` instead of
at 10 Hz.

On a fault or `SIGTERM`, the common loop calls `Body::stop()` before clearing
runtime state. `Result::confirmsStop()` is true only after the vendor endpoint
acknowledges its stop/zero command and local motion readiness is invalidated.
Go2 maps this to `StopMove` (SDK2 exposes no explicit lease-release method);
Doso sends a checked zero and then releases its explicit Brainstem control
lease. Transport success alone is never treated as stop confirmation.

## Control assurance and vendor leases

`control_assured` is the vendor-neutral readiness fact used by LingTu. It means
the final command writer is unique, the vendor transport and state are fresh,
the initial zero command was acknowledged, and the adapter is currently able
to accept checked velocity commands. `lease_valid` describes only a real
vendor lease and must never be fabricated to make a robot look ready.

Doso sets both values after Brainstem grants its explicit control lease. Go2
uses SDK2's default `SportClient` without the optional background lease thread,
so it sets `control_assured=true` after the checks above and always reports
`lease_valid=false`. On the tested Go2 SDK2/Jetson target, enabling
`SportClient(true)` obtained a lease but aborted the process during normal
`SIGTERM` cleanup. Until Unitree provides a shutdown-safe lease lifecycle, that
mode is not used in the field driver. Do not issue concurrent motion commands
from the Unitree App or remote controller while LingTu owns the final command
path.

Go2's raw `SportModeState.error_code` is retained as `adapter.state_code` for
diagnosis. It is not promoted to a LingTu health fault without a documented
SDK2 interpretation; fresh Sport state and motion mode remain the public health
and readiness evidence.

The public `control` and `body` status are vendor-neutral. Transport names,
targets, and raw control owners such as `brainstem_grpc`, `unitree_sdk2`,
`grpc`, and `sdk2` appear only under the `adapter` diagnostics object.
For adapters that provide it, `body.odometry_position_m` exposes the latest
local odometry position. `velocity_tracking` retains command-versus-observation
means, while `last_output_kind` distinguishes a motion command from commanded,
watchdog, fault, or shutdown zeroes. `active_motion_run` and
`last_completed_motion_run` retain each bounded run's elapsed time, integrated
command distance, start/end odometry, measured displacement, ratio, and ending
output kind. This is the evidence used to diagnose a command such as
`0.30 m/s` for `3 s` without inferring distance from the requested speed alone.
Restarting the service does not power off the robot or expansion computer.
