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

Navigation and the vendor control loop are separate processes. Final velocity is
a best-effort DDS stream with one retained sample and a 200 ms lifespan; navd
does not wait for reliable transport delivery or a vendor API response per
velocity. Task requests retain reliable delivery. Core retains one latest
velocity; there is no per-velocity acknowledgement or receipt matching.
After synchronous vendor maintenance, the
driver takes the latest velocity again and checks the existing command watchdog
before sending it. DDS captures both boottime and local steady time; the driver
transfers the elapsed command age between those clocks instead of comparing
their epochs. Receipt does not restart the validity interval. Rejected newer
samples do not discard an already accepted pending command. It never queues old
velocities for later playback.
Command sequence numbers do not gate velocity output. The newest DDS sample
replaces the pending command; command expiration handles missing input. Multiple
matched writers remain a control-source conflict. Losing the writer does not
start a second stop/release path.
The SDK remains serial on the driver thread; this does not make a blocked vendor
call interruptible or turn the software watchdog into a hardware deadline.

`DriverControlState.zero_output` reports a successfully applied zero output,
including a checked stop. Unknown or failed output is false; it does not prove
physical settling. Navigation stop confirmation observes state newer than its
stop request and, where required, quiet odometry. SDK errors remain driver facts;
goal, posture and control-ownership requests retain their discrete replies.

Rebuild and deploy navd, driver and DDS consumers together: the control-state
schema replaces the three navigation acknowledgement fields with `zero_output`.
Old reliable-only readers cannot match a new best-effort velocity writer. The
simulation bridge, command tap and recorder use the same shared profile and also
need their matching build. Topic names and timeout settings remain the same.

A normal SDK command failure triggers one zero-output attempt and reports the
original error without releasing the adapter or clearing newer input. The next
command can be sent without waiting for a locally inferred ready flag. Explicit
shutdown, posture and control-source transitions retain their own stop semantics.
Go2 SDK acceptance is separate from telemetry readiness: stale Sport state stays
visible in diagnostics but does not reinterpret a successful SDK call as rejection.
Navigation does not use driver telemetry as an admission gate. A failed DDS write
records an error and attempts zero without latching an operator/fault hold.

DOSO SDK 0.6 or newer owns gRPC, mTLS, Lease, sequence, retry, checked stop,
and typed Brainstem control/body state. `Doso` only maps those public types to
LingTu's neutral `Body` interface. The motion adapter does not start IMU or
joint telemetry streams; sensor consumers opt into those long-lived streams
separately.

Go2 also passively subscribes to SDK2 `rt/lowstate` and publishes measured
`q` (rad), `dq` (rad/s), and `tau_est` (Nm) on the typed LingTu
`/robot/joint_states` topic (`rt/robot/joint_states` on DDS). Its 12 joint names
follow the URDF order FR, FL, RR, RL, with hip, thigh, calf in each leg.
The header keeps the sample's receive wall time; duplicate SDK ticks never
refresh it. Samples expire after 500 ms, and publication sends only new
samples, at most once per 34 ms. This telemetry does not depend on motion
readiness or issue motion commands. Doso currently exposes no joint samples.

Go2 body feedback comes from `rt/sportmodestate`. Every callback updates the
receive timestamp, including stationary samples. A sample older than 500 ms
is reported as `sport_state_stale`; it does not block velocity submission or
change SDK acceptance. Invalid or excessive measured tilt still blocks motion. The
driver keeps the triggering error when its subsequent stop also fails, and logs
the operation, vendor-call duration and body-feedback age before attempting that
stop. The Go2 adapter counts actual vendor calls; a cached-state refresh counts
zero, while acquisition counts each stop/query/configuration call.

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

## Stair controller profiles

Rejected posture requests do not change the existing motion hold unless a
body-changing operation was attempted. Cached replies and late cancellation
also preserve the current hold and any pending standing confirmation. A request
that only reads state cannot create a permanent stop. Operations with uncertain
outcomes retain their hold until explicit recovery completes.

Profile requests validate readback and model availability before stopping the
body. A failure before any body operation is reported without holding velocity;
a new task request can retry. Failures after stop/posture/motor/profile operations
still require explicit recovery. Rejected posture requests do not reset such a
failed profile transition.

The same driver consumes `/driver/profile/request` and publishes
`/driver/profile/state`. A request reuses the navigation `task_id` and asks for
`stairs_up`, `stairs_down`, `restore`, or `hold`; it introduces no per-attempt identity.
The navigation executor renews the request while the handoff or traversal is
active. DDS requests are volatile and must have a timestamp within one second.
The driver retains the existing single final-velocity-writer gate.

`ProfileSwitch` performs the landing handoff in the driver loop. It waits for a
fresh checked zero command, stops/releases locomotion, observes Grounded when
required, requests the configured controller, separately reads its actual name,
stands again and obtains motion readiness. The model command's response alone
never confirms the active profile. The planned transition reports
`ready=false`, `control_assured=false`, `lease_valid=false` and
`reason=profile_transition`; this is an intentional hold, not a terminal
navigation failure. No second velocity writer exists.

The original actual controller name is retained across consecutive flights
in this driver process. A driver restart loses that observation; a restore
request then reports `previous_profile_unknown`, never a guessed default.
`restore` returns to that exact name after nav confirms exit onto a landing.
An explicit matching-task `hold` immediately suspends further body/profile
actions while retaining the handoff. Losing requests, pausing or cancelling
never restores a normal profile on the flight. A stale request stops further body/profile commands; a subsequent fresh
request resumes the retained handoff. An attempted body operation with a failed
or uncertain result requires explicit recovery rather than automatic repetition.

DOSO maps `Body::readProfile` and `switchProfile` to SDK 0.6's `GetProfile`,
`GetCmsState`, and `SwitchProfile`. The server's Grounded prerequisite is
retained. Configure `--stairs-up-profile` and `--stairs-down-profile` only with
installed and qualified profiles (environment variables
`LINGTU_DRIVER_STAIRS_UP_PROFILE` and `LINGTU_DRIVER_STAIRS_DOWN_PROFILE`). Empty
bindings report unsupported stairs and do not disable ordinary navigation.
The source fields are `driver.stairs_up_profile` and
`driver.stairs_down_profile` in the selected RobotConfig. ProductControl's
existing RunPlan resolution exports these values to the driver; do not maintain
a separate profile binding in the browser or the stair map annotation.
Go2 currently has no verified actual controller-profile readback binding; its
profile capability remains unsupported rather than interpreting a gait number
as model confirmation. Mock tests do not qualify a real stair policy.

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
The Doso target must have DOSO SDK 0.6+ (`BrainstemClient`) installed for CMake discovery;
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
does not create a background timer. LingTu calls it on entry to the velocity
channel when there is no command-writer conflict. After a successful acquisition, the polling loop schedules
the next call no earlier than 100 ms later, so the effective cadence is at most
10 Hz and is quantized by `poll_hz`; configuration rejects polling below 10 Hz.
Brainstem's default gRPC motion lease is 300 ms, and every accepted checked
velocity command also renews that lease. Multiple command writers suspend this
channel; a missing writer is handled by command expiration. After an SDK refresh
failure LingTu retries maintenance on `driver.reconnect_interval`; this does not
block new velocity submissions.

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

The Web keyboard and browser-connected USB/Bluetooth gamepad use the existing
typed operator-input path into native navigation; neither sends `Move` directly
to the vendor endpoint. The Go2 factory handheld remote is not integrated into
that path. SDK2 exposes [`SportClient::SwitchJoystick(bool)`](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp)
and [`WirelessController_`](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/idl/go2/WirelessController_.hpp)
with four stick axes and button bits, but the message has no radio-link validity,
remote timestamp or input sequence. Fresh `LowState` alone cannot establish
fresh wireless input. Unitree's [motion service documentation](https://support.unitree.com/home/en/developer/Motion_Services_Interface_V2.0)
states that disabling joystick response prevents stick movement from
interfering with the program; it does not specify the three handoff facts below.
The current driver therefore does not disable firmware
joystick handling or claim to safely arbitrate factory-remote motion.

Before connecting that remote to the operator protocol, verify on the named Go2
firmware that input remains available after joystick handling is disabled,
confirm what happens to factory stop controls, and check transmitter disconnect
behavior. The integration must normalize vendor input and publish existing
operator intent; final checked velocity must still come back through this driver.
Do not enable the optional SDK lease thread or substitute direct remote motion
for this handoff.

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
Product travel distance is estimated by localization using continuous SLAM
odometry. The body positions above remain bounded-run diagnostics.
Restarting the service does not power off the robot or expansion computer.
