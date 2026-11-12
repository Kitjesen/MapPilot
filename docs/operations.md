# Operations

**Status:** Current field mapping, navigation, and recovery runbook
**Audience:** Field operators and robot maintainers
**Runs on:** Linux field releases and supported robot targets

This page covers an already installed robot: **build a map -> save and clean
it -> reload and localize -> navigate -> stop or continue after intervention**.
For first installation, read [Getting started](getting-started.md); release
packaging, installation, and rollback are owned by
[Deployment](../scripts/deploy/README.md). Product scope and acceptance are
owned by [Product scope](product.md) and [Testing](testing.md).

Use the actual RobotConfig throughout. The commands below use `unitree/go2` as
an example, not a claim that other profiles have passed field tests. Commands
run on the robot computer from the installed `/opt/lingtu/current` release.
Use the configured absolute `LINGTU_PYTHON` interpreter for Python commands.
For simulation, use [Simulation](simulation.md) instead of this Linux runbook.

## Before each session

Keep the robot stationary while switching modes. Check the current mode, map,
and release before starting; save any mapping session that must be retained
before a switch, restart, or shutdown:

```bash
bash /opt/lingtu/current/scripts/lingtu status --robot unitree/go2 --env real --json
```

There is no separate `contract` or `motion` preflight approval before ordinary
use. ProductControl starts the services and completes map/localization handover;
navigation owns goal admission and the inputs needed to plan and execute.
Endpoint startup does not clear a stop or authorize motion.
The systemd Host probe uses `startup_state`, not aggregate data/motion readiness.
Generic status-file probes preserve `ready` and `status` as observations; they
do not infer initialization failure from runtime values such as `ready=false`
or `status=error`. File availability and configured freshness checks remain.
The ordinary systemd startup path does not run DDS sample-count probes. Map
activation and localization handover still use their owning lifecycle steps.

Use diagnostics when investigating a problem, not as a required startup step:

```bash
PYTHONPATH=/opt/lingtu/current/src "$LINGTU_PYTHON" -m diagnostics.field.service_readiness --pretty
```

Optional `--dds-seconds` samples DDS traffic; `--strict` makes diagnostic failures
affect the command's exit code. Neither option acquires control or publishes
motion. Engineering acceptance remains in
[Testing](testing.md#field-no-motion-order).

Choose reports by the question being investigated:

| Question | Source |
| --- | --- |
| Which Product and processes are running? | `lingtu status` |
| Why did a task stop or reject a target? | Navigation status and the native command result |
| Are services, files and communication available? | Optional `diagnostics.field.service_readiness` report; add `--dds-seconds` only to sample traffic |
| Does this release/platform meet acceptance criteria? | The explicit procedures in `docs/testing.md` |

In the diagnostic driver section, `ok` and `blockers` describe service/status
availability. `motion.connected`, `motion.ready` and `motion.control` preserve
the driver's report without recomputing motion permission. A successful service
report does not mean the robot can move. The full diagnostic inventory can
include services outside the selected Product; it is not a Product startup gate.

ProductControl owns `switch`, `status`, and lifecycle `stop`. Web submits the
same lifecycle requests; `systemctl` and `journalctl` are diagnostic tools,
not an alternative startup sequence.

## Build, save, and reload a map

### 1. Start mapping

In the Scene floating controls, choose **开始建图**, or use:

```bash
bash /opt/lingtu/current/scripts/lingtu switch map --robot unitree/go2 --env real
bash /opt/lingtu/current/scripts/lingtu status --robot unitree/go2 --env real --json
```

Wait for mapping readiness before moving. Open **遥控** in the Scene controls
to collect the area; **累计地图** shows the scans collected in this session.
Mapping uses direct checked teleoperation, so the operator must actively avoid
obstacles. It is not autonomous navigation around the room.

### 2. Save and inspect

Stop manual input, keep the robot stationary, and use **保存地图**. Choose a
name that does not overwrite a map needed by an active session. The default
**保存点云后自动后台处理** first archives the point cloud, then processes it in the
background. Clear the checkbox to defer processing until **开始处理** is clicked.
`source_saved=true` confirms the archive; `SUCCEEDED` confirms the processed map
was committed. An accepted request alone confirms neither. Keep the capture
session running until the archive is confirmed; do not power off during file
writes. Saving does not activate a map.

The save transaction processes source data, builds navigation artifacts
(including OctoMap), validates them, and commits the saved version. Dynamic
object cleanup is configured inside that transaction; there is no separate Web
**清理地图** action. Check the save result's cleanup summary: missing trajectory
data, an unavailable tool, or a cleanup failure can leave this step skipped or
failed even when non-required cleanup allows the save to succeed. Do not treat
`SUCCEEDED` as proof that all dynamic objects were removed.

Use **保存记录** to find previous saves after refreshing the page, inspect their
stage/reason, download the archived PCD, or explicitly process/retry/cancel a task.
Processing failure preserves the archive. Successful tasks also retain it;
archives are separate from loadable maps. File names, pre-cleaning
backups and retry retention are defined in [Map files and save tasks](operations/map-files.md).

Use **查看完整地图** or **已存地图** to inspect the result. Cropping or editing a
saved map is a separate maintenance operation and can invalidate derived
navigation artifacts; rebuild and check readiness before using it. It is not
an extra required step after every save. Map processing details belong to
[Maps](../src/maps/README.md).

The saved artifact is the input for subsequent localization and planning.
Display height filtering and layer visibility do not change that artifact or
the planner's three-dimensional map. Preserve the saved source data when
investigating a gap; a visually empty patch is not evidence that it is free.

### 3. Load for navigation

In **已存地图**, check that the selected map shows **导航数据齐全** and has no
activation blocker, then choose **使用此地图导航**, or switch explicitly:

```bash
bash /opt/lingtu/current/scripts/lingtu switch nav --robot unitree/go2 --env real --map MAP_NAME
bash /opt/lingtu/current/scripts/lingtu status --robot unitree/go2 --env real --json
```

Use the exact saved-map name. Loading ends the current mapping session; save
recent scans first. Wait for that map to be active, a valid map-frame robot
pose, fresh localization, and navigation readiness before selecting a goal.
Loading a saved point cloud for display is not this activation step.

The switch dialog can use automatic localization or a specified map-frame
initial pose (`x/y/z/yaw`). If the automatic result is unavailable or incorrect,
keep the robot stationary and use the supported initial-pose or global
relocalization controls before sending a goal. A seed helps localization; it
is not a substitute for a confirmed pose.

An active map cannot be overwritten, deleted, renamed, or rebuilt. Save a new
map or switch away before modifying the old one. After a reboot, select the
saved map explicitly and repeat the readiness checks; an old display or
in-memory pose is not a newly localized session.

## Navigate

**单点** keeps one draft destination. Each click replaces the draft and
automatically previews a path; check the path and click **开始导航** to submit
motion. **同层选点** uses the current body height. For a different three-dimensional
surface, choose **三维表面** or hold Alt while clicking, then check the target
height. A preview does not move the robot or prove that it will arrive.

**连续点** appends numbered points in click order. Reorder or delete points
before starting; use at least two points, or choose **单点** for one destination.
The connector shows edit order, not a planned route. **开始路线** saves the
current route snapshot and starts one native task; navigation plans each leg
when it becomes current. Web does not submit the next point itself. The active
map identity binds the draft, and changing maps clears it. An executing route
snapshot is read-only.

Use task status for progress, arrival, and the final outcome. Start, pause,
resume, and cancellation acknowledgements mean that a request was accepted;
wait for the corresponding robot state. The task card provides **暂停**,
**继续导航**, and cancellation for a single navigation task; the continuous
route card provides its own **暂停**, **恢复**, and **取消路线** controls.

Current Product profiles resolve planar navigation to a **sqrt(2) m/s norm
budget (about 1.414 m/s)** with independent **1.0 m/s x/y axis limits**. A
lower single-goal speed applies to that goal; continuous routes use the active
Product limit. Session overrides can lower these defaults, so inspect the
active session rather than assuming an old deployment uses the new limit.
Source changes take effect on the next ProductControl switch.

### Manual takeover during navigation

Open **人工接管** from the fixed toolbar at the top right of the Scene map
while the `nav` Product is active. In a teleoperation or mapping Product, the
same visible entry is labelled **遥控**. It remains available when the top-left
operation dock is collapsed. The opened control bar contains **遥控限速**;
select the desired speed there before moving.
All operator Products use the same per-axis contract. Holding forward and
strafe together can request `(vx, vy) = (1, 1)` m/s, a total of approximately
1.414 m/s. Planning and tracking derive that total-speed budget from the input
limit; SCAN and the final command boundary still cap each output axis at 1.0
m/s and check the actual command's 3D braking space. An explicit lower
total-speed override still applies. Ordinary translation acceleration is now
**4.0 m/s²** across Products, for both SCAN planning and tracking. CMU retains
this same resolved value; direct teleoperation and mapping use it through their
existing velocity smoother. Turning acceleration is **4.0 rad/s²**. Product
motion defaults are resolved once rather than maintained separately in each
backend. The configured braking deceleration is **4.0 m/s²** for the same
contract; it remains a command/trajectory parameter and is not a measured
robot stopping distance.
Recovery translation and rotation candidates use **0.50 m/s** and **0.80
rad/s** by default, with the same 4.0 m/s² linear acceleration contract.
The web selector offers the backend limit as well as lower speed choices; it
initially selects 1.0 m/s per axis, capped by the backend.
Turning remains capped at 1 rad/s. These are configured command/trajectory
limits, not measured robot acceleration or stopping performance. Existing
robot sessions keep their resolved settings until the updated Product is
installed and switched through ProductControl.
Ordinary SCAN tracking and pure assisted rotation now use the existing motion
acceleration settings for command continuity. Replanning retains the command
history, while final braking reductions update that history before motion can
speed up again. Releasing input and safety stops still request immediate zero;
these changes do not establish measured stopping distance or hard jerk limits.
Opening the panel only connects the input adapter. A fresh keyboard command or
deliberate gamepad input pauses the current navigation task and uses the same
native checked motion path; it does not switch Products or replace the goal.
Release the input to request a Hold. Use the task card's **继续导航** to continue
the retained target from the measured robot position. Closing the panel releases
its manual authority and does not automatically resume the task.
Closing the panel before any manual input was admitted leaves the current
navigation running; it does not insert a stop merely to close the input source.

For **键盘**, hold W/S to move forward/back, A/D to move sideways, or Q/E to
turn. Shift requests slower motion; Space holds and Esc exits the panel. Fresh
input is required after focus returns; an old held key is not replayed.

The input selector chooses **键盘** or **手柄**; they cannot send commands at the
same time. For a USB/Bluetooth gamepad connected to the browser computer, select
the device, release L1/LB and center its sticks first. Hold **L1/LB** while using
the left stick for forward/lateral movement and the right stick for turning.
Release the shoulder button or center the sticks to hold. A disconnected
gamepad, hidden page, lost focus, or input-mode change clears the previous
intent. Restore focus, release L1/LB and center the sticks before moving again.
The existing speed selector and native final motion limits still apply.

Only the browser's `standard` gamepad mapping is supported; an unknown layout is
shown as unavailable rather than assigned guessed controls. Browser permission
and device support must be checked on the operator computer. Local callback
tests do not establish compatibility with a physical gamepad or field motion.

Go2's factory handheld remote is a separate vendor control path. It is not yet
routed through this panel or the native operator-input protocol. Its supported
handoff requires disabling competing firmware joystick motion while preserving
the intended stop behavior and obtaining usable remote input after that change.
See the [driver control boundary](../src/drivers/real/motion/README.md#control-assurance-and-vendor-leases).
Do not treat factory motion outside LingTu as a tested navigation takeover.

## Stop and recovery

Choose the action for the situation; stopping motion and shutting down the
Product are different operations:

| Action | Effect and next step |
| --- | --- |
| Task **暂停** | Retains the task. Wait for paused/stop confirmation; **继续导航** resumes the single navigation task. |
| Exit manual input | Holds motion and releases the manual source. It does not resume a paused task. |
| **解除控制暂停** | Releases a control hold after robot confirmation. It does not continue a paused task. |
| Cancel the task | Requests a terminal cancellation. Wait for the final cancelled state and stop confirmation. |
| Web **停止** or SDK `stop` | Requests software emergency stop. **解除急停** releases its latch only after confirmation; it does not replay the old task. |
| ProductControl `stop` | Ends the active Product and its managed processes. Save needed mapping data first. |

For emergency stop without changing the Product, or to end the session:

```bash
PYTHONPATH=/opt/lingtu/current/src "$LINGTU_PYTHON" -m lingtu.sdk.cli --host 127.0.0.1 --port 5050 stop
bash /opt/lingtu/current/scripts/lingtu stop --robot unitree/go2 --env real
```

Confirm terminal zero output and driver acknowledgement, and check the robot's
physical motion before treating it as stopped. A `HELD` command or a successful
request alone does not confirm physical settling. Use task pause/cancel for normal task control;
ending a completed task does not require shutting down the whole Product.

If a switch fails, inspect ProductControl's report and current status first. It
either restores the previous committed Product or leaves no committed Product;
do not guess by starting individual units.

If navigation reports `motion.permission=HELD` with
`localization_not_tracking`, the robot is withholding autonomous commands while
saved-map localization recovers. Local escape motion also waits for valid
localization. Check the current motion observation separately from
`motion.stop_confirmation`: a confirmed stop command alone does not prove
physical settling. Inspect the
current task before deciding whether continuation or a new goal is needed;
task resume does not repair missing localization. If recovery does not restore
a valid pose, inspect `/api/v1/localization/status`, verify the active map and
robot position, and use the existing relocalization interface. Do not repeatedly
submit the same goal or raise correction limits to hide the problem.

## App closure and reconnection

The robot owns motion and task execution. A client can disappear without sending
a final message. The following behavior applies to App closure, backgrounding,
power loss and network loss.

| Situation | Robot behavior | Reconnection behavior |
| --- | --- | --- |
| Active manual control disappears | Native sample freshness expires and the operator lease is reclaimed through the existing zero-output barrier. | Previous stick and key input is cleared. New physical input is required. |
| A viewer closes during autonomous navigation or inspection | The robot retains the task. Its normal execution and motion checks continue. An unused manual connection releases only its own permission. | Read current robot and task status. Opening the interface sends no task-start command. |
| Manual takeover has paused an autonomous task | Manual input expires and stops. The autonomous task remains paused. | Explicit task resume continues the retained task. |
| A mode-switch request was accepted before the App closed | ProductControl completes or rolls back the operation and retains its result independently of the client. | Query the original request ID and then read the current Product. |
| The previous request cannot be found | The client cannot infer whether it was never received, its receipt expired or the robot rebooted. | Read current state and present the missing result. A reconnect does not resubmit the request. |
| The user explicitly pauses, cancels or stops | Execute the selected operation and retain its outcome. | Restore the actual task, mode and operation state. |

On connection, the Web selects the workspace for the robot's current Product.
An explicit saved-map or workspace link takes precedence on initial connection.
An old successful receipt does not select a workspace for an obsolete Product.
Subsequent polls of the same Product session leave the user's workspace choice
alone. The existing native timeouts and leases determine manual input expiry.

Full Product shutdown also stops its Host and runtime Gateway. The independent
control service keeps the public Web/API entry and accepted operation results
available. Runtime APIs return `503 runtime_unavailable` while Host is stopped.
Use the same `/api/v1/product-control` endpoint to query state or start a Product.
After a confirmed stop, the Web shows standby and suspends runtime subscriptions
and polling. Starting a Product restores them. Unexpected disconnections retain
the existing reconnect behavior.
The stopped-state Web map catalogue still depends on Host/mapd. An SDK can
restart navigation with a known saved-map name through ProductControl.
Local tests cover manual input expiry, passive
disconnects, retained operation results and workspace restoration. Forced App
termination and network loss still require platform-level validation.

## Web mapping and navigation switches

Web mode switching and persistent remote access require `lt-control` configured at
[first installation](getting-started.md#enable-web-mode-switching).
Its public entry calls the same ProductControl operations directly and streams
runtime traffic to Host. The loopback transport serves cross-process callers.
ProductControl owns lifecycle
transactions and the native navigation/driver chain owns motion checks.
After a disconnect, query the retained request ID before retrying. An interrupted
request requires a fresh status check. Receipts in `/run/lingtu` do not survive
a reboot.

## No-motion diagnosis

These checks can reveal a wrong release, missing device, stale stream, map
identity mismatch, or an unready process without requesting motion:

```bash
bash /opt/lingtu/current/scripts/lingtu status --robot unitree/go2 --env real --json
PYTHONPATH=/opt/lingtu/current/src "$LINGTU_PYTHON" -m diagnostics.field.soak \
  --duration 120 --interval 2 --json --strict
```

Use the status report to choose relevant logs, for example:

```bash
journalctl -u lt-slam.service -n 80 --no-pager
journalctl -u lt-nav.service -n 80 --no-pager
journalctl -u lt-driver.service -n 80 --no-pager
```

The selected Product declares the required services. For service owners and
native dataflow, use [Runtime](runtime.md); do not start or restart individual
services to bypass a failed Product switch.

## Recording and replay

In Web, open **操作 → 数据流与录制**. The same workbench has **实时 / 录制 /
历史回看** tabs. Live view includes the current camera video and shows actual received frequency, serialized-payload
throughput, sample age, selectable numeric-field curves and bounded latest fields. Inspecting
a stream does not start recording. Its separate checkbox selects capture data;
the topic/camera choices persist when the panel closes or tabs change.

The live camera uses the existing WebRTC/JPEG stream and labels only fresh
decoded frames as current video. An unloaded camera, an unavailable service and
stopped frame updates have separate states. Viewing the camera does not start a
Product, capture or motion. Saved-session media belongs to **历史回看** and is
explicitly marked **录制回看 · 非实时**. A local recording-fixture page does not
provide a robot camera unless it connects to a running robot Gateway.

For an immediate numeric trend, select a stream in **实时**, then select a
physical field such as `position_rad[0]`, `angular_velocity_rad_s.x` or
`pose.position.x`. The browser samples distinct latest-message updates; this
trend is not the full sensor-rate recording. When updates stop, retained
history is marked as stale instead of being extended with repeated values.

In **录制**, choose data groups and individual streams, set a duration/name,
and start recording. The choices come
from the installed native recorder; they are supported contracts, not a live
publisher-discovery list. Color video is a separate option on Linux. Depth
images and camera intrinsics are independent DDS choices.

After stopping, select the session to compare selected topics with actual
message counts, check the file/type/CDR envelope offline, download individual
artifacts or export the complete declared data set as TAR. A quiet optional
event stream may have zero samples without failing an ordinary capture. A
missing required sensor, no received samples, or an invalid inspection evidence
chain must remain visible as a failure. Retain failed recordings for diagnosis;
starting another recording does not require deleting the previous evidence.

For historical inspection, open a saved session under **历史回看** and use its
shared timeline. Select the data stream and numeric field, click the chart or drag
the playhead, then play/pause or change the viewing speed. The time-window
controls zoom around the playhead and restore the full range. No DDS messages
are published by these controls.

Choose a second session in **对比 B** to compare the same field on a shared
axis. Both recordings start at elapsed zero by default; their wall-clock dates
do not have to match. A positive **B 偏移** moves B later, a negative offset
moves it earlier, and **重置对齐** restores zero offset. The legend retains
source A/B identity and source-local cursor times. The displayed `B − A`
difference uses available retained samples, not interpolated ground truth.
Missing spans and times outside either source remain empty. Numeric values
are unchanged; matching units and coordinate/joint meanings are necessary
before interpreting their difference. Full source timestamps remain available
in the time-source details.

Large historical curves use peak-preserving min/max reduction, identified
below the chart. Zoom in to read a narrower range at greater detail. The
underlying MCAP remains complete; the chart's selected samples and observed
gaps do not establish lossless capture or exact sensor synchronization.

Recorded camera, point-cloud frames and trajectory share this same playhead.
Play, pause, rate, scrub and B offset affect all views together. Wide layouts
can show A/B sources side by side; the embedded sidebar selects a source on
the same shared timeline. The image and scene labels retain their actual
sample time and age, so a reduced display rate is not mistaken for the full
recorded sensor rate. Paused image seeks select the last indexed frame at or
before the cursor. Time outside camera coverage remains empty.

Camera reads run one at a time. During playback the next read follows the latest
cursor instead of canceling a decoder that is still working. The last completed
image remains visible with its actual time and age while updating. The viewport
distinguishes the first read, an update, a failed read with retry, the next
available segment and the end of a recording. A no-frame response does not stay
in a loading state. Coverage ends are exclusive; the exact end of the source is
shown as ended rather than extending its last image indefinitely.

Cloud and odometry/path data keep their recorded coordinate frames. Only
matching known frames overlay; otherwise choose a frame to inspect separately.
The trajectory shows the past portion of the current buffered window. Stored
planned paths are visually distinct and may extend into the future; they are
recorded plans, not proof of actual movement. A later empty path clears the
previous plan. No transform, motion or route is inferred by the viewer.

New native color recordings retain each encoded frame's acquisition timestamp
inside the segment index. Older segment-only indexes use an explicitly labelled
frame-order time estimate. Recorded source timing and DDS receive/log timing
remain distinct; the interface does not certify hardware synchronization.
The native scene reader supplies bounded time windows with display-only cloud
sampling, while the selected original CDR remains in the MCAP. Camera-only
sessions also have a timeline without requiring a fabricated DDS stream.

The compiled catalog includes robot joint states (position, velocity and
estimated torque), sensor inputs, localization/TF/GNSS, navigation paths and
states, control requests/acknowledgments, and inspection events. A platform
must actually publish the selected contracts. Motor current/temperature,
battery and foot-contact telemetry are not captured merely because the SDK
could expose them. See the [native catalog](../src/native/recording/README.md)
and [source-backed design/gaps](research/recording.md).

For terminal operation, record native DDS without a camera, then inspect or
dry-run the result:

```bash
/opt/lingtu/current/bin/lingtu_recorder topics --json
/opt/lingtu/current/bin/lingtu_recorder record \
  --output-dir SESSION_DIR --dds on --camera off \
  --dds-topic /imu/raw --dds-topic /lidar/raw_frame \
  --dds-topic /robot/joint_states --dds-topic /slam/odometry
/opt/lingtu/current/bin/lingtu_recorder status SESSION_DIR
/opt/lingtu/current/bin/lingtu_dds_player --info SESSION_DIR/dds/sensors.mcap --json
/opt/lingtu/current/bin/lingtu_dds_player \
  SESSION_DIR/dds/sensors.mcap --dry-run --json
```

Load the installed native command environment once in an operator shell, then
use the short command:

```bash
source /opt/lingtu/current/scripts/deploy/thunder/runtime-env.sh
ltplay SESSION_DIR
ltplay SESSION_ID -r 2
ltplay --info SESSION_DIR --json
ltplay --series SESSION_DIR --topic /robot/joint_states \
  --field 'position_rad[0]' --start 0 --end 5 --max-points 2000 --json
```

`ltplay` accepts a session directory, MCAP file or bare session ID. Bare IDs use
`LINGTU_RECORDING_ROOT` or `~/data/lingtu/recordings`. The default remains
isolated domain 84 and rate 1. When raw IMU/LiDAR inputs are present, the short
entry replays those inputs by default; otherwise it replays recorded sensor
streams for inspection. Explicit `--topic` overrides this choice, for example
`ltplay SESSION_DIR --topic /robot/joint_states`. The established
`lingtu_dds_player` retains its prior default selection. The two names compile
the same player implementation.

Recording does not authorize motion. Recording and replay provide evidence;
neither owns Product lifecycle. Web's offline check does not launch timed
playback or fully decode the semantic fields of every message. The native
player republishes selected sensor streams; commands, task requests and motion
mode switches are record-only. Configure downstream consumers for the replay
domain and recorded timestamps. Web's numeric timeline is a read-only viewer;
automatic replacement of live Product inputs is separate from this interface.

DDS recordings use MCAP profile `lingtu.dds.v1`; the final
`.mcap` is published only after clean closure, while an interrupted session
keeps its temporary file for inspection. Implementation details live in
[`src/native/recording/README.md`](../src/native/recording/README.md).

## Robot and credential boundaries

Addresses, interfaces, devices, geometry, and calibration come from RobotConfig
and machine configuration. Preserve these boundaries for every supported robot.
Never store credentials in documentation, Product YAML, or release defaults.

## Go2 connection and debug records

Current Go2 connection topology, SSH paths, calibration status, and dated field
evidence live with the robot profile and field handoff:

- [Go2 RobotConfig notes](../config/robots/unitree/go2/README.md)
- [Go2/NX operations](operations/go2-offline-mapping.md)

The dated records in `docs/operations/` preserve operational context. They are
not current operating instructions or proof that this release passed field
motion acceptance; the Go2/NX handoff is their index.

For supervised motion and evidence collection, follow
[Testing](testing.md#supervised-motion). Native navigation retains final motion
arbitration and the driver is the only hardware command writer. A source or
simulation pass does not establish that the installed release has passed on
the current robot, map, or route.
