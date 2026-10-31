# Native recording

This directory contains LingTu's ROS-free native recording product surface.
`lingtu_recorder` owns one observable recording session and supervises the
specialized native DDS and camera recorders. It is an explicit field capability;
it is not part of the default Product process graph.

The same root-scoped controller also builds on Windows for native DDS recording
in simulation. Use `--camera off`: the camera recorder still requires Linux
POSIX shared memory, and a Windows camera request fails explicitly. Windows
workers receive a process-scoped stop event and finalize through the same MCAP
and session-manifest completion checks as Linux workers.

Operator commands, install paths, failure handling, replay safety, and the
rosbag2-inspired roadmap are maintained in the canonical
[native recording and replay guide](../../../docs/operations.md).

Camera images remain on the existing POSIX shared-memory data plane. They are
recorded as short external video segments with a small MCAP timeline, rather
than copied into ordinary DDS samples.

The on-disk format is MCAP profile `lingtu.dds.v1`:

- message encoding: `cdr` (the RTPS serialized-payload header is retained);
- schema encoding: `omgidl` with the self-contained `messages.idl`;
- `logTime`: recorder receive wall time;
- `publishTime`: DDS source timestamp;
- uncompressed, indexed MCAP chunks with CRCs;
- completed chunks and the final file are flushed to durable storage;
- output is first written as `<name>.mcap.tmp` and renamed after a complete
  MCAP footer has been written.

An interrupted or failed session keeps its `.tmp` file for inspection or MCAP
recovery, but a session that captures no sensor samples is never promoted to a
final `.mcap` file.
On a handled capture or timeline verification failure, buffered samples are
closed into that temporary container so a short failed run retains its real
facts. A readable `.tmp` container remains an unpublished, failed recording;
only the unchanged evidence checks can authorize promotion to `.mcap`.

MCAP C++ 2.1.3 is vendored under this module's `vendor/mcap` under its MIT license.
LZ4 and Zstd are disabled until measurements on a claimed target platform
justify either dependency.

Before creating a session or starting workers, the C++ manager checks free
space on the target filesystem. `--min-free-gib` defaults to 5 GiB and is
recorded with the observed startup capacity in `session.json`. While recording,
the manager checks free space at a one-second interval. Falling below the
threshold, or losing the ability to query storage, stops the workers through
the normal grace deadline and marks the session failed. `0` explicitly disables
both threshold checks. This is a stop watermark, not an automatic file-rotation,
retention, or deletion policy, and it cannot guarantee that a disk will never
fill between checks. See the operator guide for the supported field workflow.
The detailed topic catalog remains below so the format and generated IDL
bindings have one developer-facing reference.

## Product entry point

The short playback entry uses the same native player implementation:

```bash
ltplay SESSION_DIR
ltplay SESSION_ID
ltplay SESSION_DIR -r 2
ltplay --info SESSION_DIR --json
```

Directories resolve to `dds/sensors.mcap`. A bare session ID resolves under
`LINGTU_RECORDING_ROOT`, or `~/data/lingtu/recordings` when it is not configured.
Playback remains isolated in domain 84. When a recording contains raw IMU or
LiDAR samples, `ltplay` defaults to those estimator inputs rather than mixing
old SLAM outputs into a new estimator run. If neither raw input is recorded,
available sensor streams are replayed for inspection. Explicit `--topic` choices
override that default. Control and task messages never become replay inputs.
The internal `lingtu_dds_player` entry retains its established all-sensor
default for existing callers. Color video is not a DDS sensor playback stream.

### Live observation

Web starts one read-only observer on demand and caches its JSON-line snapshots:

```bash
lingtu_dds_observer --domain 0 --interval-ms 500
lingtu_dds_observer --topic /robot/joint_states --topic /slam/odometry --seconds 10
```

The observer uses the same compiled topic/type/QoS catalog as recording. Each
snapshot reports cumulative delivered samples and serialized CDR bytes, window
rate/throughput, receive age, original DDS source time and bounded latest-value
previews. These rates describe what this observer receives; they do not certify
device publication frequency or total network bandwidth. Unseen streams have
null age/value, and a quiet stream's rate returns to zero while its last value
and growing age remain visible.

Joint previews include position/velocity/estimated torque with units; IMU,
odometry, driver/navigation state, velocity output, camera intrinsics and GNSS
have typed fields. Less common events expose a bounded DDS textual preview.
Cloud/image previews read at most a 512-byte serialized prefix for metadata;
their point/pixel arrays are not copied into JSON. The observer retains at most
one DDS sample reference per stream and writes no recording files or commands.

Gateway and other managed callers use the root-scoped C++ control contract.
The native process allocates the session ID, serializes start/stop with the
root lock, launches the foreground manager, and returns bounded versioned JSON:

```bash
build/native-recording/lingtu_recorder start \
  --root "$HOME/data/lingtu/recordings" \
  --prefix inspection \
  --seconds 600 \
  --product inspection \
  --product-session-id "$LINGTU_PRODUCT_SESSION_ID" \
  --dds on --camera off

build/native-recording/lingtu_recorder status \
  --root "$HOME/data/lingtu/recordings"

build/native-recording/lingtu_recorder stop \
  --root "$HOME/data/lingtu/recordings" \
  --timeout-ms 15000

build/native-recording/lingtu_recorder list \
  --root "$HOME/data/lingtu/recordings" \
  --limit 100

build/native-recording/lingtu_recorder manifest \
  --root "$HOME/data/lingtu/recordings" \
  --session-id inspection_20260804T010203Z_1234_abcd

build/native-recording/lingtu_recorder remove \
  --root "$HOME/data/lingtu/recordings" \
  --session-id inspection_20260804T010203Z_1234_abcd
```

Python does not scan recording directories, choose an active manifest, create
session IDs, or own the manager process. An invalid catalog, multiple active
sessions, an unsafe path, or an unhealthy active manager fails closed.

For an operator terminal, the explicit `record` command remains the foreground
entry and records the default DDS sensor set and native color-camera SHM stream
together:

```bash
build/native-recording/lingtu_recorder record \
  --output-dir /data/runs/field-001 \
  --product nav \
  --product-session-id "$LINGTU_PRODUCT_SESSION_ID" \
  --robot-id field-robot-01
```

`Ctrl+C` or `SIGTERM` stops both workers against one shared grace deadline. A
session is `completed` only when every required worker exits successfully and
its declared artifact is a non-empty regular file. Otherwise the session is
`failed` and its logs, temporary files, and manifest remain available for
diagnosis.

```text
field-001/
  session.json
  logs/
    dds.stdout.log
    dds.stderr.log
    camera_color.stdout.log
    camera_color.stderr.log
  dds/
    sensors.mcap
  camera_color.mcap
  camera_color/
    000000.mkv
    000001.mkv
```

The manifest is atomically replaced through `session.json.tmp` and records the
session state, manager PID, Product context, effective worker argv, selected DDS
topics, required evidence topics, logs, child status, and expected artifacts.
`status` is live-aware: an active manifest is
reported as stale with a nonzero exit status if its manager no longer exists or
no longer identifies the same executable, `record` command, and session path.
It does not start middleware:

```bash
build/native-recording/lingtu_recorder status /data/runs/field-001
```

A second operator terminal or a service wrapper can request a clean stop and
wait for the final manifest:

```bash
build/native-recording/lingtu_recorder stop /data/runs/field-001 \
  --timeout-ms 15000
```

`stop` validates the manager identity before sending `SIGTERM`; it never signals
a PID taken from an untrusted or stale manifest and never kills worker processes
directly. The foreground manager remains responsible for the shared stop
deadline, worker process groups, artifact checks, and final state transition.
Repeated stop calls observe an existing `stopping` state instead of escalating
the signal.

Recorder selection is explicit when a reduced capture is needed:

```bash
# Camera-only build and capture; CycloneDDS/idlc are not required.
LINGTU_RECORDING_BUILD_DDS=OFF bash scripts/build/build_native_recording.sh
build/native-recording/lingtu_recorder record \
  --output-dir /data/runs/camera-001 --dds off --camera on

# DDS-only capture.
build/native-recording/lingtu_recorder record \
  --output-dir /data/runs/dds-001 --dds on --camera off
```

The `record` command is deliberately foreground. The `start` command does not
implement a second state machine: it forks that same foreground manager and
waits for the authoritative manifest to become ready. Gateway is a stateless
command adapter, and ProductControl only checks the native root status before a
Product switch. `lingtu_dds_recorder` and `lingtu_camera_recorder` remain worker
binaries, not competing operator entry points. Recording is not started
implicitly by a Product and does not replace separately named ROS compatibility
tools.

Product context and capture policy are deliberately separate. `--product`
only records the active Product name and Product session ID in the session manifest; it
does not compile another Product policy inside the C++ tool. The explicit
`--dds-preset inspection-evidence-v1` capability records native task events,
navigation state, driver control state, goal status, operator-motion status,
global/local paths, final logical `cmd_vel`, inspection evidence results, and
the relevant sensor context. The task event, navigation state, driver state,
and localization odometry must contribute samples or the session fails.

Bind a complete recording to the same stable task identity exposed by Gateway:

```bash
lingtu_recorder record \
  --output-dir /data/recordings/inspection-task-123 \
  --product inspection \
  --product-session-id "$LINGTU_PRODUCT_SESSION_ID" \
  --dds-preset inspection-evidence-v1 \
  --inspection-task-id inspection-task-123
```

With `--inspection-task-id`, final MCAP publication additionally requires a
captured `TASK_ACCEPTED`, contiguous native event cursors, immutable map/route
identity, exactly one terminal fact, a final zero `cmd_vel` before that fact,
and a connected driver state accepting that exact output sequence. The task
event, output command, and driver acceptance must share the same native boot
identity at the terminal transition, so a stale pre-restart ACK cannot complete
a resumed task. A retained task may continue across a boot change only through
the native restart-paused checkpoint and an explicit resume with unchanged
task/map/route identity. An already recorded terminal replay does not create a
second terminal result. Failure or
queue loss leaves the `.mcap.tmp` evidence and prevents a clean session. The
task ID is also written to `session.json`.

An explicit `--dds-topic` selection replaces the preset selection. In ordinary
sensor capture, only selected preset-required streams remain required (raw IMU
and raw LiDAR, if selected). Quiet optional request/event/status streams can have
zero samples without invalidating otherwise useful data. A capture with no samples
at all still fails. Task-bound inspection evidence retains its mandatory SLAM,
driver, task timeline, and terminal-output requirements. The effective selection
and required topics remain visible in `session.json`.

### Selectable recording catalog

`lingtu_recorder topics --json` provides the compiled recording support catalog
without starting DDS, detecting publishers, or touching a recording directory.
It contains canonical topic, IDL type, UI group/label, replay policy, defaults,
and preset requirements. Gateway and Web consume this catalog rather than
maintaining another independent list. Catalog support is not proof a sensor is
currently publishing; completed MCAP summaries expose zero-count channels.

The recorder now includes `/robot/joint_states` (joint names, position in rad,
velocity in rad/s, estimated torque in Nm), camera depth and camera intrinsics,
platform motion-profile request/state, and relocalization request/response.
Go2 already publishes joint telemetry from SDK LowState; a platform without a
joint publisher contributes no samples. Battery, temperature, current, and foot
contact are not fields of this existing joint contract and are not fabricated.
Camera depth recording uses its actual typed DDS stream; it does not infer a
depth stream from the color SHM video path. Color video remains segmented MKV
plus `camera_color.mcap`, and committed segments are included in `session.json`
when recording stops so exports include actual video pixels. Failed sessions
retain a failed state and declare actual `.mcap.tmp` evidence rather than a
missing final filename; worker stdout/stderr logs are also declared artifacts.
These partial files remain incident evidence, not successful captures.

`lingtu_dds_player --info FILE --json` provides per-topic sample counts, total
count, time range, duration, type, and replay policy. `FILE --dry-run --json`
validates recorded native schemas/CDR without creating a DDS participant;
its `skipped_record_only` count identifies validated records that timed sensor
replay would not publish. The sample summary includes zero-count optional
channels instead of silently omitting them.
Command, motion-profile, task, path, and inspection evidence topics stay
record-only; selecting them for recording does not authorize actuator replay.
Only supported sensor inputs can be republished through the existing isolated
DDS player. Offline validation is not a Web timeline or a second simulation.

### Offline numeric curves

The existing player and short entry also provide a read-only numeric query:

```bash
ltplay --series SESSION_DIR --topic /robot/joint_states \
  --field 'position_rad[0]' --start 9 --end 9.2 --max-points 2000 --json
```

Omit `--field` to discover the mapped numeric fields and select the default
physical field. Results contain units, frame ID, the exact file log-time origin
as a nanosecond string, elapsed sample times, coverage and sample counts. A
window intersects the file's coverage; no overlap returns empty samples rather
than changing the origin. The mode returns before DDS participant creation.

Historical decoding and live observation share `message_preview.cpp`; their
field paths have one native definition. Existing mapped numeric contracts
include joint position/velocity/estimated torque, IMU, odometry, final velocity,
driver/navigation state, Float32, path endpoints, GNSS and camera intrinsics.
Event types with only a text preview do not yet expose arbitrary numeric IDL
fields. Preview arrays retain their first 32 entries; clouds/images expose
bounded metadata, while the original complete CDR remains recorded.

Default output is at most 2000 samples (`--max-points` accepts 16–10000).
Min/max reduction retains extrema and segment boundaries. Non-finite values
are null; gaps break lines rather than becoming zeros. Gap detection uses five
times the median positive log-time interval in the window, sampled from up to
4096 intervals, and describes display continuity rather than packet-loss proof.
Two indexed topic/window passes retain bounded output and preserve narrow
peaks, but still scan selected messages; hour-scale full-window latency has not
been qualified. Web uses this mode for a browser-only timeline and A/B viewer,
independently of timed DDS replay.

### Recorded camera and 3D scenes

```bash
ltplay --scene-info SESSION_DIR --json
ltplay --scene SESSION_DIR --start 0 --end 5 \
  --max-frames 32 --max-points 2048 --max-poses 2000 --json
lingtu_camera_player SESSION_DIR/camera_color.mcap --json
lingtu_camera_player SESSION_DIR/camera_color.mcap --json --segment 0
```

Scene queries are offline and return before DDS participant creation. The
native reader selects PointCloud2/Livox frames, odometry and recorded Path
messages through their existing typed contracts. It retains both log-relative
time and the source header timestamp, coordinate frame, original counts and
display sampling. Indexed windows are at most 30 seconds; all returned geometry
shares a bounded display budget. This affects visualization only, not recorded
messages or robot planning. A recorded path predecessor remains state until a
later path replaces or clears it. Different coordinate frames are not treated
as if already registered.

The Web viewer shares one playhead, playback rate and A/B time offset between
numeric curves, images, point clouds and trajectories. Buffered scene windows
avoid starting a native reader on every animation tick. Camera frames are
decoded with the existing FFmpeg runtime, selected at or before the cursor,
and report their source timestamp, segment-local frame number and media PTS.
Image/scene display sampling is distinct from the complete recorded payload.
Camera-only sessions use their first acquisition timestamp as the clock origin.

## Build and test

Windows supports the same controller, manager, DDS worker and MCAP completion
contract with `--camera off` (Gateway `camera=false`). Its platform process
adapter cooperatively stops the worker so MCAP receives its final records and
footer. Camera recording still uses Linux POSIX shared memory; Windows camera
requests explicitly fail rather than silently omit video.

The local Windows controller candidate is built under
`build/native-recording/windows-x64/Release`. Set `LINGTU_RECORDING_BIN` to its
`lingtu_recorder.exe` when using Gateway, and retain the selected CycloneDDS
runtime directory on `PATH`. Local start/status/stop and MCAP playback tests
prove this component only; a repeated inspection Product run has separate
readiness, physics, capture and terminal-stop acceptance.

```bash
bash scripts/build/build_native_recording.sh

# Camera-only build: no CycloneDDS package or idlc required.
LINGTU_RECORDING_BUILD_DDS=OFF bash scripts/build/build_native_recording.sh
```

The build uses CycloneDDS and `idlc` directly. It does not source or invoke a
ROS environment. On 2026-07-29 the isolated aarch64 build, typed CDR
round-trip, CLI record/info/verify/replay flow, and all 14 native tests passed
on the declared aarch64 CycloneDDS 0.8.2 runtime. That test uses an isolated DDS
domain and synthetic typed publisher/subscriber; real LiDAR, IMU, and camera
availability remain separate field acceptance gates. Any other CycloneDDS
version, including 0.10.x, must rerun the same native suite after a runtime
upgrade.

## DDS worker

```bash
build/native-recording/lingtu_dds_recorder \
  --output /data/runs/field-001.mcap \
  --seconds 60
```

The default generic set is `/lidar/raw_frame`, `/imu/raw`, `/slam/odometry`,
and `/slam/registered_cloud`. Additional arguments must belong to the compiled
recording allowlist. Product evidence topics such as task events, paths, final
logical velocity, and driver state are record-only: they can be captured and
validated but never enter the sensor replay allowlist.

An explicit `--idl` or `--dds-idl` path wins. Otherwise the tools resolve IDL
from `LINGTU_RECORDING_IDL`, then `LINGTU_REPO/src/message/idl/messages.idl`,
then the executable-relative repository layout, and finally the compiled path.

Supported DDS channels are:

| Logical topic | DDS topic | IDL type | Replay policy |
| --- | --- | --- | --- |
| `/tf` | `rt/tf` | `lingtu.dds.TFMessage` | replayable |
| `/tf_static` | `rt/tf_static` | `lingtu.dds.TFMessage` | replayable |
| `/lidar/raw_frame` | `rt/lidar/raw_frame` | `lingtu.dds.LivoxFrame` | replayable |
| `/imu/raw` | `rt/imu/raw` | `lingtu.dds.Imu` | replayable |
| `/slam/odom_prior` | `rt/slam/odom_prior` | `lingtu.dds.Odometry` | replayable |
| `/driver/odometry` | `rt/driver/odometry` | `lingtu.dds.Odometry` | replayable |
| `/slam/odometry` | `rt/slam/odometry` | `lingtu.dds.Odometry` | replayable |
| `/slam/state_at_scan` | `rt/slam/state_at_scan` | `lingtu.dds.Odometry` | replayable |
| `/slam/registered_cloud` | `rt/slam/registered_cloud` | `lingtu.dds.PointCloud2` | replayable |
| `/slam/map_observation` | `rt/slam/map_observation` | `lingtu.dds.MapObservation` | replayable |
| `/slam/map_cloud` | `rt/slam/map_cloud` | `lingtu.dds.PointCloud2` | replayable |
| `/gnss/fix` | `rt/gnss/fix` | `lingtu.dds.GnssFix` | replayable |
| `/gnss/odom` | `rt/gnss/odom` | `lingtu.dds.Odometry` | replayable |
| `/driver/control_state` | `rt/driver/control_state` | `lingtu.dds.DriverControlState` | record-only |
| `/nav/goal/status` | `rt/nav/goal/status` | `lingtu.dds.NavigationGoalStatus` | record-only |
| `/nav/state` | `rt/nav/state` | `lingtu.dds.NavigationState` | record-only |
| `/nav/operator_motion/status` | `rt/nav/operator_motion/status` | `lingtu.dds.OperatorMotionStatus` | record-only |
| `/nav/global_path` | `rt/nav/global_path` | `lingtu.dds.Path` | record-only |
| `/nav/local_path` | `rt/nav/local_path` | `lingtu.dds.Path` | record-only |
| `/nav/cmd_vel` | `rt/nav/cmd_vel` | `lingtu.dds.FinalVelocityCommand` | record-only |
| `/nav/inspection/task/event` | `rt/nav/inspection/task/event` | `lingtu.dds.InspectionTaskEvent` | record-only |
| `/nav/inspection/evidence/result` | `rt/nav/inspection/evidence/result` | `lingtu.dds.InspectionEvidenceResult` | record-only |
| `/gnss/status` | `rt/gnss/status` | `lingtu.dds.GnssStatus` | record-only |
| `/slam/localization_health` | `rt/slam/localization_health` | `lingtu.dds.Text` | record-only |
| `/slam/localization_quality` | `rt/slam/localization_quality` | `lingtu.dds.Float32` | record-only |
| `/nav/command/request` | `rt/nav/command/request` | `lingtu.dds.NavigationCommandRequest` | record-only |
| `/nav/command/ack` | `rt/nav/command/ack` | `lingtu.dds.NavigationCommandAck` | record-only |
| `/nav/operator_motion/control` | `rt/nav/operator_motion/control` | `lingtu.dds.OperatorMotionControl` | record-only |
| `/nav/operator_motion/sample` | `rt/nav/operator_motion/sample` | `lingtu.dds.OperatorMotionSample` | record-only |
| `/nav/operator_motion/ack` | `rt/nav/operator_motion/ack` | `lingtu.dds.OperatorMotionAck` | record-only |
| `/nav/inspection/task/request` | `rt/nav/inspection/task/request` | `lingtu.dds.InspectionTaskRequest` | record-only |
| `/nav/inspection/task/ack` | `rt/nav/inspection/task/ack` | `lingtu.dds.InspectionTaskAck` | record-only |
| `/nav/inspection/status` | `rt/nav/inspection/status` | `lingtu.dds.InspectionStatus` | record-only |
| `/nav/inspection/evidence/request` | `rt/nav/inspection/evidence/request` | `lingtu.dds.InspectionEvidenceRequest` | record-only |

Only channels marked `replayable` can be published by the player.
All control, command, motion, task, status, and evidence channels remain record-only.

## Validate and replay

```bash
build/native-recording/lingtu_dds_player --list-topics
build/native-recording/lingtu_dds_player --info field-001.mcap
build/native-recording/lingtu_dds_player field-001.mcap --dry-run
build/native-recording/lingtu_dds_player inspection-task-123.mcap \
  --dry-run --verify-inspection-task inspection-task-123
build/native-recording/lingtu_dds_player field-001.mcap --domain 84
```

`--list-topics` needs no MCAP file. `--info` reads only MCAP metadata and
statistics; neither mode creates a DDS participant or publishes samples.
`--list-topics` is the compiled support catalog, not live DDS discovery.

Replay defaults to isolated DDS domain 84. Domain 0 is rejected unless the
operator also passes `--allow-live-domain`. That flag does not bypass the
sensor topic allowlist. `--dry-run` checks MCAP structure, the embedded/local
IDL match, channel metadata, and each selected CDR envelope. It does not claim
generic semantic decoding of every record-only message body. A real replay
publishes sensor channels only and reports the number of record-only messages
it skipped.

`--verify-inspection-task` independently rechecks an already finalized MCAP
against the same task acceptance, cursor, immutable identity, unique terminal,
boot-matched zero-output, and driver-confirmation rules used before recorder
publication. It is accepted only together with `--dry-run`, so this evidence
audit cannot create a DDS participant or republish path, velocity, control, or
task facts. Success prints the verified `task_id`, native terminal state, and
confirmed output sequence; failure exits nonzero with the violated evidence
rule.

## Camera segment format

Camera sessions use MCAP profile `lingtu.camera.v1`. Each JSON/JSON Schema
message indexes one completed video segment; video bytes are not embedded in
MCAP. The record contains the relative path, codec, source pixel encoding,
geometry, acquisition-time range, first/last source sequence, frame/drop
counts, byte size, CRC32, frame id, and the CameraInfo snapshot (intrinsics,
distortion, and depth scale). Segment and index files are written through a
`.tmp` path and renamed only after completion. Segment bytes are flushed before
promotion; POSIX builds also sync the containing directory.

The portable offline player never publishes DDS or writes camera SHM:

```bash
build/native-recording/lingtu_camera_player run/camera_color.mcap --list
build/native-recording/lingtu_camera_player run/camera_color.mcap --verify
build/native-recording/lingtu_camera_player run/camera_color.mcap \
  --extract-window 1750000000000000000 1750000005000000000 \
  --output /tmp/camera-window
```

Extraction copies whole overlapping segments after size/CRC verification. It
does not re-encode, decode, or inject frames into a live robot graph. Segment
paths are constrained to the session tree, including after symlink resolution.

On Linux, the explicit camera recorder reads the existing color SHM ring and
feeds RGB8/BGR8 frames to a separately spawned FFmpeg process using an argv
vector and stdin pipe (never a shell command string):

```bash
build/native-recording/lingtu_camera_recorder \
  --output-dir /data/runs/field-001 \
  --color-shm /lingtu_camera_color \
  --segment-seconds 5 \
  --codec libx264
```

The initial recorder is color-only. Depth16 is deliberately not sent through
lossy H.264; a later depth lane must use a lossless rawpack/PNG/Zstd-style
representation and preserve `depth_scale` per frame or chunk.

The camera MCAP index is promoted only after a clean recorder stop. Completed
MKV segments survive an interrupted run, but rebuilding their missing timeline
index is not implemented in this first slice. New segment indexes also retain
each encoded frame's acquisition timestamp and nominal encoding rate. JSON
catalog inspection returns compact descriptors; `--segment N` loads the chosen
segment's timestamp array. Existing indexes without it remain readable with
explicitly estimated frame timing, without claiming exact per-frame acquisition
synchronization.

The encoder is an adapter behind `CameraSegmentEncoder`; the portable core and
offline player do not link FFmpeg or GStreamer. `ffmpeg` with `libx264` is the
supported generic software recording path: it needs an FFmpeg build that
provides `libx264`, but it has no accelerator SDK or hardware-encoder
requirement. A deployment that chooses another FFmpeg codec does so explicitly
with `--codec` after validating its quality, storage cost, and output playback.
Before opening the camera SHM ring, the recorder encodes one synthetic frame
with that exact executable/codec/`yuv420p`/Matroska combination. It fails
closed if the profile is unavailable. Any observed source-frame drop also gives
the camera worker exit code `4`, so the session supervisor cannot mark that
recording as clean.

On Linux, CMake registers `native_camera_software_ffmpeg` when `ffmpeg` is on
`PATH`. That integration test feeds frames through the same encoder adapter,
then uses FFmpeg to decode the completed MKV. It is intentionally an external
runtime test rather than a linked media dependency.

## Current boundary

The DDS path handles LiDAR/IMU/odometry/point-cloud channels plus selected
low-bandwidth Product evidence. Camera recording
is deliberately a separate SHM-to-segment path and never turns the primary
camera payload into a DDS topic. DDS recorder capture keeps raw CDR. DDS player
accepts only unkeyed, little-endian XCDR1 payloads and republishes their raw CDR
with CycloneDDS `dds_forwardcdr`, preserving the recorded source timestamp.
Cyclone performs conversion to the writer's generated type. The player does not
link the non-exported `dds_stream_countops` or `dds_stream_read_sample`
internals that are unavailable in the pinned 0.8.2 shared library. Record-only
control and evidence channels are never published. Inspection verification
parses only its three declared evidence types with a bounded offline XCDR1
reader, so `record verify` remains non-publishing.

One requested `task_id` binds MCAP publication to the captured native task
facts. The Gateway inspection path already starts an evidence session before
submitting a new task, stores that session binding in its task journal, and
stops the exact session on a native terminal event or startup rollback.
A restored terminal event can retry an interrupted recording stop. This does
not make MCAP a replacement for native task checkpoints or the Gateway journal.
Ordinary Product activation does not start recording automatically, and scene
sensor recording remains separate from task-owned inspection evidence.
The existing restart reconciliation paths still require end-to-end acceptance;
native MuJoCo also needs a record-to-replay terminal-equivalence scenario.

The remaining product gaps are explicit:

- DDS MCAP rotation by elapsed time or file size and an explicit retention
  policy. The existing disk watermark stops capture; it does not rotate or
  remove recordings.
- crash recovery for an interrupted `.mcap.tmp`, including reindex/salvage;
- typed live discovery and an explicit record-all policy (session listing and
  terminal-session removal are available through the root catalog commands);
- end-to-end qualification of task-bound recording across Gateway/native
  restarts, plus record-to-replay result and terminal-state equivalence;
- real target-platform sensor-stream and camera acceptance in addition to the synthetic
  typed DDS test.
