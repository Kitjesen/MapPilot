# Native Recording and Replay

Status: product operator guide for the native C++ recording path.

LingTu records field data without ROS. The operator entry point is the C++
`lingtu_recorder` session
manager plus native CycloneDDS and camera workers. DDS samples and the camera
timeline use MCAP. Color image payloads stay on POSIX shared memory and are
written to segmented MKV files.

This guide is the source of truth for operator behavior. The implementation
catalog and developer details remain in
[`src/native/recording/README.md`](../../src/native/recording/README.md).

## One-minute workflow

Start a foreground DDS recording and stop it cleanly with Ctrl-C:

```bash
build/native-recording/lingtu_recorder record \
  --output-dir /data/recordings/run-001 --dds on --camera off
```

Add the color camera only when video is required:

```bash
build/native-recording/lingtu_recorder record \
  --output-dir /data/recordings/run-001 --dds on --camera on
```

Use a second terminal for session operations:

```bash
build/native-recording/lingtu_recorder status /data/recordings/run-001
build/native-recording/lingtu_recorder stop /data/recordings/run-001
build/native-recording/lingtu_dds_player --list-topics
build/native-recording/lingtu_dds_player \
  --info /data/recordings/run-001/dds/sensors.mcap
build/native-recording/lingtu_dds_player \
  /data/recordings/run-001/dds/sensors.mcap --dry-run
build/native-recording/lingtu_dds_player \
  /data/recordings/run-001/dds/sensors.mcap --domain 84
```

`lingtu_recorder` and `lingtu_dds_player` do not source a ROS
environment. `replay` is the long alias for `play`. Replay defaults to
isolated DDS domain `84`, so a normal verification run cannot command the
live robot.

## Install and binary lookup

The field release contains five sibling C++ tools:

```text
/opt/lingtu/current/bin/
  lingtu_recorder
  lingtu_dds_recorder
  lingtu_dds_player
  lingtu_camera_recorder
  lingtu_camera_player
```

Build them in a checkout with:

```bash
bash scripts/build/build_native_recording.sh
```

The managed Gateway boundary accepts an exact
`LINGTU_RECORDING_BIN`; otherwise it resolves binaries in
this order:

1. `LINGTU_RECORDING_BIN_DIR`;
2. `/opt/lingtu/current/bin`;
3. the checkout's `build/native-recording`;
4. `PATH`.

The release package installs and verifies all five executables. The supported
field path is C++ and CycloneDDS. Rust may be useful for a future standalone
offline inspector, but there is no second Rust recorder or player today.

## Session location and layout

Without an explicit path, sessions are created under
`$HOME/data/lingtu/recordings/<UTC-timestamp>-<pid>`.
`LINGTU_RECORDING_ROOT` changes that root. A caller can also supply the
session directory as the first argument:

```bash
build/native-recording/lingtu_recorder record \
  --output-dir /data/recordings/run-001 --dds on --camera on
```

A complete DDS-plus-camera session has this layout:

```text
run-001/
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

`session.json` is the authoritative session projection. It records the
manager identity, state, Product context, selected and required topics, worker
arguments, logs, artifacts, and startup storage observation. It is replaced
atomically through `session.json.tmp`.

The DDS file uses profile `lingtu.dds.v1`: raw CDR samples, self-contained OMG
IDL schemas, indexed MCAP chunks, timestamps, CRCs, and a durable footer. The
camera index uses profile `lingtu.camera.v1`; it stores segment timing,
geometry, codec, frame/drop counts, size, CRC, and camera calibration metadata.
Video bytes stay in the MKV segments instead of inflating DDS or MCAP messages.

## Storage admission

The C++ session manager performs a startup preflight before it creates the
session or starts a worker. `--min-free-gib` defaults to `5`, meaning at
least 5 GiB must be available on the target filesystem:

```bash
build/native-recording/lingtu_recorder record \
  --output-dir /data/recordings/run-001 --min-free-gib 20
```

A value of `0` disables this admission threshold:

```bash
build/native-recording/lingtu_recorder record \
  --output-dir /data/recordings/run-001 --min-free-gib 0
```

Disabling it is an explicit diagnostic choice, not a safe field default. The
manifest preserves the configured floor and the free space observed at start.

This batch provides startup preflight only. It does not yet provide a running
quota, reserved-space watermark, or automatic stop before the disk becomes
full. Long runs still require external disk monitoring.

## Topic selection

The default DDS capture is intentionally small:

- `/lidar/raw_frame`;
- `/imu/raw`;
- `/slam/odometry`;
- `/slam/registered_cloud`.

Select a different compiled topic set by repeating `--dds-topic`:

```bash
build/native-recording/lingtu_recorder record \
  --output-dir /data/recordings/imu-odom \
  --dds-topic /imu/raw --dds-topic /slam/odometry
```

`build/native-recording/lingtu_dds_player --list-topics` prints the compiled support catalog and replay
policy. It is not live DDS discovery. Unknown topics fail validation; they are
not guessed from the wire.

For product evidence, the native manager also exposes the explicit
`inspection-evidence-v1` preset. It records the task events, navigation state,
paths, final logical velocity, driver acceptance, localization, and relevant
sensor context required to audit one inspection task. Detailed topic and IDL
mappings are maintained in the
[developer catalog](../../src/native/recording/README.md#dds-worker).

## Replay policy and live-robot safety

The catalog separates data by what the player may publish:

| Policy | Meaning | Examples |
| --- | --- | --- |
| `replayable` | Sensor input that may be republished by the native player | LiDAR, IMU, odometry, registered cloud, GNSS |
| `record-only` | Evidence may be inspected and verified but is skipped by real replay | task events, paths, status, final `cmd_vel`, driver ACK |
| never-replay guarantee | Every command, control, motion, task, status, and evidence channel remains `record-only` | goal/control requests, operator motion, inspection requests |

“Never replay” is a product safety guarantee over the `record-only` catalog,
not a third wire encoding. Adding a new control or evidence type must default
to `record-only` and pass the replay-safety contracts before release.

The player uses domain `84` by default. Live domain `0` is rejected unless
the native player receives its separate explicit live-domain opt-in. Even that
opt-in cannot bypass the sensor-only allowlist. `lingtu_dds_player --dry-run` is safer than
live playback: it checks MCAP structure, IDL/channel identity, and CDR
envelopes without publishing DDS. Generic `--dry-run` is an integrity
check, not semantic decoding of every record-only body. Supplying an inspection
`task_id` additionally parses the declared task-event, final-output, and
driver-confirmation bodies and applies the task evidence rules.

## Failure truth and recovery boundary

A session is `completed` only after every required worker exits cleanly and
every required artifact is a non-empty regular file.

- DDS writes `sensors.mcap.tmp` first and promotes it only after a complete
  MCAP footer and durable flush.
- A DDS queue overflow or missing required evidence refuses clean publication.
- Camera source-frame loss makes the camera worker fail and prevents a clean
  session.
- Completed MKV segments survive an interrupted camera run, while an incomplete
  camera MCAP index is not promoted.
- A crash or verification failure leaves logs and `.mcap.tmp` artifacts for
  diagnosis; their presence is not proof of a valid recording.
- `lingtu_recorder status` detects a stale manager identity instead of trusting a PID
  copied from an old manifest.
- `lingtu_recorder stop` signals only the verified session manager. The manager owns
  child shutdown, the shared deadline, artifact checks, and the final state.

Automatic `reindex/recovery` for interrupted DDS MCAP or camera timeline files
is planned, not yet implemented. Preserve the entire session directory when
raising an incident.

## Runtime ownership

Recording follows the same product boundaries as the rest of LingTu:

| Component | Recording responsibility |
| --- | --- |
| Product | Declares that recording binaries/capabilities are available; does not start a session implicitly |
| ProductControl | Blocks Product switching while native recording is active; does not own or auto-stop the recording session |
| Internal SystemdRunner | Starts only processes declared by the RunPlan; does not infer recording policy |
| C++ `lingtu_recorder` | Sole authority for the recording root, active-session selection, session ID, workers, manifest, stop transaction, and artifact verdict |
| Gateway | Stateless HTTP-to-command adapter for native start/status/stop; it does not scan manifests or launch ROS/Bash |
| Recorder workers | Capture DDS CDR or camera SHM; they do not own Product lifecycle |

Automatic task-evidence binding is not yet a default Product behavior. An
operator can explicitly bind an inspection `task_id`, but ProductControl does
not automatically start recording before every task or stop it at the terminal
state. Evidence recording failure must eventually be reported separately from
navigation outcome: it must not forge a navigation failure, and it must not be
hidden after a successful task.

## Gateway API

The canonical SDK and dashboard API delegates to the native C++ root control
contract:

- `POST /api/v1/recordings/start`;
- `GET /api/v1/recordings/status`;
- `POST /api/v1/recordings/stop`.

These routes invoke `lingtu_recorder start|status|stop --root`. The implementation
does not invoke Bash, source ROS, run rosbag2, scan `session.json` in Python, or
spawn a Python-owned recorder. It starts DDS-only capture by default.
`lingtu_recorder record` remains the foreground operator interface and supports
explicit camera recording and advanced native options.

The dashboard exposes recording from the **Scene** toolbar, not as a separate
product page. Its start request has only bounded, product-level choices:

```json
{
  "duration": 600,
  "prefix": "web",
  "capture_profile": "sensors",
  "camera": false,
  "minimum_free_gib": 5
}
```

The Scene panel starts only the `sensors` profile. Task-bound inspection code
may start the `evidence` profile through the same Gateway endpoint, but must
provide the active inspection `task_id`; the Gateway rejects unbound evidence
requests. It maps both profiles to compiled native presets, keeps DDS enabled,
and does not expose raw topics, domain IDs, SHM names, or worker paths. Camera
selection is admitted only when the native camera recorder can start; otherwise
the whole session rejects rather than silently recording less data.

The Scene recording panel uses the same native catalog for history and files:

- `GET /api/v1/recordings?limit=100` lists validated sessions (newest first);
- `GET /api/v1/recordings/{session_id}` returns the public manifest for one
  terminal session;
- `GET /api/v1/recordings/{session_id}/files/{artifact_path}` downloads one
  artifact declared by that manifest;
- `DELETE /api/v1/recordings/{session_id}` removes one terminal session.

The Gateway never enumerates the recording directory, accepts an arbitrary
filesystem path, or treats a client-supplied filename as authoritative. The
native manager validates the session ID and manifest first; downloads are
limited to declared regular files below that session directory, and an active
recording cannot be deleted. The API is therefore a file-management adapter,
not a second recording database. Automatic retention and quota enforcement are
not enabled yet; keep the existing startup free-space check and external disk
monitoring for long runs.

## rosbag2 behavior we adopt

LingTu learns the mature operator behavior of rosbag2 without linking
`rclcpp`, `rmw`, rosbag2 transport, or its storage plugins into the field
runtime.

| Capability | LingTu status | Product decision |
| --- | --- | --- |
| Short foreground record/play workflow | implemented | Native C++ commands with explicit exit status |
| Explicit topic selection | implemented | Repeated `--dds-topic` against a compiled typed catalog |
| Metadata inspection and topic catalog | implemented | `info` reads MCAP; `topics` is static and middleware-free |
| Session status, stop, and artifact verification | implemented | One durable C++ manager and manifest |
| Rate-controlled sensor replay | implemented | Isolated domain and sensor-only allowlist |
| Camera sidecar timeline | implemented | SHM to segmented MKV, indexed by MCAP |
| Minimum free-space admission | implemented | Startup preflight, default 5 GiB |
| Automatic topic discovery / record-all | planned | Requires bounded discovery, type validation, and resource policy |
| Live topic discovery | planned | Must remain observability-only until typed admission is designed |
| Include/exclude regex filters | planned | Must compile to a visible effective topic set |
| DDS split/rotation by time or size | planned | Requires multi-file metadata and player ordering |
| Compression | planned | Must be measured on S100P before adding a dependency |
| Recorder pause/resume | planned | Must preserve a visible timeline gap and session state |
| Snapshot recording | planned | Requires bounded memory and explicit trigger semantics |
| Reindex/recovery | planned | Must never promote unverifiable partial evidence |
| Convert | planned | Offline transform with provenance and output verification |
| Runtime disk watermark / running quota | planned | Separate from the implemented startup check |
| QoS override policy | planned | Must be explicit and recorded in metadata |

The table is a roadmap, not a command reference. In particular, record-all,
split, pause, snapshot, reindex, convert, and live discovery are not currently
available native tool commands.

## Acceptance checklist

Before relying on a field recording:

1. `lingtu_recorder status` reports the intended session and Product context.
2. The effective topic list contains the required sensor/evidence channels.
3. Free-space admission passed for the target filesystem.
4. Ctrl-C or `lingtu_recorder stop` leads to a terminal manifest.
5. `lingtu_dds_player FILE --dry-run` exits successfully.
6. `lingtu_dds_player --info FILE` reports the expected channels and time range.
7. Camera sessions contain a verified `camera_color.mcap` and referenced MKV
   segments.
8. Replay is performed on domain `84` unless an approved live-domain test
   explicitly says otherwise.

A successful navigation or inspection task does not by itself prove that its
recording is complete. The session manifest and verifier are the recording
truth.

## Primary references

- [ROS 2 rosbag2 repository and command behavior](https://github.com/ros2/rosbag2)
- [rosbag2 C++ recorder options](https://github.com/ros2/rosbag2/blob/rolling/rosbag2_transport/include/rosbag2_transport/record_options.hpp)
- [rosbag2 C++ player options](https://github.com/ros2/rosbag2/blob/rolling/rosbag2_transport/include/rosbag2_transport/play_options.hpp)
- [MCAP format specification](https://mcap.dev/spec)
- [MCAP C++ reference](https://mcap.dev/reference/cpp/)
- [Eclipse Cyclone DDS documentation](https://cyclonedds.io/docs/cyclonedds/latest/)

Behavioral study of rosbag2 does not introduce a ROS runtime dependency. Direct
source reuse would require license and notice review; rosbag2 is Apache-2.0.
The vendored MCAP C++ library is used under its MIT license.
