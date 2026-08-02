# Native Recording and Replay

Status: product operator guide for the native C++ recording path.

LingTu records field data without ROS. The operator entry point is
`scripts/lingtu`; the implementation is the C++ `lingtu_recorder` session
manager plus native CycloneDDS and camera workers. DDS samples and the camera
timeline use MCAP. Color image payloads stay on POSIX shared memory and are
written to segmented MKV files.

This guide is the source of truth for operator behavior. The implementation
catalog and developer details remain in
[`src/native/recording/README.md`](../../src/native/recording/README.md).

## One-minute workflow

Start a foreground DDS recording and stop it cleanly with Ctrl-C:

```bash
scripts/lingtu record
```

Add the color camera only when video is required:

```bash
scripts/lingtu record --camera
```

Use a second terminal for session operations:

```bash
scripts/lingtu record status /data/recordings/run-001
scripts/lingtu record stop /data/recordings/run-001
scripts/lingtu record info /data/recordings/run-001
scripts/lingtu record topics
scripts/lingtu record verify /data/recordings/run-001
scripts/lingtu play /data/recordings/run-001
```

`record` and `play` are short native commands; they do not source a ROS
environment. `replay` is the long alias for `play`. Replay defaults to
isolated DDS domain `84`, so a normal verification run cannot command the
live robot.

## Install and binary lookup

The field release contains five sibling C++ tools:

```text
/opt/lingtu/current/build/native-recording/
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

The managed Gateway/ProductControl boundary accepts an exact
`LINGTU_RECORDING_BIN`; otherwise it and `scripts/lingtu` resolve binaries in
this order:

1. `LINGTU_RECORDING_BIN_DIR`;
2. `/opt/lingtu/current/build/native-recording`;
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
scripts/lingtu record /data/recordings/run-001 --camera
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
scripts/lingtu record /data/recordings/run-001 --min-free-gib 20
```

A value of `0` disables this admission threshold:

```bash
scripts/lingtu record /data/recordings/run-001 --min-free-gib 0
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
scripts/lingtu record /data/recordings/imu-odom   --dds-topic /imu/raw   --dds-topic /slam/odometry
```

`scripts/lingtu record topics` prints the compiled support catalog and replay
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
opt-in cannot bypass the sensor-only allowlist. `record verify` is safer than
live playback: it checks MCAP structure, IDL/channel identity, and CDR
envelopes without publishing DDS. Generic `record verify` is an integrity
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
- `record status` detects a stale manager identity instead of trusting a PID
  copied from an old manifest.
- `record stop` signals only the verified session manager. The manager owns
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

The old paths remain deprecated compatibility aliases only:

- `POST /api/v1/bag/start`;
- `GET /api/v1/bag/status`;
- `POST /api/v1/bag/stop`.

Both routes invoke `lingtu_recorder start|status|stop --root`. The implementation
does not invoke Bash, source ROS, run rosbag2, scan `session.json` in Python, or
spawn a Python-owned recorder. It starts DDS-only capture by default.
`scripts/lingtu record` remains the foreground operator interface and supports
explicit camera recording and advanced native options.

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
available `scripts/lingtu` subcommands.

## Acceptance checklist

Before relying on a field recording:

1. `record status` reports the intended session and Product context.
2. The effective topic list contains the required sensor/evidence channels.
3. Free-space admission passed for the target filesystem.
4. Ctrl-C or `record stop` leads to a terminal manifest.
5. `record verify` exits successfully.
6. `record info` reports the expected channels and time range.
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
