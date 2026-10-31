# Recording and replay: evidence before volume

Reviewed against upstream documentation on 2026-10-25. This note distinguishes
implemented local interfaces from field acceptance and future work. Operational
commands belong in [operations](../operations.md); the compiled native topic
catalog is authoritative for the supported data streams.

## What a useful recording must answer

A navigation incident needs a chain of observations, decisions, commands and
feedback. A point cloud and an estimated trajectory alone cannot explain why
the robot stopped, whether the driver accepted a command, or whether a leg was
moving. Record these boundaries without creating another runtime owner:

| Boundary | Existing source | What it can establish |
| --- | --- | --- |
| Sensor input | Raw LiDAR frames, IMU, optional camera/depth and GNSS | Input to the estimator, with the source and receive timestamps preserved |
| Robot feedback | Joint states, driver odometry and control state | Reported joint position/velocity/estimated torque and command acceptance; not an independent ground truth |
| Localization | SLAM odometry, scan-time state, health/quality, TF and relocalization messages | Estimated pose, coordinate relationships and state transitions |
| Planning and execution | Global/local paths, navigation state, goal status, final logical velocity | Planned route and output at each boundary; neither a path nor a zero command proves physical execution |
| Operator and task | Operator samples/claims/ACKs, navigation request/ACK, inspection events/evidence | Who requested an action and how the native runtime reported it |
| Reproduction context | Product, Product session, robot identity, selected streams and native worker arguments | The run that produced the data; this does not yet replace a full configuration/calibration snapshot |

For Go2, `/robot/joint_states` already originates in the native driver. It
contains joint names, positions, velocities and estimated efforts. Adding its
recording binding does not add a new device reader. The compiled topic catalog
also supports other robots publishing the same contract; it does not prove
that Thunder or another platform currently publishes it. Battery, motor
temperature/current, foot contact and commanded joint targets must not be
advertised as captured until their real adapters publish corresponding typed
messages.

## Upstream references and the design decisions they support

- [eProsima DDS Record & Replay 1.5.3 configuration](https://dds-recorder.readthedocs.io/en/latest/rst/recording/usage/configuration.html)
  is the closest transport reference: topic selection, stored types, receive
  versus publish timestamps, size limits and rotation. LingTu keeps its existing
  CycloneDDS/C++ implementation and adopts the explicit selection and evidence
  boundaries; this review does not introduce Fast DDS or its DynamicTypes stack.
- [rosbag2 upstream README](https://github.com/ros2/rosbag2/blob/rolling/README.md)
  documents selected topics, recording controls, split files, loss reporting,
  snapshots and playback controls. These are separate capabilities: a stop
  button is not pause, free-space stopping is not retention, and metadata
  inspection is not timed playback. LingTu remains ROS-free.
- [MCAP specification](https://mcap.dev/spec) and
  [format registry](https://mcap.dev/spec/registry) define schema/channel/message
  separation, receive/publish time and indexed data. LingTu uses `omgidl` plus
  `cdr`; it does not convert native DDS messages to an invented JSON envelope.
- [Foxglove playback](https://docs.foxglove.dev/docs/visualization/playback) and
  [comparison mode](https://docs.foxglove.dev/docs/visualization/comparison-mode)
  motivate a common timeline, state transitions and distinct before/after
  sources. Its [OMG IDL support](https://docs.foxglove.dev/docs/getting-started/custom/custom-schema-encodings)
  matches LingTu's encoding family. Successful loading, schema decoding and
  useful 3D visualization still require an actual compatibility test and field
  mappings; a `.mcap` extension alone proves none of these.
- [Spot 5.2 Data Buffer](https://dev.bostondynamics.com/docs/concepts/data_buffer_overview.html)
  combines internal state, API events and acquired data, with time-range
  downloads and bounded persistent storage. Its
  [robot-state schema](https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/robot_state.proto)
  is a useful completeness reference for joint, power, fault and contact
  feedback. LingTu must first expose each actual platform source.
- [DimOS recording](https://docs.dimensional.org/usage/recording/)
  connects stream selection, summaries and replacement of live inputs for
  replay. Its documented experimental native engine excludes DDS. We borrow
  the workflow rather than replace LingTu's existing DDS/MCAP format or add a
  second storage engine.

## Implementation boundaries

### Local upstream source checkouts

The six upstream reference families were cloned for source study on
2026-10-25. They live under `third_party/research_recording/`, which is ignored
by the LingTu repository and excluded from native releases. No packages,
submodules, models, ROS installations or external services were installed.

| Checkout | Source revision reviewed | Reading focus |
| --- | --- | --- |
| `dds-record-replay` | `fa89157`, 2026-10-21 | `ddsrecorder/src/cpp/tool/DdsRecorder.cpp`; monitoring, topic filtering and MCAP handler ownership |
| `rosbag2` | `910c07f`, 2026-10-22 | `rosbag2_transport/src/rosbag2_transport/recorder.cpp`; separate transport/recorder loss, clean closure; `rosbag2_cpp` circular snapshot cache |
| `mcap` | `2e658c5`, 2026-10-23 | C++ reader/writer and format schemas, chunk/index separation |
| `foxglove-studio` | `a8a589b`, 2024-03-11 | Current public checkout contains README only; not the current commercial application source |
| `foxglove-sdk` | `b71f883`, 2026-10-23 | `rust/foxglove/src/lib.rs` and channel/sink/WebSocket modules; shared channel metadata for live and file consumers |
| `dimos` | `d9793f1`, 2026-10-24 | `dimos/memory/tap.py`, `replay.py`; typed stream selection, writer queue and shared replay anchor |
| `spot-sdk` | `04537dd`, 2026-09-10 | `python/bosdyn-client/src/bosdyn/client/data_buffer.py`, BDDF reader/writer and robot-state proto; events, comments and schema-based signal ticks |
| `lichtblick` | `d64c601`, 2026-10-22 | Additional public GUI reference: `packages/suite-base/src/components/TopicList/` and MCAP iterable sources |

Foxglove Studio's public repository is archived and the checked-out revision
does not contain its GUI implementation. The additional
[Lichtblick source](https://github.com/lichtblick-suite/lichtblick) supplies a
public topic-browser implementation for study. It is a separate community
project, not the latest Foxglove commercial source. Spot SDK contains public
clients and file-format utilities, not the robot's internal data-buffer server.

The studied source is kept separate from production. DDS Record & Replay,
rosbag2 and DimOS use Apache-2.0; MCAP and Foxglove SDK use MIT; Lichtblick uses
MPL-2.0; Spot SDK has its own SDK license. This iteration implements LingTu's
existing component and transport contracts without copying these projects'
implementation files into the runtime.

Concrete lessons applied to the data workbench:

- Keep topic identity, schema and replay policy in the native catalog; expose
  the same identity to live inspection and recording selection.
- Give live inspection its own read-only subscription. Its sample age, observed
  rate and serialized-payload throughput describe messages actually received,
  not configured sensor frequency or full wire bandwidth.
- Keep UI previews bounded for clouds/images/arrays while DDS recording retains
  the complete selected CDR message. Color video remains encoded video rather
  than a lossless copy of the original RGB stream.
- Let topic inspection, capture selection and saved-session review share one
  compact workbench. Opening a field inspector does not start recording or
  change the selected capture streams.
- Preserve transport-loss, recorder-queue loss and display sampling as distinct
  facts. The new live observer cannot establish complete recording coverage.

## Runtime and operator boundaries

The native `lingtu_recorder` owns the topic catalog, session manifest and worker
lifecycle. Gateway adapts it to HTTP; Web obtains its choices from this catalog
and forwards explicit topic selections. It must distinguish compiled support
from live publisher availability and requested topics from messages actually
received. A selected stream is not automatically a mandatory event: no
relocalization request during an ordinary capture is valid. The selected
mandatory sensor inputs and task-bound inspection evidence retain their
required-sample checks. An entirely empty capture cannot become a completed
recording.

The DDS MCAP contains typed messages. Color video uses finalized MKV segments
and a separate MCAP timeline. The native manifest must enumerate finalized
segments so session export includes the actual pixels rather than only their
index. The current Web archive includes the public manifest and declared
artifacts, without discovering unrelated files through a second Python catalog.
Terminal native manifests include worker stdout/stderr. Failed sessions declare
retained `.mcap.tmp` files instead of nonexistent expected final files, so their
export keeps both the partial input and the failure logs. A readable temporary
file remains failed evidence, not a completed capture.

The existing player has three distinct uses:

1. `--info`: offline topic/count/container-time inspection.
2. `--dry-run`: offline container/schema/CDR-envelope checks; this is not full
   semantic validation of every recorded message body.
3. Timed sensor replay: selected sensor inputs are republished in isolated DDS
   domain 84. Control, velocity, task and mode-switch requests remain
   record-only. Logged actuator commands are evidence, not replay instructions.

Saved-file inspection and offline validation do not start a DDS participant.
Live Web inspection uses one disposable native read-only participant, cached
through Gateway. Opening the live tab does not start recording. The observer
reports actual receive-window frequency, serialized-payload throughput, age and
bounded values; it releases its subscriptions when unused or the Host stops.
The recorded-data inspector uses a browser-only numeric timeline with seeking,
playback speed, a shared cursor and optional A/B comparison. It does not replace
live Product sensor producers or recompute algorithms. Neither file validation
nor moving this timeline establishes a completed navigation replay.

The public `ltplay` entry uses the same compiled player source. It resolves a
session ID, directory or file and defaults to raw estimator inputs when they
are present. Native CMake and release packaging include both `ltplay` and
`lingtu_dds_observer`; sourcing the existing field `runtime-env.sh` adds the
release's `bin` directory to the operator PATH. Explicit topic selection and
the established long player name remain supported.

### Numeric inspection and comparison references

The implementation follows the official Foxglove
[Plot](https://docs.foxglove.dev/docs/visualization/panels/plot),
[Playback](https://docs.foxglove.dev/docs/visualization/playback) and
[Comparison mode](https://docs.foxglove.dev/docs/visualization/comparison-mode)
interaction documentation reviewed on 2026-10-25. The particular lessons and
their implementation boundaries are:

| Reference | Applied in LingTu | Boundary |
| --- | --- | --- |
| Foxglove Plot | Select a numeric message field, display its unit, inspect a value at a time cursor | Numeric fields exposed by native typed previews; no FoxQL/expression engine or arbitrary dynamic schema interpreter |
| Foxglove Playback | Play/pause, rate, seeking and a zoomed time window | Browser inspection only; it does not drive DDS replay or robot commands |
| Foxglove Comparison mode | Keep A/B sources distinct, align their elapsed starts, adjust B time offset and reset alignment | Two sources; no automatic trajectory registration or inference that a smaller difference means better accuracy |
| Lichtblick Plot | Study missing-value line breaks, source legend values and receive/header-time distinctions | Fresh React/SVG components using existing LingTu tokens; no Chart.js/MUI code or dependency imported |
| MCAP indexed reader | Query one topic and log-time window, retain file origin, return peak-preserving min/max samples | Complete messages stay in MCAP; the chart is a bounded view rather than a lossless export |
| DimOS replay | Use one explicit time relationship across viewed streams | LingTu retains its native DDS/MCAP path; no SQLite recorder or Python replay scheduler copied |

Both sources start at elapsed zero by default, regardless of their recording
dates. A positive B offset moves B later. Negative offsets are shown on the
shared elapsed-time axis. Values are never vertically shifted or scaled to
improve apparent agreement. The log-time origin remains a nanosecond string;
it is not converted into a JavaScript floating-point epoch timestamp.

Live numeric trends sample distinct received updates from the browser's latest
message snapshots. They are useful for immediate inspection but do not include
every high-rate sensor message. Historical series are decoded from the stored
typed CDR messages instead. The same native preview mapping defines units and
fields for both; booleans and text are not coerced into numeric curves.

Missing or non-finite values remain gaps. A temporal-gap marker describes
observed spacing in the selected series, not proof of transport packet loss.
The current segmentation threshold is five times the median positive log-time
interval, estimated from up to 4096 intervals in the selected window. It only
breaks the displayed line; it does not reject capture or affect navigation.
Min/max reduction preserves visible extrema, but a reduced cursor value is
still a retained sample rather than a reconstruction of the full waveform.
The cursor uses the preceding retained sample inside a covered segment and
does not interpolate across gaps or extrapolate outside coverage. Indexed
reads avoid unrelated topics/time windows, but min/max extraction still scans
the selected messages. Hour-scale full-window query latency has not been
qualified; no multiresolution index or precomputed result cache is claimed.
Comparing positions from different frames or joint array indices from different
robots requires matching their meaning; a field name alone is not ground truth.

## What remains before a fleet incident workflow is complete

Prioritize by the question the feature answers, rather than record everything:

1. **Reproduce the same estimator input.** Preserve effective configuration,
   calibration and map identity with the recording, and define the relationship
   between hardware time, DDS source time and recorder receive time. Current
   Product/session identity and receive-time scheduling are not a shared replay
   clock for every downstream timeout.
2. **Explain missing evidence.** Existing queue-loss and required-stream checks
   catch some failures. Per-topic gaps, middleware losses, camera coverage and
   cross-stream time overlap still need a unified report; final message counts
   alone cannot establish continuity or synchronization.
3. **Keep the seconds before an incident.** Add measured, bounded pre-trigger
   recording and explicit disk rotation/retention when field data size justifies
   them. Do not equate a free-space stop threshold with a ring buffer.
4. **Compare a fix using identical inputs.** Feed only recorded sensor inputs
   to the selected native algorithms, retain original outputs as references,
   and compare trajectory, collision/stop decisions, latency and resource use.
   Do not replay old state/output topics as inputs to the same fresh run.
5. **Complete robot feedback at its owner.** Add battery, temperatures, currents,
   foot contacts and motion-policy feedback only where the selected robot SDK
   supplies them. Document source rate, units and unsupported fields. Estimated
   torque must stay labelled as estimated torque.

## Independent evidence

Use a typed DDS publisher/subscriber fixture to prove values survive capture
and replay, the native MCAP reader to check schemas/counts, and an independent
consumer such as Foxglove to check external readability. Decode video segments
with FFmpeg and verify that the archive contains every segment referenced by
the timeline. Inject a missing stream or truncated file to check that the UI
does not report completeness. Finally repeat on the named ARM64 robot with the
real sensors. Local fixtures, codec decoding and a clean build do not establish
field timing, data completeness, or physical command execution.

### Local implementation evidence, 2026-10-25

The follow-up live-workbench implementation was verified with the actual
component in a browser, including light/dark themes and a 420 px embedded
layout. Selected joint/IMU checkboxes survived a full page reload. A new
eight-second native capture selected only those two streams and finalized
898 messages: 449 IMU and 449 joint states, with no selected stream missing.
The browser's validation action and `ltplay SESSION_ID --dry-run --json` both
checked those 898 messages. The native file summary is retained in
`build/recording-web-20261025/streams-capture.json`; actual live stats and typed
preview values are in `streams-live.json`.

The live observer and short player were built on Windows and Linux. Native
tests checked typed values, quiet-stream null/zero behavior, sample-age growth,
raw-input defaults, explicit overrides and preservation of the old player
selection. Linux CMake installed the complete seven-tool runtime component;
loading the existing field environment made `command -v ltplay` resolve the
installed binary and its help/file-summary commands succeeded. These remain
local and isolated DDS results, not NX deployment or hardware acceptance.

- Installed-catalog invocation returned 41 native DDS stream choices.
- Windows typed joint round-trip compared names, robot model, position,
  velocity, estimated torque and source timestamp after DDS capture, MCAP
  storage and sensor replay; the values matched.
- Real native control tests covered an IMU stream, a received record-only ACK
  and a quiet optional request. The capture completed, the quiet stream stayed
  zero and the offline check counted the ACK without publishing it.
- Browser operation used the real RecordingPanel, real Gateway routes and
  native binaries on loopback-only DDS domain 187. One synthetic session
  selected joints plus IMU: it captured 20 IMU samples and explicitly showed
  zero joint samples. A second fixture required LiDAR while publishing only
  those 20 IMU samples: it failed and retained the readable temporary MCAP and
  error logs. No robot or navigation Product was started.
- Both browser-downloaded TAR files were opened independently with Python's
  standard `tarfile` reader. Their declared files matched the native originals
  byte for byte; the failed manifest stayed failed. The local results are in
  `build/recording-web-20261025/integration.json`, with the operated UI screenshot
  in the same directory.
- Gateway targeted regressions, Web selection/rendering regressions and Web
  build/lint passed. Native Windows control/catalog/manager regressions passed;
  Linux GCC 13.3 Release core build and camera/core/manager regressions passed.
  These are local contract results. ARM64/NX sensor capture, actual camera
  encoding, external Foxglove loading and synchronized algorithm recomputation
  remain unverified by this iteration.

### Numeric timeline and A/B evidence, 2026-10-25

Two explicitly synthetic, native-written MCAP sessions exercise the complete
offline-reader -> Gateway HTTP -> real React workbench path. Each has 5502
joint-state messages over 12 seconds, with file origins separated by 60 seconds.
B's signal is delayed by 0.75 seconds and its amplitude is multiplied by 1.2.
The fixtures include a one-second missing interval, two non-finite intervals
and a narrow single-sample peak. They are UI/test signals, not robot recordings
or estimator accuracy results.

Actual HTTP checks retained the independent origins, returned peaks A=8 at
9.124 s and B=9.6 at 9.874 s, and kept the peak/null/gap boundaries when reduced
to ten returned points with a requested maximum of sixteen. A 9.0–9.3 s query
kept file-relative timestamps; a request after file coverage returned no samples
and retained the original duration/origin. The earlier DDS-recorded session
also returned all 449 joint velocity samples correctly. Results are retained
in `build/recording-web-20261025/history-integration.json`.

Browser operation verified physical-field selection, playback, rate changes,
seeking, zoom and manual alignment. Moving B by -0.75 s aligned the peaks while
preserving their values and displayed difference of 1.6. At 5.5 s in the known
gap the value remained empty. With B offset +3 s and a B-only zoom window, A
stayed empty while B showed its local time and value. This exposed and fixed a
transport-button bug that had disabled playback when A alone had no samples.
Rapid topic switches resolved to the final topic's field catalog rather than
retaining an earlier topic's fields. Dark/light and 420 px embedded layouts
were exercised, with no horizontal overflow in the history component.

Live browser inspection used a native loopback-only DDS publisher and actual
observer snapshots. Selecting a joint velocity field produced a curve from
distinct updates; when the publisher ended, the cumulative count stopped,
the sample age grew and the retained chart was marked stale with no current
value. No robot Product or motion was started. These numeric checks do not
qualify NX timing or automatic algorithm recomputation. The subsequent visual
timeline verification is recorded below.

### Shared camera/cloud/trajectory timeline, 2026-10-25

The visual inspector extends the same Web playhead, rate and A/B offset rather
than starting separate media clocks. It follows the official Foxglove
[3D conventions](https://docs.foxglove.dev/docs/visualization/panels/3d) and
[Image panel](https://docs.foxglove.dev/docs/visualization/panels/image) reference:
keep source identity and timestamps, distinguish log time from message time,
and require a meaningful coordinate relationship before overlaying geometry.
LingTu currently views different unknown/mismatched frames separately instead
of implementing a new transform resolver in Gateway.

The existing WebGL point-cloud viewer accepts recorded cloud/trajectory/path
buffers, preserving its existing saved-map loader. Indexed native scene reads
provide five-second windows with one second of lookbehind and display-only
point/frame reduction. A limited browser cache prefetches the next window.
Recorded paths remain plan snapshots; a clear message or later plan replaces
their prior state. Empty/resumed transitions survive frame sampling. Past
estimated trajectory and planned path have distinct styles.

Camera inspection uses native camera-index metadata and the installed FFmpeg
decoder. New captures retain acquisition timestamps for every encoded frame;
legacy segment-only captures retain an explicit estimate label. Compact catalog
metadata omits the full recording's timestamp arrays; the selected segment's
index is loaded lazily. A bounded terminal-recording/JPEG cache avoids repeated
native startup for each displayed image. Only declared artifacts are accessible;
the normal deletion API invalidates these cached entries. This is sampled
visual playback, not a claim of the original video frame rate on every target.

Actual sensor data was taken from the already downloaded public GEODE Stairs
Alpha bag, using the repository's existing ROS1 reader without installing ROS
or rosbags. Two overlapping windows (20–32 s and 22–34 s) contain 120 real camera
frames and 120 real cloud scans each. The 121 poses per window come from the
existing LTLIO `replay.odom.tum` output, not ground truth. Clouds are display
imports transformed through the official `T_IMU_LiDAR` and the nearest recorded
estimated pose (maximum association difference 1.307 ms), with finite XYZ
filtering and no newly claimed deskew. Images are downscaled to 612×512 for
the inspection fixture. The original bag and estimator output remain intact.

The native import helper writes the normal typed CDR/MCAP and camera-segment
formats. Its explicit `dataset_import` / `GEODE_PUBLIC_RECORDING` identity
distinguishes this acceptance data from a robot-side capture. A third session
contains six seconds of the same real camera recording and no DDS artifact.
Extraction inputs and provenance are in `build/recording-visual-20261025/source`;
HTTP results are in `visual-integration.json` in the same build directory.

The actual HTTP check associated the same source image timestamp
`1705542330281968764` in A and B, despite different segment-local ordinals
(19 and 50) and different file origins. The measured B alignment offset was
1.9998033 s. A five-second scene read returned 32 cloud frames, 61,894 display
points and 51 poses; retained coordinates matched their imported source.
No navigation plan was fabricated for this sensor-only dataset.

This integration exposed a normal clip-boundary issue: acquisition end is the
last frame timestamp, not the end of that frame's display interval. Camera
metadata now keeps acquisition end unchanged and separately reports a bounded
`coverage_end_s`, allowing the last frame to remain visible for its normal
frame period without filling a real recording gap. Out-of-coverage image
requests return no frame. Field capture, physical motion, hardware clock
synchronization and re-running navigation algorithms remain separate evidence.

Final browser operation used the actual RecordingPanel and Gateway on loopback.
It exercised A/B playback and pause at 0.25×, pointer dragging across the 5/10 s
scene-buffer and video-segment boundaries, 2D/3D switching and wheel zoom.
Camera-only playback also advanced and paused normally with one camera panel,
no 3D panel, and exactly one timeline scrubber. The 420 px dark sidebar kept
371 px content width equal to its scroll width; its A/B source selector retained
the same shared cursor. Camera details showed `left_camera` and the original
nanosecond timestamp rather than treating compact metadata as missing data.
File/statistics details are collapsed, and the one existing transport is sticky
above media so operators can seek while watching the result.

The initial different-frame decoder cost about 0.249 s per request on this
machine. Lazy four-frame block decoding retains the same eight-image cache
budget and amortizes process startup. Final actual HTTP alternation over twelve
A/B frames averaged 0.1935 s, including cold segment work; block hits were
0.036–0.059 s and most subsequent new blocks 0.231–0.303 s. Cold reads can still
take longer. These measurements do not establish a uniform 0.2 s deadline or
the original camera frame rate. Full results and operated screenshots are in
`build/recording-visual-20261025/visual-integration.json`,
`visual-scene-operated.png`, `visual-ab-3d.png`, `visual-sidebar-dark.png` and
`camera-only-operated.png`.

### Camera loading follow-up, 2026-10-25

Normal-speed browser playback exposed a request-lifecycle defect missed by the
earlier paused/slow-play checks. The camera cursor advances at five sampled
targets per second, but decoding a cold block can exceed 200 ms. Canceling the
previous request at every target could therefore discard every completed image.
The concrete camera loader now keeps one request in flight and coalesces forward
playback into the latest pending cursor. Source changes and explicit seeks
invalidate obsolete work; image identity includes the recording session.

No-frame responses are a distinct state. Camera coverage uses a half-open end,
with separate before-start, gap, ended, first-read, updating and error/retry
messages. The existing image and its true age remain visible during updates.
Normal frame-period jitter at adjacent segment boundaries is covered without
extending real missing-frame gaps. Actual GEODE video counts and all selected
frame timestamps match their source inputs; the blank screen was not evidence
of corrupted recordings.

A second measured bottleneck was scene JSON conversion on Gateway's event loop:
two large scene responses delayed even cached camera images. Scene response
encoding now runs off the event loop and returns the same JSON contract.
The focused regression detects event-loop encoding directly; it failed before
the change and passed afterward. Camera-loader tests use the actual API reader
with a 20 Hz cursor and deferred 300 ms responses, checking progress, one
request in flight, no forward-play cancellations, stale-source rejection,
retry and exact end coverage. Operated screenshots for this follow-up are in
`build/recording-camera-fix-20261025`; these remain local Web/Gateway evidence,
not Go2 camera capture or a uniform video-frame-rate guarantee.

### Live video versus saved history, 2026-10-25

The operator's follow-up exposed an interaction gap: the fixture's recorded
camera preview was being interpreted as the current robot camera. The Web
workbench now has a read-only live camera on the **实时** tab, reusing the
existing WHEP, JPEG WebSocket, decoded-frame freshness and rotation hooks.
Connections wait for camera-status evidence; unloaded, unavailable and stopped
frames have separate labels. A connected transport alone does not establish a
fresh image. The saved-session tab and heading are **历史回看**, explicitly
marked **录制回看 · 非实时**. This does not add a second recording or motion path.

The 29 directly affected Web tests, TypeScript, affected ESLint and production
build passed. The fixture's unavailable-source state was operated in the real
browser; it does not establish a live D435i stream. Actual NX enumeration found
SDK serial `419522073370`. Current-source ARM64 recorder/camera checks passed
19 + 2 + 2 + 8 tests, including independent H.264 decoding. The software checks
remain distinct from the subsequent Product-controlled field startup and
physical event-to-display delay measurement.
