# Camera video branch

The [camera adapter architecture](../README.md) defines the SDK-independent
capture writer, record/SHM contracts and steps for connecting another source.

Updated 2026-11-01. This document distinguishes the managed robot-side
pipeline, the LingTu Web receiver, and the DOSO App integration candidate.
The Go2/NX live result below does not establish that Thunder or G20 is ready.
The [delivery and acceptance guide](DELIVERY.md) records the supported-source
matrix, effective device configuration, release procedure and remaining field
qualification. A source adapter or active service is not a product acceptance.
For isolated receiver qualification, the [foreground video Gateway](../../../../../gateway/VIDEO_PROBE.md)
exposes real status and WHEP without starting a navigation Product. It is a
development test entry point, not a camera-only production deployment mode.

`lingtu_camera_webrtc` reads existing native color SHM into GStreamer. The
camera remains the only physical-device owner; perception still receives the
original RGB-D observations. Gateway proxies signalling only. Scene and Console
share the same browser lifecycle.

```text
native camera -> color/depth/info SHM -> Host perception/tracking
                 color SHM -> latest RGB/BGR -> exact encoder
                              -> native WHEP -> browser
                              -> H.264 stdout -> go2rtc -> browser (reference)
Gateway: status + SDP + session DELETE; no video payload
```

## Managed deployment and interfaces

The camera and video processes are already in the real Product/RunPlan and
ProductControl lifecycle. Select a Product camera variant, prepare a native
release for the target, and use the existing release installer and ProductControl
entry point; do not start competing camera owners or treat `systemctl start` as
the Product lifecycle. The release packages the target video binary and runtime;
`/etc/lingtu/video.env` must explicitly name an encoder available on that target.
See [robot operations](../../../../../../docs/operations.md),
[video.env.example](../../../../../../config/video.env.example), and the
[native release installer](../../../../../../scripts/deploy/install_native_release.sh).
Video is optional in some camera variants: an active Product is not by itself
evidence that video is running or ready.

The public Gateway routes are:

| Method and route | Meaning |
| --- | --- |
| `GET /api/v1/camera/status` | Camera source status, including freshness and counters. |
| `GET /api/v1/camera/diagnostics` | Candidate consolidated, read-only camera, video pipeline and WHEP observations. |
| `GET /api/v1/webrtc/status` | Video availability, source readiness, ICE configuration and display rotation. |
| `POST /api/v1/webrtc/whep?src=cam` | Submit an `application/sdp` offer; receive an SDP answer. The native backend ignores `src`. |
| `DELETE /api/v1/webrtc/sessions/...` | Release a returned WHEP session; send `If-Match` if an ETag was supplied. |

For a Gateway origin already verified on the target, read-only diagnostics can
start with the two status routes; use the actual configured origin, not the
Brainstem gRPC port. Product readiness checks the versioned native status,
heartbeat, source generation/sequence and fresh source in addition to the
service and WHEP endpoint. A running process, HTTP success or WebRTC connection
alone does not prove fresh decoded video. The Gateway status route performs a
more limited readiness check than Product readiness. The source now contains an
opt-in shared live-media Bearer guard, disabled by default to preserve the
existing open Gateway contract. It is not deployed or a general per-App
identity system; production requires a verified HTTPS/access boundary. See
[delivery and acceptance](DELIVERY.md) for configuration and limitations.

`lt-video` can wait for a fresh source and rebuild its pipeline after a source
change. Its service unit does not itself declare a `Restart=` policy, so do not
assume recovery from a fatal process exit without checking the active Product
supervisor. See the status snapshot and service logs for `last_error`; the
current strings are not a complete stable fault-code dictionary.

## DOSO App receiver candidate

The separate DOSO App source now has a receive-only WHEP client and control-page
video surface. It reads Gateway status and ICE configuration, sends an SDP
offer, and releases an identified WHEP session on exit. It marks video receiving
only after decoded-frame counters advance; it hides a frozen frame, retries,
and retires the old session on background, page exit or device change. A lost
live video feed also interrupts the current manual-input intent; video recovery
does not automatically restore motion.

The current App integration requires an explicitly selected navigation Gateway
whose hostname matches the selected direct Brainstem target, and invalidates the
association when the target changes. This is a conservative address association,
not device identity authentication; split-host deployments are not yet supported.
HTTPS uses normal certificate verification. Private-LAN HTTP is restricted to
the explicitly enabled development path; it is not production viewer security.
The local test entry selects the Gateway under advanced settings / navigation
service and uses the same-host video association. Navigation status must first
be read successfully by that entry: the Gateway must also serve the existing
bootstrap, capabilities, state and readiness APIs. A bare WHEP endpoint alone
cannot be selected through the current UI. Use the Gateway origin, not the
internal loopback WHEP endpoint or the Brainstem gRPC port. A `201` response
requires a same-origin session Location; a `200` answer without Location is
accepted for the existing go2rtc backend. The App source now accepts an optional
viewing credential, sends it only to the validated HTTPS Gateway and its
same-origin session endpoint, and stops retrying on 401/403. The credential is
held in memory for the selected session; no provisioning or persistent identity
management is implemented. Neither this client change nor the Gateway guard has
been deployed or verified against a real TLS endpoint.

The local G20 test route also offers a separate explicitly selected
"video-only" view. It does not open a Brainstem control session or enable
joystick input, so a Go2/NX Gateway can be tested even when it is not the
same host as a separate robot controller. It still requires a reachable
Gateway status API and a newly built native APK.

The App dependency is `react-native-webrtc` 155.0.1. App unit/lifecycle tests,
TypeScript and the Android WebRTC module's standalone Java compilation passed;
the complete new APK has not been built or installed on G20. Native module
autolinking is not proof of device decoding. The App pins the upstream Android
WebRTC artifact to 155.0.1 in its build configuration; a full APK build and
device verification are still needed. The App currently has no demonstrated Thunder-to-G20 video,
and the target Gateway's actual authentication/TLS configuration is unverified.
The DOSO design preview is mock data and does not connect to this service.

## GStreamer build and playback

Approved development versions are recorded in
[`video.json`](../../../../../../scripts/build/locks/video.json): GStreamer
1.28.8 and gst-plugins-rs 0.15.4. `whepserversink` needs GStreamer 1.28.
System GStreamer 1.24 can compile and run the H.264 reference branch, but cannot
provide native WHEP.

The isolated 1.28.8/0.15.4 runtime was built and used for actual browser playback
on 2026-10-17. Rust transitive dependencies are fixed in
[`video-cargo.lock`](../../../../../../scripts/build/locks/video-cargo.lock);
the preparation helper uses `cargo cbuild/cinstall --locked`. Source, tools and
compiler intermediates live under `${XDG_CACHE_HOME:-${HOME}/.cache}/lingtu/video/`,
so a WSL restart does not discard the build. The installed prefix stays under
`build/deps/video/prefix/` and does not replace the system GStreamer.

```bash
# Isolated dependencies, without replacing the system multimedia stack.
bash scripts/build/prepare_camera_video.sh
source build/deps/video/prefix/env.sh
bash scripts/build/build_camera_video.sh

# Foreground development session, explicit SOFTWARE encoder.
build/camera_video/lingtu_camera_webrtc --encoder x264enc \
  --listen http://127.0.0.1:9090 --fps 20 --start-bitrate 1500000 \
  --idle-timeout-ms 0 --status-file /dev/shm/lingtu/video_status.json

# Configure the Host before starting it through its existing entry point.
export LINGTU_WEBRTC_URL=http://127.0.0.1:9090/whep/endpoint
```

The standalone WebRTC executable requires an explicit `--encoder`. Managed
services supply it from the target's deployment settings.
`--encoder nvv4l2h264enc` selects NVIDIA explicitly. An unavailable encoder
fails instead of silently falling back to software. Qualify the selected
board's BSP, encoder and simultaneous perception load separately.

The managed WHEP process is long-lived. When accepted camera frames become
stale it releases the old GStreamer session and its consumers, publishes
`waiting_for_source`, and waits for a fresh source without exiting. A source
generation or geometry change creates a new session. The service becomes ready
only after that session accepts a frame which is still within `--max-age-ms`;
building a pipeline from an older format frame is not readiness evidence.
`--idle-timeout-ms 0` is accepted for WHEP service mode. Annex-B file/FD output
still requires a positive idle timeout and exits on source loss.

`--status-file` atomically replaces one flat JSON snapshot at least once per
second. Schema `lingtu.camera_video.status.v1` exposes `status`, `ready`,
`source_fresh`, `heartbeat_timestamp_s`, source generation/sequence, frame age,
the exact WHEP endpoint and encoder, and `last_error`. `ready` never requires a
viewer, but it does require a live WHEP pipeline and a fresh accepted frame.
Signal shutdown writes `stopped`; fatal pipeline/configuration failures write
`error`. Product readiness may inspect this file without making Gateway own the
native process.

WHEP defaults to LAN direct ICE without a STUN server. This explicitly clears
the sink and WHEP signaller's upstream public STUN defaults, so local playback
does not depend on Internet access. To gather public candidates, pass
`--stun-server stun://host:port`; the same URI configures the media sink and
the signaller's advertised ICE server. The browser's ICE configuration remains
the Host's `LINGTU_WEBRTC_ICE_SERVERS` setting. STUN does not relay media or
guarantee connectivity through every NAT.

For GStreamer 1.24, `--output FILE|-` selects Annex-B H.264. The
[`go2rtc template`](../../../../../../config/go2rtc.yaml) consumes stdout
without opening `/dev/video*`. This reference uses fixed encoder settings:
WebRTC feedback does not control the upstream encoder. Native WHEP leaves raw
input under the sink's encoder and congestion-control ownership; actual
adaptation remains a network qualification task. Browser and server both need
reachable ICE candidates; forwarding HTTP signalling alone is insufficient.

Appsrc and the raw queue each hold at most one frame. Padding is copied into
GStreamer video alignment. WHEP maps capture age into the live pipeline clock:
`PTS = max(0, running_time - capture_age)`. This avoids scheduling fresh frames
in the future after pipeline initialization. Metrics expose `last_pts_ns` and
`last_running_time_ns`; valid WHEP submissions keep PTS at or before the running
time. The file/FD branch retains its stream-relative timestamp origin.
Duplicate/backward capture timestamps are rejected. Source-generation, geometry
or format changes release the old session before rebinding. Encoder, queue,
consumer and RTC metrics do not measure capture-to-screen latency.

For the long-lived WHEP service, accepted-frame age is anchored to
`steady_clock`, including the age already present at acceptance. A wall-clock
correction cannot make a frozen frame younger. If a newly read SHM sequence
has a regressed Unix timestamp, the WHEP service rebinds its session so a host
clock correction does not stall playback until the old timestamp is reached.
The file/FD timestamp contract remains unchanged. URI validation runs after
GStreamer initialization; checking only a preinitialized library caller misses
that process-startup requirement.

Both browser views keep JPEG until a decoded WebRTC frame arrives. First-frame
freshness starts after the transport connects; after playback, a three-second
decoded-frame freeze triggers reconnection. Native WHEP can gather ICE for up
to 30 seconds, so Gateway bounds the complete offer request at 35 seconds and
the browser POST budget is 40 seconds. These are cancellation budgets, not
playback or latency targets. Closing a view releases peers, tracks and WHEP
resources. Rotation uses RobotConfig and fits 90/270-degree mounts without cropping.
The generic status route is
`GET /api/v1/webrtc/status`; the old go2rtc status route remains available.

## Verified chains and limits

On 2026-10-25 a Go2/NVIDIA NX deployment used a real D435i, an explicitly
selected `x264enc` software encoder and native WHEP; Edge displayed the actual
WebRTC image. The record also reports 42 consecutive ready/fresh reads and
approximately 30.2 fps **camera capture**, not measured end-to-end display rate
or latency. See the [Go2/NX field record](../../../../../../docs/operations/go2-camera-20261025.md).
This bounded observation does not establish Thunder hardware compatibility,
G20 decoding, sustained performance, or a network recovery SLA.

The earlier [local evidence page](../../../../../../build/camera_video_whep_tests/index.html)
and [result record](../../../../../../build/camera_video_whep_tests/RESULTS.md)
show continuous synthetic color SHM → software `x264enc` → native WHEP → actual
browser decode, without go2rtc between producer and browser. They also show
stale-frame withdrawal, source generation 502→503 recovery, closing the receiver,
JPEG fallback and media-service recovery without reloading the page. Closing
left `active_consumers=0`, and Gateway session DELETE returned 200.

The source is 16×16, not a physical camera or performance benchmark. Two
approximately 32.5-second upstream timeout/500 attempts during source recovery
remain in the evidence; automatic recovery does not prove a recovery-time SLA.
The latest native GStreamer clock regression passed in 1.64 seconds, and the
CLI again decoded all 32 RGB/BGR padded-row frames across two source generations.
The earlier [go2rtc reference](../../../../../../build/camera_video_gst_tests/index.html)
remains a separate record.

## FFmpeg reference

`lingtu_camera_video` consumes the existing native color SHM and feeds an
external FFmpeg H.264 encoder. It never opens a camera device or reads depth.
This is a foreground Linux executable for validating the board-side encoding
branch. It is not yet a managed Product process or a WebRTC publisher.

```text
native camera (one device owner)
  -> color/depth/info SHM -> Host perception and tracking
  -> color SHM -> latest frame -> FFmpeg -> Annex-B H.264 file/stdout
                                      -> JSONL input metadata
```

## Build and run

The FFmpeg target requires C++17, CMake and the Linux system libraries. Runtime reuses the
external FFmpeg executable already used by native camera recording; no FFmpeg
headers, GStreamer, cloud SDK or new Python dependency is linked or installed.

```bash
cmake -S src/drivers/real/camera/native/video -B build/camera_video
cmake --build build/camera_video --target lingtu_camera_video

# Run after the selected Product has started its native camera.
# Both output paths must be new; existing files are not overwritten.
build/camera_video/lingtu_camera_video \
  --output /tmp/camera-preview.h264 \
  --metadata /tmp/camera-preview.jsonl \
  --codec libx264 --fps 15 --bitrate-kbps 1500 --frames 150

# Decode the result. This checks video bytes, not just process startup.
ffmpeg -v error -i /tmp/camera-preview.h264 -f null -
```

`--output -` writes binary H.264 to stdout for a local consumer. Without
`--metadata`, stderr mixes structured events with plain-text diagnostics from
the program and FFmpeg. Use `--metadata FILE` for a clean JSONL stream.
Run `--help` for source, freshness and timeout options.

The default **libx264 is software encoding**. `--codec` selects an installed
FFmpeg encoder explicitly; unavailable or incompatible encoders fail without
silently falling back to software. Codec selection alone is not proof of
hardware acceleration. In particular, desktop `h264_nvenc` availability does
not establish Jetson encoding support. The selected board's BSP and encoder
must be qualified separately. The NVIDIA GStreamer implementation above still
needs that board qualification.

## Frame and process behavior

- `shm_reader.hpp` consumes the existing `lingtu.camera.shm_frame.v1` contract.
  It preserves source generation, sequence, capture timestamp, geometry and
  RGB/BGR encoding. Padding is removed before writing raw pixels to FFmpeg.
- Each poll opens the existing SHM name read-only. `pread` and the existing
  guards/CRC avoid retaining a stale mapping when the camera replaces or
  resizes the ring. Missing, duplicate, stale or incomplete frames are not sent.
- There is no application frame queue. After a write completes and the FPS
  interval allows another input, the consumer reads the newest published slot.
  Intermediate raw frames can be skipped. Encoding has its own bounded pipe
  and child process; it cannot lock or stall the camera writer.
- A frame write has an absolute deadline (default 500 ms). If the encoder or
  output stops draining, the program reports failure and reaps the encoder.
  It never sends the next raw frame after abandoning a partial frame write.
  Camera/Host lifecycles remain independent.
- Source generation or geometry changes finish the previous stream and start a
  new encoder on the same output. With the verified libx264 configuration the
  new stream starts with SPS/PPS and an IDR frame. Sequence reuse is scoped to
  its source generation. A fresh-frame timeout (default 5 s) exits with error
  instead of repeatedly encoding the last image.
- libx264 uses `ultrafast`, `zerolatency`, no B frames and a one-second GOP by
  default. Bitrate and GOP are initial settings, not network-adaptive controls.

## What the metadata proves

JSONL has `stream`, `submitted`, `finished` and `stopped` records. A signal between
frames drains the encoder and emits `finished`; a signal during a blocked frame
aborts the encoder, emits `stopped` with `encoder_aborted: true`, and exits
successfully. An incomplete frame is not counted or reported as finished.
Actual encoder errors still exit unsuccessfully. Each submitted input
includes the original `capture_timestamp_ns`, `source_generation`, `sequence`,
stream-local `index`, reader duration, input write duration and age at submission.
`skipped` counts sequence gaps between submitted frames within a stream; it
does not include frames published before the first submission.

`submit_us` measures pipe submission/backpressure, **not completed encoding or
capture-to-screen latency**. A successful write only means the raw bytes were
submitted. Decode validation establishes output frame count and validity.

Raw Annex-B H.264 has no per-frame wall-clock timestamps. This file is encoded
at the selected fixed FPS, so gaps in camera acquisition may compress playback
time. The JSONL preserves the source timeline separately. Use the GStreamer
branch above for live WHEP timestamp mapping; this file is not a timestamped
perception input.

## Checks

```bash
cmake -S src/drivers/real/camera/native/video -B build/camera_video_tests \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/camera_video_tests --parallel 2
ctest --test-dir build/camera_video_tests --output-on-failure
```

The reader checks latest-only delivery, restart isolation, padding and damaged
frames. Encoder tests use controlled child processes to check input bytes,
failure, blocked output/input and cleanup. The CLI test uses the real SHM writer
to exercise freshness and generation changes. If FFmpeg is found at configure
time (or supplied as `-DLINGTU_VIDEO_FFMPEG=/path/to/ffmpeg`), an additional test
encodes and decodes H.264, including a camera restart.

These FFmpeg checks are local contract/software-codec checks. They do not
establish ARM64 hardware encoding speed or live-camera performance. Native
WHEP browser playback is recorded separately above.

On 2026-10-17 all four CTest groups passed on WSL Ubuntu x86_64. The codec test
used the existing Windows FFmpeg through WSL stdio interoperability: 32 synthetic
16x16 frames across two camera generations decoded correctly, including IDR/P
frames and RGB/BGR padded rows. Blocked-write cancellation returned `stopped`
and reaped its child. This is not a board performance measurement. Local logs
and input metadata are retained under `build/camera_video_tests/`.

## Next integration boundary

First qualify the target board's encoder with actual RGB-D capture running,
including load, frame age and output bitrate. Qualify actual IDR/bitrate feedback,
first-frame/recovery time and weak-network behavior at the supported image sizes.
The local WHEP clock and playback checks do not replace these measurements. The
managed media process is already registered through Product/RunPlan and
ProductControl; Gateway must not spawn it independently. Verify viewer-demand
behavior and fatal-process recovery on the chosen deployment. Direct LAN ICE is
the current default; evaluate TURN only for a network that needs relay, and an
SFU only if the actual number of viewers warrants one.

JPEG remains the browser fallback. The go2rtc template consumes the same SHM
source through the native encoder and does not start another camera owner.
See [remote video design](../../../../../../docs/research/video.md).
