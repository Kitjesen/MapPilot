# Camera/video delivery and acceptance

Updated 2026-11-01. This is the delivery guide for the existing camera and
video processes. It does not certify a new camera, board or App installation.
The implementation and frame contract remain in [README](README.md).

## Qualification matrix

Use one row per actual device combination. `Source available` means an adapter
or build path exists; it is not field qualification.

| Target | OS / architecture evidence | Camera / capture adapter | Encoder / runtime | Evidence and remaining work |
| --- | --- | --- | --- | --- |
| Development host | Ubuntu 24.04, x86_64 is the declared dependency build target | Synthetic 16×16 RGB/BGR SHM | GStreamer 1.28.8, gst-plugins-rs 0.15.4, explicit `x264enc` | Actual browser decode, source replacement, stale-frame withdrawal and cleanup recorded on 2026-10-17. No physical-camera or throughput qualification. |
| Go2 / NVIDIA NX | Linux aarch64 native release; exact OS/BSP must be recorded again for the next run | RealSense D435i, `realsense_native`; SDK selection documented as librealsense 2.56.5 | Explicit software `x264enc`; target-built video runtime | Real camera and Edge WHEP image recorded on 2026-10-25. Approximately 30.2 fps is capture rate, not display rate. Sustained load, recovery time and G20 remain unqualified. |
| Orbbec native path | Target Linux SDK/runtime required | `orbbec_native`; Gemini 335 appears in device configuration | Explicit encoder selected for that board | Source/build/configuration support exists. Do not infer a field result from the RealSense/NX row. |
| Thunder at `192.168.66.64` | Linux aarch64 observed in the 2026-11-01 read-only inspection; exact BSP still to identify | Existing GS130W service uses TROS/ROS2 and WebSocket on TCP 8080 | Existing `hobot_codec_republish`; no LingTu WHEP qualification | `rdk-gs130w-camera.service` was active; no `lt-camera`, `lt-video` or `lt-host` unit was found. New-frame output and compatibility with the native frame contract were not established. Preserve the current camera owner while investigating its output. |
| G20 receiver | Exact Android/native APK identity must be recorded at installation | Receives color video; does not own the robot camera | DOSO App source uses `react-native-webrtc` 155.0.1 | App behavior tests and standalone Android module compilation passed. A complete new APK, actual decoding, latency and recovery have not been accepted. |

Sources: [dependency lock](../../../../../../scripts/build/locks/video.json),
[Go2/NX field record](../../../../../../docs/operations/go2-camera-20261025.md),
[camera drivers](../camera_record.hpp), and
[device declarations](../../../../../../config/devices.yaml).
Keep OS/BSP, camera firmware, selected SDK/runtime and encoder versions in the
actual release evidence. A version label or architecture override does not
cross-compile a binary or establish hardware acceleration.

On 2026-11-01 an isolated Go2/NX `.97` smoke test observed real D435i frames
through unique SHM and a native WHEP pipeline (`ready`/`source_fresh`, advancing
sequences; loopback OPTIONS 200). A separate isolated H.264 output test decoded
103 frames with FFmpeg into a null sink. No image was saved, displayed or sent
over the network; no SDP/peer was established. The temporary processes and SHM
were removed, while the managed camera/video/host services remained inactive.
These checks do not prove WHEP media reception, G20 playback, latency or the
new local diagnostic/authorization code on that target. See the local evidence
record `build/camera-video-smoke-20261101/RESULTS.md` in the LingTu workspace.
An additional isolated WHEP/Firefox probe did not decode: the available Firefox
110 offer did not advertise H.264 and its loopback-only network namespace did
not provide ICE candidates. A local proxy returned 502 without an SDP answer;
this is not evidence that the upstream service itself returned 502. All test
processes and temporary resources were cleaned up. Use an H.264-capable WebRTC
client and a verified ICE path for the next receiver test.

## Device configuration and packaging

The local source now allows the selected RobotConfig to carry camera and video
profile overrides in one `camera` section. These changes are a candidate and
are not installed on the Go2/NX or Thunder by this documentation update.
Bitrate, runtime paths and the viewing-security boundary remain deployment inputs.

| Value | Current owner | Deployment rule |
| --- | --- | --- |
| Camera driver and SDK serial | Selected `config/robots/<vendor>/<model>/robot.yaml`, resolved into RunPlan | `orbbec_native` or `realsense_native` must match the built capture binary. Use SDK enumeration for serial selection. |
| Intrinsics, depth scale, body transform and display rotation | RobotConfig and runtime CameraInfo | Keep this robot's calibration. The Go2 profile explicitly marks camera calibration unverified. |
| Capture color/depth size and FPS | Optional RobotConfig `capture_*` fields, otherwise `config/system/settings/camera.yaml` | If any capture width or height is set, set all four color/depth dimensions so RunPlan can size the SHM slots. FPS can be set independently. Calibration `width/height` remain separate. |
| Video FPS and bitrate | `camera.video` in `config/system/settings/camera.yaml`, with optional RobotConfig `video_fps` | ProductControl writes these values to the Product session. |
| Encoder and multimedia runtime | Optional RobotConfig `video_encoder`, otherwise `/etc/lingtu/video.env`; packaged runtime | Select the exact available encoder; no implicit software fallback. Build the runtime for the target architecture. Explicit RobotConfig values in the session override the device environment. |
| Product, variant and map | ProductControl | Camera variants select camera/video processes. Video can be optional, so check its own readiness. |
| Viewer origin and access | Gateway/deployment configuration and selected App target | Configure reachable ICE and verified HTTPS. The App's current same-host restriction and in-memory viewer credential apply. |

Example shape only; fill it after verifying the actual camera and encoder:

```yaml
camera:
  capture_driver: realsense_native
  serial_number: "<verified SDK serial>"
  capture_color_width: 640
  capture_color_height: 480
  capture_color_fps: 30
  capture_depth_width: 640
  capture_depth_height: 480
  capture_depth_fps: 30
  video_encoder: x264enc
  video_fps: 20
```

These numbers do not certify that a different camera or board supports them.
The original `width` and `height` fields are calibration defaults, not capture
profile overrides. The slot capacity is derived from explicit sizes; check
available shared memory and concurrent load on the actual target.

Do not copy a configuration from another board simply because both use Linux.
The current source does not support `gs130w` as a native capture-driver choice.
That path needs a documented adapter into the existing frame contract before
it can be added to the supported matrix.

On a compatible Linux build host, follow the existing native release procedure
with the selected driver pair. For example, the existing Go2/RealSense path is:

```bash
# Build host, not an activation command. Match the selected RobotConfig.
export LINGTU_DRIVER_BACKEND=go2
export LINGTU_CAMERA_DRIVER=realsense_native
make build BUILD_TYPE=Release
bash scripts/deploy/package_native_release.sh "v$(cat VERSION)" dist
```

The packager consumes built artifacts and verifies the video runtime, plugins,
selected camera SDK and native release inventory. It produces the full LingTu
native release, not a camera-only installer. Follow
[Deployment](../../../../../../scripts/deploy/README.md) for the build and
runtime preparation; the video dependency build in the main README remains
separate from service startup.

On the target, inspect the current Product and retain any map needed by it.
Verify an extracted candidate before the authorized install:

```bash
# Fill these with the actual robot profile and extracted candidate directory.
: "${LINGTU_ROBOT:?Select the actual RobotConfig}"
: "${LINGTU_PACKAGE_DIR:?Select the extracted native release}"
bash /opt/lingtu/current/scripts/lingtu status --robot "$LINGTU_ROBOT" --env real --json
bash "$LINGTU_PACKAGE_DIR/scripts/deploy/install_native_release.sh" \
  --dry-run --package-dir "$LINGTU_PACKAGE_DIR"
```

The actual installer changes the installed release and reactivates the prior
Product/map. Run it only in the deployment window, with the matching device
configuration prepared. Product changes use `scripts/lingtu switch ...
--variant camera`; do not substitute an independent camera/video systemd start
sequence. The existing `nav/camera` variant also starts navigation-related
processes: it is not an isolated camera-only test mode.

A target-qualified software-encoding example for `/etc/lingtu/video.env` is:

```ini
LINGTU_VIDEO_ENCODER=x264enc
LINGTU_VIDEO_RUNTIME_ENV=/opt/lingtu/current/lib/lingtu/video/env.sh
```

Use this example on targets with the `x264enc` encoder and packaged runtime.
Edit `camera.video` in `config/system/settings/camera.yaml` to tune frame rate,
bitrate and frame age, then start a new Product session.

## Diagnostic sequence and interpretation

Use the existing non-motion field doctor and Product status for the installed
run, then inspect the camera/video layers. No command below starts a service or
requests robot motion.

```bash
: "${LINGTU_ROBOT:?Select the actual RobotConfig}"
: "${LINGTU_PYTHON:?Use the installed LingTu interpreter}"
: "${LINGTU_GATEWAY_ORIGIN:?Use the verified Gateway origin without credentials}"
bash /opt/lingtu/current/scripts/lingtu status --robot "$LINGTU_ROBOT" --env real --json
PYTHONPATH=/opt/lingtu/current/src "$LINGTU_PYTHON" -m diagnostics.field.doctor --non-motion --json --strict
curl --fail --silent --show-error --max-time 5 "$LINGTU_GATEWAY_ORIGIN/api/v1/camera/status"
curl --fail --silent --show-error --max-time 5 "$LINGTU_GATEWAY_ORIGIN/api/v1/webrtc/status"
curl --fail --silent --show-error --max-time 5 "$LINGTU_GATEWAY_ORIGIN/api/v1/camera/diagnostics"
systemctl show lt-camera.service lt-video.service --property=LoadState,ActiveState,SubState,MainPID,NRestarts
journalctl -u lt-camera.service -u lt-video.service --since '-5 minutes' --no-pager
```

Review output before sharing it; do not include credentials or sensitive camera
images in a diagnostic handoff. Use the configured HTTPS trust chain; do not
use `curl -k` to bypass certificate verification.

| Layer | Evidence to read | Interpretation / action |
| --- | --- | --- |
| Hardware owner | Selected capture driver/SDK enumeration and camera process | Exactly the selected camera owner must run. A `/dev/video` node is not required for every SDK camera, and its presence is not proof of frames. |
| Acquisition | `/api/v1/camera/status`: frame counters, FPS, color/depth age | Take two bounded observations and confirm counters advance. A cached JPEG alone is insufficient. Resolve camera/profile/SDK errors before changing the encoder. |
| SHM source and encoder | Native `video_status.json`: schema, heartbeat, source generation/sequence, `frame_age_ms`, `source_fresh`, `ready`, encoder, `last_error` | Product checks state freshness and source identity. An older status file or waiting source cannot be called live video. Default maximum submitted frame age is 250 ms. |
| WHEP | `/api/v1/webrtc/status`, SDP response and reachable ICE candidates | HTTP signalling reachability alone does not establish a media path. Check endpoint, encoder and network route. |
| Client | Decoded-frame counters and visible changing scene | `connected`/track arrival is not decoding. Distinguish frozen video from a stationary scene using counters or a visible clock/counter. |
| Cleanup | Peer/track release, DELETE result when a session Location exists, server `active_consumers` | Closed views must not retain consumers. A go2rtc `200` answer can omit Location; peer closure is then the available client cleanup path. |

The candidate `GET /api/v1/camera/diagnostics` consolidates camera-frame age,
native video status/heartbeat, pipeline readiness, encoder and WHEP probe. It
leaves physical connection, encoded output and client decode unknown when not
observed. It does not open a camera or viewer session; it is not yet installed
on the target. Current errors are not a complete stable fault-code catalogue.
Fatal-process restart still needs target verification.

The source also includes an opt-in live-media Bearer guard controlled by
`LINGTU_MEDIA_REQUIRE_TOKEN=1` and a separately provisioned
`LINGTU_MEDIA_VIEWER_TOKEN`. Missing configured token fails closed. This is a
shared credential for new live-media requests, not per-App identity or instant
revocation of an existing RTC session. It does not protect historical recording
downloads; the existing browser JPEG WebSocket cannot supply the Bearer header
and is unavailable with the guard enabled until its client is adapted. Keep the
native media listener on loopback and provision verified HTTPS and credentials
at the deployment boundary. The App accepts a viewing credential only over
HTTPS, holds it in memory for the selected session, and does not persist it.
No access policy or credential has been deployed to Go2/NX or Thunder here.

## Acceptance record

Prepare a per-run record with target/release/App identities, effective camera
profiles, encoder, network, Product/session, selected map, observations and
result for each row. Mark `not run` or `failed` explicitly; historical Go2/NX
results cannot fill a Thunder or G20 row. Run fault injection only with a
stationary, supervised target and an agreed maintenance window; a network
disconnection must not remove the operator's only means of stopping a robot.

The following are candidate qualification gates, not existing field results
or advertised SLAs. Before a performance run, agree first-frame, recovery,
capture-to-screen latency and concurrent-load budgets for the target. Missing
budgets or measurements leave that target unqualified.

| Scenario | Procedure | Required observable result |
| --- | --- | --- |
| Baseline | Observe one real camera and one viewer for at least 15 minutes; record sampled camera sequence, video readiness, decoded frames and process load | Source and decoded counters advance; no falsely live stale image; actual profile/encoder match the record. No unexplained error or consumer accumulation. |
| Camera disconnect / reconnect | In a supervised maintenance run, remove and restore that camera connection three times | Source becomes unavailable; native WHEP releases the old session and waits for fresh source. App hides the old image; a fresh generation and decoded frames establish recovery. Record elapsed times against the agreed budget. |
| Video process exit | With the selected recovery owner identified, terminate only the video process in a maintenance run | Diagnostics report the failure. Capture remains independently observable. Recovery follows the actual Product/supervisor contract; do not claim automatic fatal-process restart until demonstrated. |
| Network loss / recovery | Interrupt only the declared media network path, restore it, repeat three times | After connected playback loses decoded-frame progression, App withdraws the frame within the configured 3-second watchdog plus polling/render delay; no old device/frame is shown during retry. Recovery opens a new session. |
| Client close / background | Close the view, background/resume the App, then close it again; repeat ten cycles | Tracks and peers close, identified WHEP sessions are deleted, and consumers return to the pre-viewer baseline. Resume waits for newly decoded frames. |
| Device switch | Switch A→B→A while offers or frames are pending | A's late response cannot appear on B. Returning to A does not resurrect a retired association/session; explicit binding rules are respected. |
| Perception plus video | Record a stationary perception-only baseline, then repeat with the declared camera/video/viewer load | Compare source age, CPU/memory, perception timing and readiness with agreed budgets. No second camera owner, OOM, new persistent readiness failure or unbounded queue. Zero motion is submitted by this test. |
| Viewing authorization | Test one authorized viewer, an unauthenticated viewer and an unauthorized viewer against the deployed boundary | Only the authorized viewer can read status/SDP/media as specified; credentials are not exposed to another origin. Current unprotected Gateway/App configuration cannot be marked passed. |

Do not use status freshness or encoded queue metrics as capture-to-screen
latency. Measure a physical event/counter and the receiving display together,
and retain the method and samples. Browser/App rendering success does not
certify camera-to-body calibration or navigation motion readiness.

## Release decision

A deliverable includes the target-specific native release, effective device
configuration, supported client build, diagnostic output and completed
acceptance record. Until the target row has real evidence, label it a candidate.
Current next baseline is Go2/NX with its existing RealSense path; Thunder's
GS130W path requires adaptation and qualification before it joins the same
matrix. No hardware was altered to create this document.
