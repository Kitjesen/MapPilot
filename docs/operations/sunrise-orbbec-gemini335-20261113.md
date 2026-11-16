# Sunrise Orbbec Gemini 335 deployment

Target: `sunrise@192.168.66.64`, Linux aarch64, 2026-11-13.

## What changed

- Confirmed the attached USB camera is Orbbec Gemini 335 (`2bc5:0800`), not GS130W MIPI and not an enumerated L355.
- Built the repository's `orbbec_native` capture binary and `lingtu_camera_dds` on the target architecture with Orbbec SDK v2.8.7 arm64.
- Installed the Gemini 335 udev rule so the Orbbec SDK can open the USB device.
- Installed `/opt/lingtu/current/bin/orbbec_capture`, `/opt/lingtu/current/bin/lingtu_camera_dds`, and the SDK runtime under `/opt/lingtu/current/lib`.
- Disabled `rdk-gs130w-camera.service`.
- Started `lt-camera.service` with the native Orbbec RGB-D path.

## Evidence

The standalone native capture first failed with `usbEnumerator openUsbDevice failed` while the USB device node was `root:root 0644`. After installing `99-lingtu-orbbec-gemini335.rules`, the same binary produced a valid RGB-D record and exited successfully.

The running `lt-camera.service` status file reported advancing streams:

```json
{
  "status": "running",
  "color_frames": 3834,
  "depth_frames": 3845,
  "info_frames": 128,
  "color_shm_sequence": 3834,
  "depth_shm_sequence": 3845,
  "info_shm_sequence": 128,
  "last_error": ""
}
```

The service uses POSIX SHM for color/depth/intrinsics and does not publish typed image DDS by default. A direct V4L2 fallback image is retained separately at `artifacts/sunrise-camera/orbbec-gemini335.jpg`; that image was captured before the native deployment and is not proof of the LingTu SHM path.

## Current boundary

The bottom and middle camera layers are running on Sunrise: USB → Orbbec SDK → native record stream → `lingtu_camera_dds` → color/depth/intrinsics SHM. Sunrise does not currently have the Python Host/Gateway dependencies or LingTu WebRTC binary installed, so the App/MCP/WHEP upper path is not yet connected to this remote SHM. The next deployment must add the selected Host/video receiver on the same machine or an explicit network bridge; POSIX SHM is local-only.



The target also built `lingtu_camera_video` with the installed FFmpeg/libx264 path. A 30-frame H.264 segment was encoded from `/lingtu_camera_color` and decoded with FFmpeg successfully (`video_rc=0`, `decode_rc=0`). The target GStreamer is 1.20.3, below the native WebRTC build's declared >=1.24 requirement, so the native `lt-video.service` remains unconfigured.

## Read-only upper deployment

On 2026-11-13 a separate, motion-free camera path was deployed to the target:

- `/usr/local/bin/go2rtc` (official `v1.9.10` arm64 release, SHA-256 verified before install).
- `/etc/go2rtc/go2rtc.yaml`, with `cam` supplied by the existing color SHM through
  `/opt/lingtu/current/bin/lingtu_camera_video` and `libx264`.
- `go2rtc.service`, listening on loopback HTTP `1984` and WebRTC transport `8555`.
- `/home/sunrise/.venv-lingtu-gateway` with the Gateway HTTP dependencies.
- `lt-video-gateway.service`, listening on `0.0.0.0:5050` and running the unmanaged
  `gateway.video_probe` in read-only mode.

The probe exposes the App-facing bootstrap, camera status, WebRTC status and WHEP
signalling routes without starting a Product, Brainstem, navigation process or
teleop owner. It reads `/dev/shm/lingtu/camera_status.json` as a file-backed health
projection. Remote HTTP checks returned `200` for both `/api/v1/camera/status` and
`/api/v1/webrtc/status`; the camera status correctly reported `no_color_frames`
while the USB camera was absent. This proves the upper control plane is installed,
not that a decoded frame has been received.

The actual App-to-WebRTC frame check remains pending until the Gemini 335 is
reconnected: the target currently has no `2bc5:0800` USB enumeration and the native
camera service is therefore waiting/restarting. Motion control is intentionally not
part of this deployment because the target Brainstem service is currently failed and
must be repaired separately before a control Gateway is allowed to own commands.

During the follow-up check the camera briefly produced more than 84,000 color
frames and the Gateway reported `streaming`. It then lost the source again and the
native status returned `camera record exceeded absolute read deadline`; the systemd
camera unit stayed enabled and continued its restart/wait policy. The field next
step is to stabilize the Gemini USB connection or power before judging the App
receiver.
 
## WebRTC pacing correction
 
The first sidecar version fed raw Annex-B H.264 directly to Go2RTC. The software
encoder produced valid H.264, but the pipe carried no presentation timestamps;
Go2RTC therefore delivered very few paced media packets and the browser decoded
only roughly 1--3 fps even though the source camera was above 30 Hz.
 
The current source tree uses the shorter supervisor
`/opt/lingtu/current/bin/cam_stream.py`. It keeps one SHM reader and software
encoder, then repackages H.264 with a 30-fps time base before Go2RTC. A fresh
local Edge check after the pacing change reported 30 decoded fps,
`packets_lost=0` and approximately 13 ms WebRTC RTT. The final replacement of
the older remote wrapper is pending until Sunrise is reachable again.
