# Camera transport

The dashboard has two camera paths:

1. go2rtc WHEP for low-latency H.264 video.
2. Gateway JPEG-over-WebSocket (`/ws/camera`) as the reliable fallback.

The browser first probes `GET /api/v1/webrtc/go2rtc/status`. When the
sidecar is available it posts SDP to `POST /api/v1/webrtc/whep`; media then
flows directly between go2rtc and the browser. If the probe, signalling, or
ICE setup fails, `CameraFeed` opens `/ws/camera`.

The Gateway bootstrap advertises WHEP support, not live sidecar health. Use
the go2rtc status endpoint for the current runtime state.

## Setup

```bash
sudo bash scripts/webrtc/install_go2rtc.sh
sudo systemctl restart go2rtc
curl -s http://localhost:5050/api/v1/webrtc/go2rtc/status
```

The sidecar configuration template is `config/go2rtc.yaml`. It binds the
go2rtc API to loopback so browsers reach it through the Gateway WHEP proxy.

## Troubleshooting

- Dashboard shows `MJPEG`: inspect the go2rtc status endpoint and service log.
- WHEP connects but no video arrives: inspect the peer in
  `chrome://webrtc-internals/` and verify the configured camera device.
- Snapshot consumers should keep using `GET /api/v1/camera/snapshot`; that API
  is independent from the dashboard streaming transport.

The removed in-process Python media path is intentionally not a fallback.
Keeping one WebRTC implementation avoids duplicate signalling, peer state,
and encoder ownership in the Gateway process.
