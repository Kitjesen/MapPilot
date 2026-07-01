"""WebRTCStreamModule 鈥?browser video over WebRTC.

Consumes the CameraBridge ``color_image`` stream (BGR8 numpy) and exposes
it as a WebRTC video track.  Browsers connect through a single
``POST /api/v1/webrtc/offer`` signalling endpoint served by
GatewayModule's FastAPI app; the module plugs its routes into the
existing uvicorn process so we stay on port 5050.

Why this over JPEG-over-WebSocket
---------------------------------
JPEG-over-WS is ~4 Mbit/s and 200-400 ms glass-to-glass because every
frame is re-encoded as a standalone image and decoded synchronously on
the browser's main thread.  WebRTC with H.264 is ~1-2 Mbit/s, uses the
browser's hardware decoder, and runs below 150 ms on a LAN.

Encoder path
------------
Phase A (this file) uses PyAV's libx264 鈥?pure software, ~30 % of one
A78AE core at 720p30.  Phase B will swap to D-Robotics ``hobot_codec``
(BPU H.264 at 5.5 ms/frame, <5 % CPU) by feeding pre-encoded NALUs
through aiortc's ``H264PayloadContext`` without changing the wire
protocol or the browser side.

Fallback
--------
If aiortc / PyAV are unavailable at import, the module becomes a no-op
so the rest of the stack still boots 鈥?the JPEG-over-WS path remains
the only video route.
"""

from __future__ import annotations

import asyncio
import logging
import os
import threading
import time
from typing import Any

from runtime.module import Module
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Image
from runtime.registry import register
from runtime.stream import In

logger = logging.getLogger(__name__)


try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
    from aiortc.contrib.media import MediaRelay
    from av import VideoFrame
    _HAVE_WEBRTC = True
except ImportError as _exc:  # pragma: no cover 鈥?exercised by deployment env
    logger.info("WebRTCStreamModule disabled: %s", _exc)
    _HAVE_WEBRTC = False


if _HAVE_WEBRTC:

    class _LatestFrameTrack(VideoStreamTrack):
        """aiortc track that yields whatever the producer last wrote.

        ``recv()`` awaits an asyncio.Event poked from the Module's
        producer thread; this rate-limits the encoder to the source's
        framerate (30 Hz) without any polling.  Multiple subscribers
        share one encoded stream via :class:`MediaRelay` upstream.
        """

        kind = "video"

        def __init__(self, module: "WebRTCStreamModule") -> None:
            super().__init__()
            self._mod = module

        async def recv(self):  # type: ignore[override]
            await self._mod._frame_event.wait()
            self._mod._frame_event.clear()
            bgr = self._mod._latest_bgr
            if bgr is None:
                return await self.recv()
            pts, time_base = await self.next_timestamp()
            try:
                frame = VideoFrame.from_ndarray(bgr, format="bgr24")
            except Exception as e:
                n = getattr(self, "_recv_err_count", 0) + 1
                self._recv_err_count = n
                if n == 1 or n % 50 == 0:
                    logger.warning("VideoFrame.from_ndarray failed (#%d) shape=%s dtype=%s: %s",
                                   n, getattr(bgr, "shape", None), getattr(bgr, "dtype", None), e)
                return await self.recv()
            frame.pts = pts
            frame.time_base = time_base
            n = getattr(self, "_recv_count", 0) + 1
            self._recv_count = n
            if n == 1 or n % 100 == 0:
                logger.info("WebRTC track.recv: served frame #%d shape=%s", n, bgr.shape)
            return frame


def _force_h264_first(sdp: str) -> str:
    """Move H.264 payload types ahead of VP8/VP9 in every m=video line.

    iOS Safari only uses the first codec it recognises; leaving VP8 first
    means iPhones get a black tile.  We keep all codecs so desktop
    browsers still negotiate something useful if H.264 fails.
    """
    lines = sdp.split("\r\n")
    out: list[str] = []
    h264_pts: list[str] = []
    # First pass: discover which payload types map to H264.
    for line in lines:
        if line.startswith("a=rtpmap:") and "H264" in line:
            pt = line.split(":", 1)[1].split(" ", 1)[0]
            h264_pts.append(pt)
    if not h264_pts:
        return sdp
    for line in lines:
        if line.startswith("m=video "):
            head, _, tail = line.partition(" UDP/TLS/RTP/SAVPF ")
            if not tail:
                out.append(line)
                continue
            pts = tail.split(" ")
            reordered = [pt for pt in h264_pts if pt in pts] + [
                pt for pt in pts if pt not in h264_pts
            ]
            out.append(f"{head} UDP/TLS/RTP/SAVPF {' '.join(reordered)}")
        else:
            out.append(line)
    return "\r\n".join(out)


@register("webrtc", "aiortc", description="WebRTC H.264 video via aiortc + PyAV")
class WebRTCStreamModule(Module, layer=6):
    """Stream camera frames to the browser over WebRTC."""

    color_image: In[Image]

    _run_in_main: bool = False

    def __init__(
        self,
        *,
        max_bitrate: int | None = None,
        max_peers: int | None = None,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        # Bitrate ceiling applied to the outbound RTP stream via
        # RTCRtpSender.setParameters once the PeerConnection is alive.
        # Defaults tuned for 720p30 over a LAN; override with env var
        # for WAN / constrained networks.
        env_br = os.environ.get("LINGTU_WEBRTC_BITRATE")
        if max_bitrate is None and env_br:
            try:
                max_bitrate = int(env_br)
            except ValueError:
                logger.warning("ignoring invalid LINGTU_WEBRTC_BITRATE=%r", env_br)
        self._max_bitrate = max_bitrate or 2_500_000

        env_peers = os.environ.get("LINGTU_WEBRTC_MAX_PEERS")
        if max_peers is None and env_peers:
            try:
                max_peers = int(env_peers)
            except ValueError:
                logger.warning("ignoring invalid LINGTU_WEBRTC_MAX_PEERS=%r", env_peers)
        self._max_peers = max_peers if max_peers is not None else 4

        self._latest_bgr: np.ndarray | None = None
        self._raw_lock = threading.Lock()
        # asyncio primitives are created lazily on the gateway's event loop
        # (see on_system_modules / register_routes).
        self._frame_event: asyncio.Event | None = None  # type: ignore[assignment]
        self._loop: asyncio.AbstractEventLoop | None = None
        self._pcs: set = set()
        self._source_track: Any = None
        self._relay: Any = None
        self._gateway: Any = None
        self._start_ts = time.time()
        # Per-peer cached stats deltas 鈥?used to compute instantaneous bitrate
        # from RTCStatsReport byte counters (getStats reports cumulative only).
        self._prev_stats: dict[Any, tuple[float, int, int]] = {}

    # -- Module lifecycle ---------------------------------------------------

    def setup(self) -> None:
        self.color_image.subscribe(self._on_image)
        self.color_image.set_policy("latest")

    def on_system_modules(self, modules: dict[str, Any]) -> None:
        """Pick up the gateway reference; actual route is served by
        GatewayModule's ``/api/v1/webrtc/offer`` handler which delegates
        back to :meth:`handle_offer`.  Routes can't be added after the
        dashboard static mount, so this indirection keeps the single-port
        architecture intact.
        """
        self._gateway = modules.get("GatewayModule")
        if self._gateway is None:
            logger.warning(
                "WebRTCStreamModule: GatewayModule not found 鈥?signalling "
                "will not work",
            )

    def teardown(self) -> None:
        if self._loop is None:
            return
        for pc in list(self._pcs):
            try:
                asyncio.run_coroutine_threadsafe(pc.close(), self._loop)
            except Exception:
                pass
        self._pcs.clear()

    # -- Image callback (Module thread) -------------------------------------

    def _on_image(self, img: Image) -> None:
        data = getattr(img, "data", None)
        if data is None:
            return
        with self._raw_lock:
            self._latest_bgr = data
        # Instrumentation: every 100 frames log so we can confirm the wire
        # is live and roughly measure source FPS without per-frame noise.
        n = getattr(self, "_rx_count", 0) + 1
        self._rx_count = n
        if n == 1 or n % 100 == 0:
            logger.info("WebRTC _on_image: got frame #%d shape=%s", n, getattr(data, "shape", None))
        ev = self._frame_event
        loop = self._loop
        if ev is not None and loop is not None:
            loop.call_soon_threadsafe(ev.set)

    # -- Signalling (asyncio, called from GatewayModule route) -------------

    async def handle_offer(self, payload: dict) -> dict:
        """Single-shot offer/answer 鈥?~20 LOC, no TURN, no reconnection state.

        Expected payload: ``{"sdp": "...", "type": "offer"}``.
        Returns: ``{"sdp": "...", "type": "answer"}``.
        """
        if len(self._pcs) >= self._max_peers:
            raise ValueError(f"webrtc peer limit reached ({self._max_peers})")

        if self._loop is None:
            self._loop = asyncio.get_running_loop()
        if self._frame_event is None:
            self._frame_event = asyncio.Event()
        if self._source_track is None:
            self._source_track = _LatestFrameTrack(self)
            self._relay = MediaRelay()

        sdp = payload.get("sdp")
        kind = payload.get("type", "offer")
        if not isinstance(sdp, str) or kind != "offer":
            raise ValueError("webrtc offer requires {'sdp': str, 'type': 'offer'}")

        pc = RTCPeerConnection()
        self._pcs.add(pc)

        relayed = self._relay.subscribe(self._source_track)
        sender = pc.addTrack(relayed)

        @pc.on("connectionstatechange")
        async def _on_state() -> None:
            if pc.connectionState in ("failed", "closed", "disconnected"):
                try:
                    relayed.stop()
                except Exception as e:
                    logger.debug("relayed track stop failed: %s", e)
                await pc.close()
                self._pcs.discard(pc)

        await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        # Cap outbound bitrate 鈥?prevents libx264 from flooding a weak
        # uplink or the encoder's internal rate-control from spiking and
        # blowing the jitter buffer.  Browsers honour this via REMB/TWCC.
        try:
            from aiortc import RTCRtpSendParameters, RTCRtpEncodingParameters
            params = sender.getParameters() if hasattr(sender, "getParameters") else None
            if params is None:
                params = RTCRtpSendParameters(encodings=[RTCRtpEncodingParameters()])
            if not params.encodings:
                params.encodings = [RTCRtpEncodingParameters()]
            params.encodings[0].maxBitrate = self._max_bitrate
            await sender.setParameters(params)
        except Exception as e:
            logger.debug("setParameters maxBitrate failed: %s", e)

        return {
            "sdp": _force_h264_first(pc.localDescription.sdp),
            "type": pc.localDescription.type,
        }

    # -- Dynamic bitrate control --------------------------------------------

    async def set_max_bitrate(self, bps: int) -> dict:
        """Update encoder cap live; applies to every active video sender.

        Lets a user tune the cap from the browser without dropping the
        peer connection 鈥?useful for on-site bandwidth spot-checks.
        The new value also becomes the default for any subsequent offers.
        """
        bps = int(bps)
        if bps < 100_000 or bps > 10_000_000:
            raise ValueError(f"bitrate out of range: {bps} bps (allowed 100000..10000000)")
        self._max_bitrate = bps
        applied = 0
        for pc in list(self._pcs):
            for sender in pc.getSenders():
                if not sender.track or getattr(sender.track, "kind", "") != "video":
                    continue
                try:
                    params = sender.getParameters() if hasattr(sender, "getParameters") else None
                    if params is None or not getattr(params, "encodings", None):
                        continue
                    params.encodings[0].maxBitrate = bps
                    await sender.setParameters(params)
                    applied += 1
                except Exception as e:
                    logger.debug("set_max_bitrate sender update failed: %s", e)
        logger.info("WebRTC max bitrate set to %d bps (applied to %d sender%s)",
                    bps, applied, "" if applied == 1 else "s")
        return {"max_bitrate": self._max_bitrate, "applied_senders": applied}

    # -- Stats (polled by Gateway /api/v1/webrtc/stats) ---------------------

    async def collect_stats(self) -> dict:
        """Summarise live RTCPeerConnection stats.

        Returns per-peer bitrate / fps / encode time + an aggregate block
        suitable for rendering in a browser HUD.  Safe to call from the
        gateway's event loop 鈥?each getStats is awaited sequentially with
        a soft cap so a stuck peer can't freeze the response.
        """
        if self._loop is None:
            return {
                "enabled": _HAVE_WEBRTC,
                "active_peers": 0,
                "max_bitrate": self._max_bitrate,
                "encode_warning": False,
                "peers": [],
            }

        peers_out: list[dict] = []
        now = time.time()
        # list() copy so discard during iteration (failed PC cleanup) is safe.
        for pc in list(self._pcs):
            row: dict[str, Any] = {
                "state": getattr(pc, "connectionState", "unknown"),
                "ice_state": getattr(pc, "iceConnectionState", "unknown"),
                "bitrate_bps": 0,
                "fps": 0.0,
                "frames_encoded": 0,
                "encode_avg_ms": 0.0,
                "packets_lost": 0,
            }
            try:
                report = await asyncio.wait_for(pc.getStats(), timeout=0.25)
            except (asyncio.TimeoutError, Exception) as e:
                row["error"] = type(e).__name__
                peers_out.append(row)
                continue

            bytes_sent = 0
            packets_sent = 0
            frames_encoded = 0
            total_encode_time = 0.0
            packets_lost = 0
            for stat in report.values():
                kind = getattr(stat, "type", "")
                if kind == "outbound-rtp" and getattr(stat, "kind", "") == "video":
                    bytes_sent = int(getattr(stat, "bytesSent", 0) or 0)
                    packets_sent = int(getattr(stat, "packetsSent", 0) or 0)
                    frames_encoded = int(getattr(stat, "framesEncoded", 0) or 0)
                    total_encode_time = float(getattr(stat, "totalEncodeTime", 0.0) or 0.0)
                elif kind == "remote-inbound-rtp":
                    packets_lost = int(getattr(stat, "packetsLost", 0) or 0)

            prev = self._prev_stats.get(pc)
            if prev is not None:
                prev_t, prev_bytes, prev_frames = prev
                dt = max(now - prev_t, 1e-3)
                row["bitrate_bps"] = int((bytes_sent - prev_bytes) * 8 / dt)
                row["fps"] = round((frames_encoded - prev_frames) / dt, 1)
            self._prev_stats[pc] = (now, bytes_sent, frames_encoded)

            row["frames_encoded"] = frames_encoded
            row["packets_sent"] = packets_sent
            row["packets_lost"] = packets_lost
            if frames_encoded > 0:
                row["encode_avg_ms"] = round(total_encode_time / frames_encoded * 1000, 2)
            peers_out.append(row)

        # Drop stats history for closed peers to avoid unbounded growth.
        self._prev_stats = {pc: v for pc, v in self._prev_stats.items() if pc in self._pcs}

        agg_bitrate = sum(p["bitrate_bps"] for p in peers_out)
        agg_fps = max((p["fps"] for p in peers_out), default=0.0)
        enc_times = [p["encode_avg_ms"] for p in peers_out if p.get("encode_avg_ms")]
        agg_enc_ms = round(sum(enc_times) / len(enc_times), 1) if enc_times else 0.0
        return {
            "enabled": _HAVE_WEBRTC,
            "active_peers": len(peers_out),
            "max_bitrate": self._max_bitrate,
            "bitrate_bps": agg_bitrate,
            "fps": agg_fps,
            "encode_avg_ms": agg_enc_ms,
            "encode_warning": agg_enc_ms > 20,
            "uptime_s": round(now - self._start_ts, 1),
            "peers": peers_out,
        }

    # -- Diagnostics --------------------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary() if hasattr(super(), "port_summary") else {}
        info.update({
            "enabled": _HAVE_WEBRTC,
            "active_peers": len(self._pcs),
            "has_frame": self._latest_bgr is not None,
        })
        return info
