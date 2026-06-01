"""Unit tests for WebRTCStreamModule — signalling-layer helpers only.

The full offer/answer dance requires a running asyncio loop + aiortc
PeerConnection, which is covered by manual E2E against a real browser.
Here we exercise the SDP reorder helper (iOS Safari compat) and the
module's graceful-degradation path when aiortc is absent.
"""

from __future__ import annotations

import numpy as np
import pytest

try:
    from webrtc.webrtc_stream_module import (
        WebRTCStreamModule,
        _force_h264_first,
        _HAVE_WEBRTC,
    )
except ImportError:  # pragma: no cover — module itself should always import
    pytest.skip("WebRTCStreamModule not importable", allow_module_level=True)


def test_force_h264_first_moves_h264_payload_type_to_front():
    sdp = "\r\n".join([
        "v=0",
        "m=video 9 UDP/TLS/RTP/SAVPF 96 97 98",
        "a=rtpmap:96 VP8/90000",
        "a=rtpmap:97 VP9/90000",
        "a=rtpmap:98 H264/90000",
    ])
    out = _force_h264_first(sdp)
    m_line = next(line for line in out.split("\r\n") if line.startswith("m=video"))
    assert m_line == "m=video 9 UDP/TLS/RTP/SAVPF 98 96 97"


def test_force_h264_first_is_noop_when_no_h264_offered():
    sdp = "\r\n".join([
        "v=0",
        "m=video 9 UDP/TLS/RTP/SAVPF 96 97",
        "a=rtpmap:96 VP8/90000",
        "a=rtpmap:97 VP9/90000",
    ])
    assert _force_h264_first(sdp) == sdp


def test_force_h264_first_handles_multiple_h264_payload_types():
    # aiortc commonly advertises H264 under two PTs (constrained baseline
    # + baseline).  Both should bubble to the front in declared order.
    sdp = "\r\n".join([
        "v=0",
        "m=video 9 UDP/TLS/RTP/SAVPF 96 100 102 97",
        "a=rtpmap:96 VP8/90000",
        "a=rtpmap:100 H264/90000",
        "a=rtpmap:102 H264/90000",
        "a=rtpmap:97 VP9/90000",
    ])
    out = _force_h264_first(sdp)
    m_line = next(line for line in out.split("\r\n") if line.startswith("m=video"))
    assert m_line == "m=video 9 UDP/TLS/RTP/SAVPF 100 102 96 97"


def test_module_instantiates_without_camera():
    mod = WebRTCStreamModule()
    health = mod.health()
    assert health["active_peers"] == 0
    assert health["has_frame"] is False
    assert health["enabled"] == _HAVE_WEBRTC


def test_image_callback_caches_latest_frame():
    mod = WebRTCStreamModule()

    class FakeImage:
        def __init__(self, arr: np.ndarray) -> None:
            self.data = arr

    frame = np.zeros((48, 64, 3), dtype=np.uint8)
    frame[..., 1] = 255  # green
    mod._on_image(FakeImage(frame))
    assert mod._latest_bgr is not None
    assert mod._latest_bgr.shape == (48, 64, 3)
    assert int(mod._latest_bgr[0, 0, 1]) == 255


def test_max_bitrate_precedence_param_over_env(monkeypatch):
    monkeypatch.setenv("LINGTU_WEBRTC_BITRATE", "800000")
    mod = WebRTCStreamModule(max_bitrate=1_200_000)
    # Explicit argument wins over the environment variable.
    assert mod._max_bitrate == 1_200_000


def test_max_bitrate_env_fallback(monkeypatch):
    monkeypatch.setenv("LINGTU_WEBRTC_BITRATE", "900000")
    mod = WebRTCStreamModule()
    assert mod._max_bitrate == 900_000


def test_max_bitrate_bogus_env_falls_back_to_default(monkeypatch):
    monkeypatch.setenv("LINGTU_WEBRTC_BITRATE", "not-a-number")
    mod = WebRTCStreamModule()
    assert mod._max_bitrate == 2_500_000


def test_collect_stats_idle_returns_zero_peers():
    import asyncio

    mod = WebRTCStreamModule(max_bitrate=1_000_000)
    stats = asyncio.run(mod.collect_stats())
    # `enabled` mirrors `_HAVE_WEBRTC` — in CI without aiortc installed it's
    # False. Either way, the shape of the dict and the idle-peer invariants
    # must hold.
    assert stats["enabled"] == _HAVE_WEBRTC
    assert stats["active_peers"] == 0
    assert stats["max_bitrate"] == 1_000_000
    assert stats["peers"] == []
