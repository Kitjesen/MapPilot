"""Contract tests for WebRTCStreamModule.

Verifies module instantiation, port registration, config parameter handling,
and lifecycle (no actual WebRTC connection needed).

These are CONTRACT tests 鈥?they verify the module interface contract, not
internal implementation details or algorithmic correctness.
"""
from __future__ import annotations

import os



# =============================================================================
# WebRTCStreamModule
# =============================================================================

class TestWebRTCStreamModule:
    """Contract tests for WebRTCStreamModule (layer=6, WebRTC H.264 video)."""

    def test_instantiation(self):
        """Creating a WebRTCStreamModule with default params should succeed."""
        from gateway.media.webrtc_stream import WebRTCStreamModule

        mod = WebRTCStreamModule()
        assert mod._max_bitrate == 2_500_000
        assert mod._latest_bgr is None
        assert len(mod._pcs) == 0
        assert mod._start_ts > 0

    def test_instantiation_with_custom_bitrate(self):
        """Custom max_bitrate must be reflected in module state."""
        from gateway.media.webrtc_stream import WebRTCStreamModule

        mod = WebRTCStreamModule(max_bitrate=1_000_000)
        assert mod._max_bitrate == 1_000_000

    def test_instantiation_with_env_bitrate(self):
        """LINGTU_WEBRTC_BITRATE env var must override default bitrate."""
        from gateway.media.webrtc_stream import WebRTCStreamModule

        os.environ["LINGTU_WEBRTC_BITRATE"] = "500000"
        try:
            mod = WebRTCStreamModule()
            assert mod._max_bitrate == 500_000
        finally:
            del os.environ["LINGTU_WEBRTC_BITRATE"]

    def test_env_bitrate_does_not_override_explicit_param(self):
        """Explicit max_bitrate parameter must take precedence over env var."""
        from gateway.media.webrtc_stream import WebRTCStreamModule

        os.environ["LINGTU_WEBRTC_BITRATE"] = "100000"
        try:
            mod = WebRTCStreamModule(max_bitrate=3_000_000)
            assert mod._max_bitrate == 3_000_000
        finally:
            del os.environ["LINGTU_WEBRTC_BITRATE"]

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from gateway.media.webrtc_stream import WebRTCStreamModule
        from runtime.msgs.sensor import Image

        mod = WebRTCStreamModule()

        # -- Input ports --
        assert "color_image" in mod._ports_in
        assert mod._ports_in["color_image"].msg_type is Image
        assert len(mod._ports_in) == 1, (
            f"expected 1 In port, got {list(mod._ports_in)}"
        )

        # -- Output ports --
        # WebRTCStreamModule has no Out ports 鈥?it only consumes images
        assert len(mod._ports_out) == 0, (
            f"expected 0 Out ports, got {list(mod._ports_out)}"
        )

    def test_lifecycle_setup_teardown(self):
        """setup() and teardown() must transition without error."""
        from gateway.media.webrtc_stream import WebRTCStreamModule

        mod = WebRTCStreamModule()

        # setup subscribes to color_image port
        mod.setup()
        assert "color_image" in mod._ports_in
        assert mod._ports_in["color_image"]._policy == "latest"

        # teardown cleans up peer connections (none in test, no error)
        mod.teardown()

    def test_on_image_does_not_crash_with_invalid_data(self):
        """_on_image must handle None data gracefully."""
        from gateway.media.webrtc_stream import WebRTCStreamModule
        from runtime.msgs.sensor import Image, ImageFormat

        mod = WebRTCStreamModule()
        mod.setup()

        # Image with None data should not crash (the method is defensive)
        img = Image(data=None, format=ImageFormat.BGR)
        mod._on_image(img)  # should not raise

    def test_health_returns_module_info(self):
        """health() must return a dict with expected keys."""
        from gateway.media.webrtc_stream import WebRTCStreamModule

        mod = WebRTCStreamModule()
        info = mod.health()
        assert isinstance(info, dict)
        # WebRTCStreamModule._HAVE_WEBRTC may be False without aiortc,
        # but health should still return expected base keys
        assert "enabled" in info
        assert "active_peers" in info
        assert "has_frame" in info

    def test_on_system_modules_no_gateway(self):
        """on_system_modules must handle missing GatewayModule gracefully."""
        from gateway.media.webrtc_stream import WebRTCStreamModule

        mod = WebRTCStreamModule()
        # Passing empty modules dict 鈥?no GatewayModule available
        mod.on_system_modules({})
        assert mod._gateway is None
        # Should not crash, just log a warning

    def test_teardown_without_setup(self):
        """Calling teardown() without prior setup() must not crash."""
        from gateway.media.webrtc_stream import WebRTCStreamModule

        mod = WebRTCStreamModule()
        # _loop is None, teardown checks and returns early
        mod.teardown()
