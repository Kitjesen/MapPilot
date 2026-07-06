"""Gateway media package -WebRTC/video streaming support.

The implementation lives in :mod:`gateway.media.webrtc_stream`. This
``__init__`` only re-exports the public symbol so ``gateway.media`` can be
imported as a package without duplicating the ``@register("webrtc", "aiortc")``
registration (duplicating the class body here would double-register the
plugin if this package is ever imported directly instead of the submodule).
"""

from __future__ import annotations

from gateway.media.webrtc_stream import WebRTCStreamModule

__all__ = ["WebRTCStreamModule"]
