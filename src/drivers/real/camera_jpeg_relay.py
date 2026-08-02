"""Camera-only JPEG relay for Gateway WebSocket and snapshot fallbacks.

This module intentionally has no joystick input, velocity output, teleop state,
or motion RPC. Field Products use it instead of ``TeleopModule`` so the Python
Host graph cannot become a second motion authority.
"""

from __future__ import annotations

import logging
import threading
from typing import Any

from runtime.module import Module
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import Image
from runtime.registry import register
from runtime.stream import In

logger = logging.getLogger(__name__)


@register("media", "jpeg_relay", description="Camera-only JPEG relay for Gateway")
class CameraJpegRelayModule(Module, layer=6):
    """Encode camera frames for Gateway without exposing motion controls."""

    color_image: In[Image]
    scene_graph: In[SceneGraph]

    def __init__(
        self,
        jpeg_quality: int = 60,
        stream_fps: float = 10.0,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._jpeg_quality = jpeg_quality
        self._stream_interval = 1.0 / max(1.0, stream_fps)
        self._gateway: Any | None = None
        self._clients = 0
        self._camera_clients = 0
        self._running = False
        self._encode_thread: threading.Thread | None = None
        self._latest_raw: Any | None = None
        self._raw_lock = threading.Lock()
        self._new_frame = threading.Event()
        self._latest_detections: list[Any] = []
        self._det_lock = threading.Lock()
        try:
            from runtime.config import get_config

            self._cam_rotate = int(get_config().raw.get("camera", {}).get("rotate", 0))
        except (KeyError, TypeError, ValueError):
            self._cam_rotate = 0

    def setup(self) -> None:
        self.color_image.subscribe(self._on_image)
        self.color_image.set_policy("latest")
        self.scene_graph.subscribe(self._on_scene_graph)
        self.scene_graph.set_policy("latest")

    def start(self) -> None:
        super().start()
        self._running = True
        self._encode_thread = threading.Thread(
            target=self._encode_loop,
            name="camera-jpeg-relay",
            daemon=True,
        )
        self._encode_thread.start()
        logger.info("CameraJpegRelayModule started")

    def stop(self) -> None:
        self._running = False
        self._new_frame.set()
        if self._encode_thread:
            self._encode_thread.join(timeout=3.0)
        self._gateway = None
        super().stop()

    def on_system_modules(self, modules: dict[str, Any]) -> None:
        gateway = modules.get("GatewayModule")
        if gateway is None:
            logger.warning("CameraJpegRelayModule: GatewayModule not found")
            return
        self._gateway = gateway
        gateway._camera_module = self
        logger.info("CameraJpegRelayModule: wired into GatewayModule")

    def on_client_connect(self) -> None:
        """Register a teleop-socket viewer without acquiring motion control."""
        self._clients += 1
        self._wake_encoder_if_ready()

    def on_client_disconnect(self) -> None:
        self._clients = max(0, self._clients - 1)

    def on_camera_client_connect(self) -> None:
        self._camera_clients += 1
        self._wake_encoder_if_ready()

    def on_camera_client_disconnect(self) -> None:
        self._camera_clients = max(0, self._camera_clients - 1)

    def _wake_encoder_if_ready(self) -> None:
        if self._latest_raw is not None:
            self._new_frame.set()

    def _on_scene_graph(self, scene_graph: SceneGraph) -> None:
        try:
            with self._det_lock:
                self._latest_detections = list(scene_graph.objects) if scene_graph.objects else []
        except (AttributeError, TypeError):
            logger.debug("CameraJpegRelayModule: scene graph callback error", exc_info=True)

    def _draw_detections(self, frame: Any, cv2: Any) -> Any:
        with self._det_lock:
            detections = list(self._latest_detections)
        if not detections:
            return frame
        height, width = frame.shape[:2]
        for detection in detections:
            bbox = getattr(detection, "bbox_2d", None) or getattr(detection, "bbox", None)
            if bbox is None or not hasattr(bbox, "__len__") or len(bbox) < 4:
                continue
            x1, y1, x2, y2 = (int(bbox[index]) for index in range(4))
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 136), 2)
            label = getattr(detection, "label", "")
            confidence = getattr(detection, "confidence", 0.0)
            text = f"{label} {confidence:.0%}" if confidence > 0 else label
            if text:
                text_width, text_height = cv2.getTextSize(
                    text,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    1,
                )[0]
                cv2.rectangle(
                    frame,
                    (x1, y1 - text_height - 6),
                    (x1 + text_width + 4, y1),
                    (0, 255, 136),
                    -1,
                )
                cv2.putText(
                    frame,
                    text,
                    (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 0),
                    1,
                )
        return frame

    def _on_image(self, image: Image) -> None:
        if self._gateway is None:
            return
        with self._raw_lock:
            self._latest_raw = image.to_bgr().data
        if self._clients + self._camera_clients > 0:
            self._new_frame.set()

    def _rotate(self, frame: Any, cv2: Any) -> Any:
        rotations = {
            90: cv2.ROTATE_90_CLOCKWISE,
            180: cv2.ROTATE_180,
            270: cv2.ROTATE_90_COUNTERCLOCKWISE,
        }
        if self._cam_rotate in rotations:
            return cv2.rotate(frame, rotations[self._cam_rotate])
        return frame

    def _encode(self, raw: Any, cv2: Any) -> bytes | None:
        frame = self._rotate(self._draw_detections(raw.copy(), cv2), cv2)
        ok, buffer = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
        )
        return buffer.tobytes() if ok else None

    def snapshot_jpeg(self) -> bytes | None:
        with self._raw_lock:
            raw = None if self._latest_raw is None else self._latest_raw.copy()
        if raw is None:
            return None
        try:
            import cv2
        except ImportError:
            return None
        try:
            data = self._encode(raw, cv2)
            if data is not None and self._gateway is not None:
                self._gateway.push_jpeg(data)
            return data
        except Exception:
            logger.debug("CameraJpegRelayModule: snapshot encode failed", exc_info=True)
            return None

    def _encode_loop(self) -> None:
        try:
            import cv2
        except ImportError:
            logger.warning("CameraJpegRelayModule: cv2 unavailable; JPEG fallback disabled")
            cv2 = None
        while self._running:
            triggered = self._new_frame.wait(timeout=self._stream_interval)
            self._new_frame.clear()
            if not triggered or not self._running or cv2 is None:
                continue
            gateway = self._gateway
            if gateway is None:
                continue
            with self._raw_lock:
                raw = self._latest_raw
            if raw is None:
                continue
            try:
                data = self._encode(raw, cv2)
                if data is not None:
                    gateway.push_jpeg(data)
            except Exception:
                logger.debug("CameraJpegRelayModule: encode iteration failed", exc_info=True)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info.update(
            {
                "role": "camera_jpeg_relay",
                "motion_capable": False,
                "teleop_clients": self._clients,
                "camera_clients": self._camera_clients,
                "stream_clients": self._clients + self._camera_clients,
            }
        )
        return info
