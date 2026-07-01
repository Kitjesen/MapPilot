"""TeleopModule 鈥?joystick remote control with live camera stream.

All teleop state and logic lives here.  GatewayModule only forwards raw
WebSocket messages to this module via the ``joy_input`` port.

Responsibilities:
  - Scale joystick inputs 鈫?Twist
  - Manage teleop active/idle state (3s idle 鈫?auto-release)
  - Publish ``teleop_active`` so Navigation can pause/resume
  - Encode camera frames 鈫?JPEG 鈫?push to GatewayModule
  - Provide @skill methods for REPL / MCP

Protocol (handled by GatewayModule):
  Client 鈫?server  JSON text:
    {"type": "joy",  "lx": 0.5, "ly": 0.0, "az": -0.3}
    {"type": "stop"}

Camera viewers use /ws/camera. Legacy teleop clients can still request
JPEG frames with /ws/teleop?video=1, but normal /ws/teleop is control-only.

Ports:
  In:  color_image (Image)  鈥?camera frames to encode + forward
       joy_input   (dict)   鈥?raw joystick message from GatewayModule WS
  Out: cmd_vel     (Twist)  鈥?scaled joystick 鈫?VelocityMux
       teleop_active (bool) 鈥?True while joystick is active
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from runtime.module import Module, skill
from runtime.msgs.geometry import Twist, Vector3
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import Image
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


@register("teleop", "default", description="Joystick teleop + camera stream via Gateway WS")
class TeleopModule(Module, layer=6):
    """Joystick remote control + camera stream.

    Teleop state (active/idle/release) is managed entirely here.
    GatewayModule forwards raw WS joy messages via ``joy_input`` port.
    cmd_vel is published to VelocityMux (not directly to the driver).
    """

    # -- Inputs --
    color_image: In[Image]
    joy_input:   In[dict]      # {"lx": float, "ly": float, "az": float}
    scene_graph: In[SceneGraph] # detection overlay (optional, from PerceptionModule)

    # -- Outputs --
    cmd_vel:        Out[Twist]
    teleop_active:  Out[bool]   # True = joystick active, False = released

    def __init__(
        self,
        max_speed: float = 0.5,
        max_yaw_rate: float = 1.0,
        release_timeout: float = 3.0,
        jpeg_quality: int = 60,
        stream_fps: float = 10.0,
        port: int = 5050,
        **kw,
    ):
        super().__init__(**kw)
        self._max_speed = max_speed
        self._max_yaw_rate = max_yaw_rate
        self._release_timeout = release_timeout
        self._jpeg_quality = jpeg_quality
        self._stream_interval = 1.0 / max(1.0, stream_fps)
        self._port = port

        # Teleop state
        self._active: bool = False
        self._last_joy_time: float = 0.0
        self._clients: int = 0
        self._camera_clients: int = 0

        # Gateway reference (for camera push + client count)
        self._gateway = None

        # Camera encoding
        self._encode_thread: threading.Thread | None = None

        # Camera rotation from config (degrees: 0/90/180/270)
        try:
            from runtime.config import get_config
            self._cam_rotate = int(get_config().raw.get("camera", {}).get("rotate", 0))
        except (KeyError, TypeError, ValueError):
            self._cam_rotate = 0
        self._idle_thread: threading.Thread | None = None

        # Detection overlay
        self._latest_detections: list = []
        self._det_lock = threading.Lock()
        self._running = False
        self._latest_raw: Any | None = None
        self._raw_lock = threading.Lock()
        self._new_frame = threading.Event()

    # -- lifecycle ----------------------------------------------------------

    def setup(self) -> None:
        self.color_image.subscribe(self._on_image)
        self.color_image.set_policy("latest")
        self.joy_input.subscribe(self._on_joy)
        self.scene_graph.subscribe(self._on_scene_graph)
        self.scene_graph.set_policy("latest")

    def start(self) -> None:
        super().start()
        self._running = True
        self._encode_thread = threading.Thread(
            target=self._encode_loop, name="teleop-encode", daemon=True
        )
        self._encode_thread.start()
        self._idle_thread = threading.Thread(
            target=self._idle_check_loop, name="teleop-idle", daemon=True
        )
        self._idle_thread.start()
        logger.info("TeleopModule started (JPEG encoder + idle checker active)")

    def stop(self) -> None:
        self._running = False
        self._new_frame.set()
        if self._encode_thread:
            self._encode_thread.join(timeout=3.0)
        if self._idle_thread:
            self._idle_thread.join(timeout=2.0)
        if self._active:
            self._release()
        self._gateway = None
        super().stop()

    def on_system_modules(self, modules: dict[str, Any]) -> None:
        """Inject GatewayModule reference for camera push + config sharing."""
        gw = modules.get("GatewayModule")
        if gw is not None:
            self._gateway = gw
            # Share config so GatewayModule knows speed limits for display
            if hasattr(gw, "configure_teleop"):
                gw.configure_teleop(
                    max_speed=self._max_speed,
                    max_yaw=self._max_yaw_rate,
                    release_timeout=self._release_timeout,
                )
            # Give GatewayModule a reference to us for teleop state queries
            gw._teleop_module = self
            logger.info("TeleopModule: wired into GatewayModule")
        else:
            logger.warning(
                "TeleopModule: GatewayModule not found 鈥?"
                "teleop WebSocket will not be available"
            )

    # -- joystick handling --------------------------------------------------

    def _on_joy(self, msg: dict) -> None:
        """Process raw joystick input from GatewayModule WS handler."""
        lx = max(-1.0, min(1.0, float(msg.get("lx", 0)))) * self._max_speed
        ly = max(-1.0, min(1.0, float(msg.get("ly", 0)))) * self._max_speed
        az = max(-1.0, min(1.0, float(msg.get("az", 0)))) * self._max_yaw_rate

        twist = Twist(
            linear=Vector3(x=lx, y=ly, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=az),
        )
        self.cmd_vel.publish(twist)
        self._last_joy_time = time.monotonic()

        if not self._active:
            self._active = True
            self.teleop_active.publish(True)
            logger.info("TeleopModule: teleop engaged")

    def _release(self) -> None:
        """Release teleop control 鈥?publish zero velocity + active=False."""
        if self._active:
            self._active = False
            self.cmd_vel.publish(Twist())
            self.teleop_active.publish(False)
            logger.info("TeleopModule: teleop released")

    def on_client_connect(self) -> None:
        """Called by GatewayModule when a teleop WS client connects."""
        self._clients += 1
        self._wake_encoder_if_ready()

    def on_client_disconnect(self) -> None:
        """Called by GatewayModule when a teleop WS client disconnects."""
        self._clients = max(0, self._clients - 1)
        if self._clients == 0:
            self._release()

    def on_camera_client_connect(self) -> None:
        """Register a camera-only viewer without acquiring teleop control."""
        self._camera_clients += 1
        self._wake_encoder_if_ready()

    def on_camera_client_disconnect(self) -> None:
        """Release a camera-only viewer without touching teleop state."""
        self._camera_clients = max(0, self._camera_clients - 1)

    def _wake_encoder_if_ready(self) -> None:
        # Kick the encoder immediately so the new client gets the most recent
        # cached raw frame encoded + pushed on the first send-loop iteration.
        if self._latest_raw is not None:
            self._new_frame.set()

    # -- idle timeout -------------------------------------------------------

    def _check_idle(self) -> None:
        """One-shot idle check: release teleop if joystick has gone quiet
        beyond the release timeout. Pulled out of the loop so tests can
        invoke a single tick deterministically.
        """
        if (self._active
                and time.monotonic() - self._last_joy_time > self._release_timeout):
            self._release()

    def _idle_check_loop(self) -> None:
        """Background thread: release teleop if joystick goes quiet."""
        while self._running:
            time.sleep(0.5)
            self._check_idle()

    # -- detection overlay ----------------------------------------------------

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        """Cache latest detections for overlay drawing."""
        try:
            with self._det_lock:
                self._latest_detections = list(sg.objects) if sg.objects else []
        except (AttributeError, TypeError):
                logger.debug("TeleopModule: scene graph callback error", exc_info=True)

    def _draw_detections(self, frame, cv2):
        """Draw bounding boxes + labels on the frame (in-place)."""
        with self._det_lock:
            dets = list(self._latest_detections)
        if not dets:
            return frame
        h, w = frame.shape[:2]
        for det in dets:
            bbox = getattr(det, "bbox_2d", None) or getattr(det, "bbox", None)
            label = getattr(det, "label", "")
            conf = getattr(det, "confidence", 0.0)
            if bbox is None:
                continue
            if hasattr(bbox, "__len__") and len(bbox) >= 4:
                x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
            else:
                continue
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 136), 2)
            text = f"{label} {conf:.0%}" if conf > 0 else label
            if text:
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), (0, 255, 136), -1)
                cv2.putText(frame, text, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        return frame

    # -- camera frame handling ----------------------------------------------

    def _on_image(self, img: Image) -> None:
        if self._gateway is None:
            return
        # Always cache the latest raw frame, even when no clients are connected,
        # so that a freshly-connecting client can immediately get an encoded
        # frame (instead of waiting ~33ms for the next camera tick and seeing
        # the Dashboard flash "offline / reconnect" initially).
        with self._raw_lock:
            self._latest_raw = img.data
        # Only wake the encoder thread when a client is connected 鈥?skipping
        # the JPEG encode step keeps CPU usage low in idle state.
        if self._clients + self._camera_clients > 0:
            self._new_frame.set()

    def snapshot_jpeg(self) -> bytes | None:
        """Encode the latest raw camera frame for one-shot HTTP snapshots."""
        with self._raw_lock:
            raw = None if self._latest_raw is None else self._latest_raw.copy()
        if raw is None:
            return None

        try:
            import cv2
        except ImportError:
            return None

        try:
            frame = self._draw_detections(raw, cv2)
            _rot_map = {
                90: cv2.ROTATE_90_CLOCKWISE,
                180: cv2.ROTATE_180,
                270: cv2.ROTATE_90_COUNTERCLOCKWISE,
            }
            if self._cam_rotate in _rot_map:
                frame = cv2.rotate(frame, _rot_map[self._cam_rotate])
            ok, buf = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
            )
            if not ok:
                return None
            data = buf.tobytes()
            if self._gateway is not None:
                self._gateway.push_jpeg(data)
            return data
        except Exception:
            logger.debug("TeleopModule: snapshot JPEG encode failed", exc_info=True)
            return None

    def _encode_loop(self) -> None:
        """Dedicated thread: encode raw frames to JPEG at stream_fps."""
        try:
            import cv2
            have_cv2 = True
        except ImportError:
            have_cv2 = False
            logger.warning("TeleopModule: cv2 not available 鈥?camera stream disabled")

        while self._running:
            triggered = self._new_frame.wait(timeout=self._stream_interval)
            self._new_frame.clear()
            if not triggered or not self._running:
                continue

            gw = self._gateway
            if gw is None:
                continue

            with self._raw_lock:
                raw = self._latest_raw

            if raw is None:
                continue

            if have_cv2:
                try:
                    frame = self._draw_detections(raw.copy(), cv2)
                    _rot_map = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
                    if self._cam_rotate in _rot_map:
                        frame = cv2.rotate(frame, _rot_map[self._cam_rotate])
                    ok, buf = cv2.imencode(
                        ".jpg", frame,
                        [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
                    )
                    if ok:
                        gw.push_jpeg(buf.tobytes())
                except Exception:
                    logger.debug("TeleopModule: encode iteration error", exc_info=True)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["active"] = self._active
        info["clients"] = self._clients
        info["camera_clients"] = self._camera_clients
        info["stream_clients"] = self._clients + self._camera_clients
        info["last_joy_age_ms"] = round(
            (time.monotonic() - self._last_joy_time) * 1000
        ) if self._last_joy_time > 0 else None
        return info

    # -- @skill methods (REPL / MCP) ----------------------------------------

    @skill
    def get_teleop_status(self) -> str:
        """Return current teleop status."""
        import json
        return json.dumps({
            "active": self._active,
            "clients": self._clients,
            "camera_clients": self._camera_clients,
            "stream_clients": self._clients + self._camera_clients,
            "port": self._port,
            "last_joy_age_ms": round(
                (time.monotonic() - self._last_joy_time) * 1000
            ) if self._last_joy_time > 0 else None,
        })

    @skill
    def force_release(self) -> str:
        """Force-release teleop control and resume autonomy."""
        self._release()
        return "Teleop released"
