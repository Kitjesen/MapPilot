"""ReconKeyframeExporterModule — robot-side keyframe collector + server uploader.

Runs on the robot alongside ReconstructionModule.  At configurable intervals
it selects keyframes (by minimum travel distance / rotation) and uploads them
to the remote reconstruction server as a multipart/form-data POST.

The server receives the frames and queues them for offline high-quality
reconstruction (TSDF, 3D Gaussian Splatting, NeRF, etc.).

Data sent per keyframe batch (POST /api/v1/keyframes):
    session_id  str  — UUID for the current run
    frame_idx   int  — monotonic frame counter
    timestamp   float — Unix epoch
    pose_json   str  — {"tx","ty","tz","qx","qy","qz","qw"} JSON
    intrinsics  str  — {"fx","fy","cx","cy","w","h"} JSON
    color_jpg   bytes — JPEG-encoded colour image
    depth_png   bytes — 16-bit PNG depth image (mm) OR depth_raw bytes

Ports:
    In: color_image (Image)
        depth_image (Image)
        camera_info (CameraIntrinsics)
        odometry (Odometry)
    Out: export_stats (dict)   — {frames_exported, bytes_sent, ...}
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
import uuid
from typing import Any

from runtime import In, Module, Out
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from runtime.registry import register

from .reconstruction_module import _load_cam_body_extrinsic, _pose_to_matrix

logger = logging.getLogger(__name__)

# Minimum keyframe selection thresholds (motion since last keyframe)
_DEFAULT_KF_DIST_M   = 0.3    # metres translation
_DEFAULT_KF_ROT_RAD  = 0.26   # radians (~15°)
_DEFAULT_KF_TIME_S   = 2.0    # seconds (max interval even if stationary)
_DEFAULT_BATCH_SIZE  = 5      # frames per HTTP POST


@register(
    "reconstruction",
    "keyframe_exporter",
    description="RGB-D keyframe reconstruction server exporter",
)
class ReconKeyframeExporterModule(Module, layer=3):
    """Select keyframes and stream them to the remote reconstruction server.

    Configuration keys:
        server_url        str   "http://localhost:7890"  — reconstruction server base URL
        session_id        str   auto-generated UUID      — tag all frames with this session
        keyframe_dist_m   float 0.3  — min translation between keyframes (m)
        keyframe_rot_rad  float 0.26 — min rotation between keyframes (rad)
        keyframe_time_s   float 2.0  — max time between keyframes (s)
        batch_size        int   5    — number of keyframes per HTTP batch upload
        jpeg_quality      int   85   — JPEG quality for colour image
        max_depth_m       float 6.0  — depth values beyond this are set to 0
        enabled           bool  True — set False to skip uploads
    """

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraIntrinsics]
    odometry: In[Odometry]
    export_stats: Out[dict]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)

        self._server_url   = str(config.get("server_url", "http://localhost:7890")).rstrip("/")
        self._session_id   = str(config.get("session_id", uuid.uuid4()))
        self._kf_dist      = float(config.get("keyframe_dist_m",  _DEFAULT_KF_DIST_M))
        self._kf_rot       = float(config.get("keyframe_rot_rad", _DEFAULT_KF_ROT_RAD))
        self._kf_time      = float(config.get("keyframe_time_s",  _DEFAULT_KF_TIME_S))
        self._batch_size   = int(config.get("batch_size", _DEFAULT_BATCH_SIZE))
        self._jpeg_quality = int(config.get("jpeg_quality", 85))
        self._max_depth_m  = float(config.get("max_depth_m", 6.0))
        self._enabled      = bool(config.get("enabled", True))

        self._camera_pose_in_body: np.ndarray = _load_cam_body_extrinsic()

        # Latest buffers (latest-wins)
        self._latest_color:  Image | None            = None
        self._latest_depth:  Image | None            = None
        self._latest_odom:   Odometry | None         = None
        self._intrinsics:    CameraIntrinsics | None = None
        self._buf_lock = threading.Lock()

        # Keyframe selection state
        self._last_kf_pos:   np.ndarray | None = None  # [x,y,z]
        self._last_kf_yaw:   float = 0.0
        self._last_kf_time:  float = 0.0
        self._frame_counter: int   = 0

        # Upload queue + stats
        self._pending_batch: list[dict] = []
        self._total_exported: int = 0
        self._total_bytes:    int = 0
        self._upload_errors:  int = 0

        self._recon_export_active = threading.Event()
        self._upload_thread: threading.Thread | None = None

    # ── Module lifecycle ────────────────────────────────────────────────────

    def setup(self) -> None:
        self.color_image.subscribe(self._on_color)
        self.depth_image.subscribe(self._on_depth)
        self.camera_info.subscribe(self._on_camera_info)
        self.odometry.subscribe(self._on_odom)

        self._recon_export_active.set()
        self._upload_thread = threading.Thread(
            target=self._upload_loop, daemon=True, name="recon_export"
        )
        self._upload_thread.start()

    def teardown(self) -> None:
        self._recon_export_active.clear()
        if self._upload_thread:
            self._upload_thread.join(timeout=10.0)

    # ── Port callbacks ──────────────────────────────────────────────────────

    def _on_color(self, img: Image) -> None:
        with self._buf_lock:
            self._latest_color = img

    def _on_depth(self, img: Image) -> None:
        with self._buf_lock:
            self._latest_depth = img

    def _on_camera_info(self, info: CameraIntrinsics) -> None:
        if self._intrinsics is None:
            self._intrinsics = info

    def _on_odom(self, odom: Odometry) -> None:
        with self._buf_lock:
            self._latest_odom = odom
        self._maybe_capture_keyframe()

    # ── Keyframe selection ──────────────────────────────────────────────────

    def _maybe_capture_keyframe(self) -> None:
        if not self._enabled:
            return

        with self._buf_lock:
            color = self._latest_color
            depth = self._latest_depth
            odom  = self._latest_odom

        if color is None or depth is None or odom is None:
            return
        if self._intrinsics is None:
            return

        now = time.time()
        pos = np.array([odom.pose.position.x,
                        odom.pose.position.y,
                        odom.pose.position.z])
        yaw = odom.pose.yaw

        # Is this a keyframe?
        if self._last_kf_pos is not None:
            dist = float(np.linalg.norm(pos - self._last_kf_pos))
            rot  = abs(_angle_diff(yaw, self._last_kf_yaw))
            dt   = now - self._last_kf_time
            if dist < self._kf_dist and rot < self._kf_rot and dt < self._kf_time:
                return  # not a keyframe yet

        # Capture
        self._last_kf_pos  = pos.copy()
        self._last_kf_yaw  = yaw
        self._last_kf_time = now
        self._frame_counter += 1

        kf = self._build_keyframe_payload(color, depth, odom, now)
        if kf is not None:
            with self._buf_lock:
                self._pending_batch.append(kf)

    def _build_keyframe_payload(
        self,
        color: Image,
        depth: Image,
        odom: Odometry,
        ts: float,
    ) -> dict | None:
        """Encode one keyframe into a serialisable dict."""
        try:
            # Encode colour as JPEG
            color_jpg = _encode_jpeg(color, quality=self._jpeg_quality)
            if color_jpg is None:
                return None

            # Encode depth as 16-bit PNG (mm)
            depth_png = _encode_depth_png(depth, max_depth_m=self._max_depth_m)
            if depth_png is None:
                return None

            # Build pose JSON (camera-to-world)
            body_to_world   = _pose_to_matrix(odom)
            cam_to_world    = body_to_world @ self._camera_pose_in_body
            pose_json = json.dumps({
                "tx": float(cam_to_world[0, 3]),
                "ty": float(cam_to_world[1, 3]),
                "tz": float(cam_to_world[2, 3]),
                "r00": float(cam_to_world[0, 0]),
                "r01": float(cam_to_world[0, 1]),
                "r02": float(cam_to_world[0, 2]),
                "r10": float(cam_to_world[1, 0]),
                "r11": float(cam_to_world[1, 1]),
                "r12": float(cam_to_world[1, 2]),
                "r20": float(cam_to_world[2, 0]),
                "r21": float(cam_to_world[2, 1]),
                "r22": float(cam_to_world[2, 2]),
            })

            intr = self._intrinsics
            intrinsics_json = json.dumps({
                "fx": float(intr.fx),
                "fy": float(intr.fy),
                "cx": float(intr.cx),
                "cy": float(intr.cy),
                "w":  int(intr.width),
                "h":  int(intr.height),
            })

            return {
                "session_id":  self._session_id,
                "frame_idx":   self._frame_counter,
                "timestamp":   ts,
                "pose_json":   pose_json,
                "intrinsics":  intrinsics_json,
                "color_jpg":   color_jpg,
                "depth_png":   depth_png,
            }
        except Exception:
            logger.exception("Failed to build keyframe payload")
            return None

    # ── Upload loop ─────────────────────────────────────────────────────────

    def _upload_loop(self) -> None:
        while self._recon_export_active.is_set():
            time.sleep(0.5)
            with self._buf_lock:
                if len(self._pending_batch) < self._batch_size:
                    continue
                batch = self._pending_batch[:self._batch_size]
                self._pending_batch = self._pending_batch[self._batch_size:]

            self._upload_batch(batch)

    def _upload_batch(self, batch: list[dict]) -> None:
        try:
            import urllib.request
            import urllib.error

            # Build multipart payload manually (no external deps)
            boundary = b"---LingTuReconBoundary"
            parts = []

            for kf in batch:
                for key in ("session_id", "frame_idx", "timestamp",
                            "pose_json", "intrinsics"):
                    val = str(kf[key]).encode()
                    parts.append(
                        b"--" + boundary + b"\r\n"
                        b'Content-Disposition: form-data; name="' + key.encode() + b'"\r\n'
                        b"\r\n" + val + b"\r\n"
                    )
                # Binary fields
                for field, ctype, fname in [
                    ("color_jpg", b"image/jpeg", b"color.jpg"),
                    ("depth_png", b"image/png",  b"depth.png"),
                ]:
                    data = kf[field]
                    parts.append(
                        b"--" + boundary + b"\r\n"
                        b'Content-Disposition: form-data; name="' + field.encode()
                        + b'"; filename="' + fname + b'"\r\n'
                        b"Content-Type: " + ctype + b"\r\n"
                        b"\r\n" + data + b"\r\n"
                    )

            body = b"".join(parts) + b"--" + boundary + b"--\r\n"
            url  = self._server_url + "/api/v1/keyframes"
            req  = urllib.request.Request(
                url,
                data=body,
                headers={"Content-Type": "multipart/form-data; boundary=" + boundary.decode()},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=15) as resp:  # noqa: S310  # trusted local server
                resp.read()

            self._total_exported += len(batch)
            self._total_bytes    += len(body)
            self.export_stats.publish({
                "frames_exported": self._total_exported,
                "bytes_sent":      self._total_bytes,
                "upload_errors":   self._upload_errors,
                "session_id":      self._session_id,
            })
            logger.debug("Uploaded %d keyframes (%d bytes) to %s",
                         len(batch), len(body), url)

        except Exception as exc:
            self._upload_errors += 1
            logger.warning("Keyframe upload failed: %s", exc)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info.update({
            "session_id":      self._session_id,
            "frames_captured": self._frame_counter,
            "frames_exported": self._total_exported,
            "bytes_sent":      self._total_bytes,
            "upload_errors":   self._upload_errors,
            "pending_batch":   len(self._pending_batch),
            "server_url":      self._server_url,
            "enabled":         self._enabled,
        })
        return info


# ── Helpers ─────────────────────────────────────────────────────────────────

def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference a - b in [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _encode_jpeg(img: Image, quality: int = 85) -> bytes | None:
    """Encode an Image to JPEG bytes. Returns None on failure."""
    try:
        import cv2  # type: ignore
        data = img.data
        if img.format == ImageFormat.RGB:
            data = data[..., ::-1]
        elif img.format == ImageFormat.GRAY:
            data = np.stack([data] * 3, axis=-1)
        ok, buf = cv2.imencode(".jpg", data, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return None
        return buf.tobytes()
    except ImportError:
        # Fallback: raw JPEG via struct (poor quality, no dependency)
        # This path is unlikely in practice but avoids hard cv2 dependency.
        return img.data.tobytes()
    except Exception:
        return None


def _encode_depth_png(depth_img: Image, max_depth_m: float = 6.0) -> bytes | None:
    """Encode depth image to 16-bit PNG bytes (values in mm). Returns None on failure."""
    try:
        import cv2  # type: ignore
        data = depth_img.data
        if depth_img.format == ImageFormat.DEPTH_F32:
            # float32 metres → uint16 mm, clamp to max_depth
            depth_mm = (data * 1000.0).astype(np.float32)
            depth_mm[depth_mm > max_depth_m * 1000] = 0
            depth_mm = depth_mm.astype(np.uint16)
        else:
            # Assume uint16 mm already
            depth_mm = data.astype(np.uint16)
            depth_mm[depth_mm > int(max_depth_m * 1000)] = 0

        ok, buf = cv2.imencode(".png", depth_mm)
        if not ok:
            return None
        return buf.tobytes()
    except ImportError:
        return depth_img.data.tobytes()
    except Exception:
        return None
