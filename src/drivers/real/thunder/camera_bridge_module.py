"""CameraBridgeModule — bridges ROS2 camera topics into Python Module pipeline.

Subscribes to ROS2:
  /camera/color/image_raw  — sensor_msgs/Image (BGR8)
  /camera/depth/image_raw  — sensor_msgs/Image (16UC1 mm or 32FC1 m)
  /camera/camera_info      — sensor_msgs/CameraInfo

Publishes to Module ports:
  color_image: Out[Image]           — already undistorted if distortion present
  depth_image: Out[Image]           — already undistorted (nearest-neighbor remap)
  camera_info: Out[CameraIntrinsics]— with zero distortion (post-undistort)

Undistortion is applied at source so all downstream consumers
(Perception, VisualServo, Reconstruction, DepthVisualOdom) operate
under the correct pinhole camera model without each needing to undistort.

Graceful no-op when rclpy unavailable (Windows dev, stub mode).

Usage::

    bp.add(CameraBridgeModule)
    bp.add(CameraBridgeModule, color_topic="/cam/rgb", depth_topic="/cam/depth")
"""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Any, Dict

from core.module import Module
from core.msgs.numpy_compat import np
from core.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from core.registry import register
from core.stream import Out

logger = logging.getLogger(__name__)

DEFAULT_CAMERA_INFO_TOPICS = (
    "/camera/color/camera_info",
    "/camera/depth/camera_info",
    "/camera/camera_info",
)


@register("camera_bridge", "default", description="ROS2 camera → Python Module bridge")
class CameraBridgeModule(Module, layer=1):
    """Bridges ROS2 camera topics (RGB + depth + intrinsics) into Module ports.

    On systems without rclpy, starts silently — no crash, no output.
    """

    # -- Outputs --
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraIntrinsics]
    alive:       Out[bool]

    def __init__(
        self,
        node_name: str = "camera_bridge",
        color_topic: str = "/camera/color/image_raw",
        depth_topic: str = "/camera/depth/image_raw",
        info_topic: str = "/camera/camera_info",
        info_topics: tuple[str, ...] | list[str] | str | None = None,
        preferred_info_topic: str | None = None,
        spin_rate: float = 30.0,
        qos_depth: int = 5,
        rotate: int = 0,
        **kw,
    ):
        super().__init__(**kw)
        self._node_name = node_name
        self._color_topic = color_topic
        self._depth_topic = depth_topic
        self._info_topic = info_topic
        self._info_topics = self._normalize_info_topics(info_topic, info_topics)
        env_preferred = os.environ.get("LINGTU_CAMERA_INFO_PREFERRED_TOPIC", "").strip()
        preferred = preferred_info_topic or env_preferred
        self._preferred_info_topic = (
            preferred
            if preferred in self._info_topics
            else (
                "/camera/color/camera_info"
                if "/camera/color/camera_info" in self._info_topics
                else self._info_topics[0]
            )
        )
        self._active_info_topic: str | None = None
        self._info_topic_cache: Dict[str, CameraIntrinsics] = {}
        self._spin_rate = spin_rate
        self._qos_depth = qos_depth
        # Rotation: 0=none, 90=CW, 180=flip, 270=CCW
        self._rotate = rotate

        self._node = None
        self._executor = None
        self._running = False
        self._rclpy_available = False
        self._dds_reader = None
        self._backend = "stub"  # "dds" | "rclpy" | "stub"

        # Undistortion maps — computed once on first camera_info with nonzero distortion
        self._undistort_maps: tuple[np.ndarray, np.ndarray] | None = None
        self._undistorted_intrinsics: CameraIntrinsics | None = None
        self._K: np.ndarray | None = None  # 3x3 camera matrix (undistorted)

        # Auto-recovery: monitor data freshness, reconnect if stale
        self._last_color_ts: float = 0.0
        self._last_depth_ts: float = 0.0
        self._stale_timeout: float = kw.get("stale_timeout", 5.0)
        self._max_reconnects: int = kw.get("max_reconnects", 10)
        raw_allow_service_recovery = kw.get(
            "allow_service_recovery",
            os.environ.get("LINGTU_CAMERA_ALLOW_SERVICE_RECOVERY", "0"),
        )
        self._allow_service_recovery = str(raw_allow_service_recovery).lower() in (
            "1",
            "true",
            "yes",
            "on",
        )
        self._service_recovery_suppressed = False
        self._reconnect_count: int = 0
        self._watchdog_thread: threading.Thread | None = None
        self._shutdown_event = threading.Event()

    @staticmethod
    def _split_topic_list(raw: str) -> tuple[str, ...]:
        return tuple(topic.strip() for topic in raw.replace(",", " ").split() if topic.strip())

    @classmethod
    def _normalize_info_topics(
        cls,
        info_topic: str,
        info_topics: tuple[str, ...] | list[str] | str | None,
    ) -> tuple[str, ...]:
        env_topics = os.environ.get("LINGTU_CAMERA_INFO_TOPICS", "").strip()
        if info_topics is None and env_topics:
            raw_topics: tuple[str, ...] | list[str] = cls._split_topic_list(env_topics)
        elif info_topics is None:
            raw_topics = (
                DEFAULT_CAMERA_INFO_TOPICS
                if info_topic == "/camera/camera_info"
                else (info_topic,)
            )
        elif isinstance(info_topics, str):
            raw_topics = cls._split_topic_list(info_topics)
        else:
            raw_topics = info_topics

        topics: list[str] = []
        for topic in raw_topics:
            clean = str(topic).strip()
            if clean and clean not in topics:
                topics.append(clean)
        return tuple(topics or (info_topic,))

    def setup(self) -> None:
        # Prefer DDS — works without `source /opt/ros/humble/setup.bash`
        # and survives the rclpy + uvicorn + aiortc thread conflict that
        # kills lingtu on-boot when ROS2 env is sourced.  slam_bridge +
        # gnss_module follow the same DDS-first policy.
        if self._create_dds_reader():
            self._backend = "dds"
            return
        if self._create_ros2_node():
            self._backend = "rclpy"
            return
        self._backend = "stub"
        logger.info("CameraBridgeModule: stub mode — no DDS or rclpy available")

    def _create_dds_reader(self) -> bool:
        """Subscribe via raw cyclonedds — no rclpy, no ROS2 env needed."""
        try:
            from core.dds import ROS2TopicReader
        except ImportError:
            return False
        reader = ROS2TopicReader()
        reader.on_image(self._color_topic, self._on_ros2_color)
        reader.on_image(self._depth_topic, self._on_ros2_depth)
        for topic in self._info_topics:
            reader.on_camera_info(
                topic,
                lambda msg, topic=topic: self._on_ros2_info(msg, topic),
            )
        if not reader.start():
            return False
        reader.spin_background()
        self._dds_reader = reader
        logger.info(
            "CameraBridgeModule (DDS): color=%s depth=%s info_topics=%s preferred=%s",
            self._color_topic,
            self._depth_topic,
            ",".join(self._info_topics),
            self._preferred_info_topic,
        )
        return True

    def _create_ros2_node(self) -> bool:
        """Create ROS2 node and subscriptions. Returns True on success."""
        node = None
        executor = None
        try:
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import CameraInfo
            from sensor_msgs.msg import Image as ROS2Image

            from core.ros2_context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            self._rclpy_available = True

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=self._qos_depth,
            )

            # Reconnection path: remove the old node from the shared executor
            # before destroying it, otherwise stale nodes remain in the spin set.
            self._cleanup_ros2_node()

            node = Node(self._node_name)
            executor = get_shared_executor()
            executor.add_node(node)
            node.create_subscription(
                ROS2Image, self._color_topic, self._on_ros2_color, qos)
            node.create_subscription(
                ROS2Image, self._depth_topic, self._on_ros2_depth, qos)
            for topic in self._info_topics:
                node.create_subscription(
                    CameraInfo,
                    topic,
                    lambda msg, topic=topic: self._on_ros2_info(msg, topic),
                    qos,
                )
            self._node = node
            self._executor = executor

            logger.info(
                "CameraBridgeModule: node '%s' — color=%s depth=%s info=%s",
                self._node_name, self._color_topic, self._depth_topic, ",".join(self._info_topics),
            )
            return True
        except ImportError:
            logger.info("CameraBridgeModule: rclpy not available, running in stub mode")
            return False
        except Exception as e:
            self._cleanup_ros2_node(node=node, executor=executor)
            logger.warning("CameraBridgeModule: setup failed: %s", e)
            return False

    def _cleanup_ros2_node(self, node=None, executor=None) -> None:
        node = node if node is not None else self._node
        executor = executor if executor is not None else self._executor
        if node is None:
            return
        try:
            if executor is not None:
                executor.remove_node(node)
        except Exception as e:
            logger.warning("CameraBridge remove_node: %s", e)
        try:
            node.destroy_node()
        except Exception as e:
            logger.warning("CameraBridge destroy_node: %s", e)
        if node is self._node:
            self._node = None
        if executor is self._executor:
            self._executor = None

    def start(self) -> None:
        super().start()
        if self._backend == "stub":
            self.alive.publish(False)
            return
        self._running = True
        self.alive.publish(True)
        # Auto-recovery watchdog only applies to rclpy path (L1 reconnect
        # requires rclpy subscription reuse).  DDS path is auto-discovered
        # by cyclonedds when the camera node comes back up.
        if self._backend == "rclpy":
            self._shutdown_event.clear()
            self._watchdog_thread = threading.Thread(
                target=self._watchdog_loop, daemon=True,
                name="camera-bridge-watchdog")
            self._watchdog_thread.start()

    def stop(self) -> None:
        self._running = False
        self._shutdown_event.set()
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=2.0)
        self._watchdog_thread = None
        if self._dds_reader:
            try:
                self._dds_reader.stop()
            except Exception as e:
                logger.warning("CameraBridge DDS reader stop: %s", e)
            self._dds_reader = None
        self._cleanup_ros2_node()
        super().stop()

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["frame_count"] = getattr(self, "_frame_count", 0)
        now = time.time()
        if self._last_color_ts > 0:
            dt = now - self._last_color_ts
            info["fps"] = round(1.0 / dt, 1) if dt > 0 else 0.0
        else:
            info["fps"] = 0.0
        info["reconnect_count"] = self._reconnect_count
        info["service_recovery_allowed"] = self._allow_service_recovery
        info["service_recovery_suppressed"] = self._service_recovery_suppressed
        info["camera_info_topics"] = list(self._info_topics)
        info["camera_info_preferred_topic"] = self._preferred_info_topic
        info["camera_info_active_topic"] = self._active_info_topic
        return info

    # ── Auto-recovery watchdog ────────────────────────────────────────────────
    #
    # Three escalation levels:
    #   L1  Reconnect rclpy subscription  (QoS / topic mismatch)
    #   L2  systemctl restart camera      (camera node crash)
    #   L3  USB reset + restart camera    (hardware disconnect)

    def _watchdog_loop(self) -> None:
        """Monitor data freshness. Escalate recovery on repeated failures."""
        startup_grace = 30.0  # seconds after start before checking "never received"
        start_time = time.time()

        while not self._shutdown_event.is_set():
            self._shutdown_event.wait(timeout=3.0)
            if not self._running or not self._rclpy_available:
                continue

            now = time.time()

            # Determine if camera is stale
            if self._last_color_ts > 0:
                color_age = now - self._last_color_ts
                is_stale = color_age > self._stale_timeout
            else:
                # Never received a frame — wait for startup grace period
                is_stale = (now - start_time) > startup_grace

            if not is_stale:
                if self._reconnect_count > 0:
                    logger.info("CameraBridge: data resumed after %d recovery attempt(s)",
                                self._reconnect_count)
                    self._reconnect_count = 0
                continue

            if self._reconnect_count >= self._max_reconnects:
                if self._reconnect_count == self._max_reconnects:
                    logger.error(
                        "CameraBridge: max recoveries (%d) reached, giving up",
                        self._max_reconnects)
                    self.alive.publish(False)
                    self._reconnect_count += 1
                continue

            self._reconnect_count += 1
            level = self._recovery_level(self._reconnect_count)
            backoff = min(5.0 * self._reconnect_count, 30.0)

            if self._suppress_service_recovery(level):
                continue

            if level == 1:
                logger.warning(
                    "CameraBridge: L1 recovery — reconnect rclpy (%d/%d)",
                    self._reconnect_count, self._max_reconnects)
                if self._create_ros2_node():
                    self.alive.publish(True)
                else:
                    self.alive.publish(False)
            elif level == 2:
                logger.warning(
                    "CameraBridge: L2 recovery — restart robot-camera.service (%d/%d)",
                    self._reconnect_count, self._max_reconnects)
                self._restart_camera_service()
                self._shutdown_event.wait(timeout=8.0)
                if self._shutdown_event.is_set():
                    break
                self._create_ros2_node()
            else:
                logger.warning(
                    "CameraBridge: L3 recovery — USB reset + restart (%d/%d)",
                    self._reconnect_count, self._max_reconnects)
                self._usb_reset_camera()
                self._shutdown_event.wait(timeout=5.0)
                self._restart_camera_service()
                self._shutdown_event.wait(timeout=8.0)
                if self._shutdown_event.is_set():
                    break
                self._create_ros2_node()

            self._shutdown_event.wait(timeout=backoff)

    def _suppress_service_recovery(self, level: int) -> bool:
        if level <= 1 or self._allow_service_recovery:
            return False
        if not self._service_recovery_suppressed:
            logger.warning(
                "CameraBridge: service/USB recovery suppressed; set "
                "LINGTU_CAMERA_ALLOW_SERVICE_RECOVERY=1 to enable "
                "robot-camera.service restarts")
            self._service_recovery_suppressed = True
        self.alive.publish(False)
        self._reconnect_count = self._max_reconnects + 1
        return True

    @staticmethod
    def _recovery_level(attempt: int) -> int:
        """L1 for first 2 attempts, L2 for next 3, L3 after that."""
        if attempt <= 2:
            return 1
        if attempt <= 5:
            return 2
        return 3

    @staticmethod
    def _restart_camera_service() -> None:
        """Restart the canonical camera systemd service.

        The robot used to have a legacy ``camera.service`` unit. Starting it
        alongside ``robot-camera.service`` can duplicate Orbbec publishers or
        node instances. A single ``/camera/camera`` component plus a single
        ``/camera/camera_container`` is normal, so recovery stops only legacy
        units before bringing the canonical unit back.
        """
        import subprocess

        canonical_unit = os.environ.get("LINGTU_CAMERA_SERVICE", "robot-camera.service")
        legacy_units = tuple(
            unit.strip()
            for unit in os.environ.get(
                "LINGTU_CAMERA_LEGACY_SERVICES",
                "camera.service orbbec-camera.service",
            ).replace(",", " ").split()
            if unit.strip()
        )

        def run_systemctl(action: str, unit: str, timeout: float = 10.0) -> None:
            subprocess.run(
                ["sudo", "systemctl", action, unit],
                capture_output=True,
                timeout=timeout,
            )

        try:
            for unit in legacy_units:
                if unit != canonical_unit:
                    run_systemctl("stop", unit)
            run_systemctl("stop", canonical_unit)

            # Clean only stale Orbbec camera launch/container processes. Avoid a
            # broad component_container kill because other ROS components may
            # share that binary name.
            for pattern in (
                r"component_container.*__ns:=/camera",
                r"ros2 launch orbbec_camera",
            ):
                subprocess.run(
                    ["sudo", "pkill", "-TERM", "-f", pattern],
                    capture_output=True,
                    timeout=5,
                )
            time.sleep(1)
            run_systemctl("start", canonical_unit)
            logger.info("CameraBridge: %s restarted (legacy units stopped)", canonical_unit)
        except Exception as e:
            logger.warning("CameraBridge: failed to restart %s: %s", canonical_unit, e)

    @staticmethod
    def _usb_reset_camera() -> None:
        """Reset Orbbec USB device without physical unplug."""
        import subprocess
        try:
            # Find Orbbec device in sysfs (vendor 2bc5)
            result = subprocess.run(
                ["bash", "-c",
                 "grep -rl '2bc5' /sys/bus/usb/devices/*/idVendor 2>/dev/null "
                 "| head -1 | xargs dirname"],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=5,
            )
            dev_path = result.stdout.strip()
            if dev_path:
                auth_path = f"{dev_path}/authorized"
                # Power cycle: deauthorize → wait → reauthorize
                subprocess.run(
                    ["bash", "-c", f"echo 0 | sudo tee {auth_path} && "
                     f"sleep 2 && echo 1 | sudo tee {auth_path}"],
                    capture_output=True, timeout=10,
                )
                logger.info("CameraBridge: USB reset via %s", auth_path)
            else:
                logger.warning("CameraBridge: Orbbec USB device not found in sysfs")
        except Exception as e:
            logger.warning("CameraBridge: USB reset failed: %s", e)

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _on_ros2_color(self, msg) -> None:
        now = time.time()
        self._last_color_ts = now
        try:
            encoding = getattr(msg, "encoding", "bgr8").lower()
            h, w = msg.height, msg.width
            raw = bytes(msg.data)

            if encoding in ("bgr8", "rgb8"):
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
                # Normalize to BGR — OpenCV's native format.
                # TeleopModule (cv2.imencode) and detection overlay
                # (cv2.rectangle/putText) all assume BGR input.
                if encoding == "rgb8":
                    import cv2
                    arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                fmt = ImageFormat.BGR
            elif encoding in ("mono8", "8uc1"):
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
                fmt = ImageFormat.GRAY
            else:
                arr = np.frombuffer(raw, dtype=np.uint8)
                fmt = ImageFormat.GRAY

            # Apply undistortion at source (linear interpolation for color)
            if self._undistortion_maps_match(arr):
                import cv2
                arr = cv2.remap(arr, *self._undistort_maps, cv2.INTER_LINEAR)

            # Rotate if camera is mounted sideways
            if self._rotate != 0:
                import cv2
                _rot_map = {90: cv2.ROTATE_90_CLOCKWISE,
                            180: cv2.ROTATE_180,
                            270: cv2.ROTATE_90_COUNTERCLOCKWISE}
                rot_code = _rot_map.get(self._rotate)
                if rot_code is not None:
                    arr = cv2.rotate(arr, rot_code)

            self.color_image.publish(Image(
                data=arr.copy(), format=fmt,
                ts=now, frame_id=msg.header.frame_id or "camera",
            ))
        except Exception as e:
            logger.debug("CameraBridge: color parse error: %s", e)

    def _on_ros2_depth(self, msg) -> None:
        now = time.time()
        self._last_depth_ts = now
        try:
            encoding = getattr(msg, "encoding", "16uc1").lower()
            h, w = msg.height, msg.width
            raw = bytes(msg.data)

            if encoding in ("16uc1",):
                arr = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
                fmt = ImageFormat.DEPTH_U16
            elif encoding in ("32fc1",):
                arr = np.frombuffer(raw, dtype=np.float32).reshape(h, w)
                fmt = ImageFormat.DEPTH_F32
            else:
                arr = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
                fmt = ImageFormat.DEPTH_U16

            # Apply undistortion at source (nearest-neighbor for depth to avoid blending)
            if self._undistortion_maps_match(arr):
                import cv2
                arr = cv2.remap(arr, *self._undistort_maps, cv2.INTER_NEAREST)

            self.depth_image.publish(Image(
                data=arr.copy(), format=fmt,
                ts=now, frame_id=msg.header.frame_id or "camera",
            ))
        except Exception as e:
            logger.debug("CameraBridge: depth parse error: %s", e)

    def _topic_priority(self, topic: str) -> int:
        try:
            return self._info_topics.index(topic)
        except ValueError:
            return len(self._info_topics)

    def _should_accept_info_topic(self, topic: str) -> bool:
        active = self._active_info_topic
        if active is None or active == topic:
            return True
        if topic == self._preferred_info_topic:
            return True
        if active == self._preferred_info_topic:
            return False
        return self._topic_priority(topic) < self._topic_priority(active)

    def _undistortion_maps_match(self, arr: np.ndarray) -> bool:
        if self._undistort_maps is None:
            return False
        map1 = self._undistort_maps[0]
        return tuple(map1.shape[:2]) == tuple(arr.shape[:2])

    def _rotate_intrinsics(self, intrinsics: CameraIntrinsics) -> CameraIntrinsics:
        """Transform camera intrinsics to match image rotation.

        When the image is rotated (e.g. camera mounted sideways), pixel
        coordinates are remapped. The intrinsics matrix K must be updated
        so downstream 3D projection (fx/fy/cx/cy/width/height) remains
        consistent with the rotated image.

        Rotation formulas (pixel coordinates):
          - 90° CW  (rotate=90):   cx' = H-1-cy,  cy' = cx,   fx'=fy, fy'=fx, w'=h, h'=w
          - 180°    (rotate=180):  cx' = W-1-cx,  cy' = H-1-cy,        kept,  kept
          - 270° CW (rotate=270, i.e. 90° CCW): cx' = cy, cy' = W-cx, fx'=fy, fy'=fx, w'=h, h'=w
        """
        rot = self._rotate
        if rot == 0:
            return intrinsics

        w, h = intrinsics.width, intrinsics.height
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.cx, intrinsics.cy

        if rot == 90:
            return CameraIntrinsics(
                fx=fy, fy=fx,
                cx=float(h - 1 - cy), cy=float(cx),
                width=h, height=w,
                dist_k1=intrinsics.dist_k1, dist_k2=intrinsics.dist_k2,
                dist_p1=intrinsics.dist_p1, dist_p2=intrinsics.dist_p2,
                dist_k3=intrinsics.dist_k3,
                depth_scale=intrinsics.depth_scale,
            )
        if rot == 180:
            return CameraIntrinsics(
                fx=fx, fy=fy,
                cx=float(w - 1 - cx), cy=float(h - 1 - cy),
                width=w, height=h,
                dist_k1=intrinsics.dist_k1, dist_k2=intrinsics.dist_k2,
                dist_p1=intrinsics.dist_p1, dist_p2=intrinsics.dist_p2,
                dist_k3=intrinsics.dist_k3,
                depth_scale=intrinsics.depth_scale,
            )
        if rot == 270:
            return CameraIntrinsics(
                fx=fy, fy=fx,
                cx=float(cy), cy=float(w - cx),
                width=h, height=w,
                dist_k1=intrinsics.dist_k1, dist_k2=intrinsics.dist_k2,
                dist_p1=intrinsics.dist_p1, dist_p2=intrinsics.dist_p2,
                dist_k3=intrinsics.dist_k3,
                depth_scale=intrinsics.depth_scale,
            )
        logger.warning("CameraBridge: unsupported rotate=%d, skipping intrinsics swap", rot)
        return intrinsics

    def _on_ros2_info(self, msg, topic: str | None = None) -> None:
        try:
            topic = topic or self._info_topic
            k = msg.k  # 3x3 intrinsic matrix, row-major
            fx, fy = float(k[0]), float(k[4])
            cx, cy = float(k[2]), float(k[5])
            w, h = int(msg.width), int(msg.height)

            # Parse distortion coefficients (plumb_bob: k1, k2, p1, p2, k3)
            d = getattr(msg, "d", None) or []
            dk1 = float(d[0]) if len(d) > 0 else 0.0
            dk2 = float(d[1]) if len(d) > 1 else 0.0
            dp1 = float(d[2]) if len(d) > 2 else 0.0
            dp2 = float(d[3]) if len(d) > 3 else 0.0
            dk3 = float(d[4]) if len(d) > 4 else 0.0

            intrinsics = CameraIntrinsics(
                fx=fx, fy=fy, cx=cx, cy=cy,
                width=w, height=h,
                dist_k1=dk1, dist_k2=dk2,
                dist_p1=dp1, dist_p2=dp2, dist_k3=dk3,
            )
            self._info_topic_cache[topic] = intrinsics
            if not self._should_accept_info_topic(topic):
                return

            if self._active_info_topic != topic:
                self._active_info_topic = topic
                self._undistort_maps = None
                self._undistorted_intrinsics = None
                self._K = None
                logger.info("CameraBridge: using camera_info topic %s", topic)

            # Build undistortion maps using ORIGINAL intrinsics — undistortion
            # is applied BEFORE rotation in the image pipeline (see _on_ros2_color).
            if self._undistort_maps is None and any(
                abs(v) > 1e-12 for v in (dk1, dk2, dp1, dp2, dk3)
            ):
                self._init_undistort_maps(fx, fy, cx, cy, w, h,
                                          dk1, dk2, dp1, dp2, dk3)

            # Apply rotation to published intrinsics so they match the
            # image after cv2.rotate() in _on_ros2_color.
            if self._rotate:
                intrinsics = self._rotate_intrinsics(intrinsics)
                if self._undistorted_intrinsics is not None:
                    self._undistorted_intrinsics = self._rotate_intrinsics(
                        self._undistorted_intrinsics
                    )
                logger.info(
                    "CameraBridge: rotate=%d applied to intrinsics "
                    "(fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f, %dx%d)",
                    self._rotate,
                    intrinsics.fx, intrinsics.fy,
                    intrinsics.cx, intrinsics.cy,
                    intrinsics.width, intrinsics.height,
                )

            # Publish intrinsics — with zero distortion since images are pre-undistorted
            if self._undistorted_intrinsics is not None:
                self.camera_info.publish(self._undistorted_intrinsics)
            else:
                self.camera_info.publish(intrinsics)
        except Exception as e:
            logger.debug("CameraBridge: info parse error: %s", e)

    def _init_undistort_maps(
        self, fx, fy, cx, cy, w, h, k1, k2, p1, p2, k3,
    ) -> None:
        """Compute undistortion remap tables. Called once."""
        try:
            import cv2
            K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
            D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
            self._undistort_maps = cv2.initUndistortRectifyMap(
                K, D, None, K, (w, h), cv2.CV_16SC2,
            )
            self._K = K
            # After undistortion, distortion coefficients are zero
            self._undistorted_intrinsics = CameraIntrinsics(
                fx=fx, fy=fy, cx=cx, cy=cy,
                width=w, height=h,
                dist_k1=0.0, dist_k2=0.0,
                dist_p1=0.0, dist_p2=0.0, dist_k3=0.0,
            )
            logger.info(
                "CameraBridge: undistortion maps computed (k1=%.4f, k2=%.4f) — "
                "all published images are now rectified",
                k1, k2,
            )
        except ImportError:
            logger.warning("CameraBridge: cv2 not available, skipping undistortion")

    # ── Spin loop ─────────────────────────────────────────────────────────────

    # Spin handled by shared executor (core.ros2_context)
