"""VLA navigation module.

Vision-Language-Action navigation via external VLM API calls.
Works as a peer module in the L4 decision layer, parallel to
SemanticPlannerModule and VisualServoModule.
"""

from __future__ import annotations

import base64
import io
import logging
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any

from decision.modules.vla_backends import MockVLABackend, VLABackend, create_vla_backend
from runtime.module import Module, skill
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import CameraIntrinsics, Image
from runtime.registry import register
from runtime.runtime_interface import map_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

VLA_MAP_FRAME_ID = map_frame_id()

# Distance routing thresholds (metres)
_FAR_DISTANCE_THRESHOLD = 3.5
_NEAR_DISTANCE_THRESHOLD = 3.0

# Circuit breaker configuration defaults
_DEFAULT_CB_THRESHOLD = 3
_DEFAULT_CB_COOLDOWN_SEC = 60.0

# Default environment/config values
_DEFAULT_TIMEOUT_SEC = 5.0
_DEFAULT_CONFIDENCE_THRESHOLD = 0.5
_DEFAULT_IMAGE_MAX_SIZE = 512


@register("vla", "default", description="Vision-Language-Action navigation module")
class VLAModule(Module, layer=4):
    """VLA navigation module - multi-modal navigation decisions via VLM API.

    Far targets are published as PoseStamped goals for the navigation stack.
    Near targets are published as Twist commands for the velocity mux.
    On API failures the module falls back to the semantic planner channel.
    On low-confidence results the target is forwarded to VisualServoModule.
    """

    _run_in_worker = True
    _worker_group = "semantic"
    SOFT_DEPENDS = ["LLMModule", "PerceptionModule"]

    # -- Inputs --
    color_image: In[Image]
    depth_image: In[Image]
    odometry: In[Odometry]
    scene_graph: In[SceneGraph]
    instruction: In[str]
    camera_info: In[CameraIntrinsics]
    mission_status: In[dict]

    # -- Outputs --
    goal_pose: Out[PoseStamped]
    cmd_vel: Out[Twist]
    vla_status: Out[dict]
    servo_target: Out[str]
    vla_action: Out[dict]
    planner_status: Out[str]

    def __init__(
        self,
        backend: str = "mock",
        model: str = "",
        api_key: str = "",
        api_key_env: str = "",
        timeout_sec: float = _DEFAULT_TIMEOUT_SEC,
        confidence_threshold: float = _DEFAULT_CONFIDENCE_THRESHOLD,
        image_max_size: int = _DEFAULT_IMAGE_MAX_SIZE,
        far_distance_threshold: float = _FAR_DISTANCE_THRESHOLD,
        near_distance_threshold: float = _NEAR_DISTANCE_THRESHOLD,
        circuit_breaker_threshold: int = _DEFAULT_CB_THRESHOLD,
        circuit_breaker_cooldown: float = _DEFAULT_CB_COOLDOWN_SEC,
        system_prompt: str = "",
        **kw,
    ):
        super().__init__(**kw)
        self._backend_name = backend
        self._model = model
        self._api_key = api_key
        self._api_key_env = api_key_env
        self._timeout_sec = timeout_sec
        self._confidence_threshold = confidence_threshold
        self._image_max_size = image_max_size
        self._far_threshold = far_distance_threshold
        self._near_threshold = near_distance_threshold
        self._cb_threshold = circuit_breaker_threshold
        self._cb_cooldown = circuit_breaker_cooldown
        self._system_prompt = system_prompt or self._default_system_prompt()

        # Lazy-initialized backend and executor
        self._backend: VLABackend | None = None
        self._executor: ThreadPoolExecutor | None = None

        # Cached sensor state
        self._latest_color: Image | None = None
        self._latest_depth: Image | None = None
        self._latest_camera_info: CameraIntrinsics | None = None
        self._robot_pose = [0.0, 0.0, 0.0]
        self._robot_yaw = 0.0
        self._latest_scene_graph: SceneGraph | None = None
        self._latest_mission_status: dict | None = None

        # Call / circuit-breaker metrics
        self._call_count = 0
        self._success_count = 0
        self._error_count = 0
        self._total_latency_ms = 0.0
        self._consecutive_failures = 0
        self._circuit_open_until = 0.0
        self._metrics_lock = threading.Lock()
        self._in_flight = 0

        # Distance routing hysteresis
        self._near_mode_active = False

    @staticmethod
    def _default_system_prompt() -> str:
        return (
            "You are the vision-language-action planner for a quadruped robot. "
            "Given a single RGB image, the robot's current 2D map position, and a "
            "natural-language instruction, decide the next navigation action. "
            "Output exactly one JSON object with no markdown formatting and these fields:\n"
            "- action: one of navigate|approach|explore|stop\n"
            "- target_x: target x coordinate in map frame (metres)\n"
            "- target_y: target y coordinate in map frame (metres)\n"
            "- target_z: target z coordinate (metres, usually 0)\n"
            "- confidence: float in [0,1]\n"
            "- reason: short explanation\n"
            "Use 'navigate' for distant goals, 'approach' for nearby precise targets, "
            "'explore' when the target is not visible, and 'stop' to halt."
        )

    def preflight(self) -> str | None:
        """Check API key availability for non-mock backends."""
        if self._backend_name == "mock":
            return None
        env_map = {
            "openai": "OPENAI_API_KEY",
            "claude": "ANTHROPIC_API_KEY",
            "qwen": "DASHSCOPE_API_KEY",
        }
        env_var = self._api_key_env or env_map.get(self._backend_name, "")
        if env_var and not os.environ.get(env_var) and not self._api_key:
            return (
                f"API key env var '{env_var}' not set for VLA backend "
                f"'{self._backend_name}'. Set it or use backend='mock'."
            )
        return None

    def setup(self) -> None:
        """Initialize backend, subscribe ports, and start executor."""
        self._backend = self._create_backend()
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vla-infer")

        self.color_image.subscribe(self._on_color_image)
        self.color_image.set_policy("latest")
        self.depth_image.subscribe(self._on_depth_image)
        self.depth_image.set_policy("latest")
        self.odometry.subscribe(self._on_odometry)
        self.scene_graph.subscribe(self._on_scene_graph)
        self.instruction.subscribe(self._on_instruction)
        self.camera_info.subscribe(self._on_camera_info)
        self.mission_status.subscribe(self._on_mission_status)

        logger.info(
            "VLAModule: backend='%s' model='%s' timeout=%.1fs confidence=%.2f",
            self._backend_name,
            self._model or "default",
            self._timeout_sec,
            self._confidence_threshold,
        )

    def _create_backend(self) -> VLABackend:
        """Factory: instantiate the selected VLA backend."""
        return create_vla_backend(
            self._backend_name,
            api_key=self._api_key,
            model=self._model,
            timeout_sec=self._timeout_sec,
        )

    def stop(self) -> None:
        """Shutdown executor and backend HTTP clients."""
        if self._executor is not None:
            self._executor.shutdown(wait=False)
            self._executor = None
        backend = self._backend
        self._backend = None
        if backend is not None and hasattr(backend, "_client") and backend._client is not None:
            try:
                # httpx.AsyncClient close is synchronous
                backend._client.close()
            except Exception:
                logger.debug("VLA: error closing backend HTTP client", exc_info=True)
        super().stop()

    # ------------------------------------------------------------------
    # Input callbacks
    # ------------------------------------------------------------------

    def _on_color_image(self, img: Image) -> None:
        self._latest_color = img

    def _on_depth_image(self, img: Image) -> None:
        self._latest_depth = img

    def _on_odometry(self, odom: Odometry) -> None:
        self._robot_pose = [odom.x, odom.y, odom.z]
        self._robot_yaw = odom.yaw

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        self._latest_scene_graph = sg

    def _on_camera_info(self, info: CameraIntrinsics) -> None:
        self._latest_camera_info = info

    def _on_mission_status(self, status: dict) -> None:
        self._latest_mission_status = status

    def _on_instruction(self, instruction: str) -> None:
        """Handle a new natural-language instruction."""
        instruction = str(instruction).strip()
        if not instruction:
            return

        # Circuit breaker check
        now = time.time()
        if self._consecutive_failures >= self._cb_threshold:
            if now < self._circuit_open_until:
                logger.warning(
                    "VLA: circuit breaker OPEN (%d failures, retry in %.0fs)",
                    self._consecutive_failures,
                    self._circuit_open_until - now,
                )
                self._publish_status(
                    {
                        "state": "circuit_open",
                        "instruction": instruction,
                        "retry_in": round(self._circuit_open_until - now, 1),
                    }
                )
                self.planner_status.publish(f"FALLBACK_TO_SEMANTIC_PLANNER: {instruction}")
                return
            logger.info("VLA: circuit half-open, probing backend")

        color = self._latest_color
        if color is None:
            logger.warning("VLA: no color image available, skipping instruction")
            self._publish_status({"state": "no_image", "instruction": instruction})
            return

        self._run_vla(instruction, color)

    # ------------------------------------------------------------------
    # Inference dispatch
    # ------------------------------------------------------------------

    def _run_vla(self, instruction: str, image: Image) -> None:
        """Dispatch VLA inference on the worker thread."""
        backend = self._backend
        executor = self._executor
        if backend is None or executor is None:
            self._publish_status({"state": "not_ready", "instruction": instruction})
            return

        with self._metrics_lock:
            self._in_flight += 1
            self._call_count += 1
        t0 = time.time()

        def _infer() -> dict:
            """Synchronous wrapper that runs the async backend call."""
            import asyncio

            image_b64 = self._encode_image_to_b64(image)
            context = {
                "x": self._robot_pose[0],
                "y": self._robot_pose[1],
                "z": self._robot_pose[2],
                "yaw": self._robot_yaw,
                "system_prompt": self._system_prompt,
                "scene_graph": self._scene_graph_summary(),
                "mission_status": self._latest_mission_status or {},
            }
            loop = asyncio.new_event_loop()
            try:
                return loop.run_until_complete(
                    asyncio.wait_for(
                        backend.infer(image_b64, instruction, context),
                        timeout=self._timeout_sec,
                    )
                )
            finally:
                loop.close()

        def _on_done(future) -> None:
            latency_ms = (time.time() - t0) * 1000.0
            try:
                action = future.result(timeout=self._timeout_sec + 2)
                self._handle_success(action, instruction, latency_ms)
            except Exception as exc:
                self._handle_error(exc, instruction, latency_ms)
            finally:
                with self._metrics_lock:
                    self._in_flight = max(0, self._in_flight - 1)

        future = executor.submit(_infer)
        future.add_done_callback(_on_done)

    def _encode_image_to_b64(self, image: Image) -> str:
        """Encode an Image message to base64 JPEG (BGR->RGB->JPEG)."""
        rgb = image.to_rgb()
        arr = np.asarray(rgb.data)
        # Resize if larger than max size while preserving aspect ratio
        h, w = arr.shape[:2]
        if max(h, w) > self._image_max_size:
            scale = self._image_max_size / max(h, w)
            new_w = int(round(w * scale))
            new_h = int(round(h * scale))
            arr = self._resize_array(arr, new_w, new_h)

        # Try cv2 first, then PIL, then fail with a clear message
        encoded = self._encode_jpeg_cv2(arr)
        if encoded is None:
            encoded = self._encode_jpeg_pil(arr)
        if encoded is None:
            raise ImportError(
                "VLA image encoding requires opencv-python or Pillow. Install one of them or disable the VLA module."
            )
        return base64.b64encode(encoded).decode("ascii")

    @staticmethod
    def _resize_array(arr: np.ndarray, new_w: int, new_h: int) -> np.ndarray:
        """Resize a numpy image array."""
        try:
            import cv2  # type: ignore[import-untyped]

            return cv2.resize(arr, (new_w, new_h))
        except Exception:
            logger.debug("VLA: cv2 resize failed, trying PIL", exc_info=True)
        try:
            from PIL import Image as PilImage  # type: ignore[import-untyped]

            mode = "RGB" if arr.ndim == 3 else "L"
            pil_img = PilImage.fromarray(arr, mode=mode)
            pil_img = pil_img.resize((new_w, new_h), PilImage.Resampling.LANCZOS)
            return np.asarray(pil_img)
        except Exception:
            logger.debug("VLA: PIL resize failed, falling back to numpy", exc_info=True)
        # Pure numpy nearest-neighbour fallback
        row_idx = (np.arange(new_h) * arr.shape[0] / new_h).astype(int)
        col_idx = (np.arange(new_w) * arr.shape[1] / new_w).astype(int)
        if arr.ndim == 2:
            return arr[np.ix_(row_idx, col_idx)]
        return arr[np.ix_(row_idx, col_idx, np.arange(arr.shape[2]))]

    @staticmethod
    def _encode_jpeg_cv2(arr: np.ndarray) -> bytes | None:
        try:
            import cv2  # type: ignore[import-untyped]

            success, buf = cv2.imencode(".jpg", arr)
            if success:
                return buf.tobytes()
        except Exception:
            logger.debug("VLA: cv2 JPEG encode failed", exc_info=True)
        return None

    @staticmethod
    def _encode_jpeg_pil(arr: np.ndarray) -> bytes | None:
        try:
            from PIL import Image as PilImage  # type: ignore[import-untyped]

            mode = "RGB" if arr.ndim == 3 else "L"
            pil_img = PilImage.fromarray(arr, mode=mode)
            buf = io.BytesIO()
            pil_img.save(buf, format="JPEG", quality=85)
            return buf.getvalue()
        except Exception:
            logger.debug("VLA: PIL JPEG encode failed", exc_info=True)
        return None

    def _scene_graph_summary(self) -> str:
        """Short scene-graph summary for the prompt context."""
        sg = self._latest_scene_graph
        if sg is None:
            return ""
        labels = []
        for obj in sg.objects:
            label = obj.label or "object"
            pos = obj.position
            labels.append(f"{label} at ({pos.x:.1f},{pos.y:.1f},{pos.z:.1f})")
        return "; ".join(labels[:10])

    # ------------------------------------------------------------------
    # Result handling
    # ------------------------------------------------------------------

    def _handle_success(self, action: dict, instruction: str, latency_ms: float) -> None:
        """Process a successful backend response."""
        with self._metrics_lock:
            self._success_count += 1
            self._total_latency_ms += latency_ms
            if self._consecutive_failures > 0:
                logger.info("VLA: backend recovered after %d failures", self._consecutive_failures)
            self._consecutive_failures = 0
            self._circuit_open_until = 0.0

        self.vla_action.publish(dict(action))
        self._publish_status(
            {
                "state": "success",
                "instruction": instruction,
                "action": action.get("action"),
                "confidence": action.get("confidence"),
                "latency_ms": round(latency_ms, 1),
            }
        )

        confidence = float(action.get("confidence", 0.0))
        if confidence < self._confidence_threshold:
            logger.info(
                "VLA: low confidence %.2f < %.2f, forwarding to VisualServo",
                confidence,
                self._confidence_threshold,
            )
            self.servo_target.publish(instruction)
            self.planner_status.publish(f"LOW_CONFIDENCE_SERVO: {instruction}")
            return

        self._route_action(action)

    def _handle_error(self, error: Exception, instruction: str, latency_ms: float) -> None:
        """Process a backend error and update circuit breaker."""
        with self._metrics_lock:
            self._error_count += 1
        logger.error("VLA: inference failed: %s", error)

        msg = str(error).lower()
        transient = any(
            p in msg
            for p in (
                "timeout",
                "timed out",
                "429",
                "rate limit",
                "connection",
                "temporary",
                "unavailable",
                "503",
            )
        )
        with self._metrics_lock:
            if transient:
                self._consecutive_failures += 1
            else:
                # Permanent errors (401, 403, bad model) open immediately
                self._consecutive_failures = self._cb_threshold
            if self._consecutive_failures >= self._cb_threshold:
                self._circuit_open_until = time.time() + self._cb_cooldown
                logger.warning(
                    "VLA: circuit breaker OPEN after %d consecutive failures (cooldown %.0fs)",
                    self._consecutive_failures,
                    self._cb_cooldown,
                )

        self._publish_status(
            {
                "state": "error",
                "instruction": instruction,
                "error": str(error),
                "latency_ms": round(latency_ms, 1),
            }
        )
        self.planner_status.publish(f"FALLBACK_TO_SEMANTIC_PLANNER: {instruction}")

    def _route_action(self, action: dict) -> None:
        """Route a high-confidence action to goal_pose or cmd_vel."""
        robot_x, robot_y = self._robot_pose[0], self._robot_pose[1]
        target_x = float(action.get("target_x", robot_x))
        target_y = float(action.get("target_y", robot_y))
        target_z = float(action.get("target_z", 0.0))
        distance = float(np.hypot(target_x - robot_x, target_y - robot_y))

        action_type = action.get("action", "stop")
        if action_type == "stop":
            self.cmd_vel.publish(Twist())
            self.planner_status.publish(f"VLA_STOP: {action.get('reason', '')}")
            return

        # Hysteresis: stay in near mode until distance exceeds far threshold
        if self._near_mode_active:
            use_servo = distance <= self._far_threshold
        else:
            use_servo = distance < self._near_threshold

        if use_servo:
            self._near_mode_active = True
            self._publish_cmd_vel(action, distance)
        else:
            self._near_mode_active = False
            self._publish_goal_pose(target_x, target_y, target_z)

    def _publish_goal_pose(self, x: float, y: float, z: float) -> None:
        """Publish a map-frame goal pose."""
        dx = x - self._robot_pose[0]
        dy = y - self._robot_pose[1]
        yaw = float(np.arctan2(dy, dx))
        self.goal_pose.publish(
            PoseStamped(
                pose=Pose(
                    position=Vector3(x, y, z),
                    orientation=Quaternion.from_yaw(yaw),
                ),
                frame_id=VLA_MAP_FRAME_ID,
            )
        )

    def _publish_cmd_vel(self, action: dict, distance: float) -> None:
        """Publish a Twist command for close-range approach."""
        # Simple proportional controller toward the target
        robot_x, robot_y = self._robot_pose[0], self._robot_pose[1]
        target_x = float(action.get("target_x", robot_x))
        target_y = float(action.get("target_y", robot_y))
        dx = target_x - robot_x
        dy = target_y - robot_y
        angle_to_target = float(np.arctan2(dy, dx))
        yaw_error = self._normalize_angle(angle_to_target - self._robot_yaw)

        linear_gain = 0.5
        angular_gain = 1.0
        max_linear = 0.5
        max_angular = 0.5

        linear_x = min(linear_gain * distance, max_linear)
        angular_z = max(-max_angular, min(angular_gain * yaw_error, max_angular))

        self.cmd_vel.publish(
            Twist(
                linear=Vector3(linear_x, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, angular_z),
            )
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize an angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return float(angle)

    def _publish_status(self, status: dict) -> None:
        """Publish VLA status at most 10 Hz."""
        status["timestamp"] = time.time()
        self.vla_status.publish(status)

    # ------------------------------------------------------------------
    # RPC / skill interface
    # ------------------------------------------------------------------

    @skill
    def vla_navigate(self, instruction: str) -> dict:
        """Send a natural-language navigation instruction to the VLA module."""
        self._on_instruction(instruction)
        return {
            "ok": True,
            "instruction": instruction,
            "backend": self._backend_name,
        }

    @skill
    def vla_status(self) -> dict:
        """Return current VLA health and metrics."""
        return self.health()

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        """Return VLA health metrics."""
        info = super().port_summary()
        with self._metrics_lock:
            avg_ms = self._total_latency_ms / self._success_count if self._success_count > 0 else 0.0
            success_rate = self._success_count / max(1, self._call_count)
            if self._consecutive_failures >= self._cb_threshold:
                circuit_state = "half-open" if time.time() >= self._circuit_open_until else "open"
            else:
                circuit_state = "closed"
            info["vla"] = {
                "backend": self._backend_name,
                "model": self._model or "default",
                "call_count": self._call_count,
                "success_count": self._success_count,
                "error_count": self._error_count,
                "success_rate": round(success_rate, 3),
                "avg_latency_ms": round(avg_ms, 1),
                "circuit_breaker": circuit_state,
                "consecutive_failures": self._consecutive_failures,
                "in_flight": self._in_flight,
                "confidence_threshold": self._confidence_threshold,
                "timeout_sec": self._timeout_sec,
            }
        return info
