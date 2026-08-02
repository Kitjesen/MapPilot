"""Visual servo Module."""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from typing import Any

from decision.vision.bbox import STATE_ARRIVED, STATE_LOST, BBoxNavConfig, BBoxNavigator
from decision.vision.person import PersonTracker
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

MODE_IDLE = "idle"
MODE_FIND = "find"
MODE_FOLLOW = "follow"  # person following
VISUAL_SERVO_MAP_FRAME_ID = map_frame_id()


@register("visual_servo", "default", description="Visual servoing: bbox tracking + person following")
class VisualServoModule(Module, layer=4):
    """Visual servo navigation with distance-based mode switching.

    Far targets are projected to PoseStamped goals for Navigation.
    Near targets are converted to Twist commands for the velocity mux path.
    Follow mode tracks a person and publishes follow waypoints through Navigation.
    """

    _run_in_worker = True
    _worker_group = "semantic"

    # -- Inputs --
    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraIntrinsics]
    odometry: In[Odometry]
    servo_target: In[str]  # "find:red chair" / "follow:person in red" / "stop"
    scene_graph: In[SceneGraph]  # detections with bboxes

    # -- Outputs --
    goal_pose: Out[PoseStamped]
    cmd_vel: Out[Twist]
    nav_stop: Out[int]  # 1=pause Navigation, 0=release
    servo_status: Out[dict]

    def __init__(
        self,
        servo_takeover_distance: float = 3.0,
        servo_takeover_hysteresis: float = 0.3,
        follow_distance: float = 1.5,
        target_distance: float = 1.5,
        lost_timeout: float = 5.0,
        **kw,
    ):
        super().__init__(**kw)
        self._takeover_dist = servo_takeover_distance
        # Asymmetric threshold prevents flapping between goal_pose / cmd_vel
        # when the target hovers near the boundary.
        self._takeover_enter = servo_takeover_distance - servo_takeover_hysteresis
        self._takeover_exit = servo_takeover_distance + servo_takeover_hysteresis

        self._bbox_nav = BBoxNavigator(
            config=BBoxNavConfig(
                target_distance=target_distance,
                lost_timeout=lost_timeout,
                servo_takeover_distance=servo_takeover_distance,
            )
        )
        self._person_tracker = PersonTracker(
            follow_distance=follow_distance,
            lost_timeout=lost_timeout,
        )

        self._mode: str = MODE_IDLE
        self._target_label: str = ""

        # Cached sensor data
        self._latest_depth: np.ndarray | None = None
        self._latest_rgb: np.ndarray | None = None
        self._intrinsics: tuple | None = None  # (fx, fy, cx, cy)
        self._robot_pose: tuple = (0.0, 0.0, 0.0)  # (x, y, yaw)
        self._latest_sg: SceneGraph | None = None

        self._servo_active = False  # True when publishing cmd_vel (close range)
        self._last_status_time = 0.0

        # Person-following target selection (description-based "person in red")
        self._vision_client = None  # LLM client w/ vision (VLM select)
        self._latest_bgr: np.ndarray | None = None  # BGR frame for CLIP/VLM selection
        self._follow_select_pending = False  # need to (re)select target
        self._follow_select_running = False  # VLM selection in flight
        self._follow_select_method = ""  # clip | vlm | degraded

    def on_system_modules(self, modules: dict) -> None:
        """On system modules."""
        encoder_backend = None
        enc_mod = modules.get("EncoderModule")
        backend = getattr(enc_mod, "_backend", None) if enc_mod is not None else None
        if backend is not None and hasattr(backend, "encode_image"):
            encoder_backend = backend
        if encoder_backend is None:
            perc = modules.get("PerceptionModule")
            cand = getattr(perc, "_clip_encoder", None) if perc is not None else None
            if cand is not None and hasattr(cand, "encode_image"):
                encoder_backend = cand
        if encoder_backend is not None:
            self._person_tracker.set_clip_encoder(encoder_backend)
            logger.info("VisualServo: CLIP encoder attached for person selection")
        else:
            logger.info("VisualServo: no image-capable CLIP encoder; person selection relies on VLM")

        llm_mod = modules.get("LLMModule")
        client = getattr(llm_mod, "_client", None) if llm_mod is not None else None
        if client is not None and self._client_supports_vision(client):
            self._vision_client = client
            logger.info("VisualServo: VLM vision client attached (%s)", type(client).__name__)
        else:
            self._vision_client = None

    @staticmethod
    def _client_supports_vision(client) -> bool:
        """Client supports vision."""
        if client is None or not hasattr(client, "chat"):
            return False
        name = type(client).__name__.lower()
        return not ("moonshot" in name or "kimi" in name or "qwen" in name)

    def setup(self) -> None:
        # the camera publisher (and, transitively, uvicorn).
        self.color_image.subscribe(self._on_color)
        self.color_image.set_policy("latest")
        self.depth_image.subscribe(self._on_depth)
        self.depth_image.set_policy("latest")
        self.camera_info.subscribe(self._on_camera_info)
        self.odometry.subscribe(self._on_odom)
        self.servo_target.subscribe(self._on_servo_target)
        self.scene_graph.subscribe(self._on_scene_graph)

        # Throttle image processing to ~10 Hz
        self.color_image.set_policy("throttle", interval=0.1)

        # Try to enable FusionMOT + OSNet Re-ID for person tracking
        self._person_tracker.enable_fusion_tracking()

    def stop(self) -> None:
        self._cancel_tracking()
        super().stop()

    def _on_depth(self, img: Image) -> None:
        self._latest_depth = img.data

    def _on_camera_info(self, info: CameraIntrinsics) -> None:
        self._intrinsics = (info.fx, info.fy, info.cx, info.cy)

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_pose = (odom.x, odom.y, odom.yaw)

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        self._latest_sg = sg

    def _on_servo_target(self, target: str) -> None:
        """Parse servo command: 'find:<desc>', 'follow:<desc>', 'stop'."""
        target = target.strip()
        if target.lower() == "stop":
            self._cancel_tracking()
            return

        if ":" in target:
            mode_str, label = target.split(":", 1)
            mode_str = mode_str.strip().lower()
            label = label.strip()
        else:
            # Default to find mode
            mode_str = "find"
            label = target

        if mode_str == "find":
            self._mode = MODE_FIND
            self._target_label = label
            self._bbox_nav.stop()  # reset state
            logger.info("VisualServo: find mode - target='%s'", label)
        elif mode_str == "follow":
            self._mode = MODE_FOLLOW
            self._target_label = label
            # Reset tracker and arm description-based selection: the next frame
            # with person detections locks onto the person matching `label` via
            # CLIP (if image-capable) or VLM (look at crops), instead of blindly
            # following the most salient person.
            self._person_tracker.reset()
            self._person_tracker._description = label
            self._follow_select_pending = True
            self._follow_select_running = False
            self._follow_select_method = ""
            logger.info("VisualServo: follow mode - target='%s' (selecting)", label)
        else:
            logger.warning("VisualServo: unknown mode '%s'", mode_str)

        self._publish_status()

    def _on_color(self, img: Image) -> None:
        """On color."""
        self._latest_rgb = img.to_rgb().data if hasattr(img, "to_rgb") else img.data
        # BGR frame for CLIP/VLM person selection (CLIPEncoder + cv2 expect BGR).
        self._latest_bgr = img.to_bgr().data if hasattr(img, "to_bgr") else img.data

        if self._mode == MODE_IDLE:
            return

        if self._mode == MODE_FIND:
            if self._latest_depth is None or self._intrinsics is None:
                return
            self._tick_find()
        elif self._mode == MODE_FOLLOW:
            self._tick_follow()

    def _tick_find(self) -> None:
        """Tick find."""
        bbox = self._find_target_bbox()
        if bbox is None:
            self._bbox_nav.tick_lost_check()
            if self._bbox_nav.state == STATE_LOST:
                self._release_servo()
                self._publish_status()
            return

        result = self._bbox_nav.update(
            bbox=bbox,
            depth_image=self._latest_depth,
            camera_intrinsics=self._intrinsics,
            robot_pose=self._robot_pose,
        )

        state = result["state"]
        distance = result["distance"]
        target_3d = result["target_3d"]

        if state == STATE_ARRIVED:
            logger.info("VisualServo: arrived at target '%s'", self._target_label)
            self._release_servo()
            self._cancel_tracking()
            self._publish_status()
            return

        if target_3d is None:
            return

        # Hysteresis: only switch modes when crossing the outer threshold
        # relative to current state. Sticky behaviour while inside the band.
        if self._servo_active:
            use_servo = distance <= self._takeover_exit
        else:
            use_servo = distance < self._takeover_enter

        if not use_servo:
            self._release_servo()
            self._publish_goal_from_3d(target_3d)
        else:
            self._engage_servo()
            self.cmd_vel.publish(
                Twist(
                    linear=Vector3(x=result["linear_x"], y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=result["angular_z"]),
                )
            )

        self._publish_status()

    def _find_target_bbox(self) -> list | None:
        """Find target bbox."""
        sg = self._latest_sg
        if sg is None or not sg.objects:
            return None

        target_lower = self._target_label.lower()
        best_score = 0.0
        best_bbox = None

        for obj in sg.objects:
            label = (obj.label or "").lower()
            if not label:
                continue
            # Simple keyword matching
            if target_lower in label or label in target_lower:
                score = getattr(obj, "confidence", 0.5)
                if score > best_score:
                    best_score = score
                    # Extract bbox from Detection3D
                    bbox = getattr(obj, "bbox_2d", None)
                    if (bbox is None or len(bbox) == 0) and hasattr(obj, "bbox"):
                        bbox = getattr(obj, "bbox", None)
                    if bbox is not None and len(bbox) > 0:
                        best_bbox = list(bbox)

        return best_bbox

    def _tick_follow(self) -> None:
        """Tick follow."""
        sg = self._latest_sg
        if sg is None:
            return

        scene_objects = self._scene_objects_from_sg(sg)

        # Re-selection: a target we locked earlier has been lost past the Re-ID

        # recovered when they reappear (e.g. after occlusion / turning a corner),
        # instead of staying lost forever. Guarded by _follow_select_method so it
        # only fires after an initial lock, and never while a VLM select is in flight.
        if (
            self._follow_select_method
            and not self._follow_select_pending
            and not self._follow_select_running
            and self._person_tracker.needs_vlm_reselect()
        ):
            logger.info("VisualServo: follow target lost - re-selecting '%s'", self._target_label)
            self._person_tracker.reset()
            self._person_tracker._description = self._target_label
            self._follow_select_pending = True
            self._follow_select_method = ""

        # Lock onto the described person before plain tracking (once per follow).
        if self._follow_select_pending and not self._follow_select_running:
            self._try_select_follow_target(scene_objects)

        # Track (BGR frame for CLIP/OSNet Re-ID) and emit follow waypoint.
        self._person_tracker.update(scene_objects, self._latest_bgr)

        wp = self._person_tracker.get_follow_waypoint(robot_pos=list(self._robot_pose[:2]))
        if wp is not None:
            # Person following always goes through planning stack
            self._release_servo()
            self._publish_goal_from_3d(np.array([float(wp["x"]), float(wp["y"]), float(wp.get("z", 0.0))]))

        self._publish_status()

    def _scene_objects_from_sg(self, sg: SceneGraph) -> list[dict]:
        """Flatten SceneGraph detections into PersonTracker's dict format."""
        scene_objects = []
        for obj in sg.objects:
            bbox = getattr(obj, "bbox_2d", None)
            if (bbox is None or len(bbox) == 0) and hasattr(obj, "bbox"):
                bbox = getattr(obj, "bbox", None)
            scene_objects.append(
                {
                    "id": obj.id,
                    "label": obj.label or "",
                    "position": [
                        float(getattr(obj.position, "x", 0)),
                        float(getattr(obj.position, "y", 0)),
                        float(getattr(obj.position, "z", 0)),
                    ]
                    if obj.position
                    else [0, 0, 0],
                    "bbox": list(bbox) if bbox is not None else [],
                    "confidence": float(getattr(obj, "confidence", 0.5)),
                }
            )
        return scene_objects

    _PERSON_LABELS = ("person", "people", "human", "pedestrian")

    def _try_select_follow_target(self, scene_objects: list[dict]) -> None:
        """Try select follow target."""
        persons = [o for o in scene_objects if o.get("label", "").lower() in self._PERSON_LABELS]
        if not persons or self._latest_bgr is None:
            return

        crops, valid = [], []
        for p in persons:
            crop = PersonTracker._crop_person(self._latest_bgr, p)
            if crop is not None and crop.size > 0:
                crops.append(crop)
                valid.append(p)
        if not crops:
            return

        desc = self._target_label

        idx = self._person_tracker.select_by_clip(desc, crops, valid)
        if idx >= 0:
            self._follow_select_pending = False
            self._follow_select_method = "clip"
            logger.info("VisualServo: locked follow target via CLIP (#%d)", idx + 1)
            return

        if self._vision_client is not None:
            self._follow_select_running = True
            threading.Thread(
                target=self._run_vlm_select_sync,
                args=(desc, crops, valid),
                name="vs_vlm_select",
                daemon=True,
            ).start()
            return

        # Clear pending so we don't spin; PersonTracker.update tracks the
        # highest-confidence person (legacy behaviour).
        self._follow_select_pending = False
        self._follow_select_method = "degraded"
        logger.warning(
            "VisualServo: no CLIP image encoder and no vision LLM - following "
            "most-salient person (description '%s' not applied)",
            desc,
        )

    def _run_vlm_select_sync(self, desc: str, crops: list, persons: list) -> None:
        """Background thread: run async VLM person selection (look at crops)."""
        try:
            loop = asyncio.new_event_loop()
            try:
                idx = loop.run_until_complete(
                    self._person_tracker.select_target_with_vlm(
                        desc,
                        crops,
                        persons,
                        self._vlm_chat,
                    )
                )
            finally:
                loop.close()
            self._follow_select_method = "vlm" if idx >= 0 else "degraded"
            logger.info("VisualServo: VLM follow selection -> idx=%d", idx)
        except Exception:
            logger.exception("VisualServo: VLM follow selection failed")
            self._follow_select_method = "degraded"
        finally:
            self._follow_select_running = False
            self._follow_select_pending = False

    async def _vlm_chat(self, messages: list) -> str:
        """Async VLM call for person selection (OpenAI multimodal messages)."""
        client = self._vision_client
        if client is None:
            return ""
        return await client.chat(messages, temperature=0.3)

    def _engage_servo(self) -> None:
        """Enter close-range servo: pause nav.mission."""
        if not self._servo_active:
            self._servo_active = True
            self.nav_stop.publish(1)
            logger.debug("VisualServo: close-range servo engaged")

    def _release_servo(self) -> None:
        """Exit close-range servo: stop cmd_vel, release nav.mission."""
        if self._servo_active:
            self._servo_active = False
            self.nav_stop.publish(0)
            # Zero velocity
            self.cmd_vel.publish(Twist())
            logger.debug("VisualServo: servo released, planning stack resumed")

    def _cancel_tracking(self) -> None:
        """Cancel tracking."""
        self._release_servo()
        self._bbox_nav.stop()
        self._mode = MODE_IDLE
        self._target_label = ""
        self._latest_sg = None
        logger.info("VisualServo: tracking cancelled, idle")

    def _publish_goal_from_3d(self, pos_3d: np.ndarray) -> None:
        """Convert 3D world position to PoseStamped and publish."""
        # Compute yaw toward target
        rx, ry, _ = self._robot_pose
        dx = float(pos_3d[0]) - rx
        dy = float(pos_3d[1]) - ry
        yaw = math.atan2(dy, dx)

        self.goal_pose.publish(
            PoseStamped(
                pose=Pose(
                    position=Vector3(x=float(pos_3d[0]), y=float(pos_3d[1]), z=float(pos_3d[2])),
                    orientation=Quaternion.from_euler(0, 0, yaw),
                ),
                frame_id=VISUAL_SERVO_MAP_FRAME_ID,
            )
        )

    def _publish_status(self) -> None:
        """Publish servo status at max 2 Hz."""
        now = time.time()
        if now - self._last_status_time < 0.5:
            return
        self._last_status_time = now

        if self._follow_select_running or self._follow_select_pending:
            select_state = "selecting"
        else:
            select_state = self._follow_select_method
        status = {
            "mode": self._mode,
            "target": self._target_label,
            "bbox_state": self._bbox_nav.state,
            "servo_active": self._servo_active,
            "select": select_state,
            "distance": 0.0,
        }
        t3d = self._bbox_nav.target_3d
        if t3d is not None:
            rx, ry, _ = self._robot_pose
            status["distance"] = float(np.hypot(t3d[0] - rx, t3d[1] - ry))
            status["target_3d"] = t3d.tolist()

        self.servo_status.publish(status)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["tracking_active"] = self._mode != MODE_IDLE
        if self._mode == MODE_IDLE:
            info["mode"] = "idle"
        elif self._servo_active:
            info["mode"] = "near"
        else:
            info["mode"] = "far"
        return info

    @skill
    def find_object(self, target: str) -> str:
        """Trigger visual find mode for a target object."""
        self._on_servo_target(f"find:{target}")
        return f"Visual servo: finding '{target}'"

    @skill
    def follow_person(self, description: str) -> str:
        """Trigger person following mode."""
        self._on_servo_target(f"follow:{description}")
        return f"Visual servo: following '{description}'"

    @skill
    def stop_servo(self) -> str:
        """Stop all visual tracking."""
        self._cancel_tracking()
        return "Visual servo: stopped"

    @skill
    def get_servo_status(self) -> dict:
        """Return current servo state."""
        return {
            "mode": self._mode,
            "target": self._target_label,
            "bbox_state": self._bbox_nav.state,
            "servo_active": self._servo_active,
        }

    @skill
    def tune_bbox_gains(self, duration: float = 6.0) -> dict:
        """Report that live bbox gain tuning is not available from this module."""
        logger.warning(
            "VisualServo: tune_bbox_gains requested but no live relay/motion "
            "authority path is implemented (duration=%.1fs)",
            duration,
        )
        return {
            "robot_id": self._bbox_nav._robot_id,
            "status": "unavailable",
            "error": (
                "live bbox gain tuning is not implemented: VisualServoModule "
                "does not own a relay excitation path, motion authority gate, "
                "or safety rollback for persisted gains"
            ),
            "converged": False,
            "requires": [
                "motion authority through the configured velocity arbiter or an explicit calibration mode",
                "fresh odometry sampled while relay excitation is applied",
                "safety interlock and rollback before persisting tuned gains",
            ],
        }
