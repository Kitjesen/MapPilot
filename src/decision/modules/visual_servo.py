"""Visual servo Module."""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from typing import Any

from decision.vision.person import PersonTracker
from runtime.module import Module, skill
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import Detection3D
from runtime.msgs.sensor import Image
from runtime.registry import register
from runtime.runtime_interface import map_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

MODE_IDLE = "idle"
MODE_FIND = "find"
MODE_FOLLOW = "follow"  # person following
VISUAL_SERVO_MAP_FRAME_ID = map_frame_id()


@register("visual_servo", "default", description="Map-frame object approach and person following")
class VisualServoModule(Module, layer=4):
    """Turn current map-frame detections into native navigation goals."""

    _GOAL_YAW_DEADBAND_RAD = math.radians(10.0)
    _MISSING_DETECTION_GRACE_S = 0.5

    # -- Inputs --
    color_image: In[Image]
    robot_pose: In[PoseStamped]
    detections_3d: In[list]
    servo_target: In[str]  # "find:red chair" / "follow:person in red" / "stop"
    goal_status: In[dict]

    # -- Outputs --
    goal_pose: Out[PoseStamped]
    goal_cancel: Out[str]
    servo_status: Out[dict]

    def __init__(
        self,
        follow_distance: float = 1.5,
        target_distance: float = 1.5,
        lost_timeout: float = 5.0,
        goal_rate_hz: float = 2.5,
        goal_deadband_m: float = 0.25,
        **kw,
    ):
        super().__init__(**kw)

        self._person_tracker = PersonTracker(
            follow_distance=follow_distance,
            lost_timeout=lost_timeout,
        )
        self._target_distance = float(target_distance)
        self._lost_timeout = float(lost_timeout)

        self._mode: str = MODE_IDLE
        self._target_label: str = ""
        self._target_id: str | None = None

        self._robot_pose: tuple = (0.0, 0.0, 0.0)  # (x, y, yaw)
        self._robot_ground_z = 0.0
        self._latest_detections: list[Detection3D] = []
        self._latest_detection_ts = 0.0
        self._target_last_seen = 0.0
        self._target_visible = False

        self._last_status_time = 0.0
        self._goal_interval_s = 1.0 / float(goal_rate_hz)
        self._goal_deadband_m = float(goal_deadband_m)
        self._last_goal_position: tuple[float, float, float] | None = None
        self._last_goal_yaw: float | None = None
        self._last_goal_time = 0.0
        self._goal_published = False
        self._navigation_tasks: set[str] = set()
        self._navigation_task_id: str | None = None
        self._navigation_state = "idle"
        self._navigation_reason = ""

        # Person-following target selection (description-based "person in red")
        self._vision_client = None  # LLM client w/ vision (VLM select)
        self._latest_bgr: np.ndarray | None = None  # BGR frame for CLIP/VLM selection
        self._latest_bgr_ts = 0.0
        self._follow_select_pending = False  # need to (re)select target
        self._follow_select_running = False  # VLM selection in flight
        self._follow_select_method = ""  # clip | vlm | unavailable
        self._intent_generation = 0
        self._state_lock = threading.RLock()
        self._last_perception_time = 0.0
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._llm_module = None

    def on_system_modules(self, modules: dict) -> None:
        """Discover the optional vision-capable LLM."""
        self._llm_module = modules.get("LLMModule")
        self._refresh_vision_client()

    def _refresh_vision_client(self) -> None:
        client = getattr(self._llm_module, "client", None)
        self._vision_client = client if self._client_supports_vision(client) else None

    def can_select_follow_target(self) -> bool:
        """Return whether descriptive person selection is available."""
        self._refresh_vision_client()
        return (
            self._single_person_id() is not None
            or self._person_tracker.has_image_selector
            or self._vision_client is not None
        )

    def _single_person_id(self) -> str | None:
        if (
            self._last_perception_time <= 0.0
            or time.monotonic() - self._last_perception_time > self._lost_timeout
        ):
            return None
        people = [
            detection
            for detection in self._latest_detections
            if (detection.label or "").lower() in self._PERSON_LABELS
        ]
        if len(people) != 1 or not people[0].id:
            return None
        return str(people[0].id)

    @staticmethod
    def _client_supports_vision(client) -> bool:
        """Use the client's declared capability instead of guessing from its name."""
        return bool(
            client is not None
            and getattr(client, "supports_vision", False) is True
            and callable(getattr(client, "chat", None))
        )

    def setup(self) -> None:
        """Subscribe to current perception and operator intent."""
        self.color_image.subscribe(self._on_color)
        self.robot_pose.subscribe(self._on_robot_pose)
        self.detections_3d.subscribe(self._on_detections)
        self.servo_target.subscribe(self._on_servo_target)
        self.goal_status.subscribe(self._on_goal_status)
        self.color_image.set_policy("throttle", interval=0.1)

    def start(self) -> None:
        """Start the timeout check after every Module has completed setup."""
        self._refresh_vision_client()
        super().start()
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watch_perception,
            name="visual-servo-watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()
        self._publish_status(force=True)

    def stop(self) -> None:
        """Cancel owned motion before stopping the Module."""
        self._watchdog_stop.set()
        watchdog = self._watchdog_thread
        if watchdog is not None and watchdog is not threading.current_thread():
            watchdog.join(timeout=0.5)
        with self._state_lock:
            self._cancel_tracking()
        super().stop()

    def _watch_perception(self) -> None:
        interval = min(0.1, max(0.01, self._lost_timeout / 2.0))
        while not self._watchdog_stop.wait(interval):
            with self._state_lock:
                if (
                    (not self._goal_published and not self._navigation_tasks)
                    or self._last_perception_time <= 0.0
                    or time.monotonic() - self._last_perception_time <= self._lost_timeout
                ):
                    continue
                self._intent_generation += 1
                self._target_visible = False
                self._cancel_navigation_goal("visual_servo_perception_stale")
                self._latest_detections = []
                self._latest_detection_ts = 0.0
                if self._mode == MODE_FOLLOW:
                    self._person_tracker.reset()
                    self._person_tracker._description = self._target_label
                    self._follow_select_pending = True
                    self._follow_select_running = False
                    self._follow_select_method = ""
                self._last_perception_time = 0.0
                self._publish_status(force=True)

    def _on_robot_pose(self, pose: PoseStamped) -> None:
        with self._state_lock:
            if pose.frame_id != VISUAL_SERVO_MAP_FRAME_ID:
                return
            self._robot_pose = (pose.x, pose.y, pose.yaw)
            self._robot_ground_z = pose.z

    def _on_detections(self, detections: list) -> None:
        with self._state_lock:
            follow_available_before = self.can_select_follow_target()
            previous_single_id = self._single_person_id()
            self._last_perception_time = time.monotonic()
            self._latest_detections = [
                detection for detection in detections if isinstance(detection, Detection3D)
            ]
            current_single_id = self._single_person_id()
            single_person_confirmed = (
                current_single_id is not None
                and current_single_id == previous_single_id
            )
            self._latest_detection_ts = max(
                (float(detection.ts) for detection in self._latest_detections),
                default=0.0,
            )
            if self._mode == MODE_FIND:
                self._tick_find()
            elif self._mode == MODE_FOLLOW:
                if self._target_id is not None:
                    self._tick_follow()
                elif (
                    not self._follow_select_pending
                    or single_person_confirmed
                    or self._selection_image_matches()
                ):
                    self._tick_follow(
                        single_person_confirmed=single_person_confirmed,
                    )
            elif follow_available_before != self.can_select_follow_target():
                self._publish_status(force=True)

    def _on_servo_target(self, target: str) -> None:
        """Parse servo command: 'find:<desc>', 'follow:<desc>', 'stop'."""
        with self._state_lock:
            target = target.strip()
            if target.lower() == "stop":
                self._intent_generation += 1
                self._cancel_tracking()
                return

            if ":" in target:
                mode_str, label = target.split(":", 1)
                mode_str = mode_str.strip().lower()
                label = label.strip()
            else:
                mode_str = "find"
                label = target

            target_id = label if mode_str == "follow_id" else None
            if target_id is not None:
                mode_str = MODE_FOLLOW

            if mode_str not in {MODE_FIND, MODE_FOLLOW}:
                logger.warning("VisualServo: unknown mode '%s'", mode_str)
                return

            if (
                self._mode == mode_str
                and self._target_label == ("" if target_id is not None else label)
                and self._target_id == target_id
            ):
                return
            self._intent_generation += 1
            if self._goal_published or self._navigation_tasks:
                self._cancel_navigation_goal("visual_servo_target_changed")
            self._reset_tracking_state()

            if mode_str == MODE_FIND:
                self._mode = MODE_FIND
                self._target_label = label
                logger.info("VisualServo: find mode - target='%s'", label)
            else:
                self._mode = MODE_FOLLOW
                self._target_label = "" if target_id is not None else label
                self._target_id = target_id
                self._person_tracker._description = self._target_label
                self._follow_select_pending = True
                self._follow_select_running = False
                self._follow_select_method = ""
                if target_id is not None:
                    logger.info("VisualServo: follow mode - target_id='%s'", target_id)
                    self._tick_follow()
                else:
                    logger.info("VisualServo: follow mode - target='%s' (selecting)", label)

            self._publish_status(force=True)

    def _on_goal_status(self, status: dict) -> None:
        """Fold the existing native goal lifecycle into the operator-visible state."""
        if not isinstance(status, dict):
            return
        action = str(status.get("action") or "").strip().lower()
        if action not in {"visual_servo", "visual_servo_stop"}:
            return

        task_id = str(status.get("task_id") or "").strip()
        state = str(status.get("state") or "").strip().lower()
        reason = str(status.get("reason") or status.get("message") or "").strip()
        accepted = status.get("accepted") is True
        terminal = status.get("terminal") is True
        source = str(status.get("source") or "").strip()

        with self._state_lock:
            if action == "visual_servo_stop":
                if not task_id or task_id not in self._navigation_tasks:
                    return
                if self._navigation_task_id and task_id != self._navigation_task_id:
                    return
                if accepted:
                    self._navigation_state = "cancel_requested"
                    self._navigation_reason = reason
                elif self._navigation_tasks:
                    self._navigation_state = "failed"
                    self._navigation_reason = reason
                self._publish_status(force=True)
                return

            if source == "native_goal_status":
                if task_id not in self._navigation_tasks:
                    return
                if task_id == self._navigation_task_id:
                    self._navigation_state = state
                    self._navigation_reason = reason
                if terminal:
                    self._navigation_tasks.discard(task_id)
                    if task_id == self._navigation_task_id:
                        self._navigation_task_id = None
                        self._goal_published = False
                        if state == "failed":
                            self._clear_goal_request(keep_rate_limit=True)
                        elif state == "cancelled":
                            self._clear_goal_request()
                self._publish_status(force=True)
                return

            if accepted and task_id:
                self._navigation_tasks.add(task_id)
                self._navigation_task_id = task_id
                self._navigation_state = state or "accepted"
                self._navigation_reason = reason
            elif not accepted:
                if self._navigation_tasks and task_id not in self._navigation_tasks:
                    return
                if self._navigation_task_id and task_id != self._navigation_task_id:
                    return
                self._navigation_state = state or "rejected"
                self._navigation_reason = reason
                self._goal_published = False
                self._clear_goal_request(keep_rate_limit=True)
            self._publish_status(force=True)

    def _on_color(self, img: Image) -> None:
        """Cache the image used to select a person from current detections."""
        with self._state_lock:
            self._latest_bgr = img.to_bgr().data if hasattr(img, "to_bgr") else img.data
            self._latest_bgr_ts = float(getattr(img, "ts", 0.0) or 0.0)
            if (
                self._mode == MODE_FOLLOW
                and self._follow_select_pending
                and self._selection_image_matches()
            ):
                self._tick_follow()

    def _selection_image_matches(self) -> bool:
        return (
            self._latest_bgr is not None
            and self._latest_bgr_ts > 0.0
            and self._latest_detection_ts > 0.0
            and abs(self._latest_bgr_ts - self._latest_detection_ts) <= 0.1
        )

    def _tick_find(self) -> None:
        """Tick find."""
        target = self._find_target_detection()
        if target is None:
            self._target_visible = False
            if (
                (self._goal_published or self._navigation_tasks)
                and self._target_last_seen > 0.0
                and time.monotonic() - self._target_last_seen > self._lost_timeout
            ):
                self._cancel_navigation_goal("visual_servo_target_lost")
            self._publish_status()
            return

        self._target_visible = True
        self._target_last_seen = time.monotonic()

        target_position = (
            float(target.position.x),
            float(target.position.y),
            float(target.position.z),
        )
        distance = math.hypot(
            target_position[0] - self._robot_pose[0],
            target_position[1] - self._robot_pose[1],
        )
        if distance <= self._target_distance:
            logger.info("VisualServo: arrived at target '%s'", self._target_label)
            self._cancel_tracking()
            return

        self._publish_goal_from_3d(
            self._standoff_position(
                target_position,
                self._target_distance,
            )
        )

        self._publish_status()

    def _find_target_detection(self) -> Detection3D | None:
        target_lower = self._target_label.lower()
        matches = [
            detection
            for detection in self._latest_detections
            if detection.label
            and (
                target_lower in detection.label.lower()
                or detection.label.lower() in target_lower
            )
        ]
        return max(matches, key=lambda detection: detection.confidence, default=None)

    def _standoff_position(
        self,
        target: tuple[float, float, float],
        distance: float,
    ) -> np.ndarray:
        robot_x, robot_y, _ = self._robot_pose
        dx = target[0] - robot_x
        dy = target[1] - robot_y
        target_distance = math.hypot(dx, dy)
        if target_distance <= distance or target_distance < 1e-6:
            return np.array([robot_x, robot_y, self._robot_ground_z])
        travel = target_distance - distance
        return np.array(
            [
                robot_x + dx / target_distance * travel,
                robot_y + dy / target_distance * travel,
                self._robot_ground_z,
            ]
        )

    def _tick_follow(self, *, single_person_confirmed: bool = False) -> None:
        """Tick follow."""
        scene_objects = self._scene_objects_from_detections(self._latest_detections)
        selected_from_current_frame = False

        if self._target_id is not None and not self._person_tracker.target_selected:
            detections_are_fresh = (
                self._last_perception_time > 0.0
                and time.monotonic() - self._last_perception_time <= self._lost_timeout
            )
            exact_objects = (
                [obj for obj in scene_objects if obj.get("id") == self._target_id]
                if detections_are_fresh
                else []
            )
            if not exact_objects:
                self._target_visible = False
                self._publish_status()
                return
            crop = None
            if self._latest_bgr is not None:
                crop = PersonTracker._crop_person(self._latest_bgr, exact_objects[0])
            self._person_tracker.lock_target(exact_objects[0], crop)
            self._follow_select_pending = False
            self._follow_select_method = "id"
            selected_from_current_frame = True

        elif self._follow_select_method and self._person_tracker.is_lost():
            self._cancel_navigation_goal("visual_servo_target_lost")
            self._person_tracker.reset()
            self._person_tracker._description = self._target_label
            self._follow_select_pending = True
            self._follow_select_method = ""

        # Re-selection: a target we locked earlier has been lost past the Re-ID

        # recovered when they reappear (e.g. after occlusion / turning a corner),
        # instead of staying lost forever. Guarded by _follow_select_method so it
        # only fires after an initial lock, and never while a VLM select is in flight.
        if self._target_id is None and (
            self._follow_select_method
            and not self._follow_select_pending
            and not self._follow_select_running
            and self._person_tracker.needs_vlm_reselect()
        ):
            logger.info("VisualServo: follow target lost - re-selecting '%s'", self._target_label)
            self._cancel_navigation_goal("visual_servo_target_lost")
            self._person_tracker.reset()
            self._person_tracker._description = self._target_label
            self._follow_select_pending = True
            self._follow_select_method = ""

        # Lock onto the described person before plain tracking (once per follow).
        if (
            self._target_id is None
            and self._follow_select_pending
            and not self._follow_select_running
        ):
            if single_person_confirmed:
                self._try_select_follow_target(
                    scene_objects,
                    single_person_confirmed=True,
                )
            else:
                self._try_select_follow_target(scene_objects)
            selected_from_current_frame = (
                not self._follow_select_pending
                and self._person_tracker.target_selected
            )

        if (
            self._follow_select_pending
            or self._follow_select_running
            or not self._person_tracker.target_selected
        ):
            self._target_visible = False
            self._publish_status()
            return

        # Track (BGR frame for CLIP/OSNet Re-ID) and emit follow waypoint.
        self._target_visible = selected_from_current_frame or self._person_tracker.update(
            scene_objects,
            self._latest_bgr,
        )
        if not self._target_visible:
            person = self._person_tracker.status()
            last_seen = float((person or {}).get("last_seen") or 0.0)
            if last_seen <= 0.0 or time.time() - last_seen >= self._MISSING_DETECTION_GRACE_S:
                self._cancel_navigation_goal("visual_servo_target_lost")
            self._publish_status()
            return

        tracked = self._person_tracker.status()
        current_track_id = str((tracked or {}).get("id") or "")
        if self._target_id is not None and current_track_id:
            self._target_id = current_track_id

        waypoint = self._person_tracker.get_follow_waypoint(list(self._robot_pose))
        if waypoint is not None:
            person_position = self._person_tracker.get_person_position()
            person_yaw = None
            if person_position is not None:
                person_yaw = math.atan2(
                    person_position[1] - self._robot_pose[1],
                    person_position[0] - self._robot_pose[0],
                )
            # Person following always goes through the native planning stack.
            self._publish_goal_from_3d(
                np.array(
                    [
                        float(waypoint["x"]),
                        float(waypoint["y"]),
                        self._robot_ground_z,
                    ]
                ),
                yaw=person_yaw,
            )

        self._publish_status()

    def _scene_objects_from_detections(
        self,
        detections: list[Detection3D],
    ) -> list[dict]:
        """Convert current map-frame detections for person tracking."""
        scene_objects = []
        for obj in detections:
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
                    "bbox": list(obj.bbox_2d),
                    "confidence": float(getattr(obj, "confidence", 0.5)),
                    "ts": float(obj.ts),
                }
            )
        return scene_objects

    _PERSON_LABELS = ("person", "people", "human", "pedestrian")

    def _try_select_follow_target(
        self,
        scene_objects: list[dict],
        *,
        single_person_confirmed: bool = False,
    ) -> None:
        """Try select follow target."""
        persons = [o for o in scene_objects if o.get("label", "").lower() in self._PERSON_LABELS]
        if not persons:
            return

        if len(persons) == 1 and single_person_confirmed:
            crop = None
            if self._latest_bgr is not None:
                crop = PersonTracker._crop_person(self._latest_bgr, persons[0])
            self._person_tracker.lock_target(persons[0], crop)
            self._follow_select_pending = False
            self._follow_select_method = "single"
            logger.info("VisualServo: locked the only visible person")
            return

        if self._latest_bgr is None:
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
            generation = self._intent_generation
            threading.Thread(
                target=self._run_vlm_select_sync,
                args=(desc, crops, valid, generation),
                name="vs_vlm_select",
                daemon=True,
            ).start()
            return

        if len(persons) == 1:
            return

        # A descriptive target without an image-capable selector is not a safe
        # instruction to follow an arbitrary nearby person.
        self._follow_select_pending = False
        self._follow_select_method = "unavailable"
        logger.warning(
            "VisualServo: target selection unavailable for '%s'",
            desc,
        )

    def _run_vlm_select_sync(
        self,
        desc: str,
        crops: list,
        persons: list,
        generation: int,
    ) -> None:
        """Background thread: run async VLM person selection (look at crops)."""
        try:
            loop = asyncio.new_event_loop()
            try:
                idx = loop.run_until_complete(
                    self._person_tracker.choose_target_with_vlm(
                        desc,
                        crops,
                        self._vlm_chat,
                    )
                )
            finally:
                loop.close()
            with self._state_lock:
                if (
                    generation != self._intent_generation
                    or self._mode != MODE_FOLLOW
                    or self._target_label != desc
                ):
                    return
                if idx >= 0:
                    self._person_tracker.lock_target(persons[idx], crops[idx])
                    self._follow_select_method = "vlm"
                else:
                    self._follow_select_method = "unavailable"
            logger.info("VisualServo: VLM follow selection -> idx=%d", idx)
        except Exception:
            logger.exception("VisualServo: VLM follow selection failed")
            with self._state_lock:
                if generation == self._intent_generation:
                    self._follow_select_method = "unavailable"
        finally:
            with self._state_lock:
                if generation == self._intent_generation:
                    self._follow_select_running = False
                    self._follow_select_pending = False

    async def _vlm_chat(self, messages: list) -> str:
        """Async VLM call for person selection (OpenAI multimodal messages)."""
        client = self._vision_client
        if client is None:
            return ""
        return await client.chat(messages, temperature=0.3)

    def _reset_tracking_state(self) -> None:
        """Reset local target selection."""
        self._person_tracker.reset()
        self._mode = MODE_IDLE
        self._target_label = ""
        self._target_id = None
        self._follow_select_pending = False
        self._follow_select_running = False
        self._follow_select_method = ""
        self._target_visible = False
        self._target_last_seen = 0.0
        self._last_goal_position = None
        self._last_goal_yaw = None
        self._last_goal_time = 0.0

    def _clear_goal_request(self, *, keep_rate_limit: bool = False) -> None:
        self._last_goal_position = None
        self._last_goal_yaw = None
        if not keep_rate_limit:
            self._last_goal_time = 0.0

    def _cancel_tracking(self, *, publish_status: bool = True) -> None:
        """Return to idle and cancel only the navigation task owned by this module."""
        with self._state_lock:
            self._cancel_navigation_goal("visual_servo_stop")
            self._reset_tracking_state()
            logger.info("VisualServo: tracking cancelled, idle")
            if publish_status:
                self._publish_status(force=True)

    def _cancel_navigation_goal(self, reason: str) -> None:
        if not self._goal_published and not self._navigation_tasks:
            return
        self.goal_cancel.publish(reason)
        self._goal_published = False
        if self._navigation_tasks:
            self._navigation_state = "cancel_requested"
            self._navigation_reason = reason
        self._clear_goal_request()

    def _publish_goal_from_3d(
        self,
        pos_3d: np.ndarray,
        *,
        yaw: float | None = None,
    ) -> bool:
        """Publish a changed map goal at the bounded visual-follow update rate."""
        position = (float(pos_3d[0]), float(pos_3d[1]), float(pos_3d[2]))
        now = time.monotonic()

        if yaw is None:
            rx, ry, _ = self._robot_pose
            yaw = math.atan2(position[1] - ry, position[0] - rx)

        if self._last_goal_time > 0.0 and now - self._last_goal_time < self._goal_interval_s:
            return False
        if self._last_goal_position is not None:
            moved = math.hypot(
                position[0] - self._last_goal_position[0],
                position[1] - self._last_goal_position[1],
            )
            yaw_delta = (
                abs(
                    math.atan2(
                        math.sin(yaw - self._last_goal_yaw),
                        math.cos(yaw - self._last_goal_yaw),
                    )
                )
                if self._last_goal_yaw is not None
                else math.inf
            )
            if moved < self._goal_deadband_m and yaw_delta < self._GOAL_YAW_DEADBAND_RAD:
                return False

        self._last_goal_position = position
        self._last_goal_yaw = yaw
        self._last_goal_time = now
        self._goal_published = True
        self.goal_pose.publish(
            PoseStamped(
                pose=Pose(
                    position=Vector3(x=position[0], y=position[1], z=position[2]),
                    orientation=Quaternion.from_euler(0, 0, yaw),
                ),
                frame_id=VISUAL_SERVO_MAP_FRAME_ID,
            )
        )
        return True

    def _publish_status(self, *, force: bool = False) -> None:
        """Publish servo status at max 2 Hz."""
        now = time.time()
        if not force and now - self._last_status_time < 0.5:
            return
        self._last_status_time = now

        self.servo_status.publish(self._status(now))

    def _status(self, now: float | None = None) -> dict:
        if self._follow_select_running or self._follow_select_pending:
            select_state = "selecting"
        else:
            select_state = self._follow_select_method
        person = self._person_tracker.status()
        robot_position = [
            float(self._robot_pose[0]),
            float(self._robot_pose[1]),
            float(self._robot_ground_z),
        ]
        goal_position = (
            list(self._last_goal_position)
            if self._last_goal_position is not None
            else None
        )
        distance_m = None
        if person is not None:
            position = person.get("position")
            if isinstance(position, list) and len(position) >= 2:
                distance_m = math.hypot(
                    float(position[0]) - robot_position[0],
                    float(position[1]) - robot_position[1],
                )
        return {
            "ts": time.time() if now is None else now,
            "frame_id": VISUAL_SERVO_MAP_FRAME_ID,
            "mode": self._mode,
            "target": self._target_label,
            "target_id": self._target_id,
            "select": select_state,
            "follow_available": self.can_select_follow_target(),
            "target_visible": self._target_visible,
            "state": self._runtime_state(),
            "navigation_state": self._navigation_state,
            "navigation_task_id": self._navigation_task_id,
            "navigation_reason": self._navigation_reason,
            "goal_rate_hz": 1.0 / self._goal_interval_s,
            "robot_position": robot_position,
            "goal_position": goal_position,
            "distance_m": distance_m,
            "desired_distance_m": float(self._person_tracker.follow_distance),
            "person": person,
        }

    def _runtime_state(self) -> str:
        if self._mode == MODE_IDLE:
            return "stopping" if self._navigation_tasks else "idle"
        if self._navigation_state in {"failed", "rejected", "unknown"}:
            return "failed"
        if self._mode == MODE_FIND:
            return "approaching" if self._target_visible else "searching"
        if (
            self._follow_select_pending
            or self._follow_select_running
            or not self._person_tracker.target_selected
        ):
            return "selecting"
        return "following" if self._target_visible else "lost"

    def health(self) -> dict[str, Any]:
        """Return current visual tracking capability and request state."""
        info = super().port_summary()
        info["tracking_active"] = self._mode != MODE_IDLE
        info["mode"] = self._mode
        info["follow_available"] = self.can_select_follow_target()
        info["state"] = self._runtime_state()
        info["navigation_state"] = self._navigation_state
        return info

    @skill
    def find_object(self, target: str) -> str:
        """Trigger visual find mode for a target object."""
        self._on_servo_target(f"find:{target}")
        return f"Visual servo: finding '{target}'"

    @skill
    def follow_person(self, description: str) -> str:
        """Trigger person following mode."""
        if not self.can_select_follow_target():
            return "Visual servo: target selection unavailable"
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
        return self._status()
