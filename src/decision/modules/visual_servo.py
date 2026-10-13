"""Visual servo Module."""

from __future__ import annotations

import asyncio
import logging
import math
import os
import threading
import time
import uuid
from collections import OrderedDict
from copy import deepcopy
from typing import Any

from decision.vision.person import PersonTracker, verify_candidate
from runtime.module import Module, skill
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import NavigationGoalStatus, VisualGoalCancel, VisualGoalRequest
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import Detection3D
from runtime.msgs.sensor import Image
from runtime.registry import get, register
from runtime.stream import In, Out
from runtime.tf.frames import map_frame_id

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
    navigation_goal_status: In[NavigationGoalStatus]

    # -- Outputs --
    goal_pose: Out[VisualGoalRequest]
    goal_cancel: Out[VisualGoalCancel]
    servo_status: Out[dict]

    def __init__(
        self,
        follow_distance: float = 1.5,
        target_distance: float = 1.5,
        lost_timeout: float = 5.0,
        goal_rate_hz: float = 2.5,
        goal_deadband_m: float = 0.25,
        person_text_model_path: str = "",
        person_text_device: str = "auto",
        person_appearance_model: str = "",
        person_appearance_device: str = "auto",
        person_appearance_cpu_threads: int = 4,
        person_appearance_min_similarity: float = 0.65,
        person_appearance_margin: float = 0.03,
        vision_service_url: str = "",
        product_session_id: str = "",
        **kw,
    ):
        super().__init__(**kw)

        self._person_tracker = PersonTracker(
            follow_distance=follow_distance,
            lost_timeout=lost_timeout,
            dino_min_similarity=person_appearance_min_similarity,
            dino_margin=person_appearance_margin,
        )
        self._target_distance = float(target_distance)
        self._person_text_model_path = person_text_model_path
        self._person_text_device = person_text_device
        self._person_appearance_model = person_appearance_model
        self._person_appearance_device = person_appearance_device
        self._person_appearance_cpu_threads = int(person_appearance_cpu_threads)
        self._vision_service_url = vision_service_url
        self._remote_vision_client = None
        self._owned_encoders: list[Any] = []
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
        self._goal_requests: dict[str, str] = {}
        self._product_session_id = str(
            product_session_id or os.environ.get("LINGTU_PRODUCT_SESSION_ID", "")
        ).strip()
        self._follow_task_id = ""
        self._follow_tasks: OrderedDict[str, dict] = OrderedDict()
        self._goals = None

        # Person-following target selection (description-based "person in red")
        self._vision_client = None  # LLM client w/ vision (VLM select)
        self._latest_bgr: np.ndarray | None = None  # BGR frame for CLIP/VLM selection
        self._latest_bgr_ts = 0.0
        self._follow_select_pending = False  # need to (re)select target
        self._follow_select_running = False  # VLM selection in flight
        self._follow_verify_running = False  # identity verification in flight
        self._follow_verify_key: tuple[int, str, float | None] | None = None
        self._follow_verify_future = None
        self._follow_select_method = ""  # clip | vlm | unavailable
        self._follow_reference_crop: np.ndarray | None = None
        self._intent_generation = 0
        self._state_lock = threading.RLock()
        self._last_perception_time = 0.0
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._llm_module = None

    def on_system_modules(self, modules: dict) -> None:
        """Discover the optional vision-capable LLM."""
        self._llm_module = modules.get("LLMModule")
        goals = modules.get("nav.goals")
        self._goals = goals
        if goals is not None:
            self._product_session_id = goals.product_session_id
        self._refresh_vision_client()

    def _refresh_vision_client(self) -> None:
        if self._remote_vision_client is not None:
            self._vision_client = self._remote_vision_client
            return
        client = getattr(self._llm_module, "client", None)
        self._vision_client = (
            client
            if self._client_supports_vision(client) and client.is_available()
            else None
        )

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
        if self._person_text_model_path:
            provider = get("encoder", "clip")
            encoder = provider.create(
                model_path=self._person_text_model_path,
                device=self._person_text_device,
            )
            encoder.load_model()
            self._owned_encoders.append(encoder)
            self._person_tracker.set_clip_encoder(encoder)
        elif self._vision_service_url:
            clip_encoder = get("encoder", "clip_remote").create(
                server_url=self._vision_service_url,
            )
            clip_encoder.load_model()
            self._owned_encoders.append(clip_encoder)
            self._person_tracker.set_clip_encoder(clip_encoder)

        if self._person_appearance_model:
            provider = get("encoder", "dino")
            encoder = provider.create(
                model_path=self._person_appearance_model,
                device=self._person_appearance_device,
                cpu_threads=self._person_appearance_cpu_threads,
            )
            encoder.load_model()
            self._owned_encoders.append(encoder)
            self._person_tracker.set_dino_encoder(encoder)
        elif self._vision_service_url:
            dino_encoder = get("encoder", "dino_remote").create(
                server_url=self._vision_service_url,
            )
            dino_encoder.load_model()
            self._owned_encoders.append(dino_encoder)
            self._person_tracker.set_dino_encoder(dino_encoder)

        if self._vision_service_url:
            self._remote_vision_client = get("vlm", "remote").create(
                server_url=self._vision_service_url,
            )
            self._remote_vision_client.load_model()
            self._refresh_vision_client()
        self.color_image.subscribe(self._on_color)
        self.robot_pose.subscribe(self._on_robot_pose)
        self.detections_3d.subscribe(self._on_detections)
        self.servo_target.subscribe(self._on_servo_target)
        self.goal_status.subscribe(self._on_goal_status)
        self.navigation_goal_status.subscribe(self._on_navigation_goal_status)

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
            for encoder in self._owned_encoders:
                shutdown = getattr(encoder, "shutdown", None)
                if callable(shutdown):
                    shutdown()
            self._owned_encoders.clear()
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
                self._clear_identity_verification()
                self._target_visible = False
                self._cancel_navigation_goal("visual_servo_perception_stale")
                self._latest_detections = []
                self._latest_detection_ts = 0.0
                if self._mode == MODE_FOLLOW:
                    if self._person_tracker.has_identity_reference:
                        self._person_tracker.invalidate_reacquisition()
                    else:
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
                if self._follow_task_id and not self._follow_tasks[self._follow_task_id]["terminal"]:
                    logger.warning("Follow cancellation requires its task and Product session IDs")
                    return
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

            if mode_str == MODE_FOLLOW:
                self.start_follow(description=label if target_id is None else "", target_id=target_id or "")
                return

            if self._follow_task_id and not self._follow_tasks[self._follow_task_id]["terminal"]:
                logger.warning("Cancel the active Follow before starting another visual task")
                return
            self._follow_task_id = ""

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
            if not self._navigation_tasks:
                self._navigation_state = "idle"
                self._navigation_reason = ""

            if mode_str == MODE_FIND:
                self._mode = MODE_FIND
                self._target_label = label
                logger.info("VisualServo: find mode - target='%s'", label)
            self._publish_status(force=True)

    @skill
    def start_follow(
        self, description: str = "", target_id: str = "",
        request_id: str = "", client_id: str = "",
    ) -> dict:
        """Start one Follow execution; retain the returned task and session IDs."""
        with self._state_lock:
            description, target_id = description.strip(), target_id.strip()
            request_id = str(request_id or "")
            client_id = str(client_id or "")
            if request_id and client_id:
                for task in self._follow_tasks.values():
                    if (task["request_id"], task["client_id"]) == (request_id, client_id):
                        if (task["target"], task["target_id"]) != (description, target_id):
                            return self._follow_receipt(False, request_id, "request_id_conflict")
                        return self._follow_receipt(True, request_id, "replayed", task)
            if not description and not target_id:
                return self._follow_receipt(False, request_id, "follow_target_required")
            if self._goals is not None and not self._goals.visual_tasks_ready:
                return self._follow_receipt(False, request_id, "recovering_visual_tasks")
            current = self._follow_tasks.get(self._follow_task_id)
            if current is not None and not current["terminal"]:
                return self._follow_receipt(False, request_id, "follow_busy", current)
            if self._navigation_tasks or self._mode != MODE_IDLE:
                return self._follow_receipt(False, request_id, "visual_servo_busy")

            self._intent_generation += 1
            self._reset_tracking_state()
            self._goal_requests.clear()
            self._navigation_task_id = None
            self._navigation_state, self._navigation_reason = "idle", ""
            self._follow_task_id = uuid.uuid4().hex
            task = {
                "follow_task_id": self._follow_task_id,
                "product_session_id": self._product_session_id,
                "request_id": request_id or uuid.uuid4().hex,
                "client_id": client_id,
                "target": description, "target_id": target_id,
                "state": "running", "terminal": False,
                "result": None, "reason": "", "children": {},
                "created_at": time.time(),
            }
            self._follow_tasks[self._follow_task_id] = task
            while len(self._follow_tasks) > 64:
                self._follow_tasks.popitem(last=False)
            self._mode = MODE_FOLLOW
            self._target_label, self._target_id = description, target_id or None
            self._person_tracker._description = description
            self._follow_select_pending = True
            if target_id:
                self._tick_follow()
            self._publish_status(force=True)
            return self._follow_receipt(True, task["request_id"], "accepted", task)

    def _follow_receipt(
        self, accepted: bool, request_id: str, reason: str, task: dict | None = None,
    ) -> dict:
        result = {
            "accepted": accepted, "request_id": request_id, "reason": reason,
            "product_session_id": self._product_session_id,
            "execution_confirmed": False,
        }
        if task is not None:
            result.update(
                follow_task_id=task["follow_task_id"],
                state=task["state"], terminal=task["terminal"],
            )
        return result

    @skill
    def get_follow_task(self, follow_task_id: str, product_session_id: str) -> dict:
        """Query a Follow in this Host's last 64 executions; no restart recovery."""
        with self._state_lock:
            if product_session_id != self._product_session_id:
                return {"found": False, "reason": "product_session_mismatch"}
            task = self._follow_tasks.get(follow_task_id)
            if task is None:
                return {"found": False, "reason": "follow_task_not_found"}
            view = deepcopy(task)
            view.pop("client_id", None)
            view["found"] = True
            view["kind"] = "follow"
            view["available_operations"] = [] if task["terminal"] else ["cancel"]
            view["retention"] = "host_last_64_executions"
            if follow_task_id == self._follow_task_id and not task["terminal"]:
                view["details"] = self._status()
                view["phase"] = self._runtime_state()
                if task["state"] != "cancelling":
                    view["reason"] = self._navigation_reason or self._tracking_reason(self._person_tracker.status())
            return view

    @skill
    def cancel_follow_task(
        self, follow_task_id: str, product_session_id: str,
        reason: str = "operator_cancel", request_id: str = "", client_id: str = "",
    ) -> dict:
        """Cancel only the referenced Follow and wait for its native child terminals."""
        with self._state_lock:
            request_id = request_id or uuid.uuid4().hex
            if product_session_id != self._product_session_id:
                return self._follow_receipt(False, request_id, "product_session_mismatch")
            task = self._follow_tasks.get(follow_task_id)
            if task is None:
                return self._follow_receipt(False, request_id, "follow_task_not_found")
            if task["terminal"]:
                return self._follow_receipt(True, request_id, "already_terminal", task)
            if task["state"] == "cancelling":
                if self._navigation_state == "unknown" or any(
                    child.get("cancel_state") == "unknown" and not child.get("terminal")
                    for child in task["children"].values()
                ):
                    self._cancel_navigation_goal(reason)
                return self._follow_receipt(True, request_id, "cancel_requested", task)
            task["state"], task["reason"] = "cancelling", reason
            self._cancel_tracking()
            self._finish_follow_cancel()
            self._publish_status(force=True)
            return self._follow_receipt(True, request_id, "cancel_requested", task)

    def _finish_follow_cancel(self) -> None:
        task = self._follow_tasks.get(self._follow_task_id)
        if task is None or task["state"] != "cancelling" or self._navigation_tasks:
            return
        task.update(state="cancelled", terminal=True, result="cancelled", finished_at=time.time())
        task["phase"] = "cancelled"
        task["details"] = self._status()

    @staticmethod
    def _trim_child_history(task: dict) -> None:
        completed = [key for key, child in task["children"].items() if child.get("terminal")]
        for key in completed[:-128]:
            del task["children"][key]

    def _on_goal_status(self, status: dict) -> None:
        """Consume admission ACKs; native lifecycle comes through the typed input."""
        if not isinstance(status, dict) or status.get("source") == "native_goal_status":
            return
        action = status.get("action")
        if action not in {"visual_servo", "visual_servo_stop"}:
            return
        task_id = str(status.get("task_id") or "")
        with self._state_lock:
            if task_id not in self._navigation_tasks:
                return
            reason = str(status.get("reason") or status.get("message") or "")
            task = self._follow_tasks.get(self._follow_task_id)
            child = task["children"].get(task_id) if task else None
            if action == "visual_servo_stop":
                if child is not None:
                    child.update(
                        cancel_state="requested" if status.get("accepted") else "unknown",
                        cancel_reason=reason,
                    )
                if task_id == self._navigation_task_id:
                    self._navigation_state = "cancel_requested" if status.get("accepted") else "unknown"
                    self._navigation_reason = reason
                self._publish_status(force=True)
                return
            if status.get("request_id") != self._goal_requests.get(task_id):
                return
            if child is not None and child.get("native_status"):
                # A late ACK must not roll native progress back to 'accepted'.
                return
            accepted = status.get("accepted") is True
            unconfirmed = status.get("admission_unconfirmed") is True or status.get("state") == "unknown"
            state = str(status.get("state") or ("accepted" if accepted else "rejected")).lower()
            if child is not None:
                child.update(state=state, reason=reason, accepted=accepted, terminal=not accepted and not unconfirmed)
            if not accepted and not unconfirmed:
                self._navigation_tasks.discard(task_id)
                self._goal_requests.pop(task_id, None)
                if task is not None:
                    self._trim_child_history(task)
            if task_id == self._navigation_task_id:
                self._navigation_state, self._navigation_reason = state, reason
                if not accepted:
                    self._goal_published = False
                    self._clear_goal_request(keep_rate_limit=True)
            if accepted and self._mode == MODE_IDLE:
                self.goal_cancel.publish(VisualGoalCancel(
                    task_ids=(task_id,), reason="visual_servo_stop",
                    product_session_id=self._product_session_id,
                ))
            self._finish_follow_cancel()
            self._publish_status(force=True)

    def _on_navigation_goal_status(self, status: NavigationGoalStatus) -> None:
        """Apply the ledger-ordered native event only to a currently owned child."""
        with self._state_lock:
            task_id = status.task_id
            if task_id not in self._navigation_tasks:
                return
            state = str(status.to_dict()["state_name"]).lower()
            task = self._follow_tasks.get(self._follow_task_id)
            if task is not None and task_id in task["children"]:
                task["children"][task_id].update(
                    state=state, reason=status.reason, terminal=status.terminal,
                    native_status={
                        "state": state, "reason": status.reason,
                        "terminal": status.terminal, "ts": status.ts,
                        "request_id": status.request_id,
                    },
                )
            if task_id == self._navigation_task_id:
                self._navigation_state, self._navigation_reason = state, status.reason
                if status.terminal:
                    self._navigation_task_id = None
                    self._goal_published = False
                    self._clear_goal_request(keep_rate_limit=state == "failed")
            if status.terminal:
                self._navigation_tasks.discard(task_id)
                self._goal_requests.pop(task_id, None)
                if task is not None:
                    self._trim_child_history(task)
            self._finish_follow_cancel()
            self._publish_status(force=True)

    def _on_color(self, img: Image) -> None:
        """Cache the image used to select a person from current detections."""
        with self._state_lock:
            self._latest_bgr = img.to_bgr().data if hasattr(img, "to_bgr") else img.data
            self._latest_bgr_ts = float(getattr(img, "ts", 0.0) or 0.0)
            if (
                self._mode == MODE_FOLLOW
                and self._selection_image_matches()
                and (
                    self._follow_select_pending
                    or (
                        not self._target_visible
                        and (
                            self._target_id is not None
                            or self._person_tracker.target_selected
                        )
                    )
                )
            ):
                self._tick_follow()

    def _selection_image_matches(self) -> bool:
        return (
            self._latest_bgr is not None
            and self._latest_bgr_ts > 0.0
            and self._latest_detection_ts > 0.0
            and self._latest_bgr_ts == self._latest_detection_ts
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
        matched_bgr = self._latest_bgr if self._selection_image_matches() else None
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
            if matched_bgr is not None:
                crop = PersonTracker._crop_person(matched_bgr, exact_objects[0])
            if self._person_tracker._dino_encoder is not None and crop is None:
                self._target_visible = False
                self._publish_status()
                return
            self._person_tracker.lock_target(exact_objects[0], crop)
            self._follow_select_pending = False
            self._follow_select_method = "id"
            selected_from_current_frame = True

        elif self._follow_select_method and self._person_tracker.is_lost():
            self._cancel_navigation_goal("visual_servo_target_lost")
            if not self._person_tracker.has_identity_reference:
                self._person_tracker.reset()
                self._person_tracker._description = self._target_label
                self._follow_select_pending = True
                self._follow_select_method = ""

        # Legacy selectors must choose again after their Re-ID window. A DINO
        # identity reference stays locked so later frames can recover the same
        # person instead of treating a visible bystander as a new selection.
        if self._target_id is None and (
            self._follow_select_method
            and not self._follow_select_pending
            and not self._follow_select_running
            and not self._person_tracker.has_identity_reference
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
            can_lock_single_without_image = (
                single_person_confirmed
                and self._vision_client is None
                and not self._person_tracker.has_image_selector
                and self._person_tracker._dino_encoder is None
            )
            if matched_bgr is None and not can_lock_single_without_image:
                self._target_visible = False
                self._publish_status()
                return
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
            matched_bgr,
        )
        if not self._target_visible:
            candidate = self._person_tracker.identity_candidate()
            if (
                candidate is not None
                and self._vision_client is not None
            ):
                self._start_identity_verification(candidate)
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
                    "dino_feature": getattr(obj, "dino_feature", None),
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
        matched_bgr = self._latest_bgr if self._selection_image_matches() else None

        if (
            len(persons) == 1
            and single_person_confirmed
            and self._vision_client is None
            and self._follow_reference_crop is None
        ):
            crop = None
            if matched_bgr is not None:
                crop = PersonTracker._crop_person(matched_bgr, persons[0])
            self._person_tracker.lock_target(persons[0], crop)
            if crop is not None:
                self._follow_reference_crop = crop.copy()
            self._follow_select_pending = False
            self._follow_select_method = "single"
            logger.info("VisualServo: locked the only visible person")
            return

        if matched_bgr is None:
            return

        crops, valid = [], []
        for p in persons:
            crop = PersonTracker._crop_person(matched_bgr, p)
            if crop is not None and crop.size > 0:
                crops.append(crop)
                valid.append(p)
        if not crops:
            return

        desc = self._target_label

        idx = -1
        if self._follow_reference_crop is None:
            idx = self._person_tracker.select_by_clip(desc, crops, valid)
        if idx >= 0:
            self._follow_reference_crop = crops[idx].copy()
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

        if len(persons) == 1 and self._follow_reference_crop is None:
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
            loop = getattr(self._llm_module, "_loop", None)
            if loop is None:
                raise RuntimeError("LLMModule is not running")
            reference_crop = (
                self._follow_reference_crop.copy()
                if self._follow_reference_crop is not None
                else None
            )

            async def select_and_verify() -> tuple[int, str | None]:
                idx = await self._person_tracker.choose_target_with_vlm(
                    desc,
                    crops,
                    self._vlm_chat,
                    reference_crop_bgr=reference_crop,
                )
                if idx < 0:
                    return idx, None
                verdict, _ = await verify_candidate(
                    desc,
                    crops[idx],
                    idx,
                    self._vlm_chat,
                    reference_crop_bgr=reference_crop,
                )
                return idx, verdict

            # The shared API client's connections belong to the LLMModule loop.
            future = asyncio.run_coroutine_threadsafe(
                asyncio.wait_for(
                    select_and_verify(),
                    timeout=self._llm_module._timeout_sec,
                ),
                loop,
            )
            idx, verdict = future.result()
            with self._state_lock:
                if (
                    generation != self._intent_generation
                    or self._mode != MODE_FOLLOW
                    or self._target_label != desc
                ):
                    return
                if idx >= 0 and verdict == "MATCH":
                    self._person_tracker.lock_target(persons[idx], crops[idx])
                    if self._follow_reference_crop is None:
                        self._follow_reference_crop = crops[idx].copy()
                    self._follow_select_method = "vlm"
                else:
                    self._follow_select_method = "unavailable"
            logger.info(
                "VisualServo: VLM follow selection -> idx=%d verdict=%s",
                idx,
                verdict,
            )
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

    def _clear_identity_verification(self) -> None:
        future = self._follow_verify_future
        self._follow_verify_future = None
        self._follow_verify_key = None
        self._follow_verify_running = False
        if future is not None and not future.done():
            future.cancel()

    def _start_identity_verification(self, candidate: dict) -> None:
        obj = candidate.get("object")
        crop = candidate.get("crop")
        reference = candidate.get("reference_crop")
        if not isinstance(obj, dict) or crop is None or reference is None:
            return
        candidate_id = str(obj.get("id") or "")
        if not candidate_id:
            return
        source_ts_value = obj.get("ts")
        source_ts = None if source_ts_value is None else float(source_ts_value)
        request_key = (self._intent_generation, candidate_id, source_ts)
        if self._follow_verify_key == request_key:
            return
        self._clear_identity_verification()
        reference_crops = [
            item.copy() for item in candidate.get("reference_crops", ())
        ]
        candidate_crops = [item.copy() for item in candidate.get("crops", ())]
        if not candidate_crops:
            candidate_crops = [crop.copy()]
        self._follow_verify_running = True
        self._follow_verify_key = request_key
        threading.Thread(
            target=self._run_identity_verification_sync,
            args=(
                self._target_label,
                candidate_id,
                source_ts,
                crop.copy(),
                reference.copy(),
                reference_crops,
                candidate_crops,
                request_key,
            ),
            name="vs_identity_verify",
            daemon=True,
        ).start()

    def _run_identity_verification_sync(
        self,
        desc: str,
        candidate_id: str,
        source_ts: float | None,
        crop: np.ndarray,
        reference: np.ndarray,
        reference_crops: list[np.ndarray],
        candidate_crops: list[np.ndarray],
        request_key: tuple[int, str, float | None],
    ) -> None:
        """Verify a DINO reacquisition without committing its pose."""
        generation = request_key[0]
        try:
            loop = getattr(self._llm_module, "_loop", None)
            if loop is None:
                raise RuntimeError("LLMModule is not running")
            future = asyncio.run_coroutine_threadsafe(
                asyncio.wait_for(
                    verify_candidate(
                        desc,
                        crop,
                        candidate_id,
                        self._vlm_chat,
                        reference_crop_bgr=reference,
                        reference_crops_bgr=reference_crops,
                        candidate_crops_bgr=candidate_crops,
                    ),
                    timeout=self._llm_module._timeout_sec,
                ),
                loop,
            )
            with self._state_lock:
                if self._follow_verify_key != request_key:
                    future.cancel()
                else:
                    self._follow_verify_future = future
            verdict, _ = future.result()
            with self._state_lock:
                if (
                    self._follow_verify_key != request_key
                    or generation != self._intent_generation
                    or self._mode != MODE_FOLLOW
                    or self._target_label != desc
                ):
                    return
                self._person_tracker.resolve_identity(candidate_id, source_ts, verdict)
        except Exception:
            with self._state_lock:
                if self._follow_verify_key != request_key:
                    return
            logger.exception("VisualServo: identity verification failed")
            with self._state_lock:
                if (
                    self._follow_verify_key == request_key
                    and generation == self._intent_generation
                    and self._mode == MODE_FOLLOW
                    and self._target_label == desc
                ):
                    self._person_tracker.resolve_identity(
                        candidate_id,
                        source_ts,
                        "UNKNOWN",
                    )
        finally:
            with self._state_lock:
                if self._follow_verify_key == request_key:
                    self._follow_verify_future = None
                    self._follow_verify_key = None
                    self._follow_verify_running = False

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
        self._clear_identity_verification()
        self._follow_select_method = ""
        self._follow_reference_crop = None
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
            self._intent_generation += 1
            task = self._follow_tasks.get(self._follow_task_id)
            if task is not None and not task["terminal"]:
                task.update(state="cancelling", reason=task["reason"] or "visual_servo_stop")
            self._reset_tracking_state()
            self._cancel_navigation_goal("visual_servo_stop")
            self._finish_follow_cancel()
            logger.info("VisualServo: tracking cancelled, idle")
            if publish_status:
                self._publish_status(force=True)

    def _cancel_navigation_goal(self, reason: str) -> None:
        if not self._goal_published and not self._navigation_tasks:
            return
        task_ids = tuple(sorted(self._navigation_tasks))
        self._goal_published = False
        if self._navigation_tasks:
            self._navigation_state = "cancel_requested"
            self._navigation_reason = reason
        self._clear_goal_request()
        if task_ids:
            self.goal_cancel.publish(VisualGoalCancel(
                task_ids=task_ids, reason=reason,
                product_session_id=self._product_session_id,
            ))

    def _publish_goal_from_3d(
        self,
        pos_3d: np.ndarray,
        *,
        yaw: float | None = None,
    ) -> bool:
        """Publish a changed map goal at the bounded visual-follow update rate."""
        if self._navigation_tasks and self._navigation_state in {"unknown", "cancel_requested"}:
            return False
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
        task_id, request_id = uuid.uuid4().hex, uuid.uuid4().hex
        self._goal_requests[task_id] = request_id
        self._navigation_tasks.add(task_id)
        self._navigation_task_id = task_id
        self._navigation_state, self._navigation_reason = "submitting", ""
        task = self._follow_tasks.get(self._follow_task_id)
        if task is not None and not task["terminal"]:
            task["children"][task_id] = {
                "request_id": request_id, "state": "submitting", "terminal": False,
            }
        self.goal_pose.publish(
            VisualGoalRequest(goal=PoseStamped(
                pose=Pose(
                    position=Vector3(x=position[0], y=position[1], z=position[2]),
                    orientation=Quaternion.from_euler(0, 0, yaw),
                ),
                frame_id=VISUAL_SERVO_MAP_FRAME_ID,
            ), task_id=task_id, request_id=request_id, product_session_id=self._product_session_id)
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
        task = self._follow_tasks.get(self._follow_task_id)
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
            "follow_task_id": self._follow_task_id or None,
            "product_session_id": self._product_session_id,
            "task_state": task["state"] if task else None,
            "task_terminal": task["terminal"] if task else None,
            "follow_capabilities": {
                "supported_operations": ["start", "query", "cancel"],
                "retention": "host_last_64_executions",
                "resume_after_restart": False,
            },
            "ts": time.time() if now is None else now,
            "frame_id": VISUAL_SERVO_MAP_FRAME_ID,
            "mode": self._mode,
            "target": self._target_label,
            "target_id": self._target_id,
            "target_identity_id": person.get("identity_id") if person else None,
            "native_track_id": person.get("native_track_id") if person else None,
            "select": select_state,
            "follow_available": self.can_select_follow_target(),
            "target_visible": self._target_visible,
            "state": self._runtime_state(),
            "reason": self._tracking_reason(person),
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

    def _tracking_reason(self, person: dict | None) -> str:
        """Explain visual evidence separately from native navigation and motion."""
        if self._mode == MODE_IDLE:
            return ""
        if (
            self._last_perception_time <= 0.0
            or time.monotonic() - self._last_perception_time > self._lost_timeout
        ):
            return (
                "visual_servo_perception_stale"
                if person is not None or self._navigation_reason == "visual_servo_perception_stale"
                else "waiting_perception"
            )
        if self._mode == MODE_FIND:
            return "" if self._target_visible else "target_not_visible"
        if self._follow_select_running:
            return "target_selection_running"
        if self._follow_select_method == "unavailable":
            return "target_selection_unavailable"
        if not self._person_tracker.target_selected:
            people = [
                detection for detection in self._latest_detections
                if (detection.label or "").lower() in self._PERSON_LABELS
                and (self._target_id is None or str(detection.id) == self._target_id)
            ]
            if not people:
                return "target_not_visible"
            if not self._selection_image_matches() and (
                self._person_tracker._dino_encoder is not None
                or self._person_tracker.has_image_selector
                or self._vision_client is not None
                or (self._target_id is None and len(people) > 1)
            ):
                return "waiting_selection_image"
            return "selecting_target"
        if self._target_visible:
            return ""
        if self._follow_verify_running:
            return "identity_verification_running"
        observation_reason = (person or {}).get("observation_reason")
        if observation_reason:
            return str(observation_reason)
        event = str((person or {}).get("dino_event") or "")
        if event == "awaiting_identity_verification" and self._vision_client is None:
            return "identity_verification_unavailable"
        if event and event not in {"disabled", "ready", "locked", "matched", "identity_verified"}:
            return event
        return "target_not_visible"

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
        if self._person_tracker.has_identity_reference and not self._target_visible:
            return "reacquiring"
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
    def follow_person(self, description: str, request_id: str = "", client_id: str = "") -> dict:
        """Trigger person following mode."""
        return self.start_follow(description=description, request_id=request_id, client_id=client_id)

    @skill
    def stop_servo(self, follow_task_id: str = "", product_session_id: str = "") -> dict:
        """Cancel Follow by reference, or stop the current object approach."""
        with self._state_lock:
            if follow_task_id:
                return self.cancel_follow_task(follow_task_id, product_session_id)
            if self._follow_task_id and not self._follow_tasks[self._follow_task_id]["terminal"]:
                return self._follow_receipt(False, "", "follow_task_reference_required")
            self._cancel_tracking()
            return {"accepted": True, "reason": "stop_requested", "execution_confirmed": False}

    @skill
    def get_servo_status(self) -> dict:
        """Return current servo state."""
        with self._state_lock:
            return self._status()
