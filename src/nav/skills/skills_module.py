# 192.168.127.10

from __future__ import annotations

import itertools
import json
import math
import threading
import time
from collections import OrderedDict
from typing import Any

from nav.model.status import MissionStatus
from nav.services.goals import build_goal_pose
from runtime.module import Module, skill
from runtime.msgs.nav import NavigationGoalStatus, NavigationState
from runtime.registry import register
from runtime.runtime_interface import map_frame_id, normalize_frame_id
from runtime.stream import In, Out


@register("navigation_skills", "default", description="MCP/AI skills for navigation control")
@register("nav", "skills", description="MCP/AI navigation command adapter")
class NavSkills(Module, layer=6):
    """Translate AI tool calls into the canonical navigation command sink."""

    runtime_id = "nav.skills"
    SOFT_DEPENDS = ["nav.goals", "nav.mission"]

    mission_status: In[MissionStatus]
    goal_status: In[dict]
    navigation_state: In[NavigationState]
    navigation_goal_status: In[NavigationGoalStatus]

    goal_command: Out[str]

    def __init__(self, planning_frame_id: str | None = None, **config: Any) -> None:
        super().__init__(**config)
        self._planning_frame_id = normalize_frame_id(planning_frame_id) or map_frame_id()
        self._cached_status: dict[str, Any] = {}
        self._cached_navigation_state: dict[str, Any] = {}
        self._acks: dict[str, dict[str, Any]] = {}
        self._pending: set[str] = set()
        self._owned_request_ids: OrderedDict[str, None] = OrderedDict()
        self._native_goal_statuses: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._command_lock = threading.RLock()
        self._sequence = itertools.count(1)

    def setup(self) -> None:
        self.mission_status.subscribe(self._on_mission_status)
        self.goal_status.subscribe(self._on_goal_status)
        self.navigation_state.subscribe(self._on_navigation_state)
        self.navigation_goal_status.subscribe(self._on_navigation_goal_status)

    def _on_mission_status(self, status: MissionStatus) -> None:
        if not isinstance(status, dict):
            return
        self._cached_status = dict(status)
        frame = normalize_frame_id(status.get("planning_frame_id"))
        if frame:
            self._planning_frame_id = frame

    def _on_goal_status(self, status: dict) -> None:
        if not isinstance(status, dict):
            return
        request_id = str(status.get("request_id") or "")
        if request_id:
            with self._command_lock:
                if request_id in self._pending:
                    self._acks[request_id] = dict(status)

    def _on_navigation_state(self, state: NavigationState) -> None:
        self._cached_navigation_state = state.to_dict()

    def _on_navigation_goal_status(self, status: NavigationGoalStatus) -> None:
        with self._command_lock:
            if status.request_id not in self._owned_request_ids:
                return
            self._native_goal_statuses.pop(status.request_id, None)
            self._native_goal_statuses[status.request_id] = status.to_dict()
            while len(self._native_goal_statuses) > 256:
                self._native_goal_statuses.popitem(last=False)

    @skill
    def navigate_to(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        z: float | None = None,
    ) -> str:
        """Submit a map-frame navigation goal."""
        try:
            goal = build_goal_pose(
                x=x,
                y=y,
                z=self._resolve_goal_z(z),
                yaw=yaw,
                planning_frame_id=self._planning_frame_id,
            )
        except (TypeError, ValueError) as exc:
            return self._json_rejection("goto", "invalid_coordinates", str(exc))
        return self._submit(
            {
                "action": "goto",
                "x": goal.x,
                "y": goal.y,
                "z": goal.z,
                "yaw": goal.yaw,
                "frame_id": goal.frame_id,
            }
        )

    @skill
    def stop_navigation(self) -> str:
        """Stop the active navigation mission without invoking hardware E-stop."""
        return self._submit({"action": "cancel", "reason": "navigation_stop"})

    @skill
    def cancel_mission(self, reason: str = "user_cancel") -> str:
        """Cancel the active navigation mission."""
        clean_reason = str(reason or "user_cancel").strip() or "user_cancel"
        return self._submit({"action": "cancel", "reason": clean_reason})

    @skill
    def get_navigation_status(self) -> str:
        """Return the canonical navigation mission status."""
        if self._cached_navigation_state:
            status = dict(self._cached_navigation_state)
            status["state"] = status.get("lifecycle_state_name", "UNKNOWN")
            status["source"] = "native_navigation_state"
            return json.dumps(status)
        if not self._cached_status:
            return json.dumps(
                {
                    "state": "UNKNOWN",
                    "reason": "mission_status_unavailable",
                    "planning_frame_id": self._planning_frame_id,
                }
            )
        return json.dumps(self._cached_status)

    @skill
    def get_navigation_result(self, request_id: str) -> str:
        """Return the native lifecycle result for a request submitted by this adapter."""

        normalized = str(request_id or "").strip()
        with self._command_lock:
            owned = normalized in self._owned_request_ids
            status = self._native_goal_statuses.get(normalized)
        if not normalized:
            return json.dumps(
                {
                    "found": False,
                    "request_id": "",
                    "reason": "request_id_required",
                }
            )
        if not owned:
            return json.dumps(
                {
                    "found": False,
                    "request_id": normalized,
                    "reason": "request_not_owned_by_nav_skills",
                }
            )
        return json.dumps(
            {
                "found": status is not None,
                "request_id": normalized,
                "status": status,
                "reason": "" if status is not None else "request_status_pending",
            }
        )

    @skill
    def start_patrol(
        self,
        waypoints: list[dict[str, Any]],
        loop: bool = False,
    ) -> str:
        """Submit a structured map-frame patrol route."""
        if not isinstance(waypoints, list) or not waypoints:
            return self._json_rejection(
                "patrol",
                "invalid_waypoints",
                "waypoints must be a non-empty list",
            )
        return self._submit(
            {
                "action": "patrol",
                "waypoints": waypoints,
                "loop": bool(loop),
            }
        )

    @skill
    def navigate_to_deg(
        self,
        x: float,
        y: float,
        yaw_deg: float = 0.0,
        z: float | None = None,
    ) -> str:
        """Submit a map-frame navigation goal with heading in degrees."""
        try:
            yaw = math.radians(float(yaw_deg))
        except (TypeError, ValueError) as exc:
            return self._json_rejection("goto", "invalid_yaw", str(exc))
        return self.navigate_to(x, y, yaw=yaw, z=z)

    @skill
    def is_navigating(self) -> str:
        """Return whether a navigation mission is currently active."""
        state = self._current_state_name()
        return json.dumps(
            {
                "state": state,
                "active": self._is_active_state(state),
            }
        )

    @skill
    def get_navigation_progress(self) -> str:
        """Return a concise progress summary derived from mission status."""
        status = self._current_status()
        if not status:
            return json.dumps(
                {
                    "state": "UNKNOWN",
                    "active": False,
                    "reason": "mission_status_unavailable",
                }
            )
        state = str(status.get("state", "UNKNOWN"))
        wp_total = self._as_int(status.get("wp_total"))
        wp_index = self._as_int(status.get("wp_index"))
        remaining = self._as_int(status.get("remaining_waypoints", max(0, wp_total - wp_index)))
        native_progress = status.get("progress")
        try:
            progress_pct = round(100.0 * float(native_progress), 1)
        except (TypeError, ValueError):
            progress_pct = round(100.0 * wp_index / wp_total, 1) if wp_total > 0 else 0.0
        return json.dumps(
            {
                "state": state,
                "active": self._is_active_state(state),
                "mission_mode": status.get("mission_mode"),
                "progress_pct": progress_pct,
                "wp_index": wp_index,
                "wp_total": wp_total,
                "remaining_waypoints": remaining,
                "distance_to_goal_m": status.get("distance_to_goal_m"),
                "active_waypoint_distance_m": status.get("active_waypoint_distance_m"),
                "failure_reason": status.get("failure_reason", ""),
            }
        )

    @skill
    def navigate_relative(
        self,
        dx: float,
        dy: float = 0.0,
        dyaw: float = 0.0,
    ) -> str:
        """Submit a goal relative to the current robot pose.

        Body-frame semantics: +dx moves forward along the current heading,
        +dy moves to the robot's left, and dyaw (radians, CCW) rotates the
        target heading. The offset is resolved into a map-frame goal and
        dispatched through the same goto sink as navigate_to.
        """
        position = (self._cached_status or {}).get("position")
        if not isinstance(position, dict) or position.get("x") is None:
            return self._json_rejection("goto", "pose_unavailable", "current robot pose is not available yet")
        try:
            px = float(position["x"])
            py = float(position["y"])
            pyaw = float(position.get("yaw", 0.0))
            fdx = float(dx)
            fdy = float(dy)
            fdyaw = float(dyaw)
        except (TypeError, ValueError, KeyError) as exc:
            return self._json_rejection("goto", "invalid_offset", str(exc))
        cos_yaw = math.cos(pyaw)
        sin_yaw = math.sin(pyaw)
        target_x = px + fdx * cos_yaw - fdy * sin_yaw
        target_y = py + fdx * sin_yaw + fdy * cos_yaw
        return self.navigate_to(target_x, target_y, yaw=pyaw + fdyaw)

    @skill
    def get_robot_pose(self) -> str:
        """Return the latest robot pose in the planning frame."""
        status = self._cached_status
        position = status.get("position") if isinstance(status, dict) else None
        if not isinstance(position, dict) or position.get("x") is None:
            return json.dumps(
                {
                    "available": False,
                    "reason": "mission_status_unavailable",
                    "planning_frame_id": self._planning_frame_id,
                }
            )
        return json.dumps(
            {
                "available": True,
                "frame_id": status.get("planning_frame_id", self._planning_frame_id),
                "x": position.get("x"),
                "y": position.get("y"),
                "z": position.get("z"),
                "yaw": position.get("yaw"),
            }
        )

    @staticmethod
    def _is_active_state(state: str) -> bool:
        return str(state).lower() in {
            "planning",
            "executing",
            "patrolling",
            "recovering",
        }

    def _current_status(self) -> dict[str, Any]:
        if self._cached_navigation_state:
            status = dict(self._cached_navigation_state)
            status["state"] = status.get("lifecycle_state_name", "UNKNOWN")
            return status
        return self._cached_status

    def _current_state_name(self) -> str:
        return str(self._current_status().get("state", "UNKNOWN"))

    @staticmethod
    def _as_int(value: Any) -> int:
        try:
            return max(0, int(value))
        except (TypeError, ValueError):
            return 0

    def _resolve_goal_z(self, z: float | None) -> float:
        """Use explicit z, then the latest robot height, then zero."""
        if z is not None:
            return float(z)
        try:
            position = self._cached_status.get("position", {})
            return float(position.get("z", 0.0))
        except (TypeError, ValueError):
            return 0.0

    def _request_id(self) -> str:
        return f"nav-{time.time_ns()}-{next(self._sequence)}"

    def _submit(self, command: dict[str, Any]) -> str:
        request_id = self._request_id()
        payload = dict(command)
        payload["request_id"] = request_id
        try:
            encoded = json.dumps(payload, separators=(",", ":"))
        except (TypeError, ValueError) as exc:
            return json.dumps(
                {
                    "request_id": request_id,
                    "action": str(command.get("action") or "unknown"),
                    "accepted": False,
                    "state": "rejected",
                    "reason": "command_not_serializable",
                    "error": str(exc),
                }
            )
        with self._command_lock:
            self._acks.pop(request_id, None)
            self._pending.add(request_id)
            self._owned_request_ids.pop(request_id, None)
            self._owned_request_ids[request_id] = None
            while len(self._owned_request_ids) > 256:
                expired, _ = self._owned_request_ids.popitem(last=False)
                self._native_goal_statuses.pop(expired, None)
            self.goal_command.publish(encoded)
            ack = self._acks.pop(request_id, None)
            self._pending.discard(request_id)
        if ack is None:
            return json.dumps(
                {
                    "request_id": request_id,
                    "action": str(command.get("action") or "unknown"),
                    "accepted": False,
                    "state": "unavailable",
                    "reason": "goal_service_ack_missing",
                }
            )
        result = dict(ack)
        accepted = result.get("accepted") is True or result.get("success") is True
        result["accepted"] = accepted
        result["state"] = "accepted" if accepted else "rejected"
        result.setdefault("reason", str(result.get("message") or ""))
        if not accepted:
            with self._command_lock:
                self._owned_request_ids.pop(request_id, None)
                self._native_goal_statuses.pop(request_id, None)
        return json.dumps(result)

    def _json_rejection(self, action: str, reason: str, error: str) -> str:
        return json.dumps(
            {
                "request_id": self._request_id(),
                "action": action,
                "accepted": False,
                "state": "rejected",
                "reason": reason,
                "error": error,
                "planning_frame_id": self._planning_frame_id,
            }
        )
