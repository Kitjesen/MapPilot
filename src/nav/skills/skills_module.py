from __future__ import annotations

import itertools
import json
import math
import threading
import time
from collections import OrderedDict
from typing import Any

from nav.services.goals import build_goal_pose
from runtime.module import Module, skill
from runtime.msgs.geometry import Vector3
from runtime.msgs.nav import NavigationGoalStatus, NavigationState, Odometry
from runtime.registry import register
from runtime.stream import In, Out
from runtime.tf.frames import map_frame_id, normalize_frame_id

# Match the Host bus's default live-state freshness window; use receipt time.
_OBSERVATION_MAX_AGE_S = 2.0


@register("navigation_skills", "default", description="MCP/AI skills for navigation control")
@register("nav", "skills", description="MCP/AI navigation command adapter")
class NavSkills(Module, layer=6):
    """Translate AI tool calls into the canonical navigation command sink."""

    runtime_id = "nav.skills"
    SOFT_DEPENDS = ["nav.goals", "host.bus"]

    goal_status: In[dict]
    navigation_state: In[NavigationState]
    navigation_goal_status: In[NavigationGoalStatus]
    odometry: In[Odometry]

    goal_command: Out[str]

    def __init__(self, planning_frame_id: str | None = None, **config: Any) -> None:
        super().__init__(**config)
        self._planning_frame_id = normalize_frame_id(planning_frame_id) or map_frame_id()
        self._cached_navigation_state: dict[str, Any] = {}
        self._navigation_state_received_s = 0.0
        self._robot_position: tuple[Vector3, str, float] | None = None
        self._acks: dict[str, dict[str, Any]] = {}
        self._pending: set[str] = set()
        self._owned_request_ids: OrderedDict[str, None] = OrderedDict()
        self._native_goal_statuses: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._command_lock = threading.RLock()
        self._sequence = itertools.count(1)

    def setup(self) -> None:
        self.goal_status.subscribe(self._on_goal_status)
        self.navigation_state.subscribe(self._on_navigation_state)
        self.navigation_goal_status.subscribe(self._on_navigation_goal_status)
        self.odometry.subscribe(self._on_odometry)

    def _on_goal_status(self, status: dict) -> None:
        if not isinstance(status, dict):
            return
        request_id = str(status.get("request_id") or "")
        if request_id:
            with self._command_lock:
                if request_id in self._pending:
                    self._acks[request_id] = dict(status)

    def _on_navigation_state(self, state: NavigationState) -> None:
        with self._command_lock:
            self._cached_navigation_state = state.to_dict()
            self._navigation_state_received_s = time.monotonic()

    def _on_odometry(self, odometry: Odometry) -> None:
        with self._command_lock:
            self._robot_position = (
                Vector3(odometry.x, odometry.y, odometry.z),
                normalize_frame_id(odometry.frame_id),
                time.monotonic(),
            )

    def _on_navigation_goal_status(self, status: NavigationGoalStatus) -> None:
        with self._command_lock:
            if status.request_id not in self._owned_request_ids:
                return
            self._native_goal_statuses.pop(status.request_id, None)
            self._native_goal_statuses[status.request_id] = status.to_dict()
            while len(self._native_goal_statuses) > 256:
                self._native_goal_statuses.popitem(last=False)

    @skill(
        description=(
            "Submit a navigation goal in the configured planning map frame. This can move the robot. "
            "The receipt is admission only: retain request_id and query get_navigation_result for native feedback. "
            "A missing or pending result does not establish success. Cancel using the task_id in native status."
        ),
        parameters={
            "x": "Goal X in metres in the configured planning map frame.",
            "y": "Goal Y in metres in the same frame.",
            "yaw": "Goal heading in radians about map Z.",
            "z": "Goal height in metres in the same frame; omit only when fresh map-frame robot height is available.",
        },
        read_only=False,
    )
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

    @skill(
        description=(
            "Request a stop of the named navigation task without hardware E-stop. "
            "Use task_id from native navigation status, not a request_id. "
            "The receipt acknowledges submission; check native status for termination."
        ),
        parameters={"task_id": "Native navigation task ID of the mission to stop."},
        read_only=False,
    )
    def stop_navigation(self, task_id: str) -> str:
        """Stop the active navigation mission without invoking hardware E-stop."""
        task = str(task_id or "").strip()
        if not task:
            return self._json_rejection("cancel", "task_id_required", "task_id is required")
        return self._submit(
            {"action": "cancel", "task_id": task, "reason": "navigation_stop"}
        )

    @skill(
        description=(
            "Request cancellation of a specific native navigation mission. "
            "Use task_id from native status, not the submission request_id. "
            "An accepted receipt does not establish termination; query native task status afterward."
        ),
        parameters={
            "task_id": "Native navigation task ID of the mission to cancel.",
            "reason": "Human-readable cancellation reason.",
        },
        read_only=False,
    )
    def cancel_mission(self, task_id: str, reason: str = "user_cancel") -> str:
        """Cancel the active navigation mission."""
        task = str(task_id or "").strip()
        if not task:
            return self._json_rejection("cancel", "task_id_required", "task_id is required")
        clean_reason = str(reason or "user_cancel").strip() or "user_cancel"
        return self._submit({"action": "cancel", "task_id": task, "reason": clean_reason})

    @skill(
        description=(
            "Read canonical native navigation mission status. UNKNOWN means feedback is absent or stale, "
            "not that the robot is idle or the task succeeded. Use get_navigation_result(request_id) "
            "to associate feedback with this adapter's specific submission."
        ),
        read_only=True,
    )
    def get_navigation_status(self) -> str:
        """Return the canonical navigation mission status."""
        return json.dumps(self._current_status())

    @skill(
        description=(
            "Read native lifecycle feedback for a request submitted by this navigation adapter. "
            "Pass request_id from the submission receipt. If found=false, do not infer completion. "
            "If found=true, inspect status for the actual native task and outcome."
        ),
        parameters={"request_id": "Submission request_id returned by this adapter, not task_id or Follow ID."},
        output_schema={
            "type": "object",
            "properties": {
                "found": {"type": "boolean"},
                "request_id": {"type": "string"},
                "reason": {"type": "string"},
                "status": {"type": ["object", "null"], "description": "Native lifecycle feedback, or null while pending."},
            },
            "required": ["found", "request_id", "reason"],
        },
        read_only=True,
    )
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
    def start_inspection(
        self,
        route_id: str,
        revision: int = 0,
    ) -> str:
        """Start a stored native inspection route."""
        route = str(route_id or "").strip()
        if not route:
            return self._json_rejection(
                "inspection",
                "route_id_required",
                "route_id is required",
            )
        if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
            return self._json_rejection(
                "inspection",
                "invalid_route_revision",
                "revision must be a non-negative integer",
            )
        return self._submit(
            {
                "action": "inspection",
                "route_id": route,
                "revision": revision,
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
        status = self._current_status()
        state = str(status["state"])
        return json.dumps(
            {
                "state": state,
                "active": self._is_active_state(state),
                "reason": status.get("reason", ""),
            }
        )

    @skill
    def get_navigation_progress(self) -> str:
        """Return a concise progress summary derived from mission status."""
        status = self._current_status()
        if status["state"] == "UNKNOWN":
            return json.dumps({**status, "active": False})
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

    @staticmethod
    def _is_active_state(state: str) -> bool:
        return str(state).lower() in {
            "planning",
            "executing",
            "patrolling",
            "recovering",
        }

    def _current_status(self) -> dict[str, Any]:
        with self._command_lock:
            if not self._cached_navigation_state:
                reason = "navigation_state_unavailable"
            elif time.monotonic() - self._navigation_state_received_s > _OBSERVATION_MAX_AGE_S:
                reason = "navigation_state_stale"
            else:
                status = dict(self._cached_navigation_state)
                status["state"] = status.get("lifecycle_state_name", "UNKNOWN")
                status["source"] = "native_navigation_state"
                return status
        return {"state": "UNKNOWN", "reason": reason, "planning_frame_id": self._planning_frame_id}

    @staticmethod
    def _as_int(value: Any) -> int:
        try:
            return max(0, int(value))
        except (TypeError, ValueError):
            return 0

    def _resolve_goal_z(self, z: float | None) -> float:
        """Use explicit map height or the current map-frame robot height."""
        if z is not None:
            return float(z)
        with self._command_lock:
            odometry = self._robot_position
        now = time.monotonic()
        if odometry is not None and now - odometry[2] <= _OBSERVATION_MAX_AGE_S:
            position, frame, _ = odometry
            if frame == self._planning_frame_id and math.isfinite(position.z):
                return position.z
        raise ValueError("z is required until fresh map-frame odometry is available")

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
