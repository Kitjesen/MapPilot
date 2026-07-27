"""Goal service for navigation, building, patrol, and cancel commands."""

from __future__ import annotations

import itertools
import json
import math
import time
import uuid
from typing import Any

from runtime import In, Module, Out
from runtime.msgs import NavigationCommandKind, NavigationCommandReceipt
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.registry import register
from runtime.runtime_interface import map_frame_id, normalize_frame_id


def finite_float(value: Any, *, label: str = "coordinate") -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def frame_id_from_sources(
    *sources: dict[str, Any],
    default_frame_id: str,
) -> str:
    for source in sources:
        frame_id = normalize_frame_id(source.get("frame_id") if isinstance(source, dict) else None)
        if frame_id:
            return frame_id
    return default_frame_id


def build_goal_pose(
    *,
    x: Any,
    y: Any,
    z: Any = 0.0,
    yaw: Any = 0.0,
    frame_id: str | None = None,
    planning_frame_id: str | None = None,
) -> PoseStamped:
    target_frame = normalize_frame_id(frame_id) or (normalize_frame_id(planning_frame_id) or map_frame_id())
    expected_frame = normalize_frame_id(planning_frame_id) or target_frame
    if target_frame != expected_frame:
        raise ValueError(f"unsupported target frame {target_frame!r}; expected {expected_frame!r}")
    goal_x = finite_float(x, label="x")
    goal_y = finite_float(y, label="y")
    goal_z = finite_float(z, label="z")
    goal_yaw = finite_float(yaw, label="yaw")
    return PoseStamped(
        pose=Pose(
            position=Vector3(goal_x, goal_y, goal_z),
            orientation=Quaternion.from_yaw(goal_yaw),
        ),
        frame_id=target_frame,
    )


def normalize_patrol_waypoint(
    item: Any,
    *,
    planning_frame_id: str,
    loop: bool,
) -> dict[str, Any]:
    if isinstance(item, dict):
        frame_id = frame_id_from_sources(item, default_frame_id=planning_frame_id)
        x = finite_float(item.get("x"), label="x")
        y = finite_float(item.get("y"), label="y")
        z = finite_float(item.get("z", 0.0), label="z")
    elif isinstance(item, (list, tuple)) and len(item) >= 2:
        frame_id = planning_frame_id
        x = finite_float(item[0], label="x")
        y = finite_float(item[1], label="y")
        z = finite_float(item[2] if len(item) > 2 else 0.0, label="z")
    else:
        raise TypeError("waypoint must be an object or [x, y, z]")
    if frame_id != planning_frame_id:
        raise ValueError(f"unsupported waypoint frame {frame_id!r}; expected {planning_frame_id!r}")
    return {"x": x, "y": y, "z": z, "frame_id": frame_id, "loop": loop}


def normalize_patrol_waypoints(
    raw_waypoints: Any,
    *,
    planning_frame_id: str,
    loop: bool,
) -> list[dict[str, Any]]:
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("waypoints must be a non-empty list")
    return [
        normalize_patrol_waypoint(
            waypoint,
            planning_frame_id=planning_frame_id,
            loop=loop,
        )
        for waypoint in raw_waypoints
    ]


INSPECTION_LIFECYCLE_ACTIONS: dict[str, tuple[str, str]] = {
    "inspection_pause": ("pause_inspection", "operator_pause"),
    "pause_inspection": ("pause_inspection", "operator_pause"),
    "inspection_resume": ("resume_inspection", "operator_resume"),
    "resume_inspection": ("resume_inspection", "operator_resume"),
    "inspection_cancel": ("cancel_inspection", "operator_cancel"),
    "cancel_inspection": ("cancel_inspection", "operator_cancel"),
}


@register(
    "nav_service",
    "goal",
    description="Map-frame navigation goal entry",
)
class GoalService(Module, layer=6):
    """Normalize goal commands into Navigation inputs.

    Accepted commands:
      {"action": "goto", "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0}
      {"action": "building_navigate", "target": {"place_id": "company-6f"}}
      {"action": "inspection", "route_id": "daily_route", "revision": 3}
      {"action": "cancel", "reason": "user"}
    """

    runtime_id = "nav.goals"

    goal_command: In[str]
    goal_request: In[PoseStamped]
    cancel_request: In[str]
    goal_pose: Out[PoseStamped]
    patrol_goals: Out[list]
    cancel: Out[str]
    goal_status: Out[dict]

    def __init__(
        self,
        planning_frame_id: str | None = None,
        command_module: str | None = None,
        building_module: str | None = None,
        **config: Any,
    ) -> None:
        super().__init__(**config)
        self._planning_frame_id = normalize_frame_id(planning_frame_id) or map_frame_id()
        self._command_module = str(command_module or "").strip()
        self._building_module = str(building_module or "").strip()
        self._commands: Any | None = None
        self._building: Any | None = None
        self._request_sequence = itertools.count(1)

    def on_system_modules(self, modules: dict[str, Module]) -> None:
        if self._command_module:
            self._commands = modules.get(self._command_module)
        if self._building_module:
            self._building = modules.get(self._building_module)

    def setup(self) -> None:
        self.goal_command.subscribe(self._on_command)
        self.goal_request.subscribe(self._on_goal_request)
        self.cancel_request.subscribe(self._on_cancel_request)

    def _on_goal_request(self, goal: PoseStamped) -> None:
        self.submit_goal(goal)

    def submit_goal(
        self,
        goal: PoseStamped,
        *,
        task_id: str | None = None,
        request_id: str | None = None,
        action: str = "goal_pose",
    ) -> dict[str, Any]:
        """Validate and synchronously admit one typed goal."""

        resolved_task_id = str(task_id or "").strip() or self._new_task_id()
        resolved_request_id = str(request_id or "").strip() or self._new_request_id()
        if resolved_task_id == resolved_request_id:
            return self._publish_status(
                action,
                False,
                "task_id and request_id must be distinct",
                task_id=resolved_task_id,
                request_id=resolved_request_id,
            )
        if not isinstance(goal, PoseStamped):
            return self._publish_status(
                action,
                False,
                "goal_request must be PoseStamped",
                task_id=resolved_task_id,
                request_id=resolved_request_id,
            )
        frame_id = normalize_frame_id(getattr(goal, "frame_id", None))
        frame_id = frame_id or self._planning_frame_id
        try:
            normalized_goal = build_goal_pose(
                x=goal.pose.position.x,
                y=goal.pose.position.y,
                z=goal.pose.position.z,
                yaw=goal.yaw,
                frame_id=frame_id,
                planning_frame_id=self._planning_frame_id,
            )
        except (AttributeError, TypeError, ValueError) as exc:
            return self._publish_status(
                action,
                False,
                str(exc),
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                frame_id=frame_id,
            )

        normalized_goal = PoseStamped(
            pose=normalized_goal.pose,
            ts=getattr(goal, "ts", 0.0),
            frame_id=frame_id,
        )
        dispatch_result = self._dispatch_goal(
            normalized_goal,
            task_id=resolved_task_id,
            request_id=resolved_request_id,
        )
        dispatch_error = dispatch_result if isinstance(dispatch_result, str) else None
        if dispatch_error:
            return self._publish_status(
                action,
                False,
                dispatch_error,
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                frame_id=normalized_goal.frame_id,
                sink=self._sink_name,
            )
        receipt_fields = self._receipt_status_fields(dispatch_result)
        if isinstance(dispatch_result, NavigationCommandReceipt) and not dispatch_result.accepted:
            return self._publish_status(
                action,
                False,
                f"goal rejected by native endpoint: {dispatch_result.reason or 'rejected'}",
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                frame_id=frame_id,
                sink=self._sink_name,
                **receipt_fields,
            )
        return self._publish_status(
            action,
            True,
            "goal accepted by native endpoint" if receipt_fields else "goal published",
            task_id=resolved_task_id,
            request_id=resolved_request_id,
            frame_id=frame_id,
            sink=self._sink_name,
            **receipt_fields,
            target={
                "x": normalized_goal.x,
                "y": normalized_goal.y,
                "z": normalized_goal.z,
                "yaw": normalized_goal.yaw,
            },
        )

    def _on_cancel_request(self, reason: str) -> None:
        self.submit_cancel(
            str(reason or "cancel"),
        )

    def submit_cancel(
        self,
        reason: str = "cancel",
        *,
        task_id: str | None = None,
        request_id: str | None = None,
        action: str = "cancel",
    ) -> dict[str, Any]:
        """Synchronously admit one cancellation request."""

        resolved_task_id = str(task_id or "").strip()
        resolved_request_id = str(request_id or "").strip() or self._new_request_id()
        if self._command_module:
            if not resolved_task_id:
                return self._publish_status(
                    action,
                    False,
                    "task_id is required to cancel a native navigation task",
                    task_id=resolved_task_id,
                    request_id=resolved_request_id,
                    reason=reason,
                    sink=self._sink_name,
                )
            if resolved_task_id == resolved_request_id:
                return self._publish_status(
                    action,
                    False,
                    "task_id and request_id must be distinct",
                    task_id=resolved_task_id,
                    request_id=resolved_request_id,
                    reason=reason,
                    sink=self._sink_name,
                )
            try:
                result = self._call_commands(
                    "cancel_task",
                    reason=reason,
                    task_id=resolved_task_id,
                    request_id=resolved_request_id,
                )
            except RuntimeError as exc:
                return self._publish_status(
                    action,
                    False,
                    str(exc),
                    task_id=resolved_task_id,
                    request_id=resolved_request_id,
                    reason=reason,
                    sink=self._sink_name,
                )
        else:
            result = None
            self.cancel.publish(reason)
        receipt_fields = self._receipt_status_fields(result)
        if isinstance(result, NavigationCommandReceipt) and not result.accepted:
            return self._publish_status(
                action,
                False,
                f"cancel rejected by native endpoint: {result.reason or 'rejected'}",
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                reason=reason,
                sink=self._sink_name,
                **receipt_fields,
            )
        return self._publish_status(
            action,
            True,
            "cancel accepted by native endpoint" if receipt_fields else "cancel published",
            task_id=resolved_task_id,
            request_id=resolved_request_id,
            reason=reason,
            state="cancel_requested",
            sink=self._sink_name,
            **receipt_fields,
        )

    def _on_command(self, raw: str) -> None:
        try:
            cmd = json.loads(raw) if isinstance(raw, str) else raw
        except (TypeError, json.JSONDecodeError):
            self._publish_status("unknown", False, "invalid JSON command")
            return
        if not isinstance(cmd, dict):
            self._publish_status("unknown", False, "command must be an object")
            return

        request_id = self._request_id_from(cmd)
        action = str(cmd.get("action") or cmd.get("type") or "").strip().lower()
        if action in {"goto", "go", "goal", "navigate", "target"}:
            task_id = self._task_id_from(cmd) or self._new_task_id()
            self._publish_goal(cmd, action=action, task_id=task_id, request_id=request_id)
        elif action in {"building_navigate", "building"}:
            self._dispatch_building(cmd, action=action, request_id=request_id)
        elif action in {"inspection", "patrol", "route"}:
            self._publish_patrol(cmd, action=action, request_id=request_id)
        elif action in INSPECTION_LIFECYCLE_ACTIONS:
            self._dispatch_inspection_lifecycle(cmd, action=action, request_id=request_id)
        elif action in {"cancel", "stop"}:
            reason = str(cmd.get("reason") or action)
            task_id = self._task_id_from(cmd)
            self.submit_cancel(reason, task_id=task_id, request_id=request_id, action=action)
        else:
            self._publish_status(
                action or "unknown",
                False,
                f"unknown action: {action}",
                request_id=request_id,
            )

    def _dispatch_building(
        self,
        cmd: dict[str, Any],
        *,
        action: str,
        request_id: str,
    ) -> None:
        command = dict(cmd)
        command["action"] = "building_navigate"
        command["request_id"] = request_id
        if not self._building_module:
            self._publish_status(
                action,
                False,
                "building navigation capability is not configured",
                request_id=request_id,
                reason="building_module_not_configured",
                sink="building",
            )
            return
        if self._building is None:
            self._publish_status(
                action,
                False,
                f"building navigation capability {self._building_module!r} is unavailable",
                request_id=request_id,
                reason="building_module_unavailable",
                sink=self._building_module,
            )
            return
        submit = getattr(self._building, "submit", None)
        if not callable(submit):
            self._publish_status(
                action,
                False,
                "building navigation capability does not implement submit",
                request_id=request_id,
                reason="building_submit_unavailable",
                sink=self._building_module,
            )
            return
        try:
            result = submit(command)
        except Exception as exc:
            self._publish_status(
                action,
                False,
                str(exc) or "building navigation dispatch failed",
                request_id=request_id,
                reason="building_dispatch_error",
                sink=self._building_module,
            )
            return
        if not isinstance(result, dict):
            self._publish_status(
                action,
                False,
                "building navigation submit must return an object",
                request_id=request_id,
                reason="invalid_building_response",
                sink=self._building_module,
            )
            return

        accepted = bool(result.get("accepted", result.get("success", False)))
        success = bool(result.get("success", accepted))
        reason = str(result.get("reason") or "").strip()
        message = str(result.get("message") or reason).strip()
        if not message:
            message = "building navigation accepted" if accepted else "building navigation rejected"
        self._publish_status(
            action,
            success,
            message,
            request_id=request_id,
            accepted=accepted,
            state="accepted" if accepted else "rejected",
            reason=reason,
            sink=self._building_module,
        )

    def _publish_goal(
        self,
        cmd: dict[str, Any],
        *,
        action: str,
        task_id: str,
        request_id: str,
    ) -> None:
        try:
            target = self._target_mapping(cmd)
        except TypeError as exc:
            self._publish_status(
                action,
                False,
                str(exc),
                task_id=task_id,
                request_id=request_id,
            )
            return
        frame_id = self._frame_id(target, cmd)
        try:
            goal = build_goal_pose(
                x=target.get("x"),
                y=target.get("y"),
                z=target.get("z", 0.0),
                yaw=target.get("yaw", cmd.get("yaw", 0.0)),
                frame_id=frame_id,
                planning_frame_id=self._planning_frame_id,
            )
        except (TypeError, ValueError) as exc:
            self._publish_status(
                action,
                False,
                str(exc),
                task_id=task_id,
                request_id=request_id,
                frame_id=frame_id,
            )
            return

        self.submit_goal(
            goal,
            task_id=task_id,
            request_id=request_id,
            action=action,
        )

    def _dispatch_goal(
        self,
        goal: PoseStamped,
        *,
        task_id: str,
        request_id: str,
    ) -> NavigationCommandReceipt | str | None:
        if not self._command_module:
            self.goal_pose.publish(goal)
            return None
        try:
            return self._call_commands(
                "send_goal",
                x=goal.x,
                y=goal.y,
                z=goal.z,
                yaw=goal.yaw,
                task_id=task_id,
                request_id=request_id,
            )
        except RuntimeError as exc:
            return str(exc)

    def _publish_patrol(
        self,
        cmd: dict[str, Any],
        *,
        action: str,
        request_id: str,
    ) -> None:
        if self._command_module:
            self._dispatch_inspection(cmd, action=action, request_id=request_id)
            return

        raw_waypoints = cmd.get("waypoints") or cmd.get("goals") or []
        loop = bool(cmd.get("loop", False))
        try:
            waypoints = normalize_patrol_waypoints(
                raw_waypoints,
                planning_frame_id=self._planning_frame_id,
                loop=loop,
            )
        except (TypeError, ValueError) as exc:
            self._publish_status(action, False, str(exc), request_id=request_id)
            return

        self.patrol_goals.publish(waypoints)
        self._publish_status(
            action,
            True,
            "patrol goals published",
            request_id=request_id,
            frame_id=self._planning_frame_id,
            waypoint_count=len(waypoints),
            loop=loop,
        )

    def _dispatch_inspection(
        self,
        cmd: dict[str, Any],
        *,
        action: str,
        request_id: str,
    ) -> None:
        route_id = str(cmd.get("route_id") or cmd.get("route") or cmd.get("name") or cmd.get("id") or "").strip()
        if not route_id:
            self._publish_status(
                action,
                False,
                "inspection route_id is required; inline patrol waypoints are retired",
                request_id=request_id,
                sink="native_dds",
            )
            return
        try:
            revision = self._route_revision(
                cmd.get("route_revision", cmd.get("revision", 0)) or 0,
            )
        except (TypeError, ValueError) as exc:
            self._publish_status(
                action,
                False,
                str(exc),
                request_id=request_id,
                sink="native_dds",
            )
            return

        try:
            self._call_commands(
                "start_inspection",
                route_id=route_id,
                revision=revision,
                request_id=request_id,
            )
        except RuntimeError as exc:
            self._publish_status(
                action,
                False,
                str(exc),
                request_id=request_id,
                route_id=route_id,
                route_revision=revision,
                sink="native_dds",
            )
            return
        self._publish_status(
            action,
            True,
            "inspection command dispatched",
            request_id=request_id,
            route_id=route_id,
            route_revision=revision,
            sink="native_dds",
        )

    def _dispatch_inspection_lifecycle(
        self,
        cmd: dict[str, Any],
        *,
        action: str,
        request_id: str,
    ) -> None:
        method, default_reason = INSPECTION_LIFECYCLE_ACTIONS[action]
        reason = str(cmd.get("reason") or default_reason)
        if not self._command_module:
            self._publish_status(
                action,
                False,
                "inspection lifecycle command requires native command capability",
                request_id=request_id,
                reason=reason,
                sink="native_dds",
            )
            return
        try:
            self._call_commands(
                method,
                reason=reason,
                request_id=request_id,
            )
        except RuntimeError as exc:
            self._publish_status(
                action,
                False,
                str(exc),
                request_id=request_id,
                reason=reason,
                sink="native_dds",
            )
            return
        self._publish_status(
            action,
            True,
            f"{action} command dispatched",
            request_id=request_id,
            reason=reason,
            sink="native_dds",
        )

    def _publish_status(
        self,
        action: str,
        success: bool,
        message: str,
        request_id: str | None = None,
        **extra: Any,
    ) -> dict[str, Any]:
        payload = {
            "request_id": request_id or self._new_request_id(),
            "action": action,
            "success": success,
            "accepted": success,
            "state": "accepted" if success else "rejected",
            "message": message,
        }
        payload.update(extra)
        self.goal_status.publish(payload)
        return payload

    def _new_request_id(self) -> str:
        return f"nav-{time.time_ns()}-{next(self._request_sequence)}"

    def _new_task_id(self) -> str:
        return f"nav-task-{uuid.uuid4().hex}"

    def _request_id_from(self, command: dict[str, Any]) -> str:
        request_id = str(command.get("request_id") or "").strip()
        return request_id or self._new_request_id()

    def _task_id_from(self, command: dict[str, Any]) -> str:
        task_id = str(command.get("task_id") or "").strip()
        return task_id

    @property
    def _sink_name(self) -> str:
        return "native_dds" if self._command_module else "module"

    def _call_commands(self, method: str, **kwargs: Any) -> Any:
        if self._commands is None:
            raise RuntimeError(f"navigation command capability {self._command_module!r} is unavailable")
        operation = getattr(self._commands, method, None)
        if not callable(operation):
            raise RuntimeError(f"navigation command capability does not implement {method}")
        try:
            result = operation(**kwargs)
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError(str(exc)) from exc
        if method in {"send_goal", "cancel_task"}:
            if not isinstance(result, NavigationCommandReceipt):
                raise RuntimeError(
                    f"navigation command {method} returned an invalid task receipt"
                )
            expected_kind = (
                NavigationCommandKind.GOAL
                if method == "send_goal"
                else NavigationCommandKind.CANCEL
            )
            if result.kind != int(expected_kind):
                raise RuntimeError(
                    f"navigation command {method} returned the wrong command kind"
                )
            expected_task_id = str(kwargs.get("task_id") or "").strip()
            if result.task_id != expected_task_id:
                raise RuntimeError(
                    f"navigation command {method} returned the wrong task_id"
                )
            expected_request_id = str(kwargs.get("request_id") or "").strip()
            if expected_request_id and not (
                result.request_id == expected_request_id
                or result.request_id.startswith(f"{expected_request_id}-clock-retry-")
            ):
                raise RuntimeError(
                    f"navigation command {method} returned the wrong request_id"
                )
        return result

    @staticmethod
    def _receipt_status_fields(result: Any) -> dict[str, Any]:
        if not isinstance(result, NavigationCommandReceipt):
            return {}
        return {
            "native_request_id": result.request_id,
            "native_ack": result.to_dict(),
        }

    @staticmethod
    def _route_revision(value: Any) -> int:
        try:
            revision = int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("inspection route revision must be an integer") from exc
        if not 0 <= revision <= (1 << 64) - 1:
            raise ValueError("inspection route revision must be between 0 and UINT64_MAX")
        return revision

    @staticmethod
    def _target_mapping(cmd: dict[str, Any]) -> dict[str, Any]:
        target = cmd.get("target") or cmd.get("goal") or cmd
        if not isinstance(target, dict):
            raise TypeError("target must be an object")
        return target

    def _frame_id(self, *sources: dict[str, Any]) -> str:
        return frame_id_from_sources(
            *sources,
            default_frame_id=self._planning_frame_id,
        )
