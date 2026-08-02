"""Goal service for navigation, building, patrol, and cancel commands."""

from __future__ import annotations

import itertools
import json
import logging
import math
import time
import uuid
from collections.abc import Mapping
from typing import Any

from nav.services.task_ledger import NavigationTaskLedger, TaskLedgerConflict
from runtime import In, Module, Out
from runtime.msgs import (
    NavigationCommandKind,
    NavigationCommandReceipt,
    NavigationGoalStatus,
)
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.registry import register
from runtime.runtime_interface import map_frame_id, normalize_frame_id

logger = logging.getLogger(__name__)


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
    "inspection_pause": ("pause_inspection_task", "operator_pause"),
    "inspection_resume": ("resume_inspection_task", "operator_resume"),
    "inspection_cancel": ("cancel_inspection_task", "operator_cancel"),
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
    navigation_goal_status: In[NavigationGoalStatus]
    goal_pose: Out[PoseStamped]
    patrol_goals: Out[list]
    cancel: Out[str]
    goal_status: Out[dict]

    def __init__(
        self,
        planning_frame_id: str | None = None,
        command_module: str | None = None,
        task_ledger_path: str | None = None,
        run_plan_fingerprint: str = "",
        map_identity: Mapping[str, Any] | None = None,
        building_module: str | None = None,
        **config: Any,
    ) -> None:
        retired_names = (
            "_".join(("product", "fingerprint")),
            "_".join(("runtime", "manifest", "fingerprint")),
        )
        for retired_name in retired_names:
            if retired_name in config:
                raise TypeError(f"{retired_name} was retired; use run_plan_fingerprint")
        super().__init__(**config)
        self._planning_frame_id = normalize_frame_id(planning_frame_id) or map_frame_id()
        self._command_module = str(command_module or "").strip()
        self._building_module = str(building_module or "").strip()
        self._commands: Any | None = None
        self._building: Any | None = None
        self._request_sequence = itertools.count(1)
        self._run_plan_fingerprint = str(run_plan_fingerprint or "").strip()
        self._map_identity = dict(map_identity) if map_identity is not None else None
        self._task_ledger = NavigationTaskLedger(":memory:" if task_ledger_path is None else task_ledger_path)

    def on_system_modules(self, modules: dict[str, Module]) -> None:
        if self._command_module:
            self._commands = modules.get(self._command_module)
        if self._building_module:
            self._building = modules.get(self._building_module)

    def setup(self) -> None:
        self.goal_command.subscribe(self._on_command)
        self.goal_request.subscribe(self._on_goal_request)
        self.cancel_request.subscribe(self._on_cancel_request)
        # Lifecycle transitions are low-rate and order-sensitive; do not coalesce them.
        self.navigation_goal_status.subscribe(self._on_navigation_goal_status)

        self._reconcile_task_history()

    def stop(self) -> None:
        """Close the task ledger and then release Module resources."""

        try:
            self._task_ledger.close()
        finally:
            super().stop()

    def _on_navigation_goal_status(self, status: NavigationGoalStatus) -> None:
        """Persist one native lifecycle event in source delivery order."""

        try:
            self._task_ledger.record_goal_status(status)
        except KeyError:
            logger.debug(
                "Ignoring native goal status for untracked task %s",
                getattr(status, "task_id", ""),
            )
        except (TaskLedgerConflict, TypeError, ValueError) as exc:
            logger.warning(
                "Rejecting invalid native goal status for task %s: %s",
                getattr(status, "task_id", ""),
                exc,
            )

    def _on_goal_request(self, goal: PoseStamped) -> None:
        self.submit_goal(goal)

    def lookup_goal_replay(
        self,
        goal: PoseStamped,
        *,
        task_id: str | None,
        request_id: str | None,
        action: str = "goal_pose",
    ) -> dict[str, Any] | None:
        """Return a prior goal admission without dispatching any motion."""

        resolved_task_id = str(task_id or "").strip()
        resolved_request_id = str(request_id or "").strip()
        if not resolved_task_id or not resolved_request_id:
            return None
        if resolved_task_id == resolved_request_id:
            return self._build_status_payload(
                action,
                False,
                "task_id and request_id must be distinct",
                request_id=resolved_request_id,
                task_id=resolved_task_id,
            )
        if not isinstance(goal, PoseStamped):
            return self._build_status_payload(
                action,
                False,
                "goal_request must be PoseStamped",
                request_id=resolved_request_id,
                task_id=resolved_task_id,
            )

        frame_id = normalize_frame_id(getattr(goal, "frame_id", None))
        frame_id = frame_id or self._planning_frame_id
        try:
            _, target = self._normalize_goal(goal, frame_id=frame_id)
        except (AttributeError, TypeError, ValueError) as exc:
            return self._build_status_payload(
                action,
                False,
                str(exc),
                request_id=resolved_request_id,
                task_id=resolved_task_id,
                frame_id=frame_id,
            )

        try:
            admission = self._task_ledger.lookup_admission(
                resolved_task_id,
                resolved_request_id,
                "goal",
                target,
                target=target,
                run_plan_fingerprint=self._run_plan_fingerprint,
                map_identity=self._map_identity,
            )
        except TaskLedgerConflict as exc:
            return self._build_status_payload(
                action,
                False,
                f"task admission conflict: {exc}",
                request_id=resolved_request_id,
                task_id=resolved_task_id,
                frame_id=frame_id,
                sink=self._sink_name,
            )
        except Exception:
            return self._build_status_payload(
                action,
                False,
                "task history unavailable; replay lookup rejected and command was not dispatched",
                request_id=resolved_request_id,
                task_id=resolved_task_id,
                frame_id=frame_id,
                reason="task_history_unavailable",
                sink=self._sink_name,
            )
        if admission is None:
            return None
        return self._build_replay_status_payload(
            admission["record"],
            action=action,
            request_id=resolved_request_id,
        )

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
            normalized_goal, target = self._normalize_goal(goal, frame_id=frame_id)
        except (AttributeError, TypeError, ValueError) as exc:
            return self._publish_status(
                action,
                False,
                str(exc),
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                frame_id=frame_id,
            )
        try:
            admission = self._task_ledger.admit(
                resolved_task_id,
                resolved_request_id,
                "goal",
                target,
                source=action,
                target=target,
                run_plan_fingerprint=self._run_plan_fingerprint,
                map_identity=self._map_identity,
            )
        except TaskLedgerConflict as exc:
            return self._publish_status(
                action,
                False,
                f"task admission conflict: {exc}",
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                frame_id=frame_id,
                sink=self._sink_name,
            )
        except Exception:
            return self._publish_status(
                action,
                False,
                "task history unavailable; command was not dispatched",
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                frame_id=frame_id,
                sink=self._sink_name,
            )
        if admission["replay"]:
            return self._publish_replay_status(
                admission["record"],
                action=action,
                request_id=resolved_request_id,
            )
        dispatch_result = self._dispatch_goal(
            normalized_goal,
            task_id=resolved_task_id,
            request_id=resolved_request_id,
        )
        if isinstance(dispatch_result, str):
            dispatch_error = dispatch_result or "navigation command failed without error detail"
            return self._publish_status(
                action,
                False,
                f"goal admission outcome unconfirmed: {dispatch_error}; do not resend with a new request_id",
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                frame_id=normalized_goal.frame_id,
                state="unknown",
                admission_confirmed=False,
                admission_unconfirmed=True,
                history_recorded=True,
                sink=self._sink_name,
            )

        history_fields = self._record_dispatch_history(
            dispatch_result,
            task_id=resolved_task_id,
            request_id=resolved_request_id,
            local_reason="goal_published",
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
                **history_fields,
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
            **history_fields,
            target={
                "x": normalized_goal.x,
                "y": normalized_goal.y,
                "z": normalized_goal.z,
                "yaw": normalized_goal.yaw,
            },
        )

    def _normalize_goal(
        self,
        goal: PoseStamped,
        *,
        frame_id: str,
    ) -> tuple[PoseStamped, dict[str, Any]]:
        normalized_goal = build_goal_pose(
            x=goal.pose.position.x,
            y=goal.pose.position.y,
            z=goal.pose.position.z,
            yaw=goal.yaw,
            frame_id=frame_id,
            planning_frame_id=self._planning_frame_id,
        )
        normalized_goal = PoseStamped(
            pose=normalized_goal.pose,
            ts=getattr(goal, "ts", 0.0),
            frame_id=frame_id,
        )
        return normalized_goal, {
            "x": normalized_goal.x,
            "y": normalized_goal.y,
            "z": normalized_goal.z,
            "yaw": normalized_goal.yaw,
            "frame_id": normalized_goal.frame_id,
        }

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

        resolved_reason = str(reason or "cancel")
        resolved_task_id = str(task_id or "").strip()
        resolved_request_id = str(request_id or "").strip() or self._new_request_id()
        if self._command_module and not resolved_task_id:
            return self._publish_status(
                action,
                False,
                "task_id is required to cancel a native navigation task",
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                reason=resolved_reason,
                sink=self._sink_name,
            )

        tracks_history = bool(resolved_task_id)
        if tracks_history:
            if resolved_task_id == resolved_request_id:
                return self._publish_status(
                    action,
                    False,
                    "task_id and request_id must be distinct",
                    task_id=resolved_task_id,
                    request_id=resolved_request_id,
                    reason=resolved_reason,
                    sink=self._sink_name,
                )
            try:
                admission = self._task_ledger.admit(
                    resolved_task_id,
                    resolved_request_id,
                    "cancel",
                    {"reason": resolved_reason},
                    source=action,
                    run_plan_fingerprint=self._run_plan_fingerprint,
                    map_identity=self._map_identity,
                )
            except TaskLedgerConflict as exc:
                return self._publish_status(
                    action,
                    False,
                    f"task admission conflict: {exc}",
                    task_id=resolved_task_id,
                    request_id=resolved_request_id,
                    reason=resolved_reason,
                    sink=self._sink_name,
                )
            except Exception:
                return self._publish_status(
                    action,
                    False,
                    "task history unavailable; command was not dispatched",
                    task_id=resolved_task_id,
                    request_id=resolved_request_id,
                    reason=resolved_reason,
                    sink=self._sink_name,
                )
            if admission["replay"]:
                return self._publish_replay_status(
                    admission["record"],
                    action=action,
                    request_id=resolved_request_id,
                )

        try:
            if self._command_module:
                result = self._call_commands(
                    "cancel_task",
                    reason=resolved_reason,
                    task_id=resolved_task_id,
                    request_id=resolved_request_id,
                )
            else:
                result = None
                self.cancel.publish(resolved_reason)
        except Exception as exc:
            return self._publish_status(
                action,
                False,
                f"cancel admission outcome unconfirmed: {exc}; do not resend with a new request_id",
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                reason=resolved_reason,
                state="unknown",
                admission_confirmed=False,
                admission_unconfirmed=True,
                history_recorded=tracks_history,
                sink=self._sink_name,
            )

        if tracks_history:
            history_fields = self._record_dispatch_history(
                result,
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                local_reason="cancel_requested",
            )
        else:
            history_fields = {
                "history_recorded": False,
                "history_warning": "cancel has no task_id and is not in task history",
            }

        receipt_fields = self._receipt_status_fields(result)
        if isinstance(result, NavigationCommandReceipt) and not result.accepted:
            return self._publish_status(
                action,
                False,
                f"cancel rejected by native endpoint: {result.reason or 'rejected'}",
                task_id=resolved_task_id,
                request_id=resolved_request_id,
                reason=resolved_reason,
                sink=self._sink_name,
                **receipt_fields,
                **history_fields,
            )
        return self._publish_status(
            action,
            True,
            "cancel accepted by native endpoint" if receipt_fields else "cancel published",
            task_id=resolved_task_id,
            request_id=resolved_request_id,
            reason=resolved_reason,
            state="cancel_requested",
            sink=self._sink_name,
            **receipt_fields,
            **history_fields,
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
            task_id = self._task_id_from(cmd) or self._new_task_id()
            self._publish_patrol(
                cmd,
                action=action,
                task_id=task_id,
                request_id=request_id,
            )
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
                reason="invalid_building_ack",
                sink=self._building_module,
            )
            return

        accepted = result.get("accepted")
        success = result.get("success")
        if not isinstance(accepted, bool) or not isinstance(success, bool):
            self._publish_status(
                action,
                False,
                "building navigation submit returned an invalid acknowledgement",
                request_id=request_id,
                reason="invalid_building_ack",
                sink=self._building_module,
            )
            return
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
        try:
            if not self._command_module:
                self.goal_pose.publish(goal)
                return None
            return self._call_commands(
                "send_goal",
                x=goal.x,
                y=goal.y,
                z=goal.z,
                yaw=goal.yaw,
                task_id=task_id,
                request_id=request_id,
            )
        except Exception as exc:
            return str(exc)

    def get_task(self, task_id: str) -> dict[str, Any] | None:
        """Return one durable task record without polling or mutating history."""

        resolved_task_id = str(task_id or "").strip()
        if not resolved_task_id:
            raise ValueError("task_id is required")
        return self._task_ledger.get_task(resolved_task_id)

    def list_tasks(
        self,
        *,
        limit: int = 50,
        active_only: bool = False,
    ) -> list[dict[str, Any]]:
        """Return stored task history without polling or mutating native state."""

        if isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0:
            raise ValueError("limit must be a positive integer")
        if limit > 1000:
            raise ValueError("limit cannot exceed 1000")
        if active_only:
            return self._task_ledger.list_open()[:limit]
        return self._task_ledger.list_recent(limit)

    def _reconcile_task_history(self) -> None:
        """Apply retained native evidence without ever replaying a command."""

        if not self._command_module:
            return
        open_tasks = self._task_ledger.list_open()
        if not open_tasks:
            return

        try:
            navigation_state = self._call_commands("read_navigation_state")
        except RuntimeError:
            navigation_state = None

        retained_statuses: list[dict[str, Any]] = []
        retained_available = True
        for task in open_tasks:
            try:
                retained = self._call_commands(
                    "get_navigation_task_status",
                    task_id=task["task_id"],
                )
            except RuntimeError:
                retained_available = False
                retained_statuses = []
                break
            if retained is not None:
                retained_statuses.append(dict(retained))

        try:
            self._task_ledger.reconcile_endpoint(
                navigation_state,
                goal_statuses=retained_statuses if retained_available else None,
            )
        except (KeyError, TaskLedgerConflict, TypeError, ValueError):
            # Bad or conflicting evidence must not create a terminal state.
            return

    def _publish_replay_status(
        self,
        record: Mapping[str, Any],
        *,
        action: str,
        request_id: str,
    ) -> dict[str, Any]:
        payload = self._build_replay_status_payload(
            record,
            action=action,
            request_id=request_id,
        )
        self.goal_status.publish(payload)
        return payload

    def _build_replay_status_payload(
        self,
        record: Mapping[str, Any],
        *,
        action: str,
        request_id: str,
    ) -> dict[str, Any]:
        attempt = next(
            (
                item
                for item in record.get("attempts", [])
                if isinstance(item, Mapping) and item.get("request_id") == request_id
            ),
            None,
        )
        attempt = attempt if isinstance(attempt, Mapping) else None
        accepted = attempt.get("accepted") if attempt is not None else None
        kind = str(attempt.get("kind") or "") if attempt is not None else ""
        if accepted is True:
            state = "cancel_requested" if kind == "cancel" else "accepted"
            message = f"{kind or 'navigation'} request was already accepted"
        elif accepted is False:
            state = "rejected"
            message = f"{kind or 'navigation'} request was already rejected"
        else:
            state = "unknown"
            message = "request was already admitted; admission outcome remains unconfirmed"

        extra: dict[str, Any] = {
            "task_id": str(record.get("task_id") or ""),
            "state": state,
            "task_state": str(record.get("execution_state") or "unknown"),
            "admission": str(record.get("admission") or "unconfirmed"),
            "reason": (
                str(attempt.get("reason") or "") if attempt is not None else str(record.get("reason") or "")
            ),
            "replay": True,
            "admission_confirmed": accepted is not None,
            "admission_unconfirmed": accepted is None,
            "history_recorded": True,
            "sink": self._sink_name,
        }
        if attempt is not None and attempt.get("native_ack") is not None:
            extra["native_request_id"] = str(attempt.get("native_request_id") or request_id)
            extra["native_ack"] = attempt["native_ack"]
        return self._build_status_payload(
            action,
            accepted is True,
            message,
            request_id=request_id,
            **extra,
        )

    def _record_dispatch_history(
        self,
        result: NavigationCommandReceipt | None,
        *,
        task_id: str,
        request_id: str,
        local_reason: str,
    ) -> dict[str, Any]:
        try:
            if isinstance(result, NavigationCommandReceipt):
                self._task_ledger.record_native_ack(result)
            else:
                self._task_ledger.record_admission_result(
                    task_id,
                    request_id,
                    accepted=True,
                    reason=local_reason,
                )
        except Exception:
            return {
                "history_recorded": False,
                "history_warning": ("command was dispatched, but durable task history could not be updated"),
            }
        return {"history_recorded": True}

    def _publish_patrol(
        self,
        cmd: dict[str, Any],
        *,
        action: str,
        task_id: str,
        request_id: str,
    ) -> None:
        if self._command_module:
            self._dispatch_inspection(
                cmd,
                action=action,
                task_id=task_id,
                request_id=request_id,
            )
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
            task_id=task_id,
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
        task_id: str,
        request_id: str,
    ) -> None:
        route_id = str(cmd.get("route_id") or cmd.get("route") or cmd.get("name") or cmd.get("id") or "").strip()
        if not route_id:
            self._publish_status(
                action,
                False,
                "inspection route_id is required; inline patrol waypoints are retired",
                task_id=task_id,
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
                task_id=task_id,
                request_id=request_id,
                sink="native_dds",
            )
            return

        try:
            accepted = self._call_commands(
                "start_inspection_task",
                task_id=task_id,
                route_id=route_id,
                revision=revision,
                request_id=request_id,
            )
        except RuntimeError as exc:
            self._publish_status(
                action,
                False,
                str(exc),
                task_id=task_id,
                request_id=request_id,
                route_id=route_id,
                route_revision=revision,
                sink="native_dds",
            )
            return
        if accepted is not True:
            self._publish_status(
                action,
                False,
                "inspection task was rejected by the native endpoint",
                task_id=task_id,
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
            task_id=task_id,
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
        task_id = self._task_id_from(cmd)
        if not task_id:
            self._publish_status(
                action,
                False,
                "inspection task_id is required",
                task_id="",
                request_id=request_id,
                reason=reason,
                sink="native_dds",
            )
            return
        if not self._command_module:
            self._publish_status(
                action,
                False,
                "inspection lifecycle command requires native command capability",
                task_id=task_id,
                request_id=request_id,
                reason=reason,
                sink="native_dds",
            )
            return
        try:
            accepted = self._call_commands(
                method,
                task_id=task_id,
                reason=reason,
                request_id=request_id,
            )
        except RuntimeError as exc:
            self._publish_status(
                action,
                False,
                str(exc),
                task_id=task_id,
                request_id=request_id,
                reason=reason,
                sink="native_dds",
            )
            return
        if accepted is not True:
            self._publish_status(
                action,
                False,
                f"{action} was rejected by the native endpoint",
                task_id=task_id,
                request_id=request_id,
                reason=reason,
                sink="native_dds",
            )
            return
        self._publish_status(
            action,
            True,
            f"{action} command dispatched",
            task_id=task_id,
            request_id=request_id,
            reason=reason,
            sink="native_dds",
        )

    @staticmethod
    def _build_status_payload(
        action: str,
        success: bool,
        message: str,
        request_id: str,
        **extra: Any,
    ) -> dict[str, Any]:
        payload = {
            "request_id": request_id,
            "action": action,
            "success": success,
            "accepted": success,
            "state": "accepted" if success else "rejected",
            "message": message,
        }
        payload.update(extra)
        return payload

    def _publish_status(
        self,
        action: str,
        success: bool,
        message: str,
        request_id: str | None = None,
        **extra: Any,
    ) -> dict[str, Any]:
        payload = self._build_status_payload(
            action,
            success,
            message,
            request_id or self._new_request_id(),
            **extra,
        )
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
                raise RuntimeError(f"navigation command {method} returned an invalid task receipt")
            expected_kind = (
                NavigationCommandKind.GOAL
                if method == "send_goal"
                else NavigationCommandKind.TASK_CANCEL
            )
            if result.kind != int(expected_kind):
                raise RuntimeError(f"navigation command {method} returned the wrong command kind")
            expected_task_id = str(kwargs.get("task_id") or "").strip()
            if result.task_id != expected_task_id:
                raise RuntimeError(f"navigation command {method} returned the wrong task_id")
            expected_request_id = str(kwargs.get("request_id") or "").strip()
            if expected_request_id and not (
                result.request_id == expected_request_id
                or result.request_id.startswith(f"{expected_request_id}-clock-retry-")
            ):
                raise RuntimeError(f"navigation command {method} returned the wrong request_id")
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
