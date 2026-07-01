"""Goal service for navigation goals, patrol routes, and cancel commands."""

from __future__ import annotations

import json
import math
from typing import Any

from runtime import In, Module, Out
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
        frame_id = normalize_frame_id(
            source.get("frame_id") if isinstance(source, dict) else None
        )
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
    target_frame = normalize_frame_id(frame_id) or (
        normalize_frame_id(planning_frame_id) or map_frame_id()
    )
    expected_frame = normalize_frame_id(planning_frame_id) or target_frame
    if target_frame != expected_frame:
        raise ValueError(
            f"unsupported target frame {target_frame!r}; expected {expected_frame!r}"
        )
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
        raise ValueError(
            f"unsupported waypoint frame {frame_id!r}; expected {planning_frame_id!r}"
        )
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


@register(
    "nav_service",
    "goal",
    description="Map-frame navigation goal entry",
)
class GoalService(Module, layer=6):
    """Normalize goal commands into Navigation inputs.

    Accepted commands:
      {"action": "goto", "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0}
      {"action": "patrol", "waypoints": [{"x": 1.0, "y": 2.0}], "loop": true}
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

    def __init__(self, planning_frame_id: str | None = None, **config: Any) -> None:
        super().__init__(**config)
        self._planning_frame_id = (
            normalize_frame_id(planning_frame_id) or map_frame_id()
        )

    def setup(self) -> None:
        self.goal_command.subscribe(self._on_command)
        self.goal_request.subscribe(self._on_goal_request)
        self.cancel_request.subscribe(self._on_cancel_request)

    def _on_goal_request(self, goal: PoseStamped) -> None:
        if not isinstance(goal, PoseStamped):
            self._publish_status(
                "goal_pose",
                False,
                "goal_request must be PoseStamped",
            )
            return
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
            self._publish_status(
                "goal_pose",
                False,
                str(exc),
                frame_id=frame_id,
            )
            return

        normalized_goal = PoseStamped(
            pose=normalized_goal.pose,
            ts=getattr(goal, "ts", 0.0),
            frame_id=frame_id,
        )
        self.goal_pose.publish(normalized_goal)
        self._publish_status(
            "goal_pose",
            True,
            "goal published",
            frame_id=frame_id,
            target={
                "x": normalized_goal.x,
                "y": normalized_goal.y,
                "z": normalized_goal.z,
                "yaw": normalized_goal.yaw,
            },
        )

    def _on_cancel_request(self, reason: str) -> None:
        reason = str(reason or "cancel")
        self.cancel.publish(reason)
        self._publish_status("cancel", True, "cancel published", reason=reason)

    def _on_command(self, raw: str) -> None:
        try:
            cmd = json.loads(raw) if isinstance(raw, str) else raw
        except (TypeError, json.JSONDecodeError):
            self._publish_status("unknown", False, "invalid JSON command")
            return
        if not isinstance(cmd, dict):
            self._publish_status("unknown", False, "command must be an object")
            return

        action = str(cmd.get("action") or cmd.get("type") or "").strip().lower()
        if action in {"goto", "go", "goal", "navigate", "target"}:
            self._publish_goal(cmd, action=action)
        elif action in {"patrol", "route"}:
            self._publish_patrol(cmd, action=action)
        elif action in {"cancel", "stop"}:
            reason = str(cmd.get("reason") or action)
            self.cancel.publish(reason)
            self._publish_status(action, True, "cancel published", reason=reason)
        else:
            self._publish_status(
                action or "unknown",
                False,
                f"unknown action: {action}",
            )

    def _publish_goal(self, cmd: dict[str, Any], *, action: str) -> None:
        try:
            target = self._target_mapping(cmd)
        except TypeError as exc:
            self._publish_status(action, False, str(exc))
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
            self._publish_status(action, False, str(exc), frame_id=frame_id)
            return

        self.goal_pose.publish(goal)
        self._publish_status(
            action,
            True,
            "goal published",
            frame_id=frame_id,
            target={"x": goal.x, "y": goal.y, "z": goal.z, "yaw": goal.yaw},
        )

    def _publish_patrol(self, cmd: dict[str, Any], *, action: str) -> None:
        raw_waypoints = cmd.get("waypoints") or cmd.get("goals") or []
        loop = bool(cmd.get("loop", False))
        try:
            waypoints = normalize_patrol_waypoints(
                raw_waypoints,
                planning_frame_id=self._planning_frame_id,
                loop=loop,
            )
        except (TypeError, ValueError) as exc:
            self._publish_status(action, False, str(exc))
            return

        self.patrol_goals.publish(waypoints)
        self._publish_status(
            action,
            True,
            "patrol goals published",
            frame_id=self._planning_frame_id,
            waypoint_count=len(waypoints),
            loop=loop,
        )

    def _publish_status(
        self,
        action: str,
        success: bool,
        message: str,
        **extra: Any,
    ) -> None:
        payload = {"action": action, "success": success, "message": message}
        payload.update(extra)
        self.goal_status.publish(payload)

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
