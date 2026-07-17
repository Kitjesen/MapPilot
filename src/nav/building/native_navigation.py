"""Correlated native endpoint observation for building mission steps."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import Any, Protocol

from nav.building.model import GoalProgress, PoseTarget


class NativeNavigationSnapshot(Protocol):
    """Structural view of one atomic native navigation snapshot."""

    map_id: str
    x: float
    y: float
    z: float
    yaw: float
    stamp_s: float
    native_request_id: str
    native_command_kind: str
    native_command_accepted: bool
    native_status_stamp_s: float
    native_control_mode: str
    native_estop_latched: bool
    native_operator_takeover_latched: bool
    native_plan_seen: bool
    native_plan_accepted: bool
    native_plan_reason: str
    native_plan_goal: tuple[float, float, float] | None
    native_goal_reached: bool


class NativeNavigationClientPort(Protocol):
    """Command-only view implemented by NativeNavigationClient."""

    def send_goal(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        request_id: str | None = None,
    ) -> None:
        """Send one correlated native map-frame goal."""

        ...

    def cancel(self, reason: str = "cancel", *, request_id: str | None = None) -> None:
        """Cancel an active native goal."""

        ...

    def stop(self, reason: str = "stop", *, request_id: str | None = None) -> None:
        """Clear native motion without latching emergency stop."""

        ...


class NativeGoalCompletionGate:
    """Correlate native endpoint completion with one immutable goal."""

    def __init__(
        self,
        *,
        request_id: str,
        map_id: str,
        target: PoseTarget,
        goal_tolerance_m: float = 0.10,
    ) -> None:
        self._request_id = str(request_id)
        self._map_id = str(map_id)
        self._target = target
        self._goal_tolerance_m = max(0.01, float(goal_tolerance_m))
        self._seen_not_reached = False

    def observe(self, snapshot: NativeNavigationSnapshot) -> str:
        """Return pending, executing, success, failed, or blocked."""

        if snapshot.map_id != self._map_id:
            return "blocked"
        try:
            robot_pose = (
                float(snapshot.x),
                float(snapshot.y),
                float(snapshot.z),
                float(snapshot.yaw),
            )
        except (TypeError, ValueError):
            return "blocked"
        if not all(math.isfinite(value) for value in robot_pose):
            return "blocked"
        if snapshot.native_control_mode != "autonomy":
            return "blocked"
        if snapshot.native_estop_latched or snapshot.native_operator_takeover_latched:
            return "blocked"
        if (
            snapshot.native_request_id != self._request_id
            or snapshot.native_command_kind != "goal"
            or not snapshot.native_command_accepted
        ):
            return "pending"

        goal = snapshot.native_plan_goal
        if goal is None:
            return "pending"
        try:
            goal = (float(goal[0]), float(goal[1]), float(goal[2]))
        except (IndexError, TypeError, ValueError):
            return "blocked"
        if not all(math.isfinite(value) for value in goal):
            return "blocked"
        goal_error = math.sqrt(
            (goal[0] - self._target.x) ** 2
            + (goal[1] - self._target.y) ** 2
            + (goal[2] - self._target.z) ** 2
        )
        if goal_error > self._goal_tolerance_m:
            return "blocked"
        if not snapshot.native_plan_accepted:
            if snapshot.native_plan_seen and snapshot.native_plan_reason not in {
                "",
                "planning",
                "no_goal_received",
            }:
                return "failed"
            return "pending"
        if not snapshot.native_goal_reached:
            self._seen_not_reached = True
            return "executing"
        if self._seen_not_reached:
            return "success"
        return "pending"


class CorrelatedNativeNavigationPort:
    """Combine native commands with fresh, request-correlated observations."""

    def __init__(
        self,
        *,
        client: NativeNavigationClientPort,
        snapshot: Callable[[], NativeNavigationSnapshot],
        clock: Callable[[], float] = time.time,
        max_native_status_age_s: float = 2.0,
        max_pose_age_s: float = 2.0,
        goal_tolerance_m: float = 0.10,
    ) -> None:
        self._client = client
        self._snapshot = snapshot
        self._clock = clock
        self._max_native_status_age_s = max(0.05, float(max_native_status_age_s))
        self._max_pose_age_s = max(0.05, float(max_pose_age_s))
        self._goal_tolerance_m = max(0.01, float(goal_tolerance_m))
        self._targets: dict[str, PoseTarget] = {}
        self._gates: dict[tuple[str, str], NativeGoalCompletionGate] = {}

    def autonomy_ready(self) -> tuple[bool, str]:
        """Require fresh pose/status and exclusive native autonomy ownership."""

        try:
            snapshot = self._snapshot()
        except Exception:
            return False, "native_navigation_snapshot_error"
        if snapshot.native_estop_latched:
            return False, "native_estop_latched"
        if snapshot.native_operator_takeover_latched:
            return False, "operator_takeover_latched"
        mode = str(snapshot.native_control_mode or "").strip().lower()
        if mode != "autonomy":
            return False, f"native_control_mode_{mode or 'unknown'}"
        now_s = float(self._clock())
        status_age = self._age(now_s, snapshot.native_status_stamp_s)
        if status_age is None or status_age > self._max_native_status_age_s:
            return False, "native_navigation_status_stale"
        pose_age = self._age(now_s, snapshot.stamp_s)
        if pose_age is None or pose_age > self._max_pose_age_s:
            return False, "native_navigation_pose_stale"
        try:
            pose = (
                float(snapshot.x),
                float(snapshot.y),
                float(snapshot.z),
                float(snapshot.yaw),
            )
        except (TypeError, ValueError):
            return False, "native_navigation_pose_invalid"
        if not all(math.isfinite(value) for value in pose):
            return False, "native_navigation_pose_invalid"
        return True, "autonomy_ready"

    def send_goal(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        request_id: str | None = None,
    ) -> None:
        """Send and remember one finite goal under a required request ID."""

        goal_id = str(request_id or "").strip()
        if not goal_id:
            raise ValueError("request_id is required for correlated building navigation")
        values = tuple(float(value) for value in (x, y, z, yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("navigation target must be finite")
        self._client.send_goal(*values, request_id=goal_id)
        self._targets[goal_id] = PoseTarget("map", *values)
        for key in [key for key in self._gates if key[0] == goal_id]:
            self._gates.pop(key, None)

    def observe_goal(
        self,
        *,
        request_id: str,
        map_id: str,
        target: PoseTarget,
    ) -> GoalProgress:
        """Map fresh request-correlated native evidence to building progress."""

        commanded = self._targets.get(str(request_id))
        if commanded is None or commanded != target or not str(map_id).strip():
            return GoalProgress.BLOCKED
        key = (str(request_id), str(map_id))
        gate = self._gates.get(key)
        if gate is None:
            gate = NativeGoalCompletionGate(
                request_id=request_id,
                map_id=map_id,
                target=commanded,
                goal_tolerance_m=self._goal_tolerance_m,
            )
            self._gates[key] = gate
        try:
            result = gate.observe(self._snapshot())
        except Exception:
            return GoalProgress.BLOCKED
        return {
            "pending": GoalProgress.PENDING,
            "executing": GoalProgress.EXECUTING,
            "success": GoalProgress.SUCCEEDED,
            "failed": GoalProgress.FAILED,
            "blocked": GoalProgress.BLOCKED,
        }.get(result, GoalProgress.BLOCKED)

    def cancel(self, reason: str = "cancel", *, request_id: str | None = None) -> None:
        """Forward cancellation to the native client."""

        self._client.cancel(reason, request_id=request_id)

    def stop(self, reason: str = "stop", *, request_id: str | None = None) -> None:
        """Forward a non-latching stop to the native client."""

        self._client.stop(reason, request_id=request_id)

    @staticmethod
    def _age(now_s: float, raw_stamp: Any) -> float | None:
        try:
            stamp_s = float(raw_stamp)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(stamp_s):
            return None
        age_s = now_s - stamp_s
        return age_s if age_s >= -0.5 else None
