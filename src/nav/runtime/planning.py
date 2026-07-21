"""Global planning runtime helpers for Navigation."""

from __future__ import annotations

import logging
import time
from typing import Any

from nav.model.geometry import point_summary
from nav.model.policy import should_use_direct_goal_fallback
from nav.model.state import MissionEvent, MissionState, MissionStateInput, mission_state_name
from nav.services.plan.contracts import GlobalPlanRequest, GlobalPlanResult
from nav.services.plan.preview import planner_last_plan_report, planner_service_name
from runtime.msgs.geometry import Pose, PoseStamped
from runtime.msgs.nav import Path
from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)

PATH_START_REPLACE_TOLERANCE_M = 0.5
PATH_START_INSERT_MAX_M = 2.0


class _MapOnlyPreviewPlanner:
    def __init__(self, planner: Any, planner_constraints: dict[str, Any] | None = None) -> None:
        self._planner = planner
        self._planner_constraints = dict(planner_constraints or {})

    @property
    def is_ready(self) -> bool:
        return bool(getattr(self._planner, "is_ready", False))

    @property
    def has_map(self) -> bool:
        return bool(getattr(self._planner, "has_map", False))

    @property
    def planner_name(self) -> str:
        return str(getattr(self._planner, "planner_name", "octoplanner3d"))

    @property
    def plan_safety_policy(self) -> str:
        return "map_only"

    @property
    def map_artifact_gate(self) -> dict[str, Any]:
        gate = getattr(self._planner, "map_artifact_gate", {}) or {}
        return dict(gate) if isinstance(gate, dict) else {}

    @property
    def last_plan_report(self) -> dict[str, Any]:
        report = getattr(self._planner, "last_plan_report", {}) or {}
        return dict(report) if isinstance(report, dict) else {}

    def backend_status(self) -> dict[str, Any]:
        status = getattr(self._planner, "backend_status", None)
        if callable(status):
            value = status()
            return dict(value) if isinstance(value, dict) else {}
        return {}

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        try:
            path, plan_ms = self._planner.plan_map_only(
                request.start,
                request.goal,
                planner_constraints=self._planner_constraints,
            )
        except Exception as exc:
            report = self.last_plan_report
            return GlobalPlanResult(
                error=str(exc) or type(exc).__name__,
                frame_id=request.frame_id,
                request_id=request.request_id,
                map_version=request.map_version,
                diagnostics=dict(report.get("planner_diagnostics", {}) or {}),
                report=report,
            )
        report = self.last_plan_report
        return GlobalPlanResult(
            path=path,
            plan_ms=float(plan_ms),
            reached_goal=bool(report.get("reached_goal", True)),
            frame_id=request.frame_id,
            request_id=request.request_id,
            map_version=request.map_version,
            adjusted_goal=report.get("adjusted_goal"),
            diagnostics=dict(report.get("planner_diagnostics", {}) or {}),
            report=report,
        )


class NavigationPlanningMixin:
    def preview_plan(
        self,
        x: float,
        y: float,
        z: float = 0.0,
        *,
        map_only: bool = False,
        planner_constraints: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Return a client-facing path preview without changing mission state."""
        frame_blocker = self._frame_contract.planning_frame_blocker(
            self._odom_frame_id,
            self._costmap_frame_id,
        )
        planner = self._planner_svc
        if map_only and hasattr(self._planner_svc, "plan_map_only"):
            planner = _MapOnlyPreviewPlanner(self._planner_svc, planner_constraints)
        return self._plan_preview.preview(
            planner=planner,
            start=self._robot_pos,
            goal=[x, y, z],
            frame_id=self._planning_frame_id,
            frame_blocker=frame_blocker,
        )

    def evaluate_path_safety(self, path: list[list[float]]) -> dict[str, Any] | None:
        """Evaluate a candidate path without exposing planner internals."""

        evaluator = getattr(self._planner_svc, "evaluate_current_path_safety", None)
        if not callable(evaluator):
            return None
        result = evaluator(path)
        return dict(result) if isinstance(result, dict) else None

    def _current_plan_report(self) -> dict[str, Any]:
        if self._using_external_strategy_path:
            return {
                "primary_planner": "external_strategy_path",
                "selected_planner": "external_strategy_path",
                "fallback_used": False,
                "path_safety_ok": None,
                "external_strategy_path_control": True,
                "points": len(self._active_external_strategy_path),
            }
        report = planner_last_plan_report(self._planner_svc)
        if self._deferred_empty_path_first_ts > 0.0:
            deferred_reason = str(report.get("fallback_reason") or self._failure_reason or "empty path deferred")
            sanitized = dict(report)
            sanitized["fallback_reason"] = ""
            sanitized["rejected_plans"] = []
            sanitized["deferred_planning"] = {
                "active": True,
                "reason": deferred_reason,
                "attempts": self._deferred_empty_path_attempts,
                "waited_s": round(time.time() - self._deferred_empty_path_first_ts, 3),
                "retry_interval_s": self._empty_path_retry_interval_s,
                "retry_timeout_s": self._empty_path_retry_timeout_s,
            }
            return sanitized
        return report

    def _republish_external_strategy_path(self) -> None:
        if self._active_external_strategy_path:
            self._publish_global_path(
                [np.asarray(point[:3], dtype=float).copy() for point in self._active_external_strategy_path]
            )
        self._publish_waypoint()

    def _publish_global_path(
        self,
        path: list[np.ndarray],
        plan_result: GlobalPlanResult | None = None,
    ) -> None:
        now = time.time()
        diagnostics: dict[str, Any] = {"source": "navigation_mission"}
        report: dict[str, Any] = {}
        plan_ms = 0.0
        reached_goal = True
        error = ""
        request_id = ""
        map_version = ""
        adjusted_goal = None
        if plan_result is not None:
            diagnostics = dict(plan_result.diagnostics)
            diagnostics.setdefault("source", "navigation_mission")
            report = dict(plan_result.report)
            plan_ms = float(plan_result.plan_ms)
            reached_goal = bool(plan_result.reached_goal)
            error = str(plan_result.error or "")
            request_id = str(plan_result.request_id or "")
            map_version = str(plan_result.map_version or "")
            adjusted_goal = plan_result.adjusted_goal
        if self._path_start_anchor_status:
            report.setdefault(
                "path_start_anchor",
                dict(self._path_start_anchor_status),
            )
        if self._direct_goal_fallback_status is not None:
            diagnostics["source"] = "direct_goal_fallback"
            report.setdefault(
                "direct_goal_fallback",
                dict(self._direct_goal_fallback_status),
            )
        self._last_global_plan = GlobalPlanResult(
            path=path,
            plan_ms=plan_ms,
            reached_goal=reached_goal,
            error=error,
            frame_id=self._planning_frame_id,
            request_id=request_id,
            map_version=map_version,
            adjusted_goal=adjusted_goal,
            diagnostics=diagnostics,
            report=report,
        ).to_wire()
        self.global_path.publish(
            Path(
                poses=[
                    PoseStamped(
                        pose=Pose(
                            float(point[0]),
                            float(point[1]),
                            float(point[2]) if len(point) > 2 else 0.0,
                        ),
                        ts=now,
                        frame_id=self._planning_frame_id,
                    )
                    for point in path
                ],
                ts=now,
                frame_id=self._planning_frame_id,
            )
        )

    def _plan(self) -> None:
        # Snapshot goal under lock for cross-thread consistency
        # (_plan is called from both the dispatch thread and recovery thread).
        _goal = self._get_goal()
        if _goal is None:
            self._apply_event(MissionEvent.PLAN_FAILED, reason="missing goal")
            return

        frame_blocker = self._frame_contract.planning_frame_blocker(
            self._odom_frame_id,
            self._costmap_frame_id,
        )
        if frame_blocker is not None:
            self._block_for_frame_mismatch(*frame_blocker)
            return

        self._apply_event(MissionEvent.REPLAN_REQUESTED, reason="planning requested")
        self._mission_start_time = time.time()
        self._last_costmap_replan_time = time.time()
        self._direct_goal_fallback_status = None
        self._last_global_plan = None
        self._using_external_strategy_path = False
        self._active_external_strategy_path = []

        path = None
        plan_result: GlobalPlanResult | None = None
        if not self._planner_svc.is_ready:
            if self._allow_direct_goal_fallback:
                path = self._direct_goal_path(reason="planner backend not ready")
            else:
                self._failure_reason = "planner backend not ready"
                self._apply_event(
                    MissionEvent.PLAN_FAILED,
                    reason=self._failure_reason,
                )
                return
        else:
            try:
                plan_request = getattr(self._planner_svc, "plan_request", None)
                if callable(plan_request):
                    plan_result = plan_request(
                        GlobalPlanRequest(
                            start=np.asarray(self._robot_pos, dtype=float),
                            goal=np.asarray(_goal, dtype=float),
                            safe_goal_tolerance=self._safe_goal_tolerance,
                            frame_id=self._planning_frame_id,
                        )
                    )
                    if plan_result.error or not plan_result.path:
                        raise RuntimeError(plan_result.error or "planner returned empty path")
                    path = plan_result.points()
                else:
                    path, plan_ms = self._planner_svc.plan(
                        self._robot_pos,
                        _goal,
                        safe_goal_tolerance=self._safe_goal_tolerance,
                    )
                    report = planner_last_plan_report(self._planner_svc)
                    plan_result = GlobalPlanResult(
                        path=path,
                        plan_ms=float(plan_ms),
                        reached_goal=bool(report.get("reached_goal", True)),
                        frame_id=self._planning_frame_id,
                        adjusted_goal=report.get("adjusted_goal"),
                        diagnostics=dict(report.get("planner_diagnostics", {}) or {}),
                        report=report,
                    )
                self._publish_plan_report(success=True)
            except Exception as exc:
                self._publish_plan_report(success=False, reason=str(exc))
                if self._should_use_direct_goal_fallback(exc):
                    path = self._direct_goal_path(reason=str(exc))
                elif self._should_defer_empty_path_failure(exc):
                    self._defer_empty_path_failure(str(exc), MissionState.EXECUTING)
                    return
                else:
                    logger.exception("Planning failed: %s", exc)
                    self._failure_reason = str(exc)
                    self._apply_event(
                        MissionEvent.PLAN_FAILED,
                        reason=self._failure_reason,
                    )
                    return

        try:
            path = self._validate_planned_path(path)
            path = self._anchor_path_start_to_robot(path)
        except RuntimeError as exc:
            if self._should_defer_empty_path_failure(exc):
                self._defer_empty_path_failure(str(exc), MissionState.EXECUTING)
                return
            logger.error("Planning failed: %s", exc)
            self._failure_reason = str(exc)
            self._tracker.clear()
            self._publish_motion_stop()
            self._apply_event(
                MissionEvent.PLAN_FAILED,
                reason=self._failure_reason,
            )
            return

        self._failure_reason = ""
        self._clear_deferred_empty_path()
        self._active_path_terminal_goal = np.asarray(path[-1][:3], dtype=float).copy()
        self._tracker.reset(path, self._robot_pos, self._robot_yaw)
        # Advance past any waypoints the robot is already at (e.g. the start)
        self._tracker.update(self._robot_pos, self._robot_yaw)
        self._publish_global_path(path, plan_result=plan_result)
        self._apply_event(MissionEvent.PLAN_OK, reason="plan ready")
        self._publish_waypoint()

    def _publish_plan_report(
        self,
        *,
        success: bool | None = None,
        reason: str = "",
    ) -> None:
        report = planner_last_plan_report(self._planner_svc)
        if not report and success is None:
            return
        selected = report.get("selected_planner") if report else None
        rejected = report.get("rejected_plans") or []
        planner_name = report.get("primary_planner") or planner_service_name(self._planner_svc, selected)
        if selected != planner_name or rejected:
            payload = {
                "event": "global_plan_selection",
                "selected_planner": selected,
                "fallback_reason": report.get("fallback_reason", ""),
                "rejected_plans": rejected,
                "policy": report.get("policy", ""),
                "ts": time.time(),
            }
            self.adapter_status.publish(payload)
        if success is not None:
            health_reason = str(
                reason or report.get("fallback_reason", "") or ("plan accepted" if success else "planning failed")
            )
            signature = (
                bool(success),
                str(selected or planner_name or "unknown"),
                health_reason,
                str(report.get("request_id") or ""),
            )
            if signature != self._last_map_health_plan_signature:
                self._last_map_health_plan_signature = signature
                self.map_health_event.publish(
                    {
                        "schema_version": "map.health.planning.v1",
                        "success": bool(success),
                        "planner": str(selected or planner_name or "unknown"),
                        "reason": health_reason,
                        "ts": time.time(),
                    }
                )

    def _should_use_direct_goal_fallback(self, exc: Exception) -> bool:
        return should_use_direct_goal_fallback(
            exc,
            allow=self._allow_direct_goal_fallback,
            on_planner_failure=self._direct_goal_fallback_on_planner_failure,
        )

    def _should_defer_empty_path_failure(self, exc: Exception) -> bool:
        if not self._defer_empty_path_planning_failure:
            return False
        text = str(exc).lower()
        return "empty path" in text or "no path" in text or "no reachable free cell" in text

    def _clear_deferred_empty_path(self) -> None:
        self._deferred_empty_path_first_ts = 0.0
        self._deferred_empty_path_attempts = 0

    def _defer_empty_path_failure(
        self,
        reason: str,
        planned_state: MissionStateInput,
    ) -> None:
        now = time.time()
        if self._deferred_empty_path_first_ts <= 0.0:
            self._deferred_empty_path_first_ts = now
            self._deferred_empty_path_attempts = 0
        self._deferred_empty_path_attempts += 1
        waited_s = now - self._deferred_empty_path_first_ts
        if waited_s > self._empty_path_retry_timeout_s:
            logger.error("Planning failed after deferred retries: %s", reason)
            self._failure_reason = reason
            self._tracker.clear()
            self._publish_motion_stop()
            self._clear_deferred_empty_path()
            self._apply_event(
                MissionEvent.PLAN_FAILED,
                reason=self._failure_reason,
            )
            return

        self._failure_reason = ""
        self._tracker.clear()
        self._publish_motion_stop()
        self.adapter_status.publish(
            {
                "event": "planning_deferred_empty_path",
                "reason": reason,
                "planned_state": mission_state_name(planned_state),
                "attempt": self._deferred_empty_path_attempts,
                "waited_s": round(waited_s, 3),
                "retry_interval_s": self._empty_path_retry_interval_s,
                "retry_timeout_s": self._empty_path_retry_timeout_s,
                "goal": point_summary(self._goal),
                "ts": now,
            }
        )
        self._apply_event(
            MissionEvent.REPLAN_REQUESTED,
            reason="planning deferred",
        )

    def _direct_goal_path(self, reason: str = "") -> list[np.ndarray]:
        _goal = self._get_goal()
        if _goal is None:
            raise RuntimeError("direct_goal_fallback called with no goal set")
        logger.warning(
            "Navigation: using direct-goal fallback waypoint (reason=%s)",
            reason or "planner unavailable",
        )
        self._direct_goal_fallback_status = {
            "used": True,
            "reason": reason or "planner unavailable",
            "goal": [float(_goal[0]), float(_goal[1]), float(_goal[2])],
            "ts": time.time(),
        }
        self.adapter_status.publish(
            {
                "event": "direct_goal_fallback",
                **self._direct_goal_fallback_status,
            }
        )
        return [_goal.copy()]

    def _should_continue_after_partial_path(self) -> bool:
        status = self._partial_path_terminal_status()
        if not status.get("partial"):
            return False
        if self._accept_partial_goal_progress:
            return False
        return True

    def _partial_path_terminal_status(self) -> dict[str, Any]:
        if self._using_external_strategy_path:
            return {"partial": False}
        if self._goal is None or self._active_path_terminal_goal is None:
            return {"partial": False}
        try:
            goal = np.asarray(self._goal[:2], dtype=float)
            terminal = np.asarray(self._active_path_terminal_goal[:2], dtype=float)
            robot = np.asarray(self._robot_pos[:2], dtype=float)
        except (TypeError, ValueError):
            return {"partial": False}
        if not (np.all(np.isfinite(goal)) and np.all(np.isfinite(terminal)) and np.all(np.isfinite(robot))):
            return {"partial": False}
        final_threshold = float(getattr(self._tracker, "_final_threshold", 0.0))
        threshold = max(final_threshold, 0.05)
        terminal_gap = float(np.linalg.norm(terminal - goal))
        robot_gap = float(np.linalg.norm(robot - goal))
        if terminal_gap <= threshold or robot_gap <= threshold:
            return {
                "partial": False,
                "terminal_gap_m": round(terminal_gap, 3),
                "robot_gap_m": round(robot_gap, 3),
            }
        report = planner_last_plan_report(self._planner_svc)
        primary_replan = report.get("primary_replan")
        partial = bool(primary_replan) or report.get("reached_goal") is False
        reason = ""
        if bool(primary_replan):
            reason = "primary_replan"
        elif report.get("reached_goal") is False:
            reason = "planner_partial_path"
        return {
            "partial": partial,
            "terminal_gap_m": round(terminal_gap, 3),
            "robot_gap_m": round(robot_gap, 3),
            "selected_planner": report.get("selected_planner"),
            "reason": reason,
        }

    def _record_partial_goal_progress_complete(self) -> None:
        if self._goal is None or self._active_path_terminal_goal is None:
            return
        self._partial_progress_completed_goal = np.asarray(self._goal[:3], dtype=float).copy()
        self._partial_progress_completed_terminal = np.asarray(self._active_path_terminal_goal[:3], dtype=float).copy()
        self._partial_progress_completed_ts = time.time()

    def _clear_partial_goal_progress(self) -> None:
        self._partial_progress_completed_goal = None
        self._partial_progress_completed_terminal = None
        self._partial_progress_completed_ts = 0.0

    def _validate_planned_path(self, path: Any) -> list[np.ndarray]:
        try:
            points = list(path) if path is not None else []
        except TypeError as exc:
            raise RuntimeError("planner returned a non-iterable path") from exc
        if not points:
            raise RuntimeError("planner returned empty path")

        validated: list[np.ndarray] = []
        for point in points:
            try:
                arr = np.asarray(point, dtype=float).reshape(-1)
            except (TypeError, ValueError) as exc:
                raise RuntimeError("planner returned an invalid path point") from exc
            if arr.size < 2:
                raise RuntimeError("planner returned a path point with fewer than 2 coordinates")
            if not np.all(np.isfinite(arr)):
                raise RuntimeError("planner returned a non-finite path point")
            if arr.size < 3:
                arr = np.pad(arr[:2], (0, 1), constant_values=0.0)
            else:
                arr = arr[:3].copy()
            validated.append(arr)
        return validated

    def _anchor_path_start_to_robot(self, path: list[np.ndarray]) -> list[np.ndarray]:
        if not path:
            return path
        robot = np.asarray(self._robot_pos[:3], dtype=float)
        first = np.asarray(path[0][:3], dtype=float)
        if not (np.all(np.isfinite(robot)) and np.all(np.isfinite(first))):
            self._path_start_anchor_status = {
                "mode": "skipped_non_finite",
            }
            return path
        start_gap_m = float(np.linalg.norm(first[:2] - robot[:2]))
        if start_gap_m > PATH_START_INSERT_MAX_M:
            self._path_start_anchor_status = {
                "mode": "skipped_far",
                "distance_m": round(start_gap_m, 3),
                "insert_max_m": PATH_START_INSERT_MAX_M,
                "replace_tolerance_m": PATH_START_REPLACE_TOLERANCE_M,
                "insert_enabled": self._allow_path_start_insert,
            }
            return path
        if start_gap_m > PATH_START_REPLACE_TOLERANCE_M and not self._allow_path_start_insert:
            self._path_start_anchor_status = {
                "mode": "skipped_insert_disabled",
                "distance_m": round(start_gap_m, 3),
                "insert_max_m": PATH_START_INSERT_MAX_M,
                "replace_tolerance_m": PATH_START_REPLACE_TOLERANCE_M,
                "insert_enabled": False,
            }
            return path
        anchored = [point.copy() for point in path]
        original_z = np.array([point[2] for point in anchored], dtype=float)
        planar_zero_height = bool(
            original_z.size > 0
            and np.all(np.isfinite(original_z))
            and float(np.ptp(original_z)) <= 1e-6
            and abs(float(original_z[0])) <= 1e-6
        )
        if planar_zero_height:
            for point in anchored:
                point[2] = robot[2]
        if start_gap_m <= PATH_START_REPLACE_TOLERANCE_M:
            anchored[0] = robot.copy()
            mode = "replace"
        else:
            anchored.insert(0, robot.copy())
            mode = "insert"
        self._path_start_anchor_status = {
            "mode": mode,
            "distance_m": round(start_gap_m, 3),
            "insert_max_m": PATH_START_INSERT_MAX_M,
            "replace_tolerance_m": PATH_START_REPLACE_TOLERANCE_M,
            "insert_enabled": self._allow_path_start_insert,
            "planar_zero_height": planar_zero_height,
        }
        return anchored
