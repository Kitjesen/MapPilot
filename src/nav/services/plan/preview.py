"""Planner preview and diagnostics helpers."""

from __future__ import annotations

import concurrent.futures
import logging
import math
import threading
import time
from typing import Any

from nav.services.plan.contracts import (
    GLOBAL_PLAN_SCHEMA_VERSION,
    GlobalPlanRequest,
    GlobalPlanResult,
)
from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


class PlanPreviewBusy(RuntimeError):
    """Raised when another non-motion plan preview is running."""


def planner_last_plan_report(planner: Any) -> dict[str, Any]:
    report = getattr(planner, "last_plan_report", {}) or {}
    if callable(report):
        report = report()
    return dict(report) if isinstance(report, dict) else {}


def planner_backend_status_payload(planner: Any) -> dict[str, Any]:
    backend_status = getattr(planner, "backend_status", None)
    if not callable(backend_status):
        return {}
    try:
        status = backend_status()
    except Exception:
        return {}
    return dict(status) if isinstance(status, dict) else {}


def planner_service_name(planner: Any, default: str | None = None) -> str | None:
    try:
        name = getattr(planner, "planner_name", None)
        if callable(name):
            name = name()
    except Exception:
        name = None
    if name:
        return str(name)

    status = planner_backend_status_payload(planner)
    name = status.get("configured_backend") or status.get("backend")
    if name:
        return str(name)

    report = planner_last_plan_report(planner)
    name = report.get("primary_planner") or report.get("selected_planner")
    return str(name) if name else default


def planner_plan_safety_policy(
    planner: Any,
    default: str | None = None,
) -> str | None:
    try:
        policy = getattr(planner, "plan_safety_policy", None)
        if callable(policy):
            policy = policy()
    except Exception:
        policy = None
    if policy:
        return str(policy)

    report = planner_last_plan_report(planner)
    policy = report.get("policy")
    return str(policy) if policy else default


def planner_map_artifact_gate(planner: Any) -> dict[str, Any]:
    try:
        gate = getattr(planner, "map_artifact_gate", {}) or {}
    except Exception:
        return {}
    if callable(gate):
        gate = gate()
    return dict(gate) if isinstance(gate, dict) else {}


def planner_preview_report_fields(planner: Any) -> dict[str, Any]:
    report = planner_last_plan_report(planner)
    status = planner_backend_status_payload(planner)
    selected = (
        report.get("selected_planner")
        or status.get("backend")
        or planner_service_name(planner)
    )
    return {
        "planner": selected or planner_service_name(planner),
        "selected_planner": selected,
        "plan_safety_policy": (
            report.get("policy") or planner_plan_safety_policy(planner)
        ),
        "path_safety": report.get("selected_path_safety"),
        "fallback_reason": report.get("fallback_reason", ""),
        "rejected_plans": list(report.get("rejected_plans") or []),
    }


def planner_backend_status(planner: Any) -> dict[str, Any]:
    backend_status = planner_backend_status_payload(planner)
    if backend_status:
        return backend_status
    report = planner_last_plan_report(planner)
    selected = str(
        report.get("selected_planner")
        or planner_service_name(planner, "unknown")
    )
    configured = str(
        report.get("primary_planner")
        or planner_service_name(planner, selected)
    )
    fallback_reason = str(report.get("fallback_reason") or "")
    return {
        "configured_backend": configured,
        "backend": selected,
        "fallback_backend": str(report.get("fallback_planner") or ""),
        "degraded": bool(fallback_reason) or selected != configured,
        "degraded_reason": fallback_reason,
    }


class PlanPreviewService:
    def __init__(self, *, timeout_s: float) -> None:
        self._timeout_s = timeout_s
        self._executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="plan-preview",
        )
        self._lock = threading.Lock()

    @property
    def closed(self) -> bool:
        return bool(getattr(self._executor, "_shutdown", False))

    def shutdown(self) -> None:
        self._executor.shutdown(wait=False, cancel_futures=True)

    def preview(
        self,
        *,
        planner: Any,
        start: Any,
        goal: Any,
        frame_id: str,
        frame_blocker: tuple[str, str] | None,
    ) -> dict[str, Any]:
        ts = time.time()
        start_arr = np.asarray(start, dtype=float).reshape(-1)[:3].copy()
        if start_arr.size < 3:
            start_arr = np.pad(start_arr, (0, 3 - start_arr.size), constant_values=0.0)
        goal_arr = np.asarray(goal, dtype=float).reshape(-1)[:3].copy()
        result: dict[str, Any] = {
            "schema_version": 1,
            "global_plan_schema_version": GLOBAL_PLAN_SCHEMA_VERSION,
            "global_plan": None,
            "ok": True,
            "feasible": False,
            "frame_id": frame_id,
            "start": self._path_point(start_arr, frame_id=frame_id, ts=ts),
            "goal": self._path_point(goal_arr, frame_id=frame_id, ts=ts),
            "adjusted_goal": None,
            "path": [],
            "count": 0,
            "distance_m": None,
            "plan_ms": None,
            "planner": planner_service_name(planner),
            "selected_planner": None,
            "plan_safety_policy": planner_plan_safety_policy(planner),
            "path_safety": None,
            "fallback_reason": "",
            "rejected_plans": [],
            "source": "navigation_preview",
            "reasons": [],
            "error": None,
            "ts": ts,
        }

        if not np.all(np.isfinite(start_arr)):
            result["reasons"] = ["odometry_invalid"]
            result["error"] = "current robot position is not finite"
            return result
        if frame_blocker is not None:
            source, bad_frame = frame_blocker
            result["reasons"] = ["frame_mismatch"]
            result["error"] = (
                f"unsupported {source} frame {bad_frame!r}; expected {frame_id!r}"
            )
            return result
        if goal_arr.size < 3 or not np.all(np.isfinite(goal_arr)):
            result["ok"] = False
            result["reasons"] = ["goal_invalid"]
            result["error"] = "goal position is not finite"
            return result

        try:
            planner_ready = bool(getattr(planner, "is_ready", False))
            planner_has_map = bool(getattr(planner, "has_map", False))
        except Exception as exc:
            result["reasons"] = ["planner_status_unavailable"]
            result["error"] = str(exc)
            return result
        if not planner_ready:
            result["reasons"] = ["planner_not_ready"] + (
                ["map_unavailable"] if not planner_has_map else []
            )
            return result

        with np.errstate(over="ignore", invalid="ignore"):
            start_goal_delta = float(np.linalg.norm(goal_arr[:2] - start_arr[:2]))
        if not math.isfinite(start_goal_delta):
            result["reasons"] = ["goal_invalid"]
            result["error"] = "goal distance from current position is not finite"
            return result
        if start_goal_delta <= 0.05:
            result.update({
                "feasible": True,
                "path": [self._path_point(start_arr, frame_id=frame_id, ts=ts, index=0)],
                "count": 1,
                "distance_m": 0.0,
                "plan_ms": 0.0,
                "global_plan": GlobalPlanResult(
                    path=[start_arr],
                    plan_ms=0.0,
                    reached_goal=True,
                    frame_id=frame_id,
                ).to_wire(),
                "reasons": ["already_at_goal"],
            })
            return result

        try:
            path, plan_ms = self._plan_with_timeout(
                planner,
                start_arr,
                goal_arr,
                frame_id=frame_id,
            )
        except TimeoutError as exc:
            logger.warning("Plan preview timed out: %s", exc)
            result["reasons"] = ["planning_timeout"]
            result["error"] = str(exc)
            return result
        except PlanPreviewBusy as exc:
            result["reasons"] = ["planning_preview_busy"]
            result["error"] = str(exc)
            return result
        except Exception as exc:
            logger.warning("Plan preview failed: %s", exc)
            result["reasons"] = ["planning_failed"]
            result["error"] = str(exc)
            result.update(planner_preview_report_fields(planner))
            return result

        return self._serialize_plan_result(
            result,
            planner=planner,
            path=path,
            plan_ms=plan_ms,
            goal=goal_arr,
            frame_id=frame_id,
            ts=ts,
        )

    def _plan_with_timeout(
        self,
        planner: Any,
        start: np.ndarray,
        goal: np.ndarray,
        frame_id: str,
    ) -> tuple[Any, Any]:
        if not self._lock.acquire(blocking=False):
            raise PlanPreviewBusy("another plan preview is already running")

        def _run() -> tuple[Any, Any]:
            try:
                plan_request = getattr(planner, "plan_request", None)
                if callable(plan_request):
                    result = plan_request(
                        GlobalPlanRequest(start=start, goal=goal, frame_id=frame_id)
                    )
                    if result.error or not result.path:
                        raise RuntimeError(result.error or "planner returned empty path")
                    return result.points(), result.plan_ms
                return planner.plan(start, goal)
            finally:
                self._lock.release()

        future = self._executor.submit(_run)
        try:
            return future.result(timeout=max(0.001, self._timeout_s))
        except concurrent.futures.TimeoutError as exc:
            if future.cancel():
                self._lock.release()
            raise TimeoutError(
                f"planner preview exceeded {self._timeout_s:.1f}s"
            ) from exc

    def _serialize_plan_result(
        self,
        result: dict[str, Any],
        *,
        planner: Any,
        path: Any,
        plan_ms: Any,
        goal: np.ndarray,
        frame_id: str,
        ts: float,
    ) -> dict[str, Any]:
        try:
            path_points = list(path) if path is not None else []
        except TypeError:
            result["reasons"] = ["planner_returned_invalid_path"]
            result["error"] = "planner returned a non-iterable path"
            return result
        if not path_points:
            result["reasons"] = ["empty_path"]
            result["error"] = "planner returned empty path"
            return result

        path_arrays: list[np.ndarray] = []
        for point in path_points:
            arr = self._path_array(point)
            if arr is None:
                result["reasons"] = ["planner_returned_nonfinite_path"]
                result["error"] = "planner returned a non-finite path point"
                return result
            path_arrays.append(arr)

        try:
            plan_ms_value = float(plan_ms)
        except (TypeError, ValueError):
            result["reasons"] = ["planner_returned_invalid_timing"]
            result["error"] = "planner returned an invalid plan_ms value"
            return result
        if not math.isfinite(plan_ms_value):
            result["reasons"] = ["planner_returned_invalid_timing"]
            result["error"] = "planner returned a non-finite plan_ms value"
            return result

        distance_m = self._path_distance(path_arrays)
        if not math.isfinite(distance_m):
            result["reasons"] = ["planner_returned_invalid_distance"]
            result["error"] = "planner returned a path with non-finite distance"
            return result

        serialized = [
            self._path_point(point, frame_id=frame_id, ts=ts, index=index)
            for index, point in enumerate(path_arrays)
        ]
        final = path_arrays[-1]
        with np.errstate(over="ignore", invalid="ignore"):
            goal_delta = float(np.linalg.norm(final[:2] - goal[:2]))
        if not math.isfinite(goal_delta):
            result["reasons"] = ["planner_returned_invalid_distance"]
            result["error"] = "planner returned a path with non-finite goal distance"
            return result
        adjusted_goal = (
            self._path_point(final, frame_id=frame_id, ts=ts)
            if goal_delta > 0.05
            else None
        )

        result.update({
            "feasible": True,
            "path": serialized,
            "count": len(serialized),
            "distance_m": distance_m,
            "plan_ms": plan_ms_value,
            "adjusted_goal": adjusted_goal,
            "global_plan": GlobalPlanResult(
                path=path_arrays,
                plan_ms=plan_ms_value,
                reached_goal=adjusted_goal is None,
                frame_id=frame_id,
                adjusted_goal=final if adjusted_goal is not None else None,
                diagnostics={"source": "navigation_preview"},
            ).to_wire(),
            "reasons": ["goal_adjusted"] if adjusted_goal is not None else [],
        })
        result.update(planner_preview_report_fields(planner))
        return result

    @staticmethod
    def _path_point(
        point: Any,
        *,
        frame_id: str,
        ts: float | None = None,
        index: int | None = None,
    ) -> dict[str, Any]:
        arr = np.asarray(point, dtype=float).reshape(-1)
        metadata = {"index": index} if index is not None else {}
        return {
            "x": float(arr[0]) if arr.size >= 1 and np.isfinite(arr[0]) else 0.0,
            "y": float(arr[1]) if arr.size >= 2 and np.isfinite(arr[1]) else 0.0,
            "z": float(arr[2]) if arr.size >= 3 and np.isfinite(arr[2]) else 0.0,
            "frame_id": frame_id,
            "ts": ts,
            "metadata": metadata,
        }

    @staticmethod
    def _path_array(point: Any) -> np.ndarray | None:
        try:
            arr = np.asarray(point, dtype=float).reshape(-1)
        except (TypeError, ValueError):
            return None
        if arr.size < 2 or not np.all(np.isfinite(arr)):
            return None
        if arr.size < 3:
            return np.pad(arr[:2], (0, 1), constant_values=0.0)
        return arr[:3].copy()

    @staticmethod
    def _path_distance(path: list[np.ndarray]) -> float:
        total = 0.0
        for prev, curr in zip(path, path[1:]):
            a = np.asarray(prev, dtype=float).reshape(-1)
            b = np.asarray(curr, dtype=float).reshape(-1)
            if a.size >= 2 and b.size >= 2:
                with np.errstate(over="ignore", invalid="ignore"):
                    total += float(np.linalg.norm(b[:2] - a[:2]))
        return total
