"""Planner preview and diagnostics helpers."""

from __future__ import annotations

import concurrent.futures
import logging
import math
import re
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
SNAP_DISTANCE_TOLERANCE_M = 0.05
SNAP_LOG_RE = re.compile(
    r"(?P<endpoint>Start|Goal) snapped to free cell:\s*"
    r"\[(?P<x>[-+0-9.eE]+),\s*(?P<y>[-+0-9.eE]+),\s*(?P<z>[-+0-9.eE]+)\]"
)


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
    fields = {
        "planner": selected or planner_service_name(planner),
        "selected_planner": selected,
        "plan_safety_policy": (
            report.get("policy") or planner_plan_safety_policy(planner)
        ),
        "path_safety": report.get("selected_path_safety"),
        "fallback_reason": report.get("fallback_reason", ""),
        "rejected_plans": list(report.get("rejected_plans") or []),
    }
    diagnostics = report.get("planner_diagnostics")
    if isinstance(diagnostics, dict):
        fields["planner_diagnostics"] = dict(diagnostics)
    if "reached_goal" in report:
        fields["reached_goal"] = bool(report.get("reached_goal"))
    for key in (
        "requested_start",
        "accepted_start",
        "adjusted_start",
        "requested_goal",
        "accepted_goal",
        "map_only_preview",
        "primary_replan",
    ):
        if key in report:
            fields[key] = report[key]
    return fields


def _xyz_list(value: Any) -> list[float] | None:
    if isinstance(value, dict):
        raw = (value.get("x"), value.get("y"), value.get("z", 0.0))
    else:
        raw = value
    try:
        xyz = [float(raw[0]), float(raw[1]), float(raw[2])]
    except (TypeError, ValueError, IndexError):
        return None
    return xyz if all(math.isfinite(item) for item in xyz) else None


def _xyz_distance(a: list[float] | None, b: list[float] | None) -> float | None:
    if a is None or b is None:
        return None
    distance = math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    )
    return distance if math.isfinite(distance) else None


def _xy_distance(a: list[float] | None, b: list[float] | None) -> float | None:
    if a is None or b is None:
        return None
    distance = math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
    )
    return distance if math.isfinite(distance) else None


def _rounded_distance(value: float | None) -> float | None:
    return round(float(value), 3) if value is not None else None


def _snap_points_from_diagnostics(diagnostics: dict[str, Any]) -> dict[str, list[float]]:
    points: dict[str, list[float]] = {}
    text = "\n".join(
        str(diagnostics.get(key) or "")
        for key in ("stderr", "stdout", "error_message")
    )
    for match in SNAP_LOG_RE.finditer(text):
        endpoint = match.group("endpoint").lower()
        points[endpoint] = [
            float(match.group("x")),
            float(match.group("y")),
            float(match.group("z")),
        ]
    return points


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
            "snap_diagnostics": None,
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
            self._attach_snap_diagnostics(result, planner=planner)
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
        report = planner_last_plan_report(planner)
        reported_reached_goal = report.get("reached_goal")
        reached_goal = (
            bool(reported_reached_goal)
            if isinstance(reported_reached_goal, bool)
            else adjusted_goal is None
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
                reached_goal=reached_goal,
                frame_id=frame_id,
                adjusted_goal=final if adjusted_goal is not None else None,
                diagnostics={"source": "navigation_preview"},
            ).to_wire(),
            "reasons": ["goal_adjusted"] if adjusted_goal is not None else [],
        })
        result.update(planner_preview_report_fields(planner))
        self._attach_snap_diagnostics(result, planner=planner)
        return result

    def _attach_snap_diagnostics(
        self,
        result: dict[str, Any],
        *,
        planner: Any,
    ) -> None:
        report = planner_last_plan_report(planner)
        diagnostics = report.get("planner_diagnostics")
        if not isinstance(diagnostics, dict):
            diagnostics = result.get("planner_diagnostics")
        if not isinstance(diagnostics, dict):
            diagnostics = {}

        path = result.get("path") or []
        logged_snap = _snap_points_from_diagnostics(diagnostics)
        requested_start = (
            _xyz_list(report.get("requested_start"))
            or _xyz_list(diagnostics.get("start_xyz"))
            or _xyz_list(result.get("start"))
        )
        accepted_start = _xyz_list(report.get("accepted_start")) or logged_snap.get("start")
        if accepted_start is None and path:
            accepted_start = _xyz_list(path[0])
        requested_goal = (
            _xyz_list(report.get("requested_goal"))
            or _xyz_list(diagnostics.get("goal_xyz"))
            or _xyz_list(result.get("goal"))
        )
        accepted_goal = _xyz_list(report.get("accepted_goal")) or logged_snap.get("goal")
        if accepted_goal is None and path:
            accepted_goal = _xyz_list(path[-1])

        start_delta = _xyz_distance(requested_start, accepted_start)
        goal_delta = _xyz_distance(requested_goal, accepted_goal)
        start_xy_delta = _xy_distance(requested_start, accepted_start)
        goal_xy_delta = _xy_distance(requested_goal, accepted_goal)
        start_snapped = (
            start_delta is not None and start_delta > SNAP_DISTANCE_TOLERANCE_M
        )
        goal_snapped = (
            goal_delta is not None and goal_delta > SNAP_DISTANCE_TOLERANCE_M
        )
        reached_goal = report.get("reached_goal")
        if not isinstance(reached_goal, bool):
            reached_goal = result.get("reached_goal")
        result["snap_diagnostics"] = {
            "requested_start": requested_start,
            "effective_start": accepted_start or requested_start,
            "snapped_start": accepted_start if start_snapped else None,
            "start_snapped": start_snapped,
            "start_snap_distance_m": _rounded_distance(start_delta),
            "start_snap_xy_distance_m": _rounded_distance(start_xy_delta),
            "requested_goal": requested_goal,
            "effective_goal": accepted_goal or requested_goal,
            "snapped_goal": accepted_goal if goal_snapped else None,
            "goal_snapped": goal_snapped,
            "goal_snap_distance_m": _rounded_distance(goal_delta),
            "goal_snap_xy_distance_m": _rounded_distance(goal_xy_delta),
            "goal_snap_accepted": reached_goal if isinstance(reached_goal, bool) else None,
            "source": "path_endpoints" if path else "planner_report",
        }

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
