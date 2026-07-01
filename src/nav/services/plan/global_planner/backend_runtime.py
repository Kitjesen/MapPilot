"""Backend lifecycle and call helpers for map-backed global planners."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

from runtime.msgs.numpy_compat import np
from runtime.profiles.planner_backends import normalize_planner_name
from nav.services.plan.contracts import (
    GlobalPlanRequest,
    GlobalPlanResult,
    PlanningMap,
    coerce_planning_map,
    require_global_planner_backend,
)

logger = logging.getLogger(__name__)

_GLOBAL_PLANNERS = ("octoplanner3d", "pct", "direct")

_OCTOPLANNER3D_CONSTRAINT_KEYS = {
    "robot_radius",
    "max_iterations",
    "snap_search_radius_cells",
    "require_ground_support",
    "strict_direct_ground_support",
    "ground_support_xy_radius_cells",
    "ground_support_depth_cells",
    "enable_preblocked_costmap",
    "preblocked_costmap_radius_cells",
    "preblocked_costmap_weight",
    "lowest_traversable_only",
}


@dataclass(slots=True)
class BackendPlanExecution:
    path: list[np.ndarray]
    plan_ms: float
    result: GlobalPlanResult


def normalize_octoplanner3d_constraints(
    constraints: dict[str, Any] | None,
) -> dict[str, Any]:
    if not constraints:
        return {}
    return {
        key: value
        for key, value in constraints.items()
        if key in _OCTOPLANNER3D_CONSTRAINT_KEYS and value is not None
    }


def create_planner_backend(name: str, map_path: str, obstacle_thr: float) -> Any:
    canonical = normalize_planner_name(name)
    if canonical == "octoplanner3d":
        from nav.services.plan.global_planner.algorithm.octoplanner3d_planner import (
            OctoPlanner3DPlanner,
        )

        backend = OctoPlanner3DPlanner(map_path, obstacle_thr)
    elif canonical == "pct":
        from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner

        backend = PCTPlanner(map_path, obstacle_thr)
    elif canonical == "direct":
        from nav.services.plan.global_planner.algorithm.direct_path import (
            DirectPathBackend,
        )

        backend = DirectPathBackend(map_path, obstacle_thr)
    else:
        raise ValueError(
            f"Unknown planner: '{canonical}'. Available: {list(_GLOBAL_PLANNERS)}"
        )
    return require_global_planner_backend(canonical, backend)


def configure_backend(backend: Any, name: str, options: dict[str, Any]) -> None:
    if normalize_planner_name(name) != "octoplanner3d":
        return
    configure_constraints = getattr(backend, "configure_constraints", None)
    if callable(configure_constraints):
        configure_constraints(options)


def load_static_occupancy_into_backend(backend: Any, occupancy_path: str) -> None:
    if not callable(getattr(backend, "update_map", None)) or not occupancy_path:
        return
    try:
        data = np.load(str(occupancy_path))
        planning_map = PlanningMap(
            grid=np.asarray(data["grid"], dtype=np.float32),
            resolution=float(np.asarray(data["resolution"]).reshape(-1)[0]),
            origin=np.asarray(data["origin"], dtype=float).reshape(-1)[:2],
            source=str(occupancy_path),
        )
    except Exception as exc:
        diagnostics = backend_plan_diagnostics(backend)
        diagnostics.update(
            {
                "static_occupancy_path": occupancy_path,
                "static_occupancy_load_error": str(exc),
            }
        )
        try:
            backend._last_plan_diagnostics = diagnostics
        except Exception:
            logger.debug("failed to attach occupancy diagnostics", exc_info=True)
        return
    push_backend_map_update(backend, planning_map)


def push_backend_map_update(backend: Any, planning_map: Any) -> None:
    if backend is None or not callable(getattr(backend, "update_map", None)):
        return
    map_payload = coerce_planning_map(planning_map)
    backend.update_map(
        map_payload.grid,
        resolution=map_payload.resolution,
        origin=map_payload.origin,
    )


def plan_backend(backend: Any, request: GlobalPlanRequest) -> BackendPlanExecution:
    t0 = time.perf_counter()
    try:
        plan_request = getattr(backend, "plan_request", None)
        if callable(plan_request):
            result = plan_request(request)
        else:
            path = backend.plan(request.start, request.goal)
            result = GlobalPlanResult(
                path=path,
                reached_goal=bool(getattr(backend, "_last_plan_reached_goal", True)),
                error=str(getattr(backend, "_last_plan_error", "") or ""),
                frame_id=request.frame_id,
                request_id=request.request_id,
                map_version=request.map_version,
                diagnostics=backend_plan_diagnostics(backend),
            )
    except Exception as exc:  # pragma: no cover - exercised through service callers
        record_backend_plan_exception(backend, exc)
        result = GlobalPlanResult(
            error=str(exc) or type(exc).__name__,
            frame_id=request.frame_id,
            request_id=request.request_id,
            map_version=request.map_version,
            diagnostics={
                "stage": "backend_plan_exception",
                "error_type": type(exc).__name__,
                "error_message": str(exc) or type(exc).__name__,
            },
        )
    store_backend_plan_result(backend, result)
    return BackendPlanExecution(
        path=result.points(),
        plan_ms=(time.perf_counter() - t0) * 1000.0,
        result=result,
    )


def backend_plan_diagnostics(backend: Any) -> dict[str, Any]:
    result = getattr(backend, "_last_plan_result", None)
    if isinstance(result, GlobalPlanResult):
        return dict(result.diagnostics)
    diagnostics = getattr(backend, "_last_plan_diagnostics", None)
    if not isinstance(diagnostics, dict):
        return {}
    return dict(diagnostics)


def store_backend_plan_result(backend: Any, result: GlobalPlanResult) -> None:
    try:
        backend._last_plan_result = result
        backend._last_plan_error = result.error
        backend._last_plan_diagnostics = dict(result.diagnostics)
        backend._last_plan_reached_goal = bool(result.reached_goal)
    except Exception:
        logger.debug("GlobalPlanner: failed to store backend plan result", exc_info=True)


def record_backend_plan_exception(backend: Any, exc: Exception) -> None:
    message = str(exc) or type(exc).__name__
    try:
        backend._last_plan_error = message
        backend._last_plan_diagnostics = {
            "stage": "backend_plan_exception",
            "error_type": type(exc).__name__,
            "error_message": message,
        }
        backend._last_plan_reached_goal = False
    except Exception:
        logger.exception("GlobalPlanner: failed to record backend exception")


def backend_unavailable_reason(backend: Any) -> str:
    if backend is None or getattr(backend, "available", True) is not False:
        return ""
    return str(
        getattr(backend, "_load_error", "")
        or getattr(backend, "_last_plan_error", "")
        or "planner backend unavailable"
    )
