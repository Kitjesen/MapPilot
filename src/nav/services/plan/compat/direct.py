"""Mapless planner service used by Thunder Lite compatibility profiles."""

from __future__ import annotations

import time
from typing import Any

from runtime.msgs.numpy_compat import np
from nav.services.plan.contracts import (
    GlobalPlanRequest,
    GlobalPlanResult,
    require_global_planner_backend,
)
from nav.services.plan.compat.direct_path import DirectPathBackend


class MaplessDirectPlannerService:
    """Planner service with the same boundary as GlobalPlanner.

    It intentionally does not import map artifact validation, path safety, or
    global planning kernels. Thunder Lite can instantiate Navigation
    without pulling in the complete map-backed planning stack.
    """

    def __init__(
        self,
        planner_name: str = "direct",
        tomogram: str = "",
        obstacle_thr: float = 49.9,
        downsample_dist: float = 2.0,
        plan_safety_policy: str = "off",
        fallback_planner_name: str = "",
        expected_saved_map_frame_id: str | None = None,
    ) -> None:
        self._planner_name = str(planner_name or "direct")
        self._tomogram = str(tomogram or "")
        self._obstacle_thr = float(obstacle_thr)
        self._downsample_dist = float(downsample_dist)
        self._plan_safety_policy = str(plan_safety_policy or "off")
        self._fallback_planner_name = str(fallback_planner_name or "")
        self._expected_saved_map_frame_id = expected_saved_map_frame_id
        self._backend: DirectPathBackend | None = None
        self._last_plan_report: dict[str, Any] = {}
        self._map_artifact_gate: dict[str, Any] = {
            "required": False,
            "ok": True,
            "reason": "mapless_direct",
            "blockers": [],
        }

    def setup(self) -> None:
        self._backend = require_global_planner_backend(
            self._planner_name,
            DirectPathBackend(
                tomogram_path=self._tomogram,
                obstacle_thr=self._obstacle_thr,
            ),
        )

    @property
    def planner_name(self) -> str:
        return self._planner_name

    @property
    def plan_safety_policy(self) -> str:
        return self._plan_safety_policy

    @property
    def is_ready(self) -> bool:
        return self._backend is not None

    @property
    def has_map(self) -> bool:
        return False

    @property
    def map_artifact_gate(self) -> dict[str, Any]:
        return dict(self._map_artifact_gate)

    @property
    def last_plan_report(self) -> dict[str, Any]:
        return dict(self._last_plan_report)

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        if self._backend is None:
            return GlobalPlanResult(
                error="MaplessDirectPlannerService: backend not set up",
                frame_id=request.frame_id,
                request_id=request.request_id,
                map_version=request.map_version,
            )

        t0 = time.perf_counter()
        result = self._backend.plan_request(request)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        path = result.path
        self._last_plan_report = {
            "primary_planner": self._planner_name,
            "selected_planner": self._planner_name,
            "selected_path_safety": {
                "status": "not_applicable",
                "reason": "mapless_direct",
            },
            "rejected_plans": [],
            "fallback_reason": "",
            "policy": self._plan_safety_policy,
            "reached_goal": result.reached_goal,
            "planner_diagnostics": dict(result.diagnostics),
        }
        if not path:
            if result.error:
                self._last_plan_report["fallback_reason"] = result.error
            error = result.error or "MaplessDirectPlannerService: empty path"
        else:
            error = ""
        return GlobalPlanResult(
            path=path,
            plan_ms=elapsed_ms,
            reached_goal=result.reached_goal,
            error=error,
            frame_id=request.frame_id,
            request_id=request.request_id,
            map_version=request.map_version,
            diagnostics=dict(result.diagnostics),
            report=dict(self._last_plan_report),
        )

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        safe_goal_tolerance: float = 4.0,
    ) -> tuple[list[np.ndarray], float]:
        result = self.plan_request(
            GlobalPlanRequest(
                start=start,
                goal=goal,
                safe_goal_tolerance=safe_goal_tolerance,
            )
        )
        if result.error or not result.path:
            raise RuntimeError(result.error or "MaplessDirectPlannerService: empty path")
        return result.points(), result.plan_ms

    def update_map(self, *args: Any, **kwargs: Any) -> None:
        if self._backend is not None:
            self._backend.update_map(*args, **kwargs)

    def backend_status(self) -> dict[str, Any]:
        return {
            "configured_backend": self.planner_name,
            "backend": self.planner_name,
            "fallback_backend": "",
            "degraded": False,
            "degraded_reason": "",
        }

    def reload_tomogram(self, tomogram: str) -> dict[str, Any]:
        self._tomogram = str(tomogram or "")
        if self._backend is not None:
            self._backend._tomogram_path = self._tomogram
        return {"ok": True, "backend": self.planner_name, "mode": "mapless_direct"}

    def reload_map(self, map_path: str = "") -> dict[str, Any]:
        return self.reload_tomogram(map_path)
