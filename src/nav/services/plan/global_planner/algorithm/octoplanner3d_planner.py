"""GlobalPlanner-compatible OctoPlanner3D implementation."""

from __future__ import annotations

import os
from collections.abc import Mapping, Sequence
from typing import Any

from runtime.msgs.numpy_compat import np
from nav.services.plan.contracts import GlobalPlanRequest, GlobalPlanResult

from .octoplanner3d_protocol import (
    DEFAULT_PLANNER_CONSTRAINTS,
    PLANNER_INPUT_SCHEMA,
    PLANNER_OUTPUT_SCHEMA,
    SUPPORTED_MAP_EXTENSIONS,
    SUPPORTED_MAP_FORMATS,
    as_xyz,
    build_payload,
    jsonable_point,
    map_format,
    map_source,
    normalize_constraints,
    parse_path,
    result_diagnostics,
)
from .octoplanner3d_runtime import OctoPlanner3DRuntime, RuntimeResult


class OctoPlanner3DPlanner:
    """Planner adapter around the OctoPlanner3D runtime."""

    def __init__(
        self,
        tomogram_path: str = "",
        obstacle_thr: float = 49.9,
        *,
        executable_path: str | os.PathLike[str] | None = None,
        timeout_s: float | None = None,
    ) -> None:
        self._tomogram_path = str(tomogram_path or "")
        self._map_path = self._tomogram_path
        self._obstacle_thr = float(obstacle_thr)
        self._planner_constraints = dict(DEFAULT_PLANNER_CONSTRAINTS)
        self._runtime = OctoPlanner3DRuntime(
            executable_path=executable_path,
            timeout_s=timeout_s,
        )

        self._grid = None
        self._static_grid = None
        self._costmap = None
        self._resolution = 0.2
        self._origin = np.asarray([0.0, 0.0], dtype=float)
        self._costmap_resolution = 0.2
        self._costmap_origin = np.asarray([0.0, 0.0], dtype=float)

        self._load_error = ""
        self._last_plan_error = ""
        self._last_plan_diagnostics: dict[str, Any] = {}
        self._last_plan_reached_goal = False

        self._validate_runtime()

    @property
    def planner_constraints(self) -> dict[str, Any]:
        return dict(self._planner_constraints)

    def configure_constraints(self, options: Mapping[str, Any] | None) -> None:
        self._planner_constraints.update(normalize_constraints(options))

    @property
    def available(self) -> bool:
        return self._load_error == ""

    @property
    def runtime_mode(self) -> str:
        return self._runtime.mode

    @property
    def executable_path(self) -> str:
        return self._runtime.executable_path

    def update_map(
        self,
        grid: Any,
        resolution: float = 0.2,
        origin: Any | None = None,
    ) -> None:
        grid_arr = np.asarray(grid, dtype=np.float32)
        if grid_arr.ndim != 2:
            self._last_plan_diagnostics = {
                **self._last_plan_diagnostics,
                "map_update": "ignored",
                "map_update_error": "grid must be 2-D",
            }
            return

        self._costmap = grid_arr.copy()
        self._grid = grid_arr.copy()
        self._static_grid = grid_arr.copy()
        self._resolution = float(resolution)
        self._costmap_resolution = float(resolution)
        if origin is not None:
            self._origin = np.asarray(origin, dtype=float).reshape(-1)[:2]
            self._costmap_origin = self._origin.copy()
        self._last_plan_diagnostics = {
            **self._last_plan_diagnostics,
            "map_update": "cached",
            "grid_shape": list(grid_arr.shape),
            "resolution": self._resolution,
            "origin": [float(self._origin[0]), float(self._origin[1])],
        }

    def _load_tomogram(self, tomogram_path: str) -> None:
        self._tomogram_path = str(tomogram_path or "")
        self._map_path = self._tomogram_path
        self._validate_runtime()

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        path = self.plan(request.start, request.goal)
        return GlobalPlanResult(
            path=path,
            reached_goal=self._last_plan_reached_goal,
            error=self._last_plan_error,
            frame_id=request.frame_id,
            request_id=request.request_id,
            map_version=request.map_version,
            diagnostics=dict(self._last_plan_diagnostics),
        )

    def plan(self, start: Sequence[float], goal: Sequence[float]) -> list[np.ndarray]:
        try:
            start_xyz = as_xyz(start)
            goal_xyz = as_xyz(goal)
        except (TypeError, ValueError) as exc:
            self._last_plan_error = str(exc)
            self._last_plan_reached_goal = False
            self._last_plan_diagnostics = self._base_diagnostics(
                stage="input_validation",
                available=self.available,
                error_message=self._last_plan_error,
            )
            return []

        if not self.available:
            self._last_plan_error = "octoplanner3d backend unavailable"
            self._last_plan_reached_goal = False
            self._last_plan_diagnostics = self._base_diagnostics(
                stage="unavailable",
                available=False,
                start_xyz=jsonable_point(start_xyz),
                goal_xyz=jsonable_point(goal_xyz),
                load_error=self._load_error,
            )
            return []

        payload = self._build_payload(start_xyz, goal_xyz)
        return self._handle_runtime_result(
            self._runtime.run(payload),
            start_xyz,
            goal_xyz,
        )

    def _handle_runtime_result(
        self,
        runtime_result: RuntimeResult,
        start_xyz: np.ndarray,
        goal_xyz: np.ndarray,
    ) -> list[np.ndarray]:
        if runtime_result.result is None:
            self._last_plan_error = runtime_result.error_message
            self._last_plan_reached_goal = False
            self._last_plan_diagnostics = self._base_diagnostics(
                stage=runtime_result.stage,
                available=True,
                start_xyz=jsonable_point(start_xyz),
                goal_xyz=jsonable_point(goal_xyz),
                **runtime_result.diagnostics(),
            )
            return []

        result = runtime_result.result
        if runtime_result.stage == "cxx_plan_failed":
            self._last_plan_error = runtime_result.error_message
            self._last_plan_reached_goal = False
            diagnostics = {
                **runtime_result.diagnostics(),
                **result_diagnostics(result),
            }
            self._last_plan_diagnostics = self._base_diagnostics(
                stage="cxx_plan_failed",
                available=True,
                start_xyz=jsonable_point(start_xyz),
                goal_xyz=jsonable_point(goal_xyz),
                **diagnostics,
            )
            return []

        if not bool(result.get("ok", True)):
            self._last_plan_error = str(
                result.get("error") or result.get("message") or "octoplanner3d plan failed"
            )
            self._last_plan_reached_goal = False
            self._last_plan_diagnostics = self._base_diagnostics(
                stage="cxx_plan_failed",
                available=True,
                start_xyz=jsonable_point(start_xyz),
                goal_xyz=jsonable_point(goal_xyz),
                **result_diagnostics(result),
            )
            return []

        path = parse_path(result.get("path"))
        if not path:
            self._last_plan_error = "octoplanner3d returned empty path"
            self._last_plan_reached_goal = False
            self._last_plan_diagnostics = self._base_diagnostics(
                stage="empty_path",
                available=True,
                start_xyz=jsonable_point(start_xyz),
                goal_xyz=jsonable_point(goal_xyz),
                **result_diagnostics(result),
            )
            return []

        self._last_plan_error = ""
        self._last_plan_reached_goal = bool(result.get("reached_goal", True))
        diagnostics = result_diagnostics(result)
        diagnostics.setdefault("path_points", len(path))
        diagnostics.setdefault("goal_reached", self._last_plan_reached_goal)
        self._last_plan_diagnostics = self._base_diagnostics(
            stage="cxx_plan_success",
            available=True,
            start_xyz=jsonable_point(start_xyz),
            goal_xyz=jsonable_point(goal_xyz),
            **diagnostics,
        )
        return path

    def _build_payload(self, start_xyz: np.ndarray, goal_xyz: np.ndarray) -> dict[str, Any]:
        return build_payload(
            runtime_map_path=self._runtime_map_path(),
            start_xyz=start_xyz,
            goal_xyz=goal_xyz,
            obstacle_thr=self._obstacle_thr,
            constraints=self._planner_constraints,
            grid=self._grid,
            resolution=self._resolution,
            origin=self._origin,
        )

    def _base_diagnostics(self, **extra: Any) -> dict[str, Any]:
        runtime_path = self._runtime_map_path()
        fmt = map_format(runtime_path)
        diagnostics: dict[str, Any] = {
            "planner": "octoplanner3d",
            "runtime_mode": self.runtime_mode,
            "process_boundary": self._runtime.process_boundary,
            "executable_path": self._runtime.executable_path,
            "wsl_executable_path": self._runtime.wsl_executable_path,
            "map_path": self._map_path,
            "runtime_map_path": runtime_path,
            "map_source": map_source(runtime_path, fmt),
            "map_format": fmt,
            "planner_family": "octoplanner3d_constrained_global_planner",
            "search_algorithm": "octomap_3d_astar",
            "constraints": dict(self._planner_constraints),
            "build_capabilities": {
                "octomap_file": True,
                "bt_without_pcl": True,
                "generic_octomap_read": True,
                "pcd_conversion": "requires PCL-enabled headless build",
                "ros2_required": False,
                "headless_executable": self._runtime.has_headless_executable,
            },
            "supported_map_formats": list(SUPPORTED_MAP_FORMATS),
            "supported_map_extensions": list(SUPPORTED_MAP_EXTENSIONS),
            "input_schema": PLANNER_INPUT_SCHEMA,
            "output_schema": PLANNER_OUTPUT_SCHEMA,
            "timeout_s": self._runtime.timeout_s,
        }
        diagnostics.update(extra)
        return diagnostics

    def _validate_runtime(self) -> None:
        self._load_error = "; ".join(
            self._runtime.validate_map(self._map_path, SUPPORTED_MAP_EXTENSIONS)
        )

    def _runtime_map_path(self) -> str:
        return self._runtime.runtime_map_path(self._map_path)
