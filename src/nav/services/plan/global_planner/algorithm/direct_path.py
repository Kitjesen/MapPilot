"""Mapless direct-path planner for lightweight local runtimes."""

from __future__ import annotations

from collections.abc import Sequence

from runtime.msgs.numpy_compat import np
from runtime.registry import register
from nav.services.plan.contracts import GlobalPlanRequest, GlobalPlanResult


def _as_xyz(value: Sequence[float]) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size < 2:
        raise ValueError("point must contain at least x and y")
    z = float(arr[2]) if arr.size >= 3 else 0.0
    return np.asarray([float(arr[0]), float(arr[1]), z], dtype=float)


@register(
    "planner_backend",
    "direct",
    description="Mapless direct-path planner for Thunder Lite local runtime",
)
class DirectPathBackend:
    """Mapless planner for lightweight local deployments."""

    def __init__(self, tomogram_path: str = "", obstacle_thr: float = 49.9) -> None:
        self._tomogram_path = str(tomogram_path or "")
        self._obstacle_thr = float(obstacle_thr)
        self._grid = None
        self._resolution = 0.2
        self._origin = [0.0, 0.0]
        self._last_plan_error = ""
        self._last_plan_diagnostics: dict[str, object] = {}
        self._last_plan_reached_goal = False

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        try:
            start_pt = _as_xyz(request.start)
            goal_pt = _as_xyz(request.goal)
        except (TypeError, ValueError) as exc:
            self._last_plan_error = str(exc)
            self._last_plan_reached_goal = False
            self._last_plan_diagnostics = {
                "mode": "direct",
                "map_required": False,
                "error": self._last_plan_error,
            }
            return GlobalPlanResult(
                error=self._last_plan_error,
                frame_id=request.frame_id,
                request_id=request.request_id,
                map_version=request.map_version,
                diagnostics=dict(self._last_plan_diagnostics),
            )

        if not np.all(np.isfinite(start_pt)) or not np.all(np.isfinite(goal_pt)):
            self._last_plan_error = "start or goal contains non-finite values"
            self._last_plan_reached_goal = False
            self._last_plan_diagnostics = {
                "mode": "direct",
                "map_required": False,
                "error": self._last_plan_error,
            }
            return GlobalPlanResult(
                error=self._last_plan_error,
                frame_id=request.frame_id,
                request_id=request.request_id,
                map_version=request.map_version,
                diagnostics=dict(self._last_plan_diagnostics),
            )

        distance_m = float(np.linalg.norm(goal_pt[:2] - start_pt[:2]))
        self._last_plan_error = ""
        self._last_plan_reached_goal = True
        self._last_plan_diagnostics = {
            "mode": "direct",
            "map_required": False,
            "distance_m": round(distance_m, 4),
        }
        if distance_m <= 1e-6:
            path = [goal_pt]
        else:
            path = [start_pt, goal_pt]
        return GlobalPlanResult(
            path=path,
            reached_goal=True,
            frame_id=request.frame_id,
            request_id=request.request_id,
            map_version=request.map_version,
            diagnostics=dict(self._last_plan_diagnostics),
        )

    def plan(self, start: Sequence[float], goal: Sequence[float]) -> list[np.ndarray]:
        return self.plan_request(
            GlobalPlanRequest(start=np.asarray(start), goal=np.asarray(goal))
        ).points()

    def update_map(self, *_args, **_kwargs) -> None:
        """Accept live-map updates without making Lite depend on map modules."""

        self._last_plan_diagnostics = {
            **self._last_plan_diagnostics,
            "map_update": "ignored",
            "map_required": False,
        }
