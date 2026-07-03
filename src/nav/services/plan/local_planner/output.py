"""Output, direct-track fallback, and health helpers for LocalPlanner."""

from __future__ import annotations

import math
import time
from typing import Any

from nav.services.plan.local_planner.geometry import (
    result_path_xy_metrics,
    straight_line,
)
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import Path
from runtime.msgs.numpy_compat import np


class LocalPlannerOutputMixin:
    @staticmethod
    def _result_path_xy_metrics(path: list[Any]) -> tuple[float, float]:
        return result_path_xy_metrics(path)

    def _straight_line(self, start: np.ndarray, goal: np.ndarray, step: float = 0.5):
        return straight_line(start, goal, step=step)

    def _publish_control_hint(
        self,
        *,
        slow_down: int = 0,
        near_field_stop: bool = False,
        safety_stop: bool | None = None,
        safety_stop_level: int | None = None,
        path_found: bool | None = None,
        recovery_state: int | None = None,
        reason: str = "",
    ) -> None:
        slow_down = max(0, min(3, int(slow_down or 0)))
        near_field_stop = bool(near_field_stop)
        if safety_stop is None:
            stop = near_field_stop and not self._ignore_near_field_stop
        else:
            stop = bool(safety_stop)
        if safety_stop_level is None:
            level = 2 if stop else 0
        else:
            level = max(0, min(2, int(safety_stop_level or 0)))
            if level > 0:
                stop = True
        payload: dict[str, Any] = {
            "ts": time.time(),
            "source": "nav.local_planner",
            "slow_down": slow_down,
            "near_field_stop": near_field_stop,
            "safety_stop": stop,
            "safety_stop_level": level,
            "reason": reason,
        }
        traversability = getattr(self, "_traversability_summary", lambda: {})()
        if traversability:
            payload["traversability"] = traversability
        frame_status = getattr(self, "_last_frame_status", None)
        if frame_status:
            payload["frame_status"] = dict(frame_status)
        if path_found is not None:
            payload["path_found"] = bool(path_found)
        if recovery_state is not None:
            payload["recovery_state"] = int(recovery_state)
        self._last_control_hint = dict(payload)
        self.control_hint.publish(payload)

    def _publish_local_path(self, poses: list[PoseStamped]) -> None:
        self._last_local_path_points = len(poses)
        if len(poses) >= 2:
            start = poses[0].pose.position
            end = poses[-1].pose.position
            self._last_local_path_span_m = float(
                math.hypot(end.x - start.x, end.y - start.y)
            )
        else:
            self._last_local_path_span_m = 0.0
        self.local_path.publish(Path(poses=poses, frame_id=self._path_frame_id))

    def _publish_direct_track_fallback(
        self,
        *,
        near_field_stop: bool,
        path_found: bool,
        recovery_state: int,
        reason: str,
    ) -> bool:
        if not self._allow_direct_track_fallback:
            return False
        if near_field_stop:
            if not self._ignore_near_field_stop:
                return False
            if not path_found:
                return False
        wp = self._latest_waypoint
        if wp is None:
            return False
        goal = self._effective_goal(wp)
        if not np.all(np.isfinite(goal[:2])):
            return False
        origin = self._planning_origin()
        span = float(np.linalg.norm(goal[:2] - origin[:2]))
        if span < max(0.0, self._direct_track_fallback_min_distance_m):
            return False
        path_points = self._straight_line(origin, goal, step=0.5)
        if len(path_points) < 2:
            return False
        if not self._direct_track_fallback_clear(origin, goal):
            return False
        poses = [
            PoseStamped(
                pose=Pose(
                    position=Vector3(float(p[0]), float(p[1]), float(p[2])),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
                frame_id=self._path_frame_id,
            )
            for p in path_points
        ]
        self._last_direct_track_fallback_ts = time.time()
        self._publish_control_hint(
            near_field_stop=near_field_stop,
            safety_stop=False,
            path_found=path_found,
            recovery_state=recovery_state,
            reason=f"direct_track_fallback:{reason}",
        )
        self._publish_local_path(poses)
        return True

    def health(self) -> dict[str, Any]:
        info = self.port_summary()
        core_paths_loaded = False
        if self._core is not None and hasattr(self._core, "paths_loaded"):
            core_paths_loaded = bool(self._core.paths_loaded())
        cmu_py_ready = self._backend == "cmu_py" and self._path_data is not None
        info["local_planner"] = {
            **self._backend_status.as_health_fields(),
            "paths_loaded":  self._path_data is not None or core_paths_loaded,
            "terrain_pts":   (self._terrain_points.shape[0]
                              if self._terrain_points is not None else 0),
            "running":       (self._core is not None) or cmu_py_ready,
            "direct_track_fallback_enabled": self._allow_direct_track_fallback,
            "direct_track_fallback_clearance_m": round(
                self._direct_track_fallback_clearance_m, 3,
            ),
            "min_trackable_local_path_m": round(self._min_trackable_local_path_xy, 3),
            "last_direct_track_fallback_age_ms": (
                round((time.time() - self._last_direct_track_fallback_ts) * 1000)
                if self._last_direct_track_fallback_ts > 0
                else None
            ),
            "last_control_hint": dict(self._last_control_hint),
            "last_local_path_points": self._last_local_path_points,
            "last_local_path_span_m": round(self._last_local_path_span_m, 3),
            "last_result": dict(self._last_result_diagnostics),
            "frame_status": dict(getattr(self, "_last_frame_status", {}) or {}),
            "odometry_transform": dict(
                getattr(self, "_last_odometry_transform_status", {}) or {}
            ),
            "traversability": getattr(self, "_traversability_summary", lambda: {})(),
            "traversability_grid": dict(
                getattr(self, "_last_traversability_grid_status", {}) or {}
            ),
            "effective_params": dict(self._effective_local_planner_params),
        }
        return info

    def _direct_track_fallback_clear(self, start: np.ndarray, goal: np.ndarray) -> bool:
        obstacles = self._merge_obstacle_clouds()
        if obstacles.shape[0] == 0:
            return True
        a = np.asarray(start[:2], dtype=float)
        b = np.asarray(goal[:2], dtype=float)
        seg = b - a
        denom = float(np.dot(seg, seg))
        if denom <= 1e-9:
            return False
        pts = np.asarray(obstacles[:, :2], dtype=float)
        if pts.size == 0:
            return True
        t = np.clip(((pts - a) @ seg) / denom, 0.0, 1.0)
        closest = a + t[:, None] * seg
        dists = np.linalg.norm(pts - closest, axis=1)
        return bool(np.min(dists) > self._direct_track_fallback_clearance_m)
