"""Path post-processing for planner service output."""

from __future__ import annotations

from runtime.msgs.numpy_compat import np


def downsample_path(path: list, goal: np.ndarray, min_distance: float) -> list[np.ndarray]:
    if not path:
        return []
    result = [np.asarray(path[0][:3], dtype=float)]
    for point in path[1:]:
        pt = (
            np.asarray(point[:3], dtype=float)
            if len(point) >= 3
            else np.asarray([point[0], point[1], 0.0], dtype=float)
        )
        if np.linalg.norm(pt - result[-1]) >= float(min_distance):
            result.append(pt)
    goal_pt = (
        np.asarray(goal[:3], dtype=float)
        if len(goal) >= 3
        else np.asarray([goal[0], goal[1], 0.0], dtype=float)
    )
    if np.linalg.norm(goal_pt - result[-1]) > 1e-6:
        result.append(goal_pt)
    return result
