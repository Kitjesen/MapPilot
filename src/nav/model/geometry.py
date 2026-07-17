"""Small geometry helpers for mission telemetry and goal comparisons."""

from __future__ import annotations

from runtime.msgs.numpy_compat import np


def point_summary(point: np.ndarray | None) -> list[float] | None:
    if point is None or len(point) < 2:
        return None
    return [
        float(point[0]),
        float(point[1]),
        float(point[2]) if len(point) > 2 else 0.0,
    ]


def distance_xy(a: np.ndarray | None, b: np.ndarray | None) -> float | None:
    if a is None or b is None or len(a) < 2 or len(b) < 2:
        return None
    av = np.asarray(a[:2], dtype=float)
    bv = np.asarray(b[:2], dtype=float)
    return round(float(np.linalg.norm(av - bv)), 3)


def distance_xyz_or_xy(a: np.ndarray, b: np.ndarray) -> float:
    av = np.asarray(a, dtype=float).reshape(-1)
    bv = np.asarray(b, dtype=float).reshape(-1)
    if av.size >= 3 and bv.size >= 3 and np.all(np.isfinite(av[:3])) and np.all(np.isfinite(bv[:3])):
        return float(np.linalg.norm(av[:3] - bv[:3]))
    return float(np.linalg.norm(av[:2] - bv[:2]))
