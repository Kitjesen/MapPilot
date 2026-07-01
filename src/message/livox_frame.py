"""Livox decoded frame contract shared by DDS and native drivers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from runtime.msgs.numpy_compat import np

POINT_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
        ("offset_time_ns", "<u4"),
        ("tag", "u1"),
        ("line", "u1"),
        ("flags", "<u2"),
    ]
)


@dataclass
class LivoxPointFrame:
    """Lossless Livox point frame normalized from official driver output."""

    points: Any
    timestamp_ns: int = 0
    sequence: int = 0

    def __post_init__(self) -> None:
        self.points = np.asarray(self.points, dtype=POINT_DTYPE)

    @property
    def point_count(self) -> int:
        return int(self.points.shape[0])

    def to_xyzi(self) -> Any:
        out = np.empty((self.point_count, 4), dtype=np.float32)
        out[:, 0] = self.points["x"]
        out[:, 1] = self.points["y"]
        out[:, 2] = self.points["z"]
        out[:, 3] = self.points["intensity"]
        return out
