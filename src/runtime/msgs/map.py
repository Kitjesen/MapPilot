"""Map data messages for spatial map streams.

These messages describe map data after a SLAM or map-building component has
already interpreted raw sensor input. They are transport-agnostic and can move
over local callbacks, SHM, DDS, LCM, or another adapter.
"""

from __future__ import annotations

import json
import struct
import time
from dataclasses import dataclass, field
from typing import Any, ClassVar

from runtime.runtime_interface import map_frame_id

from .numpy_compat import is_numpy_array, np, numpy_import_is_safe
from .sensor import PointCloud2

MAP_FRAME_ID = map_frame_id()
MAP_CLOUD_FRAME_SCHEMA = "map.cloud_frame"
VALID_MAP_CLOUD_MODES = frozenset({"FULL", "KEYFRAME", "INCREMENTAL"})

_HEADER_LEN = struct.Struct("<I")


@dataclass
class MapCloudFrame:
    """Map point-cloud frame.

    ``PointCloud2`` remains the legacy point-cloud compatibility shape.
    ``MapCloudFrame`` is the map data-plane shape: callers say whether the
    points are a complete map replacement, a keyframe, or an incremental update.
    """

    msg_name: ClassVar[str] = "map.MapCloudFrame"

    points: Any = field(default_factory=list)
    mode: str = "FULL"
    ts: float = field(default_factory=time.time)
    frame_id: str = field(default=MAP_FRAME_ID)
    map_id: str = ""
    source: str = ""
    sequence: int = 0
    metadata: dict[str, Any] = field(default_factory=dict)
    schema_version: str = MAP_CLOUD_FRAME_SCHEMA

    def __post_init__(self) -> None:
        if self.schema_version != MAP_CLOUD_FRAME_SCHEMA:
            raise ValueError(
                f"schema_version must be {MAP_CLOUD_FRAME_SCHEMA!r}, "
                f"got {self.schema_version!r}"
            )

        self.mode = str(self.mode or "FULL").strip().upper()
        if self.mode not in VALID_MAP_CLOUD_MODES:
            valid = ", ".join(sorted(VALID_MAP_CLOUD_MODES))
            raise ValueError(f"mode must be one of {valid}, got {self.mode!r}")

        self.frame_id = str(self.frame_id or MAP_FRAME_ID)
        self.map_id = str(self.map_id or "")
        self.source = str(self.source or "")
        self.sequence = int(self.sequence or 0)
        if not isinstance(self.metadata, dict):
            raise TypeError("metadata must be a dict")
        if not self.ts:
            self.ts = time.time()

        if not is_numpy_array(self.points):
            if self.points or numpy_import_is_safe():
                self.points = np.asarray(self.points, dtype=np.float32)
        if is_numpy_array(self.points):
            if self.points.ndim == 1 and self.points.size == 0:
                self.points = self.points.reshape(0, 3)
            if self.points.ndim != 2 or self.points.shape[1] < 3:
                raise ValueError(
                    "points must have shape Nx3 or wider, "
                    f"got {self.points.shape}"
                )
            self.points = self.points.astype(np.float32, copy=False)
        elif self.points:
            raise ImportError("NumPy import is unsafe in this host interpreter")

    @property
    def num_points(self) -> int:
        if is_numpy_array(self.points):
            return int(self.points.shape[0])
        return len(self.points) if isinstance(self.points, list) else 0

    @property
    def point_columns(self) -> int:
        if is_numpy_array(self.points) and self.points.ndim == 2:
            return int(self.points.shape[1])
        return 3

    def finite_xyz(self, *, max_abs: float = 500.0) -> np.ndarray:
        """Return finite XYZ points in map frame for map builders."""
        pts = np.asarray(self.points, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] < 3:
            return np.empty((0, 3), dtype=np.float32)
        xyz = pts[:, :3]
        valid = np.isfinite(xyz).all(axis=1) & (np.abs(xyz) < max_abs).all(axis=1)
        return xyz[valid].astype(np.float32, copy=True)

    def to_pointcloud2(self) -> PointCloud2:
        """Convert to the compatibility point-cloud message."""
        return PointCloud2.from_numpy(
            np.asarray(self.points, dtype=np.float32),
            frame_id=self.frame_id,
            ts=self.ts,
        )

    @classmethod
    def from_pointcloud2(
        cls,
        cloud: PointCloud2,
        *,
        mode: str = "FULL",
        map_id: str = "",
        source: str = "pointcloud2",
        sequence: int = 0,
        metadata: dict[str, Any] | None = None,
    ) -> MapCloudFrame:
        return cls(
            points=cloud.points,
            mode=mode,
            ts=float(getattr(cloud, "ts", 0.0) or time.time()),
            frame_id=str(getattr(cloud, "frame_id", MAP_FRAME_ID) or MAP_FRAME_ID),
            map_id=map_id,
            source=source,
            sequence=sequence,
            metadata=dict(metadata or {}),
        )

    def to_dict(self, *, include_points: bool = False) -> dict[str, Any]:
        data: dict[str, Any] = {
            "schema_version": self.schema_version,
            "mode": self.mode,
            "ts": float(self.ts),
            "frame_id": self.frame_id,
            "map_id": self.map_id,
            "source": self.source,
            "sequence": int(self.sequence),
            "metadata": dict(self.metadata),
            "num_points": self.num_points,
            "point_columns": self.point_columns,
        }
        if include_points:
            data["points"] = (
                self.points.tolist() if is_numpy_array(self.points) else self.points
            )
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> MapCloudFrame:
        return cls(
            points=data.get("points", []),
            mode=str(data.get("mode", "FULL")),
            ts=float(data.get("ts", 0.0) or time.time()),
            frame_id=str(data.get("frame_id", MAP_FRAME_ID) or MAP_FRAME_ID),
            map_id=str(data.get("map_id", "") or ""),
            source=str(data.get("source", "") or ""),
            sequence=int(data.get("sequence", 0) or 0),
            metadata=dict(data.get("metadata") or {}),
            schema_version=str(data.get("schema_version", MAP_CLOUD_FRAME_SCHEMA)),
        )

    def encode(self) -> bytes:
        points = np.asarray(self.points, dtype=np.float32)
        header = {
            "schema_version": self.schema_version,
            "mode": self.mode,
            "ts": float(self.ts),
            "frame_id": self.frame_id,
            "map_id": self.map_id,
            "source": self.source,
            "sequence": int(self.sequence),
            "metadata": dict(self.metadata),
            "point_shape": [int(points.shape[0]), int(points.shape[1])],
            "point_dtype": "float32",
        }
        header_bytes = json.dumps(
            header,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        return (
            _HEADER_LEN.pack(len(header_bytes))
            + header_bytes
            + points.astype("<f4", copy=False).tobytes()
        )

    @classmethod
    def decode(cls, data: bytes) -> MapCloudFrame:
        if len(data) < _HEADER_LEN.size:
            raise ValueError("encoded MapCloudFrame is too short")
        header_len = _HEADER_LEN.unpack_from(data, 0)[0]
        start = _HEADER_LEN.size
        end = start + header_len
        if len(data) < end:
            raise ValueError("encoded MapCloudFrame header is truncated")
        header = json.loads(data[start:end].decode("utf-8"))
        rows, cols = header.get("point_shape", [0, 3])
        rows = int(rows)
        cols = int(cols)
        if rows < 0 or cols < 3:
            raise ValueError(f"invalid point_shape: {[rows, cols]}")
        expected_bytes = rows * cols * 4
        payload = data[end:]
        if len(payload) != expected_bytes:
            raise ValueError(
                f"point payload size mismatch: expected {expected_bytes}, "
                f"got {len(payload)}"
            )
        points = np.frombuffer(payload, dtype="<f4").reshape(rows, cols).copy()
        return cls(
            points=points,
            mode=str(header.get("mode", "FULL")),
            ts=float(header.get("ts", 0.0) or time.time()),
            frame_id=str(header.get("frame_id", MAP_FRAME_ID) or MAP_FRAME_ID),
            map_id=str(header.get("map_id", "") or ""),
            source=str(header.get("source", "") or ""),
            sequence=int(header.get("sequence", 0) or 0),
            metadata=dict(header.get("metadata") or {}),
            schema_version=str(header.get("schema_version", MAP_CLOUD_FRAME_SCHEMA)),
        )
