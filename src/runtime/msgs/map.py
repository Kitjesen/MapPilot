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

from .geometry import Pose, Transform, Vector3
from .numpy_compat import is_numpy_array, np, numpy_import_is_safe
from .sensor import PointCloud2

MAP_FRAME_ID = map_frame_id()
MAP_CLOUD_FRAME_SCHEMA = "map.cloud_frame"
MAP_OBSERVATION_FRAME_SCHEMA = "map.observation_frame"
SEMANTIC_LABELS_FRAME_SCHEMA = "map.semantic_labels_frame"
SEMANTIC_SAVE_REQUEST_SCHEMA = "map.semantic_save_request"
SEMANTIC_SAVE_RESULT_SCHEMA = "map.semantic_save_result"
MAP_SCENE_FRAME_SCHEMA = "map.scene_frame"
MAP_CONTROL_REQUEST_SCHEMA = "map.control_request.v1"
VALID_MAP_CLOUD_MODES = frozenset({"FULL", "KEYFRAME", "INCREMENTAL"})
VALID_MAP_OBSERVATION_KINDS = frozenset({"INCREMENTAL"})

_HEADER_LEN = struct.Struct("<I")


def _decode_json_message(data: bytes, name: str) -> dict[str, Any]:
    if len(data) < _HEADER_LEN.size:
        raise ValueError(f"encoded {name} is too short")
    payload_size = _HEADER_LEN.unpack_from(data, 0)[0]
    payload = data[_HEADER_LEN.size :]
    if len(payload) != payload_size:
        raise ValueError(f"encoded {name} payload size mismatch")
    decoded = json.loads(payload.decode("utf-8"))
    if not isinstance(decoded, dict):
        raise ValueError(f"encoded {name} payload must be an object")
    return decoded


@dataclass(frozen=True)
class MapControlRequest:
    """Typed control-plane request for the native maps service.

    The runtime previously moved map commands as unvalidated JSON strings.
    Keeping the operation and its parameters in a message object makes local,
    SHM, and DDS adapters share one contract while the native service retains
    ownership of validation and persistent state.
    """

    msg_name: ClassVar[str] = "map.MapControlRequest"

    action: str
    params: dict[str, Any] = field(default_factory=dict)
    request_id: str = ""
    schema_version: str = MAP_CONTROL_REQUEST_SCHEMA

    def __post_init__(self) -> None:
        action = str(self.action or "").strip()
        if not action:
            raise ValueError("map control action is required")
        if not isinstance(self.params, dict):
            raise TypeError("map control params must be a dict")
        if self.schema_version != MAP_CONTROL_REQUEST_SCHEMA:
            raise ValueError(f"schema_version must be {MAP_CONTROL_REQUEST_SCHEMA!r}")
        object.__setattr__(self, "action", action)
        object.__setattr__(self, "request_id", str(self.request_id or ""))
        object.__setattr__(self, "params", dict(self.params))

    @classmethod
    def from_mapping(cls, value: dict[str, Any]) -> MapControlRequest:
        """Build a request from an API or CLI mapping without JSON transport."""
        if not isinstance(value, dict):
            raise TypeError("map control request must be a mapping")
        payload = dict(value)
        action = str(payload.pop("action", "") or "")
        request_id = str(payload.pop("request_id", "") or payload.get("idempotency_key", "") or "")
        payload.pop("schema_version", None)
        return cls(action=action, params=payload, request_id=request_id)

    def to_mapping(self) -> dict[str, Any]:
        """Return the command-router view used by the native adapter shell."""
        payload = dict(self.params)
        payload["action"] = self.action
        if self.request_id:
            payload.setdefault("request_id", self.request_id)
        return payload

    def encode(self) -> bytes:
        """Encode this request for a routed transport."""
        payload = json.dumps(
            {
                "schema_version": self.schema_version,
                "action": self.action,
                "request_id": self.request_id,
                "params": self.params,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        return _HEADER_LEN.pack(len(payload)) + payload

    @classmethod
    def decode(cls, data: bytes) -> MapControlRequest:
        """Decode and validate one routed control request."""
        payload = _decode_json_message(data, "MapControlRequest")
        if payload.get("schema_version") != MAP_CONTROL_REQUEST_SCHEMA:
            raise ValueError("encoded MapControlRequest schema is unsupported")
        params = payload.get("params")
        if not isinstance(params, dict):
            raise ValueError("encoded MapControlRequest params must be an object")
        return cls(
            action=str(payload.get("action") or ""),
            params=params,
            request_id=str(payload.get("request_id") or ""),
        )


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
            raise ValueError(f"schema_version must be {MAP_CLOUD_FRAME_SCHEMA!r}, got {self.schema_version!r}")

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
                raise ValueError(f"points must have shape Nx3 or wider, got {self.points.shape}")
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
            data["points"] = self.points.tolist() if is_numpy_array(self.points) else self.points
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
        return _HEADER_LEN.pack(len(header_bytes)) + header_bytes + points.astype("<f4", copy=False).tobytes()

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
            raise ValueError(f"point payload size mismatch: expected {expected_bytes}, got {len(payload)}")
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


@dataclass
class MapObservationFrame:
    """Accepted incremental LiDAR scan with its exact map-frame sensor pose."""

    msg_name: ClassVar[str] = "map.MapObservationFrame"

    points: Any = field(default_factory=list)
    sequence: int = 0
    ts: float = field(default_factory=time.time)
    frame_id: str = field(default=MAP_FRAME_ID)
    sensor_frame_id: str = "body"
    sensor_origin: Vector3 = field(default_factory=Vector3)
    map_sensor_pose: Pose = field(default_factory=Pose)
    map_sensor_transform: Transform = field(default_factory=Transform)
    pose_quality: dict[str, Any] = field(default_factory=dict)
    source: str = ""
    observation_kind: str = "INCREMENTAL"
    metadata: dict[str, Any] = field(default_factory=dict)
    schema_version: str = MAP_OBSERVATION_FRAME_SCHEMA

    def __post_init__(self) -> None:
        if self.schema_version != MAP_OBSERVATION_FRAME_SCHEMA:
            raise ValueError(f"schema_version must be {MAP_OBSERVATION_FRAME_SCHEMA!r}, got {self.schema_version!r}")
        self.observation_kind = str(self.observation_kind or "").strip().upper()
        if self.observation_kind not in VALID_MAP_OBSERVATION_KINDS:
            raise ValueError("MapObservationFrame only accepts INCREMENTAL data")
        self.sequence = int(self.sequence)
        if self.sequence <= 0:
            raise ValueError("sequence must be a positive incremental scan sequence")
        self.ts = float(self.ts or time.time())
        if not np.isfinite(self.ts):
            raise ValueError("ts must be finite")
        self.frame_id = str(self.frame_id or MAP_FRAME_ID)
        self.sensor_frame_id = str(self.sensor_frame_id or "")
        if not self.frame_id or not self.sensor_frame_id:
            raise ValueError("frame_id and sensor_frame_id are required")
        self.source = str(self.source or "")
        if not isinstance(self.pose_quality, dict):
            raise TypeError("pose_quality must be a dict")
        if not isinstance(self.metadata, dict):
            raise TypeError("metadata must be a dict")

        self.sensor_origin = (
            self.sensor_origin if isinstance(self.sensor_origin, Vector3) else Vector3.from_dict(self.sensor_origin)
        )
        self.map_sensor_pose = (
            self.map_sensor_pose if isinstance(self.map_sensor_pose, Pose) else Pose.from_dict(self.map_sensor_pose)
        )
        self.map_sensor_transform = (
            self.map_sensor_transform
            if isinstance(self.map_sensor_transform, Transform)
            else Transform.from_dict(self.map_sensor_transform)
        )
        if (
            self.map_sensor_transform.frame_id != self.frame_id
            or self.map_sensor_transform.child_frame_id != self.sensor_frame_id
        ):
            raise ValueError("map_sensor_transform frame ids must match the observation")
        origin = np.asarray(
            [self.sensor_origin.x, self.sensor_origin.y, self.sensor_origin.z],
            dtype=np.float64,
        )
        translation = np.asarray(
            [
                self.map_sensor_transform.translation.x,
                self.map_sensor_transform.translation.y,
                self.map_sensor_transform.translation.z,
            ],
            dtype=np.float64,
        )
        pose_translation = np.asarray(
            [
                self.map_sensor_pose.position.x,
                self.map_sensor_pose.position.y,
                self.map_sensor_pose.position.z,
            ],
            dtype=np.float64,
        )
        transform_quaternion = np.asarray(
            [
                self.map_sensor_transform.rotation.x,
                self.map_sensor_transform.rotation.y,
                self.map_sensor_transform.rotation.z,
                self.map_sensor_transform.rotation.w,
            ],
            dtype=np.float64,
        )
        pose_quaternion = np.asarray(
            [
                self.map_sensor_pose.orientation.x,
                self.map_sensor_pose.orientation.y,
                self.map_sensor_pose.orientation.z,
                self.map_sensor_pose.orientation.w,
            ],
            dtype=np.float64,
        )
        if not (
            np.isfinite(origin).all()
            and np.isfinite(translation).all()
            and np.isfinite(pose_translation).all()
            and np.isfinite(transform_quaternion).all()
            and np.isfinite(pose_quaternion).all()
        ):
            raise ValueError("map_sensor pose and sensor origin must be finite")
        transform_norm = float(np.linalg.norm(transform_quaternion))
        pose_norm = float(np.linalg.norm(pose_quaternion))
        if transform_norm <= 1e-9 or pose_norm <= 1e-9:
            raise ValueError("map_sensor quaternion must be non-zero")
        transform_quaternion /= transform_norm
        pose_quaternion /= pose_norm
        if not (
            np.allclose(origin, translation, atol=1e-5)
            and np.allclose(pose_translation, translation, atol=1e-5)
            and abs(float(np.dot(transform_quaternion, pose_quaternion))) >= 1.0 - 1e-5
        ):
            raise ValueError("sensor origin, pose, and transform must describe one transform")

        if not is_numpy_array(self.points):
            if self.points or numpy_import_is_safe():
                self.points = np.asarray(self.points, dtype=np.float32)
        if is_numpy_array(self.points):
            if self.points.ndim != 2 or self.points.shape[1] not in (3, 4):
                raise ValueError(f"points must have geometry-only Nx3 or XYZI Nx4 shape, got {self.points.shape}")
            if self.points.shape[0] <= 0:
                raise ValueError("points must contain at least one accepted scan point")
            self.points = self.points.astype(np.float32, copy=False)
            if not np.isfinite(self.points).all():
                raise ValueError("points must be finite")
        elif self.points:
            raise ImportError("NumPy import is unsafe in this host interpreter")
        else:
            raise ValueError("points must contain at least one accepted scan point")

    @property
    def num_points(self) -> int:
        return int(self.points.shape[0]) if is_numpy_array(self.points) else 0

    @property
    def point_columns(self) -> int:
        return int(self.points.shape[1]) if is_numpy_array(self.points) else 3

    def map_points(self) -> np.ndarray:
        """Transform the accepted sensor-frame scan into the map frame."""
        points = np.asarray(self.points, dtype=np.float32)
        q = self.map_sensor_transform.rotation
        quat = np.asarray([q.x, q.y, q.z, q.w], dtype=np.float64)
        quat /= np.linalg.norm(quat)
        x, y, z, w = (float(value) for value in quat)
        rotation = np.asarray(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
        translation = np.asarray(
            [
                self.map_sensor_transform.translation.x,
                self.map_sensor_transform.translation.y,
                self.map_sensor_transform.translation.z,
            ],
            dtype=np.float64,
        )
        xyz = points[:, :3].astype(np.float64, copy=False) @ rotation.T + translation
        mapped = points.copy()
        mapped[:, :3] = xyz.astype(np.float32)
        return mapped

    def to_map_pointcloud2(self) -> PointCloud2:
        return PointCloud2.from_numpy(self.map_points(), frame_id=self.frame_id, ts=self.ts)

    def to_dict(self, *, include_points: bool = False) -> dict[str, Any]:
        data: dict[str, Any] = {
            "schema_version": self.schema_version,
            "observation_kind": self.observation_kind,
            "sequence": int(self.sequence),
            "ts": float(self.ts),
            "frame_id": self.frame_id,
            "sensor_frame_id": self.sensor_frame_id,
            "sensor_origin": self.sensor_origin.to_dict(),
            "map_sensor_pose": self.map_sensor_pose.to_dict(),
            "map_sensor_transform": self.map_sensor_transform.to_dict(),
            "pose_quality": dict(self.pose_quality),
            "source": self.source,
            "metadata": dict(self.metadata),
            "num_points": self.num_points,
            "point_columns": self.point_columns,
        }
        if include_points:
            data["points"] = self.points.tolist()
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> MapObservationFrame:
        return cls(
            points=data.get("points", []),
            sequence=int(data.get("sequence", 0) or 0),
            ts=float(data.get("ts", 0.0) or time.time()),
            frame_id=str(data.get("frame_id", MAP_FRAME_ID) or MAP_FRAME_ID),
            sensor_frame_id=str(data.get("sensor_frame_id", "") or ""),
            sensor_origin=Vector3.from_dict(data.get("sensor_origin", {})),
            map_sensor_pose=Pose.from_dict(data.get("map_sensor_pose", {})),
            map_sensor_transform=Transform.from_dict(data.get("map_sensor_transform", {})),
            pose_quality=dict(data.get("pose_quality") or {}),
            source=str(data.get("source", "") or ""),
            observation_kind=str(data.get("observation_kind", "INCREMENTAL")),
            metadata=dict(data.get("metadata") or {}),
            schema_version=str(data.get("schema_version", MAP_OBSERVATION_FRAME_SCHEMA)),
        )

    def encode(self) -> bytes:
        points = np.asarray(self.points, dtype=np.float32)
        header = self.to_dict(include_points=False)
        header["point_shape"] = [int(points.shape[0]), int(points.shape[1])]
        header["point_dtype"] = "float32"
        header_bytes = json.dumps(
            header,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        return _HEADER_LEN.pack(len(header_bytes)) + header_bytes + points.astype("<f4", copy=False).tobytes()

    @classmethod
    def decode(cls, data: bytes) -> MapObservationFrame:
        if len(data) < _HEADER_LEN.size:
            raise ValueError("encoded MapObservationFrame is too short")
        header_len = _HEADER_LEN.unpack_from(data, 0)[0]
        start = _HEADER_LEN.size
        end = start + header_len
        if len(data) < end:
            raise ValueError("encoded MapObservationFrame header is truncated")
        header = json.loads(data[start:end].decode("utf-8"))
        rows, cols = header.get("point_shape", [0, 3])
        rows = int(rows)
        cols = int(cols)
        if rows <= 0 or cols not in (3, 4):
            raise ValueError(f"invalid point_shape: {[rows, cols]}")
        expected_bytes = rows * cols * 4
        payload = data[end:]
        if len(payload) != expected_bytes:
            raise ValueError(f"point payload size mismatch: expected {expected_bytes}, got {len(payload)}")
        header["points"] = np.frombuffer(payload, dtype="<f4").reshape(rows, cols).copy()
        return cls.from_dict(header)


@dataclass
class SemanticLabelsFrame:
    """Strict per-point uint16 semantic labels for one accepted scan."""

    msg_name: ClassVar[str] = "map.SemanticLabelsFrame"

    labels: Any = field(default_factory=list)
    confidence: Any = field(default_factory=list)
    sequence: int = 0
    ts: float = field(default_factory=time.time)
    frame_id: str = ""
    taxonomy: str = ""
    taxonomy_version: int = 0
    source: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)
    schema_version: str = SEMANTIC_LABELS_FRAME_SCHEMA

    def __post_init__(self) -> None:
        if self.schema_version != SEMANTIC_LABELS_FRAME_SCHEMA:
            raise ValueError(f"schema_version must be {SEMANTIC_LABELS_FRAME_SCHEMA!r}, got {self.schema_version!r}")
        self.sequence = int(self.sequence)
        self.ts = float(self.ts)
        self.frame_id = str(self.frame_id or "")
        self.taxonomy = str(self.taxonomy or "")
        self.taxonomy_version = int(self.taxonomy_version)
        self.source = str(self.source or "")
        if self.sequence <= 0:
            raise ValueError("sequence must be positive")
        if not np.isfinite(self.ts):
            raise ValueError("ts must be finite")
        if not self.frame_id:
            raise ValueError("frame_id is required")
        if not self.taxonomy or self.taxonomy_version <= 0:
            raise ValueError("taxonomy and positive taxonomy_version are required")
        if not isinstance(self.metadata, dict):
            raise TypeError("metadata must be a dict")

        raw_labels = np.asarray(self.labels)
        if raw_labels.ndim != 1 or raw_labels.size <= 0:
            raise ValueError("labels must be a non-empty one-dimensional uint16 array")
        if not np.issubdtype(raw_labels.dtype, np.integer):
            if not np.isfinite(raw_labels).all() or not np.equal(raw_labels, np.floor(raw_labels)).all():
                raise ValueError("labels must contain uint16 integer values")
        if (raw_labels < 0).any() or (raw_labels > 65535).any():
            raise ValueError("labels must contain uint16 values in [0, 65535]")
        self.labels = raw_labels.astype(np.uint16, copy=False)

        raw_confidence = np.asarray(self.confidence, dtype=np.float32)
        if raw_confidence.size == 0:
            self.confidence = np.empty((0,), dtype=np.float32)
        else:
            if raw_confidence.ndim != 1 or raw_confidence.size != self.labels.size:
                raise ValueError("confidence must be empty or match the label count")
            if not np.isfinite(raw_confidence).all() or ((raw_confidence < 0.0) | (raw_confidence > 1.0)).any():
                raise ValueError("confidence values must be finite and within [0, 1]")
            self.confidence = raw_confidence

    @property
    def num_labels(self) -> int:
        return int(self.labels.size)

    def to_dict(self, *, include_labels: bool = False) -> dict[str, Any]:
        data: dict[str, Any] = {
            "schema_version": self.schema_version,
            "sequence": self.sequence,
            "ts": self.ts,
            "frame_id": self.frame_id,
            "taxonomy": self.taxonomy,
            "taxonomy_version": self.taxonomy_version,
            "source": self.source,
            "metadata": dict(self.metadata),
            "num_labels": self.num_labels,
            "has_confidence": bool(self.confidence.size),
        }
        if include_labels:
            data["labels"] = self.labels.tolist()
            data["confidence"] = self.confidence.tolist()
        return data

    def encode(self) -> bytes:
        header = self.to_dict(include_labels=False)
        header_bytes = json.dumps(header, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        confidence_payload = self.confidence.astype("<f4", copy=False).tobytes() if self.confidence.size else b""
        return (
            _HEADER_LEN.pack(len(header_bytes))
            + header_bytes
            + self.labels.astype("<u2", copy=False).tobytes()
            + confidence_payload
        )

    @classmethod
    def decode(cls, data: bytes) -> SemanticLabelsFrame:
        if len(data) < _HEADER_LEN.size:
            raise ValueError("encoded SemanticLabelsFrame is too short")
        header_len = _HEADER_LEN.unpack_from(data, 0)[0]
        start = _HEADER_LEN.size
        end = start + header_len
        if len(data) < end:
            raise ValueError("encoded SemanticLabelsFrame header is truncated")
        header = json.loads(data[start:end].decode("utf-8"))
        count = int(header.get("num_labels", 0))
        if count <= 0:
            raise ValueError("encoded SemanticLabelsFrame label count is invalid")
        labels_size = count * 2
        confidence_size = count * 4 if header.get("has_confidence") else 0
        payload = data[end:]
        if len(payload) != labels_size + confidence_size:
            raise ValueError("encoded SemanticLabelsFrame payload size mismatch")
        labels = np.frombuffer(payload[:labels_size], dtype="<u2").copy()
        confidence = np.frombuffer(payload[labels_size:], dtype="<f4").copy() if confidence_size else []
        return cls(
            labels=labels,
            confidence=confidence,
            sequence=int(header.get("sequence", 0)),
            ts=float(header.get("ts", 0.0)),
            frame_id=str(header.get("frame_id", "")),
            taxonomy=str(header.get("taxonomy", "")),
            taxonomy_version=int(header.get("taxonomy_version", 0)),
            source=str(header.get("source", "")),
            metadata=dict(header.get("metadata") or {}),
            schema_version=str(header.get("schema_version", SEMANTIC_LABELS_FRAME_SCHEMA)),
        )


@dataclass(frozen=True)
class SemanticSaveRequest:
    """Request an atomic native snapshot of the live semantic map."""

    msg_name: ClassVar[str] = "map.SemanticSaveRequest"

    request_id: str
    map_id: str
    path: str
    schema_version: str = SEMANTIC_SAVE_REQUEST_SCHEMA

    def __post_init__(self) -> None:
        if not all(isinstance(value, str) for value in (self.request_id, self.map_id, self.path, self.schema_version)):
            raise TypeError("semantic save request fields must be strings")
        if self.schema_version != SEMANTIC_SAVE_REQUEST_SCHEMA:
            raise ValueError(f"schema_version must be {SEMANTIC_SAVE_REQUEST_SCHEMA!r}")
        if not self.request_id.strip():
            raise ValueError("request_id is required")
        if not self.map_id.strip():
            raise ValueError("map_id is required")
        if not self.path.strip():
            raise ValueError("path is required")

    def encode(self) -> bytes:
        """Encode the save request for routed runtime delivery."""
        payload = json.dumps(
            {
                "schema_version": self.schema_version,
                "request_id": self.request_id,
                "map_id": self.map_id,
                "path": self.path,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        return _HEADER_LEN.pack(len(payload)) + payload

    @classmethod
    def decode(cls, data: bytes) -> SemanticSaveRequest:
        """Decode a routed semantic save request."""
        payload = _decode_json_message(data, "SemanticSaveRequest")
        if not all(isinstance(payload.get(key), str) for key in ("request_id", "map_id", "path", "schema_version")):
            raise ValueError("encoded SemanticSaveRequest has invalid field types")
        return cls(
            request_id=payload["request_id"],
            map_id=payload["map_id"],
            path=payload["path"],
            schema_version=payload["schema_version"],
        )


@dataclass(frozen=True)
class SemanticSaveResult:
    """Result of an atomic native semantic-map snapshot request."""

    msg_name: ClassVar[str] = "map.SemanticSaveResult"

    request_id: str
    map_id: str
    path: str
    success: bool
    generation: int = 0
    voxel_count: int = 0
    message: str = ""
    schema_version: str = SEMANTIC_SAVE_RESULT_SCHEMA

    def __post_init__(self) -> None:
        if not all(
            isinstance(value, str)
            for value in (
                self.request_id,
                self.map_id,
                self.path,
                self.message,
                self.schema_version,
            )
        ):
            raise TypeError("semantic save result text fields must be strings")
        if not isinstance(self.success, bool):
            raise TypeError("success must be bool")
        if not isinstance(self.generation, int) or isinstance(self.generation, bool):
            raise TypeError("generation must be int")
        if not isinstance(self.voxel_count, int) or isinstance(self.voxel_count, bool):
            raise TypeError("voxel_count must be int")
        if self.schema_version != SEMANTIC_SAVE_RESULT_SCHEMA:
            raise ValueError(f"schema_version must be {SEMANTIC_SAVE_RESULT_SCHEMA!r}")
        if not self.request_id.strip():
            raise ValueError("request_id is required")
        if not self.map_id.strip():
            raise ValueError("map_id is required")
        if not self.path.strip():
            raise ValueError("path is required")
        if self.generation < 0:
            raise ValueError("generation must be non-negative")
        if self.voxel_count < 0:
            raise ValueError("voxel_count must be non-negative")

    def encode(self) -> bytes:
        """Encode the save result for routed runtime delivery."""
        payload = json.dumps(
            {
                "schema_version": self.schema_version,
                "request_id": self.request_id,
                "map_id": self.map_id,
                "path": self.path,
                "success": self.success,
                "generation": self.generation,
                "voxel_count": self.voxel_count,
                "message": self.message,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        return _HEADER_LEN.pack(len(payload)) + payload

    @classmethod
    def decode(cls, data: bytes) -> SemanticSaveResult:
        """Decode a routed semantic save result."""
        payload = _decode_json_message(data, "SemanticSaveResult")
        if not all(
            isinstance(payload.get(key), str) for key in ("request_id", "map_id", "path", "message", "schema_version")
        ):
            raise ValueError("encoded SemanticSaveResult has invalid text fields")
        if not isinstance(payload.get("success"), bool):
            raise ValueError("encoded SemanticSaveResult success must be bool")
        if not isinstance(payload.get("generation"), int) or isinstance(payload.get("generation"), bool):
            raise ValueError("encoded SemanticSaveResult generation must be int")
        if not isinstance(payload.get("voxel_count"), int) or isinstance(payload.get("voxel_count"), bool):
            raise ValueError("encoded SemanticSaveResult voxel_count must be int")
        return cls(
            request_id=payload["request_id"],
            map_id=payload["map_id"],
            path=payload["path"],
            success=payload["success"],
            generation=payload["generation"],
            voxel_count=payload["voxel_count"],
            message=payload["message"],
            schema_version=payload["schema_version"],
        )


@dataclass
class MapSceneFrame:
    """Layered map visualization/product snapshot.

    This is the map-domain scene contract consumed by Gateway and future
    visualization adapters. Heavy point payloads may stay as native message
    objects inside layer dictionaries while Module delivery is in-process.
    ``to_dict(include_payload=False)`` keeps status/debug payloads small.
    """

    msg_name: ClassVar[str] = "map.MapSceneFrame"

    layers: list[dict[str, Any]] = field(default_factory=list)
    ts: float = field(default_factory=time.time)
    frame_id: str = field(default=MAP_FRAME_ID)
    map_id: str = ""
    source: str = ""
    sequence: int = 0
    metadata: dict[str, Any] = field(default_factory=dict)
    schema_version: str = MAP_SCENE_FRAME_SCHEMA

    def __post_init__(self) -> None:
        if self.schema_version != MAP_SCENE_FRAME_SCHEMA:
            raise ValueError(f"schema_version must be {MAP_SCENE_FRAME_SCHEMA!r}, got {self.schema_version!r}")
        self.frame_id = str(self.frame_id or MAP_FRAME_ID)
        self.map_id = str(self.map_id or "")
        self.source = str(self.source or "")
        self.sequence = int(self.sequence or 0)
        if not self.ts:
            self.ts = time.time()
        if not isinstance(self.metadata, dict):
            raise TypeError("metadata must be a dict")
        normalized_layers: list[dict[str, Any]] = []
        for layer in self.layers:
            if not isinstance(layer, dict):
                raise TypeError("MapSceneFrame layers must be dict objects")
            normalized = dict(layer)
            normalized.setdefault("frame_id", self.frame_id)
            normalized.setdefault("ts", self.ts)
            normalized_layers.append(normalized)
        self.layers = normalized_layers

    def to_dict(self, *, include_payload: bool = False) -> dict[str, Any]:
        layers: list[dict[str, Any]] = []
        for layer in self.layers:
            item = dict(layer)
            if not include_payload:
                item.pop("payload", None)
                item.pop("cloud", None)
                item.pop("grid", None)
            layers.append(item)
        return {
            "schema_version": self.schema_version,
            "ts": float(self.ts),
            "frame_id": self.frame_id,
            "map_id": self.map_id,
            "source": self.source,
            "sequence": int(self.sequence),
            "metadata": dict(self.metadata),
            "layers": layers,
        }

    def encode(self) -> bytes:
        data = self.to_dict(include_payload=False)
        header_bytes = json.dumps(
            data,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        return _HEADER_LEN.pack(len(header_bytes)) + header_bytes

    @classmethod
    def decode(cls, data: bytes) -> MapSceneFrame:
        if len(data) < _HEADER_LEN.size:
            raise ValueError("encoded MapSceneFrame is too short")
        header_len = _HEADER_LEN.unpack_from(data, 0)[0]
        start = _HEADER_LEN.size
        end = start + header_len
        if len(data) < end:
            raise ValueError("encoded MapSceneFrame is truncated")
        payload = json.loads(data[start:end].decode("utf-8"))
        return cls(
            layers=list(payload.get("layers") or []),
            ts=float(payload.get("ts", 0.0) or time.time()),
            frame_id=str(payload.get("frame_id", MAP_FRAME_ID) or MAP_FRAME_ID),
            map_id=str(payload.get("map_id", "") or ""),
            source=str(payload.get("source", "") or ""),
            sequence=int(payload.get("sequence", 0) or 0),
            metadata=dict(payload.get("metadata") or {}),
            schema_version=str(payload.get("schema_version", MAP_SCENE_FRAME_SCHEMA)),
        )
