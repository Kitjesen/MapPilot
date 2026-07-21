"""Planner service contracts shared by navigation runtime components."""

from __future__ import annotations

import copy
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from runtime.msgs.geometry import Pose, PoseStamped
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.numpy_compat import np
from runtime.runtime_interface import map_frame_id

GLOBAL_PLAN_SCHEMA_VERSION = "lingtu.global_plan.v1"
GLOBAL_PLAN_REQUEST_CONTRACT = "lingtu.global_plan.request"
GLOBAL_PLAN_RESULT_CONTRACT = "lingtu.global_plan.result"
GLOBAL_PLANNING_MAP_CONTRACT = "lingtu.global_plan.map"

LOCAL_PLAN_SCHEMA_VERSION = "lingtu.local_plan.v1"
LOCAL_PLAN_REQUEST_CONTRACT = "lingtu.local_plan.request"
LOCAL_PLAN_RESULT_CONTRACT = "lingtu.local_plan.result"
LOCAL_PLANNER_BACKENDS = ("nanobind", "cmu_py", "simple")

LOCAL_PLAN_PORT_CONTRACT: dict[str, tuple[str, str]] = {
    "odometry": ("In", "Odometry"),
    "terrain_map": ("In", "PointCloud2"),
    "terrain_map_ext": ("In", "PointCloud2"),
    "traversability": ("In", "dict"),
    "waypoint": ("In", "PoseStamped"),
    "global_path": ("In", "Path"),
    "clear_path": ("In", "bool"),
    "map_odom_tf": ("In", "dict"),
    "map_frame_jump_event": ("In", "dict"),
    "boundary": ("In", "PointCloud2"),
    "added_obstacles": ("In", "PointCloud2"),
    "check_obstacle": ("In", "bool"),
    "esdf": ("In", "dict"),
    "local_path": ("Out", "Path"),
    "control_hint": ("Out", "dict"),
    "alive": ("Out", "bool"),
}


def require_local_planner_backend(backend: str) -> None:
    """Validate a local planner backend name."""

    from runtime.backend_status import require_backend

    require_backend("local_planner", backend, LOCAL_PLANNER_BACKENDS)


GLOBAL_PLAN_RESULT_SCHEMA: dict[str, Any] = {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "$id": GLOBAL_PLAN_RESULT_CONTRACT,
    "title": "LingTu Global Plan Result",
    "type": "object",
    "additionalProperties": False,
    "required": [
        "schema_version",
        "path",
        "plan_ms",
        "reached_goal",
        "error",
        "frame_id",
        "request_id",
        "map_version",
        "adjusted_goal",
        "diagnostics",
        "report",
    ],
    "properties": {
        "schema_version": {"const": GLOBAL_PLAN_SCHEMA_VERSION},
        "path": {
            "type": "array",
            "items": {
                "type": "array",
                "prefixItems": [
                    {"type": "number"},
                    {"type": "number"},
                    {"type": "number"},
                ],
                "minItems": 3,
                "maxItems": 3,
            },
        },
        "plan_ms": {"type": "number", "minimum": 0},
        "reached_goal": {"type": "boolean"},
        "error": {"type": "string"},
        "frame_id": {"type": "string", "minLength": 1},
        "request_id": {"type": "string"},
        "map_version": {"type": "string"},
        "map_generation": {"type": "integer", "minimum": 0},
        "adjusted_goal": {
            "anyOf": [
                {"type": "null"},
                {
                    "type": "array",
                    "prefixItems": [
                        {"type": "number"},
                        {"type": "number"},
                        {"type": "number"},
                    ],
                    "minItems": 3,
                    "maxItems": 3,
                },
            ]
        },
        "diagnostics": {"type": "object"},
        "report": {"type": "object"},
    },
}


@dataclass(slots=True)
class GlobalPlanningMap:
    """Map payload accepted by planner services and backend adapters."""

    grid: Any
    resolution: float = 0.2
    origin: Any | None = None
    frame_id: str = field(default_factory=map_frame_id)
    map_version: str = ""
    generation: int = 0
    source: str = ""

    def __post_init__(self) -> None:
        grid = np.asarray(self.grid, dtype=np.float32)
        if grid.ndim != 2:
            raise ValueError("planning map grid must be 2-D")
        self.grid = grid.copy()
        self.resolution = float(self.resolution)
        self.frame_id = str(self.frame_id or map_frame_id())
        self.map_version = str(self.map_version or "")
        self.generation = int(self.generation)
        if self.generation < 0:
            raise ValueError("planning map generation must be non-negative")
        if self.origin is None:
            return
        origin = np.asarray(self.origin, dtype=float).reshape(-1)
        if origin.size < 2:
            raise ValueError("planning map origin must contain x and y")
        self.origin = origin[:2].copy()

    def to_wire(self) -> dict[str, Any]:
        return {
            "schema_version": GLOBAL_PLAN_SCHEMA_VERSION,
            "grid": self.grid.tolist(),
            "resolution": self.resolution,
            "origin": None if self.origin is None else self.origin.tolist(),
            "frame_id": self.frame_id,
            "map_version": self.map_version,
            "generation": self.generation,
            "source": self.source,
        }

    @classmethod
    def from_wire(cls, payload: dict[str, Any]) -> GlobalPlanningMap:
        return cls(
            grid=payload.get("grid", []),
            resolution=float(payload.get("resolution", 0.2)),
            origin=payload.get("origin"),
            frame_id=str(payload.get("frame_id") or map_frame_id()),
            map_version=str(payload.get("map_version") or ""),
            generation=int(payload.get("generation", 0) or 0),
            source=str(payload.get("source") or ""),
        )


@dataclass(slots=True)
class GlobalPlannerDiagnostics:
    """Structured diagnostics that can still be emitted as a plain dict."""

    planner: str = ""
    stage: str = ""
    error: str = ""
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = dict(self.details)
        if self.planner:
            data.setdefault("planner", self.planner)
        if self.stage:
            data.setdefault("stage", self.stage)
        if self.error:
            data.setdefault("error", self.error)
        return data

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> GlobalPlannerDiagnostics:
        data = dict(payload)
        return cls(
            planner=str(data.pop("planner", "") or ""),
            stage=str(data.pop("stage", "") or ""),
            error=str(data.pop("error", "") or ""),
            details=data,
        )


@dataclass(slots=True)
class GlobalPlanRequest:
    """Canonical input for a global planner implementation."""

    start: np.ndarray
    goal: np.ndarray
    safe_goal_tolerance: float = 4.0
    frame_id: str = field(default_factory=map_frame_id)
    request_id: str = ""
    map_version: str = ""
    map_generation: int = 0

    def __post_init__(self) -> None:
        self.start = _coerce_xyz(self.start, field_name="start")
        self.goal = _coerce_xyz(self.goal, field_name="goal")
        self.safe_goal_tolerance = float(self.safe_goal_tolerance)
        self.frame_id = str(self.frame_id or map_frame_id())
        self.map_generation = int(self.map_generation)
        if self.map_generation < 0:
            raise ValueError("global plan map generation must be non-negative")

    def to_wire(self) -> dict[str, Any]:
        return {
            "schema_version": GLOBAL_PLAN_SCHEMA_VERSION,
            "start": self.start.tolist(),
            "goal": self.goal.tolist(),
            "safe_goal_tolerance": self.safe_goal_tolerance,
            "frame_id": self.frame_id,
            "request_id": self.request_id,
            "map_version": self.map_version,
            "map_generation": self.map_generation,
        }

    @classmethod
    def from_wire(cls, payload: dict[str, Any]) -> GlobalPlanRequest:
        return cls(
            start=payload.get("start", []),
            goal=payload.get("goal", []),
            safe_goal_tolerance=float(payload.get("safe_goal_tolerance", 4.0)),
            frame_id=str(payload.get("frame_id") or map_frame_id()),
            request_id=str(payload.get("request_id") or ""),
            map_version=str(payload.get("map_version") or ""),
            map_generation=int(payload.get("map_generation", 0) or 0),
        )


@dataclass(slots=True)
class GlobalPlanResult:
    """Canonical output from a global planner implementation."""

    path: Path | list[Any] = field(default_factory=Path)
    plan_ms: float = 0.0
    reached_goal: bool = False
    error: str = ""
    frame_id: str = field(default_factory=map_frame_id)
    request_id: str = ""
    map_version: str = ""
    map_generation: int = 0
    adjusted_goal: Any | None = None
    diagnostics: dict[str, Any] = field(default_factory=dict)
    report: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.frame_id = str(self.frame_id or map_frame_id())
        self.map_generation = int(self.map_generation)
        if self.map_generation < 0:
            raise ValueError("global plan result map generation must be non-negative")
        self.path = coerce_global_path(self.path, frame_id=self.frame_id)

    @property
    def ok(self) -> bool:
        return bool(self.path) and not self.error

    def points(self) -> list[np.ndarray]:
        """Return path waypoints as xyz numpy arrays for legacy planner internals."""

        return [point_to_xyz(point) for point in self.path]

    def to_wire(self) -> dict[str, Any]:
        return {
            "schema_version": GLOBAL_PLAN_SCHEMA_VERSION,
            "path": [point.tolist() for point in self.points()],
            "plan_ms": float(self.plan_ms),
            "reached_goal": bool(self.reached_goal),
            "error": self.error,
            "frame_id": self.frame_id,
            "request_id": self.request_id,
            "map_version": self.map_version,
            "map_generation": self.map_generation,
            "adjusted_goal": (None if self.adjusted_goal is None else point_to_xyz(self.adjusted_goal).tolist()),
            "diagnostics": dict(self.diagnostics),
            "report": dict(self.report),
        }

    @classmethod
    def from_wire(cls, payload: dict[str, Any]) -> GlobalPlanResult:
        return cls(
            path=payload.get("path", []),
            plan_ms=float(payload.get("plan_ms", 0.0)),
            reached_goal=bool(payload.get("reached_goal", False)),
            error=str(payload.get("error") or ""),
            frame_id=str(payload.get("frame_id") or map_frame_id()),
            request_id=str(payload.get("request_id") or ""),
            map_version=str(payload.get("map_version") or ""),
            map_generation=int(payload.get("map_generation", 0) or 0),
            adjusted_goal=payload.get("adjusted_goal"),
            diagnostics=dict(payload.get("diagnostics") or {}),
            report=dict(payload.get("report") or {}),
        )


PlanRequest = GlobalPlanRequest
PlanResult = GlobalPlanResult
PlanningMap = GlobalPlanningMap
PlannerDiagnostics = GlobalPlannerDiagnostics


@dataclass(slots=True)
class LocalPlanRequest:
    """Canonical input for local planning at one control tick."""

    odometry: Pose | PoseStamped | Odometry | Any
    waypoint: Pose | PoseStamped | Any
    global_path: Path | list[Any] | tuple[Any, ...] | None = None
    obstacle_points: Any | None = None
    terrain_points: Any | None = None
    frame_id: str = field(default_factory=map_frame_id)
    request_id: str = ""
    map_version: str = ""
    timestamp_s: float = 0.0

    def __post_init__(self) -> None:
        self.frame_id = str(self.frame_id or map_frame_id())
        self.timestamp_s = float(self.timestamp_s or 0.0)
        self.global_path = coerce_global_path(self.global_path, frame_id=self.frame_id)

    def robot_xyz(self) -> np.ndarray:
        return point_to_xyz(self.odometry)

    def goal_xyz(self) -> np.ndarray:
        return point_to_xyz(self.waypoint)

    def to_wire(self) -> dict[str, Any]:
        return {
            "schema_version": LOCAL_PLAN_SCHEMA_VERSION,
            "odometry": self.robot_xyz().tolist(),
            "waypoint": self.goal_xyz().tolist(),
            "global_path": [point.tolist() for point in self.global_path_points()],
            "frame_id": self.frame_id,
            "request_id": self.request_id,
            "map_version": self.map_version,
            "timestamp_s": self.timestamp_s,
        }

    def global_path_points(self) -> list[np.ndarray]:
        return [point_to_xyz(point) for point in self.global_path]


@dataclass(slots=True)
class LocalPlanResult:
    """Canonical output from local planning before path following."""

    local_path: Path | list[Any] = field(default_factory=Path)
    control_hint: dict[str, Any] = field(default_factory=dict)
    plan_ms: float = 0.0
    path_found: bool = False
    safety_stop: bool = False
    error: str = ""
    frame_id: str = field(default_factory=map_frame_id)
    request_id: str = ""
    map_version: str = ""
    backend: str = ""
    diagnostics: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.frame_id = str(self.frame_id or map_frame_id())
        self.local_path = coerce_global_path(self.local_path, frame_id=self.frame_id)
        self.path_found = bool(self.path_found or len(self.local_path.poses) >= 2)
        self.safety_stop = bool(self.safety_stop)
        self.plan_ms = float(self.plan_ms)

    @property
    def ok(self) -> bool:
        return self.path_found and not self.safety_stop and not self.error

    def points(self) -> list[np.ndarray]:
        return [point_to_xyz(point) for point in self.local_path]

    def to_wire(self) -> dict[str, Any]:
        return {
            "schema_version": LOCAL_PLAN_SCHEMA_VERSION,
            "local_path": [point.tolist() for point in self.points()],
            "control_hint": dict(self.control_hint),
            "plan_ms": self.plan_ms,
            "path_found": self.path_found,
            "safety_stop": self.safety_stop,
            "error": self.error,
            "frame_id": self.frame_id,
            "request_id": self.request_id,
            "map_version": self.map_version,
            "backend": self.backend,
            "diagnostics": dict(self.diagnostics),
        }

    @classmethod
    def from_wire(cls, payload: dict[str, Any]) -> LocalPlanResult:
        return cls(
            local_path=payload.get("local_path", []),
            control_hint=dict(payload.get("control_hint") or {}),
            plan_ms=float(payload.get("plan_ms", 0.0)),
            path_found=bool(payload.get("path_found", False)),
            safety_stop=bool(payload.get("safety_stop", False)),
            error=str(payload.get("error") or ""),
            frame_id=str(payload.get("frame_id") or map_frame_id()),
            request_id=str(payload.get("request_id") or ""),
            map_version=str(payload.get("map_version") or ""),
            backend=str(payload.get("backend") or ""),
            diagnostics=dict(payload.get("diagnostics") or {}),
        )


def global_plan_result_schema() -> dict[str, Any]:
    """Return the JSON schema consumed by UI and non-Python adapters."""

    return copy.deepcopy(GLOBAL_PLAN_RESULT_SCHEMA)


def validate_global_plan_result_wire(payload: Any) -> list[str]:
    """Return human-readable issues for a global plan result wire payload."""

    if not isinstance(payload, Mapping):
        return [f"payload must be a mapping, got {type(payload).__name__}"]
    issues: list[str] = []
    required = tuple(GLOBAL_PLAN_RESULT_SCHEMA["required"])
    for field_name in required:
        if field_name not in payload:
            issues.append(f"{field_name}: required field is missing")
    if payload.get("schema_version") != GLOBAL_PLAN_SCHEMA_VERSION:
        issues.append(f"schema_version: expected {GLOBAL_PLAN_SCHEMA_VERSION!r}, got {payload.get('schema_version')!r}")
    if "path" in payload and not _is_xyz_list(payload["path"]):
        issues.append("path: must be a list of finite [x, y, z] points")
    if "adjusted_goal" in payload and payload["adjusted_goal"] is not None:
        if not _is_xyz(payload["adjusted_goal"]):
            issues.append("adjusted_goal: must be null or a finite [x, y, z] point")
    if "plan_ms" in payload and not _is_finite_number(payload["plan_ms"], minimum=0.0):
        issues.append("plan_ms: must be a finite number >= 0")
    if "reached_goal" in payload and not isinstance(payload["reached_goal"], bool):
        issues.append("reached_goal: must be a boolean")
    if "error" in payload and not isinstance(payload["error"], str):
        issues.append("error: must be a string")
    if "frame_id" in payload and not str(payload["frame_id"]):
        issues.append("frame_id: must be a non-empty string")
    if "request_id" in payload and not isinstance(payload["request_id"], str):
        issues.append("request_id: must be a string")
    if "map_version" in payload and not isinstance(payload["map_version"], str):
        issues.append("map_version: must be a string")
    if "map_generation" in payload:
        generation = payload["map_generation"]
        if isinstance(generation, bool) or not isinstance(generation, int) or generation < 0:
            issues.append("map_generation: must be an integer >= 0")
    if "diagnostics" in payload and not isinstance(payload["diagnostics"], Mapping):
        issues.append("diagnostics: must be an object")
    if "report" in payload and not isinstance(payload["report"], Mapping):
        issues.append("report: must be an object")
    return issues


def assert_global_plan_result_wire(payload: Any) -> None:
    """Raise ValueError if a global plan result wire payload is invalid."""

    issues = validate_global_plan_result_wire(payload)
    if issues:
        raise ValueError("; ".join(issues))


def validate_local_plan_result_wire(payload: Any) -> list[str]:
    """Return human-readable issues for a local plan result wire payload."""

    if not isinstance(payload, Mapping):
        return [f"payload must be a mapping, got {type(payload).__name__}"]
    issues: list[str] = []
    required = (
        "schema_version",
        "local_path",
        "control_hint",
        "plan_ms",
        "path_found",
        "safety_stop",
        "error",
        "frame_id",
        "request_id",
        "map_version",
        "backend",
        "diagnostics",
    )
    for field_name in required:
        if field_name not in payload:
            issues.append(f"{field_name}: required field is missing")
    if payload.get("schema_version") != LOCAL_PLAN_SCHEMA_VERSION:
        issues.append(f"schema_version: expected {LOCAL_PLAN_SCHEMA_VERSION!r}, got {payload.get('schema_version')!r}")
    if "local_path" in payload and not _is_xyz_list(payload["local_path"]):
        issues.append("local_path: must be a list of finite [x, y, z] points")
    if "control_hint" in payload and not isinstance(payload["control_hint"], Mapping):
        issues.append("control_hint: must be an object")
    if "plan_ms" in payload and not _is_finite_number(payload["plan_ms"], minimum=0.0):
        issues.append("plan_ms: must be a finite number >= 0")
    if "path_found" in payload and not isinstance(payload["path_found"], bool):
        issues.append("path_found: must be a boolean")
    if "safety_stop" in payload and not isinstance(payload["safety_stop"], bool):
        issues.append("safety_stop: must be a boolean")
    if "error" in payload and not isinstance(payload["error"], str):
        issues.append("error: must be a string")
    if "frame_id" in payload and not str(payload["frame_id"]):
        issues.append("frame_id: must be a non-empty string")
    if "request_id" in payload and not isinstance(payload["request_id"], str):
        issues.append("request_id: must be a string")
    if "map_version" in payload and not isinstance(payload["map_version"], str):
        issues.append("map_version: must be a string")
    if "backend" in payload and not isinstance(payload["backend"], str):
        issues.append("backend: must be a string")
    if "diagnostics" in payload and not isinstance(payload["diagnostics"], Mapping):
        issues.append("diagnostics: must be an object")
    return issues


def assert_local_plan_result_wire(payload: Any) -> None:
    """Raise ValueError if a local plan result wire payload is invalid."""

    issues = validate_local_plan_result_wire(payload)
    if issues:
        raise ValueError("; ".join(issues))


def coerce_planning_map(
    grid: Any,
    *,
    resolution: float = 0.2,
    origin: Any | None = None,
    frame_id: str = "",
    map_version: str = "",
    generation: int = 0,
    source: str = "",
) -> GlobalPlanningMap:
    """Return a canonical PlanningMap while keeping legacy grid calls valid."""

    if isinstance(grid, GlobalPlanningMap):
        return grid
    return GlobalPlanningMap(
        grid=grid,
        resolution=resolution,
        origin=origin,
        frame_id=frame_id or map_frame_id(),
        map_version=map_version,
        generation=generation,
        source=source,
    )


def coerce_global_path(path: Path | list[Any] | tuple[Any, ...] | None, *, frame_id: str) -> Path:
    """Return a stable Path message from legacy point-list planner output."""

    if isinstance(path, Path):
        path_frame = str(path.frame_id or frame_id)
        return Path(
            poses=[_coerce_pose_stamped(point, path_frame) for point in path.poses],
            ts=float(path.ts or 0.0),
            frame_id=path_frame,
        )
    items = [] if path is None else list(path)
    poses = [_coerce_pose_stamped(point, frame_id) for point in items]
    return Path(poses=poses, frame_id=frame_id)


def point_to_xyz(point: Any) -> np.ndarray:
    """Return one waypoint as an xyz numpy array."""

    if isinstance(point, PoseStamped):
        return np.asarray([point.x, point.y, point.z], dtype=float)
    if isinstance(point, Pose):
        return np.asarray([point.x, point.y, point.z], dtype=float)
    if hasattr(point, "pose"):
        return point_to_xyz(point.pose)
    if hasattr(point, "position"):
        return point_to_xyz(point.position)
    if hasattr(point, "x") and hasattr(point, "y"):
        return np.asarray(
            [
                float(getattr(point, "x", 0.0)),
                float(getattr(point, "y", 0.0)),
                float(getattr(point, "z", 0.0)),
            ],
            dtype=float,
        )
    if isinstance(point, dict):
        if "pose" in point:
            return point_to_xyz(point["pose"])
        if "position" in point:
            return point_to_xyz(point["position"])
        return np.asarray(
            [
                float(point.get("x", 0.0)),
                float(point.get("y", 0.0)),
                float(point.get("z", 0.0)),
            ],
            dtype=float,
        )
    arr = np.asarray(point, dtype=float).reshape(-1)
    if arr.size < 2:
        raise ValueError("path point must contain at least x and y")
    out = np.zeros(3, dtype=float)
    out[: min(3, arr.size)] = arr[: min(3, arr.size)]
    return out


def _coerce_xyz(value: Any, *, field_name: str) -> np.ndarray:
    point = point_to_xyz(value)
    if not np.all(np.isfinite(point)):
        raise ValueError(f"{field_name} must contain finite x/y/z")
    return point


def _coerce_pose_stamped(point: Any, frame_id: str) -> PoseStamped:
    if isinstance(point, PoseStamped):
        return PoseStamped(
            pose=point.pose,
            ts=float(point.ts or 0.0),
            frame_id=str(point.frame_id or frame_id),
        )
    xyz = point_to_xyz(point)
    return PoseStamped(
        pose=Pose(float(xyz[0]), float(xyz[1]), float(xyz[2])),
        frame_id=frame_id,
    )


def _is_finite_number(value: Any, *, minimum: float | None = None) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    number = float(value)
    if not math.isfinite(number):
        return False
    return minimum is None or number >= minimum


def _is_xyz(value: Any) -> bool:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        return False
    if len(value) != 3:
        return False
    return all(_is_finite_number(item) for item in value)


def _is_xyz_list(value: Any) -> bool:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        return False
    return all(_is_xyz(item) for item in value)


@runtime_checkable
class GlobalPlannerBackend(Protocol):
    """Minimal backend boundary used by GlobalPlanner."""

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        """Plan using the canonical request/result contract."""
        ...

    def plan(self, start: np.ndarray, goal: np.ndarray) -> list[Any]:
        """Compatibility wrapper for legacy callers."""
        ...

    def update_map(
        self,
        grid: GlobalPlanningMap | np.ndarray,
        resolution: float = 0.2,
        origin: np.ndarray | None = None,
    ) -> None:
        """Update the backend's planning map or cost grid."""
        ...


def require_global_planner_backend(name: str, backend: Any) -> GlobalPlannerBackend:
    """Fail fast when a registered planner backend does not match the runtime boundary."""

    missing = [attr for attr in ("plan_request", "update_map") if not callable(getattr(backend, attr, None))]
    if missing:
        raise TypeError(
            f"planner_backend/{name} must implement callable "
            "plan_request(request) and update_map(grid, resolution=0.2, origin=None); "
            f"missing: {', '.join(missing)}"
        )
    return backend


@runtime_checkable
class PlannerService(Protocol):
    """Public planner boundary consumed by nav.navigation."""

    @property
    def planner_name(self) -> str:
        """Return the configured planner backend name."""
        ...

    @property
    def plan_safety_policy(self) -> str:
        """Return the selected path-safety policy."""
        ...

    @property
    def is_ready(self) -> bool:
        """Return whether the service has completed setup."""
        ...

    @property
    def has_map(self) -> bool:
        """Return whether the service currently has map data."""
        ...

    @property
    def map_artifact_gate(self) -> dict[str, Any]:
        """Return the saved-map artifact validation state."""
        ...

    @property
    def last_plan_report(self) -> dict[str, Any]:
        """Return diagnostics from the most recent plan attempt."""
        ...

    def setup(self) -> None:
        """Initialize backend resources."""
        ...

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        """Plan using the canonical request/result contract."""
        ...

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        safe_goal_tolerance: float = 4.0,
    ) -> tuple[list[np.ndarray], float]:
        """Plan a path and return waypoints plus latency in milliseconds."""
        ...

    def update_map(
        self,
        grid: GlobalPlanningMap | np.ndarray,
        resolution: float = 0.2,
        origin: np.ndarray | None = None,
    ) -> None:
        """Update the planner's live cost grid."""
        ...

    def backend_status(self) -> dict[str, Any]:
        """Return backend health and degradation status."""
        ...

    def reload_map(self, map_path: str = "") -> dict[str, Any]:
        """Reload saved-map artifacts through the planner boundary."""
        ...
