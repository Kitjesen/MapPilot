"""Goal construction helpers for App/Web navigation commands."""

from __future__ import annotations

import math
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from gateway.schemas import PlanPreviewRequest
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.runtime_interface import map_frame_id

GOAL_MAP_FRAME_ID = map_frame_id()


@dataclass(frozen=True)
class ConstructedGoal:
    x: float
    y: float
    z: float = 0.0
    yaw: float | None = None
    frame_id: str = GOAL_MAP_FRAME_ID
    source: str = "coordinate"
    target_type: str = "coordinate"
    label: str | None = None
    location_name: str | None = None
    acceptance_radius_m: float | None = None
    max_speed_mps: float | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def pose_stamped(self, *, ts: float | None = None) -> PoseStamped:
        orientation = (
            Quaternion.from_yaw(self.yaw)
            if self.yaw is not None
            else Quaternion(0.0, 0.0, 0.0, 0.0)
        )
        return PoseStamped(
            pose=Pose(
                position=Vector3(self.x, self.y, self.z),
                orientation=orientation,
            ),
            frame_id=self.frame_id,
            ts=ts or time.time(),
        )

    def preview_request(self) -> PlanPreviewRequest:
        return PlanPreviewRequest(
            x=self.x,
            y=self.y,
            z=self.z,
            frame_id=self.frame_id,
        )

    def target_payload(self, *, ts: float | None = None) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw": self.yaw,
            "frame_id": self.frame_id,
            "source": self.source,
            "target_type": self.target_type,
            "label": self.label,
            "location_name": self.location_name,
            "acceptance_radius_m": self.acceptance_radius_m,
            "max_speed_mps": self.max_speed_mps,
            "metadata": dict(self.metadata),
            "ts": ts or time.time(),
        }

    def command_payload(
        self,
        *,
        status: str = "ok",
        instruction: str | None = None,
        ts: float | None = None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "status": status,
            "goal": [self.x, self.y, self.z],
            "yaw": self.yaw,
            "frame_id": self.frame_id,
            "target": self.target_payload(ts=ts),
        }
        if instruction is not None:
            payload["instruction"] = instruction
        return payload


def construct_goal_from_request(
    body: Any,
    *,
    gw: Any = None,
    default_source: str = "coordinate",
    default_target_type: str = "coordinate",
) -> ConstructedGoal:
    """Normalize an API request or saved-location lookup into one goal shape."""
    frame_id = str(
        getattr(body, "frame_id", GOAL_MAP_FRAME_ID) or GOAL_MAP_FRAME_ID
    ).strip()
    if frame_id != GOAL_MAP_FRAME_ID:
        raise ValueError(f"goal frame_id must be {GOAL_MAP_FRAME_ID}")

    fields_set = set(getattr(body, "model_fields_set", set()) or set())
    location_name = _optional_text(getattr(body, "location_name", None))
    source = str(getattr(body, "source", default_source) or default_source)
    target_type = str(
        getattr(body, "target_type", default_target_type) or default_target_type
    )
    label = _optional_text(getattr(body, "label", None))
    yaw_value = getattr(body, "yaw", None)

    if location_name:
        entry = _resolve_location(gw, location_name)
        x, y, z = _coordinates_from_location(entry)
        yaw = _optional_float(yaw_value, "yaw")
        if yaw is None:
            yaw = _optional_float(_mapping(entry).get("yaw"), "location.yaw")
        if "source" not in fields_set:
            source = "saved_location"
        if "target_type" not in fields_set:
            target_type = "saved_location"
        if label is None:
            label = _optional_text(_mapping(entry).get("name")) or location_name
    else:
        x = _required_float(getattr(body, "x", None), "x")
        y = _required_float(getattr(body, "y", None), "y")
        z = _required_float(getattr(body, "z", 0.0), "z")
        yaw = _optional_float(yaw_value, "yaw")
        if yaw is None and default_source != "map_click":
            yaw = 0.0

    return ConstructedGoal(
        x=x,
        y=y,
        z=z,
        yaw=yaw,
        frame_id=frame_id,
        source=source,
        target_type=target_type,
        label=label,
        location_name=location_name,
        acceptance_radius_m=_optional_float(
            getattr(body, "acceptance_radius_m", None),
            "acceptance_radius_m",
        ),
        max_speed_mps=_optional_float(
            getattr(body, "max_speed_mps", None),
            "max_speed_mps",
        ),
        metadata=_metadata(getattr(body, "metadata", None)),
    )


def _resolve_location(gw: Any, location_name: str) -> Mapping[str, Any]:
    store = getattr(getattr(gw, "_tagged_loc_module", None), "store", None)
    if store is None:
        raise ValueError("location_store_unavailable")

    entry = None
    query = getattr(store, "query", None)
    if callable(query):
        entry = query(location_name)
    if entry is None:
        query_fuzzy = getattr(store, "query_fuzzy", None)
        if callable(query_fuzzy):
            entry = query_fuzzy(location_name)
    if not isinstance(entry, Mapping):
        raise ValueError(f"location_not_found:{location_name}")
    return entry


def _coordinates_from_location(entry: Mapping[str, Any]) -> tuple[float, float, float]:
    raw_position = entry.get("position")
    if isinstance(raw_position, Sequence) and not isinstance(raw_position, (str, bytes)):
        if len(raw_position) < 2:
            raise ValueError("location_position_invalid")
        x = _required_float(raw_position[0], "location.position.x")
        y = _required_float(raw_position[1], "location.position.y")
        z = _required_float(
            raw_position[2] if len(raw_position) > 2 else 0.0,
            "location.position.z",
        )
        return x, y, z
    return (
        _required_float(entry.get("x"), "location.x"),
        _required_float(entry.get("y"), "location.y"),
        _required_float(entry.get("z", 0.0), "location.z"),
    )


def _required_float(value: Any, name: str) -> float:
    result = _optional_float(value, name)
    if result is None:
        raise ValueError(f"{name} is required")
    return result


def _optional_float(value: Any, name: str) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _optional_text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _metadata(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}
