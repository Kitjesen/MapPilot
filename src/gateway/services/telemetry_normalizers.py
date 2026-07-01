"""Normalize Gateway telemetry into stable App/Web API payloads."""

from __future__ import annotations

import dataclasses
import json
import math
import time
from collections.abc import Mapping, Sequence
from typing import Any

from runtime.runtime_interface import map_frame_id


TELEMETRY_MAP_FRAME_ID = map_frame_id()


def _mapping(value: Any) -> dict[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        try:
            raw = dataclasses.asdict(value)
            return raw if isinstance(raw, dict) else {}
        except Exception:
            return {}
    to_dict = getattr(value, "to_dict", None)
    if callable(to_dict):
        try:
            raw = to_dict()
            return dict(raw) if isinstance(raw, Mapping) else {}
        except Exception:
            return {}
    return {}


def _json_or_mapping(value: Any) -> tuple[Any, dict[str, Any]]:
    if isinstance(value, str):
        try:
            parsed = json.loads(value)
        except Exception:
            return value, {}
        if isinstance(parsed, Mapping):
            return value, dict(parsed)
        return value, {}
    mapped = _mapping(value)
    return value, mapped


def _finite_float(value: Any, default: float | None = None) -> float | None:
    try:
        num = float(value)
    except (TypeError, ValueError):
        return default
    return num if math.isfinite(num) else default


def _finite_float_default(value: Any, default: float = 0.0) -> float:
    num = _finite_float(value, None)
    return default if num is None else num


def _string_or_none(value: Any) -> str | None:
    if value is None:
        return None
    return str(value)


def _metadata(value: Any) -> dict[str, Any]:
    raw = _mapping(value)
    return raw if raw else {}


def _coerce_sequence(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, str):
        return []
    if isinstance(value, Mapping):
        return list(value.values())
    if isinstance(value, Sequence):
        return list(value)
    try:
        return list(value)
    except TypeError:
        return []


def _scene_object(item: Any, index: int) -> dict[str, Any] | None:
    raw = _mapping(item)
    if not raw and isinstance(item, Sequence) and not isinstance(item, str):
        coords = list(item)
        if len(coords) >= 2:
            return {
                "id": str(index),
                "label": "",
                "x": _finite_float(coords[0]),
                "y": _finite_float(coords[1]),
                "z": _finite_float(coords[2], 0.0) if len(coords) >= 3 else 0.0,
                "metadata": {},
            }
        return None
    if not raw:
        return None
    label = raw.get("label", raw.get("name", raw.get("class", "")))
    return {
        "id": _string_or_none(raw.get("id", raw.get("track_id", raw.get("uuid")))),
        "label": str(label or ""),
        "x": _finite_float(raw.get("x")),
        "y": _finite_float(raw.get("y")),
        "z": _finite_float(raw.get("z")),
        "confidence": _finite_float(raw.get("confidence", raw.get("score"))),
        "distance": _finite_float(raw.get("distance", raw.get("range"))),
        "bbox": raw.get("bbox", raw.get("box")),
        "metadata": _metadata(raw.get("metadata")),
    }


def _scene_relation(item: Any) -> dict[str, Any] | None:
    raw = _mapping(item)
    if not raw:
        return None
    return {
        "source": _string_or_none(raw.get("source", raw.get("from"))),
        "target": _string_or_none(raw.get("target", raw.get("to"))),
        "relation": _string_or_none(raw.get("relation", raw.get("type"))),
        "confidence": _finite_float(raw.get("confidence", raw.get("score"))),
        "metadata": _metadata(raw.get("metadata")),
    }


def _scene_region(item: Any) -> dict[str, Any] | None:
    raw = _mapping(item)
    if not raw:
        return None
    return {
        "id": _string_or_none(raw.get("id")),
        "name": _string_or_none(raw.get("name")),
        "label": _string_or_none(raw.get("label", raw.get("type"))),
        "x": _finite_float(raw.get("x")),
        "y": _finite_float(raw.get("y")),
        "z": _finite_float(raw.get("z")),
        "polygon": raw.get("polygon"),
        "metadata": _metadata(raw.get("metadata")),
    }


def build_scene_graph_response(scene_graph: Any) -> dict[str, Any]:
    legacy, parsed = _json_or_mapping(scene_graph)
    objects_source = parsed.get("objects", parsed.get("nodes", []))
    relations_source = parsed.get("relations", parsed.get("edges", []))
    regions_source = parsed.get("regions", parsed.get("areas", []))

    objects = [
        obj
        for idx, item in enumerate(_coerce_sequence(objects_source))
        if (obj := _scene_object(item, idx)) is not None
    ]
    relations = [
        rel
        for item in _coerce_sequence(relations_source)
        if (rel := _scene_relation(item)) is not None
    ]
    regions = [
        region
        for item in _coerce_sequence(regions_source)
        if (region := _scene_region(item)) is not None
    ]

    return {
        "schema_version": 1,
        "frame_id": str(
            parsed.get("frame_id")
            or parsed.get("frame")
            or TELEMETRY_MAP_FRAME_ID
        ),
        "ts": _finite_float(parsed.get("ts"), time.time()),
        "objects": objects,
        "relations": relations,
        "regions": regions,
        "count": len(objects),
        "scene_graph": legacy,
    }


def _point_mapping(item: Any) -> dict[str, Any]:
    attrs = {
        key: getattr(item, key)
        for key in ("x", "y", "z", "yaw", "frame_id", "ts")
        if hasattr(item, key)
    }
    if attrs:
        return attrs
    raw = _mapping(item)
    if raw:
        return raw
    if isinstance(item, Sequence) and not isinstance(item, str):
        values = list(item)
        if len(values) >= 2:
            return {
                "x": values[0],
                "y": values[1],
                "z": values[2] if len(values) >= 3 else 0.0,
            }
    pos = _mapping(getattr(item, "position", None))
    if pos:
        return pos
    return {}


def _path_point(item: Any) -> dict[str, Any] | None:
    raw = _point_mapping(item)
    x = _finite_float(raw.get("x"))
    y = _finite_float(raw.get("y"))
    if x is None or y is None:
        return None
    return {
        "x": x,
        "y": y,
        "z": _finite_float_default(raw.get("z"), 0.0),
        "yaw": _finite_float(raw.get("yaw")),
        "frame_id": _string_or_none(raw.get("frame_id")),
        "ts": _finite_float(raw.get("ts")),
        "metadata": _metadata(raw.get("metadata")),
    }


def _robot_pose(robot: Any) -> dict[str, Any] | None:
    raw = _point_mapping(robot)
    x = _finite_float(raw.get("x"))
    y = _finite_float(raw.get("y"))
    if x is None or y is None:
        return None
    return {
        "x": x,
        "y": y,
        "z": _finite_float_default(raw.get("z"), 0.0),
        "yaw": _finite_float(raw.get("yaw")),
        "vx": _finite_float(raw.get("vx")),
        "vy": _finite_float(raw.get("vy")),
        "wz": _finite_float(raw.get("wz", raw.get("omega"))),
        "frame_id": _string_or_none(raw.get("frame_id")),
        "ts": _finite_float(raw.get("ts")),
    }


def build_path_response(path: Any, robot: Any) -> dict[str, Any]:
    points = [
        point
        for item in _coerce_sequence(path)
        if (point := _path_point(item)) is not None
    ]
    frame_id = next(
        (
            str(point["frame_id"])
            for point in points
            if point.get("frame_id") is not None
        ),
        TELEMETRY_MAP_FRAME_ID,
    )
    return {
        "schema_version": 1,
        "path": points,
        "robot": _robot_pose(robot),
        "count": len(points),
        "frame_id": frame_id,
        "ts": time.time(),
        "source": "gateway_cache",
    }


def build_locations_response(entries: Any) -> dict[str, Any]:
    locations: list[dict[str, Any]] = []
    for entry in _coerce_sequence(entries):
        raw = _mapping(entry)
        name = raw.get("name")
        if not name:
            continue
        x = _finite_float(raw.get("x"))
        y = _finite_float(raw.get("y"))
        position = raw.get("position")
        if (x is None or y is None) and isinstance(position, Sequence) and not isinstance(position, str):
            coords = list(position)
            if len(coords) >= 2:
                x = _finite_float(coords[0])
                y = _finite_float(coords[1])
                if raw.get("z") is None and len(coords) >= 3:
                    raw["z"] = coords[2]
        if x is None or y is None:
            continue
        tags = raw.get("tags", [])
        if isinstance(tags, str):
            tags = [tags]
        elif not isinstance(tags, Sequence):
            tags = []
        locations.append(
            {
                "name": str(name),
                "x": round(x, 3),
                "y": round(y, 3),
                "z": round(_finite_float_default(raw.get("z"), 0.0), 3),
                "yaw": _finite_float(raw.get("yaw")),
                "tags": [str(tag) for tag in tags],
                "source": _string_or_none(raw.get("source")),
                "ts": _finite_float(raw.get("ts")),
                "metadata": _metadata(raw.get("metadata")),
            }
        )
    return {
        "schema_version": 1,
        "locations": locations,
        "count": len(locations),
        "frame_id": TELEMETRY_MAP_FRAME_ID,
        "ts": time.time(),
        "source": "tagged_locations",
    }
