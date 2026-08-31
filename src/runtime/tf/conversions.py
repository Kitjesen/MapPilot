"""Duck-typed TF conversion helpers for ROS/DDS-shaped messages."""

from __future__ import annotations

import math
import time
from collections.abc import Iterable, Mapping
from typing import Any

from ..msgs.geometry import Quaternion, Transform, Vector3
from ..runtime_interface import map_frame_id, normalize_frame_id, odom_frame_id
from .buffer import TFMessage
from .tree import FrameError


def stamp_seconds(header: Any, *, default: float | None = None) -> float:
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return time.time() if default is None else float(default)
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1e-9


def map_from_odom_transform_from_mapping(
    value: Any,
    *,
    require_valid: bool = True,
) -> Transform | None:
    """Return canonical ``T_map_from_odom`` or reject an ambiguous payload.

    The transform stores the pose of ``odom`` in ``map`` and converts
    odom-frame data into map-frame data:
    ``p_map = T_map_from_odom * p_odom``.
    """

    if not isinstance(value, Mapping):
        return None
    if value.get("valid") is False or (require_valid and value.get("valid") is not True):
        return None
    if value.get("frame_id") != map_frame_id():
        return None
    if value.get("child_frame_id") != odom_frame_id():
        return None

    translation = value.get("translation")
    rotation = value.get("rotation")
    flat_fields = ("tx", "ty", "tz", "qx", "qy", "qz", "qw")
    has_nested_schema = "translation" in value or "rotation" in value
    has_flat_schema = any(field in value for field in flat_fields)
    if has_nested_schema == has_flat_schema:
        return None
    try:
        stamps = [float(value[key]) for key in ("stamp_s", "ts") if key in value]
        if not stamps or any(stamp <= 0.0 or not math.isfinite(stamp) for stamp in stamps):
            return None
        if len(stamps) == 2 and not math.isclose(
            stamps[0], stamps[1], rel_tol=0.0, abs_tol=1e-9
        ):
            return None
        stamp_s = stamps[0]

        if has_nested_schema:
            if not isinstance(translation, Mapping) or not isinstance(rotation, Mapping):
                return None
            if not {"x", "y", "z"} <= translation.keys() or not {
                "x",
                "y",
                "z",
                "w",
            } <= rotation.keys():
                return None
            numbers = tuple(float(translation[key]) for key in ("x", "y", "z")) + tuple(
                float(rotation[key]) for key in ("x", "y", "z", "w")
            )
        else:
            if not set(flat_fields) <= value.keys():
                return None
            numbers = tuple(float(value[key]) for key in flat_fields)
        if not all(math.isfinite(number) for number in numbers):
            return None
        translation_xyz = Vector3(*numbers[:3])
        rotation_xyzw = Quaternion(*numbers[3:]).normalize()
    except (TypeError, ValueError, ZeroDivisionError):
        return None

    return Transform(
        translation=translation_xyz,
        rotation=rotation_xyzw,
        frame_id=map_frame_id(),
        child_frame_id=odom_frame_id(),
        ts=stamp_s,
    )


def map_from_odom_transform_to_dict(transform: Transform) -> dict[str, Any]:
    """Serialize validated ``T_map_from_odom`` using the Module-port schema."""

    if not isinstance(transform, Transform):
        raise TypeError("map<-odom transform must be a Transform")
    if transform.frame_id != map_frame_id() or transform.child_frame_id != odom_frame_id():
        raise ValueError("map<-odom transform must use exact map and odom frame ids")
    numbers = (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )
    try:
        stamp_s = float(transform.ts)
    except (TypeError, ValueError) as exc:
        raise ValueError("map<-odom transform timestamp must be numeric") from exc
    if not all(math.isfinite(float(number)) for number in numbers) or not math.isfinite(
        stamp_s
    ):
        raise ValueError("map<-odom transform values must be finite")
    if stamp_s <= 0.0:
        raise ValueError("map<-odom transform timestamp must be positive")
    try:
        rotation = transform.rotation.normalize()
    except ZeroDivisionError as exc:
        raise ValueError("map<-odom transform quaternion must be non-zero") from exc

    return {
        "valid": True,
        "frame_id": map_frame_id(),
        "child_frame_id": odom_frame_id(),
        "tx": transform.translation.x,
        "ty": transform.translation.y,
        "tz": transform.translation.z,
        "qx": rotation.x,
        "qy": rotation.y,
        "qz": rotation.z,
        "qw": rotation.w,
        "ts": stamp_s,
    }


def map_from_odom_transform_from_stamped(value: Any) -> Transform | None:
    """Convert one raw stamped payload without inventing source data."""

    if isinstance(value, Transform):
        try:
            payload = map_from_odom_transform_to_dict(value)
        except (TypeError, ValueError):
            return None
        return map_from_odom_transform_from_mapping(payload)

    header = getattr(value, "header", None)
    if getattr(header, "frame_id", None) != map_frame_id():
        return None
    if getattr(value, "child_frame_id", None) != odom_frame_id():
        return None

    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    try:
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        raw_transform = value.transform
        translation = raw_transform.translation
        rotation = raw_transform.rotation
        numbers = tuple(float(getattr(translation, field)) for field in ("x", "y", "z"))
        numbers += tuple(float(getattr(rotation, field)) for field in ("x", "y", "z", "w"))
    except (AttributeError, TypeError, ValueError):
        return None

    return map_from_odom_transform_from_mapping(
        {
            "valid": True,
            "frame_id": map_frame_id(),
            "child_frame_id": odom_frame_id(),
            "tx": numbers[0],
            "ty": numbers[1],
            "tz": numbers[2],
            "qx": numbers[3],
            "qy": numbers[4],
            "qz": numbers[5],
            "qw": numbers[6],
            "ts": stamp_s,
        }
    )


def strict_transform_from_stamped(value: Any) -> Transform | None:
    """Convert a dynamic TF sample without defaulting frames, values, or time."""

    if isinstance(value, Transform):
        raw_parent = value.frame_id
        raw_child = value.child_frame_id
        stamp_s = value.ts
        translation = value.translation
        rotation = value.rotation
    else:
        header = getattr(value, "header", None)
        raw_parent = getattr(header, "frame_id", None)
        raw_child = getattr(value, "child_frame_id", None)
        stamp = getattr(header, "stamp", None)
        raw_transform = getattr(value, "transform", None)
        if stamp is None or raw_transform is None:
            return None
        try:
            stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except (AttributeError, TypeError, ValueError):
            return None
        translation = getattr(raw_transform, "translation", None)
        rotation = getattr(raw_transform, "rotation", None)

    parent = normalize_frame_id(raw_parent)
    child = normalize_frame_id(raw_child)
    if (
        parent is None
        or child is None
        or raw_parent != parent
        or raw_child != child
        or translation is None
        or rotation is None
    ):
        return None
    try:
        stamp_s = float(stamp_s)
        numbers = tuple(float(getattr(translation, field)) for field in ("x", "y", "z"))
        numbers += tuple(float(getattr(rotation, field)) for field in ("x", "y", "z", "w"))
        if stamp_s <= 0.0 or not math.isfinite(stamp_s):
            return None
        if not all(math.isfinite(number) for number in numbers):
            return None
        rotation_xyzw = Quaternion(*numbers[3:]).normalize()
    except (AttributeError, TypeError, ValueError, ZeroDivisionError):
        return None

    return Transform(
        translation=Vector3(*numbers[:3]),
        rotation=rotation_xyzw,
        frame_id=parent,
        child_frame_id=child,
        ts=stamp_s,
    )


def transform_from_stamped(value: Any) -> Transform:
    """Convert core/ROS/DDS-shaped TransformStamped into core Transform."""

    if isinstance(value, Transform):
        return value

    header = getattr(value, "header", None)
    raw_parent = getattr(header, "frame_id", None) or getattr(value, "frame_id", None)
    raw_child = getattr(value, "child_frame_id", None)
    parent = normalize_frame_id(raw_parent)
    child = normalize_frame_id(raw_child)
    if parent is None or child is None:
        raise FrameError("transform must include parent and child frame ids")

    raw_transform = getattr(value, "transform", value)
    translation = getattr(raw_transform, "translation", None)
    rotation = getattr(raw_transform, "rotation", None)
    if translation is None or rotation is None:
        raise FrameError("transform must include translation and rotation")

    return Transform(
        translation=Vector3(
            float(getattr(translation, "x", 0.0)),
            float(getattr(translation, "y", 0.0)),
            float(getattr(translation, "z", 0.0)),
        ),
        rotation=Quaternion(
            float(getattr(rotation, "x", 0.0)),
            float(getattr(rotation, "y", 0.0)),
            float(getattr(rotation, "z", 0.0)),
            float(getattr(rotation, "w", 1.0)),
        ),
        frame_id=parent,
        child_frame_id=child,
        ts=float(getattr(value, "ts", 0.0) or stamp_seconds(header)),
    )


def iter_tf_transforms(value: Any) -> Iterable[Transform]:
    """Yield core Transforms from TFMessage-like objects or iterables."""

    if isinstance(value, TFMessage):
        yield from value.transforms
        return
    if isinstance(value, Transform):
        yield value
        return

    transforms = getattr(value, "transforms", None)
    if transforms is None:
        yield transform_from_stamped(value)
        return
    for item in transforms:
        yield transform_from_stamped(item)


def tf_message_from_any(value: Any) -> TFMessage:
    return TFMessage(tuple(iter_tf_transforms(value)))


__all__ = [
    "iter_tf_transforms",
    "map_from_odom_transform_from_mapping",
    "map_from_odom_transform_from_stamped",
    "map_from_odom_transform_to_dict",
    "stamp_seconds",
    "strict_transform_from_stamped",
    "tf_message_from_any",
    "transform_from_stamped",
]
