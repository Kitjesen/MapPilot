"""Duck-typed TF conversion helpers for ROS/DDS-shaped messages."""

from __future__ import annotations

import time
from collections.abc import Iterable
from typing import Any

from ..msgs.geometry import Quaternion, Transform, Vector3
from ..runtime_interface import normalize_frame_id
from .buffer import TFMessage
from .tree import FrameError


def stamp_seconds(header: Any, *, default: float | None = None) -> float:
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return time.time() if default is None else float(default)
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1e-9


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
    "stamp_seconds",
    "tf_message_from_any",
    "transform_from_stamped",
]
