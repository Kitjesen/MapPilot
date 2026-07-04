"""Shared typed DDS message codecs."""

from __future__ import annotations

from typing import Any

from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3


def from_dds_time(stamp: Any) -> float:
    sec = float(getattr(stamp, "sec", 0.0))
    nanosec = float(getattr(stamp, "nanosec", 0.0))
    return sec + nanosec / 1_000_000_000.0


def from_dds_pose_stamped(msg: Any, frame_id: str) -> PoseStamped:
    header = getattr(msg, "header", None)
    pose = getattr(msg, "pose", None)
    if pose is None:
        raise TypeError("DDS PoseStamped missing pose")
    position = getattr(pose, "position")
    orientation = getattr(pose, "orientation", None)
    return PoseStamped(
        pose=Pose(
            position=Vector3(
                float(getattr(position, "x", 0.0)),
                float(getattr(position, "y", 0.0)),
                float(getattr(position, "z", 0.0)),
            ),
            orientation=Quaternion(
                float(getattr(orientation, "x", 0.0)),
                float(getattr(orientation, "y", 0.0)),
                float(getattr(orientation, "z", 0.0)),
                float(getattr(orientation, "w", 1.0)),
            ),
        ),
        ts=from_dds_time(getattr(header, "stamp", None)),
        frame_id=str(getattr(header, "frame_id", "") or frame_id),
    )


def from_dds_string(msg: Any) -> str:
    return str(getattr(msg, "data", msg) or "")
