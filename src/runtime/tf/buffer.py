"""tf2_ros-like API on top of LingTu's ROS-free FrameTree."""

from __future__ import annotations

import json
import time as _time
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from threading import RLock
from typing import Any

from ..clock import Clock, clock as default_clock
from ..msgs.geometry import Pose, PoseStamped, Quaternion, Transform, Vector3
from ..msgs.nav import Odometry, Path
from ..msgs.sensor import PointCloud2
from .tree import FrameTree

TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"


@dataclass(frozen=True)
class TFMessage:
    transforms: tuple[Transform, ...]

    def to_dict(self) -> dict[str, Any]:
        return {"transforms": [transform.to_dict() for transform in self.transforms]}

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> TFMessage:
        return cls(
            tuple(Transform.from_dict(item) for item in data.get("transforms", []))
        )


class TfBus:
    """Small latched topic bus for /tf and /tf_static inside one runtime."""

    def __init__(self) -> None:
        self._lock = RLock()
        self._subscribers: dict[str, list[Callable[[TFMessage], None]]] = {
            TF_TOPIC: [],
            TF_STATIC_TOPIC: [],
        }
        self._static_transforms: dict[tuple[str, str], Transform] = {}

    def publish(self, topic: str, transforms: Transform | Iterable[Transform] | TFMessage) -> None:
        msg = _tf_message(transforms)
        with self._lock:
            if topic == TF_STATIC_TOPIC:
                for transform in msg.transforms:
                    self._static_transforms[
                        (transform.frame_id, transform.child_frame_id)
                    ] = transform
            callbacks = list(self._subscribers.setdefault(topic, []))
        for callback in callbacks:
            callback(msg)

    def subscribe(
        self,
        topic: str,
        callback: Callable[[TFMessage], None],
        *,
        replay_static: bool = True,
    ) -> Callable[[], None]:
        with self._lock:
            self._subscribers.setdefault(topic, []).append(callback)
            replay = (
                [TFMessage(tuple(self._static_transforms.values()))]
                if topic == TF_STATIC_TOPIC and replay_static and self._static_transforms
                else []
            )

        for msg in replay:
            callback(msg)

        def unsubscribe() -> None:
            with self._lock:
                callbacks = self._subscribers.get(topic, [])
                if callback in callbacks:
                    callbacks.remove(callback)

        return unsubscribe


default_bus = TfBus()


class Buffer:
    """tf2_ros.Buffer-style wrapper with wait and fixed-frame lookup."""

    def __init__(
        self,
        tree: FrameTree | None = None,
        *,
        cache_time_s: float = 10.0,
        clock: Clock = default_clock,
    ) -> None:
        self.tree = tree or FrameTree(cache_time_s=cache_time_s)
        self.clock = clock

    def now(self) -> float:
        return self.clock.now()

    def set_transform(
        self,
        transform: Transform,
        authority: str = "",
        *,
        is_static: bool = False,
    ) -> None:
        self.tree.set_transform(transform, is_static=is_static, authority=authority)

    def set_static_transform(self, transform: Transform, authority: str = "") -> None:
        self.tree.set_static_transform(transform, authority=authority)

    def lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: float | None = None,
        timeout: float = 0.0,
    ) -> Transform:
        ts = _lookup_time(time)
        if timeout > 0.0:
            self.wait_for_transform(target_frame, source_frame, ts, timeout)
        return self.tree.lookup(target_frame, source_frame, ts=ts)

    def can_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: float | None = None,
        timeout: float = 0.0,
    ) -> bool:
        ts = _lookup_time(time)
        if timeout > 0.0:
            return self.wait_for_transform(target_frame, source_frame, ts, timeout)
        return self.tree.can_transform(target_frame, source_frame, ts=ts)

    def frame_exists(self, frame_id: str) -> bool:
        return frame_id in self.tree.frames()

    def clear_dynamic(self) -> None:
        self.tree.clear_dynamic()

    def wait_for_transform(
        self,
        target_frame: str,
        source_frame: str,
        time: float | None = None,
        timeout: float = 1.0,
        poll_interval: float = 0.01,
    ) -> bool:
        ts = _lookup_time(time)
        deadline = _time.monotonic() + max(0.0, float(timeout))
        while True:
            if self.tree.can_transform(target_frame, source_frame, ts=ts):
                return True
            if _time.monotonic() >= deadline:
                return False
            self.clock.sleep(min(poll_interval, max(0.0, deadline - _time.monotonic())))

    def lookup_transform_full(
        self,
        target_frame: str,
        target_time: float | None,
        source_frame: str,
        source_time: float | None,
        fixed_frame: str,
        timeout: float = 0.0,
    ) -> Transform:
        target_ts = _lookup_time(target_time)
        source_ts = _lookup_time(source_time)
        if timeout > 0.0:
            self.wait_for_transform(fixed_frame, source_frame, source_ts, timeout)
            self.wait_for_transform(target_frame, fixed_frame, target_ts, timeout)
        source_to_fixed = self.tree.lookup(fixed_frame, source_frame, ts=source_ts)
        fixed_to_target = self.tree.lookup(target_frame, fixed_frame, ts=target_ts)
        return fixed_to_target + source_to_fixed

    def transform(
        self,
        value: PoseStamped | PointCloud2 | Odometry | Path,
        target_frame: str,
        timeout: float = 0.0,
    ) -> PoseStamped | PointCloud2 | Odometry | Path:
        if isinstance(value, PoseStamped):
            transform = self.lookup_transform(
                target_frame,
                value.frame_id,
                value.ts,
                timeout,
            )
            return PoseStamped(
                pose=_apply_pose(transform, value.pose),
                ts=value.ts,
                frame_id=target_frame,
            )
        if isinstance(value, PointCloud2):
            transform = self.lookup_transform(
                target_frame,
                value.frame_id,
                getattr(value, "ts", None),
                timeout,
            )
            converted = value.transform(transform.to_matrix())
            converted.frame_id = target_frame
            return converted
        if isinstance(value, Odometry):
            transform = self.lookup_transform(
                target_frame,
                value.frame_id,
                value.ts,
                timeout,
            )
            return Odometry(
                pose=_apply_pose(transform, value.pose),
                twist=value.twist,
                ts=value.ts,
                frame_id=target_frame,
                child_frame_id=value.child_frame_id,
            )
        if isinstance(value, Path):
            return Path(
                poses=[
                    self.transform(pose, target_frame, timeout) for pose in value.poses
                ],
                ts=value.ts,
                frame_id=target_frame,
            )
        raise TypeError(f"unsupported transform type: {type(value).__name__}")

    def transform_full(
        self,
        value: PoseStamped,
        target_frame: str,
        target_time: float | None,
        fixed_frame: str,
        timeout: float = 0.0,
    ) -> PoseStamped:
        transform = self.lookup_transform_full(
            target_frame,
            target_time,
            value.frame_id,
            value.ts,
            fixed_frame,
            timeout,
        )
        return PoseStamped(
            pose=_apply_pose(transform, value.pose),
            ts=_lookup_time(target_time) or value.ts,
            frame_id=target_frame,
        )

    def frames(self) -> tuple[str, ...]:
        return self.tree.frames()

    def snapshot(self) -> dict[str, Any]:
        return self.tree.snapshot()

    def all_frames_as_yaml(self) -> str:
        # ponytail: JSON is valid YAML 1.2; add PyYAML only if consumers need YAML tags/anchors.
        return json.dumps(self.snapshot(), ensure_ascii=False, indent=2)

    def all_frames_as_string(self) -> str:
        return "\n".join(self.tree.frames())

    def get_latest_common_time(self, target_frame: str, source_frame: str) -> float:
        path = self.tree.lookup_path(target_frame, source_frame)
        snapshot = self.tree.snapshot()
        latest_times: list[float] = []
        edges = {
            (edge["parent"], edge["child"]): edge
            for edge in snapshot.get("edges", ())
        }
        for parent, child in zip(path[1:], path[:-1]):
            edge = edges.get((parent, child)) or edges.get((child, parent))
            if edge and not edge.get("is_static", False):
                latest_times.append(float(edge["latest_ts"]))
        return min(latest_times) if latest_times else 0.0

    lookupTransform = lookup_transform
    canTransform = can_transform
    lookupTransformFull = lookup_transform_full
    waitForTransform = wait_for_transform
    allFramesAsYAML = all_frames_as_yaml
    allFramesAsString = all_frames_as_string
    getLatestCommonTime = get_latest_common_time


class TransformBroadcaster:
    def __init__(self, bus: TfBus = default_bus) -> None:
        self.bus = bus

    def send_transform(self, transform: Transform | Iterable[Transform]) -> None:
        self.bus.publish(TF_TOPIC, transform)

    sendTransform = send_transform


class StaticTransformBroadcaster:
    def __init__(self, bus: TfBus = default_bus) -> None:
        self.bus = bus

    def send_transform(self, transform: Transform | Iterable[Transform]) -> None:
        self.bus.publish(TF_STATIC_TOPIC, transform)

    sendTransform = send_transform


class TransformListener:
    def __init__(
        self,
        buffer: Buffer,
        bus: TfBus = default_bus,
        *,
        spin_thread: bool = False,
    ) -> None:
        self.buffer = buffer
        self.bus = bus
        # ponytail: accepted for tf2_ros API shape; add a worker thread only if callbacks become async.
        self.spin_thread = spin_thread
        self._unsubscribers = [
            bus.subscribe(TF_TOPIC, self._on_tf),
            bus.subscribe(TF_STATIC_TOPIC, self._on_static_tf),
        ]

    def close(self) -> None:
        for unsubscribe in self._unsubscribers:
            unsubscribe()
        self._unsubscribers.clear()

    def _on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.buffer.set_transform(transform, "tf_bus")

    def _on_static_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.buffer.set_static_transform(transform, "tf_static_bus")


def _tf_message(value: Transform | Iterable[Transform] | TFMessage) -> TFMessage:
    if isinstance(value, TFMessage):
        return value
    if isinstance(value, Transform):
        return TFMessage((value,))
    return TFMessage(tuple(value))


def _lookup_time(value: float | None) -> float | None:
    if value is None or value == 0.0:
        return None
    return float(value)


def _apply_pose(transform: Transform, pose: Pose) -> Pose:
    return Pose(
        position=(
            transform.translation
            + transform.rotation.rotate_vector(Vector3(pose.position))
        ),
        orientation=transform.rotation * Quaternion(pose.orientation),
    )


__all__ = [
    "Buffer",
    "StaticTransformBroadcaster",
    "TFMessage",
    "TF_STATIC_TOPIC",
    "TF_TOPIC",
    "TfBus",
    "TransformBroadcaster",
    "TransformListener",
    "default_bus",
]
