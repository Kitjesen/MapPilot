"""ROS-free frame tree for LingTu runtime transforms.

This module owns the small subset of TF2 semantics used by normal Modules:
current transform storage, frame lookup, pose conversion, and point-cloud
coordinate conversion. ROS2 adapters can feed it ``Transform`` messages, but
the core implementation has no ROS dependency.
"""

from __future__ import annotations

import math
from bisect import bisect_left
from collections import deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from threading import RLock
from typing import Any

from ..msgs.geometry import Pose, PoseStamped, Quaternion, Transform, Vector3
from ..msgs.nav import Odometry
from ..msgs.numpy_compat import np, numpy_import_is_safe
from ..runtime_interface import (
    FRAMES,
    body_frame_id,
    camera_frame_id,
    lidar_frame_id,
    normalize_frame_id,
    real_lidar_frame_id,
)


class FrameError(ValueError):
    """Base error for frame-tree failures."""


class UnknownFrameError(FrameError):
    """Raised when a requested frame has never been registered."""


class NoTransformError(FrameError):
    """Raised when two known frames are not connected."""


class ExtrapolationError(FrameError):
    """Raised when a requested timestamp is outside the cached edge range."""


TransformException = FrameError
LookupException = UnknownFrameError
ConnectivityException = NoTransformError


@dataclass
class _EdgeHistory:
    is_static: bool = False
    authority: str = ""
    samples: list[Transform] = field(default_factory=list)

    def add(self, transform: Transform, cache_time_s: float) -> None:
        if self.is_static:
            self.samples = [transform]
            return

        times = [sample.ts for sample in self.samples]
        index = bisect_left(times, transform.ts)
        if index < len(self.samples) and abs(self.samples[index].ts - transform.ts) < 1e-9:
            self.samples[index] = transform
        else:
            self.samples.insert(index, transform)

        if cache_time_s > 0.0 and self.samples:
            cutoff = self.samples[-1].ts - cache_time_s
            while len(self.samples) > 1 and self.samples[0].ts < cutoff:
                self.samples.pop(0)

    def at(self, ts: float | None) -> Transform:
        if not self.samples:
            raise NoTransformError("edge has no transform samples")
        if self.is_static or ts is None:
            return self.samples[-1]
        if len(self.samples) == 1:
            only = self.samples[0]
            if abs(only.ts - ts) < 1e-9:
                return only
            raise ExtrapolationError(
                f"requested {ts:.6f}, only sample is {only.ts:.6f}"
            )

        first = self.samples[0]
        last = self.samples[-1]
        if ts < first.ts - 1e-9:
            raise ExtrapolationError(
                f"requested {ts:.6f} before first sample {first.ts:.6f}"
            )
        if ts > last.ts + 1e-9:
            raise ExtrapolationError(
                f"requested {ts:.6f} after last sample {last.ts:.6f}"
            )

        times = [sample.ts for sample in self.samples]
        index = bisect_left(times, ts)
        if index < len(self.samples) and abs(self.samples[index].ts - ts) < 1e-9:
            return self.samples[index]
        return _interpolate_transform(self.samples[index - 1], self.samples[index], ts)


class FrameTree:
    """Current-transform tree with TF-style parent/child semantics.

    ``Transform(frame_id="body", child_frame_id="lidar")`` has topology
    ``body -> lidar`` and numeric direction ``body <- lidar``. Applying it
    converts a point from the child frame into the parent frame. Lookups return
    ``T_target_from_source``.

    Dynamic edges keep a small timestamped cache and support interpolation.
    Static edges are valid for all timestamps.
    """

    def __init__(
        self,
        transforms: Iterable[Transform] | None = None,
        *,
        cache_time_s: float = 10.0,
    ) -> None:
        self._lock = RLock()
        self._edges: dict[str, dict[str, Transform]] = {}
        self._histories: dict[tuple[str, str], _EdgeHistory] = {}
        self._frames: set[str] = set()
        self._parent_by_child: dict[str, str] = {}
        self._aliases: dict[str, str] = {}
        self._cache_time_s = max(0.0, float(cache_time_s))
        if transforms is not None:
            for transform in transforms:
                self.set_transform(transform)

    @classmethod
    def from_robot_config(
        cls,
        config_getter: Callable[[], Any] | None = None,
    ) -> FrameTree:
        """Build a tree with static body->sensor extrinsics from robot config."""

        if config_getter is None:
            from ..config import get_config

            config_getter = get_config

        cfg = config_getter()
        tree = cls()
        tree.add_runtime_aliases()

        body_frame = body_frame_id()
        lidar_cfg = cfg.lidar
        lidar_child = getattr(lidar_cfg, "frame_id", None) or real_lidar_frame_id()
        tree.set_static_transform(
            Transform(
                translation=Vector3(
                    getattr(lidar_cfg, "offset_x", 0.0),
                    getattr(lidar_cfg, "offset_y", 0.0),
                    getattr(lidar_cfg, "offset_z", 0.0),
                ),
                rotation=Quaternion.from_euler(
                    getattr(lidar_cfg, "roll", 0.0),
                    getattr(lidar_cfg, "pitch", 0.0),
                    getattr(lidar_cfg, "yaw", 0.0),
                ),
                frame_id=body_frame,
                child_frame_id=lidar_child,
            )
        )
        tree.set_alias(lidar_frame_id(), lidar_child)

        camera_cfg = cfg.camera
        tree.set_static_transform(
            Transform(
                translation=Vector3(
                    getattr(camera_cfg, "position_x", 0.0),
                    getattr(camera_cfg, "position_y", 0.0),
                    getattr(camera_cfg, "position_z", 0.0),
                ),
                rotation=Quaternion.from_euler(
                    getattr(camera_cfg, "roll", 0.0),
                    getattr(camera_cfg, "pitch", 0.0),
                    getattr(camera_cfg, "yaw", 0.0),
                ),
                frame_id=body_frame,
                child_frame_id=camera_frame_id(),
            )
        )
        return tree

    def add_runtime_aliases(self) -> None:
        """Register canonical runtime frame aliases as identity transforms."""

        for alias in FRAMES.body_aliases:
            self.set_alias(FRAMES.body, alias)
        for alias in FRAMES.lidar_aliases:
            self.set_alias(FRAMES.lidar, alias)

    def frames(self) -> tuple[str, ...]:
        """Return all known frame names."""

        with self._lock:
            return tuple(sorted(self._frames | set(self._aliases)))

    def lookup_path(
        self,
        target_frame: str,
        source_frame: str,
        *,
        ts: float | None = None,
    ) -> tuple[str, ...]:
        """Return the resolved source->target frame path for diagnostics."""

        target = self._frame_id(target_frame, "target_frame")
        source = self._frame_id(source_frame, "source_frame")
        if target == source:
            return (source,)

        with self._lock:
            if source not in self._frames:
                raise UnknownFrameError(f"unknown source frame {source!r}")
            if target not in self._frames:
                raise UnknownFrameError(f"unknown target frame {target!r}")

            queue: deque[tuple[str, tuple[str, ...]]] = deque([(source, (source,))])
            visited = {source}
            while queue:
                current_frame, current_path = queue.popleft()
                for next_frame in self._edges.get(current_frame, {}):
                    if next_frame in visited:
                        continue
                    self._edge_at_time(current_frame, next_frame, ts)
                    next_path = current_path + (next_frame,)
                    if next_frame == target:
                        return next_path
                    visited.add(next_frame)
                    queue.append((next_frame, next_path))

        raise NoTransformError(f"no transform from {source!r} to {target!r}")

    def parent_of(self, child_frame: str) -> str | None:
        """Return the canonical direct parent for a child frame, if known."""

        child = self._frame_id(child_frame, "child_frame")
        with self._lock:
            return self._parent_by_child.get(child)

    def snapshot(self) -> dict[str, Any]:
        """Return a lightweight diagnostic snapshot of frames, aliases, and edges."""

        with self._lock:
            edges: list[dict[str, Any]] = []
            for child, parent in sorted(self._parent_by_child.items()):
                history = self._histories.get((child, parent))
                if history is None or not history.samples:
                    continue
                latest = history.samples[-1]
                first = history.samples[0]
                edges.append(
                    {
                        "parent": parent,
                        "child": child,
                        "is_static": history.is_static,
                        "authority": history.authority,
                        "sample_count": len(history.samples),
                        "first_ts": first.ts,
                        "latest_ts": latest.ts,
                        "translation": latest.translation.to_list(),
                        "rotation": latest.rotation.to_list(),
                    }
                )
            return {
                "frames": tuple(sorted(self._frames)),
                "aliases": dict(sorted(self._aliases.items())),
                "cache_time_s": self._cache_time_s,
                "edges": tuple(edges),
            }

    def clear_dynamic(self) -> None:
        """Drop dynamic transforms while preserving static mounts and aliases."""

        with self._lock:
            dynamic_children: set[str] = set()
            for child, parent in tuple(self._parent_by_child.items()):
                history = self._histories.get((child, parent))
                if history is not None and not history.is_static:
                    dynamic_children.add(child)

            for child in dynamic_children:
                parent = self._parent_by_child.pop(child, None)
                if parent is None:
                    continue
                self._remove_direct_edge(child, parent)
                self._remove_direct_edge(parent, child)

    def remove_transform(
        self,
        parent_frame: str,
        child_frame: str,
        *,
        dynamic_only: bool = False,
    ) -> bool:
        """Remove one direct parent/child transform, if present."""

        parent = self._frame_id(parent_frame, "parent_frame")
        child = self._frame_id(child_frame, "child_frame")
        with self._lock:
            if self._parent_by_child.get(child) != parent:
                return False
            history = self._histories.get((child, parent))
            if history is None or (dynamic_only and history.is_static):
                return False
            self._parent_by_child.pop(child, None)
            self._remove_direct_edge(child, parent)
            self._remove_direct_edge(parent, child)
            return True

    def set_alias(self, canonical_frame: str, alias_frame: str) -> None:
        """Treat two frame names as the same physical coordinate frame."""

        canonical = _require_frame_id(canonical_frame, "canonical_frame")
        alias = _require_frame_id(alias_frame, "alias_frame")
        with self._lock:
            canonical = self._canonical_frame(canonical)
            self._frames.add(canonical)
            if canonical != alias:
                self._aliases[alias] = canonical

    def set_identity(
        self,
        parent_frame: str,
        child_frame: str,
        *,
        is_static: bool = True,
    ) -> None:
        """Register an identity transform between parent and child frames."""

        self.set_transform(
            Transform(
                frame_id=parent_frame,
                child_frame_id=child_frame,
            ),
            is_static=is_static,
        )

    def set_static_transform(self, transform: Transform, *, authority: str = "") -> None:
        """Register a transform that is valid for every timestamp."""

        self.set_transform(transform, is_static=True, authority=authority)

    def set_transform(
        self,
        transform: Transform,
        *,
        is_static: bool = False,
        authority: str = "",
    ) -> None:
        """Register or replace the latest transform for one parent/child edge."""

        parent = self._frame_id(transform.frame_id, "transform.frame_id")
        child = self._frame_id(transform.child_frame_id, "transform.child_frame_id")
        if parent == child:
            with self._lock:
                self._frames.add(parent)
            return

        normalized = Transform(
            translation=Vector3(transform.translation),
            rotation=Quaternion(transform.rotation).normalize(),
            frame_id=parent,
            child_frame_id=child,
            ts=transform.ts,
        )
        inverse = normalized.inverse()

        with self._lock:
            old_parent = self._parent_by_child.get(child)
            if old_parent is not None and old_parent != parent:
                self._remove_direct_edge(child, old_parent)
                self._remove_direct_edge(old_parent, child)
            self._parent_by_child[child] = parent
            self._frames.update((parent, child))
            self._store_direct_edge(
                child,
                parent,
                normalized,
                is_static=is_static,
                authority=authority,
            )
            self._store_direct_edge(
                parent,
                child,
                inverse,
                is_static=is_static,
                authority=authority,
            )

    def update_odometry(self, odometry: Odometry) -> None:
        """Update the dynamic odometry parent->body transform."""

        self.set_transform(
            Transform(
                translation=Vector3(odometry.pose.position),
                rotation=Quaternion(odometry.pose.orientation),
                frame_id=odometry.frame_id,
                child_frame_id=odometry.child_frame_id,
                ts=odometry.ts,
            )
        )

    def lookup(
        self,
        target_frame: str,
        source_frame: str,
        *,
        ts: float | None = None,
    ) -> Transform:
        """Return the transform that converts source-frame data to target frame."""

        target = self._frame_id(target_frame, "target_frame")
        source = self._frame_id(source_frame, "source_frame")
        if target == source:
            return Transform(frame_id=target, child_frame_id=source, ts=ts or 0.0)

        with self._lock:
            if source not in self._frames:
                raise UnknownFrameError(f"unknown source frame {source!r}")
            if target not in self._frames:
                raise UnknownFrameError(f"unknown target frame {target!r}")

            queue: deque[tuple[str, Transform]] = deque(
                [(source, Transform(frame_id=source, child_frame_id=source))]
            )
            visited = {source}
            while queue:
                current_frame, current_transform = queue.popleft()
                for next_frame in self._edges.get(current_frame, {}):
                    if next_frame in visited:
                        continue
                    edge = self._edge_at_time(current_frame, next_frame, ts)
                    composed = edge + current_transform
                    if next_frame == target:
                        return composed
                    visited.add(next_frame)
                    queue.append((next_frame, composed))

        raise NoTransformError(f"no transform from {source!r} to {target!r}")

    def can_transform(
        self,
        target_frame: str,
        source_frame: str,
        *,
        ts: float | None = None,
    ) -> bool:
        """Return True if a lookup between two frames succeeds."""

        try:
            self.lookup(target_frame, source_frame, ts=ts)
        except FrameError:
            return False
        return True

    def transform_point(
        self,
        point: Vector3 | tuple[float, float, float] | list[float],
        target_frame: str,
        source_frame: str,
        *,
        ts: float | None = None,
    ) -> Vector3:
        """Convert one point from source coordinates into target coordinates."""

        transform = self.lookup(target_frame, source_frame, ts=ts)
        local = Vector3(point)
        return transform.translation + transform.rotation.rotate_vector(local)

    def transform_pose(
        self,
        pose: Pose,
        target_frame: str,
        source_frame: str,
        *,
        ts: float | None = None,
    ) -> Pose:
        """Convert a pose from source coordinates into target coordinates."""

        transform = self.lookup(target_frame, source_frame, ts=ts)
        return Pose(
            position=(
                transform.translation
                + transform.rotation.rotate_vector(Vector3(pose.position))
            ),
            orientation=transform.rotation * Quaternion(pose.orientation),
        )

    def transform_pose_stamped(
        self,
        pose: PoseStamped,
        target_frame: str,
    ) -> PoseStamped:
        """Convert a stamped pose into the target frame."""

        target = self._frame_id(target_frame, "target_frame")
        return PoseStamped(
            pose=self.transform_pose(pose.pose, target, pose.frame_id, ts=pose.ts),
            ts=pose.ts,
            frame_id=target,
        )

    def transform_points(
        self,
        points: Any,
        target_frame: str,
        source_frame: str,
        *,
        ts: float | None = None,
    ) -> Any:
        """Convert an ``N x 3`` or ``N x 4`` point array to the target frame."""

        if not numpy_import_is_safe():
            raise ImportError("NumPy import is unsafe in this host interpreter")

        arr = np.asarray(points)
        if arr.ndim != 2 or arr.shape[1] not in (3, 4):
            raise ValueError(f"points must be (N,3) or (N,4), got {arr.shape}")

        transform = self.lookup(target_frame, source_frame, ts=ts)
        xyz = arr[:, :3].astype(np.float64)
        rotated = (transform.rotation.to_rotation_matrix() @ xyz.T).T
        translated = rotated + np.asarray(transform.translation.to_list(), dtype=np.float64)

        out_dtype = arr.dtype if np.issubdtype(arr.dtype, np.floating) else np.float32
        result = arr.astype(out_dtype, copy=True)
        result[:, :3] = translated.astype(out_dtype, copy=False)
        return result

    def transform_cloud(self, cloud: Any, target_frame: str) -> Any:
        """Convert a ``PointCloud2`` payload to the target frame."""

        target = self._frame_id(target_frame, "target_frame")
        transform = self.lookup(target, cloud.frame_id, ts=getattr(cloud, "ts", None))
        converted = cloud.transform(transform.to_matrix())
        converted.frame_id = target
        return converted

    def edge_authority(self, target_frame: str, source_frame: str) -> str:
        """Return the publisher label for a direct edge, if one was provided."""

        target = self._frame_id(target_frame, "target_frame")
        source = self._frame_id(source_frame, "source_frame")
        with self._lock:
            history = self._histories.get((source, target))
            return history.authority if history is not None else ""

    def _frame_id(self, frame_id: str | None, role: str) -> str:
        return self._canonical_frame(_require_frame_id(frame_id, role))

    def _canonical_frame(self, frame_id: str) -> str:
        # ponytail: tiny alias chain; replace with union-find only if aliases grow.
        with self._lock:
            seen: set[str] = set()
            current = frame_id
            while current in self._aliases and current not in seen:
                seen.add(current)
                current = self._aliases[current]
            return current

    def _store_direct_edge(
        self,
        source: str,
        target: str,
        transform: Transform,
        *,
        is_static: bool,
        authority: str,
    ) -> None:
        key = (source, target)
        history = self._histories.get(key)
        if history is None or history.is_static != is_static:
            history = _EdgeHistory(is_static=is_static, authority=authority)
            self._histories[key] = history
        elif authority:
            history.authority = authority
        history.add(transform, self._cache_time_s)
        self._edges.setdefault(source, {})[target] = history.at(None)

    def _remove_direct_edge(self, source: str, target: str) -> None:
        self._edges.get(source, {}).pop(target, None)
        self._histories.pop((source, target), None)

    def _edge_at_time(
        self,
        source: str,
        target: str,
        ts: float | None,
    ) -> Transform:
        history = self._histories.get((source, target))
        if history is None:
            raise NoTransformError(f"no direct transform from {source!r} to {target!r}")
        return history.at(ts)


def _require_frame_id(frame_id: str | None, role: str) -> str:
    normalized = normalize_frame_id(frame_id)
    if normalized is None:
        raise FrameError(f"{role} must be a non-empty frame id")
    return normalized


def _interpolate_transform(left: Transform, right: Transform, ts: float) -> Transform:
    span = right.ts - left.ts
    if abs(span) < 1e-12:
        return left
    ratio = (ts - left.ts) / span
    return Transform(
        translation=left.translation * (1.0 - ratio) + right.translation * ratio,
        rotation=_slerp(left.rotation, right.rotation, ratio),
        frame_id=left.frame_id,
        child_frame_id=left.child_frame_id,
        ts=ts,
    )


def _slerp(left: Quaternion, right: Quaternion, ratio: float) -> Quaternion:
    q1 = Quaternion(left).normalize()
    q2 = Quaternion(right).normalize()
    dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
    if dot < 0.0:
        q2 = Quaternion(-q2.x, -q2.y, -q2.z, -q2.w)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return Quaternion(
            q1.x + ratio * (q2.x - q1.x),
            q1.y + ratio * (q2.y - q1.y),
            q1.z + ratio * (q2.z - q1.z),
            q1.w + ratio * (q2.w - q1.w),
        ).normalize()

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * ratio
    sin_theta = math.sin(theta)
    scale_0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    scale_1 = sin_theta / sin_theta_0
    return Quaternion(
        q1.x * scale_0 + q2.x * scale_1,
        q1.y * scale_0 + q2.y * scale_1,
        q1.z * scale_0 + q2.z * scale_1,
        q1.w * scale_0 + q2.w * scale_1,
    ).normalize()


__all__ = [
    "ConnectivityException",
    "ExtrapolationError",
    "FrameError",
    "FrameTree",
    "LookupException",
    "NoTransformError",
    "TransformException",
    "UnknownFrameError",
]
