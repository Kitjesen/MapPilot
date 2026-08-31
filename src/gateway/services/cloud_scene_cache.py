"""Accumulated point-cloud cache for the browser map viewer."""

from __future__ import annotations

import logging
import os
import threading

from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


class CloudSceneCache:
    """Owns the browser's transient point-cloud scene cache."""

    def __init__(self) -> None:
        self._map_points: np.ndarray | None = None
        self._map_point_keys: set[int] = set()
        self._map_point_stale_counts: dict[int, int] = {}
        self._map_cloud_lock = threading.Lock()
        self._map_cloud_count = 0
        self._map_voxel_size = max(0.02, _env_float("LINGTU_MAP_VIEWER_VOXEL_SIZE", 0.15))
        self._inv_map_voxel_size = 1.0 / self._map_voxel_size
        self._map_viewer_max_points = max(
            1,
            int(os.environ.get("LINGTU_MAP_VIEWER_MAX_CACHE_POINTS", "120000")),
        )
        self._voxel_hits: dict[int, int] = {}
        self._voxel_min_hits = int(os.environ.get("LINGTU_MAP_MIN_HITS", "3"))
        self._voxel_key_offset = 1 << 19
        self._map_viewer_stale_grace = max(
            1,
            int(os.environ.get("LINGTU_MAP_VIEWER_STALE_GRACE", "3")),
        )
        self._map_viewer_max_stale_drops = max(
            1,
            int(os.environ.get("LINGTU_MAP_VIEWER_MAX_STALE_DROPS", "4096")),
        )
        self._last_map_scene_ts = 0.0

    def set_voxel_size(self, value: float) -> None:
        self._map_voxel_size = max(0.02, float(value))
        self._inv_map_voxel_size = 1.0 / self._map_voxel_size

    def record_frame(self) -> int:
        self._map_cloud_count += 1
        return self._map_cloud_count

    def frames_seen(self) -> int:
        return self._map_cloud_count

    def point_count(self) -> int:
        with self._map_cloud_lock:
            return len(self._map_points) if self._map_points is not None else 0

    def snapshot_points(self) -> np.ndarray | None:
        with self._map_cloud_lock:
            return None if self._map_points is None else np.asarray(self._map_points, dtype=np.float32).copy()

    def clear(self) -> None:
        with self._map_cloud_lock:
            self._map_points = None
            self._map_point_keys.clear()
            self._map_point_stale_counts.clear()
            self._map_cloud_count = 0
            self._voxel_hits.clear()

    @staticmethod
    def voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
        if len(pts) == 0:
            return pts
        keys = (pts[:, :3] / voxel).astype(np.int32)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[np.sort(idx)]

    def pack_voxel_keys(self, pts_xyz: np.ndarray) -> np.ndarray:
        k = (pts_xyz[:, :3] * self._inv_map_voxel_size).astype(np.int64)
        k = (k + self._voxel_key_offset) & 0xFFFFF
        return (k[:, 0] << 40) | (k[:, 1] << 20) | k[:, 2]

    def pack_xy_voxel_keys(self, pts_xyz: np.ndarray) -> np.ndarray:
        k = (pts_xyz[:, :2] * self._inv_map_voxel_size).astype(np.int64)
        k = (k + self._voxel_key_offset) & 0xFFFFF
        return (k[:, 0] << 20) | k[:, 1]

    def replace_points(self, pts_xyz: np.ndarray) -> None:
        pts_xyz = self._limit_points(pts_xyz)
        self._map_points = pts_xyz
        self._map_point_stale_counts.clear()
        if pts_xyz is None or len(pts_xyz) == 0:
            self._map_point_keys = set()
            return
        self._map_point_keys = {int(k) for k in self.pack_voxel_keys(pts_xyz)}

    def update_clean_points(self, pts_xyz: np.ndarray) -> None:
        if self._map_points is None or len(self._map_points) == 0 or len(pts_xyz) == 0:
            return

        new_keys_arr = self.pack_voxel_keys(pts_xyz)
        new_xy_keys = {int(k) for k in self.pack_xy_voxel_keys(pts_xyz)}
        if not new_xy_keys:
            return
        new_by_key: dict[int, np.ndarray] = {}
        for idx, key in enumerate(new_keys_arr):
            new_by_key.setdefault(int(key), pts_xyz[idx])

        old_keys = self.pack_voxel_keys(self._map_points)
        old_xy = self.pack_xy_voxel_keys(self._map_points)
        remove_indices: list[int] = []

        for idx, key in enumerate(old_keys):
            key_int = int(key)
            fresh_point = new_by_key.get(key_int)
            if fresh_point is not None:
                self._map_points[idx] = fresh_point
                self._map_point_stale_counts.pop(key_int, None)
                continue
            if int(old_xy[idx]) not in new_xy_keys:
                continue
            miss_count = self._map_point_stale_counts.get(key_int, 0) + 1
            self._map_point_stale_counts[key_int] = miss_count
            if miss_count >= self._map_viewer_stale_grace:
                remove_indices.append(idx)

        if not remove_indices:
            return
        if len(remove_indices) > self._map_viewer_max_stale_drops:
            remove_indices = remove_indices[: self._map_viewer_max_stale_drops]

        keep = np.ones(len(self._map_points), dtype=bool)
        remove_array = np.asarray(remove_indices, dtype=np.int64)
        keep[remove_array] = False
        removed_keys = [int(old_keys[idx]) for idx in remove_indices]
        self._map_points = self._map_points[keep]
        for key in removed_keys:
            self._map_point_stale_counts.pop(key, None)
            self._map_point_keys.discard(key)
        self._map_point_keys = {int(k) for k in self.pack_voxel_keys(self._map_points)}

    def merge_points(
        self,
        pts_xyz: np.ndarray,
        *,
        replace_xy_columns: bool = False,
    ) -> None:
        if pts_xyz is None or len(pts_xyz) == 0:
            return
        if self._map_points is None or len(self._map_points) == 0:
            self.replace_points(pts_xyz)
            return

        if replace_xy_columns:
            self.update_clean_points(pts_xyz)

        if not self._map_point_keys:
            self._map_point_keys = {int(k) for k in self.pack_voxel_keys(self._map_points)}

        keys = self.pack_voxel_keys(pts_xyz)
        unique_keys, unique_idx = np.unique(keys, return_index=True)
        new_indices: list[int] = []
        new_keys: list[int] = []
        for key, idx in zip(unique_keys, unique_idx):
            key_int = int(key)
            if key_int in self._map_point_keys:
                continue
            new_keys.append(key_int)
            new_indices.append(int(idx))

        if not new_indices:
            return

        new_pts = pts_xyz[np.asarray(sorted(new_indices), dtype=np.int64)]
        self._map_points = np.concatenate([self._map_points, new_pts], axis=0)
        self._map_point_keys.update(new_keys)
        for key in new_keys:
            self._map_point_stale_counts.pop(key, None)
        if len(self._map_points) > self._map_viewer_max_points:
            self._map_points = self._limit_points(self._map_points)
            self._map_point_keys = {int(k) for k in self.pack_voxel_keys(self._map_points)}
            self._map_point_stale_counts.clear()

    def _limit_points(self, pts_xyz: np.ndarray) -> np.ndarray:
        if pts_xyz is None or len(pts_xyz) <= self._map_viewer_max_points:
            return pts_xyz

        scale = max(
            1.0,
            (len(pts_xyz) / self._map_viewer_max_points) ** (1.0 / 3.0),
        )
        voxel = self._map_voxel_size * scale
        limited = self.voxel_downsample(pts_xyz, voxel)
        for _ in range(6):
            if len(limited) <= self._map_viewer_max_points:
                return limited
            voxel *= 1.25
            limited = self.voxel_downsample(pts_xyz, voxel)

        indices = np.linspace(
            0,
            len(limited) - 1,
            self._map_viewer_max_points,
            dtype=np.int64,
        )
        return limited[indices]

    def record_hits(self, pts_xyz: np.ndarray) -> None:
        frame_keys = np.unique(self.pack_voxel_keys(pts_xyz))
        for k in frame_keys:
            self._voxel_hits[int(k)] = self._voxel_hits.get(int(k), 0) + 1
        if len(self._voxel_hits) <= 200_000:
            return
        before = len(self._voxel_hits)
        self._voxel_hits = {k: c for k, c in self._voxel_hits.items() if c >= 2}
        logger.info(
            "voxel_hits GC: %d -> %d entries (dropped hit==1)",
            before,
            len(self._voxel_hits),
        )
