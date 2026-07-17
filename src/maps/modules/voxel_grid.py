"""VoxelGridModule - native C++ 3D voxel layer from LiDAR point clouds.

The Module is intentionally thin: Python owns runtime ports and payload
packing; ``maps.voxel`` C++ owns range/z filtering, voxel dedupe, column
carving, decay, snapshot, and occupancy queries.

Ports:
  In:  map_cloud_frame (MapCloudFrame), map_cloud (PointCloud2), odometry (Odometry)
  Out: voxel_map (dict), scene (MapSceneFrame), voxel_cloud (PointCloud2)
"""

from __future__ import annotations

import json
import logging
import math
import time
from pathlib import Path
from typing import Any

from maps.adapters.python.voxel import NativeVoxelLayer, VoxelNativeUnavailable
from runtime.module import Module, skill
from runtime.msgs.map import MapObservationFrame, MapSceneFrame
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.runtime_interface import TOPICS, normalize_frame_id, topic_default_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


@register("map", "voxel", description="Native C++ 3D voxel layer from LiDAR")
class VoxelGridModule(Module, layer=2):
    """Runtime wrapper for the native C++ voxel/column-carving layer."""

    map_observation: In[MapObservationFrame]
    map_cloud: In[PointCloud2]
    odometry: In[Odometry]

    voxel_map: Out[dict]
    scene: Out[MapSceneFrame]
    voxel_cloud: Out[PointCloud2]

    def __init__(
        self,
        voxel_size: float = 0.05,
        max_range: float = 20.0,
        min_z: float = -0.5,
        max_z: float = 3.0,
        decay_rate: float = 0.01,
        publish_interval: float = 2.0,
        column_carving: bool = True,
        frame_id: str | None = None,
        backend: str = "cpp",
        state_path: str = "",
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._configured_backend = str(backend or "cpp").lower()
        self._res = float(voxel_size)
        self._max_range = float(max_range)
        self._min_z = float(min_z)
        self._max_z = float(max_z)
        self._decay = float(decay_rate)
        self._interval = float(publish_interval)
        self._column_carving = bool(column_carving)
        self._state_path = Path(state_path).expanduser() if state_path else None
        self._default_frame_id = normalize_frame_id(frame_id) or topic_default_frame_id(TOPICS.map_cloud)
        self._last_cloud_frame_id = self._default_frame_id
        self._robot_xyz = [0.0, 0.0, 0.0]
        self._last_publish: float = 0.0
        self._scene_sequence: int = 0
        self._last_update: dict[str, Any] = {
            "input_points": 0,
            "accepted_points": 0,
            "input_voxels": 0,
            "input_columns": 0,
            "carved_columns": 0,
            "carved_voxels": 0,
            "ts": 0.0,
        }
        self._native: NativeVoxelLayer | None = None
        self._backend = "cpp"
        self._backend_error = ""

    def setup(self) -> None:
        self._setup_backend()
        self.map_cloud.subscribe(self._on_cloud)
        self.map_observation.subscribe(self._on_observation)
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.set_policy("latest")
        self.map_observation.set_policy("latest")

    def start(self) -> None:
        super().start()
        self._last_publish = time.time()

    def stop(self) -> None:
        if self._native is not None:
            self._save_configured_state()
            self._native.close()
            self._native = None
        super().stop()

    def _setup_backend(self) -> None:
        if self._configured_backend not in {"cpp", "native"}:
            raise ValueError("VoxelGridModule is native-only; backend must be 'cpp' or 'native'")
        try:
            self._native = NativeVoxelLayer(
                voxel_size=self._res,
                max_range=self._max_range,
                min_z=self._min_z,
                max_z=self._max_z,
                decay_rate=self._decay,
                column_carving=self._column_carving,
            )
            self._load_configured_state()
        except VoxelNativeUnavailable as exc:
            self._backend_error = str(exc)
            raise RuntimeError(
                "VoxelGridModule requires the native lingtu_maps C++ library. "
                "Build src/maps and set LINGTU_MAPS_LIB, or install lingtu_maps "
                f"on the runtime library path. Native load error: {exc}"
            ) from exc

    def _require_native(self) -> NativeVoxelLayer:
        if self._native is None:
            raise RuntimeError("VoxelGridModule native backend is not initialized")
        return self._native

    def _load_configured_state(self) -> None:
        if self._state_path is None or self._native is None or not self._state_path.exists():
            return
        if not self._native.validate_state(self._state_path):
            raise RuntimeError(f"VoxelGridModule state file is invalid: {self._state_path}")
        self._native.load_state(self._state_path)
        logger.info("VoxelGrid loaded native state from %s", self._state_path)

    def _save_configured_state(self) -> None:
        if self._state_path is None or self._native is None:
            return
        self._state_path.parent.mkdir(parents=True, exist_ok=True)
        self._native.save_state(self._state_path)
        logger.info("VoxelGrid saved native state to %s", self._state_path)

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_xyz[0] = odom.x
        self._robot_xyz[1] = odom.y
        self._robot_xyz[2] = odom.z

    def _on_cloud(self, cloud: PointCloud2) -> None:
        if cloud.is_empty:
            return
        native = self._require_native()
        frame_id = normalize_frame_id(getattr(cloud, "frame_id", None)) or self._default_frame_id
        stamp_ns = int(float(getattr(cloud, "ts", 0.0) or time.time()) * 1_000_000_000)
        native.update(
            cloud.points,
            frame_id=frame_id,
            stamp_ns=stamp_ns,
            origin_xyz=tuple(float(v) for v in self._robot_xyz),
        )
        stats = native.stats()
        self._last_cloud_frame_id = frame_id
        self._last_update = {
            "input_points": int(stats["input_points"]),
            "accepted_points": int(stats["accepted_points"]),
            "input_voxels": int(stats["input_voxels"]),
            "input_columns": int(stats["input_columns"]),
            "carved_columns": int(stats["carved_columns"]),
            "carved_voxels": int(stats["carved_voxels"]),
            "ts": time.time(),
        }

        now = time.time()
        if now - self._last_publish >= self._interval:
            self._last_publish = now
            self._decay_and_publish()

    def _on_observation(self, frame: MapObservationFrame) -> None:
        self._on_cloud(frame.to_map_pointcloud2())

    def _decay_and_publish(self) -> None:
        """Apply native decay, then publish a snapshot for Gateway/map consumers."""

        native = self._require_native()
        native.decay()
        centers = native.snapshot_xyz()
        if centers.shape[0] == 0:
            return
        stats_native = native.stats()
        scene_metadata = native.scene_metadata()
        total = int(stats_native.get("accumulated_occupied") or stats_native["total_voxels"])
        stats: dict[str, Any] = {
            "total_voxels": total,
            "occupied": total,
            "memory_kb": round((total * 64) / 1024, 1),
            "voxel_size": self._res,
            "ts": time.time(),
            "last_input_columns": int(stats_native.get("input_columns", 0)),
            "frame_id": self._last_cloud_frame_id,
            "column_carving": self._column_carving,
            "backend": self._backend,
            "last_update": dict(self._last_update),
            "accumulated_cells": int(stats_native.get("accumulated_cells", 0)),
            "accumulated_occupied": int(stats_native.get("accumulated_occupied", total)),
            "accumulated_generation": int(stats_native.get("accumulated_generation", 0)),
            "ray_updates": int(stats_native.get("ray_updates", 0)),
            "free_updates": int(stats_native.get("free_updates", 0)),
            "hit_updates": int(stats_native.get("hit_updates", 0)),
            "pruned_cells": int(stats_native.get("pruned_cells", 0)),
        }
        cloud = PointCloud2(points=centers, frame_id=self._last_cloud_frame_id)
        self._scene_sequence += 1
        scene = MapSceneFrame(
            frame_id=self._last_cloud_frame_id,
            source="maps.voxel",
            sequence=self._scene_sequence,
            metadata={
                "primary_layer": "maps.voxel_cloud",
                "backend": self._backend,
                "column_carving": self._column_carving,
                "native_scene": scene_metadata,
            },
            layers=[
                {
                    "id": "maps.voxel_cloud",
                    "type": "pointcloud",
                    "topic": TOPICS.maps_voxel_cloud,
                    "source": "maps.voxel",
                    "semantic": "accumulated_voxel_map",
                    "point_count": int(centers.shape[0]),
                    "metadata": dict(stats),
                    "payload": cloud,
                }
            ],
        )
        self.voxel_map.publish(stats)
        self.scene.publish(scene)
        self.voxel_cloud.publish(cloud)

    @skill
    def get_voxel_stats(self) -> str:
        """Return stats about the current native voxel map."""

        native = self._require_native()
        total = native.voxel_count()
        mem_kb = (total * 64) / 1024
        native_stats = native.stats()
        return json.dumps(
            {
                "total_voxels": total,
                "occupied": total,
                "memory_kb": round(mem_kb, 1),
                "voxel_size": self._res,
                "column_carving": self._column_carving,
                "backend": self._backend,
                "backend_error": self._backend_error,
                "state_path": str(self._state_path) if self._state_path is not None else "",
                "last_update": dict(self._last_update),
                "accumulated_cells": int(native_stats.get("accumulated_cells", 0)),
                "accumulated_occupied": int(native_stats.get("accumulated_occupied", total)),
                "accumulated_generation": int(native_stats.get("accumulated_generation", 0)),
                "ray_updates": int(native_stats.get("ray_updates", 0)),
                "free_updates": int(native_stats.get("free_updates", 0)),
                "hit_updates": int(native_stats.get("hit_updates", 0)),
                "pruned_cells": int(native_stats.get("pruned_cells", 0)),
            }
        )

    @skill
    def checkpoint_voxels(self) -> str:
        """Save the configured native voxel state file."""

        if self._state_path is None:
            return json.dumps({"saved": False, "reason": "state_path_not_configured"})
        self._save_configured_state()
        return json.dumps({"saved": True, "path": str(self._state_path)})

    @skill
    def clear_voxels(self) -> str:
        """Reset the native voxel map."""

        native = self._require_native()
        count = native.voxel_count()
        native.reset()
        logger.info("VoxelGrid cleared %d voxels", count)
        return json.dumps({"cleared": count})

    @skill
    def query_voxel(self, x: float, y: float, z: float) -> str:
        """Check whether the voxel containing (x, y, z) is occupied."""

        count = self._require_native().query_count(float(x), float(y), float(z))
        key = (
            int(math.floor(x / self._res)),
            int(math.floor(y / self._res)),
            int(math.floor(z / self._res)),
        )
        return json.dumps(
            {
                "occupied": count >= 1.0,
                "count": count,
                "voxel_key": list(key),
            }
        )

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        n = self._native.voxel_count() if self._native is not None else 0
        info["voxel_grid"] = {
            "voxel_size": self._res,
            "max_range": self._max_range,
            "z_range": [self._min_z, self._max_z],
            "decay_rate": self._decay,
            "publish_interval": self._interval,
            "column_carving": self._column_carving,
            "voxel_count": n,
            "frame_id": self._last_cloud_frame_id,
            "backend": self._backend,
            "configured_backend": self._configured_backend,
            "backend_error": self._backend_error,
            "state_path": str(self._state_path) if self._state_path is not None else "",
            "last_update": dict(self._last_update),
        }
        return info
