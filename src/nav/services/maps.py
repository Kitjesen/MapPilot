"""MapService -- hive Module version of MapManager.

Manages map listing/saving/deleting/renaming/activation and POI CRUD.

Maps are stored as directories under map_dir:

    ~/data/inovxio/data/maps/
    active -> building_2f/     (symlink pointing to the active map dir)
    building_2f/
        map.pcd                (SLAM point cloud)
        tomogram.pickle        (PCT/A* compatibility map, auto-generated)
        occupancy.npz          (2D static occupancy grid, auto-generated)
    warehouse/
        map.pcd
        tomogram.pickle
        occupancy.npz

Ports:
    In:  map_command (str)   -- JSON command string
    Out: map_response (dict) -- operation result dict
    Out: map_event (dict)    -- lifecycle/artifact events for UI/Gateway
"""

from __future__ import annotations

import json
import logging
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from nav.services.map.api import MapAPIService
from nav.services.map.command_router import dispatch_map_command
from nav.services.map.control import MapControlService
from nav.services.map.facade import MapServiceFacadeMixin
from nav.services.map.pipeline import MapPipelineService
from nav.services.map.records import load_map_record
from nav.services.map.runtime_bridge import MapRuntimeBridge
from nav.services.map.storage import MapStorageService
from runtime import In, Module, Out
from runtime.msgs.map import MapCloudFrame
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register

logger = logging.getLogger(__name__)


@register("map", "manager", description="Saved-map lifecycle manager")
class MapService(Module, MapServiceFacadeMixin, layer=6):
    """Map and POI management module (hive Module).

    Maps are stored as subdirectories.  Each map directory contains:
        map.pcd          -- SLAM point cloud saved via map-save adapter
        tomogram.pickle  -- PCT/A* compatibility index auto-built from map.pcd
        occupancy.npz    -- static 2D occupancy grid derived from map.pcd
                           keys: grid (int8 HxW, 0=free/100=occupied),
                                 resolution (float, metres/cell),
                                 origin (float[2], world XY of grid[0,0])

    Local planning note: LocalPlanner, Terrain, ElevationMapModule,
    and OccupancyGridModule are all online-only.  They subscribe to the live
    /slam/map_cloud LiDAR topic at runtime and rebuild their maps on each frame.
    No persistent artifact is needed or read by those modules at startup.

    The ``active`` entry is a symlink inside map_dir pointing to the
    currently selected map directory.
    """

    runtime_id = "nav.maps"

    map_cloud: In[PointCloud2]
    map_cloud_frame: In[MapCloudFrame]
    map_command: In[str]
    map_response: Out[dict]
    map_event: Out[dict]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        default_data_dir = config.get(
            "data_dir", os.path.expanduser("~/.lingtu")
        )
        default_map_dir = config.get(
            "map_dir",
            os.environ.get(
                "NAV_MAP_DIR",
                # Canonical map dir on sunrise: REPL `map list`, Gateway,
                # and runtime.runtime_profiles._resolve_tomogram all read from
                # here. Stay consistent or saved maps won't be visible to
                # the nav profile.
                os.path.expanduser("~/data/nova/maps"),
            ),
        )
        self.storage = MapStorageService(
            data_dir=default_data_dir,
            map_dir=default_map_dir,
        )
        self._data_dir = self.storage.data_dir
        self._map_dir = self.storage.map_dir
        self._slam_profile = str(
            config.get("slam_profile")
            or config.get("backend_profile")
            or os.environ.get("LINGTU_SLAM_PROFILE")
            or ""
        ).strip().lower()
        self._runtime_data_source = str(
            config.get("data_source")
            or os.environ.get("LINGTU_RUNTIME_DATA_SOURCE")
            or "thunder_field"
        ).strip() or "thunder_field"
        self._source_profile = str(
            config.get("source_profile")
            or config.get("profile")
            or os.environ.get("LINGTU_PROFILE")
            or self._runtime_data_source
        ).strip() or self._runtime_data_source
        self.runtime_bridge = MapRuntimeBridge(
            slam_profile=self._slam_profile,
            map_save_adapter=config.get("map_save_adapter"),
            map_save_timeout_sec=float(config.get("map_save_timeout_sec", 30.0)),
        )
        self.pipeline = MapPipelineService(
            storage=self.storage,
            runtime_bridge=self.runtime_bridge,
            source_profile=self._source_profile,
            runtime_data_source=self._runtime_data_source,
            map_artifact_converter_command=(
                config.get("map_artifact_converter_command")
                or config.get("octomap_converter_command")
                or None
            ),
            octomap_build_mode=str(
                config.get("octomap_build_mode") or "external_pcl_converter"
            ).strip(),
            octomap_resolution=float(config.get("octomap_resolution", 0.20)),
            octomap_free_layers_above=int(config.get("octomap_free_layers_above", 3)),
            octomap_free_dilation_cells=int(config.get("octomap_free_dilation_cells", 1)),
            octomap_build_timeout_sec=float(
                config.get("octomap_build_timeout_sec", 60.0)
            ),
            build_octomap_on_save=bool(config.get("build_octomap_on_save", True)),
        )
        self.control = MapControlService(
            storage=self.storage,
            octomap_editor_command=(
                config.get("octomap_editor_command")
                or os.environ.get("LINGTU_OCTOMAP_EDITOR")
                or None
            ),
            octomap_edit_timeout_sec=float(
                config.get("octomap_edit_timeout_sec", 15.0)
            ),
        )
        self.api = MapAPIService(storage=self.storage)

    def setup(self) -> None:
        self.map_cloud.subscribe(self._on_map_cloud)
        self.map_cloud.set_policy("latest")
        self.map_cloud_frame.subscribe(self._on_map_cloud_frame)
        self.map_command.subscribe(self._on_command)

    def _on_map_cloud(self, cloud: PointCloud2) -> None:
        """Store the latest finite XYZ map cloud for Super-LIO snapshot saves."""
        self.runtime_bridge.on_map_cloud(cloud)

    def _on_map_cloud_frame(self, frame: MapCloudFrame | dict[str, Any]) -> None:
        """Store a typed map-cloud frame for map save and artifact builders."""
        self.runtime_bridge.on_map_cloud_frame(frame)

    # -- command dispatch -------------------------------------------------------

    def _on_command(self, raw: str) -> None:
        """Parse JSON command and dispatch to handler."""
        try:
            cmd = json.loads(raw) if isinstance(raw, str) else raw
        except (json.JSONDecodeError, TypeError):
            cmd = {}

        try:
            resp = dispatch_map_command(self, cmd)
        except Exception as exc:
            resp = {
                "action": cmd.get("action", "") if isinstance(cmd, dict) else "",
                "success": False,
                "message": str(exc),
            }

        self.map_response.publish(resp)
        event = self._map_event_from_response(resp)
        if event:
            self.map_event.publish(event)

    @staticmethod
    def _map_event_from_response(resp: dict[str, Any]) -> dict[str, Any] | None:
        action = str(resp.get("action") or "")
        success = resp.get("success") is True
        event = {
            "create": "map.created",
            "save": "map.saved",
            "delete": "map.deleted",
            "retire": "map.retired",
            "rename": "map.renamed",
            "set_active": "map.active_changed",
            "build_tomogram": "map.artifact_built",
            "build_occupancy_snapshot": "map.artifact_built",
            "build_octomap": "map.artifact_built",
            "build_artifact": "map.artifact_built",
            "edit_voxels": "map.edited",
        }.get(action)
        if not success:
            message = str(resp.get("message") or "").lower()
            metadata = resp.get("metadata") if isinstance(resp.get("metadata"), dict) else {}
            if (
                "artifact gate" not in message
                and "validation" not in message
                and "metadata contract" not in message
                and "validation" not in metadata
            ):
                return None
            event = "map.validation_failed"
        if not event:
            return None
        record = resp.get("record") if isinstance(resp.get("record"), dict) else {}
        metadata = resp.get("metadata") if isinstance(resp.get("metadata"), dict) else {}
        map_id = (
            resp.get("map_id")
            or resp.get("active")
            or record.get("map_id")
            or metadata.get("map_id")
            or ""
        )
        return {
            "schema_version": "map.event",
            "event": event,
            "action": action,
            "map_id": str(map_id or ""),
            "success": success,
            "message": str(resp.get("message") or ""),
            "record_version": str(record.get("version_id") or ""),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }

    # -- map operations ---------------------------------------------------------

    def _map_list(self) -> dict[str, Any]:
        """List map directories (exclude the 'active' symlink entry)."""
        return self.api.list_maps()

    def _get_map_types(self) -> dict[str, Any]:
        return self.api.get_map_types()

    def _map_create(self, name: str) -> dict[str, Any]:
        return self.control.create(name)

    def _build_record_for_dir(
        self,
        map_dir: Path,
        map_id: str,
        *,
        state: str | None = None,
        gate: dict[str, Any] | None = None,
    ):
        resolved_state = state
        if resolved_state is None:
            loaded = load_map_record(map_dir) or {}
            resolved_state = str(loaded.get("state") or "READY")
        return self.storage.build_record_for_dir(
            map_dir,
            map_id,
            state=resolved_state,
            gate=gate,
        )

    def _write_map_record(
        self,
        name: str,
        *,
        state: str = "READY",
        gate: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        return self.storage.write_map_record(name, state=state, gate=gate)

    def _get_record(self, name: str) -> dict[str, Any]:
        return self.api.get_record(name)

    def _get_active_map(self) -> dict[str, Any]:
        return self.api.get_active_map()

    def _get_map_health(self, name: str) -> dict[str, Any]:
        return self.api.get_map_health(name)

    def _get_map_bundle(self, name: str, capability: str) -> dict[str, Any]:
        return self.api.get_map_bundle(name, capability)

    def _build_artifact(self, name: str, artifact_type: str) -> dict[str, Any]:
        return self.pipeline.build_artifact(name, artifact_type)

    def _import_pcd(
        self,
        name: str,
        source_path: str,
        *,
        voxel_size: float = 0.0,
        bounds: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        return self.pipeline.import_pcd(
            name,
            source_path,
            voxel_size=voxel_size,
            bounds=bounds,
        )

    def _crop_map(
        self,
        name: str,
        bounds: dict[str, Any],
        *,
        invert: bool = False,
        voxel_size: float = 0.0,
    ) -> dict[str, Any]:
        return self.pipeline.crop(
            name,
            bounds,
            invert=invert,
            voxel_size=voxel_size,
        )

    def _map_save(
        self,
        name: str,
        slam_profile: str | None = None,
    ) -> dict[str, Any]:
        return self.pipeline.save(
            name,
            slam_profile=slam_profile,
            build_tomogram=self._build_tomogram,
            build_occupancy_snapshot=self._build_occupancy_snapshot,
            build_octomap_artifact=self._build_octomap_artifact,
        )

    def _save_live_map_cloud_snapshot(self, pcd_path: Path) -> dict[str, Any]:
        """Persist the latest map_cloud snapshot as binary XYZ PCD."""
        return self.runtime_bridge.save_live_map_cloud_snapshot(pcd_path)

    @staticmethod
    def _write_binary_xyz_pcd(path: Path, points: np.ndarray) -> int:
        return MapRuntimeBridge.write_binary_xyz_pcd(path, points)

    def _resolve_slam_profile(self, slam_profile: str | None = None) -> str:
        """Resolve the backend used for a map save response."""
        return self.runtime_bridge.resolve_slam_profile(slam_profile)

    def _save_map_with_adapter(self, pcd_path: Path) -> dict[str, Any]:
        """Save ``map.pcd`` through the configured map-save adapter."""
        return self.runtime_bridge.save_map_with_adapter(pcd_path)

    @staticmethod
    def _map_save_capability_fields(slam_profile: str | None) -> dict[str, Any]:
        """Return the map-save capability contract shared by Gateway/MCP users."""
        return MapRuntimeBridge.map_save_capability_fields(slam_profile)

    @staticmethod
    def _normalize_slam_profile(slam_profile: str | None) -> str:
        return MapRuntimeBridge.normalize_slam_profile(slam_profile)

    def _write_saved_map_metadata(
        self,
        name: str,
        *,
        slam_profile: str | None = None,
        mapping_source: str = "map_manager",
        tomogram_shape: list[int] | None = None,
        occupancy_shape: list[int] | None = None,
    ) -> dict[str, Any]:
        """Write metadata.json binding map.pcd and derived planning artifacts."""
        return self.pipeline.write_saved_map_metadata(
            name,
            slam_profile=slam_profile,
            mapping_source=mapping_source,
            tomogram_shape=tomogram_shape,
            occupancy_shape=occupancy_shape,
        )

    @staticmethod
    def _pcd_point_count(path: Path) -> int:
        return MapPipelineService.pcd_point_count(path)

    @staticmethod
    def _tomogram_shape_from_data(data: Any) -> list[int]:
        return MapPipelineService.tomogram_shape_from_data(data)

    @classmethod
    def _load_tomogram_shape(cls, path: Path) -> list[int]:
        return MapPipelineService.load_tomogram_shape(path)

    @staticmethod
    def _load_occupancy_shape(path: Path) -> list[int]:
        return MapPipelineService.load_occupancy_shape(path)

    def _build_tomogram(self, name: str) -> dict[str, Any]:
        """Build tomogram.pickle from map.pcd for PCT/A* compatibility."""
        return self.pipeline.build_tomogram(name)

    def _build_octomap_artifact(
        self,
        name: str,
        *,
        slam_profile: str | None = None,
    ) -> dict[str, Any]:
        """Build or reuse octomap.ot for OctoPlanner3D runtime planning."""
        return self.pipeline.build_octomap_artifact(
            name,
            slam_profile=slam_profile,
        )

    def _build_occupancy_snapshot(self, name: str) -> dict[str, Any]:
        """Build ROS2-compatible 2D occupancy grid from SLAM output."""
        return self.pipeline.build_occupancy_snapshot(name)

    # ----- Occupancy algorithms -------------------------------------------------

    def _build_occupancy_raycasting(
        self, poses_path: Path, patches_dir: Path,
    ) -> tuple[np.ndarray, float, np.ndarray]:
        """Log-odds Bayesian occupancy via raycasting from keyframe poses."""
        return MapPipelineService.build_occupancy_raycasting(poses_path, patches_dir)

    def _build_occupancy_projection(
        self, pcd_path: Path,
    ) -> tuple[np.ndarray, float, np.ndarray]:
        """Fallback: height-filter + XY projection (binary, no unknown state)."""
        return MapPipelineService.build_occupancy_projection(pcd_path)

    # ----- Output helpers --------------------------------------------------------

    def _save_occupancy_pgm_yaml(
        self, grid: np.ndarray, resolution: float, origin: np.ndarray,
        pgm_path: Path, yaml_path: Path,
    ) -> None:
        """ROS2 map_server compatible PGM + YAML."""
        MapPipelineService.save_occupancy_pgm_yaml(
            grid,
            resolution,
            origin,
            pgm_path,
            yaml_path,
        )

    @staticmethod
    def _raycast_free(
        log_odds: np.ndarray, ox: int, oy: int,
        end_col: np.ndarray, end_row: np.ndarray,
        grid_w: int, grid_h: int, log_free: float,
    ) -> None:
        """Vectorised free-space update along rays via DDA rasterisation."""
        MapPipelineService.raycast_free(
            log_odds,
            ox,
            oy,
            end_col,
            end_row,
            grid_w,
            grid_h,
            log_free,
        )

    @staticmethod
    def _parse_poses_txt(path: Path) -> list[dict]:
        """Parse PGO poses.txt: each line 'patch.pcd tx ty tz qw qx qy qz'."""
        return MapPipelineService.parse_poses_txt(path)

    @staticmethod
    def _quat_to_rot(q: np.ndarray) -> np.ndarray:
        """Quaternion (w, x, y, z) -> 3x3 rotation matrix."""
        return MapPipelineService.quat_to_rot(q)

    @staticmethod
    def _load_pcd_points(pcd_path: str) -> np.ndarray | None:
        """Load XYZ points from a PCD file."""
        return MapPipelineService.load_pcd_points(pcd_path)

    def _map_delete(self, name: str) -> dict[str, Any]:
        """Delete an entire map directory."""
        return self.control.delete(name)

    def _map_retire(self, name: str) -> dict[str, Any]:
        """Mark a map retired without deleting its artifacts."""
        return self.control.retire(name)

    def _map_rename(self, name: str, new_name: str) -> dict[str, Any]:
        """Rename a map directory; update active symlink if needed."""
        return self.control.rename(name, new_name)

    def _map_set_active(self, name: str) -> dict[str, Any]:
        """Create/update the ``active`` symlink to point at the named map dir."""
        return self.control.set_active(name)

    def _map_edit_voxels(self, name: str, cmd: dict[str, Any]) -> dict[str, Any]:
        """Edit saved OctoMap voxels used by OctoPlanner3D."""
        return self.control.edit_voxels(name, cmd)

    def _save_active_map(self) -> None:
        self.storage.save_active_map()

