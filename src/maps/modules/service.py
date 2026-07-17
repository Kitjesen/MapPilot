"""Native maps service Module.

This file is the Python Module/runtime adapter for the native maps domain. It
owns ports, command dispatch, and compatibility helpers; map ids, active-map
state, and live layer math are owned by ``lingtu_maps`` under ``src/maps``.

Maps are stored as directories under map_dir:

    ~/data/inovxio/data/maps/
    active_map.txt             (native active-map state)
    building_2f/
        map.pcd                (SLAM point cloud)
        occupancy.npz          (2D static occupancy grid, auto-generated)
    warehouse/
        map.pcd
        occupancy.npz

Ports:
    In:  map_command (str)   -- JSON command string
    Out: map_response (dict) -- operation result dict
    Out: map_event (dict)    -- lifecycle/artifact events for UI/Gateway
"""

from __future__ import annotations

import logging
import os
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from maps.paths import nav_map_root_str
from maps.services.api import MapAPIService
from maps.services.command_router import dispatch_map_command
from maps.services.control import MapControlService
from maps.services.facade import MapsFacadeMixin
from maps.services.pipeline import MapPipelineService
from maps.services.runtime_bridge import MapRuntimeBridge
from maps.services.storage import MapStorageService
from runtime import In, Module, Out
from runtime.msgs.map import (
    MapCloudFrame,
    MapControlRequest,
    SemanticSaveRequest,
    SemanticSaveResult,
)
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register

logger = logging.getLogger(__name__)


@register("map", "manager", description="Native saved-map lifecycle module")
@register("map", "service", description="Native saved-map lifecycle module")
class MapsModule(Module, MapsFacadeMixin, layer=6):
    """Map command/query Module backed by native maps storage.

    Maps are stored as subdirectories.  Each map directory contains:
        map.pcd          -- SLAM point cloud saved via map-save adapter
        occupancy.npz    -- static 2D occupancy grid derived from map.pcd
                           keys: grid (int8 HxW, 0=free/100=occupied),
                                 resolution (float, metres/cell),
                                 origin (float[2], world XY of grid[0,0])

    Local planning note: LocalPlanner, Terrain, ElevationMapModule,
    and OccupancyGridModule are all online-only.  They subscribe to the live
    /slam/map_cloud LiDAR topic at runtime and rebuild their maps on each frame.
    No persistent artifact is needed or read by those modules at startup.

    ``active_map.txt`` is the native active-map source of truth. The legacy
    ``active`` symlink may still be written only for compatibility consumers.
    """

    runtime_id = "maps.service"

    map_cloud: In[PointCloud2]
    map_cloud_frame: In[MapCloudFrame]
    map_command: In[MapControlRequest]
    semantic_save_result: In[SemanticSaveResult]
    localization_status: In[dict]
    planning_health: In[dict]
    collision_event: In[dict]
    map_response: Out[dict]
    map_event: Out[dict]
    semantic_save_request: Out[SemanticSaveRequest]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        default_data_dir = config.get("data_dir", os.path.expanduser("~/.lingtu"))
        default_map_dir = config.get(
            "map_dir",
            nav_map_root_str(),
        )
        self.storage = MapStorageService(
            data_dir=default_data_dir,
            map_dir=default_map_dir,
        )
        self._data_dir = self.storage.data_dir
        self._map_dir = self.storage.map_dir
        self._slam_profile = (
            str(
                config.get("slam_profile")
                or config.get("backend_profile")
                or os.environ.get("LINGTU_SLAM_PROFILE")
                or ""
            )
            .strip()
            .lower()
        )
        self._runtime_data_source = (
            str(config.get("data_source") or os.environ.get("LINGTU_RUNTIME_DATA_SOURCE") or "thunder_field").strip()
            or "thunder_field"
        )
        self._source_profile = (
            str(
                config.get("source_profile")
                or config.get("profile")
                or os.environ.get("LINGTU_PROFILE")
                or self._runtime_data_source
            ).strip()
            or self._runtime_data_source
        )
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
                or os.environ.get("LINGTU_MAP_ARTIFACT_CONVERTER")
                or os.environ.get("LINGTU_OCTOPLANNER3D_PCD_CONVERTER")
                or os.environ.get("LINGTU_OCTOMAP_CONVERTER")
                or None
            ),
            octomap_build_mode=str(config.get("octomap_build_mode") or "external_pcl_converter").strip(),
            octomap_resolution=float(config.get("octomap_resolution", 0.20)),
            octomap_free_layers_above=int(config.get("octomap_free_layers_above", 3)),
            octomap_free_dilation_cells=int(config.get("octomap_free_dilation_cells", 1)),
            octomap_build_timeout_sec=float(config.get("octomap_build_timeout_sec", 60.0)),
            build_octomap_on_save=bool(config.get("build_octomap_on_save", True)),
            map_prune_command=(
                config.get("map_prune_command")
                or config.get("dynamic_filter_command")
                or os.environ.get("LINGTU_SAVE_DYNAMIC_FILTER_COMMAND")
                or None
            ),
            map_opt=(
                config.get("map_opt") or config.get("map_optimization") or os.environ.get("LINGTU_MAP_OPT") or "off"
            ),
            map_opt_command=(config.get("map_opt_command") or config.get("map_optimization_command") or None),
            map_opt_timeout_sec=float(config.get("map_opt_timeout_sec", 120.0)),
            map_opt_required=bool(config.get("map_opt_required", False)),
        )
        self.control = MapControlService(
            storage=self.storage,
            octomap_editor_command=(
                config.get("octomap_editor_command") or os.environ.get("LINGTU_OCTOMAP_EDITOR") or None
            ),
            octomap_edit_timeout_sec=float(config.get("octomap_edit_timeout_sec", 15.0)),
        )
        self.api = MapAPIService(storage=self.storage)
        self._semantic_save_timeout_sec = max(0.1, float(config.get("semantic_save_timeout_sec", 3.0)))
        self._semantic_save_lock = threading.Lock()
        self._semantic_save_waiters: dict[str, tuple[threading.Event, list[SemanticSaveResult], str, Path]] = {}

    def setup(self) -> None:
        self.map_cloud.subscribe(self._on_map_cloud)
        self.map_cloud.set_policy("latest")
        self.map_cloud_frame.subscribe(self._on_map_cloud_frame)
        self.map_command.subscribe(self._on_command)
        self.semantic_save_result.subscribe(self._on_semantic_save_result)
        self.localization_status.subscribe(self._on_localization_status)
        self.planning_health.subscribe(self._on_planning_health)
        self.collision_event.subscribe(self._on_collision_event)

    def stop(self) -> None:
        self.storage.close()
        super().stop()

    def _on_map_cloud(self, cloud: PointCloud2) -> None:
        """Store the latest finite XYZ map cloud for Super-LIO snapshot saves."""
        self.runtime_bridge.on_map_cloud(cloud)

    def _on_map_cloud_frame(self, frame: MapCloudFrame | dict[str, Any]) -> None:
        """Store a typed map-cloud frame for map save and artifact builders."""
        self.runtime_bridge.on_map_cloud_frame(frame)

    def _on_semantic_save_result(self, result: SemanticSaveResult) -> None:
        with self._semantic_save_lock:
            waiter = self._semantic_save_waiters.get(result.request_id)
            if waiter is None:
                return
            event, results, expected_map_id, expected_path = waiter
            if result.map_id != expected_map_id:
                return
            try:
                if Path(result.path).resolve() != expected_path.resolve():
                    return
            except OSError:
                return
            results.append(result)
            event.set()

    def _on_localization_status(self, status: dict[str, Any]) -> None:
        """Store snapshot health and feed one real localization observation."""
        payload = dict(status or {})
        self.runtime_bridge.on_localization_status(payload)
        state = (
            str(payload.get("state") or payload.get("status") or payload.get("localization_state") or "")
            .strip()
            .upper()
        )
        localized = (
            payload.get("healthy") is not False
            and payload.get("ok") is not False
            and state not in {"ERROR", "FAILED", "LOST", "DIVERGED", "UNHEALTHY"}
        )
        quality = payload.get("quality", payload.get("confidence", 1.0 if localized else 0.0))
        position_error = payload.get(
            "position_error_m",
            payload.get("drift_m", payload.get("residual_m", 0.0)),
        )
        covariance = payload.get("covariance_trace", 0.0)
        try:
            self.storage.native_service.ingest_localization_health(
                self.storage.active_map,
                timestamp_s=float(payload.get("ts") or time.time()),
                localized=localized,
                position_error_m=float(position_error or 0.0),
                covariance_trace=float(covariance or 0.0),
                quality=float(quality or 0.0),
                source=str(payload.get("source") or "runtime.localization_status"),
            )
        except Exception:
            logger.debug("native map localization health ingest skipped", exc_info=True)

    def _on_planning_health(self, event: dict[str, Any]) -> None:
        """Feed one completed global-planning attempt into native MapHealth."""
        payload = dict(event or {})
        try:
            self.storage.native_service.ingest_planning_outcome(
                self.storage.active_map,
                timestamp_s=float(payload.get("ts") or time.time()),
                success=bool(payload.get("success")),
                planner=str(payload.get("planner") or "unknown"),
                reason=str(payload.get("reason") or ""),
            )
        except Exception:
            logger.debug("native map planning health ingest skipped", exc_info=True)

    def _on_collision_event(self, event: dict[str, Any]) -> None:
        """Feed a projected or observed collision event into native MapHealth."""
        payload = dict(event or {})
        try:
            self.storage.native_service.ingest_collision_event(
                self.storage.active_map,
                timestamp_s=float(payload.get("ts") or time.time()),
                severity=float(payload.get("severity") or 1.0),
                source=str(payload.get("source") or "runtime.safety"),
                reason=str(payload.get("reason") or ""),
            )
        except Exception:
            logger.debug("native map collision health ingest skipped", exc_info=True)

    # -- public maps contract --------------------------------------------------

    def list_maps(self) -> dict[str, Any]:
        """List maps from the native maps service."""
        return self.api.list_maps()

    def get_map_types(self) -> dict[str, Any]:
        """Return native map classes, artifacts, and capabilities."""
        return self.api.get_map_types()

    def get_record(self, name: str) -> dict[str, Any]:
        """Return one native MapRecord."""
        return self.api.get_record(name)

    def get_active_map(self) -> dict[str, Any]:
        """Return active-map state from the native store."""
        return self.api.get_active_map()

    def get_map_health(self, name: str = "") -> dict[str, Any]:
        """Return native health data for a map or the active map."""
        return self.api.get_map_health(name)

    def get_map_bundle(
        self,
        name: str,
        capability: str,
    ) -> dict[str, Any]:
        """Resolve an artifact by capability through the native service."""
        return self.api.get_map_bundle(name, capability)

    def get_map_points(
        self,
        name: str,
        *,
        max_points: int = 0,
    ) -> dict[str, Any]:
        """Read saved-map points through the native query API."""
        return self.api.get_map_points(name, max_points=max_points)

    def validate_map_artifacts(
        self,
        name: str,
        *,
        require_octomap: bool = False,
        require_occupancy: bool = False,
        expected_data_source: str | None = None,
        expected_source_profile: str | None = None,
        expected_frame_id: str | None = None,
    ) -> dict[str, Any]:
        """Validate one map package inside the maps domain."""
        return self.api.validate_artifacts(
            name,
            require_octomap=require_octomap,
            require_occupancy=require_occupancy,
            expected_data_source=expected_data_source,
            expected_source_profile=expected_source_profile,
            expected_frame_id=expected_frame_id,
        )

    def set_active_map(self, name: str) -> dict[str, Any]:
        """Activate a validated map through map control."""
        return self.control.set_active(name)

    # -- command dispatch -------------------------------------------------------

    def execute(self, request: MapControlRequest | dict[str, Any]) -> dict[str, Any]:
        """Execute one typed control request against the native maps service."""
        try:
            typed = request if isinstance(request, MapControlRequest) else MapControlRequest.from_mapping(request)
            cmd = typed.to_mapping()
            resp = dispatch_map_command(self, cmd)
        except Exception as exc:
            resp = {
                "action": getattr(request, "action", ""),
                "success": False,
                "reason_code": "invalid_control_request",
                "message": str(exc),
            }
        event = self._map_event_from_response(resp)
        if event:
            self.map_event.publish(event)
        return resp

    def _on_command(self, request: MapControlRequest) -> None:
        """Handle a typed runtime control-plane request."""
        self.map_response.publish(self.execute(request))

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
            "restore_source": "map.source_restored",
            "set_active": "map.active_changed",
            "build_occupancy_snapshot": "map.artifact_built",
            "build_octomap": "map.artifact_built",
            "build_esdf_artifact": "map.artifact_built",
            "build_traversability_artifact": "map.artifact_built",
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
        map_id = resp.get("map_id") or resp.get("active") or record.get("map_id") or metadata.get("map_id") or ""
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
        return self.list_maps()

    def _get_map_types(self) -> dict[str, Any]:
        return self.get_map_types()

    def _map_create(self, name: str) -> dict[str, Any]:
        return self.control.create(name)

    def _map_restore_source_backup(self, name: str) -> dict[str, Any]:
        return self.control.restore_source_backup(name)

    def _get_record(self, name: str) -> dict[str, Any]:
        return self.get_record(name)

    def _get_active_map(self) -> dict[str, Any]:
        return self.get_active_map()

    def _get_map_health(self, name: str) -> dict[str, Any]:
        return self.get_map_health(name)

    def _get_map_bundle(self, name: str, capability: str) -> dict[str, Any]:
        return self.get_map_bundle(name, capability)

    def _get_map_points(self, name: str, *, max_points: int = 0) -> dict[str, Any]:
        return self.get_map_points(name, max_points=max_points)

    def _validate_map_artifacts(
        self,
        name: str,
        *,
        require_octomap: bool = False,
        require_occupancy: bool = False,
        expected_data_source: str | None = None,
        expected_source_profile: str | None = None,
        expected_frame_id: str | None = None,
    ) -> dict[str, Any]:
        return self.validate_map_artifacts(
            name,
            require_octomap=require_octomap,
            require_occupancy=require_occupancy,
            expected_data_source=expected_data_source,
            expected_source_profile=expected_source_profile,
            expected_frame_id=expected_frame_id,
        )

    def _get_voxel_edits(self, name: str) -> dict[str, Any]:
        return self.api.get_voxel_edits(name)

    def _build_artifact(self, name: str, artifact_type: str) -> dict[str, Any]:
        return self.pipeline.build_artifact(name, artifact_type)

    def _get_pipeline_status(self, name: str) -> dict[str, Any]:
        return self.pipeline.get_status(name)

    def _get_save_map_status(self, job_id: str) -> dict[str, Any]:
        return self.storage.native_service.get_save_map_status(job_id)

    def _list_save_map_jobs(self, limit: int = 100) -> dict[str, Any]:
        return self.storage.native_service.list_save_map_jobs(limit=limit)

    def _cancel_save_map(self, job_id: str) -> dict[str, Any]:
        return self.storage.native_service.cancel_save_map(job_id)

    def _retry_save_map(self, job_id: str) -> dict[str, Any]:
        return self.storage.native_service.retry_save_map(job_id)

    def _list_map_versions(self, name: str) -> dict[str, Any]:
        return self.storage.native_service.list_map_versions(name)

    def _rollback_map_version(self, name: str, version: int) -> dict[str, Any]:
        if version <= 0:
            return {
                "action": "rollback_map_version",
                "success": False,
                "reason_code": "invalid_version",
                "map_id": name,
                "version": version,
                "message": "version must be a positive integer",
            }
        return self.storage.native_service.rollback_map_version(name, version)

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
        map_opt: str | None = None,
        request_id: str | None = None,
    ) -> dict[str, Any]:
        semantic_snapshot = None
        if self.semantic_save_request.callback_count > 0:

            def capture_semantic(path: Path) -> dict[str, Any]:
                return self._save_live_semantic_map(
                    name,
                    artifact=Path(path),
                    commit=False,
                )

            semantic_snapshot = capture_semantic
        return self.pipeline.save(
            name,
            slam_profile=slam_profile,
            map_opt=map_opt,
            request_id=request_id,
            semantic_snapshot=semantic_snapshot,
        )

    def _save_live_semantic_map(
        self,
        name: str,
        *,
        artifact: Path | None = None,
        commit: bool = True,
    ) -> dict[str, Any]:
        if self.semantic_save_request.callback_count == 0:
            return {
                "action": "save_semantic_artifact",
                "success": False,
                "map_id": name,
                "message": "semantic map snapshot is not wired",
                "reason_code": "semantic_snapshot_unavailable",
            }
        target = artifact or (self.storage.map_path(name) / "semantic_map.bin")
        request_id = uuid.uuid4().hex
        event = threading.Event()
        results: list[SemanticSaveResult] = []
        with self._semantic_save_lock:
            self._semantic_save_waiters[request_id] = (event, results, name, target)
        try:
            self.semantic_save_request.publish(
                SemanticSaveRequest(
                    request_id=request_id,
                    map_id=name,
                    path=str(target),
                )
            )
            if not event.wait(self._semantic_save_timeout_sec) or not results:
                return {
                    "action": "save_semantic_artifact",
                    "success": False,
                    "map_id": name,
                    "message": "semantic map snapshot timed out or is not wired",
                    "reason_code": "semantic_snapshot_unavailable",
                }
            result = results[-1]
        finally:
            with self._semantic_save_lock:
                self._semantic_save_waiters.pop(request_id, None)
        payload: dict[str, Any] = {
            "action": "save_semantic_artifact",
            "success": result.success,
            "map_id": name,
            "path": str(target),
            "generation": result.generation,
            "voxel_count": result.voxel_count,
            "message": result.message,
        }
        if not result.success:
            payload["reason_code"] = "semantic_snapshot_failed"
            return payload
        if not commit:
            payload["transactional_visibility"] = "captured_for_native_save_job"
            return payload
        committed = self.pipeline.build_semantic_artifact(name)
        if committed.get("success") is not True:
            return {
                **payload,
                "success": False,
                "reason_code": str(committed.get("reason_code") or "semantic_commit_failed"),
                "message": str(committed.get("message") or "native semantic commit failed"),
                "validation": committed,
            }
        payload["validation"] = committed
        payload["transactional_visibility"] = "committed_atomic_artifact"
        return payload

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

    def _build_esdf_artifact(self, name: str) -> dict[str, Any]:
        """Build esdf.npz from occupancy.npz using native maps C++."""
        return self.pipeline.build_esdf_artifact(name)

    def _build_traversability_artifact(self, name: str) -> dict[str, Any]:
        """Build traversability.npz from occupancy/esdf using native maps C++."""
        return self.pipeline.build_traversability_artifact(name)

    def _map_delete(self, name: str) -> dict[str, Any]:
        """Delete an entire map directory."""
        return self.control.delete(name)

    def _map_retire(self, name: str) -> dict[str, Any]:
        """Mark a map retired without deleting its artifacts."""
        return self.control.retire(name)

    def _map_rename(self, name: str, new_name: str) -> dict[str, Any]:
        """Rename a map through the native lifecycle service."""
        return self.control.rename(name, new_name)

    def _map_set_active(self, name: str) -> dict[str, Any]:
        """Select the native active map after artifact validation."""
        return self.set_active_map(name)

    def _map_edit_voxels(self, name: str, cmd: dict[str, Any]) -> dict[str, Any]:
        """Edit saved OctoMap voxels used by OctoPlanner3D."""
        return self.control.edit_voxels(name, cmd)

    def _save_active_map(self) -> None:
        self.storage.save_active_map()
