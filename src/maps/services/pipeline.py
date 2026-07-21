"""Map artifact build pipeline for maps."""

from __future__ import annotations

import logging
import os
import re
import shlex
import time
from pathlib import Path
from typing import Any

from maps.services.opt import MapOpt
from maps.services.runtime_bridge import (
    MapRuntimeBridge,
    MapSaveAdapterFailed,
)
from maps.services.storage import InvalidMapName, MapStorageService
from runtime.runtime_interface import TOPICS, topic_default_frame_id

logger = logging.getLogger(__name__)


def _quote_command_part(value: str) -> str:
    if os.name != "nt":
        return shlex.quote(value)
    if value and not re.search(r'[\s"]', value):
        return value
    return '"' + value.replace('"', r"\"") + '"'


def command_template(value: Any) -> str:
    if value is None or value == "":
        return ""
    if isinstance(value, (list, tuple)):
        return " ".join(_quote_command_part(str(part)) for part in value)
    return str(value)


def _as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


class MapPipelineService:
    """Builds persistent map artifacts from source point clouds."""

    def __init__(
        self,
        *,
        storage: MapStorageService,
        runtime_bridge: MapRuntimeBridge,
        source_profile: str,
        runtime_data_source: str,
        map_artifact_converter_command: Any = None,
        octomap_build_mode: str = "external_pcl_converter",
        octomap_resolution: float = 0.20,
        octomap_free_layers_above: int = 3,
        octomap_free_dilation_cells: int = 1,
        octomap_build_timeout_sec: float = 60.0,
        build_octomap_on_save: bool = True,
        map_prune_command: Any = None,
        map_opt: str | None = None,
        map_opt_command: Any = None,
        map_opt_timeout_sec: float = 120.0,
        map_opt_required: bool = False,
        semantic_taxonomy_path: str | Path | None = None,
    ) -> None:
        self.storage = storage
        self.runtime_bridge = runtime_bridge
        self.source_profile = source_profile
        self.runtime_data_source = runtime_data_source
        self.map_artifact_converter_command = map_artifact_converter_command
        self.octomap_build_mode = str(octomap_build_mode or "external_pcl_converter")
        self.octomap_resolution = float(octomap_resolution)
        self.octomap_free_layers_above = int(octomap_free_layers_above)
        self.octomap_free_dilation_cells = int(octomap_free_dilation_cells)
        self.octomap_build_timeout_sec = float(octomap_build_timeout_sec)
        self.build_octomap_on_save = bool(build_octomap_on_save)
        self.map_prune_command = map_prune_command
        self.map_opt = MapOpt(
            strategy=map_opt,
            command=map_opt_command,
            timeout_sec=map_opt_timeout_sec,
            required=map_opt_required,
        )
        default_taxonomy = Path(__file__).resolve().parents[3] / "config" / "semantic_taxonomy.json"
        self.semantic_taxonomy_path = Path(
            semantic_taxonomy_path
            or os.environ.get("LINGTU_SEMANTIC_TAXONOMY")
            or default_taxonomy
        ).expanduser()

    def build_artifact(self, name: str, artifact_type: str) -> dict[str, Any]:
        kind = self.resolve_artifact_build_type(artifact_type)
        if kind in {"OCCUPANCY", "OCCUPANCY_2D"}:
            response = self.build_occupancy_snapshot(name)
        elif kind in {"OCTOMAP", "OCTOMAP_3D"}:
            response = self.build_octomap_artifact(name)
        elif kind == "ESDF":
            response = self.build_esdf_artifact(name)
        elif kind == "TRAVERSABILITY":
            response = self.build_traversability_artifact(name)
        elif kind == "SEMANTIC":
            response = self.build_semantic_artifact(name)
        else:
            return {
                "action": "build_artifact",
                "success": False,
                "reason_code": "unsupported_artifact_type",
                "message": f"unsupported artifact_type: {artifact_type}",
            }
        payload = dict(response)
        payload.setdefault("map_id", name)
        payload.setdefault("artifact_type", kind)
        return payload

    def get_status(self, name: str) -> dict[str, Any]:
        try:
            return self.storage.native_service.get_build_status(name)
        except Exception as exc:
            return {
                "action": "get_build_status",
                "success": False,
                "reason_code": "native_build_status_unavailable",
                "message": str(exc),
            }

    def resolve_artifact_build_type(self, value: str) -> str:
        key = str(value or "").strip()
        if not key:
            return ""
        lower = key.lower()
        catalog = _as_dict(self.storage.native_service.get_map_types())
        aliases = _as_dict(catalog.get("aliases"))
        lower = str(aliases.get(lower) or lower)
        artifacts = _as_dict(catalog.get("artifacts"))
        capabilities = _as_dict(catalog.get("capabilities"))
        spec = artifacts.get(lower)
        if isinstance(spec, dict) and spec.get("type"):
            return str(spec["type"])
        if lower in capabilities:
            return str(capabilities[lower])
        for item in artifacts.values():
            if isinstance(item, dict) and lower == str(item.get("map_class") or "").lower():
                return str(item.get("type") or "")
        upper = key.strip().upper()
        if upper in {"OCCUPANCY", "OCTOMAP"}:
            return upper
        return upper

    def import_pcd(
        self,
        name: str,
        source_path: str,
        *,
        voxel_size: float = 0.0,
        bounds: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Import a PCD into the canonical map package as map.pcd."""
        if not name:
            return {"action": "import_pcd", "success": False, "message": "missing map name"}
        if not source_path:
            return {"action": "import_pcd", "success": False, "message": "missing source_path"}
        try:
            resp = self.storage.native_service.import_pcd(
                name,
                Path(source_path).expanduser(),
                voxel_size=voxel_size,
                bounds=bounds,
            )
        except Exception as exc:
            return {"action": "import_pcd", "success": False, "message": str(exc)}
        return resp

    def crop(
        self,
        name: str,
        bounds: dict[str, Any],
        *,
        invert: bool = False,
        voxel_size: float = 0.0,
    ) -> dict[str, Any]:
        """Crop an existing map.pcd and mark derived artifacts stale."""
        try:
            resp = self.storage.native_service.crop_pcd(
                name,
                bounds,
                invert=invert,
                voxel_size=voxel_size,
            )
        except Exception as exc:
            return {"action": "crop", "success": False, "message": str(exc)}
        return resp

    def _save_success_response(
        self,
        name: str,
        *,
        job_status: dict[str, Any],
        resolved_slam_profile: str,
        capability_fields: dict[str, Any],
        semantic_result: dict[str, Any] | None = None,
        replayed: bool = False,
    ) -> dict[str, Any]:
        record_response = self.storage.native_service.get_record(name)
        record = _as_dict(record_response.get("record"))
        artifacts = record.get("artifacts")
        artifacts = artifacts if isinstance(artifacts, list) else []
        by_type = {str(item.get("type")): item for item in artifacts if isinstance(item, dict)}
        source_report = _as_dict(job_status.get("source_report"))
        map_optimization = _as_dict(source_report.get("map_optimization"))
        map_optimization_performed = map_optimization.get("performed") is True
        occupancy_ok = "OCCUPANCY_2D" in by_type
        octomap_ok = "OCTOMAP_3D" in by_type
        semantic_ok = "SEMANTIC" in by_type
        navigation_ready = octomap_ok if self.build_octomap_on_save else False
        version_dir = Path(str(job_status.get("version_dir") or ""))
        return {
            "action": "save",
            "success": True,
            "status": "ready" if navigation_ready else "partial",
            "job_id": job_status.get("job_id"),
            "map_id": name,
            "version": job_status.get("version"),
            "map_dir": job_status.get("version_dir"),
            "manifest": job_status.get("manifest_path"),
            "pcd": (by_type.get("POINTCLOUD") or {}).get("uri"),
            "occupancy": (by_type.get("OCCUPANCY_2D") or {}).get("uri"),
            "octomap": (by_type.get("OCTOMAP_3D") or {}).get("uri"),
            "esdf": (by_type.get("ESDF") or {}).get("uri"),
            "traversability": (by_type.get("TRAVERSABILITY") or {}).get("uri"),
            "metadata": str(version_dir / "metadata.json"),
            "occupancy_ok": occupancy_ok,
            "octomap_ok": octomap_ok,
            "metadata_ok": (version_dir / "metadata.json").is_file(),
            "point_count": source_report.get("point_count"),
            "map_optimization": map_optimization,
            "map_optimization_ok": (
                map_optimization_performed
                and map_optimization.get("success") is True
                and map_optimization.get("status") == "ok"
            ),
            "map_optimization_performed": map_optimization_performed,
            "source_map_transaction": source_report,
            "semantic": semantic_result,
            "semantic_ok": semantic_ok,
            "navigation_ready": navigation_ready,
            "compatibility_ready": job_status.get("compatibility_ready"),
            "compatibility_message": job_status.get("compatibility_message"),
            "transactional_visibility": "immutable_version_then_atomic_pointer",
            "record": record,
            "job": job_status,
            "slam_profile": resolved_slam_profile,
            "replayed": replayed,
            **capability_fields,
        }

    def save(
        self,
        name: str,
        slam_profile: str | None = None,
        *,
        request_id: str | None = None,
        map_opt: str | None = None,
        build_occupancy_snapshot: Any | None = None,
        build_octomap_artifact: Any | None = None,
        semantic_snapshot: Any | None = None,
    ) -> dict[str, Any]:
        """Capture runtime state and submit one durable native SaveMap job."""
        if not name:
            return {"action": "save", "success": False, "message": "missing map name"}
        if build_occupancy_snapshot is not None or build_octomap_artifact is not None:
            return {
                "action": "save",
                "success": False,
                "reason_code": "custom_map_builders_unsupported",
                "message": "SaveMap only accepts the native C++ artifact pipeline",
            }

        try:
            self.storage.map_path(name)
        except InvalidMapName as exc:
            return {
                "action": "save",
                "success": False,
                "reason_code": "invalid_map_name",
                "message": str(exc),
            }

        resolved_slam_profile = self.runtime_bridge.resolve_slam_profile(slam_profile)
        capability_fields = self.runtime_bridge.map_save_capability_fields(resolved_slam_profile)
        if not capability_fields.get("map_save_supported", True):
            return {
                "action": "save",
                "success": False,
                "reason_code": "map_save_unsupported",
                "message": (f"Map save is not supported for slam_profile={resolved_slam_profile!r}"),
                "slam_profile": resolved_slam_profile,
                **capability_fields,
            }

        request_id = str(request_id or f"save_{time.time_ns()}")
        if re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_-]{0,127}", request_id) is None:
            return {
                "action": "save",
                "success": False,
                "reason_code": "invalid_request_id",
                "message": "request_id must contain only letters, digits, '_' or '-'",
            }
        dynamic_filter_enabled = str(os.environ.get("LINGTU_SAVE_DYNAMIC_FILTER", "1")).strip().lower() not in {
            "0",
            "false",
            "no",
            "off",
            "",
        }
        dynamic_filter_required = str(os.environ.get("LINGTU_SAVE_DYNAMIC_FILTER_REQUIRED", "1")).strip().lower() in {
            "1",
            "true",
            "yes",
            "on",
        }
        opt_options = self.map_opt.source_options(map_opt)
        frame_id = topic_default_frame_id(TOPICS.saved_map_cloud)
        native = self.storage.native_service
        begin = native.begin_save_map(
            request_id,
            name,
            requirements={
                "occupancy": True,
                "octomap": self.build_octomap_on_save,
                "esdf": self.build_octomap_on_save,
                "traversability": self.build_octomap_on_save,
                "semantic": semantic_snapshot is not None,
            },
            source={
                "voxel_size": 0.0,
                "dynamic_filter_enabled": dynamic_filter_enabled,
                "dynamic_filter_required": dynamic_filter_required,
                "dynamic_filter_command": command_template(self.map_prune_command),
                "optimizer_strategy": opt_options["strategy"],
                "optimizer_required": opt_options["required"],
                "optimizer_command": command_template(opt_options["command"]),
                "optimizer_timeout_sec": opt_options["timeout_sec"],
            },
            octomap={
                "converter_command": command_template(self.map_artifact_converter_command),
                "build_mode": self.octomap_build_mode,
                "resolution": self.octomap_resolution,
                "support_dilation_cells": 1,
                "free_layers_above": self.octomap_free_layers_above,
                "free_dilation_cells": self.octomap_free_dilation_cells,
                "frame_id": frame_id,
                "source_profile": self.source_profile,
                "data_source": self.runtime_data_source,
                "slam_source": resolved_slam_profile,
                "localization_source": resolved_slam_profile,
                "mapping_source": "save_map_product_chain",
                "timeout_sec": self.octomap_build_timeout_sec,
            },
        )
        if begin.get("accepted") is not True:
            return {
                "action": "save",
                "success": False,
                "reason_code": str(begin.get("reason_code") or "save_job_rejected"),
                "message": "native SaveMap job was rejected",
                "job": begin,
            }
        begin_status = _as_dict(begin.get("status"))
        if begin.get("replayed") is True:
            replay_state = str(begin_status.get("state") or "")
            if replay_state == "SUCCEEDED":
                return self._save_success_response(
                    name,
                    job_status=begin_status,
                    resolved_slam_profile=resolved_slam_profile,
                    capability_fields=capability_fields,
                    replayed=True,
                )
            if replay_state in {"FAILED", "CANCELLED"}:
                return {
                    "action": "save",
                    "success": False,
                    "status": replay_state.lower(),
                    "reason_code": str(begin_status.get("reason_code") or "save_map_failed"),
                    "message": str(begin_status.get("message") or "previous SaveMap attempt did not succeed"),
                    "job_id": request_id,
                    "job": begin_status,
                    "replayed": True,
                }
            if replay_state in {"QUEUED", "RUNNING"}:
                return {
                    "action": "save",
                    "success": False,
                    "accepted": True,
                    "status": "running",
                    "reason_code": "save_job_in_progress",
                    "message": "the idempotent SaveMap job is already running",
                    "job_id": request_id,
                    "job": begin_status,
                    "replayed": True,
                }
        capture_dir_value = str(begin_status.get("capture_dir") or "")
        if not capture_dir_value:
            native.cancel_save_map(request_id)
            return {
                "action": "save",
                "success": False,
                "reason_code": "capture_dir_missing",
                "message": "native SaveMap job did not provide a capture directory",
                "job_id": request_id,
            }
        capture_dir = Path(capture_dir_value)
        capture_pcd = capture_dir / "map.pcd"

        try:
            if resolved_slam_profile == "super_lio":
                snapshot_result = self.runtime_bridge.save_live_map_cloud_snapshot(capture_pcd)
            else:
                snapshot_result = self.runtime_bridge.save_map_with_adapter(capture_pcd)
        except MapSaveAdapterFailed as exc:
            native.cancel_save_map(request_id)
            return {
                "action": "save",
                "success": False,
                "reason_code": "map_snapshot_failed",
                "message": str(exc),
                "job_id": request_id,
            }
        if snapshot_result.get("success") is False or not capture_pcd.is_file():
            native.cancel_save_map(request_id)
            snapshot_message = snapshot_result.get("message")
            if snapshot_result.get("success") is not False and not capture_pcd.is_file():
                snapshot_message = "Map save reported success but did not write map.pcd"
            return {
                "action": "save",
                "success": False,
                "reason_code": "map_snapshot_failed",
                "diagnostic_code": "map_pcd_missing_after_save",
                "message": str(snapshot_message or "map snapshot did not produce map.pcd"),
                "job_id": request_id,
            }

        semantic_result: dict[str, Any] | None = None
        if semantic_snapshot is not None:
            semantic_result = semantic_snapshot(capture_dir / "semantic_map.bin")
            if semantic_result.get("success") is not True:
                native.cancel_save_map(request_id)
                return {
                    "action": "save",
                    "success": False,
                    "reason_code": str(semantic_result.get("reason_code") or "semantic_snapshot_failed"),
                    "message": str(semantic_result.get("message") or "semantic snapshot failed"),
                    "job_id": request_id,
                    "semantic": semantic_result,
                }

        frame_info = dict(self.runtime_bridge.latest_map_frame_info)
        slam_healthy, health_message = self.runtime_bridge.snapshot_health()
        provided = native.provide_save_map_snapshot(
            request_id,
            capture_dir,
            snapshot_id=f"snapshot_{request_id}",
            frame_id=str(frame_info.get("frame_id") or frame_id),
            captured_at_ns=int(float(frame_info.get("ts") or time.time()) * 1e9),
            first_sequence=int(frame_info.get("first_sequence") or frame_info.get("sequence") or 0),
            last_sequence=int(frame_info.get("last_sequence") or frame_info.get("sequence") or 0),
            slam_healthy=slam_healthy,
            health_message=health_message,
        )
        if provided.get("accepted") is not True:
            return {
                "action": "save",
                "success": False,
                "reason_code": str(provided.get("reason_code") or "snapshot_rejected"),
                "message": "native SaveMap rejected the captured snapshot",
                "job_id": request_id,
                "job": provided,
            }

        wait_budget = max(
            30.0,
            float(self.runtime_bridge.map_save_timeout_sec)
            + float(self.map_opt.timeout_sec)
            + float(self.octomap_build_timeout_sec)
            + 30.0,
        )
        deadline = time.monotonic() + wait_budget
        job_status: dict[str, Any] = {}
        while time.monotonic() < deadline:
            status_response = native.get_save_map_status(request_id)
            job_status = _as_dict(status_response.get("status"))
            if job_status.get("state") in {"SUCCEEDED", "FAILED", "CANCELLED"}:
                break
            time.sleep(0.05)

        state = str(job_status.get("state") or "RUNNING")
        if state == "RUNNING" or state == "QUEUED":
            return {
                "action": "save",
                "success": False,
                "status": "running",
                "reason_code": "save_job_timeout",
                "message": "SaveMap is still running; query by job_id",
                "job_id": request_id,
                "job": job_status,
            }
        if state != "SUCCEEDED":
            return {
                "action": "save",
                "success": False,
                "status": state.lower(),
                "reason_code": str(job_status.get("reason_code") or "save_map_failed"),
                "message": str(job_status.get("message") or "SaveMap failed"),
                "job_id": request_id,
                "job": job_status,
                "semantic": semantic_result,
            }

        return self._save_success_response(
            name,
            job_status=job_status,
            resolved_slam_profile=resolved_slam_profile,
            capability_fields=capability_fields,
            semantic_result=semantic_result,
        )

    def build_navigation_package(
        self,
        name: str,
        *,
        slam_profile: str | None = None,
    ) -> dict[str, Any]:
        """Build the saved-map navigation package through one native transaction."""
        if not name:
            return {
                "action": "build_navigation_package",
                "success": False,
                "message": "missing map name",
            }
        resolved_slam_profile = (
            self.runtime_bridge.normalize_slam_profile(
                slam_profile or self.runtime_bridge.slam_profile or self.runtime_bridge.resolve_slam_profile(None)
            )
            or "unknown"
        )
        try:
            native_resp = self.storage.native_service.build_navigation_package(
                name,
                converter_command=command_template(self.map_artifact_converter_command),
                build_mode=self.octomap_build_mode,
                resolution=self.octomap_resolution,
                support_dilation_cells=1,
                free_layers_above=self.octomap_free_layers_above,
                free_dilation_cells=self.octomap_free_dilation_cells,
                frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
                source_profile=self.source_profile,
                data_source=self.runtime_data_source,
                slam_source=resolved_slam_profile,
                localization_source=resolved_slam_profile,
                mapping_source="map_manager_build_navigation_package",
                timeout_sec=self.octomap_build_timeout_sec,
                include_esdf=True,
                include_traversability=True,
            )
        except Exception as exc:
            return {
                "action": "build_navigation_package",
                "success": False,
                "status": "native_maps_unavailable",
                "message": f"native maps build_navigation_package failed: {exc}",
            }
        return native_resp

    def build_semantic_artifact(self, name: str) -> dict[str, Any]:
        """Delegate semantic artifact validation and catalog commit to C++."""
        try:
            return self.storage.native_service.build_semantic_artifact(name)
        except Exception as exc:
            return {
                "action": "build_semantic_artifact",
                "success": False,
                "reason_code": "native_semantic_service_unavailable",
                "message": str(exc),
            }

    def import_unity_semantic_artifact(
        self,
        name: str,
        scene_dir: str | Path,
        *,
        taxonomy_path: str | Path | None = None,
        frame_id: str = "map",
        voxel_size_m: float = 0.20,
        occupied_probability: float = 0.95,
        shell_thickness_voxels: float = 0.75,
        generation: int = 1,
        max_objects: int = 100_000,
        max_voxels: int = 2_000_000,
        max_voxel_checks: int = 50_000_000,
        include_unknown_geometry: bool = False,
        exclude_dynamic_classes: bool = True,
    ) -> dict[str, Any]:
        """Import one Unity scene through the native transactional semantic pipeline."""
        if not name:
            return {
                "action": "import_unity_semantic_artifact",
                "success": False,
                "reason_code": "missing_map_name",
                "message": "missing map name",
            }
        if not str(scene_dir or "").strip():
            return {
                "action": "import_unity_semantic_artifact",
                "success": False,
                "reason_code": "missing_scene_dir",
                "message": "missing Unity scene directory",
            }
        try:
            return self.storage.native_service.import_unity_semantic_artifact(
                name,
                Path(scene_dir).expanduser(),
                taxonomy_path=Path(taxonomy_path or self.semantic_taxonomy_path).expanduser(),
                frame_id=frame_id,
                voxel_size_m=voxel_size_m,
                occupied_probability=occupied_probability,
                shell_thickness_voxels=shell_thickness_voxels,
                generation=generation,
                max_objects=max_objects,
                max_voxels=max_voxels,
                max_voxel_checks=max_voxel_checks,
                include_unknown_geometry=include_unknown_geometry,
                exclude_dynamic_classes=exclude_dynamic_classes,
            )
        except Exception as exc:
            return {
                "action": "import_unity_semantic_artifact",
                "success": False,
                "reason_code": "native_unity_semantic_import_unavailable",
                "message": str(exc),
            }

    def build_octomap_artifact(
        self,
        name: str,
        *,
        slam_profile: str | None = None,
    ) -> dict[str, Any]:
        """Build or reuse octomap.ot for OctoPlanner3D runtime planning."""
        if not name:
            return {
                "action": "build_octomap",
                "success": False,
                "message": "missing map name",
            }

        resolved_slam_profile = (
            self.runtime_bridge.normalize_slam_profile(
                slam_profile or self.runtime_bridge.slam_profile or self.runtime_bridge.resolve_slam_profile(None)
            )
            or "unknown"
        )
        try:
            native_resp = self.storage.native_service.build_octomap_artifact(
                name,
                converter_command=command_template(self.map_artifact_converter_command),
                build_mode=self.octomap_build_mode,
                resolution=self.octomap_resolution,
                support_dilation_cells=1,
                free_layers_above=self.octomap_free_layers_above,
                free_dilation_cells=self.octomap_free_dilation_cells,
                frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
                source_profile=self.source_profile,
                data_source=self.runtime_data_source,
                slam_source=resolved_slam_profile,
                localization_source=resolved_slam_profile,
                mapping_source="map_manager_build_octomap",
                timeout_sec=self.octomap_build_timeout_sec,
            )
        except Exception as exc:
            return {
                "action": "build_octomap",
                "success": False,
                "status": "native_maps_unavailable",
                "message": f"native maps build_octomap failed: {exc}",
            }
        return native_resp

    def build_occupancy_snapshot(self, name: str) -> dict[str, Any]:
        """Build ROS2-compatible 2D occupancy grid from SLAM output."""
        if not name:
            return {
                "action": "build_occupancy_snapshot",
                "success": False,
                "message": "missing map name",
            }

        try:
            return self.storage.native_service.build_occupancy_snapshot(name)

        except Exception as exc:
            logger.warning("build_occupancy_snapshot failed for '%s': %s", name, exc)
            return {
                "action": "build_occupancy_snapshot",
                "success": False,
                "message": str(exc),
            }

    def build_esdf_artifact(self, name: str) -> dict[str, Any]:
        """Build esdf.npz from occupancy.npz using native maps C++."""
        if not name:
            return {
                "action": "build_esdf_artifact",
                "success": False,
                "message": "missing map name",
            }
        try:
            return self.storage.native_service.build_esdf_artifact(name)
        except Exception as exc:
            logger.warning("build_esdf_artifact failed for '%s': %s", name, exc)
            return {
                "action": "build_esdf_artifact",
                "success": False,
                "message": str(exc),
            }

    def build_traversability_artifact(self, name: str) -> dict[str, Any]:
        """Build traversability.npz from occupancy/esdf using native maps C++."""
        if not name:
            return {
                "action": "build_traversability_artifact",
                "success": False,
                "message": "missing map name",
            }
        try:
            return self.storage.native_service.build_traversability_artifact(name)
        except Exception as exc:
            logger.warning("build_traversability_artifact failed for '%s': %s", name, exc)
            return {
                "action": "build_traversability_artifact",
                "success": False,
                "message": str(exc),
            }
