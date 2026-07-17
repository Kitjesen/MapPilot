"""Maps Module compatibility helpers, public skills, and health."""

from __future__ import annotations

import json
import threading
from typing import Any

from runtime import skill
from runtime.msgs.numpy_compat import np


class MapsFacadeMixin:
    @property
    def _active_map(self) -> str:
        return self.storage.active_map

    @_active_map.setter
    def _active_map(self, value: str) -> None:
        self.storage.active_map = value

    @property
    def _map_save_adapter(self) -> Any:
        return self.runtime_bridge.map_save_adapter

    @_map_save_adapter.setter
    def _map_save_adapter(self, value: Any) -> None:
        self.runtime_bridge.map_save_adapter = value

    @property
    def _map_save_timeout_sec(self) -> float:
        return self.runtime_bridge.map_save_timeout_sec

    @_map_save_timeout_sec.setter
    def _map_save_timeout_sec(self, value: float) -> None:
        self.runtime_bridge.map_save_timeout_sec = float(value)

    @property
    def _map_cloud_lock(self) -> threading.Lock:
        return self.runtime_bridge.map_cloud_lock

    @property
    def _latest_map_points(self) -> np.ndarray | None:
        return self.runtime_bridge.latest_map_points

    @_latest_map_points.setter
    def _latest_map_points(self, value: np.ndarray | None) -> None:
        self.runtime_bridge.latest_map_points = value

    @property
    def _map_artifact_converter_command(self) -> Any:
        return self.pipeline.map_artifact_converter_command

    @_map_artifact_converter_command.setter
    def _map_artifact_converter_command(self, value: Any) -> None:
        self.pipeline.map_artifact_converter_command = value

    @property
    def _octomap_build_mode(self) -> str:
        return self.pipeline.octomap_build_mode

    @_octomap_build_mode.setter
    def _octomap_build_mode(self, value: str) -> None:
        self.pipeline.octomap_build_mode = str(value)

    @property
    def _octomap_resolution(self) -> float:
        return self.pipeline.octomap_resolution

    @_octomap_resolution.setter
    def _octomap_resolution(self, value: float) -> None:
        self.pipeline.octomap_resolution = float(value)

    @property
    def _octomap_build_timeout_sec(self) -> float:
        return self.pipeline.octomap_build_timeout_sec

    @_octomap_build_timeout_sec.setter
    def _octomap_build_timeout_sec(self, value: float) -> None:
        self.pipeline.octomap_build_timeout_sec = float(value)

    @property
    def _build_octomap_on_save(self) -> bool:
        return self.pipeline.build_octomap_on_save

    @_build_octomap_on_save.setter
    def _build_octomap_on_save(self, value: bool) -> None:
        self.pipeline.build_octomap_on_save = bool(value)

    def get_active_octomap(self) -> str | None:
        """Return the active OctoMap path for the active map, or None."""
        return self.storage.get_active_octomap()

    def get_active_occupancy(self) -> str | None:
        """Return the occupancy.npz path for the active map, or None.

        A* backend and Dashboard may use this as a static planning grid.
        Load with: data = np.load(path); grid=data['grid'], res=data['resolution'],
        origin=data['origin'].
        """
        return self.storage.get_active_occupancy()

    def get_active_artifacts(self) -> dict[str, Any]:
        """Return active runtime artifact paths for map consumers."""
        return self.storage.get_active_artifacts()

    def get_map_types(self) -> dict[str, Any]:
        """Return the stable map class, artifact, and capability catalog."""
        return self.api.get_map_types()

    @skill
    def list_maps(self) -> str:
        """List saved maps and which one is active."""
        return json.dumps(self._map_list(), default=str)

    @skill
    def list_map_types(self) -> str:
        """List supported map classes, artifacts, and capabilities."""
        return json.dumps(self.get_map_types(), default=str)

    @skill
    def save_map(self, name: str, slam_profile: str | None = None) -> str:
        """Save current SLAM map as *name* and build all artifacts."""
        return json.dumps(self._map_save(name, slam_profile=slam_profile), default=str)

    @skill
    def use_map(self, name: str) -> str:
        """Activate *name* as the current map."""
        return json.dumps(self._map_set_active(name), default=str)

    def _poi_set(self, cmd: dict[str, Any]) -> dict[str, Any]:
        return self.api.poi_set(cmd)

    def _poi_delete(self, name: str, map_id: str = "") -> dict[str, Any]:
        return self.api.poi_delete(name, map_id=map_id)

    def _poi_list(self, map_id: str = "") -> dict[str, Any]:
        return self.api.poi_list(map_id=map_id)

    def _map_graph(self) -> dict[str, Any]:
        return self.api.list_map_graph()

    def _map_edge_set(self, cmd: dict[str, Any]) -> dict[str, Any]:
        return self.api.set_map_edge(cmd)

    def _map_edge_delete(self, cmd: dict[str, Any]) -> dict[str, Any]:
        return self.api.delete_map_edge(cmd)

    def _shortest_route(self, cmd: dict[str, Any]) -> dict[str, Any]:
        return self.api.shortest_route(cmd)

    def _rollback_active_map(self) -> dict[str, Any]:
        return self.api.rollback_active_map()

    def _active_slots(self) -> dict[str, Any]:
        return self.api.list_active_slots()

    def _get_active_slot(self, slot: str) -> dict[str, Any]:
        return self.api.get_active_slot(slot)

    def _set_active_slot(self, cmd: dict[str, Any]) -> dict[str, Any]:
        return self.api.set_active_slot(cmd)

    def _clear_active_slot(self, slot: str) -> dict[str, Any]:
        return self.api.clear_active_slot(slot)

    def _build_queue(self) -> dict[str, Any]:
        return self.api.get_build_queue()

    def _enqueue_build(self, cmd: dict[str, Any]) -> dict[str, Any]:
        return self.api.enqueue_build(cmd)

    def _pop_build_queue(self) -> dict[str, Any]:
        return self.api.pop_build_queue()

    def _get_artifact_job(self, request_id: str) -> dict[str, Any]:
        return self.api.get_artifact_job(request_id)

    def _cancel_artifact_job(self, request_id: str) -> dict[str, Any]:
        return self.api.cancel_artifact_job(request_id)

    def _retry_artifact_job(self, request_id: str) -> dict[str, Any]:
        return self.api.retry_artifact_job(request_id)

    def _audit_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self.api.audit_versions(dry_run=dry_run)

    def _quarantine_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self.api.quarantine_corrupt_versions(dry_run=dry_run)

    def _gc_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self.api.garbage_collect_versions(dry_run=dry_run)

    def _migrate_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self.api.migrate_versions(dry_run=dry_run)

    def _export_version(
        self,
        map_id: str,
        version: int,
        package_dir: str,
        *,
        dry_run: bool = False,
    ) -> dict[str, Any]:
        return self.api.export_version(
            map_id,
            version,
            package_dir,
            dry_run=dry_run,
        )

    def _import_package(
        self,
        package_dir: str,
        *,
        requested_map_id: str = "",
        dry_run: bool = False,
    ) -> dict[str, Any]:
        return self.api.import_package(
            package_dir,
            requested_map_id=requested_map_id,
            dry_run=dry_run,
        )

    def health(self) -> dict[str, Any]:
        info = self.port_summary()
        listed = self.api.list_maps()
        maps = listed.get("maps") if listed.get("success") is True else []
        maps = maps if isinstance(maps, list) else []
        pois = self.api.poi_list(self._active_map) if self._active_map else {}
        poi_items = pois.get("pois") if pois.get("success") is True else []
        poi_items = poi_items if isinstance(poi_items, list) else []
        info["map_count"] = len(maps)
        info["active_map"] = self._active_map
        info["poi_count"] = len(poi_items)
        info["map_dir"] = str(self._map_dir)
        return info
