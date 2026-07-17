"""Query and POI API layer for maps."""

from __future__ import annotations

from typing import Any

from maps.paths import map_export_root, map_import_root, resolve_exchange_path
from maps.services.storage import MapStorageService


class MapAPIService:
    """Read/query side of the Spatial Data Service."""

    def __init__(self, *, storage: MapStorageService) -> None:
        self.storage = storage

    def get_map_types(self) -> dict[str, Any]:
        """Return the native map class, artifact, and capability catalog."""
        return self.storage.native_service.get_map_types()

    def list_maps(self) -> dict[str, Any]:
        """List maps without rebuilding native records in Python."""
        response = self.storage.native_service.list_maps()
        if response.get("success") is True:
            response.setdefault("map_dir", str(self.storage.map_dir))
        return response

    def get_record(self, name: str) -> dict[str, Any]:
        return self.storage.native_service.get_record(name)

    def get_active_map(self) -> dict[str, Any]:
        native_resp = self.storage.native_service.get_active_map()
        if native_resp.get("success") is True:
            self.storage.active_map = str(native_resp.get("active") or "")
        return native_resp

    def get_map_health(self, name: str) -> dict[str, Any]:
        return self.storage.native_service.get_health(name)

    def get_map_bundle(self, name: str, capability: str) -> dict[str, Any]:
        return self.storage.native_service.get_bundle(name, capability)

    def get_map_points(self, name: str, *, max_points: int = 0) -> dict[str, Any]:
        if not name:
            name = self.storage.active_map
        if not name:
            return {
                "action": "get_map_points",
                "success": False,
                "reason_code": "missing_map_name",
                "message": "missing map name",
            }
        try:
            return self.storage.native_service.get_map_points(
                name,
                max_points=max(0, int(max_points or 0)),
            )
        except Exception as exc:
            return {
                "action": "get_map_points",
                "success": False,
                "reason_code": "native_map_points_unavailable",
                "message": str(exc),
            }

    def validate_artifacts(
        self,
        name: str,
        *,
        require_octomap: bool = False,
        require_occupancy: bool = False,
        expected_data_source: str | None = None,
        expected_source_profile: str | None = None,
        expected_frame_id: str | None = None,
    ) -> dict[str, Any]:
        """Validate one saved-map package inside the maps domain."""
        if not name:
            name = self.storage.active_map
        if not name:
            return {
                "action": "validate_artifacts",
                "success": False,
                "reason_code": "missing_map_name",
                "message": "missing map name",
            }
        response = self.storage.native_service.validate_artifacts(
            name,
            require_octomap=require_octomap,
            require_occupancy=require_occupancy,
            expected_frame_id=str(expected_frame_id or ""),
        )
        constraints = [
            label
            for label, value in (
                ("expected_data_source", expected_data_source),
                ("expected_source_profile", expected_source_profile),
            )
            if value
        ]
        if constraints and response.get("success") is True:
            gate = dict(response.get("gate") or {})
            blockers = list(gate.get("blockers") or [])
            blockers.append("native map provenance query does not expose: " + ", ".join(constraints))
            gate["ok"] = False
            gate["blockers"] = blockers
            gate["unsupported_constraints"] = constraints
            response["gate"] = gate
        return response

    def get_voxel_edits(self, name: str) -> dict[str, Any]:
        """Return the native manual-voxel edit journal."""
        map_id = str(name or self.storage.active_map or "")
        if not map_id:
            return {
                "action": "get_voxel_edits",
                "success": False,
                "reason_code": "missing_map_name",
                "message": "missing map name",
            }
        return self.storage.native_service.get_voxel_edits(map_id)

    def poi_set(self, cmd: dict[str, Any]) -> dict[str, Any]:
        name = str(cmd.get("name", ""))
        if not name:
            return {"action": "poi_set", "success": False, "message": "missing name"}
        return self.storage.native_service.set_poi(
            str(cmd.get("map_id", "") or cmd.get("map", "") or self.storage.active_map),
            name,
            x=float(cmd.get("x", 0.0)),
            y=float(cmd.get("y", 0.0)),
            z=float(cmd.get("z", 0.0)),
            yaw=(
                None
                if cmd.get("yaw") is None and cmd.get("yaw_rad") is None
                else float(cmd.get("yaw", cmd.get("yaw_rad", 0.0)))
            ),
            frame_id=str(cmd.get("frame_id", "map") or "map"),
            tags=cmd.get("tags") if isinstance(cmd.get("tags"), dict) else None,
        )

    def poi_delete(self, name: str, map_id: str = "") -> dict[str, Any]:
        return self.storage.native_service.delete_poi(map_id or self.storage.active_map, name)

    def poi_list(self, map_id: str = "") -> dict[str, Any]:
        return self.storage.native_service.list_poi(map_id or self.storage.active_map)

    def list_map_graph(self) -> dict[str, Any]:
        return self.storage.native_service.list_map_graph()

    def set_map_edge(self, cmd: dict[str, Any]) -> dict[str, Any]:
        return self.storage.native_service.set_map_edge(
            str(cmd.get("from", "") or cmd.get("from_map_id", "")),
            str(cmd.get("to", "") or cmd.get("to_map_id", "")),
            edge_type=str(cmd.get("type", "") or cmd.get("edge_type", "") or "link"),
            bidirectional=bool(cmd.get("bidirectional", True)),
        )

    def delete_map_edge(self, cmd: dict[str, Any]) -> dict[str, Any]:
        return self.storage.native_service.delete_map_edge(
            str(cmd.get("from", "") or cmd.get("from_map_id", "")),
            str(cmd.get("to", "") or cmd.get("to_map_id", "")),
        )

    def shortest_route(self, cmd: dict[str, Any]) -> dict[str, Any]:
        return self.storage.native_service.shortest_route(
            str(cmd.get("from", "") or cmd.get("start", "") or cmd.get("start_map_id", "")),
            str(cmd.get("to", "") or cmd.get("goal", "") or cmd.get("goal_map_id", "")),
        )

    def rollback_active_map(self) -> dict[str, Any]:
        response = self.storage.native_service.rollback_active_map()
        if response.get("success") is True:
            self.storage.active_map = str(response.get("active") or "")
        return response

    def list_active_slots(self) -> dict[str, Any]:
        return self.storage.native_service.list_active_slots()

    def get_active_slot(self, slot: str) -> dict[str, Any]:
        return self.storage.native_service.get_active_slot(slot)

    def set_active_slot(self, cmd: dict[str, Any]) -> dict[str, Any]:
        slot = str(cmd.get("slot", "") or cmd.get("name", ""))
        response = self.storage.native_service.set_active_slot(
            slot,
            str(cmd.get("map_id", "") or cmd.get("map", "") or cmd.get("active", "")),
            strict=bool(cmd.get("strict", True)),
        )
        if response.get("success") is True and slot == "navigation":
            self.storage.active_map = str(response.get("active") or "")
        return response

    def clear_active_slot(self, slot: str) -> dict[str, Any]:
        response = self.storage.native_service.clear_active_slot(slot)
        if response.get("success") is True and slot == "navigation":
            self.storage.active_map = ""
        return response

    def get_build_queue(self) -> dict[str, Any]:
        return self.storage.native_service.get_build_queue()

    def enqueue_build(self, cmd: dict[str, Any]) -> dict[str, Any]:
        return self.storage.native_service.enqueue_build(
            str(cmd.get("name", "") or cmd.get("map_id", "")),
            str(cmd.get("artifact_type", "") or cmd.get("type", "") or cmd.get("capability", "")),
        )

    def pop_build_queue(self) -> dict[str, Any]:
        return self.storage.native_service.pop_build_queue()

    def get_artifact_job(self, request_id: str) -> dict[str, Any]:
        return self.storage.native_service.get_artifact_job(request_id)

    def cancel_artifact_job(self, request_id: str) -> dict[str, Any]:
        return self.storage.native_service.cancel_artifact_job(request_id)

    def retry_artifact_job(self, request_id: str) -> dict[str, Any]:
        return self.storage.native_service.retry_artifact_job(request_id)

    def audit_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self.storage.native_service.audit_versions(dry_run=dry_run)

    def quarantine_corrupt_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self.storage.native_service.quarantine_corrupt_versions(dry_run=dry_run)

    def garbage_collect_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self.storage.native_service.garbage_collect_versions(dry_run=dry_run)

    def migrate_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self.storage.native_service.migrate_versions(dry_run=dry_run)

    def export_version(
        self,
        map_id: str,
        version: int,
        package_dir: str,
        *,
        dry_run: bool = False,
    ) -> dict[str, Any]:
        try:
            target = resolve_exchange_path(
                package_dir,
                root=map_export_root(self.storage.map_dir),
                must_exist=False,
            )
        except ValueError as exc:
            return {
                "action": "export_version",
                "success": False,
                "reason_code": "unsafe_exchange_path",
                "message": str(exc),
            }
        return self.storage.native_service.export_version(
            map_id,
            version,
            target,
            dry_run=dry_run,
        )

    def import_package(
        self,
        package_dir: str,
        *,
        requested_map_id: str = "",
        dry_run: bool = False,
    ) -> dict[str, Any]:
        try:
            source = resolve_exchange_path(
                package_dir,
                root=map_import_root(self.storage.map_dir),
                must_exist=True,
                require_dir=True,
            )
        except ValueError as exc:
            return {
                "action": "import_package",
                "success": False,
                "reason_code": "unsafe_exchange_path",
                "message": str(exc),
            }
        return self.storage.native_service.import_package(
            source,
            requested_map_id=requested_map_id,
            dry_run=dry_run,
        )
