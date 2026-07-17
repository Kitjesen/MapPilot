"""Control-plane lifecycle operations for maps."""

from __future__ import annotations

from typing import Any

from maps.services.pipeline import command_template
from maps.services.storage import InvalidMapName, MapStorageService
from runtime.runtime_interface import TOPICS, topic_default_frame_id


class MapControlService:
    """Owns map lifecycle mutations and active-map gating."""

    def __init__(
        self,
        *,
        storage: MapStorageService,
        octomap_editor_command: Any = None,
        octomap_edit_timeout_sec: float = 15.0,
    ) -> None:
        self.storage = storage
        self.octomap_editor_command = octomap_editor_command
        self.octomap_edit_timeout_sec = float(octomap_edit_timeout_sec)

    def create(self, name: str) -> dict[str, Any]:
        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {"action": "create", "success": False, "message": str(exc)}
        if map_dir.exists():
            return {"action": "create", "success": False, "message": f"map exists: {name}"}
        try:
            return self.storage.create_map(name)
        except RuntimeError as exc:
            return {"action": "create", "success": False, "message": str(exc)}

    def delete(self, name: str) -> dict[str, Any]:
        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {"action": "delete", "success": False, "message": str(exc)}
        if not map_dir.is_dir():
            return {
                "action": "delete",
                "success": False,
                "message": f"map not found: {name}",
            }

        native_resp = self.storage.delete_map(name)
        return native_resp

    def retire(self, name: str) -> dict[str, Any]:
        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {"action": "retire", "success": False, "message": str(exc)}
        if not map_dir.is_dir():
            return {"action": "retire", "success": False, "message": f"map not found: {name}"}
        return self.storage.retire_map(name)

    def rename(self, name: str, new_name: str) -> dict[str, Any]:
        try:
            src = self.storage.map_path(name)
            dst = self.storage.map_path(new_name)
        except InvalidMapName as exc:
            return {"action": "rename", "success": False, "message": str(exc)}

        if not src.is_dir():
            return {
                "action": "rename",
                "success": False,
                "message": f"map not found: {name}",
            }
        if dst.exists():
            return {
                "action": "rename",
                "success": False,
                "message": f"target exists: {new_name}",
            }

        try:
            return self.storage.rename_map(name, new_name)
        except (OSError, RuntimeError) as exc:
            return {"action": "rename", "success": False, "message": str(exc)}

    def restore_source_backup(self, name: str) -> dict[str, Any]:
        """Restore a source cloud through the native staged transaction."""
        response = self.storage.native_service.restore_source_backup(name)
        if response.get("success") is True and response.get("deactivated") is True:
            self.storage.active_map = ""
        return response

    def set_active(self, name: str) -> dict[str, Any]:
        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {"action": "set_active", "success": False, "message": str(exc)}
        if not map_dir.is_dir():
            return {
                "action": "set_active",
                "success": False,
                "message": f"map not found: {name}",
            }
        validation = self.storage.native_service.validate_artifacts(
            name,
            expected_frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
        )
        if validation.get("success") is not True:
            return validation
        artifact_gate = dict(validation.get("gate") or {})
        artifact_gate["required"] = True
        artifact_report = artifact_gate.get("artifacts") or {}
        has_runtime_artifact = any(
            bool((artifact_report.get(item) or {}).get("sha256_ok")) for item in ("octomap", "occupancy_grid")
        )
        if not has_runtime_artifact:
            artifact_gate.setdefault("blockers", []).append(
                "saved map has no usable runtime planning artifact: expected octomap.ot, octomap.bt, or occupancy.npz"
            )
            artifact_gate["ok"] = False
        if artifact_gate.get("ok") is not True:
            blockers = [str(item) for item in (artifact_gate.get("blockers") or []) if str(item)]
            detail = "; ".join(blockers) if blockers else "unknown blocker"
            return {
                "action": "set_active",
                "success": False,
                "message": f"saved map artifact gate failed: {detail}",
                "artifact_gate": artifact_gate,
            }

        try:
            response = self.storage.set_active_map(name, strict=True)
        except RuntimeError as exc:
            return {
                "action": "set_active",
                "success": False,
                "message": str(exc),
                "artifact_gate": artifact_gate,
            }
        response["artifact_gate"] = artifact_gate
        return response

    def edit_voxels(self, name: str, cmd: dict[str, Any]) -> dict[str, Any]:
        map_name = str(name or self.storage.active_map or "")
        center = cmd.get("center")
        try:
            if isinstance(center, dict):
                x = float(center.get("x"))
                y = float(center.get("y"))
                z = float(center.get("z", 0.0))
            elif isinstance(center, (list, tuple)):
                if len(center) < 2:
                    raise ValueError("center must contain at least x and y")
                x = float(center[0])
                y = float(center[1])
                z = float(center[2]) if len(center) > 2 else 0.0
            else:
                x = float(cmd.get("x"))
                y = float(cmd.get("y"))
                z = float(cmd.get("z", 0.0))
            return self.storage.native_service.edit_octomap_voxels(
                map_name,
                editor_command=command_template(self.octomap_editor_command),
                state=str(cmd.get("state") or cmd.get("mode") or "").strip().lower(),
                shape=str(cmd.get("shape") or "sphere").strip().lower(),
                x=x,
                y=y,
                z=z,
                radius=float(cmd.get("radius", 0.2)),
                timeout_sec=self.octomap_edit_timeout_sec,
            )
        except (TypeError, ValueError) as exc:
            return {
                "action": "edit_voxels",
                "success": False,
                "reason_code": "invalid_edit_request",
                "message": str(exc),
            }
