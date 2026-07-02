"""Control-plane lifecycle operations for MapService."""

from __future__ import annotations

import shutil
from typing import Any

from nav.services.map.records import load_map_record
from nav.services.map.storage import InvalidMapName, MapStorageService
from nav.services.map.voxel_edit import edit_saved_octomap
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.same_source_map_artifacts import validate_saved_map_artifact_dir


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
        map_dir.mkdir(parents=True)
        record = self.storage.write_map_record(name, state="CREATED")
        return {
            "action": "create",
            "success": True,
            "map_id": name,
            "map_dir": str(map_dir),
            "record": record,
        }

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

        try:
            shutil.rmtree(map_dir)
            if self.storage.active_map == name:
                active_link = self.storage.active_link()
                if active_link.is_symlink() or active_link.exists():
                    active_link.unlink()
                self.storage.active_map = ""
                self.storage.save_active_map()
            return {
                "action": "delete",
                "success": True,
                "map_id": name,
                "message": f"deleted: {name}",
            }
        except OSError as exc:
            return {"action": "delete", "success": False, "message": str(exc)}

    def retire(self, name: str) -> dict[str, Any]:
        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {"action": "retire", "success": False, "message": str(exc)}
        if not map_dir.is_dir():
            return {"action": "retire", "success": False, "message": f"map not found: {name}"}
        if self.storage.active_map == name:
            active_link = self.storage.active_link()
            if active_link.is_symlink() or active_link.exists():
                active_link.unlink()
            self.storage.active_map = ""
            self.storage.save_active_map()
        record = self.storage.write_map_record(name, state="RETIRED")
        return {
            "action": "retire",
            "success": True,
            "map_id": name,
            "record": record,
        }

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
            was_active = self.storage.active_map == name
            shutil.move(str(src), str(dst))
            if was_active:
                self.storage.active_map = new_name
                self.storage.save_active_map()
                active_link = self.storage.active_link()
                if active_link.is_symlink() or active_link.exists():
                    active_link.unlink()
                active_link.symlink_to(dst)
            self.storage.write_map_record(
                new_name,
                state="ACTIVE" if was_active else "READY",
            )
            return {
                "action": "rename",
                "success": True,
                "map_id": new_name,
                "old_map_id": name,
                "message": f"{name} -> {new_name}",
            }
        except OSError as exc:
            return {"action": "rename", "success": False, "message": str(exc)}

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
        loaded_record = load_map_record(map_dir) or {}
        if str(loaded_record.get("state") or "").upper() == "RETIRED":
            return {
                "action": "set_active",
                "success": False,
                "message": f"map is retired: {name}",
                "record": loaded_record,
            }
        artifact_gate = validate_saved_map_artifact_dir(
            map_dir,
            expected_frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
        )
        artifact_gate["required"] = True
        artifact_report = artifact_gate.get("artifacts") or {}
        has_runtime_artifact = any(
            bool((artifact_report.get(item) or {}).get("sha256_ok"))
            for item in ("octomap", "tomogram", "occupancy_grid")
        )
        if not has_runtime_artifact:
            artifact_gate.setdefault("blockers", []).append(
                "saved map has no usable runtime planning artifact: expected "
                "octomap.ot, octomap.bt, tomogram.pickle, or occupancy.npz"
            )
            artifact_gate["ok"] = False
        if artifact_gate.get("ok") is not True:
            blockers = [
                str(item)
                for item in (artifact_gate.get("blockers") or [])
                if str(item)
            ]
            detail = "; ".join(blockers) if blockers else "unknown blocker"
            return {
                "action": "set_active",
                "success": False,
                "message": f"saved map artifact gate failed: {detail}",
                "artifact_gate": artifact_gate,
            }

        active_link = self.storage.active_link()
        if active_link.is_symlink() or active_link.exists():
            active_link.unlink()
        active_link.symlink_to(map_dir)

        self.storage.active_map = name
        self.storage.save_active_map()
        record = self.storage.write_map_record(name, state="ACTIVE", gate=artifact_gate)
        return {
            "action": "set_active",
            "success": True,
            "active": name,
            "tomogram": str(map_dir / "tomogram.pickle") if (map_dir / "tomogram.pickle").exists() else None,
            "octomap": self.storage.get_active_octomap(),
            "occupancy": str(map_dir / "occupancy.npz") if (map_dir / "occupancy.npz").exists() else None,
            "artifact_gate": artifact_gate,
            "record": record,
        }

    def edit_voxels(self, name: str, cmd: dict[str, Any]) -> dict[str, Any]:
        map_name = str(name or self.storage.active_map or "")
        try:
            map_dir = self.storage.map_path(map_name)
        except InvalidMapName as exc:
            return {"action": "edit_voxels", "success": False, "message": str(exc)}
        if not map_dir.is_dir():
            return {
                "action": "edit_voxels",
                "success": False,
                "message": f"map not found: {map_name}",
            }

        resp = edit_saved_octomap(
            map_dir,
            cmd,
            editor_command=self.octomap_editor_command,
            timeout_sec=self.octomap_edit_timeout_sec,
        )
        if resp.get("success") is True:
            resp["record"] = self.storage.write_map_record(
                map_name,
                state="ACTIVE" if self.storage.active_map == map_name else "READY",
            )
        return resp
