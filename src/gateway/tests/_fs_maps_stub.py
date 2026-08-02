"""Patch script: add filesystem-backed NativeMapStore/NativeMapsService stubs."""
import hashlib
import json
import os
import shutil
from pathlib import Path
from unittest.mock import patch

# Known octomap filenames used by different pipeline stages.
_OCTOMAP_FILENAMES = ("octomap.ot", "navigation_safety_3d.ot", "octomap.bt")
_OCCUPANCY_FILENAMES = ("path_planning_2d.yaml", "occupancy.npz")


def _find_first(root: Path, candidates: tuple[str, ...]) -> Path | None:
    for name in candidates:
        candidate = root / name
        if candidate.is_file():
            return candidate
    return None


def _sha256_file(path: Path) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return ""


class _FilesystemNativeStore:
    """Pure-Python NativeMapStore substitute backed by the map directory."""

    def __init__(self, root_dir, active_state_filename="active_map.txt"):
        self._root = Path(root_dir)
        self._root.mkdir(parents=True, exist_ok=True)
        self._active_file = self._root / active_state_filename

    def active_map_id(self):
        if self._active_file.is_file():
            text = self._active_file.read_text(encoding="utf-8").strip()
            return text
        return ""

    def close(self):
        pass

    def list_map_ids(self):
        if not self._root.is_dir():
            return []
        return sorted(
            d.name
            for d in self._root.iterdir()
            if d.is_dir() and (d / "metadata.json").is_file()
        )

    def record(self, name):
        map_dir = self._root / name
        if not map_dir.is_dir():
            return {"success": False, "message": f"map not found: {name}"}
        meta = {}
        meta_path = map_dir / "metadata.json"
        if meta_path.is_file():
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        return {
            "success": True,
            "name": name,
            "map_dir": str(map_dir),
            "pcd": str(map_dir / "map.pcd") if (map_dir / "map.pcd").is_file() else "",
            "artifacts": self._map_artifacts(map_dir),
            **meta,
        }

    def _map_artifacts(self, map_dir):
        artifacts = {}
        octomap = _find_first(map_dir, _OCTOMAP_FILENAMES)
        if octomap is not None:
            artifacts["octomap"] = str(octomap)
        occupancy = _find_first(map_dir, _OCCUPANCY_FILENAMES)
        if occupancy is not None:
            artifacts["occupancy"] = str(occupancy)
        return artifacts


class _FilesystemNativeService:
    """Pure-Python NativeMapsService substitute backed by the map directory."""

    def __init__(self, root_dir, active_state_filename="active_map.txt"):
        self._root = Path(root_dir)
        self._root.mkdir(parents=True, exist_ok=True)
        self._active_file = self._root / active_state_filename

    def close(self):
        pass

    def _active_name(self):
        if self._active_file.is_file():
            return self._active_file.read_text(encoding="utf-8").strip()
        return ""

    def _map_dir(self, name):
        candidate = self._root / name
        return candidate if candidate.is_dir() else None

    def _map_artifacts(self, map_dir):
        artifacts = {}
        octomap = _find_first(map_dir, _OCTOMAP_FILENAMES)
        if octomap is not None:
            artifacts["octomap"] = str(octomap)
        occupancy = _find_first(map_dir, _OCCUPANCY_FILENAMES)
        if occupancy is not None:
            artifacts["occupancy"] = str(occupancy)
        return artifacts

    def _map_record(self, map_dir):
        name = map_dir.name
        meta = {}
        meta_path = map_dir / "metadata.json"
        if meta_path.is_file():
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        return {
            "name": name,
            "map_dir": str(map_dir),
            "has_pcd": (map_dir / "map.pcd").is_file(),
            "has_octomap": _find_first(map_dir, _OCTOMAP_FILENAMES) is not None,
            "artifacts": self._map_artifacts(map_dir),
            **meta,
        }

    def create_map(self, name):
        map_dir = self._root / name
        map_dir.mkdir(parents=True, exist_ok=True)
        meta_path = map_dir / "metadata.json"
        if not meta_path.is_file():
            meta_path.write_text(json.dumps({"name": name}), encoding="utf-8")
        return {"success": True, "name": name, "map_dir": str(map_dir)}

    def rename_map(self, old_name, new_name):
        src = self._map_dir(old_name)
        if src is None:
            return {"success": False, "message": f"map not found: {old_name}"}
        dst = self._root / new_name
        src.rename(dst)
        return {"success": True, "name": new_name}

    def retire_map(self, name):
        return {"success": True, "name": name}

    def delete_map(self, name):
        map_dir = self._map_dir(name)
        if map_dir is None:
            return {"success": False, "message": f"map not found: {name}"}
        shutil.rmtree(map_dir, ignore_errors=True)
        return {"success": True, "name": name}

    def set_active_map(self, name, *, strict=False):
        map_dir = self._map_dir(name)
        if map_dir is None:
            return {"success": False, "message": f"map not found: {name}"}
        self._active_file.write_text(f"{name}\n", encoding="utf-8")
        return {"success": True, "active": name, "artifacts": self._map_artifacts(map_dir)}

    def clear_active_map(self):
        if self._active_file.is_file():
            self._active_file.unlink()
        return {"success": True}

    def get_active_map(self):
        active = self._active_name()
        if not active:
            return {"success": True, "active": "", "artifacts": {}, "record": None}
        map_dir = self._map_dir(active)
        if map_dir is None:
            return {"success": True, "active": "", "artifacts": {}, "record": None}
        return {
            "success": True,
            "active": active,
            "artifacts": self._map_artifacts(map_dir),
            "record": self._map_record(map_dir),
        }

    def get_save_map_status(self, job_id):
        return {"success": True, "job_id": job_id, "status": "idle"}

    def list_save_map_jobs(self, limit=10):
        return []

    def cancel_save_map(self, job_id):
        return {"success": True}

    def retry_save_map(self, job_id):
        return {"success": True}

    def list_map_versions(self, name):
        return []

    def rollback_map_version(self, name, version):
        return {"success": True}

    def ingest_localization_health(self, map_name, health):
        pass

    def ingest_planning_outcome(self, map_name, outcome):
        pass

    def ingest_collision_event(self, map_name, event):
        pass

    # -- list_maps ------------------------------------------------------------

    def list_maps(self):
        """Return a list of map records from the filesystem."""
        active = self._active_name()
        items = []
        for map_dir in sorted(self._root.iterdir()):
            if not map_dir.is_dir():
                continue
            if not (map_dir / "metadata.json").is_file():
                continue
            record = self._map_record(map_dir)
            name = record.get("name") or map_dir.name
            items.append({
                "name": name,
                "has_pcd": record.get("has_pcd", False),
                "has_octomap": record.get("has_octomap", False),
                "has_occupancy": "occupancy" in record.get("artifacts", {}),
                "is_active": name == active,
                "state": "STALE",
                "navigation_ready": False,
                "size_mb": 0.0,
                "patch_count": 0,
                "record": record,
                "artifacts": record.get("artifacts", {}),
            })
        return {
            "success": True,
            "active": active,
            "maps": items,
            "map_dir": str(self._root),
        }

    # -- artifact validation --------------------------------------------------

    def validate_artifacts(
        self,
        map_id,
        *,
        require_octomap=False,
        require_occupancy=False,
        expected_frame_id="",
    ):
        """Pure-Python validate_artifacts matching the native contract."""
        map_dir = self._map_dir(map_id)
        if map_dir is None:
            return {
                "success": False,
                "map_id": map_id,
                "message": f"map not found: {map_id}",
                "gate": {"ok": False, "blockers": [f"map not found: {map_id}"]},
            }
        meta_path = map_dir / "metadata.json"
        meta = {}
        if meta_path.is_file():
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        meta_artifacts = meta.get("artifacts") or {}
        gate_artifacts = {}
        blockers = []
        # -- octomap --
        octomap_meta = meta_artifacts.get("octomap") or {}
        octomap_path_rel = octomap_meta.get("path") or ""
        octomap_full = map_dir / octomap_path_rel if octomap_path_rel else _find_first(map_dir, _OCTOMAP_FILENAMES)
        if require_octomap:
            if not octomap_full or not Path(octomap_full).is_file():
                blockers.append("required octomap artifact missing on disk")
            else:
                actual_sha = _sha256_file(Path(octomap_full))
                expected_sha = str(octomap_meta.get("sha256") or "")
                sha_ok = (not expected_sha) or actual_sha == expected_sha
                gate_artifacts["octomap"] = {
                    "sha256_ok": sha_ok,
                    "source_map_sha256_matches_map": self._check_source_map_sha256(
                        map_dir, meta_artifacts, octomap_meta
                    ),
                }
                if not sha_ok:
                    blockers.append("octomap sha256 mismatch")
        else:
            if octomap_full and Path(octomap_full).is_file():
                gate_artifacts["octomap"] = {"sha256_ok": True}
        # -- occupancy --
        occ_meta = meta_artifacts.get("occupancy") or {}
        occ_path_rel = occ_meta.get("path") or ""
        occ_full = map_dir / occ_path_rel if occ_path_rel else _find_first(map_dir, _OCCUPANCY_FILENAMES)
        if require_occupancy:
            if not occ_full or not Path(occ_full).is_file():
                blockers.append("required occupancy artifact missing on disk")
            else:
                gate_artifacts.setdefault("occupancy", {"sha256_ok": True})
        # -- frame --
        checked_frame_id = str(meta.get("frame_id") or "")
        ok = not blockers
        return {
            "success": ok,
            "map_id": map_id,
            "map_dir": str(map_dir),
            "gate": {
                "ok": ok,
                "artifacts": gate_artifacts,
                "checked_required_artifacts": [
                    k for k, v in (("octomap", require_octomap), ("occupancy", require_occupancy)) if v
                ],
                "blockers": blockers,
                "checked_frame_id": checked_frame_id,
                "metadata": {
                    "source_profile": meta.get("source_profile", ""),
                    "data_source": meta.get("data_source", ""),
                    "frame_id": checked_frame_id,
                },
            },
        }

    @staticmethod
    def _check_source_map_sha256(map_dir, meta_artifacts, octomap_meta):
        source_map_sha = str(octomap_meta.get("source_map_sha256") or "")
        if not source_map_sha:
            return True
        pcd_meta = meta_artifacts.get("map_pcd") or {}
        pcd_rel = pcd_meta.get("path") or "map.pcd"
        pcd_full = map_dir / pcd_rel
        if not pcd_full.is_file():
            return False
        return _sha256_file(pcd_full) == source_map_sha

    # -- import_pcd -----------------------------------------------------------

    def import_pcd(self, map_id, source_path, *, voxel_size=0.0, bounds=None):
        """Copy a PCD file into the map directory as map.pcd."""
        src = Path(source_path)
        if not src.is_file():
            return {"action": "import_pcd", "success": False, "message": f"source not found: {source_path}"}
        map_dir = self._root / map_id
        map_dir.mkdir(parents=True, exist_ok=True)
        dst = map_dir / "map.pcd"
        shutil.copy2(src, dst)
        meta = {"name": map_id}
        meta_path = map_dir / "metadata.json"
        if meta_path.is_file():
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        meta_path.write_text(json.dumps(meta, sort_keys=True), encoding="utf-8")
        return {
            "action": "import_pcd",
            "success": True,
            "name": map_id,
            "map_dir": str(map_dir),
            "has_pcd": True,
            "has_octomap": False,
            "navigation_ready": False,
        }


def install_filesystem_maps_patches():
    """Return context managers that patch native map classes with filesystem fakes."""
    store_patch = patch("maps.services.storage.NativeMapStore", _FilesystemNativeStore)
    service_patch = patch("maps.services.storage.NativeMapsService", _FilesystemNativeService)
    return store_patch, service_patch
