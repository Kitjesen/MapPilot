"""Active saved-map asset lookup owned by the maps domain."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from maps.adapters.python.store import MapStoreNativeUnavailable, NativeMapStore
from maps.paths import map_root_candidates
from runtime.runtime_interface import normalize_frame_id

_FILENAME_CAPABILITIES = {
    "map.pcd": "source_pointcloud",
    "occupancy.npz": "path_planning_2d",
    "octomap.ot": "navigation_safety_3d",
    "octomap.bt": "navigation_safety_3d",
    "esdf.npz": "trajectory_optimization",
    "traversability.npz": "traversability",
    "semantic_map.bin": "semantic_query",
}


def default_map_roots() -> tuple[Path, ...]:
    """Return canonical and explicit compatibility roots in priority order."""
    return map_root_candidates()


@dataclass(frozen=True)
class ActiveMapAssets:
    """Resolve active-map records and artifacts without leaking layout to clients."""

    map_roots: tuple[Path, ...] = ()

    @classmethod
    def from_runtime(cls) -> ActiveMapAssets:
        return cls(map_roots=default_map_roots())

    def active_artifact(self, filename: str) -> str:
        capability = _FILENAME_CAPABILITIES.get(str(filename or ""))
        if not capability:
            return ""
        bundle = self.active_bundle(capability)
        return str((bundle.get("artifact") or {}).get("uri") or "")

    def active_bundle(self, capability: str) -> dict[str, Any]:
        for root in self.map_roots:
            if not root.is_dir():
                continue
            store = NativeMapStore(root)
            try:
                active = store.active_map_id()
                if not active:
                    continue
                bundle = store.bundle(active, capability)
                if bundle.get("success") is True:
                    return bundle
            finally:
                store.close()
        return {}

    def static_occupancy_path(self, map_path: str = "") -> str:
        bundle = self.active_bundle("path_planning_2d")
        if bundle:
            return str((bundle.get("artifact") or {}).get("uri") or "")
        return ""

    def validate_artifact_path(
        self,
        artifact_path: str,
        *,
        require_octomap: bool = False,
        require_occupancy: bool = False,
        expected_frame_id: str | None = None,
    ) -> dict[str, Any]:
        capability = (
            "navigation_safety_3d"
            if require_octomap
            else "path_planning_2d"
            if require_occupancy
            else "source_pointcloud"
        )
        expected_frame = normalize_frame_id(str(expected_frame_id or "")) or ""
        required_artifacts = ["map_pcd"]
        if require_octomap:
            required_artifacts.append("octomap")
        if require_occupancy:
            required_artifacts.append("occupancy_grid")

        bundle: dict[str, Any] = {}
        native_gate: dict[str, Any] = {}
        validation_errors: list[str] = []
        for root in self.map_roots:
            if not root.is_dir():
                continue
            store: NativeMapStore | None = None
            try:
                store = NativeMapStore(root)
                active_map_id = store.active_map_id()
                if not active_map_id:
                    continue
                candidate = store.bundle(active_map_id, capability)
                if candidate.get("success") is not True:
                    continue
                bundle = candidate
                validation = store.validate_artifacts(
                    active_map_id,
                    require_octomap=require_octomap,
                    require_occupancy=require_occupancy,
                    expected_frame_id=expected_frame,
                )
                final_active_map_id = store.active_map_id()
                candidate_map_id = str(candidate.get("map_id") or "")
                validation_map_id = str(validation.get("map_id") or "")
                if (
                    final_active_map_id != active_map_id
                    or candidate_map_id != active_map_id
                    or validation_map_id != active_map_id
                ):
                    validation_errors.append("active map changed during artifact validation")
                    break
                candidate_gate = validation.get("gate")
                if validation.get("success") is True and isinstance(candidate_gate, dict):
                    native_gate = dict(candidate_gate)
                else:
                    message = str(validation.get("message") or "invalid native gate response")
                    validation_errors.append(f"native artifact validation failed: {message}")
                break
            except (MapStoreNativeUnavailable, OSError, RuntimeError, ValueError) as exc:
                validation_errors.append(f"native artifact validation unavailable: {exc}")
            finally:
                if store is not None:
                    store.close()

        blockers = [str(blocker) for blocker in (native_gate.get("blockers") or []) if str(blocker)]
        if native_gate and native_gate.get("ok") is not True and not blockers:
            blockers.append("native artifact validation rejected active map")
        blockers.extend(validation_errors)
        artifact = bundle.get("artifact") if isinstance(bundle, dict) else {}
        artifact = artifact if isinstance(artifact, dict) else {}
        uri = str(artifact.get("uri") or "")
        if not bundle or not uri:
            blockers.append(f"active map capability unavailable: {capability}")
        elif Path(uri).resolve() != Path(artifact_path).resolve():
            blockers.append("artifact path does not match active native bundle")
        digest = str(artifact.get("hash") or "")
        if uri and len(digest) != 64:
            blockers.append("native artifact hash is unavailable")
        checked_frame = (
            normalize_frame_id(str(native_gate.get("checked_frame_id") or bundle.get("frame_id") or "")) or ""
        )
        if expected_frame and checked_frame != expected_frame:
            mismatch = f"frame mismatch: expected {expected_frame}, got {checked_frame or 'unknown'}"
            if mismatch not in blockers:
                blockers.append(mismatch)

        gate = dict(native_gate)
        gate.update(
            {
                "schema_version": "lingtu.saved_map_artifacts.gate.v2",
                "ok": native_gate.get("ok") is True and not blockers,
                "checked_frame_id": checked_frame,
                "expected_frame_id": expected_frame,
                "checked_required_artifacts": required_artifacts,
                "blockers": blockers,
                "map_bundle": bundle,
            }
        )
        if not isinstance(gate.get("artifacts"), dict):
            gate["artifacts"] = {
                "octomap" if require_octomap else "occupancy_grid" if require_occupancy else "map_pcd": {
                    "exists": bool(uri),
                    "path": uri or None,
                    "sha256": digest,
                    "sha256_ok": len(digest) == 64,
                }
            }
        return gate
