"""Map artifact build pipeline for MapService."""

from __future__ import annotations

import json
import logging
import pickle
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any

from runtime.dynamic_filter import apply_dynamic_filter_step1half
from runtime.msgs.numpy_compat import np
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.same_source_map_artifacts import (
    build_saved_map_metadata,
    sha256_file,
    validate_same_source_map_metadata,
    validate_saved_map_artifact_dir,
)
from nav.services.map_layers.map_artifact_builder import (
    MapArtifactBuilder,
    MapArtifactBuilderConfig,
)
from nav.services.map.runtime_bridge import (
    LiveMapCloudSnapshotSaved,
    MapRuntimeBridge,
    MapSaveAdapterFailed,
)
from nav.services.map.records import (
    ARTIFACT_ALIASES,
    ARTIFACT_SPECS,
    CAPABILITY_TO_ARTIFACT_TYPE,
)
from nav.services.map.storage import InvalidMapName, MapStorageService

logger = logging.getLogger(__name__)


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
        octomap_build_timeout_sec: float = 60.0,
        build_octomap_on_save: bool = True,
    ) -> None:
        self.storage = storage
        self.runtime_bridge = runtime_bridge
        self.source_profile = source_profile
        self.runtime_data_source = runtime_data_source
        self.map_artifact_converter_command = map_artifact_converter_command
        self.octomap_build_mode = str(octomap_build_mode or "external_pcl_converter")
        self.octomap_resolution = float(octomap_resolution)
        self.octomap_build_timeout_sec = float(octomap_build_timeout_sec)
        self.build_octomap_on_save = bool(build_octomap_on_save)

    def build_artifact(self, name: str, artifact_type: str) -> dict[str, Any]:
        kind = self.resolve_artifact_build_type(artifact_type)
        if kind in {"TOMOGRAM", "TOMOGRAM_2_5D"}:
            return self.build_tomogram(name)
        if kind in {"OCCUPANCY", "OCCUPANCY_2D"}:
            return self.build_occupancy_snapshot(name)
        if kind in {"OCTOMAP", "OCTOMAP_3D"}:
            return self.build_octomap_artifact(name)
        if kind in {"ESDF", "SEMANTIC"}:
            return {
                "action": "build_artifact",
                "success": False,
                "reason_code": "builder_unavailable",
                "artifact_type": kind,
                "message": f"builder unavailable for artifact_type: {artifact_type}",
            }
        return {
            "action": "build_artifact",
            "success": False,
            "reason_code": "unsupported_artifact_type",
            "message": f"unsupported artifact_type: {artifact_type}",
        }

    @staticmethod
    def resolve_artifact_build_type(value: str) -> str:
        key = str(value or "").strip()
        if not key:
            return ""
        lower = key.lower()
        lower = ARTIFACT_ALIASES.get(lower, lower)
        if lower in ARTIFACT_SPECS:
            return ARTIFACT_SPECS[lower]["type"]
        if lower in CAPABILITY_TO_ARTIFACT_TYPE:
            return CAPABILITY_TO_ARTIFACT_TYPE[lower]
        for spec in ARTIFACT_SPECS.values():
            if lower == spec["map_class"]:
                return spec["type"]
        upper = key.strip().upper()
        if upper in {"OCCUPANCY", "TOMOGRAM", "OCTOMAP"}:
            return upper
        return upper

    def save(
        self,
        name: str,
        slam_profile: str | None = None,
        *,
        build_tomogram: Any | None = None,
        build_occupancy_snapshot: Any | None = None,
        build_octomap_artifact: Any | None = None,
    ) -> dict[str, Any]:
        """Save a SLAM map and build all planning artifacts."""
        if not name:
            return {"action": "save", "success": False, "message": "missing map name"}
        build_tomogram = build_tomogram or self.build_tomogram
        build_occupancy_snapshot = (
            build_occupancy_snapshot or self.build_occupancy_snapshot
        )
        build_octomap_artifact = build_octomap_artifact or self.build_octomap_artifact

        resolved_slam_profile = self.runtime_bridge.resolve_slam_profile(slam_profile)
        capability_fields = self.runtime_bridge.map_save_capability_fields(
            resolved_slam_profile
        )
        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {"action": "save", "success": False, "message": str(exc)}
        map_dir.mkdir(parents=True, exist_ok=True)
        pcd_path = map_dir / "map.pcd"

        if not capability_fields.get("map_save_supported", True):
            return {
                "action": "save",
                "success": False,
                "message": (
                    f"Map save is not supported for slam_profile="
                    f"{resolved_slam_profile!r}"
                ),
                "slam_profile": resolved_slam_profile,
                **capability_fields,
            }

        try:
            if resolved_slam_profile == "super_lio":
                snapshot_resp = self.runtime_bridge.save_live_map_cloud_snapshot(
                    pcd_path
                )
                if not snapshot_resp["success"]:
                    return {
                        "action": "save",
                        "success": False,
                        "message": snapshot_resp["message"],
                        "slam_profile": resolved_slam_profile,
                        **capability_fields,
                    }
                raise LiveMapCloudSnapshotSaved()

            adapter_result = self.runtime_bridge.save_map_with_adapter(pcd_path)
            if adapter_result.get("success") is False:
                raise MapSaveAdapterFailed(
                    str(adapter_result.get("message") or "Map save failed")
                )
        except MapSaveAdapterFailed as exc:
            return {
                "action": "save",
                "success": False,
                "message": str(exc),
                "slam_profile": resolved_slam_profile,
                **capability_fields,
            }
        except LiveMapCloudSnapshotSaved:
            pass

        dufo_result: dict[str, Any] | None = apply_dynamic_filter_step1half(map_dir)

        tomo_result = build_tomogram(name)
        tomo_ok = bool(tomo_result.get("success", False))
        if not tomo_ok and resolved_slam_profile != "super_lio":
            return {
                "action": "save",
                "success": False,
                "message": (
                    "Tomogram build failed; map was saved but is not ready "
                    f"for navigation: {tomo_result.get('message', 'unknown error')}"
                ),
                "map_dir": str(map_dir),
                "pcd": str(pcd_path),
                "slam_profile": resolved_slam_profile,
                "source": capability_fields["map_save_source"],
                "tomogram": tomo_result.get("tomogram"),
                "tomogram_ok": False,
                "tomogram_message": tomo_result.get("message", ""),
                **capability_fields,
            }

        occ_result = build_occupancy_snapshot(name)

        octomap_result = (
            build_octomap_artifact(
                name,
                slam_profile=resolved_slam_profile,
            )
            if self.build_octomap_on_save
            else {
                "action": "build_octomap",
                "success": False,
                "status": "disabled",
                "message": "octomap build on save disabled",
                "octomap": str(map_dir / "octomap.bt"),
            }
        )
        octomap_ok = bool(octomap_result.get("success", False))
        metadata_result = (
            octomap_result.get("metadata")
            if octomap_ok
            else None
        ) or (
            occ_result.get("metadata")
            or tomo_result.get("metadata")
            or self.write_saved_map_metadata(
                name,
                slam_profile=resolved_slam_profile,
                mapping_source=capability_fields["map_save_source"],
            )
        )

        resp: dict[str, Any] = {
            "action": "save",
            "success": True,
            "map_id": name,
            "map_dir": str(map_dir),
            "pcd": str(pcd_path),
            "slam_profile": resolved_slam_profile,
            "source": capability_fields["map_save_source"],
            "tomogram": tomo_result.get("tomogram"),
            "tomogram_ok": tomo_ok,
            "occupancy": occ_result.get("occupancy"),
            "occupancy_ok": occ_result.get("success", False),
            "octomap": octomap_result.get("octomap"),
            "octomap_ok": octomap_ok,
            "octomap_status": octomap_result.get("status", ""),
            "metadata": metadata_result.get("path"),
            "metadata_ok": bool(metadata_result.get("ok")),
            **capability_fields,
        }
        if resolved_slam_profile == "super_lio":
            resp["warnings"] = [
                "Super-LIO: saved map uses a live map_cloud snapshot; "
                "saved-map relocalization is unsupported"
            ]
        if not occ_result.get("success"):
            resp["occupancy_message"] = occ_result.get("message", "")
        if not octomap_ok:
            resp["octomap_message"] = octomap_result.get("message", "")
        if dufo_result is not None:
            resp["dynamic_filter"] = dufo_result
        return resp

    def write_saved_map_metadata(
        self,
        name: str,
        *,
        slam_profile: str | None = None,
        mapping_source: str = "map_manager",
        tomogram_shape: list[int] | None = None,
        occupancy_shape: list[int] | None = None,
    ) -> dict[str, Any]:
        """Write metadata.json binding map.pcd and derived planning artifacts."""
        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {"ok": False, "path": "", "blockers": [str(exc)]}
        pcd_path = map_dir / "map.pcd"
        metadata_path = map_dir / "metadata.json"
        if not pcd_path.exists():
            return {
                "ok": False,
                "path": str(metadata_path),
                "blockers": [f"no PCD file at {pcd_path}"],
            }

        resolved_slam = self.runtime_bridge.normalize_slam_profile(
            slam_profile
            or self.runtime_bridge.slam_profile
            or self.runtime_bridge.resolve_slam_profile(None)
        )
        frame_id = topic_default_frame_id(TOPICS.saved_map_cloud)
        pcd_sha = sha256_file(pcd_path)
        artifacts: dict[str, dict[str, Any]] = {
            "map_pcd": {
                "path": pcd_path.name,
                "sha256": pcd_sha,
                "source_profile": self.source_profile,
                "data_source": self.runtime_data_source,
                "slam_source": resolved_slam,
                "frame_id": frame_id,
                "point_count": self.pcd_point_count(pcd_path),
            }
        }

        tomogram_path = map_dir / "tomogram.pickle"
        if tomogram_path.exists():
            shape = tomogram_shape or self.load_tomogram_shape(tomogram_path)
            artifacts["tomogram"] = {
                "path": tomogram_path.name,
                "sha256": sha256_file(tomogram_path),
                "source_map_sha256": pcd_sha,
                "source_profile": self.source_profile,
                "data_source": self.runtime_data_source,
                "frame_id": frame_id,
                "shape": shape,
            }

        occupancy_path = map_dir / "occupancy.npz"
        if occupancy_path.exists():
            artifacts["occupancy_grid"] = {
                "path": occupancy_path.name,
                "sha256": sha256_file(occupancy_path),
                "source_map_sha256": pcd_sha,
                "source_profile": self.source_profile,
                "data_source": self.runtime_data_source,
                "frame_id": frame_id,
                "shape": occupancy_shape or self.load_occupancy_shape(occupancy_path),
            }

        metadata = build_saved_map_metadata(
            source_profile=self.source_profile,
            data_source=self.runtime_data_source,
            slam_source=resolved_slam or "unknown",
            localization_source=resolved_slam or "unknown",
            mapping_source=mapping_source,
            frame_id=frame_id,
            artifacts=artifacts,
            extra_metadata={"map_name": name, "source": "map_manager"},
        )
        source_validation = validate_same_source_map_metadata(metadata)
        metadata_path.write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        gate = validate_saved_map_artifact_dir(
            map_dir,
            expected_frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
        )
        record = self.storage.write_map_record(
            name,
            state="READY",
            gate=gate,
        )
        return {
            "ok": source_validation["ok"],
            "path": str(metadata_path),
            "sha256": sha256_file(metadata_path),
            "blockers": list(source_validation["blockers"]),
            "map_record": record["path"],
            "record": record,
        }

    def build_tomogram(self, name: str) -> dict[str, Any]:
        """Build tomogram.pickle from map.pcd for PCT/A* compatibility."""
        if not name:
            return {
                "action": "build_tomogram",
                "success": False,
                "message": "missing map name",
            }

        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {
                "action": "build_tomogram",
                "success": False,
                "message": str(exc),
            }
        pcd_path = map_dir / "map.pcd"
        tomogram_path = map_dir / "tomogram.pickle"

        if not pcd_path.exists():
            return {
                "action": "build_tomogram",
                "success": False,
                "message": f"no PCD file at {pcd_path}",
            }

        try:
            from nav.services.plan.global_planner.algorithm.pct.vendor.pct_planner.tomography.scripts.build_tomogram import (
                build_tomogram_from_pcd,
            )

            tomogram_data = build_tomogram_from_pcd(str(pcd_path), str(tomogram_path))
            metadata = self.write_saved_map_metadata(
                name,
                slam_profile=self.runtime_bridge.slam_profile,
                mapping_source="map_manager_build_tomogram",
                tomogram_shape=self.tomogram_shape_from_data(tomogram_data),
            )
            if not metadata["ok"]:
                return {
                    "action": "build_tomogram",
                    "success": False,
                    "message": (
                        "saved map metadata contract failed: "
                        + "; ".join(metadata["blockers"])
                    ),
                    "tomogram": str(tomogram_path),
                    "metadata": metadata,
                }
            return {
                "action": "build_tomogram",
                "success": True,
                "tomogram": str(tomogram_path),
                "metadata": metadata,
            }
        except Exception as exc:
            return {
                "action": "build_tomogram",
                "success": False,
                "message": str(exc),
            }

    def build_octomap_artifact(
        self,
        name: str,
        *,
        slam_profile: str | None = None,
    ) -> dict[str, Any]:
        """Build or reuse octomap.bt for OctoPlanner3D runtime planning."""
        if not name:
            return {
                "action": "build_octomap",
                "success": False,
                "message": "missing map name",
            }

        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {
                "action": "build_octomap",
                "success": False,
                "message": str(exc),
            }
        resolved_slam_profile = self.runtime_bridge.normalize_slam_profile(
            slam_profile
            or self.runtime_bridge.slam_profile
            or self.runtime_bridge.resolve_slam_profile(None)
        ) or "unknown"
        builder = MapArtifactBuilder(
            MapArtifactBuilderConfig(
                converter_command=self.map_artifact_converter_command,
                use_env_converter=True,
                build_mode=self.octomap_build_mode,
                resolution=self.octomap_resolution,
                frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
                source_profile=self.source_profile,
                data_source=self.runtime_data_source,
                slam_source=resolved_slam_profile,
                localization_source=resolved_slam_profile,
                mapping_source="map_manager_build_octomap",
                timeout_sec=self.octomap_build_timeout_sec,
            )
        )
        report = builder.build_for_saved_map(map_dir)
        payload = report.to_dict()
        message = ""
        if not report.ok:
            message = "; ".join(report.blockers) or report.status
        record = None
        artifact_gate = None
        success = bool(report.ok)
        if report.ok:
            artifact_gate = validate_saved_map_artifact_dir(
                map_dir,
                expected_frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
            )
            if artifact_gate.get("ok") is True:
                record = self.storage.write_map_record(
                    name,
                    state="READY",
                    gate=artifact_gate,
                )
            else:
                success = False
                blockers = [str(item) for item in artifact_gate.get("blockers") or []]
                message = "saved map artifact gate failed"
                if blockers:
                    message += ": " + "; ".join(blockers)
        return {
            "action": "build_octomap",
            "success": success,
            "status": report.status,
            "message": message,
            "octomap": report.octomap_path,
            "metadata": {
                "ok": success,
                "path": report.metadata_path,
                "validation": report.metadata_validation,
                "artifact_gate": artifact_gate,
                "map_record": None if record is None else record["path"],
            },
            "record": record,
            "report": payload,
        }

    def build_occupancy_snapshot(self, name: str) -> dict[str, Any]:
        """Build ROS2-compatible 2D occupancy grid from SLAM output."""
        if not name:
            return {
                "action": "build_occupancy_snapshot",
                "success": False,
                "message": "missing map name",
            }

        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {
                "action": "build_occupancy_snapshot",
                "success": False,
                "message": str(exc),
            }
        pcd_path = map_dir / "map.pcd"
        occ_path = map_dir / "occupancy.npz"
        pgm_path = map_dir / "map.pgm"
        yaml_path = map_dir / "map.yaml"
        poses_path = map_dir / "poses.txt"
        patches_dir = map_dir / "patches"

        if not pcd_path.exists():
            return {
                "action": "build_occupancy_snapshot",
                "success": False,
                "message": f"no PCD file at {pcd_path}",
            }

        try:
            has_pgo = (
                poses_path.exists()
                and patches_dir.is_dir()
                and any(patches_dir.iterdir())
            )
            if has_pgo:
                grid, resolution, origin = self.build_occupancy_raycasting(
                    poses_path,
                    patches_dir,
                )
                mode = "raycasting"
            else:
                grid, resolution, origin = self.build_occupancy_projection(pcd_path)
                mode = "projection"

            np.savez_compressed(
                str(occ_path),
                grid=grid,
                resolution=np.float64(resolution),
                origin=origin.astype(np.float64),
            )
            self.save_occupancy_pgm_yaml(
                grid,
                resolution,
                origin,
                pgm_path,
                yaml_path,
            )

            n_unk = int((grid == -1).sum())
            n_fre = int((grid == 0).sum())
            n_occ = int((grid == 100).sum())
            logger.info(
                "Occupancy '%s' (%s) shape=%s res=%.3f  unknown=%d free=%d occupied=%d",
                name,
                mode,
                grid.shape,
                resolution,
                n_unk,
                n_fre,
                n_occ,
            )
            metadata = self.write_saved_map_metadata(
                name,
                slam_profile=self.runtime_bridge.slam_profile,
                mapping_source=f"map_manager_build_occupancy_{mode}",
                occupancy_shape=[int(value) for value in grid.shape],
            )
            if not metadata["ok"]:
                return {
                    "action": "build_occupancy_snapshot",
                    "success": False,
                    "message": (
                        "saved map metadata contract failed: "
                        + "; ".join(metadata["blockers"])
                    ),
                    "occupancy": str(occ_path),
                    "metadata": metadata,
                }
            return {
                "action": "build_occupancy_snapshot",
                "success": True,
                "occupancy": str(occ_path),
                "pgm": str(pgm_path),
                "yaml": str(yaml_path),
                "metadata": metadata,
                "mode": mode,
                "grid_shape": list(grid.shape),
                "resolution": float(resolution),
                "origin": origin.tolist(),
                "counts": {"unknown": n_unk, "free": n_fre, "occupied": n_occ},
            }

        except Exception as exc:
            logger.warning("build_occupancy_snapshot failed for '%s': %s", name, exc)
            return {
                "action": "build_occupancy_snapshot",
                "success": False,
                "message": str(exc),
            }

    @staticmethod
    def pcd_point_count(path: Path) -> int:
        try:
            with path.open("rb") as fh:
                for raw in fh:
                    line = raw.decode("ascii", errors="ignore").strip()
                    upper = line.upper()
                    if upper.startswith("POINTS"):
                        parts = line.split()
                        if len(parts) >= 2:
                            return int(parts[1])
                    if upper.startswith("DATA"):
                        break
        except Exception as exc:
            logger.debug("_pcd_point_count failed for %s: %s", path, exc)
            return 0
        return 0

    @staticmethod
    def tomogram_shape_from_data(data: Any) -> list[int]:
        if not isinstance(data, dict):
            return []
        if isinstance(data.get("shape"), (list, tuple)):
            return [int(value) for value in data["shape"]]
        for key in ("data", "tomogram"):
            if key in data:
                arr = np.asarray(data[key])
                return [int(value) for value in arr.shape]
        return []

    @classmethod
    def load_tomogram_shape(cls, path: Path) -> list[int]:
        try:
            with path.open("rb") as fh:
                return cls.tomogram_shape_from_data(pickle.load(fh))  # noqa: S301
        except Exception as exc:
            logger.debug("_load_tomogram_shape failed for %s: %s", path, exc)
            return []

    @staticmethod
    def load_occupancy_shape(path: Path) -> list[int]:
        try:
            data = np.load(str(path))
            return [int(value) for value in np.asarray(data["grid"]).shape]
        except Exception as exc:
            logger.debug("_load_occupancy_shape failed for %s: %s", path, exc)
            return []

    @classmethod
    def build_occupancy_raycasting(
        cls,
        poses_path: Path,
        patches_dir: Path,
    ) -> tuple[np.ndarray, float, np.ndarray]:
        """Log-odds Bayesian occupancy via raycasting from keyframe poses."""
        resolution: float = 0.10
        z_min_rel: float = 0.10
        z_max_rel: float = 2.00
        max_range: float = 30.0
        log_occ: float = 0.85
        log_free: float = -0.40
        log_min: float = -2.0
        log_max: float = 3.5
        thresh_occ: float = 0.65
        thresh_free: float = 0.196

        poses = cls.parse_poses_txt(poses_path)
        if not poses:
            raise RuntimeError(f"no valid poses in {poses_path}")

        frame_data: list[tuple[np.ndarray, np.ndarray]] = []
        ground_z_list: list[float] = []
        xy_min = np.array([+np.inf, +np.inf], dtype=np.float32)
        xy_max = np.array([-np.inf, -np.inf], dtype=np.float32)

        for pose in poses:
            patch_path = cls.safe_patch_path(patches_dir, str(pose["patch"]))
            if patch_path is None:
                continue
            if not patch_path.is_file():
                continue
            body_pts = cls.load_pcd_points(str(patch_path))
            if body_pts is None or body_pts.shape[0] == 0:
                continue
            rot = cls.quat_to_rot(pose["q"])
            trans = pose["t"]
            world_pts = body_pts @ rot.T + trans
            dx = world_pts[:, 0] - trans[0]
            dy = world_pts[:, 1] - trans[1]
            dist = np.sqrt(dx * dx + dy * dy)
            world_pts = world_pts[dist < max_range]
            if world_pts.shape[0] == 0:
                continue
            frame_data.append((trans[:2].copy(), world_pts))
            xy_min = np.minimum(xy_min, world_pts[:, :2].min(axis=0))
            xy_max = np.maximum(xy_max, world_pts[:, :2].max(axis=0))
            ground_z_list.append(float(np.percentile(world_pts[:, 2], 5)))

        if not frame_data:
            raise RuntimeError("no usable patches loaded")

        ground_z = float(np.median(ground_z_list))
        z_lo, z_hi = ground_z + z_min_rel, ground_z + z_max_rel

        border = 1.0
        origin = (xy_min - border).astype(np.float64)
        grid_w = int(np.ceil((xy_max[0] + border - origin[0]) / resolution)) + 1
        grid_h = int(np.ceil((xy_max[1] + border - origin[1]) / resolution)) + 1
        if grid_w <= 0 or grid_h <= 0 or grid_w * grid_h > 25_000_000:
            raise RuntimeError(f"grid size out of range: {grid_w}x{grid_h}")

        log_odds = np.zeros((grid_h, grid_w), dtype=np.float32)

        for origin_xy, world_pts in frame_data:
            ox = int(np.floor((origin_xy[0] - origin[0]) / resolution))
            oy = int(np.floor((origin_xy[1] - origin[1]) / resolution))

            end_col = np.floor((world_pts[:, 0] - origin[0]) / resolution).astype(np.int32)
            end_row = np.floor((world_pts[:, 1] - origin[1]) / resolution).astype(np.int32)

            cls.raycast_free(
                log_odds,
                ox,
                oy,
                end_col,
                end_row,
                grid_w,
                grid_h,
                log_free,
            )

            obs_mask = (world_pts[:, 2] >= z_lo) & (world_pts[:, 2] <= z_hi)
            hc = end_col[obs_mask]
            hr = end_row[obs_mask]
            valid = (hc >= 0) & (hc < grid_w) & (hr >= 0) & (hr < grid_h)
            if valid.any():
                np.add.at(log_odds, (hr[valid], hc[valid]), log_occ)

            np.clip(log_odds, log_min, log_max, out=log_odds)

        grid = np.full((grid_h, grid_w), -1, dtype=np.int8)
        prob = 1.0 / (1.0 + np.exp(-log_odds))
        grid[prob > thresh_occ] = 100
        grid[(prob < thresh_free) & (log_odds < -0.01)] = 0

        return grid, resolution, origin

    @classmethod
    def build_occupancy_projection(
        cls,
        pcd_path: Path,
    ) -> tuple[np.ndarray, float, np.ndarray]:
        """Fallback: height-filter + XY projection."""
        pts = cls.load_pcd_points(str(pcd_path))
        if pts is None or pts.shape[0] == 0:
            raise RuntimeError("PCD file empty or unparseable")

        resolution: float = 0.20
        z_min_rel: float = 0.10
        z_max_rel: float = 2.00

        ground_z = float(np.percentile(pts[:, 2], 5))
        z_lo, z_hi = ground_z + z_min_rel, ground_z + z_max_rel
        mask = (pts[:, 2] >= z_lo) & (pts[:, 2] <= z_hi)
        obs_pts = pts[mask, :2]

        xy_min = pts[:, :2].min(axis=0)
        xy_max = pts[:, :2].max(axis=0)
        border = resolution
        origin = (xy_min - border).astype(np.float64)
        grid_w = int(np.ceil((xy_max[0] + border - origin[0]) / resolution)) + 1
        grid_h = int(np.ceil((xy_max[1] + border - origin[1]) / resolution)) + 1
        grid = np.zeros((grid_h, grid_w), dtype=np.int8)

        if obs_pts.shape[0] > 0:
            col = np.floor((obs_pts[:, 0] - origin[0]) / resolution).astype(np.int32)
            row = np.floor((obs_pts[:, 1] - origin[1]) / resolution).astype(np.int32)
            valid = (col >= 0) & (col < grid_w) & (row >= 0) & (row < grid_h)
            grid[row[valid], col[valid]] = 100

        return grid, resolution, origin

    @staticmethod
    def save_occupancy_pgm_yaml(
        grid: np.ndarray,
        resolution: float,
        origin: np.ndarray,
        pgm_path: Path,
        yaml_path: Path,
    ) -> None:
        """ROS2 map_server compatible PGM + YAML."""
        pgm = np.full_like(grid, 205, dtype=np.uint8)
        pgm[grid == 0] = 254
        pgm[grid == 100] = 0
        pgm = np.flipud(pgm)

        height, width = pgm.shape
        with open(pgm_path, "wb") as fh:
            fh.write(f"P5\n{width} {height}\n255\n".encode())
            fh.write(pgm.tobytes())

        yaml_body = {
            "image": pgm_path.name,
            "resolution": float(resolution),
            "origin": [float(origin[0]), float(origin[1]), 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
            "mode": "trinary",
        }
        try:
            from runtime.yaml_helpers import yaml as yaml_lib

            if yaml_lib is None:
                raise ImportError
            with open(yaml_path, "w", encoding="utf-8") as fh:
                yaml_lib.safe_dump(
                    yaml_body,
                    fh,
                    default_flow_style=False,
                    sort_keys=False,
                )
        except ImportError:
            yaml_path.write_text(
                f"image: {yaml_body['image']}\n"
                f"resolution: {yaml_body['resolution']}\n"
                f"origin: [{yaml_body['origin'][0]}, {yaml_body['origin'][1]}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
                f"mode: trinary\n"
            )

    @staticmethod
    def raycast_free(
        log_odds: np.ndarray,
        ox: int,
        oy: int,
        end_col: np.ndarray,
        end_row: np.ndarray,
        grid_w: int,
        grid_h: int,
        log_free: float,
    ) -> None:
        """Vectorised free-space update along rays via DDA rasterisation."""
        if end_col.size == 0:
            return
        dx = end_col - ox
        dy = end_row - oy
        steps = np.maximum(np.abs(dx), np.abs(dy))
        steps = np.minimum(steps, 500).astype(np.int32)
        max_s = int(steps.max()) if steps.size else 0
        if max_s < 1:
            return

        chunk = max(1, 200_000 // max_s)
        n = end_col.size
        ks = np.arange(max_s, dtype=np.float32)

        for start in range(0, n, chunk):
            end = min(n, start + chunk)
            dx_b = dx[start:end].astype(np.float32)
            dy_b = dy[start:end].astype(np.float32)
            steps_b = steps[start:end].astype(np.float32)
            frac = ks[None, :] / np.maximum(steps_b[:, None], 1.0)
            cols = ox + frac * dx_b[:, None]
            rows = oy + frac * dy_b[:, None]
            valid_len = ks[None, :] < steps_b[:, None]
            cols = np.floor(cols).astype(np.int32)
            rows = np.floor(rows).astype(np.int32)
            in_bounds = (cols >= 0) & (cols < grid_w) & (rows >= 0) & (rows < grid_h)
            mask = in_bounds & valid_len
            if not mask.any():
                continue
            cols = cols[mask]
            rows = rows[mask]
            np.add.at(log_odds, (rows, cols), log_free)

    @staticmethod
    def parse_poses_txt(path: Path) -> list[dict]:
        """Parse PGO poses.txt: each line 'patch.pcd tx ty tz qw qx qy qz'."""
        poses: list[dict] = []
        with open(path, encoding="utf-8") as fh:
            for line in fh:
                parts = line.strip().split()
                if len(parts) != 8:
                    continue
                if MapPipelineService._unsafe_relative_path(parts[0]):
                    continue
                try:
                    poses.append(
                        {
                            "patch": parts[0],
                            "t": np.array(
                                [
                                    float(parts[1]),
                                    float(parts[2]),
                                    float(parts[3]),
                                ],
                                dtype=np.float32,
                            ),
                            "q": np.array(
                                [
                                    float(parts[4]),
                                    float(parts[5]),
                                    float(parts[6]),
                                    float(parts[7]),
                                ],
                                dtype=np.float32,
                            ),
                        }
                    )
                except ValueError:
                    continue
        return poses

    @staticmethod
    def safe_patch_path(patches_dir: Path, patch_name: str) -> Path | None:
        """Resolve a poses.txt patch path without allowing directory escape."""
        if MapPipelineService._unsafe_relative_path(patch_name):
            return None
        root = patches_dir.resolve()
        candidate = (patches_dir / patch_name).resolve()
        try:
            candidate.relative_to(root)
        except ValueError:
            return None
        return candidate

    @staticmethod
    def _unsafe_relative_path(value: str) -> bool:
        if not value:
            return True
        if Path(value).is_absolute() or PureWindowsPath(value).is_absolute():
            return True
        for path_cls in (PurePosixPath, PureWindowsPath):
            if any(part in {"", ".", ".."} for part in path_cls(value).parts):
                return True
        return False

    @staticmethod
    def quat_to_rot(q: np.ndarray) -> np.ndarray:
        """Quaternion (w, x, y, z) -> 3x3 rotation matrix."""
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        n = np.sqrt(w * w + x * x + y * y + z * z)
        if n < 1e-9:
            return np.eye(3, dtype=np.float32)
        w, x, y, z = w / n, x / n, y / n, z / n
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float32,
        )

    @staticmethod
    def load_pcd_points(pcd_path: str) -> np.ndarray | None:
        """Load XYZ points from a PCD file."""
        try:
            import open3d as o3d  # type: ignore

            cloud = o3d.io.read_point_cloud(pcd_path)
            pts = np.asarray(cloud.points, dtype=np.float32)
            if pts.shape[0] > 0:
                return pts
        except Exception as exc:
            logger.debug("_load_pcd_points: open3d failed for %s: %s", pcd_path, exc)

        try:
            pts_list: list[list[float]] = []
            header: dict[str, list[str]] = {}
            with open(pcd_path, "rb") as fh:
                while True:
                    raw = fh.readline()
                    if not raw:
                        return None
                    line = raw.decode("ascii", errors="replace").strip()
                    if not line or line.startswith("#"):
                        continue
                    parts = line.split()
                    key = parts[0].upper()
                    header[key] = parts[1:]
                    if key == "DATA":
                        data_fmt = parts[-1].lower() if len(parts) > 1 else ""
                        if data_fmt == "binary":
                            return MapPipelineService._load_binary_pcd_points(
                                fh,
                                header,
                            )
                        if data_fmt not in ("ascii", ""):
                            return None
                        break
                for raw in fh:
                    line = raw.decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) >= 3:
                        try:
                            pts_list.append(
                                [
                                    float(parts[0]),
                                    float(parts[1]),
                                    float(parts[2]),
                                ]
                            )
                        except ValueError:
                            continue
            if pts_list:
                return np.array(pts_list, dtype=np.float32)
        except Exception as exc:
            logger.debug(
                "_load_pcd_points: numpy ASCII fallback failed for %s: %s",
                pcd_path,
                exc,
            )

        return None

    @staticmethod
    def _load_binary_pcd_points(fh: Any, header: dict[str, list[str]]) -> np.ndarray | None:
        """Load binary PCD XYZ points for MapService-generated map snapshots."""
        fields = [field.lower() for field in header.get("FIELDS", [])]
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = [value.upper() for value in header.get("TYPE", [])]
        counts = [int(value) for value in header.get("COUNT", [])] or [1] * len(fields)
        if not fields or len(sizes) != len(fields) or len(types) != len(fields):
            return None
        if len(counts) != len(fields):
            return None
        if any(size != 4 for size in sizes) or any(kind != "F" for kind in types):
            return None

        scalar_offsets: dict[str, int] = {}
        scalar_count = 0
        for name, count in zip(fields, counts, strict=False):
            if count <= 0:
                return None
            if count == 1:
                scalar_offsets[name] = scalar_count
            scalar_count += count
        xyz_offsets = [scalar_offsets.get(axis, -1) for axis in ("x", "y", "z")]
        if any(offset < 0 for offset in xyz_offsets):
            return None

        declared_points = 0
        if header.get("POINTS"):
            declared_points = int(header["POINTS"][0])
        elif header.get("WIDTH") and header.get("HEIGHT"):
            declared_points = int(header["WIDTH"][0]) * int(header["HEIGHT"][0])
        payload = fh.read()
        if scalar_count <= 0:
            return None
        available_points = len(payload) // (scalar_count * 4)
        point_count = declared_points if declared_points > 0 else available_points
        point_count = min(point_count, available_points)
        if point_count <= 0:
            return np.empty((0, 3), dtype=np.float32)
        values = np.frombuffer(
            payload[: point_count * scalar_count * 4],
            dtype="<f4",
        ).reshape(point_count, scalar_count)
        return values[:, xyz_offsets].astype(np.float32, copy=True)
