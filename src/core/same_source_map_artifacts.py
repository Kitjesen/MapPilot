"""Same-source map artifact writer shared by simulation and map services.

The writer persists a point cloud and optional PCT tomogram with enough
provenance metadata to prove both artifacts came from the same live mapping
source. Simulation gates may call this module, but the artifact contract lives
under ``src`` so product map/relocalization/PCT flows can reuse it.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from core.runtime_interface import (
    ARTIFACT_FORMATS,
    TOPICS,
    normalize_frame_id,
    runtime_topic_allowed_frame_ids,
    topic_default_frame_id,
)

_SAVED_MAP_ARTIFACT_FRAMES = runtime_topic_allowed_frame_ids(None)[
    TOPICS.saved_map_cloud
]
_SAVED_MAP_DEFAULT_FRAME_ID = topic_default_frame_id(TOPICS.saved_map_cloud)


def normalize_saved_map_frame_id(frame_id: str | None) -> str:
    """Normalize a saved-map artifact frame, defaulting to the contract frame."""

    return normalize_frame_id(frame_id) or _SAVED_MAP_DEFAULT_FRAME_ID


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_saved_map_metadata(
    *,
    source_profile: str,
    data_source: str,
    slam_source: str,
    localization_source: str,
    mapping_source: str,
    frame_id: str,
    artifacts: Mapping[str, Mapping[str, Any]],
    extra_metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build the canonical metadata.json payload for saved map artifacts."""

    normalized_frame_id = normalize_saved_map_frame_id(frame_id)
    artifact_payload: dict[str, dict[str, Any]] = {}
    for name, entry in artifacts.items():
        payload = dict(entry)
        if "frame_id" in payload:
            artifact_frame_id = normalize_frame_id(str(payload.get("frame_id") or ""))
            if artifact_frame_id is not None:
                payload["frame_id"] = artifact_frame_id
        artifact_payload[name] = payload

    metadata: dict[str, Any] = {
        "schema_version": "lingtu.saved_map_artifacts.v1",
        "source_profile": source_profile,
        "data_source": data_source,
        "slam_source": slam_source,
        "localization_source": localization_source,
        "mapping_source": mapping_source,
        "frame_id": normalized_frame_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "artifacts": artifact_payload,
    }
    if extra_metadata:
        for key, value in extra_metadata.items():
            if key != "artifacts":
                metadata[key] = value
    return metadata


def validate_same_source_map_metadata(
    metadata: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate saved-map metadata against the runtime artifact contract."""

    blockers: list[str] = []
    _require_fields(
        blockers,
        "metadata",
        metadata,
        ARTIFACT_FORMATS["metadata"].required_fields,
    )

    frame_id = normalize_frame_id(str(metadata.get("frame_id") or ""))
    if frame_id not in _SAVED_MAP_ARTIFACT_FRAMES:
        allowed = ", ".join(_SAVED_MAP_ARTIFACT_FRAMES)
        blockers.append(f"metadata.frame_id {frame_id!r} is not one of {allowed}")

    artifacts = metadata.get("artifacts")
    if not isinstance(artifacts, Mapping):
        blockers.append("metadata.artifacts is not a mapping")
        return {
            "schema_version": "lingtu.saved_map_artifacts.validation.v1",
            "ok": False,
            "blockers": blockers,
        }

    map_pcd = artifacts.get("map_pcd")
    if not isinstance(map_pcd, Mapping):
        blockers.append("metadata.artifacts.map_pcd missing")
        pcd_sha = ""
    else:
        _check_artifact_entry(blockers, "map_pcd", map_pcd, metadata)
        pcd_sha = str(map_pcd.get("sha256") or "")
        if not pcd_sha:
            blockers.append("metadata.artifacts.map_pcd.sha256 missing")
        if int(map_pcd.get("point_count") or 0) <= 0:
            blockers.append("metadata.artifacts.map_pcd.point_count is not positive")

    for name in ("tomogram", "occupancy_grid"):
        entry = artifacts.get(name)
        if entry is None:
            continue
        if not isinstance(entry, Mapping):
            blockers.append(f"metadata.artifacts.{name} is not a mapping")
            continue
        _check_artifact_entry(blockers, name, entry, metadata)
        if str(entry.get("source_map_sha256") or "") != pcd_sha:
            blockers.append(
                f"metadata.artifacts.{name}.source_map_sha256 does not match map_pcd.sha256"
            )

    return {
        "schema_version": "lingtu.saved_map_artifacts.validation.v1",
        "ok": not blockers,
        "blockers": blockers,
    }


def validate_saved_map_artifact_dir(
    map_dir: Path | str,
    *,
    require_tomogram: bool = False,
    require_occupancy: bool = False,
    expected_data_source: str | None = None,
    expected_source_profile: str | None = None,
    expected_frame_id: str | None = None,
) -> dict[str, Any]:
    """Validate a saved map directory against metadata and on-disk files."""

    root = Path(map_dir)
    metadata_path = root / "metadata.json"
    blockers: list[str] = []
    artifacts_report: dict[str, dict[str, Any]] = {}
    checked_required_artifacts = ["map_pcd"]
    if require_tomogram:
        checked_required_artifacts.append("tomogram")
    if require_occupancy:
        checked_required_artifacts.append("occupancy_grid")
    payload: dict[str, Any] = {
        "schema_version": "lingtu.saved_map_artifacts.gate.v1",
        "ok": False,
        "map_dir": str(root),
        "checked_artifact_formats": list(ARTIFACT_FORMATS),
        "checked_required_artifacts": checked_required_artifacts,
        "checked_allowed_frame_ids": list(_SAVED_MAP_ARTIFACT_FRAMES),
        "checked_frame_id": None,
        "checked_expected": {
            "data_source": expected_data_source,
            "source_profile": expected_source_profile,
            "frame_id": normalize_frame_id(expected_frame_id),
        },
        "checked_source_fields": [
            "source_profile",
            "data_source",
            "slam_source",
            "localization_source",
            "mapping_source",
        ],
        "metadata": {
            "path": str(metadata_path),
            "exists": metadata_path.exists(),
        },
        "metadata_validation": {
            "schema_version": "lingtu.saved_map_artifacts.validation.v1",
            "ok": False,
            "blockers": [],
        },
        "artifacts": artifacts_report,
        "blockers": blockers,
    }

    if not root.is_dir():
        blockers.append("map directory missing")
        return payload
    if not metadata_path.exists():
        blockers.append("metadata.json missing")
        return payload

    try:
        loaded = json.loads(metadata_path.read_text(encoding="utf-8"))
    except Exception as exc:
        blockers.append(f"metadata.json unreadable: {type(exc).__name__}: {exc}")
        return payload
    if not isinstance(loaded, Mapping):
        blockers.append("metadata.json is not a JSON object")
        return payload

    metadata_validation = validate_same_source_map_metadata(loaded)
    payload["metadata_validation"] = metadata_validation
    blockers.extend(str(item) for item in metadata_validation["blockers"])
    payload["checked_frame_id"] = normalize_frame_id(str(loaded.get("frame_id") or ""))

    if expected_data_source and loaded.get("data_source") != expected_data_source:
        blockers.append("metadata.data_source does not match expected data source")
    if expected_source_profile and loaded.get("source_profile") != expected_source_profile:
        blockers.append("metadata.source_profile does not match expected source profile")
    if expected_frame_id and normalize_frame_id(str(loaded.get("frame_id") or "")) != (
        normalize_frame_id(expected_frame_id)
    ):
        blockers.append("metadata.frame_id does not match expected frame_id")

    artifacts = loaded.get("artifacts")
    if not isinstance(artifacts, Mapping):
        payload["ok"] = False
        return payload

    map_sha = _validate_artifact_file(root, artifacts, artifacts_report, "map_pcd")
    if not map_sha:
        blockers.append(_artifact_file_blocker(artifacts_report, "map_pcd"))

    for name, required, missing_msg in (
        ("tomogram", require_tomogram, "tomogram required but missing"),
        ("occupancy_grid", require_occupancy, "occupancy_grid required but missing"),
    ):
        if name not in artifacts:
            if required:
                blockers.append(missing_msg)
            continue
        derived_sha = _validate_artifact_file(root, artifacts, artifacts_report, name)
        if not derived_sha:
            blockers.append(_artifact_file_blocker(artifacts_report, name))
            continue
        entry = artifacts.get(name)
        if isinstance(entry, Mapping) and str(entry.get("source_map_sha256") or "") != map_sha:
            blockers.append(f"{name} source_map_sha256 does not match current map_pcd sha256")

    payload["ok"] = not blockers
    return payload


def _require_fields(
    blockers: list[str],
    prefix: str,
    values: Mapping[str, Any],
    required: tuple[str, ...],
) -> None:
    for field in required:
        if _missing_value(values.get(field)):
            blockers.append(f"{prefix}.{field} missing")


def _check_artifact_entry(
    blockers: list[str],
    name: str,
    entry: Mapping[str, Any],
    metadata: Mapping[str, Any],
) -> None:
    _require_fields(
        blockers,
        f"metadata.artifacts.{name}",
        entry,
        ARTIFACT_FORMATS[name].required_metadata,
    )
    for key in ("source_profile", "data_source", "frame_id"):
        if key == "frame_id":
            entry_value = normalize_frame_id(str(entry.get(key) or ""))
            metadata_value = normalize_frame_id(str(metadata.get(key) or ""))
        else:
            entry_value = str(entry.get(key) or "")
            metadata_value = str(metadata.get(key) or "")
        if entry_value != metadata_value:
            blockers.append(f"metadata.artifacts.{name}.{key} does not match metadata.{key}")


def _missing_value(value: Any) -> bool:
    if value is None:
        return True
    if isinstance(value, str):
        return not value.strip()
    if isinstance(value, (list, tuple, dict)):
        return len(value) == 0
    return False


def _validate_artifact_file(
    root: Path,
    artifacts: Mapping[str, Any],
    report: dict[str, dict[str, Any]],
    name: str,
) -> str:
    entry = artifacts.get(name)
    if not isinstance(entry, Mapping):
        return ""
    path = _artifact_file_path(root, name, entry)
    declared_sha = str(entry.get("sha256") or "")
    exists = path.is_file()
    actual_sha = sha256_file(path) if exists else ""
    sha_ok = bool(exists and declared_sha and actual_sha == declared_sha)
    report[name] = {
        "path": str(path),
        "exists": exists,
        "declared_sha256": declared_sha,
        "actual_sha256": actual_sha,
        "sha256_ok": sha_ok,
    }
    return actual_sha if sha_ok else ""


def _artifact_file_path(root: Path, name: str, entry: Mapping[str, Any]) -> Path:
    path_value = str(entry.get("path") or "")
    if path_value:
        path = Path(path_value)
        if path.is_absolute():
            return path
        return root / path
    return root / ARTIFACT_FORMATS[name].path


def _artifact_file_blocker(report: Mapping[str, Mapping[str, Any]], name: str) -> str:
    entry = report.get(name) or {}
    if entry.get("exists") is not True:
        return f"{name} file missing"
    if not entry.get("declared_sha256"):
        return f"{name} declared sha256 missing"
    return f"{name} sha256 does not match file"


def write_ascii_pcd(path: Path, points: np.ndarray) -> int:
    """Write finite XYZ points to a portable ASCII PCD artifact."""
    path.parent.mkdir(parents=True, exist_ok=True)
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"expected Nx3 points, got shape={pts.shape}")
    pts = pts[:, :3]
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]

    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {len(pts)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(pts)}",
            "DATA ascii",
        ]
    )
    with path.open("w", encoding="ascii") as fh:
        fh.write(header)
        fh.write("\n")
        for x, y, z in pts:
            fh.write(f"{float(x):.5f} {float(y):.5f} {float(z):.5f}\n")
    return int(len(pts))


def add_points_to_voxel_store(
    store: dict[tuple[int, int, int], tuple[float, float, float]],
    points: np.ndarray,
    *,
    voxel_size: float,
    max_points: int,
) -> int:
    """Add a bounded finite XYZ point sample into an in-memory voxel store."""
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3 or pts.shape[0] == 0:
        return 0
    if len(store) >= int(max_points):
        return 0
    inv = 1.0 / max(float(voxel_size), 1.0e-6)
    added = 0
    for x, y, z in pts[:, :3]:
        if not (
            math.isfinite(float(x))
            and math.isfinite(float(y))
            and math.isfinite(float(z))
        ):
            continue
        cell = (
            int(math.floor(float(x) * inv)),
            int(math.floor(float(y) * inv)),
            int(math.floor(float(z) * inv)),
        )
        if cell in store:
            continue
        store[cell] = (float(x), float(y), float(z))
        added += 1
        if len(store) >= int(max_points):
            break
    return added


def point_bounds(points: np.ndarray) -> dict[str, list[float]] | None:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] < 3:
        return None
    pts = pts[:, :3]
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    if pts.shape[0] == 0:
        return None
    return {
        "min": pts.min(axis=0).astype(float).tolist(),
        "max": pts.max(axis=0).astype(float).tolist(),
    }


def build_tomogram_artifact(
    *,
    pcd_path: Path,
    tomogram_path: Path,
    resolution: float,
    slice_dh: float,
    ground_h: float,
) -> dict[str, Any]:
    # Lazy: global_planning is a nav/semantic-layer package.  This import is
    # inside the build_tomogram_artifact() function so core/ modules remain
    # importable without triggering global_planning dependencies at module level.
    from global_planning.pct_planner.tomography.scripts.build_tomogram import (
        build_tomogram_from_pcd,
    )

    data = build_tomogram_from_pcd(
        str(pcd_path),
        str(tomogram_path),
        resolution=float(resolution),
        slice_dh=float(slice_dh),
        ground_h=float(ground_h),
    )
    arr = np.asarray(data.get("data"))
    return {
        "path": str(tomogram_path),
        "exists": tomogram_path.exists(),
        "sha256": sha256_file(tomogram_path) if tomogram_path.exists() else "",
        "input_pcd": str(pcd_path),
        "input_pcd_sha256": sha256_file(pcd_path) if pcd_path.exists() else "",
        "same_source_input": True,
        "shape": list(arr.shape),
        "resolution": float(data.get("resolution", resolution)),
        "center": np.asarray(data.get("center", [0.0, 0.0]), dtype=float).tolist(),
        "slice_h0": float(data.get("slice_h0", 0.0)),
        "slice_dh": float(data.get("slice_dh", slice_dh)),
    }


def write_same_source_map_artifacts(
    *,
    artifact_dir: Path,
    points: np.ndarray,
    frame_id: str,
    world: Path,
    source_topics: tuple[str, ...],
    mapping_input_path: str,
    build_tomogram: bool,
    tomogram_resolution: float,
    tomogram_slice_dh: float,
    tomogram_ground_h: float,
    map_artifact_max_span_m: float,
    tomogram_max_cells: int,
    source: str = "same_source_map_artifact_writer",
    extra_metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Persist same-run map artifacts for saved-map/PCT follow-up gates."""
    artifact_dir.mkdir(parents=True, exist_ok=True)
    frame_id = normalize_saved_map_frame_id(frame_id)
    report: dict[str, Any] = {
        "ok": False,
        "artifact_dir": str(artifact_dir),
        "blockers": [],
        "source_contract": {
            "map_save_source": source,
            "same_source_pcd": False,
            "same_source_tomogram": False,
            "source_topics": list(source_topics),
            "mapping_input_path": mapping_input_path,
            "frame_id": frame_id,
            "world": str(world),
        },
        "assets": {},
    }
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[0] <= 0 or pts.shape[1] < 3:
        report["blockers"].append(f"no accumulated {TOPICS.map_cloud} points for map.pcd")
        return report
    pts = pts[:, :3]
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    if pts.shape[0] <= 0:
        report["blockers"].append(f"accumulated {TOPICS.map_cloud} points are not finite")
        return report

    bounds = point_bounds(pts)
    span = None
    if bounds is not None:
        min_xyz = np.asarray(bounds["min"], dtype=np.float64)
        max_xyz = np.asarray(bounds["max"], dtype=np.float64)
        span = (max_xyz - min_xyz).astype(float).tolist()
        max_span = max(float(value) for value in span)
        if max_span > float(map_artifact_max_span_m):
            report["blockers"].append(
                "map bounds span exceeds limit "
                f"(span={span}, max_allowed={float(map_artifact_max_span_m):.3f}m)"
            )

    pcd_path = artifact_dir / "map.pcd"
    metadata_path = artifact_dir / "metadata.json"
    point_count = write_ascii_pcd(pcd_path, pts)
    pcd_sha = sha256_file(pcd_path)
    extra = dict(extra_metadata or {})
    source_profile = str(extra.get("source_profile") or extra.get("profile") or source)
    data_source = str(extra.get("data_source") or extra.get("nav_data_source") or source)
    slam_source = str(extra.get("slam_source") or "fastlio2")
    localization_source = str(extra.get("localization_source") or slam_source)
    mapping_source = str(extra.get("mapping_source") or mapping_input_path)
    artifact_entries: dict[str, dict[str, Any]] = {
        "map_pcd": {
            "path": str(pcd_path),
            "sha256": pcd_sha,
            "source_profile": source_profile,
            "data_source": data_source,
            "slam_source": slam_source,
            "frame_id": frame_id,
            "point_count": int(point_count),
        }
    }
    report["assets"]["map_pcd"] = {
        "path": str(pcd_path),
        "exists": pcd_path.exists(),
        "point_count": int(point_count),
        "sha256": pcd_sha,
        "frame_id": frame_id,
        "bounds": bounds,
        "span_m": span,
    }
    report["assets"]["metadata"] = {
        "path": str(metadata_path),
        "exists": metadata_path.exists(),
        "sha256": "",
        "required_fields": [
            "source_profile",
            "data_source",
            "slam_source",
            "localization_source",
            "mapping_source",
            "frame_id",
            "created_at",
            "artifacts",
        ],
    }
    report["source_contract"]["same_source_pcd"] = bool(
        pcd_path.exists() and point_count > 0 and bool(pcd_sha)
    )

    if build_tomogram and not report["blockers"]:
        if bounds is not None and span is not None:
            nx = max(1, int(math.ceil(float(span[0]) / max(float(tomogram_resolution), 1e-6))))
            ny = max(1, int(math.ceil(float(span[1]) / max(float(tomogram_resolution), 1e-6))))
            nz = max(1, int(math.ceil(float(span[2]) / max(float(tomogram_slice_dh), 1e-6))))
            cell_count = int(nx) * int(ny) * int(nz)
            report["assets"]["tomogram_estimate"] = {
                "cells": cell_count,
                "shape_xyz": [nx, ny, nz],
                "max_allowed_cells": int(tomogram_max_cells),
            }
            if cell_count > int(tomogram_max_cells):
                report["blockers"].append(
                    "tomogram cell estimate exceeds limit "
                    f"(cells={cell_count}, max_allowed={int(tomogram_max_cells)})"
                )
        if report["blockers"]:
            report["source_contract"]["same_source_tomogram"] = False

    if build_tomogram and not report["blockers"]:
        tomogram_path = artifact_dir / "tomogram.pickle"
        try:
            tomogram = build_tomogram_artifact(
                pcd_path=pcd_path,
                tomogram_path=tomogram_path,
                resolution=tomogram_resolution,
                slice_dh=tomogram_slice_dh,
                ground_h=tomogram_ground_h,
            )
            report["assets"]["tomogram"] = tomogram
            artifact_entries["tomogram"] = {
                "path": str(tomogram_path),
                "sha256": str(tomogram.get("sha256") or ""),
                "source_map_sha256": str(tomogram.get("input_pcd_sha256") or ""),
                "source_profile": source_profile,
                "data_source": data_source,
                "frame_id": frame_id,
                "shape": list(tomogram.get("shape") or []),
            }
            report["source_contract"]["same_source_tomogram"] = bool(
                tomogram.get("exists")
                and tomogram.get("input_pcd_sha256") == pcd_sha
                and bool(tomogram.get("sha256"))
            )
            if not report["source_contract"]["same_source_tomogram"]:
                report["blockers"].append("tomogram was not built from the saved map.pcd")
        except Exception as exc:
            report["blockers"].append(
                f"tomogram build failed: {type(exc).__name__}: {exc}"
            )

    metadata = build_saved_map_metadata(
        source_profile=source_profile,
        data_source=data_source,
        slam_source=slam_source,
        localization_source=localization_source,
        mapping_source=mapping_source,
        frame_id=frame_id,
        artifacts=artifact_entries,
        extra_metadata={
            **extra,
            "source": source,
            "source_topics": list(source_topics),
            "mapping_input_path": mapping_input_path,
            "world": str(world),
            "point_count": int(point_count),
            "bounds": bounds,
            "span_m": span,
            "max_allowed_span_m": float(map_artifact_max_span_m),
            "pcd": str(pcd_path),
            "pcd_sha256": pcd_sha,
            "generated_unix_s": time.time(),
        },
    )
    metadata_validation = validate_same_source_map_metadata(metadata)
    if not metadata_validation["ok"]:
        report["blockers"].extend(
            f"metadata contract: {blocker}"
            for blocker in metadata_validation["blockers"]
        )
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    metadata_sha = sha256_file(metadata_path)
    report["assets"]["metadata"].update(
        {
            "path": str(metadata_path),
            "exists": metadata_path.exists(),
            "sha256": metadata_sha,
            "validation": metadata_validation,
        }
    )

    report["ok"] = (
        bool(report["source_contract"]["same_source_pcd"])
        and (not build_tomogram or bool(report["source_contract"]["same_source_tomogram"]))
        and not report["blockers"]
    )
    return report
