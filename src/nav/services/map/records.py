"""File-backed MapRecord model for MapService."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

from runtime.same_source_map_artifacts import sha256_file

MAP_RECORD_SCHEMA = "map.record"
MAP_TYPES_SCHEMA = "map.types"
MAP_BUNDLE_SCHEMA = "map.bundle"
MAP_RECORD_FILENAME = "map_record.json"

MAP_STATES = (
    "EMPTY",
    "CREATED",
    "BUILDING",
    "READY",
    "ACTIVE",
    "STALE",
    "FAILED",
    "RETIRED",
)

MAP_CLASSES = (
    "saved_point_cloud",
    "static_2d_occupancy",
    "portable_2d_occupancy",
    "global_2_5d_terrain",
    "global_3d_occupancy",
    "esdf",
    "semantic",
    "map_metadata",
)

ARTIFACT_SPECS: dict[str, dict[str, str]] = {
    "map_pcd": {
        "type": "POINTCLOUD",
        "filename": "map.pcd",
        "map_class": "saved_point_cloud",
        "capability": "source_pointcloud",
        "role": "raw_slam_map_source",
    },
    "occupancy_grid": {
        "type": "OCCUPANCY_2D",
        "filename": "occupancy.npz",
        "map_class": "static_2d_occupancy",
        "capability": "path_planning_2d",
        "role": "static_grid_helper",
    },
    "tomogram": {
        "type": "TOMOGRAM_2_5D",
        "filename": "tomogram.pickle",
        "map_class": "global_2_5d_terrain",
        "capability": "terrain_reasoning",
        "role": "pct_astar_compatibility",
    },
    "octomap": {
        "type": "OCTOMAP_3D",
        "filename": "octomap.ot",
        "map_class": "global_3d_occupancy",
        "capability": "navigation_safety_3d",
        "role": "octoplanner3d_global_planning",
    },
    "map_yaml": {
        "type": "OCCUPANCY_YAML",
        "filename": "map.yaml",
        "map_class": "portable_2d_occupancy",
        "capability": "occupancy_yaml",
        "role": "occupancy_yaml_compatibility",
    },
    "map_pgm": {
        "type": "OCCUPANCY_PGM",
        "filename": "map.pgm",
        "map_class": "portable_2d_occupancy",
        "capability": "occupancy_image",
        "role": "occupancy_image_compatibility",
    },
    "metadata": {
        "type": "MAP_METADATA",
        "filename": "metadata.json",
        "map_class": "map_metadata",
        "capability": "artifact_provenance",
        "role": "artifact_provenance",
    },
    "esdf": {
        "type": "ESDF",
        "filename": "esdf.npz",
        "map_class": "esdf",
        "capability": "trajectory_optimization",
        "role": "future_local_planner_constraint",
    },
    "semantic": {
        "type": "SEMANTIC",
        "filename": "semantic.json",
        "map_class": "semantic",
        "capability": "semantic_query",
        "role": "semantic_query",
    },
}

ARTIFACT_ALIASES = {
    "occupancy": "occupancy_grid",
}

CAPABILITY_TO_ARTIFACT_TYPE = {
    spec["capability"]: spec["type"]
    for spec in ARTIFACT_SPECS.values()
}
CAPABILITY_TO_ARTIFACT_TYPE.update(
    {
        "path_planning": "OCCUPANCY_2D",
        "global_planning_3d": "OCTOMAP_3D",
        "global_planning_2_5d": "TOMOGRAM_2_5D",
    }
)


@dataclass(frozen=True)
class Artifact:
    type: str
    uri: str
    hash: str
    source_map_id: str
    generator: str
    build_config: dict[str, Any] = field(default_factory=dict)
    name: str = ""
    filename: str = ""
    map_class: str = ""
    capability: str = ""
    role: str = ""


@dataclass(frozen=True)
class MapHealth:
    localization_stability: float
    planning_success_rate: float
    collision_rate: float
    map_freshness: float
    overall_score: float
    active_allowed: bool
    blockers: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class MapRecord:
    schema_version: str
    map_id: str
    lineage_id: str
    version: int
    state: str
    scope: dict[str, Any]
    artifacts: list[Artifact]
    metadata: dict[str, Any]
    health: MapHealth
    lifecycle: dict[str, Any]
    capabilities: list[str]
    version_id: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def load_map_record(map_dir: Path) -> dict[str, Any] | None:
    path = map_dir / MAP_RECORD_FILENAME
    if not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


def write_map_record(map_dir: Path, record: MapRecord) -> Path:
    path = map_dir / MAP_RECORD_FILENAME
    path.write_text(
        json.dumps(record.to_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return path


def normalize_map_state(state: str | None) -> str:
    value = str(state or "").strip().upper()
    if not value:
        return "EMPTY"
    if value not in MAP_STATES:
        raise ValueError(f"unsupported map state: {state}")
    return value


def map_artifact_catalog(
    *,
    include_aliases: bool = False,
) -> dict[str, dict[str, str]]:
    """Return the canonical file-backed artifact catalog."""

    catalog = {
        name: {
            "filename": spec["filename"],
            "map_class": spec["map_class"],
            "role": spec["role"],
            "type": spec["type"],
            "capability": spec["capability"],
        }
        for name, spec in ARTIFACT_SPECS.items()
    }
    if include_aliases:
        for alias, target in ARTIFACT_ALIASES.items():
            catalog[alias] = {**catalog[target], "alias_for": target}
    return catalog


def map_type_catalog() -> dict[str, Any]:
    """Return JSON-ready map type metadata for docs, Gateway, and tests."""

    return {
        "schema_version": MAP_TYPES_SCHEMA,
        "record_schema_version": MAP_RECORD_SCHEMA,
        "bundle_schema_version": MAP_BUNDLE_SCHEMA,
        "states": list(MAP_STATES),
        "classes": list(MAP_CLASSES),
        "artifacts": map_artifact_catalog(),
        "aliases": dict(ARTIFACT_ALIASES),
        "capabilities": {
            capability: artifact_type
            for capability, artifact_type in sorted(CAPABILITY_TO_ARTIFACT_TYPE.items())
        },
    }


def build_map_record(
    map_dir: Path,
    *,
    map_id: str,
    metadata: Mapping[str, Any] | None = None,
    gate: Mapping[str, Any] | None = None,
    state: str = "READY",
    version: int | None = None,
) -> MapRecord:
    now = datetime.now(timezone.utc).isoformat()
    loaded = load_map_record(map_dir) or {}
    metadata_dict = dict(metadata or _read_json(map_dir / "metadata.json") or {})
    lineage_id = str(loaded.get("lineage_id") or metadata_dict.get("lineage_id") or map_id)
    record_version = int(version or loaded.get("version") or 1)
    normalized_state = normalize_map_state(state)
    artifacts = _artifacts_from_dir(map_dir, map_id=map_id, metadata=metadata_dict)
    gate_dict = dict(gate or {})
    health = _health_from_gate(gate_dict, artifacts, bool(metadata_dict))
    capabilities = sorted(
        {
            spec["capability"]
            for spec in ARTIFACT_SPECS.values()
            if any(artifact.type == spec["type"] for artifact in artifacts)
        }
    )
    lifecycle = dict(loaded.get("lifecycle") or {})
    lifecycle.setdefault("created_at", metadata_dict.get("created_at") or now)
    lifecycle["updated_at"] = now
    lifecycle["state"] = normalized_state
    version_id = f"{lineage_id}:v{record_version}"
    return MapRecord(
        schema_version=MAP_RECORD_SCHEMA,
        map_id=map_id,
        lineage_id=lineage_id,
        version=record_version,
        state=normalized_state,
        scope={
            "map_dir": str(map_dir),
            "frame_id": metadata_dict.get("frame_id") or gate_dict.get("checked_frame_id"),
        },
        artifacts=artifacts,
        metadata={
            "source_profile": metadata_dict.get("source_profile"),
            "data_source": metadata_dict.get("data_source"),
            "slam_source": metadata_dict.get("slam_source"),
            "localization_source": metadata_dict.get("localization_source"),
            "mapping_source": metadata_dict.get("mapping_source"),
        },
        health=health,
        lifecycle=lifecycle,
        capabilities=capabilities,
        version_id=version_id,
    )


def artifact_for_capability(
    record: Mapping[str, Any],
    capability: str,
) -> dict[str, Any] | None:
    artifact_type = CAPABILITY_TO_ARTIFACT_TYPE.get(str(capability))
    if not artifact_type:
        return None
    for artifact in record.get("artifacts") or []:
        if isinstance(artifact, Mapping) and artifact.get("type") == artifact_type:
            return dict(artifact)
    return None


def _artifacts_from_dir(
    map_dir: Path,
    *,
    map_id: str,
    metadata: Mapping[str, Any],
) -> list[Artifact]:
    metadata_artifacts = metadata.get("artifacts")
    if not isinstance(metadata_artifacts, Mapping):
        metadata_artifacts = {}
    generator = str(metadata.get("mapping_source") or "map_service")
    artifacts: list[Artifact] = []
    for metadata_name, spec in ARTIFACT_SPECS.items():
        entry = metadata_artifacts.get(metadata_name)
        if isinstance(entry, Mapping):
            try:
                path = _artifact_path(map_dir, str(entry.get("path") or spec["filename"]))
            except ValueError:
                continue
            digest = str(entry.get("sha256") or "")
            build_config = {
                key: value
                for key, value in entry.items()
                if key not in {"path", "sha256", "source_map_sha256"}
            }
        else:
            path = map_dir / spec["filename"]
            digest = ""
            build_config = {}
        if not path.is_file():
            continue
        if not digest:
            digest = sha256_file(path)
        artifacts.append(
            Artifact(
                type=spec["type"],
                uri=str(path),
                hash=digest,
                source_map_id=map_id,
                generator=generator,
                build_config=build_config,
                name=metadata_name,
                filename=spec["filename"],
                map_class=spec["map_class"],
                capability=spec["capability"],
                role=spec["role"],
            )
        )
    return artifacts


def _health_from_gate(
    gate: Mapping[str, Any],
    artifacts: list[Artifact],
    has_metadata: bool,
) -> MapHealth:
    blockers = [str(item) for item in gate.get("blockers") or [] if str(item)]
    has_source = any(item.type == "POINTCLOUD" for item in artifacts)
    has_planning = any(
        item.type in {"OCCUPANCY_2D", "TOMOGRAM_2_5D", "OCTOMAP_3D", "ESDF"}
        for item in artifacts
    )
    gate_ok = gate.get("ok") is True if gate else bool(has_source and has_planning)
    localization_stability = 1.0 if has_source else 0.0
    planning_success_rate = 1.0 if has_planning else 0.0
    collision_rate = 0.0
    map_freshness = 1.0 if has_metadata else (0.5 if artifacts else 0.0)
    overall = (
        localization_stability
        + planning_success_rate
        + (1.0 - collision_rate)
        + map_freshness
    ) / 4.0
    if not gate_ok:
        overall = min(overall, 0.49)
    return MapHealth(
        localization_stability=localization_stability,
        planning_success_rate=planning_success_rate,
        collision_rate=collision_rate,
        map_freshness=map_freshness,
        overall_score=round(float(overall), 4),
        active_allowed=bool(gate_ok and has_planning),
        blockers=blockers,
    )


def _artifact_path(map_dir: Path, value: str) -> Path:
    path = Path(value)
    if path.is_absolute():
        raise ValueError(f"artifact path must be relative: {value}")
    root = map_dir.resolve()
    candidate = (map_dir / path).resolve()
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"artifact path escapes map directory: {value}") from exc
    return candidate


def _read_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None
