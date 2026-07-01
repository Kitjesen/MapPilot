"""Saved-map artifact builder for OctoPlanner3D runtime maps.

This module owns the bounded map-save/build conversion step from LingTu saved-map
source data (``map.pcd``) to the OctoPlanner3D runtime artifact
(``octomap.bt``).  It intentionally does not run conversion during each
navigation plan call.
"""

from __future__ import annotations

import json
import os
import shlex
import subprocess
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.same_source_map_artifacts import (
    build_saved_map_metadata,
    normalize_saved_map_frame_id,
    sha256_file,
    validate_same_source_map_metadata,
)

BUILDER_VERSION = "0.1.0"
SUPPORTED_BUILD_MODES = ("raycast", "occupied_only", "external_pcl_converter")
IMPLEMENTED_BUILD_MODES = ("external_pcl_converter",)
_CONVERTER_ENV_VARS = ("LINGTU_MAP_ARTIFACT_CONVERTER", "LINGTU_OCTOMAP_CONVERTER")


@dataclass(slots=True)
class MapArtifactBuilderConfig:
    """Configuration for saved-map artifact generation."""

    converter_command: Sequence[str] | str | None = None
    use_env_converter: bool = True
    build_mode: str = "external_pcl_converter"
    resolution: float = 0.20
    frame_id: str | None = None
    source_profile: str | None = None
    data_source: str | None = None
    slam_source: str | None = None
    localization_source: str | None = None
    mapping_source: str = "map_artifact_builder"
    timeout_sec: float = 60.0
    builder_name: str = "LingTu MapArtifactBuilder"
    builder_version: str = BUILDER_VERSION

    def resolved_converter_command(self) -> Sequence[str] | str | None:
        """Return the configured converter command, including env fallback."""

        if self.converter_command:
            return self.converter_command
        if not self.use_env_converter:
            return None
        for env_name in _CONVERTER_ENV_VARS:
            value = os.environ.get(env_name)
            if value:
                return value
        return None


@dataclass(slots=True)
class MapArtifactBuildReport:
    """Structured result for one saved-map artifact build attempt."""

    ok: bool
    status: str
    map_dir: str
    pcd_path: str
    octomap_path: str
    metadata_path: str
    build_mode: str
    resolution: float
    frame_id: str
    reused: bool = False
    blockers: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    source_hashes: dict[str, Any] = field(default_factory=dict)
    artifacts: dict[str, Any] = field(default_factory=dict)
    converter: dict[str, Any] = field(default_factory=dict)
    metadata_validation: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable report dictionary."""

        payload = asdict(self)
        payload["success"] = self.ok
        return payload


class MapArtifactBuilder:
    """Build stable saved-map artifacts once during map save/build."""

    def __init__(self, config: MapArtifactBuilderConfig | None = None) -> None:
        self.config = config or MapArtifactBuilderConfig()

    def build_for_saved_map(self, map_dir: Path | str) -> MapArtifactBuildReport:
        """Build or reuse ``octomap.bt`` and write ``metadata.json`` for *map_dir*.

        The first implementation slice supports an external PCL/OctoMap
        converter command only.  If no converter is configured and no reusable
        artifact exists, the report fails clearly instead of producing a fake
        OctoMap file.
        """

        root = Path(map_dir)
        pcd_path = root / "map.pcd"
        octomap_path = root / "octomap.bt"
        metadata_path = root / "metadata.json"
        existing_metadata = _read_json_object(metadata_path)
        frame_id = self._resolve_frame_id(existing_metadata)

        report = MapArtifactBuildReport(
            ok=False,
            status="not_started",
            map_dir=str(root),
            pcd_path=str(pcd_path),
            octomap_path=str(octomap_path),
            metadata_path=str(metadata_path),
            build_mode=self.config.build_mode,
            resolution=float(self.config.resolution),
            frame_id=frame_id,
        )

        mode = str(self.config.build_mode or "").strip()
        if mode not in SUPPORTED_BUILD_MODES:
            report.status = "unsupported_build_mode"
            report.blockers.append(
                f"unsupported build_mode={mode!r}; supported={list(SUPPORTED_BUILD_MODES)}"
            )
            return report
        if mode not in IMPLEMENTED_BUILD_MODES:
            report.status = "unimplemented_build_mode"
            report.blockers.append(
                "build_mode "
                f"{mode!r} is not implemented in this slice; "
                "use external_pcl_converter"
            )
            return report

        if not root.is_dir():
            report.status = "missing_map_dir"
            report.blockers.append(f"map directory missing: {root}")
            return report
        if not pcd_path.is_file():
            report.status = "missing_map_pcd"
            report.blockers.append(f"missing required source map.pcd at {pcd_path}")
            return report

        pcd_sha = sha256_file(pcd_path)
        source_hashes = _collect_source_hashes(root, pcd_sha)
        report.source_hashes = source_hashes

        reusable, reuse_reason = self._metadata_matches_reusable_octomap(
            metadata=existing_metadata,
            map_dir=root,
            pcd_sha=pcd_sha,
            octomap_path=octomap_path,
            frame_id=frame_id,
        )
        if reusable:
            octomap_sha = sha256_file(octomap_path)
            report.ok = True
            report.status = "reused"
            report.reused = True
            report.artifacts = {
                "map_pcd": {
                    "path": str(pcd_path),
                    "sha256": pcd_sha,
                    "point_count": _pcd_point_count(pcd_path),
                },
                "octomap": {
                    "path": str(octomap_path),
                    "sha256": octomap_sha,
                    "source_map_sha256": pcd_sha,
                },
            }
            return report
        if reuse_reason:
            report.warnings.append(f"existing octomap was not reusable: {reuse_reason}")

        converter_command = self.config.resolved_converter_command()
        if not converter_command:
            report.status = "missing_converter"
            report.blockers.append(
                "no external PCL/OctoMap converter configured; pass "
                "converter_command or set LINGTU_MAP_ARTIFACT_CONVERTER"
            )
            return report

        argv = self._converter_argv(converter_command, pcd_path, octomap_path, frame_id)
        report.converter["argv"] = argv
        try:
            completed = subprocess.run(
                argv,
                cwd=str(root),
                capture_output=True,
                text=True,
                timeout=float(self.config.timeout_sec),
                check=False,
            )
        except FileNotFoundError as exc:
            report.status = "converter_not_found"
            report.blockers.append(f"converter executable not found: {exc}")
            return report
        except subprocess.TimeoutExpired as exc:
            report.status = "converter_timeout"
            report.converter.update(
                {
                    "timeout_sec": float(self.config.timeout_sec),
                    "stdout": _decode_process_text(exc.stdout),
                    "stderr": _decode_process_text(exc.stderr),
                }
            )
            report.blockers.append(
                f"converter timed out after {float(self.config.timeout_sec):.3f}s"
            )
            return report

        report.converter.update(
            {
                "returncode": int(completed.returncode),
                "stdout": completed.stdout,
                "stderr": completed.stderr,
            }
        )
        if completed.returncode != 0:
            report.status = "converter_failed"
            detail = completed.stderr.strip() or completed.stdout.strip() or "no output"
            report.blockers.append(
                f"converter exited with code {completed.returncode}: {detail}"
            )
            return report
        if not octomap_path.is_file() or octomap_path.stat().st_size <= 0:
            report.status = "converter_missing_output"
            report.blockers.append(
                f"converter succeeded but did not write non-empty {octomap_path.name}"
            )
            return report

        octomap_sha = sha256_file(octomap_path)
        metadata = self._build_metadata(
            map_dir=root,
            pcd_path=pcd_path,
            octomap_path=octomap_path,
            pcd_sha=pcd_sha,
            octomap_sha=octomap_sha,
            source_hashes=source_hashes,
            frame_id=frame_id,
            existing_metadata=existing_metadata,
        )
        validation = validate_same_source_map_metadata(metadata)
        metadata_path.write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        report.metadata_validation = validation
        report.artifacts = {
            "map_pcd": dict(metadata["artifacts"]["map_pcd"]),
            "octomap": dict(metadata["artifacts"]["octomap"]),
        }
        if validation.get("ok") is not True:
            report.status = "metadata_invalid"
            report.blockers.extend(str(item) for item in validation.get("blockers", []))
            return report

        report.ok = True
        report.status = "built"
        return report

    def _resolve_frame_id(self, existing_metadata: Mapping[str, Any] | None) -> str:
        if self.config.frame_id:
            return normalize_saved_map_frame_id(self.config.frame_id)
        if isinstance(existing_metadata, Mapping) and existing_metadata.get("frame_id"):
            return normalize_saved_map_frame_id(str(existing_metadata.get("frame_id")))
        return topic_default_frame_id(TOPICS.saved_map_cloud)

    def _converter_argv(
        self,
        converter_command: Sequence[str] | str,
        pcd_path: Path,
        octomap_path: Path,
        frame_id: str,
    ) -> list[str]:
        if isinstance(converter_command, str):
            tokens = shlex.split(converter_command, posix=(os.name != "nt"))
        else:
            tokens = [str(part) for part in converter_command]

        replacements = {
            "input": str(pcd_path),
            "output": str(octomap_path),
            "resolution": f"{float(self.config.resolution):g}",
            "frame": frame_id,
            "map_dir": str(pcd_path.parent),
        }
        has_placeholder = any("{" in token and "}" in token for token in tokens)
        if has_placeholder:
            return [token.format(**replacements) for token in tokens]
        return [
            *tokens,
            "--input",
            str(pcd_path),
            "--output",
            str(octomap_path),
            "--resolution",
            f"{float(self.config.resolution):g}",
            "--frame",
            frame_id,
        ]

    def _metadata_matches_reusable_octomap(
        self,
        *,
        metadata: Mapping[str, Any] | None,
        map_dir: Path,
        pcd_sha: str,
        octomap_path: Path,
        frame_id: str,
    ) -> tuple[bool, str]:
        if not octomap_path.is_file():
            return False, "octomap.bt missing"
        if not isinstance(metadata, Mapping):
            return False, "metadata.json missing or unreadable"

        metadata_pcd_sha = _metadata_source_hash(metadata, "map_pcd")
        if metadata_pcd_sha != pcd_sha:
            return False, "metadata source hash does not match current map.pcd"

        if str(metadata.get("build_mode") or "") != self.config.build_mode:
            return False, "metadata build_mode differs"
        if not _float_equal(metadata.get("resolution"), self.config.resolution):
            return False, "metadata resolution differs"
        if normalize_saved_map_frame_id(str(metadata.get("frame_id") or "")) != frame_id:
            return False, "metadata frame_id differs"

        artifacts = metadata.get("artifacts")
        if not isinstance(artifacts, Mapping):
            return False, "metadata artifacts missing"
        octomap_entry = artifacts.get("octomap")
        if not isinstance(octomap_entry, Mapping):
            return False, "metadata artifacts.octomap missing"
        if str(octomap_entry.get("source_map_sha256") or "") != pcd_sha:
            return False, "octomap source_map_sha256 does not match map.pcd"
        declared_octomap_sha = str(octomap_entry.get("sha256") or "")
        if not declared_octomap_sha:
            return False, "octomap sha256 missing in metadata"
        actual_octomap_sha = sha256_file(octomap_path)
        if actual_octomap_sha != declared_octomap_sha:
            return False, "octomap sha256 does not match metadata"

        entry_path = _artifact_path(map_dir, octomap_entry, "octomap.bt")
        if entry_path.resolve() != octomap_path.resolve():
            return False, "metadata octomap path differs"
        return True, ""

    def _build_metadata(
        self,
        *,
        map_dir: Path,
        pcd_path: Path,
        octomap_path: Path,
        pcd_sha: str,
        octomap_sha: str,
        source_hashes: Mapping[str, Any],
        frame_id: str,
        existing_metadata: Mapping[str, Any] | None,
    ) -> dict[str, Any]:
        source_profile = _first_nonempty(
            self.config.source_profile,
            _mapping_str(existing_metadata, "source_profile"),
            os.environ.get("LINGTU_PROFILE"),
            "map_artifact_builder",
        )
        data_source = _first_nonempty(
            self.config.data_source,
            _mapping_str(existing_metadata, "data_source"),
            os.environ.get("LINGTU_RUNTIME_DATA_SOURCE"),
            source_profile,
        )
        slam_source = _first_nonempty(
            self.config.slam_source,
            _mapping_str(existing_metadata, "slam_source"),
            "unknown",
        )
        localization_source = _first_nonempty(
            self.config.localization_source,
            _mapping_str(existing_metadata, "localization_source"),
            slam_source,
        )

        artifacts: dict[str, dict[str, Any]] = {
            "map_pcd": {
                "path": _relative_path(map_dir, pcd_path),
                "sha256": pcd_sha,
                "source_profile": source_profile,
                "data_source": data_source,
                "slam_source": slam_source,
                "frame_id": frame_id,
                "point_count": _pcd_point_count(pcd_path),
            }
        }
        artifacts.update(
            _preserved_same_source_artifacts(
                map_dir=map_dir,
                metadata=existing_metadata,
                pcd_sha=pcd_sha,
                source_profile=source_profile,
                data_source=data_source,
                frame_id=frame_id,
            )
        )
        artifacts["octomap"] = {
            "path": _relative_path(map_dir, octomap_path),
            "sha256": octomap_sha,
            "source_map_sha256": pcd_sha,
            "source_profile": source_profile,
            "data_source": data_source,
            "frame_id": frame_id,
            "resolution": float(self.config.resolution),
            "build_mode": self.config.build_mode,
            "builder": {
                "name": self.config.builder_name,
                "version": self.config.builder_version,
            },
        }

        return build_saved_map_metadata(
            source_profile=source_profile,
            data_source=data_source,
            slam_source=slam_source,
            localization_source=localization_source,
            mapping_source=self.config.mapping_source,
            frame_id=frame_id,
            artifacts=artifacts,
            extra_metadata={
                "map_name": map_dir.name,
                "source": "map_artifact_builder",
                "build_mode": self.config.build_mode,
                "supported_build_modes": list(SUPPORTED_BUILD_MODES),
                "resolution": float(self.config.resolution),
                "frame": frame_id,
                "builder": {
                    "name": self.config.builder_name,
                    "version": self.config.builder_version,
                },
                "builder_version": self.config.builder_version,
                "source_hashes": dict(source_hashes),
                "octomap": {
                    "path": _relative_path(map_dir, octomap_path),
                    "sha256": octomap_sha,
                    "source_map_sha256": pcd_sha,
                },
            },
        )


def build_for_saved_map(
    map_dir: Path | str,
    **config_overrides: Any,
) -> MapArtifactBuildReport:
    """Convenience API for one saved-map artifact build."""

    return MapArtifactBuilder(
        MapArtifactBuilderConfig(**config_overrides)
    ).build_for_saved_map(map_dir)


def _read_json_object(path: Path) -> Mapping[str, Any] | None:
    if not path.is_file():
        return None
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return loaded if isinstance(loaded, Mapping) else None


def _collect_source_hashes(map_dir: Path, pcd_sha: str) -> dict[str, Any]:
    source_hashes: dict[str, Any] = {"map_pcd": pcd_sha}
    poses_path = map_dir / "poses.txt"
    if poses_path.is_file():
        source_hashes["poses_txt"] = sha256_file(poses_path)
    patches_dir = map_dir / "patches"
    if patches_dir.is_dir():
        patch_hashes = {
            _relative_path(map_dir, path): sha256_file(path)
            for path in sorted(patches_dir.glob("*.pcd"))
            if path.is_file()
        }
        if patch_hashes:
            source_hashes["patches"] = patch_hashes
    return source_hashes


def _metadata_source_hash(metadata: Mapping[str, Any], key: str) -> str:
    source_hashes = metadata.get("source_hashes")
    if isinstance(source_hashes, Mapping):
        value = source_hashes.get(key)
        if isinstance(value, Mapping):
            return str(value.get("sha256") or "")
        if value:
            return str(value)
    artifacts = metadata.get("artifacts")
    if isinstance(artifacts, Mapping):
        entry = artifacts.get(key)
        if isinstance(entry, Mapping):
            return str(entry.get("sha256") or "")
    return ""


def _preserved_same_source_artifacts(
    *,
    map_dir: Path,
    metadata: Mapping[str, Any] | None,
    pcd_sha: str,
    source_profile: str,
    data_source: str,
    frame_id: str,
) -> dict[str, dict[str, Any]]:
    if not isinstance(metadata, Mapping):
        return {}
    artifacts = metadata.get("artifacts")
    if not isinstance(artifacts, Mapping):
        return {}

    preserved: dict[str, dict[str, Any]] = {}
    for name, default_path in (
        ("tomogram", "tomogram.pickle"),
        ("occupancy_grid", "occupancy.npz"),
    ):
        entry = artifacts.get(name)
        if not isinstance(entry, Mapping):
            continue
        try:
            path = _artifact_path(map_dir, entry, default_path)
        except ValueError:
            continue
        if not path.is_file():
            continue
        declared_sha = str(entry.get("sha256") or "")
        if not declared_sha or sha256_file(path) != declared_sha:
            continue
        if str(entry.get("source_map_sha256") or "") != pcd_sha:
            continue
        if str(entry.get("source_profile") or "") != source_profile:
            continue
        if str(entry.get("data_source") or "") != data_source:
            continue
        if normalize_saved_map_frame_id(str(entry.get("frame_id") or "")) != frame_id:
            continue
        preserved[name] = dict(entry)
    return preserved


def _artifact_path(map_dir: Path, entry: Mapping[str, Any], default_name: str) -> Path:
    value = str(entry.get("path") or default_name)
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


def _relative_path(root: Path, path: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return str(path)


def _pcd_point_count(path: Path) -> int:
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
    except Exception:
        return 0
    return 0


def _first_nonempty(*values: Any) -> str:
    for value in values:
        text = str(value or "").strip()
        if text:
            return text
    return "unknown"


def _mapping_str(mapping: Mapping[str, Any] | None, key: str) -> str:
    if not isinstance(mapping, Mapping):
        return ""
    return str(mapping.get(key) or "")


def _float_equal(left: Any, right: Any, *, eps: float = 1.0e-9) -> bool:
    try:
        return abs(float(left) - float(right)) <= eps
    except (TypeError, ValueError):
        return False


def _decode_process_text(value: bytes | str | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value
