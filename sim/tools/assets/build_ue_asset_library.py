"""Build a deterministic UE5.8 visual-asset view over the canonical SimCatalog."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import yaml
from sim.catalog.management import SimCatalog
from sim.catalog.resolver import PackageRecord
from sim.catalog.visual_binding import VisualBindingError, compile_robot_visual_manifest
from sim.catalog.visual_projection import (
    VisualProjectionError,
    validate_robot_visual_projection_matches_manifest,
)
from sim.importers.contracts import write_json
from sim.tools.assets.tripo_visual_candidate import validate_visual_candidate_manifest

UE_ASSET_LIBRARY_SCHEMA = "lingtu.sim.ue-asset-library.v1"
DEFAULT_PACKAGE_REFERENCES: tuple[tuple[str, str], ...] = (
    ("robot", "thunderv4@1.0.3"),
    ("payload", "fictional_rws_01@1.0.0"),
)

_SUPPORTED_PACKAGE_KINDS = {"payload", "robot"}
_PACKAGE_REFERENCE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*@[A-Za-z0-9][A-Za-z0-9+_.-]*$")
_WINDOWS_ABSOLUTE_PATH = re.compile(r"(?:^|[\s\"'])(?:[A-Za-z]:[\\/]|\\\\)")


class UEAssetLibraryError(ValueError):
    """Raised when an asset source cannot support the fail-closed library view."""


def _mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise UEAssetLibraryError(f"{context} must be a JSON object")
    return value


def _string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise UEAssetLibraryError(f"{context} must be a non-empty string")
    return value


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _path_label(path: Path, root: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(root.resolve()).as_posix()
    except ValueError as exc:
        raise UEAssetLibraryError(
            "catalog package evidence must remain inside the catalog repository root"
        ) from exc


def _assert_portable_qualification_value(
    value: Any,
    context: str,
    *,
    filesystem_path: bool = False,
) -> None:
    if isinstance(value, Mapping):
        for key, item in value.items():
            _assert_portable_qualification_value(
                item,
                f"{context}.{key}",
                filesystem_path=key in {"path", "paths"},
            )
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _assert_portable_qualification_value(
                item,
                f"{context}[{index}]",
                filesystem_path=filesystem_path,
            )
        return
    if not isinstance(value, str):
        return
    if _WINDOWS_ABSOLUTE_PATH.search(value) is not None or (
        filesystem_path and (value.startswith("/") or "\\" in value or ":" in value)
    ):
        raise UEAssetLibraryError(
            f"{context} contains one machine-specific filesystem path"
        )


def _read_regular_file(path: Path, context: str) -> bytes:
    candidate = Path(path)
    if candidate.is_symlink() or not candidate.is_file():
        raise UEAssetLibraryError(f"{context} must identify one regular, link-free file")
    try:
        return candidate.read_bytes()
    except OSError as exc:
        raise UEAssetLibraryError(f"cannot read {context}: {exc}") from exc


def _package_manifest_descriptor(
    catalog: SimCatalog,
    record: PackageRecord,
) -> dict[str, Any]:
    body = _read_regular_file(record.manifest_path, f"{record.ref} package manifest")
    try:
        observed = yaml.safe_load(body.decode("utf-8"))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise UEAssetLibraryError(f"cannot decode {record.ref} package manifest: {exc}") from exc
    if observed != record.data:
        raise UEAssetLibraryError(f"{record.ref} package manifest changed during indexing")
    return {
        "path": _path_label(record.manifest_path, catalog.repo_root),
        "bytes": len(body),
    }


def _read_json_object(path: Path, context: str) -> tuple[Mapping[str, Any], bytes]:
    body = _read_regular_file(path, context)
    try:
        value = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise UEAssetLibraryError(f"cannot decode {context}: {exc}") from exc
    return _mapping(value, context), body


def _safe_relative_file(package_root: Path, value: Any, context: str) -> Path:
    relative = _string(value, context)
    parts = relative.split("/")
    if (
        relative.startswith("/")
        or "\\" in relative
        or ":" in relative
        or any(not part or part in {".", ".."} for part in parts)
    ):
        raise UEAssetLibraryError(f"{context} must be one safe relative POSIX path")
    raw_path = package_root / Path(*parts)
    if raw_path.is_symlink():
        raise UEAssetLibraryError(f"{context} must not identify a link")
    resolved = raw_path.resolve()
    try:
        resolved.relative_to(package_root.resolve())
    except ValueError as exc:
        raise UEAssetLibraryError(f"{context} escapes its package root") from exc
    if not resolved.is_file():
        raise UEAssetLibraryError(f"{context} must identify one existing regular file")
    return resolved


def _unreal_asset_path(value: Any, context: str) -> str:
    path = _string(value, context)
    if (
        not path.startswith(("/Game/", "/Engine/"))
        or "\\" in path
        or "//" in path
        or any(character.isspace() for character in path)
        or any(component in {".", ".."} for component in path.split("/"))
    ):
        raise UEAssetLibraryError(f"{context} must be one canonical Unreal asset path")
    return path


def _projection_source(record: PackageRecord) -> str:
    visual = _mapping(record.data.get("visual"), f"{record.ref}.visual")
    projection = visual.get("projection")
    if record.kind == "robot":
        return _string(projection, f"{record.ref}.visual.projection")
    if record.kind == "payload":
        descriptor = _mapping(projection, f"{record.ref}.visual.projection")
        return _string(descriptor.get("path"), f"{record.ref}.visual.projection.path")
    raise UEAssetLibraryError(f"unsupported package kind for UE assets: {record.kind}")


def _projection_assets(
    projection: Mapping[str, Any],
    *,
    record: PackageRecord,
) -> tuple[str, list[str]]:
    expected_schema = {
        "robot": "lingtu.sim.robot-visual-projection.v1",
        "payload": "lingtu.sim.payload-visual-projection.v1",
    }[record.kind]
    if projection.get("schema") != expected_schema:
        raise UEAssetLibraryError(f"{record.ref} visual projection must use {expected_schema}")
    package = _mapping(projection.get("package"), f"{record.ref}.projection.package")
    if package.get("id") != record.id or package.get("version") != record.version:
        raise UEAssetLibraryError(f"{record.ref} visual projection package identity is stale")
    visual = _mapping(record.data.get("visual"), f"{record.ref}.visual")
    binding = _string(projection.get("binding"), f"{record.ref}.projection.binding")
    if binding != visual.get("binding"):
        raise UEAssetLibraryError(f"{record.ref} visual projection binding is stale")

    components = projection.get("components")
    if not isinstance(components, list) or not components:
        raise UEAssetLibraryError(f"{record.ref} visual projection must contain components")
    assets: list[str] = []
    for index, component_value in enumerate(components):
        component = _mapping(component_value, f"{record.ref}.components[{index}]")
        if record.kind == "robot":
            unreal = _mapping(
                component.get("unreal"),
                f"{record.ref}.components[{index}].unreal",
            )
            asset = unreal.get("asset_path")
        else:
            asset = component.get("unreal_asset")
        assets.append(_unreal_asset_path(asset, f"{record.ref}.components[{index}].unreal_asset"))
    return binding, sorted(assets)


def _validate_robot_projection_against_package(
    projection: Mapping[str, Any],
    record: PackageRecord,
) -> None:
    try:
        visual_manifest = compile_robot_visual_manifest(record.manifest_path.parent).to_dict()
        canonical_projection = validate_robot_visual_projection_matches_manifest(
            projection,
            visual_manifest,
        ).to_dict()
    except (VisualBindingError, VisualProjectionError) as exc:
        raise UEAssetLibraryError(
            f"{record.ref} canonical visual projection is invalid: {exc}"
        ) from exc
    if canonical_projection != dict(projection):
        raise UEAssetLibraryError(
            f"{record.ref} canonical visual projection changed during indexing"
        )


def _package_entry(catalog: SimCatalog, *, kind: str, reference: str) -> dict[str, Any]:
    record = catalog.resolver.find_package(reference, kind=kind)
    projection_relative = _projection_source(record)
    projection_path = _safe_relative_file(
        record.manifest_path.parent,
        projection_relative,
        f"{record.ref}.visual.projection",
    )
    projection, projection_bytes = _read_json_object(
        projection_path,
        f"{record.ref} visual projection",
    )
    if record.kind == "robot":
        _validate_robot_projection_against_package(projection, record)
    binding, unreal_assets = _projection_assets(projection, record=record)

    if record.kind == "payload":
        if projection.get("authority") != "mujoco":
            raise UEAssetLibraryError(f"{record.ref} must preserve MuJoCo authority")
        visual_policy = _mapping(
            projection.get("visual_policy"),
            f"{record.ref}.visual_policy",
        )
        if (
            visual_policy.get("collision_enabled") != "no_collision"
            or visual_policy.get("simulate_physics") is not False
        ):
            raise UEAssetLibraryError(f"{record.ref} cannot introduce Unreal collision or physics authority")

    manifest = _package_manifest_descriptor(catalog, record)
    qualification = catalog.qualification(reference, kind=kind)
    _assert_portable_qualification_value(
        qualification,
        f"{record.ref}.qualification",
    )
    return {
        "entry_id": f"{kind}:{reference}",
        "source_type": "catalog_package",
        "asset_kind": kind,
        "availability": "CATALOG_PACKAGE",
        "package": {
            "kind": record.kind,
            "id": record.id,
            "version": record.version,
            "ref": record.ref,
        },
        "source": {"manifest": manifest},
        "qualification": qualification,
        "visual": {
            "binding": binding,
            "projection": {
                "path": _path_label(projection_path, catalog.repo_root),
                "bytes": len(projection_bytes),
                "schema": projection["schema"],
            },
            "component_count": len(unreal_assets),
            "unreal_assets": unreal_assets,
        },
        "render_policy": {
            "physics_authority": "mujoco",
            "unreal_collision_profile": "NoCollision",
            "unreal_simulate_physics": False,
        },
    }


def _candidate_artifact_path(manifest_path: Path, value: Any) -> Path:
    artifact_path = _string(value, "candidate.asset.artifact.path")
    parts = artifact_path.split("/")
    if (
        artifact_path.startswith("/")
        or "\\" in artifact_path
        or ":" in artifact_path
        or any(not part or part in {".", ".."} for part in parts)
    ):
        raise UEAssetLibraryError("candidate.asset.artifact.path must be one safe relative POSIX path")
    root = manifest_path.parent.resolve()
    raw_path = root / Path(*parts)
    if raw_path.is_symlink():
        raise UEAssetLibraryError("candidate artifact must not be a link")
    path = raw_path.resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise UEAssetLibraryError("candidate artifact escapes its manifest root") from exc
    return path


def _candidate_entry(manifest_path: Path) -> dict[str, Any]:
    path = Path(manifest_path).resolve()
    document, manifest_bytes = _read_json_object(path, "visual candidate manifest")
    try:
        document = validate_visual_candidate_manifest(document)
    except ValueError as exc:
        raise UEAssetLibraryError(f"visual candidate manifest is invalid: {exc}") from exc

    asset = _mapping(document.get("asset"), "candidate.asset")
    asset_id = _string(asset.get("id"), "candidate.asset.id")
    artifact = _mapping(asset.get("artifact"), "candidate.asset.artifact")
    artifact_path = _candidate_artifact_path(path, artifact.get("path"))
    artifact_bytes = _read_regular_file(artifact_path, "visual candidate artifact")
    artifact_sha256 = _sha256_bytes(artifact_bytes)
    artifact_descriptor = {
        "path": _string(artifact.get("path"), "candidate.asset.artifact.path"),
        "bytes": len(artifact_bytes),
    }
    if artifact_descriptor["bytes"] != artifact.get("bytes") or artifact_sha256 != artifact.get("sha256"):
        raise UEAssetLibraryError("visual candidate artifact identity is stale")
    qualification = _mapping(document.get("qualification"), "candidate.qualification")
    binding = _mapping(document.get("binding"), "candidate.binding")
    unreal = _mapping(binding.get("unreal"), "candidate.binding.unreal")
    unreal_asset = _unreal_asset_path(
        unreal.get("asset_path"),
        "candidate.binding.unreal.asset_path",
    )
    if not unreal_asset.startswith("/Game/RobotSim/Staging/"):
        raise UEAssetLibraryError("visual candidates must remain under /Game/RobotSim/Staging")

    return {
        "entry_id": f"candidate:{asset_id}",
        "source_type": "generated_candidate",
        "asset_kind": asset.get("role"),
        "availability": "QUARANTINED",
        "source": {
            "manifest": {
                "path": path.name,
                "bytes": len(manifest_bytes),
            },
            "artifact": artifact_descriptor,
        },
        "provenance": dict(_mapping(document.get("provenance"), "candidate.provenance")),
        "qualification": dict(qualification),
        "binding": {
            "world_entity_id": binding.get("world_entity_id"),
            "semantic_class": binding.get("semantic_class"),
        },
        "visual": {
            "component_count": 1,
            "unreal_assets": [unreal_asset],
            "geometry": dict(_mapping(document.get("geometry"), "candidate.geometry")),
        },
        "render_policy": {
            "physics_authority": "mujoco",
            "unreal_collision_profile": "NoCollision",
            "unreal_simulate_physics": False,
        },
    }


def _normalized_package_references(
    values: Sequence[tuple[str, str]],
) -> tuple[tuple[str, str], ...]:
    result: list[tuple[str, str]] = []
    for kind, reference in values:
        if kind not in _SUPPORTED_PACKAGE_KINDS:
            raise UEAssetLibraryError(f"unsupported UE asset package kind: {kind}")
        if _PACKAGE_REFERENCE.fullmatch(reference) is None:
            raise UEAssetLibraryError(f"package reference must be exact id@version: {reference}")
        result.append((kind, reference))
    if len(set(result)) != len(result):
        raise UEAssetLibraryError("package references must be unique")
    return tuple(sorted(result))


def build_ue_asset_library(
    catalog: SimCatalog,
    *,
    package_references: Sequence[tuple[str, str]],
    candidate_manifest_paths: Sequence[Path] = (),
) -> dict[str, Any]:
    """Project validated packages and quarantined candidates into one offline view."""

    packages = _normalized_package_references(package_references)
    candidates = tuple(sorted(Path(path).resolve() for path in candidate_manifest_paths))
    if len(set(candidates)) != len(candidates):
        raise UEAssetLibraryError("candidate manifest paths must be unique")

    package_entries = [_package_entry(catalog, kind=kind, reference=reference) for kind, reference in packages]
    candidate_entries = [_candidate_entry(path) for path in candidates]
    entries = [*package_entries, *candidate_entries]
    entries.sort(key=lambda entry: entry["entry_id"])
    entry_ids = [entry["entry_id"] for entry in entries]
    if len(set(entry_ids)) != len(entry_ids):
        raise UEAssetLibraryError("UE asset library entry identities must be unique")

    body: dict[str, Any] = {
        "schema": UE_ASSET_LIBRARY_SCHEMA,
        "engine": {"name": "Unreal Engine", "major": 5, "minor": 8},
        "policy": {
            "catalog_authority": "SimCatalog",
            "runtime_materialization": "ResolvedSessionBundle",
            "generated_candidates_auto_promote": False,
            "physics_authority": "mujoco",
            "unreal_collision_profile": "NoCollision",
        },
        "selection": {
            "packages": [{"kind": kind, "ref": reference} for kind, reference in packages],
            "candidate_manifests": [
                entry["source"]["manifest"]["path"]
                for entry in sorted(candidate_entries, key=lambda item: item["entry_id"])
            ],
        },
        "summary": {
            "entry_count": len(entries),
            "catalog_package_count": sum(entry["source_type"] == "catalog_package" for entry in entries),
            "quarantined_candidate_count": sum(entry["availability"] == "QUARANTINED" for entry in entries),
        },
        "entries": entries,
    }
    return body


def build_repository_ue_asset_library(
    repo_root: Path,
    *,
    package_references: Sequence[tuple[str, str]] = DEFAULT_PACKAGE_REFERENCES,
    candidate_manifest_paths: Sequence[Path] = (),
) -> dict[str, Any]:
    """Open the canonical repository catalog and build its selected asset view."""

    return build_ue_asset_library(
        SimCatalog.from_repository(Path(repo_root)),
        package_references=package_references,
        candidate_manifest_paths=candidate_manifest_paths,
    )


def write_ue_asset_library(document: Mapping[str, Any], output_path: Path) -> Path:
    """Write one deterministic UE asset-library document."""

    result: Path = write_json(Path(output_path), dict(document))
    return result


def _package_argument(value: str) -> tuple[str, str]:
    kind, separator, reference = value.partition(":")
    if separator != ":":
        raise argparse.ArgumentTypeError("package must use KIND:ID@VERSION")
    try:
        return _normalized_package_references(((kind, reference),))[0]
    except UEAssetLibraryError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=("Build a deterministic UE5.8 visual-asset library without launching Unreal, Blender, or Tripo.")
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[3],
    )
    parser.add_argument(
        "--package",
        action="append",
        type=_package_argument,
        dest="packages",
        help="Exact selected package as KIND:ID@VERSION; repeatable.",
    )
    parser.add_argument(
        "--candidate-manifest",
        action="append",
        type=Path,
        default=[],
        help="Quarantined visual-asset-candidate manifest; repeatable.",
    )
    parser.add_argument("--output", required=True, type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Build and write the selected offline UE5.8 asset library."""

    args = _parser().parse_args(argv)
    document = build_repository_ue_asset_library(
        args.repo_root,
        package_references=args.packages or DEFAULT_PACKAGE_REFERENCES,
        candidate_manifest_paths=args.candidate_manifest,
    )
    write_ue_asset_library(document, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "DEFAULT_PACKAGE_REFERENCES",
    "UE_ASSET_LIBRARY_SCHEMA",
    "UEAssetLibraryError",
    "build_repository_ue_asset_library",
    "build_ue_asset_library",
    "write_ue_asset_library",
]
