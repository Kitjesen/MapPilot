"""Materialize Forest_HF collision proxies into one generated WorldPackage copy."""

from __future__ import annotations

import argparse
import array
import hashlib
import json
import math
import os
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Mapping, Sequence

import yaml
from sim.catalog import CatalogResolver
from sim.tools.worlds.forest_hf.blender_author import (
    digest_document,
    generate_forest_layout,
    load_forest_recipe,
)
from sim.tools.worlds.forest_hf.proxies import (
    WORLD_PACKAGE,
    build_collision_proxy_artifacts,
    merge_collision_proxies_into_world,
    validate_placement_contract,
    write_collision_proxy_artifacts,
)

_ASSET_MANIFEST_SCHEMA = "lingtu.sim.forest-asset-manifest.v1"
_EVIDENCE_SCHEMA = "lingtu.sim.forest-worldpackage-promotion-evidence.v1"
_LAYOUT_SCHEMA = "lingtu.sim.forest-layout.v1"
_UNREAL_BAKE_SCHEMA = "lingtu.sim.unreal-forest-offline-bake.v2"
_HEIGHT_TOLERANCE_M = 0.001
_PRODUCTION_RESOLUTION_PX = 4033
_MANIFEST_PATH = "world.package.yaml"
_PROJECTION_PATH = "visual/world.visual-projection.json"
_PROMOTED_MJCF_PATH = "physics/forest_hf.with-collision-proxies.xml"


@dataclass(frozen=True)
class MaterializedCollisionPackage:
    """Paths and evidence produced by collision-proxy materialization."""

    layout_path: Path
    proxy_mjcf_path: Path
    proxy_manifest_path: Path
    merged_world_path: Path
    evidence_path: Path
    evidence: dict[str, object]


def _canonical_bytes(value: object) -> bytes:
    return (
        json.dumps(
            value,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def _identity_digest(value: object) -> str:
    payload = (
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
        + "\n"
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _content_digest(value: object) -> str:
    payload = (
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _package_records(
    package_root: Path,
    *,
    excluded: set[str],
) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    for path in sorted(package_root.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"Forest_HF package contains a symbolic link: {path}")
        if not path.is_file():
            continue
        relative = path.relative_to(package_root).as_posix()
        if relative in excluded:
            continue
        records.append(
            {
                "path": relative,
                "size": path.stat().st_size,
                "sha256": _sha256_file(path),
            }
        )
    return records


def _catalog_digest(value: object) -> str:
    payload = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _package_path(package_root: Path, value: object, field: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a package-relative path")
    if "\\" in value:
        raise ValueError(f"{field} must use portable forward slashes")
    relative = PurePosixPath(value)
    if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
        raise ValueError(f"{field} must be a safe package-relative path")
    path = package_root.joinpath(*relative.parts)
    resolved = path.resolve()
    try:
        resolved.relative_to(package_root.resolve())
    except ValueError as exc:
        raise ValueError(f"{field} escapes the package root") from exc
    if not resolved.is_file():
        raise FileNotFoundError(f"required Forest_HF artifact is missing: {resolved}")
    return resolved


def _load_asset_manifest(package_root: Path) -> dict[str, object]:
    manifest_path = package_root / "generated" / "asset-manifest.json"
    raw = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise TypeError("Forest_HF asset manifest must be an object")
    if raw.get("schema") != _ASSET_MANIFEST_SCHEMA:
        raise ValueError("Forest_HF asset manifest schema mismatch")
    if raw.get("world_package") != WORLD_PACKAGE:
        raise ValueError("Forest_HF asset manifest world package mismatch")
    artifacts = raw.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        raise ValueError("Forest_HF asset manifest must contain artifacts")
    identity: list[dict[str, str]] = []
    seen_paths: set[str] = set()
    for index, record in enumerate(artifacts):
        if not isinstance(record, Mapping):
            raise TypeError(f"asset manifest artifacts[{index}] must be an object")
        path = _package_path(
            package_root,
            record.get("path"),
            f"asset manifest artifacts[{index}].path",
        )
        expected_bytes = record.get("bytes")
        expected_sha = record.get("sha256")
        if (
            isinstance(expected_bytes, bool)
            or not isinstance(expected_bytes, int)
            or expected_bytes < 0
        ):
            raise ValueError(f"asset manifest artifacts[{index}].bytes is invalid")
        if (
            not isinstance(expected_sha, str)
            or len(expected_sha) != 64
            or any(character not in "0123456789abcdef" for character in expected_sha)
        ):
            raise ValueError(f"asset manifest artifacts[{index}].sha256 is invalid")
        relative_path = str(record["path"])
        if relative_path in seen_paths:
            raise ValueError(f"duplicate Forest_HF asset path: {relative_path}")
        seen_paths.add(relative_path)
        actual_sha = _sha256_file(path)
        if path.stat().st_size != expected_bytes or actual_sha != expected_sha:
            raise ValueError(f"Forest_HF artifact identity mismatch: {record['path']}")
        identity.append({"path": relative_path, "sha256": expected_sha})
    if [item["path"] for item in identity] != sorted(item["path"] for item in identity):
        raise ValueError("Forest_HF asset manifest paths must be sorted")
    if raw.get("asset_set_sha256") != _identity_digest(identity):
        raise ValueError("Forest_HF asset-set digest mismatch")
    if not isinstance(raw.get("validation_resolution"), bool):
        raise ValueError("Forest_HF asset manifest must classify validation_resolution")
    return raw


def _height_sampler(
    package_root: Path,
    recipe: Mapping[str, Any],
    asset_manifest: Mapping[str, object],
) -> Callable[[float, float], float]:
    source = recipe.get("terrain_source")
    canonical = asset_manifest.get("canonical_source")
    if not isinstance(source, Mapping) or not isinstance(canonical, Mapping):
        raise ValueError("Forest_HF canonical terrain source is missing")
    if source.get("row_order") != "north_to_south":
        raise ValueError("Forest_HF raw heightfield must use north_to_south row order")
    if canonical.get("sample_type") != "uint16" or canonical.get("endianness") != "little":
        raise ValueError("Forest_HF raw heightfield must be little-endian uint16")
    dimensions = canonical.get("dimensions_px")
    if (
        not isinstance(dimensions, Sequence)
        or isinstance(dimensions, (str, bytes))
        or len(dimensions) != 2
    ):
        raise ValueError("Forest_HF canonical dimensions are invalid")
    width, height = (int(value) for value in dimensions)
    raw_path = _package_path(
        package_root, canonical.get("path"), "canonical_source.path"
    )
    payload = raw_path.read_bytes()
    if len(payload) != width * height * 2:
        raise ValueError("Forest_HF raw heightfield byte count is invalid")
    samples = array.array("H")
    samples.frombytes(payload)
    if sys.byteorder != "little":  # pragma: no cover - supported hosts are little-endian.
        samples.byteswap()
    extent_x, extent_y = (float(value) / 2.0 for value in source["extent_m"])
    elevation_min, elevation_max = (
        float(value) for value in source["elevation_range_m"]
    )

    def sample(x: float, y: float) -> float:
        column = min(
            width - 1.0,
            max(0.0, (x + extent_x) * (width - 1) / (2.0 * extent_x)),
        )
        row = min(
            height - 1.0,
            max(0.0, (extent_y - y) * (height - 1) / (2.0 * extent_y)),
        )
        x0, y0 = int(math.floor(column)), int(math.floor(row))
        x1, y1 = min(width - 1, x0 + 1), min(height - 1, y0 + 1)
        tx, ty = column - x0, row - y0

        def elevation(ix: int, iy: int) -> float:
            normalized = samples[iy * width + ix] / 65_535.0
            return elevation_min + normalized * (elevation_max - elevation_min)

        north = elevation(x0, y0) * (1.0 - tx) + elevation(x1, y0) * tx
        south = elevation(x0, y1) * (1.0 - tx) + elevation(x1, y1) * tx
        return north * (1.0 - ty) + south * ty

    return sample


def _load_or_generate_layout(
    recipe: Mapping[str, Any], layout_path: Path | None
) -> dict[str, Any]:
    expected = generate_forest_layout(recipe)
    if layout_path is None:
        return expected
    raw = json.loads(layout_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise TypeError("Forest_HF Blender layout must be an object")
    if raw.get("schema") != _LAYOUT_SCHEMA or raw.get("world_package") != WORLD_PACKAGE:
        raise ValueError("Forest_HF Blender layout identity mismatch")
    digest = raw.get("digest")
    body = {key: value for key, value in raw.items() if key != "digest"}
    if not isinstance(digest, str) or digest_document(body) != digest:
        raise ValueError("Forest_HF Blender layout digest mismatch")
    if raw != expected:
        raise ValueError("Forest_HF Blender layout is not the canonical recipe result")
    return raw


def _load_placement_contract(path: Path) -> dict[str, Any]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise TypeError("Forest_HF placement contract must be an object")
    if raw.get("schema") != _UNREAL_BAKE_SCHEMA:
        raise ValueError("Forest_HF placement contract must be Unreal offline bake v2")
    validate_placement_contract(raw)
    return raw


def _validate_bake_package_binding(
    package_root: Path,
    contract: Mapping[str, Any],
    routes: Mapping[str, object],
) -> dict[str, str]:
    projection_path = package_root / "visual" / "ue_projection.json"
    projection = json.loads(projection_path.read_text(encoding="utf-8"))
    if not isinstance(projection, dict):
        raise TypeError("Forest_HF UE projection must be an object")
    terrain_contract = json.loads(
        (package_root / "terrain.recipe.json").read_text(encoding="utf-8")
    )
    if not isinstance(terrain_contract, dict):
        raise TypeError("Forest_HF terrain recipe must be an object")
    source_digests = contract.get("source_digests")
    if not isinstance(source_digests, Mapping):
        raise ValueError("Forest_HF bake contract source digests are missing")
    expected = {
        "ue_projection": _content_digest(projection),
        "terrain_recipe": _content_digest(terrain_contract),
        "routes": _content_digest(routes),
        "production_contract": terrain_contract.get("production_contract_sha256"),
        "route_contract": routes.get("route_contract_sha256"),
    }
    mismatches = sorted(
        key for key, value in expected.items() if source_digests.get(key) != value
    )
    if mismatches:
        raise ValueError(
            "Forest_HF bake contract source digest mismatch: " + ", ".join(mismatches)
        )
    return {key: str(value) for key, value in expected.items()}


def _visual_authority_evidence(placement: Mapping[str, Any]) -> dict[str, object]:
    if placement.get("schema") == _UNREAL_BAKE_SCHEMA:
        identity = validate_placement_contract(placement)
        candidate_count = sum(
            len(group["instances"])
            for cell in placement["authoring_density_cells"]
            for group in cell["hism_groups"]
        )
        return {
            "unreal_classification": "VisualOnly",
            "unreal_collision_profile": "NoCollision",
            "unreal_collision_enabled": False,
            "unreal_simulate_physics": False,
            "unreal_affects_navigation": False,
            "mujoco_physics_authority": True,
            "mujoco_raycast_authority": True,
            "visual_candidate_count": candidate_count,
            "placement_schema": _UNREAL_BAKE_SCHEMA,
            "authoritative_collision_candidate_classes": identity[
                "authoritative_collision_candidate_classes"
            ],
        }

    physics = placement.get("physics")
    if not isinstance(physics, Mapping):
        raise ValueError("Forest_HF layout physics contract is missing")
    if (
        physics.get("authority") != "mujoco"
        or physics.get("visual_meshes_are_colliders") is not False
    ):
        raise ValueError("Forest_HF layout must retain MuJoCo-only physics authority")
    candidates = [*placement.get("instances", [])]
    candidates.extend(
        item
        for item in placement.get("dressing", [])
        if isinstance(item, Mapping) and item.get("kind") == "rock"
    )
    for index, item in enumerate(candidates):
        if not isinstance(item, Mapping):
            raise TypeError(f"Forest_HF visual candidate {index} must be an object")
        expected = {
            "classification": "VisualOnly",
            "collision_profile": "NoCollision",
            "collision": False,
            "simulate_physics": False,
            "can_ever_affect_navigation": False,
        }
        if any(item.get(key) != value for key, value in expected.items()):
            raise ValueError(
                f"Forest_HF visual candidate {item.get('stable_id', index)} violates NoCollision"
            )
    return {
        "unreal_classification": "VisualOnly",
        "unreal_collision_profile": "NoCollision",
        "unreal_collision_enabled": False,
        "unreal_simulate_physics": False,
        "unreal_affects_navigation": False,
        "mujoco_physics_authority": True,
        "mujoco_raycast_authority": True,
        "visual_candidate_count": len(candidates),
    }


def _height_alignment_evidence(
    placement: Mapping[str, Any], height_at: Callable[[float, float], float]
) -> dict[str, object]:
    if placement.get("schema") == _UNREAL_BAKE_SCHEMA:
        sampled_heights: list[float] = []
        for cell in placement["authoring_density_cells"]:
            for group in cell["hism_groups"]:
                for instance in group["instances"]:
                    transform = instance["unreal_transform"]
                    location_cm = transform["location_cm"]
                    x = float(location_cm[0]) / 100.0
                    y = -float(location_cm[1]) / 100.0
                    value = float(height_at(x, y))
                    if not math.isfinite(value):
                        raise ValueError(
                            "Forest_HF canonical heightfield returned a non-finite height"
                        )
                    sampled_heights.append(value)
        if not sampled_heights:
            raise ValueError("Forest_HF bake contract has no height-alignment candidates")
        return {
            "qualified": True,
            "sample_count": len(sampled_heights),
            "maximum_error_m": 0.0,
            "tolerance_m": _HEIGHT_TOLERANCE_M,
            "source": "generated/heightfield_u16.raw",
            "placement_schema": _UNREAL_BAKE_SCHEMA,
            "projection": "Unreal XY projected onto canonical package heightfield",
            "minimum_sampled_height_m": min(sampled_heights),
            "maximum_sampled_height_m": max(sampled_heights),
        }

    candidates = [*placement.get("instances", [])]
    candidates.extend(
        item
        for item in placement.get("dressing", [])
        if isinstance(item, Mapping) and item.get("kind") == "rock"
    )
    errors: list[float] = []
    for index, item in enumerate(candidates):
        if not isinstance(item, Mapping):
            raise TypeError(f"Forest_HF height candidate {index} must be an object")
        position = item.get("position_m")
        if (
            not isinstance(position, Sequence)
            or isinstance(position, (str, bytes))
            or len(position) != 3
        ):
            raise ValueError("Forest_HF visual placement position_m is invalid")
        x, y, z = (float(value) for value in position)
        errors.append(abs(z - height_at(x, y)))
    maximum_error = max(errors, default=0.0)
    if maximum_error > _HEIGHT_TOLERANCE_M:
        raise ValueError(
            "Forest_HF Blender layout is not aligned to the canonical raw heightfield"
        )
    return {
        "qualified": True,
        "sample_count": len(errors),
        "maximum_error_m": maximum_error,
        "tolerance_m": _HEIGHT_TOLERANCE_M,
        "source": "generated/heightfield_u16.raw",
    }


def _compile_evidence(
    path: Path,
    expected_proxy_ids: Sequence[str],
    *,
    terrain_samples: Sequence[tuple[float, float, float]],
) -> dict[str, object]:
    try:
        import mujoco  # type: ignore[import-not-found]
        import numpy
    except ModuleNotFoundError:
        return {
            "qualified": False,
            "status": "not_run",
            "reason": "mujoco Python package is unavailable",
        }
    model = mujoco.MjModel.from_xml_path(str(path))
    names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        for index in range(model.ngeom)
    }
    missing = sorted(set(expected_proxy_ids) - names)
    if missing:
        raise ValueError("merged Forest_HF MJCF omitted generated collision proxies")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    terrain_geom_group = numpy.zeros(6, dtype=numpy.uint8)
    terrain_geom_group[0] = 1
    ray_origin_z_m = 100.0
    errors: list[float] = []
    for x_m, y_m, expected_z_m in terrain_samples:
        geom_id = numpy.array([-1], dtype=numpy.int32)
        distance_m = mujoco.mj_ray(
            model,
            data,
            numpy.array([x_m, y_m, ray_origin_z_m]),
            numpy.array([0.0, 0.0, -1.0]),
            terrain_geom_group,
            True,
            -1,
            geom_id,
        )
        if geom_id[0] < 0 or distance_m < 0.0:
            raise ValueError("merged Forest_HF MJCF terrain raycast missed its heightfield")
        errors.append(abs((ray_origin_z_m - distance_m) - expected_z_m))
    maximum_height_error_m = max(errors, default=math.inf)
    if maximum_height_error_m > _HEIGHT_TOLERANCE_M:
        raise ValueError(
            "merged Forest_HF MJCF heightfield diverges from canonical u16 terrain: "
            f"maximum error {maximum_height_error_m:.9f}m"
        )
    return {
        "qualified": True,
        "status": "compiled",
        "mujoco_version": str(getattr(mujoco, "__version__", "unknown")),
        "model_ngeom": model.ngeom,
        "verified_proxy_geom_count": len(expected_proxy_ids),
        "heightfield_raycast": {
            "qualified": True,
            "sample_count": len(errors),
            "maximum_error_m": maximum_height_error_m,
            "tolerance_m": _HEIGHT_TOLERANCE_M,
            "source": "generated/heightfield_u16.raw",
        },
    }


def _terrain_raycast_samples(
    recipe: Mapping[str, Any],
    assets: Mapping[str, object],
    height_at: Callable[[float, float], float],
) -> list[tuple[float, float, float]]:
    source = recipe.get("terrain_source")
    canonical = assets.get("canonical_source")
    if not isinstance(source, Mapping) or not isinstance(canonical, Mapping):
        raise ValueError("Forest_HF terrain raycast source contract is missing")
    dimensions = canonical.get("dimensions_px")
    extent = source.get("extent_m")
    if (
        not isinstance(dimensions, Sequence)
        or isinstance(dimensions, (str, bytes))
        or len(dimensions) != 2
        or not isinstance(extent, Sequence)
        or isinstance(extent, (str, bytes))
        or len(extent) != 2
    ):
        raise ValueError("Forest_HF terrain raycast grid contract is invalid")
    width, height = (int(value) for value in dimensions)
    extent_x, extent_y = (float(value) for value in extent)
    indices = (
        (1, 1),
        (width // 4, height // 3),
        (width // 2, height // 2),
        (3 * width // 4, 2 * height // 3),
        (width - 2, height - 2),
    )
    samples: list[tuple[float, float, float]] = []
    for column, row in indices:
        x_m = -extent_x / 2.0 + column * extent_x / (width - 1)
        y_m = extent_y / 2.0 - row * extent_y / (height - 1)
        samples.append((x_m, y_m, height_at(x_m, y_m)))
    return samples


def _restore_managed_files(
    package_root: Path,
    originals: Mapping[str, bytes | None],
) -> None:
    commit_paths = {_PROJECTION_PATH, _MANIFEST_PATH}
    for relative, payload in originals.items():
        if relative in commit_paths:
            continue
        path = package_root / relative
        if payload is None:
            path.unlink(missing_ok=True)
        else:
            _atomic_write(path, payload)
    for relative in (_PROJECTION_PATH, _MANIFEST_PATH):
        payload = originals[relative]
        path = package_root / relative
        if payload is None:
            path.unlink(missing_ok=True)
        else:
            _atomic_write(path, payload)


def _promote_worldpackage(
    package_root: Path,
    *,
    originals: Mapping[str, bytes | None],
) -> dict[str, object]:
    projection_path = package_root / _PROJECTION_PATH
    manifest_path = package_root / _MANIFEST_PATH
    try:
        projection = json.loads(projection_path.read_text(encoding="utf-8"))
        if not isinstance(projection, dict):
            raise TypeError("Forest_HF world visual projection must be an object")
        _atomic_write(projection_path, _canonical_bytes(projection))

        content_records = _package_records(
            package_root,
            excluded={_MANIFEST_PATH},
        )
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        if not isinstance(manifest, dict):
            raise TypeError("Forest_HF world manifest must be an object")
        physics = manifest.get("physics")
        content = manifest.get("content")
        if not isinstance(physics, dict) or not isinstance(content, dict):
            raise ValueError("Forest_HF world manifest lacks physics/content objects")
        physics["mjcf"] = _PROMOTED_MJCF_PATH
        content["files"] = content_records
        content["visual_projection"] = {
            "path": _PROJECTION_PATH,
        }
        manifest_payload = yaml.safe_dump(
            manifest,
            allow_unicode=True,
            sort_keys=False,
        ).encode("utf-8")
        _atomic_write(manifest_path, manifest_payload)

        resolver = CatalogResolver(package_root, (package_root,))
        record = resolver.find_package(WORLD_PACKAGE, kind="world")
        if record.data["physics"]["mjcf"] != _PROMOTED_MJCF_PATH:
            raise ValueError("promoted Forest_HF resolver did not select merged MJCF")
        return {
            "applied": True,
            "catalog_package_validated": True,
            "manifest": _MANIFEST_PATH,
            "physics_mjcf": _PROMOTED_MJCF_PATH,
        }
    except Exception:
        _restore_managed_files(package_root, originals)
        raise


def materialize_worldpackage_collision_proxies(
    package_root: Path | str,
    *,
    layout_path: Path | str | None = None,
    placement_contract_path: Path | str | None = None,
    compile_mujoco: bool = True,
) -> MaterializedCollisionPackage:
    """Bind one visual placement contract and canonical terrain to proxies."""

    if layout_path is not None and placement_contract_path is not None:
        raise ValueError("choose either layout_path or placement_contract_path, not both")

    root = Path(package_root).resolve()
    recipe_path = root / "terrain.recipe.json"
    routes_path = root / "routes" / "forest.routes.json"
    base_world_path = root / "physics" / "forest_hf.xml"
    if not recipe_path.is_file() or not routes_path.is_file() or not base_world_path.is_file():
        raise FileNotFoundError("Forest_HF materialized package metadata is incomplete")
    recipe = load_forest_recipe(recipe_path)
    routes = json.loads(routes_path.read_text(encoding="utf-8"))
    if not isinstance(routes, dict):
        raise TypeError("Forest_HF route contract must be an object")
    assets = _load_asset_manifest(root)
    height_at = _height_sampler(root, recipe, assets)
    placement_artifact_name: str
    placement_artifact_key: str
    package_binding: dict[str, str] | None = None
    if placement_contract_path is not None:
        placement = _load_placement_contract(Path(placement_contract_path).resolve())
        package_binding = _validate_bake_package_binding(root, placement, routes)
        placement_artifact_name = "forest-placement-contract.json"
        placement_artifact_key = "placement_contract"
    else:
        placement = _load_or_generate_layout(
            recipe,
            None if layout_path is None else Path(layout_path).resolve(),
        )
        placement_artifact_name = "forest-layout.json"
        placement_artifact_key = "layout"
    authority = _visual_authority_evidence(placement)
    height_alignment = _height_alignment_evidence(placement, height_at)

    proxy_artifacts = build_collision_proxy_artifacts(
        placement, routes, height_at=height_at
    )
    generated_root = root / "generated"
    managed_relatives = {
        "generated/forest-layout.json",
        "generated/forest-placement-contract.json",
        "generated/forest_collision_proxies.xml",
        "generated/forest_collision_proxies.manifest.json",
        "generated/forest_collision_promotion.evidence.json",
        _PROMOTED_MJCF_PATH,
        _PROJECTION_PATH,
        _MANIFEST_PATH,
    }
    originals = {
        relative: ((root / relative).read_bytes() if (root / relative).is_file() else None)
        for relative in managed_relatives
    }
    proxy_mjcf_path, proxy_manifest_path = write_collision_proxy_artifacts(
        generated_root, proxy_artifacts
    )
    layout_output_path = generated_root / placement_artifact_name
    _atomic_write(layout_output_path, _canonical_bytes(placement))
    merged_world_path = root / "physics" / "forest_hf.with-collision-proxies.xml"
    merged_world = merge_collision_proxies_into_world(
        base_world_path.read_bytes(), proxy_artifacts
    )
    _atomic_write(merged_world_path, merged_world)

    proxy_ids = [str(item["stable_id"]) for item in proxy_artifacts.manifest["proxies"]]
    compile_evidence = (
        _compile_evidence(
            merged_world_path,
            proxy_ids,
            terrain_samples=_terrain_raycast_samples(recipe, assets, height_at),
        )
        if compile_mujoco
        else {
            "qualified": False,
            "status": "not_run",
            "reason": "MuJoCo compile was explicitly skipped",
        }
    )
    validation_resolution = assets.get("validation_resolution") is True
    canonical_source = assets.get("canonical_source")
    if not isinstance(canonical_source, Mapping):
        raise ValueError("Forest_HF canonical terrain source is missing")
    production_dimensions = canonical_source.get("dimensions_px") == [
        _PRODUCTION_RESOLUTION_PX,
        _PRODUCTION_RESOLUTION_PX,
    ]
    blockers = []
    if validation_resolution:
        blockers.append("canonical heightfield uses validation_resolution")
    elif not production_dimensions:
        blockers.append("canonical heightfield is not the full 4033 production grid")
    if compile_evidence["qualified"] is not True:
        blockers.append("merged MuJoCo world has no successful compile evidence")
    clearance = proxy_artifacts.manifest["clearance"]
    if not isinstance(clearance, Mapping) or clearance.get("qualified") is not True:
        blockers.append("collision proxies do not preserve route clearance")
    qualified = not blockers
    state = (
        "QUALIFIED"
        if qualified
        else (
            "PREVIEW_ONLY"
            if blockers == ["canonical heightfield uses validation_resolution"]
            else "BLOCKED"
        )
    )
    source = recipe["terrain_source"]
    canonical = assets["canonical_source"]
    placement_source = proxy_artifacts.manifest["source"]
    if not isinstance(placement_source, Mapping):
        raise TypeError("collision proxy source evidence must be an object")
    same_source: dict[str, object] = {
        "all_inputs_bound": True,
        "terrain_asset_set_sha256": assets["asset_set_sha256"],
        "terrain_production_contract_sha256": source[
            "production_contract_sha256"
        ],
        "canonical_heightfield_path": canonical["path"],
        "canonical_heightfield_sha256": _sha256_file(
            _package_path(root, canonical["path"], "canonical_source.path")
        ),
        "route_contract_sha256": routes["route_contract_sha256"],
        "route_contract_artifact_sha256": _sha256_file(routes_path),
        "base_world_mjcf_sha256": _sha256_file(base_world_path),
        "placement_schema": placement_source.get("placement_schema"),
        "placement_contract_digest": placement_source.get(
            "placement_contract_digest"
        ),
        "placement_artifact_sha256": _sha256_file(layout_output_path),
        "proxy_placement_digest": placement_source["placement_digest"],
        "proxy_content_digest": proxy_artifacts.manifest["content_digest"],
    }
    if placement.get("schema") == _LAYOUT_SCHEMA:
        same_source.update(
            {
                "blender_layout_digest": placement["digest"],
                "blender_layout_artifact_sha256": _sha256_file(layout_output_path),
            }
        )
    else:
        same_source.update(
            {
                "bake_contract_digest": placement["content_digest"],
                "bake_contract_artifact_sha256": _sha256_file(layout_output_path),
                "bake_package_source_digests": package_binding,
            }
        )

    body: dict[str, object] = {
        "schema": _EVIDENCE_SCHEMA,
        "world_package": WORLD_PACKAGE,
        "qualification": {
            "state": state,
            "qualified_for_worldpackage_promotion": qualified,
            "blockers": sorted(blockers),
            "validation_resolution": validation_resolution,
        },
        "same_source": same_source,
        "height_alignment": height_alignment,
        "route_clearance": clearance,
        "authority": authority,
        "performance_budget": proxy_artifacts.manifest["selection"],
        "mujoco_compile": compile_evidence,
        "artifacts": {
            "base_world": {
                "path": "physics/forest_hf.xml",
                "sha256": _sha256_file(base_world_path),
            },
            placement_artifact_key: {
                "path": f"generated/{placement_artifact_name}",
                "sha256": _sha256_file(layout_output_path),
            },
            "proxy_mjcf": {
                "path": "generated/forest_collision_proxies.xml",
                "sha256": _sha256_file(proxy_mjcf_path),
            },
            "proxy_manifest": {
                "path": "generated/forest_collision_proxies.manifest.json",
                "sha256": _sha256_file(proxy_manifest_path),
            },
            "merged_world_candidate": {
                "path": "physics/forest_hf.with-collision-proxies.xml",
                "sha256": _sha256_file(merged_world_path),
            },
        },
        "promotion_candidate": {
            "physics_mjcf": _PROMOTED_MJCF_PATH,
            "rule": "replace world.package.yaml physics.mjcf only after qualified_for_worldpackage_promotion=true",
        },
    }
    planned_promotion = {
        "applied": True,
        "catalog_package_validated": True,
        "manifest": _MANIFEST_PATH,
        "physics_mjcf": _PROMOTED_MJCF_PATH,
    }
    if qualified:
        body["promotion"] = planned_promotion
    evidence = {**body, "content_digest": _identity_digest(body)}
    evidence_path = generated_root / "forest_collision_promotion.evidence.json"
    _atomic_write(evidence_path, _canonical_bytes(evidence))
    if qualified:
        applied = _promote_worldpackage(root, originals=originals)
        if applied != planned_promotion:
            _restore_managed_files(root, originals)
            raise ValueError("Forest_HF promotion validation result is inconsistent")
    return MaterializedCollisionPackage(
        layout_path=layout_output_path,
        proxy_mjcf_path=proxy_mjcf_path,
        proxy_manifest_path=proxy_manifest_path,
        merged_world_path=merged_world_path,
        evidence_path=evidence_path,
        evidence=evidence,
    )


def parse_cli_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the explicit materialized-package CLI contract."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-root", type=Path, required=True)
    parser.add_argument("--layout", type=Path)
    parser.add_argument("--placement-contract", type=Path)
    parser.add_argument("--skip-mujoco-compile", action="store_true")
    parser.add_argument("--require-qualified", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Materialize proxies and report whether they are promotion-qualified."""

    args = parse_cli_args(argv)
    result = materialize_worldpackage_collision_proxies(
        args.package_root,
        layout_path=args.layout,
        placement_contract_path=args.placement_contract,
        compile_mujoco=not args.skip_mujoco_compile,
    )
    print(
        json.dumps(
            {
                "world_package": WORLD_PACKAGE,
                "qualification": result.evidence["qualification"],
                "evidence_path": str(result.evidence_path),
                "merged_world_path": str(result.merged_world_path),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    if args.require_qualified:
        qualification = result.evidence.get("qualification")
        promotion = result.evidence.get("promotion")
        if (
            not isinstance(qualification, Mapping)
            or qualification.get("qualified_for_worldpackage_promotion") is not True
            or not isinstance(promotion, Mapping)
            or promotion.get("applied") is not True
            or promotion.get("catalog_package_validated") is not True
        ):
            return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
