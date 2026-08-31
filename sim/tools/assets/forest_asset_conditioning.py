"""Freeze visual-only conditioning plans for generated forest assets.

This module does not run Blender or assert that an Unreal import succeeded.
The generic static-prop conditioner is intentionally incompatible because its
LOD policy is too dense for the first Tripo forest candidates.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import stat
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath
from typing import Any, cast

from sim.tools.assets.static_prop_conditioning import (
    _atomic_publish_no_replace,
    _open_file_with_parent_identity,
    _validate_self_contained_glb,
)

PLAN_SCHEMA = "lingtu.sim.forest-asset-conditioning-plan.v1"
CONDITIONER_CONTRACT = "lingtu.sim.forest-asset-conditioner.v1"
MAX_SOURCE_GLB_BYTES = 512 * 1024 * 1024
MAX_TASK_JSON_BYTES = 1024 * 1024
PROFILE_PATH = Path(__file__).with_name("tripo_hero_static_pbr.v1.json")

_IDENTITY = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_GENERIC_CONDITIONER = "blender_static_prop_conditioner.py"

_FOREST_SPECS: dict[str, dict[str, Any]] = {
    "pine": {
        "target_dimensions_m": [3.7, 3.7, 8.0],
        "topology_policy": "branched_foliage_visual_review",
        "lods": [("LOD0", 0.08, 120_000), ("LOD1", 0.03, 45_000), ("LOD2", 0.01, 15_000)],
    },
    "birch": {
        "target_dimensions_m": [2.3, 2.3, 7.0],
        "topology_policy": "branched_foliage_visual_review",
        "lods": [("LOD0", 0.08, 120_000), ("LOD1", 0.03, 45_000), ("LOD2", 0.01, 15_000)],
    },
    "boulder": {
        "target_dimensions_m": [1.6, 1.3, 1.1],
        "topology_policy": "closed_surface_topology_review",
        "lods": [("LOD0", 0.06, 90_000), ("LOD1", 0.02, 30_000), ("LOD2", 0.005, 8_000)],
    },
    "grass_clump": {
        "target_dimensions_m": [0.55, 0.42, 0.22],
        "topology_policy": "thin_foliage_visual_review",
        "lods": [("LOD0", 0.12, 24_000), ("LOD1", 0.04, 8_000), ("LOD2", 0.01, 2_000)],
    },
    "fern_clump": {
        "target_dimensions_m": [0.78, 0.68, 0.43],
        "topology_policy": "thin_foliage_visual_review",
        "lods": [("LOD0", 0.15, 36_000), ("LOD1", 0.05, 12_000), ("LOD2", 0.012, 3_000)],
    },
    "forest_floor_debris": {
        "target_dimensions_m": [0.9, 0.68, 0.1],
        "topology_policy": "mixed_floor_debris_visual_review",
        "lods": [("LOD0", 0.12, 30_000), ("LOD1", 0.04, 10_000), ("LOD2", 0.01, 2_500)],
    },
}

_BLOCKERS = [
    "license_and_usage_rights_unverified",
    "unreal_import_not_verified",
    "topology_not_verified",
]


def _digest(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _identity(value: Any, context: str) -> str:
    if not isinstance(value, str) or _IDENTITY.fullmatch(value) is None:
        raise ValueError(f"{context} must be one stable identifier")
    return value


def _portable_path(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{context} must be a non-empty path")
    path = PurePosixPath(value)
    if (
        path.is_absolute()
        or "\\" in value
        or ":" in value
        or any(part in {"", ".", ".."} for part in path.parts)
        or any(character.isspace() for character in value)
    ):
        raise ValueError(f"{context} must be one canonical relative path")
    return path.as_posix()


def _game_path(value: Any) -> str:
    if (
        not isinstance(value, str)
        or not value.startswith("/Game/")
        or "\\" in value
        or "//" in value
        or any(character.isspace() for character in value)
        or any(part in {"", ".", ".."} for part in value[1:].split("/"))
    ):
        raise ValueError("unreal_asset_path must be one canonical /Game asset path")
    return value


def _link_free_absolute(path: Path, context: str) -> Path:
    candidate = Path(os.path.abspath(os.fspath(path)))
    current = Path(candidate.anchor)
    reparse_flag = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    for component in candidate.parts[1:]:
        current /= component
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            continue
        if current.is_symlink() or (
            reparse_flag and getattr(metadata, "st_file_attributes", 0) & reparse_flag
        ):
            raise ValueError(f"{context} must identify one link-free path")
    return candidate


def _source_record(path: Path, root: Path, context: str) -> dict[str, Any]:
    resolved = _link_free_absolute(path, context).resolve()
    if not resolved.is_file():
        raise ValueError(f"{context} must identify one regular, link-free file")
    try:
        relative = resolved.relative_to(root).as_posix()
    except ValueError as exc:
        raise ValueError(f"{context} must remain inside artifact_root") from exc
    return {"path": relative, "bytes": resolved.stat().st_size, "sha256": _sha256_file(resolved)}


def _read_bound_file(path: Path, *, maximum_bytes: int, context: str) -> bytes:
    candidate = _link_free_absolute(path, context)
    descriptor, _parent_identity = _open_file_with_parent_identity(candidate)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_size <= 0:
            raise ValueError(f"{context} must identify one non-empty regular file")
        if before.st_size > maximum_bytes:
            raise ValueError(f"{context} exceeds the physical byte budget")
        payload = bytearray()
        while len(payload) <= maximum_bytes:
            block = os.read(descriptor, min(1024 * 1024, maximum_bytes + 1 - len(payload)))
            if not block:
                break
            payload.extend(block)
        after = os.fstat(descriptor)
        if len(payload) != before.st_size or (before.st_dev, before.st_ino) != (
            after.st_dev,
            after.st_ino,
        ):
            raise ValueError(f"{context} changed while it was being read")
        return bytes(payload)
    finally:
        os.close(descriptor)


def _axis_order(value: Any) -> list[str]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or list(value) != ["x", "y", "z"]
    ):
        raise ValueError("source_axis_order must be x, y, z for Blender-imported Tripo GLB")
    return list(value)


def _lods(asset_class: str) -> list[dict[str, Any]]:
    return [
        {"name": name, "triangle_ratio": ratio, "max_triangles": maximum}
        for name, ratio, maximum in _FOREST_SPECS[asset_class]["lods"]
    ]


def _outputs(asset_id: str, asset_class: str, directory: str) -> dict[str, Any]:
    return {
        "directory": directory,
        "blend": f"{directory}/{asset_id}.blend",
        "preview": f"{directory}/preview.png",
        "inspection": f"{directory}/conditioning-report.json",
        "lods": [
            {
                "name": name,
                "glb": f"{directory}/{asset_id}.{name}.glb",
                "fbx": f"{directory}/{asset_id}.{name}.fbx",
            }
            for name, _ratio, _maximum in _FOREST_SPECS[asset_class]["lods"]
        ],
    }


def asset_class_from_id(asset_id: str) -> str:
    """Return the forest class encoded as a stable token in an asset id."""

    lowered = asset_id.lower()
    matches = [
        asset_class
        for asset_class in _FOREST_SPECS
        if re.search(rf"(?:^|[.-]){re.escape(asset_class)}(?:$|[.-])", lowered)
    ]
    if len(matches) != 1:
        classes = ", ".join(_FOREST_SPECS)
        raise ValueError(f"asset_id must contain exactly one supported forest class token: {classes}")
    return matches[0]


def build_forest_asset_conditioning_plan(
    *,
    artifact_root: Path,
    asset_id: str,
    asset_class: str,
    source_model_path: Path,
    task_path: Path,
    output_directory: str,
    world_entity_id: str,
    unreal_asset_path: str,
    source_axis_order: Sequence[str] = ("x", "y", "z"),
) -> dict[str, Any]:
    """Build one content-addressed, visual-only forest conditioning plan."""

    root = _link_free_absolute(artifact_root, "artifact_root").resolve()
    if not root.is_dir():
        raise ValueError("artifact_root must identify one existing directory")
    canonical_id = _identity(asset_id, "asset_id")
    if asset_class not in _FOREST_SPECS or asset_class_from_id(canonical_id) != asset_class:
        raise ValueError("asset_class must match the supported forest class encoded in asset_id")
    directory = _portable_path(output_directory, "output_directory")
    if "/" in directory:
        raise ValueError("output_directory must be one direct child directory")

    if Path(source_model_path).suffix.lower() != ".glb":
        raise ValueError("source_model_path must be one single-file .glb closure")
    model_relative = _source_record(source_model_path, root, "source_model_path")["path"]
    task_relative = _source_record(task_path, root, "task_path")["path"]
    model_bytes = _read_bound_file(
        source_model_path, maximum_bytes=MAX_SOURCE_GLB_BYTES, context="source_model_path"
    )
    task_bytes = _read_bound_file(
        task_path, maximum_bytes=MAX_TASK_JSON_BYTES, context="task_path"
    )
    profile_bytes = _read_bound_file(
        PROFILE_PATH, maximum_bytes=MAX_TASK_JSON_BYTES, context="repository profile"
    )
    _validate_self_contained_glb(model_bytes, "source_model_path")
    model = {
        "path": model_relative,
        "bytes": len(model_bytes),
        "sha256": hashlib.sha256(model_bytes).hexdigest(),
    }
    task = {
        "path": task_relative,
        "bytes": len(task_bytes),
        "sha256": hashlib.sha256(task_bytes).hexdigest(),
    }
    try:
        task_document = json.loads(task_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read task: {exc}") from exc
    task_input = task_document.get("input")
    if not isinstance(task_input, Mapping):
        raise ValueError("task.input must be one JSON object")
    if task_document.get("status") != "success":
        raise ValueError("Tripo task must have completed successfully")
    if task_document.get("type") not in {"text_to_model", "multiview_to_model"}:
        raise ValueError("Tripo task type is unsupported")
    if task_input.get("pbr") is not True or task_input.get("texture") is not True:
        raise ValueError("Tripo task must contain textured PBR output")
    task["task_id"] = _identity(task_document.get("task_id"), "task.task_id")
    task["task_type"] = task_document["type"]
    task["model_version"] = _identity(task_input.get("model_version"), "task.input.model_version")

    body = {
        "schema": PLAN_SCHEMA,
        "conditioner_contract": CONDITIONER_CONTRACT,
        "asset": {"id": canonical_id, "class": asset_class, "role": "forest_visual_candidate"},
        "source": {
            "model": model,
            "task": task,
            "profile": {
                "path": "sim/tools/assets/tripo_hero_static_pbr.v1.json",
                "bytes": len(profile_bytes),
                "sha256": hashlib.sha256(profile_bytes).hexdigest(),
            },
        },
        "geometry": {
            "target_dimensions_m": _FOREST_SPECS[asset_class]["target_dimensions_m"],
            "source_axis_order": _axis_order(source_axis_order),
            "imported_coordinate_system": {"up": "+Z", "unit": "meter"},
            "normalization": "axis_fit_center_xy_ground_z",
            "topology_policy": _FOREST_SPECS[asset_class]["topology_policy"],
        },
        "lods": _lods(asset_class),
        "binding": {
            "world_entity_id": _identity(world_entity_id, "world_entity_id"),
            "physics_authority": "mujoco_world_proxy",
            "visual_mesh_is_physics_proxy": False,
            "unreal": {
                "asset_path": _game_path(unreal_asset_path),
                "collision_profile": "NoCollision",
                "collision_enabled": False,
                "simulate_physics": False,
                "generate_overlap_events": False,
                "can_ever_affect_navigation": False,
            },
        },
        "outputs": _outputs(canonical_id, asset_class, directory),
        "qualification": {
            "state": "QUARANTINED",
            "blockers": list(_BLOCKERS),
            "promotion_target": "WorldPackage.visual facet",
        },
    }
    return validate_forest_asset_conditioning_plan({**body, "digest": _digest(body)})


def validate_forest_asset_conditioning_plan(document: Mapping[str, Any]) -> dict[str, Any]:
    """Validate a plan by rebuilding its deterministic policy-bearing fields."""

    if not isinstance(document, Mapping):
        raise ValueError("conditioning plan must be one JSON object")
    expected_keys = {
        "schema", "conditioner_contract", "asset", "source", "geometry", "lods",
        "binding", "outputs", "qualification", "digest",
    }
    if set(document) != expected_keys:
        raise ValueError("conditioning plan keys must match the forest contract")
    if document.get("schema") != PLAN_SCHEMA or document.get("conditioner_contract") != CONDITIONER_CONTRACT:
        raise ValueError("conditioning plan schema or conditioner contract is unsupported")
    digest = document.get("digest")
    body = {key: value for key, value in document.items() if key != "digest"}
    if not isinstance(digest, str) or _SHA256.fullmatch(digest) is None or _digest(body) != digest:
        raise ValueError("conditioning plan digest does not match its content")

    asset = document.get("asset")
    if not isinstance(asset, Mapping):
        raise ValueError("asset must be one JSON object")
    asset_id = _identity(asset.get("id"), "asset.id")
    asset_class = asset.get("class")
    if asset != {"id": asset_id, "class": asset_class, "role": "forest_visual_candidate"}:
        raise ValueError("asset must be one forest visual candidate")
    if asset_class not in _FOREST_SPECS or asset_class_from_id(asset_id) != asset_class:
        raise ValueError("asset class and id do not match")

    source = document.get("source")
    if not isinstance(source, Mapping) or set(source) != {"model", "task", "profile"}:
        raise ValueError("source must bind model, task, and profile artifacts")
    for name, keys in (
        ("model", {"path", "bytes", "sha256"}),
        ("task", {"path", "bytes", "sha256", "task_id", "task_type", "model_version"}),
        ("profile", {"path", "bytes", "sha256"}),
    ):
        record = source.get(name)
        if not isinstance(record, Mapping) or set(record) != keys:
            raise ValueError(f"source.{name} does not match the identity contract")
        _portable_path(record.get("path"), f"source.{name}.path")
        if isinstance(record.get("bytes"), bool) or not isinstance(record.get("bytes"), int) or record["bytes"] <= 0:
            raise ValueError(f"source.{name}.bytes must be positive")
        if not isinstance(record.get("sha256"), str) or _SHA256.fullmatch(record["sha256"]) is None:
            raise ValueError(f"source.{name}.sha256 must be a lowercase SHA-256")
    _identity(source["task"]["task_id"], "source.task.task_id")
    _identity(source["task"]["model_version"], "source.task.model_version")
    if source["task"]["task_type"] not in {"text_to_model", "multiview_to_model"}:
        raise ValueError("source.task.task_type is unsupported")

    geometry = document.get("geometry")
    spec = _FOREST_SPECS[asset_class]
    if not isinstance(geometry, Mapping) or geometry != {
        "target_dimensions_m": spec["target_dimensions_m"],
        "source_axis_order": ["x", "y", "z"],
        "imported_coordinate_system": {"up": "+Z", "unit": "meter"},
        "normalization": "axis_fit_center_xy_ground_z",
        "topology_policy": spec["topology_policy"],
    }:
        raise ValueError("geometry must match the forest class conditioning policy")
    if document.get("lods") != _lods(asset_class):
        raise ValueError("lods must match the deterministic forest reduction policy")

    binding = document.get("binding")
    if not isinstance(binding, Mapping) or set(binding) != {
        "world_entity_id", "physics_authority", "visual_mesh_is_physics_proxy", "unreal"
    }:
        raise ValueError("binding does not match the visual-only authority contract")
    _identity(binding.get("world_entity_id"), "binding.world_entity_id")
    if (
        binding.get("physics_authority") != "mujoco_world_proxy"
        or binding.get("visual_mesh_is_physics_proxy") is not False
    ):
        raise ValueError("physics authority must remain the MuJoCo world proxy")
    unreal = binding.get("unreal")
    expected_unreal = {
        "asset_path": _game_path(unreal.get("asset_path") if isinstance(unreal, Mapping) else None),
        "collision_profile": "NoCollision",
        "collision_enabled": False,
        "simulate_physics": False,
        "generate_overlap_events": False,
        "can_ever_affect_navigation": False,
    }
    if unreal != expected_unreal:
        raise ValueError("Unreal binding must remain visual-only and NoCollision")
    outputs = document.get("outputs")
    directory = outputs.get("directory", "") if isinstance(outputs, Mapping) else ""
    directory = _portable_path(directory, "outputs.directory")
    if "/" in directory:
        raise ValueError("outputs.directory must be one direct child directory")
    if outputs != _outputs(asset_id, asset_class, directory):
        raise ValueError("outputs do not match the deterministic forest naming policy")
    qualification = document.get("qualification")
    if qualification != {
        "state": "QUARANTINED", "blockers": _BLOCKERS, "promotion_target": "WorldPackage.visual facet"
    }:
        raise ValueError("qualification must preserve forest candidate quarantine")
    return cast(dict[str, Any], json.loads(json.dumps(document, allow_nan=False)))


def write_forest_asset_conditioning_plan(path: Path, plan: Mapping[str, Any]) -> Path:
    """Validate and write a deterministic forest plan without running a DCC."""

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise FileExistsError(f"conditioning plan already exists: {target}")
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=target.parent, prefix=".plan-", suffix=".json", delete=False
    ) as stream:
        stream.write(
            json.dumps(validate_forest_asset_conditioning_plan(plan), sort_keys=True, indent=2)
            + "\n"
        )
        stream.flush()
        os.fsync(stream.fileno())
        temporary = Path(stream.name)
    try:
        _atomic_publish_no_replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
    return target


def build_forest_blender_conditioning_command(
    blender_executable: str | Path,
    *,
    repo_root: Path,
    plan_path: Path,
) -> list[str]:
    """Build a command for a script implementing ``CONDITIONER_CONTRACT``.

    No compatible materializer is claimed in this module.  In particular, the
    repository's generic static-prop script is rejected because it hard-codes
    1.0/0.5/0.2 LOD ratios.  A compatible script must validate this forest
    plan schema and accept ``--plan <path>`` after Blender's ``--`` separator.
    """

    repository = _link_free_absolute(repo_root, "repo_root")
    script = _link_free_absolute(
        repository / "sim/tools/assets/blender_forest_asset_conditioner.py",
        "repository forest conditioner",
    )
    plan = _link_free_absolute(plan_path, "plan_path")
    if not script.is_file():
        raise ValueError("repository forest conditioner is missing")
    if not plan.is_file():
        raise ValueError("plan_path must be one regular, link-free file")
    plan_bytes = _read_bound_file(
        plan, maximum_bytes=MAX_TASK_JSON_BYTES, context="plan_path"
    )
    raw = json.loads(plan_bytes.decode("utf-8"))
    validate_forest_asset_conditioning_plan(raw)
    return [
        str(Path(blender_executable)), "--factory-startup", "--background", "--disable-autoexec",
        "--python-exit-code", "1", "--python", str(script), "--", "--plan", str(plan),
        "--plan-bytes", str(len(plan_bytes)), "--plan-sha256", hashlib.sha256(plan_bytes).hexdigest(),
        "--conditioner-contract", CONDITIONER_CONTRACT,
        "--script-sha256", _sha256_file(script),
    ]


__all__ = [
    "CONDITIONER_CONTRACT",
    "PLAN_SCHEMA",
    "asset_class_from_id",
    "build_forest_asset_conditioning_plan",
    "build_forest_blender_conditioning_command",
    "validate_forest_asset_conditioning_plan",
    "write_forest_asset_conditioning_plan",
]
