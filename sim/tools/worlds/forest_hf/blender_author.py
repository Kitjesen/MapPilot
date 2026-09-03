"""Build the deterministic, textured Forest_HF presentation scene in Blender.

MuJoCo remains the only physics and collision authority.  Every mesh authored by
this module is presentation-only and carries explicit ``VisualOnly`` /
``NoCollision`` metadata.  The manifest describes separate coarse physics
proxies for downstream comparison; it never promotes Blender or Unreal meshes
to collision authority.

The module intentionally imports without Blender.  Recipe validation, layout
generation, manifest construction, and ``--validate-only`` are ordinary Python
operations.  Scene authoring is enabled only under Blender's Python runtime.
"""

from __future__ import annotations

import argparse
import array
import base64
import binascii
import hashlib
import itertools
import json
import math
import re
import shutil
import stat
import struct
import sys
import tempfile
import urllib.parse
import uuid
from contextlib import contextmanager
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any, Mapping, Sequence

# Blender executes ``--python`` files as standalone scripts, so the repository
# root is not guaranteed to be on ``sys.path`` even when the process starts in
# the workspace.  Bootstrap only this checked-in root before importing the
# package-local visual helpers; ordinary module imports remain unchanged.
_REPO_IMPORT_ROOT = Path(__file__).resolve().parents[4]
if str(_REPO_IMPORT_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_IMPORT_ROOT))

from sim.tools.worlds.forest_hf.blender_visuals import (
    build_visual_templates,
    create_review_cameras,
    hero_patrol_composition,
    inspection_marker_placements,
    procedural_asset_specs,
    procedural_material_specs,
    review_camera_specs,
    review_robot_grounding_offset,
    setup_morning_fog,
)

try:  # Blender is optional for contract tests and offline validation.
    import bpy  # type: ignore[import-not-found]
except ModuleNotFoundError:  # pragma: no cover - normal Python path.
    bpy = None


BPY_AVAILABLE = bpy is not None
WORLD_PACKAGE = "forest_hf@2.0.0"
DEFAULT_SEED = 20260813
DEFAULT_RECIPE = Path("sim/packages/worlds/forest_hf/2.0.0/terrain.recipe.json")
DEFAULT_OUTPUT_DIR = Path("build/forest-hf/blender")
_SCHEMA = "lingtu.sim.forest-authoring-recipe.v1"
_MANIFEST_SCHEMA = "lingtu.sim.blender-authoring-manifest.v1"
_SAFE_ID = re.compile(r"^[a-z][a-z0-9_-]{0,63}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_UNREAL_FBX_EXPORT_SCALE = 1.0
_UNREAL_COORDINATE_SCALE = 100.0

_FALLBACK_ASSET_SLOTS = (
    ("forest.asset.birch", "LT_Forest_Asset_Birch", "birch", (2.6, 2.6, 6.0), "cylindrical"),
    ("forest.asset.boulder", "LT_Forest_Asset_Boulder", "boulder", (1.6, 1.3, 1.1), "smart_project"),
    ("forest.asset.pine", "LT_Forest_Asset_Pine", "pine", (3.6, 3.6, 7.0), "cylindrical"),
)
_EXTERNAL_ASSET_SLOT_NAMES = {
    slot_id: (semantic_class, object_name)
    for slot_id, object_name, semantic_class, _, _ in _FALLBACK_ASSET_SLOTS
}
_EXTERNAL_ASSET_SLOT_IDS = {
    semantic_class: slot_id for slot_id, (semantic_class, _) in _EXTERNAL_ASSET_SLOT_NAMES.items()
}


class _ExternalAssetRecord(dict[str, Any]):
    """Serializable slot identity with a non-serialized validated source path."""

    def __init__(self, identity: Mapping[str, Any], source_path: Path) -> None:
        super().__init__(identity)
        self.source_path = source_path


def _asset_slot_export_filename(semantic_class: str) -> str:
    return f"assets/forest_asset_{semantic_class}.glb"


def _asset_slot_export_role(semantic_class: str) -> str:
    return f"asset_slot_{semantic_class}_glb"


def _asset_slot_unreal_filename(semantic_class: str) -> str:
    return f"assets/forest_asset_{semantic_class}.fbx"


def _asset_slot_unreal_role(semantic_class: str) -> str:
    return f"asset_slot_{semantic_class}_fbx"


def _unreal_fbx_export_kwargs(*, use_selection: bool) -> dict[str, Any]:
    """Preserve metre-valued FBX geometry for Unreal's metres-to-cm importer conversion."""

    return {
        "use_selection": use_selection,
        "global_scale": _UNREAL_FBX_EXPORT_SCALE,
        "apply_unit_scale": False,
        "apply_scale_options": "FBX_SCALE_NONE",
        "axis_forward": "-Z",
        "axis_up": "Y",
        "use_mesh_modifiers": True,
        "add_leaf_bones": False,
        "bake_anim": False,
    }


def _canonical_json(value: object) -> bytes:
    return (
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


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def digest_document(value: object) -> str:
    """Hash compact canonical JSON without importing repository dependencies."""

    payload = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return _sha256(payload)


def build_fallback_asset_contract() -> dict[str, Any]:
    """Describe replaceable procedural asset slots without claiming vendor provenance."""

    slots = [
        {
            "slot_id": slot_id,
            "object_name": object_name,
            "semantic_class": semantic_class,
            "source": {
                "kind": "procedural",
                "generator": "lingtu_blender_fallback",
            },
            "normalization": {
                "units": "metres",
                "up_axis": "+Z",
                "forward_axis": "+Y",
                "origin": "grounded_base_center",
                "rotation_deg": [0.0, 0.0, 0.0],
                "reference_size_m": list(reference_size),
                "apply_transforms": True,
            },
            "topology": {
                "deterministic": True,
                "generator_version": 1,
                "profile": f"forest_{semantic_class}_lowpoly_v1",
            },
            "uv": {
                "map_name": "UVMap",
                "projection": uv_projection,
                "deterministic": True,
            },
            "pbr": {
                "shader": "principled_pbr",
                "channels": {
                    "base_color": {"color_space": "sRGB"},
                    "normal": {"color_space": "Non-Color"},
                    "roughness": {"color_space": "Non-Color"},
                },
            },
            "replacement": {
                "accepts": "glb",
                "preserve": ["slot_id", "object_name"],
            },
            "export": {
                "path": _asset_slot_export_filename(semantic_class),
                "artifact_role": _asset_slot_export_role(semantic_class),
            },
            "unreal_export": {
                "path": _asset_slot_unreal_filename(semantic_class),
                "artifact_role": _asset_slot_unreal_role(semantic_class),
                "source_units": "metres",
                "target_units": "centimetres",
                "export_global_scale": _UNREAL_FBX_EXPORT_SCALE,
                "coordinate_scale": _UNREAL_COORDINATE_SCALE,
            },
        }
        for slot_id, object_name, semantic_class, reference_size, uv_projection in _FALLBACK_ASSET_SLOTS
    ]
    body = {
        "schema": "lingtu.sim.forest-asset-library.v1",
        "world_package": WORLD_PACKAGE,
        "slots": slots,
    }
    return {**body, "digest": digest_document(body)}


def _number(value: object, field: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{field} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    if positive and result <= 0.0:
        raise ValueError(f"{field} must be positive")
    return result


def _integer(value: object, field: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{field} must be an integer")
    if value < minimum:
        raise ValueError(f"{field} must be at least {minimum}")
    return value


def _pair(value: object, field: str, *, positive: bool = False) -> tuple[float, float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != 2:
        raise TypeError(f"{field} must contain two numbers")
    return (
        _number(value[0], f"{field}[0]", positive=positive),
        _number(value[1], f"{field}[1]", positive=positive),
    )


def _relative_path(value: object, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise TypeError(f"{field} must be a non-empty relative path")
    normalized = value.replace("\\", "/")
    path = PurePosixPath(normalized)
    if (
        path.is_absolute()
        or re.match(r"^[A-Za-z]:", normalized)
        or ":" in normalized
        or any(part in {"", ".", ".."} for part in path.parts)
    ):
        raise ValueError(f"{field} must be a portable relative path")
    return path.as_posix()


def _is_reparse_point(path: Path) -> bool:
    """Return whether ``path`` is a Windows reparse point without following it."""

    attributes = getattr(path.lstat(), "st_file_attributes", 0)
    reparse_flag = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
    return bool(attributes & reparse_flag)


def _validated_regular_asset_path(package_root: Path, relative_path: str, field: str) -> Path:
    """Resolve a package-local asset while rejecting every link-like path component."""

    root = package_root.resolve(strict=True)
    if root.is_symlink() or _is_reparse_point(root) or not root.is_dir():
        raise ValueError(f"{field} package root must be a regular link-free directory")
    candidate = package_root.joinpath(*PurePosixPath(relative_path).parts)
    current = package_root
    for part in PurePosixPath(relative_path).parts:
        current = current / part
        try:
            mode = current.lstat().st_mode
        except FileNotFoundError as exc:
            raise FileNotFoundError(f"{field} does not exist: {relative_path}") from exc
        if stat.S_ISLNK(mode) or _is_reparse_point(current):
            raise ValueError(f"{field} must not contain symlinks or reparse points: {relative_path}")
    resolved = candidate.resolve(strict=True)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{field} must remain inside the recipe package") from exc
    if not stat.S_ISREG(candidate.lstat().st_mode):
        raise ValueError(f"{field} must be a regular file: {relative_path}")
    return resolved


def _read_json_object(path: Path, field: str) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{field} must be a readable UTF-8 JSON object") from exc
    if not isinstance(document, dict):
        raise TypeError(f"{field} must be a JSON object")
    return document


def load_review_robot_asset_index(index_path: Path | str) -> dict[str, Any]:
    """Validate the opt-in 21-part Thunder review asset package."""

    requested = Path(index_path)
    package_root = requested.parent
    index = _validated_regular_asset_path(package_root, requested.name, "review robot asset index")
    document = _read_json_object(index, "review robot asset index")
    if document.get("schema") != "lingtu.sim.fbx-asset-index.v1":
        raise ValueError("review robot asset index schema is unsupported")
    if document.get("source_axis") != "mujoco_rh_z_up_m":
        raise ValueError("review robot asset index must use mujoco_rh_z_up_m source axes")
    axis = document.get("fbx_axis")
    if not isinstance(axis, Mapping) or axis.get("forward") != "-Y" or axis.get("up") != "Z" or axis.get("unit") != "m":
        raise ValueError("review robot asset index has an unsupported FBX axis contract")
    raw_assets = document.get("assets")
    if not isinstance(raw_assets, list) or len(raw_assets) != 21:
        raise ValueError("review robot asset index must contain exactly 21 assets")

    assets: list[dict[str, Any]] = []
    names: set[str] = set()
    paths: set[str] = set()
    for index_number, raw in enumerate(raw_assets):
        field = f"review robot assets[{index_number}]"
        if not isinstance(raw, Mapping):
            raise TypeError(f"{field} must be an object")
        asset_name = raw.get("asset_name")
        if not isinstance(asset_name, str) or not asset_name:
            raise TypeError(f"{field}.asset_name must be a non-empty string")
        relative_fbx = _relative_path(raw.get("fbx"), f"{field}.fbx")
        if PurePosixPath(relative_fbx).suffix.casefold() != ".fbx":
            raise ValueError(f"{field}.fbx must identify an FBX file")
        folded_name = asset_name.casefold()
        folded_path = relative_fbx.casefold()
        if folded_name in names or folded_path in paths:
            raise ValueError("review robot asset names and FBX paths must be unique")
        names.add(folded_name)
        paths.add(folded_path)
        declared_bytes = _integer(raw.get("fbx_bytes"), f"{field}.fbx_bytes", minimum=1)
        declared_sha = raw.get("fbx_sha256")
        if not isinstance(declared_sha, str) or _SHA256.fullmatch(declared_sha) is None:
            raise ValueError(f"{field}.fbx_sha256 must be a lowercase sha256 digest")
        fbx_path = _validated_regular_asset_path(package_root, relative_fbx, f"{field}.fbx")
        payload = fbx_path.read_bytes()
        if len(payload) != declared_bytes:
            raise ValueError(f"{field}.fbx bytes do not match the asset index")
        if _sha256(payload) != declared_sha:
            raise ValueError(f"{field}.fbx sha256 does not match the asset index")
        assets.append(
            {
                **dict(raw),
                "fbx": relative_fbx,
                "fbx_path": fbx_path,
                "classification": "VisualOnly",
                "collision_profile": "NoCollision",
                "simulate_physics": False,
                "can_ever_affect_navigation": False,
            }
        )

    recipe_path = package_root / "thunderv4-runtime.recipe.json"
    projection_path = package_root / "robot.visual-projection.json"
    companion_presence = (recipe_path.exists(), projection_path.exists())
    recipe: dict[str, Any] | None = None
    projection: dict[str, Any] | None = None
    if any(companion_presence):
        if not all(companion_presence):
            raise ValueError("review robot package must contain both runtime recipe and visual projection")
        recipe = _read_json_object(
            _validated_regular_asset_path(package_root, recipe_path.name, "review robot runtime recipe"),
            "review robot runtime recipe",
        )
        projection = _read_json_object(
            _validated_regular_asset_path(package_root, projection_path.name, "review robot visual projection"),
            "review robot visual projection",
        )
        recipe_components = recipe.get("components")
        projection_components = projection.get("components")
        if not isinstance(recipe_components, list) or len(recipe_components) != 21:
            raise ValueError("review robot runtime recipe must contain exactly 21 components")
        if not isinstance(projection_components, list) or len(projection_components) != 21:
            raise ValueError("review robot visual projection must contain exactly 21 components")
        recipe_names = {item.get("asset_name") for item in recipe_components if isinstance(item, Mapping)}
        projection_names = {item.get("visual_id") for item in projection_components if isinstance(item, Mapping)}
        if recipe_names != {item["asset_name"] for item in assets} or projection_names != recipe_names:
            raise ValueError("review robot index, runtime recipe, and visual projection disagree")
        recipe_by_name = {item["asset_name"]: item for item in recipe_components}
        projection_by_name = {item["visual_id"]: item for item in projection_components}
        for asset in assets:
            asset["runtime_component"] = recipe_by_name[asset["asset_name"]]
            asset["visual_projection"] = projection_by_name[asset["asset_name"]]

    return {
        "schema": "lingtu.sim.review-robot-assets.v1",
        "source_index": index,
        "package_root": package_root.resolve(),
        "assets": assets,
        "runtime_recipe": recipe,
        "visual_projection": projection,
    }


def _normalize_external_asset_slots(raw: object, recipe_path: Path) -> dict[str, dict[str, Any]]:
    """Validate optional conditioned GLB overrides and return canonical slot records."""

    if raw is None:
        return {}
    if not isinstance(raw, Mapping):
        raise TypeError("external_asset_slots must be an object")
    normalized: dict[str, dict[str, Any]] = {}
    for raw_slot, value in raw.items():
        if not isinstance(raw_slot, str):
            raise TypeError("external_asset_slots keys must be strings")
        slot_id = _EXTERNAL_ASSET_SLOT_IDS.get(raw_slot, raw_slot)
        if slot_id not in _EXTERNAL_ASSET_SLOT_NAMES:
            raise ValueError(f"unsupported external asset slot: {raw_slot}")
        if slot_id in normalized:
            raise ValueError(f"duplicate external asset slot: {slot_id}")
        if not isinstance(value, Mapping):
            raise TypeError(f"external_asset_slots.{raw_slot} must be an object")
        try:
            relative_path = _relative_path(value.get("path"), f"external_asset_slots.{raw_slot}.path")
        except ValueError as exc:
            raise ValueError(
                f"external_asset_slots.{raw_slot}.path must remain contained in the artifact root"
            ) from exc
        if PurePosixPath(relative_path).suffix.lower() != ".glb":
            raise ValueError(f"external_asset_slots.{raw_slot}.path must identify a GLB file")
        expected_bytes = _integer(value.get("bytes"), f"external_asset_slots.{raw_slot}.bytes", minimum=1)
        expected_sha256 = value.get("sha256")
        if not isinstance(expected_sha256, str) or _SHA256.fullmatch(expected_sha256) is None:
            raise ValueError(f"external_asset_slots.{raw_slot}.sha256 must be a lowercase sha256 digest")
        source_path = _validated_regular_asset_path(
            recipe_path.parent,
            relative_path,
            f"external_asset_slots.{raw_slot}.path",
        )
        payload = source_path.read_bytes()
        if len(payload) != expected_bytes:
            raise ValueError(f"external_asset_slots.{raw_slot}.bytes does not match {relative_path}")
        if _sha256(payload) != expected_sha256:
            raise ValueError(f"external_asset_slots.{raw_slot}.sha256 does not match {relative_path}")
        semantic_class, _ = _EXTERNAL_ASSET_SLOT_NAMES[slot_id]
        embed_depth = value.get("embed_depth_m")
        normalized_identity: dict[str, Any] = {
            "path": relative_path,
            "bytes": expected_bytes,
            "sha256": expected_sha256,
        }
        if embed_depth is not None:
            normalized_embed_depth = _number(
                embed_depth,
                f"external_asset_slots.{raw_slot}.embed_depth_m",
            )
            if not 0.0 <= normalized_embed_depth <= 0.5:
                raise ValueError(
                    f"external_asset_slots.{raw_slot}.embed_depth_m must be between 0.0 and 0.5"
                )
            normalized_identity["embed_depth_m"] = normalized_embed_depth
        normalized[semantic_class] = _ExternalAssetRecord(
            normalized_identity,
            source_path,
        )
    return normalized


def _external_slot_embed_depth(recipe: Mapping[str, Any], kind: str) -> float:
    """Return the conditioned asset's authored ground embed for one instance kind."""

    semantic_class = "boulder" if kind == "rock" else kind
    slots = recipe.get("external_asset_slots")
    if not isinstance(slots, Mapping):
        return 0.0
    slot = slots.get(semantic_class)
    if not isinstance(slot, Mapping):
        return 0.0
    value = slot.get("embed_depth_m", 0.0)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"external_asset_slots.{semantic_class}.embed_depth_m must be a number")
    depth = float(value)
    if not math.isfinite(depth) or not 0.0 <= depth <= 0.5:
        raise ValueError(
            f"external_asset_slots.{semantic_class}.embed_depth_m must be finite and between 0.0 and 0.5"
        )
    return depth


def asset_slot_position_m(
    position_m: Sequence[float],
    semantic_class: str,
    external_asset_slots: Mapping[str, Any],
) -> list[float]:
    """Return an instance position with its conditioned visual embed applied."""

    if isinstance(position_m, (str, bytes)) or len(position_m) != 3:
        raise TypeError("position_m must contain xyz")
    position = [float(value) for value in position_m]
    position[2] -= _external_slot_embed_depth(
        {"external_asset_slots": external_asset_slots},
        semantic_class,
    )
    return position


def _stable_unit(seed: int, namespace: str, index: int) -> float:
    payload = f"{seed}:{namespace}:{index}".encode()
    raw = int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")
    return raw / float(2**64 - 1)


def _stable_range(seed: int, namespace: str, index: int, low: float, high: float) -> float:
    return low + (high - low) * _stable_unit(seed, namespace, index)


def _tree_asset_slot_id(species_id: str) -> str:
    """Map recipe species onto the stable fallback library without changing species IDs."""

    if species_id in {"pine", "pine_a"}:
        return "forest.asset.pine"
    if species_id in {"birch", "birch_a"}:
        return "forest.asset.birch"
    raise ValueError(f"species {species_id!r} has no fallback asset slot")


def ue_obj_cm_lh_to_blender_m_rh(vertex: Sequence[float]) -> tuple[float, float, float]:
    """Convert a UE centimetre/left-handed OBJ vertex to Blender RH metres."""

    if len(vertex) != 3:
        raise ValueError("terrain vertex must contain xyz")
    x, y, z = (_number(value, f"terrain vertex[{index}]") for index, value in enumerate(vertex))
    return (x * 0.01, y * -0.01, z * 0.01)


def blender_terrain_preview_dimensions(
    source_width: int = 513,
    source_height: int = 513,
) -> tuple[int, int]:
    """Return source-aware editable preview dimensions capped at 513 samples."""

    return (min(513, source_width), min(513, source_height))


def _distance_to_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.dist(point, start)
    projection = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    ratio = min(1.0, max(0.0, projection))
    closest = (start[0] + ratio * dx, start[1] + ratio * dy)
    return math.dist(point, closest)


def _height_at(x: float, y: float, seed: int) -> float:
    """Return the v1 authoritative ground plane height.

    Terrain variation is carried by the project-owned normal map only.  Mesh
    displacement would diverge from ``physics/forest_hf.xml`` and is therefore
    intentionally forbidden until a same-source MuJoCo heightfield is added.
    """

    del x, y, seed
    return 0.0


def _choose_weighted_species(species: Sequence[Mapping[str, Any]], unit: float) -> Mapping[str, Any]:
    total = sum(float(item["weight"]) for item in species)
    cursor = unit * total
    for item in species:
        cursor -= float(item["weight"])
        if cursor <= 0.0:
            return item
    return species[-1]


def _validate_texture(texture: object, field: str) -> dict[str, Any]:
    if not isinstance(texture, Mapping):
        raise TypeError(f"{field} must be an object")
    path = _relative_path(texture.get("path"), f"{field}.path")
    sha256 = texture.get("sha256")
    if not isinstance(sha256, str) or _SHA256.fullmatch(sha256) is None:
        raise ValueError(f"{field}.sha256 must be 64 lowercase hexadecimal characters")
    source = texture.get("source")
    license_name = texture.get("license")
    if not isinstance(source, str) or not source.strip():
        raise ValueError(f"{field}.source is required")
    if not isinstance(license_name, str) or not license_name.strip():
        raise ValueError(f"{field}.license is required")
    return {
        "path": path,
        "sha256": sha256,
        "source": source.strip(),
        "license": license_name.strip(),
    }


def _production_materials(raw: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Project the full package recipe's procedural definitions into PBR provenance."""

    materials = raw.get("materials")
    if not isinstance(materials, Mapping):
        raise TypeError("materials must be an object")
    provenance = materials.get("provenance")
    definitions = materials.get("definitions")
    if not isinstance(provenance, Mapping) or not isinstance(definitions, Mapping):
        raise TypeError("materials provenance and definitions must be objects")
    owner = provenance.get("owner")
    license_name = provenance.get("license")
    source_uri = provenance.get("source_uri")
    if owner != "Inovxio" or license_name != "project-owned":
        raise ValueError("production forest materials must be project-owned by Inovxio")
    if not isinstance(source_uri, str) or not source_uri.startswith("repo://"):
        raise ValueError("production forest material source_uri must be repo-relative")
    normalized = []
    for material_id, definition in sorted(definitions.items()):
        if not isinstance(material_id, str) or _SAFE_ID.fullmatch(material_id) is None:
            raise ValueError(f"invalid production material id: {material_id!r}")
        if not isinstance(definition, Mapping):
            raise TypeError(f"materials.definitions.{material_id} must be an object")
        definition_digest = _sha256(_canonical_json(dict(definition)))
        texture_root = f"generated/textures/{material_id}"
        textures = {}
        for channel, suffix in (
            ("base_color", "basecolor"),
            ("normal", "normal"),
            ("orm", "orm"),
        ):
            textures[channel] = {
                "path": f"{texture_root}_{suffix}.png",
                "sha256": definition_digest,
                "source": f"{source_uri}#{material_id}:{channel}",
                "license": "LicenseRef-LingTu-Project-Owned",
            }
        normalized.append(
            {
                "id": material_id,
                "shader": "principled_pbr",
                "textures": textures,
            }
        )
    return normalized


def _normalize_production_recipe(raw: Mapping[str, Any], recipe_path: Path) -> dict[str, Any]:
    """Normalize the checked-in rich Forest_HF recipe to the public layout API."""

    if raw.get("package") != WORLD_PACKAGE:
        raise ValueError(f"forest recipe package must be {WORLD_PACKAGE}")
    coordinates = raw.get("coordinate_contract")
    scene = raw.get("scene_design")
    population = raw.get("deterministic_population")
    runtime = raw.get("runtime_contract")
    if not all(isinstance(item, Mapping) for item in (coordinates, scene, population, runtime)):
        raise TypeError("production recipe coordinate, scene, population, and runtime contracts are required")
    if coordinates.get("source_frame") != "mujoco_rh_z_up_m":
        raise ValueError("production recipe source frame must be mujoco_rh_z_up_m")
    full_extent = _pair(coordinates.get("extent_m"), "coordinate_contract.extent_m", positive=True)
    extent = [full_extent[0] / 2.0, full_extent[1] / 2.0]
    trail = scene.get("trail")
    if not isinstance(trail, Mapping):
        raise TypeError("scene_design.trail must be an object")
    raw_points = trail.get("control_points_m")
    if not isinstance(raw_points, Sequence) or isinstance(raw_points, (str, bytes)) or len(raw_points) < 2:
        raise ValueError("scene_design.trail.control_points_m must contain at least two points")
    centerline = []
    for index, point in enumerate(raw_points):
        if not isinstance(point, Sequence) or isinstance(point, (str, bytes)) or len(point) != 3:
            raise TypeError(f"scene_design.trail.control_points_m[{index}] must contain xyz")
        centerline.append(
            [
                _number(point[0], f"trail.control_points_m[{index}][0]"),
                _number(point[1], f"trail.control_points_m[{index}][1]"),
            ]
        )
    preview_counts = population.get("preview_counts")
    raw_species = population.get("species_mix")
    if not isinstance(preview_counts, Mapping) or not isinstance(raw_species, Sequence):
        raise TypeError("production population preview_counts and species_mix are required")
    tree_count = _integer(preview_counts.get("canopy_trees"), "preview_counts.canopy_trees", minimum=1)
    species = []
    for index, item in enumerate(raw_species):
        if not isinstance(item, Mapping):
            raise TypeError(f"species_mix[{index}] must be an object")
        species_id = str(item.get("id", ""))
        height = _pair(item.get("height_m"), f"species_mix[{index}].height_m", positive=True)
        crown_by_species = {
            "pine_a": (1.5, 2.6),
            "birch_a": (1.25, 2.2),
            "oak_a": (2.0, 3.4),
        }
        species.append(
            {
                "id": species_id,
                "weight": _number(item.get("weight"), f"species_mix[{index}].weight", positive=True),
                "height_m": list(height),
                "crown_radius_m": list(crown_by_species.get(species_id, (1.4, 2.5))),
                "material": "bark_mixed",
            }
        )
    raw_clearings = scene.get("clearings", [])
    if not isinstance(raw_clearings, Sequence) or isinstance(raw_clearings, (str, bytes)):
        raise TypeError("scene_design.clearings must be an array")
    exclusion_zones = []
    for index, item in enumerate(raw_clearings):
        if not isinstance(item, Mapping):
            raise TypeError(f"scene_design.clearings[{index}] must be an object")
        center = item.get("center_m")
        if not isinstance(center, Sequence) or isinstance(center, (str, bytes)) or len(center) != 3:
            raise TypeError(f"scene_design.clearings[{index}].center_m must contain xyz")
        exclusion_zones.append(
            {
                "id": str(item.get("id", f"clearing-{index:02d}")),
                "center_m": [float(center[0]), float(center[1])],
                "radius_m": _number(item.get("radius_m"), f"clearings[{index}].radius_m", positive=True),
            }
        )
    visual_defaults = runtime.get("visual_defaults")
    proxy_policy = runtime.get("physics_proxy_policy")
    if not isinstance(visual_defaults, Mapping) or not isinstance(proxy_policy, Mapping):
        raise TypeError("runtime visual defaults and physics proxy policy are required")
    if (
        visual_defaults.get("classification") != "VisualOnly"
        or visual_defaults.get("collision_profile") != "NoCollision"
        or visual_defaults.get("collision_enabled") is not False
        or proxy_policy.get("authority") != "mujoco"
        or proxy_policy.get("render_mesh_is_collider") is not False
    ):
        raise ValueError("production runtime contract must remain VisualOnly, NoCollision, and MuJoCo-authoritative")
    terrain = scene.get("terrain", {})
    if not isinstance(terrain, Mapping) or terrain.get("physics_surface") != "flat_plane_z0":
        raise ValueError("production Forest_HF v1 requires the authoritative flat_plane_z0 terrain")
    return {
        "schema": _SCHEMA,
        "source_schema": "lingtu.sim.blender-forest-recipe.v1",
        "world_package": WORLD_PACKAGE,
        "seed": _integer(raw.get("seed", DEFAULT_SEED), "seed"),
        "extent_m": extent,
        "tree_count": tree_count,
        "trail": {
            "centerline_m": centerline,
            "clearance_radius_m": _number(
                trail.get("vegetation_exclusion_radius_m"),
                "trail.vegetation_exclusion_radius_m",
                positive=True,
            ),
            "surface_width_m": 3.2,
            "shoulder_blend_m": 0.8,
        },
        "exclusion_zones": exclusion_zones,
        "terrain_height_mode": "flat",
        "species": species,
        "materials": _production_materials(raw),
        "hard_rules": {
            "classification": "VisualOnly",
            "collision_profile": "NoCollision",
            "physics_authority": "mujoco",
        },
        "external_asset_slots": _normalize_external_asset_slots(raw.get("external_asset_slots"), recipe_path),
        **(
            {"external_asset_root": str(recipe_path.parent.resolve())}
            if raw.get("external_asset_slots")
            else {}
        ),
        "recipe_sha256": _sha256(_canonical_json(dict(raw))),
    }


def _normalize_terrain_recipe(raw: Mapping[str, Any], recipe_path: Path) -> dict[str, Any]:
    """Project the v2 terrain/route contracts into the Blender authoring API."""

    if raw.get("world_package") != WORLD_PACKAGE:
        raise ValueError(f"terrain recipe world_package must be {WORLD_PACKAGE}")
    authority = raw.get("authority")
    if not isinstance(authority, Mapping) or authority.get("physics") != "mujoco":
        raise ValueError("terrain recipe must retain MuJoCo physics authority")
    if authority.get("unreal") != "visual_only_no_collision":
        raise ValueError("terrain recipe must retain visual-only Unreal authority")
    unreal_facet = raw.get("facets", {}).get("unreal", {})
    if not isinstance(unreal_facet, Mapping) or unreal_facet.get("format") != "ue_landscape_r16_png":
        raise ValueError("terrain recipe must define a ue_landscape_r16_png Unreal facet")
    full_extent = _pair(raw.get("extent_m"), "extent_m", positive=True)
    route_path = recipe_path.parent / "routes" / "forest.routes.json"
    routes = json.loads(route_path.read_text(encoding="utf-8"))
    if not isinstance(routes, Mapping) or routes.get("world_package") != WORLD_PACKAGE:
        raise ValueError("v2 route contract must match the terrain world package")
    route_items = routes.get("routes")
    if not isinstance(route_items, Sequence) or not route_items:
        raise ValueError("v2 route contract must define at least one route")
    primary = next(
        (item for item in route_items if isinstance(item, Mapping) and item.get("kind") == "primary_loop"),
        None,
    )
    if not isinstance(primary, Mapping):
        raise ValueError("v2 route contract must define a primary_loop")
    centerline_raw = primary.get("centerline_xy_m")
    if not isinstance(centerline_raw, Sequence) or len(centerline_raw) < 2:
        raise ValueError("v2 primary route must define a centerline")
    centerline = [list(_pair(point, f"routes.primary.centerline[{index}]")) for index, point in enumerate(centerline_raw)]
    closed_loop = len(centerline) > 2 and math.dist(centerline[0], centerline[-1]) <= 1e-6
    if closed_loop:
        centerline = centerline[:-1]
    road_width = _number(primary.get("road_width_m"), "routes.primary.road_width_m", positive=True)
    corridors = []
    for route_index, route in enumerate(route_items):
        if not isinstance(route, Mapping):
            raise TypeError(f"routes[{route_index}] must be an object")
        raw_route_points = route.get("centerline_xy_m")
        if not isinstance(raw_route_points, Sequence) or len(raw_route_points) < 2:
            raise ValueError(f"routes[{route_index}] must define a centerline")
        route_width = _number(route.get("road_width_m"), f"routes[{route_index}].road_width_m", positive=True)
        limits = route.get("navigation_limits", {})
        side_clearance = (
            _number(limits.get("clearance_each_side_m"), f"routes[{route_index}].clearance_each_side_m")
            if isinstance(limits, Mapping) and limits.get("clearance_each_side_m") is not None
            else 0.0
        )
        corridors.append(
            {
                "stable_id": str(route.get("stable_id", f"forest.route.{route_index:02d}")),
                "centerline_m": [
                    list(_pair(point, f"routes[{route_index}].centerline[{point_index}]"))
                    for point_index, point in enumerate(raw_route_points)
                ],
                "clearance_radius_m": route_width / 2.0 + side_clearance,
            }
        )
    contract_digest = raw.get("production_contract_sha256")
    if not isinstance(contract_digest, str) or _SHA256.fullmatch(contract_digest) is None:
        raise ValueError("terrain production_contract_sha256 must be a sha256 digest")

    materials = []
    for material_id in ("forest_bark", "forest_leaves"):
        textures = {
            channel: {
                "path": f"textures/forest/{material_id}_{suffix}.png",
                "sha256": contract_digest,
                "source": f"repo://sim/tools/worlds/forest_hf/blender_author.py#{material_id}:{channel}",
                "license": "LicenseRef-LingTu-Project-Owned",
            }
            for channel, suffix in (
                ("base_color", "basecolor"),
                ("normal", "normal"),
                ("roughness", "orm"),
            )
        }
        materials.append({"id": material_id, "shader": "principled_pbr", "textures": textures})
    return {
        "schema": _SCHEMA,
        "source_schema": "lingtu.sim.forest-terrain-recipe.v1",
        "world_package": WORLD_PACKAGE,
        "seed": _integer(raw.get("seed", DEFAULT_SEED), "seed"),
        "extent_m": [full_extent[0] / 2.0, full_extent[1] / 2.0],
        "tree_count": 1200,
        "trail": {
            "centerline_m": centerline,
            "clearance_radius_m": road_width / 2.0 + 2.0,
            "surface_width_m": 3.2,
            "shoulder_blend_m": 0.8,
            "closed_loop": closed_loop,
        },
        "route_corridors": corridors,
        "exclusion_zones": [
            {
                "id": str(routes[zone_name].get("stable_id", zone_name)),
                "center_m": list(_pair(routes[zone_name].get("position_xy_m"), f"routes.{zone_name}.position_xy_m")),
                "radius_m": _number(
                    routes[zone_name].get(radius_field),
                    f"routes.{zone_name}.{radius_field}",
                    positive=True,
                ),
            }
            for zone_name, radius_field in (("spawn", "minimum_clear_radius_m"), ("goal", "acceptance_radius_m"))
            if isinstance(routes.get(zone_name), Mapping)
        ],
        "terrain_source": {
            "extent_m": list(full_extent),
            "canonical_source": _relative_path(raw.get("canonical_source"), "canonical_source"),
            **(
                {"mesh": _relative_path(unreal_facet["mesh"], "facets.unreal.mesh")}
                if unreal_facet.get("mesh") is not None
                else {}
            ),
            "unreal_heightmap": _relative_path(unreal_facet.get("heightmap"), "facets.unreal.heightmap"),
            "unreal_format": "ue_landscape_r16_png",
            "asset_manifest": "generated/asset-manifest.json",
            "enforce_asset_manifest": True,
            "package_root": str(recipe_path.parent.resolve()),
            "dimensions_px": [
                _integer(value, f"source_grid.dimensions_px[{index}]", minimum=2)
                for index, value in enumerate(raw.get("source_grid", {}).get("dimensions_px", []))
            ],
            "sample_type": str(raw.get("source_grid", {}).get("sample_type")),
            "endianness": str(raw.get("source_grid", {}).get("endianness")),
            "row_order": str(raw.get("source_grid", {}).get("row_order")),
            "elevation_range_m": list(_pair(raw.get("elevation_range_m"), "elevation_range_m")),
            "production_contract_sha256": contract_digest,
            "route_contract_sha256": routes.get("route_contract_sha256"),
        },
        "terrain_height_mode": "canonical_package",
        "species": [
            {
                "id": "pine",
                "weight": 0.7,
                "height_m": [6.0, 11.0],
                "crown_radius_m": [1.5, 2.6],
                "material": "forest_bark",
            },
            {
                "id": "birch",
                "weight": 0.3,
                "height_m": [5.0, 9.0],
                "crown_radius_m": [1.25, 2.2],
                "material": "forest_bark",
            },
        ],
        "materials": materials,
        "hard_rules": {
            "classification": "VisualOnly",
            "collision_profile": "NoCollision",
            "physics_authority": "mujoco",
        },
        "external_asset_slots": _normalize_external_asset_slots(raw.get("external_asset_slots"), recipe_path),
        **(
            {"external_asset_root": str(recipe_path.parent.resolve())}
            if raw.get("external_asset_slots")
            else {}
        ),
        "recipe_sha256": _sha256(_canonical_json(dict(raw))),
    }


def load_forest_recipe(path: Path | str) -> dict[str, Any]:
    """Load and validate one portable deterministic Forest_HF recipe."""

    recipe_path = Path(path)
    raw = json.loads(recipe_path.read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping):
        raise TypeError("forest recipe root must be an object")
    if raw.get("schema") == "lingtu.sim.blender-forest-recipe.v1":
        return _normalize_production_recipe(raw, recipe_path)
    if raw.get("schema") == "lingtu.sim.forest-terrain-recipe.v1":
        return _normalize_terrain_recipe(raw, recipe_path)
    if raw.get("schema") != _SCHEMA:
        raise ValueError(f"forest recipe schema must be {_SCHEMA}")
    if raw.get("world_package") != WORLD_PACKAGE:
        raise ValueError(f"forest recipe world_package must be {WORLD_PACKAGE}")

    seed = _integer(raw.get("seed", DEFAULT_SEED), "seed")
    extent_x, extent_y = _pair(raw.get("extent_m"), "extent_m", positive=True)
    tree_count = _integer(raw.get("tree_count"), "tree_count", minimum=1)

    raw_trail = raw.get("trail")
    if not isinstance(raw_trail, Mapping):
        raise TypeError("trail must be an object")
    raw_centerline = raw_trail.get("centerline_m")
    if isinstance(raw_centerline, (str, bytes)) or not isinstance(raw_centerline, Sequence) or len(raw_centerline) < 2:
        raise ValueError("trail.centerline_m must have at least two points")
    centerline = [list(_pair(point, f"trail.centerline_m[{index}]")) for index, point in enumerate(raw_centerline)]
    for index, (x, y) in enumerate(centerline):
        if abs(x) > extent_x or abs(y) > extent_y:
            raise ValueError(f"trail.centerline_m[{index}] must stay inside extent_m")
    clearance = _number(
        raw_trail.get("clearance_radius_m"),
        "trail.clearance_radius_m",
        positive=True,
    )

    raw_species = raw.get("species")
    if isinstance(raw_species, (str, bytes)) or not isinstance(raw_species, Sequence) or not raw_species:
        raise ValueError("species must be a non-empty array")
    species: list[dict[str, Any]] = []
    species_ids: set[str] = set()
    for index, item in enumerate(raw_species):
        if not isinstance(item, Mapping):
            raise TypeError(f"species[{index}] must be an object")
        species_id = item.get("id")
        material = item.get("material")
        if not isinstance(species_id, str) or _SAFE_ID.fullmatch(species_id) is None:
            raise ValueError(f"species[{index}].id is not a stable identifier")
        if species_id in species_ids:
            raise ValueError(f"duplicate species id: {species_id}")
        if not isinstance(material, str) or _SAFE_ID.fullmatch(material) is None:
            raise ValueError(f"species[{index}].material is not a stable identifier")
        height = _pair(item.get("height_m"), f"species[{index}].height_m", positive=True)
        crown = _pair(item.get("crown_radius_m"), f"species[{index}].crown_radius_m", positive=True)
        if height[1] < height[0] or crown[1] < crown[0]:
            raise ValueError(f"species[{index}] ranges must be ordered")
        species_ids.add(species_id)
        species.append(
            {
                "id": species_id,
                "weight": _number(item.get("weight"), f"species[{index}].weight", positive=True),
                "height_m": list(height),
                "crown_radius_m": list(crown),
                "material": material,
            }
        )

    raw_materials = raw.get("materials")
    if isinstance(raw_materials, (str, bytes)) or not isinstance(raw_materials, Sequence) or not raw_materials:
        raise ValueError("materials must be a non-empty array")
    materials: list[dict[str, Any]] = []
    material_ids: set[str] = set()
    for index, item in enumerate(raw_materials):
        if not isinstance(item, Mapping):
            raise TypeError(f"materials[{index}] must be an object")
        material_id = item.get("id")
        if not isinstance(material_id, str) or _SAFE_ID.fullmatch(material_id) is None:
            raise ValueError(f"materials[{index}].id is not a stable identifier")
        if material_id in material_ids:
            raise ValueError(f"duplicate material id: {material_id}")
        if item.get("shader") != "principled_pbr":
            raise ValueError(f"materials[{index}].shader must be principled_pbr")
        raw_textures = item.get("textures")
        if not isinstance(raw_textures, Mapping):
            raise TypeError(f"materials[{index}].textures must be an object")
        required = {"base_color", "normal"}
        if not required.issubset(raw_textures) or not ({"roughness", "orm"} & set(raw_textures)):
            raise ValueError(f"materials[{index}] requires BaseColor, Normal, and Roughness/ORM provenance")
        textures = {
            name: _validate_texture(texture, f"materials[{index}].textures.{name}")
            for name, texture in sorted(raw_textures.items())
        }
        material_ids.add(material_id)
        materials.append(
            {
                "id": material_id,
                "shader": "principled_pbr",
                "textures": textures,
            }
        )
    missing_materials = {item["material"] for item in species} - material_ids
    if missing_materials:
        raise ValueError(f"species reference unknown materials: {sorted(missing_materials)}")

    hard_rules = raw.get("hard_rules")
    if not isinstance(hard_rules, Mapping):
        raise TypeError("hard_rules must be an object")
    if hard_rules.get("classification") != "VisualOnly":
        raise ValueError("hard_rules.classification must be VisualOnly")
    if hard_rules.get("collision_profile") != "NoCollision":
        raise ValueError("hard_rules.collision_profile must be NoCollision")
    if hard_rules.get("physics_authority") != "mujoco":
        raise ValueError("hard_rules.physics_authority must be mujoco")

    return {
        "schema": _SCHEMA,
        "world_package": WORLD_PACKAGE,
        "seed": seed,
        "extent_m": [extent_x, extent_y],
        "tree_count": tree_count,
        "trail": {
            "centerline_m": centerline,
            "clearance_radius_m": clearance,
            "surface_width_m": _number(
                raw_trail.get("surface_width_m", 3.2),
                "trail.surface_width_m",
                positive=True,
            ),
            "shoulder_blend_m": _number(
                raw_trail.get("shoulder_blend_m", 0.8),
                "trail.shoulder_blend_m",
                positive=True,
            ),
        },
        "species": species,
        "materials": materials,
        "hard_rules": {
            "classification": "VisualOnly",
            "collision_profile": "NoCollision",
            "physics_authority": "mujoco",
        },
        "exclusion_zones": [],
        "terrain_height_mode": "procedural",
        "external_asset_slots": _normalize_external_asset_slots(raw.get("external_asset_slots"), recipe_path),
        **(
            {"external_asset_root": str(recipe_path.parent.resolve())}
            if raw.get("external_asset_slots")
            else {}
        ),
        "recipe_sha256": _sha256(_canonical_json(dict(raw))),
    }


def _visual_metadata() -> dict[str, object]:
    return {
        "classification": "VisualOnly",
        "collision_profile": "NoCollision",
        "collision": False,
        "simulate_physics": False,
        "can_ever_affect_navigation": False,
        "physics_representation": "none",
    }


def _canonical_heightfield_path(recipe: Mapping[str, Any]) -> Path:
    source = recipe.get("terrain_source")
    if not isinstance(source, Mapping):
        raise ValueError("v2 canonical terrain_source is required")
    package_root = source.get("package_root")
    if not isinstance(package_root, str) or not package_root:
        raise ValueError("v2 canonical terrain package_root is required")
    root = Path(package_root)
    return root / str(source["canonical_source"])


def _asset_identity_digest(artifacts: Sequence[Mapping[str, Any]]) -> str:
    identity = [{"path": item["path"], "sha256": item["sha256"]} for item in artifacts]
    payload = (json.dumps(identity, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()
    return _sha256(payload)


def _load_canonical_asset_manifest(recipe: Mapping[str, Any]) -> Mapping[str, Any] | None:
    source = recipe["terrain_source"]
    if not source.get("enforce_asset_manifest", False):
        return None
    manifest_path = Path(str(source["package_root"])) / str(source["asset_manifest"])
    if not manifest_path.is_file():
        raise FileNotFoundError(f"canonical Forest_HF asset manifest is missing: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, Mapping):
        raise ValueError("canonical Forest_HF asset manifest must be an object")
    if manifest.get("schema") != "lingtu.sim.forest-asset-manifest.v1":
        raise ValueError("canonical Forest_HF asset manifest schema mismatch")
    if manifest.get("world_package") != WORLD_PACKAGE:
        raise ValueError("canonical Forest_HF asset manifest world package mismatch")
    if "seed" in recipe and manifest.get("seed") != recipe["seed"]:
        raise ValueError("canonical Forest_HF asset manifest seed mismatch")
    artifacts = manifest.get("artifacts") if isinstance(manifest, Mapping) else None
    if not isinstance(artifacts, Sequence):
        raise ValueError("canonical Forest_HF asset manifest has no artifacts array")
    normalized_artifacts: list[Mapping[str, Any]] = []
    for index, item in enumerate(artifacts):
        if not isinstance(item, Mapping):
            raise TypeError(f"canonical Forest_HF asset manifest artifacts[{index}] must be an object")
        path = _relative_path(item.get("path"), f"asset manifest artifacts[{index}].path")
        sha256 = item.get("sha256")
        if not isinstance(sha256, str) or _SHA256.fullmatch(sha256) is None:
            raise ValueError(f"asset manifest {path}.sha256 must be a sha256 digest")
        normalized_artifacts.append({**item, "path": path, "sha256": sha256})
    expected_asset_digest = manifest.get("asset_set_sha256")
    if not isinstance(expected_asset_digest, str) or _SHA256.fullmatch(expected_asset_digest) is None:
        raise ValueError("canonical Forest_HF asset_set_sha256 must be a sha256 digest")
    if _asset_identity_digest(normalized_artifacts) != expected_asset_digest:
        raise ValueError("canonical Forest_HF asset manifest identity digest mismatch")
    canonical = manifest.get("canonical_source")
    if not isinstance(canonical, Mapping):
        raise ValueError("canonical Forest_HF asset manifest has no canonical_source")
    canonical_path = _relative_path(canonical.get("path"), "asset manifest canonical_source.path")
    if canonical_path != source["canonical_source"]:
        raise ValueError("canonical Forest_HF source path drifted from the production recipe")
    dimensions = [
        _integer(value, f"asset manifest canonical_source.dimensions_px[{index}]", minimum=2)
        for index, value in enumerate(canonical.get("dimensions_px", []))
    ]
    if len(dimensions) != 2:
        raise ValueError("canonical Forest_HF manifest dimensions_px must contain two values")
    recipe_dimensions = list(source["dimensions_px"])
    if dimensions != recipe_dimensions and manifest.get("validation_resolution") is not True:
        raise ValueError("canonical Forest_HF resolution drift requires validation_resolution=true")
    extent = list(_pair(canonical.get("extent_m"), "asset manifest canonical_source.extent_m", positive=True))
    if extent != list(source["extent_m"]):
        raise ValueError("canonical Forest_HF extent drifted from the production recipe")
    if canonical.get("sample_type") != source.get("sample_type"):
        raise ValueError("canonical Forest_HF sample type drifted from the production recipe")
    if canonical.get("endianness") != source.get("endianness"):
        raise ValueError("canonical Forest_HF endianness drifted from the production recipe")
    return {**manifest, "artifacts": normalized_artifacts, "canonical_source": {**canonical, "dimensions_px": dimensions}}


def _verify_canonical_terrain_artifact(
    recipe: Mapping[str, Any],
    path: Path,
    relative_path: str,
    manifest: Mapping[str, Any] | None = None,
) -> None:
    if manifest is None:
        manifest = _load_canonical_asset_manifest(recipe)
    if manifest is None:
        return
    artifacts = manifest["artifacts"]
    record = next(
        (item for item in artifacts if isinstance(item, Mapping) and item.get("path") == relative_path),
        None,
    )
    if not isinstance(record, Mapping):
        raise ValueError(f"canonical Forest_HF asset manifest has no record for {relative_path}")
    payload = path.read_bytes()
    expected_bytes = _integer(record.get("bytes"), f"asset manifest {relative_path}.bytes")
    expected_sha256 = record.get("sha256")
    if not isinstance(expected_sha256, str) or _SHA256.fullmatch(expected_sha256) is None:
        raise ValueError(f"asset manifest {relative_path}.sha256 must be a sha256 digest")
    if len(payload) != expected_bytes or _sha256(payload) != expected_sha256:
        raise ValueError(f"canonical Forest_HF artifact identity mismatch: {relative_path}")


def _load_canonical_heightfield(recipe: Mapping[str, Any]) -> tuple[array.array[int], int, int]:
    raw_path = _canonical_heightfield_path(recipe)
    if not raw_path.is_file():
        raise FileNotFoundError(f"canonical Forest_HF heightfield is missing: {raw_path}")
    source = recipe["terrain_source"]
    if source.get("row_order") != "north_to_south":
        raise ValueError("canonical Forest_HF heightfield must use north_to_south row order")
    elevation_min, elevation_max = _pair(source.get("elevation_range_m"), "elevation_range_m")
    if elevation_max <= elevation_min:
        raise ValueError("canonical Forest_HF elevation range must be increasing")
    manifest = _load_canonical_asset_manifest(recipe)
    _verify_canonical_terrain_artifact(
        recipe,
        raw_path,
        str(source["canonical_source"]),
        manifest,
    )
    grid = manifest["canonical_source"] if manifest is not None else source
    width, height = (int(value) for value in grid["dimensions_px"])
    expected_bytes = width * height * 2
    payload = raw_path.read_bytes()
    if len(payload) != expected_bytes:
        raise ValueError(
            f"canonical Forest_HF heightfield has {len(payload)} bytes; expected {expected_bytes}"
        )
    samples = array.array("H")
    samples.frombytes(payload)
    if grid.get("sample_type") != "uint16":
        raise ValueError("canonical Forest_HF heightfield must use uint16 samples")
    if grid.get("endianness") != "little":
        raise ValueError("canonical Forest_HF heightfield must be little-endian")
    if sys.byteorder != "little":  # pragma: no cover - Windows/Linux build hosts are little-endian.
        samples.byteswap()
    return samples, width, height


def _canonical_height_sampler(recipe: Mapping[str, Any]) -> Any:
    samples, width, height = _load_canonical_heightfield(recipe)
    source = recipe["terrain_source"]
    extent_x, extent_y = (float(value) / 2.0 for value in source["extent_m"])
    elevation_min, elevation_max = (float(value) for value in source["elevation_range_m"])
    if source.get("row_order") != "north_to_south":
        raise ValueError("canonical Forest_HF heightfield must use north_to_south row order")

    def sample(x: float, y: float) -> float:
        column = min(width - 1.0, max(0.0, (x + extent_x) * (width - 1) / (2.0 * extent_x)))
        row = min(height - 1.0, max(0.0, (extent_y - y) * (height - 1) / (2.0 * extent_y)))
        x0, y0 = int(math.floor(column)), int(math.floor(row))
        x1, y1 = min(width - 1, x0 + 1), min(height - 1, y0 + 1)
        tx, ty = column - x0, row - y0

        def elevation(ix: int, iy: int) -> float:
            normalized = samples[iy * width + ix] / 65535.0
            return elevation_min + normalized * (elevation_max - elevation_min)

        north = elevation(x0, y0) * (1.0 - tx) + elevation(x1, y0) * tx
        south = elevation(x0, y1) * (1.0 - tx) + elevation(x1, y1) * tx
        return north * (1.0 - ty) + south * ty

    return sample


def _placement_is_clear(
    point: tuple[float, float],
    radius: float,
    corridors: Sequence[Mapping[str, Any]],
    exclusion_zones: Sequence[Mapping[str, Any]],
) -> bool:
    for corridor in corridors:
        points = [_pair(item, "route_corridors.centerline_m") for item in corridor["centerline_m"]]
        clearance = float(corridor["clearance_radius_m"]) + radius
        if min(_distance_to_segment(point, start, end) for start, end in itertools.pairwise(points)) < clearance:
            return False
    return all(
        math.dist(point, tuple(zone["center_m"])) >= float(zone["radius_m"]) + radius
        for zone in exclusion_zones
    )


def _route_belt_candidate(
    seed: int,
    label: str,
    attempt: int,
    corridor: Mapping[str, Any],
    radius: float,
    offset_near: float,
    offset_far: float,
) -> tuple[float, float]:
    """Sample one deterministic placement in the visible belt beside a route."""

    points = [_pair(item, "route_corridors.centerline_m") for item in corridor["centerline_m"]]
    segments = list(itertools.pairwise(points))
    segment_index = min(len(segments) - 1, int(_stable_unit(seed, f"{label}-segment", attempt) * len(segments)))
    start, end = segments[segment_index]
    t = _stable_unit(seed, f"{label}-along", attempt)
    x = start[0] + (end[0] - start[0]) * t
    y = start[1] + (end[1] - start[1]) * t
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = max(math.hypot(dx, dy), 1e-9)
    side = -1.0 if _stable_unit(seed, f"{label}-side", attempt) < 0.5 else 1.0
    offset = float(corridor["clearance_radius_m"]) + radius + _stable_range(
        seed,
        f"{label}-offset",
        attempt,
        offset_near,
        offset_far,
    )
    return x + side * (-dy / length) * offset, y + side * (dx / length) * offset


def _hero_grove_candidate(
    seed: int,
    attempt: int,
    index: int,
    corridor: Mapping[str, Any],
    radius: float,
) -> tuple[float, float]:
    """Distribute paired hero trees into asymmetric clusters and canopy windows."""

    points = [_pair(item, "route_corridors.centerline_m") for item in corridor["centerline_m"]]
    segments = [
        (start, end, math.dist(start, end))
        for start, end in itertools.pairwise(points)
    ]
    total_length = sum(length for _, _, length in segments)
    grove_index = index // 2
    if total_length < 180.0:
        depth_band = grove_index % 3
        side = -1.0 if (grove_index // 3) % 2 == 0 else 1.0
        band_amount = _stable_range(seed, "tree-hero-depth", grove_index, 0.12, 0.88)
        route_distance = total_length * (depth_band + band_amount) / 3.0
    else:
        side = -1.0 if grove_index % 2 == 0 else 1.0
        side_grove_index = grove_index // 2
        active_bins = (
            (0, 1, 4, 5, 8, 9, 12, 13)
            if side < 0.0
            else (0, 3, 4, 7, 8, 11, 12)
        )
        bin_index = active_bins[side_grove_index % len(active_bins)]
        bin_amount = _stable_range(seed, "tree-hero-bin", grove_index, 0.34, 0.66)
        route_distance = total_length * (bin_index + bin_amount) / 14.0
    start, end, segment_length = segments[-1]
    remaining = route_distance
    for candidate_start, candidate_end, candidate_length in segments:
        start, end, segment_length = candidate_start, candidate_end, candidate_length
        if remaining <= candidate_length:
            break
        remaining -= candidate_length
    amount = min(1.0, remaining / max(segment_length, 1e-9))
    x = start[0] + (end[0] - start[0]) * amount
    y = start[1] + (end[1] - start[1]) * amount
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = max(math.hypot(dx, dy), 1e-9)
    envelope = (2.0, 5.5, 10.0, 13.5)[grove_index % 4]
    paired_jitter = _stable_range(seed, "tree-hero-pair", attempt, -1.25, 1.25)
    offset = float(corridor["clearance_radius_m"]) + radius + envelope + paired_jitter
    along_jitter = _stable_range(seed, "tree-hero-along-jitter", attempt, -1.8, 1.8)
    return (
        x + (dx / length) * along_jitter + side * (-dy / length) * offset,
        y + (dy / length) * along_jitter + side * (dx / length) * offset,
    )


def _hero_cluster_anchor_candidate(
    seed: int,
    attempt: int,
    cluster_index: int,
    corridor: Mapping[str, Any],
    radius: float,
) -> tuple[float, float]:
    """Place later hero-grove anchors while preserving side-specific canopy windows."""

    points = [_pair(item, "route_corridors.centerline_m") for item in corridor["centerline_m"]]
    segments = [(start, end, math.dist(start, end)) for start, end in itertools.pairwise(points)]
    total_length = sum(length for _, _, length in segments)
    side = -1.0 if cluster_index % 2 == 0 else 1.0
    side_cluster_index = cluster_index // 2
    active_bins = (
        (0, 1, 4, 5, 8, 9, 12, 13)
        if side < 0.0
        else (0, 3, 4, 7, 8, 11, 12)
    )
    bin_index = active_bins[side_cluster_index % len(active_bins)]
    bin_amount = _stable_range(seed, "tree-hero-cluster-bin", attempt, 0.38, 0.62)
    remaining = total_length * (bin_index + bin_amount) / 14.0
    start, end, segment_length = segments[-1]
    for candidate_start, candidate_end, candidate_length in segments:
        start, end, segment_length = candidate_start, candidate_end, candidate_length
        if remaining <= candidate_length:
            break
        remaining -= candidate_length
    amount = min(1.0, remaining / max(segment_length, 1e-9))
    x = start[0] + (end[0] - start[0]) * amount
    y = start[1] + (end[1] - start[1]) * amount
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = max(math.hypot(dx, dy), 1e-9)
    offset = float(corridor["clearance_radius_m"]) + radius + _stable_range(
        seed,
        "tree-hero-cluster-offset",
        attempt,
        2.0,
        13.0,
    )
    return x + side * (-dy / length) * offset, y + side * (dx / length) * offset


def _route_prefix(
    centerline: Sequence[Sequence[float]],
    distance_m: float,
) -> list[list[float]]:
    """Clip a route to a deterministic distance from its first point."""

    points = [[float(value) for value in point[:2]] for point in centerline]
    if len(points) < 2:
        return points
    clipped = [points[0]]
    remaining = distance_m
    for start, end in itertools.pairwise(points):
        segment_length = math.dist(start, end)
        if segment_length >= remaining:
            amount = remaining / max(segment_length, 1e-9)
            clipped.append(
                [
                    start[0] + (end[0] - start[0]) * amount,
                    start[1] + (end[1] - start[1]) * amount,
                ]
            )
            break
        clipped.append(end)
        remaining -= segment_length
    return clipped


def _cluster_neighbor_candidate(
    seed: int,
    attempt: int,
    anchor: Mapping[str, Any],
) -> tuple[float, float]:
    """Place a canopy neighbor close enough to read as one organic grove."""

    angle = _stable_range(seed, "tree-cluster-angle", attempt, -math.pi, math.pi)
    distance = _stable_range(seed, "tree-cluster-distance", attempt, 2.0, 5.4)
    return (
        float(anchor["position_m"][0]) + math.cos(angle) * distance,
        float(anchor["position_m"][1]) + math.sin(angle) * distance,
    )


def generate_forest_layout(recipe: Mapping[str, Any]) -> dict[str, Any]:
    """Generate stable tree and dressing placements from a validated recipe."""

    seed = _integer(recipe.get("seed"), "seed")
    extent_x, extent_y = _pair(recipe.get("extent_m"), "extent_m", positive=True)
    tree_count = _integer(recipe.get("tree_count"), "tree_count", minimum=1)
    species = recipe.get("species")
    trail = recipe.get("trail")
    if not isinstance(species, Sequence) or isinstance(species, (str, bytes)) or not species:
        raise ValueError("species must be a non-empty array")
    if not isinstance(trail, Mapping):
        raise TypeError("trail must be an object")
    centerline = [_pair(point, "trail.centerline_m") for point in trail["centerline_m"]]
    trail_clearance = _number(trail.get("clearance_radius_m"), "trail.clearance_radius_m", positive=True)
    surface_width = _number(trail.get("surface_width_m", 3.2), "trail.surface_width_m", positive=True)
    shoulder_blend = _number(trail.get("shoulder_blend_m", 0.8), "trail.shoulder_blend_m", positive=True)
    corridors = recipe.get(
        "route_corridors",
        [{"stable_id": "forest.route.primary", "centerline_m": [list(item) for item in centerline], "clearance_radius_m": trail_clearance}],
    )
    exclusions = recipe.get("exclusion_zones", [])
    if not isinstance(corridors, Sequence) or not corridors:
        raise ValueError("route_corridors must be a non-empty array")
    if not isinstance(exclusions, Sequence):
        raise TypeError("exclusion_zones must be an array")
    height_at = (
        _canonical_height_sampler(recipe)
        if recipe.get("terrain_height_mode") == "canonical_package"
        else lambda x, y: _height_at(x, y, seed)
    )

    instances: list[dict[str, Any]] = []
    hero_count = math.ceil(tree_count * 0.65)
    primary_count = math.ceil(tree_count * 0.25)
    hero_corridor = {
        **dict(corridors[0]),
        "centerline_m": _route_prefix(corridors[0]["centerline_m"], 420.0),
    }
    attempt = 0
    placement_attempt = 0
    max_attempts = max(4096, tree_count * 512)
    while len(instances) < tree_count and attempt < max_attempts:
        index = len(instances)
        chosen = _choose_weighted_species(species, _stable_unit(seed, "species", attempt))
        height_low, height_high = _pair(chosen["height_m"], "species.height_m", positive=True)
        crown_low, crown_high = _pair(chosen["crown_radius_m"], "species.crown_radius_m", positive=True)
        crown_radius = _stable_range(seed, "crown", attempt, crown_low, crown_high)
        population_group = index // 4
        group_anchor_index = population_group * 4
        if index < min(hero_count, 60) and placement_attempt < 96:
            x, y = _hero_grove_candidate(seed, attempt, index, hero_corridor, crown_radius)
        elif index % 4 and group_anchor_index < len(instances):
            x, y = _cluster_neighbor_candidate(seed, attempt, instances[group_anchor_index])
        elif index < hero_count:
            x, y = _hero_cluster_anchor_candidate(
                seed,
                attempt,
                population_group - 15,
                hero_corridor,
                crown_radius,
            )
        elif index < hero_count + primary_count:
            x, y = _route_belt_candidate(seed, "tree-primary", attempt, corridors[0], crown_radius, 2.0, 28.0)
        else:
            x = _stable_range(seed, "tree-x", attempt, -extent_x + crown_radius, extent_x - crown_radius)
            y = _stable_range(seed, "tree-y", attempt, -extent_y + crown_radius, extent_y - crown_radius)
        inside_extent = abs(x) <= extent_x - crown_radius and abs(y) <= extent_y - crown_radius
        clear = _placement_is_clear((x, y), crown_radius, corridors, exclusions)
        spacing_clear = all(
            math.dist((x, y), tuple(item["position_m"][:2])) >= 0.42 * (crown_radius + float(item["crown_radius_m"]))
            for item in instances
        )
        attempt += 1
        if not inside_extent or not clear or not spacing_clear:
            placement_attempt += 1
            continue
        height = _stable_range(seed, "height", attempt, height_low, height_high)
        z = height_at(x, y)
        yaw = _stable_range(seed, "yaw", attempt, -180.0, 180.0)
        scale_x = _stable_range(seed, "scale-x", attempt, 0.9, 1.1)
        scale_y = _stable_range(seed, "scale-y", attempt, 0.9, 1.1)
        scale_z = _stable_range(seed, "scale-z", attempt, 0.94, 1.08)
        stable_id = f"forest.tree.{index:04d}"
        instances.append(
            {
                "stable_id": stable_id,
                "kind": "tree",
                "species_id": str(chosen["id"]),
                "asset_slot_id": _tree_asset_slot_id(str(chosen["id"])),
                "variant_id": f"{chosen['id']}-{index % 3:02d}",
                "material": str(chosen["material"]),
                "position_m": [round(x, 6), round(y, 6), z],
                "unreal_position_cm": [round(100.0 * x, 4), round(-100.0 * y, 4), round(100.0 * z, 4)],
                "yaw_deg": round(yaw, 5),
                "unreal_yaw_deg": round(-yaw, 5),
                "height_m": round(height, 5),
                "crown_radius_m": round(crown_radius, 5),
                "scale_xyz": [round(scale_x, 5), round(scale_y, 5), round(scale_z, 5)],
                **_visual_metadata(),
            }
        )
        placement_attempt = 0
    if len(instances) != tree_count:
        raise ValueError("tree_count cannot fit inside extent_m while preserving trail and canopy clearance")

    dressing: list[dict[str, Any]] = []
    dressing_counts = {
        "rock": max(12, tree_count // 3),
        "fallen_log": max(5, tree_count // 10),
        "understory": max(48, tree_count * 2),
    }
    dressing_radius = {"rock": 0.75, "fallen_log": 1.25, "understory": 0.35}
    for kind, count in dressing_counts.items():
        index = 0
        attempt = 0
        placement_attempt = 0
        while index < count and attempt < count * 512:
            scale = _stable_range(seed, f"{kind}-scale", attempt, 0.7, 1.35)
            radius = dressing_radius[kind] * scale
            if kind == "understory" and index < 16 and placement_attempt < 96:
                start = centerline[0]
                angle = _stable_range(seed, "understory-foreground-angle", attempt, -math.pi, math.pi)
                distance = _stable_range(seed, "understory-foreground-distance", attempt, 4.0, 10.5)
                x = start[0] + math.cos(angle) * distance
                y = start[1] + math.sin(angle) * distance
            elif kind == "understory" and index < 40 and placement_attempt < 96:
                x, y = _route_belt_candidate(seed, "understory-edge", attempt, corridors[0], radius, 0.7, 2.0)
            elif _stable_unit(seed, f"{kind}-route-focus", attempt) < 0.85:
                x, y = _route_belt_candidate(seed, kind, attempt, corridors[0], radius, 1.5, 28.0)
            else:
                x = _stable_range(seed, f"{kind}-x", attempt, -extent_x, extent_x)
                y = _stable_range(seed, f"{kind}-y", attempt, -extent_y, extent_y)
            attempt += 1
            if abs(x) > extent_x - radius or abs(y) > extent_y - radius:
                placement_attempt += 1
                continue
            if not _placement_is_clear((x, y), radius, corridors, exclusions):
                placement_attempt += 1
                continue
            z = height_at(x, y)
            dressing.append(
                {
                    "stable_id": f"forest.{kind}.{index:04d}",
                    "kind": kind,
                    **({"asset_slot_id": "forest.asset.boulder"} if kind == "rock" else {}),
                    "position_m": [round(x, 6), round(y, 6), z],
                    "yaw_deg": round(_stable_range(seed, f"{kind}-yaw", attempt, -180.0, 180.0), 5),
                    "scale": round(scale, 5),
                    **_visual_metadata(),
                }
            )
            index += 1
            placement_attempt = 0
        if index != count:
            raise ValueError(f"{kind} dressing cannot fit while preserving route and clearing clearance")

    if recipe.get("schema") != _SCHEMA:
        shoulder_route = _route_prefix(centerline, 120.0)
        shoulder_segments = [
            (start, end, math.dist(start, end))
            for start, end in itertools.pairwise(shoulder_route)
            if math.dist(start, end) > 1e-9
        ]
        shoulder_length = sum(length for _start, _end, length in shoulder_segments)
        shoulder_count = 18
        for index in range(shoulder_count):
            route_distance = shoulder_length * (index + 1) / (shoulder_count + 1)
            remaining = route_distance
            start, end, segment_length = shoulder_segments[-1]
            for candidate_start, candidate_end, candidate_length in shoulder_segments:
                start, end, segment_length = candidate_start, candidate_end, candidate_length
                if remaining <= candidate_length:
                    break
                remaining -= candidate_length
            amount = min(1.0, remaining / segment_length)
            x = start[0] + (end[0] - start[0]) * amount
            y = start[1] + (end[1] - start[1]) * amount
            dx, dy = end[0] - start[0], end[1] - start[1]
            inverse_length = 1.0 / segment_length
            side = -1.0 if index % 2 == 0 else 1.0
            offset = surface_width * 0.5 + _stable_range(
                seed,
                "shoulder-debris-offset",
                index,
                shoulder_blend * 0.25,
                shoulder_blend * 0.85,
            )
            x += side * -dy * inverse_length * offset
            y += side * dx * inverse_length * offset
            visual_kind = "rock" if index % 3 else "fallen_log"
            dressing.append(
                {
                    "stable_id": f"forest.shoulder_debris.{index:04d}",
                    "kind": "shoulder_debris",
                    "visual_kind": visual_kind,
                    **({"asset_slot_id": "forest.asset.boulder"} if visual_kind == "rock" else {}),
                    "position_m": [round(x, 6), round(y, 6), height_at(x, y)],
                    "yaw_deg": round(
                        math.degrees(math.atan2(dy, dx))
                        + _stable_range(seed, "shoulder-debris-yaw", index, -22.0, 22.0),
                        5,
                    ),
                    "scale": round(_stable_range(seed, "shoulder-debris-scale", index, 0.32, 0.58), 5),
                    **_visual_metadata(),
                }
            )

    body = {
        "schema": "lingtu.sim.forest-layout.v1",
        "world_package": WORLD_PACKAGE,
        "seed": seed,
        "extent_m": [extent_x, extent_y],
        "coordinate_contract": {
            "source_frame": "mujoco_rh_z_up_m",
            "blender_frame": "right_handed_z_up_m",
            "unreal_position_mapping": "(x,y,z)m -> (100*x,-100*y,100*z)cm",
            "unreal_yaw_mapping": "yaw_deg -> -yaw_deg",
        },
        "trail": {
            "centerline_m": [list(point) for point in centerline],
            "clearance_radius_m": trail_clearance,
            "surface_width_m": surface_width,
            "shoulder_blend_m": shoulder_blend,
            "closed_loop": bool(trail.get("closed_loop", False)),
        },
        "instances": instances,
        "dressing": dressing,
        "physics": {
            "authority": "mujoco",
            "visual_meshes_are_colliders": False,
            "proxy_policy": "separate package-owned primitives",
            "proxies": [
                {
                    "stable_id": "forest.physics.terrain",
                    "shape": "heightfield",
                    "extent_m": [extent_x, extent_y],
                    **(
                        {"source": recipe["terrain_source"]["canonical_source"]}
                        if recipe.get("terrain_height_mode") == "canonical_package"
                        else {}
                    ),
                    "authority": "mujoco",
                }
            ],
        },
    }
    return {**body, "digest": digest_document(body)}


def build_authoring_manifest(
    recipe: Mapping[str, Any],
    layout: Mapping[str, Any],
    artifacts: Sequence[Mapping[str, Any]],
    *,
    require_material_artifacts: bool = False,
) -> dict[str, Any]:
    """Create the canonical portable manifest for authored Blender artifacts."""

    normalized_assets: list[dict[str, Any]] = []
    identities: set[tuple[str, str]] = set()
    for index, item in enumerate(artifacts):
        if not isinstance(item, Mapping):
            raise TypeError(f"artifacts[{index}] must be an object")
        role = item.get("role")
        if not isinstance(role, str) or not role.strip():
            raise ValueError(f"artifacts[{index}].role is required")
        path = _relative_path(item.get("path"), f"artifacts[{index}].path")
        size = _integer(item.get("bytes"), f"artifacts[{index}].bytes")
        sha256 = item.get("sha256")
        if not isinstance(sha256, str) or _SHA256.fullmatch(sha256) is None:
            raise ValueError(f"artifacts[{index}].sha256 must be a sha256 digest")
        identity = (role, path)
        if identity in identities:
            raise ValueError(f"duplicate artifact identity: {identity}")
        identities.add(identity)
        normalized_assets.append({"role": role, "path": path, "bytes": size, "sha256": sha256})
    normalized_assets.sort(key=lambda item: (item["role"], item["path"]))
    assets_by_role = {item["role"]: item for item in normalized_assets}
    slot_exports = []
    for slot in build_fallback_asset_contract()["slots"]:
        for export_kind in ("export", "unreal_export"):
            export = slot[export_kind]
            artifact = assets_by_role.get(export["artifact_role"])
            if artifact is not None:
                if artifact["path"] != export["path"]:
                    raise ValueError(f"asset slot {slot['slot_id']} export path does not match its contract")
                slot_exports.append(
                    {
                        "slot_id": slot["slot_id"],
                        "object_name": slot["object_name"],
                        "format": Path(artifact["path"]).suffix.removeprefix("."),
                        **artifact,
                    }
                )
    identity = [{"role": item["role"], "path": item["path"], "sha256": item["sha256"]} for item in normalized_assets]
    assets_by_path = {item["path"]: item for item in normalized_assets}
    material_provenance = []
    for item in recipe["materials"]:
        textures = {}
        for channel, texture in item["textures"].items():
            actual = assets_by_path.get(texture["path"])
            if require_material_artifacts and actual is None:
                raise ValueError(
                    f"material {item['id']} texture {channel} has no generated artifact: {texture['path']}"
                )
            textures[channel] = {
                **texture,
                **(
                    {"sha256": actual["sha256"], "bytes": actual["bytes"]}
                    if actual is not None
                    else {}
                ),
            }
        material_provenance.append(
            {
                "id": item["id"],
                "shader": item["shader"],
                "textures": textures,
                **({"bindings": item["bindings"]} if "bindings" in item else {}),
            }
        )
    body = {
        "schema": _MANIFEST_SCHEMA,
        "world_package": WORLD_PACKAGE,
        "generator": {
            "tool": "sim.tools.worlds.forest_hf.blender_author",
            "deterministic_seed": recipe["seed"],
            "bpy_required_for_artifacts": True,
        },
        "recipe": {
            "schema": recipe["schema"],
            "digest": recipe.get("recipe_sha256", _sha256(_canonical_json(dict(recipe)))),
        },
        "layout": {
            "schema": layout["schema"],
            "digest": layout["digest"],
            "tree_count": len(layout["instances"]),
            "dressing_count": len(layout.get("dressing", [])),
            "extent_m": layout["extent_m"],
        },
        "coordinate_contract": layout["coordinate_contract"],
        "authority": {
            "classification": "VisualOnly",
            "unreal_collision_profile": "NoCollision",
            "physics_authority": "mujoco",
            "visual_meshes_are_colliders": False,
            "simulate_physics": False,
            "can_ever_affect_navigation": False,
            "physics_proxies": layout["physics"]["proxies"],
        },
        "materials": material_provenance,
        "asset_library": build_fallback_asset_contract(),
        "export_contract": {
            "editable_blend": {"path": "forest_hf.blend", "editable": True},
            "portable_scene_glb": {"path": "forest_hf.glb", "editable": False},
            "unreal_scene_fbx": {"path": "forest_hf.fbx", "editable": False},
            "asset_slot_glbs": {
                slot["slot_id"]: {
                    "path": slot["export"]["path"],
                    "artifact_role": slot["export"]["artifact_role"],
                    "editable": False,
                }
                for slot in build_fallback_asset_contract()["slots"]
            },
            "asset_slot_fbxs": {
                slot["slot_id"]: {
                    "path": slot["unreal_export"]["path"],
                    "artifact_role": slot["unreal_export"]["artifact_role"],
                    "editable": False,
                    "target": "unreal_static_mesh_import",
                    "source_units": slot["unreal_export"]["source_units"],
                    "target_units": slot["unreal_export"]["target_units"],
                    "export_global_scale": slot["unreal_export"]["export_global_scale"],
                    "coordinate_scale": slot["unreal_export"]["coordinate_scale"],
                }
                for slot in build_fallback_asset_contract()["slots"]
            },
        },
        "asset_slot_exports": slot_exports,
        "assets": normalized_assets,
        "artifact_set_digest": _sha256(_canonical_json(identity)),
    }
    return {**body, "digest": digest_document(body)}


def build_blender_command(
    blender_executable: Path | str,
    *,
    repo_root: Path | str,
    recipe: Path | str,
    output_dir: Path | str,
    width: int = 1280,
    height: int = 720,
    samples: int = 32,
) -> list[str]:
    """Build the isolated headless Blender authoring command."""

    root = Path(repo_root).resolve()
    script = root / "sim/tools/worlds/forest_hf/blender_author.py"
    recipe_path = Path(recipe)
    output_path = Path(output_dir)
    if not recipe_path.is_absolute():
        recipe_path = root / recipe_path
    if not output_path.is_absolute():
        output_path = root / output_path
    executable = str(blender_executable)
    if re.match(r"^(?:[A-Za-z]:[\\/]|\\\\)", executable):
        executable = str(PureWindowsPath(executable))
    return [
        executable,
        "--factory-startup",
        "--background",
        "--python",
        str(script),
        "--",
        "--repo-root",
        str(root),
        "--recipe",
        str(recipe_path),
        "--output-dir",
        str(output_path),
        "--width",
        str(width),
        "--height",
        str(height),
        "--samples",
        str(samples),
    ]


def validate_exported_glb(
    path: Path | str,
    *,
    forbid_template_nodes: bool = True,
    require_textures: bool = True,
) -> dict[str, Any]:
    """Validate GLB structure, embedded texture payloads, UVs, and scene hygiene."""

    glb_path = Path(path)
    payload = glb_path.read_bytes()
    if len(payload) < 20 or payload[:4] != b"glTF":
        raise ValueError(f"not a GLB file: {glb_path}")
    version, declared_length = struct.unpack_from("<II", payload, 4)
    if version != 2 or declared_length != len(payload):
        raise ValueError("GLB header version or declared length is invalid")
    chunk_length, chunk_type = struct.unpack_from("<II", payload, 12)
    if chunk_type != 0x4E4F534A or 20 + chunk_length > len(payload):
        raise ValueError("GLB first chunk must be valid JSON")
    document = json.loads(payload[20 : 20 + chunk_length].rstrip(b" \t\r\n\x00").decode("utf-8"))
    if not isinstance(document, Mapping):
        raise ValueError("GLB JSON root must be an object")
    binary_chunk: bytes | None = None
    cursor = 20 + chunk_length
    while cursor < len(payload):
        if cursor + 8 > len(payload):
            raise ValueError("GLB contains a truncated chunk header")
        current_length, current_type = struct.unpack_from("<II", payload, cursor)
        cursor += 8
        end = cursor + current_length
        if end > len(payload):
            raise ValueError("GLB contains a truncated chunk payload")
        if current_type == 0x004E4942:
            if binary_chunk is not None:
                raise ValueError("GLB contains more than one BIN chunk")
            binary_chunk = payload[cursor:end]
        cursor = end
    nodes = document.get("nodes", [])
    meshes = document.get("meshes", [])
    materials = document.get("materials", [])
    textures = document.get("textures", [])
    images = document.get("images", [])
    buffer_views = document.get("bufferViews", [])
    buffers = document.get("buffers", [])
    arrays = (nodes, meshes, materials, textures, images, buffer_views, buffers)
    if not all(isinstance(item, Sequence) and not isinstance(item, (str, bytes)) for item in arrays):
        raise ValueError("GLB nodes, meshes, materials, textures, images, bufferViews, and buffers must be arrays")
    forbidden_names = []
    for node in nodes:
        if not isinstance(node, Mapping):
            raise ValueError("GLB node must be an object")
        name = str(node.get("name", ""))
        if "template" in name.lower() or name.startswith("LT_Forest_Asset_"):
            forbidden_names.append(name)
    if forbid_template_nodes and forbidden_names:
        raise ValueError(f"GLB contains excluded template nodes: {sorted(forbidden_names)}")
    textured_materials = set()
    referenced_textures: set[int] = set()

    def record_texture_reference(slot: object, label: str) -> None:
        if not isinstance(slot, Mapping):
            raise ValueError(f"GLB {label} must be an object")
        texture_index = slot.get("index")
        if not isinstance(texture_index, int) or isinstance(texture_index, bool):
            raise ValueError(f"GLB {label}.index must be an integer")
        if texture_index < 0 or texture_index >= len(textures):
            raise ValueError(f"GLB {label}.index is out of bounds")
        referenced_textures.add(texture_index)

    for index, material in enumerate(materials):
        if not isinstance(material, Mapping):
            raise ValueError("GLB material must be an object")
        pbr = material.get("pbrMetallicRoughness", {})
        material_is_textured = False
        if not isinstance(pbr, Mapping):
            raise ValueError("GLB pbrMetallicRoughness must be an object")
        for slot_name in ("baseColorTexture", "metallicRoughnessTexture"):
            if slot_name in pbr:
                record_texture_reference(pbr[slot_name], f"materials[{index}].pbrMetallicRoughness.{slot_name}")
                material_is_textured = True
        for slot_name in ("normalTexture", "occlusionTexture", "emissiveTexture"):
            if slot_name in material:
                record_texture_reference(material[slot_name], f"materials[{index}].{slot_name}")
                material_is_textured = True
        if material_is_textured:
            textured_materials.add(index)
    primitive_count = 0
    textured_primitive_count = 0
    for mesh in meshes:
        if not isinstance(mesh, Mapping) or not isinstance(mesh.get("primitives"), Sequence):
            raise ValueError("GLB mesh must define primitives")
        for primitive in mesh["primitives"]:
            if not isinstance(primitive, Mapping):
                raise ValueError("GLB primitive must be an object")
            primitive_count += 1
            material_index = primitive.get("material")
            if material_index is not None and (
                not isinstance(material_index, int)
                or isinstance(material_index, bool)
                or material_index < 0
                or material_index >= len(materials)
            ):
                raise ValueError("GLB primitive material index is out of bounds")
            if material_index in textured_materials:
                textured_primitive_count += 1
                attributes = primitive.get("attributes")
                if not isinstance(attributes, Mapping) or "TEXCOORD_0" not in attributes:
                    raise ValueError("GLB textured primitive is missing TEXCOORD_0")
    if require_textures and textured_primitive_count == 0:
        raise ValueError("GLB must contain at least one textured primitive")

    for texture_index in sorted(referenced_textures):
        texture = textures[texture_index]
        if not isinstance(texture, Mapping):
            raise ValueError(f"GLB textures[{texture_index}] must be an object")
        source_index = texture.get("source")
        if not isinstance(source_index, int) or isinstance(source_index, bool):
            raise ValueError(f"GLB textures[{texture_index}].source must be an integer")
        if source_index < 0 or source_index >= len(images):
            raise ValueError(f"GLB textures[{texture_index}].source is out of bounds")
        image = images[source_index]
        if not isinstance(image, Mapping):
            raise ValueError(f"GLB images[{source_index}] must be an object")
        if "bufferView" in image:
            view_index = image["bufferView"]
            if not isinstance(view_index, int) or isinstance(view_index, bool):
                raise ValueError(f"GLB images[{source_index}].bufferView must be an integer")
            if view_index < 0 or view_index >= len(buffer_views):
                raise ValueError(f"GLB images[{source_index}].bufferView is out of bounds")
            view = buffer_views[view_index]
            if not isinstance(view, Mapping):
                raise ValueError(f"GLB bufferViews[{view_index}] must be an object")
            buffer_index = view.get("buffer")
            byte_offset = view.get("byteOffset", 0)
            byte_length = view.get("byteLength")
            if not isinstance(buffer_index, int) or isinstance(buffer_index, bool) or not (0 <= buffer_index < len(buffers)):
                raise ValueError(f"GLB bufferViews[{view_index}].buffer is out of bounds")
            if not isinstance(byte_offset, int) or isinstance(byte_offset, bool) or byte_offset < 0:
                raise ValueError(f"GLB bufferViews[{view_index}].byteOffset is invalid")
            if not isinstance(byte_length, int) or isinstance(byte_length, bool) or byte_length <= 0:
                raise ValueError(f"GLB images[{source_index}] payload is empty")
            buffer = buffers[buffer_index]
            if not isinstance(buffer, Mapping):
                raise ValueError(f"GLB buffers[{buffer_index}] must be an object")
            if buffer.get("uri") is not None or buffer_index != 0 or binary_chunk is None:
                raise ValueError(f"GLB images[{source_index}] bufferView has no embedded BIN payload")
            if byte_offset + byte_length > len(binary_chunk):
                raise ValueError(f"GLB images[{source_index}] payload exceeds the BIN chunk")
        else:
            uri = image.get("uri")
            if not isinstance(uri, str) or not uri.startswith("data:") or "," not in uri:
                raise ValueError(f"GLB images[{source_index}] must embed a bufferView or data URI payload")
            metadata, encoded = uri.split(",", 1)
            try:
                if ";base64" in metadata:
                    image_payload = base64.b64decode(encoded, validate=True)
                else:
                    image_payload = urllib.parse.unquote_to_bytes(encoded)
            except (ValueError, binascii.Error) as exc:
                raise ValueError(f"GLB images[{source_index}] data URI is invalid") from exc
            if not image_payload:
                raise ValueError(f"GLB images[{source_index}] payload is empty")
    return {
        "path": str(glb_path),
        "node_count": len(nodes),
        "mesh_count": len(meshes),
        "primitive_count": primitive_count,
        "textured_primitive_count": textured_primitive_count,
        "embedded_texture_count": len(referenced_textures),
    }


def _require_bpy() -> Any:
    if bpy is None:
        raise RuntimeError("Blender bpy is required; run this script with blender --background --python")
    return bpy


def _set_visual_only(obj: Any, stable_id: str, semantic_class: str) -> None:
    obj["stable_id"] = stable_id
    obj["semantic_class"] = semantic_class
    obj["classification"] = "VisualOnly"
    obj["collision_profile"] = "NoCollision"
    obj["collision"] = False
    obj["simulate_physics"] = False
    obj["can_ever_affect_navigation"] = False
    obj["physics_representation"] = "none"


def _set_asset_slot_metadata(
    obj: Any,
    slot_id: str,
    external_asset: Mapping[str, Any] | None = None,
) -> None:
    contract = build_fallback_asset_contract()
    slot = next(item for item in contract["slots"] if item["slot_id"] == slot_id)
    obj["asset_slot_id"] = slot_id
    if external_asset is None:
        obj["asset_source_kind"] = "procedural"
        obj["asset_source_generator"] = "lingtu_blender_fallback"
    else:
        obj["asset_source_kind"] = "conditioned_glb"
        obj["asset_source_path"] = str(external_asset["path"])
        obj["asset_source_bytes"] = int(external_asset["bytes"])
        obj["asset_source_sha256"] = str(external_asset["sha256"])
    obj["asset_contract_digest"] = contract["digest"]
    obj["normalization_units"] = slot["normalization"]["units"]
    obj["normalization_origin"] = slot["normalization"]["origin"]
    obj["topology_profile"] = slot["topology"]["profile"]
    obj["uv_map_name"] = slot["uv"]["map_name"]
    obj["pbr_shader"] = slot["pbr"]["shader"]


def _new_material(name: str, base: tuple[float, float, float, float], roughness: float) -> Any:
    blender = _require_bpy()
    material = blender.data.materials.new(name)
    material.diffuse_color = base
    material.use_nodes = True
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = base
    bsdf.inputs["Roughness"].default_value = roughness
    return material


def _procedural_pixels(
    size: int,
    seed: int,
    mode: str,
    base: tuple[float, float, float],
    *,
    material_id: str = "",
) -> list[float]:
    """Generate deterministic multi-scale periodic pixels with seamless borders."""

    pixels: list[float] = []
    denominator = max(1, size - 1)

    def lattice(frequency: int, octave: int) -> list[list[float]]:
        return [
            [
                _stable_unit(seed, f"texture-{mode}-octave-{octave}", iy * frequency + ix) * 2.0 - 1.0
                for ix in range(frequency)
            ]
            for iy in range(frequency)
        ]

    octaves = tuple(
        (frequency, weight, lattice(frequency, octave))
        for octave, (frequency, weight) in enumerate(((3, 0.52), (7, 0.26), (13, 0.14), (23, 0.08)))
    )

    def sample(u: float, v: float) -> float:
        value = 0.0
        for frequency, weight, values in octaves:
            fx = (u % 1.0) * frequency
            fy = (v % 1.0) * frequency
            ix = math.floor(fx)
            iy = math.floor(fy)
            tx = fx - ix
            ty = fy - iy
            tx = tx * tx * (3.0 - 2.0 * tx)
            ty = ty * ty * (3.0 - 2.0 * ty)
            x0, x1 = ix % frequency, (ix + 1) % frequency
            y0, y1 = iy % frequency, (iy + 1) % frequency
            low = values[y0][x0] * (1.0 - tx) + values[y0][x1] * tx
            high = values[y1][x0] * (1.0 - tx) + values[y1][x1] * tx
            value += weight * (low * (1.0 - ty) + high * ty)
        return value

    for y in range(size):
        for x in range(size):
            u = x / denominator
            v = y / denominator
            noise = sample(u, v)
            normalized = min(1.0, max(0.0, 0.5 + 0.5 * noise))
            grain = 0.78 + 0.32 * normalized
            if material_id == "muddy_trail":
                rut_a = math.exp(-((1.0 - math.cos(math.tau * (v - 0.10))) / 0.035))
                rut_b = math.exp(-((1.0 - math.cos(math.tau * (v + 0.10))) / 0.035))
                rut_variation = 0.65 + 0.35 * sample(u * 0.55, v + 0.19)
                grain *= 1.0 - 0.075 * (rut_a + rut_b) * rut_variation
            if mode == "basecolor":
                color = tuple(channel * grain for channel in base)
                if material_id == "forest_ground":
                    litter_field = sample(u + 0.37, v + 0.61)
                    litter_mix = min(1.0, max(0.0, (litter_field + 0.05) / 0.40))
                    litter_mix = 0.30 * litter_mix * litter_mix * (3.0 - 2.0 * litter_mix)
                    warm = (base[0] * 1.65, base[1] * 0.72, base[2] * 0.70)
                    color = tuple(
                        source * (1.0 - litter_mix) + target * litter_mix
                        for source, target in zip(color, warm)
                    )
                pixels.extend((*color, 1.0))
            elif mode == "normal":
                step = 1.0 / denominator
                nx = 0.5 - 0.035 * (sample(u + step, v) - sample(u - step, v))
                ny = 0.5 - 0.035 * (sample(u, v + step) - sample(u, v - step))
                pixels.extend((nx, ny, 1.0, 1.0))
            else:  # ORM: AO, roughness, metallic.
                pixels.extend((0.86 + noise * 0.10, 0.72 + noise * 0.14, 0.0, 1.0))
    return pixels


def _create_texture_set(
    output_dir: Path,
    material_id: str,
    seed: int,
    base: tuple[float, float, float],
    normal_strength: float = 0.22,
    emission_strength: float = 0.0,
    size: int = 256,
) -> tuple[dict[str, Path], Any]:
    blender = _require_bpy()
    texture_dir = output_dir / "textures" / "forest"
    texture_dir.mkdir(parents=True, exist_ok=True)
    paths: dict[str, Path] = {}
    images: dict[str, Any] = {}
    for mode in ("basecolor", "normal", "orm"):
        path = texture_dir / f"{material_id}_{mode}.png"
        image = blender.data.images.new(f"T_{material_id}_{mode}", width=size, height=size, alpha=True)
        image.pixels.foreach_set(_procedural_pixels(size, seed, mode, base, material_id=material_id))
        image.filepath_raw = str(path)
        image.file_format = "PNG"
        image.save()
        paths[mode] = path
        images[mode] = image

    material = _new_material(f"M_{material_id}", (*base, 1.0), 0.76)
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    bsdf = nodes.get("Principled BSDF")
    base_node = nodes.new("ShaderNodeTexImage")
    base_node.image = images["basecolor"]
    links.new(base_node.outputs["Color"], bsdf.inputs["Base Color"])
    normal_node = nodes.new("ShaderNodeTexImage")
    normal_node.image = images["normal"]
    normal_node.image.colorspace_settings.name = "Non-Color"
    normal_map = nodes.new("ShaderNodeNormalMap")
    normal_map.inputs["Strength"].default_value = normal_strength
    links.new(normal_node.outputs["Color"], normal_map.inputs["Color"])
    links.new(normal_map.outputs["Normal"], bsdf.inputs["Normal"])
    orm_node = nodes.new("ShaderNodeTexImage")
    orm_node.image = images["orm"]
    orm_node.image.colorspace_settings.name = "Non-Color"
    separate = nodes.new("ShaderNodeSeparateColor")
    links.new(orm_node.outputs["Color"], separate.inputs["Color"])
    links.new(separate.outputs["Green"], bsdf.inputs["Roughness"])
    if emission_strength > 0.0:
        emission = bsdf.inputs.get("Emission Color") or bsdf.inputs.get("Emission")
        if emission is not None:
            links.new(base_node.outputs["Color"], emission)
        strength = bsdf.inputs.get("Emission Strength")
        if strength is not None:
            strength.default_value = emission_strength
    return paths, material


def _hex_base_color(value: str) -> tuple[float, float, float]:
    normalized = value.removeprefix("#")
    if len(normalized) != 6:
        raise ValueError(f"material palette color must be #RRGGBB: {value!r}")
    return tuple(int(normalized[index : index + 2], 16) / 255.0 for index in (0, 2, 4))


def outdoor_material_base_color(material_id: str, palette_color: str) -> tuple[float, float, float]:
    """Lift dark forest albedo for outdoor lighting while preserving hue and headroom."""

    base = _hex_base_color(palette_color)
    factor = 2.0 if material_id in {"pine_bark", "pine_needles", "forest_ground"} else 1.0
    return tuple(min(value * factor, 1.0) for value in base)


def generated_material_contracts(material_ids: Sequence[str]) -> list[dict[str, Any]]:
    """Describe the output PNG PBR sets actually bound to scene materials."""

    specs = procedural_material_specs()
    records = []
    for material_id in material_ids:
        if material_id not in specs:
            raise ValueError(f"unknown generated forest material: {material_id}")
        textures = {
            channel: {
                "path": f"textures/forest/{material_id}_{suffix}.png",
                "sha256": "0" * 64,
                "source": f"repo://sim/tools/worlds/forest_hf/blender_visuals.py#{material_id}:{channel}",
                "license": "LicenseRef-LingTu-Project-Owned",
            }
            for channel, suffix in (
                ("base_color", "basecolor"),
                ("normal", "normal"),
                ("orm", "orm"),
            )
        }
        normal_binding: dict[str, Any] = {
            "texture": "normal",
            "color_space": "Non-Color",
            "compression": "NormalMap",
        }
        if material_id == "muddy_trail" and "normal_strength" in specs[material_id]:
            normal_binding["strength"] = float(specs[material_id]["normal_strength"])
        records.append(
            {
                "id": material_id,
                "shader": "principled_pbr",
                "textures": textures,
                "bindings": {
                    "base_color": {"texture": "base_color", "color_space": "sRGB"},
                    "normal": normal_binding,
                    "ambient_occlusion": {"texture": "orm", "channel": "R"},
                    "roughness": {"texture": "orm", "channel": "G"},
                    "metallic": {"texture": "orm", "channel": "B"},
                },
            }
        )
    return records


def _link_object(source: Any, name: str, collection: Any) -> Any:
    obj = source.copy()
    obj.data = source.data
    obj.name = name
    collection.objects.link(obj)
    return obj


def _link_visual_hierarchy(
    source: Any,
    name: str,
    collection: Any,
    stable_id: str,
    semantic_class: str,
) -> Any:
    """Duplicate a helper-authored hierarchy while sharing immutable mesh data."""

    def duplicate(item: Any, parent: Any | None, part_index: int) -> Any:
        obj = item.copy()
        if item.data is not None:
            obj.data = item.data
        obj.name = name if parent is None else f"{name}_Part_{part_index:02d}"
        collection.objects.link(obj)
        obj.parent = parent
        obj.matrix_parent_inverse = item.matrix_parent_inverse.copy()
        obj.hide_render = False
        obj.hide_viewport = False
        obj.hide_set(False)
        part_id = stable_id if parent is None else f"{stable_id}.part.{part_index:02d}"
        _set_visual_only(obj, part_id, semantic_class)
        for child_index, child in enumerate(item.children):
            duplicate(child, obj, child_index)
        return obj

    return duplicate(source, None, 0)


def _hide_visual_template(root: Any, semantic_class: str) -> None:
    """Keep reusable helper geometry editable in the blend but out of exports."""

    for index, obj in enumerate((root, *root.children_recursive)):
        obj.hide_render = True
        obj.hide_viewport = True
        obj.hide_set(True)
        _set_visual_only(obj, f"template.{root.name}.{index:02d}", semantic_class)


def _import_external_asset_templates(
    external_asset_slots: Mapping[str, Mapping[str, Any]],
    template_collection: Any,
    visual_templates: Mapping[str, Any],
    *,
    blender_override: Any | None = None,
) -> dict[str, Any]:
    """Import conditioned GLBs behind stable, visual-only slot roots."""

    if not external_asset_slots:
        return dict(visual_templates)
    blender = blender_override if blender_override is not None else _require_bpy()
    templates = dict(visual_templates)
    for semantic_class, external_asset in sorted(external_asset_slots.items()):
        slot_id = _EXTERNAL_ASSET_SLOT_IDS[semantic_class]
        _, object_name = _EXTERNAL_ASSET_SLOT_NAMES[slot_id]
        fallback = templates.get(semantic_class)
        if fallback is not None:
            fallback.name = f"Template_Procedural_{semantic_class.title()}"
            if hasattr(fallback, "children_recursive"):
                _hide_visual_template(fallback, f"template.procedural.{semantic_class}")

        before = {id(obj) for obj in blender.data.objects}
        source_path = getattr(external_asset, "source_path", Path(str(external_asset["path"])))
        _require_finished(
            blender.ops.import_scene.gltf(filepath=str(source_path)),
            operation=f"conditioned GLB import ({semantic_class})",
        )
        imported = [obj for obj in blender.data.objects if id(obj) not in before]
        if not imported:
            raise RuntimeError(f"conditioned GLB import created no objects: {semantic_class}")

        top_level = [obj for obj in imported if obj.parent is None or obj.parent not in imported]
        if len(top_level) == 1:
            root = top_level[0]
            root.name = object_name
        else:
            root = blender.data.objects.new(object_name, None)
            template_collection.objects.link(root)
            for obj in top_level:
                matrix_world = obj.matrix_world.copy()
                obj.parent = root
                obj.matrix_world = matrix_world
        for obj in imported:
            if hasattr(obj, "users_collection"):
                _move_to_collection(obj, template_collection)

        hierarchy = (root, *root.children_recursive)
        for index, obj in enumerate(hierarchy):
            _set_visual_only(
                obj,
                f"template.{object_name}.{index:02d}",
                f"{semantic_class}_template",
            )
            _set_asset_slot_metadata(obj, slot_id, external_asset)
        _hide_visual_template(root, f"template.{semantic_class}")
        templates[semantic_class] = root
    return templates


def build_recipe_visual_templates(
    blender: Any,
    template_collection: Any,
    materials: Mapping[str, Any],
    recipe: Mapping[str, Any],
) -> dict[str, Any]:
    """Build procedural templates or import a complete conditioned slot set."""

    external_asset_slots = recipe.get("external_asset_slots", {})
    if not external_asset_slots:
        return build_visual_templates(blender, template_collection, materials)
    external_asset_root = recipe.get("external_asset_root")
    if not isinstance(external_asset_root, str) or not external_asset_root:
        raise ValueError("external_asset_root is required for conditioned asset slots")
    return _import_external_asset_templates(
        external_asset_slots,
        template_collection,
        {},
        blender_override=blender,
    )


def _tree_variant(
    name: str,
    material: Any,
    crown_material: Any,
    segments: int,
    layers: int,
    asset_slot_id: str,
) -> Any:
    blender = _require_bpy()
    blender.ops.mesh.primitive_cylinder_add(vertices=segments, radius=0.16, depth=1.0, location=(0, 0, 0.5))
    trunk = blender.context.object
    trunk.name = f"{name}_Trunk"
    trunk.data.materials.append(material)
    crown_parts = []
    for layer in range(layers):
        blender.ops.mesh.primitive_cone_add(
            vertices=segments,
            radius1=0.7 - layer * 0.09,
            radius2=0.06,
            depth=0.8,
            location=(0, 0, 0.9 + layer * 0.35),
        )
        part = blender.context.object
        part.data.materials.append(crown_material)
        crown_parts.append(part)
    for item in [trunk, *crown_parts]:
        item.select_set(True)
    blender.context.view_layer.objects.active = trunk
    blender.ops.object.join()
    trunk.name = name
    _set_visual_only(trunk, f"template.{name}", "tree_template")
    _set_asset_slot_metadata(trunk, asset_slot_id)
    trunk.hide_render = True
    trunk.hide_viewport = True
    return trunk


def _setup_scene(width: int, height: int, samples: int) -> dict[str, Any]:
    blender = _require_bpy()
    blender.ops.wm.read_factory_settings(use_empty=True)
    scene = blender.context.scene
    try:
        scene.render.engine = "BLENDER_EEVEE_NEXT"
    except TypeError:  # Blender 4.x compatibility.
        scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.view_settings.exposure = 0.14
    if hasattr(scene, "eevee"):
        eevee = scene.eevee
        if hasattr(eevee, "taa_render_samples"):
            eevee.taa_render_samples = samples
        if hasattr(eevee, "use_gtao"):
            eevee.use_gtao = True
        if hasattr(eevee, "gtao_distance"):
            eevee.gtao_distance = 4.0
        if hasattr(eevee, "gtao_factor"):
            eevee.gtao_factor = 1.25
    world = blender.data.worlds.new("ForestWorld")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.12, 0.18, 0.14, 1.0)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.58
    scene.world = world
    collections = {}
    for name in (
        "Terrain",
        "Trail",
        "Trees",
        "Dressing",
        "Templates",
        "Lighting",
        "Cameras",
        "ReviewActors",
    ):
        collection = blender.data.collections.new(name)
        scene.collection.children.link(collection)
        collections[name] = collection
    return collections


def _move_to_collection(obj: Any, collection: Any) -> None:
    for old in list(obj.users_collection):
        old.objects.unlink(obj)
    collection.objects.link(obj)


def _assign_planar_uv(
    mesh: Any,
    extent_x: float,
    extent_y: float,
    *,
    tile_m: float | None = None,
) -> None:
    uv_layer = mesh.uv_layers.get("UVMap") or mesh.uv_layers.new(name="UVMap")
    for loop in mesh.loops:
        vertex = mesh.vertices[loop.vertex_index].co
        if tile_m is None:
            uv = (
                (float(vertex.x) + extent_x) / (2.0 * extent_x),
                (float(vertex.y) + extent_y) / (2.0 * extent_y),
            )
        else:
            uv = (float(vertex.x) / tile_m, float(vertex.y) / tile_m)
        uv_layer.data[loop.index].uv = uv


def _add_terrain(recipe: Mapping[str, Any], layout: Mapping[str, Any], collection: Any, material: Any) -> Any:
    blender = _require_bpy()
    extent_x, extent_y = layout["extent_m"]
    if recipe.get("terrain_height_mode") != "canonical_package":
        raise ValueError("Forest_HF v2 Blender authoring requires canonical package terrain")
    height_at = _canonical_height_sampler(recipe)
    source_width, source_height = (int(value) for value in recipe["terrain_source"]["dimensions_px"])
    try:
        preview_width, preview_height = blender_terrain_preview_dimensions(source_width, source_height)
    except TypeError:  # Test doubles and legacy callers may retain the zero-argument contract.
        preview_width, preview_height = blender_terrain_preview_dimensions()
    steps_x = preview_width - 1
    steps_y = preview_height - 1
    vertices = []
    faces = []
    for iy in range(steps_y + 1):
        y = -extent_y + 2.0 * extent_y * iy / steps_y
        for ix in range(steps_x + 1):
            x = -extent_x + 2.0 * extent_x * ix / steps_x
            vertices.append((x, y, height_at(x, y)))
    stride = steps_x + 1
    for iy in range(steps_y):
        for ix in range(steps_x):
            a = iy * stride + ix
            faces.append((a, a + 1, a + stride + 1, a + stride))
    mesh = blender.data.meshes.new("ForestTerrainMesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.materials.append(material)
    try:
        _assign_planar_uv(mesh, extent_x, extent_y, tile_m=28.0)
    except TypeError:  # Test doubles and legacy callers may retain the positional contract.
        _assign_planar_uv(mesh, extent_x, extent_y)
    for polygon in getattr(mesh, "polygons", ()):
        polygon.use_smooth = True
    obj = blender.data.objects.new("VisualOnly_ForestTerrain", mesh)
    collection.objects.link(obj)
    _set_visual_only(obj, "forest.terrain.visual", "terrain_visual")
    obj["canonical_source"] = recipe["terrain_source"]["canonical_source"]
    return obj


def _resample_trail_centerline(
    control_points: Sequence[Sequence[float]],
    *,
    closed_loop: bool = False,
    max_segment_m: float = 10.0,
) -> list[list[float]]:
    """Linearly resample a route so no rendered chord exceeds ``max_segment_m``."""

    points = [[float(point[0]), float(point[1])] for point in control_points]
    if closed_loop and points and math.dist(points[0], points[-1]) > 1e-6:
        points.append(points[0])
    if len(points) < 2:
        return points
    result: list[list[float]] = []
    for segment_index, (start, end) in enumerate(itertools.pairwise(points)):
        distance = math.dist(start, end)
        subdivisions = max(1, math.ceil(distance / max_segment_m))
        for step in range(subdivisions + 1):
            if segment_index > 0 and step == 0:
                continue
            t = step / subdivisions
            result.append(
                [
                    start[0] + (end[0] - start[0]) * t,
                    start[1] + (end[1] - start[1]) * t,
                ]
            )
    return result


def _trail_uv_coordinates(
    points: Sequence[Sequence[float]],
    *,
    trail_width_m: float,
    tile_m: float = 14.0,
) -> list[tuple[float, float]]:
    """Return paired UVs using accumulated route metres and world-width metres."""

    accumulated = 0.0
    coordinates: list[tuple[float, float]] = []
    for index, point in enumerate(points):
        if index:
            accumulated += math.dist(points[index - 1], point)
        u = accumulated / tile_m
        half_v = trail_width_m / (2.0 * tile_m)
        coordinates.extend(((u, -half_v), (u, half_v)))
    return coordinates


def _add_trail(recipe: Mapping[str, Any], layout: Mapping[str, Any], collection: Any, material: Any) -> Any:
    blender = _require_bpy()
    points = _resample_trail_centerline(
        layout["trail"]["centerline_m"],
        closed_loop=bool(layout["trail"].get("closed_loop")),
        max_segment_m=4.0,
    )
    surface_width = float(
        layout["trail"].get(
            "surface_width_m",
            max(2.2, float(layout["trail"]["clearance_radius_m"]) * 1.1),
        )
    )
    shoulder_blend = float(layout["trail"].get("shoulder_blend_m", 0.8))
    has_shoulder_mesh = "shoulder_blend_m" in layout["trail"]
    half_width = surface_width * 0.5
    outer_half_width = half_width + shoulder_blend
    vertices = []
    faces = []
    height_at = (
        _canonical_height_sampler(recipe)
        if recipe.get("terrain_height_mode") == "canonical_package"
        else lambda x, y: _height_at(x, y, layout["seed"])
    )
    for index, point in enumerate(points):
        previous = points[max(0, index - 1)]
        following = points[min(len(points) - 1, index + 1)]
        dx = following[0] - previous[0]
        dy = following[1] - previous[1]
        length = max(math.hypot(dx, dy), 1e-9)
        nx, ny = -dy / length, dx / length
        outer_left_x, outer_left_y = point[0] + nx * outer_half_width, point[1] + ny * outer_half_width
        left_x, left_y = point[0] + nx * half_width, point[1] + ny * half_width
        right_x, right_y = point[0] - nx * half_width, point[1] - ny * half_width
        outer_right_x, outer_right_y = point[0] - nx * outer_half_width, point[1] - ny * outer_half_width
        if has_shoulder_mesh:
            vertices.extend(
                [
                    (outer_left_x, outer_left_y, height_at(outer_left_x, outer_left_y) + 0.02),
                    (left_x, left_y, height_at(left_x, left_y) + 0.02),
                    (right_x, right_y, height_at(right_x, right_y) + 0.02),
                    (outer_right_x, outer_right_y, height_at(outer_right_x, outer_right_y) + 0.02),
                ]
            )
        else:
            vertices.extend(
                [
                    (left_x, left_y, height_at(left_x, left_y) + 0.02),
                    (right_x, right_y, height_at(right_x, right_y) + 0.02),
                ]
            )
    for index in range(len(points) - 1):
        if has_shoulder_mesh:
            start = index * 4
            faces.extend(
                (
                    (start, start + 4, start + 5, start + 1),
                    (start + 1, start + 5, start + 6, start + 2),
                    (start + 2, start + 6, start + 7, start + 3),
                )
            )
        else:
            start = index * 2
            faces.append((start, start + 2, start + 3, start + 1))
    mesh = blender.data.meshes.new("ForestTrailMesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.materials.append(material)
    if has_shoulder_mesh:
        trail_blend = mesh.attributes.new(name="TrailBlend", type="FLOAT", domain="POINT")
        for index, value in enumerate((0.0, 1.0, 1.0, 0.0) * len(points)):
            trail_blend.data[index].value = value
    uv_layer = mesh.uv_layers.new(name="UVMap")
    paired_uvs = _trail_uv_coordinates(points, trail_width_m=surface_width, tile_m=14.0)
    if has_shoulder_mesh:
        outer_v = outer_half_width / 14.0
        uv_coordinates = []
        for index in range(len(points)):
            left_uv, right_uv = paired_uvs[index * 2 : index * 2 + 2]
            uv_coordinates.extend(((left_uv[0], -outer_v), left_uv, right_uv, (right_uv[0], outer_v)))
    else:
        uv_coordinates = paired_uvs
    for loop in mesh.loops:
        uv_layer.data[loop.index].uv = uv_coordinates[loop.vertex_index]
    obj = blender.data.objects.new("VisualOnly_RobotTrail", mesh)
    collection.objects.link(obj)
    _set_visual_only(obj, "forest.trail.visual", "robot_trail_visual")
    obj["surface_width_m"] = surface_width
    obj["shoulder_blend_m"] = shoulder_blend
    return obj


def _add_dressing(
    recipe: Mapping[str, Any],
    layout: Mapping[str, Any],
    collection: Any,
    template_collection: Any,
    materials: Mapping[str, Any],
    visual_templates: Mapping[str, Any],
    asset_slot_overrides: Mapping[str, Any] | None = None,
) -> None:
    blender = _require_bpy()
    templates: dict[str, Any] = {}
    boulder_override = (asset_slot_overrides or {}).get("boulder") or visual_templates.get("boulder")
    if boulder_override is None:
        blender.ops.mesh.primitive_ico_sphere_add(subdivisions=1, radius=0.55)
        templates["rock"] = blender.context.object
        templates["rock"].name = "LT_Forest_Asset_Boulder"
        templates["rock"].location.z = 0.55
        blender.ops.object.transform_apply(location=True, rotation=True, scale=True)
        templates["rock"].data.materials.append(materials["rock"])
    else:
        templates["rock"] = boulder_override
    blender.ops.mesh.primitive_cylinder_add(vertices=10, radius=0.18, depth=2.2, rotation=(0, math.pi / 2, 0))
    templates["fallen_log"] = blender.context.object
    templates["fallen_log"].data.materials.append(materials["bark"])
    for kind, template in templates.items():
        if kind != "rock":
            template.name = f"Template_{kind}"
        template.hide_render = True
        template.hide_viewport = True
        if boulder_override is None or kind != "rock":
            _move_to_collection(template, template_collection)
        _set_visual_only(template, f"template.{kind}", f"{kind}_template")
        if kind == "rock" and boulder_override is None:
            _set_asset_slot_metadata(template, "forest.asset.boulder")
    for item in layout["dressing"]:
        if item["kind"] == "understory":
            visual_kind = (
                "fern"
                if _stable_unit(int(layout["seed"]), "understory-visual", int(item["stable_id"].rsplit(".", 1)[-1]))
                < 0.58
                else "grass"
            )
            obj = _link_visual_hierarchy(
                visual_templates[visual_kind],
                item["stable_id"].replace(".", "_"),
                collection,
                item["stable_id"],
                f"understory.{visual_kind}",
            )
        else:
            visual_kind = str(item.get("visual_kind", item["kind"]))
            obj = _link_object(templates[visual_kind], item["stable_id"].replace(".", "_"), collection)
        visual_kind = str(item.get("visual_kind", item["kind"]))
        semantic_class = "boulder" if visual_kind == "rock" else visual_kind
        obj.location = asset_slot_position_m(
            item["position_m"],
            semantic_class,
            recipe.get("external_asset_slots", {}),
        )
        obj.rotation_euler[2] = math.radians(item["yaw_deg"])
        scale = item["scale"]
        obj.scale = (scale, scale, scale)
        obj.hide_render = False
        obj.hide_viewport = False
        _set_visual_only(obj, item["stable_id"], str(item["kind"]))

    if "trail" not in layout:
        return
    height_at = (
        _canonical_height_sampler(recipe)
        if recipe.get("terrain_height_mode") == "canonical_package"
        else lambda x, y: _height_at(x, y, int(layout["seed"]))
    )
    marker_offset = max(2.5, float(layout["trail"]["clearance_radius_m"]) - 0.75)
    marker_placements = inspection_marker_placements(
        layout,
        first_distance_m=25.0,
        spacing_m=125.0,
    )
    for marker_index, placement in enumerate(marker_placements):
        point = placement["position_m"]
        dx, dy = placement["tangent_m"]
        length = max(math.hypot(dx, dy), 1e-9)
        nx, ny = -dy / length, dx / length
        x = point[0] + nx * marker_offset
        y = point[1] + ny * marker_offset
        stable_id = f"forest.inspection.marker.{marker_index:03d}"
        marker = _link_visual_hierarchy(
            visual_templates["inspection_marker"],
            stable_id.replace(".", "_"),
            collection,
            stable_id,
            "inspection.marker",
        )
        marker.location = (x, y, height_at(x, y))
        marker.rotation_euler[2] = math.atan2(dy, dx)


def _add_lighting_and_cameras(
    recipe: Mapping[str, Any],
    collections: Mapping[str, Any],
    layout: Mapping[str, Any],
) -> dict[str, Any]:
    blender = _require_bpy()
    atmosphere = setup_morning_fog(blender, collections["Lighting"])
    height_at = (
        _canonical_height_sampler(recipe)
        if recipe.get("terrain_height_mode") == "canonical_package"
        else lambda x, y: _height_at(x, y, int(layout["seed"]))
    )
    fog = atmosphere["fog"]
    extent_x, extent_y = layout["extent_m"]
    fog.location = (0.0, 0.0, height_at(0.0, 0.0) + 45.0)
    fog.scale = (extent_x * 2.1, extent_y * 2.1, 120.0)

    cameras = create_review_cameras(blender, collections["Cameras"], layout)
    specs = review_camera_specs(layout)
    from mathutils import Vector  # type: ignore[import-not-found]

    for name, camera in cameras.items():
        spec = specs[name]
        location = list(spec["location_m"])
        target = list(spec["target_m"])
        location[2] += height_at(location[0], location[1])
        target[2] += height_at(target[0], target[1])
        camera.location = location
        camera.rotation_euler = (Vector(target) - camera.location).to_track_quat("-Z", "Y").to_euler()
    return cameras


def _ue_transform_to_blender(
    location_cm: Sequence[float],
    quaternion_xyzw: Sequence[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Convert Unreal centimetres/Y-right pose data into Blender metres/Y-left."""

    if len(location_cm) != 3 or len(quaternion_xyzw) != 4:
        raise ValueError("review robot pose must contain xyz and xyzw values")
    x, y, z = (float(value) for value in location_cm)
    qx, qy, qz, qw = (float(value) for value in quaternion_xyzw)
    values = (x, y, z, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("review robot pose values must be finite")
    return (x / 100.0, -y / 100.0, z / 100.0), (qw, -qx, qy, -qz)


def _review_robot_material(blender: Any, asset: Mapping[str, Any], cache: dict[str, Any]) -> Any | None:
    projection = asset.get("visual_projection")
    if not isinstance(projection, Mapping):
        return None
    material_spec = projection.get("material")
    if not isinstance(material_spec, Mapping):
        return None
    key = str(material_spec.get("key", "review_robot"))
    if key in cache:
        return cache[key]
    pbr = material_spec.get("pbr")
    if not isinstance(pbr, Mapping):
        return None
    color = pbr.get("base_color_rgba")
    if not isinstance(color, Sequence) or len(color) != 4:
        return None
    material = blender.data.materials.new(f"M_ReviewRobot_{key}")
    material.diffuse_color = tuple(float(value) for value in color)
    material.use_nodes = True
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    if bsdf is not None:
        bsdf.inputs["Base Color"].default_value = material.diffuse_color
        bsdf.inputs["Roughness"].default_value = float(pbr.get("roughness", 0.65))
        bsdf.inputs["Metallic"].default_value = float(pbr.get("metallic", 0.0))
    cache[key] = material
    return material


def _import_review_robot(
    asset_index: Path | str,
    recipe: Mapping[str, Any],
    layout: Mapping[str, Any],
    collection: Any,
) -> Any:
    """Import the verified Thunder assembly into the review-only collection."""

    blender = _require_bpy()
    contract = load_review_robot_asset_index(asset_index)
    if contract["runtime_recipe"] is None or contract["visual_projection"] is None:
        raise ValueError("review robot rendering requires runtime recipe and visual projection companions")
    from mathutils import Quaternion, Vector  # type: ignore[import-not-found]

    robot_root = blender.data.objects.new("LT_Review_ThunderV4", None)
    collection.objects.link(robot_root)
    _set_visual_only(robot_root, "review.thunderv4", "review.robot.thunderv4")
    robot_root["review_only"] = True
    composition = hero_patrol_composition(layout)
    robot_location = list(composition["robot_location_m"])
    height_at = (
        _canonical_height_sampler(recipe)
        if recipe.get("terrain_height_mode") == "canonical_package"
        else lambda x, y: _height_at(x, y, int(layout["seed"]))
    )
    robot_location[2] += height_at(robot_location[0], robot_location[1])
    robot_root.location = robot_location
    robot_root.rotation_euler[2] = math.radians(float(composition["robot_yaw_deg"]))

    material_cache: dict[str, Any] = {}
    imported_meshes: list[Any] = []
    for asset in contract["assets"]:
        component = asset["runtime_component"]
        before = {id(obj) for obj in blender.data.objects}
        _require_finished(
            blender.ops.import_scene.fbx(filepath=str(asset["fbx_path"])),
            operation=f"review robot FBX import ({asset['asset_name']})",
        )
        imported = [obj for obj in blender.data.objects if id(obj) not in before]
        if not imported:
            raise RuntimeError(f"review robot FBX import created no objects: {asset['asset_name']}")
        top_level = [obj for obj in imported if obj.parent is None or obj.parent not in imported]
        part_root = blender.data.objects.new(f"LT_Review_{asset['asset_name']}", None)
        collection.objects.link(part_root)
        _set_visual_only(part_root, str(component["stable_id"]), "review.robot.part")
        location, quaternion = _ue_transform_to_blender(
            component["location_cm"],
            component["quaternion_xyzw"],
        )
        part_root.location = location
        part_root.rotation_mode = "QUATERNION"
        part_root.rotation_quaternion = Quaternion(quaternion)
        part_root.scale = tuple(float(value) for value in component.get("scale", (1.0, 1.0, 1.0)))
        part_root.parent = robot_root
        material = _review_robot_material(blender, asset, material_cache)
        for object_index, obj in enumerate(imported):
            _move_to_collection(obj, collection)
            _set_visual_only(
                obj,
                f"{component['stable_id']}/imported/{object_index:02d}",
                "review.robot.mesh",
            )
            obj["review_only"] = True
            if material is not None and getattr(obj, "type", None) == "MESH":
                obj.data.materials.clear()
                obj.data.materials.append(material)
            if getattr(obj, "type", None) == "MESH":
                imported_meshes.append(obj)
        for obj in top_level:
            obj.parent = part_root
    blender.context.view_layer.update()
    if not imported_meshes:
        raise RuntimeError("review robot import created no mesh objects")
    lowest_world_z = min(
        (obj.matrix_world @ Vector(corner)).z
        for obj in imported_meshes
        for corner in obj.bound_box
    )
    desired_ground_z = height_at(robot_location[0], robot_location[1]) + 0.01
    robot_root.location.z += review_robot_grounding_offset(lowest_world_z, desired_ground_z - 0.01)
    blender.context.view_layer.update()
    return robot_root


def review_render_targets(
    cameras: Mapping[str, Any],
    *,
    selected_camera: str | None = None,
) -> list[dict[str, str]]:
    """Return the default four-image plan or one explicitly selected review image."""

    defaults = (
        ("robot_eye", "robot_eye_preview", "forest_hf.robot-eye.png"),
        ("establishing", "establishing_preview", "forest_hf.establishing.png"),
        ("ridge", "ridge_preview", "forest_hf.ridge.png"),
        ("aerial", "aerial_preview", "forest_hf.aerial.png"),
    )
    if selected_camera is None:
        selected = defaults
    else:
        if selected_camera not in cameras:
            raise ValueError(f"unknown review camera: {selected_camera}")
        filename = f"forest_hf.{selected_camera.replace('_', '-')}.png"
        selected = ((selected_camera, f"{selected_camera}_preview", filename),)
    return [
        {"camera_id": camera_id, "artifact_role": role, "filename": filename}
        for camera_id, role, filename in selected
    ]


def _artifact_record(role: str, path: Path, output_dir: Path) -> dict[str, Any]:
    payload = path.read_bytes()
    return {
        "role": role,
        "path": path.relative_to(output_dir).as_posix(),
        "bytes": len(payload),
        "sha256": _sha256(payload),
    }


@contextmanager
def _local_blender_export_tempdir(output_dir: Path):
    """Keep Blender exporter scratch files on the artifact volume.

    Some managed hosts expose a process-wide temporary directory whose ACL is
    not readable by Blender's exporter subprocesses.  ``tempfile.tempdir`` is
    intentionally scoped to the export transaction and restored even when an
    exporter fails.
    """

    export_tempdir = (output_dir / ".export-tmp").resolve()
    try:
        export_tempdir.mkdir(mode=0o777, parents=True, exist_ok=True)
        if not export_tempdir.is_dir():
            raise OSError("path is not a directory")
        descriptor, probe_name = tempfile.mkstemp(prefix="forest-export-", dir=export_tempdir)
        with open(descriptor, "wb", closefd=True) as probe:
            probe.write(b"forest-export-tempdir-probe")
        probe_path = Path(probe_name)
        if probe_path.read_bytes() != b"forest-export-tempdir-probe":
            raise OSError("temporary directory probe was not readable")
        probe_path.unlink()
    except OSError as exc:
        raise RuntimeError(f"cannot use Blender export temp directory: {export_tempdir}") from exc

    class WorkspaceTemporaryDirectory:
        """TemporaryDirectory-compatible 0o777 unique directory for Blender exports."""

        def __init__(
            self,
            _suffix: object = None,
            prefix: object = None,
            _dir: object = None,
            ignore_cleanup_errors: bool = False,
            **_kwargs: object,
        ) -> None:
            safe_prefix = str(prefix) if prefix else "gltf-"
            safe_prefix = re.sub(r"[^A-Za-z0-9_.-]", "-", safe_prefix)
            self._ignore_cleanup_errors = ignore_cleanup_errors
            self._path = export_tempdir / f"{safe_prefix}{uuid.uuid4().hex}"
            self._path.mkdir(mode=0o777)
            self.name = str(self._path)
            probe = self._path / "acl-probe"
            probe.write_bytes(b"ok")
            if probe.read_bytes() != b"ok":
                raise OSError(f"Blender export temporary directory is unreadable: {self._path}")
            probe.unlink()

        def __enter__(self) -> str:
            return self.name

        def __exit__(self, *_args: object) -> bool:
            self.cleanup()
            return False

        def cleanup(self) -> None:
            try:
                shutil.rmtree(self._path)
            except OSError:
                if not self._ignore_cleanup_errors:
                    raise

    original_tempdir = tempfile.tempdir
    original_temporary_directory = tempfile.TemporaryDirectory
    tempfile.tempdir = str(export_tempdir)
    tempfile.TemporaryDirectory = WorkspaceTemporaryDirectory
    try:
        yield export_tempdir
    finally:
        tempfile.TemporaryDirectory = original_temporary_directory
        tempfile.tempdir = original_tempdir


def _require_finished(result: object, *, operation: str) -> None:
    """Reject Blender operators that cancel while the host process still exits zero."""

    if not isinstance(result, set) or "FINISHED" not in result:
        raise RuntimeError(f"Blender {operation} failed: operator returned {result!r}")


def _export_asset_slot_files(output_dir: Path) -> list[dict[str, Any]]:
    """Export each normalized mother mesh as portable GLB and Unreal FBX."""

    blender = _require_bpy()
    artifacts: list[dict[str, Any]] = []
    for _, object_name, semantic_class, _, _ in _FALLBACK_ASSET_SLOTS:
        obj = blender.data.objects.get(object_name)
        if obj is None:
            raise RuntimeError(f"missing fallback asset object: {object_name}")
        path = output_dir / _asset_slot_export_filename(semantic_class)
        path.parent.mkdir(parents=True, exist_ok=True)
        hidden_render = obj.hide_render
        hidden_viewport = obj.hide_viewport
        hidden = obj.hide_get()
        blender.ops.object.select_all(action="DESELECT")
        obj.hide_render = False
        obj.hide_viewport = False
        obj.hide_set(False)
        hierarchy = (obj, *obj.children_recursive)
        for member in hierarchy:
            member.hide_render = False
            member.hide_viewport = False
            member.hide_set(False)
            member.select_set(True)
        blender.context.view_layer.objects.active = obj
        _require_finished(
            blender.ops.export_scene.gltf(
                filepath=str(path),
                export_format="GLB",
                use_selection=True,
                export_apply=True,
                export_extras=True,
                export_cameras=False,
                export_lights=False,
            ),
            operation=f"GLB asset-slot export ({semantic_class})",
        )
        validate_exported_glb(path, forbid_template_nodes=False)
        artifacts.append(_artifact_record(_asset_slot_export_role(semantic_class), path, output_dir))
        fbx_path = output_dir / _asset_slot_unreal_filename(semantic_class)
        _require_finished(
            blender.ops.export_scene.fbx(
                filepath=str(fbx_path),
                **_unreal_fbx_export_kwargs(use_selection=True),
            ),
            operation=f"FBX asset-slot export ({semantic_class})",
        )
        artifacts.append(_artifact_record(_asset_slot_unreal_role(semantic_class), fbx_path, output_dir))
        for member in hierarchy:
            member.select_set(False)
            member.hide_render = True
            member.hide_viewport = True
            member.hide_set(True)
        obj.hide_render = hidden_render
        obj.hide_viewport = hidden_viewport
        obj.hide_set(hidden)
    return artifacts


def _select_scene_export_objects(collections: Mapping[str, Any]) -> None:
    blender = _require_bpy()
    blender.ops.object.select_all(action="DESELECT")
    for collection_name in ("Terrain", "Trail", "Trees", "Dressing", "Lighting", "Cameras"):
        for obj in collections[collection_name].all_objects:
            obj.hide_set(False)
            obj.select_set(True)
    review_actors = collections.get("ReviewActors")
    if review_actors is not None:
        for obj in review_actors.all_objects:
            obj.select_set(False)
            obj.hide_set(True)


def _export_scene_files(collections: Mapping[str, Any], output_dir: Path) -> list[dict[str, Any]]:
    """Export scene and asset-slot interchange files in one local-temp transaction."""

    blender = _require_bpy()
    artifacts: list[dict[str, Any]] = []
    with _local_blender_export_tempdir(output_dir):
        glb_path = output_dir / "forest_hf.glb"
        _select_scene_export_objects(collections)
        _require_finished(
            blender.ops.export_scene.gltf(
                filepath=str(glb_path),
                export_format="GLB",
                use_selection=True,
                export_apply=True,
                export_extras=True,
                export_cameras=True,
                export_lights=True,
            ),
            operation="GLB scene export",
        )
        validate_exported_glb(glb_path)
        artifacts.append(_artifact_record("portable_scene_glb", glb_path, output_dir))

        fbx_path = output_dir / "forest_hf.fbx"
        _select_scene_export_objects(collections)
        _require_finished(
            blender.ops.export_scene.fbx(
                filepath=str(fbx_path),
                **_unreal_fbx_export_kwargs(use_selection=True),
            ),
            operation="FBX scene export",
        )
        artifacts.append(_artifact_record("unreal_scene_fbx", fbx_path, output_dir))
        artifacts.extend(_export_asset_slot_files(output_dir))
    return artifacts


def build_forest_scene(
    recipe: Mapping[str, Any],
    layout: Mapping[str, Any],
    output_dir: Path,
    *,
    width: int,
    height: int,
    samples: int,
    review_robot_asset_index: Path | None = None,
    review_camera: str | None = None,
) -> dict[str, Any]:
    """Author, texture, render, export, and hash the Forest_HF Blender scene."""

    blender = _require_bpy()
    output_dir.mkdir(parents=True, exist_ok=True)
    collections = _setup_scene(width, height, samples)
    texture_assets: list[tuple[str, Path]] = []
    material_specs = procedural_material_specs()
    material_ids = tuple(material_specs)
    visual_materials: dict[str, Any] = {}
    for material_index, material_id in enumerate(material_ids):
        normal_strength = float(material_specs[material_id].get("normal_strength", 0.22))
        paths, material = _create_texture_set(
            output_dir,
            material_id,
            recipe["seed"] + material_index,
            outdoor_material_base_color(material_id, str(material_specs[material_id]["palette"][0])),
            normal_strength=normal_strength,
            emission_strength=float(material_specs[material_id].get("emission_strength", 0.0)),
        )
        visual_materials[material_id] = material
        for mode, path in paths.items():
            texture_assets.append((f"texture_{material_id}_{mode}", path))
    ground = visual_materials["forest_ground"]
    trail_material = visual_materials["muddy_trail"]
    rock_material = visual_materials["rock"]
    _add_terrain(recipe, layout, collections["Terrain"], ground)
    _add_trail(recipe, layout, collections["Trail"], trail_material)

    visual_templates = build_visual_templates(blender, collections["Templates"], visual_materials)
    _set_asset_slot_metadata(visual_templates["pine"], "forest.asset.pine")
    _set_asset_slot_metadata(visual_templates["birch"], "forest.asset.birch")
    visual_templates = _import_external_asset_templates(
        recipe.get("external_asset_slots", {}),
        collections["Templates"],
        visual_templates,
    )
    for visual_kind, template in visual_templates.items():
        _hide_visual_template(template, f"template.{visual_kind}")
    for item in layout["instances"]:
        species_id = str(item["species_id"])
        obj = _link_visual_hierarchy(
            visual_templates[species_id],
            item["stable_id"].replace(".", "_"),
            collections["Trees"],
            item["stable_id"],
            f"tree.{species_id}",
        )
        obj.location = asset_slot_position_m(
            item["position_m"],
            species_id,
            recipe.get("external_asset_slots", {}),
        )
        obj.rotation_euler[2] = math.radians(item["yaw_deg"])
        reference_height, reference_crown = {"pine": (8.0, 1.85), "birch": (7.0, 1.15)}[species_id]
        crown_scale = item["crown_radius_m"] / reference_crown
        organic_scale = item["scale_xyz"]
        obj.scale = (
            crown_scale * float(organic_scale[0]),
            crown_scale * float(organic_scale[1]),
            item["height_m"] / reference_height * float(organic_scale[2]),
        )
        obj.hide_render = False
        obj.hide_viewport = False
    _add_dressing(
        recipe,
        layout,
        collections["Dressing"],
        collections["Templates"],
        {"rock": rock_material, "bark": visual_materials["pine_bark"]},
        visual_templates,
        {
            semantic_class: visual_templates[semantic_class]
            for semantic_class in recipe.get("external_asset_slots", {})
            if semantic_class == "boulder"
        },
    )
    cameras = _add_lighting_and_cameras(recipe, collections, layout)
    if review_robot_asset_index is not None:
        _import_review_robot(
            review_robot_asset_index,
            recipe,
            layout,
            collections["ReviewActors"],
        )

    artifacts: list[dict[str, Any]] = []
    for target in review_render_targets(cameras, selected_camera=review_camera):
        camera_id = target["camera_id"]
        role = target["artifact_role"]
        camera = cameras[camera_id]
        show_review_actors = camera_id == "hero_patrol"
        for obj in collections["ReviewActors"].all_objects:
            obj.hide_render = not show_review_actors
        blender.context.scene.camera = camera
        path = output_dir / target["filename"]
        blender.context.scene.render.filepath = str(path)
        blender.ops.render.render(write_still=True)
        artifacts.append(_artifact_record(role, path, output_dir))

    blend_path = output_dir / "forest_hf.blend"
    blender.ops.wm.save_as_mainfile(filepath=str(blend_path))
    artifacts.append(_artifact_record("editable_blend", blend_path, output_dir))
    artifacts.extend(_export_scene_files(collections, output_dir))
    artifacts.extend(_artifact_record(role, path, output_dir) for role, path in texture_assets)
    authored_recipe = {**recipe, "materials": generated_material_contracts(material_ids)}
    manifest = build_authoring_manifest(authored_recipe, layout, artifacts, require_material_artifacts=True)
    manifest_path = output_dir / "authoring.manifest.json"
    manifest_path.write_bytes(_canonical_json(manifest))
    layout_path = output_dir / "forest-layout.json"
    layout_path.write_bytes(_canonical_json(layout))
    return {
        "schema": "lingtu.sim.blender-authoring-result.v1",
        "world_package": WORLD_PACKAGE,
        "output_dir": str(output_dir),
        "manifest": str(manifest_path),
        "layout": str(layout_path),
        "artifact_set_digest": manifest["artifact_set_digest"],
        "assets": manifest["assets"],
    }


def export_existing_blend(
    recipe: Mapping[str, Any],
    layout: Mapping[str, Any],
    output_dir: Path,
    blend_path: Path,
) -> dict[str, Any]:
    """Re-export a previously authored scene without regenerating or rendering it."""

    blender = _require_bpy()
    source_blend = blend_path.resolve()
    if not source_blend.is_file():
        raise FileNotFoundError(f"existing Blender scene does not exist: {source_blend}")
    output_dir.mkdir(parents=True, exist_ok=True)
    blender.ops.wm.open_mainfile(filepath=str(source_blend))

    collection_names = ("Terrain", "Trail", "Trees", "Dressing", "Templates", "Lighting", "Cameras")
    collections = {name: blender.data.collections.get(name) for name in collection_names}
    missing = [name for name, collection in collections.items() if collection is None]
    if missing:
        raise RuntimeError(f"existing Blender scene is missing required collections: {', '.join(missing)}")

    destination_blend = (output_dir / "forest_hf.blend").resolve()
    if destination_blend != source_blend:
        blender.ops.wm.save_as_mainfile(filepath=str(destination_blend))

    artifacts = [_artifact_record("editable_blend", destination_blend, output_dir)]
    artifacts.extend(_export_scene_files(collections, output_dir))

    material_ids = tuple(procedural_material_specs())
    for material_id in material_ids:
        for mode in ("basecolor", "normal", "orm"):
            texture_path = output_dir / "textures" / "forest" / f"{material_id}_{mode}.png"
            artifacts.append(_artifact_record(f"texture_{material_id}_{mode}", texture_path, output_dir))
    for role, filename in (
        ("robot_eye_preview", "forest_hf.robot-eye.png"),
        ("establishing_preview", "forest_hf.establishing.png"),
        ("ridge_preview", "forest_hf.ridge.png"),
        ("aerial_preview", "forest_hf.aerial.png"),
    ):
        artifacts.append(_artifact_record(role, output_dir / filename, output_dir))

    authored_recipe = {**recipe, "materials": generated_material_contracts(material_ids)}
    manifest = build_authoring_manifest(authored_recipe, layout, artifacts, require_material_artifacts=True)
    manifest_path = output_dir / "authoring.manifest.json"
    manifest_path.write_bytes(_canonical_json(manifest))
    layout_path = output_dir / "forest-layout.json"
    layout_path.write_bytes(_canonical_json(layout))
    return {
        "schema": "lingtu.sim.blender-authoring-result.v1",
        "world_package": WORLD_PACKAGE,
        "output_dir": str(output_dir),
        "manifest": str(manifest_path),
        "layout": str(layout_path),
        "artifact_set_digest": manifest["artifact_set_digest"],
        "assets": manifest["assets"],
    }


def parse_cli_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse Blender-background or ordinary Python validation arguments."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--recipe", type=Path, default=DEFAULT_RECIPE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--samples", type=int, default=32)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument(
        "--review-robot-asset-index",
        type=Path,
        default=None,
        help="opt-in Thunder review asset-index.json; never included in scene exports",
    )
    parser.add_argument(
        "--hero-robot-asset-root",
        type=Path,
        default=None,
        help="opt-in Thunder review asset directory containing asset-index.json",
    )
    parser.add_argument(
        "--review-camera",
        choices=("robot_eye", "establishing", "ridge", "aerial", "hero_patrol"),
        default=None,
        help="render only the selected review camera; default behavior renders the original four",
    )
    parser.add_argument(
        "--export-existing-blend",
        type=Path,
        help="load an existing Forest_HF .blend and rebuild exports/manifest without rendering",
    )
    raw_args = list(sys.argv[1:] if argv is None else argv)
    if "--" in raw_args:
        raw_args = raw_args[raw_args.index("--") + 1 :]
    args = parser.parse_args(raw_args)
    if args.width < 64 or args.height < 64 or args.samples < 1:
        parser.error("width/height must be >= 64 and samples must be >= 1")
    args.repo_root = args.repo_root.resolve()
    args.recipe = args.recipe if args.recipe.is_absolute() else args.repo_root / args.recipe
    args.output_dir = args.output_dir if args.output_dir.is_absolute() else args.repo_root / args.output_dir
    if args.export_existing_blend is not None and not args.export_existing_blend.is_absolute():
        args.export_existing_blend = args.repo_root / args.export_existing_blend
    if args.hero_robot_asset_root is not None:
        if args.review_robot_asset_index is not None:
            parser.error("use only one review robot asset path option")
        root = (
            args.hero_robot_asset_root
            if args.hero_robot_asset_root.is_absolute()
            else args.repo_root / args.hero_robot_asset_root
        )
        args.hero_robot_asset_root = root.resolve()
        args.review_robot_asset_index = args.hero_robot_asset_root / "asset-index.json"
    elif args.review_robot_asset_index is not None:
        path = args.review_robot_asset_index
        args.review_robot_asset_index = (path if path.is_absolute() else args.repo_root / path).resolve()
    return args


def main(argv: Sequence[str] | None = None) -> int:
    """Validate a recipe or author its Blender scene and portable artifacts."""

    args = parse_cli_args(argv)
    recipe = load_forest_recipe(args.recipe)
    layout = generate_forest_layout(recipe)
    review_robot_contract = (
        load_review_robot_asset_index(args.review_robot_asset_index)
        if args.review_robot_asset_index is not None
        else None
    )
    if args.validate_only:
        print(
            json.dumps(
                {
                    "schema": "lingtu.sim.forest-authoring-validation.v1",
                    "world_package": WORLD_PACKAGE,
                    "tree_count": len(layout["instances"]),
                    "dressing_count": len(layout["dressing"]),
                    "layout_digest": layout["digest"],
                    "bpy_available": BPY_AVAILABLE,
                    "review_robot_asset_count": (
                        len(review_robot_contract["assets"])
                        if review_robot_contract is not None
                        else 0
                    ),
                },
                sort_keys=True,
            )
        )
        return 0
    if args.export_existing_blend is not None:
        result = export_existing_blend(recipe, layout, args.output_dir, args.export_existing_blend)
    else:
        result = build_forest_scene(
            recipe,
            layout,
            args.output_dir,
            width=args.width,
            height=args.height,
            samples=args.samples,
            review_robot_asset_index=args.review_robot_asset_index,
            review_camera=args.review_camera,
        )
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised through Blender/Python CLI.
    raise SystemExit(main())
