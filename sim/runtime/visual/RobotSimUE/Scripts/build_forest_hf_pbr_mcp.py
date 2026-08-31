"""Bind deterministic Forest_HF PBR textures and materials through UE 5.8 MCP.

Dry-run is the default. ``--apply`` is required before the script connects to
Unreal Editor.  The binder deliberately does not trust materials created by FBX
import: every texture, material graph, mesh slot, terrain type, and collision
boundary is validated against the Blender authoring manifest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
import zlib
from pathlib import Path
from typing import Any, Mapping, NamedTuple, Protocol, Sequence, TypedDict

REPO_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_MANIFEST = REPO_ROOT / "build" / "forest-hf" / "blender" / "authoring.manifest.json"
DEFAULT_ENDPOINT = "http://127.0.0.1:8000/mcp"

WORLD_PACKAGE = "forest_hf@2.0.0"
MANIFEST_SCHEMA = "lingtu.sim.blender-authoring-manifest.v1"
PLAN_SCHEMA = "lingtu.sim.unreal-forest-pbr-plan.v1"
RESULT_SCHEMA = "lingtu.sim.unreal-forest-pbr-result.v1"
FOREST_AUTHORING_PLAN_SCHEMA = "lingtu.sim.unreal-forest-mcp-authoring-plan.v1"
FOREST_POINTS_SCHEMA = "lingtu.sim.unreal-forest-create-points.v2"
LANDSCAPE_IMPORT_RECEIPT_SCHEMA = "lingtu.sim.unreal-landscape-import-receipt.v1"
MAP_PATH = "/Game/RobotSim/Maps/Forest_HF_2km"
OPEN_WORLD_TEMPLATE = "/Engine/Maps/Templates/OpenWorld"
ASSET_ROOT = "/Game/RobotSim/Worlds/ForestHF2km"
MATERIAL_FOLDER = f"{ASSET_ROOT}/Materials"
TEXTURE_FOLDER = f"{MATERIAL_FOLDER}/Textures"

ASSET_TOOLS = "editor_toolset.toolsets.asset.AssetTools"
SCENE_TOOLS = "editor_toolset.toolsets.scene.SceneTools"
ACTOR_TOOLS = "editor_toolset.toolsets.actor.ActorTools"
OBJECT_TOOLS = "editor_toolset.toolsets.object.ObjectTools"
STATIC_MESH_TOOLS = "editor_toolset.toolsets.static_mesh.StaticMeshTools"
TEXTURE_TOOLS = "editor_toolset.toolsets.texture.TextureTools"
MATERIAL_TOOLS = "editor_toolset.toolsets.material.MaterialTools"

TEXTURE_SAMPLE_CLASS = "/Script/Engine.MaterialExpressionTextureSample"
STATIC_MESH_ACTOR_CLASS = "/Script/Engine.StaticMeshActor"
STATIC_MESH_COMPONENT_CLASS = "/Script/Engine.StaticMeshComponent"
LANDSCAPE_CLASS = "/Script/Landscape.Landscape"
LANDSCAPE_COMPONENT_CLASS = "/Script/Landscape.LandscapeComponent"
LANDSCAPE_COLLISION_COMPONENT_CLASS = "/Script/Landscape.LandscapeHeightfieldCollisionComponent"
HISM_COMPONENT_CLASS = "/Script/Engine.HierarchicalInstancedStaticMeshComponent"

VALIDATION_PROFILE = "validation257_static"
PRODUCTION_PROFILE = "production4033_landscape"
VALIDATION_TERRAIN_LABEL = "ForestTerrain_2km_Validation257"
PRODUCTION_TERRAIN_LABEL = "ForestTerrain_2km_Production4033"
VALIDATION_TERRAIN_MESH = f"{ASSET_ROOT}/Terrain/SM_ForestTerrain_2km_Validation257"
VALIDATION_TERRAIN_MESH_REF = f"{VALIDATION_TERRAIN_MESH}.SM_ForestTerrain_2km_Validation257"

MATERIAL_NAMES = {
    "forest_ground": "M_ForestGround_PBR",
    "pine_bark": "M_PineBark_PBR",
    "pine_needles": "M_PineNeedles_PBR",
    "birch_bark": "M_BirchBark_PBR",
    "birch_leaves": "M_BirchLeaves_PBR",
}
TEXTURE_STEMS = {
    "base_color": "BaseColor",
    "normal": "Normal",
    "orm": "ORM",
}
TEXTURE_SETTINGS = {
    "base_color": {"sRGB": True, "compressionSettings": "TC_Default", "samplerType": "SAMPLERTYPE_Color"},
    "normal": {"sRGB": False, "compressionSettings": "TC_Normalmap", "samplerType": "SAMPLERTYPE_Normal"},
    "orm": {"sRGB": False, "compressionSettings": "TC_Masks", "samplerType": "SAMPLERTYPE_Masks"},
}
EXPECTED_BINDINGS = {
    "base_color": {"texture": "base_color", "color_space": "sRGB"},
    "normal": {"texture": "normal", "color_space": "Non-Color", "compression": "NormalMap"},
    "ambient_occlusion": {"texture": "orm", "channel": "R"},
    "roughness": {"texture": "orm", "channel": "G"},
    "metallic": {"texture": "orm", "channel": "B"},
}
MATERIAL_OUTPUTS = {
    "base_color": ("RGB", "MP_BaseColor"),
    "normal": ("RGB", "MP_Normal"),
    "ambient_occlusion": ("R", "MP_AmbientOcclusion"),
    "roughness": ("G", "MP_Roughness"),
    "metallic": ("B", "MP_Metallic"),
}


class TreeMeshSpec(TypedDict):
    """Exact static-mesh destination and material-slot contract for one species."""

    asset_path: str
    asset_ref: str
    slots: dict[str, str]


TREE_MESHES: dict[str, TreeMeshSpec] = {
    "pine": {
        "asset_path": f"{ASSET_ROOT}/Assets/SM_Forest_Pine_final",
        "asset_ref": f"{ASSET_ROOT}/Assets/SM_Forest_Pine_final.SM_Forest_Pine_final",
        "slots": {"M_forest_bark": "pine_bark", "M_forest_leaves": "pine_needles"},
    },
    "birch": {
        "asset_path": f"{ASSET_ROOT}/Assets/SM_Forest_Birch_final",
        "asset_ref": f"{ASSET_ROOT}/Assets/SM_Forest_Birch_final.SM_Forest_Birch_final",
        "slots": {"M_forest_bark": "birch_bark", "M_forest_leaves": "birch_leaves"},
    },
}

SOURCE_SHA_TAG = "LingTu.SourceTextureSha256"
MANIFEST_DIGEST_TAG = "LingTu.BlenderManifestDigest"
MATERIAL_ID_TAG = "LingTu.ForestMaterialId"
TEXTURE_CHANNEL_TAG = "LingTu.PbrTextureChannel"
MATERIAL_BINDING_TAG = "LingTu.PbrBindingSha256"
AUTHORITY_TAG = "LingTu.Authority"

TREE_SLOT_TO_SPECIES = {
    "forest.asset.birch": "birch",
    "forest.asset.pine": "pine",
}
EXPECTED_PARTITION_COORDINATES = frozenset((x, y) for x in range(-4, 4) for y in range(-4, 4))


class ForestPbrError(RuntimeError):
    """Base error for fail-closed Forest_HF PBR authoring."""


class PbrInputError(ForestPbrError, ValueError):
    """Raised when local PBR inputs violate the fixed contract."""


class PbrMcpError(ForestPbrError):
    """Raised when a remote MCP response cannot prove the requested state."""


class McpTransport(Protocol):
    """Small transport boundary shared with the forest MCP authoring adapter."""

    def initialize(self) -> None:
        """Open an MCP session."""

    def notify_initialized(self) -> None:
        """Complete MCP session initialization."""

    def call_tool(self, toolset_name: str, tool_name: str, arguments: Mapping[str, object]) -> Any:
        """Call one reviewed Unreal tool."""


class TextureSource(NamedTuple):
    """One hash-bound Blender texture and its deterministic Unreal destination."""

    channel: str
    source_path: Path
    source_relative_path: str
    sha256: str
    byte_count: int
    asset_name: str
    asset_path: str
    asset_ref: str


class MaterialSource(NamedTuple):
    """One exact PBR material graph description."""

    material_id: str
    asset_name: str
    asset_path: str
    asset_ref: str
    textures: tuple[TextureSource, ...]
    binding_sha256: str


class TerrainInput(NamedTuple):
    """Terrain profile and optional production Landscape heightmap identity."""

    profile: str
    actor_label: str
    heightmap_path: Path | None
    heightmap_sha256: str | None
    import_receipt_path: Path | None
    import_receipt_digest: str | None


class ForestInstanceProvenance(NamedTuple):
    """Hash-bound exact PCG instance contract consumed by the HISM audit."""

    plan_path: Path
    plan_sha256: str
    points_path: Path
    points_sha256: str
    point_count: int
    expected_by_mesh: dict[str, int]


class PbrInputs(NamedTuple):
    """Fully validated offline inputs for a PBR authoring transaction."""

    manifest_path: Path
    manifest: dict[str, Any]
    manifest_sha256: str
    manifest_digest: str
    artifact_set_digest: str
    materials: tuple[MaterialSource, ...]
    terrain: TerrainInput
    instances: ForestInstanceProvenance
    binding_set_sha256: str


class TerrainTarget(NamedTuple):
    """Read-only preflight result for the exact terrain representation."""

    profile: str
    actor_path: str
    actor_class: str
    render_components: tuple[str, ...]
    collision_components: tuple[str, ...]
    mesh_path: str | None
    mesh_ref: str | None
    material_slot: str | None
    source_heightmap_sha256: str | None
    source_import_receipt_digest: str | None


def _compact_json(value: object) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def canonical_json_bytes(value: object) -> bytes:
    """Serialize stable pretty JSON exactly like the Blender artifact manifest."""

    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, separators=(",", ": "), allow_nan=False) + "\n"
    ).encode("utf-8")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _ref_path(value: object) -> str | None:
    if isinstance(value, str) and value != "None":
        return value
    if isinstance(value, Mapping):
        ref_path = value.get("refPath")
        if isinstance(ref_path, str):
            return ref_path
    return None


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError as error:
            raise PbrMcpError(f"{label} returned invalid JSON") from error
    if not isinstance(value, Mapping):
        raise PbrMcpError(f"{label} did not return an object")
    return value


def _relative_manifest_path(value: object, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise PbrInputError(f"{label} must be a non-empty relative path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise PbrInputError(f"{label} must remain inside the Blender output directory")
    return path


def _png_info(path: Path) -> tuple[int, int, int, int]:
    """Validate the complete PNG container and decoded scanline shape."""

    try:
        payload = path.read_bytes()
    except OSError as error:
        raise PbrInputError(f"could not read production Landscape heightmap {path}: {error}") from error
    if payload[:8] != b"\x89PNG\r\n\x1a\n":
        raise PbrInputError("production Landscape heightmap must be a complete valid PNG")

    offset = 8
    ihdr: tuple[int, int, int, int, int, int, int] | None = None
    idat_parts: list[bytes] = []
    idat_finished = False
    saw_iend = False
    chunk_index = 0
    while offset < len(payload):
        if len(payload) - offset < 12:
            raise PbrInputError("production Landscape heightmap has a truncated PNG chunk")
        length = struct.unpack(">I", payload[offset : offset + 4])[0]
        chunk_type = payload[offset + 4 : offset + 8]
        end = offset + 12 + length
        if end > len(payload):
            raise PbrInputError("production Landscape heightmap has a truncated PNG chunk payload")
        data = payload[offset + 8 : offset + 8 + length]
        observed_crc = struct.unpack(">I", payload[offset + 8 + length : end])[0]
        if zlib.crc32(chunk_type + data) & 0xFFFFFFFF != observed_crc:
            raise PbrInputError("production Landscape heightmap has an invalid PNG chunk CRC")
        if chunk_index == 0 and chunk_type != b"IHDR":
            raise PbrInputError("production Landscape heightmap PNG must start with IHDR")
        if chunk_type == b"IHDR":
            if ihdr is not None or length != 13:
                raise PbrInputError("production Landscape heightmap has an invalid PNG IHDR")
            ihdr = struct.unpack(">IIBBBBB", data)
        elif chunk_type == b"IDAT":
            if ihdr is None or idat_finished:
                raise PbrInputError("production Landscape heightmap has invalid PNG IDAT ordering")
            idat_parts.append(data)
        elif chunk_type == b"IEND":
            if length != 0 or not idat_parts or end != len(payload):
                raise PbrInputError("production Landscape heightmap has an invalid PNG IEND")
            saw_iend = True
            offset = end
            break
        else:
            if idat_parts:
                idat_finished = True
            if chunk_type[:1].isupper():
                raise PbrInputError(f"production Landscape heightmap contains unsupported PNG chunk {chunk_type!r}")
        offset = end
        chunk_index += 1

    if ihdr is None or not saw_iend or offset != len(payload):
        raise PbrInputError("production Landscape heightmap must contain complete IHDR/IDAT/IEND data")
    width, height, bit_depth, color_type, compression, filter_method, interlace = ihdr
    if width <= 0 or height <= 0 or width * height > 20_000_000:
        raise PbrInputError("production Landscape heightmap dimensions are invalid")
    channels_by_color_type = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}
    legal_depths = {0: {1, 2, 4, 8, 16}, 2: {8, 16}, 3: {1, 2, 4, 8}, 4: {8, 16}, 6: {8, 16}}
    if (
        color_type not in channels_by_color_type
        or bit_depth not in legal_depths[color_type]
        or compression != 0
        or filter_method != 0
        or interlace != 0
    ):
        raise PbrInputError("production Landscape heightmap uses unsupported PNG encoding")
    row_bytes = (width * channels_by_color_type[color_type] * bit_depth + 7) // 8
    expected_bytes = height * (row_bytes + 1)
    try:
        decompressor = zlib.decompressobj()
        decoded = decompressor.decompress(b"".join(idat_parts), expected_bytes + 1)
        decoded += decompressor.flush()
    except zlib.error as error:
        raise PbrInputError("production Landscape heightmap PNG pixel data are corrupt") from error
    if len(decoded) != expected_bytes or not decompressor.eof or decompressor.unused_data:
        raise PbrInputError("production Landscape heightmap PNG pixel data are incomplete or oversized")
    if any(decoded[row * (row_bytes + 1)] > 4 for row in range(height)):
        raise PbrInputError("production Landscape heightmap contains an invalid PNG row filter")
    return width, height, bit_depth, color_type


def _read_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise PbrInputError(f"could not load {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise PbrInputError(f"{label} root must be an object")
    return value


def _load_landscape_import_receipt(path: Path, heightmap_sha256: str) -> tuple[Path, str]:
    resolved = path.expanduser().resolve()
    receipt = _read_json_object(resolved, "Landscape import receipt")
    digest = receipt.get("digest")
    body = {key: value for key, value in receipt.items() if key != "digest"}
    expected_digest = hashlib.sha256(_compact_json(body).encode("utf-8")).hexdigest()
    expected = {
        "schema": LANDSCAPE_IMPORT_RECEIPT_SCHEMA,
        "world_package": WORLD_PACKAGE,
        "map_path": MAP_PATH,
        "actor_label": PRODUCTION_TERRAIN_LABEL,
        "actor_class": LANDSCAPE_CLASS,
        "heightmap": {
            "sha256": heightmap_sha256,
            "width": 4033,
            "height": 4033,
            "bit_depth": 16,
            "color_type": 0,
        },
        "topology": {
            "grid_components_x": 32,
            "grid_components_y": 32,
            "component_size_quads": 126,
            "subsection_size_quads": 63,
            "num_subsections": 2,
        },
    }
    if digest != expected_digest or any(receipt.get(key) != value for key, value in expected.items()):
        raise PbrInputError("Landscape import receipt is not bound to the exact 4033 heightmap and topology")
    importer = receipt.get("importer")
    if not isinstance(importer, Mapping) or not all(
        isinstance(importer.get(key), str) and importer.get(key) for key in ("tool", "version")
    ):
        raise PbrInputError("Landscape import receipt must identify the importer tool and version")
    return resolved, expected_digest


def _load_forest_instance_provenance(path: Path) -> ForestInstanceProvenance:
    resolved = path.expanduser().resolve()
    plan = _read_json_object(resolved, "Forest MCP authoring plan")
    if (
        plan.get("schema") != FOREST_AUTHORING_PLAN_SCHEMA
        or plan.get("world_package") != WORLD_PACKAGE
        or plan.get("seed") != 20260813
        or plan.get("selection_policy") != "exact_mesh_slot_branches"
    ):
        raise PbrInputError("Forest MCP authoring plan does not match the fixed Forest_HF contract")
    target = plan.get("target")
    inputs = plan.get("inputs")
    if not isinstance(target, Mapping) or target.get("map") != MAP_PATH or not isinstance(inputs, Mapping):
        raise PbrInputError("Forest MCP authoring plan targets the wrong map")
    create_points = inputs.get("create_points")
    if not isinstance(create_points, Mapping):
        raise PbrInputError("Forest MCP authoring plan has no create-points provenance")
    points_path_value = create_points.get("path")
    points_sha256 = create_points.get("sha256")
    if not isinstance(points_path_value, str) or not isinstance(points_sha256, str) or len(points_sha256) != 64:
        raise PbrInputError("Forest MCP authoring plan create-points identity is invalid")
    points_path = Path(points_path_value).expanduser().resolve()
    if not points_path.is_file() or _sha256_file(points_path) != points_sha256:
        raise PbrInputError("Forest MCP create-points file does not match its bound SHA256")
    points = _read_json_object(points_path, "Forest MCP create-points contract")
    slot_counts = create_points.get("slot_point_counts")
    point_count = create_points.get("point_count")
    if (
        points.get("schema") != FOREST_POINTS_SCHEMA
        or points.get("world_package") != WORLD_PACKAGE
        or points.get("seed") != 20260813
        or points.get("selection_policy") != "exact_mesh_slot_branches"
        or points.get("slot_counts") != slot_counts
        or points.get("point_count") != point_count
        or points.get("content_digest") != create_points.get("content_digest")
    ):
        raise PbrInputError("Forest MCP create-points contract does not match its authoring plan")
    if not isinstance(slot_counts, Mapping) or set(slot_counts) != set(TREE_SLOT_TO_SPECIES):
        raise PbrInputError("Forest MCP authoring plan must contain exact pine and birch instance counts")
    if isinstance(point_count, bool) or not isinstance(point_count, int) or point_count <= 0:
        raise PbrInputError("Forest MCP authoring point count is invalid")
    expected_by_mesh: dict[str, int] = {}
    for slot, species in TREE_SLOT_TO_SPECIES.items():
        count = slot_counts.get(slot)
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise PbrInputError(f"Forest MCP instance count for {slot} is invalid")
        expected_by_mesh[str(TREE_MESHES[species]["asset_ref"])] = count
    if sum(expected_by_mesh.values()) != point_count:
        raise PbrInputError("Forest MCP per-mesh counts do not sum to its bound point count")
    slot_assets = inputs.get("slot_assets")
    if isinstance(slot_assets, (str, bytes)) or not isinstance(slot_assets, Sequence):
        raise PbrInputError("Forest MCP authoring plan slot assets are missing")
    observed_slots = {
        item.get("slot"): item.get("asset_ref")
        for item in slot_assets
        if isinstance(item, Mapping) and item.get("slot") in TREE_SLOT_TO_SPECIES
    }
    expected_slots = {slot: TREE_MESHES[species]["asset_ref"] for slot, species in TREE_SLOT_TO_SPECIES.items()}
    if observed_slots != expected_slots:
        raise PbrInputError("Forest MCP authoring plan does not bind the exact pine and birch mesh assets")
    return ForestInstanceProvenance(
        plan_path=resolved,
        plan_sha256=_sha256_file(resolved),
        points_path=points_path,
        points_sha256=points_sha256,
        point_count=point_count,
        expected_by_mesh=expected_by_mesh,
    )


def _verify_manifest_digest(manifest: Mapping[str, object]) -> str:
    digest = manifest.get("digest")
    if not isinstance(digest, str) or len(digest) != 64:
        raise PbrInputError("Blender manifest digest must be a SHA256")
    body = dict(manifest)
    del body["digest"]
    expected = hashlib.sha256(_compact_json(body).encode("utf-8")).hexdigest()
    if digest != expected:
        raise PbrInputError("Blender manifest digest does not cover the current manifest body")
    return digest


def _verify_artifact_set(manifest: Mapping[str, object], root: Path) -> str:
    assets = manifest.get("assets")
    if isinstance(assets, (str, bytes)) or not isinstance(assets, Sequence):
        raise PbrInputError("Blender manifest assets must be an array")
    identity: list[dict[str, str]] = []
    seen: set[tuple[str, str]] = set()
    for index, item in enumerate(assets):
        if not isinstance(item, Mapping):
            raise PbrInputError(f"Blender manifest asset {index} must be an object")
        role = item.get("role")
        if not isinstance(role, str) or not role:
            raise PbrInputError(f"Blender manifest asset {index} role is missing")
        relative = _relative_manifest_path(item.get("path"), f"assets[{index}].path")
        sha256 = item.get("sha256")
        byte_count = item.get("bytes")
        if not isinstance(sha256, str) or len(sha256) != 64:
            raise PbrInputError(f"Blender manifest asset {index} SHA256 is invalid")
        if isinstance(byte_count, bool) or not isinstance(byte_count, int) or byte_count < 0:
            raise PbrInputError(f"Blender manifest asset {index} byte count is invalid")
        key = (role, relative.as_posix())
        if key in seen:
            raise PbrInputError(f"duplicate Blender artifact identity: {key}")
        seen.add(key)
        absolute = (root / relative).resolve()
        if not absolute.is_file() or absolute.stat().st_size != byte_count or _sha256_file(absolute) != sha256:
            raise PbrInputError(f"Blender artifact does not match its manifest identity: {relative.as_posix()}")
        identity.append({"role": role, "path": relative.as_posix(), "sha256": sha256})
    identity.sort(key=lambda item: (item["role"], item["path"]))
    observed = manifest.get("artifact_set_digest")
    expected = hashlib.sha256(canonical_json_bytes(identity)).hexdigest()
    if observed != expected:
        raise PbrInputError("Blender artifact_set_digest does not cover the current artifact identities")
    return expected


def _material_asset_name(material_id: str) -> str:
    return MATERIAL_NAMES[material_id]


def _texture_asset_name(material_id: str, channel: str) -> str:
    material_stem = _material_asset_name(material_id).removeprefix("M_").removesuffix("_PBR")
    return f"T_{material_stem}_{TEXTURE_STEMS[channel]}"


def _load_materials(manifest: Mapping[str, object], root: Path, manifest_digest: str) -> tuple[MaterialSource, ...]:
    records = manifest.get("materials")
    if isinstance(records, (str, bytes)) or not isinstance(records, Sequence):
        raise PbrInputError("Blender manifest materials must be an array")
    by_id: dict[str, Mapping[str, object]] = {}
    for item in records:
        if not isinstance(item, Mapping) or not isinstance(item.get("id"), str):
            raise PbrInputError("Blender manifest contains an invalid material record")
        material_id = str(item["id"])
        if material_id in by_id:
            raise PbrInputError(f"duplicate Blender material id: {material_id}")
        by_id[material_id] = item
    if missing := set(MATERIAL_NAMES) - by_id.keys():
        raise PbrInputError(f"Blender manifest is missing required forest materials: {sorted(missing)}")

    result: list[MaterialSource] = []
    for material_id in MATERIAL_NAMES:
        record = by_id[material_id]
        if record.get("shader") != "principled_pbr" or record.get("bindings") != EXPECTED_BINDINGS:
            raise PbrInputError(f"material {material_id} does not declare the exact base/normal/ORM binding contract")
        textures = record.get("textures")
        if not isinstance(textures, Mapping) or set(textures) != set(TEXTURE_SETTINGS):
            raise PbrInputError(f"material {material_id} must contain exactly base_color, normal, and orm textures")
        texture_sources: list[TextureSource] = []
        for channel in TEXTURE_SETTINGS:
            texture = textures[channel]
            if not isinstance(texture, Mapping):
                raise PbrInputError(f"material {material_id} texture {channel} must be an object")
            relative = _relative_manifest_path(texture.get("path"), f"materials.{material_id}.textures.{channel}.path")
            source_path = (root / relative).resolve()
            sha256 = texture.get("sha256")
            byte_count = texture.get("bytes")
            if not isinstance(sha256, str) or len(sha256) != 64:
                raise PbrInputError(f"material {material_id} texture {channel} SHA256 is invalid")
            if isinstance(byte_count, bool) or not isinstance(byte_count, int) or byte_count <= 0:
                raise PbrInputError(f"material {material_id} texture {channel} byte count is invalid")
            if (
                not source_path.is_file()
                or source_path.stat().st_size != byte_count
                or _sha256_file(source_path) != sha256
            ):
                raise PbrInputError(f"material {material_id} texture {channel} does not match its manifest identity")
            asset_name = _texture_asset_name(material_id, channel)
            asset_path = f"{TEXTURE_FOLDER}/{asset_name}"
            texture_sources.append(
                TextureSource(
                    channel=channel,
                    source_path=source_path,
                    source_relative_path=relative.as_posix(),
                    sha256=sha256,
                    byte_count=byte_count,
                    asset_name=asset_name,
                    asset_path=asset_path,
                    asset_ref=f"{asset_path}.{asset_name}",
                )
            )
        binding_body = {
            "manifest_digest": manifest_digest,
            "material_id": material_id,
            "textures": [
                {"channel": item.channel, "path": item.source_relative_path, "sha256": item.sha256}
                for item in texture_sources
            ],
            "bindings": EXPECTED_BINDINGS,
        }
        binding_sha256 = hashlib.sha256(canonical_json_bytes(binding_body)).hexdigest()
        asset_name = _material_asset_name(material_id)
        asset_path = f"{MATERIAL_FOLDER}/{asset_name}"
        result.append(
            MaterialSource(
                material_id=material_id,
                asset_name=asset_name,
                asset_path=asset_path,
                asset_ref=f"{asset_path}.{asset_name}",
                textures=tuple(texture_sources),
                binding_sha256=binding_sha256,
            )
        )
    return tuple(result)


def load_pbr_inputs(
    manifest_path: Path,
    terrain_profile: str,
    production_heightmap: Path | None = None,
    forest_authoring_plan: Path | None = None,
    production_import_receipt: Path | None = None,
) -> PbrInputs:
    """Validate all local identities before any MCP session can be opened."""

    resolved_manifest = manifest_path.expanduser().resolve()
    try:
        manifest = json.loads(resolved_manifest.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise PbrInputError(f"could not load Blender authoring manifest {resolved_manifest}: {error}") from error
    if not isinstance(manifest, dict):
        raise PbrInputError("Blender authoring manifest root must be an object")
    if manifest.get("schema") != MANIFEST_SCHEMA or manifest.get("world_package") != WORLD_PACKAGE:
        raise PbrInputError("Blender manifest schema or world package is not Forest_HF 2.0.0")
    generator = manifest.get("generator")
    authority = manifest.get("authority")
    if not isinstance(generator, Mapping) or generator.get("deterministic_seed") != 20260813:
        raise PbrInputError("Blender manifest must use deterministic seed 20260813")
    if not isinstance(authority, Mapping) or any(
        authority.get(key) != expected
        for key, expected in {
            "classification": "VisualOnly",
            "unreal_collision_profile": "NoCollision",
            "physics_authority": "mujoco",
            "visual_meshes_are_colliders": False,
            "simulate_physics": False,
            "can_ever_affect_navigation": False,
        }.items()
    ):
        raise PbrInputError("Blender manifest does not preserve MuJoCo-only physics authority")
    manifest_digest = _verify_manifest_digest(manifest)
    artifact_set_digest = _verify_artifact_set(manifest, resolved_manifest.parent)
    materials = _load_materials(manifest, resolved_manifest.parent, manifest_digest)
    if forest_authoring_plan is None:
        raise PbrInputError("--forest-authoring-plan is required for exact HISM provenance")
    instances = _load_forest_instance_provenance(forest_authoring_plan)

    heightmap_path: Path | None = None
    heightmap_sha256: str | None = None
    import_receipt_path: Path | None = None
    import_receipt_digest: str | None = None
    if terrain_profile == VALIDATION_PROFILE:
        if production_heightmap is not None or production_import_receipt is not None:
            raise PbrInputError("validation257_static must not accept production Landscape inputs")
        actor_label = VALIDATION_TERRAIN_LABEL
    elif terrain_profile == PRODUCTION_PROFILE:
        if production_heightmap is None or production_import_receipt is None:
            raise PbrInputError(
                "production4033_landscape requires --production-heightmap and --production-import-receipt"
            )
        heightmap_path = production_heightmap.expanduser().resolve()
        width, height, bit_depth, color_type = _png_info(heightmap_path)
        if (width, height, bit_depth, color_type) != (4033, 4033, 16, 0):
            raise PbrInputError("production Landscape heightmap must be a 4033x4033 16-bit grayscale PNG")
        heightmap_sha256 = _sha256_file(heightmap_path)
        import_receipt_path, import_receipt_digest = _load_landscape_import_receipt(
            production_import_receipt, heightmap_sha256
        )
        actor_label = PRODUCTION_TERRAIN_LABEL
    else:
        raise PbrInputError(f"unsupported Forest_HF terrain profile: {terrain_profile}")

    binding_set_sha256 = hashlib.sha256(
        canonical_json_bytes(
            {
                "manifest_digest": manifest_digest,
                "materials": [
                    {"material_id": item.material_id, "binding_sha256": item.binding_sha256} for item in materials
                ],
                "terrain_profile": terrain_profile,
                "heightmap_sha256": heightmap_sha256,
                "landscape_import_receipt_digest": import_receipt_digest,
                "forest_authoring_plan_sha256": instances.plan_sha256,
                "forest_points_sha256": instances.points_sha256,
            }
        )
    ).hexdigest()
    return PbrInputs(
        manifest_path=resolved_manifest,
        manifest=manifest,
        manifest_sha256=_sha256_file(resolved_manifest),
        manifest_digest=manifest_digest,
        artifact_set_digest=artifact_set_digest,
        materials=materials,
        terrain=TerrainInput(
            terrain_profile,
            actor_label,
            heightmap_path,
            heightmap_sha256,
            import_receipt_path,
            import_receipt_digest,
        ),
        instances=instances,
        binding_set_sha256=binding_set_sha256,
    )


def build_pbr_plan(inputs: PbrInputs) -> dict[str, object]:
    """Return the complete no-network mutation plan."""

    return {
        "schema": PLAN_SCHEMA,
        "mode": "dry_run",
        "apply_required": True,
        "world_package": WORLD_PACKAGE,
        "inputs": {
            "manifest": {
                "path": str(inputs.manifest_path),
                "file_sha256": inputs.manifest_sha256,
                "digest": inputs.manifest_digest,
                "artifact_set_digest": inputs.artifact_set_digest,
            },
            "production_heightmap": (
                {"path": str(inputs.terrain.heightmap_path), "sha256": inputs.terrain.heightmap_sha256}
                if inputs.terrain.heightmap_path is not None
                else None
            ),
            "production_import_receipt": (
                {
                    "path": str(inputs.terrain.import_receipt_path),
                    "digest": inputs.terrain.import_receipt_digest,
                }
                if inputs.terrain.import_receipt_path is not None
                else None
            ),
            "forest_authoring": {
                "plan_path": str(inputs.instances.plan_path),
                "plan_sha256": inputs.instances.plan_sha256,
                "points_path": str(inputs.instances.points_path),
                "points_sha256": inputs.instances.points_sha256,
                "point_count": inputs.instances.point_count,
                "expected_by_mesh": dict(inputs.instances.expected_by_mesh),
                "expected_partition_coordinates": sorted(EXPECTED_PARTITION_COORDINATES),
            },
            "binding_set_sha256": inputs.binding_set_sha256,
        },
        "target": {
            "map": MAP_PATH,
            "terrain_profile": inputs.terrain.profile,
            "terrain_actor_label": inputs.terrain.actor_label,
            "terrain_state": "preview_only" if inputs.terrain.profile == VALIDATION_PROFILE else "production_4033",
            "expected_actor_class": (
                STATIC_MESH_ACTOR_CLASS if inputs.terrain.profile == VALIDATION_PROFILE else LANDSCAPE_CLASS
            ),
            "materials_folder": MATERIAL_FOLDER,
            "textures_folder": TEXTURE_FOLDER,
        },
        "materials": [
            {
                "id": material.material_id,
                "asset": material.asset_path,
                "binding_sha256": material.binding_sha256,
                "textures": [
                    {
                        "channel": texture.channel,
                        "source": str(texture.source_path),
                        "source_sha256": texture.sha256,
                        "asset": texture.asset_path,
                        "sRGB": TEXTURE_SETTINGS[texture.channel]["sRGB"],
                        "compression": TEXTURE_SETTINGS[texture.channel]["compressionSettings"],
                    }
                    for texture in material.textures
                ],
                "channel_routing": EXPECTED_BINDINGS,
            }
            for material in inputs.materials
        ],
        "mesh_slot_bindings": {species: dict(record["slots"]) for species, record in TREE_MESHES.items()},
        "authority": {
            "physics": "mujoco",
            "raycast": "mujoco",
            "unreal": "VisualOnly",
            "collision_profile": "NoCollision",
            "simulate_physics": False,
            "overlap_events": False,
            "affects_navigation": False,
        },
        "phases": [
            "validate_manifest_artifact_and_texture_hashes_offline",
            "load_map_and_fail_closed_on_wrong_terrain_or_mesh_type",
            "prove_complete_64_cell_partition_and_exact_per_mesh_instance_counts",
            "import_textures_with_explicit_srgb_and_compression",
            "build_and_audit_basecolor_normal_orm_material_graphs",
            "bind_exact_tree_and_terrain_material_slots",
            "repair_and_audit_visual_only_nocollision_navigation_boundary",
            "save_reload_and_repeat_material_and_collision_audits",
        ],
    }


class _Session:
    def __init__(self, transport: McpTransport) -> None:
        self.transport = transport
        self.call_count = 0

    def call(self, toolset: str, tool: str, arguments: Mapping[str, object]) -> Any:
        self.call_count += 1
        try:
            return self.transport.call_tool(toolset, tool, arguments)
        except ForestPbrError:
            raise
        except Exception as error:
            raise PbrMcpError(f"{toolset}.{tool} failed closed: {error}") from error

    def exists(self, path: str) -> bool:
        result = self.call(ASSET_TOOLS, "exists", {"path": path})
        if not isinstance(result, bool):
            raise PbrMcpError(f"AssetTools.exists({path}) did not return a boolean")
        return result

    def require_true(self, toolset: str, tool: str, arguments: Mapping[str, object]) -> None:
        if self.call(toolset, tool, arguments) is not True:
            raise PbrMcpError(f"{toolset}.{tool} did not report success")

    def ensure_folder(self, path: str) -> None:
        if not self.exists(path):
            self.require_true(ASSET_TOOLS, "create_folder", {"path": path})
            if not self.exists(path):
                raise PbrMcpError(f"AssetTools.create_folder did not create {path}")


def _load_level(session: _Session, level_path: str) -> None:
    if session.call(SCENE_TOOLS, "load_level", {"level_path": level_path}) is False:
        raise PbrMcpError(f"SceneTools.load_level did not load {level_path}")
    current = _ref_path(session.call(SCENE_TOOLS, "get_current_level", {}))
    if current is None or level_path not in current:
        raise PbrMcpError(f"current editor level is not {level_path}: {current}")


def _find_exact_actor(session: _Session, label: str) -> str:
    actors = session.call(SCENE_TOOLS, "find_actors", {"name": label, "tag": "", "collision_channels": []})
    if isinstance(actors, (str, bytes)) or not isinstance(actors, Sequence):
        raise PbrMcpError(f"SceneTools.find_actors({label}) did not return an array")
    matches: list[str] = []
    for actor in actors:
        actor_path = _ref_path(actor)
        if actor_path is None:
            raise PbrMcpError(f"SceneTools.find_actors({label}) returned an invalid actor reference")
        if session.call(ACTOR_TOOLS, "get_label", {"actor": {"refPath": actor_path}}) == label:
            matches.append(actor_path)
    if len(matches) != 1:
        raise PbrMcpError(f"expected exactly one terrain actor labelled {label}, found {len(matches)}")
    return matches[0]


def _class_path(session: _Session, ref_path: str) -> str:
    observed = _ref_path(session.call(OBJECT_TOOLS, "get_class", {"instance": {"refPath": ref_path}}))
    if observed is None:
        raise PbrMcpError(f"ObjectTools.get_class did not identify {ref_path}")
    return observed


def _component_paths(session: _Session, actor_path: str, component_class: str) -> tuple[str, ...]:
    components = session.call(
        ACTOR_TOOLS,
        "get_components",
        {"actor": {"refPath": actor_path}, "component_type": {"refPath": component_class}},
    )
    if isinstance(components, (str, bytes)) or not isinstance(components, Sequence):
        raise PbrMcpError(f"component lookup failed for {actor_path} and {component_class}")
    paths = tuple(path for item in components if (path := _ref_path(item)) is not None)
    if len(paths) != len(components):
        raise PbrMcpError(f"component lookup returned an invalid ref for {actor_path}")
    return paths


def _production_heightmap_tag(sha256: str) -> str:
    return f"LingTuHeightmapSha256_{sha256}"


def _production_receipt_tag(digest: str) -> str:
    return f"LingTuLandscapeImportReceipt_{digest}"


def _verify_production_landscape(
    session: _Session,
    inputs: PbrInputs,
    actor_path: str,
    render_components: Sequence[str],
    collision_components: Sequence[str],
) -> None:
    """Require the canonical UE 4033 Landscape topology and importer provenance."""

    heightmap_sha256 = inputs.terrain.heightmap_sha256
    receipt_digest = inputs.terrain.import_receipt_digest
    if heightmap_sha256 is None or receipt_digest is None:
        raise PbrMcpError("production Landscape preflight has no bound heightmap/import receipt identity")
    actor = _mapping(
        session.call(
            OBJECT_TOOLS,
            "get_properties",
            {"instance": {"refPath": actor_path}, "properties": ["tags"]},
        ),
        "production Landscape provenance readback",
    )
    tags = actor.get("tags")
    required_tags = {
        "Production4033",
        _production_heightmap_tag(heightmap_sha256),
        _production_receipt_tag(receipt_digest),
    }
    if isinstance(tags, (str, bytes)) or not isinstance(tags, Sequence) or not required_tags.issubset(tags):
        raise PbrMcpError("production Landscape is missing its exact 4033 heightmap provenance tags")

    if len(render_components) != 1024 or len(collision_components) != 1024:
        raise PbrMcpError(
            "production 4033 Landscape must contain exactly 1024 render and 1024 heightfield collision components"
        )
    expected_bases = {(x * 126, y * 126) for x in range(32) for y in range(32)}
    render_by_base: dict[tuple[int, int], str] = {}
    heightmap_refs: set[str] = set()
    for component_path in render_components:
        observed = _mapping(
            session.call(
                OBJECT_TOOLS,
                "get_properties",
                {
                    "instance": {"refPath": component_path},
                    "properties": [
                        "sectionBaseX",
                        "sectionBaseY",
                        "componentSizeQuads",
                        "subsectionSizeQuads",
                        "numSubsections",
                        "heightmapTexture",
                    ],
                },
            ),
            f"production Landscape topology readback for {component_path}",
        )
        base_x = observed.get("sectionBaseX")
        base_y = observed.get("sectionBaseY")
        heightmap_ref = _ref_path(observed.get("heightmapTexture"))
        if (
            not isinstance(base_x, int)
            or isinstance(base_x, bool)
            or not isinstance(base_y, int)
            or isinstance(base_y, bool)
            or observed.get("componentSizeQuads") != 126
            or observed.get("subsectionSizeQuads") != 63
            or observed.get("numSubsections") != 2
            or heightmap_ref is None
            or (base_x, base_y) in render_by_base
        ):
            raise PbrMcpError("production Landscape does not match the canonical 4033 topology")
        render_by_base[(base_x, base_y)] = component_path
        heightmap_refs.add(heightmap_ref)
    if set(render_by_base) != expected_bases:
        raise PbrMcpError("production Landscape render components are not the exact 32x32 section grid")

    for heightmap_ref in heightmap_refs:
        heightmap_path = heightmap_ref.rsplit(".", 1)[0]
        if not session.exists(heightmap_path) or _class_path(session, heightmap_ref) != "/Script/Engine.Texture2D":
            raise PbrMcpError(f"production Landscape references an invalid heightmap texture: {heightmap_ref}")

    collision_by_base: dict[tuple[int, int], str] = {}
    for component_path in collision_components:
        observed = _mapping(
            session.call(
                OBJECT_TOOLS,
                "get_properties",
                {
                    "instance": {"refPath": component_path},
                    "properties": ["sectionBaseX", "sectionBaseY", "collisionSizeQuads", "renderComponentRef"],
                },
            ),
            f"production Landscape collision topology readback for {component_path}",
        )
        base_x = observed.get("sectionBaseX")
        base_y = observed.get("sectionBaseY")
        render_ref = _ref_path(observed.get("renderComponentRef"))
        if (
            not isinstance(base_x, int)
            or isinstance(base_x, bool)
            or not isinstance(base_y, int)
            or isinstance(base_y, bool)
            or observed.get("collisionSizeQuads") != 126
            or (base_x, base_y) in collision_by_base
            or render_ref != render_by_base.get((base_x, base_y))
        ):
            raise PbrMcpError("production Landscape collision/render components are not paired by section base")
        collision_by_base[(base_x, base_y)] = component_path
    if set(collision_by_base) != expected_bases:
        raise PbrMcpError("production Landscape collision components are not the exact 32x32 section grid")


def _preflight_terrain(session: _Session, inputs: PbrInputs) -> TerrainTarget:
    actor_path = _find_exact_actor(session, inputs.terrain.actor_label)
    actor_class = _class_path(session, actor_path)
    if inputs.terrain.profile == VALIDATION_PROFILE:
        if actor_class != STATIC_MESH_ACTOR_CLASS:
            raise PbrMcpError(f"validation terrain must be a StaticMeshActor, got {actor_class}")
        components = _component_paths(session, actor_path, STATIC_MESH_COMPONENT_CLASS)
        if len(components) != 1:
            raise PbrMcpError(
                f"validation terrain must expose exactly one StaticMeshComponent, found {len(components)}"
            )
        props = _mapping(
            session.call(
                OBJECT_TOOLS,
                "get_properties",
                {"instance": {"refPath": components[0]}, "properties": ["staticMesh"]},
            ),
            "validation terrain mesh readback",
        )
        if _ref_path(props.get("staticMesh")) != VALIDATION_TERRAIN_MESH_REF:
            raise PbrMcpError("validation terrain actor does not reference the exact Validation257 mesh")
        if _class_path(session, VALIDATION_TERRAIN_MESH_REF) != "/Script/Engine.StaticMesh":
            raise PbrMcpError("validation terrain asset is not a StaticMesh")
        slots = session.call(
            STATIC_MESH_TOOLS, "get_material_slots", {"mesh": {"refPath": VALIDATION_TERRAIN_MESH_REF}}
        )
        if slots != ["defaultMat"]:
            raise PbrMcpError(f"validation terrain material slots must be exactly ['defaultMat'], got {slots}")
        return TerrainTarget(
            profile=inputs.terrain.profile,
            actor_path=actor_path,
            actor_class=actor_class,
            render_components=components,
            collision_components=components,
            mesh_path=VALIDATION_TERRAIN_MESH,
            mesh_ref=VALIDATION_TERRAIN_MESH_REF,
            material_slot="defaultMat",
            source_heightmap_sha256=None,
            source_import_receipt_digest=None,
        )

    if actor_class != LANDSCAPE_CLASS:
        raise PbrMcpError(f"production terrain must be an actual 4033 Landscape, got {actor_class}")
    render_components = _component_paths(session, actor_path, LANDSCAPE_COMPONENT_CLASS)
    collision_components = _component_paths(session, actor_path, LANDSCAPE_COLLISION_COMPONENT_CLASS)
    if not render_components or not collision_components:
        raise PbrMcpError("production Landscape must expose render and heightfield collision components")
    _verify_production_landscape(session, inputs, actor_path, render_components, collision_components)
    return TerrainTarget(
        profile=inputs.terrain.profile,
        actor_path=actor_path,
        actor_class=actor_class,
        render_components=render_components,
        collision_components=collision_components,
        mesh_path=None,
        mesh_ref=None,
        material_slot=None,
        source_heightmap_sha256=inputs.terrain.heightmap_sha256,
        source_import_receipt_digest=inputs.terrain.import_receipt_digest,
    )


def _preflight_tree_meshes(session: _Session) -> None:
    for species, record in TREE_MESHES.items():
        asset_path = str(record["asset_path"])
        asset_ref = str(record["asset_ref"])
        if not session.exists(asset_path) or _class_path(session, asset_ref) != "/Script/Engine.StaticMesh":
            raise PbrMcpError(f"Forest_HF {species} target is not an exact StaticMesh asset")
        slots = session.call(STATIC_MESH_TOOLS, "get_material_slots", {"mesh": {"refPath": asset_ref}})
        expected = list(record["slots"])
        if slots != expected:
            raise PbrMcpError(f"Forest_HF {species} material slots must be exactly {expected}, got {slots}")


def _texture_metadata(inputs: PbrInputs, material: MaterialSource, texture: TextureSource) -> dict[str, str]:
    return {
        SOURCE_SHA_TAG: texture.sha256,
        MANIFEST_DIGEST_TAG: inputs.manifest_digest,
        MATERIAL_ID_TAG: material.material_id,
        TEXTURE_CHANNEL_TAG: texture.channel,
        AUTHORITY_TAG: "VisualOnly;NoCollision;MuJoCoPhysicsAndRaycast",
    }


def _ensure_texture(
    session: _Session,
    inputs: PbrInputs,
    material: MaterialSource,
    texture: TextureSource,
) -> str:
    existed = session.exists(texture.asset_path)
    expected_metadata = _texture_metadata(inputs, material, texture)
    metadata: Mapping[str, object] = {}
    settings: Mapping[str, object] = {}
    if existed:
        if session.call(ASSET_TOOLS, "get_asset_class", {"asset_path": texture.asset_path}) != "Texture2D":
            raise PbrMcpError(f"PBR texture target is not a Texture2D: {texture.asset_path}")
        metadata = session.call(ASSET_TOOLS, "get_metadata_tags", {"asset_path": texture.asset_path})
        settings = _mapping(
            session.call(
                OBJECT_TOOLS,
                "get_properties",
                {"instance": {"refPath": texture.asset_ref}, "properties": ["sRGB", "compressionSettings"]},
            ),
            f"texture settings readback for {texture.asset_path}",
        )
        if not isinstance(metadata, Mapping) or metadata.get(SOURCE_SHA_TAG) != texture.sha256:
            raise PbrMcpError(
                "existing texture source identity differs or is unknown; explicit asset replacement is required: "
                f"{texture.asset_path}"
            )
    matches = (
        existed
        and isinstance(metadata, Mapping)
        and all(metadata.get(key) == value for key, value in expected_metadata.items())
        and all(
            settings.get(key) == value
            for key, value in TEXTURE_SETTINGS[texture.channel].items()
            if key != "samplerType"
        )
    )
    if not matches:
        if not existed:
            imported = session.call(
                TEXTURE_TOOLS,
                "import_file",
                {
                    "folder_path": TEXTURE_FOLDER,
                    "asset_name": texture.asset_name,
                    "source_file": str(texture.source_path),
                },
            )
            if isinstance(imported, (str, bytes)) or not isinstance(imported, Sequence):
                raise PbrMcpError(f"TextureTools.import_file returned no asset array for {texture.source_path}")
            if texture.asset_ref not in {_ref_path(item) for item in imported} or not session.exists(
                texture.asset_path
            ):
                raise PbrMcpError(f"TextureTools.import_file did not create exact texture {texture.asset_ref}")
        session.require_true(
            OBJECT_TOOLS,
            "set_properties",
            {
                "instance": {"refPath": texture.asset_ref},
                "values": _compact_json(
                    {key: value for key, value in TEXTURE_SETTINGS[texture.channel].items() if key != "samplerType"}
                ),
            },
        )
        session.call(
            ASSET_TOOLS,
            "update_metadata_tags",
            {"asset_path": texture.asset_path, "set_tags": expected_metadata},
        )
    _audit_texture(session, inputs, material, texture)
    return "reused" if matches else ("repaired" if existed else "created")


def _audit_texture(
    session: _Session,
    inputs: PbrInputs,
    material: MaterialSource,
    texture: TextureSource,
) -> None:
    if not session.exists(texture.asset_path):
        raise PbrMcpError(f"PBR texture is missing: {texture.asset_path}")
    if session.call(ASSET_TOOLS, "get_asset_class", {"asset_path": texture.asset_path}) != "Texture2D":
        raise PbrMcpError(f"PBR texture target is not a Texture2D: {texture.asset_path}")
    expected_metadata = _texture_metadata(inputs, material, texture)
    observed_settings = _mapping(
        session.call(
            OBJECT_TOOLS,
            "get_properties",
            {"instance": {"refPath": texture.asset_ref}, "properties": ["sRGB", "compressionSettings"]},
        ),
        f"texture settings verification for {texture.asset_path}",
    )
    observed_metadata = session.call(ASSET_TOOLS, "get_metadata_tags", {"asset_path": texture.asset_path})
    if (
        any(
            observed_settings.get(key) != value
            for key, value in TEXTURE_SETTINGS[texture.channel].items()
            if key != "samplerType"
        )
        or not isinstance(observed_metadata, Mapping)
        or any(observed_metadata.get(key) != value for key, value in expected_metadata.items())
    ):
        raise PbrMcpError(f"texture import settings or provenance did not persist: {texture.asset_path}")


def _material_metadata(inputs: PbrInputs, material: MaterialSource) -> dict[str, str]:
    return {
        MANIFEST_DIGEST_TAG: inputs.manifest_digest,
        MATERIAL_ID_TAG: material.material_id,
        MATERIAL_BINDING_TAG: material.binding_sha256,
        AUTHORITY_TAG: "VisualOnly;NoCollision;MuJoCoPhysicsAndRaycast",
    }


def _audit_material_graph(session: _Session, material: MaterialSource) -> bool:
    expressions = session.call(
        MATERIAL_TOOLS,
        "get_expressions",
        {"material_or_function": {"refPath": material.asset_ref}},
    )
    if isinstance(expressions, (str, bytes)) or not isinstance(expressions, Sequence) or len(expressions) != 3:
        return False
    expression_settings: dict[str, tuple[str, str]] = {}
    for expression in expressions:
        expression_path = _ref_path(expression)
        if expression_path is None or _class_path(session, expression_path) != TEXTURE_SAMPLE_CLASS:
            return False
        props = _mapping(
            session.call(
                OBJECT_TOOLS,
                "get_properties",
                {"instance": {"refPath": expression_path}, "properties": ["texture", "samplerType"]},
            ),
            f"material expression readback for {expression_path}",
        )
        texture_ref = _ref_path(props.get("texture"))
        sampler_type = props.get("samplerType")
        if texture_ref is None or not isinstance(sampler_type, str):
            return False
        expression_settings[expression_path] = (texture_ref, sampler_type)
    by_channel = {texture.channel: texture for texture in material.textures}
    for output_id, (output_name, material_property) in MATERIAL_OUTPUTS.items():
        source_channel = EXPECTED_BINDINGS[output_id]["texture"]
        source = session.call(
            MATERIAL_TOOLS,
            "get_property_input",
            {"material": {"refPath": material.asset_ref}, "material_property": material_property},
        )
        if not isinstance(source, Mapping):
            return False
        expression_path = _ref_path(source.get("expression"))
        if expression_path is None or source.get("output_name") != output_name:
            return False
        expected_texture = by_channel[str(source_channel)]
        expected_sampler = str(TEXTURE_SETTINGS[str(source_channel)]["samplerType"])
        if expression_settings.get(expression_path) != (expected_texture.asset_ref, expected_sampler):
            return False
    return True


def _rebuild_material_graph(session: _Session, material: MaterialSource) -> None:
    existing = session.call(
        MATERIAL_TOOLS,
        "get_expressions",
        {"material_or_function": {"refPath": material.asset_ref}},
    )
    if isinstance(existing, (str, bytes)) or not isinstance(existing, Sequence):
        raise PbrMcpError(f"MaterialTools.get_expressions failed for {material.asset_path}")
    for expression in existing:
        expression_path = _ref_path(expression)
        if expression_path is None:
            raise PbrMcpError(f"material {material.asset_path} contains an invalid expression ref")
        session.call(
            MATERIAL_TOOLS,
            "delete_expression",
            {
                "material_or_function": {"refPath": material.asset_ref},
                "expression": {"refPath": expression_path},
            },
        )

    expression_by_channel: dict[str, str] = {}
    positions = {"base_color": (-600, -200), "normal": (-600, 0), "orm": (-600, 240)}
    for texture in material.textures:
        x, y = positions[texture.channel]
        added = session.call(
            MATERIAL_TOOLS,
            "add_expression",
            {
                "material_or_function": {"refPath": material.asset_ref},
                "expression_class": {"refPath": TEXTURE_SAMPLE_CLASS},
                "x": x,
                "y": y,
            },
        )
        expression_path = _ref_path(added)
        if expression_path is None:
            raise PbrMcpError(f"MaterialTools.add_expression did not create {texture.channel} texture sample")
        session.require_true(
            OBJECT_TOOLS,
            "set_properties",
            {
                "instance": {"refPath": expression_path},
                "values": _compact_json(
                    {
                        "texture": {"refPath": texture.asset_ref},
                        "samplerType": TEXTURE_SETTINGS[texture.channel]["samplerType"],
                    }
                ),
            },
        )
        expression_by_channel[texture.channel] = expression_path
    for output_id, (output_name, material_property) in MATERIAL_OUTPUTS.items():
        source_channel = str(EXPECTED_BINDINGS[output_id]["texture"])
        session.call(
            MATERIAL_TOOLS,
            "connect_to_output",
            {
                "expression": {"refPath": expression_by_channel[source_channel]},
                "output_name": output_name,
                "material_property": material_property,
            },
        )
    session.call(MATERIAL_TOOLS, "layout_expressions", {"material_or_function": {"refPath": material.asset_ref}})
    session.call(MATERIAL_TOOLS, "recompile", {"material_or_function": {"refPath": material.asset_ref}})


def _ensure_material(session: _Session, inputs: PbrInputs, material: MaterialSource) -> str:
    existed = session.exists(material.asset_path)
    expected_metadata = _material_metadata(inputs, material)
    metadata: Mapping[str, object] = {}
    if existed:
        if session.call(ASSET_TOOLS, "get_asset_class", {"asset_path": material.asset_path}) != "Material":
            raise PbrMcpError(f"PBR material target is not a Material: {material.asset_path}")
        metadata = session.call(ASSET_TOOLS, "get_metadata_tags", {"asset_path": material.asset_path})
    metadata_matches = (
        existed
        and isinstance(metadata, Mapping)
        and all(metadata.get(key) == value for key, value in expected_metadata.items())
    )
    graph_matches = metadata_matches and _audit_material_graph(session, material)
    if not graph_matches:
        if not existed:
            created = session.call(
                MATERIAL_TOOLS,
                "create_material",
                {"folder_path": MATERIAL_FOLDER, "asset_name": material.asset_name},
            )
            if _ref_path(created) != material.asset_ref or not session.exists(material.asset_path):
                raise PbrMcpError(f"MaterialTools.create_material did not create exact asset {material.asset_ref}")
        _rebuild_material_graph(session, material)
        session.call(
            ASSET_TOOLS,
            "update_metadata_tags",
            {"asset_path": material.asset_path, "set_tags": expected_metadata},
        )
    _audit_material(session, inputs, material)
    return "reused" if graph_matches else ("updated" if existed else "created")


def _audit_material(session: _Session, inputs: PbrInputs, material: MaterialSource) -> None:
    if not session.exists(material.asset_path):
        raise PbrMcpError(f"PBR material is missing: {material.asset_path}")
    if session.call(ASSET_TOOLS, "get_asset_class", {"asset_path": material.asset_path}) != "Material":
        raise PbrMcpError(f"PBR material target is not a Material: {material.asset_path}")
    expected_metadata = _material_metadata(inputs, material)
    observed_metadata = session.call(ASSET_TOOLS, "get_metadata_tags", {"asset_path": material.asset_path})
    if (
        not isinstance(observed_metadata, Mapping)
        or any(observed_metadata.get(key) != value for key, value in expected_metadata.items())
        or not _audit_material_graph(session, material)
    ):
        raise PbrMcpError(f"material graph or provenance did not persist: {material.asset_path}")


def _bind_tree_meshes(session: _Session, materials: Mapping[str, MaterialSource]) -> None:
    for species, record in TREE_MESHES.items():
        mesh_ref = str(record["asset_ref"])
        session.require_true(STATIC_MESH_TOOLS, "remove_collisions", {"mesh": {"refPath": mesh_ref}})
        for slot_name, material_id in record["slots"].items():
            material = materials[str(material_id)]
            session.require_true(
                STATIC_MESH_TOOLS,
                "set_material",
                {
                    "mesh": {"refPath": mesh_ref},
                    "slot_name": slot_name,
                    "material": {"refPath": material.asset_ref},
                },
            )
            observed = _ref_path(
                session.call(
                    STATIC_MESH_TOOLS,
                    "get_material",
                    {"mesh": {"refPath": mesh_ref}, "slot_name": slot_name},
                )
            )
            if observed != material.asset_ref:
                raise PbrMcpError(f"{species} mesh slot {slot_name} did not retain {material.asset_ref}")


def _authority_body() -> dict[str, object]:
    return {
        "collisionProfileName": "NoCollision",
        "bSimulatePhysics": False,
        "bEnableGravity": False,
        "bNotifyRigidBodyCollision": False,
    }


def _set_component_authority(session: _Session, component_path: str, tags: Sequence[str]) -> None:
    session.require_true(
        OBJECT_TOOLS,
        "set_properties",
        {
            "instance": {"refPath": component_path},
            "values": _compact_json(
                {
                    "bodyInstance": _authority_body(),
                    "bGenerateOverlapEvents": False,
                    "bCanEverAffectNavigation": False,
                    "componentTags": list(tags),
                }
            ),
        },
    )


def _audit_component_authority(session: _Session, component_path: str) -> None:
    observed = _mapping(
        session.call(
            OBJECT_TOOLS,
            "get_properties",
            {
                "instance": {"refPath": component_path},
                "properties": ["bodyInstance", "bGenerateOverlapEvents", "bCanEverAffectNavigation", "componentTags"],
            },
        ),
        f"NoCollision readback for {component_path}",
    )
    body = observed.get("bodyInstance")
    tags = observed.get("componentTags")
    if not isinstance(body, Mapping) or any(body.get(key) != value for key, value in _authority_body().items()):
        raise PbrMcpError(f"component did not retain the MuJoCo-only body policy: {component_path}")
    if observed.get("bGenerateOverlapEvents") is not False or observed.get("bCanEverAffectNavigation") is not False:
        raise PbrMcpError(f"component retained overlap or navigation authority: {component_path}")
    if (
        isinstance(tags, (str, bytes))
        or not isinstance(tags, Sequence)
        or not {"VisualOnly", "NoCollision"}.issubset(tags)
    ):
        raise PbrMcpError(f"component is missing VisualOnly/NoCollision tags: {component_path}")


def _audit_actor_query_collision_disabled(session: _Session, actor_path: str) -> None:
    """Prove the actor is absent from UE's native physics overlap query.

    ``FBodyInstance.collisionEnabled`` is serialized as the raw body enum and can
    remain ``QueryAndPhysics`` after an editor property write changes the profile
    to ``NoCollision``.  It is therefore not accepted as effective-collision
    evidence.  The native overlap query is the fail-closed runtime observation.
    """

    bounds = session.call(ACTOR_TOOLS, "get_actor_bounds", {"actor": {"refPath": actor_path}})
    if not isinstance(bounds, Mapping) or bounds.get("isValid") is not True:
        raise PbrMcpError(f"actor bounds are unavailable for effective collision audit: {actor_path}")
    channels = session.call(SCENE_TOOLS, "get_collision_channels", {})
    if isinstance(channels, (str, bytes)) or not isinstance(channels, Sequence) or not channels:
        raise PbrMcpError("SceneTools.get_collision_channels returned no collision channels")
    colliders = session.call(
        SCENE_TOOLS,
        "find_actors",
        {
            "root": {"refPath": actor_path},
            "name": "",
            "tag": "",
            "bounds": bounds,
            "collision_channels": list(channels),
        },
    )
    if isinstance(colliders, (str, bytes)) or not isinstance(colliders, Sequence):
        raise PbrMcpError(f"native overlap query failed for {actor_path}")
    if actor_path in {_ref_path(item) for item in colliders}:
        raise PbrMcpError(f"actor remains query-collidable after NoCollision binding: {actor_path}")


def _set_actor_authority(session: _Session, target: TerrainTarget) -> None:
    terrain_state_tag = "Validation257" if target.profile == VALIDATION_PROFILE else "Production4033"
    expected_tags = ["LingTuForestTerrain", "VisualOnly", "NoCollision", terrain_state_tag]
    if target.source_heightmap_sha256 is not None:
        expected_tags.append(_production_heightmap_tag(target.source_heightmap_sha256))
    if target.source_import_receipt_digest is not None:
        expected_tags.append(_production_receipt_tag(target.source_import_receipt_digest))
    for tag in expected_tags:
        if session.call(ACTOR_TOOLS, "has_tag", {"actor": {"refPath": target.actor_path}, "tag": tag}) is not True:
            added = session.call(
                ACTOR_TOOLS,
                "add_tag",
                {"actor": {"refPath": target.actor_path}, "tag": tag},
            )
            if added is False:
                raise PbrMcpError(f"ActorTools.add_tag did not add {tag} to {target.actor_path}")
    session.require_true(
        OBJECT_TOOLS,
        "set_properties",
        {
            "instance": {"refPath": target.actor_path},
            "values": _compact_json(
                {
                    "bGenerateOverlapEventsDuringLevelStreaming": False,
                }
            ),
        },
    )
    observed = _mapping(
        session.call(
            OBJECT_TOOLS,
            "get_properties",
            {
                "instance": {"refPath": target.actor_path},
                "properties": ["bGenerateOverlapEventsDuringLevelStreaming"],
            },
        ),
        "terrain actor authority readback",
    )
    observed_tags = session.call(ACTOR_TOOLS, "get_tags", {"actor": {"refPath": target.actor_path}})
    if (
        isinstance(observed_tags, (str, bytes))
        or not isinstance(observed_tags, Sequence)
        or not set(expected_tags).issubset(observed_tags)
        or observed.get("bGenerateOverlapEventsDuringLevelStreaming") is not False
    ):
        raise PbrMcpError("terrain actor did not retain the VisualOnly/NoCollision authority policy")


def _bind_terrain(session: _Session, target: TerrainTarget, ground: MaterialSource) -> None:
    _set_actor_authority(session, target)
    tags = ["LingTuForestTerrain", "VisualOnly", "NoCollision"]
    if target.profile == VALIDATION_PROFILE:
        if target.mesh_ref is None or target.material_slot is None:
            raise PbrMcpError("validation terrain preflight omitted its mesh material target")
        session.require_true(STATIC_MESH_TOOLS, "remove_collisions", {"mesh": {"refPath": target.mesh_ref}})
        session.require_true(
            STATIC_MESH_TOOLS,
            "set_material",
            {
                "mesh": {"refPath": target.mesh_ref},
                "slot_name": target.material_slot,
                "material": {"refPath": ground.asset_ref},
            },
        )
        observed = _ref_path(
            session.call(
                STATIC_MESH_TOOLS,
                "get_material",
                {"mesh": {"refPath": target.mesh_ref}, "slot_name": target.material_slot},
            )
        )
        if observed != ground.asset_ref:
            raise PbrMcpError("validation terrain did not retain the ground PBR material")
    else:
        session.require_true(
            OBJECT_TOOLS,
            "set_properties",
            {
                "instance": {"refPath": target.actor_path},
                "values": _compact_json({"landscapeMaterial": {"refPath": ground.asset_ref}}),
            },
        )
        observed_landscape = _mapping(
            session.call(
                OBJECT_TOOLS,
                "get_properties",
                {"instance": {"refPath": target.actor_path}, "properties": ["landscapeMaterial"]},
            ),
            "production Landscape material readback",
        )
        if _ref_path(observed_landscape.get("landscapeMaterial")) != ground.asset_ref:
            raise PbrMcpError("production Landscape did not retain the ground PBR material")
    for component_path in dict.fromkeys((*target.render_components, *target.collision_components)):
        _set_component_authority(session, component_path, tags)
        _audit_component_authority(session, component_path)
    _audit_actor_query_collision_disabled(session, target.actor_path)


def _partition_coordinates(actor_path: str) -> tuple[int, int] | None:
    marker = "PCGPartitionGridActor_25600_"
    if marker not in actor_path:
        return None
    pieces = actor_path.rsplit(marker, 1)[1].split("_")
    if len(pieces) != 2:
        return None
    try:
        return int(pieces[0]), int(pieces[1])
    except ValueError:
        return None


def _audit_hism_authority(session: _Session, inputs: PbrInputs) -> dict[str, object]:
    actors = session.call(SCENE_TOOLS, "find_actors", {"name": "", "tag": "", "collision_channels": []})
    if isinstance(actors, (str, bytes)) or not isinstance(actors, Sequence):
        raise PbrMcpError("SceneTools.find_actors did not return an array for the HISM audit")
    actor_by_coordinates: dict[tuple[int, int], str] = {}
    for actor in actors:
        path = _ref_path(actor)
        if path is None or "PCGPartitionGridActor_25600_" not in path:
            continue
        coordinates = _partition_coordinates(path)
        if coordinates is None or MAP_PATH not in path or coordinates in actor_by_coordinates:
            raise PbrMcpError(f"Forest_HF contains an invalid or duplicate PCG partition actor: {path}")
        actor_by_coordinates[coordinates] = path
    if set(actor_by_coordinates) != EXPECTED_PARTITION_COORDINATES:
        raise PbrMcpError("Forest_HF must have all exact 64 loaded 256 m partition cells before PBR binding")
    actor_paths = [actor_by_coordinates[key] for key in sorted(actor_by_coordinates)]
    allowed_meshes = {str(record["asset_ref"]) for record in TREE_MESHES.values()}
    expected_by_mesh = dict(inputs.instances.expected_by_mesh)
    by_mesh = {mesh: 0 for mesh in sorted(allowed_meshes)}
    component_count = 0
    instance_count = 0
    for actor_path in actor_paths:
        for component_path in _component_paths(session, actor_path, HISM_COMPONENT_CLASS):
            props = _mapping(
                session.call(
                    OBJECT_TOOLS,
                    "get_properties",
                    {
                        "instance": {"refPath": component_path},
                        "properties": [
                            "staticMesh",
                            "perInstanceSMData",
                            "bodyInstance",
                            "bGenerateOverlapEvents",
                            "bCanEverAffectNavigation",
                            "componentTags",
                        ],
                    },
                ),
                f"HISM authority readback for {component_path}",
            )
            mesh_ref = _ref_path(props.get("staticMesh"))
            if mesh_ref not in allowed_meshes:
                raise PbrMcpError(f"Forest_HF HISM uses an unapproved mesh: {component_path}")
            _audit_component_authority_from_properties(props, component_path)
            instances = props.get("perInstanceSMData")
            if isinstance(instances, (str, bytes)) or not isinstance(instances, Sequence):
                raise PbrMcpError(f"Forest_HF HISM instance data are unavailable: {component_path}")
            count = len(instances)
            by_mesh[mesh_ref] += count
            component_count += 1
            instance_count += count
        _audit_actor_query_collision_disabled(session, actor_path)
    if component_count == 0 or instance_count != inputs.instances.point_count or by_mesh != expected_by_mesh:
        raise PbrMcpError(
            "Forest_HF HISM totals do not match the hash-bound authoring contract: "
            f"observed_total={instance_count}, observed_by_mesh={by_mesh}, "
            f"expected_total={inputs.instances.point_count}, expected_by_mesh={expected_by_mesh}"
        )
    return {
        "partition_actor_count": len(actor_paths),
        "component_count": component_count,
        "instance_count": instance_count,
        "by_mesh": by_mesh,
        "expected_by_mesh": expected_by_mesh,
    }


def _audit_component_authority_from_properties(props: Mapping[str, object], component_path: str) -> None:
    body = props.get("bodyInstance")
    tags = props.get("componentTags")
    if not isinstance(body, Mapping) or any(body.get(key) != value for key, value in _authority_body().items()):
        raise PbrMcpError(f"generated HISM did not retain NoCollision physics settings: {component_path}")
    if props.get("bGenerateOverlapEvents") is not False or props.get("bCanEverAffectNavigation") is not False:
        raise PbrMcpError(f"generated HISM retained overlap or navigation authority: {component_path}")
    if (
        isinstance(tags, (str, bytes))
        or not isinstance(tags, Sequence)
        or not {"VisualOnly", "NoCollision"}.issubset(tags)
    ):
        raise PbrMcpError(f"generated HISM is missing VisualOnly/NoCollision tags: {component_path}")


def _audit_bound_assets(
    session: _Session,
    inputs: PbrInputs,
    materials: Mapping[str, MaterialSource],
) -> tuple[TerrainTarget, dict[str, object]]:
    target = _preflight_terrain(session, inputs)
    _preflight_tree_meshes(session)
    for material in inputs.materials:
        for texture in material.textures:
            _audit_texture(session, inputs, material, texture)
        _audit_material(session, inputs, material)
    for species, record in TREE_MESHES.items():
        mesh_ref = str(record["asset_ref"])
        for slot_name, material_id in record["slots"].items():
            observed = _ref_path(
                session.call(
                    STATIC_MESH_TOOLS,
                    "get_material",
                    {"mesh": {"refPath": mesh_ref}, "slot_name": slot_name},
                )
            )
            if observed != materials[str(material_id)].asset_ref:
                raise PbrMcpError(f"{species} mesh material binding changed after save/reload")
    ground = materials["forest_ground"]
    if target.profile == VALIDATION_PROFILE:
        if target.mesh_ref is None or target.material_slot is None:
            raise PbrMcpError("validation terrain persistence audit omitted its mesh target")
        observed_ground = _ref_path(
            session.call(
                STATIC_MESH_TOOLS,
                "get_material",
                {"mesh": {"refPath": target.mesh_ref}, "slot_name": target.material_slot},
            )
        )
    else:
        props = _mapping(
            session.call(
                OBJECT_TOOLS,
                "get_properties",
                {"instance": {"refPath": target.actor_path}, "properties": ["landscapeMaterial"]},
            ),
            "Landscape material persistence audit",
        )
        observed_ground = _ref_path(props.get("landscapeMaterial"))
    if observed_ground != ground.asset_ref:
        raise PbrMcpError("terrain ground material changed after save/reload")
    for component_path in dict.fromkeys((*target.render_components, *target.collision_components)):
        _audit_component_authority(session, component_path)
    _audit_actor_query_collision_disabled(session, target.actor_path)
    return target, _audit_hism_authority(session, inputs)


def author_pbr(transport: McpTransport, inputs: PbrInputs) -> dict[str, object]:
    """Apply and prove the exact Forest_HF PBR binding through MCP."""

    transport.initialize()
    transport.notify_initialized()
    session = _Session(transport)
    _load_level(session, MAP_PATH)
    target = _preflight_terrain(session, inputs)
    _preflight_tree_meshes(session)

    session.ensure_folder(MATERIAL_FOLDER)
    session.ensure_folder(TEXTURE_FOLDER)
    texture_outcomes: dict[str, str] = {}
    for material in inputs.materials:
        for texture in material.textures:
            texture_outcomes[f"{material.material_id}:{texture.channel}"] = _ensure_texture(
                session, inputs, material, texture
            )
    material_outcomes = {
        material.material_id: _ensure_material(session, inputs, material) for material in inputs.materials
    }
    by_id = {material.material_id: material for material in inputs.materials}
    _bind_tree_meshes(session, by_id)
    _bind_terrain(session, target, by_id["forest_ground"])
    hism_audit = _audit_hism_authority(session, inputs)

    save_actor_result = session.call(SCENE_TOOLS, "save_actor", {"actor": {"refPath": target.actor_path}})
    if save_actor_result is False:
        raise PbrMcpError("SceneTools.save_actor did not save the World Partition terrain actor")

    save_paths = sorted(
        {
            MAP_PATH,
            *(str(record["asset_path"]) for record in TREE_MESHES.values()),
            *(item.asset_path for item in inputs.materials),
            *(texture.asset_path for item in inputs.materials for texture in item.textures),
            *([target.mesh_path] if target.mesh_path is not None else []),
        }
    )
    session.require_true(ASSET_TOOLS, "save_assets", {"asset_paths": save_paths})
    dirty = [path for path in save_paths if session.call(ASSET_TOOLS, "is_dirty", {"asset_path": path}) is not False]
    if dirty:
        raise PbrMcpError(f"PBR assets remain dirty after save: {dirty}")
    if not session.exists(OPEN_WORLD_TEMPLATE):
        raise PbrMcpError("OpenWorld template is unavailable for forced disk-reload verification")
    _load_level(session, OPEN_WORLD_TEMPLATE)
    _load_level(session, MAP_PATH)
    reloaded_target, reload_hism_audit = _audit_bound_assets(session, inputs, by_id)
    if reloaded_target.profile != target.profile or reload_hism_audit != hism_audit:
        raise PbrMcpError("terrain or HISM authority audit changed after map save/reload")
    return {
        "schema": RESULT_SCHEMA,
        "world_package": WORLD_PACKAGE,
        "binding_set_sha256": inputs.binding_set_sha256,
        "manifest": {
            "path": str(inputs.manifest_path),
            "file_sha256": inputs.manifest_sha256,
            "digest": inputs.manifest_digest,
        },
        "terrain": {
            "profile": target.profile,
            "state": "preview_only" if target.profile == VALIDATION_PROFILE else "production_4033",
            "actor_ref": target.actor_path,
            "actor_class": target.actor_class,
            "heightmap_sha256": inputs.terrain.heightmap_sha256,
            "import_receipt_digest": inputs.terrain.import_receipt_digest,
            "collision_profile": "NoCollision",
            "affects_navigation": False,
        },
        "textures": texture_outcomes,
        "materials": material_outcomes,
        "hism_audit": hism_audit,
        "authority": {"physics": "mujoco", "raycast": "mujoco", "unreal_collision": "NoCollision"},
        "mcp_tool_calls": session.call_count,
    }


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--terrain-profile",
        choices=(VALIDATION_PROFILE, PRODUCTION_PROFILE),
        default=VALIDATION_PROFILE,
    )
    parser.add_argument(
        "--forest-authoring-plan",
        type=Path,
        required=True,
        help="hash-bound dry-run plan emitted by build_forest_hf_mcp.py",
    )
    parser.add_argument("--production-heightmap", type=Path)
    parser.add_argument("--production-import-receipt", type=Path)
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    parser.add_argument("--timeout-seconds", type=float, default=60.0)
    parser.add_argument("--apply", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Run an offline dry-run plan or the explicit MCP apply workflow."""

    args = _parse_args(argv)
    inputs = load_pbr_inputs(
        args.manifest,
        args.terrain_profile,
        args.production_heightmap,
        args.forest_authoring_plan,
        args.production_import_receipt,
    )
    if not args.apply:
        print(json.dumps(build_pbr_plan(inputs), ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False))
        return 0
    if args.timeout_seconds <= 0:
        raise PbrInputError("--timeout-seconds must be positive")
    try:
        from build_forest_hf_mcp import HttpMcpTransport
    except ImportError as error:
        raise PbrInputError("build_forest_hf_mcp.py must remain beside the PBR binder") from error
    transport = HttpMcpTransport(endpoint=args.endpoint, timeout_seconds=args.timeout_seconds)
    print(json.dumps(author_pbr(transport, inputs), ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
