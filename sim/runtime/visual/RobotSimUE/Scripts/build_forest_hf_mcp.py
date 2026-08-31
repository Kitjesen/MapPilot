"""Author the deterministic Forest_HF UE 5.8 map through the reviewed MCP surface.

Dry-run is the default.  ``--apply`` is required before this module opens an
MCP session or mutates the editor.  MuJoCo remains the sole physics, contact,
and raycast authority; generated Unreal content is visual-only and explicitly
uses ``NoCollision``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any, Mapping, NamedTuple, Protocol, Sequence

REPO_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_CONTRACT_PATH = REPO_ROOT / "build" / "forest-hf-2km" / "unreal" / "forest-hf.bake.json"
DEFAULT_POINTS_PATH = REPO_ROOT / "build" / "forest-hf-2km" / "unreal" / "forest-pcg-create-points-dense.json"
DEFAULT_ENDPOINT = "http://127.0.0.1:8000/mcp"

WORLD_PACKAGE = "forest_hf@2.0.0"
CONTRACT_SCHEMA = "lingtu.sim.unreal-forest-offline-bake.v2"
CREATE_POINTS_SCHEMA = "lingtu.sim.unreal-forest-create-points.v2"
PLAN_SCHEMA = "lingtu.sim.unreal-forest-mcp-authoring-plan.v1"
EXACT_MESH_SLOT_SELECTION_POLICY = "exact_mesh_slot_branches"
FIXED_SEED = 20260813
MAP_PATH = "/Game/RobotSim/Maps/Forest_HF_2km"
MAP_FOLDER = MAP_PATH.rsplit("/", 1)[0]
OPEN_WORLD_TEMPLATE = "/Engine/Maps/Templates/OpenWorld"
GRAPH_FOLDER = "/Game/RobotSim/PCG"
GRAPH_NAME = "PCG_ForestScatter_2km"
GRAPH_ASSET_PATH = f"{GRAPH_FOLDER}/{GRAPH_NAME}"
GRAPH_OBJECT_PATH = f"{GRAPH_ASSET_PATH}.{GRAPH_NAME}"
ASSET_FOLDER = "/Game/RobotSim/Worlds/ForestHF2km/Assets"
VOLUME_NAME = GRAPH_NAME

ASSET_TOOLS = "editor_toolset.toolsets.asset.AssetTools"
SCENE_TOOLS = "editor_toolset.toolsets.scene.SceneTools"
ACTOR_TOOLS = "editor_toolset.toolsets.actor.ActorTools"
OBJECT_TOOLS = "editor_toolset.toolsets.object.ObjectTools"
STATIC_MESH_TOOLS = "editor_toolset.toolsets.static_mesh.StaticMeshTools"
PCG_TOOLS = "PCGToolset.PCGToolset"

LEGACY_CREATE_POINTS_NODE_NAME = "ForestHeightSampledPoints"
LEGACY_SPAWNER_NODE_NAME = "ForestMeshSpawner"
CREATE_POINTS_NODE_NAMES = {
    "forest.asset.birch": "ForestBirchHeightSampledPoints",
    "forest.asset.pine": "ForestPineHeightSampledPoints",
}
SPAWNER_NODE_NAMES = {
    "forest.asset.birch": "ForestBirchMeshSpawner",
    "forest.asset.pine": "ForestPineMeshSpawner",
}
OUTPUT_NODE_NAME = "DefaultOutputNode"
INPUT_NODE_NAME = "DefaultInputNode"
CREATE_POINTS_NATIVE_ALIASES = ("Create Points", "创建点")
SPAWNER_NATIVE_ALIASES = ("Static Mesh Spawner", "静态网格体生成器")

SLOT_ASSET_NAMES = {
    "forest.asset.birch": "SM_Forest_Birch_final",
    "forest.asset.boulder": "SM_Forest_Boulder_final",
    "forest.asset.pine": "SM_Forest_Pine_final",
}
SLOT_ALIASES = {
    "birch": "forest.asset.birch",
    "boulder": "forest.asset.boulder",
    "pine": "forest.asset.pine",
}
REQUIRED_TREE_SLOTS = frozenset(("forest.asset.birch", "forest.asset.pine"))
CONTENT_ADDRESS_DIGEST_LENGTH = 12

GRAPH_DESCRIPTION = (
    "LingTu Forest_HF 2 km deterministic offline PCG bake. MuJoCo is the sole physics, "
    "contact, and raycast authority. Every generated Unreal mesh is VisualOnly, NoCollision, "
    "non-overlapping, and excluded from navigation. Pine/birch retain their exact bake-contract "
    "mesh_slot through separate deterministic Create Points and HISM spawner branches. Seed=20260813."
)
CANONICAL_VOLUME_TRANSFORM = {
    "location": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
    "scale": {"x": 1000.0, "y": 1000.0, "z": 100.0},
}
CANONICAL_VOLUME_BOUNDS = {
    "min": {"x": -100000.0, "y": -100000.0, "z": -10000.0},
    "max": {"x": 100000.0, "y": 100000.0, "z": 10000.0},
    "isValid": True,
}
SOURCE_SHA_TAG = "LingTu.SourceFbxSha256"
SOURCE_SLOT_TAG = "LingTu.ForestAssetSlot"
AUTHORITY_TAG = "LingTu.Authority"


class ForestMcpError(RuntimeError):
    """Base error for fail-closed Forest_HF MCP authoring."""


class McpProtocolError(ForestMcpError):
    """Raised when the MCP transport does not return an explicit valid result."""


class McpToolError(ForestMcpError):
    """Raised when a remote MCP tool reports failure."""


class AuthoringInputError(ForestMcpError, ValueError):
    """Raised when an offline authoring input violates the fixed contract."""


class McpTransport(Protocol):
    """Minimal transport used by the deterministic authoring workflow."""

    def initialize(self) -> None:
        """Create an MCP session and retain its session identifier."""

    def notify_initialized(self) -> None:
        """Send the MCP initialized notification for the current session."""

    def call_tool(self, toolset_name: str, tool_name: str, arguments: Mapping[str, object]) -> Any:
        """Call one reviewed UE tool and return its decoded ``returnValue``."""


class SlotFbx(NamedTuple):
    """One validated deterministic slot-to-FBX binding."""

    slot: str
    source_path: Path
    asset_name: str
    asset_path: str
    asset_ref: str
    sha256: str


class AuthoringInputs(NamedTuple):
    """Validated offline inputs consumed by the UE mutation workflow."""

    contract_path: Path
    points_path: Path
    contract: dict[str, Any]
    points: dict[str, Any]
    point_groups: tuple[PointGroup, ...]
    slots: tuple[SlotFbx, ...]
    contract_sha256: str
    points_sha256: str
    point_count: int
    slot_point_counts: dict[str, int]
    slot_sequence_sha256: str
    placement_sha256: str
    binding_sha256: str


class ContractPoint(NamedTuple):
    """One contract instance projected into the fields Create Points must preserve."""

    slot: str
    stable_id: str
    location_xy: tuple[float, float]
    rotation: tuple[float, float, float]
    scale: tuple[float, float, float]
    seed: int


class PointGroup(NamedTuple):
    """One exact mesh-slot branch with native Create Points JSON parameters."""

    slot: str
    json_params: dict[str, Any]
    point_count: int


def canonical_json_bytes(value: object) -> bytes:
    """Return the same indented canonical JSON bytes as ``build_forest_hf.py``."""

    return (json.dumps(value, sort_keys=True, indent=2, separators=(",", ": "), allow_nan=False) + "\n").encode("utf-8")


def _compact_json(value: object) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AuthoringInputError(f"could not load {label} JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise AuthoringInputError(f"{label} JSON root must be an object: {path}")
    return value


def _finite_number(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AuthoringInputError(f"{field} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise AuthoringInputError(f"{field} must be a finite number")
    return number


def _validate_xyz(value: object, field: str) -> None:
    if not isinstance(value, Mapping) or set(value) != {"x", "y", "z"}:
        raise AuthoringInputError(f"{field} must contain exactly x, y, and z")
    for axis in ("x", "y", "z"):
        _finite_number(value[axis], f"{field}.{axis}")


def _finite_sequence(value: object, size: int, field: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != size:
        raise AuthoringInputError(f"{field} must contain exactly {size} finite numbers")
    return tuple(_finite_number(item, f"{field}[{index}]") for index, item in enumerate(value))


def _validate_contract(contract: Mapping[str, object]) -> tuple[ContractPoint, ...]:
    if contract.get("schema") != CONTRACT_SCHEMA:
        raise AuthoringInputError(f"contract schema must be {CONTRACT_SCHEMA}")
    if contract.get("world_package") != WORLD_PACKAGE:
        raise AuthoringInputError(f"contract world_package must be {WORLD_PACKAGE}")
    if contract.get("seed") != FIXED_SEED:
        raise AuthoringInputError(f"contract seed must be {FIXED_SEED}")

    content_digest = contract.get("content_digest")
    if not isinstance(content_digest, str) or len(content_digest) != 64:
        raise AuthoringInputError("contract content_digest must be a lowercase SHA256")
    body = dict(contract)
    del body["content_digest"]
    expected_digest = hashlib.sha256(canonical_json_bytes(body)).hexdigest()
    if content_digest != expected_digest:
        raise AuthoringInputError("contract content_digest does not cover the current contract body")

    target = contract.get("target")
    if not isinstance(target, Mapping):
        raise AuthoringInputError("contract target is missing")
    if target.get("map") != MAP_PATH or target.get("asset_root") != ASSET_FOLDER.rsplit("/", 1)[0]:
        raise AuthoringInputError("contract targets an unexpected Unreal map or asset root")
    world_partition = target.get("world_partition")
    if not isinstance(world_partition, Mapping) or world_partition.get("enabled") is not True:
        raise AuthoringInputError("contract must require World Partition")

    generation = contract.get("generation")
    if not isinstance(generation, Mapping):
        raise AuthoringInputError("contract generation policy is missing")
    if (
        generation.get("mode") != "offline_baked_hism"
        or generation.get("deterministic") is not True
        or generation.get("runtime_generation") is not False
    ):
        raise AuthoringInputError("contract generation policy is not deterministic offline HISM")

    authority = contract.get("authority")
    if not isinstance(authority, Mapping):
        raise AuthoringInputError("contract authority policy is missing")
    required_authority = {
        "physics": "mujoco",
        "raycast": "mujoco",
        "render_actors": "VisualOnly",
        "collision_profile": "NoCollision",
        "render_meshes_are_colliders": False,
    }
    if any(authority.get(key) != expected for key, expected in required_authority.items()):
        raise AuthoringInputError("contract does not keep MuJoCo authority and Unreal NoCollision")

    density = contract.get("foliage_density_contract")
    if not isinstance(density, Mapping):
        raise AuthoringInputError("contract foliage density policy is missing")
    accepted = density.get("accepted_point_count")
    if isinstance(accepted, bool) or not isinstance(accepted, int) or accepted <= 0:
        raise AuthoringInputError("contract accepted_point_count must be a positive integer")

    scale_contract = contract.get("tree_scale_contract")
    if not isinstance(scale_contract, Mapping):
        raise AuthoringInputError("contract tree_scale_contract is missing")
    scale_factor = _finite_number(scale_contract.get("pcg_scale_factor"), "tree_scale_contract.pcg_scale_factor")
    if scale_factor <= 0.0:
        raise AuthoringInputError("tree_scale_contract.pcg_scale_factor must be positive")

    slots: set[str] = set()
    cells = contract.get("authoring_density_cells")
    if isinstance(cells, (str, bytes)) or not isinstance(cells, Sequence):
        raise AuthoringInputError("contract authoring_density_cells are missing")
    expected_points: list[ContractPoint] = []
    for cell_index, cell in enumerate(cells):
        if not isinstance(cell, Mapping):
            raise AuthoringInputError(f"contract cell {cell_index} must be an object")
        groups = cell.get("hism_groups")
        if isinstance(groups, (str, bytes)) or not isinstance(groups, Sequence):
            raise AuthoringInputError(f"contract cell {cell_index} HISM groups are missing")
        for group_index, group in enumerate(groups):
            if not isinstance(group, Mapping):
                raise AuthoringInputError(f"contract cell {cell_index} group {group_index} must be an object")
            slot = group.get("mesh_slot")
            if not isinstance(slot, str):
                raise AuthoringInputError("contract HISM mesh_slot must be a string")
            slots.add(slot)
            if (
                group.get("classification") != "VisualOnly"
                or group.get("collision_profile") != "NoCollision"
                or group.get("collision_enabled") is not False
                or group.get("generate_overlap_events") is not False
                or group.get("simulate_physics") is not False
                or group.get("can_ever_affect_navigation") is not False
            ):
                raise AuthoringInputError(f"contract HISM group {slot} violates the NoCollision boundary")
            instances = group.get("instances")
            if isinstance(instances, (str, bytes)) or not isinstance(instances, Sequence):
                raise AuthoringInputError(f"contract HISM group {slot} instances are missing")
            for instance_index, instance in enumerate(instances):
                if not isinstance(instance, Mapping):
                    raise AuthoringInputError(f"contract HISM group {slot} instance {instance_index} must be an object")
                stable_id = instance.get("source_stable_id")
                transform = instance.get("unreal_transform")
                if not isinstance(stable_id, str) or not stable_id:
                    raise AuthoringInputError(f"contract HISM group {slot} instance stable id is missing")
                if not isinstance(transform, Mapping):
                    raise AuthoringInputError(f"contract HISM group {slot} instance transform is missing")
                location = _finite_sequence(transform.get("location_cm"), 3, "instance.location_cm")
                rotation = _finite_sequence(transform.get("rotation_deg"), 3, "instance.rotation_deg")
                source_scale = _finite_sequence(transform.get("scale_xyz"), 3, "instance.scale_xyz")
                scaled = tuple(scale_factor * number for number in source_scale)
                expected_points.append(
                    ContractPoint(
                        slot=slot,
                        stable_id=stable_id,
                        location_xy=(location[0], location[1]),
                        rotation=(rotation[0], rotation[1], rotation[2]),
                        scale=(scaled[0], scaled[1], scaled[2]),
                        seed=int.from_bytes(hashlib.sha256(stable_id.encode()).digest()[:4], "big") & 0x7FFFFFFF,
                    )
                )
    if slots != REQUIRED_TREE_SLOTS:
        raise AuthoringInputError(f"contract tree slots must be exactly {sorted(REQUIRED_TREE_SLOTS)}")
    if len(expected_points) != accepted:
        raise AuthoringInputError("contract accepted_point_count does not match HISM instances")
    return tuple(expected_points)


def _validate_native_points(
    json_params: Mapping[str, object],
    expected_points: Sequence[ContractPoint],
    slot: str,
) -> None:
    if set(json_params) != {"pointsToCreate", "coordinateSpace", "bCullPointsOutsideVolume"}:
        raise AuthoringInputError(f"Create Points branch {slot} contains unsupported native fields")
    if json_params.get("coordinateSpace") != "World" or json_params.get("bCullPointsOutsideVolume") is not True:
        raise AuthoringInputError(f"Create Points branch {slot} must use World space and volume culling")
    records = json_params.get("pointsToCreate")
    if isinstance(records, (str, bytes)) or not isinstance(records, Sequence):
        raise AuthoringInputError(f"Create Points branch {slot} pointsToCreate must be an array")
    if len(records) != len(expected_points):
        raise AuthoringInputError(
            f"Create Points branch {slot} has {len(records)} points; contract requires {len(expected_points)}"
        )
    for index, (point, expected) in enumerate(zip(records, expected_points)):
        if not isinstance(point, Mapping):
            raise AuthoringInputError(f"Create Points branch {slot} record {index} must be an object")
        if set(point) != {
            "transform",
            "density",
            "boundsMin",
            "boundsMax",
            "color",
            "steepness",
            "seed",
            "metadataEntry",
        }:
            raise AuthoringInputError(f"Create Points branch {slot} record {index} contains unsupported fields")
        if point.get("metadataEntry") != index:
            raise AuthoringInputError(f"Create Points branch {slot} record {index} has unstable metadataEntry")
        transform = point.get("transform")
        if not isinstance(transform, Mapping):
            raise AuthoringInputError(f"Create Points branch {slot} record {index} transform is missing")
        for field in ("location", "rotation", "scale"):
            _validate_xyz(transform.get(field), f"{slot}.pointsToCreate[{index}].transform.{field}")
        location = transform["location"]
        rotation = transform["rotation"]
        scale = transform["scale"]
        if (float(location["x"]), float(location["y"])) != expected.location_xy:  # type: ignore[index]
            raise AuthoringInputError(f"Create Points branch {slot} record {index} location does not match its contract instance")
        if tuple(float(rotation[axis]) for axis in ("x", "y", "z")) != expected.rotation:  # type: ignore[index]
            raise AuthoringInputError(f"Create Points branch {slot} record {index} rotation does not match its contract instance")
        if tuple(float(scale[axis]) for axis in ("x", "y", "z")) != expected.scale:  # type: ignore[index]
            raise AuthoringInputError(f"Create Points branch {slot} record {index} scale does not match its contract instance")
        _validate_xyz(point.get("boundsMin"), f"{slot}.pointsToCreate[{index}].boundsMin")
        _validate_xyz(point.get("boundsMax"), f"{slot}.pointsToCreate[{index}].boundsMax")
        density = _finite_number(point.get("density"), f"{slot}.pointsToCreate[{index}].density")
        if not 0.0 <= density <= 1.0:
            raise AuthoringInputError(f"Create Points branch {slot} record {index} density is outside [0, 1]")
        _finite_number(point.get("steepness"), f"{slot}.pointsToCreate[{index}].steepness")
        if point.get("seed") != expected.seed:
            raise AuthoringInputError(f"Create Points branch {slot} record {index} seed does not match its contract instance")
        if point.get("color") != {"x": 1.0, "y": 1.0, "z": 1.0, "w": 1.0}:
            raise AuthoringInputError(f"Create Points branch {slot} record {index} color must remain canonical white")


def _validate_points(
    points: Mapping[str, object],
    expected_points: Sequence[ContractPoint],
    contract_content_digest: str,
) -> tuple[PointGroup, ...]:
    expected_fields = {
        "schema",
        "world_package",
        "seed",
        "contract_content_digest",
        "selection_policy",
        "point_count",
        "slot_counts",
        "groups",
        "content_digest",
    }
    if set(points) != expected_fields:
        raise AuthoringInputError("Create Points wrapper contains unsupported or missing fields")
    if points.get("schema") != CREATE_POINTS_SCHEMA:
        raise AuthoringInputError(f"Create Points schema must be {CREATE_POINTS_SCHEMA}")
    if points.get("world_package") != WORLD_PACKAGE or points.get("seed") != FIXED_SEED:
        raise AuthoringInputError("Create Points wrapper does not target the fixed Forest_HF package and seed")
    if points.get("contract_content_digest") != contract_content_digest:
        raise AuthoringInputError("Create Points wrapper is not bound to the supplied bake content_digest")
    if points.get("selection_policy") != EXACT_MESH_SLOT_SELECTION_POLICY:
        raise AuthoringInputError("Create Points wrapper must require exact mesh-slot branches")
    content_digest = points.get("content_digest")
    body = dict(points)
    body.pop("content_digest", None)
    if not isinstance(content_digest, str) or content_digest != hashlib.sha256(canonical_json_bytes(body)).hexdigest():
        raise AuthoringInputError("Create Points content_digest does not cover the current wrapper body")

    expected_by_slot = {
        slot: [point for point in expected_points if point.slot == slot]
        for slot in sorted(REQUIRED_TREE_SLOTS)
    }
    expected_counts = {slot: len(records) for slot, records in expected_by_slot.items()}
    if points.get("slot_counts") != expected_counts:
        raise AuthoringInputError("Create Points slot_counts do not match the bake contract")
    if points.get("point_count") != len(expected_points):
        raise AuthoringInputError("Create Points point_count does not match the bake contract")

    raw_groups = points.get("groups")
    if isinstance(raw_groups, (str, bytes)) or not isinstance(raw_groups, Sequence):
        raise AuthoringInputError("Create Points groups must be an array")
    groups: list[PointGroup] = []
    observed_slots: list[str] = []
    for group_index, group in enumerate(raw_groups):
        if not isinstance(group, Mapping) or set(group) != {
            "mesh_slot",
            "point_count",
            "source_bindings",
            "json_params",
        }:
            raise AuthoringInputError(f"Create Points group {group_index} contains unsupported fields")
        slot = group.get("mesh_slot")
        if not isinstance(slot, str) or slot not in REQUIRED_TREE_SLOTS:
            raise AuthoringInputError(f"Create Points group {group_index} has an unsupported mesh_slot")
        observed_slots.append(slot)
        expected_slot_points = expected_by_slot[slot]
        if group.get("point_count") != len(expected_slot_points):
            raise AuthoringInputError(f"Create Points group {slot} point_count does not match the bake contract")
        bindings = group.get("source_bindings")
        if isinstance(bindings, (str, bytes)) or not isinstance(bindings, Sequence):
            raise AuthoringInputError(f"Create Points group {slot} source_bindings must be an array")
        if len(bindings) != len(expected_slot_points):
            raise AuthoringInputError(f"Create Points group {slot} source binding count does not match the bake contract")
        for index, (binding, expected) in enumerate(zip(bindings, expected_slot_points)):
            if not isinstance(binding, Mapping) or set(binding) != {
                "metadataEntry",
                "mesh_slot",
                "source_stable_id",
            }:
                raise AuthoringInputError(f"Create Points group {slot} source binding {index} is invalid")
            if (
                binding.get("metadataEntry") != index
                or binding.get("mesh_slot") != slot
                or binding.get("source_stable_id") != expected.stable_id
            ):
                raise AuthoringInputError(
                    f"Create Points group {slot} source binding does not match contract instance {index}"
                )
        json_params = group.get("json_params")
        if not isinstance(json_params, Mapping):
            raise AuthoringInputError(f"Create Points group {slot} json_params must be an object")
        _validate_native_points(json_params, expected_slot_points, slot)
        groups.append(PointGroup(slot=slot, json_params=dict(json_params), point_count=len(expected_slot_points)))
    if observed_slots != sorted(REQUIRED_TREE_SLOTS):
        raise AuthoringInputError("Create Points groups must contain each canonical tree slot exactly once in order")
    return tuple(groups)


def _parse_slot_fbx(bindings: Sequence[str]) -> tuple[SlotFbx, ...]:
    parsed: dict[str, Path] = {}
    for binding in bindings:
        requested_slot, separator, raw_path = binding.partition("=")
        if not separator or not requested_slot or not raw_path:
            raise AuthoringInputError(f"invalid --slot-fbx binding {binding!r}; expected SLOT=PATH")
        slot = SLOT_ALIASES.get(requested_slot, requested_slot)
        if slot not in SLOT_ASSET_NAMES:
            raise AuthoringInputError(f"unsupported Forest_HF asset slot: {requested_slot}")
        if slot in parsed:
            raise AuthoringInputError(f"duplicate Forest_HF asset slot: {slot}")
        parsed[slot] = Path(raw_path).expanduser().resolve()
    missing = REQUIRED_TREE_SLOTS - parsed.keys()
    if missing:
        raise AuthoringInputError(f"missing required Forest_HF FBX slots: {sorted(missing)}")

    result: list[SlotFbx] = []
    for slot in sorted(parsed):
        source_path = parsed[slot]
        if source_path.suffix.lower() != ".fbx":
            raise AuthoringInputError(f"Forest_HF slot {slot} must reference an FBX file")
        if not source_path.is_file():
            raise AuthoringInputError(f"Forest_HF FBX does not exist: {source_path}")
        sha256 = _sha256_file(source_path)
        asset_name = f"{SLOT_ASSET_NAMES[slot]}_{sha256[:CONTENT_ADDRESS_DIGEST_LENGTH]}"
        asset_path = f"{ASSET_FOLDER}/{asset_name}"
        result.append(
            SlotFbx(
                slot=slot,
                source_path=source_path,
                asset_name=asset_name,
                asset_path=asset_path,
                asset_ref=f"{asset_path}.{asset_name}",
                sha256=sha256,
            )
        )
    return tuple(result)


def load_authoring_inputs(
    contract_path: Path,
    points_path: Path,
    slot_fbx_bindings: Sequence[str],
) -> AuthoringInputs:
    """Load and fully validate the fixed bake contract, points, and FBX files."""

    resolved_contract = contract_path.expanduser().resolve()
    resolved_points = points_path.expanduser().resolve()
    contract = _load_json_object(resolved_contract, "Forest_HF bake contract")
    expected_points = _validate_contract(contract)
    points = _load_json_object(resolved_points, "Forest_HF Create Points")
    point_groups = _validate_points(points, expected_points, str(contract["content_digest"]))
    point_count = sum(group.point_count for group in point_groups)
    slot_point_counts = {group.slot: group.point_count for group in point_groups}
    slot_sequence_sha256 = hashlib.sha256(
        (_compact_json([point.slot for point in expected_points]) + "\n").encode("utf-8")
    ).hexdigest()
    placement_sha256 = hashlib.sha256(
        (
            _compact_json(
                [
                    {
                        "slot": point.slot,
                        "stable_id": point.stable_id,
                        "location_xy": point.location_xy,
                        "rotation": point.rotation,
                        "scale": point.scale,
                        "seed": point.seed,
                    }
                    for point in expected_points
                ]
            )
            + "\n"
        ).encode("utf-8")
    ).hexdigest()
    contract_sha256 = _sha256_file(resolved_contract)
    points_sha256 = _sha256_file(resolved_points)
    binding_sha256 = hashlib.sha256(
        canonical_json_bytes(
            {
                "contract_content_digest": contract["content_digest"],
                "contract_file_sha256": contract_sha256,
                "create_points_file_sha256": points_sha256,
                "create_points_content_digest": points["content_digest"],
                "point_count": point_count,
                "selection_policy": EXACT_MESH_SLOT_SELECTION_POLICY,
                "slot_point_counts": slot_point_counts,
                "placement_sha256": placement_sha256,
                "slot_sequence_sha256": slot_sequence_sha256,
            }
        )
    ).hexdigest()
    return AuthoringInputs(
        contract_path=resolved_contract,
        points_path=resolved_points,
        contract=contract,
        points=points,
        point_groups=point_groups,
        slots=_parse_slot_fbx(slot_fbx_bindings),
        contract_sha256=contract_sha256,
        points_sha256=points_sha256,
        point_count=point_count,
        slot_point_counts=slot_point_counts,
        slot_sequence_sha256=slot_sequence_sha256,
        placement_sha256=placement_sha256,
        binding_sha256=binding_sha256,
    )


def build_authoring_plan(inputs: AuthoringInputs) -> dict[str, object]:
    """Build the deterministic no-network plan displayed by the default dry-run."""

    assets = [
        {
            "slot": slot.slot,
            "source_fbx": str(slot.source_path),
            "source_sha256": slot.sha256,
            "asset_path": slot.asset_path,
            "asset_ref": slot.asset_ref,
        }
        for slot in inputs.slots
    ]
    return {
        "schema": PLAN_SCHEMA,
        "mode": "dry_run",
        "apply_required": True,
        "mesh_identity_policy": "fbx_sha256_content_addressed",
        "world_package": WORLD_PACKAGE,
        "seed": FIXED_SEED,
        "inputs": {
            "contract": {"path": str(inputs.contract_path), "sha256": inputs.contract_sha256},
            "create_points": {
                "path": str(inputs.points_path),
                "sha256": inputs.points_sha256,
                "content_digest": inputs.points["content_digest"],
                "point_count": inputs.point_count,
                "slot_point_counts": inputs.slot_point_counts,
                "contract_slot_sequence_sha256": inputs.slot_sequence_sha256,
                "contract_placement_sha256": inputs.placement_sha256,
                "binding_sha256": inputs.binding_sha256,
            },
            "slot_assets": assets,
        },
        "target": {
            "map": MAP_PATH,
            "map_template": OPEN_WORLD_TEMPLATE,
            "graph": GRAPH_ASSET_PATH,
            "graph_object": GRAPH_OBJECT_PATH,
            "asset_folder": ASSET_FOLDER,
            "pcg_volume_name": VOLUME_NAME,
        },
        "authority": {
            "physics": "mujoco",
            "raycast": "mujoco",
            "unreal": "VisualOnly",
            "collision_profile": "NoCollision",
            "overlap_events": False,
            "affects_navigation": False,
        },
        "topology": [
            *[CREATE_POINTS_NODE_NAMES[slot] for slot in sorted(REQUIRED_TREE_SLOTS)],
            *[SPAWNER_NODE_NAMES[slot] for slot in sorted(REQUIRED_TREE_SLOTS)],
            OUTPUT_NODE_NAME,
        ],
        "pcg_spawner_slots": sorted(REQUIRED_TREE_SLOTS),
        "selection_policy": EXACT_MESH_SLOT_SELECTION_POLICY,
        "import_only_slots": sorted(slot.slot for slot in inputs.slots if slot.slot not in REQUIRED_TREE_SLOTS),
        "phases": [
            "initialize_mcp_session",
            "ensure_open_world_map",
            "import_or_reuse_static_meshes_and_remove_collision",
            "create_or_update_pcg_graph",
            "configure_exact_mesh_slot_hism_spawners",
            "create_or_update_partitioned_pcg_volume",
            "execute_graph_instance",
            "audit_generated_hism_count_meshes_and_nocollision",
            "save_exact_assets",
            "reload_map_and_repeat_generated_hism_audit",
        ],
    }


class HttpMcpTransport:
    """Strict standard-library JSON-RPC transport for the loopback MCP endpoint."""

    def __init__(self, endpoint: str = DEFAULT_ENDPOINT, timeout_seconds: float = 30.0, opener: Any = None) -> None:
        """Create a transport without opening a network connection."""

        parsed = urllib.parse.urlsplit(endpoint)
        if parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost", "::1"}:
            raise McpProtocolError("Forest_HF authoring only permits a loopback HTTP MCP endpoint")
        if parsed.username is not None or parsed.password is not None or parsed.fragment:
            raise McpProtocolError("MCP endpoint must not contain credentials or a fragment")
        self._endpoint = endpoint
        self._timeout_seconds = timeout_seconds
        self._opener = opener or urllib.request.urlopen
        self._next_id = 1
        self._session_id: str | None = None
        self._initialized = False

    @property
    def session_id(self) -> str | None:
        """Return the negotiated MCP session identifier, if initialized."""

        return self._session_id

    def _post(
        self,
        message: Mapping[str, object],
        *,
        include_session: bool,
        allow_empty: bool = False,
    ) -> tuple[dict[str, Any] | None, Mapping[str, str]]:
        headers = {
            "Accept": "application/json, text/event-stream",
            "Content-Type": "application/json",
        }
        if include_session:
            if self._session_id is None:
                raise McpProtocolError("MCP session has not been initialized")
            headers["Mcp-Session-Id"] = self._session_id
        request = urllib.request.Request(  # noqa: S310 - endpoint is validated as loopback HTTP above.
            self._endpoint,
            data=_compact_json(message).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        try:
            with self._opener(request, timeout=self._timeout_seconds) as response:
                payload = response.read().decode("utf-8")
                response_headers = response.headers
        except (OSError, UnicodeError, urllib.error.URLError) as error:
            raise McpProtocolError(f"MCP request failed closed: {error}") from error
        if not payload.strip():
            if allow_empty:
                return None, response_headers
            raise McpProtocolError("MCP response body is empty")
        text = payload.strip()
        if text.startswith("data:"):
            data_lines = [line[5:].strip() for line in text.splitlines() if line.startswith("data:")]
            if not data_lines:
                raise McpProtocolError("MCP SSE response contains no data event")
            text = data_lines[-1]
        try:
            decoded = json.loads(text)
        except json.JSONDecodeError as error:
            raise McpProtocolError("MCP response is not valid JSON") from error
        if not isinstance(decoded, dict):
            raise McpProtocolError("MCP response root must be an object")
        if "error" in decoded:
            raise McpToolError(f"MCP JSON-RPC error: {_compact_json(decoded['error'])}")
        return decoded, response_headers

    def initialize(self) -> None:
        """Negotiate one MCP session and require an explicit session header."""

        request_id = self._next_id
        self._next_id += 1
        response, headers = self._post(
            {
                "jsonrpc": "2.0",
                "id": request_id,
                "method": "initialize",
                "params": {
                    "protocolVersion": "2025-06-18",
                    "capabilities": {},
                    "clientInfo": {"name": "lingtu-forest-hf-authoring", "version": "1.0.0"},
                },
            },
            include_session=False,
        )
        if response is None or response.get("id") != request_id or "result" not in response:
            raise McpProtocolError("MCP initialize response is incomplete")
        session_id = headers.get("Mcp-Session-Id") or headers.get("mcp-session-id")
        if not isinstance(session_id, str) or not session_id.strip():
            raise McpProtocolError("MCP initialize response omitted Mcp-Session-Id")
        self._session_id = session_id.strip()

    def notify_initialized(self) -> None:
        """Complete MCP initialization for the negotiated session."""

        self._post(
            {"jsonrpc": "2.0", "method": "notifications/initialized", "params": {}},
            include_session=True,
            allow_empty=True,
        )
        self._initialized = True

    def call_tool(self, toolset_name: str, tool_name: str, arguments: Mapping[str, object]) -> Any:
        """Call the MCP ``call_tool`` bridge and decode its explicit return value."""

        if not self._initialized:
            raise McpProtocolError("MCP initialized notification has not completed")
        request_id = self._next_id
        self._next_id += 1
        response, _ = self._post(
            {
                "jsonrpc": "2.0",
                "id": request_id,
                "method": "tools/call",
                "params": {
                    "name": "call_tool",
                    "arguments": {
                        "toolset_name": toolset_name,
                        "tool_name": tool_name,
                        "arguments": dict(arguments),
                    },
                },
            },
            include_session=True,
        )
        if response is None or response.get("id") != request_id:
            raise McpProtocolError("MCP tool response has the wrong request id")
        result = response.get("result")
        if not isinstance(result, Mapping):
            raise McpProtocolError("MCP tool response omitted result")
        if result.get("isError") is True:
            raise McpToolError(f"{toolset_name}.{tool_name} failed: {_compact_json(result)}")
        content = result.get("content")
        if isinstance(content, (str, bytes)) or not isinstance(content, Sequence):
            raise McpProtocolError(f"{toolset_name}.{tool_name} returned no structured content")
        text_blocks = [
            block.get("text")
            for block in content
            if isinstance(block, Mapping) and block.get("type") == "text" and isinstance(block.get("text"), str)
        ]
        if len(text_blocks) != 1:
            raise McpProtocolError(f"{toolset_name}.{tool_name} must return exactly one text result")
        text_value = text_blocks[0]
        if not isinstance(text_value, str):
            raise McpProtocolError(f"{toolset_name}.{tool_name} returned an invalid text block")
        try:
            tool_result = json.loads(text_value)
        except json.JSONDecodeError as error:
            raise McpProtocolError(f"{toolset_name}.{tool_name} returned non-JSON text") from error
        if not isinstance(tool_result, Mapping) or "returnValue" not in tool_result:
            raise McpProtocolError(f"{toolset_name}.{tool_name} omitted returnValue")
        return tool_result["returnValue"]


def _ref_path(value: object) -> str | None:
    if isinstance(value, str):
        return value
    if isinstance(value, Mapping):
        ref_path = value.get("refPath")
        if isinstance(ref_path, str):
            return ref_path
    return None


def _json_mapping(value: object, label: str) -> Mapping[str, object]:
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError as error:
            raise McpProtocolError(f"{label} returned invalid JSON") from error
    if not isinstance(value, Mapping):
        raise McpProtocolError(f"{label} did not return an object")
    return value


def _find_ref_path(value: object, key: str | None = None) -> str | None:
    if isinstance(value, Mapping):
        if key is None or key in value:
            candidate = value if key is None else value[key]
            direct = _ref_path(candidate)
            if direct is not None:
                return direct
        for nested_key, nested_value in value.items():
            if key is not None and nested_key == key:
                direct = _ref_path(nested_value)
                if direct is not None:
                    return direct
            nested = _find_ref_path(nested_value, key)
            if nested is not None:
                return nested
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for nested_value in value:
            nested = _find_ref_path(nested_value, key)
            if nested is not None:
                return nested
    return None


def _node_records(structure: object) -> list[Mapping[str, object]]:
    if not isinstance(structure, Mapping):
        raise McpProtocolError("PCG GetGraphStructure did not return an object")
    nodes = structure.get("nodes")
    if isinstance(nodes, (str, bytes)) or not isinstance(nodes, Sequence):
        raise McpProtocolError("PCG graph structure omitted nodes")
    records = [node for node in nodes if isinstance(node, Mapping)]
    if len(records) != len(nodes):
        raise McpProtocolError("PCG graph structure contains an invalid node")
    return records


def _node_path(structure: object, name: str) -> str | None:
    matches: list[str] = []
    for node in _node_records(structure):
        if node.get("name") == name:
            path = _ref_path(node.get("path")) or _ref_path(node)
            if path is not None:
                matches.append(path)
            continue
        path = _ref_path(node.get("path")) or _ref_path(node)
        if isinstance(path, str) and path.rsplit(":", 1)[-1] == name:
            matches.append(path)
    unique_matches = list(dict.fromkeys(matches))
    if len(unique_matches) > 1:
        raise McpProtocolError(f"PCG graph contains duplicate {name} nodes")
    return unique_matches[0] if unique_matches else None


def _edge_node_path(structure: object, token: object) -> str | None:
    """Resolve either a full node ref or UE's bare edge-node name to one full ref."""

    path_or_name = _ref_path(token)
    if path_or_name is None:
        return None
    matches: list[str] = []
    for node in _node_records(structure):
        path = _ref_path(node.get("path")) or _ref_path(node)
        if path is None:
            continue
        name = node.get("name")
        if path_or_name == path or (isinstance(name, str) and path_or_name == name):
            matches.append(path)
    unique_matches = list(dict.fromkeys(matches))
    if len(unique_matches) > 1:
        raise McpProtocolError(f"PCG edge node token is ambiguous: {path_or_name}")
    return unique_matches[0] if unique_matches else None


def _edge_exists(
    structure: object,
    source_path: str,
    source_pin: str,
    target_path: str,
    target_pin: str,
) -> bool:
    if not isinstance(structure, Mapping):
        raise McpProtocolError("PCG graph structure did not return an object")
    edges = structure.get("edges")
    if isinstance(edges, (str, bytes)) or not isinstance(edges, Sequence):
        raise McpProtocolError("PCG graph structure omitted edges")
    for edge in edges:
        if not isinstance(edge, Mapping):
            raise McpProtocolError("PCG graph structure contains an invalid edge")
        if (
            _edge_node_path(structure, edge.get("srcNode")) == source_path
            and edge.get("srcPin") == source_pin
            and _edge_node_path(structure, edge.get("destNode")) == target_path
            and edge.get("destPin") == target_pin
        ):
            return True
    return False


def _validate_canonical_topology(
    structure: object,
    branch_paths: Mapping[str, tuple[str, str]],
    output_path: str,
) -> None:
    allowed_names = {
        INPUT_NODE_NAME,
        OUTPUT_NODE_NAME,
        *CREATE_POINTS_NODE_NAMES.values(),
        *SPAWNER_NODE_NAMES.values(),
    }
    unexpected_nodes: list[str] = []
    for node in _node_records(structure):
        name = node.get("name")
        path = _ref_path(node.get("path")) or _ref_path(node)
        resolved_name = name if isinstance(name, str) else (path.rsplit(":", 1)[-1] if path else "<unknown>")
        if resolved_name not in allowed_names:
            unexpected_nodes.append(resolved_name)
    if unexpected_nodes:
        raise McpProtocolError(f"Forest_HF PCG graph contains unmanaged nodes: {sorted(unexpected_nodes)}")

    if not isinstance(structure, Mapping):
        raise McpProtocolError("PCG graph structure did not return an object")
    edges = structure.get("edges")
    if isinstance(edges, (str, bytes)) or not isinstance(edges, Sequence):
        raise McpProtocolError("PCG graph structure omitted edges")
    observed: set[tuple[str | None, object, str | None, object]] = set()
    for edge in edges:
        if not isinstance(edge, Mapping):
            raise McpProtocolError("PCG graph structure contains an invalid edge")
        observed.add(
            (
                _edge_node_path(structure, edge.get("srcNode")),
                edge.get("srcPin"),
                _edge_node_path(structure, edge.get("destNode")),
                edge.get("destPin"),
            )
        )
    expected = {
        edge
        for points_path, spawner_path in branch_paths.values()
        for edge in (
            (points_path, "Out", spawner_path, "In"),
            (spawner_path, "Out", output_path, "Out"),
        )
    }
    if observed != expected:
        raise McpProtocolError(
            "Forest_HF PCG graph topology is not exactly two mesh-slot Create Points -> HISM Spawner -> Output branches"
        )


def _select_native_node(native_nodes: object, aliases: Sequence[str], role: str) -> str:
    if isinstance(native_nodes, (str, bytes)) or not isinstance(native_nodes, Sequence):
        raise McpProtocolError("PCG ListNativeNodes did not return an array")
    names = [item for item in native_nodes if isinstance(item, str)]
    for alias in aliases:
        if alias in names:
            return alias
    raise McpProtocolError(f"UE PCG native node for {role} is unavailable; candidates={list(aliases)}")


def _mesh_entry(inputs: AuthoringInputs, slot_name: str) -> dict[str, object]:
    by_slot = {slot.slot: slot for slot in inputs.slots}
    slot = by_slot[slot_name]
    return {
        "descriptor": {
            "componentTags": ["LingTuForest", "VisualOnly", "NoCollision"],
            "additionalCommaSeparatedTags": "LingTuForest,VisualOnly,NoCollision",
            "staticMesh": {"refPath": slot.asset_ref},
            "componentClass": {"refPath": "/Script/Engine.HierarchicalInstancedStaticMeshComponent"},
            "mobility": "Static",
            "bodyInstance": {
                "collisionProfileName": "NoCollision",
                "collisionEnabled": "NoCollision",
                "bSimulatePhysics": False,
                "bEnableGravity": False,
                "bNotifyRigidBodyCollision": False,
            },
            "bHasCustomNavigableGeometry": "DontExport",
            "bUseDefaultCollision": False,
            "bGenerateOverlapEvents": False,
            "bCanEverAffectNavigation": False,
            "bForceNavigationObstacle": False,
            "bCastShadow": True,
            "bCastDynamicShadow": True,
            "bCastStaticShadow": True,
            "bVisible": True,
            "bHiddenInGame": False,
            "bIncludeInHLOD": True,
            "instanceStartCullDistance": 5000,
            "instanceEndCullDistance": 120000,
        },
        "weight": 1,
    }


def _projection_mismatch(observed: object, expected: object, path: str) -> str | None:
    """Return the first semantic projection path that differs, ignoring extra map keys."""

    if isinstance(expected, Mapping):
        if not isinstance(observed, Mapping):
            return path
        for key, expected_value in expected.items():
            child_path = f"{path}.{key}" if path else str(key)
            if key not in observed:
                return child_path
            mismatch = _projection_mismatch(observed[key], expected_value, child_path)
            if mismatch is not None:
                return mismatch
        return None
    if isinstance(expected, Sequence) and not isinstance(expected, (str, bytes)):
        if isinstance(observed, (str, bytes)) or not isinstance(observed, Sequence):
            return path
        if path.endswith("componentTags"):
            if (
                len(observed) != len(expected)
                or not all(isinstance(item, str) for item in observed)
                or set(observed) != set(expected)
            ):
                return path
            return None
        if len(observed) != len(expected):
            return path
        for index, (observed_value, expected_value) in enumerate(zip(observed, expected, strict=True)):
            mismatch = _projection_mismatch(observed_value, expected_value, f"{path}[{index}]")
            if mismatch is not None:
                return mismatch
        return None
    if isinstance(expected, bool):
        return None if observed is expected else path
    if isinstance(expected, int):
        return None if isinstance(observed, int) and not isinstance(observed, bool) and observed == expected else path
    return None if observed == expected else path


def _selector_property(selector: Mapping[str, object], name: str) -> object:
    aliases = {
        "MeshEntries": ("MeshEntries", "meshEntries"),
        "MaterialOverrideAttributes": ("MaterialOverrideAttributes", "materialOverrideAttributes"),
    }
    for alias in aliases.get(name, (name,)):
        if alias in selector:
            return selector[alias]
    return None


def _validate_mesh_selector_semantics(
    observed_selector: Mapping[str, object],
    desired_selector: Mapping[str, object],
    slot: str,
) -> None:
    """Audit safety/reproducibility fields while tolerating UE-expanded defaults."""

    normalized = {
        key: _selector_property(observed_selector, key)
        for key in ("MeshEntries", "bUseAttributeMaterialOverrides", "MaterialOverrideAttributes")
    }
    mismatch = _projection_mismatch(normalized, desired_selector, "selector")
    if mismatch is not None:
        raise McpToolError(f"PCG mesh selector for {slot} has safety semantic drift at {mismatch}")


class _AuthoringSession:
    def __init__(self, transport: McpTransport) -> None:
        self.transport = transport
        self.call_count = 0

    def call(self, toolset: str, tool: str, arguments: Mapping[str, object]) -> Any:
        self.call_count += 1
        try:
            return self.transport.call_tool(toolset, tool, arguments)
        except ForestMcpError:
            raise
        except Exception as error:
            raise McpToolError(f"{toolset}.{tool} failed closed: {error}") from error

    def exists(self, path: str) -> bool:
        result = self.call(ASSET_TOOLS, "exists", {"path": path})
        if not isinstance(result, bool):
            raise McpProtocolError(f"AssetTools.exists({path}) did not return a boolean")
        return result

    def require_true(self, toolset: str, tool: str, arguments: Mapping[str, object]) -> None:
        result = self.call(toolset, tool, arguments)
        if result is not True:
            raise McpToolError(f"{toolset}.{tool} did not report success")

    def ensure_folder(self, path: str) -> None:
        if not self.exists(path):
            self.require_true(ASSET_TOOLS, "create_folder", {"path": path})
            if not self.exists(path):
                raise McpToolError(f"AssetTools.create_folder did not create {path}")


def _ensure_map(session: _AuthoringSession) -> str:
    created = False
    if not session.exists(MAP_PATH):
        session.ensure_folder(MAP_FOLDER)
        if not session.exists(OPEN_WORLD_TEMPLATE):
            raise McpToolError(f"required OpenWorld template is missing: {OPEN_WORLD_TEMPLATE}")
        session.require_true(
            ASSET_TOOLS,
            "duplicate",
            {"path": OPEN_WORLD_TEMPLATE, "new_path": MAP_PATH},
        )
        created = True
        if not session.exists(MAP_PATH):
            raise McpToolError(f"AssetTools.duplicate did not create {MAP_PATH}")
    _load_map(session)
    return "created" if created else "reused"


def _load_map(session: _AuthoringSession) -> None:
    _load_level(session, MAP_PATH)


def _load_level(session: _AuthoringSession, level_path: str) -> None:
    loaded = session.call(SCENE_TOOLS, "load_level", {"level_path": level_path})
    if loaded is False:
        raise McpToolError(f"SceneTools.load_level did not load {level_path}")
    current = session.call(SCENE_TOOLS, "get_current_level", {})
    current_path = _ref_path(current)
    if current_path is None or level_path not in current_path:
        raise McpToolError(f"current editor level is not {level_path}: {current_path}")


def _expected_mesh_metadata(slot: SlotFbx) -> dict[str, str]:
    return {
        SOURCE_SHA_TAG: slot.sha256,
        SOURCE_SLOT_TAG: slot.slot,
        AUTHORITY_TAG: "VisualOnly;NoCollision;MuJoCoPhysicsAndRaycast",
    }


def _import_exact_mesh(session: _AuthoringSession, slot: SlotFbx) -> None:
    imported = session.call(
        STATIC_MESH_TOOLS,
        "import_file",
        {
            "folder_path": ASSET_FOLDER,
            "asset_name": slot.asset_name,
            "source_file": str(slot.source_path),
            "import_materials": True,
            "import_textures": True,
            "combine_meshes": True,
        },
    )
    if isinstance(imported, (str, bytes)) or not isinstance(imported, Sequence):
        raise McpProtocolError("StaticMeshTools.import_file did not return imported asset refs")
    imported_refs = {_ref_path(item) for item in imported}
    if slot.asset_ref not in imported_refs or not session.exists(slot.asset_path):
        raise McpToolError(f"StaticMeshTools.import_file did not create exact asset {slot.asset_ref}")


def _configure_mesh(session: _AuthoringSession, slot: SlotFbx, metadata: Mapping[str, str]) -> None:
    session.call(
        ASSET_TOOLS,
        "update_metadata_tags",
        {"asset_path": slot.asset_path, "set_tags": metadata},
    )
    session.require_true(
        STATIC_MESH_TOOLS,
        "remove_collisions",
        {"mesh": {"refPath": slot.asset_ref}},
    )
    verified_metadata = session.call(
        ASSET_TOOLS,
        "get_metadata_tags",
        {"asset_path": slot.asset_path},
    )
    if not isinstance(verified_metadata, Mapping) or any(
        verified_metadata.get(key) != value for key, value in metadata.items()
    ):
        raise McpToolError(f"static mesh provenance/authority metadata did not persist: {slot.asset_path}")


def _ensure_meshes(session: _AuthoringSession, inputs: AuthoringInputs) -> dict[str, str]:
    session.ensure_folder(ASSET_FOLDER)
    outcomes: dict[str, str] = {}
    for slot in inputs.slots:
        existed = session.exists(slot.asset_path)
        expected_metadata = _expected_mesh_metadata(slot)
        if existed:
            metadata = session.call(ASSET_TOOLS, "get_metadata_tags", {"asset_path": slot.asset_path})
            if not isinstance(metadata, Mapping):
                raise McpProtocolError(f"AssetTools.get_metadata_tags({slot.asset_path}) returned no object")
            if any(metadata.get(key) != value for key, value in expected_metadata.items()):
                raise McpToolError(
                    f"content-addressed asset metadata mismatch at {slot.asset_path}; "
                    "refusing to adopt, overwrite, or delete the existing asset"
                )
            _configure_mesh(session, slot, expected_metadata)
            outcomes[slot.slot] = "reused"
        else:
            _import_exact_mesh(session, slot)
            _configure_mesh(session, slot, expected_metadata)
            outcomes[slot.slot] = "created"
    return outcomes


def _add_node(
    session: _AuthoringSession,
    native_type: str,
    node_name: str,
    title: str,
    json_params: str,
    x_index: int,
    y_index: int,
) -> None:
    added = session.call(
        PCG_TOOLS,
        "AddNode",
        {
            "graph": {"refPath": GRAPH_OBJECT_PATH},
            "nativeNodeType": native_type,
            "nodeName": node_name,
            "jsonParams": json_params,
            "nodeTitle": title,
            "nodeComment": GRAPH_DESCRIPTION,
            "xPositionIdx": x_index,
            "yPositionIdx": y_index,
        },
    )
    expected_path = f"{GRAPH_OBJECT_PATH}:{node_name}"
    if _ref_path(added) != expected_path:
        raise McpToolError(f"PCG AddNode did not create exact node {expected_path}")


def _ensure_graph(session: _AuthoringSession, inputs: AuthoringInputs) -> tuple[str, dict[str, str]]:
    session.ensure_folder(GRAPH_FOLDER)
    graph_existed = session.exists(GRAPH_ASSET_PATH)
    if not graph_existed:
        created_graph = session.call(PCG_TOOLS, "CreateGraph", {"name": GRAPH_NAME, "path": GRAPH_FOLDER})
        if _ref_path(created_graph) != GRAPH_OBJECT_PATH:
            raise McpToolError(f"PCG CreateGraph did not create exact graph {GRAPH_OBJECT_PATH}")
        if not session.exists(GRAPH_ASSET_PATH):
            raise McpToolError(f"PCG CreateGraph did not create {GRAPH_ASSET_PATH}")

    graph_ref = {"refPath": GRAPH_OBJECT_PATH}
    structure = session.call(PCG_TOOLS, "GetGraphStructure", {"graph": graph_ref})
    graph_mutated = False
    for legacy_name in (LEGACY_CREATE_POINTS_NODE_NAME, LEGACY_SPAWNER_NODE_NAME):
        legacy_path = _node_path(structure, legacy_name)
        if legacy_path is not None:
            session.require_true(
                PCG_TOOLS,
                "RemoveNode",
                {"graph": graph_ref, "node": {"refPath": legacy_path}},
            )
            graph_mutated = True
    if graph_mutated:
        structure = session.call(PCG_TOOLS, "GetGraphStructure", {"graph": graph_ref})
        if any(
            _node_path(structure, legacy_name) is not None
            for legacy_name in (LEGACY_CREATE_POINTS_NODE_NAME, LEGACY_SPAWNER_NODE_NAME)
        ):
            raise McpToolError("legacy weighted Forest_HF PCG nodes were not removed")

    groups_by_slot = {group.slot: group for group in inputs.point_groups}
    existing_paths = {
        slot: (
            _node_path(structure, CREATE_POINTS_NODE_NAMES[slot]),
            _node_path(structure, SPAWNER_NODE_NAMES[slot]),
        )
        for slot in sorted(REQUIRED_TREE_SLOTS)
    }
    if any(points_path is None or spawner_path is None for points_path, spawner_path in existing_paths.values()):
        native_nodes = session.call(PCG_TOOLS, "ListNativeNodes", {"bCommonOnly": False})
        create_points_type = _select_native_node(native_nodes, CREATE_POINTS_NATIVE_ALIASES, "Create Points")
        spawner_type = _select_native_node(native_nodes, SPAWNER_NATIVE_ALIASES, "Static Mesh Spawner")
        for y_index, slot in enumerate(sorted(REQUIRED_TREE_SLOTS)):
            species = slot.rsplit(".", 1)[-1].title()
            points_path, spawner_path = existing_paths[slot]
            if points_path is None:
                _add_node(
                    session,
                    create_points_type,
                    CREATE_POINTS_NODE_NAMES[slot],
                    f"Forest {species} Height-Sampled Points",
                    _compact_json(groups_by_slot[slot].json_params),
                    0,
                    y_index,
                )
                graph_mutated = True
            if spawner_path is None:
                _add_node(
                    session,
                    spawner_type,
                    SPAWNER_NODE_NAMES[slot],
                    f"Forest {species} Exact NoCollision HISM Spawner",
                    _compact_json({"bSynchronousLoad": True}),
                    1,
                    y_index,
                )
                graph_mutated = True
        structure = session.call(PCG_TOOLS, "GetGraphStructure", {"graph": graph_ref})
    branch_paths: dict[str, tuple[str, str]] = {}
    for slot in sorted(REQUIRED_TREE_SLOTS):
        points_path = _node_path(structure, CREATE_POINTS_NODE_NAMES[slot])
        spawner_path = _node_path(structure, SPAWNER_NODE_NAMES[slot])
        if points_path is None or spawner_path is None:
            raise McpProtocolError(f"PCG graph does not expose the exact {slot} branch")
        branch_paths[slot] = (points_path, spawner_path)
        if existing_paths[slot][0] is not None:
            session.require_true(
                PCG_TOOLS,
                "UpdateNode",
                {
                    "node": {"refPath": points_path},
                    "jsonParams": _compact_json(groups_by_slot[slot].json_params),
                    "nodeTitle": f"Forest {slot.rsplit('.', 1)[-1].title()} Height-Sampled Points",
                },
            )
        if existing_paths[slot][1] is not None:
            session.require_true(
                PCG_TOOLS,
                "UpdateNode",
                {
                    "node": {"refPath": spawner_path},
                    "jsonParams": _compact_json({"bSynchronousLoad": True}),
                    "nodeTitle": f"Forest {slot.rsplit('.', 1)[-1].title()} Exact NoCollision HISM Spawner",
                },
            )
    output_path = _node_path(structure, OUTPUT_NODE_NAME)
    if output_path is None:
        raise McpProtocolError("PCG graph does not expose the canonical output node")

    for points_path, spawner_path in branch_paths.values():
        if not _edge_exists(structure, points_path, "Out", spawner_path, "In"):
            connected_nodes = session.call(
                PCG_TOOLS,
                "ConnectNodePins",
                {
                    "fromNode": {"refPath": points_path},
                    "fromPinLabel": "Out",
                    "toNode": {"refPath": spawner_path},
                    "toPinLabel": "In",
                },
            )
            if isinstance(connected_nodes, (str, bytes)) or not isinstance(connected_nodes, Sequence):
                raise McpProtocolError("PCG ConnectNodePins did not return its conversion-node array")
            graph_mutated = True
        if not _edge_exists(structure, spawner_path, "Out", output_path, "Out"):
            connected_nodes = session.call(
                PCG_TOOLS,
                "ConnectNodePins",
                {
                    "fromNode": {"refPath": spawner_path},
                    "fromPinLabel": "Out",
                    "toNode": {"refPath": output_path},
                    "toPinLabel": "Out",
                },
            )
            if isinstance(connected_nodes, (str, bytes)) or not isinstance(connected_nodes, Sequence):
                raise McpProtocolError("PCG ConnectNodePins did not return its conversion-node array")
            graph_mutated = True
    session.require_true(
        PCG_TOOLS,
        "SetGraphDescription",
        {"graph": graph_ref, "description": GRAPH_DESCRIPTION},
    )
    verified = session.call(PCG_TOOLS, "GetGraphStructure", {"graph": graph_ref})
    _validate_canonical_topology(verified, branch_paths, output_path)

    selector_paths: dict[str, str] = {}
    for slot, (_points_path, spawner_path) in branch_paths.items():
        node_info = session.call(PCG_TOOLS, "GetNodeInfo", {"node": {"refPath": spawner_path}})
        selector_path = _find_ref_path(node_info, "meshSelectorParameters")
        if selector_path is None:
            raise McpProtocolError(f"PCG Static Mesh Spawner for {slot} did not expose meshSelectorParameters")
        desired_selector = {
            "MeshEntries": [_mesh_entry(inputs, slot)],
            "bUseAttributeMaterialOverrides": False,
            "MaterialOverrideAttributes": [],
        }
        session.require_true(
            OBJECT_TOOLS,
            "set_properties",
            {
                "instance": {"refPath": selector_path},
                "values": _compact_json(desired_selector),
            },
        )
        observed_selector = _json_mapping(
            session.call(
                OBJECT_TOOLS,
                "get_properties",
                {
                    "instance": {"refPath": selector_path},
                    "properties": list(desired_selector),
                },
            ),
            f"PCG mesh selector readback for {slot}",
        )
        _validate_mesh_selector_semantics(observed_selector, desired_selector, slot)
        selector_paths[slot] = selector_path
    outcome = "created" if not graph_existed else ("updated" if graph_mutated else "reused")
    return outcome, selector_paths


def _graph_instance_matches(item: object) -> bool:
    if not isinstance(item, Mapping):
        return False
    return _ref_path(item.get("graph")) == GRAPH_OBJECT_PATH


def _ensure_volume(session: _AuthoringSession) -> tuple[str, str]:
    instances = session.call(PCG_TOOLS, "ListGraphInstances", {})
    if isinstance(instances, (str, bytes)) or not isinstance(instances, Sequence):
        raise McpProtocolError("PCG ListGraphInstances did not return an array")
    matches = [item for item in instances if _graph_instance_matches(item)]
    if len(matches) > 1:
        raise McpProtocolError("multiple PCG volumes reference the Forest_HF graph; refusing to guess")
    if matches:
        actor_path = _find_ref_path(matches[0].get("actor") if isinstance(matches[0], Mapping) else None)
        outcome = "reused"
    else:
        spawned = session.call(
            PCG_TOOLS,
            "SpawnGraphInstance",
            {
                "graph": {"refPath": GRAPH_OBJECT_PATH},
                "name": VOLUME_NAME,
                "transform": CANONICAL_VOLUME_TRANSFORM,
                "jsonParams": "{}",
            },
        )
        actor_path = _find_ref_path(spawned)
        outcome = "created"
    if actor_path is None:
        raise McpProtocolError("PCG graph instance did not expose its actor refPath")
    component_path = f"{actor_path}.PCG Component"
    tags = ["LingTuForestPCG", "VisualOnly", "NoCollision", f"Seed{FIXED_SEED}"]
    session.require_true(
        ACTOR_TOOLS,
        "set_actor_transform",
        {
            "actor": {"refPath": actor_path},
            "xform": CANONICAL_VOLUME_TRANSFORM,
            "worldspace": True,
        },
    )
    observed_transform = session.call(
        ACTOR_TOOLS,
        "get_actor_transform",
        {"actor": {"refPath": actor_path}},
    )
    if observed_transform != CANONICAL_VOLUME_TRANSFORM:
        raise McpToolError("PCG volume did not retain the canonical 2 km transform")
    observed_bounds = session.call(
        ACTOR_TOOLS,
        "get_actor_bounds",
        {"actor": {"refPath": actor_path}},
    )
    if observed_bounds != CANONICAL_VOLUME_BOUNDS:
        raise McpToolError("PCG volume did not retain canonical 2 km x 2 km bounds")
    session.require_true(
        SCENE_TOOLS,
        "set_actor_folder",
        {"actor": {"refPath": actor_path}, "folder_path": "LingTu/ForestHF2km/PCG"},
    )
    actor_values = {
        "tags": tags,
        "bActorEnableCollision": False,
        "bGenerateOverlapEventsDuringLevelStreaming": False,
        "bIsSpatiallyLoaded": True,
    }
    session.require_true(
        OBJECT_TOOLS,
        "set_properties",
        {
            "instance": {"refPath": actor_path},
            "values": _compact_json(actor_values),
        },
    )
    observed_actor = _json_mapping(
        session.call(
            OBJECT_TOOLS,
            "get_properties",
            {"instance": {"refPath": actor_path}, "properties": list(actor_values)},
        ),
        "PCG volume actor property readback",
    )
    if any(observed_actor.get(key) != value for key, value in actor_values.items()):
        raise McpToolError("PCG volume actor did not retain VisualOnly/NoCollision properties")
    component_values = {
        "seed": FIXED_SEED,
        "bActivated": True,
        "bIsComponentPartitioned": True,
        "generationTrigger": "GenerateOnDemand",
        "bGenerateOnDropWhenTriggerOnDemand": False,
        "bRegenerateInEditor": False,
        "bOnlyTrackItself": True,
        "bIgnoreLandscapeTracking": True,
        "bCanEverAffectNavigation": False,
        "componentTags": ["LingTuForest", "VisualOnly", "NoCollision", f"Seed{FIXED_SEED}"],
    }
    session.require_true(
        OBJECT_TOOLS,
        "set_properties",
        {"instance": {"refPath": component_path}, "values": _compact_json(component_values)},
    )
    observed = _json_mapping(
        session.call(
            OBJECT_TOOLS,
            "get_properties",
            {
                "instance": {"refPath": component_path},
                "properties": [
                    "seed",
                    "bIsComponentPartitioned",
                    "generationTrigger",
                    "bCanEverAffectNavigation",
                    "componentTags",
                ],
            },
        ),
        "PCG component property readback",
    )
    required_observed = {
        "seed": FIXED_SEED,
        "bIsComponentPartitioned": True,
        "generationTrigger": "GenerateOnDemand",
        "bCanEverAffectNavigation": False,
    }
    if any(observed.get(key) != expected for key, expected in required_observed.items()):
        raise McpToolError("PCG component did not retain deterministic partition/authority properties")
    observed_tags = observed.get("componentTags")
    if not isinstance(observed_tags, Sequence) or isinstance(observed_tags, (str, bytes)):
        raise McpToolError("PCG component did not retain NoCollision tags")
    if not {"VisualOnly", "NoCollision"}.issubset(set(observed_tags)):
        raise McpToolError("PCG component did not retain VisualOnly/NoCollision tags")
    execution_messages = session.call(
        PCG_TOOLS,
        "ExecuteGraphInstance",
        {"pCGVolume": {"refPath": actor_path}},
    )
    if isinstance(execution_messages, (str, bytes)) or not isinstance(execution_messages, Sequence):
        raise McpProtocolError("PCG ExecuteGraphInstance did not return an execution message array")
    blocking_messages = [
        message
        for message in execution_messages
        if isinstance(message, Mapping) and str(message.get("severity", "")).lower() in {"error", "fatal"}
    ]
    if blocking_messages:
        raise McpToolError(f"PCG execution reported errors: {_compact_json(blocking_messages)}")
    return outcome, actor_path


def _audit_generated_hisms(session: _AuthoringSession, inputs: AuthoringInputs) -> dict[str, object]:
    actors = session.call(
        SCENE_TOOLS,
        "find_actors",
        {
            "name": "",
            "tag": "",
            "collision_channels": [],
        },
    )
    if isinstance(actors, (str, bytes)) or not isinstance(actors, Sequence) or not actors:
        raise McpToolError("no 256 m PCG World Partition actors were generated")
    all_actor_paths = [path for actor in actors if (path := _ref_path(actor)) is not None]
    actor_paths = sorted(path for path in all_actor_paths if "PCGPartitionGridActor_25600_" in path)
    if not actor_paths:
        raise McpToolError("no 256 m Forest_HF PCG World Partition actors were generated")

    allowed_meshes = {slot.asset_ref for slot in inputs.slots if slot.slot in REQUIRED_TREE_SLOTS}
    expected_by_mesh = {
        slot.asset_ref: inputs.slot_point_counts[slot.slot]
        for slot in inputs.slots
        if slot.slot in REQUIRED_TREE_SLOTS
    }
    by_mesh = {mesh: 0 for mesh in sorted(allowed_meshes)}
    component_count = 0
    total_instances = 0
    for actor_path in actor_paths:
        components = session.call(
            ACTOR_TOOLS,
            "get_components",
            {
                "actor": {"refPath": actor_path},
                "component_type": {"refPath": "/Script/Engine.HierarchicalInstancedStaticMeshComponent"},
            },
        )
        if isinstance(components, (str, bytes)) or not isinstance(components, Sequence):
            raise McpProtocolError(f"HISM component lookup failed for {actor_path}")
        for component in components:
            component_path = _ref_path(component)
            if component_path is None:
                raise McpProtocolError(f"partition actor {actor_path} returned an invalid HISM ref")
            properties = _json_mapping(
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
                f"HISM property readback for {component_path}",
            )
            mesh_ref = _ref_path(properties.get("staticMesh"))
            if mesh_ref not in allowed_meshes:
                raise McpToolError(f"generated HISM uses an unapproved mesh: {mesh_ref}")
            instances = properties.get("perInstanceSMData")
            if isinstance(instances, (str, bytes)) or not isinstance(instances, Sequence):
                raise McpProtocolError(f"HISM instance data are unavailable for {component_path}")
            body = properties.get("bodyInstance")
            if not isinstance(body, Mapping):
                raise McpProtocolError(f"HISM body instance is unavailable for {component_path}")
            collision_enabled = body.get("collisionEnabled")
            if (
                body.get("collisionProfileName") != "NoCollision"
                or collision_enabled not in ("NoCollision", 0)
                or body.get("bSimulatePhysics") is not False
                or properties.get("bGenerateOverlapEvents") is not False
                or properties.get("bCanEverAffectNavigation") is not False
            ):
                raise McpToolError(f"generated HISM violates MuJoCo-only NoCollision authority: {component_path}")
            tags = properties.get("componentTags")
            if isinstance(tags, (str, bytes)) or not isinstance(tags, Sequence):
                raise McpToolError(f"generated HISM is missing authority tags: {component_path}")
            if not {"VisualOnly", "NoCollision"}.issubset(set(tags)):
                raise McpToolError(f"generated HISM is missing VisualOnly/NoCollision tags: {component_path}")
            count = len(instances)
            by_mesh[mesh_ref] += count
            total_instances += count
            component_count += 1
    if total_instances != inputs.point_count:
        raise McpToolError(
            f"generated HISM instance total {total_instances} does not match bound point count {inputs.point_count}"
        )
    if by_mesh != expected_by_mesh:
        raise McpToolError(
            "generated HISM per-mesh instance counts do not match exact bake mesh slots: "
            f"observed={by_mesh}, expected={expected_by_mesh}"
        )
    return {
        "partition_actor_count": len(actor_paths),
        "hism_component_count": component_count,
        "instance_count": total_instances,
        "by_mesh": by_mesh,
        "expected_by_mesh": expected_by_mesh,
        "allowed_meshes": sorted(allowed_meshes),
        "collision_profile": "NoCollision",
        "physics_authority": "mujoco",
    }


def author_forest_hf(transport: McpTransport, inputs: AuthoringInputs) -> dict[str, object]:
    """Apply the deterministic Forest_HF map/graph/volume mutations through MCP."""

    transport.initialize()
    transport.notify_initialized()
    session = _AuthoringSession(transport)
    map_outcome = _ensure_map(session)
    mesh_outcomes = _ensure_meshes(session, inputs)
    graph_outcome, selector_paths = _ensure_graph(session, inputs)
    volume_outcome, volume_path = _ensure_volume(session)
    generated_audit = _audit_generated_hisms(session, inputs)
    save_paths = sorted([MAP_PATH, GRAPH_ASSET_PATH, *(slot.asset_path for slot in inputs.slots)])
    session.require_true(ASSET_TOOLS, "save_assets", {"asset_paths": save_paths})
    dirty_assets = [
        path for path in save_paths if session.call(ASSET_TOOLS, "is_dirty", {"asset_path": path}) is not False
    ]
    if dirty_assets:
        raise McpToolError(f"assets remain dirty after save: {dirty_assets}")
    if not session.exists(OPEN_WORLD_TEMPLATE):
        raise McpToolError("OpenWorld template is unavailable for forced disk-reload verification")
    _load_level(session, OPEN_WORLD_TEMPLATE)
    _load_map(session)
    reload_audit = _audit_generated_hisms(session, inputs)
    if reload_audit != generated_audit:
        raise McpToolError("World Partition HISM audit changed after map save/reload")
    return {
        "schema": "lingtu.sim.unreal-forest-mcp-authoring-result.v1",
        "world_package": WORLD_PACKAGE,
        "seed": FIXED_SEED,
        "map": {"path": MAP_PATH, "outcome": map_outcome},
        "graph": {
            "path": GRAPH_ASSET_PATH,
            "outcome": graph_outcome,
            "selector_refs": selector_paths,
            "spawner_slots": sorted(REQUIRED_TREE_SLOTS),
            "selection_policy": EXACT_MESH_SLOT_SELECTION_POLICY,
        },
        "volume": {"ref": volume_path, "outcome": volume_outcome},
        "meshes": mesh_outcomes,
        "import_only_slots": sorted(slot.slot for slot in inputs.slots if slot.slot not in REQUIRED_TREE_SLOTS),
        "point_count": inputs.point_count,
        "input_binding_sha256": inputs.binding_sha256,
        "generated_hism_audit": generated_audit,
        "reload_hism_audit": reload_audit,
        "tool_call_count": session.call_count,
        "saved_assets": save_paths,
        "authority": {
            "physics": "mujoco",
            "raycast": "mujoco",
            "unreal_collision": "NoCollision",
        },
    }


def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT_PATH)
    parser.add_argument("--create-points", type=Path, default=DEFAULT_POINTS_PATH)
    parser.add_argument(
        "--slot-fbx",
        action="append",
        default=[],
        metavar="SLOT=PATH",
        help=(
            "repeat for pine/birch and optionally boulder; short aliases and stable "
            "forest.asset.<slot> IDs are accepted"
        ),
    )
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    parser.add_argument("--timeout-seconds", type=float, default=30.0)
    parser.add_argument("--plan-output", type=Path)
    parser.add_argument("--apply", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Validate inputs, print a dry-run plan, and mutate UE only with ``--apply``."""

    args = _parse_args(argv)
    inputs = load_authoring_inputs(args.contract, args.create_points, args.slot_fbx)
    plan = build_authoring_plan(inputs)
    if args.plan_output is not None:
        output = args.plan_output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(canonical_json_bytes(plan))
        print(f"LINGTU_FOREST_HF_MCP_PLAN={output}")
    else:
        print(canonical_json_bytes(plan).decode("utf-8"), end="")
    if not args.apply:
        print("LINGTU_FOREST_HF_MCP_MODE=dry_run")
        return 0
    result = author_forest_hf(HttpMcpTransport(args.endpoint, timeout_seconds=args.timeout_seconds), inputs)
    print(canonical_json_bytes(result).decode("utf-8"), end="")
    print("LINGTU_FOREST_HF_MCP_MODE=applied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
