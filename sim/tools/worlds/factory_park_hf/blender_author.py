"""Author a reviewable Blender scene from the FactoryPark_HF layout.

The expanded layout is the authority for every collision-relevant object.  This
module adds presentation geometry (markings, facade detail, lamps, and planting)
only as explicitly tagged ``VisualOnly`` meshes.  The module intentionally
imports without Blender so its CLI and layout contracts can be tested with the
normal project Python interpreter.

Source coordinates are right-handed, Z-up, and measured in metres.  Blender
uses that frame unchanged.  Unreal import must apply ``(x, y, z) m ->
(100*x, -100*y, 100*z) cm`` and negate source yaw.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

try:  # Blender is deliberately optional for ordinary Python contract tests.
    import bpy  # type: ignore[import-not-found]
except ModuleNotFoundError:  # pragma: no cover - exercised by normal Python imports.
    bpy = None


BPY_AVAILABLE = bpy is not None
WORLD_PACKAGE = "factory_park_hf@1.0.0"
DEFAULT_LAYOUT = Path("sim/packages/worlds/factory_park_hf/generated/expanded-layout.json")
DEFAULT_REALISM_RECIPE = Path("sim/packages/worlds/factory_park_hf/visual/realism.recipe.json")
DEFAULT_OUTPUT_DIR = Path("build/factory-park-hf/blender-v2")
REALISM_PROFILE = "industrial_realism_v2"
DEFAULT_REALISM_SEED = 5208
DEFAULT_ACTOR_BUDGET = (1200, 1800)
_SAFE_NAME = re.compile(r"[^A-Za-z0-9_]+")
_COORDINATE_ALIASES = {
    "mujoco_rh_z_up_m",
    "rh_z_up_m",
    "right_handed_z_up_m",
    "right-handed-z-up-m",
}


@dataclass(frozen=True)
class LayoutObject:
    """Validated collision/visual declaration from the expanded layout."""

    stable_id: str
    semantic_class: str
    shape: str
    position_m: tuple[float, float, float]
    yaw_deg: float
    pitch_deg: float
    material: str
    collision: bool
    visual_only: bool
    dimensions_m: tuple[float, float, float]
    radius_m: float | None = None
    half_height_m: float | None = None

    @property
    def safe_id(self) -> str:
        """Return this object's Blender/FBX-safe stable identifier."""

        return safe_identifier(self.stable_id)

    @property
    def quaternion_wxyz(self) -> tuple[float, float, float, float]:
        """Return the MuJoCo-equivalent Rz(yaw) * Ry(pitch) quaternion."""

        half_yaw = math.radians(self.yaw_deg) / 2.0
        half_pitch = math.radians(self.pitch_deg) / 2.0
        cos_yaw = math.cos(half_yaw)
        sin_yaw = math.sin(half_yaw)
        cos_pitch = math.cos(half_pitch)
        sin_pitch = math.sin(half_pitch)
        # MuJoCo writes euler="0 pitch yaw".  This is Rz(yaw) * Ry(pitch).
        return (
            cos_yaw * cos_pitch,
            -sin_yaw * sin_pitch,
            cos_yaw * sin_pitch,
            sin_yaw * cos_pitch,
        )


@dataclass(frozen=True)
class RealismSettings:
    """Validated visual-only recipe inputs for deterministic v2 dressing."""

    path: Path
    profile: str
    seed: int
    actor_budget: tuple[int, int]
    digest: str
    source: str
    namespace: str
    dressing: tuple[Mapping[str, Any], ...]
    preview_targets: tuple[Mapping[str, Any], ...]
    raw: Mapping[str, Any]


@dataclass
class BuildState:
    """Mutable bookkeeping owned by one transient Blender build."""

    layout: Mapping[str, Any]
    layout_path: Path
    repo_root: Path
    collections: dict[str, Any]
    materials: dict[str, Any]
    realism: RealismSettings
    layout_records: list[dict[str, Any]] = field(default_factory=list)
    visual_records: list[dict[str, Any]] = field(default_factory=list)
    terrain_records: list[dict[str, Any]] = field(default_factory=list)
    terrain_feature_descriptors: list[dict[str, Any]] = field(default_factory=list)
    semantic_feature_descriptors: list[dict[str, Any]] = field(default_factory=list)
    allocated_names: set[str] = field(default_factory=set)
    detail_counts: Counter[str] = field(default_factory=Counter)


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


def stable_variation(
    stable_id: str,
    index: int,
    *,
    low: float = 0.0,
    high: float = 1.0,
    seed: int = DEFAULT_REALISM_SEED,
) -> float:
    """Map an identity to a deterministic bounded scalar without global RNG state."""

    if not math.isfinite(low) or not math.isfinite(high) or high < low:
        raise ValueError("stable variation bounds must be finite and ordered")
    identity = f"{seed}:{stable_id}:{index}".encode()
    unit = int.from_bytes(hashlib.sha256(identity).digest()[:8], "big") / float(2**64 - 1)
    return low + (high - low) * unit


def _budget_value(
    budget: Mapping[str, Any],
    candidates: Sequence[str],
    default: int,
) -> int:
    for key in candidates:
        value = budget.get(key)
        if isinstance(value, int) and not isinstance(value, bool):
            return value
    return default


def _fallback_realism_recipe() -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.factory-park-realism-recipe.v1",
        "profile": REALISM_PROFILE,
        "seed": DEFAULT_REALISM_SEED,
        "authoring_contract": {
            "dressing_id_namespace": "factory_park_hf.realism",
            "randomness": "seeded_only",
            "dressing_defaults": {
                "classification": "VisualOnly",
                "collision": False,
                "physics_representation": "none",
            },
        },
        "performance_budgets": {
            "actor_count_min": DEFAULT_ACTOR_BUDGET[0],
            "actor_count_max": DEFAULT_ACTOR_BUDGET[1],
        },
        "preview_targets": [],
    }


def load_realism_recipe(path: Path | str) -> RealismSettings:
    """Load the visual recipe, falling back safely when generation has not run yet."""

    recipe_path = Path(path).resolve()
    if recipe_path.is_file():
        payload = recipe_path.read_bytes()
        raw = json.loads(payload.decode("utf-8"))
        source = "file"
    else:
        raw = _fallback_realism_recipe()
        payload = _canonical_json(raw)
        source = "built_in_fallback"
    if not isinstance(raw, Mapping):
        raise TypeError("realism recipe root must be an object")
    if raw.get("schema") != "lingtu.sim.factory-park-realism-recipe.v1":
        raise ValueError("unsupported realism recipe schema")
    profile = str(raw.get("profile", ""))
    if profile != REALISM_PROFILE:
        raise ValueError(f"realism recipe profile must be {REALISM_PROFILE!r}")
    seed = raw.get("seed", DEFAULT_REALISM_SEED)
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise TypeError("realism recipe seed must be an integer")

    authoring = raw.get("authoring_contract", {})
    if authoring is not None and not isinstance(authoring, Mapping):
        raise TypeError("realism recipe authoring_contract must be an object")
    namespace = str((authoring or {}).get("dressing_id_namespace", "factory_park_hf.realism")).strip()
    if not namespace:
        raise ValueError("realism recipe dressing_id_namespace cannot be empty")

    budget = raw.get("performance_budgets", raw.get("actor_budget", {}))
    if not isinstance(budget, Mapping):
        raise TypeError("realism recipe performance budget must be an object")
    actor_min = _budget_value(
        budget,
        ("actor_count_min", "min_actor_count", "min_actors", "min"),
        DEFAULT_ACTOR_BUDGET[0],
    )
    actor_max = _budget_value(
        budget,
        ("actor_count_max", "max_actor_count", "max_actors", "max"),
        DEFAULT_ACTOR_BUDGET[1],
    )
    if actor_min < 1 or actor_max < actor_min:
        raise ValueError("realism recipe actor budget must be positive and ordered")

    raw_dressing = raw.get("dressing", raw.get("dressing_instances", []))
    if not isinstance(raw_dressing, list):
        raise TypeError("realism recipe dressing must be an array")
    dressing: list[Mapping[str, Any]] = []
    stable_ids: set[str] = set()
    for index, item in enumerate(raw_dressing):
        if not isinstance(item, Mapping):
            raise TypeError(f"realism recipe dressing[{index}] must be an object")
        stable_id = str(item.get("stable_id", item.get("dressing_id", ""))).strip()
        derived_from = str(item.get("derived_from", "")).strip()
        collision = item.get("collision", False)
        visual_only = item.get("visual_only", True)
        if collision is not False or visual_only is not True:
            raise ValueError("realism recipe dressing must be VisualOnly and collision-free")
        if not stable_id:
            raise ValueError("realism recipe dressing stable_id is required")
        if not derived_from:
            raise ValueError("realism recipe dressing derived_from is required")
        if stable_id in stable_ids:
            raise ValueError(f"duplicate realism dressing stable_id: {stable_id}")
        stable_ids.add(stable_id)
        dressing.append(item)

    preview_targets = raw.get("preview_targets", [])
    if not isinstance(preview_targets, list) or not all(isinstance(item, Mapping) for item in preview_targets):
        raise TypeError("realism recipe preview_targets must be an array of objects")
    return RealismSettings(
        path=recipe_path,
        profile=profile,
        seed=seed,
        actor_budget=(actor_min, actor_max),
        digest=_sha256(payload),
        source=source,
        namespace=namespace,
        dressing=tuple(dressing),
        preview_targets=tuple(preview_targets),
        raw=raw,
    )


def safe_identifier(value: str) -> str:
    """Return the stable Blender/FBX identifier for a layout ID."""

    normalized = _SAFE_NAME.sub("_", value.strip()).strip("_")
    if not normalized:
        raise ValueError(f"stable id {value!r} has no export-safe characters")
    if normalized[0].isdigit():
        normalized = f"id_{normalized}"
    return normalized[:96]


def _finite_number(value: object, *, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{field_name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field_name} must be finite")
    return result


def _vector(
    value: object,
    *,
    field_name: str,
    lengths: tuple[int, ...] = (3,),
) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise TypeError(f"{field_name} must be a numeric array")
    if len(value) not in lengths:
        expected = " or ".join(str(length) for length in lengths)
        raise ValueError(f"{field_name} must contain {expected} values")
    return tuple(_finite_number(item, field_name=f"{field_name}[{index}]") for index, item in enumerate(value))


def _coordinate_contract_is_supported(value: object) -> bool:
    if isinstance(value, str):
        normalized = value.strip().lower().replace(" ", "_")
        return normalized in _COORDINATE_ALIASES
    if not isinstance(value, Mapping):
        return False
    frame = str(value.get("frame", "")).strip().lower().replace(" ", "_")
    if frame in _COORDINATE_ALIASES:
        return True
    handedness = str(value.get("handedness", value.get("hand", ""))).lower()
    up_axis = str(value.get("up_axis", value.get("up", ""))).lower().lstrip("+")
    units = str(value.get("units", value.get("length_unit", value.get("linear_unit", "")))).lower()
    return (
        handedness in {"right", "right_handed", "rh"}
        and up_axis == "z"
        and units
        in {
            "m",
            "meter",
            "meters",
            "metre",
            "metres",
        }
    )


def validate_layout(layout: Mapping[str, Any]) -> tuple[LayoutObject, ...]:
    """Validate and normalize the expanded layout without importing Blender."""

    if not isinstance(layout, Mapping):
        raise TypeError("expanded layout root must be an object")
    if not _coordinate_contract_is_supported(layout.get("coordinate_system")):
        raise ValueError("expanded layout must declare right-handed Z-up metres")
    raw_objects = layout.get("objects")
    if isinstance(raw_objects, (str, bytes)) or not isinstance(raw_objects, Sequence):
        raise TypeError("expanded layout objects must be an array")

    normalized: list[LayoutObject] = []
    stable_ids: set[str] = set()
    export_names: set[str] = set()
    for index, raw in enumerate(raw_objects):
        if not isinstance(raw, Mapping):
            raise TypeError(f"objects[{index}] must be an object")
        stable_id = str(raw.get("id", "")).strip()
        semantic_class = str(raw.get("semantic_class", "")).strip()
        material = str(raw.get("material", "")).strip()
        shape = str(raw.get("shape", "")).strip().lower()
        if not stable_id or not semantic_class or not material:
            raise ValueError(f"objects[{index}] requires id, semantic_class, and material")
        if stable_id in stable_ids:
            raise ValueError(f"duplicate stable object id: {stable_id}")
        safe_id = safe_identifier(stable_id)
        if safe_id in export_names:
            raise ValueError(f"stable IDs collide after Blender-safe normalization: {stable_id}")
        stable_ids.add(stable_id)
        export_names.add(safe_id)

        position = _vector(raw.get("position_m"), field_name=f"{stable_id}.position_m")
        yaw_deg = _finite_number(raw.get("yaw_deg"), field_name=f"{stable_id}.yaw_deg")
        pitch_deg = _finite_number(raw.get("pitch_deg", 0.0), field_name=f"{stable_id}.pitch_deg")
        collision = raw.get("collision")
        visual_only = raw.get("visual_only")
        if not isinstance(collision, bool) or not isinstance(visual_only, bool):
            raise TypeError(f"{stable_id} collision and visual_only must be booleans")
        if collision and visual_only:
            raise ValueError(f"{stable_id} cannot be both collision and VisualOnly")

        radius: float | None = None
        half_height: float | None = None
        if shape in {"box", "plane"}:
            size = _vector(raw.get("size_m"), field_name=f"{stable_id}.size_m", lengths=(2, 3))
            if any(component <= 0.0 for component in size):
                raise ValueError(f"{stable_id}.size_m must be positive")
            dimensions = (
                size[0],
                size[1],
                size[2] if len(size) == 3 else (0.025 if shape == "plane" else 1.0),
            )
        elif shape == "cylinder":
            radius = _finite_number(raw.get("radius_m"), field_name=f"{stable_id}.radius_m")
            half_height = _finite_number(
                raw.get("half_height_m"),
                field_name=f"{stable_id}.half_height_m",
            )
            if radius <= 0.0 or half_height <= 0.0:
                raise ValueError(f"{stable_id} cylinder dimensions must be positive")
            dimensions = (2.0 * radius, 2.0 * radius, 2.0 * half_height)
        else:
            raise ValueError(f"{stable_id} uses unsupported shape {shape!r}")

        normalized.append(
            LayoutObject(
                stable_id=stable_id,
                semantic_class=semantic_class,
                shape=shape,
                position_m=(position[0], position[1], position[2]),
                yaw_deg=yaw_deg,
                pitch_deg=pitch_deg,
                material=material,
                collision=collision,
                visual_only=visual_only,
                dimensions_m=dimensions,
                radius_m=radius,
                half_height_m=half_height,
            )
        )
    return tuple(normalized)


def load_layout(path: Path | str) -> tuple[dict[str, Any], tuple[LayoutObject, ...]]:
    """Load an expanded layout and return it with normalized objects."""

    layout_path = Path(path).resolve()
    layout = json.loads(layout_path.read_text(encoding="utf-8"))
    objects = validate_layout(layout)
    return layout, objects


def compute_layout_bounds(objects: Sequence[LayoutObject]) -> dict[str, list[float]]:
    """Compute a yaw-aware axis-aligned bound in the source metre frame."""

    if not objects:
        raise ValueError("expanded layout contains no objects")
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    for item in objects:
        sx, sy, sz = item.dimensions_m
        yaw = math.radians(item.yaw_deg)
        pitch = math.radians(item.pitch_deg)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
        local_half = (sx / 2.0, sy / 2.0, sz / 2.0)
        # Absolute rows of Rz(yaw) * Ry(pitch), multiplied by local half extents.
        half = (
            abs(cos_yaw * cos_pitch) * local_half[0]
            + abs(sin_yaw) * local_half[1]
            + abs(cos_yaw * sin_pitch) * local_half[2],
            abs(sin_yaw * cos_pitch) * local_half[0]
            + abs(cos_yaw) * local_half[1]
            + abs(sin_yaw * sin_pitch) * local_half[2],
            abs(sin_pitch) * local_half[0] + abs(cos_pitch) * local_half[2],
        )
        for axis in range(3):
            minimum[axis] = min(minimum[axis], item.position_m[axis] - half[axis])
            maximum[axis] = max(maximum[axis], item.position_m[axis] + half[axis])
    return {"min": minimum, "max": maximum}


def coordinate_contract() -> dict[str, Any]:
    """Return the explicit source, exchange, and Unreal coordinate contract."""

    return {
        "source": {
            "frame": "mujoco_rh_z_up_m",
            "handedness": "right",
            "up_axis": "+Z",
            "forward_axis": "+X",
            "units": "m",
        },
        "blender": {
            "frame": "blender_rh_z_up_m",
            "transform_from_source": "identity",
            "unit_system": "METRIC",
            "unit_scale": 1.0,
        },
        "terrain_obj_to_blender": {
            "source_frame": "unreal_lh_z_up_cm",
            "target_frame": "blender_rh_z_up_m",
            "vertex_mapping": "(x_cm,y_cm,z_cm) -> (0.01*x_cm,-0.01*y_cm,0.01*z_cm)",
            "reverse_face_winding": True,
        },
        "unreal": {
            "frame": "unreal_lh_z_up_cm",
            "position_mapping": "(x,y,z)m -> (100*x,-100*y,100*z)cm",
            "direction_mapping": "(x,y,z) -> (x,-y,z)",
            "yaw_mapping": "unreal_yaw_deg = -source_yaw_deg",
            "import_scale": 1.0,
            "import_uniform_scale": 1.0,
            "import_uniform_scale_supported": False,
            "unit_conversion_strategy": "placement_actor_scale",
            "fbx_actor_uniform_scale": 100.0,
            "terrain_actor_uniform_scale": 1.0,
            "unit_conversion": (
                "UE 5.8 imports FBX local metre values as numeric Unreal units; "
                "apply uniform actor scale 100 at placement. Terrain OBJ is already centimetres."
            ),
            "combine_meshes": False,
        },
        "fbx": {
            "authored_units": "m",
            "axis_forward": "-Y",
            "axis_up": "+Z",
            "global_scale": 1.0,
            "apply_unit_scale": True,
            "bake_space_transform": False,
            "contains_authoritative_terrain": False,
            "mesh_vertices": "object-local",
            "node_transforms": "preserved",
            "unreal_transform_vertex_to_absolute": False,
            "unreal_bake_pivot_in_vertex": False,
            "unreal_import_uniform_scale": 1.0,
            "unreal_import_uniform_scale_supported": False,
            "unreal_actor_uniform_scale": 100.0,
            "unreal_terrain_actor_uniform_scale": 1.0,
        },
        "glb": {
            "standard_frame": "gltf_rh_y_up_m",
            "export_yup": True,
            "note": "Blender exporter performs the Z-up to glTF Y-up conversion.",
        },
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build the editable FactoryPark_HF Blender scene and review artifacts.",
    )
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--layout", type=Path, default=DEFAULT_LAYOUT)
    parser.add_argument("--realism-recipe", type=Path, default=DEFAULT_REALISM_RECIPE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--export-format",
        "--format",
        choices=("fbx", "glb", "both"),
        default="both",
        dest="export_format",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--samples", type=int, default=32)
    parser.add_argument("--validate-only", action="store_true")
    return parser


def parse_cli_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse either ordinary argv or Blender's arguments following ``--``."""

    arguments = list(sys.argv[1:] if argv is None else argv)
    if "--" in arguments:
        arguments = arguments[arguments.index("--") + 1 :]
    args = _build_parser().parse_args(arguments)
    if args.width < 320 or args.height < 240:
        raise ValueError("preview resolution must be at least 320x240")
    if args.samples < 1:
        raise ValueError("render samples must be positive")
    args.repo_root = args.repo_root.resolve()
    if not args.layout.is_absolute():
        args.layout = (args.repo_root / args.layout).resolve()
    else:
        args.layout = args.layout.resolve()
    if not args.realism_recipe.is_absolute():
        args.realism_recipe = (args.repo_root / args.realism_recipe).resolve()
    else:
        args.realism_recipe = args.realism_recipe.resolve()
    if not args.output_dir.is_absolute():
        args.output_dir = (args.repo_root / args.output_dir).resolve()
    else:
        args.output_dir = args.output_dir.resolve()
    return args


def _require_bpy() -> Any:
    if bpy is None:
        raise RuntimeError(
            "Blender bpy is unavailable; run with blender --background --python blender_author.py -- <arguments>",
        )
    return bpy


def _new_collection(name: str, parent: Any) -> Any:
    blender = _require_bpy()
    collection = blender.data.collections.new(name)
    parent.children.link(collection)
    return collection


def _setup_scene(
    layout: Mapping[str, Any],
    layout_path: Path,
    repo_root: Path,
    realism: RealismSettings,
) -> BuildState:
    blender = _require_bpy()
    blender.ops.wm.read_factory_settings(use_empty=True)
    scene = blender.context.scene
    scene.name = "FactoryPark_HF_Authoring"
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.unit_settings.length_unit = "METERS"
    scene.render.engine = "BLENDER_EEVEE"

    root = _new_collection("FactoryPark_HF", scene.collection)
    layout_root = _new_collection("00_LAYOUT_AUTHORITY", root)
    visual_root = _new_collection("10_VISUAL_ONLY", root)
    preview_root = _new_collection("90_PREVIEW", root)
    lighting_root = _new_collection("91_LIGHTING", root)
    collections = {
        "root": root,
        "layout_terrain": _new_collection("Terrain", layout_root),
        "layout_transport": _new_collection("Transport", layout_root),
        "layout_buildings": _new_collection("Buildings", layout_root),
        "layout_process": _new_collection("Process", layout_root),
        "layout_logistics": _new_collection("Logistics", layout_root),
        "layout_perimeter": _new_collection("Perimeter", layout_root),
        "layout_safety": _new_collection("Safety", layout_root),
        "visual_architecture": _new_collection("Architecture_Detail", visual_root),
        "visual_markings": _new_collection("Roads_Parking_Markings", visual_root),
        "visual_process": _new_collection("Process_Detail", visual_root),
        "visual_furniture": _new_collection("Street_Furniture", visual_root),
        "visual_landscape": _new_collection("Landscape", visual_root),
        "visual_realism": _new_collection("Industrial_Realism_V2", visual_root),
        "visual_vehicles": _new_collection("Vehicles_And_Clutter", visual_root),
        "visual_utilities": _new_collection("Utilities", visual_root),
        "preview": preview_root,
        "lighting": lighting_root,
    }
    state = BuildState(
        layout=layout,
        layout_path=layout_path,
        repo_root=repo_root,
        collections=collections,
        materials={},
        realism=realism,
    )
    state.materials = _create_material_library()
    return state


_MATERIAL_PALETTE: dict[str, tuple[tuple[float, float, float, float], float, float]] = {
    "asphalt": ((0.035, 0.045, 0.052, 1.0), 0.0, 0.92),
    "concrete": ((0.34, 0.37, 0.39, 1.0), 0.0, 0.82),
    "steel": ((0.18, 0.21, 0.24, 1.0), 0.78, 0.32),
    "painted_steel": ((0.055, 0.18, 0.31, 1.0), 0.36, 0.3),
    "gravel": ((0.24, 0.22, 0.18, 1.0), 0.0, 0.98),
    "rubber": ((0.018, 0.02, 0.022, 1.0), 0.0, 0.98),
    "glass": ((0.08, 0.24, 0.32, 0.38), 0.08, 0.1),
    "waterproof_concrete": ((0.29, 0.32, 0.31, 1.0), 0.0, 0.88),
    "terrain": ((0.16, 0.22, 0.105, 1.0), 0.0, 0.96),
    "roof_metal": ((0.16, 0.2, 0.22, 1.0), 0.72, 0.28),
    "window_glass": ((0.025, 0.16, 0.25, 0.72), 0.15, 0.12),
    "line_white": ((0.82, 0.84, 0.78, 1.0), 0.0, 0.48),
    "safety_yellow": ((0.95, 0.54, 0.025, 1.0), 0.08, 0.38),
    "safety_red": ((0.62, 0.025, 0.018, 1.0), 0.05, 0.4),
    "container_blue": ((0.025, 0.16, 0.36, 1.0), 0.34, 0.34),
    "container_red": ((0.43, 0.055, 0.035, 1.0), 0.34, 0.38),
    "container_orange": ((0.78, 0.22, 0.025, 1.0), 0.32, 0.4),
    "factory_cladding": ((0.29, 0.42, 0.52, 1.0), 0.42, 0.34),
    "warehouse_cladding": ((0.38, 0.43, 0.45, 1.0), 0.4, 0.38),
    "guardhouse": ((0.68, 0.69, 0.64, 1.0), 0.05, 0.62),
    "tank_white": ((0.7, 0.72, 0.68, 1.0), 0.45, 0.3),
    "fence_steel": ((0.13, 0.17, 0.18, 1.0), 0.72, 0.38),
    "checkpoint_green": ((0.02, 0.72, 0.23, 0.74), 0.02, 0.32),
    "vegetation_leaf": ((0.055, 0.22, 0.065, 1.0), 0.0, 0.88),
    "vegetation_trunk": ((0.19, 0.095, 0.035, 1.0), 0.0, 0.92),
    "grass": ((0.11, 0.28, 0.055, 1.0), 0.0, 0.98),
    "lamp_emissive": ((1.0, 0.55, 0.12, 1.0), 0.0, 0.25),
    "asphalt_patch": ((0.018, 0.022, 0.026, 1.0), 0.0, 0.97),
    "tire_mark": ((0.006, 0.007, 0.008, 0.78), 0.0, 0.94),
    "crack_fill": ((0.012, 0.014, 0.015, 1.0), 0.0, 0.99),
    "concrete_dirty": ((0.23, 0.235, 0.225, 1.0), 0.0, 0.94),
    "concrete_wet": ((0.075, 0.09, 0.085, 0.9), 0.0, 0.24),
    "galvanized": ((0.34, 0.38, 0.4, 1.0), 0.72, 0.29),
    "rollup_door": ((0.16, 0.19, 0.205, 1.0), 0.58, 0.31),
    "dock_shelter": ((0.016, 0.018, 0.02, 1.0), 0.0, 0.96),
    "rust": ((0.31, 0.075, 0.018, 1.0), 0.08, 0.92),
    "dirt": ((0.18, 0.115, 0.052, 1.0), 0.0, 0.98),
    "safety_black": ((0.015, 0.016, 0.018, 1.0), 0.05, 0.62),
    "pallet_wood": ((0.31, 0.16, 0.055, 1.0), 0.0, 0.83),
    "crate_tan": ((0.42, 0.29, 0.12, 1.0), 0.0, 0.79),
    "barrel_blue": ((0.025, 0.13, 0.27, 1.0), 0.42, 0.31),
    "vehicle_white": ((0.62, 0.64, 0.61, 1.0), 0.36, 0.27),
    "vehicle_blue": ((0.025, 0.09, 0.22, 1.0), 0.43, 0.25),
    "traffic_orange": ((0.95, 0.18, 0.018, 1.0), 0.0, 0.47),
    "foliage_dry": ((0.31, 0.28, 0.065, 1.0), 0.0, 0.96),
    "soil": ((0.13, 0.075, 0.028, 1.0), 0.0, 0.99),
    "rock": ((0.19, 0.17, 0.14, 1.0), 0.0, 0.98),
    "roof_white": ((0.57, 0.59, 0.57, 1.0), 0.62, 0.28),
    "transformer_dark": ((0.075, 0.095, 0.09, 1.0), 0.69, 0.35),
    "ceramic": ((0.31, 0.24, 0.17, 1.0), 0.0, 0.23),
}

_MATERIAL_ALIASES = {
    "bund_concrete": "waterproof_concrete",
    "curb_concrete": "concrete",
    "drainage_concrete": "waterproof_concrete",
    "lamp": "lamp_emissive",
    "parking_marking": "asphalt",
    "road_marking": "line_white",
    "painted_safety_steel": "safety_yellow",
    "warehouse_galvanized": "warehouse_cladding",
    "factory_cladding_blue": "factory_cladding",
    "tank_enamel_white": "tank_white",
    "industrial_glass": "window_glass",
}


def _scaled_color(color: Sequence[float], factor: float) -> tuple[float, float, float, float]:
    return (
        min(1.0, max(0.0, color[0] * factor)),
        min(1.0, max(0.0, color[1] * factor)),
        min(1.0, max(0.0, color[2] * factor)),
        float(color[3]),
    )


def _create_material_library() -> dict[str, Any]:
    blender = _require_bpy()
    materials: dict[str, Any] = {}
    translucent = {"glass", "window_glass", "checkpoint_green", "tire_mark", "concrete_wet"}
    corrugated = {
        "factory_cladding",
        "warehouse_cladding",
        "roof_metal",
        "galvanized",
        "rollup_door",
    }
    rugged = {
        "asphalt",
        "asphalt_patch",
        "concrete",
        "concrete_dirty",
        "gravel",
        "terrain",
        "waterproof_concrete",
        "soil",
        "rock",
        "dirt",
        "rust",
        "pallet_wood",
        "crate_tan",
    }
    for key, (base_color, metallic, roughness) in _MATERIAL_PALETTE.items():
        material = blender.data.materials.new(name=f"MAT_{safe_identifier(key)}")
        material.use_nodes = True
        material.diffuse_color = base_color
        material["lingtu_profile"] = "procedural_pbr_v2"
        material["lingtu_material_key"] = key
        nodes = material.node_tree.nodes
        links = material.node_tree.links
        principled = next(node for node in nodes if node.type == "BSDF_PRINCIPLED")
        principled.inputs["Base Color"].default_value = base_color
        principled.inputs["Metallic"].default_value = metallic
        principled.inputs["Roughness"].default_value = roughness
        texture_coordinate = nodes.new("ShaderNodeTexCoord")
        texture_coordinate.name = f"TEXCOORD_{safe_identifier(key)}"
        if key in translucent:
            principled.inputs["Alpha"].default_value = base_color[3]
            if principled.inputs.get("Transmission Weight") is not None:
                principled.inputs["Transmission Weight"].default_value = 0.35
            material.surface_render_method = "DITHERED"
        if key == "lamp_emissive":
            emission = principled.inputs.get("Emission Color")
            strength = principled.inputs.get("Emission Strength")
            if emission is not None:
                emission.default_value = base_color
            if strength is not None:
                strength.default_value = 5.0
        if key in corrugated:
            wave = nodes.new("ShaderNodeTexWave")
            wave.wave_type = "BANDS"
            wave.bands_direction = "X"
            wave.inputs["Scale"].default_value = 7.5 if key == "rollup_door" else 11.0
            wave.inputs["Distortion"].default_value = 0.35
            wave.inputs["Detail"].default_value = 2.0
            ramp = nodes.new("ShaderNodeValToRGB")
            ramp.color_ramp.elements[0].position = 0.28
            ramp.color_ramp.elements[0].color = _scaled_color(base_color, 0.68)
            ramp.color_ramp.elements[1].position = 0.72
            ramp.color_ramp.elements[1].color = _scaled_color(base_color, 1.18)
            bump = nodes.new("ShaderNodeBump")
            bump.inputs["Strength"].default_value = 0.24
            bump.inputs["Distance"].default_value = 0.035
            links.new(texture_coordinate.outputs["Generated"], wave.inputs["Vector"])
            links.new(wave.outputs["Color"], ramp.inputs["Fac"])
            links.new(ramp.outputs["Color"], principled.inputs["Base Color"])
            links.new(wave.outputs["Color"], bump.inputs["Height"])
            links.new(bump.outputs["Normal"], principled.inputs["Normal"])
        elif key in rugged:
            texture = nodes.new("ShaderNodeTexNoise")
            texture.inputs["Scale"].default_value = 5.0 if key == "terrain" else 24.0
            texture.inputs["Detail"].default_value = 4.0
            texture.inputs["Roughness"].default_value = 0.72
            ramp = nodes.new("ShaderNodeValToRGB")
            ramp.color_ramp.elements[0].position = 0.2
            ramp.color_ramp.elements[0].color = _scaled_color(base_color, 0.65)
            ramp.color_ramp.elements[1].position = 0.82
            ramp.color_ramp.elements[1].color = _scaled_color(base_color, 1.3)
            bump = nodes.new("ShaderNodeBump")
            bump.inputs["Strength"].default_value = 0.12 if key != "gravel" else 0.28
            bump.inputs["Distance"].default_value = 0.08
            links.new(texture_coordinate.outputs["Generated"], texture.inputs["Vector"])
            links.new(texture.outputs["Fac"], ramp.inputs["Fac"])
            links.new(ramp.outputs["Color"], principled.inputs["Base Color"])
            links.new(texture.outputs["Fac"], bump.inputs["Height"])
            links.new(bump.outputs["Normal"], principled.inputs["Normal"])
        elif key not in {"lamp_emissive", "glass", "window_glass", "checkpoint_green"}:
            texture = nodes.new("ShaderNodeTexNoise")
            texture.inputs["Scale"].default_value = 8.0
            texture.inputs["Detail"].default_value = 2.0
            texture.inputs["Roughness"].default_value = 0.62
            ramp = nodes.new("ShaderNodeValToRGB")
            ramp.color_ramp.elements[0].color = _scaled_color(base_color, 0.82)
            ramp.color_ramp.elements[1].color = _scaled_color(base_color, 1.12)
            links.new(texture_coordinate.outputs["Generated"], texture.inputs["Vector"])
            links.new(texture.outputs["Fac"], ramp.inputs["Fac"])
            links.new(ramp.outputs["Color"], principled.inputs["Base Color"])
        if key not in {"lamp_emissive", "checkpoint_green"}:
            base_input = principled.inputs["Base Color"]
            previous_source = base_input.links[0].from_socket if base_input.links else None
            ambient_occlusion = nodes.new("ShaderNodeAmbientOcclusion")
            ambient_occlusion.name = f"AO_{safe_identifier(key)}"
            if hasattr(ambient_occlusion, "distance"):
                ambient_occlusion.distance = 3.2
            if hasattr(ambient_occlusion, "samples"):
                ambient_occlusion.samples = 12
            multiply = nodes.new("ShaderNodeMixRGB")
            multiply.blend_type = "MULTIPLY"
            multiply.inputs["Fac"].default_value = 0.38
            if previous_source is not None:
                links.new(previous_source, multiply.inputs[1])
            else:
                multiply.inputs[1].default_value = base_color
            links.new(ambient_occlusion.outputs["Color"], multiply.inputs[2])
            links.new(multiply.outputs["Color"], base_input)
        materials[key] = material
    return materials


def _material_for(state: BuildState, key: str) -> Any:
    palette_key = _MATERIAL_ALIASES.get(key, key)
    if palette_key in state.materials:
        return state.materials[palette_key]
    state.materials[palette_key] = state.materials["concrete"].copy()
    state.materials[palette_key].name = f"MAT_{safe_identifier(palette_key)}"
    return state.materials[palette_key]


def _box_geometry(dimensions: Sequence[float]) -> tuple[list[tuple[float, float, float]], list[tuple[int, ...]]]:
    hx, hy, hz = (component / 2.0 for component in dimensions)
    vertices = [
        (-hx, -hy, -hz),
        (hx, -hy, -hz),
        (hx, hy, -hz),
        (-hx, hy, -hz),
        (-hx, -hy, hz),
        (hx, -hy, hz),
        (hx, hy, hz),
        (-hx, hy, hz),
    ]
    faces = [
        (0, 3, 2, 1),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    ]
    return vertices, faces


def _cylinder_geometry(
    radius_m: float,
    height_m: float,
    *,
    sides: int = 24,
    top_radius_m: float | None = None,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, ...]], int]:
    top_radius = radius_m if top_radius_m is None else top_radius_m
    half_height = height_m / 2.0
    vertices: list[tuple[float, float, float]] = []
    for z, radius in ((-half_height, radius_m), (half_height, top_radius)):
        for index in range(sides):
            angle = 2.0 * math.pi * index / sides
            vertices.append((radius * math.cos(angle), radius * math.sin(angle), z))
    faces: list[tuple[int, ...]] = []
    for index in range(sides):
        next_index = (index + 1) % sides
        faces.append((index, next_index, sides + next_index, sides + index))
    faces.append(tuple(reversed(range(sides))))
    faces.append(tuple(range(sides, 2 * sides)))
    return vertices, faces, sides


def _ellipsoid_geometry(
    radii: Sequence[float],
    *,
    segments: int = 20,
    rings: int = 10,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, ...]]]:
    rx, ry, rz = radii
    vertices: list[tuple[float, float, float]] = []
    for ring in range(rings + 1):
        latitude = math.pi * ring / rings
        for segment in range(segments):
            longitude = 2.0 * math.pi * segment / segments
            vertices.append(
                (
                    rx * math.sin(latitude) * math.cos(longitude),
                    ry * math.sin(latitude) * math.sin(longitude),
                    rz * math.cos(latitude),
                )
            )
    faces: list[tuple[int, ...]] = []
    for ring in range(rings):
        for segment in range(segments):
            next_segment = (segment + 1) % segments
            a = ring * segments + segment
            b = ring * segments + next_segment
            c = (ring + 1) * segments + next_segment
            d = (ring + 1) * segments + segment
            faces.append((a, b, c, d))
    return vertices, faces


def _unique_object_name(state: BuildState, stable_id: str) -> str:
    name = f"LT_{safe_identifier(stable_id)}"
    if name in state.allocated_names:
        raise ValueError(f"duplicate Blender export object name: {name}")
    state.allocated_names.add(name)
    return name


def _create_mesh_object(
    state: BuildState,
    *,
    stable_id: str,
    semantic_class: str,
    shape: str,
    position_m: Sequence[float],
    yaw_deg: float,
    dimensions_m: Sequence[float],
    material_key: str,
    collision: bool,
    visual_only: bool,
    source: str,
    collection: Any,
    vertices: Sequence[Sequence[float]],
    faces: Sequence[Sequence[int]],
    smooth_face_count: int = 0,
    derived_from: str | None = None,
    pitch_deg: float = 0.0,
) -> tuple[Any, dict[str, Any]]:
    blender = _require_bpy()
    if source == "blender_derived_visual" and (collision or not visual_only or not derived_from):
        raise ValueError("Blender-derived dressing must be traceable VisualOnly collision-free geometry")
    name = _unique_object_name(state, stable_id)
    mesh = blender.data.meshes.new(name=name)
    mesh.from_pydata(vertices, [], faces)
    mesh.update(calc_edges=True)
    for polygon in mesh.polygons[:smooth_face_count]:
        polygon.use_smooth = True
    obj = blender.data.objects.new(name=name, object_data=mesh)
    collection.objects.link(obj)
    obj.location = tuple(position_m)
    obj.rotation_euler = (0.0, math.radians(pitch_deg), math.radians(yaw_deg))
    obj.scale = (1.0, 1.0, 1.0)
    obj.data.materials.append(_material_for(state, material_key))
    obj["lingtu_stable_id"] = stable_id
    obj["lingtu_semantic_class"] = semantic_class
    obj["lingtu_material"] = material_key
    obj["lingtu_source"] = source
    obj["lingtu_collision"] = collision
    obj["lingtu_visual_only"] = visual_only
    obj["lingtu_physics_proxy"] = "authoritative_layout" if collision else "none"
    obj["lingtu_source_frame"] = "mujoco_rh_z_up_m"
    obj["lingtu_unreal_axis_mapping"] = "x,-y,z"
    obj["lingtu_pitch_deg"] = float(pitch_deg)
    if derived_from is not None:
        obj["lingtu_derived_from"] = derived_from

    half_yaw = math.radians(yaw_deg) / 2.0
    half_pitch = math.radians(pitch_deg) / 2.0
    cos_yaw = math.cos(half_yaw)
    sin_yaw = math.sin(half_yaw)
    cos_pitch = math.cos(half_pitch)
    sin_pitch = math.sin(half_pitch)
    record: dict[str, Any] = {
        "stable_id": stable_id,
        "asset_key": f"layout/{stable_id}" if source == "expanded_layout" else f"visual/{stable_id}",
        "semantic_class": semantic_class,
        "blender_object": name,
        "mesh_name": name,
        "shape": shape,
        "position_m": [float(value) for value in position_m],
        "yaw_deg": float(yaw_deg),
        "pitch_deg": float(pitch_deg),
        "quaternion_wxyz": [
            cos_yaw * cos_pitch,
            -sin_yaw * sin_pitch,
            cos_yaw * sin_pitch,
            sin_yaw * cos_pitch,
        ],
        "scale": [1.0, 1.0, 1.0],
        "dimensions_m": [float(value) for value in dimensions_m],
        "material": material_key,
        "collision": collision,
        "visual_only": visual_only,
        "physics_proxy": "authoritative_layout" if collision else "none",
        "source": source,
    }
    if derived_from is not None:
        record["derived_from"] = derived_from
    if source == "blender_derived_visual":
        state.detail_counts[semantic_class] += 1
    return obj, record


def _create_box(
    state: BuildState,
    **kwargs: Any,
) -> tuple[Any, dict[str, Any]]:
    dimensions = kwargs["dimensions_m"]
    vertices, faces = _box_geometry(dimensions)
    return _create_mesh_object(state, vertices=vertices, faces=faces, **kwargs)


def _create_cylinder(
    state: BuildState,
    *,
    radius_m: float,
    height_m: float,
    top_radius_m: float | None = None,
    **kwargs: Any,
) -> tuple[Any, dict[str, Any]]:
    vertices, faces, side_faces = _cylinder_geometry(
        radius_m,
        height_m,
        top_radius_m=top_radius_m,
    )
    return _create_mesh_object(
        state,
        vertices=vertices,
        faces=faces,
        smooth_face_count=side_faces,
        **kwargs,
    )


def _create_ellipsoid(
    state: BuildState,
    *,
    radii_m: Sequence[float],
    **kwargs: Any,
) -> tuple[Any, dict[str, Any]]:
    vertices, faces = _ellipsoid_geometry(radii_m)
    return _create_mesh_object(
        state,
        vertices=vertices,
        faces=faces,
        smooth_face_count=len(faces),
        **kwargs,
    )


def _layout_collection(state: BuildState, semantic_class: str) -> Any:
    if semantic_class in {
        "road",
        "curb",
        "loading_dock",
        "ramp",
        "parking_space",
        "drainage_ditch",
        "speed_bump",
        "gravel_test_area",
        "parking_area",
        "container_yard",
        "parking_stop",
        "drainage_grate",
    }:
        return state.collections["layout_transport"]
    if semantic_class in {
        "building",
        "guardhouse",
        "main_factory_building",
        "warehouse_building",
        "office_building",
        "roof_structure",
    }:
        return state.collections["layout_buildings"]
    if semantic_class in {"storage_tank", "containment_bund", "pipe_rack"}:
        return state.collections["layout_process"]
    if semantic_class in {"container", "shipping_container"}:
        return state.collections["layout_logistics"]
    if semantic_class in {"boundary_fence", "gate", "gate_canopy", "gate_post"}:
        return state.collections["layout_perimeter"]
    return state.collections["layout_safety"]


def _create_layout_object(state: BuildState, item: LayoutObject) -> Any:
    common = {
        "stable_id": item.stable_id,
        "semantic_class": item.semantic_class,
        "shape": item.shape,
        "position_m": item.position_m,
        "yaw_deg": item.yaw_deg,
        "pitch_deg": item.pitch_deg,
        "dimensions_m": item.dimensions_m,
        "material_key": item.material,
        "collision": item.collision,
        "visual_only": item.visual_only,
        "source": "expanded_layout",
        "collection": _layout_collection(state, item.semantic_class),
    }
    if item.shape == "cylinder":
        obj, record = _create_cylinder(
            state,
            radius_m=float(item.radius_m),
            height_m=2.0 * float(item.half_height_m),
            **common,
        )
    else:
        obj, record = _create_box(state, **common)
    if item.semantic_class == "semantic_checkpoint":
        obj.hide_render = True
        obj["lingtu_preview_visibility"] = "hidden_debug_marker"
        record["preview_visibility"] = "hidden_debug_marker"
    state.layout_records.append(record)
    return obj


def _record_terrain_feature_descriptor(state: BuildState, item: LayoutObject) -> None:
    """Record a layout volume already represented by the heightfield."""

    state.terrain_feature_descriptors.append(
        {
            "stable_id": item.stable_id,
            "asset_key": f"terrain/feature/{item.stable_id}",
            "semantic_class": item.semantic_class,
            "blender_object": None,
            "mesh_name": None,
            "shape": item.shape,
            "position_m": list(item.position_m),
            "yaw_deg": item.yaw_deg,
            "pitch_deg": item.pitch_deg,
            "quaternion_wxyz": list(item.quaternion_wxyz),
            "scale": [1.0, 1.0, 1.0],
            "dimensions_m": list(item.dimensions_m),
            "material": item.material,
            "collision": False,
            "visual_only": False,
            "physics_proxy": "authoritative_heightfield",
            "source": "expanded_layout",
            "exported_mesh": False,
        }
    )


def _record_semantic_feature_descriptor(state: BuildState, item: LayoutObject) -> None:
    """Keep semantic checkpoints traceable without creating visible/exported meshes."""

    state.semantic_feature_descriptors.append(
        {
            "stable_id": item.stable_id,
            "asset_key": f"semantic/{item.stable_id}",
            "semantic_class": item.semantic_class,
            "blender_object": None,
            "mesh_name": None,
            "shape": item.shape,
            "position_m": list(item.position_m),
            "yaw_deg": item.yaw_deg,
            "pitch_deg": item.pitch_deg,
            "quaternion_wxyz": list(item.quaternion_wxyz),
            "scale": [1.0, 1.0, 1.0],
            "dimensions_m": list(item.dimensions_m),
            "material": item.material,
            "collision": False,
            "visual_only": True,
            "physics_proxy": "none",
            "source": "expanded_layout_semantic",
            "materialized": False,
            "exported_mesh": False,
        }
    )


def _visual_box(
    state: BuildState,
    *,
    stable_id: str,
    semantic_class: str,
    position_m: Sequence[float],
    yaw_deg: float,
    dimensions_m: Sequence[float],
    material: str,
    collection: str,
    derived_from: str,
) -> Any:
    obj, record = _create_box(
        state,
        stable_id=stable_id,
        semantic_class=semantic_class,
        shape="box",
        position_m=position_m,
        yaw_deg=yaw_deg,
        dimensions_m=dimensions_m,
        material_key=material,
        collision=False,
        visual_only=True,
        source="blender_derived_visual",
        collection=state.collections[collection],
        derived_from=derived_from,
    )
    state.visual_records.append(record)
    return obj


def _visual_cylinder(
    state: BuildState,
    *,
    stable_id: str,
    semantic_class: str,
    position_m: Sequence[float],
    yaw_deg: float,
    radius_m: float,
    height_m: float,
    material: str,
    collection: str,
    derived_from: str,
    top_radius_m: float | None = None,
    pitch_deg: float = 0.0,
) -> Any:
    obj, record = _create_cylinder(
        state,
        stable_id=stable_id,
        semantic_class=semantic_class,
        shape="cylinder" if top_radius_m is None else "frustum",
        position_m=position_m,
        yaw_deg=yaw_deg,
        dimensions_m=(2.0 * radius_m, 2.0 * radius_m, height_m),
        material_key=material,
        collision=False,
        visual_only=True,
        source="blender_derived_visual",
        collection=state.collections[collection],
        derived_from=derived_from,
        radius_m=radius_m,
        height_m=height_m,
        top_radius_m=top_radius_m,
        pitch_deg=pitch_deg,
    )
    state.visual_records.append(record)
    return obj


def _visual_ellipsoid(
    state: BuildState,
    *,
    stable_id: str,
    semantic_class: str,
    position_m: Sequence[float],
    radii_m: Sequence[float],
    material: str,
    collection: str,
    derived_from: str,
) -> Any:
    obj, record = _create_ellipsoid(
        state,
        stable_id=stable_id,
        semantic_class=semantic_class,
        shape="ellipsoid",
        position_m=position_m,
        yaw_deg=0.0,
        dimensions_m=tuple(2.0 * value for value in radii_m),
        material_key=material,
        collision=False,
        visual_only=True,
        source="blender_derived_visual",
        collection=state.collections[collection],
        derived_from=derived_from,
        radii_m=radii_m,
    )
    state.visual_records.append(record)
    return obj


def _local_to_world(item: LayoutObject, local: Sequence[float]) -> tuple[float, float, float]:
    yaw = math.radians(item.yaw_deg)
    return (
        item.position_m[0] + math.cos(yaw) * local[0] - math.sin(yaw) * local[1],
        item.position_m[1] + math.sin(yaw) * local[0] + math.cos(yaw) * local[1],
        item.position_m[2] + local[2],
    )


def _decorate_building(state: BuildState, item: LayoutObject) -> None:
    sx, sy, sz = item.dimensions_m
    top = sz / 2.0
    detail_root = f"visual/{item.safe_id}/architecture"
    _visual_box(
        state,
        stable_id=f"{detail_root}/roof_cap",
        semantic_class="roof",
        position_m=_local_to_world(item, (0.0, 0.0, top + 0.13)),
        yaw_deg=item.yaw_deg,
        dimensions_m=(sx + 0.28, sy + 0.28, 0.26),
        material="roof_metal",
        collection="visual_architecture",
        derived_from=item.stable_id,
    )
    window_count = max(2, min(6, int(sx // 7.0)))
    spacing = sx / (window_count + 1)
    window_width = min(2.8, spacing * 0.6)
    window_height = min(2.2, max(1.1, sz * 0.2))
    window_z = -top + max(2.2, sz * 0.48)
    for index in range(window_count):
        local_x = -sx / 2.0 + spacing * (index + 1)
        _visual_box(
            state,
            stable_id=f"{detail_root}/front_window_{index:02d}",
            semantic_class="window",
            position_m=_local_to_world(item, (local_x, -sy / 2.0 - 0.035, window_z)),
            yaw_deg=item.yaw_deg,
            dimensions_m=(window_width, 0.07, window_height),
            material="window_glass",
            collection="visual_architecture",
            derived_from=item.stable_id,
        )
    vent_count = max(2, min(5, int(sx // 14.0)))
    for index in range(vent_count):
        local_x = (index - (vent_count - 1) / 2.0) * min(9.0, sx / max(vent_count, 1))
        _visual_cylinder(
            state,
            stable_id=f"{detail_root}/roof_vent_{index:02d}",
            semantic_class="roof_vent",
            position_m=_local_to_world(item, (local_x, 0.0, top + 0.68)),
            yaw_deg=item.yaw_deg,
            radius_m=0.42,
            height_m=1.1,
            material="steel",
            collection="visual_architecture",
            derived_from=item.stable_id,
        )


def _decorate_road(state: BuildState, item: LayoutObject) -> None:
    sx, sy, sz = item.dimensions_m
    long_x = sx >= sy
    length = sx if long_x else sy
    width = sy if long_x else sx
    if length < 12.0 or width < 3.0:
        return
    dash_length = 3.0
    pitch = 7.0
    dash_count = max(1, int((length - 4.0) // pitch) + 1)
    for index in range(dash_count):
        along = (index - (dash_count - 1) / 2.0) * pitch
        local = (along, 0.0, sz / 2.0 + 0.018) if long_x else (0.0, along, sz / 2.0 + 0.018)
        dimensions = (dash_length, 0.14, 0.025) if long_x else (0.14, dash_length, 0.025)
        _visual_box(
            state,
            stable_id=f"visual/{item.safe_id}/marking/center_dash_{index:02d}",
            semantic_class="road_marking",
            position_m=_local_to_world(item, local),
            yaw_deg=item.yaw_deg,
            dimensions_m=dimensions,
            material="line_white",
            collection="visual_markings",
            derived_from=item.stable_id,
        )


def _decorate_parking_space(state: BuildState, item: LayoutObject) -> None:
    sx, sy, sz = item.dimensions_m
    top_z = sz / 2.0 + 0.02
    line = 0.09
    root = f"visual/{item.safe_id}/parking"
    for label, local, dimensions in (
        ("left", (-sx / 2.0, 0.0, top_z), (line, sy, 0.026)),
        ("right", (sx / 2.0, 0.0, top_z), (line, sy, 0.026)),
        ("stop", (0.0, sy / 2.0, top_z), (sx, line, 0.026)),
    ):
        _visual_box(
            state,
            stable_id=f"{root}/{label}_line",
            semantic_class="parking_marking",
            position_m=_local_to_world(item, local),
            yaw_deg=item.yaw_deg,
            dimensions_m=dimensions,
            material="line_white",
            collection="visual_markings",
            derived_from=item.stable_id,
        )


def _decorate_storage_tank(state: BuildState, item: LayoutObject) -> None:
    radius = float(item.radius_m or item.dimensions_m[0] / 2.0)
    height = item.dimensions_m[2]
    root = f"visual/{item.safe_id}/tank"
    top_z = height / 2.0
    _visual_cylinder(
        state,
        stable_id=f"{root}/conical_roof",
        semantic_class="tank_roof",
        position_m=_local_to_world(item, (0.0, 0.0, top_z + max(0.25, radius * 0.12))),
        yaw_deg=item.yaw_deg,
        radius_m=radius * 1.02,
        top_radius_m=max(0.35, radius * 0.12),
        height_m=max(0.5, radius * 0.24),
        material="roof_metal",
        collection="visual_process",
        derived_from=item.stable_id,
    )
    for index, fraction in enumerate((0.2, 0.5, 0.8)):
        _visual_cylinder(
            state,
            stable_id=f"{root}/shell_band_{index:02d}",
            semantic_class="tank_band",
            position_m=_local_to_world(item, (0.0, 0.0, -height / 2.0 + height * fraction)),
            yaw_deg=item.yaw_deg,
            radius_m=radius + 0.055,
            height_m=0.1,
            material="steel",
            collection="visual_process",
            derived_from=item.stable_id,
        )
    _visual_cylinder(
        state,
        stable_id=f"{root}/service_pipe",
        semantic_class="service_pipe",
        position_m=_local_to_world(item, (radius + 0.18, 0.0, 0.0)),
        yaw_deg=item.yaw_deg,
        radius_m=0.11,
        height_m=height * 0.9,
        material="safety_yellow",
        collection="visual_process",
        derived_from=item.stable_id,
    )


def _decorate_container(state: BuildState, item: LayoutObject) -> None:
    sx, sy, sz = item.dimensions_m
    root = f"visual/{item.safe_id}/container"
    count = max(4, min(12, int(sx // 0.75)))
    spacing = sx / (count + 1)
    accent = (
        "container_red" if int(hashlib.sha256(item.stable_id.encode()).hexdigest()[:2], 16) % 2 else "container_blue"
    )
    for index in range(count):
        x = -sx / 2.0 + spacing * (index + 1)
        for side, y in (("front", -sy / 2.0 - 0.025), ("rear", sy / 2.0 + 0.025)):
            _visual_box(
                state,
                stable_id=f"{root}/{side}_rib_{index:02d}",
                semantic_class="container_rib",
                position_m=_local_to_world(item, (x, y, 0.0)),
                yaw_deg=item.yaw_deg,
                dimensions_m=(0.055, 0.05, sz * 0.9),
                material=accent,
                collection="visual_architecture",
                derived_from=item.stable_id,
            )


def _decorate_fence(state: BuildState, item: LayoutObject) -> None:
    sx, sy, sz = item.dimensions_m
    long_x = sx >= sy
    length = sx if long_x else sy
    count = max(2, int(length // 4.0) + 1)
    root = f"visual/{item.safe_id}/fence"
    for index in range(count):
        along = -length / 2.0 + length * index / (count - 1)
        local = (along, 0.0, 0.1) if long_x else (0.0, along, 0.1)
        _visual_box(
            state,
            stable_id=f"{root}/post_{index:02d}",
            semantic_class="fence_post",
            position_m=_local_to_world(item, local),
            yaw_deg=item.yaw_deg,
            dimensions_m=(0.13, 0.13, sz + 0.2),
            material="steel",
            collection="visual_furniture",
            derived_from=item.stable_id,
        )
    for index, fraction in enumerate((0.24, 0.72)):
        local_z = -sz / 2.0 + sz * fraction
        dimensions = (length, 0.09, 0.08) if long_x else (0.09, length, 0.08)
        _visual_box(
            state,
            stable_id=f"{root}/rail_{index:02d}",
            semantic_class="fence_rail",
            position_m=_local_to_world(item, (0.0, 0.0, local_z)),
            yaw_deg=item.yaw_deg,
            dimensions_m=dimensions,
            material="steel",
            collection="visual_furniture",
            derived_from=item.stable_id,
        )


def _decorate_speed_bump(state: BuildState, item: LayoutObject) -> None:
    sx, sy, sz = item.dimensions_m
    long_x = sx >= sy
    length = sx if long_x else sy
    stripe_count = max(2, min(8, int(length // 0.8)))
    for index in range(stripe_count):
        along = (index - (stripe_count - 1) / 2.0) * length / stripe_count
        local = (along, 0.0, sz / 2.0 + 0.015) if long_x else (0.0, along, sz / 2.0 + 0.015)
        dimensions = (
            (length / stripe_count * 0.55, sy * 1.01, 0.025)
            if long_x
            else (sx * 1.01, length / stripe_count * 0.55, 0.025)
        )
        _visual_box(
            state,
            stable_id=f"visual/{item.safe_id}/speed_bump/stripe_{index:02d}",
            semantic_class="hazard_marking",
            position_m=_local_to_world(item, local),
            yaw_deg=item.yaw_deg,
            dimensions_m=dimensions,
            material="safety_yellow",
            collection="visual_markings",
            derived_from=item.stable_id,
        )


def _realism_id(
    state: BuildState,
    target: str,
    feature: str,
    index: int | None = None,
) -> str:
    """Return the recipe-namespaced deterministic dressing identity."""

    suffix = f"/{index:03d}" if index is not None else ""
    return f"{state.realism.namespace}/{target}/{feature}{suffix}"


def _decorate_building_realism_v2(state: BuildState, item: LayoutObject) -> None:
    sx, sy, sz = item.dimensions_m
    root = item.safe_id
    facade_material = "factory_cladding" if item.semantic_class == "main_factory_building" else "warehouse_cladding"
    _visual_box(
        state,
        stable_id=_realism_id(state, root, "foundation_plinth"),
        semantic_class="building_foundation",
        position_m=_local_to_world(item, (0.0, 0.0, -sz / 2.0 + 0.23)),
        yaw_deg=item.yaw_deg,
        dimensions_m=(sx + 0.18, sy + 0.18, 0.46),
        material="concrete_dirty",
        collection="visual_realism",
        derived_from=item.stable_id,
    )

    front_module_count = max(4, min(24, round(sx / 3.0)))
    side_module_count = max(3, min(14, round(sy / 3.5)))
    for side_index, local_y in enumerate((-sy / 2.0 - 0.035, sy / 2.0 + 0.035)):
        for index in range(front_module_count + 1):
            local_x = -sx / 2.0 + sx * index / front_module_count
            _visual_box(
                state,
                stable_id=_realism_id(state, root, f"facade_seam_{side_index}", index),
                semantic_class="corrugated_panel_seam",
                position_m=_local_to_world(item, (local_x, local_y, 0.0)),
                yaw_deg=item.yaw_deg,
                dimensions_m=(0.045, 0.065, sz * 0.94),
                material=facade_material,
                collection="visual_realism",
                derived_from=item.stable_id,
            )
    for side_index, local_x in enumerate((-sx / 2.0 - 0.035, sx / 2.0 + 0.035)):
        for index in range(side_module_count + 1):
            local_y = -sy / 2.0 + sy * index / side_module_count
            _visual_box(
                state,
                stable_id=_realism_id(state, root, f"side_seam_{side_index}", index),
                semantic_class="corrugated_panel_seam",
                position_m=_local_to_world(item, (local_x, local_y, 0.0)),
                yaw_deg=item.yaw_deg,
                dimensions_m=(0.065, 0.045, sz * 0.94),
                material=facade_material,
                collection="visual_realism",
                derived_from=item.stable_id,
            )

    top = sz / 2.0
    for index, (local, dimensions) in enumerate(
        (
            ((0.0, -sy / 2.0, top + 0.18), (sx + 0.35, 0.18, 0.36)),
            ((0.0, sy / 2.0, top + 0.18), (sx + 0.35, 0.18, 0.36)),
            ((-sx / 2.0, 0.0, top + 0.18), (0.18, sy, 0.36)),
            ((sx / 2.0, 0.0, top + 0.18), (0.18, sy, 0.36)),
        )
    ):
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "roof_parapet", index),
            semantic_class="roof_parapet",
            position_m=_local_to_world(item, local),
            yaw_deg=item.yaw_deg,
            dimensions_m=dimensions,
            material="galvanized",
            collection="visual_realism",
            derived_from=item.stable_id,
        )
    for side_index, local_y in enumerate((-sy / 2.0 - 0.13, sy / 2.0 + 0.13)):
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "roof_gutter", side_index),
            semantic_class="roof_gutter",
            position_m=_local_to_world(item, (0.0, local_y, top - 0.02)),
            yaw_deg=item.yaw_deg,
            dimensions_m=(sx, 0.14, 0.14),
            material="galvanized",
            collection="visual_realism",
            derived_from=item.stable_id,
        )
        for corner_index, local_x in enumerate((-sx / 2.0 + 0.4, sx / 2.0 - 0.4)):
            _visual_box(
                state,
                stable_id=_realism_id(state, root, f"downspout_{side_index}", corner_index),
                semantic_class="downspout",
                position_m=_local_to_world(item, (local_x, local_y, 0.0)),
                yaw_deg=item.yaw_deg,
                dimensions_m=(0.13, 0.13, sz * 0.9),
                material="galvanized",
                collection="visual_realism",
                derived_from=item.stable_id,
            )

    if item.semantic_class in {"warehouse_building", "main_factory_building"}:
        door_count = 3 if item.semantic_class == "warehouse_building" else 2
        for index in range(door_count):
            local_x = (index - (door_count - 1) / 2.0) * min(20.0, sx / door_count)
            door_height = min(5.0, sz * 0.55)
            _visual_box(
                state,
                stable_id=_realism_id(state, root, "roller_shutter", index),
                semantic_class="roller_shutter_door",
                position_m=_local_to_world(item, (local_x, -sy / 2.0 - 0.075, -top + door_height / 2.0 + 0.35)),
                yaw_deg=item.yaw_deg,
                dimensions_m=(5.2, 0.12, door_height),
                material="rollup_door",
                collection="visual_realism",
                derived_from=item.stable_id,
            )
    else:
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "service_door"),
            semantic_class="service_door",
            position_m=_local_to_world(item, (0.0, -sy / 2.0 - 0.075, -top + 1.2)),
            yaw_deg=item.yaw_deg,
            dimensions_m=(1.25, 0.12, 2.4),
            material="painted_steel",
            collection="visual_realism",
            derived_from=item.stable_id,
        )

    skylight_count = max(2, min(7, round(sx / 12.0)))
    for index in range(skylight_count):
        local_x = (index - (skylight_count - 1) / 2.0) * sx / (skylight_count + 1)
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "roof_skylight", index),
            semantic_class="roof_skylight",
            position_m=_local_to_world(item, (local_x, 0.0, top + 0.31)),
            yaw_deg=item.yaw_deg,
            dimensions_m=(min(4.0, sx / 8.0), min(2.0, sy / 6.0), 0.18),
            material="industrial_glass",
            collection="visual_realism",
            derived_from=item.stable_id,
        )
    hvac_count = 1 if sx < 20.0 else 2
    for index in range(hvac_count):
        local_x = (index - (hvac_count - 1) / 2.0) * min(12.0, sx / 3.0)
        base_z = top + 0.55
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "hvac_body", index),
            semantic_class="hvac_unit",
            position_m=_local_to_world(item, (local_x, sy * 0.16, base_z)),
            yaw_deg=item.yaw_deg,
            dimensions_m=(2.2, 1.2, 0.9),
            material="galvanized",
            collection="visual_utilities",
            derived_from=item.stable_id,
        )
        _visual_cylinder(
            state,
            stable_id=_realism_id(state, root, "hvac_fan", index),
            semantic_class="hvac_fan",
            position_m=_local_to_world(item, (local_x, sy * 0.16, base_z + 0.5)),
            yaw_deg=item.yaw_deg,
            radius_m=0.36,
            height_m=0.08,
            material="safety_black",
            collection="visual_utilities",
            derived_from=item.stable_id,
        )


def _decorate_loading_realism_v2(state: BuildState, item: LayoutObject) -> None:
    sx, sy, sz = item.dimensions_m
    root = item.safe_id
    if item.semantic_class == "loading_dock":
        dock_top = item.position_m[2] + sz / 2.0
        door_center = (item.position_m[0], item.position_m[1] + sy / 2.0 + 0.18, 3.35)
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "dock_door"),
            semantic_class="roller_shutter_door",
            position_m=door_center,
            yaw_deg=item.yaw_deg,
            dimensions_m=(sx * 0.72, 0.16, 4.4),
            material="rollup_door",
            collection="visual_realism",
            derived_from=item.stable_id,
        )
        for index, (offset_x, dimensions) in enumerate(
            (
                (-sx * 0.39, (0.42, 0.7, 4.7)),
                (sx * 0.39, (0.42, 0.7, 4.7)),
                (0.0, (sx * 0.82, 0.7, 0.42)),
            )
        ):
            position = (
                item.position_m[0] + offset_x,
                item.position_m[1] + sy / 2.0,
                3.42 if index < 2 else 5.55,
            )
            _visual_box(
                state,
                stable_id=_realism_id(state, root, "dock_shelter", index),
                semantic_class="dock_shelter",
                position_m=position,
                yaw_deg=item.yaw_deg,
                dimensions_m=dimensions,
                material="dock_shelter",
                collection="visual_realism",
                derived_from=item.stable_id,
            )
        for index, offset_x in enumerate((-sx * 0.34, sx * 0.34)):
            _visual_box(
                state,
                stable_id=_realism_id(state, root, "rubber_bumper", index),
                semantic_class="dock_bumper",
                position_m=(
                    item.position_m[0] + offset_x,
                    item.position_m[1] - sy / 2.0 - 0.12,
                    dock_top + 0.45,
                ),
                yaw_deg=item.yaw_deg,
                dimensions_m=(0.38, 0.26, 0.9),
                material="rubber",
                collection="visual_realism",
                derived_from=item.stable_id,
            )
        for index, offset_x in enumerate((-sx * 0.46, -sx * 0.31, sx * 0.31, sx * 0.46)):
            _visual_cylinder(
                state,
                stable_id=_realism_id(state, root, "dock_bollard", index),
                semantic_class="safety_bollard",
                position_m=(item.position_m[0] + offset_x, item.position_m[1] - 1.2, dock_top + 0.55),
                yaw_deg=0.0,
                radius_m=0.12,
                height_m=1.1,
                material="safety_yellow",
                collection="visual_realism",
                derived_from=item.stable_id,
            )
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "dock_light"),
            semantic_class="wall_pack_light",
            position_m=(item.position_m[0], item.position_m[1] + 2.5, 6.0),
            yaw_deg=item.yaw_deg,
            dimensions_m=(0.9, 0.3, 0.28),
            material="lamp_emissive",
            collection="visual_realism",
            derived_from=item.stable_id,
        )
    else:
        long_x = sx >= sy
        for index in range(5):
            along = (index - 2) * (sx if long_x else sy) / 5.5
            local = (along, 0.0, sz / 2.0 + 0.026) if long_x else (0.0, along, sz / 2.0 + 0.026)
            dimensions = (0.55, sy * 1.02, 0.025) if long_x else (sx * 1.02, 0.55, 0.025)
            _visual_box(
                state,
                stable_id=_realism_id(state, root, "ramp_hazard_stripe", index),
                semantic_class="hazard_marking",
                position_m=_local_to_world(item, local),
                yaw_deg=item.yaw_deg,
                dimensions_m=dimensions,
                material="safety_yellow" if index % 2 == 0 else "safety_black",
                collection="visual_markings",
                derived_from=item.stable_id,
            )


def _add_south_gate_realism_v2(state: BuildState, objects: Sequence[LayoutObject]) -> None:
    by_id = {item.stable_id: item for item in objects}
    gate_leaves = [item for item in objects if item.semantic_class == "gate"]
    for item in gate_leaves:
        _, sy, sz = item.dimensions_m
        root = item.safe_id
        for index in range(9):
            local_y = -sy / 2.0 + sy * (index + 0.5) / 9.0
            _visual_box(
                state,
                stable_id=_realism_id(state, root, "gate_picket", index),
                semantic_class="sliding_gate_picket",
                position_m=_local_to_world(item, (0.0, local_y, 0.0)),
                yaw_deg=item.yaw_deg,
                dimensions_m=(0.09, 0.055, sz * 0.92),
                material="galvanized",
                collection="visual_realism",
                derived_from=item.stable_id,
            )
        for index, local_z in enumerate((-sz * 0.28, sz * 0.28)):
            _visual_box(
                state,
                stable_id=_realism_id(state, root, "gate_rail", index),
                semantic_class="sliding_gate_rail",
                position_m=_local_to_world(item, (0.0, 0.0, local_z)),
                yaw_deg=item.yaw_deg,
                dimensions_m=(0.11, sy, 0.11),
                material="galvanized",
                collection="visual_realism",
                derived_from=item.stable_id,
            )
    gate_source = gate_leaves[0].stable_id if gate_leaves else "layout.extent_m"
    for side_index, x in enumerate((-4.7, 4.7)):
        _visual_box(
            state,
            stable_id=_realism_id(state, "south_gate", "safety_island", side_index),
            semantic_class="gate_safety_island",
            position_m=(x, -82.0, 0.15),
            yaw_deg=0.0,
            dimensions_m=(1.15, 4.6, 0.3),
            material="concrete_dirty",
            collection="visual_realism",
            derived_from=gate_source,
        )
        _visual_box(
            state,
            stable_id=_realism_id(state, "south_gate", "barrier_cabinet", side_index),
            semantic_class="barrier_cabinet",
            position_m=(x, -82.0, 0.75),
            yaw_deg=0.0,
            dimensions_m=(0.42, 0.55, 1.2),
            material="safety_yellow",
            collection="visual_realism",
            derived_from=gate_source,
        )
        _visual_box(
            state,
            stable_id=_realism_id(state, "south_gate", "barrier_arm", side_index),
            semantic_class="barrier_arm",
            position_m=(x + (-2.4 if x > 0 else 2.4), -82.0, 1.18),
            yaw_deg=0.0,
            dimensions_m=(4.8, 0.13, 0.13),
            material="safety_red",
            collection="visual_realism",
            derived_from=gate_source,
        )
        _visual_box(
            state,
            stable_id=_realism_id(state, "south_gate", "intercom", side_index),
            semantic_class="gate_intercom",
            position_m=(x, -84.0, 1.18),
            yaw_deg=0.0,
            dimensions_m=(0.26, 0.2, 0.4),
            material="safety_black",
            collection="visual_realism",
            derived_from=gate_source,
        )
        pole_x = -9.8 if side_index == 0 else 9.8
        _visual_cylinder(
            state,
            stable_id=_realism_id(state, "south_gate", "camera_pole", side_index),
            semantic_class="security_camera_pole",
            position_m=(pole_x, -84.7, 2.25),
            yaw_deg=0.0,
            radius_m=0.08,
            height_m=4.5,
            material="galvanized",
            collection="visual_realism",
            derived_from=gate_source,
        )
        _visual_box(
            state,
            stable_id=_realism_id(state, "south_gate", "security_camera", side_index),
            semantic_class="security_camera",
            position_m=(pole_x + (-0.18 if side_index == 0 else 0.18), -84.7, 4.45),
            yaw_deg=0.0,
            dimensions_m=(0.36, 0.22, 0.22),
            material="vehicle_white",
            collection="visual_realism",
            derived_from=gate_source,
        )
    for index, x in enumerate((-7.4, -6.4, 6.4, 7.4)):
        _visual_cylinder(
            state,
            stable_id=_realism_id(state, "south_gate", "impact_bollard", index),
            semantic_class="impact_bollard",
            position_m=(x, -80.0, 0.55),
            yaw_deg=0.0,
            radius_m=0.12,
            height_m=1.1,
            material="safety_yellow",
            collection="visual_realism",
            derived_from=by_id.get("south_guardhouse", gate_leaves[0] if gate_leaves else None).stable_id
            if by_id.get("south_guardhouse", gate_leaves[0] if gate_leaves else None)
            else gate_source,
        )
    canopy = by_id.get("gate_canopy")
    if canopy is not None:
        for index, x in enumerate((-7.5, -4.5, -1.5, 1.5, 4.5, 7.5)):
            _visual_box(
                state,
                stable_id=_realism_id(state, "south_gate", "canopy_light", index),
                semantic_class="canopy_light",
                position_m=(x, -84.5, 3.28),
                yaw_deg=0.0,
                dimensions_m=(0.75, 0.28, 0.08),
                material="lamp_emissive",
                collection="visual_realism",
                derived_from=canopy.stable_id,
            )


def _decorate_tank_realism_v2(state: BuildState, item: LayoutObject) -> None:
    radius = float(item.radius_m or item.dimensions_m[0] / 2.0)
    height = item.dimensions_m[2]
    root = item.safe_id
    base_z = item.position_m[2] - height / 2.0
    ladder_x = item.position_m[0] + radius + 0.24
    for index, y_offset in enumerate((-0.23, 0.23)):
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "access_ladder_rail", index),
            semantic_class="tank_access_ladder",
            position_m=(ladder_x, item.position_m[1] + y_offset, base_z + height * 0.52),
            yaw_deg=0.0,
            dimensions_m=(0.075, 0.075, height * 1.03),
            material="galvanized",
            collection="visual_realism",
            derived_from=item.stable_id,
        )
    rung_count = 13
    for index in range(rung_count):
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "access_ladder_rung", index),
            semantic_class="tank_access_ladder",
            position_m=(
                ladder_x,
                item.position_m[1],
                base_z + 0.35 + index * (height - 0.6) / (rung_count - 1),
            ),
            yaw_deg=0.0,
            dimensions_m=(0.08, 0.55, 0.055),
            material="galvanized",
            collection="visual_realism",
            derived_from=item.stable_id,
        )
    platform_z = base_z + height + 0.2
    _visual_cylinder(
        state,
        stable_id=_realism_id(state, root, "inspection_platform"),
        semantic_class="tank_inspection_platform",
        position_m=(item.position_m[0], item.position_m[1], platform_z),
        yaw_deg=0.0,
        radius_m=radius + 0.5,
        height_m=0.12,
        material="galvanized",
        collection="visual_realism",
        derived_from=item.stable_id,
    )
    rail_count = 12
    for index in range(rail_count):
        angle = 2.0 * math.pi * index / rail_count
        x = item.position_m[0] + math.cos(angle) * (radius + 0.42)
        y = item.position_m[1] + math.sin(angle) * (radius + 0.42)
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "platform_rail_post", index),
            semantic_class="tank_platform_guardrail",
            position_m=(x, y, platform_z + 0.55),
            yaw_deg=math.degrees(angle),
            dimensions_m=(0.065, 0.065, 1.0),
            material="galvanized",
            collection="visual_realism",
            derived_from=item.stable_id,
        )
    for index, z_offset in enumerate((0.32, 0.82)):
        _visual_cylinder(
            state,
            stable_id=_realism_id(state, root, "platform_guard_ring", index),
            semantic_class="tank_platform_guardrail",
            position_m=(item.position_m[0], item.position_m[1], platform_z + z_offset),
            yaw_deg=0.0,
            radius_m=radius + 0.47,
            height_m=0.055,
            material="galvanized",
            collection="visual_realism",
            derived_from=item.stable_id,
        )
    _visual_cylinder(
        state,
        stable_id=_realism_id(state, root, "inspection_manhole"),
        semantic_class="tank_inspection_hatch",
        position_m=(item.position_m[0] - radius * 0.25, item.position_m[1], platform_z + 0.18),
        yaw_deg=0.0,
        radius_m=0.42,
        height_m=0.28,
        material="roof_white",
        collection="visual_realism",
        derived_from=item.stable_id,
    )
    for index in range(6):
        angle = math.radians(-75.0 + index * 30.0)
        x = item.position_m[0] + math.cos(angle) * (radius + 0.035)
        y = item.position_m[1] + math.sin(angle) * (radius + 0.035)
        streak_height = stable_variation(item.stable_id, index, low=0.8, high=2.4, seed=state.realism.seed)
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "rust_streak", index),
            semantic_class="tank_weathering",
            position_m=(x, y, base_z + height * 0.44),
            yaw_deg=math.degrees(angle) + 90.0,
            dimensions_m=(0.14, 0.025, streak_height),
            material="rust",
            collection="visual_realism",
            derived_from=item.stable_id,
        )
    for index, y_offset in enumerate((-0.45, 0.0, 0.45)):
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "valve_pipe", index),
            semantic_class="tank_valve_pipe",
            position_m=(item.position_m[0] - radius - 1.0, item.position_m[1] + y_offset, base_z + 0.65 + index * 0.22),
            yaw_deg=0.0,
            dimensions_m=(2.0, 0.12, 0.12),
            material="painted_safety_steel",
            collection="visual_utilities",
            derived_from=item.stable_id,
        )
        _visual_cylinder(
            state,
            stable_id=_realism_id(state, root, "valve_handwheel", index),
            semantic_class="tank_valve",
            position_m=(item.position_m[0] - radius - 1.4, item.position_m[1] + y_offset, base_z + 0.82 + index * 0.22),
            yaw_deg=0.0,
            radius_m=0.24,
            height_m=0.07,
            material="safety_red",
            collection="visual_utilities",
            derived_from=item.stable_id,
        )


def _add_tank_utilities_realism_v2(state: BuildState, objects: Sequence[LayoutObject]) -> None:
    rack = next((item for item in objects if item.semantic_class == "pipe_rack"), None)
    if rack is None:
        return
    sx, _, _ = rack.dimensions_m
    for index in range(7):
        x = rack.position_m[0] - sx / 2.0 + sx * index / 6.0
        for side_index, y in enumerate((rack.position_m[1] - 1.0, rack.position_m[1] + 1.0)):
            _visual_box(
                state,
                stable_id=_realism_id(state, rack.safe_id, f"rack_support_{side_index}", index),
                semantic_class="pipe_rack_support",
                position_m=(x, y, 1.55),
                yaw_deg=0.0,
                dimensions_m=(0.13, 0.13, 3.1),
                material="galvanized",
                collection="visual_utilities",
                derived_from=rack.stable_id,
            )
        _visual_box(
            state,
            stable_id=_realism_id(state, rack.safe_id, "rack_crossbeam", index),
            semantic_class="pipe_rack_support",
            position_m=(x, rack.position_m[1], 3.05),
            yaw_deg=0.0,
            dimensions_m=(0.16, 2.2, 0.16),
            material="galvanized",
            collection="visual_utilities",
            derived_from=rack.stable_id,
        )
    for line_index, (y_offset, z) in enumerate(((-0.55, 3.2), (0.0, 3.35), (0.55, 3.5))):
        _visual_box(
            state,
            stable_id=_realism_id(state, rack.safe_id, "process_pipe", line_index),
            semantic_class="process_pipe",
            position_m=(rack.position_m[0], rack.position_m[1] + y_offset, z),
            yaw_deg=0.0,
            dimensions_m=(sx, 0.16, 0.16),
            material="painted_safety_steel" if line_index == 0 else "galvanized",
            collection="visual_utilities",
            derived_from=rack.stable_id,
        )
        for flange_index in range(5):
            x = rack.position_m[0] - sx * 0.4 + flange_index * sx * 0.2
            _visual_cylinder(
                state,
                stable_id=_realism_id(state, rack.safe_id, f"pipe_flange_{line_index}", flange_index),
                semantic_class="pipe_flange",
                position_m=(x, rack.position_m[1] + y_offset, z),
                yaw_deg=0.0,
                pitch_deg=90.0,
                radius_m=0.18,
                height_m=0.08,
                material="galvanized",
                collection="visual_utilities",
                derived_from=rack.stable_id,
            )


def _decorate_road_surface_realism_v2(state: BuildState, item: LayoutObject) -> None:
    sx, sy, sz = item.dimensions_m
    long_x = sx >= sy
    length = sx if long_x else sy
    width = sy if long_x else sx
    root = item.safe_id
    for index in range(4):
        along = stable_variation(
            item.stable_id, index * 5, low=-length * 0.4, high=length * 0.4, seed=state.realism.seed
        )
        across = stable_variation(
            item.stable_id, index * 5 + 1, low=-width * 0.28, high=width * 0.28, seed=state.realism.seed
        )
        patch_length = stable_variation(item.stable_id, index * 5 + 2, low=1.8, high=4.8, seed=state.realism.seed)
        patch_width = stable_variation(item.stable_id, index * 5 + 3, low=0.7, high=2.0, seed=state.realism.seed)
        local = (along, across, sz / 2.0 + 0.018) if long_x else (across, along, sz / 2.0 + 0.018)
        dimensions = (patch_length, patch_width, 0.018) if long_x else (patch_width, patch_length, 0.018)
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "asphalt_patch", index),
            semantic_class="asphalt_patch",
            position_m=_local_to_world(item, local),
            yaw_deg=item.yaw_deg
            + stable_variation(item.stable_id, index * 5 + 4, low=-4.0, high=4.0, seed=state.realism.seed),
            dimensions_m=dimensions,
            material="asphalt_patch",
            collection="visual_markings",
            derived_from=item.stable_id,
        )
    for index in range(6):
        along = -length * 0.34 + index * length * 0.68 / 5.0
        across = (-0.18 if index % 2 == 0 else 0.18) * min(width, 8.0)
        local = (along, across, sz / 2.0 + 0.029) if long_x else (across, along, sz / 2.0 + 0.029)
        dimensions = (3.8, 0.16, 0.012) if long_x else (0.16, 3.8, 0.012)
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "tire_wear", index),
            semantic_class="tire_mark",
            position_m=_local_to_world(item, local),
            yaw_deg=item.yaw_deg,
            dimensions_m=dimensions,
            material="tire_mark",
            collection="visual_markings",
            derived_from=item.stable_id,
        )
    for index in range(3):
        along = stable_variation(
            item.stable_id, 80 + index, low=-length * 0.36, high=length * 0.36, seed=state.realism.seed
        )
        across = stable_variation(
            item.stable_id, 90 + index, low=-width * 0.3, high=width * 0.3, seed=state.realism.seed
        )
        local = (along, across, sz / 2.0 + 0.035) if long_x else (across, along, sz / 2.0 + 0.035)
        dimensions = (2.8, 0.045, 0.009) if long_x else (0.045, 2.8, 0.009)
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "sealed_crack", index),
            semantic_class="sealed_crack",
            position_m=_local_to_world(item, local),
            yaw_deg=item.yaw_deg
            + stable_variation(item.stable_id, 100 + index, low=-38.0, high=38.0, seed=state.realism.seed),
            dimensions_m=dimensions,
            material="crack_fill",
            collection="visual_markings",
            derived_from=item.stable_id,
        )
    if length >= 50.0:
        for index, fraction in enumerate((-0.24, 0.24)):
            along = length * fraction
            across = width * (0.27 if index == 0 else -0.27)
            position = _local_to_world(
                item,
                (along, across, sz / 2.0 + 0.032) if long_x else (across, along, sz / 2.0 + 0.032),
            )
            _visual_cylinder(
                state,
                stable_id=_realism_id(state, root, "manhole_cover", index),
                semantic_class="manhole_cover",
                position_m=position,
                yaw_deg=item.yaw_deg,
                radius_m=0.45,
                height_m=0.025,
                material="galvanized",
                collection="visual_markings",
                derived_from=item.stable_id,
            )
            grate_local = (
                (along + 4.0, width * 0.42, sz / 2.0 + 0.032)
                if long_x
                else (width * 0.42, along + 4.0, sz / 2.0 + 0.032)
            )
            _visual_box(
                state,
                stable_id=_realism_id(state, root, "storm_grate", index),
                semantic_class="storm_drain_grate",
                position_m=_local_to_world(item, grate_local),
                yaw_deg=item.yaw_deg,
                dimensions_m=(0.75, 0.45, 0.025) if long_x else (0.45, 0.75, 0.025),
                material="galvanized",
                collection="visual_markings",
                derived_from=item.stable_id,
            )


def _decorate_drainage_realism_v2(state: BuildState, item: LayoutObject) -> None:
    if item.semantic_class not in {"drainage_ditch", "drainage_grate", "curb"}:
        return
    sx, sy, sz = item.dimensions_m
    long_x = sx >= sy
    length = sx if long_x else sy
    count = max(4, min(12, int(length // 8.0)))
    material = "concrete_wet" if "drainage" in item.semantic_class else "dirt"
    for index in range(count):
        along = -length / 2.0 + length * (index + 0.5) / count
        local = (along, 0.0, sz / 2.0 + 0.02) if long_x else (0.0, along, sz / 2.0 + 0.02)
        dimensions = (
            (length / count * 0.62, min(sy, 0.24), 0.018) if long_x else (min(sx, 0.24), length / count * 0.62, 0.018)
        )
        _visual_box(
            state,
            stable_id=_realism_id(state, item.safe_id, "edge_stain", index),
            semantic_class="drainage_weathering",
            position_m=_local_to_world(item, local),
            yaw_deg=item.yaw_deg,
            dimensions_m=dimensions,
            material=material,
            collection="visual_markings",
            derived_from=item.stable_id,
        )


def _world_from_pose(origin: Sequence[float], yaw_deg: float, local: Sequence[float]) -> tuple[float, float, float]:
    yaw = math.radians(yaw_deg)
    return (
        origin[0] + math.cos(yaw) * local[0] - math.sin(yaw) * local[1],
        origin[1] + math.sin(yaw) * local[0] + math.cos(yaw) * local[1],
        origin[2] + local[2],
    )


def _add_vehicle_silhouette(
    state: BuildState,
    *,
    vehicle_id: str,
    position_m: Sequence[float],
    yaw_deg: float,
    kind: str,
    derived_from: str,
) -> None:
    if kind == "truck":
        body_length, body_width, body_height = 8.5, 2.5, 2.8
        cargo_length = 5.8
    elif kind == "forklift":
        body_length, body_width, body_height = 2.8, 1.45, 1.65
        cargo_length = 0.0
    else:
        body_length, body_width, body_height = 4.7, 2.0, 1.9
        cargo_length = 0.0
    root = f"vehicle_{vehicle_id}"
    base_material = (
        "vehicle_blue" if stable_variation(vehicle_id, 0, seed=state.realism.seed) > 0.5 else "vehicle_white"
    )
    _visual_box(
        state,
        stable_id=_realism_id(state, root, "chassis"),
        semantic_class=f"parked_{kind}",
        position_m=_world_from_pose(position_m, yaw_deg, (0.0, 0.0, 0.62)),
        yaw_deg=yaw_deg,
        dimensions_m=(body_length, body_width, 0.48),
        material="transformer_dark",
        collection="visual_vehicles",
        derived_from=derived_from,
    )
    cab_length = 2.15 if kind == "truck" else body_length * 0.72
    cab_x = -body_length / 2.0 + cab_length / 2.0 + 0.2
    _visual_box(
        state,
        stable_id=_realism_id(state, root, "cab"),
        semantic_class=f"parked_{kind}",
        position_m=_world_from_pose(position_m, yaw_deg, (cab_x, 0.0, body_height / 2.0 + 0.45)),
        yaw_deg=yaw_deg,
        dimensions_m=(cab_length, body_width * 0.94, body_height),
        material=base_material,
        collection="visual_vehicles",
        derived_from=derived_from,
    )
    _visual_box(
        state,
        stable_id=_realism_id(state, root, "windscreen"),
        semantic_class="vehicle_glass",
        position_m=_world_from_pose(position_m, yaw_deg, (cab_x - cab_length / 2.0 - 0.035, 0.0, body_height * 0.77)),
        yaw_deg=yaw_deg,
        dimensions_m=(0.08, body_width * 0.78, body_height * 0.42),
        material="industrial_glass",
        collection="visual_vehicles",
        derived_from=derived_from,
    )
    if cargo_length > 0.0:
        cargo_x = body_length / 2.0 - cargo_length / 2.0 - 0.15
        _visual_box(
            state,
            stable_id=_realism_id(state, root, "cargo_box"),
            semantic_class="parked_truck_cargo",
            position_m=_world_from_pose(position_m, yaw_deg, (cargo_x, 0.0, 2.0)),
            yaw_deg=yaw_deg,
            dimensions_m=(cargo_length, body_width, 3.05),
            material="warehouse_cladding",
            collection="visual_vehicles",
            derived_from=derived_from,
        )
    wheel_xs = (-body_length * 0.31, body_length * 0.31)
    for axle_index, wheel_x in enumerate(wheel_xs):
        for side_index, wheel_y in enumerate((-body_width * 0.51, body_width * 0.51)):
            _visual_cylinder(
                state,
                stable_id=_realism_id(state, root, f"wheel_{axle_index}", side_index),
                semantic_class="vehicle_wheel",
                position_m=_world_from_pose(position_m, yaw_deg, (wheel_x, wheel_y, 0.48)),
                yaw_deg=yaw_deg,
                pitch_deg=90.0,
                radius_m=0.42 if kind != "forklift" else 0.28,
                height_m=0.24,
                material="rubber",
                collection="visual_vehicles",
                derived_from=derived_from,
            )
    if kind == "forklift":
        mast_x = body_length / 2.0 + 0.1
        for index, y in enumerate((-0.5, 0.5)):
            _visual_box(
                state,
                stable_id=_realism_id(state, root, "fork_mast", index),
                semantic_class="forklift_mast",
                position_m=_world_from_pose(position_m, yaw_deg, (mast_x, y, 1.45)),
                yaw_deg=yaw_deg,
                dimensions_m=(0.12, 0.12, 2.6),
                material="safety_black",
                collection="visual_vehicles",
                derived_from=derived_from,
            )
            _visual_box(
                state,
                stable_id=_realism_id(state, root, "fork_tine", index),
                semantic_class="forklift_tine",
                position_m=_world_from_pose(position_m, yaw_deg, (mast_x + 1.0, y, 0.2)),
                yaw_deg=yaw_deg,
                dimensions_m=(2.0, 0.1, 0.1),
                material="galvanized",
                collection="visual_vehicles",
                derived_from=derived_from,
            )


def _add_pallet(
    state: BuildState,
    *,
    pallet_id: str,
    position_m: Sequence[float],
    yaw_deg: float,
    derived_from: str,
) -> None:
    for index, local_y in enumerate((-0.42, 0.0, 0.42)):
        _visual_box(
            state,
            stable_id=_realism_id(state, pallet_id, "deck_board", index),
            semantic_class="wood_pallet",
            position_m=_world_from_pose(position_m, yaw_deg, (0.0, local_y, 0.18)),
            yaw_deg=yaw_deg,
            dimensions_m=(1.25, 0.18, 0.11),
            material="pallet_wood",
            collection="visual_vehicles",
            derived_from=derived_from,
        )
    for index, local_x in enumerate((-0.43, 0.43)):
        _visual_box(
            state,
            stable_id=_realism_id(state, pallet_id, "runner", index),
            semantic_class="wood_pallet",
            position_m=_world_from_pose(position_m, yaw_deg, (local_x, 0.0, 0.08)),
            yaw_deg=yaw_deg,
            dimensions_m=(0.12, 1.05, 0.15),
            material="pallet_wood",
            collection="visual_vehicles",
            derived_from=derived_from,
        )


def _add_industrial_clutter_realism_v2(state: BuildState, objects: Sequence[LayoutObject]) -> None:
    by_id = {item.stable_id: item for item in objects}
    loading_source = by_id.get("loading_dock_01")
    container_source = by_id.get("container_yard_apron")
    parking_source = by_id.get("parking_apron")
    fallback = "layout.extent_m"
    vehicles = (
        ("dock_truck_01", (30.0, 7.0, 0.0), 90.0, "truck", loading_source),
        ("dock_truck_02", (70.0, 7.0, 0.0), 90.0, "truck", by_id.get("loading_dock_03")),
        ("parking_van", (37.0, -44.0, 0.0), 90.0, "van", parking_source),
        ("yard_forklift", (-24.0, -39.0, 0.0), 0.0, "forklift", container_source),
    )
    for vehicle_id, position, yaw, kind, source in vehicles:
        _add_vehicle_silhouette(
            state,
            vehicle_id=vehicle_id,
            position_m=position,
            yaw_deg=yaw,
            kind=kind,
            derived_from=source.stable_id if source else fallback,
        )

    pallet_positions = (
        (23.0, 14.0),
        (24.6, 14.0),
        (25.8, 14.0),
        (42.0, 14.0),
        (43.4, 14.0),
        (44.8, 14.0),
        (62.0, 14.0),
        (63.4, 14.0),
        (64.8, 14.0),
        (-24.0, -29.0),
        (-22.4, -29.0),
        (-20.8, -29.0),
        (-78.0, -38.0),
        (-76.4, -38.0),
        (-74.8, -38.0),
    )
    for index, (x, y) in enumerate(pallet_positions):
        source = loading_source if x > 0 else container_source
        _add_pallet(
            state,
            pallet_id=f"pallet_cluster_{index:02d}",
            position_m=(x, y, 0.0),
            yaw_deg=90.0 if x > 0 else 0.0,
            derived_from=source.stable_id if source else fallback,
        )
        if index % 2 == 0:
            _visual_box(
                state,
                stable_id=_realism_id(state, f"pallet_cluster_{index:02d}", "crate"),
                semantic_class="shipping_crate",
                position_m=(x, y, 0.72),
                yaw_deg=0.0,
                dimensions_m=(1.05, 0.82, 1.15),
                material="crate_tan",
                collection="visual_vehicles",
                derived_from=source.stable_id if source else fallback,
            )
    cone_positions = (
        (-4.4, -74.0),
        (4.4, -74.0),
        (-4.4, -70.0),
        (4.4, -70.0),
        (24.0, 9.0),
        (26.0, 9.0),
        (44.0, 9.0),
        (46.0, 9.0),
        (64.0, 9.0),
        (66.0, 9.0),
        (-20.0, -22.0),
        (-22.0, -22.0),
    )
    for index, (x, y) in enumerate(cone_positions):
        source = loading_source if y > 0 else by_id.get("road_entry_boulevard")
        _visual_cylinder(
            state,
            stable_id=_realism_id(state, "traffic_cones", "base", index),
            semantic_class="traffic_cone",
            position_m=(x, y, 0.04),
            yaw_deg=0.0,
            radius_m=0.24,
            height_m=0.08,
            material="safety_black",
            collection="visual_vehicles",
            derived_from=source.stable_id if source else fallback,
        )
        _visual_cylinder(
            state,
            stable_id=_realism_id(state, "traffic_cones", "cone", index),
            semantic_class="traffic_cone",
            position_m=(x, y, 0.36),
            yaw_deg=0.0,
            radius_m=0.16,
            top_radius_m=0.035,
            height_m=0.64,
            material="traffic_orange",
            collection="visual_vehicles",
            derived_from=source.stable_id if source else fallback,
        )
    for index in range(12):
        cluster_x = -82.0 + (index % 4) * 1.1
        cluster_y = -21.0 + (index // 4) * 1.15
        _visual_cylinder(
            state,
            stable_id=_realism_id(state, "container_yard", "empty_drum", index),
            semantic_class="empty_drum",
            position_m=(cluster_x, cluster_y, 0.45),
            yaw_deg=0.0,
            radius_m=0.29,
            height_m=0.9,
            material="barrel_blue" if index % 3 else "safety_red",
            collection="visual_vehicles",
            derived_from=container_source.stable_id if container_source else fallback,
        )
    for index in range(8):
        x = -84.0 + (index % 4) * 2.0
        y = 15.0 + (index // 4) * 2.0
        _visual_box(
            state,
            stable_id=_realism_id(state, "factory_service", "waste_bin", index),
            semantic_class="industrial_waste_bin",
            position_m=(x, y, 0.65),
            yaw_deg=0.0,
            dimensions_m=(1.25, 0.8, 1.3),
            material="transformer_dark",
            collection="visual_vehicles",
            derived_from=by_id.get("main_factory", loading_source).stable_id
            if by_id.get("main_factory", loading_source)
            else fallback,
        )


def _add_utility_yard_realism_v2(state: BuildState, objects: Sequence[LayoutObject]) -> None:
    source = next(
        (item for item in objects if item.stable_id == "administration_office"),
        None,
    )
    derived_from = source.stable_id if source else "layout.extent_m"
    center = (69.0, 76.0)
    _visual_box(
        state,
        stable_id=_realism_id(state, "utility_yard", "foundation"),
        semantic_class="utility_yard_foundation",
        position_m=(center[0], center[1], 0.09),
        yaw_deg=0.0,
        dimensions_m=(16.0, 12.0, 0.18),
        material="concrete_dirty",
        collection="visual_utilities",
        derived_from=derived_from,
    )
    for transformer_index, x in enumerate((64.8, 69.0, 73.2)):
        _visual_box(
            state,
            stable_id=_realism_id(state, "utility_yard", "transformer_body", transformer_index),
            semantic_class="power_transformer",
            position_m=(x, center[1], 1.25),
            yaw_deg=0.0,
            dimensions_m=(2.6, 2.1, 2.5),
            material="transformer_dark",
            collection="visual_utilities",
            derived_from=derived_from,
        )
        _visual_box(
            state,
            stable_id=_realism_id(state, "utility_yard", "transformer_top", transformer_index),
            semantic_class="power_transformer",
            position_m=(x, center[1], 2.65),
            yaw_deg=0.0,
            dimensions_m=(2.9, 2.35, 0.28),
            material="galvanized",
            collection="visual_utilities",
            derived_from=derived_from,
        )
        for radiator_index, y_offset in enumerate((-1.15, 1.15)):
            _visual_box(
                state,
                stable_id=_realism_id(
                    state,
                    f"utility_yard_transformer_{transformer_index}",
                    "radiator",
                    radiator_index,
                ),
                semantic_class="transformer_radiator",
                position_m=(x, center[1] + y_offset, 1.35),
                yaw_deg=0.0,
                dimensions_m=(2.2, 0.18, 1.7),
                material="galvanized",
                collection="visual_utilities",
                derived_from=derived_from,
            )
        for insulator_index in range(4):
            _visual_cylinder(
                state,
                stable_id=_realism_id(
                    state,
                    f"utility_yard_transformer_{transformer_index}",
                    "insulator",
                    insulator_index,
                ),
                semantic_class="transformer_insulator",
                position_m=(x - 0.9 + insulator_index * 0.6, center[1], 3.08),
                yaw_deg=0.0,
                radius_m=0.12,
                top_radius_m=0.08,
                height_m=0.6,
                material="ceramic",
                collection="visual_utilities",
                derived_from=derived_from,
            )
    for cabinet_index in range(5):
        x = center[0] - 5.4 + cabinet_index * 2.7
        _visual_box(
            state,
            stable_id=_realism_id(state, "utility_yard", "electrical_cabinet", cabinet_index),
            semantic_class="electrical_cabinet",
            position_m=(x, center[1] - 4.4, 0.95),
            yaw_deg=0.0,
            dimensions_m=(1.35, 0.65, 1.9),
            material="galvanized",
            collection="visual_utilities",
            derived_from=derived_from,
        )
        _visual_box(
            state,
            stable_id=_realism_id(state, "utility_yard", "cabinet_hazard_panel", cabinet_index),
            semantic_class="electrical_hazard_panel",
            position_m=(x, center[1] - 4.74, 1.05),
            yaw_deg=0.0,
            dimensions_m=(0.42, 0.025, 0.42),
            material="safety_yellow",
            collection="visual_utilities",
            derived_from=derived_from,
        )
    fence_points: list[tuple[float, float]] = []
    for index in range(7):
        fraction = index / 6.0
        fence_points.append((center[0] - 8.0 + 16.0 * fraction, center[1] - 6.0))
        fence_points.append((center[0] - 8.0 + 16.0 * fraction, center[1] + 6.0))
    for index in range(5):
        fraction = index / 4.0
        fence_points.append((center[0] - 8.0, center[1] - 6.0 + 12.0 * fraction))
        fence_points.append((center[0] + 8.0, center[1] - 6.0 + 12.0 * fraction))
    for index, (x, y) in enumerate(fence_points):
        _visual_box(
            state,
            stable_id=_realism_id(state, "utility_yard", "fence_post", index),
            semantic_class="utility_fence_post",
            position_m=(x, y, 1.15),
            yaw_deg=0.0,
            dimensions_m=(0.11, 0.11, 2.3),
            material="galvanized",
            collection="visual_utilities",
            derived_from=derived_from,
        )
    for index, (position, dimensions) in enumerate(
        (
            ((center[0], center[1] - 6.0, 0.8), (16.0, 0.08, 0.08)),
            ((center[0], center[1] - 6.0, 1.65), (16.0, 0.08, 0.08)),
            ((center[0], center[1] + 6.0, 0.8), (16.0, 0.08, 0.08)),
            ((center[0], center[1] + 6.0, 1.65), (16.0, 0.08, 0.08)),
            ((center[0] - 8.0, center[1], 0.8), (0.08, 12.0, 0.08)),
            ((center[0] + 8.0, center[1], 0.8), (0.08, 12.0, 0.08)),
        )
    ):
        _visual_box(
            state,
            stable_id=_realism_id(state, "utility_yard", "fence_rail", index),
            semantic_class="utility_fence_rail",
            position_m=position,
            yaw_deg=0.0,
            dimensions_m=dimensions,
            material="galvanized",
            collection="visual_utilities",
            derived_from=derived_from,
        )


def _add_landscape_realism_v2(
    state: BuildState,
    bounds: Mapping[str, Sequence[float]],
    objects: Sequence[LayoutObject],
) -> None:
    derived_from = "layout.extent_m"
    _visual_box(
        state,
        stable_id=_realism_id(state, "site", "distant_ground"),
        semantic_class="distant_ground",
        position_m=(0.0, 0.0, -0.42),
        yaw_deg=0.0,
        dimensions_m=(900.0, 760.0, 0.7),
        material="terrain",
        collection="visual_landscape",
        derived_from=derived_from,
    )
    backdrop_masses = (
        (-168.0, -72.0, 6.0, (48.0, 16.0, 12.0)),
        (-160.0, 58.0, 7.0, (62.0, 18.0, 14.0)),
        (166.0, -62.0, 5.0, (46.0, 15.0, 10.0)),
        (170.0, 67.0, 6.5, (58.0, 17.0, 13.0)),
        (0.0, 146.0, 5.5, (92.0, 14.0, 11.0)),
    )
    for index, (x, y, z, dimensions) in enumerate(backdrop_masses):
        _visual_box(
            state,
            stable_id=_realism_id(state, "site", "distant_industrial_mass", index),
            semantic_class="distant_industrial_backdrop",
            position_m=(x, y, z),
            yaw_deg=0.0,
            dimensions_m=dimensions,
            material="warehouse_cladding",
            collection="visual_landscape",
            derived_from=derived_from,
        )
    for index in range(18):
        side = -1.0 if index % 2 == 0 else 1.0
        x = side * stable_variation("v2_tree", index, low=98.0, high=106.0, seed=state.realism.seed)
        y = stable_variation("v2_tree_y", index, low=-74.0, high=78.0, seed=state.realism.seed)
        trunk_height = stable_variation("v2_tree_h", index, low=2.4, high=3.6, seed=state.realism.seed)
        _visual_cylinder(
            state,
            stable_id=_realism_id(state, "site_trees", "trunk", index),
            semantic_class="tree_trunk",
            position_m=(x, y, trunk_height / 2.0),
            yaw_deg=0.0,
            radius_m=0.14,
            height_m=trunk_height,
            material="vegetation_trunk",
            collection="visual_landscape",
            derived_from=derived_from,
        )
        _visual_ellipsoid(
            state,
            stable_id=_realism_id(state, "site_trees", "canopy", index),
            semantic_class="tree_canopy",
            position_m=(x, y, trunk_height + 1.15),
            radii_m=(1.0, 0.82, 1.25),
            material="vegetation_leaf" if index % 4 else "foliage_dry",
            collection="visual_landscape",
            derived_from=derived_from,
        )
    shrub_anchors = (
        (-102.0, -72.0),
        (-102.0, -38.0),
        (-102.0, 24.0),
        (-102.0, 62.0),
        (101.0, -75.0),
        (101.0, -8.0),
        (101.0, 35.0),
        (101.0, 70.0),
        (10.0, 69.0),
        (34.0, 69.0),
        (50.0, 68.0),
        (82.0, 68.0),
    )
    shrub_index = 0
    for anchor_x, anchor_y in shrub_anchors:
        for cluster_index in range(2):
            x = anchor_x + stable_variation("shrub_x", shrub_index, low=-1.3, high=1.3, seed=state.realism.seed)
            y = anchor_y + stable_variation("shrub_y", shrub_index, low=-1.3, high=1.3, seed=state.realism.seed)
            _visual_ellipsoid(
                state,
                stable_id=_realism_id(state, "site_shrubs", "bush", shrub_index),
                semantic_class="landscape_shrub",
                position_m=(x, y, 0.55),
                radii_m=(0.72, 0.58, 0.62),
                material="vegetation_leaf" if cluster_index == 0 else "foliage_dry",
                collection="visual_landscape",
                derived_from=derived_from,
            )
            shrub_index += 1
    drainage_source = next((item for item in objects if item.semantic_class == "drainage_ditch"), None)
    drainage_derived = drainage_source.stable_id if drainage_source else derived_from
    for index in range(48):
        y = -53.0 + index * 106.0 / 47.0
        x = 84.0 + (index % 3) * 0.45
        height = stable_variation("drain_reed", index, low=0.35, high=0.95, seed=state.realism.seed)
        _visual_box(
            state,
            stable_id=_realism_id(state, "east_drainage", "reed", index),
            semantic_class="drainage_reed",
            position_m=(x, y, height / 2.0 - 0.06),
            yaw_deg=stable_variation("drain_reed_yaw", index, low=-18.0, high=18.0, seed=state.realism.seed),
            dimensions_m=(0.04, 0.04, height),
            material="foliage_dry" if index % 4 == 0 else "grass",
            collection="visual_landscape",
            derived_from=drainage_derived,
        )
    gravel_source = next((item for item in objects if item.semantic_class == "gravel_test_area"), None)
    gravel_derived = gravel_source.stable_id if gravel_source else derived_from
    for index in range(32):
        x = stable_variation("rough_rock_x", index, low=-88.0, high=-64.0, seed=state.realism.seed)
        y = stable_variation("rough_rock_y", index, low=71.0, high=81.0, seed=state.realism.seed)
        radius = stable_variation("rough_rock_r", index, low=0.16, high=0.65, seed=state.realism.seed)
        _visual_ellipsoid(
            state,
            stable_id=_realism_id(state, "gravel_test_area", "rough_rock", index),
            semantic_class="rough_pad_rock",
            position_m=(x, y, radius * 0.58),
            radii_m=(radius, radius * 0.75, radius * 0.58),
            material="rock",
            collection="visual_landscape",
            derived_from=gravel_derived,
        )


def _decorate_layout_objects(state: BuildState, objects: Sequence[LayoutObject]) -> None:
    semantic_classes = {item.semantic_class for item in objects}
    for item in objects:
        if item.semantic_class in {
            "building",
            "guardhouse",
            "main_factory_building",
            "warehouse_building",
            "office_building",
        }:
            _decorate_building(state, item)
            _decorate_building_realism_v2(state, item)
        elif item.semantic_class == "road" and "lane_marking" not in semantic_classes:
            _decorate_road(state, item)
        elif item.semantic_class == "parking_space":
            _decorate_parking_space(state, item)
        elif item.semantic_class == "storage_tank":
            _decorate_storage_tank(state, item)
            _decorate_tank_realism_v2(state, item)
        elif item.semantic_class in {"container", "shipping_container"}:
            _decorate_container(state, item)
        elif item.semantic_class == "boundary_fence":
            _decorate_fence(state, item)
        elif item.semantic_class == "speed_bump":
            _decorate_speed_bump(state, item)
        if item.semantic_class in {"road", "parking_area", "container_yard"}:
            _decorate_road_surface_realism_v2(state, item)
        if item.semantic_class in {"loading_dock", "loading_ramp"}:
            _decorate_loading_realism_v2(state, item)
        _decorate_drainage_realism_v2(state, item)
    _add_south_gate_realism_v2(state, objects)
    _add_tank_utilities_realism_v2(state, objects)
    _add_industrial_clutter_realism_v2(state, objects)
    _add_utility_yard_realism_v2(state, objects)


def _record_path_value(value: object) -> str | None:
    if isinstance(value, str):
        return value
    if isinstance(value, Mapping):
        candidate = value.get("path")
        return candidate if isinstance(candidate, str) else None
    return None


def _resolve_source_path(state: BuildState, value: object) -> Path:
    raw = _record_path_value(value)
    if not raw:
        raise ValueError("terrain.terrain_obj must be a path or path record")
    candidate = Path(raw)
    if candidate.is_absolute():
        return candidate.resolve()
    from_repo = (state.repo_root / candidate).resolve()
    if from_repo.is_file():
        return from_repo
    return (state.layout_path.parent / candidate).resolve()


def _import_terrain(state: BuildState) -> None:
    blender = _require_bpy()
    import bmesh  # type: ignore[import-not-found]

    terrain = state.layout.get("terrain")
    if not isinstance(terrain, Mapping):
        raise ValueError("expanded layout must include terrain metadata")
    terrain_path = _resolve_source_path(state, terrain.get("terrain_obj"))
    if not terrain_path.is_file():
        raise FileNotFoundError(f"authoritative terrain OBJ not found: {terrain_path}")
    before = set(blender.data.objects)
    if hasattr(blender.ops.wm, "obj_import"):
        blender.ops.wm.obj_import(
            filepath=str(terrain_path),
            forward_axis="NEGATIVE_Y",
            up_axis="Z",
        )
    else:  # pragma: no cover - retained for older Blender authoring environments.
        blender.ops.import_scene.obj(filepath=str(terrain_path), axis_forward="-Y", axis_up="Z")
    imported = sorted(set(blender.data.objects) - before, key=lambda item: item.name)
    mesh_objects = [item for item in imported if item.type == "MESH"]
    if not mesh_objects:
        raise RuntimeError(f"terrain OBJ imported no mesh: {terrain_path}")
    obj_frame = str(terrain.get("obj_frame", "")).strip().lower()
    source_frame = str(terrain.get("source_frame", "")).strip().lower()
    if obj_frame == "unreal_lh_z_up_cm":
        for obj in mesh_objects:
            for vertex in obj.data.vertices:
                vertex.co.x *= 0.01
                vertex.co.y *= -0.01
                vertex.co.z *= 0.01
            editable = bmesh.new()
            editable.from_mesh(obj.data)
            bmesh.ops.reverse_faces(editable, faces=list(editable.faces))
            editable.to_mesh(obj.data)
            editable.free()
            obj.data.update()
        vertex_conversion = "(x_cm,y_cm,z_cm)->(x_cm/100,-y_cm/100,z_cm/100)"
        winding_reversed = True
    elif obj_frame in _COORDINATE_ALIASES or source_frame in _COORDINATE_ALIASES:
        vertex_conversion = "identity"
        winding_reversed = False
    else:
        raise ValueError(f"unsupported terrain OBJ frame: {terrain.get('obj_frame')!r}")

    coordinates = [tuple(vertex.co) for obj in mesh_objects for vertex in obj.data.vertices]
    converted_bounds = {
        "min": [min(point[axis] for point in coordinates) for axis in range(3)],
        "max": [max(point[axis] for point in coordinates) for axis in range(3)],
    }
    converted_extent = [converted_bounds["max"][axis] - converted_bounds["min"][axis] for axis in range(3)]
    expected_extent = _vector(
        terrain.get("extent_m", state.layout.get("extent_m")),
        field_name="terrain.extent_m",
        lengths=(2, 3),
    )
    if not math.isclose(converted_extent[0], expected_extent[0], rel_tol=0.0, abs_tol=1e-3):
        raise ValueError(f"terrain X extent is {converted_extent[0]:.6f} m; expected {expected_extent[0]:.6f} m")
    if not math.isclose(converted_extent[1], expected_extent[1], rel_tol=0.0, abs_tol=1e-3):
        raise ValueError(f"terrain Y extent is {converted_extent[1]:.6f} m; expected {expected_extent[1]:.6f} m")
    for index, obj in enumerate(mesh_objects):
        for collection in list(obj.users_collection):
            collection.objects.unlink(obj)
        state.collections["layout_terrain"].objects.link(obj)
        stable_id = (
            "terrain/authoritative_heightfield"
            if len(mesh_objects) == 1
            else f"terrain/authoritative_heightfield/{index:02d}"
        )
        name = _unique_object_name(state, stable_id)
        obj.name = name
        obj.data.name = name
        obj.location = (0.0, 0.0, 0.0)
        obj.rotation_euler = (0.0, 0.0, 0.0)
        obj.scale = (1.0, 1.0, 1.0)
        if not obj.data.materials:
            obj.data.materials.append(_material_for(state, "terrain"))
        obj["lingtu_stable_id"] = stable_id
        obj["lingtu_semantic_class"] = "terrain"
        obj["lingtu_material"] = "terrain"
        obj["lingtu_source"] = "expanded_layout.terrain.terrain_obj"
        obj["lingtu_collision"] = True
        obj["lingtu_visual_only"] = False
        obj["lingtu_physics_proxy"] = "authoritative_heightfield"
        obj["lingtu_source_frame"] = "mujoco_rh_z_up_m"
        record = {
            "stable_id": stable_id,
            "asset_key": "terrain/authoritative_heightfield",
            "semantic_class": "terrain",
            "blender_object": name,
            "mesh_name": name,
            "shape": "mesh",
            "position_m": [0.0, 0.0, 0.0],
            "yaw_deg": 0.0,
            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            "scale": [1.0, 1.0, 1.0],
            "material": "terrain",
            "collision": True,
            "visual_only": False,
            "physics_proxy": "authoritative_heightfield",
            "source": "expanded_layout.terrain.terrain_obj",
            "source_path": str(terrain_path),
            "source_sha256": _sha256(terrain_path.read_bytes()),
            "source_obj_frame": terrain.get("obj_frame"),
            "converted_frame": "blender_rh_z_up_m",
            "vertex_conversion": vertex_conversion,
            "winding_reversed": winding_reversed,
            "converted_bounds_m": converted_bounds,
            "converted_extent_m": converted_extent,
            "expected_extent_m": list(expected_extent),
            "extent_alignment_verified": True,
            "vertex_count": len(obj.data.vertices),
            "polygon_count": len(obj.data.polygons),
        }
        state.terrain_records.append(record)


def _add_streetlights(state: BuildState, roads: Sequence[LayoutObject]) -> None:
    candidates = sorted(roads, key=lambda item: max(item.dimensions_m[:2]), reverse=True)[:3]
    for road_index, road in enumerate(candidates):
        sx, sy, sz = road.dimensions_m
        long_x = sx >= sy
        length = sx if long_x else sy
        width = sy if long_x else sx
        lamp_count = max(2, min(4, int(length // 32.0) + 1))
        for lamp_index in range(lamp_count):
            along = (lamp_index - (lamp_count - 1) / 2.0) * min(30.0, length / max(lamp_count, 1))
            side = -1.0 if lamp_index % 2 == 0 else 1.0
            lateral = side * (width / 2.0 + 1.2)
            local_xy = (along, lateral) if long_x else (lateral, along)
            base = _local_to_world(road, (local_xy[0], local_xy[1], sz / 2.0))
            root = f"visual/{road.safe_id}/streetlight_{road_index:02d}_{lamp_index:02d}"
            _visual_cylinder(
                state,
                stable_id=f"{root}/pole",
                semantic_class="streetlight_pole",
                position_m=(base[0], base[1], base[2] + 3.0),
                yaw_deg=road.yaw_deg,
                radius_m=0.085,
                height_m=6.0,
                material="steel",
                collection="visual_furniture",
                derived_from=road.stable_id,
            )
            arm_local = (local_xy[0], local_xy[1] - side * 0.65) if long_x else (local_xy[0] - side * 0.65, local_xy[1])
            arm_pos = _local_to_world(road, (arm_local[0], arm_local[1], sz / 2.0 + 5.85))
            arm_dims = (0.12, 1.3, 0.12) if long_x else (1.3, 0.12, 0.12)
            _visual_box(
                state,
                stable_id=f"{root}/arm",
                semantic_class="streetlight_arm",
                position_m=arm_pos,
                yaw_deg=road.yaw_deg,
                dimensions_m=arm_dims,
                material="steel",
                collection="visual_furniture",
                derived_from=road.stable_id,
            )
            lamp_local = (
                (local_xy[0], local_xy[1] - side * 1.22) if long_x else (local_xy[0] - side * 1.22, local_xy[1])
            )
            lamp_pos = _local_to_world(road, (lamp_local[0], lamp_local[1], sz / 2.0 + 5.77))
            lamp_dims = (0.48, 0.22, 0.14) if long_x else (0.22, 0.48, 0.14)
            _visual_box(
                state,
                stable_id=f"{root}/luminaire",
                semantic_class="streetlight_luminaire",
                position_m=lamp_pos,
                yaw_deg=road.yaw_deg,
                dimensions_m=lamp_dims,
                material="lamp_emissive",
                collection="visual_furniture",
                derived_from=road.stable_id,
            )


def _layout_extent(layout: Mapping[str, Any], bounds: Mapping[str, Sequence[float]]) -> tuple[float, float]:
    raw_extent = layout.get("extent_m")
    if isinstance(raw_extent, Sequence) and not isinstance(raw_extent, (str, bytes)) and len(raw_extent) >= 2:
        return (
            _finite_number(raw_extent[0], field_name="extent_m[0]"),
            _finite_number(raw_extent[1], field_name="extent_m[1]"),
        )
    return (
        float(bounds["max"][0] - bounds["min"][0]),
        float(bounds["max"][1] - bounds["min"][1]),
    )


def _add_landscaping(state: BuildState, bounds: Mapping[str, Sequence[float]]) -> None:
    extent_x, extent_y = _layout_extent(state.layout, bounds)
    center_x = (bounds["min"][0] + bounds["max"][0]) / 2.0
    center_y = (bounds["min"][1] + bounds["max"][1]) / 2.0
    margin_x = max(6.0, extent_x / 2.0 - 6.0)
    grass_width = max(2.5, min(5.0, extent_x * 0.025))
    for index, (position, dimensions) in enumerate(
        (
            ((center_x - margin_x, center_y, 0.03), (grass_width, max(8.0, extent_y - 18.0), 0.055)),
            ((center_x + margin_x, center_y, 0.03), (grass_width, max(8.0, extent_y - 18.0), 0.055)),
        )
    ):
        _visual_box(
            state,
            stable_id=f"visual/site/landscape/grass_strip_{index:02d}",
            semantic_class="landscape_grass",
            position_m=position,
            yaw_deg=0.0,
            dimensions_m=dimensions,
            material="grass",
            collection="visual_landscape",
            derived_from="layout.extent_m",
        )
    tree_points: list[tuple[float, float]] = []
    for index in range(5):
        fraction = (index + 1) / 6.0
        tree_points.append((center_x - margin_x, center_y - extent_y * 0.4 + fraction * extent_y * 0.8))
        tree_points.append((center_x + margin_x, center_y - extent_y * 0.4 + fraction * extent_y * 0.8))
    for index, (x, y) in enumerate(tree_points):
        root = f"visual/site/landscape/tree_{index:02d}"
        trunk_height = 2.4 + (index % 3) * 0.25
        _visual_cylinder(
            state,
            stable_id=f"{root}/trunk",
            semantic_class="tree_trunk",
            position_m=(x, y, trunk_height / 2.0),
            yaw_deg=0.0,
            radius_m=0.16,
            height_m=trunk_height,
            material="vegetation_trunk",
            collection="visual_landscape",
            derived_from="layout.extent_m",
        )
        _visual_ellipsoid(
            state,
            stable_id=f"{root}/canopy",
            semantic_class="tree_canopy",
            position_m=(x, y, trunk_height + 1.15),
            radii_m=(1.0 + 0.1 * (index % 2), 0.9, 1.3),
            material="vegetation_leaf",
            collection="visual_landscape",
            derived_from="layout.extent_m",
        )


def _add_camera(
    name: str,
    collection: Any,
    location: Sequence[float],
    target: Sequence[float],
    *,
    lens_mm: float,
    orthographic_scale: float | None = None,
) -> Any:
    blender = _require_bpy()
    from mathutils import Vector  # type: ignore[import-not-found]

    camera_data = blender.data.cameras.new(name=f"{name}_Data")
    camera = blender.data.objects.new(name=name, object_data=camera_data)
    collection.objects.link(camera)
    camera.location = tuple(location)
    direction = Vector(target) - Vector(location)
    camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    camera_data.lens = lens_mm
    camera_data.sensor_width = 36.0
    camera_data.clip_start = 0.08
    camera_data.clip_end = 2500.0
    if orthographic_scale is not None:
        camera_data.type = "ORTHO"
        camera_data.ortho_scale = orthographic_scale
    return camera


def _spawn_pose(
    layout: Mapping[str, Any], bounds: Mapping[str, Sequence[float]]
) -> tuple[tuple[float, float, float], float]:
    spawn = layout.get("spawn")
    if isinstance(spawn, Mapping):
        raw_position = spawn.get("position_m", spawn.get("position"))
        try:
            position = _vector(raw_position, field_name="spawn.position_m")
            if "yaw_deg" in spawn:
                yaw = _finite_number(spawn["yaw_deg"], field_name="spawn.yaw_deg")
            else:
                center_x = (bounds["min"][0] + bounds["max"][0]) / 2.0
                center_y = (bounds["min"][1] + bounds["max"][1]) / 2.0
                yaw = math.degrees(math.atan2(center_y - position[1], center_x - position[0]))
            return (position[0], position[1], position[2]), yaw
        except (TypeError, ValueError):
            pass
    center_x = (bounds["min"][0] + bounds["max"][0]) / 2.0
    return (center_x, bounds["min"][1] + 8.0, max(0.0, bounds["min"][2])), 90.0


def _setup_lighting_and_cameras(
    state: BuildState,
    bounds: Mapping[str, Sequence[float]],
    *,
    width: int,
    height: int,
    samples: int,
) -> dict[str, Any]:
    blender = _require_bpy()
    scene = blender.context.scene
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.film_transparent = False
    renderer = str(state.realism.raw.get("lighting", {}).get("blender", {}).get("renderer", "BLENDER_EEVEE_NEXT"))
    try:
        scene.render.engine = renderer
    except TypeError:
        scene.render.engine = "BLENDER_EEVEE"
    if hasattr(scene, "eevee"):
        scene.eevee.taa_samples = samples
    try:
        scene.view_settings.look = "AgX - Medium High Contrast"
    except TypeError:
        pass
    scene.view_settings.exposure = -0.72

    world = blender.data.worlds.new("FactoryPark_Daylight")
    world.use_nodes = True
    scene.world = world
    nodes = world.node_tree.nodes
    links = world.node_tree.links
    background = next(node for node in nodes if node.type == "BACKGROUND")
    lighting = state.realism.raw.get("lighting", {})
    blender_lighting = lighting.get("blender", {}) if isinstance(lighting, Mapping) else {}
    world_strength = float(blender_lighting.get("world_strength", 0.35))
    background.inputs["Strength"].default_value = min(world_strength, 0.26)
    sky = nodes.new("ShaderNodeTexSky")
    try:
        sky.sky_type = "NISHITA"
    except TypeError:
        sky.sky_type = "MULTIPLE_SCATTERING"
    if hasattr(sky, "sun_elevation"):
        sky.sun_elevation = math.radians(42.0)
    if hasattr(sky, "sun_rotation"):
        sky.sun_rotation = math.radians(138.0)
    if hasattr(sky, "altitude"):
        sky.altitude = 0.15
    links.new(sky.outputs["Color"], background.inputs["Color"])

    sun_data = blender.data.lights.new(name="Sun_Data", type="SUN")
    sun_data.energy = 2.25
    sun_data.angle = math.radians(float(blender_lighting.get("sun_angle_deg", 0.535)))
    if hasattr(sun_data, "use_shadow"):
        sun_data.use_shadow = True
    if hasattr(sun_data, "use_contact_shadow"):
        sun_data.use_contact_shadow = True
    sun = blender.data.objects.new(name="Sun", object_data=sun_data)
    state.collections["lighting"].objects.link(sun)
    sun.rotation_euler = (math.radians(27.0), math.radians(-18.0), math.radians(132.0))

    for index, (location, energy, size) in enumerate(
        (
            ((-62.0, -28.0, 28.0), 280.0, 18.0),
            ((72.0, 18.0, 24.0), 210.0, 14.0),
        )
    ):
        area_data = blender.data.lights.new(name=f"Industrial_Fill_{index}_Data", type="AREA")
        area_data.energy = energy
        area_data.shape = "DISK"
        area_data.size = size
        if hasattr(area_data, "use_shadow"):
            area_data.use_shadow = True
        if hasattr(area_data, "use_contact_shadow"):
            area_data.use_contact_shadow = True
        area = blender.data.objects.new(name=f"Industrial_Fill_{index}", object_data=area_data)
        state.collections["lighting"].objects.link(area)
        area.location = location
        from mathutils import Vector  # type: ignore[import-not-found]

        area.rotation_euler = (Vector((0.0, 0.0, 1.5)) - Vector(location)).to_track_quat("-Z", "Y").to_euler()

    center = tuple((bounds["min"][axis] + bounds["max"][axis]) / 2.0 for axis in range(3))
    span_x = bounds["max"][0] - bounds["min"][0]
    span_y = bounds["max"][1] - bounds["min"][1]
    span = max(span_x, span_y, 40.0)
    aerial_location = (center[0] - 0.42 * span, center[1] - 0.62 * span, max(55.0, center[2] + 0.92 * span))
    aerial_target = (center[0], center[1], max(1.5, center[2] * 0.35))
    spawn_position, spawn_yaw = _spawn_pose(state.layout, bounds)
    eye = (spawn_position[0], spawn_position[1], spawn_position[2] + 1.55)
    heading = math.radians(spawn_yaw)
    robot_target = (
        eye[0] + math.cos(heading) * 24.0,
        eye[1] + math.sin(heading) * 24.0,
        eye[2] + 0.8,
    )
    defaults: dict[str, dict[str, Any]] = {
        "site_aerial": {
            "recipe_target": "site_aerial",
            "camera": "Camera_Site_Aerial",
            "position_m": list(aerial_location),
            "look_at_m": list(aerial_target),
            "lens_mm": 46.0,
            "output_basename": "factory_park_hf.site-aerial-v2.png",
        },
        "robot_eye": {
            "recipe_target": "south_gate_robot_eye",
            "camera": "Camera_Robot_Eye",
            "position_m": list(eye),
            "look_at_m": list(robot_target),
            "lens_mm": 26.0,
            "output_basename": "factory_park_hf.south-gate-robot-eye-v2.png",
        },
        "loading_dock": {
            "recipe_target": "loading_dock_robot_eye",
            "camera": "Camera_Loading_Dock",
            "position_m": [49.0, 4.0, 0.78],
            "look_at_m": [50.0, 19.0, 2.2],
            "lens_mm": 28.0,
            "output_basename": "factory_park_hf.loading-dock-v2.png",
        },
        "tank_area": {
            "recipe_target": "tank_farm_inspection",
            "camera": "Camera_Tank_Area",
            "position_m": [89.0, -36.0, 1.55],
            "look_at_m": [65.0, -36.0, 3.2],
            "lens_mm": 35.0,
            "output_basename": "factory_park_hf.tank-area-v2.png",
        },
    }
    target_to_key = {
        "site_aerial": "site_aerial",
        "south_gate_robot_eye": "robot_eye",
        "loading_dock_robot_eye": "loading_dock",
        "tank_farm_inspection": "tank_area",
    }
    for target in state.realism.preview_targets:
        key = target_to_key.get(str(target.get("id", "")))
        if key is None:
            continue
        position = _vector(target.get("position_m"), field_name=f"preview_targets.{key}.position_m")
        look_at = _vector(target.get("look_at_m"), field_name=f"preview_targets.{key}.look_at_m")
        lens = _finite_number(target.get("lens_mm"), field_name=f"preview_targets.{key}.lens_mm")
        basename = str(target.get("output_basename", defaults[key]["output_basename"]))
        if not basename or Path(basename).name != basename or not basename.lower().endswith(".png"):
            raise ValueError(f"preview target {key} output_basename must be a PNG basename")
        defaults[key].update(
            {
                "position_m": list(position),
                "look_at_m": list(look_at),
                "lens_mm": lens,
                "output_basename": basename,
            }
        )
    composition_overrides = {
        "robot_eye": {
            "position_m": [16.0, -98.0, 2.15],
            "look_at_m": [-2.5, -83.0, 1.7],
            "lens_mm": 30.0,
            "composition_adjustment": "outside_gate_three_quarter_security_context",
        },
        "loading_dock": {
            "position_m": [8.0, -4.0, 2.7],
            "look_at_m": [50.0, 18.0, 2.7],
            "lens_mm": 35.0,
            "composition_adjustment": "wide_three_dock_logistics_context",
        },
        "tank_area": {
            "position_m": [102.0, -54.0, 1.6],
            "look_at_m": [65.0, -36.0, 3.5],
            "lens_mm": 38.0,
            "composition_adjustment": "bund_three_tank_oblique_inspection_context",
        },
    }
    for key, override in composition_overrides.items():
        defaults[key].update(override)
    for camera_record in defaults.values():
        camera = _add_camera(
            camera_record["camera"],
            state.collections["preview"],
            camera_record["position_m"],
            camera_record["look_at_m"],
            lens_mm=float(camera_record["lens_mm"]),
        )
        camera_record["camera"] = camera.name
        camera_record["target_m"] = list(camera_record["look_at_m"])
        camera_record["projection"] = "perspective"
    return defaults


def _render_previews(
    state: BuildState,
    output_dir: Path,
    cameras: Mapping[str, Mapping[str, Any]],
) -> dict[str, Path]:
    blender = _require_bpy()
    scene = blender.context.scene
    role_to_key = {
        "site_aerial_preview": "site_aerial",
        "robot_eye_preview": "robot_eye",
        "loading_dock_preview": "loading_dock",
        "tank_area_preview": "tank_area",
    }
    outputs = {role: output_dir / str(cameras[key]["output_basename"]) for role, key in role_to_key.items()}
    for role, key in role_to_key.items():
        camera_name = str(cameras[key]["camera"])
        scene.camera = blender.data.objects[camera_name]
        scene.render.filepath = str(outputs[role])
        blender.ops.render.render(write_still=True)
    return outputs


def _export_scene(output_dir: Path, export_format: str) -> dict[str, Path]:
    blender = _require_bpy()
    outputs = {"editable_blend": output_dir / "factory_park_hf.blend"}
    blender.ops.wm.save_as_mainfile(filepath=str(outputs["editable_blend"]), compress=True)
    if export_format in {"fbx", "both"}:
        fbx_path = output_dir / "factory_park_hf.fbx"
        for obj in blender.context.scene.objects:
            obj.select_set(False)
        fbx_objects = [
            obj
            for obj in blender.context.scene.objects
            if obj.type == "MESH" and obj.get("lingtu_semantic_class") != "terrain"
        ]
        if not fbx_objects:
            raise RuntimeError("FBX export contains no non-terrain meshes")
        for obj in fbx_objects:
            obj.select_set(True)
        blender.context.view_layer.objects.active = fbx_objects[0]
        blender.ops.export_scene.fbx(
            filepath=str(fbx_path),
            check_existing=False,
            use_selection=True,
            object_types={"MESH"},
            global_scale=1.0,
            apply_unit_scale=True,
            apply_scale_options="FBX_SCALE_UNITS",
            use_space_transform=True,
            bake_space_transform=False,
            axis_forward="-Y",
            axis_up="Z",
            use_mesh_modifiers=True,
            mesh_smooth_type="FACE",
            add_leaf_bones=False,
            bake_anim=False,
            path_mode="AUTO",
        )
        outputs["unreal_scene_fbx"] = fbx_path
    if export_format in {"glb", "both"}:
        glb_path = output_dir / "factory_park_hf.glb"
        blender.ops.export_scene.gltf(
            filepath=str(glb_path),
            check_existing=False,
            export_format="GLB",
            use_selection=False,
            export_cameras=False,
            export_lights=False,
            export_yup=True,
        )
        outputs["portable_scene_glb"] = glb_path
    return outputs


def _asset_record(role: str, path: Path, output_dir: Path) -> dict[str, Any]:
    payload = path.read_bytes()
    formats = {
        "editable_blend": "Blender 5.2 editable scene",
        "unreal_scene_fbx": "Autodesk FBX binary",
        "portable_scene_glb": "glTF 2.0 binary",
        "site_aerial_preview": "PNG",
        "robot_eye_preview": "PNG",
        "loading_dock_preview": "PNG",
        "tank_area_preview": "PNG",
    }
    if role.endswith("_preview"):
        units = "pixels"
        axes = "image_top_left"
    elif role == "portable_scene_glb":
        units = "m"
        axes = "right-handed Y-up"
    else:
        units = "m"
        axes = "right-handed Z-up"
    record = {
        "role": role,
        "format": formats[role],
        "path": path.relative_to(output_dir).as_posix(),
        "bytes": len(payload),
        "sha256": _sha256(payload),
        "units": units,
        "axes": axes,
    }
    if role == "unreal_scene_fbx":
        record.update(
            {
                "contains_authoritative_terrain": False,
                "mesh_vertices": "object-local",
                "node_transforms": "preserved",
                "unreal_import": {
                    "combine_meshes": False,
                    "transform_vertex_to_absolute": False,
                    "bake_pivot_in_vertex": False,
                    "import_uniform_scale": 1.0,
                    "import_uniform_scale_supported": False,
                    "observed_ue58_behavior": (
                        "One authored metre imports as one numeric Unreal unit; "
                        "AssetImportTask ignores requested import uniform scale."
                    ),
                },
                "unreal_placement": {
                    "required": True,
                    "actor_uniform_scale": 100.0,
                    "terrain_actor_uniform_scale": 1.0,
                    "unit_conversion": "1 imported FBX unit * actor scale 100 = 100 Unreal cm/UU",
                },
            }
        )
    elif role == "portable_scene_glb":
        record["contains_authoritative_terrain"] = True
    return record


def artifact_set_digest(records: Iterable[Mapping[str, Any]]) -> str:
    """Digest role/path/content identity in stable order."""

    identity = [
        {"role": item["role"], "path": item["path"], "sha256": item["sha256"]}
        for item in sorted(records, key=lambda candidate: (str(candidate["role"]), str(candidate["path"])))
    ]
    return _sha256(_canonical_json(identity))


def _write_manifests(
    state: BuildState,
    *,
    output_dir: Path,
    outputs: Mapping[str, Path],
    bounds: Mapping[str, Sequence[float]],
    cameras: Mapping[str, Any],
) -> tuple[Path, Path, str]:
    blender = _require_bpy()
    layout_payload = state.layout_path.read_bytes()
    records = [_asset_record(role, path, output_dir) for role, path in sorted(outputs.items())]
    digest = artifact_set_digest(records)
    record_by_role = {str(item["role"]): item for item in records}
    camera_roles = {
        "site_aerial": "site_aerial_preview",
        "robot_eye": "robot_eye_preview",
        "loading_dock": "loading_dock_preview",
        "tank_area": "tank_area_preview",
    }
    camera_records = json.loads(json.dumps(cameras))
    for key, role in camera_roles.items():
        asset = record_by_role[role]
        camera_records[key].update(
            {
                "artifact_role": role,
                "output_path": asset["path"],
                "output_sha256": asset["sha256"],
            }
        )
    source_record = {
        "path": str(state.layout_path),
        "bytes": len(layout_payload),
        "sha256": _sha256(layout_payload),
        "layout_digest": state.layout.get("layout_digest"),
    }
    if state.realism.path.is_file():
        recipe_payload = state.realism.path.read_bytes()
    else:
        recipe_payload = _canonical_json(state.realism.raw)
    try:
        recipe_path = state.realism.path.relative_to(state.repo_root).as_posix()
    except ValueError:
        recipe_path = str(state.realism.path)
    total_mesh_actor_count = len(state.layout_records) + len(state.visual_records) + len(state.terrain_records)
    used_materials = sorted(
        {
            slot.material.name
            for obj in blender.context.scene.objects
            if obj.type == "MESH"
            for slot in obj.material_slots
            if slot.material is not None
        }
    )
    actor_min, actor_max = state.realism.actor_budget
    if not actor_min <= total_mesh_actor_count <= actor_max:
        raise RuntimeError(
            f"industrial realism mesh/actor budget violated: {total_mesh_actor_count} not in [{actor_min}, {actor_max}]"
        )
    manifest = {
        "schema": "lingtu.sim.blender-authoring-manifest.v1",
        "world_package": WORLD_PACKAGE,
        "source_layout": source_record,
        "realism": {
            "profile": state.realism.profile,
            "recipe": {
                "path": recipe_path,
                "source": state.realism.source,
                "bytes": len(recipe_payload),
                "sha256": state.realism.digest,
            },
            "seed": state.realism.seed,
            "namespace": state.realism.namespace,
            "actor_budget": {"min": actor_min, "max": actor_max},
        },
        "generator": {
            "script": str(Path(__file__).resolve()),
            "script_sha256": _sha256(Path(__file__).read_bytes()),
            "blender_version": blender.app.version_string,
            "blender_version_tuple": list(blender.app.version),
        },
        "coordinate_contract": coordinate_contract(),
        "bounds_m": {key: [float(value) for value in values] for key, values in bounds.items()},
        "scene": {
            "layout_declared_object_count": (
                len(state.layout_records)
                + len(state.terrain_feature_descriptors)
                + len(state.semantic_feature_descriptors)
            ),
            "layout_object_count": len(state.layout_records),
            "collision_object_count": sum(item["collision"] for item in state.layout_records),
            "visual_only_layout_count": sum(item["visual_only"] for item in state.layout_records),
            "generated_visual_only_count": len(state.visual_records),
            "physics_shared_count": sum(item["collision"] for item in state.layout_records),
            "physics_shared_unchanged": True,
            "material_profile": "procedural_pbr_v2",
            "material_count": len(used_materials),
            "material_library_key_count": len(state.materials),
            "pbr_material_profile_count": len(state.realism.raw.get("pbr_material_profiles", {})),
            "detail_counts": dict(sorted(state.detail_counts.items())),
            "total_mesh_actor_count": total_mesh_actor_count,
            "actor_budget_status": "within_budget",
            "terrain_object": state.terrain_records[0] if len(state.terrain_records) == 1 else None,
            "terrain_objects": state.terrain_records,
            "terrain_feature_descriptors": state.terrain_feature_descriptors,
            "semantic_feature_descriptors": state.semantic_feature_descriptors,
            "semantic_descriptors_not_materialized": True,
            "layout_objects": state.layout_records,
            "visual_only_objects": state.visual_records,
            "collections": sorted(state.collections),
            "materials": used_materials,
            "material_keys": sorted(state.materials),
            "cameras": camera_records,
        },
        "assets": records,
        "artifact_set_digest": digest,
    }
    manifest_path = output_dir / "authoring.manifest.json"
    manifest_payload = _canonical_json(manifest)
    manifest_path.write_bytes(manifest_payload)
    for record in records:
        artifact = output_dir / str(record["path"])
        payload = artifact.read_bytes()
        if len(payload) != record["bytes"] or _sha256(payload) != record["sha256"]:
            raise RuntimeError(f"artifact changed after hashing: {artifact}")
    digest_document = {
        "schema": "lingtu.sim.blender-artifact-set-digest.v1",
        "world_package": WORLD_PACKAGE,
        "layout_sha256": source_record["sha256"],
        "layout_digest": source_record["layout_digest"],
        "manifest": {
            "path": manifest_path.name,
            "bytes": len(manifest_payload),
            "sha256": _sha256(manifest_payload),
        },
        "artifact_set_digest": digest,
        "assets": [{"role": item["role"], "path": item["path"], "sha256": item["sha256"]} for item in records],
    }
    digest_path = output_dir / "artifact-set.digest.json"
    digest_path.write_bytes(_canonical_json(digest_document))
    return manifest_path, digest_path, digest


def build_factory_scene(args: argparse.Namespace) -> dict[str, Any]:
    """Build, render, export, and hash the FactoryPark_HF scene."""

    layout, objects = load_layout(args.layout)
    realism = load_realism_recipe(args.realism_recipe)
    recipe_layout_digest = realism.raw.get("layout_digest")
    if recipe_layout_digest is not None and recipe_layout_digest != layout.get("layout_digest"):
        raise ValueError("realism recipe layout_digest does not match expanded layout")
    bounds = compute_layout_bounds(objects)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    state = _setup_scene(layout, args.layout, args.repo_root, realism)
    _import_terrain(state)
    for item in objects:
        if item.semantic_class == "semantic_checkpoint":
            _record_semantic_feature_descriptor(state, item)
        elif not item.collision and not item.visual_only:
            _record_terrain_feature_descriptor(state, item)
        else:
            _create_layout_object(state, item)
    _decorate_layout_objects(state, objects)
    roads = [item for item in objects if item.semantic_class == "road"]
    if not any(item.semantic_class == "streetlight" for item in objects):
        _add_streetlights(state, roads)
    _add_landscaping(state, bounds)
    _add_landscape_realism_v2(state, bounds, objects)
    total_mesh_actor_count = len(state.layout_records) + len(state.visual_records) + len(state.terrain_records)
    actor_min, actor_max = state.realism.actor_budget
    if not actor_min <= total_mesh_actor_count <= actor_max:
        raise RuntimeError(
            "industrial realism mesh/actor budget violated before rendering: "
            f"{total_mesh_actor_count} not in [{actor_min}, {actor_max}]"
        )
    cameras = _setup_lighting_and_cameras(
        state,
        bounds,
        width=args.width,
        height=args.height,
        samples=args.samples,
    )
    previews = _render_previews(state, args.output_dir, cameras)
    exports = _export_scene(args.output_dir, args.export_format)
    outputs = {**exports, **previews}
    manifest_path, digest_path, digest = _write_manifests(
        state,
        output_dir=args.output_dir,
        outputs=outputs,
        bounds=bounds,
        cameras=cameras,
    )
    return {
        "schema": "lingtu.sim.blender-authoring-result.v1",
        "world_package": WORLD_PACKAGE,
        "manifest": str(manifest_path),
        "digest_file": str(digest_path),
        "artifact_set_digest": digest,
        "layout_objects": len(state.layout_records),
        "visual_only_objects": len(state.visual_records),
        "realism_profile": state.realism.profile,
        "recipe_digest": state.realism.digest,
        "outputs": {role: str(path) for role, path in sorted(outputs.items())},
    }


def main(argv: Sequence[str] | None = None) -> int:
    """Validate or author the requested FactoryPark_HF layout."""

    args = parse_cli_args(argv)
    layout, objects = load_layout(args.layout)
    if args.validate_only:
        print(
            json.dumps(
                {
                    "schema": layout.get("schema"),
                    "layout": str(args.layout),
                    "layout_digest": layout.get("layout_digest"),
                    "objects": len(objects),
                    "bounds_m": compute_layout_bounds(objects),
                },
                sort_keys=True,
            )
        )
        return 0
    if not BPY_AVAILABLE:
        _require_bpy()
    result = build_factory_scene(args)
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
