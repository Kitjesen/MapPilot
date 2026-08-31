"""Materialize the robot-free FactoryPark_HF world in Unreal Editor 5.8."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import time
import traceback
from pathlib import Path
from typing import Callable, Mapping, Sequence

try:
    import unreal
except ModuleNotFoundError:
    unreal = None


PROJECT_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
WORLD_RECIPE_PATH = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_HF_RECIPE",
        REPO_ROOT
        / "sim"
        / "packages"
        / "worlds"
        / "factory_park_hf"
        / "visual"
        / "ue_import.recipe.json",
    )
).resolve()
BLENDER_MANIFEST_PATH = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_HF_BLENDER_MANIFEST",
        REPO_ROOT / "build" / "factory-park-hf" / "blender-v2" / "authoring.manifest.json",
    )
).resolve()
ARTIFACT_DIGEST_PATH = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_HF_ARTIFACT_DIGEST",
        REPO_ROOT / "build" / "factory-park-hf" / "blender-v2" / "artifact-set.digest.json",
    )
).resolve()
REALISM_RECIPE_PATH = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_HF_REALISM_RECIPE",
        REPO_ROOT
        / "sim"
        / "packages"
        / "worlds"
        / "factory_park_hf"
        / "visual"
        / "realism.recipe.json",
    )
).resolve()
EVIDENCE_ROOT = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_HF_EVIDENCE_ROOT",
        REPO_ROOT / "build" / "factory-park-hf" / "unreal-v2",
    )
).resolve()
SUCCESS_SENTINEL = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_HF_SUCCESS",
        EVIDENCE_ROOT / "FactoryPark_HF.success.json",
    )
).resolve()
ERROR_SENTINEL = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_HF_ERROR",
        EVIDENCE_ROOT / "FactoryPark_HF.error.json",
    )
).resolve()
UNATTENDED = os.environ.get("LINGTU_FACTORY_PARK_HF_UNATTENDED") == "1"
REFRESH_SCREENSHOTS_ONLY = os.environ.get("LINGTU_FACTORY_PARK_HF_REFRESH_SCREENSHOTS_ONLY") == "1"
REFRESH_RENDERING_ONLY = os.environ.get("LINGTU_FACTORY_PARK_HF_REFRESH_RENDERING_ONLY") == "1"
PREVIOUS_SUCCESS_PATH = Path(
    os.environ.get(
        "LINGTU_FACTORY_PARK_HF_PREVIOUS_SUCCESS",
        EVIDENCE_ROOT / "FactoryPark_HF.previous.success.json",
    )
).resolve()

MAP_PATH = "/Game/RobotSim/Maps/FactoryPark_HF"
WORLD_PACKAGE = "factory_park_hf@1.0.0"
WORLD_BINDING = "WorldVisual:FactoryParkHF"
WORLD_ASSET_ROOT = "/Game/RobotSim/Worlds/FactoryParkHF"
TERRAIN_MESH_DESTINATION = f"{WORLD_ASSET_ROOT}/Terrain"
TERRAIN_MESH_NAME = "SM_FactoryPark_HF_Terrain"
TERRAIN_MESH_PATH = f"{TERRAIN_MESH_DESTINATION}/{TERRAIN_MESH_NAME}.{TERRAIN_MESH_NAME}"
BLENDER_MESH_DESTINATION = f"{WORLD_ASSET_ROOT}/Meshes"
MATERIAL_ASSET_ROOT = f"{WORLD_ASSET_ROOT}/Materials/IndustrialRealismV2"
MASTER_MATERIAL_NAME = "M_FactoryPark_IndustrialRealismV2"
MASTER_MATERIAL_PATH = f"{MATERIAL_ASSET_ROOT}/{MASTER_MATERIAL_NAME}.{MASTER_MATERIAL_NAME}"
FBX_IMPORT_UNIFORM_SCALE = 1.0
ENVIRONMENT_ACTOR_UNIT_SCALE = 100.0
TERRAIN_ACTOR_UNIT_SCALE = 1.0
VISUAL_INSTANCE_PRIMITIVE_MESHES = {
    "box": "/Engine/BasicShapes/Cube.Cube",
    "cylinder": "/Engine/BasicShapes/Cylinder.Cylinder",
    "ellipsoid": "/Engine/BasicShapes/Sphere.Sphere",
}

WORLD_TAG = "FactoryParkHF"
PHYSICS_SHARED_TAG = "PhysicsShared"
VISUAL_ONLY_TAG = "VisualOnly"
TERRAIN_TAG = "LingTuSameSourceTerrain"
PREPLACED_ROBOT_BINDINGS = 0
FORBIDDEN_BINDING_TAGS = {"LingTuBodyBinding"}
REALISM_PROFILE = "industrial_realism_v2"
REALISM_ACTOR_BUDGET = (1200, 1800)
REQUIRED_ACCEPTANCE_CAMERA_IDS = {
    "south_gate_robot_eye",
    "loading_dock_robot_eye",
    "tank_farm_inspection",
}

SUN_LABEL = "FactoryPark_HF_Sun"
SKYLIGHT_LABEL = "FactoryPark_HF_SkyLight"
ATMOSPHERE_LABEL = "FactoryPark_HF_SkyAtmosphere"
FOG_LABEL = "FactoryPark_HF_HeightFog"
FILL_LIGHT_LABEL = "FactoryPark_HF_IndustrialFill"
CAMERA_LABEL = "FactoryPark_HF_SessionCamera"
CAMERA_LOCATION_CM = (13_500.0, 13_000.0, 8_500.0)
CAMERA_TARGET_CM = (0.0, 0.0, 250.0)
CAMERA_FOV_DEGREES = 52.0
POST_PROCESS_LABEL = "FactoryPark_HF_LumenPostProcess"
TANK_CAMERA_REMEDIATION = {
    "position_m": (35.0, -62.0, 4.2),
    "look_at_m": (65.0, -36.0, 4.8),
    "adjustment": "avoid_east_drainage_reed_raster_occlusion",
}
TANK_BUND_FILL_LIGHT = {
    "label": "FactoryPark_HF_TankBund_QAFill",
    "position_m": (100.0, -62.0, 4.5),
    "look_at_m": (65.0, -53.5, 0.7),
    "intensity": 25000.0,
    "attenuation_radius_cm": 6500.0,
    "source_width_cm": 3500.0,
    "source_height_cm": 1200.0,
}
NON_SHADOW_CASTING_VISUAL_SEMANTICS = frozenset({"drainage_reed"})
EXPECTED_NON_SHADOW_CASTING_VISUAL_ACTOR_COUNT = 48
SCREENSHOT_TIMEOUT_SECONDS = 45.0

_SCREENSHOT_CAPTURE_PENDING = False
_SCREENSHOT_TICK_HANDLE: object | None = None

_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


def _should_cast_visual_shadow(record: Mapping[str, object]) -> bool:
    """Keep geometry visible while suppressing known micro-dressing VSM artifacts."""

    return not (
        record.get("visual_only") is True
        and str(record.get("semantic_class")) in NON_SHADOW_CASTING_VISUAL_SEMANTICS
    )


def _visual_shadow_component_policy(record: Mapping[str, object]) -> dict[str, bool]:
    if _should_cast_visual_shadow(record):
        return {}
    return {
        "cast_shadow": False,
        "affect_distance_field_lighting": False,
        "visible_in_ray_tracing": False,
    }


def _validate_derived_data_cache() -> dict[str, object]:
    """Fail closed unless this UE process uses the lane-local writable DDC."""

    local_override = os.environ.get("UE-LocalDataCachePath", "").strip()
    shared_override = os.environ.get("UE-SharedDataCachePath", "").strip()
    force_memory_cache_contract = (
        os.environ.get("LINGTU_FACTORY_PARK_HF_DDC_FORCE_MEMORY_CACHE") == "1"
    )
    command_line = str(unreal.SystemLibrary.get_command_line())
    force_memory_cache_argument = "-DDC-ForceMemoryCache" in command_line
    if not local_override:
        raise RuntimeError("FactoryPark_HF requires a lane-local UE-LocalDataCachePath")
    local_path = Path(local_override).resolve()
    try:
        local_path.relative_to(EVIDENCE_ROOT)
    except ValueError as error:
        raise RuntimeError(
            f"FactoryPark_HF DDC must stay under the v2 evidence root: {local_path}"
        ) from error
    if not local_path.is_dir():
        raise RuntimeError(f"FactoryPark_HF DDC directory is not writable/available: {local_path}")
    if shared_override.casefold() != "none":
        raise RuntimeError("FactoryPark_HF must disable the non-local shared DDC")
    if not force_memory_cache_contract or not force_memory_cache_argument:
        raise RuntimeError("FactoryPark_HF editor must run with -DDC-ForceMemoryCache")
    return {
        "local_override": str(local_path),
        "shared_override": shared_override,
        "local_override_under_evidence_root": True,
        "local_override_exists_and_is_directory": True,
        "force_memory_cache_environment_contract": force_memory_cache_contract,
        "force_memory_cache_argument": force_memory_cache_argument,
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


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json_object(path: Path) -> dict[str, object]:
    if not path.is_file():
        raise RuntimeError(f"required JSON input is missing: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"could not load JSON object {path}: {error}") from error
    if not isinstance(value, dict):
        raise RuntimeError(f"JSON root must be an object: {path}")
    return value


def _resolve_beneath(reference: object, base_dir: Path, allowed_root: Path, context: str) -> Path:
    if not isinstance(reference, str) or not reference.strip():
        raise RuntimeError(f"{context} path must be a non-empty string")
    candidate = Path(reference)
    resolved = candidate.resolve() if candidate.is_absolute() else (base_dir / candidate).resolve()
    allowed = allowed_root.resolve()
    try:
        resolved.relative_to(allowed)
    except ValueError as error:
        raise RuntimeError(f"{context} path escapes allowed root {allowed}: {reference}") from error
    return resolved


def _validate_file_record(
    name: str,
    record: object,
    base_dir: Path,
    allowed_root: Path,
) -> Path:
    if not isinstance(record, dict):
        raise RuntimeError(f"{name} file record must be an object")
    path = _resolve_beneath(record.get("path"), base_dir, allowed_root, name)
    if not path.is_file():
        raise RuntimeError(f"{name} file is missing: {path}")
    expected_bytes = record.get("bytes")
    if isinstance(expected_bytes, bool) or not isinstance(expected_bytes, int) or expected_bytes < 0:
        raise RuntimeError(f"{name} byte count must be a non-negative integer")
    actual_bytes = path.stat().st_size
    if expected_bytes != actual_bytes:
        raise RuntimeError(
            f"{name} byte count mismatch: expected={expected_bytes}, actual={actual_bytes}, path={path}"
        )
    expected_digest = record.get("sha256")
    if not isinstance(expected_digest, str) or _SHA256_PATTERN.fullmatch(expected_digest) is None:
        raise RuntimeError(f"{name} SHA256 must be 64 lowercase hexadecimal characters")
    actual_digest = _sha256_file(path)
    if expected_digest != actual_digest:
        raise RuntimeError(
            f"{name} SHA256 mismatch: expected={expected_digest}, actual={actual_digest}, path={path}"
        )
    return path


def _finite_vector(value: object, size: int, context: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise RuntimeError(f"{context} must contain exactly {size} values")
    result: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise RuntimeError(f"{context} must contain only finite numbers")
        number = float(item)
        if not math.isfinite(number):
            raise RuntimeError(f"{context} must contain only finite numbers")
        result.append(number)
    return tuple(result)


def _unit_quaternion(value: object, context: str) -> tuple[float, float, float, float]:
    quaternion = _finite_vector(value, 4, context)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm <= 1.0e-12:
        raise RuntimeError(f"{context} must not be a zero quaternion")
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-6):
        raise RuntimeError(f"{context} must be normalized, norm={norm}")
    return tuple(component / norm for component in quaternion)  # type: ignore[return-value]


def source_to_unreal_location_cm(position_m: Sequence[float]) -> tuple[float, float, float]:
    """Convert MuJoCo RH Z-up metres to Unreal LH Z-up centimetres."""

    x, y, z = _finite_vector(position_m, 3, "position_m")
    return (100.0 * x, -100.0 * y, 100.0 * z)


def source_to_unreal_quaternion_xyzw(
    quaternion_wxyz: Sequence[float],
) -> tuple[float, float, float, float]:
    """Apply the runtime's RH-to-LH quaternion reflection and xyzw ordering."""

    w, x, y, z = _unit_quaternion(quaternion_wxyz, "quaternion_wxyz")
    return (-x, y, -z, w)


def source_scale_to_unreal_actor_scale(source_scale: Sequence[float]) -> tuple[float, float, float]:
    """Scale metre-numeric FBX mesh assets into Unreal centimetre world units."""

    sx, sy, sz = _finite_vector(source_scale, 3, "source_scale")
    return (
        ENVIRONMENT_ACTOR_UNIT_SCALE * sx,
        ENVIRONMENT_ACTOR_UNIT_SCALE * sy,
        ENVIRONMENT_ACTOR_UNIT_SCALE * sz,
    )


def _build_visual_instance_plan(
    authoring: Mapping[str, object],
) -> dict[str, object]:
    """Compile Blender-derived VisualOnly primitives into deterministic HISM groups."""

    raw_records = authoring.get("visual_only_objects")
    if isinstance(raw_records, (str, bytes)) or not isinstance(raw_records, Sequence):
        raise RuntimeError("visual_only_objects must be a sequence")
    grouped: dict[tuple[str, str, str, bool], list[dict[str, object]]] = {}
    fallback_stable_ids: list[str] = []
    seen_stable_ids: set[str] = set()
    for index, value in enumerate(raw_records):
        if not isinstance(value, Mapping):
            raise RuntimeError(f"visual_only_objects[{index}] must be an object")
        stable_id = value.get("stable_id")
        if (
            not isinstance(stable_id, str)
            or not stable_id
            or stable_id != stable_id.strip()
            or stable_id in seen_stable_ids
        ):
            raise RuntimeError(
                f"visual_only_objects[{index}] has an invalid or duplicate stable_id"
            )
        seen_stable_ids.add(stable_id)
        if (
            value.get("source") != "blender_derived_visual"
            or value.get("physics_proxy") != "none"
            or value.get("collision") is not False
            or value.get("visual_only") is not True
        ):
            raise RuntimeError(
                f"visual_only_objects[{index}] violates VisualOnly authority"
            )
        shape = value.get("shape")
        if not isinstance(shape, str) or not shape:
            raise RuntimeError(f"visual_only_objects[{index}].shape is invalid")
        if shape not in VISUAL_INSTANCE_PRIMITIVE_MESHES:
            fallback_stable_ids.append(stable_id)
            continue
        material = value.get("material")
        semantic_class = value.get("semantic_class")
        if not isinstance(material, str) or not material:
            raise RuntimeError(f"visual_only_objects[{index}].material is invalid")
        if not isinstance(semantic_class, str) or not semantic_class:
            raise RuntimeError(
                f"visual_only_objects[{index}].semantic_class is invalid"
            )
        dimensions = _finite_vector(
            value.get("dimensions_m"),
            3,
            f"visual_only_objects[{index}].dimensions_m",
        )
        source_scale = _finite_vector(
            value.get("scale"),
            3,
            f"visual_only_objects[{index}].scale",
        )
        if any(component <= 0.0 for component in (*dimensions, *source_scale)):
            raise RuntimeError(
                f"visual_only_objects[{index}] dimensions and scale must be positive"
            )
        cast_shadow = _should_cast_visual_shadow(value)
        key = (shape, material, semantic_class, cast_shadow)
        grouped.setdefault(key, []).append(
            {
                "stable_id": stable_id,
                "location_cm": list(
                    source_to_unreal_location_cm(value.get("position_m"))  # type: ignore[arg-type]
                ),
                "quaternion_xyzw": list(
                    source_to_unreal_quaternion_xyzw(
                        value.get("quaternion_wxyz")  # type: ignore[arg-type]
                    )
                ),
                "scale_xyz": [
                    dimensions[axis] * source_scale[axis]
                    for axis in range(3)
                ],
            }
        )

    groups: list[dict[str, object]] = []
    for key in sorted(grouped):
        shape, material, semantic_class, cast_shadow = key
        identity = {
            "shape": shape,
            "material": material,
            "semantic_class": semantic_class,
            "cast_shadow": cast_shadow,
        }
        group_id = (
            f"factory_park.visual.{shape}."
            f"{_sha256_bytes(_canonical_json(identity))[:16]}"
        )
        instances = sorted(grouped[key], key=lambda item: str(item["stable_id"]))
        groups.append(
            {
                "group_id": group_id,
                "component_class": "HierarchicalInstancedStaticMeshComponent",
                "primitive_mesh": VISUAL_INSTANCE_PRIMITIVE_MESHES[shape],
                "shape": shape,
                "source_material": material,
                "semantic_class": semantic_class,
                "cast_shadow": cast_shadow,
                "collision_enabled": False,
                "generate_overlap_events": False,
                "can_ever_affect_navigation": False,
                "instance_count": len(instances),
                "instances": instances,
            }
        )

    instanced_count = sum(int(group["instance_count"]) for group in groups)
    fallback_stable_ids.sort()
    return {
        "schema": "lingtu.sim.factory-park-visual-instance-plan.v1",
        "source_instance_count": len(raw_records),
        "instanced_instance_count": instanced_count,
        "fallback_actor_count": len(fallback_stable_ids),
        "group_count": len(groups),
        "projected_visual_actor_count": len(fallback_stable_ids)
        + (1 if groups else 0),
        "authority": {
            "classification": "VisualOnly",
            "physics": "mujoco",
            "collision_enabled": False,
            "generate_overlap_events": False,
            "can_ever_affect_navigation": False,
        },
        "groups": groups,
        "fallback_stable_ids": fallback_stable_ids,
    }


def _asset_set_digest(records: Sequence[Mapping[str, object]]) -> str:
    identity = [
        {"path": record["path"], "sha256": record["sha256"]}
        for record in sorted(records, key=lambda item: str(item["path"]))
    ]
    return _sha256_bytes(_canonical_json(identity))


def _blender_artifact_set_digest(records: Sequence[Mapping[str, object]]) -> str:
    identity = [
        {"role": record["role"], "path": record["path"], "sha256": record["sha256"]}
        for record in sorted(records, key=lambda item: (str(item["role"]), str(item["path"])))
    ]
    return _sha256_bytes(_canonical_json(identity))


def _validate_layout_object(record: object, index: int) -> dict[str, object]:
    if not isinstance(record, dict):
        raise RuntimeError(f"layout.objects[{index}] must be an object")
    stable_id = record.get("id")
    if not isinstance(stable_id, str) or not stable_id.strip():
        raise RuntimeError(f"layout.objects[{index}].id must be a non-empty string")
    semantic_class = record.get("semantic_class")
    if not isinstance(semantic_class, str) or not semantic_class.strip():
        raise RuntimeError(f"layout object {stable_id} has no semantic_class")
    if record.get("shape") not in {"box", "cylinder"}:
        raise RuntimeError(f"layout object {stable_id} has an unsupported shape: {record.get('shape')}")
    _finite_vector(record.get("position_m"), 3, f"layout object {stable_id}.position_m")
    for angle_name in ("yaw_deg", "pitch_deg"):
        if angle_name in record:
            _finite_vector([record.get(angle_name)], 1, f"layout object {stable_id}.{angle_name}")
    if not isinstance(record.get("collision"), bool) or not isinstance(record.get("visual_only"), bool):
        raise RuntimeError(f"layout object {stable_id} must declare boolean collision and visual_only")
    if record["visual_only"] and record["collision"]:
        raise RuntimeError(f"visual-only layout object {stable_id} cannot request physics collision")
    if record["shape"] == "box":
        dimensions = _finite_vector(record.get("size_m"), 3, f"layout object {stable_id}.size_m")
    else:
        radius = _finite_vector([record.get("radius_m")], 1, f"layout object {stable_id}.radius_m")[0]
        half_height = _finite_vector(
            [record.get("half_height_m")], 1, f"layout object {stable_id}.half_height_m"
        )[0]
        dimensions = (2.0 * radius, 2.0 * radius, 2.0 * half_height)
    if any(value <= 0.0 for value in dimensions):
        raise RuntimeError(f"layout object {stable_id} dimensions must be positive")
    return record


def _validate_terrain_obj(
    terrain_path: Path,
    source_record: Mapping[str, object],
    coordinate_contract: Mapping[str, object],
    layout: Mapping[str, object],
) -> dict[str, object]:
    grid = tuple(int(value) for value in _finite_vector(coordinate_contract.get("grid_px"), 2, "grid_px"))
    extent = _finite_vector(coordinate_contract.get("extent_m"), 2, "extent_m")
    width, height = grid
    if width < 2 or height < 2:
        raise RuntimeError("terrain grid must be at least 2 by 2")
    expected_vertices = width * height
    expected_triangles = 2 * (width - 1) * (height - 1)
    if source_record.get("vertex_count") not in (None, expected_vertices):
        raise RuntimeError("terrain source vertex_count disagrees with coordinate_contract.grid_px")
    if source_record.get("triangle_count") not in (None, expected_triangles):
        raise RuntimeError("terrain source triangle_count disagrees with coordinate_contract.grid_px")

    spawn_reference = coordinate_contract.get("spawn_reference")
    if not isinstance(spawn_reference, dict):
        raise RuntimeError("coordinate_contract.spawn_reference is missing")
    sample_column, sample_row_north = (
        int(value) for value in _finite_vector(spawn_reference.get("sample_pixel"), 2, "spawn sample_pixel")
    )
    if not (0 <= sample_column < width and 0 <= sample_row_north < height):
        raise RuntimeError("spawn sample_pixel lies outside the terrain grid")
    # OBJ rows preserve image order: source north maps to negative Unreal Y.
    spawn_vertex_index = sample_row_north * width + sample_column

    coordinates_declared = False
    vertex_count = 0
    triangle_count = 0
    min_x = min_y = min_z = math.inf
    max_x = max_y = max_z = -math.inf
    spawn_height_cm: float | None = None
    with terrain_path.open("r", encoding="ascii") as stream:
        for line in stream:
            if "coordinates=unreal_lh_z_up_cm" in line:
                coordinates_declared = True
            elif line.startswith("v "):
                parts = line.split()
                if len(parts) != 4:
                    raise RuntimeError(f"malformed terrain OBJ vertex in {terrain_path}")
                x, y, z = (float(value) for value in parts[1:4])
                if not all(math.isfinite(value) for value in (x, y, z)):
                    raise RuntimeError("terrain OBJ contains a non-finite vertex")
                if vertex_count == spawn_vertex_index:
                    spawn_height_cm = z
                min_x, max_x = min(min_x, x), max(max_x, x)
                min_y, max_y = min(min_y, y), max(max_y, y)
                min_z, max_z = min(min_z, z), max(max_z, z)
                vertex_count += 1
            elif line.startswith("f "):
                triangle_count += 1
    if not coordinates_declared:
        raise RuntimeError("terrain OBJ does not declare unreal_lh_z_up_cm coordinates")
    if vertex_count != expected_vertices or triangle_count != expected_triangles:
        raise RuntimeError(
            "terrain OBJ topology disagrees with its grid: "
            f"vertices={vertex_count}/{expected_vertices}, triangles={triangle_count}/{expected_triangles}"
        )
    expected_bounds = (-50.0 * extent[0], 50.0 * extent[0], -50.0 * extent[1], 50.0 * extent[1])
    actual_bounds = (min_x, max_x, min_y, max_y)
    if any(
        not math.isclose(actual, expected, rel_tol=0.0, abs_tol=0.02)
        for actual, expected in zip(actual_bounds, expected_bounds)
    ):
        raise RuntimeError(f"terrain OBJ violates its Unreal-centimetre extent: {actual_bounds}")
    if spawn_height_cm is None:
        raise RuntimeError("terrain OBJ does not contain the declared spawn sample")

    height_encoding = coordinate_contract.get("height_encoding")
    if not isinstance(height_encoding, dict):
        raise RuntimeError("coordinate_contract.height_encoding is missing")
    elevation_scale_m = float(height_encoding.get("elevation_scale_m"))
    vertical_origin_m = float(coordinate_contract.get("vertical_origin_m"))
    sample_u16 = int(spawn_reference.get("sample_u16"))
    expected_spawn_height_m = sample_u16 / 65535.0 * elevation_scale_m + vertical_origin_m
    reference_position = _finite_vector(
        spawn_reference.get("world_position_m"), 3, "spawn_reference.world_position_m"
    )
    layout_spawn = layout.get("spawn")
    if not isinstance(layout_spawn, dict):
        raise RuntimeError("layout.spawn is missing")
    layout_spawn_position = _finite_vector(layout_spawn.get("position_m"), 3, "layout.spawn.position_m")
    if not math.isclose(expected_spawn_height_m, reference_position[2], rel_tol=0.0, abs_tol=2.0e-5):
        raise RuntimeError("height encoding and spawn reference use different vertical datums")
    if not math.isclose(reference_position[2], layout_spawn_position[2], rel_tol=0.0, abs_tol=2.0e-5):
        raise RuntimeError("terrain spawn reference and layout spawn use different vertical datums")
    if not math.isclose(spawn_height_cm, 100.0 * layout_spawn_position[2], rel_tol=0.0, abs_tol=0.02):
        raise RuntimeError(
            "terrain OBJ spawn sample has the wrong vertical offset: "
            f"terrain={spawn_height_cm} cm, layout={100.0 * layout_spawn_position[2]} cm"
        )
    landscape = coordinate_contract.get("unreal_landscape")
    if not isinstance(landscape, dict):
        raise RuntimeError("coordinate_contract.unreal_landscape is missing")
    landscape_location = _finite_vector(landscape.get("location_cm"), 3, "unreal_landscape.location_cm")
    expected_landscape_z_cm = 100.0 * (vertical_origin_m + 0.5 * elevation_scale_m)
    if not math.isclose(landscape_location[2], expected_landscape_z_cm, rel_tol=0.0, abs_tol=1.0e-6):
        raise RuntimeError("recorded Unreal Landscape vertical offset is inconsistent with height encoding")
    return {
        "path": terrain_path,
        "sha256": source_record["sha256"],
        "coordinates": "unreal_lh_z_up_cm",
        "grid": grid,
        "bounds_cm": actual_bounds,
        "height_bounds_cm": (min_z, max_z),
        "spawn_height_cm": spawn_height_cm,
        "landscape_location_cm": landscape_location,
    }


def _validate_world_recipe() -> dict[str, object]:
    recipe = _load_json_object(WORLD_RECIPE_PATH)
    if recipe.get("schema") != "lingtu.sim.unreal-world-import-recipe.v1":
        raise RuntimeError(f"unsupported FactoryPark_HF recipe schema: {recipe.get('schema')}")
    if recipe.get("world_package") != WORLD_PACKAGE:
        raise RuntimeError(f"FactoryPark_HF recipe must resolve {WORLD_PACKAGE}")
    if recipe.get("binding") != WORLD_BINDING:
        raise RuntimeError(f"FactoryPark_HF recipe binding must be {WORLD_BINDING}")
    if recipe.get("target_level") != MAP_PATH:
        raise RuntimeError(f"FactoryPark_HF recipe target must be {MAP_PATH}")

    coordinate_contract = recipe.get("coordinate_contract")
    if not isinstance(coordinate_contract, dict):
        raise RuntimeError("FactoryPark_HF recipe has no coordinate_contract")
    if coordinate_contract.get("source_frame") != "mujoco_rh_z_up_m":
        raise RuntimeError("FactoryPark_HF layout source frame must be mujoco_rh_z_up_m")
    if coordinate_contract.get("unreal_frame") != "unreal_lh_z_up_cm":
        raise RuntimeError("FactoryPark_HF Unreal frame must be unreal_lh_z_up_cm")
    if coordinate_contract.get("axis_mapping") != ["x", "-y", "z"]:
        raise RuntimeError("FactoryPark_HF axis mapping must be [x, -y, z]")
    layout_import = recipe.get("unreal_layout_import")
    if not isinstance(layout_import, dict):
        raise RuntimeError("FactoryPark_HF recipe has no unreal_layout_import")
    if layout_import.get("linear_scale_m_to_cm") != 100.0:
        raise RuntimeError("FactoryPark_HF layout scale must be exactly 100 centimetres per metre")

    sources = recipe.get("sources")
    if not isinstance(sources, dict):
        raise RuntimeError("FactoryPark_HF recipe has no sources object")
    required_sources = {
        "asset_manifest",
        "expanded_layout",
        "heightfield_png",
        "mujoco_heightfield",
        "mujoco_world",
        "realism_recipe",
        "semantic_entities",
        "site_plan_svg",
        "terrain_obj",
    }
    missing_sources = required_sources.difference(sources)
    if missing_sources:
        raise RuntimeError(f"FactoryPark_HF recipe is missing sources: {sorted(missing_sources)}")
    resolved_sources = {
        name: _validate_file_record(name, record, REPO_ROOT, REPO_ROOT)
        for name, record in sources.items()
    }

    layout_path = resolved_sources["expanded_layout"]
    if layout_path != _resolve_beneath(layout_import.get("source"), REPO_ROOT, REPO_ROOT, "layout import"):
        raise RuntimeError("unreal_layout_import source disagrees with sources.expanded_layout")
    layout = _load_json_object(layout_path)
    if layout.get("schema") != "lingtu.sim.expanded-world-layout.v1":
        raise RuntimeError("unsupported FactoryPark_HF expanded layout schema")
    if layout.get("world_package") != WORLD_PACKAGE:
        raise RuntimeError("FactoryPark_HF recipe and layout package identities disagree")
    layout_without_digest = dict(layout)
    declared_layout_digest = layout_without_digest.pop("layout_digest", None)
    actual_layout_digest = _sha256_bytes(_canonical_json(layout_without_digest))
    if declared_layout_digest != actual_layout_digest or recipe.get("layout_digest") != actual_layout_digest:
        raise RuntimeError(
            "FactoryPark_HF layout digest mismatch: "
            f"layout={declared_layout_digest}, recipe={recipe.get('layout_digest')}, actual={actual_layout_digest}"
        )
    if layout.get("coordinate_contract") != coordinate_contract:
        raise RuntimeError("FactoryPark_HF recipe and expanded layout coordinate contracts disagree")
    layout_objects_value = layout.get("objects")
    if not isinstance(layout_objects_value, list) or not layout_objects_value:
        raise RuntimeError("FactoryPark_HF expanded layout contains no objects")
    layout_objects = [_validate_layout_object(record, index) for index, record in enumerate(layout_objects_value)]
    stable_ids = [str(record["id"]) for record in layout_objects]
    if len(stable_ids) != len(set(stable_ids)):
        raise RuntimeError("FactoryPark_HF expanded layout contains duplicate stable IDs")
    if layout_import.get("object_count") != len(layout_objects):
        raise RuntimeError("unreal_layout_import.object_count disagrees with expanded layout")

    manifest_path = resolved_sources["asset_manifest"]
    asset_manifest = _load_json_object(manifest_path)
    if asset_manifest.get("schema") != "lingtu.sim.world-asset-manifest.v1":
        raise RuntimeError("unsupported FactoryPark_HF asset manifest schema")
    if asset_manifest.get("world_package") != WORLD_PACKAGE:
        raise RuntimeError("FactoryPark_HF asset manifest package identity is invalid")
    if asset_manifest.get("layout_digest") != actual_layout_digest:
        raise RuntimeError("FactoryPark_HF asset manifest has the wrong layout digest")
    asset_records = asset_manifest.get("assets")
    if not isinstance(asset_records, list) or not asset_records or not all(isinstance(item, dict) for item in asset_records):
        raise RuntimeError("FactoryPark_HF asset manifest contains no valid asset records")
    for index, record in enumerate(asset_records):
        _validate_file_record(f"asset_manifest.assets[{index}]", record, REPO_ROOT, REPO_ROOT)
    if asset_manifest.get("asset_set_digest") != _asset_set_digest(asset_records):
        raise RuntimeError("FactoryPark_HF asset manifest asset_set_digest is invalid")
    assets_by_path = {record.get("path"): record for record in asset_records}
    for source_name, source_record in sources.items():
        if source_name == "asset_manifest":
            continue
        manifest_record = assets_by_path.get(source_record.get("path"))
        if manifest_record is None:
            raise RuntimeError(f"FactoryPark_HF asset manifest omits recipe source {source_name}")
        if (
            manifest_record.get("bytes") != source_record.get("bytes")
            or manifest_record.get("sha256") != source_record.get("sha256")
        ):
            raise RuntimeError(f"FactoryPark_HF recipe and asset manifest disagree for {source_name}")

    terrain_path = resolved_sources["terrain_obj"]
    blender_authoring = recipe.get("blender_authoring")
    if not isinstance(blender_authoring, dict):
        raise RuntimeError("FactoryPark_HF recipe has no blender_authoring contract")
    if blender_authoring.get("layout_coordinates") != "mujoco_rh_z_up_m":
        raise RuntimeError("FactoryPark_HF Blender layout coordinates are invalid")
    if blender_authoring.get("terrain_coordinates") != "unreal_lh_z_up_cm":
        raise RuntimeError("FactoryPark_HF terrain coordinates are invalid")
    if terrain_path != _resolve_beneath(blender_authoring.get("terrain_obj"), REPO_ROOT, REPO_ROOT, "terrain OBJ"):
        raise RuntimeError("blender_authoring terrain path disagrees with sources.terrain_obj")
    if layout_path != _resolve_beneath(
        blender_authoring.get("expanded_layout"), REPO_ROOT, REPO_ROOT, "expanded layout"
    ):
        raise RuntimeError("blender_authoring layout path disagrees with sources.expanded_layout")
    terrain = _validate_terrain_obj(terrain_path, sources["terrain_obj"], coordinate_contract, layout)
    return {
        "recipe": recipe,
        "recipe_path": WORLD_RECIPE_PATH,
        "recipe_sha256": _sha256_file(WORLD_RECIPE_PATH),
        "layout": layout,
        "layout_path": layout_path,
        "layout_sha256": sources["expanded_layout"]["sha256"],
        "layout_digest": actual_layout_digest,
        "layout_objects": layout_objects,
        "layout_by_id": {str(record["id"]): record for record in layout_objects},
        "asset_manifest_path": manifest_path,
        "asset_manifest_sha256": sources["asset_manifest"]["sha256"],
        "resolved_sources": resolved_sources,
        "terrain": terrain,
    }


def _validate_realism_recipe_document(
    recipe: object,
    expected_layout_digest: str,
) -> dict[str, object]:
    if not isinstance(recipe, dict):
        raise RuntimeError("FactoryPark_HF realism recipe root must be an object")
    if recipe.get("schema") != "lingtu.sim.factory-park-realism-recipe.v1":
        raise RuntimeError(f"unsupported FactoryPark_HF realism recipe schema: {recipe.get('schema')}")
    if recipe.get("profile") != REALISM_PROFILE:
        raise RuntimeError(f"FactoryPark_HF realism profile must be {REALISM_PROFILE}")
    if recipe.get("world_package") != WORLD_PACKAGE:
        raise RuntimeError("FactoryPark_HF realism recipe resolves the wrong world package")
    if recipe.get("layout_digest") != expected_layout_digest:
        raise RuntimeError("FactoryPark_HF realism recipe layout digest disagrees with the world layout digest")

    expected_hard_rules = {
        "added_dressing_classification": "VisualOnly",
        "added_dressing_collision": False,
        "allow_new_physics_authority": False,
        "layout_unchanged": True,
        "mujoco_world_unchanged": True,
        "physics_authority": "mujoco",
    }
    hard_rules = recipe.get("hard_rules")
    if hard_rules != expected_hard_rules:
        raise RuntimeError(
            "FactoryPark_HF realism recipe violates the VisualOnly authority contract: "
            f"expected={expected_hard_rules}, actual={hard_rules}"
        )

    authoring = recipe.get("authoring_contract")
    if not isinstance(authoring, dict):
        raise RuntimeError("FactoryPark_HF realism recipe has no authoring_contract")
    dressing_defaults = authoring.get("dressing_defaults")
    preview_output_contract = authoring.get("preview_output_contract")
    if (
        authoring.get("coordinate_frame") != "mujoco_rh_z_up_m"
        or authoring.get("placement_source") != "expanded_layout"
        or authoring.get("randomness") != "seeded_only"
        or not isinstance(dressing_defaults, dict)
        or dressing_defaults.get("classification") != "VisualOnly"
        or dressing_defaults.get("collision") is not False
        or dressing_defaults.get("physics_representation") != "none"
    ):
        raise RuntimeError("FactoryPark_HF realism authoring contract may not create physics authority")
    if (
        not isinstance(preview_output_contract, dict)
        or preview_output_contract.get("consumer_output_root") != "consumer_owned_evidence_root"
        or preview_output_contract.get("recipe_field") != "output_basename"
    ):
        raise RuntimeError("FactoryPark_HF realism previews must use consumer-owned output roots")

    profiles_value = recipe.get("pbr_material_profiles")
    if not isinstance(profiles_value, dict) or not profiles_value:
        raise RuntimeError("FactoryPark_HF realism recipe contains no PBR material profiles")
    profiles: dict[str, dict[str, object]] = {}
    material_profile_by_source: dict[str, str] = {}
    for profile_name, value in sorted(profiles_value.items()):
        if not isinstance(profile_name, str) or not profile_name or not isinstance(value, dict):
            raise RuntimeError("FactoryPark_HF PBR material profiles must be named objects")
        applies_to = value.get("applies_to")
        if not isinstance(applies_to, list) or not applies_to or not all(
            isinstance(item, str) and item for item in applies_to
        ):
            raise RuntimeError(f"PBR profile {profile_name} has no valid applies_to list")
        roughness = _finite_vector(value.get("roughness_range"), 2, f"{profile_name}.roughness_range")
        metallic = _finite_vector([value.get("metallic")], 1, f"{profile_name}.metallic")[0]
        if not (0.0 <= roughness[0] <= roughness[1] <= 1.0 and 0.0 <= metallic <= 1.0):
            raise RuntimeError(f"PBR profile {profile_name} has invalid metallic or roughness values")
        base_color = value.get("base_color_srgb")
        if base_color is not None:
            color = _finite_vector(base_color, 3, f"{profile_name}.base_color_srgb")
            if any(channel < 0.0 or channel > 1.0 for channel in color):
                raise RuntimeError(f"PBR profile {profile_name} base color lies outside sRGB [0,1]")
        elif value.get("base_color_source") != "retain_layout_material_hue":
            raise RuntimeError(f"PBR profile {profile_name} has no deterministic base color source")
        profiles[profile_name] = value
        for source_material in applies_to:
            if source_material in material_profile_by_source:
                raise RuntimeError(f"PBR source material {source_material} is assigned to multiple profiles")
            material_profile_by_source[source_material] = profile_name

    preview_targets_value = recipe.get("preview_targets")
    if not isinstance(preview_targets_value, list) or not preview_targets_value:
        raise RuntimeError("FactoryPark_HF realism recipe has no preview targets")
    preview_targets: list[dict[str, object]] = []
    preview_ids: set[str] = set()
    output_basenames: set[str] = set()
    for index, value in enumerate(preview_targets_value):
        context = f"preview_targets[{index}]"
        if not isinstance(value, dict):
            raise RuntimeError(f"{context} must be an object")
        target_id = value.get("id")
        basename = value.get("output_basename")
        if not isinstance(target_id, str) or not target_id or target_id in preview_ids:
            raise RuntimeError(f"{context}.id must be unique and non-empty")
        if (
            not isinstance(basename, str)
            or not basename
            or Path(basename).name != basename
            or Path(basename).suffix.lower() != ".png"
            or basename in output_basenames
        ):
            raise RuntimeError(f"{context}.output_basename must be a unique PNG basename")
        if value.get("camera_frame") != "mujoco_rh_z_up_m":
            raise RuntimeError(f"{context} uses the wrong camera frame")
        _finite_vector(value.get("position_m"), 3, f"{context}.position_m")
        _finite_vector(value.get("look_at_m"), 3, f"{context}.look_at_m")
        lens_mm = _finite_vector([value.get("lens_mm")], 1, f"{context}.lens_mm")[0]
        resolution = _finite_vector(value.get("resolution_px"), 2, f"{context}.resolution_px")
        if lens_mm <= 0.0 or any(component <= 0.0 or not float(component).is_integer() for component in resolution):
            raise RuntimeError(f"{context} has an invalid lens or resolution")
        preview_ids.add(target_id)
        output_basenames.add(basename)
        preview_targets.append(value)
    if not REQUIRED_ACCEPTANCE_CAMERA_IDS.issubset(preview_ids):
        missing = sorted(REQUIRED_ACCEPTANCE_CAMERA_IDS.difference(preview_ids))
        raise RuntimeError(f"FactoryPark_HF realism recipe omits acceptance cameras: {missing}")

    lighting = recipe.get("lighting")
    if not isinstance(lighting, dict) or not isinstance(lighting.get("unreal"), dict):
        raise RuntimeError("FactoryPark_HF realism recipe has no Unreal lighting contract")
    unreal_lighting = lighting["unreal"]
    if (
        unreal_lighting.get("global_illumination") != "Lumen"
        or unreal_lighting.get("reflections") != "Lumen"
        or unreal_lighting.get("shadow_method") != "VirtualShadowMaps"
        or unreal_lighting.get("skylight_realtime_capture") is not True
    ):
        raise RuntimeError("FactoryPark_HF Unreal lighting must require Lumen and Virtual Shadow Maps")
    budgets = recipe.get("performance_budgets")
    if not isinstance(budgets, dict):
        raise RuntimeError("FactoryPark_HF realism recipe has no performance budgets")
    max_instances = budgets.get("max_visual_only_instances")
    if isinstance(max_instances, bool) or not isinstance(max_instances, int) or max_instances < REALISM_ACTOR_BUDGET[1]:
        raise RuntimeError("FactoryPark_HF realism budget cannot contain the v2 actor target")
    return {
        "document": recipe,
        "profile": REALISM_PROFILE,
        "layout_digest": expected_layout_digest,
        "seed": recipe.get("seed"),
        "hard_rules": hard_rules,
        "authoring_contract": authoring,
        "material_profiles": profiles,
        "material_profile_by_source_material": material_profile_by_source,
        "detail_targets": recipe.get("detail_targets"),
        "lighting": lighting,
        "preview_targets": preview_targets,
        "performance_budgets": budgets,
    }


def _validate_realism_recipe(world: Mapping[str, object]) -> dict[str, object]:
    resolved_sources = world.get("resolved_sources")
    if not isinstance(resolved_sources, dict) or resolved_sources.get("realism_recipe") != REALISM_RECIPE_PATH:
        raise RuntimeError("Unreal world recipe must reference the selected realism recipe exactly")
    recipe = _load_json_object(REALISM_RECIPE_PATH)
    validated = _validate_realism_recipe_document(recipe, str(world["layout_digest"]))
    source_record = world["recipe"]["sources"]["realism_recipe"]  # type: ignore[index]
    if source_record.get("sha256") != _sha256_file(REALISM_RECIPE_PATH):
        raise RuntimeError("Unreal world recipe realism source digest is stale")
    validated.update(
        {
            "recipe_path": REALISM_RECIPE_PATH,
            "recipe_sha256": source_record["sha256"],
            "recipe_bytes": source_record["bytes"],
        }
    )
    return validated


_MANIFEST_MATERIAL_SWATCHES: dict[str, tuple[float, float, float]] = {
    "asphalt": (0.12, 0.13, 0.14),
    "barrel_blue": (0.08, 0.18, 0.28),
    "bund_concrete": (0.46, 0.47, 0.45),
    "ceramic": (0.54, 0.43, 0.31),
    "checkpoint_green": (0.06, 0.45, 0.16),
    "concrete": (0.48, 0.49, 0.47),
    "container_blue": (0.08, 0.23, 0.42),
    "container_orange": (0.68, 0.24, 0.05),
    "container_red": (0.48, 0.07, 0.05),
    "crate_tan": (0.43, 0.32, 0.2),
    "curb_concrete": (0.52, 0.52, 0.49),
    "factory_cladding": (0.17, 0.29, 0.34),
    "fence_steel": (0.34, 0.37, 0.37),
    "foliage_dry": (0.29, 0.25, 0.16),
    "galvanized": (0.43, 0.46, 0.47),
    "glass": (0.08, 0.19, 0.24),
    "grass": (0.14, 0.25, 0.08),
    "guardhouse": (0.66, 0.65, 0.58),
    "lamp": (0.8, 0.82, 0.77),
    "lamp_emissive": (0.92, 0.78, 0.52),
    "line_white": (0.73, 0.72, 0.64),
    "painted_steel": (0.38, 0.4, 0.4),
    "pallet_wood": (0.42, 0.29, 0.16),
    "parking_marking": (0.76, 0.74, 0.62),
    "road_marking": (0.78, 0.76, 0.65),
    "roof_metal": (0.28, 0.31, 0.32),
    "rollup_door": (0.34, 0.37, 0.39),
    "rubber": (0.025, 0.028, 0.03),
    "safety_black": (0.035, 0.04, 0.045),
    "safety_red": (0.55, 0.045, 0.03),
    "safety_yellow": (0.86, 0.61, 0.03),
    "steel": (0.41, 0.44, 0.44),
    "tank_white": (0.77, 0.79, 0.76),
    "traffic_orange": (0.82, 0.26, 0.035),
    "transformer_dark": (0.055, 0.065, 0.07),
    "vegetation_leaf": (0.12, 0.27, 0.07),
    "vegetation_trunk": (0.24, 0.13, 0.065),
    "vehicle_blue": (0.08, 0.22, 0.34),
    "vehicle_white": (0.72, 0.74, 0.72),
    "warehouse_cladding": (0.47, 0.5, 0.5),
    "window_glass": (0.08, 0.19, 0.24),
}


def _manifest_derived_base_color(source_material: str, semantic_class: str) -> list[float]:
    if source_material in _MANIFEST_MATERIAL_SWATCHES:
        return list(_MANIFEST_MATERIAL_SWATCHES[source_material])
    identity = f"{source_material}:{semantic_class}".encode()
    digest = hashlib.sha256(identity).digest()
    # Unlisted manifest identities remain neutral instead of becoming debug-like random colors.
    shade = 0.18 + 0.2 * digest[0] / 255.0
    tint = 0.015 * (digest[1] / 255.0 - 0.5)
    return [round(shade + tint, 6), round(shade, 6), round(shade - tint, 6)]


def _profile_from_manifest_identity(
    source_material: str,
    semantic_class: str,
    realism: Mapping[str, object],
) -> tuple[str, str]:
    profile_by_source = realism["material_profile_by_source_material"]
    if not isinstance(profile_by_source, dict):
        raise RuntimeError("validated realism material index is unavailable")
    exact = profile_by_source.get(source_material)
    if isinstance(exact, str):
        return exact, "recipe_applies_to"
    identity = f"{source_material} {semantic_class}".lower()
    if "glass" in identity or "window" in identity:
        return "industrial_glass", "manifest_material_semantic"
    if any(token in identity for token in ("marking", "line_", "crossing", "checkpoint")):
        return "road_and_parking_paint", "manifest_material_semantic"
    if any(token in identity for token in ("asphalt", "rubber", "tire", "oil_stain", "sealed_crack")):
        return "asphalt_worn", "manifest_material_semantic"
    if any(token in identity for token in ("concrete", "curb", "bund", "foundation", "manhole", "drain")):
        return "concrete_weathered", "manifest_material_semantic"
    if any(token in identity for token in ("container", "shipping_")):
        return "shipping_container_weathered", "manifest_material_semantic"
    if any(token in identity for token in ("tank", "vessel")):
        return "tank_enamel_white", "manifest_material_semantic"
    if any(token in identity for token in ("factory_cladding", "corrugated_panel")):
        return "factory_cladding_blue", "manifest_material_semantic"
    if any(token in identity for token in ("safety", "bollard", "barrier_arm", "painted_steel")):
        return "painted_safety_steel", "manifest_material_semantic"
    if any(
        token in identity
        for token in (
            "steel",
            "metal",
            "pipe",
            "valve",
            "flange",
            "gutter",
            "downspout",
            "hvac",
            "forklift",
        )
    ):
        return "warehouse_galvanized", "manifest_material_semantic"
    if any(token in identity for token in ("gravel", "soil", "dust", "rough_pad")):
        return "compacted_gravel", "manifest_material_semantic"
    return "manifest_derived_fallback", "manifest_material_semantic"


def _resolve_pbr_surface(
    source_material: str,
    semantic_class: str,
    realism: Mapping[str, object],
) -> dict[str, object]:
    if not source_material or not semantic_class:
        raise RuntimeError("every Unreal placement must retain manifest material and semantic_class")
    profile_name, mapping = _profile_from_manifest_identity(source_material, semantic_class, realism)
    profiles = realism["material_profiles"]
    if not isinstance(profiles, dict):
        raise RuntimeError("validated realism profiles are unavailable")
    if profile_name == "manifest_derived_fallback":
        identity = f"{source_material} {semantic_class}".lower()
        metallic = 0.0
        roughness = 0.84
        if any(token in identity for token in ("plastic", "cone", "bin")):
            roughness = 0.66
        return {
            "source_material": source_material,
            "source_semantic_class": semantic_class,
            "profile": profile_name,
            "mapping": mapping,
            "base_color_source": (
                "manifest_material_swatch"
                if source_material in _MANIFEST_MATERIAL_SWATCHES
                else "neutral_manifest_identity_fallback"
            ),
            "workflow": "metallic_roughness",
            "base_color_srgb": _manifest_derived_base_color(source_material, semantic_class),
            "metallic": metallic,
            "roughness": roughness,
            "specular": 0.45,
            "normal_strength": 0.2,
        }
    profile = profiles.get(profile_name)
    if not isinstance(profile, dict):
        raise RuntimeError(f"manifest identity selected missing realism PBR profile {profile_name}")
    roughness_range = _finite_vector(profile.get("roughness_range"), 2, f"{profile_name}.roughness_range")
    color = profile.get("base_color_srgb")
    if source_material in _MANIFEST_MATERIAL_SWATCHES:
        base_color = list(_MANIFEST_MATERIAL_SWATCHES[source_material])
        base_color_source = "manifest_material_swatch"
    elif color is not None:
        base_color = list(_finite_vector(color, 3, f"{profile_name}.base_color_srgb"))
        base_color_source = "recipe_profile"
    else:
        base_color = _manifest_derived_base_color(source_material, semantic_class)
        base_color_source = "neutral_manifest_identity_fallback"
    channels = profile.get("channels")
    return {
        "source_material": source_material,
        "source_semantic_class": semantic_class,
        "profile": profile_name,
        "mapping": mapping,
        "base_color_source": base_color_source,
        "workflow": profile.get("workflow"),
        "base_color_srgb": base_color,
        "metallic": float(profile["metallic"]),
        "roughness": 0.5 * (roughness_range[0] + roughness_range[1]),
        "specular": 0.5 if profile_name != "industrial_glass" else 0.7,
        "normal_strength": 0.32 if isinstance(channels, list) and "normal" in channels else 0.0,
        "transmission": float(profile.get("transmission", 0.0)),
        "ior": float(profile.get("ior", 1.5)),
    }


def _same_numbers(left: object, right: object, size: int, context: str) -> bool:
    left_values = _finite_vector(left, size, f"{context}.left")
    right_values = _finite_vector(right, size, f"{context}.right")
    return all(math.isclose(a, b, rel_tol=0.0, abs_tol=1.0e-8) for a, b in zip(left_values, right_values))


def _validate_scene_record(
    record: object,
    context: str,
    *,
    mesh_required: bool,
) -> dict[str, object]:
    if not isinstance(record, dict):
        raise RuntimeError(f"{context} must be an object")
    stable_id = record.get("stable_id")
    if not isinstance(stable_id, str) or not stable_id.strip():
        raise RuntimeError(f"{context}.stable_id must be a non-empty string")
    mesh_name = record.get("mesh_name")
    if mesh_required and (not isinstance(mesh_name, str) or not mesh_name.strip()):
        raise RuntimeError(f"{context}.mesh_name must be a non-empty string")
    if not mesh_required and mesh_name is not None:
        raise RuntimeError(f"{context}.mesh_name must be null for a non-mesh feature descriptor")
    for identity_field in ("semantic_class", "shape", "material"):
        value = record.get(identity_field)
        if not isinstance(value, str) or not value:
            raise RuntimeError(f"{context}.{identity_field} must be a non-empty string")
    _finite_vector(record.get("position_m"), 3, f"{context}.position_m")
    _unit_quaternion(record.get("quaternion_wxyz"), f"{context}.quaternion_wxyz")
    scale = _finite_vector(record.get("scale"), 3, f"{context}.scale")
    if scale != (1.0, 1.0, 1.0):
        raise RuntimeError(f"{context}.scale must remain [1, 1, 1]")
    dimensions = _finite_vector(record.get("dimensions_m"), 3, f"{context}.dimensions_m")
    if any(value <= 0.0 for value in dimensions):
        raise RuntimeError(f"{context}.dimensions_m must be positive")
    if not isinstance(record.get("visual_only"), bool) or not isinstance(record.get("collision"), bool):
        raise RuntimeError(f"{context} must declare boolean visual_only and collision")
    return record


def _compare_layout_scene_record(
    scene_record: Mapping[str, object],
    layout_record: Mapping[str, object],
    context: str,
) -> None:
    if not _same_numbers(scene_record.get("position_m"), layout_record.get("position_m"), 3, context):
        raise RuntimeError(f"{context} position disagrees with expanded layout")
    shared_fields = ("semantic_class", "shape", "material", "collision", "visual_only")
    for field in shared_fields:
        if scene_record.get(field) != layout_record.get(field):
            raise RuntimeError(f"{context}.{field} disagrees with expanded layout")
    for angle_name in ("yaw_deg", "pitch_deg"):
        expected = float(layout_record.get(angle_name, 0.0))
        actual = float(scene_record.get(angle_name, 0.0))
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-8):
            raise RuntimeError(f"{context}.{angle_name} disagrees with expanded layout")
    if layout_record.get("shape") == "box":
        expected_dimensions = layout_record.get("size_m")
    else:
        radius = float(layout_record["radius_m"])
        half_height = float(layout_record["half_height_m"])
        expected_dimensions = [2.0 * radius, 2.0 * radius, 2.0 * half_height]
    if not _same_numbers(scene_record.get("dimensions_m"), expected_dimensions, 3, f"{context}.dimensions"):
        raise RuntimeError(f"{context}.dimensions_m disagrees with expanded layout")


def _validate_blender_realism_manifest(
    manifest: Mapping[str, object],
    scene: Mapping[str, object],
    placed_objects: Sequence[Mapping[str, object]],
    realism: Mapping[str, object],
) -> dict[str, object]:
    contract = manifest.get("realism")
    if not isinstance(contract, dict) or contract.get("profile") != REALISM_PROFILE:
        raise RuntimeError(f"Blender authoring must declare realism profile {REALISM_PROFILE}")
    if contract.get("seed") != realism.get("seed"):
        raise RuntimeError("Blender realism seed disagrees with the shared recipe")
    namespace = realism["authoring_contract"].get("dressing_id_namespace")  # type: ignore[union-attr]
    if contract.get("namespace") != namespace:
        raise RuntimeError("Blender realism dressing namespace disagrees with the shared recipe")
    recipe_record = contract.get("recipe")
    if not isinstance(recipe_record, dict):
        raise RuntimeError("Blender realism contract has no recipe file record")
    recipe_path = _resolve_beneath(
        recipe_record.get("path"), REPO_ROOT, REPO_ROOT, "Blender realism recipe"
    )
    if recipe_path != realism["recipe_path"]:
        raise RuntimeError("Blender realism contract references the wrong recipe")
    if (
        recipe_record.get("bytes") != realism["recipe_bytes"]
        or recipe_record.get("sha256") != realism["recipe_sha256"]
        or _sha256_file(recipe_path) != realism["recipe_sha256"]
    ):
        raise RuntimeError("Blender realism recipe file identity is stale")
    actor_budget = contract.get("actor_budget")
    if actor_budget != {"min": REALISM_ACTOR_BUDGET[0], "max": REALISM_ACTOR_BUDGET[1]}:
        raise RuntimeError(f"Blender realism actor budget must be {REALISM_ACTOR_BUDGET}")

    if scene.get("material_profile") != "procedural_pbr_v2":
        raise RuntimeError("Blender realism scene must use procedural_pbr_v2")
    material_count = scene.get("material_count")
    total_actor_count = scene.get("total_mesh_actor_count")
    if isinstance(material_count, bool) or not isinstance(material_count, int) or material_count <= 0:
        raise RuntimeError("Blender realism scene has an invalid material_count")
    unique_source_materials = {str(record["material"]) for record in placed_objects}
    if not len(realism["material_profiles"]) <= material_count <= len(unique_source_materials):  # type: ignore[arg-type]
        raise RuntimeError("Blender realism material_count is inconsistent with recipe/source materials")
    if total_actor_count != len(placed_objects) + 1:
        raise RuntimeError("Blender realism total_mesh_actor_count must include placements plus one terrain")
    if not REALISM_ACTOR_BUDGET[0] <= total_actor_count <= REALISM_ACTOR_BUDGET[1]:
        raise RuntimeError(
            f"Blender realism actor count {total_actor_count} lies outside {REALISM_ACTOR_BUDGET}"
        )
    if scene.get("actor_budget_status") != "within_budget":
        raise RuntimeError("Blender realism scene does not prove actor budget compliance")

    detail_counts = scene.get("detail_counts")
    if not isinstance(detail_counts, dict) or not detail_counts:
        raise RuntimeError("Blender realism scene has no detail_counts")
    for key, count in detail_counts.items():
        if not isinstance(key, str) or isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise RuntimeError("Blender realism detail_counts must be non-negative integer semantic counts")
    required_detail_groups = {
        "gate": {"security_camera", "barrier_arm", "impact_bollard"},
        "loading": {
            "roller_shutter_door",
            "dock_shelter",
            "dock_bumper",
            "wood_pallet",
            "parked_forklift",
        },
        "tank": {
            "tank_access_ladder",
            "tank_inspection_platform",
            "tank_valve_pipe",
            "tank_valve",
            "process_pipe",
        },
        "roads": {"asphalt_patch", "tire_mark", "manhole_cover", "storm_drain_grate"},
        "facades": {
            "corrugated_panel_seam",
            "roof_gutter",
            "downspout",
            "service_door",
            "hvac_unit",
        },
    }
    for group, semantic_keys in required_detail_groups.items():
        missing = sorted(key for key in semantic_keys if detail_counts.get(key, 0) <= 0)
        if missing:
            raise RuntimeError(f"Blender realism {group} detail acceptance is incomplete: {missing}")

    cameras = scene.get("cameras")
    if not isinstance(cameras, dict):
        raise RuntimeError("Blender realism scene has no camera records")
    preview_by_id = {target["id"]: target for target in realism["preview_targets"]}  # type: ignore[index]
    camera_mapping = {
        "site_aerial": "site_aerial",
        "robot_eye": "south_gate_robot_eye",
        "loading_dock": "loading_dock_robot_eye",
        "tank_area": "tank_farm_inspection",
    }
    validated_preview_targets: list[dict[str, object]] = []
    for manifest_camera, recipe_camera in camera_mapping.items():
        actual = cameras.get(manifest_camera)
        expected = preview_by_id[recipe_camera]
        if not isinstance(actual, dict):
            raise RuntimeError(f"Blender realism scene omits camera {manifest_camera}")
        position = _finite_vector(
            actual.get("position_m"),
            3,
            f"camera.{manifest_camera}.position_m",
        )
        look_at = _finite_vector(
            actual.get("look_at_m"),
            3,
            f"camera.{manifest_camera}.look_at_m",
        )
        lens_mm = _finite_vector(
            [actual.get("lens_mm")],
            1,
            f"camera.{manifest_camera}.lens_mm",
        )[0]
        if (
            actual.get("recipe_target") != recipe_camera
            or actual.get("output_basename") != expected.get("output_basename")
            or lens_mm <= 0.0
            or position == look_at
        ):
            raise RuntimeError(f"Blender camera {manifest_camera} has invalid preview-target provenance")
        recipe_position = _finite_vector(
            expected.get("position_m"),
            3,
            f"recipe camera {recipe_camera}.position_m",
        )
        recipe_look_at = _finite_vector(
            expected.get("look_at_m"),
            3,
            f"recipe camera {recipe_camera}.look_at_m",
        )
        adjusted = position != recipe_position or look_at != recipe_look_at or not math.isclose(
            lens_mm,
            float(expected["lens_mm"]),
            rel_tol=0.0,
            abs_tol=1.0e-8,
        )
        adjustment = actual.get("composition_adjustment")
        if adjusted and (not isinstance(adjustment, str) or not adjustment):
            raise RuntimeError(f"Blender camera {manifest_camera} changed composition without provenance")
        validated_preview_targets.append(
            {
                **expected,
                "position_m": list(position),
                "look_at_m": list(look_at),
                "lens_mm": lens_mm,
                "composition_adjustment": adjustment,
                "recipe_position_m": list(recipe_position),
                "recipe_look_at_m": list(recipe_look_at),
                "recipe_lens_mm": float(expected["lens_mm"]),
                "qa_adjusted": adjusted,
                "blender_camera_key": manifest_camera,
                "blender_preview_sha256": actual.get("output_sha256"),
            }
        )
    return {
        "profile": REALISM_PROFILE,
        "recipe": recipe_record,
        "seed": contract["seed"],
        "namespace": contract["namespace"],
        "actor_budget": actor_budget,
        "material_profile": scene["material_profile"],
        "material_count": material_count,
        "source_material_count": len(unique_source_materials),
        "detail_counts": detail_counts,
        "total_mesh_actor_count": total_actor_count,
        "actor_budget_status": scene["actor_budget_status"],
        "cameras": cameras,
        "preview_targets": validated_preview_targets,
    }


def _validate_blender_authoring(
    world: Mapping[str, object],
    realism: Mapping[str, object],
) -> dict[str, object]:
    manifest = _load_json_object(BLENDER_MANIFEST_PATH)
    if manifest.get("schema") != "lingtu.sim.blender-authoring-manifest.v1":
        raise RuntimeError(f"unsupported Blender authoring manifest schema: {manifest.get('schema')}")
    if manifest.get("world_package") != WORLD_PACKAGE:
        raise RuntimeError("Blender authoring manifest resolves the wrong world package")
    source_layout = manifest.get("source_layout")
    source_layout_path = _validate_file_record(
        "Blender source_layout", source_layout, REPO_ROOT, REPO_ROOT
    )
    if source_layout_path != world["layout_path"]:
        raise RuntimeError("Blender source_layout path disagrees with the world recipe")
    if source_layout.get("sha256") != world["layout_sha256"]:  # type: ignore[union-attr]
        raise RuntimeError("Blender source_layout SHA256 disagrees with the world recipe")
    if source_layout.get("layout_digest") != world["layout_digest"]:  # type: ignore[union-attr]
        raise RuntimeError("Blender source_layout semantic digest disagrees with the world recipe")

    coordinate_contract = manifest.get("coordinate_contract")
    if not isinstance(coordinate_contract, dict):
        raise RuntimeError("Blender authoring manifest has no coordinate_contract")
    source_coordinate = coordinate_contract.get("source")
    unreal_coordinate = coordinate_contract.get("unreal")
    fbx_coordinate = coordinate_contract.get("fbx")
    if not isinstance(source_coordinate, dict) or source_coordinate.get("frame") != "mujoco_rh_z_up_m":
        raise RuntimeError("Blender authoring source frame must be mujoco_rh_z_up_m")
    if (
        not isinstance(unreal_coordinate, dict)
        or unreal_coordinate.get("frame") != "unreal_lh_z_up_cm"
        or unreal_coordinate.get("position_mapping") != "(x,y,z)m -> (100*x,-100*y,100*z)cm"
        or unreal_coordinate.get("import_scale") != FBX_IMPORT_UNIFORM_SCALE
        or unreal_coordinate.get("import_uniform_scale") != FBX_IMPORT_UNIFORM_SCALE
        or unreal_coordinate.get("import_uniform_scale_supported") is not False
        or unreal_coordinate.get("fbx_actor_uniform_scale") != ENVIRONMENT_ACTOR_UNIT_SCALE
        or unreal_coordinate.get("terrain_actor_uniform_scale") != TERRAIN_ACTOR_UNIT_SCALE
        or unreal_coordinate.get("unit_conversion_strategy") != "placement_actor_scale"
    ):
        raise RuntimeError("Blender authoring manifest has the wrong Unreal position mapping")
    if not isinstance(fbx_coordinate, dict):
        raise RuntimeError("Blender authoring manifest has no FBX coordinate contract")
    if (
        unreal_coordinate.get("combine_meshes") is not False
        or fbx_coordinate.get("unreal_transform_vertex_to_absolute") is not False
        or fbx_coordinate.get("unreal_bake_pivot_in_vertex") is not False
        or fbx_coordinate.get("mesh_vertices") != "object-local"
        or fbx_coordinate.get("contains_authoritative_terrain") is not False
    ):
        raise RuntimeError("Blender FBX coordinate contract would combine, bake, or duplicate scene geometry")

    assets = manifest.get("assets")
    if not isinstance(assets, list) or not assets or not all(isinstance(item, dict) for item in assets):
        raise RuntimeError("Blender authoring manifest contains no valid assets")
    asset_paths: dict[str, Path] = {}
    for index, record in enumerate(assets):
        role = record.get("role")
        if not isinstance(role, str) or not role:
            raise RuntimeError(f"Blender asset {index} has no role")
        if role in asset_paths:
            raise RuntimeError(f"Blender authoring manifest repeats asset role {role}")
        asset_paths[role] = _validate_file_record(
            f"Blender assets[{index}]", record, BLENDER_MANIFEST_PATH.parent, BLENDER_MANIFEST_PATH.parent
        )
    actual_artifact_set_digest = _blender_artifact_set_digest(assets)
    if manifest.get("artifact_set_digest") != actual_artifact_set_digest:
        raise RuntimeError("Blender authoring manifest artifact_set_digest is invalid")
    scene_asset_role = "unreal_scene_fbx" if "unreal_scene_fbx" in asset_paths else "portable_scene_glb"
    if scene_asset_role not in asset_paths:
        raise RuntimeError("Blender authoring manifest provides neither unreal_scene_fbx nor portable_scene_glb")
    scene_asset_record = next(record for record in assets if record["role"] == scene_asset_role)
    if scene_asset_role == "unreal_scene_fbx":
        unreal_import = scene_asset_record.get("unreal_import")
        if (
            not isinstance(unreal_import, dict)
            or unreal_import.get("combine_meshes") is not False
            or unreal_import.get("transform_vertex_to_absolute") is not False
            or unreal_import.get("bake_pivot_in_vertex") is not False
            or unreal_import.get("import_uniform_scale") != FBX_IMPORT_UNIFORM_SCALE
            or unreal_import.get("import_uniform_scale_supported") is not False
            or scene_asset_record.get("mesh_vertices") != "object-local"
            or scene_asset_record.get("contains_authoritative_terrain") is not False
        ):
            raise RuntimeError("Unreal FBX asset record violates local-mesh or single-terrain authority")
        unreal_placement = scene_asset_record.get("unreal_placement")
        if (
            not isinstance(unreal_placement, dict)
            or unreal_placement.get("actor_uniform_scale") != ENVIRONMENT_ACTOR_UNIT_SCALE
            or unreal_placement.get("required") is not True
        ):
            raise RuntimeError("Unreal FBX asset record omits the required actor-scale placement contract")

    digest = _load_json_object(ARTIFACT_DIGEST_PATH)
    if digest.get("schema") != "lingtu.sim.blender-artifact-set-digest.v1":
        raise RuntimeError(f"unsupported Blender artifact digest schema: {digest.get('schema')}")
    digest_manifest = digest.get("manifest")
    if not isinstance(digest_manifest, dict):
        raise RuntimeError("Blender artifact digest has no manifest file record")
    digest_manifest_path = _validate_file_record(
        "Blender artifact digest manifest",
        digest_manifest,
        ARTIFACT_DIGEST_PATH.parent,
        ARTIFACT_DIGEST_PATH.parent,
    )
    if digest_manifest_path != BLENDER_MANIFEST_PATH:
        raise RuntimeError("Blender artifact digest points at the wrong manifest")
    if digest_manifest.get("sha256") != _sha256_file(BLENDER_MANIFEST_PATH):
        raise RuntimeError("Blender artifact digest has the wrong manifest_sha256")
    if digest.get("artifact_set_digest") != actual_artifact_set_digest:
        raise RuntimeError("Blender manifest and artifact digest disagree on artifact_set_digest")
    if digest.get("layout_sha256") != world["layout_sha256"]:
        raise RuntimeError("Blender artifact digest has the wrong layout_sha256")
    digest_assets = digest.get("assets")
    if not isinstance(digest_assets, list) or _blender_artifact_set_digest(digest_assets) != actual_artifact_set_digest:
        raise RuntimeError("Blender artifact digest assets do not reproduce artifact_set_digest")
    digest_identity = {
        (record.get("role"), record.get("path"), record.get("sha256")) for record in digest_assets
    }
    manifest_identity = {(record.get("role"), record.get("path"), record.get("sha256")) for record in assets}
    if digest_identity != manifest_identity:
        raise RuntimeError("Blender manifest and artifact digest asset identities disagree")

    scene = manifest.get("scene")
    if not isinstance(scene, dict):
        raise RuntimeError("Blender authoring manifest has no scene object")
    terrain_object = scene.get("terrain_object")
    terrain_objects = scene.get("terrain_objects")
    if not isinstance(terrain_object, dict) or terrain_objects != [terrain_object]:
        raise RuntimeError("Blender scene must trace exactly one authoritative terrain source")
    terrain_source_path = _resolve_beneath(
        terrain_object.get("source_path"), REPO_ROOT, REPO_ROOT, "Blender terrain source"
    )
    if (
        terrain_source_path != world["terrain"]["path"]  # type: ignore[index]
        or terrain_object.get("source_sha256") != world["terrain"]["sha256"]  # type: ignore[index]
        or terrain_object.get("physics_proxy") != "authoritative_heightfield"
        or terrain_object.get("extent_alignment_verified") is not True
    ):
        raise RuntimeError("Blender terrain provenance disagrees with the authoritative terrain OBJ")
    layout_records_value = scene.get("layout_objects")
    visual_records_value = scene.get("visual_only_objects")
    descriptor_records_value = scene.get("terrain_feature_descriptors")
    semantic_descriptor_records_value = scene.get("semantic_feature_descriptors")
    if not isinstance(layout_records_value, list) or not isinstance(visual_records_value, list):
        raise RuntimeError("Blender scene must contain layout_objects and visual_only_objects lists")
    if not isinstance(descriptor_records_value, list):
        raise RuntimeError("Blender scene must contain terrain_feature_descriptors")
    if (
        not isinstance(semantic_descriptor_records_value, list)
        or scene.get("semantic_descriptors_not_materialized") is not True
    ):
        raise RuntimeError(
            "Blender scene must preserve non-materialized semantic_feature_descriptors"
        )

    layout_records = [
        _validate_scene_record(record, f"scene.layout_objects[{index}]", mesh_required=True)
        for index, record in enumerate(layout_records_value)
    ]
    visual_records = [
        _validate_scene_record(record, f"scene.visual_only_objects[{index}]", mesh_required=True)
        for index, record in enumerate(visual_records_value)
    ]
    descriptor_records = [
        _validate_scene_record(record, f"scene.terrain_feature_descriptors[{index}]", mesh_required=False)
        for index, record in enumerate(descriptor_records_value)
    ]
    semantic_descriptor_records = [
        _validate_scene_record(
            record,
            f"scene.semantic_feature_descriptors[{index}]",
            mesh_required=False,
        )
        for index, record in enumerate(semantic_descriptor_records_value)
    ]
    layout_by_id = world["layout_by_id"]
    if not isinstance(layout_by_id, dict):
        raise RuntimeError("validated world layout index is unavailable")
    represented_layout_ids: set[str] = set()
    for index, record in enumerate(layout_records):
        stable_id = str(record["stable_id"])
        if stable_id in represented_layout_ids:
            raise RuntimeError(f"Blender scene repeats represented layout ID {stable_id}")
        layout_record = layout_by_id.get(stable_id)
        if not isinstance(layout_record, dict):
            raise RuntimeError(f"Blender layout object is not in expanded layout: {stable_id}")
        _compare_layout_scene_record(record, layout_record, f"scene.layout_objects[{index}]")
        expected_proxy = "none" if layout_record["visual_only"] else "authoritative_layout"
        if record.get("physics_proxy") != expected_proxy:
            raise RuntimeError(f"Blender layout object {stable_id} has the wrong physics_proxy")
        represented_layout_ids.add(stable_id)
    for index, record in enumerate(semantic_descriptor_records):
        stable_id = str(record["stable_id"])
        if stable_id in represented_layout_ids:
            raise RuntimeError(f"Blender scene repeats represented layout ID {stable_id}")
        layout_record = layout_by_id.get(stable_id)
        if not isinstance(layout_record, dict):
            raise RuntimeError(f"semantic feature descriptor is not in expanded layout: {stable_id}")
        _compare_layout_scene_record(
            record,
            layout_record,
            f"scene.semantic_feature_descriptors[{index}]",
        )
        if (
            record.get("semantic_class") != "semantic_checkpoint"
            or record.get("asset_key") != f"semantic/{stable_id}"
            or record.get("source") != "expanded_layout_semantic"
            or record.get("physics_proxy") != "none"
            or record.get("collision") is not False
            or record.get("visual_only") is not True
            or record.get("materialized") is not False
            or record.get("exported_mesh") is not False
            or record.get("blender_object") is not None
        ):
            raise RuntimeError(
                f"semantic feature descriptor {stable_id} must be VisualOnly metadata with no mesh/physics"
            )
        represented_layout_ids.add(stable_id)
    for index, record in enumerate(descriptor_records):
        stable_id = str(record["stable_id"])
        if stable_id in represented_layout_ids:
            raise RuntimeError(f"Blender scene repeats represented layout ID {stable_id}")
        layout_record = layout_by_id.get(stable_id)
        if not isinstance(layout_record, dict):
            raise RuntimeError(f"terrain feature descriptor is not in expanded layout: {stable_id}")
        _compare_layout_scene_record(record, layout_record, f"scene.terrain_feature_descriptors[{index}]")
        if record.get("physics_proxy") != "authoritative_heightfield":
            raise RuntimeError(f"terrain feature descriptor {stable_id} is not bound to the heightfield")
        if record.get("visual_only") or record.get("collision"):
            raise RuntimeError(f"terrain feature descriptor {stable_id} must not create an independent mesh")
        represented_layout_ids.add(stable_id)
    if represented_layout_ids != set(layout_by_id):
        missing = sorted(set(layout_by_id).difference(represented_layout_ids))
        extra = sorted(represented_layout_ids.difference(layout_by_id))
        raise RuntimeError(f"Blender scene does not represent the exact expanded layout: missing={missing}, extra={extra}")

    placed_ids = set(represented_layout_ids)
    for record in visual_records:
        stable_id = str(record["stable_id"])
        if stable_id in placed_ids:
            raise RuntimeError(f"Blender scene repeats stable ID {stable_id}")
        if (
            record.get("source") != "blender_derived_visual"
            or record.get("physics_proxy") != "none"
            or record.get("collision") is not False
            or record.get("visual_only") is not True
        ):
            raise RuntimeError(f"derived Blender visual {stable_id} violates VisualOnly authority")
        placed_ids.add(stable_id)
    mesh_names = [str(record["mesh_name"]) for record in (*layout_records, *visual_records)]
    if len(mesh_names) != len(set(mesh_names)):
        raise RuntimeError("Blender scene contains duplicate mesh names")
    placed_objects = [*layout_records, *visual_records]
    realism_manifest = _validate_blender_realism_manifest(
        manifest,
        scene,
        placed_objects,
        realism,
    )
    return {
        "manifest": manifest,
        "manifest_path": BLENDER_MANIFEST_PATH,
        "manifest_sha256": _sha256_file(BLENDER_MANIFEST_PATH),
        "artifact_digest_path": ARTIFACT_DIGEST_PATH,
        "artifact_digest_sha256": _sha256_file(ARTIFACT_DIGEST_PATH),
        "artifact_set_digest": actual_artifact_set_digest,
        "scene_asset_role": scene_asset_role,
        "scene_asset_path": asset_paths[scene_asset_role],
        "layout_objects": layout_records,
        "visual_only_objects": visual_records,
        "terrain_feature_descriptors": descriptor_records,
        "semantic_feature_descriptors": semantic_descriptor_records,
        "semantic_descriptors_not_materialized": True,
        "placed_objects": placed_objects,
        "terrain_object": terrain_object,
        "realism": realism_manifest,
    }


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f"{path.name}.tmp")
    temporary.write_bytes(_canonical_json(payload))
    temporary.replace(path)


def _set_editor_property_if_supported(target: object, property_name: str, value: object) -> None:
    try:
        target.set_editor_property(property_name, value)
    except Exception as error:
        unreal.log_warning(f"FactoryPark_HF property unavailable: {property_name} ({error})")


def _read_collision_component_policy(mesh_component: object) -> dict[str, object]:
    collision_enabled = mesh_component.get_collision_enabled()
    collision_profile_name = str(mesh_component.get_collision_profile_name())
    generate_overlap_events = bool(
        mesh_component.get_editor_property("generate_overlap_events")
    )
    expected_no_collision = unreal.CollisionEnabled.NO_COLLISION
    return {
        "collision_enabled": str(collision_enabled),
        "collision_enabled_repr": repr(collision_enabled),
        "collision_expected": str(expected_no_collision),
        "collision_expected_repr": repr(expected_no_collision),
        "collision_disabled": collision_enabled == expected_no_collision,
        "collision_profile_name": collision_profile_name,
        "generate_overlap_events": generate_overlap_events,
        "verified": (
            collision_enabled == expected_no_collision
            and collision_profile_name == "NoCollision"
            and generate_overlap_events is False
        ),
    }


def _require_collision_component_policy(
    readback: Mapping[str, object],
    context: str,
) -> None:
    if readback.get("verified") is not True:
        raise RuntimeError(
            f"FactoryPark_HF collision policy readback failed for {context}: "
            f"{json.dumps(dict(readback), sort_keys=True)}"
        )


def _disable_collision(mesh_component: object) -> dict[str, object]:
    before = _read_collision_component_policy(mesh_component)
    modified = mesh_component.modify(True)
    mesh_component.set_collision_profile_name("NoCollision")
    mesh_component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    mesh_component.set_editor_property("generate_overlap_events", False)
    after = _read_collision_component_policy(mesh_component)
    _require_collision_component_policy(after, str(mesh_component))
    return {
        "before": before,
        "after": after,
        "component_modify_called": True,
        "component_modify_returned": (
            modified if isinstance(modified, (bool, type(None))) else str(modified)
        ),
        "collision_enabled_changed": before["collision_disabled"] is not True,
        "collision_profile_changed": before["collision_profile_name"] != "NoCollision",
        "generate_overlap_events_changed": before["generate_overlap_events"] is not False,
    }


def _disable_static_mesh_asset_collision(mesh: object) -> None:
    subsystem = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    if not subsystem.remove_collisions(mesh):
        raise RuntimeError(f"could not remove UE collision geometry from {mesh.get_path_name()}")
    unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.UeCollision", "Disabled")


def _create_or_load_asset(asset_path: str, asset_name: str, asset_class: object, factory: object) -> object:
    existing = unreal.load_asset(f"{asset_path}/{asset_name}")
    if existing is not None:
        if not isinstance(existing, asset_class):
            raise RuntimeError(f"existing Unreal asset has the wrong class: {asset_path}/{asset_name}")
        return existing
    created = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        asset_name,
        asset_path,
        asset_class,
        factory,
    )
    if created is None:
        raise RuntimeError(f"could not create Unreal asset {asset_path}/{asset_name}")
    return created


def _connect_material_expression_checked(
    library: object,
    label: str,
    from_expression: object,
    from_output_name: str,
    to_expression: object,
    to_input_name: str,
) -> None:
    input_names = list(library.get_material_expression_input_names(to_expression))
    output_names = list(library.get_material_expression_output_names(from_expression))
    connected = library.connect_material_expressions(
        from_expression,
        from_output_name,
        to_expression,
        to_input_name,
    )
    unreal.log(
        "LINGTU_FACTORY_PARK_HF_MATERIAL_CONNECTION "
        f"label={label} connected={connected} outputs={output_names} inputs={input_names}"
    )
    if not connected:
        raise RuntimeError(
            "could not connect native procedural PBR node "
            f"{label}: requested_output={from_output_name!r} outputs={output_names} "
            f"requested_input={to_input_name!r} inputs={input_names}"
        )


def _configure_master_material_usages(
    material: object,
    library: object | None = None,
) -> dict[str, bool]:
    """Compile the master material for every geometry path used by this world."""

    material_library = library or unreal.MaterialEditingLibrary
    required_usages = {
        "InstancedStaticMeshes": unreal.MaterialUsage.MATUSAGE_INSTANCED_STATIC_MESHES,
        "Nanite": unreal.MaterialUsage.MATUSAGE_NANITE,
    }
    evidence: dict[str, bool] = {}
    for label, usage in required_usages.items():
        material_library.set_base_material_usage(material, usage, True)
        enabled = bool(material_library.has_material_usage(material, usage))
        if not enabled:
            raise RuntimeError(
                f"Unreal native PBR master material did not retain {label} usage"
            )
        evidence[label] = enabled
    return evidence


def _build_master_pbr_material() -> tuple[object, dict[str, object]]:
    material = _create_or_load_asset(
        MATERIAL_ASSET_ROOT,
        MASTER_MATERIAL_NAME,
        unreal.Material,
        unreal.MaterialFactoryNew(),
    )
    library = unreal.MaterialEditingLibrary
    library.delete_all_material_expressions(material)
    base_color = library.create_material_expression(
        material,
        unreal.MaterialExpressionVectorParameter,
        -520,
        -180,
    )
    base_color.set_editor_property("parameter_name", unreal.Name("BaseColor"))
    base_color.set_editor_property("default_value", unreal.LinearColor(0.3, 0.3, 0.3, 1.0))
    metallic = library.create_material_expression(
        material,
        unreal.MaterialExpressionScalarParameter,
        -520,
        -40,
    )
    metallic.set_editor_property("parameter_name", unreal.Name("Metallic"))
    metallic.set_editor_property("default_value", 0.0)
    roughness = library.create_material_expression(
        material,
        unreal.MaterialExpressionScalarParameter,
        -520,
        80,
    )
    roughness.set_editor_property("parameter_name", unreal.Name("Roughness"))
    roughness.set_editor_property("default_value", 0.75)
    specular = library.create_material_expression(
        material,
        unreal.MaterialExpressionScalarParameter,
        -520,
        200,
    )
    specular.set_editor_property("parameter_name", unreal.Name("Specular"))
    specular.set_editor_property("default_value", 0.5)
    roughness_saturate = library.create_material_expression(
        material,
        unreal.MaterialExpressionSaturate,
        -220,
        80,
    )
    expression_connections = (
        ("roughness_to_saturate", roughness, "", roughness_saturate, ""),
    )
    for connection in expression_connections:
        _connect_material_expression_checked(library, *connection)
    property_connections = (
        ("base_color", base_color, unreal.MaterialProperty.MP_BASE_COLOR),
        ("metallic", metallic, unreal.MaterialProperty.MP_METALLIC),
        ("roughness", roughness_saturate, unreal.MaterialProperty.MP_ROUGHNESS),
        ("specular", specular, unreal.MaterialProperty.MP_SPECULAR),
    )
    for label, expression, property_ in property_connections:
        connected = library.connect_material_property(expression, "", property_)
        unreal.log(
            "LINGTU_FACTORY_PARK_HF_MATERIAL_OUTPUT "
            f"label={label} connected={connected} property={property_}"
        )
        if not connected:
            raise RuntimeError(f"could not connect native PBR material output {label}")
    material_usages = _configure_master_material_usages(material, library)
    library.layout_material_expressions(material)
    compiler_errors = list(library.recompile_material(material))
    if compiler_errors:
        raise RuntimeError(f"Unreal native PBR master material did not compile: {compiler_errors}")
    unreal.EditorAssetLibrary.set_metadata_tag(material, "LingTu.RealismProfile", REALISM_PROFILE)
    unreal.EditorAssetLibrary.set_metadata_tag(material, "LingTu.MaterialAuthority", "UnrealNativePBR")
    unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False)
    return material, {
        "roughness_path": "clamped_parameter",
        "world_position_noise_used": False,
        "expression_connections": [connection[0] for connection in expression_connections],
        "expression_connection_count": len(expression_connections),
        "material_outputs": [connection[0] for connection in property_connections],
        "material_output_count": len(property_connections),
        "material_usages": material_usages,
        "compiled": True,
    }


def _material_instance_name(source_material: str, semantic_class: str) -> str:
    identity = f"{source_material}:{semantic_class}"
    readable = re.sub(r"[^A-Za-z0-9_]+", "_", identity).strip("_")[:72]
    suffix = hashlib.sha256(identity.encode("utf-8")).hexdigest()[:10]
    return f"MI_FP_{readable}_{suffix}"


def _srgb_channel_to_linear(channel: float) -> float:
    return channel / 12.92 if channel <= 0.04045 else ((channel + 0.055) / 1.055) ** 2.4


def _create_pbr_material_instance(
    master: object,
    surface: Mapping[str, object],
) -> tuple[object, dict[str, object]]:
    name = _material_instance_name(
        str(surface["source_material"]),
        str(surface["source_semantic_class"]),
    )
    instance = _create_or_load_asset(
        MATERIAL_ASSET_ROOT,
        name,
        unreal.MaterialInstanceConstant,
        unreal.MaterialInstanceConstantFactoryNew(),
    )
    library = unreal.MaterialEditingLibrary
    library.clear_all_material_instance_parameters(instance)
    library.set_material_instance_parent(instance, master)
    library.update_material_instance(instance)
    color = _finite_vector(surface["base_color_srgb"], 3, f"{name}.base_color_srgb")
    linear_color = tuple(_srgb_channel_to_linear(channel) for channel in color)
    expected_base_color = [linear_color[0], linear_color[1], linear_color[2], 1.0]
    base_color_api_return = library.set_material_instance_vector_parameter_value(
        instance,
        unreal.Name("BaseColor"),
        unreal.LinearColor(linear_color[0], linear_color[1], linear_color[2], 1.0),
    )
    base_color_readback = library.get_material_instance_vector_parameter_value(
        instance,
        unreal.Name("BaseColor"),
    )
    actual_base_color = [
        float(base_color_readback.r),
        float(base_color_readback.g),
        float(base_color_readback.b),
        float(base_color_readback.a),
    ]
    if not all(
        math.isclose(actual, expected, rel_tol=1e-6, abs_tol=1e-6)
        for actual, expected in zip(actual_base_color, expected_base_color, strict=True)
    ):
        raise RuntimeError(
            f"BaseColor readback mismatch on native PBR instance {name}: "
            f"expected={expected_base_color} actual={actual_base_color}"
        )
    scalar_evidence: dict[str, object] = {}
    for parameter, key in (
        ("Metallic", "metallic"),
        ("Roughness", "roughness"),
        ("Specular", "specular"),
    ):
        expected = float(surface[key])
        api_return = library.set_material_instance_scalar_parameter_value(
            instance,
            unreal.Name(parameter),
            expected,
        )
        actual = float(
            library.get_material_instance_scalar_parameter_value(
                instance,
                unreal.Name(parameter),
            )
        )
        if not math.isclose(actual, expected, rel_tol=1e-6, abs_tol=1e-6):
            raise RuntimeError(
                f"{parameter} readback mismatch on native PBR instance {name}: "
                f"expected={expected} actual={actual}"
            )
        scalar_evidence[parameter] = {
            "api_return": bool(api_return),
            "expected": expected,
            "actual": actual,
            "verified": True,
        }
    library.update_material_instance(instance)
    unreal.EditorAssetLibrary.set_metadata_tag(
        instance, "LingTu.SourceMaterial", str(surface["source_material"])
    )
    unreal.EditorAssetLibrary.set_metadata_tag(
        instance, "LingTu.SourceSemanticClass", str(surface["source_semantic_class"])
    )
    unreal.EditorAssetLibrary.set_metadata_tag(instance, "LingTu.PbrProfile", str(surface["profile"]))
    unreal.EditorAssetLibrary.set_metadata_tag(instance, "LingTu.RealismProfile", REALISM_PROFILE)
    unreal.EditorAssetLibrary.save_loaded_asset(instance, only_if_is_dirty=False)
    return instance, {
        "contract": "verified_by_getter_readback_due_ue58_false_return_value",
        "all_readbacks_verified": True,
        "vector": {
            "BaseColor": {
                "api_return": bool(base_color_api_return),
                "expected": expected_base_color,
                "actual": actual_base_color,
                "verified": True,
            }
        },
        "scalars": scalar_evidence,
    }


def _native_pbr_identities(authoring: Mapping[str, object]) -> set[tuple[str, str]]:
    identities = {
        (str(record["material"]), str(record["semantic_class"]))
        for record in authoring["placed_objects"]  # type: ignore[index]
    }
    identities.add(("gravel", "terrain"))
    return identities


def _expected_native_pbr_instance_count(authoring: Mapping[str, object]) -> int:
    return len(_native_pbr_identities(authoring))


def _build_native_pbr_material_library(
    authoring: Mapping[str, object],
    realism: Mapping[str, object],
) -> tuple[dict[tuple[str, str], object], object, dict[str, object]]:
    master, master_graph = _build_master_pbr_material()
    identities = _native_pbr_identities(authoring)
    instances: dict[tuple[str, str], object] = {}
    evidence: list[dict[str, object]] = []
    for source_material, semantic_class in sorted(identities):
        surface = _resolve_pbr_surface(source_material, semantic_class, realism)
        instance, parameter_write_evidence = _create_pbr_material_instance(master, surface)
        instances[(source_material, semantic_class)] = instance
        linear_color = [
            _srgb_channel_to_linear(channel)
            for channel in _finite_vector(
                surface["base_color_srgb"],
                3,
                f"{source_material}.{semantic_class}.base_color_srgb",
            )
        ]
        evidence.append(
            {
                **surface,
                "base_color_linear": linear_color,
                "material_asset": str(instance.get_path_name()),
                "parameter_write_evidence": parameter_write_evidence,
            }
        )
    return instances, master, {
        "authority": "UnrealNativePBR",
        "fbx_material_networks_used": False,
        "master_material": str(master.get_path_name()),
        "master_graph": master_graph,
        "instance_count": len(instances),
        "source_identity_count": len(identities),
        "parameter_readback_verified_count": len(instances),
        "parameter_write_contract": "verified_by_getter_readback_due_ue58_false_return_value",
        "base_color_policy": "manifest_material_swatch_then_neutral_identity_fallback",
        "materials": evidence,
    }


def _mesh_material_slot_count(mesh: object) -> int:
    try:
        slots = mesh.get_editor_property("static_materials")
        return max(1, len(slots))
    except Exception:
        return 1


def _assign_static_mesh_material(mesh: object, material: object, surface: Mapping[str, object]) -> int:
    slot_count = _mesh_material_slot_count(mesh)
    for material_index in range(slot_count):
        mesh.set_material(material_index, material)
    unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.MaterialAuthority", "UnrealNativePBR")
    unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.SourceMaterial", str(surface["source_material"]))
    unreal.EditorAssetLibrary.set_metadata_tag(
        mesh, "LingTu.SourceSemanticClass", str(surface["source_semantic_class"])
    )
    unreal.EditorAssetLibrary.save_loaded_asset(mesh, only_if_is_dirty=False)
    return slot_count


def _assign_component_material(component: object, mesh: object, material: object) -> int:
    slot_count = _mesh_material_slot_count(mesh)
    for material_index in range(slot_count):
        component.set_material(material_index, material)
    return slot_count


def _configure_optional_nanite(
    meshes: Sequence[object],
    realism: Mapping[str, object],
) -> dict[str, object]:
    budgets = realism["performance_budgets"]
    requested = budgets.get("enable_nanite", False) is True  # type: ignore[union-attr]
    if not requested:
        return {
            "requested": False,
            "supported": None,
            "enabled_mesh_count": 0,
            "failed_mesh_count": 0,
            "reason": "shared realism recipe does not require Nanite",
        }
    enabled = 0
    failed: list[dict[str, str]] = []
    for mesh in meshes:
        try:
            settings = mesh.get_editor_property("nanite_settings")
            settings.set_editor_property("enabled", True)
            mesh.set_editor_property("nanite_settings", settings)
            unreal.EditorAssetLibrary.save_loaded_asset(mesh, only_if_is_dirty=False)
            enabled += 1
        except Exception as error:
            failed.append({"mesh": str(mesh.get_path_name()), "reason": str(error)})
    return {
        "requested": True,
        "supported": enabled > 0,
        "enabled_mesh_count": enabled,
        "failed_mesh_count": len(failed),
        "failures": failed,
        "non_fatal": True,
    }


def _configure_fbx_import_task(source: Path) -> object:
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", str(source))
    task.set_editor_property("destination_path", BLENDER_MESH_DESTINATION)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("replace_existing_settings", True)
    task.set_editor_property("save", True)
    task.set_editor_property("async_", False)

    options = unreal.FbxImportUI()
    options.set_editor_property("import_mesh", True)
    options.set_editor_property("import_as_skeletal", False)
    # Geometry is authored in Blender; Unreal is the sole runtime PBR material authority.
    options.set_editor_property("import_materials", False)
    options.set_editor_property("import_textures", False)
    options.set_editor_property("create_physics_asset", False)
    options.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
    static_mesh_data = options.get_editor_property("static_mesh_import_data")
    static_mesh_data.set_editor_property("combine_meshes", False)
    static_mesh_data.set_editor_property("transform_vertex_to_absolute", False)
    static_mesh_data.set_editor_property("bake_pivot_in_vertex", False)
    static_mesh_data.set_editor_property("auto_generate_collision", False)
    static_mesh_data.set_editor_property("convert_scene", True)
    static_mesh_data.set_editor_property("convert_scene_unit", True)
    static_mesh_data.set_editor_property("import_uniform_scale", FBX_IMPORT_UNIFORM_SCALE)
    task.set_editor_property("options", options)
    return task


def _import_blender_scene(authoring: Mapping[str, object]) -> dict[str, object]:
    source = authoring["scene_asset_path"]
    if not isinstance(source, Path):
        raise RuntimeError("validated Blender scene path is unavailable")
    if authoring["scene_asset_role"] == "unreal_scene_fbx":
        task = _configure_fbx_import_task(source)
    else:
        task = unreal.AssetImportTask()
        task.set_editor_property("filename", str(source))
        task.set_editor_property("destination_path", BLENDER_MESH_DESTINATION)
        task.set_editor_property("automated", True)
        task.set_editor_property("replace_existing", True)
        task.set_editor_property("replace_existing_settings", True)
        task.set_editor_property("save", True)
        task.set_editor_property("async_", False)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])

    asset_paths = list(
        unreal.EditorAssetLibrary.list_assets(
            BLENDER_MESH_DESTINATION,
            recursive=True,
            include_folder=False,
        )
    )
    meshes_by_name: dict[str, object] = {}
    for asset_path in asset_paths:
        candidate = unreal.load_asset(str(asset_path))
        if candidate is None or not isinstance(candidate, unreal.StaticMesh):
            continue
        name = str(candidate.get_name())
        if name in meshes_by_name and meshes_by_name[name].get_path_name() != candidate.get_path_name():
            raise RuntimeError(f"Blender import produced duplicate StaticMesh name {name}")
        meshes_by_name[name] = candidate
    missing = [
        str(record["mesh_name"])
        for record in authoring["placed_objects"]  # type: ignore[index]
        if str(record["mesh_name"]) not in meshes_by_name
    ]
    if missing:
        raise RuntimeError(
            "Blender scene import is missing stable meshes: "
            f"{missing}; reported={task.get_editor_property('imported_object_paths')}"
        )
    for mesh in meshes_by_name.values():
        _disable_static_mesh_asset_collision(mesh)
        unreal.EditorAssetLibrary.set_metadata_tag(
            mesh, "LingTu.BlenderArtifactSetDigest", str(authoring["artifact_set_digest"])
        )
        unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.CollisionAuthority", "MuJoCo")
        unreal.EditorAssetLibrary.save_loaded_asset(mesh, only_if_is_dirty=False)
    return meshes_by_name


def _stage_terrain_obj(world: Mapping[str, object]) -> tuple[Path, str]:
    terrain = world["terrain"]
    if not isinstance(terrain, dict) or not isinstance(terrain.get("path"), Path):
        raise RuntimeError("validated terrain OBJ path is unavailable")
    source = terrain["path"]
    width, height = terrain["grid"]
    preface_and_vertices: list[str] = []
    faces: list[tuple[int, ...]] = []
    with source.open("r", encoding="ascii") as stream:
        for line in stream:
            stripped = line.rstrip("\r\n")
            if stripped.startswith("f "):
                indices = tuple(int(token.split("/", 1)[0]) for token in stripped.split()[1:])
                if len(indices) != 3 or any(index <= 0 for index in indices):
                    raise RuntimeError("terrain OBJ contains a non-triangle or non-positive face")
                faces.append(indices)
            elif not stripped.startswith(("vt ", "vn ")):
                preface_and_vertices.append(stripped)
    vertex_count = sum(line.startswith("v ") for line in preface_and_vertices)
    if vertex_count != width * height:
        raise RuntimeError("terrain OBJ staging vertex count disagrees with validated grid")
    output = list(preface_and_vertices)
    output.append(f"# authoritative_source_sha256={terrain['sha256']}")
    output.append("# ue58_interchange_uv_contract=vertex_index_equals_uv_index")
    for row in range(height):
        v = row / (height - 1)
        for column in range(width):
            u = column / (width - 1)
            output.append(f"vt {u:.9f} {v:.9f}")
    output.extend("f " + " ".join(f"{index}/{index}" for index in face) for face in faces)
    EVIDENCE_ROOT.mkdir(parents=True, exist_ok=True)
    staged_path = EVIDENCE_ROOT / "FactoryPark_HF_UE58_import.obj"
    staged_path.write_text("\n".join(output) + "\n", encoding="ascii")
    return staged_path, _sha256_file(staged_path)


def _import_terrain(world: Mapping[str, object]) -> tuple[object, Path, str]:
    staged_path, staged_digest = _stage_terrain_obj(world)
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", str(staged_path))
    task.set_editor_property("destination_path", TERRAIN_MESH_DESTINATION)
    task.set_editor_property("destination_name", TERRAIN_MESH_NAME)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("replace_existing_settings", True)
    task.set_editor_property("save", True)
    task.set_editor_property("async_", False)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    mesh = unreal.load_asset(TERRAIN_MESH_PATH)
    if mesh is None or not isinstance(mesh, unreal.StaticMesh):
        raise RuntimeError(
            f"Unreal did not import FactoryPark_HF terrain from {staged_path}: "
            f"{task.get_editor_property('imported_object_paths')}"
        )
    bounds = mesh.get_bounding_box()
    size_x = bounds.max.x - bounds.min.x
    size_y = bounds.max.y - bounds.min.y
    expected_x = world["terrain"]["bounds_cm"][1] - world["terrain"]["bounds_cm"][0]  # type: ignore[index]
    expected_y = world["terrain"]["bounds_cm"][3] - world["terrain"]["bounds_cm"][2]  # type: ignore[index]
    if not (
        math.isclose(size_x, expected_x, rel_tol=0.0, abs_tol=1.0)
        and math.isclose(size_y, expected_y, rel_tol=0.0, abs_tol=1.0)
    ):
        raise RuntimeError(
            "terrain import violated the 1:1 centimetre contract: "
            f"bounds=({size_x}, {size_y}), expected=({expected_x}, {expected_y})"
        )
    unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.SourceSha256", str(world["terrain"]["sha256"]))  # type: ignore[index]
    unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.ImportScale", "1.0")
    unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.CollisionAuthority", "MuJoCo")
    _disable_static_mesh_asset_collision(mesh)
    unreal.EditorAssetLibrary.save_loaded_asset(mesh, only_if_is_dirty=False)
    return mesh, staged_path, staged_digest


def _validate_local_mesh_contract(mesh: object, record: Mapping[str, object]) -> dict[str, object]:
    bounds = mesh.get_bounding_box()
    size_uu = (
        bounds.max.x - bounds.min.x,
        bounds.max.y - bounds.min.y,
        bounds.max.z - bounds.min.z,
    )
    center_uu = (
        0.5 * (bounds.max.x + bounds.min.x),
        0.5 * (bounds.max.y + bounds.min.y),
        0.5 * (bounds.max.z + bounds.min.z),
    )
    authored_dimensions_m = _finite_vector(record["dimensions_m"], 3, "dimensions_m")
    for actual, expected in zip(size_uu, authored_dimensions_m):
        tolerance = max(0.001, 0.01 * expected)
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance):
            raise RuntimeError(
                f"StaticMesh {record['mesh_name']} no longer preserves authored metre numeric dimensions: "
                f"actual_uu={size_uu}, expected_metre_numeric={authored_dimensions_m}"
            )
    if any(abs(value) > 0.01 for value in center_uu):
        raise RuntimeError(
            f"StaticMesh {record['mesh_name']} has baked world translation {center_uu}; "
            "transform_vertex_to_absolute must remain false"
        )
    expected_world_dimensions_cm = tuple(
        ENVIRONMENT_ACTOR_UNIT_SCALE * value for value in authored_dimensions_m
    )
    world_dimensions_cm = tuple(ENVIRONMENT_ACTOR_UNIT_SCALE * value for value in size_uu)
    for actual, expected in zip(world_dimensions_cm, expected_world_dimensions_cm):
        tolerance = max(0.1, 0.01 * expected)
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance):
            raise RuntimeError(
                f"StaticMesh {record['mesh_name']} actor-scale world dimensions are invalid: "
                f"actual_cm={world_dimensions_cm}, expected_cm={expected_world_dimensions_cm}"
            )
    return {
        "mesh_asset_dimensions_uu": list(size_uu),
        "mesh_asset_center_uu": list(center_uu),
        "expected_world_dimensions_cm": list(expected_world_dimensions_cm),
        "world_dimensions_cm": list(world_dimensions_cm),
    }


def _clear_current_level(actor_subsystem: object) -> None:
    for actor in actor_subsystem.get_all_level_actors():
        if not actor_subsystem.destroy_actor(actor):
            label_getter = getattr(actor, "get_actor_label", None)
            label = label_getter() if callable(label_getter) else str(actor)
            raise RuntimeError(f"FactoryPark_HF could not destroy actor {label}")


def _spawn_static_mesh_actor(
    actor_subsystem: object,
    mesh: object,
    label: str,
    transform: object,
    tags: Sequence[str],
    material: object | None = None,
    *,
    cast_shadow: bool = True,
) -> object:
    actor = actor_subsystem.spawn_actor_from_class(
        unreal.StaticMeshActor,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    if actor is None:
        raise RuntimeError(f"failed to spawn FactoryPark_HF StaticMeshActor {label}")
    actor.set_actor_label(label)
    actor.set_actor_transform(transform, False, True)
    actor.set_editor_property("tags", [unreal.Name(tag) for tag in tags])
    component = actor.get_editor_property("static_mesh_component")
    component.set_mobility(unreal.ComponentMobility.STATIC)
    component.set_static_mesh(mesh)
    component.set_editor_property("cast_shadow", cast_shadow)
    if material is not None:
        _assign_component_material(component, mesh, material)
    _disable_collision(component)
    return actor


def _spawn_terrain(
    actor_subsystem: object,
    mesh: object,
    terrain_digest: str,
    material: object,
) -> object:
    actor = _spawn_static_mesh_actor(
        actor_subsystem,
        mesh,
        "FactoryPark_HF_Terrain_SameSource",
        unreal.Transform(
            location=unreal.Vector(0.0, 0.0, 0.0),
            rotation=unreal.Rotator(0.0, 0.0, 0.0),
            scale=unreal.Vector(1.0, 1.0, 1.0),
        ),
        [WORLD_TAG, PHYSICS_SHARED_TAG, TERRAIN_TAG, terrain_digest],
        material,
    )
    scale = actor.get_actor_scale3d()
    if not all(math.isclose(value, 1.0) for value in (scale.x, scale.y, scale.z)):
        raise RuntimeError("FactoryPark_HF terrain actor must remain at unit scale")
    return actor


def _record_to_unreal_transform(
    record: Mapping[str, object],
    mesh_contract: Mapping[str, object],
) -> tuple[object, dict[str, object]]:
    location_cm = source_to_unreal_location_cm(record["position_m"])  # type: ignore[arg-type]
    quaternion_xyzw = source_to_unreal_quaternion_xyzw(record["quaternion_wxyz"])  # type: ignore[arg-type]
    source_scale = _finite_vector(record["scale"], 3, f"{record['stable_id']}.scale")
    actor_scale = source_scale_to_unreal_actor_scale(source_scale)
    transform = unreal.Transform(
        location=unreal.Vector(*location_cm),
        rotation=unreal.Quat(*quaternion_xyzw).rotator(),
        scale=unreal.Vector(*actor_scale),
    )
    evidence = {
        "stable_id": record["stable_id"],
        "mesh_name": record["mesh_name"],
        "source_position_m": list(record["position_m"]),
        "source_quaternion_wxyz": list(record["quaternion_wxyz"]),
        "location_cm": list(location_cm),
        "quaternion_xyzw": list(quaternion_xyzw),
        "source_scale": list(source_scale),
        "scale": list(actor_scale),
        **mesh_contract,
    }
    return transform, evidence


def _spawn_environment_objects(
    actor_subsystem: object,
    authoring: Mapping[str, object],
    meshes_by_name: Mapping[str, object],
    realism: Mapping[str, object],
    materials_by_identity: Mapping[tuple[str, str], object],
) -> tuple[list[object], list[dict[str, object]], list[dict[str, object]]]:
    actors: list[object] = []
    transforms: list[dict[str, object]] = []
    assignments: list[dict[str, object]] = []
    for record in authoring["placed_objects"]:  # type: ignore[index]
        mesh_name = str(record["mesh_name"])
        mesh = meshes_by_name[mesh_name]
        source_material = str(record["material"])
        semantic_class = str(record["semantic_class"])
        identity = (source_material, semantic_class)
        material = materials_by_identity[identity]
        surface = _resolve_pbr_surface(source_material, semantic_class, realism)
        mesh_slot_count = _assign_static_mesh_material(mesh, material, surface)
        mesh_contract = _validate_local_mesh_contract(mesh, record)
        transform, transform_evidence = _record_to_unreal_transform(record, mesh_contract)
        authority_tag = VISUAL_ONLY_TAG if record["visual_only"] else PHYSICS_SHARED_TAG
        actor = _spawn_static_mesh_actor(
            actor_subsystem,
            mesh,
            f"FactoryPark_{record['stable_id']}",
            transform,
            [
                WORLD_TAG,
                authority_tag,
                f"StableId:{record['stable_id']}",
                f"SemanticClass:{record['semantic_class']}",
            ],
            material,
            cast_shadow=_should_cast_visual_shadow(record),
        )
        actors.append(actor)
        component = actor.get_editor_property("static_mesh_component")
        assignments.append(
            {
                "stable_id": record["stable_id"],
                "mesh_name": mesh_name,
                "source_material": source_material,
                "source_semantic_class": semantic_class,
                "material_asset": str(material.get_path_name()),
                "mesh_slot_count": mesh_slot_count,
                "component_slot_count": _mesh_material_slot_count(mesh),
                "component": str(component.get_path_name()),
                "cast_shadow": bool(component.get_editor_property("cast_shadow")),
            }
        )
        transforms.append(transform_evidence)
    return actors, transforms, assignments


def _tag_actor(actor: object, label: str, *tags: str) -> None:
    if actor is None:
        raise RuntimeError(f"failed to spawn FactoryPark_HF actor {label}")
    actor.set_actor_label(label)
    actor.set_editor_property("tags", [unreal.Name(tag) for tag in (WORLD_TAG, *tags)])


def _editor_property_values_match(actual: object, expected: object) -> bool:
    if isinstance(expected, bool):
        return actual is expected
    if isinstance(expected, (float, int)) and not isinstance(expected, bool):
        return isinstance(actual, (float, int)) and not isinstance(actual, bool) and math.isclose(
            float(actual),
            float(expected),
            rel_tol=0.0,
            abs_tol=1.0e-4,
        )
    for components in (("r", "g", "b", "a"), ("x", "y", "z", "w"), ("pitch", "yaw", "roll")):
        if all(hasattr(expected, component) and hasattr(actual, component) for component in components):
            return all(
                _editor_property_values_match(
                    getattr(actual, component),
                    getattr(expected, component),
                )
                for component in components
            )
    return bool(actual == expected)


def _set_required_editor_property(target: object, property_name: str, value: object) -> object:
    target.set_editor_property(property_name, value)
    actual = target.get_editor_property(property_name)
    matches = _editor_property_values_match(actual, value)
    if not matches:
        raise RuntimeError(
            f"FactoryPark_HF property readback failed for {property_name}: "
            f"expected={value!r}, actual={actual!r}"
        )
    return actual


def _configure_manual_exposure(
    post_process: object,
    exposure_recipe: Mapping[str, object],
) -> dict[str, object]:
    if exposure_recipe.get("mode") != "manual":
        raise RuntimeError("FactoryPark_HF Unreal exposure mode must be manual")
    ev100 = _finite_vector([exposure_recipe.get("ev100")], 1, "lighting.exposure.ev100")[0]
    white_balance_k = _finite_vector(
        [exposure_recipe.get("white_balance_k")],
        1,
        "lighting.exposure.white_balance_k",
    )[0]
    if not 0.0 <= ev100 <= 20.0:
        raise RuntimeError("FactoryPark_HF EV100 must lie in [0,20]")
    if not 1500.0 <= white_balance_k <= 15000.0:
        raise RuntimeError("FactoryPark_HF white balance must lie in [1500,15000]K")

    camera_iso = 100.0
    camera_fstop = 8.0
    camera_shutter_speed = (2.0**ev100) / (camera_fstop**2)
    if not 0.01 <= camera_shutter_speed <= 8000.0:
        raise RuntimeError("FactoryPark_HF physical camera shutter lies outside UE limits")

    settings = post_process.get_editor_property("settings")
    properties = (
        ("override_auto_exposure_method", True),
        ("auto_exposure_method", unreal.AutoExposureMethod.AEM_MANUAL),
        ("override_auto_exposure_apply_physical_camera_exposure", True),
        ("auto_exposure_apply_physical_camera_exposure", True),
        ("override_camera_iso", True),
        ("camera_iso", camera_iso),
        ("override_camera_shutter_speed", True),
        ("camera_shutter_speed", camera_shutter_speed),
        ("override_depth_of_field_fstop", True),
        ("depth_of_field_fstop", camera_fstop),
        ("override_auto_exposure_bias", True),
        ("auto_exposure_bias", 0.0),
        ("override_temperature_type", True),
        ("temperature_type", unreal.TemperatureMethod.TEMP_WHITE_BALANCE),
        ("override_white_temp", True),
        ("white_temp", white_balance_k),
        ("override_white_tint", True),
        ("white_tint", 0.0),
    )
    for property_name, value in properties:
        _set_required_editor_property(settings, property_name, value)
    post_process.set_editor_property("settings", settings)

    return {
        "mode": "manual_physical_camera",
        "ev100": ev100,
        "camera_iso": camera_iso,
        "camera_shutter_speed": camera_shutter_speed,
        "camera_fstop": camera_fstop,
        "exposure_compensation": 0.0,
        "white_balance_k": white_balance_k,
        "white_tint": 0.0,
        "verified": True,
    }


def _configure_lumen_world(post_process: object, realism: Mapping[str, object]) -> dict[str, object]:
    requested = realism["lighting"]["unreal"]  # type: ignore[index]
    exposure = _configure_manual_exposure(
        post_process,
        realism["lighting"]["exposure"],  # type: ignore[index]
    )
    commands = (
        "r.DynamicGlobalIlluminationMethod 1",
        "r.ReflectionMethod 1",
        "r.Shadow.Virtual.Enable 1",
    )
    applied: list[str] = []
    warnings: list[str] = []
    for command in commands:
        try:
            unreal.SystemLibrary.execute_console_command(post_process, command)
            applied.append(command)
        except Exception as error:
            warnings.append(f"{command}: {error}")
            unreal.log_warning(f"FactoryPark_HF could not apply {command}: {error}")
    project_config = PROJECT_DIR / "Config" / "DefaultEngine.ini"
    config_text = project_config.read_text(encoding="utf-8") if project_config.is_file() else ""
    return {
        "requested": requested,
        "runtime_console_commands_applied": applied,
        "runtime_console_command_warnings": warnings,
        "project_config": str(project_config),
        "project_config_lumen_gi": "r.DynamicGlobalIlluminationMethod=1" in config_text,
        "project_config_lumen_reflections": "r.ReflectionMethod=1" in config_text,
        "dynamic_light_requirement": "movable",
        "exposure": exposure,
    }


def _spawn_tank_bund_fill_light(actor_subsystem: object) -> tuple[object, dict[str, object]]:
    location_cm = source_to_unreal_location_cm(TANK_BUND_FILL_LIGHT["position_m"])
    look_at_cm = source_to_unreal_location_cm(TANK_BUND_FILL_LIGHT["look_at_m"])
    location = unreal.Vector(*location_cm)
    look_at = unreal.Vector(*look_at_cm)
    rotation = unreal.MathLibrary.find_look_at_rotation(location, look_at)
    actor = actor_subsystem.spawn_actor_from_class(
        unreal.RectLight,
        location,
        rotation,
        False,
    )
    label = str(TANK_BUND_FILL_LIGHT["label"])
    _tag_actor(actor, label, "OutdoorLighting", "LightingRemediation")
    component = actor.light_component
    component.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
    component.set_editor_property("intensity", float(TANK_BUND_FILL_LIGHT["intensity"]))
    component.set_editor_property("attenuation_radius", float(TANK_BUND_FILL_LIGHT["attenuation_radius_cm"]))
    component.set_editor_property("source_width", float(TANK_BUND_FILL_LIGHT["source_width_cm"]))
    component.set_editor_property("source_height", float(TANK_BUND_FILL_LIGHT["source_height_cm"]))
    component.set_editor_property("cast_shadows", False)
    component.set_editor_property("light_color", unreal.Color(238, 245, 255, 255))
    _set_editor_property_if_supported(component, "use_temperature", True)
    _set_editor_property_if_supported(component, "temperature", 5600.0)
    evidence = {
        "label": label,
        "purpose": "lift_tank_bund_wall_vsm_grazing_shadow_floor",
        "location_cm": list(location_cm),
        "look_at_cm": list(look_at_cm),
        "intensity": float(component.get_editor_property("intensity")),
        "attenuation_radius_cm": float(component.get_editor_property("attenuation_radius")),
        "source_width_cm": float(component.get_editor_property("source_width")),
        "source_height_cm": float(component.get_editor_property("source_height")),
        "cast_shadows": bool(component.get_editor_property("cast_shadows")),
        "environment_actor": False,
        "stable_id": None,
    }
    if (
        not math.isclose(evidence["intensity"], float(TANK_BUND_FILL_LIGHT["intensity"]), abs_tol=0.01)
        or evidence["cast_shadows"] is not False
    ):
        raise RuntimeError(f"tank-bund fill-light readback failed: {evidence}")
    return actor, evidence


def _configure_sun_component(
    component: object,
    sun_recipe: Mapping[str, object],
) -> dict[str, object]:
    illuminance_lux = _finite_vector(
        [sun_recipe.get("illuminance_lux")], 1, "lighting.sun.illuminance_lux"
    )[0]
    color_temperature_k = _finite_vector(
        [sun_recipe.get("color_temperature_k")],
        1,
        "lighting.sun.color_temperature_k",
    )[0]
    angular_diameter_deg = _finite_vector(
        [sun_recipe.get("angular_diameter_deg")],
        1,
        "lighting.sun.angular_diameter_deg",
    )[0]
    if illuminance_lux <= 0.0:
        raise RuntimeError("FactoryPark_HF sun illuminance must be positive")
    if not 1700.0 <= color_temperature_k <= 12000.0:
        raise RuntimeError("FactoryPark_HF sun temperature lies outside UE limits")
    if not 0.0 < angular_diameter_deg <= 5.0:
        raise RuntimeError("FactoryPark_HF sun angular diameter lies outside UE limits")

    neutral_color = unreal.Color(255, 255, 255, 255)
    properties = (
        ("intensity", illuminance_lux),
        ("light_color", neutral_color),
        ("atmosphere_sun_light", True),
        ("use_temperature", True),
        ("temperature", color_temperature_k),
        ("light_source_angle", angular_diameter_deg),
    )
    for property_name, value in properties:
        _set_required_editor_property(component, property_name, value)

    return {
        "illuminance_lux": illuminance_lux,
        "base_light_color_rgba": [255, 255, 255, 255],
        "color_temperature_k": color_temperature_k,
        "angular_diameter_deg": angular_diameter_deg,
        "verified": True,
    }


def _find_unique_actor_by_label(actor_subsystem: object, label: str) -> object:
    matches = [
        actor
        for actor in actor_subsystem.get_all_level_actors()
        if str(actor.get_actor_label()) == label
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"FactoryPark_HF rendering refresh requires exactly one actor labelled {label}, "
            f"found {len(matches)}"
        )
    return matches[0]


def _refresh_existing_rendering_contract(
    actor_subsystem: object,
    realism: Mapping[str, object],
    master_material: object,
) -> dict[str, object]:
    lighting = realism.get("lighting")
    if not isinstance(lighting, Mapping) or not isinstance(lighting.get("sun"), Mapping):
        raise RuntimeError("FactoryPark_HF rendering refresh requires a sun lighting recipe")
    sun_actor = _find_unique_actor_by_label(actor_subsystem, SUN_LABEL)
    post_process = _find_unique_actor_by_label(actor_subsystem, POST_PROCESS_LABEL)
    sun_component = getattr(sun_actor, "light_component", None)
    if sun_component is None:
        raise RuntimeError("FactoryPark_HF rendering refresh sun has no light component")

    material_usages = _configure_master_material_usages(master_material)
    sun_evidence = _configure_sun_component(sun_component, lighting["sun"])
    lumen_evidence = _configure_lumen_world(post_process, realism)
    exposure_evidence = lumen_evidence.get("exposure")
    if not (
        all(material_usages.values())
        and sun_evidence.get("verified") is True
        and isinstance(exposure_evidence, Mapping)
        and exposure_evidence.get("verified") is True
    ):
        raise RuntimeError("FactoryPark_HF refreshed rendering contract did not verify")
    return {
        "material_usages": dict(material_usages),
        "sun": dict(sun_evidence),
        "lumen": dict(lumen_evidence),
        "verified": True,
    }


def _spawn_outdoor_lighting(
    actor_subsystem: object,
    realism: Mapping[str, object],
) -> dict[str, object]:
    lighting_recipe = realism["lighting"]
    sun_recipe = lighting_recipe["sun"]  # type: ignore[index]
    sun = actor_subsystem.spawn_actor_from_class(
        unreal.DirectionalLight,
        unreal.Vector(0.0, 0.0, 5000.0),
        unreal.Rotator(-38.0, -28.0, -8.0),
        False,
    )
    _tag_actor(sun, SUN_LABEL, "OutdoorLighting")
    sun.light_component.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
    sun_evidence = _configure_sun_component(sun.light_component, sun_recipe)

    skylight = actor_subsystem.spawn_actor_from_class(
        unreal.SkyLight,
        unreal.Vector(0.0, 0.0, 2500.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    _tag_actor(skylight, SKYLIGHT_LABEL, "OutdoorLighting")
    skylight.light_component.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
    skylight.light_component.set_editor_property("intensity", 1.15)
    skylight.light_component.set_editor_property("real_time_capture", True)

    atmosphere = actor_subsystem.spawn_actor_from_class(
        unreal.SkyAtmosphere,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    _tag_actor(atmosphere, ATMOSPHERE_LABEL, "OutdoorLighting")

    fog = actor_subsystem.spawn_actor_from_class(
        unreal.ExponentialHeightFog,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    _tag_actor(fog, FOG_LABEL, "OutdoorLighting")
    fog_component = fog.get_editor_property("component")
    _set_editor_property_if_supported(fog_component, "fog_density", 0.0018)
    _set_editor_property_if_supported(fog_component, "fog_height_falloff", 0.14)
    _set_editor_property_if_supported(fog_component, "volumetric_fog", True)

    fill_location = unreal.Vector(0.0, 0.0, 2800.0)
    fill_rotation = unreal.MathLibrary.find_look_at_rotation(fill_location, unreal.Vector(0.0, 0.0, 0.0))
    fill = actor_subsystem.spawn_actor_from_class(unreal.RectLight, fill_location, fill_rotation, False)
    _tag_actor(fill, FILL_LIGHT_LABEL, "OutdoorLighting")
    fill.light_component.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
    fill.light_component.set_editor_property("intensity", 80.0)
    fill.light_component.set_editor_property("light_color", unreal.Color(200, 220, 255, 255))
    _set_editor_property_if_supported(fill.light_component, "source_width", 8000.0)
    _set_editor_property_if_supported(fill.light_component, "source_height", 6000.0)
    tank_fill, lighting_remediation = _spawn_tank_bund_fill_light(actor_subsystem)
    post_process = actor_subsystem.spawn_actor_from_class(
        unreal.PostProcessVolume,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    _tag_actor(post_process, POST_PROCESS_LABEL, "LumenWorldSettings")
    _set_editor_property_if_supported(post_process, "unbound", True)
    _set_editor_property_if_supported(post_process, "priority", 10.0)
    lumen = _configure_lumen_world(post_process, realism)
    return {
        "actors": [
            SUN_LABEL,
            SKYLIGHT_LABEL,
            ATMOSPHERE_LABEL,
            FOG_LABEL,
            FILL_LIGHT_LABEL,
            tank_fill.get_actor_label(),
            POST_PROCESS_LABEL,
        ],
        "movable_lights": [SUN_LABEL, SKYLIGHT_LABEL, FILL_LIGHT_LABEL, tank_fill.get_actor_label()],
        "lighting_remediation": lighting_remediation,
        "sun": sun_evidence,
        "skylight_realtime_capture": True,
        "lumen": lumen,
    }


def _lens_to_horizontal_fov_degrees(lens_mm: float, sensor_width_mm: float = 36.0) -> float:
    return math.degrees(2.0 * math.atan(sensor_width_mm / (2.0 * lens_mm)))


def _resolve_acceptance_camera_frame(target: Mapping[str, object]) -> dict[str, object]:
    target_id = str(target["id"])
    requested_position = list(
        _finite_vector(target["position_m"], 3, f"{target_id}.position_m")
    )
    requested_look_at = list(
        _finite_vector(target["look_at_m"], 3, f"{target_id}.look_at_m")
    )
    if target_id == "tank_farm_inspection":
        actual_position = list(TANK_CAMERA_REMEDIATION["position_m"])
        actual_look_at = list(TANK_CAMERA_REMEDIATION["look_at_m"])
        adjustment = str(TANK_CAMERA_REMEDIATION["adjustment"])
    else:
        actual_position = requested_position
        actual_look_at = requested_look_at
        adjustment = "none"
    return {
        "requested_position_m": requested_position,
        "requested_look_at_m": requested_look_at,
        "actual_position_m": actual_position,
        "actual_look_at_m": actual_look_at,
        "adjustment": adjustment,
    }


def _spawn_acceptance_cameras(
    actor_subsystem: object,
    preview_targets: Sequence[Mapping[str, object]],
) -> tuple[dict[str, object], list[dict[str, object]]]:
    cameras: dict[str, object] = {}
    evidence: list[dict[str, object]] = []
    viewport = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
    for index, target in enumerate(preview_targets):
        target_id = str(target["id"])
        frame = _resolve_acceptance_camera_frame(target)
        location_cm = source_to_unreal_location_cm(frame["actual_position_m"])
        look_at_cm = source_to_unreal_location_cm(frame["actual_look_at_m"])
        location = unreal.Vector(*location_cm)
        look_at = unreal.Vector(*look_at_cm)
        rotation = unreal.MathLibrary.find_look_at_rotation(location, look_at)
        camera = actor_subsystem.spawn_actor_from_class(unreal.CameraActor, location, rotation, False)
        label = f"FactoryPark_HF_Camera_{target_id}"
        _tag_actor(camera, label, "AcceptanceCamera", f"PreviewTarget:{target_id}")
        lens_mm = float(target["lens_mm"])
        fov = _lens_to_horizontal_fov_degrees(lens_mm)
        resolution = _finite_vector(target["resolution_px"], 2, f"{target_id}.resolution_px")
        camera.camera_component.set_editor_property("field_of_view", fov)
        camera.camera_component.set_editor_property("aspect_ratio", resolution[0] / resolution[1])
        if index == 0:
            camera.set_editor_property("auto_activate_for_player", unreal.AutoReceiveInput.PLAYER0)
            viewport.set_level_viewport_camera_info(location, rotation)
        cameras[target_id] = camera
        evidence.append(
            {
                "id": target_id,
                "label": label,
                "purpose": target["purpose"],
                "location_cm": list(location_cm),
                "look_at_cm": list(look_at_cm),
                "lens_mm": lens_mm,
                "field_of_view_degrees": fov,
                "resolution_px": [int(resolution[0]), int(resolution[1])],
                "output_basename": target["output_basename"],
                **frame,
                "qa_adjusted": target["qa_adjusted"],
                "composition_adjustment": target["composition_adjustment"],
                "blender_preview_sha256": target["blender_preview_sha256"],
            }
        )
    return cameras, evidence


def _visual_remediation_evidence(
    acceptance_cameras: Sequence[Mapping[str, object]],
    lighting_remediation: Mapping[str, object],
    visual_shadow_policy: Mapping[str, object],
) -> dict[str, object]:
    tank_camera = next(
        record
        for record in acceptance_cameras
        if record["id"] == "tank_farm_inspection"
    )
    return {
        "profile": "tank_east_drainage_camera_v2",
        "reason": tank_camera["adjustment"],
        "environment_actors_modified": True,
        "environment_actor_modification_scope": "48 VisualOnly drainage-reed cast_shadow flags",
        "environment_component_flags_modified": visual_shadow_policy["actor_count"],
        "environment_actor_set_changed": False,
        "environment_transforms_modified": False,
        "environment_meshes_modified": False,
        "environment_materials_modified": False,
        "environment_collision_modified": False,
        "physics_authority_modified": False,
        "authoring_camera_position_m": tank_camera["requested_position_m"],
        "authoring_camera_look_at_m": tank_camera["requested_look_at_m"],
        "actual_qa_camera_position_m": tank_camera["actual_position_m"],
        "actual_qa_camera_look_at_m": tank_camera["actual_look_at_m"],
        "actual_qa_camera_location_cm": tank_camera["location_cm"],
        "actual_qa_camera_look_at_cm": tank_camera["look_at_cm"],
        "lighting_remediation": dict(lighting_remediation),
        "visual_shadow_policy": {
            "profile": visual_shadow_policy["profile"],
            "semantic_class": visual_shadow_policy["semantic_class"],
            "actor_count": visual_shadow_policy["actor_count"],
            "expected_cast_shadow": visual_shadow_policy["expected_cast_shadow"],
            "persisted_after_map_save_reload": visual_shadow_policy[
                "persisted_after_map_save_reload"
            ],
        },
    }


def _unsupported_screenshot_record(record: Mapping[str, object], reason: object) -> dict[str, object]:
    return {
        **record,
        "status": "unsupported",
        "unsupported": True,
        "reason": str(reason),
        "bytes": 0,
        "sha256": None,
    }


def _captured_screenshot_record(record: Mapping[str, object], output: Path) -> dict[str, object]:
    return {
        **record,
        "status": "captured",
        "unsupported": False,
        "bytes": output.stat().st_size,
        "sha256": _sha256_file(output),
    }


def _validate_acceptance_screenshot_result_records(
    records: Sequence[Mapping[str, object]],
) -> None:
    target_ids = [str(record.get("target_id", "")) for record in records]
    valid = (
        len(records) == len(REQUIRED_ACCEPTANCE_CAMERA_IDS)
        and set(target_ids) == REQUIRED_ACCEPTANCE_CAMERA_IDS
        and len(set(target_ids)) == len(target_ids)
        and all(
            record.get("status") == "captured"
            and record.get("unsupported") is False
            and isinstance(record.get("bytes"), int)
            and not isinstance(record.get("bytes"), bool)
            and int(record["bytes"]) > 0
            and isinstance(record.get("sha256"), str)
            and _SHA256_PATTERN.fullmatch(str(record["sha256"])) is not None
            for record in records
        )
    )
    if not valid:
        raise RuntimeError("FactoryPark_HF acceptance screenshots must be 3/3 captured")


def _finalize_acceptance_screenshot_results(
    payload: dict[str, object],
    records: Sequence[Mapping[str, object]],
    *,
    success_writer: Callable[[Mapping[str, object]], None],
    error_writer: Callable[
        [Exception, Mapping[str, object], Sequence[Mapping[str, object]]],
        None,
    ],
) -> bool:
    serializable_records = [dict(record) for record in records]
    payload["screenshots"] = serializable_records
    try:
        _validate_acceptance_screenshot_result_records(serializable_records)
    except RuntimeError as error:
        payload["result"] = "error"
        error_writer(error, payload, serializable_records)
        return False
    success_writer(payload)
    return True


def _write_success_evidence(payload: Mapping[str, object]) -> None:
    _write_json(SUCCESS_SENTINEL, payload)
    environment = payload["environment"]
    unreal.log(
        "LINGTU_FACTORY_PARK_HF_READY "
        f"map={MAP_PATH} environment_actors={environment['actor_count']} "  # type: ignore[index]
        f"terrain_actors={payload['terrain']['actor_count']} evidence={SUCCESS_SENTINEL}"  # type: ignore[index]
    )


def _write_error_evidence(
    error: Exception,
    partial_payload: Mapping[str, object] | None = None,
    screenshots: Sequence[Mapping[str, object]] = (),
) -> None:
    failure: dict[str, object] = {
        "schema": "lingtu.sim.unreal-factory-park-hf-error.v1",
        "result": "error",
        "error_type": type(error).__name__,
        "message": str(error),
        "traceback": "".join(traceback.format_exception(error)),
        "world_recipe": str(WORLD_RECIPE_PATH),
        "blender_manifest": str(BLENDER_MANIFEST_PATH),
    }
    if partial_payload is not None:
        failure["map_path"] = partial_payload.get("map_path")
        failure["partial_payload_result"] = partial_payload.get("result")
    if screenshots:
        failure["acceptance_screenshots"] = [dict(record) for record in screenshots]
    _write_json(ERROR_SENTINEL, failure)
    if unreal is not None:
        unreal.log_error(f"LINGTU_FACTORY_PARK_HF_ERROR={ERROR_SENTINEL}")


def _write_screenshot_error_evidence(
    error: Exception,
    payload: Mapping[str, object],
    screenshots: Sequence[Mapping[str, object]],
) -> None:
    _write_error_evidence(error, payload, screenshots)


def _schedule_acceptance_screenshots(
    payload: dict[str, object],
    cameras: Mapping[str, object],
    camera_evidence: Sequence[Mapping[str, object]],
) -> bool:
    global _SCREENSHOT_CAPTURE_PENDING, _SCREENSHOT_TICK_HANDLE

    screenshot_root = (EVIDENCE_ROOT / "screenshots").resolve()
    screenshot_root.mkdir(parents=True, exist_ok=True)
    evidence_by_id = {str(record["id"]): record for record in camera_evidence}
    queue: list[dict[str, object]] = []
    for target_id in sorted(REQUIRED_ACCEPTANCE_CAMERA_IDS):
        camera = cameras[target_id]
        target = evidence_by_id[target_id]
        output = (screenshot_root / str(target["output_basename"])).resolve()
        try:
            output.relative_to(screenshot_root)
        except ValueError as error:
            raise RuntimeError(f"screenshot target escapes UE evidence root: {output}") from error
        if output.is_file():
            output.unlink()
        queue.append(
            {
                "target_id": target_id,
                "camera": camera,
                "camera_label": camera.get_actor_label(),
                "path": str(output),
                "resolution_px": list(target["resolution_px"]),
                "api": "AutomationLibrary.take_high_res_screenshot",
            }
        )

    register_tick = getattr(unreal, "register_slate_post_tick_callback", None)
    unregister_tick = getattr(unreal, "unregister_slate_post_tick_callback", None)
    screenshot_api = getattr(getattr(unreal, "AutomationLibrary", None), "take_high_res_screenshot", None)
    if not callable(register_tick) or not callable(unregister_tick) or not callable(screenshot_api):
        reason = "UE5.8 asynchronous screenshot/tick API is unavailable"
        results = [
            _unsupported_screenshot_record(
                {key: value for key, value in record.items() if key != "camera"},
                reason,
            )
            for record in queue
        ]
        _finalize_acceptance_screenshot_results(
            payload,
            results,
            success_writer=_write_success_evidence,
            error_writer=_write_screenshot_error_evidence,
        )
        return False

    try:
        unreal.AutomationLibrary.finish_loading_before_screenshot()
        unreal.AutomationLibrary.set_scalability_quality_to_epic(next(iter(cameras.values())))
    except Exception as error:
        unreal.log_warning(f"FactoryPark_HF screenshot warm-up warning: {error}")

    state: dict[str, object] = {
        "queue": queue,
        "results": [],
        "current": None,
        "task": None,
        "started_at": None,
        "done_at": None,
    }

    def finish() -> None:
        global _SCREENSHOT_CAPTURE_PENDING, _SCREENSHOT_TICK_HANDLE

        if _SCREENSHOT_TICK_HANDLE is not None:
            unregister_tick(_SCREENSHOT_TICK_HANDLE)
        _SCREENSHOT_TICK_HANDLE = None
        _SCREENSHOT_CAPTURE_PENDING = False
        _finalize_acceptance_screenshot_results(
            payload,
            state["results"],  # type: ignore[arg-type]
            success_writer=_write_success_evidence,
            error_writer=_write_screenshot_error_evidence,
        )
        _quit_editor_if_unattended()

    def start_next() -> None:
        queue_value = state["queue"]
        if not queue_value:
            finish()
            return
        current = queue_value.pop(0)
        state["current"] = current
        state["started_at"] = time.monotonic()
        state["done_at"] = None
        output = Path(str(current["path"]))
        resolution = current["resolution_px"]
        try:
            task = unreal.AutomationLibrary.take_high_res_screenshot(
                int(resolution[0]),
                int(resolution[1]),
                str(output),
                current["camera"],
                False,
                False,
                unreal.ComparisonTolerance.LOW,
                f"FactoryPark_HF {current['target_id']} {REALISM_PROFILE}",
                0.35,
                True,
            )
            if task is None or not task.is_valid_task():
                raise RuntimeError("AutomationEditorTask is invalid")
            state["task"] = task
            unreal.log(f"LINGTU_FACTORY_PARK_HF_SCREENSHOT_REQUEST target={current['target_id']}")
        except Exception as error:
            serializable = {key: value for key, value in current.items() if key != "camera"}
            state["results"].append(_unsupported_screenshot_record(serializable, error))
            unreal.log_warning(f"FactoryPark_HF screenshot unavailable for {current['target_id']}: {error}")
            state["current"] = None
            state["task"] = None

    def on_post_tick(_delta_seconds: float) -> None:
        try:
            if state["current"] is None:
                start_next()
                return
            current = state["current"]
            task = state["task"]
            output = Path(str(current["path"]))
            now = time.monotonic()
            if task is not None and task.is_task_done() and state["done_at"] is None:
                state["done_at"] = now
            captured = output.is_file() and output.stat().st_size > 0
            task_done = state["done_at"] is not None
            settle_expired = task_done and now - float(state["done_at"]) >= 2.0
            timed_out = now - float(state["started_at"]) >= SCREENSHOT_TIMEOUT_SECONDS
            if not (captured and task_done) and not settle_expired and not timed_out:
                return
            serializable = {key: value for key, value in current.items() if key != "camera"}
            if captured:
                state["results"].append(_captured_screenshot_record(serializable, output))
                unreal.log(f"LINGTU_FACTORY_PARK_HF_SCREENSHOT_READY target={current['target_id']} path={output}")
            else:
                reason = (
                    "UE5.8 AutomationEditorTask completed without an image"
                    if settle_expired
                    else f"screenshot timed out after {SCREENSHOT_TIMEOUT_SECONDS:.0f}s"
                )
                state["results"].append(_unsupported_screenshot_record(serializable, reason))
                unreal.log_warning(f"FactoryPark_HF screenshot unavailable for {current['target_id']}: {reason}")
            state["current"] = None
            state["task"] = None
        except Exception as error:
            unreal.log_error(f"LINGTU_FACTORY_PARK_HF_SCREENSHOT_STATE_ERROR {error}")
            current = state.get("current")
            if isinstance(current, dict):
                serializable = {key: value for key, value in current.items() if key != "camera"}
                state["results"].append(_unsupported_screenshot_record(serializable, error))
                state["current"] = None
                state["task"] = None

    _SCREENSHOT_CAPTURE_PENDING = True
    _SCREENSHOT_TICK_HANDLE = register_tick(on_post_tick)
    return True


def _actor_tags(actor: object) -> set[str]:
    return {str(tag) for tag in actor.get_editor_property("tags")}


def _records_by_stable_id(
    records: Sequence[Mapping[str, object]],
    context: str,
) -> dict[str, Mapping[str, object]]:
    result: dict[str, Mapping[str, object]] = {}
    for record in records:
        stable_id = str(record.get("stable_id", ""))
        if not stable_id or stable_id in result:
            raise RuntimeError(f"FactoryPark_HF {context} repeats or omits a StableId")
        result[stable_id] = record
    return result


def _numeric_vectors_close(
    actual: object,
    expected: object,
    length: int,
    *,
    tolerance: float,
) -> bool:
    try:
        actual_values = _finite_vector(actual, length, "loaded-map actual vector")  # type: ignore[arg-type]
        expected_values = _finite_vector(expected, length, "loaded-map expected vector")  # type: ignore[arg-type]
    except RuntimeError:
        return False
    return all(
        math.isclose(actual_value, expected_value, rel_tol=0.0, abs_tol=tolerance)
        for actual_value, expected_value in zip(actual_values, expected_values)
    )


def _quaternions_equivalent(actual: object, expected: object, tolerance: float = 1e-5) -> bool:
    try:
        actual_values = _finite_vector(actual, 4, "loaded-map actual quaternion")  # type: ignore[arg-type]
        expected_values = _finite_vector(expected, 4, "loaded-map expected quaternion")  # type: ignore[arg-type]
    except RuntimeError:
        return False
    direct = all(
        math.isclose(actual_value, expected_value, rel_tol=0.0, abs_tol=tolerance)
        for actual_value, expected_value in zip(actual_values, expected_values)
    )
    negated = all(
        math.isclose(actual_value, -expected_value, rel_tol=0.0, abs_tol=tolerance)
        for actual_value, expected_value in zip(actual_values, expected_values)
    )
    return direct or negated


def _validate_loaded_map_observations(
    authoring: Mapping[str, object],
    expected_transforms: Sequence[Mapping[str, object]],
    expected_assignments: Sequence[Mapping[str, object]],
    observations: Sequence[Mapping[str, object]],
    *,
    terrain_actor_count: int,
    preplaced_robot_bindings: int,
    robot_actor_count: int,
    robot_mesh_references: int,
) -> dict[str, object]:
    expected_by_stable_id = _records_by_stable_id(
        authoring["placed_objects"],  # type: ignore[arg-type,index]
        "authoring placements",
    )
    transform_by_stable_id = _records_by_stable_id(expected_transforms, "transform evidence")
    assignment_by_stable_id = _records_by_stable_id(
        expected_assignments,
        "material assignment evidence",
    )
    observed_by_stable_id = _records_by_stable_id(observations, "loaded-map observations")
    expected_ids = set(expected_by_stable_id)
    if (
        set(transform_by_stable_id) != expected_ids
        or set(assignment_by_stable_id) != expected_ids
        or set(observed_by_stable_id) != expected_ids
    ):
        raise RuntimeError("FactoryPark_HF loaded-map StableId set is not exact")
    if (
        terrain_actor_count != 1
        or preplaced_robot_bindings != 0
        or robot_actor_count != 0
        or robot_mesh_references != 0
    ):
        raise RuntimeError("FactoryPark_HF loaded-map authority counts are invalid")

    verified_records: list[dict[str, object]] = []
    material_slot_readback_count = 0
    for stable_id in sorted(expected_ids):
        authored = expected_by_stable_id[stable_id]
        expected_transform = transform_by_stable_id[stable_id]
        assignment = assignment_by_stable_id[stable_id]
        observed = observed_by_stable_id[stable_id]
        expected_location_cm = source_to_unreal_location_cm(authored["position_m"])  # type: ignore[arg-type,index]
        expected_quaternion_xyzw = source_to_unreal_quaternion_xyzw(
            authored["quaternion_wxyz"]  # type: ignore[arg-type,index]
        )
        expected_scale = source_scale_to_unreal_actor_scale(
            authored["scale"]  # type: ignore[arg-type,index]
        )
        transform_matches = (
            _numeric_vectors_close(
                expected_transform.get("source_position_m"),
                authored.get("position_m"),
                3,
                tolerance=1e-9,
            )
            and _numeric_vectors_close(
                expected_transform.get("source_quaternion_wxyz"),
                authored.get("quaternion_wxyz"),
                4,
                tolerance=1e-9,
            )
            and _numeric_vectors_close(
                expected_transform.get("source_scale"),
                authored.get("scale"),
                3,
                tolerance=1e-9,
            )
            and _numeric_vectors_close(
                expected_transform.get("location_cm"),
                expected_location_cm,
                3,
                tolerance=0.01,
            )
            and _quaternions_equivalent(
                expected_transform.get("quaternion_xyzw"),
                expected_quaternion_xyzw,
            )
            and _numeric_vectors_close(
                expected_transform.get("scale"),
                expected_scale,
                3,
                tolerance=0.001,
            )
            and _numeric_vectors_close(
                observed.get("location_cm"),
                expected_transform.get("location_cm"),
                3,
                tolerance=0.05,
            )
            and _quaternions_equivalent(
                observed.get("quaternion_xyzw"),
                expected_transform.get("quaternion_xyzw"),
            )
            and _numeric_vectors_close(
                observed.get("scale"),
                expected_transform.get("scale"),
                3,
                tolerance=0.001,
            )
        )
        if not transform_matches:
            raise RuntimeError(f"FactoryPark_HF loaded-map transform drift for {stable_id}")
        if (
            observed.get("semantic_class") != authored.get("semantic_class")
            or observed.get("visual_only") is not authored.get("visual_only")
            or observed.get("mesh_name") != authored.get("mesh_name")
        ):
            raise RuntimeError(f"FactoryPark_HF loaded-map identity drift for {stable_id}")

        expected_material_path = assignment.get("material_asset")
        expected_slot_count = assignment.get("component_slot_count")
        actual_material_paths = observed.get("material_paths")
        if (
            not isinstance(expected_material_path, str)
            or not expected_material_path
            or isinstance(expected_slot_count, bool)
            or not isinstance(expected_slot_count, int)
            or expected_slot_count <= 0
            or not isinstance(actual_material_paths, list)
            or len(actual_material_paths) != expected_slot_count
            or any(path != expected_material_path for path in actual_material_paths)
        ):
            raise RuntimeError(f"FactoryPark_HF loaded-map material drift for {stable_id}")
        if (
            observed.get("collision_disabled") is not True
            or observed.get("collision_profile_name") != "NoCollision"
            or observed.get("generate_overlap_events") is not False
        ):
            diagnostic = {
                key: observed.get(key)
                for key in (
                    "collision_enabled",
                    "collision_enabled_repr",
                    "collision_expected_repr",
                    "collision_profile_name",
                    "generate_overlap_events",
                    "verified",
                )
            }
            raise RuntimeError(
                f"FactoryPark_HF loaded-map collision drift for {stable_id}: "
                f"{json.dumps(diagnostic, sort_keys=True)}"
            )
        material_slot_readback_count += expected_slot_count
        verified_records.append(dict(observed))

    return {
        "profile": "post_save_reload_runtime_v1",
        "map_path": MAP_PATH,
        "stable_id_set_exact": True,
        "expected_stable_id_count": len(expected_ids),
        "observed_stable_id_count": len(observed_by_stable_id),
        "stable_ids": sorted(expected_ids),
        "transform_readback_count": len(verified_records),
        "transforms_verified": True,
        "material_assignment_readback_count": len(verified_records),
        "material_slot_readback_count": material_slot_readback_count,
        "materials_verified": True,
        "collision_disabled_count": len(verified_records),
        "collision_all_disabled": True,
        "records": verified_records,
        "terrain_actor_count": terrain_actor_count,
        "preplaced_robot_bindings": preplaced_robot_bindings,
        "robot_actor_count": robot_actor_count,
        "robot_mesh_references": robot_mesh_references,
    }


def _collect_loaded_map_observations(
    actor_subsystem: object,
    expected_assignments: Sequence[Mapping[str, object]],
) -> list[dict[str, object]]:
    assignment_by_stable_id = _records_by_stable_id(
        expected_assignments,
        "material assignment evidence",
    )
    observations: list[dict[str, object]] = []
    for actor in actor_subsystem.get_all_level_actors():
        tags = _actor_tags(actor)
        stable_tags = sorted(tag for tag in tags if tag.startswith("StableId:"))
        if not stable_tags:
            continue
        if len(stable_tags) != 1:
            raise RuntimeError(
                f"FactoryPark_HF actor {actor.get_actor_label()} has ambiguous StableId tags"
            )
        stable_id = stable_tags[0].split(":", 1)[1]
        semantic_tags = sorted(tag for tag in tags if tag.startswith("SemanticClass:"))
        authority_tags = sorted(tags.intersection({VISUAL_ONLY_TAG, PHYSICS_SHARED_TAG}))
        if len(semantic_tags) != 1 or len(authority_tags) != 1:
            raise RuntimeError(f"FactoryPark_HF loaded actor {stable_id} has invalid authority tags")
        try:
            component = actor.get_editor_property("static_mesh_component")
            mesh = component.get_editor_property("static_mesh")
        except Exception as error:
            raise RuntimeError(
                f"FactoryPark_HF loaded actor {stable_id} has no StaticMeshComponent"
            ) from error
        if mesh is None:
            raise RuntimeError(f"FactoryPark_HF loaded actor {stable_id} has no StaticMesh")
        assignment = assignment_by_stable_id.get(stable_id)
        if assignment is None:
            slot_count_getter = getattr(component, "get_num_materials", None)
            slot_count = int(slot_count_getter()) if callable(slot_count_getter) else 0
        else:
            slot_count = int(assignment.get("component_slot_count", 0))
        material_paths: list[str] = []
        for slot_index in range(max(0, slot_count)):
            material = component.get_material(slot_index)
            material_paths.append("" if material is None else str(material.get_path_name()))
        location = actor.get_actor_location()
        rotation = actor.get_actor_rotation()
        quaternion = rotation.quaternion()
        scale = actor.get_actor_scale3d()
        collision_readback = _read_collision_component_policy(component)
        observations.append(
            {
                "stable_id": stable_id,
                "actor_label": actor.get_actor_label(),
                "component_path": str(component.get_path_name()),
                "semantic_class": semantic_tags[0].split(":", 1)[1],
                "visual_only": authority_tags[0] == VISUAL_ONLY_TAG,
                "mesh_name": str(mesh.get_name()),
                "mesh_asset": str(mesh.get_path_name()),
                "location_cm": [float(location.x), float(location.y), float(location.z)],
                "quaternion_xyzw": [
                    float(quaternion.x),
                    float(quaternion.y),
                    float(quaternion.z),
                    float(quaternion.w),
                ],
                "scale": [float(scale.x), float(scale.y), float(scale.z)],
                "material_paths": material_paths,
                **collision_readback,
            }
        )
    return observations


def _audit_loaded_factory_map(
    actor_subsystem: object,
    authoring: Mapping[str, object],
    expected_transforms: Sequence[Mapping[str, object]],
    expected_assignments: Sequence[Mapping[str, object]],
) -> dict[str, object]:
    observations = _collect_loaded_map_observations(actor_subsystem, expected_assignments)
    audit = _validate_loaded_map_observations(
        authoring,
        expected_transforms,
        expected_assignments,
        observations,
        terrain_actor_count=_count_terrain_actors(actor_subsystem),
        preplaced_robot_bindings=_count_preplaced_robot_bindings(actor_subsystem),
        robot_actor_count=_count_robot_actors(actor_subsystem),
        robot_mesh_references=_count_robot_mesh_references(actor_subsystem),
    )
    audit["map_saved_and_reloaded"] = True
    audit["observation_source"] = "loaded_unreal_world_runtime_readback"
    return audit


def _apply_collision_policy_existing_map(
    actor_subsystem: object,
    authoring: Mapping[str, object],
    *,
    apply_policy: bool,
    persisted_after_reload: bool,
) -> dict[str, object]:
    """Apply or strictly read back the persisted no-collision component policy."""

    expected_stable_ids = sorted(
        str(record["stable_id"])
        for record in authoring["placed_objects"]  # type: ignore[index]
    )
    actors_by_stable_id: dict[str, object] = {}
    for actor in actor_subsystem.get_all_level_actors():
        stable_tags = sorted(
            tag for tag in _actor_tags(actor) if tag.startswith("StableId:")
        )
        if not stable_tags:
            continue
        if len(stable_tags) != 1:
            raise RuntimeError(
                f"FactoryPark_HF actor {actor.get_actor_label()} has ambiguous StableId tags"
            )
        stable_id = stable_tags[0].split(":", 1)[1]
        if stable_id in actors_by_stable_id:
            raise RuntimeError(f"FactoryPark_HF map repeats StableId:{stable_id}")
        actors_by_stable_id[stable_id] = actor
    if set(actors_by_stable_id) != set(expected_stable_ids):
        missing = sorted(set(expected_stable_ids) - actors_by_stable_id.keys())
        extra = sorted(actors_by_stable_id.keys() - set(expected_stable_ids))
        raise RuntimeError(
            "FactoryPark_HF collision policy StableId set mismatch: "
            f"missing={missing}, extra={extra}"
        )

    readbacks: list[dict[str, object]] = []
    changed_from_enabled_count = 0
    changed_profile_count = 0
    changed_overlap_count = 0
    component_modify_true_count = 0
    component_modify_false_count = 0
    component_modify_none_count = 0
    for stable_id in expected_stable_ids:
        actor = actors_by_stable_id[stable_id]
        component = actor.get_editor_property("static_mesh_component")
        if apply_policy:
            write = _disable_collision(component)
            before = write["before"]
            after = write["after"]
            changed_from_enabled_count += int(write["collision_enabled_changed"] is True)
            changed_profile_count += int(write["collision_profile_changed"] is True)
            changed_overlap_count += int(
                write["generate_overlap_events_changed"] is True
            )
            component_modify_true_count += int(
                write["component_modify_returned"] is True
            )
            component_modify_false_count += int(
                write["component_modify_returned"] is False
            )
            component_modify_none_count += int(
                write["component_modify_returned"] is None
            )
        else:
            before = None
            after = _read_collision_component_policy(component)
            _require_collision_component_policy(after, stable_id)
        readbacks.append(
            {
                "stable_id": stable_id,
                "before": before,
                "after": after,
            }
        )

    actor_count = len(readbacks)
    return {
        "profile": "persisted_no_collision_v1",
        "expected_actor_count": len(expected_stable_ids),
        "actor_count": actor_count,
        "stable_ids": expected_stable_ids,
        "expected_collision_enabled": str(unreal.CollisionEnabled.NO_COLLISION),
        "expected_collision_profile_name": "NoCollision",
        "expected_generate_overlap_events": False,
        "component_modify_count": actor_count if apply_policy else 0,
        "component_modify_return_true_count": component_modify_true_count,
        "component_modify_return_false_count": component_modify_false_count,
        "component_modify_return_none_count": component_modify_none_count,
        "component_modify_return_semantics": "transaction_participation_not_persistence_proof",
        "collision_enabled_write_count": actor_count if apply_policy else 0,
        "collision_profile_write_count": actor_count if apply_policy else 0,
        "generate_overlap_events_write_count": actor_count if apply_policy else 0,
        "changed_from_enabled_count": changed_from_enabled_count,
        "changed_profile_count": changed_profile_count,
        "changed_overlap_count": changed_overlap_count,
        "readback_count": actor_count,
        "all_readbacks_verified": all(
            record["after"]["verified"] is True  # type: ignore[index]
            for record in readbacks
        ),
        "readbacks": readbacks,
        "persisted_after_map_save_reload": persisted_after_reload,
        "map_reload_api": "LevelEditorSubsystem.load_level" if persisted_after_reload else None,
        "environment_actor_set_changed": False,
        "transforms_modified": False,
        "meshes_modified": False,
        "materials_modified": False,
        "physics_authority_modified": False,
    }


def _set_or_read_bool_editor_property(
    target: object,
    property_name: str,
    expected: bool,
    *,
    apply_policy: bool,
    required: bool,
) -> dict[str, object]:
    try:
        before = bool(target.get_editor_property(property_name))
    except Exception as error:
        if required:
            raise RuntimeError(
                f"FactoryPark_HF required component property is unavailable: {property_name}"
            ) from error
        unreal.log_warning(
            f"FactoryPark_HF optional component property unavailable: {property_name} ({error})"
        )
        return {
            "supported": False,
            "expected": expected,
            "before": None,
            "actual": None,
            "verified": False,
            "reason": str(error),
        }
    if apply_policy:
        target.set_editor_property(property_name, expected)
    actual = bool(target.get_editor_property(property_name))
    if actual is not expected:
        raise RuntimeError(
            f"FactoryPark_HF component property readback failed: "
            f"{property_name}={actual}, expected={expected}"
        )
    return {
        "supported": True,
        "expected": expected,
        "before": before,
        "actual": actual,
        "verified": True,
        "reason": None,
    }


def _apply_visual_shadow_policy_existing_map(
    actor_subsystem: object,
    authoring: Mapping[str, object],
    *,
    apply_policy: bool,
    persisted_after_reload: bool,
) -> dict[str, object]:
    """Apply or read back the narrow render-only shadow policy by stable ID."""

    target_records = {
        str(record["stable_id"]): record
        for record in authoring["placed_objects"]  # type: ignore[index]
        if not _should_cast_visual_shadow(record)
    }
    expected_stable_ids = sorted(target_records)
    if len(expected_stable_ids) != EXPECTED_NON_SHADOW_CASTING_VISUAL_ACTOR_COUNT:
        raise RuntimeError(
            "FactoryPark_HF visual shadow policy expected "
            f"{EXPECTED_NON_SHADOW_CASTING_VISUAL_ACTOR_COUNT} drainage reeds, "
            f"found {len(expected_stable_ids)}"
        )
    if any(
        record.get("visual_only") is not True
        or record.get("semantic_class") != "drainage_reed"
        for record in target_records.values()
    ):
        raise RuntimeError("FactoryPark_HF visual shadow policy escaped VisualOnly drainage reeds")

    actors_by_stable_id: dict[str, object] = {}
    for actor in actor_subsystem.get_all_level_actors():
        tags = _actor_tags(actor)
        stable_tags = sorted(tag for tag in tags if tag.startswith("StableId:"))
        if not stable_tags:
            continue
        if len(stable_tags) != 1:
            raise RuntimeError(
                f"FactoryPark_HF actor {actor.get_actor_label()} has ambiguous StableId tags"
            )
        stable_id = stable_tags[0].split(":", 1)[1]
        if stable_id in actors_by_stable_id:
            raise RuntimeError(f"FactoryPark_HF map repeats StableId:{stable_id}")
        actors_by_stable_id[stable_id] = actor

    missing = sorted(set(expected_stable_ids) - actors_by_stable_id.keys())
    if missing:
        raise RuntimeError(
            "FactoryPark_HF visual shadow policy cannot find drainage reeds: "
            + ", ".join(missing)
        )

    readbacks: list[dict[str, object]] = []
    property_changed_from_true_counts = {
        "cast_shadow": 0,
        "affect_distance_field_lighting": 0,
        "visible_in_ray_tracing": 0,
    }
    property_supported_counts = {property_name: 0 for property_name in property_changed_from_true_counts}
    property_verified_counts = {property_name: 0 for property_name in property_changed_from_true_counts}
    for stable_id in expected_stable_ids:
        actor = actors_by_stable_id[stable_id]
        tags = _actor_tags(actor)
        if VISUAL_ONLY_TAG not in tags or "SemanticClass:drainage_reed" not in tags:
            raise RuntimeError(
                f"FactoryPark_HF shadow target {stable_id} lost its VisualOnly drainage-reed tags"
            )
        component = actor.get_editor_property("static_mesh_component")
        visible_before = bool(component.get_editor_property("visible"))
        component_policy = _visual_shadow_component_policy(target_records[stable_id])
        property_readbacks = {
            property_name: _set_or_read_bool_editor_property(
                component,
                property_name,
                expected,
                apply_policy=apply_policy,
                required=property_name != "visible_in_ray_tracing",
            )
            for property_name, expected in component_policy.items()
        }
        visible_after = bool(component.get_editor_property("visible"))
        if not visible_before or not visible_after:
            raise RuntimeError(
                f"FactoryPark_HF shadow readback failed for {stable_id}: "
                f"visible_before={visible_before}, visible_after={visible_after}"
            )
        for property_name, property_readback in property_readbacks.items():
            supported = property_readback["supported"] is True
            verified = property_readback["verified"] is True
            property_supported_counts[property_name] += int(supported)
            property_verified_counts[property_name] += int(verified)
            property_changed_from_true_counts[property_name] += int(
                apply_policy and supported and property_readback["before"] is True
            )
        readbacks.append(
            {
                "stable_id": stable_id,
                "expected_cast_shadow": property_readbacks["cast_shadow"]["expected"],
                "actual_cast_shadow": property_readbacks["cast_shadow"]["actual"],
                "cast_shadow_supported": property_readbacks["cast_shadow"]["supported"],
                "expected_affect_distance_field_lighting": property_readbacks[
                    "affect_distance_field_lighting"
                ]["expected"],
                "actual_affect_distance_field_lighting": property_readbacks[
                    "affect_distance_field_lighting"
                ]["actual"],
                "affect_distance_field_lighting_supported": property_readbacks[
                    "affect_distance_field_lighting"
                ]["supported"],
                "expected_visible_in_ray_tracing": property_readbacks[
                    "visible_in_ray_tracing"
                ]["expected"],
                "actual_visible_in_ray_tracing": property_readbacks[
                    "visible_in_ray_tracing"
                ]["actual"],
                "visible_in_ray_tracing_supported": property_readbacks[
                    "visible_in_ray_tracing"
                ]["supported"],
                "visible_before": visible_before,
                "visible_after": visible_after,
                "verified": all(
                    record["verified"] is True or record["supported"] is False
                    for record in property_readbacks.values()
                ),
            }
        )

    return {
        "profile": "visual_only_micro_dressing_v1",
        "semantic_class": "drainage_reed",
        "component_property": "StaticMeshComponent render-only lighting flags",
        "component_properties": list(property_changed_from_true_counts),
        "expected_actor_count": EXPECTED_NON_SHADOW_CASTING_VISUAL_ACTOR_COUNT,
        "actor_count": len(readbacks),
        "stable_ids": expected_stable_ids,
        "expected_cast_shadow": False,
        "expected_affect_distance_field_lighting": False,
        "expected_visible_in_ray_tracing": False,
        "property_supported_actor_counts": property_supported_counts,
        "property_verified_actor_counts": property_verified_counts,
        "readbacks": readbacks,
        "all_readbacks_verified": all(record["verified"] for record in readbacks),
        "visible_geometry_preserved": all(record["visible_after"] for record in readbacks),
        "component_flag_write_count": len(readbacks) if apply_policy else 0,
        "property_write_count": (
            sum(property_supported_counts.values()) if apply_policy else 0
        ),
        "changed_from_true_count": property_changed_from_true_counts["cast_shadow"],
        "property_changed_from_true_counts": property_changed_from_true_counts,
        "persisted_after_map_save_reload": persisted_after_reload,
        "map_reload_api": "LevelEditorSubsystem.load_level" if persisted_after_reload else None,
        "environment_actor_set_changed": False,
        "transforms_modified": False,
        "meshes_modified": False,
        "materials_modified": False,
        "collision_modified": False,
        "physics_authority_modified": False,
    }


def _find_acceptance_camera_actors(
    actor_subsystem: object,
    expected_target_ids: Sequence[str],
) -> dict[str, object]:
    expected = set(expected_target_ids)
    cameras: dict[str, object] = {}
    for actor in actor_subsystem.get_all_level_actors():
        tags = _actor_tags(actor)
        if "AcceptanceCamera" not in tags:
            continue
        target_tags = sorted(tag for tag in tags if tag.startswith("PreviewTarget:"))
        if len(target_tags) != 1:
            raise RuntimeError(
                f"FactoryPark_HF camera {actor.get_actor_label()} has ambiguous preview-target tags"
            )
        target_id = target_tags[0].split(":", 1)[1]
        if target_id in cameras:
            raise RuntimeError(f"FactoryPark_HF map repeats acceptance camera {target_id}")
        cameras[target_id] = actor
    if set(cameras) != expected:
        raise RuntimeError(
            "FactoryPark_HF acceptance cameras do not survive map reload: "
            f"expected={sorted(expected)}, actual={sorted(cameras)}"
        )
    return cameras


def _count_preplaced_robot_bindings(actor_subsystem: object) -> int:
    count = 0
    for actor in actor_subsystem.get_all_level_actors():
        class_name = str(actor.get_class().get_name())
        if _actor_tags(actor).intersection(FORBIDDEN_BINDING_TAGS) or "LingTuSimBodyActor" in class_name:
            count += 1
    return count


def _count_robot_mesh_references(actor_subsystem: object) -> int:
    count = 0
    for actor in actor_subsystem.get_all_level_actors():
        try:
            component = actor.get_editor_property("static_mesh_component")
            mesh = component.get_editor_property("static_mesh")
        except Exception:
            mesh = None
        if mesh is not None and "/Robots/" in str(mesh.get_path_name()):
            count += 1
    return count


def _count_robot_actors(actor_subsystem: object) -> int:
    count = 0
    for actor in actor_subsystem.get_all_level_actors():
        tags = _actor_tags(actor)
        class_name = str(actor.get_class().get_name())
        robot_actor = bool(tags.intersection(FORBIDDEN_BINDING_TAGS)) or (
            "LingTuSimBodyActor" in class_name
        )
        try:
            component = actor.get_editor_property("static_mesh_component")
            mesh = component.get_editor_property("static_mesh")
            robot_actor = robot_actor or (
                mesh is not None and "/Robots/" in str(mesh.get_path_name())
            )
        except Exception:
            mesh = None
        count += int(robot_actor)
    return count


def _count_terrain_actors(actor_subsystem: object) -> int:
    return sum(TERRAIN_TAG in _actor_tags(actor) for actor in actor_subsystem.get_all_level_actors())


def _validate_refresh_source_evidence(
    payload: Mapping[str, object],
    world: Mapping[str, object],
    realism: Mapping[str, object],
    authoring: Mapping[str, object],
) -> None:
    expected_environment_count = len(authoring["placed_objects"])  # type: ignore[arg-type]
    native_pbr = payload.get("native_pbr_materials")
    environment = payload.get("environment")
    blender = payload.get("blender_authoring")
    screenshots = payload.get("screenshots")
    if not isinstance(screenshots, list):
        raise RuntimeError("previous FactoryPark_HF success evidence has no screenshot list")
    _validate_acceptance_screenshot_result_records(screenshots)
    if (
        payload.get("schema") != "lingtu.sim.unreal-factory-park-hf-evidence.v1"
        or payload.get("result") != "success"
        or payload.get("map_path") != MAP_PATH
        or payload.get("world_package") != WORLD_PACKAGE
        or payload.get("preplaced_robot_bindings") != 0
        or payload.get("robot_actor_count") != 0
        or payload.get("robot_mesh_references") != 0
        or not isinstance(environment, dict)
        or environment.get("actor_count") != expected_environment_count
        or environment.get("collision") != "disabled_in_unreal"
        or not isinstance(blender, dict)
        or blender.get("artifact_set_digest") != authoring["artifact_set_digest"]
        or not isinstance(native_pbr, dict)
        or native_pbr.get("instance_count") != _expected_native_pbr_instance_count(authoring)
        or native_pbr.get("environment_mesh_assignment_count") != expected_environment_count
        or native_pbr.get("environment_component_assignment_count") != expected_environment_count
        or native_pbr.get("base_color_policy")
        != "manifest_material_swatch_then_neutral_identity_fallback"
        or not isinstance(native_pbr.get("master_graph"), dict)
        or native_pbr["master_graph"].get("roughness_path") != "clamped_parameter"  # type: ignore[index]
        or native_pbr["master_graph"].get("world_position_noise_used") is not False  # type: ignore[index]
        or payload.get("realism", {}).get("recipe_sha256") != realism["recipe_sha256"]  # type: ignore[union-attr]
        or payload.get("realism", {}).get("layout_digest") != world["layout_digest"]  # type: ignore[union-attr]
        or {record.get("target_id") for record in screenshots if isinstance(record, dict)}
        != REQUIRED_ACCEPTANCE_CAMERA_IDS
        or len(payload.get("stable_actor_world_transforms", [])) != expected_environment_count  # type: ignore[arg-type]
    ):
        raise RuntimeError("previous FactoryPark_HF success evidence is not safe for screenshot-only refresh")


def _refresh_acceptance_screenshots(*, refresh_rendering: bool) -> dict[str, object]:
    """Reopen the proven map and atomically refresh cameras plus optional rendering."""

    try:
        PREVIOUS_SUCCESS_PATH.relative_to(EVIDENCE_ROOT)
    except ValueError as error:
        raise RuntimeError("previous success evidence escapes the UE evidence root") from error
    derived_data_cache = _validate_derived_data_cache()
    world = _validate_world_recipe()
    realism = _validate_realism_recipe(world)
    authoring = _validate_blender_authoring(world, realism)
    payload = _load_json_object(PREVIOUS_SUCCESS_PATH)
    _validate_refresh_source_evidence(payload, world, realism, authoring)
    source_success_sha256 = _sha256_file(PREVIOUS_SUCCESS_PATH)

    level_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if not unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH) or not level_subsystem.load_level(MAP_PATH):
        raise RuntimeError(f"screenshot refresh could not load proven map {MAP_PATH}")
    rendering_refresh: dict[str, object] | None = None
    if refresh_rendering:
        master_material = unreal.load_asset(MASTER_MATERIAL_PATH)
        if master_material is None:
            raise RuntimeError(
                f"rendering refresh could not load native PBR master material {MASTER_MATERIAL_PATH}"
            )
        rendering_refresh = _refresh_existing_rendering_contract(
            actor_subsystem,
            realism,
            master_material,
        )
        unreal.EditorAssetLibrary.save_loaded_asset(master_material, only_if_is_dirty=False)
    actors_before = actor_subsystem.get_all_level_actors()
    environment_actor_count_before = sum(
        any(tag.startswith("StableId:") for tag in _actor_tags(actor))
        for actor in actors_before
    )
    camera_actors = [actor for actor in actors_before if "AcceptanceCamera" in _actor_tags(actor)]
    expected_camera_count = len(authoring["realism"]["preview_targets"])  # type: ignore[index]
    if len(camera_actors) != expected_camera_count:
        raise RuntimeError(
            "screenshot refresh found the wrong acceptance-camera count: "
            f"{len(camera_actors)}/{expected_camera_count}"
        )
    for camera_actor in camera_actors:
        if not actor_subsystem.destroy_actor(camera_actor):
            raise RuntimeError(f"could not replace acceptance camera {camera_actor.get_actor_label()}")
    existing_tank_fill_lights = [
        actor
        for actor in actors_before
        if actor.get_actor_label() == TANK_BUND_FILL_LIGHT["label"]
    ]
    for light_actor in existing_tank_fill_lights:
        if not actor_subsystem.destroy_actor(light_actor):
            raise RuntimeError(f"could not replace lighting remediation {light_actor.get_actor_label()}")
    cameras, acceptance_cameras = _spawn_acceptance_cameras(
        actor_subsystem,
        authoring["realism"]["preview_targets"],  # type: ignore[index]
    )
    tank_fill, lighting_remediation = _spawn_tank_bund_fill_light(actor_subsystem)
    tank_fill_label = tank_fill.get_actor_label()
    visual_shadow_policy_write = _apply_visual_shadow_policy_existing_map(
        actor_subsystem,
        authoring,
        apply_policy=True,
        persisted_after_reload=False,
    )
    collision_persistence_policy_write = _apply_collision_policy_existing_map(
        actor_subsystem,
        authoring,
        apply_policy=True,
        persisted_after_reload=False,
    )
    environment_actor_count_after_write = sum(
        any(tag.startswith("StableId:") for tag in _actor_tags(actor))
        for actor in actor_subsystem.get_all_level_actors()
    )
    if (
        environment_actor_count_before != len(authoring["placed_objects"])  # type: ignore[arg-type]
        or environment_actor_count_after_write != environment_actor_count_before
    ):
        raise RuntimeError("screenshot refresh changed the environment actor set")
    if not level_subsystem.save_current_level():
        raise RuntimeError(f"screenshot refresh could not save {MAP_PATH}")
    if not level_subsystem.load_level(MAP_PATH):
        raise RuntimeError(f"screenshot refresh could not reload saved map {MAP_PATH}")
    environment_actor_count_after_reload = sum(
        any(tag.startswith("StableId:") for tag in _actor_tags(actor))
        for actor in actor_subsystem.get_all_level_actors()
    )
    if environment_actor_count_after_reload != environment_actor_count_before:
        raise RuntimeError("screenshot refresh changed the environment actor set after map reload")
    visual_shadow_policy = _apply_visual_shadow_policy_existing_map(
        actor_subsystem,
        authoring,
        apply_policy=False,
        persisted_after_reload=True,
    )
    visual_shadow_policy["component_flag_write_count"] = visual_shadow_policy_write[
        "component_flag_write_count"
    ]
    visual_shadow_policy["property_write_count"] = visual_shadow_policy_write[
        "property_write_count"
    ]
    visual_shadow_policy["changed_from_true_count"] = visual_shadow_policy_write[
        "changed_from_true_count"
    ]
    visual_shadow_policy["property_changed_from_true_counts"] = visual_shadow_policy_write[
        "property_changed_from_true_counts"
    ]
    collision_persistence_policy = _apply_collision_policy_existing_map(
        actor_subsystem,
        authoring,
        apply_policy=False,
        persisted_after_reload=True,
    )
    for field in (
        "component_modify_count",
        "component_modify_return_true_count",
        "component_modify_return_false_count",
        "component_modify_return_none_count",
        "collision_enabled_write_count",
        "collision_profile_write_count",
        "generate_overlap_events_write_count",
        "changed_from_enabled_count",
        "changed_profile_count",
        "changed_overlap_count",
    ):
        collision_persistence_policy[field] = collision_persistence_policy_write[field]
    loaded_map_audit = _audit_loaded_factory_map(
        actor_subsystem,
        authoring,
        payload["stable_actor_world_transforms"],  # type: ignore[arg-type]
        payload["native_pbr_materials"]["assignments"],  # type: ignore[arg-type,index]
    )
    loaded_map_audit["expected_contract_source"] = "previous_success_evidence"
    loaded_map_audit["source_success_path"] = str(PREVIOUS_SUCCESS_PATH)
    loaded_map_audit["source_success_sha256"] = source_success_sha256
    cameras = _find_acceptance_camera_actors(
        actor_subsystem,
        [str(target["id"]) for target in authoring["realism"]["preview_targets"]],  # type: ignore[index]
    )

    aerial_camera = next(record for record in acceptance_cameras if record["id"] == "site_aerial")
    lighting = dict(payload["lighting"])  # type: ignore[arg-type]
    lighting["actors"] = list(dict.fromkeys([*lighting["actors"], tank_fill_label]))  # type: ignore[index]
    lighting["movable_lights"] = list(
        dict.fromkeys([*lighting["movable_lights"], tank_fill_label])  # type: ignore[index]
    )
    lighting["lighting_remediation"] = lighting_remediation
    native_pbr_materials = dict(payload["native_pbr_materials"])  # type: ignore[arg-type]
    if rendering_refresh is not None:
        lighting["sun"] = rendering_refresh["sun"]
        lighting["lumen"] = rendering_refresh["lumen"]
        master_graph = dict(native_pbr_materials["master_graph"])  # type: ignore[arg-type]
        master_graph["material_usages"] = rendering_refresh["material_usages"]
        native_pbr_materials["master_graph"] = master_graph
    payload.update(
        {
            "engine_version": unreal.SystemLibrary.get_engine_version(),
            "derived_data_cache": derived_data_cache,
            "camera": {
                "label": aerial_camera["label"],
                "location_cm": aerial_camera["location_cm"],
                "target_cm": aerial_camera["look_at_cm"],
                "field_of_view_degrees": aerial_camera["field_of_view_degrees"],
                "auto_activate_for_player": 0,
            },
            "acceptance_cameras": acceptance_cameras,
            "lighting": lighting,
            "native_pbr_materials": native_pbr_materials,
            "visual_remediation": _visual_remediation_evidence(
                acceptance_cameras,
                lighting_remediation,
                visual_shadow_policy,
            ),
            "visual_shadow_policy": visual_shadow_policy,
            "collision_persistence_policy": collision_persistence_policy,
            "loaded_map_audit": loaded_map_audit,
            "screenshots": [],
            "screenshot_refresh": {
                "mode": "screenshot_refresh_v1",
                "source_success_path": str(PREVIOUS_SUCCESS_PATH),
                "source_success_sha256": source_success_sha256,
                "cameras_replaced": len(camera_actors),
                "lighting_actors_replaced": len(existing_tank_fill_lights),
                "environment_actor_count_before": environment_actor_count_before,
                "environment_actor_count_after": environment_actor_count_after_reload,
                "environment_actors_modified": True,
                "environment_component_flags_modified": visual_shadow_policy["actor_count"],
                "environment_actor_set_changed": False,
                "environment_transforms_modified": False,
                "environment_meshes_modified": False,
                "environment_materials_modified": rendering_refresh is not None,
                "environment_material_assignments_modified": False,
                "rendering_contract_refreshed": rendering_refresh is not None,
                "sun_and_exposure_modified": rendering_refresh is not None,
                "master_material_usage_modified": rendering_refresh is not None,
                "environment_collision_modified": (
                    collision_persistence_policy["changed_from_enabled_count"] > 0
                    or collision_persistence_policy["changed_profile_count"] > 0
                    or collision_persistence_policy["changed_overlap_count"] > 0
                ),
                "environment_collision_components_modified": collision_persistence_policy[
                    "component_modify_count"
                ],
                "physics_authority_modified": False,
                "map_saved_and_reloaded": True,
            },
        }
    )
    if rendering_refresh is not None:
        payload["screenshot_refresh"]["mode"] = "rendering_contract_refresh_v1"  # type: ignore[index]
    _schedule_acceptance_screenshots(payload, cameras, acceptance_cameras)
    return payload


def refresh_acceptance_screenshots() -> dict[str, object]:
    """Preserve the public screenshot-only refresh entry point."""

    return _refresh_acceptance_screenshots(refresh_rendering=False)


def refresh_rendering_contract() -> dict[str, object]:
    """Refresh render-critical material, sun, exposure, cameras, and evidence."""

    return _refresh_acceptance_screenshots(refresh_rendering=True)


def _quit_editor_if_unattended() -> None:
    if UNATTENDED:
        unreal.log("LINGTU_FACTORY_PARK_HF_UNATTENDED_EXIT=1")
        unreal.SystemLibrary.quit_editor()


def build_factory_park_hf() -> dict[str, object]:
    """Validate all authoring inputs, materialize the world, and write success evidence."""

    derived_data_cache = _validate_derived_data_cache()
    world = _validate_world_recipe()
    realism = _validate_realism_recipe(world)
    authoring = _validate_blender_authoring(world, realism)
    materials_by_identity, master_material, native_pbr_materials = _build_native_pbr_material_library(
        authoring,
        realism,
    )
    terrain_mesh, staged_terrain_path, staged_terrain_digest = _import_terrain(world)
    meshes_by_name = _import_blender_scene(authoring)
    used_meshes_by_path = {str(terrain_mesh.get_path_name()): terrain_mesh}
    for record in authoring["placed_objects"]:  # type: ignore[index]
        mesh = meshes_by_name[str(record["mesh_name"])]
        used_meshes_by_path[str(mesh.get_path_name())] = mesh
    used_meshes = list(used_meshes_by_path.values())
    nanite = _configure_optional_nanite(used_meshes, realism)

    level_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH):
        if not level_subsystem.load_level(MAP_PATH):
            raise RuntimeError(f"could not load FactoryPark_HF map {MAP_PATH}")
    elif not level_subsystem.new_level(MAP_PATH, False):
        raise RuntimeError(f"could not create FactoryPark_HF map {MAP_PATH}")
    _clear_current_level(actor_subsystem)

    terrain_surface = _resolve_pbr_surface("gravel", "terrain", realism)
    terrain_material = materials_by_identity[("gravel", "terrain")]
    terrain_mesh_slot_count = _assign_static_mesh_material(
        terrain_mesh,
        terrain_material,
        terrain_surface,
    )
    terrain_actor = _spawn_terrain(
        actor_subsystem,
        terrain_mesh,
        str(world["terrain"]["sha256"]),  # type: ignore[index]
        terrain_material,
    )
    environment_actors, stable_actor_world_transforms, material_assignments = _spawn_environment_objects(
        actor_subsystem,
        authoring,
        meshes_by_name,
        realism,
        materials_by_identity,
    )
    lighting = _spawn_outdoor_lighting(actor_subsystem, realism)
    cameras, acceptance_cameras = _spawn_acceptance_cameras(
        actor_subsystem,
        authoring["realism"]["preview_targets"],  # type: ignore[index]
    )
    terrain_actor_label = terrain_actor.get_actor_label()
    visual_shadow_policy_write = _apply_visual_shadow_policy_existing_map(
        actor_subsystem,
        authoring,
        apply_policy=True,
        persisted_after_reload=False,
    )
    collision_persistence_policy_write = _apply_collision_policy_existing_map(
        actor_subsystem,
        authoring,
        apply_policy=True,
        persisted_after_reload=False,
    )

    terrain_actor_count = _count_terrain_actors(actor_subsystem)
    if terrain_actor_count != 1:
        raise RuntimeError(f"FactoryPark_HF must contain exactly one authoritative terrain actor, found {terrain_actor_count}")
    preplaced_robot_bindings = _count_preplaced_robot_bindings(actor_subsystem)
    if preplaced_robot_bindings != PREPLACED_ROBOT_BINDINGS:
        raise RuntimeError(
            f"FactoryPark_HF must remain robot-free, found {preplaced_robot_bindings} LingTu body bindings"
        )
    robot_mesh_references = _count_robot_mesh_references(actor_subsystem)
    if robot_mesh_references != 0:
        raise RuntimeError(f"FactoryPark_HF contains {robot_mesh_references} preplaced robot mesh references")
    if not level_subsystem.save_current_level():
        raise RuntimeError(f"could not save FactoryPark_HF map {MAP_PATH}")
    if not level_subsystem.load_level(MAP_PATH):
        raise RuntimeError(f"could not reload saved FactoryPark_HF map {MAP_PATH}")
    visual_shadow_policy = _apply_visual_shadow_policy_existing_map(
        actor_subsystem,
        authoring,
        apply_policy=False,
        persisted_after_reload=True,
    )
    visual_shadow_policy["component_flag_write_count"] = visual_shadow_policy_write[
        "component_flag_write_count"
    ]
    visual_shadow_policy["property_write_count"] = visual_shadow_policy_write[
        "property_write_count"
    ]
    visual_shadow_policy["changed_from_true_count"] = visual_shadow_policy_write[
        "changed_from_true_count"
    ]
    visual_shadow_policy["property_changed_from_true_counts"] = visual_shadow_policy_write[
        "property_changed_from_true_counts"
    ]
    collision_persistence_policy = _apply_collision_policy_existing_map(
        actor_subsystem,
        authoring,
        apply_policy=False,
        persisted_after_reload=True,
    )
    for field in (
        "component_modify_count",
        "component_modify_return_true_count",
        "component_modify_return_false_count",
        "component_modify_return_none_count",
        "collision_enabled_write_count",
        "collision_profile_write_count",
        "generate_overlap_events_write_count",
        "changed_from_enabled_count",
        "changed_profile_count",
        "changed_overlap_count",
    ):
        collision_persistence_policy[field] = collision_persistence_policy_write[field]
    loaded_map_audit = _audit_loaded_factory_map(
        actor_subsystem,
        authoring,
        stable_actor_world_transforms,
        material_assignments,
    )
    loaded_map_audit["expected_contract_source"] = "current_build_evidence"
    cameras = _find_acceptance_camera_actors(
        actor_subsystem,
        [str(target["id"]) for target in authoring["realism"]["preview_targets"]],  # type: ignore[index]
    )
    if (
        _count_terrain_actors(actor_subsystem) != terrain_actor_count
        or _count_preplaced_robot_bindings(actor_subsystem) != preplaced_robot_bindings
        or _count_robot_mesh_references(actor_subsystem) != robot_mesh_references
    ):
        raise RuntimeError("FactoryPark_HF authority counts changed after saved-map reload")

    physics_shared_count = sum(not bool(record["visual_only"]) for record in authoring["placed_objects"])
    visual_only_count = sum(bool(record["visual_only"]) for record in authoring["placed_objects"])
    native_pbr_materials.update(
        {
            "master_material": str(master_material.get_path_name()),
            "environment_mesh_assignment_count": len(material_assignments),
            "environment_component_assignment_count": len(material_assignments),
            "terrain_assignment_count": 1,
            "terrain_mesh_slot_count": terrain_mesh_slot_count,
            "assignments": material_assignments,
        }
    )
    aerial_camera = next(record for record in acceptance_cameras if record["id"] == "site_aerial")
    payload: dict[str, object] = {
        "schema": "lingtu.sim.unreal-factory-park-hf-evidence.v1",
        "result": "success",
        "map_path": MAP_PATH,
        "engine_version": unreal.SystemLibrary.get_engine_version(),
        "derived_data_cache": derived_data_cache,
        "world_package": WORLD_PACKAGE,
        "world_recipe": {
            "path": str(world["recipe_path"]),
            "sha256": world["recipe_sha256"],
        },
        "realism": {
            "profile": realism["profile"],
            "recipe": str(realism["recipe_path"]),
            "recipe_sha256": realism["recipe_sha256"],
            "layout_digest": realism["layout_digest"],
            "seed": realism["seed"],
            "hard_rules": realism["hard_rules"],
            "performance_budgets": realism["performance_budgets"],
        },
        "expanded_layout": {
            "path": str(world["layout_path"]),
            "sha256": world["layout_sha256"],
            "layout_digest": world["layout_digest"],
            "object_count": len(world["layout_objects"]),
        },
        "blender_authoring": {
            "manifest": str(authoring["manifest_path"]),
            "manifest_sha256": authoring["manifest_sha256"],
            "artifact_digest": str(authoring["artifact_digest_path"]),
            "artifact_digest_sha256": authoring["artifact_digest_sha256"],
            "artifact_set_digest": authoring["artifact_set_digest"],
            "scene_asset_role": authoring["scene_asset_role"],
            "scene_asset": str(authoring["scene_asset_path"]),
            "realism": authoring["realism"],
        },
        "terrain": {
            "actor": terrain_actor_label,
            "actor_count": terrain_actor_count,
            "asset": TERRAIN_MESH_PATH,
            "source": str(world["terrain"]["path"]),  # type: ignore[index]
            "source_sha256": world["terrain"]["sha256"],  # type: ignore[index]
            "staged_source": str(staged_terrain_path),
            "staged_sha256": staged_terrain_digest,
            "actor_scale": [1.0, 1.0, 1.0],
            "collision": "disabled_in_unreal",
            "authority": "MuJoCo heightfield",
            "spawn_height_cm": world["terrain"]["spawn_height_cm"],  # type: ignore[index]
            "landscape_location_cm": list(world["terrain"]["landscape_location_cm"]),  # type: ignore[index]
        },
        "environment": {
            "actor_count": len(environment_actors),
            "actor_scale": [
                ENVIRONMENT_ACTOR_UNIT_SCALE,
                ENVIRONMENT_ACTOR_UNIT_SCALE,
                ENVIRONMENT_ACTOR_UNIT_SCALE,
            ],
            "physics_shared_count": physics_shared_count,
            "visual_only_count": visual_only_count,
            "terrain_feature_descriptor_count": len(authoring["terrain_feature_descriptors"]),
            "semantic_feature_descriptor_count": len(authoring["semantic_feature_descriptors"]),
            "semantic_descriptors_not_materialized": authoring[
                "semantic_descriptors_not_materialized"
            ],
            "collision": "disabled_in_unreal",
            "physics_shared_tag": PHYSICS_SHARED_TAG,
            "visual_only_tag": VISUAL_ONLY_TAG,
        },
        "stable_actor_world_transforms": stable_actor_world_transforms,
        "semantic_feature_descriptors": {
            "count": len(authoring["semantic_feature_descriptors"]),
            "stable_ids": sorted(
                str(record["stable_id"])
                for record in authoring["semantic_feature_descriptors"]
            ),
            "semantic_descriptors_not_materialized": True,
            "actor_count": 0,
            "mesh_count": 0,
        },
        "native_pbr_materials": native_pbr_materials,
        "nanite": nanite,
        "coordinate_conversion": {
            "source": "mujoco_rh_z_up_m",
            "target": "unreal_lh_z_up_cm",
            "position": ["100*x", "-100*y", "100*z"],
            "quaternion_wxyz_to_xyzw": ["-x", "y", "-z", "w"],
            "fbx_mesh_vertices": "local_origin",
            "fbx_mesh_numeric_units": "authored_metre_values_interpreted_as_unreal_units",
            "transform_vertex_to_absolute": False,
            "fbx_import_uniform_scale_requested": FBX_IMPORT_UNIFORM_SCALE,
            "fbx_import_uniform_scale_supported": False,
            "fbx_import_observed_effect": "mesh_dimensions_remain_authored_metre_numeric_values",
            "environment_actor_unit_scale": ENVIRONMENT_ACTOR_UNIT_SCALE,
            "terrain_actor_unit_scale": TERRAIN_ACTOR_UNIT_SCALE,
            "unit_conversion_strategy": "placement_actor_scale",
            "world_dimension_rule": "mesh_asset_dimension_uu * actor_scale_100 = centimetres",
        },
        "lighting": lighting,
        "camera": {
            "label": aerial_camera["label"],
            "location_cm": aerial_camera["location_cm"],
            "target_cm": aerial_camera["look_at_cm"],
            "field_of_view_degrees": aerial_camera["field_of_view_degrees"],
            "auto_activate_for_player": 0,
        },
        "acceptance_cameras": acceptance_cameras,
        "visual_remediation": _visual_remediation_evidence(
            acceptance_cameras,
            lighting["lighting_remediation"],  # type: ignore[arg-type]
            visual_shadow_policy,
        ),
        "visual_shadow_policy": visual_shadow_policy,
        "collision_persistence_policy": collision_persistence_policy,
        "loaded_map_audit": loaded_map_audit,
        "screenshots": [],
        "preplaced_robot_bindings": preplaced_robot_bindings,
        "robot_actor_count": 0,
        "robot_mesh_references": robot_mesh_references,
        "mujoco_motion_authority": True,
        "evidence_scope": {
            "editor_map": True,
            "cook": False,
            "stage": False,
            "package": False,
        },
    }
    _schedule_acceptance_screenshots(payload, cameras, acceptance_cameras)
    return payload


def main() -> None:
    """Run the explicit Unreal Editor build entry point with fail-closed evidence."""

    try:
        if unreal is None:
            raise RuntimeError("build_factory_park_hf.py must run inside Unreal Editor Python")
        if REFRESH_SCREENSHOTS_ONLY and REFRESH_RENDERING_ONLY:
            raise RuntimeError("FactoryPark_HF refresh modes are mutually exclusive")
        if REFRESH_RENDERING_ONLY:
            refresh_rendering_contract()
        elif REFRESH_SCREENSHOTS_ONLY:
            refresh_acceptance_screenshots()
        else:
            build_factory_park_hf()
    except Exception as error:
        _write_error_evidence(error)
        if unreal is not None:
            _quit_editor_if_unattended()
        raise
    if not _SCREENSHOT_CAPTURE_PENDING:
        _quit_editor_if_unattended()


if __name__ == "__main__":
    main()
