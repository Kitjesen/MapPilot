"""Build the deterministic OpenField_HF editor map and capture its hero view."""

from __future__ import annotations

import hashlib
import json
import math
import os
import random
import struct
import traceback
from pathlib import Path

try:
    import unreal
except ModuleNotFoundError:
    unreal = None


PROJECT_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]

WORLD_RECIPE_PATH = Path(
    os.environ.get(
        "LINGTU_OPEN_FIELD_HF_RECIPE",
        REPO_ROOT / "sim" / "packages" / "worlds" / "open_field_hf" / "visual" / "ue_import.recipe.json",
    )
).resolve()
ROBOT_RECIPE_PATH = Path(
    os.environ.get(
        "LINGTU_OPEN_FIELD_HF_ROBOT_RECIPE",
        REPO_ROOT / "build" / "unreal-assets" / "thunderv4-runtime.recipe.json",
    )
).resolve()
FBX_DIR = Path(
    os.environ.get(
        "LINGTU_OPEN_FIELD_HF_FBX_DIR",
        REPO_ROOT / "build" / "unreal-assets" / "thunderv4-mjcf-fbx",
    )
).resolve()
EVIDENCE_DIR = Path(
    os.environ.get(
        "LINGTU_OPEN_FIELD_HF_EVIDENCE_DIR",
        REPO_ROOT / "build" / "unreal-open-field-hf",
    )
).resolve()
SCREENSHOT_PATH = Path(
    os.environ.get(
        "LINGTU_OPEN_FIELD_HF_SCREENSHOT",
        EVIDENCE_DIR / "OpenField_HF_1920x1080.png",
    )
).resolve()
SUCCESS_SENTINEL = Path(
    os.environ.get(
        "LINGTU_OPEN_FIELD_HF_SUCCESS",
        EVIDENCE_DIR / "OpenField_HF.success.json",
    )
).resolve()
ERROR_SENTINEL = Path(
    os.environ.get(
        "LINGTU_OPEN_FIELD_HF_ERROR",
        EVIDENCE_DIR / "OpenField_HF.error.txt",
    )
).resolve()
UNATTENDED = os.environ.get("LINGTU_OPEN_FIELD_HF_UNATTENDED") == "1"

MAP_PATH = "/Game/RobotSim/Maps/OpenField_HF"
WORLD_ASSET_DESTINATION = "/Game/RobotSim/Worlds/OpenFieldHF"
TERRAIN_MESH_DESTINATION = f"{WORLD_ASSET_DESTINATION}/Meshes"
TERRAIN_MESH_NAME = "SM_OpenField_HF_Terrain"
TERRAIN_MESH_PATH = f"{TERRAIN_MESH_DESTINATION}/{TERRAIN_MESH_NAME}.{TERRAIN_MESH_NAME}"
MATERIAL_DESTINATION = f"{WORLD_ASSET_DESTINATION}/Materials"
ROBOT_MESH_DESTINATION = "/Game/RobotSim/Robots/ThunderV4/Meshes"
EXPECTED_ROBOT_BODY_COUNT = 21
VISUAL_ONLY_TAG = "VisualOnly"
SCREENSHOT_WIDTH = 1920
SCREENSHOT_HEIGHT = 1080
HERO_CAMERA_X_CM = 235.0
HERO_CAMERA_Y_CM = -310.0
HERO_CAMERA_HEIGHT_CM = 145.0
HERO_CAMERA_TARGET_Z_CM = 70.0
HERO_CAMERA_FOV_DEGREES = 36.0
HERO_VIEW_NEAR_CLEARANCE_CM = 480.0
HERO_VIEW_FAR_CLEARANCE_CM = 4000.0
HERO_VIEW_FRUSTUM_MARGIN_CM = 500.0

_SCREENSHOT_TASK = None
_SCREENSHOT_TICK_HANDLE = None
_SCREENSHOT_TICK_CALLBACK = None
_SUCCESS_PAYLOAD: dict[str, object] | None = None


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


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise RuntimeError(f"JSON root must be an object: {path}")
    return value


def _resolve_repo_source(reference: object) -> Path:
    if not isinstance(reference, str) or not reference.strip():
        raise RuntimeError(f"invalid repository source path: {reference!r}")
    relative = Path(reference)
    if relative.is_absolute():
        raise RuntimeError(f"source path must be repository-relative: {reference}")
    resolved = (REPO_ROOT / relative).resolve()
    try:
        resolved.relative_to(REPO_ROOT)
    except ValueError as error:
        raise RuntimeError(f"source escapes repository root: {reference}") from error
    return resolved


def _validate_source_record(name: str, record: object) -> Path:
    if not isinstance(record, dict):
        raise RuntimeError(f"world recipe source {name} is not an object")
    path = _resolve_repo_source(record.get("path"))
    if not path.is_file():
        raise RuntimeError(f"world recipe source is missing: {path}")
    expected_bytes = record.get("bytes")
    if expected_bytes != path.stat().st_size:
        raise RuntimeError(
            f"world recipe byte count mismatch for {name}: expected={expected_bytes}, actual={path.stat().st_size}"
        )
    actual_digest = _sha256_file(path)
    if record.get("sha256") != actual_digest:
        raise RuntimeError(
            f"world recipe SHA256 mismatch for {name}: expected={record.get('sha256')}, actual={actual_digest}"
        )
    return path


def _asset_set_digest(records: list[dict]) -> str:
    identity = [
        {"path": record["path"], "sha256": record["sha256"]}
        for record in sorted(records, key=lambda item: str(item["path"]))
    ]
    return hashlib.sha256(_canonical_json(identity)).hexdigest()


def _load_terrain_vertices(
    terrain_path: Path,
    expected_grid: tuple[int, int],
) -> tuple[tuple[float, float, float], ...]:
    vertices: list[tuple[float, float, float]] = []
    triangle_count = 0
    coordinates_declared = False
    with terrain_path.open("r", encoding="ascii") as stream:
        for line in stream:
            if line.startswith("# coordinates=unreal_lh_z_up_cm"):
                coordinates_declared = True
            elif line.startswith("v "):
                parts = line.split()
                if len(parts) != 4:
                    raise RuntimeError(f"malformed OBJ vertex in {terrain_path}")
                vertices.append(tuple(float(value) for value in parts[1:4]))
            elif line.startswith("f "):
                triangle_count += 1
    expected_vertices = expected_grid[0] * expected_grid[1]
    expected_triangles = 2 * (expected_grid[0] - 1) * (expected_grid[1] - 1)
    if not coordinates_declared:
        raise RuntimeError("terrain OBJ does not declare unreal_lh_z_up_cm coordinates")
    if len(vertices) != expected_vertices or triangle_count != expected_triangles:
        raise RuntimeError(
            "terrain OBJ topology disagrees with the source grid: "
            f"vertices={len(vertices)}/{expected_vertices}, "
            f"triangles={triangle_count}/{expected_triangles}"
        )
    return tuple(vertices)


def _stage_unreal_obj(
    source: Path,
    grid: tuple[int, int],
    source_digest: str,
) -> tuple[Path, str]:
    """Add deterministic one-to-one UV indices for UE 5.8 Interchange."""

    preface_and_vertices: list[str] = []
    faces: list[tuple[int, ...]] = []
    with source.open("r", encoding="ascii") as stream:
        for line in stream:
            stripped = line.rstrip("\r\n")
            if stripped.startswith("f "):
                indices = tuple(int(token.split("/", 1)[0]) for token in stripped.split()[1:])
                if len(indices) != 3 or any(index <= 0 for index in indices):
                    raise RuntimeError("terrain OBJ contains a non-triangle face")
                faces.append(indices)
            elif not stripped.startswith(("vt ", "vn ")):
                preface_and_vertices.append(stripped)

    width, height = grid
    vertex_count = sum(line.startswith("v ") for line in preface_and_vertices)
    if vertex_count != width * height:
        raise RuntimeError("UE staging vertex count disagrees with terrain grid")

    output = list(preface_and_vertices)
    output.append(f"# authoritative_source_sha256={source_digest}")
    output.append("# ue58_interchange_uv_contract=vertex_index_equals_uv_index")
    for row in range(height):
        v = row / (height - 1)
        for column in range(width):
            u = column / (width - 1)
            output.append(f"vt {u:.9f} {v:.9f}")
    output.extend("f " + " ".join(f"{index}/{index}" for index in face) for face in faces)

    EVIDENCE_DIR.mkdir(parents=True, exist_ok=True)
    staged_path = EVIDENCE_DIR / "OpenField_HF_UE58_import.obj"
    staged_path.write_text("\n".join(output) + "\n", encoding="ascii")
    staged_digest = _sha256_file(staged_path)
    if unreal is not None:
        unreal.log(
            "LINGTU_OPEN_FIELD_HF_OBJ_STAGED "
            f"source_sha256={source_digest} staged_sha256={staged_digest} "
            f"vertices={vertex_count} faces={len(faces)} uv={vertex_count}"
        )
    return staged_path, staged_digest


def _validate_world_recipe() -> dict[str, object]:
    recipe = _load_json(WORLD_RECIPE_PATH)
    if recipe.get("schema") != "lingtu.sim.unreal-world-import-recipe.v1":
        raise RuntimeError(f"unsupported OpenField_HF recipe: {WORLD_RECIPE_PATH}")
    if recipe.get("world_package") != "open_field_hf@1.0.0":
        raise RuntimeError("world recipe does not resolve open_field_hf@1.0.0")
    if recipe.get("binding") != "WorldVisual:OpenFieldHF":
        raise RuntimeError("world recipe binding is not WorldVisual:OpenFieldHF")
    if recipe.get("target_level") != MAP_PATH:
        raise RuntimeError(f"world recipe target must be {MAP_PATH}, got {recipe.get('target_level')}")

    mesh_import = recipe.get("blender_mesh_import")
    if not isinstance(mesh_import, dict):
        raise RuntimeError("world recipe has no blender_mesh_import contract")
    if mesh_import.get("coordinates") != "unreal_lh_z_up_cm":
        raise RuntimeError("terrain mesh is not declared in Unreal centimetres")
    if mesh_import.get("import_scale") != 1.0:
        raise RuntimeError("OpenField_HF terrain import scale must be exactly 1.0")
    if mesh_import.get("vertex_per_height_sample") is not True:
        raise RuntimeError("terrain mesh must preserve one vertex per height sample")

    sources = recipe.get("sources")
    if not isinstance(sources, dict):
        raise RuntimeError("world recipe has no sources object")
    required_sources = {"asset_manifest", "heightfield_png", "terrain_obj"}
    missing_sources = required_sources.difference(sources)
    if missing_sources:
        raise RuntimeError(f"world recipe is missing sources: {sorted(missing_sources)}")
    resolved_sources = {name: _validate_source_record(name, record) for name, record in sources.items()}

    terrain_path = resolved_sources["terrain_obj"]
    declared_terrain_path = _resolve_repo_source(mesh_import.get("source"))
    if terrain_path != declared_terrain_path:
        raise RuntimeError("terrain OBJ source disagrees with blender_mesh_import.source")
    landscape_import = recipe.get("unreal_landscape_import")
    if not isinstance(landscape_import, dict):
        raise RuntimeError("world recipe has no Unreal heightfield import contract")
    declared_heightfield = _resolve_repo_source(landscape_import.get("recommended_source"))
    if declared_heightfield != resolved_sources["heightfield_png"]:
        raise RuntimeError("heightfield PNG source paths disagree")

    coordinate_contract = recipe.get("coordinate_contract")
    if not isinstance(coordinate_contract, dict):
        raise RuntimeError("world recipe has no coordinate contract")
    grid_value = coordinate_contract.get("grid_px")
    extent_value = coordinate_contract.get("extent_m")
    if grid_value != [253, 253] or extent_value != [160.0, 160.0]:
        raise RuntimeError(f"unexpected OpenField_HF geometry: grid={grid_value}, extent={extent_value}")
    vertices = _load_terrain_vertices(terrain_path, (253, 253))
    min_x = min(vertex[0] for vertex in vertices)
    max_x = max(vertex[0] for vertex in vertices)
    min_y = min(vertex[1] for vertex in vertices)
    max_y = max(vertex[1] for vertex in vertices)
    if not (
        math.isclose(min_x, -8000.0, abs_tol=0.01)
        and math.isclose(max_x, 8000.0, abs_tol=0.01)
        and math.isclose(min_y, -8000.0, abs_tol=0.01)
        and math.isclose(max_y, 8000.0, abs_tol=0.01)
    ):
        raise RuntimeError("terrain OBJ is not a 160 m Unreal-centimetre mesh")

    asset_manifest_path = resolved_sources["asset_manifest"]
    asset_manifest = _load_json(asset_manifest_path)
    if asset_manifest.get("schema") != "lingtu.sim.world-asset-manifest.v1":
        raise RuntimeError("unsupported OpenField_HF asset manifest")
    if asset_manifest.get("world_package") != recipe.get("world_package"):
        raise RuntimeError("world recipe and asset manifest package identities disagree")
    assets = asset_manifest.get("assets")
    if not isinstance(assets, list) or not assets:
        raise RuntimeError("OpenField_HF asset manifest contains no assets")
    if asset_manifest.get("asset_set_digest") != _asset_set_digest(assets):
        raise RuntimeError("OpenField_HF asset_set_digest is invalid")
    manifest_by_path = {record.get("path"): record for record in assets}
    for source_name in ("heightfield_png", "terrain_obj"):
        source_record = sources[source_name]
        manifest_record = manifest_by_path.get(source_record["path"])
        if manifest_record is None:
            raise RuntimeError(f"asset manifest omits {source_name}")
        if manifest_record.get("sha256") != source_record.get("sha256"):
            raise RuntimeError(f"asset manifest digest disagrees for {source_name}")

    return {
        "recipe": recipe,
        "recipe_digest": _sha256_file(WORLD_RECIPE_PATH),
        "asset_manifest": asset_manifest,
        "asset_manifest_digest": _sha256_file(asset_manifest_path),
        "terrain_path": terrain_path,
        "terrain_digest": sources["terrain_obj"]["sha256"],
        "heightfield_path": resolved_sources["heightfield_png"],
        "heightfield_digest": sources["heightfield_png"]["sha256"],
        "grid": (253, 253),
        "vertices": vertices,
        "bounds_cm": (min_x, max_x, min_y, max_y),
    }


def _validate_robot_recipe() -> dict:
    recipe = _load_json(ROBOT_RECIPE_PATH)
    if recipe.get("schema") != "lingtu.sim.unreal-preview-recipe.v1":
        raise RuntimeError(f"unsupported ThunderV4 visual recipe: {ROBOT_RECIPE_PATH}")
    if recipe.get("instance_id") != "thunder_01":
        raise RuntimeError("OpenField_HF preview requires instance thunder_01")
    bodies = recipe.get("bodies")
    components = recipe.get("components")
    if not isinstance(bodies, list) or not isinstance(components, list):
        raise RuntimeError("ThunderV4 recipe has no body/component arrays")
    body_ids = {body.get("stable_id") for body in bodies}
    component_ids = {component.get("stable_id") for component in components}
    if len(bodies) != EXPECTED_ROBOT_BODY_COUNT or len(body_ids) != len(bodies):
        raise RuntimeError(f"ThunderV4 must provide {EXPECTED_ROBOT_BODY_COUNT} unique body IDs")
    if len(components) != EXPECTED_ROBOT_BODY_COUNT or len(component_ids) != len(components):
        raise RuntimeError(f"ThunderV4 must provide {EXPECTED_ROBOT_BODY_COUNT} unique visual links")
    if any(component.get("body_frame_id") not in body_ids for component in components):
        raise RuntimeError("ThunderV4 visual component refers to an unknown body frame")
    for component in components:
        source = FBX_DIR / f"{component['asset_name']}.fbx"
        if not source.is_file():
            raise RuntimeError(f"missing staged ThunderV4 FBX: {source}")
    return recipe


def _write_text_sentinel(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _png_dimensions(path: Path) -> tuple[int, int]:
    with path.open("rb") as stream:
        header = stream.read(24)
    if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n":
        raise RuntimeError(f"screenshot is not a PNG: {path}")
    return struct.unpack(">II", header[16:24])


def _quit_editor_if_unattended() -> None:
    if UNATTENDED:
        unreal.log("LINGTU_OPEN_FIELD_HF_UNATTENDED_EXIT=1")
        unreal.SystemLibrary.quit_editor()


def _set_editor_property_if_supported(
    target: object,
    property_name: str,
    value: object,
) -> None:
    try:
        target.set_editor_property(property_name, value)
    except Exception as error:
        unreal.log_warning(f"OpenField_HF property unavailable: {property_name} ({error})")


def _disable_collision(mesh_component: object) -> None:
    try:
        mesh_component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    except Exception:
        _set_editor_property_if_supported(mesh_component, "collision_profile_name", "NoCollision")


def _import_static_mesh(
    source: Path,
    destination_path: str,
    destination_name: str,
) -> object:
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", str(source))
    task.set_editor_property("destination_path", destination_path)
    task.set_editor_property("destination_name", destination_name)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("replace_existing_settings", True)
    task.set_editor_property("save", True)
    task.set_editor_property("async_", False)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])

    expected_path = f"{destination_path}/{destination_name}.{destination_name}"
    mesh = unreal.load_asset(expected_path)
    if mesh is None:
        imported_paths = list(task.get_editor_property("imported_object_paths") or ())
        for imported_path in imported_paths:
            candidate = unreal.load_asset(str(imported_path))
            if candidate is not None and isinstance(candidate, unreal.StaticMesh):
                mesh = candidate
                break
    if mesh is None or not isinstance(mesh, unreal.StaticMesh):
        raise RuntimeError(
            f"Unreal did not import a StaticMesh from {source}; "
            f"reported={task.get_editor_property('imported_object_paths')}"
        )
    return mesh


def _validate_terrain_mesh_bounds(mesh: object) -> None:
    bounding_box = mesh.get_bounding_box()
    size_x = bounding_box.max.x - bounding_box.min.x
    size_y = bounding_box.max.y - bounding_box.min.y
    if not (
        math.isclose(size_x, 16000.0, rel_tol=0.0, abs_tol=1.0)
        and math.isclose(size_y, 16000.0, rel_tol=0.0, abs_tol=1.0)
    ):
        raise RuntimeError(
            f"terrain import violated the 1:1 centimetre contract: bounds=({size_x:.3f}, {size_y:.3f}) cm"
        )


def _import_terrain(world: dict[str, object]) -> object:
    staged_path, staged_digest = _stage_unreal_obj(
        world["terrain_path"],
        world["grid"],
        str(world["terrain_digest"]),
    )
    expected_digest = str(world["terrain_digest"])
    mesh = unreal.load_asset(TERRAIN_MESH_PATH)
    if mesh is not None and isinstance(mesh, unreal.StaticMesh):
        source_digest = str(unreal.EditorAssetLibrary.get_metadata_tag(mesh, "LingTu.SourceSha256") or "")
        import_scale = str(unreal.EditorAssetLibrary.get_metadata_tag(mesh, "LingTu.ImportScale") or "")
        if source_digest == expected_digest and import_scale == "1.0":
            _validate_terrain_mesh_bounds(mesh)
            unreal.log(
                "LINGTU_OPEN_FIELD_HF_TERRAIN_REUSED "
                f"asset={TERRAIN_MESH_PATH} source_sha256={expected_digest} scale=1.0"
            )
        else:
            mesh = None
    if mesh is None:
        mesh = _import_static_mesh(
            staged_path,
            TERRAIN_MESH_DESTINATION,
            TERRAIN_MESH_NAME,
        )
        _validate_terrain_mesh_bounds(mesh)
        unreal.log(
            f"LINGTU_OPEN_FIELD_HF_TERRAIN_IMPORTED asset={TERRAIN_MESH_PATH} source_sha256={expected_digest} scale=1.0"
        )
    unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.SourceSha256", str(world["terrain_digest"]))
    unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.ImportScale", "1.0")
    unreal.EditorAssetLibrary.save_loaded_asset(mesh, only_if_is_dirty=False)
    world["ue_import_staging_path"] = staged_path
    world["ue_import_staging_digest"] = staged_digest
    return mesh


def _import_robot_meshes(recipe: dict) -> None:
    missing = [component for component in recipe["components"] if unreal.load_asset(component["unreal_asset"]) is None]
    if missing:
        tasks = []
        for component in sorted(missing, key=lambda item: item["asset_name"]):
            task = unreal.AssetImportTask()
            task.set_editor_property("filename", str(FBX_DIR / f"{component['asset_name']}.fbx"))
            task.set_editor_property("destination_path", ROBOT_MESH_DESTINATION)
            task.set_editor_property("destination_name", component["asset_name"])
            task.set_editor_property("automated", True)
            task.set_editor_property("replace_existing", False)
            task.set_editor_property("save", True)
            task.set_editor_property("async_", False)
            tasks.append(task)
        unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks(tasks)
    missing_after = [
        component["unreal_asset"]
        for component in recipe["components"]
        if unreal.load_asset(component["unreal_asset"]) is None
    ]
    if missing_after:
        raise RuntimeError(f"ThunderV4 mesh import is incomplete: {missing_after}")


def _constant(material: object, value: float, x: int, y: int) -> object:
    expression = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionConstant, x, y
    )
    expression.set_editor_property("r", value)
    return expression


def _color(
    material: object,
    value: tuple[float, float, float],
    x: int,
    y: int,
) -> object:
    expression = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionConstant3Vector, x, y
    )
    expression.set_editor_property("constant", unreal.LinearColor(*value, 1.0))
    return expression


def _create_solid_material(
    name: str,
    base_color: tuple[float, float, float],
    roughness: float,
    metallic: float = 0.0,
) -> object:
    asset_path = f"{MATERIAL_DESTINATION}/{name}"
    material = unreal.load_asset(asset_path)
    if material is None:
        material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
            name,
            MATERIAL_DESTINATION,
            unreal.Material,
            unreal.MaterialFactoryNew(),
        )
    if material is None:
        raise RuntimeError(f"could not create material {asset_path}")
    editing = unreal.MaterialEditingLibrary
    editing.delete_all_material_expressions(material)
    color = _color(material, base_color, -420, -80)
    roughness_node = _constant(material, roughness, -420, 100)
    metallic_node = _constant(material, metallic, -420, 190)
    editing.connect_material_property(color, "", unreal.MaterialProperty.MP_BASE_COLOR)
    editing.connect_material_property(roughness_node, "", unreal.MaterialProperty.MP_ROUGHNESS)
    editing.connect_material_property(metallic_node, "", unreal.MaterialProperty.MP_METALLIC)
    editing.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False)
    return material


def _create_terrain_material() -> object:
    name = "M_OpenField_HF_Terrain"
    asset_path = f"{MATERIAL_DESTINATION}/{name}"
    material = unreal.load_asset(asset_path)
    if material is None:
        material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
            name,
            MATERIAL_DESTINATION,
            unreal.Material,
            unreal.MaterialFactoryNew(),
        )
    if material is None:
        raise RuntimeError(f"could not create terrain material {asset_path}")

    editing = unreal.MaterialEditingLibrary
    editing.delete_all_material_expressions(material)
    grass = _color(material, (0.055, 0.105, 0.028), -820, -150)
    mud = _color(material, (0.120, 0.055, 0.020), -820, 20)
    rock = _color(material, (0.180, 0.180, 0.160), -820, 190)
    world_position = editing.create_material_expression(material, unreal.MaterialExpressionWorldPosition, -1120, 20)

    ground_frequency = _constant(material, 0.00042, -1120, 160)
    ground_position = editing.create_material_expression(material, unreal.MaterialExpressionMultiply, -900, -10)
    ground_noise = editing.create_material_expression(material, unreal.MaterialExpressionNoise, -660, -10)
    ground_noise.set_editor_property("levels", 1)
    ground_noise.set_editor_property("quality", 1)
    ground_noise.set_editor_property("turbulence", False)
    ground_noise.set_editor_property("output_min", 0.0)
    ground_noise.set_editor_property("output_max", 1.0)
    editing.connect_material_expressions(world_position, "", ground_position, "A")
    editing.connect_material_expressions(ground_frequency, "", ground_position, "B")
    editing.connect_material_expressions(ground_position, "", ground_noise, "Position")

    rock_frequency = _constant(material, 0.00016, -1120, 330)
    rock_position = editing.create_material_expression(material, unreal.MaterialExpressionMultiply, -900, 260)
    rock_noise = editing.create_material_expression(material, unreal.MaterialExpressionNoise, -660, 260)
    rock_noise.set_editor_property("levels", 1)
    rock_noise.set_editor_property("quality", 1)
    rock_noise.set_editor_property("turbulence", False)
    rock_noise.set_editor_property("output_min", 0.0)
    rock_noise.set_editor_property("output_max", 1.0)
    editing.connect_material_expressions(world_position, "", rock_position, "A")
    editing.connect_material_expressions(rock_frequency, "", rock_position, "B")
    editing.connect_material_expressions(rock_position, "", rock_noise, "Position")

    ground_mix = editing.create_material_expression(material, unreal.MaterialExpressionLinearInterpolate, -360, -40)
    editing.connect_material_expressions(grass, "", ground_mix, "A")
    editing.connect_material_expressions(mud, "", ground_mix, "B")
    editing.connect_material_expressions(ground_noise, "", ground_mix, "Alpha")

    final_mix = editing.create_material_expression(material, unreal.MaterialExpressionLinearInterpolate, -80, 100)
    editing.connect_material_expressions(ground_mix, "", final_mix, "A")
    editing.connect_material_expressions(rock, "", final_mix, "B")
    editing.connect_material_expressions(rock_noise, "", final_mix, "Alpha")
    editing.connect_material_property(final_mix, "", unreal.MaterialProperty.MP_BASE_COLOR)
    roughness = _constant(material, 0.92, 160, 360)
    specular = _constant(material, 0.06, 340, 360)
    editing.connect_material_property(roughness, "", unreal.MaterialProperty.MP_ROUGHNESS)
    editing.connect_material_property(specular, "", unreal.MaterialProperty.MP_SPECULAR)
    editing.recompile_material(material)
    unreal.EditorAssetLibrary.set_metadata_tag(material, "LingTu.MaterialLayers", "grass,mud,rock")
    unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False)
    unreal.log(
        "LINGTU_OPEN_FIELD_HF_TERRAIN_MATERIAL_REBUILT layers=grass,mud,rock graph=dual-low-frequency-noise-lerp"
    )
    return material


def _build_materials() -> dict[str, object]:
    return {
        "terrain": _create_terrain_material(),
        "rock": _create_solid_material("M_OpenField_HF_Rock", (0.30, 0.29, 0.25), 0.88),
        "grass": _create_solid_material("M_OpenField_HF_Grass", (0.10, 0.24, 0.055), 0.90),
        "mud": _create_solid_material("M_OpenField_HF_Mud", (0.15, 0.075, 0.035), 0.94),
        "wood": _create_solid_material("M_OpenField_HF_Deadwood", (0.28, 0.13, 0.055), 0.90),
        "robot_shell": _create_solid_material("M_OpenField_Thunder_Shell", (0.08, 0.10, 0.12), 0.52, 0.08),
        "robot_joint": _create_solid_material("M_OpenField_Thunder_Joint", (0.025, 0.035, 0.045), 0.44, 0.25),
        "robot_rubber": _create_solid_material("M_OpenField_Thunder_Rubber", (0.010, 0.013, 0.016), 0.90),
        "robot_sensor": _create_solid_material("M_OpenField_Thunder_Sensor", (0.020, 0.080, 0.140), 0.28, 0.06),
    }


def _clear_current_level(actor_subsystem: object) -> None:
    for actor in actor_subsystem.get_all_level_actors():
        actor_subsystem.destroy_actor(actor)


def _spawn_static_mesh_actor(
    actor_subsystem: object,
    mesh: object,
    label: str,
    transform: object,
    material: object,
    *,
    mobility: object,
    tags: list[str],
    collision: bool = False,
) -> object:
    actor = actor_subsystem.spawn_actor_from_class(
        unreal.StaticMeshActor,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    if actor is None:
        raise RuntimeError(f"failed to spawn StaticMeshActor {label}")
    actor.set_actor_label(label)
    actor.set_actor_transform(transform, False, True)
    actor.set_editor_property("tags", [unreal.Name(tag) for tag in tags])
    component = actor.get_editor_property("static_mesh_component")
    component.set_mobility(mobility)
    component.set_static_mesh(mesh)
    material_slot_count = component.get_num_materials()
    if material_slot_count <= 0:
        raise RuntimeError(f"StaticMesh {mesh.get_path_name()} has no assignable material slots")
    material_path = material.get_path_name()
    for material_index in range(material_slot_count):
        component.set_material(material_index, material)
        assigned_material = component.get_material(material_index)
        if assigned_material is None or assigned_material.get_path_name() != material_path:
            raise RuntimeError(f"failed to assign {material_path} to {mesh.get_path_name()} slot {material_index}")
    component.set_editor_property("cast_shadow", True)
    if not collision:
        _disable_collision(component)
    return actor


def _terrain_height_cm(world: dict[str, object], x_cm: float, y_cm: float) -> float:
    width, height = world["grid"]
    min_x, max_x, min_y, max_y = world["bounds_cm"]
    column = (x_cm - min_x) / (max_x - min_x) * (width - 1)
    row = (y_cm - min_y) / (max_y - min_y) * (height - 1)
    column = max(0.0, min(width - 1.0, column))
    row = max(0.0, min(height - 1.0, row))
    x0 = math.floor(column)
    y0 = math.floor(row)
    x1 = min(width - 1, x0 + 1)
    y1 = min(height - 1, y0 + 1)
    tx = column - x0
    ty = row - y0
    vertices = world["vertices"]
    z00 = vertices[y0 * width + x0][2]
    z10 = vertices[y0 * width + x1][2]
    z01 = vertices[y1 * width + x0][2]
    z11 = vertices[y1 * width + x1][2]
    lower = z00 + (z10 - z00) * tx
    upper = z01 + (z11 - z01) * tx
    return lower + (upper - lower) * ty


def _spawn_terrain(
    actor_subsystem: object,
    mesh: object,
    material: object,
    terrain_digest: str,
) -> object:
    actor = _spawn_static_mesh_actor(
        actor_subsystem,
        mesh,
        "OpenField_HF_Terrain_SameSource",
        unreal.Transform(
            location=unreal.Vector(0.0, 0.0, 0.0),
            rotation=unreal.Rotator(0.0, 0.0, 0.0),
            scale=unreal.Vector(1.0, 1.0, 1.0),
        ),
        material,
        mobility=unreal.ComponentMobility.STATIC,
        tags=["OpenFieldHF", "LingTuSameSourceTerrain", terrain_digest],
        collision=False,
    )
    scale = actor.get_actor_scale3d()
    if not (math.isclose(scale.x, 1.0) and math.isclose(scale.y, 1.0) and math.isclose(scale.z, 1.0)):
        raise RuntimeError("terrain actor was not placed at unit scale")
    return actor


def _body_transform(body: dict) -> object:
    x, y, z = body["location_cm"]
    qx, qy, qz, qw = body["quaternion_xyzw"]
    return unreal.Transform(
        location=unreal.Vector(x, y, z),
        rotation=unreal.Quat(qx, qy, qz, qw).rotator(),
        scale=unreal.Vector(1.0, 1.0, 1.0),
    )


def _link_to_mesh_transform(component: dict) -> object:
    link_to_mesh = component["link_to_mesh"]
    x, y, z = link_to_mesh["location_cm"]
    qx, qy, qz, qw = link_to_mesh["quaternion_xyzw"]
    sx, sy, sz = link_to_mesh["scale"]
    return unreal.Transform(
        location=unreal.Vector(x, y, z),
        rotation=unreal.Quat(qx, qy, qz, qw).rotator(),
        scale=unreal.Vector(sx, sy, sz),
    )


def _spawn_body_actor(actor_subsystem: object, body: dict) -> object:
    actor = actor_subsystem.spawn_actor_from_class(
        unreal.LingTuSimBodyActor,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    if actor is None:
        raise RuntimeError(f"failed to spawn body binding {body['stable_id']}")
    actor.set_actor_label(f"Body_{body['stable_id'].replace('/', '_')}")
    actor.set_actor_transform(_body_transform(body), False, True)
    if not actor.set_body_stable_id(body["stable_id"]):
        raise RuntimeError(f"failed to bind stable body ID {body['stable_id']}")
    actor.set_editor_property(
        "tags",
        [unreal.Name("LingTuBodyBinding"), unreal.Name(body["stable_id"])],
    )
    return actor


def _robot_material(component: dict, materials: dict[str, object]) -> object:
    name = component["asset_name"].casefold()
    if "camera" in name or "lidar" in name:
        return materials["robot_sensor"]
    if "foot" in name:
        return materials["robot_rubber"]
    if "hip" in name or "thigh" in name or "calf" in name:
        return materials["robot_joint"]
    return materials["robot_shell"]


def _spawn_robot(
    actor_subsystem: object,
    recipe: dict,
    materials: dict[str, object],
) -> tuple[int, int]:
    body_actors = {body["stable_id"]: _spawn_body_actor(actor_subsystem, body) for body in recipe["bodies"]}
    visual_count = 0
    for component in recipe["components"]:
        mesh = unreal.load_asset(component["unreal_asset"])
        if mesh is None:
            raise RuntimeError(f"missing ThunderV4 mesh {component['unreal_asset']}")
        local_transform = _link_to_mesh_transform(component)
        actor = _spawn_static_mesh_actor(
            actor_subsystem,
            mesh,
            f"Thunder_{component['asset_name']}",
            local_transform,
            _robot_material(component, materials),
            mobility=unreal.ComponentMobility.MOVABLE,
            tags=["LingTuRobotVisual", component["stable_id"]],
            collision=False,
        )
        body_actor = body_actors.get(component["body_frame_id"])
        if body_actor is None:
            raise RuntimeError(f"no body actor for {component['body_frame_id']}")
        actor.attach_to_actor(
            body_actor,
            "",
            unreal.AttachmentRule.KEEP_RELATIVE,
            unreal.AttachmentRule.KEEP_RELATIVE,
            unreal.AttachmentRule.KEEP_RELATIVE,
            False,
        )
        actor.set_actor_relative_transform(local_transform, False, True)
        if actor.get_attach_parent_actor() != body_actor:
            raise RuntimeError(f"failed to bind visual {component['stable_id']}")
        visual_count += 1
    return len(body_actors), visual_count


def _visual_only_actor(
    actor_subsystem: object,
    mesh: object,
    material: object,
    label: str,
    transform: object,
    prop_kind: str,
) -> object:
    return _spawn_static_mesh_actor(
        actor_subsystem,
        mesh,
        label,
        transform,
        material,
        mobility=unreal.ComponentMobility.STATIC,
        tags=[VISUAL_ONLY_TAG, "OpenFieldHF", prop_kind],
        collision=False,
    )


def _is_clear_of_hero_view(x_cm: float, y_cm: float) -> bool:
    relative_x = x_cm - HERO_CAMERA_X_CM
    relative_y = y_cm - HERO_CAMERA_Y_CM
    if math.hypot(relative_x, relative_y) < HERO_VIEW_NEAR_CLEARANCE_CM:
        return False

    forward_x = -HERO_CAMERA_X_CM
    forward_y = -HERO_CAMERA_Y_CM
    forward_length = math.hypot(forward_x, forward_y)
    along = (relative_x * forward_x + relative_y * forward_y) / forward_length
    lateral = abs(relative_x * forward_y - relative_y * forward_x) / forward_length
    if 0.0 <= along <= HERO_VIEW_FAR_CLEARANCE_CM:
        frustum_half_width = HERO_VIEW_FRUSTUM_MARGIN_CM + along * math.tan(math.radians(HERO_CAMERA_FOV_DEGREES * 0.5))
        if lateral <= frustum_half_width:
            return False
    return True


def _sample_clear_visual_prop_xy(
    rng: random.Random,
    radius_min_cm: float,
    radius_max_cm: float,
) -> tuple[float, float]:
    for _attempt in range(512):
        angle = rng.uniform(0.0, math.tau)
        radius = rng.uniform(radius_min_cm, radius_max_cm)
        x = math.cos(angle) * radius
        y = math.sin(angle) * radius
        if _is_clear_of_hero_view(x, y):
            return x, y
    raise RuntimeError("could not place a VisualOnly prop outside the hero camera clearance corridor")


def _spawn_visual_only_props(
    actor_subsystem: object,
    world: dict[str, object],
    materials: dict[str, object],
) -> dict[str, int]:
    sphere = unreal.load_asset("/Engine/BasicShapes/Sphere.Sphere")
    cylinder = unreal.load_asset("/Engine/BasicShapes/Cylinder.Cylinder")
    if sphere is None or cylinder is None:
        raise RuntimeError("Unreal basic meshes required by OpenField_HF are unavailable")

    rng = random.Random(20260807)  # noqa: S311 - deterministic scene layout
    counts = {"rock": 0, "grass": 0, "mud": 0, "deadwood": 0}

    for index in range(22):
        radius_min, radius_max = (420.0, 1100.0) if index < 9 else (1100.0, 2800.0)
        x, y = _sample_clear_visual_prop_xy(rng, radius_min, radius_max)
        z = _terrain_height_cm(world, x, y)
        scale_xy = rng.uniform(0.28, 1.1)
        scale_z = rng.uniform(0.18, 0.65)
        _visual_only_actor(
            actor_subsystem,
            sphere,
            materials["rock"],
            f"VisualOnly_Rock_{index:03d}",
            unreal.Transform(
                location=unreal.Vector(x, y, z + 38.0 * scale_z),
                rotation=unreal.Rotator(0.0, rng.uniform(0.0, 360.0), 0.0),
                scale=unreal.Vector(
                    scale_xy,
                    scale_xy * rng.uniform(0.65, 1.2),
                    scale_z,
                ),
            ),
            "Rock",
        )
        counts["rock"] += 1

    for index in range(72):
        radius_min, radius_max = (240.0, 1250.0) if index < 40 else (1250.0, 3200.0)
        x, y = _sample_clear_visual_prop_xy(rng, radius_min, radius_max)
        z = _terrain_height_cm(world, x, y)
        grass_scale_z = rng.uniform(0.025, 0.055)
        grass_actor = _visual_only_actor(
            actor_subsystem,
            sphere,
            materials["grass"],
            f"VisualOnly_Grass_{index:03d}",
            unreal.Transform(
                location=unreal.Vector(x, y, z + 50.0 * grass_scale_z),
                rotation=unreal.Rotator(0.0, rng.uniform(0.0, 360.0), 0.0),
                scale=unreal.Vector(
                    rng.uniform(0.10, 0.24),
                    rng.uniform(0.10, 0.24),
                    grass_scale_z,
                ),
            ),
            "Grass",
        )
        grass_actor.get_editor_property("static_mesh_component").set_editor_property("cast_shadow", False)
        counts["grass"] += 1

    mud_patch_scales = ((2.6, 1.2), (1.8, 1.0), (3.0, 1.1), (2.4, 1.4))
    for index, (sx, sy) in enumerate(mud_patch_scales):
        radius_min = 700.0 + index * 350.0
        radius_max = 1500.0 + index * 450.0
        x, y = _sample_clear_visual_prop_xy(rng, radius_min, radius_max)
        z = _terrain_height_cm(world, x, y)
        _visual_only_actor(
            actor_subsystem,
            cylinder,
            materials["mud"],
            f"VisualOnly_MudPatch_{index:03d}",
            unreal.Transform(
                location=unreal.Vector(x, y, z + 0.5),
                rotation=unreal.Rotator(0.0, rng.uniform(0.0, 360.0), 0.0),
                scale=unreal.Vector(sx, sy, 0.018),
            ),
            "Mud",
        )
        counts["mud"] += 1

    for index in range(7):
        radius_min, radius_max = (650.0, 1350.0) if index < 3 else (1350.0, 2700.0)
        x, y = _sample_clear_visual_prop_xy(rng, radius_min, radius_max)
        z = _terrain_height_cm(world, x, y)
        radial_scale = rng.uniform(0.10, 0.18)
        length_scale = rng.uniform(0.9, 2.4)
        _visual_only_actor(
            actor_subsystem,
            cylinder,
            materials["wood"],
            f"VisualOnly_Deadwood_{index:03d}",
            unreal.Transform(
                location=unreal.Vector(x, y, z + 38.0 * radial_scale),
                rotation=unreal.Rotator(
                    90.0 + rng.uniform(-3.0, 3.0),
                    rng.uniform(0.0, 360.0),
                    0.0,
                ),
                scale=unreal.Vector(
                    radial_scale,
                    radial_scale,
                    length_scale,
                ),
            ),
            "Deadwood",
        )
        counts["deadwood"] += 1
    return counts


def _spawn_outdoor_lighting(actor_subsystem: object) -> None:
    sun = actor_subsystem.spawn_actor_from_class(
        unreal.DirectionalLight,
        unreal.Vector(0.0, 0.0, 3000.0),
        unreal.Rotator(-42.0, -28.0, -12.0),
        False,
    )
    sun.set_actor_label("OpenField_Sun")
    sun.light_component.set_editor_property("intensity", 6.5)
    sun.light_component.set_editor_property("light_color", unreal.Color(255, 244, 225, 255))
    _set_editor_property_if_supported(sun.light_component, "atmosphere_sun_light", True)
    _set_editor_property_if_supported(sun.light_component, "cast_cloud_shadows", True)

    skylight = actor_subsystem.spawn_actor_from_class(
        unreal.SkyLight,
        unreal.Vector(0.0, 0.0, 1200.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    skylight.set_actor_label("OpenField_SkyFill")
    skylight.light_component.set_editor_property("intensity", 1.0)
    skylight.light_component.set_editor_property("real_time_capture", True)

    fill_location = unreal.Vector(330.0, -360.0, 300.0)
    fill_target = unreal.Vector(0.0, 0.0, 65.0)
    fill_rotation = unreal.MathLibrary.find_look_at_rotation(fill_location, fill_target)
    robot_fill = actor_subsystem.spawn_actor_from_class(
        unreal.RectLight,
        fill_location,
        fill_rotation,
        False,
    )
    robot_fill.set_actor_label("OpenField_RobotFill")
    robot_fill.light_component.set_editor_property("intensity", 160.0)
    robot_fill.light_component.set_editor_property("light_color", unreal.Color(220, 235, 255, 255))
    _set_editor_property_if_supported(robot_fill.light_component, "source_width", 300.0)
    _set_editor_property_if_supported(robot_fill.light_component, "source_height", 200.0)

    atmosphere = actor_subsystem.spawn_actor_from_class(
        unreal.SkyAtmosphere,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    atmosphere.set_actor_label("OpenField_SkyAtmosphere")

    cloud_class = getattr(unreal, "VolumetricCloud", None)
    if cloud_class is not None:
        cloud = actor_subsystem.spawn_actor_from_class(
            cloud_class,
            unreal.Vector(0.0, 0.0, 0.0),
            unreal.Rotator(0.0, 0.0, 0.0),
            False,
        )
        if cloud is not None:
            cloud.set_actor_label("OpenField_CloudLayer")

    fog = actor_subsystem.spawn_actor_from_class(
        unreal.ExponentialHeightFog,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    fog.set_actor_label("OpenField_AtmosphericPerspective")
    fog_component = fog.get_editor_property("component")
    _set_editor_property_if_supported(fog_component, "fog_density", 0.0024)
    _set_editor_property_if_supported(fog_component, "fog_height_falloff", 0.12)
    _set_editor_property_if_supported(fog_component, "volumetric_fog", True)
    _set_editor_property_if_supported(
        fog_component,
        "fog_inscattering_color",
        unreal.LinearColor(0.56, 0.66, 0.72, 1.0),
    )

    post_process = actor_subsystem.spawn_actor_from_class(
        unreal.PostProcessVolume,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    post_process.set_actor_label("OpenField_ColorPipeline")
    post_process.set_editor_property("unbound", True)
    post_process.set_editor_property("blend_weight", 1.0)
    settings = post_process.get_editor_property("settings")
    settings.set_editor_property("override_auto_exposure_min_brightness", True)
    settings.set_editor_property("override_auto_exposure_max_brightness", True)
    settings.set_editor_property("auto_exposure_min_brightness", 0.90)
    settings.set_editor_property("auto_exposure_max_brightness", 0.90)
    settings.set_editor_property("override_bloom_intensity", True)
    settings.set_editor_property("bloom_intensity", 0.05)
    settings.set_editor_property("override_vignette_intensity", True)
    settings.set_editor_property("vignette_intensity", 0.04)
    post_process.set_editor_property("settings", settings)


def _spawn_camera(actor_subsystem: object, world: dict[str, object]) -> object:
    target = unreal.Vector(0.0, 0.0, HERO_CAMERA_TARGET_Z_CM)
    terrain_z = _terrain_height_cm(world, HERO_CAMERA_X_CM, HERO_CAMERA_Y_CM)
    location = unreal.Vector(HERO_CAMERA_X_CM, HERO_CAMERA_Y_CM, terrain_z + HERO_CAMERA_HEIGHT_CM)
    rotation = unreal.MathLibrary.find_look_at_rotation(location, target)
    camera = actor_subsystem.spawn_actor_from_class(unreal.CameraActor, location, rotation, False)
    camera.set_actor_label("OpenField_HF_HeroCamera")
    camera.camera_component.set_editor_property("field_of_view", HERO_CAMERA_FOV_DEGREES)
    camera.camera_component.set_editor_property("aspect_ratio", 16.0 / 9.0)
    unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(location, rotation)
    return camera


def _write_success_sentinel() -> None:
    if _SUCCESS_PAYLOAD is None:
        raise RuntimeError("OpenField_HF success payload was not initialized")
    dimensions = _png_dimensions(SCREENSHOT_PATH)
    if dimensions != (SCREENSHOT_WIDTH, SCREENSHOT_HEIGHT):
        raise RuntimeError(
            f"unexpected screenshot dimensions: {dimensions}, expected {(SCREENSHOT_WIDTH, SCREENSHOT_HEIGHT)}"
        )
    payload = dict(_SUCCESS_PAYLOAD)
    payload["screenshot"] = {
        "path": str(SCREENSHOT_PATH),
        "width": dimensions[0],
        "height": dimensions[1],
        "sha256": _sha256_file(SCREENSHOT_PATH),
    }
    SUCCESS_SENTINEL.parent.mkdir(parents=True, exist_ok=True)
    SUCCESS_SENTINEL.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _request_screenshot(camera: object) -> None:
    global _SCREENSHOT_TASK, _SCREENSHOT_TICK_HANDLE, _SCREENSHOT_TICK_CALLBACK

    SCREENSHOT_PATH.parent.mkdir(parents=True, exist_ok=True)
    unreal.AutomationLibrary.finish_loading_before_screenshot()
    task = unreal.AutomationLibrary.take_high_res_screenshot(
        SCREENSHOT_WIDTH,
        SCREENSHOT_HEIGHT,
        str(SCREENSHOT_PATH),
        camera=camera,
        mask_enabled=False,
        capture_hdr=False,
        delay=8.0,
    )
    if not task.is_valid_task():
        raise RuntimeError("Unreal rejected the OpenField_HF screenshot task")

    def wait_for_screenshot(_delta_seconds: float) -> None:
        global _SCREENSHOT_TASK, _SCREENSHOT_TICK_HANDLE, _SCREENSHOT_TICK_CALLBACK
        if not task.is_task_done():
            return
        if _SCREENSHOT_TICK_HANDLE is not None:
            unreal.unregister_slate_post_tick_callback(_SCREENSHOT_TICK_HANDLE)
            _SCREENSHOT_TICK_HANDLE = None
        _SCREENSHOT_TASK = None
        _SCREENSHOT_TICK_CALLBACK = None
        try:
            if not SCREENSHOT_PATH.is_file():
                raise RuntimeError(f"screenshot task completed without creating {SCREENSHOT_PATH}")
            _write_success_sentinel()
            unreal.log(f"LINGTU_OPEN_FIELD_HF_READY={SCREENSHOT_PATH}")
        except Exception as error:
            _write_text_sentinel(
                ERROR_SENTINEL,
                f"{type(error).__name__}: {error}\n{traceback.format_exc()}",
            )
            unreal.log_error(f"LINGTU_OPEN_FIELD_HF_ERROR={ERROR_SENTINEL}")
        _quit_editor_if_unattended()

    _SCREENSHOT_TASK = task
    _SCREENSHOT_TICK_CALLBACK = wait_for_screenshot
    _SCREENSHOT_TICK_HANDLE = unreal.register_slate_post_tick_callback(_SCREENSHOT_TICK_CALLBACK)


def build_open_field_hf() -> None:
    """Create the independent editor map and schedule its evidence capture."""

    global _SUCCESS_PAYLOAD

    world = _validate_world_recipe()
    robot_recipe = _validate_robot_recipe()
    terrain_mesh = _import_terrain(world)
    _import_robot_meshes(robot_recipe)
    materials = _build_materials()

    level_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH):
        if not level_subsystem.load_level(MAP_PATH):
            raise RuntimeError(f"could not load OpenField_HF map {MAP_PATH}")
    elif not level_subsystem.new_level(MAP_PATH, False):
        raise RuntimeError(f"could not create OpenField_HF map {MAP_PATH}")
    _clear_current_level(actor_subsystem)

    _spawn_terrain(
        actor_subsystem,
        terrain_mesh,
        materials["terrain"],
        str(world["terrain_digest"]),
    )
    body_count, robot_visual_count = _spawn_robot(actor_subsystem, robot_recipe, materials)
    prop_counts = _spawn_visual_only_props(actor_subsystem, world, materials)
    _spawn_outdoor_lighting(actor_subsystem)
    camera = _spawn_camera(actor_subsystem, world)

    if body_count != EXPECTED_ROBOT_BODY_COUNT:
        raise RuntimeError(f"OpenField_HF created only {body_count} body bindings")
    if not level_subsystem.save_current_level():
        raise RuntimeError(f"could not save OpenField_HF map {MAP_PATH}")

    _SUCCESS_PAYLOAD = {
        "schema": "lingtu.sim.unreal-open-field-hf-evidence.v1",
        "mode": "editor_high_fidelity_increment",
        "map": MAP_PATH,
        "engine_version": unreal.SystemLibrary.get_engine_version(),
        "world_package": "open_field_hf@1.0.0",
        "world_recipe": {
            "path": str(WORLD_RECIPE_PATH),
            "sha256": world["recipe_digest"],
        },
        "same_source_terrain": {
            "source": str(world["terrain_path"]),
            "sha256": world["terrain_digest"],
            "heightfield_source": str(world["heightfield_path"]),
            "heightfield_sha256": world["heightfield_digest"],
            "asset_manifest_sha256": world["asset_manifest_digest"],
            "actor_scale": [1.0, 1.0, 1.0],
            "coordinates": "unreal_lh_z_up_cm",
            "ue58_import_staging": {
                "path": str(world["ue_import_staging_path"]),
                "sha256": world["ue_import_staging_digest"],
                "transformation": "add one vt per vertex and matching v/vt face indices",
                "geometry_changed": False,
            },
        },
        "robot": {
            "instance_id": "thunder_01",
            "body_bindings": body_count,
            "visual_links": robot_visual_count,
        },
        "visual_only_props": {
            "tag": VISUAL_ONLY_TAG,
            "collision": "disabled",
            "hero_view_corridor": "clear",
            "near_camera_clearance_cm": HERO_VIEW_NEAR_CLEARANCE_CM,
            "forward_frustum_clearance_cm": HERO_VIEW_FAR_CLEARANCE_CM,
            "frustum_margin_cm": HERO_VIEW_FRUSTUM_MARGIN_CM,
            "counts": prop_counts,
            "total": sum(prop_counts.values()),
        },
        "hero_camera": {
            "location_xy_cm": [HERO_CAMERA_X_CM, HERO_CAMERA_Y_CM],
            "height_above_terrain_cm": HERO_CAMERA_HEIGHT_CM,
            "target_z_cm": HERO_CAMERA_TARGET_Z_CM,
            "horizontal_fov_degrees": HERO_CAMERA_FOV_DEGREES,
        },
        "lighting": [
            "directional_sun",
            "sky_atmosphere",
            "sky_light",
            "robot_fill",
            "height_fog",
            "post_process",
        ],
        "material_layers": ["grass", "mud", "rock"],
        "terrain_material_graph": "dual-low-frequency-noise-lerp",
        "evidence_scope": {
            "editor_map": True,
            "cook": False,
            "stage": False,
            "package": False,
        },
    }
    _request_screenshot(camera)
    unreal.log(
        "LINGTU_OPEN_FIELD_HF_STAGED "
        f"map={MAP_PATH} bodies={body_count} visuals={robot_visual_count} "
        f"visual_only_props={sum(prop_counts.values())}"
    )


def main() -> None:
    """Run the editor recipe with sentinel-backed error reporting."""

    try:
        build_open_field_hf()
    except Exception as error:
        _write_text_sentinel(
            ERROR_SENTINEL,
            f"{type(error).__name__}: {error}\n{traceback.format_exc()}",
        )
        unreal.log_error(f"LINGTU_OPEN_FIELD_HF_ERROR={ERROR_SENTINEL}")
        _quit_editor_if_unattended()
        raise


if unreal is None:
    if __name__ == "__main__":
        raise RuntimeError("build_open_field_hf.py must run inside Unreal Editor Python")
else:
    main()
