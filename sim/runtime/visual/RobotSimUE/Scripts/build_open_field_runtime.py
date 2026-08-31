"""Build the robot-free OpenFieldRuntime Unreal editor map."""

from __future__ import annotations

import hashlib
import json
import math
import os
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
        "LINGTU_OPEN_FIELD_RUNTIME_RECIPE",
        REPO_ROOT / "sim" / "packages" / "worlds" / "open_field_hf" / "visual" / "ue_import.recipe.json",
    )
).resolve()
EVIDENCE_PATH = Path(
    os.environ.get(
        "LINGTU_OPEN_FIELD_RUNTIME_EVIDENCE",
        PROJECT_DIR / "Saved" / "Automation" / "OpenFieldRuntime.evidence.json",
    )
).resolve()
ERROR_SENTINEL = Path(
    os.environ.get(
        "LINGTU_OPEN_FIELD_RUNTIME_ERROR",
        PROJECT_DIR / "Saved" / "Automation" / "OpenFieldRuntime.error.txt",
    )
).resolve()
UNATTENDED = os.environ.get("LINGTU_OPEN_FIELD_RUNTIME_UNATTENDED") == "1"

MAP_PATH = "/Game/RobotSim/Maps/OpenFieldRuntime"
SOURCE_LEVEL_PATH = "/Game/RobotSim/Maps/OpenField_HF"
WORLD_ASSET_ROOT = "/Game/RobotSim/Worlds/OpenFieldHF"
TERRAIN_MESH_PATH = f"{WORLD_ASSET_ROOT}/Meshes/SM_OpenField_HF_Terrain.SM_OpenField_HF_Terrain"
TERRAIN_MATERIAL_PATH = f"{WORLD_ASSET_ROOT}/Materials/M_OpenField_HF_Terrain.M_OpenField_HF_Terrain"
VISUAL_ONLY_TAG = "VisualOnly"
RUNTIME_TAG = "OpenFieldRuntime"
TERRAIN_LABEL = "OpenFieldRuntime_Terrain_SameSource"
SUN_LABEL = "OpenFieldRuntime_Sun"
SKYLIGHT_LABEL = "OpenFieldRuntime_SkyLight"
ATMOSPHERE_LABEL = "OpenFieldRuntime_SkyAtmosphere"
FOG_LABEL = "OpenFieldRuntime_AtmosphericFog"
POST_PROCESS_LABEL = "OpenFieldRuntime_ColorPipeline"
CAMERA_LABEL = "OpenFieldRuntime_SessionCamera"
CAMERA_LOCATION_CM = (420.0, -520.0, 245.0)
CAMERA_TARGET_CM = (0.0, 0.0, 72.0)
CAMERA_FOV_DEGREES = 42.0
EXPECTED_GRID = (253, 253)
EXPECTED_EXTENT_M = (160.0, 160.0)
PREPLACED_ROBOT_BINDINGS = 0
FORBIDDEN_BINDING_TAGS = {"LingTuBodyBinding"}


def _canonical_json(value: object) -> str:
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
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> dict[str, object]:
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


def _load_terrain_bounds_cm(terrain_path: Path, expected_grid: tuple[int, int]) -> tuple[float, float, float, float]:
    vertex_count = 0
    triangle_count = 0
    coordinates_declared = False
    min_x = min_y = math.inf
    max_x = max_y = -math.inf
    with terrain_path.open("r", encoding="ascii") as stream:
        for line in stream:
            if line.startswith("# coordinates=unreal_lh_z_up_cm"):
                coordinates_declared = True
            elif line.startswith("v "):
                parts = line.split()
                if len(parts) != 4:
                    raise RuntimeError(f"malformed OBJ vertex in {terrain_path}")
                x, y, _z = (float(value) for value in parts[1:4])
                min_x = min(min_x, x)
                max_x = max(max_x, x)
                min_y = min(min_y, y)
                max_y = max(max_y, y)
                vertex_count += 1
            elif line.startswith("f "):
                triangle_count += 1
    expected_vertices = expected_grid[0] * expected_grid[1]
    expected_triangles = 2 * (expected_grid[0] - 1) * (expected_grid[1] - 1)
    if not coordinates_declared:
        raise RuntimeError("terrain OBJ does not declare unreal_lh_z_up_cm coordinates")
    if vertex_count != expected_vertices or triangle_count != expected_triangles:
        raise RuntimeError(
            "terrain OBJ topology disagrees with the source grid: "
            f"vertices={vertex_count}/{expected_vertices}, triangles={triangle_count}/{expected_triangles}"
        )
    if not (
        math.isclose(min_x, -8000.0, abs_tol=0.01)
        and math.isclose(max_x, 8000.0, abs_tol=0.01)
        and math.isclose(min_y, -8000.0, abs_tol=0.01)
        and math.isclose(max_y, 8000.0, abs_tol=0.01)
    ):
        raise RuntimeError("terrain OBJ is not a 160 m Unreal-centimetre mesh")
    return (min_x, max_x, min_y, max_y)


def _validate_world_recipe() -> dict[str, object]:
    recipe = _load_json(WORLD_RECIPE_PATH)
    if recipe.get("schema") != "lingtu.sim.unreal-world-import-recipe.v1":
        raise RuntimeError(f"unsupported OpenFieldHF recipe: {WORLD_RECIPE_PATH}")
    if recipe.get("world_package") != "open_field_hf@1.0.0":
        raise RuntimeError("world recipe does not resolve open_field_hf@1.0.0")
    if recipe.get("binding") != "WorldVisual:OpenFieldHF":
        raise RuntimeError("world recipe binding is not WorldVisual:OpenFieldHF")

    mesh_import = recipe.get("blender_mesh_import")
    if not isinstance(mesh_import, dict):
        raise RuntimeError("world recipe has no blender_mesh_import contract")
    if mesh_import.get("coordinates") != "unreal_lh_z_up_cm":
        raise RuntimeError("terrain mesh is not declared in Unreal centimetres")
    if mesh_import.get("import_scale") != 1.0:
        raise RuntimeError("OpenFieldHF terrain import scale must be exactly 1.0")
    if mesh_import.get("vertex_per_height_sample") is not True:
        raise RuntimeError("terrain mesh must preserve one vertex per height sample")

    coordinate_contract = recipe.get("coordinate_contract")
    if not isinstance(coordinate_contract, dict):
        raise RuntimeError("world recipe has no coordinate contract")
    if coordinate_contract.get("grid_px") != list(EXPECTED_GRID):
        raise RuntimeError(f"unexpected OpenFieldHF grid: {coordinate_contract.get('grid_px')}")
    if coordinate_contract.get("extent_m") != list(EXPECTED_EXTENT_M):
        raise RuntimeError(f"unexpected OpenFieldHF extent: {coordinate_contract.get('extent_m')}")

    sources = recipe.get("sources")
    if not isinstance(sources, dict):
        raise RuntimeError("world recipe has no sources object")
    for source_name in ("asset_manifest", "heightfield_png", "terrain_obj"):
        if source_name not in sources:
            raise RuntimeError(f"world recipe is missing source: {source_name}")
    resolved_sources = {name: _validate_source_record(name, record) for name, record in sources.items()}
    terrain_path = resolved_sources["terrain_obj"]
    if terrain_path != _resolve_repo_source(mesh_import.get("source")):
        raise RuntimeError("terrain OBJ source disagrees with blender_mesh_import.source")
    bounds_cm = _load_terrain_bounds_cm(terrain_path, EXPECTED_GRID)
    return {
        "recipe": recipe,
        "recipe_digest": _sha256_file(WORLD_RECIPE_PATH),
        "terrain_path": terrain_path,
        "terrain_digest": sources["terrain_obj"]["sha256"],
        "heightfield_path": resolved_sources["heightfield_png"],
        "heightfield_digest": sources["heightfield_png"]["sha256"],
        "bounds_cm": bounds_cm,
    }


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _quit_editor_if_unattended() -> None:
    if UNATTENDED:
        unreal.log("LINGTU_OPEN_FIELD_RUNTIME_UNATTENDED_EXIT=1")
        unreal.SystemLibrary.quit_editor()


def _set_editor_property_if_supported(target: object, property_name: str, value: object) -> None:
    try:
        target.set_editor_property(property_name, value)
    except Exception as error:
        unreal.log_warning(f"OpenFieldRuntime property unavailable: {property_name} ({error})")


def _disable_collision(mesh_component: object) -> None:
    try:
        mesh_component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    except Exception:
        _set_editor_property_if_supported(mesh_component, "collision_profile_name", "NoCollision")


def _load_same_source_terrain_assets(world: dict[str, object]) -> tuple[object, object]:
    mesh = unreal.load_asset(TERRAIN_MESH_PATH)
    if mesh is None or not isinstance(mesh, unreal.StaticMesh):
        raise RuntimeError(f"required same-source terrain mesh is missing: {TERRAIN_MESH_PATH}")
    source_digest = str(unreal.EditorAssetLibrary.get_metadata_tag(mesh, "LingTu.SourceSha256") or "")
    import_scale = str(unreal.EditorAssetLibrary.get_metadata_tag(mesh, "LingTu.ImportScale") or "")
    if source_digest != str(world["terrain_digest"]) or import_scale != "1.0":
        raise RuntimeError(
            "same-source terrain mesh metadata mismatch: "
            f"source_sha256={source_digest!r}, import_scale={import_scale!r}"
        )

    material = unreal.load_asset(TERRAIN_MATERIAL_PATH)
    if material is None or not isinstance(material, unreal.Material):
        raise RuntimeError(f"required same-source terrain material is missing: {TERRAIN_MATERIAL_PATH}")
    layers = str(unreal.EditorAssetLibrary.get_metadata_tag(material, "LingTu.MaterialLayers") or "")
    if layers != "grass,mud,rock":
        raise RuntimeError(f"same-source terrain material layers are missing: {TERRAIN_MATERIAL_PATH}")
    return mesh, material


def _validate_terrain_mesh_bounds(mesh: object) -> None:
    bounding_box = mesh.get_bounding_box()
    size_x = bounding_box.max.x - bounding_box.min.x
    size_y = bounding_box.max.y - bounding_box.min.y
    if not (
        math.isclose(size_x, 16000.0, rel_tol=0.0, abs_tol=1.0)
        and math.isclose(size_y, 16000.0, rel_tol=0.0, abs_tol=1.0)
    ):
        raise RuntimeError(
            f"terrain mesh violates the 1:1 centimetre contract: bounds=({size_x:.3f}, {size_y:.3f}) cm"
        )


def _clear_current_level(actor_subsystem: object) -> None:
    for actor in actor_subsystem.get_all_level_actors():
        actor_subsystem.destroy_actor(actor)


def _spawn_terrain(actor_subsystem: object, mesh: object, material: object, terrain_digest: str) -> object:
    actor = actor_subsystem.spawn_actor_from_class(
        unreal.StaticMeshActor,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    if actor is None:
        raise RuntimeError(f"failed to spawn {TERRAIN_LABEL}")
    actor.set_actor_label(TERRAIN_LABEL)
    actor.set_editor_property(
        "tags",
        [unreal.Name(tag) for tag in (RUNTIME_TAG, VISUAL_ONLY_TAG, "LingTuSameSourceTerrain", terrain_digest)],
    )
    actor.set_actor_transform(
        unreal.Transform(
            location=unreal.Vector(0.0, 0.0, 0.0),
            rotation=unreal.Rotator(0.0, 0.0, 0.0),
            scale=unreal.Vector(1.0, 1.0, 1.0),
        ),
        False,
        True,
    )
    component = actor.get_editor_property("static_mesh_component")
    component.set_mobility(unreal.ComponentMobility.STATIC)
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
    _disable_collision(component)
    component.set_editor_property("cast_shadow", True)
    scale = actor.get_actor_scale3d()
    if not (math.isclose(scale.x, 1.0) and math.isclose(scale.y, 1.0) and math.isclose(scale.z, 1.0)):
        raise RuntimeError("terrain actor was not placed at unit scale")
    return actor


def _tag_actor(actor: object, label: str, *extra_tags: str) -> None:
    actor.set_actor_label(label)
    actor.set_editor_property("tags", [unreal.Name(tag) for tag in (RUNTIME_TAG, *extra_tags)])


def _spawn_outdoor_lighting(actor_subsystem: object) -> list[str]:
    sun = actor_subsystem.spawn_actor_from_class(
        unreal.DirectionalLight,
        unreal.Vector(0.0, 0.0, 3000.0),
        unreal.Rotator(-42.0, -28.0, -12.0),
        False,
    )
    _tag_actor(sun, SUN_LABEL, "OutdoorLighting")
    sun.light_component.set_editor_property("intensity", 6.2)
    sun.light_component.set_editor_property("light_color", unreal.Color(255, 244, 226, 255))
    _set_editor_property_if_supported(sun.light_component, "atmosphere_sun_light", True)

    skylight = actor_subsystem.spawn_actor_from_class(
        unreal.SkyLight,
        unreal.Vector(0.0, 0.0, 1200.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    _tag_actor(skylight, SKYLIGHT_LABEL, "OutdoorLighting")
    skylight.light_component.set_editor_property("intensity", 1.05)
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
    _set_editor_property_if_supported(fog_component, "fog_density", 0.0022)
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
    _tag_actor(post_process, POST_PROCESS_LABEL, "OutdoorLighting")
    post_process.set_editor_property("unbound", True)
    post_process.set_editor_property("blend_weight", 1.0)
    settings = post_process.get_editor_property("settings")
    settings.set_editor_property("override_auto_exposure_min_brightness", True)
    settings.set_editor_property("override_auto_exposure_max_brightness", True)
    settings.set_editor_property("auto_exposure_min_brightness", 0.95)
    settings.set_editor_property("auto_exposure_max_brightness", 0.95)
    settings.set_editor_property("override_bloom_intensity", True)
    settings.set_editor_property("bloom_intensity", 0.04)
    post_process.set_editor_property("settings", settings)
    return [SUN_LABEL, SKYLIGHT_LABEL, ATMOSPHERE_LABEL, FOG_LABEL, POST_PROCESS_LABEL]


def _spawn_camera(actor_subsystem: object) -> object:
    location = unreal.Vector(*CAMERA_LOCATION_CM)
    target = unreal.Vector(*CAMERA_TARGET_CM)
    rotation = unreal.MathLibrary.find_look_at_rotation(location, target)
    camera = actor_subsystem.spawn_actor_from_class(unreal.CameraActor, location, rotation, False)
    if camera is None:
        raise RuntimeError(f"failed to spawn {CAMERA_LABEL}")
    _tag_actor(camera, CAMERA_LABEL, "SessionCamera")
    camera.camera_component.set_editor_property("field_of_view", CAMERA_FOV_DEGREES)
    camera.camera_component.set_editor_property("aspect_ratio", 16.0 / 9.0)
    camera.set_editor_property("auto_activate_for_player", unreal.AutoReceiveInput.PLAYER0)
    unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(location, rotation)
    return camera


def _count_preplaced_robot_bindings(actor_subsystem: object) -> int:
    count = 0
    for actor in actor_subsystem.get_all_level_actors():
        tags = {str(tag) for tag in actor.get_editor_property("tags")}
        if tags.intersection(FORBIDDEN_BINDING_TAGS):
            count += 1
    return count


def _write_evidence(payload: dict[str, object]) -> None:
    _write_text(EVIDENCE_PATH, _canonical_json(payload))


def build_open_field_runtime() -> None:
    """Create and save the robot-free runtime showcase map."""

    world = _validate_world_recipe()
    terrain_mesh, terrain_material = _load_same_source_terrain_assets(world)
    _validate_terrain_mesh_bounds(terrain_mesh)

    level_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH):
        if not level_subsystem.load_level(MAP_PATH):
            raise RuntimeError(f"could not load OpenFieldRuntime map {MAP_PATH}")
    elif not level_subsystem.new_level(MAP_PATH, False):
        raise RuntimeError(f"could not create OpenFieldRuntime map {MAP_PATH}")
    _clear_current_level(actor_subsystem)

    terrain_actor = _spawn_terrain(actor_subsystem, terrain_mesh, terrain_material, str(world["terrain_digest"]))
    lighting_labels = _spawn_outdoor_lighting(actor_subsystem)
    camera = _spawn_camera(actor_subsystem)
    preplaced_robot_bindings = _count_preplaced_robot_bindings(actor_subsystem)
    if preplaced_robot_bindings != PREPLACED_ROBOT_BINDINGS:
        raise RuntimeError(f"OpenFieldRuntime must remain robot-free, found {preplaced_robot_bindings} bindings")
    if not level_subsystem.save_current_level():
        raise RuntimeError(f"could not save OpenFieldRuntime map {MAP_PATH}")

    _write_evidence(
        {
            "schema": "lingtu.sim.unreal-open-field-runtime-evidence.v1",
            "map_path": MAP_PATH,
            "source_level_path": SOURCE_LEVEL_PATH,
            "engine_version": unreal.SystemLibrary.get_engine_version(),
            "terrain_asset": TERRAIN_MESH_PATH,
            "terrain_material": TERRAIN_MATERIAL_PATH,
            "terrain_actor": terrain_actor.get_actor_label(),
            "terrain_collision": "disabled_visual_only",
            "same_source_terrain": {
                "source": str(world["terrain_path"]),
                "sha256": world["terrain_digest"],
                "heightfield_source": str(world["heightfield_path"]),
                "heightfield_sha256": world["heightfield_digest"],
                "recipe": str(WORLD_RECIPE_PATH),
                "recipe_sha256": world["recipe_digest"],
            },
            "camera": {
                "label": camera.get_actor_label(),
                "location_cm": list(CAMERA_LOCATION_CM),
                "target_cm": list(CAMERA_TARGET_CM),
                "field_of_view_degrees": CAMERA_FOV_DEGREES,
                "auto_activate_for_player": 0,
            },
            "lighting": lighting_labels,
            "preplaced_robot_bindings": preplaced_robot_bindings,
            "robot_actor_count": 0,
            "robot_mesh_references": 0,
            "mujoco_motion_authority": True,
        }
    )
    unreal.log(
        "LINGTU_OPEN_FIELD_RUNTIME_READY "
        f"map={MAP_PATH} terrain={TERRAIN_MESH_PATH} evidence={EVIDENCE_PATH} "
        f"preplaced_robot_bindings={preplaced_robot_bindings}"
    )


def main() -> None:
    """Run the editor recipe with evidence-backed error reporting."""

    try:
        build_open_field_runtime()
    except Exception as error:
        _write_text(ERROR_SENTINEL, f"{type(error).__name__}: {error}\n{traceback.format_exc()}")
        unreal.log_error(f"LINGTU_OPEN_FIELD_RUNTIME_ERROR={ERROR_SENTINEL}")
        _quit_editor_if_unattended()
        raise


if unreal is None:
    if __name__ == "__main__":
        raise RuntimeError("build_open_field_runtime.py must run inside Unreal Editor Python")
else:
    main()
