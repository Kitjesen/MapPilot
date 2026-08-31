"""Build and capture the ThunderV4 offline preview inside Unreal Editor."""

from __future__ import annotations

import json
import os
import traceback
from pathlib import Path

try:
    import unreal
except ModuleNotFoundError:
    unreal = None


PROJECT_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
FBX_DIR = Path(
    os.environ.get(
        "LINGTU_THUNDER_FBX_DIR",
        REPO_ROOT / "build" / "unreal-assets" / "thunderv4-mjcf-fbx",
    )
).resolve()
RECIPE_PATH = Path(
    os.environ.get(
        "LINGTU_THUNDER_PREVIEW_RECIPE",
        REPO_ROOT / "build" / "unreal-assets" / "thunderv4-runtime.recipe.json",
    )
).resolve()
SCREENSHOT_PATH = Path(
    os.environ.get(
        "LINGTU_THUNDER_SCREENSHOT",
        REPO_ROOT / "build" / "unreal-preview" / "thunderv4-runtime.png",
    )
).resolve()
SUCCESS_SENTINEL = Path(
    os.environ.get(
        "LINGTU_THUNDER_PREVIEW_SUCCESS",
        REPO_ROOT / "build" / "unreal-preview" / "thunderv4-runtime.success.json",
    )
).resolve()
ERROR_SENTINEL = Path(
    os.environ.get(
        "LINGTU_THUNDER_PREVIEW_ERROR",
        REPO_ROOT / "build" / "unreal-preview" / "thunderv4-runtime.error.txt",
    )
).resolve()
UNATTENDED = os.environ.get("LINGTU_THUNDER_PREVIEW_UNATTENDED") == "1"
REBUILD_MATERIALS = os.environ.get("LINGTU_THUNDER_REBUILD_MATERIALS") == "1"
_SCREENSHOT_TASK = None
_SCREENSHOT_TICK_HANDLE = None
_SCREENSHOT_TICK_CALLBACK = None

MESH_DESTINATION = "/Game/RobotSim/Robots/ThunderV4/Meshes"
MATERIAL_DESTINATION = "/Game/RobotSim/Materials"
MAP_PATH = "/Game/RobotSim/Maps/ThunderV4_RuntimePreview"


def _write_text_sentinel(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _write_success_sentinel() -> None:
    SUCCESS_SENTINEL.parent.mkdir(parents=True, exist_ok=True)
    SUCCESS_SENTINEL.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.unreal-offline-preview-success.v1",
                "mode": "offline_preview",
                "screenshot": str(SCREENSHOT_PATH),
                "map": MAP_PATH,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )


def _quit_editor_if_unattended() -> None:
    if UNATTENDED:
        unreal.log("LINGTU_PREVIEW_UNATTENDED_EXIT=1")
        unreal.SystemLibrary.quit_editor()


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _import_meshes(recipe: dict) -> None:
    expected = {component["asset_name"] for component in recipe["components"]}
    tasks = []
    for asset_name in sorted(expected, key=str.casefold):
        source = FBX_DIR / f"{asset_name}.fbx"
        if not source.is_file():
            raise RuntimeError(f"missing staged FBX: {source}")
        task = unreal.AssetImportTask()
        task.set_editor_property("filename", str(source))
        task.set_editor_property("destination_path", MESH_DESTINATION)
        task.set_editor_property("automated", True)
        task.set_editor_property("replace_existing", True)
        task.set_editor_property("replace_existing_settings", True)
        task.set_editor_property("save", True)
        task.set_editor_property("async_", False)
        tasks.append(task)

    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks(tasks)
    missing = [
        component["unreal_asset"]
        for component in recipe["components"]
        if unreal.load_asset(component["unreal_asset"]) is None
    ]
    if missing:
        raise RuntimeError(f"Unreal import did not create expected StaticMesh assets: {missing}")


def _material(
    name: str,
    color: tuple[float, float, float],
    metallic: float,
    roughness: float,
) -> unreal.Material:
    asset_path = f"{MATERIAL_DESTINATION}/{name}"
    material = unreal.load_asset(asset_path)
    if material is not None and not REBUILD_MATERIALS:
        return material
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
    base_color = editing.create_material_expression(
        material, unreal.MaterialExpressionConstant3Vector, -420, -80
    )
    base_color.set_editor_property("constant", unreal.LinearColor(*color, 1.0))
    metallic_node = editing.create_material_expression(
        material, unreal.MaterialExpressionConstant, -420, 80
    )
    metallic_node.set_editor_property("r", metallic)
    roughness_node = editing.create_material_expression(
        material, unreal.MaterialExpressionConstant, -420, 180
    )
    roughness_node.set_editor_property("r", roughness)
    editing.connect_material_property(
        base_color, "", unreal.MaterialProperty.MP_BASE_COLOR
    )
    editing.connect_material_property(
        metallic_node, "", unreal.MaterialProperty.MP_METALLIC
    )
    editing.connect_material_property(
        roughness_node, "", unreal.MaterialProperty.MP_ROUGHNESS
    )
    editing.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False)
    return material


def _build_materials() -> dict[str, unreal.Material]:
    shell = _material("M_Thunder_Shell", (0.12, 0.15, 0.16), 0.16, 0.56)
    rubber = _material("M_Thunder_Rubber", (0.006, 0.007, 0.008), 0.0, 0.86)
    sensor = _material("M_Thunder_Sensor", (0.018, 0.035, 0.046), 0.08, 0.18)
    ground = _material("M_TestPad_Ground", (0.07, 0.08, 0.08), 0.0, 0.78)
    joint_alloy = _material("M_TestPad_Plinth", (0.10, 0.12, 0.13), 0.72, 0.40)
    return {
        "body_shell": shell,
        "joint_alloy": joint_alloy,
        "limb_alloy": joint_alloy,
        "rubber": rubber,
        "sensor": sensor,
        "ground": ground,
        "plinth": joint_alloy,
    }


def _link_to_mesh_transform(component: dict) -> unreal.Transform:
    link_to_mesh = component["link_to_mesh"]
    x, y, z = link_to_mesh["location_cm"]
    qx, qy, qz, qw = link_to_mesh["quaternion_xyzw"]
    sx, sy, sz = link_to_mesh["scale"]
    return unreal.Transform(
        location=unreal.Vector(x, y, z),
        rotation=unreal.Quat(qx, qy, qz, qw).rotator(),
        scale=unreal.Vector(sx, sy, sz),
    )


def _body_transform(body: dict) -> unreal.Transform:
    x, y, z = body["location_cm"]
    qx, qy, qz, qw = body["quaternion_xyzw"]
    return unreal.Transform(
        location=unreal.Vector(x, y, z),
        rotation=unreal.Quat(qx, qy, qz, qw).rotator(),
        scale=unreal.Vector(1.0, 1.0, 1.0),
    )


def _spawn_body_actor(
    actor_subsystem: unreal.EditorActorSubsystem,
    body: dict,
) -> unreal.Actor:
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


def _spawn_static_mesh_actor(
    actor_subsystem: unreal.EditorActorSubsystem,
    mesh: unreal.StaticMesh,
    label: str,
    transform: unreal.Transform,
    material: unreal.Material,
) -> unreal.StaticMeshActor:
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
    mesh_component = actor.get_editor_property("static_mesh_component")
    mesh_component.set_mobility(unreal.ComponentMobility.MOVABLE)
    mesh_component.set_static_mesh(mesh)
    mesh_component.set_material(0, material)
    mesh_component.set_editor_property("cast_shadow", True)
    return actor


def _robot_material_key(component: dict) -> str:
    name = component["asset_name"].casefold()
    if "camera" in name or "lidar" in name:
        return "sensor"
    if "foot" in name or component.get("material_key") == "wheel_black":
        return "rubber"
    if "hip" in name:
        return "joint_alloy"
    if "thigh" in name or "calf" in name:
        return "limb_alloy"
    return "body_shell"


def _robot_material(component: dict, materials: dict[str, unreal.Material]) -> unreal.Material:
    return materials[_robot_material_key(component)]


def preview_post_process_profile() -> dict[str, float]:
    """Return deterministic exposure controls that preserve dark robot materials."""

    return {
        "auto_exposure_min_brightness": 0.58,
        "auto_exposure_max_brightness": 0.58,
        "auto_exposure_bias": -2.0,
        "bloom_intensity": 0.05,
    }


def _save_material_assets(materials: dict[str, unreal.Material]) -> None:
    saved_paths: set[str] = set()
    for material in materials.values():
        asset_path = str(material.get_path_name())
        if asset_path in saved_paths:
            continue
        if not unreal.EditorAssetLibrary.save_loaded_asset(
            material,
            only_if_is_dirty=False,
        ):
            raise RuntimeError(f"could not persist preview material {asset_path}")
        saved_paths.add(asset_path)


def _set_editor_property_if_supported(target: object, property_name: str, value: object) -> None:
    try:
        target.set_editor_property(property_name, value)
    except Exception:
        unreal.log_warning(f"Preview property unavailable: {property_name}")


def _clear_current_level(actor_subsystem: unreal.EditorActorSubsystem) -> None:
    for actor in actor_subsystem.get_all_level_actors():
        actor_subsystem.destroy_actor(actor)


def _spawn_environment(
    actor_subsystem: unreal.EditorActorSubsystem,
    materials: dict[str, unreal.Material],
) -> None:
    plane = unreal.load_asset("/Engine/BasicShapes/Plane.Plane")
    cube = unreal.load_asset("/Engine/BasicShapes/Cube.Cube")
    if plane is None or cube is None:
        raise RuntimeError("Unreal Engine basic shape assets are unavailable")

    _spawn_static_mesh_actor(
        actor_subsystem,
        plane,
        "TestPad_Ground",
        unreal.Transform(
            location=unreal.Vector(0.0, 0.0, 0.0),
            rotation=unreal.Rotator(0.0, 0.0, 0.0),
            scale=unreal.Vector(24.0, 24.0, 24.0),
        ),
        materials["ground"],
    )
    _spawn_static_mesh_actor(
        actor_subsystem,
        cube,
        "Thunder_Display_Plinth",
        unreal.Transform(
            location=unreal.Vector(0.0, 0.0, 4.0),
            rotation=unreal.Rotator(0.0, 0.0, 0.0),
            scale=unreal.Vector(1.72, 1.18, 0.08),
        ),
        materials["plinth"],
    )

    directional = actor_subsystem.spawn_actor_from_class(
        unreal.DirectionalLight,
        unreal.Vector(0.0, 0.0, 300.0),
        unreal.Rotator(-38.0, -32.0, -8.0),
        False,
    )
    directional.set_actor_label("Sun_Key_Neutral")
    directional.light_component.set_editor_property("intensity", 0.8)
    directional.light_component.set_editor_property(
        "light_color", unreal.Color(255, 242, 224, 255)
    )
    _set_editor_property_if_supported(directional.light_component, "atmosphere_sun_light", True)

    sky = actor_subsystem.spawn_actor_from_class(
        unreal.SkyLight,
        unreal.Vector(0.0, 0.0, 200.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    sky.set_actor_label("Sky_Fill")
    sky.light_component.set_editor_property("intensity", 0.38)
    sky.light_component.set_editor_property("real_time_capture", True)

    atmosphere = actor_subsystem.spawn_actor_from_class(
        unreal.SkyAtmosphere,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    atmosphere.set_actor_label("Sky_Atmosphere")

    fog = actor_subsystem.spawn_actor_from_class(
        unreal.ExponentialHeightFog,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    fog.set_actor_label("Preview_Neutral_Horizon")
    fog_component = fog.get_editor_property("component")
    _set_editor_property_if_supported(fog_component, "fog_density", 0.008)
    _set_editor_property_if_supported(fog_component, "fog_height_falloff", 0.18)
    _set_editor_property_if_supported(
        fog_component,
        "fog_inscattering_color",
        unreal.LinearColor(0.50, 0.55, 0.58, 1.0),
    )
    _set_editor_property_if_supported(
        fog_component,
        "inscattering_color",
        unreal.LinearColor(0.50, 0.55, 0.58, 1.0),
    )

    target = unreal.Vector(0.0, 0.0, 55.0)
    for label, location, color, intensity in (
        ("Studio_Key_Softbox", unreal.Vector(165.0, -170.0, 210.0), unreal.Color(255, 236, 214, 255), 240.0),
        ("Studio_Rim_Cool", unreal.Vector(-140.0, 110.0, 160.0), unreal.Color(192, 216, 255, 255), 150.0),
        ("Studio_Fill_Broad", unreal.Vector(35.0, 185.0, 150.0), unreal.Color(220, 230, 238, 255), 80.0),
    ):
        rotation = unreal.MathLibrary.find_look_at_rotation(location, target)
        light = actor_subsystem.spawn_actor_from_class(
            unreal.RectLight, location, rotation, False
        )
        light.set_actor_label(label)
        light.light_component.set_editor_property("intensity", intensity)
        light.light_component.set_editor_property("light_color", color)
        light.light_component.set_editor_property("source_width", 210.0)
        light.light_component.set_editor_property("source_height", 160.0)

    post_process = actor_subsystem.spawn_actor_from_class(
        unreal.PostProcessVolume,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    post_process.set_actor_label("Preview_Exposure")
    post_process.set_editor_property("unbound", True)
    post_process.set_editor_property("blend_weight", 1.0)
    settings = post_process.get_editor_property("settings")
    profile = preview_post_process_profile()
    settings.set_editor_property("override_auto_exposure_min_brightness", True)
    settings.set_editor_property("override_auto_exposure_max_brightness", True)
    settings.set_editor_property("override_auto_exposure_bias", True)
    settings.set_editor_property("override_bloom_intensity", True)
    for property_name, value in profile.items():
        settings.set_editor_property(property_name, value)
    settings.set_editor_property("override_vignette_intensity", True)
    settings.set_editor_property("vignette_intensity", 0.0)
    post_process.set_editor_property("settings", settings)


def _recipe_bounds(recipe: dict) -> tuple[unreal.Vector, float]:
    locations = [component["location_cm"] for component in recipe["components"]]
    min_x = min(location[0] for location in locations)
    max_x = max(location[0] for location in locations)
    min_y = min(location[1] for location in locations)
    max_y = max(location[1] for location in locations)
    min_z = min(location[2] for location in locations)
    max_z = max(location[2] for location in locations)
    center = unreal.Vector(
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
    )
    radius = max(max_x - min_x, max_y - min_y, max_z - min_z) * 0.5
    return center, max(radius, 60.0)


def _spawn_camera(actor_subsystem: unreal.EditorActorSubsystem, recipe: dict) -> unreal.CameraActor:
    center, radius = _recipe_bounds(recipe)
    target = unreal.Vector(center.x, center.y, center.z + 4.0)
    location = unreal.Vector(
        center.x + radius * 3.35,
        center.y - radius * 4.10,
        center.z + radius * 1.55,
    )
    rotation = unreal.MathLibrary.find_look_at_rotation(location, target)
    camera = actor_subsystem.spawn_actor_from_class(
        unreal.CameraActor, location, rotation, False
    )
    camera.set_actor_label("Thunder_Hero_Camera")
    camera.camera_component.set_editor_property("field_of_view", 46.0)
    camera.camera_component.set_editor_property("aspect_ratio", 16.0 / 9.0)
    unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(
        location, rotation
    )
    return camera


def _request_screenshot(camera: unreal.CameraActor) -> None:
    global _SCREENSHOT_TASK, _SCREENSHOT_TICK_HANDLE, _SCREENSHOT_TICK_CALLBACK

    SCREENSHOT_PATH.parent.mkdir(parents=True, exist_ok=True)
    unreal.AutomationLibrary.finish_loading_before_screenshot()
    task = unreal.AutomationLibrary.take_high_res_screenshot(
        1920,
        1080,
        str(SCREENSHOT_PATH),
        camera=camera,
        mask_enabled=False,
        capture_hdr=False,
        delay=3.0,
    )
    if not task.is_valid_task():
        raise RuntimeError("Unreal rejected the high-resolution screenshot task")

    def wait_for_screenshot(_delta_seconds: float) -> None:
        global _SCREENSHOT_TASK, _SCREENSHOT_TICK_HANDLE, _SCREENSHOT_TICK_CALLBACK
        if not task.is_task_done():
            return

        if _SCREENSHOT_TICK_HANDLE is not None:
            unreal.unregister_slate_post_tick_callback(_SCREENSHOT_TICK_HANDLE)
            _SCREENSHOT_TICK_HANDLE = None
        _SCREENSHOT_TASK = None
        _SCREENSHOT_TICK_CALLBACK = None

        if SCREENSHOT_PATH.is_file():
            _write_success_sentinel()
            unreal.log(f"LINGTU_PREVIEW_READY={SCREENSHOT_PATH}")
        else:
            error = f"Unreal screenshot task completed without creating {SCREENSHOT_PATH}"
            _write_text_sentinel(ERROR_SENTINEL, error + "\n")
            unreal.log_error(f"LINGTU_PREVIEW_ERROR={ERROR_SENTINEL}")
        _quit_editor_if_unattended()

    _SCREENSHOT_TASK = task
    _SCREENSHOT_TICK_CALLBACK = wait_for_screenshot
    _SCREENSHOT_TICK_HANDLE = unreal.register_slate_post_tick_callback(
        _SCREENSHOT_TICK_CALLBACK
    )


def build_preview() -> None:
    recipe = _load_json(RECIPE_PATH)
    if recipe.get("schema") != "lingtu.sim.unreal-preview-recipe.v1":
        raise RuntimeError(f"unsupported preview recipe: {RECIPE_PATH}")
    if len(recipe.get("components", ())) != len(
        {component["stable_id"] for component in recipe.get("components", ())}
    ):
        raise RuntimeError("preview recipe contains duplicate stable IDs")
    local_transforms = {
        component["stable_id"]: _link_to_mesh_transform(component)
        for component in recipe["components"]
    }

    _import_meshes(recipe)
    materials = _build_materials()
    level_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH):
        if not level_subsystem.load_level(MAP_PATH):
            raise RuntimeError(f"could not load preview map {MAP_PATH}")
    elif not level_subsystem.new_level(MAP_PATH, False):
        raise RuntimeError(f"could not create preview map {MAP_PATH}")
    _clear_current_level(actor_subsystem)
    _spawn_environment(actor_subsystem, materials)

    if len(recipe.get("bodies", ())) != len(
        {body["stable_id"] for body in recipe.get("bodies", ())}
    ):
        raise RuntimeError("preview recipe contains duplicate body stable IDs")
    body_actors = {
        body["stable_id"]: _spawn_body_actor(actor_subsystem, body)
        for body in recipe.get("bodies", ())
    }

    for component in recipe["components"]:
        mesh = unreal.load_asset(component["unreal_asset"])
        local_transform = local_transforms[component["stable_id"]]
        actor = _spawn_static_mesh_actor(
            actor_subsystem,
            mesh,
            f"Thunder_{component['asset_name']}",
            local_transform,
            _robot_material(component, materials),
        )
        actor.set_editor_property(
            "tags",
            [unreal.Name("LingTuPreview"), unreal.Name(component["stable_id"])],
        )
        body_actor = body_actors.get(component["body_frame_id"])
        if body_actor is None:
            raise RuntimeError(
                f"missing body binding for visual {component['stable_id']}"
            )
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
            raise RuntimeError(
                f"failed to attach visual {component['stable_id']} "
                f"to {component['body_frame_id']}"
            )

    camera = _spawn_camera(actor_subsystem, recipe)
    if not level_subsystem.save_current_level():
        raise RuntimeError(f"could not save preview map {MAP_PATH}")
    _save_material_assets(materials)
    _request_screenshot(camera)
    unreal.log(
        "LingTu Thunder offline preview staged: "
        f"bodies={len(body_actors)}, components={len(recipe['components'])}, map={MAP_PATH}"
    )


def main() -> None:
    try:
        build_preview()
    except Exception as error:
        _write_text_sentinel(
            ERROR_SENTINEL,
            f"{type(error).__name__}: {error}\n{traceback.format_exc()}",
        )
        unreal.log_error(f"LINGTU_PREVIEW_ERROR={ERROR_SENTINEL}")
        _quit_editor_if_unattended()
        raise


if unreal is None:
    if __name__ == "__main__":
        raise RuntimeError("build_thunderv4_preview.py must run inside Unreal Editor Python")
else:
    main()
