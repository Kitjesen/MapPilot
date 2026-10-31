"""Photographic surface / daylight pass on the existing instanced FactoryPark.

No mesh transforms, collision settings, actor counts or physics packages change.
Existing binary assets are backed up before editing. QA cameras are never saved.
"""

from __future__ import annotations

import importlib.util
import json
import os
import shutil
import sys
import time
import traceback
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT))
from sim.tools.worlds.factory_park_hf.lookdev import SURFACES, surface_for

RUN = Path(os.environ["LINGTU_FACTORY_LOOKDEV_RUN"])
APPLY = os.environ.get("LINGTU_FACTORY_LOOKDEV_APPLY") == "1"
LIGHTING_ONLY = os.environ.get("LINGTU_FACTORY_LOOKDEV_LIGHTING_ONLY") == "1"
PROJECT = Path(__file__).resolve().parents[1]
MAP = "/Game/RobotSim/Generated/FactoryParkHF/Maps/FactoryPark_HF_Instanced_9d314c86ef83ea06"
OLD_MATERIALS = "/Game/RobotSim/Worlds/FactoryParkHF/Materials/IndustrialRealismV2"
ART = "/Game/RobotSim/Worlds/FactoryParkHF/Materials/Photographic"
CAMERAS = [
    ("overview", (14500, 15500, 11500), (0, 0, 0), 52),
    ("entrance", (-800, 6800, 190), (0, 1000, 600), 70),
    ("loading", (2000, 1200, 650), (5000, -1800, 400), 65),
]
LIB = unreal.MaterialEditingLibrary
ASSET = unreal.EditorAssetLibrary
ACTORS = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
LEVELS = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
spec = importlib.util.spec_from_file_location("fp_builder", Path(__file__).with_name("build_factory_park_hf.py"))
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)
report = {"apply": APPLY, "map": MAP, "physics_started": False, "materials": [], "lighting": []}
tick_handle = None


def finish(error=None):
    if tick_handle is not None:
        unreal.unregister_slate_post_tick_callback(tick_handle)
    if error:
        (RUN / "error.txt").write_text(str(error), encoding="utf-8")
        unreal.log_error(str(error))
    else:
        (RUN / "success.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    unreal.SystemLibrary.quit_editor()


def backup(asset_path):
    relative = asset_path.split(".")[0].removeprefix("/Game/")
    for suffix in (".uasset", ".umap", ".uexp", ".ubulk"):
        source = PROJECT / "Content" / (relative + suffix)
        if source.exists():
            target = RUN / "backup" / (relative + suffix)
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists():
                shutil.copy2(source, target)


def geometry_snapshot():
    records = []
    for actor in ACTORS.get_all_level_actors():
        for component in actor.get_components_by_class(unreal.StaticMeshComponent):
            transform = component.get_world_transform()
            position, rotation, scale = transform.translation, transform.rotation, transform.scale3d
            values = tuple(
                round(value, 5)
                for value in (
                    position.x,
                    position.y,
                    position.z,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                    scale.x,
                    scale.y,
                    scale.z,
                )
            )
            records.append(
                (
                    actor.get_actor_label(),
                    component.get_name(),
                    values,
                    str(component.static_mesh.get_path_name()) if component.static_mesh else "",
                    str(component.get_collision_enabled()),
                    component.get_instance_count() if isinstance(component, unreal.InstancedStaticMeshComponent) else 0,
                )
            )
    return sorted(records)


def imported_textures():
    result = {}
    sources = json.loads((SURFACES / "sources.json").read_text())
    for item in sources["textures"]:
        name = "T_" + item["asset"] + "_" + item["channel"]
        task = unreal.AssetImportTask()
        for key, value in {
            "filename": str(SURFACES / item["file"]),
            "destination_path": ART + "/Textures",
            "destination_name": name,
            "automated": True,
            "replace_existing": True,
            "save": True,
        }.items():
            task.set_editor_property(key, value)
        unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
        texture = unreal.load_asset(ART + "/Textures/" + name)
        if texture is None:
            raise RuntimeError(f"Texture import failed: {name}")
        texture.set_editor_property("srgb", item["channel"] == "base")
        texture.set_editor_property(
            "compression_settings",
            unreal.TextureCompressionSettings.TC_NORMALMAP
            if item["channel"] == "normal"
            else unreal.TextureCompressionSettings.TC_DEFAULT,
        )
        ASSET.save_loaded_asset(texture)
        result.setdefault(item["asset"], {})[item["channel"]] = texture
    return result


def master_material(textures):
    material = builder._create_or_load_asset(ART, "M_PhotographicWorld", unreal.Material, unreal.MaterialFactoryNew())
    LIB.delete_all_material_expressions(material)

    def node(cls):
        return LIB.create_material_expression(material, cls)

    def connect(a, output, b, input_):
        if not LIB.connect_material_expressions(a, output, b, input_):
            raise RuntimeError(f"Material connection failed: {input_}")

    tile = node(unreal.MaterialExpressionVectorParameter)
    tile.set_editor_property("parameter_name", "SurfaceSizeCm")
    tile.set_editor_property("default_value", unreal.LinearColor(400, 400, 400, 1))
    tint = node(unreal.MaterialExpressionVectorParameter)
    tint.set_editor_property("parameter_name", "SurfaceTint")
    tint.set_editor_property("default_value", unreal.LinearColor(1, 1, 1, 1))
    outputs = {}
    for channel in ("base", "rough", "normal"):
        tex = node(unreal.MaterialExpressionTextureObjectParameter)
        tex.set_editor_property("parameter_name", "Scan_" + channel)
        tex.set_editor_property("texture", textures["asphalt_02"][channel])
        sampler = {
            "base": unreal.MaterialSamplerType.SAMPLERTYPE_COLOR,
            "rough": unreal.MaterialSamplerType.SAMPLERTYPE_LINEAR_COLOR,
            "normal": unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL,
        }[channel]
        tex.set_editor_property("sampler_type", sampler)
        aligned = node(unreal.MaterialExpressionMaterialFunctionCall)
        fn = "WorldAlignedNormal" if channel == "normal" else "WorldAlignedTexture"
        builder._set_material_function_checked(
            aligned, f"/Engine/Functions/Engine_MaterialFunctions01/Texturing/{fn}.{fn}"
        )
        connect(tex, "", aligned, "TextureObject")
        connect(tile, "", aligned, "TextureSize")
        outputs[channel] = aligned
    desaturate = node(unreal.MaterialExpressionDesaturation)
    desaturation = node(unreal.MaterialExpressionScalarParameter)
    desaturation.set_editor_property("parameter_name", "SurfaceDesaturation")
    desaturation.set_editor_property("default_value", 0.0)
    connect(outputs["base"], "XYZ Texture", desaturate, "")
    connect(desaturation, "", desaturate, "Fraction")
    color = node(unreal.MaterialExpressionMultiply)
    connect(desaturate, "", color, "A")
    connect(tint, "", color, "B")
    rough = node(unreal.MaterialExpressionComponentMask)
    rough.set_editor_property("r", True)
    connect(outputs["rough"], "XYZ Texture", rough, "")
    for expression, output, property_ in (
        (color, "", unreal.MaterialProperty.MP_BASE_COLOR),
        (rough, "", unreal.MaterialProperty.MP_ROUGHNESS),
        (outputs["normal"], "XYZ Texture", unreal.MaterialProperty.MP_NORMAL),
    ):
        if not LIB.connect_material_property(expression, output, property_):
            raise RuntimeError(f"Material output failed: {property_}")
    builder._configure_master_material_usages(material, LIB)
    errors = list(LIB.recompile_material(material))
    if errors:
        raise RuntimeError(str(errors))
    ASSET.save_loaded_asset(material)
    return material


def apply_materials():
    textures = imported_textures()
    master = master_material(textures)
    layout = json.loads((ROOT / "sim/packages/worlds/factory_park_hf/generated/expanded-layout.json").read_text())
    identities = {(item["material"], item["semantic_class"]) for item in layout["objects"]}
    identities.add(("gravel", "terrain"))
    used_paths = set()
    for actor in ACTORS.get_all_level_actors():
        for component in actor.get_components_by_class(unreal.StaticMeshComponent):
            for index in range(component.get_num_materials()):
                current = component.get_material(index)
                if current:
                    used_paths.add(current.get_path_name().split(".")[0])
    for source, semantic in sorted(identities):
        choice = surface_for(source, semantic)
        if not choice:
            continue
        path = OLD_MATERIALS + "/" + builder._material_instance_name(source, semantic)
        # Terrain-baked semantic records need not have a separate render material.
        if path not in used_paths:
            continue
        instance = unreal.load_asset(path)
        if not isinstance(instance, unreal.MaterialInstanceConstant):
            raise RuntimeError(f"Missing existing surface: {path}")
        asset, size, tint = choice
        backup(path)
        LIB.set_material_instance_parent(instance, master)
        for channel in ("base", "rough", "normal"):
            LIB.set_material_instance_texture_parameter_value(instance, "Scan_" + channel, textures[asset][channel])
            if (
                LIB.get_material_instance_texture_parameter_value(instance, "Scan_" + channel)
                != textures[asset][channel]
            ):
                raise RuntimeError(f"Texture parameter did not bind: {path}/{channel}")
        LIB.set_material_instance_vector_parameter_value(
            instance, "SurfaceSizeCm", unreal.LinearColor(size, size, size, 1)
        )
        LIB.set_material_instance_vector_parameter_value(instance, "SurfaceTint", unreal.LinearColor(*tint, 1))
        LIB.set_material_instance_scalar_parameter_value(
            instance, "SurfaceDesaturation", 1.0 if asset == "box_profile_metal_sheet" else 0.0
        )
        LIB.update_material_instance(instance)
        ASSET.save_loaded_asset(instance)
        report["materials"].append(
            {"path": path, "source": source, "semantic": semantic, "scan": asset, "tile_cm": size}
        )
    if not report["materials"]:
        raise RuntimeError("No existing material was upgraded")


def apply_lighting():
    for actor in ACTORS.get_all_level_actors():
        if isinstance(actor, unreal.DirectionalLight):
            component = actor.light_component
            component.set_editor_property("intensity", 85000.0)
            component.set_editor_property("light_color", unreal.Color(255, 255, 255, 255))
            component.set_editor_property("use_temperature", True)
            component.set_editor_property("temperature", 6000.0)
            component.set_editor_property("light_source_angle", 2.5)
            actor.set_actor_rotation(unreal.Rotator(-38, -38, 0), False)
            report["lighting"].append({"actor": actor.get_actor_label(), "sun_lux": 85000})
        elif isinstance(actor, unreal.SkyLight):
            actor.light_component.set_editor_property("intensity", 3.0)
            actor.light_component.set_editor_property("real_time_capture", True)
            actor.light_component.recapture_sky()
            report["lighting"].append({"actor": actor.get_actor_label(), "sky_intensity": 3.0})
        elif isinstance(actor, unreal.PostProcessVolume):
            builder._configure_manual_exposure(actor, {"mode": "manual", "ev100": 12.5, "white_balance_k": 6500})
            settings = actor.get_editor_property("settings")
            for key, value in {
                "override_dynamic_global_illumination_method": True,
                "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
                "override_reflection_method": True,
                "reflection_method": unreal.ReflectionMethod.LUMEN,
                "override_bloom_intensity": True,
                "bloom_intensity": 0.15,
                "override_vignette_intensity": True,
                "vignette_intensity": 0.10,
            }.items():
                settings.set_editor_property(key, value)
            actor.set_editor_property("settings", settings)


def main():
    global tick_handle
    unreal.EditorPythonScripting.set_keep_python_script_alive(True)
    if not LEVELS.load_level(MAP):
        raise RuntimeError(f"Cannot load {MAP}")
    before = geometry_snapshot()
    if APPLY:
        backup(MAP)
        if not LIGHTING_ONLY:
            apply_materials()
        apply_lighting()
        after = geometry_snapshot()
        if after != before:
            (RUN / "geometry-difference.json").write_text(json.dumps({"before": before, "after": after}))
            raise RuntimeError("Art pass unexpectedly changed geometry/collision")
        if not LEVELS.save_current_level():
            raise RuntimeError("Map save failed")
        if not LEVELS.load_level(MAP) or geometry_snapshot() != before:
            raise RuntimeError("Saved map geometry/collision changed")
    report["geometry_component_count"] = len(before)
    report["geometry_unchanged"] = True
    persisted = {}
    for actor in ACTORS.get_all_level_actors():
        for component in actor.get_components_by_class(unreal.StaticMeshComponent):
            for index in range(component.get_num_materials()):
                instance = component.get_material(index)
                if not isinstance(instance, unreal.MaterialInstanceConstant):
                    continue
                parent = instance.get_editor_property("parent")
                if not parent or not parent.get_path_name().startswith(ART):
                    continue
                channels = {}
                for channel in ("base", "rough", "normal"):
                    texture = LIB.get_material_instance_texture_parameter_value(instance, "Scan_" + channel)
                    if texture is None:
                        raise RuntimeError(f"Saved scan binding missing: {instance.get_name()}/{channel}")
                    channels[channel] = texture.get_path_name()
                persisted[instance.get_path_name()] = channels
    report["persisted_materials"] = persisted
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    for command in (
        "sg.AntiAliasingQuality 3",
        "r.AntiAliasingMethod 4",
        "sg.GlobalIlluminationQuality 3",
        "sg.ReflectionQuality 3",
        "sg.ShadowQuality 3",
        "sg.PostProcessQuality 3",
        "sg.TextureQuality 3",
        "r.ScreenPercentage 100",
        "t.MaxFPS 30",
    ):
        unreal.SystemLibrary.execute_console_command(world, command)
    cameras = []
    for name, location, target, fov in CAMERAS:
        loc = unreal.Vector(*location)
        rotation = unreal.MathLibrary.find_look_at_rotation(loc, unreal.Vector(*target))
        camera = ACTORS.spawn_actor_from_class(unreal.CameraActor, loc, rotation)
        camera.camera_component.set_editor_property("field_of_view", fov)
        cameras.append((name, camera))
    state = {"index": 0, "task": None, "framed": False, "ready": time.monotonic() + 12, "started": time.monotonic()}

    def tick(_delta):
        try:
            if time.monotonic() - state["started"] > 240:
                raise TimeoutError("Factory lookdev screenshot timeout")
            if time.monotonic() < state["ready"]:
                return
            if state["task"] is not None:
                if not state["task"].is_task_done():
                    return
                output = RUN / (cameras[state["index"]][0] + ".png")
                if not output.is_file():
                    raise RuntimeError(f"Screenshot missing: {output}")
                state["index"] += 1
                state["task"] = None
                state["framed"] = False
                state["ready"] = time.monotonic() + 4
                return
            if state["index"] == len(cameras):
                finish()
                return
            name, camera = cameras[state["index"]]
            if not state["framed"]:
                unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(
                    camera.get_actor_location(), camera.get_actor_rotation()
                )
                state["framed"] = True
                state["ready"] = time.monotonic() + 8
                return
            task = unreal.AutomationLibrary.take_high_res_screenshot(
                1920,
                1080,
                str(RUN / (name + ".png")),
                camera,
                False,
                False,
                unreal.ComparisonTolerance.LOW,
                name,
                2.0,
                True,
            )
            if task is None or not task.is_valid_task():
                raise RuntimeError("Screenshot task invalid")
            state["task"] = task
        except Exception:
            finish(traceback.format_exc())

    tick_handle = unreal.register_slate_post_tick_callback(tick)


try:
    main()
except Exception:
    finish(traceback.format_exc())
