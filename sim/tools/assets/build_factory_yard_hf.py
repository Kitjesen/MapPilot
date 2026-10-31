"""Build a reviewable high-fidelity factory-yard scene from local CC0 assets."""
from __future__ import annotations

import json
import math
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "build" / "factory-yard-hf-20261018"
SRC = ROOT / "sim" / "library" / "sources" / "polyhaven"
IND = ROOT / "sim" / "library" / "industrial"


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for datablocks in (bpy.data.meshes, bpy.data.curves, bpy.data.materials, bpy.data.cameras, bpy.data.lights):
        for block in list(datablocks):
            if block.users == 0:
                datablocks.remove(block)


def import_gltf(path: Path, name: str, location, scale=1.0):
    before = set(bpy.context.scene.objects)
    bpy.ops.import_scene.gltf(filepath=str(path))
    created = [obj for obj in bpy.context.scene.objects if obj not in before]
    root = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(root)
    for obj in created:
        if obj.parent is None:
            obj.parent = root
    root.location = location
    root.scale = (scale, scale, scale)
    return root, created


def hide_modular_kit_spacers(created):
    """Keep the facade's wall/door/window modules and hide loose kit spacers.

    Poly Haven's modular facade is delivered as a layout of reusable parts, not
    as one assembled building. The cornice/crown/dado/base pieces are useful
    source modules but read as floating construction debris when the whole kit
    is imported into a preview. They stay in the source library and remain
    available for a later authored facade assembly.
    """
    hidden_prefixes = ("cornice", "crown", "dado", "base")
    for obj in created:
        if obj.name.lower().startswith(hidden_prefixes):
            obj.hide_render = True


def make_pbr_material(name: str, diff: Path, normal: Path, rough: Path):
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    bsdf = nodes.get("Principled BSDF")
    texcoord = nodes.new("ShaderNodeTexCoord")
    mapping = nodes.new("ShaderNodeMapping")
    links.new(texcoord.outputs["Generated"], mapping.inputs["Vector"])
    albedo = nodes.new("ShaderNodeTexImage")
    albedo.image = bpy.data.images.load(str(diff), check_existing=True)
    links.new(mapping.outputs["Vector"], albedo.inputs["Vector"])
    links.new(albedo.outputs["Color"], bsdf.inputs["Base Color"])
    roughness = nodes.new("ShaderNodeTexImage")
    roughness.image = bpy.data.images.load(str(rough), check_existing=True)
    roughness.image.colorspace_settings.name = "Non-Color"
    links.new(mapping.outputs["Vector"], roughness.inputs["Vector"])
    links.new(roughness.outputs["Color"], bsdf.inputs["Roughness"])
    normal_tex = nodes.new("ShaderNodeTexImage")
    normal_tex.image = bpy.data.images.load(str(normal), check_existing=True)
    normal_tex.image.colorspace_settings.name = "Non-Color"
    bump = nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.32
    bump.inputs["Distance"].default_value = 0.08
    links.new(mapping.outputs["Vector"], normal_tex.inputs["Vector"])
    links.new(normal_tex.outputs["Color"], bump.inputs["Height"])
    links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    return material


def make_floor():
    floor = make_pbr_material(
        "Factory Yard / Worn Concrete",
        SRC / "textures" / "concrete_floor_worn_001" / "2k" / "concrete_floor_worn_001_diff_2k.jpg",
        SRC / "textures" / "concrete_floor_worn_001" / "2k" / "concrete_floor_worn_001_nor_gl_2k.jpg",
        SRC / "textures" / "concrete_floor_worn_001" / "2k" / "concrete_floor_worn_001_rough_2k.jpg",
    )
    bpy.ops.mesh.primitive_plane_add(size=2, location=(0, 2, 0))
    plane = bpy.context.object
    plane.name = "world/factory_yard/concrete_ground"
    plane.scale = (35, 28, 1)
    plane.data.materials.append(floor)
    return plane


def make_marking(name, location, scale, color=(0.95, 0.62, 0.08, 1)):
    mat = bpy.data.materials.new(name + " material")
    mat.diffuse_color = color
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = color
    bsdf.inputs["Roughness"].default_value = 0.48
    bpy.ops.mesh.primitive_cube_add(location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = scale
    obj.data.materials.append(mat)
    return obj


def look_at(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    clear_scene()
    make_floor()

    imported = []
    facade_root, facade_parts = import_gltf(
        SRC / "modular_factory_facade" / "2k" / "modular_factory_facade_2k.gltf",
        "world/factory_facade",
        (0, 19, 0),
        1.0,
    )
    hide_modular_kit_spacers(facade_parts)
    imported.append(facade_root)
    imported.append(import_gltf(SRC / "modular_chainlink_fence" / "2k" / "modular_chainlink_fence_2k.gltf", "world/perimeter_fence", (-17, 10, 0), 1.0)[0])
    imported.append(import_gltf(SRC / "concrete_road_barrier" / "2k" / "concrete_road_barrier_2k.gltf", "world/concrete_barrier_a", (-9, -7, 0), 1.0)[0])
    imported.append(import_gltf(SRC / "concrete_road_barrier" / "2k" / "concrete_road_barrier_2k.gltf", "world/concrete_barrier_b", (-5, -7, 0), 1.0)[0])
    imported.append(import_gltf(SRC / "caged_hanging_light" / "2k" / "caged_hanging_light_2k.gltf", "world/hanging_light", (2, 12, 8), 1.2)[0])
    imported.append(import_gltf(IND / "ph_storage_cart" / "1.0.0" / "model.glb", "prop/storage_cart", (5, -2, 0.02), 1.0)[0])
    imported.append(import_gltf(IND / "ph_barrel" / "1.0.0" / "model.glb", "prop/storage_barrel", (8, -5, 0.02), 1.0)[0])
    imported.append(import_gltf(IND / "ph_industrial_pipes" / "1.0.0" / "model.glb", "prop/pipe_rack", (11, 6, 0.2), 1.0)[0])
    imported.append(import_gltf(IND / "boom_barrier" / "1.0.1" / "model.glb", "prop/yard_barrier", (0, -11, 0), 1.0)[0])
    imported.append(import_gltf(IND / "service_door" / "1.0.1" / "model.glb", "prop/service_door", (-12, 9, 0), 1.0)[0])
    imported.append(import_gltf(IND / "box_truck" / "2.0.0" / "model.glb", "prop/box_truck", (-8, 1, 0), 1.0)[0])
    imported.append(import_gltf(SRC / "plastic_container" / "2k" / "plastic_container_2k.gltf", "prop/plastic_container", (5, 6, 0.02), 1.35)[0])
    imported.append(import_gltf(SRC / "plastic_crate_02" / "2k" / "plastic_crate_02_2k.gltf", "prop/plastic_crate_a", (7, 5.7, 0.02), 1.6)[0])
    imported.append(import_gltf(SRC / "plastic_crate_02" / "2k" / "plastic_crate_02_2k.gltf", "prop/plastic_crate_b", (7.8, 5.7, 0.02), 1.6)[0])

    for i, x in enumerate(range(-4, 11, 3)):
        make_marking(f"world/road_marking/{i:02d}", (x, -3.5, 0.012), (0.95, 0.055, 0.006))
    make_marking("world/loading_bay_line", (0, 12.2, 0.015), (8, 0.04, 0.006), (0.92, 0.92, 0.92, 1))

    hdri_path = SRC / "hdris" / "factory_yard" / "4k" / "factory_yard_4k.exr"
    world = bpy.data.worlds.new("Factory Yard HDRI")
    bpy.context.scene.world = world
    world.use_nodes = True
    world_nodes = world.node_tree.nodes
    world_links = world.node_tree.links
    env = world_nodes.new("ShaderNodeTexEnvironment")
    env.image = bpy.data.images.load(str(hdri_path), check_existing=True)
    env.image.colorspace_settings.name = "Non-Color"
    bg = world_nodes.get("Background")
    bg.inputs["Strength"].default_value = 0.35
    world_links.new(env.outputs["Color"], bg.inputs["Color"])

    bpy.ops.object.light_add(type="SUN", location=(0, -4, 12))
    sun = bpy.context.object
    sun.name = "lighting/sun_key"
    sun.data.energy = 3.0
    sun.rotation_euler = (math.radians(28), math.radians(-18), math.radians(-32))
    bpy.ops.object.light_add(type="AREA", location=(2, -3, 8))
    area = bpy.context.object
    area.name = "lighting/fill_area"
    area.data.energy = 800
    area.data.shape = "RECTANGLE"
    area.data.size = 8
    look_at(area, (0, 4, 1))

    bpy.ops.object.camera_add(location=(25, -29, 15))
    camera = bpy.context.object
    camera.name = "camera/factory_yard_hero"
    camera.data.lens = 48
    look_at(camera, (5, 6, 2.8))
    bpy.context.scene.camera = camera

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 900
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(OUT / "factory-yard-hero.png")
    scene.render.film_transparent = False
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.render.image_settings.color_mode = "RGBA"
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT / "factory-yard-hf.blend"))
    bpy.ops.render.render(write_still=True)

    manifest = {
        "schema": "lingtu.scene-preview.v1",
        "id": "factory_yard_hf",
        "version": "0.1.1",
        "purpose": "high-fidelity environment preview; not UE/MuJoCo qualification",
        "authority": {"physics": "mujoco", "rendering": "blender_preview", "ue": "not_bound"},
        "assets": [obj.name for obj in imported],
        "materials": {"ground": "Poly Haven concrete_floor_worn_001 2K", "lighting": "Poly Haven factory_yard 4K HDRI"},
        "files": {"blend": "factory-yard-hf.blend", "preview": "factory-yard-hero.png"},
        "limitations": [
            "visual composition only; no UE import or MuJoCo world binding",
            "new industrial props remain quarantined until mesh/body mapping and collision checks",
            "asset dimensions are source values and not field-calibrated",
            "modular_factory_facade is treated as a source kit; loose spacer modules are hidden in this preview",
        ],
    }
    (OUT / "scene.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": "completed", "preview": str(OUT / "factory-yard-hero.png"), "assets": len(imported)}))


if __name__ == "__main__":
    main()
