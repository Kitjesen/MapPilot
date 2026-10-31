"""Build a coherent mountain inspection outpost preview from licensed assets."""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.tools.assets.gltf_preview_variant import stage_mesh_variant
from sim.tools.assets.mountain_outpost_geometry import background_mesh, mountain_grid, trail_weights

SRC = ROOT / "sim" / "library" / "sources" / "polyhaven"
TRAIL = [(-29, -21), (-18, -13), (-7, -8), (2, -2), (9, 7), (14, 20)]


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for datablocks in (
        bpy.data.meshes,
        bpy.data.curves,
        bpy.data.materials,
        bpy.data.cameras,
        bpy.data.lights,
    ):
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


def clone_hierarchy(source, name: str, location, scale=1.0, rotation_z=0.0):
    def clone(obj, parent=None):
        copied = obj.copy()
        if obj.data is not None:
            copied.data = obj.data
        copied.animation_data_clear()
        bpy.context.collection.objects.link(copied)
        copied.parent = parent
        for child in obj.children:
            clone(child, copied)
        return copied

    root = clone(source)
    root.name = name
    root.location = location
    root.scale = (scale, scale, scale)
    root.rotation_euler[2] = rotation_z
    return root


def pbr_material(name: str, diffuse: Path, normal: Path, roughness: Path):
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    bsdf = nodes.get("Principled BSDF")
    texcoord = nodes.new("ShaderNodeTexCoord")
    mapping = nodes.new("ShaderNodeMapping")
    mapping.inputs["Scale"].default_value = (0.25, 0.25, 0.25)
    links.new(texcoord.outputs["UV"], mapping.inputs["Vector"])

    albedo = nodes.new("ShaderNodeTexImage")
    albedo.image = bpy.data.images.load(str(diffuse), check_existing=True)
    links.new(mapping.outputs["Vector"], albedo.inputs["Vector"])
    links.new(albedo.outputs["Color"], bsdf.inputs["Base Color"])

    rough = nodes.new("ShaderNodeTexImage")
    rough.image = bpy.data.images.load(str(roughness), check_existing=True)
    rough.image.colorspace_settings.name = "Non-Color"
    links.new(mapping.outputs["Vector"], rough.inputs["Vector"])
    links.new(rough.outputs["Color"], bsdf.inputs["Roughness"])

    normal_tex = nodes.new("ShaderNodeTexImage")
    normal_tex.image = bpy.data.images.load(str(normal), check_existing=True)
    normal_tex.image.colorspace_settings.name = "Non-Color"
    normal_map = nodes.new("ShaderNodeNormalMap")
    normal_map.inputs["Strength"].default_value = 0.7
    links.new(mapping.outputs["Vector"], normal_tex.inputs["Vector"])
    links.new(normal_tex.outputs["Color"], normal_map.inputs["Color"])
    links.new(normal_map.outputs["Normal"], bsdf.inputs["Normal"])
    return material


def simple_material(name: str, color, roughness=0.7, metallic=0.0):
    material = bpy.data.materials.new(name)
    material.diffuse_color = (*color, 1.0)
    material.use_nodes = True
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    return material


def make_path(terrain):
    """Blend a trail into the real ground, never create an elevated ribbon."""
    material = terrain.data.materials[0]
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    bsdf = nodes.get("Principled BSDF")
    attribute = terrain.data.color_attributes.new(name="trail_weight", type="FLOAT_COLOR", domain="POINT")
    weights = trail_weights([tuple(vertex.co) for vertex in terrain.data.vertices], TRAIL)
    for value, weight in zip(attribute.data, weights):
        value.color = (weight, weight, weight, 1.0)
    mask = nodes.new("ShaderNodeVertexColor")
    mask.layer_name = "trail_weight"
    texcoord = nodes.new("ShaderNodeTexCoord")
    noise = nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 3.0
    noise.inputs["Detail"].default_value = 4.0
    noise.inputs["Roughness"].default_value = 0.78
    ramp = nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (0.10, 0.105, 0.095, 1.0)
    ramp.color_ramp.elements[1].color = (0.27, 0.26, 0.23, 1.0)
    color = nodes.new("ShaderNodeMixRGB")
    links.new(bsdf.inputs["Base Color"].links[0].from_socket, color.inputs[1])
    links.new(ramp.outputs["Color"], color.inputs[2])
    links.new(mask.outputs["Color"], color.inputs[0])
    links.new(color.outputs["Color"], bsdf.inputs["Base Color"])
    rough = nodes.new("ShaderNodeMixRGB")
    links.new(bsdf.inputs["Roughness"].links[0].from_socket, rough.inputs[1])
    rough.inputs[2].default_value = (0.94, 0.94, 0.94, 1.0)
    links.new(mask.outputs["Color"], rough.inputs[0])
    links.new(rough.outputs["Color"], bsdf.inputs["Roughness"])
    links.new(texcoord.outputs["Object"], noise.inputs["Vector"])
    links.new(noise.outputs["Fac"], ramp.inputs["Fac"])
    return weights


def make_terrain(surface):
    vertices, faces = surface.mesh_data()
    mesh = bpy.data.meshes.new("mountain_ground")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    terrain = bpy.data.objects.new("world/mountain_outpost/terrain", mesh)
    bpy.context.collection.objects.link(terrain)
    uv = mesh.uv_layers.new(name="ground_meters")
    for loop in mesh.loops:
        vertex = mesh.vertices[loop.vertex_index].co
        uv.data[loop.index].uv = (vertex.x, vertex.y)
    terrain.data.materials.append(
        pbr_material(
            "Mountain / Rocky Terrain 02",
            SRC / "textures" / "rocky_terrain_02" / "2k" / "rocky_terrain_02_diff_2k.jpg",
            SRC / "textures" / "rocky_terrain_02" / "2k" / "rocky_terrain_02_nor_gl_2k.jpg",
            SRC / "textures" / "rocky_terrain_02" / "2k" / "rocky_terrain_02_rough_2k.jpg",
        )
    )
    return terrain


def ground_hierarchy(root, surface, *, embed_m=0.04, height_m=None):
    bpy.context.view_layer.update()
    meshes = [obj for obj in (root, *root.children_recursive) if obj.type == "MESH"]
    if height_m is not None:
        bounds = [(obj.matrix_world @ Vector(corner)).z for obj in meshes for corner in obj.bound_box]
        root.scale *= height_m / (max(bounds) - min(bounds))
        bpy.context.view_layer.update()
    bottom = min((obj.matrix_world @ Vector(corner)).z for obj in meshes for corner in obj.bound_box)
    ground = surface.height_at(root.location.x, root.location.y)
    root.location.z += ground - bottom - embed_m
    bpy.context.view_layer.update()
    observed_bottom = min((obj.matrix_world @ Vector(corner)).z for obj in meshes for corner in obj.bound_box)
    return {
        "asset": root.name,
        "ground_z_m": ground,
        "bottom_z_m": observed_bottom,
        "embed_m": embed_m,
        "contact_error_m": abs(observed_bottom + embed_m - ground),
    }


def make_distant_ridges(surface):
    vertices, faces = background_mesh(surface)
    mesh = bpy.data.meshes.new("mountain_surroundings")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new("background/mountain_surroundings", mesh)
    bpy.context.collection.objects.link(obj)
    uv = mesh.uv_layers.new(name="ground_meters")
    for loop in mesh.loops:
        vertex = mesh.vertices[loop.vertex_index].co
        uv.data[loop.index].uv = (vertex.x, vertex.y)
    for polygon in mesh.polygons:
        polygon.use_smooth = True
    mesh.materials.append(
        pbr_material(
            "Mountain / surrounding ground",
            SRC / "textures" / "rocky_terrain_02" / "2k" / "rocky_terrain_02_diff_2k.jpg",
            SRC / "textures" / "rocky_terrain_02" / "2k" / "rocky_terrain_02_nor_gl_2k.jpg",
            SRC / "textures" / "rocky_terrain_02" / "2k" / "rocky_terrain_02_rough_2k.jpg",
        )
    )


def make_cube(name, location, scale, material, bevel=0.0):
    bpy.ops.mesh.primitive_cube_add(location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = scale
    obj.data.materials.append(material)
    if bevel:
        modifier = obj.modifiers.new("soft manufactured edges", "BEVEL")
        modifier.width = bevel
        modifier.segments = 3
    return obj


def make_sensor_mast():
    dark = simple_material("Mast / powder coated steel", (0.08, 0.10, 0.11), 0.38, 0.7)
    indicator = simple_material("Mast / status indicator", (0.94, 0.22, 0.03), 0.3, 0.1)
    bpy.ops.mesh.primitive_cylinder_add(vertices=24, radius=0.12, depth=4.8, location=(11, 9, 3.0))
    mast = bpy.context.object
    mast.name = "infrastructure/survey_sensor_mast"
    mast.data.materials.append(dark)
    for z in (4.0, 5.2):
        bpy.ops.mesh.primitive_cube_add(location=(11, 9, z))
        arm = bpy.context.object
        arm.name = f"infrastructure/survey_sensor_mast/arm_{z:.1f}"
        arm.scale = (0.9, 0.08, 0.08)
        arm.data.materials.append(dark)
    bpy.ops.mesh.primitive_cube_add(location=(11.65, 9, 5.45))
    head = bpy.context.object
    head.name = "sensor/survey_camera_proxy"
    head.scale = (0.28, 0.22, 0.18)
    head.data.materials.append(dark)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=8, radius=0.075, location=(11.92, 8.98, 5.45))
    lens = bpy.context.object
    lens.name = "sensor/survey_camera_proxy/status_lens"
    lens.data.materials.append(indicator)
    return mast


def look_at(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path)
    arguments = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    options = parser.parse_args(arguments)
    out = options.output_dir or ROOT / "build" / "scene-previews" / (
        "mountain-outpost-" + datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    )
    out.mkdir(parents=True, exist_ok=False)
    clear_scene()
    surface = mountain_grid()
    terrain = make_terrain(surface)
    weights = make_path(terrain)
    make_distant_ridges(surface)
    before_infrastructure = set(bpy.context.scene.objects)
    mast = make_sensor_mast()
    industrial = simple_material("Outpost / weathered steel", (0.16, 0.19, 0.20), 0.52, 0.75)
    make_cube("infrastructure/outpost_pad", (11, 9, 0.18), (3.0, 2.2, 0.18), industrial, 0.16)
    make_cube("infrastructure/outpost_console", (10.6, 8.7, 1.0), (0.8, 0.45, 0.8), industrial, 0.08)
    for obj in set(bpy.context.scene.objects) - before_infrastructure:
        obj.location.z += surface.height_at(11, 9)

    imported = [terrain.name, mast.name]
    grounding = []

    rock_root, _ = import_gltf(
        SRC / "rock_09" / "2k" / "rock_09_2k.gltf",
        "world/mountain_outpost/boulder_01",
        (-15, -1, 2.2),
        2.5,
    )
    imported.append(rock_root.name)
    grounding.append(ground_hierarchy(rock_root, surface, embed_m=0.16, height_m=2.8))
    for i, (location, scale, angle) in enumerate(
        [((-4, 12, 0.8), 1.3, 0.4), ((5, 18, 1.1), 1.8, -0.7), ((-23, 14, 1.3), 1.7, 1.0)]
    ):
        rock = clone_hierarchy(rock_root, f"world/mountain_outpost/boulder_{i + 2:02d}", location, scale, angle)
        grounding.append(ground_hierarchy(rock, surface, embed_m=0.12, height_m=scale))
        imported.append(rock.name)
    for i, (location, scale, angle) in enumerate(
        [
            ((-23, -14, 0.35), 0.20, 0.2),
            ((-17, -11, 0.38), 0.14, -0.6),
            ((-10, -6, 0.42), 0.18, 0.9),
            ((-3, -2, 0.45), 0.13, -0.4),
            ((5, 4, 0.55), 0.16, 0.7),
            ((10, 10, 0.72), 0.18, -0.8),
        ]
    ):
        stone = clone_hierarchy(
            rock_root,
            f"world/mountain_outpost/trail_edge_stone_{i + 1:02d}",
            location,
            scale,
            angle,
        )
        grounding.append(ground_hierarchy(stone, surface, embed_m=0.025, height_m=scale))
        imported.append(stone.name)

    pine_preview = out / "source-variants" / "pine_tree_b.gltf"
    pine_source = stage_mesh_variant(
        SRC / "pine_tree_01" / "2k" / "pine_tree_01_2k.gltf",
        pine_preview,
        node_name="pine_tree_01_b_LOD0",
    )
    pine_root, _ = import_gltf(
        pine_preview,
        "vegetation/pine_tree_01",
        (-18, 5, 1.4),
        1.4,
    )
    imported.append(pine_root.name)
    grounding.append(ground_hierarchy(pine_root, surface, embed_m=0.03, height_m=9.0))
    for i, (location, scale, angle) in enumerate(
        [
            ((-25, 11, 0), 1.2, 0.5),
            ((-20, 18, 0), 1.6, -0.4),
            ((-3, 22, 0), 1.1, 0.9),
            ((7, 25, 0), 1.5, -0.8),
            ((-31, -4, 0), 1.2, 0.4),
            ((-25, 0, 0), 1.0, 1.7),
            ((-35, 21, 0), 1.4, -0.9),
            ((-32, 28, 0), 1.3, 0.6),
            ((-15, 31, 0), 1.1, 1.1),
            ((3, 34, 0), 1.6, -0.5),
            ((23, 16, 0), 1.2, 1.5),
            ((29, 27, 0), 1.5, -1.2),
            ((32, 4, 0), 1.1, 0.7),
            ((25, -7, 0), 1.3, 2.0),
        ]
    ):
        tree = clone_hierarchy(pine_root, f"vegetation/pine_tree_{i + 2:02d}", location, scale, angle)
        grounding.append(ground_hierarchy(tree, surface, embed_m=0.03, height_m=scale * 7.5))
        imported.append(tree.name)

    world = bpy.data.worlds.new("Mountain / neutral daylight")
    bpy.context.scene.world = world
    world.use_nodes = True
    nodes = world.node_tree.nodes
    links = world.node_tree.links
    sky = nodes.new("ShaderNodeTexSky")
    sky.sky_type = "MULTIPLE_SCATTERING"
    sky.sun_elevation = math.radians(42)
    sky.sun_rotation = math.radians(-28)
    sky.sun_disc = False
    sky.aerosol_density = 0.3
    background = nodes.get("Background")
    background.inputs["Strength"].default_value = 0.3
    links.new(sky.outputs["Color"], background.inputs["Color"])

    bpy.ops.object.light_add(type="SUN", location=(0, -10, 28))
    sun = bpy.context.object
    sun.name = "lighting/mountain_sun"
    sun.data.energy = 2.5
    sun.data.angle = math.radians(3)
    sun.rotation_euler = (math.radians(25), math.radians(-18), math.radians(-28))
    bpy.ops.object.light_add(type="AREA", location=(4, -5, 16))
    fill = bpy.context.object
    fill.name = "lighting/outpost_fill"
    fill.data.energy = 1000
    fill.data.size = 10
    look_at(fill, (4, 4, 1))

    bpy.ops.object.camera_add(location=(-27, -30, surface.height_at(-27, -30) + 2.6))
    camera = bpy.context.object
    camera.name = "camera/mountain_outpost_hero"
    camera.data.lens = 30
    look_at(camera, (1, 10, 4.0))
    bpy.context.scene.camera = camera

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 900
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.filepath = str(out / "mountain-outpost-hero.png")
    scene.render.film_transparent = False
    scene.view_settings.look = "AgX - Medium High Contrast"

    bpy.ops.wm.save_as_mainfile(filepath=str(out / "mountain-outpost-hf.blend"))
    bpy.ops.render.render(write_still=True)
    camera.location = (34, -36, 18)
    look_at(camera, (0, 6, 1))
    scene.render.filepath = str(out / "mountain-outpost-layout.png")
    bpy.ops.render.render(write_still=True)

    manifest = {
        "schema": "lingtu.scene-preview.v1",
        "id": "mountain_outpost_hf",
        "version": "0.2.0",
        "purpose": "coherent high-fidelity mountain inspection preview; not UE/MuJoCo qualification",
        "authority": {"physics": "not_bound", "rendering": "blender_preview", "ue": "not_bound"},
        "assets": imported,
        "materials": {
            "terrain": "Poly Haven rocky_terrain_02 2K",
            "lighting": "procedural multiple-scattering daylight; no photographic background",
        },
        "sources": [
            "https://polyhaven.com/a/pine_tree_01",
            "https://polyhaven.com/a/rock_09",
            "https://polyhaven.com/a/rocky_terrain_02",
        ],
        "files": {
            "blend": "mountain-outpost-hf.blend",
            "preview": "mountain-outpost-hero.png",
            "layout": "mountain-outpost-layout.png",
        },
        "geometry_checks": {
            "tree_source": pine_source,
            "terrain_vertices": len(terrain.data.vertices),
            "terrain_triangles": len(terrain.data.polygons),
            "terrain_scale": list(terrain.scale),
            "trail_representation": "terrain_vertex_material_mask",
            "trail_weight_range": [min(weights), max(weights)],
            "trail_centerline_m": [[x, y, surface.height_at(x, y)] for x, y in TRAIL],
            "grounded_assets": grounding,
        },
        "limitations": [
            "visual composition only; no UE import or MuJoCo world binding",
            "the terrain is a deterministic preview mesh and needs a production heightfield export",
            "vegetation and rocks still need authored collision proxies",
            "the selected single-tree LOD0 still needs real-time foliage LOD conditioning",
            "preview reuses source buffer/texture paths; it is not a self-contained distribution asset",
        ],
    }
    (out / "scene.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(
        json.dumps({"status": "completed", "preview": str(out / "mountain-outpost-hero.png"), "assets": len(imported)})
    )


if __name__ == "__main__":
    main()
