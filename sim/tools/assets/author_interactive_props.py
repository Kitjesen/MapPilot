"""Blender authoring for two joint-aligned props, with adjacent MuJoCo models."""

from __future__ import annotations

import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("interaction_lab", Path(__file__).with_name("interaction_lab.py"))
lab = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lab)
OUT = ROOT / "build/interactive-props-20261017"


def material(name, color, metal=0.0, rough=0.45):
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*color, 1)
    m.use_nodes = True
    shader = m.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = (*color, 1)
    shader.inputs["Metallic"].default_value = metal
    shader.inputs["Roughness"].default_value = rough
    noise = m.node_tree.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 180
    bump = m.node_tree.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.12
    bump.inputs["Distance"].default_value = 0.00025
    m.node_tree.links.new(noise.outputs["Fac"], bump.inputs["Height"])
    m.node_tree.links.new(bump.outputs["Normal"], shader.inputs["Normal"])
    return m


def cube(name, location, size, mat, parent=None, bevel=0.004):
    bpy.ops.mesh.primitive_cube_add(size=1)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.parent = parent
    obj.location = location
    obj.data.materials.append(mat)
    if bevel:
        mod = obj.modifiers.new("Manufactured edges", "BEVEL")
        mod.width = bevel
        mod.segments = 3
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=mod.name)
        mod = obj.modifiers.new("Face normals", "WEIGHTED_NORMAL")
        bpy.ops.object.modifier_apply(modifier=mod.name)
    return obj


def cylinder(name, location, radius, depth, mat, parent=None, rotation=None):
    bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=radius, depth=depth)
    obj = bpy.context.object
    obj.name = name
    obj.parent = parent
    obj.location = location
    if rotation:
        obj.rotation_euler = rotation
    obj.data.materials.append(mat)
    mod = obj.modifiers.new("Edge radius", "BEVEL")
    mod.width = 0.0015
    mod.segments = 2
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=mod.name)
    return obj


def vector(raw):
    return tuple(map(float, raw.split()))


def make_asset(asset_id, body_name, title):
    output = OUT / asset_id
    output.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    metal = material("Galvanized steel", (0.4, 0.44, 0.47), 0.8, 0.36)
    black = material("Black rubber", (0.028, 0.034, 0.039), 0, 0.73)
    white = material("Warm white enamel", (0.8, 0.82, 0.79), 0.18, 0.32)
    red = material("Safety red", (0.65, 0.045, 0.025), 0.12, 0.35)
    paint = material(
        "Machine yellow" if asset_id == "boom_barrier" else "Blue gray enamel",
        (0.82, 0.57, 0.055) if asset_id == "boom_barrier" else (0.32, 0.43, 0.48),
        0.15,
        0.38,
    )
    tree = ET.fromstring(lab.build_scene())
    world = tree.find("worldbody")
    selected = world.find(f"body[@name='{body_name}']")
    for other in list(world):
        if other is not selected and other.tag == "body":
            world.remove(other)
    selected.set("pos", "0 0 0")
    allowed = {j.get("name") for j in selected.iter("joint")}
    actuators = tree.find("actuator")
    for actuator in list(actuators):
        if actuator.get("joint") not in allowed:
            actuators.remove(actuator)
    if tree.find("equality") is not None:
        tree.remove(tree.find("equality"))
    ET.indent(tree, space="  ")
    ET.ElementTree(tree).write(output / "physics.xml", encoding="utf-8", xml_declaration=True)
    nodes = {}
    joints = []

    def build_node(element, parent=None):
        name = element.get("name")
        obj = bpy.data.objects.new(name, None)
        bpy.context.collection.objects.link(obj)
        obj.parent = parent
        obj.location = vector(element.get("pos", "0 0 0"))
        nodes[name] = obj
        for geom in element.findall("geom"):
            size = [2 * x for x in vector(geom.get("size"))]
            cube(
                geom.get("name"),
                vector(geom.get("pos", "0 0 0")),
                size,
                white if "arm" in geom.get("name") else paint,
                obj,
            )
        for joint in element.findall("joint"):
            axis = vector(joint.get("axis"))
            limits = vector(joint.get("range"))
            joints.append(
                {
                    "body": name,
                    "joint": joint.get("name"),
                    "axis": axis,
                    "range_rad": limits,
                    "pivot_m": list(obj.location),
                    "mass_source": "physics.xml",
                }
            )
        for child in element.findall("body"):
            build_node(child, obj)

    build_node(selected)
    root = nodes[body_name]
    if asset_id == "boom_barrier":
        boom = nodes["barrier/boom"]
        # Stripes belong to the arm; housing details never rotate with it.
        for i in range(6):
            cube(f"arm_stripe_{i}", (0.25 + i * 0.5, -0.046, 0), (0.20, 0.003, 0.103), red, boom, 0.001)
            cube(f"arm_stripe_back_{i}", (0.25 + i * 0.5, 0.046, 0), (0.20, 0.003, 0.103), red, boom, 0.001)
        cube("Service panel", (0.204, 0, 0.53), (0.012, 0.32, 0.70), white, root, 0.005)
        cube("Service panel seam", (0.202, 0, 0.53), (0.009, 0.34, 0.72), black, root, 0.005)
        cube("Key lock", (0.216, -0.115, 0.55), (0.016, 0.025, 0.06), black, root, 0.003)
        cube("Base plinth", (0, 0, 0.045), (0.47, 0.5, 0.09), metal, root, 0.006)
        cylinder("Drive shaft cover", (0, -0.235, 1.05), 0.085, 0.055, metal, root, (math.pi / 2, 0, 0))
        for i in range(6):
            cube(f"Cooling slot {i}", (-0.205, 0, 0.4 + i * 0.045), (0.006, 0.25, 0.009), black, root, 0.001)
        for x in [-0.19, 0.19]:
            for y in [-0.20, 0.20]:
                cylinder("Anchor bolt", (x, y, 0.10), 0.012, 0.022, metal, root)
        camera_pos = (5, -6, 3.2)
        target = (1.0, 0, 0.65)
        scale = 4.6
    else:
        for side, sign in [("left", 1), ("right", -1)]:
            leaf = nodes[f"service_door/{side}"]
            cube(f"{side} inset panel", (0.039, -sign * 0.5, 0.12), (0.014, 0.83, 1.35), white, leaf, 0.008)
            cube(f"{side} lower kickplate", (0.041, -sign * 0.5, -0.72), (0.015, 0.88, 0.28), metal, leaf, 0.003)
            for z in [-0.65, 0, 0.65]:
                cylinder(f"{side} hinge", (0, 0, z), 0.024, 0.11, metal, leaf)
            cube(f"{side} handle base", (0.062, -sign * 0.86, -0.05), (0.03, 0.055, 0.2), black, leaf, 0.004)
            cylinder(f"{side} handle", (0.12, -sign * 0.86, -0.05), 0.014, 0.16, metal, leaf)
        for sign in [-1, 1]:
            for z in [0.18, 1.05, 1.90]:
                cylinder("Frame fixing", (0.095, sign * 1.05, z), 0.012, 0.015, metal, root, (0, math.pi / 2, 0))
        camera_pos = (5, -4, 2.9)
        target = (0, 0, 1.05)
        scale = 3.1
    all_asset = list(bpy.context.scene.objects)
    # Every moving node is keyed at its physical hinge axis, not at mesh centre.
    for joint in joints:
        obj = nodes[joint["body"]]
        axis = Vector(joint["axis"])
        obj.rotation_mode = "AXIS_ANGLE"
        for frame, angle in [(1, 0), (61, joint["range_rad"][1])]:
            obj.rotation_axis_angle = (angle, axis.x, axis.y, axis.z)
            obj.keyframe_insert(data_path="rotation_axis_angle", frame=frame)
    bpy.context.scene.render.fps = 30
    bpy.context.scene.frame_end = 61
    bpy.context.scene.frame_set(1)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in all_asset:
        obj.select_set(True)
    bpy.ops.export_scene.gltf(
        filepath=str(output / "model.glb"), export_format="GLB", use_selection=True, export_animations=False
    )
    bpy.ops.export_scene.gltf(
        filepath=str(output / "open-close.glb"),
        export_format="GLB",
        use_selection=True,
        export_animations=True,
        export_animation_mode="ACTIVE_ACTIONS",
    )
    (output / "mechanism.json").write_text(
        json.dumps(
            {
                "asset_id": asset_id,
                "title": title,
                "units": "m",
                "source_axes": "RH Z-up",
                "joints": joints,
                "physics": "physics.xml",
                "ue_qualified": False,
                "parameters": "design_assumptions",
                "materials": "procedural PBR; no bitmap texture atlas",
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    ground = material("Studio ground", (0.42, 0.45, 0.47), 0, 0.8)
    cube("Preview floor", (0, 0, -0.08), (200, 200, 0.1), ground, bevel=0)
    bpy.ops.object.camera_add(location=camera_pos)
    cam = bpy.context.object
    cam.rotation_euler = (Vector(target) - cam.location).to_track_quat("-Z", "Y").to_euler()
    cam.data.type = "ORTHO"
    cam.data.ortho_scale = scale
    bpy.context.scene.camera = cam
    bpy.ops.object.light_add(type="AREA", location=(1, -4, 6))
    light = bpy.context.object
    light.data.energy = 1400
    light.data.shape = "DISK"
    light.data.size = 5
    light.rotation_euler = (Vector(target) - light.location).to_track_quat("-Z", "Y").to_euler()
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 32
    scene.cycles.use_denoising = True
    scene.world.color = (0.3, 0.3, 0.3)
    scene.render.resolution_x = 1000
    scene.render.resolution_y = 800
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.view_settings.view_transform = "AgX"
    bpy.ops.wm.save_as_mainfile(filepath=str(output / "source.blend"))
    for frame, name in [(1, "preview.png"), (61, "open.png")]:
        scene.frame_set(frame)
        if frame == 61:
            cam.data.ortho_scale = 5.0 if asset_id == "boom_barrier" else 4.1
            open_target = Vector((0.4, 0, 2.0 if asset_id == "boom_barrier" else 1.05))
            cam.rotation_euler = (open_target - cam.location).to_track_quat("-Z", "Y").to_euler()
        scene.render.filepath = str(output / name)
        bpy.ops.render.render(write_still=True)
    print("INTERACTIVE_PROP_EXPORTED", asset_id, flush=True)


requested = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
for args in [("boom_barrier", "barrier/base", "电动道闸"), ("service_door", "service_door/frame", "双开检修门")]:
    if not requested or args[0] in requested:
        make_asset(*args)
