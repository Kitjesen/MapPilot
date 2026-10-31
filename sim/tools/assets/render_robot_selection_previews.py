"""Render isolated robot selector previews from canonical robot geometry.

The ordinary Python process loads each package MJCF, applies its authored initial
pose, and runs ``mj_forward`` without stepping.  Blender receives only the
resulting geometry transforms and source mesh paths; it does not own simulation.

External ``--plan`` files use the generated v1 plan shape.  Mesh geometries may
reference STL or OBJ, plus optional ``scale``, ``visual_transform``
(``position`` and ``quaternion_wxyz``), and ``preserve_materials`` fields.  A
robot may also declare named PBR materials for known URDF/MJCF material colors.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Sequence

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_BLENDER = Path("D:/Development/Blender/5.2/blender.exe")
DEFAULT_OUTPUT_DIR = REPOSITORY_ROOT / "build/selector-isolated-20261016"
SELECTION_DIR = (
    REPOSITORY_ROOT
    / "sim/runtime/visual/RobotSimUE/Plugins/LingTuSim/Resources/Selection"
)
PREVIEW_WIDTH = 1400
PREVIEW_HEIGHT = 1200

ROBOT_SPECS: dict[str, dict[str, Any]] = {
    "thunderv4": {
        "xml": REPOSITORY_ROOT
        / "sim/packages/robots/doso/thunder_v4/mjcf/thunderv4.xml",
        "projection": REPOSITORY_ROOT
        / "sim/packages/robots/doso/thunder_v4/visual/robot.visual-projection.json",
        "filename": "robot-thunderv4.png",
        "keyframe": "v4_nominal_stand",
        "visual_group": 1,
    },
    "omni_cart": {
        "xml": REPOSITORY_ROOT / "sim/packages/robots/omni_cart/omni_cart.xml",
        "projection": REPOSITORY_ROOT
        / "sim/packages/robots/omni_cart/visual/robot.visual-projection.json",
        "filename": "robot-omni_cart.png",
        "keyframe": None,
        "visual_group": None,
    },
}


def _name(model: Any, object_type: Any, object_id: int) -> str:
    import mujoco

    return mujoco.mj_id2name(model, object_type, object_id) or f"unnamed_{object_id}"


def _material_role(robot_id: str, geom_name: str, body_name: str) -> str:
    identity = f"{geom_name} {body_name}".lower()
    if "wheel" in identity or "foot" in identity or "support" in identity:
        return "rubber"
    if "camera" in identity or "lidar" in identity or "sensor" in identity:
        return "sensor"
    if robot_id == "thunderv4" and ("thigh" in identity or "calf" in identity):
        return "graphite"
    if robot_id == "omni_cart" and "top_plate" in identity:
        return "graphite"
    return "silver"


def _projection_meshes(path: Path) -> dict[str, Path]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    result: dict[str, Path] = {}
    package_root = path.parent.parent
    for component in raw.get("components", []):
        geometry = component.get("geometry", {})
        if geometry.get("kind") != "mesh":
            continue
        mesh_name = geometry["mesh"]
        source = (package_root / geometry["source_mesh"]).resolve()
        if not source.is_file():
            raise FileNotFoundError(f"projection mesh does not exist: {source}")
        result[mesh_name] = source
    return result


def _build_robot_plan(robot_id: str, output_dir: Path) -> dict[str, Any]:
    import mujoco

    spec = ROBOT_SPECS[robot_id]
    xml_path = Path(spec["xml"])
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    keyframe = spec["keyframe"]
    if keyframe is not None:
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
        if key_id < 0:
            raise ValueError(f"{robot_id}: missing keyframe {keyframe!r}")
        data.qpos[:] = model.key_qpos[key_id]
    mujoco.mj_forward(model, data)

    mesh_sources = _projection_meshes(Path(spec["projection"]))
    geometries: list[dict[str, Any]] = []
    for geom_id in range(model.ngeom):
        geom_type = int(model.geom_type[geom_id])
        if geom_type == int(mujoco.mjtGeom.mjGEOM_PLANE):
            continue
        visual_group = spec["visual_group"]
        if visual_group is not None and int(model.geom_group[geom_id]) != visual_group:
            continue

        geom_name = _name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        body_id = int(model.geom_bodyid[geom_id])
        body_name = _name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        is_mesh = geom_type == int(mujoco.mjtGeom.mjGEOM_MESH)
        geometry: dict[str, Any] = {
            "name": f"{body_name}_{geom_name}",
            "type": mujoco.mjtGeom(geom_type).name.removeprefix("mjGEOM_").lower(),
            # Projection mesh sources are already authored in body-local space.
            # MuJoCo geom_x* includes the compiler's internal mesh recentering,
            # so applying it to the original STL would transform the mesh twice.
            "position": (data.xpos[body_id] if is_mesh else data.geom_xpos[geom_id]).tolist(),
            "rotation": (
                data.xmat[body_id] if is_mesh else data.geom_xmat[geom_id]
            ).reshape(3, 3).tolist(),
            "size": model.geom_size[geom_id].tolist(),
            "material": _material_role(robot_id, geom_name, body_name),
        }
        if is_mesh:
            mesh_id = int(model.geom_dataid[geom_id])
            mesh_name = _name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
            try:
                geometry["mesh"] = str(mesh_sources[mesh_name])
            except KeyError as exc:
                raise ValueError(
                    f"{robot_id}: projection has no source for MJCF mesh {mesh_name!r}"
                ) from exc
        geometries.append(geometry)

    if not geometries:
        raise ValueError(f"{robot_id}: no renderable robot geometry")
    return {
        "id": robot_id,
        "filename": spec["filename"],
        "xml": str(xml_path),
        "projection": str(spec["projection"]),
        "output": str((output_dir / spec["filename"]).resolve()),
        "width": PREVIEW_WIDTH,
        "height": PREVIEW_HEIGHT,
        "geometries": geometries,
        "source_xml": xml_path.resolve().relative_to(REPOSITORY_ROOT).as_posix(),
        "source_projection": Path(spec["projection"])
        .resolve()
        .relative_to(REPOSITORY_ROOT)
        .as_posix(),
        "kind": "blender_geometry_render",
        "note": "Isolated geometry preview from the package MJCF after mj_forward; no physics step or runtime validation.",
    }


def _write_plan(robot_ids: Sequence[str], output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    plan_path = output_dir / "robot-selection-render-plan.json"
    payload = {
        "schema": "lingtu.sim.robot-selection-render-plan.v1",
        "robots": [_build_robot_plan(robot_id, output_dir) for robot_id in robot_ids],
    }
    plan_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return plan_path


def _model_mesh_path(model: Any, mesh_id: int, xml_path: Path) -> Path:
    address = int(model.mesh_pathadr[mesh_id])
    if address < 0:
        raise ValueError(f"{xml_path}: mesh {mesh_id} has no source path")
    filename = model.paths[address:].split(b"\x00", 1)[0].decode("utf-8")
    candidates = (xml_path.parent / "assets" / filename, xml_path.parent / filename)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(f"{xml_path}: cannot resolve mesh source {filename!r}")


def _build_mjcf_plan(robot_id: str, xml_path: Path, output_dir: Path) -> dict[str, Any]:
    import mujoco
    import numpy as np

    xml_path = xml_path.resolve()
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    if model.nkey:
        data.qpos[:] = model.key_qpos[0]
    mujoco.mj_forward(model, data)

    geometries: list[dict[str, Any]] = []
    materials: dict[str, dict[str, Any]] = {}
    for geom_id in range(model.ngeom):
        if int(model.geom_group[geom_id]) != 2:
            continue
        if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_MESH):
            raise ValueError(f"{xml_path}: visual geom {geom_id} is not a mesh")
        mesh_id = int(model.geom_dataid[geom_id])

        geom_world = np.eye(4)
        geom_world[:3, :3] = data.geom_xmat[geom_id].reshape(3, 3)
        geom_world[:3, 3] = data.geom_xpos[geom_id]
        mesh_rotation = np.empty(9)
        mujoco.mju_quat2Mat(mesh_rotation, model.mesh_quat[mesh_id])
        mesh_center = np.eye(4)
        mesh_center[:3, :3] = mesh_rotation.reshape(3, 3)
        mesh_center[:3, 3] = model.mesh_pos[mesh_id]
        source_world = geom_world @ np.linalg.inv(mesh_center)

        material_id = int(model.geom_matid[geom_id])
        rgba = (
            model.mat_rgba[material_id]
            if material_id >= 0
            else model.geom_rgba[geom_id]
        )
        material_key = "source_" + "_".join(f"{float(value):.3f}" for value in rgba)
        materials.setdefault(
            material_key,
            {
                "base_color_rgba": [float(value) for value in rgba],
                "metallic": 0.05,
                "roughness": 0.46,
            },
        )
        geometries.append(
            {
                "name": _name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id),
                "type": "mesh",
                "mesh": str(_model_mesh_path(model, mesh_id, xml_path)),
                "position": source_world[:3, 3].tolist(),
                "rotation": source_world[:3, :3].tolist(),
                "scale": model.mesh_scale[mesh_id].tolist(),
                "size": [0.0, 0.0, 0.0],
                "material": material_key,
            }
        )
    if not geometries:
        raise ValueError(f"{xml_path}: no group-2 visual meshes")
    try:
        source_xml = xml_path.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        source_xml = xml_path.as_posix()
    return {
        "id": robot_id,
        "filename": f"robot-{robot_id}.png",
        "output": str((output_dir / f"robot-{robot_id}.png").resolve()),
        "width": PREVIEW_WIDTH,
        "height": PREVIEW_HEIGHT,
        "geometries": geometries,
        "materials": materials,
        "source_xml": source_xml,
        "kind": "blender_geometry_render",
        "note": "Isolated visual geometry from the source MJCF home keyframe after mj_forward; no physics step or runtime validation.",
    }


def _write_mjcf_plan(entries: Sequence[str], output_dir: Path) -> Path:
    robots: list[dict[str, Any]] = []
    for entry in entries:
        robot_id, separator, raw_path = entry.partition("=")
        if not separator or not robot_id or not raw_path:
            raise ValueError("--mjcf values must use ID=PATH")
        robots.append(_build_mjcf_plan(robot_id, Path(raw_path), output_dir))
    output_dir.mkdir(parents=True, exist_ok=True)
    plan_path = output_dir / "robot-selection-render-plan.json"
    plan_path.write_text(
        json.dumps(
            {"schema": "lingtu.sim.robot-selection-render-plan.v1", "robots": robots},
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return plan_path


def _load_external_plan(source_path: Path, output_dir: Path) -> Path:
    source_path = source_path.resolve()
    raw = json.loads(source_path.read_text(encoding="utf-8"))
    if raw.get("schema") != "lingtu.sim.robot-selection-render-plan.v1":
        raise ValueError(f"{source_path}: unsupported render plan schema")
    robots = raw.get("robots")
    if not isinstance(robots, list) or not robots:
        raise ValueError(f"{source_path}: robots must be a non-empty array")

    normalized: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    seen_filenames: set[str] = set()
    for index, raw_robot in enumerate(robots):
        if not isinstance(raw_robot, dict):
            raise ValueError(f"{source_path}: robot {index} must be an object")
        robot = dict(raw_robot)
        robot_id = robot.get("id")
        if not isinstance(robot_id, str) or not robot_id or robot_id in seen_ids:
            raise ValueError(f"{source_path}: robot {index} has an invalid or duplicate id")
        filename = robot.get("filename", f"robot-{robot_id}.png")
        if (
            not isinstance(filename, str)
            or Path(filename).name != filename
            or Path(filename).suffix.lower() != ".png"
            or filename in seen_filenames
        ):
            raise ValueError(f"{source_path}: robot {robot_id} has an invalid filename")
        geometries = robot.get("geometries")
        if not isinstance(geometries, list) or not geometries:
            raise ValueError(f"{source_path}: robot {robot_id} has no geometry")
        normalized_geometries: list[dict[str, Any]] = []
        for geometry_index, raw_geometry in enumerate(geometries):
            if not isinstance(raw_geometry, dict):
                raise ValueError(
                    f"{source_path}: robot {robot_id} geometry {geometry_index} must be an object"
                )
            geometry = dict(raw_geometry)
            geometry.setdefault("material", "silver")
            geometry.setdefault("scale", [1.0, 1.0, 1.0])
            if geometry.get("type") == "mesh":
                mesh = geometry.get("mesh")
                if not isinstance(mesh, str) or not mesh:
                    raise ValueError(
                        f"{source_path}: robot {robot_id} geometry {geometry_index} has no mesh"
                    )
                mesh_path = Path(mesh)
                if not mesh_path.is_absolute():
                    mesh_path = source_path.parent / mesh_path
                mesh_path = mesh_path.resolve()
                if not mesh_path.is_file() or mesh_path.suffix.lower() not in {".stl", ".obj", ".dae"}:
                    raise ValueError(
                        f"{source_path}: unsupported or missing mesh {mesh_path}"
                    )
                geometry["mesh"] = str(mesh_path)
            normalized_geometries.append(geometry)
        robot["id"] = robot_id
        robot["filename"] = filename
        robot["output"] = str((output_dir / filename).resolve())
        robot["width"] = int(robot.get("width", PREVIEW_WIDTH))
        robot["height"] = int(robot.get("height", PREVIEW_HEIGHT))
        robot["geometries"] = normalized_geometries
        robot.setdefault("kind", "blender_geometry_render")
        robot.setdefault(
            "note",
            "Isolated geometry preview from an offline geometry render plan; not runtime validation.",
        )
        try:
            robot.setdefault(
                "source_plan", source_path.relative_to(REPOSITORY_ROOT).as_posix()
            )
        except ValueError:
            robot.setdefault("source_plan", source_path.as_posix())
        normalized.append(robot)
        seen_ids.add(robot_id)
        seen_filenames.add(filename)

    output_dir.mkdir(parents=True, exist_ok=True)
    plan_path = output_dir / "robot-selection-render-plan.json"
    plan_path.write_text(
        json.dumps(
            {"schema": "lingtu.sim.robot-selection-render-plan.v1", "robots": normalized},
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return plan_path


def _validate_png(path: Path) -> dict[str, int]:
    from PIL import Image

    with Image.open(path) as image:
        image.load()
        if image.mode != "RGBA":
            raise ValueError(f"{path}: expected RGBA, got {image.mode}")
        if image.width < 1200 or image.height < 1000:
            raise ValueError(f"{path}: preview must be at least 1200x1000")
        alpha = image.getchannel("A")
        bounds = alpha.getbbox()
        if bounds is None:
            raise ValueError(f"{path}: preview is fully transparent")
        if alpha.getextrema()[0] != 0:
            raise ValueError(f"{path}: preview has no transparent background")
        left, top, right, bottom = bounds
        if left < 20 or top < 20 or right > image.width - 20 or bottom > image.height - 20:
            raise ValueError(f"{path}: robot framing is not safely inside the canvas")
        span = max((right - left) / image.width, (bottom - top) / image.height)
        if span < 0.6:
            raise ValueError(f"{path}: robot framing is too loose ({span:.1%})")
        return {
            "width": image.width,
            "height": image.height,
            "alpha_left": left,
            "alpha_top": top,
            "alpha_right": right,
            "alpha_bottom": bottom,
        }


def _update_preview_manifest(plans: Sequence[dict[str, Any]]) -> None:
    manifest_path = SELECTION_DIR / "previews.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    robot_ids = {plan["id"] for plan in plans}
    retained = [
        item for item in manifest.get("robots", []) if item.get("id") not in robot_ids
    ]
    entries = []
    metadata_keys = (
        "source_xml",
        "source_projection",
        "source_urdf",
        "source_plan",
        "kind",
        "note",
    )
    for plan in plans:
        output_path = Path(plan["output"]).resolve()
        try:
            source_path = output_path.relative_to(REPOSITORY_ROOT).as_posix()
        except ValueError:
            source_path = output_path.as_posix()
        entry = {
            "id": plan["id"],
            "filename": plan["filename"],
            "source_path": source_path,
        }
        entry.update({key: plan[key] for key in metadata_keys if key in plan})
        entries.append(entry)
    manifest["robots"] = retained + entries
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


def _install(plans: Sequence[dict[str, Any]]) -> None:
    SELECTION_DIR.mkdir(parents=True, exist_ok=True)
    for plan in plans:
        shutil.copy2(Path(plan["output"]), SELECTION_DIR / plan["filename"])
    _update_preview_manifest(plans)


def _blender_material(
    name: str,
    color: tuple[float, float, float, float],
    metallic: float,
    roughness: float,
    specular_ior_level: float = 0.5,
) -> Any:
    import bpy

    material = bpy.data.materials.new(name)
    material.diffuse_color = color
    material.use_nodes = True
    shader = material.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = color
    shader.inputs["Metallic"].default_value = metallic
    shader.inputs["Roughness"].default_value = roughness
    if specular := shader.inputs.get("Specular IOR Level"):
        specular.default_value = specular_ior_level
    return material


def _look_at(obj: Any, target: Any) -> None:
    direction = target - obj.location
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def _matrix(geometry: dict[str, Any]) -> Any:
    from mathutils import Matrix

    rotation = geometry["rotation"]
    position = geometry["position"]
    return Matrix(
        (
            (rotation[0][0], rotation[0][1], rotation[0][2], position[0]),
            (rotation[1][0], rotation[1][1], rotation[1][2], position[1]),
            (rotation[2][0], rotation[2][1], rotation[2][2], position[2]),
            (0.0, 0.0, 0.0, 1.0),
        )
    )


def _visual_matrix(geometry: dict[str, Any]) -> Any:
    from mathutils import Matrix, Quaternion

    visual = geometry.get("visual_transform", {})
    position = visual.get("position", [0.0, 0.0, 0.0])
    quaternion = visual.get("quaternion_wxyz", [1.0, 0.0, 0.0, 0.0])
    transform = Quaternion(quaternion).to_matrix().to_4x4()
    transform.translation = position
    scale = geometry.get("scale", [1.0, 1.0, 1.0])
    return transform @ Matrix.Diagonal((*scale, 1.0))


def _create_geometry(
    geometry: dict[str, Any], materials: dict[str, Any]
) -> list[Any]:
    import bpy
    from mathutils import Matrix

    geom_type = geometry["type"]
    size = geometry.get("size", [0.0, 0.0, 0.0])
    local_transform = Matrix.Identity(4)
    if geom_type == "mesh":
        before = set(bpy.context.scene.objects)
        mesh_path = Path(geometry["mesh"])
        if mesh_path.suffix.lower() == ".stl":
            bpy.ops.wm.stl_import(
                filepath=str(mesh_path), forward_axis="Y", up_axis="Z"
            )
        elif mesh_path.suffix.lower() == ".obj":
            bpy.ops.wm.obj_import(
                filepath=str(mesh_path), forward_axis="Y", up_axis="Z"
            )
        elif mesh_path.suffix.lower() == ".dae":
            bpy.ops.wm.collada_import(filepath=str(mesh_path))
        else:
            raise ValueError(f"unsupported mesh format: {mesh_path}")
        objects = [
            obj
            for obj in bpy.context.scene.objects
            if obj not in before and obj.type == "MESH"
        ]
        if not objects:
            raise RuntimeError(f"mesh import created no objects: {mesh_path}")
    elif geom_type == "box":
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        objects = [bpy.context.object]
        local_transform = Matrix.Diagonal((size[0], size[1], size[2], 1.0))
    elif geom_type == "cylinder":
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=64, radius=size[0], depth=2.0 * size[1]
        )
        objects = [bpy.context.object]
    elif geom_type == "sphere":
        bpy.ops.mesh.primitive_uv_sphere_add(
            segments=64, ring_count=32, radius=size[0]
        )
        objects = [bpy.context.object]
    else:
        raise ValueError(f"unsupported MuJoCo geometry type: {geom_type}")
    world_transform = _matrix(geometry) @ _visual_matrix(geometry)
    preserve_materials = bool(geometry.get("preserve_materials"))
    for index, obj in enumerate(objects):
        import_transform = obj.matrix_world.copy() if geom_type == "mesh" else local_transform
        obj.name = geometry["name"] if len(objects) == 1 else f"{geometry['name']}_{index:02d}"
        obj.matrix_world = world_transform @ import_transform
        if not preserve_materials or not obj.data.materials:
            obj.data.materials.clear()
            obj.data.materials.append(materials[geometry["material"]])
        if geom_type == "sphere":
            for polygon in obj.data.polygons:
                polygon.use_smooth = True
        elif geom_type == "cylinder":
            for polygon in obj.data.polygons:
                polygon.use_smooth = abs(polygon.normal.z) < 0.5
    return objects


def _world_bounds(objects: Sequence[Any]) -> tuple[Any, Any]:
    from mathutils import Vector

    points = [obj.matrix_world @ Vector(corner) for obj in objects for corner in obj.bound_box]
    minimum = Vector(tuple(min(point[axis] for point in points) for axis in range(3)))
    maximum = Vector(tuple(max(point[axis] for point in points) for axis in range(3)))
    return minimum, maximum


def _render_robot(plan: dict[str, Any]) -> None:
    import bpy
    from mathutils import Vector

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = int(plan["width"])
    scene.render.resolution_y = int(plan["height"])
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.film_transparent = True
    scene.render.filepath = plan["output"]
    scene.render.image_settings.color_depth = "8"
    scene.view_settings.look = "AgX - Medium High Contrast"

    world = bpy.data.worlds.new("SelectorTransparentWorld")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.04, 0.05, 0.07, 1.0)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.28
    scene.world = world

    materials = {
        "silver": _blender_material(
            "M_Selector_Silver", (0.20, 0.25, 0.31, 1.0), 0.50, 0.42, 0.42
        ),
        "graphite": _blender_material(
            "M_Selector_Graphite", (0.025, 0.034, 0.048, 1.0), 0.32, 0.46, 0.30
        ),
        "rubber": _blender_material(
            "M_Selector_Rubber", (0.004, 0.006, 0.009, 1.0), 0.0, 0.68, 0.18
        ),
        "sensor": _blender_material(
            "M_Selector_Sensor", (0.012, 0.020, 0.032, 1.0), 0.34, 0.30, 0.30
        ),
    }
    for material_id, material_spec in plan.get("materials", {}).items():
        materials[material_id] = _blender_material(
            f"M_Selector_{material_id}",
            tuple(material_spec["base_color_rgba"]),
            float(material_spec.get("metallic", 0.0)),
            float(material_spec.get("roughness", 0.45)),
        )
    objects = [
        obj
        for geometry in plan["geometries"]
        for obj in _create_geometry(geometry, materials)
    ]
    minimum, maximum = _world_bounds(objects)
    center = (minimum + maximum) * 0.5
    extents = maximum - minimum
    radius = max(extents.length * 0.5, 0.25)

    camera_data = bpy.data.cameras.new("SelectorCamera")
    camera = bpy.data.objects.new("SelectorCamera", camera_data)
    scene.collection.objects.link(camera)
    camera.data.lens = 58.0
    camera.data.sensor_width = 36.0
    camera.data.dof.use_dof = False
    direction = Vector((1.25, -1.55, 0.92)).normalized()
    camera.location = center + direction * radius * 4.0
    _look_at(camera, center)
    # Fit the actual transformed bounds, including slender humanoids and long legs.
    frame = camera_data.view_frame(scene=scene)
    limit_x = abs(frame[0].x / frame[0].z) * 0.90
    limit_y = abs(frame[0].y / frame[0].z) * 0.90
    inverse_rotation = camera.rotation_euler.to_matrix().transposed()
    distance = 0.0
    for obj in objects:
        for corner in obj.bound_box:
            point = inverse_rotation @ (obj.matrix_world @ Vector(corner) - center)
            distance = max(distance, point.z + abs(point.x) / limit_x,
                           point.z + abs(point.y) / limit_y)
    camera.location = center + direction * max(distance, radius)
    scene.camera = camera

    lights = (
        ("Key", (3.4, -4.0, 4.8), 1450.0, 3.0),
        ("Fill", (-3.2, -1.8, 2.5), 900.0, 2.7),
        ("Rim", (-1.4, 3.6, 4.2), 1200.0, 2.5),
    )
    for name, relative_position, energy_density, relative_size in lights:
        light_data = bpy.data.lights.new(f"Selector{name}", "AREA")
        light_data.energy = energy_density * radius * radius
        light_data.shape = "DISK"
        light_data.size = relative_size * radius
        light = bpy.data.objects.new(f"Selector{name}", light_data)
        scene.collection.objects.link(light)
        light.location = center + Vector(relative_position) * radius
        _look_at(light, center)

    bpy.ops.render.render(write_still=True)


def _run_inside_blender(plan_path: Path) -> int:
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    for robot in plan["robots"]:
        _render_robot(robot)
        print(f"rendered {robot['id']}: {robot['output']}")
    return 0


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render transparent robot selector previews from package geometry"
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--robot", action="append", choices=sorted(ROBOT_SPECS))
    source.add_argument(
        "--plan",
        type=Path,
        help="render a reusable lingtu.sim.robot-selection-render-plan.v1 JSON file",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--blender", type=Path, default=DEFAULT_BLENDER)
    parser.add_argument("--install", action="store_true")
    parser.add_argument("--blender-plan", type=Path, help=argparse.SUPPRESS)
    return parser.parse_args(list(argv))


def main(argv: Sequence[str] | None = None) -> int:
    if argv is None:
        raw_args = sys.argv[1:]
        if "--" in raw_args:
            raw_args = raw_args[raw_args.index("--") + 1 :]
    else:
        raw_args = list(argv)
    args = _parse_args(raw_args)
    if args.blender_plan is not None:
        return _run_inside_blender(args.blender_plan.resolve())

    output_dir = args.output_dir.resolve()
    blender = args.blender.resolve()
    if not blender.is_file():
        raise FileNotFoundError(f"Blender executable not found: {blender}")
    if args.plan is not None:
        plan_path = _load_external_plan(args.plan, output_dir)
    else:
        robot_ids = args.robot or list(ROBOT_SPECS)
        plan_path = _write_plan(robot_ids, output_dir)
    plans = json.loads(plan_path.read_text(encoding="utf-8"))["robots"]
    command = [
        str(blender),
        "--background",
        "--factory-startup",
        "--python",
        str(Path(__file__).resolve()),
        "--",
        "--blender-plan",
        str(plan_path),
    ]
    subprocess.run(command, cwd=REPOSITORY_ROOT, check=True, timeout=600)
    reports = {plan["id"]: _validate_png(Path(plan["output"])) for plan in plans}
    if args.install:
        _install(plans)
    print(json.dumps(reports, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
