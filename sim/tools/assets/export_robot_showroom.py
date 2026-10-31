"""Export the available real-robot showroom library as joined static FBX assets.

This is an offline geometry conversion.  It consumes the already-normalized
robot selection render plans, reuses their Blender geometry construction, and
does not load or step MuJoCo.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
DEFAULT_BLENDER = Path("D:/Development/Blender/5.2/blender.exe")
DEFAULT_OUTPUT_DIR = REPOSITORY_ROOT / "build/robot-showroom"
THUNDER_DISPLAY_BASE = (
    REPOSITORY_ROOT / "sim/runtime/visual/RobotSimUE/SourceAssets/Robots/ThunderV4/base_link_visual_high_m.stl"
)
THUNDER_DISPLAY_MANIFEST = THUNDER_DISPLAY_BASE.parent / "display_meshes.json"
AVAILABLE_PLAN = (
    REPOSITORY_ROOT
    / "build/robot-library-previews-20261016/robot-selection-render-plan.json"
)
BUILTIN_PLAN = (
    REPOSITORY_ROOT
    / "build/selector-isolated-20261016/contract-normalized/robot-selection-render-plan.json"
)
UNAVAILABLE_PLAN = (
    REPOSITORY_ROOT / "build/robot-library-previews-20261016/pending-previews.json"
)
EXPECTED_AVAILABLE_ROBOTS = 23

ROLE_MATERIALS: dict[str, dict[str, Any]] = {
    "silver": {
        "rgba": [0.20, 0.25, 0.31, 1.0],
        "metallic": 0.50,
        "roughness": 0.42,
        "specular_ior_level": 0.42,
    },
    "graphite": {
        "rgba": [0.025, 0.034, 0.048, 1.0],
        "metallic": 0.32,
        "roughness": 0.46,
        "specular_ior_level": 0.30,
    },
    "rubber": {
        "rgba": [0.004, 0.006, 0.009, 1.0],
        "metallic": 0.0,
        "roughness": 0.68,
        "specular_ior_level": 0.18,
    },
    "sensor": {
        "rgba": [0.012, 0.020, 0.032, 1.0],
        "metallic": 0.34,
        "roughness": 0.30,
        "specular_ior_level": 0.30,
    },
}


def _repository_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def _load_plan(path: Path) -> list[dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != "lingtu.sim.robot-selection-render-plan.v1":
        raise ValueError(f"{path}: unsupported render-plan schema")
    robots = payload.get("robots")
    if not isinstance(robots, list) or not robots:
        raise ValueError(f"{path}: robots must be a non-empty array")
    return robots


def _validate_robot(robot: dict[str, Any], source_plan: Path) -> None:
    robot_id = robot.get("id")
    if not isinstance(robot_id, str) or not robot_id:
        raise ValueError(f"{source_plan}: robot has no id")
    geometries = robot.get("geometries")
    if not isinstance(geometries, list) or not geometries:
        raise ValueError(f"{source_plan}: {robot_id} has no geometry")
    for index, geometry in enumerate(geometries):
        geom_type = geometry.get("type")
        if geom_type not in {"mesh", "box", "cylinder", "sphere"}:
            raise ValueError(
                f"{source_plan}: {robot_id} geometry {index} has unsupported type {geom_type!r}"
            )
        if geom_type != "mesh":
            continue
        mesh = Path(geometry.get("mesh", ""))
        if not mesh.is_file():
            raise FileNotFoundError(f"{source_plan}: {robot_id} mesh does not exist: {mesh}")
        if mesh.suffix.lower() not in {".stl", ".obj"}:
            raise ValueError(
                f"{source_plan}: {robot_id} uses unavailable mesh format: {mesh}"
            )


def _write_combined_plan(output_dir: Path) -> Path:
    sources = (AVAILABLE_PLAN, BUILTIN_PLAN)
    robots: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    for source_plan in sources:
        for source_robot in _load_plan(source_plan):
            robot = dict(source_robot)
            _validate_robot(robot, source_plan)
            robot_id = robot["id"]
            if robot_id in seen_ids:
                raise ValueError(f"duplicate robot id across showroom plans: {robot_id}")
            robot["showroom_source_plan"] = _repository_path(source_plan)
            robot["fbx_filename"] = f"SM_{robot_id}.fbx"
            robots.append(robot)
            seen_ids.add(robot_id)
    if len(robots) != EXPECTED_AVAILABLE_ROBOTS:
        raise ValueError(
            f"expected {EXPECTED_AVAILABLE_ROBOTS} available robots, found {len(robots)}"
        )

    unavailable = json.loads(UNAVAILABLE_PLAN.read_text(encoding="utf-8"))
    if not isinstance(unavailable, list):
        raise ValueError(f"{UNAVAILABLE_PLAN}: expected an array")
    payload = {
        "schema": "lingtu.sim.robot-showroom-export-plan.v1",
        "source_plans": [_repository_path(path) for path in sources],
        "unavailable_source": _repository_path(UNAVAILABLE_PLAN),
        "unavailable_robots": unavailable,
        "robots": robots,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    plan_path = output_dir / "export-plan.json"
    plan_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return plan_path


def _custom_material_specs(plan: dict[str, Any]) -> dict[str, dict[str, Any]]:
    specs: dict[str, dict[str, Any]] = {}
    for source_id, source_spec in sorted(plan.get("materials", {}).items()):
        rgba = source_spec.get("base_color_rgba")
        if not isinstance(rgba, list) or len(rgba) != 4:
            raise ValueError(f"{plan['id']}: material {source_id!r} has invalid RGBA")
        specs[source_id] = {
            "rgba": [float(value) for value in rgba],
            "metallic": float(source_spec.get("metallic", 0.0)),
            "roughness": float(source_spec.get("roughness", 0.45)),
            "specular_ior_level": float(source_spec.get("specular_ior_level", 0.5)),
        }
        if "emissive" in source_spec:
            specs[source_id]["emissive"] = source_spec["emissive"]
    return specs


def _build_materials(plan: dict[str, Any]) -> tuple[dict[str, Any], dict[Any, dict[str, Any]]]:
    from render_robot_selection_previews import _blender_material

    materials: dict[str, Any] = {}
    metadata: dict[Any, dict[str, Any]] = {}
    used_source_ids = {geometry.get("material", "silver") for geometry in plan["geometries"]}
    for role, spec in ROLE_MATERIALS.items():
        if role not in used_source_ids:
            continue
        material = _blender_material(
            role,
            tuple(spec["rgba"]),
            spec["metallic"],
            spec["roughness"],
            spec["specular_ior_level"],
        )
        materials[role] = material
        metadata[material] = {
            "role": role,
            "rgba": spec["rgba"],
            "source_material_ids": [role],
        }

    custom_specs = _custom_material_specs(plan)
    custom_ids = [source_id for source_id in sorted(used_source_ids) if source_id in custom_specs]
    for index, source_id in enumerate(custom_ids, start=1):
        spec = custom_specs[source_id]
        name = "customrgba" if len(custom_ids) == 1 else f"customrgba_{index:02d}"
        if "emissive" in spec:
            name = f"emissive_{source_id}"
        material = _blender_material(
            name,
            tuple(spec["rgba"]),
            spec["metallic"],
            spec["roughness"],
            spec["specular_ior_level"],
        )
        if emission := spec.get("emissive"):
            shader = material.node_tree.nodes.get("Principled BSDF")
            shader.inputs["Emission Color"].default_value = (*emission["rgb"], 1.0)
            shader.inputs["Emission Strength"].default_value = emission["strength"]
        materials[source_id] = material
        metadata[material] = {
            "role": "customrgba",
            "rgba": spec["rgba"],
            "source_material_ids": [source_id],
        }
        if "emissive" in spec:
            metadata[material]["emissive"] = spec["emissive"]

    missing = sorted(used_source_ids - materials.keys())
    if missing:
        raise ValueError(f"{plan['id']}: undefined materials: {', '.join(missing)}")
    return materials, metadata


def _consolidate_material_slots(obj: Any) -> None:
    """Collapse repeated references created while joining geometry objects."""

    unique: list[Any] = []
    remap: dict[int, int] = {}
    for old_index, slot in enumerate(obj.material_slots):
        material = slot.material
        if material not in unique:
            unique.append(material)
        remap[old_index] = unique.index(material)
    indices = [remap[polygon.material_index] for polygon in obj.data.polygons]
    obj.data.materials.clear()
    for material in unique:
        obj.data.materials.append(material)
    # Blender clears polygon assignments together with the slot array.
    for polygon, index in zip(obj.data.polygons, indices):
        polygon.material_index = index


def _source_provenance(plan: dict[str, Any]) -> dict[str, Any]:
    mesh_sources = sorted(
        {
            _repository_path(Path(geometry["mesh"]))
            for geometry in plan["geometries"]
            if geometry["type"] == "mesh"
        }
    )
    provenance: dict[str, Any] = {
        "render_plan": plan["showroom_source_plan"],
        "mesh_sources": mesh_sources,
    }
    for key in ("source_urdf", "source_xml", "source_projection", "source_plan", "display_mesh_manifest"):
        if key in plan:
            provenance[key] = plan[key]
    return provenance


def _clean_display_stl(obj: Any) -> dict[str, int]:
    """Remove coincident STL facets in the display copy, never the source package."""
    import math

    import bmesh

    mesh = bmesh.new()
    mesh.from_mesh(obj.data)
    before = len(mesh.faces)
    # STL duplicates vertices per facet. Weld only sub-micron coincidences.
    bmesh.ops.remove_doubles(mesh, verts=list(mesh.verts), dist=1.0e-7)
    mesh.verts.index_update()
    seen = set()
    duplicates = []
    for face in mesh.faces:
        key = tuple(sorted(vertex.index for vertex in face.verts))
        if key in seen:
            duplicates.append(face)
        else:
            seen.add(key)
    if duplicates:
        bmesh.ops.delete(mesh, geom=duplicates, context='FACES_ONLY')
    bmesh.ops.recalc_face_normals(mesh, faces=list(mesh.faces))
    for face in mesh.faces:
        face.smooth = True
    for edge in mesh.edges:
        edge.smooth = edge.is_manifold and edge.calc_face_angle() < math.radians(35)
    result = {'faces_before': before, 'faces_after': len(mesh.faces),
              'boundary_edges': sum(edge.is_boundary for edge in mesh.edges),
              'non_manifold_edges': sum(not edge.is_manifold and not edge.is_boundary for edge in mesh.edges)}
    mesh.to_mesh(obj.data)
    obj.data.update()
    mesh.free()
    return result


def _budget_display_meshes(objects: list[Any], budget: int = 600_000, protected: Sequence[Any] = ()) -> None:
    """Keep small mechanical shells intact; spend reduction on dense parts."""
    import bpy

    small = sum(len(obj.data.polygons) for obj in objects if len(obj.data.polygons) <= 20_000 or obj in protected)
    dense = sum(len(obj.data.polygons) for obj in objects if len(obj.data.polygons) > 20_000 and obj not in protected)
    if small + dense <= budget or dense == 0:
        return
    ratio = max(0.1, (budget - small) / dense)
    for obj in objects:
        if len(obj.data.polygons) <= 20_000 or obj in protected:
            continue
        bpy.context.view_layer.objects.active = obj
        modifier = obj.modifiers.new('ShowroomBudget', 'DECIMATE')
        modifier.ratio = ratio
        bpy.ops.object.modifier_apply(modifier=modifier.name)


def _display_plan(plan: dict[str, Any]) -> tuple[dict[str, Any], set[str]]:
    """Apply authored display assets while preserving the physical render plan."""
    plan = dict(plan)
    plan["geometries"] = [dict(geometry) for geometry in plan["geometries"]]
    protected_paths: set[str] = set()
    if plan["id"] != "thunderv4":
        return plan, protected_paths
    if THUNDER_DISPLAY_MANIFEST.is_file():
        display = json.loads(THUNDER_DISPLAY_MANIFEST.read_text(encoding="utf-8"))
        for geometry in plan["geometries"]:
            if geometry["type"] != "mesh":
                continue
            key = Path(geometry["mesh"]).stem.lower()
            path = THUNDER_DISPLAY_MANIFEST.parent / display["meshes"][key]
            if not path.is_file():
                raise FileNotFoundError(path)
            geometry["mesh"] = str(path)
            protected_paths.add(str(path))
        base = next(g for g in plan["geometries"] if g["name"].startswith("base_link_"))
        for extra in display.get("additional_base_meshes", []):
            path = THUNDER_DISPLAY_MANIFEST.parent / extra["mesh"]
            if not path.is_file():
                raise FileNotFoundError(path)
            geometry = dict(base)
            geometry.update(name=extra["name"], mesh=str(path), material=extra["material"])
            plan["geometries"].append(geometry)
            protected_paths.add(str(path))
        for extra in display.get("additional_link_meshes", []):
            parent = next(g for g in plan["geometries"]
                          if g["type"] == "mesh" and Path(g["mesh"]).stem.lower() == extra["parent_link"])
            path = THUNDER_DISPLAY_MANIFEST.parent / extra["mesh"]
            if not path.is_file():
                raise FileNotFoundError(path)
            geometry = dict(parent)
            geometry.update(name=extra["name"], mesh=str(path), material=extra["material"])
            plan["geometries"].append(geometry)
            protected_paths.add(str(path))
        paint = dict(base)
        paint.update(name="DOSO_side_paint", mesh=str(THUNDER_DISPLAY_MANIFEST.parent / display["paint_mesh"]), material="doso_paint")
        plan["geometries"].append(paint)
        plan["materials"] = {
            **plan.get("materials", {}),
            **display.get("materials", {}),
            "doso_paint": display["paint_material"],
        }
        plan["display_mesh_manifest"] = _repository_path(THUNDER_DISPLAY_MANIFEST)
        plan["display_triangle_budget"] = int(display.get("triangle_budget", 600_000))
        return plan, protected_paths
    for geometry in plan["geometries"]:
        if Path(geometry.get("mesh", "")).name.lower() == "base_link.stl":
            if not THUNDER_DISPLAY_BASE.is_file():
                raise FileNotFoundError(THUNDER_DISPLAY_BASE)
            geometry["mesh"] = str(THUNDER_DISPLAY_BASE)
            protected_paths.add(str(THUNDER_DISPLAY_BASE))
    return plan, protected_paths


def _export_robot(plan: dict[str, Any], output_dir: Path) -> dict[str, Any]:
    import bpy
    from mathutils import Vector
    from render_robot_selection_previews import _create_geometry, _world_bounds

    plan, protected_paths = _display_plan(plan)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.length_unit = "METERS"
    scene.unit_settings.scale_length = 1.0

    materials, material_metadata = _build_materials(plan)
    objects = []
    protected = []
    cleanup = []
    for source_geometry in plan["geometries"]:
        geometry = dict(source_geometry)
        geometry["material"] = geometry.get("material", "silver")
        geometry["preserve_materials"] = False
        parts = _create_geometry(geometry, materials)
        if geometry['type'] == 'mesh' and Path(geometry['mesh']).suffix.lower() == '.stl':
            for part in parts:
                cleanup.append({'geometry': geometry['name'], **_clean_display_stl(part)})
        objects.extend(parts)
        if geometry.get("mesh") in protected_paths:
            protected.extend(parts)
    if not objects:
        raise ValueError(f"{plan['id']}: geometry construction produced no mesh objects")

    source_minimum, source_maximum = _world_bounds(objects)
    authored_triangles = sum(len(obj.data.polygons) for obj in objects)
    triangle_budget = plan.get("display_triangle_budget", 600_000)
    if "display_mesh_manifest" in plan and authored_triangles > triangle_budget:
        raise ValueError(f"{plan['id']}: authored display exceeds {triangle_budget} triangles: {authored_triangles}")
    _budget_display_meshes(objects, budget=triangle_budget, protected=protected)
    bpy.ops.object.select_all(action='DESELECT')
    shift = Vector(
        (
            -(source_minimum.x + source_maximum.x) * 0.5,
            -(source_minimum.y + source_maximum.y) * 0.5,
            -source_minimum.z,
        )
    )
    for obj in objects:
        obj.matrix_world.translation += shift
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bpy.ops.object.join()
    joined = bpy.context.view_layer.objects.active
    joined.name = f"robot_{plan['id']}"
    joined.data.name = f"robot_{plan['id']}_mesh"
    _consolidate_material_slots(joined)

    # Joining keeps the active object's local basis and can introduce a small
    # float offset.  Correct the final joined object, then bake that correction.
    joined_minimum, joined_maximum = _world_bounds([joined])
    join_correction = Vector(
        (
            -(joined_minimum.x + joined_maximum.x) * 0.5,
            -(joined_minimum.y + joined_maximum.y) * 0.5,
            -joined_minimum.z,
        )
    )
    joined.matrix_world.translation += join_correction
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    minimum, maximum = _world_bounds([joined])
    tolerance = 1.0e-4
    if abs((minimum.x + maximum.x) * 0.5) > tolerance:
        raise ValueError(f"{plan['id']}: joined mesh is not centered on X")
    if abs((minimum.y + maximum.y) * 0.5) > tolerance:
        raise ValueError(f"{plan['id']}: joined mesh is not centered on Y")
    if abs(minimum.z) > tolerance:
        raise ValueError(f"{plan['id']}: joined mesh floor is not Z=0")
    if maximum.z - minimum.z <= 0.0:
        raise ValueError(f"{plan['id']}: joined mesh has no positive height")

    bpy.ops.object.select_all(action="DESELECT")
    joined.select_set(True)
    bpy.context.view_layer.objects.active = joined
    # Bake area-and-angle weighted split normals after the final topology and
    # transforms. CAD panels stay planar while their fillets remain smooth;
    # keep_sharp preserves the 35-degree boundaries authored during STL cleanup.
    normals = joined.modifiers.new("ShowroomCADNormals", "WEIGHTED_NORMAL")
    normals.mode = "FACE_AREA_WITH_ANGLE"
    normals.keep_sharp = True
    bpy.ops.object.modifier_apply(modifier=normals.name)
    output_path = output_dir / plan["fbx_filename"]
    result = bpy.ops.export_scene.fbx(
        filepath=str(output_path),
        check_existing=False,
        use_selection=True,
        object_types={"MESH"},
        apply_unit_scale=True,
        apply_scale_options="FBX_SCALE_UNITS",
        global_scale=1.0,
        axis_forward="-Y",
        axis_up="Z",
        bake_space_transform=False,
        mesh_smooth_type="FACE",
        use_tspace=False,
        add_leaf_bones=False,
        path_mode="AUTO",
        bake_anim=False,
    )
    if "FINISHED" not in result or not output_path.is_file():
        raise RuntimeError(f"{plan['id']}: Blender did not produce {output_path}")

    slots = []
    face_counts = [0] * len(joined.material_slots)
    for polygon in joined.data.polygons:
        face_counts[polygon.material_index] += 1
    for slot_index, slot in enumerate(joined.material_slots):
        item = material_metadata[slot.material]
        slots.append(
            {
                "slot": slot_index,
                "material_name": slot.material.name,
                "role": item["role"],
                "rgba": item["rgba"],
                "source_material_ids": item["source_material_ids"],
                "face_count": face_counts[slot_index],
                **({"emissive": item["emissive"]} if "emissive" in item else {}),
            }
        )
    return {
        "id": plan["id"],
        "label": plan.get("label", plan["id"]),
        "asset_filename": output_path.name,
        "object_name": joined.name,
        "real_height_m": float(maximum.z - minimum.z),
        "normalization": {"world_translation_m": [float(v) for v in shift + join_correction]},
        "bbox_m": {
            "min": [float(value) for value in minimum],
            "max": [float(value) for value in maximum],
            "size": [float(maximum[axis] - minimum[axis]) for axis in range(3)],
        },
        "material_slots": slots,
        "source_provenance": _source_provenance(plan),
        "display_mesh_cleanup": cleanup,
        "orientation": {
            "source_coordinates": "right-handed Z-up meters; authored robot yaw is preserved",
            "placement": "world XY bounding-box center at X=0,Y=0; world minimum Z=0",
            "fbx_axis_forward": "-Y",
            "fbx_axis_up": "Z",
            "native_geometry_unit": "m",
            "fbx_scene_unit": "meter (UnitScaleFactor 100 centimeters)",
            "ue_target_unit": "cm",
            "ue_note": "Enable Unreal scene-unit conversion. The FBX declares meter scene units so UE converts natural meter geometry to centimeters; no per-robot yaw normalization was applied.",
        },
    }


def _run_inside_blender(plan_path: Path, output_dir: Path, force: bool = False) -> int:
    import bpy

    payload = json.loads(plan_path.read_text(encoding="utf-8"))
    manifest_path = output_dir / "manifest.json"
    assets = (
        json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest_path.is_file() and not force
        else []
    )
    completed_ids = {
        asset["id"]
        for asset in assets
        if (output_dir / asset["asset_filename"]).is_file()
    }
    for plan in payload["robots"]:
        if plan["id"] in completed_ids:
            print(f"kept existing showroom robot {plan['id']}: {plan['fbx_filename']}")
            continue
        asset = _export_robot(plan, output_dir)
        assets.append(asset)
        manifest_path.write_text(
            json.dumps(assets, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"exported showroom robot {plan['id']}: {asset['asset_filename']}")
    report = {
        "schema": "lingtu.sim.robot-showroom-manifest.v1",
        "generator": {
            "script": _repository_path(Path(__file__)),
            "blender_version": bpy.app.version_string,
        },
        "asset_count": len(assets),
        "joined_static_meshes": True,
        "units": "meters",
        "fbx_axis": {"forward": "-Y", "up": "Z"},
        "available_source_plans": payload["source_plans"],
        "excluded_unavailable": {
            "source": payload["unavailable_source"],
            "robots": payload["unavailable_robots"],
            "policy": "No FBX, plinth, approximation, or substitute was generated for unavailable DAE sources.",
        },
        "manifest_shape": "manifest.json is a top-level array with one object per robot",
    }
    report_path = output_dir / "export-report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"showroom manifest: {manifest_path}")
    print(f"showroom export report: {report_path}")
    return 0


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export 23 available real robots as joined, floor-aligned FBX assets"
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--blender", type=Path, default=DEFAULT_BLENDER)
    parser.add_argument("--force", action="store_true", help="Re-export after geometry or exporter changes")
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
    output_dir = args.output_dir.resolve()
    if args.blender_plan is not None:
        return _run_inside_blender(args.blender_plan.resolve(), output_dir, args.force)

    blender = args.blender.resolve()
    if not blender.is_file():
        raise FileNotFoundError(f"Blender executable not found: {blender}")
    plan_path = _write_combined_plan(output_dir)
    command = [
        str(blender),
        "--background",
        "--factory-startup",
        "--python-exit-code",
        "1",
        "--python",
        str(Path(__file__).resolve()),
        "--",
        "--blender-plan",
        str(plan_path),
        "--output-dir",
        str(output_dir),
    ]
    if args.force:
        command.append("--force")
    subprocess.run(command, cwd=REPOSITORY_ROOT, check=True, timeout=1800)
    manifest_path = output_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, list) or len(manifest) != EXPECTED_AVAILABLE_ROBOTS:
        raise RuntimeError(f"unexpected showroom asset count in {manifest_path}")
    missing = [
        asset["asset_filename"]
        for asset in manifest
        if not (output_dir / asset["asset_filename"]).is_file()
    ]
    if missing:
        raise RuntimeError(f"showroom export is missing FBX files: {', '.join(missing)}")
    print(f"Exported {len(manifest)} showroom robots to {output_dir}")
    print(f"Manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
