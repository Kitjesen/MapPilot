"""Materialize one frozen, visual-only Tripo forest asset in Blender.

This script is intentionally a Blender ``--python`` entry point.  It consumes
only :mod:`forest_asset_conditioning` plans, creates no collision geometry, and
keeps every result quarantined until downstream visual/topology review.
"""

from __future__ import annotations

import argparse
import functools
import hashlib
import json
import math
import os
import shutil
import stat
import sys
import tempfile
from collections.abc import Callable
from pathlib import Path
from typing import Any, ParamSpec, TypeVar

try:  # Keep policy helpers importable by ordinary Python tests.
    import bmesh
    import bpy
    from mathutils import Matrix, Vector
except ModuleNotFoundError:  # pragma: no cover - Blender supplies these modules.
    bmesh = None
    bpy = None
    Matrix = None
    Vector = None

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sim.tools.assets.forest_asset_conditioning import (  # noqa: E402
    CONDITIONER_CONTRACT,
    validate_forest_asset_conditioning_plan,
)
from sim.tools.assets.static_prop_conditioning import (  # noqa: E402
    _assert_directory_identity,
    _BoundDirectoryPublication,
    _open_file_with_parent_identity,
    _TrustedDirectory,
)

REPORT_SCHEMA = "lingtu.sim.forest-asset-conditioning-report.v1"
MAX_SOURCE_GLB_BYTES = 512 * 1024 * 1024
MAX_IMPORTED_OBJECTS = 256
MAX_IMPORTED_MESHES = 128
MAX_IMPORTED_VERTICES = 2_000_000
MAX_IMPORTED_TRIANGLES = 4_000_000
MAX_IMPORTED_MATERIALS = 512
MAX_TEXTURE_PIXELS = 268_435_456
MAX_TOTAL_IMPORT_BUDGET = 300_000_000
MAX_PLAN_BYTES = 2 * 1024 * 1024
_ACTIVE_STAGING: Path | None = None
_ACTIVE_TRUSTED_ROOT: _TrustedDirectory | None = None
_ACTIVE_PUBLICATION: _BoundDirectoryPublication | None = None
_P = ParamSpec("_P")
_R = TypeVar("_R")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _digest_document(value: Any) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _validate_import_budgets(
    *,
    objects: int,
    meshes: int,
    vertices: int,
    triangles: int,
    materials: int,
    texture_pixels: int,
) -> None:
    limits = (
        (objects, MAX_IMPORTED_OBJECTS, "object"),
        (meshes, MAX_IMPORTED_MESHES, "mesh"),
        (vertices, MAX_IMPORTED_VERTICES, "vertex"),
        (triangles, MAX_IMPORTED_TRIANGLES, "triangle"),
        (materials, MAX_IMPORTED_MATERIALS, "material"),
        (texture_pixels, MAX_TEXTURE_PIXELS, "texture pixel"),
    )
    for observed, maximum, label in limits:
        if observed < 0 or observed > maximum:
            raise RuntimeError(f"imported forest exceeds the {label} budget")
    total = objects + meshes + vertices + triangles + materials + texture_pixels
    if total > MAX_TOTAL_IMPORT_BUDGET:
        raise RuntimeError("imported forest exceeds the total resource budget")


def _enforce_scene_import_budgets(imported_objects: list[Any]) -> None:
    meshes = [obj for obj in imported_objects if obj.type == "MESH"]
    vertices = sum(len(obj.data.vertices) for obj in meshes)
    triangles = 0
    materials: set[Any] = set()
    for obj in meshes:
        obj.data.calc_loop_triangles()
        triangles += len(obj.data.loop_triangles)
        materials.update(slot.material for slot in obj.material_slots if slot.material is not None)
    texture_pixels = sum(
        int(image.size[0]) * int(image.size[1])
        for image in bpy.data.images
        if image.type == "IMAGE" and image.name not in {"Render Result", "Viewer Node"}
    )
    _validate_import_budgets(
        objects=len(imported_objects),
        meshes=len(meshes),
        vertices=vertices,
        triangles=triangles,
        materials=len(materials),
        texture_pixels=texture_pixels,
    )


def _snapshot_source(source: Path, record: dict[str, Any], staging_root: Path) -> Path:
    candidate = Path(os.path.abspath(os.fspath(source)))
    current = Path(candidate.anchor)
    reparse_flag = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    for component in candidate.parts[1:]:
        current /= component
        metadata = os.lstat(current)
        if stat.S_ISLNK(metadata.st_mode) or (
            reparse_flag and getattr(metadata, "st_file_attributes", 0) & reparse_flag
        ):
            raise RuntimeError("source model path must be link-free")
    descriptor, _parent_identity = _open_file_with_parent_identity(candidate)
    snapshot = staging_root / "source.glb"
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise RuntimeError("source model must be one regular file")
        if metadata.st_size > MAX_SOURCE_GLB_BYTES:
            raise RuntimeError("source model exceeds the physical byte budget")
        digest = hashlib.sha256()
        copied = 0
        with snapshot.open("xb") as target:
            while True:
                block = os.read(descriptor, 1024 * 1024)
                if not block:
                    break
                copied += len(block)
                if copied > MAX_SOURCE_GLB_BYTES:
                    raise RuntimeError("source model exceeds the physical byte budget")
                digest.update(block)
                target.write(block)
            target.flush()
            os.fsync(target.fileno())
        after = os.fstat(descriptor)
        if (
            copied != metadata.st_size
            or copied != record["bytes"]
            or digest.hexdigest() != record["sha256"]
            or (metadata.st_dev, metadata.st_ino) != (after.st_dev, after.st_ino)
        ):
            raise RuntimeError("source model identity changed after the forest plan was frozen")
        return snapshot
    finally:
        os.close(descriptor)


def _read_plan(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    candidate = Path(os.path.abspath(os.fspath(path)))
    current = Path(candidate.anchor)
    reparse_flag = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    for component in candidate.parts[1:]:
        current /= component
        metadata = os.lstat(current)
        if stat.S_ISLNK(metadata.st_mode) or (
            reparse_flag and getattr(metadata, "st_file_attributes", 0) & reparse_flag
        ):
            raise RuntimeError("conditioning plan path must be link-free")
    descriptor, parent_identity = _open_file_with_parent_identity(candidate)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_size > MAX_PLAN_BYTES:
            raise RuntimeError("conditioning plan exceeds the physical byte budget")
        payload = bytearray()
        while True:
            block = os.read(descriptor, 64 * 1024)
            if not block:
                break
            payload.extend(block)
            if len(payload) > MAX_PLAN_BYTES:
                raise RuntimeError("conditioning plan exceeds the physical byte budget")
        document = json.loads(payload.decode("utf-8"))
        if not isinstance(document, dict):
            raise RuntimeError("conditioning plan must be one JSON object")
        return document, {
            "bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "parent_identity": list(parent_identity),
        }
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot read forest conditioning plan: {exc}") from exc
    finally:
        os.close(descriptor)


def _effective_decimation_ratio(
    source_triangles: int, triangle_ratio: float, max_triangles: int
) -> float:
    """Return a conservative Blender ratio satisfying both frozen limits."""

    if source_triangles <= 0 or max_triangles <= 0:
        raise ValueError("triangle counts must be positive")
    if not math.isfinite(triangle_ratio) or not 0.0 < triangle_ratio <= 1.0:
        raise ValueError("triangle_ratio must be finite and in (0, 1]")
    return min(triangle_ratio, max_triangles / source_triangles)


def _assert_visual_only_plan(plan: dict[str, Any]) -> None:
    """Fail closed if the validated binding ever gains physical authority."""

    binding = plan["binding"]
    unreal = binding["unreal"]
    if (
        plan["conditioner_contract"] != CONDITIONER_CONTRACT
        or binding["physics_authority"] != "mujoco_world_proxy"
        or binding["visual_mesh_is_physics_proxy"] is not False
        or unreal["collision_profile"] != "NoCollision"
        or any(
            unreal[field] is not False
            for field in (
                "collision_enabled",
                "simulate_physics",
                "generate_overlap_events",
                "can_ever_affect_navigation",
            )
        )
        or plan["qualification"]["state"] != "QUARANTINED"
    ):
        raise RuntimeError("forest conditioner accepts only QUARANTINED visual-only plans")


def _artifact_path(root: Path, relative: str, context: str) -> Path:
    canonical_root = Path(os.path.abspath(os.fspath(root)))
    candidate = Path(os.path.abspath(os.fspath(canonical_root / relative)))
    try:
        candidate.relative_to(canonical_root)
    except ValueError as exc:
        raise RuntimeError(f"{context} escapes the artifact root") from exc
    current = Path(candidate.anchor)
    reparse_flag = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    for component in candidate.parts[1:]:
        current /= component
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(metadata.st_mode) or (
            reparse_flag and getattr(metadata, "st_file_attributes", 0) & reparse_flag
        ):
            raise RuntimeError(f"{context} must remain link-free")
    return candidate


def _require_blender() -> None:
    if bpy is None or bmesh is None or Matrix is None or Vector is None:
        raise RuntimeError("forest asset conditioning must run inside Blender")


def _bounds(obj: Any) -> tuple[Any, Any]:
    points = [obj.matrix_world @ vertex.co for vertex in obj.data.vertices]
    if not points:
        raise RuntimeError(f"mesh {obj.name} contains no vertices")
    minimum = Vector(tuple(min(point[index] for point in points) for index in range(3)))
    maximum = Vector(tuple(max(point[index] for point in points) for index in range(3)))
    return minimum, maximum


def _dimensions(obj: Any) -> list[float]:
    minimum, maximum = _bounds(obj)
    return [float(maximum[index] - minimum[index]) for index in range(3)]


def _triangle_count(obj: Any) -> int:
    obj.data.calc_loop_triangles()
    return len(obj.data.loop_triangles)


def _topology_report(obj: Any) -> dict[str, Any]:
    editable = bmesh.new()
    editable.from_mesh(obj.data)
    original_vertices = len(editable.verts)
    bmesh.ops.remove_doubles(editable, verts=list(editable.verts), dist=1.0e-6)
    report = {
        "analysis_basis": "position_welded_copy",
        "position_weld_distance_m": 1.0e-6,
        "source_vertices": original_vertices,
        "vertices": len(editable.verts),
        "welded_vertices_removed": original_vertices - len(editable.verts),
        "edges": len(editable.edges),
        "faces": len(editable.faces),
        "boundary_edges": sum(edge.is_boundary for edge in editable.edges),
        "non_manifold_edges": sum(not edge.is_manifold for edge in editable.edges),
        "wire_edges": sum(edge.is_wire for edge in editable.edges),
    }
    editable.free()
    return report


def _clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)


def _join_imported_meshes(objects: list[Any], asset_id: str) -> Any:
    if not objects:
        raise RuntimeError("Tripo GLB imported no mesh objects")
    for obj in objects:
        obj.data.transform(obj.matrix_world)
        obj.matrix_world = Matrix.Identity(4)
        obj.parent = None
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    if len(objects) > 1 and "FINISHED" not in bpy.ops.object.join():
        raise RuntimeError("Blender could not join the Tripo mesh objects")
    joined = bpy.context.view_layer.objects.active
    if joined is None or joined.type != "MESH":
        raise RuntimeError("Blender did not produce one joined forest mesh")
    joined.name = asset_id
    joined.data.name = f"{asset_id}_Mesh"
    return joined


def _recalculate_normals_and_smooth(obj: Any) -> None:
    editable = bmesh.new()
    editable.from_mesh(obj.data)
    bmesh.ops.recalc_face_normals(editable, faces=editable.faces)
    for face in editable.faces:
        face.smooth = True
    editable.to_mesh(obj.data)
    editable.free()
    obj.data.validate(clean_customdata=False)
    obj.data.update(calc_edges=True)


def _normalize(
    obj: Any, target_dimensions: list[float]
) -> tuple[list[float], list[float], list[float]]:
    source_dimensions = _dimensions(obj)
    if any(value <= 1.0e-9 for value in source_dimensions):
        raise RuntimeError("generated forest mesh has a degenerate AABB")
    scale = [
        target / source for target, source in zip(target_dimensions, source_dimensions)
    ]
    if any(not math.isfinite(value) or value <= 0.0 for value in scale):
        raise RuntimeError("generated forest mesh requires an invalid exact-axis scale")
    obj.data.transform(
        Matrix(
            (
                (scale[0], 0.0, 0.0, 0.0),
                (0.0, scale[1], 0.0, 0.0),
                (0.0, 0.0, scale[2], 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )
        )
    )
    minimum, maximum = _bounds(obj)
    obj.data.transform(
        Matrix.Translation(
            Vector((-(minimum.x + maximum.x) * 0.5, -(minimum.y + maximum.y) * 0.5, -minimum.z))
        )
    )
    obj.data.update()
    _recalculate_normals_and_smooth(obj)
    return source_dimensions, _dimensions(obj), [float(value) for value in scale]


def _make_lods(source: Any, asset_id: str, specs: list[dict[str, Any]]) -> list[Any]:
    source_triangles = _triangle_count(source)
    lods: list[Any] = []
    for spec in specs:
        duplicate = source.copy()
        duplicate.data = source.data.copy()
        bpy.context.scene.collection.objects.link(duplicate)
        duplicate.name = f"{asset_id}_{spec['name']}"
        duplicate.data.name = f"{duplicate.name}_Mesh"
        bpy.ops.object.select_all(action="DESELECT")
        duplicate.select_set(True)
        bpy.context.view_layer.objects.active = duplicate
        modifier = duplicate.modifiers.new(name="LingTuForestLODDecimate", type="DECIMATE")
        modifier.decimate_type = "COLLAPSE"
        modifier.ratio = _effective_decimation_ratio(
            source_triangles, float(spec["triangle_ratio"]), int(spec["max_triangles"])
        )
        modifier.use_collapse_triangulate = True
        if "FINISHED" not in bpy.ops.object.modifier_apply(modifier=modifier.name):
            raise RuntimeError(f"Blender could not materialize {spec['name']}")
        _recalculate_normals_and_smooth(duplicate)
        triangles = _triangle_count(duplicate)
        if triangles > spec["max_triangles"]:
            raise RuntimeError(
                f"{spec['name']} exceeds its triangle budget: {triangles} > {spec['max_triangles']}"
            )
        duplicate["lingtu_asset_role"] = "VisualOnly"
        duplicate["lingtu_collision_profile"] = "NoCollision"
        duplicate["lingtu_physics_proxy"] = False
        duplicate["lingtu_qualification"] = "QUARANTINED"
        duplicate["lingtu_lod"] = spec["name"]
        lods.append(duplicate)
    bpy.data.objects.remove(source, do_unlink=True)
    return lods


def _export_lod(obj: Any, root: Path, output: dict[str, str]) -> dict[str, Any]:
    glb_path = _artifact_path(root, output["glb"], "LOD GLB output")
    fbx_path = _artifact_path(root, output["fbx"], "LOD FBX output")
    glb_path.parent.mkdir(parents=True, exist_ok=True)
    glb_path.unlink(missing_ok=True)
    fbx_path.unlink(missing_ok=True)
    bpy.ops.object.select_all(action="DESELECT")
    obj.hide_set(False)
    obj.hide_render = False
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    glb_result = bpy.ops.export_scene.gltf(
        filepath=str(glb_path),
        export_format="GLB",
        use_selection=True,
        export_apply=True,
        export_yup=True,
        export_materials="EXPORT",
    )
    if "FINISHED" not in glb_result or not glb_path.is_file():
        raise RuntimeError(f"Blender did not produce {glb_path.name}")
    fbx_result = bpy.ops.export_scene.fbx(
        filepath=str(fbx_path),
        check_existing=False,
        use_selection=True,
        object_types={"MESH"},
        apply_unit_scale=True,
        apply_scale_options="FBX_SCALE_UNITS",
        axis_forward="-Y",
        axis_up="Z",
        mesh_smooth_type="FACE",
        path_mode="COPY",
        embed_textures=True,
        add_leaf_bones=False,
    )
    if "FINISHED" not in fbx_result or not fbx_path.is_file():
        raise RuntimeError(f"Blender did not produce {fbx_path.name}")
    return {
        "name": output["name"],
        "triangles": _triangle_count(obj),
        "glb": {"path": output["glb"], "bytes": glb_path.stat().st_size, "sha256": _sha256(glb_path)},
        "fbx": {"path": output["fbx"], "bytes": fbx_path.stat().st_size, "sha256": _sha256(fbx_path)},
    }


def _point_at(obj: Any, target: Any) -> None:
    obj.rotation_euler = (target - obj.location).to_track_quat("-Z", "Y").to_euler()


def _render_neutral_preview(lods: list[Any], path: Path) -> None:
    scene = bpy.context.scene
    path.unlink(missing_ok=True)
    try:
        scene.render.engine = "BLENDER_EEVEE_NEXT"
    except TypeError:
        scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 1280
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(path)
    scene.world.color = (0.18, 0.18, 0.18)
    for obj in lods:
        obj.hide_render = obj is not lods[0]
        obj.hide_set(obj is not lods[0])
    minimum, maximum = _bounds(lods[0])
    center = (minimum + maximum) * 0.5
    extent = max(float(value) for value in maximum - minimum)
    temporary = []
    camera_data = bpy.data.cameras.new("ForestPreviewCameraData")
    camera = bpy.data.objects.new("ForestPreviewCamera", camera_data)
    bpy.context.collection.objects.link(camera)
    camera.location = center + Vector((extent * 2.4, -extent * 3.2, extent * 1.8))
    camera.data.lens = 58
    _point_at(camera, center)
    scene.camera = camera
    temporary.append(camera)
    for name, offset, energy in (("Key", (-1.5, -1.5, 2.5), 500.0), ("Fill", (1.5, -0.8, 1.5), 250.0)):
        data = bpy.data.lights.new(f"ForestPreview{name}Data", "AREA")
        data.energy = energy * extent * extent
        data.size = max(1.0, extent * 0.8)
        light = bpy.data.objects.new(f"ForestPreview{name}", data)
        light.location = center + Vector(offset) * extent
        _point_at(light, center)
        bpy.context.collection.objects.link(light)
        temporary.append(light)
    result = bpy.ops.render.render(write_still=True)
    if "FINISHED" not in result or not path.is_file():
        raise RuntimeError("Blender did not render the neutral forest preview")
    for obj in temporary:
        bpy.data.objects.remove(obj, do_unlink=True)
    scene.camera = None
    for obj in lods:
        obj.hide_set(False)
        obj.hide_render = False


def _clean_staging_on_failure(function: Callable[_P, _R]) -> Callable[_P, _R]:
    @functools.wraps(function)
    def guarded(*args: _P.args, **kwargs: _P.kwargs) -> _R:
        global _ACTIVE_PUBLICATION, _ACTIVE_STAGING, _ACTIVE_TRUSTED_ROOT
        try:
            return function(*args, **kwargs)
        except BaseException:
            if _ACTIVE_STAGING is not None:
                _remove_staging_safely(_ACTIVE_STAGING, _ACTIVE_TRUSTED_ROOT)
                _ACTIVE_STAGING = None
            if _ACTIVE_PUBLICATION is not None:
                _ACTIVE_PUBLICATION.close()
                _ACTIVE_PUBLICATION = None
            if _ACTIVE_TRUSTED_ROOT is not None:
                _ACTIVE_TRUSTED_ROOT.close()
                _ACTIVE_TRUSTED_ROOT = None
            raise

    return guarded


def _remove_staging_safely(
    staging: Path, trusted_root: _TrustedDirectory | None
) -> None:
    if trusted_root is not None:
        try:
            _assert_directory_identity(trusted_root.path, trusted_root.identity)
        except (OSError, RuntimeError, ValueError):
            return
    shutil.rmtree(staging, ignore_errors=True)


@_clean_staging_on_failure
def _condition_impl(
    plan_path: Path, *, expected_plan_bytes: int | None = None, expected_plan_sha256: str | None = None
) -> Path:
    """Materialize and report one validated forest conditioning plan."""

    _require_blender()
    plan_path = Path(os.path.abspath(os.fspath(plan_path)))
    raw, plan_evidence = _read_plan(plan_path)
    if (
        expected_plan_bytes is not None
        and plan_evidence["bytes"] != expected_plan_bytes
        or expected_plan_sha256 is not None
        and plan_evidence["sha256"] != expected_plan_sha256
    ):
        raise RuntimeError("forest conditioning plan identity changed after command construction")
    plan = validate_forest_asset_conditioning_plan(raw)
    _assert_visual_only_plan(plan)
    root = plan_path.parent
    global _ACTIVE_PUBLICATION, _ACTIVE_TRUSTED_ROOT
    trusted_parent = tuple(plan_evidence["parent_identity"])
    trusted_root = _TrustedDirectory(root, trusted_parent)
    _ACTIVE_TRUSTED_ROOT = trusted_root
    _assert_directory_identity(root, trusted_parent)
    source_record = plan["source"]["model"]
    source_path = _artifact_path(root, source_record["path"], "source model")
    output_dir = _artifact_path(root, plan["outputs"]["directory"], "output directory")
    if output_dir.exists():
        raise RuntimeError("conditioned output directory already exists")
    global _ACTIVE_STAGING
    _assert_directory_identity(root, trusted_parent)
    staging_root = Path(tempfile.mkdtemp(prefix=".lingtu-forest-conditioning-", dir=root))
    _ACTIVE_STAGING = staging_root
    staged_output = _artifact_path(
        staging_root, plan["outputs"]["directory"], "staged output directory"
    )
    staged_output.mkdir(parents=True)
    snapshot = _snapshot_source(source_path, source_record, staging_root)
    _clear_scene()
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0
    before = set(bpy.data.objects)
    if "FINISHED" not in bpy.ops.import_scene.gltf(filepath=str(snapshot)):
        raise RuntimeError("Blender could not import the Tripo forest GLB")
    all_imported = [obj for obj in bpy.data.objects if obj not in before]
    _enforce_scene_import_budgets(all_imported)
    imported = sorted(
        (obj for obj in all_imported if obj.type == "MESH"),
        key=lambda obj: obj.name,
    )
    joined = _join_imported_meshes(imported, plan["asset"]["id"])
    if not any(slot.material is not None for slot in joined.material_slots):
        raise RuntimeError("Tripo forest GLB contains no PBR material slots")
    source_dimensions, normalized_dimensions, axis_scale = _normalize(
        joined, plan["geometry"]["target_dimensions_m"]
    )
    topology = _topology_report(joined)
    source_triangles = _triangle_count(joined)
    lods = _make_lods(joined, plan["asset"]["id"], plan["lods"])
    bpy.ops.file.pack_all()
    outputs = [
        _export_lod(obj, staging_root, specification)
        for obj, specification in zip(lods, plan["outputs"]["lods"])
    ]
    preview_path = _artifact_path(staging_root, plan["outputs"]["preview"], "preview output")
    _render_neutral_preview(lods, preview_path)
    blend_path = _artifact_path(staging_root, plan["outputs"]["blend"], "Blend output")
    blend_path.unlink(missing_ok=True)
    if "FINISHED" not in bpy.ops.wm.save_as_mainfile(filepath=str(blend_path)) or not blend_path.is_file():
        raise RuntimeError("Blender did not save the forest source scene")

    images = [
        {"name": image.name, "packed": bool(image.packed_file), "size_px": [int(image.size[0]), int(image.size[1])]}
        for image in sorted(bpy.data.images, key=lambda item: item.name)
        if image.type == "IMAGE" and image.name not in {"Render Result", "Viewer Node"}
    ]
    if not images or not all(record["packed"] for record in images):
        raise RuntimeError("conditioned forest PBR textures were not embedded in the Blend")
    body = {
        "schema": REPORT_SCHEMA,
        "conditioner_contract": CONDITIONER_CONTRACT,
        "asset": plan["asset"],
        "plan_digest": plan["digest"],
        "plan": plan_evidence,
        "source": plan["source"],
        "conditioner": {
            "contract": CONDITIONER_CONTRACT,
            "script_sha256": _sha256(Path(__file__).resolve()),
        },
        "coordinate_system": {"forward": "+X", "up": "+Z", "unit": "meter"},
        "geometry": {
            "source_dimensions": source_dimensions,
            "normalized_dimensions_m": normalized_dimensions,
            "target_dimensions_m": plan["geometry"]["target_dimensions_m"],
            "axis_scale": axis_scale,
            "placement": "center_xy_ground_z",
            "normal_policy": "recalculate_consistent_outside_smooth",
        },
        "topology": {**topology, "source_triangles": source_triangles},
        "textures": images,
        "lods": outputs,
        "preview": {
            "path": plan["outputs"]["preview"],
            "bytes": preview_path.stat().st_size,
            "sha256": _sha256(preview_path),
        },
        "blend": {
            "path": plan["outputs"]["blend"],
            "bytes": blend_path.stat().st_size,
            "sha256": _sha256(blend_path),
        },
        "metadata": {
            "asset_role": "VisualOnly",
            "collision_profile": "NoCollision",
            "physics_proxy": False,
        },
        "binding": plan["binding"],
        "qualification": plan["qualification"],
    }
    report = {**body, "digest": _digest_document(body)}
    report_path = _artifact_path(staging_root, plan["outputs"]["inspection"], "inspection output")
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=staged_output, prefix=".report-", suffix=".json", delete=False
    ) as stream:
        stream.write(json.dumps(report, sort_keys=True, indent=2, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
        temporary = Path(stream.name)
    os.replace(temporary, report_path)
    publication = trusted_root.bind_publication(staged_output)
    _ACTIVE_PUBLICATION = publication
    _assert_directory_identity(root, trusted_parent)
    publication.publish(plan["outputs"]["directory"])
    publication.close()
    _ACTIVE_PUBLICATION = None
    trusted_root.close()
    _ACTIVE_TRUSTED_ROOT = None
    _remove_staging_safely(staging_root, trusted_root)
    _ACTIVE_STAGING = None
    report_path = output_dir / report_path.name
    print(f"LINGTU_FOREST_ASSET_CONDITIONED {report_path}")
    return report_path


def condition(
    plan_path: Path, *, expected_plan_bytes: int | None = None, expected_plan_sha256: str | None = None
) -> Path:
    """Materialize one forest plan and clean private staging on every exit path."""

    global _ACTIVE_PUBLICATION, _ACTIVE_STAGING, _ACTIVE_TRUSTED_ROOT
    try:
        return _condition_impl(
            plan_path,
            expected_plan_bytes=expected_plan_bytes,
            expected_plan_sha256=expected_plan_sha256,
        )
    finally:
        if _ACTIVE_STAGING is not None:
            _remove_staging_safely(_ACTIVE_STAGING, _ACTIVE_TRUSTED_ROOT)
            _ACTIVE_STAGING = None
        if _ACTIVE_PUBLICATION is not None:
            _ACTIVE_PUBLICATION.close()
            _ACTIVE_PUBLICATION = None
        if _ACTIVE_TRUSTED_ROOT is not None:
            _ACTIVE_TRUSTED_ROOT.close()
            _ACTIVE_TRUSTED_ROOT = None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Condition one visual-only forest asset in Blender")
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--plan-bytes", required=True, type=int)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--conditioner-contract", required=True)
    parser.add_argument("--script-sha256", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Blender script entry point."""

    if argv is None:
        argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    args = _parser().parse_args(argv)
    if args.conditioner_contract != CONDITIONER_CONTRACT or args.script_sha256 != _sha256(
        Path(__file__).resolve()
    ):
        raise RuntimeError("forest conditioner script contract or SHA-256 mismatch")
    condition(
        args.plan,
        expected_plan_bytes=args.plan_bytes,
        expected_plan_sha256=args.plan_sha256,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "CONDITIONER_CONTRACT",
    "REPORT_SCHEMA",
    "condition",
    "main",
]
