"""Condition one generated static prop inside headless Blender.

Run this file through Blender's ``--python`` option.  The input plan is
validated by :mod:`sim.tools.assets.static_prop_conditioning`; this script
only materializes the frozen visual build and never authors collision.
"""

from __future__ import annotations

import argparse
import functools
import hashlib
import json
import math
import os
import re
import shutil
import stat
import sys
import tempfile
from collections.abc import Callable
from pathlib import Path
from typing import Any, ParamSpec, TypeVar

try:  # Keep safety helpers importable by ordinary Python tests.
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

from sim.tools.assets.static_prop_conditioning import (  # noqa: E402
    CONDITIONER_CONTRACT,
    _assert_directory_identity,
    _BoundDirectoryPublication,
    _open_file_with_parent_identity,
    _TrustedDirectory,
    topology_requires_review,
    validate_static_prop_conditioning_plan,
)

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
    payload = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, prefix=".report-", suffix=".json", delete=False
    ) as stream:
        stream.write(json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        ) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
        temporary = Path(stream.name)
    os.replace(temporary, path)


def _validate_import_budgets(
    *, objects: int, meshes: int, vertices: int, triangles: int,
    materials: int, texture_pixels: int,
) -> None:
    for observed, maximum, label in (
        (objects, MAX_IMPORTED_OBJECTS, "object"),
        (meshes, MAX_IMPORTED_MESHES, "mesh"),
        (vertices, MAX_IMPORTED_VERTICES, "vertex"),
        (triangles, MAX_IMPORTED_TRIANGLES, "triangle"),
        (materials, MAX_IMPORTED_MATERIALS, "material"),
        (texture_pixels, MAX_TEXTURE_PIXELS, "texture pixel"),
    ):
        if observed < 0 or observed > maximum:
            raise RuntimeError(f"imported static prop exceeds the {label} budget")
    total = objects + meshes + vertices + triangles + materials + texture_pixels
    if total > MAX_TOTAL_IMPORT_BUDGET:
        raise RuntimeError("imported static prop exceeds the total resource budget")


def _enforce_scene_import_budgets(imported_objects: list[Any]) -> None:
    meshes = [obj for obj in imported_objects if obj.type == "MESH"]
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
        objects=len(imported_objects), meshes=len(meshes),
        vertices=sum(len(obj.data.vertices) for obj in meshes), triangles=triangles,
        materials=len(materials), texture_pixels=texture_pixels,
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
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_size > MAX_SOURCE_GLB_BYTES:
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
            copied != metadata.st_size or copied != record["bytes"]
            or digest.hexdigest() != record["sha256"]
            or (metadata.st_dev, metadata.st_ino) != (after.st_dev, after.st_ino)
        ):
            raise RuntimeError("source model identity changed after the plan was frozen")
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
        raise RuntimeError(f"cannot read conditioning plan: {exc}") from exc
    finally:
        os.close(descriptor)


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
        raise RuntimeError("static prop conditioning must run inside Blender")


def _bounds(obj: bpy.types.Object) -> tuple[Vector, Vector]:
    points = [obj.matrix_world @ vertex.co for vertex in obj.data.vertices]
    if not points:
        raise RuntimeError(f"mesh {obj.name} contains no vertices")
    minimum = Vector(tuple(min(point[index] for point in points) for index in range(3)))
    maximum = Vector(tuple(max(point[index] for point in points) for index in range(3)))
    return minimum, maximum


def _dimensions(obj: bpy.types.Object) -> list[float]:
    minimum, maximum = _bounds(obj)
    return [float(maximum[index] - minimum[index]) for index in range(3)]


def _triangle_count(obj: bpy.types.Object) -> int:
    obj.data.calc_loop_triangles()
    return len(obj.data.loop_triangles)


def _topology_report(obj: bpy.types.Object) -> dict[str, Any]:
    editable = bmesh.new()
    editable.from_mesh(obj.data)
    source_vertices = len(editable.verts)
    source_edges = len(editable.edges)
    source_faces = len(editable.faces)
    weld_distance_m = 1.0e-6
    bmesh.ops.remove_doubles(
        editable,
        verts=list(editable.verts),
        dist=weld_distance_m,
    )
    report = {
        "analysis_basis": "position_welded_copy",
        "position_weld_distance_m": weld_distance_m,
        "source_vertices": source_vertices,
        "source_edges": source_edges,
        "source_faces": source_faces,
        "vertices": len(editable.verts),
        "welded_vertices_removed": source_vertices - len(editable.verts),
        "edges": len(editable.edges),
        "faces": len(editable.faces),
        "boundary_edges": sum(1 for edge in editable.edges if edge.is_boundary),
        "non_manifold_edges": sum(1 for edge in editable.edges if not edge.is_manifold),
        "wire_edges": sum(1 for edge in editable.edges if edge.is_wire),
    }
    editable.free()
    return report


def _clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for data in tuple(bpy.data.meshes):
        if data.users == 0:
            bpy.data.meshes.remove(data)


def _apply_world_transforms(objects: list[bpy.types.Object]) -> None:
    for obj in objects:
        obj.data.transform(obj.matrix_world)
        obj.matrix_world = Matrix.Identity(4)
        obj.parent = None


def _join_meshes(objects: list[bpy.types.Object], name: str) -> bpy.types.Object:
    if not objects:
        raise RuntimeError("Tripo GLB imported no mesh objects")
    _apply_world_transforms(objects)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    if len(objects) > 1:
        result = bpy.ops.object.join()
        if "FINISHED" not in result:
            raise RuntimeError("Blender could not join the generated mesh objects")
    joined = bpy.context.view_layer.objects.active
    if joined is None or joined.type != "MESH":
        raise RuntimeError("Blender did not produce one joined static mesh")
    joined.name = name
    joined.data.name = f"{name}_Mesh"
    return joined


def _axis_matrix(source_axis_order: list[str]) -> Matrix:
    source_index = {"x": 0, "y": 1, "z": 2}
    rows = []
    for axis in source_axis_order:
        row = [0.0, 0.0, 0.0, 0.0]
        row[source_index[axis]] = 1.0
        rows.append(tuple(row))
    rows.append((0.0, 0.0, 0.0, 1.0))
    return Matrix(tuple(rows))


def _recalculate_normals(obj: bpy.types.Object) -> None:
    editable = bmesh.new()
    editable.from_mesh(obj.data)
    bmesh.ops.recalc_face_normals(editable, faces=editable.faces)
    for face in editable.faces:
        face.smooth = True
    editable.to_mesh(obj.data)
    editable.free()
    obj.data.validate(clean_customdata=False)
    obj.data.update(calc_edges=True)


def _normalize_geometry(
    obj: bpy.types.Object,
    *,
    source_axis_order: list[str],
    target_dimensions: list[float],
) -> tuple[list[float], list[float], float]:
    source_dimensions = _dimensions(obj)
    obj.data.transform(_axis_matrix(source_axis_order))
    obj.data.update()
    mapped_dimensions = _dimensions(obj)
    if any(dimension <= 1.0e-9 for dimension in mapped_dimensions):
        raise RuntimeError("generated static mesh has a degenerate AABB")
    scale = min(
        target / observed
        for target, observed in zip(target_dimensions, mapped_dimensions)
    )
    if not math.isfinite(scale) or scale <= 0.0:
        raise RuntimeError("generated static mesh requires an invalid normalization scale")
    obj.data.transform(Matrix.Scale(scale, 4))
    obj.data.update()
    minimum, maximum = _bounds(obj)
    offset = Vector(
        (
            -(minimum.x + maximum.x) * 0.5,
            -(minimum.y + maximum.y) * 0.5,
            -minimum.z,
        )
    )
    obj.data.transform(Matrix.Translation(offset))
    obj.data.update()
    _recalculate_normals(obj)
    normalized_dimensions = _dimensions(obj)
    return source_dimensions, normalized_dimensions, float(scale)


def _make_lods(
    source: bpy.types.Object,
    asset_id: str,
    lod_specs: list[dict[str, Any]],
) -> list[bpy.types.Object]:
    source.name = f"{asset_id}_{lod_specs[0]['name']}"
    source.data.name = f"{source.name}_Mesh"
    lods = [source]
    for specification in lod_specs[1:]:
        duplicate = source.copy()
        duplicate.data = source.data.copy()
        bpy.context.scene.collection.objects.link(duplicate)
        duplicate.name = f"{asset_id}_{specification['name']}"
        duplicate.data.name = f"{duplicate.name}_Mesh"
        bpy.context.view_layer.objects.active = duplicate
        bpy.ops.object.select_all(action="DESELECT")
        duplicate.select_set(True)
        modifier = duplicate.modifiers.new(name="LingTuLODDecimate", type="DECIMATE")
        modifier.decimate_type = "COLLAPSE"
        modifier.ratio = float(specification["triangle_ratio"])
        modifier.use_collapse_triangulate = True
        result = bpy.ops.object.modifier_apply(modifier=modifier.name)
        if "FINISHED" not in result:
            raise RuntimeError(f"Blender could not materialize {specification['name']}")
        _recalculate_normals(duplicate)
        lods.append(duplicate)
    for index, obj in enumerate(lods):
        obj["lingtu_visual_only"] = True
        obj["lingtu_collision_profile"] = "NoCollision"
        obj["lingtu_lod"] = lod_specs[index]["name"]
    return lods


def _save_textures(output_dir: Path) -> list[dict[str, Any]]:
    texture_dir = output_dir / "textures"
    texture_dir.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    images = sorted(
        (
            image
            for image in bpy.data.images
            if image.type == "IMAGE"
            and image.name not in {"Render Result", "Viewer Node"}
            and image.size[0] > 0
            and image.size[1] > 0
        ),
        key=lambda image: image.name,
    )
    for index, image in enumerate(images):
        safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", image.name).strip("._") or "texture"
        target = texture_dir / f"{index:02d}-{safe_name}.png"
        previous_path = image.filepath_raw
        previous_format = image.file_format
        try:
            image.filepath_raw = str(target)
            image.file_format = "PNG"
            image.save()
        finally:
            image.filepath_raw = previous_path
            image.file_format = previous_format
        if not target.is_file():
            raise RuntimeError(f"Blender did not extract texture {image.name}")
        records.append(
            {
                "name": image.name,
                "path": target.relative_to(output_dir.parent).as_posix(),
                "size_px": [int(image.size[0]), int(image.size[1])],
                "bytes": target.stat().st_size,
                "sha256": _sha256(target),
            }
        )
    if not records:
        raise RuntimeError("conditioned PBR asset contains no extractable textures")
    return records


def _export_lod(
    obj: bpy.types.Object,
    *,
    root: Path,
    output: dict[str, str],
) -> dict[str, Any]:
    glb_path = _artifact_path(root, output["glb"], "LOD GLB output")
    fbx_path = _artifact_path(root, output["fbx"], "LOD FBX output")
    glb_path.parent.mkdir(parents=True, exist_ok=True)
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
        bake_space_transform=False,
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
        "glb": {
            "path": output["glb"],
            "bytes": glb_path.stat().st_size,
            "sha256": _sha256(glb_path),
        },
        "fbx": {
            "path": output["fbx"],
            "bytes": fbx_path.stat().st_size,
            "sha256": _sha256(fbx_path),
        },
    }


def _point_at(obj: bpy.types.Object, target: Vector) -> None:
    obj.rotation_euler = (target - obj.location).to_track_quat("-Z", "Y").to_euler()


def _preview_material(name: str, color: tuple[float, float, float, float]) -> bpy.types.Material:
    material = bpy.data.materials.new(name)
    material.diffuse_color = color
    material.roughness = 0.82
    return material


def _render_preview(lods: list[bpy.types.Object], preview_path: Path) -> None:
    scene = bpy.context.scene
    try:
        scene.render.engine = "BLENDER_EEVEE_NEXT"
    except TypeError:
        scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(preview_path)
    scene.render.film_transparent = False
    scene.world.color = (0.012, 0.016, 0.022)
    scene.view_settings.exposure = -1.0
    try:
        scene.view_settings.look = "AgX - Medium High Contrast"
    except TypeError:
        pass

    for obj in lods:
        obj.hide_render = obj is not lods[0]
        obj.hide_set(obj is not lods[0])
    minimum, maximum = _bounds(lods[0])
    center = (minimum + maximum) * 0.5
    dimensions = maximum - minimum
    extent = max(float(value) for value in dimensions)

    temporary: list[bpy.types.Object] = []
    bpy.ops.mesh.primitive_plane_add(
        size=max(5.0, extent * 4.0),
        location=(0.0, 0.0, -0.004),
    )
    floor = bpy.context.object
    floor.name = "ConditioningPreviewFloor"
    floor.data.materials.append(
        _preview_material("M_ConditioningPreviewFloor", (0.035, 0.043, 0.055, 1.0))
    )
    temporary.append(floor)

    camera_data = bpy.data.cameras.new("ConditioningPreviewCameraData")
    camera = bpy.data.objects.new("ConditioningPreviewCamera", camera_data)
    bpy.context.collection.objects.link(camera)
    camera.location = center + Vector((extent * 3.0, -extent * 4.0, extent * 2.5))
    camera.data.lens = 54
    _point_at(camera, center)
    scene.camera = camera
    temporary.append(camera)

    for name, offset, energy, size, color in (
        ("Key", (-1.6, -1.7, 2.4), 420.0, 3.0, (1.0, 0.88, 0.76)),
        ("Fill", (1.8, -0.5, 1.3), 220.0, 2.6, (0.65, 0.76, 1.0)),
        ("Rim", (0.3, 1.8, 2.0), 320.0, 2.2, (0.78, 0.86, 1.0)),
    ):
        data = bpy.data.lights.new(f"ConditioningPreview{name}Data", "AREA")
        data.energy = energy * extent * extent
        data.shape = "DISK"
        data.size = size
        data.color = color
        light = bpy.data.objects.new(f"ConditioningPreview{name}", data)
        light.location = center + Vector(offset) * extent
        _point_at(light, center)
        bpy.context.collection.objects.link(light)
        temporary.append(light)

    result = bpy.ops.render.render(write_still=True)
    if "FINISHED" not in result or not preview_path.is_file():
        raise RuntimeError("Blender did not render the static-prop preview")
    bpy.ops.object.select_all(action="DESELECT")
    for obj in temporary:
        obj.select_set(True)
    bpy.ops.object.delete(use_global=False)
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
    """Materialize and report one validated conditioning plan."""

    _require_blender()
    plan_path = Path(os.path.abspath(os.fspath(plan_path)))
    raw, plan_evidence = _read_plan(plan_path)
    if (
        expected_plan_bytes is not None
        and plan_evidence["bytes"] != expected_plan_bytes
        or expected_plan_sha256 is not None
        and plan_evidence["sha256"] != expected_plan_sha256
    ):
        raise RuntimeError("conditioning plan identity changed after command construction")
    plan = validate_static_prop_conditioning_plan(raw)
    root = plan_path.parent
    global _ACTIVE_PUBLICATION, _ACTIVE_TRUSTED_ROOT
    trusted_parent = tuple(plan_evidence["parent_identity"])
    trusted_root = _TrustedDirectory(root, trusted_parent)
    _ACTIVE_TRUSTED_ROOT = trusted_root
    _assert_directory_identity(root, trusted_parent)
    source_path = _artifact_path(root, plan["source"]["model"]["path"], "source model")
    source_record = plan["source"]["model"]
    output_dir = _artifact_path(root, plan["outputs"]["directory"], "output directory")
    if output_dir.exists():
        raise RuntimeError("conditioned output directory already exists")
    global _ACTIVE_STAGING
    _assert_directory_identity(root, trusted_parent)
    staging_root = Path(tempfile.mkdtemp(prefix=".lingtu-static-conditioning-", dir=root))
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
    result = bpy.ops.import_scene.gltf(filepath=str(snapshot))
    if "FINISHED" not in result:
        raise RuntimeError("Blender could not import the Tripo GLB")
    all_imported = [obj for obj in bpy.data.objects if obj not in before]
    _enforce_scene_import_budgets(all_imported)
    imported_meshes = sorted(
        (obj for obj in all_imported if obj.type == "MESH"),
        key=lambda obj: obj.name,
    )
    joined = _join_meshes(imported_meshes, plan["asset"]["id"])
    source_dimensions, normalized_dimensions, uniform_scale = _normalize_geometry(
        joined,
        source_axis_order=plan["geometry"]["source_axis_order"],
        target_dimensions=plan["geometry"]["target_dimensions_m"],
    )
    topology = _topology_report(joined)
    lods = _make_lods(joined, plan["asset"]["id"], plan["lods"])
    textures = _save_textures(staged_output)
    output_records = [
        _export_lod(obj, root=staging_root, output=specification)
        for obj, specification in zip(lods, plan["outputs"]["lods"])
    ]
    preview_path = _artifact_path(staging_root, plan["outputs"]["preview"], "preview output")
    _render_preview(lods, preview_path)
    blend_path = _artifact_path(staging_root, plan["outputs"]["blend"], "Blend output")
    save_result = bpy.ops.wm.save_as_mainfile(filepath=str(blend_path))
    if "FINISHED" not in save_result or not blend_path.is_file():
        raise RuntimeError("Blender did not save the conditioned source scene")

    material_records = []
    for material in sorted(
        (slot.material for slot in lods[0].material_slots if slot.material is not None),
        key=lambda item: item.name,
    ):
        material_records.append(
            {
                "name": material.name,
                "use_nodes": bool(material.use_nodes),
                "node_count": len(material.node_tree.nodes) if material.node_tree else 0,
            }
        )
    if not material_records:
        raise RuntimeError("conditioned PBR asset contains no material slots")

    qualification = {
        **plan["qualification"],
        "blockers": sorted(
            {
                *plan["qualification"]["blockers"],
                *(
                    ["topology_review_required"]
                    if topology_requires_review(topology)
                    else []
                ),
            }
        ),
    }
    report_body: dict[str, Any] = {
        "schema": "lingtu.sim.static-prop-conditioning-report.v1",
        "asset_id": plan["asset"]["id"],
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
            "uniform_scale": uniform_scale,
            "placement": "center_xy_ground_z",
            "normal_policy": "recalculate_consistent_outside",
        },
        "topology": topology,
        "materials": material_records,
        "textures": textures,
        "lods": output_records,
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
        "binding": plan["binding"],
        "qualification": qualification,
    }
    report = {**report_body, "digest": _digest_document(report_body)}
    report_path = _artifact_path(staging_root, plan["outputs"]["inspection"], "inspection output")
    _atomic_json(report_path, report)
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
    print(f"LINGTU_STATIC_PROP_CONDITIONED {report_path}")
    return report_path


def condition(
    plan_path: Path, *, expected_plan_bytes: int | None = None, expected_plan_sha256: str | None = None
) -> Path:
    """Materialize one plan and remove private staging on every exit path."""

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
    parser = argparse.ArgumentParser(description="Condition one static visual prop in Blender")
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
        raise RuntimeError("static conditioner script contract or SHA-256 mismatch")
    condition(
        args.plan,
        expected_plan_bytes=args.plan_bytes,
        expected_plan_sha256=args.plan_sha256,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
