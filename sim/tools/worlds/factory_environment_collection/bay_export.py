"""Read-only authoring export of the SAME Fab factory bay, not a runnable world.

Owner-only commandlet entry (this module never launches Unreal):
  LINGTU_FACTORY_BAY_EXPORT_DIR=<new independent run directory> (required)
  LINGTU_FACTORY_BAY_INTAKE=<corrected bay-intake.json> (optional)
  LINGTU_FACTORY_BAY_FBX_LIMIT=0 (default; optionally 1..100 unique static assets)

No source saves, migrations, actor selection changes, collision cooking, or asset
copies. Optional FBX writes interchange files only, with material baking disabled.
All transforms stay in Unreal's left-handed cm frame; quaternion order is XYZW.
"""

from __future__ import annotations

import json
import os
from collections import deque
from itertools import product
from pathlib import Path
from typing import Any

SOURCE_PROJECT = "D:/epic/Projects/FactoryPreview/FactoryProject.uproject"
SOURCE_MAP = "/Game/Maps/Demonstration"
BAY_MIN = [3000, -2000, -100]
BAY_MAX = [7000, 1600, 1500]
DEFAULT_INTAKE = (
    Path(__file__).resolve().parents[4]
    / "build/unreal-tests/product-progress-20261021"
    / "factory-bay-intake-api-corrected/bay-intake.json"
)
MAX_ACTORS = 20000
MAX_COMPONENTS = 40000
MAX_INSTANCES = 20000
MAX_DEPENDENCIES = 2048
MAX_SHAPES = 20000
MAX_CONVEX_VERTICES = 100000

# Verified in installed UE 5.8.1 (CL 56057345), relative to Engine/Source.
# UFUNCTION ScriptName overrides matter; native C++ getters are NOT Python APIs.
API_EVIDENCE = {
    "Paths.get_project_file_path": "Runtime/Engine/Classes/Kismet/BlueprintPathsLibrary.h: GetProjectFilePath",
    "SceneComponent.get_world_transform": "Runtime/Engine/Classes/Components/SceneComponent.h: K2_GetComponentToWorld, ScriptName=GetWorldTransform",
    "SystemLibrary.get_component_bounds": "Runtime/Engine/Classes/Kismet/KismetSystemLibrary.h: GetComponentBounds",
    "InstancedStaticMeshComponent.get_instance_transform": "Runtime/Engine/Classes/Components/InstancedStaticMeshComponent.h: GetInstanceTransform (out Transform; bool result)",
    "Python_out_parameter_return": "Engine/Plugins/Experimental/PythonScriptPlugin/Source/PythonScriptPlugin/Private/PyGenUtil.cpp: PackReturnValues returns out value on bool success, None on failure",
    "InstancedStaticMeshComponent.get_instance_count": "Runtime/Engine/Classes/Components/InstancedStaticMeshComponent.h: GetInstanceCount",
    "StaticMesh.get_bounds": "Runtime/Engine/Classes/Engine/StaticMesh.h: GetBounds",
    "SplineMeshComponent.get_start_position/get_end_position": "Runtime/Engine/Classes/Components/SplineMeshComponent.h: GetStart/EndPosition; Tangent, Scale, Roll, Offset; GetForwardAxis, GetSplineUpDir, GetBoundaryMin/Max",
    "SplineMeshComponent.body_setup": "Runtime/Engine/Classes/Components/SplineMeshComponent.h: BodySetup UPROPERTY (existing deformed geometry only)",
    "SkeletalMeshComponent.get_skeletal_mesh_asset": "Runtime/Engine/Classes/Components/SkeletalMeshComponent.h: GetSkeletalMeshAsset",
    "SkinnedMeshComponent.get_bone_transform": "Runtime/Engine/Classes/Components/SkinnedMeshComponent.h: GetBoneTransform(FName, ERelativeTransformSpace), UFUNCTION",
    "SkinnedMeshComponent.physics_asset_override": "Runtime/Engine/Classes/Components/SkinnedMeshComponent.h: PhysicsAssetOverride UPROPERTY; GetPhysicsAsset is NOT UFUNCTION",
    "SkeletalMesh.physics_asset": "Runtime/Engine/Classes/Engine/SkeletalMesh.h: PhysicsAsset UPROPERTY",
    "PhysicsAsset.skeletal_body_setups": "Runtime/Engine/Classes/PhysicsEngine/PhysicsAsset.h: SkeletalBodySetups UPROPERTY",
    "StaticMesh.body_setup": "Runtime/Engine/Classes/Engine/StaticMesh.h: BodySetup UPROPERTY; GetBodySetup is NOT UFUNCTION",
    "BodySetup.agg_geom": "Runtime/Engine/Classes/PhysicsEngine/BodySetup.h: AggGeom UPROPERTY",
    "BodySetup.collision_trace_flag/bone_name": "Runtime/PhysicsCore/Public/BodySetupCore.h: CollisionTraceFlag, BoneName UPROPERTY",
    "KAggregateGeom": "Runtime/Engine/Classes/PhysicsEngine/AggregateGeom.h: Sphere/Box/Sphyl/Convex/TaperedCapsuleElems UPROPERTY",
    "KConvexElem.vertex_data/index_data/transform": "Runtime/Engine/Classes/PhysicsEngine/ConvexElem.h: VertexData, IndexData, Transform UPROPERTY (runtime exposure checked)",
    "KBoxElem/KSphereElem/KSphylElem/KTaperedCapsuleElem": "Runtime/Engine/Classes/PhysicsEngine/{Box,Sphere,Sphyl,TaperedCapsule}Elem.h: Center, Rotation, dimensions UPROPERTY; ShapeElem.h: CollisionEnabled",
    "Transform.translation/rotation/scale3d": "Runtime/CoreUObject/Public/UObject/NoExportTypes.h: FTransform UPROPERTY",
    "AssetRegistry.get_dependencies": "Runtime/AssetRegistry/Public/AssetRegistry/IAssetRegistry.h: K2_GetDependencies, ScriptName=GetDependencies; FAssetRegistryDependencyOptions",
    "AssetRegistry.get_assets_by_package_name": "Runtime/AssetRegistry/Public/AssetRegistry/IAssetRegistry.h: GetAssetsByPackageName",
    "Exporter.run_asset_export_task": "Runtime/Engine/Classes/Exporters/Exporter.h: RunAssetExportTask",
    "AssetExportTask": "Runtime/Engine/Public/AssetExportTask.h: automated, prompt, replace_identical, object, exporter, filename, options, errors",
    "FbxExportOption.bake_material_inputs": "Editor/UnrealEd/Classes/Exporters/FbxExportOption.h: BakeMaterialInputs; EFbxMaterialBakeMode::Disabled",
}


def normalized_path(value: str) -> str:
    return value.replace("\\", "/").rstrip("/").casefold()


def xyz(value: Any) -> list[float]:
    return [float(value.x), float(value.y), float(value.z)]


def transform_record(value: Any) -> dict[str, Any]:
    q = value.rotation
    return {
        "translation_cm": xyz(value.translation),
        "rotation_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
        "scale3d": xyz(value.scale3d),
    }


def transformed_bounds(low: list, high: list, transform: dict) -> tuple[list, list]:
    """Transform eight visual-bound corners; NEVER a collision approximation."""
    qx, qy, qz, qw = transform["rotation_xyzw"]
    points = []
    for corner in product(*zip(low, high)):
        v = [corner[i] * transform["scale3d"][i] for i in range(3)]
        uv = [qy * v[2] - qz * v[1], qz * v[0] - qx * v[2], qx * v[1] - qy * v[0]]
        uuv = [qy * uv[2] - qz * uv[1], qz * uv[0] - qx * uv[2], qx * uv[1] - qy * uv[0]]
        points.append([v[i] + 2 * (qw * uv[i] + uuv[i]) + transform["translation_cm"][i] for i in range(3)])
    return ([min(p[i] for p in points) for i in range(3)], [max(p[i] for p in points) for i in range(3)])


def disposition(low: list, high: list) -> str:
    if any(high[i] < BAY_MIN[i] or low[i] > BAY_MAX[i] for i in range(3)):
        return "outside_bay"
    if any(low[i] < BAY_MIN[i] or high[i] > BAY_MAX[i] for i in range(3)):
        return "crossing_pending_review_no_clip_no_migration"
    return "selected_for_authoring_review_not_qualified"


def source_id(object_path: str, instance_index: int | None = None) -> str:
    # Paths are stable across repeat exports, not guaranteed across source renames.
    # Instance indices identify this source snapshot, not edits/reordering of ISMs.
    return object_path if instance_index is None else f"{object_path}#instance:{instance_index}"


def unavailable(error: Any) -> dict:
    return {"status": "unavailable", "reason": str(error)}


def probe(read: Any) -> dict:
    """Optional reflected data must not silently become absent/empty geometry."""
    try:
        return {"status": "available", "value": read()}
    except Exception as error:
        return unavailable(f"{type(error).__name__}: {error}")


def property_probe(obj: Any, name: str, convert: Any = str) -> dict:
    return probe(lambda: convert(obj.get_editor_property(name)))


def path_or_none(obj: Any) -> str | None:
    return obj.get_path_name() if obj is not None else None


def collision_body(body: Any) -> dict:
    """Existing asset/body-local source geometry, never a qualified physics proxy."""
    result = {
        "basis": "existing_UE_BodySetup_AggGeom",
        "qualified_collision": False,
        "space": "body_local_cm",
        "visual_aabb_used": False,
    }
    if body is None:
        return {**result, "status": "absent", "reason": "source has no existing BodySetup"}
    result.update(source_body=body.get_path_name(), shapes=[], unavailable=[])
    result["collision_trace_flag"] = property_probe(body, "collision_trace_flag")
    agg = probe(lambda: body.get_editor_property("agg_geom"))
    if agg["status"] != "available":
        return {**result, **agg}
    fields = {
        "sphere_elems": "sphere",
        "box_elems": "box",
        "sphyl_elems": "capsule_z",
        "convex_elems": "convex",
        "tapered_capsule_elems": "tapered_capsule_z",
    }
    vertices_total = 0
    for field, kind in fields.items():
        values = probe(lambda field=field: list(agg["value"].get_editor_property(field)))
        if values["status"] != "available":
            result["unavailable"].append({"field": field, **values})
            continue
        for index, elem in enumerate(values["value"]):
            if len(result["shapes"]) >= MAX_SHAPES:
                raise RuntimeError("collision shape budget exceeded")
            shape = {
                "source_id": f"{body.get_path_name()}#{field}:{index}",
                "kind": kind,
                "collision_enabled": property_probe(elem, "collision_enabled"),
            }

            def geometry(elem=elem, kind=kind):
                nonlocal vertices_total
                get = elem.get_editor_property
                if kind == "convex":
                    vertices = list(get("vertex_data"))
                    vertices_total += len(vertices)
                    if vertices_total > MAX_CONVEX_VERTICES:
                        raise RuntimeError("convex vertex budget exceeded")
                    indices = property_probe(elem, "index_data", lambda a: [int(i) for i in a])
                    return {
                        "vertices_cm": [xyz(v) for v in vertices],
                        "indices": indices,
                        "element_transform": property_probe(elem, "transform", transform_record),
                    }
                data = {"center_cm": xyz(get("center"))}
                if kind != "sphere":
                    r = get("rotation")
                    data["rotation_degrees_pyr"] = [float(r.pitch), float(r.yaw), float(r.roll)]
                if kind == "box":
                    # FKBoxElem volume is X*Y*Z: these are FULL lengths, not half extents.
                    data["full_size_cm"] = [float(get(axis)) for axis in ("x", "y", "z")]
                elif kind == "tapered_capsule_z":
                    data.update(
                        radius0_cm=float(get("radius0")),
                        radius1_cm=float(get("radius1")),
                        segment_length_cm=float(get("length")),
                        width_cm=float(get("width")),
                        one_sided_collision=bool(get("one_sided_collision")),
                    )
                else:
                    data["radius_cm"] = float(get("radius"))
                    if kind == "capsule_z":
                        data["segment_length_cm"] = float(get("length"))
                return data

            shape["geometry"] = probe(geometry)
            result["shapes"].append(shape)
    for field in ("level_set_elems", "skinned_level_set_elems", "ml_level_set_elems", "skinned_triangle_mesh_elems"):
        result["unavailable"].append(
            {
                "field": field,
                "count": property_probe(agg["value"], field, len),
                "reason": "geometry serialization not supported; not substituted",
            }
        )
    result["status"] = "source_geometry_read_not_qualified"
    return result


def object_collision(obj: Any) -> dict:
    read = probe(lambda: collision_body(obj.get_editor_property("body_setup")))
    return (
        read["value"]
        if read["status"] == "available"
        else {
            **read,
            "qualified_collision": False,
            "visual_aabb_used": False,
            "api": "get_editor_property('body_setup')",
        }
    )


def excluded_package(package: str) -> str | None:
    lower = package.casefold()
    if "builtdata" in lower:
        return "baked_lighting_excluded"
    if lower.startswith("/script/"):
        return "native_type_reference_not_asset_copy"
    if lower.startswith("/game/maps/") or "pw_forklift" in lower or "physx" in lower:
        return "source_map_or_old_gameplay_excluded"
    return None


DEPENDENCY_CLASSES = {
    "StaticMesh",
    "SkeletalMesh",
    "Skeleton",
    "PhysicsAsset",
    "PhysicalMaterial",
    "Material",
    "MaterialInstanceConstant",
    "MaterialFunction",
    "MaterialFunctionInstance",
    "MaterialFunctionMaterialLayer",
    "MaterialFunctionMaterialLayerBlend",
    "MaterialParameterCollection",
    "Texture2D",
    "TextureCube",
    "Texture2DArray",
    "TextureCubeArray",
    "VolumeTexture",
    "VirtualTextureBuilder",
    "RuntimeVirtualTexture",
    "SubsurfaceProfile",
}


def dependency_manifest(registry: Any, options: Any, roots: set[str]) -> dict:
    """Bounded package closure from geometry/material roots, NEVER actor blueprints."""
    queue = deque(sorted({root.split(".", 1)[0] for root in roots}))
    visited, records = set(), []
    while queue and len(visited) < MAX_DEPENDENCIES:
        package = queue.popleft()
        if package in visited:
            continue
        visited.add(package)
        row = {"package": package}
        reason = excluded_package(package)
        if reason:
            records.append({**row, "disposition": reason})
            continue
        metadata = probe(lambda package=package: list(registry.get_assets_by_package_name(package)))
        if metadata["status"] != "available":
            records.append({**row, "disposition": "unresolved", "metadata": metadata})
            continue
        classes = sorted({str(a.asset_class_path.asset_name) for a in metadata["value"]})
        row["asset_classes"] = classes
        if not classes or not all(c in DEPENDENCY_CLASSES for c in classes):
            records.append({**row, "disposition": "unresolved" if not classes else "non_geometry_dependency_excluded"})
            continue
        refs = probe(lambda package=package: sorted(str(d) for d in registry.get_dependencies(package, options)))
        records.append({**row, "disposition": "inventory_only_no_copy", "dependencies": refs})
        if refs["status"] == "available":
            queue.extend(d for d in refs["value"] if d not in visited)
    pending = sorted(set(queue) - visited)
    return {
        "roots": sorted(roots),
        "packages": sorted(records, key=lambda r: r["package"]),
        "pending_budget": pending,
        "complete": not pending
        and all(
            r["disposition"] != "unresolved" and r.get("dependencies", {}).get("status") != "unavailable"
            for r in records
        ),
        "policy": "hard+soft game package refs; no management/searchable/editor-only; geometry/material class allowlist; no map/BuiltData/blueprint gameplay copies",
    }


def validate_intake(intake: dict) -> dict[str, dict]:
    if (
        normalized_path(intake["source_project"]) != normalized_path(SOURCE_PROJECT)
        or intake["source_world"] != SOURCE_MAP + ".Demonstration"
        or intake["bounds_min_cm"] != BAY_MIN
        or intake["bounds_max_cm"] != BAY_MAX
    ):
        raise ValueError("intake is not the selected source factory bay")
    rows = {a["source_actor"]: a for a in intake["actors"]}
    if len(rows) != intake["selected_actor_count"] or len(rows) != len(intake["actors"]):
        raise ValueError("intake actor identity/count mismatch")
    return rows


def spline_record(component: Any) -> dict:
    result = {"space": "component_local", "baked_deformed_mesh_exported": False}
    for suffix in ("start_position", "end_position", "start_tangent", "end_tangent", "spline_up_dir"):
        result[suffix] = probe(lambda suffix=suffix: xyz(getattr(component, "get_" + suffix)()))
    for suffix in ("start_scale", "end_scale", "start_offset", "end_offset"):
        result[suffix] = probe(
            lambda suffix=suffix: [
                float(v) for v in (getattr(component, "get_" + suffix)().x, getattr(component, "get_" + suffix)().y)
            ]
        )
    for suffix in ("start_roll", "end_roll", "boundary_min", "boundary_max"):
        result[suffix] = probe(lambda suffix=suffix: float(getattr(component, "get_" + suffix)()))
    result["forward_axis"] = probe(lambda: str(component.get_forward_axis()))
    result["smooth_interp_roll_scale"] = property_probe(component, "smooth_interp_roll_scale", bool)
    return result


def component_spatial(ue: Any, component: Any) -> tuple[dict, list, list]:
    transform = transform_record(component.get_world_transform())
    center, extent, _ = ue.SystemLibrary.get_component_bounds(component)
    if not any(xyz(extent)):
        # Non-rendering SceneComponents can have no visual volume. Select the
        # actual component origin, never an actor-origin or influence-radius box.
        point = transform["translation_cm"]
        return transform, point[:], point[:]
    low = [center.x - extent.x, center.y - extent.y, center.z - extent.z]
    high = [center.x + extent.x, center.y + extent.y, center.z + extent.z]
    return transform, low, high


def component_record(
    ue: Any, component: Any, assets: dict, roots: set, budget: dict, spatial: tuple | None = None
) -> dict:
    transform, low, high = spatial or component_spatial(ue, component)
    row = {
        "source_id": source_id(component.get_path_name()),
        "class": component.get_class().get_name(),
        "world_transform": transform,
        "visual_bounds_min_cm": low,
        "visual_bounds_max_cm": high,
        "disposition": disposition(low, high),
        "kind": "other_scene_component",
        "collision": {
            "status": "unavailable",
            "reason": "not a supported mesh component",
            "qualified_collision": False,
        },
    }
    is_static = isinstance(component, ue.StaticMeshComponent)
    is_skeletal = isinstance(component, ue.SkeletalMeshComponent)
    if not is_static and not is_skeletal:
        row["reconstruction"] = "inventory_only; light/VFX/spline/other payload not reconstructed"
        return row
    mesh = component.get_editor_property("static_mesh") if is_static else component.get_skeletal_mesh_asset()
    row["kind"] = "static_mesh" if is_static else "skeletal_mesh"
    row["mesh_asset"] = path_or_none(mesh)
    row["materials"] = [path_or_none(component.get_material(i)) for i in range(component.get_num_materials())]
    row["collision_enabled"] = probe(lambda: str(component.get_collision_enabled()))
    row["collision_profile"] = probe(lambda: str(component.get_collision_profile_name()))
    row["visible"] = property_probe(component, "visible", bool)
    if isinstance(component, ue.SplineMeshComponent):
        row["kind"] = "spline_mesh"
    elif isinstance(component, ue.InstancedStaticMeshComponent):
        row["kind"] = "instanced_static_mesh"
    if row["disposition"] == "outside_bay":
        row["reconstruction"] = "outside selected bay; no collision payload/dependency/export work"
        return row
    if isinstance(component, ue.SplineMeshComponent):
        row.update(kind="spline_mesh", spline=spline_record(component), collision=object_collision(component))
        row["collision"]["space"] = "deformed_component_local_cm; not undeformed StaticMesh collision"
    elif isinstance(component, ue.InstancedStaticMeshComponent):
        count = component.get_instance_count()
        budget["instances"] += count
        if budget["instances"] > MAX_INSTANCES:
            raise RuntimeError("instance budget exceeded; export incomplete")
        row.update(
            kind="instanced_static_mesh",
            instances=[],
            instance_count=count,
            instance_id_scope="source component path + index; stable only while source ordering is unchanged",
        )
        mesh_bounds = mesh.get_bounds() if mesh is not None else None
        for index in range(count):
            # Python suppresses a bool success result: Transform on success, None on failure.
            instance = component.get_instance_transform(index, world_space=True)
            if instance is None:
                raise RuntimeError(f"missing instance transform: {row['source_id']} instance {index}")
            item = {"source_id": source_id(row["source_id"], index), "world_transform": transform_record(instance)}
            if mesh_bounds is not None:
                origin, box = xyz(mesh_bounds.origin), xyz(mesh_bounds.box_extent)
                ilow, ihigh = transformed_bounds(
                    [origin[i] - box[i] for i in range(3)],
                    [origin[i] + box[i] for i in range(3)],
                    item["world_transform"],
                )
                item.update(visual_bounds_min_cm=ilow, visual_bounds_max_cm=ihigh, disposition=disposition(ilow, ihigh))
            else:
                item["disposition"] = "unavailable_no_mesh_bounds"
            row["instances"].append(item)
    elif is_skeletal:
        physics = probe(
            lambda: (
                component.get_editor_property("physics_asset_override")
                or (mesh.get_editor_property("physics_asset") if mesh is not None else None)
            )
        )
        row["collision"] = {
            "status": "unavailable",
            "qualified_collision": False,
            "reason": "physics asset not exposed or absent",
            "visual_aabb_used": False,
        }
        if physics["status"] == "available" and physics["value"] is not None:
            pa = physics["value"]
            roots.add(pa.get_path_name())
            row["physics_asset"] = pa.get_path_name()
            bodies = probe(lambda: list(pa.get_editor_property("skeletal_body_setups")))
            row["collision"] = {
                "status": "source_physics_asset_not_qualified",
                "qualified_collision": False,
                "visual_aabb_used": False,
                "bodies": [],
                "pose": "editor snapshot; no animation evaluation",
            }
            if bodies["status"] == "available":
                for body in bodies["value"]:
                    bone = property_probe(body, "bone_name")
                    row["collision"]["bodies"].append(
                        {
                            "bone": bone,
                            "geometry": collision_body(body),
                            "bone_world_transform": probe(
                                lambda bone=bone: transform_record(
                                    component.get_bone_transform(bone["value"], ue.RelativeTransformSpace.RTS_WORLD)
                                )
                            )
                            if bone["status"] == "available"
                            else unavailable("bone name unavailable"),
                        }
                    )
            else:
                row["collision"]["body_access"] = bodies
        else:
            row["collision"]["physics_access"] = {k: v for k, v in physics.items() if k != "value"}
    if mesh is not None and row["disposition"] != "outside_bay":
        path = mesh.get_path_name()
        roots.add(path)
        roots.update(m for m in row["materials"] if m is not None)
        if path not in assets:
            assets[path] = {
                "source_id": path,
                "kind": "static_mesh" if is_static else "skeletal_mesh",
                "collision": object_collision(mesh)
                if is_static
                else {"status": "see_component_physics_asset", "qualified_collision": False},
            }
        if row["kind"] in ("static_mesh", "instanced_static_mesh"):
            row["collision"] = {
                "source_asset": path,
                "basis": "asset_local_shapes + actual component/instance transform",
                "qualified_collision": False,
                "visual_aabb_used": False,
                "note": "nonuniform/negative scale and collision mode need downstream qualification",
            }
    return row


def collect_manifest(ue: Any, intake: dict, world: Any) -> tuple[dict, dict]:
    original = validate_intake(intake)
    actors = ue.get_editor_subsystem(ue.EditorActorSubsystem).get_all_level_actors()
    if len(actors) > MAX_ACTORS:
        raise RuntimeError("actor budget exceeded")
    rows, assets, roots, found = [], {}, set(), set()
    mesh_mismatches = []
    budget = {"components": 0, "instances": 0}
    for actor in sorted(actors, key=lambda a: a.get_path_name()):
        actor_path = actor.get_path_name()
        baseline = original.get(actor_path)
        if baseline is not None:
            found.add(actor_path)
        components = actor.get_components_by_class(ue.SceneComponent)
        budget["components"] += len(components)
        if budget["components"] > MAX_COMPONENTS:
            raise RuntimeError("component budget exceeded")
        # Actor bounds omit non-primitive components such as lights. Inspect
        # component bounds/origins before rejecting an actor, without reading
        # mesh/physics/dependency payloads in distant bays.
        spatial = {c.get_path_name(): component_spatial(ue, c) for c in components}
        center, extent = actor.get_actor_bounds(False)
        low, high = (
            [center.x - extent.x, center.y - extent.y, center.z - extent.z],
            [center.x + extent.x, center.y + extent.y, center.z + extent.z],
        )
        if baseline is None and not any(disposition(s[1], s[2]) != "outside_bay" for s in spatial.values()):
            continue
        if spatial:
            low = [min(s[1][i] for s in spatial.values()) for i in range(3)]
            high = [max(s[2][i] for s in spatial.values()) for i in range(3)]
        row = {
            "source_id": actor_path,
            "label": actor.get_actor_label(),
            "class": actor.get_class().get_path_name(),
            "in_original_intake": baseline is not None,
            "original_crosses_bay_boundary": baseline.get("crosses_bay_boundary") if baseline else None,
            "disposition": disposition(low, high),
            "components": [],
        }
        if "pw_forklift" in row["class"].casefold():
            row["disposition"] = "excluded_old_gameplay"
        else:
            row["components"] = [
                component_record(ue, c, assets, roots, budget, spatial[c.get_path_name()])
                for c in sorted(components, key=lambda c: c.get_path_name())
            ]
            if baseline is not None and "mesh_assets" in baseline:
                observed = sorted(
                    {c["mesh_asset"] for c in row["components"] if c.get("mesh_asset") and c["kind"] != "skeletal_mesh"}
                )
                row["original_mesh_assets_match"] = observed == sorted(baseline["mesh_assets"])
                if not row["original_mesh_assets_match"]:
                    mesh_mismatches.append(
                        {"source_id": actor_path, "expected": baseline["mesh_assets"], "observed": observed}
                    )
        if baseline is not None or any(c["disposition"] != "outside_bay" for c in row["components"]):
            rows.append(row)
    missing = sorted(set(original) - found)
    options = ue.AssetRegistryDependencyOptions()
    for name, value in {
        "include_hard_package_references": True,
        "include_soft_package_references": True,
        "include_game_package_references": True,
        "include_editor_only_package_references": False,
        "include_searchable_names": False,
        "include_soft_management_references": False,
        "include_hard_management_references": False,
    }.items():
        options.set_editor_property(name, value)
    dependencies = dependency_manifest(ue.AssetRegistryHelpers.get_asset_registry(), options, roots)
    manifest = {
        "schema": "lingtu.factory-bay-authoring.v1",
        "purpose": "same_selected_source_bay_not_runnable_or_qualified_collision",
        "source_project": SOURCE_PROJECT,
        "source_world": world.get_path_name(),
        "source_map_saved": False,
        "source_assets_copied": False,
        "units": "Unreal_cm",
        "frame": "Unreal_left_handed_Xforward_Yright_Zup",
        "bounds_min_cm": BAY_MIN,
        "bounds_max_cm": BAY_MAX,
        "bounds_role": "visual_selection_only_not_collision",
        "source_id_scope": "source object paths; not labels, hashes, or generated replacements; renames change identity",
        "original_intake_actor_count": len(original),
        "original_intake_mesh_count": len(intake["mesh_assets"]),
        "original_intake_crossing_actor_count": sum(bool(a.get("crosses_bay_boundary")) for a in original.values()),
        "original_intake_missing_actors": missing,
        "original_intake_mesh_mismatches": mesh_mismatches,
        "source_inventory_matches": not missing and not mesh_mismatches,
        "scanned_actor_count": len(actors),
        "budgets_used": budget,
        "actors": rows,
        "mesh_assets": [assets[p] for p in sorted(assets)],
        "dependencies": dependencies,
        "api_evidence": API_EVIDENCE,
        "api_evidence_level": "installed_5.8.1_headers; Python access probed; no runtime claim from offline tests",
        "qualified_collision": False,
        "limitations": [
            "No clipping, generated collision, cooked triangle data, physics qualification, or navigation clearance",
            "Crossing items remain pending; no migration approved",
            "Lights/VFX/other scene components inventoried, payloads not reconstructed",
            "Skeletal pose is editor snapshot; no simulation/animation run",
            "FBX asset material slots do not apply component overrides or spline deformation",
        ],
    }
    return manifest, assets


def export_fbx(ue: Any, manifest: dict, output: Path, limit: int) -> list[dict]:
    if not 0 <= limit <= 100:
        raise ValueError("FBX limit must be 0..100")
    reports = []
    candidates = [a for a in manifest["mesh_assets"] if a["kind"] == "static_mesh"]
    for index, asset in enumerate(candidates):
        row = {
            "source_asset": asset["source_id"],
            "space": "asset_local; UE FBX coordinate conversion",
            "material_basis": "asset material slots only; no baking; complex UE shaders not reproduced",
        }
        if index >= limit:
            reports.append({**row, "status": "not_requested" if limit == 0 else "skipped_mesh_budget"})
            continue
        filename = output / "fbx" / f"mesh-{index:03d}.fbx"
        filename.parent.mkdir(exist_ok=True)

        def run(filename=filename, asset=asset):
            options = ue.FbxExportOption()
            options.set_editor_property("bake_material_inputs", ue.FbxMaterialBakeMode.DISABLED)
            options.set_editor_property("collision", False)
            options.set_editor_property("level_of_detail", False)
            options.set_editor_property("export_source_mesh", True)
            task = ue.AssetExportTask()
            mesh = ue.load_asset(asset["source_id"])
            if mesh is None:
                raise RuntimeError("selected source mesh did not load")
            for key, value in {
                "object": mesh,
                "exporter": ue.StaticMeshExporterFBX(),
                "filename": str(filename),
                "automated": True,
                "prompt": False,
                "selected": False,
                "replace_identical": False,
                "write_empty_files": False,
                "options": options,
            }.items():
                task.set_editor_property(key, value)
            ok = ue.Exporter.run_asset_export_task(task)
            errors = [str(e) for e in task.get_editor_property("errors")]
            return {
                "exporter_returned": bool(ok),
                "errors": errors,
                "file": str(filename.relative_to(output)),
                "nonempty_file": filename.exists() and filename.stat().st_size > 0,
            }

        result = probe(run)
        reports.append(
            {
                **row,
                "attempt": result,
                "status": "written_not_material_fidelity_verified"
                if result["status"] == "available"
                and result["value"]["exporter_returned"]
                and result["value"]["nonempty_file"]
                and not result["value"]["errors"]
                else "unavailable_or_failed",
            }
        )
    return reports


def main() -> None:
    import unreal

    if normalized_path(unreal.Paths.get_project_file_path()) != normalized_path(SOURCE_PROJECT):
        raise RuntimeError("export must run in the exact isolated FactoryPreview source project")
    output = Path(os.environ["LINGTU_FACTORY_BAY_EXPORT_DIR"]).resolve()
    source_dir = normalized_path(str(Path(SOURCE_PROJECT).parent))
    if normalized_path(str(output)).startswith(source_dir + "/") or normalized_path(str(output)) == source_dir:
        raise ValueError("output must be an independent run directory outside the source project")
    limit = int(os.environ.get("LINGTU_FACTORY_BAY_FBX_LIMIT", "0"))
    if not 0 <= limit <= 100:
        raise ValueError("FBX limit must be 0..100")
    intake_path = Path(os.environ.get("LINGTU_FACTORY_BAY_INTAKE", str(DEFAULT_INTAKE)))
    intake = json.loads(intake_path.read_text(encoding="utf-8"))
    validate_intake(intake)
    output.mkdir(parents=True, exist_ok=False)
    world = unreal.EditorLoadingAndSavingUtils.load_map(SOURCE_MAP)
    if world is None or world.get_path_name() != SOURCE_MAP + ".Demonstration":
        raise RuntimeError("exact factory Demonstration world did not load")
    manifest, _ = collect_manifest(unreal, intake, world)
    manifest["intake_file"] = str(intake_path)
    # Preserve the full manifest BEFORE optional, lower-priority interchange work.
    with (output / "bay-export.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2, allow_nan=False)
    fbx = export_fbx(unreal, manifest, output, limit)
    with (output / "mesh-exports.json").open("x", encoding="utf-8") as stream:
        json.dump(fbx, stream, ensure_ascii=False, indent=2)
    unreal.log(
        f"LINGTU_FACTORY_BAY_EXPORT_COMPLETE actors={len(manifest['actors'])} "
        f"source_matches={manifest['source_inventory_matches']} saved_source=false qualified_collision=false"
    )


if __name__ == "__main__":
    main()
