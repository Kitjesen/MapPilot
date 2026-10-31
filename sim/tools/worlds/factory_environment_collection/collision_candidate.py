"""Offline AggGeom -> static MuJoCo candidate; never a WorldPackage publisher.

Run with the repository Python (NumPy/SciPy must already be installed)::

    python -m sim.tools.worlds.factory_environment_collection.collision_candidate \
        --source <bay-export.json> --output <new-independent-directory>

Only existing BodySetup boxes and convex vertex clouds are used. Missing geometry
is reported, never replaced by visual bounds. No UE, Blender, asset copies, robot
motion, or source writes. Compilation/ray tests are separate from qualification.
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from itertools import product
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull, QhullError

SOURCE_SCHEMA = "lingtu.factory-bay-authoring.v1"
SOURCE_FRAME = "Unreal_left_handed_Xforward_Yright_Zup"
FRAME = "right_handed_Xforward_Yleft_Zup_m"
UE_TO_M = np.diag([0.01, -0.01, 0.01, 1.0])

# Installed UE 5.8 Engine/Source paths, verified without launching the engine.
MATH_EVIDENCE = {
    "box": "Runtime/Engine/Classes/PhysicsEngine/BoxElem.h: GetTransform; GetScaledVolume=X*Y*Z",
    "rotator": "Runtime/Core/Public/Math/RotationTranslationMatrix.h: TRotationTranslationMatrix (row vectors)",
    "transform": "Runtime/Core/Public/Math/TransformNonVectorized.h: TransformPosition=R(Scale3D*V)+Translation",
    "quaternion": "Runtime/Core/Public/Math/Quat.h: RotateVector=V+2w(QxV)+2Qx(QxV)",
}


class GeometryError(ValueError):
    """A source primitive cannot be represented faithfully by this converter."""


def _array(value, shape, name):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise GeometryError(f"invalid {name}") from error
    if result.shape != shape or not np.isfinite(result).all():
        raise GeometryError(f"invalid {name}: expected finite {shape}")
    return result


def _available(record, name):
    if not isinstance(record, dict) or record.get("status") != "available":
        reason = record.get("reason", "not exported") if isinstance(record, dict) else "not exported"
        raise GeometryError(f"{name} unavailable: {reason}")
    return record["value"]


def transform_matrix(transform):
    """Column-vector affine equivalent of UE FTransform; no TRS composition."""
    translation = _array(transform["translation_cm"], (3,), "translation_cm")
    scale = _array(transform["scale3d"], (3,), "scale3d")
    q = _array(transform["rotation_xyzw"], (4,), "rotation_xyzw")
    if not np.isclose(q @ q, 1, atol=1e-6, rtol=0):
        raise GeometryError("non-unit source quaternion")
    if np.any(scale == 0):
        raise GeometryError("singular source scale")
    x, y, z, w = q / np.linalg.norm(q)
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    matrix = np.eye(4)
    matrix[:3, :3] = rotation @ np.diag(scale)
    matrix[:3, 3] = translation
    return matrix


def box_matrix(geometry):
    """Transpose UE's row-vector FRotationTranslationMatrix, NOT XYZ Euler."""
    pitch, yaw, roll = np.deg2rad(_array(geometry["rotation_degrees_pyr"], (3,), "rotator"))
    sp, sy, sr = np.sin([pitch, yaw, roll])
    cp, cy, cr = np.cos([pitch, yaw, roll])
    matrix = np.eye(4)
    matrix[:3, :3] = np.array(
        [
            [cp * cy, cp * sy, sp],
            [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp],
            [-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp],
        ]
    ).T
    matrix[:3, 3] = _array(geometry["center_cm"], (3,), "center_cm")
    return matrix


def apply_matrix(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def _hull(vertices):
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 4 or not np.isfinite(vertices).all():
        raise GeometryError("convex requires at least four finite XYZ vertices")
    # Translate before Qhull, avoiding precision loss from world offsets.
    try:
        hull = ConvexHull(vertices - vertices.mean(axis=0))
    except QhullError as error:
        raise GeometryError("degenerate/non-volumetric convex vertices") from error
    if hull.volume <= 0:
        raise GeometryError("zero convex volume")
    return hull


def _validate_indices(vertices, probe):
    """Validate a supplied closed convex triangular boundary, not render triangles.

    Unavailable/empty indices may use MuJoCo's vertex-cloud hull ONLY because the
    source is an existing AggGeom convex element. This remains a review issue.
    Invalid supplied indices must not silently fall back to a different hull.
    """
    if not isinstance(probe, dict) or probe.get("status") != "available":
        return {"status": "unavailable_vertex_cloud_only", "source": probe}
    indices = probe["value"]
    if not isinstance(indices, list):
        raise GeometryError("convex indices must be a flat integer list")
    if not indices:
        return {"status": "empty_vertex_cloud_only", "source": probe}
    if len(indices) % 3 or any(type(i) is not int or not 0 <= i < len(vertices) for i in indices):
        raise GeometryError("convex indices invalid triangle count/type/range")
    centered = vertices - vertices.mean(axis=0)
    extent = np.ptp(centered, axis=0).max()
    tolerance = extent * 1e-7
    edges = Counter()
    faces = set()
    for a, b, c in np.asarray(indices).reshape(-1, 3):
        face = tuple(sorted((int(a), int(b), int(c))))
        if len(set(face)) != 3 or face in faces:
            raise GeometryError("convex indices contain repeated/duplicate triangle")
        faces.add(face)
        normal = np.cross(centered[b] - centered[a], centered[c] - centered[a])
        length = np.linalg.norm(normal)
        if length <= extent * extent * 1e-12:
            raise GeometryError("convex indices contain degenerate triangle")
        distances = (centered - centered[a]) @ (normal / length)
        if distances.min() < -tolerance and distances.max() > tolerance:
            raise GeometryError("convex indices are not a convex supporting boundary")
        for edge in ((a, b), (b, c), (c, a)):
            edges[tuple(sorted(edge))] += 1
    if any(count != 2 for count in edges.values()):
        raise GeometryError("convex indices are not a closed two-manifold boundary")
    # A closed subset of faces must cover all the source hull's extreme vertices.
    if not set(_hull(vertices).vertices).issubset(set(indices)):
        raise GeometryError("convex indices omit hull vertices")
    return {"status": "validated_closed_convex_boundary", "source": probe, "triangles": len(indices) // 3}


def primitive_vertices(shape):
    """Return raw primitive vertices, primitive->asset matrix and index evidence."""
    geometry = _available(shape.get("geometry"), "source geometry")
    if shape["kind"] == "box":
        size = _array(geometry["full_size_cm"], (3,), "full_size_cm")
        if np.any(size <= 0):
            raise GeometryError("box full lengths must be positive")
        vertices = np.array(list(product((-0.5, 0.5), repeat=3))) * size
        return vertices, box_matrix(geometry), {"status": "analytic_source_box_eight_corners"}
    if shape["kind"] != "convex":
        raise GeometryError(f"unsupported AggGeom primitive: {shape['kind']}")
    try:
        vertices = np.asarray(geometry["vertices_cm"], dtype=float)
    except (TypeError, ValueError) as error:
        raise GeometryError("invalid convex vertices_cm") from error
    _hull(vertices)
    indices = _validate_indices(vertices, geometry.get("indices"))
    element = _available(geometry.get("element_transform"), "convex element transform")
    return vertices, transform_matrix(element), indices


def _physical_mode(probe):
    value = str(_available(probe, "collision_enabled"))
    # Exported enum repr is '<CollisionEnabled.QUERY_AND_PHYSICS: 3>'.
    name = value.strip("<>").split(":", 1)[0].rsplit(".", 1)[-1]
    if name not in {"QUERY_AND_PHYSICS", "PHYSICS_ONLY"}:
        raise GeometryError(f"collision mode not supported physical collision: {value}")


def _bounds(vertices):
    return {"min_m": vertices.min(axis=0).tolist(), "max_m": vertices.max(axis=0).tolist()}


def _numbers(values):
    return " ".join(format(float(v), ".17g") for v in np.asarray(values).ravel())


def _issue(row, code, detail, source_id=None):
    row["issues"].append({"code": code, "detail": detail, "source_id": source_id or row["source_id"]})


def convert_manifest(source):
    """Pure conversion: returns (MJCF text, identity/transform/coverage manifest)."""
    if (
        source.get("schema") != SOURCE_SCHEMA
        or source.get("units") != "Unreal_cm"
        or source.get("frame") != SOURCE_FRAME
    ):
        raise ValueError("expected factory bay authoring v1 in Unreal cm/LH frame")
    if source.get("source_inventory_matches") is not True:
        raise ValueError("source inventory mismatch; candidate conversion refused")
    low = _array(source["bounds_min_cm"], (3,), "selection min")
    high = _array(source["bounds_max_cm"], (3,), "selection max")
    if np.any(low >= high):
        raise ValueError("invalid selection bounds")
    selection = np.array(list(product(*zip(low, high))))
    canonical_selection = _bounds(apply_matrix(selection, UE_TO_M))
    root = ET.Element("mujoco", model="factory_source_collision_candidate_UNQUALIFIED")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    asset_node = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    manifest = {
        "schema": "lingtu.factory-collision-candidate.v1",
        "status": "offline_candidate_not_qualified_not_published",
        "qualified_collision": False,
        "published_world": False,
        "source_project": source.get("source_project"),
        "source_world": source.get("source_world"),
        "source_schema": source["schema"],
        "source_id_scope": source.get("source_id_scope"),
        "frame": FRAME,
        "frame_mapping": "(x_cm, y_cm, z_cm) -> (0.01*x_cm, -0.01*y_cm, 0.01*z_cm); no origin shift",
        "selection_bounds_m": canonical_selection,
        "selection_bounds_role": "review only; never collision geometry or clipping",
        "visual_aabb_used": False,
        "representation": "existing AggGeom primitives; affine-baked convex vertex meshes; static world geoms",
        "math_evidence": MATH_EVIDENCE,
        "limitations": [
            "Not a WorldPackage, qualified collision recipe, runtime binding, navigation map, or clearance proof",
            "Missing/unsupported source geometry is explicit; no AABB, floor plane, or render-mesh fallback",
            "Cross-boundary source/collision geometry retained whole, not clipped; review required",
            "Skeletal, spline/deformation and instanced components are not reconstructed",
            "UE project-default collision trace mode and collision profiles/response channels are not resolved",
            "Affine source geometry is not a claim of parity with Chaos cooked/scaled collision",
            "No robot motion, contacts, sensor qualification, or source asset/material migration tested",
        ],
        "components": [],
        "source_assets": [],
        "geometries": [],
    }
    assets = {}
    source_primitive_counts = Counter()
    for asset in source["mesh_assets"]:
        if asset["source_id"] in assets:
            raise ValueError(f"duplicate source asset: {asset['source_id']}")
        collision = asset.get("collision", {})
        shapes = collision.get("shapes", [])
        source_primitive_counts.update(s["kind"] for s in shapes)
        assets[asset["source_id"]] = asset
        manifest["source_assets"].append(
            {
                "source_id": asset["source_id"],
                "kind": asset["kind"],
                "source_body": collision.get("source_body"),
                "source_primitive_counts": dict(Counter(s["kind"] for s in shapes)),
                "source_geometry_status_counts": dict(
                    Counter(s.get("geometry", {}).get("status", "missing") for s in shapes)
                ),
                "source_primitives": [
                    {
                        "source_id": s["source_id"],
                        "kind": s["kind"],
                        "geometry_status": s.get("geometry", {}).get("status"),
                        "reason": s.get("geometry", {}).get("reason"),
                    }
                    for s in shapes
                ],
                "source_unavailable_fields": collision.get("unavailable", []),
            }
        )
    component_ids = set()
    primitive_uses = Counter()
    for actor in source["actors"]:
        for component in actor["components"]:
            cid = component["source_id"]
            if cid in component_ids:
                raise ValueError(f"duplicate source component: {cid}")
            component_ids.add(cid)
            row = {
                "source_id": cid,
                "actor_source_id": actor["source_id"],
                "kind": component["kind"],
                "class": component.get("class"),
                "mesh_asset": component.get("mesh_asset"),
                "source_disposition": component.get("disposition"),
                "source_world_transform": component.get("world_transform"),
                "source_collision_enabled": component.get("collision_enabled"),
                "source_collision_profile": component.get("collision_profile"),
                "geometry_ids": [],
                "rejected_primitives": [],
                "issues": [],
                "requires_review": True,
                "qualified_collision": False,
            }
            manifest["components"].append(row)
            if component["kind"] != "static_mesh":
                kind = (
                    "spline"
                    if "spline" in (component["kind"] + component.get("class", "")).lower()
                    else component["kind"]
                )
                _issue(row, f"unsupported_{kind}", "inventory only; no source geometry substitution")
                continue
            if component.get("disposition") == "outside_bay":
                _issue(row, "source_outside_selection", "exporter did not collect this component's selected geometry")
                continue
            if component.get("disposition") != "selected_for_authoring_review_not_qualified":
                _issue(row, "source_boundary_requires_review", "retained whole when source geometry is available")
            asset = assets.get(component.get("mesh_asset"))
            if asset is None:
                _issue(row, "missing_source_asset", "no exported source mesh asset record")
                continue
            collision = asset.get("collision", {})
            shapes = collision.get("shapes", [])
            row["source_primitive_counts"] = dict(Counter(s["kind"] for s in shapes))
            row["source_collision_trace_flag"] = collision.get("collision_trace_flag")
            for field in collision.get("unavailable", []):
                count = field.get("count", {})
                if count.get("status") != "available" or count.get("value") != 0:
                    _issue(row, "unconverted_agggeom_field", field, collision.get("source_body"))
            try:
                _physical_mode(component.get("collision_enabled"))
                if (
                    collision.get("basis") != "existing_UE_BodySetup_AggGeom"
                    or collision.get("space") != "body_local_cm"
                ):
                    raise GeometryError("not existing body-local UE AggGeom")
                if collision.get("status") != "source_geometry_read_not_qualified":
                    raise GeometryError(
                        f"source collision unavailable: {collision.get('reason', collision.get('status'))}"
                    )
                trace = str(_available(collision.get("collision_trace_flag"), "collision_trace_flag"))
                if "CTF_USE_COMPLEX_AS_SIMPLE" in trace:
                    raise GeometryError("complex-as-simple source collision cannot use AggGeom as physics truth")
                if "CTF_USE_DEFAULT" in trace:
                    _issue(row, "project_default_trace_requires_review", "source project default not resolved offline")
                component_matrix = transform_matrix(component["world_transform"])
                row["canonical_from_asset_cm"] = (UE_TO_M @ component_matrix).tolist()
                if not shapes:
                    _issue(row, "empty_source_agggeom", "no source primitives; no replacement generated")
                for shape in shapes:
                    sid = shape["source_id"]
                    try:
                        _physical_mode(shape.get("collision_enabled"))
                        vertices, element_matrix, indices = primitive_vertices(shape)
                        canonical_matrix = UE_TO_M @ component_matrix @ element_matrix
                        world_vertices = apply_matrix(vertices, canonical_matrix)
                        hull = _hull(world_vertices)
                        center = world_vertices.mean(axis=0)
                        local = world_vertices - center
                        name = f"source_geom_{len(manifest['geometries']):05d}"
                        bounds = _bounds(world_vertices)
                        crossing = any(
                            bounds["min_m"][i] < canonical_selection["min_m"][i]
                            or bounds["max_m"][i] > canonical_selection["max_m"][i]
                            for i in range(3)
                        )
                        ET.SubElement(asset_node, "mesh", name=name, vertex=_numbers(local))
                        ET.SubElement(
                            world,
                            "geom",
                            name=name,
                            type="mesh",
                            mesh=name,
                            pos=_numbers(center),
                            contype="1",
                            conaffinity="1",
                            group="0",
                        )
                        gid = f"{cid}::{sid}"
                        record = {
                            "source_id": gid,
                            "component_source_id": cid,
                            "actor_source_id": actor["source_id"],
                            "asset_source_id": asset["source_id"],
                            "primitive_source_id": sid,
                            "source_body": collision["source_body"],
                            "source_kind": shape["kind"],
                            "mjcf_geom": name,
                            "mjcf_mesh": name,
                            "source_geometry": shape["geometry"]["value"],
                            "source_primitive_from_local_cm": element_matrix.tolist(),
                            "canonical_from_primitive_cm": canonical_matrix.tolist(),
                            "mesh_origin_m": center.tolist(),
                            "mesh_vertices_m": local.tolist(),
                            "canonical_rotation_wxyz": [1, 0, 0, 0],
                            "index_validation": indices,
                            "hull_vertex_count": len(hull.vertices),
                            "collision_bounds": bounds,
                            "collision_crosses_selection": crossing,
                            "requires_review": True,
                            "qualified_collision": False,
                        }
                        manifest["geometries"].append(record)
                        row["geometry_ids"].append(gid)
                        primitive_uses[sid] += 1
                        if crossing:
                            _issue(
                                row,
                                "collision_boundary_requires_review",
                                "actual collision vertices extend beyond selection",
                                sid,
                            )
                        if indices["status"].endswith("vertex_cloud_only"):
                            _issue(row, "source_indices_unavailable_requires_review", indices["status"], sid)
                    except (GeometryError, KeyError) as error:
                        row["rejected_primitives"].append(
                            {"source_id": sid, "kind": shape["kind"], "reason": str(error)}
                        )
            except (GeometryError, KeyError) as error:
                _issue(row, "component_collision_rejected", str(error))
                row["rejected_primitives"] = [
                    {"source_id": s["source_id"], "kind": s["kind"], "reason": str(error)} for s in shapes
                ]
    for row in manifest["components"]:
        row["accepted_geometry_count"] = len(row["geometry_ids"])
        row["rejected_primitive_count"] = len(row["rejected_primitives"])
        row["status"] = (
            "partial_candidate"
            if row["geometry_ids"] and row["rejected_primitives"]
            else ("candidate_requires_review" if row["geometry_ids"] else "no_candidate_geometry")
        )
    for asset in manifest["source_assets"]:
        asset["accepted_geometry_count"] = sum(primitive_uses[s["source_id"]] for s in asset["source_primitives"])
        rejected = [
            p
            for row in manifest["components"]
            if row["mesh_asset"] == asset["source_id"]
            for p in row["rejected_primitives"]
        ]
        asset["rejected_primitive_instance_count"] = len(rejected)
        asset["rejection_reason_counts"] = dict(Counter(p["reason"] for p in rejected))
        for primitive in asset["source_primitives"]:
            primitive["accepted_geometry_count"] = primitive_uses[primitive["source_id"]]
            primitive["rejected_instance_count"] = sum(p["source_id"] == primitive["source_id"] for p in rejected)
            primitive["rejection_reason_counts"] = dict(
                Counter(p["reason"] for p in rejected if p["source_id"] == primitive["source_id"])
            )
    rows = manifest["components"]
    geoms = manifest["geometries"]
    manifest["summary"] = {
        "source_actors": len(source["actors"]),
        "source_components": len(rows),
        "source_static_mesh_assets": sum(a["kind"] == "static_mesh" for a in source["mesh_assets"]),
        "source_primitive_counts": dict(source_primitive_counts),
        "component_kind_counts": dict(Counter(r["kind"] for r in rows)),
        "accepted_geometry_counts": dict(Counter(g["source_kind"] for g in geoms)),
        "accepted_geometries": len(geoms),
        "accepted_unique_source_primitives": len(primitive_uses),
        "components_with_geometry": sum(bool(r["geometry_ids"]) for r in rows),
        "components_without_geometry": sum(not r["geometry_ids"] for r in rows),
        "rejected_primitive_instances": sum(r["rejected_primitive_count"] for r in rows),
        "rejection_reason_counts": dict(Counter(p["reason"] for r in rows for p in r["rejected_primitives"])),
        "component_issue_counts": dict(Counter(i["code"] for r in rows for i in r["issues"])),
        "retained_source_boundary_components": sum(
            bool(r["geometry_ids"]) and r["source_disposition"] == "crossing_pending_review_no_clip_no_migration"
            for r in rows
        ),
        "collision_boundary_geometries": sum(g["collision_crosses_selection"] for g in geoms),
    }
    if geoms:
        manifest["collision_bounds"] = _bounds(
            np.concatenate([np.asarray(g["mesh_vertices_m"]) + g["mesh_origin_m"] for g in geoms])
        )
    else:
        manifest["collision_bounds"] = None
    ET.indent(root)
    return ET.tostring(root, encoding="unicode") + "\n", manifest


def write_candidate(source_path: Path, output_dir: Path):
    """Write only into a NEW directory; existing directories are never reused."""
    source_path, output_dir = Path(source_path).resolve(), Path(output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(f"candidate directory must not exist: {output_dir}")
    source = json.loads(source_path.read_text(encoding="utf-8"))
    xml, manifest = convert_manifest(source)
    manifest["source_file"] = str(source_path)
    manifest["mjcf_file"] = "collision-candidate.xml"
    output_dir.mkdir(parents=True, exist_ok=False)
    (output_dir / "collision-candidate.xml").write_text(xml, encoding="utf-8")
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    manifest = write_candidate(args.source, args.output)
    print(
        json.dumps({"status": manifest["status"], "output": str(args.output), "summary": manifest["summary"]}, indent=2)
    )


if __name__ == "__main__":
    main()
