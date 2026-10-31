"""Read existing simple collision into transient DynamicMeshes; never launch UE.

Owner-run UE Python commandlet (no project/plugin descriptor edits):
  -EnablePlugins=GeometryScripting -run=pythonscript -script=<this absolute file>
  LINGTU_FACTORY_COLLISION_EXPORT_DIR=<NEW independent output directory> (required)
  LINGTU_FACTORY_COLLISION_BAY_MANIFEST=<prior bay-export.json> (optional)

Writes collision-mesh-export.json, including unavailable results. No source world
load, saves, cooking, collision creation, render mesh or visual AABB fallback.
merge_supplement returns a deep copy for collision_candidate.py; it never writes.
Curved primitives are tessellations, not exact analytic collision surfaces.
Available means the copied triangle boundary was exported with matching piece
counts, not that its geometry or unreferenced vertices have been qualified.
"""

from __future__ import annotations

import copy
import json
import math
import os
from collections import Counter
from pathlib import Path
from typing import Any

SOURCE_PROJECT = "D:/epic/Projects/FactoryPreview/FactoryProject.uproject"
SOURCE_WORLD = "/Game/Maps/Demonstration.Demonstration"
SOURCE_SCHEMA = "lingtu.factory-bay-authoring.v1"
SCHEMA = "lingtu.factory-collision-mesh-export.v1"
FRAME = "Unreal_left_handed_Xforward_Yright_Zup"
DEFAULT_MANIFEST = (
    Path(__file__).resolve().parents[4] / "build/unreal-tests/factory-bay-export-20261022-01/payload/bay-export.json"
)
EXPECTED_STATIC_ASSETS = 94
SPHERE_RESOLUTION = 16
MAX_VERTICES = 1_000_000
MAX_TRIANGLES = 2_000_000
MAX_PIECES = 20_000
SIMPLE_FIELDS = {
    "sphere_elems": "sphere",
    "box_elems": "box",
    "sphyl_elems": "capsule_z",
    "convex_elems": "convex",
    "tapered_capsule_elems": "tapered_capsule_z",
}
OTHER_FIELDS = ("level_set_elems", "skinned_level_set_elems", "ml_level_set_elems", "skinned_triangle_mesh_elems")
IDENTITY = {"translation_cm": [0.0, 0.0, 0.0], "rotation_xyzw": [0.0, 0.0, 0.0, 1.0], "scale3d": [1.0, 1.0, 1.0]}

# Installed UE 5.8.1 sources, relative to Engine/. No guessed class-name aliases.
API_EVIDENCE = {
    "GeometryScript_SceneUtils.copy_collision_meshes_from_object": "Plugins/Runtime/GeometryScripting/Source/GeometryScriptingCore/Public/GeometryScript/SceneUtilityFunctions.h:56,113-123; ScriptName=GeometryScript_SceneUtils; returns (DynamicMesh, LocalToWorld, Outcome)",
    "GeometryScript_MeshQueries.get_all_vertex_positions/get_all_triangle_indices": "Plugins/Runtime/GeometryScripting/Source/GeometryScriptingCore/Public/GeometryScript/MeshQueryFunctions.h:15,127,220; returns (DynamicMesh, List, HasIDGaps); skip_gaps=False keeps vertex IDs aligned",
    "GeometryScript_List.convert_vector_list_to_array/convert_triangle_list_to_array": "Plugins/Runtime/GeometryScripting/Source/GeometryScriptingCore/Public/GeometryScript/ListUtilityFunctions.h:15,95,188; void plus one out => array, not tuple",
    "copy_semantics": "Plugins/Runtime/GeometryScripting/Source/GeometryScriptingCore/Private/SceneUtilityFunctions.cpp:121-225; StaticMesh BodySetup, identity LocalToWorld; ConvertSimpleCollisionToMeshes; temporary target only",
    "element_transforms_and_disconnected_meshes": "Plugins/Runtime/MeshModelingToolset/Source/ModelingComponents/Private/Physics/ComponentCollisionUtil.cpp:335-670; primitive transforms baked, Convex.GetTransform at 461-479; AppendMesh without weld or aggregate hull; includes all collision modes; tapered capsules not handled",
    "unreferenced_vertices": "Plugins/Runtime/MeshModelingToolset/Source/ModelingComponents/Private/Physics/ComponentCollisionUtil.cpp:464-479 copies all VertexData then IndexData independently; Source/Runtime/GeometryCore/Private/DynamicMeshEditor.cpp:1916-1924,1952 copies all vertices then triangles; no assertion that every vertex is referenced or interior",
    "python_returns": "Plugins/Experimental/PythonScriptPlugin/Source/PythonScriptPlugin/Private/PyGenUtil.cpp:1152-1202; non-bool return then out parameters, tuple for multiple outputs",
    "plugin_flag": "Source/Runtime/Projects/Private/PluginManager.cpp:1546,1570; EnablePlugins=GeometryScripting",
}


def available(value: Any) -> dict:
    return {"status": "available", "value": value}


def unavailable(reason: Any) -> dict:
    return {"status": "unavailable", "reason": str(reason)}


def normalized_path(path: Any) -> str:
    return str(path).replace("\\", "/").rstrip("/").casefold()


def static_assets(source: dict, *, expected_asset_ids: set[str] | None = None) -> list[dict]:
    """Use the original 94 assets, or an explicit independently inventoried set."""
    if (
        source.get("schema") != SOURCE_SCHEMA
        or normalized_path(source.get("source_project")) != normalized_path(SOURCE_PROJECT)
        or source.get("source_world") != SOURCE_WORLD
        or source.get("units") != "Unreal_cm"
        or source.get("frame") != FRAME
        or source.get("source_inventory_matches") is not True
    ):
        raise ValueError("expected matching FactoryPreview bay authoring manifest in asset-local Unreal cm")
    rows = [a for a in source["mesh_assets"] if a["kind"] == "static_mesh"]
    ids = [a["source_id"] for a in rows]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate static source asset IDs")
    if expected_asset_ids is not None:
        if not expected_asset_ids or set(ids) != expected_asset_ids:
            raise ValueError("static assets do not match the explicit inventoried set")
    elif len(rows) != EXPECTED_STATIC_ASSETS:
        raise ValueError("expected exactly 94 distinct prior static source asset IDs")
    if any(not isinstance(sid, str) or not sid.startswith(("/Game/", "/Engine/")) or ":" in sid for sid in ids):
        raise ValueError("expected static asset object paths, not world/component paths")
    return sorted(rows, key=lambda a: a["source_id"])


def split_connected_components(vertices_cm: list, triangles: list) -> list[dict]:
    """Split by shared vertex ID, not position; no welding or geometric hull.

    Vertex slots may have gaps (unreferenced positions). Triangle rows may not:
    the caller requests compact triangle rows while retaining original vertex IDs.
    Every input triangle appears once, with orientation and multiplicity retained.
    """
    if len(vertices_cm) > MAX_VERTICES or len(triangles) > MAX_TRIANGLES:
        raise ValueError("collision mesh budget exceeded")
    for vertex in vertices_cm:
        if len(vertex) != 3 or any(not math.isfinite(v) for v in vertex):
            raise ValueError("vertices must be finite XYZ values")
    parent = {}

    def root(v):
        parent.setdefault(v, v)
        while parent[v] != v:
            parent[v] = parent[parent[v]]
            v = parent[v]
        return v

    for triangle in triangles:
        if len(triangle) != 3 or any(type(i) is not int or not 0 <= i < len(vertices_cm) for i in triangle):
            raise ValueError("triangle must contain three in-range integer vertex IDs")
        if len(set(triangle)) != 3:
            raise ValueError("triangle contains repeated vertex IDs")
        a, b, c = triangle
        for v in (b, c):
            parent[root(v)] = root(a)
    groups = {}
    for tid, triangle in enumerate(triangles):
        groups.setdefault(root(triangle[0]), []).append(tid)
    if len(groups) > MAX_PIECES:
        raise ValueError("collision piece budget exceeded")
    pieces = []
    for tids in sorted(groups.values(), key=lambda group: group[0]):
        vids = sorted({v for tid in tids for v in triangles[tid]})
        remap = {vid: index for index, vid in enumerate(vids)}
        pieces.append(
            {
                "vertices_cm": [list(vertices_cm[v]) for v in vids],
                "indices": [remap[v] for tid in tids for v in triangles[tid]],
                "source_vertex_ids": vids,
                "source_triangle_rows": tids,
            }
        )
    return pieces


def _transform_record(value: Any) -> dict:
    def xyz(v):
        return [float(v.x), float(v.y), float(v.z)]

    q = value.rotation
    result = {
        "translation_cm": xyz(value.translation),
        "rotation_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
        "scale3d": xyz(value.scale3d),
    }
    if any(not math.isfinite(v) for values in result.values() for v in values):
        raise ValueError("nonfinite LocalToWorld")
    if result != IDENTITY:
        raise ValueError("StaticMesh copy must return identity LocalToWorld")
    return result


def _triple(result: Any, mesh: Any, call: str) -> tuple:
    if not isinstance(result, tuple) or len(result) != 3 or result[0] != mesh:
        raise ValueError(f"unexpected {call} binding output; expected (same mesh, out, out)")
    return result


def _inventory(mesh: Any, prior: dict) -> tuple[dict, dict, dict]:
    body = mesh.get_editor_property("body_setup")
    if body is None or body.get_path_name() != prior.get("source_body"):
        raise ValueError("existing BodySetup missing or changed from prior export")
    agg = body.get_editor_property("agg_geom")
    expected = Counter(s["kind"] for s in prior.get("shapes", []))
    if set(expected) - set(SIMPLE_FIELDS.values()):
        raise ValueError("unknown prior primitive kind; no silent omission")
    if (
        prior.get("status") != "source_geometry_read_not_qualified"
        or prior.get("basis") != "existing_UE_BodySetup_AggGeom"
    ):
        raise ValueError("prior simple collision inventory unavailable")
    if any(row.get("field") in SIMPLE_FIELDS for row in prior.get("unavailable", [])):
        raise ValueError("prior simple primitive count unavailable")
    counts, modes = {}, []
    for field, kind in SIMPLE_FIELDS.items():
        elements = list(agg.get_editor_property(field))
        counts[kind] = len(elements)
        if len(elements) != expected[kind]:
            raise ValueError(f"existing {field} count changed from prior export")
        # No original-array ordering is used to assign modes to copied pieces.
        for element in elements:
            try:
                modes.append(available(str(element.get_editor_property("collision_enabled"))))
            except Exception as error:
                modes.append(unavailable(f"collision_enabled: {error}"))
    for field in OTHER_FIELDS:
        count = len(agg.get_editor_property(field))
        counts[field] = count
        if count:
            raise ValueError(f"{field} cannot be mapped to convex simple pieces; no substitution")
    if counts["tapered_capsule_z"]:
        raise ValueError("GeometryScript converter omits tapered capsule elements")
    uniform = unavailable("per-piece source element mapping unavailable; heterogeneous/missing collision modes")
    if modes and all(m.get("status") == "available" and m["value"] == modes[0].get("value") for m in modes):
        uniform = {
            **modes[0],
            "basis": "uniform collision_enabled across ALL live simple elements, not array-order mapping",
        }
    trace = available(str(body.get_editor_property("collision_trace_flag")))
    return counts, uniform, trace


def export_asset(ue: Any, asset: dict) -> dict:
    """One verified binding only; failure never triggers a fallback export."""
    row = {
        "source_id": asset["source_id"],
        "status": "unavailable",
        "pieces": [],
        "basis": "copied_from_existing_simple_collision",
        "space": "asset_local_cm",
        "element_transforms_baked": True,
        "visual_aabb_used": False,
        "qualified_collision": False,
        "source_element_mapping": unavailable(
            "GeometryScript has no per-element ID output; piece index is not AggGeom array index"
        ),
        "convexity": unavailable(
            "not evaluated by producer; parent candidate checker owns closed/convex/volume quality"
        ),
        "original_convex_triangle_counts": unavailable(
            "KConvexElem.index_data not read; copied boundary triangles only"
        ),
    }
    try:
        mesh = ue.load_asset(asset["source_id"])
        if not isinstance(mesh, ue.StaticMesh) or mesh.get_path_name() != asset["source_id"]:
            raise ValueError("exact prior StaticMesh asset did not load")
        counts, mode, trace = _inventory(mesh, asset.get("collision", {}))
        expected = sum(counts[k] for k in SIMPLE_FIELDS.values())
        row.update(
            simple_primitive_counts=counts,
            expected_piece_count=expected,
            collision_enabled=mode,
            collision_trace_flag=trace,
        )
        dynamic = ue.DynamicMesh()
        result = ue.GeometryScript_SceneUtils.copy_collision_meshes_from_object(
            mesh,
            dynamic,
            transform_to_world=False,
            use_complex_collision=False,
            sphere_resolution=SPHERE_RESOLUTION,
        )
        _, local_to_world, outcome = _triple(result, dynamic, "copy_collision_meshes_from_object")
        if outcome != ue.GeometryScriptOutcomePins.SUCCESS:
            raise ValueError(f"copy collision outcome is not SUCCESS: {outcome}")
        row["local_to_world"] = _transform_record(local_to_world)
        queries, lists = ue.GeometryScript_MeshQueries, ue.GeometryScript_List
        # Compacting vertices independently would corrupt triangle vertex IDs.
        _, positions, vertex_gaps = _triple(
            queries.get_all_vertex_positions(dynamic, skip_gaps=False), dynamic, "get_all_vertex_positions"
        )
        _, indices, triangle_gaps = _triple(
            queries.get_all_triangle_indices(dynamic, skip_gaps=True), dynamic, "get_all_triangle_indices"
        )
        if type(vertex_gaps) is not bool or triangle_gaps is not False:
            raise ValueError("unexpected ID-gap flags")
        vectors = lists.convert_vector_list_to_array(positions)
        triplets = lists.convert_triangle_list_to_array(indices)
        if vectors is None or triplets is None:
            raise ValueError("list converters returned no array")
        if len(vectors) > MAX_VERTICES or len(triplets) > MAX_TRIANGLES:
            raise ValueError("collision mesh budget exceeded")
        vertices = [[float(v.x), float(v.y), float(v.z)] for v in vectors]
        triangles = [[v.x, v.y, v.z] for v in triplets]
        # Keep the acquired arrays even if topology/count validation fails below.
        # Nonfinite numbers stay diagnostic strings so the failure report can be written.
        row["raw_mesh"] = {"space": "asset_local_cm", "nonfinite_number_encoding": "strings"}
        for field, values in (("vertices_cm", vertices), ("triangles", triangles)):
            row["raw_mesh"][field] = [
                [value if math.isfinite(value) else str(value) for value in triple] for triple in values
            ]
        pieces = split_connected_components(vertices, triangles)
        row.update(
            copied_vertex_slots=len(vertices),
            copied_triangle_count=len(triangles),
            copied_piece_count=len(pieces),
            vertex_id_gaps=vertex_gaps,
        )
        if len(pieces) != expected:
            raise ValueError(f"full simple hull count mismatch: expected {expected}, copied {len(pieces)}")
        referenced = {v for piece in pieces for v in piece["source_vertex_ids"]}
        row["unreferenced_vertex_ids"] = [v for v in range(len(vertices)) if v not in referenced]
        row["unreferenced_vertex_slots"] = len(row["unreferenced_vertex_ids"])
        row["unreferenced_vertex_classification"] = (
            unavailable(
                "Containment and original element assignment not evaluated; unused positions are retained in raw_mesh. "
                "Vertex ID gaps may include zero placeholders. Matching piece counts do not prove these points harmless."
            )
            if row["unreferenced_vertex_ids"]
            else available("all copied vertex slots referenced by triangles")
        )
        curved = counts["sphere"] + counts["capsule_z"] > 0
        row["approximation"] = (
            unavailable("curved simple primitives tessellated; per-piece analytic mapping unavailable")
            if curved
            else available("existing box/convex triangular boundaries; no simplification or aggregate convex hull")
        )
        row["sphere_resolution"] = SPHERE_RESOLUTION
        for index, piece in enumerate(pieces):
            piece.update(
                source_id=f"{asset['source_id']}#simple-collision-piece:{index:06d}",
                piece_index=index,
                basis=row["basis"],
                space="asset_local_cm",
                element_transforms_baked=True,
                collision_enabled=copy.deepcopy(mode),
                convexity=copy.deepcopy(row["convexity"]),
                qualified_collision=False,
            )
        row.update(status="available", pieces=pieces, full_simple_hull_count_verified=True)
    except Exception as error:
        row.update(
            status="unavailable",
            reason=f"{type(error).__name__}: {error}",
            pieces=[],
            full_simple_hull_count_verified=False,
        )
    return row


def collect_report(ue: Any, source: dict, *, expected_asset_ids: set[str] | None = None) -> dict:
    assets = static_assets(source, expected_asset_ids=expected_asset_ids)
    rows = [export_asset(ue, asset) for asset in assets]
    return {
        "schema": SCHEMA,
        "source_schema": SOURCE_SCHEMA,
        "source_project": SOURCE_PROJECT,
        "source_world": SOURCE_WORLD,
        "units": "Unreal_cm",
        "frame": FRAME,
        "status": "available" if all(r["status"] == "available" for r in rows) else "incomplete",
        "source_world_loaded": False,
        "source_assets_saved": False,
        "collision_created": False,
        "visual_aabb_used": False,
        "qualified_collision": False,
        "published_world": False,
        "piece_id_scope": "asset path + copied connected-component index; stable for same source snapshot/binding; NOT original AggGeom element index",
        "api_evidence": API_EVIDENCE,
        "assets": rows,
        "summary": {
            "requested_static_assets": len(assets),
            "available_assets": sum(r["status"] == "available" for r in rows),
            "unavailable_assets": sum(r["status"] != "available" for r in rows),
            "source_primitive_counts": dict(
                Counter(s["kind"] for a in assets for s in a.get("collision", {}).get("shapes", []))
            ),
            "copied_pieces": sum(len(r["pieces"]) for r in rows),
            "copied_triangles": sum(len(p["indices"]) // 3 for r in rows for p in r["pieces"]),
        },
    }


def merge_supplement(source: dict, report: dict, *, expected_asset_ids: set[str] | None = None) -> dict:
    """Return an independently copied candidate input; never overwrite a file.

    Replace whole available static collision sets, not just convex array slots:
    copied pieces have already baked element transforms. Unavailable assets retain
    original geometry and explicit supplemental failure. No quality is asserted.
    """
    expected = {a["source_id"] for a in static_assets(source, expected_asset_ids=expected_asset_ids)}
    rows = report.get("assets", [])
    if (
        report.get("schema") != SCHEMA
        or report.get("units") != "Unreal_cm"
        or report.get("frame") != FRAME
        or report.get("source_project") != SOURCE_PROJECT
        or report.get("source_world") != SOURCE_WORLD
        or len(rows) != len(expected)
        or {r["source_id"] for r in rows} != expected
    ):
        raise ValueError("supplement must match the exact prior static asset set and source frame")
    merged = copy.deepcopy(source)
    by_id = {r["source_id"]: r for r in rows}
    for asset in merged["mesh_assets"]:
        if asset["kind"] != "static_mesh":
            continue
        row = by_id[asset["source_id"]]
        collision = asset["collision"]
        collision["supplemental_export"] = {k: copy.deepcopy(v) for k, v in row.items() if k != "pieces"}
        if row["status"] != "available":
            continue
        if (
            row.get("full_simple_hull_count_verified") is not True
            or len(row["pieces"]) != row.get("expected_piece_count")
            or row.get("space") != "asset_local_cm"
            or row.get("element_transforms_baked") is not True
        ):
            raise ValueError("supplement has incomplete hulls or unbaked/unknown coordinates")
        pieces = row["pieces"]
        if [p["source_id"] for p in pieces] != [
            f"{asset['source_id']}#simple-collision-piece:{i:06d}" for i in range(len(pieces))
        ]:
            raise ValueError("unexpected copied piece identity")
        collision["shapes"] = [
            {
                "source_id": p["source_id"],
                "kind": "convex",
                "source_kind": "copied_simple_collision_piece",
                "collision_enabled": copy.deepcopy(p["collision_enabled"]),
                "geometry": available(
                    {
                        "vertices_cm": copy.deepcopy(p["vertices_cm"]),
                        "indices": available(copy.deepcopy(p["indices"])),
                        "element_transform": available(copy.deepcopy(IDENTITY)),
                    }
                ),
                "source_element_mapping": copy.deepcopy(row["source_element_mapping"]),
                "convexity": copy.deepcopy(row["convexity"]),
                "approximation": copy.deepcopy(row["approximation"]),
            }
            for p in pieces
        ]
        collision.update(
            status="source_geometry_read_not_qualified",
            space="body_local_cm",
            collision_trace_flag=copy.deepcopy(row["collision_trace_flag"]),
            qualified_collision=False,
        )
    merged["collision_mesh_supplement"] = {
        "schema": SCHEMA,
        "summary": copy.deepcopy(report["summary"]),
        "qualified_collision": False,
    }
    return merged


def main() -> None:
    import unreal

    if normalized_path(unreal.Paths.get_project_file_path()) != normalized_path(SOURCE_PROJECT):
        raise RuntimeError("run only in the isolated FactoryPreview source project")
    output = Path(os.environ["LINGTU_FACTORY_COLLISION_EXPORT_DIR"]).resolve()
    source_dir = normalized_path(Path(SOURCE_PROJECT).parent)
    if normalized_path(output) == source_dir or normalized_path(output).startswith(source_dir + "/"):
        raise ValueError("output must be outside the source project")
    source_path = Path(os.environ.get("LINGTU_FACTORY_COLLISION_BAY_MANIFEST", str(DEFAULT_MANIFEST))).resolve()
    source = json.loads(source_path.read_text(encoding="utf-8"))
    static_assets(source)
    output.mkdir(parents=True, exist_ok=False)
    report = collect_report(unreal, source)
    report["source_manifest"] = str(source_path)
    with (output / "collision-mesh-export.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    unreal.log(
        f"LINGTU_FACTORY_COLLISION_EXPORT_COMPLETE status={report['status']} summary={report['summary']} saved_source=false qualified_collision=false"
    )
    if report["status"] != "available":
        raise RuntimeError("supplement incomplete; see per-asset unavailable reasons in collision-mesh-export.json")


if __name__ == "__main__":
    main()
