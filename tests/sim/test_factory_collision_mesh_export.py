"""Offline producer contracts only: no Unreal process, world, save or build."""

import copy
import json
import sys
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import pytest
from sim.tools.worlds.factory_environment_collection import bay_collision_mesh_export as export

TETRA = [[10, 20, 30], [110, 20, 30], [10, 120, 30], [10, 20, 130]]
FACES = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]]
MODE = "<CollisionEnabled.QUERY_AND_PHYSICS: 3>"
REAL_EXPORT = (
    Path(__file__).resolve().parents[2] / "build/unreal-tests/factory-bay-export-20261022-01/payload/bay-export.json"
)


def test_context_extension_uses_exact_explicit_asset_set_not_the_old_94_count():
    source = source_fixture()
    source["mesh_assets"] = source["mesh_assets"][:2]
    ids = {asset["source_id"] for asset in source["mesh_assets"]}
    assert len(export.static_assets(source, expected_asset_ids=ids)) == 2
    with pytest.raises(ValueError):
        export.static_assets(source, expected_asset_ids=ids | {"/Game/Missing.Missing"})
    with pytest.raises(ValueError):
        export.static_assets(source)


def vector(values):
    return SimpleNamespace(**dict(zip(("x", "y", "z"), values)))


def identity():
    return SimpleNamespace(
        translation=vector([0, 0, 0]), scale3d=vector([1, 1, 1]), rotation=SimpleNamespace(x=0, y=0, z=0, w=1)
    )


class ReadOnly:
    def __init__(self, path, **properties):
        self.path = path
        self.properties = properties

    def get_path_name(self):
        return self.path

    def get_editor_property(self, field):
        if field not in self.properties:
            raise AttributeError(f"unexposed: {field}")
        return self.properties[field]


class StaticMesh(ReadOnly):
    pass


def source_fixture(kinds=("convex",)):
    assets = []
    for index in range(94):
        sid = f"/Game/Factory/Mesh{index}.Mesh{index}"
        assets.append(
            {
                "source_id": sid,
                "kind": "static_mesh",
                "collision": {
                    "source_body": sid + ":BodySetup_0",
                    "basis": "existing_UE_BodySetup_AggGeom",
                    "space": "body_local_cm",
                    "status": "source_geometry_read_not_qualified",
                    "unavailable": [],
                    "collision_trace_flag": export.available("CTF_USE_SIMPLE_AND_COMPLEX"),
                    "shapes": [
                        {
                            "source_id": f"{sid}:Body#{kind}:{i}",
                            "kind": kind,
                            "collision_enabled": export.available(MODE),
                            "geometry": export.unavailable("vertex_data not exposed"),
                        }
                        for i, kind in enumerate(kinds)
                    ],
                },
            }
        )
    return {
        "schema": export.SOURCE_SCHEMA,
        "source_project": export.SOURCE_PROJECT,
        "source_world": export.SOURCE_WORLD,
        "units": "Unreal_cm",
        "frame": export.FRAME,
        "source_inventory_matches": True,
        "mesh_assets": assets,
        "actors": [],
        "bounds_min_cm": [0, -1000, 0],
        "bounds_max_cm": [1000, 1000, 1000],
    }


class FakeUE:
    """Only the exact ScriptName bindings exist; no source mutation methods."""

    StaticMesh = StaticMesh
    GeometryScriptOutcomePins = SimpleNamespace(SUCCESS="success", FAILURE="failure")

    def __init__(self, source):
        self.calls = []
        self.overrides = {}
        self.meshes = {}
        self.targets = []
        for asset in source["mesh_assets"]:
            if asset["kind"] != "static_mesh":
                continue
            collision = asset["collision"]
            counts = Counter(s["kind"] for s in collision["shapes"])
            fields = {
                field: [ReadOnly("element", collision_enabled=MODE) for _ in range(counts[kind])]
                for field, kind in export.SIMPLE_FIELDS.items()
            }
            fields.update({field: [] for field in export.OTHER_FIELDS})
            body = ReadOnly(
                collision["source_body"],
                agg_geom=ReadOnly("agg", **fields),
                collision_trace_flag="CTF_USE_SIMPLE_AND_COMPLEX",
            )
            mesh = StaticMesh(asset["source_id"], body_setup=body)
            # Even coincident hulls retain distinct vertex IDs. No geometric weld.
            mesh.vertices = [list(v) for _ in collision["shapes"] for v in TETRA]
            mesh.triangles = [[v + 4 * i for v in face] for i, _ in enumerate(collision["shapes"]) for face in FACES]
            self.meshes[asset["source_id"]] = mesh
        self.Paths = SimpleNamespace(get_project_file_path=lambda: export.SOURCE_PROJECT)
        self.GeometryScript_SceneUtils = SimpleNamespace(copy_collision_meshes_from_object=self.copy_collision)
        self.GeometryScript_MeshQueries = SimpleNamespace(
            get_all_vertex_positions=self.positions, get_all_triangle_indices=self.triangles
        )
        self.GeometryScript_List = SimpleNamespace(
            convert_vector_list_to_array=lambda values: self.result("vectors", lambda: [vector(v) for v in values]),
            convert_triangle_list_to_array=lambda values: self.result("indices", lambda: [vector(v) for v in values]),
        )

    def result(self, name, default):
        self.calls.append(name)
        if name in self.overrides:
            value = self.overrides[name]
            if isinstance(value, Exception):
                raise value
            return value
        return default()

    def load_asset(self, path):
        self.calls.append(("load", path))
        return self.meshes[path]

    def DynamicMesh(self):
        target = SimpleNamespace()
        self.targets.append(target)
        return target

    def copy_collision(self, mesh, target, *, transform_to_world, use_complex_collision, sphere_resolution):
        assert transform_to_world is False
        assert use_complex_collision is False
        assert sphere_resolution == 16
        target.mesh = mesh
        return self.result("copy", lambda: (target, identity(), "success"))

    def positions(self, target, *, skip_gaps):
        assert skip_gaps is False  # Do not compact vertex IDs independently.
        return self.result("positions", lambda: (target, target.mesh.vertices, False))

    def triangles(self, target, *, skip_gaps):
        assert skip_gaps is True
        return self.result("triangles", lambda: (target, target.mesh.triangles, False))

    def log(self, message):
        self.calls.append(("log", message))


def test_components_keep_coincident_hulls_separate_with_dense_indices_and_no_lost_triangles():
    vertices = [[999, 999, 999], *TETRA, [0, 0, 0], *TETRA]
    faces = [[i + offset for i in face] for offset in (1, 6) for face in FACES]
    faces = faces[::2] + faces[1::2]
    pieces = export.split_connected_components(vertices, faces)
    assert len(pieces) == 2
    assert sorted(tid for p in pieces for tid in p["source_triangle_rows"]) == list(range(8))
    for piece in pieces:
        assert set(piece["indices"]) == set(range(4))
        assert piece["vertices_cm"] == TETRA
        restored = [
            [piece["source_vertex_ids"][i] for i in piece["indices"][j : j + 3]]
            for j in range(0, len(piece["indices"]), 3)
        ]
        assert restored == [faces[tid] for tid in piece["source_triangle_rows"]]


def test_components_union_transitively_keep_duplicate_triangles_and_winding():
    triangles = [[0, 1, 2], [3, 4, 5], [2, 3, 6], [2, 1, 0], [0, 1, 2]]
    pieces = export.split_connected_components([[i, i * 2, 0] for i in range(7)], triangles)
    assert len(pieces) == 1
    assert pieces[0]["indices"] == [v for t in triangles for v in t]
    assert pieces[0]["source_triangle_rows"] == list(range(5))
    assert export.split_connected_components([], []) == []


@pytest.mark.parametrize(
    "triangles", [[[0, 1, -1]], [[0, 1, 4]], [[0, 1]], [[0, True, 2]], [[0, 1.0, 2]], [[0, 0, 1]], [[-1, -1, -1]]]
)
def test_invalid_triangles_fail_instead_of_being_dropped(triangles):
    with pytest.raises(ValueError):
        export.split_connected_components(TETRA, triangles)


def test_nonfinite_vertex_and_budgets_fail(monkeypatch):
    with pytest.raises(ValueError, match="finite"):
        export.split_connected_components([[float("nan"), 0, 0]], [])
    monkeypatch.setattr(export, "MAX_VERTICES", 3)
    with pytest.raises(ValueError, match="budget"):
        export.split_connected_components(TETRA, FACES)


def test_one_exact_binding_bakes_no_second_transform_and_preserves_full_count():
    source = source_fixture(("box", "convex"))
    ue = FakeUE(source)
    before = copy.deepcopy(source)
    row = export.export_asset(ue, source["mesh_assets"][0])
    assert row["status"] == "available"
    assert row["copied_piece_count"] == row["expected_piece_count"] == 2
    assert row["copied_triangle_count"] == 8
    assert row["convexity"]["status"] == "unavailable"
    assert row["source_element_mapping"]["status"] == "unavailable"
    assert [p["piece_index"] for p in row["pieces"]] == [0, 1]
    assert all(p["vertices_cm"] == TETRA for p in row["pieces"])
    assert row["collision_enabled"]["value"] == MODE
    assert ue.calls.count("copy") == ue.calls.count("positions") == ue.calls.count("triangles") == 1
    assert len(ue.targets) == 1
    assert source == before


@pytest.mark.parametrize(
    "phase,value",
    [
        ("copy", None),
        ("copy", (None, None)),
        ("copy", (None, None, "success")),
        ("copy", RuntimeError("binding failure")),
        ("positions", None),
        ("positions", (None, [], False)),
        ("triangles", None),
        ("triangles", (None, [], False)),
        ("vectors", None),
        ("indices", None),
        ("indices", [vector([0, 1, 99])]),
        ("indices", [vector([0, 1.5, 2])]),
        ("vectors", [vector([float("inf"), 0, 0])]),
    ],
)
def test_bad_api_outputs_fail_closed_without_alternative_binding(phase, value):
    source = source_fixture()
    ue = FakeUE(source)
    ue.overrides[phase] = value
    row = export.export_asset(ue, source["mesh_assets"][0])
    assert row["status"] == "unavailable"
    assert row["pieces"] == []
    assert row["full_simple_hull_count_verified"] is False
    assert ue.calls.count("copy") == 1
    assert len(ue.targets) == 1


@pytest.mark.parametrize("failure", ["outcome", "transform", "count", "triangle_gaps", "vertex_gaps_type"])
def test_semantic_api_failures_are_not_available(failure):
    source = source_fixture()
    ue = FakeUE(source)
    original_copy = ue.copy_collision

    def altered_copy(mesh, target, **kwargs):
        _, pose, outcome = original_copy(mesh, target, **kwargs)
        if failure == "outcome":
            outcome = "failure"
        if failure == "transform":
            pose.translation.x = 100
        return target, pose, outcome

    ue.GeometryScript_SceneUtils.copy_collision_meshes_from_object = altered_copy
    mesh = next(iter(ue.meshes.values()))
    if failure == "count":
        mesh.vertices *= 2
        mesh.triangles += [[i + 4 for i in face] for face in FACES]
    if failure == "triangle_gaps":
        ue.GeometryScript_MeshQueries.get_all_triangle_indices = lambda target, **kwargs: (target, mesh.triangles, True)
    if failure == "vertex_gaps_type":
        ue.GeometryScript_MeshQueries.get_all_vertex_positions = lambda target, **kwargs: (
            target,
            mesh.vertices,
            "false",
        )
    row = export.export_asset(ue, source["mesh_assets"][0])
    assert row["status"] == "unavailable" and not row["pieces"]


def test_unused_vertices_are_preserved_for_review_without_failing_boundary_export():
    source = source_fixture()
    ue = FakeUE(source)
    mesh = next(iter(ue.meshes.values()))
    # One interior and one exterior point: producer must not assume either is harmless.
    mesh.vertices.extend([[20, 30, 40], [999, 999, 999]])
    report = export.collect_report(ue, source)
    row = report["assets"][0]
    assert report["status"] == row["status"] == "available"
    assert row["qualified_collision"] is False
    assert row["convexity"]["status"] == "unavailable"
    assert row["full_simple_hull_count_verified"] is True
    assert row["unreferenced_vertex_slots"] == 2
    assert row["unreferenced_vertex_ids"] == [4, 5]
    assert row["unreferenced_vertex_classification"]["status"] == "unavailable"
    assert row["raw_mesh"]["vertices_cm"] == mesh.vertices
    assert row["raw_mesh"]["triangles"] == FACES
    assert row["raw_mesh"]["space"] == "asset_local_cm"
    piece = row["pieces"][0]
    assert piece["vertices_cm"] == TETRA
    assert piece["indices"] == [v for face in FACES for v in face]
    assert piece["source_triangle_rows"] == list(range(4))
    # The independent candidate input must retain these diagnostics too.
    merged = export.merge_supplement(source, report)
    diagnostics = merged["mesh_assets"][0]["collision"]["supplemental_export"]
    assert diagnostics["raw_mesh"] == row["raw_mesh"]
    assert diagnostics["unreferenced_vertex_ids"] == [4, 5]


@pytest.mark.parametrize("failure", ["piece_count", "invalid_index", "nonfinite_position"])
def test_failed_export_retains_serializable_raw_diagnostic_geometry(failure):
    source = source_fixture()
    ue = FakeUE(source)
    mesh = next(iter(ue.meshes.values()))
    if failure == "piece_count":
        mesh.vertices += copy.deepcopy(TETRA)
        mesh.triangles += [[v + 4 for v in face] for face in FACES]
    elif failure == "invalid_index":
        mesh.triangles.append([0, 1, 99])
    else:
        mesh.vertices.append([float("inf"), 0, 0])
    report = export.collect_report(ue, source)
    row = report["assets"][0]
    assert report["status"] == "incomplete"
    assert row["status"] == "unavailable" and row["pieces"] == []
    assert row["full_simple_hull_count_verified"] is False
    expected_vertices = copy.deepcopy(mesh.vertices)
    if failure == "nonfinite_position":
        expected_vertices[-1][0] = "inf"
    assert row["raw_mesh"]["vertices_cm"] == expected_vertices
    assert row["raw_mesh"]["triangles"] == mesh.triangles
    json.dumps(report, allow_nan=False)
    merged = export.merge_supplement(source, report)
    collision = merged["mesh_assets"][0]["collision"]
    assert collision["shapes"] == source["mesh_assets"][0]["collision"]["shapes"]
    assert collision["supplemental_export"]["raw_mesh"] == row["raw_mesh"]


def test_vertex_id_gaps_are_retained_until_component_remap():
    source = source_fixture()
    ue = FakeUE(source)
    mesh = next(iter(ue.meshes.values()))
    mesh.vertices.insert(0, [0, 0, 0])
    mesh.triangles = [[i + 1 for i in face] for face in FACES]
    ue.GeometryScript_MeshQueries.get_all_vertex_positions = lambda target, **kwargs: (target, mesh.vertices, True)
    row = export.export_asset(ue, source["mesh_assets"][0])
    assert row["status"] == "available"
    assert row["pieces"][0]["vertices_cm"] == TETRA
    assert row["pieces"][0]["source_vertex_ids"] == [1, 2, 3, 4]
    assert row["unreferenced_vertex_slots"] == 1


@pytest.mark.parametrize(
    "failure", ["changed_count", "tapered", "levelset", "missing_array", "wrong_body", "wrong_asset"]
)
def test_source_inventory_failures_do_not_copy_any_collision(failure):
    source = source_fixture(("tapered_capsule_z",) if failure == "tapered" else ("convex",))
    ue = FakeUE(source)
    mesh = next(iter(ue.meshes.values()))
    body = mesh.properties["body_setup"]
    agg = body.properties["agg_geom"].properties
    if failure == "changed_count":
        agg["convex_elems"] = []
    if failure == "levelset":
        agg["level_set_elems"] = [ReadOnly("levelset")]
    if failure == "missing_array":
        del agg["sphere_elems"]
    if failure == "wrong_body":
        body.path += "changed"
    if failure == "wrong_asset":
        mesh.path += "changed"
    row = export.export_asset(ue, source["mesh_assets"][0])
    assert row["status"] == "unavailable" and not row["pieces"]
    assert "copy" not in ue.calls


def test_heterogeneous_or_missing_modes_are_explicit_and_not_order_mapped():
    source = source_fixture(("box", "convex"))
    ue = FakeUE(source)
    agg = next(iter(ue.meshes.values())).properties["body_setup"].properties["agg_geom"]
    for properties in ({"collision_enabled": "NO_COLLISION"}, {}):
        agg.properties["box_elems"][0].properties = properties
        row = export.export_asset(ue, source["mesh_assets"][0])
        assert row["status"] == "available"
        assert all(p["collision_enabled"]["status"] == "unavailable" for p in row["pieces"])


def test_curved_approximation_not_claimed_exact():
    source = source_fixture(("sphere", "capsule_z"))
    row = export.export_asset(FakeUE(source), source["mesh_assets"][0])
    assert row["status"] == "available"
    assert row["approximation"]["status"] == "unavailable"
    assert row["visual_aabb_used"] is False


def test_exact_94_load_set_report_and_merge_are_independent_candidate_input():
    from sim.tools.worlds.factory_environment_collection import collision_candidate as candidate

    source = source_fixture(("box", "convex"))
    source["actors"] = [
        {
            "source_id": "/actor",
            "components": [
                {
                    "source_id": "/actor/component",
                    "kind": "static_mesh",
                    "class": "StaticMeshComponent",
                    "mesh_asset": source["mesh_assets"][0]["source_id"],
                    "disposition": "selected_for_authoring_review_not_qualified",
                    "collision_enabled": export.available(MODE),
                    "world_transform": {
                        "translation_cm": [400, 500, 600],
                        "rotation_xyzw": [0, 0, 0, 1],
                        "scale3d": [2, 3, 4],
                    },
                    "visual_bounds_min_cm": [-999999, -999999, -999999],
                    "visual_bounds_max_cm": [999999, 999999, 999999],
                }
            ],
        }
    ]
    ue = FakeUE(source)
    before = copy.deepcopy(source)
    report = export.collect_report(ue, source)
    assert {call[1] for call in ue.calls if isinstance(call, tuple) and call[0] == "load"} == {
        a["source_id"] for a in source["mesh_assets"]
    }
    assert report["summary"]["requested_static_assets"] == 94
    assert report["summary"]["copied_pieces"] == 188
    assert report["status"] == "available"
    merged = export.merge_supplement(source, report)
    assert source == before
    shape = merged["mesh_assets"][0]["collision"]["shapes"][0]
    vertices, matrix, indices = candidate.primitive_vertices(shape)
    assert vertices.tolist() == TETRA
    assert indices["status"] == "validated_closed_convex_boundary"
    assert matrix.tolist() == [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    _, result = candidate.convert_manifest(merged)
    assert result["qualified_collision"] is False
    assert result["summary"]["accepted_geometries"] == 2
    for geometry in result["geometries"]:
        points = [[v[i] + geometry["mesh_origin_m"][i] for i in range(3)] for v in geometry["mesh_vertices_m"]]
        for observed, raw in zip(points, TETRA):
            assert observed == pytest.approx(
                [(2 * raw[0] + 400) / 100, -(3 * raw[1] + 500) / 100, (4 * raw[2] + 600) / 100]
            )
    shape["geometry"]["value"]["vertices_cm"][0][0] = -999
    assert report["assets"][0]["pieces"][0]["vertices_cm"] == TETRA


@pytest.mark.parametrize("boundary,reason", [("open", "closed"), ("nonconvex", "convex supporting")])
def test_unused_vertices_do_not_bypass_parent_boundary_quality_checks(boundary, reason):
    from sim.tools.worlds.factory_environment_collection import collision_candidate as candidate

    source = source_fixture()
    ue = FakeUE(source)
    mesh = next(iter(ue.meshes.values()))
    if boundary == "open":
        mesh.triangles.pop()
    else:
        # Replace one tetrahedron face with an inward dent: closed but concave.
        mesh.vertices.append([20, 30, 40])
        mesh.triangles = [[0, 2, 4], [2, 1, 4], [1, 0, 4], *copy.deepcopy(FACES[1:])]
    mesh.vertices.append([999, 999, 999])
    report = export.collect_report(ue, source)
    assert report["assets"][0]["status"] == "available"
    assert report["assets"][0]["copied_triangle_count"] == len(mesh.triangles)
    assert report["assets"][0]["raw_mesh"]["triangles"] == mesh.triangles
    assert report["assets"][0]["unreferenced_vertex_slots"] == 1
    assert report["assets"][0]["convexity"]["status"] == "unavailable"
    merged = export.merge_supplement(source, report)
    with pytest.raises(candidate.GeometryError, match=reason):
        candidate.primitive_vertices(merged["mesh_assets"][0]["collision"]["shapes"][0])


def test_incomplete_supplement_keeps_original_asset_and_explains_failure():
    source = source_fixture()
    ue = FakeUE(source)
    ue.overrides["copy"] = None
    report = export.collect_report(ue, source)
    assert report["status"] == "incomplete" and report["summary"]["unavailable_assets"] == 94
    merged = export.merge_supplement(source, report)
    assert merged["mesh_assets"][0]["collision"]["shapes"] == source["mesh_assets"][0]["collision"]["shapes"]
    assert merged["mesh_assets"][0]["collision"]["supplemental_export"]["status"] == "unavailable"


@pytest.mark.parametrize("failure", ["schema", "frame", "count", "duplicate", "source_world"])
def test_invalid_manifest_rejected_before_any_load(failure):
    source = source_fixture()
    ue = FakeUE(source)
    if failure in {"schema", "frame", "source_world"}:
        source[failure] = "wrong"
    if failure == "count":
        source["mesh_assets"].pop()
    if failure == "duplicate":
        source["mesh_assets"][0] = source["mesh_assets"][1]
    with pytest.raises(ValueError):
        export.collect_report(ue, source)
    assert ue.calls == []


@pytest.mark.parametrize("failure", ["asset_set", "full_count", "space", "piece_id"])
def test_merge_refuses_unmatched_or_incomplete_geometry(failure):
    source = source_fixture()
    report = export.collect_report(FakeUE(source), source)
    if failure == "asset_set":
        report["assets"].pop()
    if failure == "full_count":
        report["assets"][0]["full_simple_hull_count_verified"] = False
    if failure == "space":
        report["assets"][0]["space"] = "world_cm"
    if failure == "piece_id":
        report["assets"][0]["pieces"][0]["source_id"] = "wrong"
    with pytest.raises(ValueError):
        export.merge_supplement(source, report)


@pytest.mark.parametrize("bad_api", [False, True])
def test_main_new_directory_only_no_original_overwrite_and_durable_failure_report(tmp_path, monkeypatch, bad_api):
    source = source_fixture()
    ue = FakeUE(source)
    if bad_api:
        ue.overrides["copy"] = None
    source_path = tmp_path / "bay-export.json"
    original = json.dumps(source)
    source_path.write_text(original, encoding="utf-8")
    output = tmp_path / "supplement"
    monkeypatch.setitem(sys.modules, "unreal", ue)
    monkeypatch.setenv("LINGTU_FACTORY_COLLISION_BAY_MANIFEST", str(source_path))
    monkeypatch.setenv("LINGTU_FACTORY_COLLISION_EXPORT_DIR", str(output))
    if bad_api:
        with pytest.raises(RuntimeError, match="incomplete"):
            export.main()
    else:
        export.main()
    report = json.loads((output / "collision-mesh-export.json").read_text(encoding="utf-8"))
    assert report["status"] == ("incomplete" if bad_api else "available")
    assert report["source_world_loaded"] is False
    assert source_path.read_text(encoding="utf-8") == original
    assert [p.name for p in output.iterdir()] == ["collision-mesh-export.json"]
    loads_before = len(ue.calls)
    with pytest.raises(FileExistsError):
        export.main()
    assert len(ue.calls) == loads_before


@pytest.mark.skipif(not REAL_EXPORT.exists(), reason="local real authoring evidence not present")
def test_real_manifest_inventory_counts_and_exact_load_contract_with_fake_geometry():
    source = json.loads(REAL_EXPORT.read_text(encoding="utf-8"))
    assert export.DEFAULT_MANIFEST == REAL_EXPORT
    assets = export.static_assets(source)
    assert len(assets) == 94
    ue = FakeUE(source)
    report = export.collect_report(ue, source)
    assert report["status"] == "available"
    assert report["summary"]["source_primitive_counts"] == {"box": 61, "convex": 264}
    assert report["summary"]["copied_pieces"] == 325
    assert report["summary"]["available_assets"] == 94
    assert report["summary"]["copied_triangles"] == 1300
    # Synthetic tetrahedra prove inventory/producer behavior, not real UE geometry.
    assert all(r["convexity"]["status"] == "unavailable" for r in report["assets"])
