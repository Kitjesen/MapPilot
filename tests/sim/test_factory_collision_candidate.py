"""Source geometry, frame, coverage and native MuJoCo checks; no robot motion."""

import copy
import json
from collections import Counter
from itertools import product
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
from scipy.spatial import ConvexHull
from sim.tools.worlds.factory_environment_collection import collision_candidate as candidate

REAL_EXPORT = (
    Path(__file__).resolve().parents[2] / "build/unreal-tests/factory-bay-export-20261022-01/payload/bay-export.json"
)


def available(value):
    return {"status": "available", "value": value}


def transform(translation=(0, 0, 0), scale=(1, 1, 1), quat=(0, 0, 0, 1)):
    return {"translation_cm": list(translation), "scale3d": list(scale), "rotation_xyzw": list(quat)}


def box(pyr=(0, 0, 0), center=(0, 0, 0), size=(200, 100, 40)):
    return {
        "source_id": "/mesh:Body#box_elems:0",
        "kind": "box",
        "collision_enabled": available("<CollisionEnabled.QUERY_AND_PHYSICS: 3>"),
        "geometry": available(
            {"center_cm": list(center), "full_size_cm": list(size), "rotation_degrees_pyr": list(pyr)}
        ),
    }


def convex(*, indices=None, element=None):
    return {
        "source_id": "/mesh:Body#convex_elems:0",
        "kind": "convex",
        "collision_enabled": available("QUERY_AND_PHYSICS"),
        "geometry": available(
            {
                "vertices_cm": [[0, 0, 0], [100, 0, 0], [0, 100, 0], [0, 0, 100]],
                "indices": available([0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3] if indices is None else indices),
                "element_transform": available(element or transform()),
            }
        ),
    }


def source_fixture(shapes=None, pose=None):
    return {
        "schema": candidate.SOURCE_SCHEMA,
        "units": "Unreal_cm",
        "frame": candidate.SOURCE_FRAME,
        "source_inventory_matches": True,
        "source_project": "/source/project",
        "source_world": "/source/world",
        "bounds_min_cm": [-500, -500, -500],
        "bounds_max_cm": [500, 500, 500],
        "actors": [
            {
                "source_id": "/actor",
                "components": [
                    {
                        "source_id": "/actor/component",
                        "kind": "static_mesh",
                        "class": "StaticMeshComponent",
                        "mesh_asset": "/mesh",
                        "world_transform": pose or transform(),
                        "disposition": "selected_for_authoring_review_not_qualified",
                        "collision_enabled": available("QUERY_AND_PHYSICS"),
                        # Deliberately false visual bounds: these MUST NOT affect geometry.
                        "visual_bounds_min_cm": [-999999, -999999, -999999],
                        "visual_bounds_max_cm": [999999, 999999, 999999],
                    }
                ],
            }
        ],
        "mesh_assets": [
            {
                "source_id": "/mesh",
                "kind": "static_mesh",
                "collision": {
                    "basis": "existing_UE_BodySetup_AggGeom",
                    "space": "body_local_cm",
                    "source_body": "/mesh:Body",
                    "status": "source_geometry_read_not_qualified",
                    "shapes": shapes if shapes is not None else [box()],
                    "collision_trace_flag": available("CTF_USE_SIMPLE_AND_COMPLEX"),
                    "unavailable": [],
                },
            }
        ],
    }


def vertices(record):
    return np.array(record["mesh_vertices_m"]) + record["mesh_origin_m"]


def axis_rotate(points, axis, degrees):
    """Independent Rodrigues oracle, not the converter's Euler/matrix formula."""
    axis = np.eye(3)[axis]
    angle = np.deg2rad(degrees)
    return (
        points * np.cos(angle)
        + np.cross(axis, points) * np.sin(angle)
        + np.outer(points @ axis, axis) * (1 - np.cos(angle))
    )


def ue_rotator_oracle(points, pyr):
    p, y, r = pyr
    return axis_rotate(axis_rotate(axis_rotate(points, 0, -r), 1, -p), 2, y)


@pytest.mark.parametrize(
    ("pyr", "expected_x"),
    [
        ((0, 90, 0), (0, 1, 0)),
        ((90, 0, 0), (0, 0, 1)),
        ((0, 0, 90), (1, 0, 0)),
        ((90, 90, 90), (0, 0, 1)),
    ],
)
def test_installed_ue_box_rotation_axes(pyr, expected_x):
    assert candidate.apply_matrix(
        np.array([[1, 0, 0]]),
        candidate.box_matrix(
            {
                "rotation_degrees_pyr": pyr,
                "center_cm": [0, 0, 0],
            }
        ),
    )[0] == pytest.approx(expected_x, abs=1e-14)
    if pyr == (0, 0, 90):
        assert candidate.apply_matrix(
            np.array([[0, 1, 0]]),
            candidate.box_matrix(
                {
                    "rotation_degrees_pyr": pyr,
                    "center_cm": [0, 0, 0],
                }
            ),
        )[0] == pytest.approx([0, 0, -1], abs=1e-14)


@pytest.mark.parametrize("scale", [(2, 3, 0.5), (-2, 3, 0.5), (2, -3, -0.5)])
def test_box_full_rotation_before_component_nonuniform_reflected_scale(scale):
    pyr, center = (27, 41, -63), (17, -25, 31)
    angle = np.deg2rad(35)
    source = source_fixture(
        [box(pyr, center)], transform((1000, 2000, 3000), scale, (0, 0, np.sin(angle / 2), np.cos(angle / 2)))
    )
    snapshot = copy.deepcopy(source)
    _, result = candidate.convert_manifest(source)
    raw = np.array(list(product((-0.5, 0.5), repeat=3))) * [200, 100, 40]
    asset = ue_rotator_oracle(raw, pyr) + center
    world = axis_rotate(asset * scale, 2, 35) + np.array([1000, 2000, 3000])
    expected = world * [0.01, -0.01, 0.01]
    record = result["geometries"][0]
    np.testing.assert_allclose(vertices(record), expected, atol=1e-12)
    assert record["collision_bounds"]["min_m"] == pytest.approx(expected.min(axis=0))
    assert record["collision_bounds"]["max_m"] == pytest.approx(expected.max(axis=0))
    assert result["collision_bounds"] == record["collision_bounds"]
    assert source == snapshot
    # R_component R_element S_component is wrong when scale and element rotation do not commute.
    wrong = axis_rotate(ue_rotator_oracle(raw * scale, pyr) + np.array(center) * scale, 2, 35)
    assert not np.allclose(wrong + np.array([1000, 2000, 3000]), world)


def test_convex_element_transform_then_component_transform_and_reflection():
    q = np.sqrt(0.5)
    shape = convex(element=transform((20, 30, 40), (0.5, -2, 3), (q, 0, 0, q)))
    source = source_fixture([shape], transform((100, -200, 300), (-2, 3, 4), (0, 0, q, q)))
    _, result = candidate.convert_manifest(source)
    raw = np.array(shape["geometry"]["value"]["vertices_cm"])
    asset = axis_rotate(raw * [0.5, -2, 3], 0, 90) + np.array([20, 30, 40])
    expected = (axis_rotate(asset * [-2, 3, 4], 2, 90) + np.array([100, -200, 300])) * [0.01, -0.01, 0.01]
    record = result["geometries"][0]
    np.testing.assert_allclose(vertices(record), expected, atol=1e-14)
    assert record["index_validation"]["status"] == "validated_closed_convex_boundary"
    assert record["primitive_source_id"] == shape["source_id"]
    assert record["source_geometry"] == shape["geometry"]["value"]


@pytest.mark.parametrize(
    "indices,reason",
    [
        ([0, 1], "triangle count"),
        ([0, 1, 9], "range"),
        ([0.0, 1, 2], "type"),
        ([0, 1, 2], "closed"),
        ([0, 0, 1], "repeated"),
        ([0, 2, 1, 0, 1, 3, 1, 2, 3, 0, 2, 1], "duplicate"),
    ],
)
def test_invalid_convex_indices_rejected_without_hull_fallback(indices, reason):
    _, result = candidate.convert_manifest(source_fixture([convex(indices=indices)]))
    assert not result["geometries"]
    assert reason in result["components"][0]["rejected_primitives"][0]["reason"]


def test_nonconvex_source_triangle_and_degenerate_vertices_rejected():
    shape = convex()
    geometry = shape["geometry"]["value"]
    geometry["vertices_cm"].append([10, 10, 10])
    geometry["indices"] = available([0, 1, 4])
    _, result = candidate.convert_manifest(source_fixture([shape]))
    assert "supporting boundary" in result["components"][0]["rejected_primitives"][0]["reason"]
    geometry["vertices_cm"] = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]]
    _, result = candidate.convert_manifest(source_fixture([shape]))
    assert "non-volumetric" in result["components"][0]["rejected_primitives"][0]["reason"]


def test_convex_vertex_cloud_requires_existing_agggeom_and_explicit_index_review():
    shape = convex()
    shape["geometry"]["value"]["indices"] = {"status": "unavailable", "reason": "not reflected"}
    source = source_fixture([shape])
    _, result = candidate.convert_manifest(source)
    assert len(result["geometries"]) == 1
    assert "source_indices_unavailable_requires_review" in [i["code"] for i in result["components"][0]["issues"]]
    source["mesh_assets"][0]["collision"]["basis"] = "render_triangles"
    _, result = candidate.convert_manifest(source)
    assert not result["geometries"]
    assert "not existing" in result["components"][0]["issues"][-1]["detail"]


def test_missing_raw_geometry_does_not_create_visual_aabb_or_floor():
    shape = convex()
    shape["geometry"] = {"status": "unavailable", "reason": "vertex_data not exposed"}
    source = source_fixture([shape])
    xml, result = candidate.convert_manifest(source)
    assert len(ET.fromstring(xml).find("worldbody")) == 0
    assert "vertex_data not exposed" in result["components"][0]["rejected_primitives"][0]["reason"]
    assert result["collision_bounds"] is None
    assert not result["visual_aabb_used"]
    source["mesh_assets"][0]["collision"]["shapes"] = []
    _, result = candidate.convert_manifest(source)
    assert "empty_source_agggeom" in [i["code"] for i in result["components"][0]["issues"]]


def test_cross_boundary_floor_is_retained_whole_with_explicit_review():
    source = source_fixture([box(size=(4000, 4000, 20))])
    component = source["actors"][0]["components"][0]
    component["source_id"] = "/actor/FactoryFloor"
    component["disposition"] = "crossing_pending_review_no_clip_no_migration"
    _, result = candidate.convert_manifest(source)
    assert result["summary"]["retained_source_boundary_components"] == 1
    assert result["geometries"][0]["collision_crosses_selection"]
    assert result["collision_bounds"]["min_m"] == [-20, -20, -0.1]
    assert result["geometries"][0]["requires_review"]
    assert not result["qualified_collision"] and not result["published_world"]


@pytest.mark.parametrize(
    "kind,cls,reason",
    [
        ("skeletal_mesh", "SkeletalMeshComponent", "unsupported_skeletal_mesh"),
        ("spline_mesh", "SplineMeshComponent", "unsupported_spline"),
        ("other_scene_component", "SplineComponent", "unsupported_spline"),
        ("instanced_static_mesh", "InstancedStaticMeshComponent", "unsupported_instanced_static_mesh"),
    ],
)
def test_unsupported_components_are_reported_not_substituted(kind, cls, reason):
    source = source_fixture()
    component = source["actors"][0]["components"][0]
    component.update(kind=kind, **{"class": cls})
    _, result = candidate.convert_manifest(source)
    assert not result["geometries"]
    assert result["components"][0]["issues"][0]["code"] == reason


@pytest.mark.parametrize(
    "change,reason",
    [
        ("zero_scale", "singular"),
        ("bad_quaternion", "non-unit"),
        ("disabled", "not supported physical"),
        ("complex", "complex-as-simple"),
        ("unsupported_shape", "unsupported AggGeom"),
        ("no_transform", "transform unavailable"),
    ],
)
def test_unsupported_collision_is_explicit(change, reason):
    source = source_fixture([convex() if change == "no_transform" else box()])
    comp = source["actors"][0]["components"][0]
    collision = source["mesh_assets"][0]["collision"]
    if change == "zero_scale":
        comp["world_transform"]["scale3d"] = [1, 0, 1]
    elif change == "bad_quaternion":
        comp["world_transform"]["rotation_xyzw"] = [0, 0, 0, 2]
    elif change == "disabled":
        comp["collision_enabled"] = available("NO_COLLISION")
    elif change == "complex":
        collision["collision_trace_flag"] = available("CTF_USE_COMPLEX_AS_SIMPLE")
    elif change == "unsupported_shape":
        collision["shapes"][0]["kind"] = "sphere"
    else:
        collision["shapes"][0]["geometry"]["value"]["element_transform"] = {"status": "unavailable"}
    _, result = candidate.convert_manifest(source)
    assert not result["geometries"]
    assert reason in result["components"][0]["rejected_primitives"][0]["reason"]


def test_new_output_only_source_immutable_and_deterministic(tmp_path):
    source = tmp_path / "source.json"
    data = json.dumps(source_fixture())
    source.write_text(data, encoding="utf-8")
    output = tmp_path / "candidate"
    first = candidate.write_candidate(source, output)
    assert source.read_text(encoding="utf-8") == data
    assert json.loads((output / "manifest.json").read_text(encoding="utf-8")) == first
    assert {p.name for p in output.iterdir()} == {"manifest.json", "collision-candidate.xml"}
    before = {p.name: p.read_bytes() for p in output.iterdir()}
    with pytest.raises(FileExistsError):
        candidate.write_candidate(source, output)
    assert {p.name: p.read_bytes() for p in output.iterdir()} == before
    second = tmp_path / "another"
    assert candidate.write_candidate(source, second) == first
    assert (output / "collision-candidate.xml").read_bytes() == (second / "collision-candidate.xml").read_bytes()


@pytest.mark.parametrize(
    "field,value", [("frame", "right_handed"), ("units", "m"), ("source_inventory_matches", False)]
)
def test_wrong_source_contract_rejected_before_writing(tmp_path, field, value):
    source = source_fixture()
    source[field] = value
    path = tmp_path / "source.json"
    path.write_text(json.dumps(source), encoding="utf-8")
    with pytest.raises(ValueError):
        candidate.write_candidate(path, tmp_path / "output")
    assert not (tmp_path / "output").exists()


def ray_hull(vertices, origin, direction):
    """Independent ray vs convex supporting planes; returns first entry."""
    planes = ConvexHull(vertices).equations
    enter, leave = 0.0, np.inf
    for plane in planes:
        numerator = -(plane[:3] @ origin + plane[3])
        denominator = plane[:3] @ direction
        if abs(denominator) < 1e-12:
            if numerator < -1e-9:
                return np.inf
        elif denominator < 0:
            enter = max(enter, numerator / denominator)
        else:
            leave = min(leave, numerator / denominator)
    return enter if enter <= leave and leave >= 0 else np.inf


@pytest.mark.parametrize("shape", [box(pyr=(17, 38, -29)), convex()])
def test_native_mujoco_compile_and_rays_for_baked_source_geometry(shape):
    mujoco = pytest.importorskip("mujoco")
    xml, result = candidate.convert_manifest(source_fixture([shape], transform((1000, 2000, 3000), (-2, 3, 0.5))))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.ngeom == 1 and model.nq == 0 and model.nv == 0
    points = vertices(result["geometries"][0])
    center = points.mean(axis=0)
    for direction in np.eye(3):
        origin = center - direction * 10
        expected = ray_hull(points, origin, direction)
        geomid = np.array([-1], dtype=np.int32)
        actual = mujoco.mj_ray(model, data, origin, direction, None, 1, -1, geomid)
        assert actual == pytest.approx(expected, abs=2e-6)
        assert geomid[0] == 0


@pytest.fixture(scope="module")
def real_conversion():
    if not REAL_EXPORT.exists():
        pytest.skip("real UE export is not present; synthetic tests do not replace it")
    source = json.loads(REAL_EXPORT.read_text(encoding="utf-8"))
    snapshot = copy.deepcopy(source)
    xml, result = candidate.convert_manifest(source)
    assert source == snapshot
    return source, xml, result


def test_actual_export_coverage_and_rejection_accounting(real_conversion):
    source, _, result = real_conversion
    summary = result["summary"]
    assert summary["source_actors"] == 772
    assert summary["source_static_mesh_assets"] == 94
    assert summary["source_primitive_counts"] == {"box": 61, "convex": 264}
    source_cs = [c for a in source["actors"] for c in a["components"]]
    assert {r["source_id"] for r in result["components"]} == {c["source_id"] for c in source_cs}
    assets = {a["source_id"]: a for a in source["mesh_assets"]}
    # Count uses from source, independently of converter summary; all real convex
    # payloads are explicitly unavailable, not 264 usable vertex clouds.
    expected = Counter()
    for c in source_cs:
        if c["kind"] == "static_mesh" and c["disposition"] != "outside_bay":
            expected.update(s["kind"] for s in assets[c["mesh_asset"]]["collision"]["shapes"])
    assert summary["accepted_geometry_counts"] == {"box": expected["box"]}
    assert summary["rejected_primitive_instances"] == expected["convex"]
    assert all(g["source_kind"] == "box" for g in result["geometries"])
    assert sum(a["accepted_geometry_count"] for a in result["source_assets"]) == expected["box"]
    rows = {r["source_id"]: r for r in result["components"]}
    for c in source_cs:
        row = rows[c["source_id"]]
        if c["kind"] == "static_mesh" and c["disposition"] != "outside_bay":
            shapes = assets[c["mesh_asset"]]["collision"]["shapes"]
            assert row["accepted_geometry_count"] + row["rejected_primitive_count"] == len(shapes)
            assert all("vertex_data" in p["reason"] for p in row["rejected_primitives"])
        if c["kind"] == "skeletal_mesh":
            assert "unsupported_skeletal_mesh" in [i["code"] for i in row["issues"]]
    floors = [r for r in result["components"] if "floor" in r["source_id"].lower() and r["geometry_ids"]]
    assert floors and any(r["source_disposition"].startswith("crossing") for r in floors)
    assert not result["qualified_collision"] and not result["published_world"]


def test_actual_export_native_compile_and_source_floor_rays(real_conversion):
    mujoco = pytest.importorskip("mujoco")
    _, xml, result = real_conversion
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.ngeom == result["summary"]["accepted_geometries"]
    assert model.nq == 0 and model.nv == 0
    arrays = [vertices(g) for g in result["geometries"]]
    # MuJoCo recenters/rotates mesh assets internally. Reconstruct its compiled
    # world vertices to catch double transforms, reflections or mesh offsets.
    for geom_index, expected in enumerate(arrays):
        mesh_id = model.geom_dataid[geom_index]
        start, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
        native = model.mesh_vert[start : start + count] @ data.geom_xmat[geom_index].reshape(3, 3).T
        native += data.geom_xpos[geom_index]
        distances = np.linalg.norm(native[:, None, :] - expected[None, :, :], axis=2)
        assert distances.min(axis=0).max() < 2e-5
        assert distances.min(axis=1).max() < 2e-5
    floors = [g for g in result["geometries"] if "floor" in g["component_source_id"].lower()]
    assert len(floors) >= 3
    for record in floors[:3]:
        points = vertices(record)
        origin = points.mean(axis=0)
        origin[2] = result["collision_bounds"]["max_m"][2] + 2
        direction = np.array([0.0, 0.0, -1.0])
        expected = min(ray_hull(a, origin, direction) for a in arrays)
        assert np.isfinite(expected)
        geomid = np.array([-1], dtype=np.int32)
        actual = mujoco.mj_ray(model, data, origin, direction, None, 1, -1, geomid)
        assert actual == pytest.approx(expected, abs=2e-5)
        assert geomid[0] >= 0
    # A ray in the empty margin proves no visual AABB / infinite ground was added.
    origin = np.array(result["collision_bounds"]["max_m"]) + 10
    assert (
        mujoco.mj_ray(model, data, origin, np.array([0.0, 0.0, -1.0]), None, 1, -1, np.array([-1], dtype=np.int32))
        == -1
    )
