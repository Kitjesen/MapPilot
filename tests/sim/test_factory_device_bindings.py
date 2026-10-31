"""Local-only conversion, native MuJoCo equivalence and physical measurements."""

from __future__ import annotations

import copy
import json
from contextlib import nullcontext
from fractions import Fraction
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np
import pytest
from sim.runtime.sensors.contracts import DeadlinePolicy, ScheduledSensorSample, SensorRoute, SensorStreamPlan
from sim.runtime.sensors.measurement_configuration import make_measurement_extractor
from sim.tools.worlds.factory_environment_collection import bay_device_bindings as binding

REPO = Path(__file__).resolve().parents[2]
REAL_CANDIDATE = REPO / "build/inspection-tests/20261022/factory-collision-candidate-02"


def fixture():
    xml = """<mujoco><compiler angle="radian"/>
      <asset><mesh name="tetra" vertex="0 0 0 .2 0 0 0 .3 0 0 0 .4"/></asset>
      <worldbody>
        <geom name="target" type="sphere" pos="2 0 0" size=".2" contype="2" conaffinity="4"/>
        <geom name="unknown" type="box" pos="6 0 0" size=".2 .2 .2"/>
        <geom name="part" type="mesh" mesh="tetra" pos="2 1 0" quat=".7071067811865476 0 0 .7071067811865476"/>
      </worldbody></mujoco>"""
    candidate = {
        "schema": "lingtu.factory-collision-candidate.v1",
        "frame": binding.FRAME,
        "qualified_collision": False,
        "unknown_fields": {"trace": None, "temperature": "not_in_asset"},
        "limitations": ["boundary/skeletal/spline/default_trace unresolved"],
        "components": [
            {
                "source_id": "component/device",
                "source_world_transform": {"translation_cm": [200, -100, 50]},
                "geometry_ids": ["geometry/target", "geometry/part"],
                "issues": [{"code": "project_default_trace_requires_review"}],
            },
            {"source_id": "component/unknown", "geometry_ids": ["geometry/unknown"]},
            {"source_id": "component/empty", "geometry_ids": []},
        ],
        "geometries": [
            {"source_id": "geometry/target", "component_source_id": "component/device", "mjcf_geom": "target"},
            {"source_id": "geometry/unknown", "component_source_id": "component/unknown", "mjcf_geom": "unknown"},
            {"source_id": "geometry/part", "component_source_id": "component/device", "mjcf_geom": "part"},
        ],
    }
    recipe = {
        "schema": binding.RECIPE_SCHEMA,
        "frame": binding.FRAME,
        "simulation": True,
        "provenance": {"declarations": "SYNTHETIC SIMULATION, not factory measurements"},
        "devices": [
            {
                "device_id": "test_machine",
                "source_component_id": "component/device",
                "measurement_origin_m": [2, 0, 0],
                "thermal": {"temperature_k": 365.0, "emissivity": 0.9},
                "semantic": {"class_id": 101, "instance_id": 1001},
                "acoustic": {
                    "pressure_pa": [1.0, 0.0, 0.0],
                    "sample_rate_hz": 1000,
                    "start_sim_time_ns": 0,
                    "reference_distance_m": 1.0,
                },
            }
        ],
    }
    return xml, candidate, recipe


def compile_model(xml):
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def equivalence(before_xml, after_xml):
    before, before_data = compile_model(before_xml)
    after, after_data = compile_model(after_xml)
    assert before.ngeom == after.ngeom
    assert before.nmesh == after.nmesh
    assert before.nq == after.nq == before.nv == after.nv == before.njnt == after.njnt == 0
    errors = []
    vertex_errors = []
    for old in range(before.ngeom):
        name = mujoco.mj_id2name(before, mujoco.mjtObj.mjOBJ_GEOM, old)
        new = mujoco.mj_name2id(after, mujoco.mjtObj.mjOBJ_GEOM, name)
        assert new >= 0
        errors.append(float(np.max(np.abs(before_data.geom_xpos[old] - after_data.geom_xpos[new]))))
        np.testing.assert_allclose(before_data.geom_xpos[old], after_data.geom_xpos[new], rtol=0, atol=5e-12)
        np.testing.assert_allclose(before_data.geom_xmat[old], after_data.geom_xmat[new], rtol=0, atol=5e-12)
        for field in (
            "geom_type",
            "geom_contype",
            "geom_conaffinity",
            "geom_group",
            "geom_size",
            "geom_friction",
            "geom_solref",
            "geom_solimp",
            "geom_margin",
            "geom_gap",
            "geom_rgba",
        ):
            np.testing.assert_array_equal(getattr(before, field)[old], getattr(after, field)[new])
        if before.geom_type[old] == mujoco.mjtGeom.mjGEOM_MESH:
            points = []
            for model, data, index in ((before, before_data, old), (after, after_data, new)):
                mesh = model.geom_dataid[index]
                start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                vertices = model.mesh_vert[start : start + count]
                points.append(vertices @ data.geom_xmat[index].reshape(3, 3).T + data.geom_xpos[index])
            np.testing.assert_allclose(*points, rtol=0, atol=5e-12)
            vertex_errors.append(float(np.max(np.abs(points[0] - points[1]))))
    np.testing.assert_array_equal(before.opt.gravity, after.opt.gravity)
    for field in ("timestep", "integrator", "solver", "iterations"):
        assert getattr(before.opt, field) == getattr(after.opt, field)
    return {
        "geoms": after.ngeom,
        "meshes": after.nmesh,
        "nq": after.nq,
        "nv": after.nv,
        "max_geom_xpos_error_m": max(errors),
        "max_mesh_vertex_error_m": max(vertex_errors, default=0),
    }


@pytest.mark.parametrize("explicit_origin", [None, [1.25, -0.7, 2.0]])
def test_translation_only_grouping_preserves_geometry_defaults_unknowns_and_inputs(explicit_origin):
    xml, candidate, recipe = fixture()
    if explicit_origin is not None:
        recipe["devices"][0]["body_origin_m"] = explicit_origin
    original = copy.deepcopy((candidate, recipe))
    output, report = binding.bind_devices(xml, candidate, recipe)
    assert (candidate, recipe) == original
    assert report["source_candidate"] == candidate
    assert report["source_candidate"] is not candidate
    assert report["recipe"] == recipe
    assert report["full_world_blocked"] and not report["qualified_world"] and not report["published_world"]
    assert report["counts"] == {"devices": 1, "bound_geoms": 2, "total_geoms": 3, "added_geoms": 0, "added_joints": 0}
    assert report["unbound_geometry_ids"] == ["geometry/unknown"]
    assert "unknown" not in report["scene_measurements"]["thermal_surfaces"]
    bound = report["bindings"][0]
    assert bound["body_pose"]["position_m"] == (explicit_origin or [2.0, 1.0, 0.5])
    np.testing.assert_allclose(
        np.array(bound["body_pose"]["position_m"]) + bound["measurement_local_offset_m"], [2, 0, 0]
    )
    old_root, new_root = ET.fromstring(xml), ET.fromstring(output)
    assert old_root.find("compiler").attrib == new_root.find("compiler").attrib
    for tag in ("asset", "option", "default"):
        old_node, new_node = old_root.find(tag), new_root.find(tag)
        if old_node is None:
            assert new_node is None
        else:
            assert [n.attrib for n in old_node.iter()] == [n.attrib for n in new_node.iter()]
    assert not new_root.findall(".//joint")
    assert new_root.find("worldbody/geom").get("name") == "unknown"
    assert {g.get("name") for g in new_root.findall("worldbody/body/geom")} == {"target", "part"}
    evidence = equivalence(xml, output)
    assert evidence["geoms"] == 3


@pytest.mark.parametrize(
    "case",
    [
        "missing_component",
        "duplicate_component",
        "duplicate_device",
        "overlapping_device",
        "empty_component",
        "missing_geom",
        "duplicate_geom_id",
        "duplicate_geom_name",
        "duplicate_xml_geom",
        "missing_xml_geom",
        "duplicate_component_geom",
        "overlapping_component",
        "unowned_geom",
        "wrong_owner",
        "nonfinite_origin",
        "string_origin",
        "nonfinite_temperature",
        "bad_emissivity",
        "bad_semantic",
        "empty_waveform",
        "nonfinite_waveform",
        "string_waveform",
        "bad_sample_rate",
        "bad_reference_distance",
        "undeclared_field",
        "not_simulation",
        "pre_grouped",
        "fused_bodies",
        "fromto",
    ],
)
def test_rejects_invalid_or_ambiguous_bindings_without_mutating_input(case):
    xml, candidate, recipe = fixture()
    device = recipe["devices"][0]
    if case == "missing_component":
        device["source_component_id"] = "absent"
    elif case == "duplicate_component":
        candidate["components"].append(copy.deepcopy(candidate["components"][0]))
    elif case in {"duplicate_device", "overlapping_device"}:
        second = copy.deepcopy(device)
        if case == "overlapping_device":
            second["device_id"] = "other"
        recipe["devices"].append(second)
    elif case == "empty_component":
        device["source_component_id"] = "component/empty"
    elif case == "missing_geom":
        candidate["components"][0]["geometry_ids"].append("absent")
    elif case == "duplicate_geom_id":
        candidate["geometries"].append(copy.deepcopy(candidate["geometries"][0]))
    elif case == "duplicate_geom_name":
        candidate["geometries"][1]["mjcf_geom"] = "target"
    elif case == "duplicate_xml_geom":
        xml = xml.replace("</worldbody>", '<geom name="target" size="1"/></worldbody>')
    elif case == "missing_xml_geom":
        xml = xml.replace('name="target"', 'name="absent"')
    elif case == "duplicate_component_geom":
        candidate["components"][0]["geometry_ids"].append("geometry/target")
    elif case == "overlapping_component":
        candidate["components"][1]["geometry_ids"].append("geometry/target")
    elif case == "unowned_geom":
        candidate["components"][0]["geometry_ids"].remove("geometry/part")
    elif case == "wrong_owner":
        candidate["geometries"][0]["component_source_id"] = "component/unknown"
    elif case == "nonfinite_origin":
        device["measurement_origin_m"] = [float("inf"), 0, 0]
    elif case == "string_origin":
        device["body_origin_m"] = ["1", "2", "3"]
    elif case == "nonfinite_temperature":
        device["thermal"]["temperature_k"] = float("inf")
    elif case == "bad_emissivity":
        device["thermal"]["emissivity"] = 1.1
    elif case == "bad_semantic":
        device["semantic"]["instance_id"] = 0
    elif case == "empty_waveform":
        device["acoustic"]["pressure_pa"] = []
    elif case == "nonfinite_waveform":
        device["acoustic"]["pressure_pa"] = [float("inf")]
    elif case == "string_waveform":
        device["acoustic"]["pressure_pa"] = "sine"
    elif case == "bad_sample_rate":
        device["acoustic"]["sample_rate_hz"] = True
    elif case == "bad_reference_distance":
        device["acoustic"]["reference_distance_m"] = 0
    elif case == "undeclared_field":
        device["acoustic"]["loop"] = True
    elif case == "not_simulation":
        recipe["simulation"] = False
    elif case == "pre_grouped":
        xml = xml.replace("</worldbody>", '<body name="existing"/></worldbody>')
    elif case == "fused_bodies":
        xml = xml.replace('angle="radian"', 'angle="radian" fusestatic="true"')
    elif case == "fromto":
        xml = xml.replace('name="target"', 'name="target" fromto="0 0 0 1 1 1"')
    original = copy.deepcopy((candidate, recipe))
    with pytest.raises(ValueError):
        binding.bind_devices(xml, candidate, recipe)
    assert (candidate, recipe) == original


def stream(kind):
    return SensorStreamPlan(
        kind,
        "robot",
        f"robot.{kind}",
        f"robot/{kind}",
        "measurement",
        Fraction(1000, 6) if kind == "acoustic" else Fraction(30),
        SensorRoute("physics", "mujoco_measurement", "in_process"),
    )


def sample(
    model, data, scene, kind, *, camera_position=(0, 0, 0), camera_quat=None, microphone_position=(0, 0, 0), time_ns=0
):
    config = {"body_frame_id": "robot/receiver"}
    if kind == "acoustic":
        config["microphone"] = {
            "sample_rate_hz": 1000,
            "block_samples": 6,
            "channel_offsets_m": [list(microphone_position)],
            "speed_of_sound_mps": 1000,
        }
    else:
        config["camera"] = {
            "model": "pinhole",
            "width": 1,
            "height": 1,
            "fx_px": 1,
            "fy_px": 1,
            "cx_px": 0.5,
            "cy_px": 0.5,
        }
        config["mount_pose"] = {
            "position_m": list(camera_position),
            "orientation_wxyz": camera_quat or [np.sqrt(0.5), 0, np.sqrt(0.5), 0],
        }
        if kind == "thermal":
            config["thermal"] = {
                "background_temperature_k": 293.15,
                "calibration_id": "synthetic-test-lens",
                "temperature_mode": "surface_corrected",
            }
    extractor = make_measurement_extractor(stream(kind), config, scene, lambda: nullcontext((model, data)))
    request = ScheduledSensorSample("binding-test", stream(kind), 1, 0, 0, time_ns, DeadlinePolicy.DROP_INTERMEDIATE)
    data.time = time_ns / 1e9
    return extractor(
        request, {"session_id": "binding-test", "model_generation": 1, "reset_generation": 0, "sim_time_ns": time_ns}
    )


def receiver_xml(xml):
    # A measurement fixture only: conversion itself never adds a receiver/geom.
    return xml.replace("</worldbody>", '<body name="receiver"/></worldbody>')


def test_factory_thermal_semantic_rays_and_acoustic_arrival_use_real_model_body():
    xml, candidate, recipe = fixture()
    recipe["devices"][0]["body_origin_m"] = [1.25, -0.7, 2.0]
    output, report = binding.bind_devices(xml, candidate, recipe)
    model, data = compile_model(receiver_xml(output))
    scene = report["scene_measurements"]
    thermal = sample(model, data, scene, "thermal")
    assert thermal.valid_mask.all()
    np.testing.assert_allclose(thermal.temperature_k, [[365]])
    semantic = sample(model, data, scene, "semantic")
    np.testing.assert_array_equal(semantic.class_ids, [[101]])
    np.testing.assert_array_equal(semantic.instance_ids, [[1001]])
    acoustic = sample(model, data, scene, "acoustic")
    assert acoustic.valid_mask.all()
    np.testing.assert_allclose(acoustic.pressure_pa, [[0, 0, 0.5, 0, 0, 0]], atol=1e-7)
    assert not acoustic.metadata["scene"]["occlusion_hits"]
    np.testing.assert_allclose(acoustic.metadata["sources"][0]["position_world_m"], [2, 0, 0])
    assert acoustic.metadata["scene"]["source_mounts"][0]["body_id"] > 0
    late = sample(model, data, scene, "acoustic", time_ns=100_000_000)
    np.testing.assert_array_equal(late.pressure_pa, 0)
    assert data.time == 0.1 and model.nq == model.nv == 0
    # Unknown physical surfaces are not assigned synthetic labels/temperatures.
    missing = sample(model, data, scene, "thermal", camera_position=(4, 0, 0))
    assert not missing.valid_mask.any() and np.isnan(missing.temperature_k).all()
    unlabeled = sample(model, data, scene, "semantic", camera_position=(4, 0, 0))
    np.testing.assert_array_equal(unlabeled.class_ids, 0)


def test_world_body_before_binding_is_not_a_valid_acoustic_source():
    xml, candidate, recipe = fixture()
    _, report = binding.bind_devices(xml, candidate, recipe)
    scene = copy.deepcopy(report["scene_measurements"])
    scene["acoustic_sources"][0]["body_name"] = "world"
    model, data = compile_model(receiver_xml(xml))
    with pytest.raises(ValueError, match="non-world"):
        sample(model, data, scene, "acoustic")


def test_unbound_geom_still_occludes_source_after_grouping():
    xml, candidate, recipe = fixture()
    xml = xml.replace('name="unknown" type="box" pos="6 0 0"', 'name="unknown" type="box" pos="1 0 0"')
    output, report = binding.bind_devices(xml, candidate, recipe)
    model, data = compile_model(receiver_xml(output))
    frame = sample(model, data, report["scene_measurements"], "acoustic")
    assert not frame.valid_mask.any()
    assert frame.metadata["scene"]["occlusion_hits"][0]["geom_name"] == "unknown"


def test_real_candidate_all_1065_geoms_vertices_and_source_provenance_survive():
    if not (REAL_CANDIDATE / "manifest.json").is_file():
        pytest.skip("real local export/candidate not available; synthetic tests do not replace this evidence")
    candidate = json.loads((REAL_CANDIDATE / "manifest.json").read_text(encoding="utf-8"))
    xml = (REAL_CANDIDATE / "collision-candidate.xml").read_text(encoding="utf-8")
    recipe = binding.synthetic_factory_recipe()
    output, report = binding.bind_devices(xml, candidate, recipe)
    assert report["counts"] == {
        "devices": 2,
        "bound_geoms": 9,
        "total_geoms": 1065,
        "added_geoms": 0,
        "added_joints": 0,
    }
    assert len(report["unbound_geometry_ids"]) == 1056
    assert report["source_candidate"] == candidate
    evidence = equivalence(xml, output)
    assert evidence["geoms"] == evidence["meshes"] == 1065
    source = json.loads((REPO / recipe["provenance"]["source_export"]).read_text(encoding="utf-8"))
    components = {c["source_id"]: c for a in source["actors"] for c in a["components"]}
    for device, bound in zip(recipe["devices"], report["bindings"]):
        source_translation = components[device["source_component_id"]]["world_transform"]["translation_cm"]
        np.testing.assert_allclose(bound["body_pose"]["position_m"], np.array(source_translation) * [0.01, -0.01, 0.01])
    model, data = compile_model(receiver_xml(output))
    # Aim at an actual retained convex face, not at an AABB or an invented proxy.
    mesh = next(
        g for g in candidate["geometries"] if g["mjcf_geom"] == report["bindings"][1]["geometries"][0]["mjcf_geom"]
    )
    target = np.array(mesh["mesh_origin_m"])
    thermal = sample(
        model,
        data,
        report["scene_measurements"],
        "thermal",
        camera_position=target + np.array([0, 0, 0.5]),
        camera_quat=[0, 1, 0, 0],
    )
    assert thermal.valid_mask.all()
    np.testing.assert_allclose(thermal.temperature_k, 365)
    engine_scene = copy.deepcopy(report["scene_measurements"])
    engine_scene["acoustic_sources"] = [engine_scene["acoustic_sources"][1]]
    source_point = np.array(report["bindings"][1]["measurement_pose"]["position_m"])
    acoustic = sample(model, data, engine_scene, "acoustic", microphone_position=source_point + np.array([0, 0, 2]))
    assert acoustic.valid_mask.all()
    np.testing.assert_allclose(acoustic.pressure_pa, [[0, 0, 0.05, 0, -0.05, 0]], atol=1e-7)
    assert not acoustic.metadata["scene"]["occlusion_hits"]
