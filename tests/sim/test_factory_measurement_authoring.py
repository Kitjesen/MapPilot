"""Factory source geometry -> authored measurements -> compiled sensor plans."""

import copy
import json
import tempfile
from pathlib import Path

import numpy as np
import pytest
import yaml
from sim.catalog import CatalogResolver
from sim.tools.worlds.factory_environment_collection import measurement_authoring as author

REPO = Path(__file__).resolve().parents[2]
CANDIDATE = REPO / "build/inspection-tests/20261022/factory-collision-candidate-02"
pytestmark = [
    pytest.mark.sim,
    pytest.mark.skipif(
        not (CANDIDATE / "manifest.json").is_file(),
        reason="Local licensed Fab export absent; this skip is NOT real-source sampling evidence",
    ),
]


def real_candidate():
    return (
        (CANDIDATE / "collision-candidate.xml").read_text(encoding="utf-8"),
        json.loads((CANDIDATE / "manifest.json").read_text(encoding="utf-8")),
    )


def test_real_fab_devices_have_explicit_assumed_properties_and_finite_16k_clips():
    xml, candidate = real_candidate()
    output, report = author.author_measurements(xml, candidate)
    assert report["counts"]["devices"] == 2
    assert report["counts"]["bound_geoms"] == 6
    assert report["counts"]["total_geoms"] == 1065
    assert report["source_candidate"] == candidate
    assert report["qualified_world"] is False
    assert report["assumptions"]["basis"] == "author_assumptions_not_field_measurements"
    assert "<body" in output
    for device in report["recipe"]["devices"]:
        assert device["source_component_id"] in {author.CONTROL, author.ENGINE}
        assert device["thermal"]["temperature_k"] > 293.15
        assert 0 < device["thermal"]["emissivity"] <= 1
        assert device["semantic"]["instance_id"] > 0
        source = device["acoustic"]
        assert source["sample_rate_hz"] == 16000
        assert len(source["pressure_pa"]) == 3200
        assert np.isfinite(source["pressure_pa"]).all()
        assert np.max(np.abs(source["pressure_pa"])) > 0
        assert report["assumptions"]["devices"][device["device_id"]]["anomaly_temperature_k"] > 0


@pytest.mark.parametrize("case", ["missing_device", "empty_device", "missing_geom", "duplicate_geom", "xml_geom"])
def test_incomplete_or_duplicate_source_graph_fails_with_concrete_blocker(case):
    xml, candidate = real_candidate()
    candidate = copy.deepcopy(candidate)
    component = next(c for c in candidate["components"] if c["source_id"] == author.ENGINE)
    if case == "missing_device":
        candidate["components"].remove(component)
    elif case == "empty_device":
        component["geometry_ids"] = []
    elif case == "missing_geom":
        candidate["geometries"] = [g for g in candidate["geometries"] if g["component_source_id"] != author.ENGINE]
    elif case == "duplicate_geom":
        candidate["geometries"].append(copy.deepcopy(candidate["geometries"][0]))
    else:
        name = next(g["mjcf_geom"] for g in candidate["geometries"] if g["component_source_id"] == author.ENGINE)
        xml = xml.replace(f'name="{name}"', 'name="not_the_source_geom"')
    with pytest.raises(ValueError, match=r"component|geom"):
        author.author_measurements(xml, candidate)


@pytest.fixture
def local_output():
    with tempfile.TemporaryDirectory(prefix="factory-measurement-test-", dir=REPO / "build") as directory:
        yield Path(directory) / "new-candidate"


def test_versioned_candidate_compiles_nonempty_measurements_into_sensor_plan(local_output):
    session_path = author.write_measurement_candidate(REPO, CANDIDATE, local_output)
    resolver = CatalogResolver(REPO, [REPO / "sim/packages", session_path.parent / "packages"])
    resolved = resolver.resolve(session_path)
    world = resolver.find_package(f"{author.WORLD_ID}@{author.VERSION}")
    report = json.loads((session_path.parent / "authoring-report.json").read_text(encoding="utf-8"))
    assert report["qualified_world"] is False
    assert report["published_world"] is False
    assert report["importer_status"] == "candidate_imported_unqualified"
    assert report["draft"]["state"] == "candidate"
    assert report["draft"]["qualification_path"] is None
    assert Path(report["draft"]["provenance_path"]).is_file()
    assert resolved.sensor_plan["scene_measurements"] == world.data["measurements"]
    assert len(resolved.sensor_plan["scene_measurements"]["thermal_surfaces"]) == 6
    assert len(resolved.sensor_plan["scene_measurements"]["semantic_labels"]) == 6
    assert len(resolved.sensor_plan["scene_measurements"]["acoustic_sources"]) == 2
    assert {"thermal", "semantic", "acoustic"} <= resolved.sensor_plan["streams"].keys()
    assert yaml.safe_load(session_path.read_text(encoding="utf-8"))["world"] == world.ref
    resolved.write_bundle(session_path.parent / "bundle")
    assert json.loads((session_path.parent / "bundle/sensor.plan.json").read_text())["scene_measurements"]


def test_compiled_candidate_samples_real_fab_pixels_and_16k_pressure_without_motion(local_output):
    session_path = author.write_measurement_candidate(REPO, CANDIDATE, local_output)
    resolved = CatalogResolver(REPO, [REPO / "sim/packages", local_output / "packages"]).resolve(session_path)
    arrays, evidence = author.sample_candidate(REPO, resolved)
    assert evidence["ngeom"] == 1065
    assert evidence["nq"] == evidence["nv"] == evidence["initial_sim_time_ns"] == 0
    assert evidence["final_fixture_sim_time_ns"] == 1_000_000_000
    assert len(evidence["geom_bindings"]) == 1065
    assert evidence["fixture_only"] is True
    assert evidence["independent_scalar_rays"] == 3 * 64 * 48
    for device in ("control22", "engine"):
        valid = arrays[f"{device}_thermal_valid"]
        assert valid.any()
        assert np.isnan(arrays[f"{device}_temperature_k"][~valid]).all()
        np.testing.assert_array_equal(arrays[f"{device}_semantic_valid"], valid)
        class_id, instance_id = (101, 2101) if device == "control22" else (102, 2102)
        np.testing.assert_array_equal(arrays[f"{device}_class_ids"][valid], class_id)
        np.testing.assert_array_equal(arrays[f"{device}_instance_ids"][valid], instance_id)
        expected = (
            (0.85 * 330**4 + 0.15 * 293.15**4) ** 0.25
            if device == "control22"
            else (0.9 * 365**4 + 0.1 * 293.15**4) ** 0.25
        )
        np.testing.assert_allclose(arrays[f"{device}_temperature_k"][valid], expected, atol=2e-5)
    assert arrays["control22_acoustic_valid"].all()
    assert arrays["control22_pressure_pa"].shape == (1, 160)
    assert np.max(np.abs(arrays["control22_pressure_pa"])) > 0
    assert evidence["acoustic_sample_rate_hz"] == 16000
    assert arrays["mixed_acoustic_valid"].all()
    scene = resolved.sensor_plan["scene_measurements"]
    for name, time_s in (("control22", 0), ("mixed", 0.05)):
        metadata = evidence["captures"][f"{name}_acoustic"]["metadata"]
        positions = {s["source_id"]: np.array(s["position_world_m"]) for s in metadata["sources"]}
        receiver = np.array(metadata["channel_positions_world_m"][0])
        expected = np.zeros(160)
        for source in scene["acoustic_sources"]:
            distance = np.linalg.norm(positions[source["source_id"]] - receiver)
            indices = (time_s - distance / 343) * 16000 + np.arange(160)
            contribution = np.interp(indices, np.arange(3200), source["pressure_pa"], left=0, right=0) * min(
                1, 0.1 / distance
            )
            if name == "mixed":
                assert np.max(np.abs(contribution)) > 0, source["source_id"]
            expected += contribution
        np.testing.assert_allclose(arrays[f"{name}_pressure_pa"][0], expected, atol=1e-8, rtol=0)
    assert not arrays["wall_thermal_valid"].any()
    assert not arrays["wall_semantic_valid"].any()
    assert not arrays["wall_acoustic_valid"].any()
    assert np.isnan(arrays["wall_pressure_pa"]).all()
    assert arrays["expired_acoustic_valid"].all()
    np.testing.assert_array_equal(arrays["expired_pressure_pa"], 0)


def test_existing_output_is_rejected_without_overwriting(local_output):
    local_output.mkdir()
    with pytest.raises(FileExistsError, match="must not exist"):
        author.write_measurement_candidate(REPO, CANDIDATE, local_output)
    assert list(local_output.iterdir()) == []
