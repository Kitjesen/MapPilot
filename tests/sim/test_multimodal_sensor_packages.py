"""Real opt-in packages; fixture World proves contracts, not Fab readiness."""

from __future__ import annotations

import copy
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from sim.catalog import CatalogError, CatalogResolver, SimCatalog
from sim.catalog.measurement import validate_configuration

REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGES = REPO_ROOT / "sim/packages"
SESSION = REPO_ROOT / "sim/sessions/examples/thunderv4_controlled_headless/session.yaml"
UE_SESSION = REPO_ROOT / "sim/sessions/examples/thunderv4_unreal/session.yaml"
RIG = "thunderv4_headless_multimodal@1.0.0"
BASE_RIG = "thunderv4_headless_navigation@1.0.0"
LEGACY = {"rgb", "depth", "imu", "mid360", "truth_odom"}
SENSORS = {
    "thermal": ("thermal_camera@1.0.0", "camera/thermal", "ThermalImage", 10, "K"),
    "semantic": ("semantic_camera@1.0.0", "camera/semantic", "SemanticImage", 10, "label"),
    "acoustic": ("microphone@1.0.0", "microphone", "AcousticBlock", 100, "Pa"),
}
pytestmark = pytest.mark.sim


@pytest.fixture(scope="module")
def repository():
    return CatalogResolver.from_repository(REPO_ROOT)


def _validator(document, schema_name):
    jsonschema = pytest.importorskip("jsonschema")
    referencing = pytest.importorskip("referencing")
    schemas = [json.loads(path.read_text(encoding="utf-8"))
               for path in (REPO_ROOT / "sim/contracts/schemas").glob("*.json")]
    registry = referencing.Registry().with_resources(
        (schema["$id"], referencing.Resource.from_contents(schema)) for schema in schemas
    )
    schema = next(schema for schema in schemas if schema["$id"] == document["schema"])
    assert schema["$id"] == f"lingtu.sim.{schema_name}.v1"
    jsonschema.Draft202012Validator.check_schema(schema)
    return jsonschema.Draft202012Validator(schema, registry=registry)


def _validate(document, schema_name):
    _validator(document, schema_name).validate(document)


@pytest.fixture(scope="module")
def isolated_repository(tmp_path_factory, repository):
    """Copy real packages unchanged so every asset stays under Catalog's root."""
    root = tmp_path_factory.mktemp("multimodal_repository")
    references = ["thunderv4@1.0.3", "thunderv4_cts_moe@1.0.0", BASE_RIG, RIG]
    references.extend(sensor["package"] for sensor in repository.find_package(RIG).data["sensors"])
    for reference in dict.fromkeys(references):
        source = repository.find_package(reference).manifest_path.parent
        shutil.copytree(source, root / source.relative_to(REPO_ROOT))
    return root


@pytest.fixture
def fixture_world(isolated_repository):
    """Add only a temporary World; use the real robot, rig and sensor packages."""
    tmp_path = isolated_repository / "sim/packages/worlds/multimodal_contract_fixture"
    tmp_path.mkdir(parents=True, exist_ok=True)
    world = {
        "schema": "lingtu.sim.world-package.v1", "id": "multimodal_contract_fixture",
        "version": "1.0.0", "kind": "world",
        "physics": {"mjcf": "world.xml", "global_policy": {
            "timestep_s": 0.001, "integrator": "rk4", "solver": "newton", "iterations": 100,
            "gravity_mps2": [0, 0, -9.81],
        }},
        "visual": {"binding": "WorldVisual:MultimodalFixture", "level": "/Game/Fixture"},
        "entities": [],
        "measurements": {
            "thermal_surfaces": {"fixture_target": {"temperature_k": 310.0, "emissivity": 0.9}},
            "semantic_labels": {"fixture_target": {"class_id": 2, "instance_id": 7}},
            "acoustic_sources": [{
                "source_id": "fixture_sound", "body_name": "fixture_emitter",
                "pressure_pa": [0.0, 0.1, -0.1, 0.0], "sample_rate_hz": 16000,
                "start_sim_time_ns": 0, "reference_distance_m": 1.0,
            }],
        },
    }
    (tmp_path / "world.xml").write_text(
        '<mujoco><option timestep="0.001" integrator="RK4" solver="Newton" iterations="100" '
        'gravity="0 0 -9.81"/><worldbody><body name="fixture_emitter" pos="2 0 0">'
        '<geom name="fixture_target" type="sphere" size="0.1"/></body></worldbody></mujoco>',
        encoding="utf-8",
    )
    (tmp_path / "world.package.yaml").write_text(yaml.safe_dump(world), encoding="utf-8")
    session = yaml.safe_load(SESSION.read_text(encoding="utf-8"))
    session["world"] = "multimodal_contract_fixture@1.0.0"
    session["robots"][0]["sensor_rig"] = RIG
    path = tmp_path / "session.yaml"
    path.write_text(yaml.safe_dump(session), encoding="utf-8")
    return CatalogResolver.from_repository(isolated_repository), path, world


@pytest.mark.parametrize("kind", SENSORS)
def test_catalog_discovers_real_sensor_packages_and_schemas(repository, kind):
    reference, directory, message, rate, unit = SENSORS[kind]
    record = repository.find_package(reference, kind="sensor")
    assert record.manifest_path == PACKAGES / "sensors" / directory / "sensor.package.yaml"
    _validate(record.data, "sensor-package")
    assert record.data["outputs"] == [kind]
    assert record.data["timing"] == {"rate_hz": rate}
    assert record.data["interface"] == {"transport": "typed_dds", "message_type": f"lingtu.dds.{message}"}
    config = record.data["configuration"]["measurement"]
    assert validate_configuration(config, kind, rate) == config
    assert config["body_frame_id"] == "base_link"
    assert config["noise"] == {"unit": unit, "enabled": False, "stddev": 0.0, "seed": 0}
    assert "experimental" in record.data["description"].lower()


@pytest.mark.parametrize("reference", [*(spec[0] for spec in SENSORS.values()), RIG])
def test_catalog_api_reports_discovery_without_qualification(repository, reference):
    catalog = SimCatalog(repository)
    assert reference in {item["package"]["ref"] for item in catalog.list_packages()["packages"]}
    result = catalog.validate_package(reference)
    assert result["catalog_valid"] and result["dependencies_valid"]
    assert result["qualified"] is False


def test_rig_preserves_all_five_navigation_declarations(repository):
    baseline = repository.find_package(BASE_RIG).data["sensors"]
    record = repository.find_package(RIG, kind="sensor_rig")
    assert record.manifest_path == PACKAGES / "sensor_rigs/doso/thunder_v4/headless_multimodal/sensor-rig.package.yaml"
    appended = copy.deepcopy(record.data)
    appended["sensors"] = appended["sensors"][5:]
    _validate(appended, "sensor-rig-package")
    assert len(baseline) == 5
    assert len(record.data["sensors"]) == 8
    assert record.data["sensors"][:5] == baseline
    assert [item["package"] for item in record.data["sensors"][5:]] == [spec[0] for spec in SENSORS.values()]


def test_complete_rig_passes_strict_schema(repository):
    rig = repository.find_package(RIG).data
    baseline = repository.find_package(BASE_RIG).data
    _validate(rig, "sensor-rig-package")
    _validate(baseline, "sensor-rig-package")


@pytest.mark.parametrize("value", [True, False, 0, 1, "false", None])
def test_mid360_rig_overlay_schema_requires_boolean(repository, value):
    jsonschema = pytest.importorskip("jsonschema")
    rig = copy.deepcopy(repository.find_package(RIG).data)
    next(sensor for sensor in rig["sensors"] if sensor["id"] == "mid360")["configuration"][
        "navigation_fixture_raw_overlay"
    ] = value
    if type(value) is bool:
        _validate(rig, "sensor-rig-package")
    else:
        with pytest.raises(jsonschema.ValidationError):
            _validate(rig, "sensor-rig-package")


@pytest.mark.parametrize("sensor_id", ["imu", "front_rgb", "front_depth", "truth_odom", "microphone"])
def test_overlay_is_not_accepted_for_non_mid360_rig_sensors(repository, sensor_id):
    jsonschema = pytest.importorskip("jsonschema")
    rig = copy.deepcopy(repository.find_package(RIG).data)
    rig["sensors"] = [sensor for sensor in rig["sensors"] if sensor["id"] == sensor_id]
    rig["sensors"][0].setdefault("configuration", {})["navigation_fixture_raw_overlay"] = True
    with pytest.raises(jsonschema.ValidationError):
        _validate(rig, "sensor-rig-package")


def test_full_rig_resolves_with_only_fixture_world_properties(fixture_world):
    resolver, path, world = fixture_world
    resolved = resolver.resolve(path)
    _validate(world, "world-package")
    _validate(resolved.transport_intent, "transport-intent")
    streams = resolved.sensor_plan["streams"]
    assert set(streams) == LEGACY | SENSORS.keys()
    assert sum(map(len, streams.values())) == 8
    assert resolved.sensor_plan["scene_measurements"] == world["measurements"]
    baseline = yaml.safe_load(path.read_text(encoding="utf-8"))
    baseline["robots"][0]["sensor_rig"] = BASE_RIG
    baseline_path = path.with_name("navigation.yaml")
    baseline_path.write_text(yaml.safe_dump(baseline), encoding="utf-8")
    old = resolver.resolve(baseline_path)
    _validate(resolved.sensor_plan, "sensor-plan")
    _validate(old.sensor_plan, "sensor-plan")
    for kind in LEGACY:
        assert streams[kind] == old.sensor_plan["streams"][kind]
    old_channels = old.transport_intent["channels"]
    assert [channel for channel in resolved.transport_intent["channels"]
            if channel["payload_role"] not in SENSORS] == old_channels
    for kind, (_, _, message, rate, _) in SENSORS.items():
        stream, = streams[kind]
        assert stream["rate_hz"] == rate
        assert stream["measurement"]["body_frame_id"] == "thunder_01/base_link"
        assert (stream["owner"], stream["source"], stream["transport"]) == (
            "physics", "mujoco_measurement", "typed_dds",
        )
        channel, = [c for c in resolved.transport_intent["channels"] if c["payload_role"] == kind]
        assert channel["channel_id"] == stream["sensor_id"]
        assert channel["message_type"] == stream["message_type"] == f"lingtu.dds.{message}"
        assert (channel["owner"], channel["source"], channel["transport"]) == (
            "physics", "mujoco_measurement", "typed_dds",
        )


def test_complete_sensor_plan_strict_schema(fixture_world):
    resolver, path, _ = fixture_world
    plan = resolver.resolve(path).sensor_plan
    baseline = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).sensor_plan
    _validate(plan, "sensor-plan")
    _validate(baseline, "sensor-plan")


@pytest.mark.parametrize("session_path", [SESSION, UE_SESSION], ids=["mujoco", "unreal"])
def test_depth_schema_accepts_actual_catalog_source_contracts(repository, session_path):
    resolved = repository.resolve(session_path)
    before = resolved.sensor_json.encode("utf-8")
    depth, = resolved.sensor_plan["streams"]["depth"]
    assert depth["encoding"] == "16UC1"
    if depth["source"] == "unreal_camera":
        assert depth["requested_encoding"] == "32FC1"
        assert depth["depth_scale"] == 0.001
    else:
        assert depth["source"] == "mujoco_camera"
        assert "requested_encoding" not in depth and "depth_scale" not in depth
    _validate(resolved.sensor_plan, "sensor-plan")
    assert repository.resolve(session_path).sensor_json.encode("utf-8") == before


@pytest.mark.parametrize("field", ["requested_encoding", "depth_scale"])
def test_unreal_depth_schema_still_requires_wire_metadata(repository, field):
    jsonschema = pytest.importorskip("jsonschema")
    document = repository.resolve(UE_SESSION).sensor_plan
    del document["streams"]["depth"][0][field]
    with pytest.raises(jsonschema.ValidationError):
        _validate(document, "sensor-plan")


@pytest.mark.parametrize("session_path", [SESSION, UE_SESSION], ids=["mujoco", "unreal"])
@pytest.mark.parametrize(("field", "value"), [
    ("encoding", "32FC1"), ("requested_encoding", "rgb8"),
    ("depth_scale", 0.01), ("depth_scale", "0.001"),
])
def test_depth_schema_rejects_inaccurate_wire_metadata(repository, session_path, field, value):
    jsonschema = pytest.importorskip("jsonschema")
    document = repository.resolve(session_path).sensor_plan
    document["streams"]["depth"][0][field] = value
    with pytest.raises(jsonschema.ValidationError):
        _validate(document, "sensor-plan")


@pytest.mark.parametrize("kind", SENSORS)
def test_real_robot_mounts_have_correct_body_ancestry_and_optical_axes(fixture_world, kind):
    from sim.runtime.sensors.camera_geometry import CameraPose, GimbalPose, compose_camera_pose

    resolver, path, _ = fixture_world
    stream, = resolver.resolve(path).sensor_plan["streams"][kind]
    robot = resolver.find_package("thunderv4@1.0.3")
    frames = {frame["name"]: frame for frame in robot.data["frames"]}
    frame = "payload_top" if kind == "acoustic" else "front_camera"
    assert frames[frame]["parent_frame"] == "base_link"
    mjcf = ET.parse(robot.manifest_path.parent / robot.data["physics"]["mjcf"]).getroot()
    assert mjcf.find(".//body[@name='base_link']") is not None
    assert mjcf.find(f".//*[@name='{frame}']") is not None
    assert stream["frame_id"] == f"thunder_01/{frame}"
    mount = stream["measurement"]["mount_pose"]
    assert mount["position_m"] == pytest.approx(frames[frame]["extrinsic"]["position_m"])
    pose = CameraPose(**mount)
    expected_axes = ((1, 0, 0), (0, 1, 0), (0, 0, 1)) if kind == "acoustic" else (
        (0, -1, 0), (0, 0, -1), (1, 0, 0),
    )
    for axis, expected in zip(((1, 0, 0), (0, 1, 0), (0, 0, 1)), expected_axes):
        shifted = compose_camera_pose(pose, CameraPose(position_m=axis), GimbalPose())
        assert [a - b for a, b in zip(shifted.position_m, pose.position_m)] == pytest.approx(expected)


@pytest.mark.parametrize("kind", SENSORS)
def test_resolved_packages_construct_actual_factory_parameters_without_physics(fixture_world, kind):
    from sim.runtime.sensors import SensorRuntime
    from sim.runtime.sensors.measurement_configuration import make_measurement_extractor
    from sim.runtime.sensors.measurement_extractors import (
        AcousticExtractor,
        SemanticCameraExtractor,
        ThermalCameraExtractor,
    )

    resolver, path, _ = fixture_world
    resolved = resolver.resolve(path)
    runtime = SensorRuntime.from_plan(resolved.sensor_plan)
    assert len(runtime.streams) == 8
    stream, = [stream for stream in runtime.streams if stream.stream_kind == kind]
    declaration, = resolved.sensor_plan["streams"][kind]

    def no_physics():
        pytest.fail("Contract-only factory construction must not acquire physics")

    extractor = make_measurement_extractor(
        stream, declaration["measurement"], resolved.sensor_plan["scene_measurements"], no_physics,
    )
    expected_type = {"thermal": ThermalCameraExtractor, "semantic": SemanticCameraExtractor, "acoustic": AcousticExtractor}
    assert isinstance(extractor, expected_type[kind])
    if kind == "acoustic":
        config = extractor.scene.sensor.config
        assert (config.sample_rate_hz, config.block_samples) == (16000, 160)
        assert stream.rate_hz == config.sample_rate_hz / config.block_samples == 100
        assert len(config.channel_offsets_m) == 1
        assert config.channel_offsets_m[0] == pytest.approx([0, 0, 0.14])
        assert config.sensitivity_volts_per_pa == 0.02
        assert config.calibration_metadata["calibration_id"] == "simulated-reference-v1"
        assert extractor.scene.receiver_body_name == "base_link"
        assert extractor.scene.sources[0].body_name == "fixture_emitter"
    else:
        camera = extractor.camera.calibration
        assert (camera.width, camera.height, camera.fx_px, camera.fy_px, camera.cx_px, camera.cy_px) == (
            64, 48, 32.0, 32.0, 32.0, 24.0,
        )
        assert extractor.camera.body_name == "base_link"
        if kind == "thermal":
            assert extractor.config.camera_calibration["calibration_id"] == "simulated-reference-v1"
            assert extractor.config.background_temperature_k == 293.15
            assert extractor.config.temperature_mode == "apparent"
        else:
            assert extractor.labels == {"fixture_target": (2, 7)}
    assert extractor.measurement_settings()["noise"] == declaration["measurement"]["noise"]


def test_world_properties_are_required_not_silently_inferred(fixture_world):
    resolver, path, _ = fixture_world
    del resolver.find_package("multimodal_contract_fixture@1.0.0").data["measurements"]
    with pytest.raises(CatalogError, match="explicit world measurements"):
        resolver.resolve(path)


@pytest.mark.parametrize("group", ["thermal_surfaces", "semantic_labels", "acoustic_sources"])
def test_world_properties_must_name_real_fixture_geometry_or_bodies(fixture_world, group):
    resolver, path, _ = fixture_world
    scene = resolver.find_package("multimodal_contract_fixture@1.0.0").data["measurements"]
    if group == "acoustic_sources":
        scene[group][0]["body_name"] = "missing_body"
    else:
        scene[group]["missing_geom"] = copy.deepcopy(scene[group]["fixture_target"])
    with pytest.raises(CatalogError, match="world MJCF"):
        resolver.resolve(path)
