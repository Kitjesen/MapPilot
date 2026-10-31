"""Versioned navigation suites expose optional sensors without changing defaults."""

from __future__ import annotations

import copy
import json
import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from sim.catalog import CatalogResolver
from sim.tools.sensor_selection import build_sensor_setup, write_sensor_selection

ROOT = Path(__file__).resolve().parents[2]
RIG_ROOT = ROOT / "sim/packages/sensor_rigs/doso/thunder_v4"
EXTRAS = ("front_thermal", "front_semantic", "microphone", "gnss")
SUITES = ("navigation", "headless_navigation")
SESSIONS = (
    ("sim/sessions/examples/thunderv4_factory_park_hf/session.yaml", "navigation"),
    ("sim/sessions/examples/thunderv4_open_field_hf/session.yaml", "navigation"),
    ("sim/sessions/products/doso/thunder_v4/default.yaml", "headless_navigation"),
)
MESSAGES = {
    "front_thermal": ("thermal", "ThermalImage", 10, "mujoco_measurement"),
    "front_semantic": ("semantic", "SemanticImage", 10, "mujoco_measurement"),
    "microphone": ("acoustic", "AcousticBlock", 100, "mujoco_measurement"),
    "gnss": ("gnss", "GnssFix", 10, "mujoco_gnss"),
}


@pytest.fixture(scope="module")
def repository():
    return CatalogResolver.from_repository(ROOT)


@pytest.fixture
def empty_measurement_world(repository, monkeypatch):
    """Declare no measured properties, in memory only; never invent site data."""
    for relative, _ in SESSIONS:
        session = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
        world = repository.find_package(session["world"], kind="world")
        monkeypatch.setitem(world.data, "measurements", {})
    return repository


def _validate(document):
    import jsonschema
    import referencing

    schemas = [json.loads(path.read_text(encoding="utf-8")) for path in (ROOT / "sim/contracts/schemas").glob("*.json")]
    registry = referencing.Registry().with_resources(
        (schema["$id"], referencing.Resource.from_contents(schema)) for schema in schemas
    )
    schema = next(schema for schema in schemas if schema["$id"] == document["schema"])
    jsonschema.Draft202012Validator(schema, registry=registry).validate(document)


@pytest.mark.parametrize("suite", SUITES)
def test_versioned_suite_preserves_five_declarations_and_appends_four(repository, suite):
    identity = f"thunderv4_{suite}"
    old = repository.find_package(f"{identity}@1.0.0", kind="sensor_rig")
    new = repository.find_package(f"{identity}@1.1.0", kind="sensor_rig")
    assert new.manifest_path == RIG_ROOT / suite / "v1.1.0/sensor-rig.package.yaml"
    assert len(old.data["sensors"]) == 5
    assert new.data["sensors"][:5] == old.data["sensors"]
    assert tuple(sensor["id"] for sensor in new.data["sensors"][5:]) == EXTRAS
    verified = repository.find_package("thunderv4_multimodal_unreal@1.0.0").data["sensors"][5:]
    assert new.data["sensors"][5:8] == verified
    assert new.data["sensors"][8] == repository.find_package("thunderv4_gnss@1.0.0").data["sensors"][0]
    _validate(old.data)
    _validate(new.data)


@pytest.mark.parametrize(("relative", "suite"), SESSIONS)
def test_source_sessions_expose_nine_rows_but_compile_legacy_five(repository, tmp_path, relative, suite):
    source = ROOT / relative
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    setup = build_sensor_setup(repository, source, product="inspection")
    assert setup["rig"] == {"id": f"thunderv4_{suite}", "version": "1.1.0"}
    rows = setup["sensors"]
    assert len(rows) == 9
    assert tuple(row["id"] for row in rows[5:]) == EXTRAS
    assert all(row["enabled"] for row in rows[:5])
    assert all(not row["enabled"] and not row["required"] for row in rows[5:])
    assert document["robots"][0]["sensor_overrides"] == {identity: {"enabled": False} for identity in EXTRAS}

    legacy = copy.deepcopy(document)
    legacy["robots"][0]["sensor_rig"] = f"thunderv4_{suite}@1.0.0"
    del legacy["robots"][0]["sensor_overrides"]
    legacy_path = tmp_path / "legacy.yaml"
    legacy_path.write_text(yaml.safe_dump(legacy), encoding="utf-8")
    before = repository.resolve(legacy_path)
    after = repository.resolve(source)
    assert sum(map(len, after.sensor_plan["streams"].values())) == 5
    assert after.sensor_plan == before.sensor_plan
    assert after.transport_intent == before.transport_intent
    assert after.control_plan == before.control_plan
    assert after.visual_plan == before.visual_plan
    physics = copy.deepcopy(after.physics_plan)
    physics["robots"][0]["sensor_rig"] = before.physics_plan["robots"][0]["sensor_rig"]
    assert physics == before.physics_plan
    _validate(document)
    _validate(after.sensor_plan)
    _validate(after.transport_intent)


@pytest.mark.parametrize(("relative", "suite"), SESSIONS)
@pytest.mark.parametrize("enabled", [*((identity,) for identity in EXTRAS), EXTRAS], ids=[*EXTRAS, "all_extras"])
def test_enabled_session_compiles_optional_typed_routes_without_changing_legacy_streams(
    empty_measurement_world,
    tmp_path,
    relative,
    suite,
    enabled,
):
    repository = empty_measurement_world
    source = ROOT / relative
    original = source.read_bytes()
    baseline = repository.resolve(source)
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    document["robots"][0]["sensor_overrides"] = {identity: {"enabled": identity in enabled} for identity in EXTRAS}
    authored = tmp_path / "enabled.yaml"
    authored.write_text(yaml.safe_dump(document), encoding="utf-8")
    assert document["robots"][0]["sensor_rig"] == f"thunderv4_{suite}@1.1.0"
    resolved = repository.resolve(authored)
    streams = resolved.sensor_plan["streams"]
    assert sum(map(len, streams.values())) == 5 + len(enabled)
    assert set(streams) == set(baseline.sensor_plan["streams"]) | {MESSAGES[item][0] for item in enabled}
    for kind, group in baseline.sensor_plan["streams"].items():
        assert streams[kind] == group
    assert [
        channel
        for channel in resolved.transport_intent["channels"]
        if channel["payload_role"] not in {spec[0] for spec in MESSAGES.values()}
    ] == baseline.transport_intent["channels"]
    for identity in enabled:
        kind, message, rate, provider = MESSAGES[identity]
        (stream,) = streams[kind]
        (channel,) = [item for item in resolved.transport_intent["channels"] if item["payload_role"] == kind]
        assert stream["sensor_id"] == channel["channel_id"] == f"thunder_01.{identity}"
        assert stream["message_type"] == channel["message_type"] == f"lingtu.dds.{message}"
        assert stream["rate_hz"] == rate
        for declaration in (stream, channel):
            assert (declaration["owner"], declaration["source"], declaration["transport"]) == (
                "physics",
                provider,
                "typed_dds",
            )
    if any(identity != "gnss" for identity in enabled):
        assert resolved.sensor_plan["scene_measurements"] == {}
    else:
        assert "scene_measurements" not in resolved.sensor_plan
    _validate(document)
    _validate(resolved.sensor_plan)
    _validate(resolved.transport_intent)
    assert source.read_bytes() == original


@pytest.mark.parametrize(("relative", "suite"), SESSIONS)
def test_selector_authors_gnss_opt_in_without_changing_source(repository, relative, suite):
    source = ROOT / relative
    original = source.read_bytes()
    setup = build_sensor_setup(repository, source)
    selection = {
        "schema": "lingtu.sim.sensor-selection.v1",
        "instance_id": setup["instance_id"],
        "sensors": [
            {"id": row["id"], "enabled": row["enabled"] or row["id"] == "gnss", "frequency_hz": row["frequency_hz"]}
            for row in setup["sensors"]
        ],
    }
    with tempfile.TemporaryDirectory(prefix="sensor-suite-", dir=ROOT / "build") as temporary:
        authored = write_sensor_selection(ROOT, source, selection, Path(temporary) / "authored", product="nav")
        resolved = repository.resolve(authored)
        assert resolved.session["robots"][0]["sensor_rig"] == f"thunderv4_{suite}@1.1.0"
        assert sum(map(len, resolved.sensor_plan["streams"].values())) == 6
        assert resolved.sensor_plan["streams"]["gnss"][0]["gnss"]["fix_available"] is False
        _validate(resolved.session)
    assert source.read_bytes() == original


@pytest.fixture(params=SUITES)
def enabled_suite(empty_measurement_world, tmp_path, request):
    relative = next(relative for relative, suite in SESSIONS if suite == request.param)
    session = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
    session["robots"][0]["sensor_overrides"] = {identity: {"enabled": True} for identity in EXTRAS}
    path = tmp_path / "all_extras.yaml"
    path.write_text(yaml.safe_dump(session), encoding="utf-8")
    return empty_measurement_world.resolve(path)


@pytest.mark.parametrize("kind", ["thermal", "semantic"])
def test_physical_optical_mount_composes_with_real_camera_body(repository, enabled_suite, kind):
    from sim.runtime.sensors.camera_geometry import CameraPose, GimbalPose, compose_camera_pose

    (stream,) = enabled_suite.sensor_plan["streams"][kind]
    assert stream["frame_id"] == stream["measurement"]["body_frame_id"] == "thunder_01/camera1_link"
    robot = repository.find_package("thunderv4@1.0.3")
    tree = ET.parse(robot.manifest_path.parent / robot.data["physics"]["mjcf"]).getroot()
    body = tree.find(".//body[@name='base_link']/body[@name='camera1_link']")
    assert body is not None
    mount = stream["measurement"]["mount_pose"]
    assert mount["position_m"] == [0, 0, 0]
    body_pose = CameraPose(
        tuple(map(float, body.attrib["pos"].split())), tuple(map(float, body.attrib["quat"].split()))
    )
    optical = compose_camera_pose(body_pose, CameraPose(**mount), GimbalPose())
    assert optical.position_m == pytest.approx(body_pose.position_m)
    for axis, expected in zip(((1, 0, 0), (0, 1, 0), (0, 0, 1)), ((0, -1, 0), (0, 0, -1), (1, 0, 0))):
        shifted = compose_camera_pose(optical, CameraPose(position_m=axis), GimbalPose())
        assert [a - b for a, b in zip(shifted.position_m, optical.position_m)] == pytest.approx(expected, abs=1e-8)


@pytest.mark.parametrize("kind", ["thermal", "semantic", "acoustic"])
def test_enabled_measurements_construct_with_real_mounts_without_acquiring_physics(repository, enabled_suite, kind):
    from sim.runtime.sensors import SensorRuntime
    from sim.runtime.sensors.measurement_configuration import make_measurement_extractor

    runtime = SensorRuntime.from_plan(enabled_suite.sensor_plan)
    assert len(runtime.streams) == 9
    stream = next(stream for stream in runtime.streams if stream.stream_kind == kind)
    (declaration,) = enabled_suite.sensor_plan["streams"][kind]

    def no_physics():
        pytest.fail("Contract-only construction must not acquire physics")

    extractor = make_measurement_extractor(stream, declaration["measurement"], {}, no_physics)
    if kind == "acoustic":
        assert declaration["frame_id"] == "thunder_01/payload_top"
        assert declaration["measurement"]["body_frame_id"] == "thunder_01/base_link"
        assert extractor.scene.receiver_body_name == "base_link"
        config = extractor.scene.sensor.config
        assert config.channel_offsets_m[0] == pytest.approx([0, 0, 0.14])
        robot = repository.find_package("thunderv4@1.0.3")
        tree = ET.parse(robot.manifest_path.parent / robot.data["physics"]["mjcf"]).getroot()
        site = tree.find(".//body[@name='base_link']/site[@name='payload_top']")
        assert site is not None
        assert config.channel_offsets_m[0] == pytest.approx(tuple(map(float, site.attrib["pos"].split())))
        assert (config.sample_rate_hz, config.block_samples) == (16000, 160)
        assert extractor.scene.sources == ()
    else:
        assert extractor.camera.body_name == "camera1_link"
        assert extractor.camera.mount_pose.position_m == pytest.approx((0, 0, 0))


def test_enabled_gnss_remains_synthetic_zero_origin_no_fix_and_noise_off(enabled_suite):
    from sim.runtime.sensors import SensorRuntime
    from sim.runtime.sensors.gnss import GnssConfig, GnssExtractor

    from runtime.msgs.gnss import GnssFixType

    (declaration,) = enabled_suite.sensor_plan["streams"]["gnss"]
    config = GnssConfig.from_mapping(declaration["gnss"])
    assert (config.origin_latitude_deg, config.origin_longitude_deg, config.origin_altitude_m) == (0, 0, 0)
    assert config.world_origin_m == (0, 0, 0)
    assert config.world_to_enu_yaw_deg == 0
    assert config.fix_available is config.noise_enabled is False
    assert config.horizontal_std_m == config.vertical_std_m == config.seed == 0
    assert declaration["frame_id"] == "thunder_01/payload_top"
    assert config.body_frame_id == "thunder_01/base_link"
    assert config.antenna_offset_m == pytest.approx((0, 0, 0.14))
    runtime = SensorRuntime.from_plan(enabled_suite.sensor_plan)
    sample = next(
        sample
        for sample in runtime.advance(sim_time_ns=0, model_generation=1, reset_generation=0).samples
        if sample.stream.stream_kind == "gnss"
    )
    measured = GnssExtractor(config)(
        sample,
        {
            "session_id": enabled_suite.session_id,
            "model_generation": 1,
            "reset_generation": 0,
            "sim_time_ns": 0,
            "bodies": [{"stable_id": config.body_frame_id}],
        },
    )
    assert measured.fix.fix_type == GnssFixType.NO_FIX
    assert all(
        math.isnan(value) for value in (measured.fix.lat, measured.fix.lon, measured.fix.alt, *measured.fix.covariance)
    )
    assert measured.fix.num_sat == measured.fix.num_sat_used == 0


@pytest.mark.usefixtures("allow_unbuilt_process_artifacts")
@pytest.mark.parametrize("platform", ["windows", "linux"])
def test_formal_inspection_compiler_uses_headless_default_with_only_five_streams(monkeypatch, platform):
    from lingtu.assembly.compile_plan import compile_run_plan

    monkeypatch.setattr("lingtu.assembly.graph.processes._host_process_platform", lambda: platform)
    plan = compile_run_plan("inspection", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco"})
    simulation = plan.simulation
    assert simulation["session_source"] == "sim/sessions/products/doso/thunder_v4/default.yaml"
    assert simulation["session"]["robots"][0]["sensor_rig"] == "thunderv4_headless_navigation@1.1.0"
    assert simulation["session"]["runtime"]["mode"] == "headless"
    assert {kind: group[0]["rate_hz"] for kind, group in simulation["sensor_plan"]["streams"].items()} == {
        "imu": 200,
        "mid360": 10,
        "rgb": 1,
        "depth": 1,
        "truth_odom": 100,
    }
    assert sum(map(len, simulation["sensor_plan"]["streams"].values())) == 5
