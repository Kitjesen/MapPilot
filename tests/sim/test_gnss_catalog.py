"""Opt-in GNSS catalog -> schema -> Sensor Runtime contract, without live engines."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import pytest
import yaml
from sim.catalog import CatalogError, CatalogResolver
from sim.runtime.sensors.contracts import SensorRoute
from sim.runtime.sensors.factory import GnssEndpointFactory
from sim.runtime.sensors.gnss import GnssExtractor
from sim.runtime.sensors.runtime import SensorRuntime
from sim.runtime.sensors.session import SessionSensorRuntime

from runtime.msgs.gnss import GnssFixType

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = REPO_ROOT / "sim/sessions/examples/thunderv4_gnss/session.yaml"
LEGACY_STREAMS = {"rgb", "depth", "imu", "mid360", "truth_odom"}
pytestmark = pytest.mark.sim


@pytest.fixture
def resolver() -> CatalogResolver:
    return CatalogResolver.from_repository(REPO_ROOT)


def _sensor(resolver: CatalogResolver) -> dict:
    return resolver.find_package("thunderv4_gnss@1.0.0").data["sensors"][0]


def _schema(name: str) -> dict:
    return json.loads((REPO_ROOT / f"sim/contracts/schemas/{name}.v1.json").read_text(encoding="utf-8"))


def _validate(document: dict, name: str) -> None:
    jsonschema = pytest.importorskip("jsonschema")
    schema = _schema(name)
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.Draft202012Validator(schema).validate(document)


def _scheduled(resolved):
    runtime = SensorRuntime.from_plan(resolved.sensor_plan)
    return runtime.advance(sim_time_ns=0, model_generation=1, reset_generation=0).samples[0]


def _snapshot(resolved, *, time_ns=0, body="thunder_01/base_link") -> dict:
    return {
        "session_id": resolved.session_id,
        "model_generation": 1,
        "reset_generation": 0,
        "sim_time_ns": time_ns,
        "bodies": [{"stable_id": body, "position_m": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]}],
    }


def test_real_gnss_package_resolves_schemas_and_runtime(resolver, tmp_path):
    resolved = resolver.resolve(SESSION)
    _validate(resolver.find_package("gnss@1.0.0").data, "sensor")
    rig = resolver.find_package("thunderv4_gnss@1.0.0").data
    _validate(rig, "sensor-rig")
    assert "synthetic" in rig["description"].lower()
    assert "NOT an actual site" in rig["description"]
    _validate(resolved.sensor_plan, "sensor-plan")
    _validate(resolved.transport_intent, "transport-intent")
    assert set(resolved.sensor_plan["streams"]) == LEGACY_STREAMS | {"gnss"}
    declaration, = resolved.sensor_plan["streams"]["gnss"]
    assert declaration["gnss"]["body_frame_id"] == "thunder_01/base_link"
    assert declaration["gnss"]["antenna_offset_m"] == pytest.approx([0, 0, 0.14])
    channel, = resolved.transport_intent["channels"]
    assert channel == {
        "channel_id": "thunder_01.gnss", "direction": "publish", "owner": "physics",
        "source": "mujoco_gnss", "transport": "typed_dds", "delivery": "ordered_no_drop",
        "message_type": "lingtu.dds.GnssFix", "payload_role": "gnss",
        "contract": "field_dds_v1", "frame_policy": "source_time",
    }
    resolved.write_bundle(tmp_path / "bundle")
    runtime = SensorRuntime.from_path(tmp_path / "bundle/sensor.plan.json")
    stream, = runtime.streams
    assert stream.route == SensorRoute("physics", "mujoco_gnss", "typed_dds")
    assert stream.gnss.fix_available is False
    assert stream.gnss.noise_enabled is False
    assert (stream.gnss.origin_latitude_deg, stream.gnss.origin_longitude_deg, stream.gnss.origin_altitude_m) == (0, 0, 0)

    class CaptureSink:
        def __init__(self):
            self.samples = []
            self.closed = False

        def start(self):
            return {"ready": True}

        def publish(self, sample):
            self.samples.append(sample)

        def close(self):
            self.closed = True

    sink = CaptureSink()
    session = SessionSensorRuntime(
        plan=runtime, allocation=object(),
        endpoint_factory=GnssEndpointFactory(sink_factory=lambda stream, allocation: sink),
    )
    try:
        assert session.prepare().active_sensor_ids == ("thunder_01.gnss",)
        session.process_snapshot(_snapshot(resolved))
        session.process_snapshot(_snapshot(resolved, time_ns=100_000_000))
        assert [sample.fix.ts for sample in sink.samples] == [0, 0.1]
        assert [sample.fix.seq for sample in sink.samples] == [0, 1]
        assert all(sample.fix.fix_type == GnssFixType.NO_FIX for sample in sink.samples)
        assert all(math.isnan(sample.fix.lat) for sample in sink.samples)
    finally:
        session.close()
    assert sink.closed


@pytest.mark.parametrize("qualified", [False, True])
def test_body_frame_is_qualified_once_for_each_instance(resolver, tmp_path, qualified):
    session = yaml.safe_load(SESSION.read_text(encoding="utf-8"))
    session["robots"][0]["instance_id"] = "receiver_02"
    _sensor(resolver)["configuration"]["gnss"]["body_frame_id"] = (
        "receiver_02/base_link" if qualified else "base_link"
    )
    path = tmp_path / "session.yaml"
    path.write_text(yaml.safe_dump(session), encoding="utf-8")
    resolved = resolver.resolve(path)
    _validate(resolved.sensor_plan, "sensor-plan")
    sample = _scheduled(resolved)
    assert sample.stream.gnss.body_frame_id == "receiver_02/base_link"
    assert GnssExtractor(sample.stream.gnss)(sample, _snapshot(resolved, body="receiver_02/base_link")).fix.fix_type == (
        GnssFixType.NO_FIX
    )


@pytest.mark.parametrize("explicit_extrinsic", [False, True])
def test_antenna_offset_includes_mount_or_rig_extrinsic(resolver, explicit_extrinsic):
    sensor = _sensor(resolver)
    gnss = sensor["configuration"]["gnss"]
    gnss["antenna_offset_m"] = [0, 0, 0.1]
    gnss["fix_available"] = True
    if explicit_extrinsic:
        sensor["extrinsic"] = {"position_m": [1, 2, 3], "quaternion_wxyz": [1, 0, 0, 0]}
    resolved = resolver.resolve(SESSION)
    sample = _scheduled(resolved)
    expected = [1, 2, 3.1] if explicit_extrinsic else [0, 0, 0.24]
    assert sample.stream.gnss.antenna_offset_m == pytest.approx(expected)
    fix = GnssExtractor(sample.stream.gnss)(sample, _snapshot(resolved)).fix
    assert fix.fix_type == GnssFixType.SIMULATION
    assert fix.alt == pytest.approx(expected[2], abs=1e-5)


def test_mount_ancestry_rotates_and_translates_antenna(resolver):
    sensor = _sensor(resolver)
    sensor["parent_frame"] = "lidar1_link_site"
    sensor["extrinsic"] = {"position_m": [0, 0, 0.2], "quaternion_wxyz": [1, 0, 0, 0]}
    gnss = sensor["configuration"]["gnss"]
    gnss["antenna_offset_m"] = [0, 0, 0.1]
    # The declared lidar mount rotates Z towards X; the site adds another local 0.2 m.
    frame = next(frame for frame in resolver.find_package("thunderv4@1.0.3").data["frames"]
                 if frame["name"] == "lidar1_link")
    frame["extrinsic"] = {
        "position_m": [1, 2, 3], "quaternion_wxyz": [math.sqrt(0.5), 0, math.sqrt(0.5), 0],
    }
    sample = _scheduled(resolver.resolve(SESSION))
    assert sample.stream.gnss.antenna_offset_m == pytest.approx([1.3, 2, 3])


def test_body_mount_does_not_apply_its_own_pose_twice(resolver):
    sensor = _sensor(resolver)
    sensor["parent_frame"] = "lidar1_link"
    sensor["configuration"]["gnss"].update(body_frame_id="lidar1_link", antenna_offset_m=[0, 0, 0.1])
    stream = _scheduled(resolver.resolve(SESSION)).stream
    assert stream.gnss.body_frame_id == "thunder_01/lidar1_link"
    assert stream.gnss.antenna_offset_m == pytest.approx([0, 0, 0.1])


def test_missing_optional_knobs_keep_no_fix_defaults(resolver):
    config = _sensor(resolver)["configuration"]["gnss"]
    for name in tuple(config):
        if name not in {"origin_latitude_deg", "origin_longitude_deg", "origin_altitude_m", "body_frame_id"}:
            del config[name]
    resolved = resolver.resolve(SESSION)
    _validate(resolved.sensor_plan, "sensor-plan")
    scheduled = _scheduled(resolved)
    assert scheduled.stream.gnss.fix_available is False
    assert scheduled.stream.gnss.noise_enabled is False
    assert GnssExtractor(scheduled.stream.gnss)(scheduled, _snapshot(resolved)).fix.fix_type == GnssFixType.NO_FIX


@pytest.mark.parametrize("missing", ["origin_latitude_deg", "origin_longitude_deg", "origin_altitude_m", "body_frame_id"])
def test_catalog_rejects_missing_georeference(resolver, missing):
    del _sensor(resolver)["configuration"]["gnss"][missing]
    with pytest.raises(CatalogError, match=missing):
        resolver.resolve(SESSION)


@pytest.mark.parametrize("missing", ["configuration", "gnss"])
def test_rig_schema_and_catalog_require_gnss_configuration(resolver, missing):
    jsonschema = pytest.importorskip("jsonschema")
    sensor = _sensor(resolver)
    if missing == "configuration":
        del sensor["configuration"]
    else:
        del sensor["configuration"]["gnss"]
    with pytest.raises(jsonschema.ValidationError):
        _validate(resolver.find_package("thunderv4_gnss@1.0.0").data, "sensor-rig")
    with pytest.raises(CatalogError, match="gnss"):
        resolver.resolve(SESSION)


@pytest.mark.parametrize(("field", "value"), [
    ("origin_latitude_deg", 91), ("origin_longitude_deg", -181),
    ("origin_altitude_m", float("nan")), ("origin_altitude_m", float("inf")),
    ("origin_latitude_deg", True), ("world_to_enu_yaw_deg", "north"),
    ("noise_enabled", 1), ("fix_available", "false"),
    ("horizontal_std_m", -1), ("vertical_std_m", -0.1),
    ("seed", True), ("seed", -1), ("seed", 1.5), ("seed", 1 << 64),
    ("antenna_offset_m", [0, 0]), ("antenna_offset_m", [0, False, 0]),
    ("world_origin_m", [0, 0, float("inf")]),
    ("body_frame_id", "other/base_link"), ("body_frame_id", "missing"),
    ("body_frame_id", "payload_top"), ("body_frame_id", " base_link"),
    ("unsupported_knob", True),
])
def test_catalog_rejects_bad_gnss_configuration(resolver, field, value):
    _sensor(resolver)["configuration"]["gnss"][field] = value
    with pytest.raises(CatalogError, match=field):
        resolver.resolve(SESSION)


@pytest.mark.parametrize(("field", "value"), [
    ("owner", "visual"), ("source", "mujoco_truth"),
    ("transport", "in_process"), ("message_type", "lingtu.dds.Odometry"),
])
def test_catalog_rejects_wrong_gnss_route(resolver, field, value):
    if field in {"owner", "source"}:
        _sensor(resolver)["runtime"][field] = value
    else:
        resolver.find_package("gnss@1.0.0").data["interface"][field] = value
    with pytest.raises(CatalogError, match="GNSS requires"):
        resolver.resolve(SESSION)


@pytest.mark.parametrize("schema_name", ["sensor-plan", "sensor-rig"])
@pytest.mark.parametrize(("field", "value"), [
    ("origin_latitude_deg", -91), ("origin_longitude_deg", 181),
    ("origin_altitude_m", True), ("noise_enabled", 0), ("fix_available", "false"),
    ("horizontal_std_m", -1), ("vertical_std_m", -1),
    ("seed", -1), ("seed", 1 << 64), ("antenna_offset_m", [0, 0]),
    ("world_origin_m", [0, 0, "zero"]), ("typo", 0),
])
def test_schemas_reject_bad_gnss_configuration(resolver, schema_name, field, value):
    jsonschema = pytest.importorskip("jsonschema")
    if schema_name == "sensor-plan":
        document = resolver.resolve(SESSION).sensor_plan
        config = document["streams"]["gnss"][0]["gnss"]
    else:
        document = copy.deepcopy(resolver.find_package("thunderv4_gnss@1.0.0").data)
        config = document["sensors"][0]["configuration"]["gnss"]
    config[field] = value
    with pytest.raises(jsonschema.ValidationError):
        _validate(document, schema_name)


@pytest.mark.parametrize("relative", [
    "sim/sessions/products/doso/thunder_v4/default.yaml",
    "sim/sessions/examples/thunder_omni_contract/session.yaml",
    "sim/sessions/examples/thunderv4_controlled_headless/session.yaml",
])
def test_existing_sessions_keep_five_stream_keys_and_no_gnss_channels(resolver, relative):
    resolved = resolver.resolve(REPO_ROOT / relative)
    assert set(resolved.sensor_plan["streams"]) == LEGACY_STREAMS
    assert all(channel["payload_role"] != "gnss" for channel in resolved.transport_intent["channels"])


def test_disabled_sensor_binding_does_not_emit_gnss(resolver, tmp_path):
    session = yaml.safe_load(SESSION.read_text(encoding="utf-8"))
    session["runtime"]["required_bindings"] = ["physics"]
    path = tmp_path / "session.yaml"
    path.write_text(yaml.safe_dump(session), encoding="utf-8")
    resolved = resolver.resolve(path)
    assert set(resolved.sensor_plan["streams"]) == LEGACY_STREAMS
    assert resolved.transport_intent["channels"] == []
