"""Inspection captures use typed raw measurements, not fabricated summaries."""

import json
from dataclasses import asdict, replace
from io import BytesIO

import numpy as np
import pytest

from perception.inspection.bridge_module import InspectionEvidenceModule
from runtime.contracts.inspection_evidence import InspectionEvidenceStore
from runtime.contracts.sensor_control import SensorControlResult
from runtime.msgs.measurement import AcousticMeasurement, MeasurementIdentity, SemanticMeasurement, ThermalMeasurement
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import Image

NOW = 1_800_000_000.0


class Bridge:
    def __init__(self, action):
        self.requests = [dict(
            task_id="task", route_id="route", route_revision=1, map_id="map",
            map_content_epoch=1, point_index=0, request_id="capture", action=action,
            requested_at_s=NOW - 0.1, deadline_s=NOW + 10,
        )]
        self.results = []

    def take_request(self):
        return self.requests.pop(0) if self.requests else None

    def write_result(self, **result):
        self.results.append(result)

    def close(self):
        pass


def identity(sensor="thermal"):
    return MeasurementIdentity("product", "source", "robot", sensor, sensor + "-frame", 2, 3, 7, 10**9)


def binding(sensor="thermal"):
    stamp = asdict(identity(sensor))
    for key in ("product_session_id", "sensor_id", "sequence", "sim_time_ns"):
        del stamp[key]
    return dict(modality=sensor, **stamp)


def metadata(sensor="thermal"):
    stamp = asdict(identity(sensor))
    del stamp["product_session_id"]
    return dict(
        modality=sensor, stamp=stamp,
        units={"temperature_k": "K", "valid_mask": "1"},
        calibration={"calibration_id": "test-cal"},
        provenance={"model_id": "test-model", "model_version": "v1", "qualification": "local-contract-only"},
        frame_metadata={"source_clock": {"mapped_epoch_s": NOW}, "temperature_kind": "apparent"},
    )


def thermal(**changes):
    values = dict(identity=identity(), temperature_k=np.array([[300., 340.]], dtype=np.float32),
                  valid_mask=np.ones((1, 2), bool), metadata=metadata(), received_at_s=NOW)
    return ThermalMeasurement(**(values | changes))


def thermal_rule():
    return dict(max_kelvin=330., roi=[0, 0, 2, 1], min_valid_coverage=1., temperature_kind="apparent",
                calibration_id="test-cal", model_id="test-model", model_version="v1")


def module_for(tmp_path, monkeypatch, action="capture:thermal", *, odom_ts=NOW, **config):
    monkeypatch.setattr("perception.inspection.bridge_module.time.time", lambda: NOW)
    bridge = Bridge(action)
    module = InspectionEvidenceModule(
        evidence_root=tmp_path / "evidence", status_file=tmp_path / "status.json",
        bridge_factory=lambda: bridge, start_thread=False, jpeg_encoder=lambda _: b"rgb-original",
        **(dict(product_session_id="product", measurement_sensors={"thermal": binding()},
                measurement_rules={"thermal": thermal_rule()}) | config),
    )
    module.setup()
    publish(module.color_image, Image(np.zeros((1, 2, 3), np.uint8), ts=NOW))
    publish(module.odometry, Odometry(ts=odom_ts))
    return module, bridge


def publish(port, value):
    port._deliver(value)
    assert port.callback_errors == 0


def captured(tmp_path):
    return InspectionEvidenceStore(tmp_path / "evidence").get("capture")


def test_disabled_source_ack_blocks_fresh_cached_measurement_capture(tmp_path, monkeypatch):
    module, bridge = module_for(tmp_path, monkeypatch)
    publish(module.thermal, thermal())
    publish(module.sensor_control_state, SensorControlResult("off", "product", "source", "thermal", 2, 3,
            10**9, True, "applied", {"enabled": False}))
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is False
    assert bridge.results[0]["reason"] == "measurement_disabled:thermal"
    assert bridge.results[0]["analysis_verdict"] == "inconclusive"


def test_thermal_actual_values_share_rgb_pose_transaction(tmp_path, monkeypatch):
    module, bridge = module_for(tmp_path, monkeypatch)
    sample = thermal()
    publish(module.thermal, sample)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    result = captured(tmp_path)
    records = result.manifest["persistence"]["artifacts"]
    assert {record["kind"] for record in records} == {"rgb", "pose", "detections", "measurement"}
    raw = next(record for record in records if record["kind"] == "measurement")
    with np.load(BytesIO((result.evidence_dir / raw["path"]).read_bytes()), allow_pickle=False) as arrays:
        np.testing.assert_array_equal(arrays["temperature_k"], sample.temperature_k)
        np.testing.assert_array_equal(arrays["valid_mask"], sample.valid_mask)
        assert json.loads(str(arrays["metadata"].item())) == raw["metadata"]
        assert arrays["temperature_k"].dtype == sample.temperature_k.dtype
    assert raw["metadata"]["stamp"] == sample.metadata["stamp"]
    assert raw["metadata"]["product_session_id"] == "product"
    assert raw["metadata"]["captured_at_s"] == NOW
    assert (result.evidence_dir / "rgb.jpg").read_bytes() == b"rgb-original"
    assert result.analysis_verdict == "anomaly"
    assert result.manifest["analysis"]["observation"]["details"]["metrics"]["max_kelvin"] == 340.


@pytest.mark.parametrize("sequence,sim_time_ns", [(6, 900_000_000), (7, 10**9), (8, 900_000_000)])
@pytest.mark.parametrize("modality", ["thermal", "acoustic"])
def test_late_or_duplicate_sample_cannot_replace_latest_raw_anomaly(
    tmp_path, monkeypatch, sequence, sim_time_ns, modality,
):
    factory = thermal if modality == "thermal" else acoustic
    rule = thermal_rule() if modality == "thermal" else acoustic_rule()
    module, bridge = module_for(tmp_path, monkeypatch, f"capture:{modality}",
                                measurement_sensors={modality: binding(modality)}, measurement_rules={modality: rule})
    latest = factory()
    port = getattr(module, modality)
    publish(port, latest)
    raw_name = "temperature_k" if modality == "thermal" else "pressure_pa"
    older = factory(identity=replace(identity(modality), sequence=sequence, sim_time_ns=sim_time_ns),
                    **{raw_name: np.full_like(getattr(latest, raw_name), 300. if modality == "thermal" else 0.)})
    older.metadata["stamp"].update(sequence=sequence, sim_time_ns=sim_time_ns)
    publish(port, older)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    result = captured(tmp_path)
    assert result.analysis_verdict == "anomaly"
    raw = next(record for record in result.manifest["persistence"]["artifacts"] if record["kind"] == "measurement")
    assert raw["metadata"]["stamp"] == latest.metadata["stamp"]
    with np.load(result.evidence_dir / raw["path"], allow_pickle=False) as arrays:
        np.testing.assert_array_equal(arrays[raw_name], getattr(latest, raw_name))


@pytest.mark.parametrize("modality", ["thermal", "acoustic"])
@pytest.mark.parametrize("valid,verdict", [(False, "inconclusive"), (True, "normal")])
def test_latest_sample_is_used_including_invalid_raw_without_falling_back(
    tmp_path, monkeypatch, modality, valid, verdict,
):
    factory = thermal if modality == "thermal" else acoustic
    rule = thermal_rule() if modality == "thermal" else acoustic_rule()
    module, bridge = module_for(tmp_path, monkeypatch, f"capture:{modality}",
                                measurement_sensors={modality: binding(modality)}, measurement_rules={modality: rule})
    sample = factory()
    raw_name = "temperature_k" if modality == "thermal" else "pressure_pa"
    older = replace(sample, **{raw_name: np.full_like(getattr(sample, raw_name), 300. if modality == "thermal" else 0.)})
    publish(getattr(module, modality), older if not valid else sample)
    latest = replace(
        older, identity=replace(identity(modality), sequence=8, sim_time_ns=1_100_000_000),
        valid_mask=np.full_like(sample.valid_mask, valid),
    )
    latest.metadata["stamp"].update(sequence=8, sim_time_ns=1_100_000_000)
    publish(getattr(module, modality), latest)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    assert captured(tmp_path).analysis_verdict == verdict


def acoustic(**changes):
    meta = metadata("acoustic")
    meta["units"] = {"pressure_pa": "Pa", "voltage_v": "V", "valid_mask": "1"}
    values = dict(identity=identity("acoustic"), pressure_pa=np.array([[2., -2., 2., -2.]], np.float32),
                  valid_mask=np.ones((1, 4), bool), sample_rate_hz=4,
                  voltage_v=np.array([[.02, -.02, .02, -.02]], np.float32), metadata=meta, received_at_s=NOW)
    return AcousticMeasurement(**(values | changes))


def acoustic_rule():
    return dict(rms_pa=2., channels=[0], min_duration_s=1., min_valid_coverage=1.,
                calibration_id="test-cal", model_id="test-model", model_version="v1")


def test_acoustic_actual_pressure_voltage_and_sample_rate_are_stored(tmp_path, monkeypatch):
    module, bridge = module_for(
        tmp_path, monkeypatch, "capture:acoustic",
        measurement_sensors={"acoustic": binding("acoustic"), "thermal": binding()},
        measurement_rules={"acoustic": acoustic_rule()},
    )
    sample = acoustic()
    publish(module.acoustic, sample)
    publish(module.thermal, thermal())
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    result = captured(tmp_path)
    records = [r for r in result.manifest["persistence"]["artifacts"] if r["kind"] == "measurement"]
    assert [r["sensor_id"] for r in records] == ["acoustic"]
    raw = records[0]
    with np.load(BytesIO((result.evidence_dir / raw["path"]).read_bytes()), allow_pickle=False) as arrays:
        np.testing.assert_array_equal(arrays["pressure_pa"], sample.pressure_pa)
        np.testing.assert_array_equal(arrays["voltage_v"], sample.voltage_v)
        np.testing.assert_array_equal(arrays["valid_mask"], sample.valid_mask)
    assert raw["metadata"]["sample_rate_hz"] == 4
    assert result.analysis_verdict == "anomaly"
    assert result.manifest["analysis"]["observation"]["details"]["metrics"]["rms_pa"] == [2.]


@pytest.mark.parametrize("field,value", [
    ("product_session_id", "other"), ("session_id", "other"), ("instance_id", "other"),
    ("frame_id", "other"), ("model_generation", 1), ("reset_generation", 2),
])
def test_measurement_identity_must_match_declared_product_source_and_generation(tmp_path, monkeypatch, field, value):
    module, bridge = module_for(tmp_path, monkeypatch)
    publish(module.thermal, thermal(identity=replace(identity(), **{field: value})))
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is False
    assert bridge.results[0]["analysis_verdict"] == "inconclusive"
    assert bridge.results[0]["reason"] == "measurement_identity_mismatch:thermal"
    assert not (tmp_path / "evidence" / "requests" / "capture").exists()


@pytest.mark.parametrize("damage,reason", [
    ("missing", "measurement_missing:thermal"),
    ("unmapped", "measurement_clock_unmapped:thermal"),
    ("stale", "measurement_stale:thermal"),
    ("future", "measurement_stale:thermal"),
    ("stale_receipt", "measurement_stale:thermal"),
    ("rgb_skew", "measurement_rgb_unsynchronized:thermal"),
    ("odom_skew", "measurement_odometry_unsynchronized:thermal"),
])
def test_measurement_requires_explicit_fresh_source_clock_and_rgb_pose_match(tmp_path, monkeypatch, damage, reason):
    module, bridge = module_for(tmp_path, monkeypatch, odom_ts=NOW - .15 if damage == "odom_skew" else NOW)
    if damage != "missing":
        sample = thermal()
        if damage == "unmapped":
            sample.metadata["frame_metadata"].pop("source_clock")
        elif damage == "stale_receipt":
            sample = replace(sample, received_at_s=NOW - 10)
        else:
            offsets = {"stale": -10., "future": 10., "rgb_skew": -.3, "odom_skew": .15}
            sample.metadata["frame_metadata"]["source_clock"]["mapped_epoch_s"] += offsets[damage]
        publish(module.thermal, sample)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is False
    assert bridge.results[0]["reason"] == reason
    assert bridge.results[0]["analysis_verdict"] == "inconclusive"
    assert not (tmp_path / "evidence" / "requests" / "capture").exists()


def semantic():
    meta = metadata("semantic")
    meta["units"] = {"class_ids": "label", "instance_ids": "label", "valid_mask": "1"}
    meta.update(simulation=True, ground_truth=True)
    return SemanticMeasurement(identity("semantic"), np.array([[1, 2]], np.uint32),
                               np.array([[11, 22]], np.uint32), np.ones((1, 2), bool), meta, NOW)


def test_action_selects_declared_semantic_and_thermal_raw_without_changing_overview_verdict(tmp_path, monkeypatch):
    module, bridge = module_for(
        tmp_path, monkeypatch, "capture:overview",
        measurement_sensors={"thermal": binding(), "acoustic": binding("acoustic"), "semantic": binding("semantic")},
        capture_sensor_ids={"capture:overview": ["thermal", "semantic"]},
    )
    publish(module.thermal, thermal())
    publish(module.acoustic, acoustic())
    sample = semantic()
    publish(module.semantic, sample)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    result = captured(tmp_path)
    records = [r for r in result.manifest["persistence"]["artifacts"] if r["kind"] == "measurement"]
    assert [r["sensor_id"] for r in records] == ["thermal", "semantic"]
    raw = records[1]
    with np.load(BytesIO((result.evidence_dir / raw["path"]).read_bytes()), allow_pickle=False) as arrays:
        np.testing.assert_array_equal(arrays["class_ids"], sample.class_ids)
        np.testing.assert_array_equal(arrays["instance_ids"], sample.instance_ids)
        np.testing.assert_array_equal(arrays["valid_mask"], sample.valid_mask)
    assert raw["metadata"]["ground_truth"] is True
    assert result.analysis_verdict == "not_evaluated"
    assert result.analysis_support == "not_applicable"


@pytest.mark.parametrize("modality,damage,reason", [
    ("thermal", "coverage", "insufficient_valid_coverage"),
    ("thermal", "nan", "insufficient_valid_coverage"),
    ("thermal", "unit", "unit_missing_or_mismatched"),
    ("thermal", "calibration", "calibration_missing_or_mismatched"),
    ("thermal", "provenance", "provenance_missing_or_mismatched"),
    ("acoustic", "duration", "insufficient_duration"),
    ("acoustic", "coverage", "insufficient_valid_coverage"),
])
def test_invalid_or_insufficient_raw_observations_are_saved_but_never_normal(
    tmp_path, monkeypatch, modality, damage, reason,
):
    rule = thermal_rule() if modality == "thermal" else acoustic_rule()
    module, bridge = module_for(
        tmp_path, monkeypatch, f"capture:{modality}",
        measurement_sensors={modality: binding(modality)}, measurement_rules={modality: rule},
    )
    sample = thermal() if modality == "thermal" else acoustic()
    if damage == "coverage":
        sample = replace(sample, valid_mask=np.zeros_like(sample.valid_mask))
    elif damage == "nan":
        sample = replace(sample, temperature_k=np.full((1, 2), np.nan, np.float32))
    elif damage == "unit":
        sample.metadata["units"]["temperature_k"] = "C"
    elif damage == "calibration":
        sample.metadata["calibration"] = None
    elif damage == "provenance":
        sample.metadata.pop("provenance")
    else:
        sample = replace(sample, sample_rate_hz=8)
    publish(getattr(module, modality), sample)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    result = captured(tmp_path)
    assert result.analysis_verdict == "inconclusive"
    assert result.manifest["analysis"]["reason"] == reason


def test_multiple_selected_sensors_cannot_hide_inconclusive_with_last_normal(tmp_path, monkeypatch):
    sensors = {"thermal": binding(), "thermal-b": binding()}
    sensors["thermal-b"]["frame_id"] = "thermal-b-frame"
    rules = {key: thermal_rule() for key in sensors}
    module, bridge = module_for(tmp_path, monkeypatch, measurement_sensors=sensors, measurement_rules=rules)
    publish(module.thermal, thermal(valid_mask=np.zeros((1, 2), bool)))
    other = thermal(temperature_k=np.array([[300., 310.]], np.float32), identity=identity("thermal-b"))
    other.metadata["stamp"] = asdict(other.identity)
    other.metadata["stamp"].pop("product_session_id")
    publish(module.thermal, other)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    assert captured(tmp_path).analysis_verdict == "inconclusive"


@pytest.mark.parametrize("config", [
    {"product_session_id": ""},
    {"measurement_sensors": {"thermal": {"modality": "thermal"}}},
    {"measurement_sensors": {"thermal": binding() | {"reset_generation": True}}},
    {"measurement_sensors": {"thermal": binding() | {"modality": "depth"}}},
    {"measurement_rules": {"unknown": thermal_rule()}},
    {"measurement_rules": {"thermal": thermal_rule() | {"max_kelvin": -1}}},
    {"capture_sensor_ids": {"capture:overview": ["unknown"]}},
    {"capture_sensor_ids": {"capture:overview": "thermal"}},
    {"capture_sensor_ids": {"capture:overview": ["thermal", "thermal"]}},
    {"capture_sensor_ids": {"capture:thermal": []}},
    {"capture_sensor_ids": {"capture:made_up": ["thermal"]}},
    {"max_measurement_age_s": float("nan")},
    {"max_measurement_skew_s": -1.},
])
def test_invalid_measurement_capture_configuration_fails_before_startup(tmp_path, monkeypatch, config):
    with pytest.raises(ValueError):
        module_for(tmp_path, monkeypatch, **config)


def test_missing_declaration_is_explicit_inconclusive_not_store_failure(tmp_path, monkeypatch):
    module, bridge = module_for(tmp_path, monkeypatch, measurement_sensors={}, measurement_rules={})
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is False
    assert bridge.results[0]["analysis_verdict"] == "inconclusive"
    assert bridge.results[0]["reason"] == "measurement_not_configured:thermal"


@pytest.mark.parametrize("port,expected", [
    ("color_image", "measurement_rgb_clock_missing"), ("odometry", "measurement_odometry_clock_missing"),
])
def test_measurements_cannot_match_rgb_pose_receipt_fallback(tmp_path, monkeypatch, port, expected):
    module, bridge = module_for(tmp_path, monkeypatch, odom_ts=None if port == "odometry" else NOW)
    if port == "color_image":
        publish(module.color_image, Image(np.zeros((1, 2, 3), np.uint8), ts=None))
    publish(module.thermal, thermal())
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is False
    assert bridge.results[0]["analysis_verdict"] == "inconclusive"
    assert bridge.results[0]["reason"] == expected


def test_supplemental_sensor_must_share_source_model_reset_identity(tmp_path, monkeypatch):
    sensors = {"thermal": binding(), "semantic": binding("semantic") | {"reset_generation": 4}}
    module, bridge = module_for(tmp_path, monkeypatch, measurement_sensors=sensors,
                                capture_sensor_ids={"capture:thermal": ["thermal", "semantic"]})
    publish(module.thermal, thermal())
    sample = semantic()
    sample = replace(sample, identity=replace(sample.identity, reset_generation=4))
    sample.metadata["stamp"]["reset_generation"] = 4
    publish(module.semantic, sample)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is False
    assert bridge.results[0]["analysis_verdict"] == "inconclusive"
    assert bridge.results[0]["reason"] == "measurement_run_mismatch:semantic"


@pytest.mark.parametrize("modality", ["thermal", "acoustic"])
def test_raw_measurement_without_rule_stays_inconclusive(tmp_path, monkeypatch, modality):
    module, bridge = module_for(tmp_path, monkeypatch, f"capture:{modality}",
                                measurement_sensors={modality: binding(modality)}, measurement_rules={})
    publish(getattr(module, modality), thermal() if modality == "thermal" else acoustic())
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    assert captured(tmp_path).analysis_verdict == "inconclusive"
    assert captured(tmp_path).analysis_support == "unavailable"


def test_valid_measured_silence_without_voltage_is_normal_not_missing(tmp_path, monkeypatch):
    module, bridge = module_for(tmp_path, monkeypatch, "capture:acoustic",
                                measurement_sensors={"acoustic": binding("acoustic")},
                                measurement_rules={"acoustic": acoustic_rule()})
    sample = acoustic(pressure_pa=np.zeros((1, 4), np.float32), voltage_v=None)
    sample.metadata["units"].pop("voltage_v")
    publish(module.acoustic, sample)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    result = captured(tmp_path)
    assert result.analysis_verdict == "normal"
    raw = next(r for r in result.manifest["persistence"]["artifacts"] if r["kind"] == "measurement")
    with np.load(BytesIO((result.evidence_dir / raw["path"]).read_bytes()), allow_pickle=False) as arrays:
        assert "voltage_v" not in arrays.files
        np.testing.assert_array_equal(arrays["pressure_pa"], sample.pressure_pa)


def test_legacy_rgb_defaults_do_not_capture_incidental_measurement_traffic(tmp_path, monkeypatch):
    module, bridge = module_for(tmp_path, monkeypatch, "capture:overview",
                                product_session_id="", measurement_sensors={}, measurement_rules={})
    publish(module.thermal, thermal())
    publish(module.acoustic, acoustic())
    publish(module.semantic, semantic())
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    result = captured(tmp_path)
    assert {r["kind"] for r in result.manifest["persistence"]["artifacts"]} == {"rgb", "pose", "detections"}
    assert result.analysis_verdict == "not_evaluated"


def test_measurement_retry_replays_committed_raw_even_after_source_reset(tmp_path, monkeypatch):
    module, bridge = module_for(tmp_path, monkeypatch)
    retry = dict(bridge.requests[0])
    publish(module.thermal, thermal())
    assert module.poll_once()
    first = captured(tmp_path)
    manifest = first.manifest_path.read_bytes()
    publish(module.thermal, thermal(identity=replace(identity(), reset_generation=4)))
    bridge.requests.append(retry)
    assert module.poll_once()
    assert [r["persisted"] for r in bridge.results] == [True, True]
    assert captured(tmp_path).manifest_path.read_bytes() == manifest


def test_explicit_sensor_selection_does_not_require_unselected_sibling(tmp_path, monkeypatch):
    module, bridge = module_for(
        tmp_path, monkeypatch, measurement_sensors={"thermal": binding(), "thermal-b": binding() | {"frame_id": "thermal-b-frame"}},
        measurement_rules={"thermal-b": thermal_rule()}, capture_sensor_ids={"capture:thermal": ["thermal-b"]},
    )
    sample = thermal(identity=identity("thermal-b"))
    sample.metadata["stamp"]["sensor_id"] = "thermal-b"
    sample.metadata["stamp"]["frame_id"] = "thermal-b-frame"
    publish(module.thermal, sample)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    records = captured(tmp_path).manifest["persistence"]["artifacts"]
    assert [r["sensor_id"] for r in records if r["kind"] == "measurement"] == ["thermal-b"]


def formal_sample(modality, *, temperature_mode="apparent", calibrated=True):
    """Run the actual Product measurement producer; no renderer or live DDS."""
    from contextlib import nullcontext

    from sim.scripts.mujoco.formal_measurements import FormalMeasurements, planned_measurements

    from runtime.endpoints.dds.measurement import measurement_from_record

    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="base"><geom type="sphere" size=".01"/></body>
      <body name="pump" pos="2 0 0"><geom type="sphere" size=".1"/></body>
      <geom name="hot" type="box" pos="0 0 3" size="2 2 .1"/>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    configuration = {"body_frame_id": "robot/base"}
    scene = {}
    if modality in {"thermal", "semantic"}:
        configuration["camera"] = dict(model="pinhole", width=2, height=2, fx_px=2, fy_px=2, cx_px=1, cy_px=1)
        if modality == "thermal":
            configuration["thermal"] = dict(background_temperature_k=293.15, calibration_id="test-cal",
                                              temperature_mode=temperature_mode)
            scene["thermal_surfaces"] = {"hot": {"temperature_k": 400., "emissivity": .8}}
        else:
            scene["semantic_labels"] = {"hot": {"class_id": 7, "instance_id": 12}}
    else:
        microphone = dict(sample_rate_hz=8000, block_samples=80, channel_offsets_m=[[0., 0., 0.]])
        if calibrated:
            microphone.update(sensitivity_volts_per_pa=.02, calibration_metadata={"calibration_id": "test-cal"})
        configuration["microphone"] = microphone
        scene["acoustic_sources"] = [dict(
            source_id="pump-signal", body_name="robot/pump", pressure_pa=[1.] * 8000,
            sample_rate_hz=8000, start_sim_time_ns=0, reference_distance_m=1., local_offset_m=[0., 0., 0.],
        )]
    declaration = dict(
        sensor_id=modality, instance_id="robot", frame_id=f"robot/{modality}", owner="physics",
        source="mujoco_measurement", transport="typed_dds", rate_hz=100 if modality == "acoustic" else 10,
        message_type={"thermal": "lingtu.dds.ThermalImage", "acoustic": "lingtu.dds.AcousticBlock",
                      "semantic": "lingtu.dds.SemanticImage"}[modality],
        measurement=configuration,
    )
    planned = planned_measurements(
        dict(schema="lingtu.sim.sensor-plan.v1", session_id="source", env="sim",
             streams={modality: [declaration]}, scene_measurements=scene),
        instance_id="robot", body_names={"base", "pump"}, max_hz=200, camera_endpoint_available=True,
    )
    wires = []
    runtime = FormalMeasurements(planned, product_session_id="product", read_state=lambda: nullcontext((model, data)),
                                 enqueue=wires.append, model_generation=2, reset_generation=3)
    try:
        data.time = .02
        runtime.sample(sim_time_s=.02, mapped_epoch_s=NOW, physics_sequence=2)
        assert len(wires) == 1
        return measurement_from_record(wires[0][28:], received_at_s=NOW,
                                       expected_product_session_id="product", expected_session_id="source")
    finally:
        runtime.close()


@pytest.mark.parametrize("mode,kind,short_kind", [
    ("apparent", "apparent_brightness", "apparent"),
    ("surface_corrected", "emissivity_corrected_surface", "corrected"),
])
def test_formal_thermal_wire_to_inspection_uses_declared_producer_metadata(
    tmp_path, monkeypatch, mode, kind, short_kind,
):
    sample = formal_sample("thermal", temperature_mode=mode)
    assert "provenance" not in sample.metadata and "calibration" not in sample.metadata
    assert sample.metadata["frame_metadata"]["temperature_kind"] == kind
    rule = thermal_rule() | dict(roi=[0, 0, 2, 2], temperature_kind=short_kind,
                                  model_id="declared_surface_broadband_greybody", model_version="1")
    module, bridge = module_for(tmp_path, monkeypatch,
                                measurement_sensors={"thermal": binding() | {"frame_id": "robot/thermal"}},
                                measurement_rules={"thermal": rule})
    publish(module.thermal, sample)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    result = captured(tmp_path)
    assert result.analysis_verdict == "anomaly"
    raw = next(r for r in result.manifest["persistence"]["artifacts"] if r["kind"] == "measurement")
    assert raw["metadata"]["frame_metadata"] == sample.metadata["frame_metadata"]
    assert raw["metadata"]["stamp"] == sample.metadata["stamp"]
    with np.load(BytesIO((result.evidence_dir / raw["path"]).read_bytes()), allow_pickle=False) as arrays:
        np.testing.assert_array_equal(arrays["temperature_k"], sample.temperature_k)
        np.testing.assert_array_equal(arrays["valid_mask"], sample.valid_mask)


@pytest.mark.parametrize("calibrated,verdict", [(True, "anomaly"), (False, "inconclusive")])
def test_formal_acoustic_wire_to_inspection_uses_real_calibration_not_fabricated(
    tmp_path, monkeypatch, calibrated, verdict,
):
    sample = formal_sample("acoustic", calibrated=calibrated)
    assert "provenance" not in sample.metadata and "calibration" not in sample.metadata
    rule = acoustic_rule() | dict(min_duration_s=.01, rms_pa=.15, model_id="direct_path_pressure", model_version="1")
    module, bridge = module_for(tmp_path, monkeypatch, "capture:acoustic",
                                measurement_sensors={"acoustic": binding("acoustic") | {"frame_id": "robot/acoustic"}},
                                measurement_rules={"acoustic": rule})
    publish(module.acoustic, sample)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    result = captured(tmp_path)
    assert result.analysis_verdict == verdict
    raw = next(r for r in result.manifest["persistence"]["artifacts"] if r["kind"] == "measurement")
    assert raw["metadata"]["frame_metadata"] == sample.metadata["frame_metadata"]
    with np.load(BytesIO((result.evidence_dir / raw["path"]).read_bytes()), allow_pickle=False) as arrays:
        np.testing.assert_array_equal(arrays["pressure_pa"], sample.pressure_pa)
        np.testing.assert_array_equal(arrays["valid_mask"], sample.valid_mask)
        if calibrated:
            np.testing.assert_array_equal(arrays["voltage_v"], sample.voltage_v)
        else:
            assert "voltage_v" not in arrays.files
            assert result.manifest["analysis"]["reason"] == "calibration_missing_or_mismatched"


@pytest.mark.parametrize("damage,reason", [
    ("missing_provenance", "provenance_missing_or_mismatched"),
    ("unknown_version", "provenance_missing_or_mismatched"),
    ("unknown_temperature_kind", "temperature_kind_missing_or_mismatched"),
])
def test_real_producer_unknown_metadata_never_infers_a_model_or_temperature_kind(tmp_path, monkeypatch, damage, reason):
    sample = formal_sample("thermal")
    frame = sample.metadata["frame_metadata"]
    if damage == "missing_provenance":
        frame.pop("provenance", None)
    elif damage == "unknown_version":
        frame["provenance"] = dict(model_id="declared_surface_broadband_greybody", model_version="unknown",
                                    qualification="EXPERIMENTAL")
    else:
        frame["temperature_kind"] = "unknown_radiometry"
    module, bridge = module_for(
        tmp_path, monkeypatch, measurement_sensors={"thermal": binding() | {"frame_id": "robot/thermal"}},
        measurement_rules={"thermal": thermal_rule() | dict(roi=[0, 0, 2, 2],
            model_id="declared_surface_broadband_greybody", model_version="1")},
    )
    publish(module.thermal, sample)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    result = captured(tmp_path)
    assert result.analysis_verdict == "inconclusive"
    assert result.manifest["analysis"]["reason"] == reason


def test_formal_semantic_wire_to_inspection_stores_physical_labels_not_predictions(tmp_path, monkeypatch):
    sample = formal_sample("semantic")
    module, bridge = module_for(
        tmp_path, monkeypatch, "capture:overview", measurement_rules={},
        measurement_sensors={"semantic": binding("semantic") | {"frame_id": "robot/semantic"}},
        capture_sensor_ids={"capture:overview": ["semantic"]},
    )
    publish(module.semantic, sample)
    assert module.poll_once()
    assert bridge.results[0]["persisted"] is True
    result = captured(tmp_path)
    assert result.analysis_verdict == "not_evaluated"
    raw = next(r for r in result.manifest["persistence"]["artifacts"] if r["kind"] == "measurement")
    assert raw["metadata"]["simulation"] is True and raw["metadata"]["ground_truth"] is True
    with np.load(BytesIO((result.evidence_dir / raw["path"]).read_bytes()), allow_pickle=False) as arrays:
        np.testing.assert_array_equal(arrays["class_ids"], sample.class_ids)
        np.testing.assert_array_equal(arrays["instance_ids"], sample.instance_ids)
        np.testing.assert_array_equal(arrays["class_ids"], np.full((2, 2), 7, np.uint32))
