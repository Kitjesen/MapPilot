"""Physical K/Pa -> public capture -> stored finding -> HTTP report.

MuJoCo and the Product measurement producer are real; no Product is launched.
DDS request delivery and native task events are controlled boundary inputs, not
navigation/arrival evidence. The RGB image is a test chart, not a scene render.
"""

from contextlib import nullcontext
from dataclasses import asdict
from io import BytesIO
from types import SimpleNamespace

import numpy as np
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from sim.scripts.mujoco.formal_measurements import FormalMeasurements, planned_measurements
from sim.scripts.mujoco.native_sensor_records import HEADER, RECORD_MEASUREMENT

from gateway.routes.inspection import register_inspection_routes
from gateway.services.inspection_task_lifecycle import InspectionTaskTimeline
from perception.inspection.bridge_module import InspectionEvidenceModule
from runtime.contracts.inspection_evidence import InspectionEvidenceStore
from runtime.endpoints.dds.measurement import measurement_from_record
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import Image
from tests.perception.test_inspection_measurement_capture import NOW, Bridge, acoustic_rule, publish, thermal_rule


def physical_samples(*, temperature_k=400., pressure_pa=1., visible=True, calibrated=True):
    """Both modalities observe one physical model and the same source tick."""
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <body name="base"><geom type="sphere" size=".01"/></body>
      <body name="pump" pos="2 0 0"><geom type="sphere" size=".1"/></body>
      <geom name="target" type="box" pos="0 0 {3 if visible else -3}" size="2 2 .1"/>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    microphone = dict(sample_rate_hz=8000, block_samples=80, channel_offsets_m=[[0., 0., 0.]])
    if calibrated:
        microphone.update(sensitivity_volts_per_pa=.02, calibration_metadata={"calibration_id": "test-cal"})
    configurations = {
        "thermal": dict(
            body_frame_id="robot/base",
            camera=dict(model="pinhole", width=2, height=2, fx_px=2, fy_px=2, cx_px=1, cy_px=1),
            thermal=dict(background_temperature_k=293.15, calibration_id="test-cal"),
        ),
        "acoustic": dict(body_frame_id="robot/base", microphone=microphone),
    }
    streams = {
        modality: [dict(
            sensor_id=modality, instance_id="robot", frame_id=f"robot/{modality}", owner="physics",
            source="mujoco_measurement", transport="typed_dds", rate_hz=100 if modality == "acoustic" else 10,
            message_type="lingtu.dds.AcousticBlock" if modality == "acoustic" else "lingtu.dds.ThermalImage",
            measurement=configuration,
        )] for modality, configuration in configurations.items()
    }
    planned = planned_measurements(
        dict(schema="lingtu.sim.sensor-plan.v1", session_id="source", env="sim", streams=streams,
             scene_measurements=dict(
                 thermal_surfaces={"target": dict(temperature_k=temperature_k, emissivity=1.)},
                 acoustic_sources=[dict(
                     source_id="pump-signal", body_name="robot/pump", pressure_pa=[pressure_pa] * 8000,
                     sample_rate_hz=8000, start_sim_time_ns=0, reference_distance_m=1.,
                     local_offset_m=[0., 0., 0.],
                 )],
             )),
        instance_id="robot", body_names={"base", "pump"}, max_hz=200, camera_endpoint_available=True,
    )
    wires = []
    runtime = FormalMeasurements(
        planned, product_session_id="product", read_state=lambda: nullcontext((model, data)),
        enqueue=wires.append, model_generation=2, reset_generation=3,
    )
    try:
        data.time = .02
        runtime.sample(sim_time_s=.02, mapped_epoch_s=NOW, physics_sequence=2)
    finally:
        runtime.close()
    assert len(wires) == 1  # The existing publisher batches both records per tick.
    wire, cursor, samples = wires[0], 0, []
    while cursor < len(wire):
        magic, kind, _, _, _, length = HEADER.unpack_from(wire, cursor)
        assert magic == b"LTU1" and kind == RECORD_MEASUREMENT
        cursor += HEADER.size
        samples.append(measurement_from_record(
            wire[cursor:cursor + length], received_at_s=NOW,
            expected_product_session_id="product", expected_session_id="source",
        ))
        cursor += length
    assert cursor == len(wire) and len(samples) == 2
    return {sample.identity.sensor_id: sample for sample in samples}


def native_event(sequence, *, action, kind=2, state=11, evidence_id="", point_index=0):
    """Transport fixture only; the capture worker never produces these facts."""
    return dict(
        ts=NOW, frame_id="map", boot_id="navd-test", event_sequence=sequence, kind=kind,
        task_id="task", request_id="capture" if evidence_id else "start", state=state,
        map_id="map", map_content_epoch=1, route_id="route", route_revision=1,
        point_index=point_index, point_count=1, loop_index=0, retry_count=0,
        action=action, action_request_id="capture" if action else "", evidence_id=evidence_id,
        reason="point_action_succeeded" if evidence_id else "route_complete" if state == 7 else "point_action_pending",
    )


@pytest.mark.parametrize("modality,verdict,physical", [
    ("thermal", "normal", dict(temperature_k=300.)),
    ("thermal", "anomaly", dict(temperature_k=400.)),
    ("thermal", "inconclusive", dict(visible=False)),
    ("acoustic", "normal", dict(pressure_pa=.1)),
    ("acoustic", "anomaly", dict(pressure_pa=1.)),
    ("acoustic", "inconclusive", dict(calibrated=False)),
])
def test_physical_measurements_capture_to_reloaded_public_report(tmp_path, monkeypatch, modality, verdict, physical):
    import cv2

    monkeypatch.setattr("perception.inspection.bridge_module.time.time", lambda: NOW)
    root = tmp_path / "evidence"
    monkeypatch.setenv("LINGTU_INSPECTION_EVIDENCE_DIR", str(root))
    samples = physical_samples(**physical)
    # These expectations follow the physical scene, not the alert thresholds.
    np.testing.assert_allclose(samples["acoustic"].pressure_pa, physical.get("pressure_pa", 1.) / 2)
    if physical.get("visible", True):
        assert samples["thermal"].valid_mask.all()
        np.testing.assert_allclose(samples["thermal"].temperature_k, physical.get("temperature_k", 400.))
    else:
        assert not samples["thermal"].valid_mask.any()
        assert np.isnan(samples["thermal"].temperature_k).all()
    bindings = {}
    for sensor_id, sample in samples.items():
        binding = asdict(sample.identity)
        for key in ("product_session_id", "sensor_id", "sequence", "sim_time_ns"):
            del binding[key]
        bindings[sensor_id] = dict(modality=sensor_id, **binding)
    action = f"capture:{modality}"
    bridge = Bridge(action)
    worker = InspectionEvidenceModule(
        evidence_root=root, status_file=tmp_path / "status.json", bridge_factory=lambda: bridge,
        start_thread=False, product_session_id="product", measurement_sensors=bindings,
        measurement_rules={
            "thermal": thermal_rule() | dict(roi=[0, 0, 2, 2], model_id="declared_surface_broadband_greybody", model_version="1"),
            "acoustic": acoustic_rule() | dict(rms_pa=.15, min_duration_s=.01, model_id="direct_path_pressure", model_version="1"),
        },
        capture_sensor_ids={action: ["thermal", "acoustic"]},
    )
    journal = tmp_path / "task-journal.json"
    timeline = InspectionTaskTimeline(journal_path=journal)
    timeline.record_submission(
        task_id="task", action="start", request_id="start", route_id="route", map_id="map",
        map_content_epoch=1, route_revision=1, product_session_id="product",
        route_snapshot=dict(id="route", map_id="map", map_content_epoch=1, revision=1,
                            points=[dict(id="pump", action=action)]),
    )
    assert timeline.observe(native_event(1, action=action))
    before = timeline.query("task")
    worker.setup()
    try:
        # The default JPEG encoder must produce an actual decodable artifact.
        publish(worker.color_image, Image(np.full((8, 8, 3), 127, np.uint8), ts=NOW))
        publish(worker.odometry, Odometry(ts=NOW))
        for sensor_id, sample in samples.items():
            publish(getattr(worker, sensor_id), sample)
        assert worker.poll_once()
        assert bridge.results[0]["persisted"] is True
        assert bridge.results[0]["analysis_verdict"] == verdict
        # Persistence/analysis is not a task transition or a native arrival.
        assert timeline.query("task") == before
    finally:
        worker.stop()
    saved = InspectionEvidenceStore(root).get("capture")
    assert saved.analysis_verdict == verdict
    assert saved.analysis_support == "measurement_observation"
    assert saved.request.task_id == "task" and saved.request.point_index == 0
    observation = saved.manifest["analysis"]["observation"]
    assert observation["details"]["provenance"]["qualification"] == "EXPERIMENTAL"
    assert observation["details"]["rule_details"]["calibration_id"] == "test-cal"
    if modality == "thermal":
        assert observation["details"]["calibration"]["calibration_id"] == "test-cal"
        assert samples["thermal"].metadata["frame_metadata"]["calibrated_lwir"] is False
    assert timeline.observe(native_event(2, action=action, kind=5, evidence_id="capture"))
    assert timeline.observe(native_event(3, action="", state=7, point_index=1))
    # Reload both durable stores; HTTP cannot rely on worker-local objects.
    gateway = SimpleNamespace(_inspection_task_timeline=InspectionTaskTimeline(journal_path=journal))
    app = FastAPI()
    register_inspection_routes(app, gateway)
    with TestClient(app) as client:
        detail = client.get("/api/v1/inspection/evidence/capture")
        assert detail.status_code == 200
        assert detail.json()["evidence"]["analysis"]["verdict"] == verdict
        rgb = client.get("/api/v1/inspection/evidence/capture/artifacts/rgb")
        assert rgb.status_code == 200
        image = cv2.imdecode(np.frombuffer(rgb.content, np.uint8), cv2.IMREAD_COLOR)
        assert image.shape == (8, 8, 3)
        for record in detail.json()["evidence"]["artifacts"]:
            if record["kind"] != "measurement":
                continue
            response = client.get(record["download_url"])
            assert response.status_code == 200
            source = samples[record["sensor_id"]]
            assert record["metadata"]["product_session_id"] == "product"
            assert record["metadata"]["stamp"] == source.metadata["stamp"]
            with np.load(BytesIO(response.content), allow_pickle=False) as arrays:
                for name in ("valid_mask", "temperature_k" if record["modality"] == "thermal" else "pressure_pa"):
                    np.testing.assert_array_equal(arrays[name], getattr(source, name))
                has_voltage = record["modality"] == "acoustic" and physical.get("calibrated", True)
                assert ("voltage_v" in arrays.files) == has_voltage
                if has_voltage:
                    np.testing.assert_array_equal(arrays["voltage_v"], source.voltage_v)
        response = client.get("/api/v1/inspection/tasks/task/report")
        assert response.status_code == 200
        report = response.json()
        assert report["execution"]["state"] == "SUCCESS"
        assert report["coverage"]["verified_evidence"] == 1
        assert report["report_status"] == "COMPLETE"
        assert report["points"][0]["evidence_id"] == "capture"
        assert report["points"][0]["inspection_result"] == verdict.upper()
        assert report["inspection_result"]["status"] == verdict.upper()
