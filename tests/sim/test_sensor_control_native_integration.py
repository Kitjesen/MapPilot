"""HTTP -> native DDS -> actual MuJoCo sensor -> ACK, not a factory Product run."""

import json
import os
import tempfile
import threading
import time
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from sim.runtime.recording.measurement_archive import MeasurementArchive
from sim.scripts.mujoco.formal_measurements import FormalMeasurements, planned_measurements

from drivers.sim.measurement.control_bridge import NativeSensorControlBridge
from drivers.sim.measurement.control_module import SensorControlModule
from gateway.routes.sensor_control import register_sensor_control_routes
from tests.sim.test_formal_measurements import plan


@pytest.mark.skipif(os.name != "nt", reason="built Windows control bridge required")
def test_http_controls_real_sensor_through_native_dds_and_records_raw(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    root = Path(__file__).resolve().parents[2]
    library = root / "build/sensor-control-bridge-win/Release/lingtu_sensor_control_bridge.dll"
    if not library.is_file():
        pytest.skip("native sensor-control bridge not built")
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="base"><geom type="sphere" size=".1"/></body>'
                                         '<geom name="hot" type="box" pos="0 0 3" size="2 2 .1"/></worldbody></mujoco>')
    data = mujoco.MjData(model)
    declaration = planned_measurements(plan(), instance_id="robot", body_names={"base"}, max_hz=200,
                                       camera_endpoint_available=True)
    evidence_root = Path(os.environ.get("LINGTU_SENSOR_CONTROL_EVIDENCE_DIR", str(tmp_path)))
    evidence_root.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix="sensor-control-", dir=evidence_root))
    archive = MeasurementArchive(run / "recording", product_session_id="sensor-control-native",
                                 session_id="compiled-source", sensor_ids=["robot.thermal"])
    wires = []
    runtime = FormalMeasurements(declaration, product_session_id="sensor-control-native",
        read_state=lambda: nullcontext((model, data)), enqueue=wires.append, model_generation=2, reset_generation=1,
        on_record=archive.append)
    server = NativeSensorControlBridge(domain_id=6, product_session_id="sensor-control-native",
                                      session_id="compiled-source", owner_mode=True)
    client_module = SensorControlModule(product_session_id="sensor-control-native", session_id="compiled-source",
        domain_id=6, sensor_ids=["robot.thermal"], model_generation=2, reset_generation=1)
    stopping, errors = threading.Event(), []

    def source_loop():
        try:
            sequence = 0
            while not stopping.is_set():
                mujoco.mj_step(model, data)
                runtime.apply_commands(server, now_s=time.time(), sim_time_ns=round(data.time * 1e9))
                runtime.sample(sim_time_s=float(data.time), mapped_epoch_s=time.time(), physics_sequence=sequence)
                sequence += 1
                stopping.wait(.005)
        except Exception as exc:
            errors.append(exc)
            stopping.set()

    thread = threading.Thread(target=source_loop, name="native-sensor-control-acceptance")
    results = []
    passed = False
    try:
        client_module.setup()
        client_module.start()
        thread.start()
        app = FastAPI()
        register_sensor_control_routes(app, SimpleNamespace(_sensor_control_module=client_module))
        # DDS discovery is separate from an operator command; no command retry.
        time.sleep(.3)
        with TestClient(app) as http:
            for request_id, operation, params in [
                ("query", "query", {}), ("rate", "rate", {"rate_hz": 20.}),
                ("noise", "noise", {"enabled": True, "unit": "K", "stddev": .5, "seed": 7}),
                ("quiet", "noise_enabled", {"enabled": False}),
                ("record", "recording", {"enabled": True}),
                ("gimbal", "gimbal", {"pan_deg": 10., "tilt_deg": 0.}),
            ]:
                response = http.post("/api/v1/sensors/robot.thermal/control", json=dict(
                    request_id=request_id, expected_product_session_id="sensor-control-native",
                    operation=operation, params=params))
                assert response.status_code == 200, (response.text, [repr(error) for error in errors], len(wires))
                result = response.json()
                assert result["applied"] and result["request_id"] == request_id
                assert result["applied_sim_time_ns"] > 0
                results.append(result)
        deadline = time.monotonic() + 1.
        while not (archive.root / "index.jsonl").exists() and time.monotonic() < deadline and not errors:
            time.sleep(.01)
        assert (archive.root / "index.jsonl").is_file()
        assert not errors and wires
        with TestClient(app) as http:
            response = http.post("/api/v1/sensors/robot.thermal/control", json=dict(request_id="disable",
                expected_product_session_id="sensor-control-native", operation="enabled", params={"enabled": False}))
            assert response.status_code == 200, response.text
            results.append(response.json())
            assert results[-1]["settings"]["enabled"] is False
        passed = True
    finally:
        stopping.set()
        if thread.ident is not None:
            thread.join(timeout=3)
        client_module.close()
        server.close()
        runtime.close()
        archive.close()
        (run / "result.json").write_text(json.dumps(dict(passed=passed, scope="controlled_http_native_dds_mujoco_sensor",
            factory_product=False, operations=results, source_errors=[repr(error) for error in errors],
            source_thread_closed=not thread.is_alive(), recording_dir=str(archive.root)), indent=2), encoding="utf-8")
    assert not thread.is_alive()
    summary = json.loads((archive.root / "recording.json").read_text())
    assert summary["state"] == "closed" and summary["sample_count"] > 0
    assert results[-1]["settings"]["measurement_settings"]["camera"]["gimbal"]["pan_deg"] == 10.
    print(f"SENSOR_CONTROL_NATIVE_EVIDENCE={run}")
