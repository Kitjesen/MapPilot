"""Production feeder consumes optional RunPlan measurements, default unchanged."""

import json
from contextlib import contextmanager

import pytest
from sim.scripts.mujoco import formal_feeder as feeder
from sim.scripts.mujoco.evidence import FEEDER_STATUS_FILENAME
from sim.scripts.mujoco.native_sensor_records import HEADER

from lingtu.run_plan import RunPlan
from message.generated.enums import SensorControlKind
from runtime.contracts.sensor_control import SensorControlRequest
from tests.sim.test_formal_feeder import FakeServices, _run, _run_plan
from tests.sim.test_formal_measurements import plan as measurement_plan


def _with_measurements():
    value = _run_plan(camera=True, enable_camera=True).as_dict()
    sensor_plan = value["launch"]["simulation"]["sensor_plan"]
    addition = measurement_plan()
    stream = addition["streams"]["thermal"][0]
    stream.update(instance_id="robot_01", sensor_id="robot_01.thermal", frame_id="robot_01/camera")
    stream["measurement"]["body_frame_id"] = "robot_01/base_link"
    sensor_plan["streams"]["thermal"] = [stream]
    sensor_plan["scene_measurements"] = addition["scene_measurements"]
    return RunPlan.from_dict(value)


def test_runtime_config_exposes_only_declared_measurements():
    options = feeder._parser().parse_args([])
    assert feeder._run_plan_runtime_config(options, _run_plan()).measurements is None
    config = feeder._run_plan_runtime_config(options, _with_measurements())
    assert config.measurements.sensor_plan["streams"]["thermal"][0]["sensor_id"] == "robot_01.thermal"


@pytest.mark.parametrize("late", [False, True])
def test_formal_feeder_sends_temperature_over_owned_camera_not_imu(monkeypatch, tmp_path, late):
    mujoco = pytest.importorskip("mujoco")
    services = FakeServices(stop_after_readiness=False, stop_after_waits=50, publish_real_artifacts=True,
                            overshoot_wait=1 if late else None, overshoot_factor=7.5)
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="base_link"><geom type="sphere" size=".01"/></body>
      <geom name="hot" type="box" pos="0 0 3" size="2 2 .1"/></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    observed = {}

    @contextmanager
    def state():
        data.time = services.engine.sim_time
        observed[round(data.time * 1e9)] = services.wall_time()
        yield model, data

    services.engine.measurement_state = state
    class Control:
        def __init__(self):
            self.results = []
            self.pending = None
            self.closed = False

        def take_request(self):
            value, self.pending = self.pending, None
            return value

        def write_result(self, result):
            self.results.append(result)

        def close(self):
            self.closed = True

    control = Control()
    def connect_control(**binding):
        control.pending = SensorControlRequest(
            request_id="record-on", sensor_id="robot_01.thermal", model_generation=0, reset_generation=0,
            deadline_s=services.wall_time() + 10, operation=SensorControlKind.SET_RECORDING, enabled=True,
            product_session_id=binding["product_session_id"], session_id=binding["session_id"])
        return control

    services.connect_measurement_control = connect_control
    monkeypatch.setenv("LINGTU_PROCESS_LAUNCH_ID", "a" * 64)
    rc, identity, _ = _run(monkeypatch, tmp_path, services, plan=_with_measurements())
    assert rc == 0
    records = services.sensors["camera_publisher"].records
    assert any(HEADER.unpack_from(record)[1] == 9 for record in records)
    assert all(HEADER.unpack_from(record)[1] != 9 for record in services.sensors["imu_publisher"].records)
    from runtime.endpoints.dds.measurement import measurement_from_record

    for record in records:
        if HEADER.unpack_from(record)[1] == 9:
            measurement = measurement_from_record(
                record[HEADER.size:], received_at_s=services.wall_time(),
                expected_product_session_id=identity.product_session_id,
                expected_session_id=identity.plan.as_dict()["launch"]["simulation"]["sensor_plan"]["session_id"],
            )
            assert measurement.captured_at_s == pytest.approx(observed[measurement.identity.sim_time_ns], abs=1e-6, rel=0)
    assert services.events.count("engine_close") == 1
    stored = json.loads((identity.session_root / FEEDER_STATUS_FILENAME).read_text())
    assert stored["state"] == "stopped"
    assert stored["measurements"]["state"] == "closed"
    assert stored["measurements"]["observations"][0]["sample_count"] > 0
    assert stored["measurement_transport"]["published_count"] > 0
    assert control.closed and len(control.results) == 1 and control.results[0].applied
    recording = json.loads((identity.session_root / "measurements" / "recording.json").read_text())
    assert recording["state"] == "closed" and recording["sample_count"] > 0
