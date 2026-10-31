"""Commands affect the existing sensor runtime, not a UI-owned shadow state."""

from contextlib import nullcontext
from dataclasses import replace

import pytest
from sim.runtime.sensors.commands import SensorCommandProcessor
from sim.scripts.mujoco.formal_measurements import FormalMeasurements, planned_measurements

from message.generated.enums import SensorControlKind as Kind
from runtime.contracts.sensor_control import SensorControlRequest
from tests.sim.test_formal_measurements import plan


@pytest.fixture
def owner():
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="base"><geom type="sphere" size=".1"/></body>'
                                        '<geom name="hot" type="box" pos="0 0 3" size="2 2 .1"/></worldbody></mujoco>')
    data = mujoco.MjData(model)
    config = planned_measurements(plan(), instance_id="robot", body_names={"base"}, max_hz=200,
                                  camera_endpoint_available=True)
    records, wires = [], []
    runtime = FormalMeasurements(config, product_session_id="product", read_state=lambda: nullcontext((model, data)),
                                 enqueue=wires.append, on_record=records.append, model_generation=2, reset_generation=1)
    commands = SensorCommandProcessor(runtime.controls, product_session_id="product", session_id="compiled-source",
                                      model_generation=2, reset_generation=1)
    yield commands, runtime, data, records, wires
    runtime.close()


def request(operation=Kind.QUERY, **changes):
    return SensorControlRequest(**(dict(request_id="r1", product_session_id="product", session_id="compiled-source",
                                       sensor_id="robot.thermal", model_generation=2, reset_generation=1,
                                       deadline_s=101., operation=operation) | changes))


def test_disable_and_enable_have_effective_ack_and_change_sampling(owner):
    commands, runtime, data, _, wires = owner
    result = commands.apply(request(Kind.SET_ENABLED, enabled=False), now_s=100., sim_time_ns=0)
    assert result.applied and result.settings["enabled"] is False
    runtime.sample(sim_time_s=0., mapped_epoch_s=100., physics_sequence=0)
    assert not wires
    result = commands.apply(request(Kind.SET_ENABLED, request_id="r2", enabled=True), now_s=100., sim_time_ns=0)
    assert result.applied and result.settings["enabled"] is True
    data.time = .1
    runtime.sample(sim_time_s=.1, mapped_epoch_s=100.1, physics_sequence=1)
    assert len(wires) == 1


def test_wrong_generation_expired_and_conflicting_retry_do_not_change_settings(owner):
    commands, runtime, *_ = owner
    bad = request(Kind.SET_ENABLED, enabled=False, reset_generation=0)
    assert commands.apply(bad, now_s=100, sim_time_ns=0).reason == "generation_mismatch"
    expired = request(Kind.SET_ENABLED, enabled=False, deadline_s=99, request_id="old")
    assert commands.apply(expired, now_s=100, sim_time_ns=0).reason == "deadline_expired"
    assert runtime.controls.status("robot.thermal")["enabled"] is True
    accepted = request(Kind.SET_ENABLED, request_id="ok", enabled=False)
    first = commands.apply(accepted, now_s=100, sim_time_ns=0)
    assert commands.apply(replace(accepted, deadline_s=105), now_s=102, sim_time_ns=1) == first
    conflict = commands.apply(replace(accepted, enabled=True), now_s=100, sim_time_ns=0)
    assert not conflict.applied and conflict.reason == "request_id_conflict"
    assert runtime.controls.status("robot.thermal")["enabled"] is False


def test_effective_rate_noise_and_recording_come_from_the_actual_endpoint(owner):
    commands, runtime, data, recordings, wires = owner
    rate = commands.apply(request(Kind.SET_RATE, rate_hz=20), now_s=100, sim_time_ns=0)
    assert rate.applied and rate.settings["rate_hz"] == 20
    noise = commands.apply(request(Kind.CONFIGURE_NOISE, request_id="noise", enabled=True,
                                   noise_unit="K", noise_stddev=2., noise_seed=7), now_s=100, sim_time_ns=0)
    assert noise.applied and noise.settings["noise"] == dict(enabled=True, stddev=2., seed=7, unit="K")
    disabled = commands.apply(request(Kind.SET_NOISE_ENABLED, request_id="quiet", enabled=False),
                              now_s=100, sim_time_ns=0)
    assert disabled.applied and disabled.settings["noise"] == dict(enabled=False, stddev=2., seed=7, unit="K")
    recorded = commands.apply(request(Kind.SET_RECORDING, request_id="record", enabled=True), now_s=100, sim_time_ns=0)
    assert recorded.applied and recorded.settings["recording"] is True
    runtime.sample(sim_time_s=0, mapped_epoch_s=100, physics_sequence=0)
    data.time = .05
    runtime.sample(sim_time_s=.05, mapped_epoch_s=100.05, physics_sequence=1)
    assert len(recordings) == len(wires) == 2
    bad = commands.apply(request(Kind.CONFIGURE_NOISE, request_id="bad", enabled=True,
                                 noise_unit="Pa", noise_stddev=2), now_s=100, sim_time_ns=0)
    assert not bad.applied
    assert runtime.controls.status("robot.thermal")["noise"]["unit"] == "K"


def test_gimbal_uses_mount_limits_and_changes_actual_rays(owner):
    commands, runtime, _, _, wires = owner
    turned = commands.apply(request(Kind.SET_GIMBAL, pan_deg=180.), now_s=100, sim_time_ns=0)
    assert turned.applied
    assert turned.settings["measurement_settings"]["camera"]["gimbal"]["pan_deg"] == 180.
    runtime.sample(sim_time_s=0, mapped_epoch_s=100, physics_sequence=0)
    from runtime.endpoints.dds.measurement import measurement_from_record
    frame = measurement_from_record(wires[0][28:], received_at_s=100,
                                    expected_product_session_id="product", expected_session_id="compiled-source")
    assert not frame.valid_mask.any()
    rejected = commands.apply(request(Kind.SET_GIMBAL, request_id="outside", tilt_deg=91.), now_s=100, sim_time_ns=0)
    assert not rejected.applied
    assert runtime.controls.status("robot.thermal")["measurement_settings"]["camera"]["gimbal"]["pan_deg"] == 180.
