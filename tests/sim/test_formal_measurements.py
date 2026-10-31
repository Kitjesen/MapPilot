"""Opt-in measurements reuse the Product-owned engine and byte publisher."""

from contextlib import nullcontext

import pytest


def plan():
    return {
        "schema": "lingtu.sim.sensor-plan.v1",
        "session_id": "compiled-source",
        "env": "sim",
        "streams": {
            "thermal": [
                {
                    "sensor_id": "robot.thermal",
                    "instance_id": "robot",
                    "frame_id": "robot/camera",
                    "owner": "physics",
                    "source": "mujoco_measurement",
                    "transport": "typed_dds",
                    "message_type": "lingtu.dds.ThermalImage",
                    "rate_hz": 10,
                    "measurement": {
                        "body_frame_id": "robot/base",
                        "camera": {
                            "model": "pinhole",
                            "width": 2,
                            "height": 2,
                            "fx_px": 2,
                            "fy_px": 2,
                            "cx_px": 1,
                            "cy_px": 1,
                        },
                        "thermal": {"background_temperature_k": 293.15, "calibration_id": "declared-camera"},
                    },
                }
            ]
        },
        "scene_measurements": {"thermal_surfaces": {"hot": {"temperature_k": 400, "emissivity": 1}}},
    }


def test_absent_measurements_are_inert_and_bad_bindings_fail_before_engine_start():
    from sim.scripts.mujoco.formal_measurements import planned_measurements

    assert (
        planned_measurements(
            {"streams": {}}, instance_id="robot", body_names={"base"}, max_hz=200, camera_endpoint_available=False
        )
        is None
    )
    with pytest.raises(ValueError, match="camera"):
        planned_measurements(
            plan(), instance_id="robot", body_names={"base"}, max_hz=200, camera_endpoint_available=False
        )
    with pytest.raises(ValueError, match="body"):
        planned_measurements(
            plan(), instance_id="robot", body_names={"unknown"}, max_hz=200, camera_endpoint_available=True
        )


def test_acoustic_sample_grid_incompatible_with_physics_tick_fails_before_launch():
    from sim.scripts.mujoco.formal_measurements import planned_measurements

    value = plan()
    stream = value["streams"].pop("thermal")[0]
    stream.update(sensor_id="robot.acoustic", rate_hz=100, message_type="lingtu.dds.AcousticBlock")
    stream["measurement"] = {"body_frame_id": "robot/base", "microphone": {
        "sample_rate_hz": 44100, "block_samples": 441, "channel_offsets_m": [[0, 0, 0]]}}
    value["streams"] = {"acoustic": [stream]}
    with pytest.raises(ValueError, match="acoustic sample grid"):
        planned_measurements(value, instance_id="robot", body_names={"base"}, max_hz=200,
                             camera_endpoint_available=True)


def test_formal_measurements_keep_source_clock_and_send_owned_wire():
    from sim.scripts.mujoco.formal_measurements import FormalMeasurements, planned_measurements

    from runtime.endpoints.dds.measurement import measurement_from_record

    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <body name="base"><geom type="sphere" size=".01"/></body>
      <geom name="hot" type="box" pos="0 0 3" size="2 2 .1"/></worldbody></mujoco>""")
    data = mujoco.MjData(model)
    validated = planned_measurements(
        plan(), instance_id="robot", body_names={"base"}, max_hz=200, camera_endpoint_available=True
    )
    wires, recorded = [], []
    runtime = FormalMeasurements(
        validated,
        product_session_id="owned-product",
        read_state=lambda: nullcontext((model, data)),
        enqueue=wires.append,
        on_record=recorded.append,
        model_generation=4,
        reset_generation=2,
    )
    try:
        runtime.controls.set_recording("robot.thermal", True)
        runtime.sample(sim_time_s=0, mapped_epoch_s=1000.0, physics_sequence=0)
        assert len(wires) == 1 and len(recorded) == 1
        measurement = measurement_from_record(
            wires[0][28:],
            received_at_s=1000.1,
            expected_product_session_id="owned-product",
            expected_session_id="compiled-source",
        )
        assert measurement.identity.sim_time_ns == 0
        assert measurement.captured_at_s == 1000.0
        assert measurement.identity.model_generation == 4 and measurement.identity.reset_generation == 2
        assert measurement.temperature_k[0, 0] == pytest.approx(400)
        assert recorded[0].metadata["frame_metadata"]["source_clock"]["mapped_epoch_s"] == 1000.0
        runtime.controls.set_enabled("robot.thermal", False)
        data.time = 0.1
        runtime.sample(sim_time_s=0.1, mapped_epoch_s=1000.1, physics_sequence=1)
        assert len(wires) == 1
        with pytest.raises(ValueError, match="physics owner"):
            runtime.controls.set_rate_hz("robot.thermal", 201)
    finally:
        runtime.close()

    assert runtime.status()["state"] == "closed"
    assert runtime.status()["streams"]["robot.thermal"]["enabled"] is False


def test_publisher_failure_is_not_masked_as_measurement_success():
    from sim.scripts.mujoco.formal_measurements import FormalMeasurements, planned_measurements

    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody><body name="base"><geom type="sphere" size=".1"/></body>
      <geom name="hot" type="box" pos="0 0 3" size="1 1 .1"/></worldbody></mujoco>""")
    data = mujoco.MjData(model)
    validated = planned_measurements(
        plan(), instance_id="robot", body_names={"base"}, max_hz=200, camera_endpoint_available=True
    )

    def fail(_):
        raise OSError("owned publisher failed")

    runtime = FormalMeasurements(
        validated,
        product_session_id="owned-product",
        read_state=lambda: nullcontext((model, data)),
        enqueue=fail,
        model_generation=0,
        reset_generation=0,
    )
    try:
        with pytest.raises(OSError, match="owned publisher"):
            runtime.sample(sim_time_s=0, mapped_epoch_s=1000, physics_sequence=0)
    finally:
        runtime.close()
    assert runtime.status()["state"] == "failed"
    assert "owned publisher failed" in runtime.status()["error"]
    assert "robot.thermal" in runtime.status()["streams"]
