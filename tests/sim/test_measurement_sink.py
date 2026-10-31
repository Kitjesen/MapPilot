from dataclasses import replace

import numpy as np
import pytest
from sim.runtime.sensors.samples import SensorSampleStamp
from sim.runtime.sensors.thermal import ThermalConfig, ThermalSensor, ThermalSurface


def _frame():
    stamp = SensorSampleStamp("session", "robot", "robot.thermal", "camera", 0, 0, 0, 0)
    config = ThermalConfig(
        dict(model="pinhole", width=1, height=1, fx=1, fy=1, cx=0.5, cy=0.5, calibration_id="fixture"), 293
    )
    return ThermalSensor(config, {0: ThermalSurface(400, 1)}).measure(
        stamp,
        source_stamp=stamp,
        geom_ids=np.array([[0]]),
        distances_m=np.array([[1.0]]),
        valid_rays=np.array([[True]]),
    )


def test_sink_exposes_latest_and_records_only_while_enabled():
    from sim.runtime.recording.measurement_payload import decode_measurement_payload
    from sim.runtime.recording.measurement_sink import MeasurementSink

    records = []
    sink = MeasurementSink("robot.thermal", "thermal", on_record=records.append)
    frame = _frame()
    with pytest.raises(RuntimeError, match="started"):
        sink.publish(frame)
    assert sink.start()["ready"]
    sink.publish(frame)
    assert sink.latest is frame and not records
    sink.set_recording(True)
    sink.publish(frame)
    assert len(records) == 1
    restored = decode_measurement_payload(records[0], modality="thermal")
    np.testing.assert_array_equal(restored.temperature_k, frame.temperature_k)
    sink.set_recording(False)
    sink.publish(frame)
    assert len(records) == 1
    sink.close()
    assert sink.latest is None
    with pytest.raises(RuntimeError, match="closed"):
        sink.publish(frame)


def test_recording_failure_is_not_reported_as_success():
    from sim.runtime.recording.measurement_sink import MeasurementSink

    def failing(_):
        raise OSError("disk full")

    sink = MeasurementSink("robot.thermal", "thermal", on_record=failing)
    sink.start()
    sink.set_recording(True)
    with pytest.raises(OSError, match="disk full"):
        sink.publish(_frame())
    assert sink.latest is None
    sink.close()


def test_sink_rejects_wrong_sensor_and_missing_recording_owner():
    from sim.runtime.recording.measurement_sink import MeasurementSink

    sink = MeasurementSink("robot.thermal", "thermal")
    sink.start()
    with pytest.raises(ValueError, match="record"):
        sink.set_recording(True)
    frame = _frame()
    with pytest.raises(ValueError, match="sensor"):
        sink.publish(replace(frame, stamp=replace(frame.stamp, sensor_id="other")))
