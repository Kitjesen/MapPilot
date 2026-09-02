# ruff: noqa: S101
from __future__ import annotations

from fractions import Fraction

import pytest

from sim.runtime.sensors.contracts import SensorRoute, SensorStreamPlan
from sim.runtime.sensors.extractors import SensorSampleError, imu_from_snapshot
from sim.runtime.sensors.runtime import SensorRuntime

pytestmark = [pytest.mark.sim]


def _stream() -> SensorStreamPlan:
    return SensorStreamPlan(
        stream_kind="imu",
        instance_id="thunder_01",
        sensor_id="thunder_01.imu",
        frame_id="thunder_01/imu",
        message_type="lingtu.dds.Imu",
        rate_hz=Fraction(200, 1),
        route=SensorRoute("physics", "mujoco_sensor", "typed_dds"),
    )


def _snapshot(*, sim_time_ns: int, reset_generation: int = 0) -> dict[str, object]:
    return {
        "session_id": "a" * 64,
        "model_generation": 4,
        "reset_generation": reset_generation,
        "sim_time_ns": sim_time_ns,
        # Body values intentionally disagree: the extractor must not use them.
        "bodies": [{"stable_id": "thunder_01/imu", "angular_velocity_rps": [99, 99, 99]}],
        "sensors": [
            {
                "sensor_id": "imu.orientation",
                "name": "imu_orientation",
                "stable_id": "thunder_01/imu/orientation",
                "instance_id": "thunder_01",
                "frame_id": "thunder_01/imu",
                "sensor_type": "framequat",
                "source_stable_id": "thunder_01/imu",
                "values": [0.9, 0.1, 0.2, 0.3],
            },
            {
                "sensor_id": "imu.gyro",
                "name": "imu_gyro",
                "stable_id": "thunder_01/imu/gyro",
                "instance_id": "thunder_01",
                "frame_id": "thunder_01/imu",
                "sensor_type": "angular-velocity",
                "source_stable_id": "thunder_01/imu",
                "values": [0.01, 0.02, 0.03],
            },
            {
                "sensor_id": "imu.accel",
                "name": "imu_accel",
                "stable_id": "thunder_01/imu/accel",
                "instance_id": "thunder_01",
                "frame_id": "thunder_01/imu",
                "sensor_type": "linear-acceleration",
                "source_stable_id": "thunder_01/imu",
                "values": [1.0, 2.0, 9.81],
            },
        ],
    }


def test_imu_extractor_requires_exact_five_millisecond_deadlines() -> None:
    runtime = SensorRuntime("a" * 64, (_stream(),))
    runtime.advance(sim_time_ns=0, model_generation=4, reset_generation=0)
    assert runtime.advance(sim_time_ns=4_999_999, model_generation=4, reset_generation=0).samples == ()
    scheduled = runtime.advance(sim_time_ns=5_000_000, model_generation=4, reset_generation=0).samples
    assert len(scheduled) == 1 and scheduled[0].sequence == 1
    sample = imu_from_snapshot(scheduled[0], _snapshot(sim_time_ns=5_000_000))
    assert sample.stamp.sim_time_ns == 5_000_000
    assert sample.orientation_wxyz == pytest.approx((0.9, 0.1, 0.2, 0.3))
    assert sample.angular_velocity_rps == pytest.approx((0.01, 0.02, 0.03))
    assert sample.linear_acceleration_mps2 == pytest.approx((1.0, 2.0, 9.81))


def test_imu_extractor_reset_restarts_generation_and_sequence() -> None:
    runtime = SensorRuntime("a" * 64, (_stream(),))
    runtime.advance(sim_time_ns=0, model_generation=4, reset_generation=0)
    first = runtime.advance(sim_time_ns=5_000_000, model_generation=4, reset_generation=0).samples[-1]
    runtime.advance(sim_time_ns=10_000_000, model_generation=4, reset_generation=0)
    reset = runtime.advance(sim_time_ns=5_000_000, model_generation=4, reset_generation=1).samples[0]
    assert (first.sequence, first.reset_generation) == (1, 0)
    assert (reset.sequence, reset.reset_generation, reset.deadline_ns) == (0, 1, 0)
    sample = imu_from_snapshot(reset, _snapshot(sim_time_ns=0, reset_generation=1))
    assert sample.stamp.reset_generation == 1


@pytest.mark.parametrize(
    "mutate",
    [
        lambda snapshot: snapshot.update(sim_time_ns=5_000_001),
        lambda snapshot: snapshot["sensors"].__getitem__(0).update(source_stable_id="wrong"),
        lambda snapshot: snapshot["sensors"].__getitem__(1).update(values=[1, 2]),
    ],
)
def test_imu_extractor_fails_closed_for_wrong_deadline_or_sensor_identity(mutate) -> None:
    runtime = SensorRuntime("a" * 64, (_stream(),))
    scheduled = runtime.advance(sim_time_ns=5_000_000, model_generation=4, reset_generation=0).samples[0]
    snapshot = _snapshot(sim_time_ns=5_000_000)
    mutate(snapshot)
    with pytest.raises(SensorSampleError):
        imu_from_snapshot(scheduled, snapshot)
