from __future__ import annotations

import copy
from functools import cache
from fractions import Fraction
from pathlib import Path
from typing import Any

import pytest

from sim.catalog import CatalogResolver
from sim.runtime.sensors import (
    DeadlinePolicy,
    ImuSample,
    SensorClockError,
    SensorRoute,
    SensorRuntime,
    SensorSampleError,
    SensorSampleStamp,
    TruthOdometrySample,
    truth_odometry_from_snapshot,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION_SPEC = (
    Path(__file__).resolve().parents[2]
    / "sim"
    / "scenarios"
    / "catalog"
    / "thunderv4_unreal"
    / "session.yaml"
)


@cache
def _canonical_sensor_plan() -> dict[str, Any]:
    return CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION_SPEC).sensor_plan


def _sensor_plan() -> dict[str, Any]:
    return copy.deepcopy(_canonical_sensor_plan())


def _runtime() -> SensorRuntime:
    return SensorRuntime.from_plan(_sensor_plan())


def test_sensor_runtime_loads_plan_rates_and_routes() -> None:
    runtime = _runtime()
    streams = {stream.sensor_id: stream for stream in runtime.streams}

    assert streams["thunder_01.imu"].rate_hz == Fraction(200)
    assert streams["thunder_01.truth_odom"].rate_hz == Fraction(100)
    assert streams["thunder_01.front_rgb"].rate_hz == Fraction(30)
    assert streams["thunder_01.front_depth"].rate_hz == Fraction(30)
    assert streams["thunder_01.mid360"].rate_hz == Fraction(10)
    assert streams["thunder_01.imu"].route == SensorRoute(
        owner="physics",
        source="mujoco_sensor",
        transport="typed_dds",
    )
    assert streams["thunder_01.front_rgb"].route == SensorRoute(
        owner="visual",
        source="unreal_camera",
        transport="camera_shm",
    )


def test_multirate_schedule_catches_up_or_drops_explicitly() -> None:
    runtime = _runtime()

    initial = runtime.advance(sim_time_ns=0, reset_generation=7)
    assert {sample.sensor_id for sample in initial.samples} == {
        "thunder_01.front_depth",
        "thunder_01.front_rgb",
        "thunder_01.imu",
        "thunder_01.mid360",
        "thunder_01.truth_odom",
    }
    assert all(sample.sequence == 0 for sample in initial.samples)
    assert all(sample.model_generation == 0 for sample in initial.samples)
    assert all(sample.deadline_ns == 0 for sample in initial.samples)

    batch = runtime.advance(sim_time_ns=100_000_000, reset_generation=7)
    imu = batch.for_sensor("thunder_01.imu")
    truth_odom = batch.for_sensor("thunder_01.truth_odom")
    mid360 = batch.for_sensor("thunder_01.mid360")
    rgb = batch.for_sensor("thunder_01.front_rgb")
    depth = batch.for_sensor("thunder_01.front_depth")

    assert [sample.sequence for sample in imu] == list(range(1, 21))
    assert imu[-1].deadline_ns == 100_000_000
    assert [sample.sequence for sample in truth_odom] == list(range(1, 11))
    assert truth_odom[-1].deadline_ns == 100_000_000
    assert [sample.sequence for sample in mid360] == [1]
    assert [sample.sequence for sample in rgb] == [3]
    assert [sample.sequence for sample in depth] == [3]
    assert rgb[0].policy is DeadlinePolicy.DROP_INTERMEDIATE
    assert imu[0].policy is DeadlinePolicy.CATCH_UP
    assert {
        (drop.sensor_id, drop.first_sequence, drop.last_sequence, drop.count)
        for drop in batch.drops
    } == {
        ("thunder_01.front_depth", 1, 2, 2),
        ("thunder_01.front_rgb", 1, 2, 2),
    }


def test_schedule_batch_groups_requests_by_exact_plan_route() -> None:
    runtime = _runtime()
    batch = runtime.advance(sim_time_ns=0, reset_generation=0)

    render_route = SensorRoute(
        owner="visual",
        source="unreal_camera",
        transport="camera_shm",
    )
    imu_route = SensorRoute(
        owner="physics",
        source="mujoco_sensor",
        transport="typed_dds",
    )

    assert {sample.sensor_id for sample in batch.for_route(render_route)} == {
        "thunder_01.front_depth",
        "thunder_01.front_rgb",
    }
    assert [sample.sensor_id for sample in batch.for_route(imu_route)] == [
        "thunder_01.imu"
    ]
    assert set(batch.routes) == {sample.route for sample in batch.samples}


def test_plan_rate_drives_drift_free_integer_nanosecond_deadlines() -> None:
    plan = _sensor_plan()
    streams = plan["streams"]
    assert isinstance(streams, dict)
    imu = streams["imu"]
    assert isinstance(imu, list)
    imu[0]["rate_hz"] = 3
    for stream_kind in ("rgb", "depth", "mid360", "truth_odom"):
        streams[stream_kind] = []

    runtime = SensorRuntime.from_plan(plan)
    runtime.advance(sim_time_ns=0, reset_generation=0)
    samples = runtime.advance(
        sim_time_ns=10_000_000_000,
        reset_generation=0,
    ).samples

    assert [sample.sequence for sample in samples] == list(range(1, 31))
    assert [sample.deadline_ns for sample in samples[:3]] == [
        333_333_334,
        666_666_667,
        1_000_000_000,
    ]
    assert samples[-1].deadline_ns == 10_000_000_000
    assert all(isinstance(sample.deadline_ns, int) for sample in samples)


def test_reset_generation_restarts_sequences_and_rejects_stale_clock() -> None:
    runtime = _runtime()
    runtime.advance(sim_time_ns=0, reset_generation=4)
    before_reset = runtime.advance(sim_time_ns=10_000_000, reset_generation=4)
    assert before_reset.for_sensor("thunder_01.imu")[-1].sequence == 2

    after_reset = runtime.advance(sim_time_ns=0, reset_generation=5)
    assert after_reset.generation_changed is True
    assert all(sample.reset_generation == 5 for sample in after_reset.samples)
    assert all(sample.sequence == 0 for sample in after_reset.samples)

    repeated = runtime.advance(sim_time_ns=0, reset_generation=5)
    assert repeated.generation_changed is False
    assert repeated.samples == ()
    assert repeated.drops == ()

    with pytest.raises(SensorClockError, match="reset_generation"):
        runtime.advance(sim_time_ns=0, reset_generation=4)

    runtime.advance(sim_time_ns=20_000_000, reset_generation=5)
    with pytest.raises(SensorClockError, match="sim_time_ns"):
        runtime.advance(sim_time_ns=19_999_999, reset_generation=5)


def test_model_generation_restarts_schedule_and_rejects_old_model() -> None:
    runtime = _runtime()
    runtime.advance(sim_time_ns=20_000_000, model_generation=3, reset_generation=8)

    rebound = runtime.advance(
        sim_time_ns=0,
        model_generation=4,
        reset_generation=0,
    )
    assert rebound.generation_changed is True
    assert rebound.model_generation == 4
    assert all(sample.model_generation == 4 for sample in rebound.samples)
    assert all(sample.reset_generation == 0 for sample in rebound.samples)
    assert all(sample.sequence == 0 for sample in rebound.samples)

    with pytest.raises(SensorClockError, match="model_generation"):
        runtime.advance(
            sim_time_ns=0,
            model_generation=3,
            reset_generation=9,
        )


def test_truth_odometry_is_extracted_from_exact_generation_stamped_truth() -> None:
    runtime = _runtime()
    scheduled = runtime.advance(
        sim_time_ns=10_000_000,
        model_generation=6,
        reset_generation=2,
    ).for_sensor("thunder_01.truth_odom")[-1]
    snapshot = {
        "session_id": runtime.session_id,
        "model_generation": 6,
        "reset_generation": 2,
        "sim_time_ns": 10_000_000,
        "bodies": [
            {
                "stable_id": "thunder_01/base_link",
                "position_m": [1.0, 2.0, 0.6],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "linear_velocity_mps": [0.4, 0.0, -0.01],
                "angular_velocity_rps": [0.0, 0.0, 0.2],
            }
        ],
    }

    sample = truth_odometry_from_snapshot(scheduled, snapshot)
    assert sample.stamp.model_generation == 6
    assert sample.stamp.reset_generation == 2
    assert sample.stamp.sequence == 1
    assert sample.position_m == (1.0, 2.0, 0.6)
    assert sample.linear_velocity_mps == (0.4, 0.0, -0.01)
    assert sample.angular_velocity_rps == (0.0, 0.0, 0.2)

    snapshot["sim_time_ns"] = 12_000_000
    with pytest.raises(SensorSampleError, match="exact scheduled deadline"):
        truth_odometry_from_snapshot(scheduled, snapshot)


def test_typed_physics_samples_preserve_unavailable_values() -> None:
    runtime = _runtime()
    scheduled = runtime.advance(sim_time_ns=0, reset_generation=9)
    imu_stamp = SensorSampleStamp.from_scheduled(
        scheduled.for_sensor("thunder_01.imu")[0]
    )
    odom_stamp = SensorSampleStamp.from_scheduled(
        scheduled.for_sensor("thunder_01.truth_odom")[0]
    )

    imu = ImuSample(
        stamp=imu_stamp,
        orientation_wxyz=None,
        angular_velocity_rps=(0.1, -0.2, 0.3),
        linear_acceleration_mps2=None,
    )
    odom = TruthOdometrySample(
        stamp=odom_stamp,
        position_m=(1.0, 2.0, 0.4),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        linear_velocity_mps=None,
        angular_velocity_rps=(0.0, 0.0, 0.2),
    )

    assert imu.orientation_wxyz is None
    assert imu.linear_acceleration_mps2 is None
    assert imu.orientation_covariance is None
    assert imu.angular_velocity_covariance is None
    assert imu.linear_acceleration_covariance is None
    assert odom.linear_velocity_mps is None
    assert odom.pose_covariance is None
    assert odom.twist_covariance is None
    assert imu.stamp.reset_generation == 9
    assert imu.stamp.model_generation == 0
    assert imu.stamp.sequence == 0
    assert imu.stamp.sim_time_ns == 0

    with pytest.raises(ValueError, match="finite"):
        ImuSample(
            stamp=imu_stamp,
            orientation_wxyz=None,
            angular_velocity_rps=(float("nan"), 0.0, 0.0),
            linear_acceleration_mps2=None,
        )


def test_deadline_policy_override_makes_initial_catch_up_drop_observable() -> None:
    plan = _sensor_plan()
    streams = plan["streams"]
    assert isinstance(streams, dict)
    imu = streams["imu"]
    assert isinstance(imu, list)
    imu[0]["rate_hz"] = 4
    for stream_kind in ("rgb", "depth", "mid360", "truth_odom"):
        streams[stream_kind] = []

    runtime = SensorRuntime.from_plan(
        plan,
        policy_by_transport={"typed_dds": DeadlinePolicy.DROP_INTERMEDIATE},
    )
    batch = runtime.advance(sim_time_ns=1_000_000_000, reset_generation=0)

    assert [sample.sequence for sample in batch.samples] == [4]
    assert batch.samples[0].policy is DeadlinePolicy.DROP_INTERMEDIATE
    assert [
        (drop.first_sequence, drop.last_sequence, drop.count)
        for drop in batch.drops
    ] == [(0, 3, 4)]


@pytest.mark.parametrize(
    "rate_hz",
    [True, 0, -1, float("nan"), float("inf"), 1_000_000_001],
)
def test_sensor_plan_rejects_invalid_runtime_rates(rate_hz: object) -> None:
    plan = _sensor_plan()
    streams = plan["streams"]
    assert isinstance(streams, dict)
    imu = streams["imu"]
    assert isinstance(imu, list)
    imu[0]["rate_hz"] = rate_hz

    with pytest.raises(ValueError, match="rate_hz"):
        SensorRuntime.from_plan(plan)


def test_unknown_transport_requires_an_explicit_deadline_policy() -> None:
    plan = _sensor_plan()
    streams = plan["streams"]
    assert isinstance(streams, dict)
    imu = streams["imu"]
    assert isinstance(imu, list)
    imu[0]["transport"] = "in_process_test"
    for stream_kind in ("rgb", "depth", "mid360", "truth_odom"):
        streams[stream_kind] = []

    with pytest.raises(ValueError, match="no deadline policy"):
        SensorRuntime.from_plan(plan)

    runtime = SensorRuntime.from_plan(
        plan,
        policy_by_transport={"in_process_test": DeadlinePolicy.CATCH_UP},
    )
    scheduled = runtime.advance(sim_time_ns=0, reset_generation=0).samples
    assert scheduled[0].route.transport == "in_process_test"
