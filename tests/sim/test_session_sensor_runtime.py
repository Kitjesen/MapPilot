
from __future__ import annotations

from typing import Any

import pytest
from sim.runtime.sensors import (
    LivoxPointSample,
    Mid360FrameSample,
    SensorPlanError,
    SensorRuntime,
    SensorSampleStamp,
    TruthOdometrySample,
)
from sim.runtime.sensors.session import (
    SensorEndpoint,
    SensorSessionError,
    SessionSensorRuntime,
)

from tests.sim.fixtures.sensor_plans import (
    thunderv4_unreal_sensor_plan,
    thunderv4_unreal_sensor_runtime,
)


def _plan() -> SensorRuntime:
    return thunderv4_unreal_sensor_runtime()


class RecordingSink:
    def __init__(self, *, fail_start: bool = False) -> None:
        self.fail_start = fail_start
        self.started = 0
        self.closed = 0
        self.samples: list[Any] = []

    def start(self) -> dict[str, Any]:
        self.started += 1
        if self.fail_start:
            raise RuntimeError("publisher unavailable")
        return {"ready": True}

    def publish(self, sample: Any) -> None:
        self.samples.append(sample)

    def close(self) -> None:
        self.closed += 1


def _snapshot(
    runtime: SensorRuntime,
    *,
    sim_time_ns: int,
    reset_generation: int = 0,
) -> dict[str, Any]:
    return {
        "event": "snapshot",
        "session_id": runtime.session_id,
        "model_generation": 0,
        "reset_generation": reset_generation,
        "sequence": sim_time_ns // 2_000_000,
        "physics_step": sim_time_ns // 2_000_000,
        "sim_time_ns": sim_time_ns,
        "bodies": [
            {
                "stable_id": "thunder_01/base_link",
                "position_m": [0.1, -0.2, 0.45],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "linear_velocity_mps": [0.3, 0.0, 0.0],
                "angular_velocity_rps": [0.0, 0.0, 0.2],
            }
        ],
    }


def test_session_sensor_runtime_only_schedules_bound_streams() -> None:
    plan = _plan()
    sink = RecordingSink()

    def factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.sensor_id != "thunder_01.truth_odom":
            return None
        return SensorEndpoint.truth_odometry(
            source_id="truth-odom-dds/test",
            sink=sink,
        )

    runtime = SessionSensorRuntime(plan=plan, allocation=object(), endpoint_factory=factory)
    result = runtime.prepare()

    assert result.prepared_sensor_ids == ("thunder_01.truth_odom",)
    assert result.active_sensor_ids == ("thunder_01.truth_odom",)
    assert runtime.unbound_sensor_ids == (
        "thunder_01.front_depth",
        "thunder_01.front_rgb",
        "thunder_01.imu",
        "thunder_01.mid360",
    )

    runtime.process_snapshot(_snapshot(plan, sim_time_ns=0))
    for sim_time_ns in range(2_000_000, 12_000_000, 2_000_000):
        runtime.process_snapshot(_snapshot(plan, sim_time_ns=sim_time_ns))

    assert [sample.stamp.sequence for sample in sink.samples] == [0, 1]
    assert [sample.stamp.sim_time_ns for sample in sink.samples] == [0, 10_000_000]
    runtime.close()
    assert sink.closed == 1


def test_reset_generation_restarts_bound_stream_sequence() -> None:
    plan = _plan()
    sink = RecordingSink()

    def factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.stream_kind != "truth_odom":
            return None
        return SensorEndpoint.truth_odometry(source_id="truth/test", sink=sink)

    runtime = SessionSensorRuntime(plan=plan, allocation=object(), endpoint_factory=factory)
    runtime.prepare()
    runtime.process_snapshot(_snapshot(plan, sim_time_ns=0))
    runtime.process_snapshot(_snapshot(plan, sim_time_ns=10_000_000))
    runtime.process_snapshot(_snapshot(plan, sim_time_ns=0, reset_generation=1))

    assert [(sample.stamp.reset_generation, sample.stamp.sequence) for sample in sink.samples] == [
        (0, 0),
        (0, 1),
        (1, 0),
    ]
    evidence = runtime.evidence_observations()[0]
    assert evidence["reset_generation"] == 1
    assert evidence["sample_count"] == 1
    assert evidence["last_sample_truth_sequence"] == 0
    assert evidence["last_sample_sim_time_ns"] == 0


def test_session_runtime_exposes_generation_bound_nonzero_publication_evidence() -> None:
    plan = _plan()
    sink = RecordingSink()

    def factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.stream_kind != "truth_odom":
            return None
        return SensorEndpoint.truth_odometry(source_id="truth/test", sink=sink)

    runtime = SessionSensorRuntime(
        plan=plan,
        allocation=object(),
        endpoint_factory=factory,
    )
    runtime.prepare()
    runtime.process_snapshot(_snapshot(plan, sim_time_ns=0))
    runtime.process_snapshot(_snapshot(plan, sim_time_ns=10_000_000))

    assert runtime.evidence_observations() == (
        {
            "sensor_id": "thunder_01.truth_odom",
            "state": "ACTIVE",
            "session_id": plan.session_id,
            "model_generation": 0,
            "reset_generation": 0,
            "owner": "physics",
            "source": "mujoco_truth",
            "transport": "typed_dds",
            "message_type": "lingtu.dds.Odometry",
            "runtime_source_id": "truth/test",
            "binding_identity": runtime.binding_identity(
                "thunder_01.truth_odom"
            ),
            "sample_count": 2,
            "last_sample_truth_sequence": 1,
            "last_sample_sim_time_ns": 10_000_000,
        },
    )


def test_session_runtime_preserves_mid360_fidelity_limitations_in_evidence() -> None:
    plan = _plan()
    sink = RecordingSink()

    def factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.stream_kind != "mid360":
            return None

        def extract(scheduled: Any, snapshot: Any) -> Mid360FrameSample:
            del snapshot
            return Mid360FrameSample(
                stamp=SensorSampleStamp.from_scheduled(scheduled),
                points=(
                    LivoxPointSample(
                        x=1.0,
                        y=0.0,
                        z=0.0,
                        reflectivity=15,
                        offset_time_ns=0,
                    ),
                ),
                scan_time_profile="instantaneous_geometry/scheduled_offsets",
            )

        return SensorEndpoint(
            source_id="mid360/test",
            sink=sink,
            extractor=extract,
        )

    runtime = SessionSensorRuntime(
        plan=plan,
        allocation=object(),
        endpoint_factory=factory,
    )
    runtime.prepare()
    runtime.process_snapshot(_snapshot(plan, sim_time_ns=0))

    observation = runtime.evidence_observations()[0]
    assert observation["sample_count"] == 1
    assert observation["fidelity"] == {
        "reflectivity_semantics": "explicit_conservative_proxy",
        "scan_time_profile": "instantaneous_geometry/scheduled_offsets",
        "unknown_line_representation": "line_0_unknown_physical_channel",
    }


def test_mid360_failure_before_first_frame_still_exposes_failed_evidence() -> None:
    plan = _plan()
    sink = RecordingSink()

    def factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.stream_kind != "mid360":
            return None

        def extract(scheduled: Any, snapshot: Any) -> Mid360FrameSample:
            del scheduled, snapshot
            raise RuntimeError("raycast unavailable")

        return SensorEndpoint(
            source_id="mid360/test",
            sink=sink,
            extractor=extract,
        )

    runtime = SessionSensorRuntime(
        plan=plan,
        allocation=object(),
        endpoint_factory=factory,
    )
    runtime.prepare()

    with pytest.raises(SensorSessionError, match="raycast unavailable"):
        runtime.process_snapshot(_snapshot(plan, sim_time_ns=0))

    assert runtime.evidence_observations() == (
        {
            "sensor_id": "thunder_01.mid360",
            "state": "FAILED",
            "session_id": plan.session_id,
            "model_generation": 0,
            "reset_generation": 0,
            "owner": "physics",
            "source": "mujoco_livox_model",
            "transport": "typed_dds",
            "message_type": "lingtu.dds.LivoxFrame",
            "runtime_source_id": "mid360/test",
            "binding_identity": runtime.binding_identity("thunder_01.mid360"),
            "sample_count": 0,
            "last_sample_truth_sequence": None,
            "last_sample_sim_time_ns": None,
            "failure_reason": "raycast unavailable",
        },
    )


def test_typed_stream_rejects_a_sample_from_another_reset_generation() -> None:
    plan = _plan()
    sink = RecordingSink()

    def factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.stream_kind != "truth_odom":
            return None

        def extract(scheduled: Any, snapshot: Any) -> TruthOdometrySample:
            del snapshot
            stamp = SensorSampleStamp.from_scheduled(scheduled)
            return TruthOdometrySample(
                stamp=SensorSampleStamp(
                    session_id=stamp.session_id,
                    instance_id=stamp.instance_id,
                    sensor_id=stamp.sensor_id,
                    frame_id=stamp.frame_id,
                    model_generation=stamp.model_generation,
                    reset_generation=stamp.reset_generation + 1,
                    sequence=stamp.sequence,
                    sim_time_ns=stamp.sim_time_ns,
                ),
                position_m=(0.0, 0.0, 0.0),
                orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
                linear_velocity_mps=(0.0, 0.0, 0.0),
                angular_velocity_rps=(0.0, 0.0, 0.0),
            )

        return SensorEndpoint(
            source_id="truth/test",
            sink=sink,
            extractor=extract,
        )

    runtime = SessionSensorRuntime(
        plan=plan,
        allocation=object(),
        endpoint_factory=factory,
    )
    runtime.prepare()

    with pytest.raises(SensorSessionError, match="sample reset_generation"):
        runtime.process_snapshot(_snapshot(plan, sim_time_ns=0))

    assert sink.samples == []


def test_prepare_failure_closes_every_started_endpoint() -> None:
    plan = _plan()
    first = RecordingSink()
    failing = RecordingSink(fail_start=True)

    def factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.sensor_id == "thunder_01.front_depth":
            return SensorEndpoint(
                source_id="depth/test",
                sink=first,
                extractor=lambda scheduled, snapshot: (scheduled, snapshot),
            )
        if stream.sensor_id == "thunder_01.front_rgb":
            return SensorEndpoint(
                source_id="rgb/test",
                sink=failing,
                extractor=lambda scheduled, snapshot: (scheduled, snapshot),
            )
        return None

    runtime = SessionSensorRuntime(plan=plan, allocation=object(), endpoint_factory=factory)
    with pytest.raises(SensorSessionError, match=r"front_rgb.*publisher unavailable"):
        runtime.prepare()

    assert first.started == 1
    assert first.closed == 1
    assert failing.started == 1
    assert failing.closed == 1


def test_snapshot_publish_failure_is_fail_closed() -> None:
    plan = _plan()

    class FailingSink(RecordingSink):
        def publish(self, sample: Any) -> None:
            del sample
            raise RuntimeError("wire closed")

    sink = FailingSink()

    def factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.stream_kind != "truth_odom":
            return None
        return SensorEndpoint.truth_odometry(source_id="truth/test", sink=sink)

    runtime = SessionSensorRuntime(plan=plan, allocation=object(), endpoint_factory=factory)
    runtime.prepare()

    with pytest.raises(SensorSessionError, match=r"truth_odom.*wire closed"):
        runtime.process_snapshot(_snapshot(plan, sim_time_ns=0))

    assert sink.closed == 1
    evidence = runtime.evidence_observations()[0]
    assert evidence["state"] == "FAILED"
    assert evidence["sample_count"] == 0
    assert "wire closed" in evidence["failure_reason"]


@pytest.mark.parametrize("failure_stage", ["extract", "publish"])
def test_stream_failure_survives_secondary_close_failure_and_closes_all_endpoints(
    failure_stage: str,
) -> None:
    plan = _plan()
    stream_error = RuntimeError(f"{failure_stage} exploded")
    close_error = RuntimeError("cleanup endpoint exploded")

    class EndpointSink(RecordingSink):
        def __init__(
            self,
            *,
            publish_error: Exception | None = None,
            close_error: Exception | None = None,
        ) -> None:
            super().__init__()
            self.publish_error = publish_error
            self.close_error = close_error

        def publish(self, sample: Any) -> None:
            if self.publish_error is not None:
                raise self.publish_error
            super().publish(sample)

        def close(self) -> None:
            self.closed += 1
            if self.close_error is not None:
                raise self.close_error

    target_stream = plan.streams[0]
    cleanup_stream = plan.streams[1]
    target_sink = EndpointSink(publish_error=(stream_error if failure_stage == "publish" else None))
    cleanup_sink = EndpointSink(close_error=close_error)

    def target_extractor(scheduled: Any, snapshot: Any) -> Any:
        if failure_stage == "extract":
            raise stream_error
        return scheduled, snapshot

    def factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.sensor_id == target_stream.sensor_id:
            return SensorEndpoint(
                source_id="target/test",
                sink=target_sink,
                extractor=target_extractor,
            )
        if stream.sensor_id == cleanup_stream.sensor_id:
            return SensorEndpoint(
                source_id="cleanup/test",
                sink=cleanup_sink,
                extractor=lambda scheduled, snapshot: (scheduled, snapshot),
            )
        return None

    runtime = SessionSensorRuntime(plan=plan, allocation=object(), endpoint_factory=factory)
    runtime.prepare()

    with pytest.raises(SensorSessionError) as raised:
        runtime.process_snapshot(_snapshot(plan, sim_time_ns=0))

    error = raised.value
    assert error.sensor_id == target_stream.sensor_id
    assert error.__cause__ is stream_error
    assert target_stream.sensor_id in str(error)
    assert f"{failure_stage} exploded" in str(error)
    assert "secondary cleanup failure" in str(error)
    assert "cleanup endpoint exploded" in str(error)
    assert target_sink.closed == 1
    assert cleanup_sink.closed == 1


@pytest.mark.parametrize("failure_stage", ["factory", "invalid", "start"])
def test_prepare_failure_survives_secondary_close_failure(
    failure_stage: str,
) -> None:
    plan = _plan()
    primary_error = RuntimeError(f"{failure_stage} exploded")
    close_error = RuntimeError("earlier endpoint cleanup exploded")

    class EarlierSink(RecordingSink):
        def close(self) -> None:
            self.closed += 1
            raise close_error

    class StartFailingSink(RecordingSink):
        def start(self) -> dict[str, Any]:
            self.started += 1
            raise primary_error

    earlier_stream = plan.streams[0]
    target_stream = plan.streams[1]
    earlier_sink = EarlierSink()
    target_sink = StartFailingSink()
    invalid_endpoint: Any = object()

    def factory(stream: Any, allocation: Any) -> SensorEndpoint | None:
        del allocation
        if stream.sensor_id == earlier_stream.sensor_id:
            return SensorEndpoint(
                source_id="earlier/test",
                sink=earlier_sink,
                extractor=lambda scheduled, snapshot: (scheduled, snapshot),
            )
        if stream.sensor_id != target_stream.sensor_id:
            return None
        if failure_stage == "factory":
            raise primary_error
        if failure_stage == "invalid":
            return invalid_endpoint
        return SensorEndpoint(
            source_id="start/test",
            sink=target_sink,
            extractor=lambda scheduled, snapshot: (scheduled, snapshot),
        )

    runtime = SessionSensorRuntime(plan=plan, allocation=object(), endpoint_factory=factory)

    with pytest.raises(SensorSessionError) as raised:
        runtime.prepare()

    error = raised.value
    assert error.sensor_id == target_stream.sensor_id
    assert target_stream.sensor_id in str(error)
    if failure_stage == "invalid":
        assert "factory returned an invalid value" in str(error)
        assert error.__cause__ is None
    else:
        assert f"{failure_stage} exploded" in str(error)
        assert error.__cause__ is primary_error
    assert "secondary cleanup failure" in str(error)
    assert "earlier endpoint cleanup exploded" in str(error)
    assert earlier_sink.closed == 1
    if failure_stage == "start":
        assert target_sink.started == 1
        assert target_sink.closed == 1
def test_sensor_runtime_parses_explicit_mid360_raycast_frame() -> None:
    document = thunderv4_unreal_sensor_plan()

    runtime = SensorRuntime.from_plan(document)

    mid360 = next(stream for stream in runtime.streams if stream.stream_kind == "mid360")
    assert mid360.raycast_frame_stable_id == "thunder_01/lidar1_link_site"


def test_sensor_runtime_requires_raycast_frame_only_for_mid360() -> None:
    document = thunderv4_unreal_sensor_plan()
    document["streams"]["mid360"][0].pop("raycast_frame_stable_id")

    with pytest.raises(
        SensorPlanError,
        match=r"mid360.*raycast_frame_stable_id.*required",
    ):
        SensorRuntime.from_plan(document)

    document = thunderv4_unreal_sensor_plan()
    document["streams"]["imu"][0]["raycast_frame_stable_id"] = (
        "thunder_01/lidar1_link_site"
    )

    with pytest.raises(
        SensorPlanError,
        match=r"imu.*must not declare raycast_frame_stable_id",
    ):
        SensorRuntime.from_plan(document)
