# ruff: noqa: S101

from __future__ import annotations

import io
import json
import struct
from dataclasses import replace
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from runtime.msgs.numpy_compat import np
from sim.runtime.coordinator.run_allocation import RunAllocation
from sim.runtime.sensors import (
    DeadlinePolicy,
    LivoxPointSample,
    Mid360EndpointFactory,
    Mid360FrameSample,
    Mid360PatternCursor,
    Mid360PatternError,
    Mid360RaycastExtractor,
    Mid360RaycastPipeline,
    SensorRoute,
    SensorRuntime,
    SensorSampleStamp,
    SensorStreamPlan,
    mid360_frame_from_raycast,
)
from sim.runtime.sensors.dds_adapter import (
    Mid360AdapterError,
    Mid360DdsAdapter,
    encode_mid360_frame_sample,
)


def _scheduled():
    runtime = SensorRuntime(
        "a" * 64,
        (
            SensorStreamPlan(
                stream_kind="mid360",
                instance_id="thunder_01",
                sensor_id="thunder_01.mid360",
                frame_id="thunder_01/lidar_site",
                message_type="lingtu.dds.LivoxFrame",
                rate_hz=Fraction(10, 1),
                route=SensorRoute("physics", "mujoco_livox_model", "typed_dds"),
                raycast_frame_stable_id="thunder_01/lidar1_link_site",
            ),
        ),
    )
    runtime.advance(sim_time_ns=0, model_generation=3, reset_generation=0)
    return runtime.advance(sim_time_ns=100_000_000, model_generation=3, reset_generation=0).samples[0]


def _frame(sequence: int = 1, sim_time_ns: int = 100_000_000) -> Mid360FrameSample:
    return Mid360FrameSample(
        stamp=SensorSampleStamp(
            session_id="a" * 63,
            instance_id="thunder_01",
            sensor_id="thunder_01.mid360",
            frame_id="thunder_01/lidar_site",
            model_generation=3,
            reset_generation=0,
            sequence=sequence,
            sim_time_ns=sim_time_ns,
        ),
        points=(
            LivoxPointSample(
                x=1.0,
                y=0.0,
                z=-0.2,
                reflectivity=15,
                offset_time_ns=0,
                tag=0,
                line=0,
            ),
            LivoxPointSample(
                x=2.0,
                y=0.1,
                z=-0.1,
                reflectivity=15,
                offset_time_ns=5_000,
                tag=0,
                line=0,
            ),
        ),
        scan_time_profile="instantaneous_geometry/scheduled_offsets",
    )


def test_mid360_pattern_cursor_is_deterministic_and_uses_100ms_offsets() -> None:
    cursor = Mid360PatternCursor(points_per_frame=4)

    first_dirs, first_offsets = cursor.next_rays()
    second_dirs, second_offsets = cursor.next_rays()

    assert first_dirs.shape == (4, 3)
    assert first_offsets.tolist() == [0, 25_000_000, 50_000_000, 75_000_000]
    assert second_offsets.tolist() == first_offsets.tolist()
    assert cursor.cursor == 8
    assert not (first_dirs == second_dirs).all()


def test_mid360_frame_from_raycast_rejects_identity_time_and_order_mismatches() -> None:
    scheduled = _scheduled()
    raycast_frame = SimpleNamespace(
        sim_time_ns=100_000_000,
        model_generation=3,
        reset_generation=0,
        hits=[
            SimpleNamespace(
                xyz_sensor=(1.0, 2.0, 3.0),
                reflectivity=15,
                offset_time_ns=0,
                tag=0,
                line=0,
            )
        ],
    )

    sample = mid360_frame_from_raycast(
        scheduled,
        raycast_frame,
    )
    assert sample.scan_time_profile == "instantaneous_geometry/scheduled_offsets"
    assert sample.points[0].reflectivity == 15
    assert sample.points[0].tag == 0
    assert sample.points[0].line == 0

    raycast_frame.sim_time_ns = 100_000_001
    with pytest.raises(Mid360PatternError, match="time"):
        mid360_frame_from_raycast(
            scheduled,
            raycast_frame,
        )

    with pytest.raises(ValueError, match="non-decreasing"):
        Mid360FrameSample(
            stamp=sample.stamp,
            points=(
                LivoxPointSample(1.0, 0.0, 0.0, 15, 10),
                LivoxPointSample(2.0, 0.0, 0.0, 15, 9),
            ),
            scan_time_profile="instantaneous_geometry/scheduled_offsets",
        )


def test_mid360_binary_encoder_carries_ltu1_points() -> None:
    encoded = encode_mid360_frame_sample(_frame())

    assert encoded[:4] == b"LTU1"
    assert encoded[4] == 1
    timestamp_ns, sequence, count, payload_bytes = struct.unpack_from("<QIII", encoded, 8)
    assert timestamp_ns == 100_000_000
    assert sequence == 1
    assert count == 2
    assert payload_bytes == 48
    assert struct.unpack_from("<4fIBBH", encoded, 28) == pytest.approx(
        (1.0, 0.0, -0.2, 15.0, 0, 0, 0, 0)
    )

def test_mid360_adapter_active_only_after_first_accepted_non_empty_frame(tmp_path) -> None:
    class Allocation:
        dds_domain = 42
        session_id = "a" * 64
        run_dir = tmp_path
        log_dir = tmp_path / "logs"

        @staticmethod
        def child_environment():
            return {"LINGTU_HOST_BOOT_ID": "test-mid360-boot"}

    executable = tmp_path / "publisher.exe"
    executable.write_bytes(b"placeholder")
    adapter = Mid360DdsAdapter(
        executable,
        allocation=Allocation(),
        frame_id="thunder_01/lidar_site",
    )

    class Process:
        returncode = None
        stdin = bytearray()

        def poll(self):
            return self.returncode

    process = Process()
    process.stdin = SimpleNamespace(write=lambda data: None, flush=lambda: None)
    adapter._process = process
    assert adapter.active is False
    adapter.publish(_frame())
    assert adapter.active is True
    with pytest.raises(Mid360AdapterError, match="not increasing"):
        adapter.publish(_frame())


def test_mid360_adapter_writes_stderr_to_the_run_log(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Allocation:
        dds_domain = 42
        session_id = "a" * 64
        run_dir = tmp_path
        log_dir = tmp_path / "logs"

        @staticmethod
        def child_environment() -> dict[str, str]:
            return {"LINGTU_HOST_BOOT_ID": "test-mid360-boot"}

    class Process:
        pid = 4242
        returncode: int | None = None
        stdin = io.BytesIO()

        def poll(self) -> int | None:
            return self.returncode

        def wait(self, timeout: float) -> int:
            assert timeout == pytest.approx(5.0)
            self.returncode = 0
            return 0

    class ProcessOwner:
        @staticmethod
        def popen_options(*, creationflags: int = 0) -> dict[str, int]:
            return {"creationflags": creationflags}

        @staticmethod
        def attach(_process: Process) -> None:
            return None

        @staticmethod
        def close_after_exit() -> None:
            return None

    launched: list[dict[str, object]] = []
    process = Process()

    def popen(command: list[str], **kwargs: object) -> Process:
        launched.append(kwargs)
        ready_file = Path(command[command.index("--ready-file") + 1])
        ready_file.write_text(
            json.dumps(
                {
                    "schema": "lingtu.mujoco_sensor_publisher.ready.v1",
                    "ready": True,
                    "topic": "rt/sim/lidar/raw_frame",
                }
            ),
            encoding="utf-8",
        )
        return process

    monkeypatch.setattr("sim.runtime.sensors.dds_adapter.subprocess.Popen", popen)
    monkeypatch.setattr(
        "sim.runtime.sensors.dds_adapter.ProcessTreeOwner", ProcessOwner
    )
    executable = tmp_path / "publisher.exe"
    executable.write_bytes(b"placeholder")
    adapter = Mid360DdsAdapter(
        executable,
        allocation=Allocation(),
        frame_id="thunder_01/lidar_site",
    )

    adapter.start()
    stderr = launched[0]["stderr"]
    assert Path(stderr.name) == tmp_path / "logs" / "mid360-publisher.stderr.log"
    adapter.close()
    assert stderr.closed


class TinyCursor:
    def __init__(self) -> None:
        self.cursor = 0

    def next_rays(self) -> tuple[Any, Any]:
        self.cursor += 2
        return (
            np.asarray(((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)), dtype=np.float64),
            np.asarray((0, 5_000_000), dtype=np.uint32),
        )


class RecordingPublisher:
    def __init__(self) -> None:
        self.samples: list[Mid360FrameSample] = []

    def start(self) -> dict[str, object]:
        return {"ready": True}

    def publish(self, sample: Mid360FrameSample) -> None:
        self.samples.append(sample)

    def close(self) -> None:
        pass


class RecordingMujocoProcess:
    def __init__(self, state: str = "RUNNING", sim_time_ns: int = 100_000_000) -> None:
        self.state = state
        self.sim_time_ns = sim_time_ns
        self.requests: list[dict[str, object]] = []

    def raycast(self, **kwargs: object) -> dict[str, object]:
        self.requests.append(kwargs)
        return {
            "event": "raycast",
            "sensor_frame_id": kwargs["sensor_frame_id"],
            "model_generation": kwargs["model_generation"],
            "reset_generation": kwargs["reset_generation"],
            "sim_time_ns": 0 if kwargs["reset_generation"] == 1 else self.sim_time_ns,
            "hit_count": 2,
            "hits": [
                {
                    "xyz_sensor": [0.0, 0.0, -1.0],
                    "offset_time_ns": 0,
                    "reflectivity": 23,
                    "tag": 7,
                    "line": 4,
                },
                {
                    "xyz_sensor": [2.0, 0.0, 0.0],
                    "offset_time_ns": 5_000_000,
                    "reflectivity": 24,
                    "tag": 8,
                    "line": 5,
                },
            ],
        }


def _reset_scheduled():
    runtime = SensorRuntime(
        "a" * 64,
        (
            SensorStreamPlan(
                stream_kind="mid360",
                instance_id="thunder_01",
                sensor_id="thunder_01.mid360",
                frame_id="thunder_01/lidar_site",
                message_type="lingtu.dds.LivoxFrame",
                rate_hz=Fraction(10, 1),
                route=SensorRoute("physics", "mujoco_livox_model", "typed_dds"),
                raycast_frame_stable_id="thunder_01/lidar1_link_site",
            ),
        ),
    )
    return runtime.advance(
        sim_time_ns=0,
        model_generation=3,
        reset_generation=1,
    ).samples[0]


def test_mid360_raycast_pipeline_publishes_raycast_livox_frame_with_identity() -> None:
    process = RecordingMujocoProcess()
    publisher = RecordingPublisher()
    pipeline = Mid360RaycastPipeline(
        process=process,
        publisher=publisher,
        sensor_frame_id="lidar_site",
        cursor=TinyCursor(),
        physics_state=lambda: process.state,
    )

    assert pipeline.start() == {"ready": True}
    sample = pipeline.publish_scheduled(_scheduled())

    assert publisher.samples == [sample]
    assert process.requests == [
        {
            "sensor_frame_id": "lidar_site",
            "directions_sensor": ((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
            "offsets_time_ns": (0, 5_000_000),
            "session_id": "a" * 64,
            "model_generation": 3,
            "reset_generation": 0,
            "sequence": 1,
            "sim_time_ns": 100_000_000,
            "range_min_m": 0.1,
            "range_max_m": 40.0,
            "reflectivity_proxy": 15,
            "unknown_line": 0,
        }
    ]
    assert sample.stamp.session_id == "a" * 64
    assert sample.stamp.model_generation == 3
    assert sample.stamp.reset_generation == 0
    assert sample.stamp.sequence == 1
    assert sample.stamp.sim_time_ns == 100_000_000
    assert [(p.offset_time_ns, p.reflectivity, p.tag, p.line) for p in sample.points] == [
        (0, 23, 7, 4),
        (5_000_000, 24, 8, 5),
    ]


def test_mid360_raycast_pipeline_resets_cursor_and_sequence_on_reset_generation() -> None:
    cursor = TinyCursor()
    pipeline = Mid360RaycastPipeline(
        process=RecordingMujocoProcess(),
        publisher=RecordingPublisher(),
        sensor_frame_id="lidar_site",
        cursor=cursor,
    )
    pipeline.start()
    pipeline.publish_scheduled(_scheduled())
    assert cursor.cursor == 2

    sample = pipeline.publish_scheduled(_reset_scheduled())

    assert sample.stamp.reset_generation == 1
    assert sample.stamp.sequence == 0
    assert cursor.cursor == 2


def test_mid360_raycast_pipeline_rejects_unready_physics_and_bad_sequence() -> None:
    process = RecordingMujocoProcess(state="PAUSED")
    pipeline = Mid360RaycastPipeline(
        process=process,
        publisher=RecordingPublisher(),
        sensor_frame_id="lidar_site",
        cursor=TinyCursor(),
        physics_state=lambda: process.state,
    )
    pipeline.start()
    with pytest.raises(RuntimeError, match="READY/RUNNING"):
        pipeline.publish_scheduled(_scheduled())

    process.state = "RUNNING"
    pipeline.publish_scheduled(_scheduled())
    with pytest.raises(RuntimeError, match="sequence"):
        pipeline.publish_scheduled(_scheduled())

def _run_allocation(tmp_path: Path) -> RunAllocation:
    return RunAllocation(
        run_id="mid360-factory",
        run_dir=tmp_path,
        artifact_root=tmp_path.resolve(),
        log_dir=tmp_path / "logs",
        ports={},
        shm={},
        session_id="a" * 64,
        boot_id="boot",
        dds_domain=42,
    )
def test_mid360_scheduler_drops_intermediate_deadlines_before_raycast(
    tmp_path: Path,
) -> None:
    stream = SensorStreamPlan(
        stream_kind="mid360",
        instance_id="thunder_01",
        sensor_id="thunder_01.mid360",
        frame_id="thunder_01/lidar1_link",
        message_type="lingtu.dds.LivoxFrame",
        rate_hz=Fraction(10, 1),
        route=SensorRoute("physics", "mujoco_livox_model", "typed_dds"),
        raycast_frame_stable_id="thunder_01/lidar1_link_site",
    )
    runtime = SensorRuntime("a" * 64, (stream,))
    initial = runtime.advance(
        sim_time_ns=0,
        model_generation=3,
        reset_generation=0,
    ).samples[0]
    delayed = runtime.advance(
        sim_time_ns=300_000_000,
        model_generation=3,
        reset_generation=0,
    )

    assert delayed.samples[0].sequence == 3
    assert delayed.samples[0].policy is DeadlinePolicy.DROP_INTERMEDIATE
    assert [(drop.first_sequence, drop.last_sequence) for drop in delayed.drops] == [(1, 2)]

    process = RecordingMujocoProcess(sim_time_ns=0)
    endpoint = Mid360EndpointFactory(
        tmp_path / "lingtu_mid360_publisher.exe",
        process=process,
        cursor_factory=TinyCursor,
    )(stream, _run_allocation(tmp_path))
    assert endpoint is not None
    endpoint.extractor(initial, {})
    process.sim_time_ns = 300_000_000

    sample = endpoint.extractor(delayed.samples[0], {})

    assert sample.stamp.sequence == 3
    assert sample.stamp.sim_time_ns == 300_000_000
    assert len(process.requests) == 2



def test_mid360_endpoint_factory_raycast_uses_the_injected_physics_process(
    tmp_path: Path,
) -> None:
    process = RecordingMujocoProcess()
    factory = Mid360EndpointFactory(
        tmp_path / "lingtu_mid360_publisher.exe",
        process=process,
        cursor_factory=TinyCursor,
    )
    scheduled = _scheduled()
    allocation = _run_allocation(tmp_path)

    endpoint = factory(scheduled.stream, allocation)

    assert endpoint is not None
    assert isinstance(endpoint.extractor, Mid360RaycastExtractor)
    assert endpoint.extractor.process is process
    assert endpoint.extractor.sensor_frame_id == "thunder_01/lidar1_link_site"
    sample = endpoint.extractor(scheduled, {"ordinary_snapshot": "has no raycast hits"})
    assert sample.stamp.sensor_id == "thunder_01.mid360"
    assert process.requests[0]["sensor_frame_id"] == "thunder_01/lidar1_link_site"

    second = factory(scheduled.stream, allocation)
    assert second is not None
    assert isinstance(second.extractor, Mid360RaycastExtractor)
    assert second.extractor.cursor is not endpoint.extractor.cursor

    with pytest.raises(ValueError, match=r"compiled Mid360 stream.*raycast_frame_stable_id"):
        factory(
            replace(scheduled.stream, raycast_frame_stable_id=None),
            allocation,
        )


def test_mid360_endpoint_raycast_uses_physics_truth_sequence_not_sensor_sequence(
    tmp_path: Path,
) -> None:
    process = RecordingMujocoProcess()
    scheduled = _scheduled()
    endpoint = Mid360EndpointFactory(
        tmp_path / "lingtu_mid360_publisher.exe",
        process=process,
        cursor_factory=TinyCursor,
    )(scheduled.stream, _run_allocation(tmp_path))
    assert endpoint is not None

    sample = endpoint.extractor(
        scheduled,
        {
            "model_generation": 3,
            "reset_generation": 0,
            "sequence": 100,
            "sim_time_ns": 100_000_000,
        },
    )

    assert process.requests[0]["sequence"] == 100
    assert process.requests[0]["sim_time_ns"] == 100_000_000
    assert sample.stamp.sequence == 1
    assert sample.stamp.sim_time_ns == 100_000_000
