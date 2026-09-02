"""Camera SHM to recording payload adapter contracts."""

# ruff: noqa: S101

from __future__ import annotations

from types import SimpleNamespace

import pytest

from sim.adapters.shm import (
    CameraShmAllocation,
    CameraShmWriter,
    FrameMetadata,
    InMemoryMappingBackend,
    StreamKind,
)
from sim.runtime.recording import (
    CameraShmPayloadSource,
    SensorPayloadCaptureError,
)


def _sensor_plan(*, encoding: str = "rgb8") -> dict[str, object]:
    return {
        "schema": "lingtu.sim.sensor-plan.v1",
        "session_id": "a" * 64,
        "streams": {
            "rgb": [
                {
                    "sensor_id": "robot_01.front_rgb",
                    "frame_id": "robot_01/front_camera",
                    "source": "unreal_camera",
                    "transport": "camera_shm",
                    "rate_hz": 30,
                    "width": 2,
                    "height": 2,
                    "encoding": encoding,
                }
            ],
            "depth": [],
        },
    }


def _snapshot() -> dict[str, object]:
    return {
        "event": "snapshot",
        "session_id": "a" * 64,
        "model_generation": 2,
        "reset_generation": 3,
        "sequence": 17,
        "sim_time_ns": 1_500_000_000,
    }


def test_camera_payload_source_copies_each_valid_shm_frame_once() -> None:
    backend = InMemoryMappingBackend()
    allocation = CameraShmAllocation("camera.rgb", slot_capacity=64)
    mapping = backend.create(allocation)
    writer = CameraShmWriter(mapping, allocation, now_ns=lambda: 2_000_000_000)
    source = CameraShmPayloadSource(
        sensor_plan=_sensor_plan(),
        allocation_provider=lambda: SimpleNamespace(
            shm={"robot_01.front_rgb": allocation.name}
        ),
        mapping_backend=backend,
        slot_capacity=64,
        max_age_s=None,
    )
    writer.publish(
        FrameMetadata(
            stream_kind=StreamKind.COLOR,
            timestamp_ns=1_500_000_000,
            width=2,
            height=2,
            stride=6,
            encoding="rgb8",
            frame_id="robot_01/front_camera",
            fx=100.0,
            fy=101.0,
            cx=1.0,
            cy=1.0,
        ),
        b"abcdefghijkl",
    )

    first = source.capture(_snapshot())
    unchanged = source.capture(_snapshot())

    assert len(first) == 1
    assert unchanged == ()
    sample = first[0]
    assert sample.sensor_id == "robot_01.front_rgb"
    assert sample.stream_kind == "rgb"
    assert sample.encoding == "rgb8"
    assert sample.media_type == "application/vnd.lingtu.rgb8"
    assert sample.sample_sequence == 1
    assert sample.sample_time_ns == 1_500_000_000
    assert sample.payload == b"abcdefghijkl"
    assert sample.metadata == {
        "transport_schema": "lingtu.camera.shm_frame.v1",
        "clock_domain": "unix_realtime",
        "frame_id": "robot_01/front_camera",
        "width": 2,
        "height": 2,
        "stride_bytes": 6,
        "fx": 100.0,
        "fy": 101.0,
        "cx": 1.0,
        "cy": 1.0,
        "depth_scale": 0.001,
        "distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
    }


def test_camera_payload_source_retries_not_ready_mappings_without_fake_data() -> None:
    source = CameraShmPayloadSource(
        sensor_plan=_sensor_plan(),
        allocation_provider=lambda: SimpleNamespace(
            shm={"robot_01.front_rgb": "camera.missing"}
        ),
        mapping_backend=InMemoryMappingBackend(),
        slot_capacity=64,
        max_age_s=None,
    )

    assert source.capture(_snapshot()) == ()


def test_camera_payload_source_rejects_a_frame_that_drifted_from_sensor_plan() -> None:
    backend = InMemoryMappingBackend()
    allocation = CameraShmAllocation("camera.rgb", slot_capacity=64)
    mapping = backend.create(allocation)
    writer = CameraShmWriter(mapping, allocation, now_ns=lambda: 2_000_000_000)
    source = CameraShmPayloadSource(
        sensor_plan=_sensor_plan(encoding="bgr8"),
        allocation_provider=lambda: SimpleNamespace(
            shm={"robot_01.front_rgb": allocation.name}
        ),
        mapping_backend=backend,
        slot_capacity=64,
        max_age_s=None,
    )
    writer.publish(
        FrameMetadata(
            stream_kind=StreamKind.COLOR,
            timestamp_ns=1_500_000_000,
            width=2,
            height=2,
            stride=6,
            encoding="rgb8",
            frame_id="robot_01/front_camera",
        ),
        b"abcdefghijkl",
    )

    with pytest.raises(SensorPayloadCaptureError, match="encoding"):
        source.capture(_snapshot())
