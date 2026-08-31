from __future__ import annotations

# ruff: noqa: D103,S101
import os
import stat
import struct
import threading
import time
import zlib
from pathlib import Path

import pytest

from drivers.real.camera.shm import (
    FrameChanged,
    FrameCorrupt,
    FrameNotReady,
    FrameStale,
    ShmFrameReader,
    ShmFrameWriter,
    StreamKind,
)

_SUPERBLOCK = struct.Struct("<8sHHHHIIIIQQQQ")
_SLOT = struct.Struct("<QQQIIIIIIIIHHHH16s64s10d24sQ")


def write_committed_frame(
    path,
    *,
    stream_kind: StreamKind = StreamKind.COLOR,
    encoding: str = "rgb8",
    width: int = 2,
    height: int = 2,
    stride: int = 6,
    payload: bytes = bytes(range(12)),
    sequence: int = 7,
    timestamp_ns: int | None = None,
    guard_begin: int | None = None,
    guard_end: int | None = None,
    payload_crc32: int | None = None,
    calibration: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0),
) -> bytes:
    slot_capacity = 128
    slot_count = 2
    active_slot = 1
    timestamp_ns = timestamp_ns or time.time_ns()
    total_size = _SUPERBLOCK.size + slot_count * (_SLOT.size + slot_capacity)
    image = bytearray(total_size)
    _SUPERBLOCK.pack_into(
        image,
        0,
        b"LTCSHM01",
        1,
        _SUPERBLOCK.size,
        _SLOT.size,
        slot_count,
        slot_capacity,
        active_slot,
        0,
        0,
        sequence,
        timestamp_ns,
        timestamp_ns,
        0,
    )
    token = sequence * 2
    slot_offset = _SUPERBLOCK.size + active_slot * (_SLOT.size + slot_capacity)
    _SLOT.pack_into(
        image,
        slot_offset,
        token if guard_begin is None else guard_begin,
        sequence,
        timestamp_ns,
        width,
        height,
        stride,
        len(payload),
        slot_capacity,
        zlib.crc32(payload) & 0xFFFFFFFF if payload_crc32 is None else payload_crc32,
        0,
        0,
        int(stream_kind),
        len(encoding),
        1,
        _SLOT.size,
        (encoding.encode("ascii") + b"\0").ljust(16, b"\0"),
        b"camera_link\0".ljust(64, b"\0"),
        *calibration,
        bytes(24),
        token if guard_end is None else guard_end,
    )
    payload_offset = slot_offset + _SLOT.size
    image[payload_offset : payload_offset + len(payload)] = payload
    path.write_bytes(image)
    return payload


def test_shm_reader_returns_a_committed_frame_once(tmp_path):
    path = tmp_path / "camera_color.shm"
    expected = write_committed_frame(path)
    reader = ShmFrameReader(path, max_age_s=1.0)

    frame = reader.read_latest()

    assert frame is not None
    assert frame.schema_version == 1
    assert frame.sequence == 7
    assert frame.stream_kind is StreamKind.COLOR
    assert frame.width == 2
    assert frame.height == 2
    assert frame.stride == 6
    assert frame.encoding == "rgb8"
    assert frame.frame_id == "camera_link"
    assert frame.payload == expected
    assert reader.read_latest() is None


def test_shm_reader_rejects_stale_frames(tmp_path):
    path = tmp_path / "camera_color.shm"
    now_ns = time.time_ns()
    write_committed_frame(path, timestamp_ns=now_ns - 2_000_000_000)

    with pytest.raises(FrameStale, match="stale threshold"):
        ShmFrameReader(path, max_age_s=0.5).read_latest(now_ns=now_ns)


def test_shm_reader_rejects_a_slot_while_writer_guard_is_odd(tmp_path):
    path = tmp_path / "camera_color.shm"
    write_committed_frame(path, guard_begin=15, guard_end=0)

    with pytest.raises(FrameChanged, match="being written"):
        ShmFrameReader(path, max_age_s=None, consistency_attempts=2).read_latest()


def test_shm_reader_rejects_payload_crc_corruption(tmp_path):
    path = tmp_path / "camera_color.shm"
    write_committed_frame(path, payload_crc32=123)

    with pytest.raises(FrameCorrupt, match="CRC mismatch"):
        ShmFrameReader(path, max_age_s=None).read_latest()


def test_shm_reader_rejects_a_truncated_backing_object(tmp_path):
    path = tmp_path / "camera_color.shm"
    write_committed_frame(path)
    path.write_bytes(path.read_bytes()[:-1])

    with pytest.raises(FrameCorrupt, match="size mismatch"):
        ShmFrameReader(path, max_age_s=None).read_latest()


def test_shm_reader_accepts_same_sequence_from_a_new_writer_generation(tmp_path):
    path = tmp_path / "camera_color.shm"
    first_timestamp = time.time_ns()
    write_committed_frame(path, sequence=7, timestamp_ns=first_timestamp)
    reader = ShmFrameReader(path, max_age_s=None)
    assert reader.read_latest() is not None
    reader.close()

    second_timestamp = first_timestamp + 1_000_000
    write_committed_frame(path, sequence=7, timestamp_ns=second_timestamp)

    restarted = reader.read_latest()
    assert restarted is not None
    assert restarted.sequence == 7
    assert restarted.timestamp_ns == second_timestamp


def test_portable_writer_publishes_complete_consecutive_frames(tmp_path):
    path = tmp_path / "camera_color.shm"
    writer = ShmFrameWriter(path, stream_kind=StreamKind.COLOR, slot_capacity=128)
    reader = ShmFrameReader(path, max_age_s=None)
    first = bytes(range(12))
    second = bytes(reversed(range(12)))

    try:
        assert writer.publish(
            timestamp_ns=time.time_ns(),
            width=2,
            height=2,
            stride=6,
            encoding="rgb8",
            frame_id="camera_link",
            payload=first,
        ) == 1
        assert reader.read_latest().payload == first

        assert writer.publish(
            timestamp_ns=time.time_ns(),
            width=2,
            height=2,
            stride=6,
            encoding="rgb8",
            frame_id="camera_link",
            payload=second,
        ) == 2
        frame = reader.read_latest()
        assert frame is not None
        assert frame.sequence == 2
        assert frame.payload == second
        assert writer.last_sequence == 2
        assert path.stat().st_size == _SUPERBLOCK.size + 2 * (_SLOT.size + 128)
        if os.name != "nt":
            assert stat.S_IMODE(path.stat().st_mode) == 0o600
    finally:
        reader.close()
        writer.close()


def test_portable_writer_generation_changes_when_ring_is_recreated(tmp_path):
    path = tmp_path / "camera_info.shm"
    with ShmFrameWriter(path, stream_kind=StreamKind.INFO, slot_capacity=1) as first:
        first_generation = first.generation_ns
        first.publish(
            timestamp_ns=time.time_ns(),
            width=640,
            height=480,
            stride=0,
            encoding="camera_info",
            frame_id="camera_link",
            payload=b"",
            fx=500.0,
            fy=501.0,
            cx=320.0,
            cy=240.0,
        )
    time.sleep(0.001)
    with ShmFrameWriter(path, stream_kind=StreamKind.INFO, slot_capacity=1) as second:
        assert second.generation_ns > first_generation
        assert second.last_sequence == 0


def test_portable_writer_and_reader_never_expose_partial_concurrent_frames(tmp_path):
    path = tmp_path / "camera_color.shm"
    writer = ShmFrameWriter(path, stream_kind=StreamKind.COLOR, slot_capacity=192)
    reader = ShmFrameReader(path, max_age_s=None, consistency_attempts=20)
    expected_payloads = {bytes([value]) * 192 for value in range(1, 31)}
    observed: list[bytes] = []
    done = threading.Event()

    def publish() -> None:
        try:
            for value in range(1, 31):
                writer.publish(
                    timestamp_ns=time.time_ns(),
                    width=8,
                    height=8,
                    stride=24,
                    encoding="rgb8",
                    frame_id="camera_link",
                    payload=bytes([value]) * 192,
                )
        finally:
            done.set()

    thread = threading.Thread(target=publish)
    thread.start()
    try:
        while not done.is_set() or reader.last_sequence < writer.last_sequence:
            try:
                frame = reader.read_latest()
            except FrameChanged:
                continue
            if frame is not None:
                observed.append(frame.payload)
        thread.join(timeout=1.0)
        assert observed
        assert set(observed) <= expected_payloads
        assert all(len(set(payload)) == 1 for payload in observed)
    finally:
        reader.close()
        writer.close()


def test_portable_writer_validates_before_publishing_partial_frame(tmp_path):
    path = tmp_path / "camera_color.shm"
    with ShmFrameWriter(path, stream_kind=StreamKind.COLOR, slot_capacity=128) as writer:
        with pytest.raises(ValueError, match=r"stride \* height"):
            writer.publish(
                timestamp_ns=time.time_ns(),
                width=2,
                height=2,
                stride=6,
                encoding="rgb8",
                frame_id="camera_link",
                payload=b"partial",
            )

        with pytest.raises(FrameNotReady, match="no committed frame"):
            ShmFrameReader(path, max_age_s=None).read_latest()


def test_portable_writer_rejects_zero_timestamp_without_committing(tmp_path):
    path = tmp_path / "camera_color.shm"
    with ShmFrameWriter(path, stream_kind=StreamKind.COLOR, slot_capacity=128) as writer:
        with pytest.raises(ValueError, match="timestamp must be positive"):
            writer.publish(
                timestamp_ns=0,
                width=2,
                height=2,
                stride=6,
                encoding="rgb8",
                frame_id="camera_link",
                payload=bytes(12),
            )

        assert writer.last_sequence == 0
        with pytest.raises(FrameNotReady, match="no committed frame"):
            ShmFrameReader(path, max_age_s=None).read_latest()


def test_cpp_camera_service_uses_shm_as_default_image_data_plane():
    source = Path("src/drivers/real/camera/native/camera_dds.cpp").read_text(encoding="utf-8")
    contract = Path("src/drivers/real/camera/native/shm_frame_ring.hpp").read_text(encoding="utf-8")
    service = Path("scripts/deploy/thunder/lt-camera.service").read_text(encoding="utf-8")

    assert "publish_image_dds{false}" in source
    assert "Environment=LINGTU_CAMERA_PUBLISH_IMAGE_DDS=0" in service
    assert "Environment=LINGTU_CAMERA_PUBLISH_IMAGE_DDS=1" not in service
    assert 'arg == "--publish-image-dds"' in source
    assert "FrameWriter color_shm" in source
    assert "FrameWriter depth_shm" in source
    assert "FrameWriter info_shm" in source
    assert "kSchemaVersion = 1" in contract
    assert "payload_crc32" in contract
