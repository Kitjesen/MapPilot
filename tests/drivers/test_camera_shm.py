from __future__ import annotations

# ruff: noqa: D103,S101
import struct
import time
import zlib
from pathlib import Path

import pytest

from drivers.real.camera.shm import (
    FrameChanged,
    FrameCorrupt,
    FrameStale,
    ShmFrameReader,
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


def test_cpp_camera_service_uses_shm_as_default_image_data_plane():
    source = Path("src/drivers/real/camera/native/camera_dds.cpp").read_text(encoding="utf-8")
    contract = Path("src/drivers/real/camera/native/shm_frame_ring.hpp").read_text(encoding="utf-8")

    assert "publish_image_dds{false}" in source
    assert 'arg == "--publish-image-dds"' in source
    assert "FrameWriter color_shm" in source
    assert "FrameWriter depth_shm" in source
    assert "FrameWriter info_shm" in source
    assert "kSchemaVersion = 1" in contract
    assert "payload_crc32" in contract
