from __future__ import annotations

# Pytest assertions are the public behavior checks for this contract.
# ruff: noqa: S101
import os
import struct
import uuid

import pytest

from sim.adapters.shm import (
    SHM_MAGIC,
    SHM_SCHEMA,
    SHM_SCHEMA_VERSION,
    SLOT_HEADER,
    SUPERBLOCK,
    CameraShmAllocation,
    CameraShmCorrupt,
    CameraShmNotReady,
    CameraShmReader,
    CameraShmWriter,
    FrameMetadata,
    InMemoryMappingBackend,
    StreamKind,
    WindowsNamedMappingBackend,
)


def _allocation(*, name: str = "sim/camera/color", capacity: int = 32) -> CameraShmAllocation:
    return CameraShmAllocation(name=name, slot_count=2, slot_capacity=capacity)


def _metadata(*, timestamp_ns: int = 1_000_000_000) -> FrameMetadata:
    return FrameMetadata(
        stream_kind=StreamKind.COLOR,
        timestamp_ns=timestamp_ns,
        width=2,
        height=2,
        stride=6,
        encoding="rgb8",
        frame_id="camera_color",
        fx=10.0,
        fy=11.0,
        cx=1.0,
        cy=1.0,
    )


def test_in_memory_ring_uses_the_canonical_v1_bytes() -> None:
    allocation = _allocation()
    backend = InMemoryMappingBackend()
    mapping = backend.create(allocation)
    writer = CameraShmWriter(mapping, allocation, now_ns=lambda: 2_000_000_000)

    sequence = writer.publish(_metadata(), b"abcdefghijkl")

    assert sequence == 1
    assert len(mapping) == allocation.mapping_size
    assert SUPERBLOCK.size == 64
    assert SLOT_HEADER.size == 256

    superblock = SUPERBLOCK.unpack(bytes(mapping[: SUPERBLOCK.size]))
    assert superblock[:9] == (
        SHM_MAGIC,
        SHM_SCHEMA_VERSION,
        SUPERBLOCK.size,
        SLOT_HEADER.size,
        2,
        32,
        0,
        0,
        0,
    )
    assert superblock[9] == 1
    assert superblock[10] == 2_000_000_000
    assert superblock[11] == 2_000_000_000

    slot_offset = SUPERBLOCK.size
    slot = SLOT_HEADER.unpack(bytes(mapping[slot_offset : slot_offset + SLOT_HEADER.size]))
    assert slot[0] == 2
    assert slot[1] == 1
    assert slot[2] == 1_000_000_000
    assert slot[3:9] == (2, 2, 6, 12, 32, 0xF6781B24)
    assert slot[11:15] == (StreamKind.COLOR, 4, 1, SLOT_HEADER.size)
    assert slot[15].startswith(b"rgb8")
    assert slot[16].startswith(b"camera_color")
    assert bytes(mapping[slot_offset + SLOT_HEADER.size : slot_offset + SLOT_HEADER.size + 12]) == b"abcdefghijkl"
    assert SHM_SCHEMA == "lingtu.camera.shm_frame.v1"


def test_reader_returns_each_new_sequence_once_and_ring_wraps() -> None:
    allocation = _allocation()
    backend = InMemoryMappingBackend()
    mapping = backend.create(allocation)
    writer = CameraShmWriter(mapping, allocation, now_ns=lambda: 2_000_000_000)
    reader = CameraShmReader(mapping, max_age_s=None)

    writer.publish(_metadata(), b"abcdefghijkl")
    first = reader.read_latest(now_ns=2_000_000_000)
    assert first is not None
    assert first.sequence == 1
    assert first.payload == b"abcdefghijkl"
    assert reader.read_latest(now_ns=2_000_000_000) is None

    writer.publish(_metadata(timestamp_ns=1_000_000_001), b"mnopqrstuvwx")
    second = reader.read_latest(now_ns=2_000_000_000)
    assert second is not None
    assert second.sequence == 2
    assert second.payload == b"mnopqrstuvwx"

    writer.publish(_metadata(timestamp_ns=1_000_000_002), b"ABCDEFGHIJKL")
    third = reader.read_latest(now_ns=2_000_000_000)
    assert third is not None
    assert third.sequence == 3
    assert third.payload == b"ABCDEFGHIJKL"
    assert SUPERBLOCK.unpack(bytes(mapping[: SUPERBLOCK.size]))[6] == 0


def test_reader_requires_a_committed_frame_and_rejects_corrupt_crc() -> None:
    allocation = _allocation()
    backend = InMemoryMappingBackend()
    mapping = backend.create(allocation)
    CameraShmWriter(mapping, allocation, now_ns=lambda: 2_000_000_000)
    reader = CameraShmReader(mapping, max_age_s=None)

    with pytest.raises(CameraShmNotReady):
        reader.read_latest(now_ns=2_000_000_000)

    writer = CameraShmWriter(mapping, allocation, now_ns=lambda: 2_000_000_000)
    writer.publish(_metadata(), b"abcdefghijkl")
    payload_offset = SUPERBLOCK.size + SLOT_HEADER.size
    mapping[payload_offset] ^= 0x01

    with pytest.raises(CameraShmCorrupt, match="CRC"):
        reader.read_latest(now_ns=2_000_000_000)


def test_allocation_name_is_explicit_and_not_a_global_stream_constant() -> None:
    backend = InMemoryMappingBackend()
    first = _allocation(name="run-001/color")
    second = _allocation(name="run-002/color")

    first_mapping = backend.create(first)
    second_mapping = backend.create(second)
    assert first_mapping is not second_mapping

    CameraShmWriter(first_mapping, first, now_ns=lambda: 2_000_000_000).publish(_metadata(), b"abcdefghijkl")
    CameraShmWriter(second_mapping, second, now_ns=lambda: 2_000_000_000)
    with pytest.raises(CameraShmNotReady):
        CameraShmReader(second_mapping, max_age_s=None).read_latest(now_ns=2_000_000_000)


def test_canonical_structs_are_little_endian_without_padding_drift() -> None:
    assert SUPERBLOCK.format.startswith("<")
    assert SLOT_HEADER.format.startswith("<")
    assert struct.calcsize(SUPERBLOCK.format) == SUPERBLOCK.size
    assert struct.calcsize(SLOT_HEADER.format) == SLOT_HEADER.size


@pytest.mark.skipif(os.name != "nt", reason="named mappings are Windows-only")
def test_windows_named_mapping_uses_the_same_explicit_ring_allocation() -> None:
    allocation = _allocation(name=f"lingtu-camera-test-{uuid.uuid4().hex}")
    backend = WindowsNamedMappingBackend()
    writer_mapping = backend.create(allocation)
    reader_mapping = backend.open(allocation)
    try:
        CameraShmWriter(writer_mapping, allocation, now_ns=lambda: 2_000_000_000).publish(_metadata(), b"abcdefghijkl")
        frame = CameraShmReader(reader_mapping, max_age_s=None).read_latest(now_ns=2_000_000_000)
        assert frame is not None
        assert frame.sequence == 1
        assert frame.payload == b"abcdefghijkl"
    finally:
        reader_mapping.close()
        writer_mapping.close()
