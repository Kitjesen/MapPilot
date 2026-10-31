"""Simulation capture time travels in the same committed slot as image bytes."""

from sim.adapters.shm.camera_ring import (
    SLOT_HEADER,
    SUPERBLOCK,
    CameraShmAllocation,
    CameraShmReader,
    CameraShmWriter,
    FrameMetadata,
    InMemoryMappingBackend,
    StreamKind,
)


def test_camera_frame_preserves_the_latched_simulation_stamp_not_poll_time():
    allocation = CameraShmAllocation("capture-truth", slot_capacity=6)
    mapping = InMemoryMappingBackend().create(allocation)
    writer = CameraShmWriter(mapping, allocation, now_ns=lambda: 1_800_000_000_000_000_000,
                             model_generation=3)
    writer.publish(FrameMetadata(
        StreamKind.COLOR, 1_800_000_000_000_000_000, 2, 1, 6, "rgb8", "robot/camera",
        fx=1, fy=1, cx=0.5, cy=0.0,
        truth_sequence=17, reset_generation=7, sim_time_ns=500_000_000,
    ), bytes([10, 20, 30, 40, 50, 60]))
    frame = CameraShmReader(mapping).read_latest(now_ns=1_800_000_000_100_000_000)
    assert frame is not None
    assert (frame.model_generation, frame.reset_generation, frame.truth_sequence, frame.sim_time_ns) == (
        3, 7, 17, 500_000_000,
    )
    assert frame.timestamp_ns == 1_800_000_000_000_000_000
    assert frame.payload == bytes([10, 20, 30, 40, 50, 60])
    assert (SUPERBLOCK.size, SLOT_HEADER.size) == (64, 256)


import pytest


@pytest.fixture
def windows_mapping_api():
    import ctypes
    import os
    from ctypes import wintypes

    if os.name != "nt":
        pytest.skip("requires real Windows named mappings")
    api = ctypes.WinDLL("kernel32", use_last_error=True)
    api.OpenFileMappingW.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.LPCWSTR)
    api.OpenFileMappingW.restype = wintypes.HANDLE
    api.CloseHandle.argtypes = (wintypes.HANDLE,)
    api.CloseHandle.restype = wintypes.BOOL
    return api


def _assert_windows_mapping_missing(api, name):
    import ctypes

    handle = api.OpenFileMappingW(0x0004, False, name)
    error = ctypes.get_last_error()
    if handle:
        assert api.CloseHandle(handle)
    assert not handle, "reader leaked or created a named mapping"
    assert error == 2


def test_windows_open_missing_mapping_is_not_ready_and_never_creates(windows_mapping_api):
    import uuid

    from sim.adapters.shm.camera_ring import CameraShmNotReady, WindowsNamedMappingBackend

    allocation = CameraShmAllocation(f"Local\\lingtu-camera-open-{uuid.uuid4().hex}", slot_capacity=6)
    _assert_windows_mapping_missing(windows_mapping_api, allocation.name)
    opened = None
    try:
        with pytest.raises(CameraShmNotReady, match="unavailable"):
            opened = WindowsNamedMappingBackend().open(allocation)
        _assert_windows_mapping_missing(windows_mapping_api, allocation.name)
    finally:
        if opened is not None:
            opened.close()


def test_windows_open_reads_existing_ring_and_repeated_close_releases_all_handles(windows_mapping_api):
    import uuid

    from sim.adapters.shm.camera_ring import WindowsNamedMappingBackend

    allocation = CameraShmAllocation(f"Local\\lingtu-camera-open-{uuid.uuid4().hex}", slot_capacity=6)
    backend = WindowsNamedMappingBackend()
    owner = backend.create(allocation)
    opened = None
    timestamp = 1_800_000_000_000_000_000
    try:
        writer = CameraShmWriter(owner, allocation, now_ns=lambda: timestamp, model_generation=3)
        writer.publish(FrameMetadata(
            StreamKind.COLOR, timestamp, 2, 1, 6, "rgb8", "robot/camera",
            truth_sequence=17, reset_generation=7, sim_time_ns=500_000_000,
        ), b"abcdef")
        for _ in range(3):
            opened = backend.open(allocation)
            frame = CameraShmReader(opened).read_latest(now_ns=timestamp)
            assert frame.payload == b"abcdef"
            assert (frame.model_generation, frame.reset_generation, frame.truth_sequence, frame.sim_time_ns) == (
                3, 7, 17, 500_000_000,
            )
            with pytest.raises(TypeError):
                opened[0] = 0
            opened.close()
            opened.close()
        opened = backend.open(allocation)
        owner.close()
        assert CameraShmReader(opened).read_latest(now_ns=timestamp).payload == b"abcdef"
    finally:
        if opened is not None:
            opened.close()
            opened.close()
        owner.close()
    _assert_windows_mapping_missing(windows_mapping_api, allocation.name)


def test_windows_open_keeps_source_alive_until_mmap_constructor_returns(windows_mapping_api, monkeypatch):
    import uuid

    from sim.adapters.shm import camera_ring

    allocation = CameraShmAllocation(f"Local\\lingtu-camera-open-{uuid.uuid4().hex}", slot_capacity=6)
    backend = camera_ring.WindowsNamedMappingBackend()
    owner = backend.create(allocation)
    owner[:6] = b"source"
    real_mmap = camera_ring.mmap.mmap
    opened = None

    def producer_exits_during_open(*args, **kwargs):
        owner.close()
        return real_mmap(*args, **kwargs)

    monkeypatch.setattr(camera_ring.mmap, "mmap", producer_exits_during_open)
    try:
        opened = backend.open(allocation)
        assert opened[:6] == b"source", "opening must retain the producer mapping, not recreate zero bytes"
    finally:
        if opened is not None:
            opened.close()
        owner.close()
    _assert_windows_mapping_missing(windows_mapping_api, allocation.name)


def test_windows_open_real_mmap_failure_releases_owned_handles(windows_mapping_api):
    import uuid

    from sim.adapters.shm.camera_ring import WindowsNamedMappingBackend

    allocation = CameraShmAllocation(f"Local\\lingtu-camera-open-{uuid.uuid4().hex}", slot_capacity=6)
    oversized = CameraShmAllocation(allocation.name, slot_capacity=64 * 1024)
    backend = WindowsNamedMappingBackend()
    owner = backend.create(allocation)
    try:
        owner[:6] = b"source"
        for _ in range(3):
            with pytest.raises(OSError):
                backend.open(oversized)
        assert owner[:6] == b"source"
    finally:
        owner.close()
    _assert_windows_mapping_missing(windows_mapping_api, allocation.name)


@pytest.mark.parametrize("stamp", [
    {"truth_sequence": 1}, {"reset_generation": 1}, {"sim_time_ns": 1},
    {"truth_sequence": 1, "reset_generation": 1},
    {"truth_sequence": 1, "sim_time_ns": 1},
    {"reset_generation": 1, "sim_time_ns": 1},
])
def test_partial_truth_stamp_is_rejected_before_any_slot_is_written(stamp):
    allocation = CameraShmAllocation("partial-camera-truth", slot_capacity=6)
    mapping = InMemoryMappingBackend().create(allocation)
    writer = CameraShmWriter(mapping, allocation, now_ns=lambda: 1_800_000_000_000_000_000)
    before = bytes(mapping)
    with pytest.raises(ValueError, match="complete nonnegative simulation stamp"):
        writer.publish(FrameMetadata(
            StreamKind.COLOR, 1_800_000_000_000_000_000, 2, 1, 6, "rgb8", "robot/camera", **stamp,
        ), b"abcdef")
    assert bytes(mapping) == before
    assert writer.last_sequence == 0


@pytest.mark.parametrize("reserved", [bytes(24), b"\xff" * 24])
def test_legacy_slot_without_truth_flag_ignores_reserved_bytes(reserved):
    allocation = CameraShmAllocation("legacy-camera-truth", slot_capacity=6)
    mapping = InMemoryMappingBackend().create(allocation)
    timestamp = 1_800_000_000_000_000_000
    writer = CameraShmWriter(mapping, allocation, now_ns=lambda: timestamp, model_generation=3)
    writer.publish(FrameMetadata(StreamKind.COLOR, timestamp, 2, 1, 6, "rgb8", "robot/camera"), b"abcdef")
    slot = list(SLOT_HEADER.unpack_from(mapping, SUPERBLOCK.size))
    assert slot[9] == 0
    slot[27] = reserved
    SLOT_HEADER.pack_into(mapping, SUPERBLOCK.size, *slot)
    frame = CameraShmReader(mapping).read_latest(now_ns=timestamp)
    assert frame.payload == b"abcdef"
    assert (frame.model_generation, frame.reset_generation, frame.truth_sequence, frame.sim_time_ns) == (
        None, None, None, None,
    )


def test_camera_writer_rejects_an_existing_ring_from_another_model_generation():
    from sim.adapters.shm.camera_ring import CameraShmCorrupt

    allocation = CameraShmAllocation("different-model-camera-truth", slot_capacity=6)
    mapping = InMemoryMappingBackend().create(allocation)
    timestamp = 1_800_000_000_000_000_000
    writer = CameraShmWriter(mapping, allocation, now_ns=lambda: timestamp, model_generation=3)
    writer.publish(FrameMetadata(
        StreamKind.COLOR, timestamp, 2, 1, 6, "rgb8", "robot/camera",
        truth_sequence=17, reset_generation=7, sim_time_ns=500_000_000,
    ), b"abcdef")
    before = bytes(mapping)
    with pytest.raises(CameraShmCorrupt, match="model_generation"):
        CameraShmWriter(mapping, allocation, now_ns=lambda: timestamp, model_generation=4)
    assert bytes(mapping) == before
    assert CameraShmReader(mapping).read_latest(now_ns=timestamp).model_generation == 3
