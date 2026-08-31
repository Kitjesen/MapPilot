"""Simulation-side camera shared-memory transport adapters.

This package mirrors the canonical ``lingtu.camera.shm_frame.v1`` ABI used by
the field camera service.  It owns no stream names: callers provide a
``CameraShmAllocation`` for each runtime allocation.
"""

from .camera_ring import (
    SHM_MAGIC,
    SHM_SCHEMA,
    SHM_SCHEMA_VERSION,
    SLOT_HEADER,
    SUPERBLOCK,
    CameraShmAllocation,
    CameraShmChanged,
    CameraShmCorrupt,
    CameraShmError,
    CameraShmNotReady,
    CameraShmReader,
    CameraShmStale,
    CameraShmWriter,
    FrameMetadata,
    FrameSnapshot,
    InMemoryMappingBackend,
    StreamKind,
    WindowsNamedMappingBackend,
)

__all__ = [
    "SHM_MAGIC",
    "SHM_SCHEMA",
    "SHM_SCHEMA_VERSION",
    "SLOT_HEADER",
    "SUPERBLOCK",
    "CameraShmAllocation",
    "CameraShmChanged",
    "CameraShmCorrupt",
    "CameraShmError",
    "CameraShmNotReady",
    "CameraShmReader",
    "CameraShmStale",
    "CameraShmWriter",
    "FrameMetadata",
    "FrameSnapshot",
    "InMemoryMappingBackend",
    "StreamKind",
    "WindowsNamedMappingBackend",
]
