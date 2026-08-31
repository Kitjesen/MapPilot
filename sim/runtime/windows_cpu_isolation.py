"""Optional Windows CPU isolation for the interactive simulation runtime."""

from __future__ import annotations

import ctypes
import os
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Protocol


class WindowsCpuIsolationError(ValueError):
    """Raised when a requested Windows affinity partition is not safe."""


def validate_windows_affinity_mask(mask: object, field: str = "affinity mask") -> int:
    """Return one non-empty pointer-sized affinity mask."""

    if isinstance(mask, bool) or not isinstance(mask, int) or mask <= 0:
        raise WindowsCpuIsolationError(f"{field} must be a non-empty positive integer")
    maximum = (1 << (ctypes.sizeof(ctypes.c_size_t) * 8)) - 1
    if mask > maximum:
        raise WindowsCpuIsolationError(f"{field} exceeds the native affinity width")
    return mask


@dataclass(frozen=True, slots=True)
class WindowsPhysicalCore:
    """One physical core and every logical processor that belongs to it."""

    core_id: int
    processor_group: int
    affinity_mask: int
    efficiency_class: int = 0

    def __post_init__(self) -> None:
        if (
            isinstance(self.core_id, bool)
            or not isinstance(self.core_id, int)
            or self.core_id < 0
        ):
            raise WindowsCpuIsolationError("physical core ID must be non-negative")
        if (
            isinstance(self.processor_group, bool)
            or not isinstance(self.processor_group, int)
            or self.processor_group < 0
        ):
            raise WindowsCpuIsolationError("processor group must be non-negative")
        validate_windows_affinity_mask(self.affinity_mask, "physical core affinity mask")
        if (
            isinstance(self.efficiency_class, bool)
            or not isinstance(self.efficiency_class, int)
            or not 0 <= self.efficiency_class <= 255
        ):
            raise WindowsCpuIsolationError("efficiency class must be in [0, 255]")


@dataclass(frozen=True, slots=True)
class WindowsCpuTopology:
    """Validated physical-core topology reported by Windows."""

    cores: tuple[WindowsPhysicalCore, ...]

    def __post_init__(self) -> None:
        cores = tuple(self.cores)
        object.__setattr__(self, "cores", cores)
        if not cores:
            raise WindowsCpuIsolationError("Windows CPU topology has no physical cores")
        core_ids: set[int] = set()
        group_masks: dict[int, int] = {}
        for core in cores:
            if not isinstance(core, WindowsPhysicalCore):
                raise WindowsCpuIsolationError(
                    "Windows CPU topology entries must be physical cores"
                )
            if core.core_id in core_ids:
                raise WindowsCpuIsolationError("physical core IDs must be unique")
            core_ids.add(core.core_id)
            occupied = group_masks.get(core.processor_group, 0)
            if occupied & core.affinity_mask:
                raise WindowsCpuIsolationError(
                    "physical core affinity masks overlap within a processor group"
                )
            group_masks[core.processor_group] = occupied | core.affinity_mask

    @property
    def processor_groups(self) -> tuple[int, ...]:
        """Return the processor groups represented by this topology."""

        return tuple(sorted({core.processor_group for core in self.cores}))

    def affinity_mask(self, processor_group: int) -> int:
        """Return the complete active mask for one processor group."""

        mask = 0
        for core in self.cores:
            if core.processor_group == processor_group:
                mask |= core.affinity_mask
        return validate_windows_affinity_mask(mask, "processor group affinity mask")


@dataclass(frozen=True, slots=True)
class WindowsCpuIsolationConfig:
    """Topology-relative selection of two latency-sensitive physical cores."""

    mujoco_performance_rank: int = 0
    owner_performance_rank: int = 1

    def __post_init__(self) -> None:
        for field in ("mujoco_performance_rank", "owner_performance_rank"):
            value = getattr(self, field)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise WindowsCpuIsolationError(
                    f"{field} must be a non-negative physical-core rank"
                )
        if self.mujoco_performance_rank == self.owner_performance_rank:
            raise WindowsCpuIsolationError(
                "MuJoCo and owner thread physical-core ranks must be distinct"
            )


@dataclass(frozen=True, slots=True)
class WindowsCpuIsolationPlan:
    """Resolved, pairwise-disjoint affinity masks for one playable launch."""

    processor_group: int
    mujoco_core_id: int
    owner_core_id: int
    unreal_core_ids: tuple[int, ...]
    mujoco_affinity_mask: int
    owner_thread_affinity_mask: int
    unreal_affinity_mask: int

    def __post_init__(self) -> None:
        if (
            isinstance(self.processor_group, bool)
            or not isinstance(self.processor_group, int)
            or self.processor_group < 0
        ):
            raise WindowsCpuIsolationError("processor group must be non-negative")
        masks = (
            validate_windows_affinity_mask(
                self.mujoco_affinity_mask,
                "MuJoCo affinity mask",
            ),
            validate_windows_affinity_mask(
                self.owner_thread_affinity_mask,
                "owner thread affinity mask",
            ),
            validate_windows_affinity_mask(
                self.unreal_affinity_mask,
                "Unreal affinity mask",
            ),
        )
        if masks[0] & masks[1] or masks[0] & masks[2] or masks[1] & masks[2]:
            raise WindowsCpuIsolationError("resolved role affinity masks overlap")
        if not self.unreal_core_ids:
            raise WindowsCpuIsolationError("Unreal physical-core partition is empty")
        role_ids = (self.mujoco_core_id, self.owner_core_id, *self.unreal_core_ids)
        if any(isinstance(value, bool) or not isinstance(value, int) or value < 0 for value in role_ids):
            raise WindowsCpuIsolationError("resolved physical core IDs must be non-negative")
        if len(set(role_ids)) != len(role_ids):
            raise WindowsCpuIsolationError("resolved physical core partitions overlap")


def resolve_windows_cpu_isolation(
    config: WindowsCpuIsolationConfig,
    *,
    topology: WindowsCpuTopology | None = None,
) -> WindowsCpuIsolationPlan:
    """Resolve two whole latency cores and leave every other core to Unreal."""

    if not isinstance(config, WindowsCpuIsolationConfig):
        raise TypeError("config must be WindowsCpuIsolationConfig")
    use_launcher_affinity = topology is None
    resolved_topology = topology or discover_windows_cpu_topology()
    if not isinstance(resolved_topology, WindowsCpuTopology):
        raise TypeError("topology must be WindowsCpuTopology")
    if resolved_topology.processor_groups != (0,):
        raise WindowsCpuIsolationError(
            "affinity v1 requires one Windows processor group numbered zero"
        )
    if use_launcher_affinity:
        process_mask = discover_windows_process_affinity_mask()
        eligible_cores = tuple(
            core
            for core in resolved_topology.cores
            if core.affinity_mask & process_mask == core.affinity_mask
        )
        if not eligible_cores:
            raise WindowsCpuIsolationError(
                "launcher process affinity exposes no complete physical core"
            )
        resolved_topology = WindowsCpuTopology(cores=eligible_cores)

    ranked = sorted(
        resolved_topology.cores,
        key=lambda core: (-core.efficiency_class, core.core_id),
    )
    highest_rank = max(
        config.mujoco_performance_rank,
        config.owner_performance_rank,
    )
    if highest_rank >= len(ranked):
        raise WindowsCpuIsolationError(
            "requested physical-core rank is outside the discovered topology"
        )
    mujoco_core = ranked[config.mujoco_performance_rank]
    owner_core = ranked[config.owner_performance_rank]
    unreal_cores = tuple(
        core
        for core in resolved_topology.cores
        if core.core_id not in {mujoco_core.core_id, owner_core.core_id}
    )
    if not unreal_cores:
        raise WindowsCpuIsolationError("Unreal partition is empty after reserving two cores")
    unreal_mask = 0
    for core in unreal_cores:
        unreal_mask |= core.affinity_mask
    return WindowsCpuIsolationPlan(
        processor_group=0,
        mujoco_core_id=mujoco_core.core_id,
        owner_core_id=owner_core.core_id,
        unreal_core_ids=tuple(core.core_id for core in unreal_cores),
        mujoco_affinity_mask=mujoco_core.affinity_mask,
        owner_thread_affinity_mask=owner_core.affinity_mask,
        unreal_affinity_mask=unreal_mask,
    )


class WindowsThreadAffinityApi(Protocol):
    """Minimal native seam used by the reversible thread affinity scope."""

    def bind_current_thread(self, mask: int) -> int:
        """Bind the current thread and return its previous affinity mask."""

        ...

    def restore_current_thread(self, mask: int) -> None:
        """Restore the current thread to a previously returned mask."""

        ...


@contextmanager
def bind_current_thread_affinity(
    mask: int,
    *,
    api: WindowsThreadAffinityApi | None = None,
) -> Iterator[None]:
    """Bind the current thread and restore its exact prior affinity on exit."""

    affinity_mask = validate_windows_affinity_mask(mask, "thread affinity mask")
    native = api
    if native is None:
        if os.name != "nt":
            raise WindowsCpuIsolationError(
                "thread affinity is only available on Windows"
            )
        native = _NativeWindowsThreadAffinityApi()
    previous_mask = native.bind_current_thread(affinity_mask)
    try:
        validate_windows_affinity_mask(previous_mask, "previous thread affinity mask")
        yield
    finally:
        native.restore_current_thread(previous_mask)


def discover_windows_cpu_topology() -> WindowsCpuTopology:
    """Read physical-core masks from GetLogicalProcessorInformationEx."""

    if os.name != "nt":
        raise WindowsCpuIsolationError("CPU topology discovery is only available on Windows")
    required = wintypes.DWORD(0)
    if _KERNEL32.GetLogicalProcessorInformationEx(
        _RELATION_PROCESSOR_CORE,
        None,
        ctypes.byref(required),
    ):
        raise WindowsCpuIsolationError("Windows returned an empty topology probe")
    error = ctypes.get_last_error()
    if error != _ERROR_INSUFFICIENT_BUFFER or required.value <= 0:
        raise ctypes.WinError(error)
    buffer = ctypes.create_string_buffer(required.value)
    if not _KERNEL32.GetLogicalProcessorInformationEx(
        _RELATION_PROCESSOR_CORE,
        ctypes.byref(buffer),
        ctypes.byref(required),
    ):
        raise ctypes.WinError(ctypes.get_last_error())

    discovered: list[tuple[int, int, int]] = []
    base = ctypes.addressof(buffer)
    offset = 0
    while offset < required.value:
        if required.value - offset < _LOGICAL_PROCESSOR_HEADER_SIZE:
            raise WindowsCpuIsolationError("Windows CPU topology record is truncated")
        relationship = ctypes.c_int.from_address(base + offset).value
        size = wintypes.DWORD.from_address(base + offset + ctypes.sizeof(ctypes.c_int)).value
        if size < _LOGICAL_PROCESSOR_HEADER_SIZE or offset + size > required.value:
            raise WindowsCpuIsolationError("Windows CPU topology record has invalid size")
        if relationship == _RELATION_PROCESSOR_CORE:
            processor_address = base + offset + _LOGICAL_PROCESSOR_HEADER_SIZE
            processor = _ProcessorRelationship.from_address(processor_address)
            if processor.GroupCount != 1:
                raise WindowsCpuIsolationError(
                    "one physical core spans multiple processor groups"
                )
            group = _GroupAffinity.from_address(
                processor_address + _ProcessorRelationship.GroupMask.offset
            )
            discovered.append(
                (int(group.Group), int(group.Mask), int(processor.EfficiencyClass))
            )
        offset += size
    if offset != required.value:
        raise WindowsCpuIsolationError("Windows CPU topology buffer was not consumed")
    discovered.sort(key=lambda item: (item[0], _lowest_set_bit(item[1])))
    return WindowsCpuTopology(
        cores=tuple(
            WindowsPhysicalCore(
                core_id=index,
                processor_group=group,
                affinity_mask=mask,
                efficiency_class=efficiency_class,
            )
            for index, (group, mask, efficiency_class) in enumerate(discovered)
        )
    )


def discover_windows_process_affinity_mask() -> int:
    """Return the effective group-zero affinity mask of the launcher process."""

    if os.name != "nt":
        raise WindowsCpuIsolationError(
            "process affinity discovery is only available on Windows"
        )
    process_mask = ctypes.c_size_t()
    system_mask = ctypes.c_size_t()
    if not _KERNEL32.GetProcessAffinityMask(
        _KERNEL32.GetCurrentProcess(),
        ctypes.byref(process_mask),
        ctypes.byref(system_mask),
    ):
        raise ctypes.WinError(ctypes.get_last_error())
    return validate_windows_affinity_mask(
        int(process_mask.value),
        "launcher process affinity mask",
    )


def _lowest_set_bit(mask: int) -> int:
    return (mask & -mask).bit_length() - 1


if os.name == "nt":
    from ctypes import wintypes

    _RELATION_PROCESSOR_CORE = 0
    _ERROR_INSUFFICIENT_BUFFER = 122
    _LOGICAL_PROCESSOR_HEADER_SIZE = ctypes.sizeof(ctypes.c_int) + ctypes.sizeof(
        wintypes.DWORD
    )

    class _GroupAffinity(ctypes.Structure):
        _fields_ = [
            ("Mask", ctypes.c_size_t),
            ("Group", wintypes.WORD),
            ("Reserved", wintypes.WORD * 3),
        ]

    class _ProcessorRelationship(ctypes.Structure):
        _fields_ = [
            ("Flags", ctypes.c_ubyte),
            ("EfficiencyClass", ctypes.c_ubyte),
            ("Reserved", ctypes.c_ubyte * 20),
            ("GroupCount", wintypes.WORD),
            ("GroupMask", _GroupAffinity * 1),
        ]

    _KERNEL32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _KERNEL32.GetLogicalProcessorInformationEx.argtypes = [
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.POINTER(wintypes.DWORD),
    ]
    _KERNEL32.GetLogicalProcessorInformationEx.restype = wintypes.BOOL
    _KERNEL32.GetCurrentProcess.argtypes = []
    _KERNEL32.GetCurrentProcess.restype = wintypes.HANDLE
    _KERNEL32.GetProcessAffinityMask.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.POINTER(ctypes.c_size_t),
    ]
    _KERNEL32.GetProcessAffinityMask.restype = wintypes.BOOL
    _KERNEL32.GetCurrentThread.argtypes = []
    _KERNEL32.GetCurrentThread.restype = wintypes.HANDLE
    _KERNEL32.SetThreadAffinityMask.argtypes = [
        wintypes.HANDLE,
        ctypes.c_size_t,
    ]
    _KERNEL32.SetThreadAffinityMask.restype = ctypes.c_size_t

    class _NativeWindowsThreadAffinityApi:
        def bind_current_thread(self, mask: int) -> int:
            previous = _KERNEL32.SetThreadAffinityMask(
                _KERNEL32.GetCurrentThread(),
                mask,
            )
            if not previous:
                raise ctypes.WinError(ctypes.get_last_error())
            return int(previous)

        def restore_current_thread(self, mask: int) -> None:
            if not _KERNEL32.SetThreadAffinityMask(
                _KERNEL32.GetCurrentThread(),
                mask,
            ):
                raise ctypes.WinError(ctypes.get_last_error())
