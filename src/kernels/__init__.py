"""Algorithm-kernel package namespace."""

from kernels.catalog import (
    KERNEL_TARGETS,
    KernelTarget,
    first_wave_targets,
    target_by_key,
)

__all__ = [
    "KERNEL_TARGETS",
    "KernelTarget",
    "first_wave_targets",
    "target_by_key",
]
