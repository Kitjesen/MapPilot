"""Portable compute-kernel migration catalog.

Metadata only. Runtime code should not branch on this file; it is for audits
and build scripts.

Only list materialized kernel directories here. Production C++ navigation
kernels that already live under ``src/nav/cpp`` stay with that owning
domain instead of getting speculative placeholder paths under ``src/kernels``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

BoundaryKind = Literal["c_abi", "process_abi"]
ImplementationLane = Literal[
    "cpp_portable",
    "rust_candidate",
]
MigrationStatus = Literal[
    "contract_first",
    "first_wave",
]


@dataclass(frozen=True)
class KernelTarget:
    """One compute surface that may move behind a portable boundary."""

    key: str
    layer: str
    responsibility: str
    current_paths: tuple[str, ...]
    target_path: str
    boundary: BoundaryKind
    lane: ImplementationLane
    status: MigrationStatus
    first_wave: bool
    notes: str = ""


def _target(
    key: str,
    layer: str,
    responsibility: str,
    current_paths: tuple[str, ...],
    target_path: str,
    boundary: BoundaryKind,
    lane: ImplementationLane,
    status: MigrationStatus,
    *,
    first_wave: bool = False,
    notes: str = "",
) -> KernelTarget:
    return KernelTarget(
        key=key,
        layer=layer,
        responsibility=responsibility,
        current_paths=current_paths,
        target_path=target_path,
        boundary=boundary,
        lane=lane,
        status=status,
        first_wave=first_wave,
        notes=notes,
    )


CALIBRATION_NOTES = (
    "Offline calibration path, not a robot runtime blocker. The Rust kernel covers "
    "fixed-correspondence CT-ICP plus CT-GICP Mahalanobis residual/Jacobian/"
    "Hessian math and a CT-GICP two-pose optimizer behind a C ABI. The dynamic "
    "integrator calls the Rust optimizer ABI directly and can require Rust "
    "success instead of silently falling back to GTSAM LM. Visual calibration "
    "no longer uses GTSAM Pose3::Expmap, and the dynamic integrator header "
    "stores Eigen poses instead of exposing GTSAM Pose3. Rust-owned "
    "correspondence search and GTSAM build/package removal remain."
)

POINTCLOUD_CODEC_NOTES = (
    "Gateway/browser live-cloud hot path. Python owns control flow, but the "
    "PCLD float32->int16 wire packing can be served by this C ABI when the "
    "shared library is built."
)


KERNEL_TARGETS: tuple[KernelTarget, ...] = (
    _target(
        "path_safety",
        "L2 safety gate",
        "Accept, reject, or trim paths against live cost and geometry",
        ("src/maps/services/layers/traversability.py",),
        "src/kernels/nav/path_safety",
        "c_abi",
        "rust_candidate",
        "contract_first",
        first_wave=True,
        notes="Good Rust candidate after fixture coverage is frozen.",
    ),
    _target(
        "camera_lidar_calibration_optimizer",
        "calibration tooling",
        "Estimate camera-LiDAR extrinsics with ICP/GICP and CT interpolation factors",
        ("tools/calibration/camera_lidar/direct_visual_lidar_calibration",),
        "src/kernels/calibration/camera_lidar_optimizer",
        "process_abi",
        "rust_candidate",
        "contract_first",
        notes=CALIBRATION_NOTES,
    ),
    _target(
        "gateway_pointcloud_codec",
        "L6 interface",
        "Encode live PointCloud2 frames into compact binary PCLD websocket frames",
        ("src/runtime/utils/binary_codec.py", "src/gateway/gateway_module.py"),
        "src/kernels/gateway/pointcloud_codec",
        "c_abi",
        "cpp_portable",
        "first_wave",
        first_wave=True,
        notes=POINTCLOUD_CODEC_NOTES,
    ),
)


def first_wave_targets() -> tuple[KernelTarget, ...]:
    """Kernels that materially improve deployment when isolated first."""

    return tuple(target for target in KERNEL_TARGETS if target.first_wave)


def target_by_key(key: str) -> KernelTarget:
    """Return a kernel target by stable key."""

    for target in KERNEL_TARGETS:
        if target.key == key:
            return target
    raise KeyError(key)


__all__ = [
    "KERNEL_TARGETS",
    "BoundaryKind",
    "ImplementationLane",
    "KernelTarget",
    "MigrationStatus",
    "first_wave_targets",
    "target_by_key",
]
