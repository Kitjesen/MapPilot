"""IMU stack: inertial stream as an independent runtime role."""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import stack_module
from runtime.contracts import (
    IMU_BACKEND_DDS,
    IMU_BACKEND_LIVOX,
    IMU_BACKEND_MUJOCO,
    IMU_BACKENDS,
)

logger = logging.getLogger(__name__)


def imu(enabled: bool = True, backend: str = "livox") -> Blueprint:
    """IMU stream stack."""

    bp = Blueprint()
    if not enabled:
        return bp

    backend = (backend or IMU_BACKEND_LIVOX).strip()
    if backend not in IMU_BACKENDS:
        raise ValueError(f"Unsupported imu backend: {backend!r}; expected one of {IMU_BACKENDS}")
    if backend not in {IMU_BACKEND_LIVOX, IMU_BACKEND_MUJOCO, IMU_BACKEND_DDS}:
        raise ValueError(f"IMU backend {backend!r} is declared but not implemented by this stack")

    if backend == IMU_BACKEND_MUJOCO:
        fallback = "drivers.sim.imu.MujocoImuModule"
    elif backend == IMU_BACKEND_DDS:
        fallback = "drivers.real.imu.dds_module.DdsImuModule"
    else:
        fallback = "drivers.real.imu.ImuModule"

    try:
        ImuModule = stack_module(
            "imu",
            backend,
            seed_group="imu",
            fallback=fallback,
        )
    except ImportError as exc:
        logger.warning("IMU stack: imu backend not available: %s", exc)
        return bp

    bp.add(ImuModule, alias="imu")
    logger.info("IMU stack: imu added (backend=%s)", backend)
    return bp
