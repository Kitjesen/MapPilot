"""Backend runtime setup for nav.path_follower."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Callable

from nav.local.path_follower_backend import (
    PidFallbackParams,
    create_nav_kernel_path_follower_adapter_from_tuning,
    read_pid_fallback_params,
)
from runtime.backend_status import BackendStatus

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class PathFollowerTuning:
    max_speed: float
    lookahead: float
    goal_tolerance: float
    max_yaw_rate: float | None
    turn_speed_yaw_rate_start: float
    turn_speed_min_scale: float
    yaw_rate_gain: float
    stop_yaw_rate_gain: float
    dir_diff_thre: float
    two_way_drive: bool


@dataclass(frozen=True)
class PathFollowerRuntime:
    backend: str
    status: BackendStatus
    nav_kernel: Any | None = None
    nav_kernel_params: Any | None = None
    nav_kernel_state: Any | None = None
    pid_params: PidFallbackParams | None = None


def setup_path_follower_runtime(
    backend: str,
    *,
    status: BackendStatus,
    tuning: PathFollowerTuning,
    nav_kernel_importer: Callable[[tuple[str, ...]], Any | None] | None = None,
) -> PathFollowerRuntime:
    """Create the requested path-follower backend runtime."""

    if backend == "nav_kernel":
        return _setup_native_kernel(status, tuning, importer=nav_kernel_importer)
    return _setup_pid(status, tuning.max_speed)


def _setup_pid(
    status: BackendStatus,
    max_speed: float,
) -> PathFollowerRuntime:
    params = read_pid_fallback_params(max_speed)
    if params.loaded_from_config:
        logger.info(
            "PathFollower [pid]: loaded adaptive PP params "
            "(k_v=%.2f, L=[%.2f,%.2f], a_max=%.2f, v_max=%.2f)",
            params.k_v,
            params.l_min,
            params.l_max,
            params.a_max,
            params.v_max,
        )
    else:
        logger.info(
            "PathFollower [pid]: using default adaptive PP params "
            "(k_v=0.5, L=[0.5,2.0], a_max=1.0) - "
            "override via robot_config.yaml"
        )
    return PathFollowerRuntime(
        backend="pid",
        status=status,
        pid_params=params,
    )


def _setup_native_kernel(
    status: BackendStatus,
    tuning: PathFollowerTuning,
    *,
    importer: Callable[[tuple[str, ...]], Any | None] | None = None,
) -> PathFollowerRuntime:
    adapter = create_nav_kernel_path_follower_adapter_from_tuning(
        max_speed=tuning.max_speed,
        lookahead=tuning.lookahead,
        goal_tolerance=tuning.goal_tolerance,
        max_yaw_rate=tuning.max_yaw_rate,
        turn_speed_yaw_rate_start=tuning.turn_speed_yaw_rate_start,
        turn_speed_min_scale=tuning.turn_speed_min_scale,
        yaw_rate_gain=tuning.yaw_rate_gain,
        stop_yaw_rate_gain=tuning.stop_yaw_rate_gain,
        dir_diff_thre=tuning.dir_diff_thre,
        two_way_drive=tuning.two_way_drive,
        **({"importer": importer} if importer is not None else {}),
    )
    if adapter.runtime is None:
        reason = adapter.degraded_reason or "compatible LingTu native navigation kernel missing"
        if adapter.build_hint:
            logger.info(
                "PathFollower: LingTu native navigation kernel not found - using pid backend.\n"
                "  To enable C++ path follower:\n  %s",
                adapter.build_hint,
            )
        else:
            logger.warning(
                "PathFollower: native navigation kernel error: %s - using pid backend",
                reason,
            )
        status.use("pid", reason=reason)
        return _setup_pid(status, tuning.max_speed)

    logger.info("PathFollower [nav_kernel]: C++ compute_control loaded")
    return PathFollowerRuntime(
        backend="nav_kernel",
        status=status,
        nav_kernel=adapter.runtime,
        nav_kernel_params=adapter.params,
        nav_kernel_state=adapter.state,
    )
