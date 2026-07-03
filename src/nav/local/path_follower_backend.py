"""Backend adapter helpers for :mod:`nav.local.path_follower`.

These helpers keep optional nav_kernel backend selection and parameter assembly
out of PathFollower while preserving the module's existing runtime
behavior.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable

from nav.kernel import (
    nav_kernel_build_hint,
    try_import_nav_kernel,
)


NAV_KERNEL_PATH_FOLLOWER_SYMBOLS = (
    "PathFollowerParams",
    "PathFollowerState",
    "compute_control",
)


@dataclass(frozen=True)
class NavKernelPathFollowerConfig:
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
    max_accel: float = 1.0


@dataclass(frozen=True)
class NavKernelPathFollowerAdapter:
    runtime: Any | None
    params: Any | None = None
    state: Any | None = None
    degraded_reason: str | None = None
    build_hint: str | None = None


@dataclass(frozen=True)
class PidFallbackParams:
    k_v: float
    l_min: float
    l_max: float
    a_max: float
    v_max: float
    loaded_from_config: bool = False


def load_nav_kernel_runtime(
    importer: Callable[[tuple[str, ...]], Any | None] = try_import_nav_kernel,
) -> Any | None:
    """Return a compatible LingTu native navigation kernel, or None when unavailable."""
    return importer(NAV_KERNEL_PATH_FOLLOWER_SYMBOLS)


def build_nav_kernel_path_follower(
    nav_kernel: Any,
    config: NavKernelPathFollowerConfig,
) -> tuple[Any, Any]:
    """Build PathFollowerParams and PathFollowerState for a nav_kernel runtime."""
    params = nav_kernel.PathFollowerParams()
    params.max_speed = config.max_speed
    params.stop_dis_thre = config.goal_tolerance
    # compute_control compares the selected lookahead point distance against
    # stop_dis_thre. Dense local-planner paths can otherwise select a near-start
    # point inside the stop band and never ramp up.
    min_lookahead = max(0.2, params.stop_dis_thre + 0.05)
    params.base_look_ahead_dis = max(config.lookahead * 0.2, min_lookahead)
    params.min_look_ahead_dis = min_lookahead
    params.max_look_ahead_dis = max(min(config.lookahead, 2.0), min_lookahead)
    params.look_ahead_ratio = 0.5
    params.yaw_rate_gain = config.yaw_rate_gain
    params.stop_yaw_rate_gain = config.stop_yaw_rate_gain
    params.max_yaw_rate = (
        math.degrees(float(config.max_yaw_rate))
        if config.max_yaw_rate is not None
        else 45.0
    )
    params.max_accel = max(0.01, float(config.max_accel))
    if hasattr(params, "turn_speed_yaw_rate_start"):
        params.turn_speed_yaw_rate_start = config.turn_speed_yaw_rate_start
    if hasattr(params, "turn_speed_min_scale"):
        params.turn_speed_min_scale = config.turn_speed_min_scale
    params.switch_time_thre = 1.0
    params.dir_diff_thre = config.dir_diff_thre
    params.omni_dir_goal_thre = 1.0
    params.omni_dir_diff_thre = 1.5
    params.slow_dwn_dis_thre = 1.0
    params.two_way_drive = config.two_way_drive
    params.no_rot_at_goal = True
    return params, nav_kernel.PathFollowerState()


def create_nav_kernel_path_follower_adapter(
    config: NavKernelPathFollowerConfig,
    importer: Callable[[tuple[str, ...]], Any | None] = try_import_nav_kernel,
    build_hint: Callable[[], str] = nav_kernel_build_hint,
) -> NavKernelPathFollowerAdapter:
    """Load nav_kernel and create its PathFollower adapter state."""
    nav_kernel = load_nav_kernel_runtime(importer)
    if nav_kernel is None:
        return NavKernelPathFollowerAdapter(
            runtime=None,
            degraded_reason="compatible LingTu native navigation kernel missing",
            build_hint=build_hint(),
        )
    try:
        params, state = build_nav_kernel_path_follower(nav_kernel, config)
    except Exception as exc:
        return NavKernelPathFollowerAdapter(
            runtime=None,
            degraded_reason=f"nav_kernel init failed: {exc}",
        )
    return NavKernelPathFollowerAdapter(runtime=nav_kernel, params=params, state=state)


def create_nav_kernel_path_follower_adapter_from_tuning(
    *,
    max_speed: float,
    lookahead: float,
    goal_tolerance: float,
    max_yaw_rate: float | None,
    turn_speed_yaw_rate_start: float,
    turn_speed_min_scale: float,
    yaw_rate_gain: float,
    stop_yaw_rate_gain: float,
    dir_diff_thre: float,
    two_way_drive: bool,
    native_max_accel: float = 1.0,
    importer: Callable[[tuple[str, ...]], Any | None] = try_import_nav_kernel,
    build_hint: Callable[[], str] = nav_kernel_build_hint,
) -> NavKernelPathFollowerAdapter:
    """Create a nav_kernel adapter from Module tuning values."""
    return create_nav_kernel_path_follower_adapter(
        NavKernelPathFollowerConfig(
            max_speed=max_speed,
            lookahead=lookahead,
            goal_tolerance=goal_tolerance,
            max_yaw_rate=max_yaw_rate,
            turn_speed_yaw_rate_start=turn_speed_yaw_rate_start,
            turn_speed_min_scale=turn_speed_min_scale,
            yaw_rate_gain=yaw_rate_gain,
            stop_yaw_rate_gain=stop_yaw_rate_gain,
            dir_diff_thre=dir_diff_thre,
            two_way_drive=two_way_drive,
            max_accel=native_max_accel,
        ),
        importer=importer,
        build_hint=build_hint,
    )


def read_pid_fallback_params(
    max_speed: float,
    get_config_func: Callable[[], Any] | None = None,
) -> PidFallbackParams:
    """Read adaptive pure-pursuit fallback params, preserving old defaults."""
    params = PidFallbackParams(
        k_v=0.5,
        l_min=0.5,
        l_max=2.0,
        a_max=1.0,
        v_max=float(max_speed),
    )
    try:
        if get_config_func is None:
            from runtime.config import get_config

            get_config_func = get_config
        cfg = get_config_func()
        pf = cfg.raw.get("path_follower", {}) if hasattr(cfg, "raw") else {}
        if not pf:
            return params
        return PidFallbackParams(
            k_v=float(pf.get("k_v", params.k_v)),
            l_min=float(pf.get("L_min", params.l_min)),
            l_max=float(pf.get("L_max", params.l_max)),
            a_max=float(pf.get("a_max", params.a_max)),
            v_max=float(pf.get("v_max", params.v_max)),
            loaded_from_config=True,
        )
    except (ImportError, AttributeError, KeyError):
        return params
