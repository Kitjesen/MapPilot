from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from nav.local.path_follower_backend import (
    NAV_KERNEL_PATH_FOLLOWER_SYMBOLS,
    NavKernelPathFollowerConfig,
    create_nav_kernel_path_follower_adapter,
    create_nav_kernel_path_follower_adapter_from_tuning,
    read_pid_fallback_params,
)


class FakePathFollowerParams:
    def __init__(self) -> None:
        self.turn_speed_yaw_rate_start = 0.0
        self.turn_speed_min_scale = 1.0


class FakePathFollowerState:
    def __init__(self) -> None:
        self.vehicle_speed = 0.0
        self.nav_fwd = True


class FakeNavKernel:
    PathFollowerParams = FakePathFollowerParams
    PathFollowerState = FakePathFollowerState

    @staticmethod
    def compute_control(*args, **kwargs):
        return None


def _config(**overrides) -> NavKernelPathFollowerConfig:
    values = {
        "max_speed": 0.4,
        "lookahead": 1.5,
        "goal_tolerance": 0.2,
        "max_yaw_rate": 0.8,
        "turn_speed_yaw_rate_start": 0.3,
        "turn_speed_min_scale": 0.6,
        "yaw_rate_gain": 7.5,
        "stop_yaw_rate_gain": 6.5,
        "dir_diff_thre": 0.1,
        "two_way_drive": True,
    }
    values.update(overrides)
    return NavKernelPathFollowerConfig(**values)


def test_nav_kernel_adapter_assembles_fake_runtime_params() -> None:
    seen_symbols: list[tuple[str, ...]] = []

    def importer(symbols: tuple[str, ...]):
        seen_symbols.append(symbols)
        return FakeNavKernel

    adapter = create_nav_kernel_path_follower_adapter(_config(), importer=importer)

    assert seen_symbols == [NAV_KERNEL_PATH_FOLLOWER_SYMBOLS]
    assert adapter.runtime is FakeNavKernel
    assert isinstance(adapter.params, FakePathFollowerParams)
    assert isinstance(adapter.state, FakePathFollowerState)
    assert adapter.degraded_reason is None
    assert adapter.params.max_speed == 0.4
    assert adapter.params.stop_dis_thre == 0.2
    assert adapter.params.base_look_ahead_dis == pytest.approx(0.3)
    assert adapter.params.min_look_ahead_dis == 0.25
    assert adapter.params.max_look_ahead_dis == 1.5
    assert adapter.params.look_ahead_ratio == 0.5
    assert adapter.params.yaw_rate_gain == 7.5
    assert adapter.params.stop_yaw_rate_gain == 6.5
    assert adapter.params.max_yaw_rate == math.degrees(0.8)
    assert adapter.params.turn_speed_yaw_rate_start == 0.3
    assert adapter.params.turn_speed_min_scale == 0.6
    assert adapter.params.switch_time_thre == 1.0
    assert adapter.params.dir_diff_thre == 0.1
    assert adapter.params.omni_dir_goal_thre == 1.0
    assert adapter.params.omni_dir_diff_thre == 1.5
    assert adapter.params.slow_dwn_dis_thre == 1.0
    assert adapter.params.two_way_drive is True
    assert adapter.params.no_rot_at_goal is True


def test_nav_kernel_adapter_reports_missing_runtime_degraded_info() -> None:
    adapter = create_nav_kernel_path_follower_adapter(
        _config(),
        importer=lambda symbols: None,
        build_hint=lambda: "build nav core",
    )

    assert adapter.runtime is None
    assert adapter.params is None
    assert adapter.state is None
    assert adapter.degraded_reason == "compatible LingTu native navigation kernel missing"
    assert adapter.build_hint == "build nav core"


def test_nav_kernel_adapter_from_tuning_builds_runtime_config() -> None:
    seen_symbols: list[tuple[str, ...]] = []

    def importer(symbols: tuple[str, ...]):
        seen_symbols.append(symbols)
        return FakeNavKernel

    adapter = create_nav_kernel_path_follower_adapter_from_tuning(
        max_speed=0.6,
        lookahead=1.8,
        goal_tolerance=0.3,
        max_yaw_rate=0.5,
        turn_speed_yaw_rate_start=0.2,
        turn_speed_min_scale=0.55,
        yaw_rate_gain=6.0,
        stop_yaw_rate_gain=5.0,
        dir_diff_thre=0.12,
        two_way_drive=False,
        importer=importer,
        build_hint=lambda: "build nav core",
    )

    assert seen_symbols == [NAV_KERNEL_PATH_FOLLOWER_SYMBOLS]
    assert adapter.runtime is FakeNavKernel
    assert adapter.params.max_speed == 0.6
    assert adapter.params.stop_dis_thre == 0.3
    assert adapter.params.max_look_ahead_dis == 1.8
    assert adapter.params.max_yaw_rate == math.degrees(0.5)
    assert adapter.params.turn_speed_yaw_rate_start == 0.2
    assert adapter.params.turn_speed_min_scale == 0.55
    assert adapter.params.yaw_rate_gain == 6.0
    assert adapter.params.stop_yaw_rate_gain == 5.0
    assert adapter.params.dir_diff_thre == 0.12
    assert adapter.params.two_way_drive is False


def test_pid_fallback_params_keep_defaults_without_config() -> None:
    params = read_pid_fallback_params(0.9, get_config_func=lambda: SimpleNamespace())

    assert params.k_v == 0.5
    assert params.l_min == 0.5
    assert params.l_max == 2.0
    assert params.a_max == 1.0
    assert params.v_max == 0.9
    assert params.loaded_from_config is False


def test_pid_fallback_params_read_path_follower_config() -> None:
    cfg = SimpleNamespace(
        raw={
            "path_follower": {
                "k_v": "0.7",
                "L_min": "0.4",
                "L_max": "2.4",
                "a_max": "1.2",
                "v_max": "0.8",
            }
        }
    )

    params = read_pid_fallback_params(0.9, get_config_func=lambda: cfg)

    assert params.k_v == 0.7
    assert params.l_min == 0.4
    assert params.l_max == 2.4
    assert params.a_max == 1.2
    assert params.v_max == 0.8
    assert params.loaded_from_config is True
