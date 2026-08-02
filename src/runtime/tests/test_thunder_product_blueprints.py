from __future__ import annotations

import importlib

import pytest

from lingtu.assembly.products import (
    FIELD_PRODUCT_HOST_DEFAULTS,
    host_blueprint_for_product,
    thunder_blueprint,
)
from lingtu.plugin_seed import install_builtin_plugin_catalog

install_builtin_plugin_catalog()

_REMOVED_THUNDER_NAMES = (
    "thunder_basic_blueprint",
    "thunder_basic_config",
    "thunder_explore_blueprint",
    "thunder_explore_config",
    "thunder_lite_blueprint",
    "thunder_lite_config",
    "thunder_map_blueprint",
    "thunder_map_config",
    "thunder_nav_blueprint",
    "thunder_nav_config",
)


def _entry_names(bp) -> set[str]:
    return {entry.name for entry in bp._entries}


def _wire_set(bp) -> set[str]:
    return {f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}" for wire in bp._wires}


def _lite_host_config(**overrides):
    config = {
        "runtime_mode": "lite",
        "robot": "thunder",
        "slam_profile": "none",
        "llm": "mock",
        "planner": "direct",
        "enable_native": False,
        "python_autonomy_backend": "simple",
        "python_path_follower_backend": "pid",
        "enable_semantic": False,
        "enable_gateway": False,
        "enable_teleop": False,
        "enable_map_modules": False,
        "enable_gnss": False,
        "run_startup_checks": False,
    }
    config.update(overrides)
    return config


def test_field_product_host_defaults_do_not_enable_native_processes() -> None:
    assert FIELD_PRODUCT_HOST_DEFAULTS
    for product, config in FIELD_PRODUCT_HOST_DEFAULTS.items():
        assert config.get("enable_native") is False, product


def test_legacy_thunder_product_wrappers_are_not_exported() -> None:
    products = importlib.import_module("lingtu.assembly.products")
    thunder = importlib.import_module("lingtu.assembly.products.thunder")
    assembly = importlib.import_module("lingtu.assembly")

    for name in _REMOVED_THUNDER_NAMES:
        assert name not in products.__all__
        assert not hasattr(products, name)
        assert not hasattr(thunder, name)
        assert name not in assembly.__all__

    assert "stub_blueprint" not in assembly.__all__
    assert not hasattr(assembly, "stub_blueprint")


def test_generic_thunder_blueprint_accepts_resolved_lite_host_config() -> None:
    bp = thunder_blueprint(_lite_host_config())
    names = _entry_names(bp)
    wires = _wire_set(bp)

    assert {"ThunderDriver", "nav.mission", "nav.safety", "nav.velocity_mux"} <= names
    assert "SlamAdapterModule" not in names
    assert "SLAMModule" not in names
    assert "GatewayModule" not in names
    assert "maps.service" not in names
    assert "nav.safety.stop_cmd->ThunderDriver.stop_signal" in wires
    assert "nav.velocity_mux.driver_cmd_vel->ThunderDriver.cmd_vel" in wires


def test_product_host_blueprint_uses_generic_materializer() -> None:
    bp = host_blueprint_for_product("teleop", _lite_host_config())

    assert "ThunderDriver" in _entry_names(bp)


@pytest.mark.parametrize(
    "override",
    (
        {"slam_profile": "fastlio2"},
        {"enable_gateway": True},
        {"enable_semantic": True},
        {"enable_map_modules": True},
        {"enable_nav_in": True},
        {"enable_nav_out": True},
        {"enable_map_out": True},
        {"enable_ros2_rerun_bridge": True},
        {"run_startup_checks": True},
    ),
)
def test_lite_runtime_rejects_field_runtime_overrides(override) -> None:
    with pytest.raises(ValueError, match="lite Profile runtime"):
        thunder_blueprint(_lite_host_config(**override))
