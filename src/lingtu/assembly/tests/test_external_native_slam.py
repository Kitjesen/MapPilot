from __future__ import annotations

import sys

import pytest

from lingtu.assembly.wires import full_stack as full_stack_wires
from lingtu.assembly.compiler import compile_run_plan
from lingtu.assembly.plugins import BASE_PLUGIN_MODULES
from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.stacks.slam import slam
from lingtu.assembly.wires.full_stack import full_stack_wire_specs
from runtime.blueprint import Blueprint
from runtime.module import Module
from runtime.stream import In, Out
from runtime.wiring import WireSpec


class _WireSource(Module):
    output: Out[dict]


class _WireSink(Module):
    input: In[dict]


def test_slam_stack_requires_explicit_external_adapter() -> None:
    sys.modules.pop("localization.slam.module", None)

    with pytest.raises(ValueError, match="requires an explicit localization_adapter"):
        slam("native_dds")

    assert "localization.slam.module" not in sys.modules


def test_none_profile_stays_empty_even_when_env_has_default_adapter() -> None:
    assert not slam("none", localization_adapter="cpp_slam_status")._entries


def test_product_graphs_expose_only_external_slam_adapter() -> None:
    teleop = resolve_product_host_runtime("teleop", "real", robot="unitree/go2")
    nav = resolve_product_host_runtime("nav", "real", robot="unitree/go2")
    teleop_modules = set(
        compile_run_plan(teleop.product, teleop.env, robot="unitree/go2").modules
    )
    nav_modules = set(
        compile_run_plan(nav.product, nav.env, robot="unitree/go2").modules
    )

    assert "SlamAdapterModule" not in teleop_modules
    assert "SlamModule" not in teleop_modules
    assert "SlamAdapterModule" in nav_modules
    assert "SlamModule" not in nav_modules


def test_full_stack_has_no_python_sensor_feeds_into_slam_adapter() -> None:
    specs = full_stack_wire_specs(
        {"lidar", "imu", "gnss", "SlamAdapterModule", "GatewayModule"},
        driver_module="",
        slam_profile="native_dds",
        enable_semantic=False,
    )

    assert not {
        (spec.out_module, spec.out_port, spec.in_port)
        for spec in specs
        if spec.in_module == "SlamAdapterModule" and spec.out_module in {"lidar", "imu", "gnss"}
    }


def test_full_stack_rejects_a_selected_wire_with_a_missing_port(monkeypatch) -> None:
    bp = Blueprint().add(_WireSource, alias="source").add(_WireSink, alias="sink")
    invalid = WireSpec("source", "missing", "sink", "input")
    monkeypatch.setattr(
        full_stack_wires,
        "full_stack_wire_specs",
        lambda *_args, **_kwargs: (invalid,),
    )

    with pytest.raises(
        ValueError,
        match=r"invalid wire source\.missing->sink\.input: missing source port source\.missing",
    ):
        full_stack_wires.apply_full_stack_wires(
            bp,
            driver_module="",
            slam_profile="none",
        )


def test_builtin_slam_plugins_do_not_seed_managed_slam_module() -> None:
    assert "localization.slam.module" not in BASE_PLUGIN_MODULES["slam"]
