"""Navigation IO adapter composition."""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.blueprints.wires.context import (
    NAV_IN,
    NAV_OUT,
)
from runtime.blueprints.wires.navigation import (
    navigation_io_input_specs,
    navigation_output_specs,
)
from runtime.blueprints.wires.types import wire_present_specs
from runtime.blueprints.stacks._registry import optional_stack_module, stack_module
from runtime.profiles.binding_policy import (
    endpoint_contract_for_config,
    navigation_input_adapter_enabled,
    navigation_input_uses_dds,
    navigation_input_uses_lcm,
    navigation_input_uses_ros2,
    navigation_output_adapter_enabled,
    navigation_output_uses_dds,
    navigation_output_uses_lcm,
    navigation_output_uses_ros2,
)

logger = logging.getLogger(__name__)

def _nav_out_adapter_spec(config: dict) -> tuple[str, str] | None:
    if navigation_output_uses_lcm(config):
        return (
            "lcm_nav_output",
            "runtime.adapters.lcm.nav_output.LCMNavOutModule",
        )
    if navigation_output_uses_dds(config):
        return (
            "dds_nav_output",
            "runtime.adapters.dds.nav.DDSNavOutModule",
        )
    if not navigation_output_uses_ros2(config):
        return None
    return (
        "ros2_nav_output",
        "nav.adapters.ros2.nav.nav_out.ROS2NavOutModule",
    )


def _nav_in_adapter_spec(config: dict) -> tuple[str, str] | None:
    if navigation_input_uses_lcm(config):
        return (
            "lcm_nav_input",
            "runtime.adapters.lcm.nav_input.LCMNavInModule",
        )
    if navigation_input_uses_dds(config):
        return (
            "dds_nav_input",
            "runtime.adapters.dds.nav.DDSNavInModule",
        )
    if not navigation_input_uses_ros2(config):
        return None
    return (
        "ros2_nav_input",
        "nav.adapters.ros2.nav.nav_in.ROS2NavInModule",
    )


def _nav_adapter_seed_group(adapter_name: str) -> str:
    if adapter_name.startswith("lcm_"):
        return "navigation_lcm"
    if adapter_name.startswith("dds_"):
        return "navigation_dds"
    return "navigation_ros2"


def _nav_in_adapter_module(adapter_name: str, fallback: str) -> type:
    try:
        return stack_module(
            "navigation",
            adapter_name,
            seed_group=_nav_adapter_seed_group(adapter_name),
            fallback=fallback,
        )
    except (ImportError, AttributeError, KeyError) as exc:
        raise RuntimeError(
            "Navigation input adapter is enabled, but adapter "
            f"'{adapter_name}' is not available"
        ) from exc


def wire_navigation_output_adapter(bp: Blueprint) -> Blueprint:
    wire_present_specs(bp, navigation_output_specs())
    return bp


def add_navigation_output_adapter(bp: Blueprint, **config) -> Blueprint:
    """Add only the navigation output adapter.

    This is used by product modes that do not run NavigationModule but still
    need a command/path egress boundary, for example teleop and mapping.
    """

    if not navigation_output_adapter_enabled(config):
        return bp
    adapter_spec = _nav_out_adapter_spec(config)
    if adapter_spec is None:
        logger.warning(
            "Navigation output adapter requested without an explicit adapter; "
            "skipping instead of selecting a ROS2 adapter by default"
        )
        return bp

    adapter_name, adapter_fallback = adapter_spec
    NavOutModule = optional_stack_module(
        "navigation",
        adapter_name,
        seed_group=_nav_adapter_seed_group(adapter_name),
        fallback=adapter_fallback,
    )
    if NavOutModule is None:
        logger.warning("Navigation output adapter not available")
        return bp

    nav_out_config = {}
    if "planning_frame_id" in config:
        nav_out_config["default_frame_id"] = config["planning_frame_id"]
    if adapter_name == "lcm_nav_output":
        endpoint_contract = endpoint_contract_for_config(config)
        if endpoint_contract:
            nav_out_config["endpoint_contract"] = endpoint_contract
    bp.add(
        NavOutModule,
        alias=NAV_OUT,
        **nav_out_config,
    )
    return bp


def add_navigation_io_adapters(bp: Blueprint, **config) -> Blueprint:
    """Add optional navigation input/output adapters."""

    if navigation_input_adapter_enabled(config):
        adapter_spec = _nav_in_adapter_spec(config)
        if adapter_spec is None:
            logger.warning(
                "Navigation input adapter requested without an explicit adapter; "
                "skipping instead of selecting a ROS2 adapter by default"
            )
        else:
            adapter_name, adapter_fallback = adapter_spec
            NavInModule = _nav_in_adapter_module(
                adapter_name,
                adapter_fallback,
            )
            nav_in_config = {}
            if "planning_frame_id" in config:
                nav_in_config["default_frame_id"] = config["planning_frame_id"]
            if adapter_name == "lcm_nav_input":
                endpoint_contract = endpoint_contract_for_config(config)
                if endpoint_contract:
                    nav_in_config["endpoint_contract"] = endpoint_contract
            bp.add(
                NavInModule,
                alias=NAV_IN,
                **nav_in_config,
            )
            wire_present_specs(bp, navigation_io_input_specs())

    add_navigation_output_adapter(bp, **config)
    wire_navigation_output_adapter(bp)

    return bp
