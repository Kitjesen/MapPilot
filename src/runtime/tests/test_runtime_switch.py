"""Local Profile adapter RuntimeRunSpec and validation contracts."""

from __future__ import annotations

import json
from dataclasses import asdict

import pytest

from runtime.profiles.catalog.host_defaults import HOST_PROFILE_DEFAULTS
from runtime.profiles.profile_adapters import (
    PROFILE_ADAPTERS,
    ProfileAdapterError,
    resolve_runtime_run_spec,
)
from runtime.profiles.resolver import resolve_profile_config
from runtime.runtime_interface import (
    DATA_SOURCE_CONTRACTS,
    FRAME_LINKS,
    FRAMES,
    PROFILE_DATA_SOURCE_BINDINGS,
    RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES,
    TOPICS,
    resolved_runtime_data_flow,
    runtime_topic_allowed_frame_ids,
    runtime_topic_default_frame_ids,
)
from runtime.runtime_switch import (
    compare_runtime_switch,
    runtime_spec_summary,
    validate_runtime_switch,
)

pytestmark = pytest.mark.sim


def _resolve_profile_spec(
    profile: str,
    *,
    adapter: str | None = None,
    include_profile_metadata: bool = False,
    overrides: dict[str, object] | None = None,
    record: bool = False,
    extra_args: tuple[str, ...] = (),
):
    config = resolve_profile_config(
        profile,
        profile_adapter=adapter,
        include_profile_metadata=include_profile_metadata,
        overrides=overrides,
    )
    return resolve_runtime_run_spec(
        profile,
        config,
        record=record,
        extra_args=extra_args,
    )


def test_all_local_profiles_resolve_to_valid_runtime_specs() -> None:
    assert set(HOST_PROFILE_DEFAULTS) == set(PROFILE_DATA_SOURCE_BINDINGS)

    expected_frames = asdict(FRAMES)
    expected_frame_links = {name: asdict(link) for name, link in FRAME_LINKS.items()}
    expected_stage_interfaces = {
        name: tuple(interfaces) for name, interfaces in RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES.items()
    }

    for profile, binding in PROFILE_DATA_SOURCE_BINDINGS.items():
        spec = _resolve_profile_spec(profile)
        source = DATA_SOURCE_CONTRACTS[binding.data_source]
        contract_name = spec.runtime_contract or spec.data_source

        assert spec.profile == profile
        assert spec.data_source == binding.data_source
        assert spec.module_transport == "local"
        assert spec.simulation_only is (source.provider != "hardware")
        assert spec.command_sink == source.command_sink
        assert spec.frames == expected_frames
        assert spec.frame_links == expected_frame_links
        assert spec.topic_allowed_frame_ids == runtime_topic_allowed_frame_ids(contract_name)
        assert spec.topic_default_frame_ids == runtime_topic_default_frame_ids(contract_name)
        assert spec.resolved_runtime_data_flow == tuple(
            asdict(stage) for stage in resolved_runtime_data_flow(spec.data_source)
        )
        assert spec.runtime_data_flow_stage_algorithm_interfaces == (expected_stage_interfaces)
        assert spec.env["LINGTU_PROFILE"] == profile
        assert spec.env["LINGTU_DATA_SOURCE"] == spec.data_source
        assert spec.env["LINGTU_MODULE_TRANSPORT"] == spec.module_transport
        assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == spec.endpoint_transport
        assert spec.env["LINGTU_COMMAND_SINK"] == spec.command_sink
        assert validate_runtime_switch(spec).ok is True, profile


def test_profile_adapter_catalog_only_references_local_profiles() -> None:
    for adapter_name, adapter in PROFILE_ADAPTERS.items():
        supported_profiles = set(adapter.supported_profiles)

        assert supported_profiles
        assert supported_profiles <= set(HOST_PROFILE_DEFAULTS), adapter_name
        assert adapter.data_source in DATA_SOURCE_CONTRACTS, adapter_name
        assert set(adapter.profile_overrides) <= supported_profiles, adapter_name
        assert set(adapter.default_actions) <= supported_profiles, adapter_name
        assert set(adapter.record_actions) <= supported_profiles, adapter_name


def test_every_profile_adapter_resolves_each_supported_profile() -> None:
    for adapter_name, adapter in PROFILE_ADAPTERS.items():
        for profile in adapter.supported_profiles:
            spec = _resolve_profile_spec(
                profile,
                adapter=adapter_name,
                include_profile_metadata=True,
            )

            assert spec.adapter == adapter_name
            assert spec.data_source == adapter.data_source
            assert spec.runtime_contract == adapter.runtime_contract
            assert spec.module_transport == adapter.module_transport
            assert spec.endpoint_transport == adapter.endpoint_transport
            assert spec.endpoint_contract == adapter.endpoint_contract
            assert spec.simulation_only is adapter.simulation_only
            assert spec.env["LINGTU_PROFILE_ADAPTER"] == adapter_name
            assert spec.env["LINGTU_DATA_SOURCE"] == adapter.data_source
            assert spec.env["LINGTU_MODULE_TRANSPORT"] == adapter.module_transport
            assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == adapter.endpoint_transport
            assert validate_runtime_switch(spec).ok is True, (
                adapter_name,
                profile,
            )


@pytest.mark.parametrize(
    ("profile", "data_source", "command_sink"),
    (
        ("sim", "mujoco_module_graph", "mujoco_driver_module_cmd_vel"),
        (
            "portable_mujoco",
            "mujoco_module_graph",
            "mujoco_driver_module_cmd_vel",
        ),
        ("sim_nav", "in_process_stub", "module_graph_driver_cmd_vel"),
        ("dev", "in_process_stub", "module_graph_driver_cmd_vel"),
        ("stub", "in_process_stub", "module_graph_driver_cmd_vel"),
    ),
)
def test_in_process_profiles_have_no_profile_adapter_identity(
    profile: str,
    data_source: str,
    command_sink: str,
) -> None:
    spec = _resolve_profile_spec(profile)

    assert spec.adapter is None
    assert spec.data_source == data_source
    assert spec.runtime_contract is None
    assert spec.command_sink == command_sink
    assert spec.launcher is None
    assert spec.launcher_args == ()
    assert "LINGTU_PROFILE_ADAPTER" not in spec.env
    assert "LINGTU_RUNTIME_CONTRACT" not in spec.env
    assert validate_runtime_switch(spec).ok is True


def test_lite_profile_uses_its_default_hardware_adapter() -> None:
    spec = _resolve_profile_spec("lite")

    assert spec.adapter == "thunder_lite"
    assert spec.data_source == "thunder_lite_local"
    assert spec.runtime_contract == "thunder_lite_local"
    assert spec.simulation_only is False
    assert spec.command_sink == "driver"
    assert spec.endpoint_transport == "local"
    assert spec.endpoint_contract is None
    assert spec.env["LINGTU_PROFILE_ADAPTER"] == "thunder_lite"
    assert spec.env["LINGTU_SIMULATION_ONLY"] == "0"
    assert validate_runtime_switch(spec).ok is True


@pytest.mark.parametrize(
    ("profile", "record", "expected_args"),
    (
        ("sim_mujoco_live", False, ("gate",)),
        ("sim_mujoco_live", True, ("video",)),
        (
            "sim_mujoco_octo_live",
            False,
            ("octo-moving-obstacle-video",),
        ),
        (
            "sim_mujoco_octo_live",
            True,
            ("octo-moving-obstacle-video",),
        ),
    ),
)
def test_mujoco_profile_adapter_resolves_launcher_actions(
    profile: str,
    record: bool,
    expected_args: tuple[str, ...],
) -> None:
    spec = _resolve_profile_spec(
        profile,
        include_profile_metadata=True,
        record=record,
    )

    assert spec.adapter == "mujoco_live"
    assert spec.launcher == "sim/scripts/mujoco/launch_fastlio2_live.sh"
    assert spec.launcher_args == expected_args
    assert spec.as_command() == [
        "bash",
        "sim/scripts/mujoco/launch_fastlio2_live.sh",
        *expected_args,
    ]


def test_explicit_launcher_args_override_profile_adapter_actions() -> None:
    spec = _resolve_profile_spec(
        "sim_mujoco_live",
        include_profile_metadata=True,
        record=True,
        extra_args=("status",),
    )

    assert spec.launcher_args == ("status",)


def test_profile_adapter_rejects_an_unsupported_local_profile() -> None:
    with pytest.raises(
        ProfileAdapterError,
        match="does not support profile 'sim_nav'",
    ):
        resolve_profile_config(
            "sim_nav",
            profile_adapter="mujoco_live",
        )


def test_runtime_run_spec_rejects_product_selection_metadata() -> None:
    config = resolve_profile_config("sim_nav")
    config["_selection_kind"] = "product"

    with pytest.raises(
        ProfileAdapterError,
        match="Product RunPlans cannot be resolved through Profile adapters",
    ):
        resolve_runtime_run_spec("sim_nav", config)


def test_runtime_spec_summary_uses_profile_adapter_vocabulary_and_json_values() -> None:
    spec = _resolve_profile_spec("sim_mujoco_live")

    summary = runtime_spec_summary(spec)
    json.dumps(summary)

    assert summary["profile"] == "sim_mujoco_live"
    assert summary["adapter"] == "mujoco_live"
    assert "endpoint" not in summary
    assert summary["topic_allowed_frame_ids"][TOPICS.map_cloud] == [
        "map",
        "odom",
    ]
    assert isinstance(summary["resolved_runtime_data_flow"], list)
    assert summary["profile_semantic_overrides"] == []
    assert "product_semantic_overrides" not in summary
    assert summary["validation"] == {
        "ok": True,
        "blockers": [],
        "warnings": [],
    }


def test_compare_runtime_specs_uses_profile_adapter_identity() -> None:
    adapted = _resolve_profile_spec("sim_mujoco_live")
    in_process = _resolve_profile_spec("sim_nav")

    comparison = compare_runtime_switch(adapted, in_process)

    assert comparison["from"]["adapter"] == "mujoco_live"
    assert comparison["to"]["adapter"] == "in_process"
    assert "endpoint" not in comparison["from"]
    assert "adapter" in comparison["changed"]
    assert "data_source" in comparison["changed"]
    assert "runtime_contract" in comparison["changed"]
    assert "command_sink" in comparison["changed"]


def test_local_profile_can_resolve_a_typed_dds_endpoint_contract() -> None:
    spec = _resolve_profile_spec(
        "sim_nav",
        overrides={
            "endpoint_transport": "dds",
            "endpoint_contract": "thunder_dds_v1",
        },
    )

    assert spec.adapter is None
    assert spec.endpoint_transport == "dds"
    assert spec.endpoint_contract == "thunder_dds_v1"
    assert spec.route_contract == "robot"
    assert spec.localization_adapter == "dds_endpoint"
    assert spec.env["LINGTU_ENDPOINT_TRANSPORT"] == "dds"
    assert spec.env["LINGTU_ENDPOINT_CONTRACT"] == "thunder_dds_v1"
    assert spec.env["LINGTU_ROUTE_CONTRACT"] == "robot"
    assert spec.env["LINGTU_LOCALIZATION_ADAPTER"] == "dds_endpoint"
    assert validate_runtime_switch(spec).ok is True
