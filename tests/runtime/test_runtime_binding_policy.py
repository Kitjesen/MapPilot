from __future__ import annotations

from lingtu.assembly.binding_policy import (
    endpoint_contract_for_config,
    endpoint_transport_for_config,
    exploration_backend_for_config,
    localization_adapter_for_config,
)


def test_runtime_binding_policy_prefers_operator_transport_over_endpoint_default() -> None:
    config = {
        "_endpoint_transport": "local",
        "endpoint_transport": "dds",
    }

    assert endpoint_transport_for_config(config) == "dds"


def test_runtime_binding_policy_does_not_infer_removed_lcm_adapter() -> None:
    config = {
        "_endpoint_transport": "zmq",
        "_endpoint_contract": "custom_zmq_v1",
    }

    assert endpoint_contract_for_config(config) == "custom_zmq_v1"
    assert localization_adapter_for_config(config) == ""


def test_runtime_binding_policy_requires_explicit_localization_adapter() -> None:
    assert localization_adapter_for_config({"_endpoint_transport": "dds"}) == ""
    assert localization_adapter_for_config({"localization_adapter": "cpp_slam_status"}) == (
        "cpp_slam_status"
    )


def test_exploration_backend_for_config_defaults_to_none() -> None:
    assert exploration_backend_for_config({}) == "none"
    assert exploration_backend_for_config({"exploration_backend": "TARE"}) == "tare"
