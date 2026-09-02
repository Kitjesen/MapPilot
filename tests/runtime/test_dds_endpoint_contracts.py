from __future__ import annotations

from message.topics import dds_topic_name, topic_spec
from runtime.endpoints.dds.contracts import FIELD_DDS_CONTRACT, FIELD_DDS_CONTRACT_NAME
from runtime.graph import load_runtime_graph, resolve_env_implementation
from runtime.runtime_interface import TOPICS


def test_real_and_sim_use_the_native_dds_endpoint_contract() -> None:
    graph = load_runtime_graph()
    real = resolve_env_implementation("real", graph=graph)
    sim = resolve_env_implementation("sim", graph=graph, env_config={"backend": "mujoco"})
    for implementation in (real, sim):
        host = implementation["host_config"]
        assert host["_endpoint_transport"] == "dds"
        assert host["_endpoint_contract"] == FIELD_DDS_CONTRACT_NAME


def test_endpoint_bindings_are_native_type_metadata() -> None:
    for binding in FIELD_DDS_CONTRACT.bindings:
        spec = topic_spec(binding.topic)
        assert spec is not None
        assert binding.channel == dds_topic_name(binding.topic)
        assert binding.type_name == spec.type_name
        assert binding.idl_type == spec.idl_type
        assert not hasattr(binding, "import_path")


def test_motion_and_localization_topics_have_explicit_native_bindings() -> None:
    required = {
        TOPICS.odometry,
        TOPICS.localization_health,
        TOPICS.nav_command_request,
        TOPICS.nav_command_ack,
        TOPICS.operator_motion_control,
        TOPICS.operator_motion_ack,
        TOPICS.nav_state,
        TOPICS.cmd_vel,
    }
    assert required <= set(FIELD_DDS_CONTRACT.topics)
    assert FIELD_DDS_CONTRACT.binding_for_topic(TOPICS.cmd_vel).idl_type == (
        "lingtu.dds.FinalVelocityCommand"
    )
