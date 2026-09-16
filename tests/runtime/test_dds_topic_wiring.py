from __future__ import annotations

from lingtu.assembly.graph import load_runtime_graph, resolve_env_implementation
from message.topics import topic_spec


def test_real_and_sim_endpoints_use_catalogued_dds_types() -> None:
    graph = load_runtime_graph()
    real = resolve_env_implementation("real", graph=graph)
    sim = resolve_env_implementation("sim", graph=graph, env_config={"backend": "mujoco"})
    for implementation in (real, sim):
        assert implementation["host_config"]["_endpoint_transport"] == "dds"
        endpoint = implementation["endpoints"]["contract"]
        assert endpoint["transport"] == "dds"
        for topic in set(endpoint["source_topics"]) | set(endpoint["exposed_topics"]):
            spec = topic_spec(topic)
            assert spec is not None, topic
            assert spec.message_type == graph.topic_contracts[topic]["message_type"]
