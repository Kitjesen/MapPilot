"""Render route contracts for diagnostics and docs."""

from __future__ import annotations

from .model import RouteContract


def render_route_mermaid(contract: RouteContract) -> str:
    """Render a route contract as a Mermaid topic ownership graph."""

    lines = ["flowchart LR"]
    for topic, spec in sorted(contract.topics.items()):
        topic_node = _node("topic", topic)
        route = contract.route_for(topic)
        label = f"{topic}<br/>{spec.schema}<br/>route={route}"
        producer_node = _node("producer", spec.producer)
        lines.append(f'  {producer_node}["{spec.producer}"] --> {topic_node}["{label}"]')
        for consumer in spec.consumers:
            consumer_node = _node("consumer", consumer)
            lines.append(f'  {topic_node} --> {consumer_node}["{consumer}"]')
        for binding in spec.port_bindings:
            port_node = _node("port", f"{binding.owner}.{binding.port}.{binding.boundary}")
            port_label = f"{binding.owner}.{binding.port}<br/>{binding.boundary}"
            if binding.direction == "out":
                lines.append(f'  {port_node}["{port_label}"] --> {topic_node}')
            else:
                lines.append(f'  {topic_node} --> {port_node}["{port_label}"]')
    lines.append("  classDef dds fill:#e7f0ff,stroke:#315fbd,color:#102e61")
    lines.append("  classDef lcm fill:#fff4dd,stroke:#b46b00,color:#4a2a00")
    lines.append("  classDef endpoint fill:#e9f8ef,stroke:#257942,color:#123d24")
    for topic in sorted(contract.topics):
        route = contract.route_for(topic)
        if route in {"dds", "lcm"}:
            lines.append(f"  class {_node('topic', topic)} {route}")
        for binding in contract.topic(topic).port_bindings:
            if binding.boundary == "endpoint":
                lines.append(f"  class {_node('port', f'{binding.owner}.{binding.port}.{binding.boundary}')} endpoint")
    return "\n".join(lines)


def _node(prefix: str, value: str) -> str:
    return f"{prefix}_" + str(value).strip("/").replace("/", "_").replace("-", "_").replace(".", "_").replace(":", "_")
