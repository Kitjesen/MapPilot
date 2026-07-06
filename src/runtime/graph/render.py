"""Render Runtime Graph contracts for docs and diagnostics."""

from __future__ import annotations

from .loader import RuntimeGraph, load_runtime_graph


def render_endpoint_mermaid(endpoint_name: str, graph: RuntimeGraph | None = None) -> str:
    """Render a Runtime Graph endpoint as a Mermaid flowchart."""

    graph = graph or load_runtime_graph()
    endpoint = graph.endpoints[endpoint_name]
    lines = ["flowchart LR"]
    lines.append(f'  endpoint["{endpoint_name}"]')

    for topic in endpoint.get("source_topics", ()) or ():
        node = _topic_node(topic)
        lines.append(f'  {node}["{topic}"] --> endpoint')
    for topic in endpoint.get("exposed_topics", ()) or ():
        node = _topic_node(topic)
        lines.append(f'  endpoint --> {node}["{topic}"]')
    if endpoint.get("real_equivalent") is True:
        lines.append('  classDef native fill:#e7f7ed,stroke:#1f8f45,color:#0b3d1f')
        lines.append("  class endpoint native")
    elif endpoint.get("runtime_class") == "module_sim_harness":
        lines.append('  classDef legacy fill:#fff4dd,stroke:#b46b00,color:#4a2a00')
        lines.append("  class endpoint legacy")
    return "\n".join(lines)


def render_product_markdown(product_name: str, graph: RuntimeGraph | None = None) -> str:
    """Render a product Runtime Graph contract as Markdown."""

    graph = graph or load_runtime_graph()
    product = graph.products[product_name]
    lines = [
        f"# {product_name}",
        "",
        str(product.get("summary") or ""),
        "",
        "## Required Topics",
    ]
    lines.extend(f"- `{topic}`" for topic in product.get("required_topics", ()) or ())
    lines.append("")
    lines.append("## Required Capabilities")
    lines.extend(
        f"- `{capability}`"
        for capability in product.get("required_capabilities", ()) or ()
    )
    forbidden = product.get("forbidden_modules", ()) or ()
    if forbidden:
        lines.append("")
        lines.append("## Forbidden Modules")
        lines.extend(f"- `{module}`" for module in forbidden)
    return "\n".join(lines)


def _topic_node(topic: str) -> str:
    return (
        "topic_"
        + topic.strip("/").replace("/", "_").replace("-", "_").replace(".", "_")
    )
