"""Render Runtime Graph contracts for docs and diagnostics."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from runtime.contracts.product_runtime import resolve_product_spec_contracts

from .loader import RuntimeGraph, load_runtime_graph
from .processes import resolve_env_implementation


def render_env_mermaid(
    env_name: str,
    graph: RuntimeGraph | None = None,
    *,
    env_config: Mapping[str, Any] | None = None,
) -> str:
    """Render one selected Env implementation as a Mermaid flowchart."""

    graph = graph or load_runtime_graph()
    implementation = resolve_env_implementation(
        env_name,
        graph=graph,
        env_config=env_config,
    )
    endpoints = implementation.get("endpoints")
    endpoint = endpoints.get("contract") if isinstance(endpoints, Mapping) else None
    if not isinstance(endpoint, Mapping):
        raise ValueError(
            f"Runtime Graph env {env_name!r} implementation must declare "
            "endpoints.contract"
        )
    lines = ["flowchart LR"]
    lines.append(f'  env["{env_name}"]')
    backend = ""
    if isinstance(env_config, Mapping):
        backend = str(env_config.get("backend") or "").strip()
    if backend:
        lines.append(f'  backend["{backend}"] --> env')

    for topic in endpoint.get("source_topics", ()) or ():
        node = _topic_node(topic)
        lines.append(f'  {node}["{topic}"] --> env')
    for topic in endpoint.get("exposed_topics", ()) or ():
        node = _topic_node(topic)
        lines.append(f'  env --> {node}["{topic}"]')
    if endpoint.get("real_equivalent") is True:
        lines.append('  classDef native fill:#e7f7ed,stroke:#1f8f45,color:#0b3d1f')
        lines.append("  class env native")
    elif endpoint.get("runtime_class") == "host_simulation":
        lines.append('  classDef simulation fill:#fff4dd,stroke:#b46b00,color:#4a2a00')
        lines.append("  class env simulation")
    return "\n".join(lines)


def render_product_markdown(product_name: str, graph: RuntimeGraph | None = None) -> str:
    """Render a product Runtime Graph contract as Markdown."""

    graph = graph or load_runtime_graph()
    product = graph.products[product_name]
    contract = resolve_product_spec_contracts(product_name, product)
    lines = [
        f"# {product_name}",
        "",
        str(product.get("summary") or ""),
        "",
        "## Required Topics",
    ]
    lines.extend(f"- `{topic}`" for topic in contract.topics)
    lines.append("")
    lines.append("## Required Capabilities")
    lines.extend(
        f"- `{capability}`"
        for capability in contract.capabilities
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
