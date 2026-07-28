"""Runtime Graph contract helpers.

The Runtime Graph describes product and endpoint wiring contracts. It is not a
runtime data plane and must not start sensors, SLAM, navigation, or drivers.
"""

from .loader import RuntimeGraph, load_runtime_graph
from .processes import ProcessSpec, resolve_processes
from .render import render_endpoint_mermaid, render_product_markdown
from .validator import (
    RuntimeGraphIssue,
    assert_runtime_graph_valid,
    validate_runtime_graph,
)

__all__ = [
    "ProcessSpec",
    "RuntimeGraph",
    "RuntimeGraphIssue",
    "assert_runtime_graph_valid",
    "load_runtime_graph",
    "render_endpoint_mermaid",
    "render_product_markdown",
    "resolve_processes",
    "validate_runtime_graph",
]
