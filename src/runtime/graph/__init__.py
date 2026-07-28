"""Runtime Graph contract helpers.

The Runtime Graph describes product and endpoint wiring contracts. It is not a
runtime data plane and must not start sensors, SLAM, navigation, or drivers.
"""

from .loader import RuntimeGraph, load_runtime_graph
from .plan import RuntimePlan, RuntimeProcess, build_runtime_plan
from .render import render_endpoint_mermaid, render_product_markdown
from .validator import (
    RuntimeGraphIssue,
    assert_runtime_graph_valid,
    validate_runtime_graph,
)

__all__ = [
    "RuntimeGraph",
    "RuntimeGraphIssue",
    "RuntimePlan",
    "RuntimeProcess",
    "assert_runtime_graph_valid",
    "build_runtime_plan",
    "load_runtime_graph",
    "render_endpoint_mermaid",
    "render_product_markdown",
    "validate_runtime_graph",
]
