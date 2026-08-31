"""Runtime Graph contract helpers.

The Runtime Graph describes Product and Env wiring contracts. It is not a
runtime data plane and must not start sensors, SLAM, navigation, or drivers.
"""

from .loader import RuntimeGraph, load_runtime_graph
from .processes import (
    ProcessArtifact,
    ProcessCommand,
    ProcessReadiness,
    ProcessShutdown,
    ProcessSpec,
    resolve_env_implementation,
    resolve_processes,
    resolve_stop_before_start,
    valid_process_name,
)
from .render import render_env_mermaid, render_product_markdown
from .validator import (
    RuntimeGraphIssue,
    assert_runtime_graph_valid,
    validate_runtime_graph,
)

__all__ = [
    "ProcessArtifact",
    "ProcessCommand",
    "ProcessReadiness",
    "ProcessShutdown",
    "ProcessSpec",
    "RuntimeGraph",
    "RuntimeGraphIssue",
    "assert_runtime_graph_valid",
    "load_runtime_graph",
    "render_env_mermaid",
    "render_product_markdown",
    "resolve_env_implementation",
    "resolve_processes",
    "resolve_stop_before_start",
    "valid_process_name",
    "validate_runtime_graph",
]
