"""core.introspection.dot — Graphviz DOT renderer for SystemHandle.

Adapts dimos introspection/blueprint/dot.py to LingTu's SystemHandle API.
Modules are grouped by layer (L0-L6). Active data channels appear as
intermediate hub nodes so fan-out is visually clear.

Usage::

    from core.introspection.dot import render, render_svg

    system = autoconnect(...).build()
    print(render(system))              # DOT string → pipe to graphviz
    render_svg(system, "graph.svg")    # write SVG file (requires graphviz CLI)
    render_png(system, "graph.png")    # write PNG file (requires graphviz CLI)
"""

from __future__ import annotations

import subprocess
from collections import defaultdict
from typing import Any

_LAYER_LABELS: dict[int | None, str] = {
    0: "L0 · Safety",
    1: "L1 · Driver",
    2: "L2 · Base Autonomy",
    3: "L3 · Perception",
    4: "L4 · Semantic",
    5: "L5 · Navigation",
    6: "L6 · Gateway",
}

_LAYER_COLORS: dict[int | None, str] = {
    0: "#ff6b6b",
    1: "#4ecdc4",
    2: "#45b7d1",
    3: "#96ceb4",
    4: "#ffeaa7",
    5: "#dda0dd",
    6: "#98d8c8",
    None: "#cccccc",
}

_CHAN_COLORS = [
    "#e17055", "#74b9ff", "#a29bfe", "#fd79a8",
    "#55efc4", "#fdcb6e", "#6c5ce7", "#00cec9",
    "#e84393", "#fab1a0",
]


def _color_for(palette: list, key: str) -> str:
    return palette[hash(key) % len(palette)]


def _node_id(s: str) -> str:
    return "".join(c if c.isalnum() or c == "_" else "_" for c in s)


def render(
    handle: Any,
    *,
    ignored_streams: set[tuple[str, str]] | None = None,
    ignored_modules: set[str] | None = None,
) -> str:
    """Generate a Graphviz DOT string from a SystemHandle.

    Args:
        handle: SystemHandle or WorkerSystemHandle.
        ignored_streams: ``{(port_name, type_name), ...}`` pairs to skip.
        ignored_modules: Module names to exclude from the graph.

    Returns:
        DOT-format string. Feed to ``dot -Tsvg`` or ``dot -Tpng``.
    """
    if ignored_streams is None:
        ignored_streams = set()
    if ignored_modules is None:
        ignored_modules = set()

    modules = handle.modules      # Dict[str, Module | RPCClient]
    connections = handle.connections  # List[Tuple[str, str, str, str]]

    # Layer map — safe for RPCClient proxies
    module_layers: dict[str, int | None] = {
        name: getattr(mod, "layer", None) for name, mod in modules.items()
    }

    # Port → type name lookup (best-effort; RPCClient has no port info)
    port_type: dict[tuple[str, str], str] = {}
    for name, mod in modules.items():
        try:
            for pname, port in mod.ports_out.items():
                t = getattr(port, "msg_type", None)
                port_type[(name, pname)] = t.__name__ if t else "Any"
        except Exception:
            pass

    # Collect producers / consumers keyed by (port_name, type_name)
    producers: dict[tuple[str, str], list[str]] = defaultdict(list)
    consumers: dict[tuple[str, str], list[str]] = defaultdict(list)
    for out_mod, out_port, in_mod, _in_port in connections:
        if out_mod in ignored_modules or in_mod in ignored_modules:
            continue
        type_name = port_type.get((out_mod, out_port), "Any")
        key = (out_port, type_name)
        if key not in ignored_streams:
            producers[key].append(out_mod)
            consumers[key].append(in_mod)

    # Active channels: have both a producer and a consumer
    active: dict[tuple[str, str], str] = {}
    for key in producers:
        if key in consumers:
            active[key] = _color_for(_CHAN_COLORS, f"{key[0]}:{key[1]}")

    # Group modules by layer
    by_layer: dict[int | None, list[str]] = defaultdict(list)
    for name in modules:
        if name not in ignored_modules:
            by_layer[module_layers[name]].append(name)

    lines = [
        "digraph lingtu {",
        "    bgcolor=transparent;",
        "    rankdir=LR;",
        "    splines=true;",
        "    compound=true;",
        (
            '    node [shape=box, style=filled, fillcolor="#1e1e2e",'
            ' fontcolor="#cdd6f4", color="#89b4fa",'
            ' fontname="monospace", fontsize=11, margin="0.15,0.1"];'
        ),
        '    edge [fontname="monospace", fontsize=9, color="#6c7086"];',
        "",
    ]

    # Layer clusters
    for layer in sorted(by_layer.keys(), key=lambda x: (x is None, x)):
        mods = sorted(by_layer[layer])
        label = _LAYER_LABELS.get(layer, f"L{layer}" if layer is not None else "No Layer")
        color = _LAYER_COLORS.get(layer, _LAYER_COLORS[None])
        cid = _node_id(f"cluster_{label}")

        lines.append(f"    subgraph {cid} {{")
        lines.append(f'        label="{label}";')
        lines.append(f'        fontcolor="{color}";')
        lines.append('        fontname="monospace";')
        lines.append('        fontsize=13;')
        lines.append('        style="filled,dashed";')
        lines.append(f'        color="{color}";')
        lines.append(f'        fillcolor="{color}15";')
        for mod in mods:
            running = getattr(modules.get(mod), "running", True)
            fill = "#1e1e2e" if running else "#45475a"
            lines.append(f'        {mod} [fillcolor="{fill}", color="{color}"];')
        lines.append("    }")
        lines.append("")

    # Channel hub nodes
    lines.append("    // Data channel hubs")
    for (port_name, type_name), color in sorted(active.items()):
        nid = _node_id(f"chan_{port_name}_{type_name}")
        label = f"{port_name}\\n{type_name}"
        lines.append(
            f'    {nid} [label="{label}", shape=note, style=filled,'
            f' fillcolor="{color}28", color="{color}", fontcolor="{color}",'
            f' width=0, height=0, margin="0.08,0.04", fontsize=9];'
        )

    lines.append("")
    lines.append("    // Edges")
    for (port_name, type_name), color in sorted(active.items()):
        nid = _node_id(f"chan_{port_name}_{type_name}")
        for prod in set(producers[(port_name, type_name)]):
            lines.append(f'    {prod} -> {nid} [color="{color}", arrowhead=none];')
        for cons in set(consumers[(port_name, type_name)]):
            lines.append(f'    {nid} -> {cons} [color="{color}"];')

    lines.append("}")
    return "\n".join(lines)


def render_svg(handle: Any, output_path: str, **kwargs: Any) -> None:
    """Write a rendered SVG to *output_path* (requires the ``dot`` CLI)."""
    _run_graphviz(render(handle, **kwargs), output_path, fmt="svg")


def render_png(handle: Any, output_path: str, **kwargs: Any) -> None:
    """Write a rendered PNG to *output_path* (requires the ``dot`` CLI)."""
    _run_graphviz(render(handle, **kwargs), output_path, fmt="png")


def _run_graphviz(dot_code: str, output_path: str, fmt: str) -> None:
    try:
        result = subprocess.run(
            ["dot", f"-T{fmt}", "-o", output_path],
            input=dot_code,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            timeout=15,
        )
    except subprocess.TimeoutExpired:
        raise RuntimeError(
            f"graphviz 'dot' timed out rendering {fmt} to {output_path}"
        ) from None
    if result.returncode != 0:
        raise RuntimeError(
            f"graphviz 'dot' failed (exit {result.returncode}):\n{result.stderr}"
        )
