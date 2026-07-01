"""runtime.introspection.text — ANSI terminal renderer for SystemHandle.

No external dependencies — works without graphviz.

Usage::

    from runtime.introspection.text import render_text

    system = autoconnect(...).build()
    print(render_text(system))           # colored terminal output
    print(render_text(system, color=False))  # plain text for log files
"""

from __future__ import annotations

from collections import defaultdict
from typing import Any

_LAYER_LABELS: dict[int | None, str] = {
    0: "L0·Safety",
    1: "L1·Driver",
    2: "L2·BaseAutonomy",
    3: "L3·Perception",
    4: "L4·Semantic",
    5: "L5·Navigation",
    6: "L6·Gateway",
}


def render_text(handle: Any, *, color: bool = True) -> str:
    """Render a SystemHandle as a structured terminal table.

    Shows modules grouped by layer with port stats and connection list.

    Args:
        handle: SystemHandle or WorkerSystemHandle.
        color: Use ANSI escape codes. Set False for log-safe output.

    Returns:
        Multi-line string for ``print()``.
    """
    B  = "\033[1m"   if color else ""
    G  = "\033[32m"  if color else ""
    R  = "\033[31m"  if color else ""
    Y  = "\033[33m"  if color else ""
    C  = "\033[36m"  if color else ""
    D  = "\033[2m"   if color else ""
    M  = "\033[35m"  if color else ""
    X  = "\033[0m"   if color else ""

    modules     = handle.modules
    connections = handle.connections
    started     = getattr(handle, "started", True)

    title_status = f"{G}RUNNING{X}" if started else f"{R}STOPPED{X}"

    lines: list[str] = []
    W = 64
    lines.append(f"{B}{'═' * W}{X}")
    lines.append(
        f"{B}  LingTu  {title_status}  "
        f"{len(modules)} modules · {len(connections)} connections{X}"
    )
    lines.append(f"{B}{'═' * W}{X}")

    # Group modules by layer
    by_layer: dict[int | None, list[str]] = defaultdict(list)
    for name in modules:
        by_layer[getattr(modules[name], "layer", None)].append(name)

    for layer in sorted(by_layer.keys(), key=lambda x: (x is None, x)):
        label = _LAYER_LABELS.get(layer, f"L{layer}" if layer is not None else "NoLayer")
        lines.append(f"\n{C}{B}▶ {label}{X}")

        for mod_name in sorted(by_layer[layer]):
            mod = modules[mod_name]
            running = getattr(mod, "running", True)
            dot = f"{G}●{X}" if running else f"{R}○{X}"
            proxy_tag = f"  {D}[proxy]{X}" if _is_proxy(mod) else ""
            lines.append(f"  {dot} {B}{mod_name}{X}{proxy_tag}")

            # Input ports
            try:
                for pname, port in mod.ports_in.items():
                    t = getattr(port, "msg_type", None)
                    tname = t.__name__ if t else "?"
                    cnt  = getattr(port, "msg_count", 0)
                    conn = getattr(port, "connected", False)
                    mark = f"{G}✓{X}" if conn else f"{Y}○{X}"
                    lines.append(
                        f"      {D}←{X} {pname}: {C}{tname}{X}"
                        f"  {D}msgs={cnt}{X}  {mark}"
                    )
            except Exception:
                pass

            # Output ports
            try:
                for pname, port in mod.ports_out.items():
                    t = getattr(port, "msg_type", None)
                    tname = t.__name__ if t else "?"
                    cnt  = getattr(port, "msg_count", 0)
                    subs = getattr(port, "callback_count", 0)
                    lines.append(
                        f"      {M}→{X} {pname}: {C}{tname}{X}"
                        f"  {D}msgs={cnt}  subs={subs}{X}"
                    )
            except Exception:
                pass

    # Connection list
    lines.append(f"\n{B}{'─' * W}{X}")
    lines.append(f"{B}  Connections ({len(connections)}){X}")
    lines.append(f"{'─' * W}")
    if connections:
        for out_mod, out_port, in_mod, in_port in connections:
            same_port = out_port == in_port
            port_str = (
                f".{out_port}" if same_port
                else f".{out_port} → .{in_port}"
            )
            lines.append(
                f"  {C}{out_mod}{X}{D}{port_str.split('→')[0]}{X}"
                f" {D}→{X} {C}{in_mod}{X}"
                + (f"{D}.{in_port}{X}" if not same_port else "")
            )
    else:
        lines.append(f"  {D}(none){X}")

    lines.append(f"{B}{'═' * W}{X}")
    return "\n".join(lines)


def render_connections(handle: Any, *, color: bool = True) -> str:
    """Compact connection-only view: one line per wire."""
    C = "\033[36m" if color else ""
    D = "\033[2m"  if color else ""
    X = "\033[0m"  if color else ""
    rows = []
    for out_mod, out_port, in_mod, in_port in handle.connections:
        rows.append(
            f"  {C}{out_mod}{X}.{out_port}"
            f" {D}──►{X} "
            f"{C}{in_mod}{X}.{in_port}"
        )
    return "\n".join(rows) if rows else "(no connections)"


def _is_proxy(mod: Any) -> bool:
    return type(mod).__name__ == "RPCClient"
