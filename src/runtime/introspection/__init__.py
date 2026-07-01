"""runtime.introspection — Blueprint and SystemHandle visualization.

Quick usage::

    from runtime.introspection import render_text, render_dot, render_svg

    print(render_text(system))           # ANSI terminal tree (no deps)
    print(render_dot(system))            # Graphviz DOT string
    render_svg(system, "graph.svg")      # SVG file (needs graphviz CLI)
    render_png(system, "graph.png")      # PNG file (needs graphviz CLI)
"""

from .dot import render as render_dot
from .dot import render_png, render_svg
from .text import render_connections, render_text

__all__ = [
    "render_connections",
    "render_dot",
    "render_png",
    "render_svg",
    "render_text",
]
