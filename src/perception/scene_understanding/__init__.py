"""Spatial Connectivity Graph (SCG) scene-understanding algorithms (USS-Nav).

EXPERIMENTAL: none of these modules are wired into the production blueprint
today (see ``src/perception/README.md``). They implement the "USS-Nav"
research design -- polyhedron-based free-space decomposition, a spatial
connectivity graph, Leiden region segmentation, coverage/uncertainty
tracking, and two path planners built on top of that representation.

``memory.spatial.topology_graph.TopologySemGraph`` accepts a
``GeometryExtractor``-like object via ``set_geometry_extractor()`` (duck
typing, no import), which is the only production-side extension point that
overlaps with this package.
"""

__all__: list[str] = []
