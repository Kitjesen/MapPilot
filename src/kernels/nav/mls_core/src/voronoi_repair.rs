use crate::edges::build_graph_with_forced_nodes;
use crate::{GraphConfig, RegionBounds, SurfaceCell, SurfaceGraph};

/// Build the ownership and predecessor graph for a repair window.
///
/// Existing boundary nodes are forced into the local graph so multi-source
/// Dijkstra keeps the ownership that enters the repair window. The graph
/// builder then recomputes predecessor corridors and boundary edges from the
/// updated surface cells.
pub(crate) fn rebuild_window(
    cells: &[SurfaceCell],
    _window: RegionBounds,
    forced_nodes: &[SurfaceCell],
    config: GraphConfig,
) -> SurfaceGraph {
    build_graph_with_forced_nodes(cells, config, forced_nodes)
}
