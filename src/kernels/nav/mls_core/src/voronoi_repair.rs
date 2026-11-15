use crate::edges::build_graph;
use crate::{GraphConfig, RegionBounds, SurfaceCell, SurfaceGraph};

/// Build the ownership and predecessor graph for a repair window.
///
/// The existing Rust graph builder already performs multi-source Dijkstra and
/// corridor reconstruction. Keeping this helper separate gives the update
/// path a single place to replace with a bounded repair window implementation.
pub(crate) fn rebuild_window(
    cells: &[SurfaceCell],
    _window: RegionBounds,
    config: GraphConfig,
) -> SurfaceGraph {
    build_graph(cells, config)
}
