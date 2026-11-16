use std::collections::{BTreeSet, HashMap};

use crate::stable_graph::{edge_id, IncrementalGraph, StableEdge};
use crate::voronoi_repair::rebuild_window;
use crate::{GraphConfig, RegionBounds, SurfaceCell};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct EdgeRepairReport {
    pub invalidated: usize,
    pub rebuilt: usize,
    pub reused: usize,
    pub added_nodes: usize,
    pub local: bool,
    pub global_fallback: bool,
}

pub(crate) fn repair_edges(
    graph: &mut IncrementalGraph,
    cells: &[SurfaceCell],
    window: RegionBounds,
    config: GraphConfig,
) -> EdgeRepairReport {
    // Rebuild one link band beyond the invalidation window so a repaired edge
    // can connect to a node that remains outside the changed region.
    let graph_window = window.expand(config.max_link_cells.max(1), config.max_link_cells.max(1));
    let local_cells: Vec<_> = cells
        .iter()
        .copied()
        .filter(|cell| graph_window.contains(*cell))
        .collect();
    let next_cells: BTreeSet<_> = cells.iter().copied().collect();
    let forced_nodes: Vec<_> = graph
        .nodes
        .values()
        .map(|node| node.cell)
        .filter(|cell| graph_window.contains(*cell) && next_cells.contains(cell))
        .collect();
    let local_graph = rebuild_window(&local_cells, graph_window, &forced_nodes, config);
    let mut report = EdgeRepairReport {
        local: !local_cells.is_empty() && (!local_graph.nodes.is_empty() || graph.nodes.is_empty()),
        global_fallback: !local_cells.is_empty()
            && local_graph.nodes.is_empty()
            && !graph.nodes.is_empty(),
        ..EdgeRepairReport::default()
    };
    let old_edges: Vec<_> = graph
        .edges
        .values()
        .filter(|edge| {
            !graph.nodes.contains_key(&edge.from)
                || !graph.nodes.contains_key(&edge.to)
                || edge.corridor.iter().any(|cell| window.contains(*cell))
                || graph
                    .nodes
                    .get(&edge.from)
                    .is_some_and(|node| window.contains(node.cell))
                || graph
                    .nodes
                    .get(&edge.to)
                    .is_some_and(|node| window.contains(node.cell))
        })
        .map(|edge| edge.id)
        .collect();
    if local_cells.is_empty() && !old_edges.is_empty() {
        report.local = false;
        report.global_fallback = true;
    }
    for id in old_edges {
        if graph.edges.remove(&id).is_some() {
            report.invalidated += 1;
        }
    }
    let mut local_ids = HashMap::new();
    for node in local_graph.nodes {
        let id = if let Some(&id) = graph.node_by_cell.get(&node.cell) {
            id
        } else {
            let id = graph
                .nodes
                .keys()
                .next_back()
                .copied()
                .unwrap_or(0)
                .saturating_add(1);
            graph.node_by_cell.insert(node.cell, id);
            graph.nodes.insert(
                id,
                crate::stable_graph::StableNode {
                    id,
                    cell: node.cell,
                },
            );
            report.added_nodes += 1;
            id
        };
        local_ids.insert(node.id, id);
    }
    for edge in local_graph.edges {
        let (Some(&from), Some(&to)) = (local_ids.get(&edge.a), local_ids.get(&edge.b)) else {
            continue;
        };
        let id = edge_id(from, to);
        let replacement = StableEdge {
            id,
            from,
            to,
            cost: edge.cost,
            corridor: edge.corridor,
        };
        if graph.edges.contains_key(&id) {
            report.reused += 1;
        } else {
            report.rebuilt += 1;
        }
        graph.edges.insert(id, replacement);
    }
    report
}
