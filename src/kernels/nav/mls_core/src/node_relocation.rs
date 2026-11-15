use std::collections::BTreeSet;

use crate::stable_graph::IncrementalGraph;
use crate::{GraphConfig, RegionBounds, SurfaceCell};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct RelocationReport {
    pub relocated: usize,
    pub added: usize,
    pub removed: usize,
}

pub(crate) fn relocate_nodes(
    graph: &mut IncrementalGraph,
    next_cells: &BTreeSet<SurfaceCell>,
    window: RegionBounds,
    config: GraphConfig,
) -> RelocationReport {
    let mut report = RelocationReport::default();
    let old_nodes: Vec<_> = graph
        .nodes
        .values()
        .filter(|node| window.contains(node.cell) || !next_cells.contains(&node.cell))
        .map(|node| (node.id, node.cell))
        .collect();
    let mut used: BTreeSet<_> = graph
        .nodes
        .values()
        .filter(|node| !window.contains(node.cell) && next_cells.contains(&node.cell))
        .map(|node| node.cell)
        .collect();
    for (id, old_cell) in old_nodes {
        if next_cells.contains(&old_cell) && !window.contains(old_cell) {
            continue;
        }
        let candidate = next_cells
            .iter()
            .filter(|cell| window.contains(**cell) && !used.contains(cell))
            .min_by_key(|cell| {
                let dx = i64::from(cell.x - old_cell.x);
                let dy = i64::from(cell.y - old_cell.y);
                let dz = i64::from(cell.z - old_cell.z);
                dx * dx + dy * dy + dz * dz
            })
            .copied();
        if let Some(cell) = candidate {
            let within = (cell.x - old_cell.x).abs() <= config.max_link_cells.max(1)
                && (cell.y - old_cell.y).abs() <= config.max_link_cells.max(1)
                && (cell.z - old_cell.z).abs() <= config.max_link_cells.max(1);
            if within {
                graph.node_by_cell.remove(&old_cell);
                graph.node_by_cell.insert(cell, id);
                if let Some(node) = graph.nodes.get_mut(&id) {
                    node.cell = cell;
                }
                used.insert(cell);
                report.relocated += 1;
                continue;
            }
        }
        graph.node_by_cell.remove(&old_cell);
        graph.nodes.remove(&id);
        report.removed += 1;
    }
    report
}
