use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::{Edge, GraphConfig, RegionBounds, SurfaceCell, SurfaceGraph};

use crate::edges::build_graph;

pub type StableNodeId = u32;
pub type StableEdgeId = u64;

#[derive(Clone, Debug, PartialEq)]
pub struct StableNode {
    pub id: StableNodeId,
    pub cell: SurfaceCell,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StableEdge {
    pub id: StableEdgeId,
    pub from: StableNodeId,
    pub to: StableNodeId,
    pub cost: f32,
    pub corridor: Vec<SurfaceCell>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct IncrementalGraph {
    pub cells: BTreeSet<SurfaceCell>,
    pub nodes: BTreeMap<StableNodeId, StableNode>,
    pub edges: BTreeMap<StableEdgeId, StableEdge>,
    pub node_by_cell: HashMap<SurfaceCell, StableNodeId>,
}

impl IncrementalGraph {
    pub fn from_cells(cells: &[SurfaceCell], config: GraphConfig) -> Self {
        Self::from_graph(cells, &build_graph(cells, config))
    }

    pub fn from_graph(cells: &[SurfaceCell], graph: &SurfaceGraph) -> Self {
        let mut result = Self {
            cells: cells.iter().copied().collect(),
            ..Self::default()
        };
        for node in &graph.nodes {
            result.node_by_cell.insert(node.cell, node.id);
            result.nodes.insert(
                node.id,
                StableNode {
                    id: node.id,
                    cell: node.cell,
                },
            );
        }
        for edge in &graph.edges {
            let id = edge_id(edge.a, edge.b);
            result.edges.insert(
                id,
                StableEdge {
                    id,
                    from: edge.a,
                    to: edge.b,
                    cost: edge.cost,
                    corridor: edge.corridor.clone(),
                },
            );
        }
        result
    }

    pub fn to_surface_graph(&self) -> SurfaceGraph {
        SurfaceGraph {
            nodes: self
                .nodes
                .values()
                .map(|node| crate::Node {
                    id: node.id,
                    cell: node.cell,
                })
                .collect(),
            edges: self
                .edges
                .values()
                .map(|edge| Edge {
                    a: edge.from,
                    b: edge.to,
                    cost: edge.cost,
                    corridor: edge.corridor.clone(),
                })
                .collect(),
        }
    }

    pub fn update(
        &mut self,
        cells: &[SurfaceCell],
        region: RegionBounds,
        config: GraphConfig,
    ) -> GraphRepairReport {
        let next_cells: BTreeSet<_> = cells.iter().copied().collect();
        let changed_cells = self.cells.symmetric_difference(&next_cells).count();
        let window = region.expand(
            config.max_link_cells.max(1) + config.node_spacing_cells.max(1),
            config.max_link_cells.max(1),
        );
        let relocation = crate::node_relocation::relocate_nodes(self, &next_cells, window, config);
        let repair = crate::edge_repair::repair_edges(self, cells, window, config);
        self.cells = next_cells;
        if repair.global_fallback {
            let rebuilt = build_graph(cells, config);
            let (removed_nodes, added_nodes, reused_edges, rebuilt_edges) =
                self.replace_from_graph(rebuilt);
            return GraphRepairReport {
                changed_cells,
                window,
                relocated_nodes: relocation.relocated,
                added_nodes,
                removed_nodes,
                invalidated_edges: repair.invalidated,
                rebuilt_edges,
                reused_edges,
                local_repair: false,
                global_fallback: true,
            };
        }
        GraphRepairReport {
            changed_cells,
            window,
            relocated_nodes: relocation.relocated,
            added_nodes: relocation.added + repair.added_nodes,
            removed_nodes: relocation.removed,
            invalidated_edges: repair.invalidated,
            rebuilt_edges: repair.rebuilt,
            reused_edges: repair.reused,
            local_repair: repair.local,
            global_fallback: false,
        }
    }

    fn replace_from_graph(&mut self, graph: SurfaceGraph) -> (usize, usize, usize, usize) {
        let old_nodes = self.nodes.clone();
        let old_edges = self.edges.clone();
        let mut id_by_cell: HashMap<SurfaceCell, StableNodeId> = old_nodes
            .values()
            .map(|node| (node.cell, node.id))
            .collect();
        let mut next_id = old_nodes
            .keys()
            .next_back()
            .copied()
            .unwrap_or(0)
            .saturating_add(1);
        let mut remap = HashMap::new();
        let mut nodes = BTreeMap::new();
        for node in graph.nodes {
            let id = *id_by_cell.entry(node.cell).or_insert_with(|| {
                let id = next_id;
                next_id = next_id.saturating_add(1);
                id
            });
            remap.insert(node.id, id);
            nodes.insert(
                id,
                StableNode {
                    id,
                    cell: node.cell,
                },
            );
        }
        let mut edges = BTreeMap::new();
        let mut reused_edges = 0;
        let mut rebuilt_edges = 0;
        for edge in graph.edges {
            let (Some(&from), Some(&to)) = (remap.get(&edge.a), remap.get(&edge.b)) else {
                continue;
            };
            let id = edge_id(from, to);
            if old_edges.contains_key(&id) {
                reused_edges += 1;
            } else {
                rebuilt_edges += 1;
            }
            edges.insert(
                id,
                StableEdge {
                    id,
                    from,
                    to,
                    cost: edge.cost,
                    corridor: edge.corridor,
                },
            );
        }
        let removed_nodes = old_nodes
            .len()
            .saturating_sub(nodes.keys().filter(|id| old_nodes.contains_key(id)).count());
        let added_nodes = nodes
            .len()
            .saturating_sub(nodes.keys().filter(|id| old_nodes.contains_key(id)).count());
        self.nodes = nodes;
        self.edges = edges;
        self.node_by_cell = self
            .nodes
            .values()
            .map(|node| (node.cell, node.id))
            .collect();
        (removed_nodes, added_nodes, reused_edges, rebuilt_edges)
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct GraphRepairReport {
    pub changed_cells: usize,
    pub window: RegionBounds,
    pub relocated_nodes: usize,
    pub added_nodes: usize,
    pub removed_nodes: usize,
    pub invalidated_edges: usize,
    pub rebuilt_edges: usize,
    pub reused_edges: usize,
    pub local_repair: bool,
    pub global_fallback: bool,
}

pub(crate) fn edge_id(from: StableNodeId, to: StableNodeId) -> StableEdgeId {
    let (a, b) = if from <= to { (from, to) } else { (to, from) };
    (u64::from(a) << 32) | u64::from(b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn restored_updates_match() {
        let cells = (0..24)
            .flat_map(|x| (0..3).map(move |y| SurfaceCell::new(x, y, 0)))
            .collect::<Vec<_>>();
        let config = GraphConfig {
            node_spacing_cells: 2,
            max_link_cells: 1,
            wall_buffer_cells: 2,
            wall_buffer_weight: 0.5,
            ..GraphConfig::default()
        };
        let prepared = build_graph(&cells, config);
        let mut rebuilt = IncrementalGraph::from_cells(&cells, config);
        let mut restored = IncrementalGraph::from_graph(&cells, &prepared);
        assert_eq!(restored, rebuilt);

        let updated = cells
            .into_iter()
            .filter(|cell| *cell != SurfaceCell::new(11, 1, 0))
            .collect::<Vec<_>>();
        let region = RegionBounds::new(11, 11, 1, 1, 0, 0);
        assert_eq!(
            restored.update(&updated, region, config),
            rebuilt.update(&updated, region, config)
        );
        assert_eq!(restored, rebuilt);
    }
}
