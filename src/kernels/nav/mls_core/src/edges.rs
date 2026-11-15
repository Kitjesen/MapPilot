use std::collections::HashMap;

use crate::adjacency;
use crate::corridor;
use crate::corridor::OrderedF32;
use crate::nodes;
use crate::{Edge, GraphConfig, Node, SurfaceCell, SurfaceGraph};

pub(crate) fn build_graph(cells: &[SurfaceCell], config: GraphConfig) -> SurfaceGraph {
    let mut sorted = cells.to_vec();
    sorted.sort_unstable();
    sorted.dedup();
    if sorted.is_empty() {
        return SurfaceGraph::default();
    }

    let step = config.max_link_cells.max(1);
    let adjacency = adjacency::build_adjacency(&sorted, config);
    let columns = nodes::surface_columns(&sorted);
    let wall = nodes::wall_distances(&sorted, &adjacency, &columns, step);
    let node_cells = nodes::place_nodes(&sorted, &adjacency, &wall, config);
    let nodes: Vec<Node> = node_cells
        .iter()
        .enumerate()
        .map(|(id, &cell)| Node {
            id: id as u32,
            cell: sorted[cell],
        })
        .collect();
    if nodes.len() <= 1 {
        return SurfaceGraph {
            nodes,
            edges: Vec::new(),
        };
    }

    let mut owner = vec![usize::MAX; sorted.len()];
    let mut distance = vec![f32::INFINITY; sorted.len()];
    let mut predecessor = vec![usize::MAX; sorted.len()];
    let mut queue = std::collections::BinaryHeap::<std::cmp::Reverse<(OrderedF32, usize)>>::new();
    for (node_id, &cell) in node_cells.iter().enumerate() {
        owner[cell] = node_id;
        distance[cell] = 0.0;
        queue.push(std::cmp::Reverse((OrderedF32(0.0), cell)));
    }
    while let Some(std::cmp::Reverse((OrderedF32(cost), current))) = queue.pop() {
        if cost > distance[current] + 1e-6 {
            continue;
        }
        for &(next, base) in &adjacency[current] {
            let penalty = edge_penalty(
                sorted[current],
                sorted[next],
                wall[current].min(wall[next]),
                config,
            );
            let next_cost = cost + base + penalty;
            if next_cost + 1e-6 < distance[next] {
                distance[next] = next_cost;
                owner[next] = owner[current];
                predecessor[next] = current;
                queue.push(std::cmp::Reverse((OrderedF32(next_cost), next)));
            }
        }
    }

    let mut boundaries: HashMap<(usize, usize), (f32, usize, usize)> = HashMap::new();
    for (current, links) in adjacency.iter().enumerate() {
        for &(next, base) in links {
            let (a, b) = (owner[current], owner[next]);
            if a == usize::MAX || a == b {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            let cost = distance[current] + base + distance[next];
            let entry = boundaries
                .entry(key)
                .or_insert((f32::INFINITY, current, next));
            if cost < entry.0 {
                *entry = (cost, current, next);
            }
        }
    }

    let mut edges = Vec::with_capacity(boundaries.len());
    for ((a, b), (cost, current, next)) in boundaries {
        let (left, right) = if owner[current] == a {
            (current, next)
        } else {
            (next, current)
        };
        let mut corridor = corridor::chain_to_source(left, &predecessor, &node_cells);
        let tail = corridor::chain_to_source(right, &predecessor, &node_cells);
        corridor.reverse();
        corridor.extend(tail);
        corridor.dedup();
        edges.push(Edge {
            a: a as u32,
            b: b as u32,
            cost,
            corridor: corridor.into_iter().map(|index| sorted[index]).collect(),
        });
    }
    edges.sort_by_key(|edge| (edge.a, edge.b));
    SurfaceGraph { nodes, edges }
}

fn edge_penalty(a: SurfaceCell, b: SurfaceCell, wall: f32, config: GraphConfig) -> f32 {
    let rise = (a.z - b.z).unsigned_abs() as f32;
    let step = rise * config.step_penalty_weight.max(0.0);
    let clearance = config.wall_clearance_cells.max(0) as f32;
    let buffer = config.wall_buffer_cells.max(0) as f32;
    let wall_cost = if buffer > 0.0 && wall < clearance + buffer {
        (1.0 - (wall - clearance).max(0.0) / buffer).max(0.0) * config.wall_buffer_weight.max(0.0)
    } else {
        0.0
    };
    step + wall_cost
}
