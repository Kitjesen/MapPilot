use std::collections::HashMap;

use crate::corridor::OrderedF32;
use crate::surfaces::{neighbors, squared_distance};
use crate::{GraphConfig, SurfaceCell};

pub(crate) fn wall_distances(
    cells: &[SurfaceCell],
    adjacency: &[Vec<(usize, f32)>],
    columns: &HashMap<(i32, i32), Vec<i32>>,
    step_cells: i32,
) -> Vec<f32> {
    let mut distance = vec![f32::INFINITY; cells.len()];
    let mut queue = std::collections::BinaryHeap::<std::cmp::Reverse<(OrderedF32, usize)>>::new();
    for (index, &cell) in cells.iter().enumerate() {
        if real_wall_adjacent(cell, columns, step_cells) {
            distance[index] = 0.0;
            queue.push(std::cmp::Reverse((OrderedF32(0.0), index)));
        }
    }
    if queue.is_empty() {
        return vec![0.0; cells.len()];
    }
    while let Some(std::cmp::Reverse((OrderedF32(cost), current))) = queue.pop() {
        if cost > distance[current] + 1e-6 {
            continue;
        }
        for &(next, edge_cost) in &adjacency[current] {
            let next_cost = cost + edge_cost;
            if next_cost + 1e-6 < distance[next] {
                distance[next] = next_cost;
                queue.push(std::cmp::Reverse((OrderedF32(next_cost), next)));
            }
        }
    }
    distance
}

pub(crate) fn surface_columns(cells: &[SurfaceCell]) -> HashMap<(i32, i32), Vec<i32>> {
    let mut columns: HashMap<(i32, i32), Vec<i32>> = HashMap::new();
    for cell in cells {
        columns.entry((cell.x, cell.y)).or_default().push(cell.z);
    }
    for levels in columns.values_mut() {
        levels.sort_unstable();
        levels.dedup();
    }
    columns
}

fn real_wall_adjacent(
    cell: SurfaceCell,
    columns: &HashMap<(i32, i32), Vec<i32>>,
    step_cells: i32,
) -> bool {
    for (neighbor_x, neighbor_y) in neighbors(cell.x, cell.y) {
        if columns
            .get(&(neighbor_x, neighbor_y))
            .is_some_and(|levels| levels.iter().any(|&z| (z - cell.z).abs() <= step_cells))
        {
            continue;
        }
        let direction = (neighbor_x - cell.x, neighbor_y - cell.y);
        let mut found_column = false;
        for distance in 1..=4 {
            let key = (
                cell.x + direction.0 * distance,
                cell.y + direction.1 * distance,
            );
            let Some(levels) = columns.get(&key) else {
                continue;
            };
            found_column = true;
            if !levels.iter().any(|&z| (z - cell.z).abs() <= step_cells) {
                return true;
            }
            break;
        }
        if !found_column {
            return true;
        }
    }
    false
}

pub(crate) fn place_nodes_with_forced(
    cells: &[SurfaceCell],
    adjacency: &[Vec<(usize, f32)>],
    wall: &[f32],
    config: GraphConfig,
    forced_cells: &[SurfaceCell],
) -> Vec<usize> {
    let spacing = config.node_spacing_cells.max(1) as i64;
    let clearance = config.wall_clearance_cells.max(0) as f32;
    let mut nodes = forced_cells
        .iter()
        .filter_map(|cell| cells.binary_search(cell).ok())
        .collect::<Vec<_>>();
    nodes.sort_unstable();
    nodes.dedup();

    let far_from_nodes = |nodes: &[usize], index: usize| {
        nodes
            .iter()
            .all(|&other| squared_distance(cells[index], cells[other]) >= spacing * spacing)
    };
    let mut candidates: Vec<usize> = (0..cells.len())
        .filter(|&index| wall[index] + 1e-6 >= clearance && far_from_nodes(&nodes, index))
        .collect();
    candidates.sort_by(|&a, &b| wall[b].total_cmp(&wall[a]).then(cells[a].cmp(&cells[b])));
    for index in candidates {
        if far_from_nodes(&nodes, index) {
            nodes.push(index);
        }
    }

    let mut seen = vec![false; cells.len()];
    for seed in 0..cells.len() {
        if seen[seed] {
            continue;
        }
        let mut component = Vec::new();
        let mut stack = vec![seed];
        seen[seed] = true;
        while let Some(current) = stack.pop() {
            component.push(current);
            for &(next, _) in &adjacency[current] {
                if !seen[next] {
                    seen[next] = true;
                    stack.push(next);
                }
            }
        }
        if !component.iter().any(|index| nodes.contains(index)) {
            if let Some(&best) = component
                .iter()
                .max_by(|&&a, &&b| wall[a].total_cmp(&wall[b]).then(cells[b].cmp(&cells[a])))
            {
                nodes.push(best);
            }
        }
    }
    nodes.sort_unstable();
    nodes
}
