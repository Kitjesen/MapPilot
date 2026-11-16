use std::collections::{HashMap, HashSet};

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

    if spacing == 1 {
        // Distinct integer XYZ cells already have squared distance at least one.
        nodes.extend((0..cells.len()).filter(|&index| wall[index] + 1e-6 >= clearance));
        nodes.sort_unstable();
        nodes.dedup();
    } else {
        let far_from_nodes = |nodes: &[usize], index: usize| {
            nodes
                .iter()
                .all(|&other| {
                    let same_column = cells[index].x == cells[other].x && cells[index].y == cells[other].y
                        && cells[index].z != cells[other].z;
                    same_column || squared_distance(cells[index], cells[other]) >= spacing * spacing
                })
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
    }

    let mut selected: HashSet<usize> = nodes.iter().copied().collect();
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
        if !component.iter().any(|index| selected.contains(index)) {
            if let Some(&best) = component
                .iter()
                .max_by(|&&a, &&b| wall[a].total_cmp(&wall[b]).then(cells[b].cmp(&cells[a])))
            {
                nodes.push(best);
                selected.insert(best);
            }
        }
    }
    nodes.sort_unstable();
    nodes
}

#[cfg(test)]
mod tests {
    use super::*;

    fn original_placement(
        cells: &[SurfaceCell],
        adjacency: &[Vec<(usize, f32)>],
        wall: &[f32],
        config: GraphConfig,
        forced: &[SurfaceCell],
    ) -> Vec<usize> {
        let spacing = i64::from(config.node_spacing_cells.max(1));
        let mut nodes = forced
            .iter()
            .filter_map(|cell| cells.binary_search(cell).ok())
            .collect::<Vec<_>>();
        nodes.sort_unstable();
        nodes.dedup();
        let far = |nodes: &[usize], index: usize| {
            nodes
                .iter()
                .all(|&other| {
                    let same_column = cells[index].x == cells[other].x && cells[index].y == cells[other].y
                        && cells[index].z != cells[other].z;
                    same_column || squared_distance(cells[index], cells[other]) >= spacing * spacing
                })
        };
        let mut candidates = (0..cells.len())
            .filter(|&index| {
                wall[index] + 1e-6 >= config.wall_clearance_cells.max(0) as f32
                    && far(&nodes, index)
            })
            .collect::<Vec<_>>();
        candidates.sort_by(|&a, &b| wall[b].total_cmp(&wall[a]).then(cells[a].cmp(&cells[b])));
        for index in candidates {
            if far(&nodes, index) {
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

    #[test]
    fn placement_matches_original() {
        for levels in [&[0][..], &[0, 1, 5][..]] {
            let mut cells = Vec::new();
            for x in 0..8 {
                for y in 0..5 {
                    for &z in levels {
                        cells.push(SurfaceCell::new(x, y, z));
                    }
                }
            }
            cells.push(SurfaceCell::new(20, 0, 0));
            let wall = (0..cells.len())
                .map(|index| (index % 4) as f32)
                .collect::<Vec<_>>();
            let forced = [cells[0], cells[0], cells[5], SurfaceCell::new(100, 100, 0)];
            for spacing in [1, 2, 3] {
                for clearance in [0, 2, 9] {
                    let config = GraphConfig {
                        node_spacing_cells: spacing,
                        wall_clearance_cells: clearance,
                        max_link_cells: 1,
                        ..GraphConfig::default()
                    };
                    let adjacency = crate::adjacency::build_adjacency(&cells, config);
                    assert_eq!(
                        place_nodes_with_forced(&cells, &adjacency, &wall, config, &forced),
                        original_placement(&cells, &adjacency, &wall, config, &forced),
                        "spacing={spacing} clearance={clearance} layers={levels:?}",
                    );
                }
            }
        }
    }
}
