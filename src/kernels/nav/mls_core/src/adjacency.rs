use std::collections::HashMap;

use crate::{GraphConfig, SurfaceCell};

pub(crate) fn build_adjacency(
    cells: &[SurfaceCell],
    config: GraphConfig,
) -> Vec<Vec<(usize, f32)>> {
    let lookup: HashMap<SurfaceCell, usize> = cells
        .iter()
        .copied()
        .enumerate()
        .map(|(index, cell)| (cell, index))
        .collect();
    let mut adjacency = vec![Vec::<(usize, f32)>::new(); cells.len()];
    let vertical = config.max_link_cells.max(1);
    let horizontal = config.horizontal_hop_cells.max(1);
    let base_hop = config.node_spacing_cells.max(1);
    for (index, &cell) in cells.iter().enumerate() {
        for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
            for hop in 1..=horizontal {
                for dz in -vertical..=vertical {
                    if hop > base_hop && dz == 0 {
                        continue;
                    }
                    let Some(&other) = lookup.get(&SurfaceCell::new(
                        cell.x + dx * hop,
                        cell.y + dy * hop,
                        cell.z + dz,
                    )) else {
                        continue;
                    };
                    let rise = dz.unsigned_abs() as f32;
                    if config.max_step_cells > 0 && rise > config.max_step_cells as f32 {
                        continue;
                    }
                    if config.max_slope > 0.0
                        && rise > 0.0
                        && rise > config.max_slope * hop as f32 + 1e-6
                    {
                        continue;
                    }
                    adjacency[index].push((other, ((hop * hop) as f32 + rise * rise).sqrt()));
                }
            }
        }
        adjacency[index].sort_by_key(|&(other, cost)| (other, cost.to_bits()));
    }
    adjacency
}
