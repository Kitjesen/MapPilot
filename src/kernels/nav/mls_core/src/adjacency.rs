use std::collections::HashMap;

use crate::SurfaceCell;

pub(crate) fn build_adjacency(cells: &[SurfaceCell], step: i32) -> Vec<Vec<(usize, f32)>> {
    let lookup: HashMap<SurfaceCell, usize> = cells
        .iter()
        .copied()
        .enumerate()
        .map(|(index, cell)| (cell, index))
        .collect();
    let mut adjacency = vec![Vec::<(usize, f32)>::new(); cells.len()];
    for (index, &cell) in cells.iter().enumerate() {
        for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
            for dz in -step..=step {
                let Some(&other) =
                    lookup.get(&SurfaceCell::new(cell.x + dx, cell.y + dy, cell.z + dz))
                else {
                    continue;
                };
                let rise = dz.unsigned_abs() as f32;
                adjacency[index].push((other, (2.0_f32 + rise * rise).sqrt()));
            }
        }
        adjacency[index].sort_by_key(|&(other, cost)| (other, cost.to_bits()));
    }
    adjacency
}
