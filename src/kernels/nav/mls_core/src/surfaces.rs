use std::collections::{BTreeMap, HashMap, HashSet};

use crate::{SurfaceCell, SurfaceConfig, SurfaceMap};

/// Extract observed surfaces and inferred cells using the DimOS surface rules.
pub fn extract_surfaces(occupied: &[SurfaceCell], config: SurfaceConfig) -> SurfaceMap {
    extract_surfaces_with_free(occupied, &[], config)
}

/// Extract surfaces while excluding observed free cells from inferred support.
pub fn extract_surfaces_with_free(
    occupied: &[SurfaceCell],
    known_free: &[SurfaceCell],
    config: SurfaceConfig,
) -> SurfaceMap {
    let columns = columns(occupied);
    let mut observed = Vec::new();
    let mut by_z: BTreeMap<i32, Vec<SurfaceCell>> = BTreeMap::new();

    for (&(x, y), levels) in &columns {
        for (index, &z) in levels.iter().enumerate() {
            if index + 1 == levels.len() || levels[index + 1] - z > config.clearance_cells {
                let cell = SurfaceCell::new(x, y, z);
                observed.push(cell);
                by_z.entry(z).or_default().push(cell);
            }
        }
    }
    observed.sort_unstable();

    let inferred = close_holes(occupied, known_free, &by_z, config);
    SurfaceMap { observed, inferred }
}

fn columns(occupied: &[SurfaceCell]) -> HashMap<(i32, i32), Vec<i32>> {
    let mut columns: HashMap<(i32, i32), Vec<i32>> = HashMap::new();
    for cell in occupied {
        columns.entry((cell.x, cell.y)).or_default().push(cell.z);
    }
    for levels in columns.values_mut() {
        levels.sort_unstable();
        levels.dedup();
    }
    columns
}

fn close_holes(
    occupied: &[SurfaceCell],
    known_free: &[SurfaceCell],
    by_z: &BTreeMap<i32, Vec<SurfaceCell>>,
    config: SurfaceConfig,
) -> Vec<SurfaceCell> {
    let passes = if config.closing_radius_cells > 0.0 {
        config.closing_radius_cells.ceil() as usize
    } else {
        config.closing_passes as usize
    };
    if passes == 0 {
        return Vec::new();
    }
    let columns = columns(occupied);
    let known_free: HashSet<_> = known_free.iter().copied().collect();
    let mut inferred = HashSet::new();
    for (&z, layer) in by_z {
        let original: HashSet<(i32, i32)> = layer.iter().map(|cell| (cell.x, cell.y)).collect();
        let min_x = original.iter().map(|(x, _)| *x).min().unwrap() - passes as i32;
        let max_x = original.iter().map(|(x, _)| *x).max().unwrap() + passes as i32;
        let min_y = original.iter().map(|(_, y)| *y).min().unwrap() - passes as i32;
        let max_y = original.iter().map(|(_, y)| *y).max().unwrap() + passes as i32;
        let mut closed = original.clone();
        for _ in 0..passes {
            let mut next = closed.clone();
            for &(x, y) in &closed {
                for (nx, ny) in neighbors(x, y) {
                    if nx >= min_x && nx <= max_x && ny >= min_y && ny <= max_y {
                        next.insert((nx, ny));
                    }
                }
            }
            closed = next;
        }
        for _ in 0..passes {
            let mut next = HashSet::new();
            for &(x, y) in &closed {
                if neighbors(x, y).iter().all(|cell| closed.contains(cell)) {
                    next.insert((x, y));
                }
            }
            closed = next;
        }
        for (x, y) in closed.difference(&original) {
            let candidate = SurfaceCell::new(*x, *y, z);
            if known_free.contains(&candidate)
                || !is_standable(candidate, &columns, config.clearance_cells)
                || !has_support(&columns, *x, *y, z)
            {
                continue;
            }
            inferred.insert(candidate);
        }
    }
    let mut cells: Vec<_> = inferred.into_iter().collect();
    cells.sort_unstable();
    cells
}

fn is_standable(
    cell: SurfaceCell,
    columns: &HashMap<(i32, i32), Vec<i32>>,
    clearance_cells: i32,
) -> bool {
    let Some(levels) = columns.get(&(cell.x, cell.y)) else {
        return true;
    };
    levels
        .iter()
        .copied()
        .find(|&z| z > cell.z)
        .map(|next| next - cell.z > clearance_cells)
        .unwrap_or(true)
}

fn has_support(columns: &HashMap<(i32, i32), Vec<i32>>, x: i32, y: i32, z: i32) -> bool {
    for dx in -3..=3 {
        for dy in -3..=3 {
            if columns
                .get(&(x + dx, y + dy))
                .is_some_and(|levels| levels.iter().any(|&occupied| (occupied - z).abs() <= 3))
            {
                return true;
            }
        }
    }
    false
}

pub(crate) fn neighbors(x: i32, y: i32) -> [(i32, i32); 4] {
    [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
}

pub(crate) fn squared_distance(a: SurfaceCell, b: SurfaceCell) -> i64 {
    let dx = i64::from(a.x) - i64::from(b.x);
    let dy = i64::from(a.y) - i64::from(b.y);
    let dz = i64::from(a.z) - i64::from(b.z);
    dx * dx + dy * dy + dz * dz
}
