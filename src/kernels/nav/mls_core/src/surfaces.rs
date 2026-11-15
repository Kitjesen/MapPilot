use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};
use std::ops::Bound::Included;

use crate::{SurfaceCell, SurfaceConfig, SurfaceMap};

/// Extract observed surfaces and bounded inferred cells from occupied voxels.
pub fn extract_surfaces(occupied: &[SurfaceCell], config: SurfaceConfig) -> SurfaceMap {
    extract_surfaces_with_free(occupied, &[], config)
}

/// Extract surfaces while preserving known-free evidence for inferred cells.
pub fn extract_surfaces_with_free(
    occupied: &[SurfaceCell],
    known_free: &[SurfaceCell],
    config: SurfaceConfig,
) -> SurfaceMap {
    let columns = columns(occupied);
    let mut observed = Vec::new();
    // Keep height layers ordered so hole closing only visits the slope band.
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
    if config.closing_passes == 0 || config.max_hole_cells == 0 {
        return Vec::new();
    }

    let radius = config.closing_passes as i32;
    let max_slope = f64::from(config.max_slope);
    if !max_slope.is_finite() || max_slope < 0.0 || by_z.is_empty() {
        return Vec::new();
    }
    let occupied: HashSet<SurfaceCell> = occupied.iter().copied().collect();
    let known_free: HashSet<SurfaceCell> = known_free.iter().copied().collect();
    let band = (f64::from(radius) * max_slope).ceil() as i32 + 1;
    let mut inferred = HashSet::new();
    let min_z = *by_z.keys().min().unwrap();
    let max_z = *by_z.keys().max().unwrap();
    let mut candidate_z = BTreeSet::new();
    // Heights outside these bands have no samples and cannot infer holes.
    for &layer_z in by_z.keys() {
        for offset in -band..=band {
            let z = layer_z.saturating_add(offset);
            if z >= min_z && z <= max_z {
                candidate_z.insert(z);
            }
        }
    }
    for z in candidate_z {
        let mut samples = HashMap::<(i32, i32), SurfaceCell>::new();
        for (_, layer) in by_z.range((Included(z - band), Included(z + band))) {
            for &cell in layer {
                let key = (cell.x, cell.y);
                let replace = samples
                    .get(&key)
                    .map(|current| (cell.z - z).abs() < (current.z - z).abs())
                    .unwrap_or(true);
                if replace {
                    samples.insert(key, cell);
                }
            }
        }
        let Some(first) = samples.values().next() else {
            continue;
        };
        let (first_x, first_y) = (first.x, first.y);
        let (mut min_x, mut max_x, mut min_y, mut max_y) = (first_x, first_x, first_y, first_y);
        for cell in samples.values() {
            min_x = min_x.min(cell.x);
            max_x = max_x.max(cell.x);
            min_y = min_y.min(cell.y);
            max_y = max_y.max(cell.y);
        }
        min_x -= 1;
        min_y -= 1;
        max_x += 1;
        max_y += 1;

        let sample_at = |x: i32, y: i32| samples.get(&(x, y)).map(|cell| cell.z);
        let mut visited = HashSet::new();
        for sy in min_y..=max_y {
            for sx in min_x..=max_x {
                if sample_at(sx, sy).is_some() || !visited.insert((sx, sy)) {
                    continue;
                }
                let mut queue = VecDeque::from([(sx, sy)]);
                let mut component = Vec::new();
                let mut rim = HashSet::new();
                let mut touches_boundary = false;
                while let Some((cx, cy)) = queue.pop_front() {
                    component.push((cx, cy));
                    touches_boundary |= cx == min_x || cx == max_x || cy == min_y || cy == max_y;
                    for (nx, ny) in neighbors(cx, cy) {
                        if nx < min_x || nx > max_x || ny < min_y || ny > max_y {
                            continue;
                        }
                        if sample_at(nx, ny).is_some() {
                            rim.insert((nx, ny));
                        } else if visited.insert((nx, ny)) {
                            queue.push_back((nx, ny));
                        }
                    }
                }
                if touches_boundary || component.len() > config.max_hole_cells {
                    continue;
                }
                let low_x = component.iter().map(|(x, _)| *x).min().unwrap();
                let high_x = component.iter().map(|(x, _)| *x).max().unwrap();
                let low_y = component.iter().map(|(_, y)| *y).min().unwrap();
                let high_y = component.iter().map(|(_, y)| *y).max().unwrap();
                if high_x - low_x + 1 > radius * 2 || high_y - low_y + 1 > radius * 2 {
                    continue;
                }
                let center_x = 0.5 * f64::from(low_x + high_x);
                let center_y = 0.5 * f64::from(low_y + high_y);
                let boundary: Vec<(f64, f64, f64)> = rim
                    .iter()
                    .filter_map(|&(x, y)| {
                        sample_at(x, y).map(|sample_z| {
                            (
                                f64::from(x) - center_x,
                                f64::from(y) - center_y,
                                f64::from(sample_z) + 0.5,
                            )
                        })
                    })
                    .collect();
                let Some((a, b, c, rms)) = fit_boundary_plane(&boundary) else {
                    continue;
                };
                if (a * a + b * b).sqrt() > max_slope + 1e-7 || rms > 0.35 {
                    continue;
                }
                if boundary
                    .iter()
                    .any(|&(x, y, sample_z)| (sample_z - (a * x + b * y + c)).abs() > 0.6)
                {
                    continue;
                }
                for (x, y) in component {
                    let fitted = a * (f64::from(x) - center_x) + b * (f64::from(y) - center_y) + c;
                    if fitted.floor() as i32 != z {
                        continue;
                    }
                    if !rim.iter().any(|&(rx, ry)| {
                        f64::from(x - rx).hypot(f64::from(y - ry)) <= f64::from(radius) + 1e-7
                    }) {
                        continue;
                    }
                    let candidate = SurfaceCell::new(x, y, z);
                    if known_free.contains(&candidate) || occupied.contains(&candidate) {
                        continue;
                    }
                    if (1..=config.clearance_cells)
                        .any(|offset| occupied.contains(&SurfaceCell::new(x, y, z + offset)))
                    {
                        continue;
                    }
                    inferred.insert(candidate);
                }
            }
        }
    }

    let mut cells: Vec<_> = inferred.into_iter().collect();
    cells.sort_unstable();
    cells
}

fn fit_boundary_plane(points: &[(f64, f64, f64)]) -> Option<(f64, f64, f64, f64)> {
    if points.len() < 3 {
        return None;
    }
    let count = points.len() as f64;
    let (mx, my, mz) = points.iter().fold((0.0, 0.0, 0.0), |acc, point| {
        (acc.0 + point.0, acc.1 + point.1, acc.2 + point.2)
    });
    let (mx, my, mz) = (mx / count, my / count, mz / count);
    let mut xx = 0.0;
    let mut yy = 0.0;
    let mut xy = 0.0;
    let mut xz = 0.0;
    let mut yz = 0.0;
    for &(x, y, z) in points {
        let (x, y, z) = (x - mx, y - my, z - mz);
        xx += x * x;
        yy += y * y;
        xy += x * y;
        xz += x * z;
        yz += y * z;
    }
    let eigen_min = (xx + yy - (xx - yy).hypot(2.0 * xy)) / (2.0 * count);
    if eigen_min < 1e-5 {
        return None;
    }
    let determinant = xx * yy - xy * xy;
    if determinant.abs() < 1e-12 {
        return None;
    }
    let a = (xz * yy - yz * xy) / determinant;
    let b = (yz * xx - xz * xy) / determinant;
    let c = mz - a * mx - b * my;
    let rms = (points
        .iter()
        .map(|&(x, y, z)| {
            let error = z - (a * x + b * y + c);
            error * error
        })
        .sum::<f64>()
        / count)
        .sqrt();
    Some((a, b, c, rms))
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
