//! Portable path safety kernel for LingTu.
//!
//! This crate mirrors the 2D subset of `nav.services.safety.plan_safety.evaluate_plan_safety`.
//! It has no ROS, Python, NumPy, or platform dependencies and exposes both a
//! Rust API and a small C ABI for future adapters.

use core::ffi::c_int;

pub const LT_PATH_SAFETY_ABI_VERSION: u32 = 1;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IndexOrder {
    Yx,
    Xy,
}

impl IndexOrder {
    fn from_u32(value: u32) -> Option<Self> {
        match value {
            0 => Some(Self::Yx),
            1 => Some(Self::Xy),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: Option<f64>,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y, z: None }
    }

    pub fn with_z(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z: Some(z) }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SampleReason {
    Ok,
    XyOutOfBounds,
    CostGeThreshold,
}

impl SampleReason {
    fn code(self) -> i32 {
        match self {
            Self::Ok => 0,
            Self::XyOutOfBounds => 1,
            Self::CostGeThreshold => 2,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CostSample {
    pub x: f64,
    pub y: f64,
    pub z: Option<f64>,
    pub cost: f32,
    pub row: i32,
    pub col: i32,
    pub reason: SampleReason,
}

#[derive(Clone, Debug)]
pub struct PlanSafetyGrid<'a> {
    pub costs: &'a [f32],
    pub rows: usize,
    pub cols: usize,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub index_order: IndexOrder,
}

impl<'a> PlanSafetyGrid<'a> {
    pub fn sample_cost(&self, point: Point) -> CostSample {
        let col = round_ties_to_even_i32((point.x - self.origin_x) / self.resolution);
        let row = round_ties_to_even_i32((point.y - self.origin_y) / self.resolution);

        let cost = match self.index_order {
            IndexOrder::Yx => {
                if row < 0 || col < 0 || row >= self.rows as i32 || col >= self.cols as i32 {
                    None
                } else {
                    self.costs.get(row as usize * self.cols + col as usize).copied()
                }
            }
            IndexOrder::Xy => {
                let ix = col;
                let iy = row;
                if ix < 0 || iy < 0 || ix >= self.rows as i32 || iy >= self.cols as i32 {
                    None
                } else {
                    self.costs.get(ix as usize * self.cols + iy as usize).copied()
                }
            }
        };

        match cost {
            Some(value) => CostSample {
                x: point.x,
                y: point.y,
                z: point.z,
                cost: value,
                row,
                col,
                reason: SampleReason::Ok,
            },
            None => CostSample {
                x: point.x,
                y: point.y,
                z: point.z,
                cost: f32::INFINITY,
                row,
                col,
                reason: SampleReason::XyOutOfBounds,
            },
        }
    }
}

fn round_ties_to_even_i32(value: f64) -> i32 {
    if !value.is_finite() {
        return if value.is_sign_negative() {
            i32::MIN
        } else {
            i32::MAX
        };
    }

    let floor = value.floor();
    let fraction = value - floor;
    let rounded = if fraction == 0.5 {
        let floor_i = floor as i64;
        if floor_i % 2 == 0 {
            floor
        } else {
            floor + 1.0
        }
    } else {
        value.round()
    };

    if rounded < i32::MIN as f64 {
        i32::MIN
    } else if rounded > i32::MAX as f64 {
        i32::MAX
    } else {
        rounded as i32
    }
}

#[derive(Clone, Debug)]
pub struct PlanSafetyReport {
    pub ok: bool,
    pub max_cost: f32,
    pub blocked_sample_count: usize,
    pub sample_count: usize,
    pub first_blocked: Option<CostSample>,
}

pub fn path_samples(path: &[Point], max_step_m: f64) -> Vec<Point> {
    if path.is_empty() {
        return Vec::new();
    }
    let step = max_step_m.max(1e-6);
    let mut samples = vec![path[0]];
    for window in path.windows(2) {
        let start = window[0];
        let end = window[1];
        let dist = (end.x - start.x).hypot(end.y - start.y);
        let steps = ((dist / step).ceil() as usize).max(1);
        for idx in 1..=steps {
            let alpha = idx as f64 / steps as f64;
            let z = match (start.z, end.z) {
                (Some(a), Some(b)) => Some(a + (b - a) * alpha),
                _ => None,
            };
            samples.push(Point {
                x: start.x + (end.x - start.x) * alpha,
                y: start.y + (end.y - start.y) * alpha,
                z,
            });
        }
    }
    samples
}

pub fn evaluate_plan_safety(
    path: &[Point],
    grid: &PlanSafetyGrid<'_>,
    obstacle_thr: f32,
    max_step_m: Option<f64>,
) -> PlanSafetyReport {
    if path.is_empty() {
        return PlanSafetyReport {
            ok: false,
            max_cost: f32::NAN,
            blocked_sample_count: 0,
            sample_count: 0,
            first_blocked: None,
        };
    }

    let step = max_step_m.unwrap_or_else(|| (grid.resolution * 0.5).max(0.05));
    let samples = path_samples(path, step);
    let mut blocked_count = 0usize;
    let mut first_blocked = None;
    let mut max_cost = f32::NEG_INFINITY;

    for point in samples.iter().copied() {
        let mut sample = grid.sample_cost(point);
        if sample.cost.is_finite() {
            max_cost = max_cost.max(sample.cost);
        }
        let blocked = !sample.cost.is_finite() || sample.cost >= obstacle_thr;
        if blocked {
            if sample.reason == SampleReason::Ok {
                sample.reason = SampleReason::CostGeThreshold;
            }
            blocked_count += 1;
            if first_blocked.is_none() {
                first_blocked = Some(sample);
            }
        }
    }

    PlanSafetyReport {
        ok: blocked_count == 0,
        max_cost: if max_cost.is_finite() { max_cost } else { f32::INFINITY },
        blocked_sample_count: blocked_count,
        sample_count: samples.len(),
        first_blocked,
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LtPathSafetyPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub has_z: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LtPathSafetyGrid {
    pub costs: *const f32,
    pub rows: u32,
    pub cols: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub index_order: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LtPathSafetyReport {
    pub ok: u32,
    pub max_cost: f32,
    pub blocked_sample_count: u32,
    pub sample_count: u32,
    pub first_blocked_x: f64,
    pub first_blocked_y: f64,
    pub first_blocked_z: f64,
    pub first_blocked_has_z: u32,
    pub first_blocked_cost: f32,
    pub first_blocked_row: i32,
    pub first_blocked_col: i32,
    pub first_blocked_reason: i32,
}

impl Default for LtPathSafetyReport {
    fn default() -> Self {
        Self {
            ok: 0,
            max_cost: f32::NAN,
            blocked_sample_count: 0,
            sample_count: 0,
            first_blocked_x: 0.0,
            first_blocked_y: 0.0,
            first_blocked_z: 0.0,
            first_blocked_has_z: 0,
            first_blocked_cost: f32::NAN,
            first_blocked_row: -1,
            first_blocked_col: -1,
            first_blocked_reason: SampleReason::Ok.code(),
        }
    }
}

#[no_mangle]
pub extern "C" fn lt_path_safety_abi_version() -> u32 {
    LT_PATH_SAFETY_ABI_VERSION
}

#[no_mangle]
pub extern "C" fn lt_path_safety_sizeof_point() -> u32 {
    core::mem::size_of::<LtPathSafetyPoint>() as u32
}

#[no_mangle]
pub extern "C" fn lt_path_safety_sizeof_grid() -> u32 {
    core::mem::size_of::<LtPathSafetyGrid>() as u32
}

#[no_mangle]
pub extern "C" fn lt_path_safety_sizeof_report() -> u32 {
    core::mem::size_of::<LtPathSafetyReport>() as u32
}

#[no_mangle]
pub unsafe extern "C" fn lt_path_safety_evaluate_2d(
    points: *const LtPathSafetyPoint,
    point_count: u32,
    grid: *const LtPathSafetyGrid,
    obstacle_thr: f32,
    max_step_m: f64,
    out_report: *mut LtPathSafetyReport,
) -> c_int {
    if points.is_null() || grid.is_null() || out_report.is_null() {
        return -1;
    }

    let raw_grid = *grid;
    let index_order = match IndexOrder::from_u32(raw_grid.index_order) {
        Some(value) => value,
        None => return -2,
    };
    if raw_grid.costs.is_null()
        || raw_grid.rows == 0
        || raw_grid.cols == 0
        || !raw_grid.resolution.is_finite()
        || raw_grid.resolution <= 0.0
    {
        return -3;
    }

    let point_slice = core::slice::from_raw_parts(points, point_count as usize);
    let costs =
        core::slice::from_raw_parts(raw_grid.costs, raw_grid.rows as usize * raw_grid.cols as usize);
    let path: Vec<Point> = point_slice
        .iter()
        .map(|point| Point {
            x: point.x,
            y: point.y,
            z: if point.has_z != 0 { Some(point.z) } else { None },
        })
        .collect();
    let grid_view = PlanSafetyGrid {
        costs,
        rows: raw_grid.rows as usize,
        cols: raw_grid.cols as usize,
        resolution: raw_grid.resolution,
        origin_x: raw_grid.origin_x,
        origin_y: raw_grid.origin_y,
        index_order,
    };
    let step = if max_step_m.is_finite() && max_step_m > 0.0 {
        Some(max_step_m)
    } else {
        None
    };
    let report = evaluate_plan_safety(&path, &grid_view, obstacle_thr, step);
    let mut ffi_report = LtPathSafetyReport {
        ok: u32::from(report.ok),
        max_cost: report.max_cost,
        blocked_sample_count: report.blocked_sample_count as u32,
        sample_count: report.sample_count as u32,
        ..LtPathSafetyReport::default()
    };
    if let Some(sample) = report.first_blocked {
        ffi_report.first_blocked_x = sample.x;
        ffi_report.first_blocked_y = sample.y;
        ffi_report.first_blocked_z = sample.z.unwrap_or(0.0);
        ffi_report.first_blocked_has_z = u32::from(sample.z.is_some());
        ffi_report.first_blocked_cost = sample.cost;
        ffi_report.first_blocked_row = sample.row;
        ffi_report.first_blocked_col = sample.col;
        ffi_report.first_blocked_reason = sample.reason.code();
    }
    *out_report = ffi_report;
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blocks_yx_grid_path() {
        let grid_values = vec![
            0.0, 100.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0,
        ];
        let grid = PlanSafetyGrid {
            costs: &grid_values,
            rows: 3,
            cols: 4,
            resolution: 1.0,
            origin_x: 0.0,
            origin_y: 0.0,
            index_order: IndexOrder::Yx,
        };
        let report = evaluate_plan_safety(
            &[Point::with_z(0.0, 0.0, 0.0), Point::with_z(2.0, 0.0, 0.0)],
            &grid,
            49.9,
            Some(0.5),
        );
        assert!(!report.ok);
        assert_eq!(report.blocked_sample_count, 1);
        assert_eq!(report.sample_count, 5);
        let first = report.first_blocked.unwrap();
        assert_eq!(first.reason, SampleReason::CostGeThreshold);
        assert_eq!(first.row, 0);
        assert_eq!(first.col, 1);
    }

    #[test]
    fn supports_xy_order() {
        let grid_values = vec![
            0.0, 0.0, 0.0, //
            100.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0,
        ];
        let grid = PlanSafetyGrid {
            costs: &grid_values,
            rows: 4,
            cols: 3,
            resolution: 1.0,
            origin_x: 0.0,
            origin_y: 0.0,
            index_order: IndexOrder::Xy,
        };
        let report = evaluate_plan_safety(
            &[Point::with_z(0.0, 0.0, 0.0), Point::with_z(2.0, 0.0, 0.0)],
            &grid,
            49.9,
            Some(0.5),
        );
        assert!(!report.ok);
        assert_eq!(report.first_blocked.unwrap().col, 1);
    }

    #[test]
    fn c_abi_evaluates_same_path() {
        let costs = vec![0.0_f32, 100.0, 0.0, 0.0, 0.0, 0.0];
        let points = [
            LtPathSafetyPoint {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                has_z: 1,
            },
            LtPathSafetyPoint {
                x: 2.0,
                y: 0.0,
                z: 0.0,
                has_z: 1,
            },
        ];
        let grid = LtPathSafetyGrid {
            costs: costs.as_ptr(),
            rows: 2,
            cols: 3,
            resolution: 1.0,
            origin_x: 0.0,
            origin_y: 0.0,
            index_order: 0,
        };
        let mut report = LtPathSafetyReport::default();
        let rc = unsafe {
            lt_path_safety_evaluate_2d(
                points.as_ptr(),
                points.len() as u32,
                &grid,
                49.9,
                0.5,
                &mut report,
            )
        };
        assert_eq!(rc, 0);
        assert_eq!(report.ok, 0);
        assert_eq!(report.blocked_sample_count, 1);
        assert_eq!(report.first_blocked_reason, SampleReason::CostGeThreshold.code());
    }

    #[test]
    fn rounds_grid_coordinates_like_python() {
        assert_eq!(round_ties_to_even_i32(0.5), 0);
        assert_eq!(round_ties_to_even_i32(1.5), 2);
        assert_eq!(round_ties_to_even_i32(2.5), 2);
        assert_eq!(round_ties_to_even_i32(-0.5), 0);
        assert_eq!(round_ties_to_even_i32(-1.5), -2);
    }
}
