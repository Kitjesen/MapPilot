use std::collections::{HashMap, HashSet};

use crate::{route_failure, LtMlsCell, LtMlsRouteCheckResult, PlannerState, SurfaceCell};
use crate::{ROUTE_EMPTY, ROUTE_INVALID_CONFIG, ROUTE_NO_GO, ROUTE_OCCUPIED, ROUTE_OK};
use crate::{ROUTE_SLOPE, ROUTE_SOURCE_UNAVAILABLE, ROUTE_STEP, ROUTE_UNSUPPORTED};

const OUTSIDE: u32 = 10;
const EPSILON: f64 = 1e-6;
pub(crate) const CANCELLED: u32 = 11;

pub type LtMlsCancelCheck = unsafe extern "C" fn(*mut std::ffi::c_void) -> i32;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LtMlsPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl LtMlsPoint {
    pub(crate) fn native(self) -> Self {
        Self {
            x: self.x as f32 as f64,
            y: self.y as f32 as f64,
            z: self.z as f32 as f64,
        }
    }

    fn interpolate(self, to: Self, t: f64) -> Self {
        Self {
            x: self.x + t * (to.x as f32 - self.x as f32) as f64,
            y: self.y + t * (to.y as f32 - self.y as f32) as f64,
            z: self.z + t * (to.z as f32 - self.z as f32) as f64,
        }
        .native()
    }

    pub(crate) fn finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsMetricRegion {
    pub min: LtMlsPoint,
    pub max: LtMlsPoint,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsPose {
    pub point: LtMlsPoint,
    pub yaw: f64,
    pub has_yaw: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct LtMlsMetricConfig {
    pub resolution: f64,
    pub robot_radius: f64,
    pub below: f64,
    pub above: f64,
    pub support_height: f64,
    pub support_tolerance: f64,
    pub max_step: f64,
    pub max_slope: f64,
    pub min: LtMlsPoint,
    pub max: LtMlsPoint,
    pub support_depth_cells: i32,
    pub support_patch_radius_cells: i32,
    pub support_patch_min_samples: i32,
    pub ground_support: u8,
    pub direct_support_only: u8,
    /// Apply the production planner's special handling to the first route point.
    pub actual_start: u8,
    pub body_offset: f64,
    pub start: LtMlsPose,
    pub start_exemption: u8,
    pub goal_tolerance: f64,
    pub goal_xy_tolerance: f64,
    pub goal_z_tolerance: f64,
}

pub(crate) struct Index {
    occupied: HashSet<SurfaceCell>,
    occupied_columns: HashMap<(i32, i32), Vec<i32>>,
    surface: HashMap<(i32, i32), Vec<i32>>,
    inferred: HashSet<SurfaceCell>,
}

impl Index {
    fn new(state: &PlannerState) -> Self {
        let mut occupied_columns: HashMap<(i32, i32), Vec<i32>> = HashMap::new();
        for cell in &state.occupied {
            occupied_columns
                .entry((cell.x, cell.y))
                .or_default()
                .push(cell.z);
        }
        for levels in occupied_columns.values_mut() {
            levels.sort_unstable();
            levels.dedup();
        }
        let mut surface: HashMap<(i32, i32), Vec<i32>> = HashMap::new();
        for cell in state
            .artifact
            .surface
            .observed
            .iter()
            .chain(state.artifact.surface.inferred.iter())
        {
            surface.entry((cell.x, cell.y)).or_default().push(cell.z);
        }
        Self {
            occupied: state.occupied.iter().copied().collect(),
            occupied_columns,
            surface,
            inferred: state.artifact.surface.inferred.iter().copied().collect(),
        }
    }
}

pub(crate) struct Checker<'a> {
    config: LtMlsMetricConfig,
    occupied: &'a HashSet<SurfaceCell>,
    occupied_columns: &'a HashMap<(i32, i32), Vec<i32>>,
    surface: &'a HashMap<(i32, i32), Vec<i32>>,
    inferred: &'a HashSet<SurfaceCell>,
    regions: &'a [LtMlsMetricRegion],
    preblocked: HashSet<SurfaceCell>,
    cancel: Option<LtMlsCancelCheck>,
    context: *mut std::ffi::c_void,
}

impl Checker<'_> {
    pub(crate) fn resolve_goal(
        &self,
        start: LtMlsPose,
        goal: LtMlsPose,
        snap: i32,
        budget: &mut crate::query::Budget,
    ) -> Result<LtMlsPose, crate::query::QueryError> {
        use crate::query::QueryError;
        let c = self.config;
        let allowed = |pose: LtMlsPose| {
            // Without a requested heading, the connector decides body orientation.
            if c.body_offset > 0.0 && pose.has_yaw == 0 {
                self.base_query(pose.point.native(), c.ground_support != 0) == ROUTE_OK
            } else {
                self.oriented_query(pose.point.native(), pose.yaw, c.ground_support != 0, true)
                    == ROUTE_OK
            }
        };
        if self.cancelled() {
            return Err(QueryError::Cancelled);
        }
        if allowed(goal) {
            return Ok(goal);
        }
        // Keep the existing map-spaced, actual-start-anchored terminal lattice.
        let r = c.resolution;
        let rz = if c.support_height > 0.0 && c.max_slope > 0.0 {
            r / (1.0 / c.max_slope).ceil().max(1.0)
        } else {
            r
        };
        let seed = LtMlsPoint {
            x: start.point.x + ((goal.point.x - start.point.x) / r).round() * r,
            y: start.point.y + ((goal.point.y - start.point.y) / r).round() * r,
            z: start.point.z + ((goal.point.z - start.point.z) / rz).round() * rz,
        };
        let radius = snap.min(((c.goal_tolerance / r).ceil() as i32).saturating_add(1));
        let vertical = ((radius as f64 * r / rz).ceil() as i32)
            .min(((c.goal_z_tolerance / rz).ceil() as i32).saturating_add(1));
        let mut candidates = Vec::new();
        for x in -radius..=radius {
            if self.cancelled() {
                return Err(QueryError::Cancelled);
            }
            for y in -radius..=radius {
                for z in -vertical..=vertical {
                    let point = LtMlsPoint {
                        x: seed.x + x as f64 * r,
                        y: seed.y + y as f64 * r,
                        z: seed.z + z as f64 * rz,
                    }
                    .native();
                    let xy = (point.x - goal.point.x).hypot(point.y - goal.point.y);
                    let dz = (point.z - goal.point.z).abs();
                    let distance = xy.hypot(dz);
                    if distance <= c.goal_tolerance + EPSILON
                        && xy <= c.goal_xy_tolerance + EPSILON
                        && dz <= c.goal_z_tolerance + EPSILON
                    {
                        candidates.push((distance, xy, z, x, y, LtMlsPose { point, ..goal }));
                    }
                }
            }
        }
        candidates.sort_by(|a, b| {
            a.0.total_cmp(&b.0)
                .then(a.1.total_cmp(&b.1))
                .then((a.2, a.3, a.4).cmp(&(b.2, b.3, b.4)))
        });
        for (_, _, _, _, _, candidate) in candidates {
            if self.cancelled() {
                return Err(QueryError::Cancelled);
            }
            budget.spend()?;
            if allowed(candidate) {
                return Ok(candidate);
            }
        }
        Err(QueryError::Exhausted)
    }

    fn occupied_z(&self, x: i32, y: i32, low: i32, high: i32) -> &[i32] {
        let Some(levels) = self.occupied_columns.get(&(x, y)) else {
            return &[];
        };
        let first = levels.partition_point(|&z| z < low);
        let end = levels.partition_point(|&z| z <= high);
        &levels[first..end]
    }

    fn cancelled(&self) -> bool {
        self.cancel
            .is_some_and(|cancel| unsafe { cancel(self.context) != 0 })
    }

    fn exempt(&self, cell: SurfaceCell) -> bool {
        let c = self.config;
        if c.start_exemption == 0 {
            return false;
        }
        let (below, above) = self.body_extent();
        let min = point_at_cell(cell.x, cell.y, cell.z, c.resolution);
        if min.z + c.resolution <= c.start.point.z - below || min.z >= c.start.point.z + above {
            return false;
        }
        let yaw = if c.start.has_yaw != 0 {
            c.start.yaw
        } else {
            0.0
        };
        [-c.body_offset, c.body_offset].into_iter().any(|offset| {
            let x = c.start.point.x + offset * yaw.cos();
            let y = c.start.point.y + offset * yaw.sin();
            let dx = (min.x - x).max(0.0).max(x - min.x - c.resolution);
            let dy = (min.y - y).max(0.0).max(y - min.y - c.resolution);
            dx * dx + dy * dy <= c.robot_radius * c.robot_radius
        })
    }
    fn cell(&self, point: LtMlsPoint) -> SurfaceCell {
        let r = self.config.resolution;
        SurfaceCell::new(
            (point.x / r).floor() as i32,
            (point.y / r).floor() as i32,
            (point.z / r).floor() as i32,
        )
    }

    fn body_extent(&self) -> (f64, f64) {
        let c = self.config;
        if c.below > 0.0 || c.above > 0.0 {
            (c.below, c.above)
        } else {
            (c.resolution, c.resolution)
        }
    }

    fn support_range(&self, point: LtMlsPoint) -> (i32, i32) {
        let c = self.config;
        if c.support_height <= 0.0 {
            return (1, c.support_depth_cells.max(1));
        }
        let cell = self.cell(point);
        let offset = (cell.z as f64 + 0.5) * c.resolution - point.z;
        let minimum = (c.support_height - c.support_tolerance - 0.5 * c.resolution).max(0.0);
        let maximum = c.support_height + c.support_tolerance + 0.5 * c.resolution;
        let low = (((minimum + offset) / c.resolution - 1e-6).ceil() as i32).max(1);
        let high = (((maximum + offset) / c.resolution + 1e-6).floor() as i32).max(1);
        if high >= low {
            return (low, high);
        }
        let nearest = (((c.support_height + offset) / c.resolution).round() as i32).max(1);
        (nearest, nearest)
    }

    fn inside(&self, point: LtMlsPoint) -> bool {
        let c = self.config;
        point.x >= c.min.x
            && point.x < c.max.x
            && point.y >= c.min.y
            && point.y < c.max.y
            && point.z >= c.min.z
            && point.z < c.max.z
    }

    fn support_patch(&self, cell: SurfaceCell) -> bool {
        let c = self.config;
        let radius = c.support_patch_radius_cells;
        let required = c.support_patch_min_samples;
        if radius == 0 || required <= 1 {
            return true;
        }
        let samples = [(0, 0), (radius, 0), (-radius, 0), (0, radius), (0, -radius)];
        let count = samples
            .iter()
            .filter(|&&(dx, dy)| {
                (-1..=1).any(|dz| {
                    self.occupied
                        .contains(&SurfaceCell::new(cell.x + dx, cell.y + dy, cell.z + dz))
                })
            })
            .count();
        count >= required.min(5) as usize
    }

    fn observed_support(&self, cell: SurfaceCell) -> bool {
        self.occupied.contains(&cell)
            && !self
                .occupied
                .contains(&SurfaceCell::new(cell.x, cell.y, cell.z + 1))
            && self.support_patch(cell)
    }

    // A support voxel is one map cell thick. Body clearance that reaches the
    // physical surface therefore overlaps the top of the voxel the stance is
    // standing on. That overlap is terrain. Cells above the stance's support
    // band remain obstacles.
    fn is_support_terrain(&self, cell: SurfaceCell, support_z: Option<i32>) -> bool {
        if self.config.support_height <= 0.0 {
            return false;
        }
        support_z.is_some_and(|limit| cell.z <= limit && self.observed_support(cell))
    }

    fn segment_support_z(&self, from: LtMlsPoint, to: LtMlsPoint) -> Option<i32> {
        match (self.floor(from), self.floor(to)) {
            (Some(a), Some(b)) => Some(a.max(b)),
            (Some(z), None) | (None, Some(z)) => Some(z),
            (None, None) => None,
        }
    }

    fn floor(&self, point: LtMlsPoint) -> Option<i32> {
        let cell = self.cell(point);
        let (low, high) = self.support_range(point);
        if self.config.direct_support_only == 0 {
            let z = self
                .surface
                .get(&(cell.x, cell.y))?
                .iter()
                .copied()
                .filter(|&z| z <= cell.z)
                .max()?;
            return (z >= cell.z - high && z <= cell.z - low).then_some(z);
        }
        (low..=high)
            .map(|depth| cell.z - depth)
            .find(|&z| self.observed_support(SurfaceCell::new(cell.x, cell.y, z)))
    }

    pub(crate) fn observed_floor(&self, point: LtMlsPoint) -> Option<i32> {
        let cell = self.cell(point);
        let (low, high) = self.support_range(point);
        (low..=high)
            .map(|depth| cell.z - depth)
            .find(|&z| self.observed_support(SurfaceCell::new(cell.x, cell.y, z)))
    }

    pub(crate) fn attachment(&self, point: LtMlsPoint) -> Option<SurfaceCell> {
        let cell = self.cell(point);
        if let Some(z) = self.floor(point) {
            return Some(SurfaceCell::new(cell.x, cell.y, z));
        }
        if self.config.direct_support_only != 0 {
            return None;
        }
        let (low, high) = self.support_range(point);
        (low..=high)
            .map(|depth| SurfaceCell::new(cell.x, cell.y, cell.z - depth))
            .find(|surface| self.inferred.contains(surface))
    }

    fn supported(&self, point: LtMlsPoint) -> bool {
        let cell = self.cell(point);
        if self.config.direct_support_only == 0 {
            let (low, high) = self.support_range(point);
            if (low..=high).any(|depth| {
                self.inferred
                    .contains(&SurfaceCell::new(cell.x, cell.y, cell.z - depth))
            }) {
                return true;
            }
        }
        self.floor(point).is_some_and(|z| {
            (z + 1 != cell.z || !self.occupied.contains(&cell))
                && self.support_patch(SurfaceCell::new(cell.x, cell.y, z))
        })
    }

    fn base_query(&self, point: LtMlsPoint, support: bool) -> u32 {
        if self.cancelled() {
            return CANCELLED;
        }
        if !self.inside(point) {
            return OUTSIDE;
        }
        let cell = self.cell(point);
        for z in (self.cell(self.config.min).z..cell.z).rev() {
            let below = SurfaceCell::new(cell.x, cell.y, z);
            if self.occupied.contains(&below) {
                break;
            }
            if self.preblocked.contains(&below) {
                return ROUTE_NO_GO;
            }
            if self.preblocked.is_empty() {
                break;
            }
        }
        if support && !self.supported(point) {
            return ROUTE_UNSUPPORTED;
        }
        ROUTE_OK
    }

    fn body_query(&self, point: LtMlsPoint, radius: f64) -> u32 {
        let c = self.config;
        let (below, above) = self.body_extent();
        let support_z = self.floor(point);
        let low = self.cell(LtMlsPoint {
            x: point.x - radius,
            y: point.y - radius,
            z: point.z - below,
        });
        let high = self.cell(LtMlsPoint {
            x: point.x + radius,
            y: point.y + radius,
            z: point.z + above,
        });
        for x in low.x..=high.x {
            if self.cancelled() {
                return CANCELLED;
            }
            let dx = (x as f64 * c.resolution - point.x)
                .max(0.0)
                .max(point.x - (x as f64 + 1.0) * c.resolution);
            for y in low.y..=high.y {
                let dy = (y as f64 * c.resolution - point.y)
                    .max(0.0)
                    .max(point.y - (y as f64 + 1.0) * c.resolution);
                if dx * dx + dy * dy > radius * radius + 1e-9 {
                    continue;
                }
                if self.preblocked.is_empty() {
                    for &z in self.occupied_z(x, y, low.z, high.z) {
                        if (z as f64 + 1.0) * c.resolution <= point.z - below + 1e-7
                            || z as f64 * c.resolution >= point.z + above - 1e-7
                        {
                            continue;
                        }
                        let cell = SurfaceCell::new(x, y, z);
                        if !self.exempt(cell) && !self.is_support_terrain(cell, support_z) {
                            return ROUTE_OCCUPIED;
                        }
                    }
                    continue;
                }
                for z in low.z..=high.z {
                    if (z as f64 + 1.0) * c.resolution <= point.z - below + 1e-7
                        || z as f64 * c.resolution >= point.z + above - 1e-7
                    {
                        continue;
                    }
                    let cell = SurfaceCell::new(x, y, z);
                    if self.preblocked.contains(&cell) {
                        return ROUTE_NO_GO;
                    }
                    if self.occupied.contains(&cell)
                        && !self.exempt(cell)
                        && !self.is_support_terrain(cell, support_z)
                    {
                        return ROUTE_OCCUPIED;
                    }
                }
            }
        }
        ROUTE_OK
    }

    fn query(&self, point: LtMlsPoint, radius: f64, support: bool, body: bool) -> u32 {
        let (below, above) = self.body_extent();
        if self
            .regions
            .iter()
            .any(|region| no_go(*region, point, point, radius, below, above))
        {
            return ROUTE_NO_GO;
        }
        let failure = self.base_query(point, support);
        if failure != ROUTE_OK || !body {
            return failure;
        }
        self.body_query(point, radius)
    }

    fn oriented_query(&self, point: LtMlsPoint, yaw: f64, support: bool, body: bool) -> u32 {
        let failure = self.base_query(point, support);
        if failure != ROUTE_OK {
            return failure;
        }
        let (below, above) = self.body_extent();
        for offset in self.offsets() {
            let center = shifted(point, offset, yaw);
            if self.regions.iter().any(|region| {
                no_go(
                    *region,
                    center,
                    center,
                    self.config.robot_radius,
                    below,
                    above,
                )
            }) {
                return ROUTE_NO_GO;
            }
            if body {
                let failure = self.body_query(center, self.config.robot_radius);
                if failure != ROUTE_OK {
                    return failure;
                }
            }
        }
        ROUTE_OK
    }

    fn offsets(&self) -> impl Iterator<Item = f64> {
        let offset = self.config.body_offset;
        [-offset, offset]
            .into_iter()
            .take(if offset == 0.0 { 1 } else { 2 })
    }

    fn metric(&self, from: LtMlsPoint, to: LtMlsPoint) -> u32 {
        let c = self.config;
        let rise = (to.z - from.z).abs();
        let horizontal = (to.x - from.x).hypot(to.y - from.y);
        if c.max_step > 0.0 && rise > c.max_step + EPSILON {
            return ROUTE_STEP;
        }
        if rise <= EPSILON {
            return ROUTE_OK;
        }
        if horizontal > EPSILON
            && (c.max_slope <= 0.0 || rise <= c.max_slope * horizontal + EPSILON)
        {
            return ROUTE_OK;
        }
        if c.ground_support != 0 && c.support_height > 0.0 && horizontal > EPSILON {
            let floor = |point| {
                let cell = self.cell(point);
                let (low, high) = self.support_range(point);
                let z = self
                    .surface
                    .get(&(cell.x, cell.y))
                    .and_then(|levels| levels.iter().copied().filter(|&z| z <= cell.z).max());
                match z {
                    Some(z) => (z >= cell.z - high && z <= cell.z - low).then_some(z),
                    None => (low..=high)
                        .map(|depth| cell.z - depth)
                        .find(|&z| self.observed_support(SurfaceCell::new(cell.x, cell.y, z))),
                }
            };
            if let (Some(a), Some(b)) = (floor(from), floor(to)) {
                if a == b {
                    return ROUTE_OK;
                }
            }
        }
        ROUTE_SLOPE
    }

    fn occupied_sweep(&self, from: LtMlsPoint, to: LtMlsPoint) -> Option<LtMlsPoint> {
        let c = self.config;
        let (below, above) = self.body_extent();
        let support_z = self.segment_support_z(from, to);
        let bottom = from.z.min(to.z) - below;
        let top = from.z.max(to.z) + above;
        let low = self.cell(LtMlsPoint {
            x: from.x.min(to.x) - c.robot_radius,
            y: from.y.min(to.y) - c.robot_radius,
            z: bottom,
        });
        let high = self.cell(LtMlsPoint {
            x: from.x.max(to.x) + c.robot_radius,
            y: from.y.max(to.y) + c.robot_radius,
            z: top,
        });
        for x in low.x..=high.x {
            if self.cancelled() {
                return Some(from);
            }
            for y in low.y..=high.y {
                for &z in self.occupied_z(x, y, low.z, high.z) {
                    let cell = SurfaceCell::new(x, y, z);
                    if self.exempt(cell) || self.is_support_terrain(cell, support_z) {
                        continue;
                    }
                    let minimum = point_at_cell(x, y, z, c.resolution);
                    let maximum = point_at_cell(x + 1, y + 1, z + 1, c.resolution);
                    // Match query()'s open vertical contact and its existing epsilon.
                    if maximum.z <= bottom + 1e-7 || minimum.z >= top - 1e-7 {
                        continue;
                    }
                    let region = LtMlsMetricRegion {
                        min: LtMlsPoint {
                            z: minimum.z + 1e-7,
                            ..minimum
                        },
                        max: LtMlsPoint {
                            z: maximum.z - 1e-7,
                            ..maximum
                        },
                    };
                    if no_go(region, from, to, c.robot_radius, below, above) {
                        return Some(LtMlsPoint {
                            x: minimum.x + 0.5 * c.resolution,
                            y: minimum.y + 0.5 * c.resolution,
                            z: minimum.z + 0.5 * c.resolution,
                        });
                    }
                }
            }
        }
        None
    }

    #[cfg(test)]
    fn occupied_sweep_dense(&self, from: LtMlsPoint, to: LtMlsPoint) -> Option<LtMlsPoint> {
        let c = self.config;
        let (below, above) = self.body_extent();
        let support_z = self.segment_support_z(from, to);
        let bottom = from.z.min(to.z) - below;
        let top = from.z.max(to.z) + above;
        let low = self.cell(LtMlsPoint {
            x: from.x.min(to.x) - c.robot_radius,
            y: from.y.min(to.y) - c.robot_radius,
            z: bottom,
        });
        let high = self.cell(LtMlsPoint {
            x: from.x.max(to.x) + c.robot_radius,
            y: from.y.max(to.y) + c.robot_radius,
            z: top,
        });
        for x in low.x..=high.x {
            if self.cancelled() {
                return Some(from);
            }
            for y in low.y..=high.y {
                for z in low.z..=high.z {
                    let cell = SurfaceCell::new(x, y, z);
                    if !self.occupied.contains(&cell)
                        || self.exempt(cell)
                        || self.is_support_terrain(cell, support_z)
                    {
                        continue;
                    }
                    let minimum = point_at_cell(x, y, z, c.resolution);
                    let maximum = point_at_cell(x + 1, y + 1, z + 1, c.resolution);
                    // Match query()'s open vertical contact and its existing epsilon.
                    if maximum.z <= bottom + 1e-7 || minimum.z >= top - 1e-7 {
                        continue;
                    }
                    let region = LtMlsMetricRegion {
                        min: LtMlsPoint {
                            z: minimum.z + 1e-7,
                            ..minimum
                        },
                        max: LtMlsPoint {
                            z: maximum.z - 1e-7,
                            ..maximum
                        },
                    };
                    if no_go(region, from, to, c.robot_radius, below, above) {
                        return Some(LtMlsPoint {
                            x: minimum.x + 0.5 * c.resolution,
                            y: minimum.y + 0.5 * c.resolution,
                            z: minimum.z + 0.5 * c.resolution,
                        });
                    }
                }
            }
        }
        None
    }

    fn sweep(
        &self,
        from: LtMlsPoint,
        to: LtMlsPoint,
        support: bool,
        yaw: f64,
    ) -> Option<(u32, LtMlsPoint)> {
        let c = self.config;
        if self.cancelled() {
            return Some((CANCELLED, from));
        }
        let (below, above) = self.body_extent();
        for offset in self.offsets() {
            let a = shifted(from, offset, yaw);
            let b = shifted(to, offset, yaw);
            if self
                .regions
                .iter()
                .any(|region| no_go(*region, a, b, c.robot_radius, below, above))
            {
                return Some((ROUTE_NO_GO, from.interpolate(to, 0.5)));
            }
            if let Some(cell_center) = self.occupied_sweep(a, b) {
                return Some((
                    if self.cancelled() {
                        CANCELLED
                    } else {
                        ROUTE_OCCUPIED
                    },
                    cell_center,
                ));
            }
        }
        let dx = (to.x - from.x) as f32;
        let dy = (to.y - from.y) as f32;
        let dz = (to.z - from.z) as f32;
        let length = (dx * dx + dy * dy + dz * dz).sqrt() as f64;
        let count = (2.0 * length / c.resolution).ceil().max(1.0) as usize;
        for i in 1..count {
            let point = from.interpolate(to, i as f64 / count as f64);
            let failure = self.oriented_query(
                point,
                yaw,
                support && c.ground_support != 0,
                !self.preblocked.is_empty(),
            );
            if failure == ROUTE_OK {
                continue;
            }
            if failure != ROUTE_UNSUPPORTED
                || (to.z - from.z).abs() < 1e-7
                || self.oriented_query(point, yaw, false, !self.preblocked.is_empty()) != ROUTE_OK
            {
                return Some((failure, point));
            }
            let lower = from.z.min(to.z);
            let upper = from.z.max(to.z);
            let levels = ((upper - lower) / c.resolution).ceil().max(1.0) as usize;
            let supported = (0..=levels).any(|level| {
                let stance = LtMlsPoint {
                    z: (lower + (upper - lower) * level as f64 / levels as f64) as f32 as f64,
                    ..point
                };
                self.query(stance, 0.0, true, true) == ROUTE_OK
            });
            if !supported {
                return Some((failure, point));
            }
        }
        None
    }
    fn rotation(&self, point: LtMlsPoint, from_yaw: f64, to_yaw: f64) -> Option<(u32, LtMlsPoint)> {
        if self.config.body_offset == 0.0 || from_yaw == to_yaw {
            return None;
        }
        let c = self.config;
        let (below, above) = self.body_extent();
        for offset in self.offsets() {
            if self.regions.iter().any(|region| {
                arc_box(
                    *region,
                    point,
                    offset,
                    from_yaw,
                    to_yaw,
                    c.robot_radius,
                    below,
                    above,
                )
            }) {
                return Some((ROUTE_NO_GO, point));
            }
        }
        let reach = c.body_offset + c.robot_radius;
        let support_z = self.floor(point);
        let low = self.cell(LtMlsPoint {
            x: point.x - reach,
            y: point.y - reach,
            z: point.z - below,
        });
        let high = self.cell(LtMlsPoint {
            x: point.x + reach,
            y: point.y + reach,
            z: point.z + above,
        });
        for x in low.x..=high.x {
            if self.cancelled() {
                return Some((CANCELLED, point));
            }
            for y in low.y..=high.y {
                for z in low.z..=high.z {
                    let cell = SurfaceCell::new(x, y, z);
                    let blocked = self.preblocked.contains(&cell);
                    if !blocked
                        && (!self.occupied.contains(&cell)
                            || self.exempt(cell)
                            || self.is_support_terrain(cell, support_z))
                    {
                        continue;
                    }
                    let min = point_at_cell(x, y, z, c.resolution);
                    let max = point_at_cell(x + 1, y + 1, z + 1, c.resolution);
                    if max.z <= point.z - below + 1e-7 || min.z >= point.z + above - 1e-7 {
                        continue;
                    }
                    let region = LtMlsMetricRegion {
                        min: LtMlsPoint {
                            z: min.z + 1e-7,
                            ..min
                        },
                        max: LtMlsPoint {
                            z: max.z - 1e-7,
                            ..max
                        },
                    };
                    if self.offsets().any(|offset| {
                        arc_box(
                            region,
                            point,
                            offset,
                            from_yaw,
                            to_yaw,
                            c.robot_radius,
                            below,
                            above,
                        )
                    }) {
                        return Some((
                            if blocked { ROUTE_NO_GO } else { ROUTE_OCCUPIED },
                            LtMlsPoint {
                                x: min.x + c.resolution * 0.5,
                                y: min.y + c.resolution * 0.5,
                                z: min.z + c.resolution * 0.5,
                            },
                        ));
                    }
                }
            }
        }
        None
    }
}

fn point_at_cell(x: i32, y: i32, z: i32, resolution: f64) -> LtMlsPoint {
    LtMlsPoint {
        x: x as f64 * resolution,
        y: y as f64 * resolution,
        z: z as f64 * resolution,
    }
}

fn shifted(point: LtMlsPoint, offset: f64, yaw: f64) -> LtMlsPoint {
    LtMlsPoint {
        x: (point.x as f32 + (offset * yaw.cos()) as f32) as f64,
        y: (point.y as f32 + (offset * yaw.sin()) as f32) as f64,
        ..point
    }
}

fn angle_delta(from: f64, to: f64) -> f64 {
    (to - from + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

fn arc_box(
    region: LtMlsMetricRegion,
    point: LtMlsPoint,
    offset: f64,
    from_yaw: f64,
    to_yaw: f64,
    radius: f64,
    below: f64,
    above: f64,
) -> bool {
    if point.z + above < region.min.z || point.z - below > region.max.z {
        return false;
    }
    let shift = if offset < 0.0 {
        std::f64::consts::PI
    } else {
        0.0
    };
    let from = from_yaw + shift;
    let delta = angle_delta(from_yaw, to_yaw);
    let reach = offset.abs();
    let contains = |angle: f64| {
        let along = if delta >= 0.0 {
            (angle - from).rem_euclid(std::f64::consts::TAU)
        } else {
            -(from - angle).rem_euclid(std::f64::consts::TAU)
        };
        if delta >= 0.0 {
            along <= delta
        } else {
            along >= delta
        }
    };
    let touches = |angle: f64| {
        let x = point.x + reach * angle.cos();
        let y = point.y + reach * angle.sin();
        let dx = x - x.clamp(region.min.x, region.max.x);
        let dy = y - y.clamp(region.min.y, region.max.y);
        dx * dx + dy * dy <= radius * radius
    };
    if touches(from) || touches(from + delta) {
        return true;
    }
    let candidate = |angle: f64| contains(angle) && touches(angle);
    for angle in [
        0.0,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
        -std::f64::consts::FRAC_PI_2,
    ] {
        if candidate(angle) {
            return true;
        }
    }
    for x in [region.min.x, region.max.x] {
        for y in [region.min.y, region.max.y] {
            if candidate((y - point.y).atan2(x - point.x)) {
                return true;
            }
        }
        if reach > 0.0 {
            let cosine = (x - point.x) / reach;
            if (-1.0..=1.0).contains(&cosine) {
                let angle = cosine.acos();
                if candidate(angle) || candidate(-angle) {
                    return true;
                }
            }
        }
    }
    if reach > 0.0 {
        for y in [region.min.y, region.max.y] {
            let sine = (y - point.y) / reach;
            if (-1.0..=1.0).contains(&sine) {
                let angle = sine.asin();
                if candidate(angle) || candidate(std::f64::consts::PI - angle) {
                    return true;
                }
            }
        }
    }
    false
}

fn no_go(
    region: LtMlsMetricRegion,
    from: LtMlsPoint,
    to: LtMlsPoint,
    radius: f64,
    below: f64,
    above: f64,
) -> bool {
    let mut enter: f64 = 0.0;
    let mut leave: f64 = 1.0;
    let dz = to.z - from.z;
    if dz == 0.0 {
        if from.z + above < region.min.z || from.z - below > region.max.z {
            return false;
        }
    } else {
        let first = (region.min.z - above - from.z) / dz;
        let last = (region.max.z + below - from.z) / dz;
        enter = enter.max(first.min(last));
        leave = leave.min(first.max(last));
        if enter > leave {
            return false;
        }
    }
    let a = [
        from.x + enter * (to.x - from.x),
        from.y + enter * (to.y - from.y),
    ];
    let b = [
        from.x + leave * (to.x - from.x),
        from.y + leave * (to.y - from.y),
    ];
    let minimum = [region.min.x, region.min.y];
    let maximum = [region.max.x, region.max.y];
    let (mut first, mut last) = (0.0_f64, 1.0_f64);
    let mut intersects = true;
    for axis in 0..2 {
        let delta = b[axis] - a[axis];
        if delta == 0.0 {
            if a[axis] < minimum[axis] || a[axis] > maximum[axis] {
                intersects = false;
                break;
            }
        } else {
            let p = (minimum[axis] - a[axis]) / delta;
            let q = (maximum[axis] - a[axis]) / delta;
            first = first.max(p.min(q));
            last = last.min(p.max(q));
            if first > last {
                intersects = false;
                break;
            }
        }
    }
    if intersects {
        return true;
    }
    let radius2 = radius * radius;
    for p in [a, b] {
        let dx = p[0] - p[0].clamp(region.min.x, region.max.x);
        let dy = p[1] - p[1].clamp(region.min.y, region.max.y);
        if dx * dx + dy * dy <= radius2 {
            return true;
        }
    }
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let length2 = dx * dx + dy * dy;
    for x in [region.min.x, region.max.x] {
        for y in [region.min.y, region.max.y] {
            let t = if length2 > 0.0 {
                (((x - a[0]) * dx + (y - a[1]) * dy) / length2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let ex = x - a[0] - t * dx;
            let ey = y - a[1] - t * dy;
            if ex * ex + ey * ey <= radius2 {
                return true;
            }
        }
    }
    false
}

pub(crate) fn prepare<'a>(
    state: &'a PlannerState,
    config: LtMlsMetricConfig,
    regions: &'a [LtMlsMetricRegion],
    preblocked: &[SurfaceCell],
    cancel: Option<LtMlsCancelCheck>,
    context: *mut std::ffi::c_void,
) -> Result<Checker<'a>, LtMlsRouteCheckResult> {
    let valid_config = config.resolution.is_finite()
        && config.resolution > 0.0
        && [
            config.robot_radius,
            config.below,
            config.above,
            config.support_height,
            config.support_tolerance,
            config.max_step,
            config.max_slope,
            config.body_offset,
        ]
        .iter()
        .all(|v| v.is_finite() && *v >= 0.0)
        && config.min.finite()
        && config.max.finite()
        && config.min.x < config.max.x
        && config.min.y < config.max.y
        && config.min.z < config.max.z
        && config.support_depth_cells >= 0
        && config.support_patch_radius_cells >= 0
        && config.support_patch_min_samples >= 0
        && config.start.point.finite()
        && [
            config.goal_tolerance,
            config.goal_xy_tolerance,
            config.goal_z_tolerance,
        ]
        .iter()
        .all(|v| !v.is_nan() && *v >= 0.0)
        && (config.start.has_yaw == 0 || config.start.yaw.is_finite())
        && regions.iter().all(|r| {
            r.min.finite()
                && r.max.finite()
                && r.min.x <= r.max.x
                && r.min.y <= r.max.y
                && r.min.z <= r.max.z
        });
    if !valid_config {
        return Err(route_failure(
            ROUTE_INVALID_CONFIG,
            -1,
            SurfaceCell::new(0, 0, 0),
        ));
    }
    if state.surface_source {
        return Err(route_failure(
            ROUTE_SOURCE_UNAVAILABLE,
            -1,
            SurfaceCell::new(0, 0, 0),
        ));
    }
    let index = state.geometry_index.get_or_init(|| Index::new(state));
    Ok(Checker {
        config,
        occupied: &index.occupied,
        occupied_columns: &index.occupied_columns,
        surface: &index.surface,
        inferred: &index.inferred,
        regions,
        preblocked: preblocked.iter().copied().collect(),
        cancel,
        context,
    })
}

pub(crate) fn check(
    state: &PlannerState,
    route: &[LtMlsPoint],
    config: LtMlsMetricConfig,
    regions: &[LtMlsMetricRegion],
) -> LtMlsRouteCheckResult {
    let poses: Vec<_> = route
        .iter()
        .map(|&point| LtMlsPose {
            point,
            ..LtMlsPose::default()
        })
        .collect();
    match prepare(state, config, regions, &[], None, std::ptr::null_mut()) {
        Ok(checker) => checker.check_poses(&poses),
        Err(failure) => failure,
    }
}

impl Checker<'_> {
    pub(crate) fn check_poses(&self, route: &[LtMlsPose]) -> LtMlsRouteCheckResult {
        self.check_poses_from(route, self.config.actual_start != 0)
    }

    pub(crate) fn check_connection(
        &self,
        from: LtMlsPose,
        to: LtMlsPose,
        actual_start: bool,
    ) -> LtMlsRouteCheckResult {
        self.check_poses_from(&[from, to], actual_start)
    }

    fn check_poses_from(
        &self,
        route: &[LtMlsPose],
        starts_at_actual: bool,
    ) -> LtMlsRouteCheckResult {
        if route.len() < 2 {
            return route_failure(ROUTE_EMPTY, -1, SurfaceCell::new(0, 0, 0));
        }
        if route
            .iter()
            .any(|p| !p.point.finite() || (p.has_yaw != 0 && !p.yaw.is_finite()))
        {
            return route_failure(ROUTE_INVALID_CONFIG, -1, SurfaceCell::new(0, 0, 0));
        }
        let config = self.config;
        let mut previous_heading = None;
        for (index, pair) in route.windows(2).enumerate() {
            if self.cancelled() {
                return route_failure(CANCELLED, index as i32, self.cell(pair[0].point));
            }
            let failure = self.metric(pair[0].point, pair[1].point);
            if failure != ROUTE_OK {
                return route_failure(failure, index as i32, self.cell(pair[1].point));
            }
            let from = pair[0].point.native();
            let to = pair[1].point.native();
            let dx = pair[1].point.x - pair[0].point.x;
            let dy = pair[1].point.y - pair[0].point.y;
            let heading = if dx.hypot(dy) > 1e-9 {
                dy.atan2(dx)
            } else {
                previous_heading.unwrap_or(if pair[0].has_yaw != 0 {
                    pair[0].yaw
                } else {
                    0.0
                })
            };
            // Start and intermediate vertices are not in-place pivots: SCAN
            // turns along a continuous spline and checks that sweep against
            // live occupancy. Only an explicit terminal yaw needs a turn here.
            let actual_start = index == 0 && starts_at_actual;
            let failure = self.oriented_query(
                from,
                heading,
                config.ground_support != 0 && !actual_start,
                !actual_start,
            );
            if failure != ROUTE_OK {
                return route_failure(failure, index as i32, self.cell(from));
            }
            if actual_start && from.x == to.x && from.y == to.y && from.z == to.z {
                previous_heading = Some(heading);
                continue;
            }
            let failure = self.oriented_query(to, heading, config.ground_support != 0, true);
            if failure != ROUTE_OK {
                return route_failure(failure, index as i32, self.cell(to));
            }
            if let Some((failure, point)) = self.sweep(from, to, true, heading) {
                return route_failure(failure, index as i32, self.cell(point));
            }
            previous_heading = Some(heading);
        }
        let last = route.last().unwrap();
        if last.has_yaw != 0 {
            if let Some((failure, point)) =
                self.rotation(last.point.native(), previous_heading.unwrap(), last.yaw)
            {
                return route_failure(failure, (route.len() - 2) as i32, self.cell(point));
            }
        }
        LtMlsRouteCheckResult {
            valid: 1,
            failure_kind: ROUTE_OK,
            failed_segment: -1,
            failed_cell: LtMlsCell::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{GraphConfig, SurfaceConfig};

    fn point(x: f64, y: f64, z: f64) -> LtMlsPoint {
        LtMlsPoint { x, y, z }
    }

    fn config() -> LtMlsMetricConfig {
        LtMlsMetricConfig {
            resolution: 0.2,
            robot_radius: 0.01,
            below: 0.05,
            above: 0.35,
            support_height: 0.35,
            support_tolerance: 0.01,
            max_step: 0.45,
            max_slope: 0.7,
            min: point(-2.0, -2.0, -2.0),
            max: point(4.0, 4.0, 4.0),
            support_depth_cells: 2,
            ground_support: 1,
            direct_support_only: 1,
            ..LtMlsMetricConfig::default()
        }
    }

    fn state(occupied: Vec<SurfaceCell>) -> PlannerState {
        PlannerState::new(
            occupied,
            Vec::new(),
            SurfaceConfig {
                clearance_cells: 0,
                ..SurfaceConfig::default()
            },
            GraphConfig::default(),
        )
    }

    #[test]
    fn body_height_uses_support_offset() {
        let state = state((0..6).map(|x| SurfaceCell::new(x, 0, 0)).collect());
        let route = [point(0.1, 0.1, 0.45), point(0.9, 0.1, 0.45)];
        assert_eq!(check(&state, &route, config(), &[]).valid, 1);
    }

    #[test]
    fn goal_region() {
        use crate::query::{Budget, QueryError};
        let mut cells: Vec<_> = (0..6).map(|x| SurfaceCell::new(x, 0, 0)).collect();
        cells.push(SurfaceCell::new(2, 0, 2));
        let state = state(cells);
        let start = pose(point(0.1, 0.1, 0.45), None);
        let goal = pose(point(0.5, 0.1, 0.45), None);
        let c = LtMlsMetricConfig {
            goal_tolerance: 0.21,
            goal_xy_tolerance: 0.21,
            goal_z_tolerance: 0.01,
            ..config()
        };
        let checker = prepare(&state, c, &[], &[], None, std::ptr::null_mut()).unwrap();
        let resolve = |checker: &Checker<'_>, target| {
            checker.resolve_goal(start, target, 4, &mut Budget { remaining: 100 })
        };
        assert_eq!(resolve(&checker, start).unwrap().point, start.point);
        let terminal = resolve(&checker, goal).unwrap();
        assert!((terminal.point.x - goal.point.x).abs() <= 0.21);
        assert_eq!(checker.check_connection(start, terminal, true).valid, 1);
        for limits in [
            LtMlsMetricConfig {
                goal_tolerance: 0.19,
                ..c
            },
            LtMlsMetricConfig {
                goal_xy_tolerance: 0.19,
                ..c
            },
        ] {
            let checker = prepare(&state, limits, &[], &[], None, std::ptr::null_mut()).unwrap();
            assert!(matches!(
                resolve(&checker, goal),
                Err(QueryError::Exhausted)
            ));
        }
        let region = [LtMlsMetricRegion {
            min: point(0.0, 0.0, 0.2),
            max: point(1.2, 0.2, 1.0),
        }];
        let checker = prepare(&state, c, &region, &[], None, std::ptr::null_mut()).unwrap();
        assert!(matches!(
            resolve(&checker, goal),
            Err(QueryError::Exhausted)
        ));
        assert!(matches!(
            checker.resolve_goal(start, goal, 4, &mut Budget { remaining: 0 }),
            Err(QueryError::Budget)
        ));
    }

    #[test]
    fn wall_between_endpoints_blocks_body_sweep() {
        let mut cells: Vec<_> = (0..6).map(|x| SurfaceCell::new(x, 0, 0)).collect();
        cells.push(SurfaceCell::new(2, 0, 2));
        let state = state(cells);
        let route = [point(0.1, 0.1, 0.45), point(0.9, 0.1, 0.45)];
        let result = check(&state, &route, config(), &[]);
        assert_eq!(result.failure_kind, ROUTE_OCCUPIED);
        assert_eq!(result.failed_segment, 0);
    }

    #[test]
    fn grazing_corner() {
        let wall = SurfaceCell::new(53, 23, 15);
        let state = state(vec![wall]);
        let from = point(5.7238657627, 2.4249119485, 1.585);
        let to = point(5.7032648953, 2.5036701332, 1.585);
        let c = LtMlsMetricConfig {
            resolution: 0.1,
            robot_radius: 0.32,
            ground_support: 0,
            max: point(10.0, 10.0, 10.0),
            ..config()
        };
        for route in [[from, to], [to, from]] {
            let result = check(&state, &route, c, &[]);
            assert_eq!(result.failure_kind, ROUTE_OCCUPIED);
            assert_eq!(result.failed_segment, 0);
            assert_eq!(result.failed_cell.x, wall.x);
            assert_eq!(result.failed_cell.y, wall.y);
            assert_eq!(result.failed_cell.z, wall.z);
        }
        for route in [
            [
                point(from.x + 0.01, from.y, from.z),
                point(to.x + 0.01, to.y, to.z),
            ],
            [point(from.x, from.y, 2.585), point(to.x, to.y, 2.585)],
        ] {
            assert_eq!(check(&state, &route, c, &[]).valid, 1);
        }
    }

    #[test]
    fn support_voxel_overlap_is_terrain() {
        let floor: Vec<_> = (0..8).map(|x| SurfaceCell::new(x, 0, 0)).collect();
        let mut c = config();
        c.resolution = 0.2;
        c.robot_radius = 0.4;
        c.below = 0.25;
        c.above = 0.35;
        c.support_height = 0.45;
        c.support_tolerance = 0.11;
        c.max_slope = 0.0;
        c.body_offset = 0.25;
        let route = [point(0.4, 0.1, 0.44), point(0.8, 0.1, 0.65)];
        assert_eq!(check(&state(floor.clone()), &route, c, &[]).valid, 1);
        let mut blocked = floor;
        blocked.push(SurfaceCell::new(3, 0, 2));
        assert_eq!(
            check(&state(blocked), &route, c, &[]).failure_kind,
            ROUTE_OCCUPIED
        );
    }

    #[test]
    fn vertical_contact() {
        let state = state(vec![SurfaceCell::new(2, 0, 0), SurfaceCell::new(2, 0, 5)]);
        let c = LtMlsMetricConfig {
            resolution: 0.1,
            ground_support: 0,
            ..config()
        };
        let route = [point(0.15, 0.05, 0.15), point(0.85, 0.05, 0.15)];
        assert_eq!(check(&state, &route, c, &[]).valid, 1);
        for z in [0.14999, 0.15001] {
            let route = [point(0.15, 0.05, z), point(0.85, 0.05, z)];
            assert_eq!(check(&state, &route, c, &[]).failure_kind, ROUTE_OCCUPIED);
        }
    }

    #[test]
    fn stair_sweep_accepts_supported_intermediate_stance() {
        let state = state((0..8).map(|x| SurfaceCell::new(x, 0, x / 2)).collect());
        let route = [point(0.1, 0.1, 0.45), point(0.9, 0.1, 0.85)];
        assert_eq!(check(&state, &route, config(), &[]).valid, 1);
    }

    #[test]
    fn spiral_turn_checks_continuous_nogo_at_its_height() {
        let state = state(
            (0..5)
                .flat_map(|x| (0..5).map(move |y| SurfaceCell::new(x, y, 0)))
                .collect(),
        );
        let route = [point(0.1, 0.1, 0.45), point(0.9, 0.9, 0.45)];
        let mut c = config();
        c.robot_radius = 0.1;
        let region = LtMlsMetricRegion {
            min: point(0.45, 0.4, 0.4),
            max: point(0.55, 0.6, 0.8),
        };
        assert_eq!(
            check(&state, &route, c, &[region]).failure_kind,
            ROUTE_NO_GO
        );
        let high = LtMlsMetricRegion {
            min: point(0.45, 0.4, 2.0),
            max: point(0.55, 0.6, 2.5),
        };
        assert_eq!(check(&state, &route, c, &[high]).valid, 1);
    }

    #[test]
    fn same_floor_body_height_change_is_not_terrain_slope() {
        let state = state((0..3).map(|x| SurfaceCell::new(x, 0, 0)).collect());
        let route = [point(0.1, 0.1, 0.39), point(0.2, 0.1, 0.51)];
        let mut c = config();
        c.support_tolerance = 0.1;
        c.max_slope = 0.1;
        assert_eq!(check(&state, &route, c, &[]).valid, 1);
    }

    #[test]
    fn metric_check_requires_raw_occupied_evidence() {
        let state = PlannerState::from_surface(
            vec![SurfaceCell::new(0, 0, 0), SurfaceCell::new(1, 0, 0)],
            Vec::new(),
            GraphConfig::default(),
        );
        let route = [point(0.1, 0.1, 0.45), point(0.3, 0.1, 0.45)];
        assert_eq!(
            check(&state, &route, config(), &[]).failure_kind,
            ROUTE_SOURCE_UNAVAILABLE
        );
    }

    #[test]
    fn actual_start_only_skips_first_pose_body_and_support() {
        let state = state((1..6).map(|x| SurfaceCell::new(x, 0, 0)).collect());
        let route = [point(0.19, 0.1, 0.45), point(0.5, 0.1, 0.45)];
        let mut c = config();
        assert_eq!(
            check(&state, &route, c, &[]).failure_kind,
            ROUTE_UNSUPPORTED
        );
        c.actual_start = 1;
        assert_eq!(check(&state, &route, c, &[]).valid, 1);
        let return_to_gap = [route[0], route[1], route[0]];
        assert_eq!(
            check(&state, &return_to_gap, c, &[]).failure_kind,
            ROUTE_UNSUPPORTED
        );
    }

    fn pose(point: LtMlsPoint, yaw: Option<f64>) -> LtMlsPose {
        LtMlsPose {
            point,
            yaw: yaw.unwrap_or(0.0),
            has_yaw: u8::from(yaw.is_some()),
        }
    }

    #[test]
    fn dual_body() {
        let state = state(vec![SurfaceCell::new(4, 0, 4)]);
        let route = [point(0.05, 0.05, 0.45), point(0.15, 0.05, 0.45)];
        let mut c = config();
        c.resolution = 0.1;
        c.robot_radius = 0.1;
        c.ground_support = 0;
        assert_eq!(check(&state, &route, c, &[]).valid, 1);
        c.body_offset = 0.3;
        assert_eq!(check(&state, &route, c, &[]).failure_kind, ROUTE_OCCUPIED);
    }

    #[test]
    fn vertex_turn() {
        let state = state(vec![SurfaceCell::new(3, 3, 4)]);
        let mut c = config();
        c.resolution = 0.1;
        c.robot_radius = 0.05;
        c.body_offset = 0.4;
        c.ground_support = 0;
        let checker = prepare(&state, c, &[], &[], None, std::ptr::null_mut()).unwrap();
        let a = pose(point(-0.95, 0.05, 0.45), None);
        let b = pose(point(0.05, 0.05, 0.45), None);
        let d = pose(point(0.05, 1.05, 0.45), None);
        assert_eq!(checker.check_poses(&[a, b]).valid, 1);
        assert_eq!(checker.check_poses(&[b, d]).valid, 1);
        // A pivot at b would hit the wall; vertices and start yaw are not pivots.
        assert_eq!(checker.check_poses(&[a, b, d]).valid, 1);
        assert_eq!(checker.check_poses(&[pose(b.point, Some(0.0)), d]).valid, 1);
        // An explicit terminal yaw still has to turn in place.
        assert_eq!(
            checker
                .check_poses(&[a, pose(b.point, Some(std::f64::consts::FRAC_PI_2))])
                .failure_kind,
            ROUTE_OCCUPIED
        );
        let upper = [a, b, d].map(|p| pose(LtMlsPoint { z: 1.45, ..p.point }, None));
        assert_eq!(checker.check_poses(&upper).valid, 1);
    }

    #[test]
    fn saved_start_cells() {
        let state = state(vec![SurfaceCell::new(0, 0, 4), SurfaceCell::new(4, 0, 4)]);
        let route = [point(0.05, 0.05, 0.45), point(0.25, 0.05, 0.45)];
        let mut c = config();
        c.resolution = 0.1;
        c.robot_radius = 0.1;
        c.ground_support = 0;
        c.actual_start = 1;
        c.start = pose(route[0], Some(0.0));
        assert_eq!(check(&state, &route, c, &[]).failure_kind, ROUTE_OCCUPIED);
        c.start_exemption = 1;
        assert_eq!(check(&state, &route, c, &[]).valid, 1);
        assert_eq!(
            check(&state, &[route[0], point(0.55, 0.05, 0.45)], c, &[]).failure_kind,
            ROUTE_OCCUPIED
        );
        let region = LtMlsMetricRegion {
            min: point(0.0, 0.0, 0.4),
            max: point(0.1, 0.1, 0.5),
        };
        assert_eq!(
            check(&state, &route, c, &[region]).failure_kind,
            ROUTE_NO_GO
        );
        let preblocked = [SurfaceCell::new(0, 0, 4)];
        let checker = prepare(&state, c, &[], &preblocked, None, std::ptr::null_mut()).unwrap();
        assert_eq!(
            checker
                .check_poses(&route.map(|p| pose(p, None)))
                .failure_kind,
            ROUTE_NO_GO
        );
    }

    #[test]
    fn geometry_cancel() {
        unsafe extern "C" fn cancel(context: *mut std::ffi::c_void) -> i32 {
            let calls = &mut *(context as *mut usize);
            *calls += 1;
            i32::from(*calls > 3)
        }
        let state = state(Vec::new());
        let mut c = config();
        c.ground_support = 0;
        let mut calls = 0usize;
        let checker = prepare(
            &state,
            c,
            &[],
            &[],
            Some(cancel),
            &mut calls as *mut _ as *mut _,
        )
        .unwrap();
        let route = [
            pose(point(0.1, 0.1, 0.45), None),
            pose(point(2.0, 0.1, 0.45), None),
        ];
        assert_eq!(checker.check_poses(&route).failure_kind, CANCELLED);
    }

    #[test]
    fn start_gap() {
        let state = state(
            (0..20)
                .filter(|&x| x < 2 || x >= 10)
                .map(|x| SurfaceCell::new(x, 0, 0))
                .collect(),
        );
        let mut c = config();
        c.resolution = 0.1;
        c.actual_start = 1;
        let route = [point(0.05, 0.05, 0.45), point(1.85, 0.05, 0.45)];
        assert_eq!(
            check(&state, &route, c, &[]).failure_kind,
            ROUTE_UNSUPPORTED
        );
    }

    #[test]
    fn geometry_index_reuse() {
        let mut state = state(Vec::new());
        let mut c = config();
        c.resolution = 0.1;
        c.ground_support = 0;
        let route = [point(0.05, 0.05, 0.45), point(0.65, 0.05, 0.45)];
        let a = prepare(&state, c, &[], &[], None, std::ptr::null_mut()).unwrap();
        let b = prepare(&state, c, &[], &[], None, std::ptr::null_mut()).unwrap();
        assert!(std::ptr::eq(a.occupied, b.occupied));
        assert_eq!(a.check_poses(&route.map(|p| pose(p, None))).valid, 1);
        state.set_geometry(
            vec![SurfaceCell::new(3, 0, 4)],
            Vec::new(),
            SurfaceConfig::default(),
        );
        assert!(state.geometry_index.get().is_none());
        assert_eq!(check(&state, &route, c, &[]).failure_kind, ROUTE_OCCUPIED);
        state
            .update_region(crate::RegionBounds::new(3, 3, 0, 0, 4, 4), &[], &[])
            .unwrap();
        assert!(state.geometry_index.get().is_none());
        assert_eq!(check(&state, &route, c, &[]).valid, 1);
    }

    #[test]
    fn attachment_uses_standing_band() {
        let state = state(vec![SurfaceCell::new(0, 0, 0), SurfaceCell::new(0, 0, 10)]);
        let checker = prepare(&state, config(), &[], &[], None, std::ptr::null_mut()).unwrap();
        for z in [0.45, 0.53] {
            assert_eq!(
                checker.attachment(point(0.1, 0.1, z)),
                Some(SurfaceCell::new(0, 0, 0))
            );
        }
        assert_eq!(checker.attachment(point(0.1, 0.1, 0.7)), None);
        assert_eq!(checker.attachment(point(0.3, 0.1, 0.45)), None);
    }

    fn column_fixture() -> PlannerState {
        let mut cells = Vec::new();
        for x in 0..140 {
            for y in -16..16 {
                cells.push(SurfaceCell::new(x, y, -1));
                cells.push(SurfaceCell::new(x, y, 39));
                if (40..44).contains(&x) && y < 8 {
                    for z in 0..32 {
                        cells.push(SurfaceCell::new(x, y, z));
                    }
                }
            }
        }
        cells.push(SurfaceCell::new(85, 9, 8));
        cells.reverse();
        state(cells)
    }

    fn column_config() -> LtMlsMetricConfig {
        LtMlsMetricConfig {
            resolution: 0.1,
            robot_radius: 0.32,
            body_offset: 0.23,
            below: 0.05,
            above: 0.35,
            ground_support: 0,
            min: point(-10.0, -10.0, -5.0),
            max: point(30.0, 30.0, 10.0),
            ..config()
        }
    }

    #[test]
    fn occupied_columns_match_dense() {
        let state = column_fixture();
        let outside_overlay = [SurfaceCell::new(10000, 10000, 10000)];
        for exemption in [0, 1] {
            let mut config = column_config();
            config.start_exemption = exemption;
            config.start = pose(point(4.15, -0.05, 0.435), Some(0.0));
            let indexed = prepare(&state, config, &[], &[], None, std::ptr::null_mut()).unwrap();
            let dense = prepare(
                &state,
                config,
                &[],
                &outside_overlay,
                None,
                std::ptr::null_mut(),
            )
            .unwrap();
            for index in 0..240 {
                let from = point(
                    -0.2 + index as f64 * 0.057,
                    -1.8 + (index % 39) as f64 * 0.1,
                    0.4 + (index % 11) as f64 * 0.35,
                )
                .native();
                let to = point(
                    from.x + (index % 19) as f64 * 0.21,
                    from.y + (index % 7) as f64 * 0.09,
                    from.z + (index % 5) as f64 * 0.025,
                )
                .native();
                assert_eq!(
                    indexed.occupied_sweep(from, to),
                    indexed.occupied_sweep_dense(from, to)
                );
                assert_eq!(indexed.body_query(from, 0.32), dense.body_query(from, 0.32));
                for yaw in [0.0, 0.6, 1.57] {
                    assert_eq!(
                        indexed.oriented_query(from, yaw, false, true),
                        dense.oriented_query(from, yaw, false, true)
                    );
                }
            }
        }
    }

    #[test]
    fn occupied_columns_keep_cancel_order() {
        unsafe extern "C" fn cancel(context: *mut std::ffi::c_void) -> i32 {
            let calls = &mut *(context as *mut usize);
            *calls += 1;
            i32::from(*calls > 3)
        }
        let state = column_fixture();
        let mut indexed_calls = 0usize;
        let mut dense_calls = 0usize;
        let indexed = prepare(
            &state,
            column_config(),
            &[],
            &[],
            Some(cancel),
            &mut indexed_calls as *mut _ as *mut _,
        )
        .unwrap();
        let dense = prepare(
            &state,
            column_config(),
            &[],
            &[],
            Some(cancel),
            &mut dense_calls as *mut _ as *mut _,
        )
        .unwrap();
        let from = point(0.5, 1.2, 0.435);
        let to = point(12.5, 1.2, 0.435);
        assert_eq!(
            indexed.occupied_sweep(from, to),
            dense.occupied_sweep_dense(from, to)
        );
        assert_eq!(indexed_calls, dense_calls);
        assert_eq!(indexed_calls, 4);
    }

    #[test]
    #[ignore = "release microbenchmark, run explicitly with --nocapture"]
    fn occupied_column_benchmark() {
        use std::hint::black_box;
        use std::time::Instant;
        let state = column_fixture();
        let config = column_config();
        let checker = prepare(&state, config, &[], &[], None, std::ptr::null_mut()).unwrap();
        let outside_overlay = [SurfaceCell::new(10000, 10000, 10000)];
        let dense = prepare(
            &state,
            config,
            &[],
            &outside_overlay,
            None,
            std::ptr::null_mut(),
        )
        .unwrap();
        let from = point(0.5, 1.2, 0.435);
        let to = point(12.5, 1.2, 0.435);
        let low = checker.cell(point(from.x - 0.32, from.y - 0.32, from.z - 0.05));
        let high = checker.cell(point(to.x + 0.32, to.y + 0.32, to.z + 0.35));
        let dense_cells = (high.x - low.x + 1) as usize
            * (high.y - low.y + 1) as usize
            * (high.z - low.z + 1) as usize;
        let occupied_cells: usize = (low.x..=high.x)
            .flat_map(|x| (low.y..=high.y).map(move |y| (x, y)))
            .map(|(x, y)| checker.occupied_z(x, y, low.z, high.z).len())
            .sum();
        assert!(occupied_cells < dense_cells);
        let count = 5000;
        let begin = Instant::now();
        for _ in 0..count {
            black_box(checker.occupied_sweep_dense(black_box(from), black_box(to)));
        }
        let old_sweep = begin.elapsed();
        let begin = Instant::now();
        for _ in 0..count {
            black_box(checker.occupied_sweep(black_box(from), black_box(to)));
        }
        let new_sweep = begin.elapsed();
        let begin = Instant::now();
        for _ in 0..count * 10 {
            black_box(dense.body_query(black_box(from), black_box(0.32)));
        }
        let old_body = begin.elapsed();
        let begin = Instant::now();
        for _ in 0..count * 10 {
            black_box(checker.body_query(black_box(from), black_box(0.32)));
        }
        let new_body = begin.elapsed();
        println!("sweep {count} calls dense_cells={dense_cells} occupied_cells={occupied_cells} dense_ms={:.3} indexed_ms={:.3}",
            old_sweep.as_secs_f64()*1000.0, new_sweep.as_secs_f64()*1000.0);
        println!(
            "body {} calls dense_ms={:.3} indexed_ms={:.3}",
            count * 10,
            old_body.as_secs_f64() * 1000.0,
            new_body.as_secs_f64() * 1000.0
        );
    }
}
