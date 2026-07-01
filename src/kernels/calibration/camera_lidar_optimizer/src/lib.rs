use nalgebra::{
    Matrix3, Matrix3x4, Matrix6, Quaternion, SMatrix, SVector, UnitQuaternion, Vector3,
};
use std::collections::HashMap;
use std::slice;

pub type Vector6 = SVector<f64, 6>;
pub type Matrix3x6 = SMatrix<f64, 3, 6>;
pub type Matrix6x12 = SMatrix<f64, 6, 12>;
pub type Matrix12 = SMatrix<f64, 12, 12>;
pub type Vector12 = SVector<f64, 12>;

const DEFAULT_DERIVATIVE_EPS: f64 = 1e-6;
const ABI_VERSION: u32 = 2;

pub const LINGTU_CAMERA_LIDAR_OPT_OK: i32 = 0;
pub const LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER: i32 = -1;
pub const LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE: i32 = -2;
pub const LINGTU_CAMERA_LIDAR_OPT_INVALID_CORRESPONDENCE: i32 = -3;
pub const LINGTU_CAMERA_LIDAR_OPT_EMPTY_CORRESPONDENCES: i32 = -4;
pub const LINGTU_CAMERA_LIDAR_OPT_INVALID_DERIVATIVE_STEP: i32 = -5;
pub const LINGTU_CAMERA_LIDAR_OPT_INVALID_COVARIANCE: i32 = -6;
pub const LINGTU_CAMERA_LIDAR_OPT_LINEAR_SOLVE_FAILED: i32 = -7;
pub const LINGTU_CAMERA_LIDAR_OPT_INSUFFICIENT_OUTPUT_CAPACITY: i32 = -8;
pub const LINGTU_CAMERA_LIDAR_OPT_PANIC: i32 = -255;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose3 {
    pub t_xyz: [f64; 3],
    pub q_wxyz: [f64; 4],
}

impl Pose3 {
    pub fn identity() -> Self {
        Self {
            t_xyz: [0.0, 0.0, 0.0],
            q_wxyz: [1.0, 0.0, 0.0, 0.0],
        }
    }

    pub fn try_new(t_xyz: [f64; 3], q_wxyz: [f64; 4]) -> Option<Self> {
        if !t_xyz.iter().all(|value| value.is_finite())
            || !q_wxyz.iter().all(|value| value.is_finite())
        {
            return None;
        }
        let norm = q_wxyz.iter().map(|value| value * value).sum::<f64>().sqrt();
        if norm <= 1e-12 {
            return None;
        }
        let mut q = [
            q_wxyz[0] / norm,
            q_wxyz[1] / norm,
            q_wxyz[2] / norm,
            q_wxyz[3] / norm,
        ];
        if q[0] < 0.0 {
            for value in &mut q {
                *value = -*value;
            }
        }
        Some(Self { t_xyz, q_wxyz: q })
    }

    pub fn from_parts(translation: Vector3<f64>, rotation: UnitQuaternion<f64>) -> Self {
        let q = rotation.into_inner();
        Self::try_new(
            [translation.x, translation.y, translation.z],
            [q.w, q.i, q.j, q.k],
        )
        .expect("unit quaternion must produce a valid pose")
    }

    pub fn translation(self) -> Vector3<f64> {
        Vector3::new(self.t_xyz[0], self.t_xyz[1], self.t_xyz[2])
    }

    pub fn rotation(self) -> UnitQuaternion<f64> {
        UnitQuaternion::new_normalize(Quaternion::new(
            self.q_wxyz[0],
            self.q_wxyz[1],
            self.q_wxyz[2],
            self.q_wxyz[3],
        ))
    }

    pub fn rotation_matrix(self) -> Matrix3<f64> {
        self.rotation().to_rotation_matrix().into_inner()
    }

    pub fn compose(self, rhs: Self) -> Self {
        let rotation = self.rotation();
        Self::from_parts(
            self.translation() + rotation.transform_vector(&rhs.translation()),
            rotation * rhs.rotation(),
        )
    }

    pub fn inverse(self) -> Self {
        let rotation_inv = self.rotation().inverse();
        Self::from_parts(
            rotation_inv.transform_vector(&(-self.translation())),
            rotation_inv,
        )
    }

    pub fn between(self, rhs: Self) -> Self {
        self.inverse().compose(rhs)
    }

    pub fn exp(delta: &Vector6) -> Self {
        let omega = Vector3::new(delta[0], delta[1], delta[2]);
        let upsilon = Vector3::new(delta[3], delta[4], delta[5]);
        let rotation = UnitQuaternion::from_scaled_axis(omega);
        let translation = left_jacobian_so3(&omega) * upsilon;
        Self::from_parts(translation, rotation)
    }

    pub fn log(self) -> Vector6 {
        let omega = self.rotation().scaled_axis();
        let upsilon = left_jacobian_so3_inverse(&omega) * self.translation();
        Vector6::from_row_slice(&[omega.x, omega.y, omega.z, upsilon.x, upsilon.y, upsilon.z])
    }

    pub fn retract(self, delta: &Vector6) -> Self {
        self.compose(Self::exp(delta))
    }

    pub fn transform_point(self, point: &Vector3<f64>) -> Vector3<f64> {
        self.rotation().transform_vector(point) + self.translation()
    }

    pub fn interpolate_rt(self, rhs: Self, t: f64) -> Self {
        let clamped = t.clamp(0.0, 1.0);
        let delta = self.between(rhs).log() * clamped;
        self.compose(Self::exp(&delta))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CtIcpCorrespondence {
    pub time: f64,
    pub source_xyz: [f64; 3],
    pub target_xyz: [f64; 3],
    pub target_normal: [f64; 3],
}

impl CtIcpCorrespondence {
    pub fn try_new(
        time: f64,
        source_xyz: [f64; 3],
        target_xyz: [f64; 3],
        target_normal: [f64; 3],
    ) -> Option<Self> {
        let item = Self {
            time,
            source_xyz,
            target_xyz,
            target_normal,
        };
        if item.is_valid() {
            Some(item)
        } else {
            None
        }
    }

    fn is_valid(self) -> bool {
        self.time.is_finite()
            && self.source_xyz.iter().all(|value| value.is_finite())
            && self.target_xyz.iter().all(|value| value.is_finite())
            && self.target_normal.iter().all(|value| value.is_finite())
            && Vector3::from_row_slice(&self.target_normal).norm() > 1e-12
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CtGicpCorrespondence {
    pub time: f64,
    pub source_xyz: [f64; 3],
    pub target_xyz: [f64; 3],
    pub source_cov_row_major: [f64; 9],
    pub target_cov_row_major: [f64; 9],
}

impl CtGicpCorrespondence {
    pub fn try_new(
        time: f64,
        source_xyz: [f64; 3],
        target_xyz: [f64; 3],
        source_cov_row_major: [f64; 9],
        target_cov_row_major: [f64; 9],
    ) -> Option<Self> {
        let item = Self {
            time,
            source_xyz,
            target_xyz,
            source_cov_row_major,
            target_cov_row_major,
        };
        if item.is_valid() {
            Some(item)
        } else {
            None
        }
    }

    fn is_valid(self) -> bool {
        self.time.is_finite()
            && self.source_xyz.iter().all(|value| value.is_finite())
            && self.target_xyz.iter().all(|value| value.is_finite())
            && self
                .source_cov_row_major
                .iter()
                .all(|value| value.is_finite())
            && self
                .target_cov_row_major
                .iter()
                .all(|value| value.is_finite())
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CtGicpSourcePoint {
    pub time: f64,
    pub xyz: [f64; 3],
    pub cov_row_major: [f64; 9],
}

impl CtGicpSourcePoint {
    pub fn try_new(time: f64, xyz: [f64; 3], cov_row_major: [f64; 9]) -> Option<Self> {
        let item = Self {
            time,
            xyz,
            cov_row_major,
        };
        if item.is_valid() {
            Some(item)
        } else {
            None
        }
    }

    fn is_valid(self) -> bool {
        self.time.is_finite()
            && self.xyz.iter().all(|value| value.is_finite())
            && self.cov_row_major.iter().all(|value| value.is_finite())
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CtGicpTargetPoint {
    pub xyz: [f64; 3],
    pub cov_row_major: [f64; 9],
}

impl CtGicpTargetPoint {
    pub fn try_new(xyz: [f64; 3], cov_row_major: [f64; 9]) -> Option<Self> {
        let item = Self { xyz, cov_row_major };
        if item.is_valid() {
            Some(item)
        } else {
            None
        }
    }

    fn is_valid(self) -> bool {
        self.xyz.iter().all(|value| value.is_finite())
            && self.cov_row_major.iter().all(|value| value.is_finite())
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CameraLidarPose3Ffi {
    pub t_xyz: [f64; 3],
    pub q_wxyz: [f64; 4],
}

impl CameraLidarPose3Ffi {
    fn to_pose3(self) -> Option<Pose3> {
        Pose3::try_new(self.t_xyz, self.q_wxyz)
    }

    fn from_pose3(pose: Pose3) -> Self {
        Self {
            t_xyz: pose.t_xyz,
            q_wxyz: pose.q_wxyz,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CameraLidarCtIcpCorrespondenceFfi {
    pub time: f64,
    pub source_xyz: [f64; 3],
    pub target_xyz: [f64; 3],
    pub target_normal: [f64; 3],
}

impl CameraLidarCtIcpCorrespondenceFfi {
    fn to_kernel(self) -> Option<CtIcpCorrespondence> {
        CtIcpCorrespondence::try_new(
            self.time,
            self.source_xyz,
            self.target_xyz,
            self.target_normal,
        )
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CameraLidarCtGicpCorrespondenceFfi {
    pub time: f64,
    pub source_xyz: [f64; 3],
    pub target_xyz: [f64; 3],
    pub source_cov_row_major: [f64; 9],
    pub target_cov_row_major: [f64; 9],
}

impl CameraLidarCtGicpCorrespondenceFfi {
    fn to_kernel(self) -> Option<CtGicpCorrespondence> {
        CtGicpCorrespondence::try_new(
            self.time,
            self.source_xyz,
            self.target_xyz,
            self.source_cov_row_major,
            self.target_cov_row_major,
        )
    }

    fn from_kernel(correspondence: CtGicpCorrespondence) -> Self {
        Self {
            time: correspondence.time,
            source_xyz: correspondence.source_xyz,
            target_xyz: correspondence.target_xyz,
            source_cov_row_major: correspondence.source_cov_row_major,
            target_cov_row_major: correspondence.target_cov_row_major,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CameraLidarCtGicpSourcePointFfi {
    pub time: f64,
    pub xyz: [f64; 3],
    pub cov_row_major: [f64; 9],
}

impl CameraLidarCtGicpSourcePointFfi {
    fn to_kernel(self) -> Option<CtGicpSourcePoint> {
        CtGicpSourcePoint::try_new(self.time, self.xyz, self.cov_row_major)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CameraLidarCtGicpTargetPointFfi {
    pub xyz: [f64; 3],
    pub cov_row_major: [f64; 9],
}

impl CameraLidarCtGicpTargetPointFfi {
    fn to_kernel(self) -> Option<CtGicpTargetPoint> {
        CtGicpTargetPoint::try_new(self.xyz, self.cov_row_major)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CameraLidarTwoPoseLinearizationFfi {
    pub cost: f64,
    pub used_correspondences: u64,
    pub hessian_12x12_row_major: [f64; 144],
    pub gradient_12: [f64; 12],
    pub rhs_12: [f64; 12],
}

impl CameraLidarTwoPoseLinearizationFfi {
    fn from_kernel(linearization: &CtIcpLinearization) -> Self {
        let hessian = linearization.hessian_12x12();
        let mut hessian_12x12_row_major = [0.0; 144];
        for row in 0..12 {
            for col in 0..12 {
                hessian_12x12_row_major[row * 12 + col] = hessian[(row, col)];
            }
        }

        let mut gradient_12 = [0.0; 12];
        let mut rhs_12 = [0.0; 12];
        for idx in 0..6 {
            gradient_12[idx] = linearization.gradient0[idx];
            gradient_12[6 + idx] = linearization.gradient1[idx];
            rhs_12[idx] = linearization.rhs0[idx];
            rhs_12[6 + idx] = linearization.rhs1[idx];
        }

        Self {
            cost: linearization.cost,
            used_correspondences: linearization.used_correspondences as u64,
            hessian_12x12_row_major,
            gradient_12,
            rhs_12,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CameraLidarTwoPoseOptimizerConfigFfi {
    pub max_iterations: u32,
    pub derivative_eps: f64,
    pub initial_lambda: f64,
    pub lambda_factor: f64,
    pub step_tolerance: f64,
    pub relative_cost_tolerance: f64,
    pub prior_precision: f64,
    pub between_precision: f64,
}

impl CameraLidarTwoPoseOptimizerConfigFfi {
    fn to_config(self) -> Option<TwoPoseOptimizerConfig> {
        let max_iterations = if self.max_iterations == 0 {
            10
        } else {
            self.max_iterations as usize
        };
        let derivative_eps = positive_or_default(self.derivative_eps, DEFAULT_DERIVATIVE_EPS)?;
        let initial_lambda = positive_or_default(self.initial_lambda, 1e-3)?;
        let lambda_factor = positive_or_default(self.lambda_factor, 10.0)?;
        let step_tolerance = positive_or_default(self.step_tolerance, 1e-7)?;
        let relative_cost_tolerance = positive_or_default(self.relative_cost_tolerance, 1e-8)?;
        if !self.prior_precision.is_finite()
            || !self.between_precision.is_finite()
            || self.prior_precision < 0.0
            || self.between_precision < 0.0
        {
            return None;
        }
        Some(TwoPoseOptimizerConfig {
            max_iterations,
            derivative_eps,
            initial_lambda,
            lambda_factor,
            step_tolerance,
            relative_cost_tolerance,
            prior_precision: self.prior_precision,
            between_precision: self.between_precision,
        })
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CameraLidarTwoPoseOptimizationResultFfi {
    pub pose0: CameraLidarPose3Ffi,
    pub pose1: CameraLidarPose3Ffi,
    pub initial_cost: f64,
    pub final_cost: f64,
    pub iterations: u32,
    pub accepted_steps: u32,
    pub last_lambda: f64,
    pub used_correspondences: u64,
}

impl CameraLidarTwoPoseOptimizationResultFfi {
    fn from_kernel(result: &TwoPoseOptimizationResult) -> Self {
        Self {
            pose0: CameraLidarPose3Ffi::from_pose3(result.pose0),
            pose1: CameraLidarPose3Ffi::from_pose3(result.pose1),
            initial_cost: result.initial_cost,
            final_cost: result.final_cost,
            iterations: result.iterations as u32,
            accepted_steps: result.accepted_steps as u32,
            last_lambda: result.last_lambda,
            used_correspondences: result.used_correspondences as u64,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TwoPoseOptimizerConfig {
    pub max_iterations: usize,
    pub derivative_eps: f64,
    pub initial_lambda: f64,
    pub lambda_factor: f64,
    pub step_tolerance: f64,
    pub relative_cost_tolerance: f64,
    pub prior_precision: f64,
    pub between_precision: f64,
}

impl Default for TwoPoseOptimizerConfig {
    fn default() -> Self {
        Self {
            max_iterations: 10,
            derivative_eps: DEFAULT_DERIVATIVE_EPS,
            initial_lambda: 1e-3,
            lambda_factor: 10.0,
            step_tolerance: 1e-7,
            relative_cost_tolerance: 1e-8,
            prior_precision: 1e3,
            between_precision: 1e5,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TwoPoseOptimizationResult {
    pub pose0: Pose3,
    pub pose1: Pose3,
    pub initial_cost: f64,
    pub final_cost: f64,
    pub iterations: usize,
    pub accepted_steps: usize,
    pub last_lambda: f64,
    pub used_correspondences: usize,
}

#[derive(Clone, Debug)]
pub struct CtIcpLinearization {
    pub cost: f64,
    pub used_correspondences: usize,
    pub h00: Matrix6<f64>,
    pub h01: Matrix6<f64>,
    pub h11: Matrix6<f64>,
    pub gradient0: Vector6,
    pub gradient1: Vector6,
    pub rhs0: Vector6,
    pub rhs1: Vector6,
}

impl CtIcpLinearization {
    pub fn hessian_12x12(&self) -> Matrix12 {
        let mut h = Matrix12::zeros();
        h.fixed_view_mut::<6, 6>(0, 0).copy_from(&self.h00);
        h.fixed_view_mut::<6, 6>(0, 6).copy_from(&self.h01);
        h.fixed_view_mut::<6, 6>(6, 0)
            .copy_from(&self.h01.transpose());
        h.fixed_view_mut::<6, 6>(6, 6).copy_from(&self.h11);
        h
    }

    pub fn gradient_12(self) -> Vector12 {
        let mut gradient = Vector12::zeros();
        gradient.fixed_rows_mut::<6>(0).copy_from(&self.gradient0);
        gradient.fixed_rows_mut::<6>(6).copy_from(&self.gradient1);
        gradient
    }

    pub fn rhs_12(self) -> Vector12 {
        let mut rhs = Vector12::zeros();
        rhs.fixed_rows_mut::<6>(0).copy_from(&self.rhs0);
        rhs.fixed_rows_mut::<6>(6).copy_from(&self.rhs1);
        rhs
    }
}

#[derive(Clone, Debug)]
pub enum CalibrationKernelError {
    InvalidPose,
    InvalidCorrespondence(usize),
    InvalidCovariance(usize),
    EmptyCorrespondences,
    InvalidDerivativeStep,
    LinearSolveFailed,
    InsufficientOutputCapacity,
}

pub fn ct_icp_residual(pose: Pose3, correspondence: CtIcpCorrespondence) -> f64 {
    let source = Vector3::from_row_slice(&correspondence.source_xyz);
    let target = Vector3::from_row_slice(&correspondence.target_xyz);
    let normal = normalized_normal(correspondence);
    normal.dot(&(pose.transform_point(&source) - target))
}

pub fn ct_gicp_residual_vector(pose: Pose3, correspondence: CtGicpCorrespondence) -> Vector3<f64> {
    let source = Vector3::from_row_slice(&correspondence.source_xyz);
    let target = Vector3::from_row_slice(&correspondence.target_xyz);
    pose.transform_point(&source) - target
}

pub fn build_ct_gicp_nearest_neighbor_correspondences(
    pose0: Pose3,
    pose1: Pose3,
    sources: &[CtGicpSourcePoint],
    targets: &[CtGicpTargetPoint],
    max_correspondence_distance: f64,
) -> Result<Vec<CtGicpCorrespondence>, CalibrationKernelError> {
    if !max_correspondence_distance.is_finite() || max_correspondence_distance <= 0.0 {
        return Err(CalibrationKernelError::InvalidCorrespondence(0));
    }
    if sources.is_empty() || targets.is_empty() {
        return Err(CalibrationKernelError::EmptyCorrespondences);
    }

    let target_index = build_ct_gicp_target_voxel_index(targets, max_correspondence_distance)?;
    let max_distance_sq = max_correspondence_distance * max_correspondence_distance;
    let mut correspondences = Vec::with_capacity(sources.len());

    for (source_index, source) in sources.iter().copied().enumerate() {
        if !source.is_valid() {
            return Err(CalibrationKernelError::InvalidCorrespondence(source_index));
        }

        let source_point = Vector3::from_row_slice(&source.xyz);
        let pose = pose0.interpolate_rt(pose1, source.time);
        let transformed_source = pose.transform_point(&source_point);
        let Some(center_coord) =
            ct_gicp_voxel_coord(&transformed_source, max_correspondence_distance)
        else {
            return Err(CalibrationKernelError::InvalidCorrespondence(source_index));
        };

        let mut best_target_index: Option<usize> = None;
        let mut best_distance_sq = f64::INFINITY;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let coord = (
                        center_coord.0 + dx,
                        center_coord.1 + dy,
                        center_coord.2 + dz,
                    );
                    let Some(candidate_indices) = target_index.get(&coord) else {
                        continue;
                    };
                    for &target_index in candidate_indices {
                        let target = Vector3::from_row_slice(&targets[target_index].xyz);
                        let distance_sq = (transformed_source - target).norm_squared();
                        if distance_sq <= max_distance_sq && distance_sq < best_distance_sq {
                            best_distance_sq = distance_sq;
                            best_target_index = Some(target_index);
                        }
                    }
                }
            }
        }

        if let Some(target_index) = best_target_index {
            let target = targets[target_index];
            let Some(correspondence) = CtGicpCorrespondence::try_new(
                source.time,
                source.xyz,
                target.xyz,
                source.cov_row_major,
                target.cov_row_major,
            ) else {
                return Err(CalibrationKernelError::InvalidCorrespondence(source_index));
            };
            correspondences.push(correspondence);
        }
    }

    Ok(correspondences)
}

pub fn linearize_ct_icp_fixed_correspondences(
    pose0: Pose3,
    pose1: Pose3,
    correspondences: &[CtIcpCorrespondence],
) -> Result<CtIcpLinearization, CalibrationKernelError> {
    linearize_ct_icp_fixed_correspondences_with_step(
        pose0,
        pose1,
        correspondences,
        DEFAULT_DERIVATIVE_EPS,
    )
}

pub fn linearize_ct_icp_fixed_correspondences_with_step(
    pose0: Pose3,
    pose1: Pose3,
    correspondences: &[CtIcpCorrespondence],
    derivative_eps: f64,
) -> Result<CtIcpLinearization, CalibrationKernelError> {
    if !pose0.t_xyz.iter().all(|value| value.is_finite())
        || !pose0.q_wxyz.iter().all(|value| value.is_finite())
        || !pose1.t_xyz.iter().all(|value| value.is_finite())
        || !pose1.q_wxyz.iter().all(|value| value.is_finite())
    {
        return Err(CalibrationKernelError::InvalidPose);
    }
    if correspondences.is_empty() {
        return Err(CalibrationKernelError::EmptyCorrespondences);
    }
    if !derivative_eps.is_finite() || derivative_eps <= 0.0 {
        return Err(CalibrationKernelError::InvalidDerivativeStep);
    }

    let mut output = empty_two_pose_linearization();
    let mut derivative_cache: HashMap<u64, EndpointTransformDerivatives> = HashMap::new();

    for (index, correspondence) in correspondences.iter().copied().enumerate() {
        if !correspondence.is_valid() {
            return Err(CalibrationKernelError::InvalidCorrespondence(index));
        }
        let source = Vector3::from_row_slice(&correspondence.source_xyz);
        let target = Vector3::from_row_slice(&correspondence.target_xyz);
        let normal = normalized_normal(correspondence);
        let time = correspondence.time.clamp(0.0, 1.0);
        let pose = pose0.interpolate_rt(pose1, time);
        let transformed = pose.transform_point(&source);
        let residual = normal.dot(&(transformed - target));
        let derivatives = *derivative_cache
            .entry(time.to_bits())
            .or_insert_with(|| endpoint_transform_derivatives(pose0, pose1, time, derivative_eps));

        let mut j0 = Vector6::zeros();
        let mut j1 = Vector6::zeros();
        for dim in 0..6 {
            let d0 = derivatives.pose0[dim].transform_direction(&source);
            let d1 = derivatives.pose1[dim].transform_direction(&source);
            j0[dim] = normal.dot(&d0);
            j1[dim] = normal.dot(&d1);
        }

        output.cost += 0.5 * residual * residual;
        output.h00 += j0 * j0.transpose();
        output.h01 += j0 * j1.transpose();
        output.h11 += j1 * j1.transpose();
        output.gradient0 += j0 * residual;
        output.gradient1 += j1 * residual;
        output.used_correspondences += 1;
    }

    output.rhs0 = -output.gradient0;
    output.rhs1 = -output.gradient1;
    Ok(output)
}

pub fn linearize_ct_gicp_fixed_correspondences(
    pose0: Pose3,
    pose1: Pose3,
    correspondences: &[CtGicpCorrespondence],
) -> Result<CtIcpLinearization, CalibrationKernelError> {
    linearize_ct_gicp_fixed_correspondences_with_step(
        pose0,
        pose1,
        correspondences,
        DEFAULT_DERIVATIVE_EPS,
    )
}

pub fn linearize_ct_gicp_fixed_correspondences_with_step(
    pose0: Pose3,
    pose1: Pose3,
    correspondences: &[CtGicpCorrespondence],
    derivative_eps: f64,
) -> Result<CtIcpLinearization, CalibrationKernelError> {
    if !pose0.t_xyz.iter().all(|value| value.is_finite())
        || !pose0.q_wxyz.iter().all(|value| value.is_finite())
        || !pose1.t_xyz.iter().all(|value| value.is_finite())
        || !pose1.q_wxyz.iter().all(|value| value.is_finite())
    {
        return Err(CalibrationKernelError::InvalidPose);
    }
    if correspondences.is_empty() {
        return Err(CalibrationKernelError::EmptyCorrespondences);
    }
    if !derivative_eps.is_finite() || derivative_eps <= 0.0 {
        return Err(CalibrationKernelError::InvalidDerivativeStep);
    }

    let mut output = empty_two_pose_linearization();
    let mut derivative_cache: HashMap<u64, EndpointTransformDerivatives> = HashMap::new();

    for (index, correspondence) in correspondences.iter().copied().enumerate() {
        if !correspondence.is_valid() {
            return Err(CalibrationKernelError::InvalidCorrespondence(index));
        }
        let source = Vector3::from_row_slice(&correspondence.source_xyz);
        let target = Vector3::from_row_slice(&correspondence.target_xyz);
        let time = correspondence.time.clamp(0.0, 1.0);
        let pose = pose0.interpolate_rt(pose1, time);
        let transformed = pose.transform_point(&source);
        let residual = transformed - target;
        let mahalanobis = ct_gicp_mahalanobis(pose.rotation_matrix(), correspondence)
            .ok_or(CalibrationKernelError::InvalidCovariance(index))?;
        let mahalanobis_residual = mahalanobis * residual;
        let derivatives = *derivative_cache
            .entry(time.to_bits())
            .or_insert_with(|| endpoint_transform_derivatives(pose0, pose1, time, derivative_eps));

        let mut j0 = Matrix3x6::zeros();
        let mut j1 = Matrix3x6::zeros();
        for dim in 0..6 {
            let d0 = derivatives.pose0[dim].transform_direction(&source);
            let d1 = derivatives.pose1[dim].transform_direction(&source);
            j0.column_mut(dim).copy_from(&d0);
            j1.column_mut(dim).copy_from(&d1);
        }

        output.cost += 0.5 * residual.dot(&mahalanobis_residual);
        output.h00 += j0.transpose() * mahalanobis * j0;
        output.h01 += j0.transpose() * mahalanobis * j1;
        output.h11 += j1.transpose() * mahalanobis * j1;
        output.gradient0 += j0.transpose() * mahalanobis_residual;
        output.gradient1 += j1.transpose() * mahalanobis_residual;
        output.used_correspondences += 1;
    }

    output.rhs0 = -output.gradient0;
    output.rhs1 = -output.gradient1;
    Ok(output)
}

pub fn optimize_ct_gicp_two_pose(
    initial_pose0: Pose3,
    initial_pose1: Pose3,
    prior_pose0: Pose3,
    between_measurement: Pose3,
    correspondences: &[CtGicpCorrespondence],
    config: TwoPoseOptimizerConfig,
) -> Result<TwoPoseOptimizationResult, CalibrationKernelError> {
    if correspondences.is_empty() {
        return Err(CalibrationKernelError::EmptyCorrespondences);
    }
    if !optimizer_config_is_valid(config) {
        return Err(CalibrationKernelError::InvalidDerivativeStep);
    }

    let mut pose0 = initial_pose0;
    let mut pose1 = initial_pose1;
    let mut lambda = config.initial_lambda;
    let mut iterations = 0;
    let mut accepted_steps = 0;
    let mut current_cost = two_pose_problem_cost(
        pose0,
        pose1,
        prior_pose0,
        between_measurement,
        correspondences,
        config,
    )?;
    let initial_cost = current_cost;

    for _ in 0..config.max_iterations {
        iterations += 1;
        let linearization = linearize_ct_gicp_two_pose_problem(
            pose0,
            pose1,
            prior_pose0,
            between_measurement,
            correspondences,
            config,
        )?;
        let mut hessian = linearization.hessian_12x12();
        let rhs = linearization.clone().rhs_12();
        for idx in 0..12 {
            hessian[(idx, idx)] += lambda * hessian[(idx, idx)].abs().max(1.0);
        }
        let Some(step) = hessian.lu().solve(&rhs) else {
            return Err(CalibrationKernelError::LinearSolveFailed);
        };
        if !step.iter().all(|value| value.is_finite()) {
            return Err(CalibrationKernelError::LinearSolveFailed);
        }
        if step.norm() <= config.step_tolerance {
            break;
        }

        let step0 = Vector6::from_iterator(step.fixed_rows::<6>(0).iter().copied());
        let step1 = Vector6::from_iterator(step.fixed_rows::<6>(6).iter().copied());
        let candidate0 = pose0.retract(&step0);
        let candidate1 = pose1.retract(&step1);
        let candidate_cost = two_pose_problem_cost(
            candidate0,
            candidate1,
            prior_pose0,
            between_measurement,
            correspondences,
            config,
        )?;

        if candidate_cost.is_finite() && candidate_cost < current_cost {
            let decrease = current_cost - candidate_cost;
            pose0 = candidate0;
            pose1 = candidate1;
            current_cost = candidate_cost;
            accepted_steps += 1;
            lambda = (lambda / config.lambda_factor).max(1e-12);
            let scale = initial_cost.abs().max(1.0);
            if decrease / scale <= config.relative_cost_tolerance {
                break;
            }
        } else {
            lambda = (lambda * config.lambda_factor).min(1e12);
        }
    }

    Ok(TwoPoseOptimizationResult {
        pose0,
        pose1,
        initial_cost,
        final_cost: current_cost,
        iterations,
        accepted_steps,
        last_lambda: lambda,
        used_correspondences: correspondences.len(),
    })
}

pub fn optimize_ct_gicp_dynamic_two_pose(
    initial_pose0: Pose3,
    initial_pose1: Pose3,
    prior_pose0: Pose3,
    between_measurement: Pose3,
    sources: &[CtGicpSourcePoint],
    targets: &[CtGicpTargetPoint],
    max_correspondence_distance: f64,
    outer_iterations: usize,
    config: TwoPoseOptimizerConfig,
) -> Result<TwoPoseOptimizationResult, CalibrationKernelError> {
    if outer_iterations == 0 {
        return Err(CalibrationKernelError::InvalidDerivativeStep);
    }

    let mut pose0 = initial_pose0;
    let mut pose1 = initial_pose1;
    let mut first_initial_cost: Option<f64> = None;
    let mut last_result: Option<TwoPoseOptimizationResult> = None;
    let mut total_iterations = 0usize;
    let mut total_accepted_steps = 0usize;

    for _ in 0..outer_iterations {
        let correspondences = build_ct_gicp_nearest_neighbor_correspondences(
            pose0,
            pose1,
            sources,
            targets,
            max_correspondence_distance,
        )?;
        if correspondences.is_empty() {
            return Err(CalibrationKernelError::EmptyCorrespondences);
        }

        let result = optimize_ct_gicp_two_pose(
            pose0,
            pose1,
            prior_pose0,
            between_measurement,
            &correspondences,
            config,
        )?;
        first_initial_cost.get_or_insert(result.initial_cost);
        total_iterations += result.iterations;
        total_accepted_steps += result.accepted_steps;
        pose0 = result.pose0;
        pose1 = result.pose1;
        last_result = Some(result);
    }

    let Some(last_result) = last_result else {
        return Err(CalibrationKernelError::InvalidDerivativeStep);
    };
    Ok(TwoPoseOptimizationResult {
        pose0,
        pose1,
        initial_cost: first_initial_cost.unwrap_or(last_result.initial_cost),
        final_cost: last_result.final_cost,
        iterations: total_iterations,
        accepted_steps: total_accepted_steps,
        last_lambda: last_result.last_lambda,
        used_correspondences: last_result.used_correspondences,
    })
}

fn linearize_ct_gicp_two_pose_problem(
    pose0: Pose3,
    pose1: Pose3,
    prior_pose0: Pose3,
    between_measurement: Pose3,
    correspondences: &[CtGicpCorrespondence],
    config: TwoPoseOptimizerConfig,
) -> Result<CtIcpLinearization, CalibrationKernelError> {
    let mut output = linearize_ct_gicp_fixed_correspondences_with_step(
        pose0,
        pose1,
        correspondences,
        config.derivative_eps,
    )?;
    add_pose0_prior_factor(
        &mut output,
        pose0,
        prior_pose0,
        config.prior_precision,
        config.derivative_eps,
    );
    add_between_factor(
        &mut output,
        pose0,
        pose1,
        between_measurement,
        config.between_precision,
        config.derivative_eps,
    );
    output.rhs0 = -output.gradient0;
    output.rhs1 = -output.gradient1;
    Ok(output)
}

fn two_pose_problem_cost(
    pose0: Pose3,
    pose1: Pose3,
    prior_pose0: Pose3,
    between_measurement: Pose3,
    correspondences: &[CtGicpCorrespondence],
    config: TwoPoseOptimizerConfig,
) -> Result<f64, CalibrationKernelError> {
    let data = linearize_ct_gicp_fixed_correspondences_with_step(
        pose0,
        pose1,
        correspondences,
        config.derivative_eps,
    )?;
    let prior_residual = prior_pose0.between(pose0).log();
    let between_residual = between_measurement.between(pose0.between(pose1)).log();
    Ok(data.cost
        + 0.5 * config.prior_precision * prior_residual.norm_squared()
        + 0.5 * config.between_precision * between_residual.norm_squared())
}

fn add_pose0_prior_factor(
    output: &mut CtIcpLinearization,
    pose0: Pose3,
    prior_pose0: Pose3,
    precision: f64,
    eps: f64,
) {
    if precision == 0.0 {
        return;
    }
    let residual = prior_pose0.between(pose0).log();
    let mut j0 = Matrix6::zeros();
    for dim in 0..6 {
        let mut plus = Vector6::zeros();
        plus[dim] = eps;
        let mut minus = Vector6::zeros();
        minus[dim] = -eps;
        let residual_plus = prior_pose0.between(pose0.retract(&plus)).log();
        let residual_minus = prior_pose0.between(pose0.retract(&minus)).log();
        j0.column_mut(dim)
            .copy_from(&((residual_plus - residual_minus) / (2.0 * eps)));
    }
    output.cost += 0.5 * precision * residual.norm_squared();
    output.h00 += precision * j0.transpose() * j0;
    output.gradient0 += precision * j0.transpose() * residual;
}

fn add_between_factor(
    output: &mut CtIcpLinearization,
    pose0: Pose3,
    pose1: Pose3,
    between_measurement: Pose3,
    precision: f64,
    eps: f64,
) {
    if precision == 0.0 {
        return;
    }
    let residual = between_measurement.between(pose0.between(pose1)).log();
    let mut j0 = Matrix6::zeros();
    let mut j1 = Matrix6::zeros();
    for dim in 0..6 {
        let mut plus = Vector6::zeros();
        plus[dim] = eps;
        let mut minus = Vector6::zeros();
        minus[dim] = -eps;

        let residual0_plus = between_measurement
            .between(pose0.retract(&plus).between(pose1))
            .log();
        let residual0_minus = between_measurement
            .between(pose0.retract(&minus).between(pose1))
            .log();
        j0.column_mut(dim)
            .copy_from(&((residual0_plus - residual0_minus) / (2.0 * eps)));

        let residual1_plus = between_measurement
            .between(pose0.between(pose1.retract(&plus)))
            .log();
        let residual1_minus = between_measurement
            .between(pose0.between(pose1.retract(&minus)))
            .log();
        j1.column_mut(dim)
            .copy_from(&((residual1_plus - residual1_minus) / (2.0 * eps)));
    }
    output.cost += 0.5 * precision * residual.norm_squared();
    output.h00 += precision * j0.transpose() * j0;
    output.h01 += precision * j0.transpose() * j1;
    output.h11 += precision * j1.transpose() * j1;
    output.gradient0 += precision * j0.transpose() * residual;
    output.gradient1 += precision * j1.transpose() * residual;
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_abi_version() -> u32 {
    ABI_VERSION
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_sizeof_pose3() -> usize {
    std::mem::size_of::<CameraLidarPose3Ffi>()
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_sizeof_ct_icp_correspondence() -> usize {
    std::mem::size_of::<CameraLidarCtIcpCorrespondenceFfi>()
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_sizeof_ct_gicp_correspondence() -> usize {
    std::mem::size_of::<CameraLidarCtGicpCorrespondenceFfi>()
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_sizeof_ct_gicp_source_point() -> usize {
    std::mem::size_of::<CameraLidarCtGicpSourcePointFfi>()
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_sizeof_ct_gicp_target_point() -> usize {
    std::mem::size_of::<CameraLidarCtGicpTargetPointFfi>()
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_sizeof_two_pose_linearization() -> usize {
    std::mem::size_of::<CameraLidarTwoPoseLinearizationFfi>()
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_sizeof_two_pose_optimizer_config() -> usize {
    std::mem::size_of::<CameraLidarTwoPoseOptimizerConfigFfi>()
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_sizeof_two_pose_optimization_result() -> usize {
    std::mem::size_of::<CameraLidarTwoPoseOptimizationResultFfi>()
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_build_ct_gicp_correspondences(
    pose0: *const CameraLidarPose3Ffi,
    pose1: *const CameraLidarPose3Ffi,
    sources: *const CameraLidarCtGicpSourcePointFfi,
    source_count: usize,
    targets: *const CameraLidarCtGicpTargetPointFfi,
    target_count: usize,
    max_correspondence_distance: f64,
    output: *mut CameraLidarCtGicpCorrespondenceFfi,
    output_capacity: usize,
    output_count: *mut usize,
) -> i32 {
    let result = std::panic::catch_unwind(|| unsafe {
        let Some(pose0_ffi) = pose0.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(pose1_ffi) = pose1.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(pose0) = pose0_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(pose1) = pose1_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(output_count) = output_count.as_mut() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        *output_count = 0;
        let Some(sources) = ffi_slice(sources, source_count) else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(targets) = ffi_slice(targets, target_count) else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        if output_capacity > 0 && output.is_null() {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        }

        let mut kernel_sources = Vec::with_capacity(sources.len());
        for source in sources.iter().copied() {
            let Some(kernel) = source.to_kernel() else {
                return LINGTU_CAMERA_LIDAR_OPT_INVALID_CORRESPONDENCE;
            };
            kernel_sources.push(kernel);
        }
        let mut kernel_targets = Vec::with_capacity(targets.len());
        for target in targets.iter().copied() {
            let Some(kernel) = target.to_kernel() else {
                return LINGTU_CAMERA_LIDAR_OPT_INVALID_CORRESPONDENCE;
            };
            kernel_targets.push(kernel);
        }

        match build_ct_gicp_nearest_neighbor_correspondences(
            pose0,
            pose1,
            &kernel_sources,
            &kernel_targets,
            max_correspondence_distance,
        ) {
            Ok(correspondences) => {
                *output_count = correspondences.len();
                if output_capacity < correspondences.len() {
                    return LINGTU_CAMERA_LIDAR_OPT_INSUFFICIENT_OUTPUT_CAPACITY;
                }
                if !correspondences.is_empty() {
                    let output = slice::from_raw_parts_mut(output, output_capacity);
                    for (index, correspondence) in correspondences.iter().copied().enumerate() {
                        output[index] =
                            CameraLidarCtGicpCorrespondenceFfi::from_kernel(correspondence);
                    }
                }
                LINGTU_CAMERA_LIDAR_OPT_OK
            }
            Err(error) => calibration_error_code(error),
        }
    });
    result.unwrap_or(LINGTU_CAMERA_LIDAR_OPT_PANIC)
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_linearize_ct_icp(
    pose0: *const CameraLidarPose3Ffi,
    pose1: *const CameraLidarPose3Ffi,
    correspondences: *const CameraLidarCtIcpCorrespondenceFfi,
    correspondence_count: usize,
    derivative_eps: f64,
    output: *mut CameraLidarTwoPoseLinearizationFfi,
) -> i32 {
    let result = std::panic::catch_unwind(|| unsafe {
        let Some(pose0_ffi) = pose0.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(pose1_ffi) = pose1.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(pose0) = pose0_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(pose1) = pose1_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(output) = output.as_mut() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(correspondences) = ffi_slice(correspondences, correspondence_count) else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let mut kernel_correspondences = Vec::with_capacity(correspondences.len());
        for correspondence in correspondences.iter().copied() {
            let Some(kernel) = correspondence.to_kernel() else {
                return LINGTU_CAMERA_LIDAR_OPT_INVALID_CORRESPONDENCE;
            };
            kernel_correspondences.push(kernel);
        }

        match linearize_ct_icp_fixed_correspondences_with_step(
            pose0,
            pose1,
            &kernel_correspondences,
            derivative_eps,
        ) {
            Ok(linearization) => {
                *output = CameraLidarTwoPoseLinearizationFfi::from_kernel(&linearization);
                LINGTU_CAMERA_LIDAR_OPT_OK
            }
            Err(error) => calibration_error_code(error),
        }
    });
    result.unwrap_or(LINGTU_CAMERA_LIDAR_OPT_PANIC)
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_linearize_ct_gicp(
    pose0: *const CameraLidarPose3Ffi,
    pose1: *const CameraLidarPose3Ffi,
    correspondences: *const CameraLidarCtGicpCorrespondenceFfi,
    correspondence_count: usize,
    derivative_eps: f64,
    output: *mut CameraLidarTwoPoseLinearizationFfi,
) -> i32 {
    let result = std::panic::catch_unwind(|| unsafe {
        let Some(pose0_ffi) = pose0.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(pose1_ffi) = pose1.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(pose0) = pose0_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(pose1) = pose1_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(output) = output.as_mut() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(correspondences) = ffi_slice(correspondences, correspondence_count) else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let mut kernel_correspondences = Vec::with_capacity(correspondences.len());
        for correspondence in correspondences.iter().copied() {
            let Some(kernel) = correspondence.to_kernel() else {
                return LINGTU_CAMERA_LIDAR_OPT_INVALID_CORRESPONDENCE;
            };
            kernel_correspondences.push(kernel);
        }

        match linearize_ct_gicp_fixed_correspondences_with_step(
            pose0,
            pose1,
            &kernel_correspondences,
            derivative_eps,
        ) {
            Ok(linearization) => {
                *output = CameraLidarTwoPoseLinearizationFfi::from_kernel(&linearization);
                LINGTU_CAMERA_LIDAR_OPT_OK
            }
            Err(error) => calibration_error_code(error),
        }
    });
    result.unwrap_or(LINGTU_CAMERA_LIDAR_OPT_PANIC)
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_optimize_ct_gicp_two_pose(
    initial_pose0: *const CameraLidarPose3Ffi,
    initial_pose1: *const CameraLidarPose3Ffi,
    prior_pose0: *const CameraLidarPose3Ffi,
    between_measurement: *const CameraLidarPose3Ffi,
    correspondences: *const CameraLidarCtGicpCorrespondenceFfi,
    correspondence_count: usize,
    config: *const CameraLidarTwoPoseOptimizerConfigFfi,
    output: *mut CameraLidarTwoPoseOptimizationResultFfi,
) -> i32 {
    let result = std::panic::catch_unwind(|| unsafe {
        let Some(initial_pose0_ffi) = initial_pose0.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(initial_pose1_ffi) = initial_pose1.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(prior_pose0_ffi) = prior_pose0.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(between_measurement_ffi) = between_measurement.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(output) = output.as_mut() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(config) = config.as_ref().and_then(|config| config.to_config()) else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_DERIVATIVE_STEP;
        };
        let Some(initial_pose0) = initial_pose0_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(initial_pose1) = initial_pose1_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(prior_pose0) = prior_pose0_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(between_measurement) = between_measurement_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(correspondences) = ffi_slice(correspondences, correspondence_count) else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let mut kernel_correspondences = Vec::with_capacity(correspondences.len());
        for correspondence in correspondences.iter().copied() {
            let Some(kernel) = correspondence.to_kernel() else {
                return LINGTU_CAMERA_LIDAR_OPT_INVALID_CORRESPONDENCE;
            };
            kernel_correspondences.push(kernel);
        }

        match optimize_ct_gicp_two_pose(
            initial_pose0,
            initial_pose1,
            prior_pose0,
            between_measurement,
            &kernel_correspondences,
            config,
        ) {
            Ok(result) => {
                *output = CameraLidarTwoPoseOptimizationResultFfi::from_kernel(&result);
                LINGTU_CAMERA_LIDAR_OPT_OK
            }
            Err(error) => calibration_error_code(error),
        }
    });
    result.unwrap_or(LINGTU_CAMERA_LIDAR_OPT_PANIC)
}

#[no_mangle]
pub extern "C" fn lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose(
    initial_pose0: *const CameraLidarPose3Ffi,
    initial_pose1: *const CameraLidarPose3Ffi,
    prior_pose0: *const CameraLidarPose3Ffi,
    between_measurement: *const CameraLidarPose3Ffi,
    sources: *const CameraLidarCtGicpSourcePointFfi,
    source_count: usize,
    targets: *const CameraLidarCtGicpTargetPointFfi,
    target_count: usize,
    max_correspondence_distance: f64,
    outer_iterations: u32,
    config: *const CameraLidarTwoPoseOptimizerConfigFfi,
    output: *mut CameraLidarTwoPoseOptimizationResultFfi,
) -> i32 {
    let result = std::panic::catch_unwind(|| unsafe {
        let Some(initial_pose0_ffi) = initial_pose0.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(initial_pose1_ffi) = initial_pose1.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(prior_pose0_ffi) = prior_pose0.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(between_measurement_ffi) = between_measurement.as_ref() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(output) = output.as_mut() else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(config) = config.as_ref().and_then(|config| config.to_config()) else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_DERIVATIVE_STEP;
        };
        let Some(initial_pose0) = initial_pose0_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(initial_pose1) = initial_pose1_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(prior_pose0) = prior_pose0_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(between_measurement) = between_measurement_ffi.to_pose3() else {
            return LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE;
        };
        let Some(sources) = ffi_slice(sources, source_count) else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let Some(targets) = ffi_slice(targets, target_count) else {
            return LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER;
        };
        let mut kernel_sources = Vec::with_capacity(sources.len());
        for source in sources.iter().copied() {
            let Some(kernel) = source.to_kernel() else {
                return LINGTU_CAMERA_LIDAR_OPT_INVALID_CORRESPONDENCE;
            };
            kernel_sources.push(kernel);
        }
        let mut kernel_targets = Vec::with_capacity(targets.len());
        for target in targets.iter().copied() {
            let Some(kernel) = target.to_kernel() else {
                return LINGTU_CAMERA_LIDAR_OPT_INVALID_CORRESPONDENCE;
            };
            kernel_targets.push(kernel);
        }

        match optimize_ct_gicp_dynamic_two_pose(
            initial_pose0,
            initial_pose1,
            prior_pose0,
            between_measurement,
            &kernel_sources,
            &kernel_targets,
            max_correspondence_distance,
            outer_iterations as usize,
            config,
        ) {
            Ok(result) => {
                *output = CameraLidarTwoPoseOptimizationResultFfi::from_kernel(&result);
                LINGTU_CAMERA_LIDAR_OPT_OK
            }
            Err(error) => calibration_error_code(error),
        }
    });
    result.unwrap_or(LINGTU_CAMERA_LIDAR_OPT_PANIC)
}

unsafe fn ffi_slice<'a, T>(ptr: *const T, len: usize) -> Option<&'a [T]> {
    if len == 0 {
        return Some(&[]);
    }
    if ptr.is_null() {
        return None;
    }
    Some(slice::from_raw_parts(ptr, len))
}

fn calibration_error_code(error: CalibrationKernelError) -> i32 {
    match error {
        CalibrationKernelError::InvalidPose => LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE,
        CalibrationKernelError::InvalidCorrespondence(_) => {
            LINGTU_CAMERA_LIDAR_OPT_INVALID_CORRESPONDENCE
        }
        CalibrationKernelError::InvalidCovariance(_) => LINGTU_CAMERA_LIDAR_OPT_INVALID_COVARIANCE,
        CalibrationKernelError::EmptyCorrespondences => {
            LINGTU_CAMERA_LIDAR_OPT_EMPTY_CORRESPONDENCES
        }
        CalibrationKernelError::InvalidDerivativeStep => {
            LINGTU_CAMERA_LIDAR_OPT_INVALID_DERIVATIVE_STEP
        }
        CalibrationKernelError::LinearSolveFailed => LINGTU_CAMERA_LIDAR_OPT_LINEAR_SOLVE_FAILED,
        CalibrationKernelError::InsufficientOutputCapacity => {
            LINGTU_CAMERA_LIDAR_OPT_INSUFFICIENT_OUTPUT_CAPACITY
        }
    }
}

fn positive_or_default(value: f64, default: f64) -> Option<f64> {
    if value == 0.0 {
        Some(default)
    } else if value.is_finite() && value > 0.0 {
        Some(value)
    } else {
        None
    }
}

fn optimizer_config_is_valid(config: TwoPoseOptimizerConfig) -> bool {
    config.max_iterations > 0
        && config.derivative_eps.is_finite()
        && config.derivative_eps > 0.0
        && config.initial_lambda.is_finite()
        && config.initial_lambda > 0.0
        && config.lambda_factor.is_finite()
        && config.lambda_factor > 1.0
        && config.step_tolerance.is_finite()
        && config.step_tolerance > 0.0
        && config.relative_cost_tolerance.is_finite()
        && config.relative_cost_tolerance > 0.0
        && config.prior_precision.is_finite()
        && config.prior_precision >= 0.0
        && config.between_precision.is_finite()
        && config.between_precision >= 0.0
}

type CtGicpVoxelCoord = (i64, i64, i64);

fn build_ct_gicp_target_voxel_index(
    targets: &[CtGicpTargetPoint],
    voxel_size: f64,
) -> Result<HashMap<CtGicpVoxelCoord, Vec<usize>>, CalibrationKernelError> {
    let mut index: HashMap<CtGicpVoxelCoord, Vec<usize>> = HashMap::new();
    for (target_index, target) in targets.iter().copied().enumerate() {
        if !target.is_valid() {
            return Err(CalibrationKernelError::InvalidCorrespondence(target_index));
        }
        let target_point = Vector3::from_row_slice(&target.xyz);
        let Some(coord) = ct_gicp_voxel_coord(&target_point, voxel_size) else {
            return Err(CalibrationKernelError::InvalidCorrespondence(target_index));
        };
        index.entry(coord).or_default().push(target_index);
    }
    Ok(index)
}

fn ct_gicp_voxel_coord(point: &Vector3<f64>, voxel_size: f64) -> Option<CtGicpVoxelCoord> {
    Some((
        floor_to_i64(point.x / voxel_size)?,
        floor_to_i64(point.y / voxel_size)?,
        floor_to_i64(point.z / voxel_size)?,
    ))
}

fn floor_to_i64(value: f64) -> Option<i64> {
    if !value.is_finite() {
        return None;
    }
    let floored = value.floor();
    if floored < i64::MIN as f64 || floored > i64::MAX as f64 {
        return None;
    }
    Some(floored as i64)
}

fn empty_two_pose_linearization() -> CtIcpLinearization {
    CtIcpLinearization {
        cost: 0.0,
        used_correspondences: 0,
        h00: Matrix6::zeros(),
        h01: Matrix6::zeros(),
        h11: Matrix6::zeros(),
        gradient0: Vector6::zeros(),
        gradient1: Vector6::zeros(),
        rhs0: Vector6::zeros(),
        rhs1: Vector6::zeros(),
    }
}

#[derive(Clone, Copy, Debug)]
struct AffineDerivative {
    d_rotation: Matrix3<f64>,
    d_translation: Vector3<f64>,
}

impl AffineDerivative {
    fn transform_direction(self, point: &Vector3<f64>) -> Vector3<f64> {
        self.d_rotation * point + self.d_translation
    }
}

#[derive(Clone, Copy, Debug)]
struct EndpointTransformDerivatives {
    pose0: [AffineDerivative; 6],
    pose1: [AffineDerivative; 6],
}

fn endpoint_transform_derivatives(
    pose0: Pose3,
    pose1: Pose3,
    time: f64,
    eps: f64,
) -> EndpointTransformDerivatives {
    let zero = AffineDerivative {
        d_rotation: Matrix3::zeros(),
        d_translation: Vector3::zeros(),
    };
    let mut derivatives = EndpointTransformDerivatives {
        pose0: [zero; 6],
        pose1: [zero; 6],
    };
    for dim in 0..6 {
        derivatives.pose0[dim] =
            finite_difference_endpoint_derivative(pose0, pose1, time, eps, dim, true);
        derivatives.pose1[dim] =
            finite_difference_endpoint_derivative(pose0, pose1, time, eps, dim, false);
    }
    derivatives
}

fn finite_difference_endpoint_derivative(
    pose0: Pose3,
    pose1: Pose3,
    time: f64,
    eps: f64,
    dim: usize,
    perturb_first: bool,
) -> AffineDerivative {
    let mut plus_delta = Vector6::zeros();
    plus_delta[dim] = eps;
    let mut minus_delta = Vector6::zeros();
    minus_delta[dim] = -eps;

    let (plus0, plus1) = if perturb_first {
        (pose0.retract(&plus_delta), pose1)
    } else {
        (pose0, pose1.retract(&plus_delta))
    };
    let (minus0, minus1) = if perturb_first {
        (pose0.retract(&minus_delta), pose1)
    } else {
        (pose0, pose1.retract(&minus_delta))
    };
    let plus = plus0.interpolate_rt(plus1, time);
    let minus = minus0.interpolate_rt(minus1, time);
    AffineDerivative {
        d_rotation: (plus.rotation_matrix() - minus.rotation_matrix()) / (2.0 * eps),
        d_translation: (plus.translation() - minus.translation()) / (2.0 * eps),
    }
}

fn normalized_normal(correspondence: CtIcpCorrespondence) -> Vector3<f64> {
    let normal = Vector3::from_row_slice(&correspondence.target_normal);
    normal / normal.norm()
}

fn ct_gicp_mahalanobis(
    rotation: Matrix3<f64>,
    correspondence: CtGicpCorrespondence,
) -> Option<Matrix3<f64>> {
    let source_cov = symmetric_matrix3(correspondence.source_cov_row_major);
    let target_cov = symmetric_matrix3(correspondence.target_cov_row_major);
    let fused_cov = target_cov + rotation * source_cov * rotation.transpose();
    if !matrix3_is_finite(&fused_cov) {
        return None;
    }
    let inverse = fused_cov.try_inverse()?;
    if matrix3_is_finite(&inverse) {
        Some(inverse)
    } else {
        None
    }
}

fn symmetric_matrix3(row_major: [f64; 9]) -> Matrix3<f64> {
    let matrix = Matrix3::from_row_slice(&row_major);
    0.5 * (matrix + matrix.transpose())
}

fn matrix3_is_finite(matrix: &Matrix3<f64>) -> bool {
    matrix.iter().all(|value| value.is_finite())
}

fn skew(vector: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0,
    )
}

fn left_jacobian_so3(omega: &Vector3<f64>) -> Matrix3<f64> {
    let theta = omega.norm();
    let omega_hat = skew(omega);
    let omega_hat2 = omega_hat * omega_hat;
    if theta < 1e-8 {
        Matrix3::identity() + 0.5 * omega_hat + (1.0 / 6.0) * omega_hat2
    } else {
        let theta2 = theta * theta;
        Matrix3::identity()
            + ((1.0 - theta.cos()) / theta2) * omega_hat
            + ((theta - theta.sin()) / (theta2 * theta)) * omega_hat2
    }
}

fn left_jacobian_so3_inverse(omega: &Vector3<f64>) -> Matrix3<f64> {
    let theta = omega.norm();
    let omega_hat = skew(omega);
    let omega_hat2 = omega_hat * omega_hat;
    if theta < 1e-8 {
        Matrix3::identity() - 0.5 * omega_hat + (1.0 / 12.0) * omega_hat2
    } else {
        let theta2 = theta * theta;
        let half_theta = 0.5 * theta;
        let coefficient = (1.0 / theta2) * (1.0 - half_theta * theta.sin() / (1.0 - theta.cos()));
        Matrix3::identity() - 0.5 * omega_hat + coefficient * omega_hat2
    }
}

#[allow(dead_code)]
fn transform_matrix_3x4(pose: Pose3) -> Matrix3x4<f64> {
    let mut matrix = Matrix3x4::zeros();
    matrix
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&pose.rotation_matrix());
    matrix
        .fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&pose.translation());
    matrix
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f64, expected: f64, tol: f64) {
        assert!(
            (actual - expected).abs() <= tol,
            "actual={actual} expected={expected} tol={tol}"
        );
    }

    fn assert_vec6_close(actual: &Vector6, expected: &Vector6, tol: f64) {
        for idx in 0..6 {
            assert_close(actual[idx], expected[idx], tol);
        }
    }

    fn sample_correspondence(time: f64) -> CtIcpCorrespondence {
        CtIcpCorrespondence::try_new(time, [1.0, 0.2, -0.1], [0.4, 0.1, 0.0], [0.3, -0.4, 0.5])
            .unwrap()
    }

    fn sample_gicp_correspondence(time: f64) -> CtGicpCorrespondence {
        CtGicpCorrespondence::try_new(
            time,
            [1.0, 0.2, -0.1],
            [0.4, 0.1, 0.0],
            diagonal_covariance([0.05, 0.08, 0.11]),
            diagonal_covariance([0.25, 0.5, 1.5]),
        )
        .unwrap()
    }

    fn diagonal_covariance(diagonal: [f64; 3]) -> [f64; 9] {
        [
            diagonal[0],
            0.0,
            0.0,
            0.0,
            diagonal[1],
            0.0,
            0.0,
            0.0,
            diagonal[2],
        ]
    }

    fn sample_source_point(time: f64, xyz: [f64; 3]) -> CtGicpSourcePoint {
        CtGicpSourcePoint::try_new(time, xyz, diagonal_covariance([0.05, 0.08, 0.11])).unwrap()
    }

    fn sample_target_point(xyz: [f64; 3]) -> CtGicpTargetPoint {
        CtGicpTargetPoint::try_new(xyz, diagonal_covariance([0.25, 0.5, 1.5])).unwrap()
    }

    fn identity_pose_ffi() -> CameraLidarPose3Ffi {
        CameraLidarPose3Ffi {
            t_xyz: [0.0, 0.0, 0.0],
            q_wxyz: [1.0, 0.0, 0.0, 0.0],
        }
    }

    fn empty_ffi_output() -> CameraLidarTwoPoseLinearizationFfi {
        CameraLidarTwoPoseLinearizationFfi {
            cost: 0.0,
            used_correspondences: 0,
            hessian_12x12_row_major: [0.0; 144],
            gradient_12: [0.0; 12],
            rhs_12: [0.0; 12],
        }
    }

    fn empty_optimizer_output() -> CameraLidarTwoPoseOptimizationResultFfi {
        CameraLidarTwoPoseOptimizationResultFfi {
            pose0: identity_pose_ffi(),
            pose1: identity_pose_ffi(),
            initial_cost: 0.0,
            final_cost: 0.0,
            iterations: 0,
            accepted_steps: 0,
            last_lambda: 0.0,
            used_correspondences: 0,
        }
    }

    fn empty_gicp_correspondence_ffi() -> CameraLidarCtGicpCorrespondenceFfi {
        CameraLidarCtGicpCorrespondenceFfi {
            time: 0.0,
            source_xyz: [0.0, 0.0, 0.0],
            target_xyz: [0.0, 0.0, 0.0],
            source_cov_row_major: diagonal_covariance([1.0, 1.0, 1.0]),
            target_cov_row_major: diagonal_covariance([1.0, 1.0, 1.0]),
        }
    }

    fn unconstrained_optimizer_config() -> TwoPoseOptimizerConfig {
        TwoPoseOptimizerConfig {
            max_iterations: 20,
            derivative_eps: DEFAULT_DERIVATIVE_EPS,
            initial_lambda: 1e-3,
            lambda_factor: 10.0,
            step_tolerance: 1e-9,
            relative_cost_tolerance: 1e-10,
            prior_precision: 0.0,
            between_precision: 0.0,
        }
    }

    fn unconstrained_optimizer_config_ffi() -> CameraLidarTwoPoseOptimizerConfigFfi {
        CameraLidarTwoPoseOptimizerConfigFfi {
            max_iterations: 20,
            derivative_eps: DEFAULT_DERIVATIVE_EPS,
            initial_lambda: 1e-3,
            lambda_factor: 10.0,
            step_tolerance: 1e-9,
            relative_cost_tolerance: 1e-10,
            prior_precision: 0.0,
            between_precision: 0.0,
        }
    }

    fn residual_for_perturb(
        pose0: Pose3,
        pose1: Pose3,
        correspondence: CtIcpCorrespondence,
        delta: &Vector6,
        first: bool,
    ) -> f64 {
        let next0 = if first { pose0.retract(delta) } else { pose0 };
        let next1 = if first { pose1 } else { pose1.retract(delta) };
        let pose = next0.interpolate_rt(next1, correspondence.time);
        ct_icp_residual(pose, correspondence)
    }

    fn gicp_cost_for_perturb(
        pose0: Pose3,
        pose1: Pose3,
        correspondence: CtGicpCorrespondence,
        delta: &Vector6,
        first: bool,
    ) -> f64 {
        let next0 = if first { pose0.retract(delta) } else { pose0 };
        let next1 = if first { pose1 } else { pose1.retract(delta) };
        let pose = next0.interpolate_rt(next1, correspondence.time);
        let residual = ct_gicp_residual_vector(pose, correspondence);
        let mahalanobis = ct_gicp_mahalanobis(pose.rotation_matrix(), correspondence).unwrap();
        0.5 * residual.dot(&(mahalanobis * residual))
    }

    #[test]
    fn se3_exp_log_roundtrip() {
        let delta = Vector6::from_row_slice(&[0.12, -0.08, 0.04, 0.5, -0.2, 0.1]);
        let pose = Pose3::exp(&delta);
        assert_vec6_close(&pose.log(), &delta, 1e-9);
    }

    #[test]
    fn ct_icp_zero_residual_has_zero_cost_and_gradient() {
        let correspondence =
            CtIcpCorrespondence::try_new(0.0, [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0])
                .unwrap();

        let result = linearize_ct_icp_fixed_correspondences(
            Pose3::identity(),
            Pose3::identity(),
            &[correspondence],
        )
        .unwrap();

        assert_close(result.cost, 0.0, 1e-12);
        assert_close(result.gradient0.norm(), 0.0, 1e-12);
        assert_close(result.gradient1.norm(), 0.0, 1e-12);
    }

    #[test]
    fn ct_icp_t0_translation_gradient_matches_point_to_plane_error() {
        let correspondence =
            CtIcpCorrespondence::try_new(0.0, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
                .unwrap();

        let result = linearize_ct_icp_fixed_correspondences(
            Pose3::identity(),
            Pose3::identity(),
            &[correspondence],
        )
        .unwrap();

        assert_close(result.cost, 0.5, 1e-10);
        assert_close(result.gradient0[3], 1.0, 1e-8);
        assert_close(result.rhs0[3], -1.0, 1e-8);
        assert_close(result.gradient1.norm(), 0.0, 1e-8);
        assert_close(result.h00[(3, 3)], 1.0, 1e-8);
    }

    #[test]
    fn ct_interpolation_midpoint_reaches_translated_target() {
        let pose1 = Pose3::exp(&Vector6::from_row_slice(&[0.0, 0.0, 0.0, 2.0, 0.0, 0.0]));
        let correspondence =
            CtIcpCorrespondence::try_new(0.5, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0])
                .unwrap();
        let pose = Pose3::identity().interpolate_rt(pose1, 0.5);

        assert_close(ct_icp_residual(pose, correspondence), 0.0, 1e-12);
    }

    #[test]
    fn ct_icp_jacobians_match_residual_finite_difference() {
        let pose0 = Pose3::exp(&Vector6::from_row_slice(&[
            0.05, -0.1, 0.02, 0.3, -0.2, 0.1,
        ]));
        let pose1 = Pose3::exp(&Vector6::from_row_slice(&[
            0.1, 0.04, -0.08, 0.7, 0.2, -0.1,
        ]));
        let correspondence = sample_correspondence(0.35);
        let result =
            linearize_ct_icp_fixed_correspondences(pose0, pose1, &[correspondence]).unwrap();
        let eps = 1e-6;

        for dim in 0..6 {
            let mut delta_plus = Vector6::zeros();
            delta_plus[dim] = eps;
            let mut delta_minus = Vector6::zeros();
            delta_minus[dim] = -eps;
            let fd0 = (residual_for_perturb(pose0, pose1, correspondence, &delta_plus, true)
                - residual_for_perturb(pose0, pose1, correspondence, &delta_minus, true))
                / (2.0 * eps);
            let fd1 = (residual_for_perturb(pose0, pose1, correspondence, &delta_plus, false)
                - residual_for_perturb(pose0, pose1, correspondence, &delta_minus, false))
                / (2.0 * eps);
            let residual =
                residual_for_perturb(pose0, pose1, correspondence, &Vector6::zeros(), true);
            assert_close(result.gradient0[dim], fd0 * residual, 1e-7);
            assert_close(result.gradient1[dim], fd1 * residual, 1e-7);
        }
    }

    #[test]
    fn ct_icp_accumulates_symmetric_psd_hessian() {
        let pose0 = Pose3::exp(&Vector6::from_row_slice(&[
            0.01, 0.02, -0.03, 0.1, 0.0, 0.0,
        ]));
        let pose1 = Pose3::exp(&Vector6::from_row_slice(&[
            -0.02, 0.01, 0.04, 0.4, 0.1, -0.1,
        ]));
        let correspondences = [
            sample_correspondence(0.0),
            sample_correspondence(0.4),
            sample_correspondence(1.0),
        ];
        let result =
            linearize_ct_icp_fixed_correspondences(pose0, pose1, &correspondences).unwrap();
        let h = result.hessian_12x12();

        assert_eq!(result.used_correspondences, 3);
        assert_close((h - h.transpose()).norm(), 0.0, 1e-9);
        for idx in 0..12 {
            assert!(
                h[(idx, idx)] >= -1e-10,
                "negative diagonal {idx}: {}",
                h[(idx, idx)]
            );
        }
    }

    #[test]
    fn ct_gicp_zero_residual_has_zero_cost_and_gradient() {
        let correspondence = CtGicpCorrespondence::try_new(
            0.0,
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            diagonal_covariance([0.1, 0.1, 0.1]),
            diagonal_covariance([0.2, 0.2, 0.2]),
        )
        .unwrap();

        let result = linearize_ct_gicp_fixed_correspondences(
            Pose3::identity(),
            Pose3::identity(),
            &[correspondence],
        )
        .unwrap();

        assert_close(result.cost, 0.0, 1e-12);
        assert_close(result.gradient0.norm(), 0.0, 1e-12);
        assert_close(result.gradient1.norm(), 0.0, 1e-12);
    }

    #[test]
    fn ct_gicp_covariance_weighting_matches_mahalanobis_error() {
        let correspondence = CtGicpCorrespondence::try_new(
            0.0,
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            diagonal_covariance([0.0, 0.0, 0.0]),
            diagonal_covariance([0.25, 1.0, 1.0]),
        )
        .unwrap();

        let result = linearize_ct_gicp_fixed_correspondences(
            Pose3::identity(),
            Pose3::identity(),
            &[correspondence],
        )
        .unwrap();

        assert_close(result.cost, 2.0, 1e-10);
        assert_close(result.gradient0[3], 4.0, 1e-8);
        assert_close(result.rhs0[3], -4.0, 1e-8);
        assert_close(result.gradient1.norm(), 0.0, 1e-8);
        assert_close(result.h00[(3, 3)], 4.0, 1e-8);
    }

    #[test]
    fn ct_gicp_nearest_neighbor_correspondences_selects_closest_target() {
        let sources = [sample_source_point(0.0, [0.0, 0.0, 0.0])];
        let targets = [
            sample_target_point([10.0, 0.0, 0.0]),
            sample_target_point([1.0, 0.0, 0.0]),
        ];

        let correspondences = build_ct_gicp_nearest_neighbor_correspondences(
            Pose3::identity(),
            Pose3::identity(),
            &sources,
            &targets,
            2.0,
        )
        .unwrap();

        assert_eq!(correspondences.len(), 1);
        assert_eq!(correspondences[0].source_xyz, [0.0, 0.0, 0.0]);
        assert_eq!(correspondences[0].target_xyz, [1.0, 0.0, 0.0]);
    }

    #[test]
    fn ct_gicp_nearest_neighbor_correspondences_apply_interpolated_pose() {
        let pose1 = Pose3::exp(&Vector6::from_row_slice(&[0.0, 0.0, 0.0, 2.0, 0.0, 0.0]));
        let sources = [sample_source_point(0.5, [0.0, 0.0, 0.0])];
        let targets = [sample_target_point([1.0, 0.0, 0.0])];

        let correspondences = build_ct_gicp_nearest_neighbor_correspondences(
            Pose3::identity(),
            pose1,
            &sources,
            &targets,
            1e-6,
        )
        .unwrap();

        assert_eq!(correspondences.len(), 1);
        assert_eq!(correspondences[0].time, 0.5);
        assert_eq!(correspondences[0].target_xyz, [1.0, 0.0, 0.0]);
    }

    #[test]
    fn ct_gicp_nearest_neighbor_correspondences_rejects_by_max_distance() {
        let sources = [sample_source_point(0.0, [0.0, 0.0, 0.0])];
        let targets = [sample_target_point([2.0, 0.0, 0.0])];

        let correspondences = build_ct_gicp_nearest_neighbor_correspondences(
            Pose3::identity(),
            Pose3::identity(),
            &sources,
            &targets,
            0.5,
        )
        .unwrap();

        assert!(correspondences.is_empty());
    }

    #[test]
    fn ct_gicp_jacobians_match_cost_finite_difference_with_fixed_information() {
        let pose0 = Pose3::exp(&Vector6::from_row_slice(&[
            0.05, -0.1, 0.02, 0.3, -0.2, 0.1,
        ]));
        let pose1 = Pose3::exp(&Vector6::from_row_slice(&[
            0.1, 0.04, -0.08, 0.7, 0.2, -0.1,
        ]));
        let correspondence = CtGicpCorrespondence::try_new(
            0.35,
            [1.0, 0.2, -0.1],
            [0.4, 0.1, 0.0],
            diagonal_covariance([0.0, 0.0, 0.0]),
            diagonal_covariance([0.25, 0.5, 1.5]),
        )
        .unwrap();
        let result =
            linearize_ct_gicp_fixed_correspondences(pose0, pose1, &[correspondence]).unwrap();
        let eps = 1e-6;

        for dim in 0..6 {
            let mut delta_plus = Vector6::zeros();
            delta_plus[dim] = eps;
            let mut delta_minus = Vector6::zeros();
            delta_minus[dim] = -eps;
            let fd0 = (gicp_cost_for_perturb(pose0, pose1, correspondence, &delta_plus, true)
                - gicp_cost_for_perturb(pose0, pose1, correspondence, &delta_minus, true))
                / (2.0 * eps);
            let fd1 = (gicp_cost_for_perturb(pose0, pose1, correspondence, &delta_plus, false)
                - gicp_cost_for_perturb(pose0, pose1, correspondence, &delta_minus, false))
                / (2.0 * eps);
            assert_close(result.gradient0[dim], fd0, 1e-6);
            assert_close(result.gradient1[dim], fd1, 1e-6);
        }
    }

    #[test]
    fn ct_gicp_accumulates_symmetric_psd_hessian() {
        let pose0 = Pose3::exp(&Vector6::from_row_slice(&[
            0.01, 0.02, -0.03, 0.1, 0.0, 0.0,
        ]));
        let pose1 = Pose3::exp(&Vector6::from_row_slice(&[
            -0.02, 0.01, 0.04, 0.4, 0.1, -0.1,
        ]));
        let correspondences = [
            sample_gicp_correspondence(0.0),
            sample_gicp_correspondence(0.4),
            sample_gicp_correspondence(1.0),
        ];
        let result =
            linearize_ct_gicp_fixed_correspondences(pose0, pose1, &correspondences).unwrap();
        let h = result.hessian_12x12();

        assert_eq!(result.used_correspondences, 3);
        assert_close((h - h.transpose()).norm(), 0.0, 1e-9);
        for idx in 0..12 {
            assert!(
                h[(idx, idx)] >= -1e-10,
                "negative diagonal {idx}: {}",
                h[(idx, idx)]
            );
        }
    }

    #[test]
    fn ct_gicp_rejects_singular_fused_covariance() {
        let correspondence = CtGicpCorrespondence::try_new(
            0.0,
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            diagonal_covariance([0.0, 0.0, 0.0]),
            diagonal_covariance([1.0, 0.0, 0.0]),
        )
        .unwrap();

        let result = linearize_ct_gicp_fixed_correspondences(
            Pose3::identity(),
            Pose3::identity(),
            &[correspondence],
        );

        assert!(matches!(
            result,
            Err(CalibrationKernelError::InvalidCovariance(0))
        ));
    }

    #[test]
    fn abi_reports_version_and_struct_sizes() {
        assert_eq!(lingtu_camera_lidar_optimizer_abi_version(), 2);
        assert_eq!(
            lingtu_camera_lidar_optimizer_sizeof_pose3(),
            std::mem::size_of::<CameraLidarPose3Ffi>()
        );
        assert_eq!(
            lingtu_camera_lidar_optimizer_sizeof_ct_icp_correspondence(),
            std::mem::size_of::<CameraLidarCtIcpCorrespondenceFfi>()
        );
        assert_eq!(
            lingtu_camera_lidar_optimizer_sizeof_ct_gicp_correspondence(),
            std::mem::size_of::<CameraLidarCtGicpCorrespondenceFfi>()
        );
        assert_eq!(
            lingtu_camera_lidar_optimizer_sizeof_ct_gicp_source_point(),
            std::mem::size_of::<CameraLidarCtGicpSourcePointFfi>()
        );
        assert_eq!(
            lingtu_camera_lidar_optimizer_sizeof_ct_gicp_target_point(),
            std::mem::size_of::<CameraLidarCtGicpTargetPointFfi>()
        );
        assert_eq!(
            lingtu_camera_lidar_optimizer_sizeof_two_pose_linearization(),
            std::mem::size_of::<CameraLidarTwoPoseLinearizationFfi>()
        );
        assert_eq!(
            lingtu_camera_lidar_optimizer_sizeof_two_pose_optimizer_config(),
            std::mem::size_of::<CameraLidarTwoPoseOptimizerConfigFfi>()
        );
        assert_eq!(
            lingtu_camera_lidar_optimizer_sizeof_two_pose_optimization_result(),
            std::mem::size_of::<CameraLidarTwoPoseOptimizationResultFfi>()
        );
    }

    #[test]
    fn abi_linearizes_ct_icp() {
        let pose0 = identity_pose_ffi();
        let pose1 = identity_pose_ffi();
        let correspondence = CameraLidarCtIcpCorrespondenceFfi {
            time: 0.0,
            source_xyz: [1.0, 0.0, 0.0],
            target_xyz: [0.0, 0.0, 0.0],
            target_normal: [1.0, 0.0, 0.0],
        };
        let mut output = empty_ffi_output();

        let code = lingtu_camera_lidar_optimizer_linearize_ct_icp(
            &pose0,
            &pose1,
            &correspondence,
            1,
            DEFAULT_DERIVATIVE_EPS,
            &mut output,
        );

        assert_eq!(code, LINGTU_CAMERA_LIDAR_OPT_OK);
        assert_close(output.cost, 0.5, 1e-10);
        assert_eq!(output.used_correspondences, 1);
        assert_close(output.gradient_12[3], 1.0, 1e-8);
        assert_close(output.rhs_12[3], -1.0, 1e-8);
        assert_close(output.hessian_12x12_row_major[3 * 12 + 3], 1.0, 1e-8);
    }

    #[test]
    fn abi_linearizes_ct_gicp() {
        let pose0 = identity_pose_ffi();
        let pose1 = identity_pose_ffi();
        let correspondence = CameraLidarCtGicpCorrespondenceFfi {
            time: 0.0,
            source_xyz: [1.0, 0.0, 0.0],
            target_xyz: [0.0, 0.0, 0.0],
            source_cov_row_major: diagonal_covariance([0.0, 0.0, 0.0]),
            target_cov_row_major: diagonal_covariance([0.25, 1.0, 1.0]),
        };
        let mut output = empty_ffi_output();

        let code = lingtu_camera_lidar_optimizer_linearize_ct_gicp(
            &pose0,
            &pose1,
            &correspondence,
            1,
            DEFAULT_DERIVATIVE_EPS,
            &mut output,
        );

        assert_eq!(code, LINGTU_CAMERA_LIDAR_OPT_OK);
        assert_close(output.cost, 2.0, 1e-10);
        assert_eq!(output.used_correspondences, 1);
        assert_close(output.gradient_12[3], 4.0, 1e-8);
        assert_close(output.rhs_12[3], -4.0, 1e-8);
        assert_close(output.hessian_12x12_row_major[3 * 12 + 3], 4.0, 1e-8);
    }

    #[test]
    fn abi_builds_ct_gicp_correspondences() {
        let pose0 = identity_pose_ffi();
        let pose1 = CameraLidarPose3Ffi {
            t_xyz: [2.0, 0.0, 0.0],
            q_wxyz: [1.0, 0.0, 0.0, 0.0],
        };
        let sources = [CameraLidarCtGicpSourcePointFfi {
            time: 0.5,
            xyz: [0.0, 0.0, 0.0],
            cov_row_major: diagonal_covariance([0.05, 0.08, 0.11]),
        }];
        let targets = [
            CameraLidarCtGicpTargetPointFfi {
                xyz: [3.0, 0.0, 0.0],
                cov_row_major: diagonal_covariance([0.25, 0.5, 1.5]),
            },
            CameraLidarCtGicpTargetPointFfi {
                xyz: [1.0, 0.0, 0.0],
                cov_row_major: diagonal_covariance([0.3, 0.6, 1.6]),
            },
        ];
        let mut output = [empty_gicp_correspondence_ffi(); 2];
        let mut output_count = 0usize;

        let code = lingtu_camera_lidar_optimizer_build_ct_gicp_correspondences(
            &pose0,
            &pose1,
            sources.as_ptr(),
            sources.len(),
            targets.as_ptr(),
            targets.len(),
            0.25,
            output.as_mut_ptr(),
            output.len(),
            &mut output_count,
        );

        assert_eq!(code, LINGTU_CAMERA_LIDAR_OPT_OK);
        assert_eq!(output_count, 1);
        assert_eq!(output[0].time, 0.5);
        assert_eq!(output[0].source_xyz, [0.0, 0.0, 0.0]);
        assert_eq!(output[0].target_xyz, [1.0, 0.0, 0.0]);
        assert_eq!(output[0].target_cov_row_major, targets[1].cov_row_major);
    }

    #[test]
    fn abi_build_ct_gicp_correspondences_reports_required_capacity() {
        let pose = identity_pose_ffi();
        let sources = [CameraLidarCtGicpSourcePointFfi {
            time: 0.0,
            xyz: [0.0, 0.0, 0.0],
            cov_row_major: diagonal_covariance([0.05, 0.08, 0.11]),
        }];
        let targets = [CameraLidarCtGicpTargetPointFfi {
            xyz: [0.0, 0.0, 0.0],
            cov_row_major: diagonal_covariance([0.25, 0.5, 1.5]),
        }];
        let mut output_count = 0usize;

        let code = lingtu_camera_lidar_optimizer_build_ct_gicp_correspondences(
            &pose,
            &pose,
            sources.as_ptr(),
            sources.len(),
            targets.as_ptr(),
            targets.len(),
            1.0,
            std::ptr::null_mut(),
            0,
            &mut output_count,
        );

        assert_eq!(code, LINGTU_CAMERA_LIDAR_OPT_INSUFFICIENT_OUTPUT_CAPACITY);
        assert_eq!(output_count, 1);
    }

    #[test]
    fn abi_rejects_null_output_pointer() {
        let pose0 = identity_pose_ffi();
        let pose1 = identity_pose_ffi();
        let correspondence = CameraLidarCtIcpCorrespondenceFfi {
            time: 0.0,
            source_xyz: [1.0, 0.0, 0.0],
            target_xyz: [0.0, 0.0, 0.0],
            target_normal: [1.0, 0.0, 0.0],
        };

        let code = lingtu_camera_lidar_optimizer_linearize_ct_icp(
            &pose0,
            &pose1,
            &correspondence,
            1,
            DEFAULT_DERIVATIVE_EPS,
            std::ptr::null_mut(),
        );

        assert_eq!(code, LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER);
    }

    #[test]
    fn abi_separates_null_pose_from_invalid_pose() {
        let pose1 = identity_pose_ffi();
        let correspondence = CameraLidarCtIcpCorrespondenceFfi {
            time: 0.0,
            source_xyz: [1.0, 0.0, 0.0],
            target_xyz: [0.0, 0.0, 0.0],
            target_normal: [1.0, 0.0, 0.0],
        };
        let mut output = empty_ffi_output();

        let null_code = lingtu_camera_lidar_optimizer_linearize_ct_icp(
            std::ptr::null(),
            &pose1,
            &correspondence,
            1,
            DEFAULT_DERIVATIVE_EPS,
            &mut output,
        );

        let invalid_pose0 = CameraLidarPose3Ffi {
            t_xyz: [0.0, 0.0, 0.0],
            q_wxyz: [0.0, 0.0, 0.0, 0.0],
        };
        let invalid_code = lingtu_camera_lidar_optimizer_linearize_ct_icp(
            &invalid_pose0,
            &pose1,
            &correspondence,
            1,
            DEFAULT_DERIVATIVE_EPS,
            &mut output,
        );

        assert_eq!(null_code, LINGTU_CAMERA_LIDAR_OPT_NULL_POINTER);
        assert_eq!(invalid_code, LINGTU_CAMERA_LIDAR_OPT_INVALID_POSE);
    }

    #[test]
    fn ct_gicp_two_pose_optimizer_reduces_cost_and_recovers_translation() {
        let correspondence = CtGicpCorrespondence::try_new(
            0.0,
            [0.0, 0.0, 0.0],
            [1.0, 2.0, 3.0],
            diagonal_covariance([0.0, 0.0, 0.0]),
            diagonal_covariance([1.0, 1.0, 1.0]),
        )
        .unwrap();

        let result = optimize_ct_gicp_two_pose(
            Pose3::identity(),
            Pose3::identity(),
            Pose3::identity(),
            Pose3::identity(),
            &[correspondence],
            unconstrained_optimizer_config(),
        )
        .unwrap();

        assert!(result.final_cost < result.initial_cost);
        assert!(result.accepted_steps > 0);
        assert_close(result.pose0.t_xyz[0], 1.0, 1e-5);
        assert_close(result.pose0.t_xyz[1], 2.0, 1e-5);
        assert_close(result.pose0.t_xyz[2], 3.0, 1e-5);
        assert_close(result.pose1.translation().norm(), 0.0, 1e-8);
    }

    #[test]
    fn ct_gicp_dynamic_two_pose_optimizer_rebuilds_correspondences_and_recovers_translation() {
        let sources = [sample_source_point(0.0, [0.0, 0.0, 0.0])];
        let targets =
            [
                CtGicpTargetPoint::try_new([1.0, 2.0, 3.0], diagonal_covariance([1.0, 1.0, 1.0]))
                    .unwrap(),
            ];

        let result = optimize_ct_gicp_dynamic_two_pose(
            Pose3::identity(),
            Pose3::identity(),
            Pose3::identity(),
            Pose3::identity(),
            &sources,
            &targets,
            5.0,
            3,
            unconstrained_optimizer_config(),
        )
        .unwrap();

        assert!(result.final_cost < result.initial_cost);
        assert!(result.iterations >= 1);
        assert!(result.accepted_steps > 0);
        assert_eq!(result.used_correspondences, 1);
        assert_close(result.pose0.t_xyz[0], 1.0, 1e-5);
        assert_close(result.pose0.t_xyz[1], 2.0, 1e-5);
        assert_close(result.pose0.t_xyz[2], 3.0, 1e-5);
        assert_close(result.pose1.translation().norm(), 0.0, 1e-8);
    }

    #[test]
    fn abi_optimizes_ct_gicp_two_pose() {
        let initial_pose0 = identity_pose_ffi();
        let initial_pose1 = identity_pose_ffi();
        let prior_pose0 = identity_pose_ffi();
        let between_measurement = identity_pose_ffi();
        let correspondence = CameraLidarCtGicpCorrespondenceFfi {
            time: 0.0,
            source_xyz: [0.0, 0.0, 0.0],
            target_xyz: [1.0, 2.0, 3.0],
            source_cov_row_major: diagonal_covariance([0.0, 0.0, 0.0]),
            target_cov_row_major: diagonal_covariance([1.0, 1.0, 1.0]),
        };
        let config = unconstrained_optimizer_config_ffi();
        let mut output = empty_optimizer_output();

        let code = lingtu_camera_lidar_optimizer_optimize_ct_gicp_two_pose(
            &initial_pose0,
            &initial_pose1,
            &prior_pose0,
            &between_measurement,
            &correspondence,
            1,
            &config,
            &mut output,
        );

        assert_eq!(code, LINGTU_CAMERA_LIDAR_OPT_OK);
        assert!(output.final_cost < output.initial_cost);
        assert!(output.accepted_steps > 0);
        assert_eq!(output.used_correspondences, 1);
        assert_close(output.pose0.t_xyz[0], 1.0, 1e-5);
        assert_close(output.pose0.t_xyz[1], 2.0, 1e-5);
        assert_close(output.pose0.t_xyz[2], 3.0, 1e-5);
    }

    #[test]
    fn abi_optimizes_ct_gicp_dynamic_two_pose() {
        let initial_pose0 = identity_pose_ffi();
        let initial_pose1 = identity_pose_ffi();
        let prior_pose0 = identity_pose_ffi();
        let between_measurement = identity_pose_ffi();
        let sources = [CameraLidarCtGicpSourcePointFfi {
            time: 0.0,
            xyz: [0.0, 0.0, 0.0],
            cov_row_major: diagonal_covariance([0.0, 0.0, 0.0]),
        }];
        let targets = [CameraLidarCtGicpTargetPointFfi {
            xyz: [1.0, 2.0, 3.0],
            cov_row_major: diagonal_covariance([1.0, 1.0, 1.0]),
        }];
        let config = unconstrained_optimizer_config_ffi();
        let mut output = empty_optimizer_output();

        let code = lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose(
            &initial_pose0,
            &initial_pose1,
            &prior_pose0,
            &between_measurement,
            sources.as_ptr(),
            sources.len(),
            targets.as_ptr(),
            targets.len(),
            5.0,
            3,
            &config,
            &mut output,
        );

        assert_eq!(code, LINGTU_CAMERA_LIDAR_OPT_OK);
        assert!(output.final_cost < output.initial_cost);
        assert!(output.accepted_steps > 0);
        assert_eq!(output.used_correspondences, 1);
        assert_close(output.pose0.t_xyz[0], 1.0, 1e-5);
        assert_close(output.pose0.t_xyz[1], 2.0, 1e-5);
        assert_close(output.pose0.t_xyz[2], 3.0, 1e-5);
    }
}
