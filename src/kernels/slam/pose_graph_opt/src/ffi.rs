use core::ffi::c_int;

use crate::geometry::Pose3;
use crate::solver::{
    information_from_upper, optimize_pose_graph3, BetweenFactor3, OptimizerConfig, OptimizerReport,
    PoseGraph3, PriorFactor3,
};
use crate::{
    LT_POSE_GRAPH_OPT_ABI_VERSION, LT_POSE_GRAPH_OPT_BUFFER_TOO_SMALL,
    LT_POSE_GRAPH_OPT_EMPTY_GRAPH, LT_POSE_GRAPH_OPT_INVALID_CONFIG,
    LT_POSE_GRAPH_OPT_NON_FINITE_INPUT, LT_POSE_GRAPH_OPT_NULL_POINTER, LT_POSE_GRAPH_OPT_OK,
};

pub const LT_POSE_GRAPH_OPT_CONFIG_VERSION: u32 = 1;
pub const LT_POSE_GRAPH_OPT_REPORT_VERSION: u32 = 1;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LtPoseGraphOptPose3 {
    pub t_xyz: [f64; 3],
    pub q_wxyz: [f64; 4],
}

impl From<Pose3> for LtPoseGraphOptPose3 {
    fn from(value: Pose3) -> Self {
        Self {
            t_xyz: value.t_xyz,
            q_wxyz: value.q_wxyz,
        }
    }
}

impl TryFrom<LtPoseGraphOptPose3> for Pose3 {
    type Error = c_int;

    fn try_from(value: LtPoseGraphOptPose3) -> Result<Self, Self::Error> {
        Pose3::try_new(value.t_xyz, value.q_wxyz).ok_or(LT_POSE_GRAPH_OPT_NON_FINITE_INPUT)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LtPoseGraphOptPrior3 {
    pub index: u32,
    pub reserved0: u32,
    pub pose: LtPoseGraphOptPose3,
    pub information_upper: [f64; 21],
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LtPoseGraphOptBetween3 {
    pub from_index: u32,
    pub to_index: u32,
    pub pose_from_to: LtPoseGraphOptPose3,
    pub information_upper: [f64; 21],
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LtPoseGraphOptConfig {
    pub struct_size: u32,
    pub version: u32,
    pub max_iterations: u32,
    pub method: u32,
    pub fixed_pose_index: i32,
    pub auto_anchor: u32,
    pub initial_lambda: f64,
    pub tolerance: f64,
    pub numeric_epsilon: f64,
}

impl Default for LtPoseGraphOptConfig {
    fn default() -> Self {
        let config = OptimizerConfig::default();
        Self {
            struct_size: core::mem::size_of::<Self>() as u32,
            version: LT_POSE_GRAPH_OPT_CONFIG_VERSION,
            max_iterations: config.max_iterations as u32,
            method: 1,
            fixed_pose_index: config.fixed_pose_index.map(|idx| idx as i32).unwrap_or(-1),
            auto_anchor: u32::from(config.auto_anchor),
            initial_lambda: config.initial_lambda,
            tolerance: config.tolerance,
            numeric_epsilon: config.numeric_epsilon,
        }
    }
}

impl TryFrom<LtPoseGraphOptConfig> for OptimizerConfig {
    type Error = c_int;

    fn try_from(value: LtPoseGraphOptConfig) -> Result<Self, Self::Error> {
        if value.struct_size != 0
            && value.struct_size as usize != core::mem::size_of::<LtPoseGraphOptConfig>()
        {
            return Err(LT_POSE_GRAPH_OPT_INVALID_CONFIG);
        }
        if value.version != 0 && value.version != LT_POSE_GRAPH_OPT_CONFIG_VERSION {
            return Err(LT_POSE_GRAPH_OPT_INVALID_CONFIG);
        }
        if value.method > 1 {
            return Err(LT_POSE_GRAPH_OPT_INVALID_CONFIG);
        }
        Ok(OptimizerConfig {
            max_iterations: value.max_iterations as usize,
            initial_lambda: value.initial_lambda,
            tolerance: value.tolerance,
            numeric_epsilon: value.numeric_epsilon,
            fixed_pose_index: if value.fixed_pose_index >= 0 {
                Some(value.fixed_pose_index as usize)
            } else {
                None
            },
            auto_anchor: value.auto_anchor != 0,
        })
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LtPoseGraphOptReport {
    pub struct_size: u32,
    pub version: u32,
    pub status: c_int,
    pub iterations: u32,
    pub accepted_steps: u32,
    pub rejected_steps: u32,
    pub converged: u32,
    pub reserved0: u32,
    pub initial_cost: f64,
    pub final_cost: f64,
}

impl Default for LtPoseGraphOptReport {
    fn default() -> Self {
        Self {
            struct_size: core::mem::size_of::<Self>() as u32,
            version: LT_POSE_GRAPH_OPT_REPORT_VERSION,
            status: LT_POSE_GRAPH_OPT_OK,
            iterations: 0,
            accepted_steps: 0,
            rejected_steps: 0,
            converged: 0,
            reserved0: 0,
            initial_cost: f64::NAN,
            final_cost: f64::NAN,
        }
    }
}

impl From<OptimizerReport> for LtPoseGraphOptReport {
    fn from(value: OptimizerReport) -> Self {
        Self {
            iterations: value.iterations as u32,
            accepted_steps: value.accepted_steps as u32,
            rejected_steps: value.rejected_steps as u32,
            converged: u32::from(value.converged),
            initial_cost: value.initial_cost,
            final_cost: value.final_cost,
            ..LtPoseGraphOptReport::default()
        }
    }
}

pub struct LtPoseGraphOptWorkspace {
    config: OptimizerConfig,
    last_error: c_int,
    result_poses: Vec<LtPoseGraphOptPose3>,
}

#[no_mangle]
pub extern "C" fn lt_pose_graph_opt_abi_version() -> u32 {
    LT_POSE_GRAPH_OPT_ABI_VERSION
}

#[no_mangle]
pub extern "C" fn lt_pose_graph_opt_abi_sizeof_pose3() -> u64 {
    core::mem::size_of::<LtPoseGraphOptPose3>() as u64
}

#[no_mangle]
pub extern "C" fn lt_pose_graph_opt_abi_sizeof_prior3() -> u64 {
    core::mem::size_of::<LtPoseGraphOptPrior3>() as u64
}

#[no_mangle]
pub extern "C" fn lt_pose_graph_opt_abi_sizeof_between3() -> u64 {
    core::mem::size_of::<LtPoseGraphOptBetween3>() as u64
}

#[no_mangle]
pub extern "C" fn lt_pose_graph_opt_abi_sizeof_config() -> u64 {
    core::mem::size_of::<LtPoseGraphOptConfig>() as u64
}

#[no_mangle]
pub extern "C" fn lt_pose_graph_opt_abi_sizeof_report() -> u64 {
    core::mem::size_of::<LtPoseGraphOptReport>() as u64
}

#[no_mangle]
/// # Safety
///
/// `config` may be null. When non-null, it must point to a readable
/// `LtPoseGraphOptConfig` for the duration of the call.
pub unsafe extern "C" fn lt_pose_graph_opt_create(
    config: *const LtPoseGraphOptConfig,
) -> *mut LtPoseGraphOptWorkspace {
    let config = if config.is_null() {
        OptimizerConfig::default()
    } else {
        match OptimizerConfig::try_from(*config)
            .and_then(|config| config.normalized().map_err(|err| err.code()))
        {
            Ok(value) => value,
            Err(_) => return core::ptr::null_mut(),
        }
    };
    Box::into_raw(Box::new(LtPoseGraphOptWorkspace {
        config,
        last_error: LT_POSE_GRAPH_OPT_OK,
        result_poses: Vec::new(),
    }))
}

#[no_mangle]
/// # Safety
///
/// `handle` may be null. When non-null, it must be a pointer returned by
/// `lt_pose_graph_opt_create` that has not already been destroyed.
pub unsafe extern "C" fn lt_pose_graph_opt_destroy(handle: *mut LtPoseGraphOptWorkspace) {
    if !handle.is_null() {
        drop(Box::from_raw(handle));
    }
}

#[no_mangle]
/// # Safety
///
/// `handle` must be a valid pointer returned by `lt_pose_graph_opt_create`.
pub unsafe extern "C" fn lt_pose_graph_opt_reset(handle: *mut LtPoseGraphOptWorkspace) -> c_int {
    if handle.is_null() {
        return LT_POSE_GRAPH_OPT_NULL_POINTER;
    }
    (*handle).last_error = LT_POSE_GRAPH_OPT_OK;
    (*handle).result_poses.clear();
    LT_POSE_GRAPH_OPT_OK
}

#[no_mangle]
/// # Safety
///
/// `handle` must be a valid workspace pointer. `config` must point to a readable
/// `LtPoseGraphOptConfig` for the duration of the call.
pub unsafe extern "C" fn lt_pose_graph_opt_configure(
    handle: *mut LtPoseGraphOptWorkspace,
    config: *const LtPoseGraphOptConfig,
) -> c_int {
    if handle.is_null() || config.is_null() {
        return LT_POSE_GRAPH_OPT_NULL_POINTER;
    }
    match OptimizerConfig::try_from(*config)
        .and_then(|config| config.normalized().map_err(|err| err.code()))
    {
        Ok(value) => {
            (*handle).config = value;
            (*handle).last_error = LT_POSE_GRAPH_OPT_OK;
            LT_POSE_GRAPH_OPT_OK
        }
        Err(status) => {
            (*handle).last_error = status;
            status
        }
    }
}

#[no_mangle]
/// # Safety
///
/// `handle` and `out_report` must be valid pointers. `poses` must point to
/// `pose_count` readable poses. `priors` and `betweens` may be null only when
/// their counts are zero; otherwise they must point to readable arrays of the
/// corresponding lengths.
pub unsafe extern "C" fn lt_pose_graph_opt_process_se3(
    handle: *mut LtPoseGraphOptWorkspace,
    poses: *const LtPoseGraphOptPose3,
    pose_count: u64,
    priors: *const LtPoseGraphOptPrior3,
    prior_count: u64,
    betweens: *const LtPoseGraphOptBetween3,
    between_count: u64,
    out_report: *mut LtPoseGraphOptReport,
) -> c_int {
    if handle.is_null() || poses.is_null() || out_report.is_null() {
        return LT_POSE_GRAPH_OPT_NULL_POINTER;
    }
    if pose_count == 0 {
        (*handle).last_error = LT_POSE_GRAPH_OPT_EMPTY_GRAPH;
        *out_report = report_with_status(LT_POSE_GRAPH_OPT_EMPTY_GRAPH);
        return LT_POSE_GRAPH_OPT_EMPTY_GRAPH;
    }
    if (prior_count > 0 && priors.is_null()) || (between_count > 0 && betweens.is_null()) {
        (*handle).last_error = LT_POSE_GRAPH_OPT_NULL_POINTER;
        *out_report = report_with_status(LT_POSE_GRAPH_OPT_NULL_POINTER);
        return LT_POSE_GRAPH_OPT_NULL_POINTER;
    }

    let pose_slice = core::slice::from_raw_parts(poses, pose_count as usize);
    let prior_slice = if prior_count == 0 {
        &[]
    } else {
        core::slice::from_raw_parts(priors, prior_count as usize)
    };
    let between_slice = if between_count == 0 {
        &[]
    } else {
        core::slice::from_raw_parts(betweens, between_count as usize)
    };

    match graph_from_ffi(pose_slice, prior_slice, between_slice).and_then(|mut graph| {
        optimize_pose_graph3(&mut graph, (*handle).config).map(|report| (graph, report))
    }) {
        Ok((graph, report)) => {
            (*handle).result_poses = graph.poses.iter().copied().map(Into::into).collect();
            (*handle).last_error = LT_POSE_GRAPH_OPT_OK;
            *out_report = LtPoseGraphOptReport::from(report);
            LT_POSE_GRAPH_OPT_OK
        }
        Err(err) => {
            let status = err.code();
            (*handle).last_error = status;
            *out_report = report_with_status(status);
            status
        }
    }
}

#[no_mangle]
/// # Safety
///
/// `handle` must be a valid workspace pointer. `poses` must point to writable
/// storage for `capacity` poses, and `written` must be writable.
pub unsafe extern "C" fn lt_pose_graph_opt_copy_result_poses(
    handle: *const LtPoseGraphOptWorkspace,
    poses: *mut LtPoseGraphOptPose3,
    capacity: u64,
    written: *mut u64,
) -> c_int {
    if handle.is_null() || poses.is_null() || written.is_null() {
        return LT_POSE_GRAPH_OPT_NULL_POINTER;
    }
    let result = &(*handle).result_poses;
    if capacity < result.len() as u64 {
        *written = result.len() as u64;
        return LT_POSE_GRAPH_OPT_BUFFER_TOO_SMALL;
    }
    let output = core::slice::from_raw_parts_mut(poses, result.len());
    output.copy_from_slice(result);
    *written = result.len() as u64;
    LT_POSE_GRAPH_OPT_OK
}

#[no_mangle]
/// # Safety
///
/// `handle` must be a valid workspace pointer returned by
/// `lt_pose_graph_opt_create`.
pub unsafe extern "C" fn lt_pose_graph_opt_last_error(
    handle: *const LtPoseGraphOptWorkspace,
) -> c_int {
    if handle.is_null() {
        return LT_POSE_GRAPH_OPT_NULL_POINTER;
    }
    (*handle).last_error
}

fn graph_from_ffi(
    poses: &[LtPoseGraphOptPose3],
    priors: &[LtPoseGraphOptPrior3],
    betweens: &[LtPoseGraphOptBetween3],
) -> Result<PoseGraph3, crate::solver::OptimizeError> {
    let mut graph = PoseGraph3::new(
        poses
            .iter()
            .copied()
            .map(Pose3::try_from)
            .collect::<Result<Vec<_>, _>>()
            .map_err(|_| crate::solver::OptimizeError::NonFiniteInput)?,
    );
    for prior in priors {
        graph.priors.push(PriorFactor3 {
            index: prior.index as usize,
            pose: Pose3::try_from(prior.pose)
                .map_err(|_| crate::solver::OptimizeError::NonFiniteInput)?,
            information: information_from_upper(&prior.information_upper)?,
        });
    }
    for between in betweens {
        graph.betweens.push(BetweenFactor3 {
            from: between.from_index as usize,
            to: between.to_index as usize,
            measurement: Pose3::try_from(between.pose_from_to)
                .map_err(|_| crate::solver::OptimizeError::NonFiniteInput)?,
            information: information_from_upper(&between.information_upper)?,
        });
    }
    Ok(graph)
}

fn report_with_status(status: c_int) -> LtPoseGraphOptReport {
    LtPoseGraphOptReport {
        status,
        ..LtPoseGraphOptReport::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Vector6;
    use crate::solver::{diagonal_information, information_to_upper};

    #[test]
    fn c_abi_solves_se3_graph_and_copies_results() {
        let poses = [
            LtPoseGraphOptPose3::from(Pose3::identity()),
            LtPoseGraphOptPose3::from(Pose3::exp(&Vector6::from_row_slice(&[
                0.0, 0.0, 0.1, 1.3, 0.2, 0.0,
            ]))),
        ];
        let prior = [LtPoseGraphOptPrior3 {
            index: 0,
            reserved0: 0,
            pose: LtPoseGraphOptPose3::from(Pose3::identity()),
            information_upper: information_to_upper(&diagonal_information([
                100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
            ])),
        }];
        let between = [LtPoseGraphOptBetween3 {
            from_index: 0,
            to_index: 1,
            pose_from_to: LtPoseGraphOptPose3::from(Pose3::exp(&Vector6::from_row_slice(&[
                0.0, 0.0, 0.1, 1.0, 0.0, 0.0,
            ]))),
            information_upper: information_to_upper(&diagonal_information([
                10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
            ])),
        }];
        let mut report = LtPoseGraphOptReport::default();

        let handle = unsafe { lt_pose_graph_opt_create(core::ptr::null()) };
        assert!(!handle.is_null());
        let status = unsafe {
            lt_pose_graph_opt_process_se3(
                handle,
                poses.as_ptr(),
                poses.len() as u64,
                prior.as_ptr(),
                prior.len() as u64,
                between.as_ptr(),
                between.len() as u64,
                &mut report,
            )
        };
        assert_eq!(status, LT_POSE_GRAPH_OPT_OK);
        assert!(report.final_cost < report.initial_cost);
        let mut result = [LtPoseGraphOptPose3 {
            t_xyz: [0.0; 3],
            q_wxyz: [0.0; 4],
        }; 2];
        let mut written = 0u64;
        let copy_status = unsafe {
            lt_pose_graph_opt_copy_result_poses(
                handle,
                result.as_mut_ptr(),
                result.len() as u64,
                &mut written,
            )
        };
        assert_eq!(copy_status, LT_POSE_GRAPH_OPT_OK);
        assert_eq!(written, 2);
        let optimized = Pose3::try_from(result[1]).unwrap();
        let target = Pose3::try_from(between[0].pose_from_to).unwrap();
        let residual = target.inverse().compose(optimized).log();
        for idx in 0..6 {
            assert!(residual[idx].abs() < 1e-5);
        }
        unsafe { lt_pose_graph_opt_destroy(handle) };
    }

    #[test]
    fn c_abi_rejects_bad_quaternion() {
        let poses = [LtPoseGraphOptPose3 {
            t_xyz: [0.0, 0.0, 0.0],
            q_wxyz: [0.0, 0.0, 0.0, 0.0],
        }];
        let mut report = LtPoseGraphOptReport::default();
        let handle = unsafe { lt_pose_graph_opt_create(core::ptr::null()) };
        let status = unsafe {
            lt_pose_graph_opt_process_se3(
                handle,
                poses.as_ptr(),
                1,
                core::ptr::null(),
                0,
                core::ptr::null(),
                0,
                &mut report,
            )
        };
        assert_eq!(status, LT_POSE_GRAPH_OPT_NON_FINITE_INPUT);
        unsafe { lt_pose_graph_opt_destroy(handle) };
    }
}
