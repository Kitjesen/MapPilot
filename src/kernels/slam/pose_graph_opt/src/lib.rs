//! Portable SE(3) pose graph optimization kernel for LingTu.
//!
//! This crate replaces the GTSAM subset currently used by LingTu PGO/HBA:
//! Pose3 priors, Pose3 between factors, and batch Gauss-Newton/LM smoothing.
//! It has no ROS, PCL, Eigen, GTSAM, or Python dependency at the kernel layer.

pub mod ffi;
pub mod geometry;
pub mod solver;

pub use ffi::*;
pub use geometry::{Pose3, Vector6};
pub use solver::{
    BetweenFactor3, Information6, OptimizeError, OptimizerConfig, OptimizerReport, PoseGraph3,
    PriorFactor3,
};

pub const LT_POSE_GRAPH_OPT_ABI_VERSION: u32 = 2;

pub const LT_POSE_GRAPH_OPT_OK: i32 = 0;
pub const LT_POSE_GRAPH_OPT_NULL_POINTER: i32 = -1;
pub const LT_POSE_GRAPH_OPT_EMPTY_GRAPH: i32 = -2;
pub const LT_POSE_GRAPH_OPT_INVALID_INDEX: i32 = -3;
pub const LT_POSE_GRAPH_OPT_NON_FINITE_INPUT: i32 = -4;
pub const LT_POSE_GRAPH_OPT_SINGULAR_SYSTEM: i32 = -5;
pub const LT_POSE_GRAPH_OPT_BUFFER_TOO_SMALL: i32 = -6;
pub const LT_POSE_GRAPH_OPT_INVALID_CONFIG: i32 = -7;
pub const LT_POSE_GRAPH_OPT_GAUGE_FREEDOM: i32 = -8;
pub const LT_POSE_GRAPH_OPT_INVALID_INFORMATION: i32 = -9;
