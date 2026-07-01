//! Portable math kernel for LingTu's PCT/GPMP trajectory optimizer.
//!
//! This crate intentionally starts with the GTSAM-free parts of the existing
//! C++ optimizer:
//! - WNOJ/WNOA process models (`Q`, `Q^-1`, `Phi`)
//! - GP interpolation (`Lambda`, `Psi`)
//! - prior residuals (`Phi * x1 - x2`)
//! - WNOJ heading-rate residual
//! - DenseElevationMap obstacle residuals
//! - dense, sparse Cholesky, and block-tridiagonal LM/GN batch optimizers for
//!   WNOJ/WNOA trajectory graphs

use std::fmt;
use std::sync::Arc;
use std::time::Instant;

use faer::prelude::Solve;
use faer::sparse::{SparseColMat, Triplet};
use faer::{Mat, Side};
use nalgebra::{DMatrix, DVector, SMatrix, SVector};
use serde::{Deserialize, Serialize};

pub type Vec4 = SVector<f64, 4>;
pub type Vec6 = SVector<f64, 6>;
pub type Mat4 = SMatrix<f64, 4, 4>;
pub type Mat6 = SMatrix<f64, 6, 6>;
pub type Row4 = SMatrix<f64, 1, 4>;
pub type Row6 = SMatrix<f64, 1, 6>;

const SAFE_COST_THRESHOLD: f64 = 10.0;
const FULL_STEP_SCALES: [f64; 1] = [1.0];
const BACKTRACKING_STEP_SCALES: [f64; 6] = [1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125];
const SPARSE_LM_MIN_DIM: usize = 48;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LinearSolverKind {
    Auto,
    Dense,
    Sparse,
    BlockTridiagonal,
}

impl Default for LinearSolverKind {
    fn default() -> Self {
        Self::Auto
    }
}

impl LinearSolverKind {
    fn as_report_str(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Dense => "dense",
            Self::Sparse => "sparse_cholesky",
            Self::BlockTridiagonal => "block_tridiagonal",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NonlinearOptimizerKind {
    LevenbergMarquardt,
    GaussNewton,
}

impl Default for NonlinearOptimizerKind {
    fn default() -> Self {
        Self::LevenbergMarquardt
    }
}

impl NonlinearOptimizerKind {
    fn as_report_str(self) -> &'static str {
        match self {
            Self::LevenbergMarquardt => "levenberg_marquardt",
            Self::GaussNewton => "gauss_newton",
        }
    }

    fn damping(self, lambda: f64) -> f64 {
        match self {
            Self::LevenbergMarquardt => lambda,
            Self::GaussNewton => 0.0,
        }
    }
}

fn step_scales(kind: NonlinearOptimizerKind) -> &'static [f64] {
    match kind {
        NonlinearOptimizerKind::LevenbergMarquardt => &FULL_STEP_SCALES,
        NonlinearOptimizerKind::GaussNewton => &BACKTRACKING_STEP_SCALES,
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum GpmpError {
    EmptyStateVector,
    InvalidIndex,
    InvalidMapDimensions,
    SingularSystem,
}

impl fmt::Display for GpmpError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyStateVector => formatter.write_str("empty state vector"),
            Self::InvalidIndex => formatter.write_str("invalid state index"),
            Self::InvalidMapDimensions => formatter.write_str("invalid map dimensions"),
            Self::SingularSystem => formatter.write_str("singular linear system"),
        }
    }
}

impl std::error::Error for GpmpError {}

#[derive(Debug, Clone)]
pub struct DenseElevationMap {
    resolution: f64,
    max_layers: usize,
    max_x: usize,
    max_y: usize,
    cost: Vec<f64>,
    height: Vec<f64>,
    ceiling: Vec<f64>,
}

impl DenseElevationMap {
    pub fn new(
        resolution: f64,
        num_layers: usize,
        max_x: usize,
        max_y: usize,
        cost: Vec<f64>,
        height: Vec<f64>,
        ceiling: Vec<f64>,
    ) -> Result<Self, GpmpError> {
        let cells = num_layers
            .checked_mul(max_x)
            .and_then(|value| value.checked_mul(max_y))
            .ok_or(GpmpError::InvalidMapDimensions)?;
        if !resolution.is_finite()
            || resolution <= 0.0
            || num_layers == 0
            || max_x == 0
            || max_y == 0
            || cost.len() != cells
            || height.len() != cells
            || ceiling.len() != cells
        {
            return Err(GpmpError::InvalidMapDimensions);
        }
        Ok(Self {
            resolution,
            max_layers: num_layers,
            max_x,
            max_y,
            cost,
            height,
            ceiling,
        })
    }

    pub fn constant(
        resolution: f64,
        num_layers: usize,
        max_x: usize,
        max_y: usize,
        cost: f64,
        height: f64,
        ceiling: f64,
    ) -> Result<Self, GpmpError> {
        let cells = num_layers
            .checked_mul(max_x)
            .and_then(|value| value.checked_mul(max_y))
            .ok_or(GpmpError::InvalidMapDimensions)?;
        Self::new(
            resolution,
            num_layers,
            max_x,
            max_y,
            vec![cost; cells],
            vec![height; cells],
            vec![ceiling; cells],
        )
    }

    fn index_coord(&self, coord: f64, max_value: usize) -> usize {
        if max_value == 0 || !coord.is_finite() {
            return 0;
        }
        let raw = coord as isize;
        raw.clamp(0, max_value as isize - 1) as usize
    }

    fn layer_safe(&self, layer: i32) -> usize {
        if self.max_layers == 0 {
            return 0;
        }
        layer.clamp(0, self.max_layers as i32 - 1) as usize
    }

    fn row_col(&self, layer: i32, x: f64, y: f64) -> (usize, usize) {
        let safe_layer = self.layer_safe(layer);
        let col = self.index_coord(x, self.max_x);
        let row = self.index_coord(y, self.max_y) + safe_layer * self.max_y;
        (row, col)
    }

    fn value_at_row_col(values: &[f64], max_x: usize, row: usize, col: usize) -> f64 {
        values[row * max_x + col]
    }

    pub fn get_height(&self, layer: i32, x: f64, y: f64) -> f64 {
        let (row, col) = self.row_col(layer, x, y);
        Self::value_at_row_col(&self.height, self.max_x, row, col)
    }

    pub fn get_height_safe(&self, layer: i32, x: f64, y: f64, height_hint: f64) -> f64 {
        let new_height = self.get_height(layer, x, y);
        if (new_height - height_hint).abs() > 6.0 * self.resolution {
            height_hint
        } else {
            new_height
        }
    }

    pub fn get_ceiling(&self, layer: i32, x: f64, y: f64) -> f64 {
        let (row, col) = self.row_col(layer, x, y);
        Self::value_at_row_col(&self.ceiling, self.max_x, row, col)
    }

    pub fn update_layer_safe(&self, layer: i32, x: f64, y: f64, height_hint: f64) -> i32 {
        let safe_layer = self.layer_safe(layer);
        let (row, col) = self.row_col(safe_layer as i32, x, y);
        let cost = Self::value_at_row_col(&self.cost, self.max_x, row, col);
        let this_height = self.get_height(safe_layer as i32, x, y);
        let unsafe_grid =
            this_height < -50.0 || (this_height - height_hint).abs() > 5.0 * self.resolution;

        if !unsafe_grid && cost < SAFE_COST_THRESHOLD {
            return safe_layer as i32;
        }

        let mut real_layer = safe_layer;
        let mut min_cost = if unsafe_grid { 1000.0 } else { cost };

        if safe_layer > 0 {
            let lower_row = row - self.max_y;
            let lower_height = Self::value_at_row_col(&self.height, self.max_x, lower_row, col);
            let lower_cost = Self::value_at_row_col(&self.cost, self.max_x, lower_row, col);
            if (height_hint - lower_height).abs() < 5.0 * self.resolution
                && (((this_height - lower_height).abs() < 1.5 * self.resolution
                    && lower_cost < cost)
                    || (unsafe_grid && lower_cost < 2.0 * SAFE_COST_THRESHOLD))
            {
                real_layer = safe_layer - 1;
                min_cost = lower_cost;
            }
        }

        if safe_layer + 1 < self.max_layers {
            let upper_row = row + self.max_y;
            let upper_height = Self::value_at_row_col(&self.height, self.max_x, upper_row, col);
            let upper_cost = Self::value_at_row_col(&self.cost, self.max_x, upper_row, col);
            if (height_hint - upper_height).abs() < 5.0 * self.resolution
                && (((this_height - upper_height).abs() < 1.5 * self.resolution
                    && upper_cost < cost)
                    || (unsafe_grid && upper_cost < 2.0 * SAFE_COST_THRESHOLD))
                && upper_cost < min_cost
            {
                real_layer = safe_layer + 1;
            }
        }

        real_layer as i32
    }

    fn real_cost_safe(&self, layer: i32, x: f64, y: f64, height_hint: f64) -> f64 {
        let real_layer = self.update_layer_safe(layer, x, y, height_hint);
        let (row, col) = self.row_col(real_layer, x, y);
        Self::value_at_row_col(&self.cost, self.max_x, row, col)
    }

    pub fn value_bilinear_safe(
        &self,
        layer: i32,
        x: f64,
        y: f64,
        height_hint: f64,
    ) -> (f64, [f64; 2]) {
        let x_safe = if x.is_finite() { x } else { 0.0 };
        let y_safe = if y.is_finite() { y } else { 0.0 };
        let x_lb = (x_safe - 0.5).floor().max(0.0);
        let y_lb = (y_safe - 0.5).floor().max(0.0);
        let value00 = self.real_cost_safe(layer, x_lb, y_lb, height_hint);
        let value10 = self.real_cost_safe(layer, x_lb + 1.0, y_lb, height_hint);
        let value01 = self.real_cost_safe(layer, x_lb, y_lb + 1.0, height_hint);
        let value11 = self.real_cost_safe(layer, x_lb + 1.0, y_lb + 1.0, height_hint);

        let dx = x_safe - x_lb;
        let dy = y_safe - y_lb;
        let y0 = (1.0 - dx) * value00 + dx * value10;
        let y1 = (1.0 - dx) * value01 + dx * value11;
        let x0 = (1.0 - dy) * value00 + dy * value01;
        let x1 = (1.0 - dy) * value10 + dy * value11;
        ((1.0 - dy) * y0 + dy * y1, [x1 - x0, y1 - y0])
    }
}

/// WNOJ state order used by the C++ GPMP optimizer:
/// `[x, dx, ddx, y, dy, ddy]`.
pub mod wnoj {
    use super::{DenseElevationMap, Mat6, Row6, Vec6};

    #[derive(Debug, Clone)]
    pub struct ObstacleEvaluation {
        pub residual: f64,
        pub jacobian: Row6,
        pub cost: f64,
        pub updated_layer: i32,
        pub updated_height_hint: f64,
    }

    pub fn q(qc: f64, tau: f64) -> Mat6 {
        let mut out = Mat6::zeros();
        let mut powers = [0.0; 5];
        powers[0] = tau * qc;
        for index in 1..5 {
            powers[index] = tau * powers[index - 1];
        }

        let block = [
            [0.05 * powers[4], 0.125 * powers[3], (1.0 / 6.0) * powers[2]],
            [0.125 * powers[3], (1.0 / 3.0) * powers[2], 0.5 * powers[1]],
            [(1.0 / 6.0) * powers[2], 0.5 * powers[1], powers[0]],
        ];
        for row in 0..3 {
            for col in 0..3 {
                out[(row, col)] = block[row][col];
                out[(row + 3, col + 3)] = block[row][col];
            }
        }
        out
    }

    pub fn phi(tau: f64) -> Mat6 {
        let mut out = Mat6::identity();
        out[(0, 1)] = tau;
        out[(0, 2)] = 0.5 * tau * tau;
        out[(1, 2)] = tau;
        out[(3, 4)] = tau;
        out[(3, 5)] = 0.5 * tau * tau;
        out[(4, 5)] = tau;
        out
    }

    pub fn q_inverse(qc: f64, tau: f64) -> Mat6 {
        let mut out = Mat6::zeros();
        let mut powers = [0.0; 5];
        let tau_inv = 1.0 / tau;
        powers[0] = tau_inv / qc;
        for index in 1..5 {
            powers[index] = tau_inv * powers[index - 1];
        }

        let block = [
            [720.0 * powers[4], -360.0 * powers[3], 60.0 * powers[2]],
            [-360.0 * powers[3], 192.0 * powers[2], -36.0 * powers[1]],
            [60.0 * powers[2], -36.0 * powers[1], 9.0 * powers[0]],
        ];
        for row in 0..3 {
            for col in 0..3 {
                out[(row, col)] = block[row][col];
                out[(row + 3, col + 3)] = block[row][col];
            }
        }
        out
    }

    pub fn lambda_psi(qc: f64, delta: f64, tau: f64) -> (Mat6, Mat6) {
        let psi = q(qc, tau) * phi(delta - tau).transpose() * q_inverse(qc, delta);
        let lambda = phi(tau) - psi * phi(delta);
        (lambda, psi)
    }

    pub fn prior_residual(x1: &Vec6, x2: &Vec6, delta: f64) -> Vec6 {
        phi(delta) * x1 - x2
    }

    pub fn state_from_path_point(x: f64, y: f64, heading: f64, reference_velocity: f64) -> Vec6 {
        let velocity = reference_velocity.max(1.0);
        Vec6::from_row_slice(&[
            x,
            heading.cos() * velocity,
            0.0,
            y,
            heading.sin() * velocity,
            0.0,
        ])
    }

    pub fn interpolate(x1: &Vec6, x2: &Vec6, qc: f64, delta: f64, tau: f64) -> Vec6 {
        let (lambda, psi) = lambda_psi(qc, delta, tau);
        lambda * x1 + psi * x2
    }

    pub fn heading_rate(state: &Vec6) -> f64 {
        let dx = state[1];
        let ddx = state[2];
        let dy = state[4];
        let ddy = state[5];
        (ddy * dx - dy * ddx) / (dx * dx + dy * dy + 1e-6)
    }

    pub fn heading_rate_residual(state: &Vec6, max_heading_rate: f64) -> f64 {
        let rate = heading_rate(state);
        if rate > max_heading_rate {
            rate - max_heading_rate
        } else if rate < -max_heading_rate {
            -max_heading_rate - rate
        } else {
            0.0
        }
    }

    pub fn heading_rate_jacobian(state: &Vec6, max_heading_rate: f64) -> Row6 {
        let dx = state[1];
        let ddx = state[2];
        let dy = state[4];
        let ddy = state[5];
        let denom = dx * dx + dy * dy + 1e-6;
        let rate = heading_rate(state);
        if rate.abs() <= max_heading_rate {
            return Row6::zeros();
        }

        let mut jac = Row6::zeros();
        jac[(0, 1)] = (ddy * dy * dy - ddy * dx * dx + 2.0 * dx * dy * ddx) / (denom * denom);
        jac[(0, 2)] = -dy / denom;
        jac[(0, 4)] = (-ddx * dx * dx + ddx * dy * dy - 2.0 * dx * dy * ddy) / (denom * denom);
        jac[(0, 5)] = dx / denom;

        if rate < -max_heading_rate {
            -jac
        } else {
            jac
        }
    }

    pub fn interpolate_heading_rate_residual(
        x1: &Vec6,
        x2: &Vec6,
        qc: f64,
        delta: f64,
        tau: f64,
        max_heading_rate: f64,
    ) -> f64 {
        let state = interpolate(x1, x2, qc, delta, tau);
        heading_rate_residual(&state, max_heading_rate)
    }

    pub fn interpolate_heading_rate_residual_jacobians(
        x1: &Vec6,
        x2: &Vec6,
        qc: f64,
        delta: f64,
        tau: f64,
        max_heading_rate: f64,
    ) -> (f64, Row6, Row6) {
        let (lambda, psi) = lambda_psi(qc, delta, tau);
        let state = lambda * x1 + psi * x2;
        let residual = heading_rate_residual(&state, max_heading_rate);
        let jacobian = heading_rate_jacobian(&state, max_heading_rate);
        (residual, jacobian * lambda, jacobian * psi)
    }

    pub fn obstacle_residual_jacobian(
        map: &DenseElevationMap,
        state: &Vec6,
        current_layer: i32,
        height_hint: f64,
        cost_threshold: f64,
    ) -> ObstacleEvaluation {
        let (cost, grad) = map.value_bilinear_safe(current_layer, state[0], state[3], height_hint);
        let updated_layer = map.update_layer_safe(current_layer, state[0], state[3], height_hint);
        let updated_height_hint =
            map.get_height_safe(updated_layer, state[0], state[3], height_hint);
        let mut jacobian = Row6::zeros();
        let mut residual = 0.0;
        if cost > cost_threshold {
            let violation = cost - cost_threshold;
            residual = violation * violation;
            jacobian[(0, 0)] = 2.0 * violation * grad[0];
            jacobian[(0, 3)] = 2.0 * violation * grad[1];
        }
        ObstacleEvaluation {
            residual,
            jacobian,
            cost,
            updated_layer,
            updated_height_hint,
        }
    }

    pub fn interpolate_obstacle_residual_jacobians(
        map: &DenseElevationMap,
        x1: &Vec6,
        x2: &Vec6,
        qc: f64,
        delta: f64,
        tau: f64,
        current_layer: i32,
        height_hint: f64,
        cost_threshold: f64,
    ) -> (ObstacleEvaluation, Row6, Row6) {
        let (lambda, psi) = lambda_psi(qc, delta, tau);
        let state = lambda * x1 + psi * x2;
        let evaluation =
            obstacle_residual_jacobian(map, &state, current_layer, height_hint, cost_threshold);
        let jac_x1 = evaluation.jacobian * lambda;
        let jac_x2 = evaluation.jacobian * psi;
        (evaluation, jac_x1, jac_x2)
    }
}

/// WNOA state order used by the C++ GPMP optimizer:
/// `[x, dx, y, dy]`.
pub mod wnoa {
    use super::{DenseElevationMap, Mat4, Row4, Vec4};

    #[derive(Debug, Clone)]
    pub struct ObstacleEvaluation {
        pub residual: f64,
        pub jacobian: Row4,
        pub cost: f64,
        pub updated_layer: i32,
        pub updated_height_hint: f64,
    }

    pub fn q(qc: f64, tau: f64) -> Mat4 {
        let mut out = Mat4::zeros();
        let q00 = (1.0 / 3.0) * tau.powi(3) * qc;
        let q01 = 0.5 * tau.powi(2) * qc;
        let q11 = tau * qc;

        out[(0, 0)] = q00;
        out[(0, 1)] = q01;
        out[(1, 0)] = q01;
        out[(1, 1)] = q11;
        out[(2, 2)] = q00;
        out[(2, 3)] = q01;
        out[(3, 2)] = q01;
        out[(3, 3)] = q11;
        out
    }

    pub fn phi(tau: f64) -> Mat4 {
        let mut out = Mat4::identity();
        out[(0, 1)] = tau;
        out[(2, 3)] = tau;
        out
    }

    pub fn q_inverse(qc: f64, tau: f64) -> Mat4 {
        let mut out = Mat4::zeros();
        let qc_inv = 1.0 / qc;
        let q00 = 12.0 * tau.powi(-3) * qc_inv;
        let q01 = -6.0 * tau.powi(-2) * qc_inv;
        let q11 = 4.0 / tau * qc_inv;

        out[(0, 0)] = q00;
        out[(0, 1)] = q01;
        out[(1, 0)] = q01;
        out[(1, 1)] = q11;
        out[(2, 2)] = q00;
        out[(2, 3)] = q01;
        out[(3, 2)] = q01;
        out[(3, 3)] = q11;
        out
    }

    pub fn lambda_psi(qc: f64, delta: f64, tau: f64) -> (Mat4, Mat4) {
        let psi = q(qc, tau) * phi(delta - tau).transpose() * q_inverse(qc, delta);
        let lambda = phi(tau) - psi * phi(delta);
        (lambda, psi)
    }

    pub fn prior_residual(x1: &Vec4, x2: &Vec4, delta: f64) -> Vec4 {
        phi(delta) * x1 - x2
    }

    pub fn state_from_path_point(x: f64, y: f64, heading: f64, reference_velocity: f64) -> Vec4 {
        let velocity = reference_velocity.max(1.0);
        Vec4::from_row_slice(&[x, heading.cos() * velocity, y, heading.sin() * velocity])
    }

    pub fn interpolate(x1: &Vec4, x2: &Vec4, qc: f64, delta: f64, tau: f64) -> Vec4 {
        let (lambda, psi) = lambda_psi(qc, delta, tau);
        lambda * x1 + psi * x2
    }

    pub fn obstacle_residual_jacobian(
        map: &DenseElevationMap,
        state: &Vec4,
        current_layer: i32,
        height_hint: f64,
        cost_threshold: f64,
    ) -> ObstacleEvaluation {
        let (cost, grad) = map.value_bilinear_safe(current_layer, state[0], state[2], height_hint);
        let updated_layer = map.update_layer_safe(current_layer, state[0], state[2], height_hint);
        let updated_height_hint =
            map.get_height_safe(updated_layer, state[0], state[2], height_hint);
        let mut jacobian = Row4::zeros();
        let mut residual = 0.0;
        if cost > cost_threshold {
            let violation = cost - cost_threshold;
            residual = violation * violation;
            jacobian[(0, 0)] = 2.0 * violation * grad[0];
            jacobian[(0, 2)] = 2.0 * violation * grad[1];
        }
        ObstacleEvaluation {
            residual,
            jacobian,
            cost,
            updated_layer,
            updated_height_hint,
        }
    }

    pub fn interpolate_obstacle_residual_jacobians(
        map: &DenseElevationMap,
        x1: &Vec4,
        x2: &Vec4,
        qc: f64,
        delta: f64,
        tau: f64,
        current_layer: i32,
        height_hint: f64,
        cost_threshold: f64,
    ) -> (ObstacleEvaluation, Row4, Row4) {
        let (lambda, psi) = lambda_psi(qc, delta, tau);
        let state = lambda * x1 + psi * x2;
        let evaluation =
            obstacle_residual_jacobian(map, &state, current_layer, height_hint, cost_threshold);
        let jac_x1 = evaluation.jacobian * lambda;
        let jac_x2 = evaluation.jacobian * psi;
        (evaluation, jac_x1, jac_x2)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WnojOptimizerConfig {
    pub max_iterations: usize,
    pub initial_lambda: f64,
    pub lambda_up: f64,
    pub lambda_down: f64,
    pub gradient_tolerance: f64,
    pub step_tolerance: f64,
    pub cost_tolerance: f64,
    #[serde(default)]
    pub linear_solver: LinearSolverKind,
    #[serde(default)]
    pub nonlinear_optimizer: NonlinearOptimizerKind,
}

impl Default for WnojOptimizerConfig {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            initial_lambda: 200.0,
            lambda_up: 10.0,
            lambda_down: 0.3,
            gradient_tolerance: 1e-8,
            step_tolerance: 1e-8,
            cost_tolerance: 1e-9,
            linear_solver: LinearSolverKind::Auto,
            nonlinear_optimizer: NonlinearOptimizerKind::LevenbergMarquardt,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WnojOptimizeReport {
    pub initial_cost: f64,
    pub final_cost: f64,
    pub iterations: usize,
    pub accepted_steps: usize,
    pub nonlinear_optimizer: String,
    pub linear_solver: String,
    pub linear_solve_fallbacks: usize,
    pub elapsed_ms: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DenseElevationMapRequest {
    pub resolution: f64,
    pub num_layers: usize,
    pub max_x: usize,
    pub max_y: usize,
    pub cost: Vec<f64>,
    pub height: Vec<f64>,
    pub ceiling: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WnojOptimizeRequest {
    pub mode: Option<String>,
    pub schema: Option<String>,
    pub states: Vec<[f64; 6]>,
    pub layers: Option<Vec<i32>>,
    pub height_hints: Option<Vec<f64>>,
    pub map: Option<DenseElevationMapRequest>,
    pub cost_threshold: Option<f64>,
    pub endpoint_prior_sigmas: Option<[f64; 6]>,
    pub gp_qc: Option<f64>,
    pub delta: Option<f64>,
    pub obstacle_sigma: Option<f64>,
    pub heading_rate_sigma: Option<f64>,
    pub max_heading_rate: Option<f64>,
    pub interpolation_steps: Option<usize>,
    pub config: Option<WnojOptimizerConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WnojOptimizeResponse {
    pub schema: String,
    pub ok: bool,
    pub error: Option<String>,
    pub states: Vec<[f64; 6]>,
    pub layers: Vec<i32>,
    pub heights: Vec<f64>,
    pub costs: Vec<f64>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub trajectory_states: Vec<[f64; 6]>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub trajectory_layers: Vec<i32>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub trajectory_heights: Vec<f64>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub trajectory_costs: Vec<f64>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub initial_trajectory_states: Vec<[f64; 6]>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub initial_trajectory_layers: Vec<i32>,
    pub report: Option<WnojOptimizeReport>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WnoaOptimizeRequest {
    pub mode: Option<String>,
    pub schema: Option<String>,
    pub states: Vec<[f64; 4]>,
    pub layers: Option<Vec<i32>>,
    pub height_hints: Option<Vec<f64>>,
    pub map: Option<DenseElevationMapRequest>,
    pub cost_threshold: Option<f64>,
    pub endpoint_prior_sigmas: Option<[f64; 4]>,
    pub gp_qc: Option<f64>,
    pub delta: Option<f64>,
    pub obstacle_sigma: Option<f64>,
    pub interpolation_steps: Option<usize>,
    pub config: Option<WnojOptimizerConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WnoaOptimizeResponse {
    pub schema: String,
    pub ok: bool,
    pub error: Option<String>,
    pub states: Vec<[f64; 4]>,
    pub layers: Vec<i32>,
    pub heights: Vec<f64>,
    pub costs: Vec<f64>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub trajectory_states: Vec<[f64; 4]>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub trajectory_layers: Vec<i32>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub trajectory_heights: Vec<f64>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub trajectory_costs: Vec<f64>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub initial_trajectory_states: Vec<[f64; 4]>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub initial_trajectory_layers: Vec<i32>,
    pub report: Option<WnojOptimizeReport>,
}

impl WnoaOptimizeResponse {
    pub fn error(message: impl Into<String>) -> Self {
        Self {
            schema: "lingtu.pct_gpmp.optimize.response.v1".to_string(),
            ok: false,
            error: Some(message.into()),
            states: Vec::new(),
            layers: Vec::new(),
            heights: Vec::new(),
            costs: Vec::new(),
            trajectory_states: Vec::new(),
            trajectory_layers: Vec::new(),
            trajectory_heights: Vec::new(),
            trajectory_costs: Vec::new(),
            initial_trajectory_states: Vec::new(),
            initial_trajectory_layers: Vec::new(),
            report: None,
        }
    }
}

impl WnojOptimizeResponse {
    pub fn error(message: impl Into<String>) -> Self {
        Self {
            schema: "lingtu.pct_gpmp.optimize.response.v1".to_string(),
            ok: false,
            error: Some(message.into()),
            states: Vec::new(),
            layers: Vec::new(),
            heights: Vec::new(),
            costs: Vec::new(),
            trajectory_states: Vec::new(),
            trajectory_layers: Vec::new(),
            trajectory_heights: Vec::new(),
            trajectory_costs: Vec::new(),
            initial_trajectory_states: Vec::new(),
            initial_trajectory_layers: Vec::new(),
            report: None,
        }
    }
}

pub const GPMP_OPTIMIZER_ABI_VERSION: u32 = 1;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JsonOptimizeOutcome {
    pub status_code: i32,
    pub response_json: String,
}

fn response_json<T: Serialize>(response: &T) -> String {
    serde_json::to_string(response).unwrap_or_else(|error| {
        format!(
            r#"{{"schema":"lingtu.pct_gpmp.optimize.response.v1","ok":false,"error":"failed to serialize optimizer response: {error}"}}"#
        )
    })
}

pub fn optimize_request_json(input: &str) -> JsonOptimizeOutcome {
    let value = match serde_json::from_str::<serde_json::Value>(input) {
        Ok(value) => value,
        Err(error) => {
            return JsonOptimizeOutcome {
                status_code: 2,
                response_json: response_json(&WnojOptimizeResponse::error(format!(
                    "invalid request JSON: {error}"
                ))),
            };
        }
    };
    let mode = value
        .get("mode")
        .and_then(|value| value.as_str())
        .unwrap_or("wnoj");

    if mode == "wnoa" {
        let request = match serde_json::from_value::<WnoaOptimizeRequest>(value) {
            Ok(request) => request,
            Err(error) => {
                return JsonOptimizeOutcome {
                    status_code: 2,
                    response_json: response_json(&WnoaOptimizeResponse::error(format!(
                        "invalid WNOA request JSON: {error}"
                    ))),
                };
            }
        };
        match optimize_wnoa_request(request) {
            Ok(response) => JsonOptimizeOutcome {
                status_code: 0,
                response_json: response_json(&response),
            },
            Err(error) => JsonOptimizeOutcome {
                status_code: 2,
                response_json: response_json(&WnoaOptimizeResponse::error(error.to_string())),
            },
        }
    } else {
        let request = match serde_json::from_value::<WnojOptimizeRequest>(value) {
            Ok(request) => request,
            Err(error) => {
                return JsonOptimizeOutcome {
                    status_code: 2,
                    response_json: response_json(&WnojOptimizeResponse::error(format!(
                        "invalid WNOJ request JSON: {error}"
                    ))),
                };
            }
        };
        match optimize_wnoj_request(request) {
            Ok(response) => JsonOptimizeOutcome {
                status_code: 0,
                response_json: response_json(&response),
            },
            Err(error) => JsonOptimizeOutcome {
                status_code: 2,
                response_json: response_json(&WnojOptimizeResponse::error(error.to_string())),
            },
        }
    }
}

#[no_mangle]
pub extern "C" fn lingtu_gpmp_optimizer_abi_version() -> u32 {
    GPMP_OPTIMIZER_ABI_VERSION
}

/// Optimize one GPMP JSON request through a stable C ABI.
///
/// The caller owns the returned buffer and must release it with
/// `lingtu_gpmp_optimizer_free_json`. The return code mirrors the process
/// binary: `0` for an accepted request and non-zero for a structured JSON error.
#[no_mangle]
pub unsafe extern "C" fn lingtu_gpmp_optimizer_optimize_json(
    input_ptr: *const u8,
    input_len: usize,
    output_ptr: *mut *mut u8,
    output_len: *mut usize,
) -> i32 {
    if input_ptr.is_null() || output_ptr.is_null() || output_len.is_null() {
        return -1;
    }
    *output_ptr = std::ptr::null_mut();
    *output_len = 0;

    let input = std::slice::from_raw_parts(input_ptr, input_len);
    let input = match std::str::from_utf8(input) {
        Ok(input) => input,
        Err(error) => {
            let response = response_json(&WnojOptimizeResponse::error(format!(
                "request JSON is not valid UTF-8: {error}"
            )));
            let mut bytes = response.into_bytes().into_boxed_slice();
            *output_len = bytes.len();
            *output_ptr = bytes.as_mut_ptr();
            std::mem::forget(bytes);
            return 2;
        }
    };

    let outcome = match std::panic::catch_unwind(|| optimize_request_json(input)) {
        Ok(outcome) => outcome,
        Err(_) => JsonOptimizeOutcome {
            status_code: 3,
            response_json: response_json(&WnojOptimizeResponse::error(
                "optimizer panicked while handling request",
            )),
        },
    };
    let status_code = outcome.status_code;
    let mut bytes = outcome.response_json.into_bytes().into_boxed_slice();
    *output_len = bytes.len();
    *output_ptr = bytes.as_mut_ptr();
    std::mem::forget(bytes);
    status_code
}

#[no_mangle]
pub unsafe extern "C" fn lingtu_gpmp_optimizer_free_json(ptr: *mut u8, len: usize) {
    if ptr.is_null() {
        return;
    }
    drop(Vec::from_raw_parts(ptr, len, len));
}

fn vec6_from_array(values: [f64; 6]) -> Vec6 {
    Vec6::from_row_slice(&values)
}

fn vec4_from_array(values: [f64; 4]) -> Vec4 {
    Vec4::from_row_slice(&values)
}

fn array_from_vec6(state: &Vec6) -> [f64; 6] {
    [state[0], state[1], state[2], state[3], state[4], state[5]]
}

fn array_from_vec4(state: &Vec4) -> [f64; 4] {
    [state[0], state[1], state[2], state[3]]
}

fn map_from_request(request: DenseElevationMapRequest) -> Result<DenseElevationMap, GpmpError> {
    DenseElevationMap::new(
        request.resolution,
        request.num_layers,
        request.max_x,
        request.max_y,
        request.cost,
        request.height,
        request.ceiling,
    )
}

fn wnoj_result_sample(
    state: &Vec6,
    map: Option<&DenseElevationMap>,
    layer_hint: i32,
    height_hint: f64,
) -> (i32, f64, f64) {
    if let Some(map) = map {
        let layer = map.update_layer_safe(layer_hint, state[0], state[3], height_hint);
        let height = map.get_height_safe(layer, state[0], state[3], height_hint);
        let cost = map
            .value_bilinear_safe(layer, state[0], state[3], height_hint)
            .0;
        (layer, height, cost)
    } else {
        (layer_hint, height_hint, 0.0)
    }
}

fn wnoa_result_sample(
    state: &Vec4,
    map: Option<&DenseElevationMap>,
    layer_hint: i32,
    height_hint: f64,
) -> (i32, f64, f64) {
    if let Some(map) = map {
        let layer = map.update_layer_safe(layer_hint, state[0], state[2], height_hint);
        let height = map.get_height_safe(layer, state[0], state[2], height_hint);
        let cost = map
            .value_bilinear_safe(layer, state[0], state[2], height_hint)
            .0;
        (layer, height, cost)
    } else {
        (layer_hint, height_hint, 0.0)
    }
}

#[derive(Debug, Clone)]
struct WnojTrajectoryOutput {
    states: Vec<[f64; 6]>,
    layers: Vec<i32>,
    heights: Vec<f64>,
    costs: Vec<f64>,
}

#[derive(Debug, Clone)]
struct WnoaTrajectoryOutput {
    states: Vec<[f64; 4]>,
    layers: Vec<i32>,
    heights: Vec<f64>,
    costs: Vec<f64>,
}

fn build_wnoj_trajectory_output(
    states: &[Vec6],
    layers: &[i32],
    height_hints: &[f64],
    map: Option<&DenseElevationMap>,
    interpolation_steps: usize,
    gp_qc: f64,
    delta: f64,
) -> WnojTrajectoryOutput {
    let mut output = WnojTrajectoryOutput {
        states: Vec::new(),
        layers: Vec::new(),
        heights: Vec::new(),
        costs: Vec::new(),
    };
    if states.is_empty() {
        return output;
    }

    let push_sample =
        |output: &mut WnojTrajectoryOutput, state: Vec6, layer_hint: i32, height_hint: f64| {
            let (layer, height, cost) = wnoj_result_sample(&state, map, layer_hint, height_hint);
            output.states.push(array_from_vec6(&state));
            output.layers.push(layer);
            output.heights.push(height);
            output.costs.push(cost);
        };

    for index in 0..states.len() {
        let layer_hint = *layers.get(index).unwrap_or(&0);
        let height_hint = *height_hints.get(index).unwrap_or(&0.0);
        push_sample(&mut output, states[index], layer_hint, height_hint);
        if index + 1 < states.len() {
            let next_height = *height_hints.get(index + 1).unwrap_or(&height_hint);
            for step in 1..=interpolation_steps {
                let fraction = step as f64 / (interpolation_steps as f64 + 1.0);
                let tau = delta * fraction;
                let inter_state =
                    wnoj::interpolate(&states[index], &states[index + 1], gp_qc, delta, tau);
                let inter_height_hint = height_hint + (next_height - height_hint) * fraction;
                push_sample(&mut output, inter_state, layer_hint, inter_height_hint);
            }
        }
    }
    output
}

fn build_wnoa_trajectory_output(
    states: &[Vec4],
    layers: &[i32],
    height_hints: &[f64],
    map: Option<&DenseElevationMap>,
    interpolation_steps: usize,
    gp_qc: f64,
    delta: f64,
) -> WnoaTrajectoryOutput {
    let mut output = WnoaTrajectoryOutput {
        states: Vec::new(),
        layers: Vec::new(),
        heights: Vec::new(),
        costs: Vec::new(),
    };
    if states.is_empty() {
        return output;
    }

    let push_sample =
        |output: &mut WnoaTrajectoryOutput, state: Vec4, layer_hint: i32, height_hint: f64| {
            let (layer, height, cost) = wnoa_result_sample(&state, map, layer_hint, height_hint);
            output.states.push(array_from_vec4(&state));
            output.layers.push(layer);
            output.heights.push(height);
            output.costs.push(cost);
        };

    for index in 0..states.len() {
        let layer_hint = *layers.get(index).unwrap_or(&0);
        let height_hint = *height_hints.get(index).unwrap_or(&0.0);
        push_sample(&mut output, states[index], layer_hint, height_hint);
        if index + 1 < states.len() {
            let next_height = *height_hints.get(index + 1).unwrap_or(&height_hint);
            for step in 1..=interpolation_steps {
                let fraction = step as f64 / (interpolation_steps as f64 + 1.0);
                let tau = delta * fraction;
                let inter_state =
                    wnoa::interpolate(&states[index], &states[index + 1], gp_qc, delta, tau);
                let inter_height_hint = height_hint + (next_height - height_hint) * fraction;
                push_sample(&mut output, inter_state, layer_hint, inter_height_hint);
            }
        }
    }
    output
}

fn solve_dense_lm(
    hessian: DMatrix<f64>,
    gradient: DVector<f64>,
    lambda: f64,
) -> Result<DVector<f64>, GpmpError> {
    let mut lm_hessian = hessian;
    for index in 0..lm_hessian.nrows() {
        let diag = lm_hessian[(index, index)].abs().max(1.0);
        lm_hessian[(index, index)] += lambda * diag;
    }

    let rhs = -gradient;
    if let Some(cholesky) = lm_hessian.clone().cholesky() {
        Ok(cholesky.solve(&rhs))
    } else if let Some(solution) = lm_hessian.lu().solve(&rhs) {
        Ok(solution)
    } else {
        Err(GpmpError::SingularSystem)
    }
}

fn solve_sparse_lm(
    hessian: &DMatrix<f64>,
    gradient: &DVector<f64>,
    lambda: f64,
) -> Result<DVector<f64>, GpmpError> {
    if hessian.nrows() != hessian.ncols() || hessian.nrows() != gradient.len() {
        return Err(GpmpError::SingularSystem);
    }

    let sparse = dense_hessian_to_sparse_upper_damped(hessian, lambda)?;
    let llt = sparse
        .sp_cholesky(Side::Upper)
        .map_err(|_| GpmpError::SingularSystem)?;
    let mut rhs = Mat::<f64>::from_fn(gradient.len(), 1, |row, _| -gradient[row]);
    llt.solve_in_place(rhs.as_mut());

    let mut delta = DVector::<f64>::zeros(gradient.len());
    for row in 0..gradient.len() {
        delta[row] = rhs[(row, 0)];
    }
    if validate_dense_lm_solution(hessian, gradient, &delta, lambda) {
        Ok(delta)
    } else {
        Err(GpmpError::SingularSystem)
    }
}

fn solve_general_lm(
    hessian: DMatrix<f64>,
    gradient: DVector<f64>,
    lambda: f64,
    preferred: LinearSolverKind,
) -> Result<(DVector<f64>, LinearSolverKind, usize), GpmpError> {
    let use_sparse = match preferred {
        LinearSolverKind::Sparse => true,
        LinearSolverKind::Auto => hessian.nrows() >= SPARSE_LM_MIN_DIM,
        LinearSolverKind::BlockTridiagonal => hessian.nrows() >= SPARSE_LM_MIN_DIM,
        LinearSolverKind::Dense => false,
    };

    if use_sparse {
        match solve_sparse_lm(&hessian, &gradient, lambda) {
            Ok(step) => return Ok((step, LinearSolverKind::Sparse, 0)),
            Err(_) => {
                let step = solve_dense_lm(hessian, gradient, lambda)?;
                return Ok((step, LinearSolverKind::Dense, 1));
            }
        }
    }

    let step = solve_dense_lm(hessian, gradient, lambda)?;
    Ok((step, LinearSolverKind::Dense, 0))
}

fn dense_hessian_to_sparse_upper_damped(
    hessian: &DMatrix<f64>,
    lambda: f64,
) -> Result<SparseColMat<usize, f64>, GpmpError> {
    let dim = hessian.nrows();
    let mut triplets = Vec::new();
    for col in 0..dim {
        for row in 0..=col {
            let mut value = hessian[(row, col)];
            if row == col {
                value += damping_for_diagonal(value, lambda);
            }
            if value != 0.0 {
                triplets.push(Triplet::new(row, col, value));
            }
        }
    }
    SparseColMat::<usize, f64>::try_new_from_triplets(dim, dim, &triplets)
        .map_err(|_| GpmpError::SingularSystem)
}

fn damping_for_diagonal(diagonal: f64, lambda: f64) -> f64 {
    lambda * diagonal.abs().max(1.0)
}

fn validate_dense_lm_solution(
    hessian: &DMatrix<f64>,
    gradient: &DVector<f64>,
    delta: &DVector<f64>,
    lambda: f64,
) -> bool {
    if !delta.iter().all(|value| value.is_finite()) {
        return false;
    }

    let mut residual = hessian * delta;
    for index in 0..delta.len() {
        residual[index] += damping_for_diagonal(hessian[(index, index)], lambda) * delta[index];
    }
    residual += gradient;
    let residual_max = residual
        .iter()
        .fold(0.0_f64, |acc, value| acc.max(value.abs()));
    let rhs_scale = gradient
        .iter()
        .fold(1.0_f64, |acc, value| acc.max(value.abs()));
    residual_max.is_finite() && residual_max <= 1e-6 * rhs_scale
}

#[derive(Debug, Clone)]
struct BlockTridiagonalSystem<const N: usize> {
    diagonal: Vec<SMatrix<f64, N, N>>,
    upper: Vec<SMatrix<f64, N, N>>,
    gradient: Vec<SVector<f64, N>>,
}

impl<const N: usize> BlockTridiagonalSystem<N> {
    fn new(blocks: usize) -> Self {
        Self {
            diagonal: vec![SMatrix::<f64, N, N>::zeros(); blocks],
            upper: vec![SMatrix::<f64, N, N>::zeros(); blocks.saturating_sub(1)],
            gradient: vec![SVector::<f64, N>::zeros(); blocks],
        }
    }

    fn gradient_norm(&self) -> f64 {
        self.gradient
            .iter()
            .map(|block| block.norm_squared())
            .sum::<f64>()
            .sqrt()
    }

    fn add_gradient(&mut self, index: usize, gradient: SVector<f64, N>) {
        self.gradient[index] += gradient;
    }

    fn add_diagonal(&mut self, index: usize, block: SMatrix<f64, N, N>) {
        self.diagonal[index] += block;
    }

    fn add_between(
        &mut self,
        first: usize,
        second: usize,
        first_first: SMatrix<f64, N, N>,
        first_second: SMatrix<f64, N, N>,
        second_second: SMatrix<f64, N, N>,
    ) -> bool {
        self.diagonal[first] += first_first;
        self.diagonal[second] += second_second;
        if first + 1 == second {
            self.upper[first] += first_second;
            true
        } else if second + 1 == first {
            self.upper[second] += first_second.transpose();
            true
        } else {
            false
        }
    }

    fn solve_damped(&self, lambda: f64) -> Result<Vec<SVector<f64, N>>, GpmpError> {
        let blocks = self.diagonal.len();
        if blocks == 0 {
            return Err(GpmpError::EmptyStateVector);
        }

        let mut diagonal = self.diagonal.clone();
        for block in &mut diagonal {
            for dim in 0..N {
                let diag = block[(dim, dim)].abs().max(1.0);
                block[(dim, dim)] += lambda * diag;
            }
        }

        let mut c_blocks = vec![SMatrix::<f64, N, N>::zeros(); blocks.saturating_sub(1)];
        let mut d_blocks = vec![SVector::<f64, N>::zeros(); blocks];

        for index in 0..blocks {
            let mut effective_diag = diagonal[index];
            let mut rhs = -self.gradient[index];
            if index > 0 {
                let lower = self.upper[index - 1].transpose();
                effective_diag -= lower * c_blocks[index - 1];
                rhs -= lower * d_blocks[index - 1];
            }
            effective_diag = (effective_diag + effective_diag.transpose()) * 0.5;

            let upper_rhs = if index + 1 < blocks {
                Some(&self.upper[index])
            } else {
                None
            };
            let (solved_upper, solved_rhs) =
                solve_small_matrix_and_vector(&effective_diag, upper_rhs, &rhs)?;
            if let Some(solved_upper) = solved_upper {
                c_blocks[index] = solved_upper;
            }
            d_blocks[index] = solved_rhs;
        }

        let mut solution = vec![SVector::<f64, N>::zeros(); blocks];
        solution[blocks - 1] = d_blocks[blocks - 1];
        for index in (0..blocks.saturating_sub(1)).rev() {
            solution[index] = d_blocks[index] - c_blocks[index] * solution[index + 1];
        }
        Ok(solution)
    }
}

fn solve_small_matrix_and_vector<const N: usize>(
    matrix: &SMatrix<f64, N, N>,
    matrix_rhs: Option<&SMatrix<f64, N, N>>,
    vector_rhs: &SVector<f64, N>,
) -> Result<(Option<SMatrix<f64, N, N>>, SVector<f64, N>), GpmpError> {
    if let Some(cholesky) = (*matrix).cholesky() {
        let solved_matrix = matrix_rhs.map(|rhs| cholesky.solve(rhs));
        let solved_vector = cholesky.solve(vector_rhs);
        return Ok((solved_matrix, solved_vector));
    }

    let dynamic_matrix = DMatrix::from_column_slice(N, N, matrix.as_slice());
    let lu = dynamic_matrix.lu();
    let dynamic_vector_rhs = DVector::from_column_slice(vector_rhs.as_slice());
    let dynamic_vector = lu
        .solve(&dynamic_vector_rhs)
        .ok_or(GpmpError::SingularSystem)?;
    let solved_vector = SVector::<f64, N>::from_column_slice(dynamic_vector.as_slice());
    let solved_matrix = match matrix_rhs {
        Some(rhs) => {
            let dynamic_matrix_rhs = DMatrix::from_column_slice(N, N, rhs.as_slice());
            let dynamic_solution = lu
                .solve(&dynamic_matrix_rhs)
                .ok_or(GpmpError::SingularSystem)?;
            Some(SMatrix::<f64, N, N>::from_column_slice(
                dynamic_solution.as_slice(),
            ))
        }
        None => None,
    };
    Ok((solved_matrix, solved_vector))
}

fn flatten_block_step<const N: usize>(blocks: &[SVector<f64, N>]) -> DVector<f64> {
    let mut step = DVector::zeros(blocks.len() * N);
    for (block_index, block) in blocks.iter().enumerate() {
        let base = block_index * N;
        for dim in 0..N {
            step[base + dim] = block[dim];
        }
    }
    step
}

fn accumulate_block_vector_factor<const N: usize>(
    system: &mut BlockTridiagonalSystem<N>,
    from: usize,
    to: usize,
    residual: &SVector<f64, N>,
    information: &SMatrix<f64, N, N>,
    jac_from: &SMatrix<f64, N, N>,
    jac_to: &SMatrix<f64, N, N>,
) -> bool {
    let weighted_residual = information * residual;
    system.add_gradient(from, jac_from.transpose() * weighted_residual);
    system.add_gradient(to, jac_to.transpose() * weighted_residual);
    system.add_between(
        from,
        to,
        jac_from.transpose() * information * jac_from,
        jac_from.transpose() * information * jac_to,
        jac_to.transpose() * information * jac_to,
    )
}

fn accumulate_block_scalar_factor<const N: usize>(
    system: &mut BlockTridiagonalSystem<N>,
    first: usize,
    second: Option<usize>,
    residual: f64,
    weight: f64,
    jac_first: &SMatrix<f64, 1, N>,
    jac_second: Option<&SMatrix<f64, 1, N>>,
) -> bool {
    system.add_gradient(first, jac_first.transpose() * (weight * residual));

    if let (Some(second_index), Some(jac_second)) = (second, jac_second) {
        system.add_gradient(second_index, jac_second.transpose() * (weight * residual));
        system.add_between(
            first,
            second_index,
            jac_first.transpose() * jac_first * weight,
            jac_first.transpose() * jac_second * weight,
            jac_second.transpose() * jac_second * weight,
        )
    } else {
        system.add_diagonal(first, jac_first.transpose() * jac_first * weight);
        true
    }
}

pub fn optimize_wnoj_request(
    request: WnojOptimizeRequest,
) -> Result<WnojOptimizeResponse, GpmpError> {
    let initial_states = request
        .states
        .iter()
        .copied()
        .map(vec6_from_array)
        .collect::<Vec<_>>();
    let initial_node_states = initial_states.clone();
    let mut problem = WnojBatchProblem::new(initial_states)?;
    let state_count = problem.states().len();
    let delta = request.delta.unwrap_or(1.0).max(1e-6);
    let gp_qc = request.gp_qc.unwrap_or(10.0).max(1e-9);
    let endpoint_sigmas = Vec6::from_row_slice(
        &request
            .endpoint_prior_sigmas
            .unwrap_or([0.05, 0.5, 2.0, 0.05, 0.5, 2.0]),
    );
    let first = problem.states()[0];
    let last = problem.states()[state_count - 1];
    problem.add_prior(0, first, endpoint_sigmas)?;
    if state_count > 1 {
        problem.add_prior(state_count - 1, last, endpoint_sigmas)?;
    }
    for index in 0..state_count.saturating_sub(1) {
        problem.add_gp_prior(index, index + 1, delta, gp_qc)?;
    }

    let layers = request.layers.unwrap_or_else(|| vec![0; state_count]);
    let height_hints = request
        .height_hints
        .unwrap_or_else(|| vec![0.0; state_count]);
    let mut map_arc: Option<Arc<DenseElevationMap>> = None;
    if let Some(map_request) = request.map {
        map_arc = Some(Arc::new(map_from_request(map_request)?));
    }
    let cost_threshold = request.cost_threshold.unwrap_or(49.9);
    let obstacle_sigma = request.obstacle_sigma.unwrap_or(1.0).max(1e-9);
    let max_heading_rate = request.max_heading_rate.unwrap_or(10.0);
    let heading_rate_sigma = request.heading_rate_sigma.unwrap_or(1.0).max(1e-9);
    let interpolation_steps = request.interpolation_steps.unwrap_or(2);

    if let Some(map) = map_arc.clone() {
        for index in 0..state_count {
            problem.add_obstacle(
                index,
                map.clone(),
                *layers.get(index).unwrap_or(&0),
                *height_hints.get(index).unwrap_or(&0.0),
                cost_threshold,
                obstacle_sigma,
            )?;
        }
        for index in 0..state_count.saturating_sub(1) {
            for step in 1..=interpolation_steps {
                let tau = delta * step as f64 / (interpolation_steps as f64 + 1.0);
                problem.add_interpolated_obstacle(
                    index,
                    index + 1,
                    map.clone(),
                    *layers.get(index).unwrap_or(&0),
                    *height_hints.get(index).unwrap_or(&0.0),
                    cost_threshold,
                    obstacle_sigma,
                    gp_qc,
                    delta,
                    tau,
                )?;
            }
        }
    }

    for index in 0..state_count {
        problem.add_heading_rate(index, max_heading_rate, heading_rate_sigma)?;
    }
    for index in 0..state_count.saturating_sub(1) {
        for step in 1..=interpolation_steps {
            let tau = delta * step as f64 / (interpolation_steps as f64 + 1.0);
            problem.add_interpolated_heading_rate(
                index,
                index + 1,
                max_heading_rate,
                heading_rate_sigma,
                gp_qc,
                delta,
                tau,
            )?;
        }
    }

    let config = request.config.unwrap_or_default();
    let report = problem.optimize(&config)?;
    let mut result_layers = Vec::with_capacity(state_count);
    let mut result_heights = Vec::with_capacity(state_count);
    let mut result_costs = Vec::with_capacity(state_count);
    for (index, state) in problem.states().iter().enumerate() {
        let layer_hint = *layers.get(index).unwrap_or(&0);
        let height_hint = *height_hints.get(index).unwrap_or(&0.0);
        let (layer, height, cost) =
            wnoj_result_sample(state, map_arc.as_deref(), layer_hint, height_hint);
        result_layers.push(layer);
        result_heights.push(height);
        result_costs.push(cost);
    }
    let trajectory = build_wnoj_trajectory_output(
        problem.states(),
        &layers,
        &height_hints,
        map_arc.as_deref(),
        interpolation_steps,
        gp_qc,
        delta,
    );
    let initial_trajectory = build_wnoj_trajectory_output(
        &initial_node_states,
        &layers,
        &height_hints,
        map_arc.as_deref(),
        interpolation_steps,
        gp_qc,
        delta,
    );

    Ok(WnojOptimizeResponse {
        schema: "lingtu.pct_gpmp.optimize.response.v1".to_string(),
        ok: true,
        error: None,
        states: problem.states().iter().map(array_from_vec6).collect(),
        layers: result_layers,
        heights: result_heights,
        costs: result_costs,
        trajectory_states: trajectory.states,
        trajectory_layers: trajectory.layers,
        trajectory_heights: trajectory.heights,
        trajectory_costs: trajectory.costs,
        initial_trajectory_states: initial_trajectory.states,
        initial_trajectory_layers: initial_trajectory.layers,
        report: Some(report),
    })
}

pub fn optimize_wnoa_request(
    request: WnoaOptimizeRequest,
) -> Result<WnoaOptimizeResponse, GpmpError> {
    let initial_states = request
        .states
        .iter()
        .copied()
        .map(vec4_from_array)
        .collect::<Vec<_>>();
    let initial_node_states = initial_states.clone();
    let mut problem = WnoaBatchProblem::new(initial_states)?;
    let state_count = problem.states().len();
    let delta = request.delta.unwrap_or(1.0).max(1e-6);
    let gp_qc = request.gp_qc.unwrap_or(10.0).max(1e-9);
    let endpoint_sigmas = Vec4::from_row_slice(
        &request
            .endpoint_prior_sigmas
            .unwrap_or([0.05, 0.5, 0.05, 0.5]),
    );
    let first = problem.states()[0];
    let last = problem.states()[state_count - 1];
    problem.add_prior(0, first, endpoint_sigmas)?;
    if state_count > 1 {
        problem.add_prior(state_count - 1, last, endpoint_sigmas)?;
    }
    for index in 0..state_count.saturating_sub(1) {
        problem.add_gp_prior(index, index + 1, delta, gp_qc)?;
    }

    let layers = request.layers.unwrap_or_else(|| vec![0; state_count]);
    let height_hints = request
        .height_hints
        .unwrap_or_else(|| vec![0.0; state_count]);
    let mut map_arc: Option<Arc<DenseElevationMap>> = None;
    if let Some(map_request) = request.map {
        map_arc = Some(Arc::new(map_from_request(map_request)?));
    }
    let cost_threshold = request.cost_threshold.unwrap_or(49.9);
    let obstacle_sigma = request.obstacle_sigma.unwrap_or(1.0).max(1e-9);
    let interpolation_steps = request.interpolation_steps.unwrap_or(2);

    if let Some(map) = map_arc.clone() {
        for index in 0..state_count {
            problem.add_obstacle(
                index,
                map.clone(),
                *layers.get(index).unwrap_or(&0),
                *height_hints.get(index).unwrap_or(&0.0),
                cost_threshold,
                obstacle_sigma,
            )?;
        }
        for index in 0..state_count.saturating_sub(1) {
            for step in 1..=interpolation_steps {
                let tau = delta * step as f64 / (interpolation_steps as f64 + 1.0);
                problem.add_interpolated_obstacle(
                    index,
                    index + 1,
                    map.clone(),
                    *layers.get(index).unwrap_or(&0),
                    *height_hints.get(index).unwrap_or(&0.0),
                    cost_threshold,
                    obstacle_sigma,
                    gp_qc,
                    delta,
                    tau,
                )?;
            }
        }
    }

    let config = request.config.unwrap_or_default();
    let report = problem.optimize(&config)?;
    let mut result_layers = Vec::with_capacity(state_count);
    let mut result_heights = Vec::with_capacity(state_count);
    let mut result_costs = Vec::with_capacity(state_count);
    for (index, state) in problem.states().iter().enumerate() {
        let layer_hint = *layers.get(index).unwrap_or(&0);
        let height_hint = *height_hints.get(index).unwrap_or(&0.0);
        let (layer, height, cost) =
            wnoa_result_sample(state, map_arc.as_deref(), layer_hint, height_hint);
        result_layers.push(layer);
        result_heights.push(height);
        result_costs.push(cost);
    }
    let trajectory = build_wnoa_trajectory_output(
        problem.states(),
        &layers,
        &height_hints,
        map_arc.as_deref(),
        interpolation_steps,
        gp_qc,
        delta,
    );
    let initial_trajectory = build_wnoa_trajectory_output(
        &initial_node_states,
        &layers,
        &height_hints,
        map_arc.as_deref(),
        interpolation_steps,
        gp_qc,
        delta,
    );

    Ok(WnoaOptimizeResponse {
        schema: "lingtu.pct_gpmp.optimize.response.v1".to_string(),
        ok: true,
        error: None,
        states: problem.states().iter().map(array_from_vec4).collect(),
        layers: result_layers,
        heights: result_heights,
        costs: result_costs,
        trajectory_states: trajectory.states,
        trajectory_layers: trajectory.layers,
        trajectory_heights: trajectory.heights,
        trajectory_costs: trajectory.costs,
        initial_trajectory_states: initial_trajectory.states,
        initial_trajectory_layers: initial_trajectory.layers,
        report: Some(report),
    })
}

#[derive(Debug, Clone)]
struct PriorFactor6 {
    index: usize,
    target: Vec6,
    sigmas: Vec6,
}

#[derive(Debug, Clone)]
struct GpPriorFactor6 {
    from: usize,
    to: usize,
    delta: f64,
    qc: f64,
}

#[derive(Debug, Clone)]
struct ObstacleFactor6 {
    index: usize,
    map: Arc<DenseElevationMap>,
    layer: i32,
    height_hint: f64,
    cost_threshold: f64,
    sigma: f64,
}

#[derive(Debug, Clone)]
struct InterpolateObstacleFactor6 {
    from: usize,
    to: usize,
    map: Arc<DenseElevationMap>,
    layer: i32,
    height_hint: f64,
    cost_threshold: f64,
    sigma: f64,
    qc: f64,
    delta: f64,
    tau: f64,
}

#[derive(Debug, Clone)]
struct HeadingRateFactor6 {
    index: usize,
    max_heading_rate: f64,
    sigma: f64,
}

#[derive(Debug, Clone)]
struct InterpolateHeadingRateFactor6 {
    from: usize,
    to: usize,
    max_heading_rate: f64,
    sigma: f64,
    qc: f64,
    delta: f64,
    tau: f64,
}

#[derive(Debug, Clone)]
pub struct WnojBatchProblem {
    states: Vec<Vec6>,
    prior_factors: Vec<PriorFactor6>,
    gp_prior_factors: Vec<GpPriorFactor6>,
    obstacle_factors: Vec<ObstacleFactor6>,
    interpolate_obstacle_factors: Vec<InterpolateObstacleFactor6>,
    heading_rate_factors: Vec<HeadingRateFactor6>,
    interpolate_heading_rate_factors: Vec<InterpolateHeadingRateFactor6>,
}

impl WnojBatchProblem {
    pub fn new(states: Vec<Vec6>) -> Result<Self, GpmpError> {
        if states.is_empty() {
            return Err(GpmpError::EmptyStateVector);
        }
        Ok(Self {
            states,
            prior_factors: Vec::new(),
            gp_prior_factors: Vec::new(),
            obstacle_factors: Vec::new(),
            interpolate_obstacle_factors: Vec::new(),
            heading_rate_factors: Vec::new(),
            interpolate_heading_rate_factors: Vec::new(),
        })
    }

    pub fn states(&self) -> &[Vec6] {
        &self.states
    }

    pub fn states_mut(&mut self) -> &mut [Vec6] {
        &mut self.states
    }

    fn check_index(&self, index: usize) -> Result<(), GpmpError> {
        if index < self.states.len() {
            Ok(())
        } else {
            Err(GpmpError::InvalidIndex)
        }
    }

    pub fn add_prior(&mut self, index: usize, target: Vec6, sigmas: Vec6) -> Result<(), GpmpError> {
        self.check_index(index)?;
        self.prior_factors.push(PriorFactor6 {
            index,
            target,
            sigmas,
        });
        Ok(())
    }

    pub fn add_gp_prior(
        &mut self,
        from: usize,
        to: usize,
        delta: f64,
        qc: f64,
    ) -> Result<(), GpmpError> {
        self.check_index(from)?;
        self.check_index(to)?;
        self.gp_prior_factors.push(GpPriorFactor6 {
            from,
            to,
            delta,
            qc,
        });
        Ok(())
    }

    pub fn add_obstacle(
        &mut self,
        index: usize,
        map: Arc<DenseElevationMap>,
        layer: i32,
        height_hint: f64,
        cost_threshold: f64,
        sigma: f64,
    ) -> Result<(), GpmpError> {
        self.check_index(index)?;
        self.obstacle_factors.push(ObstacleFactor6 {
            index,
            map,
            layer,
            height_hint,
            cost_threshold,
            sigma,
        });
        Ok(())
    }

    pub fn add_interpolated_obstacle(
        &mut self,
        from: usize,
        to: usize,
        map: Arc<DenseElevationMap>,
        layer: i32,
        height_hint: f64,
        cost_threshold: f64,
        sigma: f64,
        qc: f64,
        delta: f64,
        tau: f64,
    ) -> Result<(), GpmpError> {
        self.check_index(from)?;
        self.check_index(to)?;
        self.interpolate_obstacle_factors
            .push(InterpolateObstacleFactor6 {
                from,
                to,
                map,
                layer,
                height_hint,
                cost_threshold,
                sigma,
                qc,
                delta,
                tau,
            });
        Ok(())
    }

    pub fn add_heading_rate(
        &mut self,
        index: usize,
        max_heading_rate: f64,
        sigma: f64,
    ) -> Result<(), GpmpError> {
        self.check_index(index)?;
        self.heading_rate_factors.push(HeadingRateFactor6 {
            index,
            max_heading_rate,
            sigma,
        });
        Ok(())
    }

    pub fn add_interpolated_heading_rate(
        &mut self,
        from: usize,
        to: usize,
        max_heading_rate: f64,
        sigma: f64,
        qc: f64,
        delta: f64,
        tau: f64,
    ) -> Result<(), GpmpError> {
        self.check_index(from)?;
        self.check_index(to)?;
        self.interpolate_heading_rate_factors
            .push(InterpolateHeadingRateFactor6 {
                from,
                to,
                max_heading_rate,
                sigma,
                qc,
                delta,
                tau,
            });
        Ok(())
    }

    pub fn cost(&self) -> f64 {
        self.linearize(false).0
    }

    pub fn optimize(
        &mut self,
        config: &WnojOptimizerConfig,
    ) -> Result<WnojOptimizeReport, GpmpError> {
        let started = Instant::now();
        let mut lambda = config.initial_lambda.max(1e-12);
        let initial_cost = self.cost();
        let mut current_cost = initial_cost;
        let mut accepted_steps = 0;
        let mut iterations = 0;
        let mut fallbacks = 0;
        let nonlinear_optimizer = config.nonlinear_optimizer;
        let prefer_block = config.linear_solver != LinearSolverKind::Dense
            && self.supports_block_tridiagonal_solver();
        let mut reported_solver = if prefer_block {
            LinearSolverKind::BlockTridiagonal
        } else {
            LinearSolverKind::Dense
        };

        for iteration in 0..config.max_iterations {
            iterations = iteration + 1;
            let step = if prefer_block {
                let (cost, system) = self.linearize_block_tridiagonal();
                let gradient_norm = system.gradient_norm();
                current_cost = cost;
                if gradient_norm <= config.gradient_tolerance {
                    break;
                }
                let damping = nonlinear_optimizer.damping(lambda);
                match system.solve_damped(damping) {
                    Ok(blocks) => {
                        reported_solver = LinearSolverKind::BlockTridiagonal;
                        flatten_block_step(&blocks)
                    }
                    Err(_) => {
                        let block_fallback = 1;
                        let (dense_cost, hessian, gradient) = self.linearize(true);
                        debug_assert!((dense_cost - cost).abs() <= 1e-7);
                        let (step, solver, solver_fallbacks) =
                            solve_general_lm(hessian, gradient, damping, LinearSolverKind::Sparse)?;
                        fallbacks += block_fallback + solver_fallbacks;
                        reported_solver = solver;
                        step
                    }
                }
            } else {
                let (cost, hessian, gradient) = self.linearize(true);
                let gradient_norm = gradient.norm();
                current_cost = cost;
                if gradient_norm <= config.gradient_tolerance {
                    break;
                }
                let (step, solver, solver_fallbacks) = solve_general_lm(
                    hessian,
                    gradient,
                    nonlinear_optimizer.damping(lambda),
                    config.linear_solver,
                )?;
                fallbacks += solver_fallbacks;
                reported_solver = solver;
                step
            };

            if step.norm() <= config.step_tolerance {
                break;
            }

            let previous_states = self.states.clone();
            let mut accepted_trial = None;
            for scale in step_scales(nonlinear_optimizer) {
                self.states = previous_states.clone();
                self.apply_scaled_step(&step, *scale);
                let trial_cost = self.cost();
                if trial_cost.is_finite() && trial_cost < current_cost {
                    accepted_trial = Some(trial_cost);
                    break;
                }
            }

            if let Some(trial_cost) = accepted_trial {
                accepted_steps += 1;
                let improvement = current_cost - trial_cost;
                current_cost = trial_cost;
                if nonlinear_optimizer == NonlinearOptimizerKind::LevenbergMarquardt {
                    lambda = (lambda * config.lambda_down).max(1e-12);
                }
                if improvement <= config.cost_tolerance {
                    break;
                }
            } else {
                self.states = previous_states;
                if nonlinear_optimizer == NonlinearOptimizerKind::LevenbergMarquardt {
                    lambda *= config.lambda_up.max(1.0);
                } else {
                    break;
                }
            }
        }

        Ok(WnojOptimizeReport {
            initial_cost,
            final_cost: current_cost,
            iterations,
            accepted_steps,
            nonlinear_optimizer: nonlinear_optimizer.as_report_str().to_string(),
            linear_solver: reported_solver.as_report_str().to_string(),
            linear_solve_fallbacks: fallbacks,
            elapsed_ms: started.elapsed().as_secs_f64() * 1000.0,
        })
    }

    fn apply_scaled_step(&mut self, step: &DVector<f64>, scale: f64) {
        for (state_index, state) in self.states.iter_mut().enumerate() {
            let base = state_index * 6;
            for dim in 0..6 {
                state[dim] += scale * step[base + dim];
            }
        }
    }

    fn linearize(&self, with_derivatives: bool) -> (f64, DMatrix<f64>, DVector<f64>) {
        let dim = self.states.len() * 6;
        let mut hessian = DMatrix::zeros(dim, dim);
        let mut gradient = DVector::zeros(dim);
        let mut cost = 0.0;

        for factor in &self.prior_factors {
            let residual = self.states[factor.index] - factor.target;
            for row in 0..6 {
                let sigma = factor.sigmas[row].abs().max(1e-12);
                let weight = 1.0 / (sigma * sigma);
                cost += weight * residual[row] * residual[row];
                if with_derivatives {
                    let global = factor.index * 6 + row;
                    gradient[global] += weight * residual[row];
                    hessian[(global, global)] += weight;
                }
            }
        }

        for factor in &self.gp_prior_factors {
            let residual = wnoj::prior_residual(
                &self.states[factor.from],
                &self.states[factor.to],
                factor.delta,
            );
            let information = wnoj::q_inverse(factor.qc, factor.delta);
            cost += residual.dot(&(information * residual));
            if with_derivatives {
                let jac_from = wnoj::phi(factor.delta);
                let jac_to = -Mat6::identity();
                self.accumulate_vector_factor(
                    &mut hessian,
                    &mut gradient,
                    factor.from,
                    factor.to,
                    &residual,
                    &information,
                    &jac_from,
                    &jac_to,
                );
            }
        }

        for factor in &self.obstacle_factors {
            let evaluation = wnoj::obstacle_residual_jacobian(
                &factor.map,
                &self.states[factor.index],
                factor.layer,
                factor.height_hint,
                factor.cost_threshold,
            );
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * evaluation.residual * evaluation.residual;
            if with_derivatives {
                self.accumulate_scalar_factor(
                    &mut hessian,
                    &mut gradient,
                    factor.index,
                    None,
                    evaluation.residual,
                    weight,
                    &evaluation.jacobian,
                    None,
                );
            }
        }

        for factor in &self.interpolate_obstacle_factors {
            let (evaluation, jac_from, jac_to) = wnoj::interpolate_obstacle_residual_jacobians(
                &factor.map,
                &self.states[factor.from],
                &self.states[factor.to],
                factor.qc,
                factor.delta,
                factor.tau,
                factor.layer,
                factor.height_hint,
                factor.cost_threshold,
            );
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * evaluation.residual * evaluation.residual;
            if with_derivatives {
                self.accumulate_scalar_factor(
                    &mut hessian,
                    &mut gradient,
                    factor.from,
                    Some(factor.to),
                    evaluation.residual,
                    weight,
                    &jac_from,
                    Some(&jac_to),
                );
            }
        }

        for factor in &self.heading_rate_factors {
            let residual =
                wnoj::heading_rate_residual(&self.states[factor.index], factor.max_heading_rate);
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * residual * residual;
            if with_derivatives {
                let jacobian = wnoj::heading_rate_jacobian(
                    &self.states[factor.index],
                    factor.max_heading_rate,
                );
                self.accumulate_scalar_factor(
                    &mut hessian,
                    &mut gradient,
                    factor.index,
                    None,
                    residual,
                    weight,
                    &jacobian,
                    None,
                );
            }
        }

        for factor in &self.interpolate_heading_rate_factors {
            let (residual, jac_from, jac_to) = wnoj::interpolate_heading_rate_residual_jacobians(
                &self.states[factor.from],
                &self.states[factor.to],
                factor.qc,
                factor.delta,
                factor.tau,
                factor.max_heading_rate,
            );
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * residual * residual;
            if with_derivatives {
                self.accumulate_scalar_factor(
                    &mut hessian,
                    &mut gradient,
                    factor.from,
                    Some(factor.to),
                    residual,
                    weight,
                    &jac_from,
                    Some(&jac_to),
                );
            }
        }

        (cost, hessian, gradient)
    }

    fn supports_block_tridiagonal_solver(&self) -> bool {
        self.gp_prior_factors
            .iter()
            .all(|factor| factor.from.abs_diff(factor.to) == 1)
            && self
                .interpolate_obstacle_factors
                .iter()
                .all(|factor| factor.from.abs_diff(factor.to) == 1)
            && self
                .interpolate_heading_rate_factors
                .iter()
                .all(|factor| factor.from.abs_diff(factor.to) == 1)
    }

    fn linearize_block_tridiagonal(&self) -> (f64, BlockTridiagonalSystem<6>) {
        let mut system = BlockTridiagonalSystem::<6>::new(self.states.len());
        let mut cost = 0.0;

        for factor in &self.prior_factors {
            let residual = self.states[factor.index] - factor.target;
            let mut gradient = Vec6::zeros();
            let mut diagonal = Mat6::zeros();
            for row in 0..6 {
                let sigma = factor.sigmas[row].abs().max(1e-12);
                let weight = 1.0 / (sigma * sigma);
                cost += weight * residual[row] * residual[row];
                gradient[row] += weight * residual[row];
                diagonal[(row, row)] += weight;
            }
            system.add_gradient(factor.index, gradient);
            system.add_diagonal(factor.index, diagonal);
        }

        for factor in &self.gp_prior_factors {
            let residual = wnoj::prior_residual(
                &self.states[factor.from],
                &self.states[factor.to],
                factor.delta,
            );
            let information = wnoj::q_inverse(factor.qc, factor.delta);
            cost += residual.dot(&(information * residual));
            let jac_from = wnoj::phi(factor.delta);
            let jac_to = -Mat6::identity();
            let accumulated = accumulate_block_vector_factor(
                &mut system,
                factor.from,
                factor.to,
                &residual,
                &information,
                &jac_from,
                &jac_to,
            );
            debug_assert!(accumulated);
        }

        for factor in &self.obstacle_factors {
            let evaluation = wnoj::obstacle_residual_jacobian(
                &factor.map,
                &self.states[factor.index],
                factor.layer,
                factor.height_hint,
                factor.cost_threshold,
            );
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * evaluation.residual * evaluation.residual;
            accumulate_block_scalar_factor(
                &mut system,
                factor.index,
                None,
                evaluation.residual,
                weight,
                &evaluation.jacobian,
                None,
            );
        }

        for factor in &self.interpolate_obstacle_factors {
            let (evaluation, jac_from, jac_to) = wnoj::interpolate_obstacle_residual_jacobians(
                &factor.map,
                &self.states[factor.from],
                &self.states[factor.to],
                factor.qc,
                factor.delta,
                factor.tau,
                factor.layer,
                factor.height_hint,
                factor.cost_threshold,
            );
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * evaluation.residual * evaluation.residual;
            let accumulated = accumulate_block_scalar_factor(
                &mut system,
                factor.from,
                Some(factor.to),
                evaluation.residual,
                weight,
                &jac_from,
                Some(&jac_to),
            );
            debug_assert!(accumulated);
        }

        for factor in &self.heading_rate_factors {
            let residual =
                wnoj::heading_rate_residual(&self.states[factor.index], factor.max_heading_rate);
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * residual * residual;
            let jacobian =
                wnoj::heading_rate_jacobian(&self.states[factor.index], factor.max_heading_rate);
            accumulate_block_scalar_factor(
                &mut system,
                factor.index,
                None,
                residual,
                weight,
                &jacobian,
                None,
            );
        }

        for factor in &self.interpolate_heading_rate_factors {
            let (residual, jac_from, jac_to) = wnoj::interpolate_heading_rate_residual_jacobians(
                &self.states[factor.from],
                &self.states[factor.to],
                factor.qc,
                factor.delta,
                factor.tau,
                factor.max_heading_rate,
            );
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * residual * residual;
            let accumulated = accumulate_block_scalar_factor(
                &mut system,
                factor.from,
                Some(factor.to),
                residual,
                weight,
                &jac_from,
                Some(&jac_to),
            );
            debug_assert!(accumulated);
        }

        (cost, system)
    }

    fn accumulate_vector_factor(
        &self,
        hessian: &mut DMatrix<f64>,
        gradient: &mut DVector<f64>,
        from: usize,
        to: usize,
        residual: &Vec6,
        information: &Mat6,
        jac_from: &Mat6,
        jac_to: &Mat6,
    ) {
        let weighted_residual = information * residual;
        let grad_from = jac_from.transpose() * weighted_residual;
        let grad_to = jac_to.transpose() * weighted_residual;
        for row in 0..6 {
            gradient[from * 6 + row] += grad_from[row];
            gradient[to * 6 + row] += grad_to[row];
        }

        let blocks = [
            (from, from, jac_from.transpose() * information * jac_from),
            (from, to, jac_from.transpose() * information * jac_to),
            (to, from, jac_to.transpose() * information * jac_from),
            (to, to, jac_to.transpose() * information * jac_to),
        ];
        for (block_row, block_col, block) in blocks {
            for row in 0..6 {
                for col in 0..6 {
                    hessian[(block_row * 6 + row, block_col * 6 + col)] += block[(row, col)];
                }
            }
        }
    }

    fn accumulate_scalar_factor(
        &self,
        hessian: &mut DMatrix<f64>,
        gradient: &mut DVector<f64>,
        first: usize,
        second: Option<usize>,
        residual: f64,
        weight: f64,
        jac_first: &Row6,
        jac_second: Option<&Row6>,
    ) {
        let mut add_block = |row_state: usize, col_state: usize, row_jac: &Row6, col_jac: &Row6| {
            for row in 0..6 {
                for col in 0..6 {
                    hessian[(row_state * 6 + row, col_state * 6 + col)] +=
                        weight * row_jac[(0, row)] * col_jac[(0, col)];
                }
            }
        };

        for dim in 0..6 {
            gradient[first * 6 + dim] += weight * jac_first[(0, dim)] * residual;
        }
        add_block(first, first, jac_first, jac_first);

        if let (Some(second_index), Some(jac_second)) = (second, jac_second) {
            for dim in 0..6 {
                gradient[second_index * 6 + dim] += weight * jac_second[(0, dim)] * residual;
            }
            add_block(first, second_index, jac_first, jac_second);
            add_block(second_index, first, jac_second, jac_first);
            add_block(second_index, second_index, jac_second, jac_second);
        }
    }
}

#[derive(Debug, Clone)]
struct PriorFactor4 {
    index: usize,
    target: Vec4,
    sigmas: Vec4,
}

#[derive(Debug, Clone)]
struct GpPriorFactor4 {
    from: usize,
    to: usize,
    delta: f64,
    qc: f64,
}

#[derive(Debug, Clone)]
struct ObstacleFactor4 {
    index: usize,
    map: Arc<DenseElevationMap>,
    layer: i32,
    height_hint: f64,
    cost_threshold: f64,
    sigma: f64,
}

#[derive(Debug, Clone)]
struct InterpolateObstacleFactor4 {
    from: usize,
    to: usize,
    map: Arc<DenseElevationMap>,
    layer: i32,
    height_hint: f64,
    cost_threshold: f64,
    sigma: f64,
    qc: f64,
    delta: f64,
    tau: f64,
}

#[derive(Debug, Clone)]
pub struct WnoaBatchProblem {
    states: Vec<Vec4>,
    prior_factors: Vec<PriorFactor4>,
    gp_prior_factors: Vec<GpPriorFactor4>,
    obstacle_factors: Vec<ObstacleFactor4>,
    interpolate_obstacle_factors: Vec<InterpolateObstacleFactor4>,
}

impl WnoaBatchProblem {
    pub fn new(states: Vec<Vec4>) -> Result<Self, GpmpError> {
        if states.is_empty() {
            return Err(GpmpError::EmptyStateVector);
        }
        Ok(Self {
            states,
            prior_factors: Vec::new(),
            gp_prior_factors: Vec::new(),
            obstacle_factors: Vec::new(),
            interpolate_obstacle_factors: Vec::new(),
        })
    }

    pub fn states(&self) -> &[Vec4] {
        &self.states
    }

    pub fn states_mut(&mut self) -> &mut [Vec4] {
        &mut self.states
    }

    fn check_index(&self, index: usize) -> Result<(), GpmpError> {
        if index < self.states.len() {
            Ok(())
        } else {
            Err(GpmpError::InvalidIndex)
        }
    }

    pub fn add_prior(&mut self, index: usize, target: Vec4, sigmas: Vec4) -> Result<(), GpmpError> {
        self.check_index(index)?;
        self.prior_factors.push(PriorFactor4 {
            index,
            target,
            sigmas,
        });
        Ok(())
    }

    pub fn add_gp_prior(
        &mut self,
        from: usize,
        to: usize,
        delta: f64,
        qc: f64,
    ) -> Result<(), GpmpError> {
        self.check_index(from)?;
        self.check_index(to)?;
        self.gp_prior_factors.push(GpPriorFactor4 {
            from,
            to,
            delta,
            qc,
        });
        Ok(())
    }

    pub fn add_obstacle(
        &mut self,
        index: usize,
        map: Arc<DenseElevationMap>,
        layer: i32,
        height_hint: f64,
        cost_threshold: f64,
        sigma: f64,
    ) -> Result<(), GpmpError> {
        self.check_index(index)?;
        self.obstacle_factors.push(ObstacleFactor4 {
            index,
            map,
            layer,
            height_hint,
            cost_threshold,
            sigma,
        });
        Ok(())
    }

    pub fn add_interpolated_obstacle(
        &mut self,
        from: usize,
        to: usize,
        map: Arc<DenseElevationMap>,
        layer: i32,
        height_hint: f64,
        cost_threshold: f64,
        sigma: f64,
        qc: f64,
        delta: f64,
        tau: f64,
    ) -> Result<(), GpmpError> {
        self.check_index(from)?;
        self.check_index(to)?;
        self.interpolate_obstacle_factors
            .push(InterpolateObstacleFactor4 {
                from,
                to,
                map,
                layer,
                height_hint,
                cost_threshold,
                sigma,
                qc,
                delta,
                tau,
            });
        Ok(())
    }

    pub fn cost(&self) -> f64 {
        self.linearize(false).0
    }

    pub fn optimize(
        &mut self,
        config: &WnojOptimizerConfig,
    ) -> Result<WnojOptimizeReport, GpmpError> {
        let started = Instant::now();
        let mut lambda = config.initial_lambda.max(1e-12);
        let initial_cost = self.cost();
        let mut current_cost = initial_cost;
        let mut accepted_steps = 0;
        let mut iterations = 0;
        let mut fallbacks = 0;
        let nonlinear_optimizer = config.nonlinear_optimizer;
        let prefer_block = config.linear_solver != LinearSolverKind::Dense
            && self.supports_block_tridiagonal_solver();
        let mut reported_solver = if prefer_block {
            LinearSolverKind::BlockTridiagonal
        } else {
            LinearSolverKind::Dense
        };

        for iteration in 0..config.max_iterations {
            iterations = iteration + 1;
            let step = if prefer_block {
                let (cost, system) = self.linearize_block_tridiagonal();
                let gradient_norm = system.gradient_norm();
                current_cost = cost;
                if gradient_norm <= config.gradient_tolerance {
                    break;
                }
                let damping = nonlinear_optimizer.damping(lambda);
                match system.solve_damped(damping) {
                    Ok(blocks) => {
                        reported_solver = LinearSolverKind::BlockTridiagonal;
                        flatten_block_step(&blocks)
                    }
                    Err(_) => {
                        let block_fallback = 1;
                        let (dense_cost, hessian, gradient) = self.linearize(true);
                        debug_assert!((dense_cost - cost).abs() <= 1e-7);
                        let (step, solver, solver_fallbacks) =
                            solve_general_lm(hessian, gradient, damping, LinearSolverKind::Sparse)?;
                        fallbacks += block_fallback + solver_fallbacks;
                        reported_solver = solver;
                        step
                    }
                }
            } else {
                let (cost, hessian, gradient) = self.linearize(true);
                let gradient_norm = gradient.norm();
                current_cost = cost;
                if gradient_norm <= config.gradient_tolerance {
                    break;
                }
                let (step, solver, solver_fallbacks) = solve_general_lm(
                    hessian,
                    gradient,
                    nonlinear_optimizer.damping(lambda),
                    config.linear_solver,
                )?;
                fallbacks += solver_fallbacks;
                reported_solver = solver;
                step
            };

            if step.norm() <= config.step_tolerance {
                break;
            }

            let previous_states = self.states.clone();
            let mut accepted_trial = None;
            for scale in step_scales(nonlinear_optimizer) {
                self.states = previous_states.clone();
                self.apply_scaled_step(&step, *scale);
                let trial_cost = self.cost();
                if trial_cost.is_finite() && trial_cost < current_cost {
                    accepted_trial = Some(trial_cost);
                    break;
                }
            }

            if let Some(trial_cost) = accepted_trial {
                accepted_steps += 1;
                let improvement = current_cost - trial_cost;
                current_cost = trial_cost;
                if nonlinear_optimizer == NonlinearOptimizerKind::LevenbergMarquardt {
                    lambda = (lambda * config.lambda_down).max(1e-12);
                }
                if improvement <= config.cost_tolerance {
                    break;
                }
            } else {
                self.states = previous_states;
                if nonlinear_optimizer == NonlinearOptimizerKind::LevenbergMarquardt {
                    lambda *= config.lambda_up.max(1.0);
                } else {
                    break;
                }
            }
        }

        Ok(WnojOptimizeReport {
            initial_cost,
            final_cost: current_cost,
            iterations,
            accepted_steps,
            nonlinear_optimizer: nonlinear_optimizer.as_report_str().to_string(),
            linear_solver: reported_solver.as_report_str().to_string(),
            linear_solve_fallbacks: fallbacks,
            elapsed_ms: started.elapsed().as_secs_f64() * 1000.0,
        })
    }

    fn apply_scaled_step(&mut self, step: &DVector<f64>, scale: f64) {
        for (state_index, state) in self.states.iter_mut().enumerate() {
            let base = state_index * 4;
            for dim in 0..4 {
                state[dim] += scale * step[base + dim];
            }
        }
    }

    fn linearize(&self, with_derivatives: bool) -> (f64, DMatrix<f64>, DVector<f64>) {
        let dim = self.states.len() * 4;
        let mut hessian = DMatrix::zeros(dim, dim);
        let mut gradient = DVector::zeros(dim);
        let mut cost = 0.0;

        for factor in &self.prior_factors {
            let residual = self.states[factor.index] - factor.target;
            for row in 0..4 {
                let sigma = factor.sigmas[row].abs().max(1e-12);
                let weight = 1.0 / (sigma * sigma);
                cost += weight * residual[row] * residual[row];
                if with_derivatives {
                    let global = factor.index * 4 + row;
                    gradient[global] += weight * residual[row];
                    hessian[(global, global)] += weight;
                }
            }
        }

        for factor in &self.gp_prior_factors {
            let residual = wnoa::prior_residual(
                &self.states[factor.from],
                &self.states[factor.to],
                factor.delta,
            );
            let information = wnoa::q_inverse(factor.qc, factor.delta);
            cost += residual.dot(&(information * residual));
            if with_derivatives {
                let jac_from = wnoa::phi(factor.delta);
                let jac_to = -Mat4::identity();
                self.accumulate_vector_factor(
                    &mut hessian,
                    &mut gradient,
                    factor.from,
                    factor.to,
                    &residual,
                    &information,
                    &jac_from,
                    &jac_to,
                );
            }
        }

        for factor in &self.obstacle_factors {
            let evaluation = wnoa::obstacle_residual_jacobian(
                &factor.map,
                &self.states[factor.index],
                factor.layer,
                factor.height_hint,
                factor.cost_threshold,
            );
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * evaluation.residual * evaluation.residual;
            if with_derivatives {
                self.accumulate_scalar_factor(
                    &mut hessian,
                    &mut gradient,
                    factor.index,
                    None,
                    evaluation.residual,
                    weight,
                    &evaluation.jacobian,
                    None,
                );
            }
        }

        for factor in &self.interpolate_obstacle_factors {
            let (evaluation, jac_from, jac_to) = wnoa::interpolate_obstacle_residual_jacobians(
                &factor.map,
                &self.states[factor.from],
                &self.states[factor.to],
                factor.qc,
                factor.delta,
                factor.tau,
                factor.layer,
                factor.height_hint,
                factor.cost_threshold,
            );
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * evaluation.residual * evaluation.residual;
            if with_derivatives {
                self.accumulate_scalar_factor(
                    &mut hessian,
                    &mut gradient,
                    factor.from,
                    Some(factor.to),
                    evaluation.residual,
                    weight,
                    &jac_from,
                    Some(&jac_to),
                );
            }
        }

        (cost, hessian, gradient)
    }

    fn supports_block_tridiagonal_solver(&self) -> bool {
        self.gp_prior_factors
            .iter()
            .all(|factor| factor.from.abs_diff(factor.to) == 1)
            && self
                .interpolate_obstacle_factors
                .iter()
                .all(|factor| factor.from.abs_diff(factor.to) == 1)
    }

    fn linearize_block_tridiagonal(&self) -> (f64, BlockTridiagonalSystem<4>) {
        let mut system = BlockTridiagonalSystem::<4>::new(self.states.len());
        let mut cost = 0.0;

        for factor in &self.prior_factors {
            let residual = self.states[factor.index] - factor.target;
            let mut gradient = Vec4::zeros();
            let mut diagonal = Mat4::zeros();
            for row in 0..4 {
                let sigma = factor.sigmas[row].abs().max(1e-12);
                let weight = 1.0 / (sigma * sigma);
                cost += weight * residual[row] * residual[row];
                gradient[row] += weight * residual[row];
                diagonal[(row, row)] += weight;
            }
            system.add_gradient(factor.index, gradient);
            system.add_diagonal(factor.index, diagonal);
        }

        for factor in &self.gp_prior_factors {
            let residual = wnoa::prior_residual(
                &self.states[factor.from],
                &self.states[factor.to],
                factor.delta,
            );
            let information = wnoa::q_inverse(factor.qc, factor.delta);
            cost += residual.dot(&(information * residual));
            let jac_from = wnoa::phi(factor.delta);
            let jac_to = -Mat4::identity();
            let accumulated = accumulate_block_vector_factor(
                &mut system,
                factor.from,
                factor.to,
                &residual,
                &information,
                &jac_from,
                &jac_to,
            );
            debug_assert!(accumulated);
        }

        for factor in &self.obstacle_factors {
            let evaluation = wnoa::obstacle_residual_jacobian(
                &factor.map,
                &self.states[factor.index],
                factor.layer,
                factor.height_hint,
                factor.cost_threshold,
            );
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * evaluation.residual * evaluation.residual;
            accumulate_block_scalar_factor(
                &mut system,
                factor.index,
                None,
                evaluation.residual,
                weight,
                &evaluation.jacobian,
                None,
            );
        }

        for factor in &self.interpolate_obstacle_factors {
            let (evaluation, jac_from, jac_to) = wnoa::interpolate_obstacle_residual_jacobians(
                &factor.map,
                &self.states[factor.from],
                &self.states[factor.to],
                factor.qc,
                factor.delta,
                factor.tau,
                factor.layer,
                factor.height_hint,
                factor.cost_threshold,
            );
            let weight = 1.0 / factor.sigma.abs().max(1e-12).powi(2);
            cost += weight * evaluation.residual * evaluation.residual;
            let accumulated = accumulate_block_scalar_factor(
                &mut system,
                factor.from,
                Some(factor.to),
                evaluation.residual,
                weight,
                &jac_from,
                Some(&jac_to),
            );
            debug_assert!(accumulated);
        }

        (cost, system)
    }

    fn accumulate_vector_factor(
        &self,
        hessian: &mut DMatrix<f64>,
        gradient: &mut DVector<f64>,
        from: usize,
        to: usize,
        residual: &Vec4,
        information: &Mat4,
        jac_from: &Mat4,
        jac_to: &Mat4,
    ) {
        let weighted_residual = information * residual;
        let grad_from = jac_from.transpose() * weighted_residual;
        let grad_to = jac_to.transpose() * weighted_residual;
        for row in 0..4 {
            gradient[from * 4 + row] += grad_from[row];
            gradient[to * 4 + row] += grad_to[row];
        }

        let blocks = [
            (from, from, jac_from.transpose() * information * jac_from),
            (from, to, jac_from.transpose() * information * jac_to),
            (to, from, jac_to.transpose() * information * jac_from),
            (to, to, jac_to.transpose() * information * jac_to),
        ];
        for (block_row, block_col, block) in blocks {
            for row in 0..4 {
                for col in 0..4 {
                    hessian[(block_row * 4 + row, block_col * 4 + col)] += block[(row, col)];
                }
            }
        }
    }

    fn accumulate_scalar_factor(
        &self,
        hessian: &mut DMatrix<f64>,
        gradient: &mut DVector<f64>,
        first: usize,
        second: Option<usize>,
        residual: f64,
        weight: f64,
        jac_first: &Row4,
        jac_second: Option<&Row4>,
    ) {
        let mut add_block = |row_state: usize, col_state: usize, row_jac: &Row4, col_jac: &Row4| {
            for row in 0..4 {
                for col in 0..4 {
                    hessian[(row_state * 4 + row, col_state * 4 + col)] +=
                        weight * row_jac[(0, row)] * col_jac[(0, col)];
                }
            }
        };

        for dim in 0..4 {
            gradient[first * 4 + dim] += weight * jac_first[(0, dim)] * residual;
        }
        add_block(first, first, jac_first, jac_first);

        if let (Some(second_index), Some(jac_second)) = (second, jac_second) {
            for dim in 0..4 {
                gradient[second_index * 4 + dim] += weight * jac_second[(0, dim)] * residual;
            }
            add_block(first, second_index, jac_first, jac_second);
            add_block(second_index, first, jac_second, jac_first);
            add_block(second_index, second_index, jac_second, jac_second);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::{
        lingtu_gpmp_optimizer_abi_version, lingtu_gpmp_optimizer_free_json,
        lingtu_gpmp_optimizer_optimize_json, optimize_request_json, optimize_wnoa_request, wnoa,
        wnoj, DenseElevationMap, DenseElevationMapRequest, LinearSolverKind,
        NonlinearOptimizerKind, Vec4, Vec6, WnoaBatchProblem, WnoaOptimizeRequest,
        WnojBatchProblem, WnojOptimizerConfig, GPMP_OPTIMIZER_ABI_VERSION,
    };

    fn assert_close(actual: f64, expected: f64, tol: f64) {
        assert!(
            (actual - expected).abs() <= tol,
            "actual={actual} expected={expected} tol={tol}"
        );
    }

    fn assert_vec_close<const N: usize>(
        actual: &nalgebra::SVector<f64, N>,
        expected: &nalgebra::SVector<f64, N>,
        tol: f64,
    ) {
        let max_error = actual
            .iter()
            .zip(expected.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        assert!(max_error <= tol, "max_error={max_error} tol={tol}");
    }

    #[test]
    fn wnoj_phi_and_q_match_cpp_formula() {
        let phi = wnoj::phi(0.5);
        assert_close(phi[(0, 1)], 0.5, 1e-12);
        assert_close(phi[(0, 2)], 0.125, 1e-12);
        assert_close(phi[(3, 4)], 0.5, 1e-12);
        assert_close(phi[(3, 5)], 0.125, 1e-12);

        let q = wnoj::q(0.1, 0.5);
        assert_close(q[(0, 0)], 0.00015625, 1e-12);
        assert_close(q[(0, 1)], 0.00078125, 1e-12);
        assert_close(q[(0, 2)], 0.0020833333333333333, 1e-12);
        assert_close(q[(1, 1)], 0.004166666666666667, 1e-12);
        assert_close(q[(2, 2)], 0.05, 1e-12);
        assert_close(q[(3, 3)], q[(0, 0)], 1e-12);
    }

    #[test]
    fn wnoa_phi_and_q_match_cpp_formula() {
        let phi = wnoa::phi(0.5);
        assert_close(phi[(0, 1)], 0.5, 1e-12);
        assert_close(phi[(2, 3)], 0.5, 1e-12);

        let q = wnoa::q(0.1, 0.5);
        assert_close(q[(0, 0)], 0.004166666666666667, 1e-12);
        assert_close(q[(0, 1)], 0.0125, 1e-12);
        assert_close(q[(1, 1)], 0.05, 1e-12);
        assert_close(q[(2, 2)], q[(0, 0)], 1e-12);
    }

    #[test]
    fn gp_prior_residual_uses_phi_x1_minus_x2() {
        let x1 = Vec6::from_row_slice(&[1.0, 2.0, 0.5, -1.0, 0.25, -0.1]);
        let x2 = Vec6::from_row_slice(&[2.1, 2.25, 0.5, -0.9, 0.20, -0.1]);
        let expected = wnoj::phi(0.4) * x1 - x2;
        assert_vec_close(&wnoj::prior_residual(&x1, &x2, 0.4), &expected, 1e-12);

        let x1 = Vec4::from_row_slice(&[1.0, 2.0, -1.0, 0.25]);
        let x2 = Vec4::from_row_slice(&[1.8, 2.0, -0.9, 0.25]);
        let expected = wnoa::phi(0.4) * x1 - x2;
        assert_vec_close(&wnoa::prior_residual(&x1, &x2, 0.4), &expected, 1e-12);
    }

    #[test]
    fn state_from_path_point_matches_cpp_path_point_to_node_order() {
        let heading = std::f64::consts::FRAC_PI_4;
        let wnoj_state = wnoj::state_from_path_point(2.0, 3.0, heading, 0.2);
        assert_vec_close(
            &wnoj_state,
            &Vec6::from_row_slice(&[2.0, heading.cos(), 0.0, 3.0, heading.sin(), 0.0]),
            1e-12,
        );

        let wnoa_state = wnoa::state_from_path_point(2.0, 3.0, heading, 2.0);
        assert_vec_close(
            &wnoa_state,
            &Vec4::from_row_slice(&[2.0, 2.0 * heading.cos(), 3.0, 2.0 * heading.sin()]),
            1e-12,
        );
    }

    #[test]
    fn interpolation_matches_lambda_x1_plus_psi_x2() {
        let x1 = Vec6::from_row_slice(&[1.0, 2.0, 0.5, -1.0, 0.25, -0.1]);
        let x2 = Vec6::from_row_slice(&[2.1, 2.25, 0.4, -0.9, 0.20, -0.2]);
        let (lambda, psi) = wnoj::lambda_psi(0.1, 1.0, 0.35);
        let expected = lambda * x1 + psi * x2;
        assert_vec_close(
            &wnoj::interpolate(&x1, &x2, 0.1, 1.0, 0.35),
            &expected,
            1e-12,
        );

        let x1 = Vec4::from_row_slice(&[1.0, 2.0, -1.0, 0.25]);
        let x2 = Vec4::from_row_slice(&[2.1, 2.25, -0.9, 0.20]);
        let (lambda, psi) = wnoa::lambda_psi(0.1, 1.0, 0.35);
        let expected = lambda * x1 + psi * x2;
        assert_vec_close(
            &wnoa::interpolate(&x1, &x2, 0.1, 1.0, 0.35),
            &expected,
            1e-12,
        );
    }

    #[test]
    fn heading_rate_residual_matches_cpp_piecewise_rule() {
        let state = Vec6::from_row_slice(&[0.0, 1.0, 0.0, 0.0, 0.0, 2.0]);
        assert_close(wnoj::heading_rate(&state), 1.999998000002, 1e-12);
        assert_close(
            wnoj::heading_rate_residual(&state, 0.5),
            1.499998000002,
            1e-12,
        );

        let state = Vec6::from_row_slice(&[0.0, 1.0, 2.0, 0.0, 0.0, 0.0]);
        assert_close(wnoj::heading_rate_residual(&state, 0.5), 0.0, 1e-12);

        let state = Vec6::from_row_slice(&[0.0, 0.0, 0.0, 0.0, 1.0, 2.0]);
        assert_close(wnoj::heading_rate_residual(&state, 0.5), 0.0, 1e-12);
    }

    #[test]
    fn heading_rate_jacobian_matches_finite_difference() {
        let state = Vec6::from_row_slice(&[0.2, 1.1, -0.4, -0.3, 0.7, 1.8]);
        let max_rate = 0.2;
        let jac = wnoj::heading_rate_jacobian(&state, max_rate);
        let eps = 1e-6;
        for index in 0..6 {
            let mut plus = state;
            let mut minus = state;
            plus[index] += eps;
            minus[index] -= eps;
            let numeric = (wnoj::heading_rate_residual(&plus, max_rate)
                - wnoj::heading_rate_residual(&minus, max_rate))
                / (2.0 * eps);
            assert_close(jac[(0, index)], numeric, 1e-6);
        }
    }

    #[test]
    fn interpolated_heading_rate_jacobians_match_finite_difference() {
        let x1 = Vec6::from_row_slice(&[0.2, 1.1, -0.4, -0.3, 0.7, 1.8]);
        let x2 = Vec6::from_row_slice(&[0.9, 1.0, -0.2, 0.4, 0.8, 1.3]);
        let max_rate = 0.2;
        let qc = 0.1;
        let delta = 1.0;
        let tau = 0.35;
        let (residual, jac_x1, jac_x2) =
            wnoj::interpolate_heading_rate_residual_jacobians(&x1, &x2, qc, delta, tau, max_rate);
        assert_close(
            residual,
            wnoj::interpolate_heading_rate_residual(&x1, &x2, qc, delta, tau, max_rate),
            1e-12,
        );

        let eps = 1e-6;
        for index in 0..6 {
            let mut plus = x1;
            let mut minus = x1;
            plus[index] += eps;
            minus[index] -= eps;
            let numeric =
                (wnoj::interpolate_heading_rate_residual(&plus, &x2, qc, delta, tau, max_rate)
                    - wnoj::interpolate_heading_rate_residual(
                        &minus, &x2, qc, delta, tau, max_rate,
                    ))
                    / (2.0 * eps);
            assert_close(jac_x1[(0, index)], numeric, 1e-5);

            let mut plus = x2;
            let mut minus = x2;
            plus[index] += eps;
            minus[index] -= eps;
            let numeric =
                (wnoj::interpolate_heading_rate_residual(&x1, &plus, qc, delta, tau, max_rate)
                    - wnoj::interpolate_heading_rate_residual(
                        &x1, &minus, qc, delta, tau, max_rate,
                    ))
                    / (2.0 * eps);
            assert_close(jac_x2[(0, index)], numeric, 1e-5);
        }
    }

    #[test]
    fn lambda_psi_endpoints_match_gp_interpolator_semantics() {
        let x1 = Vec6::from_row_slice(&[1.0, 2.0, 0.5, -1.0, 0.25, -0.1]);
        let x2 = Vec6::from_row_slice(&[2.1, 2.25, 0.4, -0.9, 0.20, -0.2]);
        assert_vec_close(&wnoj::interpolate(&x1, &x2, 0.1, 1.0, 0.0), &x1, 1e-10);
        assert_vec_close(&wnoj::interpolate(&x1, &x2, 0.1, 1.0, 1.0), &x2, 1e-10);

        let x1 = Vec4::from_row_slice(&[1.0, 2.0, -1.0, 0.25]);
        let x2 = Vec4::from_row_slice(&[2.1, 2.25, -0.9, 0.20]);
        assert_vec_close(&wnoa::interpolate(&x1, &x2, 0.1, 1.0, 0.0), &x1, 1e-10);
        assert_vec_close(&wnoa::interpolate(&x1, &x2, 0.1, 1.0, 1.0), &x2, 1e-10);
    }

    fn sloped_map() -> DenseElevationMap {
        let max_x = 8;
        let max_y = 8;
        let mut cost = Vec::with_capacity(max_x * max_y);
        let mut height = Vec::with_capacity(max_x * max_y);
        let mut ceiling = Vec::with_capacity(max_x * max_y);
        for y in 0..max_y {
            for x in 0..max_x {
                cost.push(9.0 + 0.8 * x as f64 + 1.2 * y as f64);
                height.push(0.0);
                ceiling.push(3.0);
            }
        }
        DenseElevationMap::new(1.0, 1, max_x, max_y, cost, height, ceiling).unwrap()
    }

    #[test]
    fn dense_elevation_map_bilinear_safe_matches_cpp_cost_shape() {
        let map = sloped_map();
        let (cost, grad) = map.value_bilinear_safe(0, 2.75, 2.75, 0.0);
        assert_close(cost, 9.0 + 0.8 * 2.75 + 1.2 * 2.75, 1e-12);
        assert_close(grad[0], 0.8, 1e-12);
        assert_close(grad[1], 1.2, 1e-12);
        assert_close(map.get_ceiling(0, 2.75, 2.75), 3.0, 1e-12);
    }

    #[test]
    fn obstacle_residuals_match_cpp_squared_violation_rule() {
        let map = sloped_map();
        let state = Vec6::from_row_slice(&[2.75, 0.0, 0.0, 2.75, 0.0, 0.0]);
        let evaluation = wnoj::obstacle_residual_jacobian(&map, &state, 0, 0.0, 10.0);
        assert_close(evaluation.cost, 14.5, 1e-12);
        assert_close(evaluation.residual, 20.25, 1e-12);
        assert_close(evaluation.jacobian[(0, 0)], 7.2, 1e-12);
        assert_close(evaluation.jacobian[(0, 3)], 10.8, 1e-12);

        let state = Vec4::from_row_slice(&[2.75, 0.0, 2.75, 0.0]);
        let evaluation = wnoa::obstacle_residual_jacobian(&map, &state, 0, 0.0, 10.0);
        assert_close(evaluation.cost, 14.5, 1e-12);
        assert_close(evaluation.residual, 20.25, 1e-12);
        assert_close(evaluation.jacobian[(0, 0)], 7.2, 1e-12);
        assert_close(evaluation.jacobian[(0, 2)], 10.8, 1e-12);
    }

    #[test]
    fn obstacle_jacobians_match_finite_difference() {
        let map = sloped_map();
        let state = Vec6::from_row_slice(&[2.75, 0.0, 0.0, 2.75, 0.0, 0.0]);
        let evaluation = wnoj::obstacle_residual_jacobian(&map, &state, 0, 0.0, 10.0);
        let eps = 1e-6;
        for index in [0, 3] {
            let mut plus = state;
            let mut minus = state;
            plus[index] += eps;
            minus[index] -= eps;
            let numeric = (wnoj::obstacle_residual_jacobian(&map, &plus, 0, 0.0, 10.0).residual
                - wnoj::obstacle_residual_jacobian(&map, &minus, 0, 0.0, 10.0).residual)
                / (2.0 * eps);
            assert_close(evaluation.jacobian[(0, index)], numeric, 1e-6);
        }
    }

    #[test]
    fn interpolated_obstacle_jacobians_match_finite_difference() {
        let map = sloped_map();
        let x1 = Vec6::from_row_slice(&[2.0, 0.4, 0.0, 2.2, 0.3, 0.0]);
        let x2 = Vec6::from_row_slice(&[3.2, 0.4, 0.0, 3.0, 0.3, 0.0]);
        let qc = 0.1;
        let delta = 1.0;
        let tau = 0.35;
        let (evaluation, jac_x1, jac_x2) = wnoj::interpolate_obstacle_residual_jacobians(
            &map, &x1, &x2, qc, delta, tau, 0, 0.0, 10.0,
        );
        assert!(evaluation.residual > 0.0);
        let eps = 1e-6;
        for index in 0..6 {
            let mut plus = x1;
            let mut minus = x1;
            plus[index] += eps;
            minus[index] -= eps;
            let numeric = (wnoj::interpolate_obstacle_residual_jacobians(
                &map, &plus, &x2, qc, delta, tau, 0, 0.0, 10.0,
            )
            .0
            .residual
                - wnoj::interpolate_obstacle_residual_jacobians(
                    &map, &minus, &x2, qc, delta, tau, 0, 0.0, 10.0,
                )
                .0
                .residual)
                / (2.0 * eps);
            assert_close(jac_x1[(0, index)], numeric, 1e-5);

            let mut plus = x2;
            let mut minus = x2;
            plus[index] += eps;
            minus[index] -= eps;
            let numeric = (wnoj::interpolate_obstacle_residual_jacobians(
                &map, &x1, &plus, qc, delta, tau, 0, 0.0, 10.0,
            )
            .0
            .residual
                - wnoj::interpolate_obstacle_residual_jacobians(
                    &map, &x1, &minus, qc, delta, tau, 0, 0.0, 10.0,
                )
                .0
                .residual)
                / (2.0 * eps);
            assert_close(jac_x2[(0, index)], numeric, 1e-5);
        }
    }

    #[test]
    fn wnoa_interpolated_obstacle_jacobians_match_finite_difference() {
        let map = sloped_map();
        let x1 = Vec4::from_row_slice(&[2.0, 0.4, 2.2, 0.3]);
        let x2 = Vec4::from_row_slice(&[3.2, 0.4, 3.0, 0.3]);
        let qc = 0.1;
        let delta = 1.0;
        let tau = 0.35;
        let (evaluation, jac_x1, jac_x2) = wnoa::interpolate_obstacle_residual_jacobians(
            &map, &x1, &x2, qc, delta, tau, 0, 0.0, 10.0,
        );
        assert!(evaluation.residual > 0.0);
        let eps = 1e-6;
        for index in 0..4 {
            let mut plus = x1;
            let mut minus = x1;
            plus[index] += eps;
            minus[index] -= eps;
            let numeric = (wnoa::interpolate_obstacle_residual_jacobians(
                &map, &plus, &x2, qc, delta, tau, 0, 0.0, 10.0,
            )
            .0
            .residual
                - wnoa::interpolate_obstacle_residual_jacobians(
                    &map, &minus, &x2, qc, delta, tau, 0, 0.0, 10.0,
                )
                .0
                .residual)
                / (2.0 * eps);
            assert_close(jac_x1[(0, index)], numeric, 1e-5);

            let mut plus = x2;
            let mut minus = x2;
            plus[index] += eps;
            minus[index] -= eps;
            let numeric = (wnoa::interpolate_obstacle_residual_jacobians(
                &map, &x1, &plus, qc, delta, tau, 0, 0.0, 10.0,
            )
            .0
            .residual
                - wnoa::interpolate_obstacle_residual_jacobians(
                    &map, &x1, &minus, qc, delta, tau, 0, 0.0, 10.0,
                )
                .0
                .residual)
                / (2.0 * eps);
            assert_close(jac_x2[(0, index)], numeric, 1e-5);
        }
    }

    #[test]
    fn wnoj_batch_optimizer_reduces_obstacle_cost() {
        let map = Arc::new(sloped_map());
        let x0 = Vec6::from_row_slice(&[1.0, 2.0, 0.0, 4.0, 0.0, 0.0]);
        let x1 = Vec6::from_row_slice(&[3.0, 2.0, 0.0, 4.0, 0.0, 0.0]);
        let x2 = Vec6::from_row_slice(&[5.0, 2.0, 0.0, 4.0, 0.0, 0.0]);
        let mut problem = WnojBatchProblem::new(vec![x0, x1, x2]).unwrap();
        let strong = Vec6::from_row_slice(&[0.01, 0.1, 1.0, 0.01, 0.1, 1.0]);
        problem.add_prior(0, x0, strong).unwrap();
        problem.add_prior(2, x2, strong).unwrap();
        problem.add_gp_prior(0, 1, 1.0, 10.0).unwrap();
        problem.add_gp_prior(1, 2, 1.0, 10.0).unwrap();
        problem
            .add_obstacle(1, map.clone(), 0, 0.0, 10.0, 0.2)
            .unwrap();

        let initial_residual =
            wnoj::obstacle_residual_jacobian(&map, &problem.states()[1], 0, 0.0, 10.0).residual;
        let report = problem.optimize(&WnojOptimizerConfig::default()).unwrap();
        let final_residual =
            wnoj::obstacle_residual_jacobian(&map, &problem.states()[1], 0, 0.0, 10.0).residual;

        assert!(report.accepted_steps > 0);
        assert_eq!(report.nonlinear_optimizer, "levenberg_marquardt");
        assert_eq!(report.linear_solver, "block_tridiagonal");
        assert_eq!(report.linear_solve_fallbacks, 0);
        assert!(report.final_cost < report.initial_cost);
        assert!(final_residual < initial_residual);
        assert!(problem.states()[1][3] < x1[3]);
    }

    #[test]
    fn wnoa_batch_optimizer_reduces_obstacle_cost() {
        let map = Arc::new(sloped_map());
        let x0 = Vec4::from_row_slice(&[1.0, 2.0, 4.0, 0.0]);
        let x1 = Vec4::from_row_slice(&[3.0, 2.0, 4.0, 0.0]);
        let x2 = Vec4::from_row_slice(&[5.0, 2.0, 4.0, 0.0]);
        let mut problem = WnoaBatchProblem::new(vec![x0, x1, x2]).unwrap();
        let strong = Vec4::from_row_slice(&[0.01, 0.1, 0.01, 0.1]);
        problem.add_prior(0, x0, strong).unwrap();
        problem.add_prior(2, x2, strong).unwrap();
        problem.add_gp_prior(0, 1, 1.0, 10.0).unwrap();
        problem.add_gp_prior(1, 2, 1.0, 10.0).unwrap();
        problem
            .add_obstacle(1, map.clone(), 0, 0.0, 10.0, 0.2)
            .unwrap();

        let initial_residual =
            wnoa::obstacle_residual_jacobian(&map, &problem.states()[1], 0, 0.0, 10.0).residual;
        let report = problem.optimize(&WnojOptimizerConfig::default()).unwrap();
        let final_residual =
            wnoa::obstacle_residual_jacobian(&map, &problem.states()[1], 0, 0.0, 10.0).residual;

        assert!(report.accepted_steps > 0);
        assert_eq!(report.nonlinear_optimizer, "levenberg_marquardt");
        assert_eq!(report.linear_solver, "block_tridiagonal");
        assert_eq!(report.linear_solve_fallbacks, 0);
        assert!(report.final_cost < report.initial_cost);
        assert!(final_residual < initial_residual);
        assert!(problem.states()[1][2] < x1[2]);
    }

    #[test]
    fn wnoj_block_tridiagonal_solver_matches_dense_solver() {
        let map = Arc::new(sloped_map());
        let states = vec![
            Vec6::from_row_slice(&[1.0, 2.0, 0.0, 4.0, 0.0, 0.0]),
            Vec6::from_row_slice(&[3.0, 2.0, 0.0, 4.0, 0.0, 0.0]),
            Vec6::from_row_slice(&[5.0, 2.0, 0.0, 4.0, 0.0, 0.0]),
        ];
        let mut dense_problem = WnojBatchProblem::new(states.clone()).unwrap();
        let mut sparse_problem = WnojBatchProblem::new(states).unwrap();
        for problem in [&mut dense_problem, &mut sparse_problem] {
            let strong = Vec6::from_row_slice(&[0.01, 0.1, 1.0, 0.01, 0.1, 1.0]);
            problem.add_prior(0, problem.states()[0], strong).unwrap();
            problem.add_prior(2, problem.states()[2], strong).unwrap();
            problem.add_gp_prior(0, 1, 1.0, 10.0).unwrap();
            problem.add_gp_prior(1, 2, 1.0, 10.0).unwrap();
            problem
                .add_interpolated_obstacle(0, 1, map.clone(), 0, 0.0, 10.0, 0.2, 10.0, 1.0, 0.5)
                .unwrap();
            problem
                .add_obstacle(1, map.clone(), 0, 0.0, 10.0, 0.2)
                .unwrap();
            problem
                .add_interpolated_obstacle(1, 2, map.clone(), 0, 0.0, 10.0, 0.2, 10.0, 1.0, 0.5)
                .unwrap();
        }

        let mut dense_config = WnojOptimizerConfig::default();
        dense_config.linear_solver = LinearSolverKind::Dense;
        let mut sparse_config = WnojOptimizerConfig::default();
        sparse_config.linear_solver = LinearSolverKind::BlockTridiagonal;

        let dense_report = dense_problem.optimize(&dense_config).unwrap();
        let sparse_report = sparse_problem.optimize(&sparse_config).unwrap();

        assert_eq!(dense_report.linear_solver, "dense");
        assert_eq!(sparse_report.linear_solver, "block_tridiagonal");
        assert_eq!(dense_report.nonlinear_optimizer, "levenberg_marquardt");
        assert_eq!(sparse_report.nonlinear_optimizer, "levenberg_marquardt");
        assert_close(sparse_report.final_cost, dense_report.final_cost, 1e-7);
        for (dense, sparse) in dense_problem.states().iter().zip(sparse_problem.states()) {
            assert_vec_close(sparse, dense, 1e-7);
        }
    }

    #[test]
    fn wnoa_block_tridiagonal_solver_matches_dense_solver() {
        let map = Arc::new(sloped_map());
        let states = vec![
            Vec4::from_row_slice(&[1.0, 2.0, 4.0, 0.0]),
            Vec4::from_row_slice(&[3.0, 2.0, 4.0, 0.0]),
            Vec4::from_row_slice(&[5.0, 2.0, 4.0, 0.0]),
        ];
        let mut dense_problem = WnoaBatchProblem::new(states.clone()).unwrap();
        let mut sparse_problem = WnoaBatchProblem::new(states).unwrap();
        for problem in [&mut dense_problem, &mut sparse_problem] {
            let strong = Vec4::from_row_slice(&[0.01, 0.1, 0.01, 0.1]);
            problem.add_prior(0, problem.states()[0], strong).unwrap();
            problem.add_prior(2, problem.states()[2], strong).unwrap();
            problem.add_gp_prior(0, 1, 1.0, 10.0).unwrap();
            problem.add_gp_prior(1, 2, 1.0, 10.0).unwrap();
            problem
                .add_interpolated_obstacle(0, 1, map.clone(), 0, 0.0, 10.0, 0.2, 10.0, 1.0, 0.5)
                .unwrap();
            problem
                .add_obstacle(1, map.clone(), 0, 0.0, 10.0, 0.2)
                .unwrap();
            problem
                .add_interpolated_obstacle(1, 2, map.clone(), 0, 0.0, 10.0, 0.2, 10.0, 1.0, 0.5)
                .unwrap();
        }

        let mut dense_config = WnojOptimizerConfig::default();
        dense_config.linear_solver = LinearSolverKind::Dense;
        let mut sparse_config = WnojOptimizerConfig::default();
        sparse_config.linear_solver = LinearSolverKind::BlockTridiagonal;

        let dense_report = dense_problem.optimize(&dense_config).unwrap();
        let sparse_report = sparse_problem.optimize(&sparse_config).unwrap();

        assert_eq!(dense_report.linear_solver, "dense");
        assert_eq!(sparse_report.linear_solver, "block_tridiagonal");
        assert_eq!(dense_report.nonlinear_optimizer, "levenberg_marquardt");
        assert_eq!(sparse_report.nonlinear_optimizer, "levenberg_marquardt");
        assert_close(sparse_report.final_cost, dense_report.final_cost, 1e-7);
        for (dense, sparse) in dense_problem.states().iter().zip(sparse_problem.states()) {
            assert_vec_close(sparse, dense, 1e-7);
        }
    }

    #[test]
    fn wnoj_block_tridiagonal_gauss_newton_matches_dense_solver() {
        let map = Arc::new(sloped_map());
        let states = vec![
            Vec6::from_row_slice(&[1.0, 2.0, 0.0, 4.0, 0.0, 0.0]),
            Vec6::from_row_slice(&[3.0, 2.0, 0.0, 4.0, 0.0, 0.0]),
            Vec6::from_row_slice(&[5.0, 2.0, 0.0, 4.0, 0.0, 0.0]),
        ];
        let mut dense_problem = WnojBatchProblem::new(states.clone()).unwrap();
        let mut sparse_problem = WnojBatchProblem::new(states).unwrap();
        for problem in [&mut dense_problem, &mut sparse_problem] {
            let strong = Vec6::from_row_slice(&[0.01, 0.1, 1.0, 0.01, 0.1, 1.0]);
            problem.add_prior(0, problem.states()[0], strong).unwrap();
            problem.add_prior(2, problem.states()[2], strong).unwrap();
            problem.add_gp_prior(0, 1, 1.0, 10.0).unwrap();
            problem.add_gp_prior(1, 2, 1.0, 10.0).unwrap();
            problem
                .add_interpolated_obstacle(0, 1, map.clone(), 0, 0.0, 10.0, 0.2, 10.0, 1.0, 0.5)
                .unwrap();
            problem
                .add_obstacle(1, map.clone(), 0, 0.0, 10.0, 0.2)
                .unwrap();
            problem
                .add_interpolated_obstacle(1, 2, map.clone(), 0, 0.0, 10.0, 0.2, 10.0, 1.0, 0.5)
                .unwrap();
        }

        let mut dense_config = WnojOptimizerConfig::default();
        dense_config.linear_solver = LinearSolverKind::Dense;
        dense_config.nonlinear_optimizer = NonlinearOptimizerKind::GaussNewton;
        let mut sparse_config = WnojOptimizerConfig::default();
        sparse_config.linear_solver = LinearSolverKind::BlockTridiagonal;
        sparse_config.nonlinear_optimizer = NonlinearOptimizerKind::GaussNewton;

        let dense_report = dense_problem.optimize(&dense_config).unwrap();
        let sparse_report = sparse_problem.optimize(&sparse_config).unwrap();

        assert_eq!(dense_report.nonlinear_optimizer, "gauss_newton");
        assert_eq!(sparse_report.nonlinear_optimizer, "gauss_newton");
        assert_eq!(dense_report.linear_solver, "dense");
        assert_eq!(sparse_report.linear_solver, "block_tridiagonal");
        assert!(dense_report.final_cost < dense_report.initial_cost);
        assert!(sparse_report.final_cost < sparse_report.initial_cost);
        assert_close(sparse_report.final_cost, dense_report.final_cost, 1e-7);
        for (dense, sparse) in dense_problem.states().iter().zip(sparse_problem.states()) {
            assert_vec_close(sparse, dense, 1e-7);
        }
    }

    #[test]
    fn wnoa_block_tridiagonal_gauss_newton_matches_dense_solver() {
        let map = Arc::new(sloped_map());
        let states = vec![
            Vec4::from_row_slice(&[1.0, 2.0, 4.0, 0.0]),
            Vec4::from_row_slice(&[3.0, 2.0, 4.0, 0.0]),
            Vec4::from_row_slice(&[5.0, 2.0, 4.0, 0.0]),
        ];
        let mut dense_problem = WnoaBatchProblem::new(states.clone()).unwrap();
        let mut sparse_problem = WnoaBatchProblem::new(states).unwrap();
        for problem in [&mut dense_problem, &mut sparse_problem] {
            let strong = Vec4::from_row_slice(&[0.01, 0.1, 0.01, 0.1]);
            problem.add_prior(0, problem.states()[0], strong).unwrap();
            problem.add_prior(2, problem.states()[2], strong).unwrap();
            problem.add_gp_prior(0, 1, 1.0, 10.0).unwrap();
            problem.add_gp_prior(1, 2, 1.0, 10.0).unwrap();
            problem
                .add_interpolated_obstacle(0, 1, map.clone(), 0, 0.0, 10.0, 0.2, 10.0, 1.0, 0.5)
                .unwrap();
            problem
                .add_obstacle(1, map.clone(), 0, 0.0, 10.0, 0.2)
                .unwrap();
            problem
                .add_interpolated_obstacle(1, 2, map.clone(), 0, 0.0, 10.0, 0.2, 10.0, 1.0, 0.5)
                .unwrap();
        }

        let mut dense_config = WnojOptimizerConfig::default();
        dense_config.linear_solver = LinearSolverKind::Dense;
        dense_config.nonlinear_optimizer = NonlinearOptimizerKind::GaussNewton;
        let mut sparse_config = WnojOptimizerConfig::default();
        sparse_config.linear_solver = LinearSolverKind::BlockTridiagonal;
        sparse_config.nonlinear_optimizer = NonlinearOptimizerKind::GaussNewton;

        let dense_report = dense_problem.optimize(&dense_config).unwrap();
        let sparse_report = sparse_problem.optimize(&sparse_config).unwrap();

        assert_eq!(dense_report.nonlinear_optimizer, "gauss_newton");
        assert_eq!(sparse_report.nonlinear_optimizer, "gauss_newton");
        assert_eq!(dense_report.linear_solver, "dense");
        assert_eq!(sparse_report.linear_solver, "block_tridiagonal");
        assert!(dense_report.final_cost < dense_report.initial_cost);
        assert!(sparse_report.final_cost < sparse_report.initial_cost);
        assert_close(sparse_report.final_cost, dense_report.final_cost, 1e-7);
        for (dense, sparse) in dense_problem.states().iter().zip(sparse_problem.states()) {
            assert_vec_close(sparse, dense, 1e-7);
        }
    }

    #[test]
    fn wnoj_auto_sparse_solver_handles_non_chain_graph() {
        let states = (0..9)
            .map(|index| {
                let x = 1.0 + index as f64 * 0.45;
                let y = 2.0 + index as f64 * 0.35;
                Vec6::from_row_slice(&[x, 0.9, 0.0, y, 0.7, 0.0])
            })
            .collect::<Vec<_>>();
        let mut dense_problem = WnojBatchProblem::new(states.clone()).unwrap();
        let mut sparse_problem = WnojBatchProblem::new(states).unwrap();
        for problem in [&mut dense_problem, &mut sparse_problem] {
            let strong = Vec6::from_row_slice(&[0.05, 0.2, 1.0, 0.05, 0.2, 1.0]);
            problem.add_prior(0, problem.states()[0], strong).unwrap();
            problem.add_prior(8, problem.states()[8], strong).unwrap();
            for index in 0..8 {
                problem.add_gp_prior(index, index + 1, 1.0, 10.0).unwrap();
            }
            problem.add_gp_prior(0, 8, 8.0, 8.0).unwrap();
        }

        let mut dense_config = WnojOptimizerConfig::default();
        dense_config.linear_solver = LinearSolverKind::Dense;
        let mut sparse_config = WnojOptimizerConfig::default();
        sparse_config.linear_solver = LinearSolverKind::Auto;

        let dense_report = dense_problem.optimize(&dense_config).unwrap();
        let sparse_report = sparse_problem.optimize(&sparse_config).unwrap();

        assert_eq!(dense_report.linear_solver, "dense");
        assert_eq!(sparse_report.linear_solver, "sparse_cholesky");
        assert_eq!(sparse_report.linear_solve_fallbacks, 0);
        assert_close(sparse_report.final_cost, dense_report.final_cost, 1e-7);
        for (dense, sparse) in dense_problem.states().iter().zip(sparse_problem.states()) {
            assert_vec_close(sparse, dense, 1e-7);
        }
    }

    #[test]
    fn wnoa_sparse_solver_matches_dense_on_non_chain_graph() {
        let states = vec![
            Vec4::from_row_slice(&[1.0, 0.9, 2.0, 0.7]),
            Vec4::from_row_slice(&[1.4, 0.9, 2.3, 0.7]),
            Vec4::from_row_slice(&[1.9, 0.9, 2.7, 0.7]),
            Vec4::from_row_slice(&[2.5, 0.9, 3.2, 0.7]),
        ];
        let mut dense_problem = WnoaBatchProblem::new(states.clone()).unwrap();
        let mut sparse_problem = WnoaBatchProblem::new(states).unwrap();
        for problem in [&mut dense_problem, &mut sparse_problem] {
            let strong = Vec4::from_row_slice(&[0.05, 0.2, 0.05, 0.2]);
            problem.add_prior(0, problem.states()[0], strong).unwrap();
            problem.add_prior(3, problem.states()[3], strong).unwrap();
            problem.add_gp_prior(0, 1, 1.0, 10.0).unwrap();
            problem.add_gp_prior(1, 2, 1.0, 10.0).unwrap();
            problem.add_gp_prior(2, 3, 1.0, 10.0).unwrap();
            problem.add_gp_prior(0, 3, 3.0, 8.0).unwrap();
        }

        let mut dense_config = WnojOptimizerConfig::default();
        dense_config.linear_solver = LinearSolverKind::Dense;
        let mut sparse_config = WnojOptimizerConfig::default();
        sparse_config.linear_solver = LinearSolverKind::Sparse;

        let dense_report = dense_problem.optimize(&dense_config).unwrap();
        let sparse_report = sparse_problem.optimize(&sparse_config).unwrap();

        assert_eq!(dense_report.linear_solver, "dense");
        assert_eq!(sparse_report.linear_solver, "sparse_cholesky");
        assert_eq!(sparse_report.linear_solve_fallbacks, 0);
        assert_close(sparse_report.final_cost, dense_report.final_cost, 1e-7);
        for (dense, sparse) in dense_problem.states().iter().zip(sparse_problem.states()) {
            assert_vec_close(sparse, dense, 1e-7);
        }
    }

    #[test]
    fn wnoa_optimize_request_reduces_obstacle_cost() {
        let map = sloped_map();
        let request = WnoaOptimizeRequest {
            mode: Some("wnoa".to_string()),
            schema: Some("lingtu.pct_gpmp.optimize.request.v1".to_string()),
            states: vec![
                [1.0, 2.0, 4.0, 0.0],
                [3.0, 2.0, 4.0, 0.0],
                [5.0, 2.0, 4.0, 0.0],
            ],
            layers: Some(vec![0, 0, 0]),
            height_hints: Some(vec![0.0, 0.0, 0.0]),
            map: Some(DenseElevationMapRequest {
                resolution: 1.0,
                num_layers: 1,
                max_x: 8,
                max_y: 8,
                cost: map.cost.clone(),
                height: map.height.clone(),
                ceiling: map.ceiling.clone(),
            }),
            cost_threshold: Some(10.0),
            endpoint_prior_sigmas: Some([0.01, 0.1, 0.01, 0.1]),
            gp_qc: Some(10.0),
            delta: Some(1.0),
            obstacle_sigma: Some(0.2),
            interpolation_steps: Some(1),
            config: Some(WnojOptimizerConfig::default()),
        };

        let response = optimize_wnoa_request(request).unwrap();

        assert!(response.ok);
        assert_eq!(response.states.len(), 3);
        assert_eq!(response.trajectory_states.len(), 5);
        assert_eq!(response.trajectory_layers.len(), 5);
        assert_eq!(response.trajectory_heights.len(), 5);
        assert_eq!(response.trajectory_costs.len(), 5);
        assert_eq!(response.initial_trajectory_states.len(), 5);
        assert_eq!(response.initial_trajectory_layers.len(), 5);
        let report = response.report.unwrap();
        assert_eq!(report.nonlinear_optimizer, "levenberg_marquardt");
        assert_eq!(report.linear_solver, "block_tridiagonal");
        assert_eq!(report.linear_solve_fallbacks, 0);
        assert!(report.final_cost < report.initial_cost);
        assert!(response.states[1][2] < 4.0);
    }

    #[test]
    fn optimize_request_json_dispatches_wnoa() {
        let map = sloped_map();
        let request = WnoaOptimizeRequest {
            mode: Some("wnoa".to_string()),
            schema: Some("lingtu.pct_gpmp.optimize.request.v1".to_string()),
            states: vec![
                [1.0, 2.0, 4.0, 0.0],
                [3.0, 2.0, 4.0, 0.0],
                [5.0, 2.0, 4.0, 0.0],
            ],
            layers: Some(vec![0, 0, 0]),
            height_hints: Some(vec![0.0, 0.0, 0.0]),
            map: Some(DenseElevationMapRequest {
                resolution: 1.0,
                num_layers: 1,
                max_x: 8,
                max_y: 8,
                cost: map.cost.clone(),
                height: map.height.clone(),
                ceiling: map.ceiling.clone(),
            }),
            cost_threshold: Some(10.0),
            endpoint_prior_sigmas: Some([0.01, 0.1, 0.01, 0.1]),
            gp_qc: Some(10.0),
            delta: Some(1.0),
            obstacle_sigma: Some(0.2),
            interpolation_steps: Some(1),
            config: Some(WnojOptimizerConfig::default()),
        };
        let input = serde_json::to_string(&request).unwrap();

        let outcome = optimize_request_json(&input);
        let response: serde_json::Value = serde_json::from_str(&outcome.response_json).unwrap();

        assert_eq!(outcome.status_code, 0);
        assert_eq!(response["ok"], true);
        assert_eq!(response["states"].as_array().unwrap().len(), 3);
        assert_eq!(response["trajectory_states"].as_array().unwrap().len(), 5);
    }

    #[test]
    fn c_abi_optimize_json_returns_owned_response() {
        let input = br#"{"mode":"wnoa","schema":"lingtu.pct_gpmp.optimize.request.v1","states":[[0.0,1.0,0.0,0.0],[1.0,1.0,0.0,0.0]],"layers":[0,0],"height_hints":[0.0,0.0],"interpolation_steps":1}"#;
        let mut output_ptr: *mut u8 = std::ptr::null_mut();
        let mut output_len: usize = 0;

        let status = unsafe {
            lingtu_gpmp_optimizer_optimize_json(
                input.as_ptr(),
                input.len(),
                &mut output_ptr,
                &mut output_len,
            )
        };
        assert_eq!(status, 0);
        assert_eq!(
            lingtu_gpmp_optimizer_abi_version(),
            GPMP_OPTIMIZER_ABI_VERSION
        );
        assert!(!output_ptr.is_null());
        assert!(output_len > 0);

        let response = unsafe { std::slice::from_raw_parts(output_ptr, output_len) };
        let response: serde_json::Value = serde_json::from_slice(response).unwrap();
        unsafe { lingtu_gpmp_optimizer_free_json(output_ptr, output_len) };

        assert_eq!(response["ok"], true);
        assert_eq!(response["trajectory_states"].as_array().unwrap().len(), 3);
    }
}
