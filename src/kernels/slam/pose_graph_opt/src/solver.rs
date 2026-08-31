use std::collections::BTreeMap;

use faer::sparse::{SparseColMat, Triplet};
use faer::{prelude::Solve, Mat, Side};
use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, Vector3};

use crate::geometry::{residual_between, residual_prior, Pose3, Vector6};
use crate::{
    LT_POSE_GRAPH_OPT_EMPTY_GRAPH, LT_POSE_GRAPH_OPT_GAUGE_FREEDOM,
    LT_POSE_GRAPH_OPT_INVALID_CONFIG, LT_POSE_GRAPH_OPT_INVALID_INDEX,
    LT_POSE_GRAPH_OPT_INVALID_INFORMATION, LT_POSE_GRAPH_OPT_NON_FINITE_INPUT,
    LT_POSE_GRAPH_OPT_SINGULAR_SYSTEM,
};

pub type Information6 = SMatrix<f64, 6, 6>;
type Jacobian6 = SMatrix<f64, 6, 6>;
const SPARSE_LM_MIN_DIM: usize = 72;

#[derive(Clone, Debug, PartialEq)]
pub struct PriorFactor3 {
    pub index: usize,
    pub pose: Pose3,
    pub information: Information6,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BetweenFactor3 {
    pub from: usize,
    pub to: usize,
    pub measurement: Pose3,
    pub information: Information6,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct PoseGraph3 {
    pub poses: Vec<Pose3>,
    pub priors: Vec<PriorFactor3>,
    pub betweens: Vec<BetweenFactor3>,
}

impl PoseGraph3 {
    pub fn new(poses: Vec<Pose3>) -> Self {
        Self {
            poses,
            priors: Vec::new(),
            betweens: Vec::new(),
        }
    }

    pub fn add_prior(&mut self, index: usize, pose: Pose3, information: Information6) {
        self.priors.push(PriorFactor3 {
            index,
            pose,
            information,
        });
    }

    pub fn add_between(
        &mut self,
        from: usize,
        to: usize,
        measurement: Pose3,
        information: Information6,
    ) {
        self.betweens.push(BetweenFactor3 {
            from,
            to,
            measurement,
            information,
        });
    }

    fn factor_count(&self) -> usize {
        self.priors.len() + self.betweens.len()
    }
}

#[derive(Clone, Debug, PartialEq)]
struct LinearizedSystem {
    normal: BlockSparseNormal,
    gradient: DVector<f64>,
}

#[derive(Clone, Debug, PartialEq)]
struct BlockSparseNormal {
    dim: usize,
    upper: BTreeMap<(usize, usize), f64>,
}

impl BlockSparseNormal {
    fn new(dim: usize) -> Self {
        Self {
            dim,
            upper: BTreeMap::new(),
        }
    }

    fn add_symmetric_entry(&mut self, row: usize, col: usize, value: f64) {
        if value == 0.0 {
            return;
        }
        let key = if row <= col { (row, col) } else { (col, row) };
        *self.upper.entry(key).or_insert(0.0) += value;
    }

    fn diagonal(&self, index: usize) -> f64 {
        self.upper.get(&(index, index)).copied().unwrap_or(0.0)
    }

    fn to_dense_damped(&self, lambda: f64) -> DMatrix<f64> {
        let mut dense = DMatrix::<f64>::zeros(self.dim, self.dim);
        for (&(row, col), &value) in &self.upper {
            dense[(row, col)] += value;
            if row != col {
                dense[(col, row)] += value;
            }
        }
        for idx in 0..self.dim {
            dense[(idx, idx)] += damping_for_diagonal(self.diagonal(idx), lambda);
        }
        dense
    }

    fn to_sparse_upper_damped(&self, lambda: f64) -> Option<SparseColMat<usize, f64>> {
        let mut triplets = Vec::with_capacity(self.upper.len() + self.dim);
        for (&(row, col), &value) in &self.upper {
            triplets.push(Triplet::new(row, col, value));
        }
        for idx in 0..self.dim {
            triplets.push(Triplet::new(
                idx,
                idx,
                damping_for_diagonal(self.diagonal(idx), lambda),
            ));
        }
        SparseColMat::<usize, f64>::try_new_from_triplets(self.dim, self.dim, &triplets).ok()
    }

    fn mul_damped(&self, vector: &DVector<f64>, lambda: f64) -> DVector<f64> {
        let mut out = DVector::<f64>::zeros(self.dim);
        for (&(row, col), &value) in &self.upper {
            out[row] += value * vector[col];
            if row != col {
                out[col] += value * vector[row];
            }
        }
        for idx in 0..self.dim {
            out[idx] += damping_for_diagonal(self.diagonal(idx), lambda) * vector[idx];
        }
        out
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OptimizerConfig {
    pub max_iterations: usize,
    pub initial_lambda: f64,
    pub tolerance: f64,
    pub numeric_epsilon: f64,
    pub fixed_pose_index: Option<usize>,
    pub auto_anchor: bool,
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self {
            max_iterations: 30,
            initial_lambda: 1e-3,
            tolerance: 1e-9,
            numeric_epsilon: 1e-6,
            fixed_pose_index: Some(0),
            auto_anchor: true,
        }
    }
}

impl OptimizerConfig {
    pub fn normalized(self) -> Result<Self, OptimizeError> {
        if self.fixed_pose_index == Some(usize::MAX) {
            return Err(OptimizeError::InvalidConfig);
        }
        Ok(Self {
            max_iterations: if self.max_iterations == 0 {
                Self::default().max_iterations
            } else {
                self.max_iterations.min(200)
            },
            initial_lambda: finite_positive_or(self.initial_lambda, Self::default().initial_lambda),
            tolerance: finite_positive_or(self.tolerance, Self::default().tolerance),
            numeric_epsilon: finite_positive_or(
                self.numeric_epsilon,
                Self::default().numeric_epsilon,
            ),
            fixed_pose_index: self.fixed_pose_index,
            auto_anchor: self.auto_anchor,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OptimizerReport {
    pub iterations: usize,
    pub accepted_steps: usize,
    pub rejected_steps: usize,
    pub converged: bool,
    pub initial_cost: f64,
    pub final_cost: f64,
}

impl Default for OptimizerReport {
    fn default() -> Self {
        Self {
            iterations: 0,
            accepted_steps: 0,
            rejected_steps: 0,
            converged: false,
            initial_cost: f64::NAN,
            final_cost: f64::NAN,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OptimizeError {
    EmptyGraph,
    InvalidIndex,
    NonFiniteInput,
    SingularSystem,
    InvalidConfig,
    GaugeFreedom,
    InvalidInformation,
}

impl OptimizeError {
    pub fn code(self) -> i32 {
        match self {
            Self::EmptyGraph => LT_POSE_GRAPH_OPT_EMPTY_GRAPH,
            Self::InvalidIndex => LT_POSE_GRAPH_OPT_INVALID_INDEX,
            Self::NonFiniteInput => LT_POSE_GRAPH_OPT_NON_FINITE_INPUT,
            Self::SingularSystem => LT_POSE_GRAPH_OPT_SINGULAR_SYSTEM,
            Self::InvalidConfig => LT_POSE_GRAPH_OPT_INVALID_CONFIG,
            Self::GaugeFreedom => LT_POSE_GRAPH_OPT_GAUGE_FREEDOM,
            Self::InvalidInformation => LT_POSE_GRAPH_OPT_INVALID_INFORMATION,
        }
    }
}

pub fn information_from_upper(upper: &[f64; 21]) -> Result<Information6, OptimizeError> {
    if !upper.iter().all(|value| value.is_finite()) {
        return Err(OptimizeError::NonFiniteInput);
    }
    let mut matrix = Information6::zeros();
    let mut idx = 0usize;
    for row in 0..6 {
        for col in row..6 {
            matrix[(row, col)] = upper[idx];
            matrix[(col, row)] = upper[idx];
            idx += 1;
        }
    }
    validate_information(&matrix)?;
    Ok(matrix)
}

pub fn information_to_upper(information: &Information6) -> [f64; 21] {
    let mut upper = [0.0; 21];
    let mut idx = 0usize;
    for row in 0..6 {
        for col in row..6 {
            upper[idx] = 0.5 * (information[(row, col)] + information[(col, row)]);
            idx += 1;
        }
    }
    upper
}

pub fn diagonal_information(values: [f64; 6]) -> Information6 {
    let mut information = Information6::zeros();
    for idx in 0..6 {
        information[(idx, idx)] = values[idx];
    }
    information
}

pub fn optimize_pose_graph3(
    graph: &mut PoseGraph3,
    config: OptimizerConfig,
) -> Result<OptimizerReport, OptimizeError> {
    validate_graph(graph)?;

    if graph.factor_count() == 0 {
        return Ok(OptimizerReport {
            initial_cost: 0.0,
            final_cost: 0.0,
            converged: true,
            ..OptimizerReport::default()
        });
    }

    let config = config.normalized()?;
    let fixed_pose = resolved_fixed_pose(graph, &config)?;
    ensure_components_anchored(graph, fixed_pose)?;
    let variable_columns = variable_columns(graph.poses.len(), fixed_pose);
    let dim = variable_columns
        .iter()
        .filter(|entry| entry.is_some())
        .count()
        * 6;

    let mut poses = graph.poses.clone();
    let mut cost = cost_for_poses(&poses, graph);
    let initial_cost = cost;
    let mut lambda = config.initial_lambda;
    let mut report = OptimizerReport {
        initial_cost,
        final_cost: cost,
        ..OptimizerReport::default()
    };

    if dim == 0 {
        report.converged = true;
        return Ok(report);
    }

    for iteration in 0..config.max_iterations {
        let system = linearized_normal_equations(
            &poses,
            graph,
            &variable_columns,
            dim,
            config.numeric_epsilon,
        );
        let gradient_max = system
            .gradient
            .iter()
            .fold(0.0_f64, |acc, value| acc.max(value.abs()));
        if gradient_max <= config.tolerance {
            report.iterations = iteration;
            report.converged = true;
            break;
        }

        let delta = solve_lm_step(&system, lambda).ok_or(OptimizeError::SingularSystem)?;
        let delta_norm = delta
            .iter()
            .fold(0.0_f64, |acc, value| acc.max(value.abs()));
        if delta_norm <= config.tolerance {
            report.iterations = iteration;
            report.converged = true;
            break;
        }

        let trial_poses = apply_delta(&poses, &variable_columns, &delta);
        let trial_cost = cost_for_poses(&trial_poses, graph);
        report.iterations = iteration + 1;

        if trial_cost.is_finite() && trial_cost < cost {
            let reduction = cost - trial_cost;
            poses = trial_poses;
            cost = trial_cost;
            report.final_cost = cost;
            report.accepted_steps += 1;
            lambda = (lambda * 0.5).max(1e-12);
            if reduction <= config.tolerance * (1.0 + initial_cost.abs()) {
                report.converged = true;
                break;
            }
        } else {
            report.rejected_steps += 1;
            lambda = (lambda * 10.0).min(1e12);
        }
    }

    graph.poses = poses;
    report.final_cost = cost;
    Ok(report)
}

fn finite_positive_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        fallback
    }
}

fn validate_graph(graph: &PoseGraph3) -> Result<(), OptimizeError> {
    if graph.poses.is_empty() {
        return Err(OptimizeError::EmptyGraph);
    }
    if graph.poses.iter().any(|pose| !pose.is_finite()) {
        return Err(OptimizeError::NonFiniteInput);
    }
    for prior in &graph.priors {
        if prior.index >= graph.poses.len() {
            return Err(OptimizeError::InvalidIndex);
        }
        if !prior.pose.is_finite() {
            return Err(OptimizeError::NonFiniteInput);
        }
        validate_information(&prior.information)?;
    }
    for between in &graph.betweens {
        if between.from >= graph.poses.len() || between.to >= graph.poses.len() {
            return Err(OptimizeError::InvalidIndex);
        }
        if !between.measurement.is_finite() {
            return Err(OptimizeError::NonFiniteInput);
        }
        validate_information(&between.information)?;
    }
    Ok(())
}

fn matrix_is_finite(matrix: &Information6) -> bool {
    matrix.iter().all(|value| value.is_finite())
}

fn validate_information(information: &Information6) -> Result<(), OptimizeError> {
    if !matrix_is_finite(information) {
        return Err(OptimizeError::NonFiniteInput);
    }

    let scale = information
        .iter()
        .fold(0.0_f64, |maximum, value| maximum.max(value.abs()));
    if scale == 0.0 {
        return Err(OptimizeError::InvalidInformation);
    }

    let tolerance = 1e-10 * scale;
    for row in 0..6 {
        for col in (row + 1)..6 {
            if (information[(row, col)] - information[(col, row)]).abs() > tolerance {
                return Err(OptimizeError::InvalidInformation);
            }
        }
    }

    let symmetric = 0.5 * (information + information.transpose());
    if symmetric
        .symmetric_eigen()
        .eigenvalues
        .iter()
        .any(|eigenvalue| *eigenvalue < -tolerance)
    {
        return Err(OptimizeError::InvalidInformation);
    }
    Ok(())
}

fn resolved_fixed_pose(
    graph: &PoseGraph3,
    config: &OptimizerConfig,
) -> Result<Option<usize>, OptimizeError> {
    if let Some(index) = config.fixed_pose_index {
        if index >= graph.poses.len() {
            return Err(OptimizeError::InvalidIndex);
        }
        return Ok(Some(index));
    }
    if graph.priors.is_empty() {
        if config.auto_anchor {
            Ok(Some(0))
        } else {
            Err(OptimizeError::GaugeFreedom)
        }
    } else {
        Ok(None)
    }
}

fn ensure_components_anchored(
    graph: &PoseGraph3,
    fixed_pose: Option<usize>,
) -> Result<(), OptimizeError> {
    let mut anchored = vec![false; graph.poses.len()];
    if let Some(index) = fixed_pose {
        anchored[index] = true;
    }
    for prior in &graph.priors {
        anchored[prior.index] = true;
    }

    let mut adjacency = vec![Vec::<usize>::new(); graph.poses.len()];
    for between in &graph.betweens {
        adjacency[between.from].push(between.to);
        adjacency[between.to].push(between.from);
    }

    let mut visited = vec![false; graph.poses.len()];
    for start in 0..graph.poses.len() {
        if visited[start] {
            continue;
        }
        let mut stack = vec![start];
        let mut has_anchor = anchored[start];
        visited[start] = true;
        while let Some(node) = stack.pop() {
            has_anchor |= anchored[node];
            for &neighbor in &adjacency[node] {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    stack.push(neighbor);
                }
            }
        }
        if !has_anchor {
            return Err(OptimizeError::GaugeFreedom);
        }
    }
    Ok(())
}

fn variable_columns(pose_count: usize, fixed_pose: Option<usize>) -> Vec<Option<usize>> {
    let mut columns = Vec::with_capacity(pose_count);
    let mut next_col = 0usize;
    for idx in 0..pose_count {
        if Some(idx) == fixed_pose {
            columns.push(None);
        } else {
            columns.push(Some(next_col));
            next_col += 6;
        }
    }
    columns
}

fn residual_blocks_for_poses(poses: &[Pose3], graph: &PoseGraph3) -> Vec<Vector6> {
    let mut residuals = Vec::with_capacity(graph.factor_count());
    for prior in &graph.priors {
        residuals.push(residual_prior(prior.pose, poses[prior.index]));
    }
    for between in &graph.betweens {
        residuals.push(residual_between(
            between.measurement,
            poses[between.from],
            poses[between.to],
        ));
    }
    residuals
}

fn cost_for_poses(poses: &[Pose3], graph: &PoseGraph3) -> f64 {
    let residuals = residual_blocks_for_poses(poses, graph);
    let mut cost = 0.0;
    for (factor_idx, residual) in residuals.iter().enumerate() {
        let information = factor_information(graph, factor_idx);
        cost += 0.5 * residual.dot(&(information * residual));
    }
    cost
}

fn linearized_normal_equations(
    poses: &[Pose3],
    graph: &PoseGraph3,
    variable_columns: &[Option<usize>],
    dim: usize,
    epsilon: f64,
) -> LinearizedSystem {
    let mut normal = BlockSparseNormal::new(dim);
    let mut gradient = DVector::<f64>::zeros(dim);

    for prior in &graph.priors {
        let residual = residual_prior(prior.pose, poses[prior.index]);
        let mut blocks = Vec::with_capacity(1);
        if let Some(column_start) = variable_columns[prior.index] {
            blocks.push((column_start, prior_jacobian(&residual, epsilon)));
        }
        accumulate_factor(
            &mut normal,
            &mut gradient,
            &prior.information,
            &residual,
            &blocks,
        );
    }

    for between in &graph.betweens {
        let from = poses[between.from];
        let to = poses[between.to];
        let residual = residual_between(between.measurement, from, to);
        let predicted = from.between(to);
        let (from_jacobian, to_jacobian) = between_jacobians(predicted, &residual, epsilon);
        let mut blocks = Vec::with_capacity(2);
        if let Some(column_start) = variable_columns[between.from] {
            blocks.push((column_start, from_jacobian));
        }
        if let Some(column_start) = variable_columns[between.to] {
            blocks.push((column_start, to_jacobian));
        }
        accumulate_factor(
            &mut normal,
            &mut gradient,
            &between.information,
            &residual,
            &blocks,
        );
    }

    LinearizedSystem { normal, gradient }
}

fn prior_jacobian(residual: &Vector6, epsilon: f64) -> Jacobian6 {
    local_right_jacobian_inverse(residual, epsilon)
}

fn between_jacobians(
    predicted_between: Pose3,
    residual: &Vector6,
    epsilon: f64,
) -> (Jacobian6, Jacobian6) {
    let residual_right_jacobian_inverse = local_right_jacobian_inverse(residual, epsilon);
    let from_jacobian =
        -residual_right_jacobian_inverse * pose_adjoint(predicted_between.inverse());
    let to_jacobian = residual_right_jacobian_inverse;
    (from_jacobian, to_jacobian)
}

fn local_right_jacobian_inverse(residual: &Vector6, epsilon: f64) -> Jacobian6 {
    if residual.norm() <= 1e-12 {
        return Jacobian6::identity();
    }
    let residual_pose = Pose3::exp(residual);
    numeric_jacobian6(epsilon, |delta| {
        residual_pose.compose(Pose3::exp(delta)).log()
    })
}

fn pose_adjoint(transform: Pose3) -> Jacobian6 {
    let rotation = transform.rotation().to_rotation_matrix().into_inner();
    let translation = transform.translation();
    let translation_cross_rotation = skew3(&translation) * rotation;

    let mut adjoint = Jacobian6::zeros();
    adjoint.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
    adjoint
        .fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&translation_cross_rotation);
    adjoint.fixed_view_mut::<3, 3>(3, 3).copy_from(&rotation);
    adjoint
}

fn skew3(vector: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0,
    )
}

fn numeric_jacobian6(epsilon: f64, residual_fn: impl Fn(&Vector6) -> Vector6) -> Jacobian6 {
    let mut jacobian = Jacobian6::zeros();
    for local_dim in 0..6 {
        let mut delta = Vector6::zeros();
        delta[local_dim] = epsilon;
        let plus_residual = residual_fn(&delta);
        delta[local_dim] = -epsilon;
        let minus_residual = residual_fn(&delta);
        for row in 0..6 {
            jacobian[(row, local_dim)] =
                (plus_residual[row] - minus_residual[row]) / (2.0 * epsilon);
        }
    }
    jacobian
}

fn accumulate_factor(
    normal: &mut BlockSparseNormal,
    gradient: &mut DVector<f64>,
    information: &Information6,
    residual: &Vector6,
    blocks: &[(usize, Jacobian6)],
) {
    let weighted_residual = information * residual;
    for (column_a, jacobian_a) in blocks {
        for local_a in 0..6 {
            gradient[*column_a + local_a] += jacobian_a.column(local_a).dot(&weighted_residual);
        }

        for (column_b, jacobian_b) in blocks {
            if *column_a > *column_b {
                continue;
            }
            let normal_block = jacobian_a.transpose() * information * jacobian_b;
            for local_a in 0..6 {
                let local_b_start = if column_a == column_b { local_a } else { 0 };
                for local_b in local_b_start..6 {
                    normal.add_symmetric_entry(
                        *column_a + local_a,
                        *column_b + local_b,
                        normal_block[(local_a, local_b)],
                    );
                }
            }
        }
    }
}

fn factor_information(graph: &PoseGraph3, factor_idx: usize) -> Information6 {
    if factor_idx < graph.priors.len() {
        graph.priors[factor_idx].information
    } else {
        graph.betweens[factor_idx - graph.priors.len()].information
    }
}

fn damping_for_diagonal(diagonal: f64, lambda: f64) -> f64 {
    lambda * diagonal.abs().max(1.0)
}

fn solve_lm_step(system: &LinearizedSystem, lambda: f64) -> Option<DVector<f64>> {
    if system.normal.dim >= SPARSE_LM_MIN_DIM {
        if let Some(delta) = solve_sparse_lm_step(system, lambda) {
            return Some(delta);
        }
    }
    solve_dense_lm_step(system, lambda)
}

fn solve_sparse_lm_step(system: &LinearizedSystem, lambda: f64) -> Option<DVector<f64>> {
    let sparse = system.normal.to_sparse_upper_damped(lambda)?;
    let llt = sparse.sp_cholesky(Side::Upper).ok()?;
    let mut rhs_values = Vec::with_capacity(system.gradient.len());
    rhs_values.extend(system.gradient.iter().map(|value| -value));
    let mut rhs = Mat::<f64>::from_fn(system.gradient.len(), 1, |row, _| rhs_values[row]);
    llt.solve_in_place(rhs.as_mut());

    let mut delta = DVector::<f64>::zeros(system.gradient.len());
    for row in 0..system.gradient.len() {
        delta[row] = rhs[(row, 0)];
    }
    validate_lm_solution(system, &delta, lambda).then_some(delta)
}

fn solve_dense_lm_step(system: &LinearizedSystem, lambda: f64) -> Option<DVector<f64>> {
    let damped = system.normal.to_dense_damped(lambda);
    let rhs = -&system.gradient;
    let mut faer_matrix = Mat::<f64>::zeros(damped.nrows(), damped.ncols());
    for row in 0..damped.nrows() {
        for col in 0..damped.ncols() {
            faer_matrix[(row, col)] = damped[(row, col)];
        }
    }
    let mut faer_rhs = Mat::<f64>::zeros(rhs.len(), 1);
    for row in 0..rhs.len() {
        faer_rhs[(row, 0)] = rhs[row];
    }

    faer_matrix
        .partial_piv_lu()
        .solve_in_place(faer_rhs.as_mut());

    let mut delta = DVector::<f64>::zeros(rhs.len());
    for row in 0..rhs.len() {
        delta[row] = faer_rhs[(row, 0)];
    }
    if !delta.iter().all(|value| value.is_finite()) {
        return None;
    }

    validate_lm_solution(system, &delta, lambda).then_some(delta)
}

fn validate_lm_solution(system: &LinearizedSystem, delta: &DVector<f64>, lambda: f64) -> bool {
    if !delta.iter().all(|value| value.is_finite()) {
        return false;
    }

    let rhs = -&system.gradient;
    let residual = system.normal.mul_damped(delta, lambda) - &rhs;
    let residual_max = residual
        .iter()
        .fold(0.0_f64, |acc, value| acc.max(value.abs()));
    let rhs_scale = rhs.iter().fold(1.0_f64, |acc, value| acc.max(value.abs()));
    if !residual_max.is_finite() || residual_max > 1e-6 * rhs_scale {
        return false;
    }
    true
}

fn apply_delta(
    poses: &[Pose3],
    variable_columns: &[Option<usize>],
    delta: &DVector<f64>,
) -> Vec<Pose3> {
    let mut updated = poses.to_vec();
    for (pose_idx, column_start) in variable_columns.iter().enumerate() {
        let Some(column_start) = *column_start else {
            continue;
        };
        let local_delta = Vector6::from_row_slice(&[
            delta[column_start],
            delta[column_start + 1],
            delta[column_start + 2],
            delta[column_start + 3],
            delta[column_start + 4],
            delta[column_start + 5],
        ]);
        updated[pose_idx] = updated[pose_idx].retract(&local_delta);
    }
    updated
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Pose3;

    fn near(a: f64, b: f64, tolerance: f64) {
        assert!(
            (a - b).abs() <= tolerance,
            "expected {a} to be within {tolerance} of {b}"
        );
    }

    fn assert_matrix_near(actual: &Jacobian6, expected: &Jacobian6, tolerance: f64) {
        for row in 0..6 {
            for col in 0..6 {
                assert!(
                    (actual[(row, col)] - expected[(row, col)]).abs() <= tolerance,
                    "entry ({row}, {col}): actual {}, expected {}, tolerance {}",
                    actual[(row, col)],
                    expected[(row, col)],
                    tolerance
                );
            }
        }
    }

    fn strong_info() -> Information6 {
        diagonal_information([100.0, 100.0, 100.0, 100.0, 100.0, 100.0])
    }

    fn medium_info() -> Information6 {
        diagonal_information([10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
    }

    fn chain_graph(pose_count: usize) -> PoseGraph3 {
        let mut poses = Vec::with_capacity(pose_count);
        for idx in 0..pose_count {
            poses.push(Pose3::exp(&Vector6::from_row_slice(&[
                0.0,
                0.0,
                0.001 * idx as f64,
                idx as f64 * 1.02,
                0.01 * idx as f64,
                0.0,
            ])));
        }

        let mut graph = PoseGraph3::new(poses);
        graph.add_prior(0, Pose3::identity(), strong_info());
        let odom = Pose3::exp(&Vector6::from_row_slice(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0]));
        for idx in 1..pose_count {
            graph.add_between(idx - 1, idx, odom, medium_info());
        }
        graph
    }

    fn factor_numeric_prior_jacobian(pose: Pose3, measurement: Pose3, epsilon: f64) -> Jacobian6 {
        numeric_jacobian6(epsilon, |delta| {
            residual_prior(measurement, pose.retract(delta))
        })
    }

    fn factor_numeric_between_jacobian_for_from(
        measurement: Pose3,
        from: Pose3,
        to: Pose3,
        epsilon: f64,
    ) -> Jacobian6 {
        numeric_jacobian6(epsilon, |delta| {
            residual_between(measurement, from.retract(delta), to)
        })
    }

    fn factor_numeric_between_jacobian_for_to(
        measurement: Pose3,
        from: Pose3,
        to: Pose3,
        epsilon: f64,
    ) -> Jacobian6 {
        numeric_jacobian6(epsilon, |delta| {
            residual_between(measurement, from, to.retract(delta))
        })
    }

    #[test]
    fn prior_local_jacobian_matches_factor_finite_difference() {
        let measurement = Pose3::exp(&Vector6::from_row_slice(&[
            0.08, -0.04, 0.03, 1.2, -0.5, 0.3,
        ]));
        let pose = measurement.compose(Pose3::exp(&Vector6::from_row_slice(&[
            0.03, -0.02, 0.015, 0.25, -0.1, 0.08,
        ])));
        let epsilon = 1e-6;
        let residual = residual_prior(measurement, pose);

        let local = prior_jacobian(&residual, epsilon);
        let finite_difference = factor_numeric_prior_jacobian(pose, measurement, epsilon);

        assert_matrix_near(&local, &finite_difference, 1e-6);
    }

    #[test]
    fn between_local_jacobians_match_factor_finite_difference() {
        let from = Pose3::exp(&Vector6::from_row_slice(&[
            0.15, -0.06, 0.2, 2.0, -0.7, 0.4,
        ]));
        let predicted_between = Pose3::exp(&Vector6::from_row_slice(&[
            -0.05, 0.08, 0.18, 1.1, 0.35, -0.2,
        ]));
        let to = from.compose(predicted_between);
        let measurement = Pose3::exp(&Vector6::from_row_slice(&[
            -0.04, 0.06, 0.16, 1.0, 0.28, -0.12,
        ]));
        let epsilon = 1e-6;
        let residual = residual_between(measurement, from, to);

        let (from_local, to_local) = between_jacobians(predicted_between, &residual, epsilon);
        let from_finite_difference =
            factor_numeric_between_jacobian_for_from(measurement, from, to, epsilon);
        let to_finite_difference =
            factor_numeric_between_jacobian_for_to(measurement, from, to, epsilon);

        assert_matrix_near(&from_local, &from_finite_difference, 2e-5);
        assert_matrix_near(&to_local, &to_finite_difference, 2e-5);
    }

    #[test]
    fn zero_residual_local_jacobians_match_se3_invariants() {
        let measurement = Pose3::exp(&Vector6::from_row_slice(&[
            0.1, -0.07, 0.18, 1.4, -0.2, 0.5,
        ]));
        let residual = Vector6::zeros();

        let prior = prior_jacobian(&residual, 1e-6);
        assert_matrix_near(&prior, &Jacobian6::identity(), 1e-12);

        let (from_jacobian, to_jacobian) = between_jacobians(measurement, &residual, 1e-6);
        let expected_from = -pose_adjoint(measurement.inverse());
        assert_matrix_near(&from_jacobian, &expected_from, 1e-12);
        assert_matrix_near(&to_jacobian, &Jacobian6::identity(), 1e-12);
    }

    #[test]
    fn prior_and_between_zero_residual_converge() {
        let target_delta = Vector6::from_row_slice(&[0.0, 0.0, 0.1, 1.0, 0.0, 0.0]);
        let mut graph = PoseGraph3::new(vec![
            Pose3::exp(&Vector6::from_row_slice(&[
                0.02, -0.01, 0.03, 0.1, -0.1, 0.0,
            ])),
            Pose3::exp(&Vector6::from_row_slice(&[0.01, 0.0, 0.2, 1.4, 0.2, 0.1])),
        ]);
        graph.add_prior(0, Pose3::identity(), strong_info());
        graph.add_between(0, 1, Pose3::exp(&target_delta), medium_info());

        let report = optimize_pose_graph3(&mut graph, OptimizerConfig::default()).unwrap();

        assert!(report.final_cost < report.initial_cost);
        let residual = residual_between(Pose3::exp(&target_delta), graph.poses[0], graph.poses[1]);
        for idx in 0..6 {
            near(residual[idx], 0.0, 1e-5);
        }
    }

    #[test]
    fn loop_closure_reduces_error() {
        let true_0 = Pose3::identity();
        let true_1 = true_0.compose(Pose3::exp(&Vector6::from_row_slice(&[
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        ])));
        let true_2 = true_1.compose(Pose3::exp(&Vector6::from_row_slice(&[
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_2,
            0.0,
            1.0,
            0.0,
        ])));
        let mut graph = PoseGraph3::new(vec![
            Pose3::identity(),
            Pose3::exp(&Vector6::from_row_slice(&[0.0, 0.0, 0.0, 1.2, 0.1, 0.0])),
            Pose3::exp(&Vector6::from_row_slice(&[0.0, 0.0, 1.65, 1.1, 1.3, 0.0])),
        ]);
        graph.add_prior(0, Pose3::identity(), strong_info());
        graph.add_between(0, 1, true_0.between(true_1), medium_info());
        graph.add_between(1, 2, true_1.between(true_2), medium_info());
        graph.add_between(0, 2, true_0.between(true_2), strong_info());

        let report = optimize_pose_graph3(&mut graph, OptimizerConfig::default()).unwrap();

        assert!(report.final_cost < report.initial_cost * 0.01);
    }

    #[test]
    fn optimize_large_rotated_chain_reduces_cost_on_sparse_path() {
        let pose_count = 14;
        let odom = Pose3::exp(&Vector6::from_row_slice(&[
            0.015, -0.01, 0.075, 0.9, 0.12, 0.04,
        ]));
        let mut truth = Vec::with_capacity(pose_count);
        truth.push(Pose3::identity());
        for idx in 1..pose_count {
            truth.push(truth[idx - 1].compose(odom));
        }

        let mut initial = Vec::with_capacity(pose_count);
        for (idx, pose) in truth.iter().enumerate() {
            let scale = idx as f64;
            let drift = Vector6::from_row_slice(&[
                0.001 * scale,
                -0.0008 * scale,
                0.004 * scale,
                0.025 * scale,
                -0.015 * scale,
                0.004 * scale,
            ]);
            initial.push(pose.compose(Pose3::exp(&drift)));
        }
        initial[0] = Pose3::identity();

        let mut graph = PoseGraph3::new(initial);
        graph.add_prior(0, Pose3::identity(), strong_info());
        for idx in 1..pose_count {
            graph.add_between(
                idx - 1,
                idx,
                truth[idx - 1].between(truth[idx]),
                medium_info(),
            );
        }
        graph.add_between(
            0,
            pose_count - 1,
            truth[0].between(truth[pose_count - 1]),
            strong_info(),
        );

        let variable_columns = variable_columns(graph.poses.len(), Some(0));
        let dim = variable_columns
            .iter()
            .filter(|entry| entry.is_some())
            .count()
            * 6;
        assert!(dim >= SPARSE_LM_MIN_DIM);
        let initial_cost = cost_for_poses(&graph.poses, &graph);

        let report = optimize_pose_graph3(
            &mut graph,
            OptimizerConfig {
                max_iterations: 80,
                ..OptimizerConfig::default()
            },
        )
        .unwrap();

        assert!(report.accepted_steps > 0);
        assert!(report.final_cost < initial_cost * 1e-6);
        for idx in 1..pose_count {
            let residual = residual_between(
                truth[idx - 1].between(truth[idx]),
                graph.poses[idx - 1],
                graph.poses[idx],
            );
            for residual_idx in 0..6 {
                near(residual[residual_idx], 0.0, 1e-5);
            }
        }
    }

    #[test]
    fn sparse_lm_step_solves_large_chain_system() {
        let graph = chain_graph(16);
        let fixed_pose = Some(0);
        let variable_columns = variable_columns(graph.poses.len(), fixed_pose);
        let dim = variable_columns
            .iter()
            .filter(|entry| entry.is_some())
            .count()
            * 6;

        assert!(dim >= SPARSE_LM_MIN_DIM);
        let system = linearized_normal_equations(
            &graph.poses,
            &graph,
            &variable_columns,
            dim,
            OptimizerConfig::default().numeric_epsilon,
        );
        let lambda = OptimizerConfig::default().initial_lambda;

        let delta =
            solve_sparse_lm_step(&system, lambda).expect("sparse Cholesky should solve graph");
        assert_eq!(delta.len(), dim);
        assert!(validate_lm_solution(&system, &delta, lambda));
    }

    #[test]
    fn sparse_and_dense_lm_steps_match_on_chain_system() {
        let graph = chain_graph(14);
        let fixed_pose = Some(0);
        let variable_columns = variable_columns(graph.poses.len(), fixed_pose);
        let dim = variable_columns
            .iter()
            .filter(|entry| entry.is_some())
            .count()
            * 6;
        let system = linearized_normal_equations(
            &graph.poses,
            &graph,
            &variable_columns,
            dim,
            OptimizerConfig::default().numeric_epsilon,
        );
        let lambda = OptimizerConfig::default().initial_lambda;

        let sparse = solve_sparse_lm_step(&system, lambda).unwrap();
        let dense = solve_dense_lm_step(&system, lambda).unwrap();
        for idx in 0..dim {
            near(sparse[idx], dense[idx], 1e-6);
        }
    }

    #[test]
    fn chain_linearization_keeps_sparse_structure() {
        let graph = chain_graph(50);
        let fixed_pose = Some(0);
        let variable_columns = variable_columns(graph.poses.len(), fixed_pose);
        let dim = variable_columns
            .iter()
            .filter(|entry| entry.is_some())
            .count()
            * 6;
        let system = linearized_normal_equations(
            &graph.poses,
            &graph,
            &variable_columns,
            dim,
            OptimizerConfig::default().numeric_epsilon,
        );

        let dense_upper_entries = dim * (dim + 1) / 2;
        assert!(system.normal.upper.len() < dense_upper_entries / 10);
        assert!(system.normal.upper.len() <= graph.factor_count() * 72);
    }

    #[test]
    fn full_information_matrix_is_accepted() {
        let mut info = medium_info();
        info[(3, 4)] = 1.0;
        info[(4, 3)] = 1.0;
        let upper = information_to_upper(&info);
        assert_eq!(information_from_upper(&upper).unwrap(), info);
    }

    #[test]
    fn rank_four_information_matrix_is_accepted() {
        let info = diagonal_information([10.0, 0.0, 5.0, 3.0, 0.0, 2.0]);
        let upper = information_to_upper(&info);
        assert_eq!(information_from_upper(&upper).unwrap(), info);
    }

    #[test]
    fn zero_information_matrix_is_rejected() {
        assert_eq!(
            information_from_upper(&[0.0; 21]).unwrap_err(),
            OptimizeError::InvalidInformation
        );
    }

    #[test]
    fn negative_information_diagonal_is_rejected() {
        let info = diagonal_information([10.0, 10.0, -1.0, 10.0, 10.0, 10.0]);
        assert_eq!(
            information_from_upper(&information_to_upper(&info)).unwrap_err(),
            OptimizeError::InvalidInformation
        );
    }

    #[test]
    fn indefinite_information_with_positive_diagonal_is_rejected() {
        let mut info = diagonal_information([1.0; 6]);
        info[(0, 1)] = 2.0;
        info[(1, 0)] = 2.0;
        assert_eq!(
            information_from_upper(&information_to_upper(&info)).unwrap_err(),
            OptimizeError::InvalidInformation
        );
    }

    #[test]
    fn unanchored_graph_reports_gauge_freedom() {
        let mut graph = PoseGraph3::new(vec![Pose3::identity(), Pose3::identity()]);
        graph.add_between(0, 1, Pose3::identity(), medium_info());

        let config = OptimizerConfig {
            fixed_pose_index: None,
            auto_anchor: false,
            ..OptimizerConfig::default()
        };

        assert_eq!(
            optimize_pose_graph3(&mut graph, config).unwrap_err(),
            OptimizeError::GaugeFreedom
        );
    }

    #[test]
    fn disconnected_component_reports_gauge_freedom() {
        let mut graph = PoseGraph3::new(vec![
            Pose3::identity(),
            Pose3::identity(),
            Pose3::identity(),
        ]);
        graph.add_prior(0, Pose3::identity(), strong_info());
        graph.add_between(1, 2, Pose3::identity(), medium_info());

        assert_eq!(
            optimize_pose_graph3(&mut graph, OptimizerConfig::default()).unwrap_err(),
            OptimizeError::GaugeFreedom
        );
    }

    #[test]
    fn invalid_factor_index_is_rejected() {
        let mut graph = PoseGraph3::new(vec![Pose3::identity()]);
        graph.add_between(0, 2, Pose3::identity(), medium_info());

        assert_eq!(
            optimize_pose_graph3(&mut graph, OptimizerConfig::default()).unwrap_err(),
            OptimizeError::InvalidIndex
        );
    }
}
