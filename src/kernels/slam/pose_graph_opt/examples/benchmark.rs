use std::time::Instant;

use lingtu_pose_graph_opt::geometry::{residual_between, residual_prior};
use lingtu_pose_graph_opt::solver::{diagonal_information, optimize_pose_graph3};
use lingtu_pose_graph_opt::{Information6, OptimizerConfig, Pose3, PoseGraph3, Vector6};

fn main() {
    let pose_counts = parse_pose_counts();
    let mut results = Vec::with_capacity(pose_counts.len() * 2);
    for pose_count in pose_counts {
        results.push(run_case("pgo_loop", pose_count, rotated_loop_graph));
        results.push(run_case("hba_full_info", pose_count, hba_full_info_graph));
    }

    println!("{{\"benchmark\":\"lingtu_pose_graph_opt\",\"unit\":\"milliseconds\",\"cases\":[");
    for (idx, result) in results.iter().enumerate() {
        let comma = if idx + 1 == results.len() { "" } else { "," };
        println!(
            "  {{\"case\":\"{}\",\"poses\":{},\"factors\":{},\"sparse_expected\":{},\"initial_cost\":{:.12e},\"final_cost\":{:.12e},\"final_residual_rms\":{:.12e},\"final_residual_max\":{:.12e},\"iterations\":{},\"accepted_steps\":{},\"rejected_steps\":{},\"elapsed_ms\":{:.6}}}{}",
            result.case_name,
            result.poses,
            result.factors,
            result.sparse_expected,
            result.initial_cost,
            result.final_cost,
            result.final_residual_rms,
            result.final_residual_max,
            result.iterations,
            result.accepted_steps,
            result.rejected_steps,
            result.elapsed_ms,
            comma
        );
    }
    println!("]}}");
}

fn parse_pose_counts() -> Vec<usize> {
    let mut counts: Vec<usize> = std::env::args()
        .skip(1)
        .filter_map(|value| value.parse::<usize>().ok())
        .filter(|value| *value >= 2)
        .collect();
    if counts.is_empty() {
        counts = vec![16, 64, 256];
    }
    counts
}

struct BenchResult {
    case_name: &'static str,
    poses: usize,
    factors: usize,
    sparse_expected: bool,
    initial_cost: f64,
    final_cost: f64,
    final_residual_rms: f64,
    final_residual_max: f64,
    iterations: usize,
    accepted_steps: usize,
    rejected_steps: usize,
    elapsed_ms: f64,
}

fn run_case(
    case_name: &'static str,
    pose_count: usize,
    graph_factory: fn(usize) -> PoseGraph3,
) -> BenchResult {
    let mut graph = graph_factory(pose_count);
    let initial_cost = cost_for_graph(&graph);
    let start = Instant::now();
    let report = optimize_pose_graph3(
        &mut graph,
        OptimizerConfig {
            max_iterations: 80,
            ..OptimizerConfig::default()
        },
    )
    .expect("benchmark graph should optimize");
    let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
    let residual_stats = residual_stats_for_graph(&graph);

    BenchResult {
        case_name,
        poses: pose_count,
        factors: graph.priors.len() + graph.betweens.len(),
        sparse_expected: (pose_count.saturating_sub(1) * 6) >= 72,
        initial_cost,
        final_cost: report.final_cost,
        final_residual_rms: residual_stats.rms,
        final_residual_max: residual_stats.max,
        iterations: report.iterations,
        accepted_steps: report.accepted_steps,
        rejected_steps: report.rejected_steps,
        elapsed_ms,
    }
}

fn rotated_loop_graph(pose_count: usize) -> PoseGraph3 {
    let odom = Pose3::exp(&Vector6::from_row_slice(&[
        0.012, -0.006, 0.045, 0.85, 0.08, 0.03,
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
            0.0008 * scale,
            -0.0005 * scale,
            0.0025 * scale,
            0.018 * scale,
            -0.010 * scale,
            0.003 * scale,
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
    graph
}

fn hba_full_info_graph(pose_count: usize) -> PoseGraph3 {
    let odom = Pose3::exp(&Vector6::from_row_slice(&[
        -0.006, 0.010, 0.030, 0.65, -0.04, 0.06,
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
            -0.0006 * scale,
            0.0004 * scale,
            0.0018 * scale,
            -0.012 * scale,
            0.007 * scale,
            -0.002 * scale,
        ]);
        initial.push(pose.compose(Pose3::exp(&drift)));
    }
    initial[0] = Pose3::identity();

    let mut graph = PoseGraph3::new(initial);
    for idx in 1..pose_count {
        graph.add_between(
            idx - 1,
            idx,
            truth[idx - 1].between(truth[idx]),
            full_info(),
        );
    }
    if pose_count > 4 {
        for idx in 0..(pose_count - 4) {
            graph.add_between(
                idx,
                idx + 4,
                truth[idx].between(truth[idx + 4]),
                full_info(),
            );
        }
    }
    graph
}

fn cost_for_graph(graph: &PoseGraph3) -> f64 {
    let mut cost = 0.0;
    for prior in &graph.priors {
        let residual = residual_prior(prior.pose, graph.poses[prior.index]);
        cost += weighted_cost(&prior.information, &residual);
    }
    for between in &graph.betweens {
        let residual = residual_between(
            between.measurement,
            graph.poses[between.from],
            graph.poses[between.to],
        );
        cost += weighted_cost(&between.information, &residual);
    }
    cost
}

struct ResidualStats {
    rms: f64,
    max: f64,
}

fn residual_stats_for_graph(graph: &PoseGraph3) -> ResidualStats {
    let mut sum_sq = 0.0;
    let mut max = 0.0_f64;
    let mut count = 0usize;
    for prior in &graph.priors {
        let residual = residual_prior(prior.pose, graph.poses[prior.index]);
        for value in residual.iter() {
            sum_sq += value * value;
            max = max.max(value.abs());
            count += 1;
        }
    }
    for between in &graph.betweens {
        let residual = residual_between(
            between.measurement,
            graph.poses[between.from],
            graph.poses[between.to],
        );
        for value in residual.iter() {
            sum_sq += value * value;
            max = max.max(value.abs());
            count += 1;
        }
    }
    let rms = if count == 0 {
        0.0
    } else {
        (sum_sq / count as f64).sqrt()
    };
    ResidualStats { rms, max }
}

fn weighted_cost(information: &Information6, residual: &Vector6) -> f64 {
    0.5 * residual.dot(&(information * residual))
}

fn strong_info() -> Information6 {
    diagonal_information([100.0, 100.0, 100.0, 100.0, 100.0, 100.0])
}

fn medium_info() -> Information6 {
    diagonal_information([10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
}

fn full_info() -> Information6 {
    let mut information = diagonal_information([20.0, 20.0, 20.0, 20.0, 20.0, 20.0]);
    information[(0, 3)] = 0.8;
    information[(3, 0)] = 0.8;
    information[(1, 4)] = -0.6;
    information[(4, 1)] = -0.6;
    information[(2, 5)] = 0.4;
    information[(5, 2)] = 0.4;
    information[(3, 4)] = 0.3;
    information[(4, 3)] = 0.3;
    information
}
