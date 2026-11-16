//! Run the official DimOS MLS Planner on occupied voxel centers.
//! Copy this file to the frozen upstream crate's examples directory.

use std::collections::{BTreeSet, HashMap, VecDeque};
use std::env;
use std::fs;
use std::time::Instant;

use dimos_mls_planner::mls_planner::{Config, Planner, RegionBounds};
use dimos_mls_planner::voxel::voxelize;

type Point = (f32, f32, f32);

fn percentile(values: &[f64], p: f64) -> f64 {
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

fn components(planner: &Planner) -> usize {
    let graph = planner.graph();
    let node_ids: Vec<_> = graph.nodes.iter().map(|node| node.cell_id).collect();
    let indices: HashMap<_, _> = node_ids
        .iter()
        .enumerate()
        .map(|(i, &id)| (id, i))
        .collect();
    let mut seen = vec![false; node_ids.len()];
    let mut count = 0;
    for seed in 0..node_ids.len() {
        if seen[seed] {
            continue;
        }
        count += 1;
        let mut queue = VecDeque::from([seed]);
        seen[seed] = true;
        while let Some(index) = queue.pop_front() {
            let node = node_ids[index];
            for &edge_index in graph.node_edges.adjacent(node) {
                let edge = &graph.node_edges.edges()[edge_index as usize];
                let other = if edge.a == node { edge.b } else { edge.a };
                let Some(&other_index) = indices.get(&other) else {
                    continue;
                };
                if !seen[other_index] {
                    seen[other_index] = true;
                    queue.push_back(other_index);
                }
            }
        }
    }
    count
}

fn save_route(path: &str, route: Option<&Vec<Point>>) {
    let text = route
        .into_iter()
        .flatten()
        .map(|p| format!("{} {} {}\n", p.0, p.1, p.2))
        .collect::<String>();
    fs::write(path, text).expect("route output");
}

fn main() {
    let args: Vec<String> = env::args().collect();
    assert_eq!(args.len(), 18,
        "occupied.xyz resolution start_x start_y start_z goal_x goal_y goal_z output_prefix samples robot_height node_spacing closing step goal_xy goal_z wall_buffer");
    let n = |i: usize| args[i].parse::<f32>().expect("numeric argument");
    let points: Vec<Point> = fs::read_to_string(&args[1])
        .expect("occupied xyz")
        .lines()
        .filter(|line| !line.trim().is_empty() && !line.starts_with('#'))
        .map(|line| {
            let xyz: Vec<f32> = line
                .split_whitespace()
                .map(|v| v.parse().expect("coordinate"))
                .collect();
            assert_eq!(xyz.len(), 3);
            (xyz[0], xyz[1], xyz[2])
        })
        .collect();
    let start = (n(3), n(4), n(5));
    let goal = (n(6), n(7), n(8));
    let samples: usize = args[10].parse().expect("sample count");
    assert!(samples > 0 && !points.is_empty());
    let cfg = Config {
        world_frame: "map".into(),
        base_frame: "base_link".into(),
        voxel_size: n(2),
        robot_height: n(11),
        start_z_offset_m: 0.0,
        max_overhead_m: 2.0,
        surface_closing_radius: n(13),
        node_spacing_m: n(12),
        wall_clearance_m: 0.0,
        wall_buffer_m: n(17),
        wall_buffer_weight: 0.0,
        step_threshold_m: n(14),
        step_penalty_weight: 0.0,
        goal_tolerance: n(15),
        goal_z_tolerance: n(16),
        blocked_timeout_s: 2.0,
        progress_timeout_s: 10.0,
        progress_min_m: 0.2,
        viz_publish_hz: 0.0,
        viz_cell_m: 4.0,
        viz_sweep_cells: 0,
        worker_threads: 1,
    };
    let mut cold = Vec::new();
    let mut planner = Planner::new(cfg.worker_threads);
    for _ in 0..5 {
        let mut built = Planner::new(cfg.worker_threads);
        let begin = Instant::now();
        built.update_global_map(&points, &cfg);
        cold.push(begin.elapsed().as_secs_f64() * 1e3);
        planner = built;
    }
    let surface_count = planner.surface().count();
    let node_count = planner.graph().nodes.len();
    let edge_count = planner.graph().node_edges.edges().len();
    let component_count = components(&planner);
    let route = planner.plan(start, goal, &cfg);
    let route_len: f32 = route.as_ref().map_or(0.0, |r| {
        r.windows(2)
            .map(|p| {
                ((p[1].0 - p[0].0).powi(2) + (p[1].1 - p[0].1).powi(2) + (p[1].2 - p[0].2).powi(2))
                    .sqrt()
            })
            .sum()
    });
    save_route(&format!("{}.route.xyz", args[9]), route.as_ref());
    let mut queries = Vec::new();
    for _ in 0..samples {
        let begin = Instant::now();
        let current = planner.plan(start, goal, &cfg);
        queries.push(begin.elapsed().as_secs_f64() * 1e3);
        assert_eq!(current, route, "repeated warm query changed route");
    }
    let patch_point = route
        .as_ref()
        .and_then(|r| r.get(r.len() / 2))
        .copied()
        .unwrap_or(points[0]);
    let bounds = RegionBounds {
        origin_x: patch_point.0,
        origin_y: patch_point.1,
        radius: 0.5,
        z_min: patch_point.2 - 1.0,
        z_max: patch_point.2 + 1.0,
    };
    let local: Vec<_> = points
        .iter()
        .copied()
        .filter(|p| {
            let (x, y, z) = voxelize(*p, cfg.voxel_size);
            let half = cfg.voxel_size * 0.5;
            let dx = x as f32 * cfg.voxel_size + half - bounds.origin_x;
            let dy = y as f32 * cfg.voxel_size + half - bounds.origin_y;
            let height = z as f32 * cfg.voxel_size + half;
            dx * dx + dy * dy <= bounds.radius * bounds.radius
                && height >= bounds.z_min
                && height <= bounds.z_max
        })
        .collect();
    let original_voxels: BTreeSet<_> = planner.voxel_keys().collect();
    let mut changed = local.clone();
    changed.push((
        patch_point.0,
        patch_point.1,
        patch_point.2 + cfg.voxel_size * 2.0,
    ));
    let begin = Instant::now();
    let update_changed = planner.update_region(&changed, &bounds, &cfg).is_some();
    let update_ms = begin.elapsed().as_secs_f64() * 1e3;
    let begin = Instant::now();
    let restore_changed = planner.update_region(&local, &bounds, &cfg).is_some();
    let restore_ms = begin.elapsed().as_secs_f64() * 1e3;
    assert_eq!(
        planner.voxel_keys().collect::<BTreeSet<_>>(),
        original_voxels,
        "region replacement failed to restore occupied voxels"
    );
    let restored = planner.plan(start, goal, &cfg);
    save_route(
        &format!("{}.restored.route.xyz", args[9]),
        restored.as_ref(),
    );
    println!("{{\"points\":{},\"surfaces\":{},\"nodes\":{},\"edges\":{},\"components\":{},\"route_points\":{},\"route_length_m\":{},\"cold_samples\":5,\"cold_median_ms\":{},\"cold_p95_ms\":{},\"warm_samples\":{},\"warm_median_ms\":{},\"warm_p95_ms\":{},\"update_ms\":{},\"update_changed\":{},\"restore_ms\":{},\"restore_changed\":{},\"restored_route_points\":{},\"cold_raw_ms\":{:?},\"warm_raw_ms\":{:?}}}",
        points.len(), surface_count, node_count, edge_count, component_count,
        route.as_ref().map_or(0, Vec::len), route_len,
        percentile(&cold, 0.5), percentile(&cold, 0.95), samples,
        percentile(&queries, 0.5), percentile(&queries, 0.95), update_ms, update_changed,
        restore_ms, restore_changed, restored.as_ref().map_or(0, Vec::len), cold, queries);
}
