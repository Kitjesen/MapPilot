#include "localization/opt/online_graph.hpp"
#include "localization/opt/pose_math.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace opt = lingtu::localization::opt;

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

opt::GeometricConstraint edge(std::size_t from, std::size_t to,
                             const opt::Pose& transform, double weight = 100.0) {
  opt::GeometricConstraint result;
  result.from_index = from;
  result.to_index = to;
  result.pose_from_to = transform;
  for (std::size_t i : {0U, 6U, 11U, 15U, 18U, 20U}) result.information_upper[i] = weight;
  return result;
}

opt::OnlineGraphUpdate wait_update(opt::OnlinePoseGraph& graph) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    if (auto update = graph.poll()) return std::move(*update);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  throw std::runtime_error("online optimizer did not return an update");
}

void fill_square(opt::OnlinePoseGraph& graph) {
  // Synthetic independent odometry measurements with accumulated drift, then
  // an independent loop measurement returning to the start. No cloud matcher
  // is exercised or claimed by this fixture.
  const double xy[5][2] = {{0, 0}, {1.1, 0}, {1.2, 1}, {0.2, 1}, {0.4, 0}};
  opt::Pose previous;
  for (std::size_t i = 0; i < 5; ++i) {
    opt::Keyframe frame;
    frame.patch_name = "scan_" + std::to_string(i) + ".pcd";
    frame.pose.x = xy[i][0];
    frame.pose.y = xy[i][1];
    const auto factor = i == 0 ? std::optional<opt::GeometricConstraint>{}
        : edge(i - 1, i, opt::between_poses(previous, frame.pose));
    require(graph.append(frame, factor).ok, "measured adjacent input rejected");
    previous = frame.pose;
  }
}

void test_solve_and_nonblocking_prefix() {
  opt::OnlinePoseGraph graph;
  graph.reset(17);
  fill_square(graph);
  require(graph.start_optimization().code == "waiting_for_verified_loop",
          "must not advertise loop correction without loop evidence");
  require(!graph.busy(), "no-loop input launched worker");
  require(graph.add_verified_loop(edge(0, 4, {}, 10000)).ok, "loop rejected");
  require(graph.start_optimization().code == "optimization_started", "worker not started");
  require(graph.start_optimization().code == "online_optimizer_busy", "parallel job admitted");
  opt::Keyframe next{"scan_5.pcd", {0.5, 0, 0, 1, 0, 0, 0}};
  require(graph.append(next, edge(4, 5, {0.1, 0, 0, 1, 0, 0, 0})).ok,
          "input ingestion cannot proceed while worker owns snapshot");
  auto update = wait_update(graph);
  require(update.solution.ok, update.solution.code.c_str());
  require(update.source_epoch == 17 && update.revision == 6, "wrong snapshot identity");
  require(update.solution.keyframes.size() == 5, "worker mixed later input into prefix");
  require(std::abs(update.solution.keyframes.front().pose.x) < 1e-9, "anchor moved");
  require(std::hypot(update.solution.keyframes.back().pose.x,
                     update.solution.keyframes.back().pose.y) < 0.02,
          "loop failed to reduce 40 cm synthetic drift");
  require(graph.odometry_keyframes()[4].pose.x == 0.4, "optimizer rewrote raw odometry");
  const auto corrected = opt::compose_pose(update.map_from_odom, graph.odometry_keyframes()[4].pose);
  require(std::abs(corrected.x - update.solution.keyframes.back().pose.x) < 1e-9,
          "map-from-odom transform direction is wrong");
  require(update.solution.report.final_cost <= update.solution.report.initial_cost,
          "solver returned increasing cost");
  require(graph.start_optimization().code == "optimization_started", "new prefix not scheduled");
  update = wait_update(graph);
  require(update.solution.ok && update.solution.keyframes.size() == 6, "new prefix was lost");
  require(graph.start_optimization().code == "graph_unchanged", "unchanged graph recomputed");
}

void test_reset_and_input_rejection() {
  opt::OnlinePoseGraph graph;
  graph.reset(41);
  fill_square(graph);
  auto invalid = edge(0, 4, {});
  invalid.information_upper.fill(0);
  require(!graph.add_verified_loop(invalid).ok, "invalid information admitted");
  require(!graph.add_verified_loop(edge(0, 1, {})).ok, "adjacency counted as a loop");
  require(graph.add_verified_loop(edge(0, 4, {}, 10000)).ok, "valid loop rejected");
  require(!graph.add_verified_loop(edge(4, 0, {})).ok, "reverse duplicate double weighted");
  require(graph.start_optimization().ok, "worker not launched");
  graph.reset(42);
  require(graph.odometry_keyframes().empty(), "reset retained old graph");
  require(graph.append({"new.pcd", {}}).ok, "new session ingestion failed");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (graph.busy() && std::chrono::steady_clock::now() < deadline) {
    require(!graph.poll().has_value(), "old session correction escaped after reset");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(!graph.busy(), "stale worker did not finish");
  require(!graph.append({"second.pcd", {}}).ok, "missing adjacent factor synthesized");
  require(graph.odometry_keyframes().size() == 1, "rejected input mutated graph");
  require(graph.start_optimization().code == "waiting_for_verified_loop", "old loop survived reset");
}

void test_capacity_and_direct_solver_errors() {
  opt::OnlineGraphLimits limits;
  limits.max_keyframes = 3;
  opt::OnlinePoseGraph graph(limits);
  for (std::size_t i = 0; i < 3; ++i) {
    require(graph.append({std::to_string(i), {}},
        i == 0 ? std::optional<opt::GeometricConstraint>{} : edge(i - 1, i, {})).ok,
        "bounded input failed");
  }
  require(graph.append({"overflow", {}}, edge(2, 3, {})).code == "online_graph_capacity_reached",
          "capacity silently discarded history");
  require(graph.odometry_keyframes().size() == 3, "capacity rejection mutated history");
  opt::OptimizeOptions options;
  const std::vector<opt::Keyframe> poses = {{"0", {}}, {"1", {1, 0, 0, 1, 0, 0, 0}}};
  require(!opt::optimize_graph(poses, options).ok, "pose estimates synthesized graph factors");
  options.geometric_constraints = {edge(0, 1, {1, 0, 0, 1, 0, 0, 0})};
  require(opt::optimize_graph(poses, options).ok, "in-memory adjacent graph failed");
  options.geometric_constraints[0].information_upper.fill(0);
  auto result = opt::optimize_graph(poses, options);
  require(!result.ok && result.keyframes.empty(), "invalid solve exposed corrected poses");
  options.geometric_constraints = {edge(0, 1, {})};
  auto bad_poses = poses;
  bad_poses[1].pose.x = std::numeric_limits<double>::quiet_NaN();
  require(!opt::optimize_graph(bad_poses, options).ok, "nonfinite pose admitted");
  bad_poses = poses;
  bad_poses[1].pose.qw = 0;
  require(!opt::optimize_graph(bad_poses, options).ok, "zero quaternion silently repaired");
}

int main() {
  try {
    test_solve_and_nonblocking_prefix();
    test_reset_and_input_rejection();
    test_capacity_and_direct_solver_errors();
    std::cout << "online graph: solve, prefix isolation, reset, validation and capacity passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
