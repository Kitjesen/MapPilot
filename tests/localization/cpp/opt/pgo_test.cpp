#include "localization/opt/constraints.hpp"
#include "localization/opt/graph.hpp"
#include "localization/opt/map.hpp"
#include "localization/opt/pgo.hpp"
#include "localization/opt/pose_math.hpp"
#include "localization/opt/loop_constraints.hpp"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace opt = lingtu::localization::opt;

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void write_cloud(const std::filesystem::path& path, float x) {
  std::ofstream out(path, std::ios::binary);
  out << "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
         "COUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n";
  const float point[4] = {x, 0.0F, 0.0F, 1.0F};
  out.write(reinterpret_cast<const char*>(point), sizeof(point));
}

std::string read_all(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::map<std::string, std::string> snapshot_tree(const std::filesystem::path& root) {
  std::map<std::string, std::string> snapshot;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    if (entry.is_regular_file()) {
      snapshot.emplace(
          std::filesystem::relative(entry.path(), root).generic_string(),
          read_all(entry.path()));
    }
  }
  return snapshot;
}

void write_manifest(
    const std::filesystem::path& map,
    bool complete = true,
    std::size_t patch_count = 3,
    std::uint64_t dropped_count = 0) {
  std::ofstream manifest(map / "patch_bundle.manifest");
  manifest << "LINGTU_PATCH_BUNDLE_V1\n"
           << "complete " << (complete ? 1 : 0) << "\n"
           << "dropped_count " << dropped_count << "\n"
           << "first_sequence 0\n"
           << "last_sequence " << (patch_count - 1) << "\n"
           << "patch_count " << patch_count << "\n";
}

std::filesystem::path make_map(const std::filesystem::path& root) {
  const auto map = root / "source";
  std::filesystem::create_directories(map / "patches");
  {
    std::ofstream poses(map / "poses.txt");
    poses << "scan_000000.pcd 0 0 0 1 0 0 0\n"
             "scan_000001.pcd 1 0 0 1 0 0 0\n"
             "scan_000002.pcd 2.4 0 0 1 0 0 0\n";
  }
  write_cloud(map / "patches" / "scan_000000.pcd", 0.0F);
  write_cloud(map / "patches" / "scan_000001.pcd", 0.0F);
  write_cloud(map / "patches" / "scan_000002.pcd", 0.0F);
  write_cloud(map / "map.pcd", 0.0F);
  write_manifest(map);
  std::ofstream(map / "scan_origin.txt") << "lidar_origin_in_patch 0.16 0 0.12\n";
  return map;
}

std::array<double, 21> identity_information(double weight) {
  std::array<double, 21> upper{};
  for (std::size_t index : {0U, 6U, 11U, 15U, 18U, 20U}) {
    upper[index] = weight;
  }
  return upper;
}

opt::GeometricConstraint factor(std::size_t from, std::size_t to, double x, double weight) {
  opt::GeometricConstraint constraint;
  constraint.from_index = from;
  constraint.to_index = to;
  constraint.pose_from_to.x = x;
  constraint.information_upper = identity_information(weight);
  return constraint;
}

std::vector<opt::GeometricConstraint> complete_factors() {
  return {
      factor(0, 1, 1.0, 1000.0),
      factor(1, 2, 1.0, 1000.0),
      factor(0, 2, 2.0, 1000.0),
  };
}

std::vector<opt::GeometricConstraint> odometry_factors() {
  return {
      factor(0, 1, 1.0, 1000.0),
      factor(1, 2, 1.4, 1000.0),
  };
}

void test_optimization(const std::filesystem::path& root) {
  const auto map_dir = make_map(root);
  const auto source_tree = snapshot_tree(map_dir);
  const auto baseline_output = root / "odometry-only";
  const auto baseline_result =
      opt::pgo(opt::files(map_dir, baseline_output), odometry_factors());
  require(baseline_result.ok && baseline_result.changed, "odometry baseline failed");
  const auto baseline = opt::read_poses(baseline_output / "poses.txt");
  require(std::abs(baseline.back().pose.x - 2.4) < 1e-4,
          "odometry-only baseline unexpectedly corrected terminal drift");
  const auto output = root / "optimized";
  const auto result = opt::pgo(opt::files(map_dir, output), complete_factors());
  require(result.ok && result.changed && result.code == "optimized", "valid PGO did not optimize");
  require(result.factor_count == 3, "PGO synthesized factors outside the explicit input");
  const auto optimized = opt::read_poses(output / "poses.txt");
  require(std::abs(optimized.back().pose.x - 2.0) < std::abs(baseline.back().pose.x - 2.0),
          "loop factor did not improve over the odometry-only baseline");
  require(snapshot_tree(map_dir) == source_tree, "source map bundle changed");
  require(std::filesystem::is_regular_file(output / "map.pcd"), "bundle map missing");
  require(std::filesystem::is_regular_file(output / "poses.txt"), "bundle poses missing");
  require(std::filesystem::is_regular_file(output / "patches" / "scan_000002.pcd"),
          "bundle patch missing");
  require(std::filesystem::is_regular_file(output / "patch_bundle.manifest"),
          "bundle manifest missing");
  require(read_all(output / "scan_origin.txt") == read_all(map_dir / "scan_origin.txt"),
          "optimization lost or changed the calibrated scan origin");
  require(std::filesystem::is_regular_file(output / "map_optimization.json"),
          "bundle report missing");
  const auto report = read_all(output / "map_optimization.json");
  require(report.find("\"performed\": true") != std::string::npos,
          "optimized bundle report must distinguish performed PGO from a successful skip");
}

void test_bundle_integrity(const std::filesystem::path& root) {
  const auto map_dir = root / "source";
  const auto manifest = map_dir / "patch_bundle.manifest";
  const auto manifest_contents = read_all(manifest);

  std::filesystem::remove(manifest);
  auto result = opt::pgo(opt::files(map_dir, root / "missing-manifest"), complete_factors());
  require(!result.ok && result.code == "patch_bundle_manifest_missing",
          "missing manifest accepted");
  require(!std::filesystem::exists(root / "missing-manifest"), "missing manifest published output");

  write_manifest(map_dir, false, 3, 1);
  result = opt::pgo(opt::files(map_dir, root / "incomplete-manifest"), complete_factors());
  require(!result.ok && result.code == "patch_bundle_incomplete", "incomplete manifest accepted");
  require(!std::filesystem::exists(root / "incomplete-manifest"),
          "incomplete manifest published output");

  std::ofstream(manifest, std::ios::binary) << manifest_contents;
  write_cloud(map_dir / "patches" / "extra.pcd", 0.0F);
  result = opt::pgo(opt::files(map_dir, root / "extra-patch"), complete_factors());
  require(!result.ok && result.code == "patch_bundle_manifest_mismatch", "extra patch accepted");
  std::filesystem::remove(map_dir / "patches" / "extra.pcd");

  const auto poses_path = map_dir / "poses.txt";
  const auto poses = read_all(poses_path);
  std::ofstream(poses_path) << "scan_000000.pcd 0 0 0 1 0 0 0\n"
                              "scan_000000.pcd 1 0 0 1 0 0 0\n"
                              "scan_000002.pcd 2.4 0 0 1 0 0 0\n";
  result = opt::pgo(opt::files(map_dir, root / "duplicate-name"), complete_factors());
  require(!result.ok && result.code == "duplicate_patch_name", "duplicate patch name accepted");

  std::ofstream(poses_path) << "0 0 0 1 0 0 0\n1 0 0 1 0 0 0\n2.4 0 0 1 0 0 0\n";
  result = opt::pgo(opt::files(map_dir, root / "unnamed-pose"), complete_factors());
  require(!result.ok, "unnamed pose accepted");
  std::ofstream(poses_path, std::ios::binary) << poses;
}

void test_optimizer_quality_gate(const std::filesystem::path& root) {
  opt::OptimizeOptions options;
  options.max_iterations = 2;
  options.geometric_constraints = complete_factors();
  const auto output = root / "exhausted";
  const auto result = opt::optimize_map(opt::files(root / "source", output), options);
  require(!result.ok && result.code == "optimizer_quality_failed",
          "non-converged optimizer result passed quality gate");
  require(!std::filesystem::exists(output), "failed optimizer published output");
}

void test_information_validation(const std::filesystem::path& root) {
  const auto map_dir = root / "source";
  auto zero = factor(0, 2, 2.0, 1.0);
  zero.information_upper = {};
  auto result = opt::pgo(opt::files(map_dir, root / "zero"), {zero});
  require(!result.ok && result.code == "geometric_constraint_invalid", "zero information accepted");

  auto indefinite = factor(0, 2, 2.0, 1.0);
  indefinite.information_upper = identity_information(1.0);
  indefinite.information_upper[1] = 2.0;
  result = opt::pgo(opt::files(map_dir, root / "indefinite"), {indefinite});
  require(!result.ok && result.code == "geometric_constraint_invalid",
          "indefinite information accepted");

  result = opt::pgo(opt::files(map_dir, root / "empty"), {});
  require(result.ok && !result.changed && result.code == "skipped_no_independent_constraints",
          "empty constraints did not skip");
  require(!std::filesystem::exists(root / "empty"), "skip unexpectedly created output");
}

void test_parser(const std::filesystem::path& root) {
  const auto path = root / "constraints.txt";
  std::ofstream out(path);
  out << "LINGTU_PGO_CONSTRAINTS_V1\n"
         "T_from_to tx ty tz qw qx qy qz\n"
         "RIGHT_TANGENT omega_x omega_y omega_z upsilon_x upsilon_y upsilon_z\n"
         "UPPER_TRIANGLE row_major 21\n"
         "0 2 2 0 0 1 0 0 0";
  for (double value : identity_information(10.0)) {
    out << ' ' << value;
  }
  out << '\n';
  out.close();
  const auto constraints = opt::read_constraints(path);
  require(constraints.size() == 1 && constraints.front().information_upper[20] == 10.0,
          "constraints parser did not preserve upper-21");
}

}  // namespace

void test_gravity_observations() {
  // Re-anchoring a long map must not amplify a small accepted tilt error into height drift.
  auto source_anchor = opt::pose_with_rpy({10, 4, .7}, .2, -.15, .8);
  auto target_anchor = opt::pose_with_rpy({9, 3, .8}, .22, -.14, 1.1);
  const auto alignment = opt::gravity_preserving_alignment(target_anchor, source_anchor);
  const auto aligned = opt::compose_pose(alignment, source_anchor);
  require(opt::pose_translation_distance(aligned,target_anchor)<1e-10, "reanchor moved current position");
  require(opt::pose_yaw_difference(aligned,target_anchor)<1e-10, "reanchor moved current heading");
  const auto gravity=opt::rotate_vector(alignment,0,0,1);
  require(std::abs(gravity[0])+std::abs(gravity[1])+std::abs(gravity[2]-1)<1e-10,
          "reanchor rotated map gravity");
  require(std::abs(opt::pose_rpy(aligned)[0]-.2)<1e-10, "reanchor discarded real body tilt");
  std::vector<opt::Keyframe> poses(5);
  const double x[]{0,4,4,0,0}, y[]{0,0,4,4,0};
  opt::OptimizeOptions options;
  options.max_iterations = 200;
  for (int i=0;i<5;++i) { poses[i].pose.x=x[i]; poses[i].pose.y=y[i]; }
  for (int i=0;i<5;++i) {
    opt::GeometricConstraint e;
    e.from_index=i<4?i:0; e.to_index=i<4?i+1:4;
    e.pose_from_to=opt::between_poses(poses[e.from_index].pose, poses[e.to_index].pose);
    if(i<4) e.pose_from_to.z=.03;
    std::array<std::array<double,4>,4> h{};
    for(int j=0;j<4;++j) h[j][j]=100;
    e.information_upper=opt::detail::body_right_information_upper(h,1,e.pose_from_to,opt::Pose{});
    options.geometric_constraints.push_back(e);
  }
  options.gravity_reference=poses;
  auto solved=opt::optimize_graph(poses,options);
  require(solved.ok,solved.code);
  for(const auto& frame:solved.keyframes) {
    auto angles=opt::pose_rpy(frame.pose);
    require(std::hypot(angles[0],angles[1]) < 1e-10, "rank-four graph invented tilt");
  }
  // Warm starts must recover the original measurement, not reinforce the last solution.
  auto warm=poses;
  warm[2].pose=opt::pose_with_rpy(warm[2].pose,.12,-.09,0);
  solved=opt::optimize_graph(warm,options);
  require(solved.ok,solved.code);
  require(std::abs(opt::pose_rpy(solved.keyframes[2].pose)[0]) < 1e-10, "warm start replaced gravity");

  // Real tilt and elevation are retained; yaw is allowed to change independently.
  std::vector<opt::Keyframe> slope(2);
  slope[0].pose=opt::pose_with_rpy({},.2,-.15,0);
  slope[1].pose=opt::pose_with_rpy({2,0,.5},-.1,.25,0);
  auto expected=slope[1].pose;
  expected=opt::compose_pose(opt::pose_with_rpy({},0,0,.6),expected);
  auto edge=factor(0,1,0,1000);
  edge.pose_from_to=opt::between_poses(slope[0].pose,expected);
  options.geometric_constraints={edge};options.gravity_reference=slope;
  solved=opt::optimize_graph(slope,options);
  require(solved.ok,solved.code);
  require(std::abs(solved.keyframes[1].pose.z-.5)<1e-4, "gravity flattened elevation");
  require(std::abs(opt::pose_rpy(solved.keyframes[1].pose)[2]-.6)<1e-4, "gravity constrained yaw");
  std::swap(edge.from_index, edge.to_index);
  edge.pose_from_to = opt::inverse_pose(edge.pose_from_to);
  options.geometric_constraints = {edge};
  solved = opt::optimize_graph(slope, options);
  require(solved.ok && opt::pose_translation_distance(solved.keyframes[1].pose, expected)<1e-4,
          "fixed destination factor changed pose convention");
  std::swap(edge.from_index, edge.to_index);
  edge.pose_from_to = opt::inverse_pose(edge.pose_from_to);
  edge.information_upper=identity_information(1e9);
  auto tilted=opt::pose_with_rpy(slope[1].pose,.6,.25,0);
  edge.pose_from_to=opt::between_poses(slope[0].pose,tilted);
  options.geometric_constraints={edge};
  require(opt::optimize_graph(slope,options).code=="optimizer_gravity_inconsistent",
          "low-cost but gravity-inconsistent solve was accepted");
  options.gravity_reference.pop_back();
  require(opt::optimize_graph(slope,options).code=="invalid_gravity_reference", "partial gravity accepted");
}

void test_disconnected_graph_recovery() {
  std::vector<opt::Keyframe> poses(4);
  for (std::size_t i = 0; i < poses.size(); ++i) poses[i].pose.x = i;
  opt::OptimizeOptions options;
  options.geometric_constraints = {factor(0, 1, 1, 100), factor(2, 3, 1, 100)};
  require(opt::connected_pose_indices(4, options.geometric_constraints) ==
              std::vector<std::size_t>({0, 1}), "disconnected segment counted as registered");
  require(opt::optimize_graph(poses, options).code == "graph_disconnected",
          "unanchored component entered optimizer");
  // A measured non-adjacent bridge connects the retained segment without 1->2.
  options.geometric_constraints.push_back(factor(0, 3, 3, 100));
  require(opt::connected_pose_indices(4, options.geometric_constraints).size() == 4,
          "verified bridge did not recover all retained nodes");
  const auto result = opt::optimize_graph(poses, options);
  require(result.ok, "connected graph with missing adjacent edge rejected: " + result.code);
}

int main() {
  try {
    const auto root = std::filesystem::temp_directory_path() / "lingtu-pgo-core-test";
    std::error_code error;
    std::filesystem::remove_all(root, error);
    std::filesystem::create_directories(root);
    test_optimization(root);
    test_gravity_observations();
    test_disconnected_graph_recovery();
    test_bundle_integrity(root);
    test_information_validation(root);
    test_optimizer_quality_gate(root);
    test_parser(root);
    std::filesystem::remove_all(root, error);
    std::cout << "pgo_test: passed\n";
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << "pgo_test: " << exception.what() << '\n';
    return 1;
  }
}
