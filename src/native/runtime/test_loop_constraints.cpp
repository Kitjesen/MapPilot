#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "localization/opt/loop_constraints.hpp"
#include "localization/opt/pose_math.hpp"

namespace {

using lingtu::localization::opt::GeometricConstraint;
using lingtu::localization::opt::Keyframe;
using lingtu::localization::opt::LoopCandidateDiagnostic;
using lingtu::localization::opt::LoopConstraintOptions;
using lingtu::localization::opt::LoopConstraintResult;
using lingtu::localization::opt::Map;
using lingtu::localization::opt::Pose;

constexpr double kPi = 3.14159265358979323846;

struct Point {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double intensity = 0.0;
};

class TempMapDir {
 public:
  explicit TempMapDir(const std::string &label) {
    const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("lingtu_loop_constraints_" + label + "_" + std::to_string(stamp));
    std::filesystem::remove_all(path_);
    std::filesystem::create_directories(path_ / "patches");
  }

  TempMapDir(const TempMapDir &) = delete;
  TempMapDir &operator=(const TempMapDir &) = delete;

  ~TempMapDir() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path &path() const { return path_; }

 private:
  std::filesystem::path path_;
};

void Require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

double wrap_angle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yaw_of(const Pose &pose) {
  const double sin_yaw = 2.0 * (pose.qw * pose.qz + pose.qx * pose.qy);
  const double cos_yaw = 1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz);
  return std::atan2(sin_yaw, cos_yaw);
}

Pose rpy_pose(double x, double y, double z, double roll, double pitch, double yaw) {
  Pose pose;
  pose.x = x;
  pose.y = y;
  pose.z = z;
  const double cr = std::cos(0.5 * roll);
  const double sr = std::sin(0.5 * roll);
  const double cp = std::cos(0.5 * pitch);
  const double sp = std::sin(0.5 * pitch);
  const double cy = std::cos(0.5 * yaw);
  const double sy = std::sin(0.5 * yaw);
  pose.qw = cr * cp * cy + sr * sp * sy;
  pose.qx = sr * cp * cy - cr * sp * sy;
  pose.qy = cr * sp * cy + sr * cp * sy;
  pose.qz = cr * cp * sy - sr * sp * cy;
  return pose;
}

Pose yaw_pose(double x, double y, double z, double yaw) {
  return rpy_pose(x, y, z, 0.0, 0.0, yaw);
}

Point world_to_local(const Point &point, const Pose &pose) {
  const double dx = point.x - pose.x;
  const double dy = point.y - pose.y;
  const double dz = point.z - pose.z;
  return Point{
      (1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz)) * dx +
          2.0 * (pose.qx * pose.qy + pose.qz * pose.qw) * dy +
          2.0 * (pose.qx * pose.qz - pose.qy * pose.qw) * dz,
      2.0 * (pose.qx * pose.qy - pose.qz * pose.qw) * dx +
          (1.0 - 2.0 * (pose.qx * pose.qx + pose.qz * pose.qz)) * dy +
          2.0 * (pose.qy * pose.qz + pose.qx * pose.qw) * dz,
      2.0 * (pose.qx * pose.qz + pose.qy * pose.qw) * dx +
          2.0 * (pose.qy * pose.qz - pose.qx * pose.qw) * dy +
          (1.0 - 2.0 * (pose.qx * pose.qx + pose.qy * pose.qy)) * dz,
      point.intensity,
  };
}

std::vector<Point> to_local_cloud(const std::vector<Point> &world, const Pose &pose) {
  std::vector<Point> local;
  local.reserve(world.size());
  for (const Point &point : world) {
    local.push_back(world_to_local(point, pose));
  }
  return local;
}

std::vector<Point> asymmetric_world_cloud() {
  std::vector<Point> points;

  // Three non-parallel, non-symmetric surfaces make yaw, XY, and Z observable.
  for (int i = 0; i < 27; ++i) {
    for (int j = 0; j < 13; ++j) {
      const double u = -3.2 + 0.24 * static_cast<double>(i);
      const double v = -1.1 + 0.19 * static_cast<double>(j);
      points.push_back(Point{
          u,
          -2.35 + 0.035 * std::sin(0.7 * i + 0.2 * j),
          v + 0.08 * std::sin(0.31 * i),
          10.0 + i + 0.01 * j,
      });
    }
  }
  for (int i = 0; i < 21; ++i) {
    for (int j = 0; j < 15; ++j) {
      const double u = -2.1 + 0.21 * static_cast<double>(i);
      const double v = -1.25 + 0.17 * static_cast<double>(j);
      points.push_back(Point{
          2.55 + 0.04 * std::cos(0.4 * i),
          u + 0.11 * std::sin(0.2 * j),
          v,
          100.0 + i + 0.01 * j,
      });
    }
  }
  for (int i = 0; i < 25; ++i) {
    for (int j = 0; j < 11; ++j) {
      const double u = -2.8 + 0.23 * static_cast<double>(i);
      const double v = -1.7 + 0.28 * static_cast<double>(j);
      points.push_back(Point{
          u,
          v + 0.16 * std::sin(0.37 * i),
          1.72 + 0.06 * std::cos(0.5 * j + 0.1 * i),
          200.0 + i + 0.01 * j,
      });
    }
  }

  // An off-centre vertical feature breaks the remaining 180-degree ambiguity.
  for (int i = 0; i < 90; ++i) {
    const double phase = 0.19 * static_cast<double>(i);
    points.push_back(Point{
        -1.65 + 0.23 * std::cos(phase),
        1.45 + 0.17 * std::sin(phase),
        -0.9 + 0.035 * static_cast<double>(i),
        300.0 + i,
    });
  }

  return points;
}

std::vector<Point> planar_cloud() {
  std::vector<Point> points;
  for (int i = 0; i < 35; ++i) {
    for (int j = 0; j < 25; ++j) {
      points.push_back(Point{
          -3.4 + 0.2 * static_cast<double>(i),
          -2.4 + 0.2 * static_cast<double>(j),
          0.0,
          static_cast<double>(i * 25 + j),
      });
    }
  }
  return points;
}

std::vector<Point> vertical_wall_cloud() {
  std::vector<Point> points;
  points.reserve(41U * 25U);
  for (int y_index = 0; y_index < 41; ++y_index) {
    for (int z_index = 0; z_index < 25; ++z_index) {
      points.push_back(Point{
          1.75,
          -3.0 + 0.15 * static_cast<double>(y_index),
          -1.5 + 0.15 * static_cast<double>(z_index),
          static_cast<double>(y_index * 25 + z_index),
      });
    }
  }
  return points;
}

std::vector<Point> long_corridor_cloud() {
  std::vector<Point> points;
  points.reserve(1800);

  // Two long, parallel walls constrain cross-corridor translation but provide
  // no point-to-plane information along the corridor axis.
  for (double wall_x : {-2.0, 2.0}) {
    for (int y_index = 0; y_index < 41; ++y_index) {
      for (int z_index = 0; z_index < 13; ++z_index) {
        points.push_back(Point{
            wall_x,
            -5.0 + 0.25 * static_cast<double>(y_index),
            -1.0 + 0.25 * static_cast<double>(z_index),
            1000.0 * (wall_x > 0.0 ? 1.0 : 0.0) + static_cast<double>(y_index * 13 + z_index),
        });
      }
    }
  }

  // The floor makes Z observable while intentionally leaving translation
  // along Y unobservable. There is no end wall in this fixture.
  for (int x_index = 0; x_index < 17; ++x_index) {
    for (int y_index = 0; y_index < 41; ++y_index) {
      points.push_back(Point{
          -2.0 + 0.25 * static_cast<double>(x_index),
          -5.0 + 0.25 * static_cast<double>(y_index),
          -1.0,
          3000.0 + static_cast<double>(x_index * 41 + y_index),
      });
    }
  }
  return points;
}

std::vector<Point> shifted_cloud(const std::vector<Point> &points, double x, double y, double z) {
  std::vector<Point> shifted = points;
  for (Point &point : shifted) {
    point.x += x;
    point.y += y;
    point.z += z;
  }
  return shifted;
}

void write_pcd(const std::filesystem::path &path, const std::vector<Point> &points) {
  std::ofstream out(path, std::ios::binary);
  Require(out.is_open(), "failed to create PCD fixture: " + path.string());
  out << "# .PCD v0.7 - Point Cloud Data file format\n";
  out << "VERSION 0.7\n";
  out << "FIELDS x y z intensity\n";
  out << "SIZE 4 4 4 4\n";
  out << "TYPE F F F F\n";
  out << "COUNT 1 1 1 1\n";
  out << "WIDTH " << points.size() << "\n";
  out << "HEIGHT 1\n";
  out << "VIEWPOINT 0 0 0 1 0 0 0\n";
  out << "POINTS " << points.size() << "\n";
  out << "DATA ascii\n";
  out << std::setprecision(9);
  for (const Point &point : points) {
    out << point.x << " " << point.y << " " << point.z << " " << point.intensity << "\n";
  }
  Require(out.good(), "failed while writing PCD fixture: " + path.string());
}

void write_poses(const std::filesystem::path &path, const std::vector<Keyframe> &keyframes) {
  std::ofstream out(path, std::ios::binary);
  Require(out.is_open(), "failed to create poses fixture: " + path.string());
  out << std::setprecision(17);
  for (const Keyframe &keyframe : keyframes) {
    out << keyframe.patch_name << " " << keyframe.pose.x << " " << keyframe.pose.y << " "
        << keyframe.pose.z << " " << keyframe.pose.qw << " " << keyframe.pose.qx << " "
        << keyframe.pose.qy << " " << keyframe.pose.qz << "\n";
  }
  Require(out.good(), "failed while writing poses fixture: " + path.string());
}

std::string read_bytes(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  Require(in.is_open(), "failed to read fixture bytes: " + path.string());
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

std::vector<Keyframe> loop_keyframes() {
  return {
      Keyframe{"0000.pcd", yaw_pose(0.0, 0.0, 0.0, 0.0)},
      Keyframe{"0001.pcd", yaw_pose(5.0, 0.0, 0.05, 0.05)},
      Keyframe{"0002.pcd", yaw_pose(5.0, 5.0, 0.10, 1.55)},
      Keyframe{"0003.pcd", yaw_pose(0.0, 5.0, 0.15, 3.10)},
      // The final odometry pose contains deliberate XY, Z, and yaw drift.
      Keyframe{"0004.pcd", yaw_pose(0.90, -0.55, 0.70, 0.52)},
  };
}

Map make_fixture(const std::filesystem::path &root, const std::vector<Point> &anchor_world,
                 const std::vector<Point> &closing_world, const Pose &true_closing_pose,
                 std::vector<Keyframe> *keyframes_out) {
  const std::vector<Keyframe> keyframes = loop_keyframes();
  write_pcd(root / "map.pcd", anchor_world);
  write_poses(root / "poses.txt", keyframes);

  write_pcd(root / "patches" / keyframes[0].patch_name, anchor_world);
  for (std::size_t i = 1; i + 1 < keyframes.size(); ++i) {
    write_pcd(root / "patches" / keyframes[i].patch_name,
              shifted_cloud(anchor_world, 30.0 * static_cast<double>(i), 17.0, 4.0));
  }
  write_pcd(root / "patches" / keyframes.back().patch_name,
            to_local_cloud(closing_world, true_closing_pose));

  if (keyframes_out != nullptr) {
    *keyframes_out = keyframes;
  }
  return lingtu::localization::opt::files(root);
}

LoopConstraintOptions test_options() {
  LoopConstraintOptions options;
  options.min_index_separation = 4;
  options.min_path_separation_m = 10.0;
  options.candidate_xy_radius_m = 2.0;
  options.candidate_max_abs_z_m = 2.0;
  options.submap_half_window = 0;
  options.voxel_size_m = 0.08;
  options.max_points_per_submap = 5000;
  options.angular_bins = 72;
  options.radial_bins = 16;
  options.height_bins = 12;
  options.descriptor_max_radius_m = 20.0;
  options.descriptor_min_z_m = -5.0;
  options.descriptor_max_z_m = 5.0;
  options.descriptor_min_similarity = 0.0;
  options.descriptor_min_margin = 0.0;
  options.descriptor_min_yaw_margin = 0.0;
  options.yaw_hypotheses = 8;
  options.max_candidates_per_anchor = 1;
  options.max_total_candidates = 4;
  options.max_correspondence_distance_m = 1.4;
  options.max_icp_iterations = 35;
  options.trim_fraction = 0.80;
  options.min_inliers = 120;
  options.min_inlier_ratio = 0.20;
  options.max_rmse_m = 0.18;
  options.max_p95_m = 0.30;
  options.min_xy_span_m = 2.0;
  options.min_z_span_m = 0.50;
  options.max_hessian_condition = 1.0e8;
  options.max_correction_xy_m = 3.0;
  options.max_correction_z_m = 2.0;
  options.max_correction_yaw_rad = 1.2;
  options.max_inverse_translation_m = 0.20;
  options.max_inverse_yaw_rad = 0.08;
  options.require_consensus = false;
  options.min_consistent_matches = 1;
  return options;
}

const GeometricConstraint &require_constraint(const LoopConstraintResult &result, std::size_t from,
                                              std::size_t to) {
  const auto found =
      std::find_if(result.constraints.begin(), result.constraints.end(),
                   [from, to](const GeometricConstraint &constraint) {
                     return constraint.from_index == from && constraint.to_index == to;
                   });
  if (found == result.constraints.end()) {
    std::ostringstream message;
    message << "missing expected T_" << from << "_" << to
            << " loop constraint; code=" << result.code
            << ", candidates=" << result.report.candidate_count
            << ", verified=" << result.report.geometrically_verified_count
            << ", accepted=" << result.report.accepted_constraint_count;
    for (const LoopCandidateDiagnostic &candidate : result.report.candidates) {
      message << " [" << candidate.from_index << "->" << candidate.to_index << ":"
              << candidate.reason << ",score=" << candidate.descriptor_score
              << ",rmse=" << candidate.rmse_m << ",p95=" << candidate.p95_m << "]";
    }
    throw std::runtime_error(message.str());
  }
  return *found;
}

void require_pose_near(const Pose &actual, const Pose &expected, double translation_tolerance,
                       double yaw_tolerance, const std::string &label) {
  const double dx = actual.x - expected.x;
  const double dy = actual.y - expected.y;
  const double dz = actual.z - expected.z;
  const double translation_error = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double yaw_error = std::abs(wrap_angle(yaw_of(actual) - yaw_of(expected)));
  if (translation_error > translation_tolerance || yaw_error > yaw_tolerance) {
    std::ostringstream message;
    message << label << " differs: translation error=" << translation_error
            << ", yaw error=" << yaw_error << ", actual=(" << actual.x << "," << actual.y << ","
            << actual.z << "," << yaw_of(actual) << "), expected=(" << expected.x << ","
            << expected.y << "," << expected.z << "," << yaw_of(expected) << ")";
    throw std::runtime_error(message.str());
  }
}

double rotation_distance(const Pose &lhs, const Pose &rhs) {
  const double dot =
      std::abs(lhs.qw * rhs.qw + lhs.qx * rhs.qx + lhs.qy * rhs.qy + lhs.qz * rhs.qz);
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

void test_point_to_plane_observability_accepts_asymmetric_three_surface_and_transform_direction() {
  TempMapDir temp("direction");
  const std::vector<Point> world = asymmetric_world_cloud();
  const Pose true_closing_pose = yaw_pose(0.45, -0.25, 0.32, 0.27);
  std::vector<Keyframe> keyframes;
  const Map map = make_fixture(temp.path(), world, world, true_closing_pose, &keyframes);

  const LoopConstraintResult result =
      lingtu::localization::opt::generate_loop_constraints(map, keyframes, test_options());
  Require(result.ok, "loop generation failed: " + result.code + ": " + result.message);
  Require(result.report.candidate_count >= 1, "true loop produced no candidate");
  Require(result.report.geometrically_verified_count >= 1,
          "true loop was not geometrically verified");
  Require(result.report.accepted_constraint_count == result.constraints.size(),
          "accepted constraint report count disagrees with result constraints");

  const GeometricConstraint &constraint = require_constraint(result, 0, 4);
  // Frame contract: patch points are body-local and poses are T_map_body.
  // Therefore the graph measurement must be T_from_to, here T_0_4.
  require_pose_near(constraint.pose_from_to, true_closing_pose, 0.22, 0.10, "T_from_to direction");
  Require(constraint.pose_from_to.x > 0.0 && constraint.pose_from_to.y < 0.0 &&
              constraint.pose_from_to.z > 0.0 && yaw_of(constraint.pose_from_to) > 0.0,
          "loop transform appears inverted instead of T_from_to");
  Require(std::all_of(constraint.information_diagonal.begin(),
                      constraint.information_diagonal.end(),
                      [](double value) { return value == 0.0; }),
          "shadow loop constraint unexpectedly exposed graph-compatible information");
}

void require_point_to_plane_observability_rejection(const LoopConstraintResult &result,
                                                    const std::string &fixture) {
  Require(result.ok, fixture + " shadow run failed: " + result.code + ": " + result.message);
  Require(result.report.candidate_count >= 1,
          fixture + " did not reach loop-candidate verification");
  Require(result.constraints.empty(), fixture + " produced a geometrically underconstrained loop");
  Require(result.report.rejected_count >= 1, fixture + " rejection was not reported");
  const bool rejected_by_point_to_plane_gate =
      std::any_of(result.report.candidates.begin(), result.report.candidates.end(),
                  [](const LoopCandidateDiagnostic &diagnostic) {
                    return diagnostic.reason == "point_to_plane_rank_gate" ||
                           diagnostic.reason == "point_to_plane_condition_gate";
                  });
  if (!rejected_by_point_to_plane_gate) {
    std::ostringstream message;
    message << fixture << " was not rejected by the point-to-plane observability gate";
    for (const LoopCandidateDiagnostic &diagnostic : result.report.candidates) {
      message << " [" << diagnostic.from_index << "->" << diagnostic.to_index << ":"
              << diagnostic.reason << ",rmse=" << diagnostic.rmse_m << ",p95=" << diagnostic.p95_m
              << ",inliers=" << diagnostic.inliers << "]";
    }
    throw std::runtime_error(message.str());
  }
}

void test_point_to_plane_observability_rejects_single_vertical_wall_slide() {
  TempMapDir temp("vertical_wall_slide");
  const std::vector<Point> wall = vertical_wall_cloud();
  // The closing scan is the same physical wall. Its odometry error contains
  // translation along the wall and in Z, both unobservable from the wall
  // normal even when nearest-neighbour point-to-point ICP reports low error.
  const Pose true_closing_pose = yaw_pose(0.30, -0.25, 0.20, 0.08);
  std::vector<Keyframe> keyframes;
  const Map map = make_fixture(temp.path(), wall, wall, true_closing_pose, &keyframes);
  LoopConstraintOptions options = test_options();
  options.max_rmse_m = 0.25;
  options.max_p95_m = 0.40;

  const LoopConstraintResult result =
      lingtu::localization::opt::generate_loop_constraints(map, keyframes, options);
  require_point_to_plane_observability_rejection(result, "single vertical wall slide");
}

void test_point_to_plane_observability_rejects_long_corridor_slide() {
  TempMapDir temp("long_corridor_slide");
  const std::vector<Point> corridor = long_corridor_cloud();
  // Parallel walls plus a floor constrain X, Z, and yaw, but not translation
  // along the corridor's Y axis. Finite point samples must not create a false
  // point-to-point notion of observability in that direction.
  const Pose true_closing_pose = yaw_pose(0.20, 0.65, 0.18, 0.05);
  std::vector<Keyframe> keyframes;
  const Map map = make_fixture(temp.path(), corridor, corridor, true_closing_pose, &keyframes);
  LoopConstraintOptions options = test_options();
  options.max_rmse_m = 0.25;
  options.max_p95_m = 0.40;
  // A perfectly repeated corridor can converge to different, equally good
  // longitudinal shifts in the two directions. Relax the later inverse gate
  // so this regression specifically exercises point-to-plane observability.
  options.max_inverse_translation_m = 5.0;
  options.max_inverse_yaw_rad = kPi;

  const LoopConstraintResult result =
      lingtu::localization::opt::generate_loop_constraints(map, keyframes, options);
  require_point_to_plane_observability_rejection(result, "long corridor slide");
}

void test_gravity_aligned_loop_preserves_body_tilt_relationship() {
  TempMapDir temp("gravity_alignment");
  const std::vector<Point> world = asymmetric_world_cloud();
  const Pose body_from = rpy_pose(0.0, 0.0, 0.0, 10.0 * kPi / 180.0, -4.0 * kPi / 180.0, 0.0);
  const Pose true_body_to =
      rpy_pose(0.45, -0.25, 0.32, -6.0 * kPi / 180.0, 8.0 * kPi / 180.0, 0.27);
  const Pose odom_body_to =
      rpy_pose(0.90, -0.55, 0.70, -6.0 * kPi / 180.0, 8.0 * kPi / 180.0, 0.52);
  std::vector<Keyframe> keyframes = loop_keyframes();
  keyframes.front().pose = body_from;
  keyframes.back().pose = odom_body_to;

  write_pcd(temp.path() / "map.pcd", world);
  write_poses(temp.path() / "poses.txt", keyframes);
  write_pcd(temp.path() / "patches" / keyframes.front().patch_name,
            to_local_cloud(world, body_from));
  for (std::size_t index = 1; index + 1 < keyframes.size(); ++index) {
    write_pcd(temp.path() / "patches" / keyframes[index].patch_name,
              shifted_cloud(world, 30.0 * static_cast<double>(index), 17.0, 4.0));
  }
  write_pcd(temp.path() / "patches" / keyframes.back().patch_name,
            to_local_cloud(world, true_body_to));

  const Map map = lingtu::localization::opt::files(temp.path());
  const LoopConstraintResult result =
      lingtu::localization::opt::generate_loop_constraints(map, keyframes, test_options());
  Require(result.ok, "tilted loop generation failed: " + result.code + ": " + result.message);
  const GeometricConstraint &constraint = require_constraint(result, 0, 4);
  const Pose expected = lingtu::localization::opt::between_poses(body_from, true_body_to);
  require_pose_near(constraint.pose_from_to, expected, 0.24, 0.12, "gravity-aligned T_from_to");
  Require(rotation_distance(constraint.pose_from_to, expected) < 0.14,
          "gravity-aligned loop did not preserve the body roll/pitch relationship");
}

void test_consensus_requires_independent_progress_on_both_sides() {
  const std::vector<Point> world = asymmetric_world_cloud();
  const Pose true_loop_0 = yaw_pose(0.45, -0.25, 0.32, 0.27);

  {
    TempMapDir temp("consensus_same_current");
    std::vector<Keyframe> keyframes = loop_keyframes();
    keyframes[1].pose = yaw_pose(0.15, 0.05, 0.01, 0.02);
    write_pcd(temp.path() / "map.pcd", world);
    write_poses(temp.path() / "poses.txt", keyframes);
    write_pcd(temp.path() / "patches" / keyframes[0].patch_name,
              to_local_cloud(world, keyframes[0].pose));
    write_pcd(temp.path() / "patches" / keyframes[1].patch_name,
              to_local_cloud(world, keyframes[1].pose));
    for (std::size_t index = 2; index + 1 < keyframes.size(); ++index) {
      write_pcd(temp.path() / "patches" / keyframes[index].patch_name,
                shifted_cloud(world, 30.0 * static_cast<double>(index), 17.0, 4.0));
    }
    write_pcd(temp.path() / "patches" / keyframes.back().patch_name,
              to_local_cloud(world, true_loop_0));

    LoopConstraintOptions options = test_options();
    options.min_index_separation = 3;
    options.max_candidates_per_anchor = 3;
    options.max_total_candidates = 8;
    options.require_consensus = true;
    options.min_consistent_matches = 2;
    const LoopConstraintResult result = lingtu::localization::opt::generate_loop_constraints(
        lingtu::localization::opt::files(temp.path()), keyframes, options);
    Require(result.ok, "same-current consensus fixture failed: " + result.code);
    Require(result.report.geometrically_verified_count >= 2,
            "same-current fixture did not create the intended support pairs");
    Require(result.constraints.empty(),
            "one current submap matched to adjacent history was counted as independent support");
    for (const auto &diagnostic : result.report.candidates) {
      Require(!diagnostic.accepted, "same-current consensus candidate was unexpectedly accepted");
    }
  }

  {
    TempMapDir temp("consensus_progress");
    const Pose history_0 = yaw_pose(0.0, 0.0, 0.0, 0.0);
    const Pose history_1 = yaw_pose(0.15, 0.05, 0.01, 0.02);
    const Pose true_loop_1 = yaw_pose(0.60, -0.20, 0.33, 0.29);
    std::vector<Keyframe> keyframes{
        Keyframe{"0000.pcd", history_0},
        Keyframe{"0001.pcd", history_1},
        Keyframe{"0002.pcd", yaw_pose(5.0, 0.0, 0.05, 0.05)},
        Keyframe{"0003.pcd", yaw_pose(5.0, 5.0, 0.10, 1.55)},
        Keyframe{"0004.pcd", yaw_pose(0.90, -0.55, 0.70, 0.52)},
        Keyframe{"0005.pcd", yaw_pose(1.05, -0.50, 0.71, 0.54)},
    };
    write_pcd(temp.path() / "map.pcd", world);
    write_poses(temp.path() / "poses.txt", keyframes);
    write_pcd(temp.path() / "patches" / "0000.pcd", to_local_cloud(world, history_0));
    write_pcd(temp.path() / "patches" / "0001.pcd", to_local_cloud(world, history_1));
    write_pcd(temp.path() / "patches" / "0002.pcd", shifted_cloud(world, 60.0, 17.0, 4.0));
    write_pcd(temp.path() / "patches" / "0003.pcd", shifted_cloud(world, 90.0, 17.0, 4.0));
    write_pcd(temp.path() / "patches" / "0004.pcd", to_local_cloud(world, true_loop_0));
    write_pcd(temp.path() / "patches" / "0005.pcd", to_local_cloud(world, true_loop_1));

    LoopConstraintOptions options = test_options();
    options.min_index_separation = 4;
    options.max_candidates_per_anchor = 3;
    options.max_total_candidates = 8;
    options.require_consensus = true;
    options.min_consistent_matches = 2;
    const LoopConstraintResult result = lingtu::localization::opt::generate_loop_constraints(
        lingtu::localization::opt::files(temp.path()), keyframes, options);
    Require(result.ok, "progressing consensus fixture failed: " + result.code);
    Require(!result.constraints.empty(),
            "independent 0->4 and 1->5 loop progression did not pass consensus");
    bool has_accepted = false;
    bool has_support = false;
    for (const auto &diagnostic : result.report.candidates) {
      has_accepted = has_accepted || diagnostic.accepted;
      has_support = has_support || diagnostic.reason == "consensus_support";
    }
    Require(has_accepted && has_support,
            "progressing consensus did not preserve accepted/support diagnostics");
  }
}

void test_nonoverlap_and_degenerate_geometry_are_rejected() {
  const std::vector<Point> asymmetric = asymmetric_world_cloud();
  const Pose true_closing_pose = yaw_pose(0.45, -0.25, 0.32, 0.27);

  {
    TempMapDir temp("nonoverlap");
    std::vector<Keyframe> keyframes;
    const Map map = make_fixture(temp.path(), asymmetric, shifted_cloud(asymmetric, 8.0, -7.0, 3.0),
                                 true_closing_pose, &keyframes);
    LoopConstraintOptions options = test_options();
    options.max_correspondence_distance_m = 0.45;
    const LoopConstraintResult result =
        lingtu::localization::opt::generate_loop_constraints(map, keyframes, options);
    Require(result.ok, "non-overlap shadow run failed unexpectedly: " + result.code);
    Require(result.report.candidate_count >= 1, "non-overlap case did not reach verification");
    Require(result.constraints.empty(), "non-overlapping clouds produced a loop constraint");
    Require(result.report.rejected_count >= 1, "non-overlap rejection was not reported");
  }

  {
    TempMapDir temp("degenerate");
    const std::vector<Point> plane = planar_cloud();
    std::vector<Keyframe> keyframes;
    const Map map = make_fixture(temp.path(), plane, plane, true_closing_pose, &keyframes);
    LoopConstraintOptions options = test_options();
    options.min_z_span_m = 0.50;
    LoopConstraintResult result =
        lingtu::localization::opt::generate_loop_constraints(map, keyframes, options);
    Require(result.ok, "degenerate shadow run failed unexpectedly: " + result.code);
    Require(result.report.candidate_count >= 1, "degenerate case did not reach verification");
    Require(result.constraints.empty(), "planar-degenerate geometry produced a loop constraint");
    Require(result.report.rejected_count >= 1, "degenerate rejection was not reported");
    Require(!result.report.candidates.empty(), "degenerate candidate diagnostic is missing");
    result.report.candidates.front().hessian_condition = std::numeric_limits<double>::infinity();
    std::string report_error;
    const auto report_path = temp.path() / "degenerate_loop_constraints.json";
    Require(lingtu::localization::opt::write_loop_constraints_report(report_path, result,
                                                                     &report_error),
            "degenerate report write failed: " + report_error);
    const std::string report = read_bytes(report_path);
    Require(report.find("\"hessian_condition\":null") != std::string::npos,
            "non-finite degeneracy metric was not serialized as JSON null");
  }
}

void require_deterministic_diagnostic(const LoopCandidateDiagnostic &first,
                                      const LoopCandidateDiagnostic &second) {
  Require(first.from_index == second.from_index, "candidate from_index is nondeterministic");
  Require(first.to_index == second.to_index, "candidate to_index is nondeterministic");
  Require(first.accepted == second.accepted, "candidate acceptance is nondeterministic");
  Require(first.reason == second.reason, "candidate rejection reason is nondeterministic");
  Require(first.inliers == second.inliers, "candidate inlier count is nondeterministic");
  Require(first.iterations == second.iterations, "candidate ICP iterations are nondeterministic");
  const auto same_double = [](double a, double b) { return std::abs(a - b) <= 1.0e-12; };
  Require(same_double(first.descriptor_score, second.descriptor_score),
          "descriptor score is nondeterministic");
  Require(same_double(first.descriptor_margin, second.descriptor_margin),
          "descriptor place margin is nondeterministic");
  Require(same_double(first.descriptor_yaw_margin, second.descriptor_yaw_margin),
          "descriptor yaw margin is nondeterministic");
  Require(same_double(first.yaw_hypothesis_rad, second.yaw_hypothesis_rad),
          "yaw hypothesis is nondeterministic");
  Require(same_double(first.rmse_m, second.rmse_m), "ICP RMSE is nondeterministic");
  Require(same_double(first.p95_m, second.p95_m), "ICP p95 is nondeterministic");
  Require(same_double(first.hessian_condition, second.hessian_condition),
          "ICP Hessian condition is nondeterministic");
}

void test_named_patch_mismatch_fails_closed() {
  TempMapDir temp("named_patch_mismatch");
  const std::vector<Point> world = asymmetric_world_cloud();
  const Pose true_closing_pose = yaw_pose(0.45, -0.25, 0.32, 0.27);
  std::vector<Keyframe> keyframes;
  const Map map = make_fixture(temp.path(), world, world, true_closing_pose, &keyframes);
  keyframes.back().patch_name = "missing_named_patch.pcd";

  const LoopConstraintResult result =
      lingtu::localization::opt::generate_loop_constraints(map, keyframes, test_options());
  Require(!result.ok, "missing named patch silently fell back to an ordinal cloud");
  Require(result.code == "loop_verification_failed",
          "missing named patch returned the wrong failure code: " + result.code);
  Require(result.message.find("missing named patch for keyframe") != std::string::npos,
          "missing named patch failure did not identify the keyframe");
}

void test_invalid_options_and_keyframes_fail_closed() {
  TempMapDir temp("invalid_inputs");
  const std::vector<Point> world = asymmetric_world_cloud();
  const Pose true_closing_pose = yaw_pose(0.45, -0.25, 0.32, 0.27);
  std::vector<Keyframe> keyframes;
  const Map map = make_fixture(temp.path(), world, world, true_closing_pose, &keyframes);

  LoopConstraintOptions invalid_options = test_options();
  invalid_options.candidate_xy_radius_m = std::numeric_limits<double>::quiet_NaN();
  const LoopConstraintResult invalid_option_result =
      lingtu::localization::opt::generate_loop_constraints(map, keyframes, invalid_options);
  Require(!invalid_option_result.ok, "NaN loop option failed open");
  Require(invalid_option_result.code == "invalid_loop_options",
          "NaN loop option returned the wrong code");

  invalid_options = test_options();
  invalid_options.max_total_candidates = 4097;
  const LoopConstraintResult invalid_budget_result =
      lingtu::localization::opt::generate_loop_constraints(map, keyframes, invalid_options);
  Require(!invalid_budget_result.ok, "unbounded candidate budget failed open");
  Require(invalid_budget_result.code == "invalid_loop_options",
          "candidate budget returned the wrong code");

  invalid_options = test_options();
  invalid_options.max_correspondence_distance_m = 1.0e300;
  const LoopConstraintResult invalid_scale_result =
      lingtu::localization::opt::generate_loop_constraints(map, keyframes, invalid_options);
  Require(!invalid_scale_result.ok, "unbounded physical scale failed open");
  Require(invalid_scale_result.code == "invalid_loop_options",
          "unbounded physical scale returned the wrong code");

  std::vector<Keyframe> invalid_keyframes = keyframes;
  invalid_keyframes.back().pose.qw = 0.0;
  invalid_keyframes.back().pose.qx = 0.0;
  invalid_keyframes.back().pose.qy = 0.0;
  invalid_keyframes.back().pose.qz = 0.0;
  const LoopConstraintResult invalid_pose_result =
      lingtu::localization::opt::generate_loop_constraints(map, invalid_keyframes, test_options());
  Require(!invalid_pose_result.ok, "zero quaternion keyframe failed open");
  Require(invalid_pose_result.code == "invalid_loop_keyframes",
          "zero quaternion keyframe returned the wrong code");

  std::vector<Keyframe> traversal_keyframes = keyframes;
  traversal_keyframes.back().patch_name = "../outside.pcd";
  const LoopConstraintResult traversal_result =
      lingtu::localization::opt::generate_loop_constraints(map, traversal_keyframes,
                                                           test_options());
  Require(!traversal_result.ok, "patch path traversal failed open");
  Require(traversal_result.code == "loop_verification_failed",
          "patch path traversal returned the wrong code");
}

void test_repeated_generation_is_deterministic() {
  TempMapDir temp("determinism");
  const std::vector<Point> world = asymmetric_world_cloud();
  const Pose true_closing_pose = yaw_pose(0.45, -0.25, 0.32, 0.27);
  std::vector<Keyframe> keyframes;
  const Map map = make_fixture(temp.path(), world, world, true_closing_pose, &keyframes);
  const LoopConstraintOptions options = test_options();

  const LoopConstraintResult first =
      lingtu::localization::opt::generate_loop_constraints(map, keyframes, options);
  const LoopConstraintResult second =
      lingtu::localization::opt::generate_loop_constraints(map, keyframes, options);
  Require(first.ok && second.ok, "determinism fixture generation failed");
  Require(first.code == second.code, "loop result code is nondeterministic");
  Require(first.report.poses_fingerprint == second.report.poses_fingerprint,
          "poses fingerprint is nondeterministic");
  Require(first.report.patches_fingerprint == second.report.patches_fingerprint,
          "patches fingerprint is nondeterministic");
  Require(first.constraints.size() == second.constraints.size(),
          "constraint count is nondeterministic");
  Require(first.report.candidates.size() == second.report.candidates.size(),
          "candidate count is nondeterministic");

  for (std::size_t i = 0; i < first.constraints.size(); ++i) {
    Require(first.constraints[i].from_index == second.constraints[i].from_index,
            "constraint from_index is nondeterministic");
    Require(first.constraints[i].to_index == second.constraints[i].to_index,
            "constraint to_index is nondeterministic");
    require_pose_near(first.constraints[i].pose_from_to, second.constraints[i].pose_from_to,
                      1.0e-12, 1.0e-12, "repeated loop constraint");
  }
  for (std::size_t i = 0; i < first.report.candidates.size(); ++i) {
    require_deterministic_diagnostic(first.report.candidates[i], second.report.candidates[i]);
  }
}

void test_shadow_report_preserves_source_artifacts() {
  TempMapDir temp("shadow");
  const std::vector<Point> world = asymmetric_world_cloud();
  const Pose true_closing_pose = yaw_pose(0.45, -0.25, 0.32, 0.27);
  std::vector<Keyframe> keyframes;
  const Map map = make_fixture(temp.path(), world, world, true_closing_pose, &keyframes);
  const std::string map_before = read_bytes(map.map_pcd);
  const std::string poses_before = read_bytes(map.poses_txt);

  const LoopConstraintResult result =
      lingtu::localization::opt::generate_loop_constraints(map, keyframes, test_options());
  Require(result.ok, "shadow generation failed: " + result.code);
  std::string error;
  const std::filesystem::path report_path = temp.path() / "loop_constraints.json";
  Require(lingtu::localization::opt::write_loop_constraints_report(report_path, result, &error),
          "shadow report write failed: " + error);
  Require(std::filesystem::is_regular_file(report_path), "shadow report was not created");

  Require(read_bytes(map.map_pcd) == map_before, "shadow loop detection changed map.pcd bytes");
  Require(read_bytes(map.poses_txt) == poses_before,
          "shadow loop detection changed poses.txt bytes");
  const std::string report = read_bytes(report_path);
  Require(report.find("lingtu.loop_constraints.shadow.v3") != std::string::npos,
          "shadow report schema is missing");
  Require(report.find("constraint=T_from_to") != std::string::npos,
          "shadow report frame convention is missing");
  Require(report.find("shadow_only;information_diagonal=zero;not_graph_compatible") !=
              std::string::npos,
          "shadow report did not mark information as graph-incompatible");

  const std::string report_before = read_bytes(report_path);
  std::string overwrite_error;
  Require(!lingtu::localization::opt::write_loop_constraints_report(report_path, result,
                                                                    &overwrite_error),
          "shadow report unexpectedly overwrote an existing audit artifact");
  Require(read_bytes(report_path) == report_before,
          "failed report overwrite changed the existing audit artifact");

  std::string reserved_error;
  Require(!lingtu::localization::opt::write_loop_constraints_report(map.map_pcd, result,
                                                                    &reserved_error),
          "shadow report writer overwrote map.pcd");
  Require(read_bytes(map.map_pcd) == map_before, "reserved report target changed map.pcd bytes");
}

}  // namespace

int main() {
  try {
    test_point_to_plane_observability_accepts_asymmetric_three_surface_and_transform_direction();
    test_point_to_plane_observability_rejects_long_corridor_slide();
    test_point_to_plane_observability_rejects_single_vertical_wall_slide();
    test_gravity_aligned_loop_preserves_body_tilt_relationship();
    test_consensus_requires_independent_progress_on_both_sides();
    test_nonoverlap_and_degenerate_geometry_are_rejected();
    test_repeated_generation_is_deterministic();
    test_shadow_report_preserves_source_artifacts();
    test_named_patch_mismatch_fails_closed();
    test_invalid_options_and_keyframes_fail_closed();
    std::cout << "loop constraint shadow tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "loop constraint shadow test failed: " << error.what() << "\n";
    return 1;
  }
}
