#include "localization/opt/assembly.hpp"

#include "localization/opt/constraints.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <system_error>

#ifdef _WIN32
#include <windows.h>
#endif

namespace lingtu::localization::opt {
namespace {

constexpr const char *kHeader[] = {
    "LINGTU_PGO_CONSTRAINTS_V1",
    "T_from_to tx ty tz qw qx qy qz",
    "RIGHT_TANGENT omega_x omega_y omega_z upsilon_x upsilon_y upsilon_z",
    "UPPER_TRIANGLE row_major 21",
};

bool publish_replace(const std::filesystem::path &temporary,
                     const std::filesystem::path &destination, std::string &error) {
#ifdef _WIN32
  if (MoveFileExW(temporary.c_str(), destination.c_str(),
                  MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0) {
    return true;
  }
  error = std::system_category().message(static_cast<int>(GetLastError()));
  return false;
#else
  std::error_code rename_error;
  std::filesystem::rename(temporary, destination, rename_error);
  if (!rename_error) {
    return true;
  }
  error = rename_error.message();
  return false;
#endif
}

}  // namespace

namespace detail {

std::vector<GeometricConstraint> merge_pose_graph_constraints(
    const std::vector<GeometricConstraint> &sequential,
    std::vector<GeometricConstraint> loops) {
  std::stable_sort(loops.begin(), loops.end(),
                   [](const GeometricConstraint &lhs, const GeometricConstraint &rhs) {
                     if (lhs.from_index != rhs.from_index) {
                       return lhs.from_index < rhs.from_index;
                     }
                     return lhs.to_index < rhs.to_index;
                   });
  std::vector<GeometricConstraint> merged = sequential;
  merged.reserve(sequential.size() + loops.size());
  merged.insert(merged.end(), loops.begin(), loops.end());
  return merged;
}

}  // namespace detail

PoseGraphConstraintAssembly assemble_pose_graph_constraints(
    const Map &map, const LoopConstraintOptions &options) {
  PoseGraphConstraintAssembly result;
  const Result map_result = check(map);
  if (!map_result.ok) {
    result.code = map_result.code;
    result.message = map_result.message;
    return result;
  }

  std::vector<Keyframe> keyframes;
  try {
    keyframes = read_poses(map.poses_txt);
  } catch (const std::exception &exception) {
    result.code = "poses_read_failed";
    result.message = exception.what();
    return result;
  }
  result.pose_count = keyframes.size();
  if (keyframes.size() < 2) {
    result.evidence_insufficient = true;
    result.code = "insufficient_keyframes";
    result.message = "automatic PGO requires at least two keyframes";
    return result;
  }

  std::vector<GeometricConstraint> sequential_constraints;
  sequential_constraints.reserve(keyframes.size() - 1);
  for (std::size_t from_index = 0; from_index + 1 < keyframes.size(); ++from_index) {
    const auto sequential = generate_sequential_constraint(map, keyframes, from_index, options);
    if (!sequential.ok) {
      result.evidence_insufficient = sequential.code == "sequential_registration_rejected";
      result.code = result.evidence_insufficient ? "sequential_chain_incomplete" : sequential.code;
      result.message = "edge " + std::to_string(from_index) + "->" +
                       std::to_string(from_index + 1) + ": " + sequential.code + ": " +
                       sequential.message;
      result.constraints.clear();
      return result;
    }
    sequential_constraints.push_back(sequential.constraint);
    ++result.sequential_count;
  }

  const auto loops = generate_loop_constraints(map, keyframes, options);
  if (!loops.ok) {
    result.code = "loop_verification_failed";
    result.message = loops.code + ": " + loops.message;
    result.constraints.clear();
    return result;
  }
  if (loops.constraints.empty()) {
    result.evidence_insufficient = true;
    result.code = "no_verified_loops";
    result.message = "automatic PGO requires at least one verified loop constraint";
    result.constraints.clear();
    return result;
  }
  result.loop_count = loops.constraints.size();
  result.constraints = detail::merge_pose_graph_constraints(sequential_constraints, loops.constraints);
  result.ready = true;
  result.code = "constraints_ready";
  result.message = "complete sequential chain and verified loops are ready";
  return result;
}

bool write_pose_graph_constraints_atomic(const std::filesystem::path &path,
                                         const std::vector<GeometricConstraint> &constraints,
                                         std::string *error) {
  std::filesystem::path temporary;
  try {
    if (path.empty()) {
      throw std::runtime_error("constraints path is empty");
    }
    if (constraints.empty()) {
      throw std::runtime_error("refusing to publish an empty constraint graph");
    }
    for (const auto &constraint : constraints) {
      const Pose &pose = constraint.pose_from_to;
      const double quaternion_norm = std::sqrt(pose.qw * pose.qw + pose.qx * pose.qx +
                                               pose.qy * pose.qy + pose.qz * pose.qz);
      if (constraint.from_index == constraint.to_index ||
          !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z) ||
          !std::isfinite(quaternion_norm) || quaternion_norm < 0.9 || quaternion_norm > 1.1 ||
          !valid_information_upper(constraint.information_upper)) {
        throw std::runtime_error("refusing to publish an invalid graph constraint");
      }
    }
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    temporary = path;
    temporary += ".tmp-" + std::to_string(nonce);
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output.is_open()) {
      throw std::runtime_error("failed to open temporary constraints file");
    }
    output << std::setprecision(17);
    for (const char *line : kHeader) {
      output << line << '\n';
    }
    for (const auto &constraint : constraints) {
      const Pose &pose = constraint.pose_from_to;
      output << constraint.from_index << ' ' << constraint.to_index << ' ' << pose.x << ' '
             << pose.y << ' ' << pose.z << ' ' << pose.qw << ' ' << pose.qx << ' ' << pose.qy
             << ' ' << pose.qz;
      for (double value : constraint.information_upper) {
        output << ' ' << value;
      }
      output << '\n';
    }
    output.close();
    if (!output.good()) {
      throw std::runtime_error("failed while closing temporary constraints file");
    }
    std::string publish_error;
    if (!publish_replace(temporary, path, publish_error)) {
      throw std::runtime_error("failed to publish constraints atomically: " + publish_error);
    }
    temporary.clear();
    return true;
  } catch (const std::exception &exception) {
    if (!temporary.empty()) {
      std::error_code cleanup_error;
      std::filesystem::remove(temporary, cleanup_error);
    }
    if (error != nullptr) {
      *error = exception.what();
    }
    return false;
  }
}

}  // namespace lingtu::localization::opt
