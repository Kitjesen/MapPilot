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

SequentialConstraintResult extend_sequential_graph(
    const PatchCloudSource& cloud_at, const std::vector<Keyframe>& keyframes,
    std::size_t to_index, std::vector<GeometricConstraint>& constraints,
    const LoopConstraintOptions& options) {
  if (to_index == 0 || to_index >= keyframes.size()) {
    SequentialConstraintResult invalid;
    invalid.code = "sequential_index_out_of_range";
    invalid.message = "sequential extension requires an existing predecessor";
    return invalid;
  }
  auto adjacent = generate_sequential_constraint(cloud_at, keyframes, to_index - 1, options);
  if (adjacent.ok) constraints.push_back(adjacent.constraint);
  else if (adjacent.code != "sequential_registration_rejected") return adjacent;
  // A fixed local window bounds extra registration work and avoids a stale anchor.
  for (std::size_t gap = 2; gap <= 4 && gap <= to_index; ++gap) {
    const auto from = to_index - gap;
    const auto component = connected_pose_indices(keyframes.size(), constraints, to_index);
    if (std::binary_search(component.begin(), component.end(), from)) continue;
    const std::vector<Keyframe> pair{keyframes[from], keyframes[to_index]};
    auto bridge = generate_sequential_constraint([&](std::size_t i) {
      return cloud_at(i == 0 ? from : to_index);
    }, pair, 0, options);
    if (bridge.ok) {
      bridge.constraint.from_index = from;
      bridge.constraint.to_index = to_index;
      constraints.push_back(bridge.constraint);
    } else if (bridge.code != "sequential_registration_rejected") return bridge;
  }
  return adjacent;
}

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
  if (map_result.patch_count != keyframes.size()) {
    result.code = "patch_pose_mismatch";
    result.message = "patch count must exactly match keyframe count";
    return result;
  }
  if (keyframes.size() < 2) {
    result.evidence_insufficient = true;
    result.code = "insufficient_keyframes";
    result.message = "automatic PGO requires at least two keyframes";
    return result;
  }

  std::vector<GeometricConstraint> sequential_constraints;
  sequential_constraints.reserve(keyframes.size() - 1);
  for (std::size_t from_index = 0; from_index + 1 < keyframes.size(); ++from_index) {
    const auto sequential = extend_sequential_graph([&](std::size_t i) {
      return read_point_cloud(map.patches_dir / keyframes.at(i).patch_name);
    }, keyframes, from_index + 1, sequential_constraints, options);
    if (!sequential.ok) {
      // A rejected edge does not invalidate later measured edges or a loop bridge.
      if (sequential.code == "sequential_registration_rejected") continue;
      result.code = sequential.code;
      result.message = "edge " + std::to_string(from_index) + "->" +
                       std::to_string(from_index + 1) + ": " + sequential.code + ": " +
                       sequential.message;
      result.constraints.clear();
      return result;
    }
  }
  result.sequential_count = sequential_constraints.size();

  const auto loops = generate_loop_constraints(map, keyframes, options);
  if (!loops.ok) {
    result.code = "loop_verification_failed";
    result.message = loops.code + ": " + loops.message;
    result.constraints.clear();
    return result;
  }
  result.loop_count = loops.constraints.size();
  auto constraints = detail::merge_pose_graph_constraints(sequential_constraints, loops.constraints);
  if (connected_pose_indices(keyframes.size(), constraints).size() != keyframes.size()) {
    result.evidence_insufficient = true;
    result.code = "sequential_chain_incomplete";
    result.message = "verified sequential and loop edges do not connect every saved pose";
    return result;
  }
  if (loops.constraints.empty()) {
    result.evidence_insufficient = true;
    result.code = "no_verified_loops";
    result.message = "automatic PGO requires at least one verified loop constraint";
    result.constraints.clear();
    return result;
  }
  result.constraints = std::move(constraints);
  result.ready = true;
  result.code = "constraints_ready";
  result.message = "connected measured graph with verified loops is ready";
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
