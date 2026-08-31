#pragma once

#include "localization/opt/graph.hpp"
#include "localization/opt/loop_constraints.hpp"
#include "localization/opt/map.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace lingtu::localization::opt {

namespace detail {

std::vector<GeometricConstraint> merge_pose_graph_constraints(
    const std::vector<GeometricConstraint> &sequential,
    std::vector<GeometricConstraint> loops);

}  // namespace detail

struct PoseGraphConstraintAssembly {
  bool ready = false;
  bool evidence_insufficient = false;
  std::string code;
  std::string message;
  std::size_t pose_count = 0;
  std::size_t sequential_count = 0;
  std::size_t loop_count = 0;
  std::vector<GeometricConstraint> constraints;
};

PoseGraphConstraintAssembly assemble_pose_graph_constraints(
    const Map &map, const LoopConstraintOptions &options = {});

bool write_pose_graph_constraints_atomic(const std::filesystem::path &path,
                                         const std::vector<GeometricConstraint> &constraints,
                                         std::string *error = nullptr);

}  // namespace lingtu::localization::opt
