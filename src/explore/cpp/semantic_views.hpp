#pragma once

#include "tare_policy.hpp"

namespace lingtu::explore {

// A processed camera observation, not a LiDAR coverage update. The caller
// retains this history only within one semantic search and map identity.
struct CameraSearchView {
  Pose2D pose;
  double range_m{0.0};
  double horizontal_fov_rad{0.0};
};

struct SemanticSearchHistory {
  ExploreMapIdentity map;
  std::vector<CameraSearchView> views;
};

struct SemanticViewProposals {
  ExploreMapIdentity map;
  std::vector<ExploreCandidate> candidates;
  ExploreDiagnostics diagnostics;
  std::string reason;
  bool geometry_exhausted{false};
};

// Heights are robot reference heights supplied by native terrain geometry,
// in the same grid layout. NaN means unsupported; never substitute z=0.
// This query changes neither the map nor an executing exploration policy.
// Candidates still require native 3D path admission before motion.
[[nodiscard]] SemanticViewProposals ProposeSemanticViews(
    const ExploreInput& input,
    const std::vector<double>& reference_heights_m,
    const SemanticSearchHistory& history,
    TarePolicyConfig config = {},
    const ExploreCancelCheck& cancel = {});

}  // namespace lingtu::explore
