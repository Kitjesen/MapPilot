#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "icp_localizer.h"

namespace lingtu::slam {

struct MapIcpDiagnostics {
  double quality = -1.0;
  std::string refine_backend;
  int refine_iterations = -1;
  int refine_inliers = -1;
  int input_points = 0;
  int evaluated_points = 0;
  double support_ratio = -1.0;
  double overlap_inlier_ratio = -1.0;
  bool refine_converged = false;
  double refine_pos_cov_trace = -1.0;
};

struct MapIcpResult {
  bool success = false;
  std::string message;
  M4F map_body = M4F::Identity();
  std::uint64_t map_generation = 0;
  MapIcpDiagnostics diagnostics;
};

class MapIcp {
 public:
  explicit MapIcp(const ICPConfig &config = ICPConfig{});

  bool loadMap(const std::string &pcd_path);
  bool loadSemanticMap(const std::string &semantic_map_path);
  bool hasMap() const;
  std::uint64_t mapGeneration() const;
  std::string lastError() const;
  std::string mapSource() const;

  MapIcpResult refine(const CloudType::Ptr &scan_body, const M4F &map_body_guess,
                      std::uint64_t expected_generation);
  MapIcpResult verifySeed(const CloudType::Ptr &scan_body, const M4F &map_body_guess,
                          std::uint64_t expected_generation);

 private:
  static void fillDiagnostics(MapIcpResult &result, const ICPLocalizer &icp);

  mutable std::mutex mutex_;
  ICPConfig config_;
  std::unique_ptr<ICPLocalizer> icp_;
  bool map_loaded_ = false;
  std::uint64_t map_generation_ = 0;
  std::uint64_t artifact_generation_ = 0;
  std::string last_error_;
  std::string map_source_;
};

}  // namespace lingtu::slam
