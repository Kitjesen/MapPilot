#pragma once

#include "slam.hpp"

#include <memory>
#include <string>

namespace lingtu::slam {

struct NativeRelocalizationResult {
  bool success = false;
  std::string message;
  Pose3d map_body;
  Pose3d map_odom;
  double quality = -1.0;
  std::string refine_backend;
  int refine_iterations = -1;
  int refine_inliers = -1;
  bool refine_converged = false;
  double refine_pos_cov_trace = -1.0;
};

class NativeRelocalizer {
 public:
  NativeRelocalizer();
  ~NativeRelocalizer();

  NativeRelocalizer(const NativeRelocalizer&) = delete;
  NativeRelocalizer& operator=(const NativeRelocalizer&) = delete;

  bool loadMap(const std::string& pcd_path, std::string* message = nullptr);
  bool hasMap() const;
  bool supportsSeededRelocalization() const;
  bool supportsGlobalRelocalization() const;

  // Registration is safe to run on copied scan/pose inputs while the SLAM
  // estimator continues on its owning thread. MapIcp serializes its mutable
  // localizer and rejects a result if the loaded-map generation changes.
  NativeRelocalizationResult relocalize(
      const Cloud& scan_body,
      const Pose3d& map_body_guess,
      const Pose3d& odom_body) const;

  NativeRelocalizationResult globalRelocalize(
      const Cloud& scan_body,
      const Pose3d& odom_body) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::slam
