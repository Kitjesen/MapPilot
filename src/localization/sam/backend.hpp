#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <array>
#include <ostream>
#include <string>
#include <vector>

namespace lingtu::localization::sam {
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

// Upstream odometry settings plus explicit candidate loop-verification tuning.
struct Config {
  double radius_m = 15.0;
  double min_time_s = 30.0;
  int submap_half_window = 25;
  double voxel_m = 0.4;
  double max_fitness = 0.3;
  // Candidate defaults for geometric verification, recorded with every save.
  // These are bounded tuning values, not calibrated sensor covariances.
  double correspondence_m = 1.0;
  double inlier_distance_m = 0.4;
  double min_overlap = 0.5;
  double translation_sigma_m = 0.1;
  double rotation_sigma_rad = 0.05;
  double huber_k = 1.345;
  std::array<double, 6> odom_variance{1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4};
};

struct Frame {
  double stamp_s;
  gtsam::Pose3 odom;
  Cloud::ConstPtr cloud;
};

struct LoopResult {
  std::size_t current = 0;
  double stamp_s = 0;
  bool accepted = false;
  int previous = -1;
  std::string reason = "no_candidate";
  std::vector<std::size_t> target_frames;
  double fitness = -1;
  double overlap = 0;
  double inlier_rmse_m = -1;
  double correction_translation_m = 0;
  double correction_rotation_rad = 0;
  std::array<double, 6> variance{};
  gtsam::Pose3 measurement;
  double elapsed_ms = 0;
  std::size_t source_points = 0, target_points = 0;
};

// Shared by production factors and the false-loop regression test.
gtsam::SharedNoiseModel loopNoise(const Config& config,
                                const std::array<double, 6>& variance);
void writeLoopEvidence(std::ostream& out, const Config& config,
                       const std::vector<LoopResult>& records);

// Single background-worker owner. Inputs must be continuous, uncorrected LIO
// keyframes with body-local deskewed clouds; never previously optimized poses.
class Backend {
 public:
  explicit Backend(Config config = {});
  LoopResult append(Frame frame);
  const std::vector<gtsam::Pose3>& poses() const { return poses_; }
  std::size_t loops() const { return loops_; }
  double error() const;
  const std::vector<LoopResult>& records() const { return records_; }
  Cloud::ConstPtr cloud(std::size_t index) const { return frames_.at(index).cloud; }
  std::size_t cloudBytes() const;

 private:
  Cloud::Ptr submap(const std::vector<std::size_t>& indices) const;
  LoopResult closeLoop();
  void updateEstimates();
  Config config_;
  gtsam::ISAM2 isam_;
  std::vector<Frame> frames_;
  std::vector<Cloud::ConstPtr> filtered_;
  std::vector<LoopResult> records_;
  std::vector<gtsam::Pose3> poses_;
  std::size_t loops_ = 0;
};
}  // namespace lingtu::localization::sam
