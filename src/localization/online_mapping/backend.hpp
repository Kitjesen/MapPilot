#pragma once

#include <lingtu/settings.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <array>
#include <ostream>
#include <string>
#include <vector>

namespace lingtu::localization::online_mapping {

namespace loop_defaults = lingtu::settings::slam::slam::online_mapping;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

// Upstream odometry settings plus explicit candidate loop-verification tuning.
struct Config {
  double radius_m = loop_defaults::radius_m;
  double min_time_s = loop_defaults::min_time_s;
  int submap_half_window = loop_defaults::submap_half_window;
  double voxel_m = loop_defaults::voxel_m;
  double max_fitness = loop_defaults::max_fitness;
  // Candidate defaults for geometric verification, recorded with every save.
  // These are bounded tuning values, not calibrated sensor covariances.
  double correspondence_m = loop_defaults::correspondence_m;
  double inlier_distance_m = loop_defaults::inlier_distance_m;
  double min_overlap = loop_defaults::min_overlap;
  double translation_sigma_m = loop_defaults::translation_sigma_m;
  double rotation_sigma_rad = loop_defaults::rotation_sigma_rad;
  double huber_k = loop_defaults::huber_k;
  // Initial engineering values: independent anchor motion and SE(3) cycle error.
  double confirm_distance_m = loop_defaults::confirm_distance_m;
  double consistency_m = loop_defaults::consistency_m;
  double consistency_rad = loop_defaults::consistency_rad;
  std::array<double, 6> odom_variance= loop_defaults::odom_variance;
};

struct Frame {
  double stamp_s;
  gtsam::Pose3 odom;
  Cloud::ConstPtr cloud;
  bool relocalized = false;
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
  int support_current = -1;
  int support_previous = -1;
  double consistency_translation_m = -1;
  double consistency_rotation_rad = -1;
  std::size_t candidate_count = 0, verified_candidates = 0;
  double elapsed_ms = 0;
  std::size_t source_points = 0, target_points = 0;
};

// Shared by production factors and the false-loop regression test.
gtsam::SharedNoiseModel loopNoise(const Config& config,
                                const std::array<double, 6>& variance);
void writeLoopEvidence(std::ostream& out, const Config& config,
                       const std::vector<LoopResult>& records,
                       const std::vector<std::size_t>& segment_starts = {});

// Single background-worker owner. Inputs must be continuous, uncorrected LIO
// keyframes with body-local deskewed clouds; never previously optimized poses.
class Backend {
 public:
  explicit Backend(Config config = {});
  LoopResult append(Frame frame);
  void restore(std::vector<Frame> frames, const std::vector<LoopResult>& records);
  const std::vector<gtsam::Pose3>& poses() const { return poses_; }
  std::size_t loops() const { return loops_; }
  double error() const;
  const std::vector<LoopResult>& records() const { return records_; }
  Cloud::ConstPtr cloud(std::size_t index) const { return frames_.at(index).cloud; }
  std::size_t cloudBytes() const;
  std::vector<std::size_t> segmentStarts() const;

 private:
  LoopResult append(Frame frame, const LoopResult* recorded);
  Cloud::Ptr submap(const std::vector<std::size_t>& indices) const;
  LoopResult closeLoop();
  void updateEstimates();
  Config config_;
  gtsam::ISAM2 isam_;
  std::vector<Frame> frames_;
  std::vector<Cloud::ConstPtr> filtered_;
  std::vector<LoopResult> records_;
  std::vector<LoopResult> pending_;
  std::vector<gtsam::Pose3> poses_;
  std::size_t loops_ = 0;
};
}  // namespace lingtu::localization::online_mapping
