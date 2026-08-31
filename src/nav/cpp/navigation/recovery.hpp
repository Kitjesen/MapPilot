#pragma once

#include "planning/local/planner.hpp"
#include "planning/local/recovery.hpp"

#include <memory>
#include <string>
#include <vector>

namespace lingtu::nav::navigation {

struct RecoveryConfig {
  double blocked_interval_s{2.0};
  double rotation_timeout_s{2.5};
  double translation_timeout_s{1.5};
  int max_attempts{3};
  double translation_speed_mps{0.15};
  double rotation_rate_rad_s{0.25};
  double min_rotation_rad{0.20};
  double max_rotation_rad{1.20};
  double rotation_candidate_step_rad{0.20};
  double rotation_sample_step_rad{0.05};
  std::vector<nav_kernel::RecoveryAction> behavior_order{
      nav_kernel::RecoveryAction::Translate,
      nav_kernel::RecoveryAction::Rotate,
  };
  double stuck_linear_progress_m{0.05};
  double stuck_yaw_progress_rad{0.05};
};

// Backend-neutral execution recovery. The selected local Planner remains
// responsible only for producing a path; Executor owns when recovery starts
// and this class owns candidate retries and odometry-based completion.
struct RecoveryOutput {
  int state{0};  // 0 idle/searching, 1 rotating, 2 translating
  bool active{false};
  bool exhausted{false};
  bool verified{false};
  bool direct_command{false};
  bool observation_refresh_required{false};
  nav_kernel::RecoveryAction action{nav_kernel::RecoveryAction::None};
  std::string reason{"inactive"};
  double progress{0.0};
  int attempt{0};
  int candidate_count{0};
  int rotation_direction{0};
  double rotation_target_rad{0.0};
  std::vector<nav_kernel::Vec3> path_body;
};

class Recovery {
 public:
  Recovery(const nav_kernel::LocalPlannerParams& planner_params,
           const RecoveryConfig& config);
  ~Recovery();

  Recovery(Recovery&&) noexcept;
  Recovery& operator=(Recovery&&) noexcept;
  Recovery(const Recovery&) = delete;
  Recovery& operator=(const Recovery&) = delete;

  RecoveryOutput step(const nav_kernel::LocalPlanRequest& request);
  void reset();
  [[nodiscard]] bool active() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::nav::navigation
