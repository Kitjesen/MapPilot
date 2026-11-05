#pragma once

#include <lingtu/settings.hpp>

#include "planning/local/planner.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace lingtu::nav::navigation {

inline std::vector<nav_kernel::RecoveryAction> defaultRecoveryOrder() {
  std::vector<nav_kernel::RecoveryAction> order;
  for (const std::string_view name : ::lingtu::settings::recovery::native_nav::recovery::behavior_order) {
    if (name == "translate") {
      order.push_back(nav_kernel::RecoveryAction::Translate);
    } else if (name == "rotate") {
      order.push_back(nav_kernel::RecoveryAction::Rotate);
    } else {
      throw std::invalid_argument("unknown recovery behavior in settings");
    }
  }
  return order;
}

struct RecoveryConfig {
  double blocked_interval_s{::lingtu::settings::recovery::native_nav::recovery::blocked_interval_s};
  double rotation_timeout_s{::lingtu::settings::recovery::native_nav::recovery::rotation_timeout_s};
  double translation_timeout_s{::lingtu::settings::recovery::native_nav::recovery::translation_timeout_s};
  int max_attempts{::lingtu::settings::recovery::native_nav::recovery::max_attempts};
  double translation_speed_mps{::lingtu::settings::recovery::native_nav::recovery::translation_speed_mps};
  double rotation_rate_rad_s{::lingtu::settings::recovery::native_nav::recovery::rotation_rate_rad_s};
  double min_rotation_rad{::lingtu::settings::recovery::native_nav::recovery::min_rotation_rad};
  double max_rotation_rad{::lingtu::settings::recovery::native_nav::recovery::max_rotation_rad};
  double rotation_candidate_step_rad{::lingtu::settings::recovery::native_nav::recovery::rotation_candidate_step_rad};
  double rotation_sample_step_rad{::lingtu::settings::recovery::native_nav::recovery::rotation_sample_step_rad};
  std::vector<nav_kernel::RecoveryAction> behavior_order = defaultRecoveryOrder();
  double stuck_linear_progress_m{0.05};
  double stuck_yaw_progress_rad{0.05};
};

// Owns temporary recovery goals, retries and odometry-based completion.
// SCAN plans translation trajectories; rotation and CMU retain geometric checks.
struct RecoveryOutput {
  int state{0};  // 0 idle/searching, 1 rotating, 2 translating
  bool active{false};
  bool exhausted{false};
  bool verified{false};
  bool direct_command{false};
  nav_kernel::RecoveryAction action{nav_kernel::RecoveryAction::None};
  std::string reason{"inactive"};
  double progress{0.0};
  int attempt{0};
  int candidate_count{0};
  int rotation_direction{0};
  double rotation_target_rad{0.0};
  std::vector<nav_kernel::Vec3> path_body;
  std::vector<nav_kernel::Vec3> path_world;
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
