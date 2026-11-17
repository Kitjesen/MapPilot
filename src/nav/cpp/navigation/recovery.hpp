#pragma once

#include <lingtu/settings.hpp>

#include "planning/local/planner.hpp"

#include <stdexcept>
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

}  // namespace lingtu::nav::navigation
