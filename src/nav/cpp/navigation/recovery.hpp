#pragma once

#include <lingtu/settings.hpp>

namespace lingtu::nav::navigation {

// Assisted-teleop boundary departure is the only remaining recovery motion.
struct RecoveryConfig {
  double translation_speed_mps{::lingtu::settings::recovery::native_nav::recovery::translation_speed_mps};
};

}  // namespace lingtu::nav::navigation
