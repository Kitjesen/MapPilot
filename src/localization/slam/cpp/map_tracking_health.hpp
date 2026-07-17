#pragma once

#include "slam.hpp"

#include <algorithm>
#include <cstdint>
#include <string>

namespace lingtu::slam {

struct MapTrackingHealthInput {
  bool enabled = false;
  double period_s = 5.0;
  int consecutive_failures = 0;
  std::uint64_t successes = 0U;
  double last_success_age_s = -1.0;
  int degraded_failure_count = 3;
};

struct MapTrackingHealthProjection {
  SlamState state = SlamState::Unconfigured;
  double confidence = 0.0;
  std::string reason;
  bool degraded = false;
  double stale_after_s = 0.0;
};

inline MapTrackingHealthProjection ProjectMapTrackingHealth(
    SlamState backend_state,
    double backend_confidence,
    const std::string& backend_reason,
    const MapTrackingHealthInput& tracking) {
  MapTrackingHealthProjection out;
  out.state = backend_state;
  out.confidence = backend_confidence;
  out.reason = backend_reason;
  out.stale_after_s = std::max(10.0, 3.0 * std::max(1.0, tracking.period_s));
  if (!tracking.enabled) {
    return out;
  }

  const bool backend_terminal =
      backend_state == SlamState::Lost || backend_state == SlamState::Failed;
  if (backend_terminal) {
    return out;
  }

  const int failure_limit = std::max(1, tracking.degraded_failure_count);
  if (tracking.successes == 0U) {
    out.confidence = 0.0;
    if (tracking.consecutive_failures >= failure_limit) {
      out.state = SlamState::Degraded;
      out.reason = "map_tracking_initial_alignment_failed";
      out.degraded = true;
    } else {
      out.state = SlamState::Localizing;
      out.reason = "map_tracking_initial_alignment_pending";
    }
    return out;
  }

  if (tracking.consecutive_failures >= failure_limit) {
    out.state = SlamState::Degraded;
    out.confidence = 0.0;
    out.reason = "map_tracking_repeated_failures";
    out.degraded = true;
    return out;
  }
  if (tracking.last_success_age_s < 0.0 ||
      tracking.last_success_age_s > out.stale_after_s) {
    out.state = SlamState::Degraded;
    out.confidence = 0.0;
    out.reason = "map_tracking_stale";
    out.degraded = true;
  }
  return out;
}

}  // namespace lingtu::slam
