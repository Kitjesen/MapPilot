#include "map_tracking_health.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace lingtu::slam;

namespace {

void Require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  MapTrackingHealthInput disabled;
  const auto passthrough = ProjectMapTrackingHealth(
      SlamState::Tracking, 0.9, "tracking", disabled);
  Require(passthrough.state == SlamState::Tracking, "disabled_state_changed");
  Require(std::abs(passthrough.confidence - 0.9) < 1e-9, "disabled_confidence_changed");

  MapTrackingHealthInput pending;
  pending.enabled = true;
  const auto initial = ProjectMapTrackingHealth(
      SlamState::Tracking, 1.0, "tracking", pending);
  Require(initial.state == SlamState::Localizing, "initial_alignment_false_positive");
  Require(initial.confidence == 0.0, "initial_alignment_confidence_nonzero");

  MapTrackingHealthInput repeated = pending;
  repeated.consecutive_failures = 3;
  const auto failed = ProjectMapTrackingHealth(
      SlamState::Tracking, 1.0, "tracking", repeated);
  Require(failed.state == SlamState::Degraded, "repeated_failures_not_degraded");
  Require(failed.reason == "map_tracking_initial_alignment_failed", "initial_failure_reason");

  MapTrackingHealthInput recent = pending;
  recent.successes = 2U;
  recent.last_success_age_s = 8.0;
  const auto healthy = ProjectMapTrackingHealth(
      SlamState::Tracking, 0.8, "tracking", recent);
  Require(healthy.state == SlamState::Tracking, "recent_success_degraded");
  Require(std::abs(healthy.confidence - 0.8) < 1e-9, "recent_confidence_changed");

  MapTrackingHealthInput repeated_after_success = recent;
  repeated_after_success.consecutive_failures = 3;
  const auto repeated_result = ProjectMapTrackingHealth(
      SlamState::Tracking, 1.0, "tracking", repeated_after_success);
  Require(repeated_result.state == SlamState::Degraded, "tracking_failures_not_degraded");
  Require(repeated_result.reason == "map_tracking_repeated_failures", "tracking_failure_reason");

  MapTrackingHealthInput stale = recent;
  stale.last_success_age_s = 21.0;
  const auto stale_result = ProjectMapTrackingHealth(
      SlamState::Tracking, 1.0, "tracking", stale);
  Require(stale_result.state == SlamState::Degraded, "stale_success_not_degraded");
  Require(stale_result.reason == "map_tracking_stale", "stale_reason");
  Require(stale_result.confidence == 0.0, "stale_confidence_nonzero");

  const auto lost = ProjectMapTrackingHealth(
      SlamState::Lost, 0.0, "backend_lost", stale);
  Require(lost.state == SlamState::Lost, "terminal_backend_state_overridden");
  Require(lost.reason == "backend_lost", "terminal_backend_reason_overridden");

  std::cout << "map_tracking_health: PASS\n";
  return 0;
}
