#include "core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace lingtu::driver {
namespace {

double clampMagnitude(double value, double maximum) {
  return std::clamp(value, -maximum, maximum);
}

bool validLimits(const Limits& limits) {
  return std::isfinite(limits.max_linear_mps) && limits.max_linear_mps > 0.0 &&
         std::isfinite(limits.max_angular_rps) && limits.max_angular_rps > 0.0 &&
         limits.command_timeout.count() > 0;
}

}  // namespace

Core::Core(Limits limits)
    : Core(std::move(limits), "legacy-host-boot") {}

Core::Core(Limits limits, std::string expected_host_boot_id)
    : limits_(limits),
      freshness_gate_(
          limits.command_timeout,
          std::move(expected_host_boot_id)) {
  if (!validLimits(limits_)) {
    throw std::invalid_argument("driver limits must be finite and positive");
  }
}

std::optional<Action> Core::accept(
    std::string_view frame,
    double vx,
    double vy,
    double wz,
    TimePoint now) {
  ++counters_.received;
  return acceptPayload(frame, vx, vy, wz, now);
}

FreshCommandResult Core::accept(
    const CommandFreshnessInput& freshness,
    std::string_view frame,
    double vx,
    double vy,
    double wz,
    TimePoint received_at) {
  ++counters_.received;
  const auto decision = freshness_gate_.evaluate(freshness);
  if (!decision.accepted) {
    return {decision, std::nullopt};
  }
  return {
      decision,
      acceptPayload(
          frame,
          vx,
          vy,
          wz,
          // Transfer elapsed age, not CLOCK_BOOTTIME's epoch, into the local steady clock.
          received_at - (freshness.receive_boottime - freshness.source_boottime)),
  };
}

std::optional<Action> Core::acceptPayload(
    std::string_view frame,
    double vx,
    double vy,
    double wz,
    TimePoint now) {
  if (!supportedBodyFrame(frame)) {
    ++counters_.rejected_frame;
    return reject(ActionReason::InvalidFrame);
  }
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
    ++counters_.rejected_nonfinite;
    return reject(ActionReason::NonFinite);
  }

  last_velocity_ = {
      clampMagnitude(vx, limits_.max_linear_mps),
      clampMagnitude(vy, limits_.max_linear_mps),
      clampMagnitude(wz, limits_.max_angular_rps),
  };
  last_command_ = now;
  active_ = true;
  ++counters_.accepted;
  pending_ = Action{last_velocity_, ActionReason::Command};
  return pending_;
}

std::optional<Action> Core::poll(TimePoint now) {
  if (!active_ || now < last_command_) {
    return std::nullopt;
  }
  if (now - last_command_ < limits_.command_timeout) {
    return std::nullopt;
  }
  ++counters_.watchdog_stops;
  return forceStop(ActionReason::Watchdog);
}

std::optional<Action> Core::take(TimePoint now) {
  if (auto expired = poll(now)) return expired;
  return std::exchange(pending_, std::nullopt);
}

Action Core::forceStop(ActionReason reason) noexcept {
  last_velocity_ = {};
  active_ = false;
  pending_.reset();
  return Action{last_velocity_, reason};
}

void Core::reset() noexcept {
  last_velocity_ = {};
  active_ = false;
  pending_.reset();
}

std::optional<Action> Core::reject(ActionReason reason) {
  if (!active_) {
    return std::nullopt;
  }
  ++counters_.invalid_stops;
  pending_ = forceStop(reason);
  return pending_;
}

const char* reasonName(ActionReason reason) noexcept {
  switch (reason) {
    case ActionReason::Command:
      return "command";
    case ActionReason::Watchdog:
      return "watchdog";
    case ActionReason::InvalidFrame:
      return "invalid_frame";
    case ActionReason::NonFinite:
      return "nonfinite";
    case ActionReason::ConnectProbe:
      return "connect_probe";
    case ActionReason::Fault:
      return "fault";
    case ActionReason::Shutdown:
      return "shutdown";
  }
  return "unknown";
}

bool supportedBodyFrame(std::string_view frame) noexcept {
  return frame == "body" || frame == "base_link";
}

}  // namespace lingtu::driver
