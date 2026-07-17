#include "core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace lingtu::driver {
namespace {

double normalize(double value, double maximum) {
  return std::clamp(value / maximum, -1.0, 1.0);
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
    double wz) {
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
          TimePoint{freshness.receive_boottime}),
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

  last_walk_ = {
      normalize(vx, limits_.max_linear_mps),
      normalize(vy, limits_.max_linear_mps),
      normalize(wz, limits_.max_angular_rps),
  };
  last_command_ = now;
  has_command_ = true;
  active_ = true;
  zero_sent_ = false;
  ++counters_.accepted;
  return Action{last_walk_, ActionReason::Command};
}

std::optional<Action> Core::poll(TimePoint now) {
  if (!has_command_ || !active_ || zero_sent_ || now < last_command_) {
    return std::nullopt;
  }
  if (now - last_command_ < limits_.command_timeout) {
    return std::nullopt;
  }
  ++counters_.watchdog_stops;
  return forceStop(ActionReason::Watchdog);
}

Action Core::forceStop(ActionReason reason) noexcept {
  last_walk_ = {};
  active_ = false;
  zero_sent_ = true;
  return Action{last_walk_, reason};
}

void Core::reset() noexcept {
  last_walk_ = {};
  has_command_ = false;
  active_ = false;
  zero_sent_ = true;
}

std::optional<Action> Core::reject(ActionReason reason) {
  if (!active_ && zero_sent_) {
    return std::nullopt;
  }
  ++counters_.invalid_stops;
  return forceStop(reason);
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
