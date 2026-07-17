#pragma once

#include "command_freshness_gate.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace lingtu::driver {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

struct Limits {
  double max_linear_mps{1.0};
  double max_angular_rps{1.0};
  std::chrono::milliseconds command_timeout{200};
};

struct Walk {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

enum class ActionReason {
  Command,
  Watchdog,
  InvalidFrame,
  NonFinite,
  ConnectProbe,
  Fault,
  Shutdown,
};

struct Action {
  Walk walk;
  ActionReason reason{ActionReason::Command};
};

struct FreshCommandResult {
  CommandFreshnessDecision freshness;
  std::optional<Action> action;
};

struct Counters {
  std::uint64_t received{0};
  std::uint64_t accepted{0};
  std::uint64_t rejected_frame{0};
  std::uint64_t rejected_nonfinite{0};
  std::uint64_t watchdog_stops{0};
  std::uint64_t invalid_stops{0};
};

class Core {
 public:
  explicit Core(Limits limits);
  Core(Limits limits, std::string expected_host_boot_id);

  std::optional<Action> accept(
      std::string_view frame,
      double vx,
      double vy,
      double wz,
      TimePoint now);
  FreshCommandResult accept(
      const CommandFreshnessInput& freshness,
      std::string_view frame,
      double vx,
      double vy,
      double wz);
  std::optional<Action> poll(TimePoint now);
  Action forceStop(ActionReason reason) noexcept;
  void reset() noexcept;

  bool active() const noexcept { return active_; }
  const Walk& lastWalk() const noexcept { return last_walk_; }
  const Counters& counters() const noexcept { return counters_; }
  const Limits& limits() const noexcept { return limits_; }

 private:
  std::optional<Action> acceptPayload(
      std::string_view frame,
      double vx,
      double vy,
      double wz,
      TimePoint now);
  std::optional<Action> reject(ActionReason reason);

  Limits limits_;
  CommandFreshnessGate freshness_gate_;
  Counters counters_;
  Walk last_walk_;
  TimePoint last_command_{};
  bool has_command_{false};
  bool active_{false};
  bool zero_sent_{true};
};

const char* reasonName(ActionReason reason) noexcept;
bool supportedBodyFrame(std::string_view frame) noexcept;

}  // namespace lingtu::driver
