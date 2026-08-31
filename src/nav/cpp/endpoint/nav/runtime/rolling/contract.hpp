#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lingtu::nav::endpoint {

struct RollingSegmentTarget {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct RollingSegmentCommand {
  double stamp_s{0.0};
  std::string frame_id;
  std::string request_id;
  std::int32_t kind{0};
  std::string session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t minimum_generation{0U};
  RollingSegmentTarget target;
  std::string reason;
};

struct RollingSegmentExecutionGrid {
  double stamp_s{0.0};
  std::string frame_id;
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_z{0.0};
  double origin_qx{0.0};
  double origin_qy{0.0};
  double origin_qz{0.0};
  double origin_qw{1.0};
  std::vector<std::uint8_t> occupancy;
  std::vector<std::uint8_t> terrain_cost;
  std::string session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t generation{0U};
  bool live{false};
  double terrain_risk_stamp_s{0.0};
  bool terrain_risk_ready{false};
  bool payload_complete{false};
};

// P3 is deliberately a separate contract from generic NavigationCommandRequest.
// These values are sent on DDS, so keep them stable and validate every raw value
// at the native process boundary.
enum class ExplorationSegmentCommandKind : std::int32_t {
  kExecute = 1,
  kCancel = 2,
};

enum class ExplorationSegmentState : std::int32_t {
  kAccepted = 1,
  kExecuting = 2,
  kReached = 3,
  kFailed = 4,
  kCancelled = 5,
  kStaleBinding = 6,
};

constexpr bool isKnownExplorationSegmentCommandKind(std::int32_t raw) noexcept {
  return raw == static_cast<std::int32_t>(ExplorationSegmentCommandKind::kExecute) ||
         raw == static_cast<std::int32_t>(ExplorationSegmentCommandKind::kCancel);
}

constexpr bool isKnownExplorationSegmentState(std::int32_t raw) noexcept {
  return raw >= static_cast<std::int32_t>(ExplorationSegmentState::kAccepted) &&
         raw <= static_cast<std::int32_t>(ExplorationSegmentState::kStaleBinding);
}

constexpr bool isTerminalExplorationSegmentState(ExplorationSegmentState state) noexcept {
  return state == ExplorationSegmentState::kReached || state == ExplorationSegmentState::kFailed ||
         state == ExplorationSegmentState::kCancelled ||
         state == ExplorationSegmentState::kStaleBinding;
}

}  // namespace lingtu::nav::endpoint
