#pragma once

#include <optional>
#include <string>

#include "command/batch.hpp"
#include "input/frame.hpp"

namespace lingtu::nav::endpoint {

template <typename T>
struct ParsedCommand {
  T value{};
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

struct GoalTarget {
  nav_kernel::Vec3 position{};
  std::optional<double> yaw;
};

ParsedCommand<GoalTarget> parseGoal(const GoalSample &sample,
                                    const std::optional<RigidTransform> &map_odom);
ParsedCommand<nav_kernel::Twist> parseMotion(const OperatorMotionInputSample &sample);
std::string sourceStampError(const std::string &prefix, double source_stamp_s, double receive_s,
                             double max_age_s, double future_tolerance_s);

}  // namespace lingtu::nav::endpoint
