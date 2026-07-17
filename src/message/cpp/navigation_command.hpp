#pragma once

#include <cstdint>

namespace lingtu::message {

// Stable values carried by NavigationCommandRequest.kind and
// NavigationCommandAck.kind. The protocol belongs to the message module so
// producers and the authoritative endpoint do not depend on one another.
enum class NavigationCommandKind : std::int32_t {
  Goal = 1,
  Cancel = 2,
  Teleop = 3,
  Stop = 4,
  Estop = 5,
  ClearEstop = 6,
  ResumeAutonomy = 7,
};

inline bool isKnownNavigationCommandKind(std::int32_t value) noexcept {
  return value == static_cast<std::int32_t>(NavigationCommandKind::Goal) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Cancel) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Teleop) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Stop) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::Estop) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::ClearEstop) ||
      value == static_cast<std::int32_t>(NavigationCommandKind::ResumeAutonomy);
}

inline const char* navigationCommandKindName(
    NavigationCommandKind kind) noexcept {
  switch (kind) {
    case NavigationCommandKind::Goal:
      return "goal";
    case NavigationCommandKind::Cancel:
      return "cancel";
    case NavigationCommandKind::Teleop:
      return "teleop";
    case NavigationCommandKind::Stop:
      return "stop";
    case NavigationCommandKind::Estop:
      return "estop";
    case NavigationCommandKind::ClearEstop:
      return "clear_estop";
    case NavigationCommandKind::ResumeAutonomy:
      return "resume_autonomy";
  }
  return "unknown";
}

}  // namespace lingtu::message
