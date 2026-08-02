#pragma once

#include <cstdint>

namespace lingtu::message {

// Stable values carried by InspectionTaskRequest.kind and
// InspectionTaskAck.kind.
enum class InspectionCommandKind : std::int32_t {
  kStart = 1,
  kPause = 2,
  kResume = 3,
  kCancel = 4,
};

inline bool isKnownInspectionCommandKind(std::int32_t value) noexcept {
  return value == static_cast<std::int32_t>(InspectionCommandKind::kStart) ||
      value == static_cast<std::int32_t>(InspectionCommandKind::kPause) ||
      value == static_cast<std::int32_t>(InspectionCommandKind::kResume) ||
      value == static_cast<std::int32_t>(InspectionCommandKind::kCancel);
}

inline const char* inspectionCommandKindName(
    InspectionCommandKind kind) noexcept {
  switch (kind) {
    case InspectionCommandKind::kStart:
      return "start";
    case InspectionCommandKind::kPause:
      return "pause";
    case InspectionCommandKind::kResume:
      return "resume";
    case InspectionCommandKind::kCancel:
      return "cancel";
  }
  return "unknown";
}

}  // namespace lingtu::message
