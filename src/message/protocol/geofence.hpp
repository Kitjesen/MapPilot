#pragma once

#include "message/generated/enums.hpp"

namespace lingtu::message {

// Stable values carried by GeofenceCommandRequest.action and
// GeofenceCommandAck.action.

inline bool isKnownGeofenceAction(std::int32_t value) noexcept {
  return value >= static_cast<std::int32_t>(GeofenceAction::kAdd) &&
         value <= static_cast<std::int32_t>(GeofenceAction::kList);
}

inline const char *geofenceActionName(GeofenceAction action) noexcept {
  switch (action) {
    case GeofenceAction::kAdd:
      return "add";
    case GeofenceAction::kRemove:
      return "remove";
    case GeofenceAction::kClear:
      return "clear";
    case GeofenceAction::kEnable:
      return "enable";
    case GeofenceAction::kDisable:
      return "disable";
    case GeofenceAction::kList:
      return "list";
  }
  return "unknown";
}

}  // namespace lingtu::message
