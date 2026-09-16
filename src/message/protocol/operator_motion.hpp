#pragma once

#include "message/generated/enums.hpp"

namespace lingtu::message {

// Stable values carried by OperatorMotionControl.action and
// OperatorMotionAck.action. The Interface is native-first: adapters publish
// intent, while the native endpoint owns authority, freshness, and final
// velocity.

inline bool isKnownOperatorMotionAction(std::int32_t value) noexcept {
  return value == static_cast<std::int32_t>(OperatorMotionAction::Claim) ||
      value == static_cast<std::int32_t>(OperatorMotionAction::Release) ||
      value == static_cast<std::int32_t>(OperatorMotionAction::Hold);
}

inline const char* operatorMotionActionName(
    OperatorMotionAction action) noexcept {
  switch (action) {
    case OperatorMotionAction::Claim:
      return "claim";
    case OperatorMotionAction::Release:
      return "release";
    case OperatorMotionAction::Hold:
      return "hold";
  }
  return "unknown";
}

}  // namespace lingtu::message
