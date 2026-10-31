#pragma once
#include "navigation/stairs.hpp"
#include <cmath>
#include <string>

namespace lingtu::nav::endpoint {

struct StairProfileState {
  double sourceWallS{0.0};
  double receivedS{0.0};
  std::string taskId;
  std::string requirement;
  std::string phase;
  std::string requestedProfile;
  std::string activeProfile;
  std::string reason;
};

inline bool currentStairProfile(const StairProfileState& state, const std::string& task_id,
                                double wall_s, double steady_s) {
  return !task_id.empty() && state.taskId == task_id &&
      std::isfinite(state.sourceWallS) && state.sourceWallS > 0.0 &&
      wall_s >= state.sourceWallS - 0.1 && wall_s - state.sourceWallS <= 1.0 &&
      steady_s >= state.receivedS && steady_s - state.receivedS <= 1.0;
}

inline navigation::StairProfileFeedback stairProfileFeedback(
    const StairProfileState& state, const std::string& task_id,
    double wall_s, double steady_s, double execution_s) {
  navigation::StairProfileFeedback feedback;
  if (!currentStairProfile(state, task_id, wall_s, steady_s)) return feedback;
  using Profile = navigation::StairProfile;
  feedback.requirement = state.requirement == "stairs_up" ? Profile::Ascent
      : state.requirement == "stairs_down" ? Profile::Descent
      : state.requirement == "restore" ? Profile::Restore : Profile::None;
  feedback.ready = state.phase == "ready" && !state.requestedProfile.empty() &&
      state.activeProfile == state.requestedProfile;
  feedback.unsupported = state.phase == "unsupported";
  feedback.activeProfile = state.activeProfile;
  feedback.reason = state.reason;
  feedback.stampS = execution_s;
  return feedback;
}

inline bool expectedProfileTransition(const StairProfileState& state,
    const std::string& task_id, const std::string& driver_reason,
    double wall_s, double steady_s, double request_sent_s = -1.0) {
  if (driver_reason != "profile_transition" || task_id.empty()) return false;
  // Control and profile feedback are independent DDS streams. An outstanding
  // request explains the planned zero-speed transition before its first reply;
  // stair entry still requires the separate actual-model readback above.
  if (request_sent_s >= 0.0 && steady_s >= request_sent_s &&
      steady_s - request_sent_s <= 1.0) return true;
  if (!currentStairProfile(state, task_id, wall_s, steady_s)) return false;
  return state.phase == "grounding" || state.phase == "switching" ||
      state.phase == "standing" || state.phase == "waiting" || state.phase == "ready";
}

}  // namespace lingtu::nav::endpoint
