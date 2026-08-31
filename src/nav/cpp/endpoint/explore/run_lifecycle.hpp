#pragma once

#include <cstdint>
#include <string>

#include "explore/run_event_outbox.hpp"

namespace lingtu::nav::endpoint {

struct ExplorationRunBinding {
  std::string exploration_run_id;
  std::string start_request_id;
  std::string product_session_id;
  std::string route;
  std::string map_id;
  std::int64_t map_content_epoch{0};

  [[nodiscard]] bool valid() const noexcept;
};

// Small DDS-free run FSM. ACK admission is handled by ExploreControl; this
// class records lifecycle truth and refuses paused/terminal facts until the
// caller supplies a matched post-stop goal/segment terminal.
class ExplorationRunLifecycle final {
 public:
  explicit ExplorationRunLifecycle(ExplorationRunEventOutbox &outbox) : outbox_(outbox) {}

  [[nodiscard]] bool start(const ExplorationRunBinding &binding, double timestamp_s,
                           const std::string &reason);
  [[nodiscard]] bool pause(const std::string &command_request_id, double timestamp_s,
                           const std::string &reason, bool motion_pending);
  [[nodiscard]] bool resume(const std::string &command_request_id, double timestamp_s,
                            const std::string &reason);
  [[nodiscard]] bool cancel(const std::string &command_request_id, double timestamp_s,
                            const std::string &reason, bool motion_pending);
  [[nodiscard]] bool complete(double timestamp_s, const std::string &reason);
  [[nodiscard]] bool fail(double timestamp_s, const std::string &reason, bool motion_pending);
  [[nodiscard]] bool confirmMotionStop(double timestamp_s,
                                       const std::string &motion_stop_reason);
  [[nodiscard]] bool recordStopConfirmationFailure(double timestamp_s,
                                                   const std::string &reason);

  [[nodiscard]] bool active() const noexcept { return active_; }
  [[nodiscard]] bool stopConfirmationPending() const noexcept {
    return pending_terminal_ != PendingTerminal::kNone;
  }
  [[nodiscard]] lingtu::message::ExplorationRunState state() const noexcept { return state_; }
  [[nodiscard]] const ExplorationRunBinding &binding() const noexcept { return binding_; }

 private:
  enum class PendingTerminal : std::uint8_t { kNone, kPaused, kCancelled, kFailed };

  [[nodiscard]] ExplorationRunEventRecord event(
      lingtu::message::ExplorationRunEventKind kind,
      lingtu::message::ExplorationRunState state, const std::string &command_request_id,
      double timestamp_s, const std::string &reason, bool motion_stop_confirmed = false,
      std::string motion_stop_reason = {}) const;
  [[nodiscard]] bool record(const ExplorationRunEventRecord &event);
  [[nodiscard]] bool recordMotionStop(const ExplorationRunEventRecord &event);
  [[nodiscard]] bool finishWithoutMotion(lingtu::message::ExplorationRunState state,
                                         const std::string &command_request_id,
                                         double timestamp_s, const std::string &reason,
                                         bool motion_stop_transition = false);

  ExplorationRunEventOutbox &outbox_;
  ExplorationRunBinding binding_;
  bool active_{false};
  lingtu::message::ExplorationRunState state_{lingtu::message::ExplorationRunState::kCancelled};
  PendingTerminal pending_terminal_{PendingTerminal::kNone};
  std::string pending_command_request_id_;
  std::string pending_reason_;
  bool stop_confirmation_failure_recorded_{false};
};

}  // namespace lingtu::nav::endpoint
