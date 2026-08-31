#pragma once

#include <optional>
#include <string>
#include <vector>

#include "message/cpp/navigation_command.hpp"

namespace lingtu::nav::endpoint {

struct ExploreGoalTarget {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct ExploreGoalCommandBinding {
  std::string task_id;
  std::string start_request_id;
  ExploreGoalTarget target;
};

struct ExploreGoalCommandAck {
  std::string task_id;
  std::string request_id;
  lingtu::message::NavigationCommandKind kind{lingtu::message::NavigationCommandKind::Goal};
  bool accepted{false};
  std::string reason;
};

struct ExploreGoalCommandWrite {
  std::string task_id;
  std::string request_id;
  lingtu::message::NavigationCommandKind kind{lingtu::message::NavigationCommandKind::Goal};
  ExploreGoalTarget target;
  std::string reason;
};

enum class ExploreGoalCommandDeadline {
  StartAck,
  CancelTerminal,
};

// DDS-free single-goal command lane. Cancellation retries preserve the exact
// command identity so the navigation endpoint can replay one logical request.
class ExploreGoalCommandLane {
 public:
  ExploreGoalCommandLane(double retry_s, double command_timeout_s);

  void beginGoal(std::string task_id, std::string start_request_id, ExploreGoalTarget target,
                 double now_s);
  [[nodiscard]] bool requestCancel(std::string reason, double now_s);
  void finishGoal() noexcept;

  [[nodiscard]] std::vector<ExploreGoalCommandWrite>
  advance(double now_s, bool transport_ready, const std::vector<ExploreGoalCommandAck> &acks);
  [[nodiscard]] std::optional<ExploreGoalCommandDeadline> takeDeadline() noexcept;
  void recordWriteResult(const std::string &request_id, bool written) noexcept;

  [[nodiscard]] const std::optional<ExploreGoalCommandBinding> &binding() const noexcept;
  [[nodiscard]] const std::string &cancelRequestId() const noexcept;

 private:
  double retry_s_{0.0};
  double command_timeout_s_{0.0};
  std::optional<ExploreGoalCommandBinding> binding_;
  std::string cancel_request_id_;
  std::string cancel_reason_;
  std::optional<double> last_start_write_s_;
  std::optional<double> last_cancel_write_s_;
  double phase_started_s_{0.0};
  std::optional<ExploreGoalCommandDeadline> pending_deadline_;
  bool start_written_{false};
  bool start_ack_received_{false};
  bool cancel_acked_{false};
  bool deadline_expired_{false};
};

}  // namespace lingtu::nav::endpoint
