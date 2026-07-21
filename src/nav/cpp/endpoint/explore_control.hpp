#pragma once

#include "message/cpp/exploration_command.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>

namespace lingtu::nav::endpoint {

struct ExplorationControlRequest {
  std::string request_id;
  std::int32_t kind{0};
  std::string session_id;
  std::string reason;
  std::string frame_id;
  double stamp_s{0.0};
  double now_s{0.0};
  double max_age_s{2.0};
  double future_tolerance_s{0.1};
  bool inputs_ready{false};
  bool goal_pending{false};
  bool cancellation_pending{false};
};

struct ExplorationControlResult {
  bool accepted{false};
  bool duplicate{false};
  std::string reason;
  std::string session_id;
  bool reset_planner{false};
  bool clear_queue{false};
  bool clear_history{false};
  bool request_cancel{false};
  std::string cancel_reason;
};

class ExploreControl final {
 public:
  explicit ExploreControl(std::size_t cache_limit = 128U);

  ExplorationControlResult Apply(const ExplorationControlRequest& request);

  [[nodiscard]] bool active() const noexcept { return active_; }
  [[nodiscard]] bool paused() const noexcept { return paused_; }
  [[nodiscard]] bool running() const noexcept { return active_ && !paused_; }
  [[nodiscard]] const std::string& session_id() const noexcept {
    return session_id_;
  }

 private:
  struct AckRecord {
    std::int32_t kind{0};
    bool accepted{false};
    std::string reason;
    std::string session_id;
  };

  ExplorationControlResult Finish(
      const ExplorationControlRequest& request,
      bool accepted,
      std::string reason,
      ExplorationControlResult actions = {});
  void Remember(const std::string& request_id, const AckRecord& record);

  std::size_t cache_limit_{128U};
  bool active_{false};
  bool paused_{false};
  std::string session_id_;
  std::unordered_map<std::string, AckRecord> ack_cache_;
  std::deque<std::string> ack_order_;
};

}  // namespace lingtu::nav::endpoint
