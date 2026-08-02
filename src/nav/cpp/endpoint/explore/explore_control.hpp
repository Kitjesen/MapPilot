#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "message/cpp/exploration_command.hpp"

namespace lingtu::nav::endpoint {

struct ExplorationControlRequest {
  std::string request_id;
  std::string exploration_run_id;
  std::int32_t kind{0};
  std::string session_id;
  std::string expected_session_id;
  std::string reason;
  std::string frame_id;
  double stamp_s{0.0};
  double now_s{0.0};
  double max_age_s{2.0};
  double future_tolerance_s{0.1};
  bool inputs_ready{false};
  bool snapshot_ready{false};
  bool goal_pending{false};
  bool cancellation_pending{false};
  bool event_capacity_ready{true};
  bool has_directed_target{false};
  double directed_target_x{0.0};
  double directed_target_y{0.0};
  double directed_target_ttl_s{0.0};
  double max_directed_target_ttl_s{3600.0};
};

struct ExplorationControlResult {
  bool accepted{false};
  bool duplicate{false};
  std::string reason;
  std::string exploration_run_id;
  std::string session_id;
  std::uint64_t intent_revision{0U};
  bool reset_planner{false};
  bool clear_queue{false};
  bool clear_history{false};
  bool request_cancel{false};
  std::string cancel_reason;
  bool set_directed_target{false};
  bool clear_directed_target{false};
};

class ExploreControl final {
 public:
  explicit ExploreControl(std::size_t cache_limit = 128U);

  ExplorationControlResult Apply(const ExplorationControlRequest &request);
  bool Complete();
  void RecordIntentRevision(const std::string &request_id, std::uint64_t revision);
  void RecordIntentOutcome(const std::string &request_id, bool accepted, const std::string &reason,
                           std::uint64_t revision);

  [[nodiscard]] bool active() const noexcept { return active_; }
  [[nodiscard]] bool paused() const noexcept { return paused_; }
  [[nodiscard]] bool running() const noexcept { return active_ && !paused_; }
  [[nodiscard]] const std::string &session_id() const noexcept { return session_id_; }
  [[nodiscard]] const std::string &exploration_run_id() const noexcept {
    return exploration_run_id_;
  }

 private:
  struct AckRecord {
    std::int32_t kind{0};
    std::string request_exploration_run_id;
    std::string request_session_id;
    bool accepted{false};
    std::string reason;
    std::string session_id;
    std::uint64_t intent_revision{0U};
  };

  ExplorationControlResult Finish(const ExplorationControlRequest &request, bool accepted,
                                  std::string reason, ExplorationControlResult actions = {});
  void Remember(const std::string &request_id, const AckRecord &record);

  std::size_t cache_limit_{128U};
  bool active_{false};
  bool paused_{false};
  std::string session_id_;
  std::string exploration_run_id_;
  std::string last_start_request_id_;
  std::string last_start_session_id_;
  std::string last_start_exploration_run_id_;
  std::unordered_set<std::string> used_exploration_run_ids_;
  std::unordered_map<std::string, AckRecord> ack_cache_;
  std::deque<std::string> ack_order_;
};

}  // namespace lingtu::nav::endpoint
