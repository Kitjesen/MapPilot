#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "nav/inspection/inspection.hpp"

namespace lingtu::nav::endpoint {

struct InspectionRuntimeConfig {
  double map_check_interval_s{1.0};
  double status_interval_s{0.5};
};

struct InspectionRuntimeMapIdentity {
  std::string map_id;
  std::int64_t version{0};
};

struct InspectionRuntimeRobotPosition {
  double x_m{0.0};
  double y_m{0.0};
};

// Owning, transport-free representation of an evidence worker result. DDS
// samples must be copied into this view before their loan is returned.
struct InspectionRuntimeEvidenceResult {
  std::string request_id;
  bool persisted{false};
  std::string evidence_id;
  std::string reason;
};

struct InspectionRuntimeTickInput {
  double now_s{0.0};
  std::uint64_t odom_generation{0U};
  std::optional<lingtu::nav::inspection::ArrivalSample> arrival_sample;
  std::optional<InspectionRuntimeRobotPosition> robot_position;
  bool path_active{false};
  bool goal_plan_busy{false};
  std::optional<InspectionRuntimeMapIdentity> active_map;
  bool evidence_worker_matched{false};
  std::vector<InspectionRuntimeEvidenceResult> evidence_results;
};

enum class InspectionRuntimeIntentKind {
  kClearMotion,
  kStopControlAuthority,
};

// These intents are ordered. Callers must apply them in vector order before
// dispatching a goal or evidence request from the same tick.
struct InspectionRuntimeIntent {
  InspectionRuntimeIntentKind kind{InspectionRuntimeIntentKind::kClearMotion};
  std::string reason;
};

struct InspectionGoalDispatchIntent {
  lingtu::nav::inspection::Point point;
};

struct InspectionEvidenceDispatchIntent {
  lingtu::nav::inspection::ActionRequest action;
  std::string map_id;
  std::int64_t map_version{0};
  double deadline_s{0.0};
};

struct InspectionRuntimeTickResult {
  std::vector<InspectionRuntimeIntent> ordered_intents;
  std::optional<InspectionGoalDispatchIntent> goal_dispatch;
  std::optional<InspectionEvidenceDispatchIntent> evidence_dispatch;
};

struct InspectionRuntimeDispatchCompletion {
  bool consumed{false};
  std::optional<std::string> clear_motion_reason;
};

// Transport- and file-I/O-free orchestration for the inspection runtime.
// Command admission and ACK publication intentionally stay outside this class
// so Pause/Cancel acceptance can still be downgraded when the required zero
// command cannot be published.
class InspectionRuntimeController {
 public:
  explicit InspectionRuntimeController(lingtu::nav::inspection::Executor &executor,
                                       InspectionRuntimeConfig config = {});

  InspectionRuntimeController(const InspectionRuntimeController &) = delete;
  InspectionRuntimeController &operator=(const InspectionRuntimeController &) = delete;

  [[nodiscard]] InspectionRuntimeTickResult tick(const InspectionRuntimeTickInput &input);

  // Complete the synchronous effects requested by tick(). A failed goal
  // dispatch is fed back as a leg failure. A failed evidence publication is
  // fed back with the established evidence_request_publish_failed reason.
  [[nodiscard]] InspectionRuntimeDispatchCompletion
  completeGoalDispatch(bool accepted, const std::string &reason, double now_s);
  [[nodiscard]] InspectionRuntimeDispatchCompletion
  completeEvidenceDispatch(const std::string &request_id, bool published, double now_s);

  // External state transitions (commands, goal-plan completion, autonomy
  // outcomes) use these small hooks without moving their ACK/motion effects
  // into the controller.
  void requestStatus() noexcept;
  [[nodiscard]] bool takeStatusDue(double now_s) noexcept;
  void onGoalReached(double now_s);
  void resetArrivalOdomGeneration(std::uint64_t generation) noexcept;
  void clearActivePoint() noexcept;

  [[nodiscard]] const std::optional<lingtu::nav::inspection::Point> &activePoint() const noexcept;
  [[nodiscard]] lingtu::nav::inspection::Executor &executor() noexcept;
  [[nodiscard]] const lingtu::nav::inspection::Executor &executor() const noexcept;

 private:
  [[nodiscard]] bool applyEvidenceResult(const InspectionRuntimeEvidenceResult &result,
                                         double now_s);
  void appendClearMotion(InspectionRuntimeTickResult &result, const std::string &reason) const;

  lingtu::nav::inspection::Executor &executor_;
  InspectionRuntimeConfig config_;
  std::optional<lingtu::nav::inspection::Point> active_point_;
  std::uint64_t arrival_odom_generation_{0U};
  double next_map_check_s_{0.0};
  double next_status_s_{0.0};
  bool status_requested_{false};
  bool goal_dispatch_outstanding_{false};
  std::string evidence_dispatch_outstanding_request_id_;
};

}  // namespace lingtu::nav::endpoint
