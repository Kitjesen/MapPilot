#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "message/cpp/navigation_command.hpp"
#include "runtime/goal/task.hpp"
#include "input/active/identity.hpp"

namespace lingtu::nav::endpoint {

enum class GoalPlanOrigin {
  kExternal,
  kInspection,
};

struct GoalPlanTarget {
  nav_kernel::Vec3 position{};
  std::optional<double> yaw;
};

struct GoalPlanRequest {
  std::string task_id;
  std::string request_id;
  GoalPlanOrigin origin{GoalPlanOrigin::kExternal};
  double source_stamp_s{0.0};
  std::optional<GoalPlanTarget> target;
  std::string decode_error;
};

struct GoalPlanAdmissionContext {
  bool motion_allowed{false};
  bool operator_takeover_latched{false};
  bool autonomy_mode{false};
  std::string control_mode_name{"autonomy"};
  std::string driver_control_blocker;
  double autonomy_request_not_before_s{0.0};
  std::optional<nav_kernel::Vec3> map_position;
  bool odometry_ready{false};
  bool input_ready{true};
  std::string input_gate_reason;
  bool retained_path_ready{true};
  std::string retained_path_reason{"retained_global_path_missing"};
  bool planner_map_configured{false};
  std::string planner_map_missing_reason{"active_octomap_not_configured"};
  std::uint64_t frame_epoch{0U};
  bool rolling_segment_active{false};
  lingtu::nav::plan::GlobalPlannerOptions planner_options{};
  lingtu::nav::plan::GlobalPlanTemporaryOverlay temporary_overlay{};
};

struct GoalPlanAdvanceContext {
  std::uint64_t frame_epoch{0U};
  bool operator_takeover_latched{false};
  double now_s{0.0};
};

struct GoalPlanStatus {
  std::string task_id;
  std::string request_id;
  std::uint64_t goal_epoch{0U};
  lingtu::message::NavigationGoalState state{lingtu::message::NavigationGoalState::Failed};
  std::string reason;
  bool project_to_navigation_state{true};
};

struct GoalPlanTerminalDeliveryTicket {
  std::vector<GoalPlanStatus> statuses;
};

struct GoalPlanPathTolerance {
  double position_m{0.0};
  double yaw_rad{0.0};
};

struct GoalPlanInspectionDecision {
  bool accepted{true};
  std::string reason;
  std::optional<GoalPlanPathTolerance> tolerance;
};

struct GoalPlanPathActivation {
  std::vector<nav_kernel::Vec3> path;
  std::optional<double> goal_yaw;
  std::optional<GoalPlanPathTolerance> tolerance;
  std::optional<lingtu::nav::plan::MapIdentity> map_identity;
  double stamp_s{0.0};
};

struct GoalPlanDiagnostics {
  bool seen{false};
  bool accepted{false};
  bool reached_goal{false};
  std::string reason{"no_goal_received"};
  std::size_t waypoints{0U};
  double goal_error_m{-1.0};
  double elapsed_ms{0.0};
  nav_kernel::Vec3 start{};
  nav_kernel::Vec3 goal{};
};

struct GoalPlanActions {
  std::function<bool(const std::string &)> preempt_rolling;
  std::function<void()> clear_external_inspection;
  std::function<GoalPlanMapIdentityResult()> current_map_identity;
  std::function<void(const GoalPlanStatus &)> publish_status;
  std::function<bool()> inspection_active;
  std::function<void(const std::string &, double)> inspection_leg_failed;
  std::function<void(const std::string &)> inspection_pause;
  std::function<GoalPlanInspectionDecision(double)> inspection_plan_ready;
  std::function<void(const GoalPlanPathActivation &)> activate_path;
};

using GoalPlanTerminalCommit = std::function<void()>;

struct GoalPlanTerminalCommitWithTicket {
  GoalPlanTerminalDeliveryTicket ticket;
  GoalPlanTerminalCommit commit;
};

struct GoalPlanTaskTransition {
  bool accepted{false};
  std::string reason;
  GoalPlanTerminalCommit commit;
};

struct GoalPlanTerminalAfterStop {
  std::string reason;
  GoalPlanTerminalDeliveryTicket delivery_ticket;
  GoalPlanTerminalCommit commit;
};

struct GoalPlanActiveTerminalRequest {
  lingtu::message::NavigationGoalState state{lingtu::message::NavigationGoalState::Cancelled};
  std::string reason;
};

struct GoalPlanSubmitResult {
  bool accepted{false};
  std::string reason;
  bool counted_failure{false};
  bool count_frame_rejection{false};
  bool record_frame_error{false};
  std::optional<GoalPlanActiveTerminalRequest> active_terminal_after_stop;
};

struct GoalPlanAdvanceResult {
  bool completion_consumed{false};
  bool counted_failure{false};
  bool record_frame_error{false};
  bool path_activated{false};
  bool inspection_status_changed{false};
  std::optional<GoalPlanTerminalAfterStop> terminal_after_stop;
  double elapsed_ms{0.0};
  std::string map_identity_error;
};

struct GoalPlanSnapshot {
  std::uint64_t goal_epoch{0U};
  std::string planning_task_id;
  std::string planning_request_id;
  std::uint64_t planning_goal_epoch{0U};
  bool replan_in_progress{false};
  bool replacement_plan_in_progress{false};
  std::uint64_t replan_attempt{0U};
  bool pending_plan_queued{false};
  std::string pending_task_id;
  std::string pending_request_id;
  std::uint64_t pending_goal_epoch{0U};
  std::string deferred_replacement_task_id;
  std::string deferred_replacement_request_id;
  std::uint64_t deferred_replacement_goal_epoch{0U};
  std::string active_task_id;
  std::string active_request_id;
  std::uint64_t active_goal_epoch{0U};
  std::optional<GoalPlanOrigin> active_origin;
  bool active_paused{false};
  std::optional<lingtu::nav::plan::MapIdentity> active_map_identity;
  bool busy{false};
  GoalPlanDiagnostics diagnostics;
};

[[nodiscard]] inline bool goalPlanAcceptsReplanTrigger(const GoalPlanSnapshot &snapshot) noexcept {
  return !snapshot.active_paused && !snapshot.busy && !snapshot.pending_plan_queued;
}

class GoalPlanController {
 public:
  GoalPlanController(GlobalPlanTask::Planner planner, GoalPlanActions actions);

  [[nodiscard]] GoalPlanSubmitResult submit(const GoalPlanRequest &request,
                                            const GoalPlanAdmissionContext &context);
  [[nodiscard]] GoalPlanSubmitResult replanActive(const GoalPlanAdmissionContext &context);
  [[nodiscard]] GoalPlanSubmitResult resumePending(const GoalPlanAdmissionContext &fresh_context);
  [[nodiscard]] GoalPlanAdvanceResult advance(const GoalPlanAdvanceContext &context);
  [[nodiscard]] GoalPlanAdvanceResult
  activateDeferredReplacement(double now_s, const GoalPlanAdmissionContext &fresh_context);
  [[nodiscard]] GoalPlanAdvanceResult
  failDeferredReplacement(lingtu::message::NavigationGoalState state, const std::string &reason);
  [[nodiscard]] GoalPlanTaskTransition
  deferPause(const std::string &task_id, const std::string &request_id, const std::string &reason);
  [[nodiscard]] GoalPlanTaskTransition deferResume(const std::string &task_id,
                                                   const std::string &request_id,
                                                   const GoalPlanAdmissionContext &context);
  [[nodiscard]] GoalPlanTaskTransition deferCancelPending(const std::string &task_id,
                                                          const std::string &request_id,
                                                          const std::string &reason);
  [[nodiscard]] GoalPlanTaskTransition deferCancelReplacementPlanning(const std::string &task_id,
                                                                      const std::string &request_id,
                                                                      const std::string &reason);
  [[nodiscard]] GoalPlanTerminalCommit deferAbort(const std::string &reason,
                                                  bool project_planning_to_navigation_state = true);
  [[nodiscard]] GoalPlanTerminalCommit deferFailure(const std::string &reason);
  [[nodiscard]] GoalPlanTerminalCommit
  deferActiveTerminal(lingtu::message::NavigationGoalState state, const std::string &reason);
  [[nodiscard]] GoalPlanTerminalCommitWithTicket
  deferAbortWithTicket(const std::string &reason, bool project_planning_to_navigation_state = true);
  [[nodiscard]] GoalPlanTerminalCommitWithTicket deferFailureWithTicket(const std::string &reason);
  [[nodiscard]] GoalPlanTerminalCommitWithTicket
  deferActiveTerminalWithTicket(lingtu::message::NavigationGoalState state,
                                const std::string &reason);
  void invalidateForHold(const std::string &reason);
  [[nodiscard]] GoalPlanSnapshot snapshot() const;

 private:
  [[nodiscard]] static GoalPlanTerminalAfterStop
  terminalAfterStop(std::string reason, GoalPlanTerminalCommitWithTicket terminal);
  [[nodiscard]] GoalPlanSubmitResult reject(std::string reason, bool count_frame_rejection = false,
                                            bool record_frame_error = false);
  void publishStatus(const std::string &task_id, const std::string &request_id,
                     std::uint64_t goal_epoch, lingtu::message::NavigationGoalState state,
                     const std::string &reason, bool project_to_navigation_state = true);
  [[nodiscard]] GoalPlanTerminalCommit
  terminalCommit(std::vector<GoalPlanStatus> pending_statuses) const;
  [[nodiscard]] GoalPlanTerminalCommitWithTicket
  deferPlanningAbortWithTicket(const std::string &reason,
                               bool project_planning_to_navigation_state = true);
  [[nodiscard]] GoalPlanSubmitResult
  startPlanning(const std::string &task_id, const std::string &request_id,
                const GoalPlanTarget &target, GoalPlanOrigin origin,
                const GoalPlanAdmissionContext &context, const std::string &planning_reason,
                bool is_replan, std::optional<std::uint64_t> reserved_goal_epoch = std::nullopt,
                bool project_planning_to_navigation_state = true);
  void invalidateForHold(const std::string &reason, bool close_pending_now);
  void finishPlanning(lingtu::message::NavigationGoalState state, const std::string &reason);
  void finishActive(lingtu::message::NavigationGoalState state, const std::string &reason);
  void publishPendingTerminal(lingtu::message::NavigationGoalState state,
                              const std::string &reason);
  [[nodiscard]] GoalPlanAdvanceResult
  failDeferredReplacementLocked(lingtu::message::NavigationGoalState state,
                                const std::string &reason);

  struct DeferredPlanStart {
    GoalPlanRequest request;
    std::uint64_t goal_epoch{0U};
  };
  struct DeferredReplacementActivation {
    std::string task_id;
    std::string request_id;
    std::uint64_t goal_epoch{0U};
    GoalPlanPathActivation activation;
    GoalPlanTarget target;
    GoalPlanOrigin origin{GoalPlanOrigin::kExternal};
    lingtu::nav::plan::GlobalPlannerOptions planner_options{};
  };
  void clearPlanningIdentity();

  GlobalPlanTask task_;
  GoalPlanActions actions_;
  std::uint64_t goal_epoch_{0U};
  std::string planning_task_id_;
  std::string planning_request_id_;
  std::uint64_t planning_goal_epoch_{0U};
  bool planning_is_replan_{false};
  std::uint64_t planning_replan_attempt_{0U};
  bool planning_projects_to_navigation_state_{true};
  GoalPlanOrigin planning_origin_{GoalPlanOrigin::kExternal};
  std::string active_task_id_;
  std::string active_request_id_;
  std::uint64_t active_goal_epoch_{0U};
  bool active_paused_{false};
  std::optional<lingtu::nav::plan::MapIdentity> active_map_identity_;
  std::optional<GoalPlanTarget> active_target_;
  GoalPlanOrigin active_origin_{GoalPlanOrigin::kExternal};
  lingtu::nav::plan::GlobalPlannerOptions active_planner_options_{};
  std::uint64_t replan_attempt_{0U};
  std::optional<DeferredPlanStart> pending_plan_start_;
  std::optional<DeferredReplacementActivation> deferred_replacement_activation_;
  GoalPlanDiagnostics diagnostics_;
};

}  // namespace lingtu::nav::endpoint
