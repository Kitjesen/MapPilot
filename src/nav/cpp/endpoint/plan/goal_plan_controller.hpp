#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "message/cpp/navigation_command.hpp"
#include "plan/global_plan_task.hpp"

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
  bool planner_map_configured{false};
  std::string planner_map_missing_reason{"active_octomap_not_configured"};
  std::uint64_t frame_epoch{0U};
  bool rolling_segment_active{false};
  lingtu::nav::plan::GlobalPlannerOptions planner_options{};
};

struct GoalPlanAdvanceContext {
  std::uint64_t frame_epoch{0U};
  bool operator_takeover_latched{false};
  double now_s{0.0};
};

struct GoalPlanStatus {
  std::string request_id;
  std::uint64_t goal_epoch{0U};
  lingtu::message::NavigationGoalState state{lingtu::message::NavigationGoalState::Failed};
  std::string reason;
};

struct GoalPlanMapIdentityResult {
  std::optional<lingtu::nav::plan::MapIdentity> identity;
  std::string reason;
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

struct GoalPlanTerminalAfterStop {
  std::string reason;
  GoalPlanTerminalCommit commit;
};

struct GoalPlanSubmitResult {
  bool accepted{false};
  std::string reason;
  bool counted_failure{false};
  bool count_frame_rejection{false};
  bool record_frame_error{false};
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
  std::string planning_request_id;
  std::uint64_t planning_goal_epoch{0U};
  std::string active_request_id;
  std::uint64_t active_goal_epoch{0U};
  bool busy{false};
  GoalPlanDiagnostics diagnostics;
};

class GoalPlanController {
 public:
  GoalPlanController(GlobalPlanTask::Planner planner, GoalPlanActions actions);

  [[nodiscard]] GoalPlanSubmitResult submit(const GoalPlanRequest &request,
                                            const GoalPlanAdmissionContext &context);
  [[nodiscard]] GoalPlanAdvanceResult advance(const GoalPlanAdvanceContext &context);
  [[nodiscard]] GoalPlanTerminalCommit deferAbort(const std::string &reason);
  [[nodiscard]] GoalPlanTerminalCommit deferFailure(const std::string &reason);
  [[nodiscard]] GoalPlanTerminalCommit
  deferActiveTerminal(lingtu::message::NavigationGoalState state, const std::string &reason);
  void invalidateForHold(const std::string &reason);
  [[nodiscard]] GoalPlanSnapshot snapshot() const;

 private:
  [[nodiscard]] GoalPlanSubmitResult reject(std::string reason, bool count_frame_rejection = false,
                                            bool record_frame_error = false);
  void publishStatus(const std::string &request_id, std::uint64_t goal_epoch,
                     lingtu::message::NavigationGoalState state, const std::string &reason);
  [[nodiscard]] GoalPlanTerminalCommit
  terminalCommit(std::vector<GoalPlanStatus> pending_statuses) const;
  [[nodiscard]] GoalPlanTerminalCommit deferPlanningAbort(const std::string &reason);
  void finishPlanning(lingtu::message::NavigationGoalState state, const std::string &reason);
  void finishActive(lingtu::message::NavigationGoalState state, const std::string &reason);

  GlobalPlanTask task_;
  GoalPlanActions actions_;
  std::uint64_t goal_epoch_{0U};
  std::string planning_request_id_;
  std::uint64_t planning_goal_epoch_{0U};
  std::string active_request_id_;
  std::uint64_t active_goal_epoch_{0U};
  GoalPlanDiagnostics diagnostics_;
};

}  // namespace lingtu::nav::endpoint
