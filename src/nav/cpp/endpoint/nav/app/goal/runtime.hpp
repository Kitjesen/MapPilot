#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "control/autonomy.hpp"
#include "app/goal/retry.hpp"
#include "app/goal/plan.hpp"
#include "safety/stop.hpp"

namespace lingtu::nav::endpoint {

class MotionStopBarrier;
class GoalTerminalStatusDelivery;
struct GoalTaskCancelRequest;
struct CommandAck;

struct GoalTerminalActions {
  std::function<void(const std::string &)> report_error;
  std::function<void(const std::string &)> pause_inspection;
  std::function<void()> sync_goal_diagnostics;
};

struct GoalTerminalResult {
  bool action_committed{false};
  bool delivery_acknowledged{false};
  std::string reason;
};

enum class GoalReplanRuntimeInterruption {
  kNewGoal,
  kCancel,
  kStop,
  kEstop,
  kShutdown,
  kOperatorTakeover,
  kControlHold,
  kInspectionPause,
  kInspectionCancel,
  kMapDrift,
};

enum class TerminalStopPolicy {
  kGenericStop,
  kReached,
  // Active and planning-only Stop terminal intents replay through Stop semantics.
  kStop,
  // Active cancel terminal intents replay with TerminalStopPolicy::kCancel.
  kCancel,
  // Estop terminal intents replay through the latched-estop safety barrier.
  kEstop,
  // Shutdown terminal intents replay through the final shutdown barrier.
  kShutdown,
};

struct GoalReplanRuntimeAutonomyEvent {
  AutonomyTickOutcome outcome;
  // Snapshot captured immediately before the autonomy tick which produced outcome.
  GoalPlanSnapshot goal_snapshot;
  bool inspection_active{false};
  bool rolling_segment_active{false};
};

struct GoalReplanRuntimeFrameInput {
  double steady_now_s{0.0};
  double wall_now_s{0.0};
  GoalPlanAdmissionContext fresh_admission;
  bool inspection_active{false};
  bool rolling_segment_active{false};
  bool map_drift{false};
};

struct GoalReplanRuntimeResult {
  bool handled{false};
  bool zero_kept_fresh{false};
  bool replan_started{false};
  bool pending_resumed{false};
  bool interrupted{false};
  std::string reason{"idle"};
  std::uint64_t terminal_intent_id{0U};
  std::string terminal_task_id;
  TerminalStopPolicy terminal_stop_policy{TerminalStopPolicy::kGenericStop};
  GoalPlanAdvanceResult plan_advance;
  std::optional<GoalPlanTerminalAfterStop> terminal_after_stop;
};

struct GoalTerminalSchedulingDecision {
  bool service_terminal{false};
  bool run_autonomy_tick{true};
};

struct ShutdownExitDecision {
  bool allow_exit{false};
};

struct NavigationRuntimeAutonomyObservation {
  GoalReplanRuntimeFrameInput updated_frame;
  AutonomyTickOutcome outcome;
  bool inspection_active{false};
  bool rolling_segment_active{false};
};

struct NavigationRuntimeFrameActions {
  std::function<void(const GoalReplanRuntimeResult &)> complete_endpoint_work_before_autonomy;
  std::function<NavigationRuntimeAutonomyObservation(const GoalPlanSnapshot &)> run_autonomy;
  std::function<void(const GoalReplanRuntimeResult &)> apply_autonomy_outputs;
};

struct NavigationRuntimeFrameResult {
  GoalReplanRuntimeResult planning_result;
  std::optional<GoalReplanRuntimeResult> autonomy_result;
  GoalReplanRuntimeResult pending_result;
  bool pending_cycle_advanced{false};
  bool terminal_delivery_acknowledged{false};
  std::optional<AutonomyTickOutcome> inspection_completion;
};

struct NavigationRuntimeInterruptionResult {
  GoalReplanRuntimeResult runtime_result;
  std::optional<GoalTerminalResult> terminal;
};

struct NavigationRuntimeGoalSubmissionResult {
  GoalPlanSubmitResult plan_result;
  std::optional<GoalTerminalResult> preemption_terminal;
};

struct GoalResumeActions {
  std::function<void()> release_operator;
  std::function<void()> activate_route;
  std::function<void()> changed;
};

[[nodiscard]] GoalTerminalSchedulingDecision
decideGoalTerminalScheduling(const GoalReplanRuntimeResult &result, bool terminal_pending);

[[nodiscard]] ShutdownExitDecision decideShutdownExit(bool stop_confirmed, bool terminal_required,
                                                      bool terminal_pending,
                                                      bool delivery_acknowledged);

// Transport-free single-thread coordinator. GoalPlanController remains the sole
// owner of active/pending requests and GlobalPlanTask; this class owns only the
// stopped retry timing and the replacement-plan phase. Live local
// obstacle evidence stays in SCAN/recovery and is never converted here into a
// global planner overlay.
class GoalReplanRuntimeCoordinator {
 public:
  GoalReplanRuntimeCoordinator(GoalPlanController &goal_plan, MotionStopBarrier &motion_stop,
                               GoalTerminalStatusDelivery &delivery,
                               GoalTerminalActions terminal_actions,
                               BoundedGoalReplanConfig config = {});

  // Production order: command ingress -> advancePlanningCycle -> service any
  // surfaced terminal -> capture the pre-autonomy snapshot/run autonomy only
  // when scheduling permits -> handleAutonomyOutcome -> service any resulting
  // terminal -> drainPendingCycle only when no terminal remains pending.
  [[nodiscard]] GoalReplanRuntimeResult
  advancePlanningCycle(const GoalReplanRuntimeFrameInput &frame);
  [[nodiscard]] GoalReplanRuntimeResult
  handleAutonomyOutcome(const GoalReplanRuntimeFrameInput &frame,
                        const GoalReplanRuntimeAutonomyEvent &event);
  [[nodiscard]] GoalReplanRuntimeResult drainPendingCycle(const GoalReplanRuntimeFrameInput &frame);
  [[nodiscard]] GoalTerminalResult serviceTerminal(
      const GoalReplanRuntimeResult &result, const std::string &estop_reason = "estop_latched");
  [[nodiscard]] NavigationRuntimeFrameResult
  advanceFrame(const GoalReplanRuntimeFrameInput &frame,
               const NavigationRuntimeFrameActions &actions);
  [[nodiscard]] NavigationRuntimeInterruptionResult
  interruptAndService(GoalReplanRuntimeInterruption interruption, double steady_now_s,
                      const std::string &estop_reason = "estop_latched");
  [[nodiscard]] NavigationRuntimeGoalSubmissionResult
  submitGoal(const GoalPlanRequest &request, const GoalPlanAdmissionContext &admission,
             double steady_now_s);
  [[nodiscard]] GoalTerminalResult completeTerminal(
      const GoalReplanRuntimeResult &result, const std::string &estop_reason = "estop_latched");
  [[nodiscard]] MotionStopTerminalBarrierResult stopWhileTerminalPending();
  [[nodiscard]] MotionStopTerminalBarrierResult
  estopWhileTerminalPending(const std::string &estop_reason);
  [[nodiscard]] CommandAck cancelTask(const GoalTaskCancelRequest &request);
  [[nodiscard]] std::pair<bool, std::string> pauseTask(
      const std::string &task_id, const std::string &request_id, const std::string &reason);
  [[nodiscard]] std::pair<bool, std::string> resumeTask(
      const std::string &task_id, const std::string &request_id, double source_stamp_s,
      GoalPlanAdmissionContext context, GoalResumeActions actions,
      const std::string &blocker = {});
  void retryResume(const std::function<GoalPlanAdmissionContext()> &context,
                   const GoalResumeActions &actions,
                   const std::function<std::string()> &blocker);
  void cancelResume();
  [[nodiscard]] MotionStopResult interruptRelated(
      GoalReplanRuntimeInterruption interruption, double now_s, const std::string &reason,
      std::function<bool()> commit);
  [[nodiscard]] MotionStopResult commitRelated(const GoalReplanRuntimeResult &result,
                                               const std::string &reason,
                                               std::function<bool()> commit);
  [[nodiscard]] MotionStopTerminalBarrierResult stopPendingTerminal();
  [[nodiscard]] MotionStopTerminalBarrierResult
  estopPendingTerminal(const std::string &estop_reason);
  [[nodiscard]] GoalReplanRuntimeResult interrupt(GoalReplanRuntimeInterruption interruption,
                                                  double steady_now_s);
  [[nodiscard]] GoalReplanRuntimeResult replayPendingTerminal() const;
  // A safety stop must retain full stop evidence when the terminal is replayed.
  void requirePendingTerminalOdometry();
  [[nodiscard]] bool acknowledgeTerminal(std::uint64_t terminal_intent_id);
  // Command ingress must not admit a new goal while this is true. The endpoint
  // must finish the stop-confirmed terminal transaction and acknowledge its
  // exact intent ID first.
  [[nodiscard]] bool terminalPending() const;
  [[nodiscard]] bool replanPending() const;
  [[nodiscard]] BoundedGoalReplanSnapshot snapshot() const;

 private:
  [[nodiscard]] GoalReplanRuntimeResult pause(const std::string &reason);
  [[nodiscard]] static bool attemptActive(BoundedGoalReplanState state);
  [[nodiscard]] static bool validTime(double steady_now_s);
  [[nodiscard]] static bool validPosition(const nav_kernel::Vec3 &position);
  [[nodiscard]] static bool sameActiveGoal(const GoalPlanSnapshot &lhs,
                                           const GoalPlanSnapshot &rhs);
  [[nodiscard]] static bool directReplacementPlanning(const GoalPlanSnapshot &snapshot);
  [[nodiscard]] static std::optional<BoundedGoalReplanGoal>
  activeGoal(const GoalPlanSnapshot &snapshot);
  [[nodiscard]] std::optional<BoundedGoalReplanGoal> trackedGoal() const;
  [[nodiscard]] static std::string replacementStateFailure(const GoalReplanRuntimeFrameInput &frame,
                                                   const GoalPlanSnapshot &snapshot);
  [[nodiscard]] std::optional<GoalReplanRuntimeResult>
  handleGoalReached(const GoalReplanRuntimeFrameInput &frame,
                    const GoalReplanRuntimeAutonomyEvent &event,
                    const GoalPlanSnapshot &current_snapshot);
  [[nodiscard]] std::optional<GoalReplanRuntimeResult>
  handleTerminalFailureOutcome(const GoalReplanRuntimeFrameInput &frame,
                                  const GoalReplanRuntimeAutonomyEvent &event,
                                  const GoalPlanSnapshot &current_snapshot);

  void cancelInvalidTimeFor(const BoundedGoalReplanGoal &goal, double steady_now_s);
  void observeAndCancel(const BoundedGoalReplanGoal &goal, double steady_now_s,
                        const std::string &reason);
  void attachDeferredTerminal(GoalReplanRuntimeResult &result,
                              lingtu::message::NavigationGoalState state, const std::string &reason,
                              bool invalidate_planning,
                              TerminalStopPolicy stop_policy = TerminalStopPolicy::kGenericStop);
  void attachExistingTerminal(GoalReplanRuntimeResult &result, GoalPlanTerminalAfterStop terminal,
                              const std::string &task_id,
                              TerminalStopPolicy stop_policy = TerminalStopPolicy::kGenericStop);
  [[nodiscard]] bool surfacePendingTerminal(GoalReplanRuntimeResult &result) const;
  [[nodiscard]] std::uint64_t allocateTerminalIntentId();
  enum class StopAction { kStop, kEstop };
  [[nodiscard]] MotionStopTerminalBarrierResult
  stopPendingTerminal(StopAction action, const std::string &estop_reason);

  GoalPlanController &goal_plan_;
  MotionStopBarrier &motion_stop_;
  GoalTerminalStatusDelivery &delivery_;
  GoalTerminalActions terminal_actions_;
  BoundedGoalReplanController bounded_;
  std::optional<GoalReplanTrigger> pending_replan_trigger_;
  bool replacement_plan_in_progress_{false};
  std::uint64_t next_terminal_intent_id_{1U};
  std::uint64_t pending_terminal_intent_id_{0U};
  std::string pending_terminal_task_id_;
  TerminalStopPolicy pending_terminal_stop_policy_{TerminalStopPolicy::kGenericStop};
  std::optional<GoalPlanTerminalAfterStop> pending_terminal_;
  struct PendingResume {
    std::string task_id;
    std::string request_id;
    double source_stamp_s;
  };
  std::optional<PendingResume> pending_resume_;
};

}  // namespace lingtu::nav::endpoint
