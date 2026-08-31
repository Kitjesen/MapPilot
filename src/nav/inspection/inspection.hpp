#pragma once

#include "message/cpp/inspection_command.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace lingtu::nav::inspection {

using CommandKind = lingtu::message::InspectionCommandKind;

enum class FailurePolicy : std::int32_t {
  kStop = 0,
  kRetry = 1,
  kSkip = 2,
};

enum class RunState : std::int32_t {
  kIdle = 0,
  kValidating = 1,
  kPlanning = 2,
  kNavigating = 3,
  kDwelling = 4,
  kPaused = 5,
  kRecovering = 6,
  kSucceeded = 7,
  kFailed = 8,
  kCancelled = 9,
  kSettling = 10,
  kActionPending = 11,
  // Route progression has stopped, but the final task state still awaits
  // native zero-output confirmation.
  kPausing = 12,
  kCancelling = 13,
};

struct Point {
  std::string id;
  std::string frame_id{"map"};
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
  bool has_yaw{false};
  double position_tolerance_m{0.35};
  double yaw_tolerance_rad{0.35};
  double dwell_s{0.0};
  std::string action;
  bool enabled{true};
};

struct Route {
  std::string id;
  std::string name;
  std::string map_id;
  std::int64_t map_content_epoch{0};
  std::uint64_t revision{1};
  std::uint32_t loop_count{1};
  FailurePolicy failure_policy{FailurePolicy::kStop};
  std::uint32_t max_retries{0};
  std::vector<Point> points;
};

struct ValidationResult {
  bool ok{false};
  std::string reason;
  std::size_t enabled_points{0};
};

struct ArrivalSample {
  double sample_time_s{0.0};
  double linear_speed_mps{0.0};
  double angular_speed_radps{0.0};
};

struct ActionRequest {
  std::string request_id;
  // task_id is the product identity; run_id remains for legacy evidence
  // workers that have not yet moved to the task vocabulary.
  std::string task_id;
  std::string run_id;
  std::string route_id;
  std::uint64_t route_revision{0};
  std::size_t point_index{0};
  std::string point_id;
  std::string action;
};

struct RunStatus {
  RunState state{RunState::kIdle};
  // task_id is immutable for one operator-visible inspection task. run_id is
  // an intentional compatibility mirror until the v1 DDS/status surfaces are
  // retired; new code must correlate through task_id.
  std::string task_id;
  std::string run_id;
  // The latest accepted task-control request. It is deliberately distinct
  // from task_id because retries and pause/resume/cancel each get a new ID.
  std::string request_id;
  std::string map_id;
  std::int64_t map_content_epoch{0};
  std::string route_id;
  std::uint64_t route_revision{0};
  std::size_t point_index{0};
  std::size_t point_count{0};
  std::uint32_t loop_index{0};
  std::uint32_t retry_count{0};
  std::string point_id;
  std::string action;
  std::string action_request_id;
  std::string evidence_id;
  double phase_started_at_s{0.0};
  double stable_since_s{0.0};
  double deadline_s{0.0};
  std::string reason;
};

// A task event is a fact emitted by the native task authority. The endpoint
// later adds its boot identity and transport sequence when it publishes this
// outbox to DDS; Gateway must consume these facts rather than infer them from
// command ACKs or snapshots.
enum class TaskEventKind : std::int32_t {
  kTaskAccepted = 1,
  kStateChanged = 2,
  kMilestone = 3,
  kStopConfirmationFailed = 4,
  kEvidenceRecorded = 5,
};

struct TaskEvent {
  std::uint64_t sequence{0U};
  double timestamp_s{0.0};
  TaskEventKind kind{TaskEventKind::kTaskAccepted};
  // The command or action that caused this event. The snapshot separately
  // preserves the latest accepted operator command in status.request_id.
  std::string request_id;
  RunStatus status;
};

struct TaskEventValidationResult {
  bool ok{false};
  std::string reason;
};

struct RestartTaskEventResult {
  bool ok{false};
  bool replayed_terminal{false};
  bool synthesized_failure{false};
  TaskEvent event;
  std::string reason;
};

struct TaskEventEnvelope {
  std::string boot_id;
  std::uint64_t sequence{0U};
  TaskEvent event;
};

enum class TaskEventOutboxRecordResult : std::uint8_t {
  kAccepted,
  kInvalid,
  kOutOfOrder,
  kBackpressure,
};

struct TaskEventOutboxDiagnostics {
  std::uint64_t accepted{0U};
  std::uint64_t rejected_invalid{0U};
  std::uint64_t rejected_out_of_order{0U};
  std::uint64_t rejected_backpressure{0U};
  std::uint64_t delivered{0U};
  std::uint64_t delivery_failures{0U};
  std::size_t pending{0U};
};

ValidationResult ValidateRoute(const Route& route);
bool IsKnownCommandKind(std::int32_t value) noexcept;
const char* CommandKindName(CommandKind kind) noexcept;
const char* FailurePolicyName(FailurePolicy policy) noexcept;
const char* RunStateName(RunState state) noexcept;
const char* TaskEventKindName(TaskEventKind kind) noexcept;
const char* TaskEventOutboxRecordResultName(TaskEventOutboxRecordResult result) noexcept;
bool IsActiveRunState(RunState state) noexcept;
bool IsTerminalRunState(RunState state) noexcept;
TaskEventValidationResult ValidateTaskEvent(const TaskEvent& event);
RestartTaskEventResult ReconcileTaskEventAfterRestart(
    const TaskEvent& checkpoint,
    double timestamp_s);

// Ordered, retrying delivery for one endpoint boot. Recovery explicitly sets
// the first sequence accepted by the new boot; ordinary record calls remain
// strictly contiguous after that initialization.
class InspectionTaskEventOutbox final {
 public:
  using WriteCallback = std::function<bool(const TaskEventEnvelope&)>;

  static constexpr std::size_t kDefaultCapacity = 1024U;
  static constexpr std::size_t kDefaultFlushLimit = 32U;

  InspectionTaskEventOutbox(
      std::string boot_id,
      WriteCallback write,
      std::size_t capacity = kDefaultCapacity);

  InspectionTaskEventOutbox(const InspectionTaskEventOutbox&) = delete;
  InspectionTaskEventOutbox& operator=(const InspectionTaskEventOutbox&) = delete;

  bool InitializeNextSequence(std::uint64_t next_sequence) noexcept;
  TaskEventOutboxRecordResult Record(const TaskEvent& event);
  std::size_t Flush(std::size_t max_events = kDefaultFlushLimit);
  TaskEventOutboxDiagnostics diagnostics() const noexcept;
  std::uint64_t next_sequence() const noexcept { return next_sequence_; }

 private:
  std::string boot_id_;
  WriteCallback write_;
  std::size_t capacity_;
  std::uint64_t next_sequence_{1U};
  bool initialized_{false};
  bool recording_started_{false};
  bool sequence_exhausted_{false};
  std::deque<TaskEventEnvelope> pending_;
  TaskEventOutboxDiagnostics diagnostics_;
};

class Executor {
 public:
  static constexpr double kArrivalSampleMaxAgeS = 0.25;
  static constexpr double kArrivalFutureToleranceS = 0.05;
  static constexpr double kArrivalMaxSampleGapS = 0.30;
  static constexpr double kStableLinearSpeedMps = 0.05;
  static constexpr double kStableAngularSpeedRadps = 0.10;
  static constexpr double kStableDurationS = 0.50;
  static constexpr std::size_t kStableSampleCount = 3U;
  static constexpr double kPlanningTimeoutS = 30.0;
  static constexpr double kNavigationTimeoutS = 45.0;
  static constexpr double kNavigationMinProgressM = 0.10;
  static constexpr double kSettlingTimeoutS = 10.0;
  static constexpr double kActionTimeoutS = 30.0;

  bool Start(
      Route route,
      std::string task_id,
      std::string request_id,
      const std::string& active_map_id,
      std::int64_t active_map_content_epoch,
      double now_s,
      std::string* error);
  // Legacy in-process callers have no request identity. Keep this overload
  // while all field traffic migrates through the task-aware command path.
  bool Start(
      Route route,
      std::string run_id,
      const std::string& active_map_id,
      std::int64_t active_map_content_epoch,
      double now_s,
      std::string* error);
  // A native motion authority must call the matching Commit* only after it
  // has proven the final zero output. Direct Pause/Cancel remain for legacy
  // in-process safety callers.
  bool RequestPause(
      const std::string& reason,
      const std::string& request_id = {},
      double now_s = 0.0);
  bool CommitPause(double now_s = 0.0);
  bool RequestCancel(
      const std::string& reason,
      const std::string& request_id = {},
      double now_s = 0.0);
  bool CommitCancel(double now_s = 0.0);
  void MarkStopConfirmationFailed(std::string reason, double now_s = 0.0);
  bool Pause(const std::string& reason);
  bool Resume(
      const std::string& active_map_id,
      std::int64_t active_map_content_epoch,
      double now_s,
      const std::string& request_id = {});
  bool Cancel(const std::string& reason);

  std::optional<Point> PendingGoal() const;
  bool OnPlanningStarted(double now_s);
  bool OnPlanReady(double now_s);
  bool OnNavigationProgress(double remaining_distance_m, double now_s);
  void OnLegFailed(const std::string& reason, double now_s);
  bool OnNavigationFailed(const std::string& reason, double now_s);
  void OnGoalReached(double now_s);
  bool OnArrivalSample(const ArrivalSample& sample, double now_s);
  std::optional<ActionRequest> PendingAction() const;
  bool OnActionStarted(const std::string& request_id, double now_s);
  bool OnActionResult(
      const std::string& request_id,
      bool success,
      const std::string& evidence_id,
      const std::string& reason,
      double now_s);
  void Tick(double now_s);
  void OnMapChanged(const std::string& active_map_id, std::int64_t active_map_content_epoch);

  bool active() const noexcept;
  const Route* route() const noexcept;
  const RunStatus& status() const noexcept { return status_; }
  // Hands task facts to a downstream outbox without losing the rejected head.
  // The endpoint uses this to keep core events until its ordered transport
  // outbox has accepted them.
  std::size_t FlushTaskEvents(const std::function<bool(const TaskEvent&)>& accept,
                              std::size_t max_events = 64U);
  std::vector<TaskEvent> TakeTaskEvents();
  // Recovery may advance the process-wide task-event cursor, but only before
  // this fresh Executor has acquired a route or queued any event.
  bool SetNextTaskEventSequence(std::uint64_t next_sequence) noexcept;
  std::uint64_t next_task_event_sequence() const noexcept {
    return next_task_event_sequence_;
  }
  std::size_t pending_task_event_count() const noexcept {
    return task_events_.size();
  }

 private:
  bool MapMatches(const std::string& map_id, std::int64_t map_content_epoch) const;
  bool SetRequestId(const std::string& request_id);
  void RecordTaskEvent(
      TaskEventKind kind,
      double timestamp_s,
      const std::string& request_id = {});
  bool SelectCurrentPoint();
  bool BeginPlanning(double now_s, std::string reason);
  void BeginDwell(double now_s);
  void BeginAction(double now_s);
  void HandlePointFailure(std::string reason, double now_s);
  void ResetArrivalStability();
  void ResetPointPhase();
  void AdvanceOrFinish(double now_s);
  void Fail(std::string reason, double now_s = 0.0);

  Route route_;
  RunStatus status_;
  bool has_route_{false};
  bool goal_submitted_{false};
  double dwell_deadline_s_{0.0};
  double last_arrival_sample_s_{0.0};
  bool has_arrival_sample_{false};
  std::size_t stable_sample_count_{0U};
  std::uint64_t action_request_serial_{0U};
  bool action_started_{false};
  double best_remaining_distance_m_{0.0};
  bool has_navigation_progress_{false};
  std::string pending_stop_reason_;
  std::deque<TaskEvent> task_events_;
  std::uint64_t next_task_event_sequence_{1U};
};

}  // namespace lingtu::nav::inspection
