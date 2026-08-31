#include "inspection.hpp"

#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace lingtu::nav::inspection {
namespace {

bool Finite(double value) {
  return std::isfinite(value);
}

bool SafeId(const std::string& value) {
  if (value.empty() || value.size() > 128U) {
    return false;
  }
  if (value.front() == '.' || value.front() == '-' ||
      value.find("..") != std::string::npos) {
    return false;
  }
  for (const unsigned char ch : value) {
    if (!std::isalnum(ch) && ch != '-' && ch != '_' && ch != '.') {
      return false;
    }
  }
  return true;
}

bool SafeTaskId(const std::string& value) {
  if (value.empty() || value.size() > 128U) {
    return false;
  }
  if (value.front() == '.' || value.front() == '-' ||
      value.find("..") != std::string::npos) {
    return false;
  }
  for (const unsigned char ch : value) {
    if (!std::isalnum(ch) && ch != '-' && ch != '_' && ch != '.' && ch != ':') {
      return false;
    }
  }
  return true;
}

bool SafeActionToken(const std::string& value) {
  if (value.empty()) return true;
  if (value.size() > 128U || value.front() == '.' || value.front() == '-' ||
      value.find("..") != std::string::npos) {
    return false;
  }
  for (const unsigned char ch : value) {
    if (!std::isalnum(ch) && ch != '-' && ch != '_' && ch != '.' && ch != ':') {
      return false;
    }
  }
  return true;
}

bool SafeText(const std::string& value, std::size_t max_size) {
  return value.size() <= max_size && value.find('\0') == std::string::npos;
}

bool KnownTaskEventKind(TaskEventKind kind) noexcept {
  switch (kind) {
    case TaskEventKind::kTaskAccepted:
    case TaskEventKind::kStateChanged:
    case TaskEventKind::kMilestone:
    case TaskEventKind::kStopConfirmationFailed:
    case TaskEventKind::kEvidenceRecorded:
      return true;
  }
  return false;
}

}  // namespace

ValidationResult ValidateRoute(const Route& route) {
  if (!SafeId(route.id)) return {false, "invalid_route_id", 0U};
  if (!SafeId(route.map_id)) return {false, "invalid_map_id", 0U};
  if (route.revision == 0U) return {false, "route_revision_zero", 0U};
  if (route.points.empty()) return {false, "route_has_no_points", 0U};
  if (route.points.size() > 256U) return {false, "route_point_limit_exceeded", 0U};
  if (route.loop_count == 0U) return {false, "loop_count_zero", 0U};

  std::unordered_set<std::string> ids;
  std::size_t enabled = 0U;
  for (const auto& point : route.points) {
    if (!SafeId(point.id)) return {false, "invalid_point_id", enabled};
    if (!ids.insert(point.id).second) return {false, "duplicate_point_id", enabled};
    if (point.frame_id != "map") return {false, "point_frame_must_be_map", enabled};
    if (!Finite(point.x_m) || !Finite(point.y_m) || !Finite(point.z_m) ||
        !Finite(point.yaw_rad)) {
      return {false, "point_pose_not_finite", enabled};
    }
    if (!Finite(point.position_tolerance_m) || point.position_tolerance_m <= 0.0 ||
        point.position_tolerance_m > 5.0) {
      return {false, "invalid_position_tolerance", enabled};
    }
    if (!Finite(point.yaw_tolerance_rad) || point.yaw_tolerance_rad <= 0.0 ||
        point.yaw_tolerance_rad > 3.14159265358979323846) {
      return {false, "invalid_yaw_tolerance", enabled};
    }
    if (!Finite(point.dwell_s) || point.dwell_s < 0.0 || point.dwell_s > 3600.0) {
      return {false, "invalid_dwell_time", enabled};
    }
    if (!SafeActionToken(point.action)) {
      return {false, "invalid_point_action", enabled};
    }
    if (point.enabled) ++enabled;
  }
  if (enabled == 0U) return {false, "route_has_no_enabled_points", 0U};
  return {true, "ok", enabled};
}

bool IsKnownCommandKind(std::int32_t value) noexcept {
  return lingtu::message::isKnownInspectionCommandKind(value);
}

const char* CommandKindName(CommandKind kind) noexcept {
  return lingtu::message::inspectionCommandKindName(kind);
}

const char* FailurePolicyName(FailurePolicy policy) noexcept {
  switch (policy) {
    case FailurePolicy::kStop: return "stop";
    case FailurePolicy::kRetry: return "retry";
    case FailurePolicy::kSkip: return "skip";
  }
  return "unknown";
}

const char* RunStateName(RunState state) noexcept {
  switch (state) {
    case RunState::kIdle: return "idle";
    case RunState::kValidating: return "validating";
    case RunState::kPlanning: return "planning";
    case RunState::kNavigating: return "navigating";
    case RunState::kDwelling: return "dwelling";
    case RunState::kPaused: return "paused";
    case RunState::kRecovering: return "recovering";
    case RunState::kSucceeded: return "succeeded";
    case RunState::kFailed: return "failed";
    case RunState::kCancelled: return "cancelled";
    case RunState::kSettling: return "settling";
    case RunState::kActionPending: return "action_pending";
    case RunState::kPausing: return "pausing";
    case RunState::kCancelling: return "cancelling";
  }
  return "unknown";
}

const char* TaskEventKindName(TaskEventKind kind) noexcept {
  switch (kind) {
    case TaskEventKind::kTaskAccepted: return "task_accepted";
    case TaskEventKind::kStateChanged: return "state_changed";
    case TaskEventKind::kMilestone: return "milestone";
    case TaskEventKind::kStopConfirmationFailed: return "stop_confirmation_failed";
    case TaskEventKind::kEvidenceRecorded: return "evidence_recorded";
  }
  return "unknown";
}

const char* TaskEventOutboxRecordResultName(TaskEventOutboxRecordResult result) noexcept {
  switch (result) {
    case TaskEventOutboxRecordResult::kAccepted: return "accepted";
    case TaskEventOutboxRecordResult::kInvalid: return "invalid";
    case TaskEventOutboxRecordResult::kOutOfOrder: return "out_of_order";
    case TaskEventOutboxRecordResult::kBackpressure: return "backpressure";
  }
  return "unknown";
}

bool IsActiveRunState(RunState state) noexcept {
  switch (state) {
    case RunState::kValidating:
    case RunState::kPlanning:
    case RunState::kNavigating:
    case RunState::kDwelling:
    case RunState::kPaused:
    case RunState::kRecovering:
    case RunState::kSettling:
    case RunState::kActionPending:
    case RunState::kPausing:
    case RunState::kCancelling:
      return true;
    case RunState::kIdle:
    case RunState::kSucceeded:
    case RunState::kFailed:
    case RunState::kCancelled:
      return false;
  }
  return false;
}

bool IsTerminalRunState(RunState state) noexcept {
  return state == RunState::kSucceeded || state == RunState::kFailed ||
      state == RunState::kCancelled;
}

TaskEventValidationResult ValidateTaskEvent(const TaskEvent& event) {
  const auto& status = event.status;
  if (event.sequence == 0U) return {false, "event_sequence_zero"};
  if (!Finite(event.timestamp_s) || event.timestamp_s < 0.0) {
    return {false, "event_timestamp_invalid"};
  }
  if (!KnownTaskEventKind(event.kind)) return {false, "event_kind_unknown"};
  if (!IsActiveRunState(status.state) && !IsTerminalRunState(status.state)) {
    return {false, "event_state_not_recoverable"};
  }
  if (!SafeTaskId(status.task_id) || status.task_id.size() > 96U) {
    return {false, "event_task_id_invalid"};
  }
  if (status.run_id != status.task_id) return {false, "event_run_id_mismatch"};
  if (!SafeTaskId(status.request_id) || status.request_id.size() > 96U) {
    return {false, "event_command_request_id_invalid"};
  }
  if (!SafeTaskId(event.request_id) || event.request_id.size() > 128U) {
    return {false, "event_request_id_invalid"};
  }
  if (!SafeId(status.map_id)) return {false, "event_map_id_invalid"};
  if (status.map_content_epoch < 0) return {false, "event_map_content_epoch_invalid"};
  if (!SafeId(status.route_id)) return {false, "event_route_id_invalid"};
  if (status.route_revision == 0U) return {false, "event_route_revision_zero"};
  if (status.point_index > 256U || status.point_count > 256U) {
    return {false, "event_point_index_invalid"};
  }
  if (!status.point_id.empty() && !SafeId(status.point_id)) {
    return {false, "event_point_id_invalid"};
  }
  if (!SafeActionToken(status.action)) return {false, "event_action_invalid"};
  if (!status.action_request_id.empty() &&
      (!SafeTaskId(status.action_request_id) || status.action_request_id.size() > 128U)) {
    return {false, "event_action_request_id_invalid"};
  }
  if (!status.evidence_id.empty() && !SafeId(status.evidence_id)) {
    return {false, "event_evidence_id_invalid"};
  }
  if (!Finite(status.phase_started_at_s) || status.phase_started_at_s < 0.0 ||
      !Finite(status.stable_since_s) || status.stable_since_s < 0.0 ||
      !Finite(status.deadline_s) || status.deadline_s < 0.0) {
    return {false, "event_status_time_invalid"};
  }
  if (!SafeText(status.reason, 1024U)) return {false, "event_reason_invalid"};
  if (event.kind == TaskEventKind::kTaskAccepted &&
      status.state != RunState::kValidating) {
    return {false, "task_accepted_state_invalid"};
  }
  if (event.kind == TaskEventKind::kStopConfirmationFailed &&
      status.state != RunState::kPausing && status.state != RunState::kCancelling) {
    return {false, "stop_confirmation_state_invalid"};
  }
  if (event.kind == TaskEventKind::kEvidenceRecorded && status.evidence_id.empty()) {
    return {false, "evidence_event_missing_evidence_id"};
  }
  return {true, "ok"};
}

RestartTaskEventResult ReconcileTaskEventAfterRestart(
    const TaskEvent& checkpoint,
    double timestamp_s) {
  const auto validation = ValidateTaskEvent(checkpoint);
  if (!validation.ok) {
    return {false, false, false, {}, "invalid_checkpoint:" + validation.reason};
  }
  if (IsTerminalRunState(checkpoint.status.state)) {
    return {true, true, false, checkpoint, "terminal_replay"};
  }
  if (!IsActiveRunState(checkpoint.status.state)) {
    return {false, false, false, {}, "checkpoint_state_not_recoverable"};
  }
  if (!Finite(timestamp_s) || timestamp_s < 0.0) {
    return {false, false, false, {}, "restart_timestamp_invalid"};
  }
  if (checkpoint.sequence == std::numeric_limits<std::uint64_t>::max()) {
    return {false, false, false, {}, "checkpoint_sequence_exhausted"};
  }

  TaskEvent failed = checkpoint;
  ++failed.sequence;
  failed.timestamp_s = timestamp_s;
  failed.kind = TaskEventKind::kStateChanged;
  failed.request_id = failed.status.request_id;
  failed.status.state = RunState::kFailed;
  failed.status.action_request_id.clear();
  failed.status.phase_started_at_s = 0.0;
  failed.status.stable_since_s = 0.0;
  failed.status.deadline_s = 0.0;
  failed.status.reason = "native_endpoint_restarted";
  const auto failed_validation = ValidateTaskEvent(failed);
  if (!failed_validation.ok) {
    return {false, false, false, {}, "restart_event_invalid:" + failed_validation.reason};
  }
  return {true, false, true, std::move(failed), "active_failed_after_restart"};
}

InspectionTaskEventOutbox::InspectionTaskEventOutbox(
    std::string boot_id,
    WriteCallback write,
    std::size_t capacity)
    : boot_id_(std::move(boot_id)), write_(std::move(write)), capacity_(capacity) {
  if (boot_id_.empty()) {
    throw std::invalid_argument("inspection task event outbox boot_id is required");
  }
  if (!write_) {
    throw std::invalid_argument("inspection task event outbox write callback is required");
  }
  if (capacity_ == 0U) {
    throw std::invalid_argument("inspection task event outbox capacity must be positive");
  }
}

bool InspectionTaskEventOutbox::InitializeNextSequence(std::uint64_t next_sequence) noexcept {
  if (next_sequence == 0U || initialized_ || recording_started_ || !pending_.empty() ||
      next_sequence < next_sequence_) {
    return false;
  }
  next_sequence_ = next_sequence;
  initialized_ = true;
  sequence_exhausted_ = false;
  return true;
}

TaskEventOutboxRecordResult InspectionTaskEventOutbox::Record(const TaskEvent& event) {
  recording_started_ = true;
  if (!ValidateTaskEvent(event).ok) {
    ++diagnostics_.rejected_invalid;
    return TaskEventOutboxRecordResult::kInvalid;
  }
  if (sequence_exhausted_ || event.sequence != next_sequence_) {
    ++diagnostics_.rejected_out_of_order;
    return TaskEventOutboxRecordResult::kOutOfOrder;
  }
  if (pending_.size() >= capacity_) {
    ++diagnostics_.rejected_backpressure;
    return TaskEventOutboxRecordResult::kBackpressure;
  }

  pending_.push_back(TaskEventEnvelope{boot_id_, event.sequence, event});
  if (event.sequence == std::numeric_limits<std::uint64_t>::max()) {
    sequence_exhausted_ = true;
  } else {
    ++next_sequence_;
  }
  ++diagnostics_.accepted;
  diagnostics_.pending = pending_.size();
  return TaskEventOutboxRecordResult::kAccepted;
}

std::size_t InspectionTaskEventOutbox::Flush(std::size_t max_events) {
  std::size_t delivered = 0U;
  while (delivered < max_events && !pending_.empty()) {
    bool wrote = false;
    try {
      wrote = write_(pending_.front());
    } catch (...) {
      wrote = false;
    }
    if (!wrote) {
      ++diagnostics_.delivery_failures;
      break;
    }
    pending_.pop_front();
    ++delivered;
    ++diagnostics_.delivered;
  }
  diagnostics_.pending = pending_.size();
  return delivered;
}

TaskEventOutboxDiagnostics InspectionTaskEventOutbox::diagnostics() const noexcept {
  return diagnostics_;
}

bool Executor::Start(
    Route route,
    std::string task_id,
    std::string request_id,
    const std::string& active_map_id,
    std::int64_t active_map_content_epoch,
    double now_s,
    std::string* error) {
  status_ = {};
  has_route_ = false;
  route_ = {};
  pending_stop_reason_.clear();
  ResetPointPhase();
  status_.state = RunState::kValidating;
  if (!Finite(now_s)) {
    Fail("invalid_start_time", now_s);
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  const auto validation = ValidateRoute(route);
  if (!validation.ok) {
    Fail(validation.reason, now_s);
    if (error != nullptr) *error = validation.reason;
    return false;
  }
  if (!SafeTaskId(task_id) || task_id.size() > 96U) {
    Fail("invalid_task_id", now_s);
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  if (!request_id.empty() && (!SafeTaskId(request_id) || request_id.size() > 96U)) {
    Fail("invalid_request_id", now_s);
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  if (!request_id.empty() && task_id == request_id) {
    Fail("task_id_must_differ_from_request_id", now_s);
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  route_ = std::move(route);
  has_route_ = true;
  status_.task_id = std::move(task_id);
  status_.run_id = status_.task_id;
  status_.request_id = std::move(request_id);
  status_.map_id = route_.map_id;
  status_.map_content_epoch = route_.map_content_epoch;
  status_.route_id = route_.id;
  status_.route_revision = route_.revision;
  status_.point_count = validation.enabled_points;
  if (!MapMatches(active_map_id, active_map_content_epoch)) {
    Fail("route_map_content_epoch_mismatch", now_s);
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  status_.point_index = 0U;
  status_.loop_index = 0U;
  status_.retry_count = 0U;
  goal_submitted_ = false;
  ResetPointPhase();
  if (!SelectCurrentPoint()) {
    Fail("route_has_no_enabled_points", now_s);
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  RecordTaskEvent(TaskEventKind::kTaskAccepted, now_s, status_.request_id);
  if (!BeginPlanning(now_s, "route_started")) {
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  return true;
}

bool Executor::Start(
    Route route,
    std::string run_id,
    const std::string& active_map_id,
    std::int64_t active_map_content_epoch,
    double now_s,
    std::string* error) {
  return Start(
      std::move(route),
      std::move(run_id),
      "",
      active_map_id,
      active_map_content_epoch,
      now_s,
      error);
}

bool Executor::RequestPause(
    const std::string& reason,
    const std::string& request_id,
    double now_s) {
  if (!active() || status_.state == RunState::kPaused ||
      status_.state == RunState::kPausing || status_.state == RunState::kCancelling) {
    return false;
  }
  if (!SetRequestId(request_id)) return false;
  pending_stop_reason_ = reason.empty() ? "paused" : reason;
  status_.state = RunState::kPausing;
  status_.reason = "pause_requested";
  goal_submitted_ = false;
  ResetPointPhase();
  RecordTaskEvent(TaskEventKind::kStateChanged, now_s, request_id);
  return true;
}

bool Executor::CommitPause(double now_s) {
  if (status_.state != RunState::kPausing) return false;
  status_.state = RunState::kPaused;
  status_.reason = pending_stop_reason_;
  pending_stop_reason_.clear();
  RecordTaskEvent(TaskEventKind::kStateChanged, now_s);
  return true;
}

bool Executor::RequestCancel(
    const std::string& reason,
    const std::string& request_id,
    double now_s) {
  if (!active() || status_.state == RunState::kPausing ||
      status_.state == RunState::kCancelling) {
    return false;
  }
  if (!SetRequestId(request_id)) return false;
  pending_stop_reason_ = reason.empty() ? "cancelled" : reason;
  status_.state = RunState::kCancelling;
  status_.reason = "cancel_requested";
  goal_submitted_ = false;
  ResetPointPhase();
  RecordTaskEvent(TaskEventKind::kStateChanged, now_s, request_id);
  return true;
}

bool Executor::CommitCancel(double now_s) {
  if (status_.state != RunState::kCancelling) return false;
  status_.state = RunState::kCancelled;
  status_.reason = pending_stop_reason_;
  pending_stop_reason_.clear();
  RecordTaskEvent(TaskEventKind::kStateChanged, now_s);
  return true;
}

void Executor::MarkStopConfirmationFailed(std::string reason, double now_s) {
  if (status_.state != RunState::kPausing && status_.state != RunState::kCancelling) {
    return;
  }
  status_.reason = reason.empty() ? "stop_confirmation_unconfirmed" : std::move(reason);
  RecordTaskEvent(TaskEventKind::kStopConfirmationFailed, now_s);
}

bool Executor::Pause(const std::string& reason) {
  return RequestPause(reason) && CommitPause();
}

bool Executor::Resume(
    const std::string& active_map_id,
    std::int64_t active_map_content_epoch,
    double now_s,
    const std::string& request_id) {
  if (status_.state != RunState::kPaused ||
      !MapMatches(active_map_id, active_map_content_epoch)) {
    return false;
  }
  if (!SetRequestId(request_id)) return false;
  return BeginPlanning(now_s, "resumed_replan_current_leg");
}

bool Executor::Cancel(const std::string& reason) {
  return RequestCancel(reason) && CommitCancel();
}

std::optional<Point> Executor::PendingGoal() const {
  if (!has_route_ || status_.state != RunState::kPlanning || goal_submitted_ ||
      status_.point_index >= route_.points.size()) {
    return std::nullopt;
  }
  return route_.points[status_.point_index];
}

bool Executor::OnPlanningStarted(double now_s) {
  if (status_.state != RunState::kPlanning || goal_submitted_ || !Finite(now_s)) {
    return false;
  }
  if (status_.deadline_s > 0.0 && now_s >= status_.deadline_s) {
    HandlePointFailure("planning_timeout", now_s);
    return false;
  }
  goal_submitted_ = true;
  status_.reason = "planning_started";
  RecordTaskEvent(TaskEventKind::kMilestone, now_s);
  return true;
}

bool Executor::OnPlanReady(double now_s) {
  if (status_.state != RunState::kPlanning || !goal_submitted_ || !Finite(now_s)) {
    return false;
  }
  if (status_.deadline_s > 0.0 && now_s >= status_.deadline_s) {
    HandlePointFailure("planning_timeout", now_s);
    return false;
  }
  status_.state = RunState::kNavigating;
  status_.phase_started_at_s = now_s;
  status_.deadline_s = now_s + kNavigationTimeoutS;
  status_.reason = "leg_path_ready";
  RecordTaskEvent(TaskEventKind::kStateChanged, now_s);
  return true;
}


bool Executor::OnNavigationProgress(
    double remaining_distance_m,
    double now_s) {
  if (status_.state != RunState::kNavigating || !Finite(now_s) ||
      !Finite(remaining_distance_m) || remaining_distance_m < 0.0) {
    return false;
  }
  if (status_.deadline_s > 0.0 && now_s >= status_.deadline_s) {
    HandlePointFailure("navigation_stalled", now_s);
    return false;
  }
  if (!has_navigation_progress_ ||
      best_remaining_distance_m_ - remaining_distance_m >=
          kNavigationMinProgressM) {
    best_remaining_distance_m_ = remaining_distance_m;
    has_navigation_progress_ = true;
    status_.deadline_s = now_s + kNavigationTimeoutS;
  }
  return true;
}

void Executor::OnLegFailed(const std::string& reason, double now_s) {
  if (status_.state != RunState::kPlanning) return;
  HandlePointFailure(reason.empty() ? "leg_failed" : reason, now_s);
}

bool Executor::OnNavigationFailed(const std::string& reason, double now_s) {
  if (status_.state != RunState::kNavigating || !Finite(now_s)) {
    return false;
  }
  HandlePointFailure(reason.empty() ? "navigation_failed" : reason, now_s);
  return true;
}

void Executor::OnGoalReached(double now_s) {
  if (status_.state != RunState::kNavigating ||
      status_.point_index >= route_.points.size() || !Finite(now_s)) {
    return;
  }
  goal_submitted_ = false;
  ResetPointPhase();
  status_.state = RunState::kSettling;
  status_.phase_started_at_s = now_s;
  status_.deadline_s = now_s + kSettlingTimeoutS;
  status_.reason = "point_reached_settling";
  RecordTaskEvent(TaskEventKind::kStateChanged, now_s);
}

bool Executor::OnArrivalSample(const ArrivalSample& sample, double now_s) {
  if (status_.state != RunState::kSettling || !Finite(now_s) ||
      !Finite(sample.sample_time_s) || !Finite(sample.linear_speed_mps) ||
      !Finite(sample.angular_speed_radps)) {
    return false;
  }

  const double previous_sample_s = last_arrival_sample_s_;
  const bool had_previous_sample = has_arrival_sample_;
  const bool monotonic =
      !had_previous_sample || sample.sample_time_s > previous_sample_s;
  const double age_s = now_s - sample.sample_time_s;
  const bool fresh =
      age_s >= -kArrivalFutureToleranceS && age_s <= kArrivalSampleMaxAgeS;
  if (!monotonic || !fresh) {
    ResetArrivalStability();
    return false;
  }
  last_arrival_sample_s_ = sample.sample_time_s;
  has_arrival_sample_ = true;
  if (had_previous_sample &&
      sample.sample_time_s - previous_sample_s > kArrivalMaxSampleGapS) {
    ResetArrivalStability();
  }

  const bool stable =
      std::abs(sample.linear_speed_mps) <= kStableLinearSpeedMps &&
      std::abs(sample.angular_speed_radps) <= kStableAngularSpeedRadps;
  if (!stable) {
    ResetArrivalStability();
    return true;
  }

  if (stable_sample_count_ == 0U) {
    status_.stable_since_s = sample.sample_time_s;
  }
  ++stable_sample_count_;
  if (stable_sample_count_ >= kStableSampleCount &&
      sample.sample_time_s - status_.stable_since_s >= kStableDurationS) {
    BeginDwell(now_s);
  }
  return true;
}

std::optional<ActionRequest> Executor::PendingAction() const {
  if (!has_route_ || status_.state != RunState::kActionPending || action_started_ ||
      status_.action_request_id.empty() || status_.point_index >= route_.points.size()) {
    return std::nullopt;
  }
  return ActionRequest{
      status_.action_request_id,
      status_.task_id,
      status_.run_id,
      status_.route_id,
      status_.route_revision,
      status_.point_index,
      status_.point_id,
      status_.action,
  };
}

bool Executor::OnActionStarted(const std::string& request_id, double now_s) {
  if (status_.state != RunState::kActionPending || action_started_ ||
      request_id != status_.action_request_id || !Finite(now_s) ||
      (status_.deadline_s > 0.0 && now_s >= status_.deadline_s)) {
    return false;
  }
  action_started_ = true;
  status_.phase_started_at_s = now_s;
  status_.deadline_s = now_s + kActionTimeoutS;
  status_.reason = "point_action_started";
  RecordTaskEvent(TaskEventKind::kMilestone, now_s, request_id);
  return true;
}

bool Executor::OnActionResult(
    const std::string& request_id,
    bool success,
    const std::string& evidence_id,
    const std::string& reason,
    double now_s) {
  if (status_.state != RunState::kActionPending || !action_started_ ||
      request_id != status_.action_request_id || !Finite(now_s) ||
      (status_.deadline_s > 0.0 && now_s >= status_.deadline_s)) {
    return false;
  }
  if (success) {
    if (!SafeId(evidence_id)) {
      HandlePointFailure("invalid_evidence_id", now_s);
      return true;
    }
    status_.evidence_id = evidence_id;
    status_.reason = "point_action_succeeded";
    RecordTaskEvent(TaskEventKind::kEvidenceRecorded, now_s, request_id);
    ResetPointPhase();
    AdvanceOrFinish(now_s);
    return true;
  }
  HandlePointFailure(reason.empty() ? "action_failed" : reason, now_s);
  return true;
}

void Executor::Tick(double now_s) {
  if (!Finite(now_s)) return;
  if (status_.state == RunState::kPlanning && status_.deadline_s > 0.0 &&
      now_s >= status_.deadline_s) {
    HandlePointFailure("planning_timeout", now_s);
    return;
  }
  if (status_.state == RunState::kNavigating && status_.deadline_s > 0.0 &&
      now_s >= status_.deadline_s) {
    HandlePointFailure("navigation_stalled", now_s);
    return;
  }
  if (status_.state == RunState::kSettling && status_.deadline_s > 0.0 &&
      now_s >= status_.deadline_s) {
    HandlePointFailure("settling_timeout", now_s);
    return;
  }
  if (status_.state == RunState::kDwelling && now_s >= dwell_deadline_s_) {
    if (status_.action.empty()) {
      ResetPointPhase();
      AdvanceOrFinish(now_s);
    } else {
      BeginAction(now_s);
    }
    return;
  }
  if (status_.state == RunState::kActionPending && status_.deadline_s > 0.0 &&
      now_s >= status_.deadline_s) {
    HandlePointFailure("action_timeout", now_s);
  }
}

void Executor::OnMapChanged(
    const std::string& active_map_id,
    std::int64_t active_map_content_epoch) {
  if (active() && status_.state != RunState::kPausing &&
      status_.state != RunState::kCancelling &&
      !MapMatches(active_map_id, active_map_content_epoch)) {
    Fail("active_map_changed", status_.phase_started_at_s);
  }
}

bool Executor::active() const noexcept {
  switch (status_.state) {
    case RunState::kValidating:
    case RunState::kPlanning:
    case RunState::kNavigating:
    case RunState::kDwelling:
    case RunState::kSettling:
    case RunState::kActionPending:
    case RunState::kPaused:
    case RunState::kRecovering:
    case RunState::kPausing:
    case RunState::kCancelling:
      return true;
    default:
      return false;
  }
}

const Route* Executor::route() const noexcept {
  return has_route_ ? &route_ : nullptr;
}

bool Executor::MapMatches(const std::string& map_id, std::int64_t map_content_epoch) const {
  return has_route_ && route_.map_id == map_id && route_.map_content_epoch == map_content_epoch;
}

bool Executor::SetRequestId(const std::string& request_id) {
  if (request_id.empty()) return true;
  if (!SafeTaskId(request_id) || request_id.size() > 96U || request_id == status_.task_id) {
    return false;
  }
  status_.request_id = request_id;
  return true;
}

void Executor::RecordTaskEvent(
    TaskEventKind kind,
    double timestamp_s,
    const std::string& request_id) {
  if (status_.task_id.empty()) return;
  const double safe_timestamp = Finite(timestamp_s) ? timestamp_s : status_.phase_started_at_s;
  task_events_.push_back(TaskEvent{
      next_task_event_sequence_++,
      Finite(safe_timestamp) ? safe_timestamp : 0.0,
      kind,
      request_id.empty() ? status_.request_id : request_id,
      status_,
  });
}

std::size_t Executor::FlushTaskEvents(
    const std::function<bool(const TaskEvent&)>& accept,
    std::size_t max_events) {
  if (!accept || max_events == 0U) return 0U;

  std::size_t delivered = 0U;
  while (delivered < max_events && !task_events_.empty()) {
    if (!accept(task_events_.front())) {
      break;
    }
    task_events_.pop_front();
    ++delivered;
  }
  return delivered;
}

std::vector<TaskEvent> Executor::TakeTaskEvents() {
  std::vector<TaskEvent> result;
  result.reserve(task_events_.size());
  while (!task_events_.empty()) {
    result.push_back(std::move(task_events_.front()));
    task_events_.pop_front();
  }
  return result;
}

bool Executor::SetNextTaskEventSequence(std::uint64_t next_sequence) noexcept {
  if (status_.state != RunState::kIdle || has_route_ || !task_events_.empty() ||
      next_sequence == 0U || next_sequence < next_task_event_sequence_ ||
      next_sequence == std::numeric_limits<std::uint64_t>::max()) {
    return false;
  }
  next_task_event_sequence_ = next_sequence;
  return true;
}

bool Executor::SelectCurrentPoint() {
  while (status_.point_index < route_.points.size() &&
         !route_.points[status_.point_index].enabled) {
    ++status_.point_index;
  }
  if (status_.point_index >= route_.points.size()) return false;
  status_.point_id = route_.points[status_.point_index].id;
  status_.action = route_.points[status_.point_index].action;
  return true;
}

bool Executor::BeginPlanning(double now_s, std::string reason) {
  if (!Finite(now_s)) {
    Fail("invalid_transition_time", now_s);
    return false;
  }
  ResetPointPhase();
  goal_submitted_ = false;
  status_.state = RunState::kPlanning;
  status_.phase_started_at_s = now_s;
  status_.deadline_s = now_s + kPlanningTimeoutS;
  status_.reason = std::move(reason);
  RecordTaskEvent(TaskEventKind::kStateChanged, now_s);
  return true;
}

void Executor::BeginDwell(double now_s) {
  const double dwell_s = route_.points[status_.point_index].dwell_s;
  dwell_deadline_s_ = now_s + dwell_s;
  status_.state = RunState::kDwelling;
  status_.phase_started_at_s = now_s;
  status_.deadline_s = dwell_deadline_s_;
  status_.reason = "point_settled_dwelling";
  stable_sample_count_ = 0U;
  RecordTaskEvent(TaskEventKind::kStateChanged, now_s);
}

void Executor::BeginAction(double now_s) {
  dwell_deadline_s_ = 0.0;
  action_started_ = false;
  status_.state = RunState::kActionPending;
  status_.phase_started_at_s = now_s;
  status_.deadline_s = now_s + kActionTimeoutS;
  status_.action_request_id =
      status_.run_id + "-action-" + std::to_string(++action_request_serial_);
  status_.reason = "point_action_pending";
  RecordTaskEvent(TaskEventKind::kStateChanged, now_s);
}

void Executor::HandlePointFailure(std::string reason, double now_s) {
  goal_submitted_ = false;
  ResetPointPhase();
  if (route_.failure_policy == FailurePolicy::kRetry &&
      status_.retry_count < route_.max_retries) {
    ++status_.retry_count;
    (void)BeginPlanning(now_s, std::move(reason));
    return;
  }
  if (route_.failure_policy == FailurePolicy::kSkip) {
    status_.reason = std::move(reason);
    RecordTaskEvent(TaskEventKind::kMilestone, now_s);
    AdvanceOrFinish(now_s);
    return;
  }
  Fail(std::move(reason));
}

void Executor::ResetArrivalStability() {
  stable_sample_count_ = 0U;
  status_.stable_since_s = 0.0;
}

void Executor::ResetPointPhase() {
  dwell_deadline_s_ = 0.0;
  last_arrival_sample_s_ = 0.0;
  has_arrival_sample_ = false;
  stable_sample_count_ = 0U;
  action_started_ = false;
  best_remaining_distance_m_ = 0.0;
  has_navigation_progress_ = false;
  status_.action_request_id.clear();
  status_.phase_started_at_s = 0.0;
  status_.stable_since_s = 0.0;
  status_.deadline_s = 0.0;
}

void Executor::AdvanceOrFinish(double now_s) {
  ResetPointPhase();
  status_.retry_count = 0U;
  ++status_.point_index;
  if (SelectCurrentPoint()) {
    (void)BeginPlanning(now_s, "next_point");
    return;
  }
  ++status_.loop_index;
  if (status_.loop_index < route_.loop_count) {
    status_.point_index = 0U;
    if (SelectCurrentPoint()) {
      (void)BeginPlanning(now_s, "next_loop");
      return;
    }
  }
  status_.state = RunState::kSucceeded;
  status_.point_index = status_.point_count;
  status_.point_id.clear();
  status_.action.clear();
  status_.reason = "route_complete";
  RecordTaskEvent(TaskEventKind::kStateChanged, now_s);
}

void Executor::Fail(std::string reason, double now_s) {
  ResetPointPhase();
  status_.state = RunState::kFailed;
  status_.reason = std::move(reason);
  goal_submitted_ = false;
  pending_stop_reason_.clear();
  RecordTaskEvent(TaskEventKind::kStateChanged, now_s);
}

}  // namespace lingtu::nav::inspection
