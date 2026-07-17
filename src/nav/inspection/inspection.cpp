#include "inspection.hpp"

#include <cctype>
#include <cmath>
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
  }
  return "unknown";
}

bool Executor::Start(
    Route route,
    std::string run_id,
    const std::string& active_map_id,
    std::int64_t active_map_version,
    double now_s,
    std::string* error) {
  status_ = {};
  has_route_ = false;
  route_ = {};
  ResetPointPhase();
  status_.state = RunState::kValidating;
  if (!Finite(now_s)) {
    Fail("invalid_start_time");
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  const auto validation = ValidateRoute(route);
  if (!validation.ok) {
    Fail(validation.reason);
    if (error != nullptr) *error = validation.reason;
    return false;
  }
  if (!SafeId(run_id) || run_id.size() > 96U) {
    Fail("invalid_run_id");
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  route_ = std::move(route);
  has_route_ = true;
  status_.run_id = std::move(run_id);
  status_.route_id = route_.id;
  status_.route_revision = route_.revision;
  status_.point_count = validation.enabled_points;
  if (!MapMatches(active_map_id, active_map_version)) {
    Fail("route_map_version_mismatch");
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  status_.point_index = 0U;
  status_.loop_index = 0U;
  status_.retry_count = 0U;
  goal_submitted_ = false;
  ResetPointPhase();
  if (!SelectCurrentPoint()) {
    Fail("route_has_no_enabled_points");
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  if (!BeginPlanning(now_s, "route_started")) {
    if (error != nullptr) *error = status_.reason;
    return false;
  }
  return true;
}

bool Executor::Pause(const std::string& reason) {
  if (!active() || status_.state == RunState::kPaused) return false;
  status_.state = RunState::kPaused;
  status_.reason = reason.empty() ? "paused" : reason;
  goal_submitted_ = false;
  ResetPointPhase();
  return true;
}

bool Executor::Resume(
    const std::string& active_map_id,
    std::int64_t active_map_version,
    double now_s) {
  if (status_.state != RunState::kPaused ||
      !MapMatches(active_map_id, active_map_version)) {
    return false;
  }
  return BeginPlanning(now_s, "resumed_replan_current_leg");
}

bool Executor::Cancel(const std::string& reason) {
  if (!active()) return false;
  status_.state = RunState::kCancelled;
  status_.reason = reason.empty() ? "cancelled" : reason;
  goal_submitted_ = false;
  ResetPointPhase();
  return true;
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
    std::int64_t active_map_version) {
  if (active() && !MapMatches(active_map_id, active_map_version)) {
    Fail("active_map_changed");
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
      return true;
    default:
      return false;
  }
}

const Route* Executor::route() const noexcept {
  return has_route_ ? &route_ : nullptr;
}

bool Executor::MapMatches(const std::string& map_id, std::int64_t map_version) const {
  return has_route_ && route_.map_id == map_id && route_.map_version == map_version;
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
    Fail("invalid_transition_time");
    return false;
  }
  ResetPointPhase();
  goal_submitted_ = false;
  status_.state = RunState::kPlanning;
  status_.phase_started_at_s = now_s;
  status_.deadline_s = now_s + kPlanningTimeoutS;
  status_.reason = std::move(reason);
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
}

void Executor::Fail(std::string reason) {
  ResetPointPhase();
  status_.state = RunState::kFailed;
  status_.reason = std::move(reason);
  goal_submitted_ = false;
}

}  // namespace lingtu::nav::inspection
