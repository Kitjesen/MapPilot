#pragma once

#include "message/cpp/inspection_command.hpp"

#include <cstddef>
#include <cstdint>
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
  std::int64_t map_version{0};
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
  std::string run_id;
  std::string route_id;
  std::uint64_t route_revision{0};
  std::size_t point_index{0};
  std::string point_id;
  std::string action;
};

struct RunStatus {
  RunState state{RunState::kIdle};
  std::string run_id;
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

ValidationResult ValidateRoute(const Route& route);
bool IsKnownCommandKind(std::int32_t value) noexcept;
const char* CommandKindName(CommandKind kind) noexcept;
const char* FailurePolicyName(FailurePolicy policy) noexcept;
const char* RunStateName(RunState state) noexcept;

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
      std::string run_id,
      const std::string& active_map_id,
      std::int64_t active_map_version,
      double now_s,
      std::string* error);
  bool Pause(const std::string& reason);
  bool Resume(
      const std::string& active_map_id,
      std::int64_t active_map_version,
      double now_s);
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
  void OnMapChanged(const std::string& active_map_id, std::int64_t active_map_version);

  bool active() const noexcept;
  const Route* route() const noexcept;
  const RunStatus& status() const noexcept { return status_; }

 private:
  bool MapMatches(const std::string& map_id, std::int64_t map_version) const;
  bool SelectCurrentPoint();
  bool BeginPlanning(double now_s, std::string reason);
  void BeginDwell(double now_s);
  void BeginAction(double now_s);
  void HandlePointFailure(std::string reason, double now_s);
  void ResetArrivalStability();
  void ResetPointPhase();
  void AdvanceOrFinish(double now_s);
  void Fail(std::string reason);

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
};

}  // namespace lingtu::nav::inspection
