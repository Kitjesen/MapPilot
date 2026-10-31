#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace lingtu::maps::health {

enum class MetricState {
  kUnknown,
  kNotEnoughData,
  kOk,
};

struct HealthConfig {
  std::size_t max_localization_samples{256};
  std::size_t max_planning_outcomes{256};
  std::size_t max_collision_events{128};
  std::size_t max_artifact_validations{128};

  std::size_t min_localization_samples{5};
  std::size_t min_planning_outcomes{3};

  double max_localization_error_m{1.5};
  double max_covariance_trace{2.0};
  double collision_severity_scale{3.0};

  double freshness_grace_s{30.0};
  double freshness_stale_s{600.0};

  double localization_weight{0.30};
  double planning_weight{0.25};
  double collision_weight{0.20};
  double freshness_weight{0.15};
  double artifact_weight{0.10};
};

struct LocalizationSample {
  double timestamp_s{0.0};
  bool localized{false};
  double position_error_m{0.0};
  double covariance_trace{0.0};
  double quality{1.0};
  std::string source;
};

struct PlanningOutcome {
  double timestamp_s{0.0};
  bool success{false};
  std::string planner;
  std::string reason;
};

struct CollisionEvent {
  double timestamp_s{0.0};
  double severity{1.0};
  std::string source;
  std::string reason;
};

struct SourceBuildTimestamps {
  double source_timestamp_s{0.0};
  double build_started_s{0.0};
  double build_finished_s{0.0};
  std::string source_uri;
  std::string builder;
};

struct ArtifactValidation {
  double timestamp_s{0.0};
  std::string artifact_id;
  bool required{true};
  bool valid{false};
  std::string validator;
  std::string reason;
};

struct MetricValue {
  MetricState state{MetricState::kUnknown};
  double score{0.0};
  std::string reason;
  std::vector<std::string> provenance;
};

struct HealthSnapshot {
  MetricValue localization;
  MetricValue planning;
  MetricValue collision;
  MetricValue freshness;
  MetricValue artifact_validation;
  MetricValue overall;
  std::uint64_t localization_sample_count{0};
  std::uint64_t planning_outcome_count{0};
  std::uint64_t collision_event_count{0};
  std::uint64_t artifact_validation_count{0};
};

class MapHealthModel {
 public:
  explicit MapHealthModel(HealthConfig config = HealthConfig{});

  void IngestLocalizationSample(const LocalizationSample& sample);
  void IngestPlanningOutcome(const PlanningOutcome& outcome);
  void IngestCollisionEvent(const CollisionEvent& event);
  void SetSourceBuildTimestamps(const SourceBuildTimestamps& timestamps);
  void IngestArtifactValidation(const ArtifactValidation& validation);
  void Clear();

  HealthSnapshot Snapshot(double now_s) const;
  HealthConfig Config() const;

 private:
  template <typename T>
  static void PushBounded(std::deque<T>& values, const T& value, std::size_t limit) {
    if (limit == 0) {
      return;
    }
    values.push_back(value);
    while (values.size() > limit) {
      values.pop_front();
    }
  }

  MetricValue EvaluateLocalization() const;
  MetricValue EvaluatePlanning() const;
  MetricValue EvaluateCollision() const;
  MetricValue EvaluateFreshness(double now_s) const;
  MetricValue EvaluateArtifacts() const;
  MetricValue EvaluateOverall(const HealthSnapshot& partial) const;
  double LatestTimestamp() const;

  HealthConfig config_;
  mutable std::mutex mutex_;
  std::deque<LocalizationSample> localization_;
  std::deque<PlanningOutcome> planning_;
  std::deque<CollisionEvent> collisions_;
  std::deque<ArtifactValidation> artifacts_;
  SourceBuildTimestamps source_build_;
  bool has_source_build_{false};
};

const char* ToString(MetricState state);

}  // namespace lingtu::maps::health
