#include "lingtu/maps/health.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace lingtu::maps::health {
namespace {

double Clamp01(double value) {
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::max(0.0, std::min(1.0, value));
}

double PositiveOr(double value, double fallback) {
  return value > 0.0 ? value : fallback;
}

std::string CountReason(std::size_t actual, std::size_t required, const char* noun) {
  std::ostringstream out;
  out << "need at least " << required << " " << noun << ", have " << actual;
  return out.str();
}

void AddProvenance(std::vector<std::string>& provenance, const std::string& value) {
  if (!value.empty()) {
    provenance.push_back(value);
  }
}

}  // namespace

MapHealthModel::MapHealthModel(HealthConfig config) : config_(config) {}

void MapHealthModel::IngestLocalizationSample(const LocalizationSample& sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  PushBounded(localization_, sample, config_.max_localization_samples);
}

void MapHealthModel::IngestPlanningOutcome(const PlanningOutcome& outcome) {
  std::lock_guard<std::mutex> lock(mutex_);
  PushBounded(planning_, outcome, config_.max_planning_outcomes);
}

void MapHealthModel::IngestCollisionEvent(const CollisionEvent& event) {
  std::lock_guard<std::mutex> lock(mutex_);
  PushBounded(collisions_, event, config_.max_collision_events);
}

void MapHealthModel::SetSourceBuildTimestamps(const SourceBuildTimestamps& timestamps) {
  std::lock_guard<std::mutex> lock(mutex_);
  source_build_ = timestamps;
  has_source_build_ = true;
}

void MapHealthModel::IngestArtifactValidation(const ArtifactValidation& validation) {
  std::lock_guard<std::mutex> lock(mutex_);
  PushBounded(artifacts_, validation, config_.max_artifact_validations);
}

void MapHealthModel::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  localization_.clear();
  planning_.clear();
  collisions_.clear();
  artifacts_.clear();
  source_build_ = SourceBuildTimestamps{};
  has_source_build_ = false;
}

HealthSnapshot MapHealthModel::Snapshot(double now_s) const {
  std::lock_guard<std::mutex> lock(mutex_);
  HealthSnapshot snapshot;
  snapshot.localization_sample_count = localization_.size();
  snapshot.planning_outcome_count = planning_.size();
  snapshot.collision_event_count = collisions_.size();
  snapshot.artifact_validation_count = artifacts_.size();
  snapshot.localization = EvaluateLocalization();
  snapshot.planning = EvaluatePlanning();
  snapshot.collision = EvaluateCollision();
  snapshot.freshness = EvaluateFreshness(now_s);
  snapshot.artifact_validation = EvaluateArtifacts();
  snapshot.overall = EvaluateOverall(snapshot);
  return snapshot;
}

HealthConfig MapHealthModel::Config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

MetricValue MapHealthModel::EvaluateLocalization() const {
  if (localization_.empty()) {
    return {MetricState::kUnknown, 0.0, "no localization samples", {}};
  }
  if (localization_.size() < config_.min_localization_samples) {
    MetricValue value;
    value.state = MetricState::kNotEnoughData;
    value.reason = CountReason(
        localization_.size(), config_.min_localization_samples, "localization samples");
    AddProvenance(value.provenance, localization_.back().source);
    return value;
  }

  double localized_count = 0.0;
  double error_score_sum = 0.0;
  double covariance_score_sum = 0.0;
  double quality_sum = 0.0;
  for (const auto& sample : localization_) {
    localized_count += sample.localized ? 1.0 : 0.0;
    error_score_sum += Clamp01(
        1.0 - std::max(0.0, sample.position_error_m) /
                  PositiveOr(config_.max_localization_error_m, 1.0));
    covariance_score_sum += Clamp01(
        1.0 - std::max(0.0, sample.covariance_trace) /
                  PositiveOr(config_.max_covariance_trace, 1.0));
    quality_sum += Clamp01(sample.quality);
  }
  const double n = static_cast<double>(localization_.size());
  const double score = Clamp01(
      0.35 * (localized_count / n) + 0.25 * (error_score_sum / n) +
      0.20 * (covariance_score_sum / n) + 0.20 * (quality_sum / n));

  MetricValue value;
  value.state = MetricState::kOk;
  value.score = score;
  value.reason = "localization stability from bounded recent sample window";
  AddProvenance(value.provenance, localization_.back().source);
  return value;
}

MetricValue MapHealthModel::EvaluatePlanning() const {
  if (planning_.empty()) {
    return {MetricState::kUnknown, 0.0, "no planning outcomes", {}};
  }
  if (planning_.size() < config_.min_planning_outcomes) {
    MetricValue value;
    value.state = MetricState::kNotEnoughData;
    value.reason = CountReason(planning_.size(), config_.min_planning_outcomes, "planning outcomes");
    AddProvenance(value.provenance, planning_.back().planner);
    return value;
  }

  const auto successes = std::count_if(planning_.begin(), planning_.end(), [](const auto& outcome) {
    return outcome.success;
  });
  MetricValue value;
  value.state = MetricState::kOk;
  value.score = Clamp01(static_cast<double>(successes) / static_cast<double>(planning_.size()));
  value.reason = "planning success rate from bounded recent outcome window";
  AddProvenance(value.provenance, planning_.back().planner);
  AddProvenance(value.provenance, planning_.back().reason);
  return value;
}

MetricValue MapHealthModel::EvaluateCollision() const {
  if (collisions_.empty()) {
    if (localization_.empty() && planning_.empty()) {
      return {MetricState::kUnknown, 0.0, "no collision or operational samples", {}};
    }
    return {MetricState::kOk, 1.0, "no collision events in bounded window", {}};
  }

  double severity_sum = 0.0;
  for (const auto& event : collisions_) {
    severity_sum += std::max(0.0, event.severity);
  }
  const double normalized = severity_sum /
      (static_cast<double>(collisions_.size()) * PositiveOr(config_.collision_severity_scale, 1.0));
  MetricValue value;
  value.state = MetricState::kOk;
  value.score = Clamp01(1.0 - normalized);
  value.reason = "collision penalty from bounded recent event window";
  AddProvenance(value.provenance, collisions_.back().source);
  AddProvenance(value.provenance, collisions_.back().reason);
  return value;
}

MetricValue MapHealthModel::EvaluateFreshness(double now_s) const {
  const double latest = LatestTimestamp();
  if (latest <= 0.0) {
    return {MetricState::kUnknown, 0.0, "no timestamps ingested", {}};
  }
  const double age_s = std::max(0.0, now_s - latest);
  const double grace_s = std::max(0.0, config_.freshness_grace_s);
  const double stale_s = std::max(grace_s, config_.freshness_stale_s);
  double score = 1.0;
  if (age_s > grace_s) {
    score = 1.0 - ((age_s - grace_s) / PositiveOr(stale_s - grace_s, 1.0));
  }
  MetricValue value;
  value.state = MetricState::kOk;
  value.score = Clamp01(score);
  value.reason = age_s <= grace_s
      ? "latest evidence is within freshness grace period"
      : (age_s >= stale_s
             ? "latest evidence is stale"
             : "latest evidence is decaying toward stale threshold");
  if (has_source_build_) {
    AddProvenance(value.provenance, source_build_.source_uri);
    AddProvenance(value.provenance, source_build_.builder);
  }
  return value;
}

MetricValue MapHealthModel::EvaluateArtifacts() const {
  if (artifacts_.empty()) {
    return {MetricState::kUnknown, 0.0, "no artifact validation results", {}};
  }

  std::size_t required = 0;
  std::size_t valid_required = 0;
  std::size_t valid_optional = 0;
  std::size_t optional = 0;
  MetricValue value;
  for (const auto& artifact : artifacts_) {
    if (artifact.required) {
      ++required;
      if (artifact.valid) {
        ++valid_required;
      }
    } else {
      ++optional;
      if (artifact.valid) {
        ++valid_optional;
      }
    }
  }
  if (required == 0) {
    value.state = MetricState::kNotEnoughData;
    value.reason = "no required artifacts validated";
  } else {
    value.state = MetricState::kOk;
    const double required_score =
        static_cast<double>(valid_required) / static_cast<double>(required);
    const double optional_score =
        optional == 0 ? 1.0 : static_cast<double>(valid_optional) / static_cast<double>(optional);
    value.score = Clamp01(0.85 * required_score + 0.15 * optional_score);
    value.reason = "artifact validation ratio with required artifacts weighted first";
  }
  AddProvenance(value.provenance, artifacts_.back().artifact_id);
  AddProvenance(value.provenance, artifacts_.back().validator);
  AddProvenance(value.provenance, artifacts_.back().reason);
  return value;
}

MetricValue MapHealthModel::EvaluateOverall(const HealthSnapshot& partial) const {
  const struct WeightedMetric {
    const MetricValue* value;
    double weight;
  } metrics[] = {
      {&partial.localization, config_.localization_weight},
      {&partial.planning, config_.planning_weight},
      {&partial.collision, config_.collision_weight},
      {&partial.freshness, config_.freshness_weight},
      {&partial.artifact_validation, config_.artifact_weight},
  };

  double weighted_sum = 0.0;
  double weight_sum = 0.0;
  bool saw_not_enough = false;
  std::vector<std::string> reasons;
  std::vector<std::string> provenance;
  for (const auto& metric : metrics) {
    if (metric.value->state == MetricState::kOk && metric.weight > 0.0) {
      weighted_sum += metric.value->score * metric.weight;
      weight_sum += metric.weight;
      provenance.insert(
          provenance.end(), metric.value->provenance.begin(), metric.value->provenance.end());
    } else if (metric.value->state == MetricState::kNotEnoughData) {
      saw_not_enough = true;
      reasons.push_back(metric.value->reason);
    }
  }

  if (weight_sum <= 0.0) {
    return {MetricState::kUnknown, 0.0, "no scored health dimensions", provenance};
  }

  MetricValue value;
  value.state = saw_not_enough ? MetricState::kNotEnoughData : MetricState::kOk;
  value.score = Clamp01(weighted_sum / weight_sum);
  if (saw_not_enough) {
    std::ostringstream reason;
    reason << "partial score; ";
    for (std::size_t i = 0; i < reasons.size(); ++i) {
      if (i != 0) {
        reason << "; ";
      }
      reason << reasons[i];
    }
    value.reason = reason.str();
  } else {
    value.reason = "weighted score across available health dimensions";
  }
  value.provenance = provenance;
  return value;
}

double MapHealthModel::LatestTimestamp() const {
  double latest = 0.0;
  for (const auto& sample : localization_) {
    latest = std::max(latest, sample.timestamp_s);
  }
  for (const auto& outcome : planning_) {
    latest = std::max(latest, outcome.timestamp_s);
  }
  for (const auto& event : collisions_) {
    latest = std::max(latest, event.timestamp_s);
  }
  for (const auto& artifact : artifacts_) {
    latest = std::max(latest, artifact.timestamp_s);
  }
  if (has_source_build_) {
    latest = std::max(latest, source_build_.source_timestamp_s);
    latest = std::max(latest, source_build_.build_started_s);
    latest = std::max(latest, source_build_.build_finished_s);
  }
  return latest;
}

const char* ToString(MetricState state) {
  switch (state) {
    case MetricState::kUnknown:
      return "unknown";
    case MetricState::kNotEnoughData:
      return "not_enough_data";
    case MetricState::kOk:
      return "ok";
  }
  return "unknown";
}

}  // namespace lingtu::maps::health
