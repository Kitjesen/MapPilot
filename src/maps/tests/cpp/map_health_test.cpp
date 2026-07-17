#include "lingtu/maps/health.hpp"

#include <cassert>
#include <thread>
#include <vector>

using lingtu::maps::health::ArtifactValidation;
using lingtu::maps::health::CollisionEvent;
using lingtu::maps::health::HealthConfig;
using lingtu::maps::health::LocalizationSample;
using lingtu::maps::health::MapHealthModel;
using lingtu::maps::health::MetricState;
using lingtu::maps::health::PlanningOutcome;
using lingtu::maps::health::SourceBuildTimestamps;

namespace {

HealthConfig TestConfig() {
  HealthConfig config;
  config.min_localization_samples = 3;
  config.min_planning_outcomes = 2;
  config.max_localization_samples = 8;
  config.max_planning_outcomes = 8;
  config.max_collision_events = 8;
  config.max_artifact_validations = 8;
  config.freshness_grace_s = 10.0;
  config.freshness_stale_s = 110.0;
  return config;
}

void AddGoodLocalization(MapHealthModel& model, double start_s, int count) {
  for (int i = 0; i < count; ++i) {
    model.IngestLocalizationSample(LocalizationSample{
        start_s + i,
        true,
        0.1,
        0.2,
        0.95,
        "slam"});
  }
}

void AddPlanning(MapHealthModel& model, double start_s, bool first, bool second) {
  model.IngestPlanningOutcome(PlanningOutcome{start_s, first, "octoplanner3d", "first"});
  model.IngestPlanningOutcome(PlanningOutcome{start_s + 1.0, second, "octoplanner3d", "second"});
}

}  // namespace

int main() {
  {
    MapHealthModel model(TestConfig());
    const auto snapshot = model.Snapshot(100.0);
    assert(snapshot.localization.state == MetricState::kUnknown);
    assert(snapshot.planning.state == MetricState::kUnknown);
    assert(snapshot.collision.state == MetricState::kUnknown);
    assert(snapshot.freshness.state == MetricState::kUnknown);
    assert(snapshot.artifact_validation.state == MetricState::kUnknown);
    assert(snapshot.overall.state == MetricState::kUnknown);
  }

  {
    MapHealthModel model(TestConfig());
    model.IngestLocalizationSample(LocalizationSample{10.0, true, 0.1, 0.1, 1.0, "slam"});
    const auto snapshot = model.Snapshot(11.0);
    assert(snapshot.localization.state == MetricState::kNotEnoughData);
    assert(snapshot.freshness.state == MetricState::kOk);
    assert(snapshot.overall.state == MetricState::kNotEnoughData);
  }

  {
    MapHealthModel model(TestConfig());
    AddGoodLocalization(model, 10.0, 3);
    AddPlanning(model, 20.0, true, false);
    model.SetSourceBuildTimestamps(SourceBuildTimestamps{5.0, 6.0, 7.0, "map.pcd", "builder"});
    model.IngestArtifactValidation(ArtifactValidation{21.0, "map.pcd", true, true, "sha256", ""});
    model.IngestArtifactValidation(
        ArtifactValidation{22.0, "occupancy.npz", true, false, "shape", "missing cells"});
    auto snapshot = model.Snapshot(25.0);
    assert(snapshot.localization.state == MetricState::kOk);
    assert(snapshot.localization.score > 0.90);
    assert(snapshot.planning.state == MetricState::kOk);
    assert(snapshot.planning.score == 0.5);
    assert(snapshot.collision.state == MetricState::kOk);
    assert(snapshot.collision.score == 1.0);
    assert(snapshot.artifact_validation.state == MetricState::kOk);
    assert(snapshot.artifact_validation.score > 0.40);
    assert(snapshot.artifact_validation.score < 0.60);
    assert(snapshot.overall.state == MetricState::kOk);
    assert(!snapshot.overall.provenance.empty());

    model.IngestCollisionEvent(CollisionEvent{26.0, 3.0, "safety_ring", "bumper"});
    snapshot = model.Snapshot(27.0);
    assert(snapshot.collision.score == 0.0);
    assert(snapshot.overall.score < 0.80);

    snapshot = model.Snapshot(140.0);
    assert(snapshot.freshness.score == 0.0);
  }

  {
    MapHealthModel model(TestConfig());
    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t) {
      threads.emplace_back([&model, t]() {
        for (int i = 0; i < 32; ++i) {
          model.IngestLocalizationSample(LocalizationSample{
              static_cast<double>(1000 + t * 100 + i),
              true,
              0.2,
              0.3,
              0.9,
              "threaded_slam"});
          model.IngestPlanningOutcome(PlanningOutcome{
              static_cast<double>(1000 + t * 100 + i),
              i % 3 != 0,
              "threaded_planner",
              ""});
        }
      });
    }
    for (auto& thread : threads) {
      thread.join();
    }
    const auto snapshot = model.Snapshot(2000.0);
    assert(snapshot.localization_sample_count == TestConfig().max_localization_samples);
    assert(snapshot.planning_outcome_count == TestConfig().max_planning_outcomes);
    assert(snapshot.localization.state == MetricState::kOk);
    assert(snapshot.planning.state == MetricState::kOk);
  }

  return 0;
}
