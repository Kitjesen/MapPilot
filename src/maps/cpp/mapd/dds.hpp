#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lingtu/maps/mapd/activation.hpp"
#include "lingtu/maps/mapd/engine.hpp"
#include "lingtu/maps/mapd/save_coordinator.hpp"

namespace lingtu::maps::mapd {

struct DdsInputState {
  std::uint64_t received_samples{0U};
  std::uint64_t decoded_samples{0U};
  std::uint64_t rejected_samples{0U};
  std::string last_error;
};

struct DdsOutputState {
  std::uint64_t write_attempts{0U};
  std::uint64_t write_failures{0U};
  std::uint64_t serialization_rejections{0U};
  std::uint64_t scene_oversize_rejections{0U};
  std::uint32_t unhealthy_writers{0U};
  std::string last_error;
};

struct PublicationCursor {
  std::uint64_t generation{0U};
  bool live{false};
  bool published_once{false};

  bool Pending(const State &state) const noexcept {
    return state.generation > 0U &&
           (!published_once || state.generation != generation || state.live != live);
  }

  bool PublishedFor(const State &state) const noexcept {
    return published_once && generation == state.generation && live == state.live;
  }

  void Complete(bool published, std::uint64_t next_generation, bool next_live) noexcept {
    if (!published) {
      return;
    }
    generation = next_generation;
    live = next_live;
    published_once = true;
  }
};

struct PublicationProgress {
  PublicationCursor state;
  PublicationCursor realtime_clouds;
  PublicationCursor map_layers;
  PublicationCursor scene;

  bool BootComplete() const noexcept {
    return state.published_once && realtime_clouds.published_once && map_layers.published_once &&
           scene.published_once;
  }

  bool CurrentGenerationPublished(const State &current) const noexcept {
    return state.PublishedFor(current) && realtime_clouds.PublishedFor(current) &&
           map_layers.PublishedFor(current) && scene.PublishedFor(current);
  }

  bool CurrentGenerationPublishedWhenStateArrives(const State &current) const noexcept {
    return realtime_clouds.PublishedFor(current) && map_layers.PublishedFor(current) &&
           scene.PublishedFor(current);
  }
};

struct ReadinessState {
  bool ready{false};
  const char *status{"stopped"};
};

inline ReadinessState EvaluateReadiness(const State &state, const DdsOutputState &output,
                                        const PublicationProgress &publications) noexcept {
  if (!state.running) {
    return {false, "stopped"};
  }
  if (!state.live) {
    return {false, "waiting_for_observation"};
  }
  if (output.unhealthy_writers != 0U) {
    return {false, "dds_output_degraded"};
  }
  if (state.capacity_limited) {
    return {false, "map_capacity_limited"};
  }
  if (!publications.BootComplete()) {
    return {false, "publishing_required_channels"};
  }
  if (!publications.CurrentGenerationPublished(state)) {
    return {false, "publishing_current_generation"};
  }
  return {true, "ready"};
}

struct DdsLimits {
  std::size_t max_points_per_observation{300000U};
  std::size_t max_cloud_bytes{16U * 1024U * 1024U};
  std::size_t max_point_fields{16U};
  std::size_t max_point_step{64U};
  std::size_t max_string_bytes{4096U};
  std::size_t max_scene_bytes{32U * 1024U * 1024U};
};

class Dds final : public SlamSnapshotExchange {
 public:
  Dds(int domain_id, std::size_t max_points_per_observation);
  Dds(int domain_id, DdsLimits limits);
  ~Dds();

  Dds(const Dds &) = delete;
  Dds &operator=(const Dds &) = delete;

  std::optional<Observation> TakeLatestObservation();
  bool PublishState(const State &state, const PublicationProgress &publications,
                    const MapIdentity &active_map);
  bool PublishRealtimeClouds(const State &state, const Snapshot &snapshot);
  bool PublishMapLayers(const State &state, const Snapshot &snapshot);
  bool PublishScene(const State &state, const Snapshot &snapshot);
  std::vector<ActivationRequest> TakeActivationRequests();
  bool PublishActivationAck(const ActivationResult &ack);
  bool Publish(const SlamSnapshotRequest &request) override;
  std::vector<SlamSnapshotAck> TakeAcks() override;

  const std::string &ProducerBootId() const;
  DdsInputState GetInputState() const;
  DdsOutputState GetOutputState() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::maps::mapd
