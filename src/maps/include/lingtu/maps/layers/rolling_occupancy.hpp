#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include "lingtu/maps/cloud.hpp"

namespace lingtu::maps::layers {

enum class OccupancyState : std::uint8_t {
  kUnknown = 0U,
  kFree = 1U,
  kOccupied = 2U,
};

struct RollingOccupancyConfig {
  std::int32_t size_x{160};
  std::int32_t size_y{160};
  std::int32_t size_z{48};
  float resolution_m{0.25F};
  float max_ray_range_m{30.0F};
  float hit_log_odds{0.85F};
  float miss_log_odds{0.40F};
  float min_log_odds{-4.0F};
  float max_log_odds{4.0F};
  float occupied_probability{0.65F};
  float free_probability{0.35F};
  std::int32_t roll_margin_x{40};
  std::int32_t roll_margin_y{40};
  std::int32_t roll_margin_z{12};
  std::int64_t decay_after_ns{0};
  float decay_factor{0.90F};
  bool auto_roll{true};
  bool reject_out_of_order{true};
};

struct RollingOccupancyCellChunk {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  std::uint64_t generation{0U};
  float resolution_m{0.25F};
  std::vector<float> center_x_m;
  std::vector<float> center_y_m;
  std::vector<float> center_z_m;
  std::vector<std::int16_t> log_odds_q8;
  std::vector<std::uint16_t> hit_count;
  std::vector<std::uint16_t> miss_count;
  std::vector<std::uint8_t> state;

  std::size_t Size() const noexcept { return state.size(); }
  bool Empty() const noexcept { return state.empty(); }
  void Validate() const;
};

struct RollingOccupancySnapshot {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  std::uint64_t generation{0U};
  float resolution_m{0.25F};
  std::int32_t size_x{0};
  std::int32_t size_y{0};
  std::int32_t size_z{0};
  float origin_x_m{0.0F};
  float origin_y_m{0.0F};
  float origin_z_m{0.0F};
  std::vector<std::uint8_t> state;
  std::vector<std::int16_t> log_odds_q8;

  std::size_t CellCount() const noexcept { return state.size(); }
  std::size_t Index(std::int32_t x, std::int32_t y, std::int32_t z) const;
  void Validate() const;
};

struct RollingOccupancyUpdateStats {
  std::size_t input_points{0U};
  std::size_t accepted_points{0U};
  std::size_t rejected_points{0U};
  std::size_t unique_rays{0U};
  std::size_t free_updates{0U};
  std::size_t hit_updates{0U};
  std::size_t rolled_out_cells{0U};
  std::size_t decayed_cells{0U};
  std::uint64_t generation{0U};
  bool rolled{false};
};

class RollingOccupancyGrid final {
 public:
  explicit RollingOccupancyGrid(RollingOccupancyConfig config = {});

  RollingOccupancyGrid(const RollingOccupancyGrid&) = delete;
  RollingOccupancyGrid& operator=(const RollingOccupancyGrid&) = delete;

  void Reset(
      std::string frame_id = "map",
      float center_x_m = 0.0F,
      float center_y_m = 0.0F,
      float center_z_m = 0.0F,
      std::int64_t stamp_ns = 0);

  RollingOccupancyCellChunk RollToCenter(
      float center_x_m,
      float center_y_m,
      float center_z_m,
      std::int64_t stamp_ns = 0);

  RollingOccupancyUpdateStats Update(const MapCloudFrame& frame);
  std::size_t Decay(std::int64_t now_ns);

  OccupancyState StateAt(float x_m, float y_m, float z_m) const;
  float OccupancyProbability(float x_m, float y_m, float z_m) const;
  bool Contains(float x_m, float y_m, float z_m) const;

  RollingOccupancySnapshot Snapshot() const;
  RollingOccupancyCellChunk ObservedCells() const;
  RollingOccupancyCellChunk LastRolledOut() const;
  RollingOccupancyUpdateStats LastStats() const;
  RollingOccupancyConfig Config() const;
  std::uint64_t Generation() const;

 private:
  struct Cell {
    float log_odds{0.0F};
    std::uint16_t hits{0U};
    std::uint16_t misses{0U};
    std::int64_t last_observed_ns{0};
    bool observed{false};
  };

  struct CellCoord {
    std::int32_t x{0};
    std::int32_t y{0};
    std::int32_t z{0};

    bool operator==(const CellCoord& other) const noexcept {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct RollResult {
    RollingOccupancyCellChunk chunk;
    bool rolled{false};
  };

  static void ValidateConfig(const RollingOccupancyConfig& config);
  static float Probability(float log_odds);
  static std::uint16_t SaturatingIncrement(std::uint16_t value);

  bool InBounds(const CellCoord& coord) const noexcept;
  bool WorldToCell(float x_m, float y_m, float z_m, CellCoord* out) const;
  std::size_t PhysicalIndex(const CellCoord& logical) const;
  OccupancyState StateFor(const Cell& cell) const;
  CellCoord PhysicalToLogical(std::size_t physical_index) const;
  void InitializeOrigin(float center_x_m, float center_y_m, float center_z_m);
  RollResult RollToCenterLocked(
      float center_x_m,
      float center_y_m,
      float center_z_m,
      std::int64_t stamp_ns,
      bool commit_revision);
  RollingOccupancyCellChunk RollByLocked(
      std::int32_t shift_x,
      std::int32_t shift_y,
      std::int32_t shift_z,
      std::int64_t stamp_ns,
      bool commit_revision);
  RollingOccupancyCellChunk ChunkFromPhysicalIndices(
      const std::vector<std::size_t>& indices,
      std::int64_t stamp_ns,
      std::uint64_t generation) const;
  RollingOccupancyCellChunk ObservedCellsLocked() const;
  std::vector<CellCoord> TraceRay(
      float origin_x_m,
      float origin_y_m,
      float origin_z_m,
      float end_x_m,
      float end_y_m,
      float end_z_m) const;
  bool ClipRayToWindow(
      float origin_x_m,
      float origin_y_m,
      float origin_z_m,
      float* end_x_m,
      float* end_y_m,
      float* end_z_m) const;
  void AdvanceMarkEpoch();
  std::size_t DecayLocked(std::int64_t now_ns);

  RollingOccupancyConfig config_;
  std::vector<Cell> cells_;
  std::vector<std::uint32_t> free_marks_;
  std::vector<std::uint32_t> hit_marks_;
  std::uint32_t mark_epoch_{0U};
  std::int32_t ring_x_{0};
  std::int32_t ring_y_{0};
  std::int32_t ring_z_{0};
  float origin_x_m_{0.0F};
  float origin_y_m_{0.0F};
  float origin_z_m_{0.0F};
  std::string frame_id_{"map"};
  std::int64_t stamp_ns_{0};
  std::uint64_t generation_{0U};
  RollingOccupancyCellChunk last_rolled_out_;
  RollingOccupancyUpdateStats last_stats_;
  mutable std::shared_mutex mutex_;
};

}  // namespace lingtu::maps::layers
