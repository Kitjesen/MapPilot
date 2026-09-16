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
  std::int32_t size_x{200};
  std::int32_t size_y{200};
  std::int32_t size_z{100};
  double resolution_m{0.05};
  double max_ray_range_m{5.0};
  double hit_log_odds{1.7346010553881064};
  double miss_log_odds{0.8472978603872036};
  double min_log_odds{-1.9924301646902063};
  double max_log_odds{3.8918202981106256};
  double occupied_probability{0.80};
  double inflation_radius_m{0.25};
  double inflation_z_up_m{0.10};
  double inflation_z_down_m{0.10};
  double ground_height_m{0.0};
  double local_update_range_x_m{0.0};
  double local_update_range_y_m{0.0};
  double local_update_range_z_m{0.0};
  std::int32_t roll_margin_x{96};
  std::int32_t roll_margin_y{96};
  std::int32_t roll_margin_z{46};
  std::int64_t decay_after_ns{0};
  double decay_factor{0.90};
  bool auto_roll{true};
  bool reject_out_of_order{true};
};

struct RollingOccupancyCellChunk {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  std::uint64_t generation{0U};
  float resolution_m{0.05F};
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
  double resolution_m{0.05};
  std::int32_t size_x{0};
  std::int32_t size_y{0};
  std::int32_t size_z{0};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  double origin_z_m{0.0};
  std::vector<std::uint8_t> state;
  std::vector<std::int16_t> log_odds_q8;

  std::size_t CellCount() const noexcept { return state.size(); }
  std::size_t Index(std::int32_t x, std::int32_t y, std::int32_t z) const;
  void Validate() const;
};

struct RollingInflatedSnapshot {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  std::uint64_t generation{0U};
  double resolution_m{0.05};
  std::int32_t size_x{0};
  std::int32_t size_y{0};
  std::int32_t size_z{0};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  double origin_z_m{0.0};
  std::size_t occupied_cells{0U};
  std::vector<std::uint8_t> occupied_bits;

  std::size_t CellCount() const noexcept;
  bool Occupied(std::int32_t x, std::int32_t y, std::int32_t z) const;
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
      double center_x_m = 0.0,
      double center_y_m = 0.0,
      double center_z_m = 0.0,
      std::int64_t stamp_ns = 0);

  RollingOccupancyCellChunk RollToCenter(
      double center_x_m,
      double center_y_m,
      double center_z_m,
      std::int64_t stamp_ns = 0);

  RollingOccupancyUpdateStats Update(const MapCloudFrame& frame);
  std::size_t Decay(std::int64_t now_ns);

  OccupancyState StateAt(double x_m, double y_m, double z_m) const;
  // A surface voxel is removable only when its entire volume is observed free.
  std::vector<std::uint8_t> ObservedFreeVoxels(
      const PointCloudView& centers, float voxel_size_m) const;
  double OccupancyProbability(double x_m, double y_m, double z_m) const;
  bool Contains(double x_m, double y_m, double z_m) const;
  bool InflatedContains(double x_m, double y_m, double z_m) const;

  RollingOccupancySnapshot Snapshot() const;
  RollingInflatedSnapshot InflatedSnapshot() const;
  RollingOccupancyCellChunk ObservedCells() const;
  RollingOccupancyCellChunk LastRolledOut() const;
  RollingOccupancyUpdateStats LastStats() const;
  RollingOccupancyConfig Config() const;
  std::uint64_t Generation() const;

 private:
  struct Cell {
    double log_odds{0.0};
    std::uint16_t hits{0U};
    std::uint16_t misses{0U};
    std::int64_t last_observed_ns{0};
    bool observed{false};
    bool unresolved_hit{false};
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
  static double Probability(double log_odds);
  static std::uint16_t SaturatingIncrement(std::uint16_t value);

  bool InBounds(const CellCoord& coord) const noexcept;
  bool WorldToCell(double x_m, double y_m, double z_m, CellCoord* out) const;
  std::size_t PhysicalIndex(const CellCoord& logical) const;
  OccupancyState StateFor(const Cell& cell) const;
  CellCoord PhysicalToLogical(std::size_t physical_index) const;
  void RefreshMembership(std::size_t physical_index);
  void UpdateInflation(
      const CellCoord& occupied,
      int delta);
  void InitializeOrigin(double center_x_m, double center_y_m, double center_z_m);
  RollResult RollToCenterLocked(
      double center_x_m,
      double center_y_m,
      double center_z_m,
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
  void TraceRay(
      double origin_x_m,
      double origin_y_m,
      double origin_z_m,
      double end_x_m,
      double end_y_m,
      double end_z_m,
      std::vector<CellCoord>* cells) const;
  bool ClipRayToWindow(
      double origin_x_m,
      double origin_y_m,
      double origin_z_m,
      double* end_x_m,
      double* end_y_m,
      double* end_z_m) const;
  std::size_t DecayLocked(std::int64_t now_ns);

  RollingOccupancyConfig config_;
  double occupied_log_odds_threshold_{0.0};
  std::vector<Cell> cells_;
  std::vector<std::uint32_t> ray_total_counts_;
  std::vector<std::uint32_t> ray_hit_counts_;
  std::vector<std::uint64_t> observed_bits_;
  // Collision seeds: historical occupancy united with the current scan hits.
  std::vector<std::uint64_t> occupied_bits_;
  std::vector<std::uint16_t> inflation_counts_;
  std::vector<std::uint64_t> inflated_bits_;
  std::vector<CellCoord> inflation_offsets_;
  bool collision_dirty_{false};
  std::int32_t ring_x_{0};
  std::int32_t ring_y_{0};
  std::int32_t ring_z_{0};
  double origin_x_m_{0.0};
  double origin_y_m_{0.0};
  double origin_z_m_{0.0};
  std::string frame_id_{"map"};
  std::int64_t stamp_ns_{0};
  std::uint64_t generation_{0U};
  RollingOccupancyCellChunk last_rolled_out_;
  RollingOccupancyUpdateStats last_stats_;
  mutable std::shared_mutex mutex_;
};

}  // namespace lingtu::maps::layers
