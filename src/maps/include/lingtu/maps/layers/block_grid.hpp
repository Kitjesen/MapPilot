#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace lingtu::maps {

struct BlockGridConfig {
  float cell_size_m{0.10F};
  std::int32_t block_size{16};
  float hit_log_odds{0.85F};
  float miss_log_odds{0.40F};
  float min_log_odds{-4.0F};
  float max_log_odds{4.0F};
  float occupied_threshold{0.60F};
  float prune_abs_log_odds_below{0.05F};
  std::uint64_t max_runtime_cells{2000000ULL};
  std::uint64_t max_runtime_blocks{4096ULL};
  std::uint64_t max_persisted_cells{20000000ULL};
};

struct BlockGridUpdateStats {
  std::size_t rays{0};
  std::size_t free_updates{0};
  std::size_t hit_updates{0};
  std::size_t cleared_cells{0};
  std::size_t pruned_cells{0};
  std::size_t capacity_rejections{0};
  std::size_t total_cells{0};
};

struct BlockGridRoi {
  float min_x_m{0.0F};
  float min_y_m{0.0F};
  float min_z_m{0.0F};
  float max_x_m{0.0F};
  float max_y_m{0.0F};
  float max_z_m{0.0F};
  std::size_t max_cells{0U};
  bool enabled{false};
};

struct BlockGridSnapshot {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  float cell_size_m{0.10F};
  std::uint64_t generation{0};
  std::vector<std::int32_t> ix;
  std::vector<std::int32_t> iy;
  std::vector<std::int32_t> iz;
  std::vector<float> center_x_m;
  std::vector<float> center_y_m;
  std::vector<float> center_z_m;
  std::vector<float> occupancy_probability;
  std::vector<std::uint16_t> hit_count;
  std::vector<std::uint16_t> miss_count;

  std::size_t Size() const { return ix.size(); }
};

class PersistentBlockGrid final {
 public:
  explicit PersistentBlockGrid(BlockGridConfig config = {});

  void Reset();
  void SetFrame(std::string frame_id);
  void SetStampNs(std::int64_t stamp_ns);

  void InsertHit(float x_m, float y_m, float z_m);
  void InsertRay(float origin_x_m, float origin_y_m, float origin_z_m, float hit_x_m, float hit_y_m,
                 float hit_z_m, float max_range_m = 0.0F);
  BlockGridUpdateStats InsertRays(const float *origins_xyz, const float *hits_xyz,
                                  std::size_t ray_count, float max_range_m = 0.0F);

  std::size_t ClearColumn(float x_m, float y_m);
  std::size_t ClearColumns(
      const float *columns_xy,
      std::size_t column_count,
      float min_z_m,
      float max_z_m);
  std::size_t Decay(float factor);
  bool Contains(float x_m, float y_m, float z_m) const;
  float OccupancyProbability(float x_m, float y_m, float z_m) const;
  std::size_t CellCount() const;
  std::uint64_t Generation() const;
  BlockGridUpdateStats LastStats() const;

  BlockGridSnapshot Snapshot(const BlockGridRoi &roi = {}) const;

  void SaveBinary(const std::filesystem::path &path) const;
  static PersistentBlockGrid LoadBinary(const std::filesystem::path &path,
                                        const BlockGridConfig &limits = {});
  static bool ValidateBinary(const std::filesystem::path &path, std::string *error = nullptr,
                             const BlockGridConfig &limits = {});

 private:
  struct CellKey {
    std::int32_t x{0};
    std::int32_t y{0};
    std::int32_t z{0};

    bool operator==(const CellKey &other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct BlockKey {
    std::int32_t x{0};
    std::int32_t y{0};
    std::int32_t z{0};

    bool operator==(const BlockKey &other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct Cell {
    float log_odds{0.0F};
    std::uint16_t hits{0};
    std::uint16_t misses{0};
  };

  struct Block {
    std::vector<Cell> cells;
    std::vector<std::uint8_t> occupied;
    std::size_t live_count{0};
  };

  struct BlockKeyHash {
    std::size_t operator()(const BlockKey &key) const;
  };

  static CellKey ToCellKey(float x_m, float y_m, float z_m, float cell_size_m);
  static BlockKey ToBlockKey(const CellKey &cell_key, std::int32_t block_size);
  static std::size_t CellOffset(const CellKey &cell_key, const BlockKey &block_key,
                                std::int32_t block_size);
  static float ProbabilityFromLogOdds(float log_odds);

  Block &EnsureBlock(const BlockKey &key);
  const Cell *FindCell(const CellKey &key) const;
  Cell *EnsureCell(const CellKey &key);
  bool ApplyMiss(const CellKey &key);
  bool ApplyHit(const CellKey &key);
  void PruneEmptyBlocks();

  BlockGridConfig config_;
  std::unordered_map<BlockKey, Block, BlockKeyHash> blocks_;
  std::string frame_id_{"map"};
  std::int64_t stamp_ns_{0};
  std::uint64_t generation_{0};
  std::size_t live_cell_count_{0U};
  BlockGridUpdateStats last_stats_;
};

}  // namespace lingtu::maps
