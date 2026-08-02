#include "lingtu/maps/block_grid.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <unordered_set>

#if defined(_WIN32)
#define NOMINMAX
#include <Windows.h>
#endif

namespace lingtu::maps {
namespace {

constexpr std::array<std::uint8_t, 8U> kMagic{'L', 'T', 'B', 'G', 'R', 'I', 'D', '\0'};
constexpr std::uint32_t kSchemaVersion = 1U;
constexpr std::uint32_t kEndianMarker = 0x01020304U;
constexpr std::uint32_t kHeaderSize = 96U;
constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

struct Header {
  std::array<std::uint8_t, 8U> magic{};
  std::uint32_t schema_version{0U};
  std::uint32_t endian_marker{0U};
  std::uint32_t header_size{0U};
  std::uint32_t frame_id_size{0U};
  float cell_size_m{0.0F};
  std::int32_t block_size{0};
  float hit_log_odds{0.0F};
  float miss_log_odds{0.0F};
  float min_log_odds{0.0F};
  float max_log_odds{0.0F};
  float occupied_threshold{0.0F};
  float prune_abs_log_odds_below{0.0F};
  std::uint64_t generation{0U};
  std::int64_t stamp_ns{0};
  std::uint64_t cell_count{0U};
  std::uint64_t body_size{0U};
  std::uint64_t checksum{0U};
};

struct PersistedCell {
  std::int32_t ix{0};
  std::int32_t iy{0};
  std::int32_t iz{0};
  float log_odds{0.0F};
  std::uint16_t hits{0};
  std::uint16_t misses{0};
};

std::uint64_t Mix(std::uint64_t value) {
  value ^= value >> 33U;
  value *= 0xff51afd7ed558ccdULL;
  value ^= value >> 33U;
  value *= 0xc4ceb9fe1a85ec53ULL;
  value ^= value >> 33U;
  return value;
}

bool IsFinite(float value) {
  return std::isfinite(static_cast<double>(value));
}

std::int32_t FloorDiv(std::int32_t value, std::int32_t divisor) {
  std::int32_t q = value / divisor;
  const std::int32_t r = value % divisor;
  if (r != 0 && ((r < 0) != (divisor < 0))) {
    --q;
  }
  return q;
}

std::uint16_t SaturatingIncrement(std::uint16_t value) {
  return value == std::numeric_limits<std::uint16_t>::max()
             ? value
             : static_cast<std::uint16_t>(value + 1U);
}

std::uint64_t Fnv1a(const std::vector<std::uint8_t> &bytes) {
  std::uint64_t hash = kFnvOffset;
  for (const auto byte : bytes) {
    hash ^= byte;
    hash *= kFnvPrime;
  }
  return hash;
}

template <typename T>
void AppendPod(std::vector<std::uint8_t> &out, const T &value) {
  const auto *bytes = reinterpret_cast<const std::uint8_t *>(&value);
  out.insert(out.end(), bytes, bytes + sizeof(T));
}

template <typename T>
void ReadPod(const std::vector<std::uint8_t> &body, std::size_t *cursor, T *out) {
  if (*cursor > body.size() || sizeof(T) > body.size() - *cursor) {
    throw std::runtime_error("block grid binary payload is truncated");
  }
  std::memcpy(out, body.data() + *cursor, sizeof(T));
  *cursor += sizeof(T);
}

void ValidateConfig(const BlockGridConfig &config) {
  if (!(config.cell_size_m > 0.0F) || !IsFinite(config.cell_size_m)) {
    throw std::invalid_argument("BlockGridConfig.cell_size_m must be finite and > 0");
  }
  if (config.block_size <= 0 || config.block_size > 128) {
    throw std::invalid_argument("BlockGridConfig.block_size must be in [1, 128]");
  }
  if (!IsFinite(config.hit_log_odds) || !(config.hit_log_odds > 0.0F) ||
      !IsFinite(config.miss_log_odds) || !(config.miss_log_odds > 0.0F)) {
    throw std::invalid_argument("BlockGridConfig hit/miss log odds must be finite and > 0");
  }
  if (!IsFinite(config.min_log_odds) || !IsFinite(config.max_log_odds) ||
      !(config.min_log_odds < config.max_log_odds)) {
    throw std::invalid_argument("BlockGridConfig log odds clamp is invalid");
  }
  if (!IsFinite(config.occupied_threshold) || config.occupied_threshold <= 0.0F ||
      config.occupied_threshold >= 1.0F) {
    throw std::invalid_argument("BlockGridConfig.occupied_threshold must be in (0, 1)");
  }
  if (!IsFinite(config.prune_abs_log_odds_below) || config.prune_abs_log_odds_below < 0.0F) {
    throw std::invalid_argument("BlockGridConfig.prune_abs_log_odds_below is invalid");
  }
  if (config.max_runtime_cells == 0U || config.max_runtime_blocks == 0U ||
      config.max_persisted_cells == 0U) {
    throw std::invalid_argument("BlockGridConfig capacity limits must be non-zero");
  }
}

void WriteHeader(std::ofstream &file, const Header &header) {
  file.write(reinterpret_cast<const char *>(header.magic.data()), header.magic.size());
  file.write(reinterpret_cast<const char *>(&header.schema_version), sizeof(header.schema_version));
  file.write(reinterpret_cast<const char *>(&header.endian_marker), sizeof(header.endian_marker));
  file.write(reinterpret_cast<const char *>(&header.header_size), sizeof(header.header_size));
  file.write(reinterpret_cast<const char *>(&header.frame_id_size), sizeof(header.frame_id_size));
  file.write(reinterpret_cast<const char *>(&header.cell_size_m), sizeof(header.cell_size_m));
  file.write(reinterpret_cast<const char *>(&header.block_size), sizeof(header.block_size));
  file.write(reinterpret_cast<const char *>(&header.hit_log_odds), sizeof(header.hit_log_odds));
  file.write(reinterpret_cast<const char *>(&header.miss_log_odds), sizeof(header.miss_log_odds));
  file.write(reinterpret_cast<const char *>(&header.min_log_odds), sizeof(header.min_log_odds));
  file.write(reinterpret_cast<const char *>(&header.max_log_odds), sizeof(header.max_log_odds));
  file.write(reinterpret_cast<const char *>(&header.occupied_threshold),
             sizeof(header.occupied_threshold));
  file.write(reinterpret_cast<const char *>(&header.prune_abs_log_odds_below),
             sizeof(header.prune_abs_log_odds_below));
  file.write(reinterpret_cast<const char *>(&header.generation), sizeof(header.generation));
  file.write(reinterpret_cast<const char *>(&header.stamp_ns), sizeof(header.stamp_ns));
  file.write(reinterpret_cast<const char *>(&header.cell_count), sizeof(header.cell_count));
  file.write(reinterpret_cast<const char *>(&header.body_size), sizeof(header.body_size));
  file.write(reinterpret_cast<const char *>(&header.checksum), sizeof(header.checksum));
}

Header ReadHeader(std::ifstream &file) {
  Header header;
  file.read(reinterpret_cast<char *>(header.magic.data()), header.magic.size());
  file.read(reinterpret_cast<char *>(&header.schema_version), sizeof(header.schema_version));
  file.read(reinterpret_cast<char *>(&header.endian_marker), sizeof(header.endian_marker));
  file.read(reinterpret_cast<char *>(&header.header_size), sizeof(header.header_size));
  file.read(reinterpret_cast<char *>(&header.frame_id_size), sizeof(header.frame_id_size));
  file.read(reinterpret_cast<char *>(&header.cell_size_m), sizeof(header.cell_size_m));
  file.read(reinterpret_cast<char *>(&header.block_size), sizeof(header.block_size));
  file.read(reinterpret_cast<char *>(&header.hit_log_odds), sizeof(header.hit_log_odds));
  file.read(reinterpret_cast<char *>(&header.miss_log_odds), sizeof(header.miss_log_odds));
  file.read(reinterpret_cast<char *>(&header.min_log_odds), sizeof(header.min_log_odds));
  file.read(reinterpret_cast<char *>(&header.max_log_odds), sizeof(header.max_log_odds));
  file.read(reinterpret_cast<char *>(&header.occupied_threshold),
            sizeof(header.occupied_threshold));
  file.read(reinterpret_cast<char *>(&header.prune_abs_log_odds_below),
            sizeof(header.prune_abs_log_odds_below));
  file.read(reinterpret_cast<char *>(&header.generation), sizeof(header.generation));
  file.read(reinterpret_cast<char *>(&header.stamp_ns), sizeof(header.stamp_ns));
  file.read(reinterpret_cast<char *>(&header.cell_count), sizeof(header.cell_count));
  file.read(reinterpret_cast<char *>(&header.body_size), sizeof(header.body_size));
  file.read(reinterpret_cast<char *>(&header.checksum), sizeof(header.checksum));
  if (!file) {
    throw std::runtime_error("block grid binary header is truncated");
  }
  return header;
}

std::filesystem::path TempPathFor(const std::filesystem::path &path) {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return path.parent_path() / (path.filename().string() + ".tmp." + std::to_string(stamp));
}

void AtomicReplace(const std::filesystem::path &temp, const std::filesystem::path &target) {
#if defined(_WIN32)
  if (!MoveFileExW(temp.c_str(), target.c_str(),
                   MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
    throw std::filesystem::filesystem_error(
        "failed to atomically replace block grid", temp, target,
        std::error_code(static_cast<int>(GetLastError()), std::system_category()));
  }
#else
  std::filesystem::rename(temp, target);
#endif
}

}  // namespace

std::size_t PersistentBlockGrid::BlockKeyHash::operator()(const BlockKey &key) const {
  const auto x = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.x));
  const auto y = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.y));
  const auto z = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.z));
  return static_cast<std::size_t>(Mix(x) ^ (Mix(y) << 1U) ^ (Mix(z) << 2U));
}

PersistentBlockGrid::PersistentBlockGrid(BlockGridConfig config) : config_(config) {
  ValidateConfig(config_);
}

void PersistentBlockGrid::Reset() {
  blocks_.clear();
  live_cell_count_ = 0U;
  ++generation_;
  last_stats_ = {};
}

void PersistentBlockGrid::SetFrame(std::string frame_id) {
  frame_id_ = frame_id.empty() ? "map" : std::move(frame_id);
}

void PersistentBlockGrid::SetStampNs(std::int64_t stamp_ns) {
  stamp_ns_ = stamp_ns;
}

void PersistentBlockGrid::InsertHit(float x_m, float y_m, float z_m) {
  if (!IsFinite(x_m) || !IsFinite(y_m) || !IsFinite(z_m)) {
    return;
  }
  const bool applied =
      ApplyHit(ToCellKey(x_m, y_m, z_m, config_.cell_size_m));
  if (applied) {
    ++generation_;
  }
  last_stats_ = {};
  last_stats_.hit_updates = applied ? 1U : 0U;
  last_stats_.capacity_rejections = applied ? 0U : 1U;
  last_stats_.total_cells = CellCount();
}

void PersistentBlockGrid::InsertRay(float origin_x_m, float origin_y_m, float origin_z_m,
                                    float hit_x_m, float hit_y_m, float hit_z_m,
                                    float max_range_m) {
  const float origins[3] = {origin_x_m, origin_y_m, origin_z_m};
  const float hits[3] = {hit_x_m, hit_y_m, hit_z_m};
  static_cast<void>(InsertRays(origins, hits, 1U, max_range_m));
}

BlockGridUpdateStats PersistentBlockGrid::InsertRays(const float *origins_xyz,
                                                     const float *hits_xyz, std::size_t ray_count,
                                                     float max_range_m) {
  BlockGridUpdateStats stats;
  if (ray_count == 0U) {
    last_stats_ = stats;
    return stats;
  }
  if (origins_xyz == nullptr || hits_xyz == nullptr) {
    throw std::invalid_argument("InsertRays requires origin and hit arrays");
  }
  if (max_range_m < 0.0F || !IsFinite(max_range_m)) {
    throw std::invalid_argument("InsertRays max_range_m must be finite and >= 0");
  }

  const float step_m = std::max(config_.cell_size_m * 0.5F, 1.0e-4F);
  for (std::size_t ray = 0; ray < ray_count; ++ray) {
    const float ox = origins_xyz[ray * 3U];
    const float oy = origins_xyz[ray * 3U + 1U];
    const float oz = origins_xyz[ray * 3U + 2U];
    float hx = hits_xyz[ray * 3U];
    float hy = hits_xyz[ray * 3U + 1U];
    float hz = hits_xyz[ray * 3U + 2U];
    if (!IsFinite(ox) || !IsFinite(oy) || !IsFinite(oz) || !IsFinite(hx) || !IsFinite(hy) ||
        !IsFinite(hz)) {
      continue;
    }

    float dx = hx - ox;
    float dy = hy - oy;
    float dz = hz - oz;
    float length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(length > 0.0F)) {
      continue;
    }
    if (max_range_m > 0.0F && length > max_range_m) {
      const float scale = max_range_m / length;
      hx = ox + dx * scale;
      hy = oy + dy * scale;
      hz = oz + dz * scale;
      dx = hx - ox;
      dy = hy - oy;
      dz = hz - oz;
      length = max_range_m;
    }

    const auto hit_key = ToCellKey(hx, hy, hz, config_.cell_size_m);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / step_m)));
    std::vector<CellKey> free_keys;
    free_keys.reserve(std::min<std::size_t>(
        static_cast<std::size_t>(steps), config_.max_runtime_cells));
    std::unordered_set<BlockKey, BlockKeyHash> new_blocks;
    std::size_t new_cells = 0U;
    bool capacity_rejected = false;
    const std::size_t available_cells =
        static_cast<std::size_t>(config_.max_runtime_cells) - live_cell_count_;
    const std::size_t available_blocks =
        static_cast<std::size_t>(config_.max_runtime_blocks) - blocks_.size();
    const auto register_new_cell = [&](const CellKey &key) {
      if (FindCell(key) != nullptr) {
        return true;
      }
      ++new_cells;
      const BlockKey block_key = ToBlockKey(key, config_.block_size);
      if (blocks_.find(block_key) == blocks_.end()) {
        new_blocks.insert(block_key);
      }
      return new_cells <= available_cells && new_blocks.size() <= available_blocks;
    };

    CellKey previous_free{std::numeric_limits<std::int32_t>::min(), 0, 0};
    for (int i = 0; i < steps; ++i) {
      const float t = static_cast<float>(i) / static_cast<float>(steps);
      const auto free_key = ToCellKey(ox + dx * t, oy + dy * t, oz + dz * t, config_.cell_size_m);
      if (free_key == hit_key || free_key == previous_free) {
        continue;
      }
      free_keys.push_back(free_key);
      if (!register_new_cell(free_key)) {
        capacity_rejected = true;
        break;
      }
      previous_free = free_key;
    }
    if (!capacity_rejected && !register_new_cell(hit_key)) {
      capacity_rejected = true;
    }
    if (capacity_rejected) {
      stats.capacity_rejections += std::max<std::size_t>(1U, new_cells);
      ++stats.rays;
      continue;
    }

    for (const CellKey &free_key : free_keys) {
      if (!ApplyMiss(free_key)) {
        throw std::logic_error("block grid ray capacity preflight invariant failed");
      }
      ++stats.free_updates;
    }
    if (!ApplyHit(hit_key)) {
      throw std::logic_error("block grid ray hit capacity preflight invariant failed");
    }
    ++stats.hit_updates;
    ++stats.rays;
  }
  if (stats.free_updates > 0U || stats.hit_updates > 0U) {
    ++generation_;
  }
  stats.total_cells = CellCount();
  last_stats_ = stats;
  return stats;
}

std::size_t PersistentBlockGrid::ClearColumn(float x_m, float y_m) {
  if (!IsFinite(x_m) || !IsFinite(y_m)) {
    return 0U;
  }
  const float column[2] = {x_m, y_m};
  return ClearColumns(
      column,
      1U,
      -std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max());
}

std::size_t PersistentBlockGrid::ClearColumns(
    const float *columns_xy,
    std::size_t column_count,
    float min_z_m,
    float max_z_m) {
  if (column_count == 0U) {
    return 0U;
  }
  if (columns_xy == nullptr || !IsFinite(min_z_m) || !IsFinite(max_z_m) ||
      min_z_m > max_z_m) {
    throw std::invalid_argument("ClearColumns arguments are invalid");
  }
  std::unordered_set<std::uint64_t> columns;
  columns.reserve(column_count);
  for (std::size_t index = 0U; index < column_count; ++index) {
    const float x_m = columns_xy[index * 2U];
    const float y_m = columns_xy[index * 2U + 1U];
    if (!IsFinite(x_m) || !IsFinite(y_m)) {
      continue;
    }
    const auto key = ToCellKey(x_m, y_m, 0.0F, config_.cell_size_m);
    columns.insert(
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.x)) << 32U) |
        static_cast<std::uint32_t>(key.y));
  }
  if (columns.empty()) {
    return 0U;
  }

  std::size_t cleared = 0U;
  for (auto &item : blocks_) {
    Block &block = item.second;
    for (std::size_t offset = 0U; offset < block.occupied.size(); ++offset) {
      if (block.occupied[offset] == 0U) {
        continue;
      }
      const std::int32_t local_x = static_cast<std::int32_t>(offset % config_.block_size);
      const std::int32_t local_y =
          static_cast<std::int32_t>((offset / config_.block_size) % config_.block_size);
      const std::int32_t global_x = item.first.x * config_.block_size + local_x;
      const std::int32_t global_y = item.first.y * config_.block_size + local_y;
      const std::int32_t local_z =
          static_cast<std::int32_t>(
              offset / (config_.block_size * config_.block_size));
      const std::int32_t global_z =
          item.first.z * config_.block_size + local_z;
      const float center_z =
          (static_cast<float>(global_z) + 0.5F) * config_.cell_size_m;
      const std::uint64_t column =
          (static_cast<std::uint64_t>(
               static_cast<std::uint32_t>(global_x))
           << 32U) |
          static_cast<std::uint32_t>(global_y);
      if (center_z >= min_z_m && center_z <= max_z_m &&
          columns.find(column) != columns.end()) {
        block.occupied[offset] = 0U;
        block.cells[offset] = {};
        --block.live_count;
        --live_cell_count_;
        ++cleared;
      }
    }
  }
  if (cleared > 0U) {
    PruneEmptyBlocks();
    ++generation_;
  }
  last_stats_ = {};
  last_stats_.cleared_cells = cleared;
  last_stats_.total_cells = CellCount();
  return cleared;
}

std::size_t PersistentBlockGrid::Decay(float factor) {
  if (!IsFinite(factor) || factor < 0.0F || factor > 1.0F) {
    throw std::invalid_argument("Decay factor must be in [0, 1]");
  }
  std::size_t pruned = 0U;
  bool changed = false;
  for (auto &item : blocks_) {
    Block &block = item.second;
    for (std::size_t i = 0U; i < block.occupied.size(); ++i) {
      if (block.occupied[i] == 0U) {
        continue;
      }
      const float previous = block.cells[i].log_odds;
      block.cells[i].log_odds *= factor;
      changed = changed || block.cells[i].log_odds != previous;
      if (std::fabs(block.cells[i].log_odds) < config_.prune_abs_log_odds_below) {
        block.occupied[i] = 0U;
        block.cells[i] = {};
        --block.live_count;
        --live_cell_count_;
        ++pruned;
      }
    }
  }
  if (pruned > 0U) {
    PruneEmptyBlocks();
  }
  if (changed) {
    ++generation_;
  }
  last_stats_ = {};
  last_stats_.pruned_cells = pruned;
  last_stats_.total_cells = CellCount();
  return pruned;
}

bool PersistentBlockGrid::Contains(float x_m, float y_m, float z_m) const {
  return OccupancyProbability(x_m, y_m, z_m) >= config_.occupied_threshold;
}

float PersistentBlockGrid::OccupancyProbability(float x_m, float y_m, float z_m) const {
  if (!IsFinite(x_m) || !IsFinite(y_m) || !IsFinite(z_m)) {
    return 0.5F;
  }
  const Cell *cell = FindCell(ToCellKey(x_m, y_m, z_m, config_.cell_size_m));
  return cell == nullptr ? 0.5F : ProbabilityFromLogOdds(cell->log_odds);
}

std::size_t PersistentBlockGrid::CellCount() const {
  return live_cell_count_;
}

std::uint64_t PersistentBlockGrid::Generation() const {
  return generation_;
}

BlockGridUpdateStats PersistentBlockGrid::LastStats() const {
  return last_stats_;
}

BlockGridSnapshot PersistentBlockGrid::Snapshot(const BlockGridRoi &roi) const {
  if (roi.enabled &&
      (!IsFinite(roi.min_x_m) || !IsFinite(roi.min_y_m) ||
       !IsFinite(roi.min_z_m) || !IsFinite(roi.max_x_m) ||
       !IsFinite(roi.max_y_m) || !IsFinite(roi.max_z_m) ||
       roi.min_x_m > roi.max_x_m || roi.min_y_m > roi.max_y_m ||
       roi.min_z_m > roi.max_z_m)) {
    throw std::invalid_argument("block grid snapshot ROI is invalid");
  }
  BlockGridSnapshot snapshot;
  snapshot.frame_id = frame_id_;
  snapshot.stamp_ns = stamp_ns_;
  snapshot.cell_size_m = config_.cell_size_m;
  snapshot.generation = generation_;
  std::vector<CellKey> keys;
  const std::size_t reserve = roi.max_cells > 0U
      ? std::min(CellCount(), roi.max_cells)
      : CellCount();
  keys.reserve(reserve);
  using Candidate =
      std::tuple<double, std::int32_t, std::int32_t, std::int32_t>;
  std::priority_queue<Candidate> nearest;
  const double center_x =
      roi.enabled ? 0.5 * (roi.min_x_m + roi.max_x_m) : 0.0;
  const double center_y =
      roi.enabled ? 0.5 * (roi.min_y_m + roi.max_y_m) : 0.0;
  const double center_z =
      roi.enabled ? 0.5 * (roi.min_z_m + roi.max_z_m) : 0.0;

  for (const auto &item : blocks_) {
    const BlockKey &block_key = item.first;
    const Block &block = item.second;
    for (std::size_t offset = 0U; offset < block.occupied.size(); ++offset) {
      if (block.occupied[offset] == 0U) {
        continue;
      }
      const std::int32_t local_x =
          static_cast<std::int32_t>(offset % config_.block_size);
      const std::int32_t local_y = static_cast<std::int32_t>(
          (offset / config_.block_size) % config_.block_size);
      const std::int32_t local_z = static_cast<std::int32_t>(
          offset / (config_.block_size * config_.block_size));
      const CellKey key{
          block_key.x * config_.block_size + local_x,
          block_key.y * config_.block_size + local_y,
          block_key.z * config_.block_size + local_z,
      };
      const float cx =
          (static_cast<float>(key.x) + 0.5F) * config_.cell_size_m;
      const float cy =
          (static_cast<float>(key.y) + 0.5F) * config_.cell_size_m;
      const float cz =
          (static_cast<float>(key.z) + 0.5F) * config_.cell_size_m;
      if (roi.enabled &&
          (cx < roi.min_x_m || cx > roi.max_x_m ||
           cy < roi.min_y_m || cy > roi.max_y_m ||
           cz < roi.min_z_m || cz > roi.max_z_m)) {
        continue;
      }
      if (roi.max_cells == 0U) {
        keys.push_back(key);
        continue;
      }
      const double dx = static_cast<double>(cx) - center_x;
      const double dy = static_cast<double>(cy) - center_y;
      const double dz = static_cast<double>(cz) - center_z;
      const Candidate candidate{
          dx * dx + dy * dy + dz * dz, key.x, key.y, key.z};
      if (nearest.size() < roi.max_cells) {
        nearest.push(candidate);
      } else if (candidate < nearest.top()) {
        nearest.pop();
        nearest.push(candidate);
      }
    }
  }
  while (!nearest.empty()) {
    const auto &[distance, x, y, z] = nearest.top();
    static_cast<void>(distance);
    keys.push_back({x, y, z});
    nearest.pop();
  }
  std::sort(keys.begin(), keys.end(), [](const CellKey &lhs, const CellKey &rhs) {
    return std::tie(lhs.x, lhs.y, lhs.z) < std::tie(rhs.x, rhs.y, rhs.z);
  });

  snapshot.ix.reserve(reserve);
  snapshot.iy.reserve(reserve);
  snapshot.iz.reserve(reserve);
  snapshot.center_x_m.reserve(reserve);
  snapshot.center_y_m.reserve(reserve);
  snapshot.center_z_m.reserve(reserve);
  snapshot.occupancy_probability.reserve(reserve);
  snapshot.hit_count.reserve(reserve);
  snapshot.miss_count.reserve(reserve);

  for (const CellKey &key : keys) {
    const Cell *cell = FindCell(key);
    if (cell == nullptr) {
      continue;
    }
    snapshot.ix.push_back(key.x);
    snapshot.iy.push_back(key.y);
    snapshot.iz.push_back(key.z);
    snapshot.center_x_m.push_back(
        (static_cast<float>(key.x) + 0.5F) * config_.cell_size_m);
    snapshot.center_y_m.push_back(
        (static_cast<float>(key.y) + 0.5F) * config_.cell_size_m);
    snapshot.center_z_m.push_back(
        (static_cast<float>(key.z) + 0.5F) * config_.cell_size_m);
    snapshot.occupancy_probability.push_back(
        ProbabilityFromLogOdds(cell->log_odds));
    snapshot.hit_count.push_back(cell->hits);
    snapshot.miss_count.push_back(cell->misses);
  }
  return snapshot;
}

void PersistentBlockGrid::SaveBinary(const std::filesystem::path &path) const {
  if (CellCount() > config_.max_persisted_cells) {
    throw std::runtime_error("block grid cell count exceeds persisted artifact limit");
  }
  std::vector<std::uint8_t> body;
  const auto snapshot = Snapshot();
  body.reserve(frame_id_.size() + snapshot.Size() * sizeof(PersistedCell));
  body.insert(body.end(), frame_id_.begin(), frame_id_.end());
  for (std::size_t i = 0U; i < snapshot.Size(); ++i) {
    const Cell *cell = FindCell({snapshot.ix[i], snapshot.iy[i], snapshot.iz[i]});
    if (cell == nullptr) {
      continue;
    }
    PersistedCell stored;
    stored.ix = snapshot.ix[i];
    stored.iy = snapshot.iy[i];
    stored.iz = snapshot.iz[i];
    stored.log_odds = cell->log_odds;
    stored.hits = cell->hits;
    stored.misses = cell->misses;
    AppendPod(body, stored);
  }

  Header header;
  header.magic = kMagic;
  header.schema_version = kSchemaVersion;
  header.endian_marker = kEndianMarker;
  header.header_size = kHeaderSize;
  header.frame_id_size = static_cast<std::uint32_t>(frame_id_.size());
  header.cell_size_m = config_.cell_size_m;
  header.block_size = config_.block_size;
  header.hit_log_odds = config_.hit_log_odds;
  header.miss_log_odds = config_.miss_log_odds;
  header.min_log_odds = config_.min_log_odds;
  header.max_log_odds = config_.max_log_odds;
  header.occupied_threshold = config_.occupied_threshold;
  header.prune_abs_log_odds_below = config_.prune_abs_log_odds_below;
  header.generation = generation_;
  header.stamp_ns = stamp_ns_;
  header.cell_count = snapshot.Size();
  header.body_size = body.size();
  header.checksum = Fnv1a(body);

  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  const auto temp = TempPathFor(path);
  try {
    {
      std::ofstream file(temp, std::ios::binary | std::ios::trunc);
      if (!file) {
        throw std::runtime_error("failed to create block grid temp file: " + temp.string());
      }
      WriteHeader(file, header);
      if (!body.empty()) {
        file.write(reinterpret_cast<const char *>(body.data()),
                   static_cast<std::streamsize>(body.size()));
      }
      file.flush();
      if (!file) {
        throw std::runtime_error("failed to write block grid temp file: " + temp.string());
      }
    }
    AtomicReplace(temp, path);
  } catch (...) {
    std::error_code cleanup_error;
    std::filesystem::remove(temp, cleanup_error);
    throw;
  }
}

PersistentBlockGrid PersistentBlockGrid::LoadBinary(const std::filesystem::path &path,
                                                    const BlockGridConfig &limits) {
  const auto file_size = std::filesystem::file_size(path);
  if (file_size < kHeaderSize) {
    throw std::runtime_error("block grid binary is smaller than its header");
  }
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to open block grid binary: " + path.string());
  }
  const Header header = ReadHeader(file);
  if (header.magic != kMagic) {
    throw std::runtime_error("block grid binary magic mismatch");
  }
  if (header.schema_version != kSchemaVersion) {
    throw std::runtime_error("unsupported block grid binary schema version");
  }
  if (header.endian_marker != kEndianMarker || header.header_size != kHeaderSize) {
    throw std::runtime_error("unsupported block grid binary layout");
  }
  if (header.cell_count > limits.max_persisted_cells) {
    throw std::runtime_error("block grid binary cell count exceeds limit");
  }
  if (header.cell_count > limits.max_runtime_cells) {
    throw std::runtime_error("block grid binary exceeds runtime cell capacity");
  }
  const std::uint64_t expected_body =
      static_cast<std::uint64_t>(header.frame_id_size) +
      header.cell_count * static_cast<std::uint64_t>(sizeof(PersistedCell));
  if (header.body_size != expected_body) {
    throw std::runtime_error("block grid binary body size does not match header fields");
  }
  if (file_size != static_cast<std::uint64_t>(header.header_size) + header.body_size) {
    throw std::runtime_error("block grid binary file size does not match header");
  }
  std::vector<std::uint8_t> body(static_cast<std::size_t>(header.body_size));
  if (!body.empty()) {
    file.read(reinterpret_cast<char *>(body.data()), static_cast<std::streamsize>(body.size()));
  }
  if (!file && !body.empty()) {
    throw std::runtime_error("block grid binary payload is truncated");
  }
  if (Fnv1a(body) != header.checksum) {
    throw std::runtime_error("block grid binary checksum mismatch");
  }

  BlockGridConfig config = limits;
  config.cell_size_m = header.cell_size_m;
  config.block_size = header.block_size;
  config.hit_log_odds = header.hit_log_odds;
  config.miss_log_odds = header.miss_log_odds;
  config.min_log_odds = header.min_log_odds;
  config.max_log_odds = header.max_log_odds;
  config.occupied_threshold = header.occupied_threshold;
  config.prune_abs_log_odds_below = header.prune_abs_log_odds_below;
  ValidateConfig(config);

  PersistentBlockGrid grid(config);
  std::size_t cursor = 0U;
  grid.frame_id_ =
      std::string(reinterpret_cast<const char *>(body.data() + cursor), header.frame_id_size);
  cursor += header.frame_id_size;
  grid.stamp_ns_ = header.stamp_ns;
  grid.generation_ = header.generation;
  for (std::uint64_t i = 0U; i < header.cell_count; ++i) {
    PersistedCell stored;
    ReadPod(body, &cursor, &stored);
    if (!IsFinite(stored.log_odds)) {
      throw std::runtime_error("block grid binary contains non-finite log odds");
    }
    auto *cell = grid.EnsureCell({stored.ix, stored.iy, stored.iz});
    if (cell == nullptr) {
      throw std::runtime_error("block grid binary exceeds runtime block capacity");
    }
    cell->log_odds = std::max(config.min_log_odds, std::min(config.max_log_odds, stored.log_odds));
    cell->hits = stored.hits;
    cell->misses = stored.misses;
  }
  if (cursor != body.size()) {
    throw std::runtime_error("block grid binary has trailing payload bytes");
  }
  grid.last_stats_.total_cells = grid.CellCount();
  return grid;
}

bool PersistentBlockGrid::ValidateBinary(const std::filesystem::path &path, std::string *error,
                                         const BlockGridConfig &limits) {
  try {
    static_cast<void>(LoadBinary(path, limits));
    if (error != nullptr) {
      error->clear();
    }
    return true;
  } catch (const std::exception &exc) {
    if (error != nullptr) {
      *error = exc.what();
    }
    return false;
  }
}

PersistentBlockGrid::CellKey PersistentBlockGrid::ToCellKey(float x_m, float y_m, float z_m,
                                                            float cell_size_m) {
  return {
      static_cast<std::int32_t>(std::floor(x_m / cell_size_m)),
      static_cast<std::int32_t>(std::floor(y_m / cell_size_m)),
      static_cast<std::int32_t>(std::floor(z_m / cell_size_m)),
  };
}

PersistentBlockGrid::BlockKey PersistentBlockGrid::ToBlockKey(const CellKey &cell_key,
                                                              std::int32_t block_size) {
  return {
      FloorDiv(cell_key.x, block_size),
      FloorDiv(cell_key.y, block_size),
      FloorDiv(cell_key.z, block_size),
  };
}

std::size_t PersistentBlockGrid::CellOffset(const CellKey &cell_key, const BlockKey &block_key,
                                            std::int32_t block_size) {
  const std::int32_t lx = cell_key.x - block_key.x * block_size;
  const std::int32_t ly = cell_key.y - block_key.y * block_size;
  const std::int32_t lz = cell_key.z - block_key.z * block_size;
  return static_cast<std::size_t>(lx + ly * block_size + lz * block_size * block_size);
}

float PersistentBlockGrid::ProbabilityFromLogOdds(float log_odds) {
  return 1.0F / (1.0F + std::exp(-log_odds));
}

PersistentBlockGrid::Block &PersistentBlockGrid::EnsureBlock(const BlockKey &key) {
  auto &block = blocks_[key];
  if (block.cells.empty()) {
    const auto count = static_cast<std::size_t>(config_.block_size) *
                       static_cast<std::size_t>(config_.block_size) *
                       static_cast<std::size_t>(config_.block_size);
    block.cells.resize(count);
    block.occupied.assign(count, 0U);
  }
  return block;
}

const PersistentBlockGrid::Cell *PersistentBlockGrid::FindCell(const CellKey &key) const {
  const BlockKey block_key = ToBlockKey(key, config_.block_size);
  const auto block_it = blocks_.find(block_key);
  if (block_it == blocks_.end()) {
    return nullptr;
  }
  const std::size_t offset = CellOffset(key, block_key, config_.block_size);
  if (block_it->second.occupied[offset] == 0U) {
    return nullptr;
  }
  return &block_it->second.cells[offset];
}

PersistentBlockGrid::Cell *PersistentBlockGrid::EnsureCell(const CellKey &key) {
  const BlockKey block_key = ToBlockKey(key, config_.block_size);
  auto block_it = blocks_.find(block_key);
  if (block_it == blocks_.end()) {
    if (blocks_.size() >= config_.max_runtime_blocks ||
        live_cell_count_ >= config_.max_runtime_cells) {
      return nullptr;
    }
    block_it = blocks_.emplace(block_key, Block{}).first;
    const auto count = static_cast<std::size_t>(config_.block_size) *
        static_cast<std::size_t>(config_.block_size) *
        static_cast<std::size_t>(config_.block_size);
    block_it->second.cells.resize(count);
    block_it->second.occupied.assign(count, 0U);
  }
  Block &block = block_it->second;
  const std::size_t offset = CellOffset(key, block_key, config_.block_size);
  if (block.occupied[offset] == 0U) {
    if (live_cell_count_ >= config_.max_runtime_cells) {
      return nullptr;
    }
    block.occupied[offset] = 1U;
    ++block.live_count;
    ++live_cell_count_;
  }
  return &block.cells[offset];
}

bool PersistentBlockGrid::ApplyMiss(const CellKey &key) {
  Cell *cell = EnsureCell(key);
  if (cell == nullptr) {
    return false;
  }
  cell->log_odds = std::max(config_.min_log_odds, cell->log_odds - config_.miss_log_odds);
  cell->misses = SaturatingIncrement(cell->misses);
  return true;
}

bool PersistentBlockGrid::ApplyHit(const CellKey &key) {
  Cell *cell = EnsureCell(key);
  if (cell == nullptr) {
    return false;
  }
  cell->log_odds = std::min(config_.max_log_odds, cell->log_odds + config_.hit_log_odds);
  cell->hits = SaturatingIncrement(cell->hits);
  return true;
}

void PersistentBlockGrid::PruneEmptyBlocks() {
  for (auto it = blocks_.begin(); it != blocks_.end();) {
    if (it->second.live_count == 0U) {
      it = blocks_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace lingtu::maps
