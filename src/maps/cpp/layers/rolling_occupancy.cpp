#include "lingtu/maps/layers/rolling_occupancy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace lingtu::maps::layers {
namespace {

constexpr float kLogOddsScale = 256.0F;
constexpr float kUnknownLogOddsOffset = 0.01F;
constexpr std::size_t kBitsPerWord = 64U;

bool IsFinite(float value) {
  return std::isfinite(static_cast<double>(value));
}

std::int32_t PositiveMod(std::int32_t value, std::int32_t modulus) {
  const std::int32_t result = value % modulus;
  return result < 0 ? result + modulus : result;
}

std::int16_t QuantizeLogOdds(float value) {
  const float scaled = std::round(value * kLogOddsScale);
  return static_cast<std::int16_t>(std::clamp(
      scaled,
      static_cast<float>(std::numeric_limits<std::int16_t>::min()),
      static_cast<float>(std::numeric_limits<std::int16_t>::max())));
}

float ReadCoordinate(const PointCloudView& cloud, std::size_t point, std::size_t axis) {
  switch (cloud.layout) {
    case CloudLayout::kXyzF32Interleaved: {
      const std::size_t index = point * 3U + axis;
      if (cloud.interleaved.data == nullptr || index >= cloud.interleaved.size) {
        throw std::invalid_argument("rolling occupancy XYZ cloud buffer is truncated");
      }
      return cloud.interleaved.data[index];
    }
    case CloudLayout::kXyziF32Interleaved: {
      const std::size_t index = point * 4U + axis;
      if (cloud.interleaved.data == nullptr || index >= cloud.interleaved.size) {
        throw std::invalid_argument("rolling occupancy XYZI cloud buffer is truncated");
      }
      return cloud.interleaved.data[index];
    }
    case CloudLayout::kXyzF32SoA:
    case CloudLayout::kXyziF32SoA: {
      const FloatArrayView* arrays[3] = {&cloud.x, &cloud.y, &cloud.z};
      const FloatArrayView& array = *arrays[axis];
      if (array.data == nullptr || point >= array.size) {
        throw std::invalid_argument("rolling occupancy SoA cloud buffer is truncated");
      }
      return array.data[point];
    }
  }
  throw std::invalid_argument("rolling occupancy cloud layout is unsupported");
}

std::size_t CheckedCellCount(const RollingOccupancyConfig& config) {
  const auto sx = static_cast<std::uint64_t>(config.size_x);
  const auto sy = static_cast<std::uint64_t>(config.size_y);
  const auto sz = static_cast<std::uint64_t>(config.size_z);
  if (sx > std::numeric_limits<std::uint64_t>::max() / sy ||
      sx * sy > std::numeric_limits<std::uint64_t>::max() / sz) {
    throw std::overflow_error("rolling occupancy grid cell count overflow");
  }
  const std::uint64_t count = sx * sy * sz;
  constexpr std::uint64_t kMaxCells = 64ULL * 1024ULL * 1024ULL;
  if (count > kMaxCells || count > std::numeric_limits<std::size_t>::max()) {
    throw std::length_error("rolling occupancy grid exceeds the 64M cell product limit");
  }
  return static_cast<std::size_t>(count);
}

std::size_t BitWordCount(std::size_t cell_count) {
  return (cell_count + kBitsPerWord - 1U) / kBitsPerWord;
}

void SetBit(std::vector<std::uint64_t>& bits, std::size_t index, bool value) {
  const std::size_t word = index / kBitsPerWord;
  const std::uint64_t mask = std::uint64_t{1} << (index % kBitsPerWord);
  if (value) {
    bits[word] |= mask;
  } else {
    bits[word] &= ~mask;
  }
}

bool BitSet(const std::vector<std::uint64_t>& bits, std::size_t index) {
  return (bits[index / kBitsPerWord] &
          (std::uint64_t{1} << (index % kBitsPerWord))) != 0U;
}

std::size_t PackedByteCount(std::size_t cell_count) {
  return (cell_count + 7U) / 8U;
}

template <typename Callback>
void ForEachSetBit(
    const std::vector<std::uint64_t>& bits,
    std::size_t cell_count,
    Callback&& callback) {
  for (std::size_t word_index = 0U; word_index < bits.size(); ++word_index) {
    std::uint64_t word = bits[word_index];
    if (word == 0U) {
      continue;
    }
    const std::size_t base = word_index * kBitsPerWord;
    for (std::size_t bit = 0U; bit < kBitsPerWord && word != 0U; ++bit, word >>= 1U) {
      if ((word & 1U) != 0U && base + bit < cell_count) {
        callback(base + bit);
      }
    }
  }
}

std::size_t CountSetBits(const std::vector<std::uint64_t>& bits) {
  std::size_t count = 0U;
  for (std::uint64_t word : bits) {
    while (word != 0U) {
      word &= word - 1U;
      ++count;
    }
  }
  return count;
}

}  // namespace

void RollingOccupancyCellChunk::Validate() const {
  if (!(resolution_m > 0.0F) || !IsFinite(resolution_m)) {
    throw std::invalid_argument("rolling occupancy chunk resolution must be finite and positive");
  }
  if (frame_id.empty()) {
    throw std::invalid_argument("rolling occupancy chunk frame_id is required");
  }
  const std::size_t count = Size();
  if (center_x_m.size() != count || center_y_m.size() != count ||
      center_z_m.size() != count || log_odds_q8.size() != count ||
      hit_count.size() != count || miss_count.size() != count) {
    throw std::invalid_argument("rolling occupancy chunk SoA fields have inconsistent lengths");
  }
  for (const auto value : state) {
    if (value > static_cast<std::uint8_t>(OccupancyState::kOccupied)) {
      throw std::invalid_argument("rolling occupancy chunk contains an invalid state");
    }
  }
}

std::size_t RollingOccupancySnapshot::Index(
    std::int32_t x,
    std::int32_t y,
    std::int32_t z) const {
  if (x < 0 || x >= size_x || y < 0 || y >= size_y || z < 0 || z >= size_z) {
    throw std::out_of_range("rolling occupancy snapshot coordinate is outside the grid");
  }
  return (static_cast<std::size_t>(z) * static_cast<std::size_t>(size_y) +
          static_cast<std::size_t>(y)) *
      static_cast<std::size_t>(size_x) + static_cast<std::size_t>(x);
}

void RollingOccupancySnapshot::Validate() const {
  if (frame_id.empty() || !(resolution_m > 0.0F) || !IsFinite(resolution_m) ||
      size_x <= 0 || size_y <= 0 || size_z <= 0) {
    throw std::invalid_argument("rolling occupancy snapshot metadata is invalid");
  }
  const std::uint64_t expected = static_cast<std::uint64_t>(size_x) *
      static_cast<std::uint64_t>(size_y) * static_cast<std::uint64_t>(size_z);
  if (expected != state.size() || log_odds_q8.size() != state.size()) {
    throw std::invalid_argument("rolling occupancy snapshot payload size is invalid");
  }
}

std::size_t RollingInflatedSnapshot::CellCount() const noexcept {
  if (size_x <= 0 || size_y <= 0 || size_z <= 0) {
    return 0U;
  }
  return static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y) *
      static_cast<std::size_t>(size_z);
}

bool RollingInflatedSnapshot::Occupied(
    std::int32_t x,
    std::int32_t y,
    std::int32_t z) const {
  if (x < 0 || x >= size_x || y < 0 || y >= size_y || z < 0 || z >= size_z) {
    return true;
  }
  const std::size_t linear =
      (static_cast<std::size_t>(z) * static_cast<std::size_t>(size_y) +
       static_cast<std::size_t>(y)) *
          static_cast<std::size_t>(size_x) +
      static_cast<std::size_t>(x);
  return (occupied_bits[linear / 8U] &
          static_cast<std::uint8_t>(1U << (linear % 8U))) != 0U;
}

void RollingInflatedSnapshot::Validate() const {
  if (frame_id.empty() || !(resolution_m > 0.0F) || !IsFinite(resolution_m) ||
      size_x <= 0 || size_y <= 0 || size_z <= 0 ||
      occupied_bits.size() != PackedByteCount(CellCount()) ||
      occupied_cells > CellCount()) {
    throw std::invalid_argument("rolling inflated snapshot is invalid");
  }
}

void RollingOccupancyGrid::ValidateConfig(const RollingOccupancyConfig& config) {
  if (config.size_x <= 0 || config.size_y <= 0 || config.size_z <= 0) {
    throw std::invalid_argument("rolling occupancy dimensions must be positive");
  }
  static_cast<void>(CheckedCellCount(config));
  if (!(config.resolution_m > 0.0F) || !IsFinite(config.resolution_m)) {
    throw std::invalid_argument("rolling occupancy resolution must be finite and positive");
  }
  if (config.max_ray_range_m < 0.0F || !IsFinite(config.max_ray_range_m)) {
    throw std::invalid_argument("rolling occupancy max ray range must be finite and non-negative");
  }
  if (!(config.hit_log_odds > 0.0F) || !(config.miss_log_odds > 0.0F) ||
      !IsFinite(config.hit_log_odds) || !IsFinite(config.miss_log_odds) ||
      !IsFinite(config.min_log_odds) || !IsFinite(config.max_log_odds) ||
      !(config.min_log_odds < 0.0F) || !(config.max_log_odds > 0.0F)) {
    throw std::invalid_argument("rolling occupancy log-odds configuration is invalid");
  }
  if (!(config.free_probability > 0.0F) || !(config.free_probability < 0.5F) ||
      !(config.occupied_probability > 0.5F) || !(config.occupied_probability < 1.0F) ||
      !(config.free_probability < config.occupied_probability)) {
    throw std::invalid_argument("rolling occupancy probability thresholds are invalid");
  }
  if (!IsFinite(config.inflation_radius_m) || config.inflation_radius_m < 0.0F ||
      !IsFinite(config.inflation_z_up_m) || config.inflation_z_up_m < 0.0F ||
      !IsFinite(config.inflation_z_down_m) || config.inflation_z_down_m < 0.0F) {
    throw std::invalid_argument("rolling occupancy inflation configuration is invalid");
  }
  const auto valid_margin = [](std::int32_t margin, std::int32_t size) {
    return margin >= 0 && margin * 2 < size;
  };
  if (!valid_margin(config.roll_margin_x, config.size_x) ||
      !valid_margin(config.roll_margin_y, config.size_y) ||
      !valid_margin(config.roll_margin_z, config.size_z)) {
    throw std::invalid_argument("rolling occupancy margins must leave a non-empty inner window");
  }
  if (config.decay_after_ns < 0 || !IsFinite(config.decay_factor) ||
      config.decay_factor < 0.0F || config.decay_factor > 1.0F) {
    throw std::invalid_argument("rolling occupancy decay configuration is invalid");
  }
}

RollingOccupancyGrid::RollingOccupancyGrid(RollingOccupancyConfig config)
    : config_(config),
      free_log_odds_threshold_(
          std::log(config.free_probability / (1.0F - config.free_probability))),
      occupied_log_odds_threshold_(
          std::log(config.occupied_probability / (1.0F - config.occupied_probability))) {
  ValidateConfig(config_);
  const std::size_t count = CheckedCellCount(config_);
  cells_.assign(count, Cell{config_.min_log_odds - kUnknownLogOddsOffset});
  ray_total_counts_.assign(count, 0U);
  ray_hit_counts_.assign(count, 0U);
  observed_bits_.assign(BitWordCount(count), 0U);
  occupied_bits_.assign(BitWordCount(count), 0U);
  inflation_counts_.assign(count, 0U);
  inflated_bits_.assign(BitWordCount(count), 0U);
  const int xy_cells = static_cast<int>(
      std::ceil(config_.inflation_radius_m / config_.resolution_m));
  const int z_cells_up = static_cast<int>(
      std::ceil(config_.inflation_z_up_m / config_.resolution_m));
  const int z_cells_down = static_cast<int>(
      std::ceil(config_.inflation_z_down_m / config_.resolution_m));
  const float radius_squared =
      config_.inflation_radius_m * config_.inflation_radius_m;
  for (int z = -z_cells_down; z <= z_cells_up; ++z) {
    for (int y = -xy_cells; y <= xy_cells; ++y) {
      for (int x = -xy_cells; x <= xy_cells; ++x) {
        const float dx = static_cast<float>(x) * config_.resolution_m;
        const float dy = static_cast<float>(y) * config_.resolution_m;
        if ((x == 0 && y == 0) || dx * dx + dy * dy < radius_squared) {
          inflation_offsets_.push_back({x, y, z});
        }
      }
    }
  }
  if (inflation_offsets_.size() > std::numeric_limits<std::uint16_t>::max()) {
    throw std::length_error("rolling occupancy inflation footprint is too large");
  }
  InitializeOrigin(0.0F, 0.0F, 0.0F);
  generation_ = 1U;
}

float RollingOccupancyGrid::Probability(float log_odds) {
  if (log_odds >= 0.0F) {
    const float exp_neg = std::exp(-log_odds);
    return 1.0F / (1.0F + exp_neg);
  }
  const float exp_pos = std::exp(log_odds);
  return exp_pos / (1.0F + exp_pos);
}

std::uint16_t RollingOccupancyGrid::SaturatingIncrement(std::uint16_t value) {
  return value == std::numeric_limits<std::uint16_t>::max()
      ? value
      : static_cast<std::uint16_t>(value + 1U);
}

void RollingOccupancyGrid::InitializeOrigin(
    float center_x_m,
    float center_y_m,
    float center_z_m) {
  const float resolution = config_.resolution_m;
  const auto center_cell = [resolution](float value) {
    return static_cast<std::int64_t>(std::floor(value / resolution));
  };
  origin_x_m_ = static_cast<float>(center_cell(center_x_m) - config_.size_x / 2) * resolution;
  origin_y_m_ = static_cast<float>(center_cell(center_y_m) - config_.size_y / 2) * resolution;
  origin_z_m_ = static_cast<float>(center_cell(center_z_m) - config_.size_z / 2) * resolution;
}

void RollingOccupancyGrid::Reset(
    std::string frame_id,
    float center_x_m,
    float center_y_m,
    float center_z_m,
    std::int64_t stamp_ns) {
  if (frame_id.empty() || !IsFinite(center_x_m) || !IsFinite(center_y_m) ||
      !IsFinite(center_z_m) || stamp_ns < 0) {
    throw std::invalid_argument("rolling occupancy reset arguments are invalid");
  }
  std::unique_lock<std::shared_mutex> lock(mutex_);
  std::fill(cells_.begin(), cells_.end(),
            Cell{config_.min_log_odds - kUnknownLogOddsOffset});
  std::fill(ray_total_counts_.begin(), ray_total_counts_.end(), 0U);
  std::fill(ray_hit_counts_.begin(), ray_hit_counts_.end(), 0U);
  std::fill(observed_bits_.begin(), observed_bits_.end(), 0U);
  std::fill(occupied_bits_.begin(), occupied_bits_.end(), 0U);
  std::fill(inflation_counts_.begin(), inflation_counts_.end(), 0U);
  std::fill(inflated_bits_.begin(), inflated_bits_.end(), 0U);
  collision_dirty_ = false;
  ring_x_ = 0;
  ring_y_ = 0;
  ring_z_ = 0;
  InitializeOrigin(center_x_m, center_y_m, center_z_m);
  frame_id_ = std::move(frame_id);
  stamp_ns_ = stamp_ns;
  ++generation_;
  last_rolled_out_ = {};
  last_rolled_out_.frame_id = frame_id_;
  last_rolled_out_.resolution_m = config_.resolution_m;
  last_rolled_out_.generation = generation_;
  last_stats_ = {};
  last_stats_.generation = generation_;
}

bool RollingOccupancyGrid::InBounds(const CellCoord& coord) const noexcept {
  return coord.x >= 0 && coord.x < config_.size_x && coord.y >= 0 &&
      coord.y < config_.size_y && coord.z >= 0 && coord.z < config_.size_z;
}

bool RollingOccupancyGrid::WorldToCell(
    float x_m,
    float y_m,
    float z_m,
    CellCoord* out) const {
  if (out == nullptr || !IsFinite(x_m) || !IsFinite(y_m) || !IsFinite(z_m)) {
    return false;
  }
  CellCoord coord;
  coord.x = static_cast<std::int32_t>(std::floor((x_m - origin_x_m_) / config_.resolution_m));
  coord.y = static_cast<std::int32_t>(std::floor((y_m - origin_y_m_) / config_.resolution_m));
  coord.z = static_cast<std::int32_t>(std::floor((z_m - origin_z_m_) / config_.resolution_m));
  if (!InBounds(coord)) {
    return false;
  }
  *out = coord;
  return true;
}

std::size_t RollingOccupancyGrid::PhysicalIndex(const CellCoord& logical) const {
  const std::int32_t px = PositiveMod(logical.x + ring_x_, config_.size_x);
  const std::int32_t py = PositiveMod(logical.y + ring_y_, config_.size_y);
  const std::int32_t pz = PositiveMod(logical.z + ring_z_, config_.size_z);
  return (static_cast<std::size_t>(pz) * static_cast<std::size_t>(config_.size_y) +
          static_cast<std::size_t>(py)) *
      static_cast<std::size_t>(config_.size_x) + static_cast<std::size_t>(px);
}

RollingOccupancyGrid::CellCoord RollingOccupancyGrid::PhysicalToLogical(
    std::size_t physical_index) const {
  const std::size_t plane = static_cast<std::size_t>(config_.size_x) *
      static_cast<std::size_t>(config_.size_y);
  const auto pz = static_cast<std::int32_t>(physical_index / plane);
  const std::size_t remainder = physical_index % plane;
  const auto py = static_cast<std::int32_t>(remainder / static_cast<std::size_t>(config_.size_x));
  const auto px = static_cast<std::int32_t>(remainder % static_cast<std::size_t>(config_.size_x));
  return {
      PositiveMod(px - ring_x_, config_.size_x),
      PositiveMod(py - ring_y_, config_.size_y),
      PositiveMod(pz - ring_z_, config_.size_z),
  };
}

OccupancyState RollingOccupancyGrid::StateFor(const Cell& cell) const {
  if (!cell.observed) {
    return OccupancyState::kUnknown;
  }
  if (cell.log_odds > occupied_log_odds_threshold_) {
    return OccupancyState::kOccupied;
  }
  if (cell.log_odds <= free_log_odds_threshold_) {
    return OccupancyState::kFree;
  }
  return OccupancyState::kUnknown;
}

void RollingOccupancyGrid::RefreshMembership(std::size_t physical_index) {
  const Cell& cell = cells_[physical_index];
  const bool was_occupied = BitSet(occupied_bits_, physical_index);
  const bool is_occupied =
      cell.observed && StateFor(cell) == OccupancyState::kOccupied;
  SetBit(observed_bits_, physical_index, cell.observed);
  if (was_occupied != is_occupied) {
    UpdateInflation(PhysicalToLogical(physical_index), is_occupied ? 1 : -1);
    SetBit(occupied_bits_, physical_index, is_occupied);
  }
}

void RollingOccupancyGrid::UpdateInflation(
    const CellCoord& occupied,
    int delta) {
  for (const CellCoord& offset : inflation_offsets_) {
    const CellCoord target{
        occupied.x + offset.x,
        occupied.y + offset.y,
        occupied.z + offset.z,
    };
    if (!InBounds(target)) {
      continue;
    }
    const std::size_t physical = PhysicalIndex(target);
    std::uint16_t& count = inflation_counts_[physical];
    const bool was_inflated = count > 0U;
    if (delta > 0) {
      if (count == std::numeric_limits<std::uint16_t>::max()) {
        throw std::overflow_error("rolling occupancy inflation count overflow");
      }
      ++count;
    } else if (count > 0U) {
      --count;
    }
    const bool is_inflated = count > 0U;
    SetBit(inflated_bits_, physical, is_inflated);
    collision_dirty_ = collision_dirty_ || was_inflated != is_inflated;
  }
}

RollingOccupancyCellChunk RollingOccupancyGrid::ChunkFromPhysicalIndices(
    const std::vector<std::size_t>& indices,
    std::int64_t stamp_ns,
    std::uint64_t generation) const {
  RollingOccupancyCellChunk chunk;
  chunk.frame_id = frame_id_;
  chunk.stamp_ns = stamp_ns;
  chunk.generation = generation;
  chunk.resolution_m = config_.resolution_m;
  chunk.center_x_m.reserve(indices.size());
  chunk.center_y_m.reserve(indices.size());
  chunk.center_z_m.reserve(indices.size());
  chunk.log_odds_q8.reserve(indices.size());
  chunk.hit_count.reserve(indices.size());
  chunk.miss_count.reserve(indices.size());
  chunk.state.reserve(indices.size());
  for (const std::size_t physical : indices) {
    if (physical >= cells_.size()) {
      throw std::out_of_range("rolling occupancy physical index is invalid");
    }
    const Cell& cell = cells_[physical];
    if (!cell.observed) {
      continue;
    }
    const CellCoord logical = PhysicalToLogical(physical);
    chunk.center_x_m.push_back(origin_x_m_ +
        (static_cast<float>(logical.x) + 0.5F) * config_.resolution_m);
    chunk.center_y_m.push_back(origin_y_m_ +
        (static_cast<float>(logical.y) + 0.5F) * config_.resolution_m);
    chunk.center_z_m.push_back(origin_z_m_ +
        (static_cast<float>(logical.z) + 0.5F) * config_.resolution_m);
    chunk.log_odds_q8.push_back(QuantizeLogOdds(cell.log_odds));
    chunk.hit_count.push_back(cell.hits);
    chunk.miss_count.push_back(cell.misses);
    chunk.state.push_back(static_cast<std::uint8_t>(StateFor(cell)));
  }
  chunk.Validate();
  return chunk;
}

RollingOccupancyCellChunk RollingOccupancyGrid::RollByLocked(
    std::int32_t shift_x,
    std::int32_t shift_y,
    std::int32_t shift_z,
    std::int64_t stamp_ns,
    bool commit_revision) {
  if (shift_x == 0 && shift_y == 0 && shift_z == 0) {
    RollingOccupancyCellChunk empty;
    empty.frame_id = frame_id_;
    empty.stamp_ns = stamp_ns;
    empty.generation = generation_;
    empty.resolution_m = config_.resolution_m;
    return empty;
  }

  const bool full_reset = std::abs(shift_x) >= config_.size_x ||
      std::abs(shift_y) >= config_.size_y || std::abs(shift_z) >= config_.size_z;
  std::vector<std::size_t> outgoing;
  if (full_reset) {
    outgoing.reserve(CountSetBits(observed_bits_));
    ForEachSetBit(observed_bits_, cells_.size(), [&](std::size_t physical) {
      outgoing.push_back(physical);
    });
  } else {
    const std::int32_t sizes[3] = {config_.size_x, config_.size_y, config_.size_z};
    const std::int32_t shifts[3] = {shift_x, shift_y, shift_z};
    std::vector<std::uint64_t> clear_bits(BitWordCount(cells_.size()), 0U);
    const auto add_clear = [&](const CellCoord& logical) {
      const std::size_t physical = PhysicalIndex(logical);
      if (!BitSet(clear_bits, physical)) {
        SetBit(clear_bits, physical, true);
        outgoing.push_back(physical);
      }
    };
    for (int dim = 0; dim < 3; ++dim) {
      const std::int32_t shift = shifts[dim];
      for (std::int32_t k = 0; k < std::abs(shift); ++k) {
        const std::int32_t cleared = shift > 0 ? k : sizes[dim] - 1 - k;
        const int dim_a = (dim + 1) % 3;
        const int dim_b = (dim + 2) % 3;
        for (std::int32_t a = 0; a < sizes[dim_a]; ++a) {
          for (std::int32_t b = 0; b < sizes[dim_b]; ++b) {
            std::int32_t coordinate[3] = {0, 0, 0};
            coordinate[dim] = cleared;
            coordinate[dim_a] = a;
            coordinate[dim_b] = b;
            add_clear({coordinate[0], coordinate[1], coordinate[2]});
          }
        }
      }
    }
  }

  RollingOccupancyCellChunk chunk =
      ChunkFromPhysicalIndices(outgoing, stamp_ns, generation_ + 1U);
  if (full_reset) {
    std::fill(cells_.begin(), cells_.end(),
              Cell{config_.min_log_odds - kUnknownLogOddsOffset});
    std::fill(observed_bits_.begin(), observed_bits_.end(), 0U);
    std::fill(occupied_bits_.begin(), occupied_bits_.end(), 0U);
    std::fill(inflation_counts_.begin(), inflation_counts_.end(), 0U);
    std::fill(inflated_bits_.begin(), inflated_bits_.end(), 0U);
  } else {
    const auto leaves_window = [&](const CellCoord& logical) {
      return (shift_x > 0 && logical.x < shift_x) ||
          (shift_x < 0 && logical.x >= config_.size_x + shift_x) ||
          (shift_y > 0 && logical.y < shift_y) ||
          (shift_y < 0 && logical.y >= config_.size_y + shift_y) ||
          (shift_z > 0 && logical.z < shift_z) ||
          (shift_z < 0 && logical.z >= config_.size_z + shift_z);
    };
    for (const std::size_t index : outgoing) {
      if (BitSet(occupied_bits_, index)) {
        const CellCoord occupied = PhysicalToLogical(index);
        for (const CellCoord& offset : inflation_offsets_) {
          const CellCoord target{
              occupied.x + offset.x,
              occupied.y + offset.y,
              occupied.z + offset.z,
          };
          if (!InBounds(target) || leaves_window(target)) {
            continue;
          }
          const std::size_t target_index = PhysicalIndex(target);
          std::uint16_t& count = inflation_counts_[target_index];
          if (count > 0U) {
            --count;
          }
          SetBit(inflated_bits_, target_index, count > 0U);
        }
      }
      cells_[index] = Cell{config_.min_log_odds - kUnknownLogOddsOffset};
      SetBit(observed_bits_, index, false);
      SetBit(occupied_bits_, index, false);
      inflation_counts_[index] = 0U;
      SetBit(inflated_bits_, index, false);
      ray_total_counts_[index] = 0U;
      ray_hit_counts_[index] = 0U;
    }
  }

  origin_x_m_ += static_cast<float>(shift_x) * config_.resolution_m;
  origin_y_m_ += static_cast<float>(shift_y) * config_.resolution_m;
  origin_z_m_ += static_cast<float>(shift_z) * config_.resolution_m;
  ring_x_ = PositiveMod(ring_x_ + shift_x, config_.size_x);
  ring_y_ = PositiveMod(ring_y_ + shift_y, config_.size_y);
  ring_z_ = PositiveMod(ring_z_ + shift_z, config_.size_z);
  stamp_ns_ = std::max(stamp_ns_, stamp_ns);
  if (commit_revision) {
    ++generation_;
    collision_dirty_ = false;
  }
  last_rolled_out_ = chunk;
  return chunk;
}

RollingOccupancyGrid::RollResult RollingOccupancyGrid::RollToCenterLocked(
    float center_x_m,
    float center_y_m,
    float center_z_m,
    std::int64_t stamp_ns,
    bool commit_revision) {
  if (!IsFinite(center_x_m) || !IsFinite(center_y_m) || !IsFinite(center_z_m) ||
      stamp_ns < 0) {
    throw std::invalid_argument("rolling occupancy center is invalid");
  }
  const auto current_cell = [this](float value, float origin) {
    return static_cast<std::int32_t>(std::floor((value - origin) / config_.resolution_m));
  };
  const std::int32_t cx = current_cell(center_x_m, origin_x_m_);
  const std::int32_t cy = current_cell(center_y_m, origin_y_m_);
  const std::int32_t cz = current_cell(center_z_m, origin_z_m_);
  const bool inside_inner = cx >= config_.roll_margin_x &&
      cx < config_.size_x - config_.roll_margin_x &&
      cy >= config_.roll_margin_y && cy < config_.size_y - config_.roll_margin_y &&
      cz >= config_.roll_margin_z && cz < config_.size_z - config_.roll_margin_z;
  if (inside_inner) {
    RollResult result;
    result.chunk.frame_id = frame_id_;
    result.chunk.stamp_ns = stamp_ns;
    result.chunk.generation = generation_;
    result.chunk.resolution_m = config_.resolution_m;
    return result;
  }

  const auto world_cell = [this](float value) {
    return static_cast<std::int64_t>(std::floor(value / config_.resolution_m));
  };
  const auto origin_cell = [this](float value) {
    return static_cast<std::int64_t>(std::llround(value / config_.resolution_m));
  };
  const std::int64_t desired_x = world_cell(center_x_m) - config_.size_x / 2;
  const std::int64_t desired_y = world_cell(center_y_m) - config_.size_y / 2;
  const std::int64_t desired_z = world_cell(center_z_m) - config_.size_z / 2;
  const std::int64_t shift_x_64 = desired_x - origin_cell(origin_x_m_);
  const std::int64_t shift_y_64 = desired_y - origin_cell(origin_y_m_);
  const std::int64_t shift_z_64 = desired_z - origin_cell(origin_z_m_);
  const auto checked_shift = [](std::int64_t value) {
    if (value < std::numeric_limits<std::int32_t>::min() ||
        value > std::numeric_limits<std::int32_t>::max()) {
      throw std::overflow_error("rolling occupancy window shift exceeds int32 range");
    }
    return static_cast<std::int32_t>(value);
  };
  RollResult result;
  result.chunk = RollByLocked(
      checked_shift(shift_x_64),
      checked_shift(shift_y_64),
      checked_shift(shift_z_64),
      stamp_ns,
      commit_revision);
  result.rolled = true;
  return result;
}

RollingOccupancyCellChunk RollingOccupancyGrid::RollToCenter(
    float center_x_m,
    float center_y_m,
    float center_z_m,
    std::int64_t stamp_ns) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  return RollToCenterLocked(center_x_m, center_y_m, center_z_m, stamp_ns, true).chunk;
}

bool RollingOccupancyGrid::ClipRayToWindow(
    float origin_x_m,
    float origin_y_m,
    float origin_z_m,
    float* end_x_m,
    float* end_y_m,
    float* end_z_m) const {
  if (end_x_m == nullptr || end_y_m == nullptr || end_z_m == nullptr) {
    return false;
  }
  CellCoord origin_cell;
  if (!WorldToCell(origin_x_m, origin_y_m, origin_z_m, &origin_cell)) {
    return false;
  }
  const float direction[3] = {
      *end_x_m - origin_x_m,
      *end_y_m - origin_y_m,
      *end_z_m - origin_z_m,
  };
  const float origins[3] = {origin_x_m, origin_y_m, origin_z_m};
  const float minimum[3] = {origin_x_m_, origin_y_m_, origin_z_m_};
  const float maximum[3] = {
      origin_x_m_ + static_cast<float>(config_.size_x) * config_.resolution_m,
      origin_y_m_ + static_cast<float>(config_.size_y) * config_.resolution_m,
      origin_z_m_ + static_cast<float>(config_.size_z) * config_.resolution_m,
  };
  float exit_t = 1.0F;
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    if (direction[axis] > 0.0F) {
      exit_t = std::min(exit_t, (maximum[axis] - origins[axis]) / direction[axis]);
    } else if (direction[axis] < 0.0F) {
      exit_t = std::min(exit_t, (minimum[axis] - origins[axis]) / direction[axis]);
    }
  }
  if (!(exit_t >= 0.0F)) {
    return false;
  }
  const float clipped_t = std::clamp(exit_t, 0.0F, 1.0F);
  *end_x_m = origin_x_m + direction[0] * clipped_t;
  *end_y_m = origin_y_m + direction[1] * clipped_t;
  *end_z_m = origin_z_m + direction[2] * clipped_t;
  const float epsilon = std::max(config_.resolution_m * 1.0e-4F, 1.0e-6F);
  *end_x_m = std::clamp(*end_x_m, minimum[0] + epsilon, maximum[0] - epsilon);
  *end_y_m = std::clamp(*end_y_m, minimum[1] + epsilon, maximum[1] - epsilon);
  *end_z_m = std::clamp(*end_z_m, minimum[2] + epsilon, maximum[2] - epsilon);
  return true;
}

void RollingOccupancyGrid::TraceRay(
    float origin_x_m,
    float origin_y_m,
    float origin_z_m,
    float end_x_m,
    float end_y_m,
    float end_z_m,
    std::vector<CellCoord>* cells) const {
  cells->clear();
  CellCoord sensor;
  CellCoord current;
  if (!WorldToCell(origin_x_m, origin_y_m, origin_z_m, &sensor) ||
      !WorldToCell(end_x_m, end_y_m, end_z_m, &current)) {
    return;
  }
  cells->reserve(static_cast<std::size_t>(
      std::abs(sensor.x - current.x) + std::abs(sensor.y - current.y) +
      std::abs(sensor.z - current.z)));
  if (current == sensor) {
    return;
  }

  const double start_x = (end_x_m - origin_x_m_) / config_.resolution_m;
  const double start_y = (end_y_m - origin_y_m_) / config_.resolution_m;
  const double start_z = (end_z_m - origin_z_m_) / config_.resolution_m;
  const double dx = static_cast<double>(sensor.x - current.x);
  const double dy = static_cast<double>(sensor.y - current.y);
  const double dz = static_cast<double>(sensor.z - current.z);
  const auto sign = [](double value) { return value == 0.0 ? 0 : value < 0.0 ? -1 : 1; };
  const auto integer_boundary = [](double value, double delta) {
    if (delta < 0.0) {
      value = -value;
      delta = -delta;
    }
    value = std::fmod(std::fmod(value, 1.0) + 1.0, 1.0);
    return (1.0 - value) / delta;
  };
  const int step_x = sign(dx);
  const int step_y = sign(dy);
  const int step_z = sign(dz);
  double t_max_x = integer_boundary(start_x, dx);
  double t_max_y = integer_boundary(start_y, dy);
  double t_max_z = integer_boundary(start_z, dz);
  const double t_delta_x = static_cast<double>(step_x) / dx;
  const double t_delta_y = static_cast<double>(step_y) / dy;
  const double t_delta_z = static_cast<double>(step_z) / dz;
  const std::size_t max_steps = cells_.size();
  while (!(current == sensor) && cells->size() <= max_steps) {
    cells->push_back(current);
    if (t_max_x < t_max_y && t_max_x < t_max_z) {
      current.x += step_x;
      t_max_x += t_delta_x;
    } else if (t_max_y < t_max_z) {
      current.y += step_y;
      t_max_y += t_delta_y;
    } else {
      current.z += step_z;
      t_max_z += t_delta_z;
    }
    if (!InBounds(current)) {
      break;
    }
  }
}

std::size_t RollingOccupancyGrid::DecayLocked(std::int64_t now_ns) {
  if (config_.decay_after_ns == 0 || now_ns <= 0) {
    return 0U;
  }
  std::size_t changed = 0U;
  ForEachSetBit(observed_bits_, cells_.size(), [&](std::size_t physical) {
    Cell& cell = cells_[physical];
    if (!cell.observed || cell.last_observed_ns <= 0 ||
        now_ns - cell.last_observed_ns < config_.decay_after_ns) {
      return;
    }
    const float previous = cell.log_odds;
    cell.log_odds *= config_.decay_factor;
    if (std::fabs(cell.log_odds) < 1.0F / kLogOddsScale) {
      cell = Cell{config_.min_log_odds - kUnknownLogOddsOffset};
    } else {
      cell.last_observed_ns = now_ns;
    }
    RefreshMembership(physical);
    if (cell.log_odds != previous) {
      ++changed;
    }
  });
  return changed;
}

RollingOccupancyUpdateStats RollingOccupancyGrid::Update(const MapCloudFrame& frame) {
  const PointCloudView& cloud = frame.cloud;
  const std::string incoming_frame = cloud.frame_id.empty() ? "map" : cloud.frame_id;
  if (cloud.stamp_ns < 0 || frame.decay_stamp_ns < 0 ||
      !IsFinite(frame.sensor_origin_x_m) ||
      !IsFinite(frame.sensor_origin_y_m) || !IsFinite(frame.sensor_origin_z_m)) {
    throw std::invalid_argument("rolling occupancy observation metadata is invalid");
  }
  const std::int64_t decay_stamp_ns =
      frame.decay_stamp_ns > 0 ? frame.decay_stamp_ns : cloud.stamp_ns;
  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (incoming_frame != frame_id_) {
    throw std::invalid_argument(
        "rolling occupancy frame mismatch: expected '" + frame_id_ + "', received '" +
        incoming_frame + "'");
  }
  if (config_.reject_out_of_order && cloud.stamp_ns > 0 && stamp_ns_ > 0 &&
      cloud.stamp_ns < stamp_ns_) {
    throw std::invalid_argument("rolling occupancy rejected an out-of-order observation");
  }

  RollingOccupancyUpdateStats stats;
  stats.input_points = cloud.point_count;
  collision_dirty_ = false;
  if (config_.auto_roll) {
    RollResult roll = RollToCenterLocked(
        frame.sensor_origin_x_m,
        frame.sensor_origin_y_m,
        frame.sensor_origin_z_m,
        cloud.stamp_ns,
        false);
    stats.rolled = roll.rolled;
    stats.rolled_out_cells = roll.chunk.Size();
  }
  stats.decayed_cells = DecayLocked(decay_stamp_ns);

  std::vector<std::size_t> touched_indices;
  touched_indices.reserve(cloud.point_count * 8U);
  std::vector<std::uint64_t> ray_endpoint_bits(BitWordCount(cells_.size()), 0U);
  std::vector<std::uint64_t> ray_traverse_bits(BitWordCount(cells_.size()), 0U);
  const auto record_ray_evidence = [&](std::size_t physical, bool hit) {
    if (ray_total_counts_[physical] == 0U) {
      touched_indices.push_back(physical);
    }
    ray_total_counts_[physical] = SaturatingIncrement(ray_total_counts_[physical]);
    if (hit) {
      ray_hit_counts_[physical] = SaturatingIncrement(ray_hit_counts_[physical]);
    }
  };
  std::vector<CellCoord> ray;
  for (std::size_t point = 0U; point < cloud.point_count; ++point) {
    float hit_x = ReadCoordinate(cloud, point, 0U);
    float hit_y = ReadCoordinate(cloud, point, 1U);
    float hit_z = ReadCoordinate(cloud, point, 2U);
    if (!IsFinite(hit_x) || !IsFinite(hit_y) || !IsFinite(hit_z)) {
      ++stats.rejected_points;
      continue;
    }
    float dx = hit_x - frame.sensor_origin_x_m;
    float dy = hit_y - frame.sensor_origin_y_m;
    float dz = hit_z - frame.sensor_origin_z_m;
    const float length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(length > 0.0F)) {
      ++stats.rejected_points;
      continue;
    }
    bool has_hit = true;
    if (config_.max_ray_range_m > 0.0F && length > config_.max_ray_range_m) {
      const float scale = config_.max_ray_range_m / length;
      hit_x = frame.sensor_origin_x_m + dx * scale;
      hit_y = frame.sensor_origin_y_m + dy * scale;
      hit_z = frame.sensor_origin_z_m + dz * scale;
      has_hit = false;
    }
    CellCoord hit_coord;
    if (!WorldToCell(hit_x, hit_y, hit_z, &hit_coord)) {
      has_hit = false;
    }
    if (!ClipRayToWindow(
            frame.sensor_origin_x_m,
            frame.sensor_origin_y_m,
            frame.sensor_origin_z_m,
            &hit_x,
            &hit_y,
            &hit_z)) {
      ++stats.rejected_points;
      continue;
    }
    CellCoord endpoint_coord;
    if (!WorldToCell(hit_x, hit_y, hit_z, &endpoint_coord)) {
      ++stats.rejected_points;
      continue;
    }
    TraceRay(
        frame.sensor_origin_x_m,
        frame.sensor_origin_y_m,
        frame.sensor_origin_z_m,
        hit_x,
        hit_y,
        hit_z,
        &ray);
    ++stats.accepted_points;
    const bool reached_hit = has_hit && endpoint_coord == hit_coord;
    const std::size_t endpoint = PhysicalIndex(endpoint_coord);
    record_ray_evidence(endpoint, reached_hit);
    if (BitSet(ray_endpoint_bits, endpoint)) {
      continue;
    }
    SetBit(ray_endpoint_bits, endpoint, true);
    ++stats.unique_rays;
    for (const CellCoord& traversed : ray) {
      const std::size_t physical = PhysicalIndex(traversed);
      record_ray_evidence(physical, false);
      if (BitSet(ray_traverse_bits, physical)) {
        break;
      }
      SetBit(ray_traverse_bits, physical, true);
    }
  }

  for (const std::size_t physical : touched_indices) {
    const std::uint16_t hits = ray_hit_counts_[physical];
    const std::uint16_t total = ray_total_counts_[physical];
    const bool hit = hits >= static_cast<std::uint16_t>(total - hits);
    Cell& cell = cells_[physical];
    cell.observed = true;
    if (hit) {
      cell.log_odds = std::min(config_.max_log_odds, cell.log_odds + config_.hit_log_odds);
      cell.hits = SaturatingIncrement(cell.hits);
      ++stats.hit_updates;
    } else {
      cell.log_odds = std::max(config_.min_log_odds, cell.log_odds - config_.miss_log_odds);
      cell.misses = SaturatingIncrement(cell.misses);
      ++stats.free_updates;
    }
    cell.last_observed_ns = decay_stamp_ns;
    RefreshMembership(physical);
    ray_total_counts_[physical] = 0U;
    ray_hit_counts_[physical] = 0U;
  }
  if (stats.rolled || collision_dirty_) {
    ++generation_;
  }
  collision_dirty_ = false;
  stamp_ns_ = std::max(stamp_ns_, cloud.stamp_ns);
  stats.generation = generation_;
  last_stats_ = stats;
  return stats;
}

std::size_t RollingOccupancyGrid::Decay(std::int64_t now_ns) {
  if (now_ns < 0) {
    throw std::invalid_argument("rolling occupancy decay timestamp is invalid");
  }
  std::unique_lock<std::shared_mutex> lock(mutex_);
  collision_dirty_ = false;
  const std::size_t changed = DecayLocked(now_ns);
  if (collision_dirty_) {
    ++generation_;
  }
  collision_dirty_ = false;
  last_stats_ = {};
  last_stats_.decayed_cells = changed;
  last_stats_.generation = generation_;
  return changed;
}

OccupancyState RollingOccupancyGrid::StateAt(float x_m, float y_m, float z_m) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  CellCoord coord;
  if (!WorldToCell(x_m, y_m, z_m, &coord)) {
    return OccupancyState::kUnknown;
  }
  return StateFor(cells_[PhysicalIndex(coord)]);
}

float RollingOccupancyGrid::OccupancyProbability(float x_m, float y_m, float z_m) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  CellCoord coord;
  if (!WorldToCell(x_m, y_m, z_m, &coord)) {
    return 0.5F;
  }
  const Cell& cell = cells_[PhysicalIndex(coord)];
  return cell.observed ? Probability(cell.log_odds) : 0.5F;
}

bool RollingOccupancyGrid::Contains(float x_m, float y_m, float z_m) const {
  return StateAt(x_m, y_m, z_m) == OccupancyState::kOccupied;
}

bool RollingOccupancyGrid::InflatedContains(float x_m, float y_m, float z_m) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  CellCoord coord;
  if (!WorldToCell(x_m, y_m, z_m, &coord)) {
    return true;
  }
  return BitSet(inflated_bits_, PhysicalIndex(coord));
}

RollingOccupancySnapshot RollingOccupancyGrid::Snapshot() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  RollingOccupancySnapshot snapshot;
  snapshot.frame_id = frame_id_;
  snapshot.stamp_ns = stamp_ns_;
  snapshot.generation = generation_;
  snapshot.resolution_m = config_.resolution_m;
  snapshot.size_x = config_.size_x;
  snapshot.size_y = config_.size_y;
  snapshot.size_z = config_.size_z;
  snapshot.origin_x_m = origin_x_m_;
  snapshot.origin_y_m = origin_y_m_;
  snapshot.origin_z_m = origin_z_m_;
  snapshot.state.assign(cells_.size(), static_cast<std::uint8_t>(OccupancyState::kUnknown));
  snapshot.log_odds_q8.assign(cells_.size(), 0);
  ForEachSetBit(observed_bits_, cells_.size(), [&](std::size_t physical) {
    const CellCoord logical_coord = PhysicalToLogical(physical);
    const std::size_t logical =
        (static_cast<std::size_t>(logical_coord.z) * static_cast<std::size_t>(config_.size_y) +
         static_cast<std::size_t>(logical_coord.y)) *
            static_cast<std::size_t>(config_.size_x) +
        static_cast<std::size_t>(logical_coord.x);
    const Cell& cell = cells_[physical];
    snapshot.state[logical] = static_cast<std::uint8_t>(StateFor(cell));
    snapshot.log_odds_q8[logical] = QuantizeLogOdds(cell.log_odds);
  });
  snapshot.Validate();
  return snapshot;
}

RollingInflatedSnapshot RollingOccupancyGrid::InflatedSnapshot() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  RollingInflatedSnapshot snapshot;
  snapshot.frame_id = frame_id_;
  snapshot.stamp_ns = stamp_ns_;
  snapshot.generation = generation_;
  snapshot.resolution_m = config_.resolution_m;
  snapshot.size_x = config_.size_x;
  snapshot.size_y = config_.size_y;
  snapshot.size_z = config_.size_z;
  snapshot.origin_x_m = origin_x_m_;
  snapshot.origin_y_m = origin_y_m_;
  snapshot.origin_z_m = origin_z_m_;
  snapshot.occupied_cells = CountSetBits(inflated_bits_);
  snapshot.occupied_bits.assign(PackedByteCount(cells_.size()), 0U);
  ForEachSetBit(inflated_bits_, cells_.size(), [&](std::size_t physical) {
    const CellCoord logical = PhysicalToLogical(physical);
    const std::size_t linear =
        (static_cast<std::size_t>(logical.z) * static_cast<std::size_t>(config_.size_y) +
         static_cast<std::size_t>(logical.y)) *
            static_cast<std::size_t>(config_.size_x) +
        static_cast<std::size_t>(logical.x);
    snapshot.occupied_bits[linear / 8U] |=
        static_cast<std::uint8_t>(1U << (linear % 8U));
  });
  snapshot.Validate();
  return snapshot;
}

RollingOccupancyCellChunk RollingOccupancyGrid::ObservedCellsLocked() const {
  std::vector<std::size_t> indices;
  indices.reserve(CountSetBits(observed_bits_));
  ForEachSetBit(observed_bits_, cells_.size(), [&](std::size_t physical) {
    indices.push_back(physical);
  });
  return ChunkFromPhysicalIndices(indices, stamp_ns_, generation_);
}

RollingOccupancyCellChunk RollingOccupancyGrid::ObservedCells() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return ObservedCellsLocked();
}

RollingOccupancyCellChunk RollingOccupancyGrid::LastRolledOut() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return last_rolled_out_;
}

RollingOccupancyUpdateStats RollingOccupancyGrid::LastStats() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return last_stats_;
}

RollingOccupancyConfig RollingOccupancyGrid::Config() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return config_;
}

std::uint64_t RollingOccupancyGrid::Generation() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return generation_;
}

}  // namespace lingtu::maps::layers
